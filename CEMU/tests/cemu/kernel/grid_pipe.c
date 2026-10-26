/*
 * grid_pipe.c —— grid-cdc 的**流水线版**设备侧算子。
 *
 * 与 kernel/grid_ds.c 的关系：算法完全一致（同一套 CDC 切点、同一套网格折叠
 * weak、同一把 SHA1），只是**执行模型**改成流水线。刻意不共用文件：
 * grid_ds 是"一个窗口一个 chunk"，grid_pipe 是"一个窗口 P 个 chunk 并发"，
 * 两者的 MR 布局和偏移口径都不同，合成一个文件只会互相牵连。
 *
 * ── 流水线在哪 ────────────────────────────────────────────────
 * FEMU 的 indirect 流水线（hw/femu/csd/compute.c: indirect_main）本身就是条
 * 三级流水：stage0 读闪存 → stage1 跑 CSF → stage2 写回。关键在于
 * `nr_concurrent_chunks`：P 个 chunk 各有自己的 MR 切片，主循环一开始就把
 * **P 个读请求全部塞进 FTL 队列**，IO 在 FTL 线程里异步完成、通过 ring 回头，
 * 而 CSF 在主线程里跑。于是：
 *
 *     时刻 →   t1    t2    t3    t4    t5    t6    t7
 *     读：    |读0 |读1 |读2 |读3 |
 *     切算：        |切算0|切算1|切算2|切算3|
 *     写：                |写0 |写1 |写2 |
 *
 * 也就是 chunk j 的闪存读和 chunk i 的 CPU 计算重叠 —— 闪存通道和 CPU 是
 * 两种资源，本来就不该串行。此前所有实现都传 nr_concurrent_chunks=1，
 * 等于把这条硬件流水线关掉了：读完全部数据、才开始算。
 *
 * ── 切 与 算 能不能重叠 ───────────────────────────────────────
 * 不能真的并行：两者都用 CPU，单核上只有一份算力。能做的是**融成一遍扫描** ——
 * 扫字节时同时喂 gear（决定切点）并在需要时喂 SHA1，数据只过一遍，
 * 而不是"先扫一遍找边界、再回头扫一遍算哈希"。
 * 注意 SHA1 仍然**只对弱哈希命中的块算**（步骤 5 的语义不能破）：
 * 弱哈希来自网格、不碰数据，命中与否要等切出边界才知道，所以命中块的那次
 * SHA1 还是单独过一遍该块（块只有 ~12KB，还在 cache 里，代价很小）。
 *
 * ── 三种角色（与 grid_ds 一致，靠 extra 载荷区分） ──────────────
 *   mode 1  无载荷            client：CDC 切分 + 网格组合出 weak（不算强哈希）
 *   mode 2  载荷 = gd_tab_hdr server：CDC 切分 + 网格 weak + 与清单比对
 *                             + 只对弱哈希命中的块算 SHA1
 *   mode 3  载荷 = gd_query_hdr client：只给指定区间算强哈希（需要 mr[2] 描述符）
 *
 * MR 布局（每个 chunk 一份，共 3×P 个 MR，numr 必须能被 P 整除）：
 *   mr[0] = 本 chunk 的输出表
 *   mr[1] = 本 chunk 的数据托盘（R 阶段按 cf 读入）
 *   mr[2] = mode 1/2：本 chunk 的 512B CRC 网格（从本 chunk 首页开始）
 *           mode 3  ：per-chunk 描述符 gd_desc（告诉 CSF 自己的基址）
 *
 * 入参：cparam1 = 本 chunk 字节数 n；cparam2 不用（P 个 chunk 共用，无意义）
 * 出参：mode 1/2 的条目偏移是 **chunk 内相对偏移**，由主机按 chunk 基址回填。
 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "cemu_def.h"
#include "dsync_abi.h"
#include "chunk_abi.h"
#include "chunk_gear_v2.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"
#include "../../../hw/femu/backend/csd_sha1.h"

#define CDC_ALIGN      512ULL
#define CDC_MAX_CHUNK  65536ULL

#define ENTH_BYTES   ((uint32_t)sizeof(struct gd_ent))
#define PAIRH_BYTES  ((uint32_t)sizeof(struct gd_pair))
#define QRY_BYTES    ((uint32_t)sizeof(struct gd_query))

/* ------------------------------------------------------------------ */
/* 网格折叠 weak：把 [start, start+len) 覆盖到的 512B 扇区 CRC 组合起来    */
/* 与 grid_ds.c 同一份实现，逐字未改                                     */
/* ------------------------------------------------------------------ */
static uint32_t grid_range_crc(const uint8_t *grid, uint64_t grid_len,
                               uint64_t start, uint64_t len, int *ok)
{
    const uint64_t end = start + len;
    uint64_t cur = start;
    uint32_t crc = 0;
    int first = 1;

    while (cur < end) {
        const uint64_t page = cur / 4096ull;
        const uint64_t sec_in_page = (cur % 4096ull) / 512ull;
        const uint64_t page_end = (page + 1) * 4096ull;
        const uint64_t take = (end < page_end ? end : page_end) - cur;

        if ((page + 1) * CSD_CRC_SLOT_LEN > grid_len) {
            if (ok) {
                *ok = 0;
            }
            return 0;
        }

        const uint8_t *slot = grid + page * CSD_CRC_SLOT_LEN;
        uint32_t r = csd_crc_slot_get(slot, (uint32_t)sec_in_page);

        if (take == 4096ull) {
            r = csd_crc_slot_get(slot, 0);
            for (uint32_t s = 1; s < 8; s++) {
                r = csd_crc32c_combine512(r, csd_crc_slot_get(slot, s));
            }
        } else if (take > 512ull) {
            const uint64_t ns = take / 512ull;
            r = csd_crc_slot_get(slot, (uint32_t)sec_in_page);
            for (uint64_t s = 1; s < ns; s++) {
                r = csd_crc32c_combine512(r,
                    csd_crc_slot_get(slot, (uint32_t)(sec_in_page + s)));
            }
        }

        if (first) {
            crc = r;
            first = 0;
        } else {
            crc = csd_crc32c_combine(crc, r, take);
        }
        cur += take;
    }

    if (ok) {
        *ok = 1;
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/* 弱哈希桶：开放寻址 + 线性探测，装载因子 ≤ 0.5                        */
/* ------------------------------------------------------------------ */
#define GD_BUCKET_MAX  (1u << 17)
#define GD_EMPTY       UINT32_MAX

struct gd_slot {
    uint32_t weak;
    uint32_t len;         /* 长度也参与初筛：长度不同必然不是同一块 */
    uint32_t off_lo;
    uint32_t next;
};

static struct gd_slot gd_bucket[GD_BUCKET_MAX];

static inline uint32_t gd_hash(uint32_t w, uint32_t mask)
{
    return (w * 2654435761u) & mask;
}

static uint32_t gd_bucket_nr(uint64_t nr)
{
    uint32_t nb = 2u;
    while ((uint64_t)nb < 2u * nr + 2u && nb < GD_BUCKET_MAX) {
        nb <<= 1;
    }
    return nb;
}

/* ------------------------------------------------------------------ */
/* CDC：与 grid_ds.c 逐字节一致 —— fp 逐字节累积并逐字节检查，          */
/* 命中后把切点向上对齐到 512B。两侧必须用同一份，否则边界对不上。        */
/* ------------------------------------------------------------------ */
static uint64_t cdc_cut(const uint8_t *data, uint64_t pos, uint64_t limit)
{
    uint64_t fp = 0;

    for (uint64_t i = pos; i < limit; i++) {
        fp = (fp << 1) + chunk_gear_table[data[i]];
        if ((i + 1 - pos) < CHUNK_MIN_SIZE) {
            continue;
        }
        if ((fp & CHUNK_GEAR_MASK) == 0) {
            const uint64_t end = ((i + 1) + CDC_ALIGN - 1) & ~(CDC_ALIGN - 1);

            return (end > limit) ? limit : end;
        }
    }
    return limit;
}

/* ------------------------------------------------------------------ */

long long grid_pipe(struct cemu_args *args)
{
    const uint64_t n = args->cparam1;

    uint8_t *out = args->mr_addr[0];
    const uint64_t out_len = args->mr_len[0];
    const uint8_t *data = args->mr_addr[1];
    const uint64_t in_len = args->mr_len[1];
    const uint8_t *aux = args->mr_addr[2];
    const uint64_t aux_len = args->mr_len[2];
    const uint8_t *dbuf = args->data_buffer;
    const uint64_t dbuf_len = args->buffer_len;

    if (!out || !data) {
        return -1;
    }

    struct gd_tab_hdr *h = (struct gd_tab_hdr *)(void *)out;
    uint8_t *ents = out + sizeof(*h);

    const int is_query = (dbuf && dbuf_len >= sizeof(struct gd_query_hdr) &&
                          ((const struct gd_query_hdr *)(const void *)dbuf)->magic
                              == GD_QRY_MAGIC);
    const int is_srv = (!is_query && dbuf && dbuf_len >= sizeof(struct gd_tab_hdr) &&
                        ((const struct gd_tab_hdr *)(const void *)dbuf)->magic
                            == GD_TAB_MAGIC);

    /* ---------------- mode 3：client 按区间补算强哈希 ---------------- */
    if (is_query) {
        /* 查询表给的是绝对偏移，必须从 per-chunk 描述符里拿到自己的基址 */
        if (!aux || aux_len < (uint64_t)sizeof(struct gd_desc)) {
            return -2;
        }
        const struct gd_desc *dsc = (const struct gd_desc *)(const void *)aux;
        if (dsc->magic != GD_DESC_MAGIC) {
            return -2;
        }
        const uint64_t base = dsc->base;

        const struct gd_query_hdr *qh =
            (const struct gd_query_hdr *)(const void *)dbuf;
        const uint64_t nr_q = qh->nr;
        const uint8_t *qe = dbuf + sizeof(*qh);

        if (nr_q * QRY_BYTES + sizeof(*qh) > dbuf_len) {
            return -4;
        }
        if (sizeof(*h) + nr_q * ENTH_BYTES > out_len) {
            return -5;
        }

        h->magic = GD_TAB_MAGIC;
        h->nr = (uint32_t)nr_q;
        h->entry_bytes = ENTH_BYTES;
        h->total_len = n;
        h->build_id = GD_BUILD_ID;
        h->reserved = 0;

        if (n == 0 || in_len < n) {
            return (long long)nr_q;
        }

        const uint64_t win_end = base + n;

        for (uint64_t i = 0; i < nr_q; i++) {
            const struct gd_query *q =
                (const struct gd_query *)(const void *)(qe + i * QRY_BYTES);
            struct gd_ent *e;

            if (q->idx >= nr_q || q->len == 0) {
                continue;
            }
            /* 只处理整个区间都落在本 chunk 内的查询 */
            if (q->off < base || q->off + q->len > win_end) {
                continue;
            }
            if (q->off + q->len - base > in_len) {
                continue;
            }

            e = (struct gd_ent *)(void *)(ents + q->idx * ENTH_BYTES);
            e->off = q->off;
            e->len = (uint32_t)q->len;
            e->weak = 0;
            csd_sha1(data + (q->off - base), q->len, e->strong);
            memset(e->strong + CSD_SHA1_LEN, 0, DSYNC_FP_LEN - CSD_SHA1_LEN);
        }
        return (long long)nr_q;
    }

    /* ---------------- mode 1 / mode 2：CDC ---------------- */
    const uint32_t entry_bytes = is_srv ? PAIRH_BYTES : ENTH_BYTES;
    const uint32_t ent_cap = (uint32_t)((out_len - sizeof(*h)) / entry_bytes);

    if (!aux) {
        return -1;
    }
    if (n == 0 || in_len < n) {
        h->magic = GD_TAB_MAGIC;
        h->nr = 0;
        h->entry_bytes = entry_bytes;
        h->total_len = 0;
        h->build_id = GD_BUILD_ID;
        h->reserved = 0;
        return 0;
    }

    /* mode 2：先把 client 清单的 weak 装进桶 */
    uint32_t nb = 0, mask = 0;
    if (is_srv) {
        const struct gd_tab_hdr *nh = (const struct gd_tab_hdr *)(const void *)dbuf;
        const uint64_t nr_new = nh->nr;
        const uint8_t *ne = dbuf + sizeof(*nh);

        if (nh->entry_bytes != ENTH_BYTES ||
            nr_new * ENTH_BYTES + sizeof(*nh) > dbuf_len) {
            return -3;
        }
        nb = gd_bucket_nr(nr_new);
        if (nb > GD_BUCKET_MAX) {
            return -6;
        }
        mask = nb - 1u;

        for (uint32_t i = 0; i < nb; i++) {
            gd_bucket[i].weak = 0;
            gd_bucket[i].len = 0;
            gd_bucket[i].next = GD_EMPTY;
        }
        for (uint32_t i = 0; i < nr_new; i++) {
            const struct gd_ent *e =
                (const struct gd_ent *)(const void *)(ne + i * ENTH_BYTES);
            uint32_t b = gd_hash(e->weak, mask);

            while (gd_bucket[b].next != GD_EMPTY) {
                b = (b + 1u) & mask;
            }
            gd_bucket[b].weak = e->weak;
            gd_bucket[b].len = e->len;
            gd_bucket[b].off_lo = (uint32_t)e->off;
            gd_bucket[b].next = i;
        }
    }

    uint64_t pos = 0;
    uint64_t nr = 0;

    while (pos < n) {
        uint64_t limit = pos + CDC_MAX_CHUNK;
        uint64_t cut, len;
        int ok = 1;
        uint32_t weak;

        if (limit > n) {
            limit = n;
        }
        cut = cdc_cut(data, pos, limit);
        if (cut <= pos) {
            break;
        }
        len = cut - pos;
        if (nr >= ent_cap) {
            break;
        }

        /* 网格从本 chunk 的首页开始，所以这里传的是 chunk 内相对偏移 */
        weak = grid_range_crc(aux, aux_len, pos, len, &ok);
        if (!ok) {
            break;
        }

        if (is_srv) {
            /* 弱哈希初筛：命中才算强哈希 */
            uint32_t b = gd_hash(weak, mask);
            uint32_t probe = 0;
            int hit = 0;
            uint32_t new_off = 0;

            while (gd_bucket[b].next != GD_EMPTY && probe < nb) {
                if (gd_bucket[b].weak == weak && gd_bucket[b].len == (uint32_t)len) {
                    new_off = gd_bucket[b].off_lo;
                    hit = 1;
                    break;
                }
                b = (b + 1u) & mask;
                probe++;
            }

            if (hit) {
                struct gd_pair *p =
                    (struct gd_pair *)(void *)(ents + nr * entry_bytes);

                p->old_off = pos;          /* chunk 内相对，主机回填基址 */
                p->new_off = new_off;      /* 来自清单，本来就是绝对偏移 */
                p->len = (uint32_t)len;
                p->weak = weak;
                csd_sha1(data + pos, len, p->strong);
                memset(p->strong + CSD_SHA1_LEN, 0, DSYNC_FP_LEN - CSD_SHA1_LEN);
                nr++;
            }
        } else {
            struct gd_ent *e = (struct gd_ent *)(void *)(ents + nr * entry_bytes);

            e->off = pos;                  /* chunk 内相对，主机回填基址 */
            e->len = (uint32_t)len;
            e->weak = weak;
            memset(e->strong, 0, DSYNC_FP_LEN);   /* 步骤 5 之前不填强哈希 */
            nr++;
        }

        pos = cut;
    }

    h->magic = GD_TAB_MAGIC;
    h->nr = (uint32_t)nr;
    h->entry_bytes = entry_bytes;
    h->total_len = n;
    h->build_id = GD_BUILD_ID;
    h->reserved = 0;

    return (long long)nr;
}
