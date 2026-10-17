/*
 * grid_ds.c —— grid-cdc 的**单一**设备侧算子，承载 6 步流程里的三种角色。
 *
 * 之所以合成一个文件：两侧的 CDC 切点必须逐字节一致，放在同一个文件里
 * 共用同一份切分循环，改一处就两边同步，不会再出现"两侧边界对不上"。
 *
 * 模式由随命令下发的 extra 载荷区分（不占用已经很挤的 cparam）：
 *   mode 1  无载荷                      client：CDC 切分 + 网格组合出 weak
 *                                       **不算强哈希**
 *   mode 2  载荷 = gd_tab_hdr（清单）    server：CDC 切分 + 网格组合出 weak
 *                                       + 与清单里的 weak 比对
 *                                       + 只对弱哈希命中的块算 SHA1
 *   mode 3  载荷 = gd_query_hdr（区间表）client：只给指定区间算强哈希
 *
 * MR 布局（三种模式统一）：
 *   mr[0] = 输出表（R 阶段不会碰它）
 *   mr[1] = 输入数据（R 阶段按 cf 读入）
 *   mr[2] = 该文件的 512B CRC 网格（collect_oob 浅读出来的常驻槽；mode 3 不用）
 *
 * 入参：cparam1 = 本窗口字节数 n，cparam2 = 本窗口在文件内的起始偏移 base
 * 返回：输出条目数
 *
 * 两个踩过的坑，写在最前面免得踩回去：
 *   1. 切点条件：fp 必须**逐字节累积并逐字节检查**，命中后再把切点向上对齐
 *      到 512B。只在 512B 对齐处检查（或每 512B 只喂一个字节）都会把检查点
 *      密度降 512 倍，期望切点间隔变成 4MB，远超 CDC_MAX_CHUNK(64KB)，
 *      于是每块都被 max 强行截断 —— 边界退化成"从窗口头每 64KB 一刀"，
 *      不再是内容定义的。一旦中间有插入，之后永远差一个偏移、再也对齐不上，
 *      实测 10MB 插 4KB 的命中率直接从 ~100% 掉到 50%。
 *   2. indirect 流水线会覆盖 CSF 返回值，条数只能从输出表头 nr 读，
 *      所以输出区每窗口必须先清零。
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
/* 弱哈希桶：开放寻址 + 线性探测，装载因子 ≤ 0.5，桶数按清单条数动态取     */
/* ------------------------------------------------------------------ */
#define GD_BUCKET_MAX  (1u << 17)
#define GD_EMPTY       UINT32_MAX

struct gd_slot {
    uint32_t weak;
    uint32_t len;         /* 长度也参与初筛：长度不同必然不是同一块 */
    uint32_t off_lo;      /* 清单里该块在 f' 内的偏移（低 32 位够用） */
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
/* CDC：两侧共用同一份切分循环（这是合成单文件的最大收益）                */
/* 命中返回切点绝对位置（相对 base=0 的窗口内偏移），未命中返回 limit     */
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

long long grid_ds(struct cemu_args *args)
{
    const uint64_t n = args->cparam1;
    const uint64_t base = args->cparam2;

    uint8_t *out = args->mr_addr[0];
    const uint64_t out_len = args->mr_len[0];
    const uint8_t *data = args->mr_addr[1];
    const uint64_t in_len = args->mr_len[1];
    const uint8_t *grid = args->mr_addr[2];
    const uint64_t grid_len = args->mr_len[2];
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
            /* 只处理整个区间都落在本窗口内的查询 */
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

    if (!grid) {
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

        weak = grid_range_crc(grid, grid_len, base + pos, len, &ok);
        if (!ok) {
            break;   /* 网格覆盖不上：窗口与网格范围不一致 */
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

                p->old_off = base + pos;
                p->new_off = new_off;
                p->len = (uint32_t)len;
                p->weak = weak;
                csd_sha1(data + pos, len, p->strong);
                memset(p->strong + CSD_SHA1_LEN, 0, DSYNC_FP_LEN - CSD_SHA1_LEN);
                nr++;
            }
        } else {
            struct gd_ent *e = (struct gd_ent *)(void *)(ents + nr * entry_bytes);

            e->off = base + pos;
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
