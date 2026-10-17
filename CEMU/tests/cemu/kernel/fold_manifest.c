/*
 * fold_manifest.c —— server 侧 CSF：按 512B CRC 网格逐条折叠清单，产出"匹配表"
 *
 * 这是整套方案里唯一"不能迭代"的理由所在：
 * server **不读取 f 的一个数据字节**，只做两件事
 *   1. Stage-0 浅读 f 的 OOB，得到 512B 扇区 CRC32C 网格（32B / 4KB 页）
 *   2. 对每个清单条目，在若干候选偏移上组合出 f 的 CRC32C，与清单里的
 *      CRC32C 做定长比较，命中则记入匹配表
 *
 * ── 为什么要候选偏移 ────────────────────────────────────────────────
 * 朴素做法是"位置对齐"比较：f[off] ?= f'[off]。这在 f' 只是局部改写时成立，
 * 但一旦发生插入/删除，编辑点之后所有块在 f 与 f' 中的偏移就整体错开了，
 * 位置对齐会全线失配 —— CDC 的移位容忍特性也就白费了。
 *
 * 设插入/删除的净字节数为 B = |f'| - |f|，则编辑点之后恒有
 *     f[x] == f'[x + B]        （B > 0 为插入）
 * 于是对清单条目 (off, len)，在 f 内的候选位置就是 { off, off - B }。
 * 每个候选位置都由网格组合出 CRC32C，一次 4 字节比较即可。
 *
 * 折叠次数恒等于 N × nr_deltas —— 上界下界相同，与 f 和 f' 的内容都无关。
 * 所以这条命令的三个声明量在提交时就是常数：
 *   - 处理对象 : 清单本身是命令的输入，N 已知；候选偏移个数由主机定
 *   - MRS 容量 : 匹配表按"条目数 <= N"的硬上界分配
 *   - 执行时长 : 正比于 N × nr_deltas
 *
 * CSF 接口（indirect pipeline，R -> C -> W）:
 *   mr[0] : 输出匹配表（主机预清零）
 *   mr[1] : Stage-0 浅读下来的 f 的 OOB 网格（32B/页，按下标寻址）
 *   mr[2] : 输入块清单（client 产出，经"网络"搬到 server）
 *   mr[3] : 折叠参数块 struct fold_param（含候选偏移数组）
 *   cparam1 : 清单条数 N
 *   返回     : 命中的字节数；<0 失败
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "cemu_manifest.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"

#define FOLD_ALIGN  512ULL

#define FOLD_EINVAL ((long long)-1)
#define FOLD_ENOSPC ((long long)-2)

/*
 * 从网格拼出 [off, off+len) 的 CRC32C，全程不碰数据页。
 * off / len 必须是 512B 整数倍，否则返回 ok=0。
 */
static inline uint32_t grid_range_crc(const uint8_t *grid, uint64_t grid_len,
                                      uint64_t off, uint64_t len, int *ok)
{
    uint64_t sec, nsec, page, s, p;
    uint32_t crc;

    *ok = 0;
    if (len == 0 || (off % FOLD_ALIGN) || (len % FOLD_ALIGN)) {
        return 0;
    }

    sec  = off / FOLD_ALIGN;
    nsec = len / FOLD_ALIGN;
    page = sec / CSD_CRC_PER_PAGE;
    if ((page + 1) * CSD_CRC_SLOT_LEN > grid_len) {
        return 0;
    }

    crc = csd_crc_slot_get(grid + page * CSD_CRC_SLOT_LEN,
                           (uint32_t)(sec % CSD_CRC_PER_PAGE));

    /* 相邻格拼接：crc(A||B) = combine(crc(A), crc(B), |B|)，|B| 恒为 512 */
    for (uint64_t k = 1; k < nsec; k++) {
        s = sec + k;
        p = s / CSD_CRC_PER_PAGE;
        if ((p + 1) * CSD_CRC_SLOT_LEN > grid_len) {
            return 0;
        }
        crc = csd_crc32c_combine512(
            crc,
            csd_crc_slot_get(grid + p * CSD_CRC_SLOT_LEN,
                             (uint32_t)(s % CSD_CRC_PER_PAGE)));
    }

    *ok = 1;
    return crc;
}

long long fold_manifest(struct cemu_args *args)
{
    uint8_t *mtab;
    const uint8_t *grid;
    const struct manifest_ent *ent;
    const struct fold_param *par;
    const int64_t *deltas;
    uint64_t mtab_cap, grid_len, nr, par_len, old_nlb, nr_deltas, old_bytes;
    uint64_t matched_bytes = 0, nr_out = 0;

    if (!args || args->numr < 4 || !args->mr_addr || !args->mr_len) {
        return FOLD_EINVAL;
    }

    mtab     = (uint8_t *)args->mr_addr[0];
    grid     = (const uint8_t *)args->mr_addr[1];
    ent      = (const struct manifest_ent *)args->mr_addr[2];
    par      = (const struct fold_param *)args->mr_addr[3];
    mtab_cap = (uint64_t)args->mr_len[0];
    grid_len = (uint64_t)args->mr_len[1];
    par_len  = (uint64_t)args->mr_len[3];
    nr       = (uint64_t)args->cparam1;

    if (!mtab || !grid || !ent || !par) {
        return FOLD_EINVAL;
    }
    if (nr * MANIFEST_ENT_BYTES > (uint64_t)args->mr_len[2]) {
        return FOLD_EINVAL;
    }
    if (par_len < FOLD_PARAM_BASE_BYTES) {
        return FOLD_EINVAL;
    }
    if (nr == 0) {
        return 0;
    }

    old_nlb   = par->old_nlb;
    nr_deltas = par->nr_deltas;
    old_bytes = old_nlb * FOLD_ALIGN;

    if (nr_deltas == 0 ||
        par_len < FOLD_PARAM_BASE_BYTES + nr_deltas * (uint64_t)sizeof(int64_t)) {
        return FOLD_EINVAL;
    }
    deltas = par->deltas;

    for (uint64_t i = 0; i < nr; i++) {
        uint64_t tgt = ent[i].off;
        uint64_t len = ent[i].len;
        int64_t found = -1;

        /*
         * 折叠 + 定长比较。这一步是整个同步里唯一的"比对"动作，
         * 它退化成 N × nr_deltas 次 4 字节整数比较 —— 没有滚动哈希、
         * 没有候选搜索、没有任何依赖上一条结果的控制流。
         */
        for (uint64_t d = 0; d < nr_deltas && found < 0; d++) {
            int64_t delta = deltas[d];
            int64_t src;
            int ok = 0;
            uint32_t crc_old;

            /* 候选偏移必须也在 512B 边界上，否则网格拼不出来 */
            if ((delta % (int64_t)FOLD_ALIGN) != 0) {
                continue;
            }
            src = (int64_t)tgt - delta;
            if (src < 0) {
                continue;
            }
            if ((uint64_t)src + len > old_bytes) {
                continue;
            }

            crc_old = grid_range_crc(grid, grid_len, (uint64_t)src, len, &ok);
            if (ok && crc_old == ent[i].crc32c) {
                found = (int64_t)d;
            }
        }

        if (found < 0) {
            continue;   /* 未命中：不进表，主机按补集当字面量处理 */
        }

        if ((nr_out + 1) * MATCH_ENT_BYTES > mtab_cap) {
            return FOLD_ENOSPC;
        }
        {
            struct match_ent *m =
                (struct match_ent *)(void *)(mtab + nr_out * MATCH_ENT_BYTES);
            m->tgt_off   = tgt;
            m->len       = (uint32_t)len;
            m->delta_idx = (uint32_t)found;
        }
        nr_out++;
        matched_bytes += len;
    }

    return (long long)matched_bytes;
}
