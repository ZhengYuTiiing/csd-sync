/*
 * chunk_manifest_grid.c —— skysync-c 的 CSD 版 client 算子
 *
 * 与 chunk_manifest.c（dsync-CSD 用）唯一的语义差异：
 *   块的 CRC32c 不从托盘数据现算，而是由写入时固化的 512B CRC 网格
 *   "折叠"出来（与 cdc_desc 的 grid_range_crc 同源）——
 *   对应主机版 skysync-c "弱哈希取自存储层 CRC" 的语义。
 *
 * 切点仍然要从数据扫出（CDC 语义），所以数据段照读；差别只在弱哈希来源。
 *
 * CSF 接口（indirect pipeline，R -> C -> W）:
 *   mr[0] : 输出块清单数组（manifest_ent，与 chunk_manifest 完全同 ABI）
 *   mr[1] : Stage-0 读入的 f' 数据段
 *   mr[2] : collect_oob 预先装好的 f' OOB 网格（32B/页）
 *   cparam1 : 本段有效字节数 n
 *   cparam2 : 本段在 f' 内的起始偏移 base
 *   返回     : 写出的条目数 N；<0 失败
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "cemu_manifest.h"
#include "chunk_abi.h"
#include "chunk_gear_v2.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"

#define MANIFEST_ALIGN      512ULL
#define MANIFEST_MAX_CHUNK  65536ULL   /* 64KB 上界，同时是 512 的倍数 */

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
    if (len == 0 || (off % MANIFEST_ALIGN) || (len % MANIFEST_ALIGN)) {
        return 0;
    }

    sec  = off / MANIFEST_ALIGN;
    nsec = len / MANIFEST_ALIGN;
    page = sec / CSD_CRC_PER_PAGE;
    if ((page + 1) * CSD_CRC_SLOT_LEN > grid_len) {
        return 0;
    }

    crc = csd_crc_slot_get(grid + page * CSD_CRC_SLOT_LEN,
                           (uint32_t)(sec % CSD_CRC_PER_PAGE));

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

long long chunk_manifest_grid(struct cemu_args *args)
{
    uint8_t *out;
    const uint8_t *data, *grid;
    uint64_t out_cap, data_cap, grid_len, n, base, pos;
    uint64_t nr = 0;

    if (!args || args->numr < 3 || !args->mr_addr || !args->mr_len) {
        return CHUNK_CSF_EINVAL;
    }

    out      = (uint8_t *)args->mr_addr[0];
    data     = (const uint8_t *)args->mr_addr[1];
    grid     = (const uint8_t *)args->mr_addr[2];
    out_cap  = (uint64_t)args->mr_len[0];
    data_cap = (uint64_t)args->mr_len[1];
    grid_len = (uint64_t)args->mr_len[2];
    n        = (uint64_t)args->cparam1;
    base     = (uint64_t)args->cparam2;

    if (!out || !data || !grid) {
        return CHUNK_CSF_EINVAL;
    }
    if (n > data_cap || (n % MANIFEST_ALIGN) || (base % MANIFEST_ALIGN)) {
        return CHUNK_CSF_EINVAL;
    }

    pos = 0;
    while (pos < n) {
        uint64_t limit = pos + MANIFEST_MAX_CHUNK;
        uint64_t cut, len, fp = 0;
        int ok = 0;

        if (limit > n) {
            limit = n;
        }

        /* CDC：切点只在 512B 边界采样，保证块是整扇区、网格可拼 */
        cut = limit;
        for (uint64_t i = pos; i < limit; i++) {
            uint64_t end;

            fp = (fp << 1) + chunk_gear_table[data[i]];
            end = i + 1;
            if ((end - pos) < CHUNK_MIN_SIZE) {
                continue;
            }
            if (end % MANIFEST_ALIGN) {
                continue;
            }
            if ((fp & CHUNK_GEAR_MASK) == 0) {
                cut = end;
                break;
            }
        }

        len = cut - pos;
        if (len == 0) {
            pos = cut;
            continue;
        }

        if ((nr + 1) * MANIFEST_ENT_BYTES > out_cap) {
            return CHUNK_CSF_ENOSPC;
        }

        {
            struct manifest_ent *e =
                (struct manifest_ent *)(void *)(out + nr * MANIFEST_ENT_BYTES);
            e->off    = base + pos;
            e->len    = (uint32_t)len;
            e->crc32c = grid_range_crc(grid, grid_len, base + pos, len, &ok);
            if (!ok) {
                e->crc32c = 0;   /* 网格覆盖不到则置 0，server 侧比对判否 */
            }
        }
        nr++;
        pos = cut;
    }

    return (long long)nr;
}
