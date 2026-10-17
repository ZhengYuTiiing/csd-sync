/*
 * sync_cdc.c —— 挑战五 CSF：设备内 CDC 切分 + 可组合 CRC 比对，直接吐"扇区位图"
 *
 * 与 rsync_match / dsync_match 的区别：那两个算子把"老文件指纹表"当输入，
 * 需要主机先把老文件扫一遍建表。本算子不需要任何预建表——老文件的索引就是
 * 写入时固化在 NAND OOB 里的 512B 扇区 CRC32C 网格，内容无关、永久有效。
 *
 * 一次调用完成三件事：
 *   1. 对自己的输入数据段跑 CDC（切点只在 512B 边界采样，保证块是整扇区）
 *   2. 对每个块，用网格"拼"出老文件同位置同长度的 CRC32C（零数据读）
 *   3. 逐扇区写位图：1 = 该扇区与老文件相同，可直接复用；0 = 需要写入新内容
 *
 * 语义边界：这是**同位置比对**（A 的 [o,o+L) 对 B 的 [o,o+L)），
 * 不是 rsync 那种任意偏移搜索。CDC 的作用是让局部修改只影响邻近几个块，
 * 而不是整文件失效。
 *
 * CSF 接口（indirect pipeline，R -> C -> W）:
 *   mr[0] : 输出扇区位图（主机预清零）
 *   mr[1] : Stage-0 读入的新文件数据段
 *   mr[2] : Stage-A 浅读下来的老文件 OOB 网格（32B/页）
 *   cparam1 : 本段有效字节数 n
 *   cparam2 : 本段在文件内的起始偏移 base（512B 对齐）
 *   返回     : 命中字节数；<0 表示错误
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "chunk_abi.h"      /* CHUNK_GEAR_MASK / CHUNK_MIN_SIZE，与主机 parasync 同源 */
#include "chunk_gear_v2.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"

#define SYNC_CDC_ALIGN      512ULL
#define SYNC_CDC_MAX_CHUNK  65536ULL   /* 64KB 上界，同时保证是 512 的倍数 */

#define SYNC_CDC_EINVAL     ((long long)-1)
#define SYNC_CDC_ENOSPC     ((long long)-2)

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
    if (len == 0 || (off % SYNC_CDC_ALIGN) || (len % SYNC_CDC_ALIGN)) {
        return 0;
    }

    sec  = off / SYNC_CDC_ALIGN;
    nsec = len / SYNC_CDC_ALIGN;
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

long long sync_cdc(struct cemu_args *args)
{
    uint8_t *bitmap;
    const uint8_t *data, *grid;
    uint64_t bitmap_cap, data_cap, grid_len, n, base;
    uint64_t pos, nr_chunks = 0, matched_bytes = 0;

    if (!args || args->numr < 3 || !args->mr_addr || !args->mr_len) {
        return SYNC_CDC_EINVAL;
    }

    bitmap     = (uint8_t *)args->mr_addr[0];
    data       = (const uint8_t *)args->mr_addr[1];
    grid       = (const uint8_t *)args->mr_addr[2];
    bitmap_cap = (uint64_t)args->mr_len[0];
    data_cap   = (uint64_t)args->mr_len[1];
    grid_len   = (uint64_t)args->mr_len[2];
    n          = (uint64_t)args->cparam1;
    base       = (uint64_t)args->cparam2;

    if (!bitmap || !data || !grid) {
        return SYNC_CDC_EINVAL;
    }
    if (n == 0) {
        return 0;
    }
    if (n > data_cap || (n % SYNC_CDC_ALIGN) || (base % SYNC_CDC_ALIGN)) {
        return SYNC_CDC_EINVAL;
    }

    pos = 0;
    while (pos < n) {
        uint64_t limit = pos + SYNC_CDC_MAX_CHUNK;
        uint64_t cut, len, s0, s1;
        uint64_t fp = 0;
        uint32_t crc_new, crc_old;
        int ok = 0;
        int match;

        if (limit > n) {
            limit = n;
        }

        /*
         * CDC：切点只在 512B 边界采样，这样每个块都是整扇区，
         * 网格才能"拼"出它的 CRC。这是与主机 chunker 的唯一语义差异，
         * 代价是候选切点少了 512 倍，块长略不精确。
         */
        cut = limit;
        for (uint64_t i = pos; i < limit; i++) {
            uint64_t end;

            fp = (fp << 1) + chunk_gear_table[data[i]];
            end = i + 1;
            if ((end - pos) < CHUNK_MIN_SIZE) {
                continue;
            }
            if (end % SYNC_CDC_ALIGN) {
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

        crc_new = csd_crc32c(0, data + pos, len);
        crc_old = grid_range_crc(grid, grid_len, base + pos, len, &ok);
        match   = ok && (crc_new == crc_old);

        s0 = (base + pos) / SYNC_CDC_ALIGN;
        s1 = s0 + len / SYNC_CDC_ALIGN;
        if ((s1 + 7) / 8 > bitmap_cap) {
            return SYNC_CDC_ENOSPC;
        }
        for (uint64_t s = s0; s < s1; s++) {
            if (match) {
                bitmap[s >> 3] |= (uint8_t)(1u << (s & 7));
            } else {
                bitmap[s >> 3] &= (uint8_t)~(1u << (s & 7));
            }
        }

        if (match) {
            matched_bytes += len;
        }
        nr_chunks++;
        pos = cut;
    }

    return (long long)matched_bytes;
}
