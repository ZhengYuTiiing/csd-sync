/*
 * cdc_desc.c —— 路线甲 CSF：设备内 CDC 切分，产出"块描述符表"
 *
 * 这一条命令由 client 与 server **各自**对**自己**的文件跑一遍：
 *   client 对 f' 跑 → 产出 f' 的块描述符表
 *   server 对 f  跑 → 产出 f  的块描述符表
 * 之后由 dsync_match 把两张表做"弱哈希初筛 -> 强哈希确认"的两级匹配。
 *
 * 每个块产出三元组：
 *   weak   : CRC32C，由 512B CRC 网格"拼"出来，**不重新扫描数据**
 *   strong : SHA1(20B)，装进 32B 字段；与 A 组 dsync 的强哈希同算法
 *   (off,len)：块在文件内的位置，切点只在 512B 边界采样
 *
 * 为什么切点必须 512B 对齐：只有整扇区块才能由网格槽位拼出 CRC。
 *
 * 三个声明量在提交时可算：
 *   - 处理对象 : a priori 已知的一段文件区间（窗口大小由主机定）
 *   - MRS 容量 : 输出按"每 4KB 一个块"的硬上界分配
 *   - 执行时长 : 正比于输入字节数 n 与内容无关（CDC 线性扫描 + 定长组合）
 *
 * CSF 接口（indirect pipeline，R -> C -> W）:
 *   mr[0] : 输出 dsync_table_hdr + N × dsync_chunk_desc
 *   mr[1] : Stage-0 读入的本方数据段（R 阶段，CSD_READ_MODE_DATA）
 *   mr[2] : Stage-A 浅读下来的本方 OOB 网格（32B/页）
 *   cparam1 : 本段有效字节数 n
 *   cparam2 : 本段在文件内的起始偏移 base（512B 对齐）
 *   返回     : 写出的条目数 N；<0 失败
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "cemu_def.h"
#include "dsync_abi.h"
#include "chunk_abi.h"      /* CHUNK_GEAR_MASK / CHUNK_MIN_SIZE，与主机 parasync 同源 */
#include "chunk_gear_v2.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"
#include "../../../hw/femu/backend/csd_sha1.h"      /* 强校验和：与 A 组同为 SHA1 */

#define CDC_ALIGN      512ULL
#define CDC_MAX_CHUNK  65536ULL   /* 64KB 上界，同时是 512 的倍数 */

#define CDC_EINVAL ((long long)-1)
#define CDC_ENOSPC ((long long)-2)

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
    if (len == 0 || (off % CDC_ALIGN) || (len % CDC_ALIGN)) {
        return 0;
    }

    sec  = off / CDC_ALIGN;
    nsec = len / CDC_ALIGN;
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

long long cdc_desc(struct cemu_args *args)
{
    uint8_t *out;
    const uint8_t *data, *grid;
    uint64_t out_cap, data_cap, grid_len, n, base;
    uint64_t pos, nr = 0;

    if (!args || args->numr < 3 || !args->mr_addr || !args->mr_len) {
        return CDC_EINVAL;
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
        return CDC_EINVAL;
    }
    if (n > data_cap || (n % CDC_ALIGN) || (base % CDC_ALIGN)) {
        return CDC_EINVAL;
    }
    if (out_cap < sizeof(struct dsync_table_hdr)) {
        return CDC_EINVAL;
    }
    if (n == 0) {
        return 0;
    }

    {
        struct dsync_table_hdr *hdr = (struct dsync_table_hdr *)(void *)out;
        hdr->magic          = DSYNC_TABLE_MAGIC;
        hdr->nr_chunks      = 0;
        hdr->entry_bytes    = (uint32_t)sizeof(struct dsync_chunk_desc);
        hdr->total_data_len = 0;
    }

    pos = 0;
    while (pos < n) {
        uint64_t limit = pos + CDC_MAX_CHUNK;
        uint64_t cut, len, fp = 0;
        int ok = 0;

        if (limit > n) {
            limit = n;
        }

        /*
         * CDC：切点只在 512B 边界采样。
         * 这是与通用 CDC 唯一的语义差异 —— 切点多一层对齐约束，
         * 换来的是每个块都是整扇区，网格才能拼出它的 CRC32C。
         */
        cut = limit;
        for (uint64_t i = pos; i < limit; i++) {
            uint64_t end;

            fp = (fp << 1) + chunk_gear_table[data[i]];
            end = i + 1;
            if ((end - pos) < CHUNK_MIN_SIZE) {
                continue;
            }
            if (end % CDC_ALIGN) {
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

        if (sizeof(struct dsync_table_hdr) +
            (nr + 1) * sizeof(struct dsync_chunk_desc) > out_cap) {
            return CDC_ENOSPC;
        }

        {
            struct dsync_chunk_desc *d =
                (struct dsync_chunk_desc *)(void *)(out + sizeof(struct dsync_table_hdr) +
                                                    nr * sizeof(struct dsync_chunk_desc));

            /* 弱哈希：由网格拼出来，不扫描数据 */
            d->offset    = base + pos;
            d->length    = (uint32_t)len;
            d->weak_hash = grid_range_crc(grid, grid_len, base + pos, len, &ok);
            if (!ok) {
                d->weak_hash = 0;   /* 网格覆盖不到则退化成 0，交给后续比对判否 */
            }

            /*
             * 强校验和：SHA1，与 A 组（dsync, algo=0）同算法。
             * SHA1 输出 20B，其余 12B 补 0 —— 比对是定长 memcmp，
             * 补齐不影响正确性，也不必改描述符 ABI。
             */
            csd_sha1(data + pos, len, d->strong_hash);
            memset(d->strong_hash + CSD_SHA1_LEN, 0, DSYNC_FP_LEN - CSD_SHA1_LEN);
        }
        nr++;
        pos = cut;
    }

    {
        struct dsync_table_hdr *hdr = (struct dsync_table_hdr *)(void *)out;
        hdr->nr_chunks      = (uint32_t)nr;
        hdr->total_data_len = n;
    }

    return (long long)nr;
}
