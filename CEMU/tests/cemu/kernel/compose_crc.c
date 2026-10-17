/*
 * compose_crc.c —— 挑战二 CSF: 由定长 OOB 槽位拼出任意边界的校验和
 *
 * 输入 A (mr_addr[1]): 老文件的 OOB 网格流，每 4KB 页 32B = 8 个扇区 CRC32C
 * 输入 B (data_buffer): 查询表 {magic, nr_queries, flags} + nr_queries × {off, len}
 * 输出   (mr_addr[0]): nr_queries 个 uint32_t，按查询顺序排列
 *
 * off 相对输入流起点，且 off 与 len 都必须 512B 对齐 —— 这样任意边界
 * 都能由槽位直接拼出，无需读取任何数据页。
 *
 * 拼接原理： crc(A||B) = combine(crc(A), crc(B), len(B))
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"
#include "rsync_csd_abi.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"

long long compose_crc(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *grid = (const uint8_t *)args->mr_addr[1];   /* 32B/页 */
    uint8_t *out        = (uint8_t *)args->mr_addr[0];
    long long grid_len  = args->mr_len[1];
    long long out_cap   = args->mr_len[0];
    const uint8_t *buf  = (const uint8_t *)args->data_buffer;
    long long buf_len   = args->buffer_len;

    if (!grid || !out || !buf || buf_len < (long long)sizeof(struct crc_query_hdr)) {
        return -1;
    }

    const struct crc_query_hdr *hdr = (const struct crc_query_hdr *)buf;
    if (hdr->magic != RSYNC_CRC_QUERY_MAGIC) {
        return -1;
    }

    const uint32_t nr = hdr->nr_queries;
    if (buf_len < (long long)(sizeof(*hdr) + (size_t)nr * sizeof(struct crc_query))) {
        return -1;
    }
    if (out_cap < (long long)((size_t)nr * sizeof(uint32_t))) {
        return -1;
    }

    const struct crc_query *q = (const struct crc_query *)(buf + sizeof(*hdr));
    uint32_t *res = (uint32_t *)out;

    for (uint32_t i = 0; i < nr; i++) {
        if (q[i].len == 0 || (q[i].off % 512) || (q[i].len % 512)) {
            return -1;
        }

        const uint64_t first_sector = q[i].off / 512;
        const uint64_t nsec         = q[i].len / 512;
        const uint64_t first_page   = first_sector / CSD_CRC_PER_PAGE;
        const uint32_t first_idx    = (uint32_t)(first_sector % CSD_CRC_PER_PAGE);

        if ((first_page + 1) * CSD_CRC_SLOT_LEN > (uint64_t)grid_len) {
            return -1;
        }

        uint32_t crc = csd_crc_slot_get(grid + first_page * CSD_CRC_SLOT_LEN, first_idx);

        for (uint64_t k = 1; k < nsec; k++) {
            const uint64_t sector = first_sector + k;
            const uint64_t page   = sector / CSD_CRC_PER_PAGE;
            const uint32_t idx    = (uint32_t)(sector % CSD_CRC_PER_PAGE);

            if ((page + 1) * CSD_CRC_SLOT_LEN > (uint64_t)grid_len) {
                return -1;
            }
            crc = csd_crc32c_combine(crc,
                                     csd_crc_slot_get(grid + page * CSD_CRC_SLOT_LEN, idx),
                                     512);
        }

        res[i] = crc;
    }

    return (long long)(nr * sizeof(uint32_t));
}
