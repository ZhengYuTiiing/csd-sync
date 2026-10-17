/*
 * rsync_match_data.c —— rsync-CSD 的 server 侧匹配算子（现场算版）
 *
 * 与 rsync_match.c（skysync-f-CSD 用）的差异只在老文件指纹的来源：
 *   rsync_match.c    : mr[1] = Stage 0 浅读的 OOB 32B/页指纹流（存储层现成）
 *   rsync_match_data : mr[1] = Stage 0 读入的老文件**数据**（CSD_READ_MODE_DATA），
 *                      本算子在设备内逐 4KB 页现场算 csd_fp256 再比对。
 *                      对应主机版 rsync "现场算校验和" 的语义。
 *
 * 新文件指纹表仍由主机下发（data_buffer），与 rsync_match.c 完全同构。
 *
 * CSF 接口:
 *   mr[0] : 输出匹配结果 (rsync_match_hdr + tokens)
 *   mr[1] : Stage-0 读入的老文件数据段（本窗口）
 *   data_buffer : 新文件指纹全表（rsync_fp_table_hdr + entries）
 *   cparam1 : 本窗口首块的全局块号 base_old_blk
 *   cparam2 : 本窗口块数 nr_old（0 则按 in_len/4KB 推导）
 *   返回     : 输出占用的 512B 扇区数
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"
#include "rsync_csd_abi.h"
#include "../../../hw/femu/backend/csd_fingerprint.h"

#define HASH_BUCKET_SIZE 4096
#define HASH_MASK        (HASH_BUCKET_SIZE - 1)

static inline uint32_t fp_hash(const uint8_t *fp)
{
    uint32_t h;
    memcpy(&h, fp, sizeof(uint32_t));
    return h;
}

long long rsync_match_data(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *old_data  = (const uint8_t *)args->mr_addr[1];
    uint8_t *out_buf         = (uint8_t *)args->mr_addr[0];
    long long in_len         = args->mr_len[1];
    long long out_cap        = args->mr_len[0];
    const uint8_t *data_buf  = (const uint8_t *)args->data_buffer;
    long long data_len       = args->buffer_len;

    if (!old_data || !out_buf || !data_buf) {
        return -1;
    }
    if (data_len < (long long)sizeof(struct rsync_fp_table_hdr)) {
        return -1;
    }

    const struct rsync_fp_table_hdr *new_hdr = (const struct rsync_fp_table_hdr *)data_buf;
    if (new_hdr->magic != RSYNC_FP_TABLE_MAGIC) {
        return -1;
    }

    uint32_t nr_new = new_hdr->nr_entries;
    const struct rsync_fp_entry *new_entries =
        (const struct rsync_fp_entry *)(data_buf + sizeof(*new_hdr));

    if (data_len < (long long)(sizeof(*new_hdr) + nr_new * sizeof(struct rsync_fp_entry))) {
        return -1;
    }

    uint32_t base_old_blk = (uint32_t)args->cparam1;
    uint32_t nr_old       = (uint32_t)args->cparam2;
    if (nr_old == 0) {
        nr_old = (uint32_t)(in_len / RSYNC_CSD_BLOCK_SIZE);
    }
    if ((uint64_t)nr_old * RSYNC_CSD_BLOCK_SIZE > (uint64_t)in_len) {
        nr_old = (uint32_t)(in_len / RSYNC_CSD_BLOCK_SIZE);
    }

    if (out_cap < (long long)sizeof(struct rsync_match_hdr)) {
        return -1;
    }

    struct rsync_match_hdr *mhdr = (struct rsync_match_hdr *)out_buf;
    mhdr->magic      = RSYNC_MATCH_MAGIC;
    mhdr->chunk_id   = base_old_blk;
    mhdr->nr_matched = 0;
    mhdr->nr_changed = 0;
    mhdr->reserved   = 0;

    struct rsync_match_token *tokens =
        (struct rsync_match_token *)(out_buf + sizeof(*mhdr));
    uint32_t max_tokens = (uint32_t)((out_cap - sizeof(*mhdr)) / sizeof(struct rsync_match_token));

    uint32_t buckets[HASH_BUCKET_SIZE];
    memset(buckets, 0, sizeof(buckets));

    for (uint32_t i = 0; i < nr_new; i++) {
        uint32_t slot = fp_hash(new_entries[i].fp) & HASH_MASK;
        for (uint32_t step = 0; step < HASH_BUCKET_SIZE; step++) {
            uint32_t idx = (slot + step) & HASH_MASK;
            if (buckets[idx] == 0) {
                buckets[idx] = i + 1;
                break;
            }
        }
    }

    for (uint32_t i = 0; i < nr_old; i++) {
        uint8_t old_fp[RSYNC_CSD_FP_LEN];

        /* 现场算：对老文件本页数据算 32B 指纹（数据已在托盘里） */
        csd_fp256(old_data + (size_t)i * RSYNC_CSD_BLOCK_SIZE,
                  RSYNC_CSD_BLOCK_SIZE, old_fp);

        uint32_t slot = fp_hash(old_fp) & HASH_MASK;
        int found = 0;
        uint32_t matched_new_idx = 0;

        for (uint32_t step = 0; step < HASH_BUCKET_SIZE; step++) {
            uint32_t b = buckets[(slot + step) & HASH_MASK];
            if (b == 0) {
                break;
            }
            uint32_t entry_idx = b - 1;
            if (memcmp(old_fp, new_entries[entry_idx].fp, RSYNC_CSD_FP_LEN) == 0) {
                found = 1;
                matched_new_idx = new_entries[entry_idx].blk_idx;
                break;
            }
        }

        if (found) {
            if (mhdr->nr_matched < max_tokens) {
                tokens[mhdr->nr_matched].old_blk_idx = base_old_blk + i;
                tokens[mhdr->nr_matched].new_blk_idx = matched_new_idx;
            }
            mhdr->nr_matched++;
        } else {
            mhdr->nr_changed++;
        }
    }

    long long total_bytes = sizeof(*mhdr) + mhdr->nr_matched * sizeof(struct rsync_match_token);
    return (total_bytes + 511) / 512;
}
