/*
 * rsync_match.c —— 挑战三 CSF: rsync 窗口化流式指纹匹配算子
 *
 * 算法原理 (流式双输入比对):
 *   输入 A (mr_addr[1]): Stage 0 浅读进来的老文件本 chunk 32B 指纹流 (恒定在 KB 级)
 *   输入 B (data_buffer): 主机一次性预下发的新文件指纹全表 (或已索引表)
 *   输出   (mr_addr[0]): 本 chunk 匹配结果 (rsync_match_hdr + matched tokens + changed blks)
 *
 * 内存与计算特性:
 *   - 避免在板载小 DRAM 上全量建 uthash 树 (解决挑战三)
 *   - 老指纹分批流经 FDM，随比随清
 *   - 返回值: 匹配条数 (dest=0 时)，或写回 NVM 块数 (dest=1 时)
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"
#include "rsync_csd_abi.h"

#define HASH_BUCKET_SIZE 4096
#define HASH_MASK        (HASH_BUCKET_SIZE - 1)

static inline uint32_t fp_hash(const uint8_t *fp)
{
    uint32_t h;
    memcpy(&h, fp, sizeof(uint32_t));
    return h;
}

long long rsync_match(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *old_fp_stream = (const uint8_t *)args->mr_addr[1];
    uint8_t *out_buf             = (uint8_t *)args->mr_addr[0];
    long long in_len             = args->mr_len[1];
    long long out_cap            = args->mr_len[0];
    const uint8_t *data_buf      = (const uint8_t *)args->data_buffer;
    long long data_len           = args->buffer_len;

    if (!old_fp_stream || !out_buf || !data_buf) {
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

    /* 老文件本 chunk 的块数与基础偏移 */
    uint32_t base_old_blk = (uint32_t)args->cparam1;
    uint32_t nr_old       = (uint32_t)args->cparam2;
    if (nr_old == 0) {
        nr_old = (uint32_t)(in_len / RSYNC_CSD_FP_LEN);
    }

    /* 输出区结构预留 */
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

    /*
     * 轻量哈希桶索引 (针对新指纹表)，KB 级板载 DRAM 友好
     * 用开放寻址法哈希表，存 entry 索引 (1-based, 0 表示空)
     */
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

    /* 窗口扫描老指纹 */
    for (uint32_t i = 0; i < nr_old; i++) {
        const uint8_t *old_fp = old_fp_stream + i * RSYNC_CSD_FP_LEN;
        uint32_t slot = fp_hash(old_fp) & HASH_MASK;
        int found = 0;
        uint32_t matched_new_idx = 0;

        for (uint32_t step = 0; step < HASH_BUCKET_SIZE; step++) {
            uint32_t b = buckets[(slot + step) & HASH_MASK];
            if (b == 0) {
                break; /* 链结束 */
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

    /* 计算写出所占 512B 扇区数 */
    long long total_bytes = sizeof(*mhdr) + mhdr->nr_matched * sizeof(struct rsync_match_token);
    return (total_bytes + 511) / 512;
}
