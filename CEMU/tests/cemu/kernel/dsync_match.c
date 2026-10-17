/*
 * dsync_match.c —— 挑战三 CSF: dsync 变长分块双哈希流式匹配算子
 *
 * 两级比对流水线:
 *   1. 弱哈希初筛 (CRC32C, 32-bit): 极速过滤大部分不相干块
 *   2. 强哈希确认 (BLAKE3/256-bit, 32B): 对初筛命中的候选块比对完整指纹，杜绝碰撞
 *
 * 内存恒定在 KB 级:
 *   老 chunk 描述符分批流过 mr_addr[1]，即比即清；
 *   新 chunk 弱哈希建立紧凑的定长桶索引，不产生动态内存分配开销。
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"
#include "dsync_abi.h"

#define DSYNC_BUCKET_SIZE 4096
#define DSYNC_BUCKET_MASK (DSYNC_BUCKET_SIZE - 1)

long long dsync_match(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    /*
     * 老 chunk 列表所在区：
     *   numr >= 3 → mr[2]。indirect 流水线的 R 阶段恒把数据读进 mr[1]，
     *               本算子又不需要读 NVM，mr[1] 会被一次哑读覆盖，
     *               所以真正的输入必须放到 mr[2]。
     *   numr == 2 → mr[1]。宿主机 dlopen 直调（benchmark/test）走这条，
     *               保持向后兼容。
     */
    const int in_idx = (args->numr >= 3) ? 2 : 1;
    const uint8_t *in_buf   = (const uint8_t *)args->mr_addr[in_idx];
    uint8_t *out_buf        = (uint8_t *)args->mr_addr[0];
    long long in_len        = args->mr_len[in_idx];
    long long out_cap       = args->mr_len[0];
    const uint8_t *data_buf = (const uint8_t *)args->data_buffer;
    long long data_len      = args->buffer_len;

    if (!in_buf || !out_buf || !data_buf) {
        return -1;
    }
    if (data_len < (long long)sizeof(struct dsync_table_hdr)) {
        return -1;
    }

    const struct dsync_table_hdr *new_hdr = (const struct dsync_table_hdr *)data_buf;
    if (new_hdr->magic != DSYNC_TABLE_MAGIC) {
        return -1;
    }

    uint32_t nr_new = new_hdr->nr_chunks;
    const struct dsync_chunk_desc *new_chunks =
        (const struct dsync_chunk_desc *)(data_buf + sizeof(*new_hdr));

    if (data_len < (long long)(sizeof(*new_hdr) + nr_new * sizeof(struct dsync_chunk_desc))) {
        return -1;
    }

    /* 解析老 chunk 列表 (支持带 header 或纯 chunk_desc 数组) */
    uint32_t nr_old = 0;
    const struct dsync_chunk_desc *old_chunks = NULL;

    if (in_len >= (long long)sizeof(struct dsync_table_hdr)) {
        const struct dsync_table_hdr *old_hdr = (const struct dsync_table_hdr *)in_buf;
        if (old_hdr->magic == DSYNC_TABLE_MAGIC) {
            nr_old = old_hdr->nr_chunks;
            old_chunks = (const struct dsync_chunk_desc *)(in_buf + sizeof(*old_hdr));
        }
    }
    if (!old_chunks) {
        nr_old = (args->cparam1 > 0) ? (uint32_t)args->cparam1 : (uint32_t)(in_len / sizeof(struct dsync_chunk_desc));
        old_chunks = (const struct dsync_chunk_desc *)in_buf;
    }

    if (out_cap < (long long)sizeof(struct dsync_match_hdr)) {
        return -1;
    }

    struct dsync_match_hdr *mhdr = (struct dsync_match_hdr *)out_buf;
    mhdr->magic         = DSYNC_MATCH_MAGIC;
    mhdr->nr_matched    = 0;
    mhdr->nr_unmatched  = 0;
    mhdr->matched_bytes = 0;

    struct dsync_match_token *tokens =
        (struct dsync_match_token *)(out_buf + sizeof(*mhdr));
    uint32_t max_tokens = (uint32_t)((out_cap - sizeof(*mhdr)) / sizeof(struct dsync_match_token));

    /* 为新 chunk 弱哈希建立紧凑桶索引 (1-based index) */
    uint32_t buckets[DSYNC_BUCKET_SIZE];
    memset(buckets, 0, sizeof(buckets));

    for (uint32_t i = 0; i < nr_new; i++) {
        uint32_t slot = new_chunks[i].weak_hash & DSYNC_BUCKET_MASK;
        for (uint32_t step = 0; step < DSYNC_BUCKET_SIZE; step++) {
            uint32_t idx = (slot + step) & DSYNC_BUCKET_MASK;
            if (buckets[idx] == 0) {
                buckets[idx] = i + 1;
                break;
            }
        }
    }

    /* 逐个老 chunk 比对 */
    for (uint32_t i = 0; i < nr_old; i++) {
        const struct dsync_chunk_desc *oc = &old_chunks[i];
        uint32_t slot = oc->weak_hash & DSYNC_BUCKET_MASK;
        int matched = 0;
        uint64_t matched_new_off = 0;

        for (uint32_t step = 0; step < DSYNC_BUCKET_SIZE; step++) {
            uint32_t b = buckets[(slot + step) & DSYNC_BUCKET_MASK];
            if (b == 0) {
                break;
            }
            uint32_t candidate_idx = b - 1;
            const struct dsync_chunk_desc *nc = &new_chunks[candidate_idx];

            /* 第一级：弱哈希初筛 */
            if (nc->weak_hash == oc->weak_hash && nc->length == oc->length) {
                /* 第二级：强哈希确认 */
                if (memcmp(nc->strong_hash, oc->strong_hash, DSYNC_FP_LEN) == 0) {
                    matched = 1;
                    matched_new_off = nc->offset;
                    break;
                }
            }
        }

        if (matched) {
            if (mhdr->nr_matched < max_tokens) {
                tokens[mhdr->nr_matched].old_offset = oc->offset;
                tokens[mhdr->nr_matched].new_offset = matched_new_off;
                tokens[mhdr->nr_matched].length     = oc->length;
                tokens[mhdr->nr_matched].reserved   = 0;
            }
            mhdr->nr_matched++;
            mhdr->matched_bytes += oc->length;
        } else {
            mhdr->nr_unmatched++;
        }
    }

    long long total_out = sizeof(*mhdr) + mhdr->nr_matched * sizeof(struct dsync_match_token);
    return (total_out + 511) / 512;
}
