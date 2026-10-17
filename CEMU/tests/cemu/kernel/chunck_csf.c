/*
 * chunk_csf.c —— ParaSync 文件同步的 CDC 分块算子（Stage-1，下沉到 CEMU CSD）
 *
 * 语义 = 主机 parasync/src/parasync/chunker.cpp 的 ParaSyncChunker::run()，
 *        外加"每切出一块就对该块算一次 CRC32"（主机 chunker 同一位置做的事）：
 *
 *     fp = 0; pre = 段首
 *     for i in [段首, 段尾):
 *         fp = (fp << 1) + GEARv2[buf[i]]
 *         if ((fp & 0x0000d93003530000) == 0) -> 切点在 i（i 属于下一块）
 *         块 = [pre, i)，weak_hash = CRC32(块)
 *     段尾残余 [pre, 段尾) 同样作为一块输出
 *   fp 在每段开头重置为 0，所以逐段切点可以独立复现（并行下沉的前提）。
 *
 * 为什么本算子只吐"原始切点"、不做 MinSize 合并：
 *   主机 Stage-2（ParaSyncWorker::detector，parasync_worker.cpp:28-83）的合并
 *   累加器是**跨段全局**的。段内先合并会让段边界处前缀长度不同，导致合并点整体
 *   漂移，最终与主机结果不一致。所以：Stage-1 出原始切点（本算子），
 *   Stage-2 出合并块（merge_csf）。
 *
 * CSF 接口（direct model + MRS）:
 *   mr[1] / mr_len[1] : 输入数据段（设备内存/FDM 映射）
 *   mr[0] / mr_len[0] : 输出切点表（chunk_abi.h 的 chunk_table_hdr + chunk_ent[]）
 *   cparam1           : 本段有效字节数 n
 *   cparam2           : 本段在文件中的起始偏移 base_offset
 *   返回值            : 切点条目数；CHUNK_CSF_ENOSPC 时主机需换更大的输出缓冲重试
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "chunk_abi.h"
#include "chunk_crc32.h"
#include "chunk_gear_v2.h"

/* 供主机校验 gear 表是否与 parasync 一致（测试用，不参与执行路径） */
const uint64_t *chunk_csf_gear_table(void);

long long chunk_csf(struct cemu_args *args)
{
    if (args == NULL || args->numr < 2 || args->mr_addr == NULL || args->mr_len == NULL)
        return CHUNK_CSF_EINVAL;

    const uint8_t *in = (const uint8_t *)args->mr_addr[1];
    uint8_t *out = (uint8_t *)args->mr_addr[0];
    uint64_t in_cap = (uint64_t)args->mr_len[1];
    uint64_t out_cap = (uint64_t)args->mr_len[0];
    uint64_t n = (uint64_t)args->cparam1;
    uint64_t base = (uint64_t)args->cparam2;

    if (in == NULL || out == NULL)
        return CHUNK_CSF_EINVAL;
    if (n > in_cap)
        return CHUNK_CSF_EINVAL;
    if (out_cap < sizeof(struct chunk_table_hdr))
        return CHUNK_CSF_ENOSPC;

    struct chunk_table_hdr *hdr = (struct chunk_table_hdr *)out;
    struct chunk_ent *ent = (struct chunk_ent *)(out + sizeof(*hdr));
    uint64_t ent_cap = (out_cap - sizeof(*hdr)) / sizeof(struct chunk_ent);

    hdr->magic = CHUNK_TABLE_MAGIC;
    hdr->nr_chunks = 0;
    hdr->base_offset = base;
    hdr->data_len = n;
    hdr->flags = 0;
    hdr->entry_bytes = 0;

    if (n == 0)
        return 0;

    uint64_t nr = 0;
    uint64_t cut_start = 0; /* 当前未闭合块的起点（段内偏移） */
    uint64_t fp = 0;

    for (uint64_t i = 0; i < n; i++) {
        fp = (fp << 1) + chunk_gear_table[in[i]];
        if ((fp & CHUNK_GEAR_MASK) == 0) {
            /* 触发切点的是第 i 个字节，它属于**下一块**（与主机 pre_offset = i 一致） */
            uint64_t len = i - cut_start;
            if (len == 0)
                continue; /* 与主机语义等价：零长块在合并阶段贡献 0 字节 */
            if (nr == ent_cap)
                return CHUNK_CSF_ENOSPC;
            ent[nr].offset = base + cut_start;
            ent[nr].length = len;
            ent[nr].crc32 = chunk_crc32(in + cut_start, len, 0);
            ent[nr].flags = CHUNK_ENT_CRC_VALID;
            nr++;
            cut_start = i;
        }
    }

    /* 段尾残余：主机 chunker 同样会无条件 push [pre_offset, seg_end) */
    if (cut_start < n) {
        uint64_t len = n - cut_start;
        if (nr == ent_cap)
            return CHUNK_CSF_ENOSPC;
        ent[nr].offset = base + cut_start;
        ent[nr].length = len;
        ent[nr].crc32 = chunk_crc32(in + cut_start, len, 0);
        ent[nr].flags = CHUNK_ENT_CRC_VALID | CHUNK_ENT_TAIL_OPEN;
        nr++;
        hdr->flags |= CHUNK_FLAG_TAIL_OPEN;
    }

    hdr->nr_chunks = nr;
    hdr->entry_bytes = nr * sizeof(struct chunk_ent);
    return (long long)nr;
}

const uint64_t *chunk_csf_gear_table(void)
{
    return chunk_gear_table;
}
