/*
 * chunk_abi.h —— CDC 算子族的共享 ABI（切点表 / 合并表）
 *
 * 设计原则：算子之间靠"表"接力，表本身是自描述的（magic + nr + entry_bytes），
 * 所以主机可以在设备侧把它们串起来，不需要把中间结果搬回主机解析。
 *
 *   数据段 --[chunk_csf]--> 切点表(原始 gear 切点 + 每块 CRC32)
 *          --[merge_csf]--> 合并表(>= MinSize 的块 + 合并后的 CRC32)
 *
 * 所有常量都必须与主机 parasync 逐位一致，改表 = 改 CDC 语义。
 */
#ifndef CHUNK_ABI_H
#define CHUNK_ABI_H

#include <stdint.h>

/* ---------- 必须与主机 parasync 保持一致 ---------- */
/* chunker.cpp: tls_FING_GEAR_08KB_64，13 个 0 bit，平均块 ~8KB */
#define CHUNK_GEAR_MASK 0x0000d93003530000ULL
/* fastcdc.cpp: MinSize = 8192 / 2 = 4KB，Stage-2 的合并阈值 */
#define CHUNK_MIN_SIZE 4096ULL

/* ---------- 表头 ---------- */
#define CHUNK_TABLE_MAGIC        0x43484e4b00000001ULL /* "CHNK" v1：Stage-1 切点表 */
#define CHUNK_TABLE_MAGIC_MERGED 0x43484e4b00000002ULL /* "CHNK" v2：Stage-2 合并表 */

/* 表级 flags */
#define CHUNK_FLAG_TAIL_OPEN (1ULL << 0) /* 最后一条是段尾残余/未闭合块 */

/* 条目 flags */
#define CHUNK_ENT_TAIL_OPEN (1u << 0) /* 该条不是真实切点（段尾残余 / 全局残余） */
#define CHUNK_ENT_CRC_VALID (1u << 1) /* crc32 字段有效 */
#define CHUNK_ENT_MERGED    (1u << 2) /* 由 merge_csf 合并产生 */

/*
 * 一个块。24 字节、自然对齐（MR 起始地址是页对齐的，所以无需 packed）。
 * offset 一律是**文件内绝对偏移**，方便下游直接做 old/new 匹配。
 */
struct chunk_ent {
    uint64_t offset;
    uint64_t length;
    uint32_t crc32; /* 标准 CRC-32（同主机 parasync_crc32） */
    uint32_t flags; /* CHUNK_ENT_* */
};

struct chunk_table_hdr {
    uint64_t magic;
    uint64_t nr_chunks;   /* out */
    uint64_t base_offset; /* in : 本表覆盖的起始文件偏移 */
    uint64_t data_len;    /* in : 本表覆盖的字节数 */
    uint64_t flags;       /* out: CHUNK_FLAG_* */
    uint64_t entry_bytes; /* out: nr_chunks * sizeof(struct chunk_ent) */
};

/* 返回码（<0 均视为失败；ENOSPC 表示主机需换更大的输出缓冲重试） */
#define CHUNK_CSF_EINVAL ((long long)-1)
#define CHUNK_CSF_ENOSPC ((long long)-2)

#endif /* CHUNK_ABI_H */
