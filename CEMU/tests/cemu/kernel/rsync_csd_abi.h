/*
 * rsync_csd_abi.h —— CSD rsync 下沉算子族共享 ABI
 *
 * 挑战二/三的核心数据结构定义：
 *   - 32 字节页指纹条目
 *   - 新旧指纹比对输出格式 (matched / changed tokens)
 *   - Delta patch 指令格式
 */

#ifndef RSYNC_CSD_ABI_H
#define RSYNC_CSD_ABI_H

#include <stdint.h>

#define RSYNC_CSD_FP_LEN        32
#define RSYNC_CSD_BLOCK_SIZE    4096

#define RSYNC_FP_TABLE_MAGIC    0x53594e4300000001ULL /* "SYNC" v1 */
#define RSYNC_MATCH_MAGIC       0x53594e4300000002ULL /* "SYNC" v2 */
#define RSYNC_DELTA_MAGIC       0x53594e4300000003ULL /* "SYNC" v3 */

/* 单条指纹条目 */
struct rsync_fp_entry {
    uint32_t blk_idx;                   /* 文件内块编号 (0, 1, 2, ...) */
    uint32_t flags;                     /* 保留标记 */
    uint8_t  fp[RSYNC_CSD_FP_LEN];      /* 32B 指纹 */
};

/* 主机预填并下发的新指纹表头 (位于 data_buffer 开头) */
struct rsync_fp_table_hdr {
    uint64_t magic;                     /* RSYNC_FP_TABLE_MAGIC */
    uint32_t nr_entries;                /* 新文件指纹条目总数 */
    uint32_t entry_bytes;               /* sizeof(struct rsync_fp_entry) */
    uint64_t file_size;                 /* 新文件总大小 */
};

/* 匹配命中 token: 记录老块与新块的对应 */
struct rsync_match_token {
    uint32_t old_blk_idx;               /* 老文件块号 */
    uint32_t new_blk_idx;               /* 对应的新文件块号 */
};

/* 匹配结果表头 (输出至 mr_addr[0]) */
struct rsync_match_hdr {
    uint64_t magic;                     /* RSYNC_MATCH_MAGIC */
    uint32_t nr_matched;                /* 匹配块数 */
    uint32_t nr_changed;                /* 变化块数 */
    uint32_t chunk_id;                  /* 当前 chunk id */
    uint32_t reserved;
};

/* Patch 算子 delta 命令 */
#define RSYNC_CMD_COPY_OLD      0       /* 复用老文件对应块 */
#define RSYNC_CMD_INSERT_LIT    1       /* 插入新文件字面量块 */

struct rsync_delta_cmd {
    uint32_t cmd_type;                  /* RSYNC_CMD_* */
    uint32_t blk_idx;                   /* 目标输出块号 */
    uint32_t old_blk_idx;               /* 若 COPY_OLD: 在当前输入 chunk 内的块索引 */
    uint32_t data_offset;               /* 若 INSERT_LIT: 在 data_buffer literal 区的字节偏移 */
};

struct rsync_delta_hdr {
    uint64_t magic;                     /* RSYNC_DELTA_MAGIC */
    uint32_t nr_cmds;                   /* delta 命令总数 */
    uint32_t blk_size;                  /* 块大小 (默认 4096) */
    uint32_t data_pool_offset;          /* literal pool 起始偏移 */
};

/* ---- 可组合 CRC 查询（compose_crc 算子） ---- */

#define RSYNC_CRC_QUERY_MAGIC   0x53594e4300000004ULL /* "SYNC" v4 */

struct crc_query_hdr {
    uint64_t magic;
    uint32_t nr_queries;
    uint32_t flags;
};

/* off 相对输入网格起点；off / len 均须 512B 对齐 */
struct crc_query {
    uint64_t off;
    uint64_t len;
};

#endif /* RSYNC_CSD_ABI_H */
