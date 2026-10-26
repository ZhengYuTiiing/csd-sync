/*
 * dsync_abi.h —— CSD dsync (变长 FastCDC 分块) 下沉算子族 ABI
 *
 * 挑战三: 双哈希两级流式匹配 (弱哈希初筛 -> 强哈希确认)
 */

#ifndef DSYNC_ABI_H
#define DSYNC_ABI_H

#include <stdint.h>

#define DSYNC_FP_LEN            32
#define DSYNC_TABLE_MAGIC       0x4453594e00000001ULL /* "DSYN" v1 */
#define DSYNC_MATCH_MAGIC       0x4453594e00000002ULL /* "DSYN" v2 */

/* 变长 chunk 描述符 */
struct dsync_chunk_desc {
    uint64_t offset;                    /* 文件内起始偏移 */
    uint32_t length;                    /* chunk 字节数 */
    uint32_t weak_hash;                 /* CRC32C 弱哈希 */
    uint8_t  strong_hash[DSYNC_FP_LEN]; /* BLAKE3 / 32B 强哈希 */
};

/* dsync chunk 描述表头 */
struct dsync_table_hdr {
    uint64_t magic;
    uint32_t nr_chunks;
    uint32_t entry_bytes;
    uint64_t total_data_len;
};

/* 匹配结果条目 */
struct dsync_match_token {
    uint64_t old_offset;                /* 老文件 chunk 偏移 */
    uint64_t new_offset;                /* 新文件 chunk 偏移 */
    uint32_t length;                    /* 匹配长度 */
    uint32_t reserved;
};

/* 匹配输出表头 */
struct dsync_match_hdr {
    uint64_t magic;
    uint32_t nr_matched;
    uint32_t nr_unmatched;
    uint64_t matched_bytes;
};

/*
 * grid-cdc 第 5 步：server 只对"弱哈希命中"的块算强校验和，回给 client。
 * 一条同时带 f 侧与 f' 侧偏移 —— 缺任何一侧 client 都无法生成 delta。
 */
struct dsync_strong_ent {
    uint64_t old_offset;                    /* f 内偏移（server 自己的块） */
    uint64_t new_offset;                    /* f' 内偏移（弱哈希命中的 client 块） */
    uint32_t length;
    uint32_t weak_hash;
    uint8_t  strong_hash[DSYNC_FP_LEN];     /* SHA1(20B) 填前 20 字节，其余置 0 */
};

/* client 侧"按区间补算强哈希"的查询表（随命令 extra 下发） */
#define DSYNC_QUERY_MAGIC 0x4453594e00000004ULL /* "DSYN" v4 */

struct dsync_query_hdr {
    uint64_t magic;
    uint64_t nr_queries;
};

struct dsync_query_ent {
    uint64_t off;                           /* f' 内绝对偏移 */
    uint64_t len;
    uint32_t idx;                           /* 回填到输出表的第 idx 条 */
    uint32_t reserved;
};

/*
 * grid_ds：单个算子承载 6 步流程里的三种角色，靠随命令下发的 extra 载荷区分：
 *   mode 1  无载荷            -> client CDC 切分 + 网格 weak（不算强哈希）
 *   mode 2  载荷 = gd_tab_hdr -> server CDC 切分 + 网格 weak + 与清单比对
 *                                + 只对弱哈希命中的块算 SHA1
 *   mode 3  载荷 = gd_query_hdr -> client 按区间补算强哈希
 */
#define GD_TAB_MAGIC 0x4752445354414231ULL   /* "GRDSTAB1" */
#define GD_QRY_MAGIC 0x4752445351525931ULL   /* "GRDSQRY1" */

/*
 * build_id：主机把它回读打印出来。设备侧 dlopen 过的 .so 不会重载，
 * 换实现必须换符号名/文件名并重启 QEMU；主机靠这个号一眼看出
 * 设备里跑的到底是哪一份 .so，不用靠块数去猜。
 */
#define GD_BUILD_ID  3u

struct gd_tab_hdr {
    uint64_t magic;
    uint32_t nr;                            /* 条目数 */
    uint32_t entry_bytes;                   /* 48 (gd_ent) 或 56 (gd_pair) */
    uint64_t total_len;                     /* 本窗口字节数 */
    uint32_t build_id;
    uint32_t reserved;
};

/* mode 1 / mode 3 的输出条目 */
struct gd_ent {
    uint64_t off;
    uint32_t len;
    uint32_t weak;
    uint8_t  strong[DSYNC_FP_LEN];          /* SHA1 占前 20 字节，其余置 0 */
};

/* mode 2 的输出条目：f 与 f' 两侧偏移都要带，缺一侧整个流程就断 */
struct gd_pair {
    uint64_t old_off;                       /* f 内偏移（server 自己的块） */
    uint64_t new_off;                       /* f' 内偏移（弱命中的 client 块） */
    uint32_t len;
    uint32_t weak;
    uint8_t  strong[DSYNC_FP_LEN];
};

struct gd_query_hdr {
    uint64_t magic;
    uint64_t nr;
};

/*
 * grid_pipe（流水线版）用的 per-chunk 描述符。
 *
 * 为什么需要它：indirect 流水线里一条命令并发跑 P 个 chunk，
 * 但 cparam1/cparam2 是**命令级**的（P 个 chunk 共用），
 * 而 struct cemu_args 里没有 chunk_id —— CSF 无法自知自己是第几个 chunk。
 * 于是把「本 chunk 在文件内的起始偏移」放进 per-chunk 的 mr[2]，
 * 每个 chunk 的 MR 切片指向不同的 FDM 文件区间，CSF 读到的就是自己的基址。
 *
 * 只有 mode 3（按区间补算强哈希）需要它：查询表给的是**绝对偏移**，
 * 不知道基址就无法判断哪些查询落在自己这段里。
 * mode 1/2 不需要：它们输出 chunk 内相对偏移，由主机侧按基址回填。
 */
#define GD_DESC_MAGIC 0x4752445344534331ULL   /* "GRDSDSC1" */

struct gd_desc {
    uint64_t magic;
    uint64_t base;          /* 本 chunk 在文件内的起始字节偏移（4KB 对齐） */
    uint64_t nbytes;        /* 本 chunk 字节数 */
};

struct gd_query {
    uint64_t off;
    uint64_t len;
    uint32_t idx;                           /* 回填到第 idx 条 */
    uint32_t reserved;
};

#endif /* DSYNC_ABI_H */
