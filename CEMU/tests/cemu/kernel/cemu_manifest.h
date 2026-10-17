/*
 * cemu_manifest.h —— client / server 两侧共享的块清单 ABI
 *
 * 清单是自描述的定长条目数组（没有表头），因为它的条数 N 在命令提交时就
 * 作为参数给出，不需要设备侧解析。
 *
 * 条目 16 字节：
 *   1GB 文件按平均 ~8KB 切块 => 131072 条 => 2.097 MB/GB
 */
#ifndef CEMU_MANIFEST_H
#define CEMU_MANIFEST_H

#include <stdint.h>

struct manifest_ent {
    uint64_t off;      /* 文件内绝对偏移（512B 对齐） */
    uint32_t len;      /* 块长（512B 的整数倍） */
    uint32_t crc32c;   /* CRC32C（Castagnoli，多项式 0x82F63B78） */
};

#define MANIFEST_ENT_BYTES ((uint64_t)sizeof(struct manifest_ent))  /* 16 */

/*
 * 匹配表条目：server 折叠后输出的"命中记录"，16 字节。
 *
 * 之所以直接用 tgt_off 而不是 tgt 扇区号，是为了让主机侧重建时能直接拿去
 * copy_file_range，不必再做扇区<->字节的换算。
 *
 * src_off 不存：它恒等于 tgt_off - deltas[delta_idx]，主机侧有候选偏移数组，
 * 自己减一下即可。省 8 字节/条。
 *
 * 未命中的块不进表 —— 主机按"匹配表覆盖区间的补集"得到字面量区间。
 */
struct match_ent {
    uint64_t tgt_off;      /* f' 内偏移（512B 对齐） */
    uint32_t len;          /* 长度（512B 整数倍） */
    uint32_t delta_idx;    /* 命中的候选偏移下标 */
};

#define MATCH_ENT_BYTES ((uint64_t)sizeof(struct match_ent))  /* 16 */

/*
 * 折叠参数块：随候选偏移一起交给设备。
 *
 * 候选偏移解决"插入/删除导致整体位移"的问题 —— 见 fold_manifest.c 的说明。
 * 做成参数块而不是塞 cparam，是因为 cparam 只有 2 个且已被 N 占用，
 * 后续加强校验也不用再改接口。
 */
struct fold_param {
    uint64_t old_nlb;      /* f 的扇区数，用于 src 区间边界检查 */
    uint64_t nr_deltas;    /* 候选偏移个数 */
    int64_t  deltas[];     /* 候选偏移（字节）；f 内位置 = f' 内位置 - delta */
};

#define FOLD_PARAM_BASE_BYTES ((uint64_t)sizeof(struct fold_param))  /* 16 */

/*
 * 注意：CSF 是不会重新装载的。prep_shared_library 只把 "<路径>\0<符号>\0"
 * 交给设备，由 QEMU 进程自己 dlopen；进程内已 dlopen 过的 .so 不会重新加载，
 * 所以改了 CSF 实现之后必须重启 QEMU 才生效（UNLOAD/DOWNLOAD 都救不了）。
 * 因此这个 ABI 尽量保持简单、稳定，新需求优先能在主机侧解决。
 */

#endif /* CEMU_MANIFEST_H */
