/*
 * grid_rb.c —— 方案 D：重建下沉到 CSD 的设备侧算子
 *
 * 运行于 indirect_main 流水线（R -> C -> W），主机侧以 destination=1 下发，
 * 因此本算子返回后，stage 2 会把 mr[0] 直接写回闪存的目标 LBA 区间，
 * 数据字节全程不进入主机 CPU 的执行路径与内核存储栈。
 *
 * MR 约定（与其它算子一致）：
 *   mr[0]  输出缓冲            —— stage 2 从这里写回闪存；大小 = 本次写回字节数
 *   mr[1]  输入缓冲            —— stage 0 把源文件的这一段读进来
 *   mr[2]  per-chunk 配方表    —— mode 1/2 用；告诉 CSF 本 chunk 该往哪插字面量
 *   mr[3]  per-chunk 字面量    —— mode 1/2 用；来自 client 经网络送到的字节
 *
 * cparam1：本次读入的字节数（命令级，P 个 chunk 共用）。
 * cparam2：mode。
 *
 * 返回值：要写回的 512B 扇区数（vadd.c:38 的约定）。返回 0 则 stage 2 不触发。
 *
 * mode 0：纯复制。输入输出等长，用于先验证 destination=1 通路。
 * mode 1：带字面量的重建（预留）。
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "cemu_def.h"

#define SEC 512ull

/* mode 1 的配方条目：把 src 的一段搬到 dst 的一段；kind=1 时改从字面量区取 */
struct rb_ent {
    uint64_t dst_off;   /* 输出文件内字节偏移，相对本 chunk 的输出基址 */
    uint64_t src_off;   /* 输入文件内字节偏移，相对本 chunk 的输入基址；kind=1 时是字面量区偏移 */
    uint64_t len;
    uint32_t kind;      /* 0=复用源文件  1=字面量 */
    uint32_t rsvd;
};

struct rb_recipe {
    uint64_t magic;
    uint64_t nr;
    uint64_t out_base;  /* 本 chunk 输出基址（字节，文件内绝对） */
    uint64_t in_base;   /* 本 chunk 输入基址（字节，文件内绝对） */
};

#define RB_MAGIC 0x4752445344524231ull   /* "GRDSDRB1" */

long long grid_rb(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    uint8_t *out = (uint8_t *)args->mr_addr[0];
    const long long out_cap = args->mr_len[0];
    const uint8_t *in = (const uint8_t *)args->mr_addr[1];
    const long long in_cap = args->mr_len[1];

    /* 本次实际读入长度由 cparam1 给出；mr_len 只是容量 */
    long long n = (long long)args->cparam1;
    if (n <= 0 || n > in_cap) {
        n = in_cap;
    }

    const long long mode = (long long)args->cparam2;

    if (mode == 0) {
        /* 纯复制：输入输出等长 */
        if (n > out_cap) {
            n = out_cap;
        }
        if (n <= 0 || !out || !in) {
            return 0;
        }
        memcpy(out, in, (size_t)n);
        return (n + (long long)SEC - 1) / (long long)SEC;
    }

    /* ---------------- mode 1：按配方拼装，字面量并入 ---------------- */
    if (args->numr < 4 || !args->mr_addr[2] || !args->mr_addr[3]) {
        return -2;
    }
    const struct rb_recipe *rc = (const struct rb_recipe *)args->mr_addr[2];
    const long long rc_cap = args->mr_len[2];
    const uint8_t *lit = (const uint8_t *)args->mr_addr[3];
    const long long lit_cap = args->mr_len[3];

    if (!rc || rc_cap < (long long)sizeof(*rc) || rc->magic != RB_MAGIC) {
        return -3;
    }
    if (rc->nr > 4096) {
        return -4;
    }
    const struct rb_ent *ent = (const struct rb_ent *)(rc + 1);
    if ((long long)(sizeof(*rc) + rc->nr * sizeof(*ent)) > rc_cap) {
        return -5;
    }

    /*
     * 本 chunk 的输出基址基准：stage 0 把源文件 [in_base, in_base+n) 读进 mr[1]，
     * stage 2 从 mr[0] 起始处写回本 chunk 输出区间的第一个扇区。
     * 所以输出字节偏移必须从 out_base 起算。
     */
    long long out_len = 0;

    for (uint64_t i = 0; i < rc->nr; i++) {
        const uint64_t d = ent[i].dst_off;
        const uint64_t l = ent[i].len;
        if (l == 0) {
            continue;
        }
        if (d + l > (uint64_t)out_cap) {
            return -6;
        }

        const uint8_t *src;
        if (ent[i].kind == 0) {
            if (ent[i].src_off + l > (uint64_t)n) {
                return -7;
            }
            src = in + ent[i].src_off;
        } else {
            if (ent[i].src_off + l > (uint64_t)lit_cap) {
                return -8;
            }
            src = lit + ent[i].src_off;
        }
        memcpy(out + d, src, (size_t)l);
        if ((long long)(d + l) > out_len) {
            out_len = (long long)(d + l);
        }
    }

    if (out_len <= 0) {
        return 0;
    }
    return (out_len + (long long)SEC - 1) / (long long)SEC;
}
