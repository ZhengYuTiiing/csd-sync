/*
 * collect_oob.c —— 挑战二/三基础 CSF: 浅读 OOB 指纹收集算子
 *
 * 运行于 indirect_main 流水线 (R -> C -> W):
 *   Stage 0: 浅读 (read_mode=1) 从 NAND OOB 将连续的 32B 指纹流灌入 mr_addr[1]
 *   Stage 1: 本算子执行。根据需要做校验、透传或按步长抽样至 mr_addr[0]
 *   Stage 2: 若 destination=1 (回写 NVM)，写回 mr_addr[0]；若 destination=0 (FDM)，返回 0 保留
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"

#define CSD_FP_LEN 32

long long collect_oob(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *in_oob = (const uint8_t *)args->mr_addr[1];
    uint8_t *out_buf      = (uint8_t *)args->mr_addr[0];
    long long in_len      = args->mr_len[1];
    long long out_cap     = args->mr_len[0];

    if (!in_oob) {
        return -1;
    }

    /* cparam1: 期望搬运的指纹个数（0 表示按 in_len / 32 全量搬运） */
    long long nr_fps = args->cparam1;
    if (nr_fps == 0) {
        nr_fps = in_len / CSD_FP_LEN;
    }

    long long bytes_to_copy = nr_fps * CSD_FP_LEN;
    if (bytes_to_copy > in_len) {
        bytes_to_copy = in_len;
    }
    if (out_buf && bytes_to_copy > out_cap) {
        bytes_to_copy = out_cap;
    }

    if (out_buf && bytes_to_copy > 0) {
        memcpy(out_buf, in_oob, (size_t)bytes_to_copy);
    }

    /*
     * 返回值语义：
     * 对于 destination=1 的 indirect task，返回写回 NVM 的 512B 扇区数；
     * 对于 destination=0 (留在 FDM)，返回 0，间接流水线不会触发 Stage 2 写。
     */
    long long blocks = (bytes_to_copy + 511) / 512;
    return blocks;
}
