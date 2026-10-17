/*
 * read_csf.c —— 最小 CSF：在 CSD 内部读取一个文件并输出
 *
 * 运行于 indirect_main 流水线 (R -> C)，destination=0（输出留在 FDM）：
 *   Stage 0: CSD 按主机下发的 extent 列表，从 NVM 命名空间把文件数据搬入 mr_addr[1]
 *   Stage 1: 本算子执行 —— 打印首部预览（证明数据确实在设备侧），再把整份数据
 *            复制到 mr_addr[0]，供主机 pread("/mnt/fdm0/0") 校验
 *
 * 通道约定（与 compute.c 的 indirect 实现对齐）:
 *   mr_addr[1] / mr_len[1] : 输入，NVM 文件数据经 FTL 搬入的 FDM 缓冲
 *   mr_addr[0] / mr_len[0] : 输出，留在 FDM，主机可直接读取
 *   data_buffer            : 可选 uint64_t data_len，文件真实字节数
 *                            （0 或缺失时退化为 mr_len[1]）
 *
 * 返回值: 实际复制的字节数（destination=0 时不触发 Stage 2 回写 NVM）
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "cemu_def.h"

#define PREVIEW_BYTES 64

long long read_csf(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *in  = (const uint8_t *)args->mr_addr[1];
    uint8_t       *out = (uint8_t *)args->mr_addr[0];
    long long in_cap   = args->mr_len[1];
    long long out_cap  = args->mr_len[0];

    if (!in || !out || in_cap <= 0) {
        return -1;
    }

    /* 优先使用主机显式给出的长度，避免把 FDM 缓冲的尾部填充当成文件内容 */
    long long len = in_cap;
    if (args->data_buffer && args->buffer_len >= (long long)sizeof(uint64_t)) {
        uint64_t want = 0;
        memcpy(&want, args->data_buffer, sizeof(want));
        if (want > 0 && (long long)want < len) {
            len = (long long)want;
        }
    }
    if (len > out_cap) {
        len = out_cap;
    }
    if (len <= 0) {
        return 0;
    }

    /* 在 CSD 内部打印：这是"数据已被设备读到"的直接证据 */
    printf("[read_csf/CSD] 收到文件数据 %lld 字节, 首 %d 字节: ",
           len, (int)PREVIEW_BYTES);
    for (long long i = 0; i < PREVIEW_BYTES && i < len; i++) {
        printf("%02x", in[i]);
    }
    printf("\n");
    fflush(stdout);

    memcpy(out, in, (size_t)len);
    return len;
}
