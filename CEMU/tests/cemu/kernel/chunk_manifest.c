/*
 * chunk_manifest.c —— client 侧 CSF：对本地 f' 做 CDC 切分，产出"块清单"
 *
 * 清单条目是定长的 16 字节：
 *
 *     struct manifest_ent { uint64_t off;  uint32_t len;  uint32_t crc32c; };
 *
 *   1GB 文件按平均 ~8KB 切块 => 131072 条 => 2.097 MB，与设计的 2.1 MB/GB 一致。
 *
 * 关键点是**这条命令的三个声明量都是提交前可算的**：
 *   - 处理对象 : a  priori 已知的一段 f' 区间（窗口大小由主机定）
 *   - MRS 容量 : 输出缓冲按"每 4KB 一个块"的硬上界分配
 *                即 nr <= ceil(n / CHUNK_MIN_SIZE) + 1
 *   - 执行时长 : 正比于输入字节数 n，与内容无关
 * 命令内部没有依赖上一步结果的跳转：一次线性扫描出全部切点。
 *
 * CSF 接口（indirect pipeline，R -> C -> W）:
 *   mr[0] : 输出块清单数组
 *   mr[1] : Stage-0 读入的 f' 数据段
 *   cparam1 : 本段有效字节数 n
 *   cparam2 : 本段在 f' 内的起始偏移 base
 *   返回     : 写出的条目数 N；<0 失败（-ENOSPC = 输出缓冲不足）
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "cemu_manifest.h"  /* struct manifest_ent，client / server 两侧共享 */
#include "chunk_abi.h"      /* CHUNK_GEAR_MASK / CHUNK_MIN_SIZE，与主机 parasync 同源 */
#include "chunk_gear_v2.h"
#include "../../../hw/femu/backend/csd_fp_crc.h"

#define MANIFEST_ALIGN      512ULL
#define MANIFEST_MAX_CHUNK  65536ULL   /* 64KB 上界，同时是 512 的倍数 */

long long chunk_manifest(struct cemu_args *args)
{
    uint8_t *out;
    const uint8_t *data;
    uint64_t out_cap, data_cap, n, base, pos;
    uint64_t nr = 0;

    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return CHUNK_CSF_EINVAL;
    }

    out      = (uint8_t *)args->mr_addr[0];
    data     = (const uint8_t *)args->mr_addr[1];
    out_cap  = (uint64_t)args->mr_len[0];
    data_cap = (uint64_t)args->mr_len[1];
    n        = (uint64_t)args->cparam1;
    base     = (uint64_t)args->cparam2;

    if (!out || !data) {
        return CHUNK_CSF_EINVAL;
    }
    if (n == 0) {
        return 0;
    }
    if (n > data_cap || (n % MANIFEST_ALIGN) || (base % MANIFEST_ALIGN)) {
        return CHUNK_CSF_EINVAL;
    }

    pos = 0;
    while (pos < n) {
        uint64_t limit = pos + MANIFEST_MAX_CHUNK;
        uint64_t cut, len, fp = 0;

        if (limit > n) {
            limit = n;
        }

        /*
         * CDC：切点只在 512B 边界采样。
         * 这是与通用 CDC 唯一的语义差异 —— 切点多一层对齐约束，
         * 换来的是每个块都是整扇区，server 侧的 512B CRC 网格才能拼出它的校验和。
         */
        cut = limit;
        for (uint64_t i = pos; i < limit; i++) {
            uint64_t end;

            fp = (fp << 1) + chunk_gear_table[data[i]];
            end = i + 1;
            if ((end - pos) < CHUNK_MIN_SIZE) {
                continue;
            }
            if (end % MANIFEST_ALIGN) {
                continue;
            }
            if ((fp & CHUNK_GEAR_MASK) == 0) {
                cut = end;
                break;
            }
        }

        len = cut - pos;
        if (len == 0) {
            pos = cut;
            continue;
        }

        if ((nr + 1) * MANIFEST_ENT_BYTES > out_cap) {
            return CHUNK_CSF_ENOSPC;
        }

        {
            /* 条目从偏移 0 开始，无表头；未写到的尾部保持主机预置的 0 */
            struct manifest_ent *e =
                (struct manifest_ent *)(void *)(out + nr * MANIFEST_ENT_BYTES);
            e->off    = base + pos;
            e->len    = (uint32_t)len;
            e->crc32c = csd_crc32c(0, data + pos, len);
        }
        nr++;
        pos = cut;
    }

    return (long long)nr;
}
