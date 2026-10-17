/*
 * rsync_patch.c —— 挑战三 CSF: rsync 下沉流式 Patch 算子
 *
 * 运行于 indirect_main 流水线:
 *   Stage 0: 从 NVM 流水读入老文件数据块至 mr_addr[1] (4KB 粒度)
 *   Stage 1: 本算子执行。按 delta 指令将老块或新 literal 拼装至 mr_addr[0]
 *   Stage 2: 将 mr_addr[0] 中的重构数据直写 NVM 新文件
 *
 * 省去将老块在网络/PCIe 上往返搬移的 RTT 消耗。
 */

#include <stdint.h>
#include <string.h>
#include "cemu_def.h"
#include "rsync_csd_abi.h"

long long rsync_patch(struct cemu_args *args)
{
    if (!args || args->numr < 2 || !args->mr_addr || !args->mr_len) {
        return -1;
    }

    const uint8_t *old_data_stream = (const uint8_t *)args->mr_addr[1];
    uint8_t *out_stream            = (uint8_t *)args->mr_addr[0];
    long long in_cap               = args->mr_len[1];
    long long out_cap              = args->mr_len[0];
    const uint8_t *data_buf        = (const uint8_t *)args->data_buffer;
    long long data_len             = args->buffer_len;

    if (!old_data_stream || !out_stream || !data_buf) {
        return -1;
    }
    if (data_len < (long long)sizeof(struct rsync_delta_hdr)) {
        return -1;
    }

    const struct rsync_delta_hdr *dhdr = (const struct rsync_delta_hdr *)data_buf;
    if (dhdr->magic != RSYNC_DELTA_MAGIC) {
        return -1;
    }

    uint32_t blk_sz = dhdr->blk_size ? dhdr->blk_size : RSYNC_CSD_BLOCK_SIZE;
    uint32_t nr_cmds = dhdr->nr_cmds;

    const struct rsync_delta_cmd *cmds =
        (const struct rsync_delta_cmd *)(data_buf + sizeof(*dhdr));
    const uint8_t *lit_pool = data_buf + dhdr->data_pool_offset;

    uint32_t max_out_blk = 0;

    for (uint32_t i = 0; i < nr_cmds; i++) {
        const struct rsync_delta_cmd *cmd = &cmds[i];
        uint64_t out_off = (uint64_t)cmd->blk_idx * blk_sz;
        if (out_off + blk_sz > (uint64_t)out_cap) {
            return -1;
        }

        if (cmd->cmd_type == RSYNC_CMD_COPY_OLD) {
            uint64_t in_off = (uint64_t)cmd->old_blk_idx * blk_sz;
            if (in_off + blk_sz > (uint64_t)in_cap) {
                return -1;
            }
            memcpy(out_stream + out_off, old_data_stream + in_off, blk_sz);
        } else if (cmd->cmd_type == RSYNC_CMD_INSERT_LIT) {
            if ((long long)(dhdr->data_pool_offset + cmd->data_offset + blk_sz) > data_len) {
                return -1;
            }
            memcpy(out_stream + out_off, lit_pool + cmd->data_offset, blk_sz);
        } else {
            return -1;
        }

        if (cmd->blk_idx + 1 > max_out_blk) {
            max_out_blk = cmd->blk_idx + 1;
        }
    }

    /* 返回写回 NVM 的 512B 扇区数 (Stage 2 消费) */
    uint64_t total_out_bytes = (uint64_t)max_out_blk * blk_sz;
    return (long long)(total_out_bytes / 512);
}
