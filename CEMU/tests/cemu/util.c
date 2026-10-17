#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
#include <linux/fiemap.h>

#include "util.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// get file's mapping (LBA mapping)
int get_file_mappings(const char *filename, uint64_t *filesize, NvmeCopyFormat2 **cf2,
                      int parallel_chunks, int *nlb_per_chunk, int *nr_cf2_per_chunk)
{
	uint64_t max_nlb = 8192;

    int ret = -1;
    int fd = -1;
    struct fiemap *fiemap = NULL;
    uint64_t *chunk_end = NULL;

    if (!filename || !filesize || !cf2 || !nlb_per_chunk || !nr_cf2_per_chunk) {
        fprintf(stderr, "get_file_mappings: invalid argument(s)\n");
        return -1;
    }
    *cf2 = NULL;

    fd = open(filename, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    if (parallel_chunks <= 0) {
        fprintf(stderr, "get_file_mappings: parallel_chunks is less than 1, parallel_chunks: %d\n", parallel_chunks);
        goto out;
    }

    // get file stat (use fd to avoid TOCTOU on filename)
    struct stat st;
    if (fstat(fd, &st) < 0) {
        perror("fstat");
        goto out;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "get_file_mappings: not a regular file\n");
        goto out;
    }

    uint64_t total_size = (uint64_t)st.st_size;
    if (*filesize == 0 || *filesize > total_size) {
        *filesize = total_size;
    }
    /*
     * 不再要求 filesize 是 512B 的整数倍：总 LBA 数在下面向上取整，
     * 最后一个不满的扇区照样被描述符覆盖。
     * 原先这里直接拒绝 + 下面向下取整，两者叠加会让大小非 512B 倍数的文件
     * 尾部（最多 511B）彻底落不到任何描述符里，重建时静默丢字节。
     */

    // empty file: return 0 mappings, 0 chunks
    if (*filesize == 0) {
        for (int i = 0; i < parallel_chunks; i++) {
            if (nlb_per_chunk[0] == 0)
                nlb_per_chunk[i] = 0;
            nr_cf2_per_chunk[i] = 0;
        }
        ret = 0;
        goto out;
    }
    // 总 LBA 数 = 文件大小向上取整到整扇区
    uint64_t total_nlb = (*filesize + LBA_SIZE - 1) / LBA_SIZE;

    // fill nlb_per_chunk if caller didn't provide it; validate otherwise
    // 决定每个 chunk 分多少 LBA：
    //   - 若调用方没给 (nlb_per_chunk[0]==0)，按 total_nlb / parallel_chunks 均分，余数分给前几块
    //   - 若调用方给了，则校验和是否等于 total_nlb
    if (nlb_per_chunk[0] == 0) {
        uint64_t base = total_nlb / (uint64_t)parallel_chunks;
        uint64_t rem = total_nlb % (uint64_t)parallel_chunks;
        for (int i = 0; i < parallel_chunks; i++) {
            uint64_t v = base + ((uint64_t)i < rem ? 1 : 0);
            if (v > (uint64_t)INT_MAX) {
                fprintf(stderr, "get_file_mappings: nlb_per_chunk[%d] overflow: %lu\n", i, v);
                goto out;
            }
            nlb_per_chunk[i] = (int)v;
        }
    } else {
        uint64_t sum = 0;
        for (int i = 0; i < parallel_chunks; i++) {
            if (nlb_per_chunk[i] < 0) {
                fprintf(stderr, "get_file_mappings: nlb_per_chunk[%d] is negative: %d\n", i, nlb_per_chunk[i]);
                goto out;
            }
            sum += (uint64_t)nlb_per_chunk[i];
        }
        if (sum != total_nlb) {
            fprintf(stderr,
                    "get_file_mappings: sum(nlb_per_chunk) != filesize / LBA_SIZE, sum(nlb_per_chunk): %lu, filesize/LBA_SIZE: %lu\n",
                    sum, total_nlb);
            goto out;
        }
    }

    // build chunk end offsets (inclusive-exclusive) in nlb
    chunk_end = (uint64_t *)calloc((size_t)parallel_chunks, sizeof(uint64_t));
    if (!chunk_end) {
        perror("calloc");
        goto out;
    }
    uint64_t csum = 0;
    for (int i = 0; i < parallel_chunks; i++) {
        csum += (uint64_t)nlb_per_chunk[i];
        chunk_end[i] = csum;
        nr_cf2_per_chunk[i] = 0;
    }
    if (csum != total_nlb) {
        fprintf(stderr, "get_file_mappings: internal error, chunk sum %lu != total_nlb %lu\n", csum, total_nlb);
        goto out;
    }

    // get number of extents
    // 第一次调用 FIEMAP：只查询 extent 数量（fm_extent_count=0）
    // 关键：这是"主机解析文件→LBA 映射"的核心步骤，也是 NearSync 能下发
    //      轻量描述符的前提
    struct fiemap tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.fm_start = 0;
    tmp.fm_length = *filesize;
    tmp.fm_flags = FIEMAP_FLAG_SYNC;
    tmp.fm_extent_count = 0;
    if (ioctl(fd, FS_IOC_FIEMAP, &tmp) < 0) {
        perror("ioctl");
        goto out;
    }
    int nr_extents = tmp.fm_mapped_extents;
    if (nr_extents <= 0) {
        fprintf(stderr, "get_file_mappings: no mapped extents (nr_extents=%d)\n", nr_extents);
        goto out;
    }

    // get fiemap extents
    // 分配足够大的 fiemap 缓冲区，第二次调用真正获取 extent 细节
    fiemap = (struct fiemap *)malloc(sizeof(struct fiemap) + sizeof(struct fiemap_extent) * (nr_extents));
    if (!fiemap) {
        perror("malloc");
        goto out;
    }
    memset(fiemap, 0, sizeof(struct fiemap) + sizeof(struct fiemap_extent) * (nr_extents));
    fiemap->fm_start = 0;
    fiemap->fm_length = *filesize;
    fiemap->fm_flags = FIEMAP_FLAG_SYNC;
    fiemap->fm_extent_count = nr_extents;
    if (ioctl(fd, FS_IOC_FIEMAP, fiemap) < 0) {
        perror("ioctl");
        goto out;
    }

	// pass 1: calc nr_cf2, validate extents, and count entries per chunk
    int nr_cf2 = 0;
    uint64_t processed_nlb = 0;
    int cur_chunk = 0;
    int *tmp_cf2_per_chunk = (int *)calloc((size_t)parallel_chunks, sizeof(int));
    if (!tmp_cf2_per_chunk) {
        perror("calloc");
        goto out;
    }
    for (int i = 0; i < nr_extents && processed_nlb < total_nlb; i++) {
        struct fiemap_extent *extent = &fiemap->fm_extents[i];
        if (extent->fe_length % LBA_SIZE != 0) {
            fprintf(stderr, "get_file_mappings: extent length not LBA aligned: %llu\n",
                    (unsigned long long)extent->fe_length);
            free(tmp_cf2_per_chunk);
            goto out;
        }
        uint64_t remain_nlb = extent->fe_length / LBA_SIZE;
        while (remain_nlb > 0 && processed_nlb < total_nlb) {
            while (cur_chunk < parallel_chunks && processed_nlb >= chunk_end[cur_chunk])
                cur_chunk++;
            if (cur_chunk >= parallel_chunks) {
                fprintf(stderr, "get_file_mappings: ran out of chunks during pass1\n");
                free(tmp_cf2_per_chunk);
                goto out;
            }
            uint64_t chunk_left = chunk_end[cur_chunk] - processed_nlb;
            uint64_t nlb = MIN(remain_nlb, max_nlb);
            nlb = MIN(nlb, chunk_left);
            nlb = MIN(nlb, total_nlb - processed_nlb);
            if (nlb == 0) {
                fprintf(stderr, "get_file_mappings: internal error (nlb==0) during pass1\n");
                free(tmp_cf2_per_chunk);
                goto out;
            }
            nr_cf2++;
            tmp_cf2_per_chunk[cur_chunk]++;
            remain_nlb -= nlb;
            processed_nlb += nlb;
        }
    }
    if (processed_nlb != total_nlb) {
        fprintf(stderr, "get_file_mappings: extents do not cover requested size (processed=%lu, total=%lu)\n",
                processed_nlb, total_nlb);
        free(tmp_cf2_per_chunk);
        goto out;
    }
    // 按统计结果精确分配 cf2 数组
    *cf2 = (NvmeCopyFormat2 *)malloc(sizeof(NvmeCopyFormat2) * (size_t)nr_cf2);
    if (*cf2 == NULL) {
        perror("malloc");
        free(tmp_cf2_per_chunk);
        goto out;
    }

    // pass 2: fill cf2 and output per-chunk counts
    // ===== Pass 2: 填充 cf2 描述符 =====
    // 每条 cf2 描述一段连续物理区间 [slba, slba+nlb)，nlb 以 0 基编码
    for (int i = 0; i < parallel_chunks; i++)
        nr_cf2_per_chunk[i] = 0;

    int cur_cf = 0;
    processed_nlb = 0;
    cur_chunk = 0;
    for (int i = 0; i < nr_extents && processed_nlb < total_nlb; i++) {
        struct fiemap_extent *extent = &fiemap->fm_extents[i];
        uint64_t slba = extent->fe_physical / LBA_SIZE;
        uint64_t remain_nlb = extent->fe_length / LBA_SIZE;
        while (remain_nlb > 0 && processed_nlb < total_nlb) {
            while (cur_chunk < parallel_chunks && processed_nlb >= chunk_end[cur_chunk])
                cur_chunk++;
            if (cur_chunk >= parallel_chunks) {
                fprintf(stderr, "get_file_mappings: ran out of chunks during pass2\n");
                free(tmp_cf2_per_chunk);
                goto out;
            }
            uint64_t chunk_left = chunk_end[cur_chunk] - processed_nlb;
            uint64_t nlb = MIN(remain_nlb, max_nlb);
            nlb = MIN(nlb, chunk_left);
            nlb = MIN(nlb, total_nlb - processed_nlb);
            if (nlb == 0) {
                fprintf(stderr, "get_file_mappings: internal error (nlb==0) during pass2\n");
                free(tmp_cf2_per_chunk);
                goto out;
            }
            (*cf2)[cur_cf].slba = slba;
            (*cf2)[cur_cf].nlb = (uint16_t)(nlb - 1);
            (*cf2)[cur_cf].snsid = 1;
            cur_cf++;
            nr_cf2_per_chunk[cur_chunk]++;
            remain_nlb -= nlb;
            slba += nlb;
            processed_nlb += nlb;
        }
    }
    free(tmp_cf2_per_chunk);
    if (cur_cf != nr_cf2 || processed_nlb != total_nlb) {
        fprintf(stderr, "get_file_mappings: internal error, cur_cf=%d nr_cf2=%d processed=%lu total=%lu\n",
                cur_cf, nr_cf2, processed_nlb, total_nlb);
        goto out;
    }

    ret = nr_cf2;

out:
    if (fiemap)
        free(fiemap);
    if (chunk_end)
        free(chunk_end);
    if (fd >= 0)
        close(fd);
    if (ret < 0 && cf2 && *cf2) {
        free(*cf2);
        *cf2 = NULL;
    }
    return ret;
}

void prep_shared_library(const char *file, const char *kernel, struct ioctl_download *download)
{
    char *program = (char *)aligned_alloc(4096, 4096);
    sprintf(program, "%s", file);
    sprintf(program + strlen(file) + 1, "%s", kernel);
    download->addr = program;
    download->size = strlen(file) + strlen(kernel) + 2;
    download->ptype = PROGRAM_TYPE_SHARED_LIB;
}

void prep_ebpf(const char *file, const char *kernel, bool jit, struct ioctl_download *download)
{
    char *program = (char *)aligned_alloc(4096, 4096);
    sprintf(program, "%s", file);
    sprintf(program + strlen(file) + 1, "%s", kernel);
    download->addr = program;
    download->size = strlen(file) + strlen(kernel) + 2;
    download->ptype = PROGRAM_TYPE_EBPF;
    download->jit = jit;
}

void prep_mrs(struct ioctl_create_mrs *mrs, int nr_fd)
{
    mrs->nr_fd = nr_fd;
    mrs->fd = (int *)malloc(sizeof(int) * nr_fd);
    mrs->off = (long long *)calloc(nr_fd, sizeof(long long));
    mrs->size = (long long *)calloc(nr_fd, sizeof(long long));
    mrs->rsid = 0;
}

//构造一条 NVMe 直通（passthrough）命令
void prep_nvme_passthru_program_execute(struct nvme_passthru_cmd *cmd,
                                       uint64_t cparam1,
                                       uint64_t cparam2,
                                       uint16_t pind,
                                       uint16_t rsid,
                                       uint32_t numr,
                                       uint32_t group,
                                       uint32_t chunk_nlb,
                                       uint32_t user_runtime,
                                       void *data,
                                       uint32_t data_len)
{
    memset(cmd, 0, sizeof(*cmd));

    struct nvme_program_execute_cmd *exec =
        (struct nvme_program_execute_cmd *)cmd;

    exec->nsid = 3;
    exec->opcode = 0x01;
    exec->cparam1 = cparam1;
    exec->cparam2 = cparam2;
    exec->pind = pind;
    exec->rsid = rsid;
    exec->numr = numr;
    exec->group = group;
    exec->chunk_nlb = chunk_nlb;
    exec->user_runtime = user_runtime;

    if (data && data_len) {
        exec->dlen = data_len;
        cmd->addr = (uintptr_t)data;
        cmd->data_len = data_len;
    } else {
        exec->dlen = 0;
        cmd->addr = 0;
        cmd->data_len = 0;
    }
}

void prep_nvme_uring_program_execute(struct nvme_uring_cmd *cmd,
                                     uint64_t cparam1,
                                     uint64_t cparam2,
                                     uint16_t pind,
                                     uint16_t rsid,
                                     uint32_t numr,
                                     uint32_t group,
                                     uint32_t chunk_nlb,
                                     uint32_t user_runtime,
                                     void *data,
                                     uint32_t data_len)
{
    memset(cmd, 0, sizeof(*cmd));

    struct nvme_program_execute_cmd *exec =
        (struct nvme_program_execute_cmd *)cmd;

    exec->nsid = 3;
    exec->opcode = 0x01;
    exec->cparam1 = cparam1;
    exec->cparam2 = cparam2;
    exec->pind = pind;
    exec->rsid = rsid;
    exec->numr = numr;
    exec->group = group;
    exec->chunk_nlb = chunk_nlb;
    exec->user_runtime = user_runtime;

    if (data && data_len) {
        exec->dlen = data_len;
        cmd->addr = (uintptr_t)data;
        cmd->data_len = data_len;
    } else {
        exec->dlen = 0;
        cmd->addr = 0;
        cmd->data_len = 0;
    }
}

size_t calc_indirect_task_buffer_size(int parallel_chunks,
                                      int destination,
                                      int nr_in_cf,
                                      int nr_out_cf,
                                      size_t extra_len)
{
    size_t sz = sizeof(int) * 4; // header
    sz += sizeof(int) * parallel_chunks; // nr_in_cf_per_chunk
    if ((destination & 0xffff) == 1) {
        sz += sizeof(int) * parallel_chunks; // nr_out_cf_per_chunk
    }
    sz += sizeof(NvmeCopyFormat2) * nr_in_cf;
    if ((destination & 0xffff) == 1) {
        sz += sizeof(NvmeCopyFormat2) * nr_out_cf;
    }
    sz += extra_len;
    return sz;
}

size_t pack_indirect_task_buffer(void *dst,
                                 int parallel_chunks,
                                 int destination,
                                 int read_mode,
                                 int nr_in_cf,
                                 const int *nr_in_cf_per_chunk,
                                 const NvmeCopyFormat2 *in_cf,
                                 int nr_out_cf,
                                 const int *nr_out_cf_per_chunk,
                                 const NvmeCopyFormat2 *out_cf,
                                 const void *extra_payload,
                                 size_t extra_len)
{
    uint8_t *p = (uint8_t *)dst;
    int *hdr = (int *)p;
    int dest_lo = destination & 0xffff;

    hdr[0] = parallel_chunks;
    hdr[1] = ((read_mode & 0xffff) << 16) | dest_lo;
    hdr[2] = nr_in_cf;
    hdr[3] = (dest_lo == 1) ? nr_out_cf : 0;
    p += sizeof(int) * 4;

    if (nr_in_cf_per_chunk && parallel_chunks > 0) {
        memcpy(p, nr_in_cf_per_chunk, sizeof(int) * parallel_chunks);
    }
    p += sizeof(int) * parallel_chunks;

    if (dest_lo == 1) {
        if (nr_out_cf_per_chunk && parallel_chunks > 0) {
            memcpy(p, nr_out_cf_per_chunk, sizeof(int) * parallel_chunks);
        }
        p += sizeof(int) * parallel_chunks;
    }

    if (in_cf && nr_in_cf > 0) {
        memcpy(p, in_cf, sizeof(NvmeCopyFormat2) * nr_in_cf);
    }
    p += sizeof(NvmeCopyFormat2) * nr_in_cf;

    if (dest_lo == 1) {
        if (out_cf && nr_out_cf > 0) {
            memcpy(p, out_cf, sizeof(NvmeCopyFormat2) * nr_out_cf);
        }
        p += sizeof(NvmeCopyFormat2) * nr_out_cf;
    }

    if (extra_payload && extra_len > 0) {
        memcpy(p, extra_payload, extra_len);
        p += extra_len;
    }

    return (size_t)(p - (uint8_t *)dst);
}

size_t prep_oob_fingerprint_task_buffer(void *dst,
                                        int parallel_chunks,
                                        int nr_in_cf,
                                        const int *nr_in_cf_per_chunk,
                                        const NvmeCopyFormat2 *in_cf,
                                        const void *extra_payload,
                                        size_t extra_len)
{
    return pack_indirect_task_buffer(dst,
                                     parallel_chunks,
                                     /*destination=*/0,
                                     /*read_mode=*/CSD_READ_MODE_OOB,
                                     nr_in_cf,
                                     nr_in_cf_per_chunk,
                                     in_cf,
                                     /*nr_out_cf=*/0,
                                     NULL,
                                     NULL,
                                     extra_payload,
                                     extra_len);
}

