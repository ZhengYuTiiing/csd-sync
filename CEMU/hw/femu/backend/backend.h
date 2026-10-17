#ifndef __FEMU_MEM_BACKEND
#define __FEMU_MEM_BACKEND

#include <stdint.h>
#include <stdbool.h>
#include "qemu/osdep.h"
#include "csd_fingerprint.h"

typedef struct QEMUSGList QEMUSGList;

typedef enum BackendType{
    FEMU_DRAM_BACKEND,
} BackendType;

/*
 * 页粒度常量 — 用于 OOB 固定分配
 * (ns 自己可能 secsz=512 secs_per_pg=8，但 backend 不依赖 ns 结构，
 *  这里把物理页统一按 4KB 处理，与 bbssd 的 ssd_write 粒度一致。)
 */
#define CSD_PAGE_SIZE   4096u

/* SSD Backend */
typedef struct SsdBackend {
    void        *logical_space;
    int64_t     size; /* in bytes */
    int         femu_mode;
    BackendType type;

    /* OOB 平行区 — 挑战二: 写时指纹固化 */
    uint8_t     *oob_space;         /* 大小 = tt_pgs * oob_sz_per_pg */
    uint32_t    oob_sz_per_pg;      /* 每页 OOB 字节数 (CSD_FP_LEN=32) */
    uint64_t    tt_pgs;             /* 后端总页数 (size / CSD_PAGE_SIZE) */
} SsdBackend;

int init_dram_backend(SsdBackend *b);
void free_dram_backend(SsdBackend *b);

int init_backend(SsdBackend **b, BackendType type, char *path, int64_t nbytes);
void free_backend(SsdBackend *b);

int backend_rw(SsdBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write);
void backend_rw_internal(SsdBackend *b, void *buf, uint64_t data_offset,
                           uint64_t data_size, int is_write);
void *backend_get_ptr(SsdBackend *b, uint64_t offset);
void backend_copy_internal(SsdBackend *b, uint64_t doff, uint64_t soff,
                           uint64_t data_size);
static inline void *backend_addr(SsdBackend *b, uint64_t offset)
{
    return b->logical_space + offset;
}
static inline void backend_fill(SsdBackend *b, uint64_t offset, uint64_t len)
{
    memset(backend_addr(b, offset), 0, len);
}

/*
 * OOB 访问 — pg_id 以 4KB 页为单位 (= byte_offset / CSD_PAGE_SIZE)
 */
static inline uint8_t *backend_oob_addr(SsdBackend *b, uint64_t pg_id)
{
    if (!b->oob_space || pg_id >= b->tt_pgs) {
        return NULL;
    }
    return b->oob_space + pg_id * b->oob_sz_per_pg;
}

/*
 * 写时 hook — 对 [data_offset, data_offset+data_size) 覆盖到的每一整页重算指纹
 * 并写回 OOB。半页写也触发整页重算（NAND 真实行为近似）。
 */
void backend_fingerprint_range(SsdBackend *b, uint64_t data_offset, uint64_t data_size);

/*
 * 把 [pg_start, pg_start+pg_count) 的 OOB 连续拷到 dst。
 * 返回成功拷贝的字节数 (= pg_count * oob_sz_per_pg)；越界返回 0。
 */
uint64_t backend_oob_gather(SsdBackend *b, uint64_t pg_start, uint64_t pg_count, void *dst);

#endif
