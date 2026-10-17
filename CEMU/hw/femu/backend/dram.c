#include "../nvme.h"
#include <stdlib.h>

/* Coperd: FEMU Memory Backend (mbe) for emulated SSD */

int init_dram_backend(SsdBackend *b)
{
    // b->logical_space = mmap(NULL, b->size, PROT_READ | PROT_WRITE,
    //                         MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE | MAP_HUGETLB | MAP_POPULATE, -1, 0);
    b->logical_space = aligned_alloc(4096, b->size);
    if (b->logical_space == NULL || b->logical_space == (void *)-1) {
        femu_err("Failed to allocate memory for ssd backend!\n");
        abort();
    }

    // if (mlock(b->logical_space, b->size) == -1) {
    //     femu_err("Failed to pin the memory backend to the host DRAM\n");
    // }

    /*
     * 挑战二: 分配 OOB 平行区
     * 每个 4KB 物理页配一个 CSD_FP_LEN(32B) 的指纹槽。
     * tt_pgs = size / 4096 向下取整；对齐在页边界的 size 恰好填满。
     */
    b->oob_sz_per_pg = CSD_FP_LEN;
    b->tt_pgs = (uint64_t)b->size / CSD_PAGE_SIZE;
    if (b->tt_pgs > 0) {
        size_t oob_bytes = (size_t)b->tt_pgs * b->oob_sz_per_pg;
        b->oob_space = (uint8_t *)aligned_alloc(4096, (oob_bytes + 4095) & ~4095ULL);
        if (!b->oob_space) {
            femu_err("Failed to allocate OOB region for ssd backend!\n");
            abort();
        }
        memset(b->oob_space, 0, oob_bytes);
    } else {
        b->oob_space = NULL;
    }

    return 0;
}

void free_dram_backend(SsdBackend *b)
{
    if (b->logical_space) {
        // munmap(b->logical_space, b->size);
        free(b->logical_space);
        // munlock(b->logical_space, b->size);
    }
    if (b->oob_space) {
        free(b->oob_space);
        b->oob_space = NULL;
    }
}
