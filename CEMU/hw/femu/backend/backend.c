#include "./backend.h"
#include "./csd_fp_crc.h"
#include "../nvme.h"

int init_backend(SsdBackend **mbe, BackendType type, char *path, int64_t nbytes)
{
    SsdBackend *b = *mbe = g_malloc0(sizeof(SsdBackend));
    if (b == NULL) {
        femu_err("Failed to allocate memory for ssd backend!\n");
        return -1;
    }

    b->type = type;
    b->size = nbytes;

    switch(b->type) {
    case FEMU_DRAM_BACKEND:
        return init_dram_backend(b);
    default:
        femu_err("Unknown backend type!\n");
        abort();
        return -1;
    }
}

void free_backend(SsdBackend *b)
{
    switch(b->type) {
    case FEMU_DRAM_BACKEND:
        free_dram_backend(b);
        break;
    default:
        femu_err("Unknown backend type!\n");
    }
    g_free(b);
}

int backend_rw(SsdBackend *b, QEMUSGList *qsg, uint64_t *lbal, bool is_write)
{
    int sg_cur_index = 0;
    dma_addr_t sg_cur_byte = 0;
    dma_addr_t cur_addr, cur_len;
    uint64_t mb_oft = lbal[0];
    void *mb = b->logical_space;

    DMADirection dir = DMA_DIRECTION_FROM_DEVICE;

    if (is_write) {
        dir = DMA_DIRECTION_TO_DEVICE;
    }

    while (sg_cur_index < qsg->nsg) {
        cur_addr = qsg->sg[sg_cur_index].base + sg_cur_byte;
        cur_len = qsg->sg[sg_cur_index].len - sg_cur_byte;
        if (dma_memory_rw(qsg->as, cur_addr, mb + mb_oft, cur_len, dir, MEMTXATTRS_UNSPECIFIED)) {
            femu_err("dma_memory_rw error\n");
        }

        sg_cur_byte += cur_len;
        if (sg_cur_byte == qsg->sg[sg_cur_index].len) {
            sg_cur_byte = 0;
            ++sg_cur_index;
        }

        if (b->femu_mode == FEMU_OCSSD_MODE) {
            mb_oft = lbal[sg_cur_index];
        } else if (b->femu_mode == FEMU_BBSSD_MODE ||
                   b->femu_mode == FEMU_NOSSD_MODE ||
                   b->femu_mode == FEMU_ZNSSD_MODE ||
                   b->femu_mode == FEMU_CSD_MODE) {
            mb_oft += cur_len;
        } else {
            assert(0);
        }
    }

    qemu_sglist_destroy(qsg);

    return 0;
}

void backend_rw_internal(SsdBackend *b, void *buf, uint64_t data_offset,
                         uint64_t data_size, int is_write)
{
    const uint64_t copy_threshold = 16UL * 1024UL;
    const uint64_t page_size = 4096;
    const uint64_t page_mask = page_size - 1;

    void *src, *dest;
    if (is_write) {
        src = buf;
        dest = b->logical_space + data_offset;
    } else {
        src = b->logical_space + data_offset;
        dest = buf;
    }
    if (data_size <= copy_threshold) {
        memmove(dest, src, data_size);
    } else {
        uint64_t remain = data_size;
        if (data_offset & page_mask) {
            uint64_t sz = page_size - (data_offset & page_mask);
            memmove(dest, src, sz);
            remain -= sz;
            dest += sz;
            src += sz;
        }
        while (remain > 0) {
            uint64_t sz = remain > copy_threshold ? copy_threshold : remain;
            memmove(dest, src, sz);
            remain -= sz;
            dest += sz;
            src += sz;
        }
    }
}

void *backend_get_ptr(SsdBackend *b, uint64_t offset)
{
    return b->logical_space + offset;
}

void backend_copy_internal(SsdBackend *b, uint64_t doff, uint64_t soff,
                           uint64_t data_size)
{
    void *dest = b->logical_space + doff;
    backend_rw_internal(b, dest, soff, data_size, 0);
}

/*
 * 挑战二 — 写时指纹固化
 *
 * 对 [data_offset, data_offset+data_size) 覆盖到的每个 4KB 物理页，
 * 重新读取整页内容算 csd_fp256，写回 OOB。
 * 半页写也触发整页 fingerprint（真实 NAND 页粒度写行为近似）。
 *
 * 调用点：所有真实落盘的数据路径都应该在写完 backend 之后 call 一次：
 *   - nvme-io.c:nvme_rw() 的 backend_rw() 之后
 *   - bbssd/ftl.c:memory_copy_nvm() 的 backend_rw_internal() 之后（is_write 分支）
 */
void backend_fingerprint_range(SsdBackend *b, uint64_t data_offset, uint64_t data_size)
{
    if (!b || !b->oob_space || data_size == 0) {
        return;
    }
    uint64_t pg_start = data_offset / CSD_PAGE_SIZE;
    uint64_t pg_end   = (data_offset + data_size - 1) / CSD_PAGE_SIZE;
    if (pg_end >= b->tt_pgs) {
        pg_end = b->tt_pgs - 1;
    }
    for (uint64_t pg = pg_start; pg <= pg_end; pg++) {
        uint8_t *page = (uint8_t *)b->logical_space + pg * CSD_PAGE_SIZE;
        uint8_t *oob  = b->oob_space + pg * b->oob_sz_per_pg;

        /*
         * 挑战二：不再写不可组合的 32B 强摘要，而是写 8 个扇区级 CRC32C。
         * 每个槽位仍是 32B，但任意 512B 对齐区间的校验和都可以由槽位拼接得到
         * （crc(A||B) = combine(crc(A), crc(B), len(B))），
         * 这样定长槽位也能服务内容定义分块的任意边界。
         */
        csd_crc_slot_build(page, oob);
    }
}

/*
 * 挑战二 / 挑战三的桥 — 浅读 OOB
 *
 * 把 [pg_start, pg_start+pg_count) 的 OOB 一口气拷到 dst。
 * 典型调用方：bbssd/ftl.c:memory_copy_oob（stage=0 的浅读分支）。
 */
uint64_t backend_oob_gather(SsdBackend *b, uint64_t pg_start, uint64_t pg_count, void *dst)
{
    if (!b || !b->oob_space || pg_count == 0) {
        return 0;
    }
    if (pg_start + pg_count > b->tt_pgs) {
        return 0;
    }
    uint64_t bytes = (uint64_t)pg_count * b->oob_sz_per_pg;
    memcpy(dst, b->oob_space + pg_start * b->oob_sz_per_pg, bytes);
    return bytes;
}