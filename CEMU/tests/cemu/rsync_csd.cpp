/*
 * rsync_csd.cpp —— rsync 的 CSD 版（"现场算校验和"语义整体下沉）
 *
 * 与 sync_file.cpp（skysync-f 的 CSD 版）共享同一套三阶段流程：
 *   Phase 1  rsync_match_data : 设备读入老文件**数据**（CSD_READ_MODE_DATA），
 *                               在设备内逐 4KB 页**现场算** csd_fp256 指纹，
 *                               与主机下发的新文件指纹表比对，产出命中表。
 *                               （对照：sync_file 的 rsync_match 读的是 OOB
 *                                现成指纹，不读数据——这正是两者的语义分界）
 *   Phase 2  (主机)           : 依据命中表生成 delta 指令（COPY_OLD / INSERT_LIT）
 *   Phase 3  rsync_patch      : 设备内重构新文件，直接写回 NVM
 *
 * 单 CSD：old / new / target 都在一个设备上（默认 /dev/nvme0）。
 * 老文件按 32MB 窗口分批读入（数据模式不再像 OOB 那样恒定 KB 级），
 * 每窗口一条匹配命令，指纹全表随命令 extra 下发。
 *
 * 前置条件：old 与 new 大小均为 4KB 的整数倍；
 *           /mnt/fdm0/{0,1,2,3} 容量足够（见 vm_setup.sh）。
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <liburing.h>

#include "util.h"
#include "kernel/rsync_csd_abi.h"

/* 与设备侧同源的指纹实现，避免两份代码漂移导致永不匹配 */
#include "../../hw/femu/backend/csd_fingerprint.h"
#include "../../hw/femu/backend/csd_sha1.h"

#define CSD_CTL_DEV "/dev/nvme0c3"
#define CSD_NG_DEV  "/dev/ng0n3"
#define FDM_MATCH_OUT "/mnt/fdm0/0"
#define FDM_DATA_IN   "/mnt/fdm0/1"
#define FDM_OLD_DATA  "/mnt/fdm0/2"
#define FDM_PATCH_OUT "/mnt/fdm0/3"

#define IOU_QUEUE_DEPTH 64
#define BLK            RSYNC_CSD_BLOCK_SIZE
#define FP_LEN         RSYNC_CSD_FP_LEN

static const char *DEF_OLD = "/mnt/nvme0/sync/old.bin";
static const char *DEF_NEW = "/mnt/nvme0/sync/new.bin";
static const char *DEF_TGT = "/mnt/nvme0/sync/target.bin";

static inline uint64_t round_up_4k(uint64_t v)
{
    return (v + 4095) & ~4095ULL;
}

static int submit_execute_cmd(int ng_fd, const struct nvme_uring_cmd *cmd,
                              long long *out_result)
{
    struct io_uring ring;
    if (io_uring_queue_init(IOU_QUEUE_DEPTH, &ring,
                            IORING_SETUP_SQE128 | IORING_SETUP_CQE32) < 0) {
        return -1;
    }

    struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
    if (!sqe) {
        io_uring_queue_exit(&ring);
        return -1;
    }
    memset(sqe, 0, sizeof(*sqe));
    sqe->opcode = IORING_OP_URING_CMD;
    sqe->cmd_op = NVME_URING_CMD_IO;
    sqe->fd     = ng_fd;
    memcpy(sqe->cmd, cmd, sizeof(*cmd));

    if (io_uring_submit(&ring) < 0) {
        io_uring_queue_exit(&ring);
        return -1;
    }

    struct io_uring_cqe *cqe = nullptr;
    if (io_uring_wait_cqe(&ring, &cqe) < 0 || !cqe) {
        io_uring_queue_exit(&ring);
        return -1;
    }

    int ret = cqe->res;
    if (out_result && ret >= 0) {
        *out_result = (long long)cqe->big_cqe[0];
    }
    io_uring_cqe_seen(&ring, cqe);
    io_uring_queue_exit(&ring);

    return ret < 0 ? ret : 0;
}

static int load_csf(int ctl_fd, const char *so, const char *fn,
                    const char *name, uint16_t *pind)
{
    struct ioctl_download dl {};
    dl.name = (char *)name;
    dl.indirect = 1;
    prep_shared_library(so, fn, &dl);

    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);

    errno = 0;
    int ret = ioctl(ctl_fd, IOCTL_CEMU_DOWNLOAD, &dl);
    if (ret <= 0 && errno != EEXIST) {
        fprintf(stderr, "DOWNLOAD %s 失败: %s\n", name, strerror(errno));
        return -1;
    }

    if (ioctl(ctl_fd, IOCTL_CEMU_ACTIVATE, &dl) < 0) {
        fprintf(stderr, "ACTIVATE %s 失败: %s\n", name, strerror(errno));
        return -1;
    }
    *pind = dl.pind;
    return 0;
}

/*
 * 提交一条 indirect 计算命令。
 * chunk_nlb 决定输入侧（老文件）读多少 LBA；输出侧长度由算子返回值决定。
 */
static int run_indirect(int ng_fd, uint16_t pind, uint16_t rsid,
                        uint64_t cparam1, uint64_t cparam2,
                        int destination, int read_mode,
                        const NvmeCopyFormat2 *in_cf, int nr_in,
                        const NvmeCopyFormat2 *out_cf, int nr_out,
                        const void *extra, size_t extra_len,
                        uint32_t chunk_nlb, long long *csf_ret)
{
    std::vector<int> in_pc(1, nr_in);
    std::vector<int> out_pc(1, nr_out);

    const size_t need = calc_indirect_task_buffer_size(
        1, destination, nr_in, nr_out, extra_len);
    std::vector<uint8_t> task(need, 0);

    size_t packed = pack_indirect_task_buffer(
        task.data(), 1, destination, read_mode,
        nr_in, in_pc.data(), in_cf,
        nr_out, out_pc.data(), out_cf,
        extra, extra_len);

    if (packed != need) {
        fprintf(stderr, "打包长度 %zu != 预算 %zu\n", packed, need);
        return -1;
    }

    struct nvme_uring_cmd cmd {};
    prep_nvme_uring_program_execute(&cmd, cparam1, cparam2, pind, rsid,
                                    (uint32_t)nr_in, 0, chunk_nlb, 0,
                                    task.data(), (uint32_t)packed);
    return submit_execute_cmd(ng_fd, &cmd, csf_ret);
}

static bool read_full(int fd, void *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t r = read(fd, (uint8_t *)buf + got, len - got);
        if (r <= 0) {
            return false;
        }
        got += (size_t)r;
    }
    return true;
}

/* 从指定偏移写满 len 字节（主机侧重建用） */
static bool write_full_at(int fd, const void *buf, size_t len, off_t off)
{
    size_t done = 0;
    while (done < len) {
        ssize_t r = pwrite(fd, (const uint8_t *)buf + done, len - done,
                           (off_t)(off + (off_t)done));
        if (r <= 0) {
            return false;
        }
        done += (size_t)r;
    }
    return true;
}

static void dump_hex(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        printf("%02x", p[i]);
    }
}

/* 从整文件的物理区间表里切出逻辑区间 [off, off+len) 对应的子表 */
static int slice_cf(const NvmeCopyFormat2 *cf, int nr, uint64_t off, uint64_t len,
                    std::vector<NvmeCopyFormat2> &out)
{
    out.clear();
    uint64_t pos = 0;
    for (int i = 0; i < nr && len > 0; i++) {
        const uint64_t ent_len = (uint64_t)(cf[i].nlb + 1) * LBA_SIZE;
        const uint64_t ent_start = pos;
        const uint64_t ent_end = pos + ent_len;
        pos = ent_end;

        const uint64_t a = std::max(off, ent_start);
        const uint64_t b = std::min(off + len, ent_end);
        if (a >= b) {
            continue;
        }

        NvmeCopyFormat2 c = cf[i];
        c.slba = cf[i].slba + (a - ent_start) / LBA_SIZE;
        c.nlb  = (uint16_t)((b - a) / LBA_SIZE - 1);
        out.push_back(c);
    }
    return (int)out.size();
}

/* 目标文件尺寸与 new 不一致时，扩展/收缩到 new 的尺寸，并保证块已分配 */
static int ensure_target_size(const std::string &path, uint64_t want)
{
    struct stat st {};
    if (stat(path.c_str(), &st) < 0) {
        fprintf(stderr, "目标文件不存在: %s\n", path.c_str());
        return -1;
    }
    if ((uint64_t)st.st_size == want) {
        return 0;
    }

    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0) {
        perror("open target");
        return -1;
    }

    if ((uint64_t)st.st_size > want) {
        if (ftruncate(fd, (off_t)want) < 0) {
            perror("ftruncate");
            close(fd);
            return -1;
        }
    } else {
        /*
         * 只 ftruncate/fallocate 会留下 unwritten extent，ext4 读它们时直接返回零，
         * 设备侧写进去的内容对 guest 不可见。因此新增的尾段必须真实写一遍。
         */
        const uint64_t from = (uint64_t)st.st_size;
        if (ftruncate(fd, (off_t)want) < 0) {
            perror("ftruncate");
            close(fd);
            return -1;
        }
        static uint8_t zeros[4096] = {0};
        uint64_t off = from;
        while (off < want) {
            const size_t n = (size_t)std::min<uint64_t>(sizeof(zeros), want - off);
            if (pwrite(fd, zeros, n, (off_t)off) != (ssize_t)n) {
                perror("pwrite zeros");
                close(fd);
                return -1;
            }
            off += n;
        }
        fsync(fd);
    }
    close(fd);

    if (stat(path.c_str(), &st) < 0 || (uint64_t)st.st_size != want) {
        fprintf(stderr, "目标文件尺寸调整失败\n");
        return -1;
    }
    printf("目标文件尺寸已调整为 %lu B\n", (unsigned long)want);
    return 0;
}

int main(int argc, char **argv)
{
    const std::string old_path = (argc > 1) ? argv[1] : DEF_OLD;
    const std::string new_path = (argc > 2) ? argv[2] : DEF_NEW;
    const std::string tgt_path = (argc > 3) ? argv[3] : DEF_TGT;

    printf("=== rsync-CSD（指纹现场算 + 增量重构全部下沉）===\n");
    printf("老文件: %s\n新文件: %s\n目标  : %s\n", old_path.c_str(),
           new_path.c_str(), tgt_path.c_str());

    struct stat st_old {}, st_new {};
    if (stat(old_path.c_str(), &st_old) < 0 || stat(new_path.c_str(), &st_new) < 0) {
        perror("stat");
        return 1;
    }

    const uint64_t old_size = (uint64_t)st_old.st_size;
    const uint64_t new_size = (uint64_t)st_new.st_size;

    if (old_size == 0 || new_size == 0) {
        fprintf(stderr, "文件不能为空\n");
        return 1;
    }
    if (old_size % BLK != 0 || new_size % BLK != 0) {
        fprintf(stderr, "老/新文件大小必须是 4KB 的整数倍（当前 %lu / %lu）\n",
                (unsigned long)old_size, (unsigned long)new_size);
        return 1;
    }
    if (old_size / LBA_SIZE > 0xFFFFFFULL || new_size / LBA_SIZE > 0xFFFFFFULL) {
        fprintf(stderr, "文件过大（chunk_nlb 为 24bit）\n");
        return 1;
    }

    const uint32_t nr_old_pages = (uint32_t)(old_size / BLK);
    const uint32_t nr_new_pages = (uint32_t)(new_size / BLK);

    printf("版本: 老 %lu B (%u 块) -> 新 %lu B (%u 块)\n",
           (unsigned long)old_size, nr_old_pages,
           (unsigned long)new_size, nr_new_pages);

    if (ensure_target_size(tgt_path, new_size) < 0) {
        return 1;
    }

    /* ---------- FIEMAP：老文件与目标文件的物理布局 ---------- */
    NvmeCopyFormat2 *old_cf = nullptr, *tgt_cf = nullptr;
    uint64_t old_sz = 0, tgt_sz = 0;
    int npc = 0, npc2 = 0;
    int nr_old = get_file_mappings(old_path.c_str(), &old_sz, &old_cf, 1, &npc, &npc2);
    if (nr_old <= 0 || old_sz != old_size) {
        fprintf(stderr, "老文件 FIEMAP 失败\n");
        return 1;
    }
    npc = 0; npc2 = 0;
    int nr_tgt = get_file_mappings(tgt_path.c_str(), &tgt_sz, &tgt_cf, 1, &npc, &npc2);
    if (nr_tgt <= 0 || tgt_sz != new_size) {
        fprintf(stderr, "目标文件 FIEMAP 失败（%lu B != %lu B）\n",
                (unsigned long)tgt_sz, (unsigned long)new_size);
        return 1;
    }
    printf("FIEMAP: 老文件 %d 段, 目标 %d 段\n", nr_old, nr_tgt);

    /* ---------- 打开设备节点 ---------- */
    int ctl_fd = open(CSD_CTL_DEV, O_RDWR);
    if (ctl_fd < 0) { perror("open " CSD_CTL_DEV); return 1; }
    int ng_fd = open(CSD_NG_DEV, O_RDWR);
    if (ng_fd < 0) { perror("open " CSD_NG_DEV); return 1; }

    int m_out = open(FDM_MATCH_OUT, O_RDWR);
    int d_in  = open(FDM_DATA_IN, O_RDWR);
    int d_old = open(FDM_OLD_DATA, O_RDWR);
    int p_out = open(FDM_PATCH_OUT, O_RDWR);
    if (m_out < 0 || d_in < 0 || d_old < 0 || p_out < 0) {
        perror("open fdm0 文件（先执行 vm_setup.sh）");
        return 1;
    }

    uint16_t pind_match = 0, pind_patch = 0;
    if (load_csf(ctl_fd, "./build/rsync_match_data.so", "rsync_match_data",
                 "rsync_match_data_v1", &pind_match)) return 1;
    if (load_csf(ctl_fd, "./build/rsync_patch.so", "rsync_patch",
                 "rsync_patch", &pind_patch)) return 1;
    printf("CSF 已装载: rsync_match_data pind=%u, rsync_patch pind=%u\n",
           pind_match, pind_patch);

    /* ---------- 主机侧：算新文件每页指纹表 ---------- */
    std::vector<uint8_t> new_data(new_size);
    {
        int fd = open(new_path.c_str(), O_RDONLY);
        if (fd < 0 || !read_full(fd, new_data.data(), new_size)) {
            perror("read new");
            return 1;
        }
        close(fd);
    }

    std::vector<uint8_t> fp_table(sizeof(struct rsync_fp_table_hdr) +
                                  (size_t)nr_new_pages * sizeof(struct rsync_fp_entry), 0);
    {
        auto *hdr = (struct rsync_fp_table_hdr *)fp_table.data();
        hdr->magic = RSYNC_FP_TABLE_MAGIC;
        hdr->nr_entries = nr_new_pages;
        hdr->entry_bytes = sizeof(struct rsync_fp_entry);
        hdr->file_size = new_size;

        auto *ent = (struct rsync_fp_entry *)(fp_table.data() + sizeof(*hdr));
        for (uint32_t i = 0; i < nr_new_pages; i++) {
            ent[i].blk_idx = i;
            ent[i].flags = 0;
            csd_fp256(new_data.data() + (size_t)i * BLK, BLK, ent[i].fp);
        }
    }

    /* ---------- Phase 1：窗口化"读数据 + 现场算指纹 + 匹配"（全部在 CSD） ---------- */
    const uint64_t MATCH_WIN_BYTES = 32ULL << 20;   /* 每条命令 32MB 数据窗口 */
    const uint32_t win_pages_max = (uint32_t)(MATCH_WIN_BYTES / BLK);

    std::vector<struct rsync_match_token> all_tokens;
    uint64_t matched_total = 0, changed_total = 0;
    uint32_t base_old_blk = 0;
    int nr_win = 0;

    /* 各窗口输出区容量相同（末窗更小），按最大值分配清零缓冲 */
    const long long match_out_cap = (long long)round_up_4k(
        sizeof(struct rsync_match_hdr) + (size_t)win_pages_max * sizeof(struct rsync_match_token));
    uint8_t *zbuf = nullptr;
    if (posix_memalign((void **)&zbuf, 4096, (size_t)match_out_cap) != 0 || !zbuf) {
        fprintf(stderr, "分配对齐缓冲失败\n");
        return 1;
    }
    memset(zbuf, 0, (size_t)match_out_cap);

    /*
     * 回读缓冲同样必须 4KB 对齐：fdmfs 对 pread/pwrite 的用户缓冲区有对齐要求，
     * 未对齐直接返回 EINVAL（曾用普通 std::vector 回读，导致必然崩溃）。
     */
    uint8_t *mbuf = nullptr;
    if (posix_memalign((void **)&mbuf, 4096, (size_t)match_out_cap) != 0 || !mbuf) {
        fprintf(stderr, "分配对齐回读缓冲失败\n");
        return 1;
    }

    auto t0 = std::chrono::high_resolution_clock::now();
    while (base_old_blk < nr_old_pages) {
        const uint32_t pages = (uint32_t)std::min<uint64_t>(
            (uint64_t)win_pages_max, nr_old_pages - base_old_blk);
        const uint64_t win_bytes = (uint64_t)pages * BLK;

        std::vector<NvmeCopyFormat2> wcf;
        if (slice_cf(old_cf, nr_old, (uint64_t)base_old_blk * BLK, win_bytes, wcf) <= 0) {
            fprintf(stderr, "窗口 @%u 物理区间切片失败\n", base_old_blk);
            return 1;
        }

        struct ioctl_create_mrs mrs_a {};
        prep_mrs(&mrs_a, 2);
        mrs_a.fd[0] = m_out; mrs_a.off[0] = 0; mrs_a.size[0] = match_out_cap;
        mrs_a.fd[1] = d_in;  mrs_a.off[1] = 0; mrs_a.size[1] = (long long)round_up_4k(win_bytes);
        if (ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_a) != 0) {
            perror("CREATE_MRS(match)");
            return 1;
        }

        /* 输出区清零：tokens 依赖 header 计数，防上一窗口残迹 */
        if (pwrite(m_out, zbuf, (size_t)match_out_cap, 0) != (ssize_t)match_out_cap) {
            perror("清零 match_out");
            return 1;
        }

        const uint32_t chunk_nlb = (uint32_t)(win_bytes / LBA_SIZE) - 1;
        long long match_ret = -1;
        int r = run_indirect(ng_fd, pind_match, mrs_a.rsid,
                             /*cparam1=*/base_old_blk, /*cparam2=*/pages,
                             /*destination=*/0, /*read_mode=*/CSD_READ_MODE_DATA,
                             wcf.data(), (int)wcf.size(), nullptr, 0,
                             fp_table.data(), fp_table.size(),
                             chunk_nlb, &match_ret);
        if (r < 0) {
            fprintf(stderr, "rsync_match_data 窗口 @%u 失败: %d (%s)\n",
                    base_old_blk, r, strerror(-r));
            return 1;
        }

        /* 回读本窗口匹配结果（对齐缓冲） */
        if (pread(m_out, mbuf, (size_t)match_out_cap, 0) != (ssize_t)match_out_cap) {
            perror("pread match_out");
            return 1;
        }
        auto *mhdr = (struct rsync_match_hdr *)mbuf;
        if (mhdr->magic != RSYNC_MATCH_MAGIC) {
            fprintf(stderr, "匹配结果 magic 错误: 0x%lx\n", (unsigned long)mhdr->magic);
            return 1;
        }
        auto *tokens = (struct rsync_match_token *)(mbuf + sizeof(*mhdr));
        uint32_t take = std::min<uint32_t>(mhdr->nr_matched, pages);
        all_tokens.insert(all_tokens.end(), tokens, tokens + take);
        matched_total += take;
        changed_total += pages - take;

        ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs_a);
        free(mrs_a.fd); free(mrs_a.off); free(mrs_a.size);

        base_old_blk += pages;
        nr_win++;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    free(zbuf);
    free(mbuf);

    printf("Phase1 rsync_match_data: 完成, %d 窗口, 命中 %lu 块, 未命中 %lu 块, 耗时 %ld us\n",
           nr_win, (unsigned long)matched_total, (unsigned long)changed_total,
           (long)std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

    /* ---------- Phase 2：生成 delta（以新文件块数为准） ---------- */
    std::vector<uint32_t> old_of_new(nr_new_pages, UINT32_MAX);
    for (size_t i = 0; i < all_tokens.size(); i++) {
        if (all_tokens[i].new_blk_idx < nr_new_pages &&
            all_tokens[i].old_blk_idx < nr_old_pages) {
            old_of_new[all_tokens[i].new_blk_idx] = all_tokens[i].old_blk_idx;
        }
    }

    /*
     * ---------- Phase 1.5：强校验（只对候选块） ----------
     * 设备内那一级（csd_fp256）只当弱哈希挑候选。按统一口径，
     * 候选块还要在主机上过一道真 SHA-1：
     *   - 新文件块：已在内存的 new_data，直接算
     *   - 老文件块：经 PCIe pread 读进主机再算
     * 不一致的降级为字面量。
     */
    uint32_t nr_strong_drop = 0;
    {
        auto ts0 = std::chrono::high_resolution_clock::now();

        int fd_so = open(old_path.c_str(), O_RDONLY);
        if (fd_so < 0) { perror("open old(强校验)"); return 1; }

        std::vector<uint8_t> obuf;
        uint64_t nr_cand = 0, bytes_old = 0;

        uint32_t i = 0;
        while (i < nr_new_pages) {
            if (old_of_new[i] == UINT32_MAX) {
                i++;
                continue;
            }
            /* 新块号与老块号同时连续 -> 合并成一次 pread */
            uint32_t j = i + 1;
            while (j < nr_new_pages &&
                   old_of_new[j] != UINT32_MAX &&
                   old_of_new[j] == old_of_new[i] + (j - i)) {
                j++;
            }

            const uint64_t run_blk = (uint64_t)(j - i);
            const uint64_t run_bytes = run_blk * BLK;
            const uint64_t ooff = (uint64_t)old_of_new[i] * BLK;

            obuf.resize((size_t)run_bytes);
            uint64_t done = 0;
            bool read_ok = true;
            while (done < run_bytes) {
                const ssize_t r = pread(fd_so, obuf.data() + done,
                                        (size_t)(run_bytes - done),
                                        (off_t)(ooff + done));
                if (r <= 0) { read_ok = false; break; }
                done += (uint64_t)r;
            }
            if (!read_ok) {
                for (uint32_t k = i; k < j; k++) {
                    old_of_new[k] = UINT32_MAX;
                    nr_strong_drop++;
                }
                i = j;
                continue;
            }

            for (uint32_t k = 0; k < run_blk; k++) {
                uint8_t h_new[CSD_SHA1_LEN], h_old[CSD_SHA1_LEN];
                csd_sha1(new_data.data() + (size_t)(i + k) * BLK, BLK, h_new);
                csd_sha1(obuf.data() + (size_t)k * BLK, BLK, h_old);

                nr_cand++;
                bytes_old += BLK;
                if (memcmp(h_new, h_old, CSD_SHA1_LEN) != 0) {
                    old_of_new[i + k] = UINT32_MAX;
                    nr_strong_drop++;
                }
            }
            i = j;
        }
        close(fd_so);

        auto ts1 = std::chrono::high_resolution_clock::now();
        printf("Phase1.5 强校验 SHA-1: 候选 %lu 块, 淘汰 %u 块, "
               "读老文件 %lu B (经 PCIe), 耗时 %ld us\n",
               (unsigned long)nr_cand, nr_strong_drop, (unsigned long)bytes_old,
               (long)std::chrono::duration_cast<std::chrono::microseconds>(ts1 - ts0).count());
    }

    uint32_t nr_lit = 0;
    for (uint32_t i = 0; i < nr_new_pages; i++) {
        if (old_of_new[i] == UINT32_MAX) {
            nr_lit++;
        }
    }

    printf("Phase2 delta: %u 条 COPY_OLD, %u 条 INSERT_LIT, 字面量共 %lu B (占比 %.2f%%)\n",
           nr_new_pages - nr_lit, nr_lit, (unsigned long)nr_lit * BLK,
           (double)nr_lit * 100.0 * BLK / (double)new_size);

    /* ---------- Phase 3：主机侧重建（数据经 PCIe 读回主机内存，再写目标） ---------- */
    /*
     * 与其它各组口径统一：不再用 rsync_patch 在设备内重构写回，
     * 改成「pread 老文件块 -> 主机内存 -> pwrite 目标文件」，字面量从
     * 已在主机内存里的 new_data 取。连续且老块号也连续的 COPY_OLD 合并成
     * 一段，减少系统调用次数。
     */
    const size_t RB_BUF = 1ULL << 20;           /* 重建搬运缓冲 1MB */

    int fd_old_rb = open(old_path.c_str(), O_RDONLY);
    if (fd_old_rb < 0) { perror("open old(重建)"); return 1; }
    int fd_tgt_rb = open(tgt_path.c_str(), O_WRONLY);
    if (fd_tgt_rb < 0) { perror("open target(重建)"); return 1; }

    std::vector<uint8_t> rb_buf(RB_BUF);

    auto t2 = std::chrono::high_resolution_clock::now();
    uint32_t seg_start = 0;
    int nr_seg = 0;
    uint64_t moved = 0;
    bool rb_ok = true;

    while (seg_start < nr_new_pages && rb_ok) {
        const bool is_lit = (old_of_new[seg_start] == UINT32_MAX);
        uint32_t seg_end = seg_start;

        if (!is_lit) {
            /* 合并连续的、且老块号也连续的 COPY_OLD */
            while (seg_end < nr_new_pages &&
                   old_of_new[seg_end] != UINT32_MAX &&
                   old_of_new[seg_end] == old_of_new[seg_start] + (seg_end - seg_start)) {
                seg_end++;
            }
        } else {
            while (seg_end < nr_new_pages && old_of_new[seg_end] == UINT32_MAX) {
                seg_end++;
            }
        }

        const uint64_t off_out = (uint64_t)seg_start * BLK;
        const uint64_t len = (uint64_t)(seg_end - seg_start) * BLK;

        if (is_lit) {
            /* 字面量：源已经在主机内存的 new_data 里 */
            if (!write_full_at(fd_tgt_rb, new_data.data() + off_out, (size_t)len,
                               (off_t)off_out)) {
                perror("pwrite literal");
                rb_ok = false;
                break;
            }
        } else {
            const uint64_t off_in = (uint64_t)old_of_new[seg_start] * BLK;
            uint64_t done = 0;
            while (done < len) {
                const size_t want = (size_t)std::min<uint64_t>(RB_BUF, len - done);
                ssize_t r = pread(fd_old_rb, rb_buf.data(), want, (off_t)(off_in + done));
                if (r <= 0) { perror("pread old"); rb_ok = false; break; }
                if (!write_full_at(fd_tgt_rb, rb_buf.data(), (size_t)r,
                                   (off_t)(off_out + done))) {
                    perror("pwrite target");
                    rb_ok = false;
                    break;
                }
                done += (uint64_t)r;
            }
        }

        moved += len;
        seg_start = seg_end;
        nr_seg++;
    }

    close(fd_old_rb);
    close(fd_tgt_rb);
    auto t3 = std::chrono::high_resolution_clock::now();

    if (!rb_ok) {
        return 1;
    }

    printf("Phase3 主机重建: 完成, %d 段, 搬运 %lu B (%u 块), 耗时 %ld us, %.1f MB/s\n",
           nr_seg, (unsigned long)moved, nr_new_pages,
           (long)std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count(),
           (double)moved / 1048576.0 /
               ((double)std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count()
                / 1e6));

    /* ---------- 校验：目标文件是否等于新文件 ---------- */
    /*
     * 关键：设备侧的写入不经过 guest 的块层，ext4 页缓存里仍是旧内容，
     * 必须先丢弃该文件的缓存页，否则校验必然读回旧数据。
     */
    std::vector<uint8_t> got(new_size);
    {
        int fd = open(tgt_path.c_str(), O_RDONLY);
        if (fd < 0) {
            perror("open target");
            return 1;
        }
        posix_fadvise(fd, 0, (off_t)new_size, POSIX_FADV_DONTNEED);
        if (!read_full(fd, got.data(), new_size)) {
            perror("read target");
            return 1;
        }
        close(fd);
    }

    const int cmp = memcmp(got.data(), new_data.data(), new_size);
    printf("\n--- 结果 ---\n");
    printf("老文件        : %lu B (%u 块)\n", (unsigned long)old_size, nr_old_pages);
    printf("新文件        : %lu B (%u 块)\n", (unsigned long)new_size, nr_new_pages);
    printf("命中复用      : %lu 块 (%lu B)\n", (unsigned long)matched_total,
           (unsigned long)matched_total * BLK);
    printf("字面量传输    : %u 块 (%lu B)\n", nr_lit, (unsigned long)nr_lit * BLK);
    printf("%s\n", cmp == 0 ? "[PASS] 同步完成，目标文件与新文件逐字节一致"
                            : "[FAIL] 目标文件与新文件不一致");

    /* ---------- 清理 ---------- */
    struct ioctl_download dl {};
    dl.name = (char *)"rsync_match_data";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    dl.name = (char *)"rsync_patch";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);

    free(old_cf); free(tgt_cf);
    close(m_out); close(d_in); close(d_old); close(p_out);
    close(ng_fd); close(ctl_fd);

    return cmp == 0 ? 0 : 1;
}
