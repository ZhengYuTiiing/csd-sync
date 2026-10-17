/*
 * fastsync.cpp —— 一条命令跑完"设备内增量同步匹配"
 *
 *   ./build/fastsync <old> <new> [-o target] [-w MB]
 *
 * 全部流程：
 *   Phase A  设备浅读老文件 OOB，把 512B 扇区 CRC32C 网格拉到 FDM（8MB / 1GB）
 *   Phase B  设备读新文件，在设备内拍 CDC + 用网格拼出老文件同位置 CRC + 比对，
 *            直接产出"扇区级可复用位图"（1GB → 256KB）
 *   Phase C  主机按位图复用老数据 / 写入新数据，得到目标文件并逐字节校验
 *
 * 主机在 Phase B 期间：下发 N 条命令（每 32MB 一条），不做任何哈希、不比任何数据。
 *
 * 前置条件：
 *   - old / new 都在 /mnt/nvme0 上（FIEMAP 可解析），且大小是 4KB 整数倍
 *   - old 曾由设备写路径写过（OOB 网格已固化）
 *   - /mnt/fdm0/0..3 可用（容量不够会自动扩容）
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <liburing.h>

#include "util.h"
#include "kernel/rsync_csd_abi.h"
#include "../../hw/femu/backend/csd_fp_crc.h"

#define CSD_CTL_DEV   "/dev/nvme0c3"
#define CSD_NG_DEV    "/dev/ng0n3"

#define FDM_BITMAP    "/mnt/fdm0/0"
#define FDM_GRID      "/mnt/fdm0/1"
#define FDM_OOB       "/mnt/fdm0/2"
#define FDM_WIN       "/mnt/fdm0/3"

#define PAGE          4096ull
#define SEC           512ull
#define DEF_WIN_MB    32ull
#define MIN_WIN_MB    4ull

/* ------------------------------------------------------------------ */

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int submit_execute_cmd(int ng_fd, const struct nvme_uring_cmd *cmd,
                              long long *out_result)
{
    struct io_uring ring;

    if (io_uring_queue_init(16, &ring, IORING_SETUP_SQE128 | IORING_SETUP_CQE32) < 0) {
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
    sqe->fd = ng_fd;
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

static int load_csf(int ctl_fd, const char *so, const char *fn, const char *name,
                    uint16_t *pind)
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
 * 提交一条 indirect 命令。
 *   cf/nr_cf  : 本段要读的物理区间（逻辑顺序）
 *   chunk_nlb : 本段总 LBA 数（设备侧会 +1，所以这里传 nlb-1 由调用方处理）
 */
static int run_indirect(int ng_fd, uint16_t pind, uint16_t rsid,
                        uint64_t cparam1, uint64_t cparam2,
                        int read_mode, int destination,
                        const NvmeCopyFormat2 *cf, int nr_cf,
                        uint32_t chunk_nlb_minus1,
                        const void *extra, size_t extra_len,
                        long long *ret_out)
{
    std::vector<int> in_pc(1, nr_cf);
    const size_t need = calc_indirect_task_buffer_size(1, destination, nr_cf, 0, extra_len);
    std::vector<uint8_t> task(need, 0);

    size_t packed = pack_indirect_task_buffer(task.data(), 1, destination, read_mode,
                                              nr_cf, in_pc.data(), cf,
                                              0, nullptr, nullptr,
                                              extra, extra_len);
    if (packed != need) {
        return -1;
    }

    struct nvme_uring_cmd cmd {};
    prep_nvme_uring_program_execute(&cmd, cparam1, cparam2, pind, rsid,
                                    (uint32_t)nr_cf, 0, chunk_nlb_minus1, 0,
                                    task.data(), (uint32_t)packed);
    return submit_execute_cmd(ng_fd, &cmd, ret_out);
}

static uint64_t round_up_4k(uint64_t v)
{
    return (v + PAGE - 1) & ~(PAGE - 1);
}

/* fdmfs 只接受 O_DIRECT 读写：缓冲必须 4KB 对齐、长度必须 4KB 整数倍 */
static void *fdm_alloc(size_t bytes)
{
    void *p = nullptr;
    if (posix_memalign(&p, PAGE, round_up_4k(bytes)) != 0 || !p) {
        return nullptr;
    }
    memset(p, 0, round_up_4k(bytes));
    return p;
}

static int ensure_fdm(const char *path, uint64_t need, const char *what, int *io_fd_out)
{
    uint64_t want = round_up_4k(need);
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        fprintf(stderr, "open %s (%s): %s\n", path, what, strerror(errno));
        return -1;
    }
    if (io_fd_out) {
        *io_fd_out = -1;
    }
    struct stat st {};
    if (fstat(fd, &st) < 0) {
        fprintf(stderr, "fstat %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    /*
     * fdmfs 的两个坑（实测）：
     *   1. 只接受 O_DIRECT 读写，普通 write 返回 EINVAL
     *   2. 文件"原地长大"（ftruncate / 追加 fallocate）不会真正分配底层块，
     *      后续设备写进去的数据读回来是空的 —— 必须重建文件
     * 所以容量不够时一律 unlink 重建，而不是扩容。
     */
    if ((uint64_t)st.st_size < want) {
        close(fd);
        unlink(path);
        fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            fprintf(stderr, "recreate %s (%s): %s\n", path, what, strerror(errno));
            return -1;
        }
        int err = posix_fallocate(fd, 0, (off_t)want);
        if (err != 0 && ftruncate(fd, (off_t)want) != 0) {
            err = errno;
        }
        if (fstat(fd, &st) < 0 || (uint64_t)st.st_size < want) {
            fprintf(stderr,
                    "FDM %s (%s) 需要 %llu B，重建后仍是 %llu B（err=%d）。\n"
                    "请先在虚拟机内执行：fallocate -l %lluM %s\n",
                    path, what, (unsigned long long)need,
                    (unsigned long long)st.st_size, err,
                    (unsigned long long)((need + (1u << 20) - 1) >> 20), path);
            close(fd);
            return -1;
        }
    }
    /* MRS 用普通 fd（设备侧解析内存区间），主机读写用 O_DIRECT fd */
    if (io_fd_out) {
        *io_fd_out = open(path, O_RDWR | O_DIRECT);
        if (*io_fd_out < 0) {
            fprintf(stderr, "open %s O_DIRECT: %s\n", path, strerror(errno));
            close(fd);
            return -1;
        }
    }
    return fd;
}

/* 按位图把一个区间从 src 拷到 dst（优先走内核内 copy_file_range） */
static bool copy_run(int src_fd, int dst_fd, uint64_t off, uint64_t len, bool *fell_back)
{
    uint64_t done = 0;

    while (done < len) {
        loff_t in = (loff_t)(off + done);
        loff_t out = (loff_t)(off + done);
        ssize_t r = copy_file_range(src_fd, &in, dst_fd, &out, (size_t)(len - done), 0);
        if (r <= 0) {
            *fell_back = true;
            break;
        }
        done += (uint64_t)r;
    }
    if (done >= len) {
        return true;
    }

    /* 回退：用户态缓冲搬运 */
    static std::vector<uint8_t> buf;
    if (buf.empty()) {
        buf.resize(4u << 20);
    }
    while (done < len) {
        size_t chunk = (size_t)std::min<uint64_t>(buf.size(), len - done);
        ssize_t r = pread(src_fd, buf.data(), chunk, (off_t)(off + done));
        if (r != (ssize_t)chunk) {
            return false;
        }
        if (pwrite(dst_fd, buf.data(), chunk, (off_t)(off + done)) != r) {
            return false;
        }
        done += chunk;
    }
    return true;
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    setvbuf(stdout, nullptr, _IOLBF, 0);

    std::string old_path, new_path, tgt_path;
    uint64_t win_mb = DEF_WIN_MB;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            tgt_path = argv[++i];
        } else if (!strcmp(argv[i], "-w") && i + 1 < argc) {
            win_mb = strtoull(argv[++i], nullptr, 10);
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("用法: %s <old> <new> [-o target] [-w window_MB]\n", argv[0]);
            return 0;
        } else if (old_path.empty()) {
            old_path = argv[i];
        } else if (new_path.empty()) {
            new_path = argv[i];
        } else {
            fprintf(stderr, "多余的参数: %s\n", argv[i]);
            return 1;
        }
    }
    if (old_path.empty() || new_path.empty()) {
        fprintf(stderr, "用法: %s <old> <new> [-o target] [-w window_MB]\n", argv[0]);
        return 1;
    }
    if (tgt_path.empty()) {
        tgt_path = new_path + ".synced";
    }
    if (win_mb < MIN_WIN_MB) {
        win_mb = MIN_WIN_MB;
    }

    /* ---------- 0. 文件基本信息 ---------- */
    struct stat st_old {}, st_new {};
    if (stat(old_path.c_str(), &st_old) < 0) { perror("stat old"); return 1; }
    if (stat(new_path.c_str(), &st_new) < 0) { perror("stat new"); return 1; }

    const uint64_t old_size = (uint64_t)st_old.st_size;
    const uint64_t new_size = (uint64_t)st_new.st_size;
    if (!old_size || !new_size) {
        fprintf(stderr, "文件为空\n");
        return 1;
    }
    /* 不要求 4KB 整数倍：页数/扇区数向上取整，尾部不满一扇区走字面量 */

    const uint64_t old_pages = (old_size + PAGE - 1) / PAGE;   /* 向上取整 */
    const uint64_t old_nlb   = (old_size + SEC  - 1) / SEC;    /* 向上取整 */
    const uint64_t grid_bytes = old_pages * CSD_CRC_SLOT_LEN;
    const uint64_t new_nlb = (new_size + SEC - 1) / SEC;       /* 向上取整 */
    const uint64_t bitmap_bytes = (new_nlb + 7) / 8;

    printf("=== CSD 设备内增量同步 ===\n");
    printf("老文件 %s  (%llu B)\n", old_path.c_str(), (unsigned long long)old_size);
    printf("新文件 %s  (%llu B)\n", new_path.c_str(), (unsigned long long)new_size);
    printf("网格 %llu B (%.3f%% 于数据)   位图 %llu B\n",
           (unsigned long long)grid_bytes,
           100.0 * (double)grid_bytes / (double)old_size,
           (unsigned long long)bitmap_bytes);

    /* 自检：缓存算子的 combine 必须与朴素 combine 一致 */
    for (uint32_t v = 0; v < 4; v++) {
        uint32_t c1 = 0x12345678u + v * 0x01010101u;
        uint32_t c2 = 0x9abcdef0u - v * 0x00010001u;
        if (csd_crc32c_combine512(c1, c2) != csd_crc32c_combine(c1, c2, 512)) {
            fprintf(stderr, "内部错误：512B 移位算子自检失败\n");
            return 1;
        }
    }

    /* ---------- 1. FIEMAP ---------- */
    NvmeCopyFormat2 *old_cf = nullptr, *new_cf = nullptr;
    uint64_t fsz = old_size;
    int nlb_pc = 0, ncf_pc = 0;
    int nr_old_cf = get_file_mappings(old_path.c_str(), &fsz, &old_cf, 1, &nlb_pc, &ncf_pc);
    if (nr_old_cf <= 0 || fsz != old_size) {
        fprintf(stderr, "老文件 FIEMAP 失败（是否在 /mnt/nvme0 上？）\n");
        return 1;
    }

    fsz = new_size;
    nlb_pc = 0;
    ncf_pc = 0;
    int nr_new_cf = get_file_mappings(new_path.c_str(), &fsz, &new_cf, 1, &nlb_pc, &ncf_pc);
    if (nr_new_cf <= 0 || fsz != new_size) {
        fprintf(stderr, "新文件 FIEMAP 失败（是否在 /mnt/nvme0 上？）\n");
        return 1;
    }
    printf("FIEMAP: 老文件 %d 段, 新文件 %d 段\n", nr_old_cf, nr_new_cf);

    /* ---------- 2. 准备 FDM 缓冲 ---------- */
    /* 窗口缓冲按设备可用容量自适应收缩 */
    uint64_t win_bytes = win_mb << 20;
    int fd_win = -1, io_win = -1;
    while (win_bytes >= (MIN_WIN_MB << 20)) {
        fd_win = ensure_fdm(FDM_WIN, win_bytes, "窗口缓冲", &io_win);
        if (fd_win >= 0) {
            break;
        }
        win_bytes >>= 1;
    }
    if (fd_win < 0) {
        return 1;
    }
    if (win_bytes != (win_mb << 20)) {
        printf("窗口缓冲缩到 %llu MB（FDM 容量限制）\n", (unsigned long long)(win_bytes >> 20));
    }

    /* FDM 只吃 4KB 对齐的 I/O，所有区域大小上取整 */
    const uint64_t grid_io = round_up_4k(grid_bytes);
    const uint64_t bitmap_io = round_up_4k(bitmap_bytes);

    int io_bitmap = -1;
    int fd_bitmap = ensure_fdm(FDM_BITMAP, bitmap_io, "位图", &io_bitmap);
    int fd_grid = ensure_fdm(FDM_GRID, grid_io, "网格", nullptr);
    int fd_oob = ensure_fdm(FDM_OOB, grid_io, "OOB 暂存", nullptr);
    if (fd_bitmap < 0 || fd_grid < 0 || fd_oob < 0) {
        return 1;
    }

    /* 位图清零 */
    {
        uint8_t *zero = (uint8_t *)fdm_alloc(1u << 20);
        if (!zero) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }
        uint64_t off = 0;
        while (off < bitmap_io) {
            size_t n = (size_t)std::min<uint64_t>(1u << 20, bitmap_io - off);
            if (pwrite(io_bitmap, zero, n, (off_t)off) != (ssize_t)n) {
                perror("清空位图");
                return 1;
            }
            off += n;
        }
        free(zero);
    }

    /* ---------- 3. 装算子 ---------- */
    int ctl_fd = open(CSD_CTL_DEV, O_RDWR);
    int ng_fd = open(CSD_NG_DEV, O_RDWR);
    if (ctl_fd < 0 || ng_fd < 0) {
        perror("open csd dev");
        return 1;
    }

    uint16_t pind_collect = 0, pind_cdc = 0;
    if (load_csf(ctl_fd, "./build/collect_oob.so", "collect_oob", "collect_oob", &pind_collect) ||
        load_csf(ctl_fd, "./build/sync_cdc.so", "sync_cdc", "sync_cdc", &pind_cdc)) {
        return 1;
    }

    /* ---------- 4. Phase A：浅读老文件 OOB 网格 ---------- */
    struct ioctl_create_mrs mrs_a {};
    prep_mrs(&mrs_a, 2);
    mrs_a.fd[0] = fd_grid; mrs_a.off[0] = 0; mrs_a.size[0] = (long long)grid_io;
    mrs_a.fd[1] = fd_oob;  mrs_a.off[1] = 0; mrs_a.size[1] = (long long)grid_io;
    if (ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_a) != 0) {
        perror("CREATE_MRS(grid)");
        return 1;
    }

    long long r_a = -1;
    double t0 = now_ms();
    if (run_indirect(ng_fd, pind_collect, mrs_a.rsid, 0, 0,
                     CSD_READ_MODE_OOB, 0,
                     old_cf, nr_old_cf, (uint32_t)(old_nlb - 1),
                     nullptr, 0, &r_a) < 0) {
        fprintf(stderr, "Phase A 失败\n");
        return 1;
    }
    double t_a = now_ms() - t0;
    printf("Phase A  浅读 OOB 网格: 搬运 %llu B（数据量 %llu B，1/128），耗时 %.2f ms\n",
           (unsigned long long)grid_bytes, (unsigned long long)old_size, t_a);

    /* ---------- 5. Phase B：逐窗口 CDC + 拼接比对 ---------- */
    struct ioctl_create_mrs mrs_b {};
    prep_mrs(&mrs_b, 3);
    mrs_b.fd[0] = fd_bitmap; mrs_b.off[0] = 0; mrs_b.size[0] = (long long)bitmap_io;
    mrs_b.fd[1] = fd_win;    mrs_b.off[1] = 0; mrs_b.size[1] = (long long)win_bytes;
    mrs_b.fd[2] = fd_grid;   mrs_b.off[2] = 0; mrs_b.size[2] = (long long)grid_io;
    if (ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_b) != 0) {
        perror("CREATE_MRS(cdc)");
        return 1;
    }

    const uint64_t win_nlb_cap = win_bytes / SEC;
    uint64_t win_idx = 0, cur_nlb = 0;
    double t_b = 0;

    t0 = now_ms();
    for (int i = 0; i < nr_new_cf; ) {
        int start = i;
        uint64_t base_nlb = cur_nlb;

        /* 把连续的 CF2 攒成一个窗口，保证总 LBA 不超过窗口缓冲 */
        uint64_t acc = 0;
        while (i < nr_new_cf) {
            uint64_t nlb = (uint64_t)new_cf[i].nlb + 1;
            if (acc && acc + nlb > win_nlb_cap) {
                break;
            }
            acc += nlb;
            i++;
        }
        if (acc == 0) {
            break;
        }

        long long r_b = -1;
        double w0 = now_ms();
        if (run_indirect(ng_fd, pind_cdc, mrs_b.rsid,
                         acc * SEC, base_nlb * SEC,
                         CSD_READ_MODE_DATA, 0,
                         &new_cf[start], i - start, (uint32_t)(acc - 1),
                         nullptr, 0, &r_b) < 0) {
            fprintf(stderr, "Phase B 窗口 #%llu 失败\n", (unsigned long long)win_idx);
            return 1;
        }
        (void)r_b;
        t_b += now_ms() - w0;
        cur_nlb += acc;
        win_idx++;
    }
    if (cur_nlb != new_nlb) {
        fprintf(stderr, "覆盖不完整: %llu / %llu LBA\n",
                (unsigned long long)cur_nlb, (unsigned long long)new_nlb);
        return 1;
    }

    printf("Phase B  设备内 CDC + 拼接比对: %llu 个窗口, 耗时 %.2f ms\n",
           (unsigned long long)win_idx, t_b);

    /* ---------- 6. 读回位图（O_DIRECT：对齐缓冲 + 4KB 整数倍） ---------- */
    std::vector<uint8_t> bitmap(bitmap_bytes, 0);
    {
        uint8_t *buf = (uint8_t *)fdm_alloc(1u << 20);
        if (!buf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }
        uint64_t off = 0;
        while (off < bitmap_io) {
            size_t n = (size_t)std::min<uint64_t>(1u << 20, bitmap_io - off);
            if (pread(io_bitmap, buf, n, (off_t)off) != (ssize_t)n) {
                perror("pread bitmap");
                return 1;
            }
            size_t keep = (size_t)std::min<uint64_t>(n, bitmap_bytes - off);
            memcpy(bitmap.data() + off, buf, keep);
            off += n;
        }
        free(buf);
    }

    /* 命中量由位图统计（算子返回值会被流水线覆盖，位图才是权威） */
    {
        const uint64_t sec_cap = std::min<uint64_t>(new_nlb, old_nlb);
        uint64_t hit = 0;
        for (uint64_t s = 0; s < sec_cap; s++) {
            hit += (bitmap[s >> 3] >> (s & 7)) & 1;
        }
        printf("        设备判定可复用扇区 %llu / %llu (%.1f%%)\n",
               (unsigned long long)hit, (unsigned long long)new_nlb,
               100.0 * (double)hit / (double)new_nlb);
    }

    /* ---------- 7. Phase C：按位图复用 / 写入 ---------- */
    int fd_old = open(old_path.c_str(), O_RDONLY);
    int fd_new = open(new_path.c_str(), O_RDONLY);
    int fd_tgt = open(tgt_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd_old < 0 || fd_new < 0 || fd_tgt < 0) {
        perror("open data");
        return 1;
    }
    if (ftruncate(fd_tgt, (off_t)new_size) != 0) {
        perror("ftruncate target");
        return 1;
    }

    uint64_t reuse_bytes = 0, literal_bytes = 0, runs = 0;
    bool fbr = false;
    uint64_t run_start = UINT64_MAX;
    int run_bit = 0;

    auto flush_run = [&](uint64_t end_sec) -> bool {
        if (run_start == UINT64_MAX || end_sec <= run_start) {
            return true;
        }
        uint64_t off = run_start * SEC;
        /* 夹到真实文件长度：最后一个扇区可能只用了不到 512B */
        uint64_t len = std::min<uint64_t>((end_sec - run_start) * SEC, new_size - off);
        int src = run_bit ? fd_old : fd_new;
        bool ok = copy_run(src, fd_tgt, off, len, &fbr);
        if (ok) {
            if (run_bit) {
                reuse_bytes += len;
            } else {
                literal_bytes += len;
            }
            runs++;
        }
        return ok;
    };

    t0 = now_ms();
    for (uint64_t s = 0; s < new_nlb; s++) {
        int bit = (bitmap[s >> 3] >> (s & 7)) & 1;
        /*
         * 老文件最后一个不满 512B 的扇区一律不复用：
         * 它里面超出 EOF 的填充字节是设备页里的残留，两侧不一定一致，
         * 拿它复用会静默写错。走字面量最多多传 511B。
         * （old_size 本身是 512B 倍数时这条不触发）
         */
        if (bit && (s * SEC + SEC) > old_size) {
            bit = 0;
        }
        if (run_start == UINT64_MAX) {
            run_start = s;
            run_bit = bit;
        } else if (bit != run_bit) {
            if (!flush_run(s)) {
                fprintf(stderr, "Phase C 拷贝失败 @ 扇区 %llu\n", (unsigned long long)s);
                return 1;
            }
            run_start = s;
            run_bit = bit;
        }
    }
    if (!flush_run(new_nlb)) {
        fprintf(stderr, "Phase C 收尾拷贝失败\n");
        return 1;
    }
    double t_c = now_ms() - t0;

    /* ---------- 8. 校验 ---------- */
    bool same = true;
    {
        static std::vector<uint8_t> a, b;
        a.resize(4u << 20);
        b.resize(4u << 20);
        uint64_t off = 0;
        while (off < new_size && same) {
            size_t n = (size_t)std::min<uint64_t>(a.size(), new_size - off);
            if (pread(fd_new, a.data(), n, (off_t)off) != (ssize_t)n ||
                pread(fd_tgt, b.data(), n, (off_t)off) != (ssize_t)n) {
                perror("verify read");
                return 1;
            }
            if (memcmp(a.data(), b.data(), n) != 0) {
                same = false;
            }
            off += n;
        }
    }

    /* ---------- 9. 报告 ---------- */
    const double pct = 100.0 * (double)reuse_bytes / (double)new_size;
    printf("\n--- 结果 ---\n");
    printf("复用老数据 %10llu B (%.1f%%)  写入新数据 %10llu B (%.1f%%)\n",
           (unsigned long long)reuse_bytes, pct,
           (unsigned long long)literal_bytes, 100.0 - pct);
    printf("拷贝段 %llu 个（%s）\n", (unsigned long long)runs,
           fbr ? "copy_file_range 部分回退到缓冲拷贝" : "全程 copy_file_range");
    printf("主机在匹配阶段下发命令 %llu 条，读回位图 %llu B，主机侧哈希 0 次\n",
           (unsigned long long)win_idx, (unsigned long long)bitmap_bytes);
    printf("耗时: A %.2f ms / B %.2f ms (设备内) / C %.2f ms\n", t_a, t_b, t_c);

    /* 复用率条带图（64 列） */
    printf("复用分布: |");
    const int cols = 64;
    for (int c = 0; c < cols; c++) {
        uint64_t s0 = new_nlb * (uint64_t)c / (uint64_t)cols;
        uint64_t s1 = new_nlb * (uint64_t)(c + 1) / (uint64_t)cols;
        uint64_t hit = 0, tot = 0;
        for (uint64_t s = s0; s < s1; s++) {
            hit += (bitmap[s >> 3] >> (s & 7)) & 1;
            tot++;
        }
        double r = tot ? (double)hit / (double)tot : 0.0;
        const char *ramp = " .:-=+*#%@";
        int lv = (int)(r * 9.0);
        if (lv < 0) lv = 0;
        if (lv > 9) lv = 9;
        putchar(ramp[lv]);
    }
    printf("|\n");

    printf("%s  目标文件: %s\n", same ? "[PASS] 目标文件与新文件逐字节一致"
                                     : "[FAIL] 目标文件与新文件不一致",
           tgt_path.c_str());
    printf("验证命令: cmp %s %s\n", new_path.c_str(), tgt_path.c_str());

    /* ---------- 10. 清理 ---------- */
    struct ioctl_download dl {};
    dl.name = (char *)"collect_oob";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    dl.name = (char *)"sync_cdc";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs_a);
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs_b);
    free(mrs_a.fd); free(mrs_a.off); free(mrs_a.size);
    free(mrs_b.fd); free(mrs_b.off); free(mrs_b.size);
    free(old_cf);
    free(new_cf);
    close(fd_old); close(fd_new); close(fd_tgt);
    close(fd_bitmap); close(fd_grid); close(fd_oob); close(fd_win);
    if (io_bitmap >= 0) close(io_bitmap);
    if (io_win >= 0) close(io_win);
    close(ng_fd); close(ctl_fd);

    return same ? 0 : 1;
}
