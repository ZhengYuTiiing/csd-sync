/*
 * conc_test.cpp —— 并发实验：多条 in-flight indirect 命令能否真并行
 *
 * 背景：CEMU 里每条 indirect 命令在 compute_thread 收到后另起一个
 * "indirect_scheduler" 线程（compute.c:1270），命令内部 R->C->W 串行，
 * 但命令之间是不同线程。本实验验证：
 *   串行提交 N 条窗口命令  vs  一次提交 N 条再统一收割
 * 挂钟是否有收益，以决定 dsync3 能否启用深度提交。
 *
 * 每条命令用独立托盘 / carry / desc / 均质网格切片，互不共享，
 * 保证测的是"并行能力"而不是"依赖串行"。
 *
 * 用法:
 *   ./build/conc_test <file> [-n 窗口数] [-w 窗口MB] [-r 重复次数] [-s dev]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>
#include <algorithm>
#include <ctime>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <liburing.h>

#include "util.h"
#include "kernel/dsync_abi.h"
#include "kernel/chunk_abi.h"
#include "../../hw/femu/backend/csd_fp_crc.h"

#define PAGE       4096ull
#define SEC        512ull

#define CDC_CARRY_MAGIC 0x4341525231000001ULL
struct cdc_carry_hdr {
    uint64_t magic;
    uint64_t carry_bytes;
    uint64_t carry_base;
    uint64_t total_size;
    uint32_t flip;
    uint32_t reserved;
};

#define CARRY_IO (round_up_4k(sizeof(struct cdc_carry_hdr) + 65536 + 2 * 17 * CSD_CRC_SLOT_LEN))

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static uint64_t round_up_4k_def(uint64_t v) { return (v + PAGE - 1) & ~(PAGE - 1); }
#define round_up_4k(v) round_up_4k_def(v)

static void *fdm_alloc(size_t bytes)
{
    void *p = nullptr;
    if (posix_memalign(&p, PAGE, round_up_4k(bytes)) != 0 || !p) {
        return nullptr;
    }
    memset(p, 0, round_up_4k(bytes));
    return p;
}

static int ensure_fdm(const char *dir, int slot, uint64_t need, const char *what,
                      int *io_fd_out)
{
    char path[256];
    uint64_t want = round_up_4k(need);

    snprintf(path, sizeof(path), "%s%d", dir, slot);
    if (io_fd_out) {
        *io_fd_out = -1;
    }
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        fprintf(stderr, "open %s (%s): %s\n", path, what, strerror(errno));
        return -1;
    }
    struct stat st {};
    fstat(fd, &st);
    if ((uint64_t)st.st_size < want) {
        close(fd);
        unlink(path);
        fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            return -1;
        }
        int err = posix_fallocate(fd, 0, (off_t)want);
        if (err != 0 && ftruncate(fd, (off_t)want) != 0) {
            err = errno;
        }
        fstat(fd, &st);
        if ((uint64_t)st.st_size < want) {
            fprintf(stderr, "FDM %s 需要 %llu B，实际 %llu B\n", path,
                    (unsigned long long)want, (unsigned long long)st.st_size);
            close(fd);
            return -1;
        }
    }
    if (io_fd_out) {
        *io_fd_out = open(path, O_RDWR);
        if (*io_fd_out < 0) {
            close(fd);
            return -1;
        }
    }
    return fd;
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

static int delete_mrs(int ctl_fd, uint16_t rsid)
{
    struct ioctl_create_mrs arg {};
    arg.rsid = rsid;
    return ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &arg);
}

/*
 * 窗口描述：一条独立命令所需的全部资源
 */
struct Win {
    std::vector<NvmeCopyFormat2> cf;   /* 本窗口物理段 */
    uint64_t nlb = 0;
    uint64_t base = 0;

    int fd_tray = -1, fd_desc = -1, fd_carry = -1, fd_slice = -1;
    uint16_t rsid = 0;

    std::vector<uint8_t> task;        /* 序列化的间接任务元数据 */
};

int main(int argc, char **argv)
{
    std::string path;
    int n_win = 4;
    uint64_t win_mb = 4;
    int reps = 3;
    int dev = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-n") && i + 1 < argc) {
            n_win = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-w") && i + 1 < argc) {
            win_mb = strtoull(argv[++i], nullptr, 10);
        } else if (!strcmp(argv[i], "-r") && i + 1 < argc) {
            reps = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-s") && i + 1 < argc) {
            dev = atoi(argv[++i]);
        } else if (path.empty()) {
            path = argv[i];
        }
    }
    if (path.empty() || n_win < 1) {
        fprintf(stderr, "用法: %s <file> [-n 窗口数] [-w 窗口MB] [-r 重复] [-s dev]\n",
                argv[0]);
        return 1;
    }

    int fd_file = open(path.c_str(), O_RDONLY);
    if (fd_file < 0) {
        perror("open file");
        return 1;
    }
    struct stat st {};
    fstat(fd_file, &st);
    const uint64_t fsize = (uint64_t)st.st_size;
    const uint64_t win_bytes = win_mb << 20;

    if (fsize < win_bytes * (uint64_t)n_win) {
        fprintf(stderr, "文件 %llu B 不够 %d 个 %llu MB 窗口\n",
                (unsigned long long)fsize, n_win, (unsigned long long)win_mb);
        return 1;
    }

    /* FIEMAP 取物理段，按窗口切片 */
    NvmeCopyFormat2 *cf = nullptr;
    int nr_cf = 0, nlb_pc = 0, ncf_pc = 0;
    uint64_t fsz = 0;
    nr_cf = get_file_mappings(path.c_str(), &fsz, &cf, 1, &nlb_pc, &ncf_pc);
    if (nr_cf <= 0) {
        fprintf(stderr, "FIEMAP 失败\n");
        return 1;
    }

    /* 按 LBA 流切成 n_win 段，每段 win_bytes */
    std::vector<Win> wins(n_win);
    {
        const uint64_t win_nlb = win_bytes / SEC;
        uint64_t used = 0, idx = 0, cur_nlb = 0;
        for (int i = 0; i < nr_cf && (int)idx < n_win; i++) {
            uint64_t slba = (uint64_t)cf[i].slba;
            uint64_t left = (uint64_t)cf[i].nlb + 1;
            while (left > 0 && (int)idx < n_win) {
                uint64_t take = std::min(left, win_nlb - used);
                NvmeCopyFormat2 e {};
                e.slba = (uint32_t)slba;
                e.nlb  = (uint32_t)(take - 1);
                wins[idx].cf.push_back(e);
                wins[idx].nlb += take;
                used += take;
                slba += take;
                left -= take;
                if (used == win_nlb) {
                    wins[idx].base = cur_nlb * SEC;
                    cur_nlb += wins[idx].nlb;
                    idx++;
                    used = 0;
                }
            }
        }
        if ((int)idx < n_win) {
            fprintf(stderr, "只切出 %llu 个窗口\n", (unsigned long long)idx);
            return 1;
        }
    }

    /* 设备与算子 */
    char ctl[64], ng[64], fdm_dir[64];
    snprintf(ctl, sizeof(ctl), "/dev/nvme%dc3", dev);
    snprintf(ng, sizeof(ng), "/dev/ng%dn3", dev);
    snprintf(fdm_dir, sizeof(fdm_dir), "/mnt/fdm%d/", dev);

    int ctl_fd = open(ctl, O_RDWR);
    int ng_fd = open(ng, O_RDWR);
    if (ctl_fd < 0 || ng_fd < 0) {
        perror("open dev");
        return 1;
    }
    uint16_t pind = 0;
    if (load_csf(ctl_fd, "./build/cdc_pipe.so", "cdc_pipe", "cdc_pipe_conc", &pind) < 0) {
        return 1;
    }

    const uint64_t ent_cap  = win_bytes / CHUNK_MIN_SIZE + 2;
    const uint64_t cut_io   = round_up_4k(ent_cap * 24);
    const uint64_t tray_io  = win_bytes + cut_io;
    const uint64_t desc_io  = round_up_4k(sizeof(struct dsync_table_hdr) + ent_cap * sizeof(struct dsync_chunk_desc));
    const uint64_t slice_io = round_up_4k(win_bytes / PAGE * CSD_CRC_SLOT_LEN);
    const uint64_t carry_io = CARRY_IO;

    printf("文件 %s (%llu B)  窗口 %d × %llu MB  设备 nvme%d\n",
           path.c_str(), (unsigned long long)fsize, n_win,
           (unsigned long long)win_mb, dev);
    printf("单窗口资源: 托盘 %llu + desc %llu + 切片 %llu + carry %llu B\n",
           (unsigned long long)tray_io, (unsigned long long)desc_io,
           (unsigned long long)slice_io, (unsigned long long)carry_io);

    /* 每窗口独立槽位：托盘 1,7,8.., carry 6,10.., desc 0, 切片 2
     * 托盘/carry 用不同 slot 避免互相踩，desc 每窗口单独一份 */
    for (int i = 0; i < n_win; i++) {
        Win &w = wins[i];
        int tray_slot  = (i == 0) ? 1 : 7 + i;      /* 1, 8, 9, 10... */
        int carry_slot = (i == 0) ? 6 : 10 + i;     /* 6, 11, 12, 13... */
        int desc_slot  = (i == 0) ? 0 : 20 + i;     /* 0, 21, 22, 23... */

        w.fd_tray  = ensure_fdm(fdm_dir, tray_slot, tray_io, "托盘", nullptr);
        w.fd_carry = ensure_fdm(fdm_dir, carry_slot, carry_io, "carry", nullptr);
        w.fd_desc  = ensure_fdm(fdm_dir, desc_slot, desc_io, "desc", nullptr);
        w.fd_slice = (i == 0) ? ensure_fdm(fdm_dir, 2, slice_io, "切片", nullptr)
                              : dup(wins[0].fd_slice);
        if (w.fd_tray < 0 || w.fd_carry < 0 || w.fd_desc < 0 || w.fd_slice < 0) {
            return 1;
        }

        /* carry 初始化：空、total_size = 本窗口末尾（闭合判定用） */
        {
            uint8_t *b = (uint8_t *)fdm_alloc(carry_io);
            struct cdc_carry_hdr h {};
            h.magic = CDC_CARRY_MAGIC;
            h.carry_bytes = 0;
            h.carry_base = w.base;
            h.total_size = w.base + w.nlb * SEC;
            h.flip = 0;
            memcpy(b, &h, sizeof(h));
            if (pwrite(w.fd_carry, b, (size_t)carry_io, 0) != (ssize_t)carry_io) {
                perror("init carry");
                return 1;
            }
            free(b);
        }

        /* 清零 desc */
        {
            uint8_t *z = (uint8_t *)fdm_alloc(desc_io);
            pwrite(w.fd_desc, z, (size_t)desc_io, 0);
            free(z);
        }

        /* 建 MR（每窗口独立 rsid） */
        struct ioctl_create_mrs mrs {};
        prep_mrs(&mrs, 4);
        mrs.fd[0] = w.fd_desc;  mrs.off[0] = 0; mrs.size[0] = (long long)desc_io;
        mrs.fd[1] = w.fd_tray;  mrs.off[1] = 0; mrs.size[1] = (long long)tray_io;
        mrs.fd[2] = w.fd_slice; mrs.off[2] = 0; mrs.size[2] = (long long)slice_io;
        mrs.fd[3] = w.fd_carry; mrs.off[3] = 0; mrs.size[3] = (long long)carry_io;
        int ret = ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs);
        if (ret != 0) {
            fprintf(stderr, "CREATE_MRS 窗口 %d 失败 ret=%d\n", i, ret);
            return 1;
        }
        w.rsid = mrs.rsid;

        /* 打包任务元数据 */
        std::vector<int> in_pc(1, (int)w.cf.size());
        size_t need = calc_indirect_task_buffer_size(1, 0, (int)w.cf.size(), 0, 0);
        w.task.assign(need, 0);
        pack_indirect_task_buffer(w.task.data(), 1, 0, CSD_READ_MODE_DATA,
                                 (int)w.cf.size(), in_pc.data(), w.cf.data(),
                                 0, nullptr, nullptr, nullptr, 0);
    }

    auto fill_sqe = [&](struct io_uring_sqe *sqe, Win &w, uint64_t *ud) {
        struct nvme_uring_cmd cmd {};
        prep_nvme_uring_program_execute(&cmd, w.nlb * SEC, w.base, pind, w.rsid,
                                        (uint32_t)w.cf.size(), 0,
                                        (uint32_t)(w.nlb - 1), 0,
                                        w.task.data(), (uint32_t)w.task.size());
        memset(sqe, 0, sizeof(*sqe));
        sqe->opcode  = IORING_OP_URING_CMD;
        sqe->cmd_op  = NVME_URING_CMD_IO;
        sqe->fd      = ng_fd;
        sqe->user_data = ++(*ud);
        memcpy(sqe->cmd, &cmd, sizeof(cmd));
    };

    std::vector<long long> last_res;

    auto run_serial = [&]() -> double {
        struct io_uring ring;
        io_uring_queue_init(32, &ring, IORING_SETUP_SQE128 | IORING_SETUP_CQE32);

        last_res.clear();
        double t0 = now_ms();
        for (int i = 0; i < n_win; i++) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            uint64_t ud = 0;
            fill_sqe(sqe, wins[i], &ud);
            io_uring_submit(&ring);

            struct io_uring_cqe *cqe = nullptr;
            io_uring_wait_cqe(&ring, &cqe);
            last_res.push_back(cqe->res >= 0 ? (long long)cqe->big_cqe[0] : (long long)cqe->res);
            io_uring_cqe_seen(&ring, cqe);
        }
        double dt = now_ms() - t0;
        io_uring_queue_exit(&ring);
        return dt;
    };

    auto run_parallel = [&]() -> double {
        struct io_uring ring;
        io_uring_queue_init(32, &ring, IORING_SETUP_SQE128 | IORING_SETUP_CQE32);

        uint64_t ud = 0;
        double t0 = now_ms();
        for (int i = 0; i < n_win; i++) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) {
                fprintf(stderr, "get_sqe 失败（队列深度不足）\n");
                break;
            }
            fill_sqe(sqe, wins[i], &ud);
        }
        io_uring_submit(&ring);
        for (int i = 0; i < n_win; i++) {
            struct io_uring_cqe *cqe = nullptr;
            io_uring_wait_cqe(&ring, &cqe);
            io_uring_cqe_seen(&ring, cqe);
        }
        double dt = now_ms() - t0;
        io_uring_queue_exit(&ring);
        return dt;
    };

    auto median = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };

    printf("\n--- 预热 1 轮 ---\n");
    run_serial();

    std::vector<double> ts, tp;
    for (int r = 0; r < reps; r++) {
        ts.push_back(run_serial());
        tp.push_back(run_parallel());

        /* 校验两条路径产出的 desc 块数（fdmfs 只收 4KB 对齐访问） */
        uint64_t nr = 0;
        uint8_t *rb = (uint8_t *)fdm_alloc(desc_io);
        for (int i = 0; i < n_win; i++) {
            if (pread(wins[i].fd_desc, rb, (size_t)desc_io, 0) != (ssize_t)desc_io) {
                continue;
            }
            const struct dsync_table_hdr *h =
                (const struct dsync_table_hdr *)(const void *)rb;
            nr += (h->magic == DSYNC_TABLE_MAGIC) ? h->nr_chunks : 0;
        }
        free(rb);
        printf("  rep %d: 串行 %8.2f ms   并发 %8.2f ms   总块数 %llu   CSF 返回",
               r, ts.back(), tp.back(), (unsigned long long)nr);
        for (size_t k = 0; k < last_res.size(); k++) {
            printf(" %lld", last_res[k]);
        }
        printf("\n");
    }

    double ms_ser = median(ts), ms_par = median(tp);
    printf("\n=== 结论（%d 窗口，中位 %d 轮）===\n", n_win, reps);
    printf("串行提交: %8.2f ms   (%.2f ms/窗口)\n", ms_ser, ms_ser / n_win);
    printf("并发提交: %8.2f ms   (%.2f ms/窗口)\n", ms_par, ms_par / n_win);
    printf("加速比  : %.2f×  %s\n", ms_ser / ms_par,
           ms_ser / ms_par > 1.5 ? "=> 引擎支持多 in-flight，值得做深度提交"
                                 : "=> 无明显并行收益，深度提交收益有限");

    for (int i = 0; i < n_win; i++) {
        delete_mrs(ctl_fd, wins[i].rsid);
    }
    close(fd_file);
    close(ctl_fd);
    close(ng_fd);
    return 0;
}
