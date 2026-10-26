/*
 * gridpipe.cpp —— grid-cdc 的**流水线版**驱动
 *
 *   ./build/gridpipe <server:f> <client:f'> [-o target] [-w MB] [-p P] [-c N] [-s N]
 *
 * 与 gridds.cpp 的关系：6 步流程、算法、ABI 完全一致，**只有执行模型不同** ——
 * 每条命令从「1 个 chunk」改成「P 个 chunk 并发」。刻意不共用文件，免得互相牵连。
 *
 * ── 流水线是怎么开出来的 ────────────────────────────────────────
 * FEMU 的 indirect 流水线（hw/femu/csd/compute.c: indirect_main）本身就是三级流水：
 *   stage0 读闪存 → stage1 跑 CSF → stage2 写回
 * `nr_concurrent_chunks` 决定一条命令里并发多少个 chunk：主循环一开始就把
 * **P 个读请求全部塞进 FTL 队列**（compute.c:1044-1077），IO 在 FTL 线程里异步
 * 完成、通过 ring 回头，CSF 在主线程里跑。于是 chunk j 的闪存读和 chunk i 的
 * CPU 计算是重叠的 —— 闪存通道和 CPU 本来就是两种资源。
 *
 * 此前所有实现都传 nr_concurrent_chunks = 1，等于把这条流水线关掉：
 * 先把整窗数据读完，才开始算。
 *
 * ── P 个 chunk 共用 cparam，CSF 怎么知道自己在哪 ────────────────
 * cparam1/cparam2 是**命令级**的（P 个 chunk 共用），而 struct cemu_args 里
 * 没有 chunk_id。解决办法：
 *   · mode 1/2：CSF 输出 **chunk 内相对偏移**，主机按 chunk 基址回填（主机
 *     自己切的 chunk，当然知道基址）。
 *   · mode 3 ：查询表给的是绝对偏移，只能把基址塞进 per-chunk 的 mr[2]
 *     （gd_desc）。mode 3 不用网格，正好腾出这个 MR。
 *
 * ── 约束 ────────────────────────────────────────────────────────
 *   · numr 必须能被 P 整除（compute.c:633），所以用 3×P 个 MR（out/data/grid）。
 *   · chunk 边界必须 4KB 对齐：CSF 用相对偏移查网格，网格 MR 从本 chunk 首页起。
 *   · 每个 chunk 只跑一轮（chunk_nlb = 本 chunk 全部 LBA）：多轮迭代会覆盖
 *     mr[0]/mr[1]，而 CSF 无法知道自己是第几轮。
 *   · 文件尾部的零头用 P=1 单独跑一条命令，保证每个 chunk 长度精确等于
 *     chunk_nlb（否则 CSF 按 cparam1 会多读越界数据）。
 *
 * 前置条件与 gridds 一致：f 经 server 设备写路径、f' 经 client 设备写路径写入
 * （OOB 网格写时固化），/mnt/fdm{C}/ 与 /mnt/fdm{S}/ 可用。
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
#include "kernel/dsync_abi.h"
#include "kernel/chunk_abi.h"
#include "../../hw/femu/backend/csd_fp_crc.h"
#include "../../hw/femu/backend/csd_sha1.h"

#define PAGE          4096ull
#define SEC           512ull
#define DEF_WIN_MB    32ull
#define MIN_WIN_MB    4ull
#define DEF_P         4           /* 默认流水线深度 */
#define MAX_MRS       32          /* MR 总数上限，3×P 不得超过它 */

#define GD_HDR_BYTES  ((uint64_t)sizeof(struct gd_tab_hdr))
#define GD_ENT_BYTES  ((uint64_t)sizeof(struct gd_ent))
#define GD_PAIR_BYTES ((uint64_t)sizeof(struct gd_pair))
#define GD_QHDR_BYTES ((uint64_t)sizeof(struct gd_query_hdr))
#define GD_Q_BYTES    ((uint64_t)sizeof(struct gd_query))

/* ------------------------------------------------------------------ */

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

enum PhaseWhere { PH_HOST, PH_DEVICE, PH_WIRE };

struct PhaseStat {
    const char *name;
    PhaseWhere  where;
    uint64_t    bytes;
    int         cnt;
    double      sum, min, max;
};

static std::vector<PhaseStat> g_ph;
static double g_t0_prog = 0.0;

static void phase_rec(const char *name, PhaseWhere where, uint64_t bytes, double ms)
{
    for (size_t i = 0; i < g_ph.size(); i++) {
        PhaseStat &p = g_ph[i];
        if (strcmp(p.name, name) == 0) {
            p.cnt++;
            p.sum += ms;
            p.min = std::min(p.min, ms);
            p.max = std::max(p.max, ms);
            p.bytes += bytes;
            return;
        }
    }
    PhaseStat p;
    p.name = name; p.where = where; p.bytes = bytes;
    p.cnt = 1; p.sum = p.min = p.max = ms;
    g_ph.push_back(p);
}

struct PhaseScope {
    const char     *name;
    PhaseWhere      where;
    const uint64_t *bytes;
    double          t0;
    PhaseScope(const char *n, PhaseWhere w, const uint64_t *b = nullptr)
        : name(n), where(w), bytes(b), t0(now_ms()) {}
    ~PhaseScope() { phase_rec(name, where, bytes ? *bytes : 0, now_ms() - t0); }
};

static const char *phase_where_str(PhaseWhere w)
{
    return w == PH_DEVICE ? "设备" : (w == PH_WIRE ? "线上" : "主机");
}

static std::string pad_disp(const char *s, size_t width)
{
    size_t w = 0;
    const unsigned char *p = (const unsigned char *)s;
    while (*p) {
        if (*p < 0x80)             { w += 1; p += 1; }
        else if ((*p & 0xE0) == 0xC0) { w += 1; p += 2; }
        else if ((*p & 0xF0) == 0xE0) { w += 2; p += 3; }
        else                        { w += 2; p += 4; }
    }
    std::string out(s);
    if (w < width) {
        out.append(width - w, ' ');
    }
    return out;
}

static void phase_report(double total_ms)
{
    printf("\n--- 分阶段耗时（墙钟，clock_gettime(CLOCK_MONOTONIC)）---\n");
    printf("%s %-6s %10s %5s %10s %7s %11s\n",
           pad_disp("阶段", 24).c_str(), "位置", "总ms", "次数", "均值ms",
           "占比", "吞吐MB/s");

    double acc = 0.0;
    for (size_t i = 0; i < g_ph.size(); i++) {
        acc += g_ph[i].sum;
    }
    for (size_t i = 0; i < g_ph.size(); i++) {
        const PhaseStat &p = g_ph[i];
        char thr[32];
        if (p.bytes && p.sum > 0.0) {
            snprintf(thr, sizeof(thr), "%.1f",
                     (double)p.bytes / (p.sum / 1000.0) / 1e6);
        } else {
            snprintf(thr, sizeof(thr), "-");
        }
        printf("%s %-6s %10.3f %5d %10.3f %6.1f%% %11s\n",
               pad_disp(p.name, 24).c_str(), phase_where_str(p.where),
               p.sum, p.cnt, p.sum / p.cnt,
               acc > 0.0 ? 100.0 * p.sum / acc : 0.0, thr);
    }
    printf("%s %-6s %10.3f %5s %10s %6.1f%%\n",
           pad_disp("上表合计", 24).c_str(), "", acc, "", "", 100.0);
    printf("%s %-6s %10.3f\n", pad_disp("程序总耗时", 24).c_str(), "", total_ms);
    if (total_ms > acc) {
        printf("%s %-6s %10.3f   (启动/参数解析/printf/收尾)\n",
               pad_disp("未计入", 24).c_str(), "", total_ms - acc);
    }
}

/* ---------------- 基础工具 ---------------- */

static uint64_t round_up_4k(uint64_t v)
{
    return (v + PAGE - 1) & ~(PAGE - 1);
}

/*
 * chunk 粒度必须对齐到 1024 LBA（512KB），不能只对齐到 8 LBA（4KB）。
 *
 * 踩过的坑：chunk i 的网格 MR 偏移 = (base_byte/4096) × 32 字节。
 * fdmfs 的 MR 是按页映射的，偏移必须是 4096 的倍数 ——
 * 只有 base_byte 是 512KB 的整数倍时，page×32 才是 4096 的倍数。
 * 只对齐到 4KB 时，P=8 的奇数 chunk 偏移 = i×10240 字节，不是 4096 的倍数，
 * 读到的网格是错位的：弱哈希全错，块数从 876 掉到 520、命中率崩到 58.7%。
 * （P=2/P=4 恰好落在 4096 边界上，所以当时没暴露。）
 */
static uint64_t align_down_512k(uint64_t v)
{
    return v & ~1023ull;    /* 1024 LBA = 512KB */
}

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
    if (fstat(fd, &st) < 0) {
        fprintf(stderr, "fstat %s: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }
    if ((uint64_t)st.st_size < want) {
        close(fd);
        unlink(path);
        fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            fprintf(stderr, "recreate %s: %s\n", path, strerror(errno));
            return -1;
        }
        int err = posix_fallocate(fd, 0, (off_t)want);
        if (err != 0 && ftruncate(fd, (off_t)want) != 0) {
            err = errno;
        }
        if (fstat(fd, &st) < 0 || (uint64_t)st.st_size < want) {
            fprintf(stderr, "FDM %s (%s) 需要 %llu B，重建后仍是 %llu B（err=%d）\n",
                    path, what, (unsigned long long)need,
                    (unsigned long long)st.st_size, err);
            close(fd);
            return -1;
        }
    }
    if (io_fd_out) {
        *io_fd_out = open(path, O_RDWR);
        if (*io_fd_out < 0) {
            fprintf(stderr, "open io %s: %s\n", path, strerror(errno));
            close(fd);
            return -1;
        }
    }
    return fd;
}

struct Dev {
    int idx = 0;
    int ctl_fd = -1;
    int ng_fd = -1;
    char fdm_dir[64];
};

static bool dev_open(Dev *d, int idx)
{
    char path[128];

    d->idx = idx;
    snprintf(d->fdm_dir, sizeof(d->fdm_dir), "/mnt/fdm%d/", idx);

    snprintf(path, sizeof(path), "/dev/nvme%dc3", idx);
    d->ctl_fd = open(path, O_RDWR);
    if (d->ctl_fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return false;
    }
    snprintf(path, sizeof(path), "/dev/ng%dn3", idx);
    d->ng_fd = open(path, O_RDWR);
    if (d->ng_fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return false;
    }
    return true;
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

/*
 * 下发一条 indirect 命令。P > 1 时就是流水线：
 * 设备会把 P 个 chunk 的读请求同时塞进 FTL，让闪存读与 CSF 计算重叠。
 */
static int run_indirect(int ng_fd, uint16_t pind, uint16_t rsid,
                        uint64_t cparam1, uint64_t cparam2,
                        int read_mode, int destination,
                        const NvmeCopyFormat2 *cf, int nr_cf,
                        const int *ncf_per_chunk, int P,
                        uint32_t chunk_nlb_minus1,
                        const void *extra, size_t extra_len,
                        long long *ret_out)
{
    const size_t need =
        calc_indirect_task_buffer_size(P, destination, nr_cf, 0, extra_len);
    std::vector<uint8_t> task(need, 0);

    size_t packed = pack_indirect_task_buffer(task.data(), P, destination,
                                              read_mode, nr_cf, ncf_per_chunk, cf,
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

/* ------------------------------------------------------------------ */
/* 一个 chunk：窗口内的一段连续 LBA + 它自己的 cf 列表                    */
/* ------------------------------------------------------------------ */
struct Chunk {
    uint64_t base_in_win = 0;                 /* 窗口内起始 LBA */
    uint64_t nlb = 0;
    std::vector<NvmeCopyFormat2> cf;
};

/*
 * 把一个窗口的 cf 列表按 LBA 切成 P 段。cf 条目允许被切开
 * （文件在物理上不连续，切点常常落在一个 extent 中间）。
 */
static void split_win(const NvmeCopyFormat2 *cf, int start, int cnt,
                      uint64_t per, std::vector<Chunk> &out)
{
    Chunk cur;
    bool open = false;
    uint64_t in_cur = 0;

    for (int i = start; i < start + cnt; i++) {
        uint64_t slba = cf[i].slba;
        uint64_t n = (uint64_t)cf[i].nlb + 1;

        while (n > 0) {
            if (!open || in_cur == per) {
                if (open) {
                    out.push_back(cur);
                }
                cur = Chunk();
                cur.base_in_win = 0;   /* 下面统一回填 */
                cur.nlb = 0;
                cur.cf.clear();
                open = true;
                in_cur = 0;
            }
            const uint64_t take = std::min(n, per - in_cur);
            NvmeCopyFormat2 e = cf[i];
            e.slba = slba;
            e.nlb = (uint32_t)(take - 1);
            cur.cf.push_back(e);
            cur.nlb += take;
            in_cur += take;
            slba += take;
            n -= take;
        }
    }
    if (open && cur.nlb) {
        out.push_back(cur);
    }

    uint64_t acc = 0;
    for (size_t i = 0; i < out.size(); i++) {
        out[i].base_in_win = acc;
        acc += out[i].nlb;
    }
}

/* 窗口划分：整窗 = P × per 个 LBA；零头单独成一个 P=1 的窗口 */
struct Win {
    int      start;
    int      cnt;
    uint64_t base_nlb;
    uint64_t n;
};

static void build_wins(const NvmeCopyFormat2 *cf, int nr_cf, uint64_t win_nlb,
                       std::vector<Win> &out)
{
    uint64_t cur = 0;

    for (int i = 0; i < nr_cf; ) {
        const int start = i;
        const uint64_t base = cur;
        uint64_t n = 0;

        while (i < nr_cf) {
            uint64_t nlb = (uint64_t)cf[i].nlb + 1;
            if (n && n + nlb > win_nlb) {
                break;
            }
            n += nlb;
            i++;
        }
        if (n == 0) {
            break;
        }
        Win w;
        w.start = start; w.cnt = i - start; w.base_nlb = base; w.n = n;
        out.push_back(w);
        cur += n;
    }
}

static bool copy_range2(int dst_fd, int src_fd, uint64_t dst_off,
                        uint64_t src_off, uint64_t len)
{
    off_t in = (off_t)src_off, out = (off_t)dst_off;
    uint64_t done = 0;

    while (done < len) {
        /*
         * 单次 copy_file_range 的请求长度：1MB，不要给 1GB。
         * 实测（冷盘，10MB，ext4 on FEMU）：
         *   chunk=1GB   142 ms  74 MB/s
         *   chunk=1MB   113 ms  93 MB/s
         *   chunk=256KB 114 ms  92 MB/s
         * 大请求会掉进慢路径（纯读也是 4MB 块 168ms vs 1MB 块 41ms，差 4 倍），
         * 而 1MB 已经贴到设备读(254MB/s)+写(151MB/s)的带宽下限了。
         */
        size_t chunk = (size_t)std::min<uint64_t>(1u << 20, len - done);
        ssize_t r = copy_file_range(src_fd, &in, dst_fd, &out, chunk, 0);
        if (r > 0) {
            done += (uint64_t)r;
            continue;
        }
        break;
    }
    if (done >= len) {
        return true;
    }

    static std::vector<uint8_t> buf;
    if (buf.empty()) {
        buf.resize(4u << 20);
    }
    while (done < len) {
        size_t chunk = (size_t)std::min<uint64_t>(buf.size(), len - done);
        ssize_t r = pread(src_fd, buf.data(), chunk, (off_t)(src_off + done));
        if (r != (ssize_t)chunk) {
            return false;
        }
        if (pwrite(dst_fd, buf.data(), chunk, (off_t)(dst_off + done)) != r) {
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
    g_t0_prog = now_ms();

    std::string old_path, new_path, tgt_path;
    uint64_t win_mb = DEF_WIN_MB;
    int client_dev = 0, server_dev = 1;
    int P = DEF_P;
    bool verify = true;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--no-verify")) {
            verify = false;
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            tgt_path = argv[++i];
        } else if (!strcmp(argv[i], "-w") && i + 1 < argc) {
            win_mb = strtoull(argv[++i], nullptr, 10);
        } else if (!strcmp(argv[i], "-p") && i + 1 < argc) {
            P = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
            client_dev = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-s") && i + 1 < argc) {
            server_dev = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("用法: %s <server:f> <client:f'> [-o target] [-w MB] [-p P] [-c N] [-s N]\n",
                   argv[0]);
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
        fprintf(stderr, "用法: %s <server:f> <client:f'> [-o target] [-w MB] [-p P] [-c N] [-s N]\n",
                argv[0]);
        return 1;
    }
    if (client_dev == server_dev) {
        fprintf(stderr, "client 和 server 必须是两个不同的 CSD（-c / -s）。\n");
        return 1;
    }
    if (P < 1) {
        P = 1;
    }
    if (3 * P > MAX_MRS) {
        fprintf(stderr, "-p %d 需要 %d 个 MR，超过上限 %d\n", P, 3 * P, MAX_MRS);
        return 1;
    }
    if (tgt_path.empty()) {
        tgt_path = new_path + ".gridpipe";
    }
    if (win_mb < MIN_WIN_MB) {
        win_mb = MIN_WIN_MB;
    }

    /* ---------- 0. stat ---------- */
    uint64_t old_size = 0, new_size = 0;
    {
        PhaseScope ph("stat 两个文件", PH_HOST);
        struct stat st {};
        if (stat(old_path.c_str(), &st) < 0) {
            fprintf(stderr, "stat %s: %s\n", old_path.c_str(), strerror(errno));
            return 1;
        }
        old_size = (uint64_t)st.st_size;
        if (stat(new_path.c_str(), &st) < 0) {
            fprintf(stderr, "stat %s: %s\n", new_path.c_str(), strerror(errno));
            return 1;
        }
        new_size = (uint64_t)st.st_size;
    }
    if (!old_size || !new_size) {
        fprintf(stderr, "文件为空\n");
        return 1;
    }

    const uint64_t old_nlb = (old_size + SEC - 1) / SEC;
    const uint64_t new_nlb = (new_size + SEC - 1) / SEC;
    const uint64_t grid_s_pages = (old_size + PAGE - 1) / PAGE;
    const uint64_t grid_c_pages = (new_size + PAGE - 1) / PAGE;
    const uint64_t grid_s_io = round_up_4k(grid_s_pages * CSD_CRC_SLOT_LEN);
    const uint64_t grid_c_io = round_up_4k(grid_c_pages * CSD_CRC_SLOT_LEN);

    printf("=== grid-cdc 流水线版：双 CSD，6 步流程，nr_concurrent_chunks=%d ===\n", P);
    printf("server CSD (nvme%d) 持有 f    %s  (%llu B)\n",
           server_dev, old_path.c_str(), (unsigned long long)old_size);
    printf("client CSD (nvme%d) 持有 f'   %s  (%llu B)\n",
           client_dev, new_path.c_str(), (unsigned long long)new_size);

    /* ---------- 1. FIEMAP ---------- */
    NvmeCopyFormat2 *old_cf = nullptr, *new_cf = nullptr;
    int nr_old_cf = 0, nr_new_cf = 0;
    {
        PhaseScope ph("FIEMAP 物理映射", PH_HOST);
        int nlb_pc = 0, ncf_pc = 0;
        uint64_t fsz = 0;
        nr_old_cf = get_file_mappings(old_path.c_str(), &fsz, &old_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_old_cf <= 0) {
            fprintf(stderr, "FIEMAP f 失败（是否在 /mnt/nvme%d 上？）\n", server_dev);
            return 1;
        }
        nlb_pc = 0; ncf_pc = 0;
        fsz = 0;
        nr_new_cf = get_file_mappings(new_path.c_str(), &fsz, &new_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_new_cf <= 0) {
            fprintf(stderr, "FIEMAP f' 失败（是否在 /mnt/nvme%d 上？）\n", client_dev);
            return 1;
        }
    }

    /* ---------- 2. 打开设备 ---------- */
    Dev cli, srv;
    if (!dev_open(&cli, client_dev) || !dev_open(&srv, server_dev)) {
        return 1;
    }

    /* ---------- 3. 装载 CSF ---------- */
    uint16_t pind_gd_c = 0, pind_gd_s = 0;
    uint16_t pind_collect_c = 0, pind_collect_s = 0;
    {
        PhaseScope ph("装载 CSF", PH_HOST);
        if (load_csf(cli.ctl_fd, "./build/grid_pipe.so", "grid_pipe",
                     "gridpipe_c_v1", &pind_gd_c) < 0 ||
            load_csf(cli.ctl_fd, "./build/collect_oob.so", "collect_oob",
                     "gpcollect_c_v1", &pind_collect_c) < 0 ||
            load_csf(srv.ctl_fd, "./build/grid_pipe.so", "grid_pipe",
                     "gridpipe_s_v1", &pind_gd_s) < 0 ||
            load_csf(srv.ctl_fd, "./build/collect_oob.so", "collect_oob",
                     "gpcollect_s_v1", &pind_collect_s) < 0) {
            return 1;
        }
    }

    /* ---------- 4. FDM 准备 ---------- */
    uint64_t win_bytes = win_mb << 20;
    int fd_win_c = -1, fd_win_s = -1;
    int fd_tab_c = -1, io_tab_c = -1;
    int fd_pair_s = -1, io_pair_s = -1;
    int fd_strong_c = -1, io_strong_c = -1;
    int fd_desc_c = -1;
    int fd_grid_c = -1, fd_grid_s = -1;
    uint64_t mf_win_io = 0, mf_win_io_s = 0;
    uint64_t per_c = 0, per_s = 0, win_nlb_c = 0, win_nlb_s = 0;

    {
        PhaseScope ph("FDM 准备", PH_HOST);

        while (win_bytes >= (MIN_WIN_MB << 20)) {
            fd_win_c = ensure_fdm(cli.fdm_dir, 1, win_bytes, "client 窗口缓冲", nullptr);
            if (fd_win_c >= 0) {
                break;
            }
            win_bytes >>= 1;
        }
        if (fd_win_c < 0) {
            return 1;
        }
        if (win_bytes != (win_mb << 20)) {
            printf("窗口缓冲缩到 %llu MB（FDM 容量限制）\n",
                   (unsigned long long)(win_bytes >> 20));
        }
        fd_win_s = ensure_fdm(srv.fdm_dir, 1, win_bytes, "server 窗口缓冲", nullptr);

        /*
         * 每个 chunk 的 LBA 数：窗口 / P，向下取到 4KB 对齐。
         * 窗口必须按**实际文件大小**收，不能死用 win_bytes ——
         * 否则文件比窗口小时整窗都被划成一个窗口、又不足 P×per，
         * P 会退化成 1（流水线白开）。两侧文件大小不同，所以各算各的。
         */
        const uint64_t cap_nlb = win_bytes / SEC;

        per_c = align_down_512k(std::min(cap_nlb, new_nlb) / (uint64_t)P);
        per_s = align_down_512k(std::min(cap_nlb, old_nlb) / (uint64_t)P);
        if (per_c < 1024 || per_s < 1024) {
            fprintf(stderr, "-p %d 相对文件大小太大，每 chunk 不足 512KB\n", P);
            return 1;
        }
        win_nlb_c = per_c * (uint64_t)P;
        win_nlb_s = per_s * (uint64_t)P;

        /* 单 chunk 输出槽：按"每 4KB 一块"估块数上界 */
        const uint64_t ent_cap_c = per_c * SEC / CHUNK_MIN_SIZE + 2;
        const uint64_t ent_cap_s = per_s * SEC / CHUNK_MIN_SIZE + 2;
        mf_win_io   = round_up_4k(GD_HDR_BYTES + ent_cap_c * GD_ENT_BYTES);
        mf_win_io_s = round_up_4k(GD_HDR_BYTES + ent_cap_s * GD_PAIR_BYTES);

        fd_tab_c   = ensure_fdm(cli.fdm_dir, 0, mf_win_io * P,   "client 清单输出", &io_tab_c);
        fd_pair_s  = ensure_fdm(srv.fdm_dir, 0, mf_win_io_s * P, "server 强哈希输出", &io_pair_s);
        fd_grid_c  = ensure_fdm(cli.fdm_dir, 2, grid_c_io,   "f' 网格", nullptr);
        fd_grid_s  = ensure_fdm(srv.fdm_dir, 2, grid_s_io,   "f 网格", nullptr);

        if (fd_win_s < 0 || fd_tab_c < 0 || fd_pair_s < 0 ||
            fd_grid_c < 0 || fd_grid_s < 0) {
            return 1;
        }
    }

    printf("窗口 f' %llu LBA = %d chunk × %llu LBA；f %llu LBA = %d chunk × %llu LBA\n",
           (unsigned long long)win_nlb_c, P, (unsigned long long)per_c,
           (unsigned long long)win_nlb_s, P, (unsigned long long)per_s);

    std::vector<Win> new_wins, old_wins;
    build_wins(new_cf, nr_new_cf, win_nlb_c, new_wins);
    build_wins(old_cf, nr_old_cf, win_nlb_s, old_wins);
    {
        uint64_t cn = 0, on = 0;
        for (size_t i = 0; i < new_wins.size(); i++) { cn += new_wins[i].n; }
        for (size_t i = 0; i < old_wins.size(); i++) { on += old_wins[i].n; }
        if (cn != new_nlb || on != old_nlb) {
            fprintf(stderr, "窗口划分未覆盖全文件: f' %llu/%llu, f %llu/%llu LBA\n",
                    (unsigned long long)cn, (unsigned long long)new_nlb,
                    (unsigned long long)on, (unsigned long long)old_nlb);
            return 1;
        }
    }

    /* ---------- Stage-A：两侧各自浅读自己的 OOB 网格（P=1，与 gridds 同口径） ---------- */
    {
        struct { Dev *d; int fd_grid; uint64_t grid_io; const NvmeCopyFormat2 *cf;
                 int nr_cf; uint64_t nlb; uint16_t pind; const char *tag; } jobs[2] = {
            { &cli, fd_grid_c, grid_c_io, new_cf, nr_new_cf, new_nlb, pind_collect_c, "网格装载 (client)" },
            { &srv, fd_grid_s, grid_s_io, old_cf, nr_old_cf, old_nlb, pind_collect_s, "网格装载 (server)" },
        };

        for (int k = 0; k < 2; k++) {
            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 2);
            mrs.fd[0] = jobs[k].fd_grid;  mrs.off[0] = 0; mrs.size[0] = (long long)jobs[k].grid_io;
            mrs.fd[1] = (k == 0) ? fd_win_c : fd_win_s;
            mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            if (ioctl(jobs[k].d->ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(网格)");
                return 1;
            }
            std::vector<int> pc(1, jobs[k].nr_cf);
            uint64_t gb = jobs[k].nlb * SEC / PAGE * CSD_CRC_SLOT_LEN;
            {
                PhaseScope ph(jobs[k].tag, PH_DEVICE, &gb);
                long long r = -1;
                if (run_indirect(jobs[k].d->ng_fd, jobs[k].pind, mrs.rsid, 0, 0,
                                 CSD_READ_MODE_OOB, 0, jobs[k].cf, jobs[k].nr_cf,
                                 pc.data(), 1, (uint32_t)(jobs[k].nlb - 1),
                                 nullptr, 0, &r) < 0) {
                    fprintf(stderr, "[%s] 网格装载失败\n", jobs[k].tag);
                    return 1;
                }
            }
            struct ioctl_create_mrs dm {};
            dm.rsid = mrs.rsid;
            ioctl(jobs[k].d->ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
        }
    }

    /* ==================================================================
     * [步骤 1+2] client：f' 读入 CSD 内存 -> CDC 切分 -> 网格组合出 weak
     * ================================================================== */
    std::vector<uint8_t> cli_tab;
    {
        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io * P);
        if (!rbuf || !zbuf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }

        std::vector<uint8_t> acc;
        bool bad = false;

        for (size_t w = 0; w < new_wins.size() && !bad; w++) {
            const Win &win = new_wins[w];

            /* 本窗口实际能开几个 chunk：零头窗口用 1 个（保证 chunk 长度精确） */
            std::vector<Chunk> ch;
            if (win.n == win_nlb_c) {
                split_win(new_cf, win.start, win.cnt, per_c, ch);
            }
            if (ch.size() != (size_t)P) {
                ch.clear();
                split_win(new_cf, win.start, win.cnt, win.n, ch);
            }
            const int Pw = (int)ch.size();
            const uint64_t chunk_nlb = ch[0].nlb;

            {
                PhaseScope ph("FDM 清零 (client)", PH_HOST);
                if (pwrite(io_tab_c, zbuf, (size_t)(mf_win_io * P), 0) !=
                    (ssize_t)(mf_win_io * P)) {
                    perror("清零清单区");
                    bad = true;
                    break;
                }
            }

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3 * Pw);
            for (int i = 0; i < Pw; i++) {
                const uint64_t base_byte = (win.base_nlb + ch[i].base_in_win) * SEC;

                mrs.fd[3*i + 0] = fd_tab_c;
                mrs.off[3*i + 0] = (long long)(i * mf_win_io);
                mrs.size[3*i + 0] = (long long)mf_win_io;

                mrs.fd[3*i + 1] = fd_win_c;
                mrs.off[3*i + 1] = (long long)(ch[i].base_in_win * SEC);
                mrs.size[3*i + 1] = (long long)round_up_4k(chunk_nlb * SEC);

                mrs.fd[3*i + 2] = fd_grid_c;
                mrs.off[3*i + 2] = (long long)((base_byte / PAGE) * CSD_CRC_SLOT_LEN);
                mrs.size[3*i + 2] =
                    (long long)round_up_4k(((chunk_nlb * SEC + PAGE - 1) / PAGE + 1)
                                           * CSD_CRC_SLOT_LEN);
            }
            if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(client CDC)");
                bad = true;
                break;
            }

            /* 把各 chunk 的 cf 摊平成一个连续数组，并记录每 chunk 条数 */
            std::vector<NvmeCopyFormat2> all_cf;
            std::vector<int> ncf_pc(Pw, 0);
            for (int i = 0; i < Pw; i++) {
                ncf_pc[i] = (int)ch[i].cf.size();
                all_cf.insert(all_cf.end(), ch[i].cf.begin(), ch[i].cf.end());
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("CDC+网格弱哈希 (client)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(cli.ng_fd, pind_gd_c, mrs.rsid,
                                 chunk_nlb * SEC, 0,
                                 CSD_READ_MODE_DATA, 0,
                                 all_cf.data(), (int)all_cf.size(),
                                 ncf_pc.data(), Pw,
                                 (uint32_t)(chunk_nlb - 1), nullptr, 0, &r) < 0) {
                    fprintf(stderr, "[client] 窗口 #%zu 命令失败\n", w);
                    bad = true;
                }
            }
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(cli.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }
            if (bad) {
                break;
            }

            /* 回读各 chunk 的输出，并把相对偏移回填成绝对偏移 */
            for (int i = 0; i < Pw; i++) {
                PhaseScope ph("FDM 回读 (client)", PH_HOST);
                if (pread(io_tab_c, rbuf, (size_t)mf_win_io,
                          (off_t)(i * mf_win_io)) != (ssize_t)mf_win_io) {
                    perror("pread 清单");
                    bad = true;
                    break;
                }
                const struct gd_tab_hdr *h = (const struct gd_tab_hdr *)(const void *)rbuf;
                uint64_t nr_w = (h->magic == GD_TAB_MAGIC) ? h->nr : 0;
                if (h->entry_bytes != (uint32_t)GD_ENT_BYTES) {
                    fprintf(stderr, "[client] chunk #%d entry_bytes=%u 与预期 %llu 不符\n",
                            i, h->entry_bytes, (unsigned long long)GD_ENT_BYTES);
                    bad = true;
                    break;
                }
                if (GD_HDR_BYTES + nr_w * GD_ENT_BYTES > mf_win_io) {
                    fprintf(stderr, "[client] chunk #%d 条数越界 %llu\n",
                            i, (unsigned long long)nr_w);
                    bad = true;
                    break;
                }
                const uint64_t base_byte = (win.base_nlb + ch[i].base_in_win) * SEC;
                const uint8_t *src = rbuf + GD_HDR_BYTES;
                for (uint64_t k = 0; k < nr_w; k++) {
                    const struct gd_ent *e =
                        (const struct gd_ent *)(const void *)(src + k * GD_ENT_BYTES);
                    struct gd_ent a = *e;
                    a.off += base_byte;            /* 回填 chunk 基址 */
                    acc.insert(acc.end(), (uint8_t *)&a,
                               (uint8_t *)&a + GD_ENT_BYTES);
                }
            }
        }
        free(rbuf);
        free(zbuf);
        if (bad) {
            return 1;
        }

        cli_tab.assign(GD_HDR_BYTES, 0);
        cli_tab.insert(cli_tab.end(), acc.begin(), acc.end());
        {
            struct gd_tab_hdr *h = (struct gd_tab_hdr *)(void *)cli_tab.data();
            h->magic       = GD_TAB_MAGIC;
            h->nr          = (uint32_t)(cli_tab.size() > GD_HDR_BYTES
                                        ? (cli_tab.size() - GD_HDR_BYTES) / GD_ENT_BYTES : 0);
            h->entry_bytes = (uint32_t)GD_ENT_BYTES;
            h->total_len   = new_size;
            h->build_id    = GD_BUILD_ID;
            h->reserved    = 0;
        }
    }
    const struct gd_tab_hdr *cli_h = (const struct gd_tab_hdr *)(const void *)cli_tab.data();
    const uint64_t n_cli_chunks = cli_h->nr;
    const uint64_t manifest_bytes = cli_tab.size();
    printf("[1][2] client CSD CDC+网格弱哈希 : %llu 块, weak 清单 %llu B (%.1f KB/GB)\n",
           (unsigned long long)n_cli_chunks, (unsigned long long)manifest_bytes,
           (double)manifest_bytes / ((double)new_size / (1ull << 30)) / (1 << 10));
    printf("    设备内算子 build_id = %u（主机侧为 %u，不一致说明 .so 没换掉）\n",
           cli_h->build_id, (unsigned)GD_BUILD_ID);

    /* ---------- [步骤 3] 线上：weak 清单 client -> server ---------- */
    {
        uint64_t b = manifest_bytes;
        PhaseScope ph("线上 weak 清单 -> server", PH_WIRE, &b);
    }
    printf("[3] 线上 weak 清单 -> server  : %llu B\n", (unsigned long long)manifest_bytes);

    /* ==================================================================
     * [步骤 4+5] server：CDC 分块 -> 网格 weak -> 与清单比对 -> 只对命中算强哈希
     * ================================================================== */
    std::vector<uint8_t> srv_tab;
    {
        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io_s);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io_s * P);
        if (!rbuf || !zbuf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }

        std::vector<uint8_t> acc;
        bool bad = false;

        for (size_t w = 0; w < old_wins.size() && !bad; w++) {
            const Win &win = old_wins[w];

            std::vector<Chunk> ch;
            if (win.n == win_nlb_s) {
                split_win(old_cf, win.start, win.cnt, per_s, ch);
            }
            if (ch.size() != (size_t)P) {
                ch.clear();
                split_win(old_cf, win.start, win.cnt, win.n, ch);
            }
            const int Pw = (int)ch.size();
            const uint64_t chunk_nlb = ch[0].nlb;

            {
                PhaseScope ph("FDM 清零 (server)", PH_HOST);
                if (pwrite(io_pair_s, zbuf, (size_t)(mf_win_io_s * P), 0) !=
                    (ssize_t)(mf_win_io_s * P)) {
                    perror("清零强哈希区");
                    bad = true;
                    break;
                }
            }

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3 * Pw);
            for (int i = 0; i < Pw; i++) {
                const uint64_t base_byte = (win.base_nlb + ch[i].base_in_win) * SEC;

                mrs.fd[3*i + 0] = fd_pair_s;
                mrs.off[3*i + 0] = (long long)(i * mf_win_io_s);
                mrs.size[3*i + 0] = (long long)mf_win_io_s;

                mrs.fd[3*i + 1] = fd_win_s;
                mrs.off[3*i + 1] = (long long)(ch[i].base_in_win * SEC);
                mrs.size[3*i + 1] = (long long)round_up_4k(chunk_nlb * SEC);

                mrs.fd[3*i + 2] = fd_grid_s;
                mrs.off[3*i + 2] = (long long)((base_byte / PAGE) * CSD_CRC_SLOT_LEN);
                mrs.size[3*i + 2] =
                    (long long)round_up_4k(((chunk_nlb * SEC + PAGE - 1) / PAGE + 1)
                                           * CSD_CRC_SLOT_LEN);
            }
            if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(server CDC)");
                bad = true;
                break;
            }

            std::vector<NvmeCopyFormat2> all_cf;
            std::vector<int> ncf_pc(Pw, 0);
            for (int i = 0; i < Pw; i++) {
                ncf_pc[i] = (int)ch[i].cf.size();
                all_cf.insert(all_cf.end(), ch[i].cf.begin(), ch[i].cf.end());
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("CDC+weak 比对+强哈希 (server)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(srv.ng_fd, pind_gd_s, mrs.rsid,
                                 chunk_nlb * SEC, 0,
                                 CSD_READ_MODE_DATA, 0,
                                 all_cf.data(), (int)all_cf.size(),
                                 ncf_pc.data(), Pw,
                                 (uint32_t)(chunk_nlb - 1),
                                 cli_tab.data(), cli_tab.size(), &r) < 0) {
                    fprintf(stderr, "[server] 窗口 #%zu 命令失败\n", w);
                    bad = true;
                }
            }
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(srv.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }
            if (bad) {
                break;
            }

            for (int i = 0; i < Pw; i++) {
                PhaseScope ph("FDM 回读 (server)", PH_HOST);
                if (pread(io_pair_s, rbuf, (size_t)mf_win_io_s,
                          (off_t)(i * mf_win_io_s)) != (ssize_t)mf_win_io_s) {
                    perror("pread 强哈希表");
                    bad = true;
                    break;
                }
                const struct gd_tab_hdr *h = (const struct gd_tab_hdr *)(const void *)rbuf;
                uint64_t nr_w = (h->magic == GD_TAB_MAGIC) ? h->nr : 0;
                if (h->entry_bytes != (uint32_t)GD_PAIR_BYTES) {
                    fprintf(stderr, "[server] chunk #%d entry_bytes=%u 与预期 %llu 不符\n",
                            i, h->entry_bytes, (unsigned long long)GD_PAIR_BYTES);
                    bad = true;
                    break;
                }
                if (GD_HDR_BYTES + nr_w * GD_PAIR_BYTES > mf_win_io_s) {
                    fprintf(stderr, "[server] chunk #%d 条数越界 %llu\n",
                            i, (unsigned long long)nr_w);
                    bad = true;
                    break;
                }
                const uint64_t base_byte = (win.base_nlb + ch[i].base_in_win) * SEC;
                const uint8_t *src = rbuf + GD_HDR_BYTES;
                for (uint64_t k = 0; k < nr_w; k++) {
                    const struct gd_pair *p =
                        (const struct gd_pair *)(const void *)(src + k * GD_PAIR_BYTES);
                    struct gd_pair a = *p;
                    a.old_off += base_byte;        /* 回填 f 侧基址 */
                    acc.insert(acc.end(), (uint8_t *)&a,
                               (uint8_t *)&a + GD_PAIR_BYTES);
                }
            }
        }
        free(rbuf);
        free(zbuf);
        if (bad) {
            return 1;
        }

        srv_tab.assign(GD_HDR_BYTES, 0);
        srv_tab.insert(srv_tab.end(), acc.begin(), acc.end());
        {
            struct gd_tab_hdr *h = (struct gd_tab_hdr *)(void *)srv_tab.data();
            h->magic       = GD_TAB_MAGIC;
            h->nr          = (uint32_t)(srv_tab.size() > GD_HDR_BYTES
                                        ? (srv_tab.size() - GD_HDR_BYTES) / GD_PAIR_BYTES : 0);
            h->entry_bytes = (uint32_t)GD_PAIR_BYTES;
            h->total_len   = old_size;
            h->build_id    = GD_BUILD_ID;
            h->reserved    = 0;
        }
    }
    const uint64_t n_hit =
        ((const struct gd_tab_hdr *)(const void *)srv_tab.data())->nr;
    const uint64_t strong_bytes = srv_tab.size();
    printf("[4][5] server CDC+weak 比对      : 弱哈希命中 %llu 块，只对这些块算了 SHA1\n",
           (unsigned long long)n_hit);

    {
        uint64_t b = strong_bytes;
        PhaseScope ph("线上 强哈希列表 -> client", PH_WIRE, &b);
    }
    printf("[5] 线上 强哈希列表 -> client : %llu 条 / %llu B\n",
           (unsigned long long)n_hit, (unsigned long long)strong_bytes);

    /* ==================================================================
     * [步骤 6] client CSD：算出那几块的强校验和 -> 比对 -> delta
     * ================================================================== */
    const struct gd_pair *pairs =
        (const struct gd_pair *)(const void *)(srv_tab.data() + GD_HDR_BYTES);

    std::vector<uint8_t> qtab;
    qtab.assign(GD_QHDR_BYTES + n_hit * GD_Q_BYTES, 0);
    {
        struct gd_query_hdr *qh = (struct gd_query_hdr *)(void *)qtab.data();
        qh->magic = GD_QRY_MAGIC;
        qh->nr = n_hit;
        for (uint64_t i = 0; i < n_hit; i++) {
            struct gd_query *q =
                (struct gd_query *)(void *)(qtab.data() + GD_QHDR_BYTES +
                                            i * GD_Q_BYTES);
            q->off = pairs[i].new_off;
            q->len = pairs[i].len;
            q->idx = (uint32_t)i;
            q->reserved = 0;
        }
    }

    std::vector<struct gd_ent> cli_strong(n_hit);
    const uint64_t strong_c_io = round_up_4k(GD_HDR_BYTES + n_hit * GD_ENT_BYTES);

    if (n_hit) {
        {
            PhaseScope ph("FDM 准备 (client 强哈希)", PH_HOST);
            fd_strong_c = ensure_fdm(cli.fdm_dir, 3, strong_c_io * P,
                                     "client 强哈希输出", &io_strong_c);
            fd_desc_c   = ensure_fdm(cli.fdm_dir, 4, PAGE * P,
                                     "client chunk 描述符", nullptr);
            if (fd_strong_c < 0 || fd_desc_c < 0) {
                return 1;
            }
            uint8_t *zero = (uint8_t *)fdm_alloc(strong_c_io * P);
            if (!zero) {
                return 1;
            }
            if (pwrite(io_strong_c, zero, (size_t)(strong_c_io * P), 0) !=
                (ssize_t)(strong_c_io * P)) {
                perror("清零 client 强哈希区");
                free(zero);
                return 1;
            }
            free(zero);
        }

        bool bad = false;
        for (size_t w = 0; w < new_wins.size() && !bad; w++) {
            const Win &win = new_wins[w];

            std::vector<Chunk> ch;
            if (win.n == win_nlb_c) {
                split_win(new_cf, win.start, win.cnt, per_c, ch);
            }
            if (ch.size() != (size_t)P) {
                ch.clear();
                split_win(new_cf, win.start, win.cnt, win.n, ch);
            }
            const int Pw = (int)ch.size();
            const uint64_t chunk_nlb = ch[0].nlb;

            /*
             * per-chunk 描述符：告诉 CSF 自己这一段在 f' 里的基址。
             * fdmfs 走 iomap_dio_rw，写入长度必须是 4KB 整数倍、缓冲 4KB 对齐，
             * 所以只能整页写（写 24 字节会 EINVAL）。
             */
            uint8_t *dpage = (uint8_t *)fdm_alloc(PAGE);
            if (!dpage) {
                fprintf(stderr, "分配描述符页失败\n");
                return 1;
            }
            for (int i = 0; i < Pw; i++) {
                struct gd_desc d {};
                d.magic = GD_DESC_MAGIC;
                d.base = (win.base_nlb + ch[i].base_in_win) * SEC;
                d.nbytes = ch[i].nlb * SEC;
                memset(dpage, 0, PAGE);
                memcpy(dpage, &d, sizeof(d));
                if (pwrite(fd_desc_c, dpage, (size_t)PAGE, (off_t)(i * PAGE)) !=
                    (ssize_t)PAGE) {
                    perror("写 chunk 描述符");
                    free(dpage);
                    return 1;
                }
            }
            free(dpage);

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3 * Pw);
            for (int i = 0; i < Pw; i++) {
                mrs.fd[3*i + 0] = fd_strong_c;
                mrs.off[3*i + 0] = (long long)(i * strong_c_io);
                mrs.size[3*i + 0] = (long long)strong_c_io;

                mrs.fd[3*i + 1] = fd_win_c;
                mrs.off[3*i + 1] = (long long)(ch[i].base_in_win * SEC);
                mrs.size[3*i + 1] = (long long)round_up_4k(chunk_nlb * SEC);

                mrs.fd[3*i + 2] = fd_desc_c;      /* mode 3：mr[2] 是描述符 */
                mrs.off[3*i + 2] = (long long)(i * PAGE);
                mrs.size[3*i + 2] = (long long)PAGE;
            }
            if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(client 强哈希)");
                bad = true;
                break;
            }

            std::vector<NvmeCopyFormat2> all_cf;
            std::vector<int> ncf_pc(Pw, 0);
            for (int i = 0; i < Pw; i++) {
                ncf_pc[i] = (int)ch[i].cf.size();
                all_cf.insert(all_cf.end(), ch[i].cf.begin(), ch[i].cf.end());
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("补算强哈希 (client)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(cli.ng_fd, pind_gd_c, mrs.rsid,
                                 chunk_nlb * SEC, 0,
                                 CSD_READ_MODE_DATA, 0,
                                 all_cf.data(), (int)all_cf.size(),
                                 ncf_pc.data(), Pw,
                                 (uint32_t)(chunk_nlb - 1),
                                 qtab.data(), qtab.size(), &r) < 0) {
                    fprintf(stderr, "[client] 强哈希窗口 #%zu 命令失败\n", w);
                    bad = true;
                }
            }
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(cli.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }
        }
        if (bad) {
            return 1;
        }

        /* 各 chunk 按全局 idx 写，合并：只取真正写过的（len>0） */
        uint8_t *rbuf = (uint8_t *)fdm_alloc(strong_c_io);
        if (!rbuf) {
            return 1;
        }
        for (int i = 0; i < P; i++) {
            PhaseScope ph("FDM 回读 (client 强哈希)", PH_HOST);
            if (pread(io_strong_c, rbuf, (size_t)strong_c_io,
                      (off_t)(i * strong_c_io)) != (ssize_t)strong_c_io) {
                perror("pread client 强哈希");
                free(rbuf);
                return 1;
            }
            const struct gd_tab_hdr *h = (const struct gd_tab_hdr *)(const void *)rbuf;
            const uint64_t nr_w = (h->magic == GD_TAB_MAGIC) ? h->nr : 0;
            const uint8_t *src = rbuf + GD_HDR_BYTES;
            for (uint64_t k = 0; k < nr_w && k < n_hit; k++) {
                const struct gd_ent *e =
                    (const struct gd_ent *)(const void *)(src + k * GD_ENT_BYTES);
                if (e->len) {
                    cli_strong[k] = *e;
                }
            }
        }
        free(rbuf);
    }

    struct Token {
        uint64_t old_off;
        uint64_t new_off;
        uint64_t len;
    };
    std::vector<Token> tokens;
    uint64_t nr_strong_ok = 0;
    {
        PhaseScope ph("强哈希比对", PH_HOST);
        for (uint64_t i = 0; i < n_hit; i++) {
            if (cli_strong[i].len != pairs[i].len) {
                continue;
            }
            if (memcmp(cli_strong[i].strong, pairs[i].strong, CSD_SHA1_LEN) != 0) {
                continue;
            }
            Token t;
            t.old_off = pairs[i].old_off;
            t.new_off = pairs[i].new_off;
            t.len     = pairs[i].len;
            tokens.push_back(t);
            nr_strong_ok++;
        }
    }
    std::sort(tokens.begin(), tokens.end(),
              [](const Token &a, const Token &b) { return a.new_off < b.new_off; });

    printf("[6] client 补算强哈希并比对    : 弱命中 %llu 块 -> 强确认 %llu 块"
           "（淘汰 %llu 块）\n",
           (unsigned long long)n_hit, (unsigned long long)nr_strong_ok,
           (unsigned long long)(n_hit - nr_strong_ok));

    /* ---------- [步骤 6 收尾] delta -> server + server 重建 f' ---------- */
    int fd_old = -1, fd_new = -1, fd_tgt = -1;
    {
        PhaseScope ph("打开数据文件", PH_HOST);
        fd_old = open(old_path.c_str(), O_RDONLY);
        fd_new = open(new_path.c_str(), O_RDONLY);
        fd_tgt = open(tgt_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd_old < 0 || fd_new < 0 || fd_tgt < 0) {
            perror("open 数据文件");
            return 1;
        }
        if (ftruncate(fd_tgt, (off_t)new_size) != 0) {
            perror("ftruncate target");
            return 1;
        }
    }

    uint64_t reuse_bytes = 0, literal_bytes = 0;
    double t_reuse = 0, t_literal = 0;
    {
        /*
         * 先把令牌合并成连续区间，再拷。
         *
         * 命中率高时，相邻令牌在 f' 和 f 里都是连续的 —— 插入 4KB 的场景
         * 874 个令牌只被插入点切成 2 段。逐个拷就是 874 次 12KB 的
         * copy_file_range：冷盘时每次请求的固定开销约 0.4ms，合计 370ms；
         * 而一次性拷 10MB 只要 142ms。合并后请求数从 874 降到 2。
         */
        std::vector<Token> runs;
        for (size_t k = 0; k < tokens.size(); k++) {
            const uint64_t tgt = tokens[k].new_off;
            const uint64_t len = std::min<uint64_t>(tokens[k].len,
                                                    new_size > tgt ? new_size - tgt : 0);
            if (len == 0) {
                continue;
            }
            if (!runs.empty()) {
                Token &c = runs.back();
                if (tgt < c.new_off + c.len) {
                    continue;           /* 重叠令牌：前一段已覆盖，跳过 */
                }
                if (tgt == c.new_off + c.len &&
                    tokens[k].old_off == c.old_off + c.len) {
                    c.len += len;       /* f' 和 f 里都连续 → 并成一段 */
                    continue;
                }
            }
            Token r;
            r.old_off = tokens[k].old_off;
            r.new_off = tgt;
            r.len = len;
            runs.push_back(r);
        }

        uint64_t pos = 0;
        for (size_t k = 0; k < runs.size(); k++) {
            const uint64_t tgt = runs[k].new_off;
            const uint64_t len = runs[k].len;

            if (len == 0 || tgt < pos) {
                continue;
            }
            if (tgt > pos) {
                const double q0 = now_ms();
                if (!copy_range2(fd_tgt, fd_new, pos, pos, tgt - pos)) {
                    fprintf(stderr, "字面量[%llu,%llu) 失败\n",
                            (unsigned long long)pos, (unsigned long long)tgt);
                    return 1;
                }
                t_literal += now_ms() - q0;
                literal_bytes += tgt - pos;
            }
            {
                const double q0 = now_ms();
                if (!copy_range2(fd_tgt, fd_old, tgt, runs[k].old_off, len)) {
                    fprintf(stderr, "复用 f[%llu] -> f'[%llu] 失败\n",
                            (unsigned long long)runs[k].old_off,
                            (unsigned long long)tgt);
                    return 1;
                }
                t_reuse += now_ms() - q0;
                reuse_bytes += len;
            }
            pos = tgt + len;
        }
        if (pos < new_size) {
            const double q0 = now_ms();
            if (!copy_range2(fd_tgt, fd_new, pos, pos, new_size - pos)) {
                fprintf(stderr, "尾部字面量[%llu,%llu) 失败\n",
                        (unsigned long long)pos, (unsigned long long)new_size);
                return 1;
            }
            t_literal += now_ms() - q0;
            literal_bytes += new_size - pos;
        }
    }
    phase_rec("重建: 复用 f", PH_HOST, reuse_bytes, t_reuse);
    phase_rec("重建: 读 f'", PH_HOST, literal_bytes, t_literal);

    printf("[6] 线上 delta -> server       : 令牌 %llu 条 + 字面量 %llu B (%.1f%% of f')\n",
           (unsigned long long)tokens.size(), (unsigned long long)literal_bytes,
           100.0 * (double)literal_bytes / (double)new_size);
    printf("[6] server 重建 f'             : 复用 %llu B + 字面量 %llu B\n",
           (unsigned long long)reuse_bytes, (unsigned long long)literal_bytes);

    const uint64_t wire_bytes = manifest_bytes + strong_bytes + literal_bytes;
    printf("\n--- 跨界字节汇总 ---\n");
    printf("weak 清单   (client->server) %10llu B\n", (unsigned long long)manifest_bytes);
    printf("强哈希列表  (server->client) %10llu B\n", (unsigned long long)strong_bytes);
    printf("字面量      (client->server) %10llu B\n", (unsigned long long)literal_bytes);
    printf("合计                         %10llu B   直接传 f' 需要 %llu B\n",
           (unsigned long long)wire_bytes, (unsigned long long)new_size);
    printf("    命中率: %llu / %llu B (%.1f%%)\n",
           (unsigned long long)reuse_bytes, (unsigned long long)new_size,
           100.0 * (double)reuse_bytes / (double)new_size);

    const double total_ms = now_ms() - g_t0_prog;
    phase_report(total_ms);

    /* ---------- 逐字节校验（测试开销；--no-verify 跳过） ---------- */
    const double tv0 = now_ms();
    bool ok = true;
    if (verify) {
        static std::vector<uint8_t> a, b;
        a.resize(1u << 20);
        b.resize(1u << 20);
        uint64_t off = 0;
        while (off < new_size) {
            size_t n = (size_t)std::min<uint64_t>(a.size(), new_size - off);
            if (pread(fd_new, a.data(), n, (off_t)off) != (ssize_t)n ||
                pread(fd_tgt, b.data(), n, (off_t)off) != (ssize_t)n) {
                perror("校验读");
                ok = false;
                break;
            }
            if (memcmp(a.data(), b.data(), n) != 0) {
                ok = false;
                break;
            }
            off += n;
        }
    }
    const double t_verify = now_ms() - tv0;
    if (verify) {
        if (!ok) {
            printf("[FAIL] 重建结果与 f' 不一致\n");
        } else {
            printf("[PASS] 重建出的 f' 与 client 的 f' 逐字节一致\n");
        }
        printf("目标文件: %s\n", tgt_path.c_str());
        printf("[T] 逐字节校验 %.2f ms (%.1f MB/s) —— 测试开销，不计入上表\n",
               t_verify,
               t_verify > 0.0 ? (double)new_size / (t_verify / 1000.0) / 1e6 : 0.0);
    } else {
        printf("目标文件: %s\n", tgt_path.c_str());
    }

    close(fd_old); close(fd_new); close(fd_tgt);
    close(fd_win_c); close(fd_win_s);
    close(fd_tab_c); close(fd_grid_c); close(fd_grid_s);
    close(fd_pair_s);
    if (fd_strong_c >= 0) { close(fd_strong_c); }
    if (fd_desc_c >= 0)   { close(fd_desc_c); }
    close(io_tab_c); close(io_pair_s);
    if (io_strong_c >= 0) { close(io_strong_c); }
    close(cli.ctl_fd); close(cli.ng_fd);
    close(srv.ctl_fd); close(srv.ng_fd);
    return ok ? 0 : 1;
}
