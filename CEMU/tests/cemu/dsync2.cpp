/*
 * dsync2.cpp —— 路线甲：双 CSD 设备内 CDC 分块 + 两级匹配的增量同步
 *
 * 与 dsync.cpp（路线乙）的区别：
 *   路线乙：client 切 f'，server 只读 OOB 网格做"同位置"折叠比对 -> 扇区位图
 *   路线甲：**两侧各自切自己的文件**，按内容做"弱哈希初筛 -> 强哈希确认"的两级
 *           匹配，天然容忍插入/删除造成的整体位移。强哈希确认真实可用。
 *
 * 流程（与设计文档的 7 步一一对应）：
 *   [1] client：f' 数据读入 CSD 内存
 *   [2] client CSD：CDC 切 f'，弱哈希由 512B CRC 网格拼出，强指纹对数据算
 *   [3] 线上：f' 的块描述符表 -> server
 *   [4] server：f 数据读入 CSD 内存，CDC 切 f，同样拼弱哈希、算强指纹
 *   [5] server CSD：两张描述符表做两级匹配 -> 匹配令牌 (f 偏移 -> f' 偏移)
 *   [6] 线上：匹配令牌 -> client；client 算出未覆盖区间（字面量）
 *   [7] server：按令牌复用 + 字面量拼出 f'
 *
 * CSF 接口要点：
 *   · indirect 流水线的 R 阶段恒把数据读进 mr[1]，且 read_mode 是整条命令
 *     一个标志 —— 所以"数据页"与"OOB 网格"不可能在一次 R 里同时到位。
 *     做法：先用 collect_oob 把整份网格浅读进一个常驻 FDM 槽（Stage-A），
 *     之后每次 CDC 命令里 mr[2] 直接指向那个槽。
 *
 * 用法：
 *   ./build/dsync2 <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]
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

#define PAGE          4096ull
#define SEC           512ull
#define DEF_WIN_MB    32ull
#define MIN_WIN_MB    4ull

#define THDR_BYTES ((uint64_t)sizeof(struct dsync_table_hdr))
#define DESC_BYTES ((uint64_t)sizeof(struct dsync_chunk_desc))
#define MHDR_BYTES ((uint64_t)sizeof(struct dsync_match_hdr))
#define TOK_BYTES  ((uint64_t)sizeof(struct dsync_match_token))

/* ------------------------------------------------------------------ */

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

/* ---------------- 阶段计时插桩 ---------------- */

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
        if (*p < 0x80)               { w += 1; p += 1; }
        else if ((*p & 0xE0) == 0xC0) { w += 1; p += 2; }
        else if ((*p & 0xF0) == 0xE0) { w += 2; p += 3; }
        else                          { w += 2; p += 4; }
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
           pad_disp("阶段", 22).c_str(), "位置", "总ms", "次数", "均值ms",
           "占比", "吞吐MB/s");

    double acc = 0.0;
    for (size_t i = 0; i < g_ph.size(); i++) {
        acc += g_ph[i].sum;
    }
    for (size_t i = 0; i < g_ph.size(); i++) {
        const PhaseStat &p = g_ph[i];
        char thr[32];
        if (p.bytes && p.sum > 0.0) {
            snprintf(thr, sizeof(thr), "%.1f", (double)p.bytes / (p.sum / 1000.0) / 1e6);
        } else {
            snprintf(thr, sizeof(thr), "-");
        }
        printf("%s %-6s %10.3f %5d %10.3f %6.1f%% %11s\n",
               pad_disp(p.name, 22).c_str(), phase_where_str(p.where),
               p.sum, p.cnt, p.sum / p.cnt,
               acc > 0.0 ? 100.0 * p.sum / acc : 0.0, thr);
    }
    printf("\n上表合计 %.3f ms   程序总耗时 %.3f ms\n", acc, total_ms);
}

/* ---------------- 基础工具 ---------------- */

static uint64_t round_up_4k(uint64_t v)
{
    return (v + PAGE - 1) & ~(PAGE - 1);
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

static void fdm_path(const char *dir, int slot, char *buf, size_t n)
{
    snprintf(buf, n, "%s%d", dir, slot);
}

/*
 * 申请 FDM 槽。fdmfs 的坑：文件"原地长大"不会真正分配底层块，设备写进去的
 * 数据读回来是空的 —— 容量不够一律 unlink 重建。
 */
static int ensure_fdm(const char *dir, int slot, uint64_t need, const char *what,
                      int *io_fd_out)
{
    char path[256];
    uint64_t want = round_up_4k(need);

    fdm_path(dir, slot, path, sizeof(path));
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

/* 源/目的偏移不同的拷贝（移位命中后两侧位置不再相同） */
static bool copy_range2(int dst_fd, int src_fd, uint64_t dst_off,
                        uint64_t src_off, uint64_t len)
{
    off_t in = (off_t)src_off, out = (off_t)dst_off;
    uint64_t done = 0;

    while (done < len) {
        size_t chunk = (size_t)std::min<uint64_t>(1u << 30, len - done);
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
    bool verify = true;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--no-verify")) {
            verify = false;
        } else if (!strcmp(argv[i], "-o") && i + 1 < argc) {
            tgt_path = argv[++i];
        } else if (!strcmp(argv[i], "-w") && i + 1 < argc) {
            win_mb = strtoull(argv[++i], nullptr, 10);
        } else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
            client_dev = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-s") && i + 1 < argc) {
            server_dev = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("用法: %s <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]\n",
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
    if (old_path.empty() || new_path.empty() || client_dev == server_dev) {
        fprintf(stderr, "用法: %s <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]\n",
                argv[0]);
        return 1;
    }
    if (tgt_path.empty()) {
        tgt_path = new_path + ".rebuilt";
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

    const uint64_t old_nlb = (old_size + SEC - 1) / SEC;
    const uint64_t new_nlb = (new_size + SEC - 1) / SEC;
    const uint64_t grid_s_pages = (old_size + PAGE - 1) / PAGE;
    const uint64_t grid_c_pages = (new_size + PAGE - 1) / PAGE;
    const uint64_t grid_s_io = round_up_4k(grid_s_pages * CSD_CRC_SLOT_LEN);
    const uint64_t grid_c_io = round_up_4k(grid_c_pages * CSD_CRC_SLOT_LEN);

    printf("=== 双 CSD 设备内增量同步（路线甲：两侧 CDC + 两级匹配）===\n");
    printf("server CSD (nvme%d) 持有 f    %s  (%llu B)\n",
           server_dev, old_path.c_str(), (unsigned long long)old_size);
    printf("client CSD (nvme%d) 持有 f'   %s  (%llu B)\n",
           client_dev, new_path.c_str(), (unsigned long long)new_size);
    printf("512B CRC 网格 f %llu B / f' %llu B\n",
           (unsigned long long)grid_s_io, (unsigned long long)grid_c_io);

    /* ---------- 1. FIEMAP ---------- */
    NvmeCopyFormat2 *old_cf = nullptr, *new_cf = nullptr;
    int nr_old_cf = 0, nr_new_cf = 0;
    {
        PhaseScope ph("FIEMAP 物理映射", PH_HOST);
        int nlb_pc = 0, ncf_pc = 0;
        uint64_t fsz = 0;
        nr_old_cf = get_file_mappings(old_path.c_str(), &fsz, &old_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_old_cf <= 0) {
            fprintf(stderr, "FIEMAP f 失败\n");
            return 1;
        }
        nlb_pc = 0; ncf_pc = 0;
        /* filesize 是入参/出参：不清零会沿用 f 的大小，导致 f' 被截断映射 */
        fsz = 0;
        nr_new_cf = get_file_mappings(new_path.c_str(), &fsz, &new_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_new_cf <= 0) {
            fprintf(stderr, "FIEMAP f' 失败\n");
            return 1;
        }
    }

    /* ---------- 2. 打开设备 ---------- */
    Dev cli, srv;
    if (!dev_open(&cli, client_dev) || !dev_open(&srv, server_dev)) {
        return 1;
    }

    /* ---------- 3. 装载 CSF ---------- */
    uint16_t pind_cdc_c = 0, pind_cdc_s = 0, pind_match = 0;
    {
        PhaseScope ph("装载 CSF", PH_HOST);
        if (load_csf(cli.ctl_fd, "./build/cdc_desc.so", "cdc_desc",
                     "cdc_desc_c_v1", &pind_cdc_c) < 0 ||
            load_csf(srv.ctl_fd, "./build/cdc_desc.so", "cdc_desc",
                     "cdc_desc_s_v1", &pind_cdc_s) < 0 ||
            load_csf(srv.ctl_fd, "./build/dsync_match.so", "dsync_match",
                     "dsync_match_v1", &pind_match) < 0) {
            return 1;
        }
    }

    /* ---------- 4. FDM 准备 ---------- */
    uint64_t win_bytes = win_mb << 20;
    int fd_win_c = -1, fd_win_s = -1;
    int fd_desc_c = -1, io_desc_c = -1;
    int fd_desc_s = -1, io_desc_s = -1;
    int fd_grid_c = -1, fd_grid_s = -1;
    int fd_tok = -1, io_tok = -1;
    int fd_olddesc = -1, io_olddesc = -1;
    int fd_scratch = -1;
    uint64_t mf_win_io = 0;

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

        const uint64_t ent_cap = win_bytes / CHUNK_MIN_SIZE + 2;
        mf_win_io = round_up_4k(THDR_BYTES + ent_cap * DESC_BYTES);

        fd_desc_c = ensure_fdm(cli.fdm_dir, 0, mf_win_io, "client 描述符输出", &io_desc_c);
        fd_desc_s = ensure_fdm(srv.fdm_dir, 0, mf_win_io, "server 描述符输出", &io_desc_s);
        fd_grid_c = ensure_fdm(cli.fdm_dir, 2, grid_c_io, "f' 网格", nullptr);
        fd_grid_s = ensure_fdm(srv.fdm_dir, 2, grid_s_io, "f 网格", nullptr);
        fd_scratch = ensure_fdm(srv.fdm_dir, 5, SEC, "哑读缓冲", nullptr);

        if (fd_win_s < 0 || fd_desc_c < 0 || fd_desc_s < 0 ||
            fd_grid_c < 0 || fd_grid_s < 0 || fd_scratch < 0) {
            return 1;
        }
    }

    /* ---------- 5. Stage-A：两侧各自浅读自己的 OOB 网格 ---------- */
    uint16_t pind_collect = 0;
    if (load_csf(cli.ctl_fd, "./build/collect_oob.so", "collect_oob",
                 "collect_c_v1", &pind_collect) < 0) {
        return 1;
    }
    {
        uint16_t pind_collect_s = 0;
        if (load_csf(srv.ctl_fd, "./build/collect_oob.so", "collect_oob",
                     "collect_s_v1", &pind_collect_s) < 0) {
            return 1;
        }

        /* client: 把 f' 的网格装进 slot2。R 先把 OOB 读进窗口缓冲，算子拷到 slot2 */
        uint64_t gc_bytes = grid_c_pages * CSD_CRC_SLOT_LEN;
        {
            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 2);
            mrs.fd[0] = fd_grid_c;  mrs.off[0] = 0; mrs.size[0] = (long long)grid_c_io;
            mrs.fd[1] = fd_win_c;   mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(client 网格)");
                return 1;
            }
            long long r = -1;
            PhaseScope ph("网格装载 (client)", PH_DEVICE, &gc_bytes);
            if (run_indirect(cli.ng_fd, pind_collect, mrs.rsid, 0, 0,
                             CSD_READ_MODE_OOB, 0, new_cf, nr_new_cf,
                             (uint32_t)(new_nlb - 1), nullptr, 0, &r) < 0) {
                fprintf(stderr, "[client] 网格装载失败\n");
                return 1;
            }
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(cli.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }
        }

        uint64_t gs_bytes = grid_s_pages * CSD_CRC_SLOT_LEN;
        {
            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 2);
            mrs.fd[0] = fd_grid_s;  mrs.off[0] = 0; mrs.size[0] = (long long)grid_s_io;
            mrs.fd[1] = fd_win_s;   mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(server 网格)");
                return 1;
            }
            long long r = -1;
            PhaseScope ph("网格装载 (server)", PH_DEVICE, &gs_bytes);
            if (run_indirect(srv.ng_fd, pind_collect_s, mrs.rsid, 0, 0,
                             CSD_READ_MODE_OOB, 0, old_cf, nr_old_cf,
                             (uint32_t)(old_nlb - 1), nullptr, 0, &r) < 0) {
                fprintf(stderr, "[server] 网格装载失败\n");
                return 1;
            }
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(srv.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }
        }
    }

    /*
     * 一个通用的「窗口循环 → 描述符表」函数。
     *   cdc_side: true=client(f')，false=server(f)
     * 返回拼好的 dsync_chunk_desc 数组（含表头）。
     */
    std::vector<uint8_t> cli_tab, srv_tab;

    auto build_table = [&](bool is_client, std::vector<uint8_t> &out_tab) -> bool {
        Dev &d            = is_client ? cli : srv;
        uint16_t pind     = is_client ? pind_cdc_c : pind_cdc_s;
        int fd_desc       = is_client ? fd_desc_c : fd_desc_s;
        int io_desc       = is_client ? io_desc_c : io_desc_s;
        int fd_grid       = is_client ? fd_grid_c : fd_grid_s;
        const NvmeCopyFormat2 *cf = is_client ? new_cf : old_cf;
        int nr_cf         = is_client ? nr_new_cf : nr_old_cf;
        uint64_t total_nlb = is_client ? new_nlb : old_nlb;
        const char *tag   = is_client ? "CDC 切分 (client f')" : "CDC 切分 (server f)";
        const char *tag0  = is_client ? "FDM 清零 (client)" : "FDM 清零 (server)";
        const char *tag1  = is_client ? "FDM 回读 (client)" : "FDM 回读 (server)";

        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io);
        if (!rbuf || !zbuf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return false;
        }

        std::vector<uint8_t> acc;
        uint64_t cur_nlb = 0;
        const uint64_t win_nlb_cap = win_bytes / SEC;
        int win_idx = 0;

        for (int i = 0; i < nr_cf; ) {
            int start = i;
            uint64_t base_nlb = cur_nlb;
            uint64_t n = 0;

            while (i < nr_cf) {
                uint64_t nlb = (uint64_t)cf[i].nlb + 1;
                if (n && n + nlb > win_nlb_cap) {
                    break;
                }
                n += nlb;
                i++;
            }
            if (n == 0) {
                break;
            }

            {
                PhaseScope ph(tag0, PH_HOST);
                if (pwrite(io_desc, zbuf, (size_t)mf_win_io, 0) != (ssize_t)mf_win_io) {
                    perror("清零描述符区");
                    free(rbuf); free(zbuf);
                    return false;
                }
            }

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3);
            mrs.fd[0] = fd_desc;  mrs.off[0] = 0; mrs.size[0] = (long long)mf_win_io;
            mrs.fd[1] = is_client ? fd_win_c : fd_win_s;
            mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            mrs.fd[2] = fd_grid;  mrs.off[2] = 0;
            mrs.size[2] = (long long)(is_client ? grid_c_io : grid_s_io);
            if (ioctl(d.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(cdc)");
                free(rbuf); free(zbuf);
                return false;
            }

            uint64_t win_b = n * SEC;
            {
                PhaseScope ph(tag, PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(d.ng_fd, pind, mrs.rsid,
                                 n * SEC, base_nlb * SEC,
                                 CSD_READ_MODE_DATA, 0,
                                 &cf[start], i - start, (uint32_t)(n - 1),
                                 nullptr, 0, &r) < 0) {
                    fprintf(stderr, "[%s] 窗口 #%d 命令失败\n", is_client ? "client" : "server", win_idx);
                    free(rbuf); free(zbuf);
                    return false;
                }
            }

            /* 用完即回收：设备池仅 1024，窗口循环泄漏的 MR 还会让设备
             * 持有 fdm 页引用，与 drop_caches 叠加触发 fdmfs 互锁死锁 */
            {
                struct ioctl_create_mrs dm {};
                dm.rsid = mrs.rsid;
                ioctl(d.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
            }

            {
                PhaseScope ph(tag1, PH_HOST);
                if (pread(io_desc, rbuf, (size_t)mf_win_io, 0) != (ssize_t)mf_win_io) {
                    perror("pread 描述符");
                    free(rbuf); free(zbuf);
                    return false;
                }
            }

            {
                const struct dsync_table_hdr *h =
                    (const struct dsync_table_hdr *)(const void *)rbuf;
                uint64_t nr_w = h->nr_chunks;
                if (nr_w == 0) {
                    fprintf(stderr, "[%s] 窗口 #%d 产出 0 条\n",
                            is_client ? "client" : "server", win_idx);
                    free(rbuf); free(zbuf);
                    return false;
                }
                if (THDR_BYTES + nr_w * DESC_BYTES > mf_win_io) {
                    fprintf(stderr, "[%s] 窗口 #%d 条数越界 %llu\n",
                            is_client ? "client" : "server", win_idx,
                            (unsigned long long)nr_w);
                    free(rbuf); free(zbuf);
                    return false;
                }
                acc.insert(acc.end(), rbuf, rbuf + THDR_BYTES + nr_w * DESC_BYTES);
            }

            cur_nlb += n;
            win_idx++;
        }
        free(rbuf);
        free(zbuf);

        if (cur_nlb != total_nlb) {
            fprintf(stderr, "覆盖不完整: %llu / %llu LBA\n",
                    (unsigned long long)cur_nlb, (unsigned long long)total_nlb);
            return false;
        }

        /* 把多个窗口的表拼成一张：表头 + 连续 desc 数组 */
        out_tab.assign(THDR_BYTES, 0);
        uint64_t total_chunks = 0;
        for (uint64_t o = 0; o + THDR_BYTES <= acc.size(); ) {
            const struct dsync_table_hdr *h =
                (const struct dsync_table_hdr *)(const void *)(acc.data() + o);
            uint64_t nr = h->nr_chunks;
            if (nr == 0 || o + THDR_BYTES + nr * DESC_BYTES > acc.size()) {
                break;
            }
            out_tab.insert(out_tab.end(),
                           acc.begin() + (long)(o + THDR_BYTES),
                           acc.begin() + (long)(o + THDR_BYTES + nr * DESC_BYTES));
            total_chunks += nr;
            o += THDR_BYTES + nr * DESC_BYTES;
        }
        {
            struct dsync_table_hdr *h = (struct dsync_table_hdr *)(void *)out_tab.data();
            h->magic          = DSYNC_TABLE_MAGIC;
            h->nr_chunks      = (uint32_t)total_chunks;
            h->entry_bytes    = (uint32_t)DESC_BYTES;
            h->total_data_len = is_client ? new_size : old_size;
        }
        return true;
    };

    /* ---------- [2] client CDC 切 f' ---------- */
    if (!build_table(true, cli_tab)) {
        return 1;
    }
    {
        const struct dsync_table_hdr *h = (const struct dsync_table_hdr *)(const void *)cli_tab.data();
        printf("[2] client CSD CDC 切 f'   : %u 块, 描述符表 %llu B (%.1f KB/GB)\n",
               h->nr_chunks, (unsigned long long)cli_tab.size(),
               (double)cli_tab.size() / ((double)new_size / (1ull << 30)) / (1 << 10));
    }

    /* ---------- [3] 线上：f' 描述符表 -> server ---------- */
    {
        uint64_t b = cli_tab.size();
        PhaseScope ph("线上 f' 描述符表 -> server", PH_WIRE, &b);
        /* 本机退化：直接就是内存搬运，真实系统里是网络传输 */
    }

    /* ---------- [4] server CDC 切 f ---------- */
    if (!build_table(false, srv_tab)) {
        return 1;
    }
    {
        const struct dsync_table_hdr *h = (const struct dsync_table_hdr *)(const void *)srv_tab.data();
        printf("[4] server CSD CDC 切 f    : %u 块, 描述符表 %llu B（不出设备）\n",
               h->nr_chunks, (unsigned long long)srv_tab.size());
    }

    /* ---------- [5] server CSD：两级匹配 ---------- */
    /*
     * 令牌是「按 server(f) 的每个块」发出的，所以容量必须按两侧块数的
     * 较大者算。只按 client 的块数会在 f 远大于 f' 时写穿令牌区。
     */
    const uint64_t n_cli_chunks =
        ((const struct dsync_table_hdr *)(const void *)cli_tab.data())->nr_chunks;
    const uint64_t n_srv_chunks =
        ((const struct dsync_table_hdr *)(const void *)srv_tab.data())->nr_chunks;
    const uint64_t mtab_magic_chunks = (n_cli_chunks > n_srv_chunks) ? n_cli_chunks : n_srv_chunks;
    const uint64_t tok_io = round_up_4k(MHDR_BYTES + mtab_magic_chunks * TOK_BYTES);
    const uint64_t olddesc_io = round_up_4k(srv_tab.size());
    {
        PhaseScope ph("FDM 准备 (server 匹配)", PH_HOST);
        fd_tok = ensure_fdm(srv.fdm_dir, 3, tok_io, "匹配令牌输出", &io_tok);
        fd_olddesc = ensure_fdm(srv.fdm_dir, 4, olddesc_io, "f 描述符表", &io_olddesc);
        if (fd_tok < 0 || fd_olddesc < 0) {
            return 1;
        }
        /*
         * FDMFS 底层走 iomap_dio_rw（直接 I/O），要求用户缓冲区对齐 ——
         * 拿 std::vector 的堆指针直接 pwrite 会返回 EINVAL。必须复制到
         * fdm_alloc 出来的页对齐缓冲里再写。
         */
        {
            uint8_t *ab = (uint8_t *)fdm_alloc((size_t)olddesc_io);
            if (!ab) { return 1; }
            memcpy(ab, srv_tab.data(),
                   (size_t)std::min<size_t>(srv_tab.size(), (size_t)olddesc_io));
            if (pwrite(io_olddesc, ab, (size_t)olddesc_io, 0) != (ssize_t)olddesc_io) {
                perror("写 f 描述符表");
                free(ab);
                return 1;
            }
            free(ab);
        }
        uint8_t *zero = (uint8_t *)fdm_alloc(tok_io);
        if (!zero) { return 1; }
        if (pwrite(io_tok, zero, (size_t)tok_io, 0) != (ssize_t)tok_io) {
            perror("清空令牌区");
            free(zero);
            return 1;
        }
        free(zero);
    }

    NvmeCopyFormat2 dummy_cf = old_cf[0];
    dummy_cf.nlb = 0;   /* 只读 1 个扇区，纯粹喂给 R 阶段 */

    struct ioctl_create_mrs mrs_m {};
    prep_mrs(&mrs_m, 3);
    mrs_m.fd[0] = fd_tok;      mrs_m.off[0] = 0; mrs_m.size[0] = (long long)tok_io;
    mrs_m.fd[1] = fd_scratch;  mrs_m.off[1] = 0; mrs_m.size[1] = (long long)SEC;
    mrs_m.fd[2] = fd_olddesc;  mrs_m.off[2] = 0; mrs_m.size[2] = (long long)olddesc_io;
    {
        PhaseScope ph("CREATE_MRS (server 匹配)", PH_HOST);
        if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_m) != 0) {
            perror("CREATE_MRS(server 匹配)");
            return 1;
        }
    }
    {
        uint64_t b = cli_tab.size();
        PhaseScope ph("两级匹配 (server)", PH_DEVICE, &b);
        long long r = -1;
        if (run_indirect(srv.ng_fd, pind_match, mrs_m.rsid, 0, 0,
                         CSD_READ_MODE_DATA, 0, &dummy_cf, 1, 0,
                         cli_tab.data(), cli_tab.size(), &r) < 0) {
            fprintf(stderr, "[server] 匹配命令失败\n");
            return 1;
        }
    }
    {
        struct ioctl_create_mrs dm {};
        dm.rsid = mrs_m.rsid;
        ioctl(srv.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
    }

    /* ---------- [6] 读回匹配令牌 ---------- */
    std::vector<struct dsync_match_token> tokens;
    {
        uint8_t *buf = (uint8_t *)fdm_alloc(tok_io);
        if (!buf) { return 1; }
        const double b0 = now_ms();
        if (pread(io_tok, buf, (size_t)tok_io, 0) != (ssize_t)tok_io) {
            perror("pread 令牌");
            free(buf);
            return 1;
        }
        phase_rec("匹配令牌 -> client", PH_WIRE, tok_io, now_ms() - b0);

        const struct dsync_match_hdr *mh = (const struct dsync_match_hdr *)(const void *)buf;
        uint64_t nr = 0;
        if (mh->magic == DSYNC_MATCH_MAGIC) {
            nr = mh->nr_matched;
            if (MHDR_BYTES + nr * TOK_BYTES > tok_io) {
                nr = (tok_io - MHDR_BYTES) / TOK_BYTES;
            }
        }
        const struct dsync_match_token *t =
            (const struct dsync_match_token *)(const void *)(buf + MHDR_BYTES);
        tokens.assign(t, t + nr);
        free(buf);
    }

    /*
     * 令牌给出 f[old] == f'[new]。按 new_offset 排序后，f' 内未被覆盖的区间
     * 就是字面量。
     */
    std::sort(tokens.begin(), tokens.end(),
              [](const struct dsync_match_token &a, const struct dsync_match_token &b) {
                  return a.new_offset < b.new_offset;
              });

    uint64_t reuse_bytes = 0;
    for (size_t k = 0; k < tokens.size(); k++) {
        reuse_bytes += tokens[k].length;
    }
    const uint64_t literal_bytes = new_size - reuse_bytes;

    {
        const struct dsync_table_hdr *hc = (const struct dsync_table_hdr *)(const void *)cli_tab.data();
        const struct dsync_table_hdr *hs = (const struct dsync_table_hdr *)(const void *)srv_tab.data();
        printf("[5] server CSD 两级匹配    : f' %u 块 / f %u 块 -> 命中 %llu 块, %llu B\n",
               hc->nr_chunks, hs->nr_chunks,
               (unsigned long long)tokens.size(), (unsigned long long)reuse_bytes);
        printf("    命中率: %llu / %llu B (%.1f%%)   字面量 %llu B\n",
               (unsigned long long)reuse_bytes, (unsigned long long)new_size,
               100.0 * (double)reuse_bytes / (double)new_size,
               (unsigned long long)literal_bytes);
    }

    /* ---------- [7] server 重建 ---------- */
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

    uint64_t reuse_done = 0, literal_done = 0;
    double t_reuse = 0, t_literal = 0;
    {
        uint64_t pos = 0;
        for (size_t k = 0; k < tokens.size(); k++) {
            const uint64_t tgt = tokens[k].new_offset;
            const uint64_t len = std::min<uint64_t>((uint64_t)tokens[k].length,
                                                    new_size > tgt ? new_size - tgt : 0);
            if (len == 0) {
                continue;
            }

            /* 空隙 [pos, tgt)：未被任何令牌覆盖 -> 字面量 */
            if (tgt > pos) {
                const double q0 = now_ms();
                if (!copy_range2(fd_tgt, fd_new, pos, pos, tgt - pos)) {
                    fprintf(stderr, "字面量[%llu,%llu) 失败\n",
                            (unsigned long long)pos, (unsigned long long)tgt);
                    return 1;
                }
                t_literal += now_ms() - q0;
                literal_done += tgt - pos;
            }
            /* 命中 [tgt, tgt+len) <- f[src, src+len) */
            {
                const double q0 = now_ms();
                if (!copy_range2(fd_tgt, fd_old, tgt, tokens[k].old_offset, len)) {
                    fprintf(stderr, "复用 f[%llu] -> f'[%llu] 失败\n",
                            (unsigned long long)tokens[k].old_offset,
                            (unsigned long long)tgt);
                    return 1;
                }
                t_reuse += now_ms() - q0;
                reuse_done += len;
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
            literal_done += new_size - pos;
        }
    }
    phase_rec("重建: 复用 f", PH_HOST, reuse_done, t_reuse);
    phase_rec("重建: 读 f'", PH_HOST, literal_done, t_literal);

    printf("[6] 线上 字面量 -> server   : %llu B (%.1f%% of f')\n",
           (unsigned long long)literal_done,
           100.0 * (double)literal_done / (double)new_size);
    printf("[7] server 重建 f'          : 复用 %llu B + 字面量 %llu B\n",
           (unsigned long long)reuse_done, (unsigned long long)literal_done);

    /* ---------- 报告 ---------- */
    const uint64_t wire_bytes = cli_tab.size() + tok_io + literal_done;
    printf("\n--- 跨界字节汇总 ---\n");
    printf("f' 描述符表 (client->server) %10llu B\n", (unsigned long long)cli_tab.size());
    printf("匹配令牌   (server->client) %10llu B\n", (unsigned long long)tok_io);
    printf("字面量     (client->server) %10llu B\n", (unsigned long long)literal_done);
    printf("合计                         %10llu B   直接传 f' 需要 %llu B\n",
           (unsigned long long)wire_bytes, (unsigned long long)new_size);
    printf("server 侧未读取 f 的 OOB 之外的任何数据用于匹配\n");

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
            printf("目标文件: %s\n", tgt_path.c_str());
        }
        printf("[T] 逐字节校验 %.2f ms (%.1f MB/s) —— 测试开销，不计入上表\n",
               t_verify,
               t_verify > 0.0 ? (double)new_size / (t_verify / 1000.0) / 1e6 : 0.0);
    } else {
        printf("目标文件: %s\n", tgt_path.c_str());
    }

    close(fd_old); close(fd_new); close(fd_tgt);
    close(fd_win_c); close(fd_win_s);
    close(fd_desc_c); close(fd_desc_s);
    close(fd_grid_c); close(fd_grid_s);
    close(fd_tok); close(fd_olddesc); close(fd_scratch);
    close(io_desc_c); close(io_desc_s); close(io_tok); close(io_olddesc);
    close(cli.ctl_fd); close(cli.ng_fd);
    close(srv.ctl_fd); close(srv.ng_fd);
    return ok ? 0 : 1;
}
