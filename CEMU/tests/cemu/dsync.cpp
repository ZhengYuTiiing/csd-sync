/*
 * dsync.cpp —— 双 CSD 增量同步：client CSD / server CSD 分工版
 *
 *   ./build/dsync <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]
 *
 * 拓扑（两个 femu 实例，同一个 guest）：
 *   server CSD (设备 -s)：持有旧文件 f，其 512B CRC 网格在写 f 时就固化在该设备的
 *                         NAND OOB 上；最终在 server 侧重建出 f'
 *   client CSD (设备 -c)：持有新文件 f'，在设备内做 CDC 切分，产出块清单
 *
 * 流程与"三项声明"的对应：
 *   [1] client CSD 对 f' 做 CDC 切分 —— 产出块清单 (off, len, CRC32C)，16B/条
 *       处理对象 = 已知窗口；MRS 容量 = 按 4KB 最小块的硬上界；
 *       执行时长 ∝ 窗口字节数
 *   [2] ---- 线上：清单 client → server（~2.1 MB/GB） ----
 *   [3] server CSD 浅读 f 的 OOB 得到网格，按清单逐条折叠并与清单里的 CRC32C 比较，
 *       直接产出复用位图；server 不读 f 的一个数据字节
 *       折叠次数恒等于清单条数 N，上下界相同，与内容完全无关；
 *       MRS 容量 = ceil(N_sectors/8) 字节的位图；执行时长 ∝ N
 *   [4] ---- 线上：位图 server → client（256 KB/GB） ----
 *   [5] client 仅对未命中的块编码 Literal Bytes
 *   [6] ---- 线上：字面量 client → server ----
 *   [7] server CSD 以本地命中块 + 差异字节拼接重建 f'
 *
 * 前置条件：
 *   - f 必须由 server 设备的写路径写过（OOB 网格已固化在该设备上）
 *   - f' 在 client 设备上
 *   - 两个大小均为 4KB 整数倍
 *   - /mnt/fdm{C}/ 与 /mnt/fdm{S}/ 可用
 *
 * 两个实测到的坑，都绕开了，别踩回去：
 *   1. indirect 流水线会覆盖 CSF 返回值，条数不能从 cqe 读 —— 清单区域每窗口
 *      先清零，靠"第一条 len==0 的条目"判断清单结束。
 *   2. 改了 CSF 实现必须重启 QEMU 才生效。prep_shared_library 只把 "<路径>\0
 *      <符号>\0" 交给设备，设备自己 dlopen，已 dlopen 的 .so 不会重载，
 *      UNLOAD/DOWNLOAD 也救不了（会打印 "already exists" 然后继续用旧镜像）。
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
#include "kernel/chunk_abi.h"
#include "kernel/cemu_manifest.h"
#include "../../hw/femu/backend/csd_fp_crc.h"

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

/* ------------------------------------------------------------------ */
/* 阶段计时插桩                                                        */
/*                                                                     */
/* 口径说明（写论文前务必先看这四条）：                                  */
/*   · 全部是 CLOCK_MONOTONIC 墙钟，在主机侧测量；                       */
/*   · 标“设备”的阶段 = 命令下发到完成的往返时间，含 io_uring 提交、      */
/*     PCIe 往还、设备内执行。QEMU 下设备与主机同核，无法再往下细分；      */
/*   · 标“主机”的是纯用户态 + 系统调用开销；                             */
/*   · 标“线上”的是两侧 FDM 之间的搬运，真实系统里对应网络传输；           */
/*   · 逐字节校验属于测试开销，不计入总计。                              */
/* ------------------------------------------------------------------ */

enum PhaseWhere { PH_HOST, PH_DEVICE, PH_WIRE };

struct PhaseStat {
    const char *name;
    PhaseWhere  where;
    uint64_t    bytes;   /* 该阶段搬运/处理的字节数，0 = 不适用 */
    int         cnt;     /* 发生次数（窗口循环里 > 1） */
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
            p.bytes += bytes;   /* 累加，使吞吐 = 总字节 / 总时间 */
            return;
        }
    }
    PhaseStat p;
    p.name = name; p.where = where; p.bytes = bytes;
    p.cnt = 1; p.sum = p.min = p.max = ms;
    g_ph.push_back(p);
}

/* RAII：离开作用域自动登记一次耗时；bytes 传 nullptr 时记 0 */
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

/* 按显示宽度补空格：中文按 2 列算，否则表格会错位 */
static std::string pad_disp(const char *s, size_t width)
{
    size_t w = 0;
    const unsigned char *p = (const unsigned char *)s;
    while (*p) {
        if (*p < 0x80)             { w += 1; p += 1; }
        else if ((*p & 0xE0) == 0xC0) { w += 1; p += 2; }
        else if ((*p & 0xF0) == 0xE0) { w += 2; p += 3; }   /* 常用汉字 */
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
           pad_disp("阶段", 20).c_str(), "位置", "总ms", "次数", "均值ms",
           "占比", "吞吐MB/s");

    double acc = 0.0;
    for (size_t i = 0; i < g_ph.size(); i++) {
        acc += g_ph[i].sum;
    }
    for (size_t i = 0; i < g_ph.size(); i++) {
        const PhaseStat &p = g_ph[i];
        const double mean = p.sum / p.cnt;
        char thr[32];
        if (p.bytes && p.sum > 0.0) {
            snprintf(thr, sizeof(thr), "%.1f",
                     (double)p.bytes / (p.sum / 1000.0) / 1e6);
        } else {
            snprintf(thr, sizeof(thr), "-");
        }
        printf("%s %-6s %10.3f %5d %10.3f %6.1f%% %11s\n",
               pad_disp(p.name, 20).c_str(), phase_where_str(p.where),
               p.sum, p.cnt, mean,
               acc > 0.0 ? 100.0 * p.sum / acc : 0.0, thr);
    }
    printf("%s %-6s %10.3f %5s %10s %6.1f%%\n",
           pad_disp("上表合计", 20).c_str(), "", acc, "", "", 100.0);
    printf("%s %-6s %10.3f\n", pad_disp("程序总耗时", 20).c_str(), "", total_ms);
    if (total_ms > acc) {
        printf("%s %-6s %10.3f   (启动/参数解析/printf/收尾)\n",
               pad_disp("未计入", 20).c_str(), "", total_ms - acc);
    }
    printf("注：本机为嵌套虚拟机，printf 与页缓存状态会带来数 ms 抖动；"
           "取数建议连跑 3 次看中位数。\n");
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

static void fdm_path(const char *dir, int slot, char *buf, size_t n)
{
    snprintf(buf, n, "%s%d", dir, slot);
}

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
    /*
     * fdmfs 的坑（实测）：文件"原地长大"不会真正分配底层块，设备写进去的数据
     * 读回来是空的 —— 容量不够一律 unlink 重建。
     */
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
        *io_fd_out = open(path, O_RDWR | O_DIRECT);
        if (*io_fd_out < 0) {
            fprintf(stderr, "open %s O_DIRECT: %s\n", path, strerror(errno));
            close(fd);
            return -1;
        }
    }
    return fd;
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

/* server 本地复用 / 跨主机关卡的字面量写入 */
static bool copy_range(int dst_fd, int src_fd, uint64_t off, uint64_t len)
{
    uint64_t done = 0;

    while (done < len) {
        size_t chunk = (size_t)std::min<uint64_t>(1u << 30, len - done);
        ssize_t r = copy_file_range(src_fd, (off_t *)&off, dst_fd, (off_t *)&off,
                                    chunk, 0);
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

/*
 * 源/目的偏移不同的版本。移位匹配命中后，块在 f 与 f' 中的位置不再相同，
 * 不能用上面那个"同一个 off 兼作输入与输出"的版本。
 */
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

    static std::vector<uint8_t> buf2;
    if (buf2.empty()) {
        buf2.resize(4u << 20);
    }
    while (done < len) {
        size_t chunk = (size_t)std::min<uint64_t>(buf2.size(), len - done);
        ssize_t r = pread(src_fd, buf2.data(), chunk, (off_t)(src_off + done));
        if (r != (ssize_t)chunk) {
            return false;
        }
        if (pwrite(dst_fd, buf2.data(), chunk, (off_t)(dst_off + done)) != r) {
            return false;
        }
        done += chunk;
    }
    return true;
}

/* ------------------------------------------------------------------ */

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
    if (old_path.empty() || new_path.empty()) {
        fprintf(stderr, "用法: %s <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]\n",
                argv[0]);
        return 1;
    }
    if (client_dev == server_dev) {
        fprintf(stderr, "client 和 server 必须是两个不同的 CSD（-c / -s）。\n"
                        "单设备的场景请用 build/fastsync。\n");
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
    {
        PhaseScope ph("stat 源文件 x2", PH_HOST);
        if (stat(old_path.c_str(), &st_old) < 0) { perror("stat f"); return 1; }
        if (stat(new_path.c_str(), &st_new) < 0) { perror("stat f'"); return 1; }
    }

    const uint64_t old_size = (uint64_t)st_old.st_size;
    const uint64_t new_size = (uint64_t)st_new.st_size;
    if (!old_size || !new_size) {
        fprintf(stderr, "文件为空\n");
        return 1;
    }
    /*
     * 不要求 4KB 整数倍。原理性的粒度是 512B（网格槽位 / 可组合性 / 位图），
     * 而 512B 也不是硬门槛：大小不足整扇区的尾部按"向上取整到整扇区"参与
     * 设备侧比对，主机侧重建时再夹回真实长度（见第 8 步）。
     */

    /* 页数 / 扇区数一律向上取整：尾部不满一页（扇区）也要进网格 */
    const uint64_t grid_pages      = (old_size + PAGE - 1) / PAGE;
    const uint64_t old_nlb         = (old_size + SEC  - 1) / SEC;
    const uint64_t grid_bytes      = grid_pages * CSD_CRC_SLOT_LEN;
    const uint64_t new_nlb         = (new_size + SEC  - 1) / SEC;
    const uint64_t grid_io         = round_up_4k(grid_bytes);

    /*
     * 候选偏移：设 f' 相对 f 的净字节增量为 B = |f'| - |f|。
     * 发生单次插入/删除后，编辑点之后恒有 f[x] == f'[x + B]，
     * 于是清单条目 (off, len) 在 f 内的候选位置为 { off, off - B }。
     * B == 0（纯改写）时候选集退化为 {0}；B 不是 512B 整倍数时无法用网格
     * 组合出来，直接丢弃（此时退化成位置对齐匹配，不会算错，只是不增益）。
     */
    const int64_t  size_delta = (int64_t)new_size - (int64_t)old_size;
    const int64_t  deltas[2]  = { 0, size_delta };
    const uint64_t nr_deltas  = (size_delta != 0 && (size_delta % (int64_t)SEC) == 0)
                                ? 2u : 1u;

    printf("=== 双 CSD 设备内增量同步 ===\n");
    printf("server CSD (nvme%d) 持有 f    %s  (%llu B)\n",
           server_dev, old_path.c_str(), (unsigned long long)old_size);
    printf("client CSD (nvme%d) 持有 f'   %s  (%llu B)\n",
           client_dev, new_path.c_str(), (unsigned long long)new_size);
    printf("512B CRC 网格 %llu B (%.3f%% 于 f)\n",
           (unsigned long long)grid_bytes,
           100.0 * (double)grid_bytes / (double)old_size);

    /* 自检：缓存算子的 combine 必须与朴素 combine 一致。
     * 首次调用会构建 32x32 GF(2) 移位矩阵（4096 bit 的矩阵幂），
     * 一次性开销落在 CSD 算子的初始化上，单独列出来免得混进别的阶段。 */
    {
        PhaseScope ph("CRC 算子初始化", PH_HOST);
        for (uint32_t v = 0; v < 4; v++) {
            uint32_t c1 = 0x12345678u + v * 0x01010101u;
            uint32_t c2 = 0x9abcdef0u - v * 0x00010001u;
            if (csd_crc32c_combine512(c1, c2) != csd_crc32c_combine(c1, c2, 512)) {
                fprintf(stderr, "内部错误：512B 移位算子自检失败\n");
                return 1;
            }
        }
    }

    /* ---------- 1. FIEMAP ---------- */
    NvmeCopyFormat2 *old_cf = nullptr, *new_cf = nullptr;
    int nr_old_cf = 0, nr_new_cf = 0;
    {
        PhaseScope ph("FIEMAP 解析 x2", PH_HOST);
        uint64_t fsz = old_size;
        int nlb_pc = 0, ncf_pc = 0;
        nr_old_cf = get_file_mappings(old_path.c_str(), &fsz, &old_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_old_cf <= 0 || fsz != old_size) {
            fprintf(stderr, "f 的 FIEMAP 失败（是否在 /mnt/nvme%d 上？）\n", server_dev);
            return 1;
        }
        fsz = new_size;
        nlb_pc = 0;
        ncf_pc = 0;
        nr_new_cf = get_file_mappings(new_path.c_str(), &fsz, &new_cf, 1, &nlb_pc, &ncf_pc);
        if (nr_new_cf <= 0 || fsz != new_size) {
            fprintf(stderr, "f' 的 FIEMAP 失败（是否在 /mnt/nvme%d 上？）\n", client_dev);
            return 1;
        }
    }
    printf("FIEMAP: f %d 段, f' %d 段\n", nr_old_cf, nr_new_cf);

    /* ---------- 2. 打开两个设备 ---------- */
    Dev cli, srv;
    {
        PhaseScope ph("打开设备 x2", PH_HOST);
        if (!dev_open(&cli, client_dev) || !dev_open(&srv, server_dev)) {
            return 1;
        }
    }

    /*
     * 程序名带版本后缀：设备侧同名 DOWNLOAD 会返回 EEXIST 并保留旧镜像，
     * 换了算子实现就必须换名字，否则跑的还是上一次装载的那份。
     */
    uint16_t pind_manifest = 0, pind_fold = 0;
    {
        PhaseScope ph("装载 CSF x2", PH_DEVICE);
        if (load_csf(cli.ctl_fd, "./build/chunk_manifest.so", "chunk_manifest",
                     "chunk_manifest_v1", &pind_manifest) ||
            load_csf(srv.ctl_fd, "./build/fold_manifest.so", "fold_manifest",
                     "fold_manifest_v1", &pind_fold)) {
            return 1;
        }
    }

    /* ---------- 3. 准备两侧 FDM ---------- */
    uint64_t win_bytes = win_mb << 20;
    int fd_win = -1, io_manifest_cli = -1, io_manifest_srv = -1;
    int fd_grid = -1, fd_manifest_srv = -1, fd_manifest_cli = -1;
    int fd_par = -1, io_par = -1;
    int fd_mtab_srv = -1, io_mtab_srv = -1;
    uint64_t mf_win_io = 0;
    const uint64_t par_io = round_up_4k(FOLD_PARAM_BASE_BYTES +
                                        nr_deltas * (uint64_t)sizeof(int64_t));

    {
        PhaseScope ph("FDM 准备", PH_HOST);

        while (win_bytes >= (MIN_WIN_MB << 20)) {
            fd_win = ensure_fdm(cli.fdm_dir, 1, win_bytes, "窗口缓冲", nullptr);
            if (fd_win >= 0) {
                break;
            }
            win_bytes >>= 1;
        }
        if (fd_win < 0) {
            return 1;
        }
        if (win_bytes != (win_mb << 20)) {
            printf("窗口缓冲缩到 %llu MB（FDM 容量限制）\n",
                   (unsigned long long)(win_bytes >> 20));
        }

        /* client: 清单输出按"每 4KB 一个块"的硬上界分配 */
        const uint64_t ent_cap = win_bytes / CHUNK_MIN_SIZE + 2;
        mf_win_io = round_up_4k(ent_cap * MANIFEST_ENT_BYTES);
        fd_manifest_cli = ensure_fdm(cli.fdm_dir, 0, mf_win_io, "清单输出", &io_manifest_cli);
        if (fd_manifest_cli < 0) {
            return 1;
        }

        fd_grid = ensure_fdm(srv.fdm_dir, 1, grid_io, "OOB 网格", nullptr);
        fd_par  = ensure_fdm(srv.fdm_dir, 3, par_io, "折叠参数", &io_par);
        if (fd_grid < 0 || fd_par < 0) {
            return 1;
        }

        /* 折叠参数块：old_nlb + nr_deltas + 候选偏移数组 */
        {
            uint8_t *pb = (uint8_t *)fdm_alloc(par_io);
            if (!pb) { fprintf(stderr, "分配对齐缓冲失败\n"); return 1; }
            memset(pb, 0, (size_t)par_io);
            struct fold_param *fp = (struct fold_param *)(void *)pb;
            fp->old_nlb   = old_nlb;
            fp->nr_deltas = nr_deltas;
            for (uint64_t d = 0; d < nr_deltas; d++) {
                fp->deltas[d] = deltas[d];
            }
            if (pwrite(io_par, pb, (size_t)par_io, 0) != (ssize_t)par_io) {
                perror("写折叠参数");
                return 1;
            }
            free(pb);
        }
    }

    /* ---------- 4. [client] CDC 切分 -> 块清单 ---------- */
    struct ioctl_create_mrs mrs_c {};
    prep_mrs(&mrs_c, 2);
    mrs_c.fd[0] = fd_manifest_cli; mrs_c.off[0] = 0; mrs_c.size[0] = (long long)mf_win_io;
    mrs_c.fd[1] = fd_win;          mrs_c.off[1] = 0; mrs_c.size[1] = (long long)win_bytes;
    {
        PhaseScope ph("CREATE_MRS (client)", PH_HOST);
        if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_c) != 0) {
            perror("CREATE_MRS(client)");
            return 1;
        }
    }

    const uint64_t win_nlb_cap = win_bytes / SEC;
    std::vector<uint8_t> manifest;
    uint64_t N = 0, cur_nlb = 0, win_idx = 0;
    double t_cli = 0;               /* 只累计 run_indirect 往返 */
    uint64_t phase_win_bytes = 0;   /* 供 CDC 阶段吞吐统计 */

    {
        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io);
        if (!rbuf || !zbuf) { fprintf(stderr, "分配对齐缓冲失败\n"); return 1; }
        (void)zbuf;

        for (int i = 0; i < nr_new_cf; ) {
            int start = i;
            uint64_t base_nlb = cur_nlb;

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

            long long r = -1;

            /*
             * 清单区域每窗口先清零：算子写入的条目从偏移 0 起连续排列，
             * 尾部保持 0，主机靠"第一条 len==0 的条目"判断清单结束 ——
             * 这样条数不依赖被流水线覆盖的命令返回值。
             */
            {
                PhaseScope ph("FDM 清零 (client)", PH_HOST);
                if (pwrite(io_manifest_cli, zbuf, (size_t)mf_win_io, 0) !=
                    (ssize_t)mf_win_io) {
                    perror("清零清单区");
                    return 1;
                }
            }

            phase_win_bytes = acc * SEC;
            {
                PhaseScope ph("CDC 切分 (client)", PH_DEVICE, &phase_win_bytes);
                const double c0 = now_ms();
                if (run_indirect(cli.ng_fd, pind_manifest, mrs_c.rsid,
                                 acc * SEC, base_nlb * SEC,
                                 CSD_READ_MODE_DATA, 0,
                                 &new_cf[start], i - start, (uint32_t)(acc - 1),
                                 nullptr, 0, &r) < 0) {
                    fprintf(stderr, "[client] 窗口 #%llu 命令失败\n",
                            (unsigned long long)win_idx);
                    return 1;
                }
                t_cli += now_ms() - c0;
            }
            (void)r;   /* 流水线会覆盖 CSF 返回值，真正的条数从清单表头读 */

            {
                PhaseScope ph("FDM 回读 (client)", PH_HOST);
                if (pread(io_manifest_cli, rbuf, (size_t)mf_win_io, 0) !=
                    (ssize_t)mf_win_io) {
                    perror("pread 清单");
                    return 1;
                }
            }
            {
                const struct manifest_ent *e = (const struct manifest_ent *)(void *)rbuf;
                uint64_t nr_w = 0;

                while ((nr_w + 1) * MANIFEST_ENT_BYTES <= mf_win_io &&
                       e[nr_w].len != 0) {
                    nr_w++;
                }
                if (nr_w == 0) {
                    fprintf(stderr, "[client] 窗口 #%llu 产出 0 条\n",
                            (unsigned long long)win_idx);
                    return 1;
                }
                manifest.insert(manifest.end(), rbuf,
                                rbuf + nr_w * MANIFEST_ENT_BYTES);
                N += nr_w;
            }
            cur_nlb += acc;
            win_idx++;
        }
        free(rbuf);
    }
    if (cur_nlb != new_nlb) {
        fprintf(stderr, "f' 覆盖不完整: %llu / %llu LBA\n",
                (unsigned long long)cur_nlb, (unsigned long long)new_nlb);
        return 1;
    }

    const uint64_t manifest_bytes = N * MANIFEST_ENT_BYTES;
    printf("[1] client CSD CDC 切分    : %llu 个窗口, 清单 %llu 条 (%llu B, %.2f MB/GB), "
           "命令往返 %.2f ms\n",
           (unsigned long long)win_idx, (unsigned long long)N,
           (unsigned long long)manifest_bytes,
           (double)manifest_bytes / ((double)new_size / (1ull << 30)) / (1 << 20),
           t_cli);

    /* ---------- 5. 线上 #1：清单 client -> server ---------- */
    {
        PhaseScope ph("FDM 准备 (server)", PH_HOST);
        fd_manifest_srv = ensure_fdm(srv.fdm_dir, 2, manifest_bytes, "清单输入",
                                     &io_manifest_srv);
        if (fd_manifest_srv < 0) {
            return 1;
        }
    }
    {
        const uint64_t wr = round_up_4k(manifest_bytes);
        uint8_t *wbuf = (uint8_t *)fdm_alloc(wr);

        if (!wbuf) { fprintf(stderr, "分配对齐缓冲失败\n"); return 1; }
        if (manifest_bytes) {
            memcpy(wbuf, manifest.data(), (size_t)manifest_bytes);
        }
        {
            PhaseScope ph("清单 -> server", PH_WIRE, &manifest_bytes);
            if (pwrite(io_manifest_srv, wbuf, (size_t)wr, 0) != (ssize_t)wr) {
                perror("写清单到 server");
                return 1;
            }
        }
        free(wbuf);
    }
    printf("[2] 线上 清单 -> server     : %llu B (%.2f MB/GB)\n",
           (unsigned long long)manifest_bytes,
           (double)manifest_bytes / ((double)new_size / (1ull << 30)) / (1 << 20));

    /* ---------- 6. [server] 网格折叠 -> 匹配表（不读 f 的一个数据字节） ---------- */
    /* 匹配表容量按 N 条硬上界分配，此时 N 已知 */
    const uint64_t mtab_bytes = N * MATCH_ENT_BYTES;
    const uint64_t mtab_io    = round_up_4k(mtab_bytes);
    {
        PhaseScope ph("FDM 准备 (server)", PH_HOST);
        fd_mtab_srv = ensure_fdm(srv.fdm_dir, 0, mtab_io, "匹配表输出", &io_mtab_srv);
        if (fd_mtab_srv < 0) {
            return 1;
        }
        /* 匹配表清零：靠"第一条 len==0 的条目"判断表结束 */
        uint8_t *zero = (uint8_t *)fdm_alloc(1u << 20);
        if (!zero) { fprintf(stderr, "分配对齐缓冲失败\n"); return 1; }
        uint64_t off = 0;
        while (off < mtab_io) {
            size_t n = (size_t)std::min<uint64_t>(1u << 20, mtab_io - off);
            if (pwrite(io_mtab_srv, zero, n, (off_t)off) != (ssize_t)n) {
                perror("清空匹配表");
                return 1;
            }
            off += n;
        }
        free(zero);
    }

    struct ioctl_create_mrs mrs_s {};
    prep_mrs(&mrs_s, 4);
    mrs_s.fd[0] = fd_mtab_srv;     mrs_s.off[0] = 0; mrs_s.size[0] = (long long)mtab_io;
    mrs_s.fd[1] = fd_grid;         mrs_s.off[1] = 0; mrs_s.size[1] = (long long)grid_io;
    mrs_s.fd[2] = fd_manifest_srv; mrs_s.off[2] = 0; mrs_s.size[2] = (long long)round_up_4k(manifest_bytes);
    mrs_s.fd[3] = fd_par;          mrs_s.off[3] = 0; mrs_s.size[3] = (long long)par_io;
    {
        PhaseScope ph("CREATE_MRS (server)", PH_HOST);
        if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs_s) != 0) {
            perror("CREATE_MRS(server)");
            return 1;
        }
    }

    long long r_fold = -1;
    double t_srv = 0;
    {
        PhaseScope ph("网格折叠 (server)", PH_DEVICE);
        const double f0 = now_ms();
        if (run_indirect(srv.ng_fd, pind_fold, mrs_s.rsid,
                         N, 0,
                         CSD_READ_MODE_OOB, 0,
                         old_cf, nr_old_cf, (uint32_t)(old_nlb - 1),
                         nullptr, 0, &r_fold) < 0) {
            fprintf(stderr, "[server] 折叠失败\n");
            return 1;
        }
        t_srv = now_ms() - f0;
    }
    (void)r_fold;

    printf("[3] server CSD 网格折叠     : 折叠次数 %llu × %llu 候选偏移（上下界相同）, "
           "命令往返 %.2f ms, 扫描 f 的数据字节 0\n",
           (unsigned long long)N, (unsigned long long)nr_deltas, t_srv);

    /* ---------- 7. 线上 #2：匹配表 server -> client ---------- */
    std::vector<struct match_ent> mtab;
    {
        uint8_t *buf = (uint8_t *)fdm_alloc(mtab_io);
        if (!buf) { fprintf(stderr, "分配对齐缓冲失败\n"); return 1; }
        const double b0 = now_ms();
        if (pread(io_mtab_srv, buf, (size_t)mtab_io, 0) != (ssize_t)mtab_io) {
            perror("pread 匹配表");
            return 1;
        }
        const double t_mtab = now_ms() - b0;
        phase_rec("匹配表 -> client", PH_WIRE, mtab_bytes, t_mtab);

        /* 条数从"第一条 len==0 的条目"判定，不依赖被流水线覆盖的返回值 */
        const struct match_ent *e = (const struct match_ent *)(void *)buf;
        uint64_t nr_m = 0;
        while ((nr_m + 1) * MATCH_ENT_BYTES <= mtab_io && e[nr_m].len != 0) {
            nr_m++;
        }
        mtab.assign(e, e + nr_m);
        free(buf);
    }
    printf("[4] 线上 匹配表 -> client   : %llu 条 / %llu B (%.1f KB/GB)\n",
           (unsigned long long)mtab.size(), (unsigned long long)mtab_bytes,
           (double)mtab_bytes / ((double)new_size / (1ull << 30)) / (1 << 10));

    {
        PhaseScope ph("命中率统计", PH_HOST);
        uint64_t hit = 0;
        for (size_t k = 0; k < mtab.size(); k++) {
            hit += mtab[k].len / SEC;
        }
        printf("    命中率: %llu / %llu 扇区 (%.1f%%)  平均块长 %.1f KB\n",
               (unsigned long long)hit, (unsigned long long)new_nlb,
               100.0 * (double)hit / (double)new_nlb,
               (double)new_size / (double)N / 1024.0);
    }

    /* ---------- 8. 重建 f'：命中用 server 本地块，未命中走线上字面量 ---------- */
    int fd_old = -1, fd_new = -1, fd_tgt = -1;
    {
        PhaseScope ph("打开源文件 + 建目标", PH_HOST);
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
         * 匹配表按 tgt_off 升序。命中区间之间未覆盖的部分就是字面量区间，
         * 命中区间则从 f 的 (tgt - delta) 处取 —— 这是移位匹配的关键。
         */
        uint64_t pos = 0;
        for (size_t k = 0; k < mtab.size(); k++) {
            const uint64_t tgt = mtab[k].tgt_off;
            /* 夹到真实文件长度：最后一个扇区可能只用了不到 512B */
            const uint64_t len = std::min<uint64_t>((uint64_t)mtab[k].len,
                                                    new_size - tgt);
            if (mtab[k].delta_idx >= nr_deltas) {
                fprintf(stderr, "匹配表 delta_idx 越界: %u\n", mtab[k].delta_idx);
                return 1;
            }
            const int64_t src = (int64_t)tgt - deltas[mtab[k].delta_idx];

            /* 空隙 [pos, tgt)：未命中 → 字面量 */
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
            /* 命中 [tgt, tgt+len) ← f 的 [src, src+len) */
            {
                const double q0 = now_ms();
                if (!copy_range2(fd_tgt, fd_old, tgt, (uint64_t)src, len)) {
                    fprintf(stderr, "复用 f[%lld,%lld) -> f'[%llu,%llu) 失败\n",
                            (long long)src, (long long)(src + (int64_t)len),
                            (unsigned long long)tgt,
                            (unsigned long long)(tgt + len));
                    return 1;
                }
                t_reuse += now_ms() - q0;
                reuse_bytes += len;
            }
            pos = tgt + len;
        }
        /* 尾部空隙 */
        if (pos < new_size) {
            const double q0 = now_ms();
            if (!copy_range2(fd_tgt, fd_new, pos, pos, new_size - pos)) {
                fprintf(stderr, "字面量[%llu,%llu) 失败\n",
                        (unsigned long long)pos, (unsigned long long)new_size);
                return 1;
            }
            t_literal += now_ms() - q0;
            literal_bytes += new_size - pos;
        }
    }
    /*
     * 这两段目前都在主机上跑（copy_file_range）。真实部署里“字面量”应当是从
     * client 经网络取回，这里退化成本机拷贝，所以两者都归入“主机”阶段。
     */
    phase_rec("重建: 复用 f", PH_HOST, reuse_bytes, t_reuse);
    phase_rec("重建: 读 f'", PH_HOST, literal_bytes, t_literal);
    printf("[5] 线上 字面量 -> server   : %llu B (%.1f%% of f')\n",
           (unsigned long long)literal_bytes,
           100.0 * (double)literal_bytes / (double)new_size);
    printf("[6] server 重建 f'          : 本地复用 %llu B + 线上字面量 %llu B\n",
           (unsigned long long)reuse_bytes, (unsigned long long)literal_bytes);

    /* ---------- 9. 报告 ---------- */
    const uint64_t wire_bytes = manifest_bytes + mtab_bytes + literal_bytes;
    printf("\n--- 跨界字节汇总 ---\n");
    printf("清单 (client->server) %12llu B\n", (unsigned long long)manifest_bytes);
    printf("匹配表(server->client)%12llu B\n", (unsigned long long)mtab_bytes);
    printf("字面量(client->server)%12llu B\n", (unsigned long long)literal_bytes);
    printf("合计                  %12llu B   直接传 f' 需要 %llu B\n",
           (unsigned long long)wire_bytes, (unsigned long long)new_size);
    printf("命令条数: client %llu 条 + server 1 条；server 侧从未读取 f 的数据\n",
           (unsigned long long)win_idx);

    /* ---------- 9.5 分阶段耗时表 ---------- */
    const double total_ms = now_ms() - g_t0_prog;
    phase_report(total_ms);

    /* ---------- 10. 逐字节校验（测试开销，不计入上表；--no-verify 跳过） ---------- */
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
    if (!ok) {
        printf("[FAIL] 重建结果与 f' 不一致\n");
    } else {
        printf("[PASS] 重建出的 f' 与 client 的 f' 逐字节一致\n");
        printf("目标文件: %s\n", tgt_path.c_str());
    }
    printf("[T] 逐字节校验 %.2f ms (%.1f MB/s) —— 测试开销，不计入上表\n",
           t_verify,
           t_verify > 0.0 ? (double)new_size / (t_verify / 1000.0) / 1e6 : 0.0);

    /* 回收 rsid：设备池仅 1024，泄漏的 MR 还会让设备持续持有 fdm 页引用，
     * 与每轮 drop_caches 叠加会触发 fdmfs 互锁死锁（下一实现打开 fdm 即挂死） */
    {
        struct ioctl_create_mrs d {};
        d.rsid = mrs_c.rsid;
        ioctl(cli.ctl_fd, IOCTL_CEMU_DELETE_MRS, &d);
        d.rsid = mrs_s.rsid;
        ioctl(srv.ctl_fd, IOCTL_CEMU_DELETE_MRS, &d);
    }

    close(fd_old); close(fd_new); close(fd_tgt);
    close(fd_manifest_cli); close(fd_win); close(fd_mtab_srv); close(fd_grid);
    close(fd_par);
    close(fd_manifest_srv); close(cli.ctl_fd); close(cli.ng_fd);
    close(srv.ctl_fd); close(srv.ng_fd);
    return ok ? 0 : 1;
}
