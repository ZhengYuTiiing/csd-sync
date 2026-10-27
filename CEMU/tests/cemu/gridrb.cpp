/*
 * gridds.cpp —— grid-cdc：严格按 6 步编排的双 CSD 增量同步
 *
 *   ./build/gridds <server:f> <client:f'> [-o target] [-w MB] [-c N] [-s N]
 *
 * 设备侧只有**一个**算子 kernel/grid_ds.so（三种角色靠随命令下发的载荷区分），
 * 主机侧只有这一个驱动文件 —— 两侧的 CDC 切点在同一份代码里，改一处两边同步。
 *
 * 六步（谁在什么时候算什么都是硬的）：
 *   [1] client：f' 数据读入 CSD 内存                    —— indirect R 阶段
 *   [2] client CSD：CDC 切分 + 网格 CRC 组合出 weak     —— grid_ds mode 1
 *       **不算强哈希**
 *   [3] 线上：weak 清单 client → server
 *   [4] server：f 数据读入 CSD 内存，CDC 分块 + 网格 CRC 组合出 weak，
 *       与清单里的 weak 比对                            —— grid_ds mode 2
 *   [5] server：只对**弱哈希命中**的块算强校验和，列表发回 client
 *   [6] client CSD：算出那几块的强校验和                —— grid_ds mode 3
 *       比对 → delta（令牌 + 字面量）→ server → server 重建 f'
 *
 * 前置条件（与 dsync 一致）：
 *   - f 由 server 设备的写路径写过（OOB 网格已固化在该设备上）
 *   - f' 由 client 设备的写路径写过
 *   - /mnt/fdm{C}/ 与 /mnt/fdm{S}/ 可用
 *
 * 三个踩过的坑，别踩回去：
 *   1. indirect 流水线的 R 阶段恒把数据读进 mr[1]，read_mode 是整条命令一个
 *      标志 —— 数据页与 OOB 网格不可能在同一次 R 里同时到位。所以先用
 *      collect_oob 把整份网格浅读进常驻 FDM 槽（Stage-A），之后每次 CDC
 *      命令里 mr[2] 直接指向那个槽。
 *   2. 命令返回值会被流水线覆盖，条数只能从输出表头 nr 读 —— 输出区每窗口
 *      （client 强哈希是每轮）必须先清零。
 *   3. 设备侧 dlopen 过的 .so 不会重载，UNLOAD/DOWNLOAD 也救不了。所以本
 *      算子的符号名是 grid_ds，主机把输出表头里的 build_id 打印出来，
 *      一眼就能确认设备里跑的是哪一份 .so。
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

/* 按显示宽度补空格：中文按 2 列算，否则表格会错位 */
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

/* fdmfs 走 iomap_dio_rw：缓冲必须 4KB 对齐、长度必须 4KB 整数倍 */
static void *fdm_alloc(size_t bytes)
{
    void *p = nullptr;
    if (posix_memalign(&p, PAGE, round_up_4k(bytes)) != 0 || !p) {
        return nullptr;
    }
    memset(p, 0, round_up_4k(bytes));
    return p;
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

/*
 * 窗口划分。client 的两趟（CDC 切分 / 补算强哈希）必须用**同一套窗口**，
 * 否则第二趟会有块的区间跨在窗口边界上，那一块的强哈希就取不到。
 */
struct Win {
    int      start;
    int      cnt;
    uint64_t base_nlb;
    uint64_t n;
};

static void build_wins(const NvmeCopyFormat2 *cf, int nr_cf, uint64_t win_nlb_cap,
                       std::vector<Win> &out)
{
    uint64_t cur = 0;

    for (int i = 0; i < nr_cf; ) {
        const int start = i;
        const uint64_t base = cur;
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
        Win w;
        w.start = start; w.cnt = i - start; w.base_nlb = base; w.n = n;
        out.push_back(w);
        cur += n;
    }
}

/* ================= 方案 D：文件字节区间 <-> LBA 映射 ================= */

static uint64_t round_up_sec(uint64_t v)
{
    return (v + SEC - 1) & ~(SEC - 1);
}

static uint64_t round_down_sec(uint64_t v)
{
    return v & ~(SEC - 1);
}

/*
 * 设备写的是 LBA 而不是文件偏移，所以下沉重建之前必须把
 * 「文件内的字节区间」翻译成「LBA 区间」。
 * get_file_mappings 给的是文件展开后的段列表，这里转成
 * {文件内起始偏移, 起始 LBA, 扇区数} 的连续段表。
 */
struct FileSeg {
    uint64_t off;
    uint64_t slba;
    uint64_t nlb;
};

static std::vector<FileSeg> file_segs(const NvmeCopyFormat2 *cf, int nr_cf)
{
    std::vector<FileSeg> v;
    uint64_t off = 0;

    for (int i = 0; i < nr_cf; i++) {
        FileSeg s;
        s.off  = off;
        s.slba = cf[i].slba;
        s.nlb  = (uint64_t)cf[i].nlb + 1;
        v.push_back(s);
        off += s.nlb * SEC;
    }
    return v;
}

static bool seg_find(const std::vector<FileSeg> &m, uint64_t off, size_t *idx)
{
    for (size_t i = 0; i < m.size(); i++) {
        if (off >= m[i].off && off < m[i].off + m[i].nlb * SEC) {
            *idx = i;
            return true;
        }
    }
    return false;
}

/*
 * 把源文件 [src_off, src_off+len) -> 目标文件 [dst_off, dst_off+len) 这一段
 * 翻译成成对的 (in_cf, out_cf)：in_cf[i] 读源文件、out_cf[i] 写目标文件，
 * 两边扇区数相同、一一对应。按两个文件各自的 extent 边界切开。
 */
static bool emit_pair(const std::vector<FileSeg> &src_map, uint64_t src_off,
                      const std::vector<FileSeg> &dst_map, uint64_t dst_off,
                      uint64_t len,
                      std::vector<NvmeCopyFormat2> &in_cf,
                      std::vector<NvmeCopyFormat2> &out_cf)
{
    if (((len | src_off | dst_off) & (SEC - 1)) != 0) {
        return false;               /* 必须是 512B 的整数倍 */
    }
    uint64_t done = 0;
    while (done < len) {
        size_t si = 0, di = 0;
        if (!seg_find(src_map, src_off + done, &si) ||
            !seg_find(dst_map, dst_off + done, &di)) {
            return false;
        }
        const uint64_t so = src_off + done - src_map[si].off;
        const uint64_t dd = dst_off + done - dst_map[di].off;
        const uint64_t avail_s = (uint64_t)(src_map[si].nlb * SEC - so);
        const uint64_t avail_d = (uint64_t)(dst_map[di].nlb * SEC - dd);
        uint64_t n = std::min<uint64_t>(len - done, std::min<uint64_t>(avail_s, avail_d));
        n &= ~(SEC - 1);
        if (n == 0) {
            return false;
        }
        NvmeCopyFormat2 a {}, b {};
        a.snsid = 1;
        a.slba  = src_map[si].slba + so / SEC;
        a.nlb   = (uint16_t)(n / SEC - 1);
        b.snsid = 1;
        b.slba  = dst_map[di].slba + dd / SEC;
        b.nlb   = (uint16_t)(n / SEC - 1);
        in_cf.push_back(a);
        out_cf.push_back(b);
        done += n;
    }
    return true;
}

/* 源/目的偏移不同的拷贝（移位命中后两侧位置不再相同） */
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

/* ---------- 重建命令的 chunk 切分（P>1 时用） ---------- */
struct RbChunk {
    std::vector<NvmeCopyFormat2> in_cf;     /* 源（f）侧 LBA 段 */
    std::vector<NvmeCopyFormat2> out_cf;    /* 目的（f'）侧 LBA 段，与 in_cf 一一对应 */
    uint64_t nlb = 0;
};

/*
 * 把 [begin,end) 这段 cf 对按每 chunk `per` 个扇区切开。
 * 源/目的必须同步切（两边长度恒等），否则复用区间会错位。
 * chunk_nlb 是命令级单值（compute.c:609），所以除了末尾零头，
 * 每个 chunk 的 nlb 必须精确等于 per —— 零头由调用方退回 P=1 处理。
 */
static void split_pairs(const std::vector<NvmeCopyFormat2> &in_cf,
                        const std::vector<NvmeCopyFormat2> &out_cf,
                        size_t begin, size_t end, uint64_t per,
                        std::vector<RbChunk> &out)
{
    RbChunk cur;
    bool open = false;
    uint64_t in_cur = 0;

    for (size_t i = begin; i < end; i++) {
        uint64_t n = (uint64_t)in_cf[i].nlb + 1;
        uint64_t slba_in = in_cf[i].slba;
        uint64_t slba_out = out_cf[i].slba;

        while (n > 0) {
            if (!open || in_cur == per) {
                if (open) {
                    out.push_back(cur);
                }
                cur = RbChunk();
                open = true;
                in_cur = 0;
            }
            const uint64_t take = std::min(n, per - in_cur);
            NvmeCopyFormat2 a = in_cf[i];
            a.slba = slba_in;
            a.nlb = (uint32_t)(take - 1);
            NvmeCopyFormat2 b = out_cf[i];
            b.slba = slba_out;
            b.nlb = (uint32_t)(take - 1);
            cur.in_cf.push_back(a);
            cur.out_cf.push_back(b);
            cur.nlb += take;
            in_cur += take;
            slba_in += take;
            slba_out += take;
            n -= take;
        }
    }
    if (open && cur.nlb > 0) {
        out.push_back(cur);
    }
}

int main(int argc, char **argv)
{
    setvbuf(stdout, nullptr, _IOLBF, 0);
    g_t0_prog = now_ms();

    std::string old_path, new_path, tgt_path;
    uint64_t win_mb = DEF_WIN_MB;
    int client_dev = 0, server_dev = 1;
    int P = 1;                  /* 重建命令的 chunk 并发度；1 = 不开流水线 */
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
            if (P < 1) {
                P = 1;
            }
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
    if (tgt_path.empty()) {
        tgt_path = new_path + ".gridds";
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

    printf("=== grid-cdc：双 CSD，6 步流程（强哈希只在弱哈希命中后才算）===\n");
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
            fprintf(stderr, "FIEMAP f 失败（是否在 /mnt/nvme%d 上？）\n", server_dev);
            return 1;
        }
        nlb_pc = 0; ncf_pc = 0;
        /* filesize 是入参/出参：不清零会沿用 f 的大小，导致 f' 被截断映射 */
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

    /* ---------- 3. 装载 CSF：只有一个算子 + 网格浅读 ---------- */
    uint16_t pind_gd_c = 0, pind_gd_s = 0;
    uint16_t pind_collect_c = 0, pind_collect_s = 0;
    uint16_t pind_rb = 0;
    {
        PhaseScope ph("装载 CSF", PH_HOST);
        if (load_csf(cli.ctl_fd, "./build/grid_ds.so", "grid_ds",
                     "grid_ds_v1", &pind_gd_c) < 0 ||
            load_csf(cli.ctl_fd, "./build/collect_oob.so", "collect_oob",
                     "gcollect_c_v1", &pind_collect_c) < 0 ||
            load_csf(srv.ctl_fd, "./build/grid_ds.so", "grid_ds",
                     "grid_ds_v1", &pind_gd_s) < 0 ||
            load_csf(srv.ctl_fd, "./build/collect_oob.so", "collect_oob",
                     "gcollect_s_v1", &pind_collect_s) < 0 ||
            /* 方案 D 的重建算子：新 .so + 新符号名 → 全新 dlopen，不用重启 QEMU */
            load_csf(srv.ctl_fd, "./build/grid_rb.so", "grid_rb",
                     "grid_rb_v1", &pind_rb) < 0) {
            return 1;
        }
    }

    /* ---------- 4. FDM 准备 ---------- */
    uint64_t win_bytes = win_mb << 20;
    int fd_win_c = -1, fd_win_s = -1;
    int fd_tab_c = -1, io_tab_c = -1;
    int fd_pair_s = -1, io_pair_s = -1;
    int fd_strong_c = -1, io_strong_c = -1;
    int fd_grid_c = -1, fd_grid_s = -1;
    uint64_t mf_win_io = 0, mf_win_io_s = 0;

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

        /* 块数上界都按"每 4KB 一块"估 */
        const uint64_t ent_cap = win_bytes / CHUNK_MIN_SIZE + 2;
        mf_win_io   = round_up_4k(GD_HDR_BYTES + ent_cap * GD_ENT_BYTES);
        mf_win_io_s = round_up_4k(GD_HDR_BYTES + ent_cap * GD_PAIR_BYTES);

        fd_tab_c   = ensure_fdm(cli.fdm_dir, 0, mf_win_io,   "client 清单输出", &io_tab_c);
        fd_pair_s  = ensure_fdm(srv.fdm_dir, 0, mf_win_io_s, "server 强哈希输出", &io_pair_s);
        fd_grid_c  = ensure_fdm(cli.fdm_dir, 2, grid_c_io,   "f' 网格", nullptr);
        fd_grid_s  = ensure_fdm(srv.fdm_dir, 2, grid_s_io,   "f 网格", nullptr);

        if (fd_win_s < 0 || fd_tab_c < 0 || fd_pair_s < 0 ||
            fd_grid_c < 0 || fd_grid_s < 0) {
            return 1;
        }
    }

    const uint64_t win_nlb_cap = win_bytes / SEC;
    std::vector<Win> new_wins, old_wins;
    build_wins(new_cf, nr_new_cf, win_nlb_cap, new_wins);
    build_wins(old_cf, nr_old_cf, win_nlb_cap, old_wins);
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

    /* ---------- Stage-A：两侧各自浅读自己的 OOB 网格 ---------- */
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
            uint64_t gb = jobs[k].nlb * SEC / PAGE * CSD_CRC_SLOT_LEN;
            {
                PhaseScope ph(jobs[k].tag, PH_DEVICE, &gb);
                long long r = -1;
                if (run_indirect(jobs[k].d->ng_fd, jobs[k].pind, mrs.rsid, 0, 0,
                                 CSD_READ_MODE_OOB, 0, jobs[k].cf, jobs[k].nr_cf,
                                 (uint32_t)(jobs[k].nlb - 1), nullptr, 0, &r) < 0) {
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
    std::vector<uint8_t> cli_tab;   /* gd_tab_hdr + N × gd_ent */
    {
        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io);
        if (!rbuf || !zbuf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }

        std::vector<uint8_t> acc;
        bool bad = false;

        for (size_t w = 0; w < new_wins.size() && !bad; w++) {
            const Win &win = new_wins[w];

            {
                PhaseScope ph("FDM 清零 (client)", PH_HOST);
                if (pwrite(io_tab_c, zbuf, (size_t)mf_win_io, 0) != (ssize_t)mf_win_io) {
                    perror("清零清单区");
                    bad = true;
                    break;
                }
            }

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3);
            mrs.fd[0] = fd_tab_c;  mrs.off[0] = 0; mrs.size[0] = (long long)mf_win_io;
            mrs.fd[1] = fd_win_c;  mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            mrs.fd[2] = fd_grid_c; mrs.off[2] = 0; mrs.size[2] = (long long)grid_c_io;
            if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(client CDC)");
                bad = true;
                break;
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("CDC+网格弱哈希 (client)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(cli.ng_fd, pind_gd_c, mrs.rsid,
                                 win.n * SEC, win.base_nlb * SEC,
                                 CSD_READ_MODE_DATA, 0,
                                 &new_cf[win.start], win.cnt,
                                 (uint32_t)(win.n - 1), nullptr, 0, &r) < 0) {
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

            {
                PhaseScope ph("FDM 回读 (client)", PH_HOST);
                if (pread(io_tab_c, rbuf, (size_t)mf_win_io, 0) != (ssize_t)mf_win_io) {
                    perror("pread 清单");
                    bad = true;
                    break;
                }
            }

            const struct gd_tab_hdr *h = (const struct gd_tab_hdr *)(const void *)rbuf;
            uint64_t nr_w = (h->magic == GD_TAB_MAGIC) ? h->nr : 0;
            if (h->entry_bytes != (uint32_t)GD_ENT_BYTES) {
                fprintf(stderr, "[client] 窗口 #%zu entry_bytes=%u 与预期 %llu 不符\n",
                        w, h->entry_bytes, (unsigned long long)GD_ENT_BYTES);
                bad = true;
                break;
            }
            if (GD_HDR_BYTES + nr_w * GD_ENT_BYTES > mf_win_io) {
                fprintf(stderr, "[client] 窗口 #%zu 条数越界 %llu\n",
                        w, (unsigned long long)nr_w);
                bad = true;
                break;
            }
            acc.insert(acc.end(), rbuf + GD_HDR_BYTES,
                       rbuf + GD_HDR_BYTES + nr_w * GD_ENT_BYTES);
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
    printf("[1][2] client CSD CDC+网格弱哈希 : %llu 块, weak 清单 %llu B (%.1f KB/GB), "
           "未算任何强哈希\n",
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
     * [步骤 4] server：f 读入 CSD 内存 -> CDC 分块 -> 网格 weak -> 与清单比对
     * [步骤 5] 只对弱哈希命中的块算强校验和
     * ================================================================== */
    std::vector<uint8_t> srv_tab;   /* gd_tab_hdr + M × gd_pair */
    {
        uint8_t *rbuf = (uint8_t *)fdm_alloc(mf_win_io_s);
        uint8_t *zbuf = (uint8_t *)fdm_alloc(mf_win_io_s);
        if (!rbuf || !zbuf) {
            fprintf(stderr, "分配对齐缓冲失败\n");
            return 1;
        }

        std::vector<uint8_t> acc;
        bool bad = false;

        for (size_t w = 0; w < old_wins.size() && !bad; w++) {
            const Win &win = old_wins[w];

            {
                PhaseScope ph("FDM 清零 (server)", PH_HOST);
                if (pwrite(io_pair_s, zbuf, (size_t)mf_win_io_s, 0) !=
                    (ssize_t)mf_win_io_s) {
                    perror("清零强哈希区");
                    bad = true;
                    break;
                }
            }

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 3);
            mrs.fd[0] = fd_pair_s; mrs.off[0] = 0; mrs.size[0] = (long long)mf_win_io_s;
            mrs.fd[1] = fd_win_s;  mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            mrs.fd[2] = fd_grid_s; mrs.off[2] = 0; mrs.size[2] = (long long)grid_s_io;
            if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(server CDC)");
                bad = true;
                break;
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("CDC+weak 比对+强哈希 (server)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(srv.ng_fd, pind_gd_s, mrs.rsid,
                                 win.n * SEC, win.base_nlb * SEC,
                                 CSD_READ_MODE_DATA, 0,
                                 &old_cf[win.start], win.cnt,
                                 (uint32_t)(win.n - 1),
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

            {
                PhaseScope ph("FDM 回读 (server)", PH_HOST);
                if (pread(io_pair_s, rbuf, (size_t)mf_win_io_s, 0) !=
                    (ssize_t)mf_win_io_s) {
                    perror("pread 强哈希表");
                    bad = true;
                    break;
                }
            }

            const struct gd_tab_hdr *h = (const struct gd_tab_hdr *)(const void *)rbuf;
            uint64_t nr_w = (h->magic == GD_TAB_MAGIC) ? h->nr : 0;
            if (h->entry_bytes != (uint32_t)GD_PAIR_BYTES) {
                fprintf(stderr, "[server] 窗口 #%zu entry_bytes=%u 与预期 %llu 不符\n",
                        w, h->entry_bytes, (unsigned long long)GD_PAIR_BYTES);
                bad = true;
                break;
            }
            if (GD_HDR_BYTES + nr_w * GD_PAIR_BYTES > mf_win_io_s) {
                fprintf(stderr, "[server] 窗口 #%zu 条数越界 %llu\n",
                        w, (unsigned long long)nr_w);
                bad = true;
                break;
            }
            acc.insert(acc.end(), rbuf + GD_HDR_BYTES,
                       rbuf + GD_HDR_BYTES + nr_w * GD_PAIR_BYTES);
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

    /* ---------- 线上：强校验和列表 server -> client ---------- */
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

    /* 查询表：一条弱命中一条查询，idx 直接等于条目下标 */
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
            fd_strong_c = ensure_fdm(cli.fdm_dir, 3, strong_c_io, "client 强哈希输出",
                                     &io_strong_c);
            if (fd_strong_c < 0) {
                return 1;
            }
            uint8_t *zero = (uint8_t *)fdm_alloc(strong_c_io);
            if (!zero) {
                return 1;
            }
            /* 整轮只清零一次：各窗口按 idx 分批填，不能每窗口清 */
            if (pwrite(io_strong_c, zero, (size_t)strong_c_io, 0) != (ssize_t)strong_c_io) {
                perror("清零 client 强哈希区");
                free(zero);
                return 1;
            }
            free(zero);
        }

        bool bad = false;
        for (size_t w = 0; w < new_wins.size() && !bad; w++) {
            const Win &win = new_wins[w];

            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 2);
            mrs.fd[0] = fd_strong_c; mrs.off[0] = 0; mrs.size[0] = (long long)strong_c_io;
            mrs.fd[1] = fd_win_c;    mrs.off[1] = 0; mrs.size[1] = (long long)win_bytes;
            if (ioctl(cli.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                perror("CREATE_MRS(client 强哈希)");
                bad = true;
                break;
            }

            uint64_t win_b = win.n * SEC;
            {
                PhaseScope ph("补算强哈希 (client)", PH_DEVICE, &win_b);
                long long r = -1;
                if (run_indirect(cli.ng_fd, pind_gd_c, mrs.rsid,
                                 win.n * SEC, win.base_nlb * SEC,
                                 CSD_READ_MODE_DATA, 0,
                                 &new_cf[win.start], win.cnt,
                                 (uint32_t)(win.n - 1),
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

        uint8_t *rbuf = (uint8_t *)fdm_alloc(strong_c_io);
        if (!rbuf) {
            return 1;
        }
        {
            PhaseScope ph("FDM 回读 (client 强哈希)", PH_HOST);
            if (pread(io_strong_c, rbuf, (size_t)strong_c_io, 0) != (ssize_t)strong_c_io) {
                perror("pread client 强哈希");
                free(rbuf);
                return 1;
            }
        }
        const struct gd_ent *e =
            (const struct gd_ent *)(const void *)(rbuf + GD_HDR_BYTES);
        for (uint64_t i = 0; i < n_hit; i++) {
            cli_strong[i] = e[i];
        }
        free(rbuf);
    }

    /* 主机侧两级比对：弱哈希已命中，这里用强哈希确认 */
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
        /*
         * 不截断：方案 D 要把数据直接写到输出文件的 LBA 上，ext4 必须先"写实"
         * 这些 extent（否则是 unwritten extent，读回来全是零）。这个"写实"
         * 是一次性成本，所以复用上次留下的同尺寸文件，不要每次 O_TRUNC 掉。
         */
        fd_tgt = open(tgt_path.c_str(), O_RDWR | O_CREAT, 0644);
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
        /* ---------- 1. 把令牌合并成连续区间 ---------- */
        std::vector<Token> runs;
        for (size_t k = 0; k < tokens.size(); k++) {
            const uint64_t tgt = tokens[k].new_off;
            /* 夹到真实文件长度：最后一个块可能只用了不到 length 字节 */
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
        printf("    重建前合并令牌: %zu 条 -> %zu 段连续区间\n",
               tokens.size(), runs.size());

        /*
         * ---------- 2. 收缩到 4KB 对齐 ----------
         * 两层原因：
         *   a) 设备按扇区写，复用区间必须落在整扇区上；
         *   b) 更要紧的是**页**：字面量由主机 pwrite 写入、复用区间由设备直接写 LBA，
         *      如果两者共用同一个 4KB 页，页缓存回写就会把设备写进去的内容覆盖掉。
         * 对齐到 4KB 后，每个页要么整页归主机、要么整页归设备，互不相干 ——
         * 于是不必在设备写之前 fsync，省掉一次全文件刷盘。
         * 代价是每条边界最多多出 4KB 字面量（本例 2 段 → 最多 16KB）。
         */
        struct Piece {
            uint64_t src, dst, len;
        };
        std::vector<Piece> reuse;
        for (size_t k = 0; k < runs.size(); k++) {
            const uint64_t s = round_up_4k(runs[k].new_off);
            const uint64_t e = (runs[k].new_off + runs[k].len) & ~(PAGE - 1);
            if (e <= s) {
                continue;
            }
            Piece p;
            p.dst = s;
            p.src = runs[k].old_off + (s - runs[k].new_off);
            p.len = e - s;
            if (!reuse.empty() && p.dst < reuse.back().dst + reuse.back().len) {
                continue;
            }
            reuse.push_back(p);
        }

        /* ---------- 3. 准备输出文件 + 写字面量（主机侧，量很小） ---------- */
        double q0 = now_ms();

        /*
         * ext4 的 extent 必须处于 "written" 状态。fallocate 产生的是 unwritten
         * extent，设备绕过文件系统写进 LBA 之后，ext4 仍认为那块没写，任何
         * buffered read 都返回零 —— 表现为"设备写了但校验全 0"。
         * 首次真要写一遍零 + fsync 把它转成 written；之后复用同尺寸文件即可跳过。
         */
        bool need_prep = false;
        {
            struct stat stt {};
            if (fstat(fd_tgt, &stt) != 0 || (uint64_t)stt.st_size != new_size) {
                need_prep = true;
            } else {
                uint64_t fsz = new_size;
                NvmeCopyFormat2 *tcf = nullptr;
                int nlb_pc_t = 0, ncf_pc_t = 0;
                int n = get_file_mappings(tgt_path.c_str(), &fsz, &tcf, 1,
                                          &nlb_pc_t, &ncf_pc_t);
                uint64_t covered = 0;
                for (int i = 0; i < n; i++) {
                    covered += ((uint64_t)tcf[i].nlb + 1) * SEC;
                }
                if (n <= 0 || covered < new_size) {
                    need_prep = true;
                }
            }
        }
        if (need_prep) {
            printf("    [提示] 首次准备输出文件（写实 extent，一次性成本，不计入重建）\n");
            if (ftruncate(fd_tgt, 0) != 0) {
                perror("ftruncate");
                return 1;
            }
            std::vector<uint8_t> zero(1u << 20, 0);
            uint64_t off = 0;
            while (off < new_size) {
                size_t n = (size_t)std::min<uint64_t>(new_size - off, zero.size());
                if (pwrite(fd_tgt, zero.data(), n, (off_t)off) != (ssize_t)n) {
                    perror("写实输出文件");
                    return 1;
                }
                off += n;
            }
            fsync(fd_tgt);
        }
        q0 = now_ms();

        /* 字面量 = 复用区间的补集（同样 512B 对齐），从 f' 原样搬过来 */
        auto copy_lit = [&](uint64_t a, uint64_t b) -> bool {
            if (b <= a) {
                return true;
            }
            if (!copy_range2(fd_tgt, fd_new, a, a, b - a)) {
                return false;
            }
            literal_bytes += b - a;
            return true;
        };
        uint64_t cur = 0;
        bool lit_ok = true;
        for (size_t k = 0; k < reuse.size() && lit_ok; k++) {
            lit_ok = copy_lit(cur, reuse[k].dst);
            cur = reuse[k].dst + reuse[k].len;
            reuse_bytes += reuse[k].len;
        }
        if (lit_ok) {
            lit_ok = copy_lit(cur, new_size);
        }
        if (!lit_ok) {
            fprintf(stderr, "写字母量失败\n");
            return 1;
        }
        /*
         * 这里**不 fsync**：上面已经把区间对齐到 4KB，主机写的字面量页和设备
         * 写的复用页完全不重叠，脏页晚点自然回写也不会覆盖设备的数据。
         * 加 fsync 会让这一阶段从 ~30ms 涨到 50~200ms（且抖动很大），
         * 且它并非正确性所必需。
         */
        t_literal = now_ms() - q0;

        /* ---------- 4. 设备内重建：复用区间由 CSD 直接搬，不经过主机 ---------- */
        q0 = now_ms();
        if (!reuse.empty()) {
            uint64_t tsz = new_size;
            NvmeCopyFormat2 *tgt_cf = nullptr;
            int nlb_pc_t = 0, ncf_pc_t = 0;
            int nr_tgt_cf = get_file_mappings(tgt_path.c_str(), &tsz, &tgt_cf, 1,
                                              &nlb_pc_t, &ncf_pc_t);
            if (nr_tgt_cf <= 0) {
                fprintf(stderr, "取输出文件映射失败\n");
                return 1;
            }

            const std::vector<FileSeg> src_map = file_segs(old_cf, nr_old_cf);
            const std::vector<FileSeg> dst_map = file_segs(tgt_cf, nr_tgt_cf);

            std::vector<NvmeCopyFormat2> in_cf, out_cf;
            for (size_t k = 0; k < reuse.size(); k++) {
                if (!emit_pair(src_map, reuse[k].src, dst_map, reuse[k].dst,
                               reuse[k].len, in_cf, out_cf)) {
                    fprintf(stderr, "复用区间 <<%llu,%llu) 映射失败\n",
                            (unsigned long long)reuse[k].dst,
                            (unsigned long long)(reuse[k].dst + reuse[k].len));
                    return 1;
                }
            }

            /*
             * 单条命令的数据上限 8MB。cparam1 是命令级参数（P 个 chunk 共用），
             * CSF 靠它知道本次读入多少字节，所以一条命令内只能有一轮 IO ——
             * 超过上限就拆成多条命令，每条各自算准自己的长度。
             *
             * -p P 让一条命令内并发 P 个 chunk：indirect_main 会把 mr_addr 按
             *   chunk 偏移后再交给 CSF（compute.c:1037-1051），grid_rb 看到的
             *   仍是自己的 mr[0]/mr[1]，所以设备侧算子不用改。约束是
             *   chunk_nlb 为命令级单值、各 chunk 必须等长，且 numr 能被 P 整除
             *   （compute.c:633）—— 这里 numr = 2*P。
             *   尾部不足 P*per 的零头退回 P=1 单独一条命令。
             */
            const uint64_t cap_nlb = 16384;                 /* 8MB */
            const uint64_t cap_bytes = round_up_4k(cap_nlb * SEC);

            int fd_out_buf = ensure_fdm(srv.fdm_dir, 6, cap_bytes, "重建输出缓冲", nullptr);
            int fd_in_buf  = ensure_fdm(srv.fdm_dir, 7, cap_bytes, "重建输入缓冲", nullptr);
            if (fd_out_buf < 0 || fd_in_buf < 0) {
                return 1;
            }

            /* 每 chunk 扇区数：取 4KB 对齐，且 P 份仍不超过单条命令上限 */
            const uint64_t per = P > 1 ? (cap_nlb / (uint64_t)P / 8) * 8 : cap_nlb;
            const bool use_p = (P > 1 && per >= 8);

            size_t i = 0;
            int nr_cmd = 0;
            bool dev_ok = true;
            while (i < in_cf.size() && dev_ok) {
                const uint64_t batch_cap = use_p ? per * (uint64_t)P : cap_nlb;
                size_t j = i;
                uint64_t acc = 0;
                while (j < in_cf.size()) {
                    const uint64_t s = (uint64_t)in_cf[j].nlb + 1;
                    if (j > i && acc + s > batch_cap) {
                        break;
                    }
                    acc += s;
                    j++;
                }

                std::vector<RbChunk> chunks;
                int Pw = 1;
                uint64_t chunk_nlb = acc;

                if (use_p && acc == per * (uint64_t)P) {
                    split_pairs(in_cf, out_cf, i, j, per, chunks);
                    if ((int)chunks.size() == P) {
                        Pw = P;
                        chunk_nlb = per;
                    } else {
                        chunks.clear();
                    }
                }
                if (Pw == 1) {
                    RbChunk one;
                    one.in_cf.assign(in_cf.begin() + i, in_cf.begin() + j);
                    one.out_cf.assign(out_cf.begin() + i, out_cf.begin() + j);
                    one.nlb = acc;
                    chunks.push_back(one);
                }

                const uint64_t chunk_bytes = round_up_4k(chunk_nlb * SEC);

                struct ioctl_create_mrs mrs {};
                prep_mrs(&mrs, 2 * Pw);
                for (int k = 0; k < Pw; k++) {
                    mrs.fd[2*k + 0] = fd_out_buf;
                    mrs.off[2*k + 0] = (long long)((uint64_t)k * chunk_bytes);
                    mrs.size[2*k + 0] = (long long)chunk_bytes;
                    mrs.fd[2*k + 1] = fd_in_buf;
                    mrs.off[2*k + 1] = (long long)((uint64_t)k * chunk_bytes);
                    mrs.size[2*k + 1] = (long long)chunk_bytes;
                }
                if (ioctl(srv.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
                    perror("CREATE_MRS(重建)");
                    return 1;
                }

                std::vector<NvmeCopyFormat2> all_in, all_out;
                std::vector<int> ncf_in(Pw, 0), ncf_out(Pw, 0);
                for (int k = 0; k < Pw; k++) {
                    ncf_in[k]  = (int)chunks[k].in_cf.size();
                    ncf_out[k] = (int)chunks[k].out_cf.size();
                    all_in.insert(all_in.end(),
                                  chunks[k].in_cf.begin(), chunks[k].in_cf.end());
                    all_out.insert(all_out.end(),
                                   chunks[k].out_cf.begin(), chunks[k].out_cf.end());
                }

                const size_t need = calc_indirect_task_buffer_size(Pw, 1, (int)all_in.size(),
                                                                   (int)all_out.size(), 0);
                std::vector<uint8_t> task(need, 0);
                size_t packed = pack_indirect_task_buffer(task.data(), Pw, 1,
                                                          CSD_READ_MODE_DATA,
                                                          (int)all_in.size(), ncf_in.data(),
                                                          all_in.data(),
                                                          (int)all_out.size(), ncf_out.data(),
                                                          all_out.data(),
                                                          nullptr, 0);
                struct nvme_uring_cmd cmd {};
                if (packed != need) {
                    fprintf(stderr, "重建任务打包长度不符 %zu != %zu\n", packed, need);
                    dev_ok = false;
                } else {
                    prep_nvme_uring_program_execute(&cmd, chunk_nlb * SEC, 0, pind_rb,
                                                    mrs.rsid, 2, 0,
                                                    (uint32_t)(chunk_nlb - 1), 0,
                                                    task.data(), (uint32_t)packed);
                    long long rr = -1;
                    if (submit_execute_cmd(srv.ng_fd, &cmd, &rr) < 0) {
                        fprintf(stderr, "设备内重建命令失败\n");
                        dev_ok = false;
                    }
                }

                {
                    struct ioctl_create_mrs dm {};
                    dm.rsid = mrs.rsid;
                    ioctl(srv.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
                }
                if (!dev_ok) {
                    break;
                }
                nr_cmd++;
                i = j;
            }
            if (!dev_ok) {
                return 1;
            }
            printf("    设备内重建: %zu 对 cf / %d 条命令，单条上限 %llu KB，P=%d\n",
                   in_cf.size(), nr_cmd,
                   (unsigned long long)(cap_nlb * SEC / 1024), P);
        }
        t_reuse = now_ms() - q0;

        /* 设备直接写了 LBA，页缓存里可能还是旧内容 —— 读之前必须丢掉 */
        posix_fadvise(fd_tgt, 0, 0, POSIX_FADV_DONTNEED);
    }
    phase_rec("重建: 准备+字面量", PH_HOST, literal_bytes, t_literal);
    phase_rec("重建: 设备内复用 f", PH_DEVICE, reuse_bytes, t_reuse);

    printf("[6] 线上 delta -> server       : 令牌 %llu 条 + 字面量 %llu B (%.1f%% of f')\n",
           (unsigned long long)tokens.size(), (unsigned long long)literal_bytes,
           100.0 * (double)literal_bytes / (double)new_size);
    printf("[6] server 重建 f'             : 复用 %llu B + 字面量 %llu B\n",
           (unsigned long long)reuse_bytes, (unsigned long long)literal_bytes);

    /* ---------- 报告 ---------- */
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
    close(io_tab_c); close(io_pair_s);
    if (io_strong_c >= 0) { close(io_strong_c); }
    close(cli.ctl_fd); close(cli.ng_fd);
    close(srv.ctl_fd); close(srv.ng_fd);
    return ok ? 0 : 1;
}
