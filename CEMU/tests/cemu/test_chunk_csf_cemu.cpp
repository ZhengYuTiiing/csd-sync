/*
 * test_chunk_csf_cemu.cpp —— L1 验证：在真实 CEMU CSD 上跑 CDC 算子链
 *
 * ============================ L1 在验什么 ============================
 * L0（test_chunk_csf）已经把**逻辑**验完了（算子 .so 在主机 dlopen 直接调用，
 * 与主机参考实现逐项对拍）。L1 不再重复验逻辑，只验**通路**：
 *
 *   1. .so 能被 CSD 正确 dlopen/dlsym + 下载/激活（pind 分配、program 状态机）
 *   2. MRS 建表：多个 FDM 文件 → 多条 memory range → mr_addr/mr_len 是否如预期
 *      （mr[0] 是输出、mr[1] 是输入，和文档/vadd 约定一致）
 *   3. 程序返回值能回到主机：CSD 把 CSF 的返回值 TLV 到 req->cqe.res64
 *      （compute.c:1265），io_uring 走 cqe->big_cqe[0]，ioctl 走 cmd.result
 *   4. cparam1/cparam2 能正确送达（段长 / 段偏移）
 *   5. 算子写出的表在设备内存里、主机 pread 能读回，且内容与 L0 的参考实
 *      现一致（这一步顺带复核了"设备内存里的表"和"主机算的表"逐字节相同）
 *   6. 两个算子接力：Stage-1 的输出表留在设备内存/FDM 里，直接作为 Stage-2
 *      的输入 mr[]，主机只读最终合并表（这就是"数据流不回来"的形态）
 *
 * 分工建议：L1 用少量用例（1~3 个规模），逻辑覆盖交给 L0 —— 因为 L1 一次要
 * 秒级/分钟级，不适合堆用例。
 *
 * ============================ 拓扑 ============================
 *   /mnt/fdm0/0 .. /mnt/fdm0/{k-1} : 各段切点表(Stage-1 输出 / Stage-2 输入)
 *   /mnt/fdm0/{k}                  : 整文件数据(输入) —— 若还有空闲文件，则
 *                                    Stage-2 的合并表用 /mnt/fdm0/{k+1}，
 *                                    否则复用数据文件（Stage-1 结束后它就没用了）
 *   MRS_s1[i] = { fd_table[i], fd_data(off=seg_off[i], size=seg_len[i]) }
 *   MRS_s2    = { fd_merged, fd_table[0], .., fd_table[k-1] }
 *
 * 默认只要求 tests/cemu/prep_data.sh 建好的 4 个 FDM 文件 → k = 3。
 * 想跑更多段：先把 prep_data.sh 的循环改成 0..15（或手工 fallocate），
 * 再用 --segments N（需要 N+1 个文件）。
 *
 * 用法:
 *   make -C kernel && make
 *   sudo ./build/test_chunk_csf_cemu [--segments 3] [--size-mb 4] [--sync]
 *                                    [--pattern 0|1|2] [--ctl /dev/nvme0c3]
 *                                    [--ng /dev/ng0n3] [--fdm-dir /mnt/fdm0]
 *                                    [--host-header /home/zyt/parasync/src/fastcdc.h]
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/nvme_ioctl.h>
#include <zlib.h>

#include "util.h"
#include "kernel/chunk_abi.h"

/*
 * io_uring 并发提交需要 liburing >= 2.2（SQE128/CQE32 + URING_CMD）。
 * 本机 /usr/include/liburing.h 是 2.1 —— 自动退化成 ioctl 串行提交：
 * MRS / 返回值 / 读回校验全都照验，只是 Stage-1 各段不并发。
 * 想跑并发路径（推荐，能顺带验 CSD 的调度）：
 *   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src
 *
 * 探测方法注意：IORING_SETUP_SQE128/CQE32 是宏（defined() 可探），
 * 但 IORING_OP_URING_CMD 是枚举成员（defined() 探不到），所以用
 * sqe->cmd / cqe->big_cqe 的成员存在性（SFINAE）做二次确认。
 */
#include <type_traits>
#include <liburing.h>

template <typename...> struct l1_void { using type = void; };
template <typename... Ts> using l1_void_t = typename l1_void<Ts...>::type;

template <typename T, typename = void> struct sqe_has_cmd : std::false_type {};
template <typename T> struct sqe_has_cmd<T, l1_void_t<decltype(T::cmd)>> : std::true_type {};

template <typename T, typename = void> struct cqe_has_big_cqe : std::false_type {};
template <typename T> struct cqe_has_big_cqe<T, l1_void_t<decltype(T::big_cqe)>> : std::true_type {};

#if defined(IORING_SETUP_SQE128) && defined(IORING_SETUP_CQE32)
#  define HAVE_URING_CMD 1
static_assert(sqe_has_cmd<struct io_uring_sqe>::value &&
              cqe_has_big_cqe<struct io_uring_cqe>::value,
              "liburing 头自相矛盾：有 SQE128/CQE32 宏却没有 sqe->cmd/cqe->big_cqe 成员");
#else
#  define HAVE_URING_CMD 0
#endif
#define URG_QUEUE_DEPTH 64

static_assert(sizeof(struct chunk_ent) == 24, "chunk_ent 必须是 24 字节");
static_assert(sizeof(struct chunk_table_hdr) == 48, "chunk_table_hdr 必须是 48 字节");

static int g_fail = 0;

static void checkf(bool ok, const char *fmt, ...)
{
    if (ok)
        return;
    g_fail++;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [FAIL] ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static void info(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [info] ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static double ms_since(const std::chrono::steady_clock::time_point &t0)
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0).count();
}

/* ---------------- 主机参考实现（与 L0 harness 完全同一份逻辑） ---------------- */

static inline uint32_t zcrc(const void *p, size_t n, uint32_t prev)
{
    return (uint32_t)crc32((uLong)prev, (const Bytef *)p, (uInt)n);
}

static void ref_seg_cuts(const uint8_t *buf, uint64_t off, uint64_t len,
                         const uint64_t *gear, std::vector<chunk_ent> &out)
{
    uint64_t fp = 0, pre = off;
    for (uint64_t i = off; i < off + len; i++) {
        fp = (fp << 1) + gear[buf[i]];
        if ((fp & CHUNK_GEAR_MASK) == 0) {
            if (i > pre) {
                chunk_ent e = {pre, i - pre, zcrc(buf + pre, i - pre, 0), CHUNK_ENT_CRC_VALID};
                out.push_back(e);
            }
            pre = i;
        }
    }
    if (pre < off + len) {
        uint64_t l = off + len - pre;
        chunk_ent e = {pre, l, zcrc(buf + pre, l, 0), CHUNK_ENT_CRC_VALID | CHUNK_ENT_TAIL_OPEN};
        out.push_back(e);
    }
}

static std::vector<chunk_ent> ref_detector(const std::vector<chunk_ent> &raw, const uint8_t *buf)
{
    std::vector<chunk_ent> out;
    uint64_t total = 0, start = 0;
    for (const auto &c : raw) {
        if (total == 0)
            start = c.offset;
        total += c.length;
        if (total >= CHUNK_MIN_SIZE) {
            chunk_ent e = {start, total, zcrc(buf + start, total, 0), CHUNK_ENT_MERGED};
            out.push_back(e);
            total = 0;
        }
    }
    if (total > 0) {
        chunk_ent e = {start, total, zcrc(buf + start, total, 0),
                       CHUNK_ENT_MERGED | CHUNK_ENT_TAIL_OPEN};
        out.push_back(e);
    }
    return out;
}

static void gen(uint8_t *buf, uint64_t n, int pattern, uint32_t seed)
{
    std::mt19937 rng(seed);
    if (pattern == 0) {
        for (uint64_t i = 0; i < n; i++)
            buf[i] = (uint8_t)rng();
    } else if (pattern == 1) {
        memset(buf, 0, n);
    } else {
        for (uint64_t i = 0; i < n; i++)
            buf[i] = (uint8_t)((i % 4096) * 31 + (i / 4096) % 7);
    }
}

static uint64_t fnv1a(const std::vector<chunk_ent> &v)
{
    uint64_t h = 1469598103934665603ULL;
    for (const auto &e : v) {
        uint64_t x[3] = {e.offset, e.length, (uint64_t)e.crc32};
        for (int k = 0; k < 3; k++)
            for (int b = 0; b < 8; b++) {
                h ^= (uint8_t)(x[k] >> (8 * b));
                h *= 1099511628211ULL;
            }
    }
    return h;
}

static bool load_host_gear(const char *path, uint64_t out[256])
{
    std::ifstream f(path);
    if (!f)
        return false;
    std::string line;
    bool in = false;
    int cnt = 0;
    while (std::getline(f, line)) {
        if (!in) {
            if (line.find("GEARv2[256]") != std::string::npos)
                in = true;
            continue;
        }
        if (line.find("};") != std::string::npos)
            break;
        size_t p = 0;
        while ((p = line.find("0x", p)) != std::string::npos) {
            char *end = nullptr;
            unsigned long long v = strtoull(line.c_str() + p, &end, 16);
            if (cnt < 256)
                out[cnt++] = (uint64_t)v;
            p = (size_t)(end - line.c_str());
        }
    }
    return cnt == 256;
}

/* ---------------- CEMU 侧基础设施 ---------------- */

struct ExecReq {
    uint64_t c1 = 0, c2 = 0;
    uint16_t pind = 0, rsid = 0;
    long long res = 0;      /* 程序返回值（CSD 侧 jit_fn 的返回值） */
    bool err = false;
    const char *tag = "";
};

/* 等价于 tests/cemu/iouring.h:io_uring_prep_compute()；这里内联一份，
 * 因为那份头文件在旧 liburing（缺 sqe->cmd / cmd_op）下根本无法编译。 */
#if HAVE_URING_CMD
static inline void prep_compute_sqe(struct io_uring_sqe *sqe, int fd,
                                    uint64_t cparam1, uint64_t cparam2,
                                    uint16_t pind, uint16_t rsid)
{
    memset(sqe, 0, sizeof(*sqe));
    sqe->opcode = IORING_OP_URING_CMD;
    sqe->cmd_op = NVME_URING_CMD_IO; /* util.h 兜底定义，见其 compat 块 */
    sqe->fd = fd;
    struct nvme_program_execute_cmd *cmd = (struct nvme_program_execute_cmd *)sqe->cmd;
    cmd->nsid = 3;
    cmd->opcode = 0x01;
    cmd->cparam1 = cparam1;
    cmd->cparam2 = cparam2;
    cmd->pind = pind;
    cmd->rsid = rsid;
    cmd->numr = 0; /* 0 = 用 MRS（rsid）里的全部 range */
    cmd->dlen = 0;
    cmd->group = 0;
    cmd->chunk_nlb = 0;
    cmd->user_runtime = 0;
}
#endif

/* 一次提交一批 compute 作业；uring=true 时用 io_uring 并发提交 */
static bool submit_batch(int ng_fd, bool uring, std::vector<ExecReq> &reqs)
{
    if (uring) {
#if HAVE_URING_CMD
        struct io_uring ring;
        if (io_uring_queue_init(URG_QUEUE_DEPTH, &ring,
                                IORING_SETUP_SQE128 | IORING_SETUP_CQE32) < 0) {
            perror("io_uring_queue_init");
            return false;
        }
        for (size_t i = 0; i < reqs.size(); i++) {
            struct io_uring_sqe *sqe = io_uring_get_sqe(&ring);
            if (!sqe) {
                fprintf(stderr, "io_uring_get_sqe 失败\n");
                io_uring_queue_exit(&ring);
                return false;
            }
            prep_compute_sqe(sqe, ng_fd, reqs[i].c1, reqs[i].c2, reqs[i].pind, reqs[i].rsid);
            io_uring_sqe_set_data(sqe, (void *)(uintptr_t)i);
        }
        int nr = io_uring_submit(&ring);
        if (nr != (int)reqs.size()) {
            fprintf(stderr, "io_uring_submit 只提交了 %d/%zu\n", nr, reqs.size());
            io_uring_queue_exit(&ring);
            return false;
        }
        size_t got = 0;
        while (got < reqs.size()) {
            struct io_uring_cqe *cqe = nullptr;
            if (io_uring_wait_cqe(&ring, &cqe) < 0) {
                perror("io_uring_wait_cqe");
                io_uring_queue_exit(&ring);
                return false;
            }
            size_t idx = (size_t)(uintptr_t)io_uring_cqe_get_data(cqe);
            long long res = (long long)cqe->big_cqe[0]; /* CQE32: 就是 req->cqe.res64 */
            int32_t err = cqe->res;
            if (idx < reqs.size()) {
                reqs[idx].res = res;
                reqs[idx].err = err < 0;
                if (err < 0)
                    fprintf(stderr, "  作业 %s cqe->res = %d (%s)\n",
                            reqs[idx].tag, err, strerror(-err));
            }
            io_uring_cqe_seen(&ring, cqe);
            got++;
        }
        io_uring_queue_exit(&ring);
        return true;
#else
        (void)ng_fd;
        (void)reqs;
        return false; /* 调用方已保证不会走到这里 */
#endif
    }

    for (auto &r : reqs) {
        struct nvme_passthru_cmd cmd {};
        prep_nvme_passthru_program_execute(&cmd, r.c1, r.c2, r.pind, r.rsid,
                                           /*numr=*/0, /*group=*/0, /*chunk_nlb=*/0,
                                           /*user_runtime=*/0, nullptr, 0);
        int ret = ioctl(ng_fd, NVME_IOCTL_IO_CMD, &cmd);
        if (ret < 0) {
            perror("ioctl NVME_IOCTL_IO_CMD");
            return false;
        }
        r.res = (long long)(int32_t)cmd.result;
        r.err = false;
    }
    return true;
}

static int open_fdm(const char *dir, int i)
{
    char p[256];
    snprintf(p, sizeof(p), "%s/%d", dir, i);
    int fd = open(p, O_RDWR);
    if (fd < 0)
        info("打不开 %s（当作文件数到顶）", p);
    return fd;
}

/* 建一个 MRS；n = 条数。返回 rsid，<0 失败 */
static int create_mrs(int ctl_fd, const int *fd, const long long *off,
                      const long long *size, int n)
{
    struct ioctl_create_mrs mrs;
    prep_mrs(&mrs, n);
    for (int i = 0; i < n; i++) {
        mrs.fd[i] = fd[i];
        mrs.off[i] = off[i];
        mrs.size[i] = size[i];
    }
    int ret = ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs);
    int rsid = mrs.rsid;
    free(mrs.fd);
    free(mrs.off);
    free(mrs.size);
    if (ret) {
        perror("ioctl CREATE_MRS");
        return -1;
    }
    return rsid;
}

static void delete_mrs(int ctl_fd, int rsid)
{
    struct ioctl_create_mrs mrs;
    prep_mrs(&mrs, 1);
    mrs.rsid = (uint16_t)rsid;
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs);
    free(mrs.fd);
    free(mrs.off);
    free(mrs.size);
}

/* 从设备侧读回一张表并做结构校验 */
static bool read_table(int fd, std::vector<chunk_ent> &out, uint64_t expect_magic,
                       const char *who)
{
    struct chunk_table_hdr h;
    if (pread(fd, &h, sizeof(h), 0) != (ssize_t)sizeof(h)) {
        checkf(false, "%s: pread 表头失败", who);
        return false;
    }
    checkf(h.magic == expect_magic, "%s: magic 0x%lx 期望 0x%lx", who, h.magic, expect_magic);
    if (h.magic != expect_magic)
        return false;

    size_t bytes = h.nr_chunks * sizeof(struct chunk_ent);
    std::vector<uint8_t> raw(bytes);
    if (bytes && pread(fd, raw.data(), bytes, sizeof(h)) != (ssize_t)bytes) {
        checkf(false, "%s: pread %zu 字节表体失败", who, bytes);
        return false;
    }
    const struct chunk_ent *e = (const struct chunk_ent *)raw.data();
    uint64_t cov = h.base_offset;
    for (uint64_t i = 0; i < h.nr_chunks; i++) {
        if (e[i].offset != cov)
            checkf(false, "%s: 第 %lu 条 offset %lu != 期望 %lu", who, i, e[i].offset, cov);
        if (!(e[i].flags & CHUNK_ENT_CRC_VALID))
            checkf(false, "%s: 第 %lu 条缺 CRC_VALID", who, i);
        out.push_back(e[i]);
        cov += e[i].length;
    }
    checkf(cov - h.base_offset == h.data_len, "%s: 覆盖 %lu != data_len %lu",
           who, cov - h.base_offset, h.data_len);
    return true;
}

static bool cmp_entries(const std::vector<chunk_ent> &got, const std::vector<chunk_ent> &want,
                        const char *who)
{
    checkf(got.size() == want.size(), "%s: 条目数 %zu != 参考 %zu", who, got.size(), want.size());
    size_t n = got.size() < want.size() ? got.size() : want.size();
    for (size_t i = 0; i < n; i++) {
        if (got[i].offset != want[i].offset || got[i].length != want[i].length ||
            got[i].crc32 != want[i].crc32) {
            checkf(false, "%s: 第 %zu 块 (off %lu len %lu crc %08x) != 参考 (off %lu len %lu crc %08x)",
                   who, i, got[i].offset, got[i].length, got[i].crc32,
                   want[i].offset, want[i].length, want[i].crc32);
            return false;
        }
    }
    return got.size() == want.size();
}

int main(int argc, char **argv)
{
    const char *chunk_so = "./build/chunk_csf.so";
    const char *merge_so = "./build/merge_csf.so";
    const char *host_header = "/home/zyt/parasync/src/fastcdc.h";
    const char *ctl_path = "/dev/nvme0c3";
    const char *ng_path = "/dev/ng0n3";
    const char *fdm_dir = "/mnt/fdm0";
    int nseg = 3;
    int size_mb = 4;
    int pattern = 0;
    bool uring = HAVE_URING_CMD ? true : false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--chunk-so") && i + 1 < argc)
            chunk_so = argv[++i];
        else if (!strcmp(argv[i], "--merge-so") && i + 1 < argc)
            merge_so = argv[++i];
        else if (!strcmp(argv[i], "--host-header") && i + 1 < argc)
            host_header = argv[++i];
        else if (!strcmp(argv[i], "--ctl") && i + 1 < argc)
            ctl_path = argv[++i];
        else if (!strcmp(argv[i], "--ng") && i + 1 < argc)
            ng_path = argv[++i];
        else if (!strcmp(argv[i], "--fdm-dir") && i + 1 < argc)
            fdm_dir = argv[++i];
        else if (!strcmp(argv[i], "--segments") && i + 1 < argc)
            nseg = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--size-mb") && i + 1 < argc)
            size_mb = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pattern") && i + 1 < argc)
            pattern = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sync"))
            uring = false;
        else {
            fprintf(stderr, "未知参数 %s\n", argv[i]);
            return 1;
        }
    }

    if (nseg < 1 || nseg > URG_QUEUE_DEPTH ||
        (uint64_t)nseg > ((uint64_t)size_mb << 20) || nseg + 1 > 127) {
        fprintf(stderr, "段数 %d 非法（要求 1 <= k <= %d，且 <= 总字节数，且 k+1 <= 127）\n",
                nseg, URG_QUEUE_DEPTH);
        return 1;
    }
#if !HAVE_URING_CMD
    if (uring) {
        info("liburing < 2.2（缺 URING_CMD/SQE128），自动退化为 ioctl 串行提交；"
             "并发路径见 Makefile 里的 LIBURING_PREFIX");
        uring = false;
    }
#endif

    printf("== L1: CDC 算子链在 CEMU CSD 上的通路验证 ==\n");
    printf("Stage-1 %s\nStage-2 %s\n提交方式: %s\n",
           chunk_so, merge_so, uring ? "io_uring(并发)" : "ioctl(串行)");

    /* ---------- 1. 加载并激活两个算子 ---------- */
    struct ioctl_download dl_c {}, dl_m {};
    prep_shared_library(chunk_so, "chunk_csf", &dl_c);
    prep_shared_library(merge_so, "merge_csf", &dl_m);

    int ctl_fd = open(ctl_path, O_RDWR);
    if (ctl_fd < 0) {
        perror("open ctl");
        return 1;
    }
    int ng_fd = open(ng_path, O_RDWR);
    if (ng_fd < 0) {
        perror("open ng");
        return 1;
    }
    if (ioctl(ctl_fd, IOCTL_CEMU_DOWNLOAD, &dl_c) <= 0 ||
        ioctl(ctl_fd, IOCTL_CEMU_ACTIVATE, &dl_c)) {
        perror("download/activate chunk_csf");
        return 1;
    }
    if (ioctl(ctl_fd, IOCTL_CEMU_DOWNLOAD, &dl_m) <= 0 ||
        ioctl(ctl_fd, IOCTL_CEMU_ACTIVATE, &dl_m)) {
        perror("download/activate merge_csf");
        return 1;
    }
    printf("pind: chunk_csf=%d, merge_csf=%d\n", dl_c.pind, dl_m.pind);

    /* ---------- 2. 打开 FDM 文件 ---------- */
    std::vector<int> fdm;
    for (int i = 0; i < 32; i++) {
        int fd = open_fdm(fdm_dir, i);
        if (fd < 0)
            break;
        fdm.push_back(fd);
    }
    if ((int)fdm.size() < nseg + 1) {
        fprintf(stderr, "%s 下只有 %zu 个 FDM 文件，跑 %d 段需要 %d 个\n"
                        "（先把 tests/cemu/prep_data.sh 的文件数调大）\n",
                fdm_dir, fdm.size(), nseg, nseg + 1);
        return 1;
    }
    int fd_data = fdm[nseg];                    /* 整文件数据 */
    int fd_merged = ((int)fdm.size() >= nseg + 2) ? fdm[nseg + 1] : fd_data;
    printf("FDM 文件 %zu 个: 表=[0..%d), 数据=%d, 合并表=%d\n",
           fdm.size(), nseg, fd_data, fd_merged);

    /* ---------- 3. 生成数据并写进设备侧 ---------- */
    uint64_t n = (uint64_t)size_mb << 20;
    uint64_t seg = (n + nseg - 1) / nseg;
    uint8_t *data = (uint8_t *)aligned_alloc(4096, ((n + 4095) / 4096) * 4096);
    if (!data) {
        fprintf(stderr, "alloc %lu 失败\n", n);
        return 1;
    }
    memset(data, 0, ((n + 4095) / 4096) * 4096);
    gen(data, n, pattern, 12345);
    /* 写 512B 对齐的长度，但 MRS 只暴露精确的段长（cparam1 决定有效字节） */
    uint64_t wr = ((n + 511) / 512) * 512;
    if (pwrite(fd_data, data, wr, 0) != (ssize_t)wr) {
        perror("pwrite 数据到 FDM");
        return 1;
    }

    /* 表容量：平均块 ~8KB，一条 24B；给足 4 倍余量 + 表头 */
    uint64_t tbl_cap = sizeof(struct chunk_table_hdr) + (n / 8192 + 16) * 4 * sizeof(struct chunk_ent);
    tbl_cap = ((tbl_cap + 511) / 512) * 512;

    /* ---------- 4. 参考实现（同一份主机语义） ---------- */
    uint64_t gear[256];
    bool have_gear = load_host_gear(host_header, gear);
    if (!have_gear)
        info("读不到 %s 的 GEARv2，跳过逐项内容比对（仍会打印指纹，可与 L0 对拍）",
             host_header);

    std::vector<std::vector<chunk_ent>> ref_seg(nseg);
    std::vector<chunk_ent> ref_raw;
    if (have_gear) {
        for (int s = 0; s < nseg; s++) {
            uint64_t off = (uint64_t)s * seg;
            if (off >= n)
                break;
            uint64_t len = n - off;
            if (len > seg)
                len = seg;
            ref_seg_cuts(data, off, len, gear, ref_seg[s]);
            ref_raw.insert(ref_raw.end(), ref_seg[s].begin(), ref_seg[s].end());
        }
    }
    std::vector<chunk_ent> ref_merged = have_gear ? ref_detector(ref_raw, data)
                                                  : std::vector<chunk_ent>();

    /* ---------- 5. Stage-1：每段一个 MRS、一个作业 ---------- */
    std::vector<int> rsid_s1(nseg, -1);
    std::vector<ExecReq> reqs(nseg);
    for (int s = 0; s < nseg; s++) {
        uint64_t off = (uint64_t)s * seg;
        uint64_t len = (off < n) ? ((n - off > seg) ? seg : n - off) : 0;
        int mfd[2] = {fdm[s], fd_data};
        long long moff[2] = {0, (long long)off};
        long long msz[2] = {(long long)tbl_cap, (long long)len};
        rsid_s1[s] = create_mrs(ctl_fd, mfd, moff, msz, 2);
        if (rsid_s1[s] < 0)
            return 1;
        reqs[s].c1 = len;             /* cparam1 = 段内有效字节数 */
        reqs[s].c2 = off;             /* cparam2 = 段在文件中的起始偏移 */
        reqs[s].pind = dl_c.pind;
        reqs[s].rsid = (uint16_t)rsid_s1[s];
        reqs[s].tag = "stage1";
    }

    auto t0 = std::chrono::steady_clock::now();
    if (!submit_batch(ng_fd, uring, reqs))
        return 1;
    double t_stage1 = ms_since(t0);

    std::vector<std::vector<chunk_ent>> got_seg(nseg);
    uint64_t expected_total = 0;
    for (int s = 0; s < nseg; s++) {
        uint64_t off = (uint64_t)s * seg;
        uint64_t len = (off < n) ? ((n - off > seg) ? seg : n - off) : 0;
        checkf(!reqs[s].err, "Stage-1 段 %d cqe 报错", s);
        checkf(reqs[s].res >= 0, "Stage-1 段 %d 返回 %lld（应为切点数）", s, reqs[s].res);
        if (have_gear)
            checkf((uint64_t)reqs[s].res == ref_seg[s].size(),
                   "Stage-1 段 %d 返回值 %lld != 参考切点数 %zu",
                   s, reqs[s].res, ref_seg[s].size());
        expected_total += (uint64_t)reqs[s].res;
        if (reqs[s].res < 0)
            continue;
        read_table(fdm[s], got_seg[s], CHUNK_TABLE_MAGIC, "Stage-1 表");
        if (have_gear)
            cmp_entries(got_seg[s], ref_seg[s], "Stage-1 表");
        if (len == 0)
            checkf(got_seg[s].empty(), "Stage-1 段 %d 空段却有 %zu 条", s, got_seg[s].size());
    }
    printf("Stage-1: %d 个作业, 合计切点 %lu, %.3f ms (单作业均值 %.3f ms)\n",
           nseg, expected_total, t_stage1, t_stage1 / nseg);

    /* ---------- 6. Stage-2：一个作业吃掉所有段表 ---------- */
    std::vector<int> mfd;
    std::vector<long long> moff, msz;
    mfd.push_back(fd_merged);
    moff.push_back(0);
    msz.push_back((long long)tbl_cap);
    for (int s = 0; s < nseg; s++) {
        mfd.push_back(fdm[s]);
        moff.push_back(0);
        msz.push_back((long long)tbl_cap);
    }
    int rsid_s2 = create_mrs(ctl_fd, mfd.data(), moff.data(), msz.data(), (int)mfd.size());
    if (rsid_s2 < 0)
        return 1;

    ExecReq r2;
    r2.c1 = (uint64_t)nseg; /* cparam1 = 段数 k */
    r2.c2 = 0;
    r2.pind = dl_m.pind;
    r2.rsid = (uint16_t)rsid_s2;
    r2.tag = "stage2";
    std::vector<ExecReq> reqs2 = {r2};

    t0 = std::chrono::steady_clock::now();
    if (!submit_batch(ng_fd, uring, reqs2))
        return 1;
    double t_stage2 = ms_since(t0);
    checkf(!reqs2[0].err, "Stage-2 cqe 报错");
    checkf(reqs2[0].res >= 0, "Stage-2 返回 %lld（应为合并块数）", reqs2[0].res);
    if (have_gear)
        checkf((uint64_t)reqs2[0].res == ref_merged.size(),
               "Stage-2 返回值 %lld != 参考块数 %zu", reqs2[0].res, ref_merged.size());

    std::vector<chunk_ent> got_merged;
    read_table(fd_merged, got_merged, CHUNK_TABLE_MAGIC_MERGED, "Stage-2 表");
    if (have_gear)
        cmp_entries(got_merged, ref_merged, "Stage-2 表");

    /* 覆盖性：合并表必须覆盖整个文件 */
    if (!got_merged.empty()) {
        uint64_t b = got_merged.front().offset;
        uint64_t e = got_merged.back().offset + got_merged.back().length;
        checkf(b == 0 && e == n, "合并表覆盖 [%lu,%lu) != [0,%lu)", b, e, n);
        for (size_t i = 1; i < got_merged.size(); i++)
            checkf(got_merged[i].offset == got_merged[i - 1].offset + got_merged[i - 1].length,
                   "合并表第 %zu 条与前一条不连续", i);
    }
    printf("Stage-2: 合并后 %zu 块, %.3f ms\n", got_merged.size(), t_stage2);

    /* ---------- 7. 指纹（与 L0 / 主机对拍用） ---------- */
    printf("\n合并表 FNV-1a64 指纹 (n=%lu B, %d 段, pattern %d): 0x%016lx\n",
           n, nseg, pattern, fnv1a(got_merged));
    printf("主机侧复核: ./build/test_chunk_csf --size-mb %d --segments %d 应得到同一指纹\n",
           size_mb, nseg);

    /* ---------- 8. 清理 ---------- */
    for (int s = 0; s < nseg; s++)
        if (rsid_s1[s] >= 0)
            delete_mrs(ctl_fd, rsid_s1[s]);
    delete_mrs(ctl_fd, rsid_s2);
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl_c);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl_c);
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl_m);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl_m);
    close(ctl_fd);
    close(ng_fd);
    for (int fd : fdm)
        close(fd);
    free(data);

    printf("\n== 结果: %s (失败 %d 项) ==\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
