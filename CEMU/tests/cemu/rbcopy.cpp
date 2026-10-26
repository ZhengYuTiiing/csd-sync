/*
 * rbcopy.cpp —— 方案 D 的最小验证：让设备把 src 整个拷到 dst。
 *
 * 只验证高风险的那几步：out_cf / destination=1 / stage 2 写回 / 页缓存失效。
 * 这条通路在本代码库里从没被用过（现有实现一律 destination=0、nr_out_cf=0），
 * 所以先用最小例子确认它能跑通，再接进 gridds 的重建流程。
 *
 * 用法：./rbcopy <src 在 nvme<dev> 上> <dst 在 nvme<dev> 上> <dev> [P]
 *
 * 注意 P 暂时只支持 1：get_file_mappings 按 P 均分时若除不尽，
 * 各 chunk 的 nlb 会不等，而 chunk_nlb 是命令级单值（compute.c:609/704）。
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <vector>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <liburing.h>

#include "util.h"

#define SEC  512ull
#define PAGE 4096ull

static uint64_t round_up_4k(uint64_t v)
{
    return (v + PAGE - 1) & ~(PAGE - 1);
}

struct Dev {
    int  idx = 0;
    int  ctl_fd = -1;
    int  ng_fd = -1;
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
        close(d->ctl_fd);
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

static int submit_execute_cmd(int ng_fd, const struct nvme_uring_cmd *cmd)
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
    if (ret >= 0) {
        fprintf(stderr, "  cqe.res=%d  big_cqe[0]=%lld\n", ret,
                (long long)cqe->big_cqe[0]);
    }
    io_uring_cqe_seen(&ring, cqe);
    io_uring_queue_exit(&ring);
    return ret < 0 ? ret : 0;
}

static int ensure_fdm(const char *dir, int slot, uint64_t need, const char *what)
{
    char path[256];
    uint64_t want = round_up_4k(need);

    snprintf(path, sizeof(path), "%s%d", dir, slot);
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        fprintf(stderr, "open %s (%s): %s\n", path, what, strerror(errno));
        return -1;
    }
    struct stat st {};
    if (fstat(fd, &st) < 0) {
        close(fd);
        return -1;
    }
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
        if (fstat(fd, &st) < 0 || (uint64_t)st.st_size < want) {
            fprintf(stderr, "FDM %s (%s) 需要 %llu B，仍是 %llu B（err=%d）\n",
                    path, what, (unsigned long long)want,
                    (unsigned long long)st.st_size, err);
            close(fd);
            return -1;
        }
    }
    return fd;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "用法: %s <src> <dst> <dev> [P]\n", argv[0]);
        return 2;
    }
    const char *src = argv[1];
    const char *dst = argv[2];
    const int dev = atoi(argv[3]);
    const int P = (argc > 4) ? atoi(argv[4]) : 1;

    if (P != 1) {
        fprintf(stderr, "暂时只支持 P=1（见文件头关于 chunk_nlb 的说明）\n");
        return 2;
    }

    Dev d;
    if (!dev_open(&d, dev)) {
        return 1;
    }

    uint16_t pind = 0;
    if (load_csf(d.ctl_fd, "./build/grid_rb.so", "grid_rb", "grid_rb", &pind) < 0) {
        return 1;
    }
    printf("算子 grid_rb 已装载，pind=%u\n", pind);

    /* ---- 源文件映射 ---- */
    uint64_t src_size = 0;
    NvmeCopyFormat2 *in_cf = nullptr;
    int nlb_pc[1] = {0};
    int ncf_pc[1] = {0};
    int nr_in_cf = get_file_mappings(src, &src_size, &in_cf, P, nlb_pc, ncf_pc);
    if (nr_in_cf <= 0) {
        fprintf(stderr, "取 %s 映射失败\n", src);
        return 1;
    }
    printf("源 %s : %llu B, %d 条 cf, chunk_nlb=%d\n",
           src, (unsigned long long)src_size, nr_in_cf, nlb_pc[0]);

    /*
     * 建目标文件并**真正写一遍零**，不能用 posix_fallocate。
     *
     * 坑：ext4 的 fallocate 产生的是 unwritten extent，设备绕过文件系统直接写 LBA
     * 之后，ext4 仍然认为这些块"未写"，任何 buffered read 都返回零 ——
     * 表现就是设备明明写了、校验却是全 0。
     * 走一遍 pwrite + fsync 让 extent 变成 written，再丢页缓存，读才能看到设备写的内容。
     */
    int fd_dst = open(dst, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd_dst < 0) {
        perror("open dst");
        return 1;
    }
    {
        std::vector<uint8_t> zero(1u << 20, 0);
        uint64_t off = 0;
        while (off < src_size) {
            size_t n = (size_t)((src_size - off) < zero.size() ? (src_size - off) : zero.size());
            if (pwrite(fd_dst, zero.data(), n, (off_t)off) != (ssize_t)n) {
                perror("pwrite zeros");
                return 1;
            }
            off += n;
        }
    }
    fsync(fd_dst);
    posix_fadvise(fd_dst, 0, 0, POSIX_FADV_DONTNEED);
    close(fd_dst);

    uint64_t dst_size = src_size;
    NvmeCopyFormat2 *out_cf = nullptr;
    int ncf_out_pc[1] = {0};
    int nr_out_cf = get_file_mappings(dst, &dst_size, &out_cf, P, nlb_pc, ncf_out_pc);
    if (nr_out_cf <= 0) {
        fprintf(stderr, "取 %s 映射失败（extent 可能还没分配）\n", dst);
        return 1;
    }
    printf("目标 %s: %llu B, %d 条 out cf\n",
           dst, (unsigned long long)dst_size, nr_out_cf);
    printf("  in_cf[0]  slba=%llu nlb=%u   out_cf[0] slba=%llu nlb=%u\n",
           (unsigned long long)in_cf[0].slba, (unsigned)in_cf[0].nlb + 1,
           (unsigned long long)out_cf[0].slba, (unsigned)out_cf[0].nlb + 1);

    /* ---- 2 个 MR：mr[0]=输出缓冲, mr[1]=输入缓冲 ---- */
    const uint64_t chunk_bytes = round_up_4k((uint64_t)nlb_pc[0] * SEC);
    int fd_out_buf = ensure_fdm(d.fdm_dir, 6, chunk_bytes, "重建输出缓冲");
    int fd_in_buf  = ensure_fdm(d.fdm_dir, 7, chunk_bytes, "重建输入缓冲");
    if (fd_out_buf < 0 || fd_in_buf < 0) {
        return 1;
    }

    struct ioctl_create_mrs mrs {};
    prep_mrs(&mrs, 2 * P);
    for (int i = 0; i < P; i++) {
        mrs.fd[2 * i + 0]   = fd_out_buf;
        mrs.off[2 * i + 0]  = 0;
        mrs.size[2 * i + 0] = (long long)chunk_bytes;

        mrs.fd[2 * i + 1]   = fd_in_buf;
        mrs.off[2 * i + 1]  = 0;
        mrs.size[2 * i + 1] = (long long)chunk_bytes;
    }
    if (ioctl(d.ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
        perror("CREATE_MRS");
        return 1;
    }

    /* ---- 打包任务：destination=1，同时给 in_cf 和 out_cf ---- */
    const size_t need = calc_indirect_task_buffer_size(P, 1, nr_in_cf, nr_out_cf, 0);
    std::vector<uint8_t> task(need, 0);
    size_t packed = pack_indirect_task_buffer(task.data(), P, 1, CSD_READ_MODE_DATA,
                                              nr_in_cf, ncf_pc, in_cf,
                                              nr_out_cf, ncf_out_pc, out_cf,
                                              nullptr, 0);
    if (packed != need) {
        fprintf(stderr, "打包长度不符: %zu != %zu\n", packed, need);
        return 1;
    }

    struct nvme_uring_cmd cmd {};
    prep_nvme_uring_program_execute(&cmd, chunk_bytes, 0, pind, mrs.rsid,
                                    (uint32_t)(2 * P), 0,
                                    (uint32_t)((uint64_t)nlb_pc[0] - 1), 0,
                                    task.data(), (uint32_t)packed);

    double t0 = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t0 = (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
    }

    int ret = submit_execute_cmd(d.ng_fd, &cmd);

    double t1 = 0;
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t1 = (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
    }

    {
        struct ioctl_create_mrs dm {};
        dm.rsid = mrs.rsid;
        ioctl(d.ctl_fd, IOCTL_CEMU_DELETE_MRS, &dm);
    }
    close(fd_out_buf);
    close(fd_in_buf);

    if (ret < 0) {
        fprintf(stderr, "设备侧拷贝失败 ret=%d\n", ret);
        return 1;
    }
    printf("设备侧拷贝完成，耗时 %.3f ms（%.1f MB/s）\n",
           t1 - t0, (double)src_size / (t1 - t0) / 1000.0);

    /* ---- 校验：先丢掉目标文件的页缓存，否则可能读到陈旧页 ---- */
    int fdc = open(dst, O_RDONLY);
    if (fdc < 0) {
        perror("open dst for verify");
        return 1;
    }
    posix_fadvise(fdc, 0, 0, POSIX_FADV_DONTNEED);
    int fds = open(src, O_RDONLY);
    if (fds < 0) {
        return 1;
    }
    posix_fadvise(fds, 0, 0, POSIX_FADV_DONTNEED);

    const size_t BUF = 1u << 20;
    std::vector<uint8_t> a(BUF), b(BUF);
    uint64_t off = 0;
    bool same = true;
    while (off < src_size) {
        size_t n = (size_t)((src_size - off) < BUF ? (src_size - off) : BUF);
        if (pread(fds, a.data(), n, (off_t)off) != (ssize_t)n ||
            pread(fdc, b.data(), n, (off_t)off) != (ssize_t)n) {
            fprintf(stderr, "读失败 @%llu\n", (unsigned long long)off);
            return 1;
        }
        if (memcmp(a.data(), b.data(), n) != 0) {
            for (size_t k = 0; k < n; k++) {
                if (a[k] != b[k]) {
                    fprintf(stderr, "首个不一致 @%llu (src=%02x dst=%02x)\n",
                            (unsigned long long)(off + k), a[k], b[k]);
                    break;
                }
            }
            same = false;
            break;
        }
        off += n;
    }
    printf("%s\n", same ? "[PASS] 设备侧拷贝结果与源逐字节一致"
                        : "[FAIL] 结果不一致");
    return same ? 0 : 1;
}
