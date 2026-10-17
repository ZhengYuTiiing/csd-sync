/*
 * crc_oob_test.cpp —— 验证设备侧"写时把可组合 CRC 固化进 OOB，读时按需拼接"
 *
 * 与 crc_compose_test 的区别：网格不再由主机生成并传入，而是用 read_mode=1
 * 让设备直接浅读 NAND OOB。程序做两项检查：
 *
 *   检查 1：用 collect_oob 把老文件的 OOB 槽位流拉回主机，与主机按同样规则
 *           算出的 8×CRC32C/页 逐字节比对 —— 证明"写时固化"确实发生了。
 *   检查 2：把随机 512B 对齐区间的查询交给 compose_crc，由设备只用槽位拼出
 *           校验和，再与主机独立计算的结果比对 —— 证明"任意边界可拼接"。
 *
 * 需要在虚拟机内运行，文件须位于 /mnt/nvme0 且为 4KB 整数倍。
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
#include "kernel/cemu_def.h"
#include "kernel/rsync_csd_abi.h"
#include "../../hw/femu/backend/csd_fp_crc.h"

#define CSD_CTL_DEV   "/dev/nvme0c3"
#define CSD_NG_DEV    "/dev/ng0n3"
#define FDM_OUT       "/mnt/fdm0/0"
#define FDM_OOB_IN    "/mnt/fdm0/1"

#define BLK 4096u

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

static int submit_execute_cmd(int ng_fd, const struct nvme_uring_cmd *cmd,
                              long long *out_result)
{
    struct io_uring ring;
    if (io_uring_queue_init(64, &ring, IORING_SETUP_SQE128 | IORING_SETUP_CQE32) < 0) {
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

/* 提交一条"浅读 OOB"的 indirect 命令：输入是老文件的物理区间，输出为算子结果 */
static int run_oob_cmd(int ng_fd, uint16_t pind, uint16_t rsid,
                       const NvmeCopyFormat2 *in_cf, int nr_in,
                       const void *extra, size_t extra_len,
                       uint32_t chunk_nlb, long long *ret_out)
{
    std::vector<int> in_pc(1, nr_in);
    const size_t need = calc_indirect_task_buffer_size(1, 0, nr_in, 0, extra_len);
    std::vector<uint8_t> task(need, 0);
    size_t packed = pack_indirect_task_buffer(task.data(), 1, 0, CSD_READ_MODE_OOB,
                                              nr_in, in_pc.data(), in_cf,
                                              0, nullptr, nullptr,
                                              extra, extra_len);
    if (packed != need) {
        return -1;
    }

    struct nvme_uring_cmd cmd {};
    prep_nvme_uring_program_execute(&cmd, 0, 0, pind, rsid,
                                    (uint32_t)nr_in, 0, chunk_nlb, 0,
                                    task.data(), (uint32_t)packed);
    return submit_execute_cmd(ng_fd, &cmd, ret_out);
}

int main(int argc, char **argv)
{
    const std::string path = (argc > 1) ? argv[1] : "/mnt/nvme0/dataset/10M-new/10MB";

    printf("=== OOB 固化 + 任意边界拼接（设备侧） ===\n");
    printf("文件: %s\n", path.c_str());

    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) { perror("open"); return 1; }
    struct stat st {};
    if (fstat(fd, &st) < 0) { perror("fstat"); return 1; }
    const size_t size = (size_t)st.st_size;
    if (size == 0 || size % BLK) {
        fprintf(stderr, "文件大小必须是 4KB 的整数倍\n");
        return 1;
    }
    std::vector<uint8_t> data(size);
    if (!read_full(fd, data.data(), size)) { perror("read"); return 1; }
    close(fd);

    const uint32_t nr_pages = (uint32_t)(size / BLK);
    const uint32_t chunk_nlb = (uint32_t)(size / LBA_SIZE) - 1;

    /* 主机侧按同一规则算出期望网格 */
    std::vector<uint8_t> want_grid((size_t)nr_pages * CSD_CRC_SLOT_LEN, 0);
    for (uint32_t p = 0; p < nr_pages; p++) {
        csd_crc_slot_build(data.data() + (size_t)p * BLK,
                           want_grid.data() + (size_t)p * CSD_CRC_SLOT_LEN);
    }

    /* FIEMAP */
    NvmeCopyFormat2 *cf = nullptr;
    uint64_t fsz = 0;
    int pc = 0, pc2 = 0;
    int nr_cf = get_file_mappings(path.c_str(), &fsz, &cf, 1, &pc, &pc2);
    if (nr_cf <= 0 || fsz != size) {
        fprintf(stderr, "FIEMAP 失败\n");
        return 1;
    }

    int ctl_fd = open(CSD_CTL_DEV, O_RDWR);
    if (ctl_fd < 0) { perror("open " CSD_CTL_DEV); return 1; }
    int ng_fd = open(CSD_NG_DEV, O_RDWR);
    if (ng_fd < 0) { perror("open " CSD_NG_DEV); return 1; }
    int out_fd = open(FDM_OUT, O_RDWR);
    int oob_fd = open(FDM_OOB_IN, O_RDWR);
    if (out_fd < 0 || oob_fd < 0) { perror("open fdm"); return 1; }

    const long long grid_bytes = (long long)nr_pages * CSD_CRC_SLOT_LEN;

    /* ---------- 检查 1：把 OOB 槽位流拉回来比对 ---------- */
    uint16_t pind_collect = 0;
    if (load_csf(ctl_fd, "./build/collect_oob.so", "collect_oob", "collect_oob", &pind_collect)) {
        return 1;
    }

    struct ioctl_create_mrs mrs {};
    prep_mrs(&mrs, 2);
    mrs.fd[0] = out_fd; mrs.off[0] = 0; mrs.size[0] = grid_bytes;
    mrs.fd[1] = oob_fd; mrs.off[1] = 0; mrs.size[1] = grid_bytes;
    if (ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs) != 0) {
        perror("CREATE_MRS");
        return 1;
    }

    long long r1 = -1;
    int rc = run_oob_cmd(ng_fd, pind_collect, mrs.rsid, cf, nr_cf,
                         nullptr, 0, chunk_nlb, &r1);
    if (rc < 0) {
        fprintf(stderr, "collect_oob 失败: %d (%s)\n", rc, strerror(-rc));
        return 1;
    }

    void *dump = nullptr;
    if (posix_memalign(&dump, 4096, (size_t)grid_bytes) != 0) {
        fprintf(stderr, "posix_memalign 失败\n");
        return 1;
    }
    memset(dump, 0, (size_t)grid_bytes);
    if (pread(out_fd, dump, (size_t)grid_bytes, 0) != (ssize_t)grid_bytes) {
        perror("pread oob dump");
        return 1;
    }

    const int grid_cmp = memcmp(dump, want_grid.data(), (size_t)grid_bytes);
    printf("检查1 写时固化: OOB 槽位 %lld B, 与主机期望网格 %s\n",
           grid_bytes, grid_cmp == 0 ? "完全一致" : "不一致");

    /* ---------- 检查 2：任意 512B 对齐边界的拼接 ---------- */
    const uint32_t nr_q = 32;
    std::vector<struct crc_query> qs(nr_q);
    srand(20261017);
    const uint64_t max_sec = size / 512;
    for (uint32_t i = 0; i < nr_q; i++) {
        const uint64_t off_sec = (uint64_t)(rand() % (uint32_t)max_sec);
        uint64_t len_sec = 1 + (uint64_t)(rand() % 32);
        if (off_sec + len_sec > max_sec) {
            len_sec = max_sec - off_sec;
        }
        qs[i].off = off_sec * 512;
        qs[i].len = len_sec * 512;
    }

    std::vector<uint8_t> req(sizeof(struct crc_query_hdr) +
                             (size_t)nr_q * sizeof(struct crc_query), 0);
    auto *qh = (struct crc_query_hdr *)req.data();
    qh->magic = RSYNC_CRC_QUERY_MAGIC;
    qh->nr_queries = nr_q;
    memcpy(req.data() + sizeof(*qh), qs.data(), (size_t)nr_q * sizeof(struct crc_query));

    uint16_t pind_compose = 0;
    if (load_csf(ctl_fd, "./build/compose_crc.so", "compose_crc", "compose_crc", &pind_compose)) {
        return 1;
    }

    const long long out_bytes = (long long)(((size_t)nr_q * 4 + 4095) & ~4095ULL);
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs);

    struct ioctl_create_mrs mrs2 {};
    prep_mrs(&mrs2, 2);
    mrs2.fd[0] = out_fd; mrs2.off[0] = 0; mrs2.size[0] = out_bytes;
    mrs2.fd[1] = oob_fd; mrs2.off[1] = 0; mrs2.size[1] = grid_bytes;
    if (ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs2) != 0) {
        perror("CREATE_MRS(compose)");
        return 1;
    }

    long long r2 = -1;
    rc = run_oob_cmd(ng_fd, pind_compose, mrs2.rsid, cf, nr_cf,
                     req.data(), req.size(), chunk_nlb, &r2);
    if (rc < 0) {
        fprintf(stderr, "compose_crc 失败: %d (%s)\n", rc, strerror(-rc));
        return 1;
    }

    void *crc_out = nullptr;
    if (posix_memalign(&crc_out, 4096, (size_t)out_bytes) != 0) {
        fprintf(stderr, "posix_memalign 失败\n");
        return 1;
    }
    memset(crc_out, 0, (size_t)out_bytes);
    if (pread(out_fd, crc_out, (size_t)out_bytes, 0) != (ssize_t)out_bytes) {
        perror("pread crc out");
        return 1;
    }

    uint32_t *got = (uint32_t *)crc_out;
    int bad = 0;
    for (uint32_t i = 0; i < nr_q; i++) {
        const uint32_t want = csd_crc32c(0, data.data() + qs[i].off, qs[i].len);
        if (want != got[i]) {
            if (bad < 5) {
                printf("  MISMATCH q[%u] off=%lu len=%lu: 设备 0x%08x, 主机 0x%08x\n",
                       i, (unsigned long)qs[i].off, (unsigned long)qs[i].len,
                       got[i], want);
            }
            bad++;
        }
    }
    printf("检查2 边界拼接: %u 个随机区间, %s\n", nr_q,
           bad == 0 ? "全部由 OOB 槽位正确拼出" : "存在错误");

    printf("%s\n", (grid_cmp == 0 && bad == 0)
                       ? "[PASS] 写时固化 + 任意边界拼接，均已在设备侧验证"
                       : "[FAIL]");

    /* 清理 */
    struct ioctl_download dl {};
    dl.name = (char *)"collect_oob";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    dl.name = (char *)"compose_crc";
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs2);
    free(mrs2.fd); free(mrs2.off); free(mrs2.size);
    free(cf); free(dump); free(crc_out);
    close(out_fd); close(oob_fd); close(ng_fd); close(ctl_fd);

    return (grid_cmp == 0 && bad == 0) ? 0 : 1;
}
