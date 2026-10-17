/*
 * read_file.cpp —— 最小验证：让 CSD 自己读取一个文件并输出
 *
 * 数据通路（全部在 CSD 内部完成，主机只下发"地址清单"）：
 *
 *   主机: FIEMAP 解析文件 -> NvmeCopyFormat2{nsid=1, slba, nlb}[]
 *         -> pack_indirect_task_buffer -> execute 命令的 data_buffer(PRP)
 *   设备: compute.c 解析 extent 列表 -> memory_copy_nvm 从 NVM 搬到 FDM
 *         -> read_csf.so 在设备内执行：打印预览 + 复制到 mr[0]
 *   主机: pread("/mnt/fdm0/0") 与源文件逐字节比对
 *
 * 前置条件：
 *   - 在 CEMU 虚拟机内运行（需要 /dev/nvme0c3 与 /dev/ng0n3）
 *   - 目标文件位于 NVM 命名空间上，即 /mnt/nvme0/ 之下（FIEMAP 的 LBA 才有意义）
 *   - /mnt/fdm0/0 与 /mnt/fdm0/1 已存在（prep_data.sh 创建）
 *   - 已 make -C tests/cemu/kernel all 生成 build/read_csf.so
 *
 * 用法：
 *   ./build/read_file [文件路径]        默认 /mnt/nvme0/hello.txt
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

/*
 * 必须走 io_uring 而非 NVME_IOCTL_IO_CMD：
 * 内核 nvme_user_cmd() 不会把 cmd.metadata 拷进 c.common.metadata，而
 * NvmeProgramExecuteCmd.dlen 恰好位于该偏移，导致设备收到 dlen=0 并返回
 * NVME_INVALID_FIELD。io_uring 路径 (ioctl.c:462) 有 nsid==3 的特殊处理。
 *
 * 注意包含顺序：<liburing.h> 必须早于任何 <linux/io_uring.h>，
 * 否则系统旧版内核头会用同名 include guard 把 liburing 自带的
 * io_uring.h（含 URING_CMD / SQE128 / big_cqe）整体挡掉。
 */
#include <liburing.h>
#include "util.h"

#define IOU_QUEUE_DEPTH 64

#define CSD_CTL_DEV "/dev/nvme0c3"   /* compute 命名空间的 CEMU 控制节点 */
#define CSD_NG_DEV  "/dev/ng0n3"     /* nsid=3 的 generic char device */
#define FDM_OUT     "/mnt/fdm0/0"    /* mr[0]: CSF 输出，留在 FDM */
#define FDM_IN      "/mnt/fdm0/1"    /* mr[1]: NVM 数据搬入的暂存区 */

/* 读满整个文件；read() 可能短读，必须循环 */
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

static void dump_hex(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        printf("%02x", p[i]);
    }
}

/* 同步提交一条 NVMe URING_CMD；creq 里的 64B 命令原样透传给设备 */
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
    sqe->cmd_op = NVME_URING_CMD_IO;   /* 64B 命令走 SQE128 的 cmd 区 */
    sqe->fd     = ng_fd;
    memcpy(sqe->cmd, cmd, sizeof(*cmd));

    if (io_uring_submit(&ring) < 0) {
        io_uring_queue_exit(&ring);
        return -1;
    }

    struct io_uring_cqe *cqe = nullptr;
    int ret = io_uring_wait_cqe(&ring, &cqe);
    if (ret < 0 || !cqe) {
        io_uring_queue_exit(&ring);
        return -1;
    }

    ret = cqe->res;                              /* 负值 = 内核错误码 */
    if (out_result && ret >= 0) {
        *out_result = (long long)cqe->big_cqe[0]; /* 设备返回的 64bit 结果 */
    }
    io_uring_cqe_seen(&ring, cqe);
    io_uring_queue_exit(&ring);

    return ret < 0 ? ret : 0;
}

int main(int argc, char **argv)
{
    const std::string path = (argc > 1) ? argv[1] : "/mnt/nvme0/hello.txt";

    printf("=== CSD 内部读取文件最小测试 ===\n");
    printf("目标文件: %s\n", path.c_str());

    if (path.rfind("/mnt/nvme0/", 0) != 0) {
        printf("[WARN] 文件不在 /mnt/nvme0/ 下，FIEMAP 得到的 LBA 可能不对应这块数据\n");
    }

    /* ---------- 1. 取文件大小并做对齐检查 ---------- */
    struct stat st {};
    if (stat(path.c_str(), &st) < 0) {
        perror("stat");
        return 1;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "不是普通文件: %s\n", path.c_str());
        return 1;
    }
    const uint64_t file_size = (uint64_t)st.st_size;
    if (file_size == 0 || file_size % LBA_SIZE != 0) {
        fprintf(stderr, "文件大小必须是 512B 的正整数倍，当前 %lu\n", file_size);
        return 1;
    }
    /* chunk_nlb 是 24bit 字段 */
    if (file_size / LBA_SIZE > 0xFFFFFFULL) {
        fprintf(stderr, "文件过大（chunk_nlb 为 24bit）: %lu\n", file_size);
        return 1;
    }

    /* ---------- 2. 下载并激活 CSF ---------- */
    struct ioctl_download dl {};
    dl.name = "read_csf";
    dl.indirect = 1;
    prep_shared_library("./build/read_csf.so", "read_csf", &dl);

    int ctl_fd = open(CSD_CTL_DEV, O_RDWR);
    if (ctl_fd < 0) {
        perror("open " CSD_CTL_DEV);
        return 1;
    }

    /* 先清理上次异常退出残留在设备里的同名程序（不存在时失败可忽略） */
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);

    errno = 0;
    int ret = ioctl(ctl_fd, IOCTL_CEMU_DOWNLOAD, &dl);
    if (ret <= 0) {
        if (errno == EEXIST) {
            /* 内核在 EEXIST 时也会回填 pind（cemu.c:237），直接复用 */
            printf("复用已加载的同名 CSF: pind=%d\n", dl.pind);
        } else {
            perror("IOCTL_CEMU_DOWNLOAD");
            return 1;
        }
    } else {
        printf("CSF 已下载: pind=%d\n", dl.pind);
    }

    /*
     * 注意：该 ioctl 失败时返回的是"正的 NVMe status"（如 2 = INVALID_FIELD），
     * 不是负 errno，所以只能用 ret < 0 判定真正的系统调用失败。
     * 程序已处于激活态时会返回 INVALID_FIELD，可安全继续。
     */
    ret = ioctl(ctl_fd, IOCTL_CEMU_ACTIVATE, &dl);
    if (ret < 0) {
        perror("IOCTL_CEMU_ACTIVATE");
        return 1;
    }

    /* ---------- 3. 用两个 FDM 文件建内存区间 ---------- */
    int out_fd = open(FDM_OUT, O_RDWR);
    int in_fd  = open(FDM_IN,  O_RDWR);
    if (out_fd < 0 || in_fd < 0) {
        perror("open fdm data file (先执行 prep_data.sh)");
        return 1;
    }

    struct ioctl_create_mrs mrs {};
    prep_mrs(&mrs, 2);
    mrs.fd[0] = out_fd; mrs.off[0] = 0; mrs.size[0] = (long long)file_size;
    mrs.fd[1] = in_fd;  mrs.off[1] = 0; mrs.size[1] = (long long)file_size;

    ret = ioctl(ctl_fd, IOCTL_CEMU_CREATE_MRS, &mrs);
    if (ret != 0) {
        perror("IOCTL_CEMU_CREATE_MRS");
        return 1;
    }
    printf("内存区间已建立: rsid=%u, mr[0]=输出, mr[1]=输入暂存\n", mrs.rsid);

    /* ---------- 4. FIEMAP：把文件解析成扁平 extent 列表 ---------- */
    uint64_t mapped_size = file_size;
    NvmeCopyFormat2 *cf = nullptr;
    int nlb_per_chunk[1] = { 0 };   /* 0 表示由 get_file_mappings 自动均分 */
    int nr_cf_per_chunk[1] = { 0 };

    auto t_fiemap_0 = std::chrono::high_resolution_clock::now();
    int nr_cf = get_file_mappings(path.c_str(), &mapped_size, &cf, 1,
                                  nlb_per_chunk, nr_cf_per_chunk);
    auto t_fiemap_1 = std::chrono::high_resolution_clock::now();

    if (nr_cf <= 0 || !cf) {
        fprintf(stderr, "get_file_mappings 失败（文件可能不在支持 FIEMAP 的文件系统上）\n");
        return 1;
    }
    if (mapped_size != file_size) {
        fprintf(stderr, "FIEMAP 覆盖 %lu B != 文件大小 %lu B\n", mapped_size, file_size);
        free(cf);
        return 1;
    }

    printf("FIEMAP 解析: %d 条 extent, 耗时 %ld us\n", nr_cf,
           (long)(std::chrono::duration_cast<std::chrono::microseconds>(t_fiemap_1 - t_fiemap_0).count()));
    for (int i = 0; i < nr_cf && i < 4; i++) {
        printf("  extent[%d]: nsid=%u slba=%lu nlb=%u (%u B)\n",
               i, cf[i].snsid, (unsigned long)cf[i].slba,
               (unsigned)cf[i].nlb + 1, (unsigned)(cf[i].nlb + 1) * LBA_SIZE);
    }
    if (nr_cf > 4) {
        printf("  ... 其余 %d 条省略\n", nr_cf - 4);
    }

    /* ---------- 5. 打包 indirect task（destination=0：输出留 FDM） ---------- */
    uint64_t data_len = file_size;
    const size_t extra_sz = sizeof(data_len);
    const size_t req_sz = calc_indirect_task_buffer_size(1, 0, nr_cf, 0, extra_sz);

    std::vector<uint8_t> task(req_sz, 0);
    size_t packed = pack_indirect_task_buffer(task.data(),
                                              /*parallel_chunks=*/1,
                                              /*destination=*/0,
                                              /*read_mode=*/CSD_READ_MODE_DATA,
                                              nr_cf, nr_cf_per_chunk, cf,
                                              /*nr_out_cf=*/0, nullptr, nullptr,
                                              &data_len, extra_sz);
    if (packed != req_sz) {
        fprintf(stderr, "打包长度 %zu != 预算 %zu\n", packed, req_sz);
        free(cf);
        return 1;
    }
    printf("描述符已打包: %zu B (占文件 %.2f%%)\n",
           req_sz, (double)req_sz * 100.0 / (double)file_size);

    /* ---------- 6. 提交到 CSD（io_uring 直通：唯一能携带 dlen 的路径） ---------- */
    int ng_fd = open(CSD_NG_DEV, O_RDWR);
    if (ng_fd < 0) {
        perror("open " CSD_NG_DEV);
        return 1;
    }

    struct nvme_uring_cmd cmd {};
    const uint32_t chunk_nlb = (uint32_t)(file_size / LBA_SIZE) - 1;  /* 设备侧 +1 */
    prep_nvme_uring_program_execute(&cmd,
                                    /*cparam1=*/0,
                                    /*cparam2=*/0,
                                    /*pind=*/dl.pind,
                                    /*rsid=*/mrs.rsid,
                                    /*numr=*/nr_cf,
                                    /*group=*/0,
                                    /*chunk_nlb=*/chunk_nlb,
                                    /*user_runtime=*/0,
                                    /*data=*/task.data(),
                                    /*data_len=*/req_sz);

    printf("提交 execute 命令: nr_cf=%d, chunk_nlb=%u\n", nr_cf, chunk_nlb);

    long long csf_ret = -1;
    auto t0 = std::chrono::high_resolution_clock::now();
    int cmd_ret = submit_execute_cmd(ng_fd, &cmd, &csf_ret);
    auto t1 = std::chrono::high_resolution_clock::now();

    if (cmd_ret < 0) {
        fprintf(stderr, "execute 命令失败: %d (%s)\n", cmd_ret, strerror(-cmd_ret));
        return 1;
    }
    printf("命令完成: 设备端 CSF 返回 %lld 字节, 往返耗时 %ld us\n",
           csf_ret,
           (long)(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));

    /* ---------- 7. 校验：CSD 读到的内容是否等于文件内容 ---------- */
    /* fdmfs 的 pread 要求缓冲区 4096 对齐，否则返回 EINVAL */
    void *got_raw = nullptr;
    if (posix_memalign(&got_raw, 4096, (size_t)file_size) != 0 || !got_raw) {
        fprintf(stderr, "posix_memalign 失败\n");
        return 1;
    }
    uint8_t *got = (uint8_t *)got_raw;
    memset(got, 0, (size_t)file_size);
    ssize_t r = pread(out_fd, got, file_size, 0);
    if (r != (ssize_t)file_size) {
        perror("pread " FDM_OUT);
        free(got);
        return 1;
    }

    int src_fd = open(path.c_str(), O_RDONLY);
    if (src_fd < 0) {
        perror("open source");
        return 1;
    }
    std::vector<uint8_t> want(file_size, 0);
    if (!read_full(src_fd, want.data(), file_size)) {
        perror("read source");
        return 1;
    }
    close(src_fd);

    const int cmp = memcmp(got, want.data(), file_size);

    printf("\n--- 结果 ---\n");
    printf("文件大小      : %lu B\n", (unsigned long)file_size);
    printf("CSD 读回首 32B: ");
    dump_hex(got, (size_t)std::min<uint64_t>(32, file_size));
    printf("\n");
    printf("源文件 首 32B: ");
    dump_hex(want.data(), (size_t)std::min<uint64_t>(32, file_size));
    printf("\n");
    printf("%s\n", cmp == 0 ? "[PASS] CSD 读到的内容与文件完全一致"
                            : "[FAIL] 内容不一致");

    /* ---------- 8. 清理 ---------- */
    ioctl(ctl_fd, IOCTL_CEMU_DEACTIVATE, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_UNLOAD, &dl);
    ioctl(ctl_fd, IOCTL_CEMU_DELETE_MRS, &mrs);

    free(mrs.fd); free(mrs.off); free(mrs.size);
    free(got); free(cf);
    close(out_fd); close(in_fd); close(ng_fd); close(ctl_fd);

    return cmp == 0 ? 0 : 1;
}
