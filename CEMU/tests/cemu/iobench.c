/*
 * iobench.c —— 量清楚「重建」这一步到底慢在哪
 *
 * 背景：gridds/gridpipe 的最后一步是
 *     copy_file_range(/mnt/nvme1/10MB -> /mnt/nvme1/sync/xxx.out)
 * 10.4MB 要 148ms，只有 70 MB/s。而 FEMU 的命名空间是 DRAM 兜底的，
 * 这个数字慢得可疑。所以分项测：纯读、纯写、以及几种拷贝方式各多快。
 *
 * 用法（必须在 VM 里以 root 跑，drop_caches 需要 root）：
 *     ./iobench <源文件> <目标目录>
 * 例：
 *     ./iobench /mnt/nvme1/dataset/10M/10MB /mnt/nvme1/sync
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/sendfile.h>

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void drop_caches(void)
{
    int fd = open("/proc/sys/vm/drop_caches", O_WRONLY);
    if (fd < 0) {
        return;
    }
    write(fd, "3\n", 2);
    close(fd);
}

static void *aligned_buf(size_t n)
{
    void *p = NULL;
    if (posix_memalign(&p, 4096, n) != 0) {
        return NULL;
    }
    memset(p, 0, n);
    return p;
}

static double mbps(uint64_t bytes, double ms)
{
    return ms > 0.0 ? (double)bytes / (ms / 1000.0) / 1e6 : 0.0;
}

/* 纯读：pread 顺序读完整个文件 */
static double bench_read(const char *path, uint64_t size, size_t bs)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return -1.0;
    }
    void *buf = aligned_buf(bs);
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < bs ? (size - done) : bs);
        ssize_t r = pread(fd, buf, n, (off_t)done);
        if (r <= 0) {
            break;
        }
        done += (uint64_t)r;
    }
    double t1 = now_ms();
    free(buf);
    close(fd);
    return t1 - t0;
}

/* 纯写：pwrite 顺序写一个新文件 */
static double bench_write(const char *path, uint64_t size, size_t bs)
{
    int fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        return -1.0;
    }
    void *buf = aligned_buf(bs);
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < bs ? (size - done) : bs);
        ssize_t r = pwrite(fd, buf, n, (off_t)done);
        if (r <= 0) {
            break;
        }
        done += (uint64_t)r;
    }
    double t1 = now_ms();
    free(buf);
    close(fd);
    unlink(path);
    return t1 - t0;
}

/* 写完再 fsync：量「真正落盘」比「只写进页缓存」多花多少 */
static double bench_write_sync(const char *path, uint64_t size, size_t bs)
{
    int fd = open(path, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        return -1.0;
    }
    void *buf = aligned_buf(bs);
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < bs ? (size - done) : bs);
        ssize_t r = pwrite(fd, buf, n, (off_t)done);
        if (r <= 0) {
            break;
        }
        done += (uint64_t)r;
    }
    double t_write = now_ms() - t0;
    double f0 = now_ms();
    fsync(fd);
    double t_fsync = now_ms() - f0;
    free(buf);
    close(fd);
    unlink(path);
    printf("%-34s %10.2f %10s   (其中 fsync %.2f ms)\n",
           "  └ 上项 + fsync 落盘", t_write + t_fsync, "-", t_fsync);
    return t_write;
}

/* copy_file_range，chunk = 单次调用请求的最大长度 */
static double bench_cfr(const char *src, const char *dst, uint64_t size, size_t chunk)
{
    int fd_in = open(src, O_RDONLY);
    int fd_out = open(dst, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd_in < 0 || fd_out < 0) {
        return -1.0;
    }
    off_t in = 0, out = 0;
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < chunk ? (size - done) : chunk);
        ssize_t r = copy_file_range(fd_in, &in, fd_out, &out, n, 0);
        if (r <= 0) {
            break;
        }
        done += (uint64_t)r;
    }
    double t1 = now_ms();
    close(fd_in);
    close(fd_out);
    unlink(dst);
    return t1 - t0;
}

/* pread + pwrite */
static double bench_rw(const char *src, const char *dst, uint64_t size, size_t bs)
{
    int fd_in = open(src, O_RDONLY);
    int fd_out = open(dst, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd_in < 0 || fd_out < 0) {
        return -1.0;
    }
    void *buf = aligned_buf(bs);
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < bs ? (size - done) : bs);
        ssize_t r = pread(fd_in, buf, n, (off_t)done);
        if (r <= 0) {
            break;
        }
        if (pwrite(fd_out, buf, (size_t)r, (off_t)done) != r) {
            break;
        }
        done += (uint64_t)r;
    }
    double t1 = now_ms();
    free(buf);
    close(fd_in);
    close(fd_out);
    unlink(dst);
    return t1 - t0;
}

/* sendfile */
static double bench_sendfile(const char *src, const char *dst, uint64_t size)
{
    int fd_in = open(src, O_RDONLY);
    int fd_out = open(dst, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (fd_in < 0 || fd_out < 0) {
        return -1.0;
    }
    off_t off = 0;
    uint64_t done = 0;
    double t0 = now_ms();
    while (done < size) {
        size_t n = (size_t)((size - done) < (1u << 30) ? (size - done) : (1u << 30));
        ssize_t r = sendfile(fd_out, fd_in, &off, n);
        if (r <= 0) {
            break;
        }
        done += (uint64_t)r;
    }
    double t1 = now_ms();
    close(fd_in);
    close(fd_out);
    unlink(dst);
    return t1 - t0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        printf("用法: %s <源文件> <目标目录>\n", argv[0]);
        return 1;
    }
    const char *src = argv[1];
    char dst[512];

    struct stat st;
    memset(&st, 0, sizeof(st));
    if (stat(src, &st) < 0) {
        perror("stat 源文件");
        return 1;
    }
    uint64_t size = (uint64_t)st.st_size;
    printf("源文件 %s  %llu B (%.1f MB)\n\n", src,
           (unsigned long long)size, (double)size / 1e6);
    printf("%-34s %10s %10s\n", "项目", "耗时ms", "MB/s");
    printf("--------------------------------------------------\n");

#define RUN(label, expr)                                                    \
    do {                                                                    \
        snprintf(dst, sizeof(dst), "%s/.iobench.tmp", argv[2]);             \
        drop_caches();                                                      \
        double _ms = (expr);                                                \
        if (_ms < 0) {                                                      \
            printf("%-34s %10s %10s\n", label, "失败", "-");                \
        } else {                                                            \
            printf("%-34s %10.2f %10.1f\n", label, _ms, mbps(size, _ms));   \
        }                                                                   \
    } while (0)

    RUN("纯读 pread  bs=4MB",    bench_read(src, size, 4u << 20));
    RUN("纯读 pread  bs=1MB",    bench_read(src, size, 1u << 20));
    RUN("纯读 pread  bs=256KB",  bench_read(src, size, 256u << 10));
    RUN("纯读 pread  bs=64KB",   bench_read(src, size, 64u << 10));
    RUN("纯读 pread  bs=32KB",   bench_read(src, size, 32u << 10));
    RUN("纯写 pwrite bs=4MB",    bench_write(dst, size, 4u << 20));
    RUN("纯写 pwrite bs=64KB",   bench_write(dst, size, 64u << 10));
    RUN("纯写+fsync 落盘 bs=64KB", bench_write_sync(dst, size, 64u << 10));
    RUN("copy_file_range chunk=1GB",  bench_cfr(src, dst, size, 1u << 30));
    RUN("copy_file_range chunk=1MB",  bench_cfr(src, dst, size, 1u << 20));
    RUN("copy_file_range chunk=256KB", bench_cfr(src, dst, size, 256u << 10));
    RUN("copy_file_range chunk=64KB", bench_cfr(src, dst, size, 64u << 10));
    RUN("pread+pwrite bs=4MB",   bench_rw(src, dst, size, 4u << 20));
    RUN("pread+pwrite bs=64KB",  bench_rw(src, dst, size, 64u << 10));
    RUN("sendfile",              bench_sendfile(src, dst, size));

#undef RUN
    printf("\n注：每项前都 drop_caches，所以是冷盘口径。\n");
    return 0;
}
