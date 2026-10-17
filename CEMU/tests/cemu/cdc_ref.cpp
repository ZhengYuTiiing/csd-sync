/*
 * cdc_ref.cpp —— 整文件一次扫描的 CDC 块数参照（仲裁用）
 *
 * 与 kernel/cdc_desc.c 完全同一套循环：同 gear 表、同 CHUNK_MIN_SIZE、
 * 同 512 对齐采样、同 64KB 上界。差别只在"没有窗口"——从文件头扫到文件尾。
 * 用来仲裁 dsync2（32MiB 硬切窗）与 dsync3（carry 接力）哪个贴合真值。
 */

#include <cstdio>
#include <cstdint>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "kernel/chunk_abi.h"
#include "kernel/chunk_gear_v2.h"

#define CDC_ALIGN     512ULL
#define CDC_MAX_CHUNK 65536ULL

static uint64_t cdc_count(const uint8_t *buf, uint64_t n)
{
    uint64_t pos = 0, nr = 0;

    while (pos < n) {
        uint64_t limit = pos + CDC_MAX_CHUNK;
        uint64_t cut, fp = 0;

        if (limit > n) {
            limit = n;
        }
        cut = limit;
        for (uint64_t i = pos; i < limit; i++) {
            uint64_t end;

            fp = (fp << 1) + chunk_gear_table[buf[i]];
            end = i + 1;
            if ((end - pos) < CHUNK_MIN_SIZE) {
                continue;
            }
            if (end % CDC_ALIGN) {
                continue;
            }
            if ((fp & CHUNK_GEAR_MASK) == 0) {
                cut = end;
                break;
            }
        }
        nr++;
        pos = cut;
    }
    return nr;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s <file> [file...]\n", argv[0]);
        return 1;
    }

    for (int a = 1; a < argc; a++) {
        int fd = open(argv[a], O_RDONLY);
        if (fd < 0) {
            perror(argv[a]);
            return 1;
        }
        struct stat st {};
        if (fstat(fd, &st) < 0) {
            perror("fstat");
            return 1;
        }
        uint64_t n = (uint64_t)st.st_size;
        std::vector<uint8_t> buf(n);
        uint64_t got = 0;
        while (got < n) {
            ssize_t r = pread(fd, buf.data() + got, (size_t)(n - got), (off_t)got);
            if (r <= 0) {
                perror("pread");
                return 1;
            }
            got += (uint64_t)r;
        }
        printf("%s: %llu B -> %llu 块 (整文件一次扫描)\n",
               argv[a], (unsigned long long)n,
               (unsigned long long)cdc_count(buf.data(), n));
        close(fd);
    }
    return 0;
}
