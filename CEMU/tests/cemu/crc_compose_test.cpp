/*
 * crc_compose_test.cpp —— 验证"由定长槽位拼出任意边界校验和"
 *
 * 流程：
 *   1. 主机读文件，按 4KB 页生成 8 个扇区 CRC32C（每页 32B），构成定长网格
 *   2. 随机挑选若干个 512B 对齐的区间 (off, len)
 *   3. 把网格和查询表交给 compose_crc 算子（与设备侧同一份代码）
 *   4. 算子只用网格里的定长槽位拼出每个区间的 CRC，主机再独立算一遍比对
 *
 * 后续设备侧 OOB 换成 CRC 网格后，第 1 步改由 read_mode=1 浅读 OOB 即可，
 * 算子与查询协议完全不变。
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <dlfcn.h>

#include <fcntl.h>
#include <unistd.h>

#include "kernel/cemu_def.h"
#include "kernel/rsync_csd_abi.h"
#include "../../hw/femu/backend/csd_fp_crc.h"

#define BLK 4096u

typedef long long (*csf_func_t)(struct cemu_args *);

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

int main(int argc, char **argv)
{
    const std::string path = (argc > 1) ? argv[1] : "/mnt/nvme0/dataset/10M-new/10MB";

    printf("=== 可组合 CRC：任意 512B 对齐边界的校验和拼接 ===\n");
    printf("文件: %s\n", path.c_str());

    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    off_t sz = lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);
    if (sz <= 0 || (sz % BLK) != 0) {
        fprintf(stderr, "文件大小必须是 4KB 的整数倍\n");
        return 1;
    }
    const size_t size = (size_t)sz;
    std::vector<uint8_t> data(size);
    if (!read_full(fd, data.data(), size)) {
        perror("read");
        return 1;
    }
    close(fd);

    const uint32_t nr_pages = (uint32_t)(size / BLK);

    /* 1. 生成定长网格（每页 32B） */
    std::vector<uint8_t> grid((size_t)nr_pages * CSD_CRC_SLOT_LEN, 0);
    for (uint32_t p = 0; p < nr_pages; p++) {
        csd_crc_slot_build(data.data() + (size_t)p * BLK,
                           grid.data() + (size_t)p * CSD_CRC_SLOT_LEN);
    }
    printf("网格: %u 页 × %u B = %zu B (相对原始数据 %.3f%%)\n",
           nr_pages, CSD_CRC_SLOT_LEN, grid.size(),
           (double)grid.size() * 100.0 / (double)size);

    /* 2. 随机生成 512B 对齐的查询区间 */
    const uint32_t nr_q = 32;
    std::vector<struct crc_query> qs(nr_q);
    srand(20261017);
    for (uint32_t i = 0; i < nr_q; i++) {
        const uint64_t max_sec = size / 512;
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
    auto *hdr = (struct crc_query_hdr *)req.data();
    hdr->magic = RSYNC_CRC_QUERY_MAGIC;
    hdr->nr_queries = nr_q;
    hdr->flags = 0;
    memcpy(req.data() + sizeof(*hdr), qs.data(), (size_t)nr_q * sizeof(struct crc_query));

    std::vector<uint32_t> out(nr_q, 0);

    /* 3. 调用设备侧同一份算子 */
    void *h = dlopen("./build/compose_crc.so", RTLD_NOW);
    if (!h) {
        fprintf(stderr, "dlopen compose_crc.so 失败: %s\n", dlerror());
        return 1;
    }
    csf_func_t fn = (csf_func_t)dlsym(h, "compose_crc");
    if (!fn) {
        fprintf(stderr, "dlsym 失败\n");
        return 1;
    }

    void *mr_addr[2] = { out.data(), grid.data() };
    long long mr_len[2] = { (long long)(out.size() * sizeof(uint32_t)),
                            (long long)grid.size() };

    struct cemu_args args;
    memset(&args, 0, sizeof(args));
    args.numr = 2;
    args.mr_addr = mr_addr;
    args.mr_len = mr_len;
    args.data_buffer = req.data();
    args.buffer_len = (long long)req.size();

    long long ret = fn(&args);
    if (ret < 0) {
        fprintf(stderr, "compose_crc 返回错误 %lld\n", ret);
        return 1;
    }
    dlclose(h);

    /* 4. 主机独立计算并比对 */
    int bad = 0;
    for (uint32_t i = 0; i < nr_q; i++) {
        const uint32_t want = csd_crc32c(0, data.data() + qs[i].off, qs[i].len);
        if (want != out[i]) {
            if (bad < 5) {
                printf("  MISMATCH q[%u] off=%lu len=%lu: 拼出 0x%08x, 实算 0x%08x\n",
                       i, (unsigned long)qs[i].off, (unsigned long)qs[i].len, out[i], want);
            }
            bad++;
        }
    }

    printf("查询数: %u (随机 512B 对齐区间, 长度 512B~16KB)\n", nr_q);
    for (uint32_t i = 0; i < 3; i++) {
        printf("  q[%u] off=%-9lu len=%-6lu crc=0x%08x\n", i,
               (unsigned long)qs[i].off, (unsigned long)qs[i].len, out[i]);
    }
    printf("%s\n", bad == 0 ? "[PASS] 所有区间校验和均由定长槽位正确拼出"
                            : "[FAIL] 存在拼接错误");

    return bad == 0 ? 0 : 1;
}
