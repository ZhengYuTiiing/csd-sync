/*
 * test_sync_csd.cpp —— 文件同步下沉至 CEMU CSD 的完整功能与三层挑战验证
 *
 * 本测试验证整个下沉体系的关键路径:
 *   [挑战一] Extent 扁平化描述符下发与 indirect buffer 打包 (read_mode = 1)
 *   [挑战二] 32B 指纹写时固化与极速浅读 (OOB gather 与 collect_oob 算子)
 *   [挑战三] rsync 窗口化流式比对 (rsync_match 算子)
 *   [挑战三] rsync 设备端流式重构 (rsync_patch 算子)
 *   [挑战三] dsync 变长分块两级匹配与防碰撞 (dsync_match 算子: CRC32C 弱筛 -> 256b 强验)
 */

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <vector>
#include <iostream>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>

#include "util.h"
#include "kernel/cemu_def.h"
#include "kernel/rsync_csd_abi.h"
#include "kernel/dsync_abi.h"
#include "../../hw/femu/backend/csd_fingerprint.h"

typedef long long (*csf_func_t)(struct cemu_args *args);

#include <sys/ioctl.h>
#include <linux/fs.h>
#include <linux/fiemap.h>

static std::string get_fiemap_test_path()
{
    std::vector<std::string> dirs;
    if (access("/mnt/nvme0", W_OK) == 0) dirs.push_back("/mnt/nvme0");
    if (access("/var/tmp", W_OK) == 0) dirs.push_back("/var/tmp");
    if (access("/tmp", W_OK) == 0) dirs.push_back("/tmp");
    dirs.push_back(".");

    for (const auto &d : dirs) {
        std::string p = d + "/cemu_fiemap_probe.tmp";
        int fd = open(p.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        if (fd < 0) continue;
        uint8_t zero[4096];
        memset(zero, 0, sizeof(zero));
        write(fd, zero, sizeof(zero));
        fsync(fd);
        close(fd);

        int rfd = open(p.c_str(), O_RDONLY);
        if (rfd >= 0) {
            struct fiemap tmp;
            memset(&tmp, 0, sizeof(tmp));
            tmp.fm_length = 4096;
            tmp.fm_flags = FIEMAP_FLAG_SYNC;
            int ret = ioctl(rfd, FS_IOC_FIEMAP, &tmp);
            close(rfd);
            unlink(p.c_str());
            if (ret == 0) {
                return d + "/cemu_challenge1_extent.dat";
            }
        } else {
            unlink(p.c_str());
        }
    }
    return "/tmp/cemu_challenge1_extent.dat";
}
static void test_challenge1_packing()
{
    std::cout << "[Test 1/5] 挑战一: Extent 描述符 (get_file_mappings / FS_IOC_FIEMAP) 与 read_mode schema 组装..." << std::endl;

    std::string tmp_path = get_fiemap_test_path();
    const size_t file_bytes = 64 * 1024; // 64KB = 128 LBAs
    int fd = open(tmp_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
    assert(fd >= 0);
    std::vector<uint8_t> dummy(file_bytes, 0xAB);
    ssize_t written = write(fd, dummy.data(), file_bytes);
    assert(written == (ssize_t)file_bytes);
    fsync(fd);
    close(fd);

    uint64_t filesize = file_bytes;
    int parallel_chunks = 2;
    std::vector<int> nlb_per_chunk(parallel_chunks, 0);
    std::vector<int> nr_cf_per_chunk(parallel_chunks, 0);
    NvmeCopyFormat2 *cfs = nullptr;

    // 真实调用内核接口 get_file_mappings (内部使用 ioctl(FS_IOC_FIEMAP))
    int nr_cf = get_file_mappings(tmp_path.c_str(), &filesize, &cfs, parallel_chunks,
                                  nlb_per_chunk.data(), nr_cf_per_chunk.data());
    assert(nr_cf > 0);
    assert(cfs != nullptr);
    std::cout << "  -> 真实通过 Linux FS_IOC_FIEMAP 成功解析出 " << nr_cf << " 个连续 Extent 区段" << std::endl;
    for (int i = 0; i < std::min(nr_cf, 3); i++) {
        std::cout << "     Extent[" << i << "]: slba=" << cfs[i].slba
                  << ", nlb=" << cfs[i].nlb << ", snsid=" << cfs[i].snsid << std::endl;
    }

    const char *extra_str = "NEW_FINGERPRINTS_PAYLOAD_TEST";
    size_t extra_sz = strlen(extra_str) + 1;
    size_t req_sz = calc_indirect_task_buffer_size(parallel_chunks, 0, nr_cf, 0, extra_sz);
    std::vector<uint8_t> buf(req_sz, 0);

    size_t packed_sz = pack_indirect_task_buffer(buf.data(),
                                                parallel_chunks,
                                                /*destination=*/0,
                                                /*read_mode=*/CSD_READ_MODE_OOB,
                                                nr_cf, nr_cf_per_chunk.data(), cfs,
                                                0, NULL, NULL,
                                                extra_str, extra_sz);
    assert(packed_sz == req_sz);

    // 校验解析逻辑与 compute.c:program_execute 一致
    int *raw_task = (int *)buf.data();
    assert(raw_task[0] == parallel_chunks); // nr_concurrent_chunks

    int raw_dest = raw_task[1];
    int parsed_read_mode   = (raw_dest >> 16) & 0xffff;
    int parsed_destination = raw_dest & 0xffff;
    assert(parsed_read_mode == 1); // CSD_READ_MODE_OOB
    assert(parsed_destination == 0); // FDM output

    assert(raw_task[2] == nr_cf); // nr_total_input_cf2
    assert(raw_task[3] == 0); // nr_total_output_cf2

    // 校验 extra payload
    uint8_t *payload_ptr = buf.data() + (packed_sz - extra_sz);
    assert(strcmp((char *)payload_ptr, extra_str) == 0);

    free(cfs);
    unlink(tmp_path.c_str());

    std::cout << "  -> Extent 物理区段打包与 read_mode=1 协议组装验证成功！" << std::endl;
}

static void test_challenge2_oob_solidification_and_shallow_read()
{
    std::cout << "[Test 2/5] 挑战二: 写时 32B 指纹固化与极速浅读 (collect_oob)..." << std::endl;

    // 准备 8 个 4KB 页数据
    const int nr_pages = 8;
    std::vector<uint8_t> pages(nr_pages * CSD_PAGE_SIZE);
    std::vector<uint8_t> oob_area(nr_pages * CSD_FP_LEN);

    for (int p = 0; p < nr_pages; p++) {
        uint8_t *page = pages.data() + p * CSD_PAGE_SIZE;
        for (size_t b = 0; b < CSD_PAGE_SIZE; b++) {
            page[b] = (uint8_t)((p * 37 + b) ^ 0x5a);
        }
        // 模拟 FTL 写时 hook (backend_fingerprint_range -> csd_fp256)
        csd_fp256(page, CSD_PAGE_SIZE, oob_area.data() + p * CSD_FP_LEN);
    }

    // 验证浅读: 拷贝 OOB 区只需要 8 * 32 = 256 字节，而不是 8 * 4096 = 32KB
    assert(oob_area.size() == 256);
    std::cout << "  -> 8 页全数据大小: " << pages.size() << " B，浅读 OOB 大小: " << oob_area.size() << " B (仅 1/128 搬运量！)" << std::endl;

    // 加载 collect_oob.so 算子
    void *h = dlopen("./build/collect_oob.so", RTLD_NOW);
    if (!h) {
        std::cerr << "dlopen collect_oob.so failed: " << dlerror() << std::endl;
        exit(1);
    }
    csf_func_t collect_fn = (csf_func_t)dlsym(h, "collect_oob");
    assert(collect_fn != NULL);

    std::vector<uint8_t> out_scratch(oob_area.size(), 0);
    void *mr_addrs[2] = { out_scratch.data(), oob_area.data() };
    long long mr_lens[2] = { (long long)out_scratch.size(), (long long)oob_area.size() };

    struct cemu_args args;
    memset(&args, 0, sizeof(args));
    args.numr = 2;
    args.mr_addr = mr_addrs;
    args.mr_len = mr_lens;
    args.cparam1 = nr_pages;

    long long ret = collect_fn(&args);
    assert(ret >= 0);
    assert(memcmp(out_scratch.data(), oob_area.data(), oob_area.size()) == 0);
    dlclose(h);

    std::cout << "  -> collect_oob CSF 执行成功，浅读指纹逐字节无损传递！" << std::endl;
}

static void test_challenge3_rsync_match()
{
    std::cout << "[Test 3/5] 挑战三: rsync 窗口化流式指纹比对 (rsync_match)..." << std::endl;

    const int nr_blks = 16;
    // 构造老文件 16 块的指纹
    std::vector<uint8_t> old_fps(nr_blks * RSYNC_CSD_FP_LEN);
    for (int i = 0; i < nr_blks; i++) {
        std::vector<uint8_t> dummy_page(CSD_PAGE_SIZE, (uint8_t)(i + 1));
        csd_fp256(dummy_page.data(), CSD_PAGE_SIZE, old_fps.data() + i * RSYNC_CSD_FP_LEN);
    }

    // 构造新文件指纹表:
    // 偶数块与老文件相同 (8 块命中)，奇数块为新内容 (8 块变化)
    std::vector<struct rsync_fp_entry> new_entries(nr_blks);
    for (int i = 0; i < nr_blks; i++) {
        new_entries[i].blk_idx = i;
        new_entries[i].flags = 0;
        if (i % 2 == 0) {
            // 命中老文件相同块
            memcpy(new_entries[i].fp, old_fps.data() + i * RSYNC_CSD_FP_LEN, RSYNC_CSD_FP_LEN);
        } else {
            // 变化块
            std::vector<uint8_t> dummy_page(CSD_PAGE_SIZE, (uint8_t)(i + 100));
            csd_fp256(dummy_page.data(), CSD_PAGE_SIZE, new_entries[i].fp);
        }
    }

    // 组装新文件指纹表 payload
    size_t table_sz = sizeof(struct rsync_fp_table_hdr) + new_entries.size() * sizeof(struct rsync_fp_entry);
    std::vector<uint8_t> data_buffer(table_sz);
    struct rsync_fp_table_hdr *hdr = (struct rsync_fp_table_hdr *)data_buffer.data();
    hdr->magic = RSYNC_FP_TABLE_MAGIC;
    hdr->nr_entries = (uint32_t)new_entries.size();
    hdr->entry_bytes = sizeof(struct rsync_fp_entry);
    hdr->file_size = nr_blks * RSYNC_CSD_BLOCK_SIZE;
    memcpy(data_buffer.data() + sizeof(*hdr), new_entries.data(), new_entries.size() * sizeof(struct rsync_fp_entry));

    // 加载 rsync_match.so
    void *h = dlopen("./build/rsync_match.so", RTLD_NOW);
    if (!h) {
        std::cerr << "dlopen rsync_match.so failed: " << dlerror() << std::endl;
        exit(1);
    }
    csf_func_t match_fn = (csf_func_t)dlsym(h, "rsync_match");
    assert(match_fn != NULL);

    std::vector<uint8_t> out_match(4096, 0);
    void *mr_addrs[2] = { out_match.data(), old_fps.data() };
    long long mr_lens[2] = { (long long)out_match.size(), (long long)old_fps.size() };

    struct cemu_args args;
    memset(&args, 0, sizeof(args));
    args.numr = 2;
    args.mr_addr = mr_addrs;
    args.mr_len = mr_lens;
    args.cparam1 = 0; // base block
    args.cparam2 = nr_blks;
    args.data_buffer = data_buffer.data();
    args.buffer_len = (long long)data_buffer.size();

    long long ret = match_fn(&args);
    assert(ret > 0);

    struct rsync_match_hdr *mhdr = (struct rsync_match_hdr *)out_match.data();
    assert(mhdr->magic == RSYNC_MATCH_MAGIC);
    assert(mhdr->nr_matched == 8);
    assert(mhdr->nr_changed == 8);

    struct rsync_match_token *tokens = (struct rsync_match_token *)(out_match.data() + sizeof(*mhdr));
    for (uint32_t i = 0; i < mhdr->nr_matched; i++) {
        assert(tokens[i].old_blk_idx == tokens[i].new_blk_idx);
        assert(tokens[i].old_blk_idx % 2 == 0);
    }
    dlclose(h);

    std::cout << "  -> rsync_match 准确比对: 匹配 " << mhdr->nr_matched << " 块，变化 " << mhdr->nr_changed << " 块，映射严格匹配！" << std::endl;
}

static void test_challenge3_rsync_patch()
{
    std::cout << "[Test 4/5] 挑战三: rsync 设备端流式 Patch 重构 (rsync_patch)..." << std::endl;

    const int nr_blks = 4;
    // 老文件 4 块
    std::vector<uint8_t> old_data(nr_blks * RSYNC_CSD_BLOCK_SIZE);
    for (size_t i = 0; i < old_data.size(); i++) {
        old_data[i] = (uint8_t)(i & 0xff);
    }

    // 目标新文件: blk 0, 2 复用老文件; blk 1, 3 插入新 literal
    std::vector<uint8_t> lit_block1(RSYNC_CSD_BLOCK_SIZE, 0xaa);
    std::vector<uint8_t> lit_block3(RSYNC_CSD_BLOCK_SIZE, 0xbb);

    std::vector<struct rsync_delta_cmd> cmds(4);
    cmds[0].cmd_type = RSYNC_CMD_COPY_OLD;
    cmds[0].blk_idx = 0;
    cmds[0].old_blk_idx = 0;
    cmds[0].data_offset = 0;

    cmds[1].cmd_type = RSYNC_CMD_INSERT_LIT;
    cmds[1].blk_idx = 1;
    cmds[1].old_blk_idx = 0;
    cmds[1].data_offset = 0; // offset in lit pool

    cmds[2].cmd_type = RSYNC_CMD_COPY_OLD;
    cmds[2].blk_idx = 2;
    cmds[2].old_blk_idx = 2;
    cmds[2].data_offset = 0;

    cmds[3].cmd_type = RSYNC_CMD_INSERT_LIT;
    cmds[3].blk_idx = 3;
    cmds[3].old_blk_idx = 0;
    cmds[3].data_offset = RSYNC_CSD_BLOCK_SIZE; // offset in lit pool

    // 组装 delta_buffer
    size_t pool_offset = sizeof(struct rsync_delta_hdr) + cmds.size() * sizeof(struct rsync_delta_cmd);
    size_t total_delta_sz = pool_offset + 2 * RSYNC_CSD_BLOCK_SIZE;
    std::vector<uint8_t> delta_buffer(total_delta_sz);

    struct rsync_delta_hdr *dhdr = (struct rsync_delta_hdr *)delta_buffer.data();
    dhdr->magic = RSYNC_DELTA_MAGIC;
    dhdr->nr_cmds = 4;
    dhdr->blk_size = RSYNC_CSD_BLOCK_SIZE;
    dhdr->data_pool_offset = (uint32_t)pool_offset;

    memcpy(delta_buffer.data() + sizeof(*dhdr), cmds.data(), cmds.size() * sizeof(struct rsync_delta_cmd));
    memcpy(delta_buffer.data() + pool_offset, lit_block1.data(), RSYNC_CSD_BLOCK_SIZE);
    memcpy(delta_buffer.data() + pool_offset + RSYNC_CSD_BLOCK_SIZE, lit_block3.data(), RSYNC_CSD_BLOCK_SIZE);

    // 加载 rsync_patch.so
    void *h = dlopen("./build/rsync_patch.so", RTLD_NOW);
    if (!h) {
        std::cerr << "dlopen rsync_patch.so failed: " << dlerror() << std::endl;
        exit(1);
    }
    csf_func_t patch_fn = (csf_func_t)dlsym(h, "rsync_patch");
    assert(patch_fn != NULL);

    std::vector<uint8_t> reconstructed(nr_blks * RSYNC_CSD_BLOCK_SIZE, 0);
    void *mr_addrs[2] = { reconstructed.data(), old_data.data() };
    long long mr_lens[2] = { (long long)reconstructed.size(), (long long)old_data.size() };

    struct cemu_args args;
    memset(&args, 0, sizeof(args));
    args.numr = 2;
    args.mr_addr = mr_addrs;
    args.mr_len = mr_lens;
    args.data_buffer = delta_buffer.data();
    args.buffer_len = (long long)delta_buffer.size();

    long long ret = patch_fn(&args);
    // 4 块 * (4096 / 512) = 32 个 512B 扇区
    assert(ret == 32);

    // 校验重构内容
    assert(memcmp(reconstructed.data() + 0 * RSYNC_CSD_BLOCK_SIZE, old_data.data() + 0 * RSYNC_CSD_BLOCK_SIZE, RSYNC_CSD_BLOCK_SIZE) == 0);
    assert(memcmp(reconstructed.data() + 1 * RSYNC_CSD_BLOCK_SIZE, lit_block1.data(), RSYNC_CSD_BLOCK_SIZE) == 0);
    assert(memcmp(reconstructed.data() + 2 * RSYNC_CSD_BLOCK_SIZE, old_data.data() + 2 * RSYNC_CSD_BLOCK_SIZE, RSYNC_CSD_BLOCK_SIZE) == 0);
    assert(memcmp(reconstructed.data() + 3 * RSYNC_CSD_BLOCK_SIZE, lit_block3.data(), RSYNC_CSD_BLOCK_SIZE) == 0);
    dlclose(h);

    std::cout << "  -> rsync_patch 成功重构 4 块数据，扇区数 32，逐字节与预期一致！" << std::endl;
}

static void test_challenge3_dsync_match()
{
    std::cout << "[Test 5/5] 挑战三: dsync FastCDC 两级匹配与防碰撞 (dsync_match)..." << std::endl;

    // 构造 4 个老 chunk
    std::vector<struct dsync_chunk_desc> old_chunks(4);
    for (int i = 0; i < 4; i++) {
        old_chunks[i].offset    = (uint64_t)i * 8192;
        old_chunks[i].length    = 8192;
        old_chunks[i].weak_hash = 0x12340000 + i;
        memset(old_chunks[i].strong_hash, (uint8_t)(i + 1), DSYNC_FP_LEN);
    }

    // 构造新 chunk 表:
    // chunk 0: 完全匹配
    // chunk 1: 弱哈希碰撞测试! (弱哈希相同，但强哈希不同 -> 必须判定不匹配)
    // chunk 2: 完全不同
    // chunk 3: 完全匹配 (偏移不同)
    std::vector<struct dsync_chunk_desc> new_chunks(3);
    new_chunks[0] = old_chunks[0]; // match

    new_chunks[1].offset    = 10000;
    new_chunks[1].length    = 8192;
    new_chunks[1].weak_hash = old_chunks[1].weak_hash; // 同弱哈希
    memset(new_chunks[1].strong_hash, 0xff, DSYNC_FP_LEN); // 异强哈希 (碰撞用例)

    new_chunks[2].offset    = 20000;
    new_chunks[2].length    = 8192;
    new_chunks[2].weak_hash = old_chunks[3].weak_hash;
    memcpy(new_chunks[2].strong_hash, old_chunks[3].strong_hash, DSYNC_FP_LEN); // match old_chunks[3]

    // 组装新 chunk 表
    size_t new_tbl_sz = sizeof(struct dsync_table_hdr) + new_chunks.size() * sizeof(struct dsync_chunk_desc);
    std::vector<uint8_t> new_tbl_buf(new_tbl_sz);
    struct dsync_table_hdr *nhdr = (struct dsync_table_hdr *)new_tbl_buf.data();
    nhdr->magic = DSYNC_TABLE_MAGIC;
    nhdr->nr_chunks = (uint32_t)new_chunks.size();
    nhdr->entry_bytes = sizeof(struct dsync_chunk_desc);
    nhdr->total_data_len = 3 * 8192;
    memcpy(new_tbl_buf.data() + sizeof(*nhdr), new_chunks.data(), new_chunks.size() * sizeof(struct dsync_chunk_desc));

    // 加载 dsync_match.so
    void *h = dlopen("./build/dsync_match.so", RTLD_NOW);
    if (!h) {
        std::cerr << "dlopen dsync_match.so failed: " << dlerror() << std::endl;
        exit(1);
    }
    csf_func_t dmatch_fn = (csf_func_t)dlsym(h, "dsync_match");
    assert(dmatch_fn != NULL);

    std::vector<uint8_t> out_match(4096, 0);
    void *mr_addrs[2] = { out_match.data(), old_chunks.data() };
    long long mr_lens[2] = { (long long)out_match.size(), (long long)(old_chunks.size() * sizeof(struct dsync_chunk_desc)) };

    struct cemu_args args;
    memset(&args, 0, sizeof(args));
    args.numr = 2;
    args.mr_addr = mr_addrs;
    args.mr_len = mr_lens;
    args.cparam1 = (long long)old_chunks.size();
    args.data_buffer = new_tbl_buf.data();
    args.buffer_len = (long long)new_tbl_buf.size();

    long long ret = dmatch_fn(&args);
    assert(ret > 0);

    struct dsync_match_hdr *mhdr = (struct dsync_match_hdr *)out_match.data();
    assert(mhdr->magic == DSYNC_MATCH_MAGIC);
    assert(mhdr->nr_matched == 2);   // 只有 chunk 0 和 chunk 3 匹配
    assert(mhdr->nr_unmatched == 2); // chunk 1 (碰撞过滤) + chunk 2 未匹配
    assert(mhdr->matched_bytes == 16384);

    struct dsync_match_token *tokens = (struct dsync_match_token *)(out_match.data() + sizeof(*mhdr));
    assert(tokens[0].old_offset == 0 && tokens[0].new_offset == 0);
    assert(tokens[1].old_offset == 24576 && tokens[1].new_offset == 20000);
    dlclose(h);

    std::cout << "  -> dsync_match 两级比对成功: 命中 " << mhdr->nr_matched
              << " 块 (共 " << mhdr->matched_bytes << " 字节)，成功拦截弱哈希伪碰撞！" << std::endl;
}

int main()
{
    std::cout << "==========================================================" << std::endl;
    std::cout << "       CEMU CSD 文件同步下沉 (rsync / dsync) 验证测试     " << std::endl;
    std::cout << "==========================================================" << std::endl;

    test_challenge1_packing();
    test_challenge2_oob_solidification_and_shallow_read();
    test_challenge3_rsync_match();
    test_challenge3_rsync_patch();
    test_challenge3_dsync_match();

    std::cout << "==========================================================" << std::endl;
    std::cout << "       所有 5 项测试全部通过 (ALL TESTS PASSED)！          " << std::endl;
    std::cout << "==========================================================" << std::endl;
    return 0;
}
