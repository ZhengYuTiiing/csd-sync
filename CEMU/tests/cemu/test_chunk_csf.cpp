/*
 * test_chunk_csf.cpp —— CDC 算子族 (chunk_csf + merge_csf) 的 L0 正确性验证
 *
 * ============================ 什么是 L0 harness ============================
 * 算子验证分三层，L0 是最底层、最便宜、也最该天天跑的一层：
 *
 *   L0（本文件，纯主机，不需要 CEMU/QEMU 在跑）
 *       把算子编译出来的真实 .so 用 dlopen 加载进来，在主机上直接调用，
 *       和"主机上那份 trusted 参考实现"逐项对拍。
 *       价值：全部逻辑与 ABI 都能在秒级验证完，改一行就能立刻知道对错，
 *             不用起虚拟机、不用碰硬件、不用等 IO。
 *
 *   L1（在 CEMU 里跑一遍）—— 验证"算子能被 CSD 正确加载/调度/访问内存"
 *       用 MRS + program_execute 真的把数据放进设备内存跑一次，读回结果。
 *       验证的是通路（dlopen、dlsym、mr_addr/mr_len、chunk_nlb、返回值），
 *       逻辑正确性已经在 L0 保证了，所以 L1 只用少量用例即可。
 *
 *   L2（和主机端到端对拍）—— 验证"整条同步链路结果一致"
 *       同一份文件跑主机 parallel_cdc 与 CSD 算子链，比对切点/弱哈希/最终 delta。
 *       本文件 --hash 打印的指纹就是给 L2 当锚点用的。
 *
 * 本 harness 覆盖的两条不变量（这是整个 CDC 下沉的正确性基石）：
 *   A. chunk_csf 的切点 + 每块 CRC32 == 主机 ParaSyncChunker 的产物
 *   B. merge_csf 的合并块 + 合并 CRC32 == 主机 ParaSyncWorker::detector 的产物
 *      其中参考实现的 CRC 是**直接用 zlib 对合并区间重算**得到的，
 *      而算子走的是 crc32_comb 增量拼接 —— 两者相等才说明合并语义正确。
 * ==========================================================================
 *
 * 用法:
 *   make -C kernel && make
 *   ./build/test_chunk_csf [--chunk-so ./build/chunk_csf.so]
 *                          [--merge-so ./build/merge_csf.so]
 *                          [--host-header /home/zyt/parasync/src/fastcdc.h]
 *                          [--segments 4] [--hash]
 */

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
#include <dlfcn.h>
#include <zlib.h>

#include "kernel/cemu_def.h"
#include "kernel/chunk_abi.h"

static_assert(sizeof(struct chunk_ent) == 24, "chunk_ent 必须是 24 字节");
static_assert(sizeof(struct chunk_table_hdr) == 48, "chunk_table_hdr 必须是 48 字节");

typedef long long (*csf_fn_t)(struct cemu_args *);
typedef const uint64_t *(*gear_fn_t)(void);
typedef uint32_t (*crc_fn_t)(const void *, size_t, uint32_t);
typedef uint32_t (*comb_fn_t)(uint32_t, uint32_t, size_t);

static int g_fail = 0;
static int g_case = 0;

static void checkf(bool ok, const char *fmt, ...)
{
    if (ok)
        return;
    g_fail++;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "  [FAIL] case %d: ", g_case);
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static double ms_since(const std::chrono::steady_clock::time_point &t0)
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0).count();
}

/* zlib 的 crc32 就是标准 CRC-32，作为完全独立的参考实现 */
static inline uint32_t zcrc(const void *p, size_t n, uint32_t prev)
{
    return (uint32_t)crc32((uLong)prev, (const Bytef *)p, (uInt)n);
}

/* ---------------- 主机参考实现（不依赖算子里的任何代码） ---------------- */

/* ParaSyncChunker::run(): 逐段 gear 切 + 每块 CRC32 */
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
        chunk_ent e = {pre, l, zcrc(buf + pre, l, 0),
                       CHUNK_ENT_CRC_VALID | CHUNK_ENT_TAIL_OPEN};
        out.push_back(e);
    }
}

/* ParaSyncWorker::detector(): 跨段全局累加，>= MinSize 就 flush */
static std::vector<chunk_ent> ref_detector(const std::vector<chunk_ent> &raw,
                                           const uint8_t *buf)
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

/* ---------------- 表缓冲 ---------------- */

struct TBuf {
    uint8_t *p = nullptr;
    size_t cap = 0;  /* 分配容量（4096 对齐） */
    size_t used = 0; /* 表实际占用字节数 */
    void release()
    {
        if (p)
            free(p);
        p = nullptr;
        cap = used = 0;
    }
};

static bool tb_alloc(TBuf &t, size_t want)
{
    t.release();
    size_t a = (want + 4095) / 4096 * 4096;
    t.p = (uint8_t *)aligned_alloc(4096, a);
    if (!t.p)
        return false;
    memset(t.p, 0, a);
    t.cap = a;
    return true;
}

/* 解析并做通用结构校验；cov_begin/cov_end 用于校验覆盖范围 */
static bool tb_parse(const TBuf &t, uint64_t expect_magic, std::vector<chunk_ent> &out,
                     uint64_t *cov_begin, uint64_t *cov_end, const char *who)
{
    if (t.used < sizeof(struct chunk_table_hdr)) {
        checkf(false, "%s: 表太小 (%zu)", who, t.used);
        return false;
    }
    struct chunk_table_hdr h;
    memcpy(&h, t.p, sizeof(h));
    checkf(h.magic == expect_magic, "%s: magic 0x%lx 期望 0x%lx", who, h.magic, expect_magic);
    checkf(h.entry_bytes == h.nr_chunks * sizeof(struct chunk_ent),
           "%s: entry_bytes %lu 与 nr_chunks %lu 不符", who, h.entry_bytes, h.nr_chunks);
    checkf(sizeof(h) + h.entry_bytes <= t.cap, "%s: 表越界写 %lu > %zu",
           who, sizeof(h) + h.entry_bytes, t.cap);

    uint64_t cov = h.base_offset;
    const struct chunk_ent *e = (const struct chunk_ent *)(t.p + sizeof(h));
    for (uint64_t i = 0; i < h.nr_chunks; i++) {
        if (e[i].length == 0)
            checkf(false, "%s: 第 %lu 条 length 为 0", who, i);
        if (e[i].offset != cov)
            checkf(false, "%s: 第 %lu 条 offset %lu != %lu", who, i, e[i].offset, cov);
        if (!(e[i].flags & CHUNK_ENT_CRC_VALID))
            checkf(false, "%s: 第 %lu 条没有 CRC_VALID", who, i);
        out.push_back(e[i]);
        cov += e[i].length;
    }
    if (cov_begin)
        *cov_begin = h.base_offset;
    if (cov_end)
        *cov_end = cov;
    checkf(cov - h.base_offset == h.data_len, "%s: 覆盖 %lu != data_len %lu",
           who, cov - h.base_offset, h.data_len);
    return true;
}

/* ---------------- 算子调用 ---------------- */

static bool run_chunk_csf(csf_fn_t fn, const uint8_t *data, uint64_t base, uint64_t n,
                          TBuf &out, double *ms)
{
    size_t cap = 64 * 1024;
    for (int attempt = 0; attempt < 12; attempt++) {
        TBuf in;
        size_t in_alloc = ((n ? n : 1) + 4095) / 4096 * 4096;
        in.p = (uint8_t *)aligned_alloc(4096, in_alloc);
        if (!in.p || !tb_alloc(out, cap)) {
            if (in.p)
                free(in.p);
            return false;
        }
        in.cap = in_alloc;
        if (n)
            memcpy(in.p, data, n);

        void *mr_addr[2] = {out.p, in.p};
        long long mr_len[2] = {(long long)out.cap, (long long)n};
        struct cemu_args args;
        memset(&args, 0, sizeof(args));
        args.numr = 2;
        args.mr_addr = mr_addr;
        args.mr_len = mr_len;
        args.cparam1 = (long long)n;
        args.cparam2 = (long long)base;

        auto t0 = std::chrono::steady_clock::now();
        long long rc = fn(&args);
        if (ms)
            *ms += ms_since(t0);
        free(in.p);

        if (rc == -2) { /* 输出不够：主机按协议放大重试 */
            cap *= 4;
            continue;
        }
        checkf(rc >= 0, "chunk_csf 返回 %lld (base=%lu n=%lu)", rc, base, n);
        if (rc < 0)
            return false;
        out.used = sizeof(struct chunk_table_hdr) + (size_t)rc * sizeof(struct chunk_ent);

        /* 表头/段内覆盖性 */
        std::vector<chunk_ent> tmp;
        uint64_t b = 0, e = 0;
        if (!tb_parse(out, CHUNK_TABLE_MAGIC, tmp, &b, &e, "chunk_csf"))
            return false;
        checkf(b == base, "chunk_csf: base_offset %lu != %lu", b, base);
        checkf(e - b == n, "chunk_csf: 覆盖 %lu != n %lu", e - b, n);
        if (n)
            checkf(((struct chunk_table_hdr *)out.p)->flags & CHUNK_FLAG_TAIL_OPEN,
                   "chunk_csf: 非空段未标 TAIL_OPEN");
        return true;
    }
    checkf(false, "chunk_csf: ENOSPC 重试 12 次");
    return false;
}

static bool run_merge_csf(csf_fn_t fn, const std::vector<const TBuf *> &ins,
                          TBuf &out, size_t out_cap_hint, double *ms)
{
    size_t k = ins.size();
    if (k == 0 || k + 1 > 128) {
        checkf(false, "merge_csf: 段数 %zu 非法", k);
        return false;
    }
    size_t cap = out_cap_hint ? out_cap_hint : 64 * 1024;
    for (int attempt = 0; attempt < 12; attempt++) {
        if (!tb_alloc(out, cap))
            return false;

        void *mr_addr[128];
        long long mr_len[128];
        mr_addr[0] = out.p;
        mr_len[0] = (long long)out.cap;
        for (size_t i = 0; i < k; i++) {
            mr_addr[i + 1] = ins[i]->p;
            mr_len[i + 1] = (long long)ins[i]->used;
        }
        struct cemu_args args;
        memset(&args, 0, sizeof(args));
        args.numr = (int)k + 1;
        args.mr_addr = mr_addr;
        args.mr_len = mr_len;
        args.cparam1 = (long long)k;
        args.cparam2 = 0;

        auto t0 = std::chrono::steady_clock::now();
        long long rc = fn(&args);
        if (ms)
            *ms += ms_since(t0);

        if (rc == -2) {
            cap *= 4;
            continue;
        }
        checkf(rc >= 0, "merge_csf 返回 %lld (k=%zu)", rc, k);
        if (rc < 0)
            return false;
        out.used = sizeof(struct chunk_table_hdr) + (size_t)rc * sizeof(struct chunk_ent);
        return true;
    }
    checkf(false, "merge_csf: ENOSPC 重试 12 次");
    return false;
}

/* ---------------- 单元检查：CRC32 与 CRC32 拼接 ---------------- */

static void test_crc_impl(crc_fn_t f)
{
    g_case++;
    std::mt19937 rng(2024);
    const size_t lens[] = {0, 1, 2, 7, 8, 9, 15, 16, 17, 31, 33, 255, 256, 1000,
                           4096, 8192, 65536, 1 << 20};
    for (size_t len : lens) {
        size_t alloc = len + 8;
        uint8_t *b = (uint8_t *)aligned_alloc(4096, (alloc + 4095) / 4096 * 4096);
        if (!b) {
            checkf(false, "alloc 失败");
            return;
        }
        for (size_t i = 0; i < alloc; i++)
            b[i] = (uint8_t)rng();
        uint32_t prevs[] = {0u, 0xdeadbeefu, 0xffffffffu};
        for (uint32_t pv : prevs) {
            uint32_t got = f(b + 1, len, pv); /* 故意用非对齐起点 */
            uint32_t want = zcrc(b + 1, len, pv);
            checkf(got == want, "chunk_crc32(len=%zu,prev=0x%08x)=0x%08x != zlib 0x%08x",
                   len, pv, got, want);
        }
        free(b);
    }
}

static void test_crc_comb(comb_fn_t comb)
{
    g_case++;
    std::mt19937 rng(7);
    for (int t = 0; t < 400; t++) {
        size_t la = rng() % 40000;
        size_t lb = 1 + rng() % 40000; /* len2 == 0 单独测，见下 */
        std::vector<uint8_t> a(la + lb);
        for (auto &x : a)
            x = (uint8_t)rng();

        uint32_t ca = zcrc(a.data(), la, 0);
        uint32_t cb = zcrc(a.data() + la, lb, 0);
        uint32_t cab = zcrc(a.data() + la, lb, ca); /* CRC32(A||B)，直接算 */
        uint32_t got = comb(ca, cb, lb);
        checkf(got == cab, "crc32_comb(0x%08x,0x%08x,%zu)=0x%08x != CRC32(A||B)=0x%08x",
               ca, cb, lb, got, cab);
    }
    /* 主机实现的一个 quirk：len2 == 0 时 crc32_comb 返回 crc1 ^ crc2（与 zlib 不同）。
     * 这里把它钉住：如果我们"顺手修好"它，就会和主机 combined_hash 对不上。 */
    uint32_t c1 = 0x12345678u, c2 = 0x9abcdef0u;
    checkf(comb(c1, c2, 0) == (c1 ^ c2), "len2==0 的行为与主机 crc32_comb 不一致");
}

/* ---------------- gear 表一致性 ---------------- */

static bool load_host_gear(const char *path, uint64_t out[256])
{
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "[WARN] 打不开 %s，跳过 gear 表比对\n", path);
        return false;
    }
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
    if (cnt != 256) {
        fprintf(stderr, "[WARN] 从 %s 解析到 %d 项（期望 256），跳过比对\n", path, cnt);
        return false;
    }
    return true;
}

/* ---------------- 单个用例 ---------------- */

static uint64_t g_last_merged_hash = 0;

static uint64_t fnv1a_into(uint64_t h, const std::vector<chunk_ent> &v)
{
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

static void run_case(csf_fn_t chunk_fn, csf_fn_t merge_fn, const uint64_t *gear,
                     const uint8_t *buf, uint64_t n, int nseg, bool verbose)
{
    g_case++;
    uint64_t seg = (n + nseg - 1) / nseg;
    if (seg == 0)
        seg = 1;

    /* ---- 参考实现 ---- */
    std::vector<chunk_ent> raw_ref;
    for (int s = 0; s < nseg; s++) {
        uint64_t off = (uint64_t)s * seg;
        if (off >= n)
            break;
        uint64_t len = n - off;
        if (len > seg)
            len = seg;
        ref_seg_cuts(buf, off, len, gear, raw_ref);
    }
    std::vector<chunk_ent> want = ref_detector(raw_ref, buf);

    /* ---- Stage-1：逐段调用 chunk_csf ---- */
    std::vector<TBuf> tabs;
    std::vector<chunk_ent> raw_csf;
    double ms1 = 0;
    bool ok = true;
    for (int s = 0; s < nseg; s++) {
        uint64_t off = (uint64_t)s * seg;
        if (off >= n)
            break;
        uint64_t len = n - off;
        if (len > seg)
            len = seg;
        tabs.emplace_back();
        ok = run_chunk_csf(chunk_fn, buf + off, off, len, tabs.back(), &ms1) && ok;
        if (!ok)
            break;
        std::vector<chunk_ent> got;
        if (tb_parse(tabs.back(), CHUNK_TABLE_MAGIC, got, nullptr, nullptr, "chunk_csf"))
            raw_csf.insert(raw_csf.end(), got.begin(), got.end());
    }

    /* Stage-1 与参考的"原始切点"逐项比对（含每块 CRC32） */
    checkf(raw_csf.size() == raw_ref.size(), "Stage-1 原始切点数 %zu != 参考 %zu",
           raw_csf.size(), raw_ref.size());
    size_t cmp = raw_csf.size() < raw_ref.size() ? raw_csf.size() : raw_ref.size();
    for (size_t i = 0; i < cmp; i++) {
        if (raw_csf[i].offset != raw_ref[i].offset ||
            raw_csf[i].length != raw_ref[i].length ||
            raw_csf[i].crc32 != raw_ref[i].crc32) {
            checkf(false, "Stage-1 第 %zu 块 (off %lu len %lu crc %08x) != 参考 (off %lu len %lu crc %08x)",
                   i, raw_csf[i].offset, raw_csf[i].length, raw_csf[i].crc32,
                   raw_ref[i].offset, raw_ref[i].length, raw_ref[i].crc32);
            break;
        }
    }

    /* ---- Stage-2：一次作业吃掉所有段表 ---- */
    std::vector<const TBuf *> ins;
    for (auto &t : tabs)
        ins.push_back(&t);

    std::vector<chunk_ent> got;
    double ms2 = 0;
    if (!tabs.empty()) {
        TBuf merged;
        /* 输出容量按输入条目总数给（merge 不会增加条目数） */
        size_t total_ent = 0;
        for (auto &t : tabs)
            total_ent += ((struct chunk_table_hdr *)t.p)->nr_chunks;
        size_t hint = sizeof(struct chunk_table_hdr) + total_ent * sizeof(struct chunk_ent);
        if (run_merge_csf(merge_fn, ins, merged, hint, &ms2)) {
            uint64_t b = 0, e = 0;
            tb_parse(merged, CHUNK_TABLE_MAGIC_MERGED, got, &b, &e, "merge_csf");
            checkf(e - b == n, "merge_csf: 覆盖 %lu != n %lu", e - b, n);
            if (!want.empty())
                checkf(b == want[0].offset, "merge_csf: 起点 %lu != %lu", b, want[0].offset);
            g_last_merged_hash = fnv1a_into(0, got);
        }
        merged.release();
    }

    /* Stage-2 与参考 detector 逐项比对（含合并后的 CRC32） */
    checkf(got.size() == want.size(), "Stage-2 块数 %zu != 参考 %zu", got.size(), want.size());
    cmp = got.size() < want.size() ? got.size() : want.size();
    for (size_t i = 0; i < cmp; i++) {
        if (got[i].offset != want[i].offset || got[i].length != want[i].length ||
            got[i].crc32 != want[i].crc32) {
            checkf(false, "Stage-2 第 %zu 块 (off %lu len %lu crc %08x) != 参考 (off %lu len %lu crc %08x)",
                   i, got[i].offset, got[i].length, got[i].crc32,
                   want[i].offset, want[i].length, want[i].crc32);
            break;
        }
    }

    if (verbose)
        printf("case %2d: n=%8lu B, %2d 段, 原始切点 %zu, 合并后 %zu 块 "
               "(chunk %6.3f ms, merge %6.3f ms)\n",
               g_case, n, nseg, raw_csf.size(), got.size(), ms1, ms2);

    for (auto &t : tabs)
        t.release();
}

/* ---- 负例：段序错乱 / 表被截断，都必须被 merge_csf 明确拒绝 ---- */

static void negative_tests(csf_fn_t chunk_fn, csf_fn_t merge_fn, const uint8_t *buf, uint64_t n)
{
    g_case++;
    uint64_t half = (n / 2 + 511) / 512 * 512;
    std::vector<TBuf> t(2);
    if (!run_chunk_csf(chunk_fn, buf, 0, half, t[0], nullptr) ||
        !run_chunk_csf(chunk_fn, buf + half, half, n - half, t[1], nullptr))
        return;

    /* 1) 段序颠倒 */
    {
        std::vector<const TBuf *> ins = {&t[1], &t[0]};
        TBuf out;
        if (tb_alloc(out, 64 * 1024)) {
            void *mr_addr[3] = {out.p, ins[0]->p, ins[1]->p};
            long long mr_len[3] = {(long long)out.cap, (long long)ins[0]->used,
                                   (long long)ins[1]->used};
            struct cemu_args args;
            memset(&args, 0, sizeof(args));
            args.numr = 3;
            args.mr_addr = mr_addr;
            args.mr_len = mr_len;
            args.cparam1 = 2;
            long long rc = merge_fn(&args);
            checkf(rc == CHUNK_CSF_EINVAL, "段序颠倒应返回 EINVAL，实际 %lld", rc);
            out.release();
        }
    }
    /* 2) 表被截断（mr_len 少 1 字节） */
    {
        std::vector<const TBuf *> ins = {&t[0], &t[1]};
        TBuf out;
        if (tb_alloc(out, 64 * 1024)) {
            void *mr_addr[3] = {out.p, ins[0]->p, ins[1]->p};
            long long mr_len[3] = {(long long)out.cap, (long long)ins[0]->used,
                                   (long long)ins[1]->used - 1};
            struct cemu_args args;
            memset(&args, 0, sizeof(args));
            args.numr = 3;
            args.mr_addr = mr_addr;
            args.mr_len = mr_len;
            args.cparam1 = 2;
            long long rc = merge_fn(&args);
            checkf(rc == CHUNK_CSF_EINVAL, "截断表应返回 EINVAL，实际 %lld", rc);
            out.release();
        }
    }
    /* 3) 输出缓冲太小 -> 必须返回 ENOSPC 让主机放大重试 */
    {
        std::vector<const TBuf *> ins = {&t[0], &t[1]};
        TBuf out;
        /* 只给表头、0 个条目：只要有任意一块输出就必然 ENOSPC */
        size_t small = sizeof(struct chunk_table_hdr);
        if (tb_alloc(out, small)) {
            /* 手工把可用长度改成 small（模拟 MRS 范围比表小） */
            void *mr_addr[3] = {out.p, ins[0]->p, ins[1]->p};
            long long mr_len[3] = {(long long)small, (long long)ins[0]->used,
                                   (long long)ins[1]->used};
            struct cemu_args args;
            memset(&args, 0, sizeof(args));
            args.numr = 3;
            args.mr_addr = mr_addr;
            args.mr_len = mr_len;
            args.cparam1 = 2;
            long long rc = merge_fn(&args);
            checkf(rc == CHUNK_CSF_ENOSPC, "输出过小应返回 ENOSPC，实际 %lld", rc);
            out.release();
        }
    }
    /* 4) 段数越界 */
    {
        std::vector<const TBuf *> ins = {&t[0], &t[1]};
        TBuf out;
        if (tb_alloc(out, 64 * 1024)) {
            void *mr_addr[3] = {out.p, ins[0]->p, ins[1]->p};
            long long mr_len[3] = {(long long)out.cap, (long long)ins[0]->used,
                                   (long long)ins[1]->used};
            struct cemu_args args;
            memset(&args, 0, sizeof(args));
            args.numr = 3;
            args.mr_addr = mr_addr;
            args.mr_len = mr_len;
            args.cparam1 = 3; /* > numr-1 */
            long long rc = merge_fn(&args);
            checkf(rc == CHUNK_CSF_EINVAL, "段数越界应返回 EINVAL，实际 %lld", rc);
            out.release();
        }
    }
    for (auto &x : t)
        x.release();
}

/* ---------------- 数据生成 ---------------- */

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

int main(int argc, char **argv)
{
    const char *chunk_so = "./build/chunk_csf.so";
    const char *merge_so = "./build/merge_csf.so";
    const char *host_header = "/home/zyt/parasync/src/fastcdc.h";
    bool print_hash = false;
    int seg_override = 0;
    int size_mb = 0;
    uint64_t hash_n = 0;
    int hash_k = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--chunk-so") && i + 1 < argc)
            chunk_so = argv[++i];
        else if (!strcmp(argv[i], "--merge-so") && i + 1 < argc)
            merge_so = argv[++i];
        else if (!strcmp(argv[i], "--host-header") && i + 1 < argc)
            host_header = argv[++i];
        else if (!strcmp(argv[i], "--segments") && i + 1 < argc)
            seg_override = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--size-mb") && i + 1 < argc)
            size_mb = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--hash"))
            print_hash = true;
        else {
            fprintf(stderr, "未知参数 %s\n", argv[i]);
            return 1;
        }
    }

    void *h1 = dlopen(chunk_so, RTLD_NOW);
    if (!h1) {
        fprintf(stderr, "dlopen %s 失败: %s\n", chunk_so, dlerror());
        return 1;
    }
    void *h2 = dlopen(merge_so, RTLD_NOW);
    if (!h2) {
        fprintf(stderr, "dlopen %s 失败: %s\n", merge_so, dlerror());
        return 1;
    }
    csf_fn_t chunk_fn = (csf_fn_t)dlsym(h1, "chunk_csf");
    csf_fn_t merge_fn = (csf_fn_t)dlsym(h2, "merge_csf");
    gear_fn_t gear_of = (gear_fn_t)dlsym(h1, "chunk_csf_gear_table");
    crc_fn_t crc_fn = (crc_fn_t)dlsym(h1, "chunk_crc32");
    comb_fn_t comb_fn = (comb_fn_t)dlsym(h1, "chunk_crc32_comb");
    if (!chunk_fn || !merge_fn || !gear_of || !crc_fn || !comb_fn) {
        fprintf(stderr, "dlsym 失败: chunk_csf=%p merge_csf=%p gear=%p crc32=%p comb=%p (%s)\n",
                (void *)chunk_fn, (void *)merge_fn, (void *)gear_of,
                (void *)crc_fn, (void *)comb_fn, dlerror());
        return 1;
    }
    const uint64_t *gear = gear_of();

    printf("== L0: CDC 算子族正确性验证 ==\n");
    printf("Stage-1: %s\nStage-2: %s\n", chunk_so, merge_so);

    /* 0. CRC32 / CRC32 拼接 */
    g_case = 0;
    test_crc_impl(crc_fn);
    printf("chunk_crc32  : 与 zlib crc32 逐值一致\n");
    test_crc_comb(comb_fn);
    printf("chunk_crc32_comb: 与 CRC32(A||B) 逐值一致\n");

    /* 1. gear 表一致性 */
    uint64_t host_gear[256];
    if (load_host_gear(host_header, host_gear)) {
        int diff = -1;
        for (int i = 0; i < 256; i++)
            if (gear[i] != host_gear[i]) {
                diff = i;
                break;
            }
        if (diff >= 0)
            checkf(false, "gear 表第 %d 项与 %s 不一致: csf=0x%lx host=0x%lx",
                   diff, host_header, gear[diff], host_gear[diff]);
        else
            printf("gear 表      : 与 %s 的 GEARv2 逐字节一致 (256 项)\n", host_header);
    }

    /* 2. 用例矩阵 */
    struct {
        uint64_t n;
        int pattern;
    } cases[] = {
        {0, 0}, {1, 0}, {4095, 0}, {4096, 0}, {4097, 0},
        {8192, 0}, {64 * 1024, 0}, {1 << 20, 0}, {1 << 20, 1}, {1 << 20, 2},
        {4 << 20, 2},
    };
    int nseg_list[] = {1, 2, 3, 4, 7};

    for (auto &c : cases) {
        uint64_t cap = c.n ? (((c.n + 4095) / 4096) * 4096) : 4096;
        uint8_t *buf = (uint8_t *)aligned_alloc(4096, cap);
        memset(buf, 0, cap);
        gen(buf, c.n, c.pattern, 12345);
        for (int ns : nseg_list) {
            if (c.n < 16 && ns > 1)
                continue;
            run_case(chunk_fn, merge_fn, gear, buf, c.n, seg_override ? seg_override : ns, true);
        }
        if (c.n >= 4096)
            negative_tests(chunk_fn, merge_fn, buf, c.n);
        free(buf);
    }

    /* 3. 可选大数据：--size-mb N（测吞吐 + 大输入正确性） */
    if (size_mb > 0) {
        uint64_t n = (uint64_t)size_mb << 20;
        uint64_t cap = (n + 4095) / 4096 * 4096;
        uint8_t *buf = (uint8_t *)aligned_alloc(4096, cap);
        if (!buf) {
            fprintf(stderr, "alloc %lu 失败\n", n);
            return 1;
        }
        memset(buf, 0, cap);
        gen(buf, n, 0, 12345);
        hash_n = n;
        hash_k = seg_override ? seg_override : 4;
        printf("\n-- 大数据: %d MiB, %d 段 --\n", size_mb, hash_k);
        run_case(chunk_fn, merge_fn, gear, buf, n, hash_k, true);
        print_hash = true;
        free(buf);
    }

    if (print_hash) {
        if (hash_n == 0) {
            /* 没指定规模时用固定锚点：1MiB 随机(seed=12345) + 4 段 */
            hash_n = 1 << 20;
            hash_k = 4;
            uint8_t *buf = (uint8_t *)aligned_alloc(4096, hash_n);
            if (buf) {
                memset(buf, 0, hash_n);
                gen(buf, hash_n, 0, 12345);
                run_case(chunk_fn, merge_fn, gear, buf, hash_n, hash_k, false);
                free(buf);
            }
        }
        printf("\n合并表 FNV-1a64 指纹 (n=%lu B, %d 段, pattern 0): 0x%016lx\n",
               hash_n, hash_k, g_last_merged_hash);
        printf("CEMU 侧复核: sudo ./build/test_chunk_csf_cemu --size-mb %lu --segments %d "
               "应得到同一指纹\n", hash_n >> 20, hash_k);
    }

    printf("\n== 结果: %s (失败 %d 项) ==\n", g_fail ? "FAIL" : "PASS", g_fail);
    dlclose(h2);
    dlclose(h1);
    return g_fail ? 1 : 0;
}
