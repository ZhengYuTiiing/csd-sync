/*
 * chunk_crc32.c —— CSD 侧 CRC-32 与 CRC-32 拼接（与主机 parasync 一致）
 *
 * 1) chunk_crc32()：标准 CRC-32/ISO-HDLC（多项式 0xEDB88320、反射、初值 0、
 *    末尾取反），语义等价于主机 parasync_crc32()：
 *        g_parasync_hw_crc32 ? crc32_isal(data,len,crc) : crc32_fast(data,len,crc)
 *    这两个实现都必须等于 zlib 的 crc32()（见 parasync_common.h:23）。
 *    本实现用 slicing-by-8，表在 dlopen 时由 constructor 建好，不依赖 zlib/isa-l。
 *    （L0 harness 会用 zlib 的 crc32 逐字节比对，见 test_chunk_csf.cpp）
 *
 * 2) chunk_crc32_comb()：逐字复刻 parasync/src/crc32/crc32.c 里的 pigz 风格
 *    crc32_comb()（含 multmodp / x2nmodp / x2n_table）。Stage-2 合并算子必须用
 *    它把分块 CRC 增量拼成整块 CRC，才能和主机 detector() 的 combined_hash
 *    逐位一致。注意 len2 == 0 时主机实现返回 crc1 ^ crc2（与 zlib 的
 *    crc32_combine 不同），这里保持完全一致。
 */

#include <stdint.h>
#include <stddef.h>

#include "chunk_crc32.h"

/* =================== 1. 标准 CRC-32（slicing-by-8） =================== */

#define CRC32_POLY 0xEDB88320u

static uint32_t s_tab[8][256];
static volatile int s_tab_ready;

static void crc32_build_tables(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1u) ? (CRC32_POLY ^ (c >> 1)) : (c >> 1);
        s_tab[0][i] = c;
    }
    for (uint32_t i = 0; i < 256; i++)
        for (int j = 1; j < 8; j++)
            s_tab[j][i] = (s_tab[j - 1][i] >> 8) ^ s_tab[0][s_tab[j - 1][i] & 0xFFu];

    s_tab_ready = 1;
}

/* dlopen 时建表（单线程），避免 CU 并发首次调用时的竞态 */
__attribute__((constructor)) static void crc32_ctor(void)
{
    crc32_build_tables();
}

static inline uint32_t load_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t chunk_crc32(const void *data, size_t length, uint32_t previous_crc32)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc;

    if (!s_tab_ready) /* 兜底：正常情况下 constructor 已建好表 */
        crc32_build_tables();

    crc = ~previous_crc32;

    while (length >= 8) {
        uint32_t lo = load_le32(p);
        uint32_t hi = load_le32(p + 4);
        crc ^= lo;
        crc = s_tab[7][crc & 0xFFu] ^ s_tab[6][(crc >> 8) & 0xFFu] ^
              s_tab[5][(crc >> 16) & 0xFFu] ^ s_tab[4][(crc >> 24) & 0xFFu] ^
              s_tab[3][hi & 0xFFu] ^ s_tab[2][(hi >> 8) & 0xFFu] ^
              s_tab[1][(hi >> 16) & 0xFFu] ^ s_tab[0][(hi >> 24) & 0xFFu];
        p += 8;
        length -= 8;
    }
    while (length--)
        crc = s_tab[0][(crc ^ *p++) & 0xFFu] ^ (crc >> 8);

    return ~crc;
}

/* =================== 2. CRC-32 拼接（逐字复刻主机实现） =================== */
/* 以下 multmodp / x2nmodp / x2n_table / crc32_comb 复制自
 * parasync/src/crc32/crc32.c（pigz crc32_combine），不要改动。 */

#define POLY       0xedb88320
#define PIGZ_BLOCK 131072UL
#define PIGZ_SHIFT x2nmodp(PIGZ_BLOCK, 3)

/* Table of x^2^n modulo p(x). */
static const uint32_t x2n_table[32] = {
    0x40000000, 0x20000000, 0x08000000, 0x00800000, 0x00008000,
    0xedb88320, 0xb1e6b092, 0xa06a2517, 0xed627dae, 0x88d14467,
    0xd7bbfe6a, 0xec447f11, 0x8e7ea170, 0x6427800e, 0x4d47bae0,
    0x09fe548f, 0x83852d0f, 0x30362f1a, 0x7b5a9cc3, 0x31fec169,
    0x9fec022a, 0x6c8dedc4, 0x15d6874d, 0x5fde7a4e, 0xbad90e37,
    0x2e4e5eef, 0x4eaba214, 0xa8a472c0, 0x429a969e, 0x148d302a,
    0xc40ba6d0, 0xc4e22c3c
};

static uint32_t multmodp(uint32_t a, uint32_t b)
{
    uint32_t m = (uint32_t)1 << 31;
    uint32_t p = 0;
    for (;;) {
        if (a & m) {
            p ^= b;
            if ((a & (m - 1)) == 0)
                break;
        }
        m >>= 1;
        b = b & 1 ? (b >> 1) ^ POLY : b >> 1;
    }
    return p;
}

/* Return x^(n*2^k) modulo p(x). */
static uint32_t x2nmodp(size_t n, unsigned k)
{
    uint32_t p = (uint32_t)1 << 31; /* x^0 == 1 */
    while (n) {
        if (n & 1)
            p = multmodp(x2n_table[k & 31], p);
        n >>= 1;
        k++;
    }
    return p;
}

uint32_t chunk_crc32_comb(uint32_t crc1, uint32_t crc2, size_t len2)
{
    return multmodp(len2 == PIGZ_BLOCK ? PIGZ_SHIFT : x2nmodp(len2, 3), crc1) ^ crc2;
}

uint32_t chunk_crc32_add0(uint32_t crc1, size_t len2)
{
    return multmodp(len2 == PIGZ_BLOCK ? PIGZ_SHIFT : x2nmodp(len2, 3), crc1);
}
