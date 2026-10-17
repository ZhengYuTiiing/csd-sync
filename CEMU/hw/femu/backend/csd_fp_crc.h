#ifndef __FEMU_CSD_FP_CRC_H
#define __FEMU_CSD_FP_CRC_H

/*
 * 可组合的扇区级校验和网格。
 *
 * 每个 4KB 页的 OOB 槽（CSD_FP_LEN = 32B）存 8 个 CRC32C，每个对应页内一个
 * 512B 扇区。CRC32C 满足
 *
 *     crc(A||B) = combine(crc(A), crc(B), len(B))
 *
 * 因此任意 512B 对齐的区间，其校验和都能由若干定长槽位拼出来，
 * 不需要读任何数据、也不需要重新哈希。
 *
 * 非 512B 对齐的边界无法只用槽位得到，需要额外读取至多两个扇区的数据。
 */

#include <stdint.h>
#include <stddef.h>

#define CSD_CRC_LEN        4u
#define CSD_CRC_PER_PAGE   8u                        /* 4096 / 512 */
#define CSD_CRC_SLOT_LEN   (CSD_CRC_LEN * CSD_CRC_PER_PAGE)   /* 32 = CSD_FP_LEN */

#define CSD_CRC32C_POLY    0x82F63B78u   /* Castagnoli, reflected */

static uint32_t csd_crc_tbl[256];
static int csd_crc_tbl_ready;

static inline void csd_crc32c_init(void)
{
    if (csd_crc_tbl_ready) {
        return;
    }
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (CSD_CRC32C_POLY ^ (c >> 1)) : (c >> 1);
        }
        csd_crc_tbl[i] = c;
    }
    csd_crc_tbl_ready = 1;
}

/* 标准 CRC-32C：init 0xFFFFFFFF，输出再异或 0xFFFFFFFF */
static inline uint32_t csd_crc32c(uint32_t crc, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;

    csd_crc32c_init();
    crc ^= 0xFFFFFFFFu;
    while (len--) {
        crc = csd_crc_tbl[(crc ^ *p++) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

static inline uint32_t csd_gf2_times(const uint32_t *mat, uint32_t vec)
{
    uint32_t sum = 0;

    while (vec) {
        if (vec & 1u) {
            sum ^= *mat;
        }
        vec >>= 1;
        mat++;
    }
    return sum;
}

static inline void csd_gf2_square(uint32_t *square, const uint32_t *mat)
{
    for (int n = 0; n < 32; n++) {
        square[n] = csd_gf2_times(mat, mat[n]);
    }
}

/*
 * crc(A||B) = csd_crc32c_combine(crc(A), crc(B), len(B))
 * 与 zlib 的 crc32_combine 同构，多项式换成 Castagnoli。
 */
static inline uint32_t csd_crc32c_combine(uint32_t crc1, uint32_t crc2, uint64_t len2)
{
    uint32_t even[32];
    uint32_t odd[32];
    uint32_t row;

    if (len2 == 0) {
        return crc1;
    }

    /* 一位零比特的算子 */
    odd[0] = CSD_CRC32C_POLY;
    row = 1;
    for (int n = 1; n < 32; n++) {
        odd[n] = row;
        row <<= 1;
    }

    csd_gf2_square(even, odd);      /* 两位 */
    csd_gf2_square(odd, even);      /* 四位 */

    do {
        csd_gf2_square(even, odd);  /* 一字节 */
        if (len2 & 1u) {
            crc1 = csd_gf2_times(even, crc1);
        }
        len2 >>= 1;
        if (len2 == 0) {
            break;
        }
        csd_gf2_square(odd, even);  /* 两字节 */
        if (len2 & 1u) {
            crc1 = csd_gf2_times(odd, crc1);
        }
        len2 >>= 1;
    } while (len2 != 0);

    return crc1 ^ crc2;
}

/*
 * 512B 移位算子缓存
 *
 * 朴素 combine 每次调用都要重算 log2(len) 次 32x32 的 GF(2) 矩阵平方
 * （单次约 9.2e3 步）。按扇区折叠时 len 恒为 512，这 9.2e3 步完全是在
 * 重复计算同一个算子。这里把它预先算成矩阵 M，于是
 *
 *     combine(c1, c2, 512) = M·c1 ^ c2
 *
 * 退化成一次 32 步矩阵乘，快约 300 倍。1GB 文件有 2M 个扇区，
 * 不做这步优化整个拼接路径是不可用的。
 */
static uint32_t csd_shift_mat[32];
static int csd_shift_mat_ready;

static inline void csd_gf2_mat_mul(uint32_t *res, const uint32_t *a, const uint32_t *b)
{
    for (int n = 0; n < 32; n++) {
        res[n] = csd_gf2_times(b, a[n]);
    }
}

/* 构造"追加 nbits 个零比特"的线性算子 */
static inline void csd_gf2_mat_pow(uint32_t *out, uint64_t nbits)
{
    uint32_t one[32], base[32], acc[32], tmp[32];
    uint32_t row;

    /* 一位零比特的算子 */
    one[0] = CSD_CRC32C_POLY;
    row = 1;
    for (int n = 1; n < 32; n++) {
        one[n] = row;
        row <<= 1;
    }
    for (int n = 0; n < 32; n++) {
        base[n] = one[n];
        acc[n] = 1u << n;      /* 单位算子 */
    }

    while (nbits) {
        if (nbits & 1u) {
            csd_gf2_mat_mul(tmp, acc, base);
            for (int n = 0; n < 32; n++) {
                acc[n] = tmp[n];
            }
        }
        nbits >>= 1;
        if (nbits) {
            csd_gf2_mat_mul(tmp, base, base);
            for (int n = 0; n < 32; n++) {
                base[n] = tmp[n];
            }
        }
    }
    for (int n = 0; n < 32; n++) {
        out[n] = acc[n];
    }
}

/* combine(c1, c2, 512)，单次 32 步 */
static inline uint32_t csd_crc32c_combine512(uint32_t crc1, uint32_t crc2)
{
    if (!csd_shift_mat_ready) {
        csd_gf2_mat_pow(csd_shift_mat, 512ULL * 8ULL);
        csd_shift_mat_ready = 1;
    }
    return csd_gf2_times(csd_shift_mat, crc1) ^ crc2;
}

/* 把一页的 8 个扇区 CRC 写进 OOB 槽（小端） */
static inline void csd_crc_slot_build(const void *page4k, uint8_t slot[CSD_CRC_SLOT_LEN])
{
    const uint8_t *p = (const uint8_t *)page4k;

    for (uint32_t s = 0; s < CSD_CRC_PER_PAGE; s++) {
        uint32_t c = csd_crc32c(0, p + (size_t)s * 512, 512);
        slot[s * 4 + 0] = (uint8_t)(c & 0xFF);
        slot[s * 4 + 1] = (uint8_t)((c >> 8) & 0xFF);
        slot[s * 4 + 2] = (uint8_t)((c >> 16) & 0xFF);
        slot[s * 4 + 3] = (uint8_t)((c >> 24) & 0xFF);
    }
}

static inline uint32_t csd_crc_slot_get(const uint8_t slot[CSD_CRC_SLOT_LEN], uint32_t sector)
{
    const uint8_t *p = slot + sector * 4;

    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#endif /* __FEMU_CSD_FP_CRC_H */
