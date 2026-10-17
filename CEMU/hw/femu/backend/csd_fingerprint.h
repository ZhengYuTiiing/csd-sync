#ifndef __FEMU_CSD_FINGERPRINT_H
#define __FEMU_CSD_FINGERPRINT_H

/*
 * 固定 32 字节的页指纹，用于写时 OOB 固化 + 读时浅读。
 * 实现采用 4 条独立 FNV1a-mix 并行吸收，输出 4×u64 拼成 32B。
 * 不追求密码学强度 — 用于模拟 NAND OOB 中 32B SHA256 的占位。
 * 若需要换成 BLAKE3/SHA256 真实算法，只替换 csd_fp256() 函数体即可。
 */

#include <stdint.h>
#include <string.h>

#define CSD_FP_LEN 32

#ifndef CSD_PAGE_SIZE
#define CSD_PAGE_SIZE 4096u
#endif

static inline void csd_fp256(const void *data, uint64_t len, uint8_t out[CSD_FP_LEN])
{
    const uint64_t seeds[4] = {
        0x9E3779B97F4A7C15ULL,
        0xBF58476D1CE4E5B9ULL,
        0x94D049BB133111EBULL,
        0x9FB21C651E98DF25ULL,
    };
    uint64_t h[4] = { seeds[0], seeds[1], seeds[2], seeds[3] };
    const uint8_t *p = (const uint8_t *)data;
    uint64_t i;

    for (i = 0; i + 8 <= len; i += 8) {
        uint64_t v;
        memcpy(&v, p + i, 8);
        h[0] ^= v;              h[0] *= 0x100000001B3ULL;
        h[1] ^= (v + 1);        h[1] *= 0xC6BC279692B5C323ULL;
        h[2] ^= (v + 2);        h[2] *= 0xA4093822299F31D0ULL;
        h[3] ^= (v + 3);        h[3] *= 0xDA942042E4DD58B5ULL;
        h[0] ^= h[0] >> 33;
        h[1] ^= h[1] >> 29;
        h[2] ^= h[2] >> 31;
        h[3] ^= h[3] >> 27;
    }
    if (i < len) {
        uint64_t tail = 0;
        memcpy(&tail, p + i, (size_t)(len - i));
        h[0] ^= tail; h[0] *= 0x100000001B3ULL; h[0] ^= h[0] >> 33;
        h[1] ^= tail; h[1] *= 0xC6BC279692B5C323ULL; h[1] ^= h[1] >> 29;
        h[2] ^= tail; h[2] *= 0xA4093822299F31D0ULL; h[2] ^= h[2] >> 31;
        h[3] ^= tail; h[3] *= 0xDA942042E4DD58B5ULL; h[3] ^= h[3] >> 27;
    }

    memcpy(out +  0, &h[0], 8);
    memcpy(out +  8, &h[1], 8);
    memcpy(out + 16, &h[2], 8);
    memcpy(out + 24, &h[3], 8);
}

#endif /* __FEMU_CSD_FINGERPRINT_H */
