#ifndef __FEMU_CSD_SHA1_H
#define __FEMU_CSD_SHA1_H

/*
 * 设备内 SHA1 —— 与主机侧 A 组（dsync, StrongHashingAlgorithm::SHA1）同算法，
 * 这样 C 的「强校验和」与 A 是同一把尺子，两组的命中率/耗时才有可比性。
 *
 * 自包含实现，不依赖 openssl（CSF 是设备内 .so，依赖越少越好）。
 * 输出 20 字节，调用方负责按需补齐到描述符的 32B 字段。
 */

#include <stdint.h>
#include <string.h>

#define CSD_SHA1_LEN 20

typedef struct {
    uint32_t h[5];
    uint64_t total;          /* 累计输入字节数 */
    uint8_t  buf[64];
    uint32_t buflen;         /* buf 中待处理的字节数 */
} csd_sha1_ctx;

static inline uint32_t csd_sha1_rol(uint32_t v, int n)
{
    return (v << n) | (v >> (32 - n));
}

static inline void csd_sha1_block(csd_sha1_ctx *c, const uint8_t *p)
{
    uint32_t w[80];
    uint32_t a, b, d, e, f, k, t;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ((uint32_t)p[i * 4]     << 24) |
               ((uint32_t)p[i * 4 + 1] << 16) |
               ((uint32_t)p[i * 4 + 2] <<  8) |
               ((uint32_t)p[i * 4 + 3]);
    }
    for (i = 16; i < 80; i++) {
        w[i] = csd_sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    a = c->h[0]; b = c->h[1]; f = c->h[2]; d = c->h[3]; e = c->h[4];

    for (i = 0; i < 80; i++) {
        if (i < 20) {
            t = ((b & f) | (~b & d)) + 0x5A827999u;
            k = csd_sha1_rol(a, 5);
        } else if (i < 40) {
            t = (b ^ f ^ d) + 0x6ED9EBA1u;
            k = csd_sha1_rol(a, 5);
        } else if (i < 60) {
            t = ((b & f) | (b & d) | (f & d)) + 0x8F1BBCDCu;
            k = csd_sha1_rol(a, 5);
        } else {
            t = (b ^ f ^ d) + 0xCA62C1D6u;
            k = csd_sha1_rol(a, 5);
        }
        t += k + e + w[i];
        e = d;
        d = f;
        f = csd_sha1_rol(b, 30);
        b = a;
        a = t;
    }

    c->h[0] += a; c->h[1] += b; c->h[2] += f; c->h[3] += d; c->h[4] += e;
}

static inline void csd_sha1_init(csd_sha1_ctx *c)
{
    c->h[0] = 0x67452301u;
    c->h[1] = 0xEFCDAB89u;
    c->h[2] = 0x98BADCFEu;
    c->h[3] = 0x10325476u;
    c->h[4] = 0xC3D2E1F0u;
    c->total = 0;
    c->buflen = 0;
}

static inline void csd_sha1_update(csd_sha1_ctx *c, const void *data, uint64_t len)
{
    const uint8_t *p = (const uint8_t *)data;

    c->total += len;

    if (c->buflen) {
        uint32_t need = 64u - c->buflen;
        if (len < need) {
            memcpy(c->buf + c->buflen, p, (size_t)len);
            c->buflen += (uint32_t)len;
            return;
        }
        memcpy(c->buf + c->buflen, p, (size_t)need);
        csd_sha1_block(c, c->buf);
        c->buflen = 0;
        p += need;
        len -= need;
    }

    while (len >= 64) {
        csd_sha1_block(c, p);
        p += 64;
        len -= 64;
    }

    if (len) {
        memcpy(c->buf, p, (size_t)len);
        c->buflen = (uint32_t)len;
    }
}

static inline void csd_sha1_final(csd_sha1_ctx *c, uint8_t out[CSD_SHA1_LEN])
{
    uint64_t bits = c->total * 8u;
    uint8_t pad[72];
    uint32_t padlen, i;

    /* 0x80 + 补零到 (总长 ≡ 56 mod 64)，再追加 8 字节大端位长 */
    pad[0] = 0x80;
    padlen = (c->buflen < 56u) ? (56u - c->buflen) : (120u - c->buflen);
    memset(pad + 1, 0, (size_t)(padlen - 1));
    for (i = 0; i < 8; i++) {
        pad[padlen + i] = (uint8_t)(bits >> (56 - 8 * i));
    }
    csd_sha1_update(c, pad, (uint64_t)padlen + 8u);

    for (i = 0; i < 5; i++) {
        out[i * 4]     = (uint8_t)(c->h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(c->h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(c->h[i] >>  8);
        out[i * 4 + 3] = (uint8_t)(c->h[i]);
    }
}

/* 一次性接口 */
static inline void csd_sha1(const void *data, uint64_t len, uint8_t out[CSD_SHA1_LEN])
{
    csd_sha1_ctx c;
    csd_sha1_init(&c);
    csd_sha1_update(&c, data, len);
    csd_sha1_final(&c, out);
}

#endif /* __FEMU_CSD_SHA1_H */
