/*
 * chunk_crc32.h —— CSD 侧 CRC-32 与 CRC-32 拼接
 *
 * chunk_crc32()      : 标准 CRC-32/ISO-HDLC（同 zlib crc32 / 主机 parasync_crc32）
 * chunk_crc32_comb() : 逐字复刻主机 parasync/src/crc32/crc32.c 的 pigz 风格
 *                      crc32_comb()，用于把分块 CRC 增量拼成整块 CRC
 */
#ifndef CHUNK_CRC32_H
#define CHUNK_CRC32_H

#include <stdint.h>
#include <stddef.h>

/* 等价于 zlib 的 crc32(previous_crc32, data, length) */
uint32_t chunk_crc32(const void *data, size_t length, uint32_t previous_crc32);

/* 等价于主机 crc32_comb(crc1, crc2, len2)：CRC32(A||B) = comb(CRC32(A), CRC32(B), |B|) */
uint32_t chunk_crc32_comb(uint32_t crc1, uint32_t crc2, size_t len2);

/* 追加 len2 个 0 字节后的 CRC32 */
uint32_t chunk_crc32_add0(uint32_t crc1, size_t len2);

#endif /* CHUNK_CRC32_H */
