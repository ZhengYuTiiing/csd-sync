/*
 * merge_csf.c —— ParaSync CDC 的 Stage-2 合并算子（下沉到 CEMU CSD）
 *
 * 语义 = 主机 parasync/src/parasync/parasync_worker.cpp 的
 *        ParaSyncWorker::detector()（parasync_worker.cpp:28-83）逐字翻译：
 *
 *     total = 0; combined = 0; first = true
 *     for (段序) for (段内条目序):
 *         if (first) { start = e.offset; first = false; }
 *         total += e.length;
 *         combined = crc32_comb(combined, e.crc32, e.length);
 *         if (total >= MinSize) { 输出(start,total,combined); total=0; combined=0; first=true; }
 *     最后把全局残余输出成一块
 *
 * 为什么它必须是"单独一个作业、吃有序拼接后的所有段表"：
 *   detector 的累加器（total/combined/first）是**跨段全局**的，第 i 段合并时必须
 *   知道前 i-1 段遗留了多少 pending 字节。所以本算子一次作业就要拿到按段序排好的
 *   全部切点表，中间不能让任何一段落到其它作业里。
 *
 * 与主机的逐项等价性（都是必须成立的强约束）：
 *   - 输入顺序 = 段序 = 文件偏移序（本算子会校验覆盖连续性，顺序错直接报错）
 *   - total >= MinSize 才 flush；flush 用 combined，不重算
 *   - combined 用 chunk_crc32_comb(...)，与主机 crc32_comb 逐位一致
 *   - 因为 A||B 的 CRC32 == comb(CRC32(A), CRC32(B), |B|)，所以输出块的 crc32
 *     等于对合并区间直接算 CRC32（L0 harness 用 zlib 直接重算来验证这条不变量）
 *
 * CSF 接口（direct model + MRS）:
 *   mr[1..k] / mr_len[1..k] : k 个 Stage-1 切点表，**必须按文件段序排列**
 *   mr[0]    / mr_len[0]    : 输出合并表
 *   cparam1                 : k（段数，1 <= k <= numr-1，numr 上限 128）
 *   cparam2                 : 保留，必须为 0
 *   返回值                  : 合并后的块数；<0 错误
 *                             CHUNK_CSF_ENOSPC：输出表放不下（merge 不会增加条目数，
 *                             所以只要 out 容量 >= 各输入表 entry_bytes 之和就不会发生）
 */

#include <stdint.h>
#include <stddef.h>

#include "cemu_def.h"
#include "chunk_abi.h"
#include "chunk_crc32.h"

long long merge_csf(struct cemu_args *args)
{
    if (args == NULL || args->mr_addr == NULL || args->mr_len == NULL)
        return CHUNK_CSF_EINVAL;

    uint64_t k = (uint64_t)args->cparam1;
    if (args->numr < 2 || k == 0 || k > (uint64_t)args->numr - 1)
        return CHUNK_CSF_EINVAL;
    if (args->cparam2 != 0)
        return CHUNK_CSF_EINVAL;

    uint8_t *out = (uint8_t *)args->mr_addr[0];
    uint64_t out_cap = (uint64_t)args->mr_len[0];
    if (out == NULL)
        return CHUNK_CSF_EINVAL;
    if (out_cap < sizeof(struct chunk_table_hdr))
        return CHUNK_CSF_ENOSPC;

    struct chunk_table_hdr *oh = (struct chunk_table_hdr *)out;
    struct chunk_ent *oe = (struct chunk_ent *)(out + sizeof(*oh));
    uint64_t ocap = (out_cap - sizeof(*oh)) / sizeof(struct chunk_ent);

    uint64_t base0 = 0, cov_end = 0;
    int have_base = 0;
    int crc_ok = 1;
    uint64_t nr = 0;

    /* detector 的全局累加器 */
    uint64_t total = 0;
    uint64_t start = 0;
    uint32_t combined = 0;
    int first = 1;

    for (uint64_t s = 1; s <= k; s++) {
        const uint8_t *tp = (const uint8_t *)args->mr_addr[s];
        uint64_t tcap = (uint64_t)args->mr_len[s];

        if (tp == NULL || tcap < sizeof(struct chunk_table_hdr))
            return CHUNK_CSF_EINVAL;

        const struct chunk_table_hdr *th = (const struct chunk_table_hdr *)tp;
        if (th->magic != CHUNK_TABLE_MAGIC && th->magic != CHUNK_TABLE_MAGIC_MERGED)
            return CHUNK_CSF_EINVAL;
        if (th->entry_bytes != th->nr_chunks * sizeof(struct chunk_ent))
            return CHUNK_CSF_EINVAL;
        if (sizeof(*th) + th->entry_bytes > tcap)
            return CHUNK_CSF_EINVAL;

        /* 段序 + 覆盖连续性：这是唯一会"静默算错"的风险，宁可报错 */
        if (!have_base) {
            base0 = th->base_offset;
            cov_end = th->base_offset + th->data_len;
            have_base = 1;
        } else {
            if (th->base_offset != cov_end)
                return CHUNK_CSF_EINVAL;
            cov_end += th->data_len;
        }

        const struct chunk_ent *te = (const struct chunk_ent *)(tp + sizeof(*th));
        for (uint64_t i = 0; i < th->nr_chunks; i++) {
            if (first) {
                start = te[i].offset;
                first = 0;
            }
            total += te[i].length;
            if (te[i].flags & CHUNK_ENT_CRC_VALID)
                combined = chunk_crc32_comb(combined, te[i].crc32, te[i].length);
            else
                crc_ok = 0;

            if (total >= CHUNK_MIN_SIZE) {
                if (nr == ocap)
                    return CHUNK_CSF_ENOSPC;
                oe[nr].offset = start;
                oe[nr].length = total;
                oe[nr].crc32 = combined;
                oe[nr].flags = CHUNK_ENT_MERGED | CHUNK_ENT_CRC_VALID;
                nr++;
                total = 0;
                combined = 0;
                first = 1;
            }
        }
    }

    /* 全局残余（主机 detector 最后那个 if (total_length > 0)） */
    if (total > 0) {
        if (nr == ocap)
            return CHUNK_CSF_ENOSPC;
        oe[nr].offset = start;
        oe[nr].length = total;
        oe[nr].crc32 = combined;
        oe[nr].flags = CHUNK_ENT_MERGED | CHUNK_ENT_CRC_VALID | CHUNK_ENT_TAIL_OPEN;
        nr++;
    }

    /* 输入没带 CRC 时，输出也不谎报 CRC_VALID（主机需另找弱校验和算子补） */
    if (!crc_ok)
        for (uint64_t i = 0; i < nr; i++)
            oe[i].flags &= ~(uint32_t)CHUNK_ENT_CRC_VALID;

    oh->magic = CHUNK_TABLE_MAGIC_MERGED;
    oh->nr_chunks = nr;
    oh->base_offset = have_base ? base0 : 0;
    oh->data_len = have_base ? (cov_end - base0) : 0;
    oh->flags = (nr > 0 && (oe[nr - 1].flags & CHUNK_ENT_TAIL_OPEN)) ? CHUNK_FLAG_TAIL_OPEN : 0;
    oh->entry_bytes = nr * sizeof(struct chunk_ent);
    return (long long)nr;
}
