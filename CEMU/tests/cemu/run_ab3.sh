#!/bin/bash
# 三组同环境对比（在虚拟机里跑）
#   [A] host-rsync   : 不在 CSD。数据搬到主机，CPU 现场算 CDC + 弱/强校验和
#                      （/root/dsync_time_test，LD_LIBRARY_PATH=/root/libs）
#   [B] grid-no-cdc  : 在 CSD。校验和由网格组合出来；只有 client 侧 CDC，
#                      server 只折叠网格、不读 f 的数据
#                      （build/dsync）
#   [C] grid-cdc     : 在 CSD。校验和由网格组合出来；两侧各自 CDC，按内容匹配
#                      （build/gridds）
#
# 控制变量：同一台虚拟机、同一对文件、同一 vCPU、同一存储、每轮前 drop_caches。
# 自变量：A vs B/C = 同步计算是否下沉到 CSD；B vs C = server 侧是否做 CDC。
#
# 时间口径：
#   A = 四项合计 + Server Patch（它的四项里不含 patch）
#   B/C = 程序总耗时（含设备侧各阶段 + 主机侧重建）
#   三组都带 --no-verify：逐字节校验是测试开销，三组量级相当但无意义，
#   放进来只会把差异摊平。正确性由每组开头单独跑一次带校验的把关。
#
# 用法: ./run_ab3.sh [每组跑几次，默认 5] ["数据集列表"]
#   ./run_ab3.sh 5
#   ./run_ab3.sh 3 "10MB-insert-4KB 10MB-cut-4KB"

OLD=/mnt/nvme1/dataset/10M/10MB
DIR=/mnt/nvme0/dataset/10M
N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
RES=/tmp/ab3_result.txt
OUTDIR=/mnt/nvme1/sync
CEMU=/root/CEMU/tests/cemu

mkdir -p "$OUTDIR"

f1()  { grep -oE '[0-9]+\.[0-9]+' | head -1; }
add() { awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{printf "%.1f", a+b}'; }
div() { awk -v a="${1:-0}" -v b="${2:-0}" \
            'BEGIN{ if(b+0>0 && a!="NA" && b!="NA") printf "%.2f", a/b; else print "NA" }'; }
stats() { sort -n | awk '{a[NR]=$1} END{
            n=NR; if(n==0){print "NA NA NA"; exit}
            med = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2;
            printf "%.1f %.1f %.1f\n", med, a[1], a[n] }'; }

: > "$RES"
: > /tmp/ab3_sum.txt

if [ ! -x /root/dsync_time_test ]; then
    echo "警告: /root/dsync_time_test 不存在，[A] 组将全部记 NA" | tee -a "$RES"
fi

for ds in $DATASETS; do
    NEW=$DIR/$ds
    NEW_SZ=$(stat -c %s "$NEW" 2>/dev/null)

    echo "==================== $ds ====================" | tee -a "$RES"

    # ---------- 正确性把关：每组跑一次带逐字节校验的 ----------
    echo "  [正确性] 带逐字节校验各跑一次" | tee -a "$RES"
    if [ -x /root/dsync_time_test ]; then
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$OLD" "$NEW" 0 \
            >/tmp/o_a.txt 2>&1
        rc_a=$(grep -oE 'reconstructed=[0-9]+' /tmp/o_a.txt | grep -oE '[0-9]+')
        if [ -n "$rc_a" ] && [ -n "$NEW_SZ" ] && [ "$rc_a" = "$NEW_SZ" ]; then
            echo "    [A] 重建 $rc_a B == f' 大小，OK" | tee -a "$RES"
        else
            echo "    [A] 重建字节数 '${rc_a:-空}' != f' 大小 ${NEW_SZ:-?}，请检查" | tee -a "$RES"
        fi
    fi
    for grp in b c; do
        if [ "$grp" = b ]; then bin=dsync; desc="grid-no-cdc"; else bin=gridds; desc="grid-cdc"; fi
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        ( cd "$CEMU" && ./build/$bin "$OLD" "$NEW" -c 0 -s 1 \
              -o "$OUTDIR/ab_${grp}.out" ) >/tmp/o_${grp}.txt 2>&1
        if grep -q '\[PASS\]' /tmp/o_${grp}.txt; then
            echo "    [${grp^^}] $desc : PASS 逐字节一致" | tee -a "$RES"
        else
            echo "    [${grp^^}] $desc : **FAIL** 结果不一致，后续时间仅供参考" | tee -a "$RES"
        fi
        # 命中率 / 跨界字节只取一次（每次跑都一样）
        eval "HIT_${grp}=\"$(grep -oE '命中率: .*' /tmp/o_${grp}.txt | tail -1)\""
        eval "XFER_${grp}=\"$(grep -oE '合计 +[0-9]+ B' /tmp/o_${grp}.txt | tail -1)\""
    done

    # ---------- 计时：N 轮，run1 当预热丢弃 ----------
    : > /tmp/v_a; : > /tmp/v_b; : > /tmp/v_c
    FAIL_A=0; FAIL_B=0; FAIL_C=0

    echo "  [计时] 每组 $N 轮（run1 预热丢弃），每轮前 drop_caches" | tee -a "$RES"
    for i in $(seq 1 "$N"); do
        # ---- [A] host-rsync ----
        if [ -x /root/dsync_time_test ]; then
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$OLD" "$NEW" 0 \
                >/tmp/o_a.txt 2>&1
            four=$(grep '四项合计'      /tmp/o_a.txt | f1)
            pat=$( grep 'Server Patch:' /tmp/o_a.txt | f1)
            if [ -z "$four" ]; then
                FAIL_A=$((FAIL_A+1)); printf "    run%-2s [A] FAILED\n" "$i" | tee -a "$RES"
            else
                tot=$(add "$four" "$pat")
                printf "    run%-2s [A] 四项=%-9s Patch=%-8s 总=%s ms\n" \
                       "$i" "$four" "$pat" "$tot" | tee -a "$RES"
                [ "$i" -gt 1 ] && echo "$tot" >> /tmp/v_a
            fi
        fi

        # ---- [B] grid-no-cdc ----
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        ( cd "$CEMU" && ./build/dsync "$OLD" "$NEW" -c 0 -s 1 --no-verify \
              -o "$OUTDIR/ab_b.out" ) >/tmp/o_b.txt 2>&1
        tot=$(grep '程序总耗时' /tmp/o_b.txt | f1)
        if [ -z "$tot" ]; then
            FAIL_B=$((FAIL_B+1)); printf "    run%-2s [B] FAILED\n" "$i" | tee -a "$RES"
        else
            printf "    run%-2s [B] 总=%s ms\n" "$i" "$tot" | tee -a "$RES"
            [ "$i" -gt 1 ] && echo "$tot" >> /tmp/v_b
        fi

        # ---- [C] grid-cdc ----
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        ( cd "$CEMU" && ./build/gridds "$OLD" "$NEW" -c 0 -s 1 --no-verify \
              -o "$OUTDIR/ab_c.out" ) >/tmp/o_c.txt 2>&1
        tot=$(grep '程序总耗时' /tmp/o_c.txt | f1)
        if [ -z "$tot" ]; then
            FAIL_C=$((FAIL_C+1)); printf "    run%-2s [C] FAILED\n" "$i" | tee -a "$RES"
        else
            printf "    run%-2s [C] 总=%s ms\n" "$i" "$tot" | tee -a "$RES"
            [ "$i" -gt 1 ] && echo "$tot" >> /tmp/v_c
        fi
    done

    # ---------- 统计（run2..N） ----------
    as=$(stats < /tmp/v_a); bs=$(stats < /tmp/v_b); cs=$(stats < /tmp/v_c)
    am=$(echo $as | cut -d' ' -f1); bm=$(echo $bs | cut -d' ' -f1); cm=$(echo $cs | cut -d' ' -f1)

    echo "  ---- run2..$N 中位 / min / max ----" | tee -a "$RES"
    line() { # $1=标签 $2="med min max"
        echo "    $1 中位 $(echo $2 | cut -d' ' -f1) ms" \
             " (min $(echo $2 | cut -d' ' -f2) / max $(echo $2 | cut -d' ' -f3))" \
             " 失败 $3" | tee -a "$RES"
    }
    line "[A] host-rsync :" "$as" "$FAIL_A"
    line "[B] grid-no-cdc:" "$bs" "$FAIL_B"
    line "[C] grid-cdc   :" "$cs" "$FAIL_C"
    echo "    加速比 A/B (下沉)         : $(div "$am" "$bm")x" | tee -a "$RES"
    echo "    加速比 A/C (下沉+CDC)     : $(div "$am" "$cm")x" | tee -a "$RES"
    echo "    加速比 B/C (server 做 CDC): $(div "$bm" "$cm")x" | tee -a "$RES"
    [ -n "$HIT_b"  ] && echo "    [B] ${HIT_b}"   | tee -a "$RES"
    [ -n "$XFER_b" ] && echo "    [B] 跨界 ${XFER_b}" | tee -a "$RES"
    [ -n "$HIT_c"  ] && echo "    [C] ${HIT_c}"   | tee -a "$RES"
    [ -n "$XFER_c" ] && echo "    [C] 跨界 ${XFER_c}" | tee -a "$RES"

    # 汇总行，方便最后拼总表
    echo "$ds|$am|$bm|$cm|$(div "$am" "$bm")|$(div "$am" "$cm")" >> /tmp/ab3_sum.txt
    echo "" | tee -a "$RES"
done

# ---------- 跨数据集总表 ----------
if [ -s /tmp/ab3_sum.txt ]; then
    echo "==================== 总表（中位 ms） ====================" | tee -a "$RES"
    printf "%-18s %10s %10s %10s %8s %8s\n" \
           "数据集" "A主机" "B网格" "C网格CDC" "A/B" "A/C" | tee -a "$RES"
    while IFS='|' read -r d a b c ab ac; do
        printf "%-18s %10s %10s %10s %8s %8s\n" "$d" "$a" "$b" "$c" "${ab}x" "${ac}x" | tee -a "$RES"
    done < /tmp/ab3_sum.txt
    echo "" | tee -a "$RES"
fi

echo "完整结果: $RES"
