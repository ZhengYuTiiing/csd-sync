#!/bin/bash
# 六向时间总对比
#
#   [1] rsync      : 主机, 不在 CSD（经典基线）
#   [2] dsync      : 主机, 不在 CSD（现场算校验和, algo=0）
#   [3] skysync-c  : 主机, 不在 CSD
#   [4] skysync-f  : 主机, 不在 CSD（SOTA）
#   [5] grid-no-cdc: 在 CSD, 网格折叠; server 不做 CDC
#   [6] grid-cdc   : 在 CSD, 网格折叠 + 两侧各自 CDC 后内容寻址
#
# 主口径 = 阶段合计:
#   主机四组 = 各自报告的「四项合计 + Server Patch」
#   CSD 两组 = 「上表合计」（含重建，不含逐字节校验 —— 本方额外 QA，主机四组没有）
#
# 前提说明（两边对称，故可比）:
#   skysync-c / skysync-f 假设存储层在写入时维护好 FSC，其 calc_fsc_hw 未计入；
#   我方假设设备在写入时维护好 512B OOB 网格，其生成成本也未计入。
#
# 用法: ./run_ab6.sh [每组跑几次，默认 15]

OLD=/mnt/nvme1/dataset/10M/10MB
DIR=/mnt/nvme0/dataset/10M
N=${1:-15}
RES=/tmp/ab6_result.txt
LIBS=/root/libs
CEMU=/root/CEMU/tests/cemu
OUTDIR=/mnt/nvme1/sync

DATASETS="10MB.new 10MB-insert-2KB 10MB-cut-256B 10MB-inverse-1MB"
HOSTS="rsync:rsync_time_test dsync:dsync_time_test skysync-c:skysync_c_time_test skysync-f:skysync_f_time_test"

mkdir -p "$OUTDIR"

f1() { grep -oE '[0-9]+\.[0-9]+' | head -1; }
x1000() { awk -v p="${1:-0}" 'BEGIN{printf "%.1f", p*1000}'; }
add2() { awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{printf "%.1f", a+b}'; }
stat5() {
    sort -n | awk '{a[NR]=$1; s+=$1} END{
        n=NR; if(n==0){print "NA NA NA NA NA"; exit}
        m=(n%2)?a[(n+1)/2]:(a[n/2]+a[n/2+1])/2;
        mean=s/n;
        for(i=1;i<=n;i++){d=a[i]-mean; v+=d*d}
        printf "%.1f %.1f %.1f %.1f %.1f\n", m, a[1], a[n], mean, sqrt(v/n)}'
}
get() { echo "$1" | cut -d' ' -f"$2"; }

# 主机侧阶段合计 = 四项合计 + Patch（各程序 Patch 行格式/单位不同）
host_sum() {
    local bin=$1 four patch
    four=$(grep '四项合计' /tmp/o_ab.txt | f1)
    case "$bin" in
        skysync_f_time_test) patch=$(x1000 "$(grep 'Server Patch delta applied in' /tmp/o_ab.txt | f1)") ;;
        skysync_c_time_test) patch=$(x1000 "$(grep 'Server Patch Delta time,' /tmp/o_ab.txt | f1)") ;;
        *)                   patch=$(grep 'Server Patch:' /tmp/o_ab.txt | f1) ;;
    esac
    add2 "$four" "$patch"
}

: > "$RES"

for ds in $DATASETS; do
    NEW=$DIR/$ds
    echo "==================== $ds ====================" | tee -a "$RES"

    for pair in $HOSTS; do
        tag=${pair%%:*}; bin=${pair##*:}
        : > /tmp/ab_s; : > /tmp/ab_w
        for i in $(seq 1 "$N"); do
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            t0=$(date +%s%N)
            ( cd /root && LD_LIBRARY_PATH=$LIBS /root/$bin "$OLD" "$NEW" 0 ) \
                >/tmp/o_ab.txt 2>&1
            t1=$(date +%s%N)
            ms=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.1f", (b-a)/1000000}')
            sm=$(host_sum "$bin")
            if [ "$i" -gt 1 ]; then
                echo "$sm" >> /tmp/ab_s
                echo "$ms" >> /tmp/ab_w
            fi
        done
        s=$(stat5 < /tmp/ab_s); w=$(stat5 < /tmp/ab_w)
        printf "    %-11s 阶段合计 %8s ms (min %s / max %s, sd %s) | 墙钟 %8s ms\n" \
            "$tag" "$(get "$s" 1)" "$(get "$s" 2)" "$(get "$s" 3)" "$(get "$s" 5)" \
            "$(get "$w" 1)" | tee -a "$RES"
    done

    for g in b c; do
        if [ "$g" = c ]; then bin=dsync2; name=grid-cdc; else bin=dsync; name=grid-no-cdc; fi
        : > /tmp/ab_s; : > /tmp/ab_w; : > /tmp/ab_v
        for i in $(seq 1 "$N"); do
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            t0=$(date +%s%N)
            ( cd $CEMU && ./build/$bin "$OLD" "$NEW" -c 0 -s 1 -o "$OUTDIR/ab_$g.out" ) \
                >/tmp/o_ab.txt 2>&1
            t1=$(date +%s%N)
            ms=$(awk -v a="$t0" -v b="$t1" 'BEGIN{printf "%.1f", (b-a)/1000000}')
            sm=$(grep '上表合计' /tmp/o_ab.txt | f1)
            vr=$(grep '逐字节校验' /tmp/o_ab.txt | f1)
            if [ "$i" -gt 1 ]; then
                echo "$sm" >> /tmp/ab_s
                echo "$ms" >> /tmp/ab_w
                echo "$vr" >> /tmp/ab_v
            fi
        done
        s=$(stat5 < /tmp/ab_s); w=$(stat5 < /tmp/ab_w); v=$(stat5 < /tmp/ab_v)
        printf "    %-11s 阶段合计 %8s ms (min %s / max %s, sd %s) | 墙钟 %8s ms | 逐字节校验 %s ms(未含)\n" \
            "$name" "$(get "$s" 1)" "$(get "$s" 2)" "$(get "$s" 3)" "$(get "$s" 5)" \
            "$(get "$w" 1)" "$(get "$v" 1)" | tee -a "$RES"
    done
    echo "" | tee -a "$RES"
done

echo "完整结果: $RES" | tee -a "$RES"
