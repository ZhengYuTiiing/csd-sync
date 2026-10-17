#!/bin/bash
# dsync vs skysync-c 高重复次数对比
#
# 口径: 两者都用各自报告的「四项合计 + Server Patch」（生产口径）。
# 注意: skysync-c 的 calc_fsc_hw / write_fsc / read_fsc 未被计时（存储层提供 FSC 的前提），
#       这与 skysync-f 一致，是它的部署假设，不补回。
#       dsync 没有 FSC 可用，其校验和成本本来就含在自己的四项合计里。
#
# 用法: ./run_dc.sh [每组跑几次，默认 15]

OLD=/mnt/nvme1/dataset/10M/10MB
DIR=/mnt/nvme0/dataset/10M
N=${1:-15}
DATASETS="10MB.new 10MB-insert-2KB 10MB-cut-256B 10MB-inverse-1MB"

f1() { grep -oE '[0-9]+\.[0-9]+' | head -1; }
x1000() { awk -v p="${1:-0}" 'BEGIN{printf "%.1f", p*1000}'; }
add2() { awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{printf "%.1f", a+b}'; }
stat5() {
    sort -n | awk '{a[NR]=$1; s+=$1} END{
        n=NR; if(n==0){print "NA"; exit}
        m=(n%2)?a[(n+1)/2]:(a[n/2]+a[n/2+1])/2;
        mean=s/n;
        for(i=1;i<=n;i++){d=a[i]-mean; v+=d*d}
        printf "中位 %.1f | min %.1f | max %.1f | 均值 %.1f | sd %.1f", m, a[1], a[n], mean, sqrt(v/n)}'
}

for ds in $DATASETS; do
    echo "==================== $ds ===================="
    for b in dsync_time_test skysync_c_time_test; do
        : > /tmp/dcv
        for i in $(seq 1 "$N"); do
            sync; echo 3 >/proc/sys/vm/drop_caches 2>/dev/null
            ( cd /root && LD_LIBRARY_PATH=/root/libs /root/$b "$OLD" "$DIR/$ds" 0 ) \
                >/tmp/o_dc.txt 2>&1
            four=$(grep '四项合计' /tmp/o_dc.txt | f1)
            if [ "$b" = skysync_c_time_test ]; then
                patch=$(x1000 "$(grep 'Server Patch Delta time,' /tmp/o_dc.txt | f1)")
            else
                patch=$(grep 'Server Patch:' /tmp/o_dc.txt | f1)
            fi
            tot=$(add2 "$four" "$patch")
            [ -n "$tot" ] && [ "$i" -gt 1 ] && echo "$tot" >> /tmp/dcv
        done
        printf "  %-22s %s   n=%s\n" "$b" "$(stat5 < /tmp/dcv)" "$(wc -l < /tmp/dcv)"
    done
done
