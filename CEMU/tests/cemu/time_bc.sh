#!/bin/bash
# 只比 B(grid-no-cdc) 与 C(grid-cdc) 的端到端时间，高重复次数压制噪声。
# 输出: 中位 / 最小 / 最大 / 均值 / 标准差
OLD=/mnt/nvme1/dataset/10M/10MB
DIR=/mnt/nvme0/dataset/10M
N=${1:-15}
DATASETS="10MB.new 10MB-insert-2KB 10MB-cut-256B 10MB-inverse-1MB"

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
  for g in B C; do
    : > /tmp/tv
    for i in $(seq 1 "$N"); do
      sync
      echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
      if [ "$g" = "B" ]; then
        bin=/root/CEMU/tests/cemu/build/dsync
        out=/mnt/nvme1/sync/t_b.out
      else
        bin=/root/CEMU/tests/cemu/build/dsync2
        out=/mnt/nvme1/sync/t_c.out
      fi
      ms=$("$bin" "$OLD" "$DIR/$ds" -c 0 -s 1 -o "$out" 2>&1 \
           | sed -n 's/.*程序总耗时[^0-9]*\([0-9][0-9.]*\).*/\1/p' | tail -1)
      [ -n "$ms" ] && echo "$ms" >> /tmp/tv
    done
    echo "  [$g] $(stat5 < /tmp/tv)   n=$(wc -l < /tmp/tv)"
  done
done
