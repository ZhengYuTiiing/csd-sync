#!/bin/bash
# 六方案端到端文件同步时间对比
#   rsync      : 传统 rsync（delta 传输）
#   dsync      : A 组，主机侧串行 CDC + patch（/root/dsync_time_test）
#   skysync-f  : skysync fast 路径（/root/skysync_f_time_test）
#   skysync-c  : skysync cold 路径（/root/skysync_c_time_test）
#   grid-cdc   : CEMU 设备内网格 CDC + 内容匹配（build/dsync2）
#   pipeline   : CEMU 设备内流水线 CDC（build/dsync3）
#
# 口径：外部挂钟（date +%s%N），含各自全部的读/算/写；
#       CEMU 两版加 --no-verify 跳过测试用逐字节校验。
# 每轮: 清理产物 -> 丢弃页缓存 -> 计时。
#
# 用法: ./bench_sync.sh [数据集组: 10M|100M|all] [每轮次数，默认 5]

SET=${1:-10M}
N=${2:-5}
OUT=/mnt/nvme1/sync
mkdir -p "$OUT"

case "$SET" in
  10M)
    OLD=/mnt/nvme1/dataset/10M/10MB
    DIR=/mnt/nvme0/dataset/10M
    DATASETS="10MB.new 10MB-insert-2KB 10MB-cut-256B 10MB-inverse-1MB"
    ;;
  100M)
    OLD=/mnt/nvme1/dataset/100M/100MB
    DIR=/mnt/nvme0/dataset/100M
    DATASETS="100MB-insert-8MB"
    ;;
  *)
    echo "未知数据集组: $SET（10M | 100M）"
    exit 1
    ;;
esac

IMPLS="rsync dsync skysync-f skysync-c grid-cdc pipeline"
MED=/tmp/bench_med.$$
: > "$MED"

median() { sort -n | awk '{a[NR]=$1} END{
    if (NR==0) { print "NA"; exit }
    if (NR%2) printf "%.0f\n", a[(NR+1)/2];
    else printf "%.0f\n", (a[NR/2]+a[NR/2+1])/2 }'; }

prep_impl() {
    local impl=$1 old=$2
    case $impl in
      rsync)
        rm -f "$OUT/rsync_dst"
        cp "$old" "$OUT/rsync_dst" || return 1
        ;;
    esac
    return 0
}

run_impl() {
    local impl=$1 old=$2 new=$3
    case $impl in
      rsync)
        /root/rsync_bin -a -I --no-whole-file "$new" "$OUT/rsync_dst"
        ;;
      dsync)
        LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$old" "$new" 0
        ;;
      skysync-f)
        LD_LIBRARY_PATH=/root/libs /root/skysync_f_time_test "$old" "$new"
        ;;
      skysync-c)
        LD_LIBRARY_PATH=/root/libs /root/skysync_c_time_test "$old" "$new"
        ;;
      grid-cdc)
        ( cd /root/CEMU/tests/cemu && ./build/dsync2 "$old" "$new" -c 0 -s 1 \
              --no-verify -o "$OUT/grid_cdc.out" )
        ;;
      pipeline)
        ( cd /root/CEMU/tests/cemu && ./build/dsync3 "$old" "$new" -c 0 -s 1 \
              --no-verify -o "$OUT/pipe.out" )
        ;;
    esac
}

echo "==================== 数据集组: $SET  每方案 $N 轮 ===================="

for ds in $DATASETS; do
    NEW=$DIR/$ds
    [ -f "$NEW" ] || { echo "缺文件 $NEW"; continue; }
    echo ""
    echo "---- $ds ----"

    for impl in $IMPLS; do
        vals=""
        last_ms=""
        for i in $(seq 1 "$N"); do
            # 清理上轮产物，避免"已存在"改变行为
            rm -f "$OLD.patch" "$NEW.patch" "$NEW.rebuilt" "$OUT"/*.out
            prep_impl "$impl" "$OLD" || { echo "  准备失败: $impl"; break; }

            sync
            echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

            t0=$(date +%s%N)
            run_impl "$impl" "$OLD" "$NEW" >"/tmp/bench_${impl}_${ds}.log" 2>&1
            rc=$?
            t1=$(date +%s%N)
            ms=$(( (t1 - t0) / 1000000 ))

            if [ "$rc" -ne 0 ]; then
                printf "  %-10s run%-2s %6s ms  rc=%s  <== 失败\n" "$impl" "$i" "$ms" "$rc"
                last_ms=""
                break
            fi
            printf "  %-10s run%-2s %6s ms\n" "$impl" "$i" "$ms"
            last_ms="$ms"
            # 丢弃第一轮（冷启动含未缓存元数据）；N=1 时退回唯一一轮
            [ "$i" -gt 1 ] && vals="$vals $ms"
        done

        [ -z "$vals" ] && vals="$last_ms"
        if [ -n "$vals" ]; then
            med=$(echo "$vals" | tr ' ' '\n' | median)
        else
            med="NA"
        fi
        echo "$ds $impl $med" >> "$MED"
    done
done

echo ""
echo "==================== 中位数汇总（ms，run2..N） ===================="
awk -v SET="$SET" '
    { key=$1; impl=$2; v=$3;
      if (!(key in seen)) { order[++n]=key; seen[key]=1 }
      val[key, impl]=v; impls[impl]=1 }
    END {
      printf "%-22s", "数据集"
      split("rsync dsync skysync-f skysync-c grid-cdc pipeline", I, " ")
      for (i=1; i<=6; i++) printf "%12s", I[i]
      printf "\n"
      for (k=1; k<=n; k++) {
        printf "%-22s", order[k]
        for (i=1; i<=6; i++) {
          v = val[order[k], I[i]]
          printf "%12s", (v=="" ? "-" : v)
        }
        printf "\n"
      }
    }' "$MED"

rm -f "$MED"
