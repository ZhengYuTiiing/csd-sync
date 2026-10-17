#!/bin/bash
# run_ab4.sh —— 五个 CSD 实现的端到端同步时间对比
#
#   rsync-CSD     : build/rsync_csd      定长块 + 设备读数据现场算指纹 + 设备内 patch
#   dsync-CSD     : build/dsync          CDC + 设备读数据算 CRC 清单 + 网格折叠
#   skysync-f-CSD : build/sync_file      定长块 + server 零数据读（OOB 指纹）+ 设备内 patch
#   skysync-c-CSD : build/skysync_c_csd  CDC + 弱哈希取自存储层网格 + 网格折叠
#   grid-cdc      : build/gridds         CDC + 网格折叠弱哈希 + 弱命中后才算强哈希（单算子）
#
# 计时：外部挂钟（date +%s%N），每轮 rm 产物 + drop_caches；取 run2..N 的中位数。
# 用法: ./run_ab4.sh [10M|100M] [每组跑几次，默认 3]

SET=${1:-10M}
N=${2:-3}
RES=/tmp/ab4_result.txt
OUT=/mnt/nvme1/sync

OLD=/mnt/nvme1/dataset/$SET/10MB
DIR=/mnt/nvme0/dataset/$SET
if [ "$SET" = "10M" ]; then
    DATASETS="10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"
else
    DATASETS="100MB-insert-8MB"
    OLD=/mnt/nvme1/dataset/100M/100MB
fi

mkdir -p "$OUT"
: > "$RES"

median() { sort -n | awk '{a[NR]=$1} END{
    if (NR==0){print "NA"; exit}
    if (NR%2) printf "%.0f\n", a[(NR+1)/2];
    else printf "%.0f\n", (a[NR/2]+a[NR/2+1])/2 }'; }

run_impl() {
    local impl=$1 old=$2 new=$3
    case $impl in
      rsync-CSD)
        ( cd /root/CEMU/tests/cemu && ./build/rsync_csd /mnt/nvme0/sync/f_copy \
              "$new" "$OUT/rsync_csd.out" )
        ;;
      dsync-CSD)
        ( cd /root/CEMU/tests/cemu && ./build/dsync "$old" "$new" -c 0 -s 1 \
              --no-verify -o "$OUT/dsync_csd.out" )
        ;;
      skysync-f-CSD)
        ( cd /root/CEMU/tests/cemu && ./build/sync_file /mnt/nvme0/sync/f_copy \
              "$new" "$OUT/sf.out" )
        ;;
      skysync-c-CSD)
        ( cd /root/CEMU/tests/cemu && ./build/skysync_c_csd "$old" "$new" -c 0 -s 1 \
              --no-verify -o "$OUT/scc.out" )
        ;;
      grid-cdc)
        ( cd /root/CEMU/tests/cemu && ./build/gridds "$old" "$new" -c 0 -s 1 \
              --no-verify -o "$OUT/grid.out" )
        ;;
    esac
}

prep_impl() {
    # 单 CSD 实现要求 old 在 nvme0，且目标文件必须已存在（设备端 patch 原地写入
    # 既有 extent）：目标先放老版本，由 patch 更新成新版本（准备工作，不计入计时）
    rm -f /mnt/nvme0/sync/f_copy
    cp "$OLD" /mnt/nvme0/sync/f_copy
    case $1 in
      rsync-CSD)
        rm -f "$OUT/rsync_csd.out"
        cp "$OLD" "$OUT/rsync_csd.out"
        ;;
      skysync-f-CSD)
        rm -f "$OUT/sf.out"
        cp "$OLD" "$OUT/sf.out"
        ;;
    esac
    sync
}

for ds in $DATASETS; do
    NEW=$DIR/$ds
    echo "==================== $ds ====================" | tee -a "$RES"

    declare -A MED
    for impl in rsync-CSD dsync-CSD skysync-f-CSD skysync-c-CSD grid-cdc; do
        vals=""
        last_ms=""
        for i in $(seq 1 "$N"); do
            rm -f "$OUT"/*.out /mnt/nvme0/sync/target.bin
            prep_impl "$impl"
            sync; echo 3 >/proc/sys/vm/drop_caches 2>/dev/null
            t0=$(date +%s%N)
            run_impl "$impl" "$OLD" "$NEW" >"/tmp/ab4_${impl}_${ds}.log" 2>&1
            rc=$?
            t1=$(date +%s%N)
            ms=$(( (t1 - t0) / 1000000 ))
            printf "  %-14s run%-2s %6s ms  (rc=%s)\n" "$impl" "$i" "$ms" "$rc"
            last_ms="$ms"
            [ "$i" -gt 1 ] && vals="$vals $ms"
        done
        [ -z "$vals" ] && vals="$last_ms"
        MED[$impl]=$(echo "$vals" | tr ' ' '\n' | median)
    done
    unset MED_MED

    {
        printf "%-18s" "$ds"
        for impl in rsync-CSD dsync-CSD skysync-f-CSD skysync-c-CSD grid-cdc; do
            printf " %10s" "${MED[$impl]}"
        done
        echo
    } >> "$RES"
    unset MED
done

echo
echo "==== 中位数汇总（ms，列: rsync-CSD dsync-CSD skysync-f-CSD skysync-c-CSD grid-cdc）===="
cat "$RES"
