#!/bin/bash
# run_eg.sh —— 补测 E / G 两组（A/B/C/D 的数据由 run_abcd.sh 提供，不必重跑）
#
#   [E] rsync-CSD     : build/rsync_csd      定长块 + 设备读数据现场算指纹 + 设备内 patch
#   [G] skysync-c-CSD : build/skysync_c_csd  CDC + 弱哈希取自存储层网格 + 网格折叠
#
# 为什么单独一个脚本、而不是加进 run_abcd.sh：
#   run_abcd.sh 用内部计时（读“程序总耗时”），但 E 不打印这一行（只有 G 打印），
#   内部口径套不到 E 上。这里两组统一用外部挂钟（date +%s%N 包住整个进程），
#   保证 E 与 G 之间可比。
#   【跨脚本比较的口径差】外部挂钟比内部计时多约 200~300ms 固定启动开销
#   （dlopen 若干 CSF + 打开设备）。所以本脚本的 E/G 不能直接和 run_abcd.sh 的
#   A/B/C/D 相减，要么换算到同一口径，要么在表里注明。
#
# E 的设备约束（关键，踩过坑，别改回去）：
#   E 把 FDM 硬编码成 /mnt/fdm0，即设备 0，它的 CSF 只在设备 0 上跑。
#   因此 E 的老版本副本（f_copy）和目标文件都必须在 nvme0；放 nvme1 的话设备会
#   拿着 nvme1 的 LBA 去写 nvme0，目标文件原封不动 —— 表现为 cmp 的第一个差异
#   恰好等于老/新版本首次不同的位置，看起来像“错位 bug”，其实一个字节都没写。
#
# 用法:
#   ./run_eg.sh [每组跑几次，默认 5] ["数据集列表"]
#   ./run_eg.sh 5
#   ./run_eg.sh 3 "10MB-insert-4KB"
# 100MB:
#   OLD=/mnt/nvme1/dataset/100M/100MB DIR=/mnt/nvme0/dataset/100M \
#       ./run_eg.sh 3 "100MB-insert-8MB"

OLD=${OLD:-/mnt/nvme1/dataset/10M/10MB}
DIR=${DIR:-/mnt/nvme0/dataset/10M}
N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
RES=/tmp/eg_result.txt
SUM=/tmp/eg_sum.txt
CEMU=/root/CEMU/tests/cemu
FCOPY=/mnt/nvme0/sync/f_copy
OUT0=/mnt/nvme0/sync          # E 的目标必须在 nvme0
OUT1=/mnt/nvme1/sync          # G 无所谓，沿用 run_abcd.sh 的目录

mkdir -p "$OUT0" "$OUT1"

stats() { sort -n | awk '{a[NR]=$1} END{
            n=NR; if(n==0){print "NA NA NA"; exit}
            med = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2;
            printf "%.1f %.1f %.1f\n", med, a[1], a[n] }'; }

: > "$RES"
: > "$SUM"

for b in rsync_csd skysync_c_csd; do
    if [ ! -x "$CEMU/build/$b" ]; then
        echo "错误: $CEMU/build/$b 不存在，请先 make" | tee -a "$RES"
        exit 1
    fi
done

# E 的准备工作（不计入计时）：目标先放老版本，由设备原地 patch 成新版本
prep_e() {
    rm -f "$OUT0/eg_e.out"
    cp "$OLD" "$OUT0/eg_e.out"
    rm -f "$FCOPY"
    cp "$OLD" "$FCOPY"
    sync
}

for ds in $DATASETS; do
    NEW=$DIR/$ds
    NEW_SZ=$(stat -c %s "$NEW" 2>/dev/null)

    echo "==================== $ds ====================" | tee -a "$RES"

    if [ -z "$NEW_SZ" ]; then
        echo "  跳过：$NEW 不存在" | tee -a "$RES"
        continue
    fi

    # FDM 存活门禁：设备被卡死之后，程序不会报错、只会一直等，
    # 跑出来的数全是假的 —— 所以每个数据集开跑前先探一次。
    if ! timeout 5 ls -1 /mnt/fdm0/ >/dev/null 2>&1; then
        echo "  [中止] fdmfs 无响应（设备已被卡死），后续数据集不再测" | tee -a "$RES"
        break
    fi

    # ---------- 正确性把关 ----------
    echo "  [正确性] 丢缓存后 cmp 与 f' 逐字节比对" | tee -a "$RES"

    prep_e
    echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
    ( cd "$CEMU" && ./build/rsync_csd "$FCOPY" "$NEW" "$OUT0/eg_e.out" ) \
        >/tmp/eg_o_e.txt 2>&1
    sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
    if cmp -s "$OUT0/eg_e.out" "$NEW"; then
        echo "    [E] rsync-CSD     : cmp 一致" | tee -a "$RES"
    else
        echo "    [E] rsync-CSD     : **cmp 不一致**（后续时间仅供参考）" | tee -a "$RES"
    fi

    rm -f "$OUT1/eg_g.out"
    sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
    ( cd "$CEMU" && ./build/skysync_c_csd "$OLD" "$NEW" -c 0 -s 1 \
          -o "$OUT1/eg_g.out" ) >/tmp/eg_o_g.txt 2>&1
    if grep -q '\[PASS\]' /tmp/eg_o_g.txt; then
        echo "    [G] skysync-c-CSD : PASS 逐字节一致" | tee -a "$RES"
    else
        echo "    [G] skysync-c-CSD : **FAIL**" | tee -a "$RES"
    fi

    # ---------- 计时：N 轮，run1 当预热丢弃 ----------
    : > /tmp/eg_v_e; : > /tmp/eg_v_g
    echo "  [计时] 每组 $N 轮（run1 预热丢弃），每轮前 drop_caches" | tee -a "$RES"

    for i in $(seq 1 "$N"); do
        prep_e
        echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        t0=$(date +%s%N)
        ( cd "$CEMU" && ./build/rsync_csd "$FCOPY" "$NEW" "$OUT0/eg_e.out" ) \
            >/tmp/eg_o_e.txt 2>&1
        t1=$(date +%s%N)
        ms=$(( (t1 - t0) / 1000000 ))
        printf "    run%-2s [E] %6s ms\n" "$i" "$ms" | tee -a "$RES"
        [ "$i" -gt 1 ] && echo "$ms" >> /tmp/eg_v_e

        rm -f "$OUT1/eg_g.out"
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        t0=$(date +%s%N)
        ( cd "$CEMU" && ./build/skysync_c_csd "$OLD" "$NEW" -c 0 -s 1 --no-verify \
              -o "$OUT1/eg_g.out" ) >/tmp/eg_o_g.txt 2>&1
        t1=$(date +%s%N)
        ms=$(( (t1 - t0) / 1000000 ))
        printf "    run%-2s [G] %6s ms\n" "$i" "$ms" | tee -a "$RES"
        [ "$i" -gt 1 ] && echo "$ms" >> /tmp/eg_v_g
    done

    es=$(stats < /tmp/eg_v_e); gs=$(stats < /tmp/eg_v_g)
    em=$(echo $es | cut -d' ' -f1); gm=$(echo $gs | cut -d' ' -f1)

    echo "  ---- run2..$N 中位 / min / max ----" | tee -a "$RES"
    echo "    [E] rsync-CSD     : $em ms  (min $(echo $es | cut -d' ' -f2) / max $(echo $es | cut -d' ' -f3))" | tee -a "$RES"
    echo "    [G] skysync-c-CSD : $gm ms  (min $(echo $gs | cut -d' ' -f2) / max $(echo $gs | cut -d' ' -f3))" | tee -a "$RES"

    echo "$ds|$em|$gm" >> "$SUM"
    echo "" | tee -a "$RES"
done

if [ -s "$SUM" ]; then
    echo "==================== 总表（中位 ms，外部挂钟） ====================" | tee -a "$RES"
    printf "%-18s %14s %14s\n" "数据集" "E rsync-CSD" "G skysync-c" | tee -a "$RES"
    while IFS='|' read -r d e g; do
        printf "%-18s %14s %14s\n" "$d" "$e" "$g" | tee -a "$RES"
    done < "$SUM"
    echo "" | tee -a "$RES"
fi

echo "完整结果: $RES"
