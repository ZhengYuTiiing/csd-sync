#!/bin/bash
# run_grid7.sh —— 七方案端到端对比
#
#   组   方案                          二进制              下沉内容
#   [B]  dsync-CSD                     build/dsync        定长块 + 设备读数据算 CRC + 网格折叠
#   [E]  rsync-CSD                     build/rsync_csd    定长块 + 设备现场算指纹 + 设备内 patch
#   [F]  skysync-f-CSD                 build/sync_file    定长块 + server 零数据读（OOB 指纹）
#   [G]  skysync-c-CSD                 build/skysync_c_csd CDC + 弱哈希取自存储层网格 + 网格折叠
#   [C]  grid-cdc                      build/gridds       CDC + 网格折叠弱哈希；重建在主机侧
#   [H]  grid-cdc+pipeline             build/gridpipe -p P 同 C，但每条命令 P 个 chunk 并发
#   [I]  grid-cdc+pipeline+重建下沉    build/gridrb  -p P 重建下沉到设备，且重建命令 P 个 chunk 并发
#
# C/H/I 三者的关系（唯一变量是「流水线」和「重建在哪做」）：
#   C : 流水线 off，重建在主机
#   H : 流水线 on ，重建在主机
#   I : 流水线 on ，重建在设备
# 注意缺一格：「重建下沉 + 流水线 off」就是旧脚本里的 D（build/gridrb -p 1）。
# 要隔离「流水线」和「重建下沉」各自的净收益，把 d 加进 GSET 即可：
#   GSET="c d h i" ./run_grid7.sh 5 "10MB-insert-4KB"
#   则 C/H 之比 = 流水线净收益，D/I 之比 = 流水线在重建下沉基础上的净收益，
#   C/D 之比 = 重建下沉的净收益。
#
# 关于 I 的流水线范围：目前 -p 只作用于「设备内重建」那条命令。
#   CDC / 弱哈希比对 / 补算强哈希 这几条命令仍是 P=1。
#   （设备侧按 chunk 偏移 mr_addr，grid_rb 只用 mr[0]/mr[1]，所以算子本身不用改；
#     要把 P 也铺到 CDC 各阶段，需要把 gridpipe.cpp 的切窗逻辑移植进 gridrb.cpp。）
#
# 计时口径（统一）：外部挂钟（date +%s%N）包住整个进程。
#   之所以不用各程序自报的“程序总耗时”：E/F 根本不打印它，七组无法共用。
#
# 正确性判定（统一）：sync + drop_caches 之后用 cmp 与 f' 逐字节比对。
#
# 【重要】drop_caches 只能写 1，不能写 3。
#   echo 2/3 会连 inode/dentry 一起回收，fdmfs 目录项没有 evict_inode，
#   槽位会整个消失而设备侧内存不释放 —— 结果是 fallocate 报 Cannot allocate
#   memory、程序挂在 D 状态。所有“槽位莫名消失/设备被楔死”的根因都是这一句。
#
# 用法:
#   ./run_grid7.sh [每组跑几次，默认 5] ["数据集列表"]
#   ./run_grid7.sh 5
#   ./run_grid7.sh 3 "10MB-insert-4KB"
#   PIPE_P=8 ./run_grid7.sh 3 "10MB-inverse-1MB"
#   OLD=/mnt/nvme1/dataset/100M/100MB DIR=/mnt/nvme0/dataset/100M \
#       ./run_grid7.sh 3 "100MB-insert-8MB"
#   SKIP="f" 跳过指定组（F 历史上会把 fdmfs 楔死）

OLD=${OLD:-/mnt/nvme1/dataset/10M/10MB}
DIR=${DIR:-/mnt/nvme0/dataset/10M}
N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
PIPE_P=${PIPE_P:-4}          # H/I 的 chunk 并发度
RES=${RES:-/tmp/grid7_result.txt}
SUM=${SUM:-/tmp/grid7_sum.txt}

OUTDIR=/mnt/nvme1/sync       # C/H/I 的输出（I 必须落 nvme1：重建是设备写回自身闪存）
# E/F 把 FDM 硬编码成 /mnt/fdm0（设备 0），CSF 只在设备 0 上跑，
# 所以它们的老版本副本和目标文件都必须落在 nvme0。
OUTDIR0=/mnt/nvme0/sync
CEMU=/root/CEMU/tests/cemu
FCOPY=/mnt/nvme0/sync/f_copy
GSET=${GSET:-"b e f g c h i"}

if [ -n "$SKIP" ]; then
    _g=""
    for x in $GSET; do
        case " $SKIP " in *" $x "*) continue ;; esac
        _g="$_g $x"
    done
    GSET="$_g"
    echo "跳过组: $SKIP （实际跑: $GSET ）"
fi

mkdir -p "$OUTDIR" "$OUTDIR0"

div() { awk -v a="${1:-0}" -v b="${2:-0}" \
            'BEGIN{ if(b+0>0 && a!="NA" && b!="NA") printf "%.2f", a/b; else print "NA" }'; }
med() { sort -n | awk '{a[NR]=$1} END{
            n=NR; if(n==0){print "NA"; exit}
            m = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2;
            printf "%.0f\n", m }'; }

desc_of() {
    case $1 in
        b) echo "dsync-CSD     " ;;
        e) echo "rsync-CSD     " ;;
        f) echo "skysync-f-CSD " ;;
        g) echo "skysync-c-CSD " ;;
        c) echo "grid-cdc      " ;;
        d) echo "grid-cdc+下沉 " ;;
        h) echo "grid-cdc+流水 " ;;
        i) echo "grid+流水+下沉" ;;
    esac
}

out_of() {
    case $1 in
        # d/i 的输出按数据集命名并长期保留，避免重复付 extent 写实的成本
        d) echo "$OUTDIR/g7_d_${CUR_DS}.out" ;;
        i) echo "$OUTDIR/g7_i_${CUR_DS}.out" ;;
        e) echo "$OUTDIR0/g7_e.out" ;;
        f) echo "$OUTDIR0/g7_f.out" ;;
        b) echo "$OUTDIR/g7_b.out" ;;
        c) echo "$OUTDIR/g7_c.out" ;;
        g) echo "$OUTDIR/g7_g.out" ;;
        h) echo "$OUTDIR/g7_h.out" ;;
        *) echo "" ;;
    esac
}

bin_ok() {
    case $1 in
        b) [ -x "$CEMU/build/dsync" ] ;;
        e) [ -x "$CEMU/build/rsync_csd" ] ;;
        f) [ -x "$CEMU/build/sync_file" ] ;;
        g) [ -x "$CEMU/build/skysync_c_csd" ] ;;
        c) [ -x "$CEMU/build/gridds" ] ;;
        d) [ -x "$CEMU/build/gridrb" ] ;;
        h) [ -x "$CEMU/build/gridpipe" ] ;;
        i) [ -x "$CEMU/build/gridrb" ] ;;
    esac
}

# FDM 存活门禁：设备卡死后程序不报错、只会一直等，跑出来的数全是假的。
# 用 timeout 包住，免得探测自己挂住。
fdm_alive() { timeout 5 ls -1 /mnt/fdm0/ >/dev/null 2>&1; }

prep_group() {
    local g=$1
    case $g in
      e|f)
        cp "$OLD" "$FCOPY"          # 老版本放在 device0（这两个工具硬编码 dev0）
        cp "$OLD" "$(out_of $g)"    # 目标先放着老版本，patch 原地更新
        sync                        # 必须落盘：脏页会在设备写完之后回写，覆盖结果
        ;;
      d|i)
        # 重建下沉的两个组：输出文件的 extent 必须先“写实”，
        # 否则设备绕过文件系统直写 LBA 后，ext4 读回全是零。
        local o; o=$(out_of $g)
        if [ ! -f "$o" ] || [ "$(stat -c %s "$o" 2>/dev/null)" != "$NEW_SZ" ]; then
            rm -f "$o"
            ( cd "$CEMU" && ./build/gridrb "$OLD" "$NEW" -c 0 -s 1 --no-verify \
                  -o "$o" ) >/dev/null 2>&1
            sync
        fi
        ;;
    esac
}

run_group() {
    local g=$1
    case $g in
      b) ( cd "$CEMU" && ./build/dsync "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of b)" ) ;;
      e) ( cd "$CEMU" && ./build/rsync_csd "$FCOPY" "$NEW" "$(out_of e)" ) ;;
      f) ( cd "$CEMU" && ./build/sync_file "$FCOPY" "$NEW" "$(out_of f)" ) ;;
      g) ( cd "$CEMU" && ./build/skysync_c_csd "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of g)" ) ;;
      c) ( cd "$CEMU" && ./build/gridds "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of c)" ) ;;
      d) ( cd "$CEMU" && ./build/gridrb  "$OLD" "$NEW" -c 0 -s 1 --no-verify -p 1      -o "$(out_of d)" ) ;;
      h) ( cd "$CEMU" && ./build/gridpipe "$OLD" "$NEW" -c 0 -s 1 --no-verify -p "$PIPE_P" -o "$(out_of h)" ) ;;
      i) ( cd "$CEMU" && ./build/gridrb  "$OLD" "$NEW" -c 0 -s 1 --no-verify -p "$PIPE_P" -o "$(out_of i)" ) ;;
    esac
}

verify_group() {
    local g=$1
    local o; o=$(out_of "$g")
    [ -f "$o" ] || { echo FAIL; return; }
    cmp -s "$o" "$NEW" && echo OK || echo FAIL
}

: > "$RES"
: > "$SUM"

for g in $GSET; do
    if ! bin_ok "$g"; then
        echo "警告: [$g] 的可执行文件不存在，该组记 NA" | tee -a "$RES"
    fi
done

echo "并发度 PIPE_P=$PIPE_P （H/I 共用）" | tee -a "$RES"
echo "组: $GSET" | tee -a "$RES"

for ds in $DATASETS; do
    CUR_DS=$ds
    NEW=$DIR/$ds
    NEW_SZ=$(stat -c %s "$NEW" 2>/dev/null)

    echo "==================== $ds ====================" | tee -a "$RES"
    if [ -z "$NEW_SZ" ]; then
        echo "  跳过：$NEW 不存在" | tee -a "$RES"
        continue
    fi

    if ! fdm_alive; then
        echo "  [中止] fdmfs 无响应（设备已被卡死），后续数据集不再测" | tee -a "$RES"
        break
    fi

    # ---------- 正确性把关 ----------
    echo "  [正确性] 丢缓存后 cmp 与 f' 逐字节比对" | tee -a "$RES"
    declare -A STAT
    for g in $GSET; do
        if ! bin_ok "$g"; then STAT[$g]=NA; continue; fi
        if ! fdm_alive; then
            echo "  [中止] 跑 [$g] 前发现 fdmfs 已死，本数据集后续组记 NA" | tee -a "$RES"
            STAT[$g]=NA; DEAD=1; continue
        fi
        prep_group "$g"
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        run_group "$g" >"/tmp/g7_${g}_${ds}.log" 2>&1
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        STAT[$g]=$(verify_group "$g")
        printf "    [%s] %s : %s\n" "${g^^}" "$(desc_of $g)" "${STAT[$g]}" | tee -a "$RES"
    done

    # ---------- 计时 ----------
    for g in $GSET; do : > /tmp/g7_v_$g; done
    declare -A FAIL
    for g in $GSET; do FAIL[$g]=0; done

    echo "  [计时] 每组 $N 轮（run1 预热丢弃），每轮前 drop_caches；外部挂钟" \
        | tee -a "$RES"
    for i in $(seq 1 "$N"); do
        for g in $GSET; do
            if ! bin_ok "$g"; then continue; fi
            if ! fdm_alive; then
                printf "    run%-2s [%s] 设备已死，跳过\n" "$i" "${g^^}" | tee -a "$RES"
                FAIL[$g]=$(( ${FAIL[$g]} + 1 )); DEAD=1; continue
            fi
            prep_group "$g"
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            t0=$(date +%s%N)
            run_group "$g" >"/tmp/g7_${g}_run.txt" 2>&1
            t1=$(date +%s%N)
            ms=$(( (t1 - t0) / 1000000 ))
            if [ ! -s "/tmp/g7_${g}_run.txt" ]; then
                FAIL[$g]=$(( ${FAIL[$g]} + 1 ))
                printf "    run%-2s [%s] FAILED\n" "$i" "${g^^}" | tee -a "$RES"
            else
                printf "    run%-2s [%s] %6s ms\n" "$i" "${g^^}" "$ms" | tee -a "$RES"
                [ "$i" -gt 1 ] && echo "$ms" >> /tmp/g7_v_$g
            fi
        done
        [ -n "$DEAD" ] && break
    done

    # ---------- 统计 ----------
    echo "  ---- run2..$N 中位（外部挂钟, ms）----" | tee -a "$RES"
    declare -A MED
    for g in $GSET; do
        if ! bin_ok "$g"; then MED[$g]=NA; else MED[$g]=$(med < /tmp/g7_v_$g); fi
        printf "    [%s] %s : 中位 %s ms   正确性 %s   失败 %s\n" \
               "${g^^}" "$(desc_of $g)" "${MED[$g]}" "${STAT[$g]:-NA}" "${FAIL[$g]}" \
               | tee -a "$RES"
    done

    echo "    加速比 B/C  (dsync-CSD / grid-cdc)        : $(div "${MED[b]}" "${MED[c]}")x" | tee -a "$RES"
    echo "    加速比 B/G  (dsync-CSD / skysync-c-CSD)   : $(div "${MED[b]}" "${MED[g]}")x" | tee -a "$RES"
    echo "    加速比 E/F  (rsync-CSD / skysync-f-CSD)   : $(div "${MED[e]}" "${MED[f]}")x" | tee -a "$RES"
    echo "    加速比 C/H  (流水线净收益, 主机重建)      : $(div "${MED[c]}" "${MED[h]}")x" | tee -a "$RES"
    echo "    加速比 C/I  (流水线+重建下沉, 合计)       : $(div "${MED[c]}" "${MED[i]}")x" | tee -a "$RES"
    echo "    加速比 H/I  (重建下沉净收益, 开流水线后)  : $(div "${MED[h]}" "${MED[i]}")x" | tee -a "$RES"
    if [ -n "${MED[d]}" ] && [ "${MED[d]}" != NA ]; then
        echo "    加速比 C/D  (重建下沉净收益, 无流水线)   : $(div "${MED[c]}" "${MED[d]}")x" | tee -a "$RES"
        echo "    加速比 D/I  (流水线净收益, 重建下沉后)   : $(div "${MED[d]}" "${MED[i]}")x" | tee -a "$RES"
    fi

    for g in $GSET; do
        h=$(grep -oE '命中率: .*' "/tmp/g7_${g}_${ds}.log" | tail -1)
        [ -n "$h" ] && echo "    [${g^^}] $h" | tee -a "$RES"
    done

    echo "$ds|${MED[b]}|${MED[e]}|${MED[f]}|${MED[g]}|${MED[c]}|${MED[h]}|${MED[i]}|${MED[d]}|${STAT[c]}|${STAT[h]}|${STAT[i]}" >> "$SUM"
    echo "" | tee -a "$RES"
    unset MED STAT FAIL DEAD
done

# ---------- 总表 ----------
if [ -s "$SUM" ]; then
    echo "==================== 总表（外部挂钟中位, ms, P=$PIPE_P） ====================" | tee -a "$RES"
    printf "%-18s %8s %8s %8s %8s %8s %8s %8s %8s\n" \
           "数据集" "B" "E" "F" "G" "C" "H" "I" "D" | tee -a "$RES"
    printf "%-18s %8s %8s %8s %8s %8s %8s %8s %8s\n" \
           "" "dsync" "rsync" "sky-f" "sky-c" "grid" "+流水" "+下沉" "仅下沉" | tee -a "$RES"
    while IFS='|' read -r d b e f g c hh i dd sc sh si; do
        printf "%-18s %8s %8s %8s %8s %8s %8s %8s %8s\n" \
               "$d" "$b" "$e" "$f" "$g" "$c" "$hh" "$i" "$dd" | tee -a "$RES"
        printf "  %-16s C=%s H=%s I=%s  （正确性 OK/FAIL）\n" "" "$sc" "$sh" "$si" | tee -a "$RES"
    done < "$SUM"
    echo "" | tee -a "$RES"
fi

echo "完整结果: $RES"
