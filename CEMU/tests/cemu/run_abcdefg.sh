#!/bin/bash
# run_abcdefg.sh —— A~G 七个实现端到端对比
#
#   [A] host-dsync    : 不在 CSD。数据搬到主机，CPU 现场算 CDC + 弱/强校验和
#                       /root/dsync_time_test（LD_LIBRARY_PATH=/root/libs）
#   [B] dsync-CSD     : skysync/dsync 下沉。定长块 + 设备读数据算 CRC + 网格折叠
#                       build/dsync
#   [C] grid-cdc      : CDC + 网格折叠弱哈希，弱命中后才算强哈希；重建在主机侧
#                       build/gridds
#   [D] grid-cdc+CSD  : 同 C，但重建下沉到 CSD（设备直接写回闪存）
#                       build/gridrb
#   [E] rsync-CSD     : skysync/rsync 下沉。定长块 + 设备现场算指纹 + 设备内 patch
#                       build/rsync_csd
#   [F] skysync-f-CSD : 定长块 + server 零数据读（OOB 指纹）+ 设备内 patch
#                       build/sync_file
#   [G] skysync-c-CSD : CDC + 弱哈希取自存储层网格 + 网格折叠
#                       build/skysync_c_csd
#
# 计时口径（统一）：外部挂钟（date +%s%N）包住整个进程。
#   之所以不用各程序自报的“程序总耗时”：E/F 根本不打印它，七组无法共用。
#   程序自报值仍会记录到日志里，供交叉验证。
#
# 正确性判定（统一）：sync + drop_caches 之后用 cmp 与 f' 逐字节比对。
#
# 【重要】drop_caches 只能写 1，不能写 3。
#   echo 1 = 只丢页缓存，正是计时需要的（避免上轮数据留在缓存里影响读数）。
#   echo 2/3 = 连 inode/dentry 一起回收，而 fdmfs 的目录项是纯内存结构、
#      没有 evict_inode，inode 一被回收，/mnt/fdm0/* 的槽位就整个消失，
#      设备侧内存还不释放 —— 结果是 fallocate 报 Cannot allocate memory、
#      程序挂在 D 状态（旧内核还会永久死锁在 fdmfs_lookup 的 iget_locked 上）。
#   之前所有"槽位莫名消失 / 设备被 F 楔死 / 程序挂死"的现象，根因都是这一句。
#   不用各程序自带的 [PASS]/[FAIL]——凡是用设备直写 LBA 的实现，自校验时
#   页缓存里可能还是旧内容，会给出假 FAIL（E/F 尤其明显）。
#
# 各组的准备动作（都不计入计时）：
#   E/F : 目标必须先放着老版本（设备内原地 patch），每轮 cp OLD -> tgt 后 sync
#   D   : 目标文件的 extent 必须先“写实”，否则设备直写后 ext4 读回全是零。
#         首次跑一次让它自己完成，之后复用同尺寸文件
#
# 用法:
#   ./run_abcdefg.sh [每组跑几次，默认 5] ["数据集列表"]
#   ./run_abcdefg.sh 5
#   ./run_abcdefg.sh 3 "10MB-insert-4KB"
#   OLD=/mnt/nvme1/dataset/100M/100MB DIR=/mnt/nvme0/dataset/100M \
#       ./run_abcdefg.sh 3 "100MB-insert-8MB"

OLD=${OLD:-/mnt/nvme1/dataset/10M/10MB}
DIR=${DIR:-/mnt/nvme0/dataset/10M}
N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
RES=/tmp/abcdefg_result.txt
SUM=/tmp/abcdefg_sum.txt
OUTDIR=/mnt/nvme1/sync
# E/F 把 FDM 硬编码成 /mnt/fdm0，也就是**设备 0**，它们的 CSF 只在设备 0 上跑。
# 因此 E/F 的老版本副本和目标文件都必须落在 nvme0，否则设备会拿着 nvme1 的
# LBA 去写 nvme0，目标文件原封不动 —— 表现是 cmp 的第一个差异恰好等于
# 老版本与新版本首次不同的位置（insert-4KB 场景就是 5MB），很像是错位 bug。
OUTDIR0=/mnt/nvme0/sync
CEMU=/root/CEMU/tests/cemu
FCOPY=/mnt/nvme0/sync/f_copy
# 注意：不能用 GROUPS 作变量名 —— 它是 bash 的特殊变量（当前用户的组 ID 列表），
# 赋值会被忽略，$GROUPS 会展开成 0，导致循环只跑一次。
GALL="a b c d e f g"

# SKIP="f" 可跳过指定组。
# F（sync_file）目前每次运行都会把 fdmfs 楔死（连 ls /mnt/fdm0 都挂住，
# 只能重启 VM），所以默认建议跳过它，否则一轮跑不完且拿不到可信数据。
if [ -n "$SKIP" ]; then
    _g=""
    for x in $GALL; do
        case " $SKIP " in *" $x "*) continue ;; esac
        _g="$_g $x"
    done
    GALL="$_g"
    echo "跳过组: $SKIP （实际跑: $GALL ）"
fi

mkdir -p "$OUTDIR" /mnt/nvme0/sync

div() { awk -v a="${1:-0}" -v b="${2:-0}" \
            'BEGIN{ if(b+0>0 && a!="NA" && b!="NA") printf "%.2f", a/b; else print "NA" }'; }
med() { sort -n | awk '{a[NR]=$1} END{
            n=NR; if(n==0){print "NA"; exit}
            m = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2;
            printf "%.0f\n", m }'; }

desc_of() {
    case $1 in
        a) echo "host-dsync   " ;;
        b) echo "dsync-CSD    " ;;
        c) echo "grid-cdc     " ;;
        d) echo "grid-cdc+CSD " ;;
        e) echo "rsync-CSD    " ;;
        f) echo "skysync-f-CSD" ;;
        g) echo "skysync-c-CSD" ;;
    esac
}

out_of() {
    # D 的输出按数据集命名并长期保留（避免重复付 extent 写实成本）
    case $1 in
        d) echo "$OUTDIR/abc_d_${CUR_DS}.out" ;;
        # E/F 的目标必须在 nvme0（见上面 OUTDIR0 的说明）
        e) echo "$OUTDIR0/abc_e.out" ;;
        f) echo "$OUTDIR0/abc_f.out" ;;
        g) echo "$OUTDIR/abc_g.out" ;;
        c) echo "$OUTDIR/abc_c.out" ;;
        b) echo "$OUTDIR/abc_b.out" ;;
        *) echo "" ;;
    esac
}

bin_ok() {
    case $1 in
        a) [ -x /root/dsync_time_test ] ;;
        b) [ -x "$CEMU/build/dsync" ] ;;
        c) [ -x "$CEMU/build/gridds" ] ;;
        d) [ -x "$CEMU/build/gridrb" ] ;;
        e) [ -x "$CEMU/build/rsync_csd" ] ;;
        f) [ -x "$CEMU/build/sync_file" ] ;;
        g) [ -x "$CEMU/build/skysync_c_csd" ] ;;
    esac
}

prep_group() {
    local g=$1
    case $g in
      e|f)
        cp "$OLD" "$FCOPY"          # 老版本放在 device0（这两个工具硬编码 dev0）
        cp "$OLD" "$(out_of $g)"    # 目标先放着老版本，patch 原地更新
        sync                        # 必须落盘：脏页会在设备写完之后回写，覆盖结果
        ;;
      d)
        local o; o=$(out_of d)
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
      a) LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$OLD" "$NEW" 0 ;;
      b) ( cd "$CEMU" && ./build/dsync "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of b)" ) ;;
      c) ( cd "$CEMU" && ./build/gridds "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of c)" ) ;;
      d) ( cd "$CEMU" && ./build/gridrb "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of d)" ) ;;
      e) ( cd "$CEMU" && ./build/rsync_csd "$FCOPY" "$NEW" "$(out_of e)" ) ;;
      f) ( cd "$CEMU" && ./build/sync_file "$FCOPY" "$NEW" "$(out_of f)" ) ;;
      g) ( cd "$CEMU" && ./build/skysync_c_csd "$OLD" "$NEW" -c 0 -s 1 --no-verify -o "$(out_of g)" ) ;;
    esac
}

# 正确性：丢缓存后 cmp。A 没有可控输出路径，退回用它的 reconstructed 字节数
verify_group() {
    local g=$1 log=$2
    if [ "$g" = a ]; then
        local rc; rc=$(grep -oE 'reconstructed=[0-9]+' "$log" | grep -oE '[0-9]+')
        if [ -n "$rc" ] && [ "$rc" = "$NEW_SZ" ]; then echo OK; else echo FAIL; fi
        return
    fi
    local o; o=$(out_of "$g")
    [ -f "$o" ] || { echo FAIL; return; }
    cmp -s "$o" "$NEW" && echo OK || echo FAIL
}

: > "$RES"
: > "$SUM"

for g in $GALL; do
    if ! bin_ok "$g"; then
        echo "警告: [$g] 的可执行文件不存在，该组记 NA" | tee -a "$RES"
    fi
done

for ds in $DATASETS; do
    CUR_DS=$ds
    NEW=$DIR/$ds
    NEW_SZ=$(stat -c %s "$NEW" 2>/dev/null)

    echo "==================== $ds ====================" | tee -a "$RES"
    if [ -z "$NEW_SZ" ]; then
        echo "  跳过：$NEW 不存在" | tee -a "$RES"
        continue
    fi

    # FDM 存活门禁。
    # 设备被卡死之后程序不会报错、只会一直等，跑出来的数全是假的 ——
    # 所以每个数据集开跑前先探一次；用 timeout 包住，免得这个探测自己挂住。
    if ! timeout 5 ls -1 /mnt/fdm0/ >/dev/null 2>&1; then
        echo "  [中止] fdmfs 无响应（设备已被卡死），后续数据集不再测" | tee -a "$RES"
        break
    fi

    # ---------- 正确性把关 ----------
    echo "  [正确性] 丢缓存后 cmp 与 f' 逐字节比对" | tee -a "$RES"
    declare -A STAT
    for g in $GALL; do
        if ! bin_ok "$g"; then STAT[$g]=NA; continue; fi
        prep_group "$g"
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        run_group "$g" >"/tmp/abc_${g}_${ds}.log" 2>&1
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        STAT[$g]=$(verify_group "$g" "/tmp/abc_${g}_${ds}.log")
        printf "    [%s] %s : %s\n" "${g^^}" "$(desc_of $g)" "${STAT[$g]}" | tee -a "$RES"
    done

    # ---------- 计时 ----------
    for g in $GALL; do : > /tmp/abc_v_$g; done
    declare -A FAIL
    for g in $GALL; do FAIL[$g]=0; done

    echo "  [计时] 每组 $N 轮（run1 预热丢弃），每轮前 drop_caches；外部挂钟" \
        | tee -a "$RES"
    for i in $(seq 1 "$N"); do
        for g in $GALL; do
            if ! bin_ok "$g"; then continue; fi
            if [ "${STAT[$g]}" = "FAIL" ] && [ "$i" = 1 ]; then
                : # 仍然测，只是结果不可信，表里会标出来
            fi
            prep_group "$g"
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            t0=$(date +%s%N)
            run_group "$g" >"/tmp/abc_${g}_run.txt" 2>&1
            t1=$(date +%s%N)
            ms=$(( (t1 - t0) / 1000000 ))
            if [ ! -s "/tmp/abc_${g}_run.txt" ] && [ "$g" != a ]; then
                FAIL[$g]=$(( ${FAIL[$g]} + 1 ))
                printf "    run%-2s [%s] FAILED\n" "$i" "${g^^}" | tee -a "$RES"
            else
                printf "    run%-2s [%s] %6s ms\n" "$i" "${g^^}" "$ms" | tee -a "$RES"
                [ "$i" -gt 1 ] && echo "$ms" >> /tmp/abc_v_$g
            fi
        done
    done

    # ---------- 统计 ----------
    echo "  ---- run2..$N 中位（外部挂钟, ms）----" | tee -a "$RES"
    declare -A MED
    for g in $GALL; do
        if ! bin_ok "$g"; then MED[$g]=NA; else MED[$g]=$(med < /tmp/abc_v_$g); fi
        printf "    [%s] %s : 中位 %s ms   正确性 %s   失败 %s\n" \
               "${g^^}" "$(desc_of $g)" "${MED[$g]}" "${STAT[$g]:-NA}" "${FAIL[$g]}" \
               | tee -a "$RES"
    done

    echo "    加速比 A/G (最大下沉)        : $(div "${MED[a]}" "${MED[g]}")x" | tee -a "$RES"
    echo "    加速比 A/D (校验和+重建下沉) : $(div "${MED[a]}" "${MED[d]}")x" | tee -a "$RES"
    echo "    加速比 A/C                   : $(div "${MED[a]}" "${MED[c]}")x" | tee -a "$RES"
    echo "    加速比 B/D                   : $(div "${MED[b]}" "${MED[d]}")x" | tee -a "$RES"
    echo "    加速比 C/D (仅重建下沉)      : $(div "${MED[c]}" "${MED[d]}")x" | tee -a "$RES"
    echo "    加速比 A/B (仅校验和下沉)    : $(div "${MED[a]}" "${MED[b]}")x" | tee -a "$RES"

    for g in $GALL; do
        if [ "$g" = a ]; then
            h=$(grep -oE '命中率: .*' "/tmp/abc_a_${ds}.log" | tail -1)
        else
            h=$(grep -oE '命中率: .*' "/tmp/abc_${g}_${ds}.log" | tail -1)
        fi
        [ -n "$h" ] && echo "    [${g^^}] $h" | tee -a "$RES"
    done

    echo "$ds|${MED[a]}|${MED[b]}|${MED[c]}|${MED[d]}|${MED[e]}|${MED[f]}|${MED[g]}|${STAT[d]}|${STAT[e]}|${STAT[f]}" >> "$SUM"
    echo "" | tee -a "$RES"
    unset MED STAT FAIL
done

# ---------- 总表 ----------
if [ -s "$SUM" ]; then
    echo "==================== 总表（外部挂钟中位, ms） ====================" | tee -a "$RES"
    printf "%-18s %8s %8s %8s %8s %8s %8s %8s\n" \
           "数据集" "A主机" "B" "C" "D" "E" "F" "G" | tee -a "$RES"
    while IFS='|' read -r d a b c dd e f g sd se sf; do
        printf "%-18s %8s %8s %8s %8s %8s %8s %8s\n" \
               "$d" "$a" "$b" "$c" "$dd" "$e" "$f" "$g" | tee -a "$RES"
        printf "  %-16s D=%s E=%s F=%s  （正确性 OK/FAIL）\n" "" "$sd" "$se" "$sf" | tee -a "$RES"
    done < "$SUM"
    echo "" | tee -a "$RES"
fi

echo "完整结果: $RES"
