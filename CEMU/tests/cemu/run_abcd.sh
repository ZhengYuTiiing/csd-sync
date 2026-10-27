#!/bin/bash
# run_abcd.sh —— A/B/C/D 四方案端到端同步时间对比
#
#   [A] host-rsync   : 不在 CSD。数据搬到主机，CPU 现场算 CDC + 弱/强校验和
#                      （/root/dsync_time_test，LD_LIBRARY_PATH=/root/libs）
#   [B] grid-no-cdc  : 在 CSD。只有 client 侧 CDC，server 只折叠网格、不读 f 的数据
#                      （build/dsync）
#   [C] grid-cdc     : 在 CSD。两侧各自 CDC，按内容匹配；命中后重建在主机侧做
#                      （build/gridds）
#   [D] grid-cdc+CSD : 同 C，但**重建下沉到 CSD**：复用区间由设备直接写回闪存，
#                      主机只负责那 0.x% 的字面量
#                      （build/gridrb）
#
# 控制变量：同一台虚拟机、同一对文件、同一 vCPU、同一存储、每轮前 drop_caches。
# 自变量：A vs B/C/D = 同步计算是否下沉到 CSD；C vs D = 重建是否也下沉。
#
# 时间口径（沿用 run_ab3.sh）：
#   A = 四项合计 + Server Patch（它的四项里不含 patch）
#   B/C/D = 程序总耗时
#   四组都带 --no-verify：逐字节校验是测试开销，量级相当但无意义，
#   放进来只会把差异摊平。正确性由每组开头单独跑一次带校验的把关。
#
# 方案 D 的特别说明：
#   设备直接写闪存 LBA，要求输出文件的 extent 处于 "written" 状态，否则
#   ext4 的 buffered read 会返回零。首次需要真写一遍零 + fsync（10MB 约 140ms），
#   这一步是**一次性成本**，且不属于同步开销（真实场景里是更新既有文件）。
#   所以脚本给每个数据集用一个固定输出文件：正确性把关那一跑顺手完成准备，
#   计时阶段全部复用，不计入。为此 D 的输出文件在轮次之间**不删除**。
#
# 用法:
#   ./run_abcd.sh [每组跑几次，默认 5] ["数据集列表"]
#   ./run_abcd.sh 5
#   ./run_abcd.sh 3 "10MB-insert-4KB 10MB-cut-4KB"
#
# 100MB:
#   OLD=/mnt/nvme1/dataset/100M/100MB DIR=/mnt/nvme0/dataset/100M \
#       ./run_abcd.sh 3 "100MB-insert-8MB"

OLD=${OLD:-/mnt/nvme1/dataset/10M/10MB}
DIR=${DIR:-/mnt/nvme0/dataset/10M}
N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
RES=/tmp/abcd_result.txt
SUM=/tmp/abcd_sum.txt
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

# 从一行里取阶段耗时：形如「重建: 复用 f 主机 194.5 1 ...」→ 取"主机/设备"后面那个数
phase_ms() { grep -F -- "$2" "$1" | head -1 | \
             awk '{for(i=1;i<=NF;i++) if($i=="主机"||$i=="设备"){print $(i+1); exit}}'; }

: > "$RES"
: > "$SUM"

if [ ! -x /root/dsync_time_test ]; then
    echo "警告: /root/dsync_time_test 不存在，[A] 组将全部记 NA" | tee -a "$RES"
fi
for b in dsync gridds gridrb; do
    if [ ! -x "$CEMU/build/$b" ]; then
        echo "错误: $CEMU/build/$b 不存在，请先 make" | tee -a "$RES"
        exit 1
    fi
done

for ds in $DATASETS; do
    NEW=$DIR/$ds
    NEW_SZ=$(stat -c %s "$NEW" 2>/dev/null)

    echo "==================== $ds ====================" | tee -a "$RES"

    if [ -z "$NEW_SZ" ]; then
        echo "  跳过：$NEW 不存在" | tee -a "$RES"
        continue
    fi

    # ---------- 正确性把关：每组跑一次带逐字节校验的 ----------
    echo "  [正确性] 带逐字节校验各跑一次" | tee -a "$RES"
    if [ -x /root/dsync_time_test ]; then
        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$OLD" "$NEW" 0 \
            >/tmp/abcd_o_a.txt 2>&1
        rc_a=$(grep -oE 'reconstructed=[0-9]+' /tmp/abcd_o_a.txt | grep -oE '[0-9]+')
        if [ -n "$rc_a" ] && [ "$rc_a" = "$NEW_SZ" ]; then
            echo "    [A] 重建 $rc_a B == f' 大小，OK" | tee -a "$RES"
        else
            echo "    [A] 重建字节数 '${rc_a:-空}' != f' 大小 ${NEW_SZ:-?}，请检查" | tee -a "$RES"
        fi
    fi

    for grp in b c d; do
        case $grp in
            b) bin=dsync;  desc="grid-no-cdc" ;;
            c) bin=gridds; desc="grid-cdc" ;;
            d) bin=gridrb; desc="grid-cdc+CSD" ;;
        esac
        out="$OUTDIR/abcd_${grp}.out"
        # D 的输出文件按数据集命名并长期保留（见文件头：避免重复付 extent 准备成本）
        [ "$grp" = d ] && out="$OUTDIR/abcd_d_${ds}.out"

        sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
        ( cd "$CEMU" && ./build/$bin "$OLD" "$NEW" -c 0 -s 1 -o "$out" ) \
            >/tmp/abcd_o_${grp}.txt 2>&1
        if grep -q '\[PASS\]' /tmp/abcd_o_${grp}.txt; then
            echo "    [${grp^^}] $desc : PASS 逐字节一致" | tee -a "$RES"
        else
            echo "    [${grp^^}] $desc : **FAIL** 结果不一致，后续时间仅供参考" | tee -a "$RES"
        fi
        eval "HIT_${grp}=\"$(grep -oE '命中率: .*' /tmp/abcd_o_${grp}.txt | tail -1)\""
        eval "XFER_${grp}=\"$(grep -oE '合计 +[0-9]+ B' /tmp/abcd_o_${grp}.txt | tail -1)\""
    done

    # ---------- 计时：N 轮，run1 当预热丢弃 ----------
    : > /tmp/abcd_v_a; : > /tmp/abcd_v_b; : > /tmp/abcd_v_c; : > /tmp/abcd_v_d
    : > /tmp/abcd_r_c; : > /tmp/abcd_r_d
    FAIL_A=0; FAIL_B=0; FAIL_C=0; FAIL_D=0

    echo "  [计时] 每组 $N 轮（run1 预热丢弃），每轮前 drop_caches" | tee -a "$RES"
    for i in $(seq 1 "$N"); do
        # ---- [A] host-rsync ----
        if [ -x /root/dsync_time_test ]; then
            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            LD_LIBRARY_PATH=/root/libs /root/dsync_time_test "$OLD" "$NEW" 0 \
                >/tmp/abcd_o_a.txt 2>&1
            four=$(grep '四项合计'      /tmp/abcd_o_a.txt | f1)
            pat=$( grep 'Server Patch:' /tmp/abcd_o_a.txt | f1)
            if [ -z "$four" ]; then
                FAIL_A=$((FAIL_A+1)); printf "    run%-2s [A] FAILED\n" "$i" | tee -a "$RES"
            else
                tot=$(add "$four" "$pat")
                printf "    run%-2s [A] 四项=%-9s Patch=%-8s 总=%s ms\n" \
                       "$i" "$four" "$pat" "$tot" | tee -a "$RES"
                [ "$i" -gt 1 ] && echo "$tot" >> /tmp/abcd_v_a
            fi
        fi

        # ---- [B] / [C] / [D] ----
        for grp in b c d; do
            case $grp in
                b) bin=dsync  ;;
                c) bin=gridds ;;
                d) bin=gridrb ;;
            esac
            out="$OUTDIR/abcd_${grp}.out"
            [ "$grp" = d ] && out="$OUTDIR/abcd_d_${ds}.out"

            sync; echo 1 >/proc/sys/vm/drop_caches 2>/dev/null
            ( cd "$CEMU" && ./build/$bin "$OLD" "$NEW" -c 0 -s 1 --no-verify \
                  -o "$out" ) >/tmp/abcd_o_${grp}.txt 2>&1
            tot=$(grep '程序总耗时' /tmp/abcd_o_${grp}.txt | f1)
            if [ -z "$tot" ]; then
                eval "FAIL_${grp^^}=\$((FAIL_${grp^^}+1))"
                printf "    run%-2s [%s] FAILED\n" "$i" "${grp^^}" | tee -a "$RES"
            else
                printf "    run%-2s [%s] 总=%s ms\n" "$i" "${grp^^}" "$tot" | tee -a "$RES"
                [ "$i" -gt 1 ] && echo "$tot" >> /tmp/abcd_v_${grp}
                # 重建阶段分项：C 主机侧复用拷贝 vs D 设备内复用
                if [ "$i" -gt 1 ]; then
                    if [ "$grp" = c ]; then
                        r=$(phase_ms /tmp/abcd_o_c.txt "重建: 复用 f")
                        [ -n "$r" ] && echo "$r" >> /tmp/abcd_r_c
                    elif [ "$grp" = d ]; then
                        r=$(phase_ms /tmp/abcd_o_d.txt "设备内复用 f")
                        [ -n "$r" ] && echo "$r" >> /tmp/abcd_r_d
                    fi
                fi
            fi
        done
    done

    # ---------- 统计（run2..N） ----------
    as=$(stats < /tmp/abcd_v_a); bs=$(stats < /tmp/abcd_v_b)
    cs=$(stats < /tmp/abcd_v_c); ds_=$(stats < /tmp/abcd_v_d)
    am=$(echo $as | cut -d' ' -f1); bm=$(echo $bs | cut -d' ' -f1)
    cm=$(echo $cs | cut -d' ' -f1); dm=$(echo $ds_ | cut -d' ' -f1)

    echo "  ---- run2..$N 中位 / min / max ----" | tee -a "$RES"
    line() { # $1=标签 $2="med min max" $3=失败数
        echo "    $1 中位 $(echo $2 | cut -d' ' -f1) ms" \
             " (min $(echo $2 | cut -d' ' -f2) / max $(echo $2 | cut -d' ' -f3))" \
             " 失败 $3" | tee -a "$RES"
    }
    line "[A] host-rsync    :" "$as"  "$FAIL_A"
    line "[B] grid-no-cdc   :" "$bs"  "$FAIL_B"
    line "[C] grid-cdc      :" "$cs"  "$FAIL_C"
    line "[D] grid-cdc+CSD  :" "$ds_" "$FAIL_D"

    echo "    加速比 A/D (全程下沉)      : $(div "$am" "$dm")x" | tee -a "$RES"
    echo "    加速比 C/D (仅重建下沉)    : $(div "$cm" "$dm")x" | tee -a "$RES"
    echo "    加速比 A/C                 : $(div "$am" "$cm")x" | tee -a "$RES"
    echo "    加速比 B/D                 : $(div "$bm" "$dm")x" | tee -a "$RES"
    echo "    加速比 A/B (仅校验和下沉)  : $(div "$am" "$bm")x" | tee -a "$RES"

    # 重建阶段单独对比：这是 D 的直接作用点
    if [ -s /tmp/abcd_r_c ] && [ -s /tmp/abcd_r_d ]; then
        rcm=$(stats < /tmp/abcd_r_c | cut -d' ' -f1)
        rdm=$(stats < /tmp/abcd_r_d | cut -d' ' -f1)
        echo "    重建阶段: C 主机复用 ${rcm} ms -> D 设备内 ${rdm} ms" \
             " = $(div "$rcm" "$rdm")x" | tee -a "$RES"
    fi

    [ -n "$HIT_a" ] && echo "    [A] ${HIT_a}"          | tee -a "$RES"
    [ -n "$HIT_b" ] && echo "    [B] ${HIT_b}"          | tee -a "$RES"
    [ -n "$HIT_c" ] && echo "    [C] ${HIT_c}"          | tee -a "$RES"
    [ -n "$HIT_d" ] && echo "    [D] ${HIT_d}"          | tee -a "$RES"
    [ -n "$XFER_b" ] && echo "    [B] 跨界 ${XFER_b}"   | tee -a "$RES"
    [ -n "$XFER_c" ] && echo "    [C] 跨界 ${XFER_c}"   | tee -a "$RES"
    [ -n "$XFER_d" ] && echo "    [D] 跨界 ${XFER_d}"   | tee -a "$RES"

    # 汇总行，方便最后拼总表
    echo "$ds|$am|$bm|$cm|$dm|$(div "$am" "$dm")|$(div "$cm" "$dm")" >> "$SUM"
    echo "" | tee -a "$RES"
done

# ---------- 跨数据集总表 ----------
if [ -s "$SUM" ]; then
    echo "==================== 总表（中位 ms） ====================" | tee -a "$RES"
    printf "%-18s %10s %10s %10s %10s %8s %8s\n" \
           "数据集" "A主机" "B网格" "C网格CDC" "D重建下沉" "A/D" "C/D" | tee -a "$RES"
    while IFS='|' read -r d a b c dd ad cd; do
        printf "%-18s %10s %10s %10s %10s %8s %8s\n" \
               "$d" "$a" "$b" "$c" "$dd" "${ad}x" "${cd}x" | tee -a "$RES"
    done < "$SUM"
    echo "" | tee -a "$RES"
fi

echo "完整结果: $RES"
