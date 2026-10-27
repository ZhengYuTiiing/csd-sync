#!/bin/bash
# 主机侧四方案对比：dsync / rsync / skysync-f / skysync-c
#
# 四个方案全部在 host CPU 上执行，不使用 CSD —— 用来回答"这几个算法本身谁快"。
# 和 run_abcdefg.sh 的表对照，就能看出"下沉到 CSD"各自带来多少收益。
#
# 用法：
#   ./run_host4.sh                        # 5 轮，10M 的四个数据集
#   ./run_host4.sh 3                      # 3 轮
#   ./run_host4.sh 5 "10MB.new"           # 只跑指定数据集
#   OLD=/mnt/nvme1/dataset/100M/100MB DIR=/mnt/nvme0/dataset/100M \
#       ./run_host4.sh 3 "100MB-insert-8MB"
#
# 计时口径与 run_abcdefg.sh 一致：外部挂钟，run1 预热丢弃，run2..N 取中位。
#
# 注意：这四个主机版二进制只做计时，不产出结果文件，所以无法用 cmp 校验正确性。
#       正确性由 CSD 版（run_abcdefg.sh）负责验证。

N=${1:-5}
DATASETS=${2:-"10MB.new 10MB-insert-4KB 10MB-cut-4KB 10MB-inverse-1MB"}
OLD=${OLD:-/mnt/nvme1/dataset/10M/10MB}
DIR=${DIR:-/mnt/nvme0/dataset/10M}

RES=/tmp/host4_result.txt
SUM=/tmp/host4_sum.txt
: > "$RES"
: > "$SUM"

TOOLS="a b c d"

tname() {
	case $1 in
	a) echo "dsync    " ;;
	b) echo "rsync    " ;;
	c) echo "skysync-f" ;;
	d) echo "skysync-c" ;;
	esac
}

tbin() {
	case $1 in
	a) echo /root/dsync_time_test ;;
	b) echo /root/rsync_time_test ;;
	c) echo /root/skysync_f_time_test ;;
	d) echo /root/skysync_c_time_test ;;
	esac
}

med() {
	sort -n | awk '{a[NR]=$1} END{
		n=NR; if(n==0){print "NA"; exit}
		m = (n%2) ? a[(n+1)/2] : (a[n/2]+a[n/2+1])/2;
		printf "%.0f\n", m }'
}

div() {
	awk -v a="${1:-0}" -v b="${2:-0}" \
		'BEGIN{ if(b+0>0 && a!="NA" && b!="NA") printf "%.2f", a/b; else print "NA" }'
}

for b in /root/dsync_time_test /root/rsync_time_test \
	/root/skysync_f_time_test /root/skysync_c_time_test; do
	if [ ! -x "$b" ]; then
		echo "缺少 $b，先确认环境"
		exit 1
	fi
done

{
	echo "主机侧四方案（全部在 host CPU 执行，不使用 CSD）"
	echo "  老文件  : $OLD"
	echo "  数据集  : $DIR"
	echo "  轮数    : $N（run1 预热丢弃，run2..$N 取中位）"
	echo "  计时    : 外部挂钟，每轮前 sync + drop_caches(1)"
	echo
} | tee -a "$RES"

for ds in $DATASETS; do
	NEW="$DIR/$ds"
	if [ ! -f "$NEW" ]; then
		echo "[跳过] $ds 不存在" | tee -a "$RES"
		continue
	fi

	echo "==================== $ds ====================" | tee -a "$RES"

	for t in $TOOLS; do
		bin=$(tbin $t)
		vals=()
		fail=0

		for ((r = 1; r <= N; r++)); do
			sync
			# 只丢页缓存（不能写 2/3，否则 fdmfs 的 inode 被回收、槽位消失）
			echo 1 >/proc/sys/vm/drop_caches

			s=$(date +%s%N)
			LD_LIBRARY_PATH=/root/libs timeout -s KILL 600 \
				"$bin" "$OLD" "$NEW" 0 >"/tmp/host4_${t}.log" 2>&1
			rc=$?
			e=$(date +%s%N)
			ms=$(((e - s) / 1000000))

			printf "    run%d  [%s] %6d ms\n" "$r" "$(tname $t)" "$ms" | tee -a "$RES"
			[ $rc -ne 0 ] && fail=$((fail + 1))
			[ $r -ge 2 ] && vals+=("$ms")
		done

		m=$(printf '%s\n' "${vals[@]}" | med)
		eval "M_$t=$m"
		eval "F_$t=$fail"
		printf "    [%s] 中位 %s ms   失败 %s\n" "$(tname $t)" "$m" "$fail" | tee -a "$RES"
	done

	echo "$ds|${M_a}|${M_b}|${M_c}|${M_d}|${F_a}|${F_b}|${F_c}|${F_d}" >>"$SUM"
	echo | tee -a "$RES"
done

{
	echo "==================== 总表（外部挂钟中位, ms） ===================="
	printf "%-22s %10s %10s %10s %10s\n" "数据集" "dsync" "rsync" "skysync-f" "skysync-c"
	while IFS='|' read -r ds a b c d fa fb fc fd; do
		printf "%-22s %10s %10s %10s %10s\n" "$ds" "$a" "$b" "$c" "$d"
	done <"$SUM"

	echo
	echo "==================== 相对 dsync 的加速比 ===================="
	printf "%-22s %10s %10s %10s %10s\n" "数据集" "dsync" "rsync" "skysync-f" "skysync-c"
	while IFS='|' read -r ds a b c d fa fb fc fd; do
		printf "%-22s %10s %10s %10s %10s\n" "$ds" \
			"1.00" "$(div "$a" "$b")" "$(div "$a" "$c")" "$(div "$a" "$d")"
	done <"$SUM"

	echo
	echo "失败次数（应全为 0）："
	while IFS='|' read -r ds a b c d fa fb fc fd; do
		printf "  %-22s dsync=%s rsync=%s f=%s c=%s\n" "$ds" "$fa" "$fb" "$fc" "$fd"
	done <"$SUM"

	echo
	echo "完整结果: $RES"
	echo "汇总文件: $SUM"
} | tee -a "$RES"
