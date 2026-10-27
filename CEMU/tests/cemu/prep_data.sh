#!/bin/bash
# 给 E（rsync_csd）/ F（sync_file）准备 FDM 槽位 /mnt/fdm0/0..3。
#
# 这两个程序用 plain open 打开这几个文件，要求文件已存在且容量够用：
#   0 = 匹配结果输出
#   1 = 老文件指纹 / 数据输入
#   2 = 老文件数据          ← 必须装得下整个老文件
#   3 = 重构输出
#
# 槽位 2 要装下整个老文件，所以大小取决于数据集规模：
#   10MB  数据集 → 16MB 就够（原脚本写死的值）
#   100MB 数据集 → 必须 100MB 以上，16MB 会失败
# 默认取 160MB，两种都覆盖。也可以显式指定：
#   bash prep_data.sh        # 默认 160MB
#   bash prep_data.sh 256    # 256MB
#
# 用法: bash prep_data.sh [每个槽位的 MB 数，默认 160]

MB=${1:-160}
WANT=$((MB * 1024 * 1024))

cd /mnt/fdm0 || exit 1

for ((i = 0; i < 4; i++)); do
	# 已存在且够大就复用；容量不够的先删掉重建。
	# 注意必须是 rm（unlink）而不是直接 fallocate 扩容：
	# unlink 会走 fdmfs_deallocate 释放设备侧内存，否则旧分配会泄漏，
	# 泄漏累积到一定程度 fallocate 就会报 Cannot allocate memory。
	if [ -f $i ]; then
		have=$(stat -c %s "$i" 2>/dev/null || echo 0)
		if [ "$have" -ge "$WANT" ]; then
			continue
		fi
		rm -f "$i"
	fi
	touch "$i" || { echo "prep_data: touch $i 失败"; exit 1; }
	fallocate -l "${MB}m" "$i" || {
		echo "prep_data: fallocate $i (${MB}m) 失败 —— FDM 内存可能不足"
		exit 1
	}
done

echo "prep_data: /mnt/fdm0/0..3 各 ${MB}MB"
