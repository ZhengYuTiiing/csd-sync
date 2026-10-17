#!/bin/bash
# 虚拟机环境重建：NVMe 命名空间是内存模拟，QEMU 重启后内容全丢。
# 顺序很重要：先格式化并挂载 n1（数据盘），再挂 m2（fdmfs），
# 否则自动探测会把 n1 也按 fdmfs 挂上并把设备占住。
set -e

for dev in 0 1; do
    mountpoint -q /mnt/nvme$dev && umount /mnt/nvme$dev
    mkfs.ext4 -q -F /dev/nvme${dev}n1
    mount -t ext4 /dev/nvme${dev}n1 /mnt/nvme$dev
done

# 槽位一次性预分配：ensure_fdm 在容量不够时会 unlink 重建，
# 旧内核的 fdmfs 在这条路径上会与设备侧残留引用互锁（已修，但预分配更稳）。
# 0..3 = 160MB：sync_file / rsync_csd 要放老文件数据与 patch 输出（最大 ~113MB）
# 4..11 = 64MB：其余算子的托盘/网格/清单/描述符
for dev in 0 1; do
    mountpoint -q /mnt/fdm$dev || mount -t fdmfs /dev/nvme${dev}m2 /mnt/fdm$dev
    for j in 0 1 2 3; do
        f=/mnt/fdm$dev/$j
        if [ ! -f "$f" ]; then
            touch "$f"
            fallocate -l 160m "$f"
        fi
    done
    for j in 4 5 6 7 8 9 10 11; do
        f=/mnt/fdm$dev/$j
        if [ ! -f "$f" ]; then
            touch "$f"
            fallocate -l 64m "$f"
        fi
    done
done

mkdir -p /mnt/nvme1/dataset/10M /mnt/nvme0/dataset/10M
mkdir -p /mnt/nvme1/dataset/100M /mnt/nvme0/dataset/100M
mkdir -p /mnt/nvme0/sync /mnt/nvme1/sync

# 全部数据集均为 4KB 整数倍：sync_file / rsync_csd 的硬性前置。
# 变体命名与大小对应：insert-4KB / cut-4KB。
python3 - <<'PY'
import os

def rnd(n):
    return os.urandom(n)

MB = 1024 * 1024

base = rnd(10 * MB)
open('/mnt/nvme1/dataset/10M/10MB', 'wb').write(base)
open('/mnt/nvme0/dataset/10M/10MB.new', 'wb').write(base + rnd(1 * MB))
open('/mnt/nvme0/dataset/10M/10MB-insert-4KB', 'wb').write(
    base[:5 * MB] + rnd(4096) + base[5 * MB:])
open('/mnt/nvme0/dataset/10M/10MB-cut-4KB', 'wb').write(
    base[:3 * MB] + base[3 * MB + 4096:])
open('/mnt/nvme0/dataset/10M/10MB-inverse-1MB', 'wb').write(
    base[:4 * MB] + rnd(1 * MB) + base[5 * MB:])

big = rnd(100 * MB)
open('/mnt/nvme1/dataset/100M/100MB', 'wb').write(big)
open('/mnt/nvme0/dataset/100M/100MB-insert-8MB', 'wb').write(
    big[:50 * MB] + rnd(8 * MB) + big[50 * MB:])
PY

sync
ls -l /mnt/nvme1/dataset/10M/ /mnt/nvme0/dataset/10M/ | awk '{print $NF, $5}'
ls -l /mnt/nvme1/dataset/100M/ /mnt/nvme0/dataset/100M/ | awk '{print $NF, $5}'
