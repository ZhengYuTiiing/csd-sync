# CSD 增量同步实验台（CEMU / FEMU）

在一台跑 FEMU（NVMe SSD 模拟器）的虚拟机里，把「增量同步」里的校验和计算下沉到
计算型存储设备（CSD）上，并和纯主机侧实现做同环境对比。

当前主线实现是 **grid-cdc**：6 步流程，设备侧**只有一个算子** `grid_ds.so`、
主机侧**只有一个驱动** `gridds`。

---

## 目录速览

```
~/cemu/
├── CEMU/
│   ├── build/                 QEMU 二进制 + 启动脚本 run-csd.sh（在这里执行）
│   ├── femu-scripts/          启动脚本原件（femu-copy-scripts.sh 拷到 build/）
│   ├── hw/femu/backend/       FEMU 后端；csd_fp_crc.h / csd_sha1.h 是网格与强哈希
│   └── tests/cemu/
│       ├── gridds.cpp         主机侧驱动（grid-cdc 6 步编排）
│       ├── dsync.cpp          主机侧驱动（grid-no-cdc 基线）
│       ├── kernel/grid_ds.c   设备侧算子（三种角色合一）
│       ├── kernel/dsync_abi.h 主机/设备共享的 ABI
│       ├── vm_setup.sh        虚拟机环境重建 + 数据集生成
│       └── run_ab3.sh         三组对比脚本
├── linux-cemu/                内核（含 fdmfs、CSD ioctl）
├── dqib_amd64-pc/             虚拟机启动盘（image.qcow2 2.8G，别进 git）
└── vssh.py                    主机 → 虚拟机一键执行命令
```

---

## 1. 编译（在主机上，不用进虚拟机）

```bash
cd ~/cemu/CEMU/tests/cemu
make -C kernel                                                  # 设备侧算子 → build/*.so
make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src build/gridds
```

源码目录通过 virtio-9p 共享进虚拟机（主机 `~/cemu/` → VM `/root/CEMU/`），
所以**主机编译完，虚拟机里立刻能看到**，不要在 VM 里重编。

---

## 2. 启动虚拟机

```bash
cd ~/cemu/CEMU/build
./run-csd.sh
```

> 必须在 `CEMU/build` 目录下执行 —— 脚本里全是相对路径
> （`../../linux-cemu/vmlinux`、`../`、`./cemu_config.json`）。
> 脚本内部是 `sudo ./x86_64-softmmu/qemu-system-x86_64 ...`，要 root。

启动后是 `-nographic`，**当前终端会被虚拟机串口占住**。想回到 VM 控制台就在这个终端
按回车登录；要跑命令另开一个终端走 ssh。

停止：`sudo pkill -f qemu-system-x86_64`。

---

## 3. 登录虚拟机

```bash
ssh root@localhost -p 2222          # 密码 root
```

端口 2222 来自启动参数里的 `hostfwd=tcp::2222-:22`。

更省事的是从主机一键执行（自动填密码、自动带回输出）：

```bash
cd ~/cemu
python3 vssh.py "hostname"
python3 vssh.py "cd /root/CEMU/tests/cemu && ./vm_setup.sh"
```

`vssh.py` 只接受**一个参数**，整条命令要一起加引号。

---

## 4. 生成数据集（每次重启虚拟机后都要重做）

NVMe 命名空间是**内存模拟**的，QEMU 一重启 `/mnt/nvme0`、`/mnt/nvme1` 内容全丢。

```bash
# 在虚拟机里
cd /root/CEMU/tests/cemu && ./vm_setup.sh

# 或从主机
cd ~/cemu && python3 vssh.py "cd /root/CEMU/tests/cemu && ./vm_setup.sh"
```

`vm_setup.sh` 依次做三件事：

1. `mkfs.ext4` + 挂载 `/dev/nvme0n1` → `/mnt/nvme0`、`/dev/nvme1n1` → `/mnt/nvme1`
2. 挂载 fdmfs（`/dev/nvme{0,1}m2` → `/mnt/fdm{0,1}`）并**预分配槽位**
   （0..3 各 160MB，4..11 各 64MB —— 不预分配的话 fdmfs 在扩容路径上会和
   设备侧残留引用互锁）
3. 生成数据集

### 数据集布局

约定：**f 在 server（`nvme1`），f' 在 client（`nvme0`）**。

| 文件 | 位置 | 含义 |
|---|---|---|
| `10MB` | nvme1 | f，10 MB 随机 |
| `10MB.new` | nvme0 | f' = f + 尾部追加 1 MB |
| `10MB-insert-4KB` | nvme0 | f' = f 中间（5MB 处）插入 4 KB |
| `10MB-cut-4KB` | nvme0 | f' = f 中间（3MB 处）删除 4 KB |
| `10MB-inverse-1MB` | nvme0 | f' = f 的 [4MB,5MB) 换成了 1 MB 新数据 |
| `100MB` | nvme1 | f，100 MB 随机 |
| `100MB-insert-8MB` | nvme0 | f' = f 中间（50MB 处）插入 8 MB |

全部是 4KB 整数倍，这是 `sync_file` / `rsync_csd` 的硬性前置。

### 一个必须满足的前置条件

`gridds` 依赖两侧 OOB 里已经固化的 **512B CRC 网格**，网格是**文件经该设备写路径写入时**
顺便算出来的。所以：

- f 必须写在 **server 设备**（`nvme1`）上
- f' 必须写在 **client 设备**（`nvme0`）上

`vm_setup.sh` 生成的文件天然满足。**把数据集用 `scp` 拷到 VM 里、或拷错设备，
网格就是空的，同步会直接失败或命中率归零。**

---

## 5. 跑一次同步

```bash
cd ~/cemu && python3 vssh.py "cd /root/CEMU/tests/cemu && ./build/gridds \
  /mnt/nvme1/dataset/10M/10MB \
  /mnt/nvme0/dataset/10M/10MB-insert-4KB \
  -c 0 -s 1 -o /mnt/nvme1/sync/out.bin"
```

- `-c 0` client 设备号（持 f'），`-s 1` server 设备号（持 f），**必须是两个不同设备**
- `-w MB` 窗口大小（默认 32MB，FDM 容量不够会自动折半）
- `--no-verify` 跳过逐字节校验（测时间时用它，校验是测试开销）
- 省略 `-o` 时输出为 `<f'>.gridds`

看到的输出对应 6 步：

```
[1][2] client CSD CDC+网格弱哈希 : 876 块, weak 清单 42080 B, 未算任何强哈希
[3] 线上 weak 清单 -> server      : 42080 B
[4][5] server CDC+weak 比对       : 弱哈希命中 874 块，只对这些块算了 SHA1
[5] 线上 强哈希列表 -> client     : 874 条 / 48976 B
[6] client 补算强哈希并比对       : 弱命中 874 块 -> 强确认 874 块（淘汰 0 块）
[6] 线上 delta -> server          : 令牌 874 条 + 字面量 18944 B (0.2% of f')
    命中率: 99.8%      跨界字节 110000 B vs 直接传 f' 需要 10489856 B
[PASS] 重建出的 f' 与 client 的 f' 逐字节一致
```

后面还有一张分阶段耗时表（设备 / 主机 / 线上三类分别统计）。
`设备内算子 build_id = 3` 那行用来确认设备里跑的是哪份 `.so`（见第 7 节）。

---

## 6. 三组对比：`run_ab3.sh`

```bash
# 默认 4 个 10M 数据集，每组 5 轮
cd ~/cemu && python3 vssh.py "cd /root/CEMU/tests/cemu && bash ./run_ab3.sh 5"

# 指定轮数和数据子集
python3 vssh.py "cd /root/CEMU/tests/cemu && bash ./run_ab3.sh 3 '10MB-insert-4KB 10MB-cut-4KB'"
```

结果写在虚拟机的 `/tmp/ab3_result.txt`。

### 三组是什么

| 组 | 实现 | 含义 |
|---|---|---|
| **A** | `/root/dsync_time_test` | 不在 CSD。数据搬到主机，CPU 现场算 CDC + 弱/强校验和 |
| **B** | `build/dsync` | 在 CSD。校验和由网格组合出；只有 client 侧 CDC，server 只折叠网格、**不读 f 的数据** |
| **C** | `build/gridds` | 在 CSD。校验和由网格组合出；**两侧各自 CDC**，按内容匹配 |

自变量：A vs B/C = 计算是否下沉到 CSD；B vs C = server 侧是否做 CDC。

### 脚本怎么保证可比

- 每组跑 N 轮，**run1 当预热丢弃**，取 run2..N 的中位 / min / max
- 每轮前 `sync; echo 3 > /proc/sys/vm/drop_caches`
- 计时阶段统一带 `--no-verify`；正确性由**每组开头单独跑一次带逐字节校验的**把关
- 时间口径：A = `四项合计 + Server Patch`；B/C = `程序总耗时`

### 我这轮实测（10MB，中位 ms，N=5）

| 数据集 | A 主机 | B 网格 | C 网格+CDC | A/B | A/C |
|---|---|---|---|---|---|
| 10MB.new | 1238.7 | 385.7 | 463.3 | 3.21x | 2.67x |
| 10MB-insert-4KB | 1204.0 | 371.9 | 456.7 | 3.24x | 2.64x |
| 10MB-cut-4KB | 1245.8 | 326.5 | 450.1 | 3.82x | 2.77x |
| 10MB-inverse-1MB | 1175.0 | 382.9 | 448.3 | 3.07x | 2.62x |

命中率：B 是 90.9 / 99.4 / 100 / 90.0 %，C 是 90.9 / 99.8 / 99.9 / 89.7 %。

C 比 B 慢 20~30% 是**结构性**的：C 要读 f' 三遍（CDC 切分 → 补算强哈希）、f 一遍，
且设备侧逐字节扫 gear 表；B 的 server 侧一次数据都不读，只折叠写时固化的网格。
另外 B/C 的时间里含 Stage-A「网格装载」，真实部署里网格写文件时就算好了，这趟不该算。

---

## 7. 几个会浪费你一整天的坑

### 1）改了设备侧算子，必须重启 QEMU 才生效（最重要）

CSF（`.so`）由 **QEMU 进程自己 dlopen**，已经 dlopen 过的 `.so` **不会重新加载**，
`UNLOAD` / `DOWNLOAD` / `ACTIVATE` 都救不了，换注册名也没用。
所以改完 `kernel/*.c` 之后：

```bash
sudo pkill -f qemu-system-x86_64     # 或直接在串口终端 Ctrl-A X
cd ~/cemu/CEMU/build && ./run-csd.sh
cd ~/cemu && python3 vssh.py "cd /root/CEMU/tests/cemu && ./vm_setup.sh"
```

**唯一的例外**是换一个全新的 `.so` 文件名 + 全新符号名 —— 那是个全新的 dlopen，
不重启也能加载（`grid_ds` 就是这么上线的，当时省了一次重启）。

怎么确认设备里跑的到底是哪份 `.so`：驱动会打印输出表头里的 `build_id`
（`设备内算子 build_id = 3`，主机侧是 `GD_BUILD_ID`）。**改实现时记得把
`GD_BUILD_ID` 加一**，两边不一致就说明设备里还是旧的。

### 2）重启后数据集全丢

见第 4 节。忘了重跑 `vm_setup.sh` 的表现是 `/mnt/nvme0` 空、`gridds` 报 stat 失败。

### 3）indirect 流水线的 R 阶段读不进 OOB 网格

R 阶段恒把数据读进 `mr[1]`，`read_mode` 是整条命令一个标志 —— 数据页和 OOB 网格
不可能在同一次 R 里同时到位。所以先用 `collect_oob` 把整份网格浅读进常驻 FDM 槽
（Stage-A），之后每条 CDC 命令里 `mr[2]` 直接指向那个槽。

### 4）命令返回值会被流水线覆盖

条数只能从输出表头的 `nr` 读，所以输出区**每窗口**（client 补强哈希是每轮）必须先清零。

### 5）CDC 切点必须逐字节检查

`fp` 要**逐字节累积并逐字节检查**，命中后再向上对齐到 512B。
只在 512B 对齐处检查、或每 512B 只喂一个字节，都会把检查点密度降 512 倍 →
期望切点间隔 4MB，远超 `CDC_MAX_CHUNK=64KB` → 每块都被 max 强行截断 →
边界退化成「从窗口头每 64KB 一刀」，不再是内容定义的。
一旦中间有插入，之后永远差一个偏移、再也对齐不上，10MB 插 4KB 的命中率直接卡在 50%。

---

## 8. 其他脚本

| 脚本 | 用途 |
|---|---|
| `run_ab4.sh` | 另一个对比编排（含 100MB、多次重复） |
| `run_ab6.sh` | 同上，不同分组 |
| `test_sync_breakdown.cpp` | 同步各阶段分解计时 |
| `crc_oob_test.cpp` / `crc_compose_test.cpp` | 网格 CRC 的独立单元测试 |

---

## 9. Git

仓库已连 `https://github.com/ZhengYuTiiing/csd-sync.git`（分支 `main`）。

> **注意**：`dqib_amd64-pc/image.qcow2`（2.8 GB）和 `dqib.tar.xz`（826 MB）
> 已被跟踪，超过 GitHub 单文件 100 MB 的硬上限，直接 push 会被拒。
> 推之前要把它们从索引里摘掉（远端目前是空的，重写那唯一一个提交没有风险）：
>
> ```bash
> git rm --cached --ignore-unmatch dqib.tar.xz dqib_amd64-pc/image.qcow2
> git commit --amend --no-edit
> git push -u origin main        # 密码用 PAT，不是 GitHub 登录密码
> ```
