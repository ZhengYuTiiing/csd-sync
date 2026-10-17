BUILD_DIR := build
TARGETS = \
	$(BUILD_DIR)/libcemu.a \
	$(BUILD_DIR)/cemu_benchmark \
	$(BUILD_DIR)/test_fdmfs \
	$(BUILD_DIR)/vadd_example \
	$(BUILD_DIR)/test_indirect \
	$(BUILD_DIR)/test_p2p \
	$(BUILD_DIR)/test_iouring \
	$(BUILD_DIR)/test_sync_breakdown \
	$(BUILD_DIR)/test_chunk_csf \
	$(BUILD_DIR)/test_chunk_csf_cemu \
	$(BUILD_DIR)/test_sync_csd \
	$(BUILD_DIR)/benchmark_sync_csd \
	$(BUILD_DIR)/read_file \
	$(BUILD_DIR)/sync_file \
	$(BUILD_DIR)/crc_compose_test \
	$(BUILD_DIR)/crc_oob_test \
	$(BUILD_DIR)/fastsync \
	$(BUILD_DIR)/dsync \
	$(BUILD_DIR)/gridds

CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -Wno-unused-function -pedantic -g
CXXFLAGS = $(CFLAGS)

.PHONY: all kernel

all: $(BUILD_DIR) $(TARGETS)

kernel:
	$(MAKE) -C kernel all

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	chmod 777 $(BUILD_DIR)

$(BUILD_DIR)/libcemu.a: util.c util.h cemu_ioctl.h
	$(CC) $(CFLAGS) -c -o $(BUILD_DIR)/util.o $<
	ar rcs $@ $(BUILD_DIR)/util.o

$(BUILD_DIR)/test_fdmfs: test_fdmfs.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CFLAGS) -o $@ $^

$(BUILD_DIR)/vadd_example: vadd_example.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/cemu_benchmark: cemu_benchmark.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/test_iouring: test_iouring.c $(BUILD_DIR)/libcemu.a
	$(CC) $(CFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/test_chunk_csf: test_chunk_csf.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -ldl -lz

# L1（CEMU 上真跑）需要 liburing >= 2.2 才有 URING_CMD/SQE128：
# 系统头不够新时 test_chunk_csf_cemu 会自动退化成 ioctl 串行提交（功能照验）。
# 想跑 io_uring 并发提交（推荐，顺带验 CSD 调度）：
#   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src
LIBURING_PREFIX ?=
ifneq ($(LIBURING_PREFIX),)
URING_CFLAGS  = -I$(LIBURING_PREFIX)/include
URING_LDFLAGS = -L$(LIBURING_PREFIX) -Wl,-rpath,$(LIBURING_PREFIX)
endif

$(BUILD_DIR)/test_chunk_csf_cemu: test_chunk_csf_cemu.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring -lz

$(BUILD_DIR)/test_sync_csd: test_sync_csd.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -ldl

$(BUILD_DIR)/benchmark_sync_csd: benchmark_sync_csd.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -ldl -lz

# 最小验证：让 CSD 读取文件
# 需 liburing >= 2.2（URING_CMD/SQE128/CQE32）：
#   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src build/read_file
$(BUILD_DIR)/read_file: read_file.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# 文件同步整体下沉：rsync_match(浅读指纹) + rsync_patch(设备内重构)
#   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src build/sync_file
$(BUILD_DIR)/sync_file: sync_file.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# 可组合 CRC 验证：任意 512B 对齐区间的校验和由定长槽位拼出
$(BUILD_DIR)/crc_compose_test: crc_compose_test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -ldl

# 端到端验证：设备浅读 OOB 取回固化槽位 + 设备侧拼接任意边界 CRC32C
$(BUILD_DIR)/crc_oob_test: crc_oob_test.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring -ldl

# 一行命令跑完设备内增量同步（设备 CDC + 网格拼接 + 扇区位图）
#   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src build/fastsync
$(BUILD_DIR)/fastsync: fastsync.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# 双 CSD 版：client CSD 出清单 / server CSD 折网格，显式统计跨界字节
#   make LIBURING_PREFIX=/home/zyt/skysync/thirdparty/liburing/src build/dsync
$(BUILD_DIR)/dsync: dsync.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# 回收泄漏的 memory range set (rsid)：设备池 1024，历史测试只建不删会耗尽
$(BUILD_DIR)/rsid_reaper: rsid_reaper.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^

# 整文件一次扫描的 CDC 块数参照（仲裁 dsync2 硬切窗 vs dsync3 carry 接力）
$(BUILD_DIR)/cdc_ref: cdc_ref.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

# 并发实验：多条 in-flight indirect 命令是否真并行
$(BUILD_DIR)/conc_test: conc_test.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# rsync-CSD：单 CSD，设备读数据现场算指纹 + 设备内 patch
$(BUILD_DIR)/rsync_csd: rsync_csd.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

# skysync-c-CSD：双 CSD，CDC 切点照扫 + 弱哈希网格折叠 + fold_manifest 匹配
$(BUILD_DIR)/skysync_c_csd: skysync_c_csd.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring

$(BUILD_DIR)/test_indirect: test_indirect.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/test_parallel_chunks_threads: test_parallel_chunks_threads.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/test_sync_breakdown: test_sync_breakdown.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -luring

$(BUILD_DIR)/test_p2p: test_p2p.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) -o $@ $^ -luring

clean:
	rm -rf $(BUILD_DIR)

# grid-cdc：双 CSD，6 步流程，设备侧只有一个算子 grid_ds.so
$(BUILD_DIR)/gridds: gridds.cpp $(BUILD_DIR)/libcemu.a
	$(CXX) $(CXXFLAGS) $(URING_CFLAGS) -o $@ $^ $(URING_LDFLAGS) -luring
