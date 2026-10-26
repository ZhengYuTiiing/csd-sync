BUILD_DIR := ../build
TARGETS = \
	$(BUILD_DIR)/vadd.so \
	$(BUILD_DIR)/lz4.so \
	$(BUILD_DIR)/grep.so \
	$(BUILD_DIR)/sql.so \
	$(BUILD_DIR)/knn.so \
	$(BUILD_DIR)/chunk_csf.so \
	$(BUILD_DIR)/merge_csf.so \
	$(BUILD_DIR)/collect_oob.so \
	$(BUILD_DIR)/rsync_match.so \
	$(BUILD_DIR)/rsync_patch.so \
	$(BUILD_DIR)/dsync_match.so \
	$(BUILD_DIR)/compose_crc.so \
	$(BUILD_DIR)/sync_cdc.so \
	$(BUILD_DIR)/chunk_manifest.so \
	$(BUILD_DIR)/fold_manifest.so \
	$(BUILD_DIR)/cdc_desc.so \
	$(BUILD_DIR)/read_csf.so \
	$(BUILD_DIR)/grid_ds.so \
	$(BUILD_DIR)/grid_pipe.so \
	$(BUILD_DIR)/grid_rb.so \
	$(BUILD_DIR)/vadd.bpf.o

all: $(BUILD_DIR) $(TARGETS)

.PHONY: gear-table
# 从主机 parasync 实现重新生成 gear 表，保证 CSD 与主机 CDC 语义逐字节一致。
# HOST_CDC_HEADER 默认指向本机的 parasync 源码，可按需覆盖。
HOST_CDC_HEADER ?= /home/zyt/parasync/src/fastcdc.h
gear-table:
	@echo '/* 自动生成：与主机 CDC (parasync) 的 gear 表逐字节一致。' > chunk_gear_v2.h
	@echo ' * 源: $(HOST_CDC_HEADER) 中的 GEARv2[256]' >> chunk_gear_v2.h
	@echo ' * 重新生成: make -C tests/cemu/kernel gear-table' >> chunk_gear_v2.h
	@echo ' * 改表 = 改 CDC 语义，必须与主机同步。' >> chunk_gear_v2.h
	@echo ' */' >> chunk_gear_v2.h
	@echo '#ifndef CHUNK_GEAR_V2_H' >> chunk_gear_v2.h
	@echo '#define CHUNK_GEAR_V2_H' >> chunk_gear_v2.h
	@echo '' >> chunk_gear_v2.h
	@echo '#include <stdint.h>' >> chunk_gear_v2.h
	@echo '' >> chunk_gear_v2.h
	awk '/^static uint64_t GEARv2\[256\] = \{/,/^\};/' $(HOST_CDC_HEADER) \
		| sed '1s/.*/static const uint64_t chunk_gear_table[256] = {/' >> chunk_gear_v2.h
	@echo '' >> chunk_gear_v2.h
	@echo '#endif /* CHUNK_GEAR_V2_H */' >> chunk_gear_v2.h
	@grep -c 0x chunk_gear_v2.h

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	chmod 777 $(BUILD_DIR)

CC = gcc
CXX = g++
AR = ar
CFLAGS = -Wall -Wextra -O2 -march=native -std=c11
CXXFLAGS = -Wall -Wextra -O2 -march=native -std=c++11

$(BUILD_DIR)/vadd.so: vadd.c
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/lz4.so: lz4.cc
	$(CXX) $(CXXFLAGS) -shared -fPIC -llz4 -o $@ $< /usr/lib/x86_64-linux-gnu/liblz4.a

$(BUILD_DIR)/grep.so: grep.cc
	$(CXX) $(CXXFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/sql.so: sql.cc
	$(CXX) $(CXXFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/knn.so: knn.cc
	$(CXX) $(CXXFLAGS) -shared -fPIC -o $@ $<

# CDC 算子族共享的 ABI + CRC32（chunk_crc32.c 编进每个 .so，避免运行期外部依赖）
CSF_COMMON = chunk_abi.h chunk_crc32.h chunk_crc32.c cemu_def.h

$(BUILD_DIR)/chunk_csf.so: chunck_csf.c $(CSF_COMMON) chunk_gear_v2.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ chunck_csf.c chunk_crc32.c

$(BUILD_DIR)/merge_csf.so: merge_csf.c $(CSF_COMMON)
	$(CC) $(CFLAGS) -shared -fPIC -o $@ merge_csf.c chunk_crc32.c

# 挑战二 / 挑战三：文件同步下沉算子族
$(BUILD_DIR)/collect_oob.so: collect_oob.c cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/rsync_match.so: rsync_match.c rsync_csd_abi.h cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/rsync_patch.so: rsync_patch.c rsync_csd_abi.h cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

$(BUILD_DIR)/dsync_match.so: dsync_match.c dsync_abi.h cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

# 最小验证算子：在 CSD 内部读取文件数据并输出
$(BUILD_DIR)/read_csf.so: read_csf.c cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

# 可组合 CRC：由定长 OOB 槽位拼出任意 512B 对齐区间的校验和
$(BUILD_DIR)/compose_crc.so: compose_crc.c rsync_csd_abi.h cemu_def.h ../../../hw/femu/backend/csd_fp_crc.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ compose_crc.c

# 挑战五：设备内 CDC + 网格拼接比对，直接吐扇区位图
$(BUILD_DIR)/sync_cdc.so: sync_cdc.c cemu_def.h chunk_abi.h chunk_gear_v2.h ../../../hw/femu/backend/csd_fp_crc.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ sync_cdc.c

# 挑战二（双 CSD 版）：client 侧 CDC 切分 -> 块清单
$(BUILD_DIR)/chunk_manifest.so: chunk_manifest.c cemu_def.h cemu_manifest.h chunk_abi.h chunk_gear_v2.h ../../../hw/femu/backend/csd_fp_crc.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ chunk_manifest.c

# 挑战二（双 CSD 版）：server 侧按 512B CRC 网格折叠清单 -> 扇区位图
$(BUILD_DIR)/fold_manifest.so: fold_manifest.c cemu_def.h cemu_manifest.h ../../../hw/femu/backend/csd_fp_crc.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ fold_manifest.c

# 路线甲：两侧各自的 设备内 CDC + 网格组合弱哈希 + 强指纹 -> 块描述符表
$(BUILD_DIR)/cdc_desc.so: cdc_desc.c cemu_def.h dsync_abi.h chunk_abi.h chunk_gear_v2.h \
                          ../../../hw/femu/backend/csd_fp_crc.h ../../../hw/femu/backend/csd_fingerprint.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ cdc_desc.c

$(BUILD_DIR)/vadd.bpf.o: vadd.bpf.c
	clang -target bpf -static -O2 -g -c -o $@ $<

clean:
	rm -rf $(TARGETS)

# skysync-c-CSD client: CDC cut points + weak hash folded from stored grid
$(BUILD_DIR)/chunk_manifest_grid.so: chunk_manifest_grid.c cemu_def.h cemu_manifest.h chunk_abi.h chunk_gear_v2.h ../../../hw/femu/backend/csd_fp_crc.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

# rsync-CSD server: device reads old file data, computes fp256 per 4KB page on the fly
$(BUILD_DIR)/rsync_match_data.so: rsync_match_data.c cemu_def.h rsync_csd_abi.h ../../../hw/femu/backend/csd_fingerprint.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ $<

# grid-cdc：单一算子承载 client CDC / server CDC+比对 / client 补算强哈希三种角色。
# 两侧切点共用同一份循环，改一处两边同步。
$(BUILD_DIR)/grid_ds.so: grid_ds.c cemu_def.h dsync_abi.h chunk_abi.h chunk_gear_v2.h \
                          ../../../hw/femu/backend/csd_fp_crc.h ../../../hw/femu/backend/csd_sha1.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ grid_ds.c

# grid-cdc 流水线版：算法与 grid_ds 一致，改成一条命令并发跑 P 个 chunk
# （nr_concurrent_chunks=P），让闪存读与 CSF 计算重叠。
$(BUILD_DIR)/grid_pipe.so: grid_pipe.c cemu_def.h dsync_abi.h chunk_abi.h chunk_gear_v2.h \
                          ../../../hw/femu/backend/csd_fp_crc.h ../../../hw/femu/backend/csd_sha1.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ grid_pipe.c

# grid-rb：方案 D 的重建算子（destination=1，设备直接写回闪存）
$(BUILD_DIR)/grid_rb.so: grid_rb.c cemu_def.h
	$(CC) $(CFLAGS) -shared -fPIC -o $@ grid_rb.c
