/*
 * rsid_reaper.cpp —— 回收泄漏的 memory range set (rsid)
 *
 * 背景：CREATE_MRS 向设备申请 rsid，从不归还会漏；设备池 MAX_RSID=1024。
 * 历史测试进程退出后其 rsid 仍占着设备内存。本工具对每台设备逐 rsid
 * 调 DELETE_MRS（sel=1），把 numr!=0 且 !in_use 的槽位全部回收。
 *
 * 用法: ./build/rsid_reaper
 */

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "util.h"

int main()
{
    for (int dev = 0; dev < 2; dev++) {
        char path[64];
        snprintf(path, sizeof(path), "/dev/nvme%dc3", dev);
        int fd = open(path, O_RDWR);
        if (fd < 0) {
            printf("nvme%d: open %s 失败 (%s)，跳过\n", dev, path, strerror(errno));
            continue;
        }

        /* 1) 逐 rsid 回收 */
        int freed = 0, in_use = 0;
        for (int rsid = 1; rsid <= 1024; rsid++) {
            struct ioctl_create_mrs arg {};
            arg.rsid = (uint16_t)rsid;
            int ret = ioctl(fd, IOCTL_CEMU_DELETE_MRS, &arg);
            if (ret == 0) {
                freed++;
            }
            /* 返回正值是 NVMe 状态码：MR_SET_IN_USE 说明槽位被活程序占着 */
            else if (ret > 0) {
                in_use++;
            }
        }
        printf("nvme%d: 回收 %d 个 rsid, %d 个 in_use 不可回收\n", dev, freed, in_use);

        /* 2) 验证：现在能否 CREATE 成功 */
        char tray[64];
        snprintf(tray, sizeof(tray), "/mnt/fdm%d/1", dev);
        int tfd = open(tray, O_RDWR);
        if (tfd >= 0) {
            struct ioctl_create_mrs mrs {};
            prep_mrs(&mrs, 1);
            mrs.fd[0] = tfd; mrs.off[0] = 0; mrs.size[0] = 512;
            int ret = ioctl(fd, IOCTL_CEMU_CREATE_MRS, &mrs);
            printf("nvme%d: CREATE 验证 %s (ret=%d, rsid=%u)\n",
                   dev, ret == 0 ? "OK" : "FAIL", ret, mrs.rsid);
            if (ret == 0) {
                struct ioctl_create_mrs arg {};
                arg.rsid = mrs.rsid;
                ioctl(fd, IOCTL_CEMU_DELETE_MRS, &arg);
            }
            close(tfd);
        }
        close(fd);
    }
    return 0;
}
