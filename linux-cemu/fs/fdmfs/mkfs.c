#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include "fdmfs.h"

int main(int argc, char *argv[]) {
	if (argc != 2) {
		printf("Usage: mkfs.fdmfs /dev/cemuX\n");
		return 1;
	}

	char *devname = argv[1];
	if (strncmp(devname, "/dev/cemu", 9) != 0) {
		printf("Device to build must be cemu block device! e.g., /dev/cemu0\n");
		return 1;
	}

	int fd = open(devname, O_RDWR);

	struct fdmfs_super_block *sb = malloc(sizeof(struct fdmfs_super_block));
	memset(sb, 0, sizeof(struct fdmfs_super_block));
	sb->magic = FDMFS_MAGIC;

	lseek(fd, FDMFS_SUPER_OFFSET, SEEK_SET);
	write(fd, sb, sizeof(struct fdmfs_super_block));
	close(fd);

	printf("Finished!\n");
	return 0;
}