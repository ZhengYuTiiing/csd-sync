#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "fdmfs.h"
#include "../../drivers/nvme/host/cemu.h"

// file.c
static struct super_operations fdmfs_super_ops = {
	.statfs		= simple_statfs,
	.drop_inode	= generic_delete_inode,
};

static struct inode *fdmfs_get_inode(struct super_block *sb,
		const struct inode *dir, umode_t mode) {
	struct timespec64 ts;
	struct inode *ino = new_inode(sb);

	if (!ino)
		return ERR_PTR(-ENOMEM);

	ino->i_ino = get_next_ino();
	ino->i_mode = mode;
	inode_init_owner(&nop_mnt_idmap, ino, dir, mode);

	ts = current_time(ino);
	inode_set_mtime_to_ts(ino, ts);
	inode_set_atime_to_ts(ino, ts);
	inode_set_ctime_to_ts(ino, ts);

	switch (mode & S_IFMT) {
	case S_IFDIR:
		ino->i_op = &fdmfs_inode_ops;
		ino->i_fop = &fdmfs_fops;
		inc_nlink(ino);
		break;
	case S_IFREG:
		ino->i_op = &fdmfs_inode_ops;
		ino->i_fop = &fdmfs_fops;
		break;
	default:
		pr_err("FDMFS: unknown inode type\n");
		return ERR_PTR(-EINVAL);
	}

	return ino;
}

static int read_raw_super_block(struct fdmfs_sb_info *sbi)
{
	struct super_block *sb = sbi->sb;
	struct buffer_head *bh;
	struct fdmfs_super_block *super;
	int err = 0;

	super = kzalloc(sizeof(struct fdmfs_super_block), GFP_KERNEL);
	if (!super)
		return -ENOMEM;

	// read superblock from disk
	bh = sb_bread(sb, 0);
	if (!bh) {
		pr_err("FDMFS: Unable to read superblock\n");
		err = -EIO;
		goto free_super;
	}
	memcpy(super, bh->b_data, sizeof(*super));
	brelse(bh);

	// check magic number
	if (super->magic != sb->s_magic) {
		pr_err("FDMFS: Wrong magic number\n");
		err = -EINVAL;
		goto free_super;
	}

	sbi->raw_super = super;
	return 0;

free_super:
	kfree(super);
	return err;
}

/*
 * FDMFS is a temporary file system, the data is not persistent.
 */
static int fdmfs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct inode *root_inode;
	struct fdmfs_inode *root;
	struct fdmfs_sb_info *sbi;
	struct fdm_region *region;
	int err;

	sbi = kzalloc(sizeof(struct fdmfs_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;

	sbi->sb = sb;
	sbi->root_ino = 3;
	sbi->ino_count = 3;
	sbi->cemu_dev = sb->s_dev;
	sbi->cemu_bdev = blkdev_get_no_open(sbi->cemu_dev);
	sbi->fdm_size = cemu_dev_get_size(sbi->cemu_bdev);
	sbi->fdm_addr = cemu_dev_get_p2p_addr(sbi->cemu_bdev);

	pr_info("FDMFS: mount... fdm_size %lu\n", sbi->fdm_size);

	// add free region
	INIT_LIST_HEAD(&sbi->fdm_free);
	INIT_LIST_HEAD(&sbi->fdm_used);
	region = kzalloc(sizeof(struct fdm_region), GFP_KERNEL);
	region->inode = NULL;
	region->off = 0;
	region->size = sbi->fdm_size;
	list_add(&region->list, &sbi->fdm_free);

	if (unlikely(!sb_set_blocksize(sb, FDMFS_BLKSIZE))) {
		printk(KERN_ERR "FDMFS: unable to set blocksize\n");
	}

	err = read_raw_super_block(sbi);
	if (err)
		return err;

	sb->s_op = &fdmfs_super_ops;
	sb->s_magic = FDMFS_MAGIC;
	sb->s_time_gran = 1;
	// memcpy(&sb->s_uuid, raw_super->uuid, sizeof(raw_super->uuid));
	sb->s_maxbytes = MAX_LFS_FILESIZE;
	sb->s_fs_info = sbi;

	// alloc root inode
	root_inode = fdmfs_icreate(sbi, &nop_mnt_idmap, NULL, S_IFDIR | 0755, "/");
	if (IS_ERR(root_inode))
		return PTR_ERR(root_inode);

	// set root inode
	root = root_inode->i_private;
	sbi->root_inode = root;
	sb->s_root = d_make_root(root_inode);
	if (!sb->s_root)
		return -ENOMEM;

	return 0;
}

static struct dentry *fdmfs_mount(struct file_system_type *fs_type, int flags,
			const char *dev_name, void *data)
{
	return mount_bdev(fs_type, flags, dev_name, data, fdmfs_fill_super);
}

static void fdmfs_kill_sb(struct super_block *sb)
{
}

static struct file_system_type fdmfs_type = {
	.owner		= THIS_MODULE,
	.name		= "fdmfs",
	.mount		= fdmfs_mount,
	.kill_sb	= fdmfs_kill_sb,
	.fs_flags	= FS_REQUIRES_DEV,
};

static int __init fdmfs_init(void) {
	register_filesystem(&fdmfs_type);
	pr_info("FDMFS: Module loaded.\n");
	return 0;
}

static void __exit fdmfs_exit(void) {
	unregister_filesystem(&fdmfs_type);
	pr_info("FDMFS: Module unloaded.\n");
}

module_init(fdmfs_init);
module_exit(fdmfs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Qiuyang Zhang");
MODULE_DESCRIPTION("CEMU CSD FDM FS");
