// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/namei.h>
#include <linux/io_uring.h>
#include <linux/string.h>

#include <uapi/linux/io_uring.h>

#include "io_uring.h"
#include "copy_file_range.h"

ssize_t fdmfs_copy_file_range_kiocb(struct kiocb *kiocb, struct file *file_in,
					loff_t pos_in, struct file *file_out,
					loff_t pos_out, size_t size);

/* borrowed from splice.c */
struct io_copy_file_range {
	struct file			*file_out;
	union {
		struct {
			loff_t				off_out;
			loff_t				off_in;
			u64				len;
			int				copy_fd_in;
			unsigned int			flags;
		};
		struct kiocb kiocb;
	};
};

int io_copy_file_range_prep(struct io_kiocb *req, const struct io_uring_sqe *sqe)
{
	struct io_copy_file_range *cp = io_kiocb_to_cmd(req, struct io_copy_file_range);

	cp->off_in = READ_ONCE(sqe->splice_off_in);
	cp->off_out = READ_ONCE(sqe->off);

	cp->len = READ_ONCE(sqe->len);
	cp->copy_fd_in = READ_ONCE(sqe->splice_fd_in);
	cp->flags = READ_ONCE(sqe->splice_flags);
	return 0;
}

static void copy_complete_iopoll(struct kiocb *kiocb, long res)
{
	struct io_copy_file_range *cp = container_of(kiocb, struct io_copy_file_range, kiocb);
	struct io_kiocb *req = cmd_to_io_kiocb(cp);

	if (unlikely(res != req->cqe.res)) {
		req->cqe.res = res;
	}

	/* order with io_iopoll_complete() checking ->iopoll_completed */
	smp_store_release(&req->iopoll_completed, 1);
}

static void copy_req_complete(struct io_kiocb *req, struct io_tw_state *ts)
{
	io_req_task_complete(req, ts);
}

static void copy_complete(struct kiocb *kiocb, long res)
{
	struct io_copy_file_range *cp = container_of(kiocb, struct io_copy_file_range, kiocb);
	struct io_kiocb *req = cmd_to_io_kiocb(cp);

	if (!kiocb->dio_complete || !(kiocb->ki_flags & IOCB_DIO_CALLER_COMP)) {
		if (unlikely(res != req->cqe.res)) {
			req_set_fail(req);
			req->cqe.res = res;
		}
		io_req_set_res(req, res, 0);
	}
	req->io_task_work.func = copy_req_complete;
	__io_req_task_work_add(req, IOU_F_TWQ_LAZY_WAKE);
}

int io_copy_file_range(struct io_kiocb *req, unsigned int issue_flags)
{
	struct io_copy_file_range *cp = io_kiocb_to_cmd(req, struct io_copy_file_range);
	struct io_ring_ctx *ctx = req->ctx;
	struct file *out = cp->file_out;
	struct file *in;
	ssize_t ret = 0;

	loff_t		off_out = cp->off_out;
	loff_t		off_in = cp->off_in;
	u64		len = cp->len;
	int		copy_fd_in = cp->copy_fd_in;
	unsigned int	flags = cp->flags;

	struct kiocb *kiocb = &cp->kiocb;
	memset(kiocb, 0, sizeof(struct kiocb));
	kiocb->ki_flags |= IOCB_NOWAIT;
	kiocb->dio_complete = NULL;

	req->flags |= REQ_F_NOWAIT;
	if (ctx->flags & IORING_SETUP_IOPOLL) {
		kiocb->private = NULL;
		kiocb->ki_flags |= IOCB_HIPRI;
		kiocb->ki_complete = copy_complete_iopoll;
		req->iopoll_completed = 0;
	} else {
		if (kiocb->ki_flags & IOCB_HIPRI)
			return -EINVAL;
		kiocb->ki_complete = copy_complete;
	}

	kiocb->ki_flags |= IOCB_ALLOC_CACHE;
	kiocb->ki_ioprio = get_current_ioprio();

	if (flags & SPLICE_F_FD_IN_FIXED)
		in = io_file_get_fixed(req, copy_fd_in, issue_flags);
	else
		in = io_file_get_normal(req, copy_fd_in);
	if (!in) {
		ret = -EBADF;
		goto done;
	}

	if (!(req->flags & REQ_F_FIXED_FILE))
		req->flags |= io_file_get_flags(in);
	req->flags |= REQ_F_NOWAIT;

	if (cp->len) {
		ret = fdmfs_copy_file_range_kiocb(kiocb, in, off_in, out, off_out, len);
	}

	if (ret == -EIOCBQUEUED) {
		return IOU_ISSUE_SKIP_COMPLETE;
	}

	if (!(flags & SPLICE_F_FD_IN_FIXED))
		fput(in);
done:
	if (ret != cp->len)
		req_set_fail(req);
	io_req_set_res(req, ret, 0);
	return IOU_OK;
}