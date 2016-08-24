/******************************************************************************
 * Intel mei_dal Linux driver
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *	Intel Corporation.
 *	linux-mei@linux.intel.com
 *	http://www.intel.com
 *
 * BSD LICENSE
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/mei_cl_bus.h>
#include <linux/notifier.h>
#include <linux/dal.h>

#include "dal_dev.h"
#include "dal_cdev.h"

/* KDI user space devices major and minor numbers */
static dev_t dal_devt;

static int dal_dev_open(struct inode *inode, struct file *fp)
{
	int ret;
	struct dal_device *ddev;

	ddev = container_of(inode->i_cdev, struct dal_device, cdev);
	if (!ddev)
		return -ENODEV;

	/* single open */
	if (test_and_set_bit(DAL_DEV_OPENED, &ddev->status))
		return -EBUSY;

	ret = dal_dc_setup(ddev, DAL_INTF_USER_SPACE);
	if (ret)
		goto err;

	fp->private_data = ddev->clients[DAL_INTF_USER_SPACE];

	return nonseekable_open(inode, fp);

err:
	clear_bit(DAL_DEV_OPENED, &ddev->status);
	return ret;
}

static int dal_dev_release(struct inode *inode, struct file *fp)
{
	struct dal_client *dc = fp->private_data;
	struct dal_device *ddev = dc->ddev;

	if (mutex_lock_interruptible(&ddev->context_lock)) {
		dev_err(&ddev->dev, "signal interrupted");
		return -ERESTARTSYS;
	}

	dev_dbg(&ddev->dev, "free user space client");
	ddev->clients[dc->intf] = NULL;

	kfifo_free(&dc->read_queue);
	kfree(dc);

	mutex_unlock(&ddev->context_lock);

	clear_bit(DAL_DEV_OPENED, &ddev->status);

	return 0;
}

/* This is only for tmp read data to client */
static struct dal_bh_msg bh_msg[DAL_MEI_DEVICE_MAX];
#define BH_MSG_HEADER_LENGTH           32
static ssize_t dal_dev_read(struct file *fp, char __user *buff,
			    size_t count, loff_t *offp)
{
	struct dal_client *dc = fp->private_data;
	struct dal_device *ddev = dc->ddev;
	ssize_t ret;

	ret = dal_read(dc);

	if (ret != 0)
		return ret;

	if (kfifo_is_empty(&dc->read_queue))
		return 0;

	ret = kfifo_out(&dc->read_queue, &bh_msg[ddev->device_id],
			sizeof(struct dal_bh_msg));
	dev_dbg(&ddev->dev, "kfifo_out() ret = %zd\n", ret);

	if (bh_msg[ddev->device_id].len > count) {
		dev_dbg(&ddev->dev, "could not copy buffer: src size = %zd, dest size = %zu\n",
			bh_msg[ddev->device_id].len, count);
		return -EFAULT;
	}

	if (copy_to_user(buff, bh_msg[ddev->device_id].msg,
			 bh_msg[ddev->device_id].len)) {
		dev_dbg(&ddev->dev, "copy_to_user() failed\n");
		return -EFAULT;
	}

	return bh_msg[ddev->device_id].len;
}

static ssize_t dal_dev_write(struct file *fp, const char __user *buff,
			     size_t count, loff_t *offp)
{
	struct dal_device *ddev;
	struct dal_client *dc = fp->private_data;

	ddev = dc->ddev;

	if (count > DAL_MAX_BUFFER_SIZE) {
		dev_err(&ddev->dev, "count is too big, count = %zu\n", count);
		return -EMSGSIZE;
	}

	if (count == 0)
		return 0;

	if (!buff)
		return -EINVAL;

	if (copy_from_user(dc->write_buffer, buff, count))
		return -EFAULT;

	return dal_write(dc, count, 0);
}

static const struct file_operations mei_dal_fops = {
	.owner    = THIS_MODULE,
	.open     = dal_dev_open,
	.release  = dal_dev_release,
	.read     = dal_dev_read,
	.write    = dal_dev_write,
	.llseek   = no_llseek,
};

void dal_dev_del(struct dal_device *ddev)
{
	cdev_del(&ddev->cdev);
}

void dal_dev_setup(struct dal_device *ddev)
{
	dev_t devno;

	cdev_init(&ddev->cdev, &mei_dal_fops);
	devno = MKDEV(MAJOR(dal_devt), ddev->device_id);
	ddev->cdev.owner = THIS_MODULE;
	ddev->dev.devt = devno;
	ddev->cdev.kobj.parent = &ddev->dev.kobj;
}

int dal_dev_add(struct dal_device *ddev)
{
	return cdev_add(&ddev->cdev, ddev->dev.devt, 1);
}

int __init dal_dev_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&dal_devt, 0, DAL_MEI_DEVICE_MAX, "dal");
	if (ret < 0)
		pr_err("failed allocate chrdev region = %d\n", ret);

	return ret;
}

void dal_dev_exit(void)
{
	unregister_chrdev_region(dal_devt, DAL_MEI_DEVICE_MAX);
}
