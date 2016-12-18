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

#include "bhp_impl.h"
#include "dal_dev.h"
#include "dal_cdev.h"

/*
 * this class contains the 3 mei_cl_device, ivm, sdm, rtm.
 * it is initialized during dal_probe and is used by the kernel space kdi
 * to send/recv data to/from mei.
 *
 * this class must be initialized before the kernel space kdi uses it.
 */
struct class *dal_class;

/* comperator for cl devices */
static int dal_dev_match(struct device *dev, const void *data)
{
	struct dal_device *ddev;
	const enum dal_dev_type *device_id =
			(enum dal_dev_type *)data;

	ddev = container_of(dev, struct dal_device, dev);

	return ddev->device_id == *device_id;
}

/* find device in dal_class object */
struct device *dal_find_dev(enum dal_dev_type device_id)
{
	return class_find_device(dal_class, NULL, &device_id, dal_dev_match);
}

/* prints client data */
void dal_dc_print(struct device *dev, struct dal_client *dc)
{
	if (!dc) {
		dev_dbg(dev, "dc is null\n");
		return;
	}

	dev_dbg(dev, "client data:\n"
		     "is_user_space_client = %d\n"
		     "read_buffer_size = %zu\n"
		     "expected_msg_size_from_fw = %d\n"
		     "expected_msg_size_to_fw = %d\n"
		     "bytes_rcvd_from_fw = %d\n"
		     "bytes_sent_to_fw = %d\n"
		     "bytes_sent_to_host = %d\n",
		dc->intf,
		dc->read_buffer_size,
		dc->expected_msg_size_from_fw,
		dc->expected_msg_size_to_fw,
		dc->bytes_rcvd_from_fw,
		dc->bytes_sent_to_fw,
		dc->bytes_sent_to_host);
}

/* wait until we can write to MEI,
 * on success will return with the mutex locked
 */
static int dal_wait_for_write(struct dal_device *ddev, struct dal_client *dc)
{
	/*
	 * wait until current write client is null OR
	 * we are the current writer
	 */
	if (wait_event_interruptible(ddev->wq,
				     !ddev->current_write_client ||
				     ddev->current_write_client == dc ||
				     ddev->is_device_removed)) {
		dev_err(&ddev->dev, "wait_for_write(): signal interrupted\n");
		return -ERESTARTSYS;
	}

	/* if the device was removed indicate that to the caller */
	if (ddev->is_device_removed) {
		dev_dbg(&ddev->dev, "wait_for_write(): woke up, device was removed\n");
		return -ENODEV;
	}

	return 0;
}

/* put response msg with error code 'access denied' in client's queue */
static int dal_send_error_access_denied(struct dal_client *dc)
{
	struct dal_device *ddev = dc->ddev;
	struct bhp_response_header res;
	size_t len;
	int ret;

	mutex_lock(&ddev->context_lock);

	bh_prep_access_denied_response(dc->write_buffer, &res);
	len = sizeof(res);

	ret = kfifo_in(&dc->read_queue, &len, sizeof(len));
	ret += kfifo_in(&dc->read_queue, &res, len);
	if (ret < len + sizeof(len)) {
		dev_err(&ddev->dev, "queue is full - access denied MSG THROWN");
		mutex_unlock(&ddev->context_lock);
		return -ENOMEM;
	}

	dev_dbg(&ddev->dev, "calls wake_up_interruptible\n");
	wake_up_interruptible(&ddev->wq);

	mutex_unlock(&ddev->context_lock);

	return 0;
}

static int dal_validate_access(const struct bhp_command_header *hdr,
			       size_t count, void *ctx)
{
	struct dal_client *dc = ctx;
	struct dal_device *ddev = dc->ddev;
	const uuid_be *ta_id;

	if (!bh_msg_is_cmd_open_session(hdr))
		return 0;

	ta_id = bh_open_session_ta_id(hdr, count);
	if (!ta_id)
		return -EINVAL;

	return dal_access_policy_allowed(ddev, *ta_id, dc);
}

static bool bh_is_kdi_hdr(const struct bhp_command_header *hdr)
{
	return hdr->seq >= MSG_SEQ_START_NUMBER;
}

static int dal_validate_seq(const struct bhp_command_header *hdr,
			    size_t count, void *ctx)
{
	struct dal_client *dc = ctx;

	if (dc->intf != DAL_INTF_KDI && bh_is_kdi_hdr(hdr))
		return -EPERM;

	return 0;
}

static const bh_filter_func dal_write_filter_tbl[] = {
	dal_validate_access,
	dal_validate_seq,
	NULL,
};

/* Write BH msg via MEI*/
ssize_t dal_write(struct dal_client *dc, size_t count, u64 seq)
{
	struct dal_device *ddev = dc->ddev;
	struct device *dev;
	ssize_t wr;
	ssize_t ret;
	int status;
	enum dal_intf intf = dc->intf;
	struct dal_client *curr_wc; /* debug */

	dev = &ddev->dev;

	dev_dbg(dev, "client interface %d", intf);
	dal_dc_print(dev, dc);

	/* lock for adding new client that want to write to fifo */
	dev_dbg(dev, "before write_queue_lock - client type %d", intf);
	mutex_lock(&ddev->write_queue_lock);

	/* update client on latest msg seq number*/
	dc->seq = seq;
	dev_dbg(dev, "current_write_client seq = %llu", dc->seq);

	/* put dc in write queue*/
	if (ddev->current_write_client != dc) {
		/* adding client to write queue - this is the first fragment */
		const struct bhp_command_header *hdr;

		hdr = bh_msg_cmd_hdr(dc->write_buffer, count);
		if (!hdr) {
			mutex_unlock(&ddev->write_queue_lock);
			return -EINVAL;
		}

		ret = bh_filter_hdr(hdr, count, dc, dal_write_filter_tbl);
		if (ret == -EPERM) {
			ret = dal_send_error_access_denied(dc);
			ret = ret ?: count;
		}
		if (ret) {
			mutex_unlock(&ddev->write_queue_lock);
			return ret;
		}

		dc->bytes_sent_to_fw = 0;
		dc->expected_msg_size_to_fw = hdr->h.length;
		dev_dbg(dev, "This is first fragment - client type %d, cmd id = %d",
				intf, hdr->id);

		if (kfifo_is_empty(&ddev->write_queue))
			ddev->current_write_client = dc;

		ret = kfifo_in(&ddev->write_queue, dc, sizeof(dc));
		dev_dbg(dev, "kfifo_in returned %zu - client type %d",
			ret, intf);
		dev_dbg(dev, "kfifo_avail = %d",
			kfifo_avail(&ddev->write_queue));
		if (ret < sizeof(dc)) {
			dev_dbg(dev, "queue is full probably a bug");

			mutex_unlock(&ddev->write_queue_lock);
			return -EBUSY;
		}
	}

	dev_dbg(dev, "dal_write_mutex_unlock - client type %d\n", intf);
	dev_dbg(dev, "dal_write(): before wait_for_write - client type %d",
			intf);

	/* wait for current writer to finish his write session */
	mutex_unlock(&ddev->write_queue_lock);
	ret = dal_wait_for_write(ddev, dc);
	if (ret < 0) {
		mutex_lock(&ddev->context_lock);
		goto out;
	}

	dev_dbg(dev, "before mei_cldev_send - client type %d", intf);
	print_hex_dump_bytes("Buffer to send:",
			DUMP_PREFIX_NONE, dc->write_buffer, count);

	/* send msg via MEI */
	wr = mei_cldev_send(ddev->cldev, dc->write_buffer, count);
	if (wr != count) {
		/* TODO: check if dev_err is debug only */
		dev_err(dev, "mei_cl_send() failed, write_bytes != count (%zd != %zu)\n",
			wr, count);
		ret = -EFAULT;
		mutex_lock(&ddev->context_lock);
		goto out;
	}

	dev_dbg(dev, "wrote %zu bytes to fw - client type %d", wr, intf);

	/* lock to prevent write to MEI while reading from MEI */
	/* TODO: check if this lock is needed */
	mutex_lock(&ddev->context_lock);

	/* update client byte sent */
	dc->bytes_sent_to_fw += count;
	ret = wr;

	if (dc->bytes_sent_to_fw != dc->expected_msg_size_to_fw) {
		dev_dbg(dev, "expecting to write more data to FW - client type %d",
				intf);
		goto write_more;
	}

out:
	dev_dbg(&ddev->dev, "removing CURRENT_WRITER\n");
	/* init current to NULL */
	ddev->current_write_client = NULL;
	/* remove current dc from the queue */
	status = kfifo_out(&ddev->write_queue, &curr_wc, sizeof(dc));
	dev_dbg(&ddev->dev, "kfifo_out returned %d\n", status);

	/* set new dal client as current,
	 * if fifo empty current writer wont change
	 */
	status = kfifo_out_peek(&ddev->write_queue,
				&ddev->current_write_client,
				sizeof(dc));
	dev_dbg(&ddev->dev, "kfifo_out_peek returned %d\n", status);

	wake_up_interruptible(&ddev->wq);

write_more:
	mutex_unlock(&ddev->context_lock);

	return ret;
}

/*
 * blocking function, it waits until the caller (dc)
 * will have data on his read_queue
 */
ssize_t dal_read(struct dal_client *dc)
{
	struct dal_device *ddev = dc->ddev;
	struct device *dev = &ddev->dev;

	dal_dc_print(dev, dc);

	dev_dbg(dev, "before wait_for_data_to_read() - client type %d kfifo status %d",
		dc->intf, kfifo_is_empty(&dc->read_queue));

	/* wait until there is data in the read_queue */
	wait_event_interruptible(ddev->wq, !kfifo_is_empty(&dc->read_queue) ||
				 ddev->is_device_removed);

	dev_dbg(dev, "after wait_for_data_to_read() - client type %d",
		dc->intf);

	/* FIXME: use reference counter */
	if (ddev->is_device_removed) {
		dev_dbg(dev, "woke up, device was removed\n");
		return -ENODEV;
	}

	return 0;
}

/**
 * dal_dc_update_read_state - update relevant client state variables
 *      according to the msg received header or payload
 * @dc : dal client
 * @len: received message length
 *
 * Lock: called from 'dal_recv_cb' which is under lock.
 */
static void dal_dc_update_read_state(struct dal_client *dc, ssize_t len)
{
	struct dal_device *ddev = dc->ddev;
	struct transport_msg_header *header =
			(struct transport_msg_header *)dc->ddev->bh_fw_msg.msg;

	/* check BH msg magic, if it exists this is the header */
	if (bh_msg_is_response(ddev->bh_fw_msg.msg, len)) {
		dc->expected_msg_size_from_fw = header->length;
		dev_dbg(&ddev->dev, "expected_msg_size_from_fw = %d bytes read = %zd",
			dc->expected_msg_size_from_fw, len);

		/* clear data from the past. */
		dc->bytes_sent_to_host = 0;
		dc->bytes_rcvd_from_fw = 0;
	}

	/* update number of bytes rcvd */
	dc->bytes_rcvd_from_fw += len;
	dc->read_buffer_size += len;
}

/*
 * get interface (user space OR kernel space) to send the received msg
 */
static enum dal_intf get_client_by_squence_number(struct dal_device *ddev)
{
	struct bhp_response_header *head;

	if (!ddev->clients[DAL_INTF_KDI])
		return DAL_INTF_CDEV;

	head = (struct bhp_response_header *)ddev->bh_fw_msg.msg;

	dev_dbg(&ddev->dev, "msg seq = %llu", head->seq);

	if (head->seq == ddev->clients[DAL_INTF_KDI]->seq)
		return DAL_INTF_KDI;

	return DAL_INTF_CDEV;
}

static void dal_recv_cb(struct mei_cl_device *cldev)
{
	struct dal_device *ddev;
	struct dal_client *dc;
	enum dal_intf intf;
	ssize_t len;
	ssize_t ret;
	bool is_unexpected_msg = false;

	ddev = mei_cldev_get_drvdata(cldev);

	/*
	 * read the msg from MEI
	 */
	len = mei_cldev_recv(cldev, ddev->bh_fw_msg.msg, DAL_MAX_BUFFER_SIZE);
	if (len < 0) {
		dev_err(&cldev->dev, "recv failed %zd\n", len);
		return;
	}

	/*
	 * lock to prevent read from MEI while writing to MEI and to
	 * deal with just one msg at the same time
	 */
	mutex_lock(&ddev->context_lock);

	/* save msg len */
	ddev->bh_fw_msg.len = len;

	/* set to which interface the msg should be sent */
	if (bh_msg_is_response(ddev->bh_fw_msg.msg, len)) {
		intf = get_client_by_squence_number(ddev);
		dev_dbg(&ddev->dev, "recv_cb(): Client set by sequence number");
		dc = ddev->clients[intf];
	} else if (!ddev->current_read_client) {
		intf = DAL_INTF_CDEV;
		dev_dbg(&ddev->dev, "recv_cb(): EXTRA msg received - curr == NULL");
		dc = ddev->clients[intf];
		is_unexpected_msg = true;
	} else {
		dc = ddev->current_read_client;
		dev_dbg(&ddev->dev, "recv_cb(): FRAGMENT msg received - curr != NULL");
	}

	if (!dc) {/* TODO: fix me - why device removed */
		dev_dbg(&ddev->dev, "recv_cb(): dc is null");
		goto out;
	}

	/* save the current read client */
	ddev->current_read_client = dc;
	dev_dbg(&cldev->dev, "read client type %d data from mei client seq =  %llu ",
		dc->intf, dc->seq);

	/*
	 * save new msg in queue,
	 * if the queue is full all new messages will be thrown
	 */
	ret = kfifo_in(&dc->read_queue, &ddev->bh_fw_msg, len + sizeof(len));
	if (ret < len + sizeof(len))
		dev_err(&ddev->dev, "queue is full - MSG THROWN");

	dal_dc_update_read_state(dc, len);

	/*
	 * To clear current client we check if the whole msg received
	 * for the current client
	 */
	if (is_unexpected_msg ||
	    (dc->bytes_rcvd_from_fw == dc->expected_msg_size_from_fw)) {
		dev_dbg(&ddev->dev, "recv_cb(): setting CURRENT_READER to NULL\n");
		ddev->current_read_client = NULL;
	}
	/* wake up all clients waiting for read or write */
	wake_up_interruptible(&ddev->wq);

out:
	mutex_unlock(&ddev->context_lock);
	dev_dbg(&cldev->dev, "recv_cb(): unlock\n");
}

void dal_dc_destroy(struct dal_device *ddev, enum dal_intf intf)
{
	struct dal_client *dc;

	dc = ddev->clients[intf];
	if (!dc)
		return;

	kfifo_free(&dc->read_queue);
	kfree(dc);
	ddev->clients[intf] = NULL;
}

int dal_dc_setup(struct dal_device *ddev, enum dal_intf intf)
{
	int ret;
	struct dal_client *dc;
	size_t readq_sz = DAL_MAX_BUFFER_PER_CLIENT * sizeof(struct dal_bh_msg);

	if (ddev->clients[intf]) {
		dev_err(&ddev->dev, "client already set\n");
		return -EINVAL;
	}

	dc = kzalloc(sizeof(*dc), GFP_KERNEL);
	if (!dc)
		return  -ENOMEM;

	ret = kfifo_alloc(&dc->read_queue, readq_sz, GFP_KERNEL);
	if (ret) {
		kfree(dc);
		return ret;
	}

	dc->intf = intf;
	dc->ddev = ddev;
	ddev->clients[intf] = dc;
	return 0;
}

/* FIXME: should be under lock ? */
static int dal_remove(struct mei_cl_device *cldev)
{
	struct dal_device *ddev = mei_cldev_get_drvdata(cldev);

	dal_dev_del(ddev);

	ddev->is_device_removed = true;
	/* make sure the above is set */
	smp_mb();
	/* wakeup write waiters so we can unload */
	if (waitqueue_active(&ddev->wq))
		wake_up_interruptible(&ddev->wq);

	kfifo_free(&ddev->write_queue);

	device_del(&ddev->dev);

	mei_cldev_set_drvdata(cldev, NULL);

	mei_cldev_disable(cldev);

	put_device(&ddev->dev);

	return 0;
}

static void dal_class_release(struct device *dev)
{
	struct dal_device *ddev = to_dal_device(dev);

	dal_access_list_free(ddev);

	kfree(ddev);
}

static int dal_probe(struct mei_cl_device *cldev,
		     const struct mei_cl_device_id *id)
{
	struct dal_device *ddev;
	int ret;

	ddev = kzalloc(sizeof(*ddev), GFP_KERNEL);
	if (!ddev)
		return -ENOMEM;

	/* initialize the mutex and wait queue */
	mutex_init(&ddev->context_lock);
	mutex_init(&ddev->write_queue_lock);
	init_waitqueue_head(&ddev->wq);
	ddev->cldev = cldev;
	ddev->device_id = id->driver_info;

	ret = mei_cldev_enable(cldev);
	if (ret < 0) {
		dev_err(&cldev->dev, "mei_cl_enable_device() failed with ret = %d\n",
			ret);
		goto free_context;
	}

	ret = kfifo_alloc(&ddev->write_queue,
			  sizeof(struct dal_client *) * DAL_CLIENTS_PER_DEVICE,
			  GFP_KERNEL);
	if (ret != 0)
		goto free_context;

	/* save pointer to the context in the device */
	mei_cldev_set_drvdata(cldev, ddev);

	/* register to mei bus callbacks */
	ret = mei_cldev_register_rx_cb(cldev, dal_recv_cb);
	if (ret) {
		dev_err(&cldev->dev, "mei_cl_register_event_cb() failed ret = %d\n",
			ret);
		goto disable_cldev;
	}

	ddev->dev.parent = &cldev->dev;
	ddev->dev.class  = dal_class;
	ddev->dev.release = dal_class_release;
	dev_set_name(&ddev->dev, "dal%d", ddev->device_id);

	dal_dev_setup(ddev);

	if (ddev->device_id == DAL_MEI_DEVICE_IVM) {
		ret = dal_access_list_init(ddev);
		if (ret) {
			dev_err(&cldev->dev, "failed to init access list\n");
			goto err_dev_create;
		}
	}

	ret = device_register(&ddev->dev);
	if (ret) {
		dev_err(&cldev->dev, "unable to register device\n");
		goto err_dev_create;
	}

	ret = dal_dev_add(ddev);
	if (ret)
		goto err_dev_create;

	return 0;

err_dev_create:
	dal_dev_del(ddev);

disable_cldev:
	mei_cldev_disable(cldev);
	dal_remove(cldev);
	kfifo_free(&ddev->write_queue);

free_context:
	kfree(ddev);

	return ret;
}

/* DAL FW HECI client GUIDs */
#define IVM_UUID UUID_LE(0x3c4852d6, 0xd47b, 0x4f46, \
			 0xb0, 0x5e, 0xb5, 0xed, 0xc1, 0xaa, 0x44, 0x0e)
#define SDM_UUID UUID_LE(0xdba4d603, 0xd7ed, 0x4931, \
			 0x88, 0x23, 0x17, 0xad, 0x58, 0x57, 0x05, 0xd5)
#define RTM_UUID UUID_LE(0x5565a099, 0x7fe2, 0x45c1, \
			 0xa2, 0x2b, 0xd7, 0xe9, 0xdf, 0xea, 0x9a, 0x2e)

#define DAL_DEV_ID(__uuid, __device_type) \
	{.uuid = __uuid,                  \
	 .version = MEI_CL_VERSION_ANY,   \
	 .driver_info = __device_type}

static const struct mei_cl_device_id dal_device_id[] = {
	DAL_DEV_ID(IVM_UUID, DAL_MEI_DEVICE_IVM),
	DAL_DEV_ID(SDM_UUID, DAL_MEI_DEVICE_SDM),
	DAL_DEV_ID(RTM_UUID, DAL_MEI_DEVICE_RTM),
	/* required last entry */
	{ }
};
MODULE_DEVICE_TABLE(mei, dal_device_id);

static struct mei_cl_driver dal_driver = {
	.id_table = dal_device_id,
	.name = KBUILD_MODNAME,

	.probe  = dal_probe,
	.remove = dal_remove,
};

static void __exit mei_dal_exit(void)
{
	pr_info("Kernel DAL Interface shutdown\n");

	mei_cldev_driver_unregister(&dal_driver);

	dal_dev_exit();

	dal_kdi_exit();

	class_destroy(dal_class);
}

static int __init mei_dal_init(void)
{
	int ret;

	dal_class = class_create(THIS_MODULE, "dal");
	if (IS_ERR(dal_class)) {
		pr_err("couldn't create class\n");
		return PTR_ERR(dal_class);
	}

	ret = dal_dev_init();
	if (ret < 0)
		goto err_class;

	ret = dal_kdi_init();
	if (ret)
		goto err_dev;

	ret = mei_cldev_driver_register(&dal_driver);
	if (ret < 0) {
		pr_err("mei_cl_driver_register failed with status = %d\n", ret);
		goto err;
	}

	return 0;

err:
	dal_kdi_exit();
err_dev:
	dal_dev_exit();
err_class:
	class_destroy(dal_class);
	return ret;
}

module_init(mei_dal_init);
module_exit(mei_dal_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) MEI Dynamic Application Loader (DAL)");
MODULE_LICENSE("GPL v2");
