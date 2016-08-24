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
#ifndef _DAL_KDI_H_
#define _DAL_KDI_H_

#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kfifo.h>

#define DAL_MAX_BUFFER_SIZE            4096

#define DAL_MAX_BUFFER_PER_CLIENT      10 /* TODO: arbitrary number */
#define DAL_CLIENTS_PER_DEVICE         2

/*
 * this array contains  pointers to 3 mei_cl_device, ivm, sdm, rtm.
 * it is initialized during dal_probe and is used by the kernel space kdi
 * to send/recv data to/from mei.
 *
 * this array must be initialized before the kernel space kdi uses it.
 */
extern struct class *dal_class;

/**
 * enum intf_intf - represents dal interface type
 *
 * @DAL_INTF_KDI:  (kdi) kernel space interface
 * @DAL_INTF_CDEV: char device interface
 */
enum dal_intf {
	DAL_INTF_KDI,
	DAL_INTF_CDEV,
};

/** enum dal_notify_action_type:
 *   represents the actions dal_mei inform on
 */
enum dal_notify_action_type {
	DAL_NOTIFY_ACTION_IVM_REGISTERED,
	DAL_NOTIFY_ACTION_SDM_REGISTERED,
	DAL_NOTIFY_ACTION_RTM_REGISTERED,

	DAL_NOTIFY_ACTION_IVM_UNREGISTERED,
	DAL_NOTIFY_ACTION_SDM_UNREGISTERED,
	DAL_NOTIFY_ACTION_RTM_UNREGISTERED,

	DAL_NOTIFY_ACTION_MAX
};

/** enum dal_dev_type:
 *   represents the devices that are exposed to userspace
 *
 * @DAL_MEI_DEVICE_IVM: IVM - Intel/Issuer Virtual Machine
 * @DAL_MEI_DEVICE_SDM: SDM - Security Domain Manager
 * @DAL_MEI_DEVICE_RTM: RTM - Run Time Manager (Launcher)
 * @DAL_MEI_DEVICE_SVM: SVM - Secondary Virtual Machine
 */
enum dal_dev_type {
	DAL_MEI_DEVICE_IVM,
	DAL_MEI_DEVICE_SDM,
	DAL_MEI_DEVICE_RTM,

	DAL_MEI_DEVICE_MAX
};

struct dal_client;

/**
 * struct dal_bh_msg: represent msg sent from the FW.
 *
 * @len: message length
 * @msg: message buffer
 */
struct dal_bh_msg {
	size_t  len;
	char msg[DAL_MAX_BUFFER_SIZE];
};

/**
 * struct dal_device: represents the context for a device,
 *        each device has a context (i.e IVM, SDM, RTM)
 *
 * @cdev: the character device structure.
 * @context_lock:  a lock for synchronizing access to sensitive
 * variables/data structures
 * @write_queue_lock: synchronizing access to write queue
 * structures - for current client in write function
 * @rd_wq: a wait queue, for synchronizing requests in a FIFO manner
 * @clients: the clients on this device ( userspace or kernel ).
 * @num_user_space_clients: ttrack the number of times the device file has
 * been opened
 * @bh_fw_msg: a struct represent msg kdi receive from the FW.
 * @current_write_client: stores the current client being served,
 * (needed since rcv is async, need to know who the received data belongs to)
 * @cldev: the MEI CL device which corresponds to a single DAL FW HECI client
 * @is_device_remove: a variable that signals that the device is removed
 * and waiting threads on queue should wake up.
 * @device_id: saves device type id.
 */
struct dal_device {
	struct device dev;
	struct cdev cdev;
#define DAL_DEV_OPENED 0
	unsigned long status;

	struct mutex context_lock;
	struct mutex write_queue_lock;
	wait_queue_head_t wq;
	struct kfifo write_queue;
	struct dal_client *clients[DAL_CLIENTS_PER_DEVICE];
	struct dal_bh_msg bh_fw_msg;
	struct dal_client *current_write_client;
	struct dal_client *current_read_client;

	struct mei_cl_device *cldev;
	bool is_write_pending;

	bool is_device_removed;

	int device_id;
};

#define to_dal_device(d) container_of(d, struct dal_device, dev)

/**
 * struct dal_client: represents the host client
 *
 * @is_user_space_client:  indicates whether this client is user
 * space or kernel space
 * @read_buffer: buffer containing data received from DAL FW for this client
 * @write_buffer: buffer containing data to send to DAL FW
 * @rcv_callback_queue: a wait queue for synchronizing mei rcv callback and
 * read operations
 * @is_buffer_busy: indicates whether buffer is busy or is it ok to copy
 * data from mei.
 * @read_buffer_size: the amount of data in read_buffer
 * @expected_msg_size_from_fw: the expected msg size from FW
 * @expected_msg_size_to_fw: the expected msg size that will be sent to FW
 * @bytes_rcvd_from_fw: number of bytes that were received from FW
 * @bytes_sent_to_fw: number of bytes that were sent to FW
 * @bytes_sent_to_host: number of bytes that were sent to host
 * @is_another_write_pending: indicates whether this client has another
 * write pending - required to prevent kernel/user space sending
 * interleaving writes
 */
struct dal_client {
	struct dal_device *ddev;
	struct kfifo read_queue;
	char write_buffer[DAL_MAX_BUFFER_SIZE];
	enum dal_intf intf;

	size_t read_buffer_size;

	u64 seq;
	u32 expected_msg_size_from_fw;
	u32 expected_msg_size_to_fw;
	u32 bytes_rcvd_from_fw;
	u32 bytes_sent_to_fw;
	u32 bytes_sent_to_host;
};

ssize_t dal_write(struct dal_client *dc, size_t count, u64 seq);
ssize_t dal_read(struct  dal_client *dc);


struct device *dal_find_dev(enum dal_dev_type device_id);

void dal_dc_print(struct device *dev, struct dal_client *dc);
int dal_dc_setup(struct dal_device *ddev, enum dal_intf intf);
void dal_dc_destroy(struct dal_device *ddev, enum dal_intf intf);

#endif  /* _DAL_KDI_H_ */
