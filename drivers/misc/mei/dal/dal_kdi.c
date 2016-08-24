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

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/mei_cl_bus.h>
#include <linux/mei.h>
#include <linux/sched.h>
#include <linux/dal.h>

#include "bhp_exp.h"
#include "bhp_impl.h"
#include "dal_dev.h"

static atomic_t kdi_ref_count = ATOMIC_INIT(0);
static DEFINE_MUTEX(kdi_lock);

#define BH_MSG_MAGIC_LENGTH            4
#define BH_MSG_SEQUENCE_OFFSET         8

static int bh_err_to_kdi_err(int bh_err)
{
	switch (bh_err) {
	case BH_SUCCESS:
		return DAL_KDI_SUCCESS;
	case BPE_INTERNAL_ERROR:
		return DAL_KDI_STATUS_INTERNAL_ERROR;
	case BPE_INVALID_PARAMS:
	case BHE_INVALID_PARAMS:
		return DAL_KDI_STATUS_INVALID_PARAMS;
	case BHE_INVALID_HANDLE:
		return DAL_KDI_STATUS_INVALID_HANDLE;
	case BPE_NOT_INIT:
		return DAL_KDI_STATUS_NOT_INITIALIZED;
	case BPE_NO_CONNECTION_TO_FIRMWARE:
		return DAL_KDI_STATUS_NO_FW_CONNECTION;
	case BPE_OUT_OF_MEMORY:
	case BHE_OUT_OF_MEMORY:
		return DAL_KDI_STATUS_OUT_OF_MEMORY;
	case BHE_INSUFFICIENT_BUFFER:
	case BHE_APPLET_SMALL_BUFFER:
		return DAL_KDI_STATUS_BUFFER_TOO_SMALL;
	case BPE_OUT_OF_RESOURCE:
		return DAL_KDI_STATUS_OUT_OF_RESOURCE;
	case BHE_SESSION_NUM_EXCEED:
		return DAL_KDI_STATUS_MAX_SESSIONS_REACHED;
	default:
		return DAL_KDI_STATUS_INTERNAL_ERROR;
	}
}

static int kdi_add_dev(struct device *dev,
		       struct class_interface *class_intf)
{
	int ret;
	struct dal_device *ddev;

	ddev = to_dal_device(dev);
	mutex_lock(&ddev->context_lock);
	ret = dal_dc_setup(ddev, DAL_INTF_KDI);
	mutex_unlock(&ddev->context_lock);
	return ret;
}

static void kdi_rm_dev(struct device *dev,
		       struct class_interface *class_intf)
{
	struct dal_device *ddev;

	ddev = to_dal_device(dev);
	mutex_lock(&ddev->context_lock);
	dal_dc_destroy(ddev, DAL_INTF_KDI);
	mutex_unlock(&ddev->context_lock);
}

static struct class_interface kdi_interface __refdata = {
	.add_dev        = kdi_add_dev,
	.remove_dev     = kdi_rm_dev,
};

static int kdi_send(unsigned int handle,
		    unsigned char *buf, unsigned int len, u64 seq)
{
	enum dal_dev_type mei_device;
	struct dal_device *ddev;
	struct dal_client *dc;
	struct device *dev;
	ssize_t ret;

	mei_device = (enum dal_dev_type)handle;

	if (!buf)
		return BPE_INVALID_PARAMS;

	if (mei_device < DAL_MEI_DEVICE_IVM || mei_device >= DAL_MEI_DEVICE_MAX)
		return BPE_INVALID_PARAMS;

	if (!len)
		return BH_SUCCESS;

	dev = dal_find_dev(mei_device);
	if (!dev) {
		dev_err(dev, "can't find device\n");
		return BPE_INTERNAL_ERROR;
	}

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];
	if (!dc) {
		dev_err(dev, "client is NULL\n");
		ret = BPE_INTERNAL_ERROR;
		goto out;
	}

	/* copy data to client object */
	memcpy(dc->write_buffer, buf, len);
	ret = dal_write(dc, len, seq);

	if (ret <= 0)
		ret = BPE_COMMS_ERROR;
	else
		ret = BH_SUCCESS;

out:
	put_device(dev);
	return ret;
}

static int kdi_recv(unsigned int handle,
		    unsigned char *buf, unsigned int *count)
{
	enum dal_dev_type mei_device;
	struct dal_device *ddev;
	struct dal_client *dc;
	struct device *dev;
	ssize_t ret;
	size_t len;

	mei_device = (enum dal_dev_type)handle;

	if (!buf || !count)
		return BPE_INVALID_PARAMS;

	if (mei_device < DAL_MEI_DEVICE_IVM || mei_device >= DAL_MEI_DEVICE_MAX)
		return BPE_INVALID_PARAMS;

	dev = dal_find_dev(mei_device);
	if (!dev)
		return BPE_INTERNAL_ERROR;

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];
	if (!dc) {
		dev_err(dev, "client is NULL\n");
		ret = BPE_INTERNAL_ERROR;
		goto out;
	}

	ret = dal_read(dc);

	if (ret != 0)
		goto out;

	if (kfifo_is_empty(&dc->read_queue)) {
		ret = 0;
		goto out;
	}

	ret = kfifo_out(&dc->read_queue, &len, sizeof(len));
	if (ret != sizeof(len)) {
		dev_err(&ddev->dev, "could not copy buffer: cannot fetch size");
		ret = BPE_COMMS_ERROR;
		goto out;
	}

	if (len > *count) {
		dev_err(&ddev->dev, "could not copy buffer: src size = %zd > dest size = %u\n",
			len, *count);
		ret = BPE_COMMS_ERROR;
		goto out;
	}

	ret = kfifo_out(&dc->read_queue, buf, len);
	if (ret != len) {
		dev_err(&ddev->dev, "could not copy buffer: src size = %zd, dest size = %zd\n",
			len, ret);
		ret = BPE_COMMS_ERROR;
	}

	*count = len;
	ret = BH_SUCCESS;
out:
	put_device(dev);
	return ret;
}

static struct bhp_transport kdi_transport = {
	.send = kdi_send,
	.recv = kdi_recv,
};

static int kdi_create_session(u64 *handle, const char *jta_id,
			      const u8 *buffer, size_t buffer_length,
			      const u8 *init_param, size_t init_param_length)
{
	struct ac_ins_jta_pack_ext pack;
	char *ta_pkg;
	int ta_pkg_size;
	int ret, bh_err;

	if (!jta_id || !buffer || !buffer_length || !handle)
		return DAL_KDI_STATUS_INVALID_PARAMS;

	/* init_param are optional, but if they exists the length should be
	 * positive and if param buffer is not exists the length must be 0
	 */
	if (!init_param && init_param_length != 0) {
		pr_err("INVALID_PARAMS init_param %p init_param_length %zu",
		       init_param, init_param_length);
		return DAL_KDI_STATUS_INVALID_PARAMS;
	}

	bh_err = acp_pload_ins_jta(buffer, buffer_length, &pack);
	ret = bh_err_to_kdi_err(bh_err);
	if (ret) {
		pr_err("acp_pload_ins_jta() return %d", bh_err);
		return ret;
	}

	ta_pkg = pack.ta_pack;
	if (!ta_pkg)
		return DAL_KDI_STATUS_INTERNAL_ERROR;

	ta_pkg_size = ta_pkg - (char *)buffer;

	if (ta_pkg_size < 0 || (unsigned int)ta_pkg_size > buffer_length)
		return DAL_KDI_STATUS_INTERNAL_ERROR;

	ta_pkg_size = buffer_length - ta_pkg_size;

	bh_err = bhp_open_ta_session(handle, jta_id, ta_pkg, ta_pkg_size,
				     init_param, init_param_length);

	return bh_err_to_kdi_err(bh_err);
}

static inline bool kdi_check_handle(u64 handle)
{
	return (handle == (u64)dal_class);
}

int kdi_init(u32 flags, u64 *handle)
{
	int ret, bh_err;

	if (!handle)
		return DAL_KDI_STATUS_INVALID_PARAMS;

	if (atomic_inc_return(&kdi_ref_count) > 1)
		goto out;

	kdi_interface.class = dal_class;
	ret = class_interface_register(&kdi_interface);
	if (ret)
		return DAL_KDI_STATUS_INTERNAL_ERROR;

	bh_err = bhp_init_internal(&kdi_transport);
	ret = bh_err_to_kdi_err(bh_err);
	if (bh_err) {
		class_interface_unregister(&kdi_interface);
		pr_err("BHP_Init failed with status = %d\n", bh_err);
		return ret;
	}
out:

	*handle = (u64)dal_class;
	return DAL_KDI_SUCCESS;
}
EXPORT_SYMBOL(kdi_init);

int kdi_deinit(u64 handle)
{
	int ret, bh_err;

	/* check handle first */
	if (!kdi_check_handle(handle))
		return DAL_KDI_STATUS_INVALID_HANDLE;

	ret = DAL_KDI_SUCCESS;
	if (atomic_dec_if_positive(&kdi_ref_count) == 0) {
		class_interface_unregister(&kdi_interface);
		bh_err = bhp_deinit_internal();
		ret = bh_err_to_kdi_err(bh_err);
		if (bh_err)
			pr_warn("bhp_deinit_internal failed: = %d\n", bh_err);
	}

	return ret;
}
EXPORT_SYMBOL(kdi_deinit);

int dal_create_session(u64 handle,
		       u64 *session_handle,
		       const char *app_id,
		       const u8 *acp_pkg,
		       size_t acp_pkg_len,
		       const u8 *init_param,
		       size_t init_param_len)
{
	int ret;

	if (atomic_read(&kdi_ref_count) == 0)
		return DAL_KDI_STATUS_NOT_INITIALIZED;

	if (!kdi_check_handle(handle))
		return DAL_KDI_STATUS_INVALID_HANDLE;

	mutex_lock(&kdi_lock);

	ret = kdi_create_session(session_handle, app_id,
				 acp_pkg, acp_pkg_len,
				 init_param, init_param_len);
	if (ret)
		pr_err("kdi_create_session failed = %d\n", ret);

	mutex_unlock(&kdi_lock);

	return ret;
}
EXPORT_SYMBOL(dal_create_session);

int dal_send_and_receive(u64 handle,
			 u64 session_handle,
			 int command_id,
			 const u8 *input,
			 size_t input_len,
			 u8 **output,
			 size_t *output_len,
			 int *response_code)
{
	int ret, bh_err;

	if (atomic_read(&kdi_ref_count) == 0)
		return DAL_KDI_STATUS_NOT_INITIALIZED;

	if (!kdi_check_handle(handle))
		return DAL_KDI_STATUS_INVALID_HANDLE;

	mutex_lock(&kdi_lock);

	bh_err = bhp_send_and_recv(session_handle, command_id, input, input_len,
				   (void **)output, output_len, response_code);

	if (bh_err)
		pr_err("bhp_send_and_recv failed with status = %d\n", bh_err);

	ret = bh_err_to_kdi_err(bh_err);

	mutex_unlock(&kdi_lock);

	return ret;
}
EXPORT_SYMBOL(dal_send_and_receive);

int dal_close_session(u64 handle, u64 session_handle)
{
	int ret, bh_err;

	if (atomic_read(&kdi_ref_count) == 0)
		return DAL_KDI_STATUS_NOT_INITIALIZED;

	if (!kdi_check_handle(handle))
		return DAL_KDI_STATUS_INVALID_HANDLE;

	mutex_lock(&kdi_lock);

	bh_err = bhp_close_ta_session(session_handle);

	if (bh_err)
		pr_err("hp_close_ta_session failed = %d\n", bh_err);

	ret = bh_err_to_kdi_err(bh_err);

	mutex_unlock(&kdi_lock);

	return ret;
}
EXPORT_SYMBOL(dal_close_session);

#define KDI_MAJOR_VER         "1"
#define KDI_MINOR_VER         "0"
#define KDI_HOTFIX_VER        "0"

#define KDI_VERSION KDI_MAJOR_VER "." \
		    KDI_MINOR_VER "." \
		    KDI_HOTFIX_VER

int dal_get_version_info(struct dal_version_info *version_info)
{
	if (!version_info)
		return DAL_KDI_STATUS_INVALID_PARAMS;

	memset(version_info, 0x00, sizeof(*version_info));
	snprintf(version_info->version, DAL_VERSION_LEN, "%s", KDI_VERSION);

	return 0;
}
EXPORT_SYMBOL(dal_get_version_info);
