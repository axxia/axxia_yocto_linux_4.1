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
#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

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

static DEFINE_MUTEX(kdi_lock);

#define BH_MSG_MAGIC_LENGTH            4
#define BH_MSG_SEQUENCE_OFFSET         8

static int to_kdi_err(int err)
{

	if (err)
		pr_info("got error: %d\n", err);

	if (err <=0)
		return err;

	/* err > 0: is error from FW */
	switch (err) {
	case BPE_INTERNAL_ERROR:
		return DAL_KDI_STATUS_INTERNAL_ERROR;
	case BPE_INVALID_PARAMS:
	case BHE_INVALID_PARAMS:
		return DAL_KDI_STATUS_INVALID_PARAMS;
	case BHE_INVALID_HANDLE:
		return DAL_KDI_STATUS_INVALID_HANDLE;
	case BPE_NOT_INIT:
		return DAL_KDI_STATUS_NOT_INITIALIZED;
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
	case BHE_UNCAUGHT_EXCEPTION:
		return DAL_KDI_STATUS_UNCAUGHT_EXCEPTION;
	case BHE_WD_TIMEOUT:
		return DAL_KDI_STATUS_WD_TIMEOUT;
	case BHE_APPLET_CRASHED:
		return DAL_KDI_STATUS_APPLET_CRASHED;
	case BHE_TA_PACKAGE_HASH_VERIFY_FAIL:
		return DAL_KDI_STATUS_INVALID_ACP;
	case BHE_PACKAGE_NOT_FOUND:
		return DAL_KDI_STATUS_TA_NOT_FOUND;
	case BHE_PACKAGE_EXIST:
		return DAL_KDI_STATUS_TA_EXIST;
	default:
		return DAL_KDI_STATUS_INTERNAL_ERROR;
	}
}

int kdi_send(unsigned int handle, const unsigned char *buf,
	     size_t len, u64 seq)
{
	enum dal_dev_type mei_device;
	struct dal_device *ddev;
	struct dal_client *dc;
	struct device *dev;
	ssize_t wr;
	int ret;

	mei_device = (enum dal_dev_type)handle;

	if (!buf)
		return -EINVAL;

	if (mei_device < DAL_MEI_DEVICE_IVM || mei_device >= DAL_MEI_DEVICE_MAX)
		return -EINVAL;

	if (!len)
		return 0;

	dev = dal_find_dev(mei_device);
	if (!dev) {
		dev_err(dev, "can't find device\n");
		return -ENODEV;
	}

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];
	if (!dc) {
		dev_err(dev, "client is NULL\n");
		ret = -EFAULT;
		goto out;
	}

	/* copy data to client object */
	memcpy(dc->write_buffer, buf, len);
	wr = dal_write(dc, len, seq);
	if (wr > 0)
		ret = 0;
	else
		ret = wr;
out:
	put_device(dev);
	return ret;
}

int kdi_recv(unsigned int handle, unsigned char *buf, size_t *count)
{
	enum dal_dev_type mei_device;
	struct dal_device *ddev;
	struct dal_client *dc;
	struct device *dev;
	ssize_t ret;
	size_t len;

	mei_device = (enum dal_dev_type)handle;

	if (!buf || !count)
		return -EINVAL;

	if (mei_device < DAL_MEI_DEVICE_IVM || mei_device >= DAL_MEI_DEVICE_MAX)
		return -EINVAL;

	dev = dal_find_dev(mei_device);
	if (!dev)
		return -ENODEV;

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];
	if (!dc) {
		dev_err(dev, "client is NULL\n");
		ret = -EFAULT;
		goto out;
	}

	ret = dal_read(dc);

	if (ret)
		goto out;

	if (kfifo_is_empty(&dc->read_queue)) {
		goto out;
	}

	ret = kfifo_out(&dc->read_queue, &len, sizeof(len));
	if (ret != sizeof(len)) {
		dev_err(&ddev->dev, "could not copy buffer: cannot fetch size");
		ret = -EFAULT;
		goto out;
	}

	if (len > *count) {
		dev_err(&ddev->dev, "could not copy buffer: src size = %zd > dest size = %zd\n",
			len, *count);
		ret = -EMSGSIZE;
		goto out;
	}

	ret = kfifo_out(&dc->read_queue, buf, len);
	if (ret != len) {
		dev_err(&ddev->dev, "could not copy buffer: src size = %zd, dest size = %zd\n",
			len, ret);
		ret = -EFAULT;
	}

	*count = len;
	ret = 0;
out:
	put_device(dev);
	return ret;
}

static int kdi_create_session(u64 *handle, const char *jta_id,
			      const u8 *buffer, size_t buffer_length,
			      const u8 *init_param, size_t init_param_length)
{
	struct ac_ins_jta_pack_ext pack;
	char *ta_pkg;
	int ta_pkg_size;
	int ret;

	if (!jta_id || !buffer || !buffer_length || !handle)
		return -EINVAL;

	/* init_param are optional, but if they exists the length should be
	 * positive and if param buffer is not exists the length must be 0
	 */
	if (!init_param && init_param_length != 0) {
		pr_err("INVALID_PARAMS init_param %p init_param_length %zu",
		       init_param, init_param_length);
		return -EINVAL;
	}

	ret = acp_pload_ins_jta(buffer, buffer_length, &pack);
	if (ret) {
		pr_err("acp_pload_ins_jta() return %d", ret);
		return to_kdi_err(ret);
	}

	ta_pkg = pack.ta_pack;
	if (!ta_pkg)
		return -EINVAL;

	ta_pkg_size = ta_pkg - (char *)buffer;

	if (ta_pkg_size < 0 || (unsigned int)ta_pkg_size > buffer_length)
		return -EINVAL;

	ta_pkg_size = buffer_length - ta_pkg_size;

	ret = bhp_open_ta_session(handle, jta_id, ta_pkg, ta_pkg_size,
				     init_param, init_param_length);

	return to_kdi_err(ret);
}

int dal_create_session(u64 *session_handle,  const char *app_id,
		       const u8 *acp_pkg, size_t acp_pkg_len,
		       const u8 *init_param, size_t init_param_len)
{
	int ret;

	mutex_lock(&kdi_lock);

	ret = kdi_create_session(session_handle, app_id, acp_pkg, acp_pkg_len,
				 init_param, init_param_len);
	if (ret)
		pr_err("kdi_create_session failed = %d\n", ret);

	mutex_unlock(&kdi_lock);

	return to_kdi_err(ret);
}
EXPORT_SYMBOL(dal_create_session);

int dal_send_and_receive(u64 session_handle, int command_id, const u8 *input,
			 size_t input_len, u8 **output, size_t *output_len,
			 int *response_code)
{
	int ret;

	mutex_lock(&kdi_lock);

	ret = bhp_send_and_recv(session_handle, command_id, input, input_len,
				(void **)output, output_len, response_code);

	if (ret)
		pr_err("bhp_send_and_recv failed with status = %d\n", ret);

	mutex_unlock(&kdi_lock);

	return to_kdi_err(ret);
}
EXPORT_SYMBOL(dal_send_and_receive);

int dal_close_session(u64 session_handle)
{
	int ret;

	mutex_lock(&kdi_lock);

	ret = bhp_close_ta_session(session_handle);

	if (ret)
		pr_err("hp_close_ta_session failed = %d\n", ret);

	mutex_unlock(&kdi_lock);

	return to_kdi_err(ret);
}
EXPORT_SYMBOL(dal_close_session);

/**
 * dal_set_exclusive_access - set given uuid exclusive
 *
 * @ta_id: trusted applet id
 *
 * Return:
 *    DAL_KDI_SUCCESS
 *    DAL_KDI_STATUS_INTERNAL_ERROR
 *    DAL_KDI_STATUS_TA_EXIST
 *    DAL_KDI_STATUS_NON_EXCLUSIVENESS_TA;
 */
int dal_set_ta_exclusive_access(uuid_be ta_id)
{
	struct dal_device *ddev;
	struct device *dev;
	struct dal_client *dc;
	int ret;

	mutex_lock(&kdi_lock);

	dev = dal_find_dev(DAL_MEI_DEVICE_IVM);
	if (!dev) {
		dev_err(dev, "can't find device\n");
		ret = -ENODEV;
		goto unlock;
	}

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];

	ret = dal_access_policy_add(ddev, ta_id, dc);

	put_device(dev);
unlock:
	mutex_unlock(&kdi_lock);
	return ret;
}
EXPORT_SYMBOL(dal_set_ta_exclusive_access);

/**
 * dal_unset_ta_exclusive_access - remove exclusiveness from uuid
 *
 * @ta_id: trusted applet id
 *
 * Return:
 */
int dal_unset_ta_exclusive_access(uuid_be ta_id)
{
	struct dal_device *ddev;
	struct device *dev;
	struct dal_client *dc;
	int ret;

	mutex_lock(&kdi_lock);

	dev = dal_find_dev(DAL_MEI_DEVICE_IVM);
	if (!dev) {
		dev_err(dev, "can't find device\n");
		ret = -ENODEV;
		goto unlock;
	}

	ddev = to_dal_device(dev);
	dc = ddev->clients[DAL_INTF_KDI];

	ret = dal_access_policy_remove(ddev, ta_id, dc);

	put_device(dev);
unlock:
	mutex_unlock(&kdi_lock);
	return ret;
}
EXPORT_SYMBOL(dal_unset_ta_exclusive_access);

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

	memset(version_info, 0, sizeof(*version_info));
	snprintf(version_info->version, DAL_VERSION_LEN, "%s", KDI_VERSION);

	return 0;
}
EXPORT_SYMBOL(dal_get_version_info);

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
	.add_dev    = kdi_add_dev,
	.remove_dev = kdi_rm_dev,
};

int dal_kdi_init(void)
{
	int ret;

	ret = bhp_init_internal();
	if (ret) {
		pr_err("bhp_init: failed with status = 0x%x\n", ret);
		return to_kdi_err(ret);
	}

	kdi_interface.class = dal_class;
	ret = class_interface_register(&kdi_interface);
	if (ret) {
		pr_err("failed reister class interface = %d\n", ret);
		goto err;
	}

	return 0;

err:
	bhp_deinit_internal();
	return ret;
}

void dal_kdi_exit(void)
{
	bhp_deinit_internal();
	class_interface_unregister(&kdi_interface);
}
