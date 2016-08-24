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
static struct mutex kdi_lock;

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

static bool kdi_check_handle(u64 handle)
{
	return (handle == (u64)dal_class);
}

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

static bool kdi_is_mei_ready(void)
{
	struct device *dev;
	int i;

	/* TODO: don't use loop here there is
	 * already an iterator  over class list
	 */
	for (i = 0; i < DAL_MEI_DEVICE_MAX; ++i) {
		dev = dal_find_dev(i);
		if (!dev || !dev->parent)
			return false;

		put_device(dev);
	}

	return true;
}

static int kdi_create_kernel_clients(void)
{
	struct device *dev;
	struct dal_device *ddev;
	int i;
	int ret;

	/* FIXME: use an iterrator */
	/* Check that all the device are init */
	for (i = 0; i < DAL_MEI_DEVICE_MAX; ++i) {
		dev = dal_find_dev(i);
		if (!dev || !dev->parent) {
			pr_err("device=%d is NULL\n", i);
			return -EFAULT;
		}
		ddev = to_dal_device(dev);
		ret = dal_dc_setup(ddev, DAL_INTF_KDI);
		put_device(dev);
		if (ret)
			return ret;
	}

	return 0;
}

static bool kdi_is_init_done(void)
{
	return (atomic_read(&kdi_ref_count) >= 1);
}

/* FIMXE: not sure this is needed at all
 * this will be destroyed using class release function
 */

static void kdi_destroy_kernel_clients(void)
{
	struct device *dev;
	struct dal_device *ddev;
	struct dal_client *dc;
	int i;

	/* use iterator */
	for (i = 0; i < DAL_MEI_DEVICE_MAX; ++i) {
		dev = dal_find_dev(i);
		if (!dev || !dev->parent)
			continue;
		/* TODO: just call destroy kdic ...  */
		ddev = to_dal_device(dev);
		dev_dbg(&ddev->dev, "kdi_destroy_kernel_clients(): free kernel space client");
		dc = ddev->clients[DAL_INTF_KDI];
		kfifo_free(&dc->read_queue);
		kfree(dc);
		ddev->clients[DAL_INTF_KDI] = NULL;
		put_device(dev);
	}
}

int kdi_init(u32 flags, u64 *handle)
{
	int ret, bh_err;

	if (!handle)
		return DAL_KDI_STATUS_INVALID_PARAMS;

	/* Check that all the device are ready */
	if (!kdi_is_mei_ready()) {
		pr_err("dal_init(): mei devices was not initialized\n");
		ret = DAL_KDI_STATUS_NOT_READY;
		*handle = DAL_KDI_INVALID_HANDLE;
		goto end;
	}

	ret = DAL_KDI_SUCCESS;
	if (atomic_inc_return(&kdi_ref_count) == 1) {
		mutex_init(&kdi_lock);
		pr_debug("ref count == 1, performing init\n");
		if (kdi_create_kernel_clients() < 0) {
			kdi_destroy_kernel_clients();
			atomic_dec(&kdi_ref_count);
			ret = DAL_KDI_STATUS_INTERNAL_ERROR;
			*handle = DAL_KDI_INVALID_HANDLE;
			goto end;
		}

		bh_err = bhp_init_internal(NULL);
		if (bh_err) {
			pr_err("BHP_Init failed with status = %d\n", bh_err);
			ret = bh_err_to_kdi_err(bh_err);
			*handle = DAL_KDI_INVALID_HANDLE;
		}
	}

	*handle = (u64)dal_class;
end:
	return ret;
}
EXPORT_SYMBOL(kdi_init);

int kdi_deinit(u64 handle)
{
	int ret, bh_err;

	/* TODO: add a kernel cleanup code to module deinit possible flow:
	 * one kernel module invokes init and a different one invokes deinit
	 */

	/* check handle first */
	if (!kdi_check_handle(handle))
		return DAL_KDI_STATUS_INVALID_HANDLE;

	ret = DAL_KDI_SUCCESS;
	if (atomic_dec_and_test(&kdi_ref_count)) {
		bh_err = bhp_deinit_internal();
		if (bh_err)
			pr_warn("bhp_deinit_internal failed: = %d\n", bh_err);

		ret = bh_err_to_kdi_err(ret);

		mutex_destroy(&kdi_lock);
		kdi_destroy_kernel_clients();
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

	if (!kdi_is_init_done())
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

	if (!kdi_is_init_done())
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

	if (!kdi_is_init_done())
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
