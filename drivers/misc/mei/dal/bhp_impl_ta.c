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

/*
 *
 * @file  bhp_impl_ta.cpp
 * @brief This file implements Beihai Host Proxy (BHP) module TA related API.
 * @author
 * @version
 *
 */

#include <linux/string.h>
#include "bh_errcode.h"
#include "bhp_impl.h"

enum bhp_vm_mode {
	BHP_OPEN_VM_QUERY_MODE = 0,
	BHP_OPEN_VM_NORMAL_MODE = 1
};

static int char2hex(char c)
{
	if (c >= '0' && c <= '9')
		return (c - '0');
	else if (c >= 'a' && c <= 'f')
		return (c - 'a' + 0xA);
	else
		return (c - 'A' + 0xA);
}

static int string_check1_uuid(const char *str)
{
	int i;

	for (i = 0; i < BH_GUID_LENGTH * 2; i++, str++)
		if (!((*str >= '0' && *str <= '9') ||
				(*str >= 'a' && *str <= 'f') ||
				(*str >= 'A' && *str <= 'F')))
			return 0;

	/* incorrect string length */
	if (*str != 0)
		return 0;

	return 1;
}

static int string_check2_uuid(const char *str)
{
	int i;

	for (i = 0; i < BH_GUID_LENGTH * 2; i++, str++) {

		if (*str == '-' && (i == 8 || i == 12 || i == 16 || i == 20))
			str++;

		if (!((*str >= '0' && *str <= '9') ||
				(*str >= 'a' && *str <= 'f') ||
				(*str >= 'A' && *str <= 'F')))
			return 0;
	}

	/* incorrect string length */
	if (*str != 0)
		return 0;

	return 1;
}

static s32 string_to_uuid(const s8 *str, s8 *uuid)
{
	int i;

	if (!string_check1_uuid(str) && !string_check2_uuid(str))
		return 0;

	for (i = 0; i < BH_GUID_LENGTH; i++, uuid++) {

		if (*str == '-')
			str++;

		*uuid = (s8) char2hex(*str++);
		*uuid <<= 4;
		*uuid += (s8) char2hex(*str++);
	}
	return 1;
}

/* try to session_enter for IVM, then SVM */
static struct bh_response_record *
session_enter_vm(u64 seq, int *conn_idx, int lock_session)
{
	struct bh_response_record *rr = NULL;

	if (!conn_idx)
		return NULL;

	rr = session_enter(CONN_IDX_IVM, seq, lock_session);
	if (rr)
		*conn_idx = CONN_IDX_IVM;

	return rr;
}

static int bh_proxy_get_sd_by_ta(struct bh_ta_id taid, struct bh_sd_id *sdid)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *) cmdbuf;
	struct bhp_get_sd_by_ta_cmd *cmd =
			(struct bhp_get_sd_by_ta_cmd *) h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(&rr, 0x00, sizeof(rr));

	if (!sdid)
		return BPE_INVALID_PARAMS;

	h->id = BHP_CMD_GET_SD_BY_TA;
	cmd->taid = taid;

	ret = bh_cmd_transfer(CONN_IDX_SDM, (char *) h,
			sizeof(*h) + sizeof(*cmd), NULL, 0,
			rrmap_add(CONN_IDX_SDM, &rr));

	if (ret == BH_SUCCESS)
		ret = rr.code;

	if (ret != BH_SUCCESS)
		goto cleanup;

	if (rr.buffer && rr.length ==
			sizeof(struct bhp_get_sd_by_ta_response)) {
		struct bhp_get_sd_by_ta_response *resp =
				(struct bhp_get_sd_by_ta_response *) rr.buffer;
		*sdid = resp->sdid;
	} else
		ret = BPE_MESSAGE_ILLEGAL;

cleanup:
	kfree(rr.buffer);

	return ret;
}

static int bh_proxy_check_svl_ta_blocked_state(struct bh_ta_id taid)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *) cmdbuf;
	struct bhp_check_svl_ta_blocked_state_cmd *cmd =
			(struct bhp_check_svl_ta_blocked_state_cmd *) h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(&rr, 0x00, sizeof(rr));

	h->id = BHP_CMD_CHECK_SVL_TA_BLOCKED_STATE;
	cmd->taid = taid;
	memcpy(&cmd->taid, &taid,
			sizeof(struct bhp_check_svl_ta_blocked_state_cmd));

	ret = bh_cmd_transfer(CONN_IDX_SDM, (char *) h,
			sizeof(*h) + sizeof(*cmd), NULL, 0,
			rrmap_add(CONN_IDX_SDM, &rr));
	if (ret == BH_SUCCESS)
		ret = rr.code;

	kfree(rr.buffer);

	return ret;
}

static int bh_proxy_listJTAPackages(int conn_idx, int *count,
				    struct bh_ta_id **app_ids)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *) cmdbuf;
	struct bh_response_record rr;
	struct bhp_list_ta_packages_response *resp;
	struct bh_ta_id *outbuf;
	unsigned int i;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(&rr, 0x00, sizeof(rr));

	if (!bhp_is_initialized())
		return BPE_NOT_INIT;

	if (!count || !app_ids)
		return BPE_INVALID_PARAMS;

	h->id = BHP_CMD_LIST_TA_PACKAGES;

	ret = bh_cmd_transfer(conn_idx, (char *)h, sizeof(*h),
			     NULL, 0, rrmap_add(conn_idx, &rr));

	if (ret == BH_SUCCESS)
		ret = rr.code;

	*app_ids = NULL;
	*count = 0;

	if (ret != BH_SUCCESS)
		goto out;

	if (!rr.buffer) {
		ret = BPE_MESSAGE_ILLEGAL;
		goto out;
	}

	resp = (struct bhp_list_ta_packages_response *)rr.buffer;
	if (!resp->count)
		goto out;

	if (rr.length != sizeof(struct bh_ta_id) *
			 resp->count +
			sizeof(struct bhp_list_ta_packages_response)) {
		ret = BPE_MESSAGE_ILLEGAL;
		goto out;
	}

	outbuf = kcalloc(resp->count, sizeof(struct bh_ta_id), GFP_KERNEL);

	if (!outbuf) {
		ret = BPE_OUT_OF_MEMORY;
		goto out;
	}
	for (i = 0; i < resp->count; i++)
		outbuf[i] = resp->app_ids[i];

	*app_ids = outbuf;
	*count = resp->count;

out:
	kfree(rr.buffer);
	return ret;
}

static int bh_proxy_download_javata(
		int conn_idx,
		struct bh_ta_id ta_id,
		const char *ta_pkg,
		unsigned int pkg_len)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *) cmdbuf;
	struct bhp_download_javata_cmd *cmd =
			(struct bhp_download_javata_cmd *) h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));
	memset(&rr, 0x00, sizeof(rr));

	if (!ta_pkg || !pkg_len)
		return BPE_INVALID_PARAMS;

	h->id = BHP_CMD_DOWNLOAD_JAVATA;
	cmd->appid = ta_id;

	ret = bh_cmd_transfer(conn_idx, (char *) h, sizeof(*h) + sizeof(*cmd),
			ta_pkg, pkg_len, rrmap_add(conn_idx, &rr));

	if (ret == BH_SUCCESS)
		ret = rr.code;

	kfree(rr.buffer);

	return ret;
}

static int bh_proxy_openjtasession(
		int conn_idx,
		struct bh_ta_id ta_id,
		const char *init_buffer,
		unsigned int init_len,
		u64 *handle,
		int *vm_conn_closed,
		const char *ta_pkg,
		unsigned int pkg_len)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *) cmdbuf;
	struct bhp_open_jtasession_cmd *cmd =
			(struct bhp_open_jtasession_cmd *) h->cmd;
	struct bh_response_record *rr = NULL;
	u64 seq;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));

	if (!handle)
		return BPE_INVALID_PARAMS;
	if (!init_buffer && init_len > 0)
		return BPE_INVALID_PARAMS;

	rr = kzalloc(sizeof(struct bh_response_record), GFP_KERNEL);
	if (!rr)
		return BPE_OUT_OF_MEMORY;

	memset(rr, 0, sizeof(struct bh_response_record));

	rr->count = 1;
	rr->is_session = true;
	seq = rrmap_add(conn_idx, rr);

	h->id = BHP_CMD_OPEN_JTASESSION;
	cmd->appid = ta_id;

	ret = bh_cmd_transfer(conn_idx, (char *) h, sizeof(*h) + sizeof(*cmd),
			     (char *) init_buffer, init_len, seq);

	if (ret == BH_SUCCESS)
		ret = rr->code;

	kfree(rr->buffer);
	rr->buffer = NULL;

	if (ret == BHE_PACKAGE_NOT_FOUND) {
		/*
		 * VM might delete the TA pkg when no live session.
		 * Download the TA pkg and open session again
		 */
		ret = bh_proxy_download_javata(conn_idx, ta_id,
					       ta_pkg, pkg_len);
		if (ret == BH_SUCCESS) {
			ret = bh_cmd_transfer(conn_idx, (char *)h,
					sizeof(*h) + sizeof(*cmd),
					(char *)init_buffer, init_len, seq);

			if (ret == BH_SUCCESS)
				ret = rr->code;

			kfree(rr->buffer);
			rr->buffer = NULL;
		}
	}

	if (ret == BH_SUCCESS) {
		*handle = (u64) seq;
		session_exit(conn_idx, rr, seq, 0);
	} else {
		/*
		 * bh_do_closeVM() will be called in following
		 * session_close(), as rr->count is 1
		 */
		session_close(conn_idx, rr, seq, 0);
		*vm_conn_closed = 1;
	}

	return ret;
}

int bhp_open_ta_session(u64 *session, const char *app_id,
			const u8 *ta_pkg, size_t pkg_len,
			const u8 *init_buffer, size_t init_len)
{
	int ret;
	struct bh_ta_id ta_id;
	int conn_idx = 0;
	int vm_conn_closed = 0;
	struct bh_sd_id sdid;
	int ta_existed = 0;
	int count = 0;
	struct bh_ta_id *app_ids = NULL;
	int i;

	if (!app_id || !session)
		return BPE_INVALID_PARAMS;

	if (!ta_pkg || !pkg_len)
		return BPE_INVALID_PARAMS;

	if (!init_buffer && init_len != 0)
		return BPE_INVALID_PARAMS;

	if (!string_to_uuid(app_id, (char *) &ta_id))
		return BPE_INVALID_PARAMS;

	*session = 0;

	/* 1.1: get the TA's sdid */
	ret = bh_proxy_get_sd_by_ta(ta_id, &sdid);
	if (ret != BH_SUCCESS)
		return ret;

	ret = bh_proxy_check_svl_ta_blocked_state(ta_id);
	if (ret != BH_SUCCESS)
		return ret;

	/* 1.2: get corresponding vm conn_idx */
	ret = bh_do_open_vm(sdid, &conn_idx, BHP_OPEN_VM_NORMAL_MODE);
	if (ret != BH_SUCCESS)
		return ret;

	/* 2.1: check whether the ta pkg existed in VM or not */
	ret = bh_proxy_listJTAPackages(conn_idx, &count, &app_ids);
	if (ret == BH_SUCCESS) {
		for (i = 0; i < count; i++) {
			if (!memcmp(&ta_id, &app_ids[i], sizeof(struct bh_ta_id))) {
				ta_existed = 1;
				break;
			}
		}
		kfree(app_ids);
	}

	/* 2.2: download ta pkg if not existed. */
	if (!ta_existed) {
		ret = bh_proxy_download_javata(conn_idx,
					       ta_id, ta_pkg, pkg_len);
		if (ret != BH_SUCCESS && ret != BHE_PACKAGE_EXIST)
			goto cleanup;
	}

	/* 3: send opensession cmd to VM */
	ret = bh_proxy_openjtasession(conn_idx, ta_id,
				      init_buffer, init_len,
				      session, &vm_conn_closed,
				      ta_pkg, pkg_len);

cleanup:
	/*
	 * closeVM only when this process failed and vm has not been closed
	 * inside openjtasession, otherwise the session is created.
	 */
	if (ret != BH_SUCCESS && !vm_conn_closed)
		bh_do_close_vm(conn_idx);

	return ret;
}

int bhp_send_and_recv(const u64 handle, int command_id,
		      const void *input, size_t length,
		      void **output, size_t *output_length,
		      int *response_code)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bhp_snr_cmd *cmd = (struct bhp_snr_cmd *)h->cmd;
	u64 seq = (u64) handle;
	struct bh_response_record *rr = NULL;
	int conn_idx = 0;
	unsigned int len;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));

	if (!bhp_is_initialized())
		return BPE_NOT_INIT;

	if (!input && length != 0)
		return BPE_INVALID_PARAMS;

	if (!output_length)
		return BPE_INVALID_PARAMS;

	if (output)
		*output = NULL;

	rr = session_enter_vm(seq, &conn_idx, 1);
	if (!rr)
		return BPE_INVALID_PARAMS;

	rr->buffer = NULL;
	h->id = BHP_CMD_SENDANDRECV;

	cmd->ta_session_id = rr->addr;
	cmd->command = command_id;
	cmd->outlen = *output_length;

	ret = bh_cmd_transfer(conn_idx, (char *)h, sizeof(*h) + sizeof(*cmd),
			      (char *) input, length, seq);

	if (ret == BH_SUCCESS)
		ret = rr->code;

	if (rr->killed)
		ret = BHE_UNCAUGHT_EXCEPTION;

	if (ret == BH_SUCCESS) {
		struct bhp_snr_response *resp = NULL;

		if (rr->buffer &&
		    rr->length >= sizeof(struct bhp_snr_response)) {
			resp = (struct bhp_snr_response *)rr->buffer;
			if (response_code)
				*response_code = be32_to_cpu(resp->response);

			len = rr->length - sizeof(struct bhp_snr_response);

			if (len > 0) {
				if (output && *output_length >= len) {
					*output = kzalloc(len, GFP_KERNEL);
					if (*output)
						memcpy(*output,
						       resp->buffer, len);
					else
						ret = BPE_OUT_OF_MEMORY;

				} else
					ret = BHE_APPLET_SMALL_BUFFER;
			}

			*output_length = len;
		} else
			ret = BPE_MESSAGE_TOO_SHORT;

	} else if (ret == BHE_APPLET_SMALL_BUFFER && rr->buffer &&
		   rr->length == sizeof(struct bhp_snr_bof_response)) {

		struct bhp_snr_bof_response *resp =
				(struct bhp_snr_bof_response *)rr->buffer;

		if (response_code)
			*response_code = be32_to_cpu(resp->response);

		*output_length = be32_to_cpu(resp->request_length);
	}

	kfree(rr->buffer);
	rr->buffer = NULL;

	session_exit(conn_idx, rr, seq, 1);

	return ret;
}

int bhp_close_ta_session(const u64 handle)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bhp_close_jtasession_cmd *cmd =
			(struct bhp_close_jtasession_cmd *)h->cmd;
	struct bh_response_record *rr;
	u64 seq = (u64)handle;
	int conn_idx = 0;

	memset(cmdbuf, 0x00, sizeof(cmdbuf));

	rr = session_enter_vm(seq, &conn_idx, 1);
	if (!rr)
		return BPE_INVALID_PARAMS;

	h->id = BHP_CMD_CLOSE_JTASESSION;
	cmd->ta_session_id = rr->addr;

	ret = bh_cmd_transfer(conn_idx, (char *) h, sizeof(*h) + sizeof(*cmd),
			      NULL, 0, seq);

	if (ret == BH_SUCCESS)
		ret = rr->code;

	if (rr->killed)
		ret = BHE_UNCAUGHT_EXCEPTION;

	/*
	 * internal session exists, so we should not close the hmc session.
	 * It means that host app should call this API at approciate time later.
	 */
	if (ret == BHE_IAC_EXIST_INTERNAL_SESSION)
		session_exit(conn_idx, rr, seq, 1);
	else
		session_close(conn_idx, rr, seq, 1);

	return ret;
}
