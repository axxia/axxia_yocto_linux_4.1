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
#include <linux/uuid.h>
#include <linux/ctype.h>
#include <linux/dal.h>

#include "bh_errcode.h"
#include "bhp_impl.h"

static bool uuid_is_valid_hyphenless(const char *uuid_str)
{
	unsigned int i;

	/* exclude (i == 8 || i == 13 || i == 18 || i == 23) */
	for (i = 0; i < UUID_STRING_LEN - 4; i++)
		if (!isxdigit(uuid_str[i]))
			return false;

	return true;
}

static void uuid_normalize_hyphenless(const char *uuid_hl, char *uuid_str)
{
	unsigned int i;

	for (i = 0; i < UUID_STRING_LEN; i++) {
		if (i == 8 || i == 13 || i == 18 || i == 23)
			uuid_str[i] = '-';
		else
			uuid_str[i] = *uuid_hl++;
	}
	uuid_str[i] = '\0';
}

int dal_uuid_be_to_bin(const char *uuid_str, uuid_be *uuid)
{
	char __uuid_str[UUID_STRING_LEN + 1];

	if (uuid_is_valid_hyphenless(uuid_str)) {
		uuid_normalize_hyphenless(uuid_str, __uuid_str);
		uuid_str = __uuid_str;
	}

	return uuid_be_to_bin(uuid_str, uuid);
}
EXPORT_SYMBOL(dal_uuid_be_to_bin);

/*
 * 4 bytes array to identify BH headers
 */
static const u8 BH_MSG_RESP_MAGIC[]  = {0xff, 0xa5, 0xaa, 0x55};
static const u8 BH_MSG_CMD_MAGIC[]   = {0xff, 0xa3, 0xaa, 0x55};

/* Check for response msg */
bool bh_msg_is_response(const char *msg, size_t len)
{
	return (len >= sizeof(struct bhp_response_header)) &&
		!memcmp(msg, BH_MSG_RESP_MAGIC, sizeof(BH_MSG_CMD_MAGIC));
}

/* Check for command msg */
bool bh_msg_is_cmd(const char *msg, size_t len)
{
	return (len >= sizeof(struct bhp_command_header)) &&
		!memcmp(msg, BH_MSG_CMD_MAGIC, sizeof(BH_MSG_CMD_MAGIC));
}

const struct bhp_command_header *bh_msg_cmd_hdr(const void *msg, size_t len)
{
	/* check that len is valid before checking the hdr MAGIC,
	in case that len is smaller than the MAGIC size */
	if (!bh_msg_is_cmd(msg, len))
		return NULL;

	return msg;
}

bool bh_msg_is_cmd_open_session(const struct bhp_command_header *hdr)
{
	return hdr->id == BHP_CMD_OPEN_JTASESSION;
}

const uuid_be *bh_open_session_ta_id(const struct bhp_command_header *hdr,
				     size_t count)
{
	struct bhp_open_jtasession_cmd *open_cmd;

	if (count < sizeof(struct bhp_command_header) +
			sizeof(struct bhp_open_jtasession_cmd))
		return NULL;

	open_cmd = (struct bhp_open_jtasession_cmd *)hdr->cmd;

	return &open_cmd->appid;
}

static int bh_cmd(int conn_idx,
		  void *cmd, unsigned int cmd_len,
		  const void *data, unsigned int data_len,
		  u64 seq, struct bh_response_record *rr)
{
	int ret;

	ret = bh_request(conn_idx, cmd, cmd_len, data, data_len, seq);

	kfree(rr->buffer);
	rr->buffer = NULL;

	return ret;
}


enum bhp_vm_mode {
	BHP_OPEN_VM_QUERY_MODE = 0,
	BHP_OPEN_VM_NORMAL_MODE = 1
};

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

static int bh_proxy_get_sd_by_ta(uuid_be taid, uuid_be *sdid)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bhp_get_sd_by_ta_cmd *cmd =
			(struct bhp_get_sd_by_ta_cmd *)h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0, sizeof(cmdbuf));
	memset(&rr, 0, sizeof(rr));

	if (!sdid)
		return -EINVAL;

	h->id = BHP_CMD_GET_SD_BY_TA;
	cmd->taid = taid;

	ret = bh_request(CONN_IDX_SDM, h, sizeof(*h) + sizeof(*cmd), NULL,
			 0, rrmap_add(CONN_IDX_SDM, &rr));

	if (!ret)
		ret = rr.code;

	if (ret)
		goto cleanup;

	if (rr.buffer && rr.length ==
			sizeof(struct bhp_get_sd_by_ta_response)) {
		struct bhp_get_sd_by_ta_response *resp =
				(struct bhp_get_sd_by_ta_response *)rr.buffer;
		*sdid = resp->sdid;
	} else
		ret = -EBADMSG;

cleanup:
	kfree(rr.buffer);

	return ret;
}

static int bh_proxy_check_svl_ta_blocked_state(uuid_be taid)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bhp_check_svl_ta_blocked_state_cmd *cmd =
			(struct bhp_check_svl_ta_blocked_state_cmd *)h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0, sizeof(cmdbuf));
	memset(&rr, 0, sizeof(rr));

	h->id = BHP_CMD_CHECK_SVL_TA_BLOCKED_STATE;
	cmd->taid = taid;
	memcpy(&cmd->taid, &taid,
	       sizeof(struct bhp_check_svl_ta_blocked_state_cmd));

	ret = bh_cmd(CONN_IDX_SDM, h, sizeof(*h) + sizeof(*cmd), NULL,
		     0, rrmap_add(CONN_IDX_SDM, &rr), &rr);
	if (!ret)
		ret = rr.code;

	return ret;
}

static int bh_proxy_listJTAPackages(int conn_idx, int *count,
				    uuid_be **app_ids)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bh_response_record rr;
	struct bhp_list_ta_packages_response *resp;
	uuid_be *outbuf;
	unsigned int i;

	memset(cmdbuf, 0, sizeof(cmdbuf));
	memset(&rr, 0, sizeof(rr));

	if (!bhp_is_initialized())
		return -EFAULT;

	if (!count || !app_ids)
		return -EINVAL;

	*app_ids = NULL;
	*count = 0;

	h->id = BHP_CMD_LIST_TA_PACKAGES;

	ret = bh_request(conn_idx, h, sizeof(*h), NULL, 0,
			 rrmap_add(conn_idx, &rr));
	if (!ret)
		ret = rr.code;
	if (ret)
		goto out;

	if (!rr.buffer) {
		ret = -EBADMSG;
		goto out;
	}

	resp = (struct bhp_list_ta_packages_response *)rr.buffer;
	if (!resp->count) {
		ret = -EBADMSG;
		goto out;
	}

	if (rr.length != sizeof(uuid_be) * resp->count +
			sizeof(struct bhp_list_ta_packages_response)) {
		ret = -EBADMSG;
		goto out;
	}

	outbuf = kcalloc(resp->count, sizeof(uuid_be), GFP_KERNEL);

	if (!outbuf) {
		ret = -ENOMEM;
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

static int bh_proxy_download_javata(int conn_idx,
				    uuid_be ta_id,
				    const char *ta_pkg,
				    unsigned int pkg_len)
{
	int ret;
	char cmdbuf[CMDBUF_SIZE];
	struct bhp_command_header *h = (struct bhp_command_header *)cmdbuf;
	struct bhp_download_javata_cmd *cmd =
			(struct bhp_download_javata_cmd *)h->cmd;
	struct bh_response_record rr;

	memset(cmdbuf, 0, sizeof(cmdbuf));
	memset(&rr, 0, sizeof(rr));

	if (!ta_pkg || !pkg_len)
		return -EINVAL;

	h->id = BHP_CMD_DOWNLOAD_JAVATA;
	cmd->appid = ta_id;

	ret = bh_cmd(conn_idx, h, sizeof(*h) + sizeof(*cmd), ta_pkg,
		     pkg_len, rrmap_add(conn_idx, &rr), &rr);

	if (!ret)
		ret = rr.code;

	return ret;
}

static int bh_proxy_openjtasession(int conn_idx,
				   uuid_be ta_id,
				   const char *init_buffer,
				   unsigned int init_len,
				   u64 *handle,
				   int *vm_conn_closed,
				   const char *ta_pkg,
				   unsigned int pkg_len)
{
	int ret;
	struct bhp_command_header *h;
	struct bhp_open_jtasession_cmd *cmd;
	const size_t cmd_sz = sizeof(*h) + sizeof(*cmd);
	char cmd_buf[cmd_sz];
	struct bh_response_record *rr;
	u64 seq;

	if (!handle)
		return -EINVAL;

	if (!init_buffer && init_len > 0)
		return -EINVAL;

	memset(cmd_buf, 0, cmd_sz);

	h = (struct bhp_command_header *)cmd_buf;
	cmd = (struct bhp_open_jtasession_cmd *)h->cmd;

	rr = kzalloc(sizeof(*rr), GFP_KERNEL);
	if (!rr)
		return -ENOMEM;

	rr->count = 1;
	rr->is_session = true;
	seq = rrmap_add(conn_idx, rr);

	h->id = BHP_CMD_OPEN_JTASESSION;
	cmd->appid = ta_id;

	ret = bh_cmd(conn_idx, cmd_buf, cmd_sz, init_buffer, init_len, seq, rr);
	if (!ret)
		ret = rr->code;

	if (ret == BHE_PACKAGE_NOT_FOUND) {
		/*
		 * VM might delete the TA pkg when no live session.
		 * Download the TA pkg and open session again
		 */
		ret = bh_proxy_download_javata(conn_idx, ta_id,
					       ta_pkg, pkg_len);
		if (ret)
			goto out_err;

		ret = bh_cmd(conn_idx, cmd_buf, cmd_sz, init_buffer,
			     init_len, seq, rr);

		if (!ret)
			ret = rr->code;
	}

	if (ret)
		goto out_err;

	*handle = (u64)seq;
	session_exit(conn_idx, rr, seq, 0);

	return 0;

out_err:
	/*
	 * bh_do_closeVM() will be called in following
	 * session_close(), as rr->count is 1
	 */
	session_close(conn_idx, rr, seq, 0);
	*vm_conn_closed = 1;

	return ret;
}

int bhp_open_ta_session(u64 *session, const char *app_id,
			const u8 *ta_pkg, size_t pkg_len,
			const u8 *init_buffer, size_t init_len)
{
	int ret;
	uuid_be ta_id;
	int conn_idx = 0;
	int vm_conn_closed = 0;
	uuid_be sdid;
	int ta_existed = 0;
	int count = 0;
	uuid_be *app_ids = NULL;
	int i;

	if (!app_id || !session)
		return -EINVAL;

	if (!ta_pkg || !pkg_len)
		return -EINVAL;

	if (!init_buffer && init_len != 0)
		return -EINVAL;

	if (dal_uuid_be_to_bin(app_id, &ta_id))
		return -EINVAL;

	*session = 0;

	/* 1.1: get the TA's sdid */
	ret = bh_proxy_get_sd_by_ta(ta_id, &sdid);
	if (ret)
		return ret;

	ret = bh_proxy_check_svl_ta_blocked_state(ta_id);
	if (ret)
		return ret;

	/* 1.2: get corresponding vm conn_idx */
	ret = bh_do_open_vm(sdid, &conn_idx, BHP_OPEN_VM_NORMAL_MODE);
	if (ret)
		return ret;

	/* 2.1: check whether the ta pkg existed in VM or not */
	ret = bh_proxy_listJTAPackages(conn_idx, &count, &app_ids);
	if (!ret) {
		for (i = 0; i < count; i++) {
			if (!uuid_be_cmp(ta_id, app_ids[i])) {
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
		if (ret && ret != BHE_PACKAGE_EXIST)
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
	if (ret && !vm_conn_closed)
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
	u64 seq = (u64)handle;
	struct bh_response_record *rr = NULL;
	int conn_idx = 0;
	unsigned int len;

	memset(cmdbuf, 0, sizeof(cmdbuf));

	if (!bhp_is_initialized())
		return -EFAULT;

	if (!input && length != 0)
		return -EINVAL;

	if (!output_length)
		return -EINVAL;

	if (output)
		*output = NULL;

	rr = session_enter_vm(seq, &conn_idx, 1);
	if (!rr)
		return -EINVAL;

	rr->buffer = NULL;
	h->id = BHP_CMD_SENDANDRECV;
	cmd->ta_session_id = rr->addr;
	cmd->command = command_id;
	cmd->outlen = *output_length;

	ret = bh_request(conn_idx, h, sizeof(*h) + sizeof(*cmd), input,
			 length, seq);
	if (!ret)
		ret = rr->code;

	if (rr->killed)
		ret = BHE_UNCAUGHT_EXCEPTION;

	if (ret == BHE_APPLET_SMALL_BUFFER && rr->buffer &&
			rr->length == sizeof(struct bhp_snr_bof_response)) {
		struct bhp_snr_bof_response *bof_resp =
			(struct bhp_snr_bof_response *)rr->buffer;

		if (response_code)
			*response_code = be32_to_cpu(bof_resp->response);

		*output_length = be32_to_cpu(bof_resp->request_length);
	}

	if (ret)
		goto out;

	if (rr->buffer && rr->length >= sizeof(struct bhp_snr_response)) {
		struct bhp_snr_response *resp =
			(struct bhp_snr_response *)rr->buffer;

		if (response_code)
			*response_code = be32_to_cpu(resp->response);

		len = rr->length - sizeof(struct bhp_snr_response);

		if (*output_length < len) {
			ret = -EMSGSIZE;
			goto out;
		}

		if (len > 0 && output) {
			*output = kzalloc(len, GFP_KERNEL);
			if (!*output) {
				ret = -ENOMEM;
				goto out;
			}
			memcpy(*output, resp->buffer, len);
		}

		*output_length = len;
	} else
		ret = -EBADMSG;

out:
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

	memset(cmdbuf, 0, sizeof(cmdbuf));

	rr = session_enter_vm(seq, &conn_idx, 1);
	if (!rr)
		return -EINVAL;

	h->id = BHP_CMD_CLOSE_JTASESSION;
	cmd->ta_session_id = rr->addr;

	ret = bh_cmd(conn_idx, h, sizeof(*h) + sizeof(*cmd), NULL, 0, seq, rr);

	if (!ret)
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

int bh_filter_hdr(const struct bhp_command_header *hdr, size_t count, void *ctx,
		  const bh_filter_func tbl[])
{
	int i;
	int ret;

	for (i = 0; tbl[i]; i++) {
		ret = tbl[i](hdr, count, ctx);
		if (ret < 0)
			return ret;
	}
	return 0;
}

void bh_prep_access_denied_response(const char *cmd,
				    struct bhp_response_header *res)
{
	struct bhp_command_header *cmd_hdr = (struct bhp_command_header *)cmd;

	memcpy(res->h.magic, BHP_MSG_RESPONSE_MAGIC, BHP_MSG_MAGIC_LENGTH);
	res->h.length = sizeof(struct bhp_response_header);
	res->code = BHE_OPERATION_NOT_PERMITTED;
	res->seq = cmd_hdr->seq;
}
