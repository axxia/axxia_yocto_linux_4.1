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

#include <linux/printk.h>
#include <linux/mei_cl_bus.h>
#include "bhp_impl.h"
#include "bhp_exp.h"
#include "dal_dev.h"

static unsigned int init_state = DEINITED;
static u64 sequence_number = MSG_SEQ_START_NUMBER;
static struct bh_connection_item connections[MAX_CONNECTIONS];

/*
 * increment_seq_number():
 * increase the shared variable sequence_number by 1 and wrap around if needed.
 * note: sequence_number is shared resource among all connections/threads.
 */
static u64 increment_sequence_number(void)
{
	u64 ret = 0;

	mutex_enter(bhm_seqno);
	sequence_number++;
	/* wrap around. sequence_number must
	 * not be 0, as required by Firmware VM
	 */
	if (sequence_number == 0)
		sequence_number = MSG_SEQ_START_NUMBER;

	ret = sequence_number;
	mutex_exit(bhm_seqno);

	return ret;
}

struct RR_MAP_INFO {
	struct list_head link;
	u64 seq;
	struct bh_response_record *rr;
};

#if 0 /* for debug */
static void rrmap_dump(struct list_head *rr_map_header)
{
	struct list_head *pos;
	struct RR_MAP_INFO *rrmap_info;
	size_t count;

	count = 0;

	list_for_each(pos, rr_map_header) {
		rrmap_info = list_entry(pos, struct RR_MAP_INFO, link);
		if (rrmap_info) {
			pr_debug("[%02x] seq: %llu, rr->addr: %llu",
				  count, rrmap_info->seq, rrmap_info->rr->addr);
			count++;
		}
	}
}
#endif

static struct RR_MAP_INFO *rrmap_find_by_addr(struct list_head *rr_map_header,
					      u64 seq)
{
	struct list_head *pos;
	struct RR_MAP_INFO *rrmap_info;

	list_for_each(pos, rr_map_header) {
		rrmap_info = list_entry(pos, struct RR_MAP_INFO, link);
		if (rrmap_info && rrmap_info->seq == seq)
			return rrmap_info;
	}

	return NULL;
}

u64 rrmap_add(int conn_idx, struct bh_response_record *rr)
{
	u64 seq = increment_sequence_number();
	struct RR_MAP_INFO *rrmap_info;

	/* TODO: check if malloc succeeded: need to refactor the usage
	 * of rrmap_add() to check and handle errors
	 */
	rrmap_info = kzalloc(sizeof(struct RR_MAP_INFO), GFP_KERNEL);

	rrmap_info->seq = seq;
	rrmap_info->rr = rr;

	list_add_tail(&rrmap_info->link,
		      &connections[conn_idx].rr_map_list_header);

	return rrmap_info->seq;
}

/**
 * in the original BHP they use a map, in the kernel we don't have a map.
 * we're using a list.
 * in BHP they simply delete an element from the map.
 * so in order to remove a record which is a session we added a parameter
 * 'remove_record'.
 */
static struct bh_response_record *rrmap_remove(int conn_idx, u64 seq,
					       bool remove_record)
{
	struct RR_MAP_INFO *rrmap_info;
	struct bh_response_record *rr = NULL;

	rrmap_info =
		rrmap_find_by_addr(&connections[conn_idx].rr_map_list_header,
				   seq);

	if (rrmap_info != NULL) {
		rr = rrmap_info->rr;
		if (!rr->is_session || remove_record) {
			list_del_init(&rrmap_info->link);
			kfree(rrmap_info);
		}
	}

	return rr;
}

static struct bh_response_record *addr2record(int conn_idx, u64 seq)
{
	struct bh_response_record *rr = NULL;
	struct RR_MAP_INFO *rrmap_info;

	rrmap_info =
		rrmap_find_by_addr(&connections[conn_idx].rr_map_list_header,
				   seq);

	if (rrmap_info != NULL)
		rr = rrmap_info->rr;

	return rr;
}

static void destroy_session(struct bh_response_record *session)
{
	if (session)
		kfree(session->buffer);
	kfree(session);
}

struct bh_response_record *session_enter(int conn_idx, u64 seq,
					 int lock_session)
{
	struct bh_response_record *session = NULL;
	struct RR_MAP_INFO *rrmap_info;

	mutex_enter(connections[conn_idx].bhm_rrmap);

	rrmap_info =
		rrmap_find_by_addr(&connections[conn_idx].rr_map_list_header,
				   seq);

	if (rrmap_info) {
		if (rrmap_info->rr->is_session && !rrmap_info->rr->killed) {
			session = rrmap_info->rr;

			if (session->count < MAX_SESSION_LIMIT)
				session->count++;
			else
				session = NULL;
		}
	}

	mutex_exit(connections[conn_idx].bhm_rrmap);

	if (session && lock_session) {
		mutex_enter(session->session_lock);

		/* check whether session has been
		 * killed before session operation
		 */
		if (session->killed) {
			session_exit(conn_idx, session, seq, 1);
			session = NULL;
		}
	}

	return session;
}

void session_exit(int conn_idx, struct bh_response_record *session,
		  u64 seq, int unlock_session)
{
	bool close_vm_conn = false;

	mutex_enter(connections[conn_idx].bhm_rrmap);
	session->count--;

	if (session->count == 0 && session->killed) {
		rrmap_remove(conn_idx, seq, true);

		if (unlock_session)
			mutex_exit(session->session_lock);

		destroy_session(session);
		if (conn_idx >= CONN_IDX_SVM)
			close_vm_conn = true;
	} else {
		if (unlock_session)
			mutex_exit(session->session_lock);
	}

	mutex_exit(connections[conn_idx].bhm_rrmap);

	/* remove the VM conn counter of
	 * this session:only for connected SVM
	 */
	if (close_vm_conn)
		bh_do_close_vm(conn_idx);
}

void session_close(int conn_idx, struct bh_response_record *session,
		   u64 seq, int unlock_session)
{
	bool close_vm_conn = false;

	mutex_enter(connections[conn_idx].bhm_rrmap);
	session->count--;

	if (session->count == 0) {
		rrmap_remove(conn_idx, seq, true);
		if (unlock_session)
			mutex_exit(session->session_lock);
		destroy_session(session);

		if (conn_idx >= CONN_IDX_SVM)
			close_vm_conn = true;
	} else {
		session->killed = true;
		if (unlock_session)
			mutex_exit(session->session_lock);
	}

	mutex_exit(connections[conn_idx].bhm_rrmap);

	/* remove the VM conn counter of
	 * this session:only for connected SVM
	 */
	if (close_vm_conn)
		bh_do_close_vm(conn_idx);
}

static void session_kill(int conn_idx, struct bh_response_record *session,
			 u64 seq, bool is_caller_svm_recv_thread)
{
	bool close_vm_conn = false;

	mutex_enter(connections[conn_idx].bhm_rrmap);
	session->killed = true;
	if (session->count == 0) {
		rrmap_remove(conn_idx, seq, true);
		destroy_session(session);
		if (conn_idx >= CONN_IDX_SVM)
			close_vm_conn = true;
	}
	mutex_exit(connections[conn_idx].bhm_rrmap);

	/* decrease the VM connection counter of this session:
	 * only for connected SVM
	 */
	if (close_vm_conn) {
		if (!is_caller_svm_recv_thread) {
			bh_do_close_vm(conn_idx);
		} else {
			mutex_enter(connections[conn_idx].lock);
			if (connections[conn_idx].conn_count != 1)
				connections[conn_idx].conn_count--;

			mutex_exit(connections[conn_idx].lock);
		}
	}
}

bool bhp_is_initialized(void)
{
	return (READ_ONCE(init_state) == INITED);
}

static char skip_buffer[DAL_MAX_BUFFER_SIZE] = {0};
static int bh_transport_recv(unsigned int handle, void *buffer, size_t size)
{
	size_t got;
	size_t count = 0;
	int ret;
	char *buf = buffer;

	if (handle > DAL_MEI_DEVICE_MAX)
		return -ENODEV;

	while (size - count > 0) {
		got = min_t(size_t, size - count, DAL_MAX_BUFFER_SIZE);
		if (buf)
			ret = kdi_recv(handle, buf + count, &got);
		else
			ret = kdi_recv(handle, skip_buffer, &got);

		if (ret)
			return ret;

		count += got;
	}

	if (count != size)
		return -EFAULT;

	return 0;
}

static int bh_transport_send(unsigned int handle, const void *buffer,
			     unsigned int size, u64 seq)
{
	size_t chunk_sz;
	unsigned int count = 0;
	int ret;
	const char *buf = buffer;

	if (handle > DAL_MEI_DEVICE_MAX)
		return -ENODEV;

	while (size - count > 0) {
		chunk_sz = min_t(size_t, size - count, DAL_MAX_BUFFER_SIZE);
		ret = kdi_send(handle, buf + count, chunk_sz, seq);
		if (ret)
			return ret;

		count += chunk_sz;
	}

	return 0;
}

int bh_do_open_vm(uuid_be sdid, int *conn_idx, int mode)
{
	if (!conn_idx)
		return -EINVAL;

	*conn_idx = CONN_IDX_IVM;
	return 0;
}

int bh_do_close_vm(int conn_idx)
{
	return 0;
}

static int bh_send_message(int conn_idx, void *cmd, unsigned int clen,
		const void *data, unsigned int dlen, u64 seq)
{
	int ret;
	struct bh_response_record *rr = addr2record(conn_idx, seq);
	struct bhp_command_header *h = NULL;

	if (!rr)
		return -EFAULT;

	mutex_enter(connections[conn_idx].bhm_send);

	if (clen < sizeof(struct bhp_command_header) || !cmd || !rr)
		return -EINVAL;

	rr->buffer = NULL;
	rr->length = 0;

	memcpy(cmd, BHP_MSG_CMD_MAGIC, BHP_MSG_MAGIC_LENGTH);

	h = (struct bhp_command_header *)cmd;
	h->h.length = clen + dlen;
	h->seq = seq;

	ret = bh_transport_send(conn_idx, cmd, clen, seq);
	if (!ret && dlen > 0)
		ret = bh_transport_send(conn_idx, data, dlen, seq);

	if (ret)
		rrmap_remove(conn_idx, seq, false);

	mutex_exit(connections[conn_idx].bhm_send);

	return ret;
}

static int bh_recv_message(int conn_idx, u64 *seq)
{
	int ret;
	struct bhp_response_header headbuf;
	struct bhp_response_header *head = &headbuf;
	char *data = NULL;
	unsigned int dlen = 0;
	struct bh_response_record *rr = NULL;
	int session_killed;

	ret = bh_transport_recv(conn_idx, head,
				sizeof(struct bhp_response_header));
	if (ret)
		return ret;

	/* check magic */
	if (memcmp(BHP_MSG_RESPONSE_MAGIC,
		   head->h.magic, BHP_MSG_MAGIC_LENGTH) != 0)
		return -EBADMSG;

	/* verify rr */
	rr = rrmap_remove(conn_idx, head->seq, false);

	if (head->h.length > sizeof(struct bhp_response_header)) {
		dlen = head->h.length - sizeof(struct bhp_response_header);
		data = kzalloc(dlen, GFP_KERNEL);
		ret = bh_transport_recv(conn_idx, data, dlen);
		if (!ret && !data)
			ret = -ENOMEM;
	}

	if (rr) {
		rr->buffer = data;
		rr->length = dlen;

		if (!ret)
			rr->code = (int)head->code;
		else
			rr->code = ret;

		if (head->addr)
			rr->addr = head->addr;

		session_killed = (rr->is_session &&
				  (rr->code == BHE_WD_TIMEOUT ||
				  rr->code == BHE_UNCAUGHT_EXCEPTION ||
				  rr->code == BHE_APPLET_CRASHED));

		/* set killed flag before wake up send_wait thread */
		if (session_killed) {
			rr->killed = true;
			session_kill(conn_idx, rr, head->seq, true);
		}

	} else {
		kfree(data);
	}

	if (seq)
		*seq = head->seq;

	return ret;
}

static void bh_do_connect(int conn_idx)
{
	struct bh_connection_item *conn = &connections[conn_idx];

	conn->handle = 0;
	conn->conn_count = 0;
	INIT_LIST_HEAD(&conn->rr_map_list_header);
	memset(&conn->sdid, 0, sizeof(uuid_be));
}

static int bh_do_disconnect(int conn_idx)
{
	struct list_head *pos, *tmp;
	struct RR_MAP_INFO *rrmap_info;
	struct bh_connection_item *conn;

	conn = &connections[conn_idx];

	conn->conn_count = 0;
	conn->handle = 0;

	list_for_each_safe(pos, tmp, &conn->rr_map_list_header) {
		rrmap_info = list_entry(pos, struct RR_MAP_INFO, link);
		if (rrmap_info) {
			list_del(pos);
			kfree(rrmap_info);
		}
	}

	INIT_LIST_HEAD(&conn->rr_map_list_header);
	memset(&conn->sdid, 0, sizeof(uuid_be));

	return 0;
}

static void bh_connections_init(void)
{
	int i;

	for (i = CONN_IDX_START; i < MAX_CONNECTIONS; i++) {
		connections[i].conn_count = 0;
		connections[i].handle = 0;
		INIT_LIST_HEAD(&connections[i].rr_map_list_header);
	}

	/* connect to predefined heci ports, except SVM */
	for (i = CONN_IDX_START; i < CONN_IDX_SVM; i++)
		bh_do_connect(i);
}

static void bh_connections_deinit(void)
{
	int i;

	for (i = CONN_IDX_START; i < MAX_CONNECTIONS; i++)
		bh_do_disconnect(i);
}

#define MAX_RETRY_COUNT 3
int bh_request(int conn_idx, void *cmd, unsigned int clen,
	       const void *data, unsigned int dlen, u64 seq)
{
	int ret;
	u32 retry_count;
	u64 seq_response = 0;

	ret = bh_send_message(conn_idx, cmd, clen, data, dlen, seq);
	if (ret)
		return ret;

	for (retry_count = 0; retry_count < MAX_RETRY_COUNT; retry_count++) {
		ret = bh_recv_message(conn_idx, &seq_response);
		if (ret) {
			pr_debug("failed to recv msg = %d\n", ret);
			continue;
		}

		if (seq_response != seq) {
			pr_debug("recv message with seq=%llu != seq_response=%llu\n",
				 seq, seq_response);
			continue;
		}

		pr_debug("recv message with try=%d seq=%llu\n",
			 retry_count, seq_response);
		break;
	}

	if (retry_count == MAX_RETRY_COUNT) {
		pr_err("out of retry attempts\n");
		ret = -EFAULT;
	}

	return ret;
}

int bhp_init_internal(void)
{
	if (bhp_is_initialized())
		return 0;

	/* step 1: init connections to each process */
	bh_connections_init();

	/* RESET flow removed to allow JHI and KDI to coexist */
	/* this assignment is atomic operation */
	WRITE_ONCE(init_state, INITED);

	return 0;
}

int bhp_deinit_internal(void)
{
	mutex_enter(bhm_gInit);

	if (bhp_is_initialized()) {
		/* RESET flow removed to allow JHI and KDI to coexist */
		bh_connections_deinit();
		WRITE_ONCE(init_state, DEINITED);
	}

	mutex_exit(bhm_gInit);

	return 0;
}
