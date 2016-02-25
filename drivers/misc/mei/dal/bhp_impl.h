/******************************************************************************
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

#ifndef _BHP_IMPL_H_
#define _BHP_IMPL_H_

#include "bh_shared_types.h"
#include "bhp_exp.h"
#include "bh_acp_exp.h"
#include "bhp_heci.h"
#include <linux/list.h>
#include <linux/slab.h>

/** struct bh_response_record
 *   represents a beihai response record
 *
 * @code: the response code from firmware
 * @length: length of the response buffer
 * @buffer: the response buffer
 * @addr: remote address in firmware
 * @is_session: whether this record relates with session
 * @killed: whether this session is killed or not,
 * valid only if is_session is true
 * @count: the count of users who are using this session,
 * valid only for is_session is 1
 */
struct bh_response_record {
	int code;
	unsigned int length;
	void *buffer;
	u64 addr;
	bool is_session;
	bool killed;
	unsigned int count;
};

/** struct bh_connection_item
 *   represents a vm connection
 *
 * @handle: physical connection handle
 * @rr_map_list_header: the list
 * @conn_count: VM connection counter, only valid for VM
 * @sdid: remote the sd id it serves, only valid for VM
 */
struct bh_connection_item {
	unsigned int handle;
	struct list_head rr_map_list_header;
	unsigned int conn_count;
	struct bh_sd_id sdid;
};

/* maximum concurrent activities on one session */
#define MAX_SESSION_LIMIT 20

/* heci command header buffer size in bytes */
#define CMDBUF_SIZE 100

/* TODO: review to avoid seq conflicts */
# define MSG_SEQ_START_NUMBER (1UL << 32)

/**
 * enum bhp_connection_index -
 *   represents a connection index to the different clients
 *
 * @CONN_IDX_IVM:
 * @CONN_IDX_SDM:
 * @CONN_IDX_LAUNCHER:
 * @CONN_IDX_SVM:
 */
enum bhp_connection_index {
	CONN_IDX_START = 0,
	CONN_IDX_IVM = 0,
	CONN_IDX_SDM = 1,
	CONN_IDX_LAUNCHER = 2,
	CONN_IDX_SVM = 3,
	MAX_CONNECTIONS
};

/**
 * enum bhp_state - represents the current state of bhp.
 *
 * @DEINITED:
 * @INITED:
 */
enum bhp_state {
	DEINITED = 0,
	INITED = 1,
};

/* whether BHP is inited or not */
bool bhp_is_initialized(void);

/* Add a rr to rrmap and return a new seq number. */
u64 rrmap_add(int conn_idx, struct bh_response_record *rr);

/* session enter with session handle seq */
struct bh_response_record *session_enter(int conn_idx, u64 seq,
					 int lock_session);

/* session exit */
void session_exit(int conn_idx, struct bh_response_record *session,
		  u64 seq, int unlock_session);

/* session close */
void session_close(int conn_idx, struct bh_response_record *session,
		   u64 seq, int unlock_session);

/* send one message through mei */
int bh_cmd_transfer(int conn_idx, void *cmd, unsigned int clen,
		    const void *data, unsigned int dlen, u64 seq);

/* open vm connection for sdid and increase vm connection counter by 1 */
int bh_do_open_vm(struct bh_sd_id sdid, int *conn_idx, int mode);

/* decrease vm connection counter by 1 */
int bh_do_close_vm(int conn_idx);

#define mutex_enter(s) {}
#define mutex_exit(s)  {}

#endif /* _BHP_IMPL_H_ */
