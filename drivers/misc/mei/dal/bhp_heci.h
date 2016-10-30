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
 * @file  bhp_heci.h
 * @brief This file defines heci command and response format
 * for Beihai Host Proxy (BHP) module.
 * @author
 * @version
 *
 */

#ifndef __BHP_HECI_H
#define __BHP_HECI_H

#include <linux/types.h>
#include <linux/uuid.h>

#include "bh_types.h"
#include "bh_errcode.h"

#define BHP_MSG_MAGIC_LENGTH (4)
#define BHP_MSG_CMD_MAGIC "\xff\xa3\xaa\x55"
#define BHP_MSG_RESPONSE_MAGIC "\xff\xa5\xaa\x55"

enum bhp_command_id {
	BHP_CMD_INIT = 0,
	BHP_CMD_DEINIT,
	BHP_CMD_VERIFY_JAVATA,
	BHP_CMD_DOWNLOAD_JAVATA,
	BHP_CMD_OPEN_JTASESSION,
	BHP_CMD_CLOSE_JTASESSION,
	BHP_CMD_FORCECLOSE_JTASESSION,
	BHP_CMD_SENDANDRECV,
	BHP_CMD_SENDANDRECV_INTERNAL,
	BHP_CMD_RUN_NATIVETA,
	BHP_CMD_STOP_NATIVETA,
	BHP_CMD_OPEN_SDSESSION,
	BHP_CMD_CLOSE_SDSESSION,
	BHP_CMD_INSTALL_SD,
	BHP_CMD_UNINSTALL_SD,
	BHP_CMD_INSTALL_JAVATA,
	BHP_CMD_UNINSTALL_JAVATA,
	BHP_CMD_INSTALL_NATIVETA,
	BHP_CMD_UNINSTALL_NATIVETA,
	BHP_CMD_LIST_SD,
	BHP_CMD_LIST_TA,
	BHP_CMD_RESET,
	BHP_CMD_LIST_TA_PROPERTIES,
	BHP_CMD_QUERY_TA_PROPERTY,
	BHP_CMD_LIST_JTA_SESSIONS,
	BHP_CMD_LIST_TA_PACKAGES,
	BHP_CMD_GET_ISD,
	BHP_CMD_GET_SD_BY_TA,
	BHP_CMD_LAUNCH_VM,
	BHP_CMD_CLOSE_VM,
	BHP_CMD_QUERY_NATIVETA_STATUS,
	BHP_CMD_QUERY_SD_STATUS,
	BHP_CMD_LIST_DOWNLOADED_NTA,
	BHP_CMD_UPDATE_SVL,
	BHP_CMD_CHECK_SVL_TA_BLOCKED_STATE,
	BHP_CMD_QUERY_TEE_METADATA,
	BHP_CMD_MAX
};

struct transport_msg_header {
	u8 magic[BHP_MSG_MAGIC_LENGTH];
	u32 length;
};

struct bhp_command_header {
	struct transport_msg_header h;
	u64 seq;
	enum bhp_command_id id;
	u8 pad[4];
	s8 cmd[0];
};

struct bhp_response_header {
	struct transport_msg_header h;
	u64 seq;
	u64 addr;
	int code;
	u8 pad[4];
	s8 data[0];
};

struct bhp_download_javata_cmd {
	uuid_be appid;
	s8 appblob[0];
};

struct bhp_open_jtasession_cmd {
	uuid_be appid;
	s8 buffer[0];
};

struct bhp_close_jtasession_cmd {
	u64 ta_session_id;
};

struct bhp_snr_cmd {
	u64 ta_session_id;
	s32 command;
	u32 outlen;
	s8 buffer[0];
};

struct bhp_check_svl_ta_blocked_state_cmd {
	uuid_be taid;
};

struct bhp_reset_launcher_response {
	u32 count; /* count of svm heci ports */
	s32 vm_heci_port_list[0];
};

struct bhp_get_sd_by_ta_cmd {
	uuid_be taid;
};

struct bhp_get_sd_by_ta_response {
	uuid_be sdid;
};

struct bhp_get_isd_response {
	uuid_be sdid;
};

struct bhp_snr_response {
	/* field response comes from java BIG endian */
	__be32 response;
	s8 buffer[0];
};

struct bhp_snr_bof_response {
	/* field response comes from java BIG endian */
	__be32 response;
	__be32 request_length;
};

struct bhp_list_ta_packages_response {
	u32 count;
	uuid_be app_ids[0];
};

#endif /* __BHP_HECI_H */
