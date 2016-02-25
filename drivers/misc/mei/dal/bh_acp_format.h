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

#ifndef BH_ACP_FORMAT_H
#define BH_ACP_FORMAT_H

#include "bh_shared_types.h"

/* make sure those limitation values are adjusted to real world */
#define BH_MAX_ACP_INS_REASONS_LENGTH 1024
#define BH_MAX_ACP_USED_SERVICES 20
#define BH_MAX_ACP_PROPS_LENGTH 2048

enum ac_cmd_id {

	AC_CMD_INVALID,
	AC_INSTALL_SD,
	AC_UNINSTALL_SD,
	AC_INSTALL_JTA,
	AC_UNINSTALL_JTA,
	AC_INSTALL_NTA,
	AC_UNINSTALL_NTA,
	AC_UPDATE_SVL,
	AC_INSTALL_JTA_PROP,
	AC_CMD_NUM
};

enum ac_ta_type {
	AC_TA_TYPE_BOTH,
	AC_TA_TYPE_JAVA,
	AC_TA_TYPE_NATIVE,
};

#pragma pack(1)

struct ac_pack_header {
	/*ACP Header*/
	u8 magic[4];
	u8 version;
	u8 little_endian;
	u16 reserved;
	u32 size;/*total package size in byte except signature*/
	u32 cmd_id;
	u32 svn;

	/*Section Index*/
	u32 idx_num;
	u32 idx_condition;
	/*TBD: BH_U32 idx_encrypt;*/
	u32 idx_data;
};

struct ac_name {
	u8 len;/*the size of data in byte*/
	s8 data[0];
};

struct bh_ta_id_list {
	u32 num;
	struct bh_ta_id list[0];
};

/*
 * Firmware properties are formatted as "type\0key\0value\0"
 * Example: "string\0name\0Tom\0int\0Age\013\0"
 */
struct bh_prop_list {
	u32 num; /*number of properties*/
	u32 len; /*the size of data in byte*/
	s8 data[0];
};

struct ac_ins_reasons {
	/* NOTE: len means the amount of items in data,
	 * when counting total bytes, you need to multiply
	 * it with sizeof(data[0])
	 */
	u32 len;
	u32 data[0];
};

#pragma pack()

#pragma pack(1)

/*
 * below structures are the parsing result that application layer should use
 * they are in-memory representation of admin packages
 */
struct ac_pack {
	struct ac_pack_header *head;
	/*the type of data depends on head->cmd_id*/
	char data[0];
/*--ACSignature is appendeded after command package*/
};

struct ac_ins_ta_header {
	struct bh_ta_id ta_id;
	u32 ta_svn;
	u8 hash_alg_type;
	u8 ta_reserved[3];
	struct bh_pack_hash hash;
};
/*header struct shared between JTA and NTA*/

struct ac_ins_jta_pack {
	struct bh_prop_list *ins_cond;
	struct ac_ins_ta_header *head;
};

struct ac_ins_jta_prop_header {
	u32 mem_quota;
	u8 ta_encrypted;
	u8 padding;
	u16 allowed_inter_session_num;
	u64 ac_groups;
	u32 timeout;
};

struct ac_ins_jta_prop {
	struct ac_ins_jta_prop_header *head;
	struct ac_ins_reasons *post_reasons;
	struct ac_ins_reasons *reg_reasons;
	struct bh_prop_list *prop;
	struct bh_ta_id_list *used_service_list;
};

#pragma pack()

#endif
