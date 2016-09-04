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

#ifndef _BH_ACP_INTERNAL_H
#define _BH_ACP_INTERNAL_H

#include "bh_types.h"
#include "bh_errcode.h"
#include "bh_acp_format.h"

/* Intel CSS Header + CSS Cypto Block which prefixes each signed ACP pkg */
#define BH_ACP_CSS_HEADER_LENGTH    (128 + 520)

/**
 * struct pack_reader - hold a reference of raw pack and read
 *     items with alignment support
 * @cur   : current reader position
 * @head  : buffer  head
 * @total : bytes in the buffer
 */
struct pack_reader {
	const char *cur;
	const char *head;
	unsigned int total;
};

int pr_init(struct pack_reader *pr, const char *data, unsigned int n);

bool pr_is_end(struct pack_reader *pr);

int acp_load_pack_head(struct pack_reader *pr, struct ac_pack_header **head);

int acp_load_ins_jta(struct pack_reader *pr, struct ac_ins_jta_pack *pack);

int acp_load_ta_pack(struct pack_reader *pr, char **ta_pack);

int acp_load_ins_jta_prop(struct pack_reader *pr, struct ac_ins_jta_prop *pack);

#endif /*_BH_ACP_INTERNAL_H */
