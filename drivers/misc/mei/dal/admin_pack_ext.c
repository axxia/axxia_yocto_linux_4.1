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
#include <linux/errno.h>

#include "bh_acp_exp.h"
#include "bh_acp_internal.h"

static int acp_load_pack(const char *raw_pack, unsigned int size,
			 int cmd_id, struct ac_pack *pack)
{
	int ret;
	struct pack_reader pr;
	struct ac_ins_jta_pack_ext *pack_ext;
	struct ac_ins_jta_prop_ext *prop_ext;

	ret = pr_init(&pr, raw_pack, size);
	if (ret)
		return ret;

	if (cmd_id != AC_INSTALL_JTA_PROP) {
		ret = acp_load_pack_head(&pr, &pack->head);
		if (ret)
			return ret;
	}

	if (cmd_id != AC_INSTALL_JTA_PROP && cmd_id != pack->head->cmd_id)
		return -EINVAL;

	switch (cmd_id) {
	case AC_INSTALL_JTA:
		pack_ext = (struct ac_ins_jta_pack_ext *)pack;
		ret = acp_load_ins_jta(&pr, &pack_ext->cmd_pack);
		if (ret)
			break;
		ret = acp_load_ta_pack(&pr, &pack_ext->ta_pack);
		break;
	case AC_INSTALL_JTA_PROP:
		prop_ext = (struct ac_ins_jta_prop_ext *)pack;
		ret = acp_load_ins_jta_prop(&pr, &prop_ext->cmd_pack);
		if (ret)
			break;
		/* Note: the next section is JEFF file,
		 * and not ta_pack(JTA_properties+JEFF file),
		 * but we could reuse the ACP_load_ta_pack() here.
		 */
		ret = acp_load_ta_pack(&pr, &prop_ext->jeff_pack);
		break;
	default:
		return -EINVAL;
	}

	if (!pr_is_end(&pr))
		return -EINVAL;

	return ret;
}

int acp_pload_ins_jta(const void *raw_data, unsigned int size,
		      struct ac_ins_jta_pack_ext *pack)
{
	int ret;

	if (!raw_data || size <= BH_ACP_CSS_HEADER_LENGTH || !pack)
		return -EINVAL;

	ret = acp_load_pack((const char *)raw_data + BH_ACP_CSS_HEADER_LENGTH,
			    size - BH_ACP_CSS_HEADER_LENGTH,
			    AC_INSTALL_JTA, (struct ac_pack *)pack);

	return ret;
}

int acp_pload_ins_jta_prop(const void *raw_data, unsigned int size,
			   struct ac_ins_jta_prop_ext *pack)
{
	if (!raw_data || !pack)
		return -EINVAL;

	return acp_load_pack(raw_data, size, AC_INSTALL_JTA_PROP,
			    (struct ac_pack *)pack);
}

int acp_get_cmd_id(const void *raw_data, unsigned int size, int *cmd_id)
{
	int ret;
	struct pack_reader pr;
	struct ac_pack_header *ph = NULL;

	if (!raw_data || size <= BH_ACP_CSS_HEADER_LENGTH || !cmd_id)
		return BHE_BAD_PARAMETER;

	*cmd_id = AC_CMD_INVALID;

	ret = pr_init(&pr, raw_data + BH_ACP_CSS_HEADER_LENGTH,
		      size - BH_ACP_CSS_HEADER_LENGTH);
	if (ret != BH_SUCCESS)
		return BHE_INVALID_BPK_FILE;

	ret = acp_load_pack_head(&pr, &ph);
	if (ret != BH_SUCCESS)
		return ret;

	*cmd_id = ph->cmd_id;
	return BH_SUCCESS;
}
