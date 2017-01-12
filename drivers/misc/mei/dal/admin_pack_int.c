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
 * @file  admin_pack_int.cpp
 * @brief This file implements internal atomic api of admin command parsing
 *        The counter part which generate admin package is BPKT
 * @author Wenlong Feng(wenlong.feng@intel.com)
 */

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/errno.h>

#include "bh_errcode.h"
#include "bh_acp_format.h"
#include "bh_acp_internal.h"
#include "bh_acp_exp.h"

#define PR_ALIGN 4

int pr_init(struct pack_reader *pr, const char *data, unsigned int n)
{
	/* check integer overflow */
	if ((size_t)data > SIZE_MAX - n)
		return -EINVAL;

	pr->cur = data;
	pr->head = data;
	pr->total = n;
	return 0;
}

static int pr_8b_align_move(struct pack_reader *pr, size_t n_move)
{
	unsigned long offset;
	const char *new_cur = pr->cur + n_move;
	size_t len_from_head = new_cur - pr->head;

	if ((size_t)pr->cur > SIZE_MAX - n_move || new_cur < pr->head)
		return -EINVAL;

	offset = ((8 - (len_from_head & 7)) & 7);
	if ((size_t)new_cur > SIZE_MAX - offset)
		return -EINVAL;

	new_cur = new_cur + offset;
	if (new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;
	return 0;
}

static int pr_align_move(struct pack_reader *pr, size_t n_move)
{
	const char *new_cur = pr->cur + n_move;
	size_t len_from_head = new_cur - pr->head;
	size_t offset;

	if ((size_t)pr->cur > SIZE_MAX - n_move || new_cur < pr->head)
		return -EINVAL;

	offset = ((4 - (len_from_head & 3)) & 3);
	if ((size_t)new_cur > SIZE_MAX - offset)
		return -EINVAL;

	new_cur = new_cur + offset;
	if (new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;
	return 0;
}

static int pr_move(struct pack_reader *pr, size_t n_move)
{
	const char *new_cur = pr->cur + n_move;

	/* integer overflow or out of acp pkg size */
	if ((size_t)pr->cur > SIZE_MAX - n_move ||
			new_cur > pr->head + pr->total)
		return -EINVAL;

	pr->cur = new_cur;

	return 0;
}

static bool pr_is_safe_to_read(const struct pack_reader *pr, size_t n_move)
{
	/* pointer overflow */
	if ((size_t)pr->cur > SIZE_MAX - n_move)
		return false;

	if (pr->cur + n_move > pr->head + pr->total)
		return false;

	return true;
}

bool pr_is_end(struct pack_reader *pr)
{
	return (pr->cur == pr->head + pr->total);
}

static int acp_load_reasons(struct pack_reader *pr,
			    struct ac_ins_reasons **reasons)
{
	size_t len;
	struct ac_ins_reasons *r;

	if (!pr_is_safe_to_read(pr, sizeof(*r)))
		return -EINVAL;

	r = (struct ac_ins_reasons *)pr->cur;

	if (r->len > BH_MAX_ACP_INS_REASONS_LENGTH)
		return -EINVAL;

	len = sizeof(*r) + r->len * sizeof(r->data[0]);
	if (!pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*reasons = r;
	return pr_align_move(pr, len);
}

static int acp_load_taid_list(struct pack_reader *pr,
			      struct bh_ta_id_list **taid_list)
{
	size_t len;
	struct bh_ta_id_list *t;

	if (!pr_is_safe_to_read(pr, sizeof(*t)))
		return -EINVAL;

	t = (struct bh_ta_id_list *)pr->cur;
	if (t->num > BH_MAX_ACP_USED_SERVICES)
		return -EINVAL;

	len = sizeof(*t) + t->num * sizeof(t->list[0]);

	if (!pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*taid_list = t;
	return pr_align_move(pr, len);
}

static int acp_load_prop(struct pack_reader *pr, struct bh_prop_list **prop)
{
	size_t len;
	struct bh_prop_list *p;

	if (!pr_is_safe_to_read(pr, sizeof(*p)))
		return -EINVAL;

	p = (struct bh_prop_list *)pr->cur;
	if (p->len > BH_MAX_ACP_PROPS_LENGTH)
		return -EINVAL;

	len = sizeof(*p) + p->len * sizeof(p->data[0]);

	if (!pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*prop = p;
	return pr_align_move(pr, len);
}

int acp_load_ta_pack(struct pack_reader *pr, char **ta_pack)
{
	size_t len;
	char *t;

	/*8 byte align to obey jeff rule*/
	if (pr_8b_align_move(pr, 0))
		return -EINVAL;

	t = (char *)pr->cur;

	/*
	 *assume ta pack is the last item of one package,
	 *move cursor to the end directly
	 */
	if (pr->cur > pr->head + pr->total)
		return -EINVAL;

	len = pr->head + pr->total - pr->cur;
	if (!pr_is_safe_to_read(pr, len))
		return -EINVAL;

	*ta_pack = t;
	return pr_move(pr, len);
}

static int acp_load_ins_jta_prop_head(struct pack_reader *pr,
				      struct ac_ins_jta_prop_header **head)
{
	if (!pr_is_safe_to_read(pr, sizeof(struct ac_ins_jta_prop_header)))
		return -EINVAL;

	*head = (struct ac_ins_jta_prop_header *)pr->cur;
	return pr_align_move(pr, sizeof(struct ac_ins_jta_prop_header));
}

int acp_load_ins_jta_prop(struct pack_reader *pr, struct ac_ins_jta_prop *pack)
{
	int ret;

	ret = acp_load_ins_jta_prop_head(pr, &pack->head);
	if (ret)
		goto out;

	ret = acp_load_reasons(pr, &pack->post_reasons);
	if (ret)
		goto out;

	ret = acp_load_reasons(pr, &pack->reg_reasons);
	if (ret)
		goto out;

	ret = acp_load_prop(pr, &pack->prop);
	if (ret)
		goto out;

	ret = acp_load_taid_list(pr, &pack->used_service_list);
	if (ret)
		return ret;

out:
	return ret;
}

static int acp_load_ins_jta_head(struct pack_reader *pr,
				 struct ac_ins_ta_header **head)
{
	if (!pr_is_safe_to_read(pr, sizeof(struct ac_ins_ta_header)))
		return -EINVAL;

	*head = (struct ac_ins_ta_header *)pr->cur;
	return pr_align_move(pr, sizeof(struct ac_ins_ta_header));
}

int acp_load_ins_jta(struct pack_reader *pr, struct ac_ins_jta_pack *pack)
{
	int ret;

	ret = acp_load_prop(pr, &pack->ins_cond);
	if (ret)
		return ret;

	ret = acp_load_ins_jta_head(pr, &pack->head);

	return ret;
}

int acp_load_pack_head(struct pack_reader *pr, struct ac_pack_header **head)
{
	if (!pr_is_safe_to_read(pr, sizeof(struct ac_pack_header)))
		return -EINVAL;

	*head = (struct ac_pack_header *)pr->cur;
	return pr_align_move(pr, sizeof(struct ac_pack_header));
}
