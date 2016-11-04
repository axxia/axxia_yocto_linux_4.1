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
#include <linux/slab.h>
#include <linux/uuid.h>

#include <linux/mei_cl_bus.h>
#include "dal_dev.h"

struct dal_access_policy {
	struct list_head list;
	uuid_be app_id;
	void *owner;
};

static struct list_head *dal_dev_get_access_list(struct dal_device *ddev)
{
	return dev_get_drvdata(&ddev->dev);
}

static struct dal_access_policy *
dal_access_policy_alloc(uuid_be app_id, void *owner)
{
	struct dal_access_policy *e;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return NULL;

	INIT_LIST_HEAD(&e->list);
	e->app_id = app_id;
	e->owner = owner;

	return e;
}

static struct dal_access_policy *
dal_access_policy_find(struct list_head *access_list, uuid_be app_id)
{
	struct dal_access_policy *e;

	list_for_each_entry(e, access_list, list) {
		if (!uuid_be_cmp(e->app_id, app_id))
			return e;
	}
	return NULL;
}

int dal_access_policy_add(struct dal_device *ddev, uuid_be app_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, app_id);
	if (e) {
		if (!e->owner)
			return -EPERM;

		return -EEXIST;
	}

	e = dal_access_policy_alloc(app_id, owner);
	if (!e)
		return -ENOMEM;

	list_add_tail(&e->list, access_list);
	return 0;
}

int dal_access_policy_remove(struct dal_device *ddev,
			     uuid_be app_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, app_id);
	if (!e)
		return -ENOENT;

	if (!e->owner || e->owner != owner)
		return -EPERM;

	list_del(&e->list);
	kfree(e);
	return 0;
}

int dal_access_policy_allowed(struct dal_device *ddev,
			      uuid_be app_id, void *owner)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e;

	e = dal_access_policy_find(access_list, app_id);
	if (!e)
		return 0;

	if (e->owner && e->owner != owner)
		return -EPERM;

	return 0;
}

void dal_access_list_free(struct dal_device *ddev)
{
	struct list_head *access_list = dal_dev_get_access_list(ddev);
	struct dal_access_policy *e, *n;

	if  (!access_list)
		return;

	list_for_each_entry_safe(e, n, access_list, list) {
		list_del(&e->list);
		kfree(e);
	}

	kfree(access_list);
	dev_set_drvdata(&ddev->dev, NULL);
}

int dal_access_list_init(struct dal_device *ddev)
{
	struct list_head *access_list;

	access_list = kzalloc(sizeof(*access_list), GFP_KERNEL);
	if (!access_list)
		return -ENOMEM;

	INIT_LIST_HEAD(access_list);
	dev_set_drvdata(&ddev->dev, access_list);

	return 0;
}
