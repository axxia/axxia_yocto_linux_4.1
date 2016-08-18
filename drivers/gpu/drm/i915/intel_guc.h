/*
 * Copyright © 2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */
#ifndef _INTEL_GUC_H_
#define _INTEL_GUC_H_

#include "intel_guc_fwif.h"
#include "i915_guc_reg.h"

struct i915_guc_client {
	struct intel_guc *guc;
	struct intel_context *owner;
	struct drm_i915_gem_object *client_obj;
	uint64_t client_gtt;		/* GTT offset of client_obj	*/
	uint32_t priority;
	uint32_t ctx_index;

	uint32_t doorbell_offset;	/* offset within client obj	*/
	uint32_t proc_desc_offset;	/* offset within client_obj	*/
	uint32_t wq_offset;		/* offset within client_obj	*/
	uint32_t wq_size;
	uint32_t doorbell_cookie;
	uint16_t doorbell_id;
	uint16_t padding;		/* Maintain alignment		*/

	spinlock_t wq_lock;		/* Protects all data below	*/
	uint32_t wq_tail;
	uint32_t wq_head;

	/* GuC submission statistics & status */
	uint64_t submissions[GUC_MAX_ENGINES_NUM];
	uint32_t q_fail;
	uint32_t b_fail;
	int retcode;
};

enum intel_uc_fw_status {
	UC_FIRMWARE_FAIL = -1,
	UC_FIRMWARE_NONE = 0,
	UC_FIRMWARE_PENDING,
	UC_FIRMWARE_SUCCESS
};

#define UC_FW_TYPE_GUC		0
#define UC_FW_TYPE_HUC		1

/*
 * This structure encapsulates all the data needed during the process
 * of fetching, caching, and loading the firmware image.
 */
struct intel_uc_fw {
	struct drm_device *		uc_dev;
	const char *			uc_fw_path;
	size_t				uc_fw_size;
	struct drm_i915_gem_object *	uc_fw_obj;
	enum intel_uc_fw_status		fetch_status;
	enum intel_uc_fw_status		load_status;

	uint16_t			major_ver_wanted;
	uint16_t			minor_ver_wanted;
	uint16_t			major_ver_found;
	uint16_t			minor_ver_found;

	uint32_t fw_type;
	uint32_t header_size;
	uint32_t header_offset;
	uint32_t rsa_size;
	uint32_t rsa_offset;
	uint32_t ucode_size;
	uint32_t ucode_offset;
};

struct intel_guc {
	struct intel_uc_fw guc_fw;
	uint32_t log_flags;
	struct drm_i915_gem_object *log_obj;

	struct drm_i915_gem_object *ads_obj;

	struct drm_i915_gem_object *ctx_pool_obj;
	struct ida ctx_ids;

	struct i915_guc_client *execbuf_client;
	struct i915_guc_client *preempt_client;

	DECLARE_BITMAP(doorbell_bitmap, GUC_MAX_DOORBELLS);
	uint32_t db_cacheline;		/* Cyclic counter mod pagesize	*/

	/* Action status & statistics */
	uint64_t action_count;		/* Total commands issued	*/
	uint32_t action_cmd;		/* Last command word		*/
	uint32_t action_status;		/* Last return status		*/

	uint32_t action_fail_count;	/* Total number of failures	*/
	uint32_t action_fail_cmd;	/* Last failed command		*/
	uint32_t action_fail_status;	/* Last bad return status	*/
	int32_t action_err;		/* Last (nonzero) error code	*/

	/* Submission status & statistics */
	uint64_t submissions[GUC_MAX_ENGINES_NUM];
	uint32_t last_seqno[GUC_MAX_ENGINES_NUM];
	uint32_t failures[GUC_MAX_ENGINES_NUM];
	uint32_t preemptions[GUC_MAX_ENGINES_NUM];
	uint32_t last_preempt[GUC_MAX_ENGINES_NUM];
	uint32_t preempt_failures[GUC_MAX_ENGINES_NUM];
};

/* intel_guc_loader.c */
extern void intel_guc_ucode_init(struct drm_device *dev);
extern int intel_guc_ucode_load(struct drm_device *dev);
extern void intel_guc_ucode_fini(struct drm_device *dev);
extern const char *intel_uc_fw_status_repr(enum intel_uc_fw_status status);
extern int intel_guc_suspend(struct drm_device *dev);
extern int intel_guc_resume(struct drm_device *dev);
void intel_uc_fw_fetch(struct drm_device *dev, struct intel_uc_fw *uc_fw);

/* i915_guc_submission.c */
int i915_guc_submission_init(struct drm_device *dev);
int i915_guc_submission_enable(struct drm_device *dev);
int i915_guc_submit(struct i915_guc_client *client,
		    struct drm_i915_gem_request *rq);
void i915_guc_submission_disable(struct drm_device *dev);
void i915_guc_submission_fini(struct drm_device *dev);
int i915_guc_wq_check_space(struct i915_guc_client *client);
int i915_guc_sample_forcewake(struct drm_device *dev);

#endif
