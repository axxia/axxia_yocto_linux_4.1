/*
 * Copyright (c) 2014 Intel Corporation
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

#ifndef _I915_SCHEDULER_H_
#define _I915_SCHEDULER_H_

enum i915_scheduler_queue_status {
	/* Limbo: */
	I915_SQS_NONE = 0,
	/* Not yet submitted to hardware: */
	I915_SQS_QUEUED,
	/* Popped from queue, ready to fly: */
	I915_SQS_POPPED,
	/* Sent to hardware for processing: */
	I915_SQS_FLYING,
	/* Finished processing on the hardware: */
	I915_SQS_COMPLETE,
	/* Killed by watchdog or catastrophic submission failure: */
	I915_SQS_DEAD,
	/* Limit value for use with arrays/loops */
	I915_SQS_MAX
};

#define I915_SQS_IS_QUEUED(node)	(((node)->status == I915_SQS_QUEUED))
#define I915_SQS_IS_FLYING(node)	(((node)->status == I915_SQS_FLYING))
#define I915_SQS_IS_COMPLETE(node)	(((node)->status == I915_SQS_COMPLETE) || \
					 ((node)->status == I915_SQS_DEAD))

struct i915_scheduler_obj_entry {
	struct drm_i915_gem_object *obj;
	bool read_only;
};

struct i915_scheduler_queue_entry {
	/* Any information required to submit this batch buffer to the hardware */
	struct i915_execbuffer_params params;

	/* -1023 = lowest priority, 0 = default, 1023 = highest */
	int32_t priority;
	bool bumped;

	/* Objects referenced by this batch buffer */
	struct i915_scheduler_obj_entry *objs;
	int num_objs;

	/* Batch buffers this one is dependent upon */
	struct i915_scheduler_queue_entry **dep_list;
	int num_deps;

	enum i915_scheduler_queue_status status;
	unsigned long stamp;

	/* List of all scheduler queue entry nodes */
	struct list_head link;
};

struct i915_scheduler {
	struct list_head node_queue[I915_NUM_ENGINES];
	uint32_t flags[I915_NUM_ENGINES];
	spinlock_t lock;

	/* Tuning parameters: */
	int32_t priority_level_min;
	int32_t priority_level_max;
	int32_t priority_level_bump;
	int32_t priority_level_preempt;
	uint32_t min_flying;
};

/* Flag bits for i915_scheduler::flags */
enum {
	I915_SF_INTERRUPTS_ENABLED  = (1 << 0),
	I915_SF_SUBMITTING          = (1 << 1),
};

bool i915_scheduler_is_enabled(struct drm_device *dev);
int i915_scheduler_init(struct drm_device *dev);
void i915_scheduler_closefile(struct drm_device *dev, struct drm_file *file);
void i915_scheduler_clean_node(struct i915_scheduler_queue_entry *node);
int i915_scheduler_queue_execbuffer(struct i915_scheduler_queue_entry *qe);
bool i915_scheduler_notify_request(struct drm_i915_gem_request *req);
void i915_scheduler_wakeup(struct drm_device *dev);

#endif  /* _I915_SCHEDULER_H_ */