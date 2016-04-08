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
char i915_scheduler_queue_status_chr(enum i915_scheduler_queue_status status);
const char *i915_scheduler_queue_status_str(
				enum i915_scheduler_queue_status status);

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
const char *i915_qe_state_str(struct i915_scheduler_queue_entry *node);

struct i915_scheduler_stats_nodes {
	uint32_t counts[I915_SQS_MAX + 1];
};

struct i915_scheduler_stats {
	/* Batch buffer counts: */
	uint32_t queued;
	uint32_t submitted;
	uint32_t completed;
	uint32_t expired;

	/* Other stuff: */
	uint32_t flush_obj;
	uint32_t flush_req;
	uint32_t flush_stamp;
	uint32_t flush_all;
	uint32_t flush_bump;
	uint32_t flush_submit;

	uint32_t exec_early;
	uint32_t exec_again;
	uint32_t exec_dead;
	uint32_t kill_flying;
	uint32_t kill_queued;
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
	uint32_t file_queue_max;
	uint32_t dump_flags;

	/* Statistics: */
	struct i915_scheduler_stats stats[I915_NUM_ENGINES];
};

/* Flag bits for i915_scheduler::flags */
enum {
	/* Internal state */
	I915_SF_INTERRUPTS_ENABLED  = (1 << 0),
	I915_SF_SUBMITTING          = (1 << 1),

	/* Dump/debug flags */
	I915_SF_DUMP_FORCE          = (1 << 8),
	I915_SF_DUMP_DETAILS        = (1 << 9),
	I915_SF_DUMP_DEPENDENCIES   = (1 << 10),
	I915_SF_DUMP_SEQNO          = (1 << 11),

	I915_SF_DUMP_MASK           = I915_SF_DUMP_FORCE        |
				      I915_SF_DUMP_DETAILS      |
				      I915_SF_DUMP_DEPENDENCIES |
				      I915_SF_DUMP_SEQNO,
};
const char *i915_scheduler_flag_str(uint32_t flags);

bool i915_scheduler_is_enabled(struct drm_device *dev);
int i915_scheduler_init(struct drm_device *dev);
void i915_scheduler_closefile(struct drm_device *dev, struct drm_file *file);
void i915_scheduler_reset_cleanup(struct intel_engine_cs *engine);
void i915_scheduler_clean_node(struct i915_scheduler_queue_entry *node);
int i915_scheduler_queue_execbuffer(struct i915_scheduler_queue_entry *qe);
bool i915_scheduler_notify_request(struct drm_i915_gem_request *req);
void i915_scheduler_wakeup(struct drm_device *dev);
bool i915_scheduler_is_engine_flying(struct intel_engine_cs *engine);
void i915_scheduler_work_handler(struct work_struct *work);
int i915_scheduler_flush(struct intel_engine_cs *engine, bool is_locked);
int i915_scheduler_flush_stamp(struct intel_engine_cs *engine,
			       unsigned long stamp, bool is_locked);
int i915_scheduler_dump(struct intel_engine_cs *engine,
			const char *msg);
int i915_scheduler_dump_all(struct drm_device *dev, const char *msg);
bool i915_scheduler_is_mutex_required(struct drm_i915_gem_request *req);
bool i915_scheduler_is_request_batch_buffer(struct drm_i915_gem_request *req);
int i915_scheduler_query_stats(struct intel_engine_cs *engine,
			       struct i915_scheduler_stats_nodes *stats);
bool i915_scheduler_file_queue_wait(struct drm_file *file);

#endif  /* _I915_SCHEDULER_H_ */
