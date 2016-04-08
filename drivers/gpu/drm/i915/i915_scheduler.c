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

#include "i915_drv.h"
#include "intel_drv.h"
#include "i915_scheduler.h"

#define for_each_scheduler_node(node, id)				\
	list_for_each_entry((node), &scheduler->node_queue[(id)], link)

#define assert_scheduler_lock_held(scheduler)				\
	do {								\
		WARN_ONCE(!spin_is_locked(&(scheduler)->lock), "Spinlock not locked!");	\
	} while(0)

/**
 * i915_scheduler_is_enabled - Returns true if the scheduler is enabled.
 * @dev: DRM device
 */
bool i915_scheduler_is_enabled(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!i915.enable_scheduler)
		return false;

	return dev_priv->scheduler != NULL;
}

const char *i915_qe_state_str(struct i915_scheduler_queue_entry *node)
{
	uint32_t sched_flags = node->params.request->scheduler_flags;
	static char	str[50];
	char		*ptr = str;

	*(ptr++) = node->bumped ? 'B' : '-',
	*(ptr++) = (sched_flags & I915_REQ_SF_PREEMPT) ? 'P' : '-';
	*(ptr++) = (sched_flags & I915_REQ_SF_WAS_PREEMPT) ? 'p' : '-';
	*(ptr++) = i915_gem_request_completed(node->params.request) ? 'C' : '-';

	*ptr = 0;

	return str;
}

char i915_scheduler_queue_status_chr(enum i915_scheduler_queue_status status)
{
	switch (status) {
	case I915_SQS_NONE:
	return 'N';

	case I915_SQS_QUEUED:
	return 'Q';

	case I915_SQS_POPPED:
	return 'X';

	case I915_SQS_FLYING:
	return 'F';

	case I915_SQS_OVERTAKING:
	return 'O';

	case I915_SQS_PREEMPTED:
	return 'P';

	case I915_SQS_COMPLETE:
	return 'C';

	case I915_SQS_DEAD:
	return 'D';

	default:
	break;
	}

	return '?';
}

const char *i915_scheduler_queue_status_str(
				enum i915_scheduler_queue_status status)
{
	static char	str[50];

	switch (status) {
	case I915_SQS_NONE:
	return "None";

	case I915_SQS_QUEUED:
	return "Queued";

	case I915_SQS_POPPED:
	return "Popped";

	case I915_SQS_FLYING:
	return "Flying";

	case I915_SQS_OVERTAKING:
	return "Overtaking";

	case I915_SQS_PREEMPTED:
	return "Preempted";

	case I915_SQS_COMPLETE:
	return "Complete";

	case I915_SQS_DEAD:
	return "Dead";

	case I915_SQS_MAX:
	return "Invalid";

	default:
	break;
	}

	sprintf(str, "[Unknown_%d!]", status);
	return str;
}

const char *i915_scheduler_flag_str(uint32_t flags)
{
	static char str[100];
	char *ptr = str;

	*ptr = 0;

#define TEST_FLAG(flag, msg)						\
	do {								\
		if (flags & (flag)) {					\
			strcpy(ptr, msg);				\
			ptr += strlen(ptr);				\
			flags &= ~(flag);				\
		}							\
	} while (0)

	TEST_FLAG(I915_SF_INTERRUPTS_ENABLED, "IntOn|");
	TEST_FLAG(I915_SF_SUBMITTING,         "Submitting|");
	TEST_FLAG(I915_SF_DUMP_FORCE,         "DumpForce|");
	TEST_FLAG(I915_SF_DUMP_DETAILS,       "DumpDetails|");
	TEST_FLAG(I915_SF_DUMP_DEPENDENCIES,  "DumpDeps|");
	TEST_FLAG(I915_SF_DUMP_SEQNO,         "DumpSeqno|");

#undef TEST_FLAG

	if (flags) {
		sprintf(ptr, "Unknown_0x%X!", flags);
		ptr += strlen(ptr);
	}

	if (ptr == str)
		strcpy(str, "-");
	else
		ptr[-1] = 0;

	return str;
};

/**
 * i915_scheduler_init - Initialise the scheduler.
 * @dev: DRM device
 * Returns zero on success or -ENOMEM if memory allocations fail.
 */
int i915_scheduler_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	int r;

	if (scheduler)
		return 0;

	scheduler = kzalloc(sizeof(*scheduler), GFP_KERNEL);
	if (!scheduler)
		return -ENOMEM;

	spin_lock_init(&scheduler->lock);

	for (r = 0; r < I915_NUM_ENGINES; r++)
		INIT_LIST_HEAD(&scheduler->node_queue[r]);

	/* Default tuning values: */
	scheduler->priority_level_min     = -1023;
	scheduler->priority_level_max     = 1023;
	scheduler->priority_level_bump    = 50;
	scheduler->priority_level_preempt = 900;
	scheduler->min_flying             = 8;
	scheduler->file_queue_max         = 64;
	scheduler->dump_flags             = I915_SF_DUMP_FORCE   |
					    I915_SF_DUMP_DETAILS |
					    I915_SF_DUMP_SEQNO   |
					    I915_SF_DUMP_DEPENDENCIES;

	dev_priv->scheduler = scheduler;

	return 0;
}

/*
 * Add a popped node back in to the queue. For example, because the engine
 * was hung when execfinal() was called and thus the engine submission needs
 * to be retried later.
 */
static void i915_scheduler_node_requeue(struct i915_scheduler_queue_entry *node)
{
	WARN_ON(!I915_SQS_IS_FLYING(node));

	/* Seqno will be reassigned on relaunch */
	node->params.request->seqno = 0;
	node->status = I915_SQS_QUEUED;
	trace_i915_scheduler_unfly(node->params.engine, node);
	trace_i915_scheduler_node_state_change(node->params.engine, node);
}

/*
 * Give up on a node completely. For example, because it is causing the
 * engine to hang or is using some resource that no longer exists.
 */
static void i915_scheduler_node_kill(struct i915_scheduler *scheduler,
				     struct i915_scheduler_queue_entry *node)
{
	assert_scheduler_lock_held(scheduler);

	WARN_ON(I915_SQS_IS_COMPLETE(node));

	if (I915_SQS_IS_FLYING(node)) {
		trace_i915_scheduler_unfly(node->params.engine, node);
		scheduler->stats[node->params.engine->id].kill_flying++;
	} else
		scheduler->stats[node->params.engine->id].kill_queued++;

	node->status = I915_SQS_DEAD;
	trace_i915_scheduler_node_state_change(node->params.engine, node);
}

/* Mark a node as in flight on the hardware. */
static void i915_scheduler_node_fly(struct i915_scheduler_queue_entry *node)
{
	struct drm_i915_private *dev_priv = node->params.dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct intel_engine_cs *engine = node->params.engine;
	struct drm_i915_gem_request *req = node->params.request;

	assert_scheduler_lock_held(scheduler);

	WARN_ON(node->status != I915_SQS_POPPED);

	/*
	 * Add the node (which should currently be in state popped) to the
	 * front of the queue. This ensure that flying nodes are always held
	 * in hardware submission order.
	 */
	list_add(&node->link, &scheduler->node_queue[engine->id]);

	if (req->scheduler_flags & I915_REQ_SF_PREEMPT)
		node->status = I915_SQS_OVERTAKING;
	else
		node->status = I915_SQS_FLYING;

	trace_i915_scheduler_fly(engine, node);
	trace_i915_scheduler_node_state_change(engine, node);

	if (!(scheduler->flags[engine->id] & I915_SF_INTERRUPTS_ENABLED)) {
		bool success = true;

		success = engine->irq_get(engine);
		if (success)
			scheduler->flags[engine->id] |= I915_SF_INTERRUPTS_ENABLED;
	}
}

/* An untracked request is being launched ... */
void i915_scheduler_fly_request(struct drm_i915_gem_request *req)
{
	struct drm_i915_private *dev_priv = req->i915;
	struct i915_scheduler *scheduler = dev_priv->scheduler;

	WARN_ON(!mutex_is_locked(&dev_priv->dev->struct_mutex));

	/* This shouldn't happen */
	WARN_ON(i915_scheduler_is_engine_busy(req->engine));

	/* We don't expect to see nodes that are already tracked */
	if (!WARN_ON(req->scheduler_qe)) {
		/*
		 * Untracked node (e.g. context initialisation batch buffer),
		 * must not be inside scheduler submission path.
		 */
		WARN_ON((scheduler->flags[req->engine->id] & I915_SF_SUBMITTING));
		scheduler->stats[req->engine->id].non_batch++;
		req->scheduler_flags |= I915_REQ_SF_UNTRACKED;
	}
}

static uint32_t i915_scheduler_count_flying(struct i915_scheduler *scheduler,
					    struct intel_engine_cs *engine)
{
	struct i915_scheduler_queue_entry *node;
	uint32_t flying = 0;

	assert_scheduler_lock_held(scheduler);

	for_each_scheduler_node(node, engine->id)
		if (I915_SQS_IS_FLYING(node))
			flying++;

	return flying;
}

static void i915_scheduler_priority_bump_clear(struct i915_scheduler *scheduler)
{
	struct i915_scheduler_queue_entry *node;
	int i;

	assert_scheduler_lock_held(scheduler);

	/*
	 * Ensure circular dependencies don't cause problems and that a bump
	 * by object usage only bumps each using buffer once:
	 */
	for (i = 0; i < I915_NUM_ENGINES; i++) {
		for_each_scheduler_node(node, i)
			node->bumped = false;
	}
}

static int i915_scheduler_priority_bump(struct i915_scheduler *scheduler,
				struct i915_scheduler_queue_entry *target,
				uint32_t bump)
{
	uint32_t new_priority;
	int i, count;

	if (target->priority >= scheduler->priority_level_max)
		return 1;

	if (target->bumped)
		return 0;

	new_priority = target->priority + bump;
	if ((new_priority <= target->priority) ||
	    (new_priority > scheduler->priority_level_max))
		target->priority = scheduler->priority_level_max;
	else
		target->priority = new_priority;

	count = 1;
	target->bumped = true;

	for (i = 0; i < target->num_deps; i++) {
		if (!target->dep_list[i])
			continue;

		if (target->dep_list[i]->bumped)
			continue;

		count += i915_scheduler_priority_bump(scheduler,
						      target->dep_list[i],
						      bump);
	}

	return count;
}

/*
 * Nodes are considered valid dependencies if they are queued on any engine
 * or if they are in flight on a different engine. In flight on the same
 * engine is no longer interesting for non-premptive nodes as the engine
 * serialises execution. For pre-empting nodes, all in flight dependencies
 * are valid as they must not be jumped by the act of pre-empting.
 *
 * Anything that is neither queued nor flying is uninteresting.
 */
static inline bool i915_scheduler_is_dependency_valid(
			struct i915_scheduler_queue_entry *node, uint32_t idx)
{
	struct i915_scheduler_queue_entry *dep;

	dep = node->dep_list[idx];
	if (!dep)
		return false;

	if (I915_SQS_IS_QUEUED(dep))
		return true;

	if (I915_SQS_IS_FLYING(dep)) {
		if (node->params.engine != dep->params.engine)
			return true;

		if (node->params.request->scheduler_flags & I915_REQ_SF_PREEMPT)
			return true;
	}

	return false;
}

static int i915_scheduler_pop_from_queue_locked(struct intel_engine_cs *engine,
				struct i915_scheduler_queue_entry **pop_node)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *best = NULL;
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_gem_request *req;
	int ret;
	int i;
	bool any_queued = false;
	bool has_local, has_remote, only_remote = false;
	bool local_preempt_only;

	assert_scheduler_lock_held(scheduler);

	*pop_node = NULL;
	ret = -ENODATA;

	for_each_scheduler_node(node, engine->id) {
		if (!I915_SQS_IS_QUEUED(node))
			continue;
		any_queued = true;

		/*
		 * Attempt to re-enable pre-emption if a node wants to pre-empt
		 * but previously got downgraded.
		 */
		req = node->params.request;
		if (req->scheduler_flags & I915_REQ_SF_WAS_PREEMPT)
			req->scheduler_flags |= I915_REQ_SF_PREEMPT;

		has_local  = false;
		has_remote = false;
		local_preempt_only = (req->scheduler_flags & I915_REQ_SF_PREEMPT) != 0;
		for (i = 0; i < node->num_deps; i++) {
			if (!i915_scheduler_is_dependency_valid(node, i))
				continue;

			if (node->dep_list[i]->params.engine == node->params.engine) {
				has_local = true;

				if (local_preempt_only) {
					req->scheduler_flags &= ~I915_REQ_SF_PREEMPT;
					if (i915_scheduler_is_dependency_valid(node, i))
						local_preempt_only = false;
					req->scheduler_flags |= I915_REQ_SF_PREEMPT;
				}
			} else
				has_remote = true;
		}

		if (has_local && local_preempt_only) {
			/*
			 * If a preemptive node's local dependencies are all
			 * flying, then they can be ignored by un-preempting
			 * the node.
			 */
			req->scheduler_flags &= ~I915_REQ_SF_PREEMPT;
			has_local = false;
		}

		if (has_remote && !has_local)
			only_remote = true;

		if (!has_local && !has_remote) {
			if (!best ||
			    (node->priority > best->priority))
				best = node;
		}
	}

	if (best) {
		list_del(&best->link);

		INIT_LIST_HEAD(&best->link);
		best->status = I915_SQS_POPPED;

		trace_i915_scheduler_node_state_change(engine, best);

		ret = 0;
	} else {
		/* Can only get here if:
		 * (a) there are no buffers in the queue
		 * (b) all queued buffers are dependent on other buffers
		 *     e.g. on a buffer that is in flight on a different engine
		 */
		if (only_remote) {
			/* The only dependent buffers are on another engine. */
			ret = -EAGAIN;
		} else if (any_queued) {
			/* It seems that something has gone horribly wrong! */
			WARN_ONCE(true, "Broken dependency tracking on engine %d!\n",
				  (int) engine->id);
		}
	}

	trace_i915_scheduler_pop_from_queue(engine, best);

	*pop_node = best;
	return ret;
}

/*
 * NB: The driver mutex lock must be held before calling this function. It is
 * only really required during the actual back end submission call. However,
 * attempting to acquire a mutex while holding a spin lock is a Bad Idea.
 * And releasing the one before acquiring the other leads to other code
 * being run and interfering.
 *
 * Hence any caller that does not already have the mutex lock for other
 * reasons should call i915_scheduler_submit_unlocked() instead in order to
 * obtain the lock first.
 */
static int i915_scheduler_submit(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_gem_request *req;
	int ret, count = 0, flying;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	spin_lock_irq(&scheduler->lock);

	WARN_ON(scheduler->flags[engine->id] & I915_SF_SUBMITTING);
	scheduler->flags[engine->id] |= I915_SF_SUBMITTING;

	/*
	 * If pre-emption is in progress on an engine then no further work
	 * may be submitted to that same engine. Come back later...
	 */
	if (i915_scheduler_is_engine_preempting(engine)) {
		ret = -EAGAIN;
		goto error;
	}

	/* First time around, complain if anything unexpected occurs: */
	ret = i915_scheduler_pop_from_queue_locked(engine, &node);
	if (ret)
		goto error;

	do {
		WARN_ON(node->params.engine != engine);
		WARN_ON(node->status != I915_SQS_POPPED);
		count++;

		req = node->params.request;
		if (req->scheduler_flags & I915_REQ_SF_PREEMPT) {
			struct i915_scheduler_queue_entry *fly;
			bool got_flying = false;

			for_each_scheduler_node(fly, engine->id) {
				if (!I915_SQS_IS_FLYING(fly))
					continue;

				got_flying = true;
				if (fly->priority >= node->priority) {
					/*
					 * Already working on something at least
					 * as important, so don't interrupt it.
					 */
					req->scheduler_flags &= ~I915_REQ_SF_PREEMPT;
					break;
				}
			}

			if (!got_flying) {
				/* Nothing to preempt so don't bother. */
				req->scheduler_flags &= ~I915_REQ_SF_PREEMPT;
			}
		}

		/*
		 * The call to pop above will have removed the node from the
		 * list. So add it back in and mark it as in flight.
		 */
		i915_scheduler_node_fly(node);

		if (req->scheduler_flags & I915_REQ_SF_PREEMPT) {
			/*
			 * If this batch is pre-emptive then it will tie the
			 * hardware up at least until it has begun to be
			 * executed. That is, if a pre-emption request is in
			 * flight then no other work may be submitted until
			 * it resolves.
			 */
			scheduler->flags[engine->id] |= I915_SF_PREEMPTING;
			scheduler->stats[engine->id].preempts_submitted++;
		} else
			scheduler->stats[engine->id].submitted++;

		spin_unlock_irq(&scheduler->lock);
		ret = dev_priv->gt.execbuf_final(&node->params);
		spin_lock_irq(&scheduler->lock);

		/*
		 * Handle failed submission but first check that the
		 * watchdog/reset code has not nuked the node while we
		 * weren't looking:
		 */
		if (ret == 0) {
			req->scheduler_flags &= ~I915_REQ_SF_RESTART;
		} else if (node->status != I915_SQS_DEAD) {
			bool requeue = true;

			/*
			 * Oh dear! Either the node is broken or the engine is
			 * busy. So need to kill the node or requeue it and try
			 * again later as appropriate. Either way, clear the
			 * pre-emption flag as it ain't happening.
			 */
			scheduler->flags[engine->id] &= ~I915_SF_PREEMPTING;

			switch (-ret) {
			case ENODEV:
			case ENOENT:
				/* Fatal errors. Kill the node. */
				requeue = false;
				scheduler->stats[engine->id].exec_dead++;
				i915_scheduler_node_kill(scheduler, node);
				break;

			case EAGAIN:
			case EBUSY:
			case EIO:
			case ENOMEM:
			case ERESTARTSYS:
			case EINTR:
				/* Supposedly recoverable errors. */
				scheduler->stats[engine->id].exec_again++;
				break;

			default:
				/*
				 * Assume the error is recoverable and hope
				 * for the best.
				 */
				MISSING_CASE(-ret);
				scheduler->stats[engine->id].exec_again++;
				break;
			}

			if (requeue) {
				i915_scheduler_node_requeue(node);
				/*
				 * No point spinning if the engine is currently
				 * unavailable so just give up and come back
				 * later.
				 */
				break;
			}
		}

		/* If pre-emption is now in progress then stop launching */
		if (i915_scheduler_is_engine_preempting(engine))
			break;

		/* Keep launching until the sky is sufficiently full. */
		flying = i915_scheduler_count_flying(scheduler, engine);
		if (flying >= scheduler->min_flying)
			break;

		/* Grab another node and go round again... */
		ret = i915_scheduler_pop_from_queue_locked(engine, &node);
	} while (ret == 0);

	/* Don't complain about not being able to submit extra entries */
	if (ret == -ENODATA)
		ret = 0;

	/*
	 * Bump the priority of everything that was not submitted to prevent
	 * starvation of low priority tasks by a spamming high priority task.
	 */
	i915_scheduler_priority_bump_clear(scheduler);
	for_each_scheduler_node(node, engine->id) {
		if (!I915_SQS_IS_QUEUED(node))
			continue;

		i915_scheduler_priority_bump(scheduler, node,
					     scheduler->priority_level_bump);
	}

	/* On success, return the number of buffers submitted. */
	if (ret == 0)
		ret = count;

error:
	scheduler->flags[engine->id] &= ~I915_SF_SUBMITTING;
	spin_unlock_irq(&scheduler->lock);
	return ret;
}

static int i915_scheduler_submit_unlocked(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	int ret;

	ret = i915_mutex_lock_interruptible(dev);
	if (ret)
		return ret;

	ret = i915_scheduler_submit(engine);

	mutex_unlock(&dev->struct_mutex);

	return ret;
}

/**
 * i915_scheduler_file_queue_inc - Increment the file's request queue count.
 * @file: File object to process.
 */
static void i915_scheduler_file_queue_inc(struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	file_priv->scheduler_queue_length++;
}

/**
 * i915_scheduler_file_queue_dec - Decrement the file's request queue count.
 * @file: File object to process.
 */
static void i915_scheduler_file_queue_dec(struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;

	file_priv->scheduler_queue_length--;
}

static void i915_generate_dependencies(struct i915_scheduler *scheduler,
				       struct i915_scheduler_queue_entry *node,
				       uint32_t engine)
{
	struct i915_scheduler_obj_entry *this, *that;
	struct i915_scheduler_queue_entry *test;
	int i, j;
	bool found;

	for_each_scheduler_node(test, engine) {
		if (I915_SQS_IS_COMPLETE(test))
			continue;

		/*
		 * Batches on the same engine for the same
		 * context must be kept in order.
		 */
		found = (node->params.ctx == test->params.ctx) &&
			(node->params.engine == test->params.engine);

		/*
		 * Batches working on the same objects must
		 * be kept in order.
		 */
		for (i = 0; (i < node->num_objs) && !found; i++) {
			this = node->objs + i;

			for (j = 0; j < test->num_objs; j++) {
				that = test->objs + j;

				if (this->obj != that->obj)
					continue;

				/* Only need to worry about writes */
				if (this->read_only && that->read_only)
					continue;

				found = true;
				break;
			}
		}

		if (found) {
			node->dep_list[node->num_deps] = test;
			node->num_deps++;
		}
	}
}

static int i915_scheduler_queue_execbuffer_bypass(struct i915_scheduler_queue_entry *qe)
{
	struct drm_i915_private *dev_priv = qe->params.dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	int ret;

	scheduler->stats[qe->params.engine->id].queued++;

	trace_i915_scheduler_queue(qe->params.engine, qe);

	intel_ring_reserved_space_cancel(qe->params.request->ringbuf);

	scheduler->flags[qe->params.engine->id] |= I915_SF_SUBMITTING;
	ret = dev_priv->gt.execbuf_final(&qe->params);
	scheduler->stats[qe->params.engine->id].submitted++;
	scheduler->flags[qe->params.engine->id] &= ~I915_SF_SUBMITTING;

	/*
	 * Don't do any clean up on failure because the caller will
	 * do it all anyway.
	 */
	if (ret)
		return ret;

	/* Need to release any resources held by the node: */
	qe->status = I915_SQS_COMPLETE;
	i915_scheduler_clean_node(qe);

	scheduler->stats[qe->params.engine->id].expired++;

	return 0;
}

static uint32_t i915_scheduler_count_incomplete(struct i915_scheduler *scheduler)
{
	struct i915_scheduler_queue_entry *test;
	int r, incomplete = 0;

	for (r = 0; r < I915_NUM_ENGINES; r++) {
		for_each_scheduler_node(test, r) {
			if (I915_SQS_IS_COMPLETE(test))
				continue;

			incomplete++;
		}
	}

	return incomplete;
}

/**
 * i915_scheduler_queue_execbuffer - Submit a batch buffer request to the
 * scheduler.
 * @qe: The batch buffer request to be queued.
 * The expectation is the qe passed in is a local stack variable. This
 * function will copy its contents into a freshly allocated list node. The
 * new node takes ownership of said contents so the original qe should simply
 * be discarded and not cleaned up (i.e. don't free memory it points to or
 * dereference objects it holds). The node is added to the scheduler's queue
 * and the batch buffer will be submitted to the hardware at some future
 * point in time (which may be immediately, before returning or may be quite
 * a lot later).
 */
int i915_scheduler_queue_execbuffer(struct i915_scheduler_queue_entry *qe)
{
	struct drm_i915_private *dev_priv = qe->params.dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct intel_engine_cs *engine = qe->params.engine;
	struct i915_scheduler_queue_entry *node;
	bool not_flying, want_preempt;
	int i, e;
	int incomplete;

	/* Bypass the scheduler and send the buffer immediately? */
	if (!i915.enable_scheduler)
		return i915_scheduler_queue_execbuffer_bypass(qe);

	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	*node = *qe;
	INIT_LIST_HEAD(&node->link);
	node->status = I915_SQS_QUEUED;
	node->stamp  = jiffies;
	i915_gem_request_reference(node->params.request);

	intel_ring_reserved_space_cancel(node->params.request->ringbuf);

	WARN_ON(node->params.request->scheduler_qe);
	node->params.request->scheduler_qe = node;

	/*
	 * Need to determine the number of incomplete entries in the list as
	 * that will be the maximum size of the dependency list.
	 *
	 * Note that the allocation must not be made with the spinlock acquired
	 * as kmalloc can sleep. However, the unlock/relock is safe because no
	 * new entries can be queued up during the unlock as the i915 driver
	 * mutex is still held. Entries could be removed from the list but that
	 * just means the dep_list will be over-allocated which is fine.
	 */
	spin_lock_irq(&scheduler->lock);
	incomplete = i915_scheduler_count_incomplete(scheduler);

	/* Temporarily unlock to allocate memory: */
	spin_unlock_irq(&scheduler->lock);
	if (incomplete) {
		node->dep_list = kmalloc_array(incomplete,
					       sizeof(*node->dep_list),
					       GFP_KERNEL);
		if (!node->dep_list) {
			kfree(node);
			return -ENOMEM;
		}
	} else
		node->dep_list = NULL;

	spin_lock_irq(&scheduler->lock);
	node->num_deps = 0;

	if (node->dep_list) {
		for (e = 0; e < I915_NUM_ENGINES; e++)
			i915_generate_dependencies(scheduler, node, e);

		WARN_ON(node->num_deps > incomplete);
	}

	node->priority = clamp(node->priority,
			       scheduler->priority_level_min,
			       scheduler->priority_level_max);

	if ((node->priority > 0) && node->num_deps) {
		i915_scheduler_priority_bump_clear(scheduler);

		for (i = 0; i < node->num_deps; i++)
			i915_scheduler_priority_bump(scheduler,
					node->dep_list[i], node->priority);
	}

	list_add_tail(&node->link, &scheduler->node_queue[engine->id]);

	i915_scheduler_file_queue_inc(node->params.file);

	not_flying = i915_scheduler_count_flying(scheduler, engine) <
						 scheduler->min_flying;

	want_preempt = node->priority >= scheduler->priority_level_preempt;

	if (!i915.enable_preemption)
		want_preempt = false;

	/* Pre-emption is not yet implemented in non-execlist mode */
	if (!i915.enable_execlists)
		want_preempt = false;

	/* Pre-emption is not yet implemented in non-GUC mode */
	if (!i915.enable_guc_submission)
		want_preempt = false;

	if (want_preempt) {
		node->params.request->scheduler_flags |=
			I915_REQ_SF_WAS_PREEMPT | I915_REQ_SF_PREEMPT;
		scheduler->stats[engine->id].preempts_queued++;
	}

	scheduler->stats[engine->id].queued++;

	trace_i915_scheduler_queue(engine, node);
	trace_i915_scheduler_node_state_change(engine, node);

	spin_unlock_irq(&scheduler->lock);

	if (not_flying || want_preempt)
		i915_scheduler_submit(engine);

	return 0;
}

/**
 * i915_scheduler_notify_request - Notify the scheduler that the given
 * request has completed on the hardware.
 * @req: Request structure which has completed
 * @preempt: Did it complete pre-emptively?
 * A sequence number has popped out of the hardware and the request handling
 * code has mapped it back to a request and will mark that request complete.
 * It also calls this function to notify the scheduler about the completion
 * so the scheduler's node can be updated appropriately.
 * Returns true if the request is scheduler managed, false if not. The return
 * value is combined for all freshly completed requests and if any were true
 * then i915_scheduler_wakeup() is called so the scheduler can do further
 * processing (submit more work) at the end.
 */
bool i915_scheduler_notify_request(struct drm_i915_gem_request *req,
				   bool preempt)
{
	struct drm_i915_private *dev_priv = req->i915;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node = req->scheduler_qe;
	uint32_t engine_id = req->engine->id;
	unsigned long flags;
	bool result;

	trace_i915_scheduler_landing(req, preempt);

	spin_lock_irqsave(&scheduler->lock, flags);

	if (!node) {
		/* Untracked request, presumably engine init */
		WARN_ON(preempt);
		WARN_ON(!(req->scheduler_flags & I915_REQ_SF_UNTRACKED));
		scheduler->stats[engine_id].non_batch_done++;
		result = false;
	} else if (WARN(!I915_SQS_IS_FLYING(node), "Node not flying: %d:%d -> %s! [preempt = %d]\n",
			req->uniq, req->seqno,
			i915_scheduler_queue_status_str(node->status),
			preempt)) {
		/* This shouldn't happen */
		result = false;
	} else if (req->cancelled) {
		/* If a preemption was in progress, it won't complete now. */
		/* Need to clear I915_PREEMPTIVE_ACTIVE_SEQNO??? */
		if (node->status == I915_SQS_OVERTAKING)
			scheduler->flags[engine_id] &=
				    ~(I915_SF_PREEMPTING | I915_SF_PREEMPTED);

		node->status = I915_SQS_DEAD;
		scheduler->stats[engine_id].kill_flying++;
		result = true;
	} else if (node->status == I915_SQS_FLYING) {
		WARN(preempt, "Got flying node with preemption!\n");

		/* Node was in flight so mark it as complete. */
		node->status = I915_SQS_COMPLETE;
		scheduler->stats[engine_id].completed++;
		result = true;
	} else if (node->status == I915_SQS_OVERTAKING) {
		WARN(!preempt, "Got overtaking node without preemption!\n");

		/* Preempting request has completed & becomes preempted */
		node->status = I915_SQS_PREEMPTED;
		trace_i915_scheduler_unfly(node->params.engine, node);

		/* Scheduler is now in post-preemption state */
		scheduler->flags[engine_id] |= I915_SF_PREEMPTED;
		scheduler->stats[engine_id].preempts_completed++;
		result = true;
	} else {
		WARN(true, "Unknown node state: %s [%s]!\n",
		     i915_scheduler_queue_status_str(node->status),
		     preempt ? "preempting" : "regular");
		result = false;
	}

	if (result)
		trace_i915_scheduler_node_state_change(req->engine, node);

	spin_unlock_irqrestore(&scheduler->lock, flags);

	return result;
}

static int i915_scheduler_remove_dependent(struct i915_scheduler *scheduler,
				struct i915_scheduler_queue_entry *remove)
{
	struct i915_scheduler_queue_entry *node;
	int i, r;
	int count = 0;

	/*
	 * Ensure that a node is not being removed which is still dependent
	 * upon other (not completed) work. If that happens, it implies
	 * something has gone very wrong with the dependency tracking! Note
	 * that there is no need to worry if this node has been explicitly
	 * killed for some reason - it might be being killed before it got
	 * sent to the hardware.
	 */
	if (remove->status != I915_SQS_DEAD) {
		for (i = 0; i < remove->num_deps; i++)
			if ((remove->dep_list[i]) &&
			    (!I915_SQS_IS_COMPLETE(remove->dep_list[i])))
				count++;
		WARN_ON(count);
	}

	/*
	 * Remove this node from the dependency lists of any other node which
	 * might be waiting on it.
	 */
	for (r = 0; r < I915_NUM_ENGINES; r++) {
		for_each_scheduler_node(node, r) {
			for (i = 0; i < node->num_deps; i++) {
				if (node->dep_list[i] != remove)
					continue;

				node->dep_list[i] = NULL;
			}
		}
	}

	return 0;
}

/**
 * i915_scheduler_wakeup - wake the scheduler's worker thread
 * @dev: DRM device
 * Called at the end of seqno interrupt processing if any request has
 * completed that corresponds to a scheduler node.
 */
void i915_scheduler_wakeup(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	queue_work(dev_priv->wq, &dev_priv->mm.scheduler_work);
}

/**
 * i915_scheduler_clean_node - free up any allocations/references
 * associated with the given scheduler queue entry.
 * @node: Queue entry structure which is complete
 * After a give batch buffer completes on the hardware, all the information
 * required to resubmit it is no longer required. However, the node entry
 * itself might still be required for tracking purposes for a while longer.
 * This function should be called as soon as the node is known to be complete
 * so that these resources may be freed even though the node itself might
 * hang around.
 */
void i915_scheduler_clean_node(struct i915_scheduler_queue_entry *node)
{
	int i;

	if (!I915_SQS_IS_COMPLETE(node)) {
		WARN(!node->params.request->cancelled,
		     "Cleaning active node: %d!\n", node->status);
		return;
	}

	if (node->params.batch_obj) {
		/*
		 * The batch buffer must be unpinned before it is unreferenced
		 * otherwise the unpin fails with a missing vma!?
		 */
		if (node->params.dispatch_flags & I915_DISPATCH_SECURE)
			i915_gem_execbuff_release_batch_obj(node->params.batch_obj);

		node->params.batch_obj = NULL;
	}

	/* Release the locked buffers: */
	for (i = 0; i < node->num_objs; i++)
		drm_gem_object_unreference(&node->objs[i].obj->base);
	kfree(node->objs);
	node->objs = NULL;
	node->num_objs = 0;

	/* Context too: */
	if (node->params.ctx) {
		i915_gem_context_unreference(node->params.ctx);
		node->params.ctx = NULL;
	}

	/* And anything else owned by the node: */
	if (node->params.cliprects) {
		kfree(node->params.cliprects);
		node->params.cliprects = NULL;
	}
}

void i915_scheduler_reset_cleanup(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;

	if (scheduler->flags[engine->id] & I915_SF_INTERRUPTS_ENABLED) {
		engine->irq_put(engine);
		scheduler->flags[engine->id] &= ~I915_SF_INTERRUPTS_ENABLED;
	}
}

/*
 * At this point, preemption has occurred.
 *
 * All the requests that had already completed before preemption will
 * have been taken off the fence_signal_list, signalled, and put onto
 * the fence_unsignal_list for cleanup. The preempting request itself
 * should however still be on the fence_signal_list (and has not been
 * signalled). There may also be additional requests on this list; they
 * have been preempted.
 *
 * The request_list has not yet been processed, so it may still contain
 * requests that have already completed. It should also contain the
 * preempting request (not completed), and maybe additional requests;
 * again, these have been preempted and need to be recycled through the
 * scheduler.
 */
noinline
static void
i915_scheduler_preemption_postprocess(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *pnode = NULL;
	struct drm_i915_gem_request *preq = NULL;
	struct drm_i915_gem_request *midp = NULL;
	struct i915_scheduler_stats *stats;
	unsigned long flags;
	int preempted = 0, preemptive = 0;

	mutex_lock(&engine->dev->struct_mutex);

	/*
	 * FIXME: grab & empty fence_signal_list with spinlock,
	 * then iterate after?
	 */
	spin_lock_irqsave(&engine->fence_lock, flags);
	while (!list_empty(&engine->fence_signal_list)) {
		struct i915_scheduler_queue_entry *node;
		struct drm_i915_gem_request *req;

		req = list_first_entry(&engine->fence_signal_list,
				       struct drm_i915_gem_request,
				       signal_link);
		list_del_init(&req->signal_link);
		spin_unlock_irqrestore(&engine->fence_lock, flags);

		/* We should find only tracked unsignalled requests */
		node = req->scheduler_qe;
		WARN(!node || i915_gem_request_completed(req) ||
		     (node->status == I915_SQS_PREEMPTED),
		     "Invalid node state: %s [req = %d:%d]\n",
		     node ? i915_scheduler_queue_status_str(node->status) : "NULL",
		     req->uniq, req->seqno);

		i915_gem_request_unreference(req);

		spin_lock_irqsave(&engine->fence_lock, flags);
	}
	spin_unlock_irqrestore(&engine->fence_lock, flags);
	/* Fence signal list must now be empty */

	/*
	 * The preemptive request and all other requests remaining on the
	 * engine's work-in-progress list must be marked as preempted, so
	 * the scheduler will reselect and resubmit them ...
	 */
	spin_lock_irqsave(&scheduler->lock, flags);

	{
		struct drm_i915_gem_request *req, *next;

		list_for_each_entry_safe(req, next, &engine->request_list, list) {
			struct i915_scheduler_queue_entry *node;

			node = req->scheduler_qe;
			if (WARN_ON(req->engine != engine))
				continue;
			if (i915_gem_request_completed(req))
				continue;
			/* Let's hope there aren't any untracked nodes here! */
			if (WARN_ON(!node))
				continue;

			if (node->status == I915_SQS_PREEMPTED) {
				/* Already processed in _notify() above */
				preemptive += 1;
				preq = req;
				pnode = req->scheduler_qe;
			} else if (!WARN_ON(!I915_SQS_IS_FLYING(node))) {
				preempted += 1;
				node->status = I915_SQS_PREEMPTED;
				trace_i915_scheduler_unfly(engine, node);
				trace_i915_scheduler_node_state_change(engine, node);

				/* Identify a mid-batch preemption */
				if (req->seqno == engine->last_batch_start) {
					WARN(midp, "Multiple mid-batch-preempted requests?\n");
					midp = req;
				}
			}

			i915_gem_request_dequeue(req);
		}
	}

	/* We should have found exactly ONE preemptive request */
	WARN(preemptive != 1, "Got unexpected preemptive count II: %d!\n",
	     preemptive);
	stats = &scheduler->stats[engine->id];
	stats->preempted += preempted;
	if (stats->max_preempted < preempted)
		stats->max_preempted = preempted;

	/* Now fix up the contexts of all preempt{ive,ed} requests */
	{
		struct intel_context *mid_ctx = NULL;
		struct i915_scheduler_queue_entry *node;
		u32 started = engine->last_batch_start;

		/*
		 * Iff preemption was mid-batch, we should have found a
		 * mid-batch-preempted request
		 */
		if (started && started != engine->last_irq_seqno)
			WARN(!midp, "Mid-batch preempted, but request not found\n");
		else
			WARN(midp, "Found unexpected mid-batch preemption?\n");

		if (midp) {
			/* Rewrite this context rather than emptying it */
			intel_lr_context_resync_req(midp);
			midp->scheduler_flags |= I915_REQ_SF_RESTART;
			mid_ctx = midp->ctx;
			stats->mid_preempted += 1;
			WARN_ON(preq == midp);
		}

		for_each_scheduler_node(node, engine->id) {
			/* XXX: Sky should be empty now */
			if (WARN_ON(I915_SQS_IS_FLYING(node)))
				continue;

			/* Clean up preempted contexts */
			if (node->status != I915_SQS_PREEMPTED)
				continue;

			if (node->params.ctx != mid_ctx) {
				/* Empty the preempted ringbuffer */
				intel_lr_context_resync(node->params.ctx, engine,
							false);
				/* Request is now queued, not preempted */
				node->status = I915_SQS_QUEUED;
				trace_i915_scheduler_node_state_change(engine,
								       node);
			}
		}
	}

	/* Anything else to do here ... ? */

	/*
	 * Postprocessing complete; the scheduler is now back in
	 * normal non-preemptive state and can submit more requests
	 */
	scheduler->flags[engine->id] &= ~(I915_SF_PREEMPTING|I915_SF_PREEMPTED);

	spin_unlock_irqrestore(&scheduler->lock, flags);

	/* XXX: Should be nothing outstanding on request list */
	{
		struct drm_i915_gem_request *req;

		list_for_each_entry(req, &engine->request_list, list)
			WARN_ON(!i915_gem_request_completed(req));
	}

	/* Anything else to do here ... ? */
	if (!WARN_ON(pnode == NULL || preq == NULL)) {
		WARN_ON(pnode->params.request != preq);
		WARN_ON(preq->scheduler_qe != pnode);
		WARN_ON(preq->seqno);

		/*
		 * FIXME: assign a new reserved seqno here to ensure
		 * we don't relaunch this request with the same seqno
		 * FIXME: can we just clear it here instead?
		 */
		if (dev_priv->next_seqno == 0)
			dev_priv->next_seqno = 1;
		dev_priv->last_seqno = dev_priv->next_seqno++;
		DRM_DEBUG_DRIVER("reassigning reserved seqno %08x->%08x, (seqno %08x, uniq %d)\n",
			preq->reserved_seqno, dev_priv->last_seqno,
			preq->seqno, preq->uniq);
		preq->reserved_seqno = dev_priv->last_seqno;

		/* FIXME: don't sleep, don't empty context? */
		msleep(1);
		/* Empty the preempted ringbuffer */
		intel_lr_context_resync(preq->ctx, engine, false);
	}

	mutex_unlock(&engine->dev->struct_mutex);
}

noinline
static bool i915_scheduler_remove(struct i915_scheduler *scheduler,
				  struct intel_engine_cs *engine,
				  struct list_head *remove)
{
	struct i915_scheduler_queue_entry *node, *node_next;
	int flying = 0, queued = 0;
	bool do_submit;
	uint32_t min_seqno;

	spin_lock_irq(&scheduler->lock);

	/*
	 * In the case where the system is idle, starting 'min_seqno' from a big
	 * number will cause all nodes to be removed as they are now back to
	 * being in-order. However, this will be a problem if the last one to
	 * complete was actually out-of-order as the engine seqno value will be
	 * lower than one or more completed buffers. Thus code looking for the
	 * completion of said buffers will wait forever.
	 * Instead, use the hardware seqno as the starting point. This means
	 * that some buffers might be kept around even in a completely idle
	 * system but it should guarantee that no-one ever gets confused when
	 * waiting for buffer completion.
	 */
	min_seqno = engine->get_seqno(engine, true);

	for_each_scheduler_node(node, engine->id) {
		if (I915_SQS_IS_QUEUED(node))
			queued++;
		else if (I915_SQS_IS_FLYING(node))
			flying++;
		else if (I915_SQS_IS_COMPLETE(node))
			continue;

		if (node->params.request->seqno == 0)
			continue;

		if (!i915_seqno_passed(node->params.request->seqno, min_seqno))
			min_seqno = node->params.request->seqno;
	}

	INIT_LIST_HEAD(remove);
	list_for_each_entry_safe(node, node_next, &scheduler->node_queue[engine->id], link) {
		/*
		 * Only remove completed nodes which have a lower seqno than
		 * all pending nodes. While there is the possibility of the
		 * engine's seqno counting backwards, all higher buffers must
		 * be remembered so that the 'i915_seqno_passed()' test can
		 * report that they have in fact passed.
		 *
		 * NB: This is not true for 'dead' nodes. The GPU reset causes
		 * the software seqno to restart from its initial value. Thus
		 * the dead nodes must be removed even though their seqno values
		 * are potentially vastly greater than the current engine seqno.
		 */
		if (!I915_SQS_IS_COMPLETE(node))
			continue;

		if (node->status != I915_SQS_DEAD) {
			if (i915_seqno_passed(node->params.request->seqno, min_seqno) &&
			    (node->params.request->seqno != min_seqno))
				continue;
		}

		list_del(&node->link);
		list_add(&node->link, remove);
		scheduler->stats[engine->id].expired++;

		/* Strip the dependency info while the mutex is still locked */
		i915_scheduler_remove_dependent(scheduler, node);

		/* Likewise clean up the file pointer. */
		if (node->params.file) {
			i915_scheduler_file_queue_dec(node->params.file);
			node->params.file = NULL;
		}

		continue;
	}

	/*
	 * Release the interrupt reference count if there are no longer any
	 * nodes to worry about.
	 */
	if (!flying && !queued &&
	    (scheduler->flags[engine->id] & I915_SF_INTERRUPTS_ENABLED)) {
		engine->irq_put(engine);
		scheduler->flags[engine->id] &= ~I915_SF_INTERRUPTS_ENABLED;
	}

	/* Launch more packets now? */
	do_submit = (queued > 0) && (flying < scheduler->min_flying);

	trace_i915_scheduler_remove(engine, min_seqno, do_submit);

	spin_unlock_irq(&scheduler->lock);

	return do_submit;
}

static void i915_scheduler_process_work(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;
	bool do_submit;
	struct list_head remove;

	if (list_empty(&scheduler->node_queue[engine->id]))
		return;

	/* Remove completed nodes. */
	do_submit = i915_scheduler_remove(scheduler, engine, &remove);

	if (!do_submit && list_empty(&remove))
		return;

	/* Need to grab the pm lock outside of the mutex lock */
	if (do_submit)
		intel_runtime_pm_get(dev_priv);

	mutex_lock(&engine->dev->struct_mutex);

	if (do_submit)
		i915_scheduler_submit(engine);

	while (!list_empty(&remove)) {
		node = list_first_entry(&remove, typeof(*node), link);
		list_del(&node->link);

		trace_i915_scheduler_destroy(engine, node);

		/* Free up all the DRM references */
		i915_scheduler_clean_node(node);

		/* And anything else owned by the node: */
		node->params.request->scheduler_qe = NULL;
		i915_gem_request_unreference(node->params.request);
		kfree(node->dep_list);
		kfree(node);
	}

	mutex_unlock(&engine->dev->struct_mutex);

	if (do_submit)
		intel_runtime_pm_put(dev_priv);
}

/**
 * i915_scheduler_work_handler - scheduler's work handler callback.
 * @work: Work structure
 * A lot of the scheduler's work must be done asynchronously in response to
 * an interrupt or other event. However, that work cannot be done at
 * interrupt time or in the context of the event signaller (which might in
 * fact be an interrupt). Thus a worker thread is required. This function
 * will cause the thread to wake up and do its processing.
 */
void i915_scheduler_work_handler(struct work_struct *work)
{
	struct intel_engine_cs *engine;
	struct drm_i915_private *dev_priv;
	struct i915_scheduler *scheduler;

	dev_priv = container_of(work, struct drm_i915_private, mm.scheduler_work);
	scheduler = dev_priv->scheduler;

	for_each_engine(engine, dev_priv) {
		if (scheduler->flags[engine->id] & I915_SF_PREEMPTED)
			i915_scheduler_preemption_postprocess(engine);
		i915_scheduler_process_work(engine);
	}
}

/**
 * i915_scheduler_file_queue_wait - Waits for space in the per file queue.
 * @file: File object to process.
 * This allows throttling of applications by limiting the total number of
 * outstanding requests to a specified level. Once that limit is reached,
 * this call will stall waiting on the oldest outstanding request. If it can
 * not stall for any reason it returns true to mean that the queue is full
 * and no more requests should be accepted.
 */
bool i915_scheduler_file_queue_wait(struct drm_file *file)
{
	struct drm_i915_file_private *file_priv = file->driver_priv;
	struct drm_i915_private *dev_priv  = file_priv->dev_priv;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct drm_i915_gem_request *req = NULL;
	struct i915_scheduler_queue_entry *node;
	unsigned reset_counter;
	int ret;
	struct intel_engine_cs *engine;

	if (file_priv->scheduler_queue_length < scheduler->file_queue_max)
		return false;

	do {
		spin_lock_irq(&scheduler->lock);

		/*
		 * Find the first (i.e. oldest) request for this file. In the
		 * case where an app is using multiple engines, this search
		 * might be skewed by engine. However, worst case is an app has
		 * queued ~60 requests to a high indexed engine and then one
		 * request to a low indexed engine. In such a case, the driver
		 * will wait for longer than necessary but operation will
		 * still be correct and that case is not rare enough to add
		 * jiffy based inter-engine checks.
		 */
		req = NULL;
		for_each_engine(engine, dev_priv) {
			for_each_scheduler_node(node, engine->id) {
				if (I915_SQS_IS_COMPLETE(node))
					continue;

				if (node->params.file != file)
					continue;

				req = node->params.request;
				break;
			}

			if (req)
				break;
		}

		if (!req) {
			spin_unlock_irq(&scheduler->lock);
			return false;
		}

		i915_gem_request_reference(req);

		spin_unlock_irq(&scheduler->lock);

		ret = i915_gem_check_wedge(&dev_priv->gpu_error, false);
		if (ret)
			goto err_unref;

		reset_counter = atomic_read(&dev_priv->gpu_error.reset_counter);

		ret = __i915_wait_request(req, reset_counter,
				   I915_WAIT_REQUEST_INTERRUPTIBLE, NULL, NULL);
		if (ret)
			goto err_unref;

		/* Make sure the request's resources actually get cleared up */
		i915_scheduler_process_work(req->engine);

		i915_gem_request_unreference(req);
	} while(file_priv->scheduler_queue_length >= scheduler->file_queue_max);

	return false;

err_unref:
	i915_gem_request_unreference(req);
	return true;
}

static int i915_scheduler_dump_locked(struct intel_engine_cs *engine,
				      const char *msg)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;
	int flying = 0, queued = 0, complete = 0, other = 0;
	static int old_flying = -1, old_queued = -1, old_complete = -1;
	bool b_dump;
	char brkt[2] = { '<', '>' };

	if (!engine)
		return -EINVAL;

	for_each_scheduler_node(node, engine->id) {
		if (I915_SQS_IS_QUEUED(node))
			queued++;
		else if (I915_SQS_IS_FLYING(node))
			flying++;
		else if (I915_SQS_IS_COMPLETE(node))
			complete++;
		else
			other++;
	}

	b_dump = (flying != old_flying) ||
		 (queued != old_queued) ||
		 (complete != old_complete);
	if (scheduler->flags[engine->id] & I915_SF_DUMP_FORCE) {
		if (!b_dump) {
			b_dump = true;
			brkt[0] = '{';
			brkt[1] = '}';
		}

		scheduler->flags[engine->id] &= ~I915_SF_DUMP_FORCE;
	}

	if (b_dump) {
		old_flying   = flying;
		old_queued   = queued;
		old_complete = complete;
		DRM_DEBUG_DRIVER("<%s> Q:%02d, F:%02d, C:%02d, O:%02d, "
				 "Flags = %s, Next = %d:%d %c%s%c\n",
				 engine->name, queued, flying, complete, other,
				 i915_scheduler_flag_str(scheduler->flags[engine->id]),
				 dev_priv->request_uniq, dev_priv->next_seqno,
				 brkt[0], msg, brkt[1]);
	} else {
		/*DRM_DEBUG_DRIVER("<%s> Q:%02d, F:%02d, C:%02d, O:%02d"
				 ", Flags = %s, Next = %d:%d [%s]\n",
				 engine->name,
				 queued, flying, complete, other,
				 i915_scheduler_flag_str(scheduler->flags[engine->id]),
				 dev_priv->request_uniq, dev_priv->next_seqno, msg); */

		return 0;
	}

	if (scheduler->flags[engine->id] & I915_SF_DUMP_SEQNO) {
		uint32_t seqno, b_active, b_done, p_active, p_done;

		seqno    = engine->get_seqno(engine, true);
		p_done   = intel_read_status_page(engine, I915_PREEMPTIVE_DONE_SEQNO);
		p_active = intel_read_status_page(engine, I915_PREEMPTIVE_ACTIVE_SEQNO);
		b_done   = intel_read_status_page(engine, I915_BATCH_DONE_SEQNO);
		b_active = intel_read_status_page(engine, I915_BATCH_ACTIVE_SEQNO);

		DRM_DEBUG_DRIVER("<%s> Seqno = %08x, BD = %08x, BA = %08x, PD = %08x, PA = %08x\n",
				 engine->name, seqno, b_done, b_active,
				 p_done, p_active);
	}

	if (scheduler->flags[engine->id] & I915_SF_DUMP_DETAILS) {
		int i, deps;
		uint32_t count, counts[I915_SQS_MAX];

		memset(counts, 0x00, sizeof(counts));

		for_each_scheduler_node(node, engine->id) {
			if (node->status < I915_SQS_MAX) {
				count = counts[node->status]++;
			} else {
				DRM_DEBUG_DRIVER("<%s>   Unknown status: %d!\n",
						 engine->name, node->status);
				count = -1;
			}

			deps = 0;
			for (i = 0; i < node->num_deps; i++)
				if (i915_scheduler_is_dependency_valid(node, i))
					deps++;

			DRM_DEBUG_DRIVER("<%s>   %c:%02d> uniq = %d, seqno"
					 " = %d/%s, deps = %d / %d, %s [pri = "
					 "%4d]\n", engine->name,
					 i915_scheduler_queue_status_chr(node->status),
					 count,
					 node->params.request->uniq,
					 node->params.request->seqno,
					 node->params.engine->name,
					 deps, node->num_deps,
					 i915_qe_state_str(node),
					 node->priority);

			if ((scheduler->flags[engine->id] & I915_SF_DUMP_DEPENDENCIES)
				== 0)
				continue;

			for (i = 0; i < node->num_deps; i++)
				if (node->dep_list[i])
					DRM_DEBUG_DRIVER("<%s>       |-%c:"
						"%02d%c uniq = %d, seqno = %d/%s, %s [pri = %4d]\n",
						engine->name,
						i915_scheduler_queue_status_chr(node->dep_list[i]->status),
						i,
						i915_scheduler_is_dependency_valid(node, i)
							? '>' : '#',
						node->dep_list[i]->params.request->uniq,
						node->dep_list[i]->params.request->seqno,
						node->dep_list[i]->params.engine->name,
						i915_qe_state_str(node->dep_list[i]),
						node->dep_list[i]->priority);
		}
	}

	return 0;
}

/**
 * i915_scheduler_dump - dump the scheduler's internal state to the debug log.
 * @engine: Engine to dump info for
 * @msg: A reason why it is being dumped
 * For debugging purposes, it can be very useful to see the internal state of
 * the scheduler for a given engine.
 */
int i915_scheduler_dump(struct intel_engine_cs *engine, const char *msg)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&scheduler->lock, flags);
	ret = i915_scheduler_dump_locked(engine, msg);
	spin_unlock_irqrestore(&scheduler->lock, flags);

	return ret;
}

static int i915_scheduler_dump_all_locked(struct drm_device *dev,
					  const char *msg)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct intel_engine_cs *engine;
	int r, ret = 0;

	for_each_engine(engine, dev_priv) {
		scheduler->flags[engine->id] |= scheduler->dump_flags & I915_SF_DUMP_MASK;
		r = i915_scheduler_dump_locked(engine, msg);
		if (ret == 0)
			ret = r;
	}

	return ret;
}

/**
 * i915_scheduler_dump_all - dump the scheduler's internal state to the debug
 * log.
 * @dev: DRM device
 * @msg: A reason why it is being dumped
 * For debugging purposes, it can be very useful to see the internal state of
 * the scheduler.
 */
int i915_scheduler_dump_all(struct drm_device *dev, const char *msg)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&scheduler->lock, flags);
	ret = i915_scheduler_dump_all_locked(dev, msg);
	spin_unlock_irqrestore(&scheduler->lock, flags);

	return ret;
}

/**
 * i915_scheduler_query_stats - return various scheduler statistics
 * @engine: Engine to report on
 * @stats: Stats structure to be filled in
 * For various reasons (debugging, performance analysis, curiosity) it is
 * useful to see statistics about what the scheduler is doing. This function
 * returns the stats that have been gathered in a data structure. The
 * expectation is that this will be returned to the user via debugfs.
 */
int i915_scheduler_query_stats(struct intel_engine_cs *engine,
			       struct i915_scheduler_stats_nodes *stats)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;

	memset(stats, 0x00, sizeof(*stats));

	spin_lock_irq(&scheduler->lock);

	for_each_scheduler_node(node, engine->id) {
		if (node->status >= I915_SQS_MAX) {
			DRM_DEBUG_DRIVER("Invalid node state: %d! [uniq = %d, seqno = %d]\n",
					 node->status, node->params.request->uniq,
					 node->params.request->seqno);

			stats->counts[I915_SQS_MAX]++;
			continue;
		}

		stats->counts[node->status]++;
	}

	spin_unlock_irq(&scheduler->lock);

	return 0;
}

static int i915_scheduler_submit_max_priority(struct intel_engine_cs *engine,
					      bool is_locked)
{
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	int ret, count = 0;
	bool found;

	do {
		found = false;
		spin_lock_irq(&scheduler->lock);
		for_each_scheduler_node(node, engine->id) {
			if (!I915_SQS_IS_QUEUED(node))
				continue;

			if (node->priority < scheduler->priority_level_max)
				continue;

			found = true;
			break;
		}
		spin_unlock_irq(&scheduler->lock);

		if (!found)
			break;

		if (is_locked)
			ret = i915_scheduler_submit(engine);
		else
			ret = i915_scheduler_submit_unlocked(engine);
		if (ret < 0)
			return ret;

		count += ret;
	} while (found);

	return count;
}

/**
 * i915_scheduler_flush_stamp - force requests of a given age through the
 * scheduler.
 * @engine: Engine to be flushed
 * @target: Jiffy based time stamp to flush up to
 * @is_locked: Is the driver mutex lock held?
 * DRM has a throttle by age of request facility. This requires waiting for
 * outstanding work over a given age. This function helps that by forcing
 * queued batch buffers over said age through the system.
 * Returns zero on success or -EAGAIN if the scheduler is busy (e.g. waiting
 * for a pre-emption event to complete) but the mutex lock is held which
 * would prevent the scheduler's asynchronous processing from completing.
 */
int i915_scheduler_flush_stamp(struct intel_engine_cs *engine,
			       unsigned long target,
			       bool is_locked)
{
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_private *dev_priv;
	struct i915_scheduler *scheduler;
	int flush_count = 0;

	if (!engine)
		return -EINVAL;

	dev_priv  = engine->dev->dev_private;
	scheduler = dev_priv->scheduler;

	if (!scheduler)
		return 0;

	if (is_locked && (scheduler->flags[engine->id] & I915_SF_SUBMITTING)) {
		/*
		 * Scheduler is busy already submitting another batch,
		 * come back later rather than going recursive...
		 */
		return -EAGAIN;
	}

	spin_lock_irq(&scheduler->lock);
	scheduler->stats[engine->id].flush_stamp++;
	i915_scheduler_priority_bump_clear(scheduler);
	for_each_scheduler_node(node, engine->id) {
		if (!I915_SQS_IS_QUEUED(node))
			continue;

		if (node->stamp > target)
			continue;

		flush_count = i915_scheduler_priority_bump(scheduler,
					node, scheduler->priority_level_max);
		scheduler->stats[engine->id].flush_bump += flush_count;
	}
	spin_unlock_irq(&scheduler->lock);

	if (flush_count) {
		DRM_DEBUG_DRIVER("<%s> Bumped %d entries\n", engine->name, flush_count);
		flush_count = i915_scheduler_submit_max_priority(engine, is_locked);
		if (flush_count > 0)
			scheduler->stats[engine->id].flush_submit += flush_count;
	}

	return flush_count;
}

/**
 * i915_scheduler_flush - force all requests through the scheduler.
 * @engine: Engine to be flushed
 * @is_locked: Is the driver mutex lock held?
 * For various reasons it is sometimes necessary to the scheduler out, e.g.
 * due to engine reset.
 * Returns zero on success or -EAGAIN if the scheduler is busy (e.g. waiting
 * for a pre-emption event to complete) but the mutex lock is held which
 * would prevent the scheduler's asynchronous processing from completing.
 */
int i915_scheduler_flush(struct intel_engine_cs *engine, bool is_locked)
{
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_private *dev_priv;
	struct i915_scheduler *scheduler;
	bool found;
	int ret;
	uint32_t count = 0;

	if (!engine)
		return -EINVAL;

	dev_priv  = engine->dev->dev_private;
	scheduler = dev_priv->scheduler;

	if (!scheduler)
		return 0;

	WARN_ON(is_locked && (scheduler->flags[engine->id] & I915_SF_SUBMITTING));

	scheduler->stats[engine->id].flush_all++;

	do {
		found = false;
		spin_lock_irq(&scheduler->lock);
		for_each_scheduler_node(node, engine->id) {
			if (!I915_SQS_IS_QUEUED(node))
				continue;

			found = true;
			break;
		}
		spin_unlock_irq(&scheduler->lock);

		if (found) {
			if (is_locked)
				ret = i915_scheduler_submit(engine);
			else
				ret = i915_scheduler_submit_unlocked(engine);
			scheduler->stats[engine->id].flush_submit++;
			if (ret < 0)
				return ret;

			count += ret;
		}
	} while (found);

	return count;
}

/**
 * i915_scheduler_is_mutex_required - query if it is safe to hold the mutex
 * lock while waiting for the given request.
 * @req: request to be queried
 *
 * Looks up the given request in the scheduler's internal queue and reports
 * on whether the scheduler will need to acquire the driver's mutex lock in
 * order for the that request to complete.
 */
bool i915_scheduler_is_mutex_required(struct drm_i915_gem_request *req)
{
	struct drm_i915_private *dev_priv = req->engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;

	if (!scheduler)
		return false;

	if (req->scheduler_qe == NULL)
		return false;

	if (I915_SQS_IS_QUEUED(req->scheduler_qe))
		return true;

	return false;
}

/**
 * i915_scheduler_is_request_batch_buffer - is this request related to a
 * batch buffer object?
 * @req: request to be queried
 *
 * Returns true if the given request is for a batch buffer. False means it
 * is for something else - page flip, context initialisation, etc.
 */
bool i915_scheduler_is_request_batch_buffer(struct drm_i915_gem_request *req)
{
	if (req->scheduler_qe == NULL)
		return false;

	if (req->scheduler_qe->params.batch_obj == NULL)
		return false;

	return true;
}

/**
 * i915_scheduler_closefile - notify the scheduler that a DRM file handle
 * has been closed.
 * @dev: DRM device
 * @file: file being closed
 *
 * Goes through the scheduler's queues and removes all connections to the
 * disappearing file handle that still exist. There is an argument to say
 * that this should also flush such outstanding work through the hardware.
 * However, with pre-emption, TDR and other such complications doing so
 * becomes a locking nightmare. So instead, just warn with a debug message
 * if the application is leaking uncompleted work and make sure a null
 * pointer dereference will not follow.
 */
void i915_scheduler_closefile(struct drm_device *dev, struct drm_file *file)
{
	struct i915_scheduler_queue_entry *node;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct intel_engine_cs *engine;

	if (!scheduler)
		return;

	spin_lock_irq(&scheduler->lock);

	for_each_engine(engine, dev_priv) {
		for_each_scheduler_node(node, engine->id) {
			if (node->params.file != file)
				continue;

			if (!I915_SQS_IS_COMPLETE(node))
				DRM_DEBUG_DRIVER("Closing file handle with outstanding work: %d:%d/%s on %s\n",
						 node->params.request->uniq,
						 node->params.request->seqno,
						 i915_qe_state_str(node),
						 engine->name);

			i915_scheduler_file_queue_dec(node->params.file);
			node->params.file = NULL;
		}
	}

	spin_unlock_irq(&scheduler->lock);
}

/**
 * i915_scheduler_is_engine_preempting - is a pre-emption event in progress?
 * @engine: Engine to query
 * Returns true if a pre-emption event is currently in progress (which would
 * mean that various other operations may be unsafe) or false if not.
 */
bool i915_scheduler_is_engine_preempting(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	uint32_t sched_flags = scheduler->flags[engine->id];

	/*
	 * The scheduler is prevented from sending batches to the hardware
	 * while preemption is in progress (flag bit I915_SF_PREEMPTING).
	 *
	 * Post-preemption (I915_SF_PREEMPTED), the hardware engine will be
	 * empty, and the scheduler therefore needs a chance to run the
	 * delayed work task to retire completed work and restart submission
	 *
	 * Therefore, if either flag is set, the scheduler is busy.
	 */
	if (sched_flags & (I915_SF_PREEMPTING | I915_SF_PREEMPTED))
		return true;

	return false;
}

/**
 * i915_scheduler_is_engine_busy - is the scheduler busy on the given engine?
 * @engine: Engine to query
 * Returns true if the scheduler is busy and cannot immediately perform
 * operations such as submitting a batch buffer to the hardware or false
 * if it is not.
 */
bool i915_scheduler_is_engine_busy(struct intel_engine_cs *engine)
{
	/* Currently only pre-emption ties up the scheduler. */
	return i915_scheduler_is_engine_preempting(engine);
}

/**
 * i915_scheduler_is_engine_flying - does the given engine have in flight batches?
 * @engine: Engine to query
 * Used by TDR to distinguish hung engines (not moving but with work to do)
 * from idle engines (not moving because there is nothing to do). Returns true
 * if the given engine has batches currently executing on the hardware.
 */
bool i915_scheduler_is_engine_flying(struct intel_engine_cs *engine)
{
	struct drm_i915_private *dev_priv = engine->dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;
	unsigned long flags;
	bool found = false;

	/* With the scheduler in bypass mode, no information can be returned. */
	if (!i915.enable_scheduler)
		return true;

	spin_lock_irqsave(&scheduler->lock, flags);

	for_each_scheduler_node(node, engine->id) {
		if (I915_SQS_IS_FLYING(node)) {
			found = true;
			break;
		}
	}

	spin_unlock_irqrestore(&scheduler->lock, flags);

	return found;
}
