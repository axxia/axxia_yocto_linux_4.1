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

	return dev_priv->scheduler != NULL;
}

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
	scheduler->min_flying             = 2;

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
}

/*
 * Give up on a node completely. For example, because it is causing the
 * engine to hang or is using some resource that no longer exists.
 */
static void i915_scheduler_node_kill(struct i915_scheduler_queue_entry *node)
{
	WARN_ON(I915_SQS_IS_COMPLETE(node));

	node->status = I915_SQS_DEAD;
}

/* Mark a node as in flight on the hardware. */
static void i915_scheduler_node_fly(struct i915_scheduler_queue_entry *node)
{
	struct drm_i915_private *dev_priv = node->params.dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct intel_engine_cs *engine = node->params.engine;

	assert_scheduler_lock_held(scheduler);

	WARN_ON(node->status != I915_SQS_POPPED);

	/*
	 * Add the node (which should currently be in state popped) to the
	 * front of the queue. This ensure that flying nodes are always held
	 * in hardware submission order.
	 */
	list_add(&node->link, &scheduler->node_queue[engine->id]);

	node->status = I915_SQS_FLYING;

	if (!(scheduler->flags[engine->id] & I915_SF_INTERRUPTS_ENABLED)) {
		bool success = true;

		success = engine->irq_get(engine);
		if (success)
			scheduler->flags[engine->id] |= I915_SF_INTERRUPTS_ENABLED;
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
	int ret;
	int i;
	bool any_queued = false;
	bool has_local, has_remote, only_remote = false;

	assert_scheduler_lock_held(scheduler);

	*pop_node = NULL;
	ret = -ENODATA;

	for_each_scheduler_node(node, engine->id) {
		if (!I915_SQS_IS_QUEUED(node))
			continue;
		any_queued = true;

		has_local  = false;
		has_remote = false;
		for (i = 0; i < node->num_deps; i++) {
			if (!i915_scheduler_is_dependency_valid(node, i))
				continue;

			if (node->dep_list[i]->params.engine == node->params.engine)
				has_local = true;
			else
				has_remote = true;
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

	*pop_node = best;
	return ret;
}

/*
 * NB: The driver mutex lock must be held before calling this function. It is
 * only really required during the actual back end submission call. However,
 * attempting to acquire a mutex while holding a spin lock is a Bad Idea.
 * And releasing the one before acquiring the other leads to other code
 * being run and interfering.
 */
static int i915_scheduler_submit(struct intel_engine_cs *engine)
{
	struct drm_device *dev = engine->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node;
	int ret, count = 0, flying;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	spin_lock_irq(&scheduler->lock);

	WARN_ON(scheduler->flags[engine->id] & I915_SF_SUBMITTING);
	scheduler->flags[engine->id] |= I915_SF_SUBMITTING;

	/* First time around, complain if anything unexpected occurs: */
	ret = i915_scheduler_pop_from_queue_locked(engine, &node);
	if (ret)
		goto error;

	do {
		WARN_ON(node->params.engine != engine);
		WARN_ON(node->status != I915_SQS_POPPED);
		count++;

		/*
		 * The call to pop above will have removed the node from the
		 * list. So add it back in and mark it as in flight.
		 */
		i915_scheduler_node_fly(node);

		spin_unlock_irq(&scheduler->lock);
		ret = dev_priv->gt.execbuf_final(&node->params);
		spin_lock_irq(&scheduler->lock);

		/*
		 * Handle failed submission but first check that the
		 * watchdog/reset code has not nuked the node while we
		 * weren't looking:
		 */
		if (ret && (node->status != I915_SQS_DEAD)) {
			bool requeue = true;

			/*
			 * Oh dear! Either the node is broken or the engine is
			 * busy. So need to kill the node or requeue it and try
			 * again later as appropriate.
			 */

			switch (-ret) {
			case ENODEV:
			case ENOENT:
				/* Fatal errors. Kill the node. */
				requeue = false;
				i915_scheduler_node_kill(node);
				break;

			case EAGAIN:
			case EBUSY:
			case EIO:
			case ENOMEM:
			case ERESTARTSYS:
			case EINTR:
				/* Supposedly recoverable errors. */
				break;

			default:
				/*
				 * Assume the error is recoverable and hope
				 * for the best.
				 */
				MISSING_CASE(-ret);
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

	scheduler->flags[qe->params.engine->id] |= I915_SF_SUBMITTING;
	ret = dev_priv->gt.execbuf_final(&qe->params);
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
	bool not_flying;
	int i, e;
	int incomplete;

	/* Bypass the scheduler and send the buffer immediately? */
	if (1/*!i915.enable_scheduler*/)
		return i915_scheduler_queue_execbuffer_bypass(qe);

	node = kmalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	*node = *qe;
	INIT_LIST_HEAD(&node->link);
	node->status = I915_SQS_QUEUED;
	node->stamp  = jiffies;
	i915_gem_request_reference(node->params.request);

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

	not_flying = i915_scheduler_count_flying(scheduler, engine) <
						 scheduler->min_flying;

	spin_unlock_irq(&scheduler->lock);

	if (not_flying)
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
bool i915_scheduler_notify_request(struct drm_i915_gem_request *req)
{
	struct drm_i915_private *dev_priv = to_i915(req->engine->dev);
	struct i915_scheduler *scheduler = dev_priv->scheduler;
	struct i915_scheduler_queue_entry *node = req->scheduler_qe;
	unsigned long flags;

	if (!node)
		return false;

	spin_lock_irqsave(&scheduler->lock, flags);

	WARN_ON(!I915_SQS_IS_FLYING(node));

	/* Node was in flight so mark it as complete. */
	if (req->cancelled)
		node->status = I915_SQS_DEAD;
	else
		node->status = I915_SQS_COMPLETE;

	spin_unlock_irqrestore(&scheduler->lock, flags);

	return true;
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

	/* And anything else owned by the node: */
	if (node->params.cliprects) {
		kfree(node->params.cliprects);
		node->params.cliprects = NULL;
	}
}

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

		/* Strip the dependency info while the mutex is still locked */
		i915_scheduler_remove_dependent(scheduler, node);

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

	dev_priv = container_of(work, struct drm_i915_private, mm.scheduler_work);

	for_each_engine(engine, dev_priv)
		i915_scheduler_process_work(engine);
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
				DRM_DEBUG_DRIVER("Closing file handle with outstanding work: %d:%d/%d on %s\n",
						 node->params.request->uniq,
						 node->params.request->seqno,
						 node->status,
						 engine->name);

			node->params.file = NULL;
		}
	}

	spin_unlock_irq(&scheduler->lock);
}
