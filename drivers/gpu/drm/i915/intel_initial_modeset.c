/*
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions: *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * DOC: Boot-time mode setting.
 *
 * There exists a use case where the kernel graphics needs to be initialized
 * with a valid display configuration with full display pipeline programming
 * in place before user space is initialized and without a fbdev & fb console.
 *
 * The primary motivation is to allow early user space applications to
 * display a frame (or frames) as soon as possible after user space starts.
 * Eliminating the time it takes userspace to program the display configuration
 * benefits this use case.
 *
 * By doing all the display programming in the kernel, it can be done in
 * parallel with other kernel startup tasks without adding significant
 * elapshed time before user space starts.
 */

#include "intel_drv.h"
#include "i915_drv.h"


static int disable_planes(struct drm_device *dev,
			  struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	int ret;

	drm_for_each_plane(plane, dev) {
		struct drm_plane_state *plane_state;

		plane->old_fb = plane->fb;

		plane_state = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state)) {
			return PTR_ERR(plane_state);
		}

		ret = drm_atomic_plane_set_property(plane, plane_state,
					dev->mode_config.rotation_property,
					BIT(DRM_ROTATE_0));
		WARN_ON(ret);

		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret != 0)
			return ret;

		drm_atomic_set_fb_for_plane(plane_state, NULL);
	}

	return 0;
}

/*
 * The modeset_config is scheduled to run via an async
 * schedule call from the main driver load.
 */
static void modeset_config_fn(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, typeof(*dev_priv), initial_modeset_work);
	struct drm_device *dev = &dev_priv->drm;
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_plane *plane;
	int ret;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return;

	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
	drm_modeset_lock_all_ctx(dev, &ctx);

	ret = drm_modeset_lock(&dev->mode_config.connection_mutex,
			       state->acquire_ctx);
	if (ret)
		goto out;

retry:
	ret = disable_planes(dev, state);
	if (ret == 0)
		ret = drm_atomic_commit(state);

	if (ret == -EDEADLK) {
		DRM_DEBUG_KMS("modeset commit deadlock, retry...\n");
		drm_modeset_backoff(&ctx);
		drm_atomic_state_clear(state);
		goto retry;
	}

out:
	if (ret)
		drm_atomic_state_free(state);
	else
		drm_for_each_plane(plane, dev) {
			if (plane->old_fb)
				drm_framebuffer_unreference(plane->old_fb);
		}

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

void intel_initial_mode_config_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	INIT_WORK(&dev_priv->initial_modeset_work, modeset_config_fn);
	schedule_work(&dev_priv->initial_modeset_work);
}

void intel_initial_mode_config_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	flush_work(&dev_priv->initial_modeset_work);
}
