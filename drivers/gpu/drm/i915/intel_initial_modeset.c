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

/*
 * This makes use of the video= kernel command line to determine what
 * connectors to configure. See Documentation/fb/modedb.txt for details
 * on the format.  There are 3 specific cases that are used:
 *
 * 1) video=<connector>
 *      - assume monitor is connected, use EDID preferred mode
 * 2) video=<connector:e>
 *      - use regardless of monitor connected, use EDID prefered mode
 * 3) video=<connector:xres x yres @ refresh e
 *      - use regardless of monitor connected and use specified mode.
 */
static bool use_connector(struct drm_connector *connector)
{
	char *option = NULL;
	struct drm_cmdline_mode *cl_mode = &connector->cmdline_mode;

	fb_get_options(connector->name, &option);
	if (option) {
		if (!drm_mode_parse_command_line_for_connector(option,
							      connector,
							      cl_mode))
			return false;

		if (cl_mode->force == DRM_FORCE_OFF)
			return false;

		connector->status = connector->funcs->detect(connector, true);
		if (connector->status != connector_status_connected) {
			connector->force = cl_mode->force;
			connector->status = connector_status_connected;
		}
		return true;
	}

	return false;
}

static void attach_crtc(struct drm_device *dev, struct drm_encoder *encoder)
{
	struct drm_crtc *possible_crtc;

	drm_for_each_crtc(possible_crtc, dev) {
		if (!(encoder->possible_crtcs & drm_crtc_mask(possible_crtc)))
			continue;
		if (possible_crtc->state->enable)
			continue;
		encoder->crtc = possible_crtc;
		break;
	}
}

static struct drm_encoder *get_encoder(struct drm_device *dev,
				       struct drm_connector *connector)
{
	const struct drm_connector_helper_funcs *connector_funcs;
	struct drm_encoder *encoder;

	connector_funcs = connector->helper_private;
	encoder = connector_funcs->best_encoder(connector);
	if (!encoder) {
		DRM_DEBUG_KMS("connector %s has no encoder\n",
			      connector->name);
		return NULL;
	}

	if (!encoder->crtc) {
		attach_crtc(dev, encoder);
		if (!encoder->crtc)
			return NULL;
	}

	return encoder;
}

static struct drm_display_mode *get_modeline(struct drm_i915_private *dev_priv,
					     struct drm_connector *connector,
					     int width, int height)
{
	struct drm_display_mode *mode = NULL;
	bool found = false;

	/*
	 * fill_modes() takes a bit of time but is necessary.
	 * It is reading the EDID (or loading the EDID firmware blob
	 * and building the connector mode list. The time can be
	 * minimized by using a small EDID blob built into the kernel.
	 */

	connector->funcs->fill_modes(connector, width, height);

	if (!mode) {
		list_for_each_entry(mode, &connector->modes, head) {
			if (mode->hdisplay > width ||
			    mode->vdisplay > height)
				continue;
			if (mode->type & DRM_MODE_TYPE_PREFERRED) {
				found = true;
				break;
			}
		}
	}
	if (!found) {
		DRM_ERROR("Failed to find a valid mode.\n");
		mode = NULL;
	}

	return mode;
}

static int update_crtc_state(struct drm_atomic_state *state,
			     struct drm_display_mode *mode,
			     struct drm_crtc *crtc)
{
	struct drm_crtc_state *crtc_state;
	int ret;

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	ret = drm_atomic_set_mode_for_crtc(crtc_state, mode);
	if (ret) {
		crtc_state->active = false;
		return ret;
	}

	crtc_state->active = true;
	return 0;
}

static int update_connector_state(struct drm_atomic_state *state,
				  struct drm_connector *connector,
				  struct drm_crtc *crtc)
{
	struct drm_connector_state *conn_state;
	int ret;

	conn_state = drm_atomic_get_connector_state(state, connector);
	if (IS_ERR(conn_state)) {
		DRM_DEBUG_KMS("failed to get connector %s state\n",
			      connector->name);
		return PTR_ERR(conn_state);
	}

	ret = drm_atomic_set_crtc_for_connector(conn_state, crtc);
	if (ret) {
		DRM_DEBUG_KMS("failed to set crtc for connector\n");
		return ret;
	}

	return 0;
}

static int update_primary_plane_state(struct drm_atomic_state *state,
				      struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_framebuffer *fb)
{
	int hdisplay, vdisplay;
	struct drm_plane_state *primary_state;
	int ret;

	primary_state = drm_atomic_get_plane_state(state, crtc->primary);
	ret = drm_atomic_set_crtc_for_plane(primary_state, crtc);
	if (ret)
		return ret;
	drm_crtc_get_hv_timing(mode, &hdisplay, &vdisplay);
	drm_atomic_set_fb_for_plane(primary_state, fb);
	primary_state->crtc_x = 0;
	primary_state->crtc_y = 0;
	primary_state->crtc_w = hdisplay;
	primary_state->crtc_h = vdisplay;
	primary_state->src_x = 0 << 16;
	primary_state->src_y = 0 << 16;
	primary_state->src_w = hdisplay << 16;
	primary_state->src_h = vdisplay << 16;

	return 0;
}

static int update_atomic_state(struct drm_device *dev,
			       struct drm_atomic_state *state,
			       struct drm_connector *connector,
			       struct drm_encoder *encoder)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_display_mode *mode;
	struct drm_framebuffer *fb = NULL;
	struct drm_crtc *crtc = encoder->crtc;
	int ret;

	mode = get_modeline(dev_priv, connector,
			    dev->mode_config.max_width,
			    dev->mode_config.max_height);
	if (!mode)
		return -EINVAL;

	ret = update_crtc_state(state, mode, crtc);
	if (ret)
		return ret;

	/* attach connector to atomic state */
	ret = update_connector_state(state, connector, crtc);
	if (ret)
		return ret;

	/* Set up primary plane if a framebuffer is allocated */
	if (fb) {
		ret = update_primary_plane_state(state, crtc, mode, fb);
		if (ret)
			return ret;
	}

	return 0;
}


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
	struct drm_connector *connector;
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;
	struct drm_plane *plane;
	int ret;
	bool found = false;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return;

	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
	drm_modeset_lock_all_ctx(dev, &ctx);

retry:
	ret = disable_planes(dev, state);
	if (ret)
		goto early_fail;

	/*
	 * For each connector that we want to set up, update the atomic
	 * state to include the connector and crtc mode.
	 */
	mutex_lock(&dev->mode_config.mutex);
	drm_for_each_connector(connector, dev) {
		struct drm_encoder *encoder;

		if (use_connector(connector)) {
			if (!(encoder = get_encoder(dev, connector)))
				continue;

			ret = update_atomic_state(dev, state,
					    connector, encoder);
			if (ret)
				goto fail;
			found = true;
		}
	}

	if (!found) {
		/* Try to detect attached connectors */
		drm_for_each_connector(connector, dev) {
			struct drm_encoder *encoder;
			connector->status = connector->funcs->detect(connector,
								     true);
			if (connector->status == connector_status_connected) {
				if (!(encoder = get_encoder(dev, connector)))
					continue;

				ret = update_atomic_state(dev, state,
							  connector, encoder);
				if (ret)
					goto fail;
				found = true;
			}
			if (ret)
				goto fail;
		}
	}

	if (found) {
		ret = drm_modeset_lock(&dev->mode_config.connection_mutex,
				       state->acquire_ctx);
		if (ret)
			goto fail;

		ret = drm_atomic_commit(state);
		if (ret)
			goto fail;
	}
	mutex_unlock(&dev->mode_config.mutex);
	goto out;

fail:
	mutex_unlock(&dev->mode_config.mutex);

early_fail:
	if (ret == -EDEADLK) {
		DRM_DEBUG_KMS("modeset commit deadlock, retry...\n");
		drm_modeset_backoff(&ctx);
		drm_atomic_state_clear(state);
		goto retry;
	}
	drm_atomic_state_free(state);

out:
	if (!ret) {
		drm_for_each_plane(plane, dev) {
			if (plane->old_fb)
				drm_framebuffer_unreference(plane->old_fb);
		}
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
