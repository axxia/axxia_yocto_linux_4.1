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

#include <linux/firmware.h>
#include "intel_drv.h"
#include "i915_drv.h"

static unsigned int bg_color = 0x00000000;

module_param_named(bg_color, bg_color, uint, 0400);
MODULE_PARM_DESC(bg_color, "Set the background (canvas) color while "
		 "doing initial mode set. In XRGB8888 little endian format.");

static char splash[PATH_MAX] = "";
module_param_string(splash, splash, sizeof(splash), 0400);
MODULE_PARM_DESC(splash,
		 "Load a splash screen binary image for a specific display."
		 "splash=<connector>:<image>:w,h,crtc_x,crtc_y,crtc_w,crtc_h");

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

static struct drm_framebuffer *
intel_splash_screen_fb(struct drm_device *dev,
		       struct splash_screen_info *splash_info)
{
	struct drm_framebuffer *fb;
	struct drm_mode_fb_cmd2 mode_cmd = {0};

	if (splash_info->obj == NULL)
		return NULL;

	mode_cmd.width = splash_info->width;
	mode_cmd.height = splash_info->height;

	mode_cmd.pitches[0] = splash_info->pitch * 4;
	mode_cmd.pixel_format = DRM_FORMAT_ARGB8888;

	mutex_lock(&dev->struct_mutex);
	fb = __intel_framebuffer_create(dev, &mode_cmd, splash_info->obj);
	mutex_unlock(&dev->struct_mutex);

	return fb;
}

static char *get_splash_val(char *splash_str, int *val)
{
	char *sep;

	if ((sep = strchr(splash_str, ','))) {
		*val = simple_strtol(splash_str, NULL, 10);
		splash_str = sep + 1;
	} else {
		*val = simple_strtol(splash_str, NULL, 10);
	}

	return splash_str;
}

static struct splash_screen_info *
intel_splash_screen_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct splash_screen_info *splash_info;
	char *splash_dup;
	char *splash_str;
	char *sep;
	u32 fw_npages;

	if (splash[0] == '\0')
		return NULL;

	splash_info = kzalloc(sizeof(struct splash_screen_info), GFP_KERNEL);
	if (splash_info == NULL)
		return NULL;

	dev_priv->splash_screen_info = splash_info;

	splash_dup = kstrdup(splash, GFP_KERNEL);
	splash_str = splash_dup;

	/* Pull connector name from string */
	sep = strchr(splash_str, ':');
	if (sep == NULL)
		goto fail;

	*sep = '\0';
	splash_info->connector_name = kstrdup(splash_str, GFP_KERNEL);
	splash_str = sep + 1;

	/* Pull firmware file name from string */
	sep = strchr(splash_str, ':');
	if (sep == NULL)
		goto fail;

	*sep = '\0';
	request_firmware(&splash_info->fw, splash_str,
			 &dev_priv->drm.pdev->dev);
	if (splash_info->fw == NULL)
		goto fail;
	splash_str = sep + 1;

	/* Pull splash screen width, height, crtc */
	splash_str = get_splash_val(splash_str, &splash_info->width);
	splash_str = get_splash_val(splash_str, &splash_info->height);
	splash_str = get_splash_val(splash_str, &splash_info->pitch);
	splash_str = get_splash_val(splash_str, &splash_info->crtc_x);
	splash_str = get_splash_val(splash_str, &splash_info->crtc_y);
	splash_str = get_splash_val(splash_str, &splash_info->crtc_w);
	splash_str = get_splash_val(splash_str, &splash_info->crtc_h);

	/*
	 * If splash image is baked into the kernel, we just get
	 * a pointer.  Otherwise we'll get a list of pages.
	 */
	fw_npages = DIV_ROUND_UP_ULL(splash_info->fw->size, PAGE_SIZE);
	if (splash_info->fw->pages == NULL)
		splash_info->obj = i915_gem_object_create_splash(dev,
				   splash_info->fw->data, fw_npages);
	else
		splash_info->obj = i915_gem_object_create_splash_pages(dev,
				   splash_info->fw->pages, fw_npages);

	kfree(splash_dup);

	return splash_info;

fail:
	kfree(splash_dup);
	release_firmware(splash_info->fw);
	kfree(splash_info->connector_name);
	kfree(splash_info);
	dev_priv->splash_screen_info = NULL;
	return NULL;
}


static struct drm_display_mode *get_modeline(struct drm_i915_private *dev_priv,
					     struct drm_connector *connector,
					     int width, int height)
{
	struct drm_display_mode *mode;
	struct drm_cmdline_mode *cl_mode = &connector->cmdline_mode;

	/*
	 * fill_modes() takes a bit of time but is necessary.
	 * It is reading the EDID (or loading the EDID firmware blob
	 * and building the connector mode list. The time can be
	 * minimized by using a small EDID blob built into the kernel.
	 */

	connector->funcs->fill_modes(connector, width, height);

	/*
	 * Search the mode list.  If a mode was specified using the
	 * video= command line, use that.  Otherwise look for the
	 * preferred mode.
	 *
	 * <connector:><xres>x<yres>[M][R][-<bpp>][@<refresh>][i][m][eDd]
	 */
	list_for_each_entry(mode, &connector->modes, head) {
		if (cl_mode && cl_mode->specified &&
		    cl_mode->refresh_specified) {
			if (mode->hdisplay == cl_mode->xres &&
			    mode->vdisplay == cl_mode->yres &&
			    mode->vrefresh == cl_mode->refresh)
				return mode;
		} else if (cl_mode && cl_mode->specified) {
			if (mode->hdisplay == cl_mode->xres &&
			    mode->vdisplay == cl_mode->yres)
				return mode;
		} else {
			if (mode->type & DRM_MODE_TYPE_PREFERRED)
				return mode;
		}
	}

	DRM_ERROR("Failed to find a valid mode.\n");
	return NULL;
}

static int update_crtc_state(struct drm_atomic_state *state,
			     struct drm_display_mode *mode,
			     struct drm_crtc *crtc)
{
	struct drm_crtc_state *crtc_state;
	struct drm_rgba bgcolor;
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

	if (!IS_GEN9(state->dev))
	    return 0;

	/* Set the background color based on module parameter */
	bgcolor =drm_rgba(8,
			  (bg_color & 0x000000ff),
			  (bg_color & 0x0000ff00) >> 8,
			  (bg_color & 0x00ff0000) >> 16,
			  (bg_color & 0xff000000) >> 24);

	ret = drm_atomic_crtc_set_property(crtc, crtc_state,
					   state->dev->mode_config.prop_background_color,
					   bgcolor.v);
	WARN_ON(ret);

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
				      struct splash_screen_info *splash_info,
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

	primary_state->crtc_x = splash_info->crtc_x;
	primary_state->crtc_y = splash_info->crtc_y;
	primary_state->crtc_w = (splash_info->crtc_w) ?
		splash_info->crtc_w : hdisplay;
	primary_state->crtc_h = (splash_info->crtc_h) ?
		splash_info->crtc_h : vdisplay;

	primary_state->src_x = 0 << 16;
	primary_state->src_y = 0 << 16;
	primary_state->src_w = splash_info->width << 16;
	primary_state->src_h = splash_info->height << 16;

	return 0;
}

static int update_atomic_state(struct drm_device *dev,
			       struct drm_atomic_state *state,
			       struct splash_screen_info *splash_info,
			       struct drm_framebuffer *fb,
			       struct drm_connector *connector,
			       struct drm_encoder *encoder)

{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_display_mode *mode;
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

	/* set up primary plane if a splash screen is requested */
	if (fb && splash_info) {
		ret = update_primary_plane_state(state, splash_info,
						 crtc, mode, fb);
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
	struct splash_screen_info *splash_info, *info;
	struct drm_framebuffer *fb = NULL;
	bool found = false;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return;

	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
	drm_modeset_lock_all_ctx(dev, &ctx);

	splash_info = intel_splash_screen_init(dev);
	if (splash_info) {
		fb = intel_splash_screen_fb(dev, splash_info);
		if (IS_ERR(fb))
			fb = NULL;
	}

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

		info = NULL;
		if (use_connector(connector)) {
			if (!(encoder = get_encoder(dev, connector)))
				continue;

			if (splash_info &&
			    strcmp(splash_info->connector_name, connector->name) == 0)
					info = splash_info;
			ret = update_atomic_state(dev, state, info, fb,
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

			info = NULL;
			connector->status = connector->funcs->detect(connector,
								     true);
			if (connector->status == connector_status_connected) {
				if (!(encoder = get_encoder(dev, connector)))
					continue;

				if (splash_info &&
				    strcmp(splash_info->connector_name, connector->name) == 0)
					info = splash_info;
				ret = update_atomic_state(dev, state, info, fb,
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

static void initial_mode_destroy(struct drm_device *dev)
{
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	state = drm_atomic_state_alloc(dev);
	if (!state)
		return;

	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
	drm_modeset_lock_all_ctx(dev, &ctx);

retry:
	ret = disable_planes(dev, state);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		drm_atomic_state_clear(state);
		goto retry;
	}

	ret = drm_atomic_commit(state);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
		drm_atomic_state_clear(state);
		goto retry;
	}

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

void intel_initial_mode_config_fini(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct splash_screen_info *splash_info = dev_priv->splash_screen_info;

	flush_work(&dev_priv->initial_modeset_work);

	initial_mode_destroy(dev);

	if (splash_info) {
		release_firmware(splash_info->fw);
		kfree(splash_info->connector_name);
		kfree(splash_info);
	}
}
