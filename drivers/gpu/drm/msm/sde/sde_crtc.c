/* Copyright (c) 2015-2016, 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/sort.h>
#include <linux/debugfs.h>
#include <linux/ktime.h>
#include <uapi/drm/sde_drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_flip_work.h>

#include "sde_kms.h"
#include "sde_hw_lm.h"
#include "sde_hw_ctl.h"
#include "sde_crtc.h"

#define CTL(i)       (CTL_0 + (i))
#define LM(i)        (LM_0  + (i))
#define INTF(i)      (INTF_0 + (i))

/* uncomment to enable higher level IRQ msg's */
/*#define DBG_IRQ      DBG*/
#define DBG_IRQ(fmt, ...)

/* default input fence timeout, in ms */
#define SDE_CRTC_INPUT_FENCE_TIMEOUT    2000

/*
 * The default input fence timeout is 2 seconds while max allowed
 * range is 10 seconds. Any value above 10 seconds adds glitches beyond
 * tolerance limit.
 */
#define SDE_CRTC_MAX_INPUT_FENCE_TIMEOUT 10000

static struct sde_kms *get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;
	return to_sde_kms(priv->kms);
}

static inline int sde_crtc_mixer_width(struct sde_crtc *sde_crtc,
	struct drm_display_mode *mode)
{
	if (!sde_crtc || !mode)
		return 0;

	return  sde_crtc->num_mixers == CRTC_DUAL_MIXERS ?
		mode->hdisplay / CRTC_DUAL_MIXERS : mode->hdisplay;
}

static void sde_crtc_destroy(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);

	DBG("");

	if (!crtc)
		return;

	msm_property_destroy(&sde_crtc->property_info);
	debugfs_remove_recursive(sde_crtc->debugfs_root);
	sde_fence_deinit(&sde_crtc->output_fence);

	drm_crtc_cleanup(crtc);
	kfree(sde_crtc);
}

static bool sde_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	DBG("");

	if (msm_is_mode_seamless(adjusted_mode)) {
		SDE_DEBUG("seamless mode set requested\n");
		if (!crtc->enabled || crtc->state->active_changed) {
			SDE_ERROR("crtc state prevents seamless transition\n");
			return false;
		}
	}

	return true;
}

static void sde_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	DBG("");
}

static void sde_crtc_get_blend_cfg(struct sde_hw_blend_cfg *cfg,
		struct sde_plane_state *pstate)
{
	struct drm_plane *plane;
	const struct sde_format *format;
	uint32_t blend_op;

	format = to_sde_format(
			msm_framebuffer_format(pstate->base.fb));
	plane = pstate->base.plane;

	memset(cfg, 0, sizeof(*cfg));

	/* default to opaque blending */
	cfg->fg.alpha_sel = ALPHA_FG_CONST;
	cfg->bg.alpha_sel = ALPHA_BG_CONST;
	cfg->fg.const_alpha =
		sde_plane_get_property32(pstate, PLANE_PROP_ALPHA);
	cfg->bg.const_alpha = 0xFF - cfg->fg.const_alpha;

	blend_op = sde_plane_get_property32(pstate, PLANE_PROP_BLEND_OP);

	if (format->alpha_enable) {
		switch (blend_op) {
		case SDE_DRM_BLEND_OP_PREMULTIPLIED:
			cfg->fg.alpha_sel = ALPHA_FG_CONST;
			cfg->bg.alpha_sel = ALPHA_FG_PIXEL;
			if (cfg->fg.const_alpha != 0xff) {
				cfg->bg.const_alpha = cfg->fg.const_alpha;
				cfg->bg.mod_alpha = 1;
				cfg->bg.inv_alpha_sel = 1;
			} else {
				cfg->bg.inv_mode_alpha = 1;
			}
			break;
		case SDE_DRM_BLEND_OP_COVERAGE:
			cfg->fg.alpha_sel = ALPHA_FG_PIXEL;
			cfg->bg.alpha_sel = ALPHA_FG_PIXEL;
			if (cfg->fg.const_alpha != 0xff) {
				cfg->bg.const_alpha = cfg->fg.const_alpha;
				cfg->fg.mod_alpha = 1;
				cfg->bg.inv_alpha_sel = 1;
				cfg->bg.mod_alpha = 1;
				cfg->bg.inv_mode_alpha = 1;
			} else {
				cfg->bg.inv_mode_alpha = 1;
			}
			break;
		default:
			/* do nothing */
			break;
		}
	} else {
		/* force 100% alpha */
		cfg->fg.const_alpha = 0xFF;
		cfg->bg.const_alpha = 0x00;
	}

	SDE_DEBUG("format 0x%x, alpha_enable %u blend_op %u\n",
			format->base.pixel_format, format->alpha_enable,
			blend_op);
	SDE_DEBUG("fg alpha config %d %d %d %d %d\n",
		cfg->fg.alpha_sel, cfg->fg.const_alpha, cfg->fg.mod_alpha,
		cfg->fg.inv_alpha_sel, cfg->fg.inv_mode_alpha);
	SDE_DEBUG("bg alpha config %d %d %d %d %d\n",
		cfg->bg.alpha_sel, cfg->bg.const_alpha, cfg->bg.mod_alpha,
		cfg->bg.inv_alpha_sel, cfg->bg.inv_mode_alpha);
}

static u32 blend_config_per_mixer(struct drm_crtc *crtc,
	struct sde_crtc *sde_crtc, struct sde_crtc_mixer *mixer,
	struct sde_hw_color3_cfg *alpha_out)
{
	struct drm_plane *plane;
	struct drm_display_mode *mode;

	struct sde_plane_state *pstate;
	struct sde_hw_blend_cfg blend;
	struct sde_hw_ctl *ctl = mixer->hw_ctl;
	struct sde_hw_mixer *lm = mixer->hw_lm;

	u32 flush_mask = 0, crtc_split_width;
	bool dual_pipe = false;

	mode = &crtc->state->adjusted_mode;
	crtc_split_width = sde_crtc_mixer_width(sde_crtc, mode);

	drm_atomic_crtc_for_each_plane(plane, crtc) {
		pstate = to_sde_plane_state(plane->state);
		/*
		 * Always program right lm first if in dual mixer mode,
		 * it could be overwrote later.
		 */
		dual_pipe = (sde_crtc->num_mixers == CRTC_DUAL_MIXERS) ||
				(sde_plane_num_of_phy_pipe(plane) > 1);
		if (dual_pipe)
			sde_crtc->stage_cfg.stage[pstate->stage][1] =
				sde_plane_pipe(plane, 1);
		sde_crtc->stage_cfg.stage[pstate->stage][0] =
			sde_plane_pipe(plane, 0);

		SDE_DEBUG("crtc_id %d pipe %d at stage %d\n",
			crtc->base.id,
			sde_plane_pipe(plane, 0),
			pstate->stage);

		/**
		 * cache the flushmask for this layer
		 * sourcesplit is always enabled, so this layer will
		 * be staged on both the mixers
		 */
		if (dual_pipe)
			ctl->ops.get_bitmask_sspp(ctl, &flush_mask,
					sde_plane_pipe(plane, 1));
		ctl->ops.get_bitmask_sspp(ctl, &flush_mask,
				sde_plane_pipe(plane, 0));

		/* blend config */
		sde_crtc_get_blend_cfg(&blend, pstate);
		lm->ops.setup_blend_config(lm, pstate->stage, &blend);
		alpha_out->keep_fg[pstate->stage] = 1;
	}

	return flush_mask;
}

static void sde_crtc_blend_setup(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_crtc_mixer *mixer = sde_crtc->mixers;
	struct sde_hw_ctl *ctl;
	struct sde_hw_mixer *lm;
	struct sde_hw_color3_cfg alpha_out;

	int i;

	SDE_DEBUG("%s\n", sde_crtc->name);

	if (sde_crtc->num_mixers > CRTC_DUAL_MIXERS) {
		SDE_ERROR("invalid number mixers: %d\n", sde_crtc->num_mixers);
		return;
	}

	if (!sde_kms) {
		SDE_ERROR("invalid sde_kms\n");
		return;
	}

	sinfo = &sde_kms->splash_info;
	if (!sinfo) {
		SDE_ERROR("invalid splash info\n");
		return;
	}

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		if (!mixer[i].hw_lm || !mixer[i].hw_ctl) {
			SDE_ERROR("invalid lm or ctl assigned to mixer\n");
			return;
		}
		mixer[i].mixer_op_mode = 0;
		mixer[i].flush_mask = 0;
		if (mixer[i].hw_ctl->ops.clear_all_blendstages)
			mixer[i].hw_ctl->ops.clear_all_blendstages(
					mixer[i].hw_ctl,
					sinfo->handoff,
					sinfo->reserved_pipe_info,
					MAX_BLOCKS);
	}

	rp = &to_sde_crtc_state(state)->rp;
	return _sde_crtc_rp_add(rp, type, tag, val, ops);
}

void *sde_crtc_res_get(struct drm_crtc_state *state, u32 type, u64 tag)
{
	struct sde_crtc_respool *rp;
	void *val;

	if (!state) {
		SDE_ERROR("invalid parameters\n");
		return NULL;
	}

	rp = &to_sde_crtc_state(state)->rp;
	val = _sde_crtc_rp_get(rp, type, tag);
	if (IS_ERR(val)) {
		SDE_ERROR("failed to get res type:0x%x:0x%llx\n",
				type, tag);
		return NULL;
	}

	return val;
}

void sde_crtc_res_put(struct drm_crtc_state *state, u32 type, u64 tag)
{
	struct sde_crtc_respool *rp;

	if (!state) {
		SDE_ERROR("invalid parameters\n");
		return;
	}

	rp = &to_sde_crtc_state(state)->rp;
	_sde_crtc_rp_put(rp, type, tag);
}

static void _sde_crtc_deinit_events(struct sde_crtc *sde_crtc)
{
	if (!sde_crtc)
		return;
}

static int _sde_debugfs_fps_status_show(struct seq_file *s, void *data)
{
	struct sde_crtc *sde_crtc;
	u64 fps_int, fps_float;
	ktime_t current_time_us;
	u64 fps, diff_us;

	if (!s || !s->private) {
		SDE_ERROR("invalid input param(s)\n");
		return -EAGAIN;
	}

	sde_crtc = s->private;

	current_time_us = ktime_get();
	diff_us = (u64)ktime_us_delta(current_time_us,
			sde_crtc->fps_info.last_sampled_time_us);

	if (diff_us >= CRTC_TIME_PERIOD_CALC_FPS_US) {
		fps = ((u64)sde_crtc->fps_info.frame_count) * 10000000;
		do_div(fps, diff_us);
		sde_crtc->fps_info.measured_fps = (unsigned int)fps;
		sde_crtc->fps_info.last_sampled_time_us = current_time_us;
		sde_crtc->fps_info.frame_count = 0;
		SDE_DEBUG("Measured FPS for crtc%d is %d.%d\n",
				sde_crtc->base.base.id, (unsigned int)fps/10,
				(unsigned int)fps%10);
	}

	fps_int = (unsigned int) sde_crtc->fps_info.measured_fps;
	fps_float = do_div(fps_int, 10);

	seq_printf(s, "fps: %llu.%llu\n", fps_int, fps_float);

	return 0;
}


static int _sde_debugfs_fps_status(struct inode *inode, struct file *file)
{
	return single_open(file, _sde_debugfs_fps_status_show,
			inode->i_private);
}

static ssize_t vsync_event_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct drm_crtc *crtc;
	struct sde_crtc *sde_crtc;

	if (!device || !buf) {
		SDE_ERROR("invalid input param(s)\n");
		return -EAGAIN;
	}

	crtc = dev_get_drvdata(device);
	sde_crtc = to_sde_crtc(crtc);
	return scnprintf(buf, PAGE_SIZE, "VSYNC=%llu\n",
			ktime_to_ns(sde_crtc->vblank_last_cb_time));
}

static DEVICE_ATTR_RO(vsync_event);
static struct attribute *sde_crtc_dev_attrs[] = {
	&dev_attr_vsync_event.attr,
	NULL
};

static const struct attribute_group sde_crtc_attr_group = {
	.attrs = sde_crtc_dev_attrs,
};

static const struct attribute_group *sde_crtc_attr_groups[] = {
	&sde_crtc_attr_group,
	NULL,
};

static void sde_crtc_destroy(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);

	SDE_DEBUG("\n");

	if (!crtc)
		return;

	if (sde_crtc->vsync_event_sf)
		sysfs_put(sde_crtc->vsync_event_sf);
	if (sde_crtc->sysfs_dev)
		device_unregister(sde_crtc->sysfs_dev);

	if (sde_crtc->blob_info)
		drm_property_unreference_blob(sde_crtc->blob_info);
	msm_property_destroy(&sde_crtc->property_info);
	sde_cp_crtc_destroy_properties(crtc);

	sde_fence_deinit(&sde_crtc->output_fence);
	_sde_crtc_deinit_events(sde_crtc);

	drm_crtc_cleanup(crtc);
	mutex_destroy(&sde_crtc->crtc_lock);
	kfree(sde_crtc);
}

static bool sde_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	SDE_DEBUG("\n");

	if ((msm_is_mode_seamless(adjusted_mode) ||
			msm_is_mode_seamless_vrr(adjusted_mode)) &&
		(!crtc->enabled)) {
		SDE_ERROR("crtc state prevents seamless transition\n");
		return false;
	}

	return true;
}
static int _sde_crtc_get_ctlstart_timeout(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	int rc = 0;

	if (!crtc || !crtc->dev)
		return 0;

	list_for_each_entry(encoder,
			&crtc->dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;

		if (sde_encoder_get_intf_mode(encoder) == INTF_MODE_CMD)
			rc += sde_encoder_get_ctlstart_timeout_state(encoder);
	}

	return rc;
}

static void _sde_crtc_setup_blend_cfg(struct sde_crtc_mixer *mixer,
	struct sde_plane_state *pstate, struct sde_format *format)
{
	uint32_t blend_op, fg_alpha, bg_alpha;
	uint32_t blend_type;
	struct sde_hw_mixer *lm = mixer->hw_lm;

	/* default to opaque blending */
	fg_alpha = sde_plane_get_property(pstate, PLANE_PROP_ALPHA);
	bg_alpha = 0xFF - fg_alpha;
	blend_op = SDE_BLEND_FG_ALPHA_FG_CONST | SDE_BLEND_BG_ALPHA_BG_CONST;
	blend_type = sde_plane_get_property(pstate, PLANE_PROP_BLEND_OP);

	SDE_DEBUG("blend type:0x%x blend alpha:0x%x\n", blend_type, fg_alpha);

	switch (blend_type) {

	case SDE_DRM_BLEND_OP_OPAQUE:
		blend_op = SDE_BLEND_FG_ALPHA_FG_CONST |
			SDE_BLEND_BG_ALPHA_BG_CONST;
		break;

	case SDE_DRM_BLEND_OP_PREMULTIPLIED:
		if (format->alpha_enable) {
			blend_op = SDE_BLEND_FG_ALPHA_FG_CONST |
				SDE_BLEND_BG_ALPHA_FG_PIXEL;
			if (fg_alpha != 0xff) {
				bg_alpha = fg_alpha;
				blend_op |= SDE_BLEND_BG_MOD_ALPHA |
					SDE_BLEND_BG_INV_MOD_ALPHA;
			} else {
				blend_op |= SDE_BLEND_BG_INV_ALPHA;
			}
		}
		break;

	case SDE_DRM_BLEND_OP_COVERAGE:
		if (format->alpha_enable) {
			blend_op = SDE_BLEND_FG_ALPHA_FG_PIXEL |
				SDE_BLEND_BG_ALPHA_FG_PIXEL;
			if (fg_alpha != 0xff) {
				bg_alpha = fg_alpha;
				blend_op |= SDE_BLEND_FG_MOD_ALPHA |
					SDE_BLEND_FG_INV_MOD_ALPHA |
					SDE_BLEND_BG_MOD_ALPHA |
					SDE_BLEND_BG_INV_MOD_ALPHA;
			} else {
				blend_op |= SDE_BLEND_BG_INV_ALPHA;
			}
		}
		break;
	default:
		/* do nothing */
		break;
	}

	lm->ops.setup_blend_config(lm, pstate->stage, fg_alpha,
						bg_alpha, blend_op);
	SDE_DEBUG(
		"format: %4.4s, alpha_enable %u fg alpha:0x%x bg alpha:0x%x blend_op:0x%x\n",
		(char *) &format->base.pixel_format,
		format->alpha_enable, fg_alpha, bg_alpha, blend_op);
}

static void _sde_crtc_setup_dim_layer_cfg(struct drm_crtc *crtc,
		struct sde_crtc *sde_crtc, struct sde_crtc_mixer *mixer,
		struct sde_hw_dim_layer *dim_layer)
{
	struct sde_crtc_state *cstate;
	struct sde_hw_mixer *lm;
	struct sde_hw_dim_layer split_dim_layer;
	int i;

	if (!dim_layer->rect.w || !dim_layer->rect.h) {
		SDE_DEBUG("empty dim_layer\n");
		return;
	}

	cstate = to_sde_crtc_state(crtc->state);

	SDE_DEBUG("dim_layer - flags:%d, stage:%d\n",
			dim_layer->flags, dim_layer->stage);

	split_dim_layer.stage = dim_layer->stage;
	split_dim_layer.color_fill = dim_layer->color_fill;

	/*
	 * traverse through the layer mixers attached to crtc and find the
	 * intersecting dim layer rect in each LM and program accordingly.
	 */
	for (i = 0; i < sde_crtc->num_mixers; i++) {
		split_dim_layer.flags = dim_layer->flags;

		sde_kms_rect_intersect(&cstate->lm_roi[i], &dim_layer->rect,
					&split_dim_layer.rect);
		if (sde_kms_rect_is_null(&split_dim_layer.rect)) {
			/*
			 * no extra programming required for non-intersecting
			 * layer mixers with INCLUSIVE dim layer
			 */
			if (split_dim_layer.flags & SDE_DRM_DIM_LAYER_INCLUSIVE)
				continue;

			/*
			 * program the other non-intersecting layer mixers with
			 * INCLUSIVE dim layer of full size for uniformity
			 * with EXCLUSIVE dim layer config.
			 */
			split_dim_layer.flags &= ~SDE_DRM_DIM_LAYER_EXCLUSIVE;
			split_dim_layer.flags |= SDE_DRM_DIM_LAYER_INCLUSIVE;
			memcpy(&split_dim_layer.rect, &cstate->lm_bounds[i],
					sizeof(split_dim_layer.rect));

		} else {
			split_dim_layer.rect.x =
					split_dim_layer.rect.x -
						cstate->lm_roi[i].x;
			split_dim_layer.rect.y =
					split_dim_layer.rect.y -
						cstate->lm_roi[i].y;
		}

		SDE_EVT32_VERBOSE(DRMID(crtc),
				cstate->lm_roi[i].x,
				cstate->lm_roi[i].y,
				cstate->lm_roi[i].w,
				cstate->lm_roi[i].h,
				dim_layer->rect.x,
				dim_layer->rect.y,
				dim_layer->rect.w,
				dim_layer->rect.h,
				split_dim_layer.rect.x,
				split_dim_layer.rect.y,
				split_dim_layer.rect.w,
				split_dim_layer.rect.h);

		SDE_DEBUG("split_dim_layer - LM:%d, rect:{%d,%d,%d,%d}}\n",
			i, split_dim_layer.rect.x, split_dim_layer.rect.y,
			split_dim_layer.rect.w, split_dim_layer.rect.h);

		lm = mixer[i].hw_lm;
		mixer[i].mixer_op_mode |= 1 << split_dim_layer.stage;
		lm->ops.setup_dim_layer(lm, &split_dim_layer);
	}
}

void sde_crtc_get_crtc_roi(struct drm_crtc_state *state,
		const struct sde_rect **crtc_roi)
{
	struct sde_crtc_state *crtc_state;

	if (!state || !crtc_roi)
		return;

	crtc_state = to_sde_crtc_state(state);
	*crtc_roi = &crtc_state->crtc_roi;
}

bool sde_crtc_is_crtc_roi_dirty(struct drm_crtc_state *state)
{
	struct sde_crtc_state *cstate;
	struct sde_crtc *sde_crtc;

	if (!state || !state->crtc)
		return false;

	sde_crtc = to_sde_crtc(state->crtc);
	cstate = to_sde_crtc_state(state);

	return msm_property_is_dirty(&sde_crtc->property_info,
			&cstate->property_state, CRTC_PROP_ROI_V1);
}

static int _sde_crtc_set_roi_v1(struct drm_crtc_state *state,
		void __user *usr_ptr)
{
	struct drm_crtc *crtc;
	struct sde_crtc_state *cstate;
	struct sde_drm_roi_v1 roi_v1;
	int i;

	if (!state) {
		SDE_ERROR("invalid args\n");
		return -EINVAL;
	}

	cstate = to_sde_crtc_state(state);
	crtc = cstate->base.crtc;

	memset(&cstate->user_roi_list, 0, sizeof(cstate->user_roi_list));

	if (!usr_ptr) {
		SDE_DEBUG("crtc%d: rois cleared\n", DRMID(crtc));
		return 0;
	}

	if (copy_from_user(&roi_v1, usr_ptr, sizeof(roi_v1))) {
		SDE_ERROR("crtc%d: failed to copy roi_v1 data\n", DRMID(crtc));
		return -EINVAL;
	}

	SDE_DEBUG("crtc%d: num_rects %d\n", DRMID(crtc), roi_v1.num_rects);

	if (roi_v1.num_rects == 0) {
		SDE_DEBUG("crtc%d: rois cleared\n", DRMID(crtc));
		return 0;
	}

	if (roi_v1.num_rects > SDE_MAX_ROI_V1) {
		SDE_ERROR("crtc%d: too many rects specified: %d\n", DRMID(crtc),
				roi_v1.num_rects);
		return -EINVAL;
	}

	cstate->user_roi_list.num_rects = roi_v1.num_rects;
	for (i = 0; i < roi_v1.num_rects; ++i) {
		cstate->user_roi_list.roi[i] = roi_v1.roi[i];
		SDE_DEBUG("crtc%d: roi%d: roi (%d,%d) (%d,%d)\n",
				DRMID(crtc), i,
				cstate->user_roi_list.roi[i].x1,
				cstate->user_roi_list.roi[i].y1,
				cstate->user_roi_list.roi[i].x2,
				cstate->user_roi_list.roi[i].y2);
		SDE_EVT32_VERBOSE(DRMID(crtc),
				cstate->user_roi_list.roi[i].x1,
				cstate->user_roi_list.roi[i].y1,
				cstate->user_roi_list.roi[i].x2,
				cstate->user_roi_list.roi[i].y2);
	}

	return 0;
}

static bool _sde_crtc_setup_is_3dmux_dsc(struct drm_crtc_state *state)
{
	int i;
	struct sde_crtc_state *cstate;
	bool is_3dmux_dsc = false;

	cstate = to_sde_crtc_state(state);

	for (i = 0; i < cstate->num_connectors; i++) {
		struct drm_connector *conn = cstate->connectors[i];

		if (sde_connector_get_topology_name(conn) ==
				SDE_RM_TOPOLOGY_DUALPIPE_3DMERGE_DSC)
			is_3dmux_dsc = true;
	}

	return is_3dmux_dsc;
}

static int _sde_crtc_set_crtc_roi(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	struct sde_rect *crtc_roi;
	struct msm_mode_info mode_info;
	int i = 0;
	int rc;
	bool is_crtc_roi_dirty;
	bool is_any_conn_roi_dirty;

	if (!crtc || !state)
		return -EINVAL;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);
	crtc_roi = &crtc_state->crtc_roi;

	is_crtc_roi_dirty = sde_crtc_is_crtc_roi_dirty(state);
	is_any_conn_roi_dirty = false;

	for_each_connector_in_state(state->state, conn, conn_state, i) {
		struct sde_connector *sde_conn;
		struct sde_connector_state *sde_conn_state;
		struct sde_rect conn_roi;

		if (!conn_state || conn_state->crtc != crtc)
			continue;

		rc = sde_connector_get_mode_info(conn_state, &mode_info);
		if (rc) {
			SDE_ERROR("failed to get mode info\n");
			return -EINVAL;
		}

		if (!mode_info.roi_caps.enabled)
			continue;

		sde_conn = to_sde_connector(conn_state->connector);
		sde_conn_state = to_sde_connector_state(conn_state);

		is_any_conn_roi_dirty = is_any_conn_roi_dirty ||
				msm_property_is_dirty(
						&sde_conn->property_info,
						&sde_conn_state->property_state,
						CONNECTOR_PROP_ROI_V1);

		/*
		 * current driver only supports same connector and crtc size,
		 * but if support for different sizes is added, driver needs
		 * to check the connector roi here to make sure is full screen
		 * for dsc 3d-mux topology that doesn't support partial update.
		 */
		if (memcmp(&sde_conn_state->rois, &crtc_state->user_roi_list,
				sizeof(crtc_state->user_roi_list))) {
			SDE_ERROR("%s: crtc -> conn roi scaling unsupported\n",
					sde_crtc->name);
			return -EINVAL;
		}

		sde_kms_rect_merge_rectangles(&sde_conn_state->rois, &conn_roi);
		SDE_EVT32_VERBOSE(DRMID(crtc), DRMID(conn),
				conn_roi.x, conn_roi.y,
				conn_roi.w, conn_roi.h);
	}

	/*
	 * Check against CRTC ROI and Connector ROI not being updated together.
	 * This restriction should be relaxed when Connector ROI scaling is
	 * supported.
	 */
	if (is_any_conn_roi_dirty != is_crtc_roi_dirty) {
		SDE_ERROR("connector/crtc rois not updated together\n");
		return -EINVAL;
	}

	sde_kms_rect_merge_rectangles(&crtc_state->user_roi_list, crtc_roi);

	/* clear the ROI to null if it matches full screen anyways */
	if (crtc_roi->x == 0 && crtc_roi->y == 0 &&
			crtc_roi->w == state->adjusted_mode.hdisplay &&
			crtc_roi->h == state->adjusted_mode.vdisplay)
		memset(crtc_roi, 0, sizeof(*crtc_roi));

	SDE_DEBUG("%s: crtc roi (%d,%d,%d,%d)\n", sde_crtc->name,
			crtc_roi->x, crtc_roi->y, crtc_roi->w, crtc_roi->h);
	SDE_EVT32_VERBOSE(DRMID(crtc), crtc_roi->x, crtc_roi->y, crtc_roi->w,
			crtc_roi->h);

	return 0;
}

static int _sde_crtc_check_autorefresh(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	int i;

	if (!crtc || !state)
		return -EINVAL;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);

	if (sde_kms_rect_is_null(&crtc_state->crtc_roi))
		return 0;

	/* partial update active, check if autorefresh is also requested */
	for_each_connector_in_state(state->state, conn, conn_state, i) {
		uint64_t autorefresh;

		if (!conn_state || conn_state->crtc != crtc)
			continue;

		autorefresh = sde_connector_get_property(conn_state,
				CONNECTOR_PROP_AUTOREFRESH);
		if (autorefresh) {
			SDE_ERROR(
				"%s: autorefresh & partial crtc roi incompatible %llu\n",
					sde_crtc->name, autorefresh);
			return -EINVAL;
		}
	}

	return 0;
}

static int _sde_crtc_set_lm_roi(struct drm_crtc *crtc,
		struct drm_crtc_state *state, int lm_idx)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	const struct sde_rect *crtc_roi;
	const struct sde_rect *lm_bounds;
	struct sde_rect *lm_roi;

	if (!crtc || !state || lm_idx >= ARRAY_SIZE(crtc_state->lm_bounds))
		return -EINVAL;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);
	crtc_roi = &crtc_state->crtc_roi;
	lm_bounds = &crtc_state->lm_bounds[lm_idx];
	lm_roi = &crtc_state->lm_roi[lm_idx];

	if (sde_kms_rect_is_null(crtc_roi))
		memcpy(lm_roi, lm_bounds, sizeof(*lm_roi));
	else
		sde_kms_rect_intersect(crtc_roi, lm_bounds, lm_roi);

	SDE_DEBUG("%s: lm%d roi (%d,%d,%d,%d)\n", sde_crtc->name, lm_idx,
			lm_roi->x, lm_roi->y, lm_roi->w, lm_roi->h);

	/*
	 * partial update is not supported with 3dmux dsc or dest scaler.
	 * hence, crtc roi must match the mixer dimensions.
	 */
	if (crtc_state->num_ds_enabled ||
		_sde_crtc_setup_is_3dmux_dsc(state)) {
		if (memcmp(lm_roi, lm_bounds, sizeof(struct sde_rect))) {
			SDE_ERROR("Unsupported: Dest scaler/3d mux DSC + PU\n");
			return -EINVAL;
		}
	}

	/* if any dimension is zero, clear all dimensions for clarity */
	if (sde_kms_rect_is_null(lm_roi))
		memset(lm_roi, 0, sizeof(*lm_roi));

	return 0;
}

static u32 _sde_crtc_get_displays_affected(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	u32 disp_bitmask = 0;
	int i;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);

	/* pingpong split: one ROI, one LM, two physical displays */
	if (crtc_state->is_ppsplit) {
		u32 lm_split_width = crtc_state->lm_bounds[0].w / 2;
		struct sde_rect *roi = &crtc_state->lm_roi[0];

		if (sde_kms_rect_is_null(roi))
			disp_bitmask = 0;
		else if ((u32)roi->x + (u32)roi->w <= lm_split_width)
			disp_bitmask = BIT(0);		/* left only */
		else if (roi->x >= lm_split_width)
			disp_bitmask = BIT(1);		/* right only */
		else
			disp_bitmask = BIT(0) | BIT(1); /* left and right */
	} else {
		for (i = 0; i < sde_crtc->num_mixers; i++) {
			if (!sde_kms_rect_is_null(&crtc_state->lm_roi[i]))
				disp_bitmask |= BIT(i);
		}
	}

	SDE_DEBUG("affected displays 0x%x\n", disp_bitmask);

	return disp_bitmask;
}

static int _sde_crtc_check_rois_centered_and_symmetric(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	const struct sde_rect *roi[CRTC_DUAL_MIXERS];

	if (!crtc || !state)
		return -EINVAL;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);

	if (sde_crtc->num_mixers > CRTC_DUAL_MIXERS) {
		SDE_ERROR("%s: unsupported number of mixers: %d\n",
				sde_crtc->name, sde_crtc->num_mixers);
		return -EINVAL;
	}

	/*
	 * If using pingpong split: one ROI, one LM, two physical displays
	 * then the ROI must be centered on the panel split boundary and
	 * be of equal width across the split.
	 */
	if (crtc_state->is_ppsplit) {
		u16 panel_split_width;
		u32 display_mask;

		roi[0] = &crtc_state->lm_roi[0];

		if (sde_kms_rect_is_null(roi[0]))
			return 0;

		display_mask = _sde_crtc_get_displays_affected(crtc, state);
		if (display_mask != (BIT(0) | BIT(1)))
			return 0;

		panel_split_width = crtc_state->lm_bounds[0].w / 2;
		if (roi[0]->x + roi[0]->w / 2 != panel_split_width) {
			SDE_ERROR("%s: roi x %d w %d split %d\n",
					sde_crtc->name, roi[0]->x, roi[0]->w,
					panel_split_width);
			return -EINVAL;
		}

		return 0;
	}

	/*
	 * On certain HW, if using 2 LM, ROIs must be split evenly between the
	 * LMs and be of equal width.
	 */
	if (sde_crtc->num_mixers < 2)
		return 0;

	roi[0] = &crtc_state->lm_roi[0];
	roi[1] = &crtc_state->lm_roi[1];

	/* if one of the roi is null it's a left/right-only update */
	if (sde_kms_rect_is_null(roi[0]) || sde_kms_rect_is_null(roi[1]))
		return 0;

	/* check lm rois are equal width & first roi ends at 2nd roi */
	if (roi[0]->x + roi[0]->w != roi[1]->x || roi[0]->w != roi[1]->w) {
		SDE_ERROR(
			"%s: rois not centered and symmetric: roi0 x %d w %d roi1 x %d w %d\n",
				sde_crtc->name, roi[0]->x, roi[0]->w,
				roi[1]->x, roi[1]->w);
		return -EINVAL;
	}

	return 0;
}

static int _sde_crtc_check_planes_within_crtc_roi(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	const struct sde_rect *crtc_roi;
	const struct drm_plane_state *pstate;
	struct drm_plane *plane;

	if (!crtc || !state)
		return -EINVAL;

	/*
	 * Reject commit if a Plane CRTC destination coordinates fall outside
	 * the partial CRTC ROI. LM output is determined via connector ROIs,
	 * if they are specified, not Plane CRTC ROIs.
	 */

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(state);
	crtc_roi = &crtc_state->crtc_roi;

	if (sde_kms_rect_is_null(crtc_roi))
		return 0;

	drm_atomic_crtc_state_for_each_plane_state(plane, pstate, state) {
		struct sde_rect plane_roi, intersection;

		if (IS_ERR_OR_NULL(pstate)) {
			int rc = PTR_ERR(pstate);

			SDE_ERROR("%s: failed to get plane%d state, %d\n",
					sde_crtc->name, plane->base.id, rc);
			return rc;
		}

		plane_roi.x = pstate->crtc_x;
		plane_roi.y = pstate->crtc_y;
		plane_roi.w = pstate->crtc_w;
		plane_roi.h = pstate->crtc_h;
		sde_kms_rect_intersect(crtc_roi, &plane_roi, &intersection);
		if (!sde_kms_rect_is_equal(&plane_roi, &intersection)) {
			SDE_ERROR(
				"%s: plane%d crtc roi (%d,%d,%d,%d) outside crtc roi (%d,%d,%d,%d)\n",
					sde_crtc->name, plane->base.id,
					plane_roi.x, plane_roi.y,
					plane_roi.w, plane_roi.h,
					crtc_roi->x, crtc_roi->y,
					crtc_roi->w, crtc_roi->h);
			return -E2BIG;
		}
	}

	return 0;
}

static int _sde_crtc_check_rois(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *sde_crtc_state;
	struct msm_mode_info mode_info;
	int rc, lm_idx, i;

	if (!crtc || !state)
		return -EINVAL;

	memset(&mode_info, 0, sizeof(mode_info));

	sde_crtc = to_sde_crtc(crtc);
	sde_crtc_state = to_sde_crtc_state(state);

	/*
	 * check connector array cached at modeset time since incoming atomic
	 * state may not include any connectors if they aren't modified
	 */
	for (i = 0; i < ARRAY_SIZE(sde_crtc_state->connectors); i++) {
		struct drm_connector *conn = sde_crtc_state->connectors[i];

		if (!conn || !conn->state)
			continue;

		rc = sde_connector_get_mode_info(conn->state, &mode_info);
		if (rc) {
			SDE_ERROR("failed to get mode info\n");
			return -EINVAL;
		}

		if (!mode_info.roi_caps.enabled)
			continue;

		if (sde_crtc_state->user_roi_list.num_rects >
				mode_info.roi_caps.num_roi) {
			SDE_ERROR("roi count is exceeding limit, %d > %d\n",
					sde_crtc_state->user_roi_list.num_rects,
					mode_info.roi_caps.num_roi);
			return -E2BIG;
		}

		rc = _sde_crtc_set_crtc_roi(crtc, state);
		if (rc)
			return rc;

		rc = _sde_crtc_check_autorefresh(crtc, state);
		if (rc)
			return rc;

		for (lm_idx = 0; lm_idx < sde_crtc->num_mixers; lm_idx++) {
			rc = _sde_crtc_set_lm_roi(crtc, state, lm_idx);
			if (rc)
				return rc;
		}

		rc = _sde_crtc_check_rois_centered_and_symmetric(crtc, state);
		if (rc)
			return rc;

		rc = _sde_crtc_check_planes_within_crtc_roi(crtc, state);
		if (rc)
			return rc;
	}

	return 0;
}

static void _sde_crtc_program_lm_output_roi(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *crtc_state;
	const struct sde_rect *lm_roi;
	struct sde_hw_mixer *hw_lm;
	int lm_idx, lm_horiz_position;

	if (!crtc)
		return;

	sde_crtc = to_sde_crtc(crtc);
	crtc_state = to_sde_crtc_state(crtc->state);

	lm_horiz_position = 0;
	for (lm_idx = 0; lm_idx < sde_crtc->num_mixers; lm_idx++) {
		struct sde_hw_mixer_cfg cfg;

		lm_roi = &crtc_state->lm_roi[lm_idx];
		hw_lm = sde_crtc->mixers[lm_idx].hw_lm;

		SDE_EVT32(DRMID(crtc_state->base.crtc), lm_idx,
			lm_roi->x, lm_roi->y, lm_roi->w, lm_roi->h);

		if (sde_kms_rect_is_null(lm_roi))
			continue;

		hw_lm->cfg.out_width = lm_roi->w;
		hw_lm->cfg.out_height = lm_roi->h;
		hw_lm->cfg.right_mixer = lm_horiz_position;

		cfg.out_width = lm_roi->w;
		cfg.out_height = lm_roi->h;
		cfg.right_mixer = lm_horiz_position++;
		cfg.flags = 0;
		hw_lm->ops.setup_mixer_out(hw_lm, &cfg);
	}
}

/**
 * _sde_crtc_calc_inline_prefill - calculate rotator start prefill
 * @crtc: Pointer to drm crtc
 * return: prefill time in lines
 */
static u32 _sde_crtc_calc_inline_prefill(struct drm_crtc *crtc)
{
	struct sde_kms *sde_kms;

	if (!crtc) {
		SDE_ERROR("invalid parameters\n");
		return 0;
	}

	sde_kms = _sde_crtc_get_kms(crtc);
	if (!sde_kms || !sde_kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return 0;
	}

	return sde_kms->catalog->sbuf_prefill + sde_kms->catalog->sbuf_headroom;
}

uint64_t sde_crtc_get_sbuf_clk(struct drm_crtc_state *state)
{
	struct sde_crtc_state *cstate;
	u64 tmp;

	if (!state) {
		SDE_ERROR("invalid crtc state\n");
		return 0;
	}
	cstate = to_sde_crtc_state(state);

	/*
	 * Select the max of the current and previous frame's user mode
	 * clock setting so that reductions in clock voting don't take effect
	 * until the current frame has completed.
	 *
	 * If the sbuf_clk_rate[] FIFO hasn't yet been updated in this commit
	 * cycle (as part of the CRTC's atomic check), compare the current
	 * clock value against sbuf_clk_rate[1] instead of comparing the
	 * sbuf_clk_rate[0]/sbuf_clk_rate[1] values.
	 */
	if (cstate->sbuf_clk_shifted)
		tmp = cstate->sbuf_clk_rate[0];
	else
		tmp = sde_crtc_get_property(cstate, CRTC_PROP_ROT_CLK);

	return max_t(u64, cstate->sbuf_clk_rate[1], tmp);
}

static void _sde_crtc_blend_setup_mixer(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state, struct sde_crtc *sde_crtc,
		struct sde_crtc_mixer *mixer)
{
	struct drm_plane *plane;
	struct drm_framebuffer *fb;
	struct drm_plane_state *state;
	struct sde_crtc_state *cstate;
	struct sde_plane_state *pstate = NULL;
	struct sde_format *format;
	struct sde_hw_ctl *ctl;
	struct sde_hw_mixer *lm;
	struct sde_hw_stage_cfg *stage_cfg;
	struct sde_rect plane_crtc_roi;

	u32 flush_mask, flush_sbuf, prefill;
	uint32_t stage_idx, lm_idx;
	int zpos_cnt[SDE_STAGE_MAX + 1] = { 0 };
	int i;
	bool bg_alpha_enable = false;

	if (!sde_crtc || !crtc->state || !mixer) {
		SDE_ERROR("invalid sde_crtc or mixer\n");
		return;
	}

	ctl = mixer->hw_ctl;
	lm = mixer->hw_lm;
	stage_cfg = &sde_crtc->stage_cfg;
	cstate = to_sde_crtc_state(crtc->state);

	cstate->sbuf_prefill_line = _sde_crtc_calc_inline_prefill(crtc);
	sde_crtc->sbuf_flush_mask_old = sde_crtc->sbuf_flush_mask_all;
	sde_crtc->sbuf_flush_mask_all = 0x0;
	sde_crtc->sbuf_flush_mask_delta = 0x0;

	drm_atomic_crtc_for_each_plane(plane, crtc) {
		state = plane->state;
		if (!state)
			continue;

		plane_crtc_roi.x = state->crtc_x;
		plane_crtc_roi.y = state->crtc_y;
		plane_crtc_roi.w = state->crtc_w;
		plane_crtc_roi.h = state->crtc_h;

		pstate = to_sde_plane_state(state);
		fb = state->fb;

		/* assume all rotated planes report the same prefill amount */
		prefill = sde_plane_rot_get_prefill(plane);
		if (prefill)
			cstate->sbuf_prefill_line = prefill;

		sde_plane_get_ctl_flush(plane, ctl, &flush_mask, &flush_sbuf);

		/* save sbuf flush value for later */
		if (old_state && drm_atomic_get_existing_plane_state(
					old_state->state, plane))
			sde_crtc->sbuf_flush_mask_delta |= flush_sbuf;
		sde_crtc->sbuf_flush_mask_all |= flush_sbuf;

		SDE_DEBUG("crtc %d stage:%d - plane %d sspp %d fb %d\n",
				crtc->base.id,
				pstate->stage,
				plane->base.id,
				sde_plane_pipe(plane) - SSPP_VIG0,
				state->fb ? state->fb->base.id : -1);

		format = to_sde_format(msm_framebuffer_format(pstate->base.fb));
		if (!format) {
			SDE_ERROR("invalid format\n");
			return;
		}

		if (pstate->stage == SDE_STAGE_BASE && format->alpha_enable)
			bg_alpha_enable = true;

		SDE_EVT32(DRMID(crtc), DRMID(plane),
				state->fb ? state->fb->base.id : -1,
				state->src_x >> 16, state->src_y >> 16,
				state->src_w >> 16, state->src_h >> 16,
				state->crtc_x, state->crtc_y,
				state->crtc_w, state->crtc_h,
				flush_sbuf != 0);

		stage_idx = zpos_cnt[pstate->stage]++;
		stage_cfg->stage[pstate->stage][stage_idx] =
					sde_plane_pipe(plane);
		stage_cfg->multirect_index[pstate->stage][stage_idx] =
					pstate->multirect_index;

		SDE_EVT32(DRMID(crtc), DRMID(plane), stage_idx,
			sde_plane_pipe(plane) - SSPP_VIG0, pstate->stage,
			pstate->multirect_index, pstate->multirect_mode,
			format->base.pixel_format, fb ? fb->modifier[0] : 0);

		/* blend config update */
		for (lm_idx = 0; lm_idx < sde_crtc->num_mixers; lm_idx++) {
			_sde_crtc_setup_blend_cfg(mixer + lm_idx, pstate,
								format);
			mixer[lm_idx].flush_mask |= flush_mask;

			if (bg_alpha_enable && !format->alpha_enable)
				mixer[lm_idx].mixer_op_mode = 0;
			else
				mixer[lm_idx].mixer_op_mode |=
						1 << pstate->stage;
		}
	}

	if (lm && lm->ops.setup_dim_layer) {
		cstate = to_sde_crtc_state(crtc->state);
		for (i = 0; i < cstate->num_dim_layers; i++)
			_sde_crtc_setup_dim_layer_cfg(crtc, sde_crtc,
					mixer, &cstate->dim_layer[i]);
		if (cstate->fingerprint_dim_layer)
			{
			_sde_crtc_setup_dim_layer_cfg(crtc, sde_crtc,
					mixer, cstate->fingerprint_dim_layer);
			}
	}

	_sde_crtc_program_lm_output_roi(crtc);
}

static void _sde_crtc_swap_mixers_for_right_partial_update(
		struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	struct drm_encoder *drm_enc;
	bool is_right_only;
	bool encoder_in_dsc_merge = false;

	if (!crtc || !crtc->state)
		return;

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(crtc->state);

	if (sde_crtc->num_mixers != CRTC_DUAL_MIXERS)
		return;

	drm_for_each_encoder(drm_enc, crtc->dev) {
		if (drm_enc->crtc == crtc &&
				sde_encoder_is_dsc_merge(drm_enc)) {
			encoder_in_dsc_merge = true;
			break;
		}
	}

	/**
	 * For right-only partial update with DSC merge, we swap LM0 & LM1.
	 * This is due to two reasons:
	 * - On 8996, there is a DSC HW requirement that in DSC Merge Mode,
	 *   the left DSC must be used, right DSC cannot be used alone.
	 *   For right-only partial update, this means swap layer mixers to map
	 *   Left LM to Right INTF. On later HW this was relaxed.
	 * - In DSC Merge mode, the physical encoder has already registered
	 *   PP0 as the master, to switch to right-only we would have to
	 *   reprogram to be driven by PP1 instead.
	 * To support both cases, we prefer to support the mixer swap solution.
	 */
	if (!encoder_in_dsc_merge)
		return;

	is_right_only = sde_kms_rect_is_null(&cstate->lm_roi[0]) &&
			!sde_kms_rect_is_null(&cstate->lm_roi[1]);

	if (is_right_only && !sde_crtc->mixers_swapped) {
		/* right-only update swap mixers */
		swap(sde_crtc->mixers[0], sde_crtc->mixers[1]);
		sde_crtc->mixers_swapped = true;
	} else if (!is_right_only && sde_crtc->mixers_swapped) {
		/* left-only or full update, swap back */
		swap(sde_crtc->mixers[0], sde_crtc->mixers[1]);
		sde_crtc->mixers_swapped = false;
	}

	SDE_DEBUG("%s: right_only %d swapped %d, mix0->lm%d, mix1->lm%d\n",
			sde_crtc->name, is_right_only, sde_crtc->mixers_swapped,
			sde_crtc->mixers[0].hw_lm->idx - LM_0,
			sde_crtc->mixers[1].hw_lm->idx - LM_0);
	SDE_EVT32(DRMID(crtc), is_right_only, sde_crtc->mixers_swapped,
			sde_crtc->mixers[0].hw_lm->idx - LM_0,
			sde_crtc->mixers[1].hw_lm->idx - LM_0);
}

/**
 * _sde_crtc_blend_setup - configure crtc mixers
 * @crtc: Pointer to drm crtc structure
 * @old_state: Pointer to old crtc state
 * @add_planes: Whether or not to add planes to mixers
 */
static void _sde_crtc_blend_setup(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state, bool add_planes)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *sde_crtc_state;
	struct sde_crtc_mixer *mixer;
	struct sde_hw_ctl *ctl;
	struct sde_hw_mixer *lm;

	int i;

	if (!crtc)
		return;

	sde_crtc = to_sde_crtc(crtc);
	sde_crtc_state = to_sde_crtc_state(crtc->state);
	mixer = sde_crtc->mixers;

	SDE_DEBUG("%s\n", sde_crtc->name);

	if (sde_crtc->num_mixers > CRTC_DUAL_MIXERS) {
		SDE_ERROR("invalid number mixers: %d\n", sde_crtc->num_mixers);
		return;
	}

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		if (!mixer[i].hw_lm || !mixer[i].hw_ctl) {
			SDE_ERROR("invalid lm or ctl assigned to mixer\n");
			return;
		}
		mixer[i].mixer_op_mode = 0;
		mixer[i].flush_mask = 0;
		if (mixer[i].hw_ctl->ops.clear_all_blendstages)
			mixer[i].hw_ctl->ops.clear_all_blendstages(
					mixer[i].hw_ctl);

		/* clear dim_layer settings */
		lm = mixer[i].hw_lm;
		if (lm->ops.clear_dim_layer)
			lm->ops.clear_dim_layer(lm);
	}

	_sde_crtc_swap_mixers_for_right_partial_update(crtc);

	/* initialize stage cfg */
	memset(&sde_crtc->stage_cfg, 0, sizeof(struct sde_hw_stage_cfg));

	if (add_planes)
		_sde_crtc_blend_setup_mixer(crtc, old_state, sde_crtc, mixer);

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		const struct sde_rect *lm_roi = &sde_crtc_state->lm_roi[i];

		ctl = mixer[i].hw_ctl;
		lm = mixer[i].hw_lm;

		if (sde_kms_rect_is_null(lm_roi)) {
			SDE_DEBUG(
				"%s: lm%d leave ctl%d mask 0 since null roi\n",
					sde_crtc->name, lm->idx - LM_0,
					ctl->idx - CTL_0);
			continue;
		}

		lm->ops.setup_alpha_out(lm, mixer[i].mixer_op_mode);

		mixer[i].pipe_mask = mixer[i].flush_mask;
		mixer[i].flush_mask |= ctl->ops.get_bitmask_mixer(ctl,
			mixer[i].hw_lm->idx);

		/* stage config flush mask */
		ctl->ops.update_pending_flush(ctl, mixer[i].flush_mask);

		SDE_DEBUG("lm %d, op_mode 0x%X, ctl %d, flush mask 0x%x\n",
			mixer[i].hw_lm->idx - LM_0,
			mixer[i].mixer_op_mode,
			ctl->idx - CTL_0,
			mixer[i].flush_mask);

		ctl->ops.setup_blendstage(ctl, mixer[i].hw_lm->idx,
			&sde_crtc->stage_cfg);
	}

	_sde_crtc_program_lm_output_roi(crtc);
}

static int _sde_crtc_find_plane_fb_modes(struct drm_crtc_state *state,
		uint32_t *fb_ns,
		uint32_t *fb_sec,
		uint32_t *fb_sec_dir)
{
	struct drm_plane *plane;
	const struct drm_plane_state *pstate;
	struct sde_plane_state *sde_pstate;
	uint32_t mode = 0;
	int rc;

	if (!state) {
		SDE_ERROR("invalid state\n");
		return -EINVAL;
	}

	*fb_ns = 0;
	*fb_sec = 0;
	*fb_sec_dir = 0;
	drm_atomic_crtc_state_for_each_plane_state(plane, pstate, state) {
		if (IS_ERR_OR_NULL(pstate)) {
			rc = PTR_ERR(pstate);
			SDE_ERROR("crtc%d failed to get plane%d state%d\n",
					state->crtc->base.id,
					plane->base.id, rc);
			return rc;
		}
		sde_pstate = to_sde_plane_state(pstate);
		mode = sde_plane_get_property(sde_pstate,
				PLANE_PROP_FB_TRANSLATION_MODE);
		switch (mode) {
		case SDE_DRM_FB_NON_SEC:
			(*fb_ns)++;
			break;
		case SDE_DRM_FB_SEC:
			(*fb_sec)++;
			break;
		case SDE_DRM_FB_SEC_DIR_TRANS:
			(*fb_sec_dir)++;
			break;
		default:
			SDE_ERROR("Error: Plane[%d], fb_trans_mode:%d",
					plane->base.id, mode);
			return -EINVAL;
		}
	}
	return 0;
}

/**
 * sde_crtc_get_secure_transition_ops - determines the operations that
 * need to be performed before transitioning to secure state
 * This function should be called after swapping the new state
 * @crtc: Pointer to drm crtc structure
 * Returns the bitmask of operations need to be performed, -Error in
 * case of error cases
 */
int sde_crtc_get_secure_transition_ops(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state,
		bool old_valid_fb)
{
	struct drm_plane *plane;
	struct drm_encoder *encoder;
	struct sde_crtc *sde_crtc;
	struct sde_kms *sde_kms;
	struct sde_mdss_cfg *catalog;
	struct sde_kms_smmu_state_data *smmu_state;
	uint32_t translation_mode = 0, secure_level;
	int ops  = 0;
	bool post_commit = false;

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid crtc\n");
		return -EINVAL;
	}

	sde_kms = _sde_crtc_get_kms(crtc);
	if (!sde_kms)
		return -EINVAL;

	smmu_state = &sde_kms->smmu_state;
	sde_crtc = to_sde_crtc(crtc);
	secure_level = sde_crtc_get_secure_level(crtc, crtc->state);
	catalog = sde_kms->catalog;

	SDE_DEBUG("crtc%d, secure_level%d old_valid_fb%d\n",
			crtc->base.id, secure_level, old_valid_fb);

	SDE_EVT32_VERBOSE(DRMID(crtc), secure_level, smmu_state->state,
			old_valid_fb, SDE_EVTLOG_FUNC_ENTRY);
	/**
	 * SMMU operations need to be delayed in case of
	 * video mode panels when switching back to non_secure
	 * mode
	 */
	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		post_commit |= sde_encoder_check_mode(encoder,
						MSM_DISPLAY_CAP_VID_MODE);
	}

	drm_atomic_crtc_for_each_plane(plane, crtc) {
		if (!plane->state)
			continue;

		translation_mode = sde_plane_get_property(
				to_sde_plane_state(plane->state),
				PLANE_PROP_FB_TRANSLATION_MODE);
		if (translation_mode > SDE_DRM_FB_SEC_DIR_TRANS) {
			SDE_ERROR("crtc%d, invalid translation_mode%d\n",
					crtc->base.id, translation_mode);
			return -EINVAL;
		}

		/**
		 * we can break if we find sec_fir or non_sec_dir
		 * plane
		 */
		if (translation_mode == SDE_DRM_FB_SEC_DIR_TRANS)
			break;
	}

	mutex_lock(&sde_kms->secure_transition_lock);

	switch (translation_mode) {
	case SDE_DRM_FB_SEC_DIR_TRANS:
		/* secure display usecase */
		if ((smmu_state->state == ATTACHED) &&
				(secure_level == SDE_DRM_SEC_ONLY)) {
			smmu_state->state = DETACH_ALL_REQ;
			smmu_state->transition_type = PRE_COMMIT;
			ops |= SDE_KMS_OPS_SECURE_STATE_CHANGE;
			if (old_valid_fb) {
				ops |= (SDE_KMS_OPS_WAIT_FOR_TX_DONE  |
					SDE_KMS_OPS_CLEANUP_PLANE_FB);
			}
			if (catalog->sui_misr_supported &&
					sde_crtc->enable_sui_enhancement)
				smmu_state->sui_misr_state =
						SUI_MISR_ENABLE_REQ;
		/* secure camera usecase */
		} else if (smmu_state->state == ATTACHED) {
			smmu_state->state = DETACH_SEC_REQ;
			smmu_state->transition_type = PRE_COMMIT;
			ops |= SDE_KMS_OPS_SECURE_STATE_CHANGE;
		}
		break;

	case SDE_DRM_FB_SEC:
	case SDE_DRM_FB_NON_SEC:
		if ((smmu_state->state == DETACHED_SEC) ||
			(smmu_state->state == DETACH_SEC_REQ)) {
			smmu_state->state = ATTACH_SEC_REQ;
			smmu_state->transition_type = post_commit ?
				POST_COMMIT : PRE_COMMIT;
			ops |= SDE_KMS_OPS_SECURE_STATE_CHANGE;
			if (old_valid_fb)
				ops |= SDE_KMS_OPS_WAIT_FOR_TX_DONE;
		} else if ((smmu_state->state == DETACHED) ||
				(smmu_state->state == DETACH_ALL_REQ)) {
			smmu_state->state = ATTACH_ALL_REQ;
			smmu_state->transition_type = post_commit ?
				POST_COMMIT : PRE_COMMIT;
			ops |= SDE_KMS_OPS_SECURE_STATE_CHANGE;
			if (old_valid_fb)
				ops |= (SDE_KMS_OPS_WAIT_FOR_TX_DONE |
				 SDE_KMS_OPS_CLEANUP_PLANE_FB);
			if (catalog->sui_misr_supported &&
					sde_crtc->enable_sui_enhancement)
				smmu_state->sui_misr_state =
						SUI_MISR_DISABLE_REQ;
		}
		break;

	default:
		SDE_ERROR("invalid plane fb_mode:%d\n", translation_mode);
		ops = -EINVAL;
	}

	SDE_DEBUG("SMMU State:%d, type:%d ops:%x\n", smmu_state->state,
			smmu_state->transition_type, ops);
	/* log only during actual transition times */
	if (ops)
		SDE_EVT32(DRMID(crtc), secure_level, translation_mode,
				smmu_state->state, smmu_state->transition_type,
				ops, old_valid_fb, SDE_EVTLOG_FUNC_EXIT);

	mutex_unlock(&sde_kms->secure_transition_lock);

	return ops;
}

/**
 * _sde_crtc_setup_scaler3_lut - Set up scaler lut
 * LUTs are configured only once during boot
 * @sde_crtc: Pointer to sde crtc
 * @cstate: Pointer to sde crtc state
 */
static int _sde_crtc_set_dest_scaler_lut(struct sde_crtc *sde_crtc,
		struct sde_crtc_state *cstate, uint32_t lut_idx)
{
	struct sde_hw_scaler3_lut_cfg *cfg;
	u32 *lut_data = NULL;
	size_t len = 0;
	int ret = 0;

	if (!sde_crtc || !cstate) {
		SDE_ERROR("invalid args\n");
		return -EINVAL;
	}

	lut_data = msm_property_get_blob(&sde_crtc->property_info,
			&cstate->property_state, &len, lut_idx);
	if (!lut_data || !len) {
		SDE_DEBUG("%s: lut(%d): cleared: %pK, %zu\n", sde_crtc->name,
				lut_idx, lut_data, len);
		lut_data = NULL;
		len = 0;
	}

	cfg = &cstate->scl3_lut_cfg;

	switch (lut_idx) {
	case CRTC_PROP_DEST_SCALER_LUT_ED:
		cfg->dir_lut = lut_data;
		cfg->dir_len = len;
		break;
	case CRTC_PROP_DEST_SCALER_LUT_CIR:
		cfg->cir_lut = lut_data;
		cfg->cir_len = len;
		break;
	case CRTC_PROP_DEST_SCALER_LUT_SEP:
		cfg->sep_lut = lut_data;
		cfg->sep_len = len;
		break;
	default:
		ret = -EINVAL;
		SDE_ERROR("%s:invalid LUT idx(%d)\n", sde_crtc->name, lut_idx);
		SDE_EVT32(DRMID(&sde_crtc->base), lut_idx, SDE_EVTLOG_ERROR);
		break;
	}

	cfg->is_configured = cfg->dir_lut && cfg->cir_lut && cfg->sep_lut;

	SDE_EVT32_VERBOSE(DRMID(&sde_crtc->base), ret, lut_idx, len,
			cfg->is_configured);
	return ret;
}

void sde_crtc_timeline_status(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	sde_fence_timeline_status(&sde_crtc->output_fence, &crtc->base);
}

static int _sde_validate_hw_resources(struct sde_crtc *sde_crtc)
{
	int i;

	/**
	 * Check if sufficient hw resources are
	 * available as per target caps & topology
	 */
	if (!sde_crtc) {
		SDE_ERROR("invalid argument\n");
		return -EINVAL;
	}

	if (!sde_crtc->num_mixers ||
		sde_crtc->num_mixers > CRTC_DUAL_MIXERS) {
		SDE_ERROR("%s: invalid number mixers: %d\n",
			sde_crtc->name, sde_crtc->num_mixers);
		SDE_EVT32(DRMID(&sde_crtc->base), sde_crtc->num_mixers,
			SDE_EVTLOG_ERROR);
		return -EINVAL;
	}

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		if (!sde_crtc->mixers[i].hw_lm || !sde_crtc->mixers[i].hw_ctl
			|| !sde_crtc->mixers[i].hw_ds) {
			SDE_ERROR("%s:insufficient resources for mixer(%d)\n",
				sde_crtc->name, i);
			SDE_EVT32(DRMID(&sde_crtc->base), sde_crtc->num_mixers,
				i, sde_crtc->mixers[i].hw_lm,
				sde_crtc->mixers[i].hw_ctl,
				sde_crtc->mixers[i].hw_ds, SDE_EVTLOG_ERROR);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * _sde_crtc_dest_scaler_setup - Set up dest scaler block
 * @crtc: Pointer to drm crtc
 */
static void _sde_crtc_dest_scaler_setup(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	struct sde_hw_mixer *hw_lm;
	struct sde_hw_ctl *hw_ctl;
	struct sde_hw_ds *hw_ds;
	struct sde_hw_ds_cfg *cfg;
	struct sde_kms *kms;
	u32 flush_mask = 0, op_mode = 0;
	u32 lm_idx = 0, num_mixers = 0;
	int i, count = 0;
	bool ds_dirty = false;

	if (!crtc)
		return;

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(crtc->state);
	kms = _sde_crtc_get_kms(crtc);
	num_mixers = sde_crtc->num_mixers;
	count = cstate->num_ds;

	SDE_DEBUG("crtc%d\n", crtc->base.id);
	SDE_EVT32(DRMID(crtc), num_mixers, count, cstate->ds_dirty,
		sde_crtc->ds_reconfig, cstate->num_ds_enabled);

	/**
	 * destination scaler configuration will be done either
	 * or on set property or on power collapse (idle/suspend)
	 */
	ds_dirty = (cstate->ds_dirty || sde_crtc->ds_reconfig);
	if (sde_crtc->ds_reconfig) {
		SDE_DEBUG("reconfigure dest scaler block\n");
		sde_crtc->ds_reconfig = false;
	}

	if (!ds_dirty) {
		SDE_DEBUG("no change in settings, skip commit\n");
	} else if (!kms || !kms->catalog) {
		SDE_ERROR("crtc%d:invalid parameters\n", crtc->base.id);
	} else if (!kms->catalog->mdp[0].has_dest_scaler) {
		SDE_DEBUG("dest scaler feature not supported\n");
	} else if (_sde_validate_hw_resources(sde_crtc)) {
		//do nothing
	} else if (!cstate->scl3_lut_cfg.is_configured) {
		SDE_ERROR("crtc%d:no LUT data available\n", crtc->base.id);
	} else {
		for (i = 0; i < count; i++) {
			cfg = &cstate->ds_cfg[i];

			if (!cfg->flags)
				continue;

			lm_idx = cfg->idx;
			hw_lm  = sde_crtc->mixers[lm_idx].hw_lm;
			hw_ctl = sde_crtc->mixers[lm_idx].hw_ctl;
			hw_ds  = sde_crtc->mixers[lm_idx].hw_ds;

			/* Setup op mode - Dual/single */
			if (cfg->flags & SDE_DRM_DESTSCALER_ENABLE)
				op_mode |= BIT(hw_ds->idx - DS_0);

			if ((i == count-1) && hw_ds->ops.setup_opmode) {
				op_mode |= (cstate->num_ds_enabled ==
					CRTC_DUAL_MIXERS) ?
					SDE_DS_OP_MODE_DUAL : 0;
				hw_ds->ops.setup_opmode(hw_ds, op_mode);
				SDE_EVT32_VERBOSE(DRMID(crtc), op_mode);
			}

			/* Setup scaler */
			if ((cfg->flags & SDE_DRM_DESTSCALER_SCALE_UPDATE) ||
				(cfg->flags &
					SDE_DRM_DESTSCALER_ENHANCER_UPDATE)) {
				if (hw_ds->ops.setup_scaler)
					hw_ds->ops.setup_scaler(hw_ds,
						&cfg->scl3_cfg,
						&cstate->scl3_lut_cfg);

			}

			/*
			 * Dest scaler shares the flush bit of the LM in control
			 */
			if (hw_ctl->ops.get_bitmask_mixer) {
				flush_mask = hw_ctl->ops.get_bitmask_mixer(
						hw_ctl, hw_lm->idx);
				SDE_DEBUG("Set lm[%d] flush = %d",
					hw_lm->idx, flush_mask);
				hw_ctl->ops.update_pending_flush(hw_ctl,
							flush_mask);
			}
		}
	}
}

static void sde_crtc_frame_event_cb(void *data, u32 event)
{
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct sde_crtc *sde_crtc;
	struct msm_drm_private *priv;
	struct sde_crtc_frame_event *fevent;
	struct sde_crtc_frame_event_cb_data *cb_data;
	unsigned long flags;
	u32 crtc_id;

	cb_data = (struct sde_crtc_frame_event_cb_data *)data;
	if (!data) {
		SDE_ERROR("invalid parameters\n");
		return;
	}

	crtc = cb_data->crtc;
	if (!crtc || !crtc->dev || !crtc->dev->dev_private) {
		SDE_ERROR("invalid parameters\n");
		return;
	}
	sde_crtc = to_sde_crtc(crtc);
	priv = crtc->dev->dev_private;
	crtc_id = drm_crtc_index(crtc);

	SDE_DEBUG("crtc%d\n", crtc->base.id);
	SDE_EVT32_VERBOSE(DRMID(crtc), event);

	spin_lock_irqsave(&sde_crtc->spin_lock, flags);
	fevent = list_first_entry_or_null(&sde_crtc->frame_event_list,
			struct sde_crtc_frame_event, list);
	if (fevent)
		list_del_init(&fevent->list);
	spin_unlock_irqrestore(&sde_crtc->spin_lock, flags);

	if (!fevent) {
		SDE_ERROR("crtc%d event %d overflow\n",
				crtc->base.id, event);
		SDE_EVT32(DRMID(crtc), event);
		return;
	}

	fevent->event = event;
	fevent->crtc = crtc;
	fevent->connector = cb_data->connector;
	fevent->ts = ktime_get();
	kthread_queue_work(&priv->event_thread[crtc_id].worker, &fevent->work);
}

void sde_crtc_prepare_commit(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	struct drm_connector *conn;
	struct drm_encoder *encoder;

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(crtc->state);
	SDE_EVT32_VERBOSE(DRMID(crtc));

	/* identify connectors attached to this crtc */
	cstate->num_connectors = 0;

	drm_for_each_connector(conn, crtc->dev)
		if (conn->state && conn->state->crtc == crtc &&
				cstate->num_connectors < MAX_CONNECTORS) {
			encoder = conn->state->best_encoder;
			if (encoder)
				sde_encoder_register_frame_event_callback(
						encoder,
						sde_crtc_frame_event_cb,
						crtc);

			cstate->connectors[cstate->num_connectors++] = conn;
			sde_connector_prepare_fence(conn);
		}

	/* prepare main output fence */
	sde_fence_prepare(&sde_crtc->output_fence);
}

/**
 *  _sde_crtc_complete_flip - signal pending page_flip events
 * Any pending vblank events are added to the vblank_event_list
 * so that the next vblank interrupt shall signal them.
 * However PAGE_FLIP events are not handled through the vblank_event_list.
 * This API signals any pending PAGE_FLIP events requested through
 * DRM_IOCTL_MODE_PAGE_FLIP and are cached in the sde_crtc->event.
 * if file!=NULL, this is preclose potential cancel-flip path
 * @crtc: Pointer to drm crtc structure
 * @file: Pointer to drm file
 */
static void _sde_crtc_complete_flip(struct drm_crtc *crtc,
		struct drm_file *file)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = sde_crtc->event;
	if (event) {
		/* if regular vblank case (!file) or if cancel-flip from
		 * preclose on file that requested flip, then send the
		 * event:
		 */
		if (!file || (event->base.file_priv == file)) {
			sde_crtc->event = NULL;
			DRM_DEBUG_VBL("%s: send event: %pK\n",
						sde_crtc->name, event);
			SDE_EVT32_VERBOSE(DRMID(crtc));
			drm_crtc_send_vblank_event(crtc, event);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

enum sde_intf_mode sde_crtc_get_intf_mode(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;

	if (!crtc || !crtc->dev) {
		SDE_ERROR("invalid crtc\n");
		return INTF_MODE_NONE;
	}

	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		/* continue if copy encoder is encountered */
		if (sde_encoder_in_clone_mode(encoder))
			continue;

		return sde_encoder_get_intf_mode(encoder);
	}

	return INTF_MODE_NONE;
}

static void sde_crtc_vblank_cb(void *data)
{
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);

	/* keep statistics on vblank callback - with auto reset via debugfs */
	if (ktime_equal(sde_crtc->vblank_cb_time, ktime_set(0, 0)))
		sde_crtc->vblank_cb_time = ktime_get();
	else
		sde_crtc->vblank_cb_count++;

	sde_crtc->vblank_last_cb_time = ktime_get();
	sysfs_notify_dirent(sde_crtc->vsync_event_sf);

	_sde_crtc_complete_flip(crtc, NULL);
	drm_crtc_handle_vblank(crtc);
	DRM_DEBUG_VBL("crtc%d\n", crtc->base.id);
	SDE_EVT32_VERBOSE(DRMID(crtc));
}

static void _sde_crtc_retire_event(struct drm_connector *connector,
		ktime_t ts, bool is_error)
{
	if (!connector) {
		SDE_ERROR("invalid param\n");
		return;
	}

	SDE_ATRACE_BEGIN("signal_retire_fence");
	sde_connector_complete_commit(connector, ts, is_error);
	SDE_ATRACE_END("signal_retire_fence");
}

static void sde_crtc_frame_event_work(struct kthread_work *work)
{
	struct msm_drm_private *priv;
	struct sde_crtc_frame_event *fevent;
	struct drm_crtc *crtc;
	struct sde_crtc *sde_crtc;
	struct sde_kms *sde_kms;
	unsigned long flags;
	bool in_clone_mode = false;

	if (!work) {
		SDE_ERROR("invalid work handle\n");
		return;
	}

	fevent = container_of(work, struct sde_crtc_frame_event, work);
	if (!fevent->crtc || !fevent->crtc->state) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	crtc = fevent->crtc;
	sde_crtc = to_sde_crtc(crtc);

	sde_kms = _sde_crtc_get_kms(crtc);
	if (!sde_kms) {
		SDE_ERROR("invalid kms handle\n");
		return;
	}
	priv = sde_kms->dev->dev_private;
	SDE_ATRACE_BEGIN("crtc_frame_event");

	SDE_DEBUG("crtc%d event:%u ts:%lld\n", crtc->base.id, fevent->event,
			ktime_to_ns(fevent->ts));

	SDE_EVT32_VERBOSE(DRMID(crtc), fevent->event, SDE_EVTLOG_FUNC_ENTRY);

	in_clone_mode = sde_encoder_in_clone_mode(fevent->connector->encoder);

	if (!in_clone_mode && (fevent->event & (SDE_ENCODER_FRAME_EVENT_ERROR
					| SDE_ENCODER_FRAME_EVENT_PANEL_DEAD
					| SDE_ENCODER_FRAME_EVENT_DONE))) {
		if (atomic_read(&sde_crtc->frame_pending) < 1) {
			/* this should not happen */
			SDE_ERROR("crtc%d ts:%lld invalid frame_pending:%d\n",
					crtc->base.id,
					ktime_to_ns(fevent->ts),
					atomic_read(&sde_crtc->frame_pending));
			SDE_EVT32(DRMID(crtc), fevent->event,
							SDE_EVTLOG_FUNC_CASE1);
		} else if (atomic_dec_return(&sde_crtc->frame_pending) == 0) {
			/* release bandwidth and other resources */
			SDE_DEBUG("crtc%d ts:%lld last pending\n",
					crtc->base.id,
					ktime_to_ns(fevent->ts));
			SDE_EVT32(DRMID(crtc), fevent->event,
							SDE_EVTLOG_FUNC_CASE2);
			sde_core_perf_crtc_release_bw(crtc);
		} else {
			SDE_EVT32_VERBOSE(DRMID(crtc), fevent->event,
							SDE_EVTLOG_FUNC_CASE3);
		}
	}

	if (fevent->event & SDE_ENCODER_FRAME_EVENT_SIGNAL_RELEASE_FENCE) {
		SDE_ATRACE_BEGIN("signal_release_fence");
		sde_fence_signal(&sde_crtc->output_fence, fevent->ts,
				(fevent->event & SDE_ENCODER_FRAME_EVENT_ERROR)
				? SDE_FENCE_SIGNAL_ERROR : SDE_FENCE_SIGNAL);
		SDE_ATRACE_END("signal_release_fence");
	}

	if (fevent->event & SDE_ENCODER_FRAME_EVENT_SIGNAL_RETIRE_FENCE)
		/* this api should be called without spin_lock */
		_sde_crtc_retire_event(fevent->connector, fevent->ts,
				(fevent->event & SDE_ENCODER_FRAME_EVENT_ERROR)
				? SDE_FENCE_SIGNAL_ERROR : SDE_FENCE_SIGNAL);

	if (fevent->event & SDE_ENCODER_FRAME_EVENT_PANEL_DEAD)
		SDE_ERROR("crtc%d ts:%lld received panel dead event\n",
				crtc->base.id, ktime_to_ns(fevent->ts));

	spin_lock_irqsave(&sde_crtc->spin_lock, flags);
	list_add_tail(&fevent->list, &sde_crtc->frame_event_list);
	spin_unlock_irqrestore(&sde_crtc->spin_lock, flags);
	SDE_ATRACE_END("crtc_frame_event");
}

void sde_crtc_complete_commit(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state)
{
	struct sde_crtc *sde_crtc;

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	SDE_EVT32_VERBOSE(DRMID(crtc));

	sde_core_perf_crtc_update(crtc, 0, false);
	{
		struct sde_crtc_state *old_cstate;
		struct sde_crtc_state *cstate;
		struct msm_drm_notifier notifier_data;
		int blank;

		if (!old_state) {
			SDE_ERROR("failed to find old cstate");
			return;
		}
		old_cstate = to_sde_crtc_state(old_state);
		cstate = to_sde_crtc_state(crtc->state);
		if (old_cstate->fingerprint_pressed != cstate->fingerprint_pressed) {
			blank = cstate->fingerprint_pressed;
			notifier_data.data = &blank;
			notifier_data.id = MSM_DRM_PRIMARY_DISPLAY;
			pr_err("fingerprint status: %s",
			       blank ? "pressed" : "up");
			SDE_ATRACE_BEGIN("press_event_notify");
			msm_drm_notifier_call_chain(MSM_DRM_ONSCREENFINGERPRINT_EVENT,
					&notifier_data);
			SDE_ATRACE_END("press_event_notify");
		}
	}
}

/**
 * _sde_crtc_set_input_fence_timeout - update ns version of in fence timeout
 * @cstate: Pointer to sde crtc state
 */
static void _sde_crtc_set_input_fence_timeout(struct sde_crtc_state *cstate)
{
	if (!cstate) {
		SDE_ERROR("invalid cstate\n");
		return;
	}
	cstate->input_fence_timeout_ns =
		sde_crtc_get_property(cstate, CRTC_PROP_INPUT_FENCE_TIMEOUT);
	cstate->input_fence_timeout_ns *= NSEC_PER_MSEC;
}

/**
 * _sde_crtc_clear_dim_layers_v1 - clear all dim layer settings
 * @cstate:      Pointer to sde crtc state
 */
static void _sde_crtc_clear_dim_layers_v1(struct sde_crtc_state *cstate)
{
	u32 i;

	if (!cstate)
		return;

	for (i = 0; i < cstate->num_dim_layers; i++)
		memset(&cstate->dim_layer[i], 0, sizeof(cstate->dim_layer[i]));

	cstate->num_dim_layers = 0;
}

/**
 * _sde_crtc_set_dim_layer_v1 - copy dim layer settings from userspace
 * @cstate:      Pointer to sde crtc state
 * @user_ptr:    User ptr for sde_drm_dim_layer_v1 struct
 */
static void _sde_crtc_set_dim_layer_v1(struct sde_crtc_state *cstate,
		void __user *usr_ptr)
{
	struct sde_drm_dim_layer_v1 dim_layer_v1;
	struct sde_drm_dim_layer_cfg *user_cfg;
	struct sde_hw_dim_layer *dim_layer;
	u32 count, i;

	if (!cstate) {
		SDE_ERROR("invalid cstate\n");
		return;
	}
	dim_layer = cstate->dim_layer;

	if (!usr_ptr) {
		/* usr_ptr is null when setting the default property value */
		_sde_crtc_clear_dim_layers_v1(cstate);
		SDE_DEBUG("dim_layer data removed\n");
		return;
	}

	if (copy_from_user(&dim_layer_v1, usr_ptr, sizeof(dim_layer_v1))) {
		SDE_ERROR("failed to copy dim_layer data\n");
		return;
	}

	count = dim_layer_v1.num_layers;
	if (count > SDE_MAX_DIM_LAYERS) {
		SDE_ERROR("invalid number of dim_layers:%d", count);
		return;
	}

	/* populate from user space */
	cstate->num_dim_layers = count;
	for (i = 0; i < count; i++) {
		user_cfg = &dim_layer_v1.layer_cfg[i];

		dim_layer[i].flags = user_cfg->flags;
		dim_layer[i].stage = user_cfg->stage + SDE_STAGE_0;

		dim_layer[i].rect.x = user_cfg->rect.x1;
		dim_layer[i].rect.y = user_cfg->rect.y1;
		dim_layer[i].rect.w = user_cfg->rect.x2 - user_cfg->rect.x1;
		dim_layer[i].rect.h = user_cfg->rect.y2 - user_cfg->rect.y1;

		dim_layer[i].color_fill = (struct sde_mdss_color) {
				user_cfg->color_fill.color_0,
				user_cfg->color_fill.color_1,
				user_cfg->color_fill.color_2,
				user_cfg->color_fill.color_3,
		};

		SDE_DEBUG("dim_layer[%d] - flags:%d, stage:%d\n",
				i, dim_layer[i].flags, dim_layer[i].stage);
		SDE_DEBUG(" rect:{%d,%d,%d,%d}, color:{%d,%d,%d,%d}\n",
				dim_layer[i].rect.x, dim_layer[i].rect.y,
				dim_layer[i].rect.w, dim_layer[i].rect.h,
				dim_layer[i].color_fill.color_0,
				dim_layer[i].color_fill.color_1,
				dim_layer[i].color_fill.color_2,
				dim_layer[i].color_fill.color_3);
	}
}

bool sde_crtc_get_dimlayer_mode(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state)
		return false;

	cstate = to_sde_crtc_state(crtc_state);
	return !!cstate->fingerprint_dim_layer;
}
bool sde_crtc_get_fingerprint_mode(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state)
		return false;

	cstate = to_sde_crtc_state(crtc_state);
	return !!cstate->fingerprint_mode;
}

bool sde_crtc_get_fingerprint_pressed(struct drm_crtc_state *crtc_state)
{
	struct sde_crtc_state *cstate;

	if (!crtc_state)
		return false;

	cstate = to_sde_crtc_state(crtc_state);
	return cstate->fingerprint_pressed;
}

/*******************************************************************/

extern int oneplus_force_screenfp;
extern int oneplus_panel_alpha;
struct ba {
	u32 brightness;
	u32 alpha;
};

struct ba brightness_alpha_lut[] = {
	{0, 0xff},
	{1, 0xf1},
	{2, 0xec},
	{3, 0xeb},
	{4, 0xea},
	{6, 0xe8},
	{10, 0xe4},
	{20, 0xdc},
	{30, 0xd4},
	{45, 0xcc},
	{70, 0xbe},
	{100, 0xb3},
	{150, 0xa6},
	{227, 0x90},
	{300, 0x83},
	{400, 0x70},
	{500, 0x60},
	{600, 0x53},
	{800, 0x3c},
	{1023, 0x22},
	{2000, 0x83},
};
struct ba brightness_alpha_lut_dc[] = {

   {0, 0xff},
   {1, 0xE0},
   {2, 0xd5},
   {3, 0xd3},
   {4, 0xd0},
   {5, 0xce},
   {6, 0xcb},
   {8, 0xc8},
   {10, 0xc4},
   {15, 0xba},
   {20, 0xb0},
   {30, 0xa0},
   {45, 0x8b},
   {70, 0x72},
   {100, 0x5a},
   {150, 0x38},
   {227, 0xe},
   {260, 0x00},
};

static int interpolate(int x, int xa, int xb, int ya, int yb)
{
	int bf, factor, plus;
	int sub = 0;

	bf = 2 * (yb - ya) * (x - xa) / (xb - xa);
	factor = bf / 2;
	plus = bf % 2;
	if ((xa - xb) && (yb - ya))
		sub = 2 * (x - xa) * (x - xb) / (yb - ya) / (xa - xb);

	return ya + factor + plus + sub;
}

int brightness_to_alpha(int brightness)
{
	int level = ARRAY_SIZE(brightness_alpha_lut);
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut); i++){
		if (brightness_alpha_lut[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		return brightness_alpha_lut[0].alpha;
	else if (i == level)
		return brightness_alpha_lut[level - 1].alpha;

	return interpolate(brightness,
			brightness_alpha_lut[i-1].brightness,
			brightness_alpha_lut[i].brightness,
			brightness_alpha_lut[i-1].alpha,
			brightness_alpha_lut[i].alpha);
}

int bl_to_alpha_dc(int brightness)
{
	int level = ARRAY_SIZE(brightness_alpha_lut_dc);
	int i = 0;
	int alpha;

	for (i = 0; i < ARRAY_SIZE(brightness_alpha_lut_dc); i++) {
		if (brightness_alpha_lut_dc[i].brightness >= brightness)
			break;
	}

	if (i == 0)
		alpha = brightness_alpha_lut_dc[0].alpha;
	else if (i == level)
		alpha = brightness_alpha_lut_dc[level - 1].alpha;
	else
		alpha = interpolate(brightness,
			brightness_alpha_lut_dc[i-1].brightness,
			brightness_alpha_lut_dc[i].brightness,
			brightness_alpha_lut_dc[i-1].alpha,
			brightness_alpha_lut_dc[i].alpha);
	return alpha;
}

int oneplus_get_panel_brightness_to_alpha(void)
{
	struct dsi_display *display = get_main_display();

	if (!display)
		return 0;
	if (oneplus_panel_alpha)
		return oneplus_panel_alpha;
	//if(display->panel->aod_status==1)
	//	return brightness_to_alpha(2000);	
	//else if ((display->panel->aod_status==1) && (display->panel->aod_mode==3))
	//	return brightness_to_alpha(1023);		
	//else
    if (display->panel->dim_status)
		return brightness_to_alpha(display->panel->hbm_backlight);
    else
	return bl_to_alpha_dc(display->panel->hbm_backlight);

}

int oneplus_onscreenaod_hid = 0;
int oneplus_aod_hid = 0;
ssize_t oneplus_display_notify_aod_hid(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int onscreenaod_hid = 0;
	SDE_ATRACE_BEGIN("aod_hid_node");
	sscanf(buf, "%du", &onscreenaod_hid);
	oneplus_onscreenaod_hid = !!onscreenaod_hid;
	if (onscreenaod_hid == oneplus_onscreenaod_hid)
		{
		SDE_ATRACE_END("oneplus_display_notify_fp_press");
		return count;
		}

	pr_err("notify aod hid %d\n", onscreenaod_hid );
	oneplus_onscreenaod_hid = onscreenaod_hid;
	SDE_ATRACE_END("aod_hid_node");
	return count;
}

int oneplus_onscreenfp_status = 0;
ssize_t oneplus_display_notify_fp_press(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
    struct drm_device *drm_dev = display->drm_dev;
	struct drm_connector *dsi_connector = display->drm_conn;
	struct drm_mode_config *mode_config = &drm_dev->mode_config;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
    struct msm_drm_private *priv;
    int err;
    ktime_t now;
    bool need_commit = false;

	int onscreenfp_status = 0;
	sscanf(buf, "%du", &onscreenfp_status);
	onscreenfp_status = !!onscreenfp_status;
	if (onscreenfp_status == oneplus_onscreenfp_status)
		{
		SDE_ATRACE_END("oneplus_display_notify_fp_press");
		return count;
		}

	pr_err("notify fingerpress %d\n", onscreenfp_status );
	oneplus_onscreenfp_status = onscreenfp_status;

	drm_modeset_lock_all(drm_dev);
	state = drm_atomic_state_alloc(drm_dev);
	state->acquire_ctx = mode_config->acquire_ctx;
	crtc = dsi_connector->state->crtc;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	priv = drm_dev->dev_private;
	now = ktime_get();
	need_commit = (((now - priv->commit_end_time) > 20000000 ? true:false)&&display->panel->aod_status==0);

	if(need_commit){
	err = drm_atomic_commit(state);
		if (err < 0)
			drm_atomic_state_free(state);
	}
	drm_modeset_unlock_all(drm_dev);
	SDE_ATRACE_END("oneplus_display_notify_fp_press");
	return count;
}
extern bool HBM_flag;
int oneplus_dim_status = 0;
int oneplus_aod_fod = 0;
int oneplus_aod_dc = 0;

 ssize_t oneplus_display_notify_dim(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct dsi_display *display = get_main_display();
	struct drm_device *drm_dev = display->drm_dev;
	struct drm_connector *dsi_connector = display->drm_conn;
	struct drm_mode_config *mode_config = &drm_dev->mode_config;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_crtc *crtc;
	int dim_status = 0;
	int err;
	sscanf(buf, "%du", &dim_status);
	//dim_status = !!dim_status;
	pr_err("notify dim %d\n", dim_status);

	if(display->panel->aod_status==0 && (dim_status == 2)){
		pr_err("fp set it in normal status\n");
		if (dim_status == oneplus_dim_status)
			return count;
		oneplus_dim_status = dim_status;
		SDE_ATRACE_END("oneplus_display_notify_dim");
		return count;
	}else if(display->panel->aod_status==1&& dim_status == 2){
			oneplus_onscreenfp_status = 1;
		 	oneplus_aod_fod = 1;
	}else if(display->panel->aod_status==1&& dim_status == 0){
		oneplus_onscreenfp_status = 0;
		}else if(display->panel->aod_status==1&& dim_status == 5){
        oneplus_aod_dc = 1;
    }

    if (dim_status == oneplus_dim_status)
		return count;
	oneplus_dim_status = dim_status;
	drm_modeset_lock_all(drm_dev);

	state = drm_atomic_state_alloc(drm_dev);
	state->acquire_ctx = mode_config->acquire_ctx;
	crtc = dsi_connector->state->crtc;
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
    if((oneplus_dim_status !=0) && !(oneplus_dim_status == 5 && display->panel->aod_status==0)){
		err = drm_atomic_commit(state);
		if (err < 0)
			drm_atomic_state_free(state);
	}
	drm_modeset_unlock_all(drm_dev);
	SDE_ATRACE_END("oneplus_display_notify_dim");
	return count;
}
/***************************************************************************/
static int sde_crtc_config_fingerprint_dim_layer(struct drm_crtc_state *crtc_state, int stage)
{
	struct sde_crtc_state *cstate;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct sde_hw_dim_layer *fingerprint_dim_layer;
	int alpha = oneplus_get_panel_brightness_to_alpha();
	struct sde_kms *kms;
	struct dsi_display *display = get_main_display();
	kms = _sde_crtc_get_kms(crtc_state->crtc);
	if (!kms || !kms->catalog) {
		SDE_ERROR("invalid kms\n");
		return -EINVAL;
	}
	
	if(display == NULL || display->panel == NULL){
	SDE_ERROR("display panel is null\n");
	return 0;
	}
	
	if(display->panel->aod_status==1){
	if(oneplus_dim_status == 2){
	alpha = 255;
	}
	}

	cstate = to_sde_crtc_state(crtc_state);

	if (cstate->num_dim_layers == SDE_MAX_DIM_LAYERS - 1) {
		pr_err("failed to get available dim layer for custom\n");
		return -EINVAL;
	}

	if (!alpha) {
		cstate->fingerprint_dim_layer = NULL;
		return 0;
	}

	if ((stage + SDE_STAGE_0) >= kms->catalog->mixer[0].sblk->maxblendstages) {
		return -EINVAL;
	}

	fingerprint_dim_layer = &cstate->dim_layer[cstate->num_dim_layers];
	fingerprint_dim_layer->flags = SDE_DRM_DIM_LAYER_INCLUSIVE;
	fingerprint_dim_layer->stage = stage + SDE_STAGE_0;

	fingerprint_dim_layer->rect.x = 0;
	fingerprint_dim_layer->rect.y = 0;
	fingerprint_dim_layer->rect.w = mode->hdisplay;
	fingerprint_dim_layer->rect.h = mode->vdisplay;
	fingerprint_dim_layer->color_fill = (struct sde_mdss_color) {0, 0, 0, alpha};
	cstate->fingerprint_dim_layer = fingerprint_dim_layer;
	SDE_ATRACE_END("set_dim_layer");

	return 0;
}


/**
 * _sde_crtc_set_dest_scaler - copy dest scaler settings from userspace
 * @sde_crtc   :  Pointer to sde crtc
 * @cstate :  Pointer to sde crtc state
 * @usr_ptr:  User ptr for sde_drm_dest_scaler_data struct
 */
static int _sde_crtc_set_dest_scaler(struct sde_crtc *sde_crtc,
				struct sde_crtc_state *cstate,
				void __user *usr_ptr)
{
	struct sde_drm_dest_scaler_data ds_data;
	struct sde_drm_dest_scaler_cfg *ds_cfg_usr;
	struct sde_drm_scaler_v2 scaler_v2;
	void __user *scaler_v2_usr;
	int i, count;

	if (!sde_crtc || !cstate) {
		SDE_ERROR("invalid sde_crtc/state\n");
		return -EINVAL;
	}

	SDE_DEBUG("crtc %s\n", sde_crtc->name);

	if (!usr_ptr) {
		SDE_DEBUG("ds data removed\n");
		return 0;
	}

	if (copy_from_user(&ds_data, usr_ptr, sizeof(ds_data))) {
		SDE_ERROR("%s:failed to copy dest scaler data from user\n",
			sde_crtc->name);
		return -EINVAL;
	}

	count = ds_data.num_dest_scaler;
	if (!count) {
		SDE_DEBUG("no ds data available\n");
		return 0;
	}

	if (count > SDE_MAX_DS_COUNT) {
		SDE_ERROR("%s: invalid config: num_ds(%d) max(%d)\n",
			sde_crtc->name, count, SDE_MAX_DS_COUNT);
		SDE_EVT32(DRMID(&sde_crtc->base), count, SDE_EVTLOG_ERROR);
		return -EINVAL;
	}

	/* Populate from user space */
	for (i = 0; i < count; i++) {
		ds_cfg_usr = &ds_data.ds_cfg[i];

		cstate->ds_cfg[i].idx = ds_cfg_usr->index;
		cstate->ds_cfg[i].flags = ds_cfg_usr->flags;
		cstate->ds_cfg[i].lm_width = ds_cfg_usr->lm_width;
		cstate->ds_cfg[i].lm_height = ds_cfg_usr->lm_height;
		memset(&scaler_v2, 0, sizeof(scaler_v2));

		if (ds_cfg_usr->scaler_cfg) {
			scaler_v2_usr =
			(void __user *)((uintptr_t)ds_cfg_usr->scaler_cfg);

			if (copy_from_user(&scaler_v2, scaler_v2_usr,
					sizeof(scaler_v2))) {
				SDE_ERROR("%s:scaler: copy from user failed\n",
					sde_crtc->name);
				return -EINVAL;
			}
		}

		sde_set_scaler_v2(&cstate->ds_cfg[i].scl3_cfg, &scaler_v2);

		SDE_DEBUG("en(%d)dir(%d)de(%d) src(%dx%d) dst(%dx%d)\n",
			scaler_v2.enable, scaler_v2.dir_en, scaler_v2.de.enable,
			scaler_v2.src_width[0], scaler_v2.src_height[0],
			scaler_v2.dst_width, scaler_v2.dst_height);
		SDE_EVT32_VERBOSE(DRMID(&sde_crtc->base),
			scaler_v2.enable, scaler_v2.dir_en, scaler_v2.de.enable,
			scaler_v2.src_width[0], scaler_v2.src_height[0],
			scaler_v2.dst_width, scaler_v2.dst_height);

		SDE_DEBUG("ds cfg[%d]-ndx(%d) flags(%d) lm(%dx%d)\n",
			i, ds_cfg_usr->index, ds_cfg_usr->flags,
			ds_cfg_usr->lm_width, ds_cfg_usr->lm_height);
		SDE_EVT32_VERBOSE(DRMID(&sde_crtc->base), i, ds_cfg_usr->index,
			ds_cfg_usr->flags, ds_cfg_usr->lm_width,
			ds_cfg_usr->lm_height);
	}

	cstate->num_ds = count;
	cstate->ds_dirty = true;
	SDE_EVT32_VERBOSE(DRMID(&sde_crtc->base), count, cstate->ds_dirty);

	return 0;
}

/**
 * _sde_crtc_check_dest_scaler_data - validate the dest scaler data
 * @crtc  :  Pointer to drm crtc
 * @state :  Pointer to drm crtc state
 */
static int _sde_crtc_check_dest_scaler_data(struct drm_crtc *crtc,
				struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	struct drm_display_mode *mode;
	struct sde_kms *kms;
	struct sde_hw_ds *hw_ds;
	struct sde_hw_ds_cfg *cfg;
	u32 i, ret = 0, lm_idx;
	u32 num_ds_enable = 0, hdisplay = 0;
	u32 max_in_width = 0, max_out_width = 0;
	u32 prev_lm_width = 0, prev_lm_height = 0;

	if (!crtc || !state)
		return -EINVAL;

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(state);
	kms = _sde_crtc_get_kms(crtc);
	mode = &state->adjusted_mode;

	SDE_DEBUG("crtc%d\n", crtc->base.id);

	if (!cstate->ds_dirty) {
		SDE_DEBUG("dest scaler property not set, skip validation\n");
		return 0;
	}

	if (!kms || !kms->catalog) {
		SDE_ERROR("crtc%d: invalid parameters\n", crtc->base.id);
		return -EINVAL;
	}

	if (!kms->catalog->mdp[0].has_dest_scaler) {
		SDE_DEBUG("dest scaler feature not supported\n");
		return 0;
	}

	if (!sde_crtc->num_mixers) {
		SDE_DEBUG("mixers not allocated\n");
		return 0;
	}

	ret = _sde_validate_hw_resources(sde_crtc);
	if (ret)
		goto err;

	/**
	 * No of dest scalers shouldn't exceed hw ds block count and
	 * also, match the num of mixers unless it is partial update
	 * left only/right only use case - currently PU + DS is not supported
	 */
	if (cstate->num_ds > kms->catalog->ds_count ||
		((cstate->num_ds != sde_crtc->num_mixers) &&
		!(cstate->ds_cfg[0].flags & SDE_DRM_DESTSCALER_PU_ENABLE))) {
		SDE_ERROR("crtc%d: num_ds(%d), hw_ds_cnt(%d) flags(%d)\n",
			crtc->base.id, cstate->num_ds, kms->catalog->ds_count,
			cstate->ds_cfg[0].flags);
		ret = -EINVAL;
		goto err;
	}

	/**
	 * Check if DS needs to be enabled or disabled
	 * In case of enable, validate the data
	 */
	if (!(cstate->ds_cfg[0].flags & SDE_DRM_DESTSCALER_ENABLE)) {
		SDE_DEBUG("disable dest scaler, num(%d) flags(%d)\n",
			cstate->num_ds, cstate->ds_cfg[0].flags);
		goto disable;
	}

	/* Display resolution */
	hdisplay = mode->hdisplay/sde_crtc->num_mixers;

	/* Validate the DS data */
	for (i = 0; i < cstate->num_ds; i++) {
		cfg = &cstate->ds_cfg[i];
		lm_idx = cfg->idx;

		/**
		 * Validate against topology
		 * No of dest scalers should match the num of mixers
		 * unless it is partial update left only/right only use case
		 */
		if (lm_idx >= sde_crtc->num_mixers || (i != lm_idx &&
			!(cfg->flags & SDE_DRM_DESTSCALER_PU_ENABLE))) {
			SDE_ERROR("crtc%d: ds_cfg id(%d):idx(%d), flags(%d)\n",
				crtc->base.id, i, lm_idx, cfg->flags);
			SDE_EVT32(DRMID(crtc), i, lm_idx, cfg->flags,
				SDE_EVTLOG_ERROR);
			ret = -EINVAL;
			goto err;
		}

		hw_ds = sde_crtc->mixers[lm_idx].hw_ds;

		if (!max_in_width && !max_out_width) {
			max_in_width = hw_ds->scl->top->maxinputwidth;
			max_out_width = hw_ds->scl->top->maxoutputwidth;

			if (cstate->num_ds == CRTC_DUAL_MIXERS)
				max_in_width -= SDE_DS_OVERFETCH_SIZE;

			SDE_DEBUG("max DS width [%d,%d] for num_ds = %d\n",
				max_in_width, max_out_width, cstate->num_ds);
		}

		/* Check LM width and height */
		if (cfg->lm_width > hdisplay || cfg->lm_height > mode->vdisplay
			|| !cfg->lm_width || !cfg->lm_height) {
			SDE_ERROR("crtc%d: lm size[%d,%d] display [%d,%d]\n",
				crtc->base.id, cfg->lm_width, cfg->lm_height,
				hdisplay, mode->vdisplay);
			SDE_EVT32(DRMID(crtc),  cfg->lm_width, cfg->lm_height,
				hdisplay, mode->vdisplay, SDE_EVTLOG_ERROR);
			ret = -E2BIG;
			goto err;
		}

		if (!prev_lm_width && !prev_lm_height) {
			prev_lm_width = cfg->lm_width;
			prev_lm_height = cfg->lm_height;
		} else {
			if (cfg->lm_width != prev_lm_width ||
				cfg->lm_height != prev_lm_height) {
				SDE_ERROR("crtc%d:lm left[%d,%d]right[%d %d]\n",
					crtc->base.id, cfg->lm_width,
					cfg->lm_height, prev_lm_width,
					prev_lm_height);
				SDE_EVT32(DRMID(crtc), cfg->lm_width,
					cfg->lm_height, prev_lm_width,
					prev_lm_height, SDE_EVTLOG_ERROR);
				ret = -EINVAL;
				goto err;
			}
		}

		/* Check scaler data */
		if (cfg->flags & SDE_DRM_DESTSCALER_SCALE_UPDATE ||
			cfg->flags & SDE_DRM_DESTSCALER_ENHANCER_UPDATE) {

			/**
			 * Scaler src and dst width shouldn't exceed the maximum
			 * width limitation. Also, if there is no partial update
			 * dst width and height must match display resolution.
			 */
			if (cfg->scl3_cfg.src_width[0] > max_in_width ||
				cfg->scl3_cfg.dst_width > max_out_width ||
				!cfg->scl3_cfg.src_width[0] ||
				!cfg->scl3_cfg.dst_width ||
				(!(cfg->flags & SDE_DRM_DESTSCALER_PU_ENABLE)
				 && (cfg->scl3_cfg.dst_width != hdisplay ||
				 cfg->scl3_cfg.dst_height != mode->vdisplay))) {
				SDE_ERROR("crtc%d: ", crtc->base.id);
				SDE_ERROR("src_w(%d) dst(%dx%d) display(%dx%d)",
					cfg->scl3_cfg.src_width[0],
					cfg->scl3_cfg.dst_width,
					cfg->scl3_cfg.dst_height,
					hdisplay, mode->vdisplay);
				SDE_ERROR("num_mixers(%d) flags(%d) ds-%d:\n",
					sde_crtc->num_mixers, cfg->flags,
					hw_ds->idx - DS_0);
				SDE_ERROR("scale_en = %d, DE_en =%d\n",
					cfg->scl3_cfg.enable,
					cfg->scl3_cfg.de.enable);

				SDE_EVT32(DRMID(crtc), cfg->scl3_cfg.enable,
					cfg->scl3_cfg.de.enable, cfg->flags,
					max_in_width, max_out_width,
					cfg->scl3_cfg.src_width[0],
					cfg->scl3_cfg.dst_width,
					cfg->scl3_cfg.dst_height, hdisplay,
					mode->vdisplay, sde_crtc->num_mixers,
					SDE_EVTLOG_ERROR);

				cfg->flags &=
					~SDE_DRM_DESTSCALER_SCALE_UPDATE;
				cfg->flags &=
					~SDE_DRM_DESTSCALER_ENHANCER_UPDATE;

				ret = -EINVAL;
				goto err;
			}
		}

		if (cfg->flags & SDE_DRM_DESTSCALER_ENABLE)
			num_ds_enable++;

		SDE_DEBUG("ds[%d]: flags[0x%X]\n",
			hw_ds->idx - DS_0, cfg->flags);
		SDE_EVT32_VERBOSE(DRMID(crtc), hw_ds->idx - DS_0, cfg->flags);
	}

disable:
	SDE_DEBUG("dest scaler status : %d -> %d\n",
		cstate->num_ds_enabled,	num_ds_enable);
	SDE_EVT32_VERBOSE(DRMID(crtc), cstate->num_ds_enabled, num_ds_enable,
			cstate->num_ds, cstate->ds_dirty);

	if (cstate->num_ds_enabled != num_ds_enable) {
		/* Disabling destination scaler */
		if (!num_ds_enable) {
			for (i = 0; i < cstate->num_ds; i++) {
				cfg = &cstate->ds_cfg[i];
				cfg->idx = i;
				/* Update scaler settings in disable case */
				cfg->flags = SDE_DRM_DESTSCALER_SCALE_UPDATE;
				cfg->scl3_cfg.enable = 0;
				cfg->scl3_cfg.de.enable = 0;
			}
		}
		cstate->num_ds_enabled = num_ds_enable;
		cstate->ds_dirty = true;
	} else {
		if (!cstate->num_ds_enabled)
			cstate->ds_dirty = false;
	}

	return 0;

err:
	cstate->ds_dirty = false;
	return ret;
}

/**
 * _sde_crtc_wait_for_fences - wait for incoming framebuffer sync fences
 * @crtc: Pointer to CRTC object
 */
static void _sde_crtc_wait_for_fences(struct drm_crtc *crtc)
{
	struct drm_plane *plane = NULL;
	uint32_t wait_ms = 1;
	ktime_t kt_end, kt_wait;
	int rc = 0;

	SDE_DEBUG("\n");

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid crtc/state %pK\n", crtc);
		return;
	}

	/* use monotonic timer to limit total fence wait time */
	kt_end = ktime_add_ns(ktime_get(),
		to_sde_crtc_state(crtc->state)->input_fence_timeout_ns);

	/*
	 * Wait for fences sequentially, as all of them need to be signalled
	 * before we can proceed.
	 *
	 * Limit total wait time to INPUT_FENCE_TIMEOUT, but still call
	 * sde_plane_wait_input_fence with wait_ms == 0 after the timeout so
	 * that each plane can check its fence status and react appropriately
	 * if its fence has timed out. Call input fence wait multiple times if
	 * fence wait is interrupted due to interrupt call.
	 */
	SDE_ATRACE_BEGIN("plane_wait_input_fence");
	drm_atomic_crtc_for_each_plane(plane, crtc) {
		do {
			kt_wait = ktime_sub(kt_end, ktime_get());
			if (ktime_compare(kt_wait, ktime_set(0, 0)) >= 0)
				wait_ms = ktime_to_ms(kt_wait);
			else
				wait_ms = 0;

			rc = sde_plane_wait_input_fence(plane, wait_ms);
		} while (wait_ms && rc == -ERESTARTSYS);
	}
	SDE_ATRACE_END("plane_wait_input_fence");
}

static void _sde_crtc_setup_mixer_for_encoder(
		struct drm_crtc *crtc,
		struct drm_encoder *enc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_kms *sde_kms = _sde_crtc_get_kms(crtc);
	struct sde_rm *rm = &sde_kms->rm;
	struct sde_crtc_mixer *mixer;
	struct sde_hw_ctl *last_valid_ctl = NULL;
	int i;
	struct sde_rm_hw_iter lm_iter, ctl_iter, dspp_iter, ds_iter;

	sde_rm_init_hw_iter(&lm_iter, enc->base.id, SDE_HW_BLK_LM);
	sde_rm_init_hw_iter(&ctl_iter, enc->base.id, SDE_HW_BLK_CTL);
	sde_rm_init_hw_iter(&dspp_iter, enc->base.id, SDE_HW_BLK_DSPP);
	sde_rm_init_hw_iter(&ds_iter, enc->base.id, SDE_HW_BLK_DS);

	/* Set up all the mixers and ctls reserved by this encoder */
	for (i = sde_crtc->num_mixers; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		mixer = &sde_crtc->mixers[i];

		if (!sde_rm_get_hw(rm, &lm_iter))
			break;
		mixer->hw_lm = (struct sde_hw_mixer *)lm_iter.hw;

		/* CTL may be <= LMs, if <, multiple LMs controlled by 1 CTL */
		if (!sde_rm_get_hw(rm, &ctl_iter)) {
			SDE_DEBUG("no ctl assigned to lm %d, using previous\n",
					mixer->hw_lm->idx - LM_0);
			mixer->hw_ctl = last_valid_ctl;
		} else {
			mixer->hw_ctl = (struct sde_hw_ctl *)ctl_iter.hw;
			last_valid_ctl = mixer->hw_ctl;
		}

		/* Shouldn't happen, mixers are always >= ctls */
		if (!mixer->hw_ctl) {
			SDE_ERROR("no valid ctls found for lm %d\n",
					mixer->hw_lm->idx - LM_0);
			return;
		}

		/* Dspp may be null */
		(void) sde_rm_get_hw(rm, &dspp_iter);
		mixer->hw_dspp = (struct sde_hw_dspp *)dspp_iter.hw;

		/* DS may be null */
		(void) sde_rm_get_hw(rm, &ds_iter);
		mixer->hw_ds = (struct sde_hw_ds *)ds_iter.hw;

		mixer->encoder = enc;

		sde_crtc->num_mixers++;
		SDE_DEBUG("setup mixer %d: lm %d\n",
				i, mixer->hw_lm->idx - LM_0);
		SDE_DEBUG("setup mixer %d: ctl %d\n",
				i, mixer->hw_ctl->idx - CTL_0);
		if (mixer->hw_ds)
			SDE_DEBUG("setup mixer %d: ds %d\n",
				i, mixer->hw_ds->idx - DS_0);
	}
}

static void _sde_crtc_setup_mixers(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_encoder *enc;

	sde_crtc->num_mixers = 0;
	sde_crtc->mixers_swapped = false;
	memset(sde_crtc->mixers, 0, sizeof(sde_crtc->mixers));

	mutex_lock(&sde_crtc->crtc_lock);
	/* Check for mixers on all encoders attached to this crtc */
	list_for_each_entry(enc, &crtc->dev->mode_config.encoder_list, head) {
		if (enc->crtc != crtc)
			continue;

		/* avoid overwriting mixers info from a copy encoder */
		if (sde_encoder_in_clone_mode(enc))
			continue;

		_sde_crtc_setup_mixer_for_encoder(crtc, enc);
	}

	mutex_unlock(&sde_crtc->crtc_lock);
}

static void _sde_crtc_setup_is_ppsplit(struct drm_crtc_state *state)
{
	int i;
	struct sde_crtc_state *cstate;

	cstate = to_sde_crtc_state(state);

	cstate->is_ppsplit = false;
	for (i = 0; i < cstate->num_connectors; i++) {
		struct drm_connector *conn = cstate->connectors[i];

		if (sde_connector_get_topology_name(conn) ==
				SDE_RM_TOPOLOGY_PPSPLIT)
			cstate->is_ppsplit = true;
	}
}

static void _sde_crtc_setup_lm_bounds(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	struct drm_display_mode *adj_mode;
	u32 crtc_split_width;
	int i;

	if (!crtc || !state) {
		SDE_ERROR("invalid args\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(state);

	adj_mode = &state->adjusted_mode;
	crtc_split_width = sde_crtc_get_mixer_width(sde_crtc, cstate, adj_mode);

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		cstate->lm_bounds[i].x = crtc_split_width * i;
		cstate->lm_bounds[i].y = 0;
		cstate->lm_bounds[i].w = crtc_split_width;
		cstate->lm_bounds[i].h =
			sde_crtc_get_mixer_height(sde_crtc, cstate, adj_mode);
		memcpy(&cstate->lm_roi[i], &cstate->lm_bounds[i],
				sizeof(cstate->lm_roi[i]));
		SDE_EVT32_VERBOSE(DRMID(crtc), i,
				cstate->lm_bounds[i].x, cstate->lm_bounds[i].y,
				cstate->lm_bounds[i].w, cstate->lm_bounds[i].h);
		SDE_DEBUG("%s: lm%d bnd&roi (%d,%d,%d,%d)\n", sde_crtc->name, i,
				cstate->lm_roi[i].x, cstate->lm_roi[i].y,
				cstate->lm_roi[i].w, cstate->lm_roi[i].h);
	}

	drm_mode_debug_printmodeline(adj_mode);
}

static void sde_crtc_atomic_begin(struct drm_crtc *crtc,
		struct drm_crtc_state *old_state)
{
	struct sde_crtc *sde_crtc;
	struct drm_encoder *encoder;
	struct drm_device *dev;
	unsigned long flags;
	struct sde_kms *sde_kms;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	if (!crtc->state->enable) {
		SDE_DEBUG("crtc%d -> enable %d, skip atomic_begin\n",
				crtc->base.id, crtc->state->enable);
		return;
	}

	if (!sde_kms_power_resource_is_enabled(crtc->dev)) {
		SDE_ERROR("power resource is not enabled\n");
		return;
	}

	sde_kms = _sde_crtc_get_kms(crtc);
	if (!sde_kms)
		return;

	SDE_DEBUG("crtc%d\n", crtc->base.id);

	sde_crtc = to_sde_crtc(crtc);
	dev = crtc->dev;

	if (!sde_crtc->num_mixers) {
		_sde_crtc_setup_mixers(crtc);
		_sde_crtc_setup_is_ppsplit(crtc->state);
		_sde_crtc_setup_lm_bounds(crtc, crtc->state);
	}

	if (sde_crtc->event) {
		WARN_ON(sde_crtc->event);
	} else {
		spin_lock_irqsave(&dev->event_lock, flags);
		sde_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;

		/* encoder will trigger pending mask now */
		sde_encoder_trigger_kickoff_pending(encoder);
	}

	/*
	 * If no mixers have been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_mixers))
		return;

	if (_sde_crtc_get_ctlstart_timeout(crtc)) {
		_sde_crtc_blend_setup(crtc, old_state, false);
		SDE_ERROR("border fill only commit after ctlstart timeout\n");
	} else {
		_sde_crtc_blend_setup(crtc, old_state, true);
	}

	_sde_crtc_dest_scaler_setup(crtc);

	/* cancel the idle notify delayed work */
	if (sde_encoder_check_mode(sde_crtc->mixers[0].encoder,
					MSM_DISPLAY_CAP_VID_MODE) &&
		kthread_cancel_delayed_work_sync(&sde_crtc->idle_notify_work))
		SDE_DEBUG("idle notify work cancelled\n");

	/*
	 * Since CP properties use AXI buffer to program the
	 * HW, check if context bank is in attached state,
	 * apply color processing properties only if
	 * smmu state is attached,
	 */
	if (!sde_kms_is_secure_session_inprogress(sde_kms) && sde_crtc->enabled)
		sde_cp_crtc_apply_properties(crtc);

	/*
	 * PP_DONE irq is only used by command mode for now.
	 * It is better to request pending before FLUSH and START trigger
	 * to make sure no pp_done irq missed.
	 * This is safe because no pp_done will happen before SW trigger
	 * in command mode.
	 */
}

static void sde_crtc_atomic_flush(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct drm_encoder *encoder;
	struct sde_crtc *sde_crtc;
	struct drm_device *dev;
	struct drm_plane *plane;
	struct msm_drm_private *priv;
	struct msm_drm_thread *event_thread;
	unsigned long flags;
	struct sde_crtc_state *cstate;
	struct sde_kms *sde_kms;
	int idle_time = 0;

	if (!crtc || !crtc->dev || !crtc->dev->dev_private) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	if (!crtc->state->enable) {
		SDE_DEBUG("crtc%d -> enable %d, skip atomic_flush\n",
				crtc->base.id, crtc->state->enable);
		return;
	}

	if (!sde_kms_power_resource_is_enabled(crtc->dev)) {
		SDE_ERROR("power resource is not enabled\n");
		return;
	}

	sde_kms = _sde_crtc_get_kms(crtc);
	if (!sde_kms) {
		SDE_ERROR("invalid kms\n");
		return;
	}

	SDE_DEBUG("crtc%d\n", crtc->base.id);

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(crtc->state);
	dev = crtc->dev;
	priv = dev->dev_private;

	if (crtc->index >= ARRAY_SIZE(priv->event_thread)) {
		SDE_ERROR("invalid crtc index[%d]\n", crtc->index);
		return;
	}

	event_thread = &priv->event_thread[crtc->index];
	idle_time = sde_crtc_get_property(cstate, CRTC_PROP_IDLE_TIMEOUT);

	if (sde_crtc->event) {
		SDE_DEBUG("already received sde_crtc->event\n");
	} else {
		spin_lock_irqsave(&dev->event_lock, flags);
		sde_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	/*
	 * If no mixers has been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_mixers))
		return;

	_sde_crtc_blend_setup_mixer(crtc, sde_crtc, mixer);

	for (i = 0; i < sde_crtc->num_mixers; i++) {
		uint32_t flush_mask = 0;

		if ((!mixer[i].hw_lm) || (!mixer[i].hw_ctl)) {
			sde_crtc->stage_cfg.border_enable[i] = true;
			continue;
		}

		ctl = mixer[i].hw_ctl;
		lm = mixer[i].hw_lm;
		memset(&alpha_out, 0, sizeof(alpha_out));

		flush_mask = blend_config_per_mixer(crtc, sde_crtc,
				mixer + i, &alpha_out);

		if (sde_crtc->stage_cfg.stage[SDE_STAGE_BASE][i] == SSPP_NONE)
			sde_crtc->stage_cfg.border_enable[i] = true;

		lm->ops.setup_alpha_out(lm, &alpha_out);

		/* get the flush mask for mixer */
		ctl->ops.get_bitmask_mixer(ctl, &flush_mask,
			mixer[i].hw_lm->idx);

		/* stage config flush mask */
		ctl->ops.update_pending_flush(ctl, flush_mask);
		SDE_DEBUG("lm %d ctl %d add mask 0x%x to pending flush\n",
				mixer->hw_lm->idx, ctl->idx, flush_mask);
	}

	/* Program ctl_paths */
	for (i = 0; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		if ((!mixer[i].hw_lm) || (!mixer[i].hw_ctl))
			continue;

		ctl = mixer[i].hw_ctl;
		lm = mixer[i].hw_lm;

		/* same stage config to all mixers */
		ctl->ops.setup_blendstage(ctl, mixer[i].hw_lm->idx,
			&sde_crtc->stage_cfg, i);
	}
}

void sde_crtc_prepare_fence(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);

	MSM_EVT(crtc->dev, crtc->base.id, 0);

	sde_fence_prepare(&sde_crtc->output_fence);
}

/* if file!=NULL, this is preclose potential cancel-flip path */
static void complete_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = sde_crtc->event;
	if (event) {
		/* if regular vblank case (!file) or if cancel-flip from
		 * preclose on file that requested flip, then send the
		 * event:
		 */
		if (!file || (event->base.file_priv == file)) {
			sde_crtc->event = NULL;
			SDE_DEBUG("%s: send event: %pK\n",
						sde_crtc->name, event);
			drm_send_vblank_event(dev, sde_crtc->drm_crtc_id,
					event);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static void sde_crtc_vblank_cb(void *data)
{
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_kms *sde_kms = get_kms(crtc);
	struct drm_device *dev = sde_kms->dev;
	unsigned pending;

	pending = atomic_xchg(&sde_crtc->pending, 0);

	if (pending & PENDING_FLIP) {
		complete_flip(crtc, NULL);
		/* free ref count paired with the atomic_flush */
		drm_crtc_vblank_put(crtc);
	}

	if (atomic_read(&sde_crtc->drm_requested_vblank)) {
		drm_handle_vblank(dev, sde_crtc->drm_crtc_id);
		DBG_IRQ("");
		MSM_EVT(crtc->dev, crtc->base.id, 0);
	}
}

void sde_crtc_complete_commit(struct drm_crtc *crtc)
{
	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	/* signal out fence at end of commit */
	sde_fence_signal(&to_sde_crtc(crtc)->output_fence, 0);
}

/**
 * _sde_crtc_trigger_kickoff - Iterate through the control paths and trigger
 *	the hw_ctl object to flush any pending flush mask, and trigger
 *	control start if the interface types require it.
 *
 *	This is currently designed to be called only once per crtc, per flush.
 *	It should be called from the encoder, through the
 *	sde_encoder_schedule_kickoff callflow, after all the encoders are ready
 *	to have CTL_START triggered.
 *
 *	It is called from the commit thread context.
 * @data: crtc pointer
 */
static void _sde_crtc_trigger_kickoff(void *data)
{
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_crtc_mixer *mixer;
	struct sde_hw_ctl *ctl;
	int i;

	if (!data) {
		SDE_ERROR("invalid argument\n");
		return;
	}

	MSM_EVT(crtc->dev, crtc->base.id, 0);

	/* Commit all pending flush masks to hardware */
	for (i = 0; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		ctl = sde_crtc->mixers[i].hw_ctl;
		if (ctl) {
			ctl->ops.trigger_flush(ctl);
			MSM_EVT(crtc->dev, crtc->base.id, ctl->idx);
		}
	}

	/* Signal start to any interface types that require it */
	for (i = 0; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		mixer = &sde_crtc->mixers[i];
		ctl = mixer->hw_ctl;
		if (ctl && sde_encoder_needs_ctl_start(mixer->encoder)) {
			ctl->ops.trigger_start(ctl);
			MSM_EVT(crtc->dev, crtc->base.id, ctl->idx);
		}
	}
}

/**
 * _sde_crtc_set_input_fence_timeout - update ns version of in fence timeout
 * @cstate: Pointer to sde crtc state
 */
static void _sde_crtc_set_input_fence_timeout(struct sde_crtc_state *cstate)
{
	if (!cstate) {
		SDE_ERROR("invalid cstate\n");
		return;
	}
	cstate->input_fence_timeout_ns =
		sde_crtc_get_property(cstate, CRTC_PROP_INPUT_FENCE_TIMEOUT);
	cstate->input_fence_timeout_ns *= NSEC_PER_MSEC;
}

/**
 * _sde_crtc_wait_for_fences - wait for incoming framebuffer sync fences
 * @crtc: Pointer to CRTC object
 */
static void _sde_crtc_wait_for_fences(struct drm_crtc *crtc)
{
	struct drm_plane *plane = NULL;
	uint32_t wait_ms = 1;
	u64 ktime_end;
	s64 ktime_wait; /* need signed 64-bit type */

	DBG("");

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid crtc/state %pK\n", crtc);
		return;
	}

	/* use monotonic timer to limit total fence wait time */
	ktime_end = ktime_get_ns() +
		to_sde_crtc_state(crtc->state)->input_fence_timeout_ns;

	/*
	 * Wait for fences sequentially, as all of them need to be signalled
	 * before we can proceed.
	 *
	 * Limit total wait time to INPUT_FENCE_TIMEOUT, but still call
	 * sde_plane_wait_input_fence with wait_ms == 0 after the timeout so
	 * that each plane can check its fence status and react appropriately
	 * if its fence has timed out.
	 */
	drm_atomic_crtc_for_each_plane(plane, crtc) {
		if (wait_ms) {
			/* determine updated wait time */
			ktime_wait = ktime_end - ktime_get_ns();
			if (ktime_wait >= 0)
				wait_ms = ktime_wait / NSEC_PER_MSEC;
			else
				wait_ms = 0;
		}
		sde_plane_wait_input_fence(plane, wait_ms);
	}
}

static void _sde_crtc_setup_mixer_for_encoder(
		struct drm_crtc *crtc,
		struct drm_encoder *enc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct sde_kms *sde_kms = get_kms(crtc);
	struct sde_rm *rm = &sde_kms->rm;
	struct sde_crtc_mixer *mixer;
	struct sde_hw_ctl *last_valid_ctl = NULL;
	int i;
	struct sde_rm_hw_iter lm_iter, ctl_iter;

	DBG("");
	sde_rm_init_hw_iter(&lm_iter, enc->base.id, SDE_HW_BLK_LM);
	sde_rm_init_hw_iter(&ctl_iter, enc->base.id, SDE_HW_BLK_CTL);

	/* Set up all the mixers and ctls reserved by this encoder */
	for (i = sde_crtc->num_mixers; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		mixer = &sde_crtc->mixers[i];

		if (!sde_rm_get_hw(rm, &lm_iter))
			break;
		mixer->hw_lm = (struct sde_hw_mixer *)lm_iter.hw;

		/* CTL may be <= LMs, if <, multiple LMs controlled by 1 CTL */
		if (!sde_rm_get_hw(rm, &ctl_iter)) {
			SDE_DEBUG("no ctl assigned to lm %d, using previous\n",
					mixer->hw_lm->idx);
			mixer->hw_ctl = last_valid_ctl;
		} else {
			mixer->hw_ctl = (struct sde_hw_ctl *)ctl_iter.hw;
			last_valid_ctl = mixer->hw_ctl;
		}

		/* Shouldn't happen, mixers are always >= ctls */
		if (!mixer->hw_ctl) {
			SDE_ERROR("no valid ctls found for lm %d\n",
					mixer->hw_lm->idx);
			return;
		}

		mixer->encoder = enc;

		sde_crtc->num_mixers++;
		SDE_DEBUG("setup mixer %d: lm %d\n", i, mixer->hw_lm->idx);
		SDE_DEBUG("setup mixer %d: ctl %d\n", i, mixer->hw_ctl->idx);
	}
}

static void _sde_crtc_setup_mixers(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_encoder *enc;

	sde_crtc->num_mixers = 0;
	memset(sde_crtc->mixers, 0, sizeof(sde_crtc->mixers));

	/* Check for mixers on all encoders attached to this crtc */
	list_for_each_entry(enc, &crtc->dev->mode_config.encoder_list, head) {
		if (enc->crtc != crtc)
			continue;

		_sde_crtc_setup_mixer_for_encoder(crtc, enc);
	}
}

static void sde_crtc_atomic_begin(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct sde_crtc *sde_crtc;
	struct drm_device *dev;
	unsigned long flags;
	u32 i;

	DBG("");

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	dev = crtc->dev;

	DBG("crtc:%d num_mixers=%d\n", sde_crtc->drm_crtc_id,
		sde_crtc->num_mixers);
	if (!sde_crtc->num_mixers)
		_sde_crtc_setup_mixers(crtc);

	if (sde_crtc->event) {
		WARN_ON(sde_crtc->event);
	} else {
		spin_lock_irqsave(&dev->event_lock, flags);
		sde_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	/* Reset flush mask from previous commit */
	for (i = 0; i < ARRAY_SIZE(sde_crtc->mixers); i++) {
		struct sde_hw_ctl *ctl = sde_crtc->mixers[i].hw_ctl;

		if (ctl)
			ctl->ops.clear_pending_flush(ctl);
	}

	/*
	 * If no mixers have been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_mixers))
		return;

	sde_crtc_blend_setup(crtc);

	/*
	 * PP_DONE irq is only used by command mode for now.
	 * It is better to request pending before FLUSH and START trigger
	 * to make sure no pp_done irq missed.
	 * This is safe because no pp_done will happen before SW trigger
	 * in command mode.
	 */
}

static void request_pending(struct drm_crtc *crtc, u32 pending)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);

	atomic_or(pending, &sde_crtc->pending);

	/* ref count the vblank event and interrupts over the atomic commit */
	if (drm_crtc_vblank_get(crtc))
		return;
}

static void sde_crtc_atomic_flush(struct drm_crtc *crtc,
		struct drm_crtc_state *old_crtc_state)
{
	struct sde_crtc *sde_crtc;
	struct drm_device *dev;
	struct drm_plane *plane;
	unsigned long flags;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	DBG("");

	sde_crtc = to_sde_crtc(crtc);

	dev = crtc->dev;

	if (sde_crtc->event) {
		SDE_DEBUG("already received sde_crtc->event\n");
	} else {
		spin_lock_irqsave(&dev->event_lock, flags);
		sde_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	/*
	 * If no mixers has been allocated in sde_crtc_atomic_check(),
	 * it means we are trying to flush a CRTC whose state is disabled:
	 * nothing else needs to be done.
	 */
	if (unlikely(!sde_crtc->num_mixers))
		return;

	/* wait for acquire fences before anything else is done */
	_sde_crtc_wait_for_fences(crtc);

	/*
	 * Final plane updates: Give each plane a chance to complete all
	 *                      required writes/flushing before crtc's "flush
	 *                      everything" call below.
	 */
	drm_atomic_crtc_for_each_plane(plane, crtc)
		sde_plane_flush(plane);

	request_pending(crtc, PENDING_FLIP);

	/* Kickoff will be scheduled by outer layer */
}

/**
 * sde_crtc_destroy_state - state destroy hook
 * @crtc: drm CRTC
 * @state: CRTC state object to release
 */
static void sde_crtc_destroy_state(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;

	if (!crtc || !state) {
		SDE_ERROR("invalid argument(s)\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	cstate = to_sde_crtc_state(state);

	DBG("");

	__drm_atomic_helper_crtc_destroy_state(crtc, state);

	/* destroy value helper */
	msm_property_destroy_state(&sde_crtc->property_info, cstate,
			cstate->property_values, cstate->property_blobs);
}

void sde_crtc_commit_kickoff(struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct drm_device *dev;

	if (!crtc) {
		SDE_ERROR("invalid argument\n");
		return;
	}
	dev = crtc->dev;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;

		/*
		 * Encoder will flush/start now, unless it has a tx pending.
		 * If so, it may delay and flush at an irq event (e.g. ppdone)
		 */
		sde_encoder_schedule_kickoff(encoder, _sde_crtc_trigger_kickoff,
				crtc);
	}
}

/**
 * sde_crtc_duplicate_state - state duplicate hook
 * @crtc: Pointer to drm crtc structure
 * @Returns: Pointer to new drm_crtc_state structure
 */
static struct drm_crtc_state *sde_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate, *old_cstate;

	if (!crtc || !crtc->state) {
		SDE_ERROR("invalid argument(s)\n");
		return NULL;
	}

	sde_crtc = to_sde_crtc(crtc);
	old_cstate = to_sde_crtc_state(crtc->state);
	cstate = msm_property_alloc_state(&sde_crtc->property_info);
	if (!cstate) {
		SDE_ERROR("failed to allocate state\n");
		return NULL;
	}

	/* duplicate value helper */
	msm_property_duplicate_state(&sde_crtc->property_info,
			old_cstate, cstate,
			cstate->property_values, cstate->property_blobs);

	/* duplicate base helper */
	__drm_atomic_helper_crtc_duplicate_state(crtc, &cstate->base);

	return &cstate->base;
}

/**
 * sde_crtc_reset - reset hook for CRTCs
 * Resets the atomic state for @crtc by freeing the state pointer (which might
 * be NULL, e.g. at driver load time) and allocating a new empty state object.
 * @crtc: Pointer to drm crtc structure
 */
static void sde_crtc_reset(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	/* remove previous state, if present */
	if (crtc->state) {
		sde_crtc_destroy_state(crtc, crtc->state);
		crtc->state = 0;
	}

	sde_crtc = to_sde_crtc(crtc);
	cstate = msm_property_alloc_state(&sde_crtc->property_info);
	if (!cstate) {
		SDE_ERROR("failed to allocate state\n");
		return;
	}

	/* reset value helper */
	msm_property_reset_state(&sde_crtc->property_info, cstate,
			cstate->property_values, cstate->property_blobs);

	_sde_crtc_set_input_fence_timeout(cstate);

	cstate->base.crtc = crtc;
	crtc->state = &cstate->base;
}

static int sde_crtc_cursor_set(struct drm_crtc *crtc,
		struct drm_file *file, uint32_t handle,
		uint32_t width, uint32_t height)
{
	return 0;
}

static int sde_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	return 0;
}

static void sde_crtc_disable(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;

	if (!crtc) {
		DRM_ERROR("invalid crtc\n");
		return;
	}
	sde_crtc = to_sde_crtc(crtc);

	DBG("");

	memset(sde_crtc->mixers, 0, sizeof(sde_crtc->mixers));
	sde_crtc->num_mixers = 0;
}

static void sde_crtc_enable(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_mixer *mixer;
	struct sde_hw_mixer *lm;
	struct drm_display_mode *mode;
	struct sde_hw_mixer_cfg cfg;
	int i;

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	DBG("");

	sde_crtc = to_sde_crtc(crtc);
	mixer = sde_crtc->mixers;

int op_dimlayer_bl_alpha = 260;
int op_dimlayer_bl_enabled = 0;
int op_dimlayer_bl_enable_real = 0;
int op_dimlayer_bl = 0;
extern int op_dimlayer_bl_enable;
extern int op_dp_enable;
extern int sde_plane_check_fingerprint_layer(const struct drm_plane_state *drm_state);
static int sde_crtc_onscreenfinger_atomic_check(struct sde_crtc_state *cstate,
		struct plane_state *pstates, int cnt)
{
	int fp_index = -1;
	int fppressed_index = -1;
    int aod_index = -1;
	int zpos = INT_MAX;
	int mode;
	int fp_mode = oneplus_onscreenfp_status;
	int dim_mode = oneplus_dim_status;
    int aod_mode = -1;
	int fppressed_index_rt = -1;
	int i;
    struct dsi_display *display = get_main_display();
	int dim_backlight = 0;

	mode = &crtc->state->adjusted_mode;

	drm_mode_debug_printmodeline(mode);

	for (i = 0; i < cnt; i++) {
		mode = sde_plane_check_fingerprint_layer(pstates[i].drm_pstate);
		if (mode == 1)
			fp_index = i;
		if (mode == 2) {
			fppressed_index = i;
			fppressed_index_rt = i;
		}
        if (mode ==3)
            aod_index = i;
	}

	if(fp_mode == 1 && dim_mode!=0) {
		display->panel->dim_status = true;
		cstate->fingerprint_pressed = true;
		return 0;
	} else {
		display->panel->dim_status = false;
		cstate->fingerprint_pressed = false;
		cstate->fingerprint_dim_layer = NULL;
		return 0;
	}
	sde_crtc->enabled = true;
	mutex_unlock(&sde_crtc->crtc_lock);

	if(aod_index <0){
		oneplus_aod_hid = 0;
		}
		
	if(fppressed_index_rt < 0){
			oneplus_aod_fod = 0;
			oneplus_aod_dc = 0;
	}


    if ((fp_index >= 0 && dim_mode!=0)||(display->panel->aod_status==1&& oneplus_aod_dc ==0)) {
	op_dimlayer_bl = 0;
    } else{
	if (op_dimlayer_bl_enable && !op_dp_enable) {
		if (display->panel->bl_config.bl_level != 0 &&
			display->panel->bl_config.bl_level < op_dimlayer_bl_alpha){
			dim_backlight = 1;
			op_dimlayer_bl = 1;
		} else{
			op_dimlayer_bl = 0;
		}
	} else{
		op_dimlayer_bl = 0;
		}
	}

	if (fp_index >= 0 || fppressed_index >= 0 || oneplus_force_screenfp || dim_backlight==1) {
	//if (fp_index >= 0 || fppressed_index >= 0 || oneplus_force_screenfp ) {
		if (fp_index >= 0 && fppressed_index >= 0) {
			if (pstates[fp_index].stage >= pstates[fppressed_index].stage) {
				SDE_ERROR("Bug!!@@@@: fp layer top of fppressed layer\n");
				return -EINVAL;
			}
		}
		if (fppressed_index >= 0) {
			if (fp_mode == 0) {
				pstates[fppressed_index].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0;
				if(oneplus_aod_fod == 1 && aod_index < 0) {
					for (i = 0; i < cnt; i++) {
						if(i!=fppressed_index ) {
							if(pstates[i].sde_pstate->property_values[PLANE_PROP_ALPHA].value == 0){
								pstates[i].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0xff;
							}
						}
					}
				}				
				fppressed_index = -1;
			} else {
				pstates[fppressed_index].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0xff;
			}
		}
	   if (fp_index >= 0) {
			if (dim_mode == 0) {
				pstates[fp_index].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0;
				fp_index = -1;
			} else {
				pstates[fp_index].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0xff;
			}
		}
        
         if (aod_index >= 0) {
			if (aod_mode == 1) {
                SDE_ATRACE_BEGIN("aod_layer_hid");
				pstates[aod_index].sde_pstate->property_values[PLANE_PROP_ALPHA].value = 0;
				aod_index = -1;
                SDE_ATRACE_END("aod_layer_hid");
			}
		}

	int stage;
};

static int pstate_cmp(const void *a, const void *b)
{
	struct plane_state *pa = (struct plane_state *)a;
	struct plane_state *pb = (struct plane_state *)b;
	int rc = 0;
	int pa_zpos, pb_zpos;

		if (fp_index >= 0) {
			if (zpos > pstates[fp_index].stage)
				zpos = pstates[fp_index].stage;
			pstates[fp_index].stage++;
		}
		for (i = 0; i < cnt; i++) {
				if (i == fp_index || i == fppressed_index || i == aod_index)
					{
					continue;
					}
				if (pstates[i].stage >= zpos) {
				//	SDE_ERROR("Warn!!: the fp layer not on top");
				pstates[i].stage++;
				}
			}
		if (zpos == INT_MAX) {
			zpos = 0;
			for (i = 0; i < cnt; i++) {
				if (pstates[i].stage > zpos)
					zpos = pstates[i].stage;
			}
			zpos++;
		}
		if (fp_index >= 0)
			cstate->fingerprint_mode = true;
		else
			cstate->fingerprint_mode = false;

		if ((fp_index >= 0 || dim_backlight > 0) && sde_crtc_config_fingerprint_dim_layer(&cstate->base, zpos)) {
			SDE_ERROR("Failed to config dim layer\n");
			return -EINVAL;
		}
		if (fppressed_index >= 0)
			cstate->fingerprint_pressed = true;
		else {
			cstate->fingerprint_pressed = false;
		}
	}else{
	cstate->fingerprint_pressed = false;
	cstate->fingerprint_mode = false;
    }
	if(fp_index < 0 && !dim_backlight){
		cstate->fingerprint_dim_layer = NULL;
	}
	if (fppressed_index < 0)
		cstate->fingerprint_pressed = false;

	return rc;
}

static int sde_crtc_atomic_check(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct sde_crtc *sde_crtc;
	struct plane_state pstates[SDE_STAGE_MAX * 2];

	struct drm_plane_state *pstate;
	struct drm_plane *plane;
	struct drm_display_mode *mode;

	int cnt = 0, rc = 0, mixer_width, i, z_pos_cur, z_pos_prev = 0;
	int z_pos = 0;
	int left_crtc_zpos_cnt[SDE_STAGE_MAX] = {0};
	int right_crtc_zpos_cnt[SDE_STAGE_MAX] = {0};

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return -EINVAL;
	}

	sde_crtc = to_sde_crtc(crtc);
	mode = &state->adjusted_mode;
	SDE_DEBUG("%s: check", sde_crtc->name);

	mixer_width = sde_crtc_mixer_width(sde_crtc, mode);

	 /* get plane state for all drm planes associated with crtc state */
	drm_atomic_crtc_state_for_each_plane(plane, state) {
		pstate = state->state->plane_states[drm_plane_index(plane)];

		/* plane might not have changed, in which case take
		 * current state:
		 */
		if (!pstate)
			pstate = plane->state;

		pstates[cnt].sde_pstate = to_sde_plane_state(pstate);
		pstates[cnt].drm_pstate = pstate;
		cnt++;

		if (CHECK_LAYER_BOUNDS(pstate->crtc_y, pstate->crtc_h,
				mode->vdisplay) ||
		    CHECK_LAYER_BOUNDS(pstate->crtc_x, pstate->crtc_w,
				mode->hdisplay)) {
			SDE_ERROR("invalid vertical/horizontal destination\n");
			SDE_ERROR("y:%d h:%d vdisp:%d x:%d w:%d hdisp:%d\n",
				pstate->crtc_y, pstate->crtc_h, mode->vdisplay,
				pstate->crtc_x, pstate->crtc_w, mode->hdisplay);
			rc = -E2BIG;
			goto end;
		}
	}

	/* sort planes based on sorted zpos property */
	sort(pstates, cnt, sizeof(pstates[0]), pstate_cmp, NULL);

	for (i = 0; i < cnt; i++) {
		z_pos_cur = sde_plane_get_property(pstates[i].sde_pstate,
			PLANE_PROP_ZPOS);
		if (z_pos_cur != z_pos_prev)
			z_pos++;
		z_pos_prev = z_pos_cur;

		if (pstates[i].drm_pstate->crtc_x < mixer_width) {
			if (left_crtc_zpos_cnt[z_pos] == 2) {
				SDE_ERROR("> 2 plane @ stage%d on left\n",
					z_pos);
				rc = -EINVAL;
				goto end;
			}
			left_crtc_zpos_cnt[z_pos]++;
		} else {
			if (right_crtc_zpos_cnt[z_pos] == 2) {
				SDE_ERROR("> 2 plane @ stage%d on right\n",
					z_pos);
				rc = -EINVAL;
				goto end;
			}
			right_crtc_zpos_cnt[z_pos]++;
		}
		pstates[i].sde_pstate->stage = z_pos;
		/*
		 * Hardware crossbar still limits the cursor layer always to be
		 * topmost layer even though mixer could support any stage for
		 * cursor.
		 * The topmost blend stage needs to be moved to catalog or dtsi
		 * as well.
		 */
		if ((sde_plane_pipe(pstates[i].drm_pstate->plane, 0)
				== SSPP_CURSOR0) ||
			(sde_plane_pipe(pstates[i].drm_pstate->plane, 0)
				== SSPP_CURSOR1))
			pstates[i].sde_pstate->stage = SDE_STAGE_6;
		SDE_DEBUG("%s: zpos %d", sde_crtc->name,
				pstates[i].sde_pstate->stage);
	}

end:
	return rc;
}

int sde_crtc_vblank(struct drm_crtc *crtc, bool en)
{
	struct sde_crtc *sde_crtc = to_sde_crtc(crtc);
	struct drm_encoder *encoder;
	struct drm_device *dev = crtc->dev;

	SDE_DEBUG("%d", en);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		/*
		 * Mark that framework requested vblank,
		 * as opposed to enabling vblank only for our internal purposes
		 * Currently this variable isn't required, but may be useful for
		 * future features
		 */
		atomic_set(&sde_crtc->drm_requested_vblank, en);
		MSM_EVT(crtc->dev, crtc->base.id, en);

		if (en)
			sde_encoder_register_vblank_callback(encoder,
					sde_crtc_vblank_cb, (void *)crtc);
		else
			sde_encoder_register_vblank_callback(encoder, NULL,
					NULL);
	}

	return 0;
}

void sde_crtc_cancel_pending_flip(struct drm_crtc *crtc, struct drm_file *file)
{
}

/**
 * sde_crtc_install_properties - install all drm properties for crtc
 * @crtc: Pointer to drm crtc structure
 */
static void sde_crtc_install_properties(struct drm_crtc *crtc)
{
	struct sde_crtc *sde_crtc;
	struct drm_device *dev;

	DBG("");

	if (!crtc) {
		SDE_ERROR("invalid crtc\n");
		return;
	}

	sde_crtc = to_sde_crtc(crtc);
	dev = crtc->dev;

	/* range properties */
	msm_property_install_range(&sde_crtc->property_info,
		"input_fence_timeout", 0x0, 0, SDE_CRTC_MAX_INPUT_FENCE_TIMEOUT,
		SDE_CRTC_INPUT_FENCE_TIMEOUT, CRTC_PROP_INPUT_FENCE_TIMEOUT);

	msm_property_install_range(&sde_crtc->property_info, "output_fence",
			0x0, 0, INR_OPEN_MAX, 0x0, CRTC_PROP_OUTPUT_FENCE);
}

/**
 * sde_crtc_atomic_set_property - atomically set a crtc drm property
 * @crtc: Pointer to drm crtc structure
 * @state: Pointer to drm crtc state structure
 * @property: Pointer to targeted drm property
 * @val: Updated property value
 * @Returns: Zero on success
 */
static int sde_crtc_atomic_set_property(struct drm_crtc *crtc,
		struct drm_crtc_state *state,
		struct drm_property *property,
		uint64_t val)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	int idx, ret = -EINVAL;

	if (!crtc || !state || !property) {
		SDE_ERROR("invalid argument(s)\n");
	} else {
		sde_crtc = to_sde_crtc(crtc);
		cstate = to_sde_crtc_state(state);
		ret = msm_property_atomic_set(&sde_crtc->property_info,
				cstate->property_values, cstate->property_blobs,
				property, val);
		if (!ret) {
			idx = msm_property_index(&sde_crtc->property_info,
					property);
			if (idx == CRTC_PROP_INPUT_FENCE_TIMEOUT)
				_sde_crtc_set_input_fence_timeout(cstate);
		}
	}

	return ret;
}

/**
 * sde_crtc_set_property - set a crtc drm property
 * @crtc: Pointer to drm crtc structure
 * @property: Pointer to targeted drm property
 * @val: Updated property value
 * @Returns: Zero on success
 */
static int sde_crtc_set_property(struct drm_crtc *crtc,
		struct drm_property *property, uint64_t val)
{
	DBG("");

	return sde_crtc_atomic_set_property(crtc, crtc->state, property, val);
}

/**
 * sde_crtc_atomic_get_property - retrieve a crtc drm property
 * @crtc: Pointer to drm crtc structure
 * @state: Pointer to drm crtc state structure
 * @property: Pointer to targeted drm property
 * @val: Pointer to variable for receiving property value
 * @Returns: Zero on success
 */
static int sde_crtc_atomic_get_property(struct drm_crtc *crtc,
		const struct drm_crtc_state *state,
		struct drm_property *property,
		uint64_t *val)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_state *cstate;
	int i, ret = -EINVAL;

	if (!crtc || !state) {
		SDE_ERROR("invalid argument(s)\n");
	} else {
		sde_crtc = to_sde_crtc(crtc);
		cstate = to_sde_crtc_state(state);
		i = msm_property_index(&sde_crtc->property_info, property);
		if (i == CRTC_PROP_OUTPUT_FENCE) {
			ret = sde_fence_create(&sde_crtc->output_fence, val);
		} else {
			ret = msm_property_atomic_get(&sde_crtc->property_info,
					cstate->property_values,
					cstate->property_blobs, property, val);
		}
	}

	return ret;
}

static int _sde_debugfs_mixer_read(struct seq_file *s, void *data)
{
	struct sde_crtc *sde_crtc;
	struct sde_crtc_mixer *m;
	int i, j;

	if (!s || !s->private)
		return -EINVAL;

	sde_crtc = s->private;
	for (i = 0; i < sde_crtc->num_mixers; ++i) {
		m = &sde_crtc->mixers[i];
		if (!m->hw_lm) {
			seq_printf(s, "Mixer[%d] has no LM\n", i);
		} else if (!m->hw_ctl) {
			seq_printf(s, "Mixer[%d] has no CTL\n", i);
		} else {
			seq_printf(s, "LM_%d/CTL_%d\n",
					m->hw_lm->idx - LM_0,
					m->hw_ctl->idx - CTL_0);
		}
		seq_printf(s, "Border: %d\n",
				sde_crtc->stage_cfg.border_enable[i]);
	}
	for (i = 0; i < SDE_STAGE_MAX; ++i) {
		if (i == SDE_STAGE_BASE)
			seq_puts(s, "Base Stage:");
		else
			seq_printf(s, "Stage %d:", i - SDE_STAGE_0);

		for (j = 0; j < SDE_MAX_PIPES_PER_STAGE; ++j)
			seq_printf(s, " % 2d", sde_crtc->stage_cfg.stage[i][j]);
		seq_puts(s, "\n");
	}
	return 0;
}

static int _sde_debugfs_mixer_open(struct inode *inode, struct file *file)
{
	return single_open(file, _sde_debugfs_mixer_read, inode->i_private);
}

static const struct drm_crtc_funcs sde_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = sde_crtc_destroy,
	.page_flip = drm_atomic_helper_page_flip,
	.set_property = sde_crtc_set_property,
	.atomic_set_property = sde_crtc_atomic_set_property,
	.atomic_get_property = sde_crtc_atomic_get_property,
	.reset = sde_crtc_reset,
	.atomic_duplicate_state = sde_crtc_duplicate_state,
	.atomic_destroy_state = sde_crtc_destroy_state,
	.cursor_set = sde_crtc_cursor_set,
	.cursor_move = sde_crtc_cursor_move,
};

static const struct drm_crtc_helper_funcs sde_crtc_helper_funcs = {
	.mode_fixup = sde_crtc_mode_fixup,
	.mode_set_nofb = sde_crtc_mode_set_nofb,
	.disable = sde_crtc_disable,
	.enable = sde_crtc_enable,
	.atomic_check = sde_crtc_atomic_check,
	.atomic_begin = sde_crtc_atomic_begin,
	.atomic_flush = sde_crtc_atomic_flush,
};

static void _sde_crtc_init_debugfs(struct sde_crtc *sde_crtc,
		struct sde_kms *sde_kms)
{
	static const struct file_operations debugfs_mixer_fops = {
		.open =		_sde_debugfs_mixer_open,
		.read =		seq_read,
		.llseek =	seq_lseek,
		.release =	single_release,
	};
	if (sde_crtc && sde_kms) {
		sde_crtc->debugfs_root = debugfs_create_dir(sde_crtc->name,
				sde_debugfs_get_root(sde_kms));
		if (sde_crtc->debugfs_root) {
			/* don't error check these */
			debugfs_create_file("mixers", S_IRUGO,
					sde_crtc->debugfs_root,
					sde_crtc, &debugfs_mixer_fops);
		}
	}
}

/* initialize crtc */
struct drm_crtc *sde_crtc_init(struct drm_device *dev,
		struct drm_plane *primary_plane,
		struct drm_plane *cursor_plane,
		int drm_crtc_id)
{
	struct drm_crtc *crtc = NULL;
	struct sde_crtc *sde_crtc = NULL;
	struct msm_drm_private *priv = NULL;
	struct sde_kms *kms = NULL;

	priv = dev->dev_private;
	kms = to_sde_kms(priv->kms);

	sde_crtc = kzalloc(sizeof(*sde_crtc), GFP_KERNEL);
	if (!sde_crtc)
		return ERR_PTR(-ENOMEM);

	crtc = &sde_crtc->base;

	sde_crtc->drm_crtc_id = drm_crtc_id;
	atomic_set(&sde_crtc->drm_requested_vblank, 0);

	drm_crtc_init_with_planes(dev, crtc, primary_plane, cursor_plane,
					&sde_crtc_funcs);

	drm_crtc_helper_add(crtc, &sde_crtc_helper_funcs);
	if (primary_plane)
		primary_plane->crtc = crtc;
	if (cursor_plane)
		cursor_plane->crtc = crtc;

	/* save user friendly CRTC name for later */
	snprintf(sde_crtc->name, SDE_CRTC_NAME_SIZE, "crtc%u", crtc->base.id);

	/*
	 * Initialize output fence support. Set output fence offset to zero
	 * so that fences returned during a commit will signal at the end of
	 * the same commit.
	 */
	sde_fence_init(dev, &sde_crtc->output_fence, sde_crtc->name, 0);

	/* initialize debugfs support */
	_sde_crtc_init_debugfs(sde_crtc, kms);

	/* create CRTC properties */
	msm_property_init(&sde_crtc->property_info, &crtc->base, dev,
			priv->crtc_property, sde_crtc->property_data,
			CRTC_PROP_COUNT, CRTC_PROP_BLOBCOUNT,
			sizeof(struct sde_crtc_state));

	sde_crtc_install_properties(crtc);

	SDE_DEBUG("%s: successfully initialized crtc=%pK\n",
			sde_crtc->name, crtc);
	return crtc;
}
