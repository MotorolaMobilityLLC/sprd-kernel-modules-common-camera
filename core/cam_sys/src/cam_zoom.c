/*
 * Copyright (C) 2021-2022 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "cam_zoom.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_ZOOM: %d %d %s : " fmt, current->pid, __LINE__, __func__

static inline uint32_t camzoom_ratio16_divide(uint64_t num, uint32_t ratio16)
{
	return (uint32_t)div64_u64(num << 16, ratio16);
}

static inline uint32_t camzoom_scale_fix(uint32_t size_in, uint32_t ratio16)
{
	uint64_t size_scaled = 0;

	size_scaled = (uint64_t)size_in;
	size_scaled <<= (2 * RATIO_SHIFT);
	size_scaled = ((div64_u64(size_scaled, ratio16)) >> RATIO_SHIFT);
	return (uint32_t)size_scaled;
}

static inline void camzoom_largest_crop_get(
	struct sprd_img_rect *crop_dst, struct sprd_img_rect *crop1)
{
	uint32_t end_x = 0, end_y = 0;
	uint32_t end_x_new = 0, end_y_new = 0;

	if (crop1) {
		end_x = crop_dst->x + crop_dst->w;
		end_y = crop_dst->y + crop_dst->h;
		end_x_new = crop1->x + crop1->w;
		end_y_new = crop1->y + crop1->h;

		crop_dst->x = MIN(crop1->x, crop_dst->x);
		crop_dst->y = MIN(crop1->y, crop_dst->y);
		end_x_new = MAX(end_x, end_x_new);
		end_y_new = MAX(end_y, end_y_new);
		crop_dst->w = end_x_new - crop_dst->x;
		crop_dst->h = end_y_new - crop_dst->y;
	}
}

static inline int camzoom_deci_enable_get(uint32_t intrim_w, uint32_t intrim_h, uint32_t dst_w, uint32_t dst_h)
{
	return intrim_w > DCAM_SCALER_DECI_MAX_WIDTH && ((intrim_w > dst_w * DCAM_SCALE_DOWN_MAX) || (intrim_h > dst_h * DCAM_SCALE_DOWN_MAX));
}

static int camzoom_binning_swapsize_get(struct camera_module *module, struct img_size *max_bin)
{
	uint32_t i = 0, shift = 0, binning_limit = 2;
	uint32_t output_stg = 0;
	struct channel_context *ch_prev = NULL;
	struct channel_context *ch_vid = NULL;
	struct img_size dst_p = {0}, dst_v = {0}, max = {0};

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_vid = &module->channel[CAM_CH_VID];
	output_stg = module->grp->hw_info->ip_dcam[0]->dcam_output_strategy;

	dst_p.w = ch_prev->ch_uinfo.dst_size.w;
	dst_p.h = ch_prev->ch_uinfo.dst_size.h;
	dst_v.w = dst_v.h = 1;
	if (ch_vid->enable) {
		dst_p.w = ch_vid->ch_uinfo.dst_size.w;
		dst_p.h = ch_vid->ch_uinfo.dst_size.h;
	}

	switch (output_stg) {
	case IMG_POWER_CONSUM_PRI:
		if ((ch_prev->ch_uinfo.src_size.w * 2) <= module->cam_uinfo.sn_max_size.w) {
			if ((max_bin->w > g_camctrl.isp_linebuf_len) &&
				(dst_p.w <= g_camctrl.isp_linebuf_len) &&
				(dst_v.w <= g_camctrl.isp_linebuf_len))
				shift = 1;

			module->binning_limit = 0;
			if (module->zoom_solution == ZOOM_BINNING4)
				module->binning_limit = 1;
			else if (shift == 1)
				module->binning_limit = 1;
		} else {
			if ((max_bin->w >= (ch_prev->ch_uinfo.dst_size.w * 2)) &&
				(max_bin->w >= (ch_vid->ch_uinfo.dst_size.w * 2)))
				shift = 1;
			else if ((max_bin->w > g_camctrl.isp_linebuf_len) &&
				(dst_p.w <= g_camctrl.isp_linebuf_len) &&
				(dst_v.w <= g_camctrl.isp_linebuf_len))
				shift = 1;

			module->binning_limit = 1;
			if (module->zoom_solution == ZOOM_BINNING4)
				module->binning_limit = 2;
		}
		break;
	case IMG_QUALITY_PRI:
		max.w = max_bin->w;
		while (i < binning_limit) {
			if (max.w > g_camctrl.isp_linebuf_len) {
				max.w = max.w >> 1;
				if ((max.w > dst_p.w) && (max.w > dst_v.w))
					shift++;
				else
					break;
			}
			i++;
		}
		break;
	default:
		pr_err("fail to get invalid output type %d\n", output_stg);
		break;
	}

	if (SEC_UNABLE != module->grp->camsec_cfg.camsec_mode)
		shift = 1;
	max_bin->w >>= shift;
	max_bin->h >>= shift;

	pr_debug("shift %d for binning, p=%d v=%d src=%d, %d\n",
		shift, dst_p.w, dst_v.w, max_bin->w, g_camctrl.isp_linebuf_len);

	if ((dst_p.w == 0) || (dst_p.h == 0)) {
		pr_err("fail to get valid w %d h %d\n", dst_p.w, dst_p.h);
		return -EFAULT;
	}
	return 0;
}

static void camzoom_scaler_swapsize_get(struct img_size src_size, struct channel_context *ch_prev,
	struct channel_context *ch_vid, struct img_size *max_scaler, struct img_size *max_bin)
{
	uint32_t ratio_p = 0, ratio_v = 0, ratio_src = 0;
	uint32_t max_dst_size = 0, min_dst_size = 0, min_dst_ratio = 0, max_dst_ratio = 0;

	/* TBD: slowmotion case dcam keep no scaler first */
	if (ch_prev->ch_uinfo.is_high_fps ||
		camzoom_deci_enable_get(ch_prev->ch_uinfo.src_crop.w, ch_prev->ch_uinfo.src_crop.h, ch_prev->ch_uinfo.dst_size.w, ch_prev->ch_uinfo.dst_size.h)) {
		max_scaler->w = MAX(ch_prev->ch_uinfo.dst_size.w, ch_vid->ch_uinfo.dst_size.w);
		max_scaler->h = MAX(ch_prev->ch_uinfo.dst_size.h, ch_vid->ch_uinfo.dst_size.h);
		max_scaler->w = MAX(max_bin->w, max_scaler->w);
		max_scaler->h = MAX(max_bin->h, max_scaler->h);
		return;
	}

	ratio_src = (1 << RATIO_SHIFT) * src_size.w / src_size.h;
	if (ch_prev->enable)
		ratio_p = (1 << RATIO_SHIFT) * ch_prev->ch_uinfo.dst_size.w / ch_prev->ch_uinfo.dst_size.h;
	if (ch_vid->enable)
		ratio_v = (1 << RATIO_SHIFT) * ch_vid->ch_uinfo.dst_size.w / ch_vid->ch_uinfo.dst_size.h;
	min_dst_ratio = max_dst_ratio = ratio_src;
	if (ratio_p) {
		min_dst_ratio = MIN(min_dst_ratio, ratio_p);
		max_dst_ratio = MAX(max_dst_ratio, ratio_p);
	}
	if (ratio_v) {
		min_dst_ratio = MIN(min_dst_ratio, ratio_v);
		max_dst_ratio = MAX(max_dst_ratio, ratio_v);
	}
	max_scaler->w = MAX(ch_prev->ch_uinfo.dst_size.w, ch_vid->ch_uinfo.dst_size.w);
	max_scaler->h = MAX(ch_prev->ch_uinfo.dst_size.h, ch_vid->ch_uinfo.dst_size.h);
	max_dst_size = MAX(max_scaler->w, max_scaler->h);
	min_dst_size = MIN(max_scaler->w, max_scaler->h);
	pr_debug("ratio %d %d %d max_scaler %d %d min_ratio %d max_ratio %d min_dst %d max_dst %d\n", ratio_p,
		ratio_v, ratio_src, max_scaler->w, max_scaler->h, min_dst_ratio, max_dst_ratio, min_dst_size, max_dst_size);
	/* use max size update swap size */
	max_scaler->w = min_dst_size * max_dst_ratio / (1 << RATIO_SHIFT);
	max_scaler->h = camzoom_ratio16_divide(max_dst_size, min_dst_ratio);
	max_scaler->w = ALIGN_UP(max_scaler->w, 2);
	max_scaler->h = ALIGN_UP(max_scaler->h, 2);
	max_scaler->w = MIN(max_scaler->w, src_size.w);
	max_scaler->h = MIN(max_scaler->h, src_size.h);
}

static int camzoom_channel_swapsize_calc(struct camera_module *module)
{
	uint32_t ret = 0;
	struct channel_context *ch_prev = NULL;
	struct channel_context *ch_vid = NULL;
	struct channel_context *ch_cap = NULL;
	struct channel_context *ch_raw = NULL;
	struct img_size max_bypass = {0}, max_bin = {0}, max_scaler = {0}, max = {0}, src_p = {0};

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	ch_raw = &module->channel[CAM_CH_RAW];

	if (ch_cap->enable) {
		max.w = ch_cap->ch_uinfo.src_size.w;
		max.h = ch_cap->ch_uinfo.src_size.h;
		ch_cap->swap_size = max;
		pr_info("idx %d , cap swap size %d %d\n", module->idx, max.w, max.h);
	}

	if (ch_raw->enable) {
		max.w = ch_raw->ch_uinfo.src_size.w;
		max.h = ch_raw->ch_uinfo.src_size.h;
		ch_raw->swap_size = max;
		pr_info("idx %d , raw swap size %d %d\n", module->idx, max.w, max.h);
	}

	if (ch_prev->enable)
		ch_prev = &module->channel[CAM_CH_PRE];
	else if (!ch_prev->enable && ch_vid->enable)
		ch_prev = &module->channel[CAM_CH_VID];
	else
		return 0;

	if (module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode) {
		ch_prev->ch_uinfo.src_size.w >>= 1;
		ch_prev->ch_uinfo.src_size.h >>= 1;
		ch_vid->ch_uinfo.src_size.w >>= 1;
		ch_vid->ch_uinfo.src_size.h >>= 1;
	}

	src_p.w = ch_prev->ch_uinfo.src_size.w;
	src_p.h = ch_prev->ch_uinfo.src_size.h;

	/* max_bypass: no scaler & binning; max_bin: bining */
	max_bypass = max_bin = src_p;
	/* update max_bin */
	ret = camzoom_binning_swapsize_get(module, &max_bin);

	switch (module->zoom_solution) {
	case ZOOM_DEFAULT:
		ch_prev->swap_size = max_bypass;
		break;
	case ZOOM_BINNING2:
	case ZOOM_BINNING4:
		ch_prev->swap_size = max_bin;
		break;
	case ZOOM_SCALER:
		camzoom_scaler_swapsize_get(src_p, ch_prev, ch_vid, &max_scaler, &max_bin);
		ch_prev->swap_size = max_scaler;
		break;
	default:
		pr_warn("warning: unknown zoom solution %d\n", module->zoom_solution);
		ch_prev->swap_size = max_bypass;
		break;
	}
	pr_info("prev bypass size (%d %d), bin size (%d %d)\n",
		max_bypass.w, max_bypass.h, max_bin.w, max_bin.h);
	pr_info("prev swap size (%d %d)\n", ch_prev->swap_size.w, ch_prev->swap_size.h);

	return 0;
}

static int camzoom_binning_shift_calc(struct camera_module *module, struct img_trim trim_pv) {
	uint32_t i = 0, shift = 0, binning_limit = 2;
	uint32_t factor = 0, src_binning = 0;
	uint32_t output_stg = 0;
	struct channel_context *ch_prev = NULL;
	struct channel_context *ch_vid = NULL;
	struct img_size max_dst_pv = {0}, max = {0};

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_vid = &module->channel[CAM_CH_VID];
	output_stg = module->grp->hw_info->ip_dcam[0]->dcam_output_strategy;

	switch (output_stg) {
	case IMG_POWER_CONSUM_PRI:
		if ((ch_prev->ch_uinfo.src_size.w * 2) <= module->cam_uinfo.sn_max_size.w)
			src_binning = 1;
		factor = (src_binning ? 10 : 9);
		if ((trim_pv.size_x >= (ch_prev->ch_uinfo.dst_size.w * 4)) &&
			(trim_pv.size_x >= (ch_vid->ch_uinfo.dst_size.w * 4)) &&
			(trim_pv.size_y >= (ch_prev->ch_uinfo.dst_size.h * 4)) &&
			(trim_pv.size_y >= (ch_vid->ch_uinfo.dst_size.h * 4)))
			shift = 2;
		else if ((trim_pv.size_x >= (ch_prev->ch_uinfo.dst_size.w * 2 * factor / 10)) &&
			(trim_pv.size_x >= (ch_vid->ch_uinfo.dst_size.w * 2 * factor / 10)) &&
			(trim_pv.size_y >= (ch_prev->ch_uinfo.dst_size.h * 2 * factor / 10)) &&
			(trim_pv.size_y >= (ch_vid->ch_uinfo.dst_size.h * 2 * factor / 10)))
				shift = 1;
		break;
	case IMG_QUALITY_PRI:
		max_dst_pv.w = MAX(ch_prev->ch_uinfo.dst_size.w, ch_vid->ch_uinfo.dst_size.w);
		max.w = trim_pv.size_x;
		while (i < binning_limit) {
			if (max.w > g_camctrl.isp_linebuf_len) {
				max.w = max.w >> 1;
				if (max.w > max_dst_pv.w)
					shift++;
				else
					break;
			}
			i++;
		}
		break;
	default:
		pr_err("fail to get invalid output type %d\n", output_stg);
		break;
	}
	if (((trim_pv.size_x >> shift) > ch_prev->swap_size.w) ||
		((trim_pv.size_y >> shift) > ch_prev->swap_size.h))
			shift++;
	pr_debug("dcam binning zoom shift %d, type %d\n", shift, output_stg);
	if (shift > 2) {
		pr_info("dcam binning should limit to 1/4\n");
		shift = 2;
	}
	if (shift > module->binning_limit) {
		pr_info("bin shift limit to %d\n", module->binning_limit);
		shift = module->binning_limit;
	}

	return shift;
}

static int camzoom_port_info_cfg(struct cam_zoom_port *zoom_port,
	uint32_t node_type, uint32_t node_id, struct cam_zoom_desc *param)
{
	int valid = 0;
	struct cam_zoom_base *zoom_base = NULL;

	zoom_base = &zoom_port->zoom_base;
	switch (node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		if (zoom_port->port_id == PORT_BIN_OUT ||
			zoom_port->port_id == PORT_RAW_OUT ||
			zoom_port->port_id == PORT_FULL_OUT) {
			zoom_base->src = param->sn_rect_size;
			zoom_base->crop = param->dcam_crop[zoom_port->port_id];
			zoom_base->dst = param->dcam_dst[zoom_port->port_id];
			zoom_base->ratio_width = param->zoom_ratio_width;
			zoom_base->total_crop_width = param->total_crop_width;
			valid = 1;
			CAM_ZOOM_DEBUG("dcam size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		if (zoom_port->port_type == PORT_TRANSFER_IN) {
			zoom_base->src = param->isp_src_size;
			/* node input no need crop same with pre node output */
			zoom_base->crop.start_x = 0;
			zoom_base->crop.start_y = 0;
			zoom_base->crop.size_x = zoom_base->src.w;
			zoom_base->crop.size_y = zoom_base->src.h;
			zoom_base->dst = zoom_base->src;
			valid = 1;
			CAM_ZOOM_DEBUG("isp size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		if (zoom_port->port_type == PORT_TRANSFER_OUT &&
			(zoom_port->port_id == PORT_PRE_OUT || zoom_port->port_id == PORT_VID_OUT
			|| zoom_port->port_id == PORT_CAP_OUT || zoom_port->port_id == PORT_THUMB_OUT)) {
			zoom_base->src = param->isp_src_size;
			zoom_base->crop = param->isp_crop[zoom_port->port_id];
			zoom_base->dst = param->dcam_isp[zoom_port->port_id];
			valid = 1;
			CAM_ZOOM_DEBUG("isp id %d size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_port->port_id, zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		break;
	default :
		pr_err("fail to support port %d zoom cfg in node %d\n", zoom_port->port_id, node_type);
		break;
	}

	return valid;
}

static void camzoom_frame_param_cfg(struct cam_zoom_desc *param, struct cam_zoom_frame *zoom_param)
{
	int ret = 0, i = 0, j = 0, m = 0, n = 0;
	struct cam_pipeline_topology *graph = NULL;

	graph = param->pipeline_graph;
	for (i = 0; i < graph->node_cnt; i++) {
		n = 0;
		if (graph->nodes[i].state != CAM_NODE_STATE_WORK ||
			graph->nodes[i].type == CAM_NODE_TYPE_DATA_COPY ||
			graph->nodes[i].type == CAM_NODE_TYPE_FRAME_CACHE)
			continue;
		zoom_param->zoom_node[m].node_type = graph->nodes[i].type;
		zoom_param->zoom_node[m].node_id = graph->nodes[i].id;
		for (j = 0; j < CAM_NODE_PORT_IN_NUM; j++) {
			if (graph->nodes[i].inport[j].link_state != PORT_LINK_NORMAL)
				continue;
			zoom_param->zoom_node[m].zoom_port[n].port_type = graph->nodes[i].inport[j].transfer_type;
			zoom_param->zoom_node[m].zoom_port[n].port_id = graph->nodes[i].inport[j].id;
			ret = camzoom_port_info_cfg(&zoom_param->zoom_node[m].zoom_port[n],
				graph->nodes[i].type, graph->nodes[i].id, param);
			if (!ret)
				continue;
			n++;
			if (n >= PORT_ZOOM_CNT_MAX)
				break;
		}

		for (j = 0; j < CAM_NODE_PORT_OUT_NUM; j++) {
			if (graph->nodes[i].outport[j].link_state != PORT_LINK_NORMAL)
				continue;
			zoom_param->zoom_node[m].zoom_port[n].port_type = graph->nodes[i].outport[j].transfer_type;
			zoom_param->zoom_node[m].zoom_port[n].port_id = graph->nodes[i].outport[j].id;
			ret = camzoom_port_info_cfg(&zoom_param->zoom_node[m].zoom_port[n],
				graph->nodes[i].type, graph->nodes[i].id, param);
			if (!ret)
				continue;
			n++;
			if (n >= PORT_ZOOM_CNT_MAX)
				break;
		}
		m++;
		if (m >= NODE_ZOOM_CNT_MAX)
			break;
	}
}

void cam_zoom_diff_trim_get(struct sprd_img_rect *orig,
	uint32_t ratio16, struct img_trim *trim0, struct img_trim *trim1)
{
	trim1->start_x = camzoom_scale_fix(orig->x - trim0->start_x, ratio16);
	trim1->start_y = camzoom_scale_fix(orig->y - trim0->start_y, ratio16);
	trim1->size_x = camzoom_scale_fix(orig->w, ratio16);
	trim1->size_x = ALIGN(trim1->size_x, 2);
	trim1->size_y = camzoom_scale_fix(orig->h, ratio16);
}

int cam_zoom_crop_size_align(struct camera_module *module,
		struct sprd_img_rect *crop, uint32_t channel_id)
{
	struct img_size max_size = {0};

	max_size.w = module->cam_uinfo.sn_rect.w;
	max_size.h = module->cam_uinfo.sn_rect.h;

	/* 4in1 prev, enable 4in1 binning OR size > 24M, size/2 */
	if ((module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode) &&
		((channel_id == CAM_CH_PRE) || (channel_id == CAM_CH_VID))) {
		crop->x >>= 1;
		crop->y >>= 1;
		crop->w >>= 1;
		crop->h >>= 1;
		max_size.w >>= 1;
		max_size.h >>= 1;
	}

	crop->w = ALIGN_UP(crop->w, DCAM_PATH_CROP_ALIGN);
	crop->h = ALIGN_UP(crop->h, DCAM_PATH_CROP_ALIGN);
	if (max_size.w > crop->w)
		crop->x = (max_size.w - crop->w) / 2;
	if (max_size.h > crop->h)
		crop->y = (max_size.h - crop->h) / 2;
	crop->x &= ~1;
	crop->y &= ~1;

	if ((crop->x + crop->w) > max_size.w)
		crop->w -= DCAM_PATH_CROP_ALIGN;
	if ((crop->y + crop->h) > max_size.h)
		crop->h -= DCAM_PATH_CROP_ALIGN;

	pr_info("aligned crop %d %d %d %d.  max %d %d\n",
		crop->x, crop->y, crop->w, crop->h, max_size.w, max_size.h);

	return 0;
}

int cam_zoom_channels_size_init(struct camera_module *module)
{
	uint32_t format = module->cam_uinfo.sensor_if.img_fmt;

	module->zoom_solution = module->grp->hw_info->ip_dcam[0]->dcam_zoom_mode;
	if (g_camctrl.dcam_zoom_mode >= ZOOM_DEBUG_DEFAULT)
		module->zoom_solution = g_camctrl.dcam_zoom_mode - ZOOM_DEBUG_DEFAULT;

	/* force binning as smaller as possible for security */
	if (module->grp->camsec_cfg.camsec_mode != SEC_UNABLE)
		module->zoom_solution = ZOOM_BINNING4;

	if (format == DCAM_CAP_MODE_YUV)
		module->zoom_solution = ZOOM_DEFAULT;

	camzoom_channel_swapsize_calc(module);

	pr_info("zoom_solution %d, binning_limit %d\n", module->zoom_solution, module->binning_limit);

	return 0;
}

int cam_zoom_channel_size_calc(struct camera_module *module)
{
	uint32_t shift = 0, align_size = 0;
	uint32_t ratio_p_w = 0, ratio_p_h = 0, ratio_v_w = 0, ratio_v_h = 0;
	uint32_t ratio_min_w = 0, ratio_min_h = 0, ratio_min = 0;
	struct channel_context *ch_prev = NULL;
	struct channel_context *ch_vid = NULL;
	struct channel_context *ch_cap = NULL;
	struct channel_context *ch_cap_thm = NULL;
	struct sprd_img_rect *crop_p = NULL, *crop_v = NULL;
	struct sprd_img_rect *total_crop_p = NULL;
	struct sprd_img_rect crop_dst = {0};
	struct sprd_img_rect total_crop_dst = {0};
	struct img_trim total_trim_pv = {0};
	struct img_trim trim_pv = {0};
	struct img_trim trim_c = {0};
	struct img_trim *isp_trim = NULL;
	struct img_size src_p = {0}, dst_p = {0}, dst_v = {0}, dcam_out = {0}, max_dst_pv = {0};

	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	ch_cap_thm = &module->channel[CAM_CH_CAP_THM];
	if (!ch_prev->enable && !ch_cap->enable && !ch_vid->enable)
		return 0;

	dst_p.w = dst_p.h = 1;
	dst_v.w = dst_v.h = 1;
	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		src_p.w = ch_prev->ch_uinfo.src_size.w;
		src_p.h = ch_prev->ch_uinfo.src_size.h;
		crop_p = &ch_prev->ch_uinfo.src_crop;
		total_crop_p = &ch_prev->ch_uinfo.total_src_crop;
		dst_p.w = ch_prev->ch_uinfo.dst_size.w;
		dst_p.h = ch_prev->ch_uinfo.dst_size.h;
		pr_info("src crop prev %u %u %u %u\n", crop_p->x, crop_p->y, crop_p->w, crop_p->h);
	}
	if (ch_vid->enable) {
		crop_v = &ch_vid->ch_uinfo.src_crop;
		dst_v.w = ch_vid->ch_uinfo.dst_size.w;
		dst_v.h = ch_vid->ch_uinfo.dst_size.h;
		pr_info("src crop vid %u %u %u %u\n",
			crop_v->x, crop_v->y, crop_v->w, crop_v->h);
	}
	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		crop_dst = *crop_p;
		camzoom_largest_crop_get(&crop_dst, crop_v);
		trim_pv.start_x = crop_dst.x;
		trim_pv.start_y = crop_dst.y;
		trim_pv.size_x = crop_dst.w;
		trim_pv.size_y = crop_dst.h;
		max_dst_pv.w = MAX(dst_p.w, dst_v.w);
		max_dst_pv.h = MAX(dst_p.h, dst_v.h);
	}

	if (ch_cap->enable) {
		trim_c.start_x = ch_cap->ch_uinfo.src_crop.x;
		trim_c.start_y = ch_cap->ch_uinfo.src_crop.y;
		trim_c.size_x = ch_cap->ch_uinfo.src_crop.w;
		trim_c.size_y = ch_cap->ch_uinfo.src_crop.h;
	}
	pr_info("trim_pv: %u %u %u %u\n", trim_pv.start_x,
		trim_pv.start_y, trim_pv.size_x, trim_pv.size_y);
	pr_info("trim_c: %u %u %u %u\n", trim_c.start_x,
		trim_c.start_y, trim_c.size_x, trim_c.size_y);

	if (ch_prev->enable || (!ch_prev->enable && ch_vid->enable)) {
		if (module->zoom_solution == ZOOM_BINNING2 || module->zoom_solution == ZOOM_BINNING4)
			shift = camzoom_binning_shift_calc(module, trim_pv);
		if (shift == 2) {
			/* make sure output 2 aligned and trim invalid */
			pr_debug("shift 2 trim_pv %d %d %d %d\n",
				trim_pv.start_x, trim_pv.start_y,
				trim_pv.size_x, trim_pv.size_y);
			if ((trim_pv.size_x >> 2) & 1) {
				trim_pv.size_x = (trim_pv.size_x + 4) & ~7;
				if ((trim_pv.start_x + trim_pv.size_x) > src_p.w)
					trim_pv.size_x -= 8;
				trim_pv.start_x = (src_p.w - trim_pv.size_x) >> 1;
			}
			if ((trim_pv.size_y >> 2) & 1) {
				trim_pv.size_y = (trim_pv.size_y + 4) & ~7;
				if ((trim_pv.start_y + trim_pv.size_y) > src_p.h)
					trim_pv.size_y -= 8;
				trim_pv.start_y = (src_p.h - trim_pv.size_y) >> 1;
			}
			pr_debug("shift 2 trim_pv final %d %d %d %d\n",
				trim_pv.start_x, trim_pv.start_y,
				trim_pv.size_x, trim_pv.size_y);
		}

		if (shift == 1)
			align_size = 8;
		else if (shift == 2)
			align_size = 16;
		else
			align_size = 4;

		trim_pv.size_x = ALIGN_DOWN(trim_pv.size_x, align_size);
		trim_pv.size_y = ALIGN_DOWN(trim_pv.size_y, align_size / 2);

		dcam_out.w = (trim_pv.size_x >> shift);
		dcam_out.w = ALIGN_DOWN(dcam_out.w, 2);
		dcam_out.h = (trim_pv.size_y >> shift);
		dcam_out.h = ALIGN_DOWN(dcam_out.h, 2);

		if (module->zoom_solution == ZOOM_SCALER) {
			ratio_min = 1 << RATIO_SHIFT;
			ratio_min_w = 1 << RATIO_SHIFT;
			ratio_min_h = 1 << RATIO_SHIFT;
			/* TBD: slowmotion case dcam keep no scaler first */
			if (ch_prev->ch_uinfo.is_high_fps) {
				dst_p = ch_prev->swap_size;
				dst_v = ch_prev->swap_size;
			}
			if (trim_pv.size_x > ch_prev->swap_size.w || trim_pv.size_y > ch_prev->swap_size.h) {
				ratio_p_w = (1 << RATIO_SHIFT) * trim_pv.size_x / dst_p.w;
				ratio_p_h = (1 << RATIO_SHIFT) * trim_pv.size_y / dst_p.h;
				ratio_v_w = (1 << RATIO_SHIFT) * trim_pv.size_x / dst_v.w;
				ratio_v_h = (1 << RATIO_SHIFT) * trim_pv.size_y / dst_v.h;
				ratio_min = MIN(MIN(ratio_p_w, ratio_p_h), MIN(ratio_v_w, ratio_v_h));
				ratio_min_w = ratio_min_h = ratio_min;
				dcam_out.w = camzoom_ratio16_divide(trim_pv.size_x, ratio_min);
				dcam_out.h = camzoom_ratio16_divide(trim_pv.size_y, ratio_min);
				dcam_out.w = ALIGN(dcam_out.w, 4);
				dcam_out.h = ALIGN(dcam_out.h, 2);
				if ((dcam_out.w != max_dst_pv.w && abs(dcam_out.w - max_dst_pv.w) <= DCAM_PATH_CROP_ALIGN) ||
					(dcam_out.h != max_dst_pv.h && abs(dcam_out.h - max_dst_pv.h) <= DCAM_PATH_CROP_ALIGN)) {
					dcam_out.w = dcam_out.w > max_dst_pv.w ? max_dst_pv.w : dcam_out.w;
					dcam_out.h = dcam_out.h > max_dst_pv.h ? max_dst_pv.h : dcam_out.h;
					ratio_min_w = (1 << RATIO_SHIFT) * trim_pv.size_x / dcam_out.w;
					ratio_min_h = (1 << RATIO_SHIFT) * trim_pv.size_y / dcam_out.h;
					pr_debug("ratio %d %d\n", ratio_min_w, ratio_min_h);
				}
				if (abs(dcam_out.w - max_dst_pv.w) > DCAM_PATH_CROP_ALIGN ||
					abs(dcam_out.h - max_dst_pv.h) > DCAM_PATH_CROP_ALIGN)
					pr_warn("warning: inconsistent scaling of width and height\n");
			}

			if (camzoom_deci_enable_get(trim_pv.size_x, trim_pv.size_y, dcam_out.w, dcam_out.h)) {
				dcam_out.w = trim_pv.size_x / DCAM_SCALE_DOWN_MAX;
				dcam_out.h = (dcam_out.w * trim_pv.size_y + trim_pv.size_x - 1) / trim_pv.size_x;
				dcam_out.w = ALIGN(dcam_out.w, 4);
				dcam_out.h = ALIGN(dcam_out.h, 2);
				if (ch_prev->compress_en)
					dcam_out.h = ALIGN(dcam_out.h, DCAM_FBC_TILE_HEIGHT);
				ratio_min_w = (1 << RATIO_SHIFT) * trim_pv.size_x / dcam_out.w;
				ratio_min_h = (1 << RATIO_SHIFT) * trim_pv.size_y / dcam_out.h;
			}
		}

		if (dcam_out.w > DCAM_SCALER_MAX_WIDTH) {
			dcam_out.h = dcam_out.h * DCAM_SCALER_MAX_WIDTH / dcam_out.w;
			dcam_out.h = ALIGN_DOWN(dcam_out.h, 2);
			dcam_out.w = DCAM_SCALER_MAX_WIDTH;
			ratio_min_w = (1 << RATIO_SHIFT) * trim_pv.size_x / dcam_out.w;
			ratio_min_h = (1 << RATIO_SHIFT) * trim_pv.size_y / dcam_out.h;
		}

		if (ch_prev->compress_en)
			dcam_out.h = ALIGN_DOWN(dcam_out.h, DCAM_FBC_TILE_HEIGHT);

		pr_info("shift %d, dst_p %u %u, dst_v %u %u, dcam_out %u %u swap w:%d h:%d\n",
			shift, dst_p.w, dst_p.h, dst_v.w, dst_v.h, dcam_out.w, dcam_out.h,
			ch_prev->swap_size.w, ch_prev->swap_size.h);

		/* applied latest rect for aem */
		ch_prev->trim_dcam = trim_pv;

		total_crop_dst = *total_crop_p;
		total_trim_pv.size_x = total_crop_dst.w;
		total_trim_pv.size_y = total_crop_dst.h;
		ch_prev->total_trim_dcam = total_trim_pv;
		ch_prev->dst_dcam = dcam_out;

		isp_trim = &ch_prev->trim_isp;
		if (module->zoom_solution == ZOOM_SCALER) {
			isp_trim->size_x =
				camzoom_ratio16_divide(ch_prev->ch_uinfo.src_crop.w, ratio_min_w);
			isp_trim->size_y =
				camzoom_ratio16_divide(ch_prev->ch_uinfo.src_crop.h, ratio_min_h);
			isp_trim->size_x = ALIGN(isp_trim->size_x, 4);
			isp_trim->size_y = ALIGN(isp_trim->size_y, 2);
		} else {
			isp_trim->size_x = ((ch_prev->ch_uinfo.src_crop.w >> shift) + 1) & ~1;
			isp_trim->size_y = ((ch_prev->ch_uinfo.src_crop.h >> shift) + 1) & ~1;
		}
		isp_trim->size_x = min(isp_trim->size_x, dcam_out.w);
		isp_trim->size_y = min(isp_trim->size_y, dcam_out.h);
		isp_trim->start_x = ((dcam_out.w - isp_trim->size_x) >> 1) & ~1;
		isp_trim->start_y = ((dcam_out.h - isp_trim->size_y) >> 1) & ~1;
		pr_info("trim isp, prev %u %u %u %u\n",
			isp_trim->start_x, isp_trim->start_y,
			isp_trim->size_x, isp_trim->size_y);
	}

	if (ch_vid->enable) {
		ch_vid->dst_dcam = dcam_out;
		ch_vid->trim_dcam = trim_pv;
		isp_trim = &ch_vid->trim_isp;
		if (module->zoom_solution == ZOOM_SCALER) {
			isp_trim->size_x =
				camzoom_ratio16_divide(ch_vid->ch_uinfo.src_crop.w, ratio_min_w);
			isp_trim->size_y =
				camzoom_ratio16_divide(ch_vid->ch_uinfo.src_crop.h, ratio_min_h);
			isp_trim->size_x = ALIGN(isp_trim->size_x, 4);
			isp_trim->size_y = ALIGN(isp_trim->size_y, 2);
		} else {
			isp_trim->size_x = ((ch_vid->ch_uinfo.src_crop.w >> shift) + 1) & ~1;
			isp_trim->size_y = ((ch_vid->ch_uinfo.src_crop.h >> shift) + 1) & ~1;
		}
		isp_trim->size_x = min(isp_trim->size_x, dcam_out.w);
		isp_trim->size_y = min(isp_trim->size_y, dcam_out.h);
		isp_trim->start_x = ((dcam_out.w - isp_trim->size_x) >> 1) & ~1;
		isp_trim->start_y = ((dcam_out.h - isp_trim->size_y) >> 1) & ~1;
		pr_info("trim isp, vid %u %u %u %u\n",
			isp_trim->start_x, isp_trim->start_y,
			isp_trim->size_x, isp_trim->size_y);
	}

	if (ch_cap->enable) {
		ch_cap->trim_dcam = trim_c;
		ch_cap->dst_dcam.w = ch_cap->trim_dcam.size_x;
		ch_cap->dst_dcam.h = ch_cap->trim_dcam.size_y;
		cam_zoom_diff_trim_get(&ch_cap->ch_uinfo.src_crop,
			(1 << RATIO_SHIFT), &trim_c, &ch_cap->trim_isp);
		ch_cap->trim_isp.start_x = ALIGN_DOWN(ch_cap->trim_isp.start_x, 2);
		ch_cap->trim_isp.start_y = ALIGN_DOWN(ch_cap->trim_isp.start_y, 2);
		ch_cap->trim_isp.size_x = ALIGN_DOWN(ch_cap->trim_isp.size_x, 2);
		ch_cap->trim_isp.size_y = ALIGN_DOWN(ch_cap->trim_isp.size_y, 2);
		ch_cap->trim_isp.size_x = min(ch_cap->trim_isp.size_x, trim_c.size_x);
		ch_cap->trim_isp.size_y = min(ch_cap->trim_isp.size_y, trim_c.size_y);
		pr_info("trim isp, cap %d %d %d %d\n",
			ch_cap->trim_isp.start_x, ch_cap->trim_isp.start_y,
			ch_cap->trim_isp.size_x, ch_cap->trim_isp.size_y);
		if (ch_cap_thm->enable) {
			ch_cap_thm->trim_dcam = ch_cap->trim_dcam;
			ch_cap_thm->trim_isp = ch_cap->trim_isp;
		}
	}

	pr_info("done\n");
	return 0;
}

int cam_zoom_channel_size_config(
	struct camera_module *module, struct channel_context *channel)
{
	int ret = 0;
	uint32_t need_raw_port = 0, raw_port_id = 0;
	struct channel_context *ch_vid = NULL;
	struct camera_uchannel *ch_uinfo = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_zoom_desc zoom_info = {0};
	struct cam_zoom_base raw_zoom_base = {0};

	if (!module || !channel) {
		pr_err("fail to get valid param %p %p\n", module, channel);
		return -EFAULT;
	}

	ch_uinfo = &channel->ch_uinfo;
	ch_vid = &module->channel[CAM_CH_VID];
	hw = module->grp->hw_info;

	raw_port_id = hw->ip_dcam[0]->dcam_raw_path_id;
	raw_port_id = dcamonline_pathid_convert_to_portid(raw_port_id);
	raw_zoom_base.src = ch_uinfo->src_size;
	raw_zoom_base.dst = ch_uinfo->src_size;
	raw_zoom_base.crop.start_x = 0;
	raw_zoom_base.crop.start_y = 0;
	raw_zoom_base.crop.size_x = ch_uinfo->src_size.w;
	raw_zoom_base.crop.size_y = ch_uinfo->src_size.h;
	if (channel->ch_id == CAM_CH_RAW || channel->dcam_port_id == PORT_RAW_OUT
		|| module->cam_uinfo.raw_alg_type == RAW_ALG_AI_SFNR || module->cam_uinfo.is_4in1) {
		need_raw_port = 1;
	}

	zoom_info.pipeline_type = channel->pipeline_type;
	zoom_info.pipeline_graph = &module->static_topology->pipeline_list[channel->pipeline_type];
	zoom_info.latest_zoom_info = &channel->latest_zoom_param;
	zoom_info.zoom_lock = &channel->lastest_zoom_lock;
	zoom_info.zoom_info_q = &channel->zoom_param_q;
	zoom_info.sn_rect_size = ch_uinfo->src_size;
	zoom_info.zoom_ratio_width = ch_uinfo->zoom_ratio_base.w;
	zoom_info.total_crop_width = channel->total_trim_dcam.size_x;
	if (IS_VALID_DCAM_IMG_PORT(channel->dcam_port_id)) {
		zoom_info.dcam_crop[channel->dcam_port_id] = channel->trim_dcam;
		zoom_info.dcam_dst[channel->dcam_port_id] = channel->dst_dcam;
	}
	if (need_raw_port && IS_VALID_DCAM_IMG_PORT(raw_port_id)) {
		zoom_info.dcam_crop[raw_port_id] = raw_zoom_base.crop;
		zoom_info.dcam_dst[raw_port_id] = raw_zoom_base.dst;
	}
	zoom_info.isp_src_size = channel->dst_dcam;
	if (IS_VALID_ISP_IMG_PORT(channel->isp_port_id)) {
		zoom_info.dcam_isp[channel->isp_port_id] = channel->ch_uinfo.dst_size;
		zoom_info.isp_crop[channel->isp_port_id] = channel->trim_isp;
	}
	if (ch_vid->enable) {
		zoom_info.isp_crop[PORT_VID_OUT] = ch_vid->trim_isp;
		zoom_info.dcam_isp[PORT_VID_OUT] = ch_vid->ch_uinfo.dst_size;
	}
	ret = cam_zoom_param_set(&zoom_info);

	return ret;
}

int cam_zoom_4in1_channel_size_config(struct camera_module *module)
{
	int ret = 0;
	struct channel_context *ch = NULL;
	struct camera_uchannel ch_uinfo = {0};
	struct img_trim trim_dcam = {0};
	struct img_trim trim_isp = {0};
	struct dcam_path_cfg_param ch_desc = {0};
	struct isp_size_desc size_cfg = {0};

	ch = &module->channel[CAM_CH_CAP];
	memcpy(&ch_uinfo, &ch->ch_uinfo, sizeof(struct camera_uchannel));

	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.input_size.w = ch_uinfo.src_size.w;
	ch_desc.input_size.h = ch_uinfo.src_size.h;
	ch_desc.output_size = ch_desc.input_size;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->aux_dcam_port_id, CAM_PIPELINE_CFG_SIZE, &ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE);

	pr_info("src[%d %d], crop[%d %d %d %d] dst[%d %d]\n",
		ch_uinfo.src_size.w,ch_uinfo.src_size.h,
		ch_uinfo.src_crop.x, ch_uinfo.src_crop.y, ch_uinfo.src_crop.w, ch_uinfo.src_crop.h,
		ch_uinfo.dst_size.w, ch_uinfo.dst_size.h);
	trim_dcam.start_x = ch_uinfo.src_crop.x;
	trim_dcam.start_y = ch_uinfo.src_crop.y;
	trim_dcam.size_x = ch_uinfo.src_crop.w;
	trim_dcam.size_y = ch_uinfo.src_crop.h;
	cam_zoom_diff_trim_get(&ch_uinfo.src_crop, (1 << RATIO_SHIFT), &trim_dcam, &trim_isp);
	pr_info("trim_isp[%d %d %d %d]\n", trim_isp.start_x, trim_isp.start_y,
		trim_isp.size_x, trim_isp.size_y);

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_warn("warning: cam%d state:%d\n", module->idx, atomic_read(&module->state));
		return 0;
	}

	size_cfg.size = ch_uinfo.src_size;
	size_cfg.trim = trim_dcam;
	ret = CAM_PIPEINE_ISP_IN_PORT_CFG(ch, PORT_ISP_OFFLINE_IN, CAM_PIPELINE_CFG_SIZE, &size_cfg);
	if (ret) {
		goto exit;
	}
	pr_info("cfg size, path trim %d %d %d %d\n", size_cfg.trim.start_x, size_cfg.trim.start_y,
		size_cfg.trim.size_x, size_cfg.trim.size_y);

	size_cfg.size = ch_uinfo.dst_size;
	size_cfg.trim = trim_isp;
	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_SIZE, &size_cfg);
	if (ret)
		goto exit;
	pr_info("update channel size done for CAP\n");

exit:
	return ret;
}

int cam_zoom_channel_bigsize_config(
	struct camera_module *module, struct channel_context *channel)
{
	int ret = 0;
	struct camera_uchannel *ch_uinfo = NULL;
	struct dcam_path_cfg_param ch_desc = {0};

	ch_uinfo = &channel->ch_uinfo;
	/* dcam1 cfg path size */
	memset(&ch_desc, 0, sizeof(ch_desc));
	ch_desc.input_size.w = ch_uinfo->src_size.w;
	ch_desc.input_size.h = ch_uinfo->src_size.h;
	ch_desc.output_size = ch_desc.input_size;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	ch_desc.is_4in1 = module->cam_uinfo.is_4in1;
	if (module->cam_uinfo.is_pyr_dec) {
		ch_desc.input_size.w = ch_uinfo->src_crop.w;
		ch_desc.input_size.h = ch_uinfo->src_crop.h;
		ch_desc.input_trim.start_x = 0;
		ch_desc.input_trim.start_y = 0;
		ch_desc.input_trim.size_x = ch_uinfo->src_crop.w;
		ch_desc.input_trim.size_y = ch_uinfo->src_crop.h;
		ch_desc.output_size.w = ch_uinfo->src_crop.w;
		ch_desc.output_size.h = ch_uinfo->src_crop.h;
	}

	pr_info("update dcam port %d size for channel %d 4in1 %d\n",
		channel->aux_dcam_port_id, channel->ch_id, ch_desc.is_4in1);

	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(channel, channel->aux_dcam_port_id,
		CAM_PIPELINE_CFG_SIZE, &ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE);

	pr_info("update channel size done for CAP\n");
	return ret;
}

int cam_zoom_start_proc(void *param)
{
	int update_pv = 0, update_c = 0, ret = 0;
	struct camera_module *module = NULL;
	struct channel_context *ch_prev = NULL, *ch_vid = NULL, *ch_cap = NULL;
	struct camera_zoom_frame pre_zoom_coeff = {0};
	struct camera_zoom_frame vid_zoom_coeff = {0};
	struct camera_zoom_frame cap_zoom_coeff = {0};

	module = (struct camera_module *)param;
	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
next:
	update_pv = update_c = 0;
	/* Get node from the preview/video/cap coef queue if exist */
	if (ch_prev->enable)
		ret = cam_queue_dequeue_new(&ch_prev->zoom_user_crop_q, struct camera_zoom_frame, list, &pre_zoom_coeff);
	if (!ret) {
		ch_prev->ch_uinfo.src_crop = pre_zoom_coeff.zoom_crop;
		ch_prev->ch_uinfo.total_src_crop = pre_zoom_coeff.total_zoom_crop;
		update_pv |= 1;
	}

	if (ch_vid->enable)
		ret = cam_queue_dequeue_new(&ch_vid->zoom_user_crop_q, struct camera_zoom_frame, list, &vid_zoom_coeff);
	if (!ret) {
		ch_vid->ch_uinfo.src_crop = vid_zoom_coeff.zoom_crop;
		update_pv |= 1;
	}

	if (ch_cap->enable)
		ret = cam_queue_dequeue_new(&ch_cap->zoom_user_crop_q, struct camera_zoom_frame, list, &cap_zoom_coeff);
	if (!ret) {
		ch_cap->ch_uinfo.src_crop = cap_zoom_coeff.zoom_crop;
		update_c |= 1;
	}

	if (update_pv || update_c) {
		cam_zoom_channel_size_calc(module);
		if (ch_cap->enable && update_c) {
			mutex_lock(&module->zoom_lock);
			cam_zoom_channel_size_config(module, ch_cap);
			mutex_unlock(&module->zoom_lock);
		}
		if (ch_prev->enable && update_pv) {
			mutex_lock(&module->zoom_lock);
			cam_zoom_channel_size_config(module, ch_prev);
			mutex_unlock(&module->zoom_lock);
		}
		goto next;
	}

	return 0;
}

int cam_zoom_stream_state_get(struct isp_node *node, struct camera_frame *pframe)
{
	int ret = 0,i = 0, hw_path_id = 0;
	uint32_t normal_cnt = 0, postproc_cnt = 0;
	uint32_t maxw = 0, maxh = 0, scl_x = 0, scl_w = 0, scl_h = 0;
	uint32_t min_crop_w = 0xFFFFFFFF, min_crop_h = 0xFFFFFFFF;
	struct isp_stream_ctrl tmp_stream[5] = {0};
	struct isp_port *port = NULL;

	if (!node || !pframe) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	normal_cnt = 1;
	list_for_each_entry(port, &node->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
			maxw = MAX(maxw, port->size.w);
			maxh = MAX(maxh, port->size.h);
			min_crop_w = MIN(min_crop_w, port->trim.size_x);
			min_crop_h = MIN(min_crop_h, port->trim.size_y);
		}
	}
	if (!maxw || !maxh) {
		pr_err("fail to get valid max dst size\n");
		return -EFAULT;
	}

	scl_w = maxw / min_crop_w;
	if ((maxw % min_crop_w) != 0)
		scl_w ++;
	scl_h = maxh / min_crop_h;
	if ((maxh % min_crop_h) != 0)
		scl_h ++;
	scl_x = MAX(scl_w, scl_h);
	pr_debug("scl_x %d scl_w %d scl_h %d max_w %d max_h %d\n", scl_x,
		scl_w, scl_h, maxw, maxh);
	do {
		if (scl_x == ISP_SCALER_UP_MAX)
			break;
		scl_x = scl_x / ISP_SCALER_UP_MAX;
		if (scl_x)
			postproc_cnt++;
	} while (scl_x);

	/* If need postproc, first normal proc need to mark as postproc too*/
	if (postproc_cnt) {
		postproc_cnt++;
		normal_cnt--;
	}

	for (i = postproc_cnt - 1; i >= 0; i--) {
		maxw = maxw / ISP_SCALER_UP_MAX;
		maxw = ISP_ALIGN_W(maxw);
		maxh = maxh / ISP_SCALER_UP_MAX;
		maxh = ISP_ALIGN_H(maxh);

		if (i == 0) {
			list_for_each_entry(port, &node->port_queue.head, list)
			if (port->type == PORT_TRANSFER_IN) {
				tmp_stream[i].in_fmt = port->fmt;
				tmp_stream[i].in = port->size;
				tmp_stream[i].in_crop = port->trim;
			}
		} else {
			tmp_stream[i].in_fmt = CAM_YVU420_2FRAME;
			tmp_stream[i].in.w = maxw;
			tmp_stream[i].in.h = maxh;
			tmp_stream[i].in_crop.start_x = 0;
			tmp_stream[i].in_crop.start_y = 0;
			tmp_stream[i].in_crop.size_x = maxw;
			tmp_stream[i].in_crop.size_y = maxh;
		}

		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				/* This is for ensure the last postproc frame buf is OUT for user
				.* Then the frame before should be POST. Thus, only one post
				.* buffer is enough for all the isp postproc process */
				if ((postproc_cnt + i) % 2 == 1)
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_OUT;
				else
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_POSTPROC;
				if (hw_path_id != ISP_SPATH_CP && (i != postproc_cnt - 1))
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_RESERVED;
				if (i == (postproc_cnt - 1)) {
					tmp_stream[i].out[hw_path_id] = port->size;
					tmp_stream[i].out_crop[hw_path_id].start_x = 0;
					tmp_stream[i].out_crop[hw_path_id].start_y = 0;
					tmp_stream[i].out_crop[hw_path_id].size_x = maxw;
					tmp_stream[i].out_crop[hw_path_id].size_y = maxh;
				} else if (i == 0) {
					tmp_stream[i].out[hw_path_id].w = tmp_stream[i + 1].in.w;
					tmp_stream[i].out[hw_path_id].h = tmp_stream[i + 1].in.h;
					tmp_stream[i].out_crop[hw_path_id] = port->trim;
				} else {
					tmp_stream[i].out[hw_path_id].w = tmp_stream[i + 1].in.w;
					tmp_stream[i].out[hw_path_id].h = tmp_stream[i + 1].in.h;
					tmp_stream[i].out_crop[hw_path_id].start_x = 0;
					tmp_stream[i].out_crop[hw_path_id].start_y = 0;
					tmp_stream[i].out_crop[hw_path_id].size_x = maxw;
					tmp_stream[i].out_crop[hw_path_id].size_y = maxh;
				}
				pr_debug("isp %d i %d hw_path_id %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, i, hw_path_id, tmp_stream[i].out[hw_path_id].w, tmp_stream[i].out[hw_path_id].h,
					tmp_stream[i].out_crop[hw_path_id].start_x, tmp_stream[i].out_crop[hw_path_id].start_y,
					tmp_stream[i].out_crop[hw_path_id].size_x, tmp_stream[i].out_crop[hw_path_id].size_y);
			}
		}
		pr_debug("isp %d index %d in_size %d %d crop_szie %d %d %d %d\n",
			node->cfg_id, i, tmp_stream[i].in.w, tmp_stream[i].in.h,
			tmp_stream[i].in_crop.start_x, tmp_stream[i].in_crop.start_y,
			tmp_stream[i].in_crop.size_x, tmp_stream[i].in_crop.size_y);
	}

	/***one scaler out_size***/
	for (i = 0; i < normal_cnt; i++) {
		pframe->state = ISP_STREAM_NORMAL_PROC;
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				pframe->in = port->size;
				pframe->in_crop = port->trim;
				pr_debug("isp %d hw_path_id %d normal_cnt %d in_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, normal_cnt ,pframe->in.w, pframe->in.h,
					pframe->in_crop.start_x, pframe->in_crop.start_y,
					pframe->in_crop.size_x, pframe->in_crop.size_y);
			}
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				pframe->buf_type = ISP_STREAM_BUF_OUT;
				pframe->out[hw_path_id] = port->size;
				pframe->out_crop[hw_path_id] = port->trim;
				pr_debug("isp %d hw_path_id %d normal_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, normal_cnt ,pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);
			}
		}
	}

	/***two scaler out_size***/
	if (postproc_cnt >= 2) {
		pframe->state = ISP_STREAM_POST_PROC;
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				pframe->buf_type = tmp_stream[0].buf_type[hw_path_id];
				if (hw_path_id != ISP_SPATH_CP)
					pframe->buf_type = ISP_STREAM_BUF_RESERVED;
				pframe->out[hw_path_id] = tmp_stream[0].out[hw_path_id];
				pframe->out_crop[hw_path_id] = tmp_stream[0].out_crop[hw_path_id];
				pr_debug("isp %d hw_path_id %d postproc_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, postproc_cnt, pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);

			}
		}
	}

	/***3dnr cap or 3dnr cap 10x out_size***/
	if (node->uinfo.mode_3dnr == MODE_3DNR_CAP) {
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				if (postproc_cnt && node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					pframe->buf_type = ISP_STREAM_BUF_POSTPROC;
				else if (node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					pframe->buf_type = ISP_STREAM_BUF_OUT;
				else
					pframe->buf_type = ISP_STREAM_BUF_RESERVED;
				pframe->nr3_blend_cnt = node->nr3_blend_cnt;
				if (node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					node->nr3_blend_cnt = 0;
				node->nr3_blend_cnt++;
				pframe->out[hw_path_id] = port->size;
				pframe->out_crop[hw_path_id] = port->trim;
				if (postproc_cnt) {
					pframe->out[hw_path_id] = tmp_stream[0].out[hw_path_id];
					pframe->out_crop[hw_path_id] = tmp_stream[0].out_crop[hw_path_id];
				}
				pr_debug("isp %d hw_path_id %d postproc_cnt %d node->nr3_blend_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, postproc_cnt, node->nr3_blend_cnt, pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);
			}
		}
	}
	return ret;
}

int cam_zoom_param_set(struct cam_zoom_desc *in_ptr)
{
	int ret = 0, i = 0;
	unsigned long flag = 0;
	struct cam_zoom_frame cur_zoom = {0};
	struct cam_zoom_frame *latest_zoom = NULL;
	struct camera_queue *zoom_q = NULL;

	if (!in_ptr) {
		pr_err("fail to get valid ptr\n");
		return -EFAULT;
	}

	latest_zoom = in_ptr->latest_zoom_info;
	zoom_q = in_ptr->zoom_info_q;
	if (!latest_zoom || !zoom_q || !in_ptr->pipeline_graph) {
		pr_err("fail to get valid %px %px\n", latest_zoom, zoom_q);
		return -EFAULT;
	}

	if (in_ptr->pipeline_type < CAM_PIPELINE_TYPE_MAX) {
		camzoom_frame_param_cfg(in_ptr, &cur_zoom);
		pr_debug("set cur_zoom %px\n", cur_zoom);
		ret = cam_queue_enqueue_new(zoom_q, struct cam_zoom_frame, &cur_zoom);
		if (ret) {
			pr_err("fail to enq zoom cnt %d, state:%d\n", zoom_q->cnt, zoom_q->state);
			return ret;
		}
		spin_lock_irqsave(in_ptr->zoom_lock, flag);
		for (i = 0; i < NODE_ZOOM_CNT_MAX; i++)
			latest_zoom->zoom_node[i] = cur_zoom.zoom_node[i];
		spin_unlock_irqrestore(in_ptr->zoom_lock, flag);
	} else {
		pr_err("fail to support pipeline type %d\n", in_ptr->pipeline_type);
	}

	return ret;
}

int cam_zoom_frame_base_get(struct cam_zoom_base *zoom_base, struct cam_zoom_index *zoom_index)
{
	int ret = 0, i = 0, j = 0;
	struct cam_zoom_frame *cur_zoom = NULL;
	struct cam_zoom_node *zoom_node = NULL;
	struct cam_zoom_port *zoom_port = NULL;

	if (!zoom_base || !zoom_index) {
		pr_err("fail to get valid param %px %px\n", zoom_base, zoom_index);
		return -EFAULT;
	}

	cur_zoom = (struct cam_zoom_frame *)zoom_index->zoom_data;
	for (i = 0; i < NODE_ZOOM_CNT_MAX; i++) {
		zoom_node = &cur_zoom->zoom_node[i];
		if (zoom_index->node_type == zoom_node->node_type
			&& zoom_index->node_id == zoom_node->node_id) {
			for (j = 0; j < PORT_ZOOM_CNT_MAX; j++) {
				zoom_port = &zoom_node->zoom_port[j];
				if (zoom_index->port_id == zoom_port->port_id
					&& zoom_index->port_type == zoom_port->port_type) {
					zoom_base->src = zoom_port->zoom_base.src;
					zoom_base->crop = zoom_port->zoom_base.crop;
					zoom_base->dst = zoom_port->zoom_base.dst;
					zoom_base->ratio_width = zoom_port->zoom_base.ratio_width;
					zoom_base->total_crop_width = zoom_port->zoom_base.total_crop_width;
					break;
				}
			}
			break;
		}
	}

	if (i == NODE_ZOOM_CNT_MAX && j == PORT_ZOOM_CNT_MAX)
		pr_warn("warning: not find correct zoom data\n");

	return ret;
}
