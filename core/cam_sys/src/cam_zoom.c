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
	output_stg = module->grp->hw_info->ip_dcam[0]->dcamhw_abt->output_strategy;

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
	output_stg = module->grp->hw_info->ip_dcam[0]->dcamhw_abt->output_strategy;

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

static int camzoom_scaler_node_need(struct cam_zoom_base *zoom_base,
	struct cam_zoom_desc *zoom_info, uint32_t port_id)
{
	struct img_size dst = {0};
	struct img_trim crop = {0};

	crop = zoom_base->crop;
	dst = zoom_base->dst;
	if (dst.w > (crop.size_x * ISP_SCALER_UP_MAX) || dst.h > (crop.size_y * ISP_SCALER_UP_MAX)) {
		/*TBD: isp more than two scaler scenes need to confirm*/
		dst.w = dst.w / ISP_SCALER_UP_MAX;
		dst.w = ALIGN_DOWN(dst.w, ISP_PIXEL_ALIGN_WIDTH);
		dst.h = dst.h / ISP_SCALER_UP_MAX;
		dst.h = ALIGN_DOWN(dst.h, ISP_PIXEL_ALIGN_HEIGHT);
		/* update current isp node dst size */
		zoom_base->dst.w = dst.w;
		zoom_base->dst.h = dst.h;
		/* update next scaler node trim size */
		zoom_info->isp_src_size.w = zoom_base->dst.w;
		zoom_info->isp_src_size.h = zoom_base->dst.h;
		zoom_info->isp_crop[port_id].start_x = 0;
		zoom_info->isp_crop[port_id].start_y = 0;
		zoom_info->isp_crop[port_id].size_x = zoom_base->dst.w;
		zoom_info->isp_crop[port_id].size_y = zoom_base->dst.h;
		CAM_ZOOM_DEBUG("isp port %d size update old %d %d new %d %d\n",
			port_id, zoom_info->isp_dst[port_id].w, zoom_info->isp_dst[port_id].h, zoom_base->dst.w, zoom_base->dst.h);

		return 1;
	}
	return 0;
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
			CAM_ZOOM_DEBUG("%s: dcam port %d size %d %d trim %d %d %d %d dst %d %d\n",
				cam_node_name_get(node_type), zoom_port->port_id, zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		if (zoom_port->port_type == PORT_TRANSFER_IN) {
			zoom_base->src = param->isp_src_size;
			/* node input no need crop same with pre node output */
			zoom_base->crop.start_x = 0;
			zoom_base->crop.start_y = 0;
			zoom_base->crop.size_x = zoom_base->src.w;
			zoom_base->crop.size_y = zoom_base->src.h;
			zoom_base->dst = zoom_base->src;
			valid = 1;
			CAM_ZOOM_DEBUG("%s: isp size %d %d trim %d %d %d %d dst %d %d\n",
				cam_node_name_get(node_type), zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		if (zoom_port->port_type == PORT_TRANSFER_OUT &&
			(zoom_port->port_id == PORT_PRE_OUT || zoom_port->port_id == PORT_VID_OUT
			|| zoom_port->port_id == PORT_CAP_OUT || zoom_port->port_id == PORT_THUMB_OUT)) {
			zoom_base->src = param->isp_src_size;
			zoom_base->crop = param->isp_crop[zoom_port->port_id];
			zoom_base->dst = param->isp_dst[zoom_port->port_id];
			if (camzoom_scaler_node_need(zoom_base, param, zoom_port->port_id))
				zoom_base->need_post_proc = ISP_STREAM_POST_PROC;
			valid = 1;
			CAM_ZOOM_DEBUG("%s: isp id %d size %d %d trim %d %d %d %d dst %d %d\n",
				cam_node_name_get(node_type), zoom_port->port_id, zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
	case CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB:
	case CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV:
		if (zoom_port->port_id == PORT_OFFLINE_BIN_OUT ||
			zoom_port->port_id == PORT_OFFLINE_RAW_OUT ||
			zoom_port->port_id == PORT_OFFLINE_FULL_OUT) {
			zoom_base->src = param->dcam_src_size;
			zoom_base->crop = param->dcam_crop[zoom_port->port_id];
			zoom_base->dst = param->dcam_dst[zoom_port->port_id];
			valid = 1;
			CAM_ZOOM_DEBUG("%s: port_id %d size %d %d trim %d %d %d %d dst %d %d\n",
				cam_node_name_get(node_type), zoom_port->port_id, zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_PYR_DEC:
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

	module->zoom_solution = module->grp->hw_info->ip_dcam[0]->dcamhw_abt->dcam_zoom_mode;
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
	uint32_t need_raw_port = 0, raw_port_id = 0, raw2yuv_port_id = 0;
	struct channel_context *ch_vid = NULL;
	struct channel_context *ch_vir = NULL;
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
	ch_vir = &module->channel[CAM_CH_VIRTUAL];
	hw = module->grp->hw_info;

	raw_port_id = hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id;
	raw_port_id = dcamonline_pathid_convert_to_portid(raw_port_id);
	raw2yuv_port_id = hw->ip_dcam[0]->dcamhw_abt->aux_dcam_path;
	raw2yuv_port_id = dcamoffline_pathid_convert_to_portid(raw2yuv_port_id);
	raw_zoom_base.src = ch_uinfo->src_size;
	raw_zoom_base.dst = ch_uinfo->src_size;
	raw_zoom_base.crop.start_x = 0;
	raw_zoom_base.crop.start_y = 0;
	raw_zoom_base.crop.size_x = ch_uinfo->src_size.w;
	raw_zoom_base.crop.size_y = ch_uinfo->src_size.h;
	if (channel->ch_id == CAM_CH_RAW || channel->dcam_port_id == PORT_RAW_OUT
		|| module->cam_uinfo.alg_type || module->cam_uinfo.is_4in1) {
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
	zoom_info.dcam_src_size = zoom_info.sn_rect_size;
	if (IS_VALID_DCAM_IMG_PORT(channel->dcam_port_id)) {
		zoom_info.dcam_crop[channel->dcam_port_id] = channel->trim_dcam;
		zoom_info.dcam_dst[channel->dcam_port_id] = channel->dst_dcam;
	}

	if (need_raw_port && IS_VALID_DCAM_IMG_PORT(raw_port_id)) {
		zoom_info.dcam_crop[raw_port_id] = raw_zoom_base.crop;
		zoom_info.dcam_dst[raw_port_id] = raw_zoom_base.dst;
		if ((module->cam_uinfo.is_4in1 || module->cam_uinfo.alg_type ||
			(channel->ch_id == CAM_CH_CAP && module->cam_uinfo.dcam_slice_mode && !module->cam_uinfo.is_4in1))
			&& IS_VALID_DCAM_IMG_PORT(raw2yuv_port_id)) {
			zoom_info.dcam_crop[raw2yuv_port_id] = channel->trim_dcam;
			zoom_info.dcam_dst[raw2yuv_port_id] = channel->dst_dcam;
		}
	}

	zoom_info.isp_src_size = channel->dst_dcam;
	if (IS_VALID_ISP_IMG_PORT(channel->isp_port_id)) {
		zoom_info.isp_crop[channel->isp_port_id] = channel->trim_isp;
		zoom_info.isp_dst[channel->isp_port_id] = channel->ch_uinfo.dst_size;
		CAM_ZOOM_DEBUG("ch %d trim isp %d %d, dst %d %d\n", channel->ch_id, channel->trim_isp.size_x,
			channel->trim_isp.size_y, channel->ch_uinfo.dst_size.w, channel->ch_uinfo.dst_size.h);
	}
	if (ch_vid->enable) {
		zoom_info.isp_crop[PORT_VID_OUT] = ch_vid->trim_isp;
		zoom_info.isp_dst[PORT_VID_OUT] = ch_vid->ch_uinfo.dst_size;
	}
	if (ch_vir->enable && channel->ch_id == CAM_CH_PRE) {
		zoom_info.isp_crop[PORT_VID_OUT].size_x = ch_vir->ch_uinfo.vir_channel[0].dst_size.w;
		zoom_info.isp_crop[PORT_VID_OUT].size_y = ch_vir->ch_uinfo.vir_channel[0].dst_size.h;
		zoom_info.isp_dst[PORT_VID_OUT].w = ch_vir->ch_uinfo.vir_channel[0].dst_size.w;
		zoom_info.isp_dst[PORT_VID_OUT].h = ch_vir->ch_uinfo.vir_channel[0].dst_size.h;
	}
	if (ch_vir->enable && channel->ch_id == CAM_CH_CAP) {
		zoom_info.isp_crop[PORT_VID_OUT].size_x = ch_vir->ch_uinfo.vir_channel[1].dst_size.w;
		zoom_info.isp_crop[PORT_VID_OUT].size_y = ch_vir->ch_uinfo.vir_channel[1].dst_size.h;
		zoom_info.isp_dst[PORT_VID_OUT].w = ch_vir->ch_uinfo.vir_channel[1].dst_size.w;
		zoom_info.isp_dst[PORT_VID_OUT].h = ch_vir->ch_uinfo.vir_channel[1].dst_size.h;
	}

	ret = cam_zoom_param_set(&zoom_info);

	return ret;
}

int cam_zoom_start_proc(void *param)
{
	int update_pv = 0, update_c = 0;
	struct camera_module *module = NULL;
	struct channel_context *ch_prev = NULL, *ch_vid = NULL, *ch_cap = NULL;
	struct cam_frame *pre_zoom_coeff = NULL;
	struct cam_frame *vid_zoom_coeff = NULL;
	struct cam_frame *cap_zoom_coeff = NULL;

	module = (struct camera_module *)param;
	ch_prev = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
next:
	update_pv = update_c = 0;
	/* Get node from the preview/video/cap coef queue if exist */
	if (ch_prev->enable)
		pre_zoom_coeff = CAM_QUEUE_DEQUEUE(&ch_prev->zoom_user_crop_q, struct cam_frame, list);
	if (pre_zoom_coeff) {
		ch_prev->ch_uinfo.src_crop = pre_zoom_coeff->user_zoom.zoom_crop;
		ch_prev->ch_uinfo.total_src_crop = pre_zoom_coeff->user_zoom.total_zoom_crop;
		update_pv |= 1;
		cam_queue_empty_frame_put(pre_zoom_coeff);
	}

	if (ch_vid->enable)
		vid_zoom_coeff = CAM_QUEUE_DEQUEUE(&ch_vid->zoom_user_crop_q, struct cam_frame, list);
	if (vid_zoom_coeff) {
		ch_vid->ch_uinfo.src_crop = vid_zoom_coeff->user_zoom.zoom_crop;
		update_pv |= 1;
		cam_queue_empty_frame_put(vid_zoom_coeff);
	}

	if (ch_cap->enable)
		cap_zoom_coeff = CAM_QUEUE_DEQUEUE(&ch_cap->zoom_user_crop_q, struct cam_frame, list);
	if (cap_zoom_coeff) {
		ch_cap->ch_uinfo.src_crop = cap_zoom_coeff->user_zoom.zoom_crop;
		update_c |= 1;
		cam_queue_empty_frame_put(cap_zoom_coeff);
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

int cam_zoom_param_set(struct cam_zoom_desc *in_ptr)
{
	int ret = 0, i = 0;
	unsigned long flag = 0;
	struct cam_frame *cur_zoom = NULL;
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
		cur_zoom = cam_queue_empty_frame_get(CAM_FRAME_NODE_ZOOM);
		if (cur_zoom) {
			camzoom_frame_param_cfg(in_ptr, &cur_zoom->node_zoom);
			pr_debug("set cur_zoom %px\n", cur_zoom);
			ret = CAM_QUEUE_ENQUEUE(zoom_q, &cur_zoom->list);
			if (ret) {
				pr_err("fail to enq zoom cnt %d, state:%d\n", zoom_q->cnt, zoom_q->state);
				return ret;
			}
			spin_lock_irqsave(in_ptr->zoom_lock, flag);
			for (i = 0; i < NODE_ZOOM_CNT_MAX; i++)
				latest_zoom->zoom_node[i] = cur_zoom->node_zoom.zoom_node[i];
			spin_unlock_irqrestore(in_ptr->zoom_lock, flag);
		}
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

	cur_zoom = zoom_index->zoom_data;
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
					zoom_base->need_post_proc = zoom_port->zoom_base.need_post_proc;
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
