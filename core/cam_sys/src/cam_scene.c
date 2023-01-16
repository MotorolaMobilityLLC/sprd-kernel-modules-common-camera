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

#include "cam_core.h"
#include "cam_scene.h"

static int camscene_cap_info_set(struct camera_module *module,
		struct dcam_mipi_info *cap_info)
{
	int ret = 0;
	struct camera_uinfo *info = &module->cam_uinfo;
	struct sprd_img_sensor_if *sensor_if = &info->sensor_if;

	cap_info->mode = info->capture_mode;
	cap_info->frm_skip = info->capture_skip;
	cap_info->is_4in1 = info->is_4in1;
	cap_info->dcam_slice_mode = info->dcam_slice_mode;
	cap_info->sensor_if = sensor_if->if_type;
	cap_info->format = sensor_if->img_fmt;
	cap_info->pattern = sensor_if->img_ptn;
	cap_info->frm_deci = sensor_if->frm_deci;
	cap_info->is_cphy = sensor_if->if_spec.mipi.is_cphy;
	if (cap_info->sensor_if == DCAM_CAP_IF_CSI2) {
		cap_info->href = sensor_if->if_spec.mipi.use_href;
		cap_info->data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
	}
	cap_info->cap_size.start_x = info->sn_rect.x;
	cap_info->cap_size.start_y = info->sn_rect.y;
	cap_info->cap_size.size_x = info->sn_rect.w;
	cap_info->cap_size.size_y = info->sn_rect.h;

	return ret;
}

int cam_scene_reserved_buf_cfg(enum reserved_buf_cb_type type,
		void *param, void *priv_data)
{
	int ret = 0, j = 0;
	struct camera_frame *pframe = NULL;
	struct camera_frame **frame = NULL;
	struct camera_frame *newfrm = NULL;
	struct camera_module *module = NULL;
	struct cam_buf_pool_id pool_id = {0};
	if (!priv_data) {
		pr_err("fail to get valid param %p\n", priv_data);
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	pool_id.reserved_pool_id = module->reserved_pool_id;
	switch (type) {
	case RESERVED_BUF_GET_CB:
		frame = (struct camera_frame **)param;
		do {
			*frame = cam_buf_manager_buf_dequeue(&pool_id, NULL);
			if (*frame == NULL) {
				pframe = module->res_frame;
				while (j < CAM_RESERVE_BUF_Q_LEN) {
					newfrm = cam_queue_empty_frame_get();
					if (newfrm) {
						newfrm->is_reserved = CAM_RESERVED_BUFFER_COPY;
						newfrm->channel_id = pframe->channel_id;
						newfrm->pyr_status = pframe->pyr_status;
						newfrm->user_fid = pframe->user_fid;
						memcpy(&newfrm->buf, &pframe->buf, sizeof(struct camera_buf));
						newfrm->buf.type = CAM_BUF_NONE;
						cam_buf_manager_buf_enqueue(&pool_id, newfrm, NULL);
						j++;
					}
				}
			}
		} while (*frame == NULL);

		pr_debug("Done. cam %d get reserved_pool_id %d frame %p buf_info %p\n", module->idx, pool_id.reserved_pool_id, *frame, (*frame)->buf);
		break;
	case RESERVED_BUF_SET_CB:
		pframe = (struct camera_frame *)param;
		if (pframe->is_reserved) {
			pframe->priv_data = NULL;
			ret = cam_buf_manager_buf_enqueue(&pool_id, pframe, NULL);
			pr_debug("cam %d dcam %d set reserved_pool_id %d frame id %d\n", module->idx, pool_id.reserved_pool_id, pframe->fid);
		}
		break;
	default:
		pr_err("fail to get invalid %d\n", type);
		break;
	}

	return ret;
}

uint32_t cam_scene_dcamonline_buffers_alloc_num(void *channel_ptr, void *module_ptr, uint32_t pipeline_type)
{
	uint32_t num = 0;
	struct channel_context *channel = NULL;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	grp = module->grp;

	num = module->static_topology->pipeline_list[pipeline_type].buf_num;

	if (channel->ch_id == CAM_CH_CAP) {
		channel->zsl_skip_num = module->cam_uinfo.zsk_skip_num;
		channel->zsl_buffer_num = module->cam_uinfo.zsl_num;
		num += channel->zsl_buffer_num;
	}

	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_dual)
		num = CAM_DUAL_ALLOC_BUF_NUM;

	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_dual && module->master_flag) /*for dual hdr*/
		num += 3;

	if (module->cam_uinfo.dcam_slice_mode
		&& channel->ch_id == CAM_CH_CAP &&
		module->channel[CAM_CH_PRE].enable == 0 &&
		module->channel[CAM_CH_VID].enable == 0)
		num = 1;

	if (module->cam_uinfo.is_4in1)
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && (grp->is_mul_buf_share && atomic_inc_return(&grp->mul_buf_alloced) > 1))
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && ((module->cam_uinfo.param_frame_sync ||
		module->cam_uinfo.raw_alg_type != RAW_ALG_AI_SFNR) && module->cam_uinfo.is_raw_alg))
		num = 0;

	/* extend buffer queue for slow motion */
	if (channel->ch_uinfo.is_high_fps)
		num = channel->ch_uinfo.high_fps_skip_num * 4;

	return num;
}

uint32_t cam_scene_dcamoffline_buffers_alloc_num(void *channel_ptr, void *module_ptr)
{
	uint32_t num = 1;
	struct channel_context *channel = NULL;
	struct camera_module *module = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;

	if (module->cam_uinfo.raw_alg_type == RAW_ALG_AI_SFNR)
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && ((module->cam_uinfo.param_frame_sync ||
		module->cam_uinfo.raw_alg_type != RAW_ALG_AI_SFNR) && module->cam_uinfo.is_raw_alg))
		num = 0;

	return num;
}

int cam_scene_dcamonline_desc_get(void *module_ptr, void *channel_ptr, uint32_t pipeline_type,
		struct dcam_online_node_desc *dcam_online_desc)
{
	int ret = 0, i = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_port_topology *outport_graph = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	dcam_online_desc->blk_pm = &channel->blk_pm;
	dcam_online_desc->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	dcam_online_desc->resbuf_cb_data = module;
	camscene_cap_info_set(module, &dcam_online_desc->cap_info);
	dcam_online_desc->is_4in1 = module->cam_uinfo.is_4in1;
	dcam_online_desc->offline = 0;
	dcam_online_desc->slowmotion_count = channel->ch_uinfo.high_fps_skip_num;
	dcam_online_desc->enable_3dnr = (module->auto_3dnr | channel->uinfo_3dnr);
	dcam_online_desc->dev = module->dcam_dev_handle;
	if ((module->grp->hw_info->prj_id == SHARKL3) && module->cam_uinfo.virtualsensor)
		dcam_online_desc->dcam_idx = 0;
	else
		dcam_online_desc->dcam_idx = module->dcam_idx;
	dcam_online_desc->raw_alg_type = module->cam_uinfo.raw_alg_type;
	dcam_online_desc->param_frame_sync = module->cam_uinfo.param_frame_sync;

	if (module->cam_uinfo.is_pyr_rec && (!channel->ch_uinfo.is_high_fps))
		dcam_online_desc->is_pyr_rec = 1;
	else
		dcam_online_desc->is_pyr_rec = 0;

	for (i = 0; i < PORT_DCAM_OUT_MAX; i++) {
		outport_graph = &module->static_topology->pipeline_list[pipeline_type].nodes[CAM_NODE_TYPE_DCAM_ONLINE].outport[i];
		if (outport_graph->link_state != PORT_LINK_NORMAL)
			continue;
		dcam_online_desc->port_desc[i].raw_src = PROCESS_RAW_SRC_SEL;
		dcam_online_desc->port_desc[i].endian = ENDIAN_LITTLE;
		dcam_online_desc->port_desc[i].bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		dcam_online_desc->port_desc[i].pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;
		dcam_online_desc->port_desc[i].reserved_pool_id = module->reserved_pool_id;
		if (i == PORT_FULL_OUT)
			dcam_online_desc->port_desc[i].share_full_path = module->cam_uinfo.need_share_buf;
		cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0]);
		dcam_online_desc->port_desc[i].dcam_out_fmt = channel->dcam_out_fmt;
		dcam_online_desc->port_desc[i].compress_en = channel->compress_en;
		if (pipeline_type == CAM_PIPELINE_SENSOR_RAW
			|| pipeline_type == CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV
			|| pipeline_type == CAM_PIPELINE_VCH_SENSOR_RAW
			|| pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV
			|| pipeline_type == CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV) {
			dcam_online_desc->port_desc[i].is_raw = 1;
			cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
			dcam_online_desc->port_desc[i].dcam_out_fmt = channel->ch_uinfo.sensor_raw_fmt;
			if (module->cam_uinfo.raw_alg_type)
				dcam_online_desc->port_desc[i].dcam_out_fmt = CAM_RAW_14;
			if (module->cam_uinfo.need_dcam_raw && hw->ip_dcam[0]->bpc_raw_support) {
				dcam_online_desc->port_desc[i].is_raw = 0;
				dcam_online_desc->port_desc[i].raw_src = BPC_RAW_SRC_SEL;
			}
		} else if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV
				|| pipeline_type == CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV
				|| pipeline_type == CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV) {
			if (outport_graph->id == PORT_RAW_OUT) {
				dcam_online_desc->port_desc[i].is_raw = 1;
				dcam_online_desc->port_desc[i].dcam_out_fmt = CAM_RAW_14;
			}
		}
	}

	return ret;
}

int cam_scene_ispoffline_desc_get(void *module_ptr, void *channel_ptr,
	uint32_t pipeline_type, struct isp_node_desc *isp_node_description)
{
	int ret = 0;
	uint32_t format = 0, temp_format = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	format = module->cam_uinfo.sensor_if.img_fmt;
	isp_node_description->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_node_description->resbuf_cb_data = module;

	if (format == DCAM_CAP_MODE_YUV)
		isp_node_description->in_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	else {
		isp_node_description->in_fmt = channel->dcam_out_fmt;
		if (module->cam_uinfo.raw_alg_type && channel->ch_id == CAM_CH_CAP)
			isp_node_description->in_fmt = CAM_YUV420_2FRAME_MIPI;
	}
	isp_node_description->pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;;
	isp_node_description->store_3dnr_fmt = hw->ip_dcam[0]->store_3dnr_fmt[0];
	temp_format = channel->ch_uinfo.dcam_raw_fmt;
	if (cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0])) {
		if (hw->ip_dcam[0]->save_band_for_bigsize)
			temp_format = CAM_RAW_PACK_10;
	}
	if ((hw->ip_isp->fetch_raw_support == 0) || (format == DCAM_CAP_MODE_YUV))
		channel->ch_uinfo.dcam_out_fmt = isp_node_description->in_fmt;
	else {
		channel->ch_uinfo.dcam_out_fmt = temp_format;
		isp_node_description->in_fmt = temp_format;
	}

	if (channel->compress_en)
		isp_node_description->fetch_path_sel = ISP_FETCH_PATH_FBD;
	else
		isp_node_description->fetch_path_sel = ISP_FETCH_PATH_NORMAL;
	isp_node_description->nr3_fbc_fbd = channel->compress_3dnr;

	isp_node_description->is_dual = module->cam_uinfo.is_dual;
	isp_node_description->bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
	isp_node_description->enable_slowmotion = channel->ch_uinfo.is_high_fps;
	isp_node_description->slowmotion_count = channel->ch_uinfo.high_fps_skip_num;
	isp_node_description->slw_state = CAM_SLOWMOTION_OFF;
	isp_node_description->ch_id = channel->ch_id;
	isp_node_description->sn_size = module->cam_uinfo.sn_size;
	isp_node_description->share_buffer = (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.need_share_buf &&
		!module->cam_uinfo.dcam_slice_mode && !module->cam_uinfo.is_4in1);
	if (channel->ch_uinfo.is_high_fps)
		isp_node_description->blkparam_node_num = CAM_SHARED_BUF_NUM / channel->ch_uinfo.high_fps_skip_num;
	else
		isp_node_description->blkparam_node_num = cam_scene_dcamonline_buffers_alloc_num(channel, module, pipeline_type) + 1;

	isp_node_description->mode_3dnr = MODE_3DNR_OFF;
	channel->type_3dnr = CAM_3DNR_OFF;
	if (module->auto_3dnr || channel->uinfo_3dnr) {
		channel->type_3dnr = CAM_3DNR_HW;
		if (channel->uinfo_3dnr) {
			if (channel->ch_id == CAM_CH_CAP)
				isp_node_description->mode_3dnr = MODE_3DNR_CAP;
			else
				isp_node_description->mode_3dnr = MODE_3DNR_PRE;
		}
	}

	isp_node_description->mode_ltm = MODE_LTM_OFF;
	if (module->cam_uinfo.is_rgb_ltm) {
		if (channel->ch_id == CAM_CH_CAP) {
			isp_node_description->mode_ltm = MODE_LTM_CAP;
		} else if (channel->ch_id == CAM_CH_PRE) {
			isp_node_description->mode_ltm = MODE_LTM_PRE;
		}
	}
	channel->mode_ltm = isp_node_description->mode_ltm;

	isp_node_description->mode_gtm = MODE_GTM_OFF;
	if (module->cam_uinfo.is_rgb_gtm) {
		if (channel->ch_id == CAM_CH_CAP) {
			isp_node_description->mode_gtm = MODE_GTM_CAP;
		} else if (channel->ch_id == CAM_CH_PRE) {
			isp_node_description->mode_gtm = MODE_GTM_PRE;
		}
	}
	channel->mode_gtm = isp_node_description->mode_gtm;

	isp_node_description->ltm_rgb = module->cam_uinfo.is_rgb_ltm;
	channel->ltm_rgb = isp_node_description->ltm_rgb;
	isp_node_description->gtm_rgb = module->cam_uinfo.is_rgb_gtm;
	channel->gtm_rgb = isp_node_description->gtm_rgb;
	isp_node_description->is_high_fps = channel->ch_uinfo.is_high_fps;
	isp_node_description->cam_id = module->idx;
	isp_node_description->dev = module->isp_dev_handle;
	isp_node_description->port_desc.out_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	isp_node_description->port_desc.endian = ENDIAN_LITTLE;
	isp_node_description->port_desc.output_size = channel->ch_uinfo.dst_size;
	isp_node_description->port_desc.regular_mode = channel->ch_uinfo.regular_desc.regular_mode;
	isp_node_description->port_desc.resbuf_get_cb = isp_node_description->resbuf_get_cb;
	isp_node_description->port_desc.resbuf_cb_data = isp_node_description->resbuf_cb_data;
	isp_node_description->port_desc.in_fmt = isp_node_description->in_fmt;
	isp_node_description->port_desc.bayer_pattern = isp_node_description->bayer_pattern;
	isp_node_description->port_desc.pyr_out_fmt = isp_node_description->pyr_out_fmt;
	isp_node_description->port_desc.store_3dnr_fmt = isp_node_description->store_3dnr_fmt;
	isp_node_description->port_desc.sn_size = isp_node_description->sn_size;
	isp_node_description->port_desc.is_high_fps = isp_node_description->is_high_fps;
	isp_node_description->port_desc.fetch_path_sel = isp_node_description->fetch_path_sel;
	isp_node_description->port_desc.hw = hw;
	if (channel->ch_uinfo.slave_img_en) {
		isp_node_description->port_desc.slave_type = ISP_PATH_MASTER;
		isp_node_description->port_desc.slave_path_id = ISP_SPATH_VID;
	}

	if (channel->ch_uinfo.is_high_fps) {
		isp_node_description->fps960_info.slowmotion_stage_a_num = channel->ch_uinfo.slowmotion_stage_a_num;
		isp_node_description->fps960_info.slowmotion_stage_a_valid_num = channel->ch_uinfo.slowmotion_stage_a_valid_num;
		isp_node_description->fps960_info.slowmotion_stage_b_num = channel->ch_uinfo.slowmotion_stage_b_num;
	}
	return ret;
}

int cam_scene_dcamoffline_desc_get(void *module_ptr, void *channel_ptr,
	uint32_t pipeline_type, struct dcam_offline_node_desc *dcam_offline_desc)
{
	int ret = 0;
	uint32_t dcam_idx = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;

	for (dcam_idx = 0; dcam_idx < DCAM_HW_CONTEXT_MAX; dcam_idx++) {
		if (dcam_idx != module->dcam_idx) {
			dcam_offline_desc->dcam_idx = dcam_idx;
			break;
		}
	}
	dcam_offline_desc->dev = module->dcam_dev_handle;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->port_desc.src_sel = PROCESS_RAW_SRC_SEL;
	cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = channel->ch_uinfo.sensor_raw_fmt;
	cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = channel->ch_uinfo.dcam_raw_fmt;
	if (module->cam_uinfo.dcam_slice_mode)
		dcam_offline_desc->port_desc.compress_en = channel->compress_offline;
	if (module->cam_uinfo.dcam_slice_mode && hw->ip_dcam[0]->save_band_for_bigsize)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_PACK_10;
	if (hw->ip_isp->fetch_raw_support == 0)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;
	channel->dcam_out_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV) {
		if (module->cam_uinfo.raw_alg_type)
			dcam_offline_desc->fetch_fmt = CAM_RAW_14;
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;
	} else if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV) {
		if (module->cam_uinfo.raw_alg_type) {
			dcam_offline_desc->fetch_fmt = CAM_RAW_14;
			dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;
		}
		if (module->cam_uinfo.is_4in1)
			dcam_offline_desc->fetch_fmt = channel->ch_uinfo.dcam_raw_fmt;
	}
	channel->ch_uinfo.dcam_raw_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;

	return ret;
}

int cam_scene_dcamoffline_bpcraw_desc_get(void *module_ptr,
		void *channel_ptr, struct dcam_offline_node_desc *dcam_offline_desc)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;

	dcam_offline_desc->dev = module->dcam_dev_handle;
	dcam_offline_desc->dcam_idx = DCAM_HW_CONTEXT_1;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->port_desc.src_sel = BPC_RAW_SRC_SEL;
	cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = CAM_RAW_14;
	cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_14;

	return ret;
}

int cam_scene_isp_yuv_scaler_desc_get(void *module_ptr,
		void *channel_ptr, struct isp_yuv_scaler_node_desc *isp_yuv_scaler_desc)
{
	int ret = 0;
	uint32_t uframe_sync = 0;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	if (!module_ptr || !channel_ptr || !isp_yuv_scaler_desc) {
		pr_err("fail to get valid inptr %p %p %p\n", module, channel, isp_yuv_scaler_desc);
		return -EINVAL;
	}

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	uframe_sync = channel->ch_id != CAM_CH_CAP;
	if (channel->ch_uinfo.frame_sync_close)
		uframe_sync = 0;
	isp_yuv_scaler_desc->uframe_sync = uframe_sync;
	isp_yuv_scaler_desc->dev = module->isp_dev_handle;
	isp_yuv_scaler_desc->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_yuv_scaler_desc->resbuf_cb_data = module;
	isp_yuv_scaler_desc->ch_id = channel->ch_id;
	isp_yuv_scaler_desc->cam_id = module->idx;
	isp_yuv_scaler_desc->out_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	isp_yuv_scaler_desc->endian = ENDIAN_LITTLE;
	isp_yuv_scaler_desc->output_size = channel->ch_uinfo.dst_size;
	isp_yuv_scaler_desc->regular_mode = channel->ch_uinfo.regular_desc.regular_mode;
	isp_yuv_scaler_desc->port_desc.resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_yuv_scaler_desc->port_desc.resbuf_cb_data = module;

	return ret;
}

int cam_scene_pipeline_need_dcamonline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 1;
		break;
	case CAM_PIPELINE_SCALER_YUV:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_dcamoffline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_dcamoffline_bpcraw(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_ispoffline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
		need = 1;
		break;
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_framecache(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
		need = 1;
		break;
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_pyrdec(uint32_t pipeline_type, uint32_t pyrdec_support)
{
	int need = 0;

	if (!pyrdec_support) {
		pr_debug("no need pyrdec node.\n");
		return need;
	}

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_yuv_scaler(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
		need = 1;
		break;
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_static_linkages_get(struct cam_scene *param, void *hw_info)
{
	int ret = 0;
	struct cam_pipeline_topology *cur_pipeline = NULL;

	if (!param || !hw_info) {
		pr_err("fail to get invalid param ptr %px %px\n", param, hw_info);
		return -EFAULT;
	}

	cur_pipeline = &param->pipeline_list[0];
	ret = cam_pipeline_static_pipelinelist_get(cur_pipeline, &param->pipeline_list_cnt, hw_info);
	if (ret) {
		pr_err("fail to get pipelinelist cnt %d\n", param->pipeline_list_cnt);
		return -EFAULT;
	}

	pr_debug("default pipelinelist cnt %d\n", param->pipeline_list_cnt);
	if (param->pipeline_list_cnt > CAM_SCENE_PIPELINE_NUM)
		pr_warn("warning: pipeline list cnt need to enlarge\n");

	return ret;
}

