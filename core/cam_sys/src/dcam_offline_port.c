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

#include "dcam_offline_port.h"
#include "cam_block.h"
#include "dcam_interface.h"
#include "dcam_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_OFFLINE_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

static int dcamoffline_port_size_cfg(void *handle, void *param)
{
	int ret = 0;
	struct dcam_offline_port *port = NULL;
	struct dcam_path_cfg_param *ch_desc = NULL;
	struct img_size crop_size = {0};
	struct img_size dst_size = {0};

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	port = (struct dcam_offline_port *)handle;
	ch_desc = (struct dcam_path_cfg_param *)param;

	port->in_size = ch_desc->input_size;
	port->in_trim = ch_desc->input_trim;
	port->out_size = ch_desc->output_size;
	port->out_pitch = dcampath_outpitch_get(port->out_size.w, port->out_fmt);
	port->priv_size_data = ch_desc->priv_size_data;

	switch (port->port_id) {
	case PORT_OFFLINE_RAW_OUT:
	case PORT_OFFLINE_FULL_OUT:
		pr_info("port %s path done. in size %d %d out size %d %d\n", cam_port_dcam_offline_out_id_name_get(port->port_id),
			port->in_size.w, port->in_size.h,
			port->out_size.w, port->out_size.h);
		pr_info("trim %d %d %d %d\n", port->in_trim.start_x, port->in_trim.start_y,
			port->in_trim.size_x, port->in_trim.size_y);
		break;
	case PORT_OFFLINE_BIN_OUT:
		crop_size.w = port->in_trim.size_x;
		crop_size.h = port->in_trim.size_y;
		dst_size = port->out_size;
		switch (port->out_fmt) {
			case CAM_YUV_BASE:
			case CAM_YUV422_2FRAME:
			case CAM_YVU422_2FRAME:
			case CAM_YUV420_2FRAME:
			case CAM_YVU420_2FRAME:
			case CAM_YUV420_2FRAME_MIPI:
			case CAM_YVU420_2FRAME_MIPI:
				if ((crop_size.w == dst_size.w) && (crop_size.h == dst_size.h)) {
					port->scaler_sel = DCAM_SCALER_BYPASS;
					break;
				}
				if (dst_size.w > DCAM_SCALER_MAX_WIDTH ||
					port->in_trim.size_x > (dst_size.w * DCAM_SCALE_DOWN_MAX)) {
					pr_err("fail to support scaler, in width %d, out width %d\n",
							port->in_trim.size_x, dst_size.w);
					ret = -1;
				}
				port->scaler_sel = DCAM_SCALER_BY_YUVSCALER;
				port->scaler_info.scaler_factor_in = port->in_trim.size_x;
				port->scaler_info.scaler_factor_out = dst_size.w;
				port->scaler_info.scaler_ver_factor_in = port->in_trim.size_y;
				port->scaler_info.scaler_ver_factor_out = dst_size.h;
				port->scaler_info.scaler_out_width = dst_size.w;
				port->scaler_info.scaler_out_height = dst_size.h;
				port->scaler_info.work_mode = 2;
				port->scaler_info.scaler_bypass = 0;
				ret = cam_scaler_coeff_calc_ex(&port->scaler_info);
				if (ret)
					pr_err("fail to calc scaler coeff\n");
				break;
			case CAM_RAW_PACK_10:
			case CAM_RAW_HALFWORD_10:
			case CAM_RAW_14:
			case CAM_RAW_8:
			case CAM_FULL_RGB14:
				dcampath_bin_scaler_get(crop_size, dst_size, &port->scaler_sel, &port->bin_ratio);
				break;
			default:
				pr_err("fail to get path->out_fmt :%s\n", camport_fmt_name_get(port->out_fmt));
				break;
		}
		break;
	default:
		pr_err("fail to get valid port id %s\n", cam_port_dcam_offline_out_id_name_get(port->port_id));
		break;
	}

	return ret;
}

static int dcamoffline_port_base_cfg(struct dcam_offline_port *port,
		struct dcam_offline_port_desc *port_desc)
{
	int ret = 0;

	if (!port || !port_desc) {
		pr_err("fail to get valid param, port=%p, param=%p.\n", port, port_desc);
		return -EFAULT;
	}

	switch (port->port_id) {
	case PORT_OFFLINE_FULL_OUT:
		port->endian = port_desc->endian;
		port->out_fmt = port_desc->dcam_out_fmt;
		port->src_sel = port_desc->src_sel;
		port->compress_en = port_desc->compress_en;
		atomic_set(&port->is_work, 1);
		break;
	case PORT_OFFLINE_BIN_OUT:
		port->endian = port_desc->endian;
		port->out_fmt = port_desc->dcam_out_fmt;
		port->src_sel = port_desc->src_sel;
		atomic_set(&port->is_work, 1);
		break;
	case PORT_OFFLINE_RAW_OUT:
		port->endian = port_desc->endian;
		port->out_fmt = port_desc->dcam_out_fmt;
		port->src_sel = port_desc->src_sel;
		atomic_set(&port->is_work, 1);
		break;
	default:
		pr_debug("get valid port %s\n", cam_port_dcam_offline_out_id_name_get(port->port_id));
		ret = -EFAULT;
		break;
	}
	pr_info("path: %d, port->out_fmt %s\n", port->port_id, camport_fmt_name_get(port->out_fmt));

	return ret;
}

static void dcamoffline_port_outframe_ret(void *param)
{
	struct camera_frame *frame = NULL;
	struct dcam_offline_port *dcam_port = NULL;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;

	if (frame->is_reserved) {
		dcam_port = (struct dcam_offline_port *)frame->priv_data;
		pr_err("fail to: dcam offline used reserved buf\n");
		/* Need Update it later for offline reserved buf use */
		/* dcam_port->resbuf_get_cb(RESERVED_BUF_SET_CB, frame, dcam_port->resbuf_cb_data); */
	} else {
		cam_buf_iommu_unmap(&frame->buf);
		dcam_port = (struct dcam_offline_port *)frame->priv_data;
		dcam_port->data_cb_func(CAM_CB_DCAM_CLEAR_BUF, frame, dcam_port->data_cb_handle);
	}
}

/* This function may put it into dcam core for hw ctx cfg,
 * or call sub function in dcam core for hw ctx cfg.
 */
static int dcamoffline_port_param_get(void *handle, void *param)
{
	int ret = 0;
	uint32_t path_id = 0;
	struct dcam_offline_port *dcam_port = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct camera_frame *frame = NULL;
	struct dcam_hw_cfg_store_addr *hw_store = NULL;
	struct dcam_hw_path_size *hw_size = NULL;
	struct dcam_hw_path_start *hw_start = NULL;
	struct dcam_hw_fbc_addr *hw_fbc_store = NULL;
	struct dcam_hw_fbc_ctrl *hw_fbc_ctrl = NULL;
	struct compressed_addr fbc_addr = {0};
	struct dcam_compress_cal_para cal_fbc = {0};

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_port = (struct dcam_offline_port *)handle;
	hw_ctx = (struct dcam_hw_context *)param;
	if (atomic_read(&dcam_port->is_work) < 1) {
		pr_debug("dcam offline port %s is not work\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		return 0;
	}

	path_id = dcamoffline_portid_convert_to_pathid(dcam_port->port_id);
	pr_info("dcam offline port %s path id %d\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id), path_id);
	hw_store = &hw_ctx->hw_path[path_id].hw_store;
	hw_size = &hw_ctx->hw_path[path_id].hw_size;
	hw_start = &hw_ctx->hw_path[path_id].hw_start;
	hw_fbc_store = &hw_ctx->hw_path[path_id].hw_fbc_store;
	hw_fbc_ctrl = &hw_ctx->hw_path[path_id].hw_fbc;

	frame = cam_queue_dequeue(&dcam_port->out_buf_queue, struct camera_frame, list);
	if (!frame) {
		pr_err("fail to get dcam offline port %s frame\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		return -EFAULT;
	}

	frame->link_from.port_id = dcam_port->port_id;
	ret = cam_queue_enqueue(&dcam_port->result_queue, &frame->list);
	if (ret) {
		pr_err("fail to enqueue dcam offline port %s frame\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		return -EFAULT;
	}
	if (dcam_port->compress_en) {
		cal_fbc.data_bits = cam_data_bits(dcam_port->out_fmt);
		cal_fbc.fbc_info = &frame->fbc_info;
		cal_fbc.in = frame->buf.iova;
		cal_fbc.fmt = dcam_port->out_fmt;
		cal_fbc.height = dcam_port->out_size.h;
		cal_fbc.width = dcam_port->out_size.w;
		cal_fbc.out = &fbc_addr;
		dcam_if_cal_compressed_addr(&cal_fbc);
		hw_fbc_store->idx = hw_ctx->hw_ctx_id;
		hw_fbc_store->path_id = path_id;
		hw_fbc_store->frame_addr[0] = fbc_addr.addr0;
		hw_fbc_store->frame_addr[1] = fbc_addr.addr1;
		hw_fbc_store->data_bits = cam_data_bits(dcam_port->out_fmt);
	}

	hw_store->idx = hw_ctx->hw_ctx_id;
	hw_store->frame_addr[0] = frame->buf.iova;
	hw_store->frame_addr[1] = 0;
	hw_store->path_id= path_id;
	hw_store->out_fmt = dcam_port->out_fmt;
	hw_store->out_size.h = dcam_port->out_size.h;
	hw_store->out_size.w = dcam_port->out_size.w;
	hw_store->out_pitch = dcam_port->out_pitch;
	hw_store->blk_param = hw_ctx->blk_pm;

	hw_size->idx = hw_ctx->hw_ctx_id;
	hw_size->path_id = path_id;
	hw_size->bin_ratio = dcam_port->bin_ratio;
	hw_size->scaler_sel = dcam_port->scaler_sel;
	hw_size->in_size = dcam_port->in_size;
	hw_size->in_trim = dcam_port->in_trim;
	hw_size->out_size = dcam_port->out_size;
	hw_size->out_pitch= dcam_port->out_pitch;
	hw_size->scaler_info = &dcam_port->scaler_info;
	hw_size->compress_info = frame->fbc_info;

	hw_start->idx = hw_ctx->hw_ctx_id;
	hw_start->path_id = path_id;
	hw_start->slowmotion_count = 0;
	hw_start->pdaf_path_eb = 0;
	hw_start->cap_info.format = DCAM_CAP_MODE_MAX;
	hw_start->src_sel = dcam_port->src_sel;
	hw_start->in_trim = dcam_port->in_trim;
	hw_start->endian = dcam_port->endian;
	hw_start->out_fmt = dcam_port->out_fmt;


	hw_fbc_ctrl->idx = hw_ctx->hw_ctx_id;
	hw_fbc_ctrl->path_id = path_id;
	hw_fbc_ctrl->fmt = dcam_port->out_fmt;
	hw_fbc_ctrl->data_bits = cam_data_bits(dcam_port->out_fmt);
	hw_fbc_ctrl->compress_en = dcam_port->compress_en;

	frame->param_data = dcam_port->priv_size_data;
	frame->width = dcam_port->out_size.w;
	frame->height = dcam_port->out_size.h;
	hw_ctx->hw_path[path_id].need_update = 1;

	return ret;
}

int dcam_offline_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct dcam_offline_port *dcam_port = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_frame **frame = NULL;
	uint32_t layer_num = ISP_PYR_DEC_LAYER_NUM;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_port = (struct dcam_offline_port *)handle;
	switch (cmd) {
	case PORT_BUFFER_CFG_SET:
		pframe = (struct camera_frame *)param;
		if ((pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
			ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_DCAM);
			if (ret)
				goto exit;
		}

		pframe->priv_data = dcam_port;
		if (pframe->pyr_status == OFFLINE_DEC_ON) {
			while (isp_rec_small_layer_w(dcam_port->out_size.w, layer_num) < MIN_PYR_WIDTH ||
				isp_rec_small_layer_h(dcam_port->out_size.h, layer_num) < MIN_PYR_HEIGHT) {
				pr_debug("layer num need decrease based on small input %d %d\n",
					dcam_port->out_size.w, dcam_port->out_size.h);
				if (--layer_num == 0)
					break;
			}
			if (!layer_num)
				pframe->need_pyr_dec = 0;
			else
				pframe->need_pyr_dec = 1;
		}
		ret = cam_queue_enqueue(&dcam_port->out_buf_queue, &pframe->list);
		if (ret) {
			pr_err("fail to enqueue frame of dcam path %s\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
			cam_buf_iommu_unmap(&pframe->buf);
			goto exit;
		}
		pr_info("config dcam offline path %s output buffer.\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		break;
	case PORT_SIZE_CFG_SET:
		ret = dcamoffline_port_size_cfg(dcam_port, param);
		break;
	case PORT_PARAM_CFG_GET:
		ret = dcamoffline_port_param_get(dcam_port, param);
		break;
	case PORT_BUFFER_CFG_GET:
		frame = (struct camera_frame **)param;
		*frame = cam_queue_dequeue(&dcam_port->result_queue, struct camera_frame, list);
		if (*frame == NULL) {
			pr_err("fail to available dcamoffline result buf %s\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
			ret = -EFAULT;
		}
		break;
	default:
		pr_err("fail to support port cfg cmd %d\n", cmd);
		ret = -EFAULT;
		break;
	}

exit:
	return ret;
}

void *dcam_offline_port_get(uint32_t port_id, struct dcam_offline_port_desc *param)
{
	struct dcam_offline_port *port = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	if (*param->port_dev == NULL) {
		port = vzalloc(sizeof(struct dcam_offline_port));
		if (!port) {
			pr_err("fail to get valid dcam offline port %s\n", cam_port_dcam_offline_out_id_name_get(port_id));
			return NULL;
		}
		pr_debug("dcam offline port %s dev %px\n", cam_port_dcam_offline_out_id_name_get(port_id), port);
	} else {
		port = *param->port_dev;
		pr_debug("dcam offline port has been alloc %p %s\n", port, cam_port_dcam_offline_out_id_name_get(port_id));
		goto exit;
	}

	port->port_id = port_id;
	*param->port_dev = port;
	atomic_set(&port->is_work, 0);
	if (port_id == PORT_OFFLINE_RAW_OUT || port_id == PORT_OFFLINE_BIN_OUT || port_id == PORT_OFFLINE_FULL_OUT) {
		cam_queue_init(&port->result_queue, DCAM_OFFLINE_RESULT_Q_LEN,
			dcamoffline_port_outframe_ret);
		cam_queue_init(&port->out_buf_queue, DCAM_OFFLINE_OUT_BUF_Q_LEN,
			dcamoffline_port_outframe_ret);
	}

	dcamoffline_port_base_cfg(port, param);
	*param->port_dev = port;
	port->port_id = port_id;
	port->data_cb_handle = param->data_cb_handle;
	port->data_cb_func = param->data_cb_func;
	pr_info("port id %s node_dev %px\n", cam_port_dcam_offline_out_id_name_get(port_id), *param->port_dev);

exit:
	atomic_inc(&port->user_cnt);
	return port;
}

void dcam_offline_port_put(struct dcam_offline_port *port)
{
	if (!port) {
		pr_err("fail to get invalid port ptr\n");
		return;
	}

	if (atomic_dec_return(&port->user_cnt) == 0) {
		cam_queue_clear(&port->result_queue, struct camera_frame, list);
		cam_queue_clear(&port->out_buf_queue, struct camera_frame, list);
		port->data_cb_handle = NULL;
		port->data_cb_func = NULL;

		pr_debug("dcam offline port %s put success\n", cam_port_dcam_offline_out_id_name_get(port->port_id));
		vfree(port);
		port = NULL;
	}
}
