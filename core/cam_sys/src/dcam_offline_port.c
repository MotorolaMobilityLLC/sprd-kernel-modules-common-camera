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
#include "dcam_offline_port.h"
#include "dcam_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_OFFLINE_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

int dcam_offline_port_size_cfg(void *handle, void *param)
{
	int ret = 0;
	struct dcam_offline_port *port = NULL;
	struct img_size crop_size = {0};
	struct img_size dst_size = {0};
	struct cam_zoom_base *zoom_base = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	port = (struct dcam_offline_port *)handle;
	zoom_base = (struct cam_zoom_base *)param;

	port->in_size = zoom_base->src;
	port->in_trim = zoom_base->crop;
	port->out_size = zoom_base->dst;
	port->out_pitch = cal_sprd_pitch(port->out_size.w, port->out_fmt);

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

	if (port->zoom_cb_func == NULL) {
		port->zoom_cb_func = port_desc->zoom_cb_func;
		port->zoom_cb_handle = port_desc->zoom_cb_handle;
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

/* This function may put it into dcam core for hw ctx cfg,
 * or call sub function in dcam core for hw ctx cfg.
 */
static int dcamoffline_port_param_get(void *handle, void *param)
{
	int ret = 0;
	uint32_t path_id = 0;
	struct dcam_offline_port *dcam_port = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct cam_frame *frame = NULL;
	struct dcam_hw_cfg_store_addr *hw_store = NULL;
	struct dcam_hw_path_size *hw_size = NULL;
	struct dcam_hw_path_start *hw_start = NULL;
	struct dcam_hw_fbc_addr *hw_fbc_store = NULL;
	struct dcam_hw_fbc_ctrl *hw_fbc_ctrl = NULL;
	struct compressed_addr fbc_addr = {0};
	struct dcam_compress_cal_para cal_fbc = {0};
	struct camera_buf_get_desc buf_desc = {0};

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

	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
	buf_desc.mmu_type = CAM_BUF_IOMMUDEV_DCAM;
	frame = cam_buf_manager_buf_dequeue(&dcam_port->unprocess_pool, &buf_desc, dcam_port->buf_manager_handle);
	if (!frame) {
		pr_err("fail to get dcam offline port %s frame\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		return -EFAULT;
	}

	frame->common.link_from.port_id = dcam_port->port_id;
	frame->common.cam_fmt = dcam_port->out_fmt;
	ret = cam_buf_manager_buf_enqueue(&dcam_port->result_pool, frame, NULL, dcam_port->buf_manager_handle);
	if (ret) {
		pr_err("fail to enqueue dcam offline port %s frame\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		return -EFAULT;
	}
	if (dcam_port->compress_en) {
		cal_fbc.data_bits = cam_data_bits(dcam_port->out_fmt);
		cal_fbc.fbc_info = &frame->common.fbc_info;
		cal_fbc.in = frame->common.buf.iova[CAM_BUF_IOMMUDEV_DCAM];
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
	hw_store->frame_addr[0] = frame->common.buf.iova[CAM_BUF_IOMMUDEV_DCAM];
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
	hw_size->compress_info = frame->common.fbc_info;

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

	frame->common.width = dcam_port->out_size.w;
	frame->common.height = dcam_port->out_size.h;
	hw_ctx->hw_path[path_id].need_update = 1;

	return ret;
}

static uint32_t dcamoffline_port_bufq_clr(struct dcam_offline_port *port, void *param)
{
	struct cam_frame *pframe = NULL;
	struct camera_buf_get_desc buf_desc = {0};

	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_MOVE_TO_ALLOC;
	pframe = cam_buf_manager_buf_dequeue(&port->result_pool, &buf_desc, port->buf_manager_handle);
	if (!pframe)
		pframe = cam_buf_manager_buf_dequeue(&port->unprocess_pool, &buf_desc, port->buf_manager_handle);

	pr_debug("port:0x%px, port_id:%d.\n", port, port->port_id);
	return 0;
}

int dcam_offline_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct dcam_offline_port *dcam_port = NULL;
	struct cam_frame *pframe = NULL;
	struct cam_frame **frame = NULL;
	struct camera_buf_get_desc buf_desc = {0};

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_port = (struct dcam_offline_port *)handle;
	switch (cmd) {
	case PORT_CFG_BUFFER_SET:
		pframe = (struct cam_frame *)param;
		buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
		buf_desc.mmu_type = CAM_BUF_IOMMUDEV_DCAM;
		pframe->common.priv_data = dcam_port;
		ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, pframe, &buf_desc, dcam_port->buf_manager_handle);
		if (ret) {
			pr_err("fail to enqueue frame of dcam path %s\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
			goto exit;
		}
		pr_info("config dcam offline path %s output buffer.\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
		break;
	case PORT_CFG_PARAM_GET:
		ret = dcamoffline_port_param_get(dcam_port, param);
		break;
	case PORT_CFG_BUFFER_GET:
		frame = (struct cam_frame **)param;
		*frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, NULL, dcam_port->buf_manager_handle);
		if (*frame == NULL) {
			pr_err("fail to available dcamoffline result buf %s\n", cam_port_dcam_offline_out_id_name_get(dcam_port->port_id));
			ret = -EFAULT;
		}
		break;
	case PORT_CFG_BUFFER_CLR:
		ret = dcamoffline_port_bufq_clr(dcam_port, param);
		break;
	default:
		pr_err("fail to support port cfg cmd %d\n", cmd);
		ret = -EFAULT;
		break;
	}

exit:
	return ret;
}

int dcam_offline_port_buf_alloc(void *handle, struct cam_buf_alloc_desc *param)
{
	int ret = 0;
	struct dcam_offline_port *port = (struct dcam_offline_port *)handle;
	struct cam_frame *pframe = NULL;
	uint32_t i = 0,size = 0, height = 0, width = 0, total = 0, out_fmt;

	if (!port || !param) {
		pr_err("fail to get valid handle %px %px\n", handle, param);
		return -1;
	}

	width = param->width;
	height = param->height;
	out_fmt = port->out_fmt;
	if (param->compress_offline) {
		struct dcam_compress_info fbc_info = {0};
		struct dcam_compress_cal_para cal_fbc = {0};
		cal_fbc.data_bits = cam_data_bits(out_fmt);
		cal_fbc.fbc_info = &fbc_info;
		cal_fbc.fmt = out_fmt;
		cal_fbc.height = height;
		cal_fbc.width = width;
		size = dcam_if_cal_compressed_size (&cal_fbc);
	} else {
		size = cal_sprd_size(width, height, out_fmt);
	}

	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);

	total = param->dcamoffline_buf_alloc_num;
	pr_info("ch %d alloc shared buffer size: %u (w %u h %u), num %d\n",
		param->ch_id, size, width, height, total);

	for (i = 0; i < total; i++) {
		do {
			pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
			pframe->common.channel_id = param->ch_id;
			pframe->common.is_compressed = param->compress_en;
			pframe->common.width = width;
			pframe->common.height = height;
			CAM_QUEUE_FRAME_FLAG_RESET(&pframe->common);
			ret = cam_buf_alloc(&pframe->common.buf, size, param->iommu_enable);
			if (ret) {
				pr_err("fail to alloc buf: %d ch %d\n",
					i, param->ch_id);
				cam_queue_empty_frame_put(pframe);
				continue;
			}
			cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_GET_IOVA, CAM_BUF_IOMMUDEV_DCAM);
			cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_GET_IOVA, CAM_BUF_IOMMUDEV_ISP);
			pframe->common.buf.bypass_iova_ops = ENABLE;

			ret = cam_buf_manager_buf_enqueue(&port->unprocess_pool, pframe, NULL, port->buf_manager_handle);
		} while (0);
	}

	ret = cam_buf_manager_pool_cnt(&port->unprocess_pool, port->buf_manager_handle);
	if (ret > 0 || !total)
		return 0;
	else
		return -1;
}

void *dcam_offline_port_get(uint32_t port_id, struct dcam_offline_port_desc *param)
{
	struct dcam_offline_port *port = NULL;
	int ret = 0;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	if (*param->port_dev == NULL) {
		port = cam_buf_kernel_sys_vzalloc(sizeof(struct dcam_offline_port));
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
	port->buf_manager_handle = param->buf_manager_handle;
	*param->port_dev = port;
	atomic_set(&port->is_work, 0);

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OFFLINE_OUT_BUF_Q_LEN, port->buf_manager_handle);
	if (ret <= 0) {
		pr_err("fail to reg unprocess pool for dcam offline node\n");
		cam_buf_kernel_sys_vfree(port);
		return NULL;
	}
	port->unprocess_pool.private_pool_id = ret;

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OFFLINE_RESULT_Q_LEN, port->buf_manager_handle);
	if (ret <= 0) {
		pr_err("fail to reg result pool for dcam offline node\n");
		cam_buf_manager_pool_unreg(&port->unprocess_pool, port->buf_manager_handle);
		cam_buf_kernel_sys_vfree(port);
		return NULL;
	}
	port->result_pool.private_pool_id = ret;
	pr_debug("%s reg pool %d %d\n", cam_port_dcam_offline_out_id_name_get(port_id), port->unprocess_pool.private_pool_id, port->result_pool.private_pool_id);
	dcamoffline_port_base_cfg(port, param);
	*param->port_dev = port;
	port->port_id = port_id;
	port->type = param->transfer_type;
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
		cam_buf_manager_pool_unreg(&port->unprocess_pool, port->buf_manager_handle);
		cam_buf_manager_pool_unreg(&port->result_pool, port->buf_manager_handle);
		port->data_cb_handle = NULL;
		port->data_cb_func = NULL;
		port->zoom_cb_func = NULL;

		pr_debug("dcam offline port %s put success\n", cam_port_dcam_offline_out_id_name_get(port->port_id));
		cam_buf_kernel_sys_vfree(port);
		port = NULL;
	}
}
