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

#include <linux/vmalloc.h>
#include <linux/slab.h>
#include "dcam_online_port.h"
#include "dcam_interface.h"
#include "dcam_core.h"
#include "dcam_online_node.h"
#include "dcam_reg.h"
#include "cam_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_ONLINE_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

static void dcamonline_port_check_status(struct dcam_online_port *dcam_port,
	struct dcam_hw_context *hw_ctx, struct dcam_isp_k_block *blk_dcam_pm)
{
	struct camera_frame *frame = NULL;
	struct camera_buf_get_desc buf_desc = {0};
	int ret = 0, recycle = 0;

	if (unlikely(!dcam_port || !hw_ctx || !blk_dcam_pm))
		return;

	buf_desc.q_ops_cmd = CAM_QUEUE_DEL_TAIL;
	if (blk_dcam_pm->gtm[DCAM_GTM_PARAM_PRE].gtm_info.bypass_info.gtm_hist_stat_bypass)
		recycle = 1;
	if (blk_dcam_pm->rgb_gtm[DCAM_GTM_PARAM_PRE].rgb_gtm_info.bypass_info.gtm_hist_stat_bypass)
		recycle = 1;
	if (g_dcam_bypass[hw_ctx->hw_ctx_id] & (1 << _E_GTM))
		recycle = 1;
	if (recycle) {
		pr_debug("dcam hw%d gtm bypass, no need buf\n", hw_ctx->hw_ctx_id);
		frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, &buf_desc);
		if (frame) {
			if (frame->is_reserved)
				ret = dcam_online_port_reserved_buf_set(dcam_port, frame);
			else
				ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, frame, NULL);
			if (ret)
				pr_err("fail to enqueue gtm\n");
		}
	}
}

int dcamonline_port_buffer_cfg(void *handle, void *param)
{
	int ret = 0;
	struct dcam_online_port *dcam_online_port = NULL;
	struct camera_frame *pframe = NULL;
	struct cam_buf_pool_id pool_id = {0};
	struct camera_buf_get_desc buf_desc = {0};

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_online_port = (struct dcam_online_port *)handle;
	pframe = (struct camera_frame *)param;

	if (dcam_online_port->port_id == PORT_BIN_OUT) {
		buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
		buf_desc.mmu_type = CAM_IOMMUDEV_DCAM;
	} else if (dcam_online_port->port_id == PORT_FULL_OUT || dcam_online_port->port_id == PORT_RAW_OUT || dcam_online_port->port_id == PORT_VCH2_OUT)
		buf_desc.buf_ops_cmd = CAM_BUF_STATUS_MOVE_TO_ION;

	pframe->is_reserved = 0;
	pframe->not_use_isp_reserved_buf = 0;
	pframe->priv_data = dcam_online_port;
	if (pframe->pyr_status == ONLINE_DEC_ON)
		pframe->need_pyr_rec = 1;
	else if (pframe->pyr_status == OFFLINE_DEC_ON)
		pframe->need_pyr_dec = 1;

	if (dcam_online_port->share_full_path)
		pool_id.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
	else
		pool_id.private_pool_idx = dcam_online_port->unprocess_pool.private_pool_idx;

	ret = cam_buf_manager_buf_enqueue(&pool_id, pframe, &buf_desc);
	if (ret) {
		pr_err("fail to enqueue frame of online port %d\n", cam_port_name_get(dcam_online_port->port_id));
		cam_buf_destory(pframe);
		return ret;
	}

	pr_debug("out_buf_queue cnt:%d, port id:%d, addr:%x.\n", cam_buf_manager_pool_cnt(&pool_id),
		dcam_online_port->port_id, pframe->buf.iova);

	return ret;
}

static int dcamonline_port_size_cfg(void *handle, void *param)
{
	int ret = 0;
	uint32_t invalid = 0;
	unsigned long flag = 0;
	struct img_size dst_size = {0};
	struct img_size crop_size = {0};
	struct dcam_online_port *dcam_online_port = NULL;
	struct dcam_path_cfg_param  *ch_desc = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_online_port = (struct dcam_online_port *)handle;
	ch_desc = (struct dcam_path_cfg_param *)param;
	if (ch_desc->is_csi_connect)
		dcam_online_port->size_update = 0;

	switch (dcam_online_port->port_id) {
	case PORT_RAW_OUT:
	case PORT_FULL_OUT:
		spin_lock_irqsave(&dcam_online_port->size_lock, flag);
		if (dcam_online_port->size_update) {
			spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);
			return -EFAULT;
		}
		dcam_online_port->in_size = ch_desc->input_size;
		dcam_online_port->in_trim = ch_desc->input_trim;
		invalid |= ((dcam_online_port->in_size.w == 0) || (dcam_online_port->in_size.h == 0));
		invalid |= ((dcam_online_port->in_trim.start_x + dcam_online_port->in_trim.size_x) > dcam_online_port->in_size.w);
		invalid |= ((dcam_online_port->in_trim.start_y + dcam_online_port->in_trim.size_y) > dcam_online_port->in_size.h);

		if (invalid) {
			spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);
			pr_err("fail to get valid size, size:%d %d, trim %d %d %d %d\n",
				dcam_online_port->in_size.w, dcam_online_port->in_size.h,
				dcam_online_port->in_trim.start_x, dcam_online_port->in_trim.start_y,
				dcam_online_port->in_trim.size_x,
				dcam_online_port->in_trim.size_y);
			return -EINVAL;
		}
		if ((dcam_online_port->in_size.w > dcam_online_port->in_trim.size_x) ||
			(dcam_online_port->in_size.h > dcam_online_port->in_trim.size_y)) {
			dcam_online_port->out_size.w = dcam_online_port->in_trim.size_x;
			dcam_online_port->out_size.h = dcam_online_port->in_trim.size_y;
		} else {
			dcam_online_port->out_size.w = dcam_online_port->in_size.w;
			dcam_online_port->out_size.h = dcam_online_port->in_size.h;
		}

		dcam_online_port->out_pitch = dcampath_outpitch_get(dcam_online_port->out_size.w, dcam_online_port->dcamout_fmt);
		dcam_online_port->priv_size_data = ch_desc->priv_size_data;
		dcam_online_port->size_update = 1;
		spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);

		pr_info("cfg %s path done. size %d %d %d %d\n",
			dcam_online_port->port_id == PORT_RAW_OUT ? "raw" : "full", dcam_online_port->in_size.w, dcam_online_port->in_size.h,
			dcam_online_port->out_size.w, dcam_online_port->out_size.h);

		pr_info("sel %d. trim %d %d %d %d\n", dcam_online_port->raw_src,
			dcam_online_port->in_trim.start_x, dcam_online_port->in_trim.start_y,
			dcam_online_port->in_trim.size_x, dcam_online_port->in_trim.size_y);
		break;

	case PORT_BIN_OUT:
		/* lock here to keep all size parameters updating is atomic.
		 * because the rds coeff caculation may be time-consuming,
		 * we should not disable irq here, or else may cause irq missed.
		 * Just trylock set_next_frame in irq handling to avoid deadlock
		 * If last updating has not been applied yet, will return error.
		 * error may happen if too frequent zoom ratio updateing,
		 * but should not happen for first time cfg before stream on,
		 * if error return, caller can discard updating
		 * or try cfg_size again after while.
		 */
		spin_lock_irqsave(&dcam_online_port->size_lock, flag);
		if (dcam_online_port->size_update) {
			spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);
			pr_warn("warning: Previous port size updating pending\n");
			return -EFAULT;
		}

		dcam_online_port->in_size = ch_desc->input_size;
		dcam_online_port->in_trim = ch_desc->input_trim;
		dcam_online_port->total_in_trim = ch_desc->total_input_trim;
		dcam_online_port->out_size = ch_desc->output_size;
		dcam_online_port->dst_crop_w = ch_desc->zoom_ratio_base.w;

		invalid = 0;
		invalid |= ((dcam_online_port->in_size.w == 0) || (dcam_online_port->in_size.h == 0));
		/* trim should not be out range of source */
		invalid |= ((dcam_online_port->in_trim.start_x +
				dcam_online_port->in_trim.size_x) > dcam_online_port->in_size.w);
		invalid |= ((dcam_online_port->in_trim.start_y +
				dcam_online_port->in_trim.size_y) > dcam_online_port->in_size.h);

		/* output size should not be larger than trim ROI */
		invalid |= dcam_online_port->in_trim.size_x < dcam_online_port->out_size.w;
		invalid |= dcam_online_port->in_trim.size_y < dcam_online_port->out_size.h;

		/* Down scaling should not be smaller then 1/4*/
		invalid |= dcam_online_port->in_trim.size_x >
				(dcam_online_port->out_size.w * DCAM_SCALE_DOWN_MAX);
		invalid |= dcam_online_port->in_trim.size_y >
				(dcam_online_port->out_size.h * DCAM_SCALE_DOWN_MAX);

		if (invalid) {
			spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);
			pr_err("fail to get valid size, size:%d %d, trim %d %d %d %d, dst %d %d\n",
				dcam_online_port->in_size.w, dcam_online_port->in_size.h,
				dcam_online_port->in_trim.start_x, dcam_online_port->in_trim.start_y,
				dcam_online_port->in_trim.size_x, dcam_online_port->in_trim.size_y,
				dcam_online_port->out_size.w, dcam_online_port->out_size.h);
			return -EINVAL;
		}
		crop_size.w = dcam_online_port->in_trim.size_x;
		crop_size.h = dcam_online_port->in_trim.size_y;
		dst_size = dcam_online_port->out_size;
		dcam_online_port->out_pitch = dcampath_outpitch_get(dcam_online_port->out_size.w, dcam_online_port->dcamout_fmt);

		switch (dcam_online_port->dcamout_fmt) {
			case CAM_YUV_BASE:
			case CAM_YUV422_2FRAME:
			case CAM_YVU422_2FRAME:
			case CAM_YUV420_2FRAME:
			case CAM_YVU420_2FRAME:
			case CAM_YUV420_2FRAME_MIPI:
			case CAM_YVU420_2FRAME_MIPI:
				if ((crop_size.w == dst_size.w) && (crop_size.h == dst_size.h)) {
					dcam_online_port->scaler_sel = PORT_SCALER_BYPASS;
					break;
				}
				if (dst_size.w > DCAM_SCALER_MAX_WIDTH || dcam_online_port->in_trim.size_x > (dst_size.w * DCAM_SCALE_DOWN_MAX)) {
					pr_err("fail to support scaler, in width %d, out width %d\n",
						dcam_online_port->in_trim.size_x, dst_size.w);
					ret = -1;
				}
				dcam_online_port->scaler_sel = PORT_SCALER_BY_YUVSCALER;
				dcam_online_port->scaler_info.scaler_factor_in = dcam_online_port->in_trim.size_x;
				dcam_online_port->scaler_info.scaler_factor_out = dst_size.w;
				dcam_online_port->scaler_info.scaler_ver_factor_in = dcam_online_port->in_trim.size_y;
				dcam_online_port->scaler_info.scaler_ver_factor_out = dst_size.h;
				dcam_online_port->scaler_info.scaler_out_width = dst_size.w;
				dcam_online_port->scaler_info.scaler_out_height = dst_size.h;

				dcam_online_port->scaler_info.work_mode = 2;
				dcam_online_port->scaler_info.scaler_bypass = 0;
				ret = cam_scaler_coeff_calc_ex(&dcam_online_port->scaler_info);
				if (ret)
					pr_err("fail to calc scaler coeff\n");
				break;
			case CAM_RAW_PACK_10:
			case CAM_RAW_HALFWORD_10:
			case CAM_RAW_14:
			case CAM_RAW_8:
			case CAM_FULL_RGB14:
				dcampath_bin_scaler_get(crop_size, dst_size, &dcam_online_port->scaler_sel, &dcam_online_port->bin_ratio);
				break;

			default:
				pr_err("fail to get path->out_fmt:%s\n", camport_fmt_name_get(dcam_online_port->dcamout_fmt));
				break;
		}

		if (dcam_online_port->priv_size_data) {
			cam_buf_kernel_sys_vfree(dcam_online_port->priv_size_data);
			dcam_online_port->priv_size_data = NULL;
		}
		dcam_online_port->priv_size_data = ch_desc->priv_size_data;
		dcam_online_port->size_update = 1;
		spin_unlock_irqrestore(&dcam_online_port->size_lock, flag);

		pr_info("cfg bin path done. size %d %d  dst %d %d\n",
			dcam_online_port->in_size.w, dcam_online_port->in_size.h,
			dcam_online_port->out_size.w, dcam_online_port->out_size.h);
		pr_info("scaler %d. trim %d %d %d %d\n", dcam_online_port->scaler_sel,
			dcam_online_port->in_trim.start_x, dcam_online_port->in_trim.start_y,
			dcam_online_port->in_trim.size_x, dcam_online_port->in_trim.size_y);
		break;
	default:
		if (dcam_online_port->port_id == PORT_VCH2_OUT && dcam_online_port->raw_src)
			return ret;

		pr_err("fail to get known path %d\n", dcam_online_port->port_id);
		ret = -EFAULT;
		break;
	}
	return ret;
}

static struct camera_frame *
dcamonline_port_reserved_buf_get(struct dcam_online_port *dcam_port)
{
	struct camera_frame *frame = NULL;
	struct camera_buf_get_desc buf_desc = {0};

	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_SINGLE_PAGE_IOVA;
	buf_desc.mmu_type = CAM_IOMMUDEV_DCAM;
	frame = cam_buf_manager_buf_dequeue(&dcam_port->reserved_pool, &buf_desc);
	if (frame != NULL) {
		frame->priv_data = dcam_port;
		pr_debug("use reserved buffer for port %d\n", dcam_port->port_id);
		if (dcam_port->compress_en)
			frame->is_compressed = 1;
	}
	return frame;
}

static inline struct camera_frame *dcamonline_port_frame_cycle(struct dcam_online_port *dcam_port, struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	struct camera_frame *frame = NULL;
	struct cam_buf_pool_id pool_id = {0};
	struct camera_buf_get_desc buf_desc = {0};

	if (!dcam_port || !hw_ctx) {
		pr_err("fail to get null pointer.\n");
		return NULL;
	}

	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
	buf_desc.mmu_type = CAM_IOMMUDEV_DCAM;

	if (dcam_port->share_full_path)
		pool_id.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
	else
		pool_id = dcam_port->unprocess_pool;
	pool_id.reserved_pool_id = dcam_port->reserved_pool.reserved_pool_id;

	frame = cam_buf_manager_buf_dequeue(&pool_id, &buf_desc);
	if (unlikely(!frame)) {
		pr_err("fail to get port%d output buf\n", dcam_port->port_id);
		return ERR_PTR(-EPERM);
	}

	frame->fid = hw_ctx->base_fid + hw_ctx->index_to_set;
	frame->priv_data = dcam_port;
	if (dcam_port->compress_en)
		frame->is_compressed = 1;
	hw_ctx->fid = frame->fid;
	frame->cam_fmt = dcam_port->dcamout_fmt;
	ret = cam_buf_manager_buf_enqueue(&dcam_port->result_pool, frame, NULL);
	if (ret) {
		pr_err("fail to set next frm buffer, pool %d, %s\n", dcam_port->result_pool.private_pool_idx, cam_port_name_get(dcam_port->port_id));
		pool_id.reserved_pool_id = 0;
		if (dcam_port->port_id == PORT_FULL_OUT || dcam_port->port_id == PORT_RAW_OUT || dcam_port->port_id == PORT_VCH2_OUT)
			buf_desc.buf_ops_cmd = CAM_BUF_STATUS_PUT_IOVA;
		if (frame->is_reserved)
			dcam_online_port_reserved_buf_set(dcam_port, frame);
		else
			cam_buf_manager_buf_enqueue(&pool_id, frame, &buf_desc);
		return NULL;
	}

	return frame;
}

static void dcamonline_port_update_path_size(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx)
{
	struct dcam_hw_path_size path_size = {0};
	struct cam_hw_info *hw;

	hw = hw_ctx->hw;

	if (dcam_port->size_update) {
		path_size.path_id = dcamonline_portid_convert_to_pathid(dcam_port->port_id);
		path_size.idx = idx;
		path_size.size_x = hw_ctx->cap_info.cap_size.size_x;
		path_size.size_y = hw_ctx->cap_info.cap_size.size_y;
		path_size.src_sel = dcam_port->src_sel;
		path_size.bin_ratio = dcam_port->bin_ratio;
		path_size.scaler_sel = dcam_port->scaler_sel;
		path_size.in_size = dcam_port->in_size;
		path_size.in_trim = dcam_port->in_trim;
		path_size.out_size = dcam_port->out_size;
		path_size.out_pitch= dcam_port->out_pitch;
		path_size.compress_info = frame->fbc_info;
		path_size.scaler_info = &dcam_port->scaler_info;
		frame->param_data = dcam_port->priv_size_data;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &path_size);

		dcam_port->size_update = 0;
		dcam_port->priv_size_data = NULL;
		if (!hw_ctx->is_virtualsensor_proc)
			dcam_port->base_update = 0;
	}
}

static int dcamonline_port_pyr_dec_cfg(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t align_w = 0, align_h = 0;
	uint32_t layer_num = 0;
	struct cam_hw_info *hw;
	struct dcam_hw_dec_store_cfg dec_store;
	struct dcam_hw_dec_online_cfg dec_online;

	if (!dcam_port || !frame || !hw_ctx) {
		pr_err("fail to check param, path%px, frame%px\n", dcam_port, frame);
		return -EINVAL;
	}
	hw = hw_ctx->hw;

	memset(&dec_store, 0, sizeof(struct dcam_hw_dec_store_cfg));
	memset(&dec_online, 0, sizeof(struct dcam_hw_dec_online_cfg));

	layer_num = dcam_port->dec_store_info.layer_num;
	dec_online.idx = idx;
	dec_online.layer_num = layer_num;
	dec_online.chksum_clr_mode = 0;
	dec_online.chksum_work_mode = 0;
	dec_online.path_sel = DACM_DEC_PATH_DEC;
	dec_online.hor_padding_num = dcam_port->dec_store_info.align_w - dcam_port->out_size.w;
	dec_online.ver_padding_num = dcam_port->dec_store_info.align_h - dcam_port->out_size.h;
	if (dec_online.hor_padding_num)
		dec_online.hor_padding_en = 1;
	if (dec_online.ver_padding_num)
		dec_online.ver_padding_en = 1;
	dec_online.flust_width = dcam_port->out_size.w;
	dec_online.flush_hblank_num = dec_online.hor_padding_num + 20;
	dec_online.flush_line_num = dec_online.ver_padding_num + 20;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_DEC_ONLINE, &dec_online);

	dec_store.idx = idx;
	dec_store.bypass = 1;
	dec_store.endian = dcam_port->endian;
	dec_store.color_format = dcam_port->dcamout_fmt;
	dec_store.border_up = 0;
	dec_store.border_down = 0;
	dec_store.border_left = 0;
	dec_store.border_right = 0;
	/*because hw limit, pyr output 10bit*/
	dec_store.data_10b = 1;
	dec_store.flip_en = 0;
	dec_store.last_frm_en = 1;
	dec_store.mirror_en = 0;
	dec_store.mono_en = 0;
	dec_store.speed2x = 1;
	dec_store.rd_ctrl = 0;
	dec_store.store_res = 0;
	dec_store.burst_len = 1;

	pr_debug("dcam %d padding w %d h %d alignw %d h%d\n", idx,
		dec_online.hor_padding_num, dec_online.ver_padding_num, align_w, align_h);
	for (i = 0; i < layer_num; i++) {
		dec_store.bypass = 0;
		dec_store.cur_layer = i;
		dec_store.width = dcam_port->dec_store_info.size_t[i].w;
		dec_store.height = dcam_port->dec_store_info.size_t[i].h;
		dec_store.pitch[0] = dcam_port->dec_store_info.pitch_t[i].pitch_ch0;
		dec_store.pitch[1] = dcam_port->dec_store_info.pitch_t[i].pitch_ch1;
		/* when zoom, if necessary size update may set with path size udapte
		 thus, the dec_store need remember on path or ctx, and calc & reg set
		 need separate too, now just */
		hw->dcam_ioctl(hw, DCAM_HW_CFG_DEC_SIZE_UPDATE, &dec_store);

		pr_debug("dcam %d dec_layer %d w %d h %d\n", idx, i, dec_store.width, dec_store.height);
	}

	return ret;
}

static int dcamonline_port_update_pyr_dec_addr(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t layer_num = 0;
	uint32_t offset = 0, align = 1, size = 0;
	uint32_t align_w = 0, align_h = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_dec_store_cfg *dec_store = NULL;

	if (!dcam_port || !frame) {
		pr_err("fail to check param, port%px, frame%px\n", dcam_port, frame);
		return -EINVAL;
	}

	dec_store = &dcam_port->dec_store_info;
	hw = hw_ctx->hw;
	dec_store->idx = idx;

	layer_num = DCAM_PYR_DEC_LAYER_NUM;
	/* update layer num based on img size */
	while (isp_rec_small_layer_w(dcam_port->out_size.w, layer_num) < MIN_PYR_WIDTH ||
		isp_rec_small_layer_h(dcam_port->out_size.h, layer_num) < MIN_PYR_HEIGHT) {
		pr_debug("layer num need decrease based on small input %d %d\n",
			dcam_port->out_size.w, dcam_port->out_size.h);
		layer_num--;
		dec_store->layer_num = layer_num;
		hw->dcam_ioctl(hw, DCAM_HW_BYPASS_DEC, dec_store);
		if (layer_num == 0)
			break;
	}
	if (!layer_num)
		frame->need_pyr_rec = 0;
	else
		frame->need_pyr_rec = 1;

	align_w = dcamonline_dec_align_width(dcam_port->out_size.w, layer_num);
	align_h = dcamonline_dec_align_heigh(dcam_port->out_size.h, layer_num);
	size = dcam_port->out_pitch * dcam_port->out_size.h;
	dec_store->layer_num = layer_num;
	dec_store->align_w = align_w;
	dec_store->align_h = align_h;

	pr_debug("dcam %d out pitch %d addr %x\n", dec_store->idx, dcam_port->out_pitch, frame->buf.iova);
	for (i = 0; i < layer_num; i++) {
		align = align * 2;
		if (i == 0 && frame->is_compressed)
			offset += frame->fbc_info.buffer_size;
		else
			offset += (size * 3 / 2);
		dec_store->cur_layer = i;
		dec_store->size_t[i].w = align_w / align;
		dec_store->size_t[i].h = align_h / align;
		dec_store->pitch_t[i].pitch_ch0 = cal_sprd_yuv_pitch(dec_store->size_t[i].w,
			cam_data_bits(dcam_port->pyr_out_fmt), cam_is_pack(dcam_port->pyr_out_fmt));
		dec_store->pitch_t[i].pitch_ch1 = dec_store->pitch_t[i].pitch_ch0;
		size = dec_store->pitch_t[i].pitch_ch0 * dec_store->size_t[i].h;
		dec_store->addr[0] = frame->buf.iova + offset;
		dec_store->addr[1] = dec_store->addr[0] + size;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_DEC_STORE_ADDR, dec_store);

		pr_debug("dcam %d dec_layer %d w %d h %d\n", dec_store->idx, i, dec_store->size_t[i].w, dec_store->size_t[i].h);
		pr_debug("dcam %d dec_layer %d w %x h %x\n", dec_store->idx, i, dec_store->addr[0], dec_store->addr[1]);
	}

	return ret;
}

static void dcamonline_port_cfg_store_addr(struct dcam_online_port *dcam_port,struct camera_frame *frame,
	struct dcam_hw_slw_fmcu_cmds *slw, struct dcam_hw_context *hw_ctx, uint32_t idx, struct dcam_isp_k_block *blk_dcam_pm)
{
	uint32_t path_id = 0;
	struct dcam_hw_fbc_addr fbcadr = {0};
	struct dcam_hw_cfg_store_addr store_arg = {0};
	struct cam_hw_info *hw = hw_ctx->hw;

	path_id = dcamonline_portid_convert_to_pathid(dcam_port->port_id);
	if (slw == NULL) {
		if (frame->is_compressed) {
			struct compressed_addr fbc_addr = {0};
			struct img_size *size = &dcam_port->out_size;
			struct dcam_compress_cal_para cal_fbc = {0};

			cal_fbc.data_bits = cam_data_bits(dcam_port->dcamout_fmt);
			cal_fbc.fbc_info = &frame->fbc_info;
			cal_fbc.in = frame->buf.iova;
			cal_fbc.fmt = dcam_port->dcamout_fmt;
			cal_fbc.height = size->h;
			cal_fbc.width = size->w;
			cal_fbc.out = &fbc_addr;
			dcam_if_cal_compressed_addr(&cal_fbc);
			fbcadr.idx = idx;
			fbcadr.frame_addr[0] = fbc_addr.addr0;
			fbcadr.frame_addr[1] = fbc_addr.addr1;
			fbcadr.path_id = path_id;
			fbcadr.data_bits = cam_data_bits(dcam_port->dcamout_fmt);
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_ADDR_SET, &fbcadr);
		} else {
			store_arg.idx = idx;
			store_arg.frame_addr[0] = frame->buf.iova;
			store_arg.frame_addr[1] = 0;
			store_arg.path_id= path_id;
			store_arg.out_fmt = dcam_port->dcamout_fmt;
			store_arg.out_size.h = dcam_port->out_size.h;
			store_arg.out_size.w = dcam_port->out_size.w;
			store_arg.out_pitch = dcam_port->out_pitch;
			store_arg.in_fmt = hw_ctx->cap_info.format;
			store_arg.blk_param = blk_dcam_pm;
			store_arg.frame_size = frame->buf.size;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_STORE_ADDR, &store_arg);
		}
	}
}

static void dcamonline_port_update_addr_and_size(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, struct dcam_hw_slw_fmcu_cmds *slw, uint32_t idx, struct dcam_isp_k_block *blk_dcam_pm)
{
	unsigned long flags = 0;
	uint32_t layer_num = ISP_PYR_DEC_LAYER_NUM;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;

	spin_lock_irqsave(&dcam_port->size_lock, flags);
	dcamonline_port_cfg_store_addr(dcam_port, frame, slw, hw_ctx, idx, blk_dcam_pm);

	switch (dcam_port->port_id) {
	case PORT_RAW_OUT:
		dcamonline_port_update_path_size(dcam_port, frame, hw_ctx, idx);
		frame->width = dcam_port->out_size.w;
		frame->height = dcam_port->out_size.h;
		break;
	case PORT_BIN_OUT:
		if ((slw == NULL) && (frame->pyr_status == ONLINE_DEC_ON))
			dcamonline_port_update_pyr_dec_addr(dcam_port, frame, hw_ctx, idx);

		if (!frame->is_reserved || slw != NULL) {
			if (frame->pyr_status == ONLINE_DEC_ON)
				dcamonline_port_pyr_dec_cfg(dcam_port, frame, hw_ctx, idx);
			dcamonline_port_update_path_size(dcam_port, frame, hw_ctx, idx);
			if (!dcam_port->size_update) {
				dcam_port->size_update = 0;
				hw_ctx->next_roi = dcam_port->in_trim;
				hw_ctx->zoom_ratio = ZOOM_RATIO_DEFAULT * dcam_port->dst_crop_w / dcam_port->in_trim.size_x;
				hw_ctx->total_zoom = ZOOM_RATIO_DEFAULT *dcam_port->in_size.w / dcam_port->total_in_trim.size_x;
				pr_debug("total_zoom %d, zoom_ratio %d\n",hw_ctx->total_zoom, hw_ctx->zoom_ratio);
			}
		}
		frame->width = dcam_port->out_size.w;
		frame->height = dcam_port->out_size.h;
		break;
	case PORT_FULL_OUT:
		if (dcam_port->base_update) {
			struct dcam_hw_path_src_sel patharg;
			patharg.idx = idx;
			patharg.src_sel = dcam_port->src_sel;
			patharg.fmt = dcam_port->dcamout_fmt;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SRC_SEL, &patharg);
		}
		dcamonline_port_update_path_size(dcam_port, frame, hw_ctx, idx);
		if (frame->pyr_status == OFFLINE_DEC_ON) {
			while (isp_rec_small_layer_w(dcam_port->out_size.w, layer_num) < MIN_PYR_WIDTH ||
				isp_rec_small_layer_h(dcam_port->out_size.h, layer_num) < MIN_PYR_HEIGHT) {
				pr_debug("layer num need decrease based on small input %d %d\n",
					dcam_port->out_size.w, dcam_port->out_size.h);
				if (--layer_num == 0)
					break;
			}
			if (!layer_num)
				frame->need_pyr_dec = 0;
			else
				frame->need_pyr_dec = 1;
		}
		frame->width = dcam_port->out_size.w;
		frame->height = dcam_port->out_size.h;
		break;
	default:
		if (!frame->is_reserved || slw != NULL)
			dcam_port->base_update = 0;
		break;
	}

	frame->zoom_ratio = hw_ctx->zoom_ratio;
	frame->total_zoom = hw_ctx->total_zoom;
	spin_unlock_irqrestore(&dcam_port->size_lock, flags);
}

static void dcamonline_port_aem_update_statis_head(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, struct dcam_isp_k_block *blk_dcam_pm)
{
	struct dcam_dev_aem_win *win;
	struct sprd_img_rect *zoom_rect;
	unsigned long flags = 0;

	/* Re-config aem win if it is updated */
	spin_lock_irqsave(&blk_dcam_pm->aem.aem_win_lock, flags);
	dcam_k_aem_win(blk_dcam_pm);

	if (frame->buf.addr_k) {
		win = (struct dcam_dev_aem_win *)(frame->buf.addr_k);
		pr_debug("kaddr %lx\n", frame->buf.addr_k);
		memcpy(win, &blk_dcam_pm->aem.win_info, sizeof(struct dcam_dev_aem_win));
		win++;
		zoom_rect = (struct sprd_img_rect *)win;
		zoom_rect->x = hw_ctx->next_roi.start_x;
		zoom_rect->y = hw_ctx->next_roi.start_y;
		zoom_rect->w = hw_ctx->next_roi.size_x;
		zoom_rect->h = hw_ctx->next_roi.size_y;
	}
	spin_unlock_irqrestore(&blk_dcam_pm->aem.aem_win_lock, flags);
}

static void dcamonline_port_hist_update_statis_head(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, struct dcam_isp_k_block *blk_dcam_pm)
{
	unsigned long flags = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;

	/* Re-config hist win if it is updated */
	spin_lock_irqsave(&blk_dcam_pm->hist.param_update_lock, flags);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_HIST_ROI_UPDATE, blk_dcam_pm);
	if (frame->buf.addr_k) {
		struct dcam_dev_hist_info *info = NULL;
		info = (struct dcam_dev_hist_info *)frame->buf.addr_k;
		memcpy(info, &blk_dcam_pm->hist.bayerHist_info,
			sizeof(struct dcam_dev_hist_info));
	}
	spin_unlock_irqrestore(&blk_dcam_pm->hist.param_update_lock, flags);

}

static inline int dcamonline_port_hist_out_frm_set(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx, struct dcam_isp_k_block *blk_dcam_pm)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_cfg_store_addr slw_addr = {0};
	uint32_t port_id = 0;
	unsigned long addr = 0;
	const int _hist = 2;
	int i = 0;
	uint32_t slm_path = 0;

	hw = hw_ctx->hw;
	port_id = dcam_port->port_id;

	/* assign last buffer for AEM and HIST in slow motion */
	i = hw_ctx->slowmotion_count - 1;
	if (hw_ctx->slowmotion_count) {
		/* slow motion HIST */
		addr = slowmotion_store_addr[_hist][i];
		frame->fid += i;
	}

	dcamonline_port_update_addr_and_size(dcam_port, frame, hw_ctx, NULL, idx, blk_dcam_pm);
	dcamonline_port_hist_update_statis_head(dcam_port, frame, hw_ctx, blk_dcam_pm);

	slm_path = hw->ip_dcam[idx]->slm_path;
	if (hw_ctx->slowmotion_count && !hw_ctx->index_to_set && (slm_path & BIT(port_id))) {
		/* configure reserved buffer for AEM and hist */
		frame = dcamonline_port_reserved_buf_get(dcam_port);
		if (!frame) {
			pr_debug("DCAM%u %s buffer unavailable\n",
				idx, cam_port_name_get(port_id));
			return -ENOMEM;
		}
		i = 1;
		while (i < hw_ctx->slowmotion_count) {
			addr = slowmotion_store_addr[_hist][i];
			slw_addr.reg_addr = addr;
			slw_addr.idx = idx;
			slw_addr.frame_addr[0] = frame->buf.iova;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLW_ADDR, &slw_addr);
			pr_debug("DCAM%u %s set reserved frame\n", idx, cam_port_name_get(port_id));
			i++;
		}

		/* put it back */
		dcam_online_port_reserved_buf_set(dcam_port, frame);
	}
	return ret;
}

static inline int dcamonline_port_aem_out_frm_set(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx, struct dcam_isp_k_block *blk_dcam_pm)
{
	int ret = 0;
	struct dcam_hw_cfg_store_addr slw_addr = {0};
	struct cam_hw_info *hw = NULL;
	uint32_t port_id = 0;
	unsigned long addr = 0;
	const int _aem = 1;
	int i = 0;
	uint32_t slm_path = 0;

	hw = hw_ctx->hw;
	port_id = dcam_port->port_id;

	/* assign last buffer for AEM and HIST in slow motion */
	i = hw_ctx->slowmotion_count - 1;
	if (hw_ctx->slowmotion_count) {
		/* slow motion AEM */
		addr = slowmotion_store_addr[_aem][i];
		frame->fid += i;
	}
	dcamonline_port_update_addr_and_size(dcam_port, frame, hw_ctx, NULL, idx, blk_dcam_pm);
	dcamonline_port_aem_update_statis_head(dcam_port, frame, hw_ctx, blk_dcam_pm);
	slm_path = hw->ip_dcam[idx]->slm_path;
	if (hw_ctx->slowmotion_count && !hw_ctx->index_to_set && (slm_path & BIT(port_id))) {
		/* configure reserved buffer for AEM and hist */
		frame = dcamonline_port_reserved_buf_get(dcam_port);
		if (!frame) {
			pr_debug("DCAM%u %s buffer unavailable\n",
				idx, cam_port_name_get(port_id));
			return -ENOMEM;
		}
		i = 1;
		while (i < hw_ctx->slowmotion_count) {
			addr = slowmotion_store_addr[_aem][i];
			slw_addr.reg_addr = addr;
			slw_addr.idx = idx;
			slw_addr.frame_addr[0] = frame->buf.iova;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLW_ADDR, &slw_addr);
			pr_debug("DCAM%u %s set reserved frame\n", idx, cam_port_name_get(port_id));
			i++;
		}

		/* put it back */
		dcam_online_port_reserved_buf_set(dcam_port, frame);
	}

	return ret;
}

static inline int dcamonline_port_bin_out_frm_set(struct dcam_online_port *dcam_port,
	struct camera_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx, struct dcam_isp_k_block *blk_dcam_pm)
{
	int ret = 0, i = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_cfg_store_addr slw_addr = {0};
	uint32_t port_id = 0;
	const int _bin = 0;
	unsigned long addr = 0;

	hw = hw_ctx->hw;
	port_id = dcam_port->port_id;

	dcamonline_port_update_addr_and_size(dcam_port, frame, hw_ctx, NULL, idx, blk_dcam_pm);

	if (hw_ctx->slowmotion_count) {
		i = 1;
		while (i < hw_ctx->slowmotion_count) {
			frame = dcamonline_port_frame_cycle(dcam_port, hw_ctx);
			/* in init phase, return failure if error happens */
			if (IS_ERR(frame) && !hw_ctx->index_to_set) {
				ret = PTR_ERR(frame);
			}

			/* in normal running, just stop configure */
			if (IS_ERR(frame))
				break;

			addr = slowmotion_store_addr[_bin][i];
			slw_addr.reg_addr = addr;
			slw_addr.idx = idx;
			slw_addr.frame_addr[0] = frame->buf.iova;
			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLW_ADDR, &slw_addr);
			atomic_inc(&dcam_port->set_frm_cnt);

			frame->fid = hw_ctx->base_fid + hw_ctx->index_to_set + i;

			pr_debug("DCAM%u set frame: fid %u, count %d\n", idx, frame->fid,
				atomic_read(&dcam_port->set_frm_cnt));
			i++;
		}

		if (unlikely(i != hw_ctx->slowmotion_count))
			pr_warn("warning: DCAM%u BIN %d frame missed\n",
				idx, hw_ctx->slowmotion_count - i);
	}
	return ret;
}

static inline struct camera_frame *
	dcamonline_port_slw_frame_cycle(struct dcam_online_port *dcam_port, struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	int slw_idx = 0;
	int port_id = 0;
	struct camera_frame *out_frame = NULL;

	if (!dcam_port || !hw_ctx) {
		pr_err("fail to get valid inptr %px %px\n", dcam_port, hw_ctx);
		return NULL;
	}

	port_id = dcam_port->port_id;
	slw_idx = hw_ctx->slw_idx;

	if (hw_ctx->slw_idx == 0) {
		out_frame = dcamonline_port_frame_cycle(dcam_port, hw_ctx);
		goto exit;
	}
	switch (port_id) {
	case PORT_RAW_OUT:
	case PORT_FULL_OUT:
	case PORT_BIN_OUT:
	case PORT_PDAF_OUT:
	case PORT_VCH2_OUT:
	case PORT_VCH3_OUT:
		out_frame = dcamonline_port_frame_cycle(dcam_port, hw_ctx);
		pr_debug("enqueue bin path buffer, no.%d, cnt %d\n", slw_idx, cam_buf_manager_pool_cnt(&dcam_port->result_pool));
		break;
	default:
		out_frame = dcamonline_port_reserved_buf_get(dcam_port);
		if (out_frame == NULL) {
			pr_err("fail to get reserve buffer, port %s\n", cam_port_name_get(port_id));
			return ERR_PTR(-ENOMEM);
		}
		ret = cam_buf_manager_buf_enqueue(&dcam_port->result_pool, out_frame, NULL);
		if (ret) {
			dcam_online_port_reserved_buf_set(dcam_port, out_frame);
			return ERR_PTR(-ENOMEM);
		}
		break;
	}

exit:
	if (IS_ERR(out_frame))
		return ERR_PTR(-ENOMEM);
	atomic_inc(&dcam_port->set_frm_cnt);

	out_frame->fid = hw_ctx->base_fid + hw_ctx->index_to_set + hw_ctx->slw_idx;
	out_frame->need_pyr_dec = 0;
	out_frame->need_pyr_rec = 0;
	out_frame->pyr_status = PYR_OFF;

	return out_frame;
}

static int dcamonline_port_base_cfg(struct dcam_online_port *port, struct dcam_online_port_desc *param)
{
	int ret = 0;
	unsigned long flags = 0;

	if (!port || !param) {
		pr_err("fail to get valid param, port=%p, param=%p.\n", port, param);
		return -EFAULT;
	}

	if (port->data_cb_func == NULL) {
		port->data_cb_func = param->data_cb_func;
		port->data_cb_handle = param->data_cb_handle;
	}
	if (port->resbuf_get_cb == NULL) {
		port->resbuf_get_cb = param->resbuf_get_cb;
		port->resbuf_cb_data = param->resbuf_cb_data;
	}

	if (port->sharebuf_get_cb == NULL) {
		port->sharebuf_get_cb = param->sharebuf_get_cb;
		port->sharebuf_cb_data = param->sharebuf_cb_data;
	}

	port->frm_skip = param->frm_skip;
	port->endian = param->endian;
	port->bayer_pattern = param->bayer_pattern;
	port->dcamout_fmt = param->dcam_out_fmt;

	switch (port->port_id) {
	case PORT_FULL_OUT:
		spin_lock_irqsave(&port->size_lock, flags);
		port->compress_en = param->compress_en;
		port->raw_src = param->is_raw ? ORI_RAW_SRC_SEL : PROCESS_RAW_SRC_SEL;
		port->base_update = 1;
		pr_info("full path out fmt %s\n",camport_fmt_name_get(port->dcamout_fmt));
		spin_unlock_irqrestore(&port->size_lock, flags);
		atomic_set(&port->is_work, 1);
		break;
	case PORT_BIN_OUT:
		port->compress_en = param->compress_en;
		port->pyr_out_fmt = param->pyr_out_fmt;
		pr_info("bin path out fmt %s\n",camport_fmt_name_get(port->dcamout_fmt));
		atomic_set(&port->is_work, 1);
		break;
	case PORT_RAW_OUT:
		port->raw_src = param->is_raw ? ORI_RAW_SRC_SEL : param->raw_src;
		pr_info("raw path src %d, raw path out fmt %s\n", port->raw_src, camport_fmt_name_get(port->dcamout_fmt));
		atomic_set(&port->is_work, 1);
		break;
	case PORT_VCH2_OUT:
		port->raw_src = param->is_raw ? 1 : 0;
		break;
	default:
		pr_debug("get known path %s\n", cam_port_name_get(port->port_id));
		break;
	}

	return ret;
}

static int dcamonline_port_slw_store_set(void *handle, void *param)
{
	int ret = 0, slw_idx = 0, port_id = 0, path_id = 0;
	struct dcam_online_port *dcam_port = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct camera_frame *out_frame = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_slw_fmcu_cmds *slw = NULL;
	uint32_t idx = 0, addr_ch = 0;
	struct dcam_isp_k_block *blk_dcam_pm = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}
	dcam_port = (struct dcam_online_port *)handle;
	hw_ctx = (struct dcam_hw_context *)param;

	hw = hw_ctx->hw;
	port_id = dcam_port->port_id;
	slw = &hw_ctx->slw;
	slw_idx = hw_ctx->slw_idx;
	idx = hw_ctx->hw_ctx_id;
	blk_dcam_pm = hw_ctx->blk_pm;

	if (idx >= DCAM_HW_CONTEXT_MAX || !blk_dcam_pm)
		return 0;

	if (atomic_read(&dcam_port->is_work) < 1 || atomic_read(&dcam_port->is_shutoff) > 0)
		return ret;

	out_frame = dcamonline_port_slw_frame_cycle(dcam_port, hw_ctx);
	path_id = dcamonline_portid_convert_to_pathid(port_id);
	if (out_frame->is_compressed) {
		struct compressed_addr fbc_addr;
		struct img_size *size = &dcam_port->out_size;
		struct dcam_compress_cal_para cal_fbc;

		cal_fbc.data_bits = cam_data_bits(dcam_port->dcamout_fmt);
		cal_fbc.fbc_info = &out_frame->fbc_info;
		cal_fbc.in = out_frame->buf.iova;
		cal_fbc.fmt = dcam_port->dcamout_fmt;
		cal_fbc.height = size->h;
		cal_fbc.width = size->w;
		cal_fbc.out = &fbc_addr;
		dcam_if_cal_compressed_addr(&cal_fbc);
		slw->store_info[path_id].is_compressed = out_frame->is_compressed;
		slw->store_info[path_id].store_addr.addr_ch0 = fbc_addr.addr0;
		slw->store_info[path_id].store_addr.addr_ch1 = fbc_addr.addr1;
		slw->store_info[path_id].store_addr.addr_ch2 = fbc_addr.addr2;
	} else {
		slw->store_info[path_id].store_addr.addr_ch0 = out_frame->buf.iova;

		if (dcam_port->dcamout_fmt & CAM_YUV_BASE)
			addr_ch = out_frame->buf.iova + dcam_port->out_pitch * dcam_port->out_size.h;

		slw->store_info[path_id].store_addr.addr_ch1 = addr_ch;
	}

	dcamonline_port_update_addr_and_size(dcam_port, out_frame, hw_ctx, slw, idx, blk_dcam_pm);
	switch (port_id) {
	case PORT_AEM_OUT:
		dcamonline_port_aem_update_statis_head(dcam_port, out_frame, hw_ctx, blk_dcam_pm);
		break;
	case PORT_FRGB_HIST_OUT:
		dcamonline_port_hist_update_statis_head(dcam_port, out_frame, hw_ctx, blk_dcam_pm);
		break;
	case PORT_AFM_OUT:
		if (slw && blk_dcam_pm) {
			uint32_t w = 0, h = 0;
			w = blk_dcam_pm->afm.win_num.width;
			h = blk_dcam_pm->afm.win_num.height;
			slw->store_info[path_id].store_addr.addr_ch1 = out_frame->buf.iova + w * h * 16;
		}
		break;
	}
	pr_debug("path%s set no.%d buffer done!pitch:%d.\n", cam_port_name_get(port_id), hw_ctx->slw_idx, dcam_port->out_pitch);
	return ret;
}

int dcamonline_port_store_set(void *handle, void *param)
{
	int ret = 0;
	struct dcam_online_port *dcam_port = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct camera_frame *frame = NULL;
	struct dcam_isp_k_block *blk_dcam_pm = NULL;
	uint32_t idx = 0;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	dcam_port = (struct dcam_online_port *)handle;
	hw_ctx = (struct dcam_hw_context *)param;

	if (!hw_ctx) {
		pr_err("fail to get hw ctx\n");
		return -EFAULT;
	}

	idx = hw_ctx->hw_ctx_id;
	blk_dcam_pm = hw_ctx->blk_pm;

	if(idx >= DCAM_HW_CONTEXT_MAX || !blk_dcam_pm)
		return 0;

	pr_debug("DCAM%u ONLINE %s enter\n", idx, cam_port_name_get(dcam_port->port_id));
	if (dcam_port->port_id == PORT_GTM_HIST_OUT)
		dcamonline_port_check_status(dcam_port, hw_ctx, blk_dcam_pm);
	frame = dcamonline_port_frame_cycle(dcam_port, hw_ctx);
	if (IS_ERR_OR_NULL(frame))
		return PTR_ERR(frame);

	atomic_inc(&dcam_port->set_frm_cnt);

	pr_debug("dcam%u, fid %u, count %d, port out_size %d %d, is_reserver %d, channel_id %d, addr %08x, fbc %u\n",
		idx, frame->fid, atomic_read(&dcam_port->set_frm_cnt), dcam_port->out_size.w, dcam_port->out_size.h,
		frame->is_reserved, frame->channel_id, (uint32_t)frame->buf.iova, frame->is_compressed);

	switch (dcam_port->port_id) {
	case PORT_BIN_OUT:
		ret = dcamonline_port_bin_out_frm_set(dcam_port, frame, hw_ctx, idx, blk_dcam_pm);
		break;
	case PORT_AEM_OUT:
		ret = dcamonline_port_aem_out_frm_set(dcam_port, frame, hw_ctx, idx, blk_dcam_pm);
		break;
	case PORT_BAYER_HIST_OUT:
		ret = dcamonline_port_hist_out_frm_set(dcam_port, frame, hw_ctx,idx, blk_dcam_pm);
		break;
	case PORT_FULL_OUT:
	case PORT_PDAF_OUT:
	case PORT_AFL_OUT:
	default:
		if (dcam_port->port_id == PORT_GTM_HIST_OUT)
			return 0;
		dcamonline_port_update_addr_and_size(dcam_port, frame, hw_ctx, NULL, idx, blk_dcam_pm);
		break;
	}
	return ret;

}

static int dcamonline_port_cfg_callback(void *param, uint32_t cmd, void *handle)
{
	int ret = 0;
	struct camera_frame **frame = NULL;
	struct dcam_online_port *dcam_port = NULL;
	struct camera_buf_get_desc buf_desc = {0};

	dcam_port = (struct dcam_online_port *)handle;
	switch (cmd) {
	case DCAM_PORT_STORE_SET:
		ret = dcamonline_port_store_set(dcam_port, param);
		break;
	case DCAM_PORT_SLW_STORE_SET:
		ret = dcamonline_port_slw_store_set(dcam_port, param);
		break;
	case DCAM_PORT_BUFFER_CFG_SET:
		ret = dcamonline_port_buffer_cfg(dcam_port, param);
		break;
	case DCAM_PORT_BUFFER_CFG_GET:
		frame = (struct camera_frame **)param;
		*frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, NULL);
		break;
	case DCAM_PORT_BUFFER_CFG_OUTBUF_GET:
		frame = (struct camera_frame **)param;
		*frame = cam_buf_manager_buf_dequeue(&dcam_port->unprocess_pool, NULL);
		break;
	case DCAM_PORT_NEXT_FRM_BUF_GET:
		frame = (struct camera_frame **)param;
		buf_desc.q_ops_cmd = CAM_QUEUE_TAIL_PEEK;
		*frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, &buf_desc);
		break;
	case DCAM_PORT_RES_BUF_CFG_SET:
		ret = dcam_online_port_reserved_buf_set(dcam_port, param);
		break;
	default:
		pr_err("fail to support port type %d\n", cmd);
		ret = -EFAULT;
		break;
	}
	return ret;
}

int inline dcam_online_port_reserved_buf_set(struct dcam_online_port *dcam_port, struct camera_frame *frame)
{
	struct camera_buf_get_desc buf_desc = {0};
	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_PUT_IOVA;
	frame->priv_data = NULL;
	frame->is_compressed = 0;
	return cam_buf_manager_buf_enqueue(&dcam_port->reserved_pool, frame, &buf_desc);
}

int dcam_online_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct dcam_online_port *dcam_port = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}
	dcam_port = (struct dcam_online_port *)handle;
	if (!dcam_port) {
		pr_err("fail to get dcam online port ptr %px\n", dcam_port);
		return -EFAULT;
	}

	switch (cmd) {
	case PORT_BUFFER_CFG_SET:
		ret = dcamonline_port_buffer_cfg(dcam_port, param);
		break;
	case PORT_SIZE_CFG_SET:
		ret = dcamonline_port_size_cfg(dcam_port, param);
		break;
	default:
		pr_err("fail to support port type %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

int dcam_online_port_skip_num_set(void *dcam_ctx_handle, uint32_t hw_id,
		int path_id, uint32_t skip_num)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)dcam_ctx_handle;
	struct dcam_hw_context *dcam_hw_ctx = NULL;
	struct dcam_hw_path *hw_path = NULL;

	dcam_hw_ctx = &dev->hw_ctx[hw_id];
	if (unlikely(!dcam_hw_ctx || !is_path_id(path_id)))
		return -EINVAL;

	hw_path = &dcam_hw_ctx->hw_path[path_id];
	hw_path->frm_deci = skip_num;
	hw_path->frm_deci_cnt = 0;

	pr_debug("DCAM%u %d set skip_num %u\n", dcam_hw_ctx->hw_ctx_id, path_id, skip_num);

	return 0;
}

int dcam_online_port_buf_alloc(void *handle, struct cam_buf_alloc_desc *param)
{
	int ret = 0;
	uint32_t i = 0, total = 0, size = 0, pitch = 0, width = 0, height = 0, ch_id = 0, iommu_enable = 0;
	uint32_t dcam_out_bits = 0, pyr_data_bits = 0, pyr_is_pack = 0, is_pack = 0, pack_bits = 0;
	struct dcam_online_port *port = (struct dcam_online_port *)handle;
	struct camera_frame *pframe = NULL;
	struct dcam_compress_info fbc_info = {0};
	struct dcam_compress_cal_para cal_fbc = {0};
	struct camera_buf_get_desc buf_desc = {0};
	struct cam_buf_pool_id share_buffer_q = {0};

	dcam_out_bits = cam_data_bits(port->dcamout_fmt);
	pack_bits = cam_pack_bits(port->dcamout_fmt);
	height = param->height;
	width = param->width;
	is_pack = cam_is_pack(port->dcamout_fmt);
	ch_id = param->ch_id;
	iommu_enable = param->iommu_enable;
	if (param->is_pyr_rec) {
		pyr_data_bits = cam_data_bits(port->pyr_out_fmt);
		pyr_is_pack = cam_is_pack(port->pyr_out_fmt);
	}
	if (param->compress_en) {
		 cal_fbc.data_bits = dcam_out_bits;
		 cal_fbc.fbc_info = &fbc_info;
		 cal_fbc.fmt = port->dcamout_fmt;
		 cal_fbc.height = height;
		 cal_fbc.width = width;
		 size = dcam_if_cal_compressed_size (&cal_fbc);
		 pr_debug("dcam fbc buffer size %u\n", size);
	} else if (camcore_raw_fmt_get(port->dcamout_fmt)) {
		size = cal_sprd_raw_pitch(width, pack_bits) * height;
		pr_debug("channel %d, raw size %d\n", ch_id, size);
	} else if ((port->dcamout_fmt == CAM_YUV420_2FRAME) || (port->dcamout_fmt == CAM_YVU420_2FRAME) ||
		(port->dcamout_fmt == CAM_YUV420_2FRAME_MIPI)) {
		pitch = cal_sprd_yuv_pitch(width, dcam_out_bits, is_pack);
		size = pitch * height * 3 / 2;
		pr_debug("ch%d, dcam yuv size %d\n", ch_id, size);
	} else
		size = width * height * 3;

	if (param->is_pyr_rec && port->port_id == PORT_BIN_OUT)
		size += dcam_if_cal_pyramid_size(width, height,
			pyr_data_bits, pyr_is_pack, 1, DCAM_PYR_DEC_LAYER_NUM);
	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);

	/* TBD: need to add buf_desc*/
	total = param->buf_alloc_num;

	if (param->share_buffer && (port->port_id == PORT_FULL_OUT)) {
		uint32_t share_buf_num = 0;
		share_buffer_q.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
		share_buf_num = cam_buf_manager_pool_cnt(&share_buffer_q);
		if (share_buf_num >= total)
			return 0;
	}

	pr_info("port:%d, cam%d, ch_id %d, camsec=%d, buffer size: %u (%u x %u), fmt %s, pyr fmt %s, num %d\n",
		port->port_id, param->cam_idx, ch_id, param->sec_mode,
		size, width, height, camport_fmt_name_get(param->dcamonline_out_fmt), camport_fmt_name_get(param->pyr_out_fmt), total);

	if (port->port_id == PORT_BIN_OUT) {
		buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
		buf_desc.mmu_type = CAM_IOMMUDEV_DCAM;
	}

	for (i = 0; i < total; i++) {
		pframe = cam_queue_empty_frame_get();
		pframe->channel_id = ch_id;
		pframe->is_compressed = param->compress_en;
		pframe->width = param->width;
		pframe->height = param->height;
		pframe->endian = ENDIAN_LITTLE;
		pframe->pattern = param->sensor_img_ptn;
		pframe->fbc_info = fbc_info;
		cam_queue_frame_flag_reset(pframe);
		if (port->port_id == PORT_BIN_OUT && param->sec_mode != SEC_UNABLE)
			pframe->buf.buf_sec = 1;
		if (param->is_pyr_rec && port->port_id == PORT_BIN_OUT)
			pframe->pyr_status = ONLINE_DEC_ON;
		if (param->is_pyr_dec && port->port_id == PORT_FULL_OUT)
			pframe->pyr_status = OFFLINE_DEC_ON;

		ret = cam_buf_alloc(&pframe->buf, size, iommu_enable);
		if (ret) {
			pr_err("fail to alloc buf: %d ch %d\n", i, ch_id);
			cam_queue_empty_frame_put(pframe);
			continue;
		}

		if (param->share_buffer && (port->port_id == PORT_FULL_OUT))
			ret = cam_buf_manager_buf_enqueue(&share_buffer_q, pframe, NULL);
		else
			ret = cam_buf_manager_buf_enqueue(&port->unprocess_pool, pframe, &buf_desc);

		if (ret) {
			pr_err("fail to enq, port %d, buffer i=%d\n", port->port_id, i);
			cam_buf_destory(pframe);
		}
	}

	return ret;
}

void *dcam_online_port_get(uint32_t port_id, struct dcam_online_port_desc *param)
{
	int ret = 0;
	struct dcam_online_port *port = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	pr_debug("port id %s param %px node_dev %px\n", cam_port_name_get(port_id), param, *param->port_dev);
	if (*param->port_dev == NULL) {
		port = cam_buf_kernel_sys_vzalloc(sizeof(struct dcam_online_port));
		if (!port) {
			pr_err("fail to get valid dcam online port %d\n", port_id);
			return NULL;
		}
	} else if (param->is_raw && (port_id == PORT_VCH2_OUT)) {
		port = *param->port_dev;
		dcamonline_port_base_cfg(port, param);
		atomic_set(&port->is_work, 1);
		pr_debug("dcam online vch2 has been alloc %p %s\n", port, cam_port_name_get(port_id));
		goto exit;
	} else {
		port = *param->port_dev;
		pr_debug("dcam online port has been alloc %p %s\n", port, cam_port_name_get(port_id));
		goto exit;
	}

	port->port_id = port_id;
	atomic_set(&port->user_cnt, 0);
	atomic_set(&port->set_frm_cnt, 0);
	atomic_set(&port->is_work, 0);
	atomic_set(&port->is_shutoff, 0);
	spin_lock_init(&port->size_lock);
	spin_lock_init(&port->state_lock);
	port->port_cfg_cb_func = dcamonline_port_cfg_callback;

	port->shutoff_cb_func = param->shutoff_cb_func;
	port->shutoff_cb_handle = param->shutoff_cb_handle;

	dcamonline_port_base_cfg(port, param);

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OUT_BUF_Q_LEN);
	if (ret < 0) {
		pr_err("fail to reg pool for port%d\n", port->port_id);
		cam_buf_kernel_sys_vfree(port);
		return NULL;
	}
	port->unprocess_pool.private_pool_idx = ret;

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OUT_BUF_Q_LEN);
	if (ret < 0) {
		pr_err("fail to reg pool for port%d\n", cam_port_name_get(port->port_id));
		cam_buf_manager_pool_unreg(&port->unprocess_pool);
		cam_buf_kernel_sys_vfree(port);
		return NULL;
	}
	port->result_pool.private_pool_idx = ret;

	port->reserved_pool.reserved_pool_id = param->reserved_pool_id;
	if (port->port_id == PORT_FULL_OUT)
		port->share_full_path = param->share_full_path;
	else
		port->share_full_path = 0;

	*param->port_dev = port;
	pr_info("port %s reg pool %d %d\n", cam_port_name_get(port_id), port->unprocess_pool.private_pool_idx, port->result_pool.private_pool_idx);

exit:
	port->data_cb_handle = param->data_cb_handle;
	atomic_inc(&port->user_cnt);
	return port;
}

void dcam_online_port_put(struct dcam_online_port *port)
{
	uint32_t port_id = 0;
	struct isp_offline_param *cur = NULL;
	struct isp_offline_param *prev = NULL;

	if (!port) {
		pr_err("fail to get invalid port ptr\n");
		return;
	}

	if (atomic_dec_return(&port->user_cnt) == 0) {
		port_id = port->port_id;
		if (port->isp_updata) {
			cur = (struct isp_offline_param *)port->isp_updata;
			port->isp_updata = NULL;
			while (cur) {
				prev = (struct isp_offline_param *)cur->prev;
				cam_buf_kernel_sys_vfree(cur);
				cur = prev;
			}
		}

		if (port->priv_size_data) {
			cur = (struct isp_offline_param *)port->priv_size_data;
			port->priv_size_data = NULL;
			while (cur) {
				prev = (struct isp_offline_param *)cur->prev;
				cam_buf_kernel_sys_vfree(cur);
				cur = prev;
			}
		}
		if (port->share_full_path) {
			struct cam_buf_pool_id pool_id = {0};
			struct camera_buf_get_desc buf_desc = {0};
			struct camera_frame *frame_unprocess = NULL;
			struct camera_frame *frame_result_pool = NULL;

			pool_id.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
			buf_desc.buf_ops_cmd = CAM_BUF_STATUS_PUT_IOVA;

			do {
				frame_unprocess = cam_buf_manager_buf_dequeue(&port->unprocess_pool, &buf_desc);
				if (!frame_unprocess)
					break;
				cam_buf_manager_buf_enqueue(&pool_id, frame_unprocess, &buf_desc);
			} while(1);

			do {
				frame_result_pool = cam_buf_manager_buf_dequeue(&port->result_pool, &buf_desc);
				if (!frame_result_pool)
					break;
				cam_buf_manager_buf_enqueue(&pool_id, frame_result_pool, &buf_desc);
			} while(1);
		}

		port->data_cb_func = NULL;
		port->resbuf_get_cb = NULL;
		port->sharebuf_get_cb = NULL;
		port->port_cfg_cb_func = NULL;
		cam_buf_manager_pool_unreg(&port->unprocess_pool);
		pr_info("unreg unprocess_pool_id %d\n", port->unprocess_pool.private_pool_idx);
		cam_buf_manager_pool_unreg(&port->result_pool);
		pr_info("unreg result_pool_id %d\n", port->result_pool.private_pool_idx);

		pr_debug("dcam online port %s put success\n", cam_port_name_get(port->port_id));
		cam_buf_kernel_sys_vfree(port);
		port = NULL;
	}
}
