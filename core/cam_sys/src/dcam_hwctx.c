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

#include "cam_offline_statis.h"
#include "cam_pipeline.h"
#include "cam_zoom.h"
#include "dcam_dummy.h"
#include "dcam_slice.h"
#include "dcam_hwctx.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_HWCTX: %d %d %s : " fmt, current->pid, __LINE__, __func__

int dcam_hwctx_offline_reset(struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	hw_ctx->fid = 0;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_RESET, &hw_ctx->hw_ctx_id);
	if (ret)
		pr_err("fail to reset dcam%d\n", hw_ctx->hw_ctx_id);
	return ret;
}

void dcam_hwctx_nr3_store_addr(struct dcam_hw_context *hw_ctx, struct cam_frame *frame)
{
	struct dcam_hw_cfg_store_addr store_arg = {0};
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	store_arg.idx = hw_ctx->hw_ctx_id;
	store_arg.frame_addr[0] = frame->common.buf.iova[CAM_BUF_IOMMUDEV_DCAM];
	store_arg.path_id = DCAM_PATH_3DNR;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_STORE_ADDR, &store_arg);
}

void dcam_hwctx_binning_4in1_set(struct dcam_hw_context *hw_ctx)
{
	struct cam_hw_info *hw = NULL;
	hw = hw_ctx->hw;
	hw_ctx->binning.binning_4in1_en = 0;
	hw_ctx->binning.idx = hw_ctx->hw_ctx_id;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BINNING_4IN1_SET, &hw_ctx->binning);
}

void dcam_hwctx_block_set(struct dcam_hw_context *hw_ctx)
{
	struct cam_hw_info *hw = NULL;
	hw = hw_ctx->hw;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETSTATIS, hw_ctx->blk_pm);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETALL, hw_ctx->blk_pm);
}

void dcam_hwctx_frame_param_set(struct dcam_hw_context *hw_ctx)
{
	int i = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	for (i = 0; i < DCAM_PATH_MAX; i++) {
		if (!hw_ctx->hw_path[i].need_update)
			continue;
		pr_debug("dcam path id %d update, compress_en %d\n", i, hw_ctx->hw_path[i].hw_fbc.compress_en);
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_ADDR_SET, &hw_ctx->hw_path[i].hw_fbc_store);
		else
			hw->dcam_ioctl(hw, DCAM_HW_CFG_STORE_ADDR, &hw_ctx->hw_path[i].hw_store);

		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &hw_ctx->hw_path[i].hw_size);
		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &hw_ctx->hw_path[i].hw_start);
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_CTRL, &hw_ctx->hw_path[i].hw_fbc);
	}
}

int dcam_hwctx_fetch_set(struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_SET, &hw_ctx->hw_fetch);
	return ret;
}

void dcam_hwctx_slice_fetch_set(struct dcam_hw_context *hw_ctx, struct dcam_fetch_info *fetch, struct dcam_offline_slice_info *slice)
{
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_slice_fetch slicearg = {0};
	hw = hw_ctx->hw;

	slicearg.idx = hw_ctx->hw_ctx_id;
	slicearg.fetch = fetch;
	slicearg.cur_slice = slice->cur_slice;
	slicearg.dcam_slice_mode = slice->dcam_slice_mode;
	slicearg.slice_num = slice->slice_num;
	slicearg.slice_count = slice->slice_count;
	slicearg.st_pack = cam_is_pack(hw_ctx->hw_path[DCAM_PATH_FULL].hw_start.out_fmt);
	if (hw_ctx->slice_proc_mode == OFFLINE_SLICE_PROC) {
		slicearg.path_id = hw->ip_dcam[hw_ctx->hw_ctx_id]->dcamhw_abt->aux_dcam_path;
		slicearg.is_compress = hw_ctx->hw_path[DCAM_PATH_FULL].hw_fbc.compress_en;
		slicearg.store_trim = hw_ctx->hw_path[DCAM_PATH_FULL].hw_size.in_trim;
	}else {
		slicearg.virtualsensor_pre_sof = 1;
		slicearg.path_id= hw->ip_dcam[1]->dcamhw_abt->aux_dcam_path;
	}
	hw->dcam_ioctl(hw, DCAM_HW_CFG_SLICE_FETCH_SET, &slicearg);
}

void dcam_hwctx_slice_force_copy(struct dcam_hw_context *hw_ctx, int x)
{
	uint32_t force_ids = DCAM_CTRL_ALL;
	struct dcam_hw_force_copy copyarg = {0};
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace = {0};
	hw = hw_ctx->hw;

	copyarg.id = force_ids;
	copyarg.idx = hw_ctx->hw_ctx_id;
	copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);
	os_adapt_time_udelay(500);
	if (x == 0) {
		trace.type = NORMAL_REG_TRACE;
		trace.idx = hw_ctx->hw_ctx_id;
		hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
	}
}

void dcam_hwctx_cfg_fetch_start(struct cam_hw_info *hw)
{
	hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_START, hw);
}

int dcam_hwctx_slw_fmcu_set(struct dcam_hw_context *hw_ctx, uint32_t hw_ctx_id, int j)
{
	int ret = 0;
	struct dcam_fmcu_ctx_desc *fmcu;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	fmcu = hw_ctx->fmcu;
	if (!fmcu)
		pr_err("fail to check param fmcu%px\n", fmcu);
	hw_ctx->slw.fmcu_handle = fmcu;
	hw_ctx->slw.ctx_id = hw_ctx_id;
	hw_ctx->slw.slw_id = j;
	hw_ctx->slw.slw_cnt = hw_ctx->slowmotion_count;
	if ((hw_ctx->index_to_set == 0) && (j == 0))
		hw_ctx->slw.is_first_cycle = 1;
	else
		hw_ctx->slw.is_first_cycle = 0;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_SLW_FMCU_CMDS, &hw_ctx->slw);
	return ret;
}

void dcam_hwctx_update_path_size(struct dcam_online_port *dcam_port,
	struct cam_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx)
{
	struct dcam_hw_path_size path_size = {0};
	struct cam_hw_info *hw;

	hw = hw_ctx->hw;

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
	path_size.compress_info = frame->common.fbc_info;
	path_size.deci = dcam_port->deci;
	path_size.scaler_info = &dcam_port->scaler_info;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &path_size);
}

int dcam_hwctx_pyr_dec_cfg(struct dcam_online_port *dcam_port,
	struct cam_frame *frame, struct dcam_hw_context *hw_ctx, uint32_t idx)
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

