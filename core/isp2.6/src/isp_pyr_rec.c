/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
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

#include "isp_pyr_rec.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_REC: %d %d %s : "fmt, current->pid, __LINE__, __func__

static int isppyrrec_ref_fetch_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t ch_offset[3] = { 0 };
	uint32_t mipi_word_num_start[16] = {0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	struct slice_fetch_info *slc_fetch = NULL;
	struct isp_rec_fetch_info *ref_fetch = NULL;
	struct isp_rec_slice_desc *cur_slc = NULL;

	if (!ctx) {
		pr_err("fail to get valid input ctx\n");
		return -EFAULT;
	}

	ref_fetch = &ctx->ref_fetch;
	ref_fetch->bypass = 0;
	ref_fetch->color_format = ctx->in_fmt;
	ref_fetch->addr[0] = ctx->store_addr[idx].addr_ch0;
	ref_fetch->addr[1] = ctx->store_addr[idx].addr_ch1;
	if (idx == ctx->layer_num) {
		ref_fetch->addr[0] = ctx->fetch_addr[idx].addr_ch0;
		ref_fetch->addr[1] = ctx->fetch_addr[idx].addr_ch1;
	}
	ref_fetch->width = ctx->pyr_layer_size[idx].w;
	ref_fetch->height = ctx->pyr_layer_size[idx].h;
	/* TBD: the pitch info need cfg by differnet fmt */
	ref_fetch->pitch[0] = ref_fetch->width * ref_fetch->height;
	ref_fetch->pitch[1] = ref_fetch->pitch[0];
	ref_fetch->chk_sum_clr_en = 0;
	ref_fetch->ft0_axi_reorder_en = 0;
	ref_fetch->ft1_axi_reorder_en = 0;
	ref_fetch->substract = 0;
	ref_fetch->ft0_max_len_sel = 1;
	ref_fetch->ft1_max_len_sel = 1;
	/* this default val come from spec need to confirm */
	ref_fetch->ft0_retain_num = 0x20;
	ref_fetch->ft1_retain_num = 0x20;

	cur_slc = &ctx->slices[0];
	for (i = 0; i < ctx->slice_num; i++, cur_slc++) {
		slc_fetch = &cur_slc->slice_ref_fetch;
		start_col = cur_slc->slice_fetch0_pos.start_col;
		start_row = cur_slc->slice_fetch0_pos.start_row;
		end_col = cur_slc->slice_fetch0_pos.end_col;
		end_row = cur_slc->slice_fetch0_pos.end_row;

		switch (ref_fetch->color_format) {
		case ISP_FETCH_YUV420_2FRAME:
		case ISP_FETCH_YVU420_2FRAME:
			ch_offset[0] = start_row * ref_fetch->pitch[0] + start_col;
			ch_offset[1] = (start_row >> 1) * ref_fetch->pitch[1] + start_col;
			break;
		case ISP_FETCH_YUV420_2FRAME_MIPI:
		case ISP_FETCH_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row * ref_fetch->pitch[0]
				+ (start_col >> 2) * 5 + (start_col & 0x3);
			slc_fetch->mipi_byte_rel_pos = start_col & 0x0f;
			slc_fetch->mipi_word_num = ((((end_col + 1) >> 4) * 5
				+ mipi_word_num_end[(end_col + 1) & 0x0f])
				-(((start_col + 1) >> 4) * 5
				+ mipi_word_num_start[(start_col + 1) & 0x0f]) + 1);
			slc_fetch->mipi_byte_rel_pos_uv = slc_fetch->mipi_byte_rel_pos;
			slc_fetch->mipi_word_num_uv = slc_fetch->mipi_word_num;
			slc_fetch->mipi10_en = 1;
			pr_debug("(%d %d %d %d), pitch %d, offset %d, mipi %d %d\n",
				start_row, start_col, end_row, end_col,
				ref_fetch->pitch[0], ch_offset[0],
				slc_fetch->mipi_byte_rel_pos, slc_fetch->mipi_word_num);
			break;
		default:
			ch_offset[0] = start_row * ref_fetch->pitch[0] + start_col * 2;
			break;
		}

		slc_fetch->addr.addr_ch0 = ref_fetch->addr[0] + ch_offset[0];
		slc_fetch->addr.addr_ch1 = ref_fetch->addr[1] + ch_offset[1];
		slc_fetch->size.h = end_row - start_row + 1;
		slc_fetch->size.w = end_col - start_col + 1;

		pr_debug("slice fetch size %d, %d\n", slc_fetch->size.w, slc_fetch->size.h);
	}

	return ret;
}

static int isppyrrec_cur_fetch_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t ch_offset[3] = { 0 };
	uint32_t mipi_word_num_start[16] = {0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	struct slice_fetch_info *slc_fetch = NULL;
	struct isp_rec_fetch_info *cur_fetch = NULL;
	struct isp_rec_slice_desc *cur_slc = NULL;

	if (!ctx || (idx < 1)) {
		pr_err("fail to get valid input ctx %p idx %d\n", ctx, idx);
		return -EFAULT;
	}

	cur_fetch = &ctx->cur_fetch;
	idx = idx - 1;
	cur_fetch->bypass = 0;
	cur_fetch->color_format = ctx->in_fmt;
	cur_fetch->addr[0] = ctx->fetch_addr[idx].addr_ch0;
	cur_fetch->addr[1] = ctx->fetch_addr[idx].addr_ch1;
	cur_fetch->width = ctx->pyr_layer_size[idx].w;
	cur_fetch->height = ctx->pyr_layer_size[idx].h;
	/* TBD: the pitch info need cfg by differnet fmt */
	cur_fetch->pitch[0] = cur_fetch->width * cur_fetch->height;
	cur_fetch->pitch[1] = cur_fetch->pitch[0];
	cur_fetch->chk_sum_clr_en = 0;
	cur_fetch->ft0_axi_reorder_en = 0;
	cur_fetch->ft1_axi_reorder_en = 0;
	cur_fetch->substract = 0;
	cur_fetch->ft0_max_len_sel = 1;
	cur_fetch->ft1_max_len_sel = 1;
	/* this default val come from spec need to confirm */
	cur_fetch->ft0_retain_num = 0;
	cur_fetch->ft1_retain_num = 0;

	cur_slc = &ctx->slices[0];
	for (i = 0; i < ctx->slice_num; i++, cur_slc++) {
		slc_fetch = &cur_slc->slice_cur_fetch;
		start_col = cur_slc->slice_fetch1_pos.start_col;
		start_row = cur_slc->slice_fetch1_pos.start_row;
		end_col = cur_slc->slice_fetch1_pos.end_col;
		end_row = cur_slc->slice_fetch1_pos.end_row;

		switch (cur_fetch->color_format) {
		case ISP_FETCH_YUV420_2FRAME:
		case ISP_FETCH_YVU420_2FRAME:
			ch_offset[0] = start_row * cur_fetch->pitch[0] + start_col;
			ch_offset[1] = (start_row >> 1) * cur_fetch->pitch[1] + start_col;
			break;
		case ISP_FETCH_YUV420_2FRAME_MIPI:
		case ISP_FETCH_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row * cur_fetch->pitch[0]
				+ (start_col >> 2) * 5 + (start_col & 0x3);
			slc_fetch->mipi_byte_rel_pos = start_col & 0x0f;
			slc_fetch->mipi_word_num = ((((end_col + 1) >> 4) * 5
				+ mipi_word_num_end[(end_col + 1) & 0x0f])
				-(((start_col + 1) >> 4) * 5
				+ mipi_word_num_start[(start_col + 1) & 0x0f]) + 1);
			slc_fetch->mipi_byte_rel_pos_uv = slc_fetch->mipi_byte_rel_pos;
			slc_fetch->mipi_word_num_uv = slc_fetch->mipi_word_num;
			pr_debug("(%d %d %d %d), pitch %d, offset %d, mipi %d %d\n",
				start_row, start_col, end_row, end_col,
				cur_fetch->pitch[0], ch_offset[0],
				slc_fetch->mipi_byte_rel_pos, slc_fetch->mipi_word_num);
			break;
		default:
			ch_offset[0] = start_row * cur_fetch->pitch[0] + start_col * 2;
			break;
		}

		slc_fetch->addr.addr_ch0 = cur_fetch->addr[0] + ch_offset[0];
		slc_fetch->addr.addr_ch1 = cur_fetch->addr[1] + ch_offset[1];
		slc_fetch->size.h = end_row - start_row + 1;
		slc_fetch->size.w = end_col - start_col + 1;

		pr_debug("slice fetch size %d, %d\n", slc_fetch->size.w, slc_fetch->size.h);
	}

	return ret;
}

static int isppyrrec_reconstruct_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	struct isp_rec_slice_desc *cur_slc = NULL;
	struct isp_pyr_rec_info *rec_cfg = NULL;
	struct slice_pyr_rec_info *slc_pyr_rec = NULL;

	if (!ctx) {
		pr_err("fail to get valid input ctx %p\n", ctx);
		return -EFAULT;
	}

	rec_cfg = &ctx->pyr_rec;
	rec_cfg->reconstruct_bypass = 0;
	rec_cfg->layer_num = idx -1;
	rec_cfg->pre_layer_width = ctx->pyr_layer_size[idx].w;
	rec_cfg->pre_layer_height = ctx->pyr_layer_size[idx].h;
	rec_cfg->cur_layer_width = ctx->pyr_layer_size[idx - 1].w;
	rec_cfg->cur_layer_height = ctx->pyr_layer_size[idx - 1].h;
	rec_cfg->out_width = ctx->pyr_layer_size[idx - 1].w;
	rec_cfg->out_height = ctx->pyr_layer_size[idx - 1].h;
	rec_cfg->hor_padding_num = ctx->pyr_padding_size.w;
	rec_cfg->ver_padding_num = ctx->pyr_padding_size.h;
	rec_cfg->drop_en = 0;
	rec_cfg->hor_padding_en = 0;
	rec_cfg->ver_padding_en = 0;
	if (rec_cfg->layer_num == 0) {
		if (rec_cfg->hor_padding_num) {
			rec_cfg->hor_padding_en = 1;
			rec_cfg->drop_en = 1;
		}
		if (rec_cfg->ver_padding_num) {
			rec_cfg->ver_padding_en = 1;
			rec_cfg->drop_en = 1;
		}
	}
	rec_cfg->reduce_flt_hblank = rec_cfg->hor_padding_num + 20;
	rec_cfg->reduce_flt_vblank = rec_cfg->ver_padding_num + 20;
	/* default param from spec */
	rec_cfg->hblank_num = 0x3c;
	rec_cfg->fifo0_nfull_num = 0x1388;
	rec_cfg->fifo1_nfull_num = 0x2710;
	rec_cfg->fifo2_nfull_num = 0x2bc;
	rec_cfg->fifo3_nfull_num = 0x578;
	rec_cfg->fifo4_nfull_num = 0x157c;
	rec_cfg->fifo5_nfull_num = 0x578;

	cur_slc = &ctx->slices[0];
	for (i = 0; i < ctx->slice_num; i++, cur_slc++) {
		slc_pyr_rec = &cur_slc->slice_pyr_rec;
		start_col = cur_slc->slice_fetch1_pos.start_col;
		start_row = cur_slc->slice_fetch1_pos.start_row;
		end_col = cur_slc->slice_fetch1_pos.end_col;
		end_row = cur_slc->slice_fetch1_pos.end_row;

		slc_pyr_rec->cur_layer.h= end_row - start_row + 1;
		slc_pyr_rec->cur_layer.w = end_col - start_col + 1;
		slc_pyr_rec->pre_layer.h = slc_pyr_rec->cur_layer.h / 2;
		slc_pyr_rec->pre_layer.w = slc_pyr_rec->cur_layer.w / 2;
		slc_pyr_rec->out.h = slc_pyr_rec->cur_layer.h;
		slc_pyr_rec->out.w = slc_pyr_rec->cur_layer.w;

		pr_debug("slice pyr_rec cur %d, %d pre %d %d out %d %d\n",
			slc_pyr_rec->cur_layer.w , slc_pyr_rec->cur_layer.h,
			slc_pyr_rec->pre_layer.w, slc_pyr_rec->pre_layer.h,
			slc_pyr_rec->out.w, slc_pyr_rec->out.h);
	}

	return ret;
}

static int isppyrrec_store_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t overlap_left = 0, overlap_up = 0;
	uint32_t overlap_right = 0, overlap_down = 0;
	uint32_t start_row_out = 0, start_col_out = 0;
	uint32_t ch_offset[3] = { 0 };
	struct isp_rec_slice_desc *cur_slc = NULL;
	struct slice_store_info *slc_rec_store = NULL;
	struct isp_rec_store_info *rec_store = NULL;

	if (!ctx || (idx < 1)) {
		pr_err("fail to get valid input ctx %p idx %d\n", ctx, idx);
		return -EFAULT;
	}

	rec_store = &ctx->rec_store;
	idx = idx - 1;
	rec_store->bypass = 0;
	rec_store->color_format = ctx->out_fmt;
	rec_store->addr[0] = ctx->fetch_addr[idx].addr_ch0;
	rec_store->addr[1] = ctx->fetch_addr[idx].addr_ch1;
	rec_store->width = ctx->pyr_layer_size[idx].w;
	rec_store->height = ctx->pyr_layer_size[idx].h;
	/* TBD: the pitch & data_10b & mipi_en info need cfg by differnet fmt */
	rec_store->pitch[0] = rec_store->width * rec_store->height;
	rec_store->pitch[1] = rec_store->pitch[0];
	rec_store->data_10b = 0;
	rec_store->mipi_en = 0;
	/* TBD: flip_en need from input param, when flip_en the address may need adjust */
	rec_store->flip_en = 0;
	rec_store->last_frm_en = 1;
	rec_store->mono_en = 0;
	rec_store->mirror_en = 0;
	rec_store->burst_len = 1;
	rec_store->speed2x = 1;
	rec_store->shadow_clr_sel = 1;
	rec_store->shadow_clr = 1;
	rec_store->rd_ctrl = 0;

	cur_slc = &ctx->slices[0];
	for (i = 0; i < ctx->slice_num; i++, cur_slc++) {
		slc_rec_store = &cur_slc->slice_rec_store;
		start_col = cur_slc->slice_store_pos.start_col;
		start_row = cur_slc->slice_store_pos.start_row;
		end_col = cur_slc->slice_store_pos.end_col;
		end_row = cur_slc->slice_store_pos.end_row;
		overlap_up = cur_slc->slice_overlap.overlap_up;
		overlap_down = cur_slc->slice_overlap.overlap_down;
		overlap_left = cur_slc->slice_overlap.overlap_left;
		overlap_right = cur_slc->slice_overlap.overlap_right;
		start_row_out = start_row + overlap_up;
		start_col_out = start_col + overlap_left;

		switch (rec_store->color_format) {
		case ISP_STORE_YUV420_2FRAME_MIPI:
		case ISP_STORE_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row_out * rec_store->pitch[0] + start_col_out * 5 /4;
			ch_offset[1] = ((start_row_out * rec_store->pitch[1]) >> 1) + start_col_out * 5 /4;
			break;
		case ISP_STORE_YUV420_2FRAME_10:
		case ISP_STORE_YVU420_2FRAME_10:
			ch_offset[0] = start_row_out * rec_store->pitch[0] + (start_col_out << 1);
			ch_offset[1] = ((start_row_out * rec_store->pitch[1]) >> 1) + (start_col_out << 1);
			break;
		default:
			ch_offset[0] = start_row_out * rec_store->pitch[0] + start_col_out;
			ch_offset[1] = ((start_row_out * rec_store->pitch[1]) >> 1) + start_col_out;
			break;
		}

		slc_rec_store->addr.addr_ch0 = rec_store->addr[0] + ch_offset[0];
		slc_rec_store->addr.addr_ch1 = rec_store->addr[1] + ch_offset[1];
		slc_rec_store->size.h = end_row - start_row + 1 - overlap_up - overlap_down;
		slc_rec_store->size.w = end_col - start_col + 1 - overlap_left - overlap_right;
		slc_rec_store->border.up_border = overlap_up;
		slc_rec_store->border.down_border = overlap_down;
		slc_rec_store->border.left_border = overlap_left;
		slc_rec_store->border.right_border = overlap_right;

		pr_debug("slice fetch size %d, %d\n", slc_rec_store->size.w, slc_rec_store->size.h);
	}

	return ret;
}

static int isppyrrec_block_cfg_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0;

	if (!ctx) {
		pr_err("fail to get valid input ctx\n");
		return -EFAULT;
	}

	ret = isppyrrec_ref_fetch_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec ref_fetch\n");
		return ret;
	}

	ret = isppyrrec_cur_fetch_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec cur_fetch\n");
		return ret;
	}

	ret = isppyrrec_reconstruct_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec reconstruct\n");
		return ret;
	}

	/* TBD dispatch/common/YNR/CNR/uvdelay */

	ret = isppyrrec_store_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec store\n");
		return ret;
	}

	return ret;
}

static int isppyrrec_slice_cfg_get(struct isp_rec_ctx_desc *ctx, uint32_t idx)
{
	int ret = 0;

	if (!ctx) {
		pr_err("fail to get valid input ctx\n");
		return -EFAULT;
	}

	ret = isppyrrec_ref_fetch_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec ref_fetch\n");
		return ret;
	}

	ret = isppyrrec_cur_fetch_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec cur_fetch\n");
		return ret;
	}

	ret = isppyrrec_reconstruct_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec reconstruct\n");
		return ret;
	}

	/* TBD dispatch/common/YNR/CNR/uvdelay */

	ret = isppyrrec_store_get(ctx, idx);
	if (ret) {
		pr_err("fail to get isp pyr_rec store\n");
		return ret;
	}

	return ret;
}

static int isppyrrec_cfg_param(void *handle,
		enum isp_rec_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct camera_frame * pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %p\n", handle);
		return -EFAULT;
	}

	rec_ctx = (struct isp_rec_ctx_desc *)handle;
	switch (cmd) {
	case ISP_REC_CFG_BUF:
		pframe = (struct camera_frame *)param;
		ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map isp pyr rec iommu buf.\n");
			ret = -EINVAL;
			goto exit;
		}
		if (rec_ctx->buf_info == NULL) {
			rec_ctx->buf_info = pframe;
			pr_debug("REC buf[0x%p] = 0x%lx\n", pframe, rec_ctx->buf_info->buf.iova[0]);
		}
		break;
	case ISP_REC_CFG_LAYER_NUM:
		rec_ctx->layer_num = *(uint32_t *)param;
		pr_debug("layer num %d\n", rec_ctx->layer_num);
		break;
	case ISP_REC_CFG_WORK_MODE:
		rec_ctx->wmode = *(uint32_t *)param;
		pr_debug("work mode %d\n", rec_ctx->wmode);
		break;
	case ISP_REC_CFG_HW_CTX_IDX:
		rec_ctx->hw_ctx_id = *(uint32_t *)param;
		pr_debug("hw ctx id %d\n", rec_ctx->hw_ctx_id);
		break;
	case ISP_REC_CFG_FMCU_HANDLE:
		rec_ctx->fmcu_handle = param;
		break;
	case ISP_REC_CFG_DEWARPING_EB:
		rec_ctx->dewarp_eb = *(uint32_t *)param;
		pr_debug("dewarp eb %d\n", rec_ctx->dewarp_eb);
		break;
	default:
		pr_err("fail to get known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

exit:
	return ret;
}

static int isppyrrec_pipe_proc(void *handle, void *param)
{
	int ret = 0;
	uint32_t i = 0, j = 0, layer_num = 0;
	uint32_t offset = 0, align = 1, size = 0;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct isp_pyr_rec_in *in_ptr = NULL;
	struct isp_hw_k_blk_func rec_share_func, rec_frame_func, rec_slice_func;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %d\n", handle);
		return -EFAULT;
	}

	in_ptr = (struct isp_pyr_rec_in *)param;
	rec_ctx = (struct isp_rec_ctx_desc *)handle;
	layer_num = rec_ctx->layer_num;

	/* calc multi layer pyramid dec input addr & size */
	pr_debug("isp %d rec layer num %d\n", rec_ctx->ctx_id, layer_num);
	size = in_ptr->src.w * in_ptr->src.h;
	rec_ctx->in_fmt = in_ptr->in_fmt;
	rec_ctx->out_fmt = in_ptr->in_fmt;
	rec_ctx->fetch_addr[0] = in_ptr->in_addr;
	rec_ctx->pyr_layer_size[0].w = isp_rec_layer0_width(in_ptr->src.w, layer_num);
	rec_ctx->pyr_layer_size[0].h = isp_rec_layer0_heigh(in_ptr->src.h, layer_num);
	rec_ctx->pyr_padding_size.w = rec_ctx->pyr_layer_size[0].w - in_ptr->src.w;
	rec_ctx->pyr_padding_size.h = rec_ctx->pyr_layer_size[0].h - in_ptr->src.h;
	for (i = 1; i < layer_num + 1; i++) {
		align = (i > 1) ? (align * 4) : 1;
		offset += (size * 3 / 2) / align;
		rec_ctx->fetch_addr[i].addr_ch0 = in_ptr->in_addr.addr_ch0 + offset;
		rec_ctx->fetch_addr[i].addr_ch1 = rec_ctx->fetch_addr[i].addr_ch0 + size / align;
		rec_ctx->pyr_layer_size[i].w = rec_ctx->pyr_layer_size[i - 1].w /2;
		rec_ctx->pyr_layer_size[i].h = rec_ctx->pyr_layer_size[i - 1].h /2;
	}
	/* layer 0 fetch & store size is real size not include padding size */
	rec_ctx->pyr_layer_size[0].w = in_ptr->src.w;
	rec_ctx->pyr_layer_size[0].h = in_ptr->src.h;

	/* calc multi layer pyramid rec output addr */
	for (i = 0; i < layer_num; i++) {
		/* This is for ensure the last store frame buf is OUT for user
		.* Then the frame before should be temp. Thus, only one tmp
		.* buffer is enough for all the isp pyramid rec process */
		if (i % 2 == 0) {
			rec_ctx->store_addr[i] = in_ptr->out_addr;
		} else {
			rec_ctx->store_addr[i].addr_ch0 = rec_ctx->buf_info->buf.iova[0];
			rec_ctx->store_addr[i].addr_ch1 = rec_ctx->store_addr[i].addr_ch0
				+ rec_ctx->buf_info->width * rec_ctx->buf_info->height;
		}
	}

	rec_share_func.index = ISP_K_BLK_PYR_REC_SHARE;
	rec_ctx->hw->isp_ioctl(rec_ctx->hw, ISP_HW_CFG_K_BLK_FUNC_GET, &rec_share_func);
	rec_frame_func.index = ISP_K_BLK_PYR_REC_FRAME;
	rec_ctx->hw->isp_ioctl(rec_ctx->hw, ISP_HW_CFG_K_BLK_FUNC_GET, &rec_frame_func);
	rec_slice_func.index = ISP_K_BLK_PYR_REC_SLICE;
	rec_ctx->hw->isp_ioctl(rec_ctx->hw, ISP_HW_CFG_K_BLK_FUNC_GET, &rec_slice_func);
	for (i = layer_num; i > 0; i--) {
		ret = isppyrrec_block_cfg_get(rec_ctx, i);
		if (ret) {
			pr_err("fail to get isp pyr_rec block_cfg\n");
			return ret;
		}

		/* rec layer0 need write to ddr when dewarp eb */
		if (i == 1 && (!rec_ctx->dewarp_eb))
			return 0;

		if (i == 2 && rec_share_func.k_blk_func)
			rec_share_func.k_blk_func(rec_ctx);

		if (rec_frame_func.k_blk_func)
			rec_frame_func.k_blk_func(rec_ctx);

		rec_ctx->slice_num = in_ptr->slice_num[i];
		for (j = 0; j < rec_ctx->slice_num; j++) {
			rec_ctx->cur_slice_id = j;
			ret = isppyrrec_slice_cfg_get(rec_ctx, i);
			if (ret) {
				pr_err("fail to get isp pyr_rec slicecfg\n");
				return ret;
			}
			if (rec_slice_func.k_blk_func)
				rec_slice_func.k_blk_func(rec_ctx);
		}
	}

	return ret;
}

void *isp_pyr_rec_ctx_get(uint32_t idx, void *hw)
{
	struct isp_rec_ctx_desc *rec_ctx = NULL;

	rec_ctx = vzalloc(sizeof(struct isp_rec_ctx_desc));
	if (!rec_ctx)
		return NULL;

	rec_ctx->ctx_id = idx;
	rec_ctx->hw = hw;
	rec_ctx->ops.cfg_param = isppyrrec_cfg_param;
	rec_ctx->ops.pipe_proc = isppyrrec_pipe_proc;

	return rec_ctx;
}

void isp_pyr_rec_ctx_put(void *rec_handle)
{
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct camera_buf *buf_info = NULL;

	if (!rec_handle) {
		pr_err("fail to get valid rec handle\n");
		return;
	}

	rec_ctx = (struct isp_rec_ctx_desc *)rec_handle;
	if (rec_ctx->buf_info) {
		buf_info = &rec_ctx->buf_info->buf;
		if (buf_info && buf_info->mapping_state & CAM_BUF_MAPPING_DEV) {
			cam_buf_iommu_unmap(buf_info);
			buf_info = NULL;
		}
	}

	if (rec_ctx)
		vfree(rec_ctx);
	rec_ctx = NULL;
}
