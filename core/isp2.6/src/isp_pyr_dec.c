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

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include "isp_pyr_dec.h"
#include "isp_fmcu.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DEC: %d %d %s : "fmt, current->pid, __LINE__, __func__

static uint32_t isppyrdec_cal_pitch(uint32_t w, uint32_t format)
{
	uint32_t pitch = 0;

	switch (format) {
	case ISP_FETCH_YUV420_2FRAME_MIPI:
	case ISP_FETCH_YVU420_2FRAME_MIPI:
		pitch = w * 10 / 8;
		break;
	case ISP_FETCH_YVU420_2FRAME_10:
	case ISP_FETCH_YUV420_2FRAME_10:
		pitch = w * 2;
		break;
	default:
		pitch =w;
		pr_info("fail to support foramt %d w%d\n", format, w);
		break;
	}

	return pitch;
}

static void isppyrdec_src_frame_ret(void *param)
{
	struct camera_frame *frame = NULL;
	struct isp_dec_sw_ctx *pctx = NULL;
	struct isp_offline_param *cur, *prev;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	pctx = (struct isp_dec_sw_ctx *)frame->priv_data;
	if (!pctx) {
		pr_err("fail to get src_frame pctx.\n");
		return;
	}

	cur = (struct isp_offline_param *)(frame->param_data);
	while (cur) {
		prev = (struct isp_offline_param *)cur->prev;
		pr_info("free %p\n", cur);
		kfree(cur);
		cur = prev;
	}
	frame->param_data = NULL;
	if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&frame->buf);
	pctx->cb_func(ISP_CB_RET_SRC_BUF, frame, pctx->cb_priv_data);
}

static int isppyrrec_irq_free(struct isp_dec_pipe_dev *ctx)
{
	struct cam_hw_info *hw = NULL;

	if (!ctx) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	hw = ctx->hw;
	devm_free_irq(&hw->pdev->dev, ctx->irq_no, (void *)ctx);

	return 0;
}

static int isppyrdec_irq_request(struct isp_dec_pipe_dev *ctx)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	char pyrdec_name[32] = { 0 };

	if (!ctx) {
		pr_err("fail to get valid input ptr ctx %p\n");
		return -EFAULT;
	}

	hw = ctx->hw;
	ctx->irq_no = hw->ip_isp->dec_irq_no;
	sprintf(pyrdec_name, "isp_pyr_dec");
	if (!ctx->isr_func) {
		pr_err("fail to get isp dec irq call back func\n");
		return -EFAULT;
	}

	ret = devm_request_irq(&hw->pdev->dev, ctx->irq_no, ctx->isr_func,
			IRQF_SHARED, pyrdec_name, (void *)ctx);
	if (ret) {
		pr_err("fail to install isp pyr dec irq_no %d\n", ctx->irq_no);
		return -EFAULT;
	}

	return ret;
}

static int isppyrdec_offline_thread_stop(void *param)
{
	int cnt = 0;
	int ret = 0;
	struct cam_thread_info *thrd = NULL;
	struct isp_dec_pipe_dev *pctx = NULL;

	thrd = (struct cam_thread_info *)param;
	pctx = (struct isp_dec_pipe_dev *)thrd->ctx_handle;

	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		while (cnt < 2500) {
			cnt++;
			if (atomic_read(&thrd->thread_stop) == 0)
				break;
			udelay(1000);
		}
		thrd->thread_task = NULL;
		pr_info("offline thread stopped. wait %d ms\n", cnt);
	}

	/* wait for last frame done */
	ret = wait_for_completion_interruptible_timeout(
		&pctx->frm_done, ISP_CONTEXT_TIMEOUT);
	if (ret == -ERESTARTSYS)
		pr_err("fail to interrupt, when isp wait\n");
	else if (ret == 0)
		pr_err("fail to wait pyr dec, timeout.\n");
	else
		pr_info("wait time %d\n", ret);
	return 0;
}

static int isppyrdec_offline_thread_loop(void *arg)
{
	struct isp_dec_pipe_dev *pctx = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_thread_info *)arg;
	pctx = (struct isp_dec_pipe_dev *)thrd->ctx_handle;

	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(&thrd->thread_stop, 1, 0) == 1) {
				pr_info("isp pyr rec thread stop.\n");
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(pctx)) {
				pr_err("fail to start isp pyr dec proc. exit thread\n");
				//pctx->isp_cb_func(ISP_CB_DEV_ERR, dev, pctx->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}

static int isppyrdec_fetch_get(struct isp_dec_pipe_dev *dev, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t ch_offset[3] = { 0 };
	uint32_t mipi_word_num_start[16] = {0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	struct slice_fetch_info *slc_fetch = NULL;
	struct isp_dec_fetch_info *fetch = NULL;
	struct isp_dec_slice_desc *cur_slc = NULL;

	if (!dev) {
		pr_err("fail to get valid input dev\n");
		return -EFAULT;
	}

	fetch = &dev->fetch_dec_info;
	fetch->bypass = 0;
	fetch->color_format = dev->in_fmt;
	fetch->addr[0] = dev->fetch_addr[idx].addr_ch0;
	fetch->addr[1] = dev->fetch_addr[idx].addr_ch1;
	fetch->width = dev->dec_layer_size[idx].w;
	fetch->height = dev->dec_layer_size[idx].h;
	fetch->pitch[0] = isppyrdec_cal_pitch(fetch->width, fetch->color_format);
	fetch->pitch[1] = fetch->pitch[0];
	fetch->chk_sum_clr_en = 0;
	fetch->ft0_axi_reorder_en = 0;
	fetch->ft1_axi_reorder_en = 0;
	fetch->substract = 0;
	fetch->ft0_max_len_sel = 1;
	fetch->ft1_max_len_sel = 1;
	/* this default val come from spec need to confirm */
	fetch->ft0_retain_num = 0x20;
	fetch->ft1_retain_num = 0x20;

	cur_slc = &dev->slices[0];
	for (i = 0; i < dev->slice_num; i++, cur_slc++) {
		slc_fetch = &cur_slc->slice_fetch;
		start_col = cur_slc->slice_fetch_pos.start_col;
		start_row = cur_slc->slice_fetch_pos.start_row;
		end_col = cur_slc->slice_fetch_pos.end_col;
		end_row = cur_slc->slice_fetch_pos.end_row;

		switch (fetch->color_format) {
		case ISP_FETCH_YUV420_2FRAME:
		case ISP_FETCH_YVU420_2FRAME:
			ch_offset[0] = start_row * fetch->pitch[0] + start_col;
			ch_offset[1] = (start_row >> 1) * fetch->pitch[1] + start_col;
			break;
		case ISP_FETCH_YUV420_2FRAME_MIPI:
		case ISP_FETCH_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row * fetch->pitch[0]
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
				fetch->pitch[0], ch_offset[0],
				slc_fetch->mipi_byte_rel_pos, slc_fetch->mipi_word_num);
			break;
		default:
			ch_offset[0] = start_row * fetch->pitch[0] + start_col * 2;
			break;
		}

		slc_fetch->addr.addr_ch0 = fetch->addr[0] + ch_offset[0];
		slc_fetch->addr.addr_ch1 = fetch->addr[1] + ch_offset[1];
		slc_fetch->size.h = end_row - start_row + 1;
		slc_fetch->size.w = end_col - start_col + 1;

		pr_debug("slice fetch size %d, %d\n", slc_fetch->size.w, slc_fetch->size.h);
	}

	return ret;
}

static int isppyrdec_store_dct_get(struct isp_dec_pipe_dev *dev, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t overlap_left = 0, overlap_up = 0;
	uint32_t overlap_right = 0, overlap_down = 0;
	uint32_t start_row_out = 0, start_col_out = 0;
	uint32_t ch_offset[3] = { 0 };
	struct isp_dec_slice_desc *cur_slc = NULL;
	struct slice_store_info *slc_dct_store = NULL;
	struct isp_dec_store_info *store_dct = NULL;

	if (!dev) {
		pr_err("fail to get valid input dev\n", dev);
		return -EFAULT;
	}

	store_dct = &dev->store_dct_info;
	if (idx != 0) {
		/* store dct only need output for layer0 */
		store_dct->bypass = 1;
		return 0;
	}
	store_dct->bypass = 0;
	store_dct->color_format = dev->out_fmt;
	store_dct->addr[0] = dev->store_addr[idx].addr_ch0;
	store_dct->addr[1] = dev->store_addr[idx].addr_ch1;
	store_dct->width = dev->dec_layer_size[idx].w;
	store_dct->height = dev->dec_layer_size[idx].h;
	store_dct->pitch[0] = isppyrdec_cal_pitch(store_dct->width, store_dct->color_format);
	store_dct->pitch[1] = store_dct->pitch[0];
	store_dct->data_10b = 0;
	store_dct->mipi_en = 0;
	store_dct->flip_en = 0;
	store_dct->last_frm_en = 1;
	store_dct->mono_en = 0;
	store_dct->mirror_en = 0;
	store_dct->burst_len = 1;
	store_dct->speed2x = 1;
	store_dct->shadow_clr_sel = 1;
	store_dct->shadow_clr = 1;
	store_dct->rd_ctrl = 0;

	cur_slc = &dev->slices[0];
	for (i = 0; i < dev->slice_num; i++, cur_slc++) {
		slc_dct_store = &cur_slc->slice_dct_store;
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

		switch (store_dct->color_format) {
		case ISP_STORE_YUV420_2FRAME_MIPI:
		case ISP_STORE_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row_out * store_dct->pitch[0] + start_col_out * 5 /4;
			ch_offset[1] = ((start_row_out * store_dct->pitch[1]) >> 1) + start_col_out * 5 /4;
			break;
		case ISP_STORE_YUV420_2FRAME_10:
		case ISP_STORE_YVU420_2FRAME_10:
			ch_offset[0] = start_row_out * store_dct->pitch[0] + (start_col_out << 1);
			ch_offset[1] = ((start_row_out * store_dct->pitch[1]) >> 1) + (start_col_out << 1);
			break;
		default:
			ch_offset[0] = start_row_out * store_dct->pitch[0] + start_col_out;
			ch_offset[1] = ((start_row_out * store_dct->pitch[1]) >> 1) + start_col_out;
			break;
		}

		slc_dct_store->addr.addr_ch0 = store_dct->addr[0] + ch_offset[0];
		slc_dct_store->addr.addr_ch1 = store_dct->addr[1] + ch_offset[1];
		slc_dct_store->size.h = end_row - start_row + 1 - overlap_up - overlap_down;
		slc_dct_store->size.w = end_col - start_col + 1 - overlap_left - overlap_right;
		slc_dct_store->border.up_border = overlap_up;
		slc_dct_store->border.down_border = overlap_down;
		slc_dct_store->border.left_border = overlap_left;
		slc_dct_store->border.right_border = overlap_right;

		pr_debug("slice fetch size %d, %d\n", slc_dct_store->size.w, slc_dct_store->size.h);
	}

	return ret;
}

static int isppyrdec_store_dec_get(struct isp_dec_pipe_dev *dev, uint32_t idx)
{
	int ret = 0, i = 0;
	uint32_t start_col = 0, start_row = 0;
	uint32_t end_col = 0, end_row = 0;
	uint32_t overlap_left = 0, overlap_up = 0;
	uint32_t overlap_right = 0, overlap_down = 0;
	uint32_t start_row_out = 0, start_col_out = 0;
	uint32_t ch_offset[3] = { 0 };
	struct isp_dec_slice_desc *cur_slc = NULL;
	struct slice_store_info *slc_dec_store = NULL;
	struct isp_dec_store_info *store_dec = NULL;

	if (!dev) {
		pr_err("fail to get valid input dev\n", dev);
		return -EFAULT;
	}

	store_dec = &dev->store_dec_info;
	idx = idx +1;
	store_dec->bypass = 0;
	store_dec->color_format = dev->out_fmt;
	store_dec->addr[0] = dev->fetch_addr[idx].addr_ch0;
	store_dec->addr[1] = dev->fetch_addr[idx].addr_ch1;
	store_dec->width = dev->dec_layer_size[idx].w / 2;
	store_dec->height = dev->dec_layer_size[idx].h / 2;
	store_dec->pitch[0] = isppyrdec_cal_pitch(store_dec->width, store_dec->color_format);
	store_dec->pitch[1] = store_dec->pitch[0];
	store_dec->data_10b = 0;
	store_dec->mipi_en = 0;
	store_dec->flip_en = 0;
	store_dec->last_frm_en = 1;
	store_dec->mono_en = 0;
	store_dec->mirror_en = 0;
	store_dec->burst_len = 1;
	store_dec->speed2x = 1;
	store_dec->shadow_clr_sel = 1;
	store_dec->shadow_clr = 1;
	store_dec->rd_ctrl = 0;

	cur_slc = &dev->slices[0];
	for (i = 0; i < dev->slice_num; i++, cur_slc++) {
		slc_dec_store = &cur_slc->slice_dct_store;
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

		switch (store_dec->color_format) {
		case ISP_STORE_YUV420_2FRAME_MIPI:
		case ISP_STORE_YVU420_2FRAME_MIPI:
			ch_offset[0] = start_row_out * store_dec->pitch[0] + start_col_out * 5 /4;
			ch_offset[1] = ((start_row_out * store_dec->pitch[1]) >> 1) + start_col_out * 5 /4;
			break;
		case ISP_STORE_YUV420_2FRAME_10:
		case ISP_STORE_YVU420_2FRAME_10:
			ch_offset[0] = start_row_out * store_dec->pitch[0] + (start_col_out << 1);
			ch_offset[1] = ((start_row_out * store_dec->pitch[1]) >> 1) + (start_col_out << 1);
			break;
		default:
			ch_offset[0] = start_row_out * store_dec->pitch[0] + start_col_out;
			ch_offset[1] = ((start_row_out * store_dec->pitch[1]) >> 1) + start_col_out;
			break;
		}

		slc_dec_store->addr.addr_ch0 = store_dec->addr[0] + ch_offset[0];
		slc_dec_store->addr.addr_ch1 = store_dec->addr[1] + ch_offset[1];
		slc_dec_store->size.h = end_row - start_row + 1 - overlap_up - overlap_down;
		slc_dec_store->size.w = end_col - start_col + 1 - overlap_left - overlap_right;
		slc_dec_store->border.up_border = overlap_up;
		slc_dec_store->border.down_border = overlap_down;
		slc_dec_store->border.left_border = overlap_left;
		slc_dec_store->border.right_border = overlap_right;

		pr_debug("slice fetch size %d, %d\n", slc_dec_store->size.w, slc_dec_store->size.h);
	}

	return ret;
}

static int isppyrdec_offline_get(struct isp_dec_pipe_dev *dev, uint32_t idx)
{
	int ret = 0;
	struct isp_dec_offline_info *dec_offline = NULL;

	if (!dev) {
		pr_err("fail to get valid input dev\n", dev);
		return -EFAULT;
	}

	dec_offline = &dev->offline_dec_info;
	dec_offline->bypass = 0;
	dec_offline->hor_padding_en = 0;
	dec_offline->hor_padding_num = 0;
	dec_offline->ver_padding_en = 0;
	dec_offline->ver_padding_num = 0;

	if(idx == 0) {
		dec_offline->hor_padding_num = dev->dec_padding_size.w;
		dec_offline->ver_padding_num = dev->dec_padding_size.h;
		if (dec_offline->hor_padding_num)
			dec_offline->hor_padding_en = 1;
		if (dec_offline->ver_padding_num)
			dec_offline->ver_padding_en = 1;
	}

	return ret;
}

static int isppyrdec_param_cfg(struct isp_dec_pipe_dev *dev, uint32_t index)
{
	int ret = 0;

	if (!dev ) {
		pr_err("fail to get valid input handle\n");
		return -EFAULT;
	}

	ret = isppyrdec_fetch_get(dev, index);
	if (ret) {
		pr_err("fail to get isp pyr_dec fetch\n");
		return ret;
	}

	ret = isppyrdec_store_dct_get(dev, index);
	if (ret) {
		pr_err("fail to get isp pyr_dec store dct\n");
		return ret;
	}

	ret = isppyrdec_store_dec_get(dev, index);
	if (ret) {
		pr_err("fail to get isp pyr_dec store dec\n");
		return ret;
	}

	ret = isppyrdec_offline_get(dev, index);
	if (ret) {
		pr_err("fail to get isp pyr_dec offline\n");
		return ret;
	}

	return ret;
}

static int isppyrdec_offline_frame_start(void *handle)
{
	int ret = 0;
	uint32_t ctx_id = 0, loop = 0;
	uint32_t layer_num = 0, i = 0, j = 0, pitch = 0;
	uint32_t offset = 0, align = 1, size = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct isp_dec_sw_ctx *pctx = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_hw_k_blk_func dec_cfg_func;

	if (!handle) {
		pr_err("fail to get valid input handle\n");
		return -EFAULT;
	}

	dec_dev = (struct isp_dec_pipe_dev *)handle;
	fmcu = (struct isp_fmcu_ctx_desc *)dec_dev->fmcu_handle;
	pframe = cam_queue_dequeue(&dec_dev->in_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_err("fail to get input frame %p\n", pframe);
		goto exit;
	}
	ctx_id = pframe->dec_ctx_id;
	pctx = &dec_dev->sw_ctx[ctx_id];
	layer_num = dec_dev->layer_num;

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n", ctx_id);
		ret = -EINVAL;
		goto map_err;
	}

	loop = 0;
	do {
		ret = cam_queue_enqueue(&dec_dev->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
		goto map_err;
	}

	pitch = isppyrdec_cal_pitch(dec_dev->src.w, dec_dev->in_fmt);
	size = pitch * dec_dev->src.h;
	dec_dev->fetch_addr[0].addr_ch0 = pframe->buf.iova[0];
	dec_dev->fetch_addr[0].addr_ch1 = pframe->buf.iova[0] + size;
	dec_dev->store_addr[0].addr_ch0 = pctx->buf_out->buf.iova[0];
	dec_dev->store_addr[0].addr_ch1 = dec_dev->store_addr[i].addr_ch0 + size;
	dec_dev->dec_layer_size[0].w = isp_rec_layer0_width(dec_dev->src.w, layer_num);
	dec_dev->dec_layer_size[0].h = isp_rec_layer0_heigh(dec_dev->src.h, layer_num);
	dec_dev->dec_padding_size.w = dec_dev->dec_layer_size[0].w - dec_dev->src.w;
	dec_dev->dec_padding_size.h = dec_dev->dec_layer_size[0].h - dec_dev->src.h;
	for (i = 1; i < layer_num + 1; i++) {
		align = align * 2;
		offset += (size * 3 / 2);
		pitch = pitch / 2;
		dec_dev->dec_layer_size[i].w = dec_dev->dec_layer_size[i - 1].w /2;
		dec_dev->dec_layer_size[i].h = dec_dev->dec_layer_size[i - 1].h /2;
		size = pitch * dec_dev->dec_layer_size[i].h;
		dec_dev->store_addr[i].addr_ch0 = pctx->buf_out->buf.iova[0] + offset;
		dec_dev->store_addr[i].addr_ch1 = dec_dev->store_addr[i].addr_ch0 + size;
		if (i < layer_num) {
			dec_dev->fetch_addr[i].addr_ch0 = dec_dev->store_addr[i - 1].addr_ch0;
			dec_dev->fetch_addr[i].addr_ch1 = dec_dev->store_addr[i - 1].addr_ch1;
		}
	}
	/* layer 0 fetch & store size is real size not include padding size */
	dec_dev->dec_layer_size[0].w = dec_dev->src.w;
	dec_dev->dec_layer_size[0].h = dec_dev->src.h;

	dec_cfg_func.index = ISP_K_BLK_PYR_DEC_CFG;
	dec_dev->hw->isp_ioctl(dec_dev->hw, ISP_HW_CFG_K_BLK_FUNC_GET, &dec_cfg_func);
	for (i = 0; i < layer_num; i++) {
		isppyrdec_param_cfg(dec_dev, i);
		/* slice num need update each layer may from input */
		for (j = 0; j < dec_dev->slice_num; j++) {
			dec_dev->cur_slice_id = j;
			if (dec_cfg_func.k_blk_func)
				dec_cfg_func.k_blk_func(dec_dev);
		}
	}

	ret = wait_for_completion_interruptible_timeout(&dec_dev->frm_done,
			ISP_CONTEXT_TIMEOUT);
	if (ret == -ERESTARTSYS) {
		pr_err("fail to interrupt, when isp dec wait\n");
		ret = -EFAULT;
		goto map_err;
	} else if (ret == 0) {
		pr_err("fail to wait isp dec context %d, timeout.\n", ctx_id);
		ret = -EFAULT;
		goto map_err;
	}

	dec_dev->cur_ctx_id = ctx_id;
	/* start pyr dec fmcu */
	fmcu->ops->hw_start(fmcu);

map_err:
	if (pframe)
		isppyrdec_src_frame_ret(pframe);
exit:
	return ret;
}

static int isppyrdec_offline_thread_create(void *param)
{
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct cam_thread_info *thrd = NULL;
	char thread_name[32] = { 0 };

	dec_dev = (struct isp_dec_pipe_dev *)param;
	thrd = &dec_dev->thread;
	thrd->ctx_handle = dec_dev;

	if (thrd->thread_task) {
		pr_info("isp pyr dec offline thread created is exist.\n");
		return 0;
	}

	thrd->proc_func = isppyrdec_offline_frame_start;
	sprintf(thread_name, "isp_pyr_dec_offline");
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);
	thrd->thread_task = kthread_run(isppyrdec_offline_thread_loop, thrd, "%s", thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start isp pyr dec thread %ld\n", PTR_ERR(thrd->thread_task));
		return -EFAULT;
	}

	return 0;
}

static int isppyrdec_cfg_param(void *handle, int ctx_id,
		enum isp_dec_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct camera_frame * pframe = NULL;
	struct isp_dec_sw_ctx *pctx = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %p\n", handle);
		return -EFAULT;
	}

	dec_dev = (struct isp_dec_pipe_dev *)handle;
	pctx = &dec_dev->sw_ctx[ctx_id];
	switch (cmd) {
	case ISP_DEC_CFG_OUT_BUF:
		pframe = (struct camera_frame *)param;
		ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map buf to ISP iommu. cxt %d\n", ctx_id);
			ret = -EINVAL;
			goto exit;
		}
		pctx->buf_out = pframe;
		pr_debug("DEC buf[0x%p] = 0x%lx\n", pframe, pctx->buf_out->buf.iova[0]);
		break;
	case ISP_DEC_CFG_PROC_SIZE:
		dec_dev->src = *(struct img_size *)param;
		pr_debug("DEC size w%d h%d\n", dec_dev->src.w, dec_dev->src.h);
		break;
	case ISP_DEC_CFG_IN_FORMAT:
		dec_dev->in_fmt = *(uint32_t *)param;
		dec_dev->out_fmt = dec_dev->in_fmt;
		pr_debug("DEC proc format %d\n", dec_dev->in_fmt);
		break;
	default:
		pr_err("fail to get known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

exit:
	return ret;
}

static int isppyrdec_proc_frame(void *handle, void *param)
{
	int ret = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct camera_frame *pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr, dec_handle %p, param %p\n", handle, param);
		return -EFAULT;
	}

	dec_dev = (struct isp_dec_pipe_dev *)handle;
	pframe = (struct camera_frame *)param;

	ret = cam_queue_enqueue(&dec_dev->in_queue, &pframe->list);
	if (ret == 0)
		complete(&dec_dev->thread.thread_com);

	return ret;
}

static int isppyrdec_callback_set(void *handle, int ctx_id, isp_dev_callback cb,
		void *priv_data)
{
	int ret = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct isp_dec_sw_ctx *pctx = NULL;

	if (!handle || !cb || !priv_data) {
		pr_err("fail to get valid input ptr, dec_handle %p, callback %p, priv_data %p\n",
			handle, cb, priv_data);
		return -EFAULT;
	}

	dec_dev = (struct isp_dec_pipe_dev *)handle;
	pctx = &dec_dev->sw_ctx[ctx_id];
	if (pctx->cb_func == NULL) {
		pctx->cb_func = cb;
		pctx->cb_priv_data = priv_data;
		pr_info("ctx %d cb %p, %p\n", ctx_id, cb, priv_data);
	}

	return ret;
}

static int isppyrdec_irq_proc(void *handle)
{
	int ret = 0;
	uint32_t cur_ctx_id = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct isp_dec_sw_ctx *pctx = NULL;
	struct camera_frame *pframe = NULL;

	if (!handle) {
		pr_err("fail to get invalid ptr\n");
		return -EFAULT;
	}

	dec_dev = (struct isp_dec_pipe_dev *)handle;
	cur_ctx_id = dec_dev->cur_ctx_id;
	pctx = &dec_dev->sw_ctx[cur_ctx_id];

	complete(&dec_dev->frm_done);

	pframe = cam_queue_dequeue(&dec_dev->proc_queue, struct camera_frame, list);
	if (pframe) {
		/* return buffer to cam channel shared buffer queue. */
		cam_buf_iommu_unmap(&pframe->buf);
		pctx->cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);
	} else {
		pr_err("fail to get src frame sw_idx=%d  proc_queue.cnt:%d\n",
			cur_ctx_id, dec_dev->proc_queue.cnt);
	}

	pframe = pctx->buf_out;
	if (pframe) {
		/* return buffer to cam core for start isp pyrrec proc */
		cam_buf_iommu_unmap(&pframe->buf);
		pctx->cb_func(ISP_CB_RET_PYR_DEC_BUF, pframe, pctx->cb_priv_data);
	} else {
		pr_err("fail to get src frame sw_idx=%d \n", cur_ctx_id);
	}

	return ret;
}

void *isp_pyr_dec_dev_get(void *isp_handle, void *hw)
{
	int ret = 0;
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_hw_k_blk_func irq_func;

	dec_dev = vzalloc(sizeof(struct isp_dec_pipe_dev));
	if (!dec_dev)
		return NULL;

	dec_dev->isp_handle = isp_handle;
	dec_dev->hw = hw;
	dec_dev->layer_num = ISP_PYR_DEC_LAYER_NUM;
	init_completion(&dec_dev->frm_done);
	complete(&dec_dev->frm_done);
	fmcu = isp_fmcu_dec_ctx_get(hw);
	if (fmcu && fmcu->ops) {
		fmcu->hw = hw;
		ret = fmcu->ops->ctx_init(fmcu);
		if (ret) {
			pr_err("fail to init fmcu ctx\n");
			isp_fmcu_ctx_desc_put(fmcu);
		} else {
			dec_dev->fmcu_handle = fmcu;
		}
	} else {
		pr_info("no more fmcu or ops\n");
	}
	/* get isp dec irq call back function */
	irq_func.index = ISP_K_BLK_PYR_DEC_IRQ_FUNC;
	dec_dev->hw->isp_ioctl(hw, ISP_HW_CFG_K_BLK_FUNC_GET, &irq_func);
	if (irq_func.k_blk_func)
		irq_func.k_blk_func(dec_dev);
	dec_dev->irq_proc_func = isppyrdec_irq_proc;
	dec_dev->ops.cfg_param = isppyrdec_cfg_param;
	dec_dev->ops.proc_frame = isppyrdec_proc_frame;
	dec_dev->ops.set_callback = isppyrdec_callback_set;

	cam_queue_init(&dec_dev->in_queue, ISP_PYRDEC_BUF_Q_LEN, isppyrdec_src_frame_ret);
	cam_queue_init(&dec_dev->proc_queue, ISP_PYRDEC_BUF_Q_LEN, isppyrdec_src_frame_ret);

	ret = isppyrdec_irq_request(dec_dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to request irq for isp pyr dec\n");
		goto irq_err;
	}
	ret = isppyrdec_offline_thread_create(dec_dev);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread for isp pyr dec\n");
		goto thrd_err;
	}

	/* irq enable */

	return dec_dev;

thrd_err:
	isppyrrec_irq_free(dec_dev);
irq_err:
	if (dec_dev) {
		vfree(dec_dev);
		dec_dev = NULL;
	}

	return dec_dev;
}

void isp_pyr_dec_dev_put(void *dec_handle)
{
	struct isp_dec_pipe_dev *dec_dev = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;

	if (!dec_handle) {
		pr_err("fail to get valid rec handle\n");
		return;
	}

	dec_dev = (struct isp_dec_pipe_dev *)dec_handle;
	/* irq disable */
	isppyrdec_offline_thread_stop(&dec_dev->thread);
	isppyrrec_irq_free(dec_dev);
	cam_queue_clear(&dec_dev->in_queue, struct camera_frame, list);
	cam_queue_clear(&dec_dev->proc_queue, struct camera_frame, list);
	fmcu = (struct isp_fmcu_ctx_desc *)dec_dev->fmcu_handle;
	if (fmcu) {
		fmcu->ops->ctx_deinit(fmcu);
		isp_fmcu_ctx_desc_put(fmcu);
	}
	dec_dev->fmcu_handle = NULL;
	if (dec_dev)
		vfree(dec_dev);
	dec_dev = NULL;
}
