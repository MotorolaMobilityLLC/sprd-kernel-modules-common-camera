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
#include <linux/uaccess.h>
#include <sprd_mm.h>

#include "isp_reg.h"
#include "isp_dec_int.h"
#include "isp_pyr_dec.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "PYR_K_DEC: %d %d %s : "fmt, current->pid, __LINE__, __func__

typedef void(*isp_dec_isr)(void *param);

static const uint32_t isp_dec_irq_process[] = {
	ISP_INT_DEC_FMCU_CONFIG_DONE,
};

static void isppyrdec_fmcu_config_done(void *dec_handle)
{
	struct isp_dec_pipe_dev *dev = NULL;

	dev = (struct isp_dec_pipe_dev *)dec_handle;
	if (dev->irq_proc_func)
		dev->irq_proc_func(dev);

	pr_debug("dec fmcu config done\n");
}

static isp_dec_isr isp_dec_isr_handler[32] = {
	[ISP_INT_DEC_FMCU_CONFIG_DONE] = isppyrdec_fmcu_config_done,
};

static irqreturn_t isppyrdec_isr_root(int irq, void *priv)
{
	uint32_t irq_line = 0, k = 0;
	uint32_t err_mask = 0;
	uint32_t irq_numbers = 0;
	struct isp_dec_pipe_dev *ctx = (struct isp_dec_pipe_dev *)priv;

	if (!ctx) {
		pr_err("fail to get valid dev\n");
		return IRQ_HANDLED;
	}

	if (unlikely(irq != ctx->irq_no)) {
		pr_err("fail to match isppyr dec irq %d %d\n", irq, ctx->irq_no);
		return IRQ_NONE;
	}

	irq_numbers = ARRAY_SIZE(isp_dec_irq_process);
	err_mask = ISP_DEC_INT_LINE_MASK_ERR;
	irq_line = ISP_HREG_RD(ISP_DEC_INT_BASE + ISP_INT_INT0);
	ISP_HREG_WR(ISP_DEC_INT_BASE + ISP_INT_CLR0, irq_line);

	pr_debug("isp pyr dec irq status %d\n", irq_line);
	if (unlikely(err_mask & irq_line)) {
		pr_err("fail to get normal isp dec status 0x%x\n", irq_line);
		/* print isp dec reg info here */
		return IRQ_HANDLED;
	}

	for (k = 0; k < irq_numbers; k++) {
		uint32_t irq_id = isp_dec_irq_process[k];

		if (irq_line & (1 << irq_id)) {
			if (isp_dec_isr_handler[irq_id]) {
				isp_dec_isr_handler[irq_id](ctx);
			}
		}
		irq_line &= ~(1 << irq_id);
		if (!irq_line)
			break;
	}

	return IRQ_HANDLED;
}

int isp_pyr_dec_irq_func(void *handle)
{
	int ret = 0;
	struct isp_dec_pipe_dev *ctx = NULL;

	if (!handle) {
		pr_err("fail to isp_dec handle NULL\n");
		return -EFAULT;
	}

	ctx = (struct isp_dec_pipe_dev *)handle;
	ctx->isr_func = isppyrdec_isr_root;

	return ret;
}

static int isppyrdec_cfg_fetch(struct isp_dec_pipe_dev *ctx)
{
	int ret = 0;
	uint32_t addr = 0, cmd = 0, base = 0, color_format = 0;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_dec_fetch_info *dec_fetch = NULL;
	struct slice_fetch_info *fetch_slc = NULL;

	fmcu = (struct isp_fmcu_ctx_desc *)ctx->fmcu_handle;
	dec_fetch = &ctx->fetch_dec_info;
	fetch_slc = &ctx->slices[ctx->cur_slice_id].slice_fetch;

	switch (dec_fetch->color_format) {
	case ISP_FETCH_YVU420_2FRAME_10:
	case ISP_FETCH_YVU420_2FRAME_MIPI:
		color_format = 1;
		break;
	case ISP_FETCH_YVU420_2FRAME:
		color_format = 3;
		break;
	case ISP_FETCH_YUV420_2FRAME_10:
	case ISP_FETCH_YUV420_2FRAME_MIPI:
		color_format = 0;
		break;
	case ISP_FETCH_YUV420_2FRAME:
		color_format = 2;
		break;
	case ISP_FETCH_FULL_RGB10:
		color_format = 4;
		break;
	default:
		pr_err("fail to get isp fetch format:%d\n", dec_fetch->color_format);
		break;
	}

	base = PYR_DEC_FETCH_BASE;
	addr = ISP_GET_REG(ISP_FETCH_PARAM0) + base;
	cmd =((dec_fetch->chk_sum_clr_en & 0x1) << 11) |
		((dec_fetch->ft1_axi_reorder_en & 0x1) << 9) |
		((dec_fetch->ft0_axi_reorder_en & 0x1) << 8) |
		((color_format & 0x7) << 4) |
		((dec_fetch->substract & 0x1) << 1) |
		((dec_fetch->bypass & 0x1) << 0);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_MEM_SLICE_SIZE) + base;
	cmd = ((fetch_slc->size.h & 0xFFFF) << 16) | (fetch_slc->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_Y_PITCH) + base;
	cmd = dec_fetch->pitch[0];
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_Y_ADDR) + base;
	cmd = fetch_slc->addr.addr_ch0;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_U_PITCH) + base;
	cmd = dec_fetch->pitch[1];
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_U_ADDR) + base;
	cmd = fetch_slc->addr.addr_ch1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_MIPI_PARAM) + base;
	cmd = (fetch_slc->mipi_word_num & 0xFFFF) |
		((fetch_slc->mipi_byte_rel_pos & 0xF) << 16) |
		((fetch_slc->mipi10_en & 0x1) << 20);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_MIPI_PARAM_UV) + base;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_DISPATCH_CH0_SIZE) + PYR_DEC_DISPATCH_BASE;
	cmd = ((fetch_slc->size.h & 0xFFFF) << 16)
		| (fetch_slc->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	return ret;
}

static int isppyrdec_cfg_store(struct isp_dec_pipe_dev *ctx, uint32_t store_block)
{
	int ret = 0;
	uint32_t addr = 0, cmd = 0, base = 0;
	uint32_t color_format = 0, data_10b = 0;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_dec_store_info *store = NULL;
	struct slice_store_info *store_slc = NULL;

	fmcu = (struct isp_fmcu_ctx_desc *)ctx->fmcu_handle;
	switch (store_block) {
	case ISP_PYR_DEC_STORE_DCT:
		base = PYR_DCT_STORE_BASE;
		store = &ctx->store_dct_info;
		store_slc = &ctx->slices[ctx->cur_slice_id].slice_dct_store;
		break;
	case ISP_PYR_DEC_STORE_DEC:
		base = PYR_DEC_STORE_BASE;
		store = &ctx->store_dec_info;
		store_slc = &ctx->slices[ctx->cur_slice_id].slice_dec_store;
		break;
	default:
		pr_err("fail to support rec fetch %d.\n", store_block);
		return -EFAULT;
	}

	switch (store->color_format) {
	case ISP_FETCH_YUV420_2FRAME_MIPI:
		color_format = 0xC;
		data_10b = 1;
		break;
	case ISP_FETCH_YVU420_2FRAME_MIPI:
		color_format = 0xD;
		data_10b = 1;
		break;
	case ISP_FETCH_YUV420_2FRAME_10:
		color_format = 0x4;
		data_10b = 1;
		break;
	case ISP_FETCH_YVU420_2FRAME_10:
		color_format = 0x5;
		data_10b = 1;
		break;
	default:
		data_10b = 0;
		pr_err("fail to support color foramt %d.\n", store->color_format);
	}

	addr = ISP_GET_REG(ISP_STORE_PARAM) + base;
	cmd = ((store->bypass & 0x1) << 0) |
		((store->burst_len & 0x1) << 1) |
		((store->speed2x & 0x1) << 2) |
		((store->mirror_en & 0x1) <<3) |
		((color_format & 0xf) << 4) |
		((store->mipi_en& 0x1) << 7) |
		((store->endian & 0x3) << 8) |
		((store->mono_en & 0x1) << 10) |
		((data_10b & 0x1) << 11) |
		((store->flip_en & 0x1) << 12) |
		((store->last_frm_en & 0x3) << 13);
	FMCU_PUSH(fmcu, addr, cmd);

	if (store->bypass)
		return 0;

	addr = ISP_GET_REG(ISP_STORE_SLICE_SIZE) + base;
	cmd = ((store_slc->size.h & 0xFFFF) << 16) | (store_slc->size.w & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_BORDER) + base;
	cmd =  (store_slc->border.left_border & 0xFFFF) |
		((store_slc->border.right_border & 0xFFFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_BORDER_1) + base;
	cmd = (store_slc->border.up_border & 0xFFFF) |
		((store_slc->border.down_border & 0xFFFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_Y_ADDR) + base;
	cmd = store_slc->addr.addr_ch0;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_Y_PITCH) + base;
	cmd = store->pitch[0];
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_U_ADDR) + base;
	cmd = store_slc->addr.addr_ch1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_U_PITCH) + base;
	cmd = store->pitch[1];
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SHADOW_CLR) + base;
	cmd = 1;
	FMCU_PUSH(fmcu, addr, cmd);

	return ret;
}

static int isppyrdec_cfg_offline(struct isp_dec_pipe_dev *ctx)
{
	int ret = 0;
	uint32_t addr = 0, cmd = 0;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_dec_offline_info *dec_off_info = NULL;
	struct slice_pyr_dec_info *pyr_dec_slc = NULL;
	struct cam_hw_info *hw = NULL;

	fmcu = (struct isp_fmcu_ctx_desc *)ctx->fmcu_handle;
	dec_off_info = &ctx->offline_dec_info;
	pyr_dec_slc = &ctx->slices[ctx->cur_slice_id].slice_pyr_dec;
	hw = ctx->hw;

	addr = ISP_GET_REG(ISP_DEC_OFFLINE_PARAM);
	cmd = ((dec_off_info->fmcu_path_sel & 0x1) << 7) |
		((dec_off_info->fetch_path_sel & 0x1) << 6) |
		((dec_off_info->vector_channel_idx & 0x7) << 3) |
		((dec_off_info->chksum_wrk_mode & 0x1) << 2) |
		((dec_off_info->chksum_clr_mode & 0x1) << 1);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_DEC_OFFLINE_PARAM1);
	cmd = (pyr_dec_slc->hor_padding_en & 0x1) |
		((pyr_dec_slc->hor_padding_num & 0x7FFF) <<1) |
		((pyr_dec_slc->ver_padding_en & 0x1) <<16) |
		((pyr_dec_slc->ver_padding_num & 0x7F) <<17);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_DISPATCH_DLY) + PYR_DEC_DISPATCH_BASE;
	cmd = ((pyr_dec_slc->dispatch_dly_height_num & 0xFFFF) << 16)
		| (pyr_dec_slc->dispatch_dly_width_num & 0xFFFF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_DISPATCH_LINE_DLY1) + PYR_DEC_DISPATCH_BASE;
	cmd = (dec_off_info->dispatch_width_dly_num_flash & 0xFFFF) |
		((dec_off_info->dispatch_done_cfg_mode & 0x1) << 16) |
		((dec_off_info->dispatch_yuv_start_row_num & 0xFF) << 20)|
		((dec_off_info->dispatch_yuv_start_order & 1) << 28)|
		((dec_off_info->dispatch_dbg_mode_ch0 & 1) << 30)|
		((dec_off_info->dispatch_width_flash_mode & 1) << 31);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_DISPATCH_PIPE_BUF_CTRL_CH0) + PYR_DEC_DISPATCH_BASE;
	cmd = (dec_off_info->dispatch_pipe_hblank_num & 0xFF) |
		((dec_off_info->dispatch_pipe_flush_num & 0xFF) << 8)
		| ((dec_off_info->dispatch_pipe_nfull_num & 0x7FF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	/* for normal fetch if use fbd need update fetch start & common sel */
	addr = ISP_GET_REG(ISP_FETCH_START) + PYR_DEC_FETCH_BASE;
	cmd = 1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = PRE0_ALL_DONE;
	FMCU_PUSH(fmcu, addr, cmd);

	hw->isp_ioctl(hw, ISP_HW_CFG_FMCU_CMD_ALIGN, fmcu);

	return ret;
}

int isp_pyr_dec_config(void *handle)
{
	int ret = 0;
	struct isp_dec_pipe_dev *ctx = NULL;

	if (!handle) {
		pr_err("fail to isp_dec handle NULL\n");
		return -EFAULT;
	}

	ctx = (struct isp_dec_pipe_dev *)handle;

	isppyrdec_cfg_fetch(ctx);
	isppyrdec_cfg_store(ctx, ISP_PYR_DEC_STORE_DCT);
	isppyrdec_cfg_store(ctx, ISP_PYR_DEC_STORE_DEC);
	isppyrdec_cfg_offline(ctx);

	return ret;
}
