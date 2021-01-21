
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
#include "isp_hw.h"

#include "isp_reg.h"
#include "cam_types.h"
#include "cam_block.h"
#include "isp_dewarping.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DEWARPING: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int isp_dewarping_dewarp_cache_set(void *handle)
{
	uint32_t val = 0x7;
	uint32_t ret = 0;
	uint32_t idx;
	struct isp_dewarp_cache_info *dewarp_cache = NULL;

	if (!handle) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}
	dewarp_cache = (struct isp_dewarp_cache_info *)handle;
	idx = dewarp_cache->ctx_id;
	pr_debug("enter: fmt:%d, pitch:%d, is_pack:%d\n", dewarp_cache->yuv_format,
			dewarp_cache->frame_pitch, dewarp_cache->dewarp_cache_mipi);

	ISP_REG_MWR(idx, ISP_DEWARPING_CACHE_PARA, BIT_0, dewarp_cache->dewarp_cache_bypass);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_10 |BIT_11 , dewarp_cache->fetch_path_sel << 10);

	val = ((dewarp_cache->dewarp_cache_endian & 0x3) << 1) | ((dewarp_cache->dewarp_cache_prefetch_len & 0x7) << 3) |
		((dewarp_cache->dewarp_cache_mipi & 0x1) << 6) | ((dewarp_cache->yuv_format & 0x1) << 7);
	ISP_REG_MWR(idx, ISP_DEWARPING_CACHE_PARA, 0xFE,val);
	ISP_REG_WR(idx, ISP_DEWARPING_CACHE_FRAME_WIDTH, dewarp_cache->frame_pitch);
	ISP_REG_WR(idx, ISP_DEWARPING_CACHE_FRAME_YADDR, dewarp_cache->addr.addr_ch0);
	ISP_REG_WR(idx, ISP_DEWARPING_CACHE_FRAME_UVADDR, dewarp_cache->addr.addr_ch1);
	return ret;
}

int isp_dewarping_config_param(void *handle)
{
	uint32_t val = 0;
	uint32_t ret = 0;
	uint32_t idx;
	struct isp_dewarping_blk_info *dewarp_ctx;

	if (!handle) {
		pr_err("fail to dewarping_config_reg parm NULL\n");
		return -EFAULT;
	}
	dewarp_ctx = (struct isp_dewarping_blk_info *)handle;
	idx = dewarp_ctx->cxt_id;

	val = ((dewarp_ctx->grid_size & 0x1ff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_GRID_SIZE, val);

	val = ((dewarp_ctx->grid_num_x & 0x3f) << 16) |
		((dewarp_ctx->grid_num_y & 0x3f) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_GRID_NUM, val);

	val = ((dewarp_ctx->pos_x & 0x1fff) << 16) |
		((dewarp_ctx->pos_y & 0x1fff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_POS, val);

	val = ((dewarp_ctx->dst_width& 0x1fff) << 16) |
		((dewarp_ctx->dst_height& 0x1fff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_DST_SIZE, val);

	val = ((dewarp_ctx->src_width & 0x1fff) << 16) |
		((dewarp_ctx->src_height & 0x1fff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_SRC_SIZE, val);

	val = ((dewarp_ctx->dewarping_lbuf_ctrl_nfull_size & 0xf) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_NFULL_SIZE, val);

	val = ((dewarp_ctx->start_mb_x & 0x3ff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_SLICE_START, val);

	val = ((dewarp_ctx->mb_y_num & 0x3ff) << 16) |
		((dewarp_ctx->mb_x_num & 0x3ff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_MB_NUM, val);

	val = ((dewarp_ctx->init_start_row & 0x1fff) << 16) |
		((dewarp_ctx->init_start_col & 0x1fff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_INIT_OFFSET, val);

	val = ((dewarp_ctx->chk_wrk_mode & 0x1) << 1) |
		((dewarp_ctx->chk_clr_mode & 0x1) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_CHK_SUM, val);

	val = ((dewarp_ctx->crop_start_x & 0xffff) << 16) |
		((dewarp_ctx->crop_start_y & 0xffff) << 0);
	ISP_REG_WR(idx, ISP_DEWARPING_CROP_PARA, val);

	/* wait otp and other coeff format */

	/*for (i = 0; i < dewarp_ctx->grid_data_size; i++) {
		val = dewarp_ctx->grid_x_ch0_buf[i];
		ISP_REG_WR(idx, ISP_DEWARPING_GRID_X_CH0 + i * 4, val);
		val = dewarp_ctx->grid_y_ch0_buf[i];
		ISP_REG_WR(idx, ISP_DEWARPING_GRID_Y_CH0 + i * 4, val);
	}*/
	return ret;
}
