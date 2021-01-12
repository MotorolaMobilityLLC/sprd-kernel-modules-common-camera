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
#include "cam_types.h"
#include "cam_block.h"
#include "isp_core.h"
#include "isp_dewarping.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DEWARPING: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int calc_grid_num(int img_size, int grid_size)
{
    return (img_size + grid_size - 1) / grid_size + 3;
}

static int ispdewarping_dewarp_cache_get(struct isp_dewarp_ctx_desc *ctx)
{
	int ret = 0;
	struct isp_dewarp_cache_info *dewarp_fetch = NULL;
	struct img_size *src = NULL;

	if (!ctx) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dewarp_fetch = &(ctx->dewarp_cache_info);
	dewarp_fetch->dewarp_cache_bypass = 0;
	dewarp_fetch->yuv_format = ctx->in_fmt;
	dewarp_fetch->fetch_path_sel = ISP_FETCH_PATH_DEWARP;
	dewarp_fetch->dewarp_cache_mipi = 0;
	dewarp_fetch->dewarp_cache_endian = 0;
	dewarp_fetch->dewarp_cache_prefetch_len = 3;

	dewarp_fetch->addr.addr_ch0 = ctx->fetch_addr.addr_ch0;
	dewarp_fetch->addr.addr_ch1 = ctx->fetch_addr.addr_ch1;
	src = &(ctx->src_size);
	if (0 == dewarp_fetch->dewarp_cache_mipi) {
		dewarp_fetch->frame_pitch = ((src->w * 2 + 15) / 16) * 16;
		dewarp_fetch->addr.addr_ch1 =
			dewarp_fetch->addr.addr_ch0 + dewarp_fetch->frame_pitch * src->h;
	} else if (1 == dewarp_fetch->dewarp_cache_mipi) {
		dewarp_fetch->frame_pitch = (src->w * 10 + 127) / 128 * 128 / 8;
		dewarp_fetch->addr.addr_ch1 =
			dewarp_fetch->addr.addr_ch0 + dewarp_fetch->frame_pitch * src->h;
	} else {
		pr_err("fail to get support dewarp_cache_mipi 0x%x\n", dewarp_fetch->dewarp_cache_mipi);
	}
	return ret;
}

static int ispdewarping_dewarp_config_get(struct isp_dewarp_ctx_desc *ctx)
{
	int32_t rtn = 0;
	struct isp_dewarping_blk_info *dewarping_info = NULL;
	uint32_t grid_size = 0;
	uint32_t pad_width = 0;
	uint32_t pad_height = 0;
	uint32_t grid_x = 0;
	uint32_t grid_y = 0;
	uint32_t grid_data_size = 0;

	if (ctx == NULL){
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dewarping_info = &(ctx->dewarp_blk_info);
	dewarping_info->src_width = ctx->src_size.w;
	dewarping_info->src_height = ctx->src_size.h;
	dewarping_info->dst_width = ctx->dst_size.w;
	dewarping_info->dst_height = ctx->dst_size.h;
	grid_size =  ctx->grid_size;
	dewarping_info->grid_size = grid_size;
	dewarping_info->pos_x = (dewarping_info->src_width - dewarping_info->dst_width) >> 1;
	dewarping_info->pos_y = (dewarping_info->src_height - dewarping_info->dst_height) >> 1;
	pad_width  = (dewarping_info->dst_width + DEWARPING_DST_MBLK_SIZE - 1) /
		DEWARPING_DST_MBLK_SIZE * DEWARPING_DST_MBLK_SIZE;
	pad_height = (dewarping_info->dst_height + DEWARPING_DST_MBLK_SIZE - 1) /
		DEWARPING_DST_MBLK_SIZE * DEWARPING_DST_MBLK_SIZE;
	grid_x = calc_grid_num(pad_width, grid_size);
	grid_y = calc_grid_num(pad_height, grid_size);
	grid_data_size = grid_x * grid_y;
	dewarping_info->grid_num_x= grid_x;
	dewarping_info->grid_num_y= grid_y;
	dewarping_info->grid_data_size = grid_data_size;

	dewarping_info->start_mb_x = 0;
	dewarping_info->mb_x_num = ((dewarping_info->src_width + 15) / 16 * 16) / 8;
	dewarping_info->mb_y_num = ((dewarping_info->src_height + 7) / 8 * 8) / 8;
	dewarping_info->dewarping_lbuf_ctrl_nfull_size = 0x4;
	dewarping_info->chk_clr_mode = 0;
	dewarping_info->chk_wrk_mode = 0;
	dewarping_info->init_start_col = 0;
	dewarping_info->init_start_row = 0;
	dewarping_info->crop_start_x = 0;
	dewarping_info->crop_start_y = 0;
	/* to be wait algo */
	return rtn;
}

static int ispdewarping_cfg_param(void *handle,
		enum isp_dewarp_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_dewarp_ctx_desc *dewarping_ctx = NULL;
	struct img_size *size = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dewarping_ctx = (struct isp_dewarp_ctx_desc *)handle;
	switch (cmd) {
	case ISP_DEWARPING_CFG_SIZE:
		size = (struct img_size *)param;
		break;
	case ISP_DEWARPING_CFG_COEFF:
		break;
	case ISP_DEWARPING_CFG_MODE:
		break;
	default:
		pr_err("fail to get known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}
	return ret;
}

static int ispdewarping_proc(void *dewarp_handle, void *param, enum isp_dewarp_mode mode)
{
	int32_t rtn = 0;
	struct isp_dewarp_ctx_desc *dewarping_desc = NULL;
	struct isp_dev_dewarping_info *dewarp_in = NULL;

	if (param == NULL || dewarp_handle == NULL){
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dewarping_desc = (struct isp_dewarp_ctx_desc *)dewarp_handle;
	dewarp_in = (struct isp_dev_dewarping_info *)param;
	dewarping_desc->dst_size.w = dewarp_in->dewarping_dst_image_width;
	dewarping_desc->dst_size.h = dewarp_in->dewarping_dst_image_height;
	dewarping_desc->grid_size = dewarp_in->isp_dewarping_grid_size;
	dewarping_desc->grid_coef_handle =  dewarp_in->dewarping_grid_xy;

	rtn = ispdewarping_dewarp_config_get(dewarping_desc);
	if (rtn != 0){
		pr_err("fail to gen dewarp\n");
		return -EFAULT;
	}
	rtn = ispdewarping_dewarp_cache_get(dewarping_desc);
	if (rtn != 0){
		pr_err("fail to get dewarp cache\n");
		return -EFAULT;
	}
	return rtn;
}

void *isp_dewarping_ctx_get(uint32_t idx)
{
	struct isp_dewarp_ctx_desc *dewarping_desc = NULL;

	dewarping_desc = vzalloc(sizeof(struct isp_dewarp_ctx_desc));
	if (!dewarping_desc)
		return NULL;

	dewarping_desc->idx = idx;
	dewarping_desc->ops.cfg_param = ispdewarping_cfg_param;
	dewarping_desc->ops.pipe_proc = ispdewarping_proc;

	return dewarping_desc;
}

void isp_dewarping_ctx_put(void *dewarp_handle)
{
	struct isp_dewarp_ctx_desc *dewarping_ctx = NULL;

	if (!dewarp_handle) {
		pr_err("fail to get valid dewarp handle\n");
		return;
	}

	dewarping_ctx = (struct isp_dewarp_ctx_desc *)dewarp_handle;

	if (dewarping_ctx)
		vfree(dewarping_ctx);
	dewarping_ctx = NULL;
}
