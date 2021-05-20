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

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "POSTCNR: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_post_cnr_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	uint32_t i = 0, val = 0, layer_num = 0;
	struct isp_dev_post_cnr_h_info *post_cnr_h_info = NULL;
	struct isp_post_cnr_h *param_post_cnr_h = NULL;

	if (!param || !isp_k_param) {
		pr_err("fail to get input ptr\n");
		return -1;
	}

	post_cnr_h_info = &isp_k_param->post_cnr_h_info;
	param_post_cnr_h = &post_cnr_h_info->param_post_cnr_h;

	ret = copy_from_user((void *)post_cnr_h_info,
			param->property_param,
			sizeof(struct isp_dev_post_cnr_h_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}

	layer_num = 0;
	ISP_REG_MWR(idx, ISP_YUV_CNR_CONTRL0, 0xE, layer_num << 1);

	if (post_cnr_h_info->bypass) {
		pr_debug("idx %d, post_cnr_bypass!\n", idx);
		return 0;
	}

	val = ((post_cnr_h_info->baseRadius & 0xFFFF) << 16) |
		((param_post_cnr_h->minRatio & 0x3FF) << 2) |
		((param_post_cnr_h->denoise_radial_en & 0x1) << 1) |
		(param_post_cnr_h->lowpass_filter_en & 0x1);
	ISP_REG_WR(idx, ISP_YUV_CNR_CFG0, val);

	val = ((param_post_cnr_h->imgCenterY & 0xFFFF) << 16) |
		(param_post_cnr_h->imgCenterX & 0xFFFF);
	ISP_REG_WR(idx, ISP_YUV_CNR_CFG1, val);

	val = ((param_post_cnr_h->filter_size & 0x3) << 28) |
		((param_post_cnr_h->slope & 0xFFF) << 16) |
		((param_post_cnr_h->luma_th[1] & 0xFF) << 8) |
		(param_post_cnr_h->luma_th[0] & 0xFF);
	ISP_REG_WR(idx, ISP_YUV_CNR_CFG2, val);

	for (i = 0; i < 18; i++) {
		val = ((param_post_cnr_h->weight_y[0][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_y[0][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_y[0][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_y[0][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_Y_L0_WHT0 + 4 * i, val);

		val = ((param_post_cnr_h->weight_y[1][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_y[1][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_y[1][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_y[1][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_Y_L1_WHT0 + 4*i, val);

		val = ((param_post_cnr_h->weight_y[2][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_y[2][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_y[2][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_y[2][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_Y_L2_WHT0 + 4 * i, val);

		val = ((param_post_cnr_h->weight_uv[0][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_uv[0][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_uv[0][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_uv[0][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_UV_L0_WHT0 + 4 * i, val);

		val = ((param_post_cnr_h->weight_uv[1][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_uv[1][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_uv[1][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_uv[1][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_UV_L1_WHT0 + 4 * i, val);

		val = ((param_post_cnr_h->weight_uv[2][4 * i + 3] & 0xFF) << 24) |
			((param_post_cnr_h->weight_uv[2][4 * i + 2] & 0xFF) << 16) |
			((param_post_cnr_h->weight_uv[2][4 * i + 1] & 0xFF) << 8) |
			(param_post_cnr_h->weight_uv[2][4 * i] & 0xFF);
		ISP_REG_WR(idx, ISP_YUV_CNR_UV_L2_WHT0 + 4 * i, val);
	}

	return ret;
}

int isp_k_cfg_post_cnr_h(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_POST_CNR_H_BLOCK:
		ret = isp_k_post_cnr_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to idx %d, support cmd id = %d\n", idx, param->property);
		break;
	}

	return ret;
}
