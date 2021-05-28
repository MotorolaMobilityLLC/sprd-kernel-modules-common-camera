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
#include "isp_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCT: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_dct_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	uint32_t val = 0;
	struct isp_dev_dct_info *dct = NULL;

	dct = &isp_k_param->dct_info;
	ret = copy_from_user((void *)dct,
			param->property_param,
			sizeof(struct isp_dev_dct_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}

	ISP_REG_MWR(idx, ISP_YNR_DCT_PARAM, BIT_0, dct->bypass);
	if (dct->bypass)
		return 0;

	val = (dct->shrink_en << 1) | (dct->lnr_en << 2) |
		(dct->fnr_en << 3) | (dct->rnr_en << 4) |
		(dct->blend_en << 5) | (dct->direction_smooth_en << 6) |
		(dct->addback_en << 7);
	ISP_REG_MWR(idx, ISP_YNR_DCT_PARAM, 0xFE, val);

	val = (dct->coef_thresh0 & 0x1F) | ((dct->coef_thresh1& 0x1F) << 8) |
		((dct->coef_thresh2 & 0x1F) << 16) | ((dct->coef_thresh3 & 0x1F) << 24);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM1, val);

	val = (dct->coef_ratio0 & 0xFF) | ((dct->coef_ratio1 & 0xFF) << 8) |
		((dct->coef_ratio2 & 0xFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM2, val);

	val = (dct->luma_thresh0 & 0x1F) | ((dct->luma_thresh1 & 0x1F) << 8) |
		((dct->luma_ratio0 & 0xFF) << 16) | ((dct->luma_ratio1 & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM3, val);

	val = (dct->flat_th & 0xFFF) | ((dct->fnr_thresh0 & 0xFF) << 12) |
		((dct->fnr_thresh1 & 0xFF) << 20);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM4, val);

	val = (dct->fnr_ratio0 & 0xFF) | ((dct->fnr_ratio1 & 0xFF) << 8) |
		((dct->rnr_radius & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM5, val);

	val = (dct->rnr_imgCenterX & 0xFFFF) | ((dct->rnr_imgCenterY & 0xFFFF) << 16);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM6, val);

	val = (dct->rnr_step & 0xFF) | ((dct->rnr_ratio0 & 0xFF) << 8) |
		((dct->rnr_ratio1 & 0xFF) << 16) | ((dct->blend_weight & 0xFF) << 24);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM7, val);

	val = (dct->blend_radius & 0x3) | ((dct->blend_epsilon & 0x3FFF) << 2) |
		((dct->blend_thresh0 & 0x1F) << 16) | ((dct->blend_thresh1 & 0x1F) << 21);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM8, val);

	val = (dct->blend_ratio0 & 0xFF) | ((dct->blend_ratio1 & 0xFF) << 8) |
		((dct->direction_mode & 0x7) << 16) | ((dct->direction_thresh_diff & 0x1FF) << 19);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM9, val);

	val = (dct->direction_thresh_min & 0x1FF) | ((dct->direction_freq_hop_total_num_thresh & 0x7F) << 9) |
		((dct->direction_freq_hop_thresh & 0xFF) << 16) | ((dct->direction_freq_hop_control_en & 0x1) << 24);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM10, val);

	val = (dct->direction_hop_thresh_diff & 0x1FF) | ((dct->direction_hop_thresh_min & 0x1FF) << 16);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM11, val);

	val = (dct->addback_ratio & 0xFF) | ((dct->addback_clip & 0xFF) << 8);
	ISP_REG_WR(idx, ISP_YNR_DCT_PARAM12, val);

	return ret;
}

int isp_k_cfg_dct(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_DCT_BLOCK:
		ret = isp_k_dct_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
