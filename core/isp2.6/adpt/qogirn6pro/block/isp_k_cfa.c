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
#include "dcam_reg.h"
#include "cam_types.h"
#include "cam_block.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CFA: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_cfa_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	uint32_t val = 0;
	struct isp_dev_cfa_info_v1 *cfa_info;

	cfa_info = &isp_k_param->cfa_info_v1;

	ret = copy_from_user((void *)cfa_info,
			param->property_param,
			sizeof(struct isp_dev_cfa_info_v1));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}

	if (g_isp_bypass[idx] & (1 << _EISP_CFA))
		cfa_info->bypass = 1;

	DCAM_REG_MWR(idx, DCAM_CFA_NEW_CFG0, BIT_0, cfa_info->bypass);
	if (cfa_info->bypass)
		return 0;

	val = (cfa_info->grid_thr & 0xFFFF);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG0, val);

	val = ((cfa_info->weight_control_bypass & 0x1) << 31) |
			((cfa_info->uni_dir_intplt_thr_new & 0xFFF) << 12) |
			((cfa_info->strong_edge_thr & 0xFF) << 4);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG1, val);

	val = (cfa_info->cdcr_adj_factor & 0x3F) |
			((cfa_info->smooth_area_thr & 0x1FFFF) << 8);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG2, val);

	val = ((cfa_info->grid_dir_weight_t2 & 0x1F) << 20) |
			((cfa_info->grid_dir_weight_t1 & 0x1F) << 12) |
			(cfa_info->readblue_high_sat_thr & 0x3FF);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG3, val);

	val = (cfa_info->round_diff_03_thr & 0xFFF) |
			((cfa_info->low_lux_03_thr & 0x3FF) << 16);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG4, val);

	val = (cfa_info->round_diff_12_thr & 0xFFF) |
			((cfa_info->low_lux_12_thr & 0x3FF) << 16);
	DCAM_REG_WR(idx, DCAM_CFA_INTP_CFG5, val);

	return ret;
}

int isp_k_cfg_cfa(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_CFA_BLOCK:
		ret = isp_k_cfa_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
