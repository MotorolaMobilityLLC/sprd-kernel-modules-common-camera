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
#define pr_fmt(fmt) "GAMMA: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_gamma_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	uint32_t i, chn, buf_sel = 0;
	uint32_t val;
	struct isp_dev_gamma_info_v1 *gamma_info = NULL;

	gamma_info = &isp_k_param->gamma_info_v1;

	ret = copy_from_user((void *)gamma_info,
			param->property_param,
			sizeof(struct isp_dev_gamma_info_v1));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return  ret;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_GAMC))
		gamma_info->bypass = 1;
	DCAM_REG_MWR(idx, DCAM_FGAMMA10_PARAM, BIT_0,
			gamma_info->bypass);
	if (gamma_info->bypass)
		return 0;

	/* only cfg mode and buf0 is selected. */
	DCAM_REG_MWR(idx, DCAM_FGAMMA10_PARAM, BIT_1, buf_sel << 1);

	for (chn = 0; chn < 3; chn++) {
		val = (chn & 0x3) << 8;
		DCAM_REG_MWR(idx, DCAM_BUF_CTRL, 0x300, val);
		if (chn == 0) {
			for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
				val = gamma_info->gain_r[i];
				DCAM_REG_WR(idx, DCAM_FGAMMA10_TABLE + i * 4, val);
			}
		} else if (chn == 1) {
			for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
				val = gamma_info->gain_g[i];
				DCAM_REG_WR(idx, DCAM_FGAMMA10_TABLE + i * 4, val);
			}
		} else {
			for (i = 0; i < ISP_FRGB_GAMMA_PT_NUM - 1; i++) {
				val = gamma_info->gain_b[i];
				DCAM_REG_WR(idx, DCAM_FGAMMA10_TABLE + i * 4, val);
			}
		}
	}

	return ret;
}

int isp_k_cfg_gamma(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_GAMMA_BLOCK:
		ret = isp_k_gamma_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
