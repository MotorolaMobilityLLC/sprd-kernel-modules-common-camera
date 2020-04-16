/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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
#include "sprd_isp_hw.h"

#include "isp_reg.h"
#include "cam_types.h"
#include "cam_block.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "PSTRZ: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static int isp_k_pstrz_block(struct isp_io_param *param, uint32_t idx)
{
	int ret = 0;
	uint32_t i = 0;
	uint32_t val = 0;
	uint32_t buf_sel;
	uint32_t buf_addr;
	struct isp_dev_posterize_info_v2 pstrz_info;

	ret = copy_from_user((void *)&pstrz_info,
			param->property_param,
			sizeof(pstrz_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_PSTRZ))
		pstrz_info.bypass = 1;
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, pstrz_info.bypass);
	if (pstrz_info.bypass)
		return 0;

	/* only cfg mode supported and no need pingpong buffer. */

	buf_addr = ISP_PSTRZ_R_BUF0;
	for (i = 0; i < POSTERIZE_NUM - 1; i++) {
		val = ((pstrz_info.pstrz_r_data[i + 1] & 0xFF) |
			(pstrz_info.pstrz_r_data[i] & 0xFF) << 8);
		ISP_REG_WR(idx, buf_addr +  i * 4, val);
	}

	buf_addr = ISP_PSTRZ_G_BUF0;
	for (i = 0; i < POSTERIZE_NUM - 1; i++) {
		val = ((pstrz_info.pstrz_g_data[i + 1] & 0xFF) |
			(pstrz_info.pstrz_g_data[i] & 0xFF) << 8);
		ISP_REG_WR(idx, buf_addr +  i * 4, val);
	}

	buf_addr = ISP_PSTRZ_B_BUF0;
	for (i = 0; i < POSTERIZE_NUM - 1; i++) {
		val = ((pstrz_info.pstrz_b_data[i + 1] & 0xFF) |
			(pstrz_info.pstrz_b_data[i] & 0xFF) << 8);
		ISP_REG_WR(idx, buf_addr +  i * 4, val);
	}

	buf_sel = 0;
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_3 | BIT_2,
		(buf_sel << 3) | (pstrz_info.sample_en << 2));

	return ret;
}

int isp_k_cfg_pstrz(struct isp_io_param *param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_POSTERIZE_BLOCK:
		ret = isp_k_pstrz_block(param, idx);
		break;
	default:
		pr_err("fail cmd id:%d, not supported.\n", param->property);
		break;
	}

	return ret;
}
