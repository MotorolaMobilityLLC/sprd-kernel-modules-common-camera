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

#define  ISP_FRGB_HSV_BUF0             0
#define  ISP_FRGB_HSV_BUF1             1


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "HSV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


static int isp_k_hsv_block(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	uint32_t val = 0;
	unsigned long buf_addr = 0;
	struct isp_dev_hsv_info_v3 *hsv_info;
	hsv_info = &isp_k_param->hsv_info3;
	ret = copy_from_user((void *)hsv_info,
			param->property_param,
			sizeof(struct isp_dev_hsv_info_v3));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}
	if (g_isp_bypass[idx] & (1 << _EISP_HSV))
		hsv_info->hsv_bypass = 1;
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, hsv_info->hsv_bypass);
	if (hsv_info->hsv_bypass)
		return 0;
	val = ((hsv_info->hsv_bypass & 0x1) ) |
		((hsv_info->buf_param.hsv_buf_sel & 0x1) << 2) |
		(hsv_info->hsv_delta_value_en & 3);
	ISP_REG_WR(idx, ISP_HSV_PARAM, val);
	val = (hsv_info->hsv_hue_thr[0][0] & 0x1F) |
		((hsv_info->hsv_hue_thr[0][1] & 0x1F) << 5) |
		((hsv_info->hsv_hue_thr[1][0] & 0x1F) << 10) |
		((hsv_info->hsv_hue_thr[1][1] & 0x1F) << 15) |
		((hsv_info->hsv_hue_thr[2][0] & 0x1F) << 20) |
		((hsv_info->hsv_hue_thr[2][1] & 0x1F) << 25);
	ISP_REG_WR(idx, ISP_HSV_CFG0, val);
	val = (hsv_info->hsv_param[0].hsv_curve_param.start_a & 0x3FF) |
		((hsv_info->hsv_param[0].hsv_curve_param.end_a & 0x3FF) << 10) |
		((hsv_info->hsv_param[0].hsv_curve_param.start_b & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_HSV_CFG1, val);
	val = (hsv_info->hsv_param[0].hsv_curve_param.end_b & 0x3FF) |
		((hsv_info->hsv_param[1].hsv_curve_param.start_a & 0x3FF) << 10) |
		((hsv_info->hsv_param[1].hsv_curve_param.end_a & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_HSV_CFG2, val);
	val = (hsv_info->hsv_param[1].hsv_curve_param.start_b & 0x3FF) |
		((hsv_info->hsv_param[1].hsv_curve_param.end_b & 0x3FF) << 10) |
		((hsv_info->hsv_param[2].hsv_curve_param.start_a & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_HSV_CFG3, val);
	val = (hsv_info->hsv_param[2].hsv_curve_param.end_a & 0x3FF) |
		((hsv_info->hsv_param[2].hsv_curve_param.start_b & 0x3FF) << 10) |
		((hsv_info->hsv_param[2].hsv_curve_param.end_b & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_HSV_CFG4, val);
	val = (hsv_info->hsv_param[3].hsv_curve_param.start_a & 0x3FF) |
		((hsv_info->hsv_param[3].hsv_curve_param.end_a & 0x3FF) << 10) |
		((hsv_info->hsv_param[3].hsv_curve_param.start_b & 0x3FF) << 20);
	ISP_REG_WR(idx, ISP_HSV_CFG5, val);
	val = (hsv_info->hsv_param[3].hsv_curve_param.end_b & 0x3FF) |
		((hsv_info->y_blending_factor & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_HSV_CFG6, val);
	for(i = 0; i < 12; i++) {
		val = (hsv_info->hsv_1d_hue_lut[2*i] & 0x1FF) |
			((hsv_info->hsv_1d_hue_lut[2*i+1] & 0x1FF) << 9) |
			((hsv_info->hsv_1d_sat_lut[i] & 0x1FFF) << 18);
		ISP_REG_WR(idx, ISP_HSV_CFG7+i*4, val);
	}
	val = (hsv_info->hsv_1d_hue_lut[24] & 0x1FF) |
		((hsv_info->hsv_1d_sat_lut[12] & 0x1FFF) << 16);
	ISP_REG_WR(idx, ISP_HSV_CFG19, val);
	val = (hsv_info->hsv_1d_sat_lut[13] & 0x1FFF) |
		((hsv_info->hsv_1d_sat_lut[14] & 0x1FFF) << 16);
	ISP_REG_WR(idx, ISP_HSV_CFG20, val);
	val = (hsv_info->hsv_1d_sat_lut[15] & 0x1FFF) |
		((hsv_info->hsv_1d_sat_lut[16] & 0x1FFF) << 16);
	ISP_REG_WR(idx, ISP_HSV_CFG21, val);
	for (i = 0; i < 25; i++) {
		for (j = 0; j < 17; j++) {
			if (((i + 1) % 2 == 1) && ((j + 1) % 2 == 1)) {
				if (hsv_info->buf_param.hsv_buf_sel == ISP_FRGB_HSV_BUF0) {
					buf_addr = ISP_HSV_A_BUF0_CH0;
					ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_hue_lut_reg[i][j]);
				} else {
					buf_addr = ISP_HSV_A_BUF1_CH0;
					ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_sat_lut[i][j]);
				}
			} else {
				if (((i + 1) % 2 == 1) && ((j + 1) % 2 != 1)) {
					if (hsv_info->buf_param.hsv_buf_sel == ISP_FRGB_HSV_BUF0) {
						buf_addr = ISP_HSV_B_BUF0_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_hue_lut_reg[i][j]);
					} else {
						buf_addr = ISP_HSV_B_BUF1_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_sat_lut[i][j]);
					}
				}
				if (((i + 1) % 2 != 1) && ((j + 1) % 2 == 1)) {
					if (hsv_info->buf_param.hsv_buf_sel == ISP_FRGB_HSV_BUF0) {
						buf_addr = ISP_HSV_C_BUF0_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_hue_lut_reg[i][j]);
					} else {
						buf_addr = ISP_HSV_C_BUF1_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_sat_lut[i][j]);
					}
				}
				if (((i + 1) % 2 != 1) && ((j + 1) % 2 != 1)) {
					if (hsv_info->buf_param.hsv_buf_sel == ISP_FRGB_HSV_BUF0) {
						buf_addr = ISP_HSV_D_BUF0_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_hue_lut_reg[i][j]);
					} else {
						buf_addr = ISP_HSV_D_BUF1_CH0;
						ISP_REG_WR(idx, buf_addr, hsv_info->buf_param.hsv_2d_sat_lut[i][j]);
					}
				}
			}
		}
	}
	return ret;
}

int isp_k_cfg_hsv(struct isp_io_param *param,
	struct isp_k_block *isp_k_param, uint32_t idx)
{
	int ret = 0;

	switch (param->property) {
	case ISP_PRO_HSV_BLOCK:
		ret = isp_k_hsv_block(param, isp_k_param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}

	return ret;
}
