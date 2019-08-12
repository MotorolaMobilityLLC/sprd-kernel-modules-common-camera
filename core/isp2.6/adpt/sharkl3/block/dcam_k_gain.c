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
#include "dcam_reg.h"
#include "dcam_interface.h"
#include "cam_types.h"
#include "cam_block.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "RGBG: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

enum {
	_UPDATE_DITH = BIT(0),
	_UPDATE_GAIN = BIT(1),
};
int dcam_k_rgb_gain_block(struct dcam_dev_param *param)
{
	int ret = 0;
	uint32_t idx = param->idx;
	uint32_t val = 0;
	struct dcam_dev_rgb_gain_info *p;

	if (param == NULL)
		return -1;
	if (!(param->rgb.update & _UPDATE_GAIN))
		return 0;
	param->rgb.update &= (~(_UPDATE_GAIN));
	p = &(param->rgb.gain_info);

	DCAM_REG_MWR(idx, ISP_RGBG_PARAM, BIT_0, p->bypass);
	if (p->bypass)
		return 0;

	val = (p->global_gain & 0xFFFF) << 16;
	DCAM_REG_MWR(idx, ISP_RGBG_PARAM, 0xFFFF0000, val);
	val = ((p->r_gain & 0xFFFF) << 16) | (p->b_gain & 0xFFFF);
	DCAM_REG_WR(idx, ISP_RGBG_RB, val);
	val = p->g_gain & 0xFFFF;
	DCAM_REG_WR(idx, ISP_RGBG_G, val);

	return ret;
}


int dcam_k_rgb_dither_random_block(struct dcam_dev_param *param)
{
	int ret = 0;
	uint32_t idx = param->idx;
	uint32_t val = 0;
	struct dcam_dev_rgb_dither_info *p;

	if (param == NULL)
		return -1;
	if (!(param->rgb.update & _UPDATE_DITH))
		return 0;
	param->rgb.update &= (~(_UPDATE_DITH));

	p = &(param->rgb.rgb_dither);
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0,
			BIT_0, p->random_bypass);
	if (p->random_bypass)
		return 0;

	val = ((p->seed & 0xFFFFFF) << 8)
		| ((p->random_mode & 0x1) << 2);
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0, 0xFFFFFF04, val);

	val = (p->r_shift & 0xF)
		| ((p->range & 0x7) << 4)
		| ((p->r_offset & 0x7FF) << 16);
	DCAM_REG_WR(idx, ISP_RGBG_YRANDOM_PARAMETER1, val);

	val = (p->takebit[0] & 0xF)
		| ((p->takebit[1] & 0xF) << 4)
		| ((p->takebit[2] & 0xF) << 8)
		| ((p->takebit[3] & 0xF) << 12)
		| ((p->takebit[4] & 0xF) << 16)
		| ((p->takebit[5] & 0xF) << 20)
		| ((p->takebit[6] & 0xF) << 24)
		| ((p->takebit[7] & 0xF) << 28);
	DCAM_REG_WR(idx, ISP_RGBG_YRANDOM_PARAMETER2, val);

	return ret;
}

int dcam_k_cfg_rgb_gain(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;

	if (param->property_param == NULL || NULL == p) {
		pr_err("fail to get property_param\n");
		return -1;
	}
	/* debug bypass rgb gain */
	if (g_dcam_bypass[p->idx] & (1 << _E_RGB))
		return 0;

	switch (param->property) {
	case DCAM_PRO_GAIN_BLOCK:
		if (DCAM_ONLINE_MODE) {
			/* memset(&gain_info, 0x00, sizeof(gain_info)); */
			ret = copy_from_user((void *)&(p->rgb.gain_info),
				param->property_param,
				sizeof(p->rgb.gain_info));
			if (ret) {
				pr_err("fail to copy from user, ret = %d\n",
					ret);
				return -1;
			}
			p->rgb.update |= _UPDATE_GAIN;
			ret = dcam_k_rgb_gain_block(p);
		} else {
			mutex_lock(&p->param_lock);
			ret = copy_from_user((void *)&(p->rgb.gain_info),
				param->property_param,
				sizeof(p->rgb.gain_info));
			if (ret) {
				mutex_unlock(&p->param_lock);
				pr_err("fail to copy from user, ret = %d\n",
					ret);
				return -1;
			}
			p->rgb.update |= _UPDATE_GAIN;
			mutex_unlock(&p->param_lock);
		}
		break;
	default:
		pr_err("fail to support property %d\n",
			param->property);
		break;
	}

	return ret;
}

int dcam_k_cfg_rgb_dither(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;

	/* debugfs bypass rgb dither(rand) */
	if (g_dcam_bypass[p->idx] & (1 << _E_RAND))
		return 0;

	switch (param->property) {
	case DCAM_PRO_GAIN_DITHER_BLOCK:
		if (DCAM_ONLINE_MODE) {
			/* memset(&rgb_gain, 0x00, sizeof(rgb_gain)); */

			ret = copy_from_user((void *)&(p->rgb.rgb_dither),
				param->property_param,
				sizeof(p->rgb.rgb_dither));
			if (ret) {
				pr_err("fail to copy from user, ret = %d\n",
					ret);
				return -1;
			}
			p->rgb.update |= _UPDATE_DITH;
			ret = dcam_k_rgb_dither_random_block(p);
		} else {
			mutex_lock(&p->param_lock);
			ret = copy_from_user((void *)&(p->rgb.rgb_dither),
				param->property_param,
				sizeof(p->rgb.rgb_dither));
			if (ret) {
				mutex_unlock(&p->param_lock);
				pr_err("fail to copy from user, ret = %d\n",
					ret);
				return -1;
			}
			p->rgb.update |= _UPDATE_DITH;
			mutex_unlock(&p->param_lock);
		}
		break;
	default:
		pr_err("fail to support property %d\n",
			param->property);
		ret = -EINVAL;
		break;
	}

	return ret;
}
