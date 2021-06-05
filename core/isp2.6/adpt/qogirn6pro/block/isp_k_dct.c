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
	struct isp_dev_dct_info *dct = NULL;

	dct = &isp_k_param->dct_info;
	ret = copy_from_user((void *)dct, param->property_param,
			sizeof(struct isp_dev_dct_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return ret;
	}

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
