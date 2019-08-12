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
#include "dcam_core.h"
#include "dcam_reg.h"
#include "dcam_interface.h"
#include "cam_types.h"
#include "cam_block.h"


#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "PDAF: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

enum {
	_UPDATE_BYPASS = BIT(0),
};

static int isp_k_pdaf_type1_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_pdaf_type2_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}
static int isp_k_pdaf_type3_block(struct isp_io_param *param, void *in)
{
	int ret = 0;
	enum dcam_id idx;
	struct dev_dcam_vc2_control vch2_info;
	struct dcam_dev_param *p = (struct dcam_dev_param *)in;
	struct dcam_pipe_dev  *dev = (struct dcam_pipe_dev *)p->dev;

	idx = dev->idx;
	dev->pdaf_type = 3;
	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}
	pr_info("pdaf_info.vch2_mode = %d\n", vch2_info.vch2_mode);
	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_dual_pdaf_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	struct dev_dcam_vc2_control vch2_info;

	memset(&vch2_info, 0x00, sizeof(vch2_info));
	ret = copy_from_user((void *)&vch2_info,
		param->property_param, sizeof(vch2_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_WR(idx, DCAM_PDAF_CONTROL,
		(vch2_info.vch2_vc & 0x03) << 16
		|(vch2_info.vch2_data_type & 0x3f) << 8
		|(vch2_info.vch2_mode & 0x03));

	return ret;
}

static int isp_k_pdaf_block(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int val = 0;
	struct isp_dev_pdaf_info pdaf_info;
	struct pdaf_ppi_info ppi_info;

	memset(&pdaf_info, 0x00, sizeof(pdaf_info));
	memset(&ppi_info, 0x00, sizeof(ppi_info));
	ret = copy_from_user((void *)&pdaf_info,
		param->property_param, sizeof(pdaf_info));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	/* phase map corr en */
	val = pdaf_info.phase_map_corr_en;
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_3, val << 3);

	/* ppi grid mode */
	val = pdaf_info.grid_mode;
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_8, val << 8);

	return ret;
}

static int isp_k_pdaf_bypass(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int bypass = 0;

	ret = copy_from_user((void *)&bypass,
		param->property_param, sizeof(unsigned int));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	bypass = !!bypass;
	DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_0, bypass);

	return ret;
}

static int isp_k_pdaf_set_skip_num(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	unsigned int skip_num = 0;

	ret = copy_from_user((void *)&skip_num,
			param->property_param, sizeof(skip_num));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	DCAM_REG_MWR(idx, DCAM_PDAF_SKIP_FRM, 0xf0, skip_num << 4);

	return ret;
}

static int isp_k_pdaf_set_mode(struct isp_io_param *param, enum dcam_id idx)
{
	int ret = 0;
	uint32_t mode = 0;

	ret = copy_from_user((void *)&mode,
			param->property_param, sizeof(mode));
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		return -1;
	}

	/* single/multi mode */
	mode = !!mode;
	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_2, mode << 2);
	DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_3, mode << 3);

	return ret;
}

int dcam_k_cfg_pdaf(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;
	enum dcam_id idx;
	struct dcam_pipe_dev *dev = NULL;
	dev = (struct dcam_pipe_dev *)p->dev;

	if (!param || !p) {
		pr_err("fail to get param\n");
		return -1;
	}

	if (param->property_param == NULL) {
		pr_err("fail to get property_param\n");
		return -1;
	}

	idx = p->idx;
	dev->is_pdaf = 1;
	switch (param->property) {
	case DCAM_PRO_PDAF_BLOCK:
		ret = isp_k_pdaf_block(param, idx);
		break;
	case DCAM_PRO_PDAF_BYPASS:
		ret = isp_k_pdaf_bypass(param, idx);
		break;
	case DCAM_PRO_PDAF_SET_MODE:
		ret = isp_k_pdaf_set_mode(param, idx);
		break;
	case DCAM_PRO_PDAF_SET_SKIP_NUM:
		ret = isp_k_pdaf_set_skip_num(param, idx);
		break;
	case DCAM_PRO_PDAF_TYPE1_BLOCK:
		ret = isp_k_pdaf_type1_block(param, idx);
		break;
	case DCAM_PRO_PDAF_TYPE2_BLOCK:
		ret = isp_k_pdaf_type2_block(param, idx);
		break;
	case DCAM_PRO_PDAF_TYPE3_BLOCK:
		ret = isp_k_pdaf_type3_block(param, p);
		break;
	case DCAM_PRO_DUAL_PDAF_BLOCK:
		ret = isp_k_dual_pdaf_block(param, idx);
		break;
	default:
		pr_err("fail to support cmd id = %d\n",
			param->property);
		break;
	}
	pr_info("idx %d, Sub %d, ret %d\n", idx, param->property, ret);

	return ret;
}
