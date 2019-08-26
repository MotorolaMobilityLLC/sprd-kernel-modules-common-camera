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

#include <linux/kernel.h>
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
#define pr_fmt(fmt) "BPC: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define DCAM_3DNR_ROI_SIZE_ALIGN 16u
#define DCAM_3DNR_ROI_LINE_CUT 32u


enum {
	_UPDATE_NR3 = BIT(0),
};

struct roi_size {
	uint32_t roi_width;
	uint32_t roi_height;
};

/* input: rect: bin path crop size, include start point(x,y), and size(w,h)
 */
void dcam_k_3dnr_set_roi(struct isp_img_rect rect,
			 uint32_t project_mode, uint32_t idx)
{
	uint32_t roi_w, roi_h, roi_x = 0, roi_y = 0;
	uint32_t lbuf_share_mode = 0;

	/* get max roi size
	 * max roi size should be half of normal value if project_mode is off
	 */
	lbuf_share_mode = DCAM_AXIM_RD(DCAM_LBUF_SHARE_MODE);
	if (project_mode == 0)
		if (rect.w > 2012)
			lbuf_share_mode = 1 << 28;
		else
			lbuf_share_mode = 0 << 28;
	else
		if (rect.w > 4024)
			lbuf_share_mode = 1 << 28;
		else
			lbuf_share_mode = 0 << 28;

	DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, BIT_28, lbuf_share_mode);

	/* get roi and align to 16 pixels */
	roi_w = ALIGN_DOWN(rect.w, DCAM_3DNR_ROI_SIZE_ALIGN);
	roi_h = ALIGN_DOWN(rect.h, DCAM_3DNR_ROI_SIZE_ALIGN);

	/* get offset */
	roi_x = (rect.w - roi_w) >> 1;
	roi_y = (rect.h - roi_h) >> 1;
	roi_x = ALIGN_DOWN(roi_x + rect.x, 2);
	roi_y = ALIGN_DOWN(roi_y + rect.y, 2);

	/* almost done! */
	DCAM_REG_WR(idx, NR3_FAST_ME_ROI_PARAM0, roi_x << 16 | roi_y);
	DCAM_REG_WR(idx, NR3_FAST_ME_ROI_PARAM1, roi_w << 16 | roi_h);

	pr_info("DCAM%u 3DNR ROI %u %u %u %u\n",
		idx, roi_x, roi_y, roi_w, roi_h);
}

int dcam_k_3dnr_me(struct dcam_dev_param *param)
{
	int ret = 0;
	uint32_t idx = param->idx;
	struct dcam_pipe_dev *dev = param->dev;
	struct dcam_dev_3dnr_me *p = NULL; /* nr3_me; */
	struct dcam_path_desc *path;
	struct isp_img_rect rect;

	if (param == NULL)
		return -EPERM;
	/* update ? */
	if (!(param->nr3.update & _UPDATE_NR3))
		return 0;
	param->nr3.update &= (~(_UPDATE_NR3));
	/* debugfs bypass nr3 */
	if (g_dcam_bypass[idx] & (1 << _E_NR3))
		return 0;

	p = &param->nr3.nr3_me;
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM,
			BIT(0), (p->bypass & 0x1));
	if (p->bypass)
		return 0;

	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM,
		0x30, (p->nr3_project_mode & 0x3) << 4);
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM,
		0xC0, (p->nr3_channel_sel & 0x3) << 6);

	/* nr3_mv_bypass:  0 - calc by hardware, 1 - not calc  */
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM, BIT(8), 0 << 8);

	/*  output_en = 0 : project value not output to ddr.  */
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM, BIT(2), 0 << 2);

	/* update ROI according to project_mode */
	path = &dev->path[DCAM_PATH_3DNR];
	rect.x = path->in_trim.start_x;
	rect.y = path->in_trim.start_y;
	rect.w = path->in_trim.size_x;
	rect.h = path->in_trim.size_y;
	if ((rect.x + rect.w) <= dev->cap_info.cap_size.size_x &&
		(rect.y + rect.h) <= dev->cap_info.cap_size.size_y)
	dcam_k_3dnr_set_roi(rect,
			    p->nr3_project_mode, idx);

	/*  sub_me_bypass.  */
	DCAM_REG_MWR(idx, NR3_FAST_ME_PARAM, BIT(3), 0 << 3);

	return ret;
}

int dcam_k_cfg_3dnr_me(struct isp_io_param *param, struct dcam_dev_param *p)
{
	int ret = 0;

	switch (param->property) {
	case DCAM_PRO_3DNR_ME:
		if (DCAM_ONLINE_MODE) {
			ret = copy_from_user((void *)&(p->nr3.nr3_me),
					param->property_param,
					sizeof(p->nr3.nr3_me));
			if (ret) {
				pr_err("blc_block: copy error, ret=0x%x\n",
					(unsigned int)ret);
				return -EPERM;
			}
			p->nr3.update |= _UPDATE_NR3;
			ret = dcam_k_3dnr_me(p);
		} else {
			mutex_lock(&p->param_lock);
			ret = copy_from_user((void *)&(p->nr3.nr3_me),
					param->property_param,
					sizeof(p->nr3.nr3_me));
			if (ret) {
				mutex_unlock(&p->param_lock);
				pr_err("blc_block: copy error, ret=0x%x\n",
					(unsigned int)ret);
				return -EPERM;
			}
			p->nr3.update |= _UPDATE_NR3;
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
