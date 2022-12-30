/*
 * Copyright (C) 2021-2022 UNISOC Communications Inc.
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

#include <linux/err.h>
#include <sprd_mm.h>

#include "dcam_core.h"
#include "cam_queue.h"
#include "sprd_cam.h"
#include "dcam_hw_adpt.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_SLICE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define ISP_SLICE_OVERLAP_W_MAX         64

uint32_t dcamslice_needed_info_get(struct dcam_hw_context *hw_ctx, uint32_t *dev_lbuf, uint32_t in_width)
{
	uint32_t out_width = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_lbuf_share lbufarg;
	struct cam_hw_lbuf_share camarg;

	hw = hw_ctx->hw;
	lbufarg.idx = hw_ctx->hw_ctx_id;
	lbufarg.width = 0;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_GET, &lbufarg);
	out_width = lbufarg.width;

	camarg.idx = hw_ctx->hw_ctx_id;
	camarg.width = in_width;
	camarg.offline_flag = 1;
	pr_debug("Dcam%d, sw_ctx%d, new width %d old linebuf %d\n", camarg.idx, hw_ctx->node_id, camarg.width, lbufarg.width);
	if (hw->ip_dcam[hw_ctx->hw_ctx_id]->lbuf_share_support && (lbufarg.width < camarg.width)) {
		hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_SET, &camarg);
		lbufarg.idx = hw_ctx->hw_ctx_id;
		lbufarg.width = 0;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_LBUF_SHARE_GET, &lbufarg);
		out_width = lbufarg.width;
	}
	if (!out_width)
		*dev_lbuf = DCAM_PATH_WMAX * 2;
	else
		*dev_lbuf = out_width;
	if (in_width > *dev_lbuf)
		return 1;
	else
		return 0;
}

int dcamslice_num_info_get(struct img_size *src, struct img_size *dst)
{
	uint32_t slice_num = 0, slice_w = 0, slice_w_out = 0;
	uint32_t slice_max_w = 0, max_w = 0;
	uint32_t linebuf_len = 0;
	uint32_t input_w = src->w;
	uint32_t output_w = dst->w;

	/* based input */
	linebuf_len = g_camctrl.isp_linebuf_len;
	max_w = input_w;
	slice_num = 1;
	slice_max_w = linebuf_len - ISP_SLICE_OVERLAP_W_MAX;
	if (max_w <= linebuf_len) {
		slice_w = max_w;
	} else {
		do {
			slice_num++;
			slice_w = (max_w + slice_num - 1) / slice_num;
		} while (slice_w >= slice_max_w);
	}
	pr_debug("input_w %d, slice_num %d, slice_w %d\n",
		max_w, slice_num, slice_w);

	/* based output */
	max_w = output_w;
	slice_num = 1;
	slice_max_w = linebuf_len;
	if (max_w > 0) {
		if (max_w > linebuf_len) {
			do {
				slice_num++;
				slice_w_out = (max_w + slice_num - 1) / slice_num;
			} while (slice_w_out >= slice_max_w);
		}
		/* set to equivalent input size, because slice size based on input. */
		slice_w_out = (input_w + slice_num - 1) / slice_num;
	} else
		slice_w_out = slice_w;
	pr_debug("max output w %d, slice_num %d, out limited slice_w %d\n",
		max_w, slice_num, slice_w_out);

	slice_w = MIN(slice_w, slice_w_out);
	slice_w = ALIGN(slice_w, 2);
	slice_num = (input_w + slice_w - 1) / slice_w;
	if (dst->h > DCAM_SW_SLICE_HEIGHT_MAX)
		slice_num *= 2;
	return slice_num;
}

int dcamslice_hw_info_set(struct dcam_offline_slice_info *slice, struct camera_frame *pframe, uint32_t slice_wmax)
{
	int ret = 0, i = 0;
	uint32_t w = 0, offset = 0;
	uint32_t slc_w = 0, f_align = 0;

	w = pframe->width;
	if (w <= slice_wmax) {
		slc_w = w;
		goto slices;
	}

	if (slice->fetch_fmt == CAM_RAW_14)
		f_align = 8;/* 8p * 16bits/p = 128 bits fetch aligned */
	else
		f_align = 64;/* 64p * 10bits/p = 128bits * 5 */

	slc_w = slice_wmax / f_align * f_align;

	/* can not get valid slice w aligned  */
	if ((slc_w > slice_wmax) ||
		(slc_w * DCAM_OFFLINE_SLC_MAX) < pframe->width) {
		pr_err("dcam failed, pic_w %d, slc_limit %d, algin %d\n",
				pframe->width, slice_wmax, f_align);
		return -EFAULT;
	}

slices:
	while (w > 0) {
		slice->slice_trim[i].start_x = offset;
		slice->slice_trim[i].start_y = 0;
		slice->slice_trim[i].size_x = (w > slc_w) ? (slc_w - DCAM_OVERLAP) : w;
		slice->slice_trim[i].size_y = pframe->height;
		pr_info("slc%d, (%d %d %d %d), limit %d\n", i,
			slice->slice_trim[i].start_x, slice->slice_trim[i].start_y,
			slice->slice_trim[i].size_x, slice->slice_trim[i].size_y,
			slice_wmax);

		w -= slice->slice_trim[i].size_x;
		offset += slice->slice_trim[i].size_x;
		i++;
	}
	slice->slice_num = i;
	slice->slice_count = i;

	return ret;
}

int dcamslice_trim_info_get(uint32_t width, uint32_t heigth, uint32_t slice_num, uint32_t slice_no, struct img_trim *slice_trim)
{
	int ret = 0;
	uint32_t slice_w = 0, slice_h = 0;
	uint32_t slice_x_num = 0, slice_y_num = 0;
	uint32_t start_x = 0, size_x = 0;
	uint32_t start_y = 0, size_y = 0;

	if (!width || !slice_num || !slice_trim) {
		pr_err("fail to get valid param %d, %d, %p.\n", width, slice_num, slice_trim);
		return -EFAULT;
	}

	if (heigth > DCAM_SW_SLICE_HEIGHT_MAX) {
		slice_x_num = slice_num / 2;
		slice_y_num = 2;
	} else {
		slice_x_num = slice_num;
		slice_y_num = 1;
	}
	slice_w = width / slice_x_num;
	slice_w = ALIGN(slice_w, 2);
	slice_h = heigth / slice_y_num;
	slice_h = ALIGN(slice_h, 2);

	start_x = slice_w * (slice_no % slice_x_num);
	size_x = slice_w;
	if (size_x & 0x03) {
		if (slice_no != 0) {
			start_x -=  ALIGN(size_x, 4) - size_x;
			size_x = ALIGN(size_x, 4);
		} else {
			start_x -=  ALIGN(size_x, 4);
			size_x = ALIGN(size_x, 4);
		}
	}

	start_y = (slice_no / slice_x_num) * slice_h;
	size_y = slice_h;

	slice_trim->start_x = start_x;
	slice_trim->size_x = size_x;
	slice_trim->start_y = start_y;
	slice_trim->size_y = size_y;
	pr_debug("slice %d [%d, %d, %d, %d].\n", slice_no, start_x, size_x, start_y, size_y);
	return ret;
}

int dcam_slice_info_cal(struct dcam_offline_slice_info *slice, struct camera_frame *pframe, uint32_t lbuf_width)
{
	int ret = 0;
	uint32_t slice_no = 0;

	if (slice->dcam_slice_mode != CAM_OFFLINE_SLICE_SW)
		slice->dcam_slice_mode = CAM_OFFLINE_SLICE_HW;

	if (slice->dcam_slice_mode == CAM_OFFLINE_SLICE_HW)
		ret = dcamslice_hw_info_set(slice, pframe, lbuf_width);
	if (slice->dcam_slice_mode == CAM_OFFLINE_SLICE_SW) {
		for (slice_no = 0; slice_no < slice->slice_num; slice_no++)
			dcamslice_trim_info_get(pframe->width, pframe->height, slice->slice_num,
					slice_no, &slice->slice_trim[slice_no]);
	}
	slice->slice_count = slice->slice_num;
	return 0;
}

