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

#ifndef _ADPT_DEFINES_H_
#define _ADPT_DEFINES_H_

/* configure fmcu with isp register offset, range is 0x620-0x630 */
#define ISP_OFFSET_RANGE		0x6300620

#define ISP_WIDTH_MAX   8048
#define ISP_HEIGHT_MAX  6036
#define DCAM_24M_WIDTH  5664
#define DCAM_PATH_WMAX			8048
#define DCAM_PATH_HMAX			6036
#define RAW_OVERLAP_UP				58
#define RAW_OVERLAP_DOWN			78
#define RAW_OVERLAP_LEFT			86
#define RAW_OVERLAP_RIGHT			138
/*
 *DCAM_CONTROL register bit map id
 * for force_cpy/auto_cpy control
 */
enum dcam_ctrl_id {
	DCAM_CTRL_CAP = (1 << 0),
	DCAM_CTRL_COEF = (1 << 1),
	DCAM_CTRL_RDS = (1 << 2),
	DCAM_CTRL_FULL = (1 << 3),
	DCAM_CTRL_BIN = (1 << 4),
	DCAM_CTRL_PDAF = (1 << 5),
	DCAM_CTRL_VCH2 = (1 << 6),
	DCAM_CTRL_VCH3 = (1 << 7),
};
#define DCAM_CTRL_ALL  0xff

static inline uint32_t cal_sprd_raw10_pitch(uint32_t w)
{
	w = (w * 16 + 127 ) /128 * 128 /8;
	return w;
}

static inline uint32_t cal_sprd_raw_pitch(uint32_t w)
{
	w = (w * 10 + 127) / 128 * 128 / 8;
	return w;
}
#endif
