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

/* configure fmcu with isp register offset, range is 0x60b-0x60c */
#define ISP_OFFSET_RANGE		0x60c060b

/*
 *DCAM_CONTROL register bit map id
 * for force_cpy/auto_cpy control
 */
enum dcam_ctrl_id {
	DCAM_CTRL_CAP = (1 << 0),
	DCAM_CTRL_RDS = (1 << 1),
	DCAM_CTRL_FULL = (1 << 2),
	DCAM_CTRL_BIN = (1 << 3),
	DCAM_CTRL_AEM = (1 << 4),
	DCAM_CTRL_PDAF = (1 << 5),
	DCAM_CTRL_VCH2 = (1 << 6),
	DCAM_CTRL_VCH3 = (1 << 7),
};
#define DCAM_CTRL_ALL  0xff

static inline uint32_t cal_sprd_raw_pitch(uint32_t w)
{
	uint32_t mod16_len[16] = {0, 8, 8, 8, 8, 12, 12, 12,
				12, 16, 16, 16, 16, 20, 20, 20};

	return ((w >> 4) * 20 + (mod16_len[w & 0xf]));
}
#endif
