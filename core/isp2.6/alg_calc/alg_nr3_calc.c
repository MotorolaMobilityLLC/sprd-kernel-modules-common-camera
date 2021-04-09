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

#include "alg_nr3_calc.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ALG_NR3_CALC: %d %d %s : "fmt, current->pid, __LINE__, __func__

static void nr3_mv_convert_ver0(struct alg_nr3_mv_cfg *param_ptr)
{
	int mv_x = 0, mv_y = 0;
	uint32_t mode_projection = 0;
	uint32_t sub_me_bypass = 0;
	int iw = 0, ow = 0;
	int ih = 0, oh = 0;
	int o_mv_x = 0, o_mv_y = 0;

	mv_x = param_ptr->mv_x;
	mv_y = param_ptr->mv_y;
	mode_projection = param_ptr->mode_projection;
	sub_me_bypass = param_ptr->sub_me_bypass;
	iw = param_ptr->iw;
	ih = param_ptr->ih;
	ow = param_ptr->ow;
	oh = param_ptr->oh;

	if (sub_me_bypass == 1) {
		if (mode_projection == 1) {
			if (mv_x > 0)
				o_mv_x = (mv_x * 2 * ow + (iw >> 1)) / iw;
			else
				o_mv_x = (mv_x * 2 * ow - (iw >> 1)) / iw;

			if (mv_y > 0)
				o_mv_y = (mv_y * 2 * oh + (ih >> 1)) / ih;
			else
				o_mv_y = (mv_y * 2 * oh - (ih >> 1)) / ih;
		} else {
			if (mv_x > 0)
				o_mv_x = (mv_x * ow + (iw >> 1)) / iw;
			else
				o_mv_x = (mv_x * ow - (iw >> 1)) / iw;

			if (mv_y > 0)
				o_mv_y = (mv_y * oh + (ih >> 1)) / ih;
			else
				o_mv_y = (mv_y * oh - (ih >> 1)) / ih;
		}
	} else {
		if (mode_projection == 1) {
			if (mv_x > 0)
				o_mv_x = (mv_x * ow + (iw >> 1)) / iw;
			else
				o_mv_x = (mv_x * ow - (iw >> 1)) / iw;

			if (mv_y > 0)
				o_mv_y = (mv_y * oh + (ih >> 1)) / ih;
			else
				o_mv_y = (mv_y * oh - (ih >> 1)) / ih;
		} else {
			if (mv_x > 0)
				o_mv_x = (mv_x * ow + iw) / (iw * 2);
			else
				o_mv_x = (mv_x * ow - iw) / (iw * 2);

			if (mv_y > 0)
				o_mv_y = (mv_y * oh + ih) / (ih * 2);
			else
				o_mv_y = (mv_y * oh - ih) / (ih * 2);
		}
	}

	param_ptr->o_mv_x = o_mv_x;
	param_ptr->o_mv_y = o_mv_y;
}

void alg_nr3_calc_mv(struct alg_nr3_mv_cfg *param_ptr)
{
	if (!param_ptr) {
		pr_err("fail to get valid in ptr\n");
		return;
	}

	switch (param_ptr->mv_version) {
		case ALG_NR3_MV_VER_0:
			nr3_mv_convert_ver0(param_ptr);
			break;
		case ALG_NR3_MV_VER_1:
			/*TBD: Just for N6pro temp*/
			param_ptr->o_mv_x = 0;
			param_ptr->o_mv_y = 0;
			break;
		default:
			pr_err("fail to get invalid version %d\n", param_ptr->mv_version);
			break;
	}
}