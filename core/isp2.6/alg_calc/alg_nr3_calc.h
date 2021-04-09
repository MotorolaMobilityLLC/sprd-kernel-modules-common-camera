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

#ifndef _ALG_NR3_CALC_H
#define _ALG_NR3_CALC_H

#include "cam_types.h"

/*
 * Before N6pro all use ALG_NR3_MV_VER_0
 * N6pro use ALG_NR3_MV_VER_1.
*/
enum alg_nr3_mv_version {
	ALG_NR3_MV_VER_0,
	ALG_NR3_MV_VER_1,
	ALG_NR3_MV_VER_MAX,
};

/*
 * INPUT:
 *   mv_x: input mv_x (3DNR GME output)
 *   mv_y: input mv_y (3DNR GME output)
 *   mode_projection: mode projection in 3DNR GME
 *:   1 - interlaced mode, 0 - step mode
 *   sub_me_bypass:
 *   1 - sub pixel motion estimation bypass
 *   0 - do sub pixel motion estimation
 *   input_width:  input image width for binning and bayer scaler
 *   output_width: output image width for binning and bayer scaler
 *   input_height:  input image width for binning and bayer scaler
 *   output_height: output image width for binning and bayer scaler
 *
 * OUTPUT:
 *   out_mv_x
 *   out_mv_y
 */
struct alg_nr3_mv_cfg {
	/*    input param    */
	enum alg_nr3_mv_version mv_version;
	uint32_t mode_projection;
	uint32_t sub_me_bypass;
	int mv_x;
	int mv_y;
	int iw;
	int ow;
	int ih;
	int oh;

	/*    output param    */
	int o_mv_x;
	int o_mv_y;
};

void alg_nr3_calc_mv(struct alg_nr3_mv_cfg *param_ptr);

#endif
