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

#ifndef _ALG_SLICE_CALC_H
#define _ALG_SLICE_CALC_H

#include "isp_interface.h"

#define PIPE_MAX_SLICE_NUM 4
#define ALG_REGIONS_NUM 4
#define DEC_OVERLAP 4

struct alg_block_calc {
	uint32_t left;
	uint32_t right;
	uint32_t up;
	uint32_t down;
};

struct alg_overlap_info {
	uint32_t ov_left;
	uint32_t ov_right;
	uint32_t ov_up;
	uint32_t ov_down;
};

struct alg_region_info {
	uint32_t sx;
	uint32_t ex;
	uint32_t sy;
	uint32_t ey;
};

struct alg_slice_regions {
	uint32_t rows;
	uint32_t cols;
	struct alg_region_info regions[ALG_REGIONS_NUM];
};

struct alg_fetch_region {
	uint32_t image_w;
	uint32_t image_h;
	uint32_t slice_w;
	uint32_t slice_h;
	uint32_t overlap_left;
	uint32_t overlap_right;
	uint32_t overlap_up;
	uint32_t overlap_down;
};

struct alg_fetch_region_context {
	uint32_t s_row;
	uint32_t e_row;
	uint32_t s_col;
	uint32_t e_col;
	uint32_t overlap_left;
	uint32_t overlap_right;
	uint32_t overlap_up;
	uint32_t overlap_down;
};

/*
* img_type: 0:bayer 1:rgb 2:yuv444 3:yuv422 4:yuv420 5:yuv400 6:FBC bayer 7:FBC yuv420
*/
struct alg_dec_offline_overlap {
	/*    input param    */
	uint32_t img_w;
	uint32_t img_h;
	uint32_t img_type;
	uint32_t crop_en;
	uint32_t crop_mode;
	uint32_t crop_sx;
	uint32_t crop_sy;
	uint32_t crop_w;
	uint32_t crop_h;
	uint32_t slice_w;
	uint32_t slice_h;
	uint32_t dct_bypass;
	uint32_t dec_offline_bypass;
	uint32_t layerNum;
	uint32_t MaxSliceWidth;
	uint32_t slice_mode;

	/*    output param    */
	uint32_t slice_rows;
	uint32_t slice_cols;

	struct alg_region_info fecth_dec_region[ISP_PYR_DEC_LAYER_NUM][PIPE_MAX_SLICE_NUM];
	struct alg_overlap_info fecth_dec_overlap[ISP_PYR_DEC_LAYER_NUM][PIPE_MAX_SLICE_NUM];
	struct alg_region_info store_dec_region[MAX_PYR_DEC_LAYER_NUM][PIPE_MAX_SLICE_NUM];
	struct alg_overlap_info store_dec_overlap[MAX_PYR_DEC_LAYER_NUM][PIPE_MAX_SLICE_NUM];
};

void alg_slice_calc_dec_offline_overlap(struct alg_dec_offline_overlap *param_ptr);

#endif
