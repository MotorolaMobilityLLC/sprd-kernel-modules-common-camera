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

#include "alg_slice_calc.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ALG_SLICE_CALC: %d %d %s : "fmt, current->pid, __LINE__, __func__

#define CAL_OVERLAP(ctrl_bypass, module_param)          \
    do                                                 \
{                                                      \
    if(!(ctrl_bypass))                                 \
{                                                      \
    ov_pipe.ov_left  += module_param.left;      \
    ov_pipe.ov_right += module_param.right;     \
    ov_pipe.ov_up    += module_param.up;        \
    ov_pipe.ov_down  += module_param.down;      \
}                                                      \
}                                                      \
    while(0)

static int check_image_resolution(uint32_t length, uint32_t layer_num)
{
	uint32_t outData = 0;
	uint32_t multiple = 1 << layer_num;

	outData = length % multiple;
	if (outData != 0)
		outData = multiple - outData;

	return outData;
}

static void core_drv_dnr_init_block(struct alg_block_calc *block_ptr)
{
	block_ptr->left = 4;
	block_ptr->right = 4;
	block_ptr->up = 4;
	block_ptr->down = 4;
}

static void core_drv_pyd_dec_offline_init_block(struct alg_block_calc *block_ptr)
{
	block_ptr->left = DEC_OVERLAP;
	block_ptr->right = 0;
	block_ptr->up = DEC_OVERLAP;
	block_ptr->down = 0;
}

void isp_drv_region_w_align(struct alg_region_info *r,
		uint32_t align_v, uint32_t min_v, uint32_t max_v)
{
	uint32_t slice_w_org, slice_w_dst, offset_v, sx, ex;

	if (align_v <= 0)
		return;

	slice_w_org = r->ex - r->sx + 1;
	if (slice_w_org % align_v != 0) {
		slice_w_dst = (slice_w_org + (align_v - 1)) / align_v * align_v;
		offset_v = slice_w_dst - slice_w_org;
		sx = r->sx - offset_v;
		if (sx >= min_v) {
			r->sx = sx;
		} else {
			ex = r->ex + offset_v;
			if (ex <= max_v)
				r->ex = ex;
		}
	}
}

uint32_t isp_drv_regions_fetch_ref(const struct alg_fetch_region *fetch_param,
	const struct alg_slice_regions *r_ref, struct alg_slice_regions *r_out)
{
	uint32_t imgW = fetch_param->image_w;
	uint32_t imgH = fetch_param->image_h;
	uint32_t overlapUp = fetch_param->overlap_up;
	uint32_t overlapDown = fetch_param->overlap_down;
	uint32_t overlapLeft = fetch_param->overlap_left;
	uint32_t overlapRight= fetch_param->overlap_right;
	uint32_t row_num = r_ref->rows;
	uint32_t col_num = r_ref->cols;
	uint32_t i ,j, index;
	struct alg_fetch_region_context context;

	r_out->rows = row_num;
	r_out->cols = col_num;
	for (i = 0;i < row_num; i++) {
		for (j = 0; j < col_num; j++) {
			index = i * col_num + j;
			context.s_row = r_ref->regions[index].sy;
			context.s_col = r_ref->regions[index].sx;
			context.e_row = r_ref->regions[index].ey;
			context.e_col = r_ref->regions[index].ex;
			context.overlap_left = overlapLeft;
			context.overlap_right = overlapRight;
			context.overlap_up = overlapUp;
			context.overlap_down = overlapDown;

			/* l-top */
			if ((0 == i) && (0 == j)) {
				context.overlap_left = 0;
				context.overlap_up = 0;
				context.s_row = 0;
				context.s_col = 0;
			}
			/* r-top */
			if ((0 == i) && (col_num-1 == j)) {
				context.overlap_right = 0;
				context.overlap_up = 0;
				context.s_row = 0;
				context.e_col = (imgW -1);
			}
			/* l-bottom */
			if ((row_num-1 == i) && (0 == j)) {
				context.overlap_left = 0;
				context.overlap_down = 0;
				context.s_col= 0;
				context.e_row= (imgH - 1);
			}
			/* r-bottom */
			if ((row_num-1 == i) && (col_num-1 == j)) {
				context.overlap_right= 0;
				context.overlap_down = 0;
				context.e_row = (imgH -1);
				context.e_col = (imgW -1);
			}
			/* up */
			if ((0 == i) && (0<j && j<col_num-1)) {
				context.overlap_up = 0;
				context.s_row = 0;
			}
			/* down */
			if ((row_num-1 == i) && (0<j && j<col_num-1)) {
				context.overlap_down = 0;
				context.e_row = (imgH - 1);
			}
			/* left */
			if ((0 == j) && (0<i && i<row_num-1)) {
				context.overlap_left = 0;
				context.s_col = 0;
			}
			/* right */
			if ((col_num-1 == j) && (0<i && i<row_num-1)) {
				context.overlap_right = 0;
				context.e_col = (imgW - 1);
			}

			context.s_row -= context.overlap_up;
			context.e_row += context.overlap_down;
			context.s_col -= context.overlap_left;
			context.e_col += context.overlap_right;

			/* add overlap overflow return -1 */
			if (context.s_col < 0 || context.s_row < 0 ||
				context.e_col >= imgW || context.e_row >= imgH)
				return -1;

			r_out->regions[index].sx = context.s_col;
			r_out->regions[index].ex = context.e_col;
			r_out->regions[index].sy = context.s_row;
			r_out->regions[index].ey = context.e_row;
		}
	}

	return 0;
}

uint32_t isp_drv_regions_fetch(const struct alg_fetch_region *fetch_param,
		struct alg_slice_regions *r_out)
{
	uint32_t imgW = fetch_param->image_w;
	uint32_t imgH = fetch_param->image_h;
	uint32_t sliceW = fetch_param->slice_w;
	uint32_t sliceH = fetch_param->slice_h;
	uint32_t col_num, row_num;
	uint32_t i, j, index, ret_val;
	struct alg_slice_regions region_temp;
	struct alg_fetch_region_context context;

	if (sliceW <= 0)
		sliceW = imgW;
	if(sliceH <= 0)
		sliceH = imgH;
	col_num = imgW / sliceW + (imgW % sliceW ? 1:0);
	row_num = imgH / sliceH + (imgH % sliceH ? 1:0);

	if (col_num * row_num > ALG_REGIONS_NUM) {
		pr_err("fail to get invalid param %d %d\n", col_num, row_num);
		return -EFAULT;
	}

	region_temp.rows = row_num;
	region_temp.cols = col_num;

	pr_debug("row_num %d col_num %d imgw %d slicew %d\n", row_num, col_num, imgW, sliceW);
	for (i = 0;i < row_num; i++) {
		for (j = 0; j < col_num; j++) {
			index = i * col_num + j;
			context.s_row = i * sliceH;
			context.s_col = j * sliceW;
			context.e_row = context.s_row + sliceH - 1;
			context.e_col = context.s_col + sliceW - 1;

			region_temp.regions[index].sx = context.s_col;
			region_temp.regions[index].ex = context.e_col;
			region_temp.regions[index].sy = context.s_row;
			region_temp.regions[index].ey = context.e_row;
		}
	}

	ret_val = isp_drv_regions_fetch_ref(fetch_param,&region_temp,r_out);

	return ret_val;
}

void isp_drv_regions_w_align(struct alg_slice_regions *r, int align_v, int min_v, int max_v)
{
	uint32_t i, j, index;
	uint32_t rows = r->rows;
	uint32_t cols = r->cols;

	for (i = 0; i < rows; i++) {
		for(j = 0; j < cols; j++) {
			index = i * cols + j;
			isp_drv_region_w_align(&r->regions[index], align_v, min_v, max_v);
		}
	}
}

void alg_slice_calc_dec_offline_overlap(struct alg_dec_offline_overlap *param_ptr)
{
	const uint32_t SLICE_W_ALIGN_V = 4;
	uint32_t layer_id = 0;
	uint32_t image_w = param_ptr->img_w;
	uint32_t image_h = param_ptr->img_h;
	uint32_t src_width = image_w;
	uint32_t src_height = image_h;
	uint32_t layer0_padding_w = 0, layer0_padding_h = 0;
	uint32_t dw = 0, dh = 0;
	uint32_t layer_slice_width[ISP_PYR_DEC_LAYER_NUM] = {0};
	uint32_t layer_slice_height[ISP_PYR_DEC_LAYER_NUM] = {0};
	uint32_t cur_layer_width[MAX_PYR_DEC_LAYER_NUM] = {0};
	uint32_t cur_layer_height[MAX_PYR_DEC_LAYER_NUM] = {0};
	struct alg_overlap_info ov_pipe = {0};
	struct alg_block_calc dct_param = {0}, dec_offline_param = {0};
	struct alg_slice_regions org_dec_slice_out[ISP_PYR_DEC_LAYER_NUM];
	struct alg_fetch_region dec_slice_in = {0}, org_dec_slice_in = {0};
	struct alg_slice_regions dec_slice_out[ISP_PYR_DEC_LAYER_NUM];
	uint32_t slice_flag[MAX_PYR_DEC_LAYER_NUM] = {0};

	for (layer_id = 0; layer_id < ISP_PYR_DEC_LAYER_NUM; layer_id++) {
		memset(&org_dec_slice_out[layer_id], 0, sizeof(struct alg_slice_regions));
		memset(&dec_slice_out[layer_id], 0, sizeof(struct alg_slice_regions));
	}

	if (param_ptr->crop_en && 0 == param_ptr->crop_mode) {
		image_w = param_ptr->crop_w;
		image_h = param_ptr->crop_h;
		src_width = image_w;
		src_height = image_h;
	}

	/* width need 4 align & high need 2 align */
	dw = check_image_resolution(image_w, param_ptr->layerNum + 1);
	dh = check_image_resolution(image_h, param_ptr->layerNum);
	layer0_padding_w = src_width + dw;
	layer0_padding_h = src_height + dh;

{
	uint32_t slice_width_org, slice_height_org, layer0_slice_num, after_padding_slice_num;
	uint32_t slice_align_shift = 3;
	for (layer_id = 0; layer_id < param_ptr->layerNum - 1; layer_id++) {
		if (!param_ptr->dec_offline_bypass) {
			slice_width_org = (param_ptr->slice_w >> layer_id);
			slice_height_org = (param_ptr->slice_h >> layer_id);
			layer0_slice_num = (src_width + param_ptr->slice_w - 1) / param_ptr->slice_w;
			after_padding_slice_num = (layer0_padding_w + param_ptr->slice_w - 1) / param_ptr->slice_w;
			if (layer0_slice_num < after_padding_slice_num) {
				layer_slice_width[layer_id] = ((layer_id ? layer0_padding_w : (src_width))
					* slice_width_org + src_width - 1 ) / src_width;
				layer_slice_height[layer_id] = ((layer_id ? layer0_padding_h : (src_height))
					* slice_height_org + src_height - 1) / src_height;
			} else {
				layer_slice_width[layer_id] = slice_width_org;
				layer_slice_height[layer_id] = slice_height_org;
			}

			layer_slice_width[layer_id] = ((layer_slice_width[layer_id] + (1 << slice_align_shift) - 1)
				>> slice_align_shift) << slice_align_shift;
			layer_slice_height[layer_id] = ((layer_slice_height[layer_id] + (1 << slice_align_shift) - 1)
				>> slice_align_shift) << slice_align_shift;
		} else {
			layer_slice_width[layer_id] = param_ptr->slice_w;
			layer_slice_height[layer_id] = param_ptr->slice_h;
		}
	}

	if (param_ptr->layerNum == 1 && param_ptr->dec_offline_bypass) {
		layer_slice_width[0] = param_ptr->slice_w;
		layer_slice_height[0] = param_ptr->slice_h;
	}

	/* if image width is too small, no slice */
	if(param_ptr->slice_mode) {
		for (layer_id = 0; layer_id < param_ptr->layerNum - 1; layer_id++) {
			const uint32_t max_slice_w = param_ptr->MaxSliceWidth;
			if (layer_id == 0) {
				cur_layer_width[layer_id] = src_width;
				cur_layer_height[layer_id] = src_height;
			} else {
				cur_layer_width[layer_id] = layer0_padding_w >> layer_id;
				cur_layer_height[layer_id] = layer0_padding_h >> layer_id;
			}

			if (layer_id == 0 && layer0_padding_w <= max_slice_w) {
				slice_flag[layer_id] = 1;
			} else if (cur_layer_width[layer_id] <= max_slice_w) {
				slice_flag[layer_id] = 1;
			}
		}

		if (param_ptr->layerNum == 1 && param_ptr->dec_offline_bypass) {
			const uint32_t max_slice_w = param_ptr->MaxSliceWidth;
			cur_layer_width[0] = src_width;
			cur_layer_height[0] = src_height;
			if (cur_layer_width[0] <= max_slice_w)
				slice_flag[0] = 1;
		}
	}
}

	/* input */
	core_drv_dnr_init_block(&dct_param);
	CAL_OVERLAP(param_ptr->dct_bypass, dct_param);
	core_drv_pyd_dec_offline_init_block(&dec_offline_param);
	CAL_OVERLAP(param_ptr->dec_offline_bypass, dec_offline_param);

	/* overlap 4 align */
	if (!param_ptr->dec_offline_bypass) {
		ov_pipe.ov_left = (ov_pipe.ov_left + 7) >> 3 << 3;
		ov_pipe.ov_right = (ov_pipe.ov_right + 7) >> 3 << 3;
		ov_pipe.ov_up = (ov_pipe.ov_up + 7) >> 3 << 3;
		ov_pipe.ov_down = (ov_pipe.ov_down + 7) >> 3 << 3;
	} else {
		ov_pipe.ov_left = (ov_pipe.ov_left + 1) >> 1 << 1;
		ov_pipe.ov_right = (ov_pipe.ov_right + 1) >> 1 << 1;
		ov_pipe.ov_up = (ov_pipe.ov_up + 1) >> 1 << 1;
		ov_pipe.ov_down = (ov_pipe.ov_down + 1) >> 1 << 1;
	}

	/* slice cfg */
	for (layer_id = 0; layer_id < param_ptr->layerNum - 1; layer_id++) {
		org_dec_slice_in.image_w = layer_id ? (layer0_padding_w >> layer_id) : (src_width);
		org_dec_slice_in.image_h = layer_id ? (layer0_padding_h >>layer_id) : (src_height);
		if (param_ptr->slice_mode && slice_flag[layer_id]) {
			org_dec_slice_in.slice_w = cur_layer_width[layer_id];
			org_dec_slice_in.slice_h = cur_layer_height[layer_id];
		} else {
			org_dec_slice_in.slice_w = layer_slice_width[layer_id];
			org_dec_slice_in.slice_h = layer_slice_height[layer_id];
		}
		org_dec_slice_in.overlap_left = 0;
		org_dec_slice_in.overlap_right = 0;
		org_dec_slice_in.overlap_up = 0;
		org_dec_slice_in.overlap_down = 0;

		/* get org slice region */
		if (-1 == isp_drv_regions_fetch(&org_dec_slice_in, &org_dec_slice_out[layer_id]))
			return;
	}

	if (param_ptr->layerNum == 1 && param_ptr->dec_offline_bypass) {
		org_dec_slice_in.image_w = src_width;
		org_dec_slice_in.image_h = src_height;
		if(param_ptr->slice_mode && slice_flag[0]) {
			org_dec_slice_in.slice_w = cur_layer_width[0];
			org_dec_slice_in.slice_h = cur_layer_height[0];
		} else {
			org_dec_slice_in.slice_w = layer_slice_width[0];
			org_dec_slice_in.slice_h = layer_slice_height[0];
		}
		org_dec_slice_in.overlap_left = 0;
		org_dec_slice_in.overlap_right = 0;
		org_dec_slice_in.overlap_up = 0;
		org_dec_slice_in.overlap_down = 0;

		/* get org slice region */
		if (-1 == isp_drv_regions_fetch(&org_dec_slice_in, &org_dec_slice_out[0]))
			return;
	}

	for (layer_id = 0; layer_id < param_ptr->layerNum - 1; layer_id++) {
		dec_slice_in.image_w = layer_id ? (layer0_padding_w >> layer_id) : (src_width);
		dec_slice_in.image_h = layer_id ? (layer0_padding_h >> layer_id) : (src_height);
		if (param_ptr->slice_mode && slice_flag[layer_id]) {
			dec_slice_in.slice_w = cur_layer_width[layer_id];
			dec_slice_in.slice_h = cur_layer_height[layer_id];
		} else {
			dec_slice_in.slice_w = layer_slice_width[layer_id];
			dec_slice_in.slice_h = layer_slice_height[layer_id];
		}
		dec_slice_in.overlap_left = ov_pipe.ov_left;
		dec_slice_in.overlap_right = ov_pipe.ov_right;
		dec_slice_in.overlap_up = ov_pipe.ov_up;
		dec_slice_in.overlap_down = ov_pipe.ov_down;

		/* get out slice region */
		if (-1 == isp_drv_regions_fetch(&dec_slice_in, &dec_slice_out[layer_id]))
			return;

		/* dec_offline slice 4 align */
		isp_drv_regions_w_align(&dec_slice_out[layer_id], SLICE_W_ALIGN_V, 0, dec_slice_in.image_w);
	}

	if (param_ptr->layerNum == 1 && param_ptr->dec_offline_bypass) {
		dec_slice_in.image_w = src_width;
		dec_slice_in.image_h = src_height;
		if (param_ptr->slice_mode && slice_flag[0]) {
			dec_slice_in.slice_w  = cur_layer_width[0];
			dec_slice_in.slice_h = cur_layer_height[0];
		} else {
			dec_slice_in.slice_w = layer_slice_width[0];
			dec_slice_in.slice_h = layer_slice_height[0];
		}
		dec_slice_in.overlap_left = ov_pipe.ov_left;
		dec_slice_in.overlap_right = ov_pipe.ov_right;
		dec_slice_in.overlap_up = ov_pipe.ov_up;
		dec_slice_in.overlap_down = ov_pipe.ov_down;

		/* get out slice region */
		if (-1 == isp_drv_regions_fetch(&dec_slice_in, &dec_slice_out[0]))
			return;

		/* dec_offline slice 4 align */
		isp_drv_regions_w_align(&dec_slice_out[0], SLICE_W_ALIGN_V, 0, dec_slice_in.image_w);
	}

{
	uint32_t i, j, index, rows, cols;
	/* fecth_dec_slice */
	for (layer_id = 0; layer_id < param_ptr->layerNum - 1; layer_id++) {
		rows = dec_slice_out[layer_id].rows;
		cols = dec_slice_out[layer_id].cols;
		for (i = 0; i < rows; i++) {
			for (j=0; j < cols; j++) {
				index = i * cols + j;
				param_ptr->fecth_dec_region[layer_id][index].sx = dec_slice_out[layer_id].regions[index].sx;
				param_ptr->fecth_dec_region[layer_id][index].ex = dec_slice_out[layer_id].regions[index].ex;
				param_ptr->fecth_dec_region[layer_id][index].sy = dec_slice_out[layer_id].regions[index].sy;
				param_ptr->fecth_dec_region[layer_id][index].ey = dec_slice_out[layer_id].regions[index].ey;
				param_ptr->fecth_dec_overlap[layer_id][index].ov_left = org_dec_slice_out[layer_id].regions[index].sx - dec_slice_out[layer_id].regions[index].sx;
				param_ptr->fecth_dec_overlap[layer_id][index].ov_right = dec_slice_out[layer_id].regions[index].ex - org_dec_slice_out[layer_id].regions[index].ex;
				param_ptr->fecth_dec_overlap[layer_id][index].ov_up = org_dec_slice_out[layer_id].regions[index].sy - dec_slice_out[layer_id].regions[index].sy;
				param_ptr->fecth_dec_overlap[layer_id][index].ov_down = dec_slice_out[layer_id].regions[index].ey - org_dec_slice_out[layer_id].regions[index].ey;
				pr_debug("layer %d fetch ov_r %d\n", layer_id, param_ptr->fecth_dec_overlap[layer_id][index].ov_right);
			}
		}
	}

	if (param_ptr->layerNum == 1 && param_ptr->dec_offline_bypass) {
		rows = dec_slice_out[0].rows;
		cols = dec_slice_out[0].cols;
		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				index = i * cols + j;
				param_ptr->fecth_dec_region[0][index].sx = dec_slice_out[0].regions[index].sx;
				param_ptr->fecth_dec_region[0][index].ex = dec_slice_out[0].regions[index].ex;
				param_ptr->fecth_dec_region[0][index].sy = dec_slice_out[0].regions[index].sy;
				param_ptr->fecth_dec_region[0][index].ey = dec_slice_out[0].regions[index].ey;
				param_ptr->fecth_dec_overlap[0][index].ov_left = org_dec_slice_out[0].regions[index].sx - dec_slice_out[0].regions[index].sx;
				param_ptr->fecth_dec_overlap[0][index].ov_right = dec_slice_out[0].regions[index].ex - org_dec_slice_out[0].regions[index].ex;
				param_ptr->fecth_dec_overlap[0][index].ov_up = org_dec_slice_out[0].regions[index].sy - dec_slice_out[0].regions[index].sy;
				param_ptr->fecth_dec_overlap[0][index].ov_down = dec_slice_out[0].regions[index].ey - org_dec_slice_out[0].regions[index].ey;
			}
		}
	}

	/* store_dec_slice */
	for (layer_id = 0; layer_id < param_ptr->layerNum; layer_id++) {
		if (layer_id == 0) {
			rows = dec_slice_out[0].rows;
			cols = dec_slice_out[0].cols;
		} else {
			rows = dec_slice_out[layer_id - 1].rows;
			cols = dec_slice_out[layer_id - 1].cols;
		}

		for (i = 0; i < rows; i++) {
			for (j = 0; j < cols; j++) {
				index = i * cols + j;
				if (layer_id == 0) {
					/* store_dct, no overlap, before padding */
					param_ptr->store_dec_overlap[layer_id][index].ov_left = param_ptr->fecth_dec_overlap[layer_id][index].ov_left;
					param_ptr->store_dec_overlap[layer_id][index].ov_right = param_ptr->fecth_dec_overlap[layer_id][index].ov_right;
					param_ptr->store_dec_overlap[layer_id][index].ov_up = param_ptr->fecth_dec_overlap[layer_id][index].ov_up;
					param_ptr->store_dec_overlap[layer_id][index].ov_down = param_ptr->fecth_dec_overlap[layer_id][index].ov_down;
					param_ptr->store_dec_region[layer_id][index].sx = param_ptr->fecth_dec_region[layer_id][index].sx + param_ptr->fecth_dec_overlap[layer_id][index].ov_left;
					param_ptr->store_dec_region[layer_id][index].ex = param_ptr->fecth_dec_region[layer_id][index].ex - param_ptr->fecth_dec_overlap[layer_id][index].ov_right;
					param_ptr->store_dec_region[layer_id][index].sy = param_ptr->fecth_dec_region[layer_id][index].sy + param_ptr->fecth_dec_overlap[layer_id][index].ov_up;
					param_ptr->store_dec_region[layer_id][index].ey = param_ptr->fecth_dec_region[layer_id][index].ey - param_ptr->fecth_dec_overlap[layer_id][index].ov_down;
				} else if (layer_id == 1) {
					param_ptr->store_dec_overlap[layer_id][index].ov_left = param_ptr->fecth_dec_overlap[0][index].ov_left / 2 ;
					param_ptr->store_dec_overlap[layer_id][index].ov_right = param_ptr->fecth_dec_overlap[0][index].ov_right / 2;
					param_ptr->store_dec_overlap[layer_id][index].ov_up = param_ptr->fecth_dec_overlap[0][index].ov_up / 2;
					param_ptr->store_dec_overlap[layer_id][index].ov_down = param_ptr->fecth_dec_overlap[0][index].ov_down / 2 ;
					if(j < cols - 1) {
						param_ptr->store_dec_region[layer_id][index].sx = (param_ptr->fecth_dec_region[0][index].sx + param_ptr->fecth_dec_overlap[0][index].ov_left ) / 2;
						param_ptr->store_dec_region[layer_id][index].sy = (param_ptr->fecth_dec_region[0][index].sy + param_ptr->fecth_dec_overlap[0][index].ov_up) / 2;
						param_ptr->store_dec_region[layer_id][index].ex = (param_ptr->fecth_dec_region[0][index].ex + 1 - param_ptr->fecth_dec_overlap[0][index].ov_right) / 2 - 1;
						param_ptr->store_dec_region[layer_id][index].ey = (param_ptr->fecth_dec_region[0][index].ey +dh + 1 - param_ptr->fecth_dec_overlap[0][index].ov_down ) / 2 - 1;
					} else if (j == cols - 1) {
						param_ptr->store_dec_region[layer_id][index].sx = (param_ptr->fecth_dec_region[0][index].sx + param_ptr->fecth_dec_overlap[0][index].ov_left) /2;
						param_ptr->store_dec_region[layer_id][index].sy = (param_ptr->fecth_dec_region[0][index].sy + param_ptr->fecth_dec_overlap[0][index].ov_up) / 2;
						param_ptr->store_dec_region[layer_id][index].ex = (param_ptr->fecth_dec_region[0][index].ex + dw + 1 - param_ptr->fecth_dec_overlap[0][index].ov_right) / 2 - 1;
						param_ptr->store_dec_region[layer_id][index].ey = (param_ptr->fecth_dec_region[0][index].ey + dh + 1 - param_ptr->fecth_dec_overlap[0][index].ov_down ) / 2 - 1;
					}
				} else {
					param_ptr->store_dec_overlap[layer_id][index].ov_left  = param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_left / 2 ;
					param_ptr->store_dec_overlap[layer_id][index].ov_right = param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_right / 2;
					param_ptr->store_dec_overlap[layer_id][index].ov_up = param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_up / 2;
					param_ptr->store_dec_overlap[layer_id][index].ov_down  = param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_down / 2;
					param_ptr->store_dec_region[layer_id][index].sx = (param_ptr->fecth_dec_region[layer_id - 1][index].sx + param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_left) / 2;
					param_ptr->store_dec_region[layer_id][index].sy = (param_ptr->fecth_dec_region[layer_id - 1][index].sy + param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_up) / 2;
					param_ptr->store_dec_region[layer_id][index].ex = (param_ptr->fecth_dec_region[layer_id - 1][index].ex + 1 - param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_right) / 2 - 1;
					param_ptr->store_dec_region[layer_id][index].ey = (param_ptr->fecth_dec_region[layer_id - 1][index].ey + 1 - param_ptr->fecth_dec_overlap[layer_id - 1][index].ov_down) / 2 - 1;
				}
			}
		}
	}
}
}
