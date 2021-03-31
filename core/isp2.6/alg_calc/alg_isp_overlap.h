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

#ifndef _ALG_ISP_OVERLAP_H_
#define _ALG_ISP_OVERLAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cam_types.h"

#define SCL_UP_MAX                      10
#define SCL_DOWN_MAX                    10
#define YUV422                          0
#define YUV420                          1
#define PIPE_MAX_SLICE_NUM              4
#define PHASE_1                         32
#define ISP_DRV_REGIONS_NUM             16
#define FBC_PADDING_W_BAYER             128
#define FBC_PADDING_H_BAYER             4
#define FBC_PADDING_W_YUV420_scaler     32
#define FBC_PADDING_H_YUV420_scaler     8
#define FBC_PADDING_W_YUV420_3dnr       256
#define FBC_PADDING_H_YUV420_3dnr       4

enum alg_isp_overlap_version {
	ALG_ISP_DYN_OVERLAP_NONE,
	ALG_ISP_OVERLAP_VER_1, /*qogirl6*/
	ALG_ISP_OVERLAP_VER_MAX,
};

typedef struct _slice_pos {
	uint32_t start_col;
	uint32_t start_row;
	uint32_t end_col;
	uint32_t end_row;
} isp_fw_slice_pos;

typedef struct _slice_overlap {
	uint32_t overlap_up;
	uint32_t overlap_down;
	uint32_t overlap_left;
	uint32_t overlap_right;
} isp_fw_slice_overlap;

typedef struct _scaler_slice {
	uint32_t bypass;
	uint32_t trim0_size_x;
	uint32_t trim0_size_y;
	uint32_t trim0_start_x;
	uint32_t trim0_start_y;
	uint32_t trim1_size_x;
	uint32_t trim1_size_y;
	uint32_t trim1_start_x;
	uint32_t trim1_start_y;
	uint32_t scaler_ip_int;
	uint32_t scaler_ip_rmd;
	uint32_t scaler_cip_int;
	uint32_t scaler_cip_rmd;
	uint32_t scaler_factor_in;
	uint32_t scaler_factor_out;
	uint32_t scaler_ip_int_ver;
	uint32_t scaler_ip_rmd_ver;
	uint32_t scaler_cip_int_ver;
	uint32_t scaler_cip_rmd_ver;
	uint32_t scaler_factor_in_ver;
	uint32_t scaler_factor_out_ver;
	uint32_t src_size_x;
	uint32_t src_size_y;
	uint32_t dst_size_x;
	uint32_t dst_size_y;
	uint32_t chk_sum_clr;
} isp_fw_scaler_slice;

enum SCALER_ID {
	SCALER_CAP_PRE,
	SCALER_VID,
	SCALER_NUM,
};

typedef struct
{
	int slice_id;
	int slice_width;
	int slice_height;

	int sliceRows;
	int sliceCols;
	int sliceRowNo;
	int sliceColNo;

	int start_col;
	int start_row;
	int end_col;
	int end_row;

	int overlap_left;
	int overlap_right;
	int overlap_up;
	int overlap_down;

	int init_phase_hor;
	int init_phase_ver;
} scaler_slice_t;

typedef struct _tag_ltm_rgb_stat_param_t
{
	uint8_t bypass;

	uint8_t strength;
	uint8_t ch_G_Y;
	uint8_t region_est_en;
	uint8_t text_point_thres;
	uint8_t text_proportion;
	uint8_t tile_num_auto;
	uint8_t tile_num_row;
	uint8_t tile_num_col;

	uint8_t binning_en;
	uint8_t cropUp_stat;
	uint8_t cropDown_stat;
	uint8_t cropLeft_stat;
	uint8_t cropRight_stat;

	uint16_t clipLimit;
	uint16_t clipLimit_min;
	uint16_t tile_width_stat;
	uint16_t tile_height_stat;
	uint16_t frame_width_stat;
	uint16_t frame_height_stat;
	uint16_t slice_width_stat;
	uint16_t slice_height_stat;
	uint32_t tile_size_stat;

	uint32_t ***tileHists;
	uint32_t ***tileHists_temp;
	uint8_t  ***flat_bin_flag;
	uint8_t *texture_thres_table;

	uint8_t frame_id;
	int in_bitwidth;
	int out_bitwidth;
	uint8_t frame_flag;
	char *stat_file_out;
	char *tile_file_out;
} ltm_rgb_stat_param_t;

enum STORE_OUTPUT_FORMAT
{
	YUV_OUT_UYVY_422,
	YUV_OUT_Y_UV_422,
	YUV_OUT_Y_VU_422,
	YUV_OUT_Y_U_V_422,
	YUV_OUT_Y_UV_420,
	YUV_OUT_Y_VU_420,
	YUV_OUT_Y_U_V_420,
	YUV_OUT_Y_400 = 7,
	NORMAL_RAW10 = 7,
	FULL_RGB8,
};

typedef struct _tag_pipe_overlap_info
{
	int ov_left;
	int ov_right;
	int ov_up;
	int ov_down;
}pipe_overlap_info;

typedef struct _tag_slice_drv_scaler_slice_init_context
{
	int slice_index;
	int rows;
	int cols;
	int slice_row_no;
	int slice_col_no;
	int slice_w;
	int slice_h;
} slice_drv_scaler_slice_init_context;

typedef struct _tagSliceWnd
{
	int s_row;
	int e_row;
	int s_col;
	int e_col;
	int overlap_left;
	int overlap_right;
	int overlap_up;
	int overlap_down;
} SliceWnd;

typedef struct _tag_isp_drv_region_fetch_context
{
	int s_row;
	int e_row;
	int s_col;
	int e_col;
	int overlap_left;
	int overlap_right;
	int overlap_up;
	int overlap_down;
} isp_drv_region_fetch_context;

typedef struct _tag_isp_drv_region_fetch_t
{
	int image_w;
	int image_h;
	int slice_w;
	int slice_h;
	int overlap_left;
	int overlap_right;
	int overlap_up;
	int overlap_down;
} isp_drv_region_fetch_t;

typedef struct _tag_isp_block_drv_t
{
	int left;
	int right;
	int up;
	int down;
} isp_block_drv_t;

typedef struct _tag_isp_drv_region_t
{
	int sx;
	int ex;
	int sy;
	int ey;
} isp_drv_region_t;

typedef struct _tag_isp_drv_regions_t
{
	int rows;
	int cols;
	isp_drv_region_t regions[ISP_DRV_REGIONS_NUM];
} isp_drv_regions_t;

typedef struct _tag_pipe_overlap_context
{
	int frameWidth;
	int frameHeight;
	int pixelFormat;
}pipe_overlap_context;

typedef struct _tag_slice_drv_overlap_info
{
	int ov_left;
	int ov_right;
	int ov_up;
	int ov_down;
}slice_drv_overlap_info;

typedef struct
{
	int32_t   scaler_init_phase[2]     ;
	int16_t   scaler_init_phase_int[2][2] ; /*[hor/ver][luma/chroma]*/
	uint16_t  scaler_init_phase_rmd[2][2] ;
} scaler_phase_info_t;

typedef struct
{
	int16_t     y_hor_coef[PHASE_1][8]; /* Luma horizontal coefficients table */
	int16_t     c_hor_coef[PHASE_1][8]; /* Chroma horizontal coefficients table */
	int16_t     y_ver_coef[PHASE_1][16]; /* Luma vertical down coefficients table */
	int16_t     c_ver_coef[PHASE_1][16]; /* Chroma veritical down coefficients table */
} scaler_coef_info_t;

typedef struct
{
	uint8_t   scaler_en; /*0: disable; 1:enable*/

	uint8_t   input_pixfmt; /*input yuv format: 0=yuv422 or 1=yuv420;*/
	uint8_t   output_pixfmt;

	uint16_t  scaler_in_width;
	uint16_t  scaler_in_height;
	uint16_t  scaler_out_width;
	uint16_t  scaler_out_height;

	uint16_t  scaler_factor_in_hor;
	uint16_t  scaler_factor_out_hor;
	uint16_t  scaler_factor_in_ver;
	uint16_t  scaler_factor_out_ver;

	int32_t   scaler_init_phase_hor;
	int32_t   scaler_init_phase_ver;

	uint8_t   scaler_y_hor_tap;
	uint8_t   scaler_y_ver_tap; /*Y Vertical tap of scaling*/
	uint8_t   scaler_uv_hor_tap;
	uint8_t   scaler_uv_ver_tap;

	scaler_phase_info_t init_phase_info;
	scaler_coef_info_t  scaler_coef_info;
} scaler_info_t;

typedef struct
{
	uint8_t   trim_en;
	uint16_t  trim_start_x;
	uint16_t  trim_start_y;
	uint16_t  trim_size_x;
	uint16_t  trim_size_y;
} trim_info_t;

typedef struct
{
	uint8_t   deci_x_en; /*0: disable; 1:enable*/
	uint8_t   deci_y_en; /* 0: disable; 1:enable*/
	uint8_t   deci_x; /*deci factor:1,2,4,8,16*/
	uint8_t   deci_y; /*deci factor:1,2,4,8,16*/
	uint8_t   deciPhase_X; /*deci phase:0,1,3,7*/
	uint8_t   deciPhase_Y; /*deci phase:0,1,3,7*/
	uint8_t   deci_cut_first_y;
	uint8_t   deci_option; /*0:direct deci; 1:average deci; 2:only average for luma*/
} deci_info_t;

typedef struct _tag_slice_drv_scaler_phase_info
{
	int init_phase_hor;
	int init_phase_ver;
} slice_drv_scaler_phase_info;

typedef struct _tag_slice_drv_region_info
{
	int sx;
	int ex;
	int sy;
	int ey;
}slice_drv_region_info;

typedef struct
{
	uint8_t           bypass;

	uint8_t           input_pixfmt; /*00:YUV422; 1:YUV420*/
	uint8_t           output_pixfmt;  /*00:YUV422; 1:YUV420*/
	uint8_t           output_align_hor;
	uint8_t           output_align_ver;

	uint16_t          src_size_x;
	uint16_t          src_size_y;
	uint16_t          dst_start_x;
	uint16_t          dst_start_y;
	uint16_t          dst_size_x;
	uint16_t          dst_size_y;

	trim_info_t     trim0_info;
	deci_info_t     deci_info;
	scaler_info_t   scaler_info;
	trim_info_t     trim1_info;
} yuvscaler_param_t;

typedef struct _tag_slice_drv_overlap_scaler_param
{
	/*in*/
	int bypass;
	int trim_eb;
	int trim_start_x;
	int trim_start_y;
	int trim_size_x;
	int trim_size_y;
	int deci_x_eb;
	int deci_y_eb;
	int deci_x;
	int deci_y;
	int scaler_en;
	int32_t scl_init_phase_hor;
	int32_t scl_init_phase_ver;
	int des_size_x;
	int des_size_y;
	int yuv_output_format;
	int FBC_enable;
	int output_align_hor;

	/*out*/
	slice_drv_scaler_phase_info phase[PIPE_MAX_SLICE_NUM];
	slice_drv_region_info region_input[PIPE_MAX_SLICE_NUM];
	slice_drv_region_info region_output[PIPE_MAX_SLICE_NUM];
	yuvscaler_param_t *frameParam;
	yuvscaler_param_t frameParamObj;
	yuvscaler_param_t sliceParam[PIPE_MAX_SLICE_NUM];
}slice_drv_overlap_scaler_param;

typedef struct _tag_slice_drv_overlap_param_t
{
	/************************************************************************/
	/* img_type:                                                            */
	/* 0:bayer 1:rgb 2:yuv444 3:yuv422 4:yuv420 5:yuv400                    */
	/* 6:FBC bayer 7:FBC yuv420                                             */
	/************************************************************************/
	int img_w;
	int img_h;
	int img_type;

	/************************************************************************/
	/* 如果有crop行为，则输入pipeline的图像大小为crop_w,crop_h              */
	/************************************************************************/
	int crop_en;
	int crop_mode;
	int crop_sx;
	int crop_sy;
	int crop_w;
	int crop_h;

	/************************************************************************/
	/* on whaleK slice_h >= img_h or slice_h >= crop_h                      */
	/************************************************************************/
	int slice_w;
	int slice_h;
	/************************************************************************/
	/* user define overlap                                                  */
	/************************************************************************/
	int offlineCfgOverlap_en;
	int offlineCfgOverlap_left;
	int offlineCfgOverlap_right;
	int offlineCfgOverlap_up;
	int offlineCfgOverlap_down;

	/*bayer*/
	int nlm_bypass;
	int imbalance_bypass;
	int raw_gtm_stat_bypass;
	int raw_gtm_map_bypass;
	int cfa_bypass;

	/*rgb*/
	int ltmsta_rgb_bypass;
	int ltmsta_rgb_binning_en;

	/*yuv*/
	int yuv420to422_bypass;
	int nr3d_bd_bypass;
	int nr3d_bd_FBC_en;
	int pre_cnr_bypass;
	int ynr_bypass;
	int ee_bypass;
	int cnr_new_bypass;
	int post_cnr_bypass;
	int iir_cnr_bypass;

	/*scaler*/
	int scaler_input_format; /*3:422 4:420*/
	slice_drv_overlap_scaler_param scaler1;
	slice_drv_overlap_scaler_param scaler2;

	/************************************************************************/
	/*  output                                                              */
	/************************************************************************/
	int slice_rows;
	int slice_cols;
	slice_drv_region_info  slice_region[PIPE_MAX_SLICE_NUM];
	slice_drv_overlap_info slice_overlap[PIPE_MAX_SLICE_NUM];
}slice_drv_overlap_param_t;

int alg_isp_get_dynamic_overlap(void *cfg_slice_in, void*slc_ctx);
int alg_isp_init_yuvscaler_slice(void *slc_cfg_input, void *slc_ctx, isp_fw_scaler_slice (*slice_param)[4]);

#endif
