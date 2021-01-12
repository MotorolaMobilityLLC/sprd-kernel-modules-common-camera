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

#ifndef _ISP_DEWARP_H_
#define _ISP_DEWARP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cam_queue.h"
#include "dcam_interface.h"
#include "isp_interface.h"

#define DEWARPING_DST_MBLK_SIZE       16
#define DEWARPING_MAX_LINE_LENGTH     0x100
#define DEWARPING_MAX_GRID_SIZE       (1 << 8)
#define DEWARPING_LXY_SHIFT           (3 + 3)
#define DEWARPING_LXY_MULT            (1 << DEWARPING_LXY_SHIFT)
#define DEWARPING_GRID_BUF            (0x1180)

enum isp_dewarp_mode{
	ISP_DEWARPING_WARP_RECTIFY,
	ISP_DEWARPING_WARP_UNDISTORT,
	ISP_DEWARPING_WARP_PROJECTIVE,
	ISP_DEWARPING_WARP_OFF,
	ISP_DEWARPING_WARP_MAX,
};

enum isp_dewarp_cfg_cmd{
	ISP_DEWARPING_CFG_SIZE,
	ISP_DEWARPING_CFG_COEFF,
	ISP_DEWARPING_CFG_MODE,
	ISP_DEWARPING_CFG_MAX,
};

struct isp_dewarp_ops {
	int (*cfg_param)(void *dewarp_handle, enum isp_dewarp_cfg_cmd cmd, void *param);
	int (*pipe_proc)(void *dewarp_handle, void *param, enum isp_dewarp_mode mode);
};

enum isp_dewarp_cache_format {
	ISP_DEWARP_YUV420_2FRAME = 0,
	ISP_DEWARP_YVU420_2FRAME,
	ISP_DEWARP_FORMAT_MAX
};

struct isp_dewarp_cache_info {
	uint32_t ctx_id;
	enum isp_fetch_path_select fetch_path_sel;
	uint32_t dewarp_cache_bypass;
	uint32_t dewarp_cache_endian;
	uint32_t dewarp_cache_prefetch_len;
	uint32_t dewarp_cache_mipi;
	enum isp_dewarp_cache_format yuv_format;
	struct img_addr addr;
	uint32_t frame_pitch;
};

struct isp_dewarping_blk_info{
	uint32_t cxt_id;
	uint32_t dst_width;
	uint32_t dst_height;
	uint32_t grid_size;
	uint32_t grid_num_x;
	uint32_t grid_num_y;
	uint32_t pos_x;
	uint32_t pos_y;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t start_mb_x;
	uint32_t mb_x_num;
	uint32_t mb_y_num;
	uint32_t init_start_row;
	uint32_t init_start_col;
	uint32_t crop_start_x;
 	uint32_t crop_start_y;
	uint32_t grid_data_size;
	uint32_t dewarping_lbuf_ctrl_nfull_size;
	uint32_t chk_clr_mode;
	uint32_t chk_wrk_mode;
	/* default 1 : dewarping_mode:0:rectify 1:undistort 2:projective */
	enum isp_dewarp_mode dewarp_mode;
	/* default 0 :grid table mode:0:matrix_file 1:grid_xy_file */
	uint32_t grid_table_mode;
	uint32_t grid_x_ch0_buf[DEWARPING_GRID_BUF];
	uint32_t grid_y_ch0_buf[DEWARPING_GRID_BUF];
	/* PXL_COEF_CH0 */
	uint32_t pixel_interp_coef[DEWARPING_LXY_MULT*3];
	/* CORD_COEF_CH0 */
	uint32_t bicubic_coef_i[DEWARPING_MAX_GRID_SIZE*3];
};

struct isp_dewarp_ctx_desc {
	uint32_t idx;
	uint32_t in_fmt;
	uint32_t hw_ctx_id;
	uint32_t grid_size;
	enum isp_dewarp_mode mode;
	struct img_addr fetch_addr;
	struct img_size src_size;
	struct img_size dst_size;
	struct camera_frame *buf_info;
	void *fmcu_handle;
	void *grid_coef_handle;
	struct isp_dewarp_cache_info  dewarp_cache_info;
	struct isp_dewarping_blk_info dewarp_blk_info;
	struct isp_dewarp_ops ops;
};

#ifdef __cplusplus
}
#endif

#endif/* _ISP_DEWARP_H_ */

