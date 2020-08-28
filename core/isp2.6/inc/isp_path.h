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

#ifndef _ISP_PATH_H_
#define _ISP_PATH_H_

#define SHRINK_Y_UP_TH                  235
#define SHRINK_Y_DN_TH                  16
#define SHRINK_UV_UP_TH                 240
#define SHRINK_UV_DN_TH                 16
#define SHRINK_Y_OFFSET                 16
#define SHRINK_Y_RANGE                  3
#define SHRINK_C_OFFSET                 16
#define SHRINK_C_RANGE                  6

int isp_cfg_ctx_base(struct isp_pipe_context *pctx, void *param);
int isp_cfg_ctx_size(struct isp_pipe_context *pctx, void *param);
int isp_cfg_ctx_compression(struct isp_pipe_context *pctx, void *param);
int isp_cfg_ctx_uframe_sync(struct isp_pipe_context *pctx, void *param);

int isp_cfg_path_base(struct isp_path_desc *path, void *param);
int isp_cfg_path_size(struct isp_path_desc *path, void *param);
int isp_cfg_path_dst_size(struct isp_path_desc *path, void *param);
int isp_cfg_path_compression(struct isp_path_desc *path, void *param);
int isp_cfg_path_uframe_sync(struct isp_path_desc *path, void *param);

int isp_set_path(struct isp_path_desc *path);
int isp_path_set_fetch_frm(struct isp_pipe_context *pctx,
		struct camera_frame *frame);
int isp_path_set_store_frm(
		struct isp_path_desc *path,
		struct camera_frame *frame);
int isp_path_set_afbc_store_frm(
		struct isp_path_desc *path,
		struct camera_frame *frame);
int isp_cfg_path_scaler(struct isp_path_desc *path);

int isp_path_set_scaler_coeff(struct coeff_arg *arg,
		uint32_t buf_sel,
		uint32_t spath_id,
		uint32_t *coeff_buf);

#endif
