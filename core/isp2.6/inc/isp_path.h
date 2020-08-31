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

int isp_path_ctx_base_cfg(struct isp_pipe_context *pctx, void *param);
int isp_path_ctx_size_cfg(struct isp_pipe_context *pctx, void *param);
int isp_path_ctx_compression_cfg(struct isp_pipe_context *pctx, void *param);
int isp_path_ctx_uframe_sync_cfg(struct isp_pipe_context *pctx, void *param);

int isp_path_base_cfg(struct isp_path_desc *path, void *param);
int isp_path_size_cfg(struct isp_path_desc *path, void *param);
int isp_path_compression_cfg(struct isp_path_desc *path, void *param);
int isp_path_uframe_sync_cfg(struct isp_path_desc *path, void *param);

int isp_path_set(struct isp_path_desc *path);
int isp_path_fetch_frm_set(struct isp_pipe_context *pctx,
		struct camera_frame *frame);
int isp_path_store_frm_set(
		struct isp_path_desc *path,
		struct camera_frame *frame);
int isp_path_afbc_store_frm_set(
		struct isp_path_desc *path,
		struct camera_frame *frame);
int isp_path_scaler_cfg(struct isp_path_desc *path);

int isp_path_scaler_coeff_set(struct coeff_arg *arg,
		uint32_t buf_sel,
		uint32_t spath_id,
		uint32_t *coeff_buf);

#endif
