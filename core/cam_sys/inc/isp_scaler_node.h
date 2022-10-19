/*
 * Copyright (C) 2021-2022 UNISOC Communications Inc.
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

#ifndef _ISP_SCALER_NODE_H_
#define _ISP_SCALER_NODE_H_

#include "cam_types.h"
#include "isp_scaler_port.h"
#include "isp_int.h"

struct scaler_tmp_param {
	int valid_out_frame;
	int hw_ctx_id;
	uint32_t multi_slice;
	uint32_t target_fid;
};

struct isp_scaler_path_uinfo {
	uint32_t out_fmt;
	uint32_t regular_mode;
	uint32_t uframe_sync;
	uint32_t scaler_coeff_ex;
	uint32_t scaler_bypass_ctrl;
	uint32_t data_endian;
	struct img_size dst;
	struct img_trim in_trim;
};

struct isp_scaler_uinfo {
	/* common info from cam core */
	uint32_t in_fmt;
	/* GR, R, B, Gb */
	uint32_t bayer_pattern;
	uint32_t uframe_sync;
	uint32_t scaler_coeff_ex;
	/* input info from cam core */
	struct img_size sn_size;/* sensor size */
	struct img_size src;
	struct img_trim crop;
	struct img_scaler_info original;
	/* output info from cam core */
	struct isp_scaler_path_uinfo path_info[ISP_SPATH_NUM];
};

enum isp_yuv_scaler_node_id {
	ISP_YUV_SCALER_PRE_NODE_ID,
	ISP_YUV_SCALER_CAP_NODE_ID,
	ISP_YUV_SCALER_MAX_NODE_ID,
};

enum isp_yuv_scaler_node_cfg_cmd {
	ISP_YUV_SCALER_NODE_CFG_BASE,
	ISP_YUV_SCALER_NODE_INSERT_PORT,
	ISP_YUV_SCALER_NODE_SIZE_CFG,
	ISP_YUV_SCALER_NODE_CFG_PORT_UFRAME,
	ISP_YUV_SCALER_NODE_CFG_PORT_BUF,
	ISP_YUV_SCALER_NODE_CFG_RESERVE_BUF,
	ISP_YUV_SCALER_NODE_CFG_MAX,
};

struct isp_yuv_scaler_node_desc {
	uint32_t node_type;
	uint32_t uframe_sync;
	void **node_dev;
	void *dev;
	enum cam_ch_id ch_id;
	uint32_t cam_id;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	struct isp_scaler_port_desc port_desc;
	uint32_t node_id;
	uint32_t hw_path_id;
	uint32_t out_fmt;
	uint32_t endian;
	struct img_size output_size;
	uint32_t regular_mode;
};

struct isp_yuv_scaler_node {
	uint32_t node_type;
	uint32_t node_id;
	uint32_t cfg_id;//cfg_id mean p0 c0 p1 c1 buffer
	enum cam_ch_id ch_id;
	enum camera_id attach_cam_id;
	uint32_t is_bind;
	atomic_t user_cnt;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	struct isp_pipe_dev *dev;
	struct isp_scaler_uinfo pipe_src;
	struct isp_scaler_uinfo uinfo;
	struct cam_thread_info thread;
	struct completion frm_done;
	struct camera_queue in_queue;
	struct camera_queue proc_queue;
	struct camera_queue port_queue;
	uint32_t started;
	uint32_t iommu_status;
	struct mutex blkpm_lock;
	uint32_t multi_slice;
	uint32_t out_fmt;
	uint32_t endian;
	struct img_size output_size;
	uint32_t regular_mode;
	void *slice_ctx;
	uint32_t valid_slc_num;
	struct cam_thread_info yuv_scaler_interrupt_thread;
	struct camera_queue yuv_scaler_interrupt_queue;
	uint32_t pctx_hw_id;
	struct isp_int_ctxs_com ctxs_com;
};

void *isp_yuv_scaler_node_get(uint32_t node_id, struct isp_yuv_scaler_node_desc *param);
void isp_yuv_scaler_node_put (struct isp_yuv_scaler_node *node);
int isp_scaler_node_request_proc(struct isp_yuv_scaler_node *node, void *param);
uint32_t isp_yuv_scaler_cfg_param(void *node, uint32_t port_id, uint32_t cmd, void *param);

#endif

