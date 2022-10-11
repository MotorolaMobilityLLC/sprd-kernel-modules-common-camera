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

#ifndef _ISP_PORT_H_
#define _ISP_PORT_H_

#include "cam_types.h"
#include "cam_port.h"

#define ISP_IN_Q_LEN               8
#define ISP_PROC_Q_LEN             2
#define ISP_RESULT_Q_LEN           2
#define ISP_SLW_IN_Q_LEN           50
#define ISP_SLW_PROC_Q_LEN         50
#define ISP_SLW_RESULT_Q_LEN       50
#define ISP_OUT_BUF_Q_LEN          96
#define ISP_RESERVE_BUF_Q_LEN      48
#define ISP_STREAM_STATE_Q_LEN     12

#define ISP_PIXEL_ALIGN_WIDTH      4
#define ISP_PIXEL_ALIGN_HEIGHT     2
#define ISP_ALIGN_W(_a)            ((_a) & ~(ISP_PIXEL_ALIGN_WIDTH - 1))
#define ISP_ALIGN_H(_a)            ((_a) & ~(ISP_PIXEL_ALIGN_HEIGHT - 1))
#define ISP_DIV_ALIGN_W(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_WIDTH - 1))
#define ISP_DIV_ALIGN_H(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_HEIGHT - 1))

enum isp_port_cfg_callback {
	ISP_PORT_SIZE_UPDATA,
	ISP_PORT_IRQ_POSTPORC,
	ISP_PORT_UFRAME_FID_GET,
	ISP_PORT_FRAME_CYCLE,
	ISP_PORT_PIPEINFO_GET,
	ISP_PORT_SLICE_NEED,
	ISP_PORT_BLKSIZE_GET,
	ISP_PORT_START_ERROR,
	ISP_PORT_SLOWMOTION,
	ISP_PORT_FAST_STOP,
	ISP_PORT_CFG_MAX,
};

struct isp_port_blksize_desc {
	uint32_t new_width;
	uint32_t old_width;
	uint32_t new_height;
	uint32_t old_height;
	struct img_size sn_size;
	struct img_size src_size;
	uint32_t ch_id;
};

struct isp_port_cfg {
	uint32_t scaler_coeff_ex;
	uint32_t scaler_bypass_ctrl;
	uint32_t cfg_id;
	uint32_t in_fmt;
	uint32_t vid_valid;
	uint32_t out_buf_clear;
	int valid_out_frame;
	int hw_ctx_id;
	uint32_t target_fid;
	struct check_blk_param *blkparam_info;
	struct camera_frame *src_frame;
	struct camera_frame *superzoom_frame;
	struct isp_ltm_ctx_desc *rgb_ltm;
	struct isp_pipe_info *pipe_info;
	enum sprd_cam_sec_mode sec_mode;
	struct isp_hw_slw_fmcu_cmds *slw;
	struct isp_pipe_dev *dev;
	struct camera_queue *param_share_queue;
	struct isp_node_uinfo *uinfo;
	uint32_t *faststop;
	struct completion *faststop_done;
	uint32_t need_post_proc[ISP_SPATH_NUM];
	struct img_size src_size;
	struct img_trim src_crop;
};

struct isp_port_desc {
	void **port_dev;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	enum cam_port_transfer_type transfer_type;
	enum export_type depend_type;
	enum isp_fetch_path_select fetch_path_sel;
	uint32_t out_fmt;
	uint32_t slave_type;
	uint32_t slave_path_id;
	uint32_t regular_mode;
	uint32_t data_bits;
	uint32_t endian;
	struct img_size output_size;
	uint32_t in_fmt;
	uint32_t pyr_out_fmt;
	uint32_t store_3dnr_fmt;
	uint32_t bayer_pattern;
	struct img_size sn_size;
	uint32_t is_high_fps;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	struct cam_hw_info *hw;
};

struct isp_port {
	struct list_head list;
	atomic_t user_cnt;
	atomic_t is_work;
	uint32_t port_id;
	uint32_t type;
	int32_t reserved_buf_fd;
	size_t reserve_buf_size;
	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
	uint32_t zoom_conflict_with_ltm;
	uint32_t fmt;
	uint32_t bind_type;
	uint32_t regular_mode;
	uint32_t uframe_sync;
	uint32_t scaler_coeff_ex;
	uint32_t scaler_bypass_ctrl;
	uint32_t data_endian;
	struct img_size size;
	struct img_trim trim;
	struct img_size ori_src;
	struct img_size sn_size;/* sensor size */
	struct img_scaler_info original;
	enum cam_format pyr_out_fmt;
	enum cam_format store_3dnr_fmt;
	uint32_t bayer_pattern;
	enum isp_fetch_path_select fetch_path_sel;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	port_cfg_cb port_cfg_cb_func;
	struct cam_hw_info *hw;
};

uint32_t isp_port_id_switch(uint32_t port_id);
int isp_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param);
void *isp_port_get(uint32_t port_id, struct isp_port_desc *param);
void isp_port_put(struct isp_port *port);

#endif
