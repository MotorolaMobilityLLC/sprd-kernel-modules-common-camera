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

#ifndef _ISP_NODE_H_
#define _ISP_NODE_H_

#include "cam_types.h"
#include "isp_port.h"
#include "isp_ltm.h"
#include "isp_int.h"

enum isp_node_mode_id {
	ISP_NODE_MODE_PRE_ID,
	ISP_NODE_MODE_CAP_ID,
	ISP_NODE_MODE_MAX_ID,
};

enum isp_node_cfg_cmd {
	ISP_NODE_CFG_INIT,
	ISP_NODE_CFG_BASE,
	ISP_NODE_CFG_INSERT_PORT,
	ISP_NODE_CFG_CLEAR_PORT,
	ISP_NODE_CFG_SIZE_CFG,
	ISP_NODE_CFG_BLK_PATAM,
	ISP_NODE_CFG_PORT_UFRAME,
	ISP_NODE_CFG_PORT_BUF,
	ISP_NODE_CFG_STATIS,
	ISP_NODE_CFG_LTM_BUF,
	ISP_NODE_CFG_RESERVE_BUF,
	ISP_NODE_CFG_3DNR_MODE,
	ISP_NODE_CFG_3DNR_BUF,
	ISP_NODE_CFG_REC_LEYER_NUM,
	ISP_NODE_CFG_REC_BUF,
	ISP_NODE_CFG_GTM,
	ISP_NODE_CFG_PARAM_SWITCH,
	ISP_NODE_CFG_PARAM_Q_CLEAR,
	ISP_NODE_CFG_FAST_STOP,
	ISP_NODE_CFG_SUPERZOOM_BUF,
	ISP_NODE_CFG_RECYCLE_BLK_PARAM,
	ISP_NODE_CFG_CMD_MAX,
};

struct isp_node_postproc_param {
	ktime_t *boot_time;
	timespec *cur_ts;
	uint32_t zoom_ratio;
	uint32_t total_zoom;
};

struct isp_node_slwm960fps_frame_num {
	uint32_t stage_a_frame_num;
	uint32_t stage_b_frame_num;
	uint32_t stage_c_frame_num;
};

struct isp_node_uinfo {
	uint32_t ltm_rgb;
	uint32_t mode_ltm;
	uint32_t gtm_rgb;
	uint32_t mode_gtm;
	uint32_t mode_3dnr;
	uint32_t slw_state;
	uint32_t enable_slowmotion;
	uint32_t slowmotion_240fp_count;
	uint32_t slowmotion_count;
	struct slowmotion_960fps_info slw_960desc;
	struct isp_node_slwm960fps_frame_num frame_num;
	uint32_t stage_a_valid_count;
	uint32_t uframe_sync;
	uint32_t scaler_coeff_ex;
	uint32_t pyr_layer_num;
	uint32_t nr3_fbc_fbd;
	enum isp_fetch_path_select fetch_path_sel;
};

struct isp_node_desc {
	uint32_t is_dual;
	uint32_t node_type;
	uint32_t mode_3dnr;
	uint32_t mode_ltm;
	uint32_t ltm_rgb;
	uint32_t mode_gtm;
	uint32_t gtm_rgb;
	uint32_t in_fmt;
	uint32_t bayer_pattern;
	uint32_t enable_slowmotion;
	uint32_t slw_state;
	uint32_t slowmotion_count;
	uint32_t pyr_out_fmt;
	uint32_t store_3dnr_fmt;
	uint32_t nr3_fbc_fbd;
	enum isp_fetch_path_select fetch_path_sel;
	enum cam_ch_id ch_id;
	struct img_size sn_size;
	uint32_t is_high_fps;
	uint32_t cam_id;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	void **node_dev;
	struct isp_port_desc port_desc;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	uint32_t blkparam_node_num;
	/*TEMP DEC*/
	void *isp_node;
	void *dev;
	struct slowmotion_960fps_info fps960_info;
};

struct isp_node {
	uint32_t cfg_id;
	uint32_t node_type;
	uint32_t node_id;
	atomic_t user_cnt;
	uint32_t is_bind;
	uint32_t is_dual;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	uint32_t pctx_hw_id;
	uint32_t in_irq_postproc;
	atomic_t state_user_cnt;
	uint32_t slw_frm_cnt;
	uint32_t is_fast_stop;
	uint32_t iommu_status;
	enum camera_id attach_cam_id;
	enum cam_ch_id ch_id;

	struct isp_node_uinfo uinfo;
	struct isp_node_uinfo pipe_src;

	struct camera_queue port_queue;
	struct isp_pipe_dev *dev;
	struct dcam_isp_k_block isp_k_param;
	struct camera_frame *isp_receive_param;
	struct dcam_isp_k_block *isp_using_param;

	struct cam_thread_info thread;
	struct cam_thread_info isp_interrupt_thread;
	struct completion frm_done;
	struct completion *fast_stop_done;
	/* lock block param to avoid acrossing frame */
	struct mutex blkpm_lock;
	struct camera_queue param_share_queue;
	struct camera_queue param_buf_queue;
	struct camera_queue isp_interrupt_queue;
	struct mutex blkpm_q_lock;

	struct camera_buf statis_buf_array[STATIS_TYPE_MAX][STATIS_BUF_NUM_MAX];
	struct camera_queue hist2_result_queue;
	struct camera_queue gtmhist_result_queue;
	struct isp_int_ctxs_com ctxs_com;
	uint32_t zoom_conflict_with_ltm;

	void *rgb_ltm_handle;
	void *rgb_gtm_handle;
	void *nr3_handle;
	void *rec_handle;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	struct camera_frame *postproc_buf;
	uint32_t nr3_blend_cnt;
};

void *isp_node_get(uint32_t node_id, struct isp_node_desc *param);
void isp_node_put(struct isp_node *node);
void isp_node_close(struct isp_node *node);
int isp_node_request_proc(struct isp_node *node, void *param);
uint32_t isp_node_config(void *node, enum isp_node_cfg_cmd cmd, void *param);
int isp_node_prepare_blk_param(struct isp_node *inode, uint32_t target_fid, struct blk_param_info *out);
void isp_node_param_buf_destroy(void *param);
#endif
