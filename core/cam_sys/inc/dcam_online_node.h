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

#ifndef _DCAM_ONLINE_NODE_H_
#define _DCAM_ONLINE_NODE_H_

#include "cam_types.h"
#include "dcam_online_port.h"
#include "cam_hw.h"
#include "dcam_interface.h"
#include "cam_block.h"

enum dcam_online_node_id {
	DCAM_ONLINE_PRE_NODE_ID,
	DCAM_ONLINE_MAX_NODE_ID,
};

/* fix result */
enum dcam_fix_result {
	DEFER_TO_NEXT,
	INDEX_FIXED,
	BUFFER_READY,
};

enum dcam_slowmotion_type {
	DCAM_SLW_OFF = 0,
	DCAM_SLW_AP,
	DCAM_SLW_FMCU,
};

enum dcam_statis_ioctl_cmd {
	DCAM_IOCTL_CFG_STATIS_BUF,
	DCAM_IOCTL_INIT_STATIS_Q,
	DCAM_IOCTL_DEINIT_STATIS_Q,
	DCAM_IOCTL_CFG_STATIS_BUF_SKIP,
};

struct dcam_statis_param {
	enum dcam_statis_ioctl_cmd statis_cmd;
	void *param;
};

struct dcam_online_node_desc {
	uint32_t node_type;
	uint32_t offline;
	uint32_t slowmotion_count;
	uint32_t enable_3dnr;
	uint32_t is_raw;
	uint32_t raw_cap;
	uint32_t is_4in1;
	uint32_t is_ebd;
	struct img_size input_size;
	struct img_trim input_trim;
	struct img_size output_size;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	share_buf_get_cb sharebuf_get_cb;
	void *sharebuf_cb_data;
	void **node_dev;
	struct dcam_mipi_info cap_info;
	struct dcam_online_port_desc port_desc[PORT_DCAM_OUT_MAX];
	void *dev;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	port_cfg_cb port_cfg_cb_func;
	void *port_cfg_cb_handle;
	shutoff_cfg_cb shutoff_cfg_cb_func;
	void *shutoff_cfg_cb_handle;
	uint32_t is_pyr_rec;
	uint32_t dcam_idx;
	uint32_t dcam_slice_mode_temp;/*TEMP:need delete later*/
	uint32_t raw_alg_type;
	uint32_t param_frame_sync;
	struct sprd_ebd_control ebd_param;
	struct dcam_isp_k_block *blk_pm;
};

struct dcam_online_node {
	atomic_t pm_cnt;
	atomic_t user_cnt;
	atomic_t state;/* TODO: use mutex to protect */
	atomic_t ch_enable_cnt;
	bool need_fix;
	uint32_t node_type;
	uint32_t node_id;
	uint32_t is_3dnr;
	uint32_t is_4in1;
	uint32_t slowmotion_count;
	uint32_t is_ebd;
	uint32_t is_pyr_rec;
	uint32_t csi_connect_stat;
	uint32_t dcam_slice_mode_temp/*TEMP:need delete later*/;
	enum dcam_slowmotion_type slw_type;
	struct sprd_ebd_control ebd_param;

	struct camera_queue port_queue;
	void *data_cb_handle;
	cam_data_cb data_cb_func;
	share_buf_get_cb sharebuf_get_cb;
	void *sharebuf_cb_data;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	port_cfg_cb port_cfg_cb_func;
	void *port_cfg_cb_handle;
	struct camera_buf statis_buf_array[STATIS_TYPE_MAX][STATIS_BUF_NUM_MAX];
	void *nr3_frm;

	uint32_t dcam_idx;
	uint32_t hw_ctx_id;
	struct dcam_pipe_dev *dev;
	struct dcam_hw_context *hw_ctx;
	struct dcam_mipi_info cap_info;
	struct nr3_me_data nr3_me;

	timespec frame_ts[DCAM_FRAME_TIMESTAMP_COUNT];
	ktime_t frame_ts_boot[DCAM_FRAME_TIMESTAMP_COUNT];
	struct dcam_isp_k_block blk_pm;
	uint32_t raw_alg_type;
	uint32_t param_frame_sync;

	shutoff_cfg_cb shutoff_cfg_cb_func;
	void *shutoff_cfg_cb_handle;
};

#define DCAM_ONLINE_NODE_SHUTOFF_CALLBACK(dcam_port)  ({ \
	int ret = 0; \
	struct cam_node_cfg_param node_param = {0}; \
	node_param.port_id = (dcam_port)->port_id; \
	node_param.param = &(dcam_port)->is_shutoff; \
	ret = (dcam_port)->shutoff_cb_func(&node_param, (dcam_port)->shutoff_cb_handle); \
	ret; \
})

int dcam_online_node_irq_proc(void *param, void *handle);
int dcam_online_node_pmctx_init(struct dcam_online_node *node);
void dcam_online_node_pmctx_update(struct dcam_isp_k_block *node_pm, struct dcam_isp_k_block *input_pm);
int dcam_online_node_pmctx_deinit(struct dcam_online_node *node);
int dcam_online_node_xtm_disable(struct dcam_online_node* node, void *param);
int dcam_online_node_port_insert(struct dcam_online_node* node, void *param);
int dcam_online_node_state_get(void *handle);
struct dcam_online_port *dcam_online_node_port_get(struct dcam_online_node *node, uint32_t port_id);
int dcam_online_node_blk_param_set(struct dcam_online_node *node, void *param);
int dcam_online_cfg_param(void *handle, uint32_t cmd, void *param);
int dcam_online_share_buf(void *handle, void *param);
int dcam_online_node_reset(struct dcam_online_node *node, void *param);
int dcam_online_node_request_proc(struct dcam_online_node *node, void *param);
int dcam_online_node_stop_proc(struct dcam_online_node *node, void *param);
int dcam_online_set_shutoff(void *handle, void *param, uint32_t port_id);
void *dcam_online_node_get(uint32_t node_id, struct dcam_online_node_desc *param);
void dcam_online_node_put(struct dcam_online_node *node);
int inline dcam_online_port_reserved_buf_set(struct dcam_online_port *dcam_port, struct camera_frame *frame);

#endif
