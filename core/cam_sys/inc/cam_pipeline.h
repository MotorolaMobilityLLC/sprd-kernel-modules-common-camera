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

#ifndef _CAM_PIPELINE_H_
#define _CAM_PIPELINE_H_

#include "cam_node.h"
#include "cam_types.h"

#define CAM_PIPELINE_NODE_NUM       CAM_NODE_TYPE_MAX
extern uint32_t g_pipeline_type;

enum cam_pipeline_cfg_cmd {
	CAM_PIPELINE_CFG_BUF,
	CAM_PIPELINE_CFG_CAP_PARAM,
	CAM_PIPELINE_CFG_ZOOM,
	CAM_PIPELINE_CFG_BASE,
	CAM_PIPELINE_CFG_CLR_CACHE_BUF,
	CAM_PIPELINE_CFG_DUAL_SYNC_BUF_GET,
	CAM_PIPELINE_CFG_BLK_PARAM,
	CAM_PIPELINE_CFG_PARAM_FID,
	CAM_PIPELINE_CFG_RESET_PARAM_PTR,
	CAM_PIPELINE_CFG_UFRAME,
	CAM_PIPELINE_CFG_STATIS_BUF,
	CAM_PIPILINE_CFG_RESEVER_BUF,
	CAM_PIPILINE_CFG_SHARE_BUF,
	CAM_PIPELINE_CFG_PATH_PAUSE,
	CAM_PIPELINE_CFG_PATH_RESUME,
	CAM_PIPELINE_CFG_RECT_GET,
	CAM_PIPELINE_CFG_GTM_LTM,
	CAM_PIPELINE_CFG_RESET,
	CAM_PIPELINE_CFG_XTM_EN,
	/*TEMP:for cfg isp cur_ctx_id to pyrdecnode*/
	CAM_PIPELINE_CFG_CTXID,
	CAM_PIPELINE_CFG_3DNR_MODE,
	CAM_PIPELINE_CFG_GTM,
	CAM_PIPELINE_CFG_PARAM_SWITCH,
	CAM_PIPRLINE_CFG_PARAM_Q_CLEAR,
	CAM_PIPELINE_CFG_FAST_STOP,
	CAM_PIPELINE_CFG_PRE_RAW_FLAG,
	CAM_PIPELINE_CFG_OPT_SCENE_SWITCH,
	CAM_PIPELINE_GET_CAP_FRAME,
	CAM_PIPELINE_CFG_MAX,
};

enum cam_pipeline_buffer_num {
	CAM_PIPELINE_BUFFER_NUM_NORMAL = 5,
	CAM_PIPELINE_BUFFER_NUM_DEC,
	CAM_PIPELINE_BUFFER_NUM_DUAL,
	CAM_PIPELINE_BUFFER_NUM_NONZSL,
	CAM_PIPELINE_BUFFER_NUM_NONZSL_DEC,
};

struct cam_pipeline_cfg_param {
	enum cam_node_type node_type;
	uint32_t node_id;
	struct cam_node_cfg_param node_param;
};

struct cam_pipeline_shutoff_param {
	enum cam_node_type node_type;
	struct cam_node_shutoff_ctrl node_shutoff;
};

struct cam_pipeline_topology {
	char *name;
	enum cam_pipeline_type type;
	uint32_t node_cnt;
	uint32_t buf_num;
	uint32_t pyr_layer_num;
	uint32_t need_dcam_online;
	uint32_t need_dcam_offline;
	uint32_t need_dcam_offline_bpc;
	uint32_t need_isp_offline;
	uint32_t need_frame_cache;
	uint32_t need_pyr_dec;
	uint32_t need_yuv_scaler;
	struct cam_node_topology nodes[CAM_PIPELINE_NODE_NUM];
};

struct cam_pipeline_desc {
	void *nodes_dev;
	cam_zoom_get_cb zoom_cb_func;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	void *buf_manager_handle;
	struct dcam_online_node_desc dcam_online_desc;
	struct dcam_offline_node_desc dcam_offline_desc;
	struct dcam_offline_node_desc dcam_offline_bpcraw_desc;
	struct dcam_offline_node_desc dcam_offline_raw2frgb_desc;
	struct dcam_offline_node_desc dcam_offline_frgb2yuv_desc;
	struct dcam_fetch_node_desc dcam_fetch_desc;
	struct isp_node_desc isp_node_description;
	struct frame_cache_node_desc frame_cache_desc;
	struct pyr_dec_node_desc pyr_dec_desc;
	struct isp_yuv_scaler_node_desc isp_yuv_scaler_desc;
	struct cam_pipeline_topology *pipeline_graph;
};

struct cam_pipeline_ops {
	int (*cfg_param)(void *handle, enum cam_pipeline_cfg_cmd cmd, void *param);
	int (*streamon)(void *handle, void *param);
	int (*streamoff)(void *handle, void *param);
	int (*cfg_shutoff)(void *handle, enum cam_node_type node_type, void *param);
};

struct cam_pipeline {
	cam_zoom_get_cb zoom_cb_func;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	void *buf_manager_handle;
	struct cam_pipeline_topology *pipeline_graph;
	struct cam_node *node_list[CAM_PIPELINE_NODE_NUM];
	struct cam_pipeline_ops ops;
	enum en_status debug_log_switch;
};

#define CAM_PIPEINE_ISP_NODE_CFG(channel, cmd, nodeid, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = (nodeid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_IN_PORT_CFG(channel, portid, cmd, nodeid, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_IN; \
	param_cfg.node_param.port_id = (portid); \
	param_cfg.node_id = (nodeid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_OUT_PORT_CFG(channel, portid, cmd, nodeid, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_OUT; \
	param_cfg.node_param.port_id = (portid); \
	param_cfg.node_id = (nodeid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_SCALER_NODE_CFG(channel, cmd, nodeid, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_YUV_SCALER; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = (nodeid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DCAM_ONLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = DCAM_ONLINE_PRE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DCAM_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = DCAM_OFFLINE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_OFFLINE_RAW2FRGB_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = DCAM_OFFLINE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_OFFLINE_FRGB2YUV_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = DCAM_OFFLINE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(channel, portid, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DCAM_ONLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_OUT; \
	param_cfg.node_param.port_id = (portid); \
	param_cfg.node_id = DCAM_ONLINE_PRE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(channel, portid, cmd, par, type)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = (type); \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_OUT; \
	param_cfg.node_param.port_id = (portid); \
	param_cfg.node_id = DCAM_OFFLINE_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_FRAME_CACHE_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_FRAME_CACHE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = FRAME_CACHE_CAP_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_PYR_DEC_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_PYR_DEC; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = PYR_DEC_NODE_ID; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_DATA_COPY_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_DATA_COPY; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_id = CAM_COPY_NODE_ID_0; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_SHUTOFF_PARAM_CFG(channel, pipeline_shutoff)  ({ \
	int ret = 0; \
	enum cam_node_type node_type; \
	struct cam_node_shutoff_ctrl node_shutoff; \
	node_type = (pipeline_shutoff).node_type; \
	node_shutoff = (pipeline_shutoff).node_shutoff; \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_shutoff((channel)->pipeline_handle, node_type, &node_shutoff); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

int cam_pipeline_buffer_alloc(struct cam_pipeline *pipe, struct cam_buf_alloc_desc *param);
void *cam_pipeline_creat(struct cam_pipeline_desc *param);
void cam_pipeline_destory(struct cam_pipeline *pipeline);

#endif
