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
#include "cam_link.h"
#include "cam_types.h"

#define CAM_PIPELINE_NODE_NUM       CAM_NODE_TYPE_MAX
extern uint32_t g_pipeline_type;

enum cam_pipeline_type {
	CAM_PIPELINE_PREVIEW,
	CAM_PIPELINE_VIDEO,
	CAM_PIPELINE_CAPTURE,
	CAM_PIPELINE_ZSL_CAPTURE,
	CAM_PIPELINE_THUMBNAIL_PREV,
	CAM_PIPELINE_THUMBNAIL_CAP,
	CAM_PIPELINE_VIDEO_CAPTURE,
	CAM_PIPELINE_SENSOR_RAW,
	CAM_PIPELINE_SCALER_YUV,
	CAM_PIPELINE_OFFLINE_RAW2YUV,
	CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV,
	CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV,
	CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV,
	CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV,
	CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV,
	CAM_PIPELINE_VCH_SENSOR_RAW,
	CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV,
	CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV,
	CAM_PIPELINE_TYPE_MAX,
};

enum cam_pipeline_cfg_cmd {
	CAM_PIPELINE_CFG_BUF,
	CAM_PIPELINE_CFG_CAP_PARAM,
	CAM_PIPELINE_CFG_SIZE,
	CAM_PIPELINE_CFG_ZOOM,
	CAM_PIPELINE_CFG_BASE,
	CAM_PIPELINE_CLR_CACHE_BUF,
	CAM_PIPELINE_DUAL_SYNC_BUF_GET,
	CAM_PIPELINE_CFG_BLK_PARAM,
	CAM_PIPELINE_CFG_PARAM_FID,
	CAM_PIPELINE_RESET_PARAM_PTR,
	CAM_PIPELINE_CFG_UFRAME,
	CAM_PIPELINE_CFG_STATIS_BUF,
	CAM_PIPILINE_CFG_RESEVER_BUF,
	CAM_PIPILINE_CFG_SHARE_BUF,
	CAM_PIPELINE_CFG_PATH_PAUSE,
	CAM_PIPELINE_CFG_PATH_RESUME,
	CAM_PIPELINE_RECT_GET,
	CAM_PIPELINE_CFG_GTM_LTM,
	CAM_PIPELINE_RESET,
	CAM_PIPELINE_CFG_XTM_EN,
	/*TEMP:for cfg isp cur_ctx_id to pyrdecnode*/
	CAM_PIPELINE_CFG_CTXID,
	CAM_PIPELINE_CFG_3DNR_MODE,
	CAM_PIPELINE_CFG_REC_LEYER_NUM,
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
	CAM_NORMAL_ALLOC_BUF_NUM = 5,
	CAM_DEC_ALLOC_BUF_NUM,
	CAM_DUAL_ALLOC_BUF_NUM,
	CAM_NONZSL_ALLOC_BUF_NUM,
	CAM_NONZSL_DEC_ALLOC_BUF_NUM,
};

struct cam_pipeline_cfg_param {
	enum cam_node_type node_type;
	struct cam_node_cfg_param node_param;
};

struct cam_pipeline_shutoff_param {
	enum cam_node_type node_type;
	struct cam_node_shutoff_ctrl node_shutoff;
};

struct cam_pipeline_topology {
	enum cam_pipeline_type type;
	uint32_t node_cnt;
	struct cam_node_topology nodes[CAM_PIPELINE_NODE_NUM];
	uint32_t buf_num;
};

struct cam_pipeline_desc {
	void *nodes_dev;
	cam_zoom_get_cb zoom_cb_func;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
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
	struct cam_pipeline_topology *pipeline_graph;
	struct cam_node *node_list[CAM_PIPELINE_NODE_NUM];
	struct cam_pipeline_ops ops;
	uint32_t debug_log_switch;
};

#define CAM_PIPEINE_ISP_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_IN_PORT_CFG(channel, portid, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_IN; \
	param_cfg.node_param.port_id = (portid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_OUT_PORT_CFG(channel, portid, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_OFFLINE; \
	param_cfg.node_param.param = (par); \
	param_cfg.node_param.port_type = PORT_TRANSFER_OUT; \
	param_cfg.node_param.port_id = (portid); \
	if ((channel)->pipeline_handle) \
		ret = (channel)->pipeline_handle->ops.cfg_param((channel)->pipeline_handle, (cmd), &param_cfg); \
	else { \
		pr_warn("warning: current channel not contain pipeline\n"); \
		ret = -EFAULT; \
	} \
	ret; \
})

#define CAM_PIPEINE_ISP_SCALER_NODE_CFG(channel, cmd, par)  ({ \
	struct cam_pipeline_cfg_param param_cfg = {0}; \
	int ret = 0; \
	param_cfg.node_type = CAM_NODE_TYPE_ISP_YUV_SCALER; \
	param_cfg.node_param.param = (par); \
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

const char *cam_pipeline_name_get(enum cam_pipeline_type type);
int cam_pipeline_static_pipelinelist_get(struct cam_pipeline_topology *param, uint32_t *cnt, void *hw_info);
void *cam_pipeline_creat(struct cam_pipeline_desc *param);
int cam_pipeline_buffer_alloc(struct cam_pipeline *pipe, struct cam_buf_alloc_desc *param);
void cam_pipeline_destory(struct cam_pipeline *pipeline);

#endif
