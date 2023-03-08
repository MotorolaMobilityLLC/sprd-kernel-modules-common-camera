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

#ifndef _CAM_SCENE_H_
#define _CAM_SCENE_H_

#include "cam_pipeline.h"

#define CAM_SCENE_PIPELINE_NUM       CAM_PIPELINE_TYPE_MAX

struct cam_scene {
	uint32_t pipeline_list_cnt;
	struct cam_pipeline_topology pipeline_list[CAM_SCENE_PIPELINE_NUM];
};

int cam_scene_static_linkages_get(struct cam_scene *param, void *hw_info);
int cam_scene_reserved_buf_cfg(enum reserved_buf_cb_type type, void *param, void *priv_data);
uint32_t cam_scene_dcamonline_buffers_alloc_num(void *channel_ptr, void *module_ptr, uint32_t pipeline_type);
uint32_t cam_scene_dcamoffline_buffers_alloc_num(void *channel_ptr, void *module_ptr);
int cam_scene_dcamonline_desc_get(void *module_ptr, void *channel_ptr, uint32_t pipeline_type, struct dcam_online_node_desc *dcam_online_desc);
int cam_scene_dcamoffline_desc_get(void *module_ptr, void *channel_ptr, uint32_t pipeline_type, struct dcam_offline_node_desc *dcam_offline_desc);
int cam_scene_dcamoffline_bpcraw_desc_get(void *module_ptr, void *channel_ptr, struct dcam_offline_node_desc *dcam_offline_desc);
int cam_scene_ispoffline_desc_get(void *module_ptr, void *channel_ptr, uint32_t pipeline_type, struct isp_node_desc *isp_node_description);
int cam_scene_isp_yuv_scaler_desc_get(void *module_ptr, void *channel_ptr, struct isp_yuv_scaler_node_desc *isp_yuv_scaler_desc);

#endif
