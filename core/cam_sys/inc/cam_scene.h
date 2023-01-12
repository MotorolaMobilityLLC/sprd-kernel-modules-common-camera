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

int cam_scene_pipeline_need_dcamoffline(uint32_t pipeline_type);
int cam_scene_pipeline_need_framecache(uint32_t pipeline_type);
int cam_scene_pipeline_need_ispoffline(uint32_t pipeline_type);
int cam_scene_pipeline_need_dcamonline(uint32_t pipeline_type);
int cam_scene_pipeline_need_dcamoffline_bpcraw(uint32_t pipeline_type);
int cam_scene_pipeline_need_pyrdec(uint32_t pipeline_type, uint32_t pyrdec_support);
int cam_scene_pipeline_need_yuv_scaler(uint32_t pipeline_type);
int cam_scene_static_linkages_get(struct cam_scene *param, void *hw_info);

#endif
