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

#include "cam_scene.h"

int cam_scene_pipeline_need_dcamonline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 1;
		break;
	case CAM_PIPELINE_SCALER_YUV:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_dcamoffline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_dcamoffline_bpcraw(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_ispoffline(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
		need = 1;
		break;
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_framecache(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
		need = 1;
		break;
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_pyrdec(uint32_t pipeline_type, uint32_t pyrdec_support)
{
	int need = 0;

	if (!pyrdec_support) {
		pr_debug("no need pyrdec node.\n");
		return need;
	}

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_VIDEO_CAPTURE:
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
		need = 1;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_pipeline_need_yuv_scaler(uint32_t pipeline_type)
{
	int need = 0;

	switch (pipeline_type) {
	case CAM_PIPELINE_PREVIEW:
	case CAM_PIPELINE_VIDEO:
	case CAM_PIPELINE_CAPTURE:
	case CAM_PIPELINE_ZSL_CAPTURE:
	case CAM_PIPELINE_THUMBNAIL_PREV:
	case CAM_PIPELINE_THUMBNAIL_CAP:
	case CAM_PIPELINE_VIDEO_CAPTURE:
		need = 1;
		break;
	case CAM_PIPELINE_SENSOR_RAW:
	case CAM_PIPELINE_SCALER_YUV:
	case CAM_PIPELINE_OFFLINE_RAW2YUV:
	case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
	case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
	case CAM_PIPELINE_VCH_SENSOR_RAW:
		need = 0;
		break;
	default :
		pr_err("fail to support pipeline_type %d\n", pipeline_type);
		break;
	}

	return need;
}

int cam_scene_static_linkages_get(struct cam_scene *param, void *hw_info)
{
	int ret = 0;
	struct cam_pipeline_topology *cur_pipeline = NULL;

	if (!param || !hw_info) {
		pr_err("fail to get invalid param ptr %px %px\n", param, hw_info);
		return -EFAULT;
	}

	cur_pipeline = &param->pipeline_list[0];
	ret = cam_pipeline_static_pipelinelist_get(cur_pipeline, &param->pipeline_list_cnt, hw_info);
	if (ret) {
		pr_err("fail to get pipelinelist cnt %d\n", param->pipeline_list_cnt);
		return -EFAULT;
	}

	pr_debug("default pipelinelist cnt %d\n", param->pipeline_list_cnt);
	if (param->pipeline_list_cnt > CAM_SCENE_PIPELINE_NUM)
		pr_warn("warning: pipeline list cnt need to enlarge\n");

	return ret;
}

