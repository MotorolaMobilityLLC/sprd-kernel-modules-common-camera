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

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>

#include "cam_copy_node.h"
#include "cam_dump_node.h"
#include "cam_node.h"
#include "cam_pipeline.h"
#include "cam_queue.h"
#include "dcam_offline_node.h"
#include "isp_scaler_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_PIPELINE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define IS_VALID_NODE(type) ((type) >= CAM_NODE_TYPE_DCAM_ONLINE && (type) < CAM_NODE_TYPE_MAX)

enum pipeline_prev_type {
	PIPELINE_PREVIEW_TYPE,
	PIPELINE_CAPTURE_TYPE,
	PIPELINE_VIDEO_TYPE,
	PIPELINE_THUMB_TYPE,
	PIPELINE_SCENE_TYPE_MAX,
};

static const char *_CAM_PIPELINE_NAMES[CAM_PIPELINE_TYPE_MAX] = {
	[CAM_PIPELINE_PREVIEW] = "CAM_PIPELINE_PREVIEW",
	[CAM_PIPELINE_VIDEO] = "CAM_PIPELINE_VIDEO",
	[CAM_PIPELINE_CAPTURE] = "CAM_PIPELINE_CAPTURE",
	[CAM_PIPELINE_ZSL_CAPTURE] = "CAM_PIPELINE_ZSL_CAPTURE",
	[CAM_PIPELINE_THUMBNAIL_PREV] = "CAM_PIPELINE_THUMBNAIL_PREV",
	[CAM_PIPELINE_THUMBNAIL_CAP] = "CAM_PIPELINE_THUMBNAIL_CAP",
	[CAM_PIPELINE_VIDEO_CAPTURE] = "CAM_PIPELINE_VIDEO_CAPTURE",
	[CAM_PIPELINE_SENSOR_RAW] = "CAM_PIPELINE_SENSOR_RAW",
	[CAM_PIPELINE_SCALER_YUV] = "CAM_PIPELINE_SCALER_YUV",
	[CAM_PIPELINE_OFFLINE_RAW2YUV] = "CAM_PIPELINE_OFFLINE_RAW2YUV",
	[CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV] = "CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV",
	[CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV] = "CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV",
	[CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV] = "CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV",
	[CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV] = "CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV",
	[CAM_PIPELINE_VCH_SENSOR_RAW] = "CAM_PIPELINE_VCH_SENSOR_RAW",
	[CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV] = "CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV",
	[CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV] = "CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV",
};

const char *cam_pipeline_name_get(enum cam_pipeline_type type)
{
	return _CAM_PIPELINE_NAMES[type];
}

static int campipeline_outport_type_get (uint32_t type)
{
	int outport_type = 0;

	switch (type) {
	case PIPELINE_PREVIEW_TYPE:
		outport_type = PORT_PRE_OUT;
		break;
	case PIPELINE_CAPTURE_TYPE:
		outport_type = PORT_CAP_OUT;
		break;
	case PIPELINE_VIDEO_TYPE:
		outport_type = PORT_VID_OUT;
		break;
	case PIPELINE_THUMB_TYPE:
		outport_type = PORT_THUMB_OUT;
		break;
	default:
		pr_err("fail to get invalid pipeline_type %s\n", cam_pipeline_name_get(type));
		break;
	}
	return outport_type;
}

static void campipeline_preview_get(struct cam_pipeline_topology *param, uint32_t type)
{
	int i = 0, outport_type = 0;
	struct cam_node_topology *cur_node = NULL;
	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
		CAM_NODE_TYPE_DUMP,
		CAM_NODE_TYPE_DATA_COPY,
		CAM_NODE_TYPE_ISP_OFFLINE,
		CAM_NODE_TYPE_ISP_YUV_SCALER,
	};

	outport_type = campipeline_outport_type_get(type);
	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	/* dump en should only com from g_debug_info */
	cur_node->outport[PORT_BIN_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_BIN_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_BIN_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->outport[PORT_BIN_OUT].link.node_id = ISP_NODE_MODE_PRE_ID;
	cur_node->outport[PORT_BIN_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
	for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
		cur_node->outport[i].link_state = PORT_LINK_NORMAL;
		cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = CAM_COPY_NODE_ID_0;
	cur_node++;
	cur_node->id = ISP_NODE_MODE_PRE_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = PORT_BIN_OUT;
	cur_node->inport[PORT_ISP_OFFLINE_IN].copy_en = 1;
	cur_node->inport[PORT_ISP_OFFLINE_IN].copy_node_id = CAM_COPY_NODE_ID_0;
	cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
	cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_ISP_YUV_SCALER;
	cur_node->outport[outport_type].link.node_id = ISP_YUV_SCALER_PRE_NODE_ID;
	cur_node->outport[outport_type].link.port_id = PORT_PRE_ISP_YUV_SCALER_IN;
	if (type == PIPELINE_PREVIEW_TYPE) {
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_SLAVE;
	} else if (type == PIPELINE_VIDEO_TYPE)
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_MASTER;
	cur_node++;
	cur_node->id = ISP_YUV_SCALER_PRE_NODE_ID;
	cur_node->inport[PORT_PRE_ISP_YUV_SCALER_IN].link_state = PORT_LINK_IDLE;
	cur_node->inport[PORT_PRE_ISP_YUV_SCALER_IN].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->inport[PORT_PRE_ISP_YUV_SCALER_IN].link.node_id = ISP_NODE_MODE_PRE_ID;
	cur_node->inport[PORT_PRE_ISP_YUV_SCALER_IN].link.port_id = outport_type;
	switch (type) {
	case PIPELINE_PREVIEW_TYPE:
		cur_node->outport[PORT_PRE_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_PRE_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_VIDEO_TYPE:
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_THUMB_TYPE:
		cur_node->outport[PORT_THUMB_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_THUMB_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	default:
		pr_err("fail to get invalid pipeline_type %s\n", cam_pipeline_name_get(type));
		break;
	}
}

static void campipeline_capture_get(struct cam_pipeline_topology *param, uint32_t type, uint32_t pyrdec_support)
{
	int i = 0, outport_type = 0;
	struct cam_node_topology *cur_node = NULL;

	outport_type = campipeline_outport_type_get(type);
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FULL_OUT;
	if (pyrdec_support) {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = PYR_DEC_NODE_ID;
		for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
	} else {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = ISP_NODE_MODE_CAP_ID;
		for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = PORT_FULL_OUT;
	cur_node->inport[PORT_ISP_OFFLINE_IN].copy_en = 1;
	cur_node->inport[PORT_ISP_OFFLINE_IN].copy_node_id = CAM_COPY_NODE_ID_0;
	cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
	cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_ISP_YUV_SCALER;
	cur_node->outport[outport_type].link.node_id = ISP_YUV_SCALER_CAP_NODE_ID;
	cur_node->outport[outport_type].link.port_id = PORT_CAP_ISP_YUV_SCALER_IN;
	if (type == PIPELINE_CAPTURE_TYPE) {
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_SLAVE;
	} else if (type == PIPELINE_VIDEO_TYPE)
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_MASTER;
	cur_node++;
	cur_node->id = ISP_YUV_SCALER_CAP_NODE_ID;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link_state = PORT_LINK_IDLE;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.port_id = outport_type;
	switch (type) {
	case PIPELINE_CAPTURE_TYPE:
		cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_VIDEO_TYPE:
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_THUMB_TYPE:
		cur_node->outport[PORT_THUMB_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_THUMB_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	default:
		pr_err("fail to get invalid pipeline_type %s\n", cam_pipeline_name_get(type));
		break;
	}

}

static void campipeline_zsl_capture_get(struct cam_pipeline_topology *param, uint32_t type, uint32_t pyrdec_support)
{
	int i = 0, outport_type = 0;
	struct cam_node_topology *cur_node = NULL;

	outport_type = campipeline_outport_type_get(type);
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FRAME_CACHE_IN;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.port_id = PORT_FULL_OUT;
	if (pyrdec_support) {
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
	} else {
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = FRAME_CACHE_CAP_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = PORT_FRAME_CACHE_OUT;
		cur_node->inport[PORT_ISP_OFFLINE_IN].copy_en = 1;
		cur_node->inport[PORT_ISP_OFFLINE_IN].copy_node_id = CAM_COPY_NODE_ID_0;
	}
	cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
	cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_ISP_YUV_SCALER;
	cur_node->outport[outport_type].link.node_id = ISP_YUV_SCALER_CAP_NODE_ID;
	cur_node->outport[outport_type].link.port_id = PORT_CAP_ISP_YUV_SCALER_IN;
	if (type == PIPELINE_CAPTURE_TYPE) {
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_SLAVE;
	} else if (type == PIPELINE_VIDEO_TYPE)
		cur_node->outport[PORT_VID_OUT].depend_type = PORT_MASTER;
	cur_node++;
	cur_node->id = ISP_YUV_SCALER_CAP_NODE_ID;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link_state = PORT_LINK_IDLE;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.port_id = outport_type;
	switch (type) {
	case PIPELINE_CAPTURE_TYPE:
		cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_VIDEO_TYPE:
		cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_VID_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	case PIPELINE_THUMB_TYPE:
		cur_node->outport[PORT_THUMB_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_THUMB_OUT].link.node_type = CAM_NODE_TYPE_USER;
		break;
	default:
		pr_err("fail to get invalid pipeline_type %s\n", cam_pipeline_name_get(type));
		break;
	}
}

static void campipeline_online_raw_get(struct cam_pipeline_topology *param, uint32_t raw_path_id)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0;
	struct cam_node_topology *cur_node = NULL;
	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
	};

	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_offline_raw2yuv_get(struct cam_pipeline_topology *param, uint32_t path_id, uint32_t pyrdec_support)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		for (i = PORT_OFFLINE_AEM_OUT; i < PORT_DCAM_OFFLINE_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		for (i = PORT_OFFLINE_AEM_OUT; i < PORT_DCAM_OFFLINE_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_onlineraw_2_offlineyuv_get(struct cam_pipeline_topology *param,
		uint32_t path_id, uint32_t pyrdec_support, uint32_t raw_path_id)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0;
	uint32_t dcam_offline_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].dynamic_link_en = 1;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[dcam_online_raw_port_id].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link.port_id = dcam_online_raw_port_id;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_id = DCAM_OFFLINE_NODE_ID;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_onlineraw_2_user_2_offlineyuv_get(struct cam_pipeline_topology *param,
		uint32_t path_id, uint32_t pyrdec_support, uint32_t raw_path_id)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0;
	uint32_t dcam_offline_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].dynamic_link_en = 1;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[dcam_online_raw_port_id].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link.port_id = dcam_online_raw_port_id;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->buf_type = CAM_NODE_BUF_USER;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_onlineraw_2_bpcraw_2_user_2_offlineyuv_get(
		struct cam_pipeline_topology *param, uint32_t raw_path_id, uint32_t raw2yuv_path_id, uint32_t pyrdec_support)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, dcam_offline_bpcraw_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_RAW_OUT].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
	cur_node->outport[PORT_RAW_OUT].link.node_id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link.port_id = PORT_FRAME_CACHE_IN;
	cur_node++;
	cur_node->id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW;
	cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_id = DCAM_OFFLINE_NODE_ID;
	cur_node++;
	dcam_offline_bpcraw_port_id = dcamoffline_pathid_convert_to_portid(raw_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_bpcraw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_offline_bpcraw_port_id].link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(raw2yuv_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_onlineraw_2_user_2_bpcraw_2_user_2_offlineyuv_get(
		struct cam_pipeline_topology *param, uint32_t raw_path_id, uint32_t raw2yuv_path_id, uint32_t pyrdec_support)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, dcam_offline_bpcraw_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_RAW_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_RAW_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_RAW_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_RAW_OUT].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	dcam_offline_bpcraw_port_id = dcamoffline_pathid_convert_to_portid(raw_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_bpcraw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_offline_bpcraw_port_id].link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[dcam_offline_bpcraw_port_id].link.port_id = dcam_offline_bpcraw_port_id;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(raw2yuv_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->buf_type = CAM_NODE_BUF_USER;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_online_normal2yuv_or_raw2user2yuv_get(struct cam_pipeline_topology *param,
		uint32_t path_id, uint32_t pyrdec_support)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;;
	cur_node->outport[PORT_RAW_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_RAW_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_RAW_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_RAW_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_RAW_OUT].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FULL_OUT;
	if (pyrdec_support) {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = ISP_NODE_MODE_CAP_ID;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->buf_type = CAM_NODE_BUF_USER;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_online_vch_raw_get(struct cam_pipeline_topology *param)
{
	int i = 0;
	struct cam_node_topology *cur_node = NULL;

	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
	};

	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_VCH2_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_VCH2_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void campipeline_online_normalzslcapture_or_raw2user2yuv_get(struct cam_pipeline_topology *param,
		uint32_t path_id, uint32_t pyrdec_support)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0;
	struct cam_node_topology *cur_node = NULL;

	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;;
	cur_node->outport[PORT_RAW_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_RAW_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_RAW_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_RAW_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_RAW_OUT].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FRAME_CACHE_IN;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.port_id = PORT_FULL_OUT;
	if (pyrdec_support) {
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_id = ISP_NODE_MODE_CAP_ID;
	}
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->buf_type = CAM_NODE_BUF_USER;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static struct cam_node *campipeline_linked_node_get(
		struct cam_pipeline *pipeline, struct camera_frame *frame)
{
	int i = 0;
	struct cam_node *cur_node = NULL;
	struct cam_linkage link = {0};

	if (!pipeline || !frame) {
		pr_err("fail to get valid param %p %p\n", pipeline, frame);
		return NULL;
	}

	link = frame->link_to;
	if (frame->dump_en) {
		link.node_type = CAM_NODE_TYPE_DUMP;
		link.node_id = frame->dump_node_id;
	}

	if (frame->copy_en) {
		link.node_type = CAM_NODE_TYPE_DATA_COPY;
		link.node_id = frame->copy_node_id;
	}

	if (link.node_type == CAM_NODE_TYPE_USER ||
		link.node_id == CAM_LINK_DEFAULT_NODE_ID)
		return NULL;

	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == link.node_type
				&& cur_node->node_graph->id == link.node_id) {
			return pipeline->node_list[i];
		}
	}

	pr_err("fail to find linked node %d, cnt %d pipeline type %s link node type %s id %d\n",
			i, pipeline->pipeline_graph->node_cnt, cam_pipeline_name_get(pipeline->pipeline_graph->type),
			cam_node_name_get(link.node_type), link.node_id);
	return NULL;
}

static int campipeline_src_cb_proc(enum cam_cb_type type,
		struct camera_frame *pframe, struct cam_pipeline *pipeline)
{
	int ret = 0;
	struct cam_node *link_node = NULL;
	struct cam_node_cfg_param node_param = {0};

	if (!pframe || !pipeline) {
		pr_err("fail to get valid param %p %p\n", pframe, pipeline);
		return -EFAULT;
	}

	link_node = campipeline_linked_node_get(pipeline, pframe);
	if (link_node && (link_node->node_graph->buf_type == CAM_NODE_BUF_KERNEL)) {
		if (pframe->copy_en) {
			node_param.param = pframe;
			link_node->ops.request_proc(link_node, &node_param);
			return 0;
		}
		node_param.param = pframe;
		node_param.port_id = pframe->link_from.port_id;
		link_node->ops.outbuf_back(link_node, &node_param);
	} else
		pipeline->data_cb_func(type, pframe, pipeline->data_cb_handle);

	return ret;
}

static int campipeline_callback(enum cam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct cam_pipeline *pipeline = NULL;
	struct camera_frame *pframe = NULL;
	struct cam_node *link_node = NULL;
	struct cam_node_cfg_param node_param = {0};

	if (!param || !priv_data) {
		pr_err("fail to get valid param %p %p\n", param, priv_data);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)priv_data;
	pframe = (struct camera_frame *)param;
	switch (type) {
	case CAM_CB_DUMP_DATA_DONE:
	case CAM_CB_DCAM_DATA_DONE:
	case CAM_CB_FRAME_CACHE_DATA_DONE:
	case CAM_CB_ISP_RET_PYR_DEC_BUF:
	case CAM_CB_YUV_SCALER_DATA_DONE:
		link_node = campipeline_linked_node_get(pipeline, pframe);
		if (link_node) {
			node_param.param = pframe;
			node_param.port_id = pframe->link_to.port_id;
			ret = link_node->ops.request_proc(link_node, &node_param);
			if (ret) {
				pframe->link_to = pframe->link_from;
				ret = campipeline_src_cb_proc(type, pframe, pipeline);
			}
		} else
			pipeline->data_cb_func(type, param, pipeline->data_cb_handle);
		break;
	case CAM_CB_DCAM_RET_SRC_BUF:
	case CAM_CB_ISP_RET_SRC_BUF:
	case CAM_CB_FRAME_CACHE_RET_SRC_BUF:
	case CAM_CB_PYRDEC_RET_SRC_BUF:
	case CAM_CB_COPY_SRC_BUFFER:
		ret = campipeline_src_cb_proc(type, pframe, pipeline);
		break;
	default :
		pipeline->data_cb_func(type, param, pipeline->data_cb_handle);
		break;
	}

	return ret;
}

static int campipeline_cfg_param(void *handle, enum cam_pipeline_cfg_cmd cmd, void *param)
{
	int ret = 0, i = 0, is_valid_node = 0;
	uint32_t node_cmd = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_pipeline_cfg_param *in_ptr = NULL;
	struct cam_node *cur_node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)handle;
	in_ptr = (struct cam_pipeline_cfg_param *)param;
	if (!IS_VALID_NODE(in_ptr->node_type)) {
		pr_err("fail to get valid node: %d\n", cam_node_name_get(in_ptr->node_type));
		return -EFAULT;
	}

	/* find specific node by node type */
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == in_ptr->node_type) {
			pr_debug("node: %s\n", cam_node_name_get(in_ptr->node_type));
			is_valid_node = 1;
			break;
		}
	}
	if (!is_valid_node || !cur_node) {
		pr_err("fail to get node invalid %d cur_node %px cmd %d need_node %s in pipeline %s\n",
			is_valid_node, cur_node, cmd, cam_node_name_get(in_ptr->node_type),
			cam_pipeline_name_get(pipeline->pipeline_graph->type));
		return -EFAULT;
	}

	switch (cmd) {
	case CAM_PIPELINE_CFG_BUF:
		node_cmd = CAM_NODE_CFG_BUF;
		break;
	case CAM_PIPELINE_CFG_CAP_PARAM:
		node_cmd = CAM_NODE_CFG_CAP_PARAM;
		break;
	case CAM_PIPELINE_CFG_SIZE:
		node_cmd = CAM_NODE_CFG_SIZE;
		break;
	case CAM_PIPELINE_CFG_BASE:
		node_cmd = CAM_NODE_CFG_BASE;
		break;
	case CAM_PIPELINE_CLR_CACHE_BUF:
		node_cmd = CAM_NODE_CLR_CACHE_BUF;
		break;
	case CAM_PIPELINE_DUAL_SYNC_BUF_GET:
		node_cmd = CAM_NODE_DUAL_SYNC_BUF_GET;
		break;
	case CAM_PIPELINE_CFG_BLK_PARAM:
		node_cmd = CAM_NODE_CFG_BLK_PARAM;
		break;
	case CAM_PIPELINE_CFG_UFRAME:
		node_cmd = CAM_NODE_CFG_UFRAME;
		break;
	case CAM_PIPELINE_CFG_STATIS_BUF:
		node_cmd = CAM_NODE_CFG_STATIS;
		break;
	case CAM_PIPILINE_CFG_RESEVER_BUF:
		node_cmd = CAM_NODE_CFG_RESERVE_BUF;
		break;
	case CAM_PIPILINE_CFG_SHARE_BUF:
		node_cmd = CAM_NODE_CFG_SHARE_BUF;
		break;
	case CAM_PIPILINE_CFG_PYRDEC_BUF:
		node_cmd = CAM_NODE_CFG_PYRDEC_BUF;
		break;
	case CAM_PIPELINE_CFG_CTXID:
		node_cmd = CAM_NODE_CFG_CTXID;
		break;
	case CAM_PIPELINE_CFG_LTM_BUF:
		node_cmd = CAM_NODE_CFG_LTM_BUF;
		break;
	case CAM_PIPELINE_CFG_3DNR_MODE:
		node_cmd = CAM_NODE_CFG_3DNR_MODE;
		break;
	case CAM_PIPELINE_CFG_3DNR_BUF:
		node_cmd = CAM_NODE_CFG_3DNR_BUF;
		break;
	case CAM_PIPELINE_CFG_REC_LEYER_NUM:
		node_cmd = CAM_NODE_CFG_REC_LEYER_NUM;
		break;
	case CAM_PIPELINE_CFG_REC_BUF:
		node_cmd = CAM_NODE_CFG_REC_BUF;
		break;
	case CAM_PIPELINE_CFG_GTM:
		node_cmd = CAM_NODE_CFG_GTM;
		break;
	case CAM_PIPELINE_CFG_PARAM_SWITCH:
		node_cmd = CAM_NODE_CFG_PARAM_SWITCH;
		break;
	case CAM_PIPRLINE_CFG_PARAM_Q_CLEAR:
		node_cmd = CAM_NODE_CFG_PARAM_Q_CLEAR;
		break;
	case CAM_PIPELINE_RECT_GET:
		node_cmd = CAM_NODE_RECT_GET;
		break;
	case CAM_PIPELINE_CFG_GTM_LTM:
		node_cmd = CAM_NODE_CFG_BLK_GTM_LTM;
		break;
	case CAM_PIPELINE_RESET:
		node_cmd = CAM_NODE_RESET;
		break;
	case CAM_PIPELINE_CFG_XTM_EN:
		node_cmd = CAM_NODE_CFG_XTM_EN;
		break;
	case CAM_PIPELINE_CFG_FAST_STOP:
		node_cmd = CAM_NODE_CFG_FAST_STOP;
		break;
	case CAM_PIPELINE_CFG_PARAM_FID:
		node_cmd = CAM_NODE_CFG_PARAM_FID;
		break;
	case CAM_PIPELINE_CFG_SUPERZOOM_BUF:
		node_cmd = CAM_NODE_CFG_SUPERZOOM_BUF;
		break;
	case CAM_PIPELINE_RESET_PARAM_PTR:
		node_cmd = CAM_NODE_RESET_PARAM_PTR;
		break;
	default:
		pr_err("fail to support cfg cmd %d\n", cmd);
		ret = -EFAULT;
		goto exit;
	}
	ret = cur_node->ops.cfg_node_param(cur_node, node_cmd, &in_ptr->node_param);

exit:
	return ret;
}

static int campipeline_stream_on(void *handle, void *param)
{
	int ret = 0, i = 0, is_valid_node = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_pipeline_cfg_param *in_ptr = NULL;
	struct cam_node *cur_node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)handle;
	in_ptr = (struct cam_pipeline_cfg_param *)param;
	if (!IS_VALID_NODE(in_ptr->node_type)) {
		pr_err("fail to get valid node type %s\n", cam_node_name_get(in_ptr->node_type));
		return -EFAULT;
	}

	/* find specific node by node type */
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == in_ptr->node_type) {
			pr_debug("node: %s\n", cam_node_name_get(in_ptr->node_type));
			is_valid_node = 1;
			break;
		}
	}
	if (!is_valid_node || !cur_node) {
		pr_err("fail to get node \n");
		return -EFAULT;
	}

	pr_info("node %s stream on\n", cam_node_name_get(in_ptr->node_type));
	ret = cur_node->ops.request_proc(cur_node, &in_ptr->node_param);

	return ret;
}

static int campipeline_stream_off(void *handle, void *param)
{
	int ret = 0, i = 0, is_valid_node = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_pipeline_cfg_param *in_ptr = NULL;
	struct cam_node *cur_node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)handle;
	in_ptr = (struct cam_pipeline_cfg_param *)param;
	if (!IS_VALID_NODE(in_ptr->node_type)) {
		pr_err("fail to get valid node type %d\n", in_ptr->node_type);
		return -EFAULT;
	}

	/* find specific node by node type */
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == in_ptr->node_type) {
			pr_debug("node: %s\n", cam_node_name_get(in_ptr->node_type));
			is_valid_node = 1;
			break;
		}
	}
	if (!is_valid_node || !cur_node) {
		pr_err("fail to get node \n");
		return -EFAULT;
	}

	pr_info("node %d stream off\n", in_ptr->node_type);
	ret = cur_node->ops.stop_proc(cur_node, &in_ptr->node_param);

	return ret;
}

static int campipeline_cfg_shutoff(void *handle, enum cam_node_type node_type,void *param)
{
	uint32_t cmd = 0;
	int ret = 0, i = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_node *cur_node = NULL;
	struct cam_node_shutoff_ctrl *node_shutoff = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)handle;
	node_shutoff = (struct cam_node_shutoff_ctrl *)param;
	if (!IS_VALID_NODE(node_type)) {
		pr_err("fail to get valid node type %s\n", cam_node_name_get(node_type));
		return -EFAULT;
	}

	/* find specific node by node type */
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == node_type)
			break;
	}

	pr_debug("node %s set shutoff\n", cam_node_name_get(node_type));

	cmd = CAM_NODE_SHUTOFF_CONFIG;
	ret = cur_node->ops.cfg_shutoff(cur_node, cmd, node_shutoff);

	return ret;
}

static uint32_t campipeline_dump_en_cfg_get(uint32_t pipeline_type,
	struct cam_node_topology *node_graph, uint32_t g_dump_en)
{
	int i = 0, j = 0;

	if (node_graph->type == CAM_NODE_TYPE_DUMP)
		return g_dump_en;

	for (i = 0; i < DUMP_NUM_MAX; i++) {
		if (!g_dbg_dump[i].dump_ongoing || (pipeline_type != g_dbg_dump[i].dump_pipeline_type)
			|| (node_graph->type != g_dbg_dump[i].dump_node_type))
			continue;

		switch (node_graph->type) {
		case CAM_NODE_TYPE_DCAM_ONLINE:
			for (j = 0; j < CAM_NODE_PORT_OUT_NUM; j++) {
				if (node_graph->outport[j].link_state != PORT_LINK_NORMAL)
					continue;
				if (node_graph->outport[j].id == g_dbg_dump[i].dump_port_id) {
					node_graph->outport[j].dump_en = g_dbg_dump[i].dump_en;
					g_dump_en = g_dump_en || node_graph->outport[j].dump_en;
				}
			}
			break;
		case CAM_NODE_TYPE_FRAME_CACHE:
		case CAM_NODE_TYPE_PYR_DEC:
		case CAM_NODE_TYPE_DATA_COPY:
			node_graph->dump_en = g_dbg_dump[i].dump_en;
			g_dump_en = g_dump_en || node_graph->dump_en;
			break;
		default:
			break;
		}
	}
	return g_dump_en;
}

int cam_pipeline_static_pipelinelist_get(struct cam_pipeline_topology *param, uint32_t *cnt, void *hw_info)
{
	int ret = 0, i = 0, pipeline_cnt = 0;
	uint32_t raw2yuv_path_id = 0, raw_path_id = 0, pyrdec_support = 0;
	struct cam_hw_info *hw = NULL;
	uint32_t pipeline_type[] = {
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
	};

	if (!param || !hw_info) {
		pr_err("fail to get invalid param ptr\n");
		return -EFAULT;
	}

	hw = (struct cam_hw_info *)hw_info;
	raw_path_id = hw->ip_dcam[0]->dcam_raw_path_id;
	raw2yuv_path_id = hw->ip_dcam[0]->aux_dcam_path;
	pyrdec_support = hw->ip_isp->pyr_dec_support;
	pipeline_cnt = sizeof(pipeline_type) / sizeof(pipeline_type[0]);
	for (i = 0; i < pipeline_cnt; i++, param++) {
		switch (pipeline_type[i]) {
		case CAM_PIPELINE_PREVIEW:
			param->type = CAM_PIPELINE_PREVIEW;
			campipeline_preview_get(param, PIPELINE_PREVIEW_TYPE);
			break;
		case CAM_PIPELINE_VIDEO:
			param->type = CAM_PIPELINE_VIDEO;
			campipeline_preview_get(param, PIPELINE_VIDEO_TYPE);
			break;
		case CAM_PIPELINE_CAPTURE:
			param->type = CAM_PIPELINE_CAPTURE;
			campipeline_capture_get(param, PIPELINE_CAPTURE_TYPE, pyrdec_support);
			break;
		case CAM_PIPELINE_ZSL_CAPTURE:
			param->type = CAM_PIPELINE_ZSL_CAPTURE;
			campipeline_zsl_capture_get(param, PIPELINE_CAPTURE_TYPE, pyrdec_support);
			break;
		case CAM_PIPELINE_THUMBNAIL_PREV:
			param->type = CAM_PIPELINE_THUMBNAIL_PREV;
			campipeline_preview_get(param, PIPELINE_THUMB_TYPE);
			break;
		case CAM_PIPELINE_THUMBNAIL_CAP:
			param->type = CAM_PIPELINE_THUMBNAIL_CAP;
			campipeline_capture_get(param, PIPELINE_THUMB_TYPE, pyrdec_support);
			break;
		case CAM_PIPELINE_VIDEO_CAPTURE:
			param->type = CAM_PIPELINE_VIDEO_CAPTURE;
			campipeline_capture_get(param, PIPELINE_VIDEO_TYPE, pyrdec_support);
			break;
		case CAM_PIPELINE_SENSOR_RAW:
			param->type = CAM_PIPELINE_SENSOR_RAW;
			campipeline_online_raw_get(param, raw_path_id);
			break;
		case CAM_PIPELINE_OFFLINE_RAW2YUV:
			param->type = CAM_PIPELINE_OFFLINE_RAW2YUV;
			campipeline_offline_raw2yuv_get(param, raw2yuv_path_id, pyrdec_support);
			break;
		case CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV:
			param->type = CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV;
			campipeline_onlineraw_2_offlineyuv_get(param, raw2yuv_path_id, pyrdec_support, raw_path_id);
			break;
		case CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV:
			param->type = CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV;
			campipeline_onlineraw_2_user_2_offlineyuv_get(param, raw2yuv_path_id, pyrdec_support, raw_path_id);
			break;
		case CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV:
			param->type = CAM_PIPELINE_ONLINERAW_2_BPCRAW_2_USER_2_OFFLINEYUV;
			campipeline_onlineraw_2_bpcraw_2_user_2_offlineyuv_get(param, raw_path_id, raw2yuv_path_id, pyrdec_support);
			break;
		case CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV:
			param->type = CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV;
			campipeline_onlineraw_2_user_2_bpcraw_2_user_2_offlineyuv_get(param, raw_path_id, raw2yuv_path_id, pyrdec_support);
			break;
		case CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV:
			param->type = CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV;
			campipeline_online_normal2yuv_or_raw2user2yuv_get(param, raw2yuv_path_id, pyrdec_support);
			break;
		case CAM_PIPELINE_VCH_SENSOR_RAW:
			param->type = CAM_PIPELINE_VCH_SENSOR_RAW;
			campipeline_online_vch_raw_get(param);
			break;
		case CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV:
			param->type = CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV;
			campipeline_online_normalzslcapture_or_raw2user2yuv_get(param, raw2yuv_path_id, pyrdec_support);
			break;
		default:
			break;
		}
		(*cnt)++;
	}

	return ret;
}

int cam_pipeline_buffer_alloc(struct cam_pipeline *pipe, struct cam_buf_alloc_desc *param)
{
	int ret = 0, i = 0;

	if (!pipe) {
		pr_err("fail to get pipe\n");
		return -1;
	}
	for (i = 0; i < CAM_PIPELINE_NODE_NUM; i++) {
		if (!pipe->node_list[i])
			break;
		ret = cam_node_buffer_alloc(pipe->node_list[i], param);
		if (ret)
			pr_err("fail to alloc buf for node %s\n", cam_node_name_get(i));
	}
	return ret;
}

void *cam_pipeline_creat(struct cam_pipeline_desc *param)
{
	int i = 0;
	uint32_t g_dump_en = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_node_desc node_desc = {0};

	if (!param) {
		pr_err("fail to get invalid pipeline desc info\n");
		return NULL;
	}

	pipeline = vzalloc(sizeof(struct cam_pipeline));
	if (!pipeline) {
		pr_err("fail to alloc cam pipeline\n");
		return NULL;
	}

	pr_info("pipeline: %s start creat\n", cam_pipeline_name_get(param->pipeline_graph->type));
	pipeline->data_cb_func = param->data_cb_func;
	pipeline->data_cb_handle = param->data_cb_handle;
	pipeline->pipeline_graph = param->pipeline_graph;
	pipeline->ops.cfg_param = campipeline_cfg_param;
	pipeline->ops.streamon = campipeline_stream_on;
	pipeline->ops.streamoff = campipeline_stream_off;
	pipeline->ops.cfg_shutoff = campipeline_cfg_shutoff;

	node_desc.nodes_dev = param->nodes_dev;
	node_desc.data_cb_func = campipeline_callback;
	node_desc.data_cb_handle = pipeline;
	node_desc.dcam_online_desc = &param->dcam_online_desc;
	node_desc.dcam_offline_desc = &param->dcam_offline_desc;
	node_desc.dcam_offline_bpcraw_desc = &param->dcam_offline_bpcraw_desc;
	node_desc.isp_node_description = &param->isp_node_description;
	node_desc.frame_cache_desc = &param->frame_cache_desc;
	node_desc.pyr_dec_desc = &param->pyr_dec_desc;
	node_desc.isp_yuv_scaler_desc = &param->isp_yuv_scaler_desc;
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		node_desc.node_graph = &pipeline->pipeline_graph->nodes[i];
		g_dump_en = campipeline_dump_en_cfg_get(pipeline->pipeline_graph->type, node_desc.node_graph, g_dump_en);
		if (!cam_node_work_state_get(node_desc.node_graph, g_dump_en))
			continue;
		pipeline->node_list[i] = cam_node_creat(&node_desc);
		if (!pipeline->node_list[i]) {
			pr_err("fail to creat node %d\n", cam_node_name_get(i));
			while (i > 0)
				cam_node_destory(pipeline->node_list[i--]);

			vfree(pipeline);
			return NULL;
		}
	}

	return pipeline;
}

void cam_pipeline_destory(struct cam_pipeline *pipeline)
{
	int i = 0;

	if (!pipeline) {
		pr_err("fail to get invalid pipeline ptr\n");
		return;
	}

	for (i = 0; i <= pipeline->pipeline_graph->node_cnt - 1; i++) {
		if (pipeline->node_list[i])
			cam_node_close(pipeline->node_list[i]);
	}

	pr_info("pipeline: %s start destory\n", cam_pipeline_name_get(pipeline->pipeline_graph->type));
	for (i = pipeline->pipeline_graph->node_cnt - 1; i >= 0; i--) {
		if (pipeline->node_list[i])
			cam_node_destory(pipeline->node_list[i]);
		pipeline->node_list[i] = NULL;
	}
	vfree(pipeline);
	pipeline = NULL;
}
