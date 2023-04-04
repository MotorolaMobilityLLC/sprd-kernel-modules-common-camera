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

#include "cam_core.h"
#include "cam_scene.h"

typedef void(*cam_pipeline_get)(struct cam_pipeline_topology *handle, void *param);

enum pipeline_prev_type {
	PIPELINE_PREVIEW_TYPE,
	PIPELINE_CAPTURE_TYPE,
	PIPELINE_VIDEO_TYPE,
	PIPELINE_SCENE_TYPE_MAX,
};

struct cam_pipeline_topology_info {
	char *name;
	enum cam_pipeline_type type;
	enum pipeline_prev_type prev_type;
	cam_pipeline_get base_cfg_func;
	uint32_t need_dcam_online;
	uint32_t need_dcam_offline;
	uint32_t need_dcam_offline_bpc;
	uint32_t need_isp_offline;
	uint32_t need_frame_cache;
	uint32_t need_pyr_dec;
	uint32_t need_yuv_scaler;
};

struct cam_scene_topology_input {
	uint32_t index;
	uint32_t raw_path_id;
	uint32_t raw2yuv_path_id;
	uint32_t pyrdec_support;
	uint32_t pyrrec_support;
	enum pipeline_prev_type prev_type;
};

void camscene_onlineraw_ports_enable(void *module_ptr, int dcam_port_id)
{
	struct camera_module *module = NULL;
	struct cam_pipeline_topology *topology = NULL;
	struct cam_node_topology *dcamonline_node = NULL;

	if (dcam_port_id < 0) {
		pr_warn("not get valid dcam_port_id\n");
		return;
	}

	module = (struct camera_module *)module_ptr;
	topology = &module->static_topology->pipeline_list[CAM_PIPELINE_SENSOR_RAW];
	dcamonline_node = &topology->nodes[0];

	dcamonline_node->outport[dcam_port_id].link_state = PORT_LINK_NORMAL;
	pr_debug("enable port id %d\n", dcam_port_id);
}

void camscene_onlineraw_ports_disable(void *module_ptr, int dcam_port_id)
{
	struct camera_module *module = NULL;
	struct cam_pipeline_topology *topology = NULL;
	struct cam_node_topology *dcamonline_node = NULL;

	if (dcam_port_id < 0) {
		pr_warn("not get valid dcam_port_id\n");
		return;
	}

	module = (struct camera_module *)module_ptr;
	topology = &module->static_topology->pipeline_list[CAM_PIPELINE_SENSOR_RAW];
	dcamonline_node = &topology->nodes[0];

	dcamonline_node->outport[dcam_port_id].link_state = PORT_LINK_IDLE;
	pr_debug("disable port id %d\n", dcam_port_id);
}

static int camscene_cap_info_set(struct camera_module *module, struct dcam_mipi_info *cap_info)
{
	int ret = 0;
	struct camera_uinfo *info = &module->cam_uinfo;
	struct sprd_img_sensor_if *sensor_if = &info->sensor_if;

	cap_info->mode = info->capture_mode;
	cap_info->frm_skip = info->capture_skip;
	cap_info->is_4in1 = info->is_4in1;
	cap_info->dcam_slice_mode = info->dcam_slice_mode;
	cap_info->sensor_if = sensor_if->if_type;
	cap_info->format = sensor_if->img_fmt;
	cap_info->pattern = sensor_if->img_ptn;
	cap_info->frm_deci = sensor_if->frm_deci;
	cap_info->is_cphy = sensor_if->if_spec.mipi.is_cphy;
	if (cap_info->sensor_if == DCAM_CAP_IF_CSI2) {
		cap_info->href = sensor_if->if_spec.mipi.use_href;
		cap_info->data_bits = sensor_if->if_spec.mipi.bits_per_pxl;
	}
	cap_info->cap_size.start_x = info->sn_rect.x;
	cap_info->cap_size.start_y = info->sn_rect.y;
	cap_info->cap_size.size_x = info->sn_rect.w;
	cap_info->cap_size.size_y = info->sn_rect.h;

	return ret;
}

static int camscene_outport_type_get(uint32_t type, uint32_t isp_outport_type)
{
	int outport_type = 0;

	switch (type) {
	case PIPELINE_PREVIEW_TYPE:
		outport_type = isp_outport_type ? PORT_PRE_ISP_YUV_SCALER_OUT : PORT_PRE_OUT;
		break;
	case PIPELINE_CAPTURE_TYPE:
		outport_type = isp_outport_type ? PORT_CAP_ISP_YUV_SCALER_OUT : PORT_CAP_OUT;
		break;
	case PIPELINE_VIDEO_TYPE:
		outport_type = isp_outport_type ? PORT_VID_ISP_YUV_SCALER_OUT : PORT_VID_OUT;
		break;
	default:
		pr_err("fail to get invalid pipeline_prev_type %d\n", type);
		break;
	}

	return outport_type;
}

static void camscene_preview_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0, outport_type = 0;
	uint32_t pyrrec_support = 0;
	enum pipeline_prev_type type = 0;
	struct cam_node_topology *cur_node = NULL;
	struct cam_scene_topology_input *in = NULL;
	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
		CAM_NODE_TYPE_DUMP,
		CAM_NODE_TYPE_REPLACE,
		CAM_NODE_TYPE_DATA_COPY,
		CAM_NODE_TYPE_ISP_OFFLINE,
		CAM_NODE_TYPE_ISP_YUV_SCALER,
	};

	in = (struct cam_scene_topology_input *)input;
	type = in->prev_type;
	pyrrec_support = in->pyrrec_support;
	param->buf_num = CAM_PIPELINE_BUFFER_NUM_NORMAL;
	if (pyrrec_support)
		param->pyr_layer_num = DCAM_PYR_DEC_LAYER_NUM;
	else
		param->pyr_layer_num = 0;
	outport_type = camscene_outport_type_get(type, ISP_OUT);
	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	/* dump en should only com from g_debug_info */
	cur_node->outport[PORT_BIN_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_BIN_OUT].replace_node_id = CAM_REPLACE_NODE_ID_0;
	cur_node->outport[PORT_BIN_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_BIN_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->outport[PORT_BIN_OUT].link.node_id = ISP_NODE_MODE_PRE_ID;
	cur_node->outport[PORT_BIN_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
	for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
		cur_node->outport[i].dump_node_id = CAM_DUMP_NODE_ID_0;
		cur_node->outport[i].link_state = PORT_LINK_NORMAL;
		cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = CAM_REPLACE_NODE_ID_0;
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
	outport_type = camscene_outport_type_get(type, ISP_SCALER_OUT);
	cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
	cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
	cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_USER;

}

static void camscene_onlineyuv_2_user_2_offlineyuv_2_nr_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0, outport_type = 0;
	uint32_t pyrrec_support = 0;
	enum pipeline_prev_type type = 0;
	struct cam_node_topology *cur_node = NULL;
	struct cam_scene_topology_input *in = NULL;
	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
		CAM_NODE_TYPE_DUMP,
		CAM_NODE_TYPE_REPLACE,
		CAM_NODE_TYPE_DATA_COPY,
		CAM_NODE_TYPE_ISP_OFFLINE,
		CAM_NODE_TYPE_ISP_YUV_SCALER,
		CAM_NODE_TYPE_PYR_DEC,
		CAM_NODE_TYPE_DUMP,
		CAM_NODE_TYPE_ISP_OFFLINE,
	};
	in = (struct cam_scene_topology_input *)input;
	type = in->prev_type;
	pyrrec_support = in->pyrrec_support;
	param->buf_num = CAM_PIPELINE_BUFFER_NUM_NORMAL;
	if (pyrrec_support)
		param->pyr_layer_num = DCAM_PYR_DEC_LAYER_NUM;
	else
		param->pyr_layer_num = 0;
	outport_type = camscene_outport_type_get(type, ISP_OUT);
	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	/* dump en should only com from g_debug_info */
	cur_node->outport[PORT_BIN_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_BIN_OUT].replace_node_id = CAM_REPLACE_NODE_ID_0;
	cur_node->outport[PORT_BIN_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_BIN_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->outport[PORT_BIN_OUT].link.node_id = ISP_NODE_MODE_PRE_ID;
	cur_node->outport[PORT_BIN_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
	for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
		cur_node->outport[i].dump_node_id = CAM_DUMP_NODE_ID_0;
		cur_node->outport[i].link_state = PORT_LINK_NORMAL;
		cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = CAM_REPLACE_NODE_ID_0;
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
	default:
		pr_err("fail to get invalid pipeline_type %s\n", type);
		break;
	}

	cur_node++;
	cur_node->id = PYR_DEC_NODE_ID;
	cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
	cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
	/* cfg port link between dcam online & isp offline node */
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_1;
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
	cur_node->outport[PORT_VID_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_VID_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void camscene_capture_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0, outport_type = 0;
	uint32_t pyrdec_support = 0;
	enum pipeline_prev_type type = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	type = in->prev_type;
	pyrdec_support = in->pyrdec_support;
	outport_type = camscene_outport_type_get(type, ISP_OUT);
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL_DEC;
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL;
		param->pyr_layer_num = 0;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].replace_node_id = CAM_REPLACE_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FULL_OUT;
	if (pyrdec_support) {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = PYR_DEC_NODE_ID;
		for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
			cur_node->outport[i].dump_node_id = CAM_DUMP_NODE_ID_0;
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node->outport[PORT_DEC_OUT].replace_node_id = CAM_REPLACE_NODE_ID_1;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_1;
		cur_node++;
		cur_node->id = ISP_NODE_MODE_CAP_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].copy_en = 1;
		cur_node->inport[PORT_ISP_OFFLINE_IN].copy_node_id = CAM_COPY_NODE_ID_0;
	} else {
		cur_node->outport[PORT_FULL_OUT].switch_link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_FULL_OUT].switch_link.node_id = ISP_NODE_MODE_CAP_ID;
		for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
			cur_node->outport[i].dump_node_id = CAM_DUMP_NODE_ID_0;
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_0;
		cur_node++;
		cur_node->id = CAM_COPY_NODE_ID_0;
		cur_node++;
		cur_node->id = ISP_NODE_MODE_CAP_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = PORT_FULL_OUT;
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
	outport_type = camscene_outport_type_get(type, ISP_SCALER_OUT);
	if (type == PIPELINE_CAPTURE_TYPE) {
		cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
		cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_USER;
	} else
		pr_err("fail to get invalid pipeline_prev_type %d\n", type);
}

static void camscene_zsl_capture_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0, outport_type = 0;
	uint32_t pyrdec_support = 0;
	enum pipeline_prev_type type = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	type = in->prev_type;
	pyrdec_support = in->pyrdec_support;
	outport_type = camscene_outport_type_get(type,ISP_OUT);
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_DEC;
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DATA_COPY,
			CAM_NODE_TYPE_ISP_OFFLINE,
			CAM_NODE_TYPE_ISP_YUV_SCALER,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NORMAL;
		param->pyr_layer_num = 0;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].replace_node_id = CAM_REPLACE_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FULL_OUT].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
	cur_node->outport[PORT_FULL_OUT].link.node_id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FULL_OUT].link.port_id = PORT_FRAME_CACHE_IN;
	for (i = PORT_AEM_OUT; i < PORT_DCAM_OUT_MAX; i++) {
		cur_node->outport[i].dump_node_id = CAM_DUMP_NODE_ID_0;
		cur_node->outport[i].link_state = PORT_LINK_NORMAL;
		cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = CAM_REPLACE_NODE_ID_0;
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
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node->outport[PORT_DEC_OUT].replace_node_id = CAM_REPLACE_NODE_ID_1;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_1;
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
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link_state = PORT_LINK_NORMAL;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.node_id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_CAP_ISP_YUV_SCALER_IN].link.port_id = outport_type;
	outport_type = camscene_outport_type_get(type,ISP_SCALER_OUT);
	if (type == PIPELINE_CAPTURE_TYPE) {
		cur_node->outport[outport_type].link_state = PORT_LINK_NORMAL;
		cur_node->outport[outport_type].link.node_type = CAM_NODE_TYPE_USER;
	} else
		pr_err("fail to get invalid pipeline_prev_type %d\n", type);
}

static void camscene_onlineraw_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0, raw_path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;
	uint32_t node_list_type[] = {
		CAM_NODE_TYPE_DCAM_ONLINE,
	};

	in = (struct cam_scene_topology_input *)input;
	raw_path_id = in->raw_path_id;
	cur_node = &param->nodes[0];
	param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
	for (i = 0; i < param->node_cnt; i++, cur_node++)
		cam_node_static_nodelist_get(cur_node, node_list_type[i]);

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_VCH2_OUT].link_state = PORT_LINK_IDLE;
	cur_node->outport[PORT_VCH2_OUT].link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_IDLE;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_USER;
}

static void camscene_onlineraw2offlineyuv_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0, dcam_offline_port_id = 0, raw_path_id = 0, pyrdec_support = 0, path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	raw_path_id = in->raw_path_id;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[dcam_online_raw_port_id].dynamic_link_en = 1;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[dcam_online_raw_port_id].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link.port_id = dcam_online_raw_port_id;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_id = DCAM_OFFLINE_NODE_ID;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
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

static void camscene_online2user2offline_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_online_raw_port_id = 0, dcam_offline_port_id = 0, raw_path_id = 0, pyrdec_support = 0, path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	raw_path_id = in->raw_path_id;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	dcam_online_raw_port_id = dcamonline_pathid_convert_to_portid(raw_path_id);
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[dcam_online_raw_port_id].dynamic_link_en = 1;
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[dcam_online_raw_port_id].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link.port_id = dcam_online_raw_port_id;
	cur_node->outport[dcam_online_raw_port_id].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
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

static void camscene_onlinebpcraw2user2offlineyuv_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, dcam_online_raw_port_id = 0;
	uint32_t raw_path_id = 0, pyrdec_support = 0, raw2yuv_path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	raw2yuv_path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	raw_path_id = in->raw_path_id;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	dcam_online_raw_port_id = dcamoffline_pathid_convert_to_portid(raw_path_id);
	cur_node->outport[dcam_online_raw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_online_raw_port_id].link.node_type = CAM_NODE_TYPE_FRAME_CACHE;
	cur_node->outport[dcam_online_raw_port_id].link.node_id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[dcam_online_raw_port_id].link.port_id = PORT_FRAME_CACHE_IN;
	cur_node++;
	cur_node->id = FRAME_CACHE_CAP_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_FRAME_CACHE_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_FRAME_CACHE_OUT].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(raw2yuv_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
	} else {
		cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->buf_type = CAM_NODE_BUF_USER;
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

static void camscene_online2user2bpc2user2offline_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, dcam_offline_bpcraw_port_id = 0;
	uint32_t raw_path_id = 0, pyrdec_support = 0, raw2yuv_path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	raw2yuv_path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	raw_path_id = in->raw_path_id;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL_DEC;
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL;
		param->pyr_layer_num = 0;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	/* cfg port link between dcam online & user node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_RAW_OUT].dynamic_link_en = 1;
	cur_node->outport[PORT_RAW_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_RAW_OUT].link.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	cur_node->outport[PORT_RAW_OUT].link.node_id = DCAM_ONLINE_PRE_NODE_ID;
	cur_node->outport[PORT_RAW_OUT].link.port_id = PORT_RAW_OUT;
	cur_node->outport[PORT_RAW_OUT].switch_link.node_type = CAM_NODE_TYPE_USER;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	dcam_offline_bpcraw_port_id = dcamoffline_pathid_convert_to_portid(raw_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_bpcraw_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[dcam_offline_bpcraw_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_offline_bpcraw_port_id].link.node_type = CAM_NODE_TYPE_USER;
	cur_node->outport[dcam_offline_bpcraw_port_id].link.port_id = dcam_offline_bpcraw_port_id;
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_1;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(raw2yuv_path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_2;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_3;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_3;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
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

static void camscene_online_normal_or_raw2user2yuv_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, pyrdec_support = 0, path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL_DEC;
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_REPLACE,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->buf_num = CAM_PIPELINE_BUFFER_NUM_NONZSL;
		param->pyr_layer_num = 0;
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
	cur_node->outport[PORT_FULL_OUT].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[PORT_FULL_OUT].replace_node_id = CAM_REPLACE_NODE_ID_0;
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
	cur_node->id = CAM_REPLACE_NODE_ID_0;
	cur_node++;
	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[dcam_offline_port_id].replace_node_id = CAM_REPLACE_NODE_ID_1;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_1;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].replace_node_id = CAM_REPLACE_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_2;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = CAM_REPLACE_NODE_ID_1;
	}
	cur_node++;
	cur_node->id = ISP_NODE_MODE_CAP_ID;
	cur_node->inport[PORT_ISP_OFFLINE_IN].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = PYR_DEC_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = dcam_offline_port_id;
	} else {
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.node_id = DCAM_OFFLINE_NODE_ID;
		cur_node->inport[PORT_ISP_OFFLINE_IN].link.port_id = PORT_DEC_OUT;
	}
	cur_node->outport[PORT_CAP_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_CAP_OUT].link.node_type = CAM_NODE_TYPE_USER;
}

static void camscene_onlinezsl_or_raw2user2yuv_pipeline_get(struct cam_pipeline_topology *param, void * input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, pyrdec_support = 0, path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_ONLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_FRAME_CACHE,
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
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
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
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

static void camscene_offlineraw2yuv_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, path_id = 0, pyrdec_support = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	/* cfg port link between dcam online & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_0;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		for (i = PORT_OFFLINE_AEM_OUT; i < PORT_DCAM_OFFLINE_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_1;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		for (i = PORT_OFFLINE_AEM_OUT; i < PORT_DCAM_OFFLINE_OUT_MAX; i++) {
			cur_node->outport[i].link_state = PORT_LINK_NORMAL;
			cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
		}
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_0;
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

static void camscene_offlinefrgb2yuv_pipeline_get(struct cam_pipeline_topology *param, void *input)
{
	int i = 0;
	uint32_t dcam_offline_port_id = 0, pyrdec_support = 0, path_id = 0;
	struct cam_scene_topology_input *in = NULL;
	struct cam_node_topology *cur_node = NULL;

	in = (struct cam_scene_topology_input *)input;
	path_id = in->raw2yuv_path_id;
	pyrdec_support = in->pyrdec_support;
	cur_node = &param->nodes[0];
	if (pyrdec_support) {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_PYR_DEC,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	} else {
		uint32_t node_list_type[] = {
			CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV,
			CAM_NODE_TYPE_DUMP,
			CAM_NODE_TYPE_ISP_OFFLINE,
		};
		param->node_cnt = sizeof(node_list_type) / sizeof(node_list_type[0]);
		param->pyr_layer_num = 0;
		for (i = 0; i < param->node_cnt; i++, cur_node++)
			cam_node_static_nodelist_get(cur_node, node_list_type[i]);
	}

	dcam_offline_port_id = dcamoffline_pathid_convert_to_portid(path_id);
	/* cfg port link between dcam offline & isp offline node */
	cur_node = &param->nodes[0];
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV;
	cur_node->outport[dcam_offline_port_id].link.node_id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].link.port_id = PORT_DCAM_OFFLINE_IN;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_0;
	for (i = PORT_OFFLINE_AEM_OUT; i < PORT_OFFLINE_GTM_HIST_OUT; i++) {
		cur_node->outport[i].link_state = PORT_LINK_NORMAL;
		cur_node->outport[i].link.node_type = CAM_NODE_TYPE_USER;
	}
	cur_node++;
	cur_node->id = CAM_DUMP_NODE_ID_0;
	cur_node++;
	cur_node->id = DCAM_OFFLINE_NODE_ID;
	cur_node->outport[dcam_offline_port_id].link_state = PORT_LINK_NORMAL;
	cur_node->outport[dcam_offline_port_id].dump_node_id = CAM_DUMP_NODE_ID_1;
	cur_node->outport[PORT_OFFLINE_GTM_HIST_OUT].link_state = PORT_LINK_NORMAL;
	cur_node->outport[PORT_OFFLINE_GTM_HIST_OUT].link.node_type = CAM_NODE_TYPE_USER;
	if (pyrdec_support) {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_PYR_DEC;
		cur_node->outport[dcam_offline_port_id].link.node_id = PYR_DEC_NODE_ID;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
		cur_node++;
		cur_node->id = PYR_DEC_NODE_ID;
		cur_node->outport[PORT_DEC_OUT].dump_node_id = CAM_DUMP_NODE_ID_2;
		cur_node->outport[PORT_DEC_OUT].link_state = PORT_LINK_NORMAL;
		cur_node->outport[PORT_DEC_OUT].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[PORT_DEC_OUT].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[PORT_DEC_OUT].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_2;
	} else {
		cur_node->outport[dcam_offline_port_id].link.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		cur_node->outport[dcam_offline_port_id].link.node_id = ISP_NODE_MODE_CAP_ID;
		cur_node->outport[dcam_offline_port_id].link.port_id = PORT_ISP_OFFLINE_IN;
		cur_node++;
		cur_node->id = CAM_DUMP_NODE_ID_1;
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

static int camscene_topology_creat(struct cam_pipeline_topology *param, struct cam_scene_topology_input *in)
{
	int ret = 0, index = 0;
	struct cam_pipeline_topology_info pipeline[CAM_PIPELINE_TYPE_MAX] = {
		[CAM_PIPELINE_PREVIEW] = {
			.name = "CAM_PIPELINE_PREVIEW", .type = CAM_PIPELINE_PREVIEW,
			.prev_type = PIPELINE_PREVIEW_TYPE, .base_cfg_func = camscene_preview_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 0, .need_yuv_scaler = 1,
		},
		[CAM_PIPELINE_VIDEO] = {
			.name = "CAM_PIPELINE_VIDEO", .type = CAM_PIPELINE_VIDEO,
			.prev_type = PIPELINE_VIDEO_TYPE, .base_cfg_func = camscene_preview_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 0, .need_yuv_scaler = 1,
		},
		[CAM_PIPELINE_CAPTURE] = {
			.name = "CAM_PIPELINE_CAPTURE", .type = CAM_PIPELINE_CAPTURE,
			.prev_type = PIPELINE_CAPTURE_TYPE, .base_cfg_func = camscene_capture_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 1,
		},
		[CAM_PIPELINE_ZSL_CAPTURE] = {
			.name = "CAM_PIPELINE_ZSL_CAPTURE", .type = CAM_PIPELINE_ZSL_CAPTURE,
			.prev_type = PIPELINE_CAPTURE_TYPE, .base_cfg_func = camscene_zsl_capture_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 1, .need_pyr_dec = 1, .need_yuv_scaler = 1,
		},
		[CAM_PIPELINE_SENSOR_RAW] = {
			.name = "CAM_PIPELINE_SENSOR_RAW", .type = CAM_PIPELINE_SENSOR_RAW,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = camscene_onlineraw_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 0, .need_frame_cache = 0, .need_pyr_dec = 0, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_SCALER_YUV] = {
			.name = "CAM_PIPELINE_SCALER_YUV", .type = CAM_PIPELINE_SCALER_YUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = NULL,
			.need_dcam_online = 0, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 0, .need_frame_cache = 0, .need_pyr_dec = 0, .need_yuv_scaler = 1,
		},
		[CAM_PIPELINE_OFFLINE_RAW2YUV] = {
			.name = "CAM_PIPELINE_OFFLINE_RAW2YUV", .type = CAM_PIPELINE_OFFLINE_RAW2YUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = camscene_offlineraw2yuv_pipeline_get,
			.need_dcam_online = 0, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV] = {
			.name = "CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV", .type = CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = camscene_onlineraw2offlineyuv_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV] = {
			.name = "CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV", .type = CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = camscene_online2user2offline_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINEBPCRAW_2_USER_2_OFFLINEYUV] = {
			.name = "CAM_PIPELINE_ONLINEBPCRAW_2_USER_2_OFFLINEYUV",
			.type = CAM_PIPELINE_ONLINEBPCRAW_2_USER_2_OFFLINEYUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX, .base_cfg_func = camscene_onlinebpcraw2user2offlineyuv_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 1, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV] = {
			.name = "CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV",
			.type = CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX,
			.base_cfg_func = camscene_online2user2bpc2user2offline_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 1,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV] = {
			.name = "CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV",
			.type = CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX,
			.base_cfg_func = camscene_online_normal_or_raw2user2yuv_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV] = {
			.name = "CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV",
			.type = CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX,
			.base_cfg_func = camscene_onlinezsl_or_raw2user2yuv_pipeline_get,
			.need_dcam_online = 1, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 1, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV] = {
			.name = "CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV",
			.type = CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV,
			.prev_type = PIPELINE_SCENE_TYPE_MAX,
			.base_cfg_func = camscene_offlinefrgb2yuv_pipeline_get,
			.need_dcam_online = 0, .need_dcam_offline = 1, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 0,
		},
		[CAM_PIPELINE_ONLINEYUV_2_USER_2_OFFLINEYUV_2_NR] = {
			.name = "CAM_PIPELINE_ONLINEYUV_2_USER_2_OFFLINEYUV_2_NR", .type = CAM_PIPELINE_ONLINEYUV_2_USER_2_OFFLINEYUV_2_NR,
			.prev_type = PIPELINE_PREVIEW_TYPE, .base_cfg_func = camscene_onlineyuv_2_user_2_offlineyuv_2_nr_get,
			.need_dcam_online = 1, .need_dcam_offline = 0, .need_dcam_offline_bpc = 0,
			.need_isp_offline = 1, .need_frame_cache = 0, .need_pyr_dec = 1, .need_yuv_scaler = 1,
		},
	};

	index = in->index;
	param->type = pipeline[index].type;
	param->name = pipeline[index].name;
	param->need_dcam_online = pipeline[index].need_dcam_online;
	param->need_dcam_offline = pipeline[index].need_dcam_offline;
	param->need_dcam_offline_bpc = pipeline[index].need_dcam_offline_bpc;
	param->need_isp_offline = pipeline[index].need_isp_offline;
	param->need_frame_cache = pipeline[index].need_frame_cache;
	param->need_pyr_dec = pipeline[index].need_pyr_dec;
	param->need_yuv_scaler = pipeline[index].need_yuv_scaler;
	in->prev_type = pipeline[index].prev_type;
	if (pipeline[index].base_cfg_func)
		pipeline[index].base_cfg_func(param, in);

	return ret;
}

static int camscene_static_pipelinelist_get(struct cam_pipeline_topology *param, uint32_t *cnt, void *hw_info)
{
	int ret = 0, i = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_scene_topology_input in = {0};

	if (!param || !hw_info) {
		pr_err("fail to get invalid param ptr\n");
		return -EFAULT;
	}

	hw = (struct cam_hw_info *)hw_info;
	in.raw_path_id = hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id;
	in.raw2yuv_path_id = hw->ip_dcam[0]->dcamhw_abt->aux_dcam_path;
	in.pyrdec_support = hw->ip_isp->isphw_abt->pyr_dec_support;
	in.pyrrec_support = hw->ip_isp->isphw_abt->pyr_rec_support;
	for (i = 0; i < CAM_PIPELINE_TYPE_MAX; i++, param++) {
		in.index = i;
		camscene_topology_creat(param, &in);
		(*cnt)++;
	}

	return ret;
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
	ret = camscene_static_pipelinelist_get(cur_pipeline, &param->pipeline_list_cnt, hw_info);
	if (ret) {
		pr_err("fail to get pipelinelist cnt %d\n", param->pipeline_list_cnt);
		return -EFAULT;
	}

	pr_debug("default pipelinelist cnt %d\n", param->pipeline_list_cnt);
	if (param->pipeline_list_cnt > CAM_SCENE_PIPELINE_NUM)
		pr_warn("warning: pipeline list cnt need to enlarge\n");

	return ret;
}

int cam_scene_reserved_buf_cfg(enum reserved_buf_cb_type type, void *param, void *priv_data)
{
	int ret = 0, j = 0;
	struct cam_frame *pframe = NULL, *newfrm = NULL;
	struct cam_frame **frame = NULL;
	struct camera_module *module = NULL;
	struct cam_buf_pool_id pool_id = {0};

	if (!priv_data) {
		pr_err("fail to get valid param %p\n", priv_data);
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	pool_id.reserved_pool_id = module->reserved_pool_id;
	switch (type) {
	case RESERVED_BUF_GET_CB:
		frame = (struct cam_frame **)param;
		do {
			*frame = cam_buf_manager_buf_dequeue(&pool_id, NULL, (void *)module->grp->global_buf_manager);
			if (*frame == NULL) {
				pframe = module->res_frame;
				while (j < CAM_RESERVE_BUF_Q_LEN) {
					newfrm = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
					if (newfrm) {
						newfrm->common.is_reserved = CAM_RESERVED_BUFFER_COPY;
						newfrm->common.channel_id = pframe->common.channel_id;
						newfrm->common.user_fid = pframe->common.user_fid;
						memcpy(&newfrm->common.buf, &pframe->common.buf, sizeof(struct camera_buf));
						newfrm->common.buf.type = CAM_BUF_NONE;
						cam_buf_manager_buf_enqueue(&pool_id, newfrm, NULL, (void *)module->grp->global_buf_manager);
						j++;
					}
				}
			}
		} while (*frame == NULL);

		pr_debug("Done. cam %d get reserved_pool_id %d frame %p buf_info %p\n", module->idx, pool_id.reserved_pool_id, *frame, (*frame)->common.buf);
		break;
	case RESERVED_BUF_SET_CB:
		pframe = (struct cam_frame *)param;
		if (pframe->common.is_reserved) {
			pframe->common.priv_data = NULL;
			ret = cam_buf_manager_buf_enqueue(&pool_id, pframe, NULL, (void *)module->grp->global_buf_manager);
			pr_debug("cam %d dcam %d set reserved_pool_id %d frame id %d\n", module->idx, pool_id.reserved_pool_id, pframe->common.fid);
		}
		break;
	default:
		pr_err("fail to get invalid %d\n", type);
		break;
	}

	return ret;
}

uint32_t cam_scene_dcamonline_buffers_alloc_num(void *channel_ptr, void *module_ptr, uint32_t pipeline_type)
{
	uint32_t num = 0;
	struct channel_context *channel = NULL;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	grp = module->grp;

	num = module->static_topology->pipeline_list[pipeline_type].buf_num;

	if (channel->ch_id == CAM_CH_CAP) {
		channel->zsl_skip_num = module->cam_uinfo.zsk_skip_num;
		channel->zsl_buffer_num = module->cam_uinfo.zsl_num;
		num += channel->zsl_buffer_num;
	}

	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_dual)
		num = CAM_PIPELINE_BUFFER_NUM_DUAL;

	if (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.is_dual && module->master_flag) /*for dual hdr*/
		num += 3;

	if (module->cam_uinfo.dcam_slice_mode
		&& channel->ch_id == CAM_CH_CAP &&
		module->channel[CAM_CH_PRE].enable == 0 &&
		module->channel[CAM_CH_VID].enable == 0)
		num = 1;

	if (module->cam_uinfo.is_4in1)
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && (grp->is_mul_buf_share && atomic_inc_return(&grp->mul_buf_alloced) > 1))
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && ((module->cam_uinfo.param_frame_sync ||
		module->cam_uinfo.alg_type != ALG_TYPE_CAP_AI_SFNR) && module->cam_uinfo.is_raw_alg))
		num = 0;

	/* extend buffer queue for slow motion */
	if (channel->ch_uinfo.is_high_fps)
		num = channel->ch_uinfo.high_fps_skip_num * 4;

	if (module->cam_uinfo.need_dcam_raw && channel->ch_id == CAM_CH_PRE)
		num += module->cam_uinfo.opt_buffer_num;

	return num;
}

uint32_t cam_scene_dcamoffline_buffers_alloc_num(void *channel_ptr, void *module_ptr)
{
	uint32_t num = 1;
	struct channel_context *channel = NULL;
	struct camera_module *module = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;

	if (module->cam_uinfo.alg_type == ALG_TYPE_CAP_AI_SFNR)
		num = 0;

	if (channel->ch_id == CAM_CH_CAP && ((module->cam_uinfo.param_frame_sync ||
		module->cam_uinfo.alg_type != ALG_TYPE_CAP_AI_SFNR) && module->cam_uinfo.is_raw_alg))
		num = 0;

	return num;
}

int cam_scene_dcamonline_desc_get(void *module_ptr, void *channel_ptr, uint32_t pipeline_type,
		struct dcam_online_node_desc *dcam_online_desc)
{
	int ret = 0, i = 0;
	uint32_t rawpath_id = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_port_topology *outport_graph = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);
	dcam_online_desc->blk_pm = &channel->blk_pm;
	dcam_online_desc->buf_manager_handle = module->grp->global_buf_manager;
	dcam_online_desc->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	dcam_online_desc->resbuf_cb_data = module;
	camscene_cap_info_set(module, &dcam_online_desc->cap_info);
	dcam_online_desc->is_4in1 = module->cam_uinfo.is_4in1;
	dcam_online_desc->offline = 0;
	dcam_online_desc->slowmotion_count = channel->ch_uinfo.high_fps_skip_num;
	dcam_online_desc->enable_3dnr = (module->auto_3dnr | channel->uinfo_3dnr);
	dcam_online_desc->dev = module->dcam_dev_handle;
	dcam_online_desc->endian = ENDIAN_LITTLE;
	dcam_online_desc->pattern = module->cam_uinfo.sensor_if.img_ptn;
	if ((module->grp->hw_info->prj_id == SHARKL3) && module->cam_uinfo.virtualsensor)
		dcam_online_desc->dcam_idx = 0;
	else
		dcam_online_desc->dcam_idx = module->dcam_idx;
	dcam_online_desc->alg_type = module->cam_uinfo.alg_type;
	dcam_online_desc->param_frame_sync = module->cam_uinfo.param_frame_sync;

	if (module->cam_uinfo.is_pyr_rec && (!channel->ch_uinfo.is_high_fps))
		dcam_online_desc->is_pyr_rec = 1;
	else
		dcam_online_desc->is_pyr_rec = 0;

	for (i = 0; i < PORT_DCAM_OUT_MAX; i++) {
		outport_graph = &module->static_topology->pipeline_list[pipeline_type].nodes[CAM_NODE_TYPE_DCAM_ONLINE].outport[i];
		if (outport_graph->link_state != PORT_LINK_NORMAL)
			continue;
		dcam_online_desc->port_desc[i].raw_src = PROCESS_RAW_SRC_SEL;
		dcam_online_desc->port_desc[i].endian = ENDIAN_LITTLE;
		dcam_online_desc->port_desc[i].bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
		dcam_online_desc->port_desc[i].pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
		dcam_online_desc->port_desc[i].reserved_pool_id = module->reserved_pool_id;
		if (i == PORT_FULL_OUT)
			dcam_online_desc->port_desc[i].share_full_path = module->cam_uinfo.need_share_buf;
		if (i == PORT_BIN_OUT)
			dcam_online_desc->port_desc[i].is_pyr_rec = dcam_online_desc->is_pyr_rec;
		cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[0]);
		dcam_online_desc->port_desc[i].dcam_out_fmt = channel->dcam_out_fmt;
		dcam_online_desc->port_desc[i].compress_en = channel->compress_en;
		if (pipeline_type == CAM_PIPELINE_SENSOR_RAW
			|| pipeline_type == CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV
			|| pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV) {
			dcam_online_desc->port_desc[i].is_raw = 1;
			cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->dcamhw_abt->sensor_raw_fmt);
			dcam_online_desc->port_desc[i].dcam_out_fmt = channel->ch_uinfo.sensor_raw_fmt;
			if (module->cam_uinfo.alg_type != ALG_TYPE_DEFAULT)
				dcam_online_desc->port_desc[i].dcam_out_fmt = CAM_RAW_14;
			if (module->cam_uinfo.need_dcam_raw && hw->ip_dcam[0]->dcamhw_abt->bpc_raw_support) {
				dcam_online_desc->port_desc[i].dcam_out_fmt = channel->ch_uinfo.dcam_raw_fmt;
				dcam_online_desc->port_desc[i].is_raw = 0;
				dcam_online_desc->port_desc[i].raw_src = BPC_RAW_SRC_SEL;
			}
		} else if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV
				|| pipeline_type == CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV
				|| pipeline_type == CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV) {
			if (outport_graph->id == PORT_RAW_OUT) {
				dcam_online_desc->port_desc[i].is_raw = 1;
				dcam_online_desc->port_desc[i].dcam_out_fmt = CAM_RAW_14;
			}
		} else if (pipeline_type == CAM_PIPELINE_ONLINEBPCRAW_2_USER_2_OFFLINEYUV) {
			if (outport_graph->id == PORT_RAW_OUT) {
				dcam_online_desc->port_desc[i].is_raw = 0;
				dcam_online_desc->port_desc[i].dcam_out_fmt = CAM_RAW_14;
				dcam_online_desc->port_desc[i].raw_src = BPC_RAW_SRC_SEL;
			}
		}
	}

	return ret;
}

int cam_scene_dcamoffline_desc_get(void *module_ptr, void *channel_ptr,
	uint32_t pipeline_type, struct dcam_offline_node_desc *dcam_offline_desc)
{
	int ret = 0;
	uint32_t dcam_idx = 0, rawpath_id = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);

	for (dcam_idx = 0; dcam_idx < DCAM_HW_CONTEXT_MAX; dcam_idx++) {
		if (dcam_idx != module->dcam_idx) {
			dcam_offline_desc->dcam_idx = dcam_idx;
			break;
		}
	}
	dcam_offline_desc->dev = module->dcam_dev_handle;
	dcam_offline_desc->buf_manager_handle = module->grp->global_buf_manager;
	dcam_offline_desc->pattern = module->cam_uinfo.sensor_if.img_ptn;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->endian = ENDIAN_LITTLE;
	dcam_offline_desc->port_desc.src_sel = PROCESS_RAW_SRC_SEL;
	cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->dcamhw_abt->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = channel->ch_uinfo.sensor_raw_fmt;
	cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = channel->ch_uinfo.dcam_raw_fmt;
	if (module->cam_uinfo.dcam_slice_mode)
		dcam_offline_desc->port_desc.compress_en = channel->compress_offline;
	if (module->cam_uinfo.dcam_slice_mode && hw->ip_dcam[0]->dcamhw_abt->save_band_for_bigsize)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_PACK_10;
	if (hw->ip_isp->isphw_abt->fetch_raw_support == 0)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YVU420_2FRAME_MIPI;
	channel->dcam_out_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV) {
		if (module->cam_uinfo.alg_type != ALG_TYPE_DEFAULT)
			dcam_offline_desc->fetch_fmt = CAM_RAW_14;
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YVU420_2FRAME_MIPI;
	} else if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV) {
		if (module->cam_uinfo.alg_type != ALG_TYPE_DEFAULT) {
			dcam_offline_desc->fetch_fmt = CAM_RAW_14;
			dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YVU420_2FRAME_MIPI;
		}
		if (module->cam_uinfo.is_4in1)
			dcam_offline_desc->fetch_fmt = channel->ch_uinfo.dcam_raw_fmt;
	}

	if (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV
		|| pipeline_type == CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV
		|| (pipeline_type == CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV && !module->cam_uinfo.is_4in1))
		dcam_offline_desc->buf_type = CAM_NODE_BUF_USER;
	channel->ch_uinfo.dcam_raw_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;

	return ret;
}

int cam_scene_dcamoffline_bpcraw_desc_get(void *module_ptr,
		void *channel_ptr, struct dcam_offline_node_desc *dcam_offline_desc)
{
	int ret = 0;
	uint32_t rawpath_id = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);

	dcam_offline_desc->dev = module->dcam_dev_handle;
	dcam_offline_desc->buf_manager_handle = module->grp->global_buf_manager;
	dcam_offline_desc->dcam_idx = DCAM_HW_CONTEXT_1;
	dcam_offline_desc->pattern = module->cam_uinfo.sensor_if.img_ptn;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->endian = ENDIAN_LITTLE;
	dcam_offline_desc->port_desc.src_sel = BPC_RAW_SRC_SEL;
	cam_valid_fmt_get(&channel->ch_uinfo.sensor_raw_fmt, hw->ip_dcam[0]->dcamhw_abt->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = CAM_RAW_14;
	cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_14;

	return ret;
}

int cam_scene_ispoffline_desc_get(void *module_ptr, void *channel_ptr,
	uint32_t pipeline_type, struct isp_node_desc *isp_node_description)
{
	int ret = 0;
	uint32_t rawpath_id = 0;
	uint32_t format = 0, temp_format = 0;
	struct cam_hw_info *hw = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	hw = module->grp->hw_info;
	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);
	format = module->cam_uinfo.sensor_if.img_fmt;
	isp_node_description->buf_manager_handle = module->grp->global_buf_manager;
	isp_node_description->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_node_description->resbuf_cb_data = module;

	if (format == DCAM_CAP_MODE_YUV)
		isp_node_description->in_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	else {
		isp_node_description->in_fmt = channel->dcam_out_fmt;
		if ((module->cam_uinfo.alg_type != ALG_TYPE_DEFAULT) && (channel->ch_id == CAM_CH_CAP))
			isp_node_description->in_fmt = CAM_YUV420_2FRAME_MIPI;
	}
	isp_node_description->pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
	isp_node_description->store_3dnr_fmt = hw->ip_dcam[0]->dcamhw_abt->store_3dnr_fmt[0];
	temp_format = channel->ch_uinfo.dcam_raw_fmt;
	if (cam_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[0])) {
		if (hw->ip_dcam[0]->dcamhw_abt->save_band_for_bigsize)
			temp_format = CAM_RAW_PACK_10;
	}
	if ((hw->ip_isp->isphw_abt->fetch_raw_support == 0) || (format == DCAM_CAP_MODE_YUV))
		channel->ch_uinfo.dcam_out_fmt = isp_node_description->in_fmt;
	else {
		channel->ch_uinfo.dcam_out_fmt = temp_format;
		isp_node_description->in_fmt = temp_format;
	}

	if (channel->compress_en)
		isp_node_description->fetch_path_sel = ISP_FETCH_PATH_FBD;
	else
		isp_node_description->fetch_path_sel = ISP_FETCH_PATH_NORMAL;
	isp_node_description->nr3_fbc_fbd = channel->compress_3dnr;

	isp_node_description->is_dual = module->cam_uinfo.is_dual;
	isp_node_description->bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;
	isp_node_description->enable_slowmotion = channel->ch_uinfo.is_high_fps;
	isp_node_description->slowmotion_count = channel->ch_uinfo.high_fps_skip_num;
	isp_node_description->slw_state = CAM_SLOWMOTION_OFF;
	isp_node_description->ch_id = channel->ch_id;
	isp_node_description->sn_size = module->cam_uinfo.sn_size;
	isp_node_description->share_buffer = (channel->ch_id == CAM_CH_CAP && module->cam_uinfo.need_share_buf &&
		!module->cam_uinfo.dcam_slice_mode && !module->cam_uinfo.is_4in1);
	if (channel->ch_uinfo.is_high_fps)
		isp_node_description->blkparam_node_num = CAM_SHARED_BUF_NUM / channel->ch_uinfo.high_fps_skip_num;
	else
		isp_node_description->blkparam_node_num = cam_scene_dcamonline_buffers_alloc_num(channel, module, pipeline_type) + 1;

	isp_node_description->mode_3dnr = MODE_3DNR_OFF;
	channel->type_3dnr = CAM_3DNR_OFF;
	if (module->auto_3dnr || channel->uinfo_3dnr) {
		channel->type_3dnr = CAM_3DNR_HW;
		if (channel->uinfo_3dnr) {
			if (channel->ch_id == CAM_CH_CAP)
				isp_node_description->mode_3dnr = MODE_3DNR_CAP;
			else
				isp_node_description->mode_3dnr = MODE_3DNR_PRE;
		}
	}

	isp_node_description->mode_ltm = MODE_LTM_OFF;
	if (module->cam_uinfo.is_rgb_ltm) {
		if (channel->ch_id == CAM_CH_CAP) {
			isp_node_description->mode_ltm = MODE_LTM_CAP;
		} else if (channel->ch_id == CAM_CH_PRE) {
			isp_node_description->mode_ltm = MODE_LTM_PRE;
		}
	}
	channel->mode_ltm = isp_node_description->mode_ltm;

	isp_node_description->mode_gtm = MODE_GTM_OFF;
	if (module->cam_uinfo.is_rgb_gtm) {
		if (channel->ch_id == CAM_CH_CAP) {
			isp_node_description->mode_gtm = MODE_GTM_CAP;
		} else if (channel->ch_id == CAM_CH_PRE) {
			isp_node_description->mode_gtm = MODE_GTM_PRE;
		}
	}
	channel->mode_gtm = isp_node_description->mode_gtm;

	isp_node_description->ltm_rgb = module->cam_uinfo.is_rgb_ltm;
	channel->ltm_rgb = isp_node_description->ltm_rgb;
	isp_node_description->gtm_rgb = module->cam_uinfo.is_rgb_gtm;
	channel->gtm_rgb = isp_node_description->gtm_rgb;
	isp_node_description->is_high_fps = channel->ch_uinfo.is_high_fps;
	isp_node_description->cam_id = module->idx;
	isp_node_description->dev = module->isp_dev_handle;
	isp_node_description->port_desc.out_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	isp_node_description->port_desc.endian = ENDIAN_LITTLE;
	isp_node_description->port_desc.output_size = channel->ch_uinfo.dst_size;
	isp_node_description->port_desc.regular_mode = channel->ch_uinfo.regular_desc.regular_mode;
	isp_node_description->port_desc.resbuf_get_cb = isp_node_description->resbuf_get_cb;
	isp_node_description->port_desc.resbuf_cb_data = isp_node_description->resbuf_cb_data;
	isp_node_description->port_desc.in_fmt = isp_node_description->in_fmt;
	isp_node_description->port_desc.bayer_pattern = isp_node_description->bayer_pattern;
	isp_node_description->port_desc.pyr_out_fmt = isp_node_description->pyr_out_fmt;
	isp_node_description->port_desc.store_3dnr_fmt = isp_node_description->store_3dnr_fmt;
	isp_node_description->port_desc.sn_size = isp_node_description->sn_size;
	isp_node_description->port_desc.is_high_fps = isp_node_description->is_high_fps;
	isp_node_description->port_desc.fetch_path_sel = isp_node_description->fetch_path_sel;
	isp_node_description->port_desc.hw = hw;

	if (channel->ch_uinfo.is_high_fps) {
		isp_node_description->fps960_info.slowmotion_stage_a_num = channel->ch_uinfo.slowmotion_stage_a_num;
		isp_node_description->fps960_info.slowmotion_stage_a_valid_num = channel->ch_uinfo.slowmotion_stage_a_valid_num;
		isp_node_description->fps960_info.slowmotion_stage_b_num = channel->ch_uinfo.slowmotion_stage_b_num;
	}
	return ret;
}

int cam_scene_isp_yuv_scaler_desc_get(void *module_ptr,
		void *channel_ptr, struct isp_yuv_scaler_node_desc *isp_yuv_scaler_desc)
{
	int ret = 0;
	uint32_t uframe_sync = 0;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;

	if (!module_ptr || !channel_ptr || !isp_yuv_scaler_desc) {
		pr_err("fail to get valid inptr %p %p %p\n", module, channel, isp_yuv_scaler_desc);
		return -EINVAL;
	}

	module = (struct camera_module *)module_ptr;
	channel = (struct channel_context *)channel_ptr;
	uframe_sync = channel->ch_id != CAM_CH_CAP;
	if (channel->ch_uinfo.frame_sync_close)
		uframe_sync = 0;
	isp_yuv_scaler_desc->uframe_sync = uframe_sync;
	isp_yuv_scaler_desc->dev = module->isp_dev_handle;
	isp_yuv_scaler_desc->buf_manager_handle = module->grp->global_buf_manager;
	isp_yuv_scaler_desc->resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_yuv_scaler_desc->resbuf_cb_data = module;
	isp_yuv_scaler_desc->ch_id = channel->ch_id;
	isp_yuv_scaler_desc->cam_id = module->idx;
	isp_yuv_scaler_desc->port_desc.resbuf_get_cb = cam_scene_reserved_buf_cfg;
	isp_yuv_scaler_desc->port_desc.resbuf_cb_data = module;
	isp_yuv_scaler_desc->port_desc.in_fmt = CAM_YVU420_2FRAME;
	isp_yuv_scaler_desc->port_desc.out_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	isp_yuv_scaler_desc->port_desc.endian = ENDIAN_LITTLE;
	isp_yuv_scaler_desc->port_desc.output_size = channel->ch_uinfo.dst_size;
	isp_yuv_scaler_desc->port_desc.regular_mode = channel->ch_uinfo.regular_desc.regular_mode;
	isp_yuv_scaler_desc->port_desc.bayer_pattern = module->cam_uinfo.sensor_if.img_ptn;

	return ret;
}
