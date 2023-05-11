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

#include <linux/sched.h>
#include <linux/vmalloc.h>

#include "cam_pipeline.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_PIPELINE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define IS_VALID_NODE(type) ((type) >= CAM_NODE_TYPE_DCAM_ONLINE && (type) < CAM_NODE_TYPE_MAX)

static struct cam_node *campipeline_linked_node_get(
		struct cam_pipeline *pipeline, struct cam_frame *frame)
{
	int i = 0;
	struct cam_node *cur_node = NULL;
	struct cam_port_linkage link = {0};

	if (!pipeline || !frame) {
		pr_err("fail to get valid param %p %p\n", pipeline, frame);
		return NULL;
	}

	link = frame->common.link_to;
	if (frame->common.dump_en) {
		link.node_type = CAM_NODE_TYPE_DUMP;
		link.node_id = frame->common.dump_node_id;
	}

	if (frame->common.replace_en) {
		link.node_type = CAM_NODE_TYPE_REPLACE;
		link.node_id = frame->common.replace_node_id;
	}

	if (frame->common.copy_en) {
		link.node_type = CAM_NODE_TYPE_DATA_COPY;
		link.node_id = frame->common.copy_node_id;
	}

	if (g_pipeline_type != CAM_PIPELINE_TYPE_MAX && frame->common.link_from.node_type == CAM_NODE_TYPE_DCAM_ONLINE) {
		if (pipeline->pipeline_graph->type == g_pipeline_type)
			pipeline->debug_log_switch = 1;
	}

	if (link.node_type == CAM_NODE_TYPE_USER ||
		link.node_id == CAM_LINK_DEFAULT_NODE_ID)
		return NULL;

	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == link.node_type
				&& cur_node->node_graph->id == link.node_id) {
			pr_debug("node: %s\n", cam_node_name_get(link.node_type));
			return pipeline->node_list[i];
		}
	}

	pr_err("fail to find linked node %d, cnt %d pipeline type %s link node type %s id %d\n",
			i, pipeline->pipeline_graph->node_cnt, pipeline->pipeline_graph->name,
			cam_node_name_get(link.node_type), link.node_id);
	return NULL;
}

static int campipeline_src_cb_proc(enum cam_cb_type type,
		struct cam_frame *pframe, struct cam_pipeline *pipeline)
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
		if (pframe->common.copy_en) {
			node_param.param = pframe;
			link_node->ops.request_proc(link_node, &node_param);
			return 0;
		}
		node_param.param = pframe;
		node_param.port_id = pframe->common.link_from.port_id;
		link_node->ops.outbuf_back(link_node, &node_param);
	} else
		pipeline->data_cb_func(type, pframe, pipeline->data_cb_handle);

	return ret;
}

static int campipeline_zoom_callback(void *priv_data, void *param)
{
	int ret = 0;
	uint32_t type = 0;
	struct cam_pipeline *pipeline = NULL;

	if (!priv_data || !param) {
		pr_err("fail to get valid param priv_data:%p, param:%p\n", priv_data, param);
		return -1;
	}

	pipeline = (struct cam_pipeline *)priv_data;
	type = pipeline->pipeline_graph->type;
	ret = pipeline->zoom_cb_func(type, pipeline->data_cb_handle, param);

	return ret;
}

static int campipeline_callback(enum cam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_frame *pframe = NULL;
	struct cam_node *link_node = NULL;
	struct cam_node_cfg_param node_param = {0};

	if (!param || !priv_data) {
		pr_err("fail to get valid param %p %p\n", param, priv_data);
		return -EFAULT;
	}

	pipeline = (struct cam_pipeline *)priv_data;
	pframe = (struct cam_frame *)param;
	switch (type) {
	case CAM_CB_DUMP_DATA_DONE:
	case CAM_CB_REPLACE_DATA_DONE:
	case CAM_CB_DCAM_DATA_DONE:
	case CAM_CB_FRAME_CACHE_DATA_DONE:
	case CAM_CB_ISP_RET_PYR_DEC_BUF:
	case CAM_CB_YUV_SCALER_DATA_DONE:
		link_node = campipeline_linked_node_get(pipeline, pframe);
		if (link_node) {
			if (pframe->common.height > ISP_SLCIE_HEIGHT_MAX
				&& (link_node->node_graph->type == CAM_NODE_TYPE_ISP_OFFLINE)) {
				ret = pipeline->slice_cb_func(pframe, pipeline->data_cb_handle);
			} else {
				node_param.param = pframe;
				node_param.port_id = pframe->common.link_to.port_id;
				ret = link_node->ops.request_proc(link_node, &node_param);
			}
			if (ret) {
				pframe->common.link_to = pframe->common.link_from;
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
	case CAM_CB_ISP_SCALE_RET_ISP_BUF:
		ret = campipeline_src_cb_proc(type, pframe, pipeline);
		break;
	case CAM_CB_DCAM_STATIS_DONE:
		if (pframe->common.dump_en) {
			link_node = campipeline_linked_node_get(pipeline, pframe);
			if (link_node) {
				node_param.param = pframe;
				ret = link_node->ops.request_proc(link_node, &node_param);
				if (ret)
					pipeline->data_cb_func(type, param, pipeline->data_cb_handle);
			} else
				pipeline->data_cb_func(type, param, pipeline->data_cb_handle);
		} else
			pipeline->data_cb_func(type, param, pipeline->data_cb_handle);
		break;
	default:
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
		pr_err("fail to get valid node: %s\n", cam_node_name_get(in_ptr->node_type));
		return -EFAULT;
	}

	/* find specific node by node type */
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		cur_node = pipeline->node_list[i];
		if (cur_node && cur_node->node_graph->type == in_ptr->node_type
			&& cur_node->node_graph->id == in_ptr->node_id) {
			pr_debug("node: %s\n", cam_node_name_get(in_ptr->node_type));
			is_valid_node = 1;
			break;
		}
	}
	if (!is_valid_node || !cur_node) {
		pr_err("fail to get node invalid %d cur_node %px cmd %d need_node %s in pipeline %s\n",
			is_valid_node, cur_node, cmd, cam_node_name_get(in_ptr->node_type),
			pipeline->pipeline_graph->name);
		return -EFAULT;
	}

	switch (cmd) {
	case CAM_PIPELINE_CFG_BUF:
		node_cmd = CAM_NODE_CFG_BUF;
		break;
	case CAM_PIPELINE_CFG_BUF_CLR:
		node_cmd = CAM_NODE_CFG_BUF_CLR;
		break;
	case CAM_PIPELINE_CFG_CAP_PARAM:
		node_cmd = CAM_NODE_CFG_CAP_PARAM;
		break;
	case CAM_PIPELINE_CFG_FMT:
		node_cmd = CAM_NODE_CFG_FMT;
		break;
	case CAM_PIPELINE_CFG_ZOOM:
		node_cmd = CAM_NODE_CFG_ZOOM;
		break;
	case CAM_PIPELINE_CFG_BASE:
		node_cmd = CAM_NODE_CFG_BASE;
		break;
	case CAM_PIPELINE_CFG_CLR_CACHE_BUF:
		node_cmd = CAM_NODE_CFG_CLR_CACHE_BUF;
		break;
	case CAM_PIPELINE_CFG_DUAL_SYNC_BUF_GET:
		node_cmd = CAM_NODE_CFG_DUAL_SYNC_BUF_GET;
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
	case CAM_PIPELINE_CFG_CTXID:
		node_cmd = CAM_NODE_CFG_CTXID;
		break;
	case CAM_PIPELINE_CFG_3DNR_MODE:
		node_cmd = CAM_NODE_CFG_3DNR_MODE;
		break;
	case CAM_PIPELINE_CFG_GTM:
		node_cmd = CAM_NODE_CFG_GTM;
		break;
	case CAM_PIPELINE_CFG_PARAM_SWITCH:
		node_cmd = CAM_NODE_CFG_PARAM_SWITCH;
		break;
	case CAM_PIPELINE_CFG_POSTPROC_PARAM:
		node_cmd = CAM_NODE_CFG_POSTPROC_PARAM;
		break;
	case CAM_PIPRLINE_CFG_PARAM_Q_CLEAR:
		node_cmd = CAM_NODE_CFG_PARAM_Q_CLEAR;
		break;
	case CAM_PIPELINE_CFG_RECT_GET:
		node_cmd = CAM_NODE_CFG_RECT_GET;
		break;
	case CAM_PIPELINE_CFG_GTM_LTM:
		node_cmd = CAM_NODE_CFG_BLK_GTM_LTM;
		break;
	case CAM_PIPELINE_CFG_RESET:
		node_cmd = CAM_NODE_CFG_RESET;
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
	case CAM_PIPELINE_CFG_RESET_PARAM_PTR:
		node_cmd = CAM_NODE_CFG_RESET_PARAM_PTR;
		break;
	case CAM_PIPELINE_CFG_PRE_RAW_FLAG:
		node_cmd = CAM_NODE_CFG_PRE_RAW_FLAG;
		break;
	case CAM_PIPELINE_CFG_OPT_SCENE_SWITCH:
		node_cmd = CAM_NODE_CFG_OPT_SCENE_SWITCH;
		break;
	case CAM_PIPELINE_GET_CAP_FRAME:
		node_cmd = CAM_NODE_CFG_GET_CAP_FRAME;
		break;
	case CAM_PIPELINE_CFG_ICAP_SCENE_SWITCH:
		node_cmd = CAM_NODE_CFG_ICAP_SCENE_SWITCH;
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
		pr_err("fail to get node:is valid node:%d, cur_node:0x%p \n", is_valid_node, cur_node);
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
	int ret = 0, i = 0, is_valid_node = 0;
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
		if (cur_node && cur_node->node_graph->type == node_type) {
			pr_debug("node: %s\n", cam_node_name_get(node_type));
			is_valid_node = 1;
			break;
		}
	}
	if (!is_valid_node || !cur_node) {
		pr_err("fail to get node \n");
		return -EFAULT;
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
		case CAM_NODE_TYPE_DCAM_OFFLINE:
		case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		case CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB:
		case CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV:
		case CAM_NODE_TYPE_PYR_DEC:
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

static uint32_t campipeline_replace_en_cfg_get(uint32_t pipeline_type,
	struct cam_node_topology *node_graph, uint32_t g_replace_en)
{
	int j = 0;

	if (node_graph->type == CAM_NODE_TYPE_REPLACE)
		return g_replace_en;

	if (!g_dbg_replace.replace_ongoing || (pipeline_type != g_dbg_replace.replace_pipeline_type)
		|| (node_graph->type != g_dbg_replace.replace_node_type))
		return g_replace_en;

	switch (node_graph->type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		for (j = 0; j < CAM_NODE_PORT_OUT_NUM; j++) {
			if (node_graph->outport[j].link_state != PORT_LINK_NORMAL)
				continue;
			if (node_graph->outport[j].id == g_dbg_replace.replace_port_id) {
				node_graph->outport[j].replace_en = g_dbg_replace.replace_en;
				g_replace_en = g_replace_en || node_graph->outport[j].replace_en;
			}
		}
		break;
	case CAM_NODE_TYPE_FRAME_CACHE:
	case CAM_NODE_TYPE_PYR_DEC:
	case CAM_NODE_TYPE_DATA_COPY:
		node_graph->replace_en = g_dbg_replace.replace_en;
		g_replace_en = g_replace_en || node_graph->replace_en;
		break;
	default:
		pr_warn("warning: not support node_type %s\n",cam_node_name_get(node_graph->type));
		break;
	}
	return g_replace_en;
}

int cam_pipeline_buffer_alloc(struct cam_pipeline *pipe, struct cam_buf_alloc_desc *param)
{
	int ret = 0, i = 0;

	if (!pipe || !param) {
		pr_err("fail to get valid param %p %p\n", pipe, param);
		return -1;
	}

	for (i = CAM_PIPELINE_NODE_NUM - 1; i >= 0; i--) {
		if (!pipe->node_list[i])
			continue;
		ret = cam_node_buffer_alloc(pipe->node_list[i], param);
		if (ret)
			pr_err("fail to alloc buf for node %s\n", cam_node_name_get(i));
	}
	return ret;
}

void *cam_pipeline_creat(struct cam_pipeline_desc *param)
{
	int i = 0;
	uint32_t g_dump_en = 0, g_replace_en = 0;
	struct cam_pipeline *pipeline = NULL;
	struct cam_node_desc node_desc = {0};

	if (!param) {
		pr_err("fail to get invalid pipeline desc info\n");
		return NULL;
	}

	pipeline = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_pipeline));
	if (!pipeline) {
		pr_err("fail to alloc cam pipeline\n");
		return NULL;
	}

	pr_info("pipeline: %s start creat\n", param->pipeline_graph->name);
	pipeline->zoom_cb_func = param->zoom_cb_func;
	pipeline->slice_cb_func = param->slice_cb_func;
	pipeline->data_cb_func = param->data_cb_func;
	pipeline->data_cb_handle = param->data_cb_handle;
	pipeline->buf_manager_handle = param->buf_manager_handle;
	pipeline->pipeline_graph = param->pipeline_graph;
	pipeline->ops.cfg_param = campipeline_cfg_param;
	pipeline->ops.streamon = campipeline_stream_on;
	pipeline->ops.streamoff = campipeline_stream_off;
	pipeline->ops.cfg_shutoff = campipeline_cfg_shutoff;

	node_desc.nodes_dev = param->nodes_dev;
	node_desc.zoom_cb_func = campipeline_zoom_callback;
	node_desc.data_cb_func = campipeline_callback;
	node_desc.data_cb_handle = pipeline;
	node_desc.buf_manager_handle = pipeline->buf_manager_handle;
	node_desc.dcam_online_desc = &param->dcam_online_desc;
	node_desc.dcam_offline_desc = &param->dcam_offline_desc;
	node_desc.dcam_offline_bpcraw_desc = &param->dcam_offline_bpcraw_desc;
	node_desc.dcam_offline_raw2frgb_desc = &param->dcam_offline_raw2frgb_desc;
	node_desc.dcam_offline_frgb2yuv_desc = &param->dcam_offline_frgb2yuv_desc;
	node_desc.dcam_fetch_desc = &param->dcam_fetch_desc;
	node_desc.isp_node_description = &param->isp_node_description;
	node_desc.frame_cache_desc = &param->frame_cache_desc;
	node_desc.pyr_dec_desc = &param->pyr_dec_desc;
	node_desc.isp_yuv_scaler_desc = &param->isp_yuv_scaler_desc;
	for (i = 0; i < pipeline->pipeline_graph->node_cnt; i++) {
		node_desc.node_graph = &pipeline->pipeline_graph->nodes[i];
		g_dump_en = campipeline_dump_en_cfg_get(pipeline->pipeline_graph->type, node_desc.node_graph, g_dump_en);
		g_replace_en = campipeline_replace_en_cfg_get(pipeline->pipeline_graph->type, node_desc.node_graph, g_replace_en);
		if (!cam_node_work_state_get(node_desc.node_graph, g_dump_en, &g_replace_en))
			continue;
		pipeline->node_list[i] = cam_node_creat(&node_desc);
		if (!pipeline->node_list[i]) {
			pr_err("fail to creat node %s\n", cam_node_name_get(node_desc.node_graph->type));
			while (i > 0)
				cam_node_destory(pipeline->node_list[--i]);

			cam_buf_kernel_sys_vfree(pipeline);
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

	pr_info("pipeline: %s start destory\n", pipeline->pipeline_graph->name);
	for (i = pipeline->pipeline_graph->node_cnt - 1; i >= 0; i--) {
		if (pipeline->node_list[i])
			cam_node_destory(pipeline->node_list[i]);
		pipeline->node_list[i] = NULL;
	}
	cam_buf_kernel_sys_vfree(pipeline);
	pipeline = NULL;
}
