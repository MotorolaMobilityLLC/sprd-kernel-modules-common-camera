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

#include "cam_zoom.h"
#include "cam_pipeline.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_ZOOM: %d %d %s : " fmt, current->pid, __LINE__, __func__

static int camzoom_port_info_cfg(struct cam_zoom_port *zoom_port,
	uint32_t node_type, uint32_t node_id, struct cam_zoom_desc *param)
{
	int valid = 0;
	struct cam_zoom_base *zoom_base = NULL;

	zoom_base = &zoom_port->zoom_base;
	switch (node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		if (zoom_port->port_id == PORT_BIN_OUT ||
			zoom_port->port_id == PORT_RAW_OUT ||
			zoom_port->port_id == PORT_FULL_OUT) {
			zoom_base->src = param->sn_rect_size;
			zoom_base->crop = param->dcam_crop[zoom_port->port_id];
			zoom_base->dst = param->dcam_dst[zoom_port->port_id];
			zoom_base->ratio_width = param->zoom_ratio_width;
			zoom_base->total_crop_width = param->total_crop_width;
			valid = 1;
			CAM_ZOOM_DEBUG("dcam size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		if (zoom_port->port_type == PORT_TRANSFER_IN) {
			zoom_base->src = param->isp_src_size;
			/* node input no need crop same with pre node output */
			zoom_base->crop.start_x = 0;
			zoom_base->crop.start_y = 0;
			zoom_base->crop.size_x = zoom_base->src.w;
			zoom_base->crop.size_y = zoom_base->src.h;
			zoom_base->dst = zoom_base->src;
			valid = 1;
			CAM_ZOOM_DEBUG("isp size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		if (zoom_port->port_type == PORT_TRANSFER_OUT &&
			(zoom_port->port_id == PORT_PRE_OUT || zoom_port->port_id == PORT_VID_OUT
			|| zoom_port->port_id == PORT_CAP_OUT || zoom_port->port_id == PORT_THUMB_OUT)) {
			zoom_base->src = param->isp_src_size;
			zoom_base->crop = param->isp_crop[zoom_port->port_id];
			zoom_base->dst = param->dcam_isp[zoom_port->port_id];
			valid = 1;
			CAM_ZOOM_DEBUG("isp id %d size %d %d trim %d %d %d %d dst %d %d\n",
				zoom_port->port_id, zoom_base->src.w, zoom_base->src.h,
				zoom_base->crop.start_x, zoom_base->crop.start_y, zoom_base->crop.size_x,
				zoom_base->crop.size_y, zoom_base->dst.w, zoom_base->dst.h);
		}
		break;
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		break;
	default :
		pr_err("fail to support port %d zoom cfg in node %d\n", zoom_port->port_id, node_type);
		break;
	}

	return valid;
}

static struct cam_zoom_frame *camzoom_frame_param_cfg(struct cam_zoom_desc *param)
{
	int ret = 0, i = 0, j = 0, m = 0, n = 0;
	struct cam_zoom_frame *pframe = NULL;
	struct cam_pipeline_topology *graph = NULL;

	graph = param->pipeline_graph;
	pframe = cam_queue_empty_zoom_get();
	for (i = 0; i < graph->node_cnt; i++) {
		n = 0;
		if (graph->nodes[i].state != CAM_NODE_STATE_WORK ||
			graph->nodes[i].type == CAM_NODE_TYPE_DATA_COPY ||
			graph->nodes[i].type == CAM_NODE_TYPE_FRAME_CACHE)
			continue;
		pframe->zoom_node[m].node_type = graph->nodes[i].type;
		pframe->zoom_node[m].node_id = graph->nodes[i].id;
		for (j = 0; j < CAM_NODE_PORT_IN_NUM; j++) {
			if (graph->nodes[i].inport[j].link_state != PORT_LINK_NORMAL)
				continue;
			pframe->zoom_node[m].zoom_port[n].port_type = graph->nodes[i].inport[j].transfer_type;
			pframe->zoom_node[m].zoom_port[n].port_id = graph->nodes[i].inport[j].id;
			ret = camzoom_port_info_cfg(&pframe->zoom_node[m].zoom_port[n],
				graph->nodes[i].type, graph->nodes[i].id, param);
			if (!ret)
				continue;
			n++;
			if (n >= PORT_ZOOM_CNT_MAX)
				break;
		}

		for (j = 0; j < CAM_NODE_PORT_OUT_NUM; j++) {
			if (graph->nodes[i].outport[j].link_state != PORT_LINK_NORMAL)
				continue;
			pframe->zoom_node[m].zoom_port[n].port_type = graph->nodes[i].outport[j].transfer_type;
			pframe->zoom_node[m].zoom_port[n].port_id = graph->nodes[i].outport[j].id;
			ret = camzoom_port_info_cfg(&pframe->zoom_node[m].zoom_port[n],
				graph->nodes[i].type, graph->nodes[i].id, param);
			if (!ret)
				continue;
			n++;
			if (n >= PORT_ZOOM_CNT_MAX)
				break;
		}
		m++;
		if (m >= NODE_ZOOM_CNT_MAX)
			break;
	}

	return pframe;
}

int cam_zoom_param_set(struct cam_zoom_desc *in_ptr)
{
	int ret = 0, i = 0;
	unsigned long flag = 0;
	struct cam_zoom_frame *cur_zoom = NULL;
	struct cam_zoom_frame *latest_zoom = NULL;
	struct camera_queue *zoom_q = NULL;

	if (!in_ptr) {
		pr_err("fail to get valid ptr\n");
		return -EFAULT;
	}

	latest_zoom = in_ptr->latest_zoom_info;
	zoom_q = in_ptr->zoom_info_q;
	if (!latest_zoom || !zoom_q || !in_ptr->pipeline_graph) {
		pr_err("fail to get valid %px %px\n", latest_zoom, zoom_q);
		return -EFAULT;
	}

	if (in_ptr->pipeline_type < CAM_PIPELINE_TYPE_MAX) {
		cur_zoom = camzoom_frame_param_cfg(in_ptr);
		if (cur_zoom) {
			pr_debug("set cur_zoom %px\n", cur_zoom);
			ret = cam_queue_enqueue(zoom_q, &cur_zoom->list);
			if (ret) {
				pr_err("fail to enq zoom cnt %d\n", zoom_q->cnt);
				cam_queue_empty_zoom_put(cur_zoom);
				return ret;
			}
			spin_lock_irqsave(in_ptr->zoom_lock, flag);
			for (i = 0; i < NODE_ZOOM_CNT_MAX; i++)
				latest_zoom->zoom_node[i] = cur_zoom->zoom_node[i];
			spin_unlock_irqrestore(in_ptr->zoom_lock, flag);
		}
	} else {
		pr_err("fail to support pipeline type %d\n", in_ptr->pipeline_type);
	}

	return ret;
}

int cam_zoom_frame_base_get(struct cam_zoom_base *zoom_base, struct cam_zoom_index *zoom_index)
{
	int ret = 0, i = 0, j = 0;
	struct cam_zoom_frame *cur_zoom = NULL;
	struct cam_zoom_node *zoom_node = NULL;
	struct cam_zoom_port *zoom_port = NULL;

	if (!zoom_base || !zoom_index) {
		pr_err("fail to get valid param %px %px\n", zoom_base, zoom_index);
		return -EFAULT;
	}

	cur_zoom = (struct cam_zoom_frame *)zoom_index->zoom_data;
	for (i = 0; i < NODE_ZOOM_CNT_MAX; i++) {
		zoom_node = &cur_zoom->zoom_node[i];
		if (zoom_index->node_type == zoom_node->node_type
			&& zoom_index->node_id == zoom_node->node_id) {
			for (j = 0; j < PORT_ZOOM_CNT_MAX; j++) {
				zoom_port = &zoom_node->zoom_port[j];
				if (zoom_index->port_id == zoom_port->port_id
					&& zoom_index->port_type == zoom_port->port_type) {
					zoom_base->src = zoom_port->zoom_base.src;
					zoom_base->crop = zoom_port->zoom_base.crop;
					zoom_base->dst = zoom_port->zoom_base.dst;
					zoom_base->ratio_width = zoom_port->zoom_base.ratio_width;
					zoom_base->total_crop_width = zoom_port->zoom_base.total_crop_width;
					break;
				}
			}
			break;
		}
	}

	if (i == NODE_ZOOM_CNT_MAX && j == PORT_ZOOM_CNT_MAX)
		pr_warn("warning: not find correct zoom data\n");

	return ret;
}

void cam_zoom_frame_free(void *param)
{
	struct cam_zoom_frame *zoom_frame = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	zoom_frame = (struct cam_zoom_frame *)param;
	if (zoom_frame) {
		cam_queue_empty_zoom_put(zoom_frame);
		zoom_frame = NULL;
	}
}

