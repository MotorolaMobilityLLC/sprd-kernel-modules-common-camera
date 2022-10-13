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

#include <linux/vmalloc.h>
#include <linux/sched.h>
#include "cam_port.h"
#include "cam_node.h"
#include "dcam_online_port.h"
#include "isp_port.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

static const char *_DCAM_PORT_NAMES[PORT_DCAM_OUT_MAX] = {
	[PORT_RAW_OUT] = "RAW",
	[PORT_BIN_OUT] = "BIN",
	[PORT_FULL_OUT] = "FULL",
	[PORT_AEM_OUT] = "AEM",
	[PORT_AFM_OUT] = "AFM",
	[PORT_AFL_OUT] = "AFL",
	[PORT_PDAF_OUT] = "PDAF",
	[PORT_VCH2_OUT] = "VCH2",
	[PORT_VCH3_OUT] = "VCH3",
	[PORT_BAYER_HIST_OUT] = "BAYER_HIST",
	[PORT_FRGB_HIST_OUT] = "FRGB_HIST",
	[PORT_LSCM_OUT] = "LSCM",
	[PORT_GTM_HIST_OUT] = "GTMHIST",
};

const char *cam_port_name_get(enum cam_port_dcam_online_out_id port_id)
{
	return is_port_id(port_id) ? _DCAM_PORT_NAMES[port_id] : "(null)";
}

static const char *_DCAM_PORT_DCAM_OFFLINE_OUT_NAMES[PORT_DCAM_OFFLINE_OUT_MAX] = {
	[PORT_OFFLINE_RAW_OUT] = "PORT_OFFLINE_RAW_OUT",
	[PORT_OFFLINE_BIN_OUT] = "PORT_OFFLINE_BIN_OUT",
	[PORT_OFFLINE_FULL_OUT] = "PORT_OFFLINE_FULL_OUT",
	[PORT_OFFLINE_AEM_OUT] = "PORT_OFFLINE_AEM_OUT",
	[PORT_OFFLINE_AFM_OUT] = "PORT_OFFLINE_AFM_OUT",
	[PORT_OFFLINE_AFL_OUT] = "PORT_OFFLINE_AFL_OUT",
	[PORT_OFFLINE_PDAF_OUT] = "PORT_OFFLINE_PDAF_OUT",
	[PORT_OFFLINE_BAYER_HIST_OUT] = "PORT_OFFLINE_BAYER_HIST_OUT",
	[PORT_OFFLINE_FRGB_HIST_OUT] = "PORT_OFFLINE_FRGB_HIST_OUT",
	[PORT_OFFLINE_LSCM_OUT] = "PORT_OFFLINE_LSCM_OUT",
	[PORT_OFFLINE_GTM_HIST_OUT] = "PORT_OFFLINE_GTMHIST_OUT",
};

const char *cam_port_dcam_offline_out_id_name_get(enum cam_port_dcam_offline_out_id port_id)
{
	return _DCAM_PORT_DCAM_OFFLINE_OUT_NAMES[port_id];
}

static void camport_dcamonline_port_get(struct cam_port_topology *port,
		uint32_t node_type, uint32_t transfer_type)
{
	uint32_t i = 0;

	if (transfer_type == PORT_TRANSFER_IN) {
		port->node_type = node_type;
		port->transfer_type = transfer_type;
		port->id = PORT_ONLINE_SENSOR_IN;
		port->dynamic_link_en = 0;
		cam_link_init(&port->link);
		cam_link_init(&port->switch_link);
	} else {
		for (i = 0; i < PORT_DCAM_OUT_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	}
}

static void camport_dcamoffline_port_get(struct cam_port_topology *port,
		uint32_t node_type, uint32_t transfer_type)
{
	uint32_t i = 0;

	if (transfer_type == PORT_TRANSFER_IN) {
		port->node_type = node_type;
		port->transfer_type = transfer_type;
		port->id = PORT_DCAM_OFFLINE_IN;
		port->dynamic_link_en = 0;
		cam_link_init(&port->link);
		cam_link_init(&port->switch_link);
	} else {
		for (i = 0; i < PORT_DCAM_OFFLINE_OUT_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	}
}

static void camport_ispoffline_port_get(struct cam_port_topology *port,
		uint32_t node_type, uint32_t transfer_type)
{
	uint32_t i = 0;

	if (transfer_type == PORT_TRANSFER_IN) {
		for (i = 0; i < PORT_ISP_IN_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	} else {
		for (i = 0; i < PORT_ISP_OUT_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	}
}

static void camport_framecache_port_get(struct cam_port_topology *port,
		uint32_t node_type, uint32_t transfer_type)
{
	uint32_t i = 0;

	if (transfer_type == PORT_TRANSFER_IN) {
		port->node_type = node_type;
		port->transfer_type = transfer_type;
		port->id = PORT_FRAME_CACHE_IN;
		port->dynamic_link_en = 0;
		cam_link_init(&port->link);
		cam_link_init(&port->switch_link);
	} else {
		for (i = 0; i < PORT_FRAME_CACHE_OUT_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	}
}

static void camport_ispyuvscaler_port_get(struct cam_port_topology *port,
		uint32_t node_type, uint32_t transfer_type)
{
	int i = 0;

	if (transfer_type == PORT_TRANSFER_IN) {
		for (i = 0; i < PORT_ISP_YUV_SCALER_IN_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	} else {
		for (i = 0; i < PORT_ISP_YUV_SCALER_OUT_MAX; i++, port++) {
			port->node_type = node_type;
			port->transfer_type = transfer_type;
			port->id = i;
			port->dynamic_link_en = 0;
			cam_link_init(&port->link);
			cam_link_init(&port->switch_link);
		}
	}

}

static int camport_cfg_param(void *handle, enum cam_port_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct cam_port *port = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	port = (struct cam_port *)handle;
	switch (port->port_graph->node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		ret = dcam_online_port_param_cfg(port->handle, cmd, param);
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		ret = dcam_offline_port_param_cfg(port->handle, cmd, param);
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		ret = isp_port_param_cfg(port->handle, cmd, param);
		break;
	default:
		pr_err("fail to support node type %s\n", cam_node_name_get(port->port_graph->node_type));
		ret = -EFAULT;
		break;
	}

	return ret;
}

int cam_port_static_portlist_get(struct cam_port_topology *param,
	uint32_t node_type, uint32_t transfer_type)
{
	int ret = 0;

	if (!param) {
		pr_err("fail to get invalid param ptr\n");
		return -EFAULT;
	}

	switch (node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		camport_dcamonline_port_get(param, node_type, transfer_type);
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		camport_dcamoffline_port_get(param, node_type, transfer_type);
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		camport_ispoffline_port_get(param, node_type, transfer_type);
		break;
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		camport_ispyuvscaler_port_get(param, node_type, transfer_type);
		break;
	case CAM_NODE_TYPE_FRAME_CACHE:
		camport_framecache_port_get(param, node_type, transfer_type);
		break;
	/* These nodes no need to use port now */
	case CAM_NODE_TYPE_PYR_DEC:
	case CAM_NODE_TYPE_DATA_COPY:
	case CAM_NODE_TYPE_DUMP:
		break;
	default:
		pr_err("fail to support node type %s\n", cam_node_name_get(node_type));
		ret = -EFAULT;
		break;
	}

	return ret;
}

int cam_port_buffers_alloc(void *handle, uint32_t node_id, struct cam_buf_alloc_desc *param)
{
	int ret = 0;
	struct cam_port *port = (struct cam_port *)handle;

	if (!handle) {
		pr_err("fail to get port handle\n");
		return -EFAULT;
	}

	if (port->port_graph->link_state != PORT_LINK_NORMAL)
		return ret;
	if ((port->port_graph->link.node_type == CAM_NODE_TYPE_USER) ||
		(port->port_graph->link.node_id == CAM_LINK_DEFAULT_NODE_ID))
		return ret;

	if ((port->port_graph->switch_link.node_type == CAM_NODE_TYPE_USER) &&
		(node_id == port->port_graph->link.node_id))
		return ret;
	switch (port->port_graph->node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		ret = dcam_online_port_buf_alloc(port->handle, param);
		break;
	default:
		pr_debug("not support node %s\n", cam_node_name_get(port->port_graph->node_type));
		return ret;
	}
	return ret;
}

void *cam_port_creat(struct cam_port_desc *param, uint32_t node_id)
{
	struct cam_port *port = NULL;
	struct cam_nodes_dev *nodes_dev = NULL;

	if (!param) {
		pr_err("fail to get invalid nodedesc info\n");
		return NULL;
	}

	port = vzalloc(sizeof(struct cam_port));
	if (!port) {
		pr_err("fail to alloc cam port\n");
		return NULL;
	}

	port->port_graph = param->port_graph;
	if (!port->port_graph) {
		pr_info("port->port_graph is NULL\n");
		vfree(port);
		return NULL;
	}

	nodes_dev = (struct cam_nodes_dev *)param->nodes_dev;
	pr_debug("port id %d nodes_dev %px\n", port->port_graph->id, nodes_dev);
	if (!nodes_dev) {
		pr_info("nodes_dev is NULL\n");
		vfree(port);
		return NULL;
	}

	port->ops.cfg_param = camport_cfg_param;
	switch (port->port_graph->node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		param->dcam_online->port_dev = (void *)&nodes_dev->dcam_online_out_port_dev[port->port_graph->id];
		param->dcam_online->data_cb_func = param->data_cb_func;
		param->dcam_online->data_cb_handle = param->data_cb_handle;
		param->dcam_online->shutoff_cb_func = param->shutoff_cb_func;
		param->dcam_online->shutoff_cb_handle = param->shutoff_cb_handle;
		port->handle = dcam_online_port_get(port->port_graph->id, param->dcam_online);
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
		param->dcam_offline->port_dev = (void *)&nodes_dev->dcam_offline_out_port_dev[port->port_graph->id];
		param->dcam_offline->data_cb_func = param->data_cb_func;
		param->dcam_offline->data_cb_handle = param->data_cb_handle;
		port->handle = dcam_offline_port_get(port->port_graph->id, param->dcam_offline);
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		param->dcam_offline_bpcraw->port_dev = (void *)&nodes_dev->dcam_offline_bpcraw_out_port_dev[port->port_graph->id];
		param->dcam_offline_bpcraw->data_cb_func = param->data_cb_func;
		param->dcam_offline_bpcraw->data_cb_handle = param->data_cb_handle;
		port->handle = dcam_offline_port_get(port->port_graph->id, param->dcam_offline_bpcraw);
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		if (param->port_graph->transfer_type == PORT_TRANSFER_IN)
			param->isp_offline->port_dev = (void *)&nodes_dev->isp_in_port_dev[node_id][port->port_graph->id];
		else
			param->isp_offline->port_dev = (void *)&nodes_dev->isp_out_port_dev[node_id][port->port_graph->id];
		param->isp_offline->data_cb_func = param->data_cb_func;
		param->isp_offline->data_cb_handle = param->data_cb_handle;
		param->isp_offline->transfer_type = param->port_graph->transfer_type;
		param->isp_offline->depend_type = param->port_graph->depend_type;
		port->handle = isp_port_get(port->port_graph->id, param->isp_offline);
		break;
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		param->isp_offline_scaler->port_dev = (void *)&nodes_dev->isp_scaler_out_port_dev[port->port_graph->id];
		param->isp_offline_scaler->data_cb_func = param->data_cb_func;
		param->isp_offline_scaler->data_cb_handle = param->data_cb_handle;
		port->handle = isp_scaler_port_get(port->port_graph->id, param->isp_offline_scaler);
		break;
	case CAM_NODE_TYPE_FRAME_CACHE:
		break;
	case CAM_NODE_TYPE_PYR_DEC:
		break;
	default:
		pr_err("fail to support node type %s port id %d\n",
			cam_node_name_get(port->port_graph->node_type), port->port_graph->id);
		break;
	}

	pr_info("node %s port %px handle %px\n", cam_node_name_get(port->port_graph->node_type), port, port->handle);
	return port;
}

void cam_port_destory(struct cam_port *port)
{
	if (!port) {
		pr_err("fail to get valid port\n");
		return;
	}

	switch (port->port_graph->node_type) {
	case CAM_NODE_TYPE_DCAM_ONLINE:
		dcam_online_port_put(port->handle);
		break;
	case CAM_NODE_TYPE_DCAM_OFFLINE:
	case CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW:
		dcam_offline_port_put(port->handle);
		break;
	case CAM_NODE_TYPE_ISP_OFFLINE:
		isp_port_put(port->handle);
		break;
	case CAM_NODE_TYPE_ISP_YUV_SCALER:
		isp_scaler_port_put(port->handle);
		break;
	case CAM_NODE_TYPE_FRAME_CACHE:
		break;
	default:
		pr_err("fail to support node type %s port id %d\n",
			cam_node_name_get(port->port_graph->node_type), port->port_graph->id);
		break;
	}

	pr_info("node type %s port %s free success\n", cam_node_name_get(port->port_graph->node_type),
			cam_port_name_get(port->port_graph->id));
	vfree(port);
	port = NULL;
}
