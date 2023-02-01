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

#ifndef _CAM_ZOOM_H_
#define _CAM_ZOOM_H_

#include "cam_core.h"
#include "cam_port.h"

#ifdef CAM_ZOOM_DEBUG_ON
#define CAM_ZOOM_DEBUG pr_info
#else
#define CAM_ZOOM_DEBUG pr_debug
#endif

#define ZOOM_PORT_DCAM_MAX   (PORT_FULL_OUT + 1)
#define ZOOM_PORT_ISP_MAX    (PORT_THUMB_OUT + 1)

struct cam_zoom_index {
	void *zoom_data;
	uint32_t node_type;
	uint32_t node_id;
	uint32_t port_type;
	uint32_t port_id;
};

struct cam_zoom_desc {
	uint32_t pipeline_type;
	struct cam_pipeline_topology *pipeline_graph;
	struct cam_zoom_frame *latest_zoom_info;
	spinlock_t *zoom_lock;
	struct camera_queue *zoom_info_q;
	struct img_size sn_rect_size;
	uint32_t zoom_ratio_width;
	uint32_t total_crop_width;
	struct img_trim dcam_crop[ZOOM_PORT_DCAM_MAX];
	struct img_size dcam_dst[ZOOM_PORT_DCAM_MAX];
	struct img_size isp_src_size;
	struct img_trim isp_crop[ZOOM_PORT_ISP_MAX];
	struct img_size dcam_isp[ZOOM_PORT_ISP_MAX];
};

void cam_zoom_diff_trim_get(struct sprd_img_rect *orig,
	uint32_t ratio16, struct img_trim *trim0, struct img_trim *trim1);
int cam_zoom_crop_size_align(struct camera_module *module,
	struct sprd_img_rect *crop, uint32_t channel_id);
int cam_zoom_channels_size_init(struct camera_module *module);
int cam_zoom_channel_size_calc(struct camera_module *module);
int cam_zoom_channel_size_config(
	struct camera_module *module, struct channel_context *channel);
int cam_zoom_4in1_channel_size_config(struct camera_module *module);
int cam_zoom_channel_bigsize_config(
	struct camera_module *module, struct channel_context *channel);
int cam_zoom_start_proc(void *param);
int cam_zoom_stream_state_get(struct isp_node *node, struct camera_frame *pframe);
int cam_zoom_param_set(struct cam_zoom_desc *zoom_info);
int cam_zoom_frame_base_get(struct cam_zoom_base *zoom_base, struct cam_zoom_index *zoom_index);

#endif
