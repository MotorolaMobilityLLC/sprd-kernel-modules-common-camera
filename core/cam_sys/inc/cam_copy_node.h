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

#ifndef _CAM_COPY_NODE_H_
#define _CAM_COPY_NODE_H_

#include "cam_types.h"
#include "cam_queue.h"

#define COPY_NODE_Q_LEN                      50

enum cam_copy_node_id {
	CAM_COPY_NODE_ID_0,
	CAM_COPY_NODE_ID_1,
	CAM_COPY_NODE_ID_MAX,
};

struct cam_copy_node {
	uint32_t node_id;
	uint32_t copy_flag;
	void *copy_cb_handle;
	cam_data_cb copy_cb_func;
	struct camera_queue in_queue;
	struct camera_queue out_queue;
	struct cam_thread_info thread;
};

int cam_copy_node_request_proc(struct cam_copy_node *node, void *param);
void *cam_copy_node_get(uint32_t node_id, cam_data_cb cb_func, void *priv_data);
void cam_copy_node_put(struct cam_copy_node *node);
int cam_copy_node_buffer_cfg(void *handle, void *param);

#endif
