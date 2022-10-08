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

#ifndef _FRAME_CACHE_NODE_H_
#define _FRAME_CACHE_NODE_H_

#include "cam_types.h"
#include "cam_queue.h"

enum frame_cache_node_id {
	FRAME_CACHE_CAP_NODE_ID,
	FRAME_CACHE_MAX_NODE_ID,
};

struct frame_cache_node_desc {
	uint32_t node_type;
	uint32_t cache_real_num;
	uint32_t cache_skip_num;
	uint32_t is_share_buf;
	uint32_t need_dual_sync;
	void *data_cb_handle;
	cam_data_cb data_cb_func;
	dual_frame_sync_cb dual_sync_func;
	dual_slave_frame_set_cb dual_slave_frame_set;
	void *dual_sync_cb_data;
};

struct frame_cache_node {
	uint32_t node_type;
	uint32_t cur_cache_skip_num;
	uint32_t cache_real_num;
	uint32_t cache_skip_num;
	uint32_t is_share_buf;
	uint32_t need_dual_sync;
	atomic_t user_cnt;
	void *data_cb_handle;
	cam_data_cb data_cb_func;
	dual_frame_sync_cb dual_sync_func;
	dual_slave_frame_set_cb dual_slave_frame_set;
	void *dual_sync_cb_data;

	struct cam_linkage link_info;
	struct cam_capture_param cap_param;
	struct camera_queue cache_buf_queue;
};

int frame_cache_outbuf_back(void *handle, void *param);
int frame_cache_cfg_param(void *handle, uint32_t cmd, void *param);
int frame_cache_node_request_proc(struct frame_cache_node *node, void *param);
void *frame_cache_node_get(uint32_t node_id, struct frame_cache_node_desc *param);
void frame_cache_node_put(struct frame_cache_node *node);

#endif
