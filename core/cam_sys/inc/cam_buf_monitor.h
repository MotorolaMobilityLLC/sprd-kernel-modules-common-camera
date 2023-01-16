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

#ifndef _CAM_BUF_MONITOR_H_
#define _CAM_BUF_MONITOR_H_

#include "cam_buf.h"

enum cam_buf_source {
	CAM_BUF_SOURCE_ALLOC_GET,
	CAM_BUF_SOURCE_MEMCPY,
	CAM_BUF_SOURCE_MAX,
};

enum cam_buf_map_type {
	CAM_BUF_DCAM_IOMMUMAP,
	CAM_BUF_DCAM_IOMMUUNMAP,
	CAM_BUF_ISP_IOMMUMAP,
	CAM_BUF_ISP_IOMMUUNMAP,
	CAM_BUF_KMAP,
	CAM_BUF_KUNMAP,
	CAM_BUF_MAP_MAX,
};

struct cam_buf_alloc_info {
	void *addr;
	timeval time;
	struct list_head list;
};

struct cam_buf_map_info {
	uint32_t buf_memcyp;
	uint32_t kmap_counts;
	uint32_t dcam_iommumap_counts;
	uint32_t isp_iommumap_counts;
	timeval buf_time;
	timeval map_time;
	struct camera_buf *buf_info;
	struct list_head list;
};

void cam_buf_monitor_memory_queue_init(void);
void cam_buf_monitor_memory_queue_deinit(void);
void cam_buf_monitor_memory_queue_check(void);
int cam_buf_monitor_mdbg_check(void);
void cam_buf_monitor_mapinfo_dequeue(struct camera_buf *buf_info);
int cam_buf_monitor_mapinfo_enqueue(struct camera_buf *buf_info, enum cam_buf_source buf_source);
void cam_buf_monitor_map_counts_calc(struct camera_buf *buf_info, enum cam_buf_map_type type);

#endif
