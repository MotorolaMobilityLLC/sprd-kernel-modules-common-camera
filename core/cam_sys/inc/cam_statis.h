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

#ifndef _CAM_STATIS_H_
#define _CAM_STATIS_H_

#include "cam_hw.h"
#include "cam_types.h"
#include "dcam_core.h"
#include "dcam_interface.h"
#include "cam_node.h"

struct statis_port_buf_info {
	enum cam_port_dcam_online_out_id port_id;
	size_t buf_size;
	size_t buf_cnt;
	uint32_t buf_type;
};

/* dcam online */
int cam_statis_dcam_port_bufferq_init(void *dcam_handle);
int cam_statis_dcam_port_bufferq_deinit(void *dcam_handle);
int cam_statis_dcam_port_buffer_cfg(void *dcam_handle, struct isp_statis_buf_input *input);
int cam_statis_dcam_port_buffer_skip_cfg(void *dcam_handle, struct camera_frame *pframe);
int cam_statis_isp_buffer_cfg(void *isp_handle, void *node, struct isp_statis_buf_input *input);
int cam_statis_isp_buffer_unmap(void *isp_handle, void *node);
void cam_statis_isp_buf_destroy(void *param);

#endif
