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
	enum isp_statis_buf_type buf_type;
	size_t buf_size;
	size_t buf_cnt;
};

/* dcam online */
int cam_statis_isp_port_buffer_deinit(void *isp_handle, void *node);
int cam_statis_isp_port_buffer_cfg(void *isp_handle, void *node, struct isp_statis_buf_input *input);
int cam_statis_dcam_port_buffer_deinit(void *dcam_handle);
int cam_statis_dcam_port_buffer_cfg(void *dcam_handle, void *param);

#endif
