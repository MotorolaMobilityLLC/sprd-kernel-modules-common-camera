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

#ifndef _DCAM_OFFLINE_PORT_H_
#define _DCAM_OFFLINE_PORT_H_

#include "cam_types.h"
#include "cam_port.h"

#define DCAM_OFFLINE_RESULT_Q_LEN                 50
#define DCAM_OFFLINE_OUT_BUF_Q_LEN                50

struct dcam_offline_port_desc {
	void **port_dev;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	uint32_t compress_en;
	uint32_t endian;
	uint32_t dcam_out_fmt;
	uint32_t src_sel;
};

struct dcam_offline_port {
	struct list_head list;
	enum cam_port_dcam_offline_out_id port_id;
	atomic_t user_cnt;
	atomic_t is_work; /* dynamic switch counter of port */

	uint32_t src_sel;
	uint32_t endian;
	uint32_t out_fmt;
	uint32_t out_pitch;
	uint32_t bin_ratio;
	uint32_t scaler_sel;
	uint32_t compress_en;
	struct img_size in_size;
	struct img_trim in_trim;
	struct img_size out_size;
	struct yuv_scaler_info scaler_info;
	void *priv_size_data;

	void *data_cb_handle;
	cam_data_cb data_cb_func;

	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
};

int dcam_offline_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param);
void *dcam_offline_port_get(uint32_t port_id, struct dcam_offline_port_desc *param);
void dcam_offline_port_put(struct dcam_offline_port *port);

#endif
