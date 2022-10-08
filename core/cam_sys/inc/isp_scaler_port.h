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

#ifndef _ISP_SCALER_PORT_H_
#define _ISP_SCALER_PORT_H_

#include "cam_types.h"
#include "cam_port.h"

struct isp_scaler_port {
	struct list_head list;
	atomic_t user_cnt;
	uint32_t port_id;
	void *data_cb_handle;
	cam_data_cb data_cb_func;
	struct camera_queue out_buf_queue;
	struct camera_queue result_queue;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
};

struct isp_scaler_port_desc {
	void **port_dev;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
};

void *isp_scaler_port_get(uint32_t port_id, struct isp_scaler_port_desc *param);
void isp_scaler_port_put(struct isp_scaler_port *port);
uint32_t isp_scaler_port_id_switch(uint32_t port_id);
struct camera_frame *ispscaler_port_reserved_buf_get(reserved_buf_get_cb resbuf_get_cb, void *cb_data, void *path);
bool ispscaler_port_fid_check(struct camera_frame *frame, void *data);
int ispscaler_port_fetch_normal_get(void *cfg_in, void *cfg_out, struct camera_frame *frame);
int ispscaler_port_store_normal_get(void *cfg_in, struct isp_hw_path_store *store_info);
int ispscaler_port_scaler_get(void *cfg_in, struct isp_hw_path_scaler *path);
int ispscaler_thumbport_scaler_get(void *cfg_in, struct isp_hw_thumbscaler_info *scalerInfo);

#endif

