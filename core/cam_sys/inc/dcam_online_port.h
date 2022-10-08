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

#ifndef _DCAM_ONLINE_PORT_H_
#define _DCAM_ONLINE_PORT_H_

#include "cam_types.h"
#include "cam_port.h"

enum dcam_port_state {
	DCAM_PORT_IDLE,
	DCAM_PORT_PAUSE,
	DCAM_PORT_RESUME,
};

enum dcam_port_cfg_callback {
	DCAM_PORT_STORE_SET,
	DCAM_PORT_SLW_STORE_SET,
	DCAM_PORT_BUFFER_CFG_OUTBUF_GET,
	DCAM_PORT_BUFFER_CFG_GET,
	DCAM_PORT_BUFFER_CFG_SET,
	DCAM_PORT_NEXT_FRM_BUF_GET,
	DCAM_PORT_SHUTOFF_CFG_SET,
	DCAM_PORT_RES_BUF_CFG_SET,
};

struct dcam_port_size_info {
	uint32_t update;
	uint32_t out_w;
	uint32_t out_h;
	uint32_t out_pitch;
	uint32_t bin_ratio;
	uint32_t scaler_sel;
	struct yuv_scaler_info *scaler_info;
	struct img_size in_size;
	struct img_trim in_trim;
	uint32_t src_sel;
	void *priv_size_data;
	struct img_trim next_roi;
	uint32_t zoom_ratio;
	uint32_t total_zoom;
	uint32_t dst_crop_w;
	struct img_trim total_in_trim;
};

struct dcam_online_port_desc {
	void **port_dev;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	shutoff_cb shutoff_cb_func;
	void *shutoff_cb_handle;
	uint32_t dcam_path_id;
	uint32_t is_raw;
	uint32_t raw_src;
	uint32_t bayer_pattern;
	uint32_t dcam_out_fmt;
	uint32_t frm_skip;
	uint32_t endian;
	uint32_t pyr_out_fmt;
	uint32_t compress_en;
	uint32_t reserved_pool_id;
	uint32_t share_full_path;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	share_buf_get_cb sharebuf_get_cb;
	void *sharebuf_cb_data;
	void *dev;
};

struct dcam_online_port {
	struct list_head list;
	atomic_t user_cnt;
	atomic_t set_frm_cnt;
	atomic_t is_work; /* dynamic switch counter of port */
	atomic_t is_shutoff;
	spinlock_t size_lock;
	uint32_t port_update;
	spinlock_t state_lock;
	uint32_t state_update;
	enum dcam_port_state port_state;
	uint32_t share_full_path;

	/* path base info */
	uint32_t frm_deci_cnt;
	uint32_t frm_skip;
	uint32_t frm_cnt;
	uint32_t endian;
	uint32_t bayer_pattern;
	uint32_t pyr_out_fmt;
	uint32_t compress_en;
	uint32_t base_update;
	uint32_t src_sel;
	uint32_t raw_src;
	enum cam_format dcamout_fmt;
	enum cam_port_dcam_online_out_id port_id;
	/* path size info */
	void *priv_size_data;
	uint32_t bin_ratio;
	uint32_t size_update;
	uint32_t out_pitch;
	uint32_t scaler_sel;/* 0: bining, 1: RDS, 2&3: bypass */
	uint32_t zoom_ratio;
	uint32_t total_zoom;
	uint32_t dst_crop_w;
	struct img_trim next_roi;
	struct img_size in_size;
	struct img_trim in_trim;
	struct img_size out_size;
	struct img_trim total_in_trim;
	struct img_deci_info deci;
	struct dcam_hw_dec_store_cfg dec_store_info;
	struct yuv_scaler_info scaler_info;
	/* queue info */
	struct cam_buf_pool_id result_pool;
	struct cam_buf_pool_id unprocess_pool;
	struct cam_buf_pool_id reserved_pool;

	cam_data_cb data_cb_func;
	void *data_cb_handle;
	share_buf_get_cb sharebuf_get_cb;
	void *sharebuf_cb_data;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	port_cfg_cb port_cfg_cb_func;
	/* to store isp offline param data if frame is discarded. */
	void *isp_updata;
	shutoff_cb shutoff_cb_func;
	void *shutoff_cb_handle;
};

int dcam_online_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param);
int dcam_online_port_skip_num_set(void *dcam_ctx_handle, uint32_t hw_id, int port_id, uint32_t skip_num);
int dcam_online_port_buf_alloc(void *handle, struct cam_buf_alloc_desc *param);
void *dcam_online_port_get(uint32_t port_id, struct dcam_online_port_desc *param);
void dcam_online_port_put(struct dcam_online_port *port);
int dcamonline_port_buffer_cfg(void *handle, void *param);

#endif
