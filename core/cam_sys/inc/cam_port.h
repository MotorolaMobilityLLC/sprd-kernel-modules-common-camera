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

#ifndef _CAM_PORT_H_
#define _CAM_PORT_H_

#include "cam_link.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "dcam_hw_adpt.h"
#include "cam_scaler.h"
#include "cam_buf_manager.h"

#define IS_VALID_DCAM_PORT_ID(id) ((id) >= PORT_RAW_OUT && (id) < PORT_DCAM_OUT_MAX)
#define IS_VALID_ISP_PORT_ID(id) ((id) >= PORT_PRE_OUT && (id) < PORT_ISP_OUT_MAX)
#define IS_VALID_DCAM_IMG_PORT(id) ((id) >= PORT_RAW_OUT && (id) < PORT_AEM_OUT)
#define IS_VALID_ISP_IMG_PORT(id) ((id) >= PORT_PRE_OUT && (id) < PORT_YUV_HIST_OUT)
#define PORT_ISP_MAX  (PORT_ISP_IN_MAX + PORT_ISP_OUT_MAX)

enum cam_port_dcam_online_in_id {
	PORT_ONLINE_SENSOR_IN,
	PORT_DCAM_ONLINE_IN_MAX,
};

enum cam_port_dcam_online_out_id {
	PORT_RAW_OUT,
	PORT_BIN_OUT,
	PORT_FULL_OUT,
	PORT_AEM_OUT,
	PORT_AFM_OUT,
	PORT_AFL_OUT,
	PORT_PDAF_OUT,
	PORT_VCH2_OUT,
	PORT_VCH3_OUT,
	PORT_BAYER_HIST_OUT,
	PORT_FRGB_HIST_OUT,
	PORT_LSCM_OUT,
	PORT_GTM_HIST_OUT,
	PORT_DCAM_OUT_MAX,
};

enum cam_port_dcam_offline_in_id {
	PORT_DCAM_OFFLINE_IN,
	PORT_DCAM_OFFLINE_IN_MAX,
};

enum cam_port_dcam_offline_out_id {
	PORT_OFFLINE_RAW_OUT,
	PORT_OFFLINE_BIN_OUT,
	PORT_OFFLINE_FULL_OUT,
	PORT_OFFLINE_AEM_OUT,
	PORT_OFFLINE_AFM_OUT,
	PORT_OFFLINE_AFL_OUT,
	PORT_OFFLINE_PDAF_OUT,
	PORT_OFFLINE_BAYER_HIST_OUT,
	PORT_OFFLINE_FRGB_HIST_OUT,
	PORT_OFFLINE_LSCM_OUT,
	PORT_OFFLINE_GTM_HIST_OUT,
	PORT_DCAM_OFFLINE_OUT_MAX,
};

enum cam_port_isp_in_id {
	PORT_ISP_OFFLINE_IN,
	PORT_ISP_OFFLINE_COMPRESS_IN,
	PORT_ISP_IN_MAX,
};

enum cam_port_isp_out_id {
	PORT_PRE_OUT,
	PORT_VID_OUT,
	PORT_CAP_OUT,
	PORT_THUMB_OUT,
	PORT_YUV_HIST_OUT,
	PORT_LTM_HIST_OUT,
	PORT_GTM_HIST_ISP_OUT,
	PORT_NR3_IN,
	PORT_NR3_OUT,
	PORT_ISP_OUT_MAX,
};

enum cam_port_isp_yuv_scaler_in_id {
	PORT_PRE_ISP_YUV_SCALER_IN,
	PORT_CAP_ISP_YUV_SCALER_IN,
	PORT_ISP_YUV_SCALER_IN_MAX,
};

enum cam_port_isp_yuv_scaler_out_id {
	PORT_PRE_ISP_YUV_SCALER_OUT,
	PORT_VID_ISP_YUV_SCALER_OUT,
	PORT_CAP_ISP_YUV_SCALER_OUT,
	PORT_THUMB_ISP_YUV_SCALER_OUT,
	PORT_ISP_YUV_SCALER_OUT_MAX,
};

enum cam_port_cache_in_id {
	PORT_FRAME_CACHE_IN,
	PORT_FRAME_CACHE_IN_MAX,
};

enum cam_port_cache_out_id {
	PORT_FRAME_CACHE_OUT,
	PORT_FRAME_CACHE_OUT_MAX,
};

enum cam_port_rec_in_id {
	PORT_REC_CUR_IN,
	PORT_REC_PRE_IN,
	PORT_REC_IN_MAX,
};

enum cam_port_rec_out_id {
	PORT_REC_OUT,
	PORT_REC_OUT_MAX,
};

enum cam_port_dec_in_id {
	PORT_DEC_IN,
	PORT_DEC_IN_MAX,
};

enum cam_port_dec_out_id {
	PORT_DEC_OUT,
	PORT_DEC_DCT_OUT,
	PORT_DEC_OUT_MAX,
};

enum cam_port_buf_type {
	PORT_BUFFER_NORMAL,
	PORT_BUFFER_RESERVED,
	PORT_BUFFER_MAX,
};

enum cam_port_cfg_cmd {
	PORT_ZOOM_CFG_SET,
	PORT_SIZE_CFG_SET,
	PORT_BUFFER_CFG_SET,
	PORT_PARAM_CFG_GET,
	PORT_BUFFER_CFG_GET,
	PORT_CFG_BASE,
	PORT_UFRAME_CFG,
	PORT_RESERVERD_BUF_CFG,
	PORT_PM_CFG_MAX,
};

enum cam_port_transfer_type {
	PORT_TRANSFER_IN,
	PORT_TRANSFER_OUT,
	PORT_TRANSFER_MAX,
};

enum cam_port_link_state {
	PORT_LINK_IDLE,
	PORT_LINK_NORMAL,
	PORT_LINK_MAX,
};

enum cam_port_scaler_type {
	PORT_SCALER_BINNING = 0,
	PORT_SCALER_BY_YUVSCALER,
	PORT_SCALER_BYPASS,
	PORT_SCALER_MAX,
};

enum export_type {
	PORT_MASTER = 0,
	PORT_SLAVE,
};

struct cam_port_shutoff_ctrl {
	atomic_t cap_cnt;
	uint32_t port_id;
	enum shutoff_scene shutoff_scene;
	enum shutoff_type shutoff_type;
};

static inline uint32_t cal_sprd_pitch_yuv(uint32_t w, uint32_t dcam_out_bits, uint32_t is_pack)
{
	if (dcam_out_bits != CAM_8_BITS) {
		if(is_pack)
			w = (w * 10 + 127) / 128 * 128 / 8;
		else
			w = (w * 16 + 127) / 128 * 128 / 8;
	}

	return w;
}

static inline uint32_t cal_sprd_pitch_raw(uint32_t w, uint32_t pack_bits)
{
	if(pack_bits == DCAM_RAW_PACK_10)
		w = (w * 10 + 127) / 128 * 128 / 8;
	else
		w = (w * 16 + 127) / 128 * 128 / 8;

	return w;
}

struct cam_port_buf_prepare_param{
	uint32_t cam_idx;
	uint32_t hw_idx;
	uint32_t reg_addr;
	uint32_t fid;
	struct cam_hw_info *hw;
	struct dcam_mipi_info *cap_info;
	struct dcam_isp_k_block *blk_dcam_pm;
	uint32_t port_need_sync;
	uint32_t is_fmcu_slowmotion;
	uint32_t slowmotion_count;
	uint32_t use_reserved_buf;
	uint32_t not_use_reserved_buf;
	uint32_t use_fdr_raw_buffer;
	uint32_t share_full_buf;
};

struct cam_port_topology {
	uint32_t node_type;
	enum cam_port_transfer_type transfer_type;
	enum cam_port_link_state link_state;
	uint32_t id;
	uint32_t dump_en;
	uint32_t dump_node_id;
	uint32_t replace_en;
	uint32_t replace_node_id;
	uint32_t copy_en;
	uint32_t copy_node_id;
	uint32_t dynamic_link_en;
	enum export_type depend_type;
	struct cam_linkage link;
	struct cam_linkage switch_link;
	uint32_t buf_num;
};

struct cam_port_desc {
	void *nodes_dev;
	void **port_dev;
	zoom_get_cb zoom_cb_func;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	void *zoom_cb_handle;
	shutoff_cb shutoff_cb_func;
	void *shutoff_cb_handle;
	struct dcam_online_port_desc *dcam_online;
	struct dcam_offline_port_desc *dcam_offline;
	struct dcam_offline_port_desc *dcam_offline_bpcraw;
	struct isp_port_desc *isp_offline;
	struct isp_scaler_port_desc *isp_offline_scaler;
	struct cam_port_topology *port_graph;
};

struct cam_port_ops {
	int (*cfg_param)(void *handle, enum cam_port_cfg_cmd cmd, void *param);
};

struct cam_port {
	void *handle;
	struct cam_port_topology *port_graph;
	struct cam_port_ops ops;
};

extern struct cam_buf_manager *global_buf_manager;
const char *camport_fmt_name_get(enum cam_format type);
const char *cam_port_name_get(enum cam_port_dcam_online_out_id port_id);
const char *cam_port_dcam_offline_out_id_name_get(enum cam_port_dcam_offline_out_id port_id);
int cam_port_static_portlist_get(struct cam_port_topology *param, uint32_t node_type, uint32_t transfer_type);
int cam_port_buffers_alloc(void *handle, uint32_t node_id,struct cam_buf_alloc_desc *param);
void *cam_port_creat(struct cam_port_desc *param, uint32_t node_id);
void cam_port_destory(struct cam_port *port);

#endif
