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

#ifndef _DCAM_INTERFACE_H_
#define _DCAM_INTERFACE_H_

#include <linux/platform_device.h>

#include "cam_hw.h"
#include "cam_types.h"

/* 4-pixel align for MIPI CAP input from CSI */
#define DCAM_MIPI_CAP_ALIGN             4
/* dcam dec pyramid layer num */
#define DCAM_PYR_DEC_LAYER_NUM          4

#define is_dcam_id(idx)                 ((idx) < DCAM_ID_MAX)
#define is_csi_id(idx)                  ((idx) < CSI_ID_MAX)
#define is_path_id(id) ((id) >= DCAM_PATH_FULL && (id) < DCAM_PATH_MAX)

enum dcam_pdaf_type {
	DCAM_PDAF_DUAL = 0,
	DCAM_PDAF_TYPE1 = 1,
	DCAM_PDAF_TYPE2 = 2,
	DCAM_PDAF_TYPE3 = 3,
};

enum dcam_sw_ctx_type {
	DCAM_ONLINE_CTX = 0,
	DCAM_OFFLINE_CTX,
};

struct statis_path_buf_info {
	enum dcam_path_index path_id;
	size_t buf_size;
	size_t buf_cnt;
	uint32_t buf_type;
};

static inline uint32_t dcam_if_cal_pyramid_size(uint32_t w, uint32_t h,
		uint32_t out_bits, uint32_t is_pack, uint32_t start_layer, uint32_t layer_num)
{
	uint32_t align = 1, i = 0, size = 0, pitch = 0;
	uint32_t w_align = PYR_DEC_WIDTH_ALIGN;
	uint32_t h_align = PYR_DEC_HEIGHT_ALIGN;

	for (i = start_layer; i <= layer_num; i++) {
		w_align *= 2;
		h_align *= 2;
	}
	for (i = 0; i < start_layer; i++)
		align = align * 2;

	w = ALIGN(w, w_align);
	h = ALIGN(h, h_align);
	for (i = start_layer; i <= layer_num; i++) {
		pitch = w / align;
		if (out_bits != CAM_8_BITS) {
			if(is_pack)
				pitch = (pitch * 10 + 127) / 128 * 128 / 8;
			else
				pitch = (pitch * 16 + 127) / 128 * 128 / 8;
		}
		size = size + pitch * (h / align) * 3 / 2;
		align = align * 2;
	}

	return size;
}

/*
 *DCAM_STOP: normal stop;
 *DCAM_PAUSE_ONLINE: online paused; pause for fdr use same dcam for following offline process
 *DCAM_PAUSE_OFFLINE: offline paused; after fdr use same dcam for offline process and prepare for online resume
 */
enum dcam_stop_cmd {
	DCAM_STOP,
	DCAM_PAUSE_ONLINE,
	DCAM_PAUSE_OFFLINE,
	DCAM_RECOVERY,
};

struct dcam_path_cfg_param {
	uint32_t offline;
	uint32_t dcam_path_id;
	uint32_t slowmotion_count;
	uint32_t enable_3dnr;
	uint32_t is_raw;
	uint32_t raw_src;
	uint32_t bayer_pattern;
	uint32_t dcam_out_fmt;
	uint32_t is_4in1;
	uint32_t frm_deci;
	uint32_t frm_skip;
	uint32_t pyr_out_fmt;
	uint32_t endian;
	struct img_size input_size;
	struct img_trim input_trim;
	struct img_trim total_input_trim;
	struct img_size output_size;
	struct dcam_mipi_info cap_info;
	cam_data_cb data_cb_func;
	void *data_cb_handle;
	struct sprd_img_rect zoom_ratio_base;
	reserved_buf_get_cb resbuf_get_cb;
	void *resbuf_cb_data;
	void *blk_dcam_pm;
	uint32_t result_pool_id;
	uint32_t unprocess_pool_id;
};

struct dcam_irq_proc_desc {
	uint32_t dcam_cb_type;
	uint32_t dcam_port_id;
};

enum dcam_of {
	CAP_START_OF_FRAME,
	CAP_END_OF_FRAME,
	PREV_START_OF_FRAME,
	SN_START_OF_FRAME,
	SN_END_OF_FRAME,
	CAP_DATA_DONE,
	DCAM_INT_ERROR,
	DUMMY_RECONFIG,
};

struct dcam_irq_proc {
	uint32_t dcam_port_id;
	void *hw_ctx;
	uint32_t dummy_enable;
	uint32_t dummy_int_status;
	uint32_t not_dispatch;
	void *param;
	enum cam_cb_type type;
	enum dcam_of of;
	uint32_t dummy_cmd;
	uint32_t status;
	uint32_t frm_cnt;
	uint32_t bin_addr_value;
	uint32_t full_addr_value;
	uint32_t handled_bits;
	uint32_t handled_bits_on_int1;
	uint32_t slw_cmds_set;
	uint32_t is_nr3_done;
};

/*
 * supported operations for dcam_if device
 *
 * @open:         initialize software and hardware resource for dcam_if
 * @close:        uninitialize resource for dcam_if
 * @bind:         bind a hw ctx from dcam online or offline node
 * @unbind:       unbind hw ctx from dcam online or offline node
 */

struct dcam_pipe_ops {
	int (*open)(void *handle);
	int (*close)(void *handle);
	int (*bind)(void *handle, void *node, uint32_t node_id, uint32_t dcam_idx,
				uint32_t slw_cnt, uint32_t *hw_id, uint32_t *slw_type);
	int (*unbind)(void *handle, void *node, uint32_t node_id);
	int (*dummy_cfg)(void *ctx, void *param);
	void (*recovery)(void *handle);
};

/*
 * A nr3_me_data object carries motion vector and settings. Downstream module
 * who peforms noice reduction operation uses these information to calculate
 * correct motion vector for target image size.
 *
 * *****Note: target image should not be cropped*****
 *
 * @valid:         valid bit
 * @sub_me_bypass: sub_me_bypass bit, this has sth to do with mv calculation
 * @project_mode:  project_mode bit, 0 for step, 1 for interlace
 * @mv_x:          motion vector in x direction
 * @mv_y:          motion vector in y direction
 * @src_width:     source image width
 * @src_height:    source image height
 */
struct nr3_me_data {
	uint32_t valid:1;
	uint32_t sub_me_bypass:1;
	uint32_t project_mode:1;
	s8 mv_x;
	s8 mv_y;
	uint32_t src_width;
	uint32_t src_height;
	uint32_t full_path_mv_ready;
	uint32_t full_path_cnt;
	uint32_t bin_path_mv_ready;
	uint32_t bin_path_cnt;
};

/*
 * Retrieve a dcam_if device for the hardware. A dcam_if device is a wrapper
 * with supported operations defined in dcam_pipe_ops.
 */
void *dcam_core_pipe_dev_get(struct cam_hw_info *hw);
/*
 * Release a dcam_if device after capture finished.
 */
int dcam_core_pipe_dev_put(void *dcam_handle);

/*
 * Retrieve hardware info from dt.
 */
int dcam_drv_hw_init(void *arg);
int dcam_drv_hw_deinit(void *arg);
int dcam_drv_dt_parse(struct platform_device *pdev,
			struct cam_hw_info *hw_info,
			uint32_t *dcam_count);

#endif
