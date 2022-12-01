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

#ifndef _DCAM_CORE_H_
#define _DCAM_CORE_H_

#include "sprd_img.h"
#include <linux/sprd_ion.h>

#include "cam_types.h"
#include "cam_queue.h"
#include "cam_block.h"
#include "cam_hw.h"
#include "dcam_interface.h"
#include "dcam_int.h"
#include "dcam_fmcu.h"
#include "dcam_online_node.h"
#include "dcam_offline_node.h"
#include "dcam_fetch_node.h"

#define DCAM_IN_Q_LEN                     12
#define DCAM_PROC_Q_LEN                   12
#define DCAM_IRQ_Q_LEN                    256
#define DCAM_FULL_MV_Q_LEN                12
#define DCAM_BIN_MV_Q_LEN                 12
#define DCAM_MIDDLE_Q_LEN                 50

/* TODO: extend these for slow motion dev */
#define DCAM_RESULT_Q_LEN                 50
#define DCAM_OUT_BUF_Q_LEN                50
#define DCAM_RESERVE_BUF_Q_LEN            50

#define DCAM_LSC_BUF_SIZE                 0x3000
#define DCAM_INT_PROC_FRM_NUM             256
#define DCAM_LSC_WEIGHT_TAB_SIZE          774

#define DCAM_OFFLINE_TIMEOUT              msecs_to_jiffies(2000)
#define DCAM_OFFLINE_SLC_MAX              3

#define DCAM_ADDR_RECORD_FRAME_NUM        5

/* get index of timestamp from frame index */
#define tsid(x)                           ((x) & (DCAM_FRAME_TIMESTAMP_COUNT - 1))
#define DCAM_FETCH_TWICE(p)               (p->raw_fetch_num > 1)
#define DCAM_FIRST_FETCH(p)               (p->raw_fetch_count == 1)
#define DCAM_LAST_FETCH(p)                (p->raw_fetch_count == 2)

#define DCAM_NR3_MV_MAX                   10

enum dcam_context_id {
	DCAM_CTX_0,
	DCAM_CTX_1,
	DCAM_CTX_2,
	DCAM_CTX_3,
	DCAM_CTX_NUM,
};

enum dcam_scaler_type {
	DCAM_SCALER_BINNING = 0,
	DCAM_SCALER_BY_YUVSCALER,
	DCAM_SCALER_BYPASS,
	DCAM_SCALER_MAX,
};

enum dcam_path_state {
	DCAM_PATH_IDLE,
	DCAM_PATH_PAUSE,
	DCAM_PATH_RESUME,
};

enum dcam_csi_state {
	DCAM_CSI_IDLE,
	DCAM_CSI_PAUSE,
	DCAM_CSI_RESUME,
};

/*
 * state machine for DCAM
 *
 * @INIT:             initial state of this dcam_pipe_dev
 * @IDLE:             after hardware initialized, power/clk/int should be
 *                    prepared, which indicates registers are ready for
 *                    accessing
 * @RUNNING:          this dcam_pipe_dev is receiving data from CSI or DDR, and
 *                    writing data from one or more paths
 * @ERROR:            something is wrong, we should reset hardware right now
 */
enum dcam_state {
	STATE_INIT = 0,
	STATE_IDLE = 1,
	STATE_RUNNING = 2,
	STATE_ERROR = 3,
};

struct dcam_offline_slice_info {
	uint32_t dcam_slice_mode;
	uint32_t slice_num;
	uint32_t slice_count;
	uint32_t fetch_fmt;
	struct img_trim *cur_slice;
	struct img_trim slice_trim[DCAM_OFFLINE_SLC_MAX];
};

struct dcam_online_start_param {
	void *lbuf_param;
	void *param;
};

struct dcam_hw_path {
	uint32_t need_update;
	uint32_t frm_deci;
	uint32_t frm_deci_cnt;
	struct img_trim in_trim;
	struct dcam_hw_cfg_store_addr hw_store;
	struct dcam_hw_path_size hw_size;
	struct dcam_hw_path_start hw_start;
	struct dcam_hw_fbc_addr hw_fbc_store;
	struct dcam_hw_fbc_ctrl hw_fbc;
};

struct dcam_hw_context {
	uint32_t is_offline_proc;
	uint32_t is_virtualsensor_proc;
	uint32_t fid;
	uint32_t base_fid;
	uint32_t frame_index;
	uint32_t index_to_set;
	uint32_t slw_idx;
	bool need_fix;
	atomic_t user_cnt;
	uint32_t is_4in1;
	uint32_t irq;
	uint32_t irq_enable;
	uint32_t node_id;
	uint32_t hw_ctx_id;
	uint32_t handled_bits;
	uint32_t handled_bits_on_int1;
	uint32_t in_irq_proc;
	uint32_t dcam_slice_mode;
	uint32_t slowmotion_count;
	spinlock_t glb_reg_lock;
	void *node;

	uint32_t zoom_ratio;
	uint32_t total_zoom;
	struct img_trim next_roi;
	uint32_t is_pyr_rec;
	uint32_t iommu_status;
	uint32_t err_count;/* iommu register dump count in dcam_err */
	uint32_t dec_all_done;
	uint32_t dec_layer0_done;
	atomic_t shadow_done_cnt;
	atomic_t shadow_config_cnt;
	uint32_t prev_fbc_done;
	uint32_t cap_fbc_done;
	spinlock_t fbc_lock;
	struct dcam_offline_slice_info slice_info;
	struct dcam_fmcu_ctx_desc *fmcu;
	struct cam_hw_info *hw;
	struct dcam_hw_fetch_set hw_fetch;
	struct dcam_hw_path hw_path[DCAM_PATH_MAX];
	struct dcam_isp_k_block *blk_pm;
	struct dcam_hw_slw_fmcu_cmds slw;
	struct dcam_mipi_info cap_info;
	struct nr3_me_data nr3_mv_ctrl[DCAM_NR3_MV_MAX];

	void *dcam_irq_cb_handle;
	dcam_irq_proc_cb dcam_irq_cb_func;
	spinlock_t ghist_read_lock;
	uint32_t gtm_hist_value[GTM_HIST_VALUE_SIZE];
	uint32_t frame_addr[DCAM_ADDR_RECORD_FRAME_NUM][DCAM_RECORD_PORT_INFO_MAX];
};

struct dcam_pipe_dev {
	atomic_t user_cnt;
	atomic_t enable;
	struct dcam_hw_context hw_ctx[DCAM_HW_CONTEXT_MAX];
	struct mutex ctx_mutex;
	struct mutex path_mutex;
	spinlock_t ctx_lock;
	struct dcam_pipe_ops *dcam_pipe_ops;
	struct cam_hw_info *hw;
};

enum dcam_bind_mode {
	DCAM_BIND_FIXED = 0,
	DCAM_BIND_DYNAMIC,
	DCAM_BIND_MAX
};

enum camera_csi_switch_mode {
	CAM_CSI_NORMAL_SWITCH,
	CAM_CSI_RECOVERY_SWITCH,
	CAM_CSI_MAX_SWITCH,
};

struct dcam_switch_param {
	uint32_t csi_id;
	uint32_t dcam_id;
	uint32_t is_recovery;
};

struct dcam_csi_reset_param {
	uint32_t mode;
	uint32_t csi_connect_stat;
	void *param;
};

static inline uint32_t cal_sprd_pitch(uint32_t w, uint32_t fmt)
{
	uint32_t pitchsize = 0;

	switch (fmt) {
	case CAM_YUV422_2FRAME:
	case CAM_YVU422_2FRAME:
	case CAM_YUV420_2FRAME:
	case CAM_YVU420_2FRAME:
	case CAM_YUV422_3FRAME:
	case CAM_YUV420_3FRAME:
	case CAM_YUYV_1FRAME:
	case CAM_UYVY_1FRAME:
	case CAM_YVYU_1FRAME:
	case CAM_VYUY_1FRAME:
		pitchsize = w;
		break;
	case CAM_RAW_14:
	case CAM_RAW_8:
	case CAM_RAW_HALFWORD_10:
		pitchsize = CAL_UNPACK_PITCH(w);
		break;
	case CAM_YUV420_2FRAME_10:
	case CAM_YVU420_2FRAME_10:
		pitchsize = (w * 16 + 127) / 128 * 128 / 8;
		break;
	case CAM_RAW_PACK_10:
		pitchsize = CAL_PACK_PITCH(w);
		break;
	case CAM_YUV420_2FRAME_MIPI:
	case CAM_YVU420_2FRAME_MIPI:
		pitchsize = (w * 10 + 127) / 128 * 128 / 8;
		break;
	case CAM_FULL_RGB14:
		pitchsize = CAL_FULLRGB14_PITCH(w);
		break;
	default :
		pr_err("fail to get fmt : %d\n", fmt);
		break;
	}

	return pitchsize;
}

static inline uint32_t cal_sprd_size(uint32_t w, uint32_t h, uint32_t fmt)
{
	uint32_t size = 0;

	switch (fmt) {
	case CAM_YUV420_2FRAME:
	case CAM_YVU420_2FRAME:
	case CAM_YUV420_2FRAME_MIPI:
	case CAM_YVU420_2FRAME_MIPI:
	case CAM_YUV420_3FRAME:
		size = cal_sprd_pitch(w, fmt) * h * 3 / 2;
		break;
	case CAM_RAW_14:
	case CAM_RAW_8:
	case CAM_RAW_HALFWORD_10:
	case CAM_RAW_PACK_10:
	case CAM_RGB_BASE:
	case CAM_FULL_RGB10:
	case CAM_FULL_RGB14:
		size = cal_sprd_pitch(w, fmt) * h;
		break;
	case CAM_YUV422_2FRAME:
	case CAM_YVU422_2FRAME:
	case CAM_YUV422_3FRAME:
	case CAM_YUYV_1FRAME:
	case CAM_UYVY_1FRAME:
	case CAM_YVYU_1FRAME:
	case CAM_VYUY_1FRAME:
		size = cal_sprd_pitch(w, fmt) * h * 2;
		break;
	case CAM_YVU420_2FRAME_10:
	case CAM_YUV420_2FRAME_10:
		size = w * h * 3;
		break;
	default :
		pr_err("fail to get fmt : %d\n", fmt);
		break;
	}

	return size;
}

static inline uint32_t cal_sprd_yuv_pitch(uint32_t w, uint32_t dcam_out_bits, uint32_t is_pack)
{
	if (dcam_out_bits != CAM_8_BITS) {
		if(is_pack)
			w = (w * 10 + 127) / 128 * 128 / 8;
		else
			w = (w * 16 + 127) / 128 * 128 / 8;
	}

	return w;
}
void dcam_core_offline_irq_proc(struct dcam_hw_context *dcam_hw_ctx,
		struct dcam_irq_info *irq_info);

#endif
