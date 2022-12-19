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

#ifndef _PYR_DEC_NODE_H_
#define _PYR_DEC_NODE_H_

#include "cam_types.h"
#include "cam_queue.h"
#include "isp_slice.h"
#include "sprd_isp_2v6.h"
#include "cam_buf_manager.h"

#ifdef PYR_DEC_DEBUG_ON
#define PYR_DEC_DEBUG pr_info
#else
#define PYR_DEC_DEBUG pr_debug
#endif

#define PYR_DEC_LAYER_NUM           5
#define PYR_DEC_BUF_Q_LEN           8
#define PYR_DEC_ADDR_NUM            PYR_DEC_LAYER_NUM + 1
#define PYR_DEC_NODE_NUM_MAX        4
#define PYR_DEC_INT_PROC_FRM_NUM    256

enum pyr_dec_node_id {
	PYR_DEC_NODE_ID,
	PYR_DEC_MAX_NODE_ID,
};

enum {
	PYR_DEC_STORE_DCT,
	PYR_DEC_STORE_DEC,
	PYR_DEC_MAX,
};

typedef int(*pyrdec_irq_proc_func)(void *handle);

static inline uint32_t pyrdec_small_layer_w(uint32_t w, uint32_t layer_num)
{
	uint32_t width = 0, i = 0;
	uint32_t w_align = PYR_DEC_WIDTH_ALIGN;

	for (i = 0; i < layer_num; i++)
		w_align *= 2;

	width = ALIGN(w, w_align);
	for (i = 0; i < layer_num; i++)
		width = width / 2;

	return width;
}

static inline uint32_t pyrdec_small_layer_h(uint32_t h, uint32_t layer_num)
{
	uint32_t height = 0, i = 0;
	uint32_t h_align = PYR_DEC_HEIGHT_ALIGN;

	for (i = 0; i < layer_num; i++)
		h_align *= 2;

	height = ALIGN(h, h_align);
	for (i = 0; i < layer_num; i++)
		height = height / 2;

	return height;
}

static inline uint32_t pyrdec_layer0_width(uint32_t w, uint32_t layer_num)
{
	uint32_t width = 0, i = 0;
	uint32_t w_align = PYR_DEC_WIDTH_ALIGN;

	for (i = 0; i < layer_num; i++)
		w_align *= 2;

	width = ALIGN(w, w_align);
	return width;
}

static inline uint32_t pyrdec_layer0_heigh(uint32_t h, uint32_t layer_num)
{
	uint32_t height = 0, i = 0;
	uint32_t h_align = PYR_DEC_HEIGHT_ALIGN;

	for (i = 0; i < layer_num; i++)
		h_align *= 2;

	height = ALIGN(h, h_align);
	return height;
}

struct pyr_dec_overlap_info {
	uint32_t slice_num;
	struct slice_pos_info slice_fetch_region[SLICE_NUM_MAX];
	struct slice_pos_info slice_store_region[SLICE_NUM_MAX];
	struct slice_overlap_info slice_fetch_overlap[SLICE_NUM_MAX];
	struct slice_overlap_info slice_store_overlap[SLICE_NUM_MAX];
};

struct pyr_dec_fetch_info {
	uint32_t bypass;
	uint32_t color_format;
	uint32_t width;
	uint32_t height;
	uint32_t pitch[2];
	uint32_t addr[2];
	uint32_t mipi_word;
	uint32_t mipi_byte;
	uint32_t mipi10_en;
	uint32_t chk_sum_clr_en;
	uint32_t ft1_axi_reorder_en;
	uint32_t ft0_axi_reorder_en;
	uint32_t substract;
	uint32_t ft1_max_len_sel;
	uint32_t ft1_retain_num;
	uint32_t ft0_max_len_sel;
	uint32_t ft0_retain_num;
};

struct pyr_dec_offline_info {
	uint32_t bypass;
	uint32_t fmcu_path_sel;
	uint32_t fetch_path_sel;
	uint32_t vector_channel_idx;
	uint32_t chksum_wrk_mode;
	uint32_t chksum_clr_mode;
	uint32_t hor_padding_en;
	uint32_t hor_padding_num;
	uint32_t ver_padding_en;
	uint32_t ver_padding_num;
	uint32_t dispatch_dbg_mode_ch0;
	uint32_t dispatch_done_cfg_mode;
	uint32_t dispatch_width_dly_num_flash;
	uint32_t dispatch_pipe_nfull_num;
	uint32_t dispatch_pipe_flush_num;
	uint32_t dispatch_pipe_hblank_num;
	uint32_t dispatch_yuv_start_order;
	uint32_t dispatch_yuv_start_row_num;
	uint32_t dispatch_width_flash_mode;
};

struct slice_pyr_dec_node_info {
	uint32_t hor_padding_en;
	uint32_t hor_padding_num;
	uint32_t ver_padding_en;
	uint32_t ver_padding_num;
	uint32_t dispatch_dly_width_num;
	uint32_t dispatch_dly_height_num;
};

struct pyr_dec_slice_desc {
	struct slice_pos_info slice_fetch_pos;
	struct slice_pos_info slice_store_dct_pos;
	struct slice_pos_info slice_store_dec_pos;
	struct slice_overlap_info slice_dct_overlap;
	struct slice_overlap_info slice_dec_overlap;
	struct slice_fetch_info slice_fetch;
	struct slice_store_info slice_dec_store;
	struct slice_store_info slice_dct_store;
	struct slice_pyr_dec_node_info slice_pyr_dec;
};

struct pyr_dec_store_info {
	uint32_t bypass;
	uint32_t endian;
	uint32_t mono_en;
	uint32_t color_format;
	uint32_t burst_len;
	uint32_t mirror_en;
	uint32_t flip_en;
	uint32_t speed2x;
	uint32_t shadow_clr_sel;
	uint32_t last_frm_en;
	uint32_t pitch[2];
	uint32_t addr[2];
	uint32_t data_10b;
	uint32_t mipi_en;
	uint32_t width;
	uint32_t height;
	uint32_t border_up;
	uint32_t border_down;
	uint32_t border_left;
	uint32_t border_right;
	uint32_t rd_ctrl;
	uint32_t shadow_clr;
	uint32_t slice_offset;
	uint32_t uvdelay_bypass;
	uint32_t uvdelay_slice_width;
};

struct pyr_dec_dct_ynr_info {
	uint32_t dct_radius;
	uint32_t old_width;
	uint32_t old_height;
	uint32_t new_width;
	uint32_t new_height;
	uint32_t sensor_width;
	uint32_t sensor_height;
	struct img_size img;
	struct img_size start[SLICE_NUM_MAX];
	struct isp_dev_dct_info *dct;
};

struct pyrdec_pipe_dev {
	uint32_t cur_node_id;
	uint32_t irq_no;
	spinlock_t ctx_lock;
	atomic_t user_cnt;
	irq_handler_t isr_func;
	uint32_t in_irq_handler;

	void *fmcu_handle;
	struct pyr_dec_node *node;
	struct cam_hw_info *hw;
	pyrdec_irq_proc_func irq_proc_func;
	struct pyr_dec_node *pyrdec_node[PYR_DEC_NODE_NUM_MAX];
	struct cam_thread_info pyrdec_irq_proc_thrd;
	struct camera_queue pyrdec_irq_sts_q;
};

struct pyr_dec_node {
	uint32_t node_id;
	/*used to connect dev&node*/
	uint32_t node_idx;
	void *isp_node;
	uint32_t isp_node_cfg_id;
	atomic_t user_cnt;
	uint32_t dcam_slice_mode;
	uint32_t is_4in1;
	uint32_t is_rawcap;
	uint32_t is_bind;
	uint32_t is_fast_stop;
	struct completion *fast_stop_done;
	struct completion frm_done;
	struct camera_frame *buf_out;
	struct isp_pipe_dev *dev;
	struct pyrdec_pipe_dev *pyrdec_dev;
	struct cam_hw_info *hw;
	struct cam_thread_info thread;
	/* lock block param to avoid acrossing frame */
	struct mutex blkpm_lock;

	uint32_t layer_num;
	uint32_t slice_num;
	uint32_t cur_layer_id;
	uint32_t cur_slice_id;
	uint32_t in_fmt;
	uint32_t pyr_out_fmt;
	uint32_t fetch_path_sel;

	struct dcam_isp_k_block isp_k_param;
	struct img_addr fetch_addr[ISP_PYR_DEC_LAYER_NUM];
	struct img_addr store_addr[PYR_DEC_ADDR_NUM];
	struct img_size src;
	struct img_size dec_padding_size;
	struct img_size dec_layer_size[PYR_DEC_ADDR_NUM];
	struct img_size sn_size;

	struct pyr_dec_fetch_info fetch_dec_info;
	struct pyr_dec_offline_info offline_dec_info;
	struct pyr_dec_store_info store_dct_info;
	struct pyr_dec_store_info store_dec_info;
	struct pyr_dec_dct_ynr_info dct_ynr_info;

	struct pyr_dec_slice_desc slices[SLICE_NUM_MAX];
	struct pyr_dec_overlap_info overlap_dec_info[MAX_PYR_DEC_LAYER_NUM];
	struct isp_fbd_yuv_info yuv_afbd_info;

	struct cam_buf_pool_id fetch_unprocess_pool;
	struct cam_buf_pool_id fetch_result_pool;
	struct cam_buf_pool_id store_result_pool;

	void *data_cb_handle;
	cam_data_cb data_cb_func;
	pyr_dec_buf_cb buf_cb_func;
};

struct pyr_dec_node_desc {
	void **node_dev;
	struct isp_pipe_dev *dev;
	struct pyrdec_pipe_dev *pyrdec_dev;
	struct cam_hw_info *hw;

	/*used to connect dev&node*/
	uint32_t node_idx;
	uint32_t in_fmt;
	uint32_t pyr_out_fmt;
	uint32_t layer_num;
	uint32_t dcam_slice_mode;
	uint32_t is_4in1;
	uint32_t is_rawcap;
	struct img_size sn_size;

	cam_data_cb data_cb_func;
	void *data_cb_handle;
	pyr_dec_buf_cb buf_cb_func;
	void *buf_cb_priv_data;
};

int pyr_dec_node_buffer_cfg(void *handle, void *param);
int pyr_dec_node_blk_param_set(void *handle, void *param);
void *pyr_dec_node_get(uint32_t node_id, struct pyr_dec_node_desc *param);
void pyr_dec_node_put(struct pyr_dec_node *node);
int pyr_dec_node_outbuf_get(void *param, void *priv_data);
int pyr_dec_node_request_proc(struct pyr_dec_node *node, void *param);
int pyr_dec_node_ctxid_cfg(void *handle, void *param);
void *pyrdec_dev_get(void *isp_handle, void *hw);
void pyrdec_dev_put(void *dec_handle);
int pyr_dec_node_fast_stop_cfg(void *handle, void *param);
int pyrdec_node_close(void *handle);
#endif
