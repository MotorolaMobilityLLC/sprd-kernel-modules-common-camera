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

#ifndef _CAM_TYPES_H_
#define _CAM_TYPES_H_

#include <linux/completion.h>
#include "sprd_img.h"
#include "cam_kernel_adapt.h"
#include <linux/spinlock.h>
#include "cam_thread.h"

#ifndef MAX
#define MAX(__a, __b) (((__a) > (__b)) ? (__a) : (__b))
#endif
#ifndef MIN
#define MIN(__a, __b) (((__a) < (__b)) ? (__a) : (__b))
#endif

#define ZOOM_RATIO_DEFAULT              1000
#define CAM_BUF_ALIGN_SIZE              4
#define ALIGN_OFFSET                    16
#define PYR_DEC_WIDTH_ALIGN             4
#define PYR_DEC_HEIGHT_ALIGN            2
/* To avoid rec fifo err, isp fetch burst_lens = 8, then MIN_PYR_WIDTH >= 128;
 isp fetch burst_lens = 16, then MIN_PYR_WIDTH >= 256. */
#define MIN_PYR_WIDTH                   128
#define MIN_PYR_HEIGHT                  16
#define PYR_IS_PACK                     1
#define IS_PACK_3DNR                    1
#define ALIGN_UP(a, x)                  (((a) + (x) - 1) & (~((x) - 1)))

/*************** for global debug starts********************/

/* for iommu
 * 0: auto - dts configured
 * 1: iommu off and reserved memory
 * 2: iommu on and reserved memory
 * 3: iommu on and system memory
 */
enum cam_iommu_mode {
	IOMMU_AUTO = 0,
	IOMMU_OFF,
	IOMMU_ON_RESERVED,
	IOMMU_ON,
};
extern int g_dbg_iommu_mode;
extern int g_dbg_set_iommu_mode;
extern uint32_t g_pyr_dec_online_bypass;
extern uint32_t g_pyr_dec_offline_bypass;
extern uint32_t g_dcam_raw_src;
extern uint32_t g_dbg_fbc_control;
extern uint32_t g_dbg_recovery;
#define VOID_PTR_TO(from_param, type) \
({	type *__a = NULL; \
	if (!(from_param)) { \
		pr_err("fail to get param\n"); \
		return -1; \
	} \
	__a = (type *)(from_param); \
})

/* for global camera control */
struct cam_global_ctrl {
	uint32_t dcam_zoom_mode;
	uint32_t isp_wmode;
	uint32_t isp_linebuf_len;
};
extern struct cam_global_ctrl g_camctrl;

/* for memory leak debug */
struct cam_mem_dbg_info {
	atomic_t ion_alloc_cnt;
	atomic_t ion_alloc_size;
	atomic_t ion_kmap_cnt;
	atomic_t ion_dma_cnt;
	atomic_t iommu_map_cnt[6];
	atomic_t empty_frm_cnt;
	atomic_t empty_interruption_cnt;
	atomic_t mem_kzalloc_cnt;
	atomic_t mem_vzalloc_cnt;
	atomic_t empty_zoom_cnt;
	struct camera_queue *empty_memory_alloc_queue;
	struct camera_queue *memory_alloc_queue;
	struct camera_queue *buf_map_calc_queue;
	uint32_t g_dbg_memory_leak_ctrl;
};
extern struct cam_mem_dbg_info *g_mem_dbg;

/*************** for global debug ends ********************/

enum camera_slice_mode {
	CAM_SLICE_NONE = 0,
	CAM_OFFLINE_SLICE_HW,
	CAM_OFFLINE_SLICE_SW
};

enum camera_cap_type {
	CAM_CAP_NORMAL = 0,
	CAM_CAP_RAW_FULL,
	CAM_CAP_RAW_BIN
};

enum camera_id {
	CAM_ID_0 = 0,
	CAM_ID_1,
	CAM_ID_2,
	CAM_ID_3,
	CAM_ID_MAX,
};

enum cam_cap_type {
	CAM_CAPTURE_STOP = 0,
	CAM_CAPTURE_START,
	CAM_CAPTURE_RAWPROC,
	CAM_CAPTURE_RAWPROC_DONE,
	CAM_CAPTURE_START_3DNR,
	CAM_CAPTURE_START_WITH_TIMESTAMP,
	CAM_CAPTURE_START_FROM_NEXT_SOF,
	CAM_CAPTURE_MAX,
};

enum cam_ch_id {
	CAM_CH_RAW = 0,
	CAM_CH_PRE,
	CAM_CH_VID,
	CAM_CH_CAP,
	CAM_CH_PRE_THM,
	CAM_CH_CAP_THM,
	CAM_CH_VIRTUAL,
	CAM_CH_DCAM_VCH,
	CAM_CH_MAX,
};

enum dump_channel_type {
	DUMP_CH_RES = 0,
	DUMP_CH_PRE,
	DUMP_CH_CAP,
	DUMP_CH_MAX,
};

enum cam_3dnr_type {
	CAM_3DNR_OFF = 0,
	CAM_3DNR_HW,
	CAM_3DNR_SW,
};

enum cam_slw_state {
	CAM_SLOWMOTION_OFF = 0,
	CAM_SLOWMOTION_ON,
};

enum cam_data_endian {
	ENDIAN_LITTLE = 0,
	ENDIAN_BIG,
	ENDIAN_HALFBIG,
	ENDIAN_HALFLITTLE,
	ENDIAN_MAX
};

enum dcam_output_strategy {
	IMG_POWER_CONSUM_PRI,
	IMG_QUALITY_PRI,
	IMG_STRATEGY_TYPEMAX,
};

enum cam_zoom_type {
	ZOOM_DEFAULT = 0,
	ZOOM_BINNING2,
	ZOOM_BINNING4,
	ZOOM_SCALER,
	ZOOM_TYPEMAX,
	ZOOM_DEBUG_DEFAULT = 10,
	ZOOM_DEBUG_BINNING2,
	ZOOM_DEBUG_BINNING4,
	ZOOM_DEBUG_SCALER,
	ZOOM_DEBUG_TYPEMAX,
};

enum cam_frame_scene {
	CAM_FRAME_COMMON = 0,
	CAM_FRAME_FDRL,
	CAM_FRAME_FDRH,
	CAM_FRAME_PRE_FDR,
	CAM_FRAME_DRC,
	CAM_FRAME_PROCESS_RAW,
	CAM_FRAME_ORIGINAL_RAW,
};

struct img_addr {
	uint32_t addr_ch0;
	uint32_t addr_ch1;
	uint32_t addr_ch2;
};

struct img_pitch {
	uint32_t pitch_ch0;
	uint32_t pitch_ch1;
	uint32_t pitch_ch2;
};

struct img_size {
	uint32_t w;
	uint32_t h;
};

struct img_border {
	uint32_t border_up;
	uint32_t border_down;
	uint32_t border_left;
	uint32_t border_right;
};

struct img_trim {
	uint32_t start_x;
	uint32_t start_y;
	uint32_t size_x;
	uint32_t size_y;
};

struct img_scaler_info {
	struct img_size src_size;
	struct img_trim src_trim;
	struct img_size dst_size;
};

enum share_buf_cb_type {
	SHARE_BUF_GET_CB,
	SHARE_BUF_SET_CB,
	SHARE_BUF_MAX_CB,
};

enum reserved_buf_cb_type {
	RESERVED_BUF_GET_CB,
	RESERVED_BUF_SET_CB,
	RESERVED_BUF_MAX_CB,
};

enum cam_reserved_buf_type {
	CAM_RESERVED_BUFFER_ORI = 1,
	CAM_RESERVED_BUFFER_COPY,
};

struct dcam_compress_info {
	uint32_t is_compress;
	uint32_t tile_col;
	uint32_t tile_row;
	uint32_t tile_row_lowbit;
	uint32_t header_size;
	uint32_t payload_size;
	uint32_t lowbits_size;
	uint32_t buffer_size;
};

struct compressed_addr {
	uint32_t addr0;
	uint32_t addr1;
	uint32_t addr2;
	uint32_t addr3;
};

struct dcam_compress_cal_para {
	uint32_t fmt;
	uint32_t data_bits;
	uint32_t width;
	uint32_t height;
	struct dcam_compress_info *fbc_info;
	unsigned long in;
	struct compressed_addr *out;
};

/* dev fetch & store data type*/
enum cam_format {
	CAM_RAW_BASE = 0x10,
	CAM_RAW_PACK_10 = 0x0,
	CAM_RAW_HALFWORD_10,
	CAM_RAW_14,
	CAM_RAW_8,
	CAM_RGB_BASE = 0x20,
	CAM_FULL_RGB10,
	CAM_FULL_RGB14,
	CAM_YUV_BASE = 0x40,
	CAM_YUV422_2FRAME,
	CAM_YVU422_2FRAME,
	CAM_YUV420_2FRAME,
	CAM_YVU420_2FRAME,
	CAM_YUV420_2FRAME_10,
	CAM_YVU420_2FRAME_10,
	CAM_YUV420_2FRAME_MIPI,
	CAM_YVU420_2FRAME_MIPI,
	CAM_YUV422_3FRAME,
	CAM_YUV420_3FRAME,
	CAM_YUYV_1FRAME,
	CAM_UYVY_1FRAME,
	CAM_YVYU_1FRAME,
	CAM_VYUY_1FRAME,
	CAM_FORMAT_MAX
};

/* dev fetch & store data bw*/
enum cam_bit_width {
	CAM_UNSUPPORT = 0,
	CAM_1_BITS = 1,
	CAM_2_BITS = 2,
	CAM_4_BITS = 4,
	CAM_8_BITS = 8,
	CAM_10_BITS = 10,
	CAM_12_BITS = 12,
	CAM_14_BITS = 14,
	CAM_BIT_MAX
};

struct cam_capture_param {
	uint32_t cap_type;
	uint32_t cap_scene;
	atomic_t cap_cnt;
	int64_t cap_timestamp;
	struct sprd_img_rect cap_user_crop;
};

struct cam_buf_alloc_desc {
	uint32_t height;
	uint32_t width;
	uint32_t is_pack;
	uint32_t ch_id;
	uint32_t compress_en;
	uint32_t compress_offline;
	uint32_t dcamonline_out_fmt;
	uint32_t dcamoffline_out_fmt;
	uint32_t is_pyr_rec;
	uint32_t is_pyr_dec;
	uint32_t pyr_out_fmt;
	uint32_t cam_idx;
	uint32_t sec_mode;
	uint32_t dcamonline_buf_alloc_num;
	uint32_t dcamoffline_buf_alloc_num;
	uint32_t sensor_img_ptn;
	uint32_t share_buffer;
	uint32_t iommu_enable;
	uint32_t stream_on_need_buf_num;
	struct completion *stream_on_buf_com;
};

enum cam_user_data_type {
	CAM_USERDATA_PREVIEW,
	CAM_USERDATA_VIDEO,
	CAM_USERDATA_CAPTURE,
	CAM_USERDATA_THUMBNAIL,
	CAM_USERDATA_AEM,
	CAM_USERDATA_AFM,
	CAM_USERDATA_PDAF,
	CAM_USERDATA_AFL,
	CAM_USERDATA_LSCM,
	CAM_USERDATA_3DNRME,
	CAM_USERDATA_BAYERHIST,
	CAM_USERDATA_HIST2,
	CAM_USERDATA_GTMHIST,
	CAM_USERDATA_LTMHIST,
	CAM_USERDATA_TYPE_NUM,
};

struct slowmotion_960fps_info {
	uint32_t slowmotion_stage_a_num;
	uint32_t slowmotion_stage_b_num;
	uint32_t slowmotion_stage_a_valid_num;
};

enum shutoff_scene {
	SHUTOFF_MULTI_PORT_SWITCH = 0,
	SHUTOFF_SINGLE_PORT_NONZSL,
	SHUTOFF_SCENE_MAX,
};

enum shutoff_type {
	SHUTOFF_PAUSE = 0,
	SHUTOFF_RESUME,
	SHUTOFF_TYPE_MAX,
};

extern struct camera_queue *g_ion_buf_q;
extern struct camera_queue *g_empty_zoom_q;
extern struct camera_queue *g_empty_frm_q;

typedef int(*isp_dev_callback)(enum cam_cb_type type, void *param,
				void *priv_data);
typedef int(*dcam_dev_callback)(enum cam_cb_type type, void *param,
				void *priv_data);
typedef int (*pyr_dec_buf_cb)(void *param, void *cb_handle);
typedef int(*share_buf_get_cb)(enum share_buf_cb_type type, void *param,
				void *cb_handle);
typedef int(*reserved_buf_get_cb)(enum reserved_buf_cb_type type, void *param,
				void *cb_handle);
typedef struct camera_frame *(*dual_frame_sync_cb)(void *param, void *cb_handle, int *flag);
typedef int(*dual_slave_frame_set_cb)(void *param, void *cb_handle);
typedef int(*port_cfg_cb)(void *param, uint32_t cmd, void *cb_handle);
typedef int(*shutoff_cfg_cb)(void *cb_handle, uint32_t cmd, void *param);
typedef int(*dcam_irq_proc_cb)(void *param, void *cb_handle);
typedef void *(*zoom_get_cb)(void *cb_handle);
typedef void *(*cam_zoom_get_cb)(uint32_t param, void *cb_handle);

#endif/* _CAM_TYPES_H_ */
