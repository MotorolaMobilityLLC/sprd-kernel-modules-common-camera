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

#ifndef _CAM_CORE_H_
#define _CAM_CORE_H_

#include "cam_pipeline.h"
#include "cam_debugger.h"
#include "cam_scene.h"

#define IMG_DEVICE_NAME                 "sprd_image"
#define CAMERA_TIMEOUT                  5000
#define CAMERA_WATCHDOG_TIMEOUT         1000
#define CAMERA_RECOVERY_TIMEOUT         100
#define CAMERA_LONGEXP_TIMEOUT          50000
#define THREAD_STOP_TIMEOUT             3000
#define FAST_STOP_TIME_OUT              200

#define CAM_COUNT                       CAM_ID_MAX
#define CAM_SHARED_BUF_NUM              50
#define CAM_FRAME_Q_LEN                 60
#define CAM_IRQ_Q_LEN                   16
#define CAM_STATIS_Q_LEN                16
#define CAM_ZOOM_COEFF_Q_LEN            64
#define CAM_ALLOC_Q_LEN                 24
#define CAM_RESERVE_BUF_Q_MAX           500
#define CAM_FRAME_DEAL                  1
#define CAM_FRAME_NO_DEAL               0

/* TODO: tuning ratio limit for power/image quality */
#define RATIO_SHIFT                     16

/* TODO: need to pass the num to driver by hal */
#define CAP_NUM_COMMON                  1

enum camera_module_state {
	CAM_INIT = 0,
	CAM_IDLE,
	CAM_CFG_CH,
	CAM_STREAM_ON,
	CAM_STREAM_OFF,
	CAM_RUNNING,
	CAM_ERROR,
};

enum camera_recovery_state {
	CAM_RECOVERY_NONE = 0,
	CAM_RECOVERY_RUNNING,
	CAM_RECOVERY_MAX,
};

/* Static Variables Declaration */
static uint32_t output_img_fmt[] = {
	/* CAM_YUV420_2FRAME */
	IMG_PIX_FMT_NV12,
	/* CAM_YVU420_2FRAME */
	IMG_PIX_FMT_NV21,
	IMG_PIX_FMT_GREY,
};

#define COMPAT_SPRD_ISP_IO_CFG_PARAM \
	_IOWR(SPRD_IMG_IO_MAGIC, 41, struct compat_isp_io_param)

struct compat_isp_io_param {
	uint32_t scene_id;
	uint32_t sub_block;
	uint32_t property;
	u32 property_param;
};

struct camera_uchannel {
	uint32_t sn_fmt;
	uint32_t dst_fmt;

	uint32_t is_high_fps;/* for DCAM slow motion feature */
	uint32_t high_fps_skip_num;/* for DCAM slow motion feature */
	uint32_t slowmotion_stage_a_num;/* for DCAM slow motion feature 60 frame*/
	uint32_t slowmotion_stage_a_valid_num;/* for DCAM slow motion feature */
	uint32_t slowmotion_stage_b_num;/* for DCAM slow motion feature 90 frame*/

	int32_t sensor_raw_fmt;
	int32_t dcam_raw_fmt;
	uint32_t dcam_out_fmt;
	uint32_t pyr_out_fmt;

	struct sprd_img_rect zoom_ratio_base;
	struct img_size src_size;
	struct sprd_img_rect src_crop;
	struct sprd_img_rect total_src_crop;
	struct img_size dst_size;

	/* for binding small picture */
	uint32_t slave_img_en;
	uint32_t slave_img_fmt;
	struct img_size slave_img_size;

	/* for close callback stream frame sync */
	uint32_t frame_sync_close;

	struct dcam_regular_desc regular_desc;
	struct sprd_vir_ch_info vir_channel[VIR_CH_NUM];
};

struct camera_uinfo {
	struct sprd_img_sensor_if sensor_if;
	struct img_size sn_size;
	struct img_size sn_max_size;
	struct sprd_img_rect sn_rect;
	uint32_t capture_mode;
	uint32_t capture_skip;
	uint32_t is_longexp;
	uint32_t is_4in1;
	uint32_t is_rgb_ltm;
	uint32_t is_pyr_rec;
	uint32_t is_pyr_dec;
	uint32_t is_rgb_gtm;
	uint32_t is_dual;
	uint32_t dcam_slice_mode;/*1: hw,  2:sw*/
	uint32_t slice_num;
	uint32_t slice_count;
	uint32_t zsl_num;
	uint32_t zsk_skip_num;
	uint32_t need_share_buf;
	uint32_t gtm_ltm_sw_calc;
	uint32_t zoom_conflict_with_ltm;
	/* for raw alg*/
	uint32_t is_raw_alg;
	uint32_t raw_alg_type;
	uint32_t param_frame_sync;
	/* for dcam raw*/
	uint32_t need_dcam_raw;
	uint32_t virtualsensor;/* 1: virtual sensor 0: normal */
	uint32_t opt_buffer_num;
};

struct sprd_img_flash_info {
	uint32_t led0_ctrl;
	uint32_t led1_ctrl;
	uint32_t led0_status;
	uint32_t led1_status;
	uint32_t flash_last_status;
	struct sprd_img_set_flash set_param;
};

struct channel_context {
	enum cam_ch_id ch_id;
	uint32_t enable;
	uint32_t frm_base_id;
	uint32_t frm_cnt;
	atomic_t err_status;

	uint32_t compress_en;
	uint32_t compress_3dnr;
	uint32_t compress_offline;

	int32_t dcam_port_id;
	int32_t aux_dcam_port_id;
	int32_t aux_raw_port_id;
	int32_t isp_port_id;
	uint32_t need_framecache;

	uint32_t zsl_buffer_num;
	uint32_t zsl_skip_num;

	uint32_t pipeline_type;
	struct cam_pipeline *pipeline_handle;

	struct camera_uchannel ch_uinfo;
	struct img_size swap_size;
	struct img_trim trim_dcam;
	struct img_trim total_trim_dcam;
	struct img_trim trim_isp;
	struct img_size dst_dcam;
	uint32_t dcam_out_fmt; /*for online path*/

	uint32_t alloc_start;
	struct completion alloc_com;
	struct completion stream_on_buf_com;
	struct completion fast_stop;
	uint32_t uinfo_3dnr;/* set by hal, 1:hw 3dnr; */
	uint32_t type_3dnr;/* CAM_3DNR_HW:enable hw,and alloc buffer */
	uint32_t mode_ltm;
	uint32_t ltm_rgb;
	uint32_t pyr_layer_num;
	uint32_t mode_gtm;
	uint32_t gtm_rgb;
	struct cam_zoom_frame latest_zoom_param;
	struct sprd_img_rect latest_user_crop;/*use to compare with current crop*/
	spinlock_t lastest_zoom_lock;

	struct camera_queue zoom_user_crop_q;/* channel specific coef queue */
	struct camera_queue zoom_param_q;
	struct dcam_isp_k_block blk_pm;
};

struct camera_module {
	uint32_t idx;
	atomic_t state;
	atomic_t timeout_flag;
	uint32_t timeout_num;
	struct mutex ioctl_lock;
	uint32_t master_flag; /* master cam capture flag */
	uint32_t compat_flag;
	struct camera_group *grp;
	uint32_t private_key;
	int attach_sensor_id;
	uint32_t iommu_enable;

	uint32_t raw_cap_fetch_fmt;
	enum cam_cap_type capture_type;

	struct isp_pipe_dev *isp_dev_handle;
	struct dcam_pipe_dev *dcam_dev_handle;
	struct pyrdec_pipe_dev *pyrdec_dev_handle;
	struct cam_flash_task_info *flash_core_handle;
	uint32_t dcam_idx;

	uint32_t zoom_solution;/* for dynamic zoom type swicth. */
	uint32_t binning_limit;
	struct camera_uinfo cam_uinfo;

	uint32_t last_channel_id;
	struct channel_context channel[CAM_CH_MAX];
	struct mutex buf_lock[CAM_CH_MAX];

	struct completion frm_com;
	struct camera_queue frm_queue;/* frame message queue for user*/
	struct camera_queue alloc_queue;/* alloc data queue or user*/

	struct cam_thread_info zoom_thrd;
	struct cam_thread_info buf_thrd;

	uint32_t opt_frame_fid;/* mul_frame opt frame fid*/

	struct timer_list cam_timer;

	struct camera_frame *dual_frame;/* 0: no, to find, -1: no need find */
	atomic_t capture_frames_dcam;/* how many frames to report, -1:always */
	atomic_t cap_total_frames;
	atomic_t cap_skip_frames;
	atomic_t cap_zsl_frames;
	atomic_t dual_select_frame_done;
	int64_t capture_times;/* *ns, timestamp get from start_capture */
	uint32_t capture_scene;
	struct camera_queue remosaic_queue;/* 4in1: save camera_frame when remosaic */
	uint32_t auto_3dnr;/* 1: enable hw,and alloc buffer before stream on */
	struct sprd_img_flash_info flash_info;
	uint32_t flash_skip_fid;
	uint32_t path_state;

	uint32_t raw_callback;
	struct mutex zoom_lock;
	/* cam data path topology info */
	struct cam_scene *static_topology;
	struct cam_nodes_dev nodes_dev;
	struct camera_frame *res_frame;
	uint32_t reserved_buf_fd;
	int reserved_pool_id;
	uint32_t dcam_ctx_bind_state;/* 0: dcam_ctx_unbind, 1: dcam_ctx_bind */
	uint32_t is_flash_status;
	uint32_t simu_fid;
};

struct camera_group {
	atomic_t camera_opened;
	bool ca_conn;

	spinlock_t module_lock;
	uint32_t module_used;
	struct camera_module *module[CAM_COUNT];

	spinlock_t dual_deal_lock;
	uint32_t dcam_count;/*dts cfg dcam count*/
	atomic_t runner_nr; /*running camera num*/
	atomic_t mul_buf_alloced;
	struct mutex mulshare_buf_lock;

	struct miscdevice *md;
	struct platform_device *pdev;
	struct camera_queue ion_buf_q;
	struct camera_queue empty_frm_q;
	struct sprd_cam_sec_cfg camsec_cfg;
	struct camera_debugger debugger;
	struct cam_hw_info *hw_info;
	struct img_size mul_sn_max_size;
	uint32_t is_mul_buf_share;
	struct wakeup_source *ws;

	atomic_t recovery_state;
	struct rw_semaphore switch_recovery_lock;
	struct cam_thread_info recovery_thrd;

	/* cam data path topology info */
	struct cam_scene topology_info;
};

struct cam_ioctl_cmd {
	unsigned int cmd;
	int (*cmd_proc)(struct camera_module *module, unsigned long arg);
};

#endif
