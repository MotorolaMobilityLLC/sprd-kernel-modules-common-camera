/*
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#ifndef _CAM_TRUSTY_H_
#define _CAM_TRUSTY_H_

#include "sprd_img.h"

#define TA_RESP_BIT		1 <<  31

/*Todo for faceid sec funcation*/
#define CAM_FACEID_SEC
#define FACEID_VERSION 			1
#define GET_TXBUF_TIMEOUT	10
#define CA_READ_TIMEOUT		1000
#define CA_CONN_TIMEOUT 		10

enum cam_command {
	TA_FACEID_ENTER_TEMODE = 1,
	TA_FACEID_EXIT_TEMODE ,

	TA_IMG_ISP_YUV_ADDR_SET ,
	TA_IMG_ISP_PITCH_SET ,

	TA_IMG_3DNR_FETCH_ADDR_SET ,
	TA_IMG_3DNR_PITCH_SET ,

	TA_IMG_DCAM_YUV_ADDR_SET ,
	TA_IMG_DCAM_PITCH_SET ,

	TA_IMG_CSI_SWICH_CTRL_SET ,

	TA_REG_SET ,
	TA_REG_GET ,
	TA_GET_CAM_STATUS,
};

enum cam_trusty_mode {
	CAM_TRUSTY_ENTER,
	CAM_TRUSTY_EXIT,
	CAM_TRUSTY_MAX,
};

struct img_yuv_reg_msg {
	uint32_t msg_cmd;
	unsigned long y_addr;
	unsigned long u_addr;
	unsigned long v_addr;
};

struct img_pitch_reg_msg {
	uint32_t msg_cmd;
	uint32_t y_pitch;
	uint32_t u_pitch;
	uint32_t v_pitch;
};

struct isp_3dnr_reg_msg {
	uint32_t msg_cmd;
	uint32_t  ft_luma_addr;
	uint32_t  ft_chroma_addr;
	uint32_t  ft_pitch;
};

struct csi_switch_ctrl_msg {
	uint32_t msg_cmd;
	uint32_t csi_sel_ctrl;
};

struct faceid_cfg_msg {
	uint32_t msg_cmd;
	uint32_t faceid_version;
	struct  sprd_cam_sec_cfg   sec_cfg;
};

enum cam_cmdack {
	TA_CMD_DONE = 0,
	TA_CMD_ERR ,
};

enum cam_ta_staus {
	CAM_NORMAL  = 0,
	CAM_SECURITY,
};

struct faceid_info_msg {
   	uint32_t msg_cmd;
	enum cam_ta_staus   sec_attr ;
	enum sprd_cam_sec_mode   sec_mode;
	enum sprd_cam_sec_mode  work_mode;
	uint64_t reserved_mem_addr;
	uint32_t reserved_mem_size;
};

struct ack_message {
	uint32_t cmd;
	enum cam_cmdack ack;
};

struct status_message {
	uint32_t cmd;
	enum cam_ta_staus status;
};

struct cam_ca_ctrl {
	int   chanel_state;
	bool con_init;
	bool cam_temode;
	struct mutex wlock;
	struct mutex rlock;
	struct tipc_chan *chan;
	wait_queue_head_t readq;
	struct list_head rx_msg_queue;
};

bool cam_ca_connect(void);
void cam_ca_disconnect(void);
bool camca_isp_fetch_addr_set(unsigned long y_addr, unsigned long u_addr,
										unsigned long v_addr);
bool  camca_isp_pitch_set(uint32_t y_pitch, uint32_t u_pitch,uint32_t v_pitch);
bool  camca_isp_3dnr_fetch_set(uint32_t chroma, uint32_t luma, uint32_t pitch);
bool  camca_csi_switch_ctrl_set(uint32_t csi_sel_ctrl);
bool  camca_security_set(struct  sprd_cam_sec_cfg *camsec_cfg, enum cam_trusty_mode mode);

#endif
