/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
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

#ifndef _CAM_DUMP_H_
#define _CAM_DUMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>

#include "cam_queue.h"
#include "cam_types.h"
#include "dcam_core.h"

#define CAMERA_DUMP_PATH "/data/ylog/"
/* will create thread in user to read raw buffer*/
#define BYTE_PER_ONCE 4096

enum dump_mode {
	DUMP_DISABLE = 0,
	DUMP_PATH_BOTH,
	DUMP_PATH_FULL,
	DUMP_PATH_BIN,
	DUMP_ISP_PYR_REC,
	DUMP_ISP_PYR_DEC,
	DUMP_CAM_MAX
};

enum dump_cfg {
	DUMP_CFG_OUT_FMT = 0,
	DUMP_CFG_IS_PACK,
	DUMP_CFG_PACK_BITS,
	DUMP_CFG_OUT_BITS,
	DUMP_CFG_PYR_LAYER_NUM,
	DUMP_CFG_PYR_START_LAYER,
	DUMP_CFG_MAX
};

/* for raw picture dump */
struct cam_dbg_dump {
	uint32_t dump_en;/* see enumerations above */
	uint32_t dump_count;
	uint32_t dump_ongoing;
	struct mutex dump_lock;
	struct completion *dump_start[6];
};
extern struct cam_dbg_dump g_dbg_dump;

struct cam_dump_ctx {
	struct camera_queue dump_queue;
	struct completion dump_com;
	struct timespec cur_dump_ts;
	uint32_t in_dump;
	uint32_t dump_count;
	enum cam_ch_id ch_id;
	uint32_t is_pack;
	uint32_t dcam_out_bits;
	uint32_t pack_bits;
	uint32_t dcam_out_fmt;
	int (*dump_cfg)(void *handle, uint32_t cmd, void *param);
	int (*dump_enqueue)(struct cam_dump_ctx *dump_base, struct camera_frame *pframe);
	int (*dump_file)(struct cam_dump_ctx *dump_base, struct camera_frame *pframe);

	/*pyr*/
	uint32_t pyr_layer_num;
	uint32_t start_layer;
	uint32_t is_pyr_rec;
	uint32_t is_pyr_dec;

	/*one*/
	struct dcam_compress_info fbc_info;
};

int camdump_start(struct cam_thread_info* thrd_info, struct cam_dump_ctx *dump_ctx, uint32_t dcam_idx);
int camdump_stop(struct cam_dump_ctx *dump_base);
#ifdef __cplusplus
}
#endif

#endif