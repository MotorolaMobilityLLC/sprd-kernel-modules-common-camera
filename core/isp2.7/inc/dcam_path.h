/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _DCAM_PATH_H_
#define _DCAM_PATH_H_

#include "dcam_core.h"

/*
 * 48M  8048 x 6036
 * 24M  5664 x 4248
 * 16M  4672 x 3504
 */
#define DCAM_PATH_WMAX  8048
#define DCAM_PATH_HMAX  6036

#define DCAM_PATH_WMAX_ROC1			6000
#define DCAM_PATH_HMAX_ROC1			4000

extern uint32_t path_ctrl_id[];

const char *to_path_name(enum dcam_path_id path_id);

int dcam_cfg_path_base(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param);

int dcam_cfg_path_size(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param);

int dcam_start_path(void *dcam_handle, struct dcam_path_desc *path);
int dcam_start_path_l5pro(void *dcam_handle, struct dcam_path_desc *path);
int dcam_stop_path(void *dcam_handle, struct dcam_path_desc *path);

int dcam_path_set_slowmotion_frame(struct dcam_pipe_dev *dev);
int dcam_path_set_skip_num(struct dcam_pipe_dev *dev,
			   int path_id, uint32_t skip_num);

/* / TODO: refine this*/
int dcam_path_set_store_frm(
			void *dcam_handle,
			struct dcam_path_desc *path,
			struct dcam_sync_helper *helper);

int dcam_set_fetch(void *dcam_handle, struct dcam_fetch_info *fetch);
void dcam_start_fetch(void);
#endif
