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
#include <linux/uaccess.h>
#include <sprd_mm.h>

#include "sprd_isp_hw.h"
#include "isp_reg.h"
#include "cam_types.h"
#include "cam_block.h"

#include "isp_ltm.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "LTM MAP: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static struct isp_dev_ltm_info g_ltm_info_pre = {
	.ltm_stat.bypass  = 0,
	.ltm_stat.tile_num.tile_num_x = 8,
	.ltm_stat.tile_num.tile_num_y = 8,
	.ltm_stat.strength = 4,
	.ltm_stat.region_est_en = 1,
	.ltm_stat.text_point_thres = 7,
	.ltm_stat.text_proportion  = 19,
	.ltm_stat.tile_num_auto    = 0,

	.ltm_map.bypass = 0,
	.ltm_map.ltm_map_video_mode = 1,
};

static struct isp_dev_ltm_info g_ltm_info_cap = {
	.ltm_stat.bypass  = 1,
	.ltm_stat.tile_num.tile_num_x = 8,
	.ltm_stat.tile_num.tile_num_y = 8,
	.ltm_stat.strength = 4,
	.ltm_stat.region_est_en = 1,
	.ltm_stat.text_point_thres = 7,
	.ltm_stat.text_proportion  = 19,
	.ltm_stat.tile_num_auto    = 0,

	.ltm_map.bypass = 0,
	.ltm_map.ltm_map_video_mode = 0,
};

int isp_ltm_config_param(struct isp_ltm_ctx_desc *ctx)
{
	return 0;
}

struct isp_dev_ltm_info *isp_ltm_get_tuning_config(int type)
{
	struct isp_dev_ltm_info *info;

	if (type == ISP_PRO_LTM_PRE_PARAM)
		info = &g_ltm_info_pre;
	else
		info = &g_ltm_info_cap;

	return info;
}
