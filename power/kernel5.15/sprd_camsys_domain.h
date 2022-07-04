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

#ifndef SPRD_CAMSYS_DOMAIN_H
#define SPRD_CAMSYS_DOMAIN_H

#include <linux/pm_domain.h>

enum {
       _E_PW_OFF = 0,
       _E_PW_ON  = 1,
};

enum sprd_campw_id {
	CAM_PW_QOGIRN6PRO,
	CAM_PW_MAX
};

struct register_gpr {
	struct regmap *gpr;
	uint32_t reg;
	uint32_t mask;
};

struct camsys_power_info;

struct camsys_power_ops {
	long (*sprd_campw_init)(struct platform_device *pdev, struct camsys_power_info *pw_info);
	int (*sprd_cam_domain_eb)(struct camsys_power_info *domain);
	int (*sprd_cam_domain_disable)(struct camsys_power_info *domain);
	int (*sprd_cam_pw_on)(struct camsys_power_info *domain);
	int (*sprd_cam_pw_off)(struct camsys_power_info *domain);
};

struct camsys_power_info {
	atomic_t inited;
	struct mutex mlock;
	struct generic_pm_domain pd;
	struct camsys_power_ops *ops;

	union {
		struct {
			atomic_t users_pw;
			atomic_t users_dcam_pw;
			atomic_t users_isp_pw;
			atomic_t users_clk;
			atomic_t users_blk_cfg_en;

			struct mutex mlock;
			struct clk *mm_eb;
			struct clk *mm_mtx_data_en;
			struct clk *ckg_en;
			struct clk *blk_cfg_en;

			struct clk *mm_mtx_clk;
			struct clk *mm_mtx_clk_parent;
			struct clk *mm_mtx_clk_defalut;
			struct register_gpr regs[9];
		} qogirl6pro;
	} u;
};

extern  struct camsys_power_ops camsys_power_ops_qogirl6pro;

int sprd_mm_pw_notify_register(struct notifier_block *nb);
int sprd_mm_pw_notify_unregister(struct notifier_block *nb);
#endif
