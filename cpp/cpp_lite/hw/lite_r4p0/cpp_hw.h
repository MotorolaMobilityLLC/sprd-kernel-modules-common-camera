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

#ifndef _CPP_HW_H_
#define _CPP_HW_H_

extern struct cpp_hw_info lite_r5p0_cpp_hw_info;
extern struct cpp_hw_info lite_r6p0_cpp_hw_info;
extern struct cpp_hw_info lite_r4p0_cpp_hw_info;
extern struct cpp_hw_info lite_r3p0_cpp_hw_info;

#define CPP_BP_SUPPORT                  0
#define CPP_SLICE_SUPPORT               0
#define CPP_ZOOM_UP_SUPPORT             0

#define CPP_SRC_W_ALIGN                 8
#define CPP_SRC_H_ALIGN                 2
#define CPP_DST_W_ALIGN                 8
#define CPP_DST_YUV420_H_ALIGN          2
#define CPP_DST_YUV422_H_ALIGN          2

#define SCALE_FRAME_WIDTH_MAX           8192
#define SCALE_FRAME_HEIGHT_MAX          8192
#define SCALE_FRAME_WIDTH_MIN           64
#define SCALE_FRAME_HEIGHT_MIN          32
#define BP_TRIM_SIZE_MIN                32
#define BP_TRIM_SIZE_MAX                8192
#define SCALE_WIDTH_MIN                 64
#define SCALE_HEIGHT_MIN                32
#define SCALE_SLICE_OUT_WIDTH_MAX       2400
#define SCALE_FRAME_OUT_WIDTH_MAX       768
#define SCALE_SC_COEFF_MAX              8
#define SCALE_SC_COEFF_MID              4
#define SCALE_DECI_FAC_MAX              3
#define SCALE_PIXEL_ALIGNED             4
#define NORMAL_CLK_DOMAIN               0
#define COEFF_CLK_DOMAIN                1

enum cpp_prj_id {
	CPP_R3P0,
	CPP_R4P0,
	CPP_R5P0,
	CPP_R6P0,
	PROJECT_MAX
};

enum  {
	CPP_RST = 0,
	CPP_PATH0_RST,
	CPP_PATH1_RST,
	CPP_DMA_RST
};

static const char * const syscon_name[] = {
	"cpp_rst",
	"path0_rst",
	"path1_rst",
	"dma_rst"
};

struct register_gprr {
	struct regmap *gpr;
	uint32_t reg;
	uint32_t mask;
};

struct cpp_hw_soc_info {
	struct platform_device *pdev;
	struct clk *cpp_clk;
	struct clk *cpp_clk_parent;
	struct clk *cpp_clk_default;
	struct clk *cpp_eb;
	struct clk *cpp_emc_clk;
	struct clk *cpp_emc_clk_parent;
	struct clk *cpp_emc_clk_default;

	struct clk *clk_mm_vsp_eb;

	struct register_gprr syscon_regs[ARRAY_SIZE(syscon_name)];
};

struct cpp_hw_ip_info {
	unsigned int irq;
	int slice_support;
	int zoom_up_support;
	int bp_support;
	void __iomem *io_base;
	struct platform_device *pdev;
};

struct cpp_hw_info {
	enum cpp_prj_id prj_id;
	struct platform_device *pdev;
	struct cpp_hw_soc_info *soc_cpp;
	struct cpp_hw_ip_info *ip_cpp;
	int (*cpp_hw_ioctl)(enum cpp_hw_cfg_cmd, void *);
	int (*cpp_probe)(struct platform_device *pdev,struct cpp_hw_info * hw_info);
};
#endif
