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

#include <linux/regmap.h>
#include <linux/spinlock.h>

#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_path.h"
#include "dcam_hw_if.h"
#include <sprd_mm.h>

#define DCAMX_STOP_TIMEOUT 2000
#define DCAM_AXI_STOP_TIMEOUT 2000
#define DCAM_AXIM_AQOS_MASK (0x30FFFF)

extern atomic_t s_dcam_working;
static const struct bypass_tag dcam_tb_bypass[] = {
	[_E_4IN1] = {"4in1", DCAM_MIPI_CAP_CFG, 12}, /* 0x100.b12 */
	[_E_PDAF] = {"pdaf", DCAM_PPE_FRM_CTRL0, 1}, /* 0x120.b1 */
	[_E_LSC]  = {"lsc", DCAM_LENS_LOAD_ENABLE, 0}, /* 0x138.b0 */
	[_E_AEM]  = {"aem",  DCAM_AEM_FRM_CTRL0, 0}, /* 0x150.b0 */
	[_E_HIST] = {"hist", DCAM_HIST_FRM_CTRL0, 0}, /* 0x160.b0 */
	[_E_AFL]  = {"afl",  ISP_AFL_FRM_CTRL0, 0}, /* 0x170.b0 */
	[_E_AFM]  = {"afm",  ISP_AFM_FRM_CTRL, 0}, /* 0x1A0.b0 */
	[_E_BPC]  = {"bpc",  ISP_BPC_PARAM, 0}, /* 0x200.b0 */
	[_E_BLC]  = {"blc",  DCAM_BLC_PARA_R_B, 31}, /* 0x268.b31 */
	[_E_RGB]  = {"rgb",  ISP_RGBG_YRANDOM_PARAMETER0, 0}, /* 0x278.b0 rgb gain */
	[_E_RAND] = {"rand", ISP_RGBG_YRANDOM_PARAMETER0, 1}, /* 0x278.b1 */
	[_E_PPI]  = {"ppi",  ISP_PPI_PARAM, 0}, /* 0x284.b0 */
	[_E_AWBC] = {"awbc", ISP_AWBC_GAIN0, 31}, /* 0x380.b31 */
	[_E_NR3]  = {"nr3",  NR3_FAST_ME_PARAM, 0}, /* 0x3F0.b0 */
};

uint32_t dcam_tb_bypass_get_count()
{
	return sizeof(dcam_tb_bypass) / sizeof(dcam_tb_bypass[0]);
}

struct bypass_tag *dcam_tb_bypass_get_data(uint32_t i)
{
	if (i >= dcam_tb_bypass_get_count()) {
		return NULL;
	}
	return (struct bypass_tag *)&dcam_tb_bypass[i];
}

static uint32_t dcam_trace_regs[] = {
		DCAM_CFG,
		DCAM_APB_SRAM_CTRL,
		DCAM_IMAGE_CONTROL,
		DCAM_PDAF_CONTROL,
		DCAM_LENS_LOAD_ENABLE,
		ISP_BPC_PARAM,
		DCAM_AEM_FRM_CTRL0,
		ISP_AFM_FRM_CTRL,
		ISP_AFL_FRM_CTRL0,
		DCAM_HIST_FRM_CTRL0,
		NR3_FAST_ME_PARAM,
		DCAM_FULL_BASE_WADDR,
		DCAM_BIN_BASE_WADDR0,
		DCAM_PDAF_BASE_WADDR,
		DCAM_VCH2_BASE_WADDR,
		DCAM_VCH3_BASE_WADDR,
		DCAM_AEM_BASE_WADDR,
		DCAM_HIST_BASE_WADDR,
		DCAM_PPE_RIGHT_WADDR,
		ISP_AFL_GLB_WADDR,
		ISP_AFL_REGION_WADDR,
		ISP_BPC_OUT_ADDR,
		ISP_AFM_BASE_WADDR,
		ISP_NR3_WADDR,
};

uint32_t dcam_trace_regs_get_count()
{
	return sizeof(dcam_trace_regs) / sizeof(dcam_trace_regs[0]);
}

uint32_t dcam_trace_regs_get_data(uint32_t i)
{
	if (i >= dcam_trace_regs_get_count()) {
		return -1;
	}
	return dcam_trace_regs[i];
}

int dcam_reg_trace(struct sprd_cam_hw_info *hw, void *arg)
{
	return 0;
}

static uint32_t path_ctrl_id[] = {
	DCAM_CTRL_FULL,
	DCAM_CTRL_BIN,
	DCAM_CTRL_PDAF,
	DCAM_CTRL_VCH2,
	DCAM_CTRL_VCH3,
	DCAM_CTRL_COEF,
	DCAM_CTRL_COEF,
	DCAM_CTRL_COEF,
	DCAM_CTRL_COEF,
	DCAM_CTRL_COEF,
	DCAM_CTRL_COEF,
};

uint32_t dcam_get_path_ctrl_id(uint32_t path_id)
{
	if (path_id >= DCAM_PATH_MAX) {
		pr_err("fail to get right path_id %d\n", path_id);
		return -1;
	}
	return path_ctrl_id[path_id];
}


/*
 * Do force copy to before capture enabled.
 * id: refer to enum dcam_ctrl_id in dcam_core.h
 * bit0 - cap, bit1 - coef, bit2 - rds coef, bit3 - full, ...
 */
void dcam_force_copy(struct dcam_pipe_dev *dev, uint32_t id)
{
	const uint32_t bitmap[] = {
		BIT_0, BIT_4, BIT_6, BIT_8, BIT_10, BIT_12, BIT_14, BIT_16
	};
	const uint32_t bitmap2 = BIT_4;
	uint32_t mask = 0, j;
	unsigned long flags = 0;

	if (unlikely(!dev)) {
		pr_warn("invalid param dev\n");
		return;
	}

	if (dev->idx < DCAM_ID_MAX) {
		for (j = 0; j < 8; j++) {
			if (id & (1 << j))
				mask |= bitmap[j];
		}
	} else if (id && DCAM_CTRL_CAP) {
		mask = bitmap2;
	}
	pr_debug("DCAM%u: force copy 0x%0x, id 0x%x\n", dev->idx, mask, id);
	if (mask == 0)
		return;

	spin_lock_irqsave(&dev->glb_reg_lock, flags);
	DCAM_REG_MWR(dev->idx, DCAM_CONTROL, mask, mask);
	spin_unlock_irqrestore(&dev->glb_reg_lock, flags);
}

void dcam_auto_copy(struct dcam_pipe_dev *dev, uint32_t id)
{
	const uint32_t bitmap[] = {
		BIT_1, BIT_5, BIT_7, BIT_9, BIT_11, BIT_13, BIT_15, BIT_17
	};
	const uint32_t bitmap2 = BIT_5;
	uint32_t mask = 0, j;
	unsigned long flags = 0;

	if (unlikely(!dev)) {
		pr_warn("invalid param dev\n");
		return;
	}
	if (dev->idx < DCAM_ID_MAX) {
		for (j = 0; j < 8; j++) {
			if (id & (1 << j))
				mask |= bitmap[j];
		}
	} else if (id && DCAM_CTRL_CAP) {
		mask = bitmap2;
	}

	pr_debug("DCAM%u: auto copy 0x%0x, id 0x%x\n", dev->idx, mask, id);
	if (mask == 0)
		return;

	spin_lock_irqsave(&dev->glb_reg_lock, flags);
	DCAM_REG_MWR(dev->idx, DCAM_CONTROL, mask, mask);
	spin_unlock_irqrestore(&dev->glb_reg_lock, flags);
}



/* TODO: check this */
/*
 * Some register is in RAM, so it's important to set initial value for them.
 */
void dcam_init_default(struct dcam_pipe_dev *dev)
{
	uint32_t idx, bypass, eb;

	if (unlikely(!dev)) {
		pr_warn("invalid param dev\n");
		return;
	}
	idx = dev->idx;

	/* init registers(sram regs) to default value */
	//dcam_reg_set_default_value(idx);


	/* disable internal logic access sram */
	DCAM_REG_MWR(idx, DCAM_APB_SRAM_CTRL, BIT_0, 0);

	DCAM_REG_WR(idx, DCAM_CFG, 0); /* disable all path */
	DCAM_REG_WR(idx, DCAM_IMAGE_CONTROL, 0x2b << 8 | 0x01);

	eb = 0;
	DCAM_REG_MWR(idx, DCAM_PDAF_CONTROL, BIT_1 | BIT_0, eb);
	DCAM_REG_MWR(idx, DCAM_CROP0_START, BIT_31, eb << 31);

	/* default bypass all blocks */
	bypass = 1;
	/*bypass pre-binning*/
	DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_0, bypass);
	/*bypass blc*/
	DCAM_REG_MWR(idx, DCAM_BLC_PARA_R_B,
		BIT_31,
		bypass<< 31);
	/*bypass RGBG*/
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0,
		BIT_1,
		bypass << 1);
	DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0,
		BIT_0,
		bypass);
	/*bypas ppi*/
	DCAM_REG_MWR(idx, ISP_PPI_PARAM,
		BIT_0,
		bypass);
	/*bypass LSCM*/
	DCAM_REG_MWR(idx, DCAM_LSCM_FRM_CTRL0,
		BIT_0,
		bypass);
	/*bypass lsc*/
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE,
		BIT_0,
		bypass);
	/*bypass aem*/
	DCAM_REG_MWR(idx, DCAM_AEM_FRM_CTRL0,
		BIT_0,
		bypass);
	/*bypass hist*/
	DCAM_REG_MWR(idx, DCAM_HIST_FRM_CTRL0,
		BIT_0,
		bypass);
	/*bypass bayer2y*/
	DCAM_REG_MWR(idx, ISP_AFL_PARAM0,
	BIT_1,
	bypass << 1);
	/*bypass afl*/
	DCAM_REG_MWR(idx, ISP_AFL_FRM_CTRL0,
		BIT_0,
		bypass);
	/*bypass Crop0*/
	DCAM_REG_MWR(idx, DCAM_CROP0_START, BIT_31, 0 << 31);
	/*bypass awbc*/
	DCAM_REG_MWR(idx, ISP_AWBC_GAIN0,
		BIT_31,
		bypass << 31);
	/*bypass bpc*/
	DCAM_REG_MWR(idx, ISP_BPC_PARAM,
		BIT_0,
		bypass);
	/*bypass afm*/
	DCAM_REG_MWR(idx, ISP_AFM_FRM_CTRL,
		BIT_0,
		bypass);
	/*bypass nr3*/
	DCAM_REG_WR(idx, NR3_FAST_ME_PARAM, 0x109);
	/*bypass GTM*/
	DCAM_REG_MWR(idx, DCAM_GTM_GLB_CTRL,
		BIT_0,
		0);
	/*fbc bypass*/
	DCAM_REG_MWR(idx, DCAM_FBC_CTRL,
		BIT_0,
		0);
}


/* TODO: check this */
int dcam_reset(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	enum dcam_id idx = dev->idx;
	struct sprd_cam_hw_info *hw = dev->hw;
	uint32_t time_out = 0, flag = 0;
	uint32_t reset_bit[DCAM_ID_MAX] = {
		BIT(5),
		BIT(4),
		BIT(3)
	};
	uint32_t sts_bit[DCAM_ID_MAX] = {
		BIT(12), BIT(13), BIT(14)
	};

	pr_info("DCAM%d: reset.\n", idx);

	/* then wait for AXIM cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXIM_RD(AXIM_DBG_STS) & sts_bit[idx]))
			break;
		udelay(1000);
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("DCAM%d: reset timeout, axim status 0x%x\n", idx,
			DCAM_AXIM_RD(AXIM_DBG_STS));
	} else {
		flag = reset_bit[idx];
		pr_debug("DCAM%d, rst=0x%x, rst_mask=0x%x flag=0x%x\n",
			idx, hw->syscon.rst, hw->syscon.rst_mask, flag);
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.rst, hw->syscon.rst_mask, hw->syscon.rst_mask);
		udelay(10);
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.rst, hw->syscon.rst_mask, ~(hw->syscon.rst_mask));
	}

	DCAM_REG_MWR(idx, DCAM_INT_CLR,
		DCAMINT_IRQ_LINE_MASK, DCAMINT_IRQ_LINE_MASK);
	DCAM_REG_MWR(idx, DCAM_INT_EN,
		DCAMINT_IRQ_LINE_MASK, DCAMINT_IRQ_LINE_MASK);

	dcam_init_default(dev);
	pr_info("DCAM%d: reset end\n", idx);

	return ret;
}


void dcam_init_axim(struct sprd_cam_hw_info *hw)
{
	uint32_t reg_val;
	uint32_t time_out = 0;

	/* firstly, stop AXI writing. */
	DCAM_AXIM_MWR(AXIM_CTRL, BIT_24 | BIT_23, (0x3 << 23));

	/* then wait for AHB busy cleared */
	while (++time_out < DCAM_AXI_STOP_TIMEOUT) {
		if (0 == (DCAM_AXIM_RD(AXIM_DBG_STS) & 0x1F00F))
			break;
		udelay(1000);
	}

	if (time_out >= DCAM_AXI_STOP_TIMEOUT) {
		pr_info("dcam axim timeout status 0x%x\n",
			DCAM_AXIM_RD(AXIM_DBG_STS));
	} else {
		pr_debug("axim all_rst=0x%x, all_rst_mask=0x%x\n",
			hw->syscon.all_rst, hw->syscon.all_rst_mask);
		/* reset dcam all (0/1/2/bus) */
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.all_rst,
			hw->syscon.all_rst_mask,
			hw->syscon.all_rst_mask);
		udelay(10);
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.all_rst,
			hw->syscon.all_rst_mask, ~(hw->syscon.all_rst_mask));
	}

	/* AXIM shared by all dcam, should be init once only...*/
	dcam_aximreg_set_default_value();

	reg_val = (0x0 << 20) |
		((hw->arqos_low & 0xF) << 12) |
		(0x8 << 8) |
		((hw->awqos_high & 0xF) << 4) |
		(hw->awqos_low & 0xF);
	DCAM_AXIM_MWR(AXIM_CTRL,
			DCAM_AXIM_AQOS_MASK, reg_val);

	/* the end, enable AXI writing */
	DCAM_AXIM_MWR(AXIM_CTRL, BIT_24 | BIT_23, (0x0 << 23));
}

int dcam_set_mipi_cap(struct dcam_pipe_dev *dev,
				struct dcam_mipi_info *cap_info)
{
	int ret = 0;
	uint32_t idx = dev->idx;
	uint32_t reg_val;

	/* set mipi interface  */
	if (cap_info->sensor_if != DCAM_CAP_IF_CSI2) {
		pr_err("error: unsupported sensor if : %d\n",
			cap_info->sensor_if);
		return -EINVAL;
	}

	/* data format */
	if (cap_info->format == DCAM_CAP_MODE_RAWRGB) {
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_1, 1 << 1);
		DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_4 | BIT_5, cap_info->pattern << 4);
	} else if (cap_info->format == DCAM_CAP_MODE_YUV) {
		if (unlikely(cap_info->data_bits != DCAM_CAP_8_BITS)) {
			pr_err("error: invalid %d bits for yuv format\n",
				cap_info->data_bits);
			return -EINVAL;
		}

		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_1,  0 << 1);
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CTRL,
				BIT_1 | BIT_0, cap_info->pattern);
		/* x & y deci */
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_FRM_CTRL,
				BIT_9 | BIT_8 | BIT_5 | BIT_4,
				(cap_info->y_factor << 8)
				| (cap_info->x_factor << 4));
	} else {
		pr_err("error: unsupported capture format: %d\n",
			cap_info->format);
		return -EINVAL;
	}

	/* data mode */
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_3, cap_info->mode << 3);
	/* href */
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_13, cap_info->href << 13);
	/* frame deci */
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_7 | BIT_6,
			cap_info->frm_deci << 6);
	/* MIPI capture start */
	reg_val = (cap_info->cap_size.start_y << 16);
	reg_val |= cap_info->cap_size.start_x;
	DCAM_REG_WR(idx, DCAM_MIPI_CAP_START, reg_val);

	/* MIPI capture end */
	reg_val = (cap_info->cap_size.start_y
			+ cap_info->cap_size.size_y - 1) << 16;
	reg_val |= (cap_info->cap_size.start_x
			+ cap_info->cap_size.size_x - 1);
	DCAM_REG_WR(idx, DCAM_MIPI_CAP_END, reg_val);

	/* frame skip before capture */
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG,
			BIT_8 | BIT_9 | BIT_10 | BIT_11,
				cap_info->frm_skip << 8);

	/* for C-phy */
	if (cap_info->is_cphy == 1)
		DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_31, BIT_31);

	/* bypass 4in1 */
	if (cap_info->is_4in1) { /* 4in1 use sum, not avrg */
		DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_1,
						(1) << 1);
		DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_2, 0 << 2);
	}
	DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_0, !cap_info->is_4in1);

	/* > 24M */
	if (cap_info->is_bigsize) {
		DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_2, 1 << 2);
		DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG, BIT_0, 0);
	}

	pr_info("cap size : %d %d %d %d\n",
		cap_info->cap_size.start_x, cap_info->cap_size.start_y,
		cap_info->cap_size.size_x, cap_info->cap_size.size_y);
	pr_info("cap: frm %d, mode %d, bits %d, pattern %d, href %d\n",
		cap_info->format, cap_info->mode, cap_info->data_bits,
		cap_info->pattern, cap_info->href);
	pr_info("cap: deci %d, skip %d, x %d, y %d, 4in1 %d\n",
		cap_info->frm_deci, cap_info->frm_skip, cap_info->x_factor,
		cap_info->y_factor, cap_info->is_4in1);

	return ret;
}

int dcam_cfg_ebd(struct dcam_pipe_dev *dev, void *param)
{
	struct sprd_ebd_control *p = (struct sprd_ebd_control *)param;
	uint32_t idx = dev->idx;

	pr_info("mode:0x%x, vc:0x%x, dt:0x%x\n", p->mode,
			p->image_vc, p->image_dt);
	dev->is_ebd = 1;

	DCAM_REG_WR(idx, DCAM_VC2_CONTROL,
		((p->image_vc & 0x3) << 16) |
		((p->image_dt & 0x3F) << 8) |
		(p->mode & 0x3) << 4);

	return 0;
}

int  dcam_cfg_path_full_source(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param)
{
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)dcam_handle;
	uint32_t lowlux_4in1 = *(uint32_t *)param;
	static const char *tb_src[] = {"(4c)raw", "bin-sum"}; /* for log */

	if (lowlux_4in1) {
		dev->lowlux_4in1 = 1;
		DCAM_REG_MWR(dev->idx, DCAM_FULL_CFG, BIT(2), BIT(2));
		dev->skip_4in1 = 1; /* auto copy, so need skip 1 frame */
	} else {
		dev->lowlux_4in1 = 0;
		DCAM_REG_MWR(dev->idx, DCAM_FULL_CFG, BIT(2), 0);
		dev->skip_4in1 = 1;
	}
	pr_info("dev%d lowlux %d, skip_4in1 %d, full src: %s\n", dev->idx,
		dev->lowlux_4in1, dev->skip_4in1, tb_src[lowlux_4in1]);

	return 0;
}

int dcam_hwsim_extra(enum dcam_id idx)
{

		DCAM_REG_MWR(idx, ISP_BPC_PARAM, ((1 & 0x1) << 7), ((0 & 0x1) << 7));
		pr_info("bpc<0x%x>[0x%x]\n", 0x0200, DCAM_REG_RD(idx, 0x0200));

#if 0

		uint32_t val;

		val = (1 & 0x1) |
		((1 & 0x1) << 1) |
		((1 & 0x1) << 2) |
		((1 & 0x1) << 3);
		DCAM_REG_MWR(idx, ISP_BPC_PARAM, 0xF, val);

		pr_info("blc\n");
		DCAM_REG_MWR(idx, DCAM_BLC_PARA_R_B, BIT_31, 1 << 31);

		pr_info("awb \n");
		DCAM_REG_MWR(idx, ISP_AWBC_GAIN0, BIT_31, 1 << 31);

		pr_info("lsc \n");
		DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 1);

		pr_info("rgbg \n");
		DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0, BIT_0, 1);

		pr_info("afl \n");
		DCAM_REG_MWR(idx, ISP_AFL_FRM_CTRL0, BIT_0, 1);
		DCAM_REG_MWR(idx, ISP_AFL_PARAM0, BIT_1, 1 << 1);

		pr_info("rgbg_yrandom\n");
		DCAM_REG_MWR(idx, ISP_RGBG_YRANDOM_PARAMETER0, BIT_1, 1 << 1);
#endif

#if 0
		val = ((0 & 1) << 3) | (1 & 1);

		pr_info("ppi ppi_phase_map_corr_en\n");
		DCAM_REG_MWR(idx, ISP_PPI_PARAM, BIT_3 | BIT_0, val);
#endif

		return 0;
}

/* config fbc, see dcam_interface.h for @fbc_mode */
 int dcam_cfg_fbc(struct dcam_pipe_dev *dev, int fbc_mode)
{
	struct dcam_path_desc *path = NULL;
	struct camera_frame *frame = NULL;

	DCAM_REG_MWR(dev->idx, DCAM_PATH_ENDIAN, 0x3, fbc_mode);
	pr_info("fbc mode %d\n", fbc_mode);

	/* update compressed flag for reserved buffer */
	if (fbc_mode == DCAM_FBC_FULL)
		path = &dev->path[DCAM_PATH_FULL];
	else if (fbc_mode == DCAM_PATH_BIN)
		path = &dev->path[DCAM_PATH_BIN];

	if (!path)
		return 0;

	/* bad code, but don't have any other choice */
	list_for_each_entry(frame, &path->reserved_buf_queue.head, list) {
		frame->is_compressed = 1;
	}

	return 0;
}

int dcam_start_path(void *dcam_handle, struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	struct dcam_pipe_dev *dev = NULL;
	struct isp_img_rect rect; /* for 3dnr */

	pr_debug("enter.");

	if (!path || !dcam_handle) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;

	switch (path_id) {
	case  DCAM_PATH_FULL:
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_17 |  BIT_16, path->endian.y_endian << 16);

		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_2 | BIT_3, path->is_loose << 2);
		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_4, path->src_sel << 4);

		/* full_path_en */
		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_0, (0x1));
		break;

	case  DCAM_PATH_BIN:
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_3 | BIT_2, path->endian.y_endian << 2);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
			BIT_2 | BIT_3, path->is_loose << 2);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
				BIT_16, !!dev->slowmotion_count << 16);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
				BIT_19 | BIT_18 | BIT_17,
				(dev->slowmotion_count & 7) << 17);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_0, 0x1);
		break;
	case DCAM_PATH_PDAF:
		/* pdaf path en */
		if (dev->is_pdaf)
			DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_0, 1);
		break;

	case DCAM_PATH_VCH2:
		/* data type for raw picture */
		if (path->src_sel)
			DCAM_REG_WR(idx, DCAM_VC2_CONTROL, 0x2b << 8 | 0x01);

		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_23 |  BIT_22, path->endian.y_endian << 22);

		/*vch2 path en */
		DCAM_REG_MWR(idx, DCAM_VC2_CONTROL, BIT_0, 1);
		break;

	case DCAM_PATH_VCH3:
		DCAM_REG_MWR(idx, DCAM_PATH_ENDIAN,
			BIT_25 |  BIT_24, path->endian.y_endian << 24);
		/*vch3 path en */
		DCAM_REG_MWR(idx, DCAM_VC3_CONTROL, BIT_0, 1);
		break;
	case DCAM_PATH_3DNR:
		/*
		 * set default value for 3DNR
		 * nr3_mv_bypass: 0
		 * nr3_channel_sel: 0
		 * nr3_project_mode: 0
		 * nr3_sub_me_bypass: 0x1
		 * nr3_out_en: 0
		 * nr3_ping_pong_en: 0
		 * nr3_bypass: 0
		 */
		rect.x = path->in_trim.start_x;
		rect.y = path->in_trim.start_y;
		rect.w = path->in_trim.size_x;
		rect.h = path->in_trim.size_y;
		if (dev->cap_info.cap_size.size_x < (rect.x + rect.w) ||
			dev->cap_info.cap_size.size_y < (rect.y + rect.h)) {
			pr_err("dcam 3dnr input rect error[%d %d %d %d]\n",
				rect.x, rect.y, rect.w, rect.h);
			break;
		}
		DCAM_REG_WR(idx, NR3_FAST_ME_PARAM, 0x8);
		dcam_k_3dnr_set_roi(rect,
				0/* project_mode=0 */, idx);
		break;
	default:
		break;
	}

	pr_debug("done\n");
	return ret;
}

int dcam_stop_path(void *dcam_handle, struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	uint32_t reg_val;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("enter.");

	if (!path || !dcam_handle) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;

	switch (path_id) {
	case  DCAM_PATH_FULL:
		reg_val = 0;
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_0, 1);
		DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_0, 0);
		break;

	case  DCAM_PATH_BIN:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_1, 1 << 1);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_0, 0);
		break;
	case  DCAM_PATH_PDAF:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_2, 1 << 2);
		DCAM_REG_MWR(idx, DCAM_PPE_FRM_CTRL0, BIT_0, 0);
		break;
	case  DCAM_PATH_VCH2:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_3, 1 << 3);
		DCAM_REG_MWR(idx, DCAM_VC2_CONTROL, BIT_0, 0);
		break;

	case  DCAM_PATH_VCH3:
		DCAM_REG_MWR(idx, DCAM_PATH_STOP, BIT_4, 1 << 4);
		DCAM_REG_MWR(idx, DCAM_VC3_CONTROL, BIT_0, 0);
		break;

	default:
		break;
	}

	pr_debug("done\n");
	return ret;
}

int dcam_set_fetch(void *dcam_handle, struct dcam_fetch_info *fetch)
{
	int ret = 0;
	uint32_t fetch_pitch;
	struct dcam_pipe_dev *dev = NULL;

	pr_debug("enter.\n");

	if (!dcam_handle || !fetch) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dcam_handle;
	/* !0 is loose */
	if (fetch->is_loose != 0) {
		fetch_pitch = (fetch->size.w * 16 + 127) / 128;
	} else {
		fetch_pitch = (fetch->size.w * 10 + 127) / 128;
	}
	pr_info("size [%d %d], start %d, pitch %d, 0x%x\n",
		fetch->trim.size_x, fetch->trim.size_y,
		fetch->trim.start_x, fetch_pitch, fetch->addr.addr_ch0);
	/* (bitfile)unit 32b,(spec)64b */

	DCAM_REG_MWR(dev->idx,
		DCAM_MIPI_CAP_CFG, BIT_12, 0x1 << 12);
	DCAM_REG_MWR(dev->idx,
		DCAM_MIPI_CAP_CFG, BIT_1, 0x1 << 1);
	DCAM_REG_MWR(dev->idx,
		DCAM_MIPI_CAP_CFG, BIT_3, 0x0 << 3);
	DCAM_REG_MWR(dev->idx, DCAM_BAYER_INFO_CFG,
		BIT_5 | BIT_4, (fetch->pattern & 3) << 4);
	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
		BIT_1 | BIT_0, fetch->is_loose);
	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
		BIT_3 | BIT_2, fetch->endian << 2);
	DCAM_AXIM_WR(IMG_FETCH_SIZE,
		(fetch->trim.size_y << 16) | (fetch->trim.size_x & 0xffff));
	DCAM_AXIM_WR(IMG_FETCH_X,
		(fetch_pitch << 16) | (fetch->trim.start_x & 0xffff));

	DCAM_AXIM_WR(IMG_FETCH_RADDR, fetch->addr.addr_ch0);

	pr_info("done.\n");

	return ret;
}

void dcam_start_fetch(void)
{
	DCAM_AXIM_WR(IMG_FETCH_START, 1);
}

 /* Start capture by set cap_eb.*/
int dcam_start(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	uint32_t idx = dev->idx;

	DCAM_REG_WR(idx, DCAM_INT_CLR, 0xFFFFFFFF);

	/* see DCAM_PREVIEW_SOF in dcam_int.h for details */
	DCAM_REG_WR(idx, DCAM_INT_EN, DCAMINT_IRQ_LINE_EN_NORMAL);

	/* enable internal logic access sram */
	DCAM_REG_MWR(idx, DCAM_APB_SRAM_CTRL, BIT_0, 1);

	/* trigger cap_en*/
	DCAM_REG_MWR(idx, DCAM_MIPI_CAP_CFG, BIT_0, 1);

	return ret;
}

int dcam_stop(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int time_out = DCAMX_STOP_TIMEOUT;
	uint32_t idx = dev->idx;

	/* reset  cap_en*/
	DCAM_REG_MWR(idx, DCAM_CFG, BIT_0, 0);
	DCAM_REG_WR(idx, DCAM_PATH_STOP, 0x2DFF);

	DCAM_REG_WR(idx, DCAM_INT_EN, 0);
	DCAM_REG_WR(idx, DCAM_INT_CLR, 0xFFFFFFFF);

	/* wait for AHB path busy cleared */
	while (time_out) {
		ret = DCAM_REG_RD(idx, DCAM_PATH_BUSY) & 0x2FFF;
		if (!ret)
			break;
		udelay(1000);
		time_out--;
	}

	if (time_out == 0)
		pr_err("DCAM%d: stop timeout for 2s\n", idx);

	pr_info("dcam%d stop\n", idx);
	return ret;
}

int dcam_update_path_size(
	struct dcam_pipe_dev *dev, struct dcam_path_desc *path)
{
	int ret = 0;
	uint32_t idx;
	uint32_t path_id;
	uint32_t reg_val;
	struct dcam_path_desc *path_3dnr;
	struct isp_img_rect rect; /* for 3dnr path */

	pr_debug("enter.");

	idx = dev->idx;
	path_id = path->path_id;

	switch (path_id) {
	case  DCAM_PATH_FULL:
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {

			DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_1, 1 << 1);

			reg_val = (path->in_trim.start_y << 16) |
						path->in_trim.start_x;
			DCAM_REG_WR(idx, DCAM_FULL_CROP_START, reg_val);

			reg_val = (path->in_trim.size_y << 16) |
						path->in_trim.size_x;
			DCAM_REG_WR(idx, DCAM_FULL_CROP_SIZE, reg_val);

		} else {
			DCAM_REG_MWR(idx, DCAM_FULL_CFG, BIT_1, 0 << 1);
		}
		break;

	case  DCAM_PATH_BIN:
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_6, path->bin_ratio << 6);
		DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG,
					BIT_5 | BIT_4,
					(path->scaler_sel & 3) << 4);
		/* set size to path[DCAM_PATH_3DNR]
		 * because, 3dnr set roi need know bin path crop size
		 * 3dnr end_y should <= bin crop.end_y
		 */
		path_3dnr = &dev->path[DCAM_PATH_3DNR];
		path_3dnr->in_trim = path->in_trim;
		path_3dnr->in_size = path->in_size;
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {

			reg_val = (path->in_trim.start_y << 16) |
						path->in_trim.start_x;
			DCAM_REG_WR(idx, DCAM_CAM_BIN_CROP_START, reg_val);

			reg_val = (path->in_trim.size_y << 16) |
						path->in_trim.size_x;
			DCAM_REG_WR(idx, DCAM_CAM_BIN_CROP_SIZE, reg_val);
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_1, 1 << 1);
		} else {
			/* bypass trim */
			DCAM_REG_MWR(idx, DCAM_CAM_BIN_CFG, BIT_1, 0 << 1);
		}

		if (path->scaler_sel == DCAM_SCALER_RAW_DOWNSISER) {
			uint32_t cnt;
			uint32_t *ptr = (uint32_t *)path->rds_coeff_buf;
			unsigned long addr = RDS_COEF_TABLE_START;

			for (cnt = 0; cnt < path->rds_coeff_size;
				cnt += 4, addr += 4)
				DCAM_REG_WR(idx, addr, *ptr++);

			reg_val = ((path->out_size.h & 0xfff) << 16) |
						(path->out_size.w & 0x1fff);
			DCAM_REG_WR(idx, DCAM_RDS_DES_SIZE, reg_val);
			dev->auto_cpy_id |= DCAM_CTRL_RDS;
		}
		break;
	case DCAM_PATH_3DNR:
		/* reset when zoom */
		rect.x = path->in_trim.start_x;
		rect.y = path->in_trim.start_y;
		rect.w = path->in_trim.size_x;
		rect.h = path->in_trim.size_y;
		if (dev->cap_info.cap_size.size_x < (rect.x + rect.w) ||
			dev->cap_info.cap_size.size_y < (rect.y + rect.h)) {
			pr_err("dcam 3dnr input rect error[%d %d %d %d]\n",
				rect.x, rect.y, rect.w, rect.h);
			break;
		}
		dcam_k_3dnr_set_roi(rect,
				0/* project_mode=0 */, idx);
		break;
	default:
		break;
	}

	pr_debug("done\n");
	return ret;
}

/* todo: enable block config one by one */
/* because parameters from user may be illegal when bringup*/
static struct dcam_cfg_entry dcam_cfg_func_tab[DCAM_BLOCK_TOTAL] = {
[DCAM_BLOCK_BLC - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_BLC,              dcam_k_cfg_blc},
[DCAM_BLOCK_AEM - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_AEM,              dcam_k_cfg_aem},
[DCAM_BLOCK_AWBC - DCAM_BLOCK_BASE]    = {DCAM_BLOCK_AWBC,             dcam_k_cfg_awbc},
[DCAM_BLOCK_AFM - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_AFM,              dcam_k_cfg_afm},
[DCAM_BLOCK_AFL - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_AFL,              dcam_k_cfg_afl},
[DCAM_BLOCK_LSC - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_LSC,              dcam_k_cfg_lsc},
[DCAM_BLOCK_BPC - DCAM_BLOCK_BASE]     = {DCAM_BLOCK_BPC,              dcam_k_cfg_bpc},
[DCAM_BLOCK_RGBG - DCAM_BLOCK_BASE]    = {DCAM_BLOCK_RGBG,             dcam_k_cfg_rgb_gain},
[DCAM_BLOCK_PDAF - DCAM_BLOCK_BASE]    = {DCAM_BLOCK_PDAF,             dcam_k_cfg_pdaf},
};

struct dcam_cfg_entry *dcam_get_cfg_func(uint32_t index)
{
	if (index < DCAM_BLOCK_TOTAL)
		return &dcam_cfg_func_tab[index];
	else {
		pr_err("fail to get right cfg index %d\n", index);
		return NULL;
	}
}

static struct dcam_if dcam_if_lite_r2p0[DCAM_ID_MAX] = {
	{
		.version = 0x0200,
		.idx = 0,
		.slowmotion_path =
			BIT(DCAM_PATH_BIN) |
			BIT(DCAM_PATH_AEM) |
			BIT(DCAM_PATH_HIST),
	},
	{
		.version = 0x0200,
		.idx = 1,
		.slowmotion_path =
			BIT(DCAM_PATH_BIN) |
			BIT(DCAM_PATH_AEM) |
			BIT(DCAM_PATH_HIST),
	},
	{
		.version = 0x0200,
		.idx = 2,
		.slowmotion_path =
			BIT(DCAM_PATH_BIN) |
			BIT(DCAM_PATH_AEM) |
			BIT(DCAM_PATH_HIST),
	}
};

struct dcam_if *dcam_get_dcam_if(enum dcam_id idx) {
	return &dcam_if_lite_r2p0[idx];
}

/* set line buffer share mode
 * Attention: set before stream on
 * Input: dcam idx, image width(max)
 */
int dcam_lbuf_share_mode(enum dcam_id idx, uint32_t width)
{
	int i = 0;
	int ret = 0;
	uint32_t tb_w[] = {
	/*     dcam0, dcam1 */
		5664, 3264,
		5184, 4160,
		4672, 4672,
		4160, 5184,
		3264, 5664,
	};
	if (atomic_read(&s_dcam_working) > 0) {
		pr_warn("dcam 0/1 already in working\n");
		return 0;
	}

	pr_debug("idx[%d] width[%d]\n", idx, width);

	switch (idx) {
	case 0:
		if (width > tb_w[0]) {
			DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, 0x7, 2);
			break;
		}
		for (i = 4; i >= 0; i--) {
			if (width <= tb_w[i * 2])
				break;
		}
		DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, 0x7, i);
		pr_info("alloc dcam linebuf %d %d\n", tb_w[i*2], tb_w[i*2 + 1]);
		break;
	case 1:
		if (width > tb_w[9]) {
			DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, 0x7, 2);
			break;
		}
		for (i = 0; i <= 4; i++) {
			if (width <= tb_w[i * 2 + 1])
				break;
		}
		DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, 0x7, i);
		pr_info("alloc dcam linebuf %d %d\n", tb_w[i*2], tb_w[i*2 + 1]);
		break;
	default:
		pr_info("dcam %d no this setting\n", idx);
		ret = 1;
	}
	DCAM_AXIM_MWR(DCAM_LBUF_SHARE_MODE, 0x3 << 8, 0 << 8);

	return ret;
}
int dcam_offline_slice_set_fetch_param(uint32_t idx, struct dcam_fetch_info *fetch)
{
	int ret = 0;
	if (fetch == NULL)
		pr_err("fail to check param");

	DCAM_REG_MWR(idx, DCAM_BAYER_INFO_CFG,
		BIT_5 | BIT_4, (fetch->pattern & 3) << 4);
	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
		BIT_1 | BIT_0, fetch->is_loose);
	DCAM_AXIM_MWR(IMG_FETCH_CTRL,
		BIT_3 | BIT_2, fetch->endian << 2);
	DCAM_AXIM_WR(IMG_FETCH_SIZE,
		(fetch->trim.size_y << 16) | (fetch->trim.size_x/2 & 0xffff));
	DCAM_AXIM_WR(IMG_FETCH_RADDR, fetch->addr.addr_ch0);

	return ret;
}

