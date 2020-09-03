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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <sprd_mm.h>
#include <video/sprd_mmsys_pw_domain.h>

#include "isp_reg.h"
#include "isp_int.h"
#include "isp_core.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

uint32_t s_isp_irq_no[ISP_LOGICAL_COUNT];
unsigned long s_isp_regbase[ISP_MAX_COUNT];
unsigned long isp_phys_base[ISP_MAX_COUNT];
unsigned long s_isp_mmubase;

int isp_drv_hw_init(void *arg)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)arg;
	hw = dev->isp_hw;

	ret = sprd_cam_pw_on();
	ret = sprd_cam_domain_eb();

	ret = hw->isp_ioctl(hw, ISP_HW_CFG_ENABLE_CLK, NULL);
	if (ret)
		goto clk_fail;

	ret = hw->isp_ioctl(hw, ISP_HW_CFG_RESET, NULL);
	if (ret)
		goto reset_fail;

	ret = isp_int_irq_request(&hw->pdev->dev, s_isp_irq_no, arg);

	return 0;

reset_fail:
	hw->isp_ioctl(hw, ISP_HW_CFG_DISABLE_CLK, NULL);
clk_fail:
	sprd_cam_domain_disable();
	sprd_cam_pw_off();

	return ret;
}

int isp_hw_deinit(void *arg)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)arg;
	hw = dev->isp_hw;

	ret = hw->isp_ioctl(hw, ISP_HW_CFG_RESET, NULL);
	ret = isp_int_irq_free(&hw->pdev->dev, arg);
	ret = hw->isp_ioctl(hw, ISP_HW_CFG_DISABLE_CLK, NULL);

	sprd_cam_domain_disable();
	sprd_cam_pw_off();

	return ret;
}

static enum isp_fetch_format ispdrv_fetch_format_get(uint32_t forcc,
	uint32_t pack_bits)
{
	enum isp_fetch_format format = ISP_FETCH_FORMAT_MAX;

	switch (forcc) {
	case IMG_PIX_FMT_GREY:
		format = ISP_FETCH_CSI2_RAW10;
		if(pack_bits == ISP_RAW_HALF14 || pack_bits == ISP_RAW_HALF10)
			format = ISP_FETCH_RAW10;
		break;
	case IMG_PIX_FMT_UYVY:
		format = ISP_FETCH_UYVY;
		break;
	case IMG_PIX_FMT_YUV422P:
		format = ISP_FETCH_YUV422_3FRAME;
		break;
	case IMG_PIX_FMT_NV12:
		format = ISP_FETCH_YUV420_2FRAME;
		break;
	case IMG_PIX_FMT_NV21:
		format = ISP_FETCH_YVU420_2FRAME;
		break;
	default:
		format = ISP_FETCH_FORMAT_MAX;
		pr_err("fail to get support format 0x%x\n", forcc);
		break;
	}
	return format;
}

static int ispdrv_fbd_raw_get(void *cfg_in, void *cfg_out)
{
	int32_t tile_col = 0, tile_row = 0;
	int32_t crop_start_x = 0, crop_start_y = 0,
		crop_width = 0, crop_height = 0;
	int32_t left_offset_tiles_num = 0,
		up_offset_tiles_num = 0;
	int32_t img_width = 0;
	int32_t end_x = 0, end_y = 0,
		left_tiles_num = 0, right_tiles_num = 0,
		middle_tiles_num = 0, left_size,right_size = 0;
	int32_t up_tiles_num = 0, down_tiles_num = 0,
		vertical_middle_tiles_num = 0,
		up_size = 0, down_size = 0;
	int32_t tiles_num_pitch = 0;
	struct isp_uinfo *pipe_src = NULL;
	struct isp_fbd_raw_info *fbd_raw = NULL;

	if (!cfg_in || !cfg_out) {
		pr_err("fail to get valid input ptr, %p, %p\n", cfg_in, cfg_out);
		return -EFAULT;
	}

	pipe_src = (struct isp_uinfo *)cfg_in;
	fbd_raw = (struct isp_fbd_raw_info *)cfg_out;
	img_width = pipe_src->src.w;

	if (pipe_src->fetch_path_sel == 0)
		return 0;

	/*Bug 1024606 sw workaround
	fetch fbd crop isp timeout when crop.start_y % 4 == 2 */
	if(pipe_src->fetch_path_sel && (pipe_src->crop.start_y % 4 == 2)) {
		pipe_src->crop.start_y -= 2;
		pipe_src->crop.size_y += 2;
		pr_info("fbd start_y: %d, size_y: %d",
			pipe_src->crop.start_y, pipe_src->crop.size_y);
	}

	fbd_raw->width = pipe_src->src.w;
	fbd_raw->height = pipe_src->src.h;
	fbd_raw->size = pipe_src->src;
	fbd_raw->trim = pipe_src->crop;
	fbd_raw->fetch_fbd_bypass = 0;
	fbd_raw->fetch_fbd_4bit_bypass = pipe_src->fetch_fbd_4bit_bypass;
	pr_debug("fbd raw info: width %d, height %d, trim: x %d, y %d, w %d, h %d\n",
		fbd_raw->size.w, fbd_raw->size.h,
		fbd_raw->trim.start_x, fbd_raw->trim.start_y,
		fbd_raw->trim.size_x, fbd_raw->trim.size_y);

	tile_col = (fbd_raw->width + ISP_FBD_TILE_WIDTH - 1) / ISP_FBD_TILE_WIDTH;
	tile_col = (tile_col + 2 - 1) / 2 * 2;
	tile_row =(fbd_raw->height + ISP_FBD_TILE_HEIGHT - 1)/ ISP_FBD_TILE_HEIGHT;

	fbd_raw->tiles_num_pitch = tile_col;
	fbd_raw->low_bit_pitch = fbd_raw->tiles_num_pitch * ISP_FBD_TILE_WIDTH / 2;
	if(fbd_raw->fetch_fbd_4bit_bypass == 0)
		fbd_raw->low_4bit_pitch = fbd_raw->tiles_num_pitch * ISP_FBD_TILE_WIDTH;

	if (fbd_raw->trim.size_x != fbd_raw->size.w ||
		fbd_raw->trim.size_y != fbd_raw->size.h) {
		tiles_num_pitch = fbd_raw->tiles_num_pitch;
		fbd_raw->width = fbd_raw->trim.size_x;
		fbd_raw->height = fbd_raw->trim.size_y;

		crop_start_x = fbd_raw->trim.start_x;
		crop_start_y = fbd_raw->trim.start_y;
		crop_width = fbd_raw->trim.size_x;
		crop_height = fbd_raw->trim.size_y;

		left_offset_tiles_num = crop_start_x / ISP_FBD_TILE_WIDTH;
		up_offset_tiles_num = crop_start_y / ISP_FBD_TILE_HEIGHT;
		end_x = crop_start_x + crop_width - 1;
		end_y = crop_start_y + crop_height - 1;
		if (crop_start_x %  ISP_FBD_TILE_WIDTH == 0) {
			left_tiles_num = 0;
			left_size = 0;
		} else {
			left_tiles_num = 1;
			left_size = ISP_FBD_TILE_WIDTH - crop_start_x %  ISP_FBD_TILE_WIDTH;
		}
		if ((end_x + 1) % ISP_FBD_TILE_WIDTH == 0)
			right_tiles_num = 0;
		else
			right_tiles_num = 1;
		right_size = (end_x + 1) % ISP_FBD_TILE_WIDTH;
		middle_tiles_num = (crop_width - left_size - right_size) / ISP_FBD_TILE_WIDTH;

		if (crop_start_y % ISP_FBD_TILE_HEIGHT == 0) {
			up_tiles_num = 0;
			up_size = 0;
		} else {
			up_tiles_num = 1;
			up_size = ISP_FBD_TILE_HEIGHT - crop_start_y % ISP_FBD_TILE_HEIGHT;
		}
		if ((end_y + 1) % ISP_FBD_TILE_HEIGHT == 0)
			down_tiles_num = 0;
		else
			down_tiles_num = 1;
		down_size = (end_y + 1) % ISP_FBD_TILE_HEIGHT;
		vertical_middle_tiles_num = (crop_height - up_size - down_size) / ISP_FBD_TILE_HEIGHT;
		fbd_raw->pixel_start_in_hor = crop_start_x % ISP_FBD_TILE_WIDTH;
		fbd_raw->pixel_start_in_ver = crop_start_y % ISP_FBD_TILE_HEIGHT;
		fbd_raw->tiles_num_in_ver = up_tiles_num + down_tiles_num + vertical_middle_tiles_num;
		fbd_raw->tiles_num_in_hor = left_tiles_num + right_tiles_num + middle_tiles_num;
		fbd_raw->tiles_start_odd = left_offset_tiles_num % 2;
		fbd_raw->header_addr_offset =
			(left_offset_tiles_num + up_offset_tiles_num * tiles_num_pitch) / 2;
		fbd_raw->tile_addr_offset_x256 =
			(left_offset_tiles_num + up_offset_tiles_num * tiles_num_pitch) * ISP_FBD_BASE_ALIGN;
		fbd_raw->low_bit_addr_offset = (crop_start_x + crop_start_y * img_width) /2;
		if(fbd_raw->fetch_fbd_4bit_bypass == 0)
			fbd_raw->low_4bit_addr_offset = (crop_start_x + crop_start_y * img_width);
		pr_debug("addr offset:header 0x%x, 8bit 0x%x, 2bit 0x%x, 4bit 0x%x\n",
			fbd_raw->header_addr_offset, fbd_raw->tile_addr_offset_x256,
			fbd_raw->low_bit_addr_offset, fbd_raw->low_4bit_addr_offset);
	} else {
		fbd_raw->pixel_start_in_hor = 0;
		fbd_raw->pixel_start_in_ver = 0;
		fbd_raw->tiles_num_in_hor = tile_col;
		fbd_raw->tiles_num_in_ver = tile_row;
		fbd_raw->tiles_start_odd = 0;
		fbd_raw->header_addr_offset = 0;
		fbd_raw->tile_addr_offset_x256 = 0;
		fbd_raw->low_bit_addr_offset = 0;
		if(fbd_raw->fetch_fbd_4bit_bypass == 0)
			fbd_raw->low_4bit_addr_offset = 0;
	}

	return 0;
}

static int ispdrv_fetch_normal_get(void *cfg_in, void *cfg_out)
{
	int ret = 0;
	unsigned long trim_offset[3] = { 0 };
	struct img_size *src = NULL;
	struct img_trim *intrim = NULL;
	struct isp_hw_fetch_info *fetch = NULL;
	struct isp_uinfo *pipe_src = NULL;

	if (!cfg_in || !cfg_out) {
		pr_err("fail to get valid input ptr, %p, %p\n", cfg_in, cfg_out);
		return -EFAULT;
	}

	pipe_src = (struct isp_uinfo *)cfg_in;
	fetch = (struct isp_hw_fetch_info *)cfg_out;

	src = &pipe_src->src;
	intrim = &pipe_src->crop;
	fetch->src = *src;
	fetch->in_trim = *intrim;
	fetch->fetch_fmt = ispdrv_fetch_format_get(pipe_src->in_fmt, pipe_src->pack_bits);
	fetch->bayer_pattern = pipe_src->bayer_pattern;
	if (pipe_src->in_fmt == IMG_PIX_FMT_GREY)
		fetch->dispatch_color = 0;
	else
		fetch->dispatch_color = 2;
	fetch->fetch_path_sel = pipe_src->fetch_path_sel;
	fetch->pack_bits = pipe_src->pack_bits;

	switch (fetch->fetch_fmt) {
	case ISP_FETCH_YUV422_3FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w / 2;
		fetch->pitch.pitch_ch2 = src->w / 2;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 + intrim->start_x / 2;
		trim_offset[2] = intrim->start_y * fetch->pitch.pitch_ch2 + intrim->start_x / 2;
		break;
	case ISP_FETCH_YUYV:
	case ISP_FETCH_UYVY:
	case ISP_FETCH_YVYU:
	case ISP_FETCH_VYUY:
		fetch->pitch.pitch_ch0 = src->w * 2;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		break;
	case ISP_FETCH_RAW10:
		fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(src->w, pipe_src->pack_bits);
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		break;
	case ISP_FETCH_YUV422_2FRAME:
	case ISP_FETCH_YVU422_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 + intrim->start_x;
		break;
	case ISP_FETCH_YUV420_2FRAME:
	case ISP_FETCH_YVU420_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 / 2 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 / 2 + intrim->start_x;
		break;
	case ISP_FETCH_FULL_RGB10:
		fetch->pitch.pitch_ch0 = src->w * 3;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 3;
		break;
	case ISP_FETCH_CSI2_RAW10:
	{
		uint32_t mipi_byte_info = 0;
		uint32_t mipi_word_info = 0;
		uint32_t start_col = intrim->start_x;
		uint32_t start_row = intrim->start_y;
		uint32_t end_col =  intrim->start_x + intrim->size_x - 1;
		uint32_t mipi_word_num_start[16] = {
			0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
		uint32_t mipi_word_num_end[16] = {
			0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};

		mipi_byte_info = start_col & 0xF;
		mipi_word_info = ((end_col + 1) >> 4) * 5
			+ mipi_word_num_end[(end_col + 1) & 0xF]
			- ((start_col + 1) >> 4) * 5
			- mipi_word_num_start[(start_col + 1) & 0xF] + 1;
		fetch->mipi_byte_rel_pos = mipi_byte_info;
		fetch->mipi_word_num = mipi_word_info;
		fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(src->w, 0);
		/* same as slice starts */
		trim_offset[0] = start_row * fetch->pitch.pitch_ch0 + (start_col >> 2) * 5 + (start_col & 0x3);
		if (!pipe_src->fetch_path_sel)
			pr_debug("fetch pitch %d, offset %ld, rel_pos %d, wordn %d\n",
				 fetch->pitch.pitch_ch0, trim_offset[0],
				 mipi_byte_info, mipi_word_info);
		break;
	}
	default:
		pr_err("fail to get fetch format: %d\n", fetch->fetch_fmt);
		break;
	}

	fetch->trim_off.addr_ch0 = trim_offset[0];
	fetch->trim_off.addr_ch1 = trim_offset[1];
	fetch->trim_off.addr_ch2 = trim_offset[2];

	return ret;
}

int isp_drv_pipeinfo_get(void *arg, uint32_t hw_id)
{
	int ret = 0;
	struct isp_sw_context *ctx = NULL;
	struct isp_pipe_info *pipe_in = NULL;
	struct isp_uinfo *pipe_src = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	ctx = (struct isp_sw_context *)arg;
	pipe_src = &ctx->pipe_src;
	pipe_in = &ctx->pipe_info;

	pipe_in->fetch.ctx_id = ctx->ctx_id;
	pipe_in->fetch.sec_mode = ctx->dev->sec_mode;
	ret = ispdrv_fetch_normal_get(pipe_src, &pipe_in->fetch);
	if (ret) {
		pr_err("fail to get pipe fetch info\n");
		return -EFAULT;
	}

	pipe_in->fetch_fbd.ctx_id = ctx->ctx_id;
	ret = ispdrv_fbd_raw_get(pipe_src, &pipe_in->fetch_fbd);
	if (ret) {
		pr_err("fail to get pipe fetch fbd info\n");
		return -EFAULT;
	}

	return ret;
}

int sprd_isp_dt_parse(struct device_node *dn,
		struct cam_hw_info *hw_info,
		uint32_t *isp_count)
{
	int i = 0;
	uint32_t count = 0;
	void __iomem *reg_base;
	struct cam_hw_soc_info *soc_isp = NULL;
	struct cam_hw_ip_info *ip_isp = NULL;
	struct device_node *isp_node = NULL;
	struct device_node *qos_node = NULL;
	struct device_node *iommu_node = NULL;
	struct resource res = {0};
	int args_count = 0;
	uint32_t args[2];

	/* todo: should update according to SharkL5/ROC1 dts
	 * or set value for required variables with hard-code
	 * for quick bringup
	 */

	if (!dn || !hw_info) {
		pr_err("fail to get dn %p hw info %p\n", dn, hw_info);
		return -EINVAL;
	}

	pr_info("isp dev device node %s, full name %s\n",
		dn->name, dn->full_name);
	isp_node = of_parse_phandle(dn, "sprd,isp", 0);
	if (isp_node == NULL) {
		pr_err("fail to parse the property of sprd,isp\n");
		return -EFAULT;
	}

	soc_isp = hw_info->soc_isp;
	ip_isp = hw_info->ip_isp;
	pr_info("after isp dev device node %s, full name %s\n",
		isp_node->name, isp_node->full_name);
	soc_isp->pdev = of_find_device_by_node(isp_node);
	pr_info("sprd s_isp_pdev name %s\n", soc_isp->pdev->name);

	if (of_device_is_compatible(isp_node, "sprd,isp")) {
		if (of_property_read_u32_index(isp_node,
			"sprd,isp-count", 0, &count)) {
			pr_err("fail to parse the property of sprd,isp-count\n");
			return -EINVAL;
		}

		if (count > 1) {
			pr_err("fail to count isp number: %d", count);
			return -EINVAL;
		}
		*isp_count = count;

		/* read clk from dts */
		soc_isp->core_eb = of_clk_get_by_name(isp_node, "isp_eb");
		if (IS_ERR_OR_NULL(soc_isp->core_eb)) {
			pr_err("fail to read dts isp eb\n");
			return -EFAULT;
		}
		soc_isp->axi_eb = of_clk_get_by_name(isp_node, "isp_axi_eb");
		if (IS_ERR_OR_NULL(soc_isp->core_eb)) {
			pr_err("fail to read dts isp axi eb\n");
			return -EFAULT;
		}
		soc_isp->clk = of_clk_get_by_name(isp_node, "isp_clk");
		if (IS_ERR_OR_NULL(soc_isp->core_eb)) {
			pr_err("fail to read dts isp clk\n");
			return -EFAULT;
		}
		soc_isp->clk_parent = of_clk_get_by_name(isp_node, "isp_clk_parent");
		if (IS_ERR_OR_NULL(soc_isp->core_eb)) {
			pr_err("fail to read dts isp clk parent\n");
			return -EFAULT;
		}
		soc_isp->clk_default = clk_get_parent(soc_isp->clk);

		iommu_node = of_parse_phandle(isp_node, "iommus", 0);
		if (iommu_node) {
			if (of_address_to_resource(iommu_node, 0, &res))
				pr_err("fail to get ISP IOMMU addr\n");
			else {
				reg_base = ioremap(res.start, res.end - res.start + 1);
				if (!reg_base)
					pr_err("fail to map ISP IOMMU base\n");
				else
					s_isp_mmubase = (unsigned long)reg_base;
			}
		}
		pr_info("ISP IOMMU Base  0x%lx\n", s_isp_mmubase);

		/* qos dt parse */
		qos_node = of_parse_phandle(isp_node, "isp_qos", 0);
		if (qos_node) {
			uint8_t val;

			if (of_property_read_u8(qos_node, "awqos-high", &val)) {
				pr_warn("isp awqos-high reading fail.\n");
				val = 7;
			}
			soc_isp->awqos_high = (uint32_t)val;
			if (of_property_read_u8(qos_node, "awqos-low", &val)) {
				pr_warn("isp awqos-low reading fail.\n");
				val = 6;
			}
			soc_isp->awqos_low = (uint32_t)val;
			if (of_property_read_u8(qos_node, "arqos-high", &val)) {
				pr_warn("isp arqos-high reading fail.\n");
				val = 7;
			}
			soc_isp->arqos_high = (uint32_t)val;
			if (of_property_read_u8(qos_node, "arqos-low", &val)) {
				pr_warn("isp arqos-low reading fail.\n");
				val = 6;
			}
			soc_isp->arqos_low = (uint32_t)val;
			pr_info("get isp qos node. r: %d %d w: %d %d\n",
				soc_isp->arqos_high, soc_isp->arqos_low,
				soc_isp->awqos_high, soc_isp->awqos_low);
		} else {
			soc_isp->awqos_high = 7;
			soc_isp->awqos_low = 6;
			soc_isp->arqos_high = 7;
			soc_isp->arqos_low = 6;
		}

		soc_isp->cam_ahb_gpr = syscon_regmap_lookup_by_phandle(isp_node,
			"sprd,cam-ahb-syscon");
		if (IS_ERR_OR_NULL(soc_isp->cam_ahb_gpr)) {
			pr_err("fail to get sprd,cam-ahb-syscon");
			return PTR_ERR(soc_isp->cam_ahb_gpr);
		}

		args_count = syscon_get_args_by_name(isp_node, "reset",
			sizeof(args), args);
		if (args_count == ARRAY_SIZE(args)) {
			ip_isp->syscon.rst = args[0];
			ip_isp->syscon.rst_mask = args[1];
		} else {
			pr_err("fail to get isp reset syscon\n");
			return -EINVAL;
		}

		args_count = syscon_get_args_by_name(isp_node, "isp_ahb_reset",
			sizeof(args), args);
		if (args_count == ARRAY_SIZE(args))
			ip_isp->syscon.rst_ahb_mask = args[1];

		args_count = syscon_get_args_by_name(isp_node, "isp_vau_reset",
			sizeof(args), args);
		if (args_count == ARRAY_SIZE(args))
			ip_isp->syscon.rst_vau_mask = args[1];

		if (of_address_to_resource(isp_node, i, &res))
			pr_err("fail to get isp phys addr\n");

		ip_isp->phy_base = (unsigned long)res.start;
		isp_phys_base[0] = ip_isp->phy_base;
		pr_info("isp phys reg base is %lx\n", isp_phys_base[0]);
		reg_base = of_iomap(isp_node, i);
		if (!reg_base) {
			pr_err("fail to get isp reg_base %d\n", i);
			return -ENXIO;
		}

		ip_isp->reg_base = (unsigned long)reg_base;
		s_isp_regbase[0] = ip_isp->reg_base;

		for (i = 0; i < ISP_LOGICAL_COUNT; i++) {
			s_isp_irq_no[i] = irq_of_parse_and_map(isp_node, i);
			if (s_isp_irq_no[i] <= 0) {
				pr_err("fail to get isp irq %d\n", i);
				return -EFAULT;
			}

			pr_info("ISP%d dts OK! regbase %lx, irq %d\n", i,
				s_isp_regbase[0], s_isp_irq_no[i]);
		}
	} else {
		pr_err("fail to match isp device node\n");
		return -EINVAL;
	}

	return 0;
}
