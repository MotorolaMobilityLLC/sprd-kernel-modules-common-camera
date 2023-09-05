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

#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <sprd_mm.h>

#include "dcam_core.h"
#include "dcam_reg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "LSC: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define LENS_LOAD_TIMEOUT 1000

struct lsc_slice {
	uint32_t relative_x;
	uint32_t current_x;
};

int dcam_init_lsc_slice(void *in, uint32_t online)
{
	uint32_t idx, val = 0;
	uint32_t start_roi = 0;
	uint32_t grid_x_num_slice = 0;
	struct lsc_slice slice;
	struct dcam_dev_lsc_info *info = NULL;
	struct dcam_dev_lsc_param *param = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct dcam_isp_k_block *blk_dcam_pm;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_force_copy copyarg;

	blk_dcam_pm = (struct dcam_isp_k_block *)in;
	dev = (struct dcam_pipe_dev *)blk_dcam_pm->dev;
	hw = dev->hw;
	param = &blk_dcam_pm->lsc;
	idx = blk_dcam_pm->idx;
	info = &param->lens_info;
	hw_ctx = &dev->hw_ctx[blk_dcam_pm->idx];

	/* need update grid_x_num and more when offline slice*/
	if (online == 0 && hw_ctx->dcam_slice_mode == CAM_OFFLINE_SLICE_HW) {
		start_roi = hw_ctx->slice_info.cur_slice ->start_x - DCAM_OVERLAP;
		grid_x_num_slice = ((hw_ctx->slice_info.cur_slice->size_x + DCAM_OVERLAP) / 2
				+ info->grid_width - 1) / info->grid_width + 3;
	} else
		grid_x_num_slice = info->grid_x_num;

	slice.current_x = (start_roi / 2) / info->grid_width;
	slice.relative_x = (start_roi / 2) % info->grid_width;

	/* only for slice mode */
	val = ((slice.relative_x & 0xff) << 16) | (slice.current_x & 0x1ff);
	DCAM_REG_WR(idx, DCAM_LENS_SLICE_CTRL0, val);
	DCAM_REG_MWR(idx, DCAM_LENS_SLICE_CTRL1, 0xff, grid_x_num_slice);

	/* force copy must be after first load done and load clear */
	copyarg.id = DCAM_CTRL_COEF;
	copyarg.idx = idx;
	copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
	hw->dcam_ioctl(hw, idx, DCAM_HW_CFG_FORCE_COPY, &copyarg);

	pr_info("w %d, grid len %d grid %d  num_t %d (%d, %d)\n",
		info->weight_num, info->gridtab_len, info->grid_width,
		info->grid_num_t, info->grid_x_num, info->grid_y_num);
	return 0;
}

int dcam_init_lsc(void *in, uint32_t online)
{
	int ret = 0;
	uint32_t idx, i = 0;
	uint32_t dst_w_num = 0;
	uint32_t val, lens_load_flag;
	uint32_t buf_sel, offset, hw_addr;
	uint32_t start_roi = 0;
	uint16_t *w_buff = NULL, *gain_tab = NULL;
	uint32_t grid_x_num_slice = 0;
	struct lsc_slice slice;
	struct dcam_dev_lsc_info *info;
	struct dcam_dev_lsc_param *param;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct dcam_isp_k_block *blk_dcam_pm;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_force_copy copyarg;

	blk_dcam_pm = (struct dcam_isp_k_block *)in;
	dev = (struct dcam_pipe_dev *)blk_dcam_pm->dev;
	hw = dev->hw;
	param = &blk_dcam_pm->lsc;
	hw_ctx = &dev->hw_ctx[blk_dcam_pm->idx];

	idx = blk_dcam_pm->idx;
	info = &param->lens_info;
	param->load_trigger = 0;
	copyarg.id = DCAM_CTRL_COEF;
	copyarg.idx = idx;
	copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
	/* debugfs bypass, not return, need force copy */
	if (g_dcam_bypass[idx] & (1 << _E_LSC))
		info->bypass = 1;
	if (info->bypass) {
		pr_debug("bypass\n");
		DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 1);
		hw->dcam_ioctl(hw, idx, DCAM_HW_CFG_FORCE_COPY, &copyarg);
		return 0;
	}

	/* need update grid_x_num and more when offline slice*/
	if (online == 0 && hw_ctx->dcam_slice_mode == CAM_OFFLINE_SLICE_HW) {
		start_roi = 0;
		grid_x_num_slice = ((hw_ctx->slice_info.cur_slice->size_x + DCAM_OVERLAP) / 2
				+ info->grid_width - 1) / info->grid_width + 3;
	} else {
		grid_x_num_slice = info->grid_x_num;
	}

	slice.current_x = (start_roi / 2) / info->grid_width;
	slice.relative_x = (start_roi / 2) % info->grid_width;

	pr_info("w %d,  grid len %d grid %d  num_t %d (%d, %d)\n",
		info->weight_num, info->gridtab_len, info->grid_width,
		info->grid_num_t, info->grid_x_num, info->grid_y_num);

	w_buff = (uint16_t *)param->weight_tab;
	gain_tab = (uint16_t *)param->buf.addr_k + param->grid_offset;
	hw_addr = (uint32_t)param->buf.iova[CAM_BUF_IOMMUDEV_DCAM] + param->grid_offset;
	if (!w_buff || !gain_tab || !hw_addr || info->grid_width >= LSC_WEI_TABLE_MAX_NUM) {
		pr_err("fail to get buf %p %p %x grid_width %x\n", w_buff, gain_tab, hw_addr, info->grid_width);
		ret = -EPERM;
		goto exit;
	}

	/* step1:  load weight tab */
	dst_w_num = (info->grid_width >> 1) + 1;
	offset = LSC_WEI_TABLE_START;
	for (i = 0; i < dst_w_num; i++) {
		val = (((uint32_t)w_buff[i * 3 + 0]) & 0xFFFF) |
			((((uint32_t)w_buff[i * 3 + 1]) & 0xFFFF) << 16);
		DCAM_REG_WR(idx, offset, val);
		offset += 4;
		val = (((uint32_t)w_buff[i * 3 + 2]) & 0xFFFF);
		DCAM_REG_WR(idx, offset, val);
		offset += 4;
	}
	pr_debug("write weight tab done\n");

	/* enable internal access sram */
	DCAM_REG_MWR(idx, DCAM_APB_SRAM_CTRL, BIT_0, 1);

	for (i = 0; i < info->grid_num_t * 4; i += 8) {
		pr_debug("gain %04x %04x %04x %04x %04x %04x %04x %04x\n",
			gain_tab[0], gain_tab[1], gain_tab[2], gain_tab[3],
			gain_tab[4], gain_tab[5], gain_tab[6], gain_tab[7]);
		gain_tab += 8;
	}

	/* step2: load grid table */
	DCAM_REG_WR(idx, DCAM_LENS_BASE_RADDR, hw_addr);
	DCAM_REG_MWR(idx, DCAM_LENS_GRID_NUMBER, 0x7FF, info->grid_num_t & 0x7FF);

	/* trigger load immediately */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_CLR, BIT_2 | BIT_0, (0 << 2) | 1);

	/* step3: configure lens enable and grid param...*/
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 0);

	val = ((info->grid_width & 0x1ff) << 16) |
			((info->grid_y_num & 0xff) << 8) |
			(info->grid_x_num & 0xff);
	DCAM_REG_WR(idx, DCAM_LENS_GRID_SIZE, val);
	/* only for slice mode */
	val = ((slice.relative_x & 0xff) << 16) |
			(slice.current_x & 0x1ff);
	DCAM_REG_WR(idx, DCAM_LENS_SLICE_CTRL0, val);
	DCAM_REG_MWR(idx, DCAM_LENS_SLICE_CTRL1, 0xff, grid_x_num_slice);

	/* lens_load_buf_sel toggle */
	val = DCAM_REG_RD(idx, DCAM_LENS_LOAD_ENABLE);
	buf_sel = !((val & BIT_1) >> 1);
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_1, buf_sel << 1);
	pr_debug("buf_sel %d\n", buf_sel);

	/* step 4: if initialized config, polling lens_load_flag done. */
	i = 0;
	while (i++ < LENS_LOAD_TIMEOUT) {
		val = DCAM_REG_RD(idx, DCAM_LENS_LOAD_ENABLE);
		lens_load_flag = (val & BIT_2);
		if (lens_load_flag)
			break;
	}

	/* clear lens_load_flag */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_CLR, BIT_1, (1 << 1));

	/* force copy must be after first load done and load clear */
	hw->dcam_ioctl(hw, idx, DCAM_HW_CFG_FORCE_COPY, &copyarg);

	if (i >= LENS_LOAD_TIMEOUT) {
		pr_err("fail to load lens grid table.\n");
		ret = -EPERM;
		goto exit;
	}

	/* trigger load to another buffer when next sof */
	/* in initial phase, there are default data in both buffers(sram)
	 * if we just load to one buffer, another buffer with default
	 * second time update lsc, buf_sel shadow and buffer loading
	 * maybe mismatched and if the buffer without loading data
	 * is applied, then image corruption will be observed.
	 * therefore we trigger loading to another buffer to avoid this case.
	 */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_CLR, BIT_2 | BIT_0, (1 << 2) | 1);
	param->load_trigger = 1;

	return 0;

exit:
	/* bypass lsc if there is exception */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 1);
	hw->dcam_ioctl(hw, idx, DCAM_HW_CFG_FORCE_COPY, &copyarg);
	return ret;
}

int dcam_update_lsc(void *in)
{
	int ret = 0;
	uint32_t idx, i = 0;
	uint32_t dst_w_num = 0;
	uint32_t val, lens_load_flag;
	uint32_t buf_sel, offset, hw_addr;
	uint16_t *w_buff = NULL, *gain_tab = NULL;
	uint32_t grid_x_num_slice = 0;
	struct dcam_dev_lsc_info *info = NULL;
	struct dcam_dev_lsc_param *param = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct dcam_isp_k_block *blk_dcam_pm;
	struct cam_hw_info *hw = NULL;
	struct dcam_hw_auto_copy copyarg;

	blk_dcam_pm = (struct dcam_isp_k_block *)in;
	dev = (struct dcam_pipe_dev *)blk_dcam_pm->dev;
	hw = dev->hw;
	param = &blk_dcam_pm->lsc;
	hw_ctx = &dev->hw_ctx[blk_dcam_pm->idx];

	idx = blk_dcam_pm->idx;
	info = &param->lens_info;
	if (g_dcam_bypass[idx] & (1 << _E_LSC))
		return 0;

	if (info->bypass) {
		pr_debug("bypass\n");
		DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 1);
		return 0;
	}

	w_buff = (uint16_t *)param->weight_tab;
	gain_tab = (uint16_t *)param->buf.addr_k;
	hw_addr = (uint32_t)param->buf.iova[CAM_BUF_IOMMUDEV_DCAM];
	if (!w_buff || !gain_tab || !hw_addr || info->grid_width >= LSC_WEI_TABLE_MAX_NUM) {
		pr_err("fail to get buf %px %px %x grid_width %x\n", w_buff, gain_tab, hw_addr, info->grid_width);
		ret = -EPERM;
		goto exit;
	}

	/* step0 */
	/* If it is not first configuration before stream on.
	 * we should check lens_load_flag to make sure
	 * last load is already done.
	 */
	if (param->load_trigger) {
		val = DCAM_REG_RD(idx, DCAM_LENS_LOAD_ENABLE);
		pr_debug("check load status 0x%x\n", val);

		lens_load_flag = (val & BIT_2);
		if (lens_load_flag == 0) {
			pr_info("last lens load is not done. skip\n");
			return 0;
		}
		/* clear lens_load_flag */
		DCAM_REG_MWR(idx, DCAM_LENS_LOAD_CLR, BIT_1, 1 << 1);
		param->load_trigger = 0;
	}

	/* step1:  load weight tab */
	dst_w_num = (info->grid_width >> 1) + 1;
	offset = LSC_WEI_TABLE_START;
	for (i = 0; i < dst_w_num; i++) {
		val = (((uint32_t)w_buff[i * 3 + 0]) & 0xFFFF) |
			((((uint32_t)w_buff[i * 3 + 1]) & 0xFFFF) << 16);
		DCAM_REG_WR(idx, offset, val);
		offset += 4;
		val = (((uint32_t)w_buff[i * 3 + 2]) & 0xFFFF);
		DCAM_REG_WR(idx, offset, val);
		offset += 4;
	}
	pr_debug("update weight tab done\n");

	/* step2: load grid table */
	DCAM_REG_WR(idx, DCAM_LENS_BASE_RADDR, hw_addr);
	DCAM_REG_MWR(idx, DCAM_LENS_GRID_NUMBER, 0x7FF,
			info->grid_num_t & 0x7FF);

	/* bit0: 1 - enable loading */
	/* bit2: 0 - trigger load immediately, 1 - trigger load next sof */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_CLR, BIT_2 | BIT_0, (0 << 2) | 1);
	param->load_trigger = 1;

	/* step3: configure lens enable and grid param...*/
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 0);

	val = ((info->grid_width & 0x1ff) << 16) |
			((info->grid_y_num & 0xff) << 8) |
			(info->grid_x_num & 0xff);
	DCAM_REG_WR(idx, DCAM_LENS_GRID_SIZE, val);
	/* only for slice mode */
	DCAM_REG_WR(idx, DCAM_LENS_SLICE_CTRL0, 0x0);
	grid_x_num_slice = info->grid_x_num;
	DCAM_REG_MWR(idx, DCAM_LENS_SLICE_CTRL1, 0xff, grid_x_num_slice);
	pr_debug("update grid %d x %d y %d\n", info->grid_width,
			info->grid_x_num, info->grid_y_num);

	/* lens_load_buf_sel toggle */
	val = DCAM_REG_RD(idx, DCAM_LENS_LOAD_ENABLE);
	buf_sel = !((val & BIT_1) >> 1);
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_1, buf_sel << 1);

	/* step 4: auto cpy lens registers next sof */
	copyarg.id = DCAM_CTRL_BIN;
	copyarg.idx = idx;
	copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
	hw->dcam_ioctl(hw, idx, DCAM_HW_CFG_AUTO_COPY, &copyarg);

	pr_debug("done\n");
	return 0;

exit:
	/* bypass lsc if there is exception */
	DCAM_REG_MWR(idx, DCAM_LENS_LOAD_ENABLE, BIT_0, 1);
	return ret;
}

int dcam_k_lsc_block(struct dcam_isp_k_block *p)
{
	int ret = 0;
	uint16_t *w_buff = NULL, *gain_tab = NULL;
	struct dcam_dev_lsc_info *info;
	struct dcam_dev_lsc_param *param;
	unsigned long wtab_uaddr, gtab_uaddr;

	param = &p->lsc;
	info = &param->lens_info;
	if (param->weight_tab == NULL) {
		pr_err("fail to get weight_tab buf\n");
		ret = -ENOMEM;
		goto exit;
	}

	if (info->weight_num > DCAM_LSC_WEIGHT_TAB_SIZE) {
		pr_err("fail to get weight length wrong %d\n", info->weight_num);
		ret = -EPERM;
		goto exit;
	}

	gain_tab = (uint16_t *)param->buf.addr_k;
	if (IS_ERR_OR_NULL(gain_tab) || (info->gridtab_len > DCAM_LSC_BUF_SIZE)) {
		pr_err("fail to get gain tab, length 0x%x\n", info->gridtab_len);
		ret = -EPERM;
		goto exit;
	}

	w_buff = (uint16_t *)param->weight_tab;
	wtab_uaddr = (unsigned long)info->weight_tab_addr;
	ret = copy_from_user((void *)w_buff,
			(void __user *)wtab_uaddr, info->weight_num);
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		ret = -EPERM;
		goto exit;
	}

	gtab_uaddr = (unsigned long)info->grid_tab_addr;
	ret = copy_from_user((void *)gain_tab,
			(void __user *)gtab_uaddr, info->gridtab_len);
	if (ret != 0) {
		pr_err("fail to copy from user, ret = %d\n", ret);
		ret = -EPERM;
		goto exit;
	}
exit:
	return ret;
}

int dcam_k_cfg_lsc(struct isp_io_param *param, struct dcam_isp_k_block *p)
{
	int ret = 0;

	switch (param->property) {
	case DCAM_PRO_LSC_BLOCK:
		pr_debug("DCAM_PRO_LSC_BLOCK\n");
		break;
	default:
		pr_err("fail to support property %d\n",
			param->property);
		return -EINVAL;
	}

	ret = copy_from_user(&p->lsc.lens_info,
			param->property_param, sizeof(p->lsc.lens_info));
	if (ret) {
		pr_err("fail to copy from user. ret %d\n", ret);
		return ret;
	}

	pr_debug("update all\n");

	ret = dcam_k_lsc_block(p);

	return ret;
}
