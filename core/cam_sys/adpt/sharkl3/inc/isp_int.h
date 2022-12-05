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

#ifndef _ISP_INT_H_
#define _ISP_INT_H_

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include "cam_types.h"
#include "isp_dev.h"
#include "isp_interface.h"

enum isp_irq_id {
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_SHADOW_DONE,
	ISP_INT_DISPATCH_DONE,
	ISP_INT_STORE_DONE_OUT,

	ISP_INT_STORE_DONE_PRE,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_STORE_DONE_VID_SKIP,
	ISP_INT_NR3_ALL_DONE,

	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_NULL9,
	ISP_INT_NULL10,
	ISP_INT_FMCU_LOAD_DONE,

	ISP_INT_FMCU_CONFIG_DONE,
	ISP_INT_FMCU_SHADOW_DONE,
	ISP_INT_FMCU_CMD_X,
	ISP_INT_FMCU_TIMEOUT,

	ISP_INT_FMCU_CMD_ERROR,
	ISP_INT_FMCU_STOP_DONE,
	ISP_INT_RESERVED,
	ISP_INT_RESERVED1,
	ISP_INT_RESERVED2,
	ISP_INT_HIST_CAL_DONE,
	ISP_INT_HIST2_CAL_DONE,
	ISP_INT_NUMBER0,
};

enum isp_mmu_irq {
	RAW_OUT_OF_RNGE_R,
	RAW_OUT_OF_RNGE_W,
	RAW_UNSECURE_R,
	RAW_UNSECURE_W,
	RAW_INVALID_PAGE_R,
	RAW_INVALID_PAGE_W,
	RAW_INVALID_ID_R,
	RAW_INVALID_ID_W,
	MMU_INT_NUMBER3,
};
#define ISP_INT_FMCU_STORE_DONE               ISP_INT_FMCU_CONFIG_DONE

#define ISP_INT_LINE_MASK_ERR                 \
	((1 << ISP_INT_FMCU_TIMEOUT) |        \
	(1 << ISP_INT_FMCU_CMD_ERROR))

#define ISP_INT_LINE_MASK_MMU                 \
	((1 << RAW_OUT_OF_RNGE_R) |           \
	(1 << RAW_OUT_OF_RNGE_W) |            \
	(1 << RAW_UNSECURE_R) |               \
	(1 << RAW_UNSECURE_W) |               \
	(1 << RAW_INVALID_PAGE_R) |           \
	(1 << RAW_INVALID_PAGE_W) |           \
	(1 << RAW_INVALID_ID_R) |             \
	(1 << RAW_INVALID_ID_W))

#define ISP_INT_LINE_FMCU_MASK                \
	((1 << ISP_INT_FMCU_LOAD_DONE) |      \
	(1 << ISP_INT_FMCU_CONFIG_DONE) |     \
	(1 << ISP_INT_FMCU_SHADOW_DONE) |     \
	(1 << ISP_INT_FMCU_CMD_X) |           \
	(1 << ISP_INT_FMCU_TIMEOUT) |         \
	(1 << ISP_INT_FMCU_CMD_ERROR) |       \
	(1 << ISP_INT_FMCU_STOP_DONE))

struct isp_int_ctxs_com {
	unsigned long irq_offset;
	uint32_t err_mask;
	uint32_t irq_numbers;
	const uint32_t *irq_vect;
};

struct ispint_isr_irq {
	uint32_t irq_line;
	uint32_t irq_line1;
};

int isp_int_irq_hw_cnt_reset(int ctx_id);
int isp_int_irq_hw_cnt_trace(int ctx_id);
int isp_int_irq_request(struct device *p_dev,
		uint32_t *irq_no, void *isp_handle);
int isp_int_irq_sw_cnt_reset(int ctx_id);
int isp_int_irq_sw_cnt_trace(int ctx_id);
int isp_int_irq_free(struct device *p_dev, void *isp_handle);

#endif
