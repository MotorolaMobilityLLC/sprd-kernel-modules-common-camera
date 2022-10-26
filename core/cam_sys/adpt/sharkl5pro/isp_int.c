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

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <sprd_mm.h>

#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"
#include "isp_interface.h"
#include "isp_reg.h"
#include "dcam_reg.h"
#include "isp_int.h"
#include "isp_scaler_node.h"
#include "isp_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_INT: %d %d %s : " fmt, current->pid, __LINE__, __func__

//#define ISP_INT_RECORD 1
#ifdef ISP_INT_RECORD
#define INT_RCD_SIZE 0x10000
static uint32_t isp_int_recorder[ISP_CONTEXT_HW_NUM][32][INT_RCD_SIZE];
static uint32_t int_index[ISP_CONTEXT_HW_NUM][32];
#endif

uint32_t irq_done[ISP_CONTEXT_HW_NUM][32];
uint32_t irq_done_sw[ISP_CONTEXT_SW_NUM][32];

char *isp_dev_name[] = {"isp0", "isp1"};

typedef void(*isp_isr)(enum isp_context_hw_id idx, void *param);

static const uint32_t isp_irq_process[] = {
	ISP_INT_SHADOW_DONE,
	ISP_INT_DISPATCH_DONE,
	ISP_INT_STORE_DONE_PRE,
	ISP_INT_STORE_DONE_VID,
	ISP_INT_NR3_ALL_DONE,
	ISP_INT_NR3_SHADOW_DONE,
	ISP_INT_STORE_DONE_THUMBNAIL,
	ISP_INT_RGB_LTMHISTS_DONE,
	ISP_INT_FMCU_LOAD_DONE,
	ISP_INT_FMCU_SHADOW_DONE,
	ISP_INT_HIST_CAL_DONE,
	ISP_INT_ISP_ALL_DONE,
	ISP_INT_FMCU_STORE_DONE,
};

static inline void ispint_isp_int_record(uint32_t cfg_id, enum isp_context_hw_id c_id, uint32_t irq_line)
{
	uint32_t k;

	if (cfg_id < 0) {
		pr_err("fail to get cfg_id:%d", cfg_id);
		return;
	}

	for (k = 0; k < 32; k++) {
		if (irq_line & (1 << k))
			irq_done[c_id][k]++;
	}

	for (k = 0; k < 32; k++) {
		if (irq_line & (1 << k))
			irq_done_sw[cfg_id][k]++;
	}

#ifdef ISP_INT_RECORD
	{
		uint32_t cnt, time, int_no;
		struct timespec cur_ts;

		ktime_get_ts(&cur_ts);
		time = (uint32_t)(cur_ts.tv_sec & 0xffff);
		time <<= 16;
		time |= (uint32_t)((cur_ts.tv_nsec / (NSEC_PER_USEC * 100)) & 0xffff);
		for (int_no = 0; int_no < 32; int_no++) {
			if (irq_line & BIT(int_no)) {
				cnt = int_index[c_id][int_no];
				isp_int_recorder[c_id][int_no][cnt] = time;
				cnt++;
				int_index[c_id][int_no] = (cnt & (INT_RCD_SIZE - 1));
			}
		}
	}
#endif
}

static void ispint_all_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;

	pctx_hw = &dev->hw_ctx[hw_idx];
	if (pctx_hw->fmcu_handle) {
		pr_debug("fmcu started. skip all done, hw_idx %d\n ", hw_idx);
		return;
	}

	pr_debug("cxt_id:%d all done.\n", hw_idx);
	if (pctx_hw->valid_slc_num) {
		pr_debug("slice done. last %d\n", pctx_hw->is_last_slice);
		if (!pctx_hw->is_last_slice) {
			complete(&pctx_hw->slice_done);
			return;
		}
		complete(&pctx_hw->slice_done);
		pr_debug("frame done.\n");
	}
	if (pctx_hw->postproc_func)
		pctx_hw->postproc_func(dev, hw_idx, POSTPROC_FRAME_DONE);
}

static void ispint_shadow_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d shadow done.\n", idx);
}

static void ispint_dispatch_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d done.\n", idx);
}

static void ispint_pre_store_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d done.\n", idx);
}

static void ispint_vid_store_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d done.\n", idx);
}

static void ispint_thumb_store_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d done.\n", idx);
}

static void ispint_fmcu_store_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx_hw = &dev->hw_ctx[hw_idx];
	if (pctx_hw->fmcu_handle == NULL) {
		pr_warn("warning: no fmcu for hw %d\n", hw_idx);
		return;
	}
	if (pctx_hw->postproc_func)
		pctx_hw->postproc_func(dev, hw_idx, POSTPROC_FRAME_DONE);
	if (pctx_hw->enable_slowmotion == 1 && pctx_hw->postproc_func)
		pctx_hw->postproc_func(dev, hw_idx, POSTPROC_SLOWMOTION_FRAMEDONE);
}

static void ispint_fmcu_shadow_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx_hw = &dev->hw_ctx[hw_idx];
	if (pctx_hw->fmcu_handle == NULL) {
		pr_warn("warning: no fmcu for hw %d\n", hw_idx);
		return;
	}

	pr_debug("cxt_id:%d done.\n", hw_idx);
}

static void ispint_fmcu_load_done(enum isp_context_hw_id idx, void *isp_handle)
{
	pr_debug("cxt_id:%d done.\n", idx);
}

static void ispint_3dnr_all_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	pr_debug("3dnr all done. cxt_id:%d\n", hw_idx);
}

static void ispint_3dnr_shadow_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	pr_debug("3dnr shadow done. cxt_id:%d\n", hw_idx);
}

static void ispint_rgb_ltm_hists_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_pipe_dev *dev = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;

	pctx_hw = &dev->hw_ctx[hw_idx];
	if (pctx_hw->postproc_func)
		pctx_hw->postproc_func(dev, hw_idx, POSTPROC_RGB_LTM_HISTS_DONE);
}

static int ispint_err_pre_proc(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	unsigned long addr = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx_hw = &dev->hw_ctx[hw_idx];

	pr_info("isp hw %d, cfg id:%d\n", hw_idx, pctx_hw->cfg_id);
	pr_info("ISP: sw cfg Register list\n");
	/* TBD: should add necessary cfg register print */
	for (addr = 0xc010; addr <= 0xc150; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(pctx_hw->cfg_id, addr),
			ISP_REG_RD(pctx_hw->cfg_id, addr + 4),
			ISP_REG_RD(pctx_hw->cfg_id, addr + 8),
			ISP_REG_RD(pctx_hw->cfg_id, addr + 12));
	}

	pr_info("ISP: hw Register list\n");
	/* TBD: should add necessary hw register print */
	for (addr = 0xc010; addr <= 0xc150; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}

	return 0;
}

static void ispint_hist_cal_done(enum isp_context_hw_id hw_idx, void *isp_handle)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx_hw = &dev->hw_ctx[hw_idx];
	if (pctx_hw->postproc_func)
		pctx_hw->postproc_func(dev, hw_idx, POSTPROC_HIST_CAL_DONE);
	return;
}

static isp_isr isp_isr_handler[32] = {
	[ISP_INT_ISP_ALL_DONE] = ispint_all_done,
	[ISP_INT_SHADOW_DONE] = ispint_shadow_done,
	[ISP_INT_DISPATCH_DONE] = ispint_dispatch_done,
	[ISP_INT_STORE_DONE_PRE] = ispint_pre_store_done,
	[ISP_INT_STORE_DONE_VID] = ispint_vid_store_done,
	[ISP_INT_NR3_ALL_DONE]	 = ispint_3dnr_all_done,
	[ISP_INT_NR3_SHADOW_DONE] = ispint_3dnr_shadow_done,
	[ISP_INT_STORE_DONE_THUMBNAIL] = ispint_thumb_store_done,
	[ISP_INT_FMCU_LOAD_DONE] = ispint_fmcu_load_done,
	[ISP_INT_FMCU_SHADOW_DONE] = ispint_fmcu_shadow_done,
	[ISP_INT_FMCU_STORE_DONE] = ispint_fmcu_store_done,
	[ISP_INT_HIST_CAL_DONE] = ispint_hist_cal_done,
	[ISP_INT_RGB_LTMHISTS_DONE]  = ispint_rgb_ltm_hists_done,
};

static struct isp_int_ctx {
	unsigned long reg_offset;
	uint32_t err_mask;
	uint32_t irq_numbers;
	const uint32_t *irq_vect;
} isp_int_ctxs[4] = {
		{ /* P0 */
			ISP_P0_INT_BASE,
			ISP_INT_LINE_MASK_ERR | ISP_INT_LINE_MASK_MMU,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{ /* C0 */
			ISP_C0_INT_BASE,
			ISP_INT_LINE_MASK_ERR | ISP_INT_LINE_MASK_MMU,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{ /* P1 */
			ISP_P1_INT_BASE,
			ISP_INT_LINE_MASK_ERR | ISP_INT_LINE_MASK_MMU,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
		{ /* C1 */
			ISP_C1_INT_BASE,
			ISP_INT_LINE_MASK_ERR | ISP_INT_LINE_MASK_MMU,
			(uint32_t)ARRAY_SIZE(isp_irq_process),
			isp_irq_process,
		},
};

static void ispint_iommu_regs_dump(void)
{
	uint32_t reg = 0;
	uint32_t val[4];

	for (reg = ISP_MMU_VERSION; reg <= ISP_MMU_INT_RAW; reg += 16) {
		val[0] = ISP_MMU_RD(reg);
		val[1] = ISP_MMU_RD(reg + 4);
		val[2] = ISP_MMU_RD(reg + 8);
		val[3] = ISP_MMU_RD(reg + 12);
		pr_err("offset=0x%04x: %08x %08x %08x %08x\n",
			reg, val[0], val[1], val[2], val[3]);
	}

	pr_err("fetch y %08x u %08x v %08x \n",
		ISP_HREG_RD(ISP_FETCH_SLICE_Y_ADDR),
		ISP_HREG_RD(ISP_FETCH_SLICE_U_ADDR),
		ISP_HREG_RD(ISP_FETCH_SLICE_V_ADDR));

	pr_err("fetch 3dnr y %08x uv %08x\n",
		ISP_HREG_RD(ISP_3DNR_MEM_CTRL_FT_CUR_LUMA_ADDR),
		ISP_HREG_RD(ISP_3DNR_MEM_CTRL_FT_CUR_CHROMA_ADDR));

	pr_err("store pre cap y %08x u %08x v %08x \n",
		ISP_HREG_RD(ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_Y_ADDR),
		ISP_HREG_RD(ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_U_ADDR),
		ISP_HREG_RD(ISP_STORE_PRE_CAP_BASE + ISP_STORE_SLICE_V_ADDR));

	pr_err("store vid y %08x u %08x v %08x\n",
		ISP_HREG_RD(ISP_STORE_VID_BASE + ISP_STORE_SLICE_Y_ADDR),
		ISP_HREG_RD(ISP_STORE_VID_BASE + ISP_STORE_SLICE_U_ADDR),
		ISP_HREG_RD(ISP_STORE_VID_BASE + ISP_STORE_SLICE_V_ADDR));

	pr_err("store thumb y %08x u %08x v %08x\n",
		ISP_HREG_RD(ISP_STORE_THUMB_BASE + ISP_STORE_SLICE_Y_ADDR),
		ISP_HREG_RD(ISP_STORE_THUMB_BASE + ISP_STORE_SLICE_U_ADDR),
		ISP_HREG_RD(ISP_STORE_THUMB_BASE + ISP_STORE_SLICE_V_ADDR));
}

static struct ispint_isr_root ispint_isr_root_readint(uint32_t irq_offset)
{
	struct ispint_isr_root com = {0};

	com.irq_line = ISP_HREG_RD(irq_offset + ISP_INT_INT0);
	return com;
}

static void ispint_isr_root_writeint(uint32_t irq_offset, struct ispint_isr_root com)
{
	ISP_HREG_WR(irq_offset + ISP_INT_CLR0, com.irq_line);
}

static struct isp_int_ctxs_com ispint_ctxs_rd(int c_id)
{
	struct isp_int_ctxs_com com = {0};

	com.irq_offset = isp_int_ctxs[c_id].reg_offset;
	com.err_mask = isp_int_ctxs[c_id].err_mask;
	com.irq_numbers = isp_int_ctxs[c_id].irq_numbers;
	com.irq_vect = isp_int_ctxs[c_id].irq_vect;
	return com;
}

static int ispint_isr_root_unlikely(int c_id, struct ispint_isr_root com, struct isp_int_ctxs_com ctxs_com, void *priv)
{
	uint32_t val = 0;
	struct isp_pipe_dev *isp_handle = (struct isp_pipe_dev *)priv;
	struct isp_hw_context *pctx_hw = NULL;
	pctx_hw = &isp_handle->hw_ctx[c_id];

	if (unlikely(ctxs_com.err_mask & com.irq_line)) {
		pr_err("fail to get normal status ISP 0x%x\n", com.irq_line);
		if (com.irq_line & ISP_INT_LINE_MASK_MMU) {
			val = ISP_MMU_RD(ISP_MMU_INT_STS);

			if (val != pctx_hw->iommu_status) {
				pctx_hw->iommu_status = val;
				ispint_iommu_regs_dump();
			}
		}

		/*handle the error here*/
		if (ispint_err_pre_proc(c_id, isp_handle)) {
			pr_err("fail to handle the error here c_id %d irq_line 0x%x\n", c_id, com.irq_line);
			return IRQ_HANDLED;
		}
	}
	return 0;
}

static void ispint_isr_root_handle(int c_id, struct ispint_isr_root *com, struct isp_int_ctxs_com ctxs_com, void *priv)
{
	uint32_t k = 0;
	struct isp_pipe_dev *isp_handle = (struct isp_pipe_dev *)priv;

	for (k = 0; k < ctxs_com.irq_numbers; k++) {
		uint32_t irq_id = ctxs_com.irq_vect[k];

		if (com->irq_line & (1 << irq_id)) {
			if (isp_isr_handler[irq_id]) {
				isp_isr_handler[irq_id](
					c_id, isp_handle);
			}
		}
		com->irq_line  &= ~(1 << irq_id);
		if (!com->irq_line)
			break;
	}
}

static irqreturn_t ispint_isr_root(int irq, void *priv)
{
	int ret = 0;
	uint32_t iid = 0, sid = 0;
	enum isp_context_hw_id pctx_hw_id;
	struct isp_pipe_dev *isp_handle = (struct isp_pipe_dev *)priv;
	struct ispint_isr_root com = {0};
	struct isp_int_ctxs_com ctxs_com = {0};
	struct isp_hw_context *hw_ctx = NULL;
	struct isp_node *node = NULL;
	struct isp_yuv_scaler_node *yuv_scaler_node = NULL;
	struct camera_interrupt *interruption = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid dev\n");
		return IRQ_HANDLED;
	}
	if (atomic_read(&isp_handle->enable) == 0) {
		pr_err("fail to get isp_handle enable\n");
		return IRQ_HANDLED;
	}
	if (irq == isp_handle->irq_no[0]) {
		iid = 0;
	} else if (irq == isp_handle->irq_no[1]) {
		iid = 1;
	} else {
		pr_err("fail to get irq %d mismatched\n", irq);
		return IRQ_NONE;
	}
	pr_debug("isp irq %d, priv %p, iid %d\n", irq, priv, iid);
	for (sid = 0; sid < 2; sid++) {
		pctx_hw_id = (iid << 1) | sid;
		ctxs_com = ispint_ctxs_rd(pctx_hw_id);

		/* read the interrupt*/
		com = ispint_isr_root_readint(ctxs_com.irq_offset);
		if (unlikely((com.irq_line == 0) && (com.irq_line1 == 0)))
			continue;

		pr_debug("hw %d, irq_line: 0x%08x 0x%08x\n", pctx_hw_id, com.irq_line, com.irq_line1);
		hw_ctx = &isp_handle->hw_ctx[pctx_hw_id];
		ispint_isp_int_record(hw_ctx->cfg_id, pctx_hw_id, com.irq_line);

		/*clear the interrupt*/
		ispint_isr_root_writeint(ctxs_com.irq_offset, com);
		pr_debug("isp ctx %d irqno %d, INT: 0x%x\n", pctx_hw_id, iid, com.irq_line);

		if (hw_ctx->scaler_debug) {
			yuv_scaler_node = (struct isp_yuv_scaler_node *)hw_ctx->node;
			if (!yuv_scaler_node) {
				pr_err("fail to get valid node\n");
				return IRQ_HANDLED;
			}
			yuv_scaler_node->pctx_hw_id = pctx_hw_id;
			yuv_scaler_node->ctxs_com.irq_offset = ctxs_com.irq_offset;
			yuv_scaler_node->ctxs_com.err_mask = ctxs_com.err_mask;
			yuv_scaler_node->ctxs_com.irq_numbers = ctxs_com.irq_numbers;
			yuv_scaler_node->ctxs_com.irq_vect = ctxs_com.irq_vect;
			interruption = cam_queue_empty_interrupt_get();
			interruption->int_status = com.irq_line;
			interruption->int_status1 = com.irq_line1;

			ret = cam_queue_enqueue(&yuv_scaler_node->yuv_scaler_interrupt_queue, &interruption->list);
			if (ret) {
				pr_err("fail to enqueue int queue cnt %d max%d.\n",
					yuv_scaler_node->yuv_scaler_interrupt_queue.cnt, yuv_scaler_node->yuv_scaler_interrupt_queue.max);
				cam_queue_empty_interrupt_put(interruption);
				return IRQ_NONE;
			} else
				complete(&yuv_scaler_node->yuv_scaler_interrupt_thread.thread_com);
		} else {
			node = (struct isp_node *)hw_ctx->node;
			if (!node) {
				pr_err("fail to get valid node\n");
				return IRQ_HANDLED;
			}
			node->pctx_hw_id = pctx_hw_id;
			node->ctxs_com.irq_offset = ctxs_com.irq_offset;
			node->ctxs_com.err_mask = ctxs_com.err_mask;
			node->ctxs_com.irq_numbers = ctxs_com.irq_numbers;
			node->ctxs_com.irq_vect = ctxs_com.irq_vect;
			interruption = cam_queue_empty_interrupt_get();
			interruption->int_status = com.irq_line;
			interruption->int_status1 = com.irq_line1;

			ret = cam_queue_enqueue(&node->isp_interrupt_queue, &interruption->list);
			if (ret) {
				pr_err("fail to enqueue int queue cnt %d max%d.\n",
					node->isp_interrupt_queue.cnt, node->isp_interrupt_queue.max);
				cam_queue_empty_interrupt_put(interruption);
				return IRQ_NONE;
			} else
				complete(&node->isp_interrupt_thread.thread_com);
		}
	}
	return IRQ_HANDLED;
}

int isp_int_isp_irq_cnt_reset(int ctx_id)
{
	if (ctx_id < ISP_CONTEXT_HW_NUM)
		memset(irq_done[ctx_id], 0, sizeof(irq_done[ctx_id]));

#ifdef ISP_INT_RECORD
	if (ctx_id < ISP_CONTEXT_HW_NUM) {
		memset(isp_int_recorder[ctx_id][0], 0, sizeof(isp_int_recorder) / ISP_CONTEXT_HW_NUM);
		memset(int_index[ctx_id], 0, sizeof(int_index) / ISP_CONTEXT_HW_NUM);
	}
#endif
	return 0;
}

int isp_int_isp_irq_cnt_trace(int ctx_id)
{
	int i;

	if (ctx_id >= ISP_CONTEXT_HW_NUM)
		return 0;

	for (i = 0; i < 32; i++)
		if(irq_done[ctx_id][i])
			pr_info("done %d %d :   %d\n", ctx_id, i, irq_done[ctx_id][i]);

#ifdef ISP_INT_RECORD
	{
		uint32_t cnt, j;
		int idx = ctx_id;
		for (cnt = 0; cnt < (uint32_t)irq_done[idx][ISP_INT_SHADOW_DONE]; cnt += 4) {
			j = (cnt & (INT_RCD_SIZE - 1)); //rolling
			pr_info("isp%u j=%d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d\n",
			idx, j, (uint32_t)isp_int_recorder[idx][ISP_INT_ISP_ALL_DONE][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_ISP_ALL_DONE][j] & 0xffff,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_SHADOW_DONE][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_SHADOW_DONE][j] & 0xffff,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_DISPATCH_DONE][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_DISPATCH_DONE][j] & 0xffff,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_STORE_DONE_PRE][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_STORE_DONE_PRE][j] & 0xffff,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_STORE_DONE_VID][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_STORE_DONE_VID][j] & 0xffff,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_FMCU_CONFIG_DONE][j] >> 16,
			 (uint32_t)isp_int_recorder[idx][ISP_INT_FMCU_CONFIG_DONE][j] & 0xffff);
		}
	}
#endif
	return 0;
}

int isp_int_interruption_proc(void *node)
{
	int ret = 0;
	enum isp_context_hw_id pctx_hw_id;
	struct isp_node *inode = NULL;
	struct isp_pipe_dev *isp_handle = NULL;
	struct camera_interrupt *interruption = NULL;
	struct ispint_isr_root com = {0, 0};

	inode = VOID_PTR_TO(node, struct isp_node);
	if (atomic_read(&inode->user_cnt) < 1) {
		pr_err("fail to init isp node\n");
		return EFAULT;
	}
	pctx_hw_id = inode->pctx_hw_id;
	isp_handle = inode->dev;
	interruption = cam_queue_dequeue(&inode->isp_interrupt_queue, struct camera_interrupt, list);
	 if (interruption) {
		com.irq_line = interruption->int_status;
		com.irq_line1 = interruption->int_status1;
		cam_queue_empty_interrupt_put(interruption);
		/*isp interrupt unlikely*/
		ret = ispint_isr_root_unlikely(pctx_hw_id, com, inode->ctxs_com, isp_handle);
		if(unlikely(ret != 0))
			return EFAULT;
		/*isp interrupt handle*/
		ispint_isr_root_handle(pctx_hw_id, &com, inode->ctxs_com, isp_handle);
	} else
		pr_err("fail to dequeue from isp_interrupt_queue cnt %d max %d",
			inode->isp_interrupt_queue.cnt, inode->isp_interrupt_queue.max);

	return 0;
}

int isp_int_irq_request(struct device *p_dev,
		uint32_t *irq_no, void *isp_handle)
{
	int ret = 0;
	uint32_t  id;
	struct isp_pipe_dev *ispdev = NULL;

	if (!p_dev || !isp_handle || !irq_no) {
		pr_err("fail to get valid input ptr p_dev %p isp_handle %p irq_no %p\n",
			p_dev, isp_handle, irq_no);
		return -EFAULT;
	}
	ispdev = (struct isp_pipe_dev *)isp_handle;

	for (id = 0; id < ISP_LOGICAL_COUNT; id++) {

		if (ispdev->isp_hw->prj_id == QOGIRN6pro) {
			if (id == (ISP_LOGICAL_COUNT - 1))
				break;
		}
		ispdev->irq_no[id] = irq_no[id];
		ret = devm_request_irq(p_dev,
				ispdev->irq_no[id], ispint_isr_root,
				IRQF_SHARED, isp_dev_name[id], (void *)ispdev);
		if (ret) {
			pr_err("fail to install isp%d irq_no %d\n",
					id, ispdev->irq_no[id]);
			if (id == 1)
				free_irq(ispdev->irq_no[0], (void *)ispdev);
			return -EFAULT;
		}
		pr_info("install isp%d irq_no %d\n", id, ispdev->irq_no[id]);
	}

	memset(irq_done, 0, sizeof(irq_done));
	memset(irq_done_sw, 0, sizeof(irq_done_sw));

	return ret;
}

int isp_int_isp_irq_sw_cnt_reset(int ctx_id)
{
	if (ctx_id < ISP_CONTEXT_SW_NUM)
		memset(irq_done_sw[ctx_id], 0, sizeof(irq_done_sw[ctx_id]));

	return 0;
}

int isp_int_isp_irq_sw_cnt_trace(int ctx_id)
{
	int i;

	if (ctx_id >= ISP_CONTEXT_SW_NUM)
		return 0;

	for (i = 0; i < 32; i++)
		if(irq_done_sw[ctx_id][i])
			pr_info("done %d %d :   %d\n", ctx_id, i, irq_done_sw[ctx_id][i]);
	return 0;
}

int isp_int_irq_free(struct device *p_dev, void *isp_handle)
{
	struct isp_pipe_dev *ispdev = NULL;

	ispdev = (struct isp_pipe_dev *)isp_handle;
	if (!ispdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	devm_free_irq(p_dev, ispdev->irq_no[0], (void *)ispdev);
	devm_free_irq(p_dev, ispdev->irq_no[1], (void *)ispdev);

	return 0;
}

int isp_int_yuv_scaler_interruption_proc(void *node)
{
	int ret = 0;
	enum isp_context_hw_id pctx_hw_id;
	struct isp_yuv_scaler_node *inode = NULL;
	struct isp_pipe_dev *isp_handle = NULL;
	struct camera_interrupt *interruption = NULL;
	struct ispint_isr_root com = {0, 0};

	inode = VOID_PTR_TO(node, struct isp_yuv_scaler_node);
	if (atomic_read(&inode->user_cnt) < 1) {
		pr_err("fail to init isp node\n");
		return EFAULT;
	}
	pctx_hw_id = inode->pctx_hw_id;
	isp_handle = inode->dev;
	interruption = cam_queue_dequeue(&inode->yuv_scaler_interrupt_queue, struct camera_interrupt, list);
	 if (interruption) {
		com.irq_line = interruption->int_status;
		com.irq_line1 = interruption->int_status1;
		cam_queue_empty_interrupt_put(interruption);
		/* isp interrupt unlikely */
		ret = ispint_isr_root_unlikely(pctx_hw_id, com, inode->ctxs_com, isp_handle);
		if(unlikely(ret != 0))
			return EFAULT;
		/* isp interrupt handle */
		ispint_isr_root_handle(pctx_hw_id, &com, inode->ctxs_com, isp_handle);
	} else
		pr_err("fail to dequeue from isp_interrupt_queue cnt %d max %d\n",
			inode->yuv_scaler_interrupt_queue.cnt, inode->yuv_scaler_interrupt_queue.max);

	return 0;
}

