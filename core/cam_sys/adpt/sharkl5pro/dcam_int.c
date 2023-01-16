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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <sprd_mm.h>

#include "dcam_core.h"
#include "dcam_reg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_INT: %d %d %s : " fmt, current->pid, __LINE__, __func__

/* #define DCAM_INT_RECORD 1 */
#ifdef DCAM_INT_RECORD
#define INT_RCD_SIZE 0x10000
static uint32_t dcam_int_recorder[DCAM_HW_CONTEXT_MAX][DCAM_IRQ_NUMBER][INT_RCD_SIZE];
static uint32_t int_index[DCAM_HW_CONTEXT_MAX][DCAM_IRQ_NUMBER];
#endif

static uint32_t dcam_int_tracker[DCAM_HW_CONTEXT_MAX][DCAM_IRQ_NUMBER];
static char *dcam_dev_name[] = {"DCAM0",
				"DCAM1",
				"DCAM2"
				};

static const char *_DCAM_ADDR_NAME[DCAM_RECORD_PORT_INFO_MAX] = {
	[DCAM_RECORD_PORT_FULL] = "full",
	[DCAM_RECORD_PORT_BIN] = "bin",
	[DCAM_RECORD_PORT_BIN_1] = "bin1",
	[DCAM_RECORD_PORT_BIN_2] = "bin2",
	[DCAM_RECORD_PORT_BIN_3] = "bin3",
	[DCAM_RECORD_PORT_PDAF] = "pdaf",
	[DCAM_RECORD_PORT_VCH2] = "vch2",
	[DCAM_RECORD_PORT_VCH3] = "vch3",
	[DCAM_RECORD_PORT_LENS] = "lens",
	[DCAM_RECORD_PORT_LSCM] = "lscm",
	[DCAM_RECORD_PORT_AEM] = "aem",
	[DCAM_RECORD_PORT_HIST] = "hist",
	[DCAM_RECORD_PORT_HIST_1] = "hist1",
	[DCAM_RECORD_PORT_HIST_2] = "hist2",
	[DCAM_RECORD_PORT_HIST_3] = "hist3",
	[DCAM_RECORD_PORT_PPE] = "ppe",
	[DCAM_RECORD_PORT_AFL_GLB] = "afl_glb",
	[DCAM_RECORD_PORT_AFL_REGION] = "afl_region",
	[DCAM_RECORD_PORT_BPC_MAP] = "bpc_map",
	[DCAM_RECORD_PORT_BPC_OUT] = "bpc_out",
	[DCAM_RECORD_PORT_AFM] = "afm",
	[DCAM_RECORD_PORT_NR3] = "nr3",
};

const char *dcamint_addr_name_get(uint32_t type)
{
	return (type < DCAM_RECORD_PORT_IMG_FETCH) ? _DCAM_ADDR_NAME[type] : "(null)";
}

static inline void dcamint_dcam_int_record(uint32_t idx, uint32_t status)
{
	uint32_t i = 0;

	for (i = 0; i < DCAM_IRQ_NUMBER; i++) {
		if (status & BIT(i))
			dcam_int_tracker[idx][i]++;
	}

#ifdef DCAM_INT_RECORD
	{
		uint32_t cnt = 0, time = 0, int_no = 0;
		timespec cur_ts = {0};

		ktime_get_ts(&cur_ts);
		time = (uint32_t)(cur_ts.tv_sec & 0xffff);
		time <<= 16;
		time |= (uint32_t)((cur_ts.tv_nsec / (NSEC_PER_USEC * 100)) & 0xffff);
		for (int_no = 0; int_no < DCAM_IRQ_NUMBER; int_no++) {
			if (status & BIT(int_no)) {
				cnt = int_index[idx][int_no];
				dcam_int_recorder[idx][int_no][cnt] = time;
				cnt++;
				int_index[idx][int_no] = (cnt & (INT_RCD_SIZE - 1));
			}
		}
	}
#endif
}

static void dcamint_irq_cap_sof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (!param || dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_START_OF_FRAME;
	irq_proc.frm_cnt = DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_CAP_FRM_CLR) & 0xFF;
	irq_proc.bin_addr_value = DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_BIN_BASE_WADDR0);
	irq_proc.full_addr_value = DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_FULL_BASE_WADDR);
	irq_proc.handled_bits = DCAMINT_ALL_TX_DONE;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_irq_sensor_sof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;

	pr_debug("DCAM%d, dcamint_sensor_sof\n", dcam_hw_ctx->hw_ctx_id);
}

static void dcamint_full_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	pr_debug("dcamint_full_path_done\n");
	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_FULL_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_bin_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	pr_debug("bin_path_done hw id:%d\n", dcam_hw_ctx->hw_ctx_id);

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_BIN_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_vch2_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	pr_debug("dcamint_vch2_path_done hw_ctx_id = %d\n", dcam_hw_ctx->hw_ctx_id);
	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_VCH2_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_vch3_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_VCH3_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

struct nr3_done dcamint_nr3_done_rd(uint32_t idx)
{
	struct nr3_done com = {0};

	com.p = DCAM_REG_RD(idx, NR3_FAST_ME_PARAM);
	com.out0 = DCAM_REG_RD(idx, NR3_FAST_ME_OUT0);
	com.out1 = DCAM_REG_RD(idx, NR3_FAST_ME_OUT1);

	return com;
}

static void dcamint_gtm_hist_value_read(struct dcam_hw_context *hw_ctx)
{
	uint32_t i = 0;
	uint32_t hw_idx = 0;
	uint32_t sum = 0;
	unsigned long flag = 0;
	if (!hw_ctx) {
		pr_err("fail to get hw ctx\n");
		return;
	}
	spin_lock_irqsave(&hw_ctx->ghist_read_lock, flag);
	hw_idx = hw_ctx->hw_ctx_id;
	for (i = 0; i < GTM_HIST_ITEM_NUM; i++) {
		hw_ctx->gtm_hist_value[i] = DCAM_REG_RD(hw_idx, i * 4 + GTM_HIST_CNT);
		sum += hw_ctx->gtm_hist_value[i];
	}
	hw_ctx->gtm_hist_value[i] = sum;
	/* GTM_HIST_ITEM_NUM + 1:fid */
	spin_unlock_irqrestore(&hw_ctx->ghist_read_lock, flag);
}

static void dcamint_gtm_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_GTM_HIST_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_irq_preview_sof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	if (dcam_hw_ctx->hw_ctx_id == DCAM_ID_1 && !dcam_hw_ctx->is_virtualsensor_proc)
		return;
	irq_proc.of = PREV_START_OF_FRAME;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_irq_sensor_eof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = SN_END_OF_FRAME;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_lscm_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_LSCM_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_aem_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_AEM_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_pdaf_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_PDAF_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_afm_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_AFM_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_afl_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_AFL_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_hist_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to dcamonline callback fun is NULL\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_BAYER_HIST_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_nr3_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	uint32_t i = 0, fid = 0;
	struct dcam_irq_proc irq_proc = {0};
	struct nr3_done nr3_done_com;

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL || dcam_hw_ctx->hw_ctx_id >= DCAM_HW_CONTEXT_MAX) {
		pr_err("fail to dcamonline callback fun is NULL, hw_ctx_id %d\n", dcam_hw_ctx->hw_ctx_id);
		return;
	}

	nr3_done_com = dcamint_nr3_done_rd(dcam_hw_ctx->hw_ctx_id);
	if(nr3_done_com.hw_ctx == 1)
		return;

	fid = dcam_hw_ctx->fid - 1;
	i = fid % DCAM_NR3_MV_MAX;
	dcam_hw_ctx->nr3_mv_ctrl[i].sub_me_bypass = (nr3_done_com.p >> 3) & 0x1;
	dcam_hw_ctx->nr3_mv_ctrl[i].project_mode = (nr3_done_com.p >> 4) & 0x3;
	/* currently ping-pong is disabled, mv will always be stored in ping */
	dcam_hw_ctx->nr3_mv_ctrl[i].mv_x = (nr3_done_com.out0 >> 8) & 0xff;
	dcam_hw_ctx->nr3_mv_ctrl[i].mv_y = nr3_done_com.out0 & 0xff;
	dcam_hw_ctx->nr3_mv_ctrl[i].src_width = dcam_hw_ctx->cap_info.cap_size.size_x;
	dcam_hw_ctx->nr3_mv_ctrl[i].src_height = dcam_hw_ctx->cap_info.cap_size.size_y;
	dcam_hw_ctx->nr3_mv_ctrl[i].valid = 1;
	pr_debug("dcam%d fid %d, mv_x %d, mv_y %d\n", dcam_hw_ctx->hw_ctx_id, fid,
		dcam_hw_ctx->nr3_mv_ctrl[i].mv_x, dcam_hw_ctx->nr3_mv_ctrl[i].mv_y);

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.is_nr3_done = 1;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

typedef void (*dcam_isr)(void *param);
static const dcam_isr _DCAM_ISR_IRQ[] = {
	[DCAM_SENSOR_SOF] = dcamint_irq_sensor_sof,
	[DCAM_SENSOR_EOF] = dcamint_irq_sensor_eof,
	[DCAM_CAP_SOF] = dcamint_irq_cap_sof,
	[DCAM_PREVIEW_SOF] = dcamint_irq_preview_sof,
	[DCAM_FULL_PATH_TX_DONE] = dcamint_full_port_done,
	[DCAM_PREV_PATH_TX_DONE] = dcamint_bin_port_done,
	[DCAM_PDAF_PATH_TX_DONE] = dcamint_pdaf_port_done,
	[DCAM_VCH2_PATH_TX_DONE] = dcamint_vch2_port_done,
	[DCAM_VCH3_PATH_TX_DONE] = dcamint_vch3_port_done,
	[DCAM_AEM_TX_DONE] = dcamint_aem_port_done,
	[DCAM_HIST_TX_DONE] = dcamint_hist_port_done,
	[DCAM_AFL_TX_DONE] = dcamint_afl_port_done,
	[DCAM_AFM_INTREQ1] = dcamint_afm_port_done,
	[DCAM_NR3_TX_DONE] = dcamint_nr3_port_done,
	[DCAM_LSCM_TX_DONE] = dcamint_lscm_port_done,
	[DCAM_GTM_TX_DONE] = dcamint_gtm_port_done,
};

static const int _DCAM0_SEQUENCE[] = {
	DCAM_CAP_SOF,
	DCAM_SENSOR_SOF,
	DCAM_PREVIEW_SOF,
	DCAM_SENSOR_EOF,
	DCAM_NR3_TX_DONE,
	DCAM_PREV_PATH_TX_DONE,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_AEM_TX_DONE,
	DCAM_HIST_TX_DONE,
	/* AFM INT0 is some tile done, INT1 is all tile done,
	 * INT0 can use to triger AF ALG eaglier, but the specific time
	 * is not sure, so INT1 is suggested to use now.
	 */
	DCAM_AFM_INTREQ1,
	DCAM_AFL_TX_DONE,
	DCAM_PDAF_PATH_TX_DONE,
	DCAM_VCH2_PATH_TX_DONE,
	DCAM_VCH3_PATH_TX_DONE,
	DCAM_LSCM_TX_DONE,
	DCAM_GTM_TX_DONE,
};

static const int _DCAM1_SEQUENCE[] = {
	DCAM_CAP_SOF,
	DCAM_SENSOR_SOF,
	DCAM_PREVIEW_SOF,
	DCAM_SENSOR_EOF,
	DCAM_NR3_TX_DONE,
	DCAM_PREV_PATH_TX_DONE,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_AEM_TX_DONE,
	DCAM_HIST_TX_DONE,
	/* AFM INT0 is some tile done, INT1 is all tile done,
	 * INT0 can use to triger AF ALG eaglier, but the specific time
	 * is not sure, so INT1 is suggested to use now.
	 */
	DCAM_AFM_INTREQ1,
	DCAM_AFL_TX_DONE,
	DCAM_PDAF_PATH_TX_DONE,
	DCAM_VCH2_PATH_TX_DONE,
	DCAM_VCH3_PATH_TX_DONE,
	DCAM_LSCM_TX_DONE,
	DCAM_GTM_TX_DONE,
};

static const int _DCAM2_SEQUENCE[] = {
	DCAM_CAP_SOF,
	DCAM_SENSOR_SOF,
	DCAM_PREVIEW_SOF,
	DCAM_SENSOR_EOF,
	DCAM_NR3_TX_DONE,
	DCAM_PREV_PATH_TX_DONE,
	DCAM_FULL_PATH_TX_DONE,
	DCAM_AEM_TX_DONE,
	DCAM_HIST_TX_DONE,
	/* AFM INT0 is some tile done, INT1 is all tile done,
	 * INT0 can use to triger AF ALG eaglier, but the specific time
	 * is not sure, so INT1 is suggested to use now.
	 */
	DCAM_AFM_INTREQ1,
	DCAM_AFL_TX_DONE,
	DCAM_PDAF_PATH_TX_DONE,
	DCAM_VCH2_PATH_TX_DONE,
	DCAM_VCH3_PATH_TX_DONE,
	DCAM_LSCM_TX_DONE,
	DCAM_GTM_TX_DONE,
};

static const struct {
	size_t count;
	const int *bits;
} DCAM_SEQUENCES[DCAM_HW_CONTEXT_MAX] = {
	{
		ARRAY_SIZE(_DCAM0_SEQUENCE),
		_DCAM0_SEQUENCE,
	},
	{
		ARRAY_SIZE(_DCAM1_SEQUENCE),
		_DCAM1_SEQUENCE,
	},
	{
		ARRAY_SIZE(_DCAM2_SEQUENCE),
		_DCAM2_SEQUENCE,
	},
};

static void dcamint_iommu_regs_dump(struct dcam_hw_context *dcam_hw_ctx)
{
	uint32_t i = 0, j = 0;

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid input hw_ctx\n");
		return;
	}

	if (dcam_hw_ctx->err_count) {
		if (dcam_hw_ctx->is_offline_proc)
			pr_err("fail to handle offline dcam frame_cnt=%d, MMU_INT_STS: %08x, MMU_INT_RAW: %08x\n",
				dcam_hw_ctx->frame_addr[0][DCAM_RECORD_FRAME_CNT_SUM],
				DCAM_MMU_RD(DCAM_MMU_INT_STS), DCAM_MMU_RD(DCAM_MMU_INT_RAW));
		else
			pr_err("fail to handle frame_cnt=%d, MMU_INT_STS: %08x, MMU_INT_RAW: %08x\n",
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_CAP_FRM_CLR) & 0xFF,
				DCAM_MMU_RD(DCAM_MMU_INT_STS), DCAM_MMU_RD(DCAM_MMU_INT_RAW));

		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_0)
			pr_err("fail to handle, MMU_OR_ADDR_RD: %08x\n", DCAM_MMU_RD(DCAM_MMU_OR_ADDR_RD));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_1)
			pr_err("fail to handle, MMU_OR_ADDR_WR: %08x\n", DCAM_MMU_RD(DCAM_MMU_OR_ADDR_WR));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_2)
			pr_err("fail to handle, MMU_INV_ADDR_RD: %08x\n", DCAM_MMU_RD(DCAM_MMU_INV_ADDR_RD));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_3)
			pr_err("fail to handle, MMU_INV_ADDR_WR: %08x\n", DCAM_MMU_RD(DCAM_MMU_INV_ADDR_WR));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_4)
			pr_err("fail to handle, MMU_UNS_ADDR_RD: %08x\n", DCAM_MMU_RD(DCAM_MMU_UNS_ADDR_RD));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_5)
			pr_err("fail to handle, MMU_UNS_ADDR_WR: %08x\n", DCAM_MMU_RD(DCAM_MMU_UNS_ADDR_WR));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_6)
			pr_err("fail to handle, MMU_VPN_PAOR_RD: %08x, MMU_PPN_PAOR_RD: %08x\n",
				DCAM_MMU_RD(DCAM_MMU_VPN_PAOR_RD), DCAM_MMU_RD(DCAM_MMU_PPN_PAOR_RD));
		if (DCAM_MMU_RD(DCAM_MMU_INT_RAW) & BIT_7)
			pr_err("fail to handle, MMU_VPN_PAOR_WR: %08x, MMU_PPN_PAOR_WR: %08x\n",
				DCAM_MMU_RD(DCAM_MMU_VPN_PAOR_WR), DCAM_MMU_RD(DCAM_MMU_PPN_PAOR_WR));


		for (i = 0; i < DCAM_ADDR_RECORD_FRAME_NUM; i++) {
			for (j = DCAM_RECORD_PORT_FULL; j < DCAM_RECORD_PORT_BPC_OUT; j += 5) {
				pr_err("fail to handle used frame_cnt%d, %s: %08x, %s: %08x, %s: %08x,"
					" %s: %08x, %s: %08x\n", dcam_hw_ctx->frame_addr[i][DCAM_RECORD_FRAME_CUR_COUNT],
					dcamint_addr_name_get(j), dcam_hw_ctx->frame_addr[i][j],
					dcamint_addr_name_get(j + 1), dcam_hw_ctx->frame_addr[i][j + 1],
					dcamint_addr_name_get(j + 2), dcam_hw_ctx->frame_addr[i][j + 2],
					dcamint_addr_name_get(j + 3), dcam_hw_ctx->frame_addr[i][j + 3],
					dcamint_addr_name_get(j + 4), dcam_hw_ctx->frame_addr[i][j + 4]);
			}
			pr_err("fail to handle used frame_cnt%d, %s: %08x, %s: %08x\n",
				dcam_hw_ctx->frame_addr[i][DCAM_RECORD_FRAME_CUR_COUNT],
				dcamint_addr_name_get(j), dcam_hw_ctx->frame_addr[i][j],
				dcamint_addr_name_get(j + 1), dcam_hw_ctx->frame_addr[i][j + 1]);
			if (dcam_hw_ctx->is_offline_proc)
				pr_err("fail to handle used frame_cnt%d, offline fetch %08x\n", dcam_hw_ctx->frame_addr[i][DCAM_RECORD_PORT_IMG_FETCH]);
		}
		dcam_hw_ctx->err_count -= 1;
	}
}

static irqreturn_t dcamint_error_handler_param(struct dcam_hw_context *dcam_hw_ctx, uint32_t status)
{
	struct dcam_irq_proc irq_proc = {0};
	enum dcam_state state = STATE_INIT;

	const char *tb_ovr[2] = {"", ", overflow"};
	const char *tb_lne[2] = {"", ", line error"};
	const char *tb_frm[2] = {"", ", frame error"};
	const char *tb_mmu[2] = {"", ", mmu"};

	if (!dcam_hw_ctx->dcam_irq_cb_handle) {
		pr_err("fail to get valid dcam_online_node\n");
		return IRQ_HANDLED;
	}

	pr_err("fail to get normal status DCAM%u 0x%x%s%s%s%s\n", dcam_hw_ctx->hw_ctx_id, status,
		tb_ovr[!!(status & BIT(DCAM_DCAM_OVF))],
		tb_lne[!!(status & BIT(DCAM_CAP_LINE_ERR))],
		tb_frm[!!(status & BIT(DCAM_CAP_FRM_ERR))],
		tb_mmu[!!(status & BIT(DCAM_MMU_INT))]);

	if (status & BIT(DCAM_MMU_INT)) {
		uint32_t val = DCAM_MMU_RD(DCAM_MMU_INT_STS);

		if (val != dcam_hw_ctx->iommu_status) {
			dcamint_iommu_regs_dump(dcam_hw_ctx);
			dcam_hw_ctx->iommu_status = val;
		}
	}

	if (dcam_hw_ctx->is_offline_proc) {
		pr_err("fail to offline int error,status:%x \n", status);
		return IRQ_HANDLED;
	}

	if (dcam_hw_ctx->is_virtualsensor_proc)
		state = dcam_fetch_node_state_get(dcam_hw_ctx->dcam_irq_cb_handle);
	else
		state = dcam_online_node_state_get(dcam_hw_ctx->dcam_irq_cb_handle);

	if (dcam_hw_ctx->dcam_irq_cb_func) {
		if ((status & DCAMINT_FATAL_ERROR) && (state != STATE_ERROR)) {
			irq_proc.of = DCAM_INT_ERROR;
			irq_proc.status = status;
			dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
		}
	}

	return IRQ_HANDLED;
}

irqreturn_t dcamint_error_handler(void *dcam_hw_ctx, uint32_t status)
{
	dcamint_error_handler_param(dcam_hw_ctx, status);
	return IRQ_HANDLED;
}

struct dcam_irq_info dcamint_dcam_int_mask_clr(uint32_t idx)
{
	struct dcam_irq_info irq_info = {0};

	pr_err("fail to check status 0x%x\n", irq_info.status);
	return irq_info;
}

void dcamint_dcam_int_clr(uint32_t idx)
{
	pr_warn_ratelimited("warning: DCAM%u ignore irq in NON-running, 0x%x\n",
		idx, DCAM_REG_RD(idx, DCAM_INT_MASK));
	DCAM_REG_WR(idx, DCAM_INT_CLR, 0xFFFFFFFF);
}

struct dcam_irq_info dcamint_isr_root_done_rd(uint32_t idx)
{
	struct dcam_irq_info irq_info = {0};

	irq_info.status = DCAM_REG_RD(idx, DCAM_INT_MASK);
	irq_info.status &= DCAMINT_IRQ_LINE_MASK;
	return irq_info;
}

void dcamint_isr_root_writeint(uint32_t idx, struct dcam_irq_info *irq_info)
{
	irq_info->irq_num = DCAM_IRQ_NUMBER;
	DCAM_REG_WR(idx, DCAM_INT_CLR, irq_info->status);
	dcamint_dcam_int_record(idx, irq_info->status);
}

void dcamint_dcam_status_rw(struct dcam_irq_info irq_info, void *priv)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)priv;
	unsigned int i = 0;

	for (i = 0; i < DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id].count; i++) {
		int cur_int = DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id].bits[i];
		if (irq_info.status & BIT(cur_int)) {
			if (_DCAM_ISR_IRQ[cur_int]) {
				_DCAM_ISR_IRQ[cur_int](dcam_hw_ctx);
				irq_info.status &= ~dcam_hw_ctx->handled_bits;
				dcam_hw_ctx->handled_bits = 0;
			} else {
				pr_warn("warning: DCAM%u missing handler for int %d\n",
					dcam_hw_ctx->hw_ctx_id, cur_int);
			}
			irq_info.status &= ~BIT(cur_int);
			if (!irq_info.status)
				break;
		}
	}

	if (unlikely(irq_info.status))
		pr_warn("warning: DCAM%u unhandled int 0x%x\n", dcam_hw_ctx->hw_ctx_id, irq_info.status);

}

irqreturn_t dcamint_isr_root(int irq, void *priv)
{
	struct dcam_hw_context *dcam_hw_ctx = NULL;
	struct dcam_irq_info irq_status = {0};
	int ret = 0;

	if (unlikely(!priv)) {
		pr_err("fail to get valid priv\n");
		return IRQ_HANDLED;
	}

	dcam_hw_ctx = (struct dcam_hw_context *)priv;
	if (unlikely(irq != dcam_hw_ctx->irq)) {
		pr_err("fail to match DCAM%u irq %d %d\n", dcam_hw_ctx->hw_ctx_id, irq, dcam_hw_ctx->irq);
		return IRQ_NONE;
	}
	if (!read_trylock(&dcam_hw_ctx->hw->soc_dcam->cam_ahb_lock))
		return IRQ_HANDLED;
	dcam_hw_ctx->in_irq_proc = 1;
	if (!dcam_hw_ctx->dcam_irq_cb_handle) {
		irq_status = dcamint_dcam_int_mask_clr(dcam_hw_ctx->hw_ctx_id);
		ret = IRQ_NONE;
		goto exit;
	}

	/* read int_mask*/
	irq_status = dcamint_isr_root_done_rd(dcam_hw_ctx->hw_ctx_id);
	if (unlikely(!irq_status.status && !irq_status.status1)) {
		ret = IRQ_NONE;
		goto exit;
	}
	/* write int_mask*/
	dcamint_isr_root_writeint(dcam_hw_ctx->hw_ctx_id, &irq_status);

	if (unlikely(DCAMINT_ALL_ERROR & irq_status.status)) {
		dcamint_error_handler(dcam_hw_ctx, irq_status.status);
		irq_status.status &= (~DCAMINT_ALL_ERROR);
	}

	if (!dcam_hw_ctx->slowmotion_count) {
		if (irq_status.status & BIT(DCAM_GTM_TX_DONE))
			dcamint_gtm_hist_value_read(dcam_hw_ctx);
	}
	if (dcam_hw_ctx->is_offline_proc) {
		dcam_core_offline_irq_proc(dcam_hw_ctx, &irq_status);
	} else {
		pr_debug("DCAM%u status=0x%x\n", dcam_hw_ctx->hw_ctx_id, irq_status.status);
		/*dcam isr root write status */
		dcamint_dcam_status_rw(irq_status, priv);
	}
	ret = IRQ_HANDLED;
exit:
	read_unlock(&dcam_hw_ctx->hw->soc_dcam->cam_ahb_lock);
	dcam_hw_ctx->in_irq_proc = 0;
	return ret;
}

int dcam_int_irq_request(struct device *pdev, int irq, void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = NULL;
	int ret = 0;

	if (unlikely(!pdev || !param || irq < 0)) {
		pr_err("fail to get valid param %p %p %d\n", pdev, param, irq);
		return -EINVAL;
	}

	dcam_hw_ctx = (struct dcam_hw_context *)param;
	dcam_hw_ctx->irq = irq;

	ret = devm_request_irq(pdev, dcam_hw_ctx->irq, dcamint_isr_root,
			IRQF_SHARED, dcam_dev_name[dcam_hw_ctx->hw_ctx_id], dcam_hw_ctx);
	if (ret < 0) {
		pr_err("DCAM%u fail to install irq %d\n", dcam_hw_ctx->hw_ctx_id, dcam_hw_ctx->irq);
		return -EFAULT;
	}
	dcam_hw_ctx->irq_enable = 1;
	dcam_int_tracker_reset(dcam_hw_ctx->hw_ctx_id);
	return ret;
}

void dcam_int_irq_free(struct device *pdev, void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = NULL;

	if (unlikely(!pdev || !param)) {
		pr_err("fail to get valid param %p %p\n", pdev, param);
		return;
	}

	dcam_hw_ctx = (struct dcam_hw_context *)param;
	devm_free_irq(pdev, dcam_hw_ctx->irq, dcam_hw_ctx);
}

int dcam_int_irq_desc_get(uint32_t index, void *param)
{
	int ret = 0;
	struct dcam_irq_proc_desc *irq_desc = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	irq_desc = (struct dcam_irq_proc_desc *)param;
	switch (index) {
	case DCAM_FULL_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_DATA_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_FULL_OUT;
		break;
	case DCAM_PREV_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_DATA_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_BIN_OUT;
		break;
	case DCAM_AEM_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_AEM_OUT;
		break;
	case DCAM_HIST_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_BAYER_HIST_OUT;
		break;
	case DCAM_LSCM_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_LSCM_OUT;
		break;
	default :
		pr_warn("Warning: not to support irq index BIT%d\n", index);
		ret = -1;
		break;
	}

	pr_debug("irq dcam port id %d\n", irq_desc->dcam_port_id);
	return ret;
}

void dcam_int_tracker_reset(uint32_t idx)
{
	if (is_dcam_id(idx))
		memset(dcam_int_tracker[idx], 0, sizeof(dcam_int_tracker[idx]));

#ifdef DCAM_INT_RECORD
	if (is_dcam_id(idx)) {
		memset(dcam_int_recorder[idx][0], 0, sizeof(dcam_int_recorder) / 3);
		memset(int_index[idx], 0, sizeof(int_index) / 3);
	}
#endif
}

void dcam_int_tracker_dump(uint32_t idx)
{
	int i = 0;

	if (!is_dcam_id(idx))
		return;

	for (i = 0; i < DCAM_IRQ_NUMBER; i++) {
		if (dcam_int_tracker[idx][i])
			pr_info("DCAM%u i=%d, int=%u\n", idx, i,
				 dcam_int_tracker[idx][i]);
	}

#ifdef DCAM_INT_RECORD
	{
		uint32_t cnt = 0, j = 0;
		for (cnt = 0; cnt < (uint32_t)dcam_int_tracker[idx][DCAM_SENSOR_EOF]; cnt += 4) {
			j = (cnt & (INT_RCD_SIZE - 1));
			pr_info("DCAM%u j=%d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d\n",
				idx, j, (uint32_t)dcam_int_recorder[idx][DCAM_SENSOR_EOF][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_SENSOR_EOF][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_CAP_SOF][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_CAP_SOF][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_FULL_PATH_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_FULL_PATH_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_PREV_PATH_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_PREV_PATH_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_AEM_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_AEM_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_AFM_INTREQ1][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_AFM_INTREQ1][j] & 0xffff);
				(uint32_t)dcam_int_recorder[idx][DCAM_LSCM_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_LSCM_TX_DONE][j] & 0xffff,
		}
	}
#endif
}

