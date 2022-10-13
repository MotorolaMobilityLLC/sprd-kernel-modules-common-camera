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
#include "isp_hw.h"
#include "sprd_img.h"
#include <sprd_mm.h>

#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_core.h"
#include "cam_queue.h"
#include "cam_types.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_INT: %d %d %s : " fmt, current->pid, __LINE__, __func__

/* #define DCAM_INT_RECORD 1 */
#ifdef DCAM_INT_RECORD
#define INT_RCD_SIZE 0x10000
static uint32_t dcam_int_recorder[DCAM_HW_CONTEXT_MAX][DCAM_IF_IRQ_INT0_NUMBER][INT_RCD_SIZE];
static uint32_t int_index[DCAM_HW_CONTEXT_MAX][DCAM_IF_IRQ_INT0_NUMBER];
#endif

static uint32_t dcam_int0_tracker[DCAM_HW_CONTEXT_MAX][DCAM_IF_IRQ_INT0_NUMBER] = {0};
static uint32_t dcam_int1_tracker[DCAM_HW_CONTEXT_MAX][DCAM_IF_IRQ_INT1_NUMBER] = {0};
static char *dcam_dev_name[] = {"DCAM0",
				"DCAM1",
				"DCAM2"
				};

static void dcamint_iommu_regs_dump(struct dcam_hw_context *dcam_hw_ctx);

static inline void dcamint_dcam_int_record(uint32_t idx, uint32_t status, uint32_t status1)
{
	uint32_t i = 0;

	for (i = 0; i < DCAM_IF_IRQ_INT0_NUMBER; i++) {
		if (status & BIT(i))
			dcam_int0_tracker[idx][i]++;
		if (status1 & BIT(i))
			dcam_int1_tracker[idx][i]++;
	}

#ifdef DCAM_INT_RECORD
	{
		uint32_t cnt = 0, time = 0, int_no = 0;
		timespec cur_ts = {0};

		ktime_get_ts(&cur_ts);
		time = (uint32_t)(cur_ts.tv_sec & 0xffff);
		time <<= 16;
		time |= (uint32_t)((cur_ts.tv_nsec / (NSEC_PER_USEC * 100)) & 0xffff);
		for (int_no = 0; int_no < DCAM_IF_IRQ_INT0_NUMBER; int_no++) {
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

static void dcamint_irq_cap_eof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	DCAM_AXIM_WR(dcam_hw_ctx->hw_ctx_id, AXIM_CNT_CLR, 1);
	pr_debug("get into dcamint_cap_eof success\n");
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
	irq_proc.bin_addr_value = DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE0_SLICE_Y_ADDR);
	irq_proc.full_addr_value = DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE4_SLICE_Y_ADDR);
	irq_proc.handled_bits = DCAMINT_INT0_TX_DONE;
	irq_proc.handled_bits_on_int1 = DCAMINT_INT1_TX_DONE;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_irq_sensor_sof(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;

	pr_debug("DCAM%d, dcamint_sensor_sof\n", dcam_hw_ctx->hw_ctx_id);

	if (dcam_hw_ctx->cap_info.cap_size.size_x > DCAM_TOTAL_LBUF) {
		if (dcam_hw_ctx->frame_index == 0)
			dcam_hw_ctx->frame_index++;
		else
			dcamint_irq_cap_sof(param);
	}
}

static void dcamint_raw_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid inptr %px\n", dcam_hw_ctx);
		return;
	}

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to get valid dcamonline callback fun.\n");
		return;
	}

	pr_debug("dcamint_raw_path_done\n");
	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_RAW_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_full_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid inptr %px\n", dcam_hw_ctx);
		return;
	}

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

	dcam_hw_ctx->dec_layer0_done = 1;
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
	irq_proc.dcam_port_id = PORT_PDAF_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

struct nr3_done dcamint_nr3_done_rd(uint32_t idx)
{
	struct nr3_done com = {0};

	com.p = DCAM_REG_RD(idx, DCAM_NR3_FAST_ME_PARAM);
	com.out0 = DCAM_REG_RD(idx, DCAM_NR3_FAST_ME_OUT0);
	com.out1 = DCAM_REG_RD(idx, DCAM_NR3_FAST_ME_OUT1);

	return com;
}

static void dcamint_frgb_hist_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx == NULL) {
		pr_err("fail to get valid dcam_hw_ctx.\n");
		return;
	}

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to get valid dcamonline callback fun.\n");
		return;
	}

	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_FRGB_HIST_OUT;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
}

static void dcamint_dec_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid inptr %px\n", dcam_hw_ctx);
		return;
	}

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to get valid dcamonline callback fun.\n");
		return;
	}

	pr_debug("dcamint_dec_done\n");
	dcam_hw_ctx->dec_all_done = 1;
	if (dcam_hw_ctx->dec_layer0_done) {
		irq_proc.of = CAP_DATA_DONE;
		irq_proc.dcam_port_id = PORT_BIN_OUT;
		dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
		dcam_hw_ctx->dec_all_done = 0;
		dcam_hw_ctx->dec_layer0_done = 0;
	}
}

static void dcamint_dummy_start(void *param)
{
	pr_debug("dcamint_dummy_start\n");
}

static void dcamint_fmcu_config_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	int i = 0;
	struct dcam_irq_proc irq_proc = {0};
	if (!dcam_hw_ctx) {
		pr_err("fail to get valid inptr\n", dcam_hw_ctx);
		return;
	}

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to get valid dcamonline callback fun.\n");
		return;
	}

	pr_debug("dcamint_fmcu_config_done\n");
	atomic_inc(&dcam_hw_ctx->shadow_config_cnt);

	while (i++ < dcam_hw_ctx->slowmotion_count) {
		irq_proc.of = CAP_DATA_DONE;
		irq_proc.dcam_port_id = PORT_BIN_OUT;
		dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
	}
	/* shadow done & config done happen in the same time
	* shadow done have smaller int num, sw will process (n+1)shadow done before (n)config done
	* int sequence : s sc sc sc, 1 more s than c
	* if lack one shadow done, slowmotion will stop after config done, re-config next cmd q in config done
	* int sequence: s sc sc sc  c (lack s, re-config cmd q) s sc sc sc... 2 more s than c
	*/
	if (atomic_read(&dcam_hw_ctx->shadow_config_cnt) == atomic_read(&dcam_hw_ctx->shadow_done_cnt)) {
		atomic_inc(&dcam_hw_ctx->shadow_done_cnt);
		atomic_inc(&dcam_hw_ctx->shadow_config_cnt);
		if (dcam_hw_ctx->fmcu) {
			dcam_hw_ctx->index_to_set = dcam_hw_ctx->index_to_set + dcam_hw_ctx->slowmotion_count;
			dcam_hw_ctx->fmcu->ops->ctx_reset(dcam_hw_ctx->fmcu);
			irq_proc.of = CAP_DATA_DONE;
			irq_proc.dcam_port_id = PORT_BIN_OUT;
			irq_proc.slw_cmds_set = 1;
			dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
			dcam_hw_ctx->fmcu->ops->cmd_ready(dcam_hw_ctx->fmcu);
		} else
			pr_err("fail to get fmcu handle\n");
	}
}

static void dcamint_fmcu_shadow_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (dcam_hw_ctx == NULL) {
		pr_err("fail to check param dcam_hw_ctx %px\n", dcam_hw_ctx);
		return;
	}

	if (dcam_hw_ctx->dcam_irq_cb_func == NULL) {
		pr_err("fail to get valid dcamonline callback fun.\n");
		return;
	}

	atomic_inc(&dcam_hw_ctx->shadow_done_cnt);

	dcam_hw_ctx->index_to_set = dcam_hw_ctx->index_to_set + dcam_hw_ctx->slowmotion_count;
	dcam_hw_ctx->fmcu->ops->ctx_reset(dcam_hw_ctx->fmcu);
	irq_proc.of = CAP_DATA_DONE;
	irq_proc.dcam_port_id = PORT_BIN_OUT;
	irq_proc.slw_cmds_set = 1;
	dcam_hw_ctx->dcam_irq_cb_func(&irq_proc, dcam_hw_ctx->dcam_irq_cb_handle);
	dcam_hw_ctx->fmcu->ops->cmd_ready(dcam_hw_ctx->fmcu);
}

static void dcamint_dummy_done(void *param)
{
	pr_debug("dcamint_dummy_done\n");
}

static void dcamint_sensor_sof1(void *param)
{
	pr_debug("dcamint_sensor_sof1\n");
}

static void dcamint_sensor_sof2(void *param)
{
	pr_debug("dcamint_sensor_sof2\n");
}

static void dcamint_sensor_sof3(void *param)
{
	pr_debug("dcamint_sensor_sof3\n");
}

static void dcamint_gtm_port_done(void *param)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)param;
	struct dcam_irq_proc irq_proc = {0};

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid inptr %px\n", dcam_hw_ctx);
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
	dcam_hw_ctx->nr3_mv_ctrl[i].sub_me_bypass = (nr3_done_com.p >> 8) & 0x1;
	dcam_hw_ctx->nr3_mv_ctrl[i].project_mode = (nr3_done_com.p >> 4) & 0x1;
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
	[DCAM_IF_IRQ_INT0_SENSOR_SOF] = dcamint_irq_sensor_sof,
	[DCAM_IF_IRQ_INT0_SENSOR_EOF] = dcamint_irq_sensor_eof,
	[DCAM_IF_IRQ_INT0_CAP_SOF] = dcamint_irq_cap_sof,
	[DCAM_IF_IRQ_INT0_CAP_EOF] = dcamint_irq_cap_eof,
	[DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE] = dcamint_raw_port_done,
	[DCAM_IF_IRQ_INT0_PREIEW_SOF] = dcamint_irq_preview_sof,
	[DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE] = dcamint_full_port_done,
	[DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE] = dcamint_bin_port_done,
	[DCAM_IF_IRQ_INT0_PDAF_PATH_TX_DONE] = dcamint_pdaf_port_done,
	[DCAM_IF_IRQ_INT0_VCH2_PATH_TX_DONE] = dcamint_vch2_port_done,
	[DCAM_IF_IRQ_INT0_VCH3_PATH_TX_DONE] = dcamint_vch3_port_done,
	[DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE] = dcamint_aem_port_done,
	[DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE] = dcamint_hist_port_done,
	[DCAM_IF_IRQ_INT0_AFL_TX_DONE] = dcamint_afl_port_done,
	[DCAM_IF_IRQ_INT0_AFM_INTREQ1] = dcamint_afm_port_done,
	[DCAM_IF_IRQ_INT0_NR3_TX_DONE] = dcamint_nr3_port_done,
	[DCAM_IF_IRQ_INT0_LSCM_TX_DONE] = dcamint_lscm_port_done,
	[DCAM_IF_IRQ_INT0_GTM_DONE] = dcamint_gtm_port_done,
};

static const dcam_isr _DCAM_ISRS1[] = {
	[DCAM_IF_IRQ_INT1_FRGB_HIST_DONE] = dcamint_frgb_hist_done,
	[DCAM_IF_IRQ_INT1_DEC_DONE] = dcamint_dec_done,
	[DCAM_IF_IRQ_INT1_SENSOR_SOF1] = dcamint_sensor_sof1,
	[DCAM_IF_IRQ_INT1_SENSOR_SOF2] = dcamint_sensor_sof2,
	[DCAM_IF_IRQ_INT1_SENSOR_SOF3] = dcamint_sensor_sof3,
	[DCAM_IF_IRQ_INT1_DUMMY_START] = dcamint_dummy_start,
	[DCAM_IF_IRQ_INT1_FMCU_INT1] = dcamint_fmcu_config_done,
	[DCAM_IF_IRQ_INT1_FMCU_INT2] = dcamint_fmcu_shadow_done,
	[DCAM_IF_IRQ_INT1_DUMMY_DONE] = dcamint_dummy_done,
};

static const int _DCAM0_SEQUENCE[] = {
	DCAM_IF_IRQ_INT0_SENSOR_SOF,
	DCAM_IF_IRQ_INT0_CAP_SOF,/* must */
	DCAM_IF_IRQ_INT0_CAP_EOF,
	DCAM_IF_IRQ_INT0_PREIEW_SOF,
	DCAM_IF_IRQ_INT0_SENSOR_EOF,/* TODO: why for flash */
	DCAM_IF_IRQ_INT0_NR3_TX_DONE,/* for 3dnr, before data path */
	DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE,/* for raw path */
	DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE,/* for bin path */
	DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE,/* for full path */
	DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE,/* for aem statis */
	DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE,/* for hist statis */
	/* for afm statis, not sure 0 or 1 */
	DCAM_IF_IRQ_INT0_AFM_INTREQ1,/* TODO: which afm interrupt to use */
	DCAM_IF_IRQ_INT0_AFL_TX_DONE,/* for afl statis */
	DCAM_IF_IRQ_INT0_PDAF_PATH_TX_DONE,/* for pdaf data */
	DCAM_IF_IRQ_INT0_VCH2_PATH_TX_DONE,/* for vch2 data */
	DCAM_IF_IRQ_INT0_VCH3_PATH_TX_DONE,/* for vch3 data */
	DCAM_IF_IRQ_INT0_LSCM_TX_DONE,/* for lscm statis */
	DCAM_IF_IRQ_INT0_GTM_DONE,
};

static const int _DCAM0_SEQUENCE_INT1[] = {
	DCAM_IF_IRQ_INT1_FRGB_HIST_DONE,
	DCAM_IF_IRQ_INT1_DEC_DONE,
	DCAM_IF_IRQ_INT1_SENSOR_SOF1,
	DCAM_IF_IRQ_INT1_SENSOR_SOF2,
	DCAM_IF_IRQ_INT1_SENSOR_SOF3,
	DCAM_IF_IRQ_INT1_DUMMY_START,
	DCAM_IF_IRQ_INT1_FMCU_INT2,
	DCAM_IF_IRQ_INT1_FMCU_INT1,
	DCAM_IF_IRQ_INT1_DUMMY_DONE,
};

static const int _DCAM1_SEQUENCE[] = {
	DCAM_IF_IRQ_INT0_SENSOR_SOF,
	DCAM_IF_IRQ_INT0_CAP_SOF,/* must */
	DCAM_IF_IRQ_INT0_CAP_EOF,
	DCAM_IF_IRQ_INT0_SENSOR_EOF,/* TODO: why for flash */
	DCAM_IF_IRQ_INT0_NR3_TX_DONE,/* for 3dnr, before data path */
	DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE,/* for raw path */
	DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE,/* for bin path */
	DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE,/* for full path */
	DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE,/* for aem statis */
	DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE,/* for hist statis */
	/* for afm statis, not sure 0 or 1 */
	DCAM_IF_IRQ_INT0_AFM_INTREQ1,/* TODO: which afm interrupt to use */
	DCAM_IF_IRQ_INT0_AFL_TX_DONE,/* for afl statis */
	DCAM_IF_IRQ_INT0_PDAF_PATH_TX_DONE,/* for pdaf data */
	DCAM_IF_IRQ_INT0_VCH2_PATH_TX_DONE,/* for vch2 data */
	DCAM_IF_IRQ_INT0_VCH3_PATH_TX_DONE,/* for vch3 data */
	DCAM_IF_IRQ_INT0_LSCM_TX_DONE,/* for lscm statis */
	DCAM_IF_IRQ_INT0_GTM_DONE,
};

static const int _DCAM1_SEQUENCE_INT1[] = {
	DCAM_IF_IRQ_INT1_FRGB_HIST_DONE,
	DCAM_IF_IRQ_INT1_DEC_DONE,
	DCAM_IF_IRQ_INT1_SENSOR_SOF1,
	DCAM_IF_IRQ_INT1_SENSOR_SOF2,
	DCAM_IF_IRQ_INT1_SENSOR_SOF3,
	DCAM_IF_IRQ_INT1_DUMMY_START,
	DCAM_IF_IRQ_INT1_FMCU_INT2,
	DCAM_IF_IRQ_INT1_FMCU_INT1,
	DCAM_IF_IRQ_INT1_DUMMY_DONE,
};

static const int _DCAM2_SEQUENCE[] = {
	DCAM_IF_IRQ_INT0_SENSOR_SOF,
	DCAM_IF_IRQ_INT0_CAP_SOF,/* must */
	DCAM_IF_IRQ_INT0_SENSOR_EOF,/* TODO: why for flash */
	DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE,/* for bin path */
	DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE,/* for full path */
};

static const struct {
	size_t count;
	const int *bits;
} DCAM_SEQUENCES[DCAM_HW_CONTEXT_MAX][2] = {
	{
		{
			ARRAY_SIZE(_DCAM0_SEQUENCE),
			_DCAM0_SEQUENCE,
		},
		{
			ARRAY_SIZE(_DCAM0_SEQUENCE_INT1),
			_DCAM0_SEQUENCE_INT1,
		},
	},
	{
		{
			ARRAY_SIZE(_DCAM1_SEQUENCE),
			_DCAM1_SEQUENCE,
		},
		{
			ARRAY_SIZE(_DCAM1_SEQUENCE_INT1),
			_DCAM1_SEQUENCE_INT1,
		},
	},
};

static void dcamint_iommu_regs_dump(struct dcam_hw_context *dcam_hw_ctx)
{
	uint32_t reg = 0;
	uint32_t val[4];

	if (!dcam_hw_ctx) {
		pr_err("fail to get valid input hw_ctx\n");
		return;
	}

	if (dcam_hw_ctx->err_count) {
		for (reg = DCAM_MMU_VERSION; reg <= DCAM_MMU_INT_RAW; reg += 16) {
			val[0] = DCAM_MMU_RD(reg);
			val[1] = DCAM_MMU_RD(reg + 4);
			val[2] = DCAM_MMU_RD(reg + 8);
			val[3] = DCAM_MMU_RD(reg + 12);
			pr_err("fail to handle,offset=0x%04x: %08x %08x %08x %08x\n",
					reg, val[0], val[1], val[2], val[3]);
		}

		pr_err("fail to handle,bin fbc %08x full fbc %08x full y %08x u %08x bin y %08x u %08x raw:%08x\n",
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_YUV_FBC_SCAL_SLICE_PLOAD_BASE_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_FBC_RAW_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE4_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE4_SLICE_U_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE0_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE0_SLICE_U_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_RAW_PATH_BASE_WADDR));
		pr_err("fail to handle,dec layer1 y %08x u %08x layer2 y %08x u %08x\n",
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L1_BASE + DCAM_STORE_DEC_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L1_BASE + DCAM_STORE_DEC_SLICE_U_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L2_BASE + DCAM_STORE_DEC_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L2_BASE + DCAM_STORE_DEC_SLICE_U_ADDR));
		pr_err("fail to handle,dec layer3 y %08x u %08x layer4 y %08x u %08x\n",
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L3_BASE + DCAM_STORE_DEC_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L3_BASE + DCAM_STORE_DEC_SLICE_U_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L4_BASE + DCAM_STORE_DEC_SLICE_Y_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_STORE_DEC_L4_BASE + DCAM_STORE_DEC_SLICE_U_ADDR));
		pr_err("fail to handle,pdaf %08x vch2 %08x vch3 %08x lsc %08x lscm %08x aem %08x "
				"hist %08x\n", DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_PDAF_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_VCH2_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_VCH3_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_LENS_BASE_RADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_LSCM_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_AEM_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_BAYER_HIST_BASE_WADDR));
		pr_err("fail to handle,ppe %08x afl %08x %08x bpc %08x %08x afm %08x nr3 %08x\n",
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_PPE_RIGHT_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, ISP_AFL_DDR_INIT_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, ISP_AFL_REGION_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_BPC_MAP_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_BPC_OUT_ADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_AFM_LUM_FV_BASE_WADDR),
				DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_NR3_WADDR));
		dcam_hw_ctx->err_count -= 1;
	}
}

static irqreturn_t dcamint_error_handler_param(struct dcam_hw_context *dcam_hw_ctx, uint32_t status)
{
	struct dcam_irq_proc irq_proc = {0};

	const char *tb_ovr[2] = {"", ", overflow"};
	const char *tb_lne[2] = {"", ", line error"};
	const char *tb_frm[2] = {"", ", frame error"};
	const char *tb_mmu[2] = {"", ", mmu"};

	if (!dcam_hw_ctx->dcam_irq_cb_handle) {
		pr_err("fail to get valid dcam_online_node\n");
		return IRQ_HANDLED;
	}

	pr_err("fail to get normal status DCAM%u 0x%x%s%s%s%s\n", dcam_hw_ctx->hw_ctx_id, status,
		tb_ovr[!!(status & BIT(DCAM_IF_IRQ_INT0_DCAM_OVF))],
		tb_lne[!!(status & BIT(DCAM_IF_IRQ_INT0_CAP_LINE_ERR))],
		tb_frm[!!(status & BIT(DCAM_IF_IRQ_INT0_CAP_FRM_ERR))],
		tb_mmu[!!(status & BIT(DCAM_IF_IRQ_INT0_MMU_INT))]);

	if (status & BIT(DCAM_IF_IRQ_INT0_MMU_INT)) {
		uint32_t val = DCAM_MMU_RD(DCAM_MMU_INT_STS);

		if (val != dcam_hw_ctx->iommu_status) {
			dcamint_iommu_regs_dump(dcam_hw_ctx);
			dcam_hw_ctx->iommu_status = val;
		}
	}

	/*
	 *bug 1651427
	 * When reset dcam appears on cap mipi overflow, clear the first frame.
	 * Need to be removed in AB chip
	 */
	if ((status & BIT(DCAM_IF_IRQ_INT0_DCAM_OVF)) && DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_CAP_FRM_CLR) == 0){
		pr_warn_ratelimited("warning: to clean first frame DCAM%u 0x%x%s\n", dcam_hw_ctx->hw_ctx_id, status,
		tb_ovr[!!(status & BIT(DCAM_IF_IRQ_INT0_DCAM_OVF))]);
		return IRQ_HANDLED;
	}
	if (dcam_hw_ctx->is_offline_proc) {
		pr_err("fail to offline int error,status:%x \n", status);
		return IRQ_HANDLED;
	}
	if (dcam_hw_ctx->dcam_irq_cb_func) {
		if ((status & DCAMINT_INT0_FATAL_ERROR)
			&& (dcam_online_node_state_get(dcam_hw_ctx->dcam_irq_cb_handle) != STATE_ERROR)) {
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

	irq_info.status = DCAM_REG_RD(idx, DCAM_INT0_MASK);
	irq_info.status1 = DCAM_REG_RD(idx, DCAM_INT1_MASK);
	DCAM_REG_WR(idx, DCAM_INT0_CLR, irq_info.status);
	DCAM_REG_WR(idx, DCAM_INT1_CLR, irq_info.status1);
	pr_err("fail to check 0x%x 0x%x\n", irq_info.status, irq_info.status1);
	return irq_info;
}

struct dcam_irq_info dcamint_isr_root_done_rd(uint32_t idx)
{
	struct dcam_irq_info irq_info = {0};

	irq_info.status = DCAM_REG_RD(idx, DCAM_INT0_MASK);
	irq_info.status1 = DCAM_REG_RD(idx, DCAM_INT1_MASK);
	return irq_info;
}

static void dcamint_fbc_set(struct dcam_hw_context *dcam_hw_ctx, uint32_t status)
{
	spin_lock(&dcam_hw_ctx->fbc_lock);
	if (status & BIT(DCAM_IF_IRQ_INT0_CAP_SOF)) {
		dcam_hw_ctx->cap_fbc_done = 1;
		dcam_hw_ctx->prev_fbc_done = 0;
		if (DCAM_REG_RD(dcam_hw_ctx->hw_ctx_id, DCAM_PATH_SEL) & BIT_8)
			dcam_hw_ctx->cap_fbc_done = 0;
	}
	if (status & BIT(DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE))
		dcam_hw_ctx->cap_fbc_done = 1;
	if (status & BIT(DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE))
		dcam_hw_ctx->prev_fbc_done = 1;
	spin_unlock(&dcam_hw_ctx->fbc_lock);
}

void dcamint_isr_root_writeint(uint32_t idx, struct dcam_irq_info *irq_info)
{
	DCAM_REG_WR(idx, DCAM_INT0_CLR, irq_info->status);
	DCAM_REG_WR(idx, DCAM_INT1_CLR, irq_info->status1);

	irq_info->irq_num = DCAM_IF_IRQ_INT0_NUMBER;
	irq_info->status &= DCAMINT_IRQ_LINE_INT0_MASK;
	irq_info->status1 &= DCAMINT_IRQ_LINE_INT1_MASK;
	dcamint_dcam_int_record(idx, irq_info->status, irq_info->status1);
}

void dcamint_dcam_status_rw(struct dcam_irq_info irq_info, void *priv)
{
	struct dcam_hw_context *dcam_hw_ctx = (struct dcam_hw_context *)priv;
	unsigned int i = 0;

	for (i = 0; i < DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id][0].count; i++) {
		int cur_int = DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id][0].bits[i];

		if (irq_info.status & BIT(cur_int)) {
			if (_DCAM_ISR_IRQ[cur_int]) {
				_DCAM_ISR_IRQ[cur_int](dcam_hw_ctx);
				irq_info.status &= ~dcam_hw_ctx->handled_bits;
				irq_info.status1 &= ~dcam_hw_ctx->handled_bits_on_int1;
				dcam_hw_ctx->handled_bits = 0;
				dcam_hw_ctx->handled_bits_on_int1 = 0;
			} else
				pr_warn("warning: DCAM%u missing handler for int %d\n",
					dcam_hw_ctx->hw_ctx_id, cur_int);
			irq_info.status &= ~BIT(cur_int);
			if (!irq_info.status)
				break;
		}
	}

	/* TODO ignore DCAM_AFM_INTREQ0 now */
	irq_info.status &= ~(BIT(DCAM_IF_IRQ_INT0_AFM_INTREQ0) | BIT(DCAM_IF_IRQ_INT0_GTM_DONE));

	if (unlikely(irq_info.status))
		pr_warn("warning: DCAM%u unhandled int0 bit0x%x\n", dcam_hw_ctx->hw_ctx_id, irq_info.status);

	for (i = 0; i < DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id][1].count; i++) {
		int cur_int = 0;
		if (DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id][1].bits == NULL)
			break;

		cur_int = DCAM_SEQUENCES[dcam_hw_ctx->hw_ctx_id][1].bits[i];
		if (irq_info.status1 & BIT(cur_int)) {
			if (_DCAM_ISRS1[cur_int]) {
				_DCAM_ISRS1[cur_int](dcam_hw_ctx);
			} else
				pr_warn("warning: DCAM%u missing handler for int1 bit%d\n",
					dcam_hw_ctx->hw_ctx_id, cur_int);
			irq_info.status1 &= ~BIT(cur_int);
			if (!irq_info.status1)
				break;
		}
	}

	if (unlikely(irq_info.status1))
		pr_warn("warning: DCAM%u unhandled int1 bit0x%x\n", dcam_hw_ctx->hw_ctx_id, irq_info.status1);

}

irqreturn_t dcamint_isr_root(int irq, void *priv)
{
	struct dcam_hw_context *dcam_hw_ctx = NULL;
	struct dcam_irq_info irq_status = {0};
	struct camera_interrupt *irq_desc = NULL;
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

	if (irq_status.status & DCAMINT_INT0_FBC)
		dcamint_fbc_set(dcam_hw_ctx, irq_status.status);

	/* Interrupt err pro: may need to put it into isr_root */
	if (unlikely(DCAMINT_ALL_ERROR & irq_status.status)) {
		dcamint_error_handler(dcam_hw_ctx, irq_status.status);
		irq_status.status &= (~DCAMINT_ALL_ERROR);
	}

	if (!dcam_hw_ctx->slowmotion_count) {
		if (irq_status.status & DCAM_CAP_SOF) {
			/* record SOF timestamp for current frame */
			if (dcam_hw_ctx->is_offline_proc) {
				struct dcam_offline_node *node = NULL;
				node = (struct dcam_offline_node *)dcam_hw_ctx->dcam_irq_cb_handle;
				node->frame_ts_boot[tsid(dcam_hw_ctx->frame_index)] = ktime_get_boottime();
				ktime_get_ts(&node->frame_ts[tsid(dcam_hw_ctx->frame_index)]);
			} else {
				struct dcam_online_node *node = NULL;
				node = (struct dcam_online_node *)dcam_hw_ctx->dcam_irq_cb_handle;
				node->frame_ts_boot[tsid(dcam_hw_ctx->frame_index)] = ktime_get_boottime();
				ktime_get_ts(&node->frame_ts[tsid(dcam_hw_ctx->frame_index)]);
			}
		}
		irq_desc = cam_queue_empty_interrupt_get();
		irq_desc->irq_num = irq_status.irq_num;
		irq_desc->int_status = irq_status.status;
		irq_desc->int_status1 = irq_status.status1;
		ret = cam_queue_enqueue(&dcam_hw_ctx->dcam_irq_sts_q, &irq_desc->list);
		if (ret) {
			pr_err("fail to dcam%d enqueue irq_status_q.\n", dcam_hw_ctx->hw_ctx_id);
			cam_queue_empty_interrupt_put(irq_desc);
			ret = IRQ_NONE;
			goto exit;
		} else
			complete(&dcam_hw_ctx->dcam_irq_proc_thrd.thread_com);
	} else {
		pr_debug("DCAM%u status=0x%x\n", dcam_hw_ctx->hw_ctx_id, irq_status.status);
		/*dcam isr root write status */
		dcamint_dcam_status_rw(irq_status, priv);
	}
	ret = IRQ_HANDLED;
exit:
	read_unlock(&dcam_hw_ctx->hw->soc_dcam->cam_ahb_lock);
	return ret;
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
	case DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_DATA_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_RAW_OUT;
		break;
	case DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_DATA_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_FULL_OUT;
		break;
	case DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_DATA_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_BIN_OUT;
		break;
	case DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_AEM_OUT;
		break;
	case DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_BAYER_HIST_OUT;
		break;
	case DCAM_IF_IRQ_INT0_LSCM_TX_DONE:
		irq_desc->dcam_cb_type = CAM_CB_DCAM_STATIS_DONE;
		irq_desc->dcam_port_id = PORT_OFFLINE_LSCM_OUT;
		break;
	default :
		pr_err("fail to support irq index BIT%d\n", index);
		ret = -1;
		break;
	}

	pr_debug("irq dcam port id %d\n", irq_desc->dcam_port_id);
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

void dcam_int_tracker_reset(uint32_t idx)
{
	if (is_dcam_id(idx)) {
		memset(dcam_int0_tracker[idx], 0, sizeof(dcam_int0_tracker[idx]));
		memset(dcam_int1_tracker[idx], 0, sizeof(dcam_int1_tracker[idx]));
	}

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

	for (i = 0; i < DCAM_IF_IRQ_INT0_NUMBER; i++) {
		if (dcam_int0_tracker[idx][i])
			pr_info("DCAM%u i=%d, int0=%u\n", idx, i, dcam_int0_tracker[idx][i]);
	}
	for (i = 0; i < DCAM_IF_IRQ_INT1_NUMBER; i++) {
		if (dcam_int1_tracker[idx][i])
			pr_info("DCAM%u i=%d, int1=%u\n", idx, i,  dcam_int1_tracker[idx][i]);
	}

#ifdef DCAM_INT_RECORD
	{
		uint32_t cnt = 0, j = 0;
		for (cnt = 0; cnt < (uint32_t)dcam_int0_tracker[idx][DCAM_IF_IRQ_INT0_SENSOR_EOF]; cnt += 4) {
			j = (cnt & (INT_RCD_SIZE - 1));
			pr_info("DCAM%u j=%d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d, %03d.%04d\n",
				idx, j, (uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_SENSOR_EOF][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_SENSOR_EOF][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_CAP_SOF][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_CAP_SOF][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE][j] & 0xffff,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_AFM_INTREQ1][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_AFM_INTREQ1][j] & 0xffff);
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_LSCM_TX_DONE][j] >> 16,
				(uint32_t)dcam_int_recorder[idx][DCAM_IF_IRQ_INT0_LSCM_TX_DONE][j] & 0xffff,
		}
	}
#endif
}

