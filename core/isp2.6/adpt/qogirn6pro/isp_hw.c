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

#ifdef CAM_HW_ADPT_LAYER

#include <video/sprd_mmsys_pw_domain_qogirn6pro.h>

#define ISP_AXI_STOP_TIMEOUT           1000
#define COEF_HOR_Y_SIZE                32
#define COEF_HOR_UV_SIZE               16
#define COEF_VOR_Y_SIZE                (32 * 8)
#define COEF_VOR_UV_SIZE               (32 * 8)

spinlock_t isp_cfg_lock;

static unsigned long irq_base[4] = {
	ISP_P0_INT_BASE,
	ISP_C0_INT_BASE,
	ISP_P1_INT_BASE,
	ISP_C1_INT_BASE
};

unsigned long cfg_cmd_addr_reg[ISP_CONTEXT_HW_NUM] = {
	ISP_CFG_PRE0_CMD_ADDR,
	ISP_CFG_CAP0_CMD_ADDR,
	ISP_CFG_PRE1_CMD_ADDR,
	ISP_CFG_CAP1_CMD_ADDR
};

static unsigned long fbc_store_base[AFBC_PATH_NUM] = {
	ISP_CAP_FBC_STORE_BASE,
	ISP_VID_FBC_STORE_BASE,
};

/*  following section is fmcu cmds of register update for slice */
static unsigned long store_base[ISP_SPATH_NUM] = {
	ISP_STORE_PRE_CAP_BASE,
	ISP_STORE_VID_BASE,
	ISP_STORE_THUMB_BASE,
};

static unsigned long scaler_base[ISP_SPATH_NUM] = {
	ISP_SCALER_PRE_CAP_BASE,
	ISP_SCALER_VID_BASE,
	ISP_SCALER_THUMB_BASE,
};

static const struct bypass_tag dcam_bypass_tab[] = {
	[_E_4IN1] = {"4in1", DCAM_MIPI_CAP_CFG,           12}, /* 0x100.b12 */
	[_E_PDAF] = {"pdaf", DCAM_PPE_FRM_CTRL0,          1}, /* 0x120.b1 */
	[_E_LSC]  = {"lsc",  DCAM_LENS_LOAD_ENABLE,       0}, /* 0x138.b0 */
	[_E_AEM]  = {"aem",  DCAM_AEM_FRM_CTRL0,          0}, /* 0x150.b0 */
	[_E_HIST] = {"hist", DCAM_BAYER_HIST_CTRL0,       0}, /* 0x160.b0 */
	[_E_AFL]  = {"afl",  ISP_AFL_PARAM0,              0}, /* 0x170.b0 */
	[_E_AFM]  = {"afm",  DCAM_AFM_FRM_CTRL,           0}, /* 0x1A0.b0 */
	[_E_BPC]  = {"bpc",  DCAM_BPC_PARAM,              0}, /* 0x200.b0 */
	[_E_BLC]  = {"blc",  DCAM_BLC_PARA_R_B,           31}, /* 0x268.b31 */
	[_E_RGB]  = {"rgb",  ISP_RGBG_YRANDOM_PARAMETER0, 0}, /* 0x278.b0 rgb gain */
	[_E_RAND] = {"rand", ISP_RGBG_YRANDOM_PARAMETER0, 1}, /* 0x278.b1 */
	[_E_PPI]  = {"ppi",  ISP_PPI_PARAM,               0}, /* 0x284.b0 */
	[_E_AWBC] = {"awbc", DCAM_AWBC_GAIN0,             31}, /* 0x380.b31 */
	[_E_NR3]  = {"nr3",  DCAM_NR3_FAST_ME_PARAM,      0}, /* 0x3F0.b0 */
};

static const struct bypass_tag isp_hw_bypass_tab[] = {
[_EISP_HSV]     = {"hsv",     ISP_HSV_PARAM,           0, 1},
[_EISP_PRECDN]  = {"precdn",  ISP_PRECDN_PARAM,        0, 1},
[_EISP_YNR]     = {"ynr",     ISP_YNR_CONTRL0,         0, 1},
[_EISP_EE]      = {"ee",      ISP_EE_PARAM,            0, 1},
[_EISP_GAMY]    = {"ygamma",  ISP_YGAMMA_PARAM,        0, 1},
[_EISP_CDN]     = {"cdn",     ISP_CDN_PARAM,           0, 1},
[_EISP_POSTCDN] = {"postcdn", ISP_POSTCDN_COMMON_CTRL, 0, 1},
[_EISP_UVD]     = {"uvd",     ISP_UVD_PARAM,           0, 1},
[_EISP_IIRCNR]  = {"iircnr",  ISP_IIRCNR_PARAM,        0, 1},
[_EISP_YRAND]   = {"yrandom", ISP_YRANDOM_PARAM1,      0, 1},
[_EISP_BCHS]    = {"bchs",    ISP_BCHS_PARAM,          0, 1},
[_EISP_YUVNF]   = {"yuvnf",   ISP_YUV_NF_CTRL,         0, 1},

	{"ydelay",    ISP_YDELAY_PARAM,                         0,  1},
	{"fetch-fbd", ISP_FBD_RAW_SEL,                          0,  1},
	/* can't bypass when prev */
	{"scale-pre", ISP_CAP_SCALER_CFG,                       20, 0},
	{"store-pre", ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM, 0,  0},
	{"scale-vid", ISP_RECORD_SCALER_CFG,                    20, 1},
	{"store-vid", ISP_STORE_VID_BASE + ISP_STORE_PARAM,     0,  1},
	{"scale-thb", ISP_THMB_SCALER_CFG,                      20, 1},
	{"store-thb", ISP_STORE_THUMB_BASE + ISP_STORE_PARAM,   0,  1},
	/* ltm */
	{"ltm-map",   ISP_LTM_MAP_PARAM0,                       0,  1},
	{"ltm-hist",  ISP_LTM_PARAMETERS,                       0,  1},
	/* 3dnr/nr3 */
	{"nr3-crop",  ISP_3DNR_CROP_PARAM0,                     0,  1},
	{"nr3-store", ISP_3DNR_STORE_PARAM,                     0,  1},
	{"nr3-mem",   ISP_3DNR_MEM_CTRL_PARAM0,                 0,  1},

	{"fetch",     ISP_FETCH_PARAM0,                         0,  0},
	{"cfg",       ISP_CFG_PAMATER,                          0,  0},
};

static uint32_t isphw_ap_fmcu_reg_get(struct isp_fmcu_ctx_desc *fmcu, uint32_t reg)
{
	uint32_t addr = 0;

	if (fmcu)
		addr = ISP_GET_REG(reg);
	else
		addr = reg;

	return addr;
}

static void isphw_ap_fmcu_reg_write(struct isp_fmcu_ctx_desc *fmcu,
		uint32_t ctx_id, uint32_t addr, uint32_t cmd)
{
	if (fmcu)
		FMCU_PUSH(fmcu, addr, cmd);
	else
		ISP_REG_WR(ctx_id, addr, cmd);
}

static int isphw_bypass_count_get(void *handle, void *arg)
{
	int cnt = 0;
	uint32_t type = 0;

	type = *(uint32_t *)arg;

	switch (type) {
	case DCAM_BYPASS_TYPE:
		cnt = sizeof(dcam_bypass_tab) /
			sizeof(dcam_bypass_tab[0]);
		break;
	case ISP_BYPASS_TYPE:
		cnt = sizeof(isp_hw_bypass_tab) /
			sizeof(isp_hw_bypass_tab[0]);
		break;
	default:
		pr_err("fail to support bypass type %d\n", type);
		break;
	}

	if (cnt == 0)
		pr_err("fail to get valid bypass %d\n", type);

	return cnt;
}

static int isphw_bypass_data_get(void *handle, void *arg)
{
	struct bypass_tag *bypass = NULL;
	struct cam_hw_bypass_data *data = NULL;

	data = (struct cam_hw_bypass_data *)arg;

	switch (data->type) {
	case DCAM_BYPASS_TYPE:
		bypass = (struct bypass_tag *)&dcam_bypass_tab[data->i];
		break;
	case ISP_BYPASS_TYPE:
		bypass = (struct bypass_tag *)&isp_hw_bypass_tab[data->i];
		break;
	default:
		pr_err("fail to support bypass type %d\n", data->type);
		break;
	}

	if (bypass == NULL) {
		pr_err("fail to get valid block func %d\n", data->type);
		return -EFAULT;
	}

	data->tag = bypass;

	return 0;
}

static uint32_t cam_reg_trace_tab[] = {
	DCAM_APB_SRAM_CTRL,
	DCAM_MIPI_CAP_CFG,
	DCAM_MIPI_CAP_END,
	DCAM_IMAGE_CONTROL,
	DCAM_PATH_SEL,
	DCAM_BUF_CTRL,
	DCAM_PDAF_CONTROL,
	DCAM_LENS_LOAD_ENABLE,
	DCAM_BPC_PARAM,
	DCAM_AEM_FRM_CTRL0,
	DCAM_AFM_FRM_CTRL,
	ISP_AFL_PARAM0,
	DCAM_BAYER_HIST_CTRL0,
	DCAM_NR3_FAST_ME_PARAM,
	DCAM_FBC_RAW_PARAM,
	DCAM_STORE4_PARAM,
	DCAM_STORE4_SLICE_Y_ADDR,
	DCAM_STORE4_SLICE_U_ADDR,
	DCAM_YUV_FBC_SCAL_PARAM,
	DCAM_STORE0_PARAM,
	DCAM_STORE0_SLICE_SIZE,
	DCAM_STORE0_SLICE_Y_ADDR,
	DCAM_STORE0_SLICE_U_ADDR,
	DCAM_SCL0_CFG,
	DCAM_RAW_PATH_CFG,
	DCAM_RAW_PATH_BASE_WADDR,
	DCAM_SCL0_CFG,
	DCAM_PDAF_BASE_WADDR,
	DCAM_VCH2_BASE_WADDR,
	DCAM_VCH3_BASE_WADDR,
	DCAM_AEM_BASE_WADDR,
	DCAM_BAYER_HIST_BASE_WADDR,
	DCAM_PPE_RIGHT_WADDR,
	ISP_AFL_DDR_INIT_ADDR,
	ISP_AFL_REGION_WADDR,
	DCAM_BPC_OUT_ADDR,
	DCAM_AFM_LUM_FV_BASE_WADDR,
	DCAM_NR3_WADDR,
	DCAM_LSCM_BASE_WADDR,
};

static int isphw_reg_trace(void *handle, void *arg)
{
	unsigned long addr = 0;
	uint32_t val_mmu, val[8], i, j, n, cnt;
	struct cam_hw_reg_trace *trace = NULL;

	trace = (struct cam_hw_reg_trace *)arg;

	if (trace->type == NORMAL_REG_TRACE) {
		goto normal_reg_trace;
	} else if (trace->type == ABNORMAL_REG_TRACE) {
		goto abnormal_reg_trace;
	} else {
		pr_err("fail to get valid type %d\n", trace->type);
		return -EFAULT;
	}

abnormal_reg_trace:
	pr_info("DCAM%d: Register list\n", trace->idx);

	for (addr = DCAM_IP_REVISION; addr <= DCAM_CAP_FBC_STATUS3;
		addr += 16) {
		pr_info("0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}
	for (addr = DCAM_MIPI_CAP_CFG; addr <= DCAM_CAP_RAW_SIZE;
		addr += 16) {
		pr_info("0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}

	pr_info("FMCU: Register list\n");
	for (addr = MM_DCAM_FMCU_BASE; addr <= DCAM_FMCU_SW_TRIGGER;
		addr += 16) {
		pr_info("0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_FMCU_RD(addr),
			DCAM_FMCU_RD(addr + 4),
			DCAM_FMCU_RD(addr + 8),
			DCAM_FMCU_RD(addr + 12));
	}

	addr = DCAM_CROP0_PARAM0;
	pr_info("CROP0:0x%03lx: 0x%x 0x%x 0x%x\n",
		addr,
		DCAM_REG_RD(trace->idx, addr),
		DCAM_REG_RD(trace->idx, addr + 4),
		DCAM_REG_RD(trace->idx, addr + 8));
	addr = DCAM_CROP1_CTRL;
	pr_info("CROP1(raw):0x%03lx: 0x%x 0x%x 0x%x\n",
		addr,
		DCAM_REG_RD(trace->idx, addr),
		DCAM_REG_RD(trace->idx, addr + 4),
		DCAM_REG_RD(trace->idx, addr + 8));
	addr = DCAM_CROP2_CTRL;
	pr_info("CROP2(cap):0x%03lx: 0x%x 0x%x 0x%x\n",
		addr,
		DCAM_REG_RD(trace->idx, addr),
		DCAM_REG_RD(trace->idx, addr + 4),
		DCAM_REG_RD(trace->idx, addr + 8));
	addr = DCAM_CROP3_CTRL;
	pr_info("CROP3(afm):0x%03lx: 0x%x 0x%x 0x%x\n",
		addr,
		DCAM_REG_RD(trace->idx, addr),
		DCAM_REG_RD(trace->idx, addr + 4),
		DCAM_REG_RD(trace->idx, addr + 8));



	for (addr = DCAM_CFA_NEW_CFG0; addr <= DCAM_CFA_GBUF_CFG; addr += 16) {
		pr_info("CFA:0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}
	for (addr = DCAM_CMC10_PARAM; addr <= DCAM_CMC10_MATRIX4; addr += 16) {
		pr_info("CMC10:0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}

	for (addr = DCAM_CCE_PARAM; addr <= DCAM_CCE_SHIFT; addr += 16) {
		pr_info("CCE:0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}
	pr_info("YUV444TO420:0x%03lx: 0x%x, 0x%x\n", DCAM_YUV444TOYUV420_PARAM,
		DCAM_REG_RD(trace->idx, DCAM_YUV444TOYUV420_PARAM),
		DCAM_REG_RD(trace->idx, DCAM_YUV444TOYUV420_PARAM + 4));

	for (addr = DCAM_SCL0_CFG; addr <= DCAM_SCL0_BWD_PARA; addr += 16) {
		pr_info("SCL(pre):0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}

	for (addr = DCAM_STORE0_PARAM; addr <= DCAM_STORE0_SHADOW_CLR;
		addr += 16) {
		pr_info("store0(pre):0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}
	for (addr = DCAM_STORE4_PARAM; addr <= DCAM_STORE4_SHADOW_CLR;
		addr += 16) {
		pr_info("store4(cap):0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}

	for (addr = DCAM_RAW_PATH_CFG; addr <= DCAM_RAW_PATH_BASE_WADDR;
		addr += 16) {
		pr_info("raw:0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_REG_RD(trace->idx, addr),
			DCAM_REG_RD(trace->idx, addr + 4),
			DCAM_REG_RD(trace->idx, addr + 8),
			DCAM_REG_RD(trace->idx, addr + 12));
	}

	pr_info("AXIM: Register list\n");
	for (addr = AXIM_STATUS0; addr <= DCAM_DUMMY_SLAVE_CFG;
		addr += 16) {
		pr_info("0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_AXIM_RD(trace->idx, addr),
			DCAM_AXIM_RD(trace->idx, addr + 4),
			DCAM_AXIM_RD(trace->idx, addr + 8),
			DCAM_AXIM_RD(trace->idx, addr + 12));
	}

	pr_info("ISP: Register list\n");
	for (addr = ISP_INT_EN0; addr <= ISP_INT_ALL_DONE_SRC_CTRL;
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}

	pr_info("ISP: Common Register list\n");
	for (addr = 0x988; addr <= 0x998; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}

	pr_info("ISP: pyr_rec Register list\n");
	for (addr = 0x9310; addr <= 0x9334; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}
	for (addr = 0x9510; addr <= 0x9534; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}
	for (addr = 0x9610; addr <= 0x963C; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}
	for (addr = 0x9B10; addr <= 0x9B14; addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}

	pr_info("ISP fetch: register list\n");
	for (addr = (ISP_FETCH_BASE + ISP_FETCH_PARAM0); addr <= (ISP_FETCH_BASE + ISP_FETCH_MIPI_PARAM_UV); addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));

		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(0, addr),
			ISP_REG_RD(0, addr + 4),
			ISP_REG_RD(0, addr + 8),
			ISP_REG_RD(0, addr + 12));
	}

	pr_info("ISP dispatch: register list\n");
	for (addr = (ISP_DISPATCH_BASE + ISP_DISPATCH_CH0_BAYER); addr <= (ISP_DISPATCH_BASE + ISP_DISPATCH_CHK_SUM);
		addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(0, addr),
			ISP_REG_RD(0, addr + 4),
			ISP_REG_RD(0, addr + 8),
			ISP_REG_RD(0, addr + 12));
	}

	for (addr = (ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG); addr <= (ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_RES);
			addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(0, addr),
			ISP_REG_RD(0, addr + 4),
			ISP_REG_RD(0, addr + 8),
			ISP_REG_RD(0, addr + 12));
	}

	pr_info("ISP store: register list\n");
	for (addr = (ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM); addr <= (ISP_STORE_PRE_CAP_BASE + ISP_STORE_SHADOW_CLR);
			addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(0, addr),
			ISP_REG_RD(0, addr + 4),
			ISP_REG_RD(0, addr + 8),
			ISP_REG_RD(0, addr + 12));
	}

	pr_info("ISP mmu: register list\n");
	for (addr = ISP_MMU_INT_EN; addr <= ISP_MMU_INT_RAW;
			addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
	}

	pr_info("ISP common status: register list\n");
	for (addr = ISP_COMMON_BASE; addr <= ISP_COMMON_YDELAY_STATUS4;
			addr += 16) {
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_HREG_RD(addr),
			ISP_HREG_RD(addr + 4),
			ISP_HREG_RD(addr + 8),
			ISP_HREG_RD(addr + 12));
		pr_info("0x%lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			ISP_REG_RD(0, addr),
			ISP_REG_RD(0, addr + 4),
			ISP_REG_RD(0, addr + 8),
			ISP_REG_RD(0, addr + 12));
	}

normal_reg_trace:
	val_mmu = DCAM_MMU_RD(MMU_EN);
	cnt = sizeof(cam_reg_trace_tab) /
		sizeof(cam_reg_trace_tab[0]);
	pr_info("dcam%d: 0x%08x, cnt %d\n", trace->idx, val_mmu, cnt);

	for (i = 0; i < cnt; i += 8) {
		memset(val, 0, sizeof(val));
		n = ((cnt - i) < 8) ? (cnt - i) : 8;
		for (j = 0; j < n; j++) {
			addr = cam_reg_trace_tab[i + j];
			val[j] = DCAM_REG_RD(trace->idx, addr);
		}
		pr_info("n=%d, %08x %08x %08x %08x %08x %08x %08x %08x\n", n,
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
	}

	pr_info("AXIM: Register list\n");
	for (addr = AXIM_STATUS0; addr <= DCAM_DUMMY_SLAVE_CFG;
		addr += 16) {
		pr_info("0x%03lx: 0x%x 0x%x 0x%x 0x%x\n",
			addr,
			DCAM_AXIM_RD(trace->idx, addr),
			DCAM_AXIM_RD(trace->idx, addr + 4),
			DCAM_AXIM_RD(trace->idx, addr + 8),
			DCAM_AXIM_RD(trace->idx, addr + 12));
	}

	return 0;
}

static int isphw_pw_on(void *handle, void *arg)
{
	int ret = 0;
	ret = sprd_isp_pw_on();
	return ret;
}

static int isphw_pw_off(void *handle, void *arg)
{
	int ret = 0;
	ret = sprd_isp_pw_off();
	return ret;
}

static int isphw_clk_eb(void *handle, void *arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_soc_info *soc = NULL;

	pr_debug(",E\n");
	if (!handle) {
		pr_err("fail to get invalid hw\n");
		return -EINVAL;
	}

	hw = (struct cam_hw_info *)handle;
	soc = hw->soc_isp;

	ret = clk_set_parent(soc->clk, soc->clk_parent);
	if (ret) {
		pr_err("fail to set parent, ret = %d\n", ret);
		clk_set_parent(soc->clk, soc->clk_default);
		return ret;
	}
	ret = clk_prepare_enable(soc->clk);
	if (ret) {
		pr_err("fail to enable isp clk, ret = %d\n", ret);
		clk_set_parent(soc->clk, soc->clk_default);
		return ret;
	}
	ret = clk_prepare_enable(soc->core_eb);
	if (ret) {
		pr_err("fail to set isp eb, ret = %d\n", ret);
		clk_disable_unprepare(soc->clk);
		return ret;
	}

	ret = clk_prepare_enable(soc->mtx_en);
	if (ret) {
		pr_err("fail to set isp mtx eb, ret = %d\n", ret);
		clk_disable_unprepare(soc->core_eb);
		return ret;
	}

	ret = clk_prepare_enable(soc->blk_cfg_en);
	if (ret) {
		pr_err("fail to set parent, ret = %d\n", ret);
		clk_disable_unprepare(soc->mtx_en);
		clk_disable_unprepare(soc->blk_cfg_en);
		clk_disable_unprepare(soc->core_eb);
		return ret;
	}
	ret = clk_prepare_enable(soc->tck_en);

	return ret;
}

static int isphw_clk_dis(void *handle, void *arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_soc_info *soc = NULL;

	pr_debug(",E\n");
	if (!handle) {
		pr_err("fail to get valid hw\n");
		return -EINVAL;
	}

	hw = (struct cam_hw_info *)handle;
	soc = hw->soc_isp;

	clk_set_parent(soc->clk, soc->clk_default);
	clk_disable_unprepare(soc->clk);

	clk_disable_unprepare(soc->tck_en);
	clk_disable_unprepare(soc->blk_cfg_en);
	clk_disable_unprepare(soc->mtx_en);
	clk_disable_unprepare(soc->core_eb);

	return ret;
}

static int isphw_reset(void *handle, void *arg)
{
	int rtn = 0;
	uint32_t cid;
	uint32_t time_out = 0;
	uint32_t flag = 0;
	uint32_t reset_flag = 0;
	struct cam_hw_soc_info *soc = NULL;
	struct cam_hw_ip_info *ip = NULL;
	struct cam_hw_info *hw = NULL;

	if (!handle) {
		pr_err("fail to get valid hw\n");
		return -EINVAL;
	}

	hw = (struct cam_hw_info *)handle;
	soc = hw->soc_isp;
	ip = hw->ip_isp;
	reset_flag = *(uint32_t *)arg;
	pr_info("ISP%d: reset\n", ip->idx);

	/* firstly stop axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT(26), BIT(26));

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		/* bit3: 1 - axi idle;  0 - axi busy */
		if (ISP_HREG_RD(ISP_INT_STATUS) & BIT_3)
			break;
		udelay(1000);
	}

	if (reset_flag == ISP_RESET_AFTER_POWER_ON) {
		if (time_out >= ISP_AXI_STOP_TIMEOUT) {
			pr_info("ISP reset timeout %d\n", time_out);
		} else {
			flag = ip->syscon.rst_mask
				| ip->syscon.rst_ahb_mask
				| ip->syscon.rst_vau_mask;
			regmap_update_bits(soc->cam_ahb_gpr,
				ip->syscon.rst, flag, flag);
			udelay(10);
			regmap_update_bits(soc->cam_ahb_gpr,
				ip->syscon.rst, flag, ~flag);
		}
	}

	/* enable axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);

	for (cid = 0; cid < 4; cid++) {
		hw->isp_ioctl(hw, ISP_HW_CFG_DISABLE_IRQ, &cid);
		hw->isp_ioctl(hw, ISP_HW_CFG_CLEAR_IRQ, &cid);
	}

	pr_info("ISP%d: reset end\n", ip->idx);
	return rtn;
}

static int isphw_irq_enable(void *handle, void *arg)
{
	uint32_t ctx_id;
	uint32_t mask = ~0;

	if (!arg) {
		pr_err("fail to get valid hw\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("fail to get valid ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_MWR(irq_base[ctx_id] + ISP_INT_EN0, mask, mask);
	ISP_HREG_MWR(irq_base[ctx_id] + ISP_INT_EN1, mask, mask);
	ISP_HREG_WR(ISP_DEC_INT_BASE + ISP_INT_EN0, ISP_DEC_INT_LINE_MASK);

	return 0;
}

static int isphw_irq_disable(void *handle, void *arg)
{
	uint32_t ctx_id;

	if (!arg) {
		pr_err("fail to get valid hw\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("fail to get valid  ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN0, 0);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN1, 0);
	ISP_HREG_WR(ISP_DEC_INT_BASE + ISP_INT_EN0, 0);

	return 0;
}

static int isphw_irq_clear(void *handle, void *arg)
{
	uint32_t ctx_id;

	if (!arg) {
		pr_err("fail to get valid hw\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("fail to get valid ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR0, 0xFFFFFFFF);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR1, 0xFFFFFFFF);

	return 0;
}

static uint32_t ISP_CFG_MAP[] __aligned(8) = {
		0x00300710, /*0x710   - 0x73C  , 12 , COMMON*/
		0x00201110, /*0x1110  - 0x112C , 8   , UVD*/
		0x00081610, /*0x1610  - 0x1614 , 2   , YUVDELAY*/
		0x01E01710, /*0x1710  - 0x18EC , 120 , CNR*/
		0x00402010, /*0x2010  - 0x204C , 16  , mem_ctrl*/
		0x00642110, /*0x2110  - 0x2170 , 25  , nr3_blend*/
		0x00302210, /*0x2210  - 0x223C , 12  , 3dnr_store*/
		0x00102310, /*0x2310  - 0x231C , 4   , crop*/
		0x00202410, /*0x2410  - 0x242C , 8   , AFBC_3DNR*/
		0x001C2510, /*0x2510  - 0x2528 , 7   , AFBD_3DNR*/
		0x00042710, /*0x2710  - 0x2710 , 1   , BWD_3DNR*/
		0x00203010, /*0x3010  - 0x302C , 8   , YUV420toRGB*/
		0x001C3110, /*0x3110  - 0x3128 , 7   , frgb_LTM_hists*/
		0x00183210, /*0x3210  - 0x3224 , 6   , frgb_LTM_map*/
		0x005C3310, /*0x3310  - 0x3368 , 23  , HSV*/
		0x00043410, /*0x3410  - 0x3410 , 1   , CTM*/
		0x00203510, /*0x3510  - 0x352C , 8   , RGBtoYUV420*/
		0x00484010, /*0x4010  - 0x4054 , 18  , CDN*/
		0x00044110, /*0x4110  - 0x4110 , 1   , YGAMMA*/
		0x011C4210, /*0x4210  - 0x4328 , 71  , NEW_EE*/
		0x00104410, /*0x4410  - 0x441C , 4   , YRANDOM*/
		0x00144510, /*0x4510  - 0x4520 , 5   , BCHS */
		0x00545010, /*0x5010  - 0x5060 , 21  , SCL_CAP*/
		0x00305210, /*0x5210  - 0x523C , 12  , SCL_CAP_store*/
		0x00205310, /*0x5310  - 0x532C , 8   , scl_afbc_store_cap*/
		0x00C05110, /*0x5110  - 0x51CC , 48  , SCL_CAP_COEF*/
		0x00345410, /*0x5410  - 0x5440 , 13  , SCL_CAP_noisefilter_add_rdm*/
		0x00C05510, /*0x5510  - 0x55CC , 48  , SCL_CAP_COEF1*/
		0x00C05610, /*0x5610  - 0x56CC , 48  , SCL_CAP_COEF2*/
		0x00C05710, /*0x5710  - 0x57CC , 48  , SCL_CAP_COEF3*/
		0x00686010, /*0x6010  - 0x6074 , 26  , SCL_VID*/
		0x00C06110, /*0x6110  - 0x61CC , 48  , SCL_VID_COEF*/
		0x00306210, /*0x6210  - 0x623C , 12  , SCL_VID_store*/
		0x00206310, /*0x6310  - 0x632C , 8   , scl_afbc_store_vid*/
		0x00C06510, /*0x6510  - 0x65CC , 48  , SCL_VID_COEF1*/
		0x00C06610, /*0x6610  - 0x66CC , 48  , SCL_VID_COEF2*/
		0x00C06710, /*0x6710  - 0x67CC , 48  , SCL_VID_COEF3*/
		0x00607010, /*0x7010  - 0x706C , 24  , SCL_THUMBNAIL*/
		0x00307110, /*0x7110  - 0x713C , 12  , SCL_THUMBNAIL_store*/
		0x00289310, /*0x9310  - 0x9334 , 10  , PYR_REC_FETCH1*/
		0x00289510, /*0x9510  - 0x9534 , 10  , PYR_REC*/
		0x00309610, /* 0x9610  - 0x963C , 12  , PYR_REC_STORE */
		0x01E09710, /*0x9710  - 0x98EC , 120 , PYR_REC_CNR*/
		0x00849A10, /*0x9A10  - 0x9A90 , 33  , PYR_REC_YNR*/
		0x00089B10, /*0x9B10  - 0x9B14 , 2   , PYR_REC_UVDELAY*/
		0x00280110, /*0x110   - 0x134  , 10  , FETCH*/
		0x001C0210, /*0x210   - 0x228  , 7   , AFBD_Fetch*/
		0x00180310, /*0x310   - 0x324  , 6   , DISPATCH*/
		0x0040A010, /*0xA010  - 0xA04C , 16  , Dewarping*/
		0x0010A110, /*0xA110  - 0xA11C , 4   , Dewarping_cache*/
		0x01D58000, /*0x18000 - 0x181D0, 117 , ISP_HSV_TABLEA_BUF0_CH0*/
		0x01A18400, /*0x18400 - 0x1859C, 104 , ISP_HSV_TABLEB_BUF0_CH0*/
		0x01B18800, /*0x18800 - 0x189AC, 108 , ISP_HSV_TABLEC_BUF0_CH0*/
		0x01818C00, /*0x18C00 - 0x18D7C, 96  , ISP_HSV_TABLED_BUF0_CH0*/
		0x08019600, /*0x19600 - 0x19DFC, 512 , ISP_YGAMMA_BUF0_CH0*/
		0x0201A000, /*0x1A000 - 0x1A1FC, 128 , ISP_RGB_LTM_BUF0_CH0*/
		0x0B65B000, /*0x1B000 - 0x1BB60, 729 , ISP_CTM_LUT_BUF0_CH0*/
		0x03FDD8F0, /*0x1D8F0 - 0x1DCE8, 255 , ISP_CAP_PATH_VER_CORF_Y_BUF0_CH0*/
		0x03FDF000, /*0x1F000 - 0x1F3F8, 255 , ISP_CAP_PATH_VER_CORF_UV_BUF0_CH0*/
		0x03FE0400, /*0x20400 - 0x207F8, 255 , ISP_VID_PATH_HOR_CORF_Y_BUF0_CH0*/
		0x03FE0C00, /*0x20C00 - 0x20FF8, 255 , ISP_VID_PATH_HOR_CORF_UV_BUF0_CH0*/
		0x03FE1400, /*0x21400 - 0x217F8, 255 , ISP_VID_PATH_VER_CORF_Y_BUF0_CH0*/
		0x03FE1C00, /*0x21C00 - 0x21FF8, 255 , ISP_VID_PATH_VER_CORF_UV_BUF0_CH0*/
		0x1183A000, /*0x3A000 - 0x3B17C, 1120, ISP_DEWARPING_GRID_X_CH0*/
		0x1183B180, /*0x3B180 - 0x3C2FC, 1120, ISP_DEWARPING_GRID_Y_CH0*/
		0x080BC300, /*0x3C300 - 0x3CB04, 514 , ISP_DEWARPING_CORD_COEF_CH0*/
		0x020BCC00, /*0x3CC00 - 0x3CE04, 130 , ISP_DEWARPING_PXL_COEF_CH0*/
};
static int isphw_cfg_map_info_get(void *handle, void *arg)
{
	struct isp_dev_cfg_info *info = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	info = (struct isp_dev_cfg_info *)arg;
	info->num_of_mod = ARRAY_SIZE(ISP_CFG_MAP);
	info->isp_cfg_map = ISP_CFG_MAP;

	return 0;
}

static int isphw_default_param_set(void *handle, void *arg)
{
	uint32_t idx = 0;
	uint32_t bypass = 1;
	uint32_t wqos_val = 0;
	uint32_t rqos_val = 0;
	uint32_t val = 0;
	struct isp_hw_default_param *param = NULL;
	struct cam_hw_info *hw = NULL;

	param = (struct isp_hw_default_param *)arg;
	hw = (struct cam_hw_info *)handle;
	spin_lock_init(&isp_cfg_lock);

	if (param->type == ISP_HW_PARA) {
		goto isp_hw_para;
	} else if (param->type == ISP_CFG_PARA) {
		idx = param->index;
		goto isp_hw_cfg_para;
	} else {
		pr_err("fail to get valid type %d\n", param->type);
	}

isp_hw_para:
	wqos_val = (0x1 << 13) | (0x0 << 12) | (0x4 << 8) |
			((hw->soc_isp->awqos_high & 0xF) << 4) |
			(hw->soc_isp->awqos_low & 0xF);
	rqos_val = (0x0 << 8) |
			((hw->soc_isp->arqos_high & 0xF) << 4) |
			(hw->soc_isp->arqos_low & 0xF);
	ISP_HREG_MWR(ISP_AXI_ARBITER_WQOS, 0x37FF, wqos_val);
	ISP_HREG_MWR(ISP_AXI_ARBITER_RQOS, 0x1FF, rqos_val);
	ISP_HREG_WR(ISP_CORE_PMU_EN, 0xFFFF0000);
	ISP_HREG_MWR(ISP_AXI_ISOLATION, BIT_0, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_0 | BIT_1, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_4 | BIT_5, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_8 | BIT_9, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN1, BIT_0 | BIT_1, 0);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM_CLR, 0);
	/* enable axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);
	/* to be defined. */
	/* dispatch_done should be disable? */
	ISP_HREG_MWR(ISP_INT_ALL_DONE_CTRL, 0x1F, 0x18);
	/* bypass config mode by default */
	ISP_HREG_MWR(ISP_CFG_PAMATER, BIT_0, 1);
	ISP_HREG_MWR(PYR_DEC_STORE_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_HREG_MWR(PYR_DCT_STORE_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_HREG_MWR(PYR_REC_STORE_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_HREG_MWR(PYR_DEC_FETCH_BASE + ISP_FETCH_PARAM0, BIT_0, 1);
	ISP_HREG_MWR(PYR_DEC_DISPATCH_BASE + ISP_DISPATCH_LINE_DLY1, BIT_16, BIT_16);
	pr_debug("end\n");
	return 0;

isp_hw_cfg_para:
	/*common*/
	ISP_REG_MWR(idx, ISP_COMMON_GCLK_CTRL_0, 0xFFFF0000, 0xFFFF0000);
	ISP_REG_MWR(idx, ISP_COMMON_GCLK_CTRL_1, 0xFFFF0000, 0xFFFF0000);
	ISP_REG_MWR(idx, ISP_COMMON_GCLK_CTRL_2, 0xFFFF0000, 0xFFFF0000);
	ISP_REG_MWR(idx, ISP_COMMON_GCLK_CTRL_3, 0xFF00, 0xFF00);

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, BIT_3 | BIT_2, 3 << 2);
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, 0x70, 5 << 4);

	ISP_REG_WR(idx, ISP_COMMON_FMCU0_PATH_SEL, 0);
	ISP_REG_WR(idx, ISP_COMMON_FMCU1_PATH_SEL, 2);
	ISP_REG_WR(idx, ISP_COMMON_FMCU2_PATH_SEL, 1);
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, BIT_16, (1 << 16));
	ISP_REG_MWR(idx, ISP_COMMON_SHADOW_CTRL_CH0, BIT_21, (1 << 21));
	ISP_REG_MWR(idx, ISP_COMMON_PMU_RAM_MASK, BIT_0, 1);

	ISP_REG_WR(idx, ISP_DISPATCH_BASE + ISP_DISPATCH_DLY, 0x250050);
	ISP_REG_WR(idx, ISP_DISPATCH_BASE + ISP_DISPATCH_LINE_DLY1, 0x100280);
	ISP_REG_WR(idx, ISP_DISPATCH_BASE + ISP_DISPATCH_PIPE_BUF_CTRL_CH0, 0x64043c);

	/* bypass all path scaler & store */
	ISP_REG_MWR(idx, ISP_CAP_SCALER_CFG, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_RECORD_SCALER_CFG, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_THMB_SCALER_CFG, BIT_0, bypass);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_HBLANK, 0x4040);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_RES, 0xFF);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_DEBUG, 1);
	ISP_REG_MWR(idx, ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_THUMB_BASE + ISP_STORE_PARAM, BIT_0, 1);

	/* default bypass all blocks */
	/* frgb domain bypass*/
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_6, 0);
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_UVD_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_LTM_PARAMETERS, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_LTM_MAP_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_CNR_CONTRL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CTM_PARAM, BIT_0, 1);

	/* FBC bypass*/
	ISP_REG_MWR(idx, ISP_CAP_FBC_STORE_BASE + ISP_FBC_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_VID_FBC_STORE_BASE + ISP_FBC_STORE_PARAM, BIT_0, 1);

	/* Dewarping bypass*/
	ISP_REG_MWR(idx, ISP_DEWARPING_PARA, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_DEWARPING_CACHE_PARA, BIT_0, 1);

	/* dec/rec bypass*/
	ISP_REG_MWR(idx, PYR_REC_CUR_FETCH_BASE + ISP_FETCH_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_DEC_OFFLINE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YNR_DCT_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_REC_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_REC_CNR_CONTRL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_REC_YNR_CONTRL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_REC_UVDELAY_PARAM, BIT_0, 1);

	/*fetch*/
	ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_PARAM1, 0xA0A0, 0xA0A0);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_HBLANK_TILE_PITCH, 0xFFFF, 0x8000);

	/* 3DNR bypass */
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_BLEND_CONTROL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_FBC_3DNR_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_FBD_3DNR_SEL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_BWD_PARA, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_CROP_PARAM0, BIT_0, 1);
	/*3DNR mem_ctrl data_toyuv_en must enable*/
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_12, 1 << 12);

	/*FRGB IN PIPELINE*/
	/*YUV2RGB*/
	ISP_REG_MWR(idx, ISP_YUV420toRGB_PARAM, BIT_0, 0);
	val = (256 & 0x7FF) | ((0 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_YUV420toRGB_MATRIX0, val);
	val = (358 & 0x7FF) | ((256 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_YUV420toRGB_MATRIX1, val);
	val = (-88 & 0x7FF) | ((-182 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_YUV420toRGB_MATRIX2, val);
	val = (256 & 0x7FF) | ((453 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_YUV420toRGB_MATRIX3, val);
	val = (0 & 0x7FF);
	ISP_REG_WR(idx, ISP_YUV420toRGB_MATRIX4, val);
	ISP_REG_MWR(idx, ISP_YUV420toRGB_SHIFT0, 0X7FF0000, 0 << 16);
	ISP_REG_MWR(idx, ISP_YUV420toRGB_SHIFT0, 0X7FF, 0);
	ISP_REG_MWR(idx, ISP_YUV420toRGB_SHIFT1, 0X7FF, 0);
	/*RGB2YUV*/
	ISP_REG_MWR(idx, ISP_RGB2YUV420_PARAM, BIT_0, 0);
	val = (77 & 0x7FF) | ((150 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_RGB2YUV420_MATRIX0, val);
	val = (29 & 0x7FF) | ((-43 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_RGB2YUV420_MATRIX1, val);
	val = (-85 & 0x7FF) | ((128 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_RGB2YUV420_MATRIX2, val);
	val = (128 & 0x7FF) | ((-107 & 0x7FF) << 16);
	ISP_REG_WR(idx, ISP_RGB2YUV420_MATRIX3, val);
	val = (-21 & 0x7FF);
	ISP_REG_WR(idx, ISP_RGB2YUV420_MATRIX4, val);
	ISP_REG_MWR(idx, ISP_RGB2YUV420_SHIFT0, 0X7FF0000, 0 << 16);
	ISP_REG_MWR(idx, ISP_RGB2YUV420_SHIFT0, 0X7FF, 0);
	ISP_REG_MWR(idx, ISP_RGB2YUV420_SHIFT1, 0X7FF, 0);

	pr_debug("end\n");

	return 0;
}

static int isphw_path_store(void *handle, void *arg)
{
	int ret = 0;
	uint32_t val = 0;
	uint32_t idx = 0;
	struct isp_hw_path_store *path_store = NULL;
	struct isp_store_info *store_info = NULL;
	unsigned long addr = 0;

	path_store = (struct isp_hw_path_store *)arg;
	idx = path_store->ctx_id;
	store_info = &path_store->store;
	addr = store_base[path_store->spath_id];

	pr_debug("isp set store in.  bypass %d, path_id:%d, w:%d,h:%d, bwd:%d\n",
			store_info->bypass, path_store->spath_id,
			store_info->size.w, store_info->size.h, store_info->need_bwd);

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, BIT_0, store_info->bypass);
	if (store_info->bypass)
		return 0;

	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, BIT_1, (store_info->max_len_sel << 1));
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, BIT_2, (store_info->speed_2x << 2));
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, BIT_3, (store_info->mirror_en << 3));
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, 0xF0, (store_info->color_fmt << 4));
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, 0x300, (store_info->endian << 8));
	ISP_REG_MWR(idx, addr + ISP_STORE_PARAM, BIT_13, 1 << 13);

	if (store_info->need_bwd) {
		if (path_store->spath_id == ISP_SPATH_CP) {
			ISP_REG_MWR(idx, ISP_CAP_SCALER_BWD_PARA, BIT_0, 0);
			ISP_REG_MWR(idx, ISP_CAP_SCALER_BWD_PARA, BIT_4, BIT_4);
		} else if (path_store->spath_id == ISP_SPATH_VID) {
			ISP_REG_MWR(idx, ISP_RECORD_SCALER_BWD_PARA, BIT_0, 0);
			ISP_REG_MWR(idx, ISP_RECORD_SCALER_BWD_PARA, BIT_4, BIT_4);
		} else if (path_store->spath_id == ISP_SPATH_FD){
			ISP_REG_MWR(idx, ISP_THMB_SCL_BWD_PARA , BIT_0, 0);
			ISP_REG_MWR(idx, ISP_THMB_SCL_BWD_PARA , BIT_4, BIT_4);
		}
	} else {
		if (path_store->spath_id == ISP_SPATH_CP)
			ISP_REG_MWR(idx, ISP_CAP_SCALER_BWD_PARA, BIT_0, 1);
		else if (path_store->spath_id == ISP_SPATH_VID)
			ISP_REG_MWR(idx, ISP_RECORD_SCALER_BWD_PARA, BIT_0, 1);
		else if (path_store->spath_id == ISP_SPATH_FD)
			ISP_REG_MWR(idx, ISP_THMB_SCL_BWD_PARA, BIT_0, 1);
	}

	val = ((store_info->size.h & 0xFFFF) << 16) | (store_info->size.w & 0xFFFF);
	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_SIZE, val);

	ISP_REG_WR(idx, addr + ISP_STORE_BORDER, 0);
	ISP_REG_WR(idx, addr + ISP_STORE_Y_PITCH, store_info->pitch.pitch_ch0);
	ISP_REG_WR(idx, addr + ISP_STORE_U_PITCH, store_info->pitch.pitch_ch1);
	ISP_REG_WR(idx, addr + ISP_STORE_V_PITCH, store_info->pitch.pitch_ch2);

	pr_debug("set_store size %d %d\n", store_info->size.w, store_info->size.h);
	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CTRL, 0x3, store_info->rd_ctrl);
	ISP_REG_MWR(idx, addr + ISP_STORE_READ_CTRL, 0xFFFFFFFC, store_info->store_res << 2);
	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR_SEL, BIT_1, store_info->shadow_clr_sel << 1);
	ISP_REG_MWR(idx, addr + ISP_STORE_SHADOW_CLR, BIT_0, store_info->shadow_clr);

	return ret;
}

static void isphw_path_shrink_info_set(
			uint32_t idx, unsigned long  scaler_base,
			struct isp_regular_info *regular_info)
{
	unsigned long addr = 0;
	uint32_t reg_val = 0;

	pr_debug("regular_mode %d\n", regular_info->regular_mode);
	addr = ISP_SCALER_CFG + scaler_base;
	ISP_REG_MWR(idx, addr, (BIT_25 | BIT_26),
		regular_info->regular_mode << 25);

	/*TBD
	 * the value need to update.
	 */
	if (regular_info->regular_mode == DCAM_REGULAR_SHRINK) {
		regular_info->shrink_y_up_th = SHRINK_Y_UP_TH;
		regular_info->shrink_y_dn_th = SHRINK_Y_DN_TH;
		regular_info->shrink_uv_up_th = SHRINK_UV_UP_TH;
		regular_info->shrink_uv_dn_th = SHRINK_UV_DN_TH;
		if (scaler_base == ISP_SCALER_VID_BASE) {
			addr = ISP_RECORD_SCALER_SHRINK_CFG_Y;
			reg_val = ((regular_info->shrink_y_dn_th  & 0x3FF) << 16) |
				((regular_info->shrink_y_up_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_RECORD_SCALER_SHRINK_CFG_UV;
			reg_val = ((regular_info->shrink_uv_dn_th & 0x3FF) << 16) |
				(regular_info->shrink_uv_up_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);

			regular_info->shrink_y_offset = SHRINK_Y_OFFSET;
			regular_info->shrink_y_range = SHRINK_Y_RANGE;
			regular_info->shrink_c_offset = SHRINK_C_OFFSET;
			regular_info->shrink_c_range = SHRINK_C_RANGE;
			addr = ISP_RECORD_SCALER_REGULAR_CFG;
			reg_val = ((regular_info->shrink_c_range & 0xF) << 24) |
				((regular_info->shrink_c_offset & 0x7F) << 16);
			reg_val |= ((regular_info->shrink_y_range & 0xF) << 8) |
				(regular_info->shrink_y_offset & 0x7F);
			ISP_REG_WR(idx, addr, reg_val);
		}else if (scaler_base == ISP_SCALER_THUMB_BASE) {
			addr = ISP_THMB_SCALER_SHRINK_CFG_Y;
			reg_val = ((regular_info->shrink_y_dn_th  & 0x3FF) << 16) |
				((regular_info->shrink_y_up_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_THMB_SCALER_SHRINK_CFG_UV;
			reg_val = ((regular_info->shrink_uv_dn_th & 0x3FF) << 16) |
				(regular_info->shrink_uv_up_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);

			regular_info->shrink_y_offset = SHRINK_Y_OFFSET;
			regular_info->shrink_y_range = SHRINK_Y_RANGE;
			regular_info->shrink_c_offset = SHRINK_C_OFFSET;
			regular_info->shrink_c_range = SHRINK_C_RANGE;
			addr = ISP_THMB_SCALER_REGULAR_CFG;
			reg_val = ((regular_info->shrink_c_range & 0xF) << 24) |
				((regular_info->shrink_c_offset & 0x7F) << 16);
			reg_val |= ((regular_info->shrink_y_range & 0xF) << 8) |
				(regular_info->shrink_y_offset & 0x7F);
			ISP_REG_WR(idx, addr, reg_val);
		}
	} else if (regular_info->regular_mode == DCAM_REGULAR_CUT) {
		if (scaler_base == ISP_SCALER_VID_BASE) {
			addr = ISP_RECORD_SCALER_SHRINK_CFG_Y;
			reg_val = ((regular_info->shrink_y_dn_th  & 0x3FF) << 16) |
				((regular_info->shrink_y_up_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_RECORD_SCALER_SHRINK_CFG_UV;
			reg_val = ((regular_info->shrink_uv_dn_th & 0x3FF) << 16) |
				(regular_info->shrink_uv_up_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);
		} else if (scaler_base == ISP_SCALER_THUMB_BASE) {
			addr = ISP_THMB_SCALER_SHRINK_CFG_Y;
			reg_val = ((regular_info->shrink_y_dn_th  & 0x3FF) << 16) |
				((regular_info->shrink_y_up_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_THMB_SCALER_SHRINK_CFG_UV;
			reg_val = ((regular_info->shrink_uv_dn_th & 0x3FF) << 16) |
				(regular_info->shrink_uv_up_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);
		}
	} else if (regular_info->regular_mode == DCAM_REGULAR_EFFECT) {
		if (scaler_base == ISP_SCALER_VID_BASE) {
			addr = ISP_RECORD_SCALER_EFFECT_CFG_Y;
			reg_val = (regular_info->effect_y_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_RECORD_SCALER_EFFECT_CFG_UV;
			reg_val = (((regular_info->effect_v_th & 0x3FF) << 16) |
					(regular_info->effect_u_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);
		} else if(scaler_base == ISP_SCALER_THUMB_BASE) {
			addr = ISP_THMB_SCALER_EFFECT_CFG_Y;
			reg_val = (regular_info->effect_y_th & 0x3FF);
			ISP_REG_WR(idx, addr, reg_val);

			addr = ISP_THMB_SCALER_EFFECT_CFG_UV;
			reg_val = (((regular_info->effect_v_th & 0x3FF) << 16) |
					(regular_info->effect_u_th & 0x3FF));
			ISP_REG_WR(idx, addr, reg_val);
		}
	} else
		pr_debug("regular_mode %d\n", regular_info->regular_mode);
}

static int isphw_path_scaler_coeff_set(
			uint32_t idx, unsigned long  scaler_base,
			uint32_t *coeff_buf,
			uint32_t spath_id)
{
	int i = 0;
	int k = 0;
	int c = 0;
	int rtn = 0;
	uint32_t *tmp_h_coeff = NULL;
	uint32_t *tmp_h_chroma_coeff = NULL;
	struct coeff_arg arg;

	arg.h_coeff = coeff_buf;
	arg.h_chroma_coeff = coeff_buf + (ISP_SC_COEFF_COEF_SIZE / 4);
	arg.v_coeff = coeff_buf + (ISP_SC_COEFF_COEF_SIZE * 2 / 4);
	arg.v_chroma_coeff = coeff_buf + (ISP_SC_COEFF_COEF_SIZE * 3 / 4);

	if (spath_id == ISP_SPATH_CP)  {
		arg.v_coeff_addr = ISP_CAP_PATH_VER_CORF_Y_BUF0_CH0;
		arg.v_chroma_coeff_addr = ISP_CAP_PATH_VER_CORF_UV_BUF0_CH0;
	} else if (spath_id == ISP_SPATH_VID) {
		arg.v_coeff_addr = ISP_VID_PATH_VER_CORF_Y_BUF0_CH0;
		arg.v_chroma_coeff_addr = ISP_VID_PATH_VER_CORF_UV_BUF0_CH0;
	}

	for (k = 0; k < 4; k++) {
		if (spath_id == ISP_SPATH_CP)  {
			if (k == 0) {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_CAP_SCL_COEF_ADDR1 ;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_CAP_SCL_COEF_ADDR1;
			} else if (k ==1) {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_CAP_SCL_COEF1_ADDR1;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_CAP_SCL_COEF1_ADDR1;
			} else if (k == 2){
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_CAP_SCL_COEF2_ADDR1;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_CAP_SCL_COEF2_ADDR1;
			} else {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_CAP_SCL_COEF3_ADDR1;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_CAP_SCL_COEF3_ADDR1;
			}
		} else if (spath_id == ISP_SPATH_VID) {
			if(k == 0) {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_VID_SCL_COEF_ADDR2;
				arg.h_chroma_coeff_addr = ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_VID_SCL_COEF_ADDR2;
			} else if(k ==1) {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_VID_SCL_COEF1_ADDR2;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_VID_SCL_COEF1_ADDR2;
			} else if(k == 2){
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_VID_SCL_COEF2_ADDR2;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_VID_SCL_COEF2_ADDR2;
			} else {
				arg.h_coeff_addr = ISP_SCL_PATH_HOR_CORF_Y + ISP_YUV_VID_SCL_COEF3_ADDR2;
				arg.h_chroma_coeff_addr =  ISP_SCL_PATH_HOR_CORF_UV + ISP_YUV_VID_SCL_COEF3_ADDR2;
			}
		}

		/*h y*/
		tmp_h_coeff = arg.h_coeff + k * COEF_HOR_Y_SIZE;
		for (c = 0; c < COEF_HOR_Y_SIZE; c++) {
			ISP_REG_WR(idx, arg.h_coeff_addr + c * 4, *(tmp_h_coeff));
			tmp_h_coeff++;
		}
		/*h uv*/
		tmp_h_chroma_coeff = arg.h_chroma_coeff + k * COEF_HOR_UV_SIZE;
		for (c = 0; c < COEF_HOR_UV_SIZE; c++) {
			ISP_REG_WR(idx, arg.h_chroma_coeff_addr + c * 4, *(tmp_h_chroma_coeff));
			tmp_h_chroma_coeff++;
		}
	}

	for (i = 0; i < COEF_VOR_Y_SIZE; i++) {
		ISP_REG_WR(idx, arg.v_coeff_addr, *arg.v_coeff);
		arg.v_coeff_addr += 4;
		arg.v_coeff++;
	}

	for (i = 0; i < COEF_VOR_UV_SIZE; i++) {
		ISP_REG_WR(idx, arg.v_chroma_coeff_addr, *arg.v_chroma_coeff);
		arg.v_chroma_coeff_addr += 4;
		arg.v_chroma_coeff++;
	}

	return rtn;
}

static int isphw_path_scaler(void *handle, void *arg)
{
	uint32_t reg_val, idx;
	struct isp_hw_path_scaler *path_scaler = NULL;
	struct yuv_scaler_info *scalerInfo = NULL;
	struct img_deci_info *deciInfo = NULL;
	unsigned long addr;
	uint32_t path_mask[ISP_SPATH_NUM] = {
		BIT_1 | BIT_0,
		BIT_3 | BIT_2,
		BIT_5 | BIT_4
	};
	uint32_t path_off[ISP_SPATH_NUM] = {0, 2, 4};

	path_scaler = (struct isp_hw_path_scaler *)arg;
	scalerInfo = &path_scaler->scaler;
	idx = path_scaler->ctx_id;
	deciInfo = &path_scaler->deci;
	addr = scaler_base[path_scaler->spath_id];

	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
		path_mask[path_scaler->spath_id],
		(0 << path_off[path_scaler->spath_id]));

	/* set path_eb*/
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_31, 1 << 31);
	/* CLK_SWITCH*/
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_30, 0 << 30);
	/* sw_switch_en*/
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_29, 0 << 29);
	/* bypass all scaler */
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_0, 0);
	/* scaler path stop */
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_2, 0);

	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, 0x1f << 20, (path_scaler->frm_deci & 0x1f) << 20);

	/*set X/Y deci */
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_6, deciInfo->deci_x_eb << 6);
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, (BIT_4 | BIT_5), deciInfo->deci_x << 4);
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_9, deciInfo->deci_y_eb << 9);
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, (BIT_7 | BIT_8), deciInfo->deci_y << 7);

	/*src size*/
	ISP_REG_WR(idx, addr + ISP_SCALER_SRC_SIZE,
			((path_scaler->src.h & 0x3FFF) << 16) |
				(path_scaler->src.w & 0x3FFF));
	/* trim0 */
	ISP_REG_WR(idx, addr + ISP_SCALER_TRIM0_START,
				((path_scaler->in_trim.start_y & 0x3FFF) << 16) |
					(path_scaler->in_trim.start_x & 0x3FFF));
	ISP_REG_WR(idx, addr + ISP_SCALER_TRIM0_SIZE,
				((path_scaler->in_trim.size_y & 0x3FFF) << 16) |
					(path_scaler->in_trim.size_x & 0x3FFF));
	/* trim1 */
	ISP_REG_WR(idx, addr + ISP_SCALER_TRIM1_START,
				((path_scaler->out_trim.start_y & 0x3FFF) << 16) |
					(path_scaler->out_trim.start_x & 0x3FFF));
	ISP_REG_WR(idx, addr + ISP_SCALER_TRIM1_SIZE,
				((path_scaler->out_trim.size_y & 0x3FFF) << 16) |
					(path_scaler->out_trim.size_x & 0x3FFF));
	/* des size */
	ISP_REG_WR(idx, addr + ISP_SCALER_DES_SIZE,
				((path_scaler->dst.h & 0x3FFF) << 16) |
					(path_scaler->dst.w & 0x3FFF));

	pr_debug("sw %d, path src: %d %d; in_trim:%d %d %d %d, out_trim: %d %d %d %d, dst: %d %d \n",
		idx, path_scaler->src.w, path_scaler->src.h, path_scaler->in_trim.start_x,
		path_scaler->in_trim.start_y, path_scaler->in_trim.size_x, path_scaler->in_trim.size_y,
		path_scaler->out_trim.start_x, path_scaler->out_trim.start_y, path_scaler->out_trim.size_x,
		path_scaler->out_trim.size_y, path_scaler->dst.w, path_scaler->dst.h);

	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_1,
			scalerInfo->scaler_bypass << 1);
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, 0xF0000,
			scalerInfo->scaler_y_ver_tap << 16);
	ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, 0xF800,
			scalerInfo->scaler_uv_ver_tap << 11);

	reg_val = ((scalerInfo->scaler_ip_int & 0x1F) << 16) |
			(scalerInfo->scaler_ip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_IP, reg_val);
	reg_val = ((scalerInfo->scaler_cip_int & 0x1F) << 16) |
			(scalerInfo->scaler_cip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_CIP, reg_val);
	reg_val = ((scalerInfo->scaler_factor_in & 0x3FFF) << 16) |
			(scalerInfo->scaler_factor_out & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_FACTOR, reg_val);

	reg_val = ((scalerInfo->scaler_ver_ip_int & 0x1F) << 16) |
				(scalerInfo->scaler_ver_ip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_VER_IP, reg_val);
	reg_val = ((scalerInfo->scaler_ver_cip_int & 0x1F) << 16) |
				(scalerInfo->scaler_ver_cip_rmd & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_VER_CIP, reg_val);
	reg_val = ((scalerInfo->scaler_ver_factor_in & 0x3FFF) << 16) |
				(scalerInfo->scaler_ver_factor_out & 0x3FFF);
	ISP_REG_WR(idx, addr + ISP_SCALER_FACTOR_VER, reg_val);

	if ((path_scaler->in_trim.size_x == path_scaler->src.w) &&
		(path_scaler->in_trim.size_y == path_scaler->src.h) &&
		(path_scaler->in_trim.size_x == path_scaler->out_trim.size_x) &&
		(path_scaler->in_trim.size_y == path_scaler->out_trim.size_y) &&
		scalerInfo->scaler_bypass) {
		ISP_REG_MWR(idx, addr + ISP_SCALER_CFG, BIT_0, 1);
		pr_debug("scaler all bypass\n");
	}

	pr_debug("set_scale_info in %d %d out %d %d\n",
		scalerInfo->scaler_factor_in,
		scalerInfo->scaler_ver_factor_in,
		scalerInfo->scaler_factor_out,
		scalerInfo->scaler_ver_factor_out);

	if (!scalerInfo->scaler_bypass)
		isphw_path_scaler_coeff_set(idx,
			addr, scalerInfo->coeff_buf, path_scaler->spath_id);

	if (path_scaler->spath_id == ISP_SPATH_VID)
		isphw_path_shrink_info_set(idx, addr, &path_scaler->regular_info);

	return 0;
}

static int isphw_path_thumbscaler(void *handle, void *arg)
{
	uint32_t val, idx;
	struct isp_hw_thumbscaler_info *scalerInfo = NULL;

	scalerInfo =(struct isp_hw_thumbscaler_info *)arg;
	idx = scalerInfo->idx;

	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_5 | BIT_4, (0 << 4));
	ISP_REG_MWR(idx, ISP_THMB_SCALER_CFG, BIT_0, (scalerInfo->scaler_bypass & 0x1));

	val = ((scalerInfo->frame_deci & 0x3) << 2) |
		((scalerInfo->odata_mode & 0x3) << 4) |
		((scalerInfo->y_deci.deci_x & 0x3) << 16) |
		((scalerInfo->y_deci.deci_x_eb & 0x1) << 19) |
		((scalerInfo->y_deci.deci_y & 0x3) << 20) |
		((scalerInfo->y_deci.deci_y_eb & 0x1) << 23) |
		((scalerInfo->uv_deci.deci_x & 0x3) << 24) |
		((scalerInfo->uv_deci.deci_x_eb & 0x1) << 27) |
		((scalerInfo->uv_deci.deci_y & 0x3) << 28) |
		((scalerInfo->uv_deci.deci_y_eb & 0x1) << 31);
	ISP_REG_MWR(idx, ISP_THMB_SCALER_CFG, 0xBBBB003C, val);

	val = ((scalerInfo->y_factor_in.w & 0x1FFF) << 16) |
		(scalerInfo->y_factor_out.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_FACTOR_HOR, val);

	val = ((scalerInfo->y_factor_in.h & 0x1FFF) << 16) |
		(scalerInfo->y_factor_out.h & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_FACTOR_VER, val);

	val = ((scalerInfo->uv_factor_in.w & 0x1FFF) << 16) |
		(scalerInfo->uv_factor_out.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_FACTOR_HOR, val);

	val = ((scalerInfo->uv_factor_in.h & 0x1FFF) << 16) |
		(scalerInfo->uv_factor_out.h & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_FACTOR_VER, val);

	val = ((scalerInfo->src0.h & 0x1FFF) << 16) |
		(scalerInfo->src0.w & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_BEFORE_TRIM_SIZE, val);

	val = ((scalerInfo->y_src_after_deci.h & 0x1FFF) << 16) |
		(scalerInfo->y_src_after_deci.w & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_SLICE_SRC_SIZE, val);

	val = ((scalerInfo->y_dst_after_scaler.h & 0x3FF) << 16) |
		(scalerInfo->y_dst_after_scaler.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_DES_SIZE, val);

	val = ((scalerInfo->y_trim.start_y & 0x1FFF) << 16) |
		(scalerInfo->y_trim.start_x & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_TRIM0_START, val);

	val = ((scalerInfo->y_trim.size_y & 0x1FFF) << 16) |
		(scalerInfo->y_trim.size_x & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_TRIM0_SIZE, val);

	val = ((scalerInfo->y_init_phase.h & 0x3FF) << 16) |
		(scalerInfo->y_init_phase.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_Y_INIT_PHASE, val);

	val = ((scalerInfo->uv_src_after_deci.h & 0x1FFF) << 16) |
		(scalerInfo->uv_src_after_deci.w & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_SLICE_SRC_SIZE, val);

	val = ((scalerInfo->uv_dst_after_scaler.h & 0x3FF) << 16) |
		(scalerInfo->uv_dst_after_scaler.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_DES_SIZE, val);

	val = ((scalerInfo->uv_trim.start_y & 0x1FFF) << 16) |
		(scalerInfo->uv_trim.start_x & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_TRIM0_START, val);

	val = ((scalerInfo->uv_trim.size_y & 0x1FFF) << 16) |
		(scalerInfo->uv_trim.size_x & 0x1FFF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_TRIM0_SIZE, val);

	val = ((scalerInfo->uv_init_phase.h & 0x3FF) << 16) |
		(scalerInfo->uv_init_phase.w & 0x3FF);
	ISP_REG_WR(idx, ISP_THMB_SCALER_UV_INIT_PHASE, val);

	/* bypass regular. */
	ISP_REG_MWR(idx, ISP_THMB_SCALER_CFG, BIT_12 | BIT_13, 0 << 12);

	return 0;
}

static int isphw_slice_scaler(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	uint32_t base = 0;
	struct isp_hw_slice_scaler *update = NULL;

	update = (struct isp_hw_slice_scaler *)arg;
	base = (uint32_t)scaler_base[update->spath_id];

	if (!update->path_en) {
		addr = ISP_SCALER_CFG + base;
		cmd = (0 << 31) | (1 << 2);
		ISP_REG_WR(update->ctx_id, addr, cmd);
		return 0;
	}

	/* bit31 enable path */
	addr = ISP_SCALER_CFG + base;
	cmd = ISP_REG_RD(update->ctx_id, base + ISP_SCALER_CFG);
	cmd |= (1 << 31);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_SRC_SIZE + base;
	cmd = (update->slc_scaler->src_size_x & 0x3FFF) |
			((update->slc_scaler->src_size_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_DES_SIZE + base;
	cmd = (update->slc_scaler->dst_size_x & 0x3FFF) |
			((update->slc_scaler->dst_size_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_TRIM0_START + base;
	cmd = (update->slc_scaler->trim0_start_x & 0x3FFF) |
			((update->slc_scaler->trim0_start_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_TRIM0_SIZE + base;
	cmd = (update->slc_scaler->trim0_size_x & 0x3FFF) |
			((update->slc_scaler->trim0_size_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_IP + base;
	cmd = (update->slc_scaler->scaler_ip_rmd & 0x3FFF) |
			((update->slc_scaler->scaler_ip_int & 0x1F) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_CIP + base;
	cmd = (update->slc_scaler->scaler_cip_rmd & 0x3FFF) |
			((update->slc_scaler->scaler_cip_int & 0x1F) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_TRIM1_START + base;
	cmd = (update->slc_scaler->trim1_start_x & 0x3FFF) |
			((update->slc_scaler->trim1_start_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_TRIM1_SIZE + base;
	cmd = (update->slc_scaler->trim1_size_x & 0x3FFF) |
			((update->slc_scaler->trim1_size_y & 0x3FFF) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_VER_IP + base;
	cmd = (update->slc_scaler->scaler_ip_rmd_ver & 0x3FFF) |
			((update->slc_scaler->scaler_ip_int_ver & 0x1F) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	addr = ISP_SCALER_VER_CIP + base;
	cmd = (update->slc_scaler->scaler_cip_rmd_ver & 0x3FFF) |
			((update->slc_scaler->scaler_cip_int_ver & 0x1F) << 16);
	ISP_REG_WR(update->ctx_id, addr, cmd);

	return 0;
}

static int isphw_slice_store(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	uint32_t base = 0;
	struct isp_hw_slice_store *store = NULL;

	store = (struct isp_hw_slice_store *)arg;
	base = (uint32_t)store_base[store->spath_id];

	if (!store->path_en) {
		/* bit0 bypass store */
		addr = ISP_STORE_PARAM + base;
		cmd = 1;
		ISP_REG_WR(store->ctx_id, addr, cmd);
		return 0;
	}
	addr = ISP_STORE_PARAM + base;
	cmd = ISP_REG_RD(store->ctx_id, base + ISP_STORE_PARAM) & ~1;
	ISP_REG_WR(store->ctx_id, addr, cmd);

	addr = ISP_STORE_SLICE_SIZE + base;
	cmd = ((store->slc_store->size.h & 0xFFFF) << 16) |
			(store->slc_store->size.w & 0xFFFF);
	ISP_REG_WR(store->ctx_id, addr, cmd);

	addr = ISP_STORE_BORDER + base;
	cmd = (store->slc_store->border.left_border & 0xFFFF) |
			((store->slc_store->border.right_border & 0xFFFF) << 16);
	ISP_REG_WR(store->ctx_id, addr, cmd);

	addr = ISP_STORE_BORDER_1+ base;
	cmd = (store->slc_store->border.up_border & 0xFFFF) |
			((store->slc_store->border.down_border & 0xFFFF) << 16);
	ISP_REG_WR(store->ctx_id, addr, cmd);

	addr = ISP_STORE_SLICE_Y_ADDR + base;
	cmd = store->slc_store->addr.addr_ch0;
	ISP_REG_WR(store->ctx_id, addr, cmd);

	addr = ISP_STORE_SLICE_U_ADDR + base;
	cmd = store->slc_store->addr.addr_ch1;
	ISP_REG_WR(store->ctx_id, addr, cmd);

	return 0;
}

static struct isp_cfg_entry isp_hw_cfg_func_tab[ISP_BLOCK_TOTAL - ISP_BLOCK_BASE] = {
[ISP_BLOCK_BCHS - ISP_BLOCK_BASE]     = {ISP_BLOCK_BCHS,     isp_k_cfg_bchs},
[ISP_BLOCK_YGAMMA - ISP_BLOCK_BASE]   = {ISP_BLOCK_YGAMMA,   isp_k_cfg_ygamma},
[ISP_BLOCK_YNR - ISP_BLOCK_BASE]      = {ISP_BLOCK_YNR,      isp_k_cfg_ynr},
[ISP_BLOCK_RGB_LTM - ISP_BLOCK_BASE]  = {ISP_BLOCK_RGB_LTM,  isp_k_cfg_rgb_ltm},
[ISP_BLOCK_UVD - ISP_BLOCK_BASE]      = {ISP_BLOCK_UVD,      isp_k_cfg_uvd},
[ISP_BLOCK_CDN - ISP_BLOCK_BASE]      = {ISP_BLOCK_CDN,      isp_k_cfg_cdn},
[ISP_BLOCK_HSV - ISP_BLOCK_BASE]      = {ISP_BLOCK_HSV,      isp_k_cfg_hsv},
[ISP_BLOCK_EDGE - ISP_BLOCK_BASE]     = {ISP_BLOCK_EDGE,     isp_k_cfg_edge},
[ISP_BLOCK_IIRCNR - ISP_BLOCK_BASE]   = {ISP_BLOCK_IIRCNR,   isp_k_cfg_iircnr},
[ISP_BLOCK_PRE_CDN - ISP_BLOCK_BASE]  = {ISP_BLOCK_PRE_CDN,  isp_k_cfg_pre_cdn},
[ISP_BLOCK_POST_CDN - ISP_BLOCK_BASE] = {ISP_BLOCK_POST_CDN, isp_k_cfg_post_cdn},
[ISP_BLOCK_YRANDOM - ISP_BLOCK_BASE]  = {ISP_BLOCK_YRANDOM,  isp_k_cfg_yrandom},
[ISP_BLOCK_3DLUT - ISP_BLOCK_BASE]    = {ISP_BLOCK_3DLUT,    isp_k_cfg_3dlut},
};

static int isphw_block_func_get(void *handle, void *arg)
{
	void *block_func = NULL;
	struct isp_hw_block_func *func_arg = NULL;

	func_arg = (struct isp_hw_block_func *)arg;

	if (func_arg->index < (ISP_BLOCK_TOTAL - ISP_BLOCK_BASE)) {
		block_func = (struct isp_cfg_entry *)&isp_hw_cfg_func_tab[func_arg->index];
		func_arg->isp_entry = block_func;
	}

	if (block_func == NULL)
		pr_err("fail to get valid block func %d\n", ISP_BLOCK_TYPE);

	return 0;
}

static isp_k_blk_func isp_hw_k_blk_func_tab[ISP_K_BLK_MAX] = {
	[ISP_K_BLK_LTM] = isp_ltm_config_param,
	[ISP_K_BLK_PYR_REC_SHARE] = isp_pyr_rec_share_config,
	[ISP_K_BLK_PYR_REC_FRAME] = isp_pyr_rec_frame_config,
	[ISP_K_BLK_PYR_REC_SLICE] = isp_pyr_rec_slice_config,
	[ISP_K_BLK_PYR_REC_SLICE_COMMON] = isp_pyr_rec_slice_common_config,
	[ISP_K_BLK_DEWARP_CACHE_CFG] = isp_dewarping_dewarp_cache_set,
	[ISP_K_BLK_DEWARP_CFG] = isp_dewarping_frame_config,
	[ISP_K_BLK_DEWARP_SLICE] = isp_k_dewarping_slice_config,
	[ISP_K_BLK_PYR_DEC_IRQ_FUNC] = isp_pyr_dec_irq_func,
	[ISP_K_BLK_PYR_DEC_CFG] = isp_pyr_dec_config,
	[ISP_K_BLK_YNR_UPDATE] = isp_k_update_ynr,
};

static int isphw_k_blk_func_get(void *handle, void *arg)
{
	struct isp_hw_k_blk_func *func_arg = NULL;

	func_arg = (struct isp_hw_k_blk_func *)arg;

	if (func_arg->index < ISP_K_BLK_MAX)
		func_arg->k_blk_func = isp_hw_k_blk_func_tab[func_arg->index];

	return 0;
}

static int isphw_fetch_set(void *handle, void *arg)
{
	uint32_t idx = 0, val = 0x7;
	struct isp_hw_fetch_info *fetch = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	fetch = (struct isp_hw_fetch_info *)arg;
	idx = fetch->ctx_id;

	pr_debug("enter: fmt:%d, w:%d, h:%d, data_bits:%d, is_pack:%d\n", fetch->fetch_fmt,
			fetch->in_trim.size_x, fetch->in_trim.size_y, fetch->data_bits, fetch->is_pack);

	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, BIT_1 | BIT_0, fetch->dispatch_color);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_10 | BIT_11, fetch->fetch_path_sel << 10);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_8, 0 << 8);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_5 | BIT_4, 3 << 4);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_3 | BIT_2, 3 << 2);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL, BIT_1 | BIT_0, 3);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL, BIT_0, 1);

	switch (fetch->fetch_fmt) {
	case ISP_FETCH_YVU420_2FRAME_10:
	case ISP_FETCH_YVU420_2FRAME_MIPI:
		val = 1;
		break;
	case ISP_FETCH_YVU420_2FRAME:
		val = 3;
		break;
	case ISP_FETCH_YUV420_2FRAME_10:
	case ISP_FETCH_YUV420_2FRAME_MIPI:
		val = 0;
		break;
	case ISP_FETCH_YUV420_2FRAME:
		val = 2;
		break;
	case ISP_FETCH_FULL_RGB10:
		val = 4;
		break;
	default:
		pr_err("fail to get isp fetch format:%d, val:%d\n", fetch->fetch_fmt, val);
		break;
	}

	ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_PARAM0, 0x70, val << 4 );
	ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_MIPI_PARAM, BIT_20, fetch->is_pack << 20);

	ISP_REG_WR(idx, ISP_DISPATCH_BASE + ISP_DISPATCH_CH0_SIZE,
		fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));

	ISP_REG_WR(idx, ISP_YDELAY_STEP, fetch->in_trim.size_y << 16);

	pr_debug("camca, isp sec mode=%d , pack_bits=%d, pitch_ch0=0x%x, 0x%x, 0x%x\n",
		fetch->sec_mode,
		fetch->pack_bits,
		fetch->pitch.pitch_ch0,
		fetch->pitch.pitch_ch1,
		fetch->pitch.pitch_ch2);

	ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_PARAM0, BIT_0, 0);
	ISP_REG_WR(idx, ISP_FETCH_BASE + ISP_FETCH_MEM_SLICE_SIZE,
			fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	if (fetch->sec_mode == SEC_SPACE_PRIORITY) {
		cam_trusty_isp_pitch_set(fetch->pitch.pitch_ch0,
			fetch->pitch.pitch_ch1,
			fetch->pitch.pitch_ch2);
	} else {
		ISP_REG_WR(idx, ISP_FETCH_BASE + ISP_FETCH_SLICE_Y_PITCH, fetch->pitch.pitch_ch0);
		ISP_REG_WR(idx, ISP_FETCH_BASE + ISP_FETCH_SLICE_U_PITCH, fetch->pitch.pitch_ch1);
	}

	if (fetch->is_pack) {
		val = (fetch->mipi_byte_rel_pos << 16) | fetch->mipi_word_num;
		ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_MIPI_PARAM, 0xFFFFF, val);
		val = (fetch->mipi_byte_rel_pos_uv << 16) | fetch->mipi_word_num_uv;
		ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_MIPI_PARAM_UV, 0xFFFFF, val);
	}

	ISP_REG_WR(idx, ISP_3DNR_MEM_CTRL_PARAM2, fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	ISP_REG_WR(idx, ISP_3DNR_MEM_CTRL_PARAM3, fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	ISP_REG_WR(idx, ISP_3DNR_MEM_CTRL_PARAM4, fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	ISP_REG_WR(idx, ISP_3DNR_MEM_CTRL_PARAM5, fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	pr_debug("end\n");
	return 0;
}

static int isphw_fetch_fbd_set(void *handle, void *arg)
{
	struct isp_fbd_yuv_info *fbd_yuv = NULL;
	uint32_t idx = 0;
	uint32_t afbc_mode = 0;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	fbd_yuv = (struct isp_fbd_yuv_info *)arg;
	idx = fbd_yuv->ctx_id;

	/* bypass normal fetch */
	ISP_REG_MWR(idx, ISP_FETCH_BASE + ISP_FETCH_PARAM0,
		    BIT(0), ~fbd_yuv->fetch_fbd_bypass);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL,
		    BIT(0), fbd_yuv->fetch_fbd_bypass);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL, BIT_1, 1 << 1);
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL, BIT_3, 1 << 3);

	if (fbd_yuv->data_bits == DCAM_STORE_8_BIT)
		afbc_mode = 5;
	else if (fbd_yuv->data_bits == DCAM_STORE_10_BIT)
		afbc_mode = 7;
	else
		pr_debug("No use afbc mode.\n");
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SEL, 0x1F0, afbc_mode << 4);

	ISP_REG_WR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SLICE_SIZE,
		   fbd_yuv->slice_size.w | (fbd_yuv->slice_size.h << 16));
	ISP_REG_MWR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_HBLANK_TILE_PITCH,
		   0x7FF0000, fbd_yuv->tile_num_pitch << 16);
	ISP_REG_WR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_PARAM0,
		   fbd_yuv->slice_start_pxl_xpt | (fbd_yuv->slice_start_pxl_ypt << 16));
	pr_debug("enable fbd: %d\n", !fbd_yuv->fetch_fbd_bypass);

	return 0;
}

static int isphw_fmcu_available(void *handle, void *arg)
{
	uint32_t fmcu_valid = 0;
	struct isp_hw_fmcu_sel *fmcu_sel = NULL;
	uint32_t reg_bits[ISP_CONTEXT_HW_NUM] = {
			ISP_CONTEXT_HW_P0, ISP_CONTEXT_HW_P1,
			ISP_CONTEXT_HW_C0, ISP_CONTEXT_HW_C1};
	uint32_t reg_offset[ISP_FMCU_NUM] = {
			ISP_COMMON_FMCU0_PATH_SEL,
			ISP_COMMON_FMCU1_PATH_SEL,
			ISP_COMMON_FMCU2_PATH_SEL};

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EINVAL;
	}

	fmcu_sel = (struct isp_hw_fmcu_sel *)arg;
	fmcu_valid = (fmcu_sel->fmcu_id > 2) ? 0 : 1;

	if (fmcu_valid)
		ISP_HREG_MWR(reg_offset[fmcu_sel->fmcu_id], BIT_1 | BIT_0,
			reg_bits[fmcu_sel->hw_idx]);
	pr_debug("fmcu C sel 0x%x\n", ISP_HREG_RD(ISP_COMMON_FMCU0_PATH_SEL));
	pr_debug("fmcu P0 sel 0x%x\n", ISP_HREG_RD(ISP_COMMON_FMCU1_PATH_SEL));
	pr_debug("fmcu P1 sel 0x%x\n", ISP_HREG_RD(ISP_COMMON_FMCU2_PATH_SEL));
	return fmcu_valid;
}

static int isphw_afbc_path_set(void *handle, void *arg)
{
	int ret = 0;
	uint32_t val = 0;
	uint32_t idx = 0;
	struct isp_hw_afbc_path *afbc_path = NULL;
	struct isp_afbc_store_info *afbc_store_info = NULL;
	unsigned long addr = 0;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EINVAL;
	}

	afbc_path = (struct isp_hw_afbc_path *)arg;
	idx = afbc_path->ctx_id;
	afbc_store_info = &afbc_path->afbc_store;
	addr = fbc_store_base[afbc_path->spath_id];

	pr_debug("isp set afbc store in. bypass %d, path_id:%d, w:%d,h:%d\n",
			afbc_store_info->bypass, afbc_path->spath_id,
			afbc_store_info->size.w, afbc_store_info->size.h);

	ISP_REG_MWR(idx, addr + ISP_FBC_STORE_PARAM,
		BIT_0, afbc_store_info->bypass);
	if (afbc_store_info->bypass)
		return 0;

	ISP_REG_MWR(idx, addr+ISP_FBC_STORE_PARAM,
		BIT_3, (afbc_store_info->mirror_en << 3));
	ISP_REG_MWR(idx, addr+ISP_FBC_STORE_PARAM,
		0xF0, (afbc_store_info->color_format << 4));
	ISP_REG_MWR(idx, addr+ISP_FBC_STORE_PARAM,
		0x300, (afbc_store_info->endian << 8));
	ISP_REG_WR(idx, addr+ISP_FBC_STORE_TILE_PITCH,
		(afbc_store_info->tile_number_pitch & 0x1FFF));
	ISP_REG_WR(idx, addr+ISP_FBC_STORE_NFULL_LEVEL, 0x20002);

	val = ((afbc_store_info->size.h & 0x1FFF) << 16) |
		(afbc_store_info->size.w & 0x1FFF);
	ISP_REG_WR(idx, addr+ISP_FBC_STORE_SLICE_SIZE, val);

	val = (afbc_store_info->border.up_border & 0xFF) |
		((afbc_store_info->border.down_border & 0xFF) << 8) |
		((afbc_store_info->border.left_border & 0xFF) << 16) |
		((afbc_store_info->border.right_border & 0xFF) << 24);
	ISP_REG_WR(idx, addr+ISP_FBC_STORE_BORDER, val);

	val = afbc_store_info->header_offset;
	ISP_REG_WR(idx, addr+ISP_FBC_STORE_SLICE_PAYLOAD_OFFSET_ADDR, val);
	ISP_REG_WR(idx, addr + ISP_FBC_STORE_SLICE_HEADER_BASE_ADDR, afbc_store_info->yheader);
	ISP_REG_WR(idx, addr + ISP_FBC_STORE_SLICE_PAYLOAD_BASE_ADDR, afbc_store_info->yaddr);

	return ret;
}

static int isphw_fbd_slice_set(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_fbd_slice *fbd_slice = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct slice_fbd_yuv_info *fbd_yuv_info = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EINVAL;
	}

	fbd_slice = (struct isp_hw_fbd_slice *)arg;
	fmcu = fbd_slice->fmcu_handle;
	fbd_yuv_info = fbd_slice->yuv_info;

	addr = ISP_GET_REG(ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_SLICE_SIZE);
	cmd = (fbd_yuv_info->slice_size.h << 16) | fbd_yuv_info->slice_size.w;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_PARAM0);
	cmd = (fbd_yuv_info->slice_start_pxl_ypt << 16) | fbd_yuv_info->slice_start_pxl_xpt;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_PARAM2);
	cmd = fbd_yuv_info->slice_start_header_addr;
	FMCU_PUSH(fmcu, addr, cmd);

	/* fetch normal path bypass */
	addr = ISP_GET_REG(ISP_FETCH_BASE + ISP_FETCH_PARAM0);
	cmd = 0x00000071;
	FMCU_PUSH(fmcu, addr, cmd);

	/* dispatch size same as fetch size */
	addr = ISP_GET_REG(ISP_DISPATCH_BASE + ISP_DISPATCH_CH0_SIZE);
	cmd = (fbd_yuv_info->slice_size.h << 16) | fbd_yuv_info->slice_size.w;
	FMCU_PUSH(fmcu, addr, cmd);

	pr_debug("pixel start: %u %u, size: %u %u, tile num: %u\n",
		fbd_yuv_info->slice_start_pxl_xpt,
		fbd_yuv_info->slice_start_pxl_ypt,
		fbd_yuv_info->slice_size.w,
		fbd_yuv_info->slice_size.h,
		fbd_yuv_info->tile_num_pitch);

	return 0;
}

static int  isphw_fbd_addr_set(void *handle, void *arg)
{
	uint32_t addr = 0;
	uint32_t idx = 0;
	struct isp_fbd_yuv_info *fbd_yuv = NULL;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}
	fbd_yuv = (struct isp_fbd_yuv_info *)arg;
	idx = fbd_yuv->ctx_id;

	addr = fbd_yuv->frame_header_base_addr;
	ISP_REG_WR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_PARAM1, addr);
	addr = fbd_yuv->slice_start_header_addr;
	ISP_REG_WR(idx, ISP_YUV_AFBD_FETCH_BASE + ISP_AFBD_FETCH_PARAM2, addr);

	pr_debug("addr0:%x, addr1:%x.\n", fbd_yuv->frame_header_base_addr, fbd_yuv->slice_start_header_addr);

	return 0;
}

static int isphw_afbc_fmcu_addr_set(void *handle, void *arg)
{
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_hw_afbc_fmcu_addr *parm = NULL;
	uint32_t addr = 0, cmd = 0;
	unsigned long afbc_base = 0;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	parm = (struct isp_hw_afbc_fmcu_addr *)arg;
	afbc_base = fbc_store_base[parm->index];
	fmcu = parm->fmcu;

	addr = ISP_GET_REG(ISP_FBC_STORE_SLICE_HEADER_BASE_ADDR) + afbc_base;
	cmd = parm->yheader;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBC_STORE_SLICE_PAYLOAD_BASE_ADDR) + afbc_base;
	cmd = parm->yaddr;
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int isphw_afbc_path_slice_set(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_afbc_path_slice *afbc_slice = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct slice_afbc_store_info *slc_afbc_store = NULL;
	unsigned long afbc_base = 0;
	unsigned long base = 0;

	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EINVAL;
	}

	afbc_slice = (struct isp_hw_afbc_path_slice *)arg;
	afbc_base = fbc_store_base[afbc_slice->spath_id];
	base = store_base[afbc_slice->spath_id];
	fmcu = afbc_slice->fmcu_handle;
	slc_afbc_store = afbc_slice->slc_afbc_store;

	if (!afbc_slice->path_en) {
		/* bit0 bypass store */
		addr = isphw_ap_fmcu_reg_get(fmcu, ISP_FBC_STORE_PARAM) + afbc_base;
		cmd = 1;
		isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);
		return 0;
	}

	addr = isphw_ap_fmcu_reg_get(fmcu, ISP_FBC_STORE_PARAM) + afbc_base;
	cmd = ISP_REG_RD(afbc_slice->ctx_idx, afbc_base + ISP_FBC_STORE_PARAM) & ~1;
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu, ISP_FBC_STORE_SLICE_SIZE) + afbc_base;
	cmd = ((slc_afbc_store->size.h & 0xFFFF) << 16) |
			(slc_afbc_store->size.w & 0xFFFF);
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu, ISP_FBC_STORE_BORDER) + afbc_base;
	cmd = (slc_afbc_store->border.up_border& 0xFF) |
			((slc_afbc_store->border.down_border& 0xFF) << 8) |
			((slc_afbc_store->border.left_border& 0xFF) << 16) |
			((slc_afbc_store->border.right_border & 0xFF) << 24);
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu,
		ISP_FBC_STORE_SLICE_PAYLOAD_OFFSET_ADDR) + afbc_base;
	cmd = slc_afbc_store->slice_offset;
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu,
		ISP_FBC_STORE_SLICE_PAYLOAD_OFFSET_ADDR) + afbc_base;
	cmd = slc_afbc_store->yheader_addr;
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu,
		ISP_FBC_STORE_SLICE_PAYLOAD_BASE_ADDR) + afbc_base;
	cmd = slc_afbc_store->yaddr;
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	addr = isphw_ap_fmcu_reg_get(fmcu, ISP_STORE_PARAM) + base;
	cmd = 1;
	isphw_ap_fmcu_reg_write(fmcu, afbc_slice->ctx_idx, addr, cmd);

	return 0;
}

static int isphw_ltm_slice_set(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0, base = 0;
	struct isp_hw_ltm_slice *ltm_slice = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct slice_ltm_map_info *map = NULL;

	ltm_slice = (struct isp_hw_ltm_slice *)arg;
	fmcu = ltm_slice->fmcu_handle;
	map = ltm_slice->map;

	if (map->bypass)
		return 0;

	switch (ltm_slice->ltm_id) {
	case LTM_RGB:
		base = ISP_LTM_MAP_RGB_BASE;
		break;
	default:
		pr_err("fail to get cmd id:%d, not supported.\n", ltm_slice->ltm_id);
		return -1;
	}

	addr = ISP_GET_REG(base + ISP_LTM_MAP_PARAM1);
	cmd = ((map->tile_num_y & 0x7) << 28) |
		((map->tile_num_x & 0x7) << 24) |
		((map->tile_height & 0x3FF) << 12) |
		(map->tile_width & 0x3FF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_LTM_MAP_PARAM3);
	cmd = ((map->tile_right_flag & 0x1) << 23) |
		((map->tile_start_y & 0x7FF) << 12) |
		((map->tile_left_flag & 0x1) << 11) |
		(map->tile_start_x & 0x7FF);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_LTM_MAP_PARAM4);
	cmd = map->mem_addr;
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_3dnr_fbc_store(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_nr3_fbc_slice *fbc_slice = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct slice_3dnr_fbc_store_info *fbc_store = NULL;

	fbc_slice = (struct isp_hw_nr3_fbc_slice *)arg;
	fmcu = fbc_slice->fmcu_handle;
	fbc_store = fbc_slice->fbc_store;

	if (fbc_store->bypass)
		return 0;

	addr = ISP_GET_REG(ISP_FBC_3DNR_PARAM);
	cmd = (fbc_store->bypass & 0x1) |
		((fbc_store->slice_mode_en & 0x1) << 1);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBC_3DNR_SLICE_SIZE);
	cmd = (fbc_store->fbc_size_in_hor & 0x1FFF) |
		((fbc_store->fbc_size_in_ver & 0x3FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBC_3DNR_SLICE_PLOAD_BASE_ADDR);
	cmd = fbc_store->fbc_y_tile_addr_init_x256;
	FMCU_PUSH(fmcu, addr, cmd);


	addr = ISP_GET_REG(ISP_FBC_3DNR_SLICE_HEADER_BASE_ADDR);
	cmd = fbc_store->fbc_y_header_addr_init;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBC_3DNR_TILE_PITCH);
	cmd = fbc_store->fbc_tile_number;
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_3dnr_fbd_fetch(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_nr3_fbd_slice *fbd_slice = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct slice_3dnr_fbd_fetch_info *fbd_fetch = NULL;

	fbd_slice = (struct isp_hw_nr3_fbd_slice *)arg;
	fmcu = fbd_slice->fmcu_handle;
	fbd_fetch = fbd_slice->fbd_fetch;

	addr = ISP_GET_REG(ISP_FBD_3DNR_PARAM1);
	cmd = fbd_fetch->fbd_y_header_addr_init;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBD_3DNR_PARAM2);
	cmd = (fbd_fetch->fbd_y_tiles_num_pitch & 0xFF) |
		((fbd_fetch->fbd_c_tile_addr_init_x256 & 0xFFFFFF) << 8);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBD_3DNR_SLICE_SIZE);
	cmd = (fbd_fetch->fbd_y_pixel_size_in_hor & 0xFFF) |
		((fbd_fetch->fbd_y_pixel_size_in_ver & 0x3FFF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FBD_3DNR_PARAM0);
	cmd = (fbd_fetch->fbd_y_pixel_start_in_ver & 0x3fff) |
		((fbd_fetch->fbd_y_pixel_start_in_hor & 0x3ffF) << 16);
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int isphw_subblock_cfg(void *handle, void *arg)
{
	return 0;
}

static int isphw_slw_fmcu_cmds(void *handle, void *arg)
{
	int i;
	unsigned long base, sbase;
	uint32_t ctx_idx;
	uint32_t reg_off, addr = 0, cmd = 0;
	struct isp_fmcu_ctx_desc *fmcu;
	struct isp_path_desc *path;
	struct img_addr *fetch_addr, *store_addr;
	struct cam_hw_info *hw = NULL;
	struct isp_hw_afbc_fmcu_addr fmcu_addr;
	struct isp_hw_slw_fmcu_cmds *slw = NULL;
	struct isp_hw_fmcu_cfg cfg;

	uint32_t shadow_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_SHADOW_DONE, CAP0_SHADOW_DONE,
		PRE1_SHADOW_DONE, CAP1_SHADOW_DONE,
	};
	uint32_t all_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_ALL_DONE, CAP0_ALL_DONE,
		PRE1_ALL_DONE, CAP1_ALL_DONE,
	};

	slw = (struct isp_hw_slw_fmcu_cmds *)arg;

	if (!slw) {
		pr_err("fail to get valid input ptr, slw %p\n", slw);
		return -EFAULT;
	}

	fmcu = slw->fmcu_handle;
	base = (fmcu->fid == 0) ? ISP_FMCU0_BASE : ISP_FMCU1_BASE;
	fetch_addr = &slw->fetchaddr;
	ctx_idx = slw->ctx_id;
	hw = (struct cam_hw_info *)handle;
	cfg.fmcu = fmcu;
	cfg.ctx_id = ctx_idx;

	hw->isp_ioctl(hw, ISP_HW_CFG_FMCU_CFG, &cfg);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_Y_ADDR);
	cmd = fetch_addr->addr_ch0;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FETCH_SLICE_U_ADDR);
	cmd = fetch_addr->addr_ch1;
	FMCU_PUSH(fmcu, addr, cmd);

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &slw->isp_path[i];

		if (atomic_read(&path->user_cnt) < 1)
			continue;

		store_addr = &slw->store[i].addr;
		sbase = store_base[i];

		addr = ISP_GET_REG(ISP_STORE_SLICE_Y_ADDR) + sbase;
		cmd = store_addr->addr_ch0;
		FMCU_PUSH(fmcu, addr, cmd);

		addr = ISP_GET_REG(ISP_STORE_SLICE_U_ADDR) + sbase;
		cmd = store_addr->addr_ch1;
		FMCU_PUSH(fmcu, addr, cmd);

		addr = ISP_GET_REG(ISP_STORE_SHADOW_CLR) + sbase;
		cmd = 1;
		FMCU_PUSH(fmcu, addr, cmd);

		if ((i < AFBC_PATH_NUM) && (slw->afbc_store[i].bypass == 0)) {
			fmcu_addr.yheader = slw->afbc_store[i].yheader;
			fmcu_addr.yaddr = slw->afbc_store[i].yaddr;
			fmcu_addr.fmcu = fmcu;
			fmcu_addr.index = i;
			hw->isp_ioctl(hw, ISP_HW_CFG_AFBC_FMCU_ADDR_SET, &fmcu_addr);
		}
	}

	reg_off = ISP_CFG_CAP_FMCU_RDY;
	addr = ISP_GET_REG(reg_off);
	cmd = 1;
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = shadow_done_cmd[ctx_idx];
	FMCU_PUSH(fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = all_done_cmd[ctx_idx];
	FMCU_PUSH(fmcu, addr, cmd);

	return 0;
}

static int isphw_fmcu_cfg(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_fmcu_cfg *cfg = NULL;
	unsigned long reg_addr[ISP_CONTEXT_HW_NUM] = {
		ISP_CFG_PRE0_START,
		ISP_CFG_CAP0_START,
		ISP_CFG_PRE1_START,
		ISP_CFG_CAP1_START,
	};

	cfg = (struct isp_hw_fmcu_cfg *)arg;
	addr = ISP_GET_REG(reg_addr[cfg->ctx_id]);
	cmd = 1;
	FMCU_PUSH(cfg->fmcu, addr, cmd);

	/*
	 * When setting CFG_TRIGGER_PULSE cmd, fmcu will wait
	 * until CFG module configs isp registers done.
	 */

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = CFG_TRIGGER_PULSE;
	FMCU_PUSH(cfg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_fetch(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_fetch *fetch = NULL;
	uint32_t base = ISP_FETCH_BASE;

	fetch = (struct isp_hw_slice_fetch *)arg;
	addr = base + ISP_FETCH_MEM_SLICE_SIZE;
	cmd = ((fetch->fetch_info->size.h & 0xFFFF) << 16) | (fetch->fetch_info->size.w & 0xFFFF);
	ISP_REG_WR(fetch->ctx_id, addr, cmd);

	addr = base + ISP_FETCH_SLICE_Y_ADDR;
	cmd = fetch->fetch_info->addr.addr_ch0;
	ISP_REG_WR(fetch->ctx_id, addr, cmd);

	addr = base + ISP_FETCH_SLICE_U_ADDR;
	cmd = fetch->fetch_info->addr.addr_ch1;
	ISP_REG_WR(fetch->ctx_id, addr, cmd);

	addr = base + ISP_FETCH_MIPI_PARAM;
	cmd = fetch->fetch_info->mipi_word_num | (fetch->fetch_info->mipi_byte_rel_pos << 16);
	ISP_REG_MWR(fetch->ctx_id, addr, 0xFFFFF, cmd);

	addr = base + ISP_FETCH_MIPI_PARAM_UV;
	cmd = fetch->fetch_info->mipi_word_num |
			(fetch->fetch_info->mipi_byte_rel_pos << 16);
	ISP_REG_WR(fetch->ctx_id, addr, cmd);

	/* dispatch size same as fetch size */
	addr = ISP_DISPATCH_BASE + ISP_DISPATCH_CH0_SIZE;
	cmd = ((fetch->fetch_info->size.h & 0xFFFF) << 16) | (fetch->fetch_info->size.w & 0xFFFF);
	ISP_REG_WR(fetch->ctx_id, addr, cmd);

	ISP_REG_WR(fetch->ctx_id, ISP_3DNR_MEM_CTRL_PARAM3,
		fetch->fetch_info->size.w | (fetch->fetch_info->size.h << 16));
	ISP_REG_WR(fetch->ctx_id, ISP_3DNR_MEM_CTRL_PARAM4,
		fetch->fetch_info->size.w | (fetch->fetch_info->size.h << 16));
	ISP_REG_WR(fetch->ctx_id, ISP_3DNR_MEM_CTRL_PARAM5,
		fetch->fetch_info->size.w | (fetch->fetch_info->size.h << 15) & 0xffff0000);

	return 0;
}

static int isphw_slice_nr_info(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_nr_info *info = NULL;

	info = (struct isp_hw_slice_nr_info *)arg;
	return 0;

	/* YNR */
	addr = ISP_YNR_CFG31;
	cmd = ((info->cur_slc->slice_ynr.center_offset_y & 0xFFFF) << 16) |
		(info->cur_slc->slice_ynr.center_offset_x & 0xFFFF);
	ISP_REG_WR(info->ctx_id, addr, cmd);

	addr = ISP_YNR_CFG33;
	cmd = ((info->cur_slc->slice_ynr.slice_height & 0xFFFF) << 16) |
		(info->cur_slc->slice_ynr.slice_width & 0xFFFF);
	ISP_REG_WR(info->ctx_id, addr, cmd);

	return 0;
}

static int isphw_slices_fmcu_cmds(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slices_fmcu_cmds *parg = NULL;
	uint32_t shadow_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_SHADOW_DONE, CAP0_SHADOW_DONE,
		PRE1_SHADOW_DONE, CAP1_SHADOW_DONE,
	};
	uint32_t all_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_ALL_DONE, CAP0_ALL_DONE,
		PRE1_ALL_DONE, CAP1_ALL_DONE,
	};

	parg = (struct isp_hw_slices_fmcu_cmds *)arg;
	if (parg->wmode == ISP_CFG_MODE)
		addr = ISP_GET_REG(ISP_CFG_CAP_FMCU_RDY);
	else
		addr = ISP_GET_REG(ISP_FETCH_BASE + ISP_FETCH_START);
	cmd = 1;
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = shadow_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = all_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slices_pyr_rec_fmcu_cmds(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slices_fmcu_cmds *parg = NULL;
	uint32_t shadow_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_SHADOW_DONE, CAP0_SHADOW_DONE,
		PRE1_SHADOW_DONE, CAP1_SHADOW_DONE,
	};
	uint32_t rec_store_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_REC_STORE_DONE, CAP0_REC_STORE_DONE,
		PRE1_REC_STORE_DONE, CAP1_REC_STORE_DONE,
	};

	parg = (struct isp_hw_slices_fmcu_cmds *)arg;
	addr = ISP_GET_REG(ISP_FETCH_BASE + ISP_FETCH_START);
	cmd = 1;
	FMCU_PUSH(parg->fmcu, addr, cmd);
	addr = ISP_GET_REG(PYR_REC_CUR_FETCH_BASE + ISP_FETCH_START);
	cmd = 1;
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = shadow_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = rec_store_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slices_dewarp_fmcu_cmds(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slices_fmcu_cmds *parg = NULL;
	uint32_t shadow_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_SHADOW_DONE, CAP0_SHADOW_DONE,
		PRE1_SHADOW_DONE, CAP1_SHADOW_DONE,
	};
	uint32_t all_done_cmd[ISP_CONTEXT_HW_NUM] = {
		PRE0_ALL_DONE, CAP0_ALL_DONE,
		PRE1_ALL_DONE, CAP1_ALL_DONE,
	};

	parg = (struct isp_hw_slices_fmcu_cmds *)arg;
	if (parg->wmode == ISP_CFG_MODE)
		addr = ISP_GET_REG(ISP_CFG_CAP_FMCU_RDY);
	else
		addr = ISP_GET_REG(ISP_DEWARPING_FSTART);
	cmd = 1;
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = shadow_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_FMCU_CMD);
	cmd = all_done_cmd[parg->hw_ctx_id];
	FMCU_PUSH(parg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_nofilter(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_nofilter *slicearg = NULL;
	struct slice_noisefilter_info *map = NULL;

	slicearg = (struct isp_hw_slice_nofilter *)arg;
	map = slicearg->noisefilter_info;
	pr_debug("seed0=%d,seed1=%d,seed2=%d,seed3=%d\n", map->seed0, map->seed1,
		map->seed2, map->seed3);
	addr = ISP_GET_REG(ISP_YUV_NF_SEED0);
	cmd = map->seed0;
	FMCU_PUSH(slicearg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YUV_NF_SEED1);
	cmd = map->seed1;
	FMCU_PUSH(slicearg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YUV_NF_SEED2);
	cmd = map->seed2;
	FMCU_PUSH(slicearg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YUV_NF_SEED3);
	cmd = map->seed3;
	FMCU_PUSH(slicearg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_3dnr_crop(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_3dnr_crop *croparg = NULL;

	croparg = (struct isp_hw_slice_3dnr_crop *)arg;

	if (croparg->crop->bypass)
		return 0;

	addr = ISP_GET_REG(ISP_3DNR_CROP_PARAM0);
	cmd = croparg->crop->bypass & 0x1;
	FMCU_PUSH(croparg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_CROP_PARAM1);
	cmd = ((croparg->crop->src.h & 0xFFFF) << 16) |
		(croparg->crop->src.w & 0xFFFF);
	FMCU_PUSH(croparg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_CROP_PARAM2);
	cmd = ((croparg->crop->dst.h & 0xFFFF) << 16) |
		(croparg->crop->dst.w & 0xFFFF);
	FMCU_PUSH(croparg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_CROP_PARAM3);
	cmd = ((croparg->crop->start_x & 0xFFFF) << 16) |
		(croparg->crop->start_y & 0xFFFF);
	FMCU_PUSH(croparg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_3dnr_store(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_3dnr_store *storearg = NULL;

	storearg = (struct isp_hw_slice_3dnr_store *)arg;

	addr = ISP_GET_REG(ISP_3DNR_STORE_SHADOW_CLR);
	cmd = 1;
	FMCU_PUSH(storearg->fmcu, addr, cmd);

	if (!storearg->store->bypass) {
		addr = ISP_GET_REG(ISP_3DNR_STORE_SLICE_SIZE);
		cmd = ((storearg->store->size.h & 0xFFFF) << 16) | (storearg->store->size.w & 0xFFFF);
		FMCU_PUSH(storearg->fmcu, addr, cmd);

		addr = ISP_GET_REG(ISP_3DNR_STORE_SLICE_Y_ADDR);
		cmd = storearg->store->addr.addr_ch0;
		FMCU_PUSH(storearg->fmcu, addr, cmd);

		addr = ISP_GET_REG(ISP_3DNR_STORE_SLICE_U_ADDR);
		cmd = storearg->store->addr.addr_ch1;
		FMCU_PUSH(storearg->fmcu, addr, cmd);

		addr = ISP_GET_REG(ISP_3DNR_STORE_SHADOW_CLR);
		cmd = 1;
		FMCU_PUSH(storearg->fmcu, addr, cmd);
	}

	return 0;
}

static int isphw_slice_3dnr_memctrl(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_3dnr_memctrl *memarg = NULL;

	memarg = (struct isp_hw_slice_3dnr_memctrl *)arg;

	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_PARAM3);
	cmd = ((memarg->mem_ctrl->src.h & 0x1FFF) << 16) |
		(memarg->mem_ctrl->src.w & 0x1FFF);
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_PARAM4);
	cmd = ((memarg->mem_ctrl->ft_y.h & 0x1FFF) << 16) |
		(memarg->mem_ctrl->ft_y.w & 0x1FFF);
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_PARAM5);
	cmd = ((memarg->mem_ctrl->ft_uv.h & 0x1FFF) << 16) |
		(memarg->mem_ctrl->ft_uv.w & 0x1FFF);
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	if (memarg->mem_ctrl->bypass)
		return 0;
	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_PARAM1);
	cmd = ((memarg->mem_ctrl->start_col & 0x1FFF) << 16) |
		(memarg->mem_ctrl->start_row & 0x1FFF);
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_FT_CUR_LUMA_ADDR);
	cmd = memarg->mem_ctrl->addr.addr_ch0;
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_3DNR_MEM_CTRL_FT_CUR_CHROMA_ADDR);
	cmd = memarg->mem_ctrl->addr.addr_ch1;
	FMCU_PUSH(memarg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_spath_store(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	unsigned long base = 0;
	struct isp_hw_slice_spath *spath = NULL;

	spath = (struct isp_hw_slice_spath *)arg;
	base = store_base[spath->spath_id];

	if (!spath->path_en) {
		/* bit0 bypass store */
		addr = ISP_GET_REG(ISP_STORE_PARAM) + base;
		cmd = 1;
		FMCU_PUSH(spath->fmcu, addr, cmd);
		return 0;
	}
	addr = ISP_GET_REG(ISP_STORE_PARAM) + base;
	cmd = ISP_REG_RD(spath->ctx_idx, base + ISP_STORE_PARAM) & ~1;
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_SIZE) + base;
	cmd = ((spath->slc_store->size.h & 0xFFFF) << 16) |
			(spath->slc_store->size.w & 0xFFFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_BORDER) + base;
	cmd = (spath->slc_store->border.left_border & 0xFFFF) |
			((spath->slc_store->border.right_border & 0xFFFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_BORDER_1)+ base;
	cmd = (spath->slc_store->border.up_border & 0xFFFF) |
			((spath->slc_store->border.down_border & 0xFFFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_Y_ADDR) + base;
	cmd = spath->slc_store->addr.addr_ch0;
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SLICE_U_ADDR) + base;
	cmd = spath->slc_store->addr.addr_ch1;
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_STORE_SHADOW_CLR) + base;
	cmd = 1;
	FMCU_PUSH(spath->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_spath_scaler(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	unsigned long base = 0;
	struct isp_hw_slice_spath *spath = NULL;

	spath = (struct isp_hw_slice_spath *)arg;
	base = scaler_base[spath->spath_id];

	if (!spath->path_en) {
		addr = ISP_GET_REG(ISP_SCALER_CFG) + base;
		cmd = (0 << 31) | (1 << 2);
		FMCU_PUSH(spath->fmcu, addr, cmd);
		return 0;
	}

	/* bit31 enable path */
	addr = ISP_GET_REG(ISP_SCALER_CFG) + base;
	cmd = ISP_REG_RD(spath->ctx_idx, base + ISP_SCALER_CFG);
	cmd |= (1 << 31);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_SRC_SIZE) + base;
	cmd = (spath->slc_scaler->src_size_x & 0x3FFF) |
			((spath->slc_scaler->src_size_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_DES_SIZE) + base;
	cmd = (spath->slc_scaler->dst_size_x & 0x3FFF) |
			((spath->slc_scaler->dst_size_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM0_START) + base;
	cmd = (spath->slc_scaler->trim0_start_x & 0x3FFF) |
			((spath->slc_scaler->trim0_start_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM0_SIZE) + base;
	cmd = (spath->slc_scaler->trim0_size_x & 0x3FFF) |
			((spath->slc_scaler->trim0_size_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_IP) + base;
	cmd = (spath->slc_scaler->scaler_ip_rmd & 0x3FFF) |
			((spath->slc_scaler->scaler_ip_int & 0x1F) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_CIP) + base;
	cmd = (spath->slc_scaler->scaler_cip_rmd & 0x3FFF) |
			((spath->slc_scaler->scaler_cip_int & 0x1F) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM1_START) + base;
	cmd = (spath->slc_scaler->trim1_start_x & 0x3FFF) |
			((spath->slc_scaler->trim1_start_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_TRIM1_SIZE) + base;
	cmd = (spath->slc_scaler->trim1_size_x & 0x3FFF) |
			((spath->slc_scaler->trim1_size_y & 0x3FFF) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_VER_IP) + base;
	cmd = (spath->slc_scaler->scaler_ip_rmd_ver & 0x3FFF) |
			((spath->slc_scaler->scaler_ip_int_ver & 0x1F) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_SCALER_VER_CIP) + base;
	cmd = (spath->slc_scaler->scaler_cip_rmd_ver & 0x3FFF) |
			((spath->slc_scaler->scaler_cip_int_ver & 0x1F) << 16);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_spath_thumbscaler(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_slice_spath_thumbscaler *spath = NULL;

	spath = (struct isp_hw_slice_spath_thumbscaler *)arg;

	if (!spath->path_en)
		return 0;

	addr = ISP_GET_REG(ISP_THMB_SCALER_BEFORE_TRIM_SIZE);
	cmd = ((spath->slc_scaler->src0.h & 0x1FFF) << 16) |
		(spath->slc_scaler->src0.w & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_Y_SLICE_SRC_SIZE);
	cmd = ((spath->slc_scaler->y_src_after_deci.h & 0x1FFF) << 16) |
		(spath->slc_scaler->y_src_after_deci.w & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_Y_DES_SIZE);
	cmd = ((spath->slc_scaler->y_dst_after_scaler.h & 0x3FF) << 16) |
		(spath->slc_scaler->y_dst_after_scaler.w & 0x3FF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_Y_TRIM0_START);
	cmd = ((spath->slc_scaler->y_trim.start_y & 0x1FFF) << 16) |
		(spath->slc_scaler->y_trim.start_x & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_Y_TRIM0_SIZE);
	cmd = ((spath->slc_scaler->y_trim.size_y & 0x1FFF) << 16) |
		(spath->slc_scaler->y_trim.size_x & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_Y_INIT_PHASE);
	cmd = ((spath->slc_scaler->y_init_phase.h & 0x3FF) << 16) |
		(spath->slc_scaler->y_init_phase.w & 0x3FF);
	FMCU_PUSH(spath->fmcu, addr, cmd);


	addr = ISP_GET_REG(ISP_THMB_SCALER_UV_SLICE_SRC_SIZE);
	cmd = ((spath->slc_scaler->uv_src_after_deci.h & 0x1FFF) << 16) |
		(spath->slc_scaler->uv_src_after_deci.w & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_UV_DES_SIZE);
	cmd = ((spath->slc_scaler->uv_dst_after_scaler.h & 0x3FF) << 16) |
		(spath->slc_scaler->uv_dst_after_scaler.w & 0x3FF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_UV_TRIM0_START);
	cmd = ((spath->slc_scaler->uv_trim.start_y & 0x1FFF) << 16) |
		(spath->slc_scaler->uv_trim.start_x & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_UV_TRIM0_SIZE);
	cmd = ((spath->slc_scaler->uv_trim.size_y & 0x1FFF) << 16) |
		(spath->slc_scaler->uv_trim.size_x & 0x1FFF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_THMB_SCALER_UV_INIT_PHASE);
	cmd = ((spath->slc_scaler->uv_init_phase.h & 0x3FF) << 16) |
		(spath->slc_scaler->uv_init_phase.w & 0x3FF);
	FMCU_PUSH(spath->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_fetch_set(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_set_slice_fetch *fetcharg = NULL;
	uint32_t base = ISP_FETCH_BASE;
	uint32_t base_dispatch = ISP_DISPATCH_BASE;

	fetcharg = (struct isp_hw_set_slice_fetch *)arg;

	addr = ISP_GET_REG(base + ISP_FETCH_MEM_SLICE_SIZE);
	cmd = ((fetcharg->fetch_info->size.h & 0xFFFF) << 16) |
			(fetcharg->fetch_info->size.w & 0xFFFF);
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_FETCH_SLICE_Y_ADDR);
	cmd = fetcharg->fetch_info->addr.addr_ch0;
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_FETCH_SLICE_U_ADDR);
	cmd = fetcharg->fetch_info->addr.addr_ch1;
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_FETCH_MIPI_PARAM);
	cmd = fetcharg->fetch_info->mipi_word_num |
			(fetcharg->fetch_info->mipi_byte_rel_pos << 16) |
			(fetcharg->fetch_info->is_pack << 20);
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	addr = ISP_GET_REG(base + ISP_FETCH_MIPI_PARAM_UV);
	cmd = fetcharg->fetch_info->mipi_word_num_uv |
			(fetcharg->fetch_info->mipi_byte_rel_pos_uv << 16);
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	/* dispatch size same as fetch size */
	addr = ISP_GET_REG(base_dispatch + ISP_DISPATCH_CH0_SIZE);
	cmd = ((fetcharg->fetch_info->size.h & 0xFFFF) << 16) |
			(fetcharg->fetch_info->size.w & 0xFFFF);
	FMCU_PUSH(fetcharg->fmcu, addr, cmd);

	return 0;
}

static int isphw_slice_nr_info_set(void *handle, void *arg)
{
	uint32_t addr = 0, cmd = 0;
	struct isp_hw_set_slice_nr_info *nrarg = NULL;

	return 0;
	nrarg = (struct isp_hw_set_slice_nr_info *)arg;
	/* Post CDN */
	addr = ISP_GET_REG(ISP_POSTCDN_SLICE_CTRL);
	cmd = nrarg->start_row_mod4;
	FMCU_PUSH(nrarg->fmcu, addr, cmd);

	/* YNR */
	addr = ISP_GET_REG(ISP_YNR_CFG31);
	cmd = ((nrarg->slice_ynr->center_offset_y & 0xFFFF) << 16) |
		(nrarg->slice_ynr->center_offset_x & 0xFFFF);
	FMCU_PUSH(nrarg->fmcu, addr, cmd);

	addr = ISP_GET_REG(ISP_YNR_CFG33);
	cmd = ((nrarg->slice_ynr->slice_height & 0xFFFF) << 16) |
		(nrarg->slice_ynr->slice_width & 0xFFFF);
	FMCU_PUSH(nrarg->fmcu, addr, cmd);

	return 0;
}

static int isphw_ltm_param_set(void *handle, void *arg)
{
	struct isp_hw_ltm_3dnr_param *parm = NULL;

	parm = (struct isp_hw_ltm_3dnr_param *)arg;

	ISP_REG_MWR(parm->idx, ISP_LTM_PARAMETERS, BIT_0, parm->val);
	ISP_REG_MWR(parm->idx, ISP_LTM_MAP_PARAM0, BIT_0, parm->val);

	return 0;
}

static int isphw_3dnr_param_set(void *handle, void *arg)
{
	struct isp_hw_ltm_3dnr_param *parm = NULL;

	parm = (struct isp_hw_ltm_3dnr_param *)arg;

	ISP_REG_MWR(parm->idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, parm->val);
	ISP_REG_MWR(parm->idx, ISP_3DNR_BLEND_CONTROL0, BIT_0, parm->val);
	ISP_REG_MWR(parm->idx, ISP_3DNR_STORE_PARAM, BIT_0, parm->val);
	ISP_REG_MWR(parm->idx, ISP_3DNR_CROP_PARAM0, BIT_0, parm->val);

	return 0;
}

static int isphw_radius_parm_adpt(void *handle, void *arg)
{
	struct isp_hw_nlm_ynr *parm = NULL;

	parm = (struct isp_hw_nlm_ynr *)arg;

	parm->val = ISP_REG_RD(parm->ctx_id, ISP_YNR_CFG31);
	parm->slc_cfg_in->ynr_center_x = parm->val & 0xFFFF;
	parm->slc_cfg_in->ynr_center_y = (parm->val >> 16) & 0xfFFF;
	pr_debug("ctx %d,  nlm center %d %d, ynr center %d, %d\n",
		parm->ctx_id, parm->slc_cfg_in->nlm_center_x, parm->slc_cfg_in->nlm_center_y,
		parm->slc_cfg_in->ynr_center_x, parm->slc_cfg_in->ynr_center_y);

	return 0;
}

static int isphw_stop(void *handle, void *arg)
{
	uint32_t id;
	uint32_t cid;
	struct cam_hw_info *hw = NULL;

	hw = (struct cam_hw_info *)handle;
	id = hw->ip_isp->idx;

	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 1 << 26);

	pr_info("ISP%d:ISP_AXI_AXIM_CTRL 0x%x INT STATUS 0x%x  0x%x 0x%x 0x%x\n",
		id, ISP_HREG_RD(ISP_AXI_ITI2AXIM_CTRL),
		ISP_HREG_RD(ISP_P0_INT_BASE + ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C0_INT_BASE + ISP_INT_STATUS),
		ISP_HREG_RD(ISP_P1_INT_BASE + ISP_INT_STATUS),
		ISP_HREG_RD(ISP_C1_INT_BASE + ISP_INT_STATUS));
	udelay(10);

	for (cid = 0; cid < 4; cid++)
		hw->isp_ioctl(hw, ISP_HW_CFG_CLEAR_IRQ, &cid);

	return 0;
}

static int isphw_frame_addr_store(void *handle, void *arg)
{
	uint32_t idx = 0;
	struct isp_hw_path_store *path_store = NULL;
	struct isp_store_info *store_info = NULL;
	unsigned long addr = 0;

	path_store = (struct isp_hw_path_store *)arg;
	idx = path_store->ctx_id;
	store_info = &path_store->store;
	addr = store_base[path_store->spath_id];

	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_Y_ADDR, store_info->addr.addr_ch0);
	ISP_REG_WR(idx, addr + ISP_STORE_SLICE_U_ADDR, store_info->addr.addr_ch1);

	return 0;
}

static int isphw_frame_addr_fetch(void *handle, void *arg)
{
	uint32_t idx = 0;
	struct isp_hw_fetch_info *fetch = NULL;
	uint32_t addr = ISP_FETCH_BASE;
	if (!arg) {
		pr_err("fail to get valid arg\n");
		return -EFAULT;
	}

	fetch = (struct isp_hw_fetch_info *)arg;
	idx = fetch->ctx_id;

	if (fetch->sec_mode == SEC_SPACE_PRIORITY) {
		cam_trusty_isp_fetch_addr_set(fetch->addr_hw.addr_ch0,
			fetch->addr_hw.addr_ch1, fetch->addr_hw.addr_ch2);
	} else {
		pr_debug("y_addr:%x, u_addr:%x.\n", fetch->addr_hw.addr_ch0, fetch->addr_hw.addr_ch1);
		ISP_REG_WR(idx, addr + ISP_FETCH_SLICE_Y_ADDR, fetch->addr_hw.addr_ch0);
		ISP_REG_WR(idx, addr + ISP_FETCH_SLICE_U_ADDR, fetch->addr_hw.addr_ch1);
	}
	return 0;
}

static int isphw_map_init_cfg(void *handle, void *arg)
{
	uint32_t val = 0;
	uint32_t i = 0;
	struct isp_hw_cfg_map *maparg = NULL;

	maparg = (struct isp_hw_cfg_map *)arg;

	val = (maparg->s_cfg_settings->pre1_cmd_ready_mode << 27) |
		(maparg->s_cfg_settings->pre0_cmd_ready_mode << 26) |
		(maparg->s_cfg_settings->cap1_cmd_ready_mode << 25) |
		(maparg->s_cfg_settings->cap0_cmd_ready_mode << 24) |
		(maparg->s_cfg_settings->bp_cap1_pixel_rdy << 23) |
		(maparg->s_cfg_settings->bp_cap0_pixel_rdy << 22) |
		(maparg->s_cfg_settings->bp_pre1_pixel_rdy << 21) |
		(maparg->s_cfg_settings->bp_pre0_pixel_rdy << 20) |
		(maparg->s_cfg_settings->cfg_main_sel << 16) |
		(maparg->s_cfg_settings->num_of_mod << 8) |
		(maparg->s_cfg_settings->sdw_mode << 5) |
		(maparg->s_cfg_settings->tm_bypass << 4) |
		(maparg->s_cfg_settings->bypass);

	if (atomic_inc_return(&maparg->map_cnt) == 1) {
		pr_info("cfg map init start\n");
		for (i = 0; i < maparg->s_cfg_settings->num_of_mod; i++) {
			ISP_HREG_WR(ISP_CFG0_BUF + i * 4,
				maparg->s_cfg_settings->isp_cfg_map[i]);
		}
	}

	ISP_HREG_WR(ISP_CFG_PAMATER, val);

	if (!maparg->s_cfg_settings->tm_bypass) {
		ISP_HREG_WR(ISP_CFG_TM_NUM,
				maparg->s_cfg_settings->tm_set_number);
		ISP_HREG_WR(ISP_CFG_CAP0_TH,
				maparg->s_cfg_settings->cap0_th);
		ISP_HREG_WR(ISP_CFG_CAP1_TH,
				maparg->s_cfg_settings->cap1_th);
	}

	ISP_HREG_MWR(ISP_ARBITER_ENDIAN_COMM, BIT_0, 0x1);

	return 0;
}

static int isphw_cfg_cmd_ready(void *handle, void *arg)
{
	struct isp_hw_cfg_info *cfg_info = (struct isp_hw_cfg_info *)arg;
	uint32_t val = 0;
	uint32_t hw_ctx_id = cfg_info->hw_ctx_id;
	uint32_t ready_mode[ISP_CONTEXT_HW_NUM] = {
		BIT_26,/* pre0_cmd_ready_mode */
		BIT_24,/* cap0_cmd_ready_mode */
		BIT_27,/* pre1_cmd_ready_mode */
		BIT_25/* cap1_cmd_ready_mode */
	};

	if (cfg_info->fmcu_enable)
		val = ready_mode[hw_ctx_id];
	else
		val = 0;

	spin_lock(&isp_cfg_lock);
	ISP_HREG_MWR(ISP_CFG_PAMATER, ready_mode[hw_ctx_id], val);
	spin_unlock(&isp_cfg_lock);

	ISP_HREG_WR(cfg_cmd_addr_reg[hw_ctx_id], cfg_info->hw_addr);
	pr_debug("ctx %d,  reg %08x  %08x, hw_addr %lx, val %08x\n",
		hw_ctx_id,
		(uint32_t)cfg_cmd_addr_reg[hw_ctx_id],
		(uint32_t)ISP_GET_REG(cfg_cmd_addr_reg[hw_ctx_id]),
		cfg_info->hw_addr,
		ISP_HREG_RD(cfg_cmd_addr_reg[hw_ctx_id]));

	return 0;
}

static int isphw_isp_start_cfg(void *handle, void *arg)
{
	uint32_t ctx_id = 0;
	unsigned long reg_addr[] = {
		ISP_CFG_PRE0_START,
		ISP_CFG_CAP0_START,
		ISP_CFG_PRE1_START,
		ISP_CFG_CAP1_START,
	};

	ctx_id = *(uint32_t *)arg;

	pr_debug("isp cfg start:  context_id %d, P0_addr 0x%x\n", ctx_id,
		ISP_HREG_RD(ISP_CFG_PRE0_CMD_ADDR));

	ISP_HREG_WR(reg_addr[ctx_id], 1);
	return 0;
}

static int isphw_fetch_start(void *handle, void *arg)
{
	ISP_HREG_WR(ISP_FETCH_BASE + ISP_FETCH_START, 1);

	return 0;
}

static int isphw_dewarp_fetch_start(void *handle, void *arg)
{
	ISP_HREG_WR(ISP_DEWARPING_FSTART, 1);

	return 0;
}

static int isphw_fmcu_cmd_set(void *handle, void *arg)
{
	struct isp_hw_fmcu_cmd *cmdarg = NULL;

	cmdarg = (struct isp_hw_fmcu_cmd *)arg;

	ISP_HREG_WR(cmdarg->base + ISP_FMCU_DDR_ADDR, cmdarg->hw_addr);
	ISP_HREG_MWR(cmdarg->base + ISP_FMCU_CTRL, 0xFFFF0000, cmdarg->cmd_num << 16);
	ISP_HREG_WR(cmdarg->base + ISP_FMCU_CMD_READY, 1);

	return 0;
}

static int isphw_fmcu_start(void *handle, void *arg)
{
	struct isp_hw_fmcu_start *startarg = NULL;
	uint32_t base = 0;

	startarg = (struct isp_hw_fmcu_start *)arg;

	if (startarg->fid == 0)
		base = ISP_FMCU0_BASE;
	else if (startarg->fid == 1)
		base = ISP_FMCU1_BASE;
	else if (startarg->fid == 2)
		base = ISP_FMCU_PRE1_BASE;
	else if (startarg->fid == ISP_FMCU_DEC)
		base = ISP_FMCU_DEC_BASE;

	ISP_HREG_WR(base + ISP_FMCU_DDR_ADDR, startarg->hw_addr);
	startarg->cmd_num = startarg->cmd_num / 2;
	ISP_HREG_MWR(base + ISP_FMCU_CTRL, 0xFFFF0000, startarg->cmd_num << 16);
	ISP_HREG_WR(base + ISP_FMCU_ISP_REG_REGION, ISP_OFFSET_RANGE);
	ISP_HREG_WR(base + ISP_FMCU_START, 1);

	return 0;
}

static int isphw_yuv_block_ctrl(void *handle, void *arg)
{
	uint32_t ret = 0;
	uint32_t idx = 0;
	uint32_t type = 0;
	struct isp_k_block *p = NULL;
	struct isp_hw_yuv_block_ctrl *blk_ctrl = NULL;
	if (!arg) {
		pr_err("fail to get valid input arg\n");
		return -EFAULT;
	}

	blk_ctrl = (struct isp_hw_yuv_block_ctrl *)arg;
	type = blk_ctrl->type;
	idx = blk_ctrl->idx;
	p = blk_ctrl->blk_param;

	/* TBD: need update if isp process more than once */
	ISP_REG_MWR(idx, ISP_BCHS_PARAM, BIT_0, p->bchs_info.bchs_bypass);
	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, p->cdn_info.bypass);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, p->edge_info_v3.bypass);
	ISP_REG_MWR(idx, ISP_YUV_CNR_CONTRL0, BIT_0, p->iircnr_info.bypass);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, p->nf_info.yrandom_bypass);
	ISP_REG_MWR(idx, ISP_UVD_PARAM, BIT_0, p->uvd_info_v1.bypass);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, p->ygamma_info.bypass);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, BIT_0, p->yrandom_info.bypass);
	ISP_REG_MWR(idx, ISP_CTM_PARAM, BIT_0, p->lut3d_info.rgb3dlut_bypass);

	return ret;
}

static int isphw_fmcu_cmd_align(void *handle, void *arg)
{
	struct isp_fmcu_ctx_desc * fmcu = (struct isp_fmcu_ctx_desc *)arg;
	int byte_num = 0;
	uint32_t addr = 0, cmd = 0;
	if (!fmcu) {
		pr_err("fail to get valid fmcu handle\n");
		return -1;
	}

	byte_num = (int) fmcu->cmdq_pos[fmcu->cur_buf_id];
	if (byte_num % 4 == 2) {
		addr = ISP_GET_REG(ISP_P0_INT_BASE + ISP_INT_CLR1);
		cmd = 0;
		FMCU_PUSH(fmcu, addr, cmd);
	} else if ((byte_num % 4 ==  1) || (byte_num % 4 ==  3)) {
		pr_err("fail to align fmcu cmd\n ");
		return -1;
	}
	return 0;
}

static int isphw_cfg_alldone_ctrl(void *handle, void *arg)
{
	struct isp_hw_alldone_ctrl *para = (struct isp_hw_alldone_ctrl *)arg;
	uint32_t ctx_id = 0;
	uint32_t val = 0, bit = 0;

	if (!arg) {
		pr_err("fail to cfg alldone ctrl\n");
		return -1;
	}
	ctx_id = para->hw_ctx_id;
	val = ISP_HREG_RD(irq_base[ctx_id] + ISP_INT_ALL_DONE_CTRL);
	switch (para->int_bit) {
	case ISP_ALLDONE_WAIT_DISPATCH:
		bit = 4;
		break;
	case ISP_ALLDONE_WAIT_LTMHIST:
		bit = 3;
		break;
	case ISP_ALLDONE_WAIT_REC:
		bit = 2;
		break;
	case ISP_ALLDONE_WAIT_YUV_DONE:
		bit = 1;
		break;
	case ISP_ALLDONE_WAIT_STORE:
		bit = 0;
		break;
	default:
		pr_debug("all done ctrl not support %d\n", para->int_bit);
	}
	if (para->wait)
		val &= ~(0x1UL << bit);
	else
		val |= (0x1UL << bit);
	pr_debug("int %d wait %d, all done ctrl 0x%x\n", para->int_bit, para->wait, val);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_ALL_DONE_CTRL, val);
	return 0;
}

static struct hw_io_ctrl_fun isp_ioctl_fun_tab[] = {
	{ISP_HW_CFG_ENABLE_CLK,              isphw_clk_eb},
	{ISP_HW_CFG_DISABLE_CLK,             isphw_clk_dis},
	{ISP_HW_CFG_PW_ON,                   isphw_pw_on},
	{ISP_HW_CFG_PW_OFF,                  isphw_pw_off},
	{ISP_HW_CFG_RESET,                   isphw_reset},
	{ISP_HW_CFG_ENABLE_IRQ,              isphw_irq_enable},
	{ISP_HW_CFG_DISABLE_IRQ,             isphw_irq_disable},
	{ISP_HW_CFG_CLEAR_IRQ,               isphw_irq_clear},
	{ISP_HW_CFG_FETCH_SET,               isphw_fetch_set},
	{ISP_HW_CFG_FETCH_FBD_SET,           isphw_fetch_fbd_set},
	{ISP_HW_CFG_DEFAULT_PARA_SET,        isphw_default_param_set},
	{ISP_HW_CFG_BLOCK_FUNC_GET,          isphw_block_func_get},
	{ISP_HW_CFG_K_BLK_FUNC_GET,          isphw_k_blk_func_get},
	{ISP_HW_CFG_CFG_MAP_INFO_GET,        isphw_cfg_map_info_get},
	{ISP_HW_CFG_FMCU_VALID_GET,          isphw_fmcu_available},
	{ISP_HW_CFG_BYPASS_DATA_GET,         isphw_bypass_data_get},
	{ISP_HW_CFG_BYPASS_COUNT_GET,        isphw_bypass_count_get},
	{ISP_HW_CFG_REG_TRACE,               isphw_reg_trace},
	{ISP_HW_CFG_ISP_CFG_SUBBLOCK,        isphw_subblock_cfg},
	{ISP_HW_CFG_SET_PATH_STORE,          isphw_path_store},
	{ISP_HW_CFG_SET_PATH_SCALER,         isphw_path_scaler},
	{ISP_HW_CFG_SET_PATH_THUMBSCALER,    isphw_path_thumbscaler},
	{ISP_HW_CFG_SLICE_SCALER,            isphw_slice_scaler},
	{ISP_HW_CFG_SLICE_STORE,             isphw_slice_store},
	{ISP_HW_CFG_AFBC_PATH_SET,           isphw_afbc_path_set},
	{ISP_HW_CFG_FBD_SLICE_SET,           isphw_fbd_slice_set},
	{ISP_HW_CFG_FBD_ADDR_SET,            isphw_fbd_addr_set},
	{ISP_HW_CFG_AFBC_FMCU_ADDR_SET,      isphw_afbc_fmcu_addr_set},
	{ISP_HW_CFG_AFBC_PATH_SLICE_SET,     isphw_afbc_path_slice_set},
	{ISP_HW_CFG_LTM_SLICE_SET,           isphw_ltm_slice_set},
	{ISP_HW_CFG_NR3_FBC_SLICE_SET,       isphw_slice_3dnr_fbc_store},
	{ISP_HW_CFG_NR3_FBD_SLICE_SET,       isphw_slice_3dnr_fbd_fetch},
	{ISP_HW_CFG_SLW_FMCU_CMDS,           isphw_slw_fmcu_cmds},
	{ISP_HW_CFG_FMCU_CFG,                isphw_fmcu_cfg},
	{ISP_HW_CFG_SLICE_FETCH,             isphw_slice_fetch},
	{ISP_HW_CFG_SLICE_NR_INFO,           isphw_slice_nr_info},
	{ISP_HW_CFG_SLICE_FMCU_CMD,          isphw_slices_fmcu_cmds},
	{ISP_HW_CFG_SLICE_FMCU_PYR_REC_CMD,  isphw_slices_pyr_rec_fmcu_cmds},
	{ISP_HW_CFG_SLICE_FMCU_DEWARP_CMD,   isphw_slices_dewarp_fmcu_cmds},
	{ISP_HW_CFG_SLICE_NOFILTER,          isphw_slice_nofilter},
	{ISP_HW_CFG_SLICE_3DNR_CROP,         isphw_slice_3dnr_crop},
	{ISP_HW_CFG_SLICE_3DNR_STORE,        isphw_slice_3dnr_store},
	{ISP_HW_CFG_SLICE_3DNR_MEMCTRL,      isphw_slice_3dnr_memctrl},
	{ISP_HW_CFG_SLICE_SPATH_STORE,       isphw_slice_spath_store},
	{ISP_HW_CFG_SLICE_SPATH_SCALER,      isphw_slice_spath_scaler},
	{ISP_HW_CFG_SLICE_SPATH_THUMBSCALER, isphw_slice_spath_thumbscaler},
	{ISP_HW_CFG_SET_SLICE_FETCH,         isphw_slice_fetch_set},
	{ISP_HW_CFG_SET_SLICE_NR_INFO,       isphw_slice_nr_info_set},
	{ISP_HW_CFG_LTM_PARAM,               isphw_ltm_param_set},
	{ISP_HW_CFG_3DNR_PARAM,              isphw_3dnr_param_set},
	{ISP_HW_CFG_GET_NLM_YNR,             isphw_radius_parm_adpt},
	{ISP_HW_CFG_STOP,                    isphw_stop},
	{ISP_HW_CFG_STORE_FRAME_ADDR,        isphw_frame_addr_store},
	{ISP_HW_CFG_FETCH_FRAME_ADDR,        isphw_frame_addr_fetch},
	{ISP_HW_CFG_MAP_INIT,                isphw_map_init_cfg},
	{ISP_HW_CFG_CMD_READY,               isphw_cfg_cmd_ready},
	{ISP_HW_CFG_START_ISP,               isphw_isp_start_cfg},
	{ISP_HW_CFG_FETCH_START,             isphw_fetch_start},
	{ISP_HW_CFG_DEWARP_FETCH_START,      isphw_dewarp_fetch_start},
	{ISP_HW_CFG_FMCU_CMD,                isphw_fmcu_cmd_set},
	{ISP_HW_CFG_FMCU_START,              isphw_fmcu_start},
	{ISP_HW_CFG_YUV_BLOCK_CTRL_TYPE,     isphw_yuv_block_ctrl},
	{ISP_HW_CFG_FMCU_CMD_ALIGN,          isphw_fmcu_cmd_align},
	{ISP_HW_CFG_ALLDONE_CTRL,            isphw_cfg_alldone_ctrl},
};

static hw_ioctl_fun isphw_ioctl_fun_get(enum isp_hw_cfg_cmd cmd)
{
	hw_ioctl_fun hw_ctrl = NULL;
	uint32_t total_num = 0;
	uint32_t i = 0;

	total_num = sizeof(isp_ioctl_fun_tab) / sizeof(struct hw_io_ctrl_fun);
	for (i = 0; i < total_num; i++) {
		if (cmd == isp_ioctl_fun_tab[i].cmd) {
			hw_ctrl = isp_ioctl_fun_tab[i].hw_ctrl;
			break;
		}
	}

	return hw_ctrl;
}
#endif
