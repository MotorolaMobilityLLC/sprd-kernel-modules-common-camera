/*
 * Copyright (C) 2019-2020 Unisoc Communications Inc.
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

#include <sprd_mm.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include "isp_hw_if.h"
#include "isp_reg.h"
#include "cam_trusty.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_HW_IF: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define ISP_AXI_STOP_TIMEOUT			1000
#define ISP_CLK_NUM                    5
#define CLK_CPPLL                      468000000
static unsigned long irq_base[4] = {
	ISP_P0_INT_BASE,
	ISP_C0_INT_BASE,
	ISP_P1_INT_BASE,
	ISP_C1_INT_BASE
};

uint32_t isp_support_fmcu_cfg_slm(void) {
	return 1;
}

uint32_t isp_ctx_support_fmcu(uint32_t ctx_id) {
	return 1;
}

/* workaround: temp disable FMCU 1 for not working */
int isp_fmcu_available(uint32_t fmcu_id)
{
	return (fmcu_id > 0) ? 0 : 1;
}

static const struct bypass_tag isp_tb_bypass[] = {
[_EISP_GC] =       {"grgb",        ISP_GRGB_CTRL, 0, 1}, /* GrGb correction */
[_EISP_NLM] =      {"nlm",       ISP_NLM_PARA, 0, 1},
[_EISP_VST] =      {"vst",       ISP_VST_PARA, 0, 1},
[_EISP_IVST] =     {"ivst",      ISP_IVST_PARA, 0, 1},
[_EISP_CFA] =      {"cfa",       ISP_CFAE_NEW_CFG0, 0, 1},
[_EISP_CMC] =      {"cmc",       ISP_CMC10_PARAM, 0, 1},
[_EISP_GAMC] =     {"gamma-c",   ISP_GAMMA_PARAM, 0, 1}, /* Gamma correction */
[_EISP_HSV] =      {"hsv",       ISP_HSV_PARAM, 0, 1},
[_EISP_HIST] =    {"hist",     ISP_HIST_PARAM, 0, 1},
[_EISP_HIST2] =    {"hist2",     ISP_HIST2_PARAM, 0, 1},
[_EISP_PSTRZ] =    {"pstrz",     ISP_PSTRZ_PARAM, 0, 1},
[_EISP_PRECDN] =   {"precdn",    ISP_PRECDN_PARAM, 0, 1},
[_EISP_YNR] =      {"ynr",       ISP_YNR_CONTRL0, 0, 1},
[_EISP_EE] =       {"ee",        ISP_EE_PARAM, 0, 1},
[_EISP_GAMY] =     {"ygamma",    ISP_YGAMMA_PARAM, 0, 1},
[_EISP_CDN] =      {"cdn",       ISP_CDN_PARAM, 0, 1},
[_EISP_POSTCDN] =  {"postcdn",   ISP_POSTCDN_COMMON_CTRL, 0, 1},
[_EISP_UVD] =      {"uvd",       ISP_UVD_PARAM, 0, 1},
[_EISP_IIRCNR] =   {"iircnr",    ISP_IIRCNR_PARAM, 0, 1},
[_EISP_YRAND] =    {"yrandom",   ISP_YRANDOM_PARAM1, 0, 1},
[_EISP_BCHS] =     {"bchs",      ISP_BCHS_PARAM, 0, 1},
[_EISP_YUVNF] =    {"yuvnf",     ISP_YUV_NF_CTRL, 0, 1},

	{"ydelay",    ISP_YDELAY_PARAM, 0, 1},
	{"cce",       ISP_CCE_PARAM, 0, 0},
	{"fetch-fbd", ISP_FBD_RAW_SEL, 0, 1},
	/* can't bypass when prev */
	{"scale-pre", ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG, 20, 0},
	{"store-pre", ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM, 0, 0},
	{"scale-vid", ISP_SCALER_VID_BASE + ISP_SCALER_CFG, 20, 1},
	{"store-vid", ISP_STORE_VID_BASE + ISP_STORE_PARAM, 0, 1},
	{"scale-thb", ISP_SCALER_THUMB_BASE + ISP_SCALER_CFG, 20, 1},
	{"store-thb", ISP_STORE_THUMB_BASE + ISP_STORE_PARAM, 0, 1},
	/* ltm */
	{"ltm-map",   ISP_LTM_MAP_PARAM0, 0, 1},
	{"ltm-hist",  ISP_LTM_HIST_PARAM, 0, 1},
	/* 3dnr/nr3 */
	{"nr3-crop",  ISP_3DNR_MEM_CTRL_PRE_PARAM0, 0, 1},
	{"nr3-store", ISP_3DNR_STORE_PARAM, 0, 1},
	{"nr3-mem",   ISP_3DNR_MEM_CTRL_PARAM0, 0, 1},

	{"fetch",     ISP_FETCH_PARAM, 0, 0},
	{"cfg",       ISP_CFG_PAMATER, 0, 0},
};

uint32_t isp_tb_bypass_get_count()
{
	return sizeof(isp_tb_bypass) / sizeof(isp_tb_bypass[0]);
}

struct bypass_tag *isp_tb_bypass_get_data(uint32_t i)
{
	if (i >= isp_tb_bypass_get_count()) {
		return NULL;
	}
	return (struct bypass_tag *)&isp_tb_bypass[i];
}

static uint32_t ISP_CFG_MAP[] __aligned(8) = {
		0x00080710, /*0x0710  - 0x0714 , 2   , common path sel*/
		0x00041A10, /*0x1A10  - 0x1A14 , 1   , BWU*/
		0x00381B10, /*0x1B10  - 0x1B44 , 14  , GRGB*/
		0x00041C10, /*0x1C10  - 0x1C10 , 1   , VST*/
		0x01AC2010, /*0x2010  - 0x21B8 , 107 , NLM*/
		0x00041E10, /*0x1E10  - 0x1E10 , 1   , IVST*/
		0x004C3010, /*0x3010  - 0x3058 , 19  , CFA_NEW*/
		0x00183110, /*0x3110  - 0x3124 , 6   , CMC10*/
		0x00043210, /*0x3210  - 0x3210 , 1   , GAMC_NEW*/
		0x005C3310, /*0x3310  - 0x3368 , 23  , HSV*/
		0x00143710, /*0x3710  - 0x3720 , 5   , HISTS*/
		0x00043410, /*0x3410  - 0x3410 , 1   , PSTRZ*/
		0x001C3510, /*0x3510  - 0x3528 , 7   , CCE*/
		0x001C3610, /*0x3610  - 0x3628 , 7   , UVD*/
		//0x00205510, /*0x5510  - 0x552C , 8   , LTM HISTS*/
		//0x00185F10, /*0x5F10  - 0x5F24 , 6   , LTM MAP*/
		0x004C5010, /*0x5010  - 0x5058 , 19  , PRECDN*/
		0x008C5110, /*0x5110  - 0x5198 , 35  , YNR*/
		0x00485610, /*0x5610  - 0x5654 , 18  , CDN*/
		0x00D85710, /*0x5710  - 0x57E4 , 54  , NEW_EE*/
		0x00745A10, /*0x5A10  - 0x5A80 , 29  , POST_CDN*/
		0x00045B10, /*0x5B10  - 0x5B10 , 1   , YGAMMA*/
		0x00085C10, /*0x5C10  - 0x5C14 , 2   , YUVDELAY*/
		0x00C85D10, /*0x5D10  - 0x5DD4 , 50  , IIRCNR*/
		0x00185E10, /*0x5E10  - 0x5E24 , 6   , YRANDOM*/
		0x00185910, /*0x5910  - 0x5924 , 6   , BCHS*/
		0x00449010, /*0x9010  - 0x9050 , 17  , 3DNR mem ctrl*/
		0x00649110, /*0x9110  - 0x9170 , 25  , 3DNR blend*/
		0x00189210, /*0x9210  - 0x9224 , 6   , 3DNR store*/
		0x00109310, /*0x9310  - 0x931C , 4   , 3DNR crop*/
		/*0x002C9410, *0x9410  - 0x9438 , 11  , FBC 3DNR store*/
		/*0x003C9510, *0x9510  - 0x9548 , 15  , FBD 3DNR fetch*/
		0x0050D010, /*0xD010  - 0xD05C , 20  , SCL_VID*/
		0x0030D110, /*0xD110  - 0xD13C , 12  , SCL_VID_store*/
		//0x0030D310, /*0xD310  - 0xD33C , 12  , SCL_VID_FBC_store*/
		0x0044C010, /*0xC010  - 0xC050 , 17  , SCL_CAP*/
		0x0030C110, /*0xC110  - 0xC13C , 12  , SCL_CAP_store*/
		0x0044C210, /*0xC210  - 0xC250 , 17  , SCL_CAP_Noisefilter_add_rdm*/
		//0x0030C310, /*0xC310  - 0xC33C , 12  , SCL_CAP_FBC_store*/
		0x0054E010, /*0xE010  - 0xE060 , 21  , SCL_THUMB*/
		0x0030E110, /*0xE110  - 0xE13C , 12  , SCL_THUMB_store*/
		0x00300110, /*0x110   - 0x13C  , 12  , FETCH*/
		0x00300210, /*0x210   - 0x23C  , 12  , STORE*/
		0x001C0310, /*0x310   - 0x328  , 7   , DISPATCH*/
		//0x00340C10, /*0x0C10  - 0x0C40 , 13  , Fetch_FBD*/
		0x05A18000, /*0x18000 - 0x1859C, 360 , ISP_HSV_BUF0_CH0*/
		0x20019000, /*0x19000 - 0x19FFC, 1024, ISP_VST_BUF0_CH0*/
		0x20029000, /*0x1A000 - 0x1AFFC, 1024, ISP_IVST_BUF0_CH0*/
		0x0401B000, /*0x1B000 - 0x1B3FC, 256 , ISP_FGAMMA_R_BUF0_CH0*/
		0x0401C000, /*0x1C000 - 0x1C3FC, 256 , ISP_FGAMMA_G_BUF0_CH0*/
		0x0401D000, /*0x1D000 - 0x1D3FC, 256 , ISP_FGAMMA_B_BUF0_CH0*/
		0x0201B400, /*0x1B400 - 0x1B5FC, 128 , ISP_PSTRZ_R_BUF0_CH0*/
		0x0201C400, /*0x1C400 - 0x1C5FC, 128 , ISP_PSTRZ_G_BUF0_CH0*/
		0x0201D400, /*0x1D400 - 0x1D5FC, 128 , ISP_PSTRZ_B_BUF0_CH0*/
		0x0205E000, /*0x1E000 - 0x1E200, 129 , ISP_YGAMMA_BUF0_CH0*/
		0x00839100, /*0x39100 - 0x3917C, 32  , CAP_HOR_CORF_Y_BUF0_CH0*/
		0x00439300, /*0x39300 - 0x3933C, 16  , CAP_HOR_CORF_UV_BUF0*/
		0x021394F0, /*0x394F0 - 0x396FC, 132 , CAP_VER_CORF_Y_BUF0_CH0*/
		0x02139AF0, /*0x39AF0 - 0x39CFC, 132 , CAP_VER_CORF_UV_BUF0*/
		0x00838100, /*0x38100 - 0x3817C, 32  , VID_HOR_CORF_Y_BUF0_CH0*/
		0x00438300, /*0x38300 - 0x3833C, 16  , VID_HOR_CORF_UV_BUF0*/
		0x021384F0, /*0x384F0 - 0x386FC, 132 , VID_VER_CORF_Y_BUF0_CH0*/
		0x02138AF0, /*0x38AF0 - 0x38CFC, 132 , VID_VER_CORF_UV_BUF0*/
};

uint32_t isp_cfg_map_get_count()
{
	return ARRAY_SIZE(ISP_CFG_MAP);
}

uint32_t *isp_cfg_map_get_data()
{
	return ISP_CFG_MAP;
}

uint32_t isp_hist_bypass_get(int ctx_id)
{
	uint32_t bypass = 0;
	bypass = ISP_REG_RD(ctx_id, ISP_HIST2_PARAM) & BIT_0;

	return bypass;
}

void set_common(struct sprd_cam_hw_info *hw)
{
	uint32_t wqos_val = 0;
	uint32_t rqos_val = 0;

	wqos_val = (0x1 << 13) | (0x0 << 12) | (0x4 << 8) |
			((hw->awqos_high & 0xF) << 4) |
			(hw->awqos_low & 0xF);
	rqos_val = (0x0 << 8) |
			((hw->arqos_high & 0xF) << 4) |
			(hw->arqos_low & 0xF);
	ISP_HREG_MWR(ISP_AXI_ARBITER_WQOS,
					0x37FF,
					wqos_val);
	ISP_HREG_MWR(ISP_AXI_ARBITER_RQOS,
					0x1FF,
					rqos_val);
	ISP_HREG_WR(ISP_CORE_PMU_EN, 0xFFFF0000);

	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_0, 0xFFFF0000);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_1, 0xFFFF0000);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_2, 0xFFFF0000);
	ISP_HREG_WR(ISP_COMMON_GCLK_CTRL_3, 0xFF00);

	ISP_HREG_MWR(ISP_AXI_ISOLATION, BIT_0, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN0, BIT_0, 0);
	ISP_HREG_MWR(ISP_ARBITER_ENDIAN1, BIT(0) | BIT(1), 0);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM_CLR, 0xF10);
	ISP_HREG_WR(ISP_ARBITER_CHK_SUM0, 0x0);
	/* enable axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);

	/* to be defined. */
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_16, (1 << 16));
	ISP_HREG_MWR(
		ISP_COMMON_SHADOW_CTRL_CH0, BIT_21, (0 << 21));

	ISP_HREG_MWR(ISP_COMMON_PMU_RAM_MASK, BIT_0, 1);
	ISP_HREG_MWR(ISP_BLOCK_MODE, 0xF, 0);

	/* dispatch_done should be disable? */
	ISP_HREG_MWR(ISP_INT_ALL_DONE_CTRL, 0x1F, 0x1C);

	/* bypass config mode by default */
	ISP_HREG_MWR(ISP_CFG_PAMATER, BIT_0, 1);
	pr_debug("end\n");
}

void isp_set_ctx_common(struct isp_pipe_context *pctx)
{
	uint32_t idx = pctx->ctx_id;
	uint32_t bypass = 0;
	uint32_t bwu_val = 0;
	uint32_t en_3dnr;
	struct isp_fetch_info *fetch = &pctx->fetch;
	struct isp_fbd_raw_info *fbd_raw = &pctx->fbd_raw;

	pr_info("enter: fmt:%d, w:%d, h:%d\n", fetch->fetch_fmt,
			fetch->in_trim.size_x, fetch->in_trim.size_y);

	en_3dnr = 0;/* (pctx->mode_3dnr == MODE_3DNR_OFF) ? 0 : 1; */
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			BIT_1 | BIT_0, pctx->dispatch_color);
	/* 11b: close store_dbg module */
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL,
			BIT_3 | BIT_2, 3 << 2);
	ISP_REG_MWR(idx, ISP_COMMON_SPACE_SEL, BIT_4, 0 << 4);

	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_10, pctx->fetch_path_sel  << 10);
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_8, en_3dnr << 8); /* 3dnr off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_5 | BIT_4, 3 << 4);  /* thumb path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_3 | BIT_2, 3 << 2); /* vid path off */
	ISP_REG_MWR(idx, ISP_COMMON_SCL_PATH_SEL,
			BIT_1 | BIT_0, 3 << 0);  /* pre/cap path off */

	ISP_REG_MWR(idx, ISP_FETCH_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_FETCH_PARAM,
			(0xF << 4), fetch->fetch_fmt << 4);
	ISP_REG_WR(idx, ISP_FETCH_MEM_SLICE_SIZE,
			fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));

	pr_info("camca, isp sec mode=%d, is_loose=%d, pitch_ch0=0x%x, 0x%x, 0x%x\n",
		pctx->dev->sec_mode,
		pctx->is_loose,
		fetch->pitch.pitch_ch0,
		fetch->pitch.pitch_ch1,
		fetch->pitch.pitch_ch2);

	if(pctx->is_loose == ISP_RAW_HALF14 || pctx->is_loose == ISP_RAW_HALF10)
		bwu_val = 0x40001;
	else
		bwu_val = 0x40000;
	ISP_REG_WR(idx, ISP_BWU_PARAM, bwu_val);

	if (pctx->dev->sec_mode == SEC_SPACE_PRIORITY) {
		camca_isp_pitch_set(fetch->pitch.pitch_ch0,
			fetch->pitch.pitch_ch1,
			fetch->pitch.pitch_ch2);
	} else {
		ISP_REG_WR(idx, ISP_FETCH_SLICE_Y_PITCH, fetch->pitch.pitch_ch0);
		ISP_REG_WR(idx, ISP_FETCH_SLICE_U_PITCH, fetch->pitch.pitch_ch1);
		ISP_REG_WR(idx, ISP_FETCH_SLICE_V_PITCH, fetch->pitch.pitch_ch2);
	}

	ISP_REG_WR(idx, ISP_FETCH_LINE_DLY_CTRL, 0x8);
	ISP_REG_WR(idx, ISP_FETCH_MIPI_INFO,
		fetch->mipi_word_num | (fetch->mipi_byte_rel_pos << 16));

	/* fetch fbd */
	if (pctx->fetch_path_sel) {
		ISP_REG_MWR(idx, ISP_FBD_RAW_SEL,
			    BIT(0), fbd_raw->fetch_fbd_bypass);
		ISP_REG_MWR(idx, ISP_FBD_RAW_SEL,
			    0x00003f00, fbd_raw->pixel_start_in_hor);
		ISP_REG_MWR(idx, ISP_FBD_RAW_SEL,
			    0x00000030, fbd_raw->pixel_start_in_ver);
		ISP_REG_WR(idx, ISP_FBD_RAW_SLICE_SIZE,
			   fbd_raw->width | (fbd_raw->height << 16));
		ISP_REG_WR(idx, ISP_FBD_RAW_PARAM0,
			   fbd_raw->tiles_num_in_hor
			   | (fbd_raw->tiles_num_in_ver << 16));
		ISP_REG_WR(idx, ISP_FBD_RAW_PARAM1,
			   0x2 << 16
			   | (fbd_raw->tiles_start_odd & 0x1) << 8
			   | ((fbd_raw->tiles_num_pitch) & 0xff));
		ISP_REG_WR(idx, ISP_FBD_RAW_LOW_PARAM1, fbd_raw->low_bit_pitch);
		pr_info("enable fbd: %d\n", !fbd_raw->fetch_fbd_bypass);
	}

	ISP_REG_WR(idx, ISP_DISPATCH_DLY,  0x253C);
	ISP_REG_WR(idx, ISP_DISPATCH_LINE_DLY1,  0x280001C);
	ISP_REG_WR(idx, ISP_DISPATCH_PIPE_BUF_CTRL_CH0,  0x64043C);
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_SIZE,
		fetch->in_trim.size_x | (fetch->in_trim.size_y << 16));
	ISP_REG_WR(idx, ISP_DISPATCH_CH0_BAYER, pctx->dispatch_bayer_mode);


	ISP_REG_WR(idx, ISP_YDELAY_STEP, 0x144);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE
		+ ISP_SCALER_HBLANK, 0x4040);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_RES, 0xFF);
	ISP_REG_WR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_DEBUG, 1);

	pr_info("end\n");
}

void isp_set_ctx_default(struct isp_pipe_context *pctx)
{
	uint32_t idx = pctx->ctx_id;
	uint32_t bypass = 1;

	ISP_REG_MWR(idx, ISP_STORE_DEBUG_BASE + ISP_STORE_PARAM, BIT_0, bypass);

	/* bypass all path scaler & store */
	ISP_REG_MWR(idx, ISP_SCALER_PRE_CAP_BASE + ISP_SCALER_CFG,
		BIT_31, 0 << 31);
	ISP_REG_MWR(idx, ISP_SCALER_VID_BASE + ISP_SCALER_CFG,
		BIT_31, 0 << 31);

	ISP_REG_MWR(idx, ISP_STORE_PRE_CAP_BASE + ISP_STORE_PARAM,
		BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_VID_BASE + ISP_STORE_PARAM,
		BIT_0, 1);
	ISP_REG_MWR(idx, ISP_STORE_THUMB_BASE + ISP_STORE_PARAM,
		BIT_0, 1);

	/* default bypass all blocks */
	ISP_REG_MWR(idx, ISP_NLM_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_VST_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_IVST_PARA, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_CMC10_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_CCE_PARAM, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_UVD_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_CFG_RDY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CFAE_NEW_CFG0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, 1);

	ISP_REG_MWR(idx, ISP_LTM_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_LTM_MAP_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_BCHS_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_POSTCDN_COMMON_CTRL, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, BIT_0, 1);

	/* 3DNR bypass */
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_BLEND_CONTROL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PRE_PARAM0, BIT_0, 1);

	/*CFA*/
	ISP_REG_MWR(idx, ISP_CFAE_NEW_CFG0, BIT_0, 0);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG0, 0x1F4 | 0x1F4 << 16);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG1,
		(0x1 << 31) | (0x14 << 12) | (0x7F << 4) | 0x4);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG2, 0x8 | (0x0 << 8));
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG3,
		(0x8 << 20) | (0x8 << 12) | 0x118);
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG4, 0x64 | (0x64 << 16));
	ISP_REG_WR(idx, ISP_CFAE_INTP_CFG5, 0xC8 | (0xC8 << 16));

	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG0, 0x64 | (0x96 << 16));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG1, 0x14 | (0x5 << 16));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG2,
		(0x28 << 20) | (0x1E << 10) | 0xF);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG3, 0xC8);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG4, (0x5 << 10));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG5, (0x50 << 9) | 0x78);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG6,
		(0x32 << 18) | (0x32 << 9));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG7, (0x64 << 18));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG8, 0x3C | (0x8 << 17));
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG9,
		(0x1FC << 20) | (0x134 << 10) | 0x27C);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG10,
		(0x214 << 20) | (0x1FF << 10) | 0x1CD);
	ISP_REG_WR(idx, ISP_CFAE_CSS_CFG11, 0x22D << 10 | 0x205);

	/*CCE*/
	ISP_REG_MWR(idx, ISP_CCE_PARAM, BIT_0, 0);
	ISP_REG_WR(idx, ISP_CCE_MATRIX0, (150 << 11) | 77);
	ISP_REG_WR(idx, ISP_CCE_MATRIX1, ((-43) << 11) | 29);
	ISP_REG_WR(idx, ISP_CCE_MATRIX2, 0x407AB);
	ISP_REG_WR(idx, ISP_CCE_MATRIX3, ((-107) << 11) | 128);
	ISP_REG_WR(idx, ISP_CCE_MATRIX4, (-21));
	ISP_REG_WR(idx, ISP_CCE_SHIFT, 0);

	pr_info("end\n");
}

static struct isp_cfg_entry isp_cfg_func_tab[ISP_BLOCK_TOTAL - ISP_BLOCK_BASE] = {
[ISP_BLOCK_BCHS - ISP_BLOCK_BASE]	= {ISP_BLOCK_BCHS, isp_k_cfg_bchs},
[ISP_BLOCK_YGAMMA - ISP_BLOCK_BASE]	= {ISP_BLOCK_YGAMMA, isp_k_cfg_ygamma},
[ISP_BLOCK_GAMMA - ISP_BLOCK_BASE]	= {ISP_BLOCK_GAMMA, isp_k_cfg_gamma},
[ISP_BLOCK_CCE - ISP_BLOCK_BASE]	= {ISP_BLOCK_CCE, isp_k_cfg_cce},
[ISP_BLOCK_UVD - ISP_BLOCK_BASE]	= {ISP_BLOCK_UVD, isp_k_cfg_uvd},
[ISP_BLOCK_CFA - ISP_BLOCK_BASE]	= {ISP_BLOCK_CFA, isp_k_cfg_cfa},
[ISP_BLOCK_CMC - ISP_BLOCK_BASE]	= {ISP_BLOCK_CMC, isp_k_cfg_cmc10},
[ISP_BLOCK_CDN - ISP_BLOCK_BASE]	= {ISP_BLOCK_CDN, isp_k_cfg_cdn},
[ISP_BLOCK_HSV - ISP_BLOCK_BASE]	= {ISP_BLOCK_HSV, isp_k_cfg_hsv},
[ISP_BLOCK_GRGB - ISP_BLOCK_BASE]	= {ISP_BLOCK_GRGB, isp_k_cfg_grgb},
[ISP_BLOCK_EDGE - ISP_BLOCK_BASE]	= {ISP_BLOCK_EDGE, isp_k_cfg_edge},
[ISP_BLOCK_HIST2 - ISP_BLOCK_BASE]	= {ISP_BLOCK_HIST2, isp_k_cfg_hist2},
[ISP_BLOCK_IIRCNR - ISP_BLOCK_BASE]	= {ISP_BLOCK_IIRCNR, isp_k_cfg_iircnr},
[ISP_BLOCK_PRE_CDN - ISP_BLOCK_BASE]	= {ISP_BLOCK_PRE_CDN, isp_k_cfg_pre_cdn},
[ISP_BLOCK_POST_CDN - ISP_BLOCK_BASE]	= {ISP_BLOCK_POST_CDN, isp_k_cfg_post_cdn},
[ISP_BLOCK_PSTRZ - ISP_BLOCK_BASE]	= {ISP_BLOCK_PSTRZ, isp_k_cfg_pstrz},
[ISP_BLOCK_YRANDOM - ISP_BLOCK_BASE]	= {ISP_BLOCK_YRANDOM, isp_k_cfg_yrandom},
};

struct isp_cfg_entry *isp_get_cfg_func(uint32_t index)
{
	if (index < (ISP_BLOCK_TOTAL - ISP_BLOCK_BASE))
		return &isp_cfg_func_tab[index];
	else {
		pr_err("fail to get right isp cfg func index %d\n", index);
		return NULL;
	}
}

int isp_irq_clear(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR0, 0xFFFFFFFF);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_CLR1, 0xFFFFFFFF);

	return 0;
}
int isp_reset(struct sprd_cam_hw_info *hw, void *arg)
{
	int rtn = 0;
	uint32_t cid;
	uint32_t time_out = 0;
	uint32_t flag = 0;

	pr_info("ISP%d: reset\n", hw->idx);

	/* firstly stop axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT(26), BIT(26));

	/* then wait for AHB busy cleared */
	while (++time_out < ISP_AXI_STOP_TIMEOUT) {
		/* bit3: 1 - axi idle;  0 - axi busy */
		if  (ISP_HREG_RD(ISP_INT_STATUS) & BIT_3)
			break;
		udelay(1000);
	}

	if (time_out >= ISP_AXI_STOP_TIMEOUT) {
		pr_info("ISP reset timeout %d\n", time_out);
	} else {
		flag = hw->syscon.rst_mask
			| hw->syscon.rst_ahb_mask
			| hw->syscon.rst_vau_mask;
		pr_debug("ISP reset rst_mask 0x%x, rst_ahb_mask 0x%x, rst_vau_mask 0x%x\n",
			hw->syscon.rst_mask, hw->syscon.rst_ahb_mask,  hw->syscon.rst_vau_mask);
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.rst, flag, flag);
		udelay(10);
		regmap_update_bits(hw->cam_ahb_gpr,
			hw->syscon.rst, flag, ~flag);
	}

	/* enable axim transfering */
	ISP_HREG_MWR(ISP_AXI_ITI2AXIM_CTRL, BIT_26, 0);

	for (cid = 0; cid < 4; cid++) {
		isp_irq_clear(hw, &cid);
		isp_irq_disable(hw, &cid);
	}

	pr_info("ISP%d: reset end\n", hw->idx);
	return rtn;
}

int isp_irq_disable(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN0, 0);
	ISP_HREG_WR(irq_base[ctx_id] + ISP_INT_EN1, 0);

	return 0;
}

int isp_irq_enable(struct sprd_cam_hw_info *hw, void *arg)
{
	uint32_t ctx_id;
	uint32_t mask = ~0;

	if (!hw || !arg) {
		pr_err("error: null input ptr.\n");
		return -EFAULT;
	}

	ctx_id = *(uint32_t *)arg;
	if (ctx_id >= 4) {
		pr_err("error ctx id %d\n", ctx_id);
		return -EFAULT;
	}

	ISP_HREG_MWR(irq_base[ctx_id] + ISP_INT_EN0, mask, mask);

	return 0;
}

