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

#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sprd_mm.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>

#include "cam_trusty.h"
#include "dcam_reg.h"
#include "dcam_hw_adpt.h"
#include "dcam_int.h"
#include "dcam_core.h"

#include "isp_reg.h"
#include "isp_hw_adpt.h"
#include "isp_int.h"
#include "isp_slice.h"
#include "isp_cfg.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_HW_IF_L3: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static unsigned long coff_buf_addr[2][3][4] = {
	{
		{
			ISP_SCALER_PRE_LUMA_HCOEFF_BUF0,
			ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0,
			ISP_SCALER_PRE_LUMA_VCOEFF_BUF0,
			ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0,
		},
		{
			ISP_SCALER_VID_LUMA_HCOEFF_BUF0,
			ISP_SCALER_VID_CHROMA_HCOEFF_BUF0,
			ISP_SCALER_VID_LUMA_VCOEFF_BUF0,
			ISP_SCALER_VID_CHROMA_VCOEFF_BUF0,
		}
	},
	{
		{
			ISP_SCALER_PRE_LUMA_HCOEFF_BUF1,
			ISP_SCALER_PRE_CHROMA_HCOEFF_BUF1,
			ISP_SCALER_PRE_LUMA_VCOEFF_BUF1,
			ISP_SCALER_PRE_CHROMA_VCOEFF_BUF1,
		},
		{
			ISP_SCALER_VID_LUMA_HCOEFF_BUF1,
			ISP_SCALER_VID_CHROMA_HCOEFF_BUF1,
			ISP_SCALER_VID_LUMA_VCOEFF_BUF1,
			ISP_SCALER_VID_CHROMA_VCOEFF_BUF1,
		}
	},
};

#define CAM_HW_ADPT_LAYER
#include "dcam_hw.c"
#include "isp_hw.c"

static int camhw_get_isp_dts_clk(void *handle, void *arg)
{
	struct cam_hw_soc_info *soc_isp = NULL;
	struct cam_hw_info *hw = NULL;
	struct device_node *isp_node = (struct device_node *)arg;

	hw = (struct cam_hw_info *)handle;
	if (!handle) {
		pr_err("fail to get invalid handle\n");
		return -EINVAL;
	}

	soc_isp = hw->soc_isp;

	soc_isp->core_eb = of_clk_get_by_name(isp_node, "isp_eb");
	if (IS_ERR_OR_NULL(soc_isp->core_eb)) {
		pr_err("fail to read dts isp eb\n");
		return -EFAULT;
	}
	soc_isp->axi_eb = of_clk_get_by_name(isp_node, "isp_axi_eb");
	if (IS_ERR_OR_NULL(soc_isp->axi_eb)) {
		pr_err("fail to read dts isp axi eb\n");
		return -EFAULT;
	}
	soc_isp->clk = of_clk_get_by_name(isp_node, "isp_clk");
	if (IS_ERR_OR_NULL(soc_isp->clk)) {
		pr_err("fail to read dts isp clk\n");
		return -EFAULT;
	}
	soc_isp->clk_parent = of_clk_get_by_name(isp_node, "isp_clk_parent");
	if (IS_ERR_OR_NULL(soc_isp->clk_parent)) {
		pr_err("fail to read dts isp clk parent\n");
		return -EFAULT;
	}
	soc_isp->clk_default = clk_get_parent(soc_isp->clk);
	return 0;
}

static int camhw_get_dcam_dts_clk(void *handle, void *arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct device_node *dn = (struct device_node *)arg;
	struct cam_hw_soc_info *soc_dcam;

	hw = (struct cam_hw_info *)handle;
	if (!handle) {
		pr_err("fail to get invalid handle\n");
		return -EINVAL;
	}

	soc_dcam = hw->soc_dcam;

	soc_dcam->core_eb = of_clk_get_by_name(dn, "dcam_eb");
	if (IS_ERR_OR_NULL(soc_dcam->core_eb)) {
		pr_err("fail to read clk, dcam_eb\n");
		ret = 1;
	}

	soc_dcam->axi_eb = of_clk_get_by_name(dn, "dcam_axi_eb");
	if (IS_ERR_OR_NULL(soc_dcam->axi_eb)) {
		pr_err("fail to read clk, dcam_axi_eb\n");
		ret = 1;
	}

	soc_dcam->clk = of_clk_get_by_name(dn, "dcam_clk");
	if (IS_ERR_OR_NULL(soc_dcam->clk)) {
		pr_err("fail to read clk, dcam_clk\n");
		ret = 1;
	}

	soc_dcam->clk_parent = of_clk_get_by_name(dn, "dcam_clk_parent");
	if (IS_ERR_OR_NULL(soc_dcam->clk_parent)) {
		pr_err("fail to read clk, dcam_clk_parent\n");
		ret = 1;
	}
	soc_dcam->clk_default = clk_get_parent(soc_dcam->clk);

	soc_dcam->bpc_clk = of_clk_get_by_name(dn, "dcam_bpc_clk");
	if (IS_ERR_OR_NULL(soc_dcam->bpc_clk)) {
		pr_err("fail to get dcam_bpc_clk\n");
		ret = 1;
	}
	soc_dcam->bpc_clk_parent = of_clk_get_by_name(dn, "dcam_bpc_clk_parent");
	if (IS_ERR_OR_NULL(soc_dcam->bpc_clk_parent)) {
		pr_err("fail to get dcam_bpc_clk_parent\n");
		ret = 1;
	}
	soc_dcam->bpc_clk_default = clk_get_parent(soc_dcam->bpc_clk);

	return ret;
}

static int camhw_get_all_rst(void *handle, void *arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	uint32_t args[2];
	struct device_node *dn = (struct device_node *)arg;
	struct cam_hw_ip_info *dcam_info = NULL;

	int i = 0;
	if (!handle) {
		pr_err("fail to get invalid handle\n");
		return -EINVAL;
	}

	hw = (struct cam_hw_info *)handle;

	ret = cam_syscon_get_args_by_name(dn, "dcam_all_reset", ARRAY_SIZE(args), args);
	if (ret) {
		pr_err("fail to get dcam all reset syscon\n");
		return -EINVAL;
	}

	for (i = 0; i < DCAM_ID_MAX; i++) {
		dcam_info = hw->ip_dcam[i];
		dcam_info->syscon.all_rst = args[0];
		dcam_info->syscon.all_rst_mask= args[1];
	}

	return ret;
}

static int camhw_get_axi_base(void *handle, void *arg)
{
	struct cam_hw_info *hw = NULL;
	struct device_node *dn = (struct device_node *)arg;
	int pos = 0;
	uint32_t count;
	struct resource reg_res = {0};
	void __iomem *reg_base = NULL;
	struct cam_hw_soc_info *soc_dcam;
	int i = 0;
	if (!handle) {
		pr_err("fail to get invalid handle\n");
		return -EINVAL;
	}

	hw = (struct cam_hw_info *)handle;
	soc_dcam = hw->soc_dcam;
	if (of_property_read_u32(dn, "sprd,dcam-count", &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	pos = count;
	if (of_address_to_resource(dn, pos, &reg_res)) {
		pr_err("fail to get AXIM phy addr\n");
		goto err_axi_iounmap;
	}

	reg_base = ioremap(reg_res.start, reg_res.end - reg_res.start + 1);
	if (!reg_base) {
		pr_err("fail to map AXIM reg base\n");
		goto err_axi_iounmap;
	}

	for (i = 0; i < count; i++)
		g_dcam_aximbase[i] = (unsigned long)reg_base;

	soc_dcam->axi_reg_base = (unsigned long)reg_base;

	return 0;

err_axi_iounmap:
	for (i = 0; i < DCAM_ID_MAX; i++) {
		g_dcam_aximbase[i] = 0;
	}
	return -1;
}

static int camhw_get_block_type(void *handle, void *arg)
{
	int ret = 0;
	struct cam_cfg_block *blk_type = (struct cam_cfg_block *)arg;

	switch (blk_type->sub_block) {
	case DCAM_BLOCK_BLC:
	case DCAM_BLOCK_RGBG:
	case DCAM_BLOCK_RGBG_DITHER:
	case DCAM_BLOCK_PDAF:
	case DCAM_BLOCK_LSC:
	case DCAM_BLOCK_BAYERHIST:
	case DCAM_BLOCK_AEM:
	case DCAM_BLOCK_AFL:
	case DCAM_BLOCK_AWBC:
	case DCAM_BLOCK_BPC:
	case DCAM_BLOCK_AFM:
	case DCAM_BLOCK_GRGB:
	case DCAM_BLOCK_3DNR_ME:
	case DCAM_BLOCK_GTM:
		blk_type->block_type = DCAM_BLOCK_TYPE;
		break;
	case ISP_BLOCK_RGB_LTM:
		if(blk_type->property == ISP_PRO_RGB_LTM_BYPASS)
			blk_type->block_type = DCAM_BLOCK_TYPE;
		else
			blk_type->block_type = ISP_BLOCK_TYPE;
		break;
	case ISP_BLOCK_GAMMA:
	case ISP_BLOCK_CMC:
	case ISP_BLOCK_CFA:
	case ISP_BLOCK_NLM:
	case ISP_BLOCK_CCE:
	case ISP_BLOCK_CDN:
	case ISP_BLOCK_EDGE:
	case ISP_BLOCK_UVD:
	case ISP_BLOCK_YGAMMA:
	case ISP_BLOCK_YRANDOM:
	case ISP_BLOCK_YNR:
	case ISP_BLOCK_HSV:
	case ISP_BLOCK_HIST2:
	case ISP_BLOCK_3DNR:
	case ISP_BLOCK_PRE_CDN:
	case ISP_BLOCK_POST_CDN:
	case ISP_BLOCK_IIRCNR:
	case ISP_BLOCK_HIST:
	case ISP_BLOCK_CSA:
	case ISP_BLOCK_HUE:
	case ISP_BLOCK_PSTRZ:
	case ISP_BLOCK_CONTRAST:
	case ISP_BLOCK_BRIGHTNESS:
	case ISP_BLOCK_NOISEFILTER:
		blk_type->block_type = ISP_BLOCK_TYPE;
		break;
	default:
		pr_err("fail to get valid block %d\n", blk_type->sub_block);
		ret = -EFAULT;
		break;
	}
	return ret;
}

static struct cam_cfg_entry cam_hw_cfg_func_tab[ISP_BLOCK_TOTAL] = {
	[DCAM_BLOCK_BLC]         = {DCAM_BLOCK_BLC,         dcam_k_cfg_blc},
	[DCAM_BLOCK_AEM]         = {DCAM_BLOCK_AEM,         dcam_k_cfg_aem},
	[DCAM_BLOCK_AWBC]        = {DCAM_BLOCK_AWBC,        dcam_k_cfg_awbc},
	[DCAM_BLOCK_AFM]         = {DCAM_BLOCK_AFM,         dcam_k_cfg_afm},
	[DCAM_BLOCK_AFL]         = {DCAM_BLOCK_AFL,         dcam_k_cfg_afl},
	[DCAM_BLOCK_LSC]         = {DCAM_BLOCK_LSC,         dcam_k_cfg_lsc},
	[DCAM_BLOCK_BPC]         = {DCAM_BLOCK_BPC,         dcam_k_cfg_bpc},
	[DCAM_BLOCK_RGBG]        = {DCAM_BLOCK_RGBG,        dcam_k_cfg_rgb_gain},
	[DCAM_BLOCK_RGBG_DITHER] = {DCAM_BLOCK_RGBG_DITHER, dcam_k_cfg_rgb_dither},
	[DCAM_BLOCK_PDAF]        = {DCAM_BLOCK_PDAF,        dcam_k_cfg_pdaf},
	[DCAM_BLOCK_3DNR_ME]     = {DCAM_BLOCK_3DNR_ME,     dcam_k_cfg_3dnr_me},
	[DCAM_BLOCK_GRGB]        = {DCAM_BLOCK_GRGB,        dcam_k_cfg_grgb},
	[DCAM_BLOCK_BAYERHIST]   = {DCAM_BLOCK_BAYERHIST,   dcamhw_k_null_cfg},
	[ISP_BLOCK_CCE]          = {ISP_BLOCK_CCE,          isp_k_cfg_cce},
	[ISP_BLOCK_NLM]          = {ISP_BLOCK_NLM,          isp_k_cfg_nlm},
	[ISP_BLOCK_CFA]          = {ISP_BLOCK_CFA,          isp_k_cfg_cfa},
	[ISP_BLOCK_YGAMMA]       = {ISP_BLOCK_YGAMMA,       isp_k_cfg_ygamma},
	[ISP_BLOCK_GAMMA]        = {ISP_BLOCK_GAMMA,        isp_k_cfg_gamma},
	[ISP_BLOCK_CMC]          = {ISP_BLOCK_CMC,          isp_k_cfg_cmc10},
	[ISP_BLOCK_PRE_CDN]      = {ISP_BLOCK_PRE_CDN,      isp_k_cfg_pre_cdn},
	[ISP_BLOCK_YNR]          = {ISP_BLOCK_YNR,          isp_k_cfg_ynr},
	[ISP_BLOCK_UVD]          = {ISP_BLOCK_UVD,          isp_k_cfg_uvd},
	[ISP_BLOCK_CDN]          = {ISP_BLOCK_CDN,          isp_k_cfg_cdn},
	[ISP_BLOCK_EDGE]         = {ISP_BLOCK_EDGE,         isp_k_cfg_edge},
	[ISP_BLOCK_POST_CDN]     = {ISP_BLOCK_POST_CDN,     isp_k_cfg_post_cdn},
	[ISP_BLOCK_IIRCNR]       = {ISP_BLOCK_IIRCNR,       isp_k_cfg_iircnr},
	[ISP_BLOCK_YRANDOM]      = {ISP_BLOCK_YRANDOM,      isp_k_cfg_yrandom},
	[ISP_BLOCK_HSV]          = {ISP_BLOCK_HSV,          isp_k_cfg_hsv},
	[ISP_BLOCK_BRIGHTNESS]   = {ISP_BLOCK_BRIGHTNESS,   isp_k_cfg_brightness},
	[ISP_BLOCK_CONTRAST]     = {ISP_BLOCK_CONTRAST,     isp_k_cfg_contrast},
	[ISP_BLOCK_CSA]          = {ISP_BLOCK_CSA,          isp_k_cfg_csa},
	[ISP_BLOCK_HUE]          = {ISP_BLOCK_HUE,          isp_k_cfg_hue},
	[ISP_BLOCK_HIST]         = {ISP_BLOCK_HIST,         isp_k_cfg_hist},
	[ISP_BLOCK_PSTRZ]        = {ISP_BLOCK_PSTRZ,        isp_k_cfg_pstrz},
};

static int camhw_get_block_fun(void *handle, void *arg)
{
	void *block_func = NULL;
	struct cam_hw_block_func_get *fucarg = NULL;

	fucarg = (struct cam_hw_block_func_get *)arg;
	if (fucarg->index < ISP_BLOCK_TOTAL) {
		block_func = (struct cam_cfg_entry *)&cam_hw_cfg_func_tab[fucarg->index];
		fucarg->cam_entry = block_func;
	}

	if (block_func == NULL)
		pr_err("fail to get valid block func %d\n", fucarg->index);

	return 0;
}

static struct hw_io_ctrl_fun cam_ioctl_fun_tab[] = {
	{CAM_HW_GET_ALL_RST,            camhw_get_all_rst},
	{CAM_HW_GET_AXI_BASE,           camhw_get_axi_base},
	{CAM_HW_GET_DCAM_DTS_CLK,       camhw_get_dcam_dts_clk},
	{CAM_HW_GET_ISP_DTS_CLK,        camhw_get_isp_dts_clk},
	{CAM_HW_GET_BLOCK_TYPE,         camhw_get_block_type},
	{CAM_HW_GET_BLK_FUN,            camhw_get_block_fun},
};

static hw_ioctl_fun camhw_ioctl_fun_get(enum cam_hw_cfg_cmd cmd)
{
	hw_ioctl_fun hw_ctrl = NULL;
	uint32_t total_num = 0;
	uint32_t i = 0;

	total_num = sizeof(cam_ioctl_fun_tab) / sizeof(struct hw_io_ctrl_fun);
	for (i = 0; i < total_num; i++) {
		if (cmd == cam_ioctl_fun_tab[i].cmd) {
			hw_ctrl = cam_ioctl_fun_tab[i].hw_ctrl;
			break;
		}
	}

	return hw_ctrl;
}
#undef CAM_HW_ADPT_LAYER

static int camhwif_dcam_ioctl(void *handle,
	enum dcam_hw_cfg_cmd cmd, void *arg)
{
	int ret = 0;
	hw_ioctl_fun hw_ctrl = NULL;

	hw_ctrl = dcamhw_ioctl_fun_get(cmd);
	if (hw_ctrl != NULL)
		ret = hw_ctrl(handle, arg);
	else
		pr_debug("not support cmd %d\n", cmd);

	return ret;
}

static int camhwif_isp_ioctl(void *handle, enum isp_hw_cfg_cmd cmd, void *arg)
{
	int ret = 0;
	hw_ioctl_fun hw_ctrl = NULL;

	hw_ctrl = isphw_ioctl_fun_get(cmd);
	if (hw_ctrl != NULL)
		ret = hw_ctrl(handle, arg);
	else
		pr_debug("not support cmd %d\n", cmd);

	return ret;
}

static int camhwif_cam_ioctl(void *handle,
	enum cam_hw_cfg_cmd cmd, void *arg)
{
	int ret = 0;
	hw_ioctl_fun hw_ctrl = NULL;

	hw_ctrl = camhw_ioctl_fun_get(cmd);
	if (hw_ctrl != NULL)
		ret = hw_ctrl(handle, arg);
	else
		pr_debug("hw_core_ctrl_fun is null, cmd %d\n", cmd);

	return ret;
}

const unsigned long slowmotion_store_addr[3][4] = {
	{
		DCAM_BIN_BASE_WADDR0,
		DCAM_BIN_BASE_WADDR1,
		DCAM_BIN_BASE_WADDR2,
		DCAM_BIN_BASE_WADDR3
	},
	{
		DCAM_AEM_BASE_WADDR,
		DCAM_AEM_BASE_WADDR1,
		DCAM_AEM_BASE_WADDR2,
		DCAM_AEM_BASE_WADDR3
	},
};

static uint32_t isp_ctx_fmcu_support[ISP_CONTEXT_HW_NUM] = {
	[ISP_CONTEXT_HW_P0] = 0,
	[ISP_CONTEXT_HW_C0] = 1,
	[ISP_CONTEXT_HW_P1] = 0,
	[ISP_CONTEXT_HW_C1] = 1,
};

static struct cam_hw_soc_info dcam_soc_info;
static struct cam_hw_soc_info isp_soc_info;
static struct cam_hw_ip_info dcam[DCAM_ID_MAX] = {
	[DCAM_ID_0] = {
		.aux_dcam_path = DCAM_PATH_BIN,
		.slm_path = BIT(PORT_BIN_OUT),
		.lbuf_share_support = 0,
		.offline_slice_support = 0,
		.superzoom_support = 1,
		.dcam_full_fbc_support = 0,
		.dcam_bin_fbc_support = 0,
		.dcam_offline_fbc_support = 0,
		.dcam_raw_path_id = DCAM_PATH_FULL,
		.bpc_raw_support = 0,
		.pyramid_support = 0,
		.fmcu_support = 0,
		.sensor_raw_fmt = CAM_RAW_PACK_10,
		.save_band_for_bigsize = 0,
		.raw_fmt_support[0] = CAM_RAW_PACK_10,
		.raw_fmt_support[1] = CAM_RAW_HALFWORD_10,
		.raw_fmt_support[2] = CAM_FORMAT_MAX,
		.dcam_zoom_mode = ZOOM_BINNING2,
		.recovery_support = DCAMINT_FATAL_ERROR,
		.dcam_output_fmt[0] = CAM_RAW_BASE,
		.store_pyr_fmt = CAM_FORMAT_MAX,
		.store_3dnr_fmt[0] = CAM_YUV420_2FRAME,
	},
	[DCAM_ID_1] = {
		.aux_dcam_path = DCAM_PATH_BIN,
		.slm_path = BIT(PORT_BIN_OUT),
		.lbuf_share_support = 0,
		.offline_slice_support = 0,
		.superzoom_support = 1,
		.dcam_full_fbc_support = 0,
		.dcam_bin_fbc_support = 0,
		.dcam_offline_fbc_support = 0,
		.dcam_raw_path_id = DCAM_PATH_FULL,
		.bpc_raw_support = 0,
		.pyramid_support = 0,
		.fmcu_support = 0,
		.sensor_raw_fmt = CAM_RAW_PACK_10,
		.save_band_for_bigsize = 0,
		.raw_fmt_support[0] = CAM_RAW_PACK_10,
		.raw_fmt_support[1] = CAM_RAW_HALFWORD_10,
		.raw_fmt_support[2] = CAM_FORMAT_MAX,
		.dcam_zoom_mode = ZOOM_BINNING2,
		.recovery_support = DCAMINT_FATAL_ERROR,
		.dcam_output_fmt[0] = CAM_RAW_BASE,
		.store_pyr_fmt = CAM_FORMAT_MAX,
		.store_3dnr_fmt[0] = CAM_YUV420_2FRAME,
	},
	[DCAM_ID_2] = {
		.aux_dcam_path = DCAM_PATH_BIN,
		.slm_path = 0,
		.lbuf_share_support = 0,
		.offline_slice_support = 0,
		.superzoom_support = 1,
		.dcam_full_fbc_support = 0,
		.dcam_bin_fbc_support = 0,
		.dcam_offline_fbc_support = 0,
		.dcam_raw_path_id = DCAM_PATH_FULL,
		.bpc_raw_support = 0,
		.pyramid_support = 0,
		.fmcu_support = 0,
		.sensor_raw_fmt = CAM_RAW_PACK_10,
		.save_band_for_bigsize = 0,
		.raw_fmt_support[0] = CAM_RAW_PACK_10,
		.raw_fmt_support[1] = CAM_RAW_HALFWORD_10,
		.raw_fmt_support[2] = CAM_FORMAT_MAX,
		.dcam_zoom_mode = ZOOM_BINNING2,
		.recovery_support = DCAMINT_FATAL_ERROR,
		.dcam_output_fmt[0] = CAM_RAW_BASE,
		.store_pyr_fmt = CAM_FORMAT_MAX,
		.store_3dnr_fmt[0] = CAM_YUV420_2FRAME,
	},
};

static struct cam_hw_ip_info isp = {
	.slm_cfg_support = 0,
	.scaler_coeff_ex = 0,
	.scaler_bypass_ctrl = 0,
	.ctx_fmcu_support = isp_ctx_fmcu_support,
	.rgb_ltm_support = 0,
	.pyr_rec_support = 0,
	.pyr_dec_support = 0,
	.fbd_yuv_support = 0,
	.rgb_gtm_support = 0,
	.frbg_hist_support = 0,
	.nr3_mv_alg_version = ALG_NR3_MV_VER_0,
	.dyn_overlap_version = ALG_ISP_DYN_OVERLAP_NONE,
	.fetch_raw_support = 1,
	.nr3_compress_support = 0,
	.capture_thumb_support = 0,
	.thumb_scaler_cal_version = ISP_THUMB_SCL_VER_0,
};

struct cam_hw_info sharkl3_hw_info = {
	.prj_id = SHARKL3,
	.pdev = NULL,
	.soc_dcam = &dcam_soc_info,
	.soc_isp = &isp_soc_info,
	.soc_dcam_lite = NULL,
	.ip_dcam[DCAM_ID_0] = &dcam[DCAM_ID_0],
	.ip_dcam[DCAM_ID_1] = &dcam[DCAM_ID_1],
	.ip_dcam[DCAM_ID_2] = &dcam[DCAM_ID_2],
	.ip_isp = &isp,
	.dcam_ioctl = camhwif_dcam_ioctl,
	.isp_ioctl = camhwif_isp_ioctl,
	.cam_ioctl = camhwif_cam_ioctl,
	.csi_connect_type = DCAM_BIND_FIXED,
};
