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

#include "cam_block.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "blk_comm: %d %d %s : " fmt, current->pid, __LINE__, __func__

int init_dcam_pm(struct dcam_isp_k_block *blk_pm_ctx )
{
	/* bypass all blocks by default */
	blk_pm_ctx->lsc.lens_info.bypass = 1;
	blk_pm_ctx->blc.blc_info.bypass = 1;
	blk_pm_ctx->rgb.gain_info.bypass = 1;
	blk_pm_ctx->rgb.rgb_dither.random_bypass = 1;
	blk_pm_ctx->hist.bayerHist_info.hist_bypass = 1;
	blk_pm_ctx->aem.bypass = 1;
	blk_pm_ctx->afl.afl_info.bypass = 1;
	blk_pm_ctx->awbc.awbc_info.awbc_bypass = 1;
	blk_pm_ctx->pdaf.ppe_ppc_info.ppi_bypass = 1;
	blk_pm_ctx->bpc.bpc_param.bpc_info.bpc_bypass = 1;
	blk_pm_ctx->bpc.bpc_param.bpc_info_l3.bpc_bypass = 1;
	blk_pm_ctx->grgb.grgb_info.bypass = 1;
	blk_pm_ctx->nr3.nr3_me.bypass = 1;
	blk_pm_ctx->afm.bypass = 1;
	blk_pm_ctx->lscm.bypass = 1;
	blk_pm_ctx->pdaf.bypass = 1;

	return 0;
}

int init_isp_pm(struct dcam_isp_k_block *isp_k_param)
{

	isp_k_param->nlm_info_base.bypass = 1;
	isp_k_param->nlm_info_base.vst_bypass = 1;
	isp_k_param->nlm_info_base.ivst_bypass = 1;
	isp_k_param->imbalance_info_v0_base.nlm_imblance_en = 1;
	isp_k_param->ynr_info_v2_base.bypass = 1;
	isp_k_param->nr3_info_base.blend.bypass = 1;

	/* sharkl5 only */
	isp_k_param->grgb_info.bypass = 1;
	isp_k_param->bchs_info.bchs_bypass = 1;
	isp_k_param->ltm_yuv_info.ltm_stat.bypass = 1;
	isp_k_param->ltm_yuv_info.ltm_map.bypass = 1;
	/* sharkl5 diff */
	isp_k_param->pstrz_info_v2.bypass = 1;
	isp_k_param->uvd_info_v2.bypass = 1;
	isp_k_param->ynr_info_v2.bypass = 1;

	/* common */
	isp_k_param->nr3d_info.blend.bypass = 1;
	isp_k_param->cce_info.bypass = 1;
	isp_k_param->pre_cdn_info.bypass = 1;
	isp_k_param->cdn_info.bypass = 1;
	isp_k_param->post_cdn_info.bypass = 1;
	isp_k_param->cfa_info.bypass = 1;
	isp_k_param->cmc10_info.bypass = 1;
	isp_k_param->edge_info.bypass = 1;
	isp_k_param->gamma_info.bypass = 1;
	isp_k_param->hsv_info.bypass = 1;
	isp_k_param->iircnr_info.bypass = 1;
	isp_k_param->ygamma_info.bypass = 1;
	isp_k_param->yrandom_info.bypass = 1;
	isp_k_param->nf_info.yrandom_bypass= 1;

	return 0;
}
