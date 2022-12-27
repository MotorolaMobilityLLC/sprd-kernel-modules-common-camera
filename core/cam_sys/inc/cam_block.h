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

#ifndef _CAM_BLOCK_H_
#define _CAM_BLOCK_H_

#include "cam_buf.h"
#include "dcam_blkparam.h"

struct isp_blkparam_adapt {
	uint32_t new_width;
	uint32_t new_height;
	uint32_t old_width;
	uint32_t old_height;
	uint32_t sensor_width;
	uint32_t sensor_height;
};

struct isp_k_block_param {
	uint32_t cfg_id;
	struct dcam_isp_k_block *isp_k_param;
	struct dcam_isp_k_block *isp_using_param;
};

struct dcam_isp_k_block {
	struct mutex param_lock;
	struct img_trim in_size;
	uint32_t idx;/* dcam dev idx */
	void *dev;/* dcam_pipe_dev dev */
	uint32_t cfg_id;/*isp cfg idx*/
	uint32_t dcam_slice_mode;
	uint32_t raw_fetch_count;
	uint32_t offline;
	uint32_t frm_idx;
	uint32_t is_high_fps;
	uint32_t seed0_for_mode1;
	uint32_t yrandom_mode;
	uint32_t gtm_calc_mode;
	uint32_t src_w;
	uint32_t src_h;
	uint32_t vst_buf[ISP_VST_IVST_NUM2];
	uint32_t ivst_buf[ISP_VST_IVST_NUM2];
	struct img_size sn_size;

	struct dcam_dev_lsc_param lsc;
	struct dcam_dev_blc_param blc;
	struct dcam_dev_rgb_param rgb;
	struct dcam_dev_hist_param hist;
	struct dcam_dev_hist_roi_param hist_roi;
	struct dcam_dev_aem_param aem;
	struct dcam_dev_afl_param afl;
	struct dcam_dev_awbc_param awbc;
	struct dcam_dev_bpc_param bpc;
	struct dcam_dev_grgb_param grgb;
	struct dcam_dev_3dnr_param nr3;
	struct dcam_dev_afm_param afm;
	struct dcam_dev_gamma_param_v1 gamma_info_v1;
	struct dcam_dev_gtm_param gtm;
	/* qogirn6pro rgb_gtm blocks*/
	struct dcam_dev_rgb_gtm_param rgb_gtm;
	/* qogirn6pro bpc blocks*/
	struct dcam_dev_n6pro_bpc_param bpc_n6pro;
	struct dcam_dev_lscm_param lscm;
	struct dcam_dev_pdaf_param pdaf;

	struct isp_dev_nlm_info_v2 nlm_info_base;
	struct isp_dev_nlm_imblance imbalance_info_v0_base;
	struct isp_dev_nlm_imblance_v1 imbalance_info_base;
	struct isp_dev_nlm_imblance_v2 imbalance_info_base2;
	struct isp_dev_ynr_info_v2 ynr_info_v2;
	struct isp_dev_ynr_info_v2 ynr_info_v2_base;
	struct isp_dev_3dnr_info nr3d_info;
	struct isp_dev_3dnr_info nr3_info_base;
	struct isp_dev_cfa_info_v1 cfa_info_v1;

	/* sharkl6 */
	struct isp_dev_grgb_info grgb_info;
	struct isp_dev_hist2_info hist2_info;
	struct isp_dev_bchs_info bchs_info;
	struct isp_dev_rgb_ltm_info ltm_rgb_info;
	struct isp_dev_yuv_ltm_info ltm_yuv_info;
	struct isp_dev_hsv_info_v3 hsv_info3;
	struct dcam_dev_raw_gtm_block_info gtm_rgb_info;
	struct cam_gtm_mapping gtm_sw_map_info;
	/* sharkl5/sharkl5pro diff blocks*/
	struct isp_dev_posterize_info_v2 pstrz_info_v2;
	struct isp_dev_uvd_info_v2 uvd_info_v2;
	/* qogirn6pro */
	struct isp_dev_edge_info_v3 edge_info_v3;
	struct isp_dev_uvd_info_v1 uvd_info_v1;
	struct isp_dev_3dnr_info_v1 nr3_info_base_v1;
	struct isp_dev_3dnr_info_v1 nr3d_info_v1;
	struct isp_dev_3dlut_info lut3d_info;
	struct isp_dev_post_cnr_h_info post_cnr_h_info;
	struct isp_dev_ynr_info_v3 ynr_info_v3;
	struct isp_dev_cnr_h_info cnr_info;
	struct isp_dev_dct_info dct_info;
	uint32_t ynr_radius;
	uint32_t cnr_radius;
	uint32_t dct_radius;
	/* sharkl3 only */
	struct isp_dev_brightness_info brightness_info;
	struct isp_dev_contrast_info contrast_info;
	struct isp_dev_csa_info csa_info;
	struct isp_dev_hue_info_l3 hue_info;
	/* sharkl3 diff blocks*/
	struct isp_dev_posterize_info pstrz_info;
	struct isp_dev_uvd_info uvd_info;
	struct isp_dev_ynr_info ynr_info;
	/*qogirn6l*/
	struct isp_dev_hsv_info_v4 hsv_info4;
	/* common */
	struct isp_blkparam_adapt blkparam_info;

	struct isp_dev_cce_info cce_info;
	struct isp_dev_pre_cdn_info pre_cdn_info;
	struct isp_dev_cdn_info cdn_info;
	struct isp_dev_post_cdn_info post_cdn_info;
	struct isp_dev_cfa_info cfa_info;
	struct isp_dev_cmc10_info cmc10_info;
	struct isp_dev_edge_info_v2 edge_info;
	struct isp_dev_gamma_info gamma_info;
	struct isp_dev_hsv_info_v2 hsv_info;
	struct isp_dev_iircnr_info iircnr_info;
	struct isp_dev_ygamma_info ygamma_info;
	struct isp_dev_ygamma_info_v1 ygamma_info_v1;
	struct isp_dev_yrandom_info yrandom_info;
	struct isp_dev_noise_filter_info nf_info;
};

int dcam_init_lsc_slice(void *param, uint32_t online);
int dcam_init_lsc(void *param, uint32_t online);
int dcam_update_lsc(void *param);
int dcam_k_cfg_blc(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_raw_gtm(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_raw_gtm_block(struct dcam_isp_k_block *p);
int dcam_k_raw_gtm_slice(uint32_t idx, struct dcam_dev_gtm_slice_info *gtm_slice);
int dcam_k_gtm_bypass(struct dcam_isp_k_block *param, struct dcam_dev_raw_gtm_bypass *bypass_info);
int dcam_k_cfg_rgb_gain(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_rgb_dither(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_pdaf(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_lsc(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_bayerhist(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_frgbhist(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_aem(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_gamma(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_cmc10(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_cfa(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_nlm(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_cce(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_lscm(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_afl(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_awbc(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_bpc(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_grgb(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_3dnr_me(struct isp_io_param *param, struct dcam_isp_k_block *p);
int dcam_k_cfg_afm(struct isp_io_param *param, struct dcam_isp_k_block *p);
void dcam_k_3dnr_set_roi(struct isp_img_rect rect, uint32_t project_mode, uint32_t idx);

/* for dcam driver internal */
int dcam_k_blc_block(struct dcam_isp_k_block *param);
int dcam_k_gamma_block(struct dcam_isp_k_block *param);
int dcam_k_cmc10_block(struct dcam_isp_k_block *param);
int dcam_k_cfa_block(struct dcam_isp_k_block *param);
int dcam_k_nlm_block(struct dcam_isp_k_block *param);
int dcam_k_nlm_imblance(struct dcam_isp_k_block *param);
int dcam_k_cce_block(struct dcam_isp_k_block *param);
int dcam_k_rgb_gain_block(struct dcam_isp_k_block *param);
int dcam_k_rgb_dither_random_block(struct dcam_isp_k_block *param);
int dcam_k_lsc_block(struct dcam_isp_k_block *param);
int dcam_k_bayerhist_block(struct dcam_isp_k_block *param);
int dcam_k_bayerhist_roi(struct dcam_isp_k_block *param);
int dcam_k_frgbhist_block(struct dcam_isp_k_block *param);
int dcam_k_frgbhist_roi(struct dcam_isp_k_block *param);

int dcam_k_aem_bypass(struct dcam_isp_k_block *param);
int dcam_k_aem_mode(struct dcam_isp_k_block *param);
int dcam_k_aem_win(struct dcam_isp_k_block *param);
int dcam_k_aem_skip_num(struct dcam_isp_k_block *param);
int dcam_k_aem_rgb_thr(struct dcam_isp_k_block *param);

int dcam_k_afl_block(struct dcam_isp_k_block *param);
int dcam_k_afl_bypass(struct dcam_isp_k_block *param);
int dcam_k_awbc_block(struct dcam_isp_k_block *param);
int dcam_k_awbc_gain(struct dcam_isp_k_block *param);
int dcam_k_awbc_block(struct dcam_isp_k_block *param);

int dcam_k_bpc_block(struct dcam_isp_k_block *param);
int dcam_k_bpc_ppi_param(struct dcam_isp_k_block *param);

int dcam_k_3dnr_me(struct dcam_isp_k_block *param);

int dcam_k_afm_block(struct dcam_isp_k_block *param);
int dcam_k_afm_bypass(struct dcam_isp_k_block *param);
int dcam_k_afm_win(struct dcam_isp_k_block *param);
int dcam_k_afm_win_num(struct dcam_isp_k_block *param);
int dcam_k_afm_mode(struct dcam_isp_k_block *param);
int dcam_k_afm_skipnum(struct dcam_isp_k_block *param);
int dcam_k_afm_crop_eb(struct dcam_isp_k_block *param);
int dcam_k_afm_crop_size(struct dcam_isp_k_block *param);
int dcam_k_afm_done_tilenum(struct dcam_isp_k_block *param);
int dcam_k_afm_iir_info(struct dcam_isp_k_block *param);

int dcam_k_lscm_bypass(struct dcam_isp_k_block *param);
int dcam_k_lscm_monitor(struct dcam_isp_k_block *param);

int dcam_k_pdaf(struct dcam_isp_k_block *param);

int isp_k_nlm_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_nlm_imblance(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_edge_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_post_cdn_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_precdn_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cnr_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_post_cnr_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_3dlut_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_dct_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_gamma_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cmc10_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_bchs_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_hsv_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_iircnr_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cfa_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cdn_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_uvd_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_yrandom_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_ynr_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_ygamma_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_cce_block(struct dcam_isp_k_block *isp_k_param, uint32_t idx);
int isp_k_brightness1_block(struct dcam_isp_k_block *isp_k_param,uint32_t idx);
int isp_k_contrast1_block(struct dcam_isp_k_block *isp_k_param,uint32_t idx);
int isp_k_csa1_block(struct dcam_isp_k_block *isp_k_param,uint32_t idx);
int isp_k_hue1_block(struct dcam_isp_k_block *isp_k_param,uint32_t idx);
void isp_3dnr_config_blend(uint32_t idx, struct isp_3dnr_blend_info *blend);

int isp_k_cpy_cce(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_nlm(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_cfa(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_ygamma(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_gamma(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_cmc10(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_pre_cdn(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_ynr(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_uvd(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_cdn(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_edge(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_post_cdn(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_bchs(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_cnr(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_post_cnr_h(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_3dlut(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_dct(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_iircnr(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_yrandom(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_rgb_ltm(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_yuv_ltm(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_rgb_gtm(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_hsv(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
int isp_k_cpy_brightness1(struct dcam_isp_k_block *param_block,struct dcam_isp_k_block * isp_k_param);
int isp_k_cpy_contrast1(struct dcam_isp_k_block *param_block,struct dcam_isp_k_block * isp_k_param);
int isp_k_cpy_csa1(struct dcam_isp_k_block *param_block,struct dcam_isp_k_block * isp_k_param);
int isp_k_cpy_hue1(struct dcam_isp_k_block *param_block,struct dcam_isp_k_block * isp_k_param);
int isp_k_cpy_3dnr(struct dcam_isp_k_block *param_block, struct dcam_isp_k_block *isp_k_param);
/* for dcam driver internal end */

int isp_k_cfg_nlm(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_ynr(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_3dnr(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_bchs(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_cce(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_cdn(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_dct(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_cfa(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_brightness(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_contrast(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_csa(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_hue(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_cmc10(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_edge(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_gamma(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_grgb(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_hist(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_hist2(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_hsv(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_iircnr(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_rgb_ltm(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_yuv_ltm(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_rgb_gtm(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_post_cdn(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_pre_cdn(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_pstrz(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_uvd(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_ygamma(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_yrandom(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_yuv_noisefilter(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
void cam_block_noisefilter_seeds(uint32_t image_width,
	uint32_t seed0, uint32_t *seed1, uint32_t *seed2, uint32_t *seed3);
int isp_k_cfg_3dlut(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_post_cnr_h(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_cfg_cnr(struct isp_io_param *param, struct dcam_isp_k_block *isp_k_param);
int isp_k_update_nlm(void *handle);
int isp_k_update_ynr(void *handle);
int isp_k_update_cnr(void *handle);
int isp_k_update_post_cnr(void *handle);
int isp_k_update_edge(void *handle);
int isp_k_update_dct(void *handle);
int isp_k_update_3dnr(uint32_t idx,
	struct dcam_isp_k_block *isp_k_param,
	uint32_t new_width, uint32_t old_width,
	uint32_t new_height, uint32_t old_height);
int isp_k_update_imbalance(void *handle);

int init_dcam_pm(struct dcam_isp_k_block *blk_pm_ctx);
int init_isp_pm(struct dcam_isp_k_block *isp_k_param);

int isp_ltm_config_param(void *handle);
int isp_pyr_rec_bypass(void *handle);
int isp_pyr_rec_share_config(void *handle);
int isp_pyr_rec_frame_config(void *handle);
int isp_pyr_rec_slice_config(void *handle);
int isp_pyr_rec_slice_common_config(void *handle);
int isp_k_gtm_block(void *pctx, void *param);
int pyr_dec_irq_func(void *handle);
int pyr_dec_config(void *handle);
int isp_k_gtm_mapping_set(void *param);
int isp_k_gtm_mapping_get(void *param);
int isp_k_gtm_sw_map_set(void *param);
int isp_k_rgb_gtm_bypass(void *param);
uint32_t cam_data_bits(uint32_t dcam_out_fmt);
uint32_t cam_pack_bits(uint32_t raw_out_fmt);
uint32_t cam_is_pack(uint32_t dcam_out_fmt);
uint32_t cam_format_get(uint32_t img_pix_fmt);
int camcore_raw_fmt_get(uint32_t fmt);
int dcampath_outpitch_get(uint32_t w, uint32_t dcam_out_fmt);
int dcampath_bin_scaler_get(struct img_size crop, struct img_size dst,
		uint32_t *scaler_sel, uint32_t *bin_ratio);
int cam_block_valid_fmt_get(int32_t *fmt, uint32_t default_value);

uint32_t dcamonline_portid_convert_to_pathid(uint32_t port_id);
uint32_t dcamoffline_portid_convert_to_pathid(uint32_t port_id);
uint32_t dcamoffline_pathid_convert_to_portid(uint32_t path_id);
uint32_t dcamonline_pathid_convert_to_portid(uint32_t path_id);

/* for bypass dcam,isp sub-block */
enum block_bypass {
  /* RAW RGB */
	_E_4IN1 = 0,
	_E_PDAF,
	_E_LSC,
	_E_AEM,
	_E_HIST,
	_E_AFL,
	_E_AFM,
	_E_BPC,
	_E_GRGB,
	_E_BLC,
	_E_RGB,
	_E_RAND, /* yuv random */
	_E_PPI,
	_E_AWBC,
	_E_NR3,
	_E_NLM,
	_E_VST,
	_E_IVST,
	_E_IBL,
  /* Full RGB */
	_E_GTM,
	_E_CCE,
	_E_CFA,
	_E_CMC,
	_E_GAMMA,
	_E_RGBHIST,
	_E_LSCM,
};
extern uint32_t g_dcam_bypass[];

enum isp_bypass {
	_EISP_GC = 0, /* E:enum, ISP: isp */
	_EISP_NLM,
	_EISP_VST,
	_EISP_IVST,
	_EISP_CFA,
	_EISP_CMC,
	_EISP_GAMC, /* gamma correction */
	_EISP_HSV,
	_EISP_HIST,
	_EISP_HIST2,
	_EISP_PSTRZ,
	_EISP_PRECDN,
	_EISP_YNR,
	_EISP_EE,
	_EISP_GAMY, /* Y gamma */
	_EISP_CDN,
	_EISP_POSTCDN,
	_EISP_UVD,
	_EISP_IIRCNR,
	_EISP_YRAND,
	_EISP_BCHS,
	_EISP_CONTRAST,
	_EISP_BRIGHT,
	_EISP_SATURATION,
	_EISP_HUE,
	_EISP_YUVNF,
	_EISP_DEWARP,
	_EISP_GTM,
	_EISP_3DLUT,
	_EISP_CNR,
	_EISP_POSTCNR,
	_EISP_DCT,
	_EISP_TOTAL, /* total 32 before this */
	_EISP_CCE = 33,
	_EISP_LTM = 34,
	_EISP_NR3 = 35,
	/*Attention up to 63*/
};

extern uint64_t g_isp_bypass[];

struct bypass_tag {
	char *p;/* abbreviation */
	uint32_t addr;
	uint32_t bpos;/* bit position */
	uint32_t all;/* 1: all bypass except preview path */
};

typedef int (*FUNC_DCAM_PARAM)(struct dcam_isp_k_block *param);

#endif
