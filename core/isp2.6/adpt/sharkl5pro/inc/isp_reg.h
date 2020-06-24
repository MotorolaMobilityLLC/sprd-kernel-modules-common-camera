/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#ifndef _ISP_REG_H_
#define _ISP_REG_H_

#define ISP_MAX_COUNT				1
#define ISP_LOGICAL_COUNT			2
#define ISP_CONTEXT_MAX				4
#define  ISP_CONTEXT_SW_MAX			9

extern uint32_t s_isp_irq_no[ISP_LOGICAL_COUNT];
extern unsigned long s_isp_regbase[ISP_MAX_COUNT];
extern unsigned long isp_phys_base[ISP_MAX_COUNT];
extern unsigned long *isp_cfg_poll_addr[ISP_CONTEXT_SW_MAX];
extern unsigned long s_isp_mmubase;


#define ISP_PHYS_ADDR(idx)		(isp_phys_base[idx])


#define ISP_P0_INT_BASE				(0x0000UL)
#define ISP_P1_INT_BASE				(0x0D00UL)
#define ISP_C0_INT_BASE				(0x0E00UL)
#define ISP_C1_INT_BASE				(0x0F00UL)

#define ISP_INT_STATUS				(0x0000UL)
#define ISP_INT_STATUS1				(0x0004UL)
#define ISP_INT_EN0				(0x0010UL)
#define ISP_INT_CLR0				(0x0014UL)
#define ISP_INT_RAW0				(0x0018UL)
#define ISP_INT_INT0				(0x001CUL)
#define ISP_INT_EN1				(0x0020UL)
#define ISP_INT_CLR1				(0x0024UL)
#define ISP_INT_RAW1				(0x0028UL)
#define ISP_INT_INT1				(0x002CUL)
#define ISP_INT_ALL_DONE_CTRL			(0x0030UL)
#define ISP_INT_SKIP_CTRL			(0x0034UL)
#define ISP_INT_SKIP_CTRL1			(0x0038UL)
#define ISP_INT_ALL_DONE_SRC_CTRL		(0x003CUL)

#define ISP_FETCH_PARAM				(0x0110UL)
#define ISP_FETCH_MEM_SLICE_SIZE		(0x0114UL)
#define ISP_FETCH_SLICE_Y_ADDR			(0x0118UL)
#define ISP_FETCH_SLICE_Y_PITCH			(0x011CUL)
#define ISP_FETCH_SLICE_U_ADDR			(0x0120UL)
#define ISP_FETCH_SLICE_U_PITCH			(0x0124UL)
#define ISP_FETCH_SLICE_V_ADDR			(0x0128UL)
#define ISP_FETCH_SLICE_V_PITCH			(0x012CUL)
#define ISP_FETCH_MIPI_INFO			(0x0130UL)
#define ISP_FETCH_LINE_DLY_CTRL			(0x0134UL)
#define ISP_FETCH_PARAM1			(0x0138UL)
#define ISP_FETCH_START				(0x013CUL)

#define ISP_DISPATCH_CH0_BAYER			(0x0310UL)
#define ISP_DISPATCH_CH0_SIZE			(0x0314UL)
#define ISP_DISPATCH_DLY			(0x0318UL)
#define ISP_DISPATCH_LINE_DLY1			(0x0320UL)
#define ISP_DISPATCH_PIPE_BUF_CTRL_CH0		(0x0324UL)
#define ISP_DISPATCH_CHK_SUM			(0x0328UL)

#define ISP_COMMON_VERSION			(0x0700UL)
#define ISP_COMMON_SPACE_SEL			(0x0710UL)
#define ISP_COMMON_SCL_PATH_SEL			(0x0714UL)
#define ISP_COMMON_FMCU0_PATH_SEL		(0x0718UL)
#define ISP_COMMON_GCLK_CTRL_0			(0x071CUL)
#define ISP_COMMON_GCLK_CTRL_1			(0x0720UL)
#define ISP_COMMON_GCLK_CTRL_2			(0x0724UL)
#define ISP_COMMON_GCLK_CTRL_3			(0x0728UL)
#define ISP_COMMON_FMCU1_PATH_SEL		(0x072CUL)
#define ISP_COMMON_SHADOW_CTRL_CH0		(0x0730UL)
#define ISP_COMMON_PMU_RAM_MASK			(0x0734UL)
#define ISP_WORK_CTRL				(0x0738UL)
#define ISP_BLOCK_MODE				(0x073CUL)

#define ISP_CORE_STATUS				(0x8000UL)
#define ISP_CORE_PMU_EN				(0x8010UL)
#define ISP_CORE_BUSY_ON			(0x8014UL)

#define ISP_CFG_STATUS0				(0x8100UL)
#define ISP_CFG_STATUS1				(0x8104UL)
#define ISP_CFG_STATUS2				(0x8108UL)
#define ISP_CFG_STATUS3				(0x810CUL)
#define ISP_CFG_PAMATER				(0x8110UL)
#define ISP_CFG_TM_NUM				(0x8114UL)
#define ISP_CFG_CAP0_TH				(0x8118UL)
#define ISP_CFG_CAP1_TH				(0x811CUL)
#define ISP_CFG_PRE0_CMD_ADDR			(0x8120UL)
#define ISP_CFG_PRE1_CMD_ADDR			(0x8124UL)
#define ISP_CFG_CAP0_CMD_ADDR			(0x8128UL)
#define ISP_CFG_CAP1_CMD_ADDR			(0x812CUL)
#define ISP_CFG_PRE0_START			(0x8130UL)
#define ISP_CFG_PRE1_START			(0x8134UL)
#define ISP_CFG_CAP0_START			(0x8138UL)
#define ISP_CFG_CAP1_START			(0x813CUL)
#define ISP_CFG_AUTO_CG_EN			(0x8140UL)
#define ISP_CFG_CAP_FMCU_RDY			(0x8144UL)
#define ISP_CFG_STATUS4				(0x8150UL)
#define ISP_CFG0_BUF				(0x36C00UL)
#define ISP_CFG1_BUF				(0x36E00UL)

#define ISP_FMCU0_BASE				(0xF000UL)
#define ISP_FMCU1_BASE				(0xF100UL)

#define ISP_FMCU_CTRL				(0x0014UL)
#define ISP_FMCU_DDR_ADDR			(0x0018UL)
#define ISP_FMCU_AHB_ARB			(0x001CUL)
#define ISP_FMCU_START				(0x0020UL)
#define ISP_FMCU_TIME_OUT_THD			(0x0024UL)
#define ISP_FMCU_CMD_READY			(0x0028UL)
#define ISP_FMCU_ISP_REG_REGION			(0x002CUL)
#define ISP_FMCU_CMD				(0x0030UL)
#define ISP_FMCU_STOP				(0x0034UL)
#define ISP_FMCU_SW_TRIGGER			(0x003CUL)

#define ISP_BWU_PARAM				(0x1A10UL)

#define ISP_GRGB_STATUS				(0x1B00UL)
#define ISP_GRGB_CTRL				(0x1B10UL)
#define ISP_GRGB_CFG0				(0x1B14UL)
#define ISP_GRGB_LUM_FLAT_T			(0x1B18UL)
#define ISP_GRGB_LUM_FLAT_R			(0x1B1CUL)
#define ISP_GRGB_LUM_EDGE_T			(0x1B20UL)
#define ISP_GRGB_LUM_EDGE_R			(0x1B24UL)
#define ISP_GRGB_LUM_TEX_T			(0x1B28UL)
#define ISP_GRGB_LUM_TEX_R			(0x1B2CUL)
#define ISP_GRGB_FREZ_FLAT_T			(0x1B30UL)
#define ISP_GRGB_FREZ_FLAT_R			(0x1B34UL)
#define ISP_GRGB_FREZ_EDGE_T			(0x1B38UL)
#define ISP_GRGB_FREZ_EDGE_R			(0x1B3CUL)
#define ISP_GRGB_FREZ_TEX_T			(0x1B40UL)
#define ISP_GRGB_FREZ_TEX_R			(0x1B44UL)

#define ISP_VST_PARA				(0x1C10UL)
#define ISP_IVST_PARA				(0x1E10UL)
#define ISP_NLM_PARA				(0x2010UL)
#define ISP_NLM_MODE_CNT			(0x2014UL)
#define ISP_NLM_SIMPLE_BPC			(0x2018UL)
#define ISP_NLM_LUM_THRESHOLD			(0x201CUL)
#define ISP_NLM_DIRECTION_TH			(0x2020UL)
#define ISP_NLM_IS_FLAT				(0x2024UL)
#define ISP_NLM_LUT_W_0				(0x2028UL)
#define ISP_NLM_LUM0_FLAT0_PARAM		(0x2088UL)
#define ISP_NLM_LUM0_FLAT0_ADDBACK		(0x208CUL)
#define ISP_NLM_LUM0_FLAT1_PARAM		(0x2090UL)
#define ISP_NLM_LUM0_FLAT1_ADDBACK		(0x2094UL)
#define ISP_NLM_LUM0_FLAT2_PARAM		(0x2098UL)
#define ISP_NLM_LUM0_FLAT2_ADDBACK		(0x209CUL)
#define ISP_NLM_LUM0_FLAT3_PARAM		(0x20A0UL)
#define ISP_NLM_LUM0_FLAT3_ADDBACK		(0x20A4UL)
#define ISP_NLM_ADDBACK3			(0x20E8UL)
#define ISP_NLM_RADIAL_1D_PARAM			(0x20ECUL)
#define ISP_NLM_RADIAL_1D_DIST			(0x20F0UL)
#define ISP_NLM_RADIAL_1D_THRESHOLD		(0x20F4UL)
#define ISP_NLM_RADIAL_1D_THR0			(0x20F8UL)
#define ISP_NLM_RADIAL_1D_RATIO			(0x211CUL)
#define ISP_NLM_RADIAL_1D_GAIN_MAX		(0x214CUL)
#define ISP_NLM_RADIAL_1D_ADDBACK00		(0x2150UL)
#define ISP_NLM_IMBLANCE_CTRL			(0x1F10UL)
#define ISP_NLM_IMBLANCE_PARA1			(0x1F14UL)
#define ISP_NLM_IMBLANCE_PARA2			(0x1F18UL)
#define ISP_NLM_IMBLANCE_PARA3			(0x1F1CUL)
#define ISP_NLM_IMBLANCE_PARA4			(0x1F20UL)
#define ISP_NLM_IMBLANCE_PARA5			(0x1F24UL)
#define ISP_NLM_IMBLANCE_PARA6			(0x1F28UL)
#define ISP_NLM_IMBLANCE_PARA7			(0x1F2CUL)
#define ISP_NLM_IMBLANCE_PARA8			(0x1F30UL)
#define ISP_NLM_IMBLANCE_PARA9			(0x1F34UL)
#define ISP_NLM_IMBLANCE_PARA10			(0x1F38UL)
#define ISP_NLM_IMBLANCE_PARA11			(0x1F3CUL)
#define ISP_NLM_IMBLANCE_PARA12			(0x1F40UL)
#define ISP_NLM_IMBLANCE_PARA13			(0x1F44UL)
#define ISP_NLM_IMBLANCE_PARA14			(0x1F48UL)
#define ISP_NLM_IMBLANCE_PARA15			(0x1F4CUL)
#define ISP_NLM_IMBLANCE_PARA16			(0x1F50UL)
#define ISP_NLM_IMBLANCE_PARA17			(0x1F54UL)
#define ISP_NLM_IMBLANCE_PARA18			(0x1F58UL)
#define ISP_NLM_IMBLANCE_PARA19			(0x1F5CUL)
#define ISP_NLM_IMBLANCE_PARA20			(0x1F60UL)
#define ISP_NLM_IMBLANCE_PARA21			(0x1F64UL)
#define ISP_NLM_IMBLANCE_PARA22			(0x1F68UL)
#define ISP_NLM_IMBLANCE_PARA23			(0x1F6CUL)
#define ISP_NLM_IMBLANCE_PARA24			(0x1F70UL)
#define ISP_NLM_IMBLANCE_PARA25			(0x1F74UL)
#define ISP_NLM_IMBLANCE_PARA26			(0x1F78UL)
#define ISP_NLM_IMBLANCE_PARA27			(0x1F7CUL)
#define ISP_NLM_IMBLANCE_PARA28			(0x1F80UL)
#define ISP_NLM_IMBLANCE_PARA29			(0x1F84UL)
#define ISP_NLM_IMBLANCE_PARA30			(0x1F88UL)

#define ISP_VST_BUF0_ADDR			(0x19000UL)
#define ISP_IVST_BUF0_ADDR			(0x29000UL)
#define ISP_VST_BUF1_ADDR			(0x22000UL)
#define ISP_IVST_BUF1_ADDR			(0x2B000UL)

#define ISP_CFAE_STATUS				(0x3000UL)
#define ISP_CFAE_NEW_CFG0			(0x3010UL)
#define ISP_CFAE_INTP_CFG0			(0x3014UL)
#define ISP_CFAE_INTP_CFG1			(0x3018UL)
#define ISP_CFAE_INTP_CFG2			(0x301CUL)
#define ISP_CFAE_INTP_CFG3			(0x3020UL)
#define ISP_CFAE_INTP_CFG4			(0x3024UL)
#define ISP_CFAE_INTP_CFG5			(0x3028UL)
#define ISP_CFAE_CSS_CFG0			(0x302CUL)
#define ISP_CFAE_CSS_CFG1			(0x3030UL)
#define ISP_CFAE_CSS_CFG2			(0x3034UL)
#define ISP_CFAE_CSS_CFG3			(0x3038UL)
#define ISP_CFAE_CSS_CFG4			(0x303CUL)
#define ISP_CFAE_CSS_CFG5			(0x3040UL)
#define ISP_CFAE_CSS_CFG6			(0x3044UL)
#define ISP_CFAE_CSS_CFG7			(0x3048UL)
#define ISP_CFAE_CSS_CFG8			(0x304CUL)
#define ISP_CFAE_CSS_CFG9			(0x3050UL)
#define ISP_CFAE_CSS_CFG10			(0x3054UL)
#define ISP_CFAE_CSS_CFG11			(0x3058UL)

#define ISP_CMC10_PARAM				(0x3110UL)
#define ISP_CMC10_MATRIX0			(0x3114UL)
#define ISP_CMC10_MATRIX1			(0x3118UL)
#define ISP_CMC10_MATRIX2			(0x311CUL)
#define ISP_CMC10_MATRIX3			(0x3120UL)
#define ISP_CMC10_MATRIX4			(0x3124UL)

#define ISP_GAMMA_STATUS			(0x3200UL)
#define ISP_GAMMA_PARAM				(0x3210UL)
#define ISP_FGAMMA_R_BUF0			(0x1B000UL)
#define ISP_FGAMMA_G_BUF0			(0x1C000UL)
#define ISP_FGAMMA_B_BUF0			(0x1D000UL)
#define ISP_FGAMMA_R_BUF1			(0x24000UL)
#define ISP_FGAMMA_G_BUF1			(0x25000UL)
#define ISP_FGAMMA_B_BUF1			(0x26000UL)

#define ISP_HSV_PARAM				(0x3310UL)
#define ISP_HSV_CFG0				(0x3314UL)
#define ISP_HSV_CFG1				(0x3318UL)
#define ISP_HSV_CFG2				(0x331CUL)
#define ISP_HSV_CFG3				(0x3320UL)
#define ISP_HSV_CFG4				(0x3324UL)
#define ISP_HSV_CFG5				(0x3328UL)
#define ISP_HSV_CFG6				(0x332CUL)
#define ISP_HSV_CFG7				(0x3330UL)
#define ISP_HSV_CFG8				(0x3334UL)
#define ISP_HSV_CFG9				(0x3338UL)
#define ISP_HSV_CFG10				(0x333CUL)
#define ISP_HSV_CFG11				(0x3340UL)
#define ISP_HSV_CFG12				(0x3344UL)
#define ISP_HSV_CFG13				(0x3348UL)
#define ISP_HSV_CFG14				(0x334CUL)
#define ISP_HSV_CFG15				(0x3350UL)
#define ISP_HSV_CFG16				(0x3354UL)
#define ISP_HSV_CFG17				(0x3358UL)
#define ISP_HSV_CFG18				(0x335cUL)
#define ISP_HSV_CFG19				(0x3360UL)
#define ISP_HSV_CFG20				(0x3364UL)
#define ISP_HSV_CFG21				(0x3368UL)
#define ISP_HSV_BUF0_ADDR			(0x18000UL)
#define ISP_HSV_BUF1_ADDR			(0x21000UL)

#define ISP_HIST2_STATUS			(0x3700UL)
#define ISP_HIST2_PARAM				(0x3710UL)
#define ISP_HIST2_ROI_S0			(0x3714UL)
#define ISP_HIST2_ROI_E0			(0x3718UL)
#define ISP_HIST2_CFG_RDY			(0x371CUL)
#define ISP_HIST2_SKIP_CLR			(0x3720UL)
#define ISP_HIST2_BUF0_ADDR			(0x16000UL)

#define ISP_PSTRZ_PARAM				(0x3410UL)
#define ISP_PSTRZ_R_BUF0			(0x1b400UL)
#define ISP_PSTRZ_G_BUF0			(0x1c400UL)
#define ISP_PSTRZ_B_BUF0			(0x1d400UL)
#define ISP_PSTRZ_R_BUF1			(0x24400UL)
#define ISP_PSTRZ_G_BUF1			(0x25400UL)
#define ISP_PSTRZ_B_BUF1			(0x26400UL)

#define ISP_CCE_STATUS0				(0x3500UL)
#define ISP_CCE_STATUS1				(0x3504UL)
#define ISP_CCE_PARAM				(0x3510UL)
#define ISP_CCE_MATRIX0				(0x3514UL)
#define ISP_CCE_MATRIX1				(0x3518UL)
#define ISP_CCE_MATRIX2				(0x351CUL)
#define ISP_CCE_MATRIX3				(0x3520UL)
#define ISP_CCE_MATRIX4				(0x3524UL)
#define ISP_CCE_SHIFT				(0x3528UL)

#define ISP_UVD_STATUS0				(0x3600UL)
#define ISP_UVD_STATUS1				(0x3604UL)
#define ISP_UVD_PARAM				(0x3610UL)
#define ISP_UVD_PARAM0				(0x3614UL)
#define ISP_UVD_PARAM1				(0x3618UL)
#define ISP_UVD_PARAM2				(0x361CUL)
#define ISP_UVD_PARAM3				(0x3620UL)
#define ISP_UVD_PARAM4				(0x3624UL)
#define ISP_UVD_PARAM5				(0x3628UL)

#define ISP_HIST_PARAM				(0x5410UL)
#define ISP_HIST_CFG_READY			(0x5414UL)
#define ISP_HIST_SKIP_NUM_CLR			(0x5418UL)

#define ISP_LTM_HIST_RGB_BASE		(0x3800UL)
#define ISP_LTM_HIST_YUV_BASE		(0x5500UL)

#define ISP_LTM_HISTS_STATUS			(0x0000UL)
#define ISP_LTM_HIST_PARAM			(0x0010UL)
#define ISP_LTM_ROI_START			(0x0014UL)
#define ISP_LTM_TILE_RANGE			(0x0018UL)
#define ISP_LTM_CLIP_LIMIT			(0x0020UL)
#define ISP_LTM_THRES				(0x0024UL)
#define ISP_LTM_ADDR				(0x0028UL)
#define ISP_LTM_PITCH				(0x002CUL)
#define ISP_LTM_RGB_HIST_BUF0_ADDR			(0x1F000UL)
#define ISP_LTM_RGB_HIST_BUF1_ADDR			(0x28000UL)
#define ISP_LTM_YUV_HIST_BUF0_ADDR			(0x1F400UL)
#define ISP_LTM_YUV_HIST_BUF1_ADDR			(0x28400UL)

#define ISP_LTM_MAP_RGB_BASE		(0x3900UL)
#define ISP_LTM_MAP_YUV_BASE		(0x5F00UL)

#define ISP_LTM_MAP_PARAM0			(0x0010UL)
#define ISP_LTM_MAP_PARAM1			(0x0014UL)
#define ISP_LTM_MAP_PARAM2			(0x0018UL)
#define ISP_LTM_MAP_PARAM3			(0x001cUL)
#define ISP_LTM_MAP_PARAM4			(0x0020UL)
#define ISP_LTM_MAP_PARAM5			(0x0024UL)

#define ISP_PRECDN_STATUS0			(0x5000UL)
#define ISP_PRECDN_STATUS1			(0x5004UL)
#define ISP_PRECDN_PARAM			(0x5010UL)
#define ISP_PRECDN_MEDIAN_THRUV01		(0x5014UL)
#define ISP_PRECDN_THRYUV			(0x5018UL)
#define ISP_PRECDN_SEGU_0			(0x501CUL)
#define ISP_PRECDN_SEGU_1			(0x5020UL)
#define ISP_PRECDN_SEGU_2			(0x5024UL)
#define ISP_PRECDN_SEGU_3			(0x5028UL)
#define ISP_PRECDN_SEGV_0			(0x502CUL)
#define ISP_PRECDN_SEGV_1			(0x5030UL)
#define ISP_PRECDN_SEGV_2			(0x5034UL)
#define ISP_PRECDN_SEGV_3			(0x5038UL)
#define ISP_PRECDN_SEGY_0			(0x503CUL)
#define ISP_PRECDN_SEGY_1			(0x5040UL)
#define ISP_PRECDN_SEGY_2			(0x5044UL)
#define ISP_PRECDN_SEGY_3			(0x5048UL)
#define ISP_PRECDN_DISTW0			(0x504CUL)
#define ISP_PRECDN_DISTW1			(0x5050UL)
#define ISP_PRECDN_DISTW2			(0x5054UL)
#define ISP_PRECDN_DISTW3			(0x5058UL)

#define ISP_YNR_CONTRL0				(0x5110UL)
#define ISP_YNR_CFG0				(0x5114UL)
#define ISP_YNR_CFG1				(0x5118UL)
#define ISP_YNR_CFG2				(0x511CUL)
#define ISP_YNR_CFG3				(0x5120UL)
#define ISP_YNR_CFG4				(0x5124UL)
#define ISP_YNR_CFG5				(0x5128UL)
#define ISP_YNR_CFG6				(0x512CUL)
#define ISP_YNR_CFG7				(0x5130UL)
#define ISP_YNR_CFG8				(0x5134UL)
#define ISP_YNR_CFG9				(0x5138UL)
#define ISP_YNR_CFG10				(0x513CUL)
#define ISP_YNR_CFG11				(0x5140UL)
#define ISP_YNR_CFG12				(0x5144UL)
#define ISP_YNR_CFG13				(0x5148UL)
#define ISP_YNR_CFG14				(0x514cUL)
#define ISP_YNR_CFG15				(0x5150UL)
#define ISP_YNR_CFG16				(0x5154UL)
#define ISP_YNR_CFG17				(0x5158UL)
#define ISP_YNR_CFG18				(0x515cUL)
#define ISP_YNR_CFG19				(0x5160UL)
#define ISP_YNR_CFG20				(0x5164UL)
#define ISP_YNR_CFG21				(0x5168UL)
#define ISP_YNR_CFG22				(0x516cUL)
#define ISP_YNR_CFG23				(0x5170UL)
#define ISP_YNR_CFG24				(0x5174UL)
#define ISP_YNR_CFG25				(0x5178UL)
#define ISP_YNR_CFG26				(0x517cUL)
#define ISP_YNR_CFG27				(0x5180UL)
#define ISP_YNR_CFG28				(0x5184UL)
#define ISP_YNR_CFG29				(0x5188UL)
#define ISP_YNR_CFG30				(0x518cUL)
#define ISP_YNR_CFG31				(0x5190UL)
#define ISP_YNR_CFG32				(0x5194UL)
#define ISP_YNR_CFG33				(0x5198UL)

#define ISP_CDN_STATUS0				(0x5600UL)
#define ISP_CDN_STATUS1				(0x5604UL)
#define ISP_CDN_PARAM				(0x5610UL)
#define ISP_CDN_THRUV				(0x5614UL)
#define ISP_CDN_U_RANWEI_0			(0x5618UL)
#define ISP_CDN_U_RANWEI_7			(0x5634UL)
#define ISP_CDN_V_RANWEI_0			(0x5638UL)
#define ISP_CDN_V_RANWEI_7			(0x5654UL)

#define ISP_EE_STATUS				(0x5700UL)
#define ISP_EE_PARAM				(0x5710UL)
#define ISP_EE_CFG0				(0x5714UL)
#define ISP_EE_CFG1				(0x5718UL)
#define ISP_EE_CFG2				(0x571CUL)
#define ISP_EE_ADP_CFG0				(0x5720UL)
#define ISP_EE_ADP_CFG1				(0x5724UL)
#define ISP_EE_ADP_CFG2				(0x5728UL)
#define ISP_EE_IPD_CFG0				(0x572CUL)
#define ISP_EE_IPD_CFG1				(0x5730UL)
#define ISP_EE_IPD_CFG2				(0x5734UL)
#define ISP_EE_LUM_CFG0				(0x5738UL)
#define ISP_EE_LUM_CFG1				(0x573CUL)
#define ISP_EE_LUM_CFG2				(0x5740UL)
#define ISP_EE_LUM_CFG3				(0x5744UL)
#define ISP_EE_LUM_CFG4				(0x5748UL)
#define ISP_EE_LUM_CFG5				(0x574CUL)
#define ISP_EE_LUM_CFG6				(0x5750UL)
#define ISP_EE_LUM_CFG7				(0x5754UL)
#define ISP_EE_LUM_CFG8				(0x5758UL)
#define ISP_EE_LUM_CFG9				(0x575CUL)
#define ISP_EE_LUM_CFG10			(0x5760UL)
#define ISP_EE_LUM_CFG11			(0x5764UL)
#define ISP_EE_LUM_CFG12			(0x5768UL)
#define ISP_EE_LUM_CFG13			(0x576CUL)
#define ISP_EE_LUM_CFG14			(0x5770UL)
#define ISP_EE_LUM_CFG15			(0x5774UL)
#define ISP_EE_LUM_CFG16			(0x5778UL)
#define ISP_EE_LUM_CFG17			(0x577CUL)
#define ISP_EE_LUM_CFG18			(0x5780UL)
#define ISP_EE_RATIO				(0x5784UL)
#define ISP_EE_OFFSET_THR_LAYER0_POS		(0x5788UL)
#define ISP_EE_OFFSET_RATIO_LAYER0_CURVE_POS	(0x578CUL)
#define ISP_EE_OFFSET_CLIP_LAYER0_POS		(0x5790UL)
#define ISP_EE_OFFSET_THR_LAYER0_NEG		(0x5794UL)
#define ISP_EE_OFFSET_RATIO_LAYER0_CURVE_NEG	(0x5798UL)
#define ISP_EE_OFFSET_CLIP_LAYER0_CURVE_NEG	(0x579CUL)
#define ISP_EE_OFFSET_RATIO_LAYER0_LUM_CURVE	(0x57A0UL)
#define ISP_EE_OFFSET_RATIO_LAYER0_FREQ_CURVE	(0x57A4UL)
#define ISP_EE_OFFSET_THR_LAYER1_POS		(0x57A8UL)
#define ISP_EE_OFFSET_RATIO_LAYER1_CURVE_POS	(0x57ACUL)
#define ISP_EE_OFFSET_CLIP_LAYER1_POS		(0x57B0UL)
#define ISP_EE_OFFSET_THR_LAYER1_NEG		(0x57B4UL)
#define ISP_EE_OFFSET_RATIO_LAYER1_CURVE_NEG	(0x57B8UL)
#define ISP_EE_OFFSET_CLIP_LAYER1_CURVE_NEG	(0x57BCUL)
#define ISP_EE_OFFSET_RATIO_LAYER1_LUM_CURVE	(0x57C0UL)
#define ISP_EE_OFFSET_RATIO_LAYER1_FREQ_CURVE	(0x57C4UL)
#define ISP_EE_OFFSET_THR_LAYER2_POS		(0x57C8UL)
#define ISP_EE_OFFSET_RATIO_LAYER2_CURVE_POS	(0x57CCUL)
#define ISP_EE_OFFSET_CLIP_LAYER2_POS		(0x57D0UL)
#define ISP_EE_OFFSET_THR_LAYER2_NEG		(0x57D4UL)
#define ISP_EE_OFFSET_RATIO_LAYER2_CURVE_NEG	(0x57D8UL)
#define ISP_EE_OFFSET_CLIP_LAYER2_CURVE_NEG	(0x57DCUL)
#define ISP_EE_OFFSET_RATIO_LAYER2_LUM_CURVE	(0x57E0UL)
#define ISP_EE_OFFSET_RATIO_LAYER2_FREQ_CURVE	(0x57E4UL)

#define ISP_POSTCDN_STATUS			(0x5A00UL)
#define ISP_POSTCDN_COMMON_CTRL			(0x5A10UL)
#define ISP_POSTCDN_ADPT_THR			(0x5A14UL)
#define ISP_POSTCDN_UVTHR			(0x5A18UL)
#define ISP_POSTCDN_THRU			(0x5A1CUL)
#define ISP_POSTCDN_THRV			(0x5A20UL)
#define ISP_POSTCDN_RSEGU01			(0x5A24UL)
#define ISP_POSTCDN_RSEGU23			(0x5A28UL)
#define ISP_POSTCDN_RSEGU45			(0x5A2CUL)
#define ISP_POSTCDN_RSEGU6			(0x5A30UL)
#define ISP_POSTCDN_RSEGV01			(0x5A34UL)
#define ISP_POSTCDN_RSEGV23			(0x5A38UL)
#define ISP_POSTCDN_RSEGV45			(0x5A3CUL)
#define ISP_POSTCDN_RSEGV6			(0x5A40UL)
#define ISP_POSTCDN_R_DISTW0			(0x5A44UL)
#define ISP_POSTCDN_SLICE_CTRL			(0x5A80UL)

#define ISP_YGAMMA_STATUS			(0x5B00UL)
#define ISP_YGAMMA_PARAM			(0x5B10UL)
#define ISP_YGAMMA_BUF0				(0x1E000UL)
#define ISP_YGAMMA_BUF1				(0x27000UL)

#define ISP_YDELAY_STATUS			(0x5C00UL)
#define ISP_YDELAY_PARAM			(0x5C10UL)
#define ISP_YDELAY_STEP				(0x5C14UL)

#define ISP_IIRCNR_STATUS0			(0x5D00UL)
#define ISP_IIRCNR_STATUS1			(0x5D04UL)
#define ISP_IIRCNR_PARAM			(0x5D10UL)
#define ISP_IIRCNR_PARAM1			(0x5D14UL)
#define ISP_IIRCNR_PARAM2			(0x5D18UL)
#define ISP_IIRCNR_PARAM3			(0x5D1CUL)
#define ISP_IIRCNR_PARAM4			(0x5D20UL)
#define ISP_IIRCNR_PARAM5			(0x5D24UL)
#define ISP_IIRCNR_PARAM6			(0x5D28UL)
#define ISP_IIRCNR_PARAM7			(0x5D2CUL)
#define ISP_IIRCNR_PARAM8			(0x5D30UL)
#define ISP_IIRCNR_PARAM9			(0x5D34UL)
#define ISP_YUV_IIRCNR_NEW_0			(0x5D38UL)
#define ISP_YUV_IIRCNR_NEW_1			(0x5D3CUL)
#define ISP_YUV_IIRCNR_NEW_2			(0x5D40UL)
#define ISP_YUV_IIRCNR_NEW_3			(0x5D44UL)
#define ISP_YUV_IIRCNR_NEW_4			(0x5D48UL)
#define ISP_YUV_IIRCNR_NEW_5			(0x5D4CUL)
#define ISP_YUV_IIRCNR_NEW_6			(0x5D50UL)
#define ISP_YUV_IIRCNR_NEW_7			(0x5D54UL)
#define ISP_YUV_IIRCNR_NEW_8			(0x5D58UL)
#define ISP_YUV_IIRCNR_NEW_9			(0x5D5CUL)
#define ISP_YUV_IIRCNR_NEW_10			(0x5D60UL)
#define ISP_YUV_IIRCNR_NEW_11			(0x5D64UL)
#define ISP_YUV_IIRCNR_NEW_12			(0x5D68UL)
#define ISP_YUV_IIRCNR_NEW_13			(0x5D6CUL)
#define ISP_YUV_IIRCNR_NEW_14			(0x5D70UL)
#define ISP_YUV_IIRCNR_NEW_15			(0x5D74UL)
#define ISP_YUV_IIRCNR_NEW_16			(0x5D78UL)
#define ISP_YUV_IIRCNR_NEW_17			(0x5D7CUL)
#define ISP_YUV_IIRCNR_NEW_18			(0x5D80UL)
#define ISP_YUV_IIRCNR_NEW_19			(0x5D84UL)
#define ISP_YUV_IIRCNR_NEW_20			(0x5D88UL)
#define ISP_YUV_IIRCNR_NEW_21			(0x5D8CUL)
#define ISP_YUV_IIRCNR_NEW_22			(0x5D90UL)
#define ISP_YUV_IIRCNR_NEW_23			(0x5D94UL)
#define ISP_YUV_IIRCNR_NEW_24			(0x5D98UL)
#define ISP_YUV_IIRCNR_NEW_25			(0x5D9CUL)
#define ISP_YUV_IIRCNR_NEW_26			(0x5DA0UL)
#define ISP_YUV_IIRCNR_NEW_27			(0x5DA4UL)
#define ISP_YUV_IIRCNR_NEW_28			(0x5DA8UL)
#define ISP_YUV_IIRCNR_NEW_29			(0x5DACUL)
#define ISP_YUV_IIRCNR_NEW_30			(0x5DB0UL)
#define ISP_YUV_IIRCNR_NEW_31			(0x5DB4UL)
#define ISP_YUV_IIRCNR_NEW_32			(0x5DB8UL)
#define ISP_YUV_IIRCNR_NEW_33			(0x5DBCUL)
#define ISP_YUV_IIRCNR_NEW_34			(0x5DC0UL)
#define ISP_YUV_IIRCNR_NEW_35			(0x5DC4UL)
#define ISP_YUV_IIRCNR_NEW_36			(0x5DC8UL)
#define ISP_YUV_IIRCNR_NEW_37			(0x5DCCUL)
#define ISP_YUV_IIRCNR_NEW_38			(0x5DD0UL)
#define ISP_YUV_IIRCNR_NEW_39			(0x5DD4UL)

#define ISP_YRANDOM_PARAM1			(0x5E10UL)
#define ISP_YRANDOM_PARAM2			(0x5E14UL)
#define ISP_YRANDOM_PARAM3			(0x5E18UL)
#define ISP_YRANDOM_INIT			(0x5E24UL)

#define ISP_BCHS_PARAM				(0x5910UL)
#define ISP_CSA_FACTOR				(0x5914UL)
#define ISP_HUA_FACTOR				(0x5918UL)
#define ISP_BRTA_FACTOR				(0x5920UL)
#define ISP_CNTA_FACTOR				(0x5924UL)

#define ISP_3DNR_MEM_CTRL_PARAM0		(0x9010UL)
#define ISP_3DNR_MEM_CTRL_LINE_MODE		(0x9014UL)
#define ISP_3DNR_MEM_CTRL_PARAM1		(0x9018UL)
#define ISP_3DNR_MEM_CTRL_PARAM2		(0x901CUL)
#define ISP_3DNR_MEM_CTRL_PARAM3		(0x9020UL)
#define ISP_3DNR_MEM_CTRL_PARAM4		(0x9024UL)
#define ISP_3DNR_MEM_CTRL_PARAM5		(0x9028UL)
#define ISP_3DNR_MEM_CTRL_PARAM7		(0x902CUL)
#define ISP_3DNR_MEM_CTRL_FT_CUR_LUMA_ADDR	(0x9030UL)
#define ISP_3DNR_MEM_CTRL_FT_CUR_CHROMA_ADDR	(0x9034UL)
#define ISP_3DNR_MEM_CTRL_FT_CTRL_PITCH		(0x9038UL)
#define ISP_3DNR_MEM_CTRL_PARAM8		(0x903CUL)
#define ISP_3DNR_MEM_CTRL_PARAM9		(0x9040UL)
#define ISP_3DNR_MEM_CTRL_PARAM10		(0x9044UL)
#define ISP_3DNR_MEM_CTRL_PARAM11		(0x9048UL)
#define ISP_3DNR_MEM_CTRL_PARAM12		(0x904CUL)
#define ISP_3DNR_MEM_CTRL_PARAM13		(0x9050UL)

#define ISP_3DNR_BLEND_CONTROL0			(0x9110UL)
#define ISP_3DNR_BLEND_CFG1			(0x9114UL)
#define ISP_3DNR_BLEND_CFG2			(0x9118UL)
#define ISP_3DNR_BLEND_CFG3			(0x911CUL)
#define ISP_3DNR_BLEND_CFG4			(0x9120UL)
#define ISP_3DNR_BLEND_CFG5			(0x9124UL)
#define ISP_3DNR_BLEND_CFG6			(0x9128UL)
#define ISP_3DNR_BLEND_CFG7			(0x912CUL)
#define ISP_3DNR_BLEND_CFG8			(0x9130UL)
#define ISP_3DNR_BLEND_CFG9			(0x9134UL)
#define ISP_3DNR_BLEND_CFG10			(0x9138UL)
#define ISP_3DNR_BLEND_CFG11			(0x913CUL)
#define ISP_3DNR_BLEND_CFG12			(0x9140UL)
#define ISP_3DNR_BLEND_CFG13			(0x9144UL)
#define ISP_3DNR_BLEND_CFG14			(0x9148UL)
#define ISP_3DNR_BLEND_CFG15			(0x914CUL)
#define ISP_3DNR_BLEND_CFG16			(0x9150UL)
#define ISP_3DNR_BLEND_CFG17			(0x9154UL)
#define ISP_3DNR_BLEND_CFG18			(0x9158UL)
#define ISP_3DNR_BLEND_CFG19			(0x915CUL)
#define ISP_3DNR_BLEND_CFG20			(0x9160UL)
#define ISP_3DNR_BLEND_CFG21			(0x9164UL)
#define ISP_3DNR_BLEND_CFG22			(0x9168UL)
#define ISP_3DNR_BLEND_CFG23			(0x916CUL)
#define ISP_3DNR_BLEND_CFG24			(0x9170UL)

#define ISP_3DNR_STORE_PARAM			(0x9210UL)
#define ISP_3DNR_STORE_SIZE			(0x9214UL)
#define ISP_3DNR_STORE_LUMA_ADDR		(0x9218UL)
#define ISP_3DNR_STORE_CHROMA_ADDR		(0x921CUL)
#define ISP_3DNR_STORE_PITCH			(0x9220UL)
#define ISP_3DNR_STORE_SHADOW_CLR		(0x9224UL)

#define ISP_3DNR_MEM_CTRL_PRE_PARAM0		(0x9310UL)
#define ISP_3DNR_MEM_CTRL_PRE_PARAM1		(0x9314UL)
#define ISP_3DNR_MEM_CTRL_PRE_PARAM2		(0x9318UL)
#define ISP_3DNR_MEM_CTRL_PRE_PARAM3		(0x931CUL)

#define ISP_FBC_3DNR_STORE_PARAM		(0x9410UL)
#define ISP_FBC_3DNR_STORE_SLICE_SIZE		(0x9414UL)
#define ISP_FBC_3DNR_STORE_SLICE_YADDR		(0x9418UL)
#define ISP_FBC_3DNR_STORE_SLICE_CADDR		(0x941CUL)
#define ISP_FBC_3DNR_STORE_TILE_PITCH		(0x9420UL)
#define ISP_FBC_3DNR_STORE_SLICE_YHEADER	(0x9424UL)
#define ISP_FBC_3DNR_STORE_SLICE_CHEADER	(0x9428UL)
#define ISP_FBC_3DNR_STORE_CONSTANT		(0x942CUL)
#define ISP_FBC_3DNR_STORE_BITS			(0x9430UL)
#define ISP_FBC_3DNR_STORE_TILE_NUM		(0x9434UL)
#define ISP_FBC_3DNR_STORE_NFULL_LEVEL		(0x9438UL)

#define ISP_FBD_NR3_PARAM0			(0x9510UL)
#define ISP_FBD_NR3_PARAM1			(0x9514UL)
#define ISP_FBD_NR3_PARAM2			(0x9518UL)
#define ISP_FBD_NR3_T_ADDR_Y			(0x951CUL)
#define ISP_FBD_NR3_H_ADDR_Y			(0x9520UL)
#define ISP_FBD_NR3_T_ADDR_C			(0x9524UL)
#define ISP_FBD_NR3_H_ADDR_C			(0x9528UL)
#define ISP_FBD_NR3_SIZE_Y			(0x952CUL)
#define ISP_FBD_NR3_SIZE_C			(0x9530UL)
#define ISP_FBD_NR3_START_Y			(0x9534UL)
#define ISP_FBD_NR3_START_C			(0x9538UL)
#define ISP_FBD_NR3_TILE_SIZE_Y			(0x953CUL)
#define ISP_FBD_NR3_TILE_SIZE_C			(0x9540UL)
#define ISP_FBD_NR3_SLICE_TILE_PARAM		(0x9544UL)
#define ISP_FBD_NR3_READ_SPECIAL		(0x9548UL)

#define ISP_SCALER_PRE_CAP_BASE			(0xC000UL)
#define ISP_SCALER_VID_BASE			(0xD000UL)
#define ISP_SCALER_THUMB_BASE			(0xE000UL)

#define ISP_SCALER_CFG				(0x0010UL)
#define ISP_SCALER_SRC_SIZE			(0x0014UL)
#define ISP_SCALER_DES_SIZE			(0x0018UL)
#define ISP_SCALER_TRIM0_START			(0x001CUL)
#define ISP_SCALER_TRIM0_SIZE			(0x0020UL)
#define ISP_SCALER_IP				(0x0024UL)
#define ISP_SCALER_CIP				(0x0028UL)
#define ISP_SCALER_FACTOR			(0x002CUL)
#define ISP_SCALER_TRIM1_START			(0x0030UL)
#define ISP_SCALER_TRIM1_SIZE			(0x0034UL)
#define ISP_SCALER_VER_IP			(0x0038UL)
#define ISP_SCALER_VER_CIP			(0x003CUL)
#define ISP_SCALER_VER_FACTOR			(0x0040UL)
#define ISP_SCALER_DEBUG			(0x0044UL)
#define ISP_SCALER_HBLANK			(0x0048UL)
#define ISP_SCALER_FRAME_CNT_CLR		(0x004CUL)
#define ISP_SCALER_RES				(0x0050UL)
#define ISP_SCALER_SHRINK_CFG			(0x0054UL)
#define ISP_SCALER_EFFECT_CFG			(0x0058UL)
#define ISP_SCALER_REGULAR_CFG			(0x005CUL)

#define ISP_SCALER_PRE_LUMA_HCOEFF_BUF0		(0x39100UL)
#define ISP_SCALER_PRE_CHROMA_HCOEFF_BUF0	(0x39300UL)
#define ISP_SCALER_PRE_LUMA_VCOEFF_BUF0		(0x394F0UL)
#define ISP_SCALER_PRE_CHROMA_VCOEFF_BUF0	(0x39AF0UL)

#define ISP_SCALER_VID_LUMA_HCOEFF_BUF0		(0x38100UL)
#define ISP_SCALER_VID_CHROMA_HCOEFF_BUF0	(0x38300UL)
#define ISP_SCALER_VID_LUMA_VCOEFF_BUF0		(0x384F0UL)
#define ISP_SCALER_VID_CHROMA_VCOEFF_BUF0	(0x38AF0UL)

#define ISP_SCALER_PRE_LUMA_HCOEFF_BUF1		(0x39200UL)
#define ISP_SCALER_PRE_CHROMA_HCOEFF_BUF1	(0x39380UL)
#define ISP_SCALER_PRE_LUMA_VCOEFF_BUF1		(0x397F0UL)
#define ISP_SCALER_PRE_CHROMA_VCOEFF_BUF1	(0x39DF0UL)

#define ISP_SCALER_VID_LUMA_HCOEFF_BUF1		(0x38200UL)
#define ISP_SCALER_VID_CHROMA_HCOEFF_BUF1	(0x38380UL)
#define ISP_SCALER_VID_LUMA_VCOEFF_BUF1		(0x387F0UL)
#define ISP_SCALER_VID_CHROMA_VCOEFF_BUF1	(0x38DF0UL)

#define ISP_STORE_DEBUG_BASE			(0x0200UL)
#define ISP_STORE_PRE_CAP_BASE			(0xC100UL)
#define ISP_STORE_VID_BASE			(0xD100UL)
#define ISP_STORE_THUMB_BASE			(0xE100UL)

#define ISP_STORE_PARAM				(0x0010UL)
#define ISP_STORE_SLICE_SIZE			(0x0014UL)
#define ISP_STORE_BORDER			(0x0018UL)
#define ISP_STORE_SLICE_Y_ADDR			(0x001CUL)
#define ISP_STORE_Y_PITCH			(0x0020UL)
#define ISP_STORE_SLICE_U_ADDR			(0x0024UL)
#define ISP_STORE_U_PITCH			(0x0028UL)
#define ISP_STORE_SLICE_V_ADDR			(0x002CUL)
#define ISP_STORE_V_PITCH			(0x0030UL)
#define ISP_STORE_READ_CTRL			(0x0034UL)
#define ISP_STORE_SHADOW_CLR_SEL		(0x0038UL)
#define ISP_STORE_SHADOW_CLR			(0x003CUL)

#define ISP_FBC_STORE1_BASE			(0xC300UL)
#define ISP_FBC_STORE2_BASE			(0xD300UL)

#define ISP_FBC_STORE_PARAM			(0x0010UL)
#define ISP_FBC_STORE_SLICE_SIZE		(0x0014UL)
#define ISP_FBC_STORE_BORDER			(0x0018UL)
#define ISP_FBC_STORE_SLICE_YADDR		(0x001CUL)
#define ISP_FBC_STORE_SLICE_CADDR		(0x0020UL)
#define ISP_FBC_STORE_P0			(0x0024UL)
#define ISP_FBC_STORE_TILE_PITCH		(0x0028UL)
#define ISP_FBC_STORE_SLICE_YHEADER		(0x002CUL)
#define ISP_FBC_STORE_SLICE_CHEADER		(0x0030UL)
#define ISP_FBC_STORE_CONSTANT			(0x0034UL)
#define ISP_FBC_STORE_TILE_NUM			(0x0038UL)
#define ISP_FBC_STORE_NFULL_LEVEL		(0x003CUL)

#define ISP_AFBC_STORE_PARAM                           (0x0010UL)
#define ISP_AFBC_STORE_SLICE_SIZE                      (0x0014UL)
#define ISP_AFBC_STORE_BORDER                          (0x0018UL)
#define ISP_AFBC_STORE_SLICE_HEADER_OFFSET_ADDR        (0x001CUL)
#define ISP_AFBC_STORE_SLICE_Y_ADDR                    (0x0020UL)
#define ISP_AFBC_STORE_SLICE_Y_HEADER                  (0x0024UL)
#define ISP_AFBC_STORE_TILE_PITCH                      (0x0028UL)
#define ISP_AFBC_STORE_NFULL_LEVEL                     (0x002CUL)
#define ISP_AFBC_STORE_P0                              (0x0030UL)

#define ISP_YUV_NF_CTRL				(0xC210UL)
#define ISP_YUV_NF_SEED0			(0xC214UL)
#define ISP_YUV_NF_SEED1			(0xC218UL)
#define ISP_YUV_NF_SEED2			(0xC21CUL)
#define ISP_YUV_NF_SEED3			(0xC220UL)
#define ISP_YUV_NF_TB4				(0xC224UL)
#define ISP_YUV_NF_SF				(0xC228UL)
#define ISP_YUV_NF_THR				(0xC22CUL)
#define ISP_YUV_NF_CV_T12			(0xC230UL)
#define ISP_YUV_NF_CV_R				(0xC234UL)
#define ISP_YUV_NF_CLIP				(0xC238UL)
#define ISP_YUV_NF_SEED0_OUT			(0xC23CUL)
#define ISP_YUV_NF_SEED_INIT			(0xC24CUL)
#define ISP_YUV_NF_CV_T34			(0xC250UL)

#define ISP_THMB_CFG				(0xE010UL)
#define ISP_THMB_BEFORE_TRIM_SIZE		(0xE014UL)
#define ISP_THMB_Y_SLICE_SRC_SIZE		(0xE018UL)
#define ISP_THMB_Y_DES_SIZE			(0xE01CUL)
#define ISP_THMB_Y_TRIM0_START			(0xE020UL)
#define ISP_THMB_Y_TRIM0_SIZE			(0xE024UL)
#define ISP_THMB_Y_INIT_PHASE			(0xE028UL)
#define ISP_THMB_Y_FACTOR_HOR			(0xE02CUL)
#define ISP_THMB_Y_FACTOR_VER			(0xE030UL)
#define ISP_THMB_UV_SLICE_SRC_SIZE		(0xE034UL)
#define ISP_THMB_UV_DES_SIZE			(0xE038UL)
#define ISP_THMB_UV_TRIM0_START			(0xE03CUL)
#define ISP_THMB_UV_TRIM0_SIZE			(0xE040UL)
#define ISP_THMB_UV_INIT_PHASE			(0xE044UL)
#define ISP_THMB_UV_FACTOR_HOR			(0xE048UL)
#define ISP_THMB_UV_FACTOR_VER			(0xE04CUL)
#define ISP_THMB_SHRINK_CFG			(0xE050UL)
#define ISP_THMB_EFFECT_CFG			(0xE054UL)
#define ISP_THMB_REGULAR_CFG			(0xE058UL)
#define ISP_THMB_SCL_DEBUG			(0xE05CUL)
#define ISP_THMB_FRM_CNT_CLR			(0xE060UL)

#define ISP_FBD_RAW_SEL				(0x0C10UL)
#define ISP_FBD_RAW_SLICE_SIZE			(0x0C14UL)
#define ISP_FBD_RAW_PARAM0			(0x0C18UL)
#define ISP_FBD_RAW_PARAM1			(0x0C1CUL)
#define ISP_FBD_RAW_PARAM2			(0x0C20UL)
#define ISP_FBD_RAW_PARAM3			(0x0C24UL)
#define ISP_FBD_RAW_PARAM4			(0x0C28UL)
#define ISP_FBD_RAW_PARAM5			(0x0C2CUL)
#define ISP_FBD_RAW_PARAM6			(0x0C30UL)
#define ISP_FBD_RAW_LOW_PARAM0			(0x0C34UL)
#define ISP_FBD_RAW_LOW_PARAM1			(0x0C38UL)
#define ISP_FBD_RAW_START			(0x0C3CUL)
#define ISP_FBD_RAW_HBLANK			(0x0C40UL)
#define ISP_FBD_RAW_LOW_4BIT_PARAM0		(0x0C44UL)
#define ISP_FBD_RAW_LOW_4BIT_PARAM1		(0x0C48UL)

#define ISP_ARBITER_WR_STATUS			(0x0400UL)
#define ISP_ARBITER_RD_STATUS			(0x0404UL)
#define ISP_ARBITER_ENDIAN_COMM			(0x0410UL)
#define ISP_ARBITER_ENDIAN0			(0x0414UL)
#define ISP_ARBITER_ENDIAN1			(0x0418UL)
#define ISP_ARBITER_WR_PARAM0			(0x041CUL)
#define ISP_ARBITER_WR_PARAM1			(0x0420UL)
#define ISP_ARBITER_WR_PARAM2			(0x0424UL)
#define ISP_ARBITER_WR_PARAM3			(0x0428UL)
#define ISP_ARBITER_WR_PARAM4			(0x042CUL)
#define ISP_ARBITER_RD_PARAM0			(0x043CUL)
#define ISP_ARBITER_RD_PARAM1			(0x0440UL)
#define ISP_ARBITER_RD_PARAM2			(0x0444UL)
#define ISP_ARBITER_RD_PARAM3			(0x0448UL)
#define ISP_ARBITER_RD_PARAM4			(0x044CUL)
#define ISP_ARBITER_RD_PARAM5			(0x0450UL)
#define ISP_ARBITER_RD_PARAM6			(0x0454UL)
#define ISP_ARBITER_CHK_SUM_CLR			(0x0460UL)
#define ISP_ARBITER_CHK_SUM0			(0x0464UL)

#define ISP_AXI_WR_MASTER_STATUS		(0x0500UL)
#define ISP_AXI_RD_MASTER_STATUS		(0x0504UL)
#define ISP_AXI_ITI2AXIM_CTRL			(0x0510UL)
#define ISP_AXI_ARBITER_WQOS			(0x0514UL)
#define ISP_AXI_ARBITER_RQOS			(0x0518UL)
#define ISP_AXI_ISOLATION			(0x051CUL)
#define ISP_AXI_PARAM0				(0x0520UL)
#define ISP_AXI_PARAM1				(0x0524UL)
#define ISP_AXI_PARAM2				(0x0528UL)
#define ISP_AXI_PARAM3				(0x052CUL)
#define ISP_AXI_CACHE_ACTIVE			(0x052CUL)
#define ISP_AXI_CHK_SUM_CTRL			(0x052CUL)
#define ISP_AXI_TIMEOUT_PARAM			(0x0558UL)

#define ISP_MMU_VERSION                 (0x0000UL)
#define ISP_MMU_EN                      (0x0004UL)
#define ISP_MMU_INT_EN                  (0x00A0UL)
#define ISP_MMU_INT_CLR                 (0x00A4UL)
#define ISP_MMU_INT_STS                 (0x00A8UL)
#define ISP_MMU_INT_RAW                 (0x00ACUL)

/******************************************************************************/


#define ISP_BASE_ADDR(idx)  (*(isp_cfg_poll_addr[idx]))
#define ISP_GET_REG(reg)  (ISP_PHYS_ADDR(0) + (reg))

#define ISP_REG_WR(idx, reg, val)  (REG_WR(ISP_BASE_ADDR(idx) + (reg), (val)))
#define ISP_REG_RD(idx, reg)  (REG_RD(ISP_BASE_ADDR(idx) + (reg)))

#define ISP_REG_MWR(idx, reg, msk, val)  (ISP_REG_WR((idx), (reg), \
		((val) & (msk)) | \
		(ISP_REG_RD((idx), (reg)) & (~(msk)))))

#define ISP_REG_OWR(idx, reg, val)  \
		(ISP_REG_WR((idx), (reg),\
			(ISP_REG_RD((idx), (reg)) | (val))))


/* won't access CFG buffers */
#define ISP_HREG_WR(reg, val) \
		(REG_WR(s_isp_regbase[0] + (reg), (val)))

#define ISP_HREG_RD(reg) \
		(REG_RD(s_isp_regbase[0] + (reg)))

#define ISP_HREG_MWR(reg, msk, val) \
		(REG_WR(s_isp_regbase[0] + (reg), \
		((val) & (msk)) | \
		(REG_RD(s_isp_regbase[0] + (reg)) & \
		(~(msk)))))

#define ISP_HREG_OWR(reg, val) \
		(REG_WR(s_isp_regbase[0] + (reg), \
			(REG_RD(s_isp_regbase[0] + (reg)) | (val))))


/* To access ISP IOMMU  Registers*/
#define ISP_MMU_BASE s_isp_mmubase
#define ISP_MMU_WR(reg, val)             (REG_WR(ISP_MMU_BASE+(reg), (val)))
#define ISP_MMU_RD(reg)                  (REG_RD(ISP_MMU_BASE+(reg)))
#define ISP_MMU_MWR(reg, msk, val) \
	ISP_MMU_WR((reg), ((val) & (msk)) | (ISP_MMU_RD(reg) & (~(msk))))

#endif
