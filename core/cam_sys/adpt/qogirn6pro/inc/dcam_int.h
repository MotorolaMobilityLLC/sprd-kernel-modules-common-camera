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

#ifndef _DCAM_INT_H_
#define _DCAM_INT_H_

#include <linux/bitops.h>
#include <linux/device.h>
#include "cam_types.h"

enum {
	DCAM_IF_IRQ_INT0_SENSOR_SOF              = 0,
	DCAM_IF_IRQ_INT0_SENSOR_EOF              = 1,
	DCAM_IF_IRQ_INT0_CAP_SOF                 = 2,
	DCAM_IF_IRQ_INT0_CAP_EOF                 = 3,
	DCAM_IF_IRQ_INT0_DCAM_OVF                = 4,
	DCAM_IF_IRQ_INT0_CAP_LINE_ERR            = 5,
	DCAM_IF_IRQ_INT0_CAP_FRM_ERR             = 6,
	DCAM_IF_IRQ_INT0_FETCH_SOF_INT           = 7,
	DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE        = 8,
	DCAM_IF_IRQ_INT0_PREIEW_SOF              = 9,
	DCAM_IF_IRQ_INT0_PDAF_SOF                = 10,
	DCAM_IF_IRQ_INT0_AEM_SOF                 = 11,
	DCAM_IF_IRQ_INT0_HIST_SOF                = 12,
	DCAM_IF_IRQ_INT0_AFL_LAST_SOF            = 13,
	DCAM_IF_IRQ_INT0_AFM_SOF                 = 14,
	DCAM_IF_IRQ_INT0_LSCM_SOF                = 15,
	DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE    = 16,
	DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE    = 17,
	DCAM_IF_IRQ_INT0_PDAF_PATH_TX_DONE       = 18,
	DCAM_IF_IRQ_INT0_VCH2_PATH_TX_DONE       = 19,
	DCAM_IF_IRQ_INT0_VCH3_PATH_TX_DONE       = 20,
	DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE        = 21,
	DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE = 22,
	DCAM_IF_IRQ_INT0_AFL_TX_DONE             = 23,
	DCAM_IF_IRQ_INT0_BPC_MAP_DONE            = 24,
	DCAM_IF_IRQ_INT0_BPC_POS_DONE            = 25,
	DCAM_IF_IRQ_INT0_AFM_INTREQ0             = 26,
	DCAM_IF_IRQ_INT0_AFM_INTREQ1             = 27,
	DCAM_IF_IRQ_INT0_NR3_TX_DONE             = 28,
	DCAM_IF_IRQ_INT0_LSCM_TX_DONE            = 29,
	DCAM_IF_IRQ_INT0_GTM_DONE                = 30,
	DCAM_IF_IRQ_INT0_MMU_INT                 = 31,
	DCAM_IF_IRQ_INT0_NUMBER
};

enum {
	DCAM_IF_IRQ_INT1_FRGB_HIST_DONE          = 0,
	DCAM_IF_IRQ_INT1_DEC_DONE                = 1,
	DCAM_IF_IRQ_INT1_DEC1_DONE               = 2,
	DCAM_IF_IRQ_INT1_DEC2_DONE               = 3,
	DCAM_IF_IRQ_INT1_DEC3_DONE               = 4,
	DCAM_IF_IRQ_INT1_DEC4_DONE               = 5,
	DCAM_IF_IRQ_INT1_SENSOR_SOF1,
	DCAM_IF_IRQ_INT1_SENSOR_SOF2,
	DCAM_IF_IRQ_INT1_SENSOR_SOF3,
	DCAM_IF_IRQ_INT1_SENSOR_EOF1,
	DCAM_IF_IRQ_INT1_SENSOR_EOF2,
	DCAM_IF_IRQ_INT1_SENSOR_EOF3,
	DCAM_IF_IRQ_INT1_FRGB_HIST_SOF,
	DCAM_IF_IRQ_INT1_DUMMY_START,
	DCAM_IF_IRQ_INT1_CAP_CSI_SOF,
	DCAM_IF_IRQ_INT1_CAP_CSI_EOF,
	DCAM_IF_IRQ_INT1_FMCU_INT6,
	DCAM_IF_IRQ_INT1_FMCU_INT5,
	DCAM_IF_IRQ_INT1_FMCU_INT4,
	DCAM_IF_IRQ_INT1_FMCU_INT3,
	DCAM_IF_IRQ_INT1_FMCU_INT2              = 20,
	DCAM_IF_IRQ_INT1_FMCU_INT1,
	DCAM_IF_IRQ_INT1_FMCU_INT0,
	DCAM_IF_IRQ_INT1_AWURGENT0,
	DCAM_IF_IRQ_INT1_AWURGENT1,
	DCAM_IF_IRQ_INT1_DUMMY_DONE,
	DCAM_IF_IRQ_INT1_NUMBER
};

/* interrupt bits for DCAM2 */
enum {
	DCAM2_SENSOR_SOF        = 0,
	DCAM2_SENSOR_EOF        = 1,
	/* reserved */
	/* reserved */

	DCAM2_DCAM_OVF          = 4,
	/* reserved */
	/* reserved */
	/* reserved */

	/* reserved */
	/* reserved */
	DCAM2_CAP_LINE_ERR      = 10,
	DCAM2_CAP_FRM_ERR       = 11,

	DCAM2_FULL_PATH0_END    = 12,
	DCAM2_FULL_PATH1_END    = 13,
	/* reserved */
	/* reserved */

	/* reserved */
	/* reserved */
	DCAM2_FULL_PATH_TX_DONE = 18,
	/* reserved */

	/* reserved */
	/* reserved */
	/* reserved */
	/* reserved */

	DCAM2_MMU_INT           = 24,
	/* reserved */
	/* reserved */
	/* reserved */

	/* reserved */
	/* reserved */
	/* reserved */
	/* reserved */
};

#define DCAMINT_INT0_FBC \
	(BIT(DCAM_IF_IRQ_INT0_CAP_SOF) | \
	 BIT(DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE) | \
	 BIT(DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE))

#define DCAMINT_INT0_ERROR \
	(BIT(DCAM_IF_IRQ_INT0_DCAM_OVF) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_LINE_ERR) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_FRM_ERR) | \
	BIT(DCAM_IF_IRQ_INT0_MMU_INT))

#define DCAMINT_INT0_FATAL_ERROR \
	(BIT(DCAM_IF_IRQ_INT0_DCAM_OVF) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_LINE_ERR) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_FRM_ERR))

#define DCAMINT_INT0_SOF \
	(BIT(DCAM_IF_IRQ_INT0_SENSOR_SOF) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_SOF) | \
	BIT(DCAM_IF_IRQ_INT0_PREIEW_SOF))

#define DCAMINT_INT0_TX_DONE \
		(BIT(DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_RAW_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_PDAF_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_VCH2_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_VCH3_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_AEM_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_BAYER_HIST_PATH_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_AFL_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_BPC_MAP_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_BPC_POS_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_AFM_INTREQ0) | \
		 BIT(DCAM_IF_IRQ_INT0_AFM_INTREQ1) | \
		 BIT(DCAM_IF_IRQ_INT0_LSCM_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_NR3_TX_DONE) | \
		 BIT(DCAM_IF_IRQ_INT0_GTM_DONE))

#define DCAMINT_IRQ_LINE_INT0_MASK \
	(DCAMINT_INT0_ERROR | DCAMINT_INT0_TX_DONE | \
	BIT(DCAM_IF_IRQ_INT0_CAP_SOF) | \
	BIT(DCAM_IF_IRQ_INT0_CAP_EOF) | \
	BIT(DCAM_IF_IRQ_INT0_PREIEW_SOF) | \
	BIT(DCAM_IF_IRQ_INT0_SENSOR_EOF) | \
	BIT(DCAM_IF_IRQ_INT0_SENSOR_SOF))

#define DCAMINT_IRQ_LINE_EN0_NORMAL \
	(DCAMINT_INT0_ERROR | DCAMINT_INT0_TX_DONE | \
	BIT(DCAM_IF_IRQ_INT0_CAP_SOF) | \
	BIT(DCAM_IF_IRQ_INT0_SENSOR_EOF))

/* for dcam it test */
#define DCAM_PREV_PATH_TX_DONE       DCAM_IF_IRQ_INT0_PREVIEW_PATH_TX_DONE
#define DCAM_FULL_PATH_TX_DONE       DCAM_IF_IRQ_INT0_CAPTURE_PATH_TX_DONE
#define DCAMINT_IRQ_LINE_MASK        DCAMINT_IRQ_LINE_INT0_MASK
#define DCAMINT_IRQ_LINE_NORMAL      DCAMINT_IRQ_LINE_EN0_NORMAL
#define DCAMINT_ALL_ERROR            DCAMINT_INT0_ERROR
#define DCAM_CAP_SOF                 DCAM_IF_IRQ_INT0_CAP_SOF

#define DCAMINT_INT1_SOF \
	(BIT(DCAM_IF_IRQ_INT1_SENSOR_SOF1) | \
	 BIT(DCAM_IF_IRQ_INT1_SENSOR_SOF2) | \
	 BIT(DCAM_IF_IRQ_INT1_SENSOR_SOF3))

#define DCAMINT_INT1_TX_DONE \
	(BIT(DCAM_IF_IRQ_INT1_FRGB_HIST_DONE) | \
	 BIT(DCAM_IF_IRQ_INT1_DEC_DONE) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT1))

#define DCAMINT_IRQ_LINE_INT1_MASK \
	(DCAMINT_INT1_TX_DONE | DCAMINT_INT1_SOF | \
	 BIT(DCAM_IF_IRQ_INT1_DUMMY_START) | \
	 BIT(DCAM_IF_IRQ_INT1_DUMMY_DONE) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT1) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT2) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT4) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT5) | \
	 BIT(DCAM_IF_IRQ_INT1_FMCU_INT6))

#define DCAMINT_IRQ_LINE_EN1_NORMAL  DCAMINT_IRQ_LINE_INT1_MASK

#define DCAMINT_IRQ_LINE_EN0_SLM \
	(DCAMINT_INT0_ERROR | DCAMINT_INT0_TX_DONE | \
	BIT(DCAM_IF_IRQ_INT0_PREIEW_SOF))

#define DCAMINT_IRQ_LINE_EN1_SLM  DCAMINT_IRQ_LINE_INT1_MASK

struct nr3_done {
	uint32_t hw_ctx;
	uint32_t p;
	uint32_t out0;
	uint32_t out1;
};

struct dcam_irq_info {
	uint32_t irq_num;
	uint32_t status;
	uint32_t status1;
};

int dcam_int_irq_request(struct device *pdev, int irq, void *param);
void dcam_int_irq_free(struct device *pdev, void *param);
int dcam_int_irq_desc_get(uint32_t index, void *param);
void dcam_int_tracker_reset(uint32_t idx);
void dcam_int_tracker_dump(uint32_t idx);
irqreturn_t dcamint_error_handler(void *dcam_hw_ctx, uint32_t status);
void dcamint_dcam_status_rw(struct dcam_irq_info irq_info, void *priv);

#endif
