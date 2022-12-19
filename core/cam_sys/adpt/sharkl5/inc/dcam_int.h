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

/* interrupt bits for DCAM0 or DCAM1 */
enum {
	DCAM_SENSOR_SOF         = 0,
	DCAM_SENSOR_EOF         = 1,
	DCAM_CAP_SOF            = 2,
	DCAM_CAP_EOF            = 3,

	DCAM_DCAM_OVF           = 4,
	DCAM_PREVIEW_SOF        = 5,
	DCAM_AFL_LAST_SOF       = 6,
	DCAM_FETCH_SOF_INT      = 7,

	DCAM_ISP_ENABLE_PULSE   = 8,
	DCAM_BPC_MEM_ERR        = 9,
	DCAM_CAP_LINE_ERR       = 10,
	DCAM_CAP_FRM_ERR        = 11,

	DCAM_FULL_PATH_END      = 12,
	DCAM_BIN_PATH_END       = 13,
	/* reserved */
	/* reserved */

	DCAM_FULL_PATH_TX_DONE  = 16,
	DCAM_PREV_PATH_TX_DONE  = 17,
	DCAM_PDAF_PATH_TX_DONE  = 18,
	DCAM_VCH2_PATH_TX_DONE  = 19,

	DCAM_VCH3_PATH_TX_DONE  = 20,
	DCAM_AEM_TX_DONE        = 21,
	DCAM_HIST_TX_DONE       = 22,
	DCAM_AFL_TX_DONE        = 23,

	DCAM_BPC_MAP_DONE       = 24,
	DCAM_BPC_POS_DONE       = 25,
	DCAM_AFM_INTREQ0        = 26,
	DCAM_AFM_INTREQ1        = 27,

	DCAM_NR3_TX_DONE        = 28,
	/* reserved */
	/* reserved */
	DCAM_MMU_INT            = 31,

	DCAM_IRQ_NUMBER         = 32,
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

enum {
	/* record frame count and addr to determine the source port of the exception
	 * and frame fid, when dcam mmu happens. */
	DCAM_RECORD_PORT_FULL,
	DCAM_RECORD_PORT_BIN,
	DCAM_RECORD_PORT_BIN_1,
	DCAM_RECORD_PORT_BIN_2,
	DCAM_RECORD_PORT_BIN_3,
	DCAM_RECORD_PORT_PDAF,
	DCAM_RECORD_PORT_VCH2,
	DCAM_RECORD_PORT_VCH3,
	DCAM_RECORD_PORT_LENS,
	DCAM_RECORD_PORT_AEM,
	DCAM_RECORD_PORT_AEM_SHORT,
	DCAM_RECORD_PORT_HIST,
	DCAM_RECORD_PORT_PPE,
	DCAM_RECORD_PORT_AFL_GLB,
	DCAM_RECORD_PORT_AFL_REGION,
	DCAM_RECORD_PORT_BPC_MAP,
	DCAM_RECORD_PORT_BPC_OUT,
	DCAM_RECORD_PORT_AFM,
	DCAM_RECORD_PORT_NR3,
	DCAM_RECORD_PORT_IMG_FETCH,
	DCAM_RECORD_FRAME_CUR_COUNT,
	DCAM_RECORD_FRAME_CNT_SUM,
	DCAM_RECORD_PORT_INFO_MAX
};

/*
 * error bits
 * same in DCAM0/1/2
 */
#define DCAMINT_ALL_ERROR                          \
	(BIT(DCAM_DCAM_OVF) |                      \
	BIT(DCAM_CAP_LINE_ERR) |                   \
	BIT(DCAM_CAP_FRM_ERR) |                    \
	BIT(DCAM_MMU_INT))

/*
 * fatal error bits
 */
#define DCAMINT_FATAL_ERROR                        \
	(BIT(DCAM_DCAM_OVF) |                      \
	BIT(DCAM_CAP_LINE_ERR) |                   \
	BIT(DCAM_CAP_FRM_ERR))

/*
 * SOF bits
 * some bits is reserved in DCAM2
 */
#define DCAMINT_ALL_SOF                            \
	(BIT(DCAM_SENSOR_SOF) |                    \
	BIT(DCAM_CAP_SOF) |                        \
	BIT(DCAM_PREVIEW_SOF) |                    \
	BIT(DCAM_AFL_LAST_SOF))

/*
 * TX DONE bits
 * some bits is reserved in DCAM2
 * NOTE: BIT(24) is mmu_int in DCAM2
 */
#define DCAMINT_ALL_TX_DONE                        \
	(BIT(DCAM_FULL_PATH_TX_DONE) |             \
	BIT(DCAM_PREV_PATH_TX_DONE) |              \
	BIT(DCAM_PDAF_PATH_TX_DONE) |              \
	BIT(DCAM_VCH2_PATH_TX_DONE) |              \
	BIT(DCAM_VCH3_PATH_TX_DONE) |              \
	BIT(DCAM_AEM_TX_DONE) |                    \
	BIT(DCAM_HIST_TX_DONE) |                   \
	BIT(DCAM_AFL_TX_DONE) |                    \
	BIT(DCAM_BPC_MAP_DONE) |                   \
	BIT(DCAM_BPC_POS_DONE) |                   \
	BIT(DCAM_AFM_INTREQ0) |                    \
	BIT(DCAM_AFM_INTREQ1) |                    \
	BIT(DCAM_NR3_TX_DONE))

/*
 * all currently useful bits on irq line
 */
#define DCAMINT_IRQ_LINE_MASK                      \
	(DCAMINT_ALL_ERROR | DCAMINT_ALL_TX_DONE | \
	BIT(DCAM_CAP_SOF) |                        \
	BIT(DCAM_SENSOR_SOF) |                    \
	BIT(DCAM_SENSOR_EOF) |                     \
	BIT(DCAM_PREVIEW_SOF))

/* enabled interrupt source in normal scene */
#define DCAMINT_IRQ_LINE_EN_NORMAL                 \
	(DCAMINT_ALL_ERROR | DCAMINT_ALL_TX_DONE | \
	BIT(DCAM_CAP_SOF) |                        \
	BIT(DCAM_SENSOR_EOF))

#define DCAM2INT_IRQ_LINE_MASK                     \
	(DCAMINT_FATAL_ERROR | BIT(DCAM2_MMU_INT)  \
	| BIT(DCAM2_FULL_PATH_TX_DONE) |           \
	BIT(DCAM2_SENSOR_EOF) |                    \
	BIT(DCAM2_SENSOR_SOF))

#define DCAM2INT_IRQ_LINE_EN_NORMAL                \
	(DCAMINT_FATAL_ERROR | BIT(DCAM2_MMU_INT)  \
	| BIT(DCAM2_FULL_PATH_TX_DONE) |           \
	BIT(DCAM2_SENSOR_EOF) |                    \
	BIT(DCAM2_SENSOR_SOF))

/*
 * enabled interrupt source in slow motion scene
 *
 * Note: this one is deprecated as we have a design defect in current DCAM IP.
 * The address written into slow motion register in DCAM_PREVIEW_SOF cannot be
 * applied by hardware because DCAM will only shadow registers at the first one
 * of four frames in slow motion mode. In order to make DCAM_PREVIEW_SOF work,
 * software has to set auto copy at the last DCAM_CAP_SOF of four frames. So we
 * just use DCAM_CAP_SOF to do all the work.
 */
#define DCAMINT_IRQ_LINE_EN_SLM                    \
	(DCAMINT_ALL_ERROR | DCAMINT_ALL_TX_DONE | \
	BIT(DCAM_PREVIEW_SOF))

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
