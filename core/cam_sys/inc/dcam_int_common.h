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

#ifndef _DCAM_INT_COMMON_H_
#define _DCAM_INT_COMMON_H_

#include <linux/bitops.h>
#include <linux/device.h>

#include "cam_types.h"
#include "dcam_core.h"

inline void dcam_int_common_record(uint32_t idx, struct dcam_irq_info *irq_info);
inline int dcam_int_common_dummy_callback(struct dcam_hw_context *dcam_hw_ctx, struct dcam_irq_proc *irq_proc);
int dcam_int_common_irq_request(struct device *pdev, int irq, void *param);
void dcam_int_common_irq_free(struct device *pdev, void *param);
void dcam_int_common_tracker_reset(uint32_t idx);
void dcam_int_common_full_port_done(void *param);
void dcam_int_common_bin_port_done(void *param);
void dcam_int_common_vch2_port_done(void *param);
void dcam_int_common_gtm_port_done(void *param);
void dcam_int_common_irq_sensor_eof(void *param);
void dcam_int_common_lscm_port_done(void *param);
void dcam_int_common_aem_port_done(void *param);
void dcam_int_common_pdaf_port_done(void *param);
void dcam_int_common_afm_port_done(void *param);
void dcam_int_common_afl_port_done(void *param);
void dcam_int_common_hist_port_done(void *param);
void dcam_int_common_nr3_port_done(void *param);
void dcam_int_common_tracker_dump(uint32_t idx);
void dcam_int_common_irq_preview_sof(void *param);
void dcam_int_common_vch3_port_done(void *param);

#endif
