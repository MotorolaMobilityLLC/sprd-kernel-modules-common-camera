#ifndef _ISP_HW_IF_H_
#define _ISP_HW_IF_H_

#include "isp_core.h"

uint32_t isp_check_context(uint32_t ctx_id,
	struct isp_init_param *init_param);
uint32_t isp_support_fmcu_cfg_slm(void);
uint32_t isp_ctx_support_fmcu(uint32_t ctx_id);
void isp_set_ctx_common(struct isp_pipe_context *pctx);
uint32_t isp_cfg_map_get_count(void);
uint32_t *isp_cfg_map_get_data(void);
void set_common(struct sprd_cam_hw_info *hw);
void isp_set_ctx_default(struct isp_pipe_context *pctx);
struct isp_cfg_entry *isp_get_cfg_func(uint32_t index);
uint32_t isp_hist_bypass_get(int ctx_id);
int isp_irq_clear(struct sprd_cam_hw_info *hw, void *arg);
int isp_reset(struct sprd_cam_hw_info *hw, void *arg);
int isp_irq_enable(struct sprd_cam_hw_info *hw, void *arg);
int isp_irq_disable(struct sprd_cam_hw_info *hw, void *arg);

#endif
