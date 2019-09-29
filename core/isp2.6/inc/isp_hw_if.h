#ifndef _ISP_HW_IF_H_
#define _ISP_HW_IF_H_

#include "isp_core.h"
#include "isp_slice.h"
#include "isp_fmcu.h"

uint32_t isp_support_fmcu_cfg_slm(void);
uint32_t isp_ctx_support_fmcu(uint32_t ctx_id);
int isp_fmcu_available(uint32_t fmcu_id);
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

void isp_set_afbc_store_addr(uint32_t idx,
	enum isp_sub_path_id spath_id, unsigned long *yuv_addr);
int set_path_afbc_store(struct isp_path_desc *path);
void isp_set_afbc_store_fmcu_addr(struct isp_fmcu_ctx_desc *fmcu,
	struct isp_afbc_store_info *afbc_addr, int i);
int set_slice_spath_afbc_store(
		struct isp_fmcu_ctx_desc *fmcu,
		uint32_t path_en,
		uint32_t ctx_idx,
		enum isp_sub_path_id spath_id,
		struct slice_afbc_store_info *slc_afbc_store);
void isp_set_fbd_fetch_addr(int idx,
	struct compressed_addr compressed_addr,
	struct isp_fbd_raw_info *fbd_raw);
int set_slice_fbd_raw(struct isp_fmcu_ctx_desc *fmcu,
		struct slice_fbd_raw_info *fbd_raw_info);

#endif
