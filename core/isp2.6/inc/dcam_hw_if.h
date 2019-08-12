#ifndef _DCAM_HW_IF_H_
#define _DCAM_HW_IF_H_

#include "dcam_core.h"

void dcam_force_copy(struct dcam_pipe_dev *dev, uint32_t id);
void dcam_auto_copy(struct dcam_pipe_dev *dev, uint32_t id);
void dcam_init_default(struct dcam_pipe_dev *dev);
int dcam_reset(struct dcam_pipe_dev *dev);
void dcam_init_axim(struct sprd_cam_hw_info *hw);
int dcam_set_mipi_cap(struct dcam_pipe_dev *dev,
				struct dcam_mipi_info *cap_info);
int dcam_cfg_ebd(struct dcam_pipe_dev *dev, void *param);
int  dcam_cfg_path_full_source(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param);
int dcam_hwsim_extra(enum dcam_id idx);
int dcam_cfg_fbc(struct dcam_pipe_dev *dev, int fbc_mode);
int dcam_start_path(void *dcam_handle, struct dcam_path_desc *path);
int dcam_stop_path(void *dcam_handle, struct dcam_path_desc *path);
int dcam_set_fetch(void *dcam_handle, struct dcam_fetch_info *fetch);
void dcam_start_fetch(void);
int dcam_start(struct dcam_pipe_dev *dev);
int dcam_stop(struct dcam_pipe_dev *dev);
int dcam_update_path_size(struct dcam_pipe_dev *dev,
			struct dcam_path_desc *path);
struct dcam_cfg_entry *dcam_get_cfg_func(uint32_t index);
uint32_t dcam_get_path_ctrl_id(uint32_t path_id);

#endif
