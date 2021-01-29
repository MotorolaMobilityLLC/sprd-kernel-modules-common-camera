/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
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
#include <sprd_mm.h>
#include <linux/mutex.h>
#include "isp_gtm.h"
#include "isp_hw.h"

 #ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_GTM: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

static struct isp_gtm_sync s_rgb_gtm_sync[GTM_ID_MAX];

static int ispgtm_cfg_param(void *handle,
		 enum isp_gtm_cfg_cmd cmd, void *param)
{
	 int ret = 0;
	 struct isp_gtm_ctx_desc *gtm_ctx = NULL;

	 if (!handle || !param) {
		 pr_err("fail to get valid input ptr\n");
		 return -EFAULT;
	 }

	 gtm_ctx = (struct isp_gtm_ctx_desc *)handle;

	 switch (cmd) {
	 case ISP_GTM_CFG_EB:
		 gtm_ctx->enable = *(uint32_t *)param;
		 pr_debug("GTM ctx_id %d, enable %d\n", gtm_ctx->ctx_id, gtm_ctx->enable);
		 break;
	 case ISP_GTM_CFG_MODE:
		 gtm_ctx->mode = *(uint32_t *)param;
		 pr_debug("GTM ctx_id %d, mode %d\n", gtm_ctx->ctx_id, gtm_ctx->mode);
		 break;
	case ISP_GTM_CFG_FRAME_ID:
		gtm_ctx->fid = *(uint32_t *)param;
		pr_debug("GTM ctx_id %d, frame id %d\n", gtm_ctx->ctx_id, gtm_ctx->fid);
		break;
	 default:
		 pr_debug("fail to get known cmd: %d\n", cmd);
		 ret = -EFAULT;
		 break;
	 }

	 return ret;
}

static int ispgtm_sync_completion_done(void *handle)
{
	int fid = 0;
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	struct isp_gtm_sync *sync = NULL;

	if (!handle) {
		pr_err("fail to get invalid ptr\n");
		return -EFAULT;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)handle;
	sync = gtm_ctx->sync;
	if (!sync) {
		pr_err("fail to get sync ptr\n");
		return -1;
	}

	mutex_lock(&sync->share_mutex);
	fid = atomic_read(&sync->wait_completion_fid);
	if (fid) {
		atomic_set(&sync->wait_completion_fid, 0);
		complete(&sync->share_comp);
	}
	mutex_unlock(&sync->share_mutex);

	return fid;
}

static int ispgtm_sync_completion_get(void *handle)
{
	int fid = 0;
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	struct isp_gtm_sync *sync = NULL;

	if (!handle) {
		pr_err("fail to get invalid ptr\n");
		return -EFAULT;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)handle;
	sync = gtm_ctx->sync;

	mutex_lock(&sync->share_mutex);
	fid = atomic_read(&sync->wait_completion_fid);
	mutex_unlock(&sync->share_mutex);

	return fid;
}

static int ispgtm_sync_completion_set(struct isp_gtm_sync *gtm_sync, int fid)
{
	if(!gtm_sync) {
		pr_err("fail to get invalid ptr\n");
		return -EFAULT;
	}

	atomic_set(&gtm_sync->wait_completion_fid, fid);

	return 0;
}

static void ispgtm_sync_fid_set(void *handle)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	struct isp_gtm_sync *sync = NULL;

	if (!handle) {
		pr_err("fail to get invalid ptr\n");
		return;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)handle;
	sync = gtm_ctx->sync;
	if (!sync) {
		pr_err("fail to get sync invalid ptr\n");
		return;
	}

	pr_debug("ctx_id %d, fid %d\n", gtm_ctx->ctx_id, gtm_ctx->fid);
	mutex_lock(&sync->share_mutex);
	atomic_set(&sync->prev_fid, gtm_ctx->fid);
	mutex_unlock(&sync->share_mutex);
}

static int ispgtm_get_preview_hist_cal(void *handle)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	struct isp_gtm_mapping *mapping = NULL;
	struct isp_hw_gtm_func gtm_func;

	if (!handle) {
		pr_err("fail to get invaild ptr\n");
		return -EFAULT;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)handle;
	mapping = &gtm_ctx->sync->mapping;

	gtm_func.index = ISP_K_GTM_MAPPING_GET;
	gtm_ctx->hw->isp_ioctl(gtm_ctx->hw, ISP_HW_CFG_GTM_FUNC_GET, &gtm_func);
	gtm_func.k_blk_func(mapping);

	mapping->fid = gtm_ctx->fid;
	pr_debug("ctx_id %d, mapping fid %d\n", gtm_ctx->ctx_id, gtm_ctx->fid);

	return 0;
}

static int ispgtm_capture_tunning_set(int cam_id,
	struct isp_dev_gtm_block_info *tunning)
{
	int i = 0;

	if (cam_id >=GTM_ID_MAX || !tunning) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	while(i < GTM_ID_MAX) {
		if (cam_id == i)
			break;
		i++;
	}

	if (i == GTM_ID_MAX) {
		pr_err("fail to get cam id\n");
		return -1;
	}

	memcpy(&s_rgb_gtm_sync[i].tuning, tunning, sizeof(struct isp_dev_gtm_block_info));
	pr_debug("cam_id %d, mod_en %d\n", cam_id, s_rgb_gtm_sync[i].tuning.gtm_mod_en);
	return 0;
}

static int ispgtm_capture_tunning_get(int cam_id,
	struct isp_dev_gtm_block_info *tunning)
{
	int i = 0;

	if (cam_id >= GTM_ID_MAX || !tunning) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	while(i < GTM_ID_MAX) {
		if (cam_id == i)
			break;
		i++;
	}

	if (i == GTM_ID_MAX) {
		pr_err("fail to get cam id\n");
		return -1;
	}

	memcpy(tunning, &s_rgb_gtm_sync[i].tuning, sizeof(struct isp_dev_gtm_block_info));
	pr_debug("cam_id %d, mod_en %d\n", cam_id, s_rgb_gtm_sync[i].tuning.gtm_mod_en);
	return 0;
}

static int ispgtm_pipe_proc(void *handle, void *param)
{
	int ret = 0;
	uint32_t idx = 0;
	int prev_fid = 0;
	int timeout = 0;
 	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	struct isp_gtm_sync *gtm_sync = NULL;
	struct isp_gtm_mapping *mapping = NULL;
	struct isp_gtm_k_block gtm_k_block ={0};
	struct isp_hw_gtm_func gtm_func;

	if (!handle || !param) {
		 pr_err("fail to get valid input ptr NULL\n");
		 return -EFAULT;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)handle;

	switch (gtm_ctx->mode) {
	case MODE_GTM_PRE:
		gtm_k_block.ctx = gtm_ctx;
		gtm_k_block.tuning = param;
		gtm_func.index = ISP_K_GTM_BLOCK_SET;
		gtm_ctx->hw->isp_ioctl(gtm_ctx->hw, ISP_HW_CFG_GTM_FUNC_GET, &gtm_func);
		gtm_func.k_blk_func(&gtm_k_block);
		if (gtm_k_block.tuning->gtm_mod_en)
			ret = ispgtm_capture_tunning_set(gtm_ctx->cam_id, param);
		break;
	 case MODE_GTM_CAP:
		gtm_sync = gtm_ctx->sync;
		if (!gtm_sync) {
			pr_err("fail to ctx_id %d get valid gtm_sync ptr NULL\n", gtm_ctx->ctx_id);
			return -1;
		}

		ret = ispgtm_capture_tunning_get(gtm_ctx->cam_id, param);
		if (ret) {
			pr_err("fail to ctx_id %d get tuning param\n", gtm_ctx->ctx_id);
			return -1;
		}

		gtm_k_block.ctx = gtm_ctx;
		gtm_k_block.tuning = param;
		/*capture: mapping enable, hist_stat disable*/
		gtm_k_block.tuning->gtm_map_bypass = 0;
		gtm_k_block.tuning->gtm_hist_stat_bypass = 1;

		if (gtm_k_block.tuning->gtm_mod_en == 0) {
			pr_debug("capture frame ctx_id %d, mod_en off\n", gtm_ctx->ctx_id);
			goto exit;
		}

	 	prev_fid = atomic_read(&gtm_sync->prev_fid);
		pr_debug("GTM ctx_id %d, capture fid [%d], preview fid [%d]\n", gtm_ctx->ctx_id, gtm_ctx->fid, prev_fid);
		if (gtm_ctx->fid > prev_fid) {
			ispgtm_sync_completion_set(gtm_sync, gtm_ctx->fid);
			timeout = wait_for_completion_interruptible_timeout(&gtm_sync->share_comp,
						ISP_GM_TIMEOUT);
			if (timeout <= 0) {
				pr_err("fail to wait gtm completion [%ld]\n", timeout);
				gtm_ctx->mode = MODE_GTM_OFF;
				gtm_ctx->bypass = 1;
				ret = -1;
				goto exit;
			}
		} else if (gtm_ctx->fid == prev_fid) {
			;
		} else {
			pr_debug("capture frame fid %d gtm discard\n", gtm_ctx->fid);
			goto exit;
		}

		mapping = &gtm_ctx->sync->mapping;
		if (gtm_ctx->fid == mapping->fid) {
			gtm_func.index = ISP_K_GTM_BLOCK_SET;
			gtm_ctx->hw->isp_ioctl(gtm_ctx->hw, ISP_HW_CFG_GTM_FUNC_GET, &gtm_func);
			gtm_func.k_blk_func(&gtm_k_block);

			mapping->ctx_id = gtm_ctx->ctx_id;
			gtm_func.index = ISP_K_GTM_MAPPING_SET;
			gtm_ctx->hw->isp_ioctl(gtm_ctx->hw, ISP_HW_CFG_GTM_FUNC_GET, &gtm_func);
			gtm_func.k_blk_func(mapping);

			idx = gtm_ctx->ctx_id;
			gtm_func.index = ISP_K_GTM_STATUS_GET;
			gtm_ctx->hw->isp_ioctl(gtm_ctx->hw, ISP_HW_CFG_GTM_FUNC_GET, &gtm_func);
			gtm_ctx->gtm_mode_en = gtm_func.k_blk_func(&idx) & gtm_k_block.tuning->gtm_mod_en;
		}
		break;
	case MODE_GTM_OFF:
		pr_debug("ctx_id %d, GTM off\n", gtm_ctx->ctx_id);
		break;
	default:
		pr_debug("waring , ctx_id %d, GTM need to check, mode %d\n", gtm_ctx->ctx_id, gtm_ctx->mode);
		break;
	 }
exit:
	return ret;
}

static struct isp_gtm_ctx_desc *ispgtm_ctx_init(uint32_t idx, uint32_t cam_id, void *hw)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;

	gtm_ctx = vzalloc(sizeof(struct isp_gtm_ctx_desc));
	if (!gtm_ctx) {
		pr_err("fail to alloc isp %d gtm ctx\n", idx);
		return NULL;
	}

	gtm_ctx->ctx_id = idx;
	gtm_ctx->cam_id = cam_id;
	gtm_ctx->hw = hw;
	atomic_set(&gtm_ctx->cnt, 0);
	gtm_ctx->gtm_ops.cfg_param = ispgtm_cfg_param;
	gtm_ctx->gtm_ops.pipe_proc = ispgtm_pipe_proc;
	gtm_ctx->gtm_ops.set_frmidx = ispgtm_sync_fid_set;
	gtm_ctx->gtm_ops.sync_completion_get = ispgtm_sync_completion_get;
	gtm_ctx->gtm_ops.sync_completion_done = ispgtm_sync_completion_done;
	gtm_ctx->gtm_ops.get_preview_hist_cal = ispgtm_get_preview_hist_cal;
	gtm_ctx->sync = &s_rgb_gtm_sync[cam_id];

	return gtm_ctx;
}

void isp_gtm_rgb_ctx_put(void *gtm_handle)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;

	if (!gtm_handle) {
		pr_err("fail to get valid gtm_handle\n");
		return;
	}

	gtm_ctx = (struct isp_gtm_ctx_desc *)gtm_handle;

	if (gtm_ctx)
		vfree(gtm_ctx);

	gtm_ctx = NULL;
}

void *ispgtm_rgb_ctx_get(uint32_t idx, enum camera_id cam_id, void *hw)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;

	gtm_ctx = ispgtm_ctx_init(idx, cam_id, hw);
	if (!gtm_ctx) {
		pr_err("fail to get invalid ltm_ctx\n");
		return NULL;
	}

	return gtm_ctx;
}

void isp_gtm_sync_init(void)
{
	int i = 0;
	struct isp_gtm_sync *sync = NULL;

	for (; i < GTM_ID_MAX ; i++) {
		sync = &s_rgb_gtm_sync[i];
		atomic_set(&sync->prev_fid, 0);
		atomic_set(&sync->user_cnt, 0);
		atomic_set(&sync->wait_completion_fid, 0);
		init_completion(&sync->share_comp);
		mutex_init(&sync->share_mutex);
	}
}

void isp_gtm_sync_deinit(void)
{
	isp_gtm_sync_init();
}


