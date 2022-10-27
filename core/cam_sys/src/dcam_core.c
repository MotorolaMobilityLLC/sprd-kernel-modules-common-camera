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

#include <linux/delay.h>
#include "cam_types.h"
#include "dcam_core.h"
#include "isp_hw.h"
#include "sprd_img.h"
#include <sprd_mm.h>

#include "dcam_reg.h"
#include "dcam_int.h"
#include "cam_queue.h"
#include "cam_types.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_CORE: %d %d %s : " fmt, current->pid, __LINE__, __func__

atomic_t s_dcam_opened[DCAM_HW_CONTEXT_MAX];
static DEFINE_MUTEX(s_dcam_dev_mutex);
static struct dcam_pipe_dev *s_dcam_dev;

static void dcamcore_get_fmcu(struct dcam_hw_context *pctx_hw)
{
	int ret = 0, hw_ctx_id = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_fmcu_ctx_desc *fmcu_info = NULL;

	if (!pctx_hw) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	hw = pctx_hw->hw;
	hw_ctx_id = pctx_hw->hw_ctx_id;

	if(hw->ip_dcam[hw_ctx_id]->fmcu_support) {
		fmcu_info = dcam_fmcu_ctx_desc_get(hw, hw_ctx_id);
		if (fmcu_info && fmcu_info->ops) {
			fmcu_info->hw = hw;
			ret = fmcu_info->ops->ctx_init(fmcu_info);
			if (ret) {
				pr_err("fail to init fmcu ctx\n");
				dcam_fmcu_ctx_desc_put(fmcu_info);
				pctx_hw->fmcu = NULL;
			} else
				pctx_hw->fmcu = fmcu_info;
		} else
			pr_debug("no more fmcu or ops\n");
	}
}

static void dcamcore_put_fmcu(struct dcam_hw_context *pctx_hw)
{
	struct dcam_fmcu_ctx_desc *fmcu_info = NULL;

	if (!pctx_hw) {
		pr_err("fail to get valid input ptr\n");
		return;
	}

	fmcu_info = (struct dcam_fmcu_ctx_desc *)pctx_hw->fmcu;
	if (fmcu_info) {
		fmcu_info->ops->ctx_deinit(fmcu_info);
		dcam_fmcu_ctx_desc_put(fmcu_info);
		pctx_hw->fmcu = NULL;
	}
}

static void dcamcore_offline_irq_proc(struct dcam_hw_context *dcam_hw_ctx,
		struct dcam_irq_info *irq_info)
{
	int i = 0, ret = 0;
	uint32_t irq_status = 0;
	struct dcam_irq_proc_desc irq_desc = {0};

	if (!dcam_hw_ctx || !irq_info) {
		pr_err("fail to get invalid hw_ctx %px %px\n", dcam_hw_ctx, irq_info);
		return;
	}

	pr_info("dcam offline node do irq proc\n");
	/* unbind alway be called in irq proc for offline proc
	 * no need to wait irq proc done */
	dcam_hw_ctx->in_irq_proc = 0;
	irq_status = irq_info->status;
	for (i = 0; i < irq_info->irq_num; i++) {
		if (irq_status & BIT(i)) {
			ret = dcam_int_irq_desc_get(i, &irq_desc);
			if (ret) {
				pr_warn("Warning: not to get valid irq desc %d\n", i);
				irq_status &= ~BIT(i);
				continue;
			}
			if (dcam_hw_ctx->dcam_irq_cb_func)
				dcam_hw_ctx->dcam_irq_cb_func(&irq_desc, dcam_hw_ctx->dcam_irq_cb_handle);
		}
		irq_status &= ~BIT(i);
		if (!irq_status)
			break;
	}
}

static int dcamcore_irq_proc(void *handle)
{
	int ret = 0;
	struct dcam_hw_context *dcam_hw_ctx = NULL;
	struct camera_interrupt *irq_status = NULL;
	struct dcam_irq_info irq_info = {0, 0};
	dcam_hw_ctx = (struct dcam_hw_context *)handle;
	if (!dcam_hw_ctx) {
		pr_warn("warning: dcam hw ctx is null\n");
		return ret;
	}

	dcam_hw_ctx->in_irq_proc = 1;
	irq_status = cam_queue_dequeue(&dcam_hw_ctx->dcam_irq_sts_q, struct camera_interrupt, list);
	if (unlikely(!irq_status)) {
		pr_warn("warning: null irq_status from queue: q may clear\n");
		dcam_hw_ctx->in_irq_proc = 0;
		return ret;
	}

	irq_info.irq_num = irq_status->irq_num;
	irq_info.status = irq_status->int_status;
	irq_info.status1 = irq_status->int_status1;
	cam_queue_empty_interrupt_put(irq_status);

	pr_debug("DCAM%d status: %x, status1: %x\n", dcam_hw_ctx->hw_ctx_id, irq_info.status, irq_info.status1);
	if (dcam_hw_ctx->is_offline_proc)
		dcamcore_offline_irq_proc(dcam_hw_ctx, &irq_info);
	else
		dcamint_dcam_status_rw(irq_info, dcam_hw_ctx);

	dcam_hw_ctx->in_irq_proc = 0;
	return ret;
}

static int dcamcore_context_init(struct dcam_pipe_dev *dev)
{
	int i = 0, ret = 0;
	struct dcam_hw_context *pctx_hw = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	pr_debug("dcam hw contexts init start!\n");
	memset(&dev->hw_ctx[0], 0, sizeof(dev->hw_ctx));
	for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
		pctx_hw = &dev->hw_ctx[i];
		pctx_hw->hw_ctx_id = i;
		pctx_hw->node_id = -1;
		pctx_hw->node = NULL;
		pctx_hw->hw = dev->hw;
		atomic_set(&pctx_hw->user_cnt, 0);
		atomic_set(&pctx_hw->shadow_done_cnt, 0);
		atomic_set(&pctx_hw->shadow_config_cnt, 0);
		spin_lock_init(&pctx_hw->glb_reg_lock);
		spin_lock_init(&pctx_hw->fbc_lock);
		spin_lock_init(&pctx_hw->ghist_read_lock);

		cam_queue_init(&pctx_hw->dcam_irq_sts_q, DCAM_INT_PROC_FRM_NUM, cam_queue_empty_interrupt_put);
		thrd = &pctx_hw->dcam_irq_proc_thrd;
		sprintf(thrd->thread_name, "dcam%d_irq_proc", pctx_hw->hw_ctx_id);
		ret = camthread_create(pctx_hw, thrd, dcamcore_irq_proc);
		if (unlikely(ret != 0)) {
			pr_err("fail to create dcam online interruption_proc thread\n");
			return -EINVAL;
		}

		pr_debug("register irq for dcam %d. irq_no %d\n", i, dev->hw->ip_dcam[i]->irq_no);
		ret = dcam_int_irq_request(&dev->hw->pdev->dev, dev->hw->ip_dcam[i]->irq_no, pctx_hw);
		if (ret)
			pr_err("fail to register irq for hw_ctx %d\n", i);

		dcamcore_get_fmcu(pctx_hw);
	}

	pr_debug("done!\n");
	return ret;
}

static int dcamcore_context_deinit(struct dcam_pipe_dev *dev)
{
	int ret = 0;
	int i = 0;
	struct dcam_hw_context *pctx_hw = NULL;

	pr_debug("enter.\n");
	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
		pctx_hw = &dev->hw_ctx[i];
		dcamcore_put_fmcu(pctx_hw);
		dcam_int_irq_free(&dev->hw->pdev->dev, pctx_hw);
		atomic_set(&pctx_hw->user_cnt, 0);
		camthread_stop(&pctx_hw->dcam_irq_proc_thrd);
		cam_queue_clear(&pctx_hw->dcam_irq_sts_q, struct camera_interrupt, list);
	}
	pr_debug("dcam contexts deinit done!\n");
	return ret;
}

static int dcamcore_dev_open(void *dcam_handle)
{
	int ret = 0;
	int hw_ctx_id = 0;
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)dcam_handle;

	pr_info("enter.\n");
	if (!dcam_handle) {
		pr_err("fail to get valid input ptr, isp_handle %p\n", dcam_handle);
		return -EFAULT;
	}

	mutex_lock(&dev->ctx_mutex);
	if (atomic_inc_return(&dev->enable) == 1) {
		ret = dcam_drv_hw_init(dev);
		if (ret) {
			pr_err("fail to init hw. ret: %d\n", ret);
			goto context_init_fail;
		}
		dev->hw->dcam_ioctl(dev->hw, DCAM_HW_CFG_INIT_AXI, &hw_ctx_id);

		ret = dcamcore_context_init(dev);
		if (ret) {
			pr_err("fail to init dcam context.\n");
			ret = -EFAULT;
			goto exit;
		}

		mutex_init(&dev->path_mutex);
		spin_lock_init(&dev->ctx_lock);
	}
	mutex_unlock(&dev->ctx_mutex);
	pr_info("open dcam pipe dev done! enable %d\n",atomic_read(&dev->enable));
	return 0;

context_init_fail:
	dcamcore_context_deinit(dev);
exit:
	atomic_dec(&dev->enable);
	mutex_unlock(&dev->ctx_mutex);
	pr_err("fail to open dcam dev!\n");
	return ret;
}

static int dcamcore_dev_close(void *dcam_handle)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)dcam_handle;

	if (!dcam_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}
	mutex_lock(&dev->ctx_mutex);
	if (atomic_dec_return(&dev->enable) == 0) {
		mutex_destroy(&dev->path_mutex);
		ret = dcamcore_context_deinit(dev);
		ret = dcam_drv_hw_deinit(dev);
	}
	mutex_unlock(&dev->ctx_mutex);
	pr_info("dcam dev disable done enable %d\n",atomic_read(&dev->enable));
	return ret;

}

static int dcamcore_ctx_bind(void *dev_handle, void *node, uint32_t node_id,
		uint32_t dcam_idx, uint32_t slw_cnt, uint32_t *hw_id, uint32_t *slw_type)
{
	int i = 0;
	uint32_t hw_ctx_id = DCAM_HW_CONTEXT_MAX, mode = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_context *pctx_hw = NULL;

	dev = (struct dcam_pipe_dev *)dev_handle;
	mode = dev->hw->csi_connect_type;
	if (mode == DCAM_BIND_FIXED) {
		pctx_hw = &dev->hw_ctx[dcam_idx];
		if (node_id == pctx_hw->node_id && pctx_hw->node == node) {
			atomic_inc(&pctx_hw->user_cnt);
			pr_info("sw %d & hw %d are already binded, cnt=%d\n",
				node_id, dcam_idx, atomic_read(&pctx_hw->user_cnt));
			return 0;
		}
		if (atomic_inc_return(&pctx_hw->user_cnt) == 1) {
			hw_ctx_id = pctx_hw->hw_ctx_id;
			goto exit;
		}
		atomic_dec(&pctx_hw->user_cnt);
	} else if (mode == DCAM_BIND_DYNAMIC) {
		for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
			pctx_hw = &dev->hw_ctx[i];
			if (node_id == pctx_hw->node_id && pctx_hw->node == node) {
				atomic_inc(&pctx_hw->user_cnt);
				pr_info("sw %d & hw %d are already binded, cnt=%d\n",
							node_id, i, atomic_read(&pctx_hw->user_cnt));
				return 0;
			}
		}
		for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
			pctx_hw = &dev->hw_ctx[i];
			if (atomic_inc_return(&pctx_hw->user_cnt) == 1) {
				hw_ctx_id = pctx_hw->hw_ctx_id;
				if (slw_cnt && dev->hw->ip_dcam[i]->fmcu_support) {
					*slw_type = DCAM_SLW_FMCU;
					if (!pctx_hw->fmcu)
						continue;
				}
				goto exit;
			}
			atomic_dec(&pctx_hw->user_cnt);
		}
	}
exit:
	if (hw_ctx_id == DCAM_HW_CONTEXT_MAX) {
		pr_err("fail to get hw_ctx_id. mode=%d\n", mode);
		return -1;
	}
	cam_queue_init(&pctx_hw->dcam_irq_sts_q, DCAM_INT_PROC_FRM_NUM, cam_queue_empty_interrupt_put);
	pctx_hw->node = node;
	pctx_hw->node_id = node_id;
	*hw_id = hw_ctx_id;
	pr_info("sw_ctx_id=%d, hw_ctx_id=%d, mode=%d, cnt=%d\n", pctx_hw->node_id, hw_ctx_id, mode, atomic_read(&pctx_hw->user_cnt));
	return 0;
}

static int dcamcore_ctx_unbind(void *dev_handle, void *node, uint32_t node_id)
{
	int i = 0, cnt = 0, loop = 0;
	int hw_ctx_id = DCAM_HW_CONTEXT_MAX;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_hw_context *pctx_hw = NULL;

	if (!dev_handle || !node) {
		pr_err("fail to get valid in ptr %px %px\n", dev_handle, node);
		return -EFAULT;
	}

	dev = (struct dcam_pipe_dev *)dev_handle;
	for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if ((node_id == pctx_hw->node_id) && (node == pctx_hw->node)) {
			hw_ctx_id = pctx_hw->hw_ctx_id;
			break;
		}
	}

	if (hw_ctx_id < 0) {
		pr_err("fail to binding sw ctx %d to any hw ctx\n", node_id);
		return -EINVAL;
	}

	for (i = 0; i < DCAM_HW_CONTEXT_MAX; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if ((node_id != pctx_hw->node_id) || (node != pctx_hw->node))
				continue;

		if (atomic_dec_return(&pctx_hw->user_cnt) == 0) {
			pr_info("sw_id=%d, hw_id=%d unbind success\n", node_id, pctx_hw->hw_ctx_id);
			cam_queue_clear(&pctx_hw->dcam_irq_sts_q, struct camera_interrupt, list);
			while (pctx_hw->in_irq_proc && loop < 1000) {
				pr_debug("ctx % in irq. wait %d", pctx_hw->hw_ctx_id, loop);
				loop++;
				udelay(500);
			};
			if (loop == 1000)
				pr_warn("warning: dcam node unind wait irq timeout\n");
			pctx_hw->node = NULL;
			pctx_hw->node_id = -1;
			goto exit;
		}

		cnt = atomic_read(&pctx_hw->user_cnt);
		if (cnt >= 1) {
			pr_info("sw id=%d, hw_id=%d, cnt=%d\n", node_id, pctx_hw->hw_ctx_id, cnt);
		} else {
			pr_info("should not be here: sw id=%d, hw id=%d, cnt=%d\n",
				node_id, pctx_hw->hw_ctx_id, cnt);
			return -EINVAL;
		}
	}

exit:
	return 0;
}

/*
 * Operations for this dcam_pipe_ops.
 */
static struct dcam_pipe_ops s_dcam_pipe_ops = {
	.open = dcamcore_dev_open,
	.close = dcamcore_dev_close,
	.bind = dcamcore_ctx_bind,
	.unbind = dcamcore_ctx_unbind,
};

void *dcam_core_pipe_dev_get(struct cam_hw_info *hw)
{
	struct dcam_pipe_dev *dev = NULL;

	mutex_lock(&s_dcam_dev_mutex);

	if (s_dcam_dev) {
		atomic_inc(&s_dcam_dev->user_cnt);
		dev = s_dcam_dev;
		pr_info("s_dcam_dev_new is already exist=%px, user_cnt=%d",
			s_dcam_dev, atomic_read(&s_dcam_dev->user_cnt));
		goto exit;
	}

	dev = vzalloc(sizeof(struct dcam_pipe_dev));
	if (!dev)
		goto exit;

	atomic_set(&dev->user_cnt, 1);
	atomic_set(&dev->enable, 0);
	mutex_init(&dev->ctx_mutex);

	dev->dcam_pipe_ops = &s_dcam_pipe_ops;
	dev->hw = hw;
	s_dcam_dev = dev;
	if (dev)
		pr_info("get dcam pipe dev: %px\n", dev);
exit:
	mutex_unlock(&s_dcam_dev_mutex);

	return dev;
}

int dcam_core_pipe_dev_put(void *dcam_handle)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = (struct dcam_pipe_dev *)dcam_handle;

	if (!dev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("put dcam pipe dev:%px, s_dcam_dev:%px,  users: %d, enable: %d\n",
		dev, s_dcam_dev, atomic_read(&dev->user_cnt), atomic_read(&dev->enable));

	mutex_lock(&s_dcam_dev_mutex);

	if (dev != s_dcam_dev) {
		mutex_unlock(&s_dcam_dev_mutex);
		pr_err("fail to match dev: %px, %px\n", dev, s_dcam_dev);
		return -EINVAL;
	}

	if (atomic_dec_return(&dev->user_cnt) == 0) {
		pr_info("free dcam pipe dev %px\n", dev);
		mutex_destroy(&dev->ctx_mutex);
		vfree(dev);
		dev = NULL;
		s_dcam_dev = NULL;
	}
	mutex_unlock(&s_dcam_dev_mutex);

	if (dev)
		pr_info("put dcam pipe dev: %px\n", dev);

	return ret;
}
