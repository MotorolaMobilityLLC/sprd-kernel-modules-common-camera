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

#include <linux/slab.h>

#include "cam_types.h"
#include "cam_buf.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "cam_queue: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

int cam_queue_enqueue_front(struct camera_queue *q, struct list_head *list)
{
	int ret = 0;
	unsigned long flags = 0;

	if (q == NULL || list == NULL) {
		pr_err("fail to get valid param %p %p\n", q, list);
		return -EINVAL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("warning: q is clear\n");
		ret = -EPERM;
		goto unlock;
	}

	if (q->cnt >= q->max) {
		pr_warn_ratelimited("warning: q full, cnt %d, max %d\n", q->cnt, q->max);
		ret = -EPERM;
		goto unlock;
	}
	q->cnt++;
	list_add(list, &q->head);

unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return ret;
}

struct camera_frame *cam_queue_dequeue_if(struct camera_queue *q,
	bool (*filter)(struct camera_frame *, void *), void *data)
{
	int fatal_err = 0;
	unsigned long flags = 0;
	struct camera_frame *pframe = NULL;

	if (q == NULL || !filter) {
		pr_err("fail to get valid param %p %p\n", q, filter);
		return NULL;
	}

	spin_lock_irqsave(&q->lock, flags);
	if (q->state == CAM_Q_CLEAR) {
		pr_warn("warning: q is clear\n");
		goto unlock;
	}

	if (list_empty(&q->head) || q->cnt == 0) {
		pr_debug("queue empty %d, %d\n",
			list_empty(&q->head), q->cnt);
		fatal_err = (list_empty(&q->head) ^ (q->cnt == 0));
		if (fatal_err)
			pr_err("fail to get list node, empty %d, cnt %d\n",
				list_empty(&q->head), q->cnt);
		goto unlock;
	}

	pframe = list_first_entry(&q->head, struct camera_frame, list);

	if (!filter(pframe, data)) {
		pframe = NULL;
		goto unlock;
	}

	list_del(&pframe->list);
	q->cnt--;

unlock:
	spin_unlock_irqrestore(&q->lock, flags);

	return pframe;
}

/* Get the match two frame from two module
 * Through the master frame, travers the slave
 * queue, find a matching frame, if there is no,
 * use the slave next frame.
 */
int cam_queue_same_frame_get(struct camera_queue *q0,
	struct camera_frame **pf0, int64_t t_sec, int64_t t_usec)
{
	int ret = 0;
	unsigned long flags0 = 0;
	struct camera_frame *tmpf0 = NULL;
	int64_t t0 = 0, t1 = 0, mint = 0;
	int64_t t_diff_time = 0;

	spin_lock_irqsave(&q0->lock, flags0);
	if (list_empty(&q0->head)) {
		pr_warn("warning:Get list node fail\n");
		ret = -EFAULT;
		goto _EXT;
	}
	/* set mint a large value */
	mint = (((uint64_t)1 << 63) - 1);
	/* get least delta time two frame */
	t1 = t_sec * 1000000ll;
	t1 += t_usec;
	list_for_each_entry(tmpf0, &q0->head, list) {
		t0 = tmpf0->sensor_time.tv_sec * 1000000ll;
		t0 += tmpf0->sensor_time.tv_usec;
		t0 -= t1;
		if (t0 < 0)
			t0 = -t0;
		if (t0 < mint) {
			*pf0 = tmpf0;
			mint = t0;
		}
		if (mint < 400)
			break;
	}
	pr_info("mint:%lld\n", mint);
	t_diff_time = div64_u64((*pf0)->frame_interval_time, 1000 * 2);
	if (mint > t_diff_time) { /* delta > slave dynamic frame rate half fail */
		ret = -EFAULT;
		goto _EXT;
	}
	list_del(&((*pf0)->list));
	q0->cnt--;
	ret = 0;
_EXT:
	spin_unlock_irqrestore(&q0->lock, flags0);

	return ret;
}

/* in irq handler, may return NULL if alloc failed
 * else: will always retry alloc and return valid frame
 */
struct camera_frame *cam_queue_empty_frame_get(void)
{
	int ret = 0;
	uint32_t i = 0;
	struct camera_frame *pframe = NULL;

	pr_debug("Enter.\n");
	do {
		pframe = cam_queue_dequeue(g_empty_frm_q, struct camera_frame, list);
		if (pframe == NULL) {
			if (in_interrupt()) {
				/* fast alloc and return for irq handler */
				pframe = cam_buf_kernel_sys_kzalloc(sizeof(*pframe), GFP_ATOMIC);
				if (pframe)
					atomic_inc(&g_mem_dbg->empty_frm_cnt);
				else
					pr_err("fail to alloc memory\n");
				return pframe;
			}

			for (i = 0; i < CAM_EMP_Q_LEN_INC; i++) {
				pframe = cam_buf_kernel_sys_kzalloc(sizeof(*pframe), GFP_KERNEL);
				if (pframe == NULL) {
					pr_err("fail to alloc memory, retry\n");
					continue;
				}
				atomic_inc(&g_mem_dbg->empty_frm_cnt);
				pr_debug("alloc frame %p\n", pframe);
				ret = cam_queue_enqueue(g_empty_frm_q, &pframe->list);
				if (ret) {
					/* q full, return pframe directly here */
					break;
				}
				pframe = NULL;
			}
			pr_info("alloc %d empty frames, cnt %d\n",
				i, atomic_read(&g_mem_dbg->empty_frm_cnt));
		}
	} while (pframe == NULL);

	pr_debug("Done. get frame %p\n", pframe);
	return pframe;
}

int cam_queue_empty_frame_put(struct camera_frame *pframe)
{
	int ret = 0;
	if (pframe == NULL) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset(pframe, 0, sizeof(struct camera_frame));
	ret = cam_queue_enqueue(g_empty_frm_q, &pframe->list);
	if (ret) {
		pr_info("queue should be enlarged\n");
		atomic_dec(&g_mem_dbg->empty_frm_cnt);
		cam_buf_kernel_sys_kfree(pframe);
	}
	return 0;
}

void cam_queue_empty_frame_free(void *param)
{
	struct camera_frame *pframe = NULL;

	pframe = (struct camera_frame *)param;

	atomic_dec(&g_mem_dbg->empty_frm_cnt);
	pr_debug("free frame %p, cnt %d\n", pframe,
		atomic_read(&g_mem_dbg->empty_frm_cnt));
	cam_buf_kernel_sys_kfree(pframe);
	pframe = NULL;
}

struct camera_interrupt *cam_queue_empty_interrupt_get(void)
{
	int ret = 0;
	uint32_t i = 0;
	struct camera_interrupt *interruption = NULL;

	pr_debug("Enter.\n");
	do {
		interruption = cam_queue_dequeue(g_empty_interruption_q, struct camera_interrupt, list);
		if (interruption == NULL) {
			if (in_interrupt()) {
				/* fast alloc and return for irq handler */
				interruption = cam_buf_kernel_sys_kzalloc(sizeof(*interruption), GFP_ATOMIC);
				if (interruption)
					atomic_inc(&g_mem_dbg->empty_interruption_cnt);
				else
					pr_err("fail to alloc memory\n");
				return interruption;
			}

			for (i = 0; i < CAM_INT_EMP_Q_LEN_INC; i++) {
				interruption = cam_buf_kernel_sys_kzalloc(sizeof(*interruption), GFP_KERNEL);
				if (interruption == NULL) {
					pr_err("fail to alloc memory, retry\n");
					continue;
				}
				atomic_inc(&g_mem_dbg->empty_interruption_cnt);
				pr_debug("alloc frame %p\n", interruption);
				ret = cam_queue_enqueue(g_empty_interruption_q, &interruption->list);
				if (ret) {
					/* q full, return pframe directly here */
					break;
				}
				interruption = NULL;
			}
			pr_info("alloc %d empty frames, cnt %d\n",
				i, atomic_read(&g_mem_dbg->empty_interruption_cnt));
		}
	} while (interruption == NULL);

	pr_debug("Done. get frame %p\n", interruption);
	return interruption;
}

void cam_queue_empty_interrupt_put(void *param)
{
	int ret = 0;
	struct camera_interrupt *interruption = NULL;

	interruption = (struct camera_interrupt *)param;
	if (interruption == NULL) {
		pr_err("fail to get valid param\n");
		return ;
	}

	memset(interruption, 0, sizeof(struct camera_interrupt));
	ret = cam_queue_enqueue(g_empty_interruption_q, &interruption->list);
	if (ret) {
		pr_info("queue should be enlarged\n");
		atomic_dec(&g_mem_dbg->empty_interruption_cnt);
		cam_buf_kernel_sys_kfree(interruption);
	}
	return ;
}

void cam_queue_empty_interrupt_free(void *param)
{
	struct camera_interrupt *interruption = NULL;

	interruption = (struct camera_interrupt *)param;
	if (interruption == NULL) {
		pr_err("fail to get valid param\n");
		return ;
	}

	atomic_dec(&g_mem_dbg->empty_interruption_cnt);
	pr_debug("free frame %p, cnt %d\n", interruption,
		atomic_read(&g_mem_dbg->empty_interruption_cnt));
	cam_buf_kernel_sys_kfree(interruption);
	interruption = NULL;
}

int cam_queue_recycle_blk_param(struct camera_queue *q, struct camera_frame *param_pframe)
{
	int ret = 0;

	if (!q || !param_pframe) {
		pr_err("fail to get valid handle %p %p\n", q, param_pframe);
		return -1;
	}

	param_pframe->blkparam_info.update = 0;
	param_pframe->fid = 0xffff;
	param_pframe->blkparam_info.param_block = NULL;

	if (param_pframe->buf.addr_k) {
		memset((void *)param_pframe->buf.addr_k, 0, sizeof(struct dcam_isp_k_block));
		ret = cam_buf_kunmap(&param_pframe->buf);
		if(ret) {
			pr_err("fail to unmap param node %px\n", param_pframe);
			goto error;
		}
	}
	ret = cam_queue_enqueue(q, &param_pframe->list);
	if(ret) {
		pr_err("fail to recycle param node %px\n", param_pframe);
		goto error;
	}
	return ret;

error:
	if (param_pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&param_pframe->buf);
	cam_buf_free(&param_pframe->buf);
	cam_queue_empty_frame_put(param_pframe);
	return ret;
}

struct camera_frame * cam_queue_empty_blk_param_get(struct camera_queue *q)
{
	int ret = 0;
	struct camera_frame *param_frame = NULL;

	if (!q) {
		pr_err("fail to get valid handle\n");
		return NULL;
	}

	param_frame = cam_queue_dequeue(q, struct camera_frame, list);
	if (param_frame) {
		ret = cam_buf_kmap(&param_frame->buf);
		param_frame->blkparam_info.param_block = (void *)param_frame->buf.addr_k;
		if (ret) {
			pr_err("fail to kmap cfg buffer\n");
			param_frame->buf.addr_k = 0;
			param_frame->blkparam_info.param_block = NULL;
			return NULL;
		}
		pr_debug("pframe_param.buf =%lu,pframe_param->param_block=%p\n",param_frame->buf,param_frame->blkparam_info.param_block);
	}
	return param_frame;
}

int cam_queue_blk_param_unbind(struct camera_queue *param_share_queue, struct camera_frame *pframe)
{
	pframe->blkparam_info.update = 0;
	pframe->blkparam_info.param_block = NULL;
	if (pframe->blkparam_info.blk_param_node) {
		cam_queue_recycle_blk_param(param_share_queue, pframe->blkparam_info.blk_param_node);
		pframe->blkparam_info.blk_param_node = NULL;
	}
	return 0;
}
