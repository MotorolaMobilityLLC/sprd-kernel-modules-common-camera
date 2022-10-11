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

#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include "cam_node.h"
#include "cam_hw.h"
#include "isp_cfg.h"
#include "isp_dev.h"
#include "isp_drv.h"
#include "isp_fmcu.h"
#include "isp_hwctx.h"
#include "isp_scaler_node.h"
#include "isp_scaler_port.h"
#include "isp_slice.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_YUV_SCALER_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

void isp_scaler_node_offline_pararm_free(void *param)
{
	struct isp_offline_param *cur, *prev;

	cur = (struct isp_offline_param *)param;
	while (cur) {
		prev = (struct isp_offline_param *)cur->prev;
		pr_info("free %p\n", cur);
		kfree(cur);
		cur = prev;
	}
}

static uint32_t isp_yuv_scaler_node_insert_port(struct isp_yuv_scaler_node* node, void *param)
{
	struct isp_scaler_port *q_port = NULL;
	struct isp_scaler_port *new_port = NULL;
	uint32_t is_exist = 0;

	new_port = VOID_PTR_TO(param, struct isp_scaler_port);
	pr_debug("node type:%d,id:%d,port type:%d new_port %px\n", node->node_type, node->node_id, new_port->port_id, new_port);

	list_for_each_entry(q_port, &node->port_queue.head, list) {
		if (new_port == q_port) {
			is_exist = 1;
			break;
		}
	}
	if (!is_exist)
		cam_queue_enqueue(&node->port_queue, &new_port->list);
	return 0;
}

uint32_t isp_yuv_scaler_cfg_param(void *node, uint32_t port_id, uint32_t cmd, void *param)
{
	int ret = 0;
	struct isp_yuv_scaler_node *inode = NULL;
	struct isp_scaler_path_uinfo *path_info = NULL;
	struct isp_scaler_uinfo *uinfo = NULL;
	int hw_path_id = isp_scaler_port_id_switch(port_id);

	inode = VOID_PTR_TO(node, struct isp_yuv_scaler_node);
	path_info = &inode->uinfo.path_info[hw_path_id];
	uinfo = &inode->uinfo;

	switch (cmd) {
	case ISP_YUV_SCALER_NODE_CFG_PORT_UFRAME:
		uinfo->uframe_sync |= *(uint32_t *)param;
		path_info->uframe_sync = *(uint32_t *)param;
		break;
	case ISP_YUV_SCALER_NODE_INSERT_PORT:
		isp_yuv_scaler_node_insert_port(inode, param);
		break;
	case ISP_YUV_SCALER_NODE_CFG_RESERVE_BUF:
		break;
	}
	return ret;
}

int isp_scaler_node_request_proc(struct isp_yuv_scaler_node *node, void *param)
{
	struct camera_frame *pframe = NULL;
	struct camera_frame *pframe1 = NULL;
	struct isp_scaler_port *port = NULL;
	int ret = 0;

	if (!node || !param) {
		pr_err("fail to get valid param\n");
		return -1;
	}

	pr_debug("node %px node->node_id %d cfg_id %d\n",node,node->node_id,node->cfg_id);
	pframe = (struct camera_frame *)param;
	pframe->priv_data = node;
	/*buffer is transing from isp_port_out_of_queue to isp_scaler_port out_of_queue*/
	pframe1 =(struct camera_frame *)pframe->pframe_data;
	pframe1->is_reserved = 0;
	pframe1->priv_data = port;

	list_for_each_entry(port, &node->port_queue.head, list) {
		ret = cam_queue_enqueue(&port->out_buf_queue, &pframe1->list);
		if (ret) {
			pr_err("fail to enqueue output buffer, path %d.\n",port->port_id);
			return ret;
		}
	}

	cam_buf_iommu_unmap(&pframe->buf);
	cam_queue_enqueue(&node->in_queue, &pframe->list);
	if (ret) {
		pr_err("fail to enqueue node in_queue %d.\n");
		return ret;
	}

	if (atomic_read(&node->user_cnt) > 0)
		complete(&node->thread.thread_com);
	return 0;
}

void *isp_yuv_scaler_node_hwctx_to_node(enum isp_context_hw_id hw_ctx_id, void *dev_handle)
{
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_yuv_scaler_node *node = NULL;
	struct isp_pipe_dev *dev = NULL;
	dev = (struct isp_pipe_dev *)dev_handle;
	if (hw_ctx_id < ISP_CONTEXT_HW_NUM) {
		pctx_hw = &dev->hw_ctx[hw_ctx_id];
		if (pctx_hw->node) {
			node = pctx_hw->node;
			return node;
		}
	}
	return NULL;
}

void *isp_scaler_node_context_bind(void *node, int fmcu_need, isp_irq_postproc postproc_func)
{
	int i = 0, m = 0, loop;
	int hw_ctx_id = -1;
	unsigned long flag = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_yuv_scaler_node *inode = NULL;

	if (!node) {
		pr_err("fail to get node\n");
		return NULL;
	}
	inode = (struct isp_yuv_scaler_node*)node;
	dev = inode->dev;
	spin_lock_irqsave(&dev->ctx_lock, flag);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if (pctx_hw->node == inode) {
			atomic_inc(&pctx_hw->user_cnt);
			pr_debug("node %d & hw %d already binding, cnt=%d\n",
				inode->node_id, i, atomic_read(&pctx_hw->user_cnt));
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return pctx_hw;
		}
	}

	loop = (fmcu_need & FMCU_IS_MUST) ? 1 : 2;
	for (m = 0; m < loop; m++) {
		for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
			pctx_hw = &dev->hw_ctx[i];
			if (m == 0) {
				/* first pass we just select fmcu/no-fmcu context */
				if ((!fmcu_need && pctx_hw->fmcu_handle) ||
					(fmcu_need && !pctx_hw->fmcu_handle))
					continue;
			}

			if (atomic_inc_return(&pctx_hw->user_cnt) == 1) {
				hw_ctx_id = pctx_hw->hw_ctx_id;
				goto exit;
			}
			atomic_dec(&pctx_hw->user_cnt);
		}
	}

	/* force fmcu used, we will retry */
	if (fmcu_need & FMCU_IS_MUST)
		goto exit;

exit:
	spin_unlock_irqrestore(&dev->ctx_lock, flag);

	if (hw_ctx_id == -1)
		return NULL;

	pctx_hw->node = inode;
	pctx_hw->node_id = inode->node_id;
	pctx_hw->cfg_id = inode->cfg_id;
	pctx_hw->fmcu_used = 0;
	pctx_hw->scaler_debug = 1;
	inode->is_bind = 1;
	pctx_hw->postproc_func = postproc_func;
	pr_debug("sw %d, hw %d %d, fmcu_need %d fmcu %px\n",
		inode->node_id, hw_ctx_id, pctx_hw->hw_ctx_id,
		fmcu_need, (unsigned long)pctx_hw->fmcu_handle);
	return pctx_hw;
}

uint32_t isp_scaler_node_context_unbind(void *node)
{
	int i, cnt;
	struct isp_pipe_dev *dev = NULL;
	struct isp_hw_context *pctx_hw = NULL;
	struct isp_yuv_scaler_node *inode = NULL;
	unsigned long flag = 0;
	inode = VOID_PTR_TO(node, struct isp_yuv_scaler_node);
	dev = inode->dev;

	spin_lock_irqsave(&dev->ctx_lock, flag);
	if (!inode->is_bind) {
		pr_debug("binding node_id %d to any hw ctx, inode->is_bind %d\n", inode->node_id,inode->is_bind);
		spin_unlock_irqrestore(&dev->ctx_lock, flag);
		return 0;
	}

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if ((inode->node_id != pctx_hw->node_id) ||
			(inode != pctx_hw->node))
			continue;

		if (atomic_dec_return(&pctx_hw->user_cnt) == 0) {
			pr_debug("sw_id=%d, hw_id=%d unbind success\n",
				inode->node_id, pctx_hw->hw_ctx_id);
			pctx_hw->node = NULL;
			pctx_hw->postproc_func = NULL;
			inode->is_bind = 0;
			pctx_hw->scaler_debug = 0;
			goto exit;
		}

		cnt = atomic_read(&pctx_hw->user_cnt);
		if (cnt >= 1) {
			pr_debug("sw id=%d, hw_id=%d, cnt=%d\n",
				inode->node_id, pctx_hw->hw_ctx_id, cnt);
		} else {
			pr_debug("should not be here: sw id=%d, hw id=%d, cnt=%d\n",
				inode->node_id, pctx_hw->hw_ctx_id, cnt);
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return -EINVAL;
		}
	}

exit:
	pctx_hw->fmcu_used = 0;
	spin_unlock_irqrestore(&dev->ctx_lock, flag);
	return 0;
}

int isp_yuv_scaler_slice_base_cfg(void *cfg_in, void *slice_ctx,
		uint32_t *valid_slc_num)
{
	int i = 0;
	int ret = 0;
	struct slice_cfg_input *in_ptr = (struct slice_cfg_input *)cfg_in;
	struct isp_slice_context *slc_ctx = (struct isp_slice_context *)slice_ctx;

	if (!in_ptr || !slc_ctx || !valid_slc_num) {
		pr_err("fail to get input ptr, null.\n");
		return -EFAULT;
	}

	memset(slc_ctx, 0, sizeof(struct isp_slice_context));
	ispslice_slice_base_info_cfg(in_ptr, slc_ctx);
	ispslice_slice_scaler_info_cfg(in_ptr, slc_ctx);

	*valid_slc_num = 0;
	for (i = 0; i < SLICE_NUM_MAX; i++) {
		if (slc_ctx->slices[i].valid)
			*valid_slc_num = (*valid_slc_num) + 1;
	}

	return ret;
}
static int isp_yuv_scaler_node_slice_needed(struct isp_yuv_scaler_node *inode)
{
	struct isp_scaler_port *port = NULL;
	int hw_path_id = 0;
	struct isp_scaler_path_uinfo *path_info = NULL;

	if (!inode) {
		pr_err("fail to get input ptr, null.\n");
		return -EFAULT;
	}

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			path_info = &inode->uinfo.path_info[hw_path_id];
			if (path_info->dst.w > g_camctrl.isp_linebuf_len)
				return 1;
		}
	}

	return 0;
}

static int isp_yuv_scaler_slice_ctx_init(struct isp_yuv_scaler_node *inode, struct isp_pipe_info *pipe_info, uint32_t *multi_slice)
{
	int ret = 0, hw_path_id = 0;
	struct slice_cfg_input slc_cfg_in = {0};
	struct cam_hw_info *hw_info = NULL;
	struct isp_scaler_port *port = NULL;

	*multi_slice = 0;

	if ((isp_yuv_scaler_node_slice_needed(inode) == 0)) {
		pr_debug("sw %d don't need to slice \n",inode->cfg_id);
		return 0;
	}

	hw_info = (struct cam_hw_info *)inode->dev->isp_hw;
	if (!hw_info) {
		pr_err("fail to get hw info NULL\n");
		goto exit;
	}

	if (inode->slice_ctx == NULL) {
		inode->slice_ctx = isp_slice_ctx_get();
		if (IS_ERR_OR_NULL(inode->slice_ctx)) {
			pr_err("fail to get memory for slice_ctx.\n");
			inode->slice_ctx = NULL;
			ret = -ENOMEM;
			goto exit;
		}
	}

	memset(&slc_cfg_in, 0, sizeof(struct slice_cfg_input));

	slc_cfg_in.frame_in_size.w = inode->pipe_src.crop.size_x;
	slc_cfg_in.frame_in_size.h = inode->pipe_src.crop.size_y;
	slc_cfg_in.frame_fetch = &pipe_info->fetch;
	pipe_info->fetch_fbd_yuv.fetch_fbd_bypass = 1;
	slc_cfg_in.frame_fbd_yuv = &pipe_info->fetch_fbd_yuv;
	slc_cfg_in.thumb_scaler = &pipe_info->thumb_scaler;

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			slc_cfg_in.frame_out_size[hw_path_id] = &pipe_info->store[hw_path_id].store.size;
			slc_cfg_in.frame_store[hw_path_id] = &pipe_info->store[hw_path_id].store;
			slc_cfg_in.frame_scaler[hw_path_id] = &pipe_info->scaler[hw_path_id].scaler;
			slc_cfg_in.frame_deci[hw_path_id] = &pipe_info->scaler[hw_path_id].deci;
			slc_cfg_in.frame_trim0[hw_path_id] = &pipe_info->scaler[hw_path_id].in_trim;
			slc_cfg_in.frame_trim1[hw_path_id] = &pipe_info->scaler[hw_path_id].out_trim;
		}
	}
	slc_cfg_in.calc_dyn_ov.verison = hw_info->ip_isp->dyn_overlap_version;
	isp_yuv_scaler_slice_base_cfg(&slc_cfg_in, inode->slice_ctx, &inode->valid_slc_num);

	pr_debug("sw %d valid_slc_num %d\n", inode->cfg_id, inode->valid_slc_num);
	if (inode->valid_slc_num > 1)
		*multi_slice = 1;
exit:
	return ret;
}

static int isp_yuv_scaler_slice_info_cfg(void *cfg_in, struct isp_slice_context *slc_ctx)
{
	struct slice_cfg_input *in_ptr = NULL;

	if (!cfg_in || !slc_ctx) {
		pr_err("fail to get input ptr, null.\n");
		return -EFAULT;
	}

	in_ptr = (struct slice_cfg_input *)cfg_in;
	ispslice_fetch_info_cfg(cfg_in, slc_ctx);
	ispslice_store_info_cfg(cfg_in, slc_ctx);

	return 0;
}

int isp_yuv_scaler_slice_fmcu_cmds_set(void *fmcu_handle, void *node, struct isp_hw_context *pctx_hw)
{
	int i, j;
	int cfg_id = 0;
	int hw_ctx_id = 0;
	struct isp_yuv_scaler_node *inode = NULL;
	enum isp_work_mode wmode;
	struct isp_slice_desc *cur_slc;
	struct slice_store_info *slc_store;
	struct slice_scaler_info *slc_scaler;
	struct isp_slice_context *slc_ctx;
	struct isp_fmcu_ctx_desc *fmcu;
	struct cam_hw_info *hw = NULL;
	struct isp_hw_fmcu_cfg fmcu_cfg;
	struct isp_hw_slices_fmcu_cmds parg;
	struct isp_hw_slice_spath spath_sotre;
	struct isp_hw_slice_spath spath_scaler;
	struct isp_hw_slice_spath_thumbscaler thumbscaler;
	struct isp_hw_set_slice_fetch fetcharg;

	if (!fmcu_handle || !node || !pctx_hw) {
		pr_err("fail to get valid input ptr, fmcu_handle %p, node %p\n",
			fmcu_handle, node, pctx_hw);
		return -EFAULT;
	}
	inode = (struct isp_yuv_scaler_node *)node;
	hw = inode->dev->isp_hw;
	cfg_id = inode->cfg_id;
	hw_ctx_id = pctx_hw->hw_ctx_id;
	pr_debug("get hw context id=%d\n", hw_ctx_id);

	wmode = inode->dev->wmode;
	slc_ctx = (struct isp_slice_context *)inode->slice_ctx;
	if (slc_ctx->slice_num < 1) {
		pr_err("fail to use slices, not support here.\n");
		return -EINVAL;
	}

	fmcu = (struct isp_fmcu_ctx_desc *)fmcu_handle;
	cur_slc = &slc_ctx->slices[0];
	for (i = 0; i < SLICE_NUM_MAX; i++, cur_slc++) {
		if (cur_slc->valid == 0)
			continue;
		if (wmode != ISP_CFG_MODE) {
			pr_debug("no need to cfg\n");
		} else {
			fmcu_cfg.fmcu = fmcu;
			fmcu_cfg.ctx_id = hw_ctx_id;
			hw->isp_ioctl(hw, ISP_HW_CFG_FMCU_CFG, &fmcu_cfg);
		}

		fetcharg.fmcu = fmcu;
		fetcharg.fetch_info = &cur_slc->slice_fetch;
		hw->isp_ioctl(fmcu, ISP_HW_CFG_SET_SLICE_FETCH, &fetcharg);

		for (j = 0; j < ISP_SPATH_NUM; j++) {
			slc_store = &cur_slc->slice_store[j];
			if (j == ISP_SPATH_FD) {
				thumbscaler.fmcu = fmcu;
				thumbscaler.path_en = cur_slc->path_en[j];
				thumbscaler.ctx_idx = cfg_id;
				thumbscaler.slc_scaler = &cur_slc->slice_thumbscaler;
				hw->isp_ioctl(hw, ISP_HW_CFG_SLICE_SPATH_THUMBSCALER, &thumbscaler);
			} else {
				slc_scaler = &cur_slc->slice_scaler[j];
				spath_scaler.fmcu = fmcu;
				spath_scaler.path_en = cur_slc->path_en[j];
				spath_scaler.ctx_idx = cfg_id;
				spath_scaler.spath_id = j;
				spath_scaler.slc_scaler = slc_scaler;
				hw->isp_ioctl(hw, ISP_HW_CFG_SLICE_SPATH_SCALER, &spath_scaler);
			}
			spath_sotre.fmcu = fmcu;
			spath_sotre.path_en = cur_slc->path_en[j];
			spath_sotre.ctx_idx = cfg_id;
			spath_sotre.spath_id = j;
			spath_sotre.slc_store = slc_store;
			hw->isp_ioctl(hw, ISP_HW_CFG_SLICE_SPATH_STORE, &spath_sotre);
		}

		parg.wmode = wmode;
		parg.hw_ctx_id = hw_ctx_id;
		parg.fmcu = fmcu;
		hw->isp_ioctl(hw, ISP_HW_CFG_SLICE_FMCU_CMD, &parg);
	}
	return 0;
}

static int isp_yuv_scaler_node_postproc_irq(void *handle, uint32_t hw_idx, enum isp_postproc_type type)
{
	timespec cur_ts = {0};
	struct isp_pipe_dev *dev;
	struct isp_yuv_scaler_node *inode = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_scaler_port *port = NULL;
	ktime_t boot_time;
	int hw_path_id= 0;

	if (!handle || type >= POSTPROC_MAX) {
		pr_err("fail to get valid input handle %p, type %d\n",
			handle, type);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)handle;
	inode = (struct isp_yuv_scaler_node *)isp_yuv_scaler_node_hwctx_to_node(hw_idx, handle);

	if (atomic_read(&inode->user_cnt) < 1) {
		pr_debug("contex %d is stopped\n", inode->node_id);
		return 0;
	}
	if (unlikely(inode->started == 0)) {
		pr_debug("ctx %d not started. irq 0x%x\n", inode->node_id);
		return 0;
	}

	isp_scaler_node_context_unbind(inode);
	complete(&inode->frm_done);
	boot_time = ktime_get_boottime();
	ktime_get_ts(&cur_ts);

	pframe = cam_queue_dequeue(&inode->proc_queue, struct camera_frame, list);
	if (pframe) {
		pr_debug("isp cfg_id %d post proc, do not need to return frame\n", inode->cfg_id);
		cam_buf_iommu_unmap(&pframe->buf);
	} else {
		pr_err("fail to get src frame  sw_idx=%d  proc_queue.cnt:%d\n",
			inode->node_id, inode->proc_queue.cnt);
	}

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			pframe = cam_queue_dequeue(&port->result_queue, struct camera_frame, list);
			if (!pframe) {
				pr_err("fail to get frame from queue. ctx:%d, path:%d\n",
					inode->node_id, hw_path_id);
				continue;
			}
			pframe->boot_time = boot_time;
			pframe->time.tv_sec = cur_ts.tv_sec;
			pframe->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;

			pr_debug("scaler node  %d hw_path_id %d, ch_id %d, fid %d, mfd 0x%x, queue cnt:%d, is_reserved %d\n",
				inode->node_id, hw_path_id, pframe->channel_id, pframe->fid, pframe->buf.mfd,
				port->result_queue.cnt, pframe->is_reserved);
			pr_debug("time_sensor %03d.%6d, time_isp %03d.%06d\n",
				(uint32_t)pframe->sensor_time.tv_sec,
				(uint32_t)pframe->sensor_time.tv_usec,
				(uint32_t)pframe->time.tv_sec,
				(uint32_t)pframe->time.tv_usec);
			if (unlikely(pframe->is_reserved)) {
				inode->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, inode->resbuf_cb_data);
			} else {
				cam_buf_iommu_unmap(&pframe->buf);
				inode->data_cb_func(CAM_CB_ISP_RET_DST_BUF, pframe, inode->data_cb_handle);
			}
		}
	}
	return 0;
}

int isp_scaler_port_store_frm_set(struct isp_pipe_info *pipe_info, struct isp_scaler_port *port, struct camera_frame *frame)
{
	int ret = 0, planes = 0, hw_path_id = 0;
	unsigned long offset_u, offset_v, yuv_addr[3] = {0};
	struct isp_store_info *store = NULL;

	if (!pipe_info || !port || !frame) {
		pr_err("fail to get valid input ptr, pipe_info %p, path %p, frame %p\n",
			pipe_info, port, frame);
		return -EINVAL;
	}

	hw_path_id = isp_scaler_port_id_switch(port->port_id);
	pr_debug("enter.port->spath_id %d\n",hw_path_id);
	store = &pipe_info->store[hw_path_id].store;

	if (store->color_fmt == CAM_UYVY_1FRAME)
		planes = 1;
	else if ((store->color_fmt == CAM_YUV422_3FRAME)
			|| (store->color_fmt == CAM_YUV420_3FRAME))
		planes = 3;
	else
		planes = 2;

	if (frame->buf.iova == 0) {
		pr_err("fail to get valid iova address, fd = 0x%x\n",
			frame->buf.mfd);
		return -EINVAL;
	}

	yuv_addr[0] = frame->buf.iova;

	pr_debug("fmt %d, planes %d addr %lx %lx %lx, pitch:%d\n",
		store->color_fmt, planes, yuv_addr[0], yuv_addr[1], yuv_addr[2], store->pitch.pitch_ch0);

	if ((planes > 1) && yuv_addr[1] == 0) {
		offset_u = store->pitch.pitch_ch0 * store->size.h;
		yuv_addr[1] = yuv_addr[0] + offset_u;
	}

	if ((planes > 2) && yuv_addr[2] == 0) {
		offset_v = store->pitch.pitch_ch1 * store->size.h;
		if (store->color_fmt == CAM_YUV420_3FRAME)
			offset_v >>= 1;
		yuv_addr[2] = yuv_addr[1] + offset_v;
	}

	pr_debug("path %d planes %d addr %lx %lx %lx\n",
		hw_path_id, planes, yuv_addr[0], yuv_addr[1], yuv_addr[2]);

	store->addr.addr_ch0 = yuv_addr[0];
	store->addr.addr_ch1 = yuv_addr[1];
	store->addr.addr_ch2 = yuv_addr[2];
	pr_debug("done %x %x %x\n", store->addr.addr_ch0, store->addr.addr_ch1, store->addr.addr_ch2);

	return ret;
}


int isp_yuv_scaler_node_pipeinfo_get(void *arg, void *param, void *frame)
{
	int ret = 0;
	uint32_t hw_path_id = 0;
	struct isp_yuv_scaler_node *inode = NULL;
	struct isp_scaler_uinfo *pipe_src = NULL;
	struct isp_scaler_path_uinfo *path_info = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_pipe_info *pipe_in =NULL;
	struct isp_scaler_port *port = NULL;

	if (!arg || !param || !frame) {
		pr_err("fail to get valid arg %p param %p frame %p\n", arg, param, frame);
		return -EFAULT;
	}
	inode = (struct isp_yuv_scaler_node *) arg;
	pipe_in = (struct isp_pipe_info *)param;
	pframe = (struct camera_frame *)frame;
	pipe_src = &inode->pipe_src;

	pipe_in->fetch.ctx_id = inode->cfg_id;
	ret = ispscaler_port_fetch_normal_get(pipe_src, &pipe_in->fetch, pframe);
	if (ret) {
		pr_err("fail to get pipe fetch info\n");
		return -EFAULT;
	}

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			path_info = &pipe_src->path_info[hw_path_id];
			pipe_in->store[hw_path_id].ctx_id = inode->cfg_id;
			pipe_in->store[hw_path_id].spath_id = hw_path_id;
			ret = ispscaler_port_store_normal_get(path_info, &pipe_in->store[hw_path_id]);
			if (ret) {
				pr_err("fail to get pipe store normal info\n");
				return -EFAULT;
			}

			if (hw_path_id != ISP_SPATH_FD) {
				pipe_in->scaler[hw_path_id].ctx_id = inode->cfg_id;
				pipe_in->scaler[hw_path_id].spath_id = hw_path_id;
				pipe_in->scaler[hw_path_id].src.w = pframe->in_crop.size_x;
				pipe_in->scaler[hw_path_id].src.h = pframe->in_crop.size_y;
				pipe_in->scaler[hw_path_id].in_trim = pframe->out_crop[hw_path_id];
				path_info->scaler_coeff_ex = inode->dev->isp_hw->ip_isp->scaler_coeff_ex;
				path_info->scaler_bypass_ctrl = inode->dev->isp_hw->ip_isp->scaler_bypass_ctrl;
				ret = ispscaler_port_scaler_get(path_info, &pipe_in->scaler[hw_path_id]);
				if (ret) {
					pr_err("fail to get pipe path scaler info\n");
					return -EFAULT;
				}
			} else {
				pipe_in->thumb_scaler.idx = inode->cfg_id;
				if (inode->dev->isp_hw->ip_isp->thumb_scaler_cal_version)
					pipe_in->thumb_scaler.thumbscl_cal_version = 1;
				ret = ispscaler_thumbport_scaler_get(path_info, &pipe_in->thumb_scaler);
				if (ret) {
					pr_err("fail to get pipe thumb scaler info\n");
					return -EFAULT;
				}
			}
		}
	}
	return 0;
}

static struct camera_frame *isp_yuv_scaler_node_path_out_frame_get(struct isp_yuv_scaler_node *inode, struct isp_scaler_port *port,
				struct scaler_tmp_param *tmp,struct camera_frame *pframe)
{
	int ret = 0, hw_path_id = 0;
	struct camera_frame *out_frame = NULL;

	if (!inode || !port || !tmp || !pframe) {
		pr_err("fail to get valid input pctx %p, path %p\n", inode, port);
		return NULL;
	}
	hw_path_id = isp_scaler_port_id_switch(port->port_id);

	pr_debug("hw_path_id %d\n",hw_path_id);

	if (inode->uinfo.path_info[hw_path_id].uframe_sync && tmp->target_fid != CAMERA_RESERVE_FRAME_NUM)
		out_frame = cam_queue_dequeue_if(&port->out_buf_queue, ispscaler_port_fid_check, (void *)&tmp->target_fid);
	else
		out_frame = cam_queue_dequeue(&port->out_buf_queue, struct camera_frame, list);

	if (out_frame)
		tmp->valid_out_frame = 1;
	else
		out_frame = ispscaler_port_reserved_buf_get(inode->resbuf_get_cb, inode->resbuf_cb_data, port);

	if (out_frame != NULL) {
		if (out_frame->is_reserved == 0 && (out_frame->buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
			ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
			pr_debug("map output buffer %08x\n", (uint32_t)out_frame->buf.iova);
			if (ret) {
				cam_queue_enqueue(&port->out_buf_queue, &out_frame->list);
				out_frame = NULL;
				pr_err("fail to map isp iommu buf.\n");
			}
		}
	}

	return out_frame;
}

static uint32_t isp_scaler_node_fid_across_context_get(struct isp_yuv_scaler_node *inode, enum camera_id cam_id)
{
	struct isp_scaler_port *port = NULL;
	struct camera_frame *frame;
	uint32_t target_fid;
	int hw_path_id = 0;

	if (!inode)
		return CAMERA_RESERVE_FRAME_NUM;

	target_fid = CAMERA_RESERVE_FRAME_NUM;

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1 && inode->uinfo.path_info[hw_path_id].uframe_sync) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			frame = cam_queue_dequeue_peek(&port->out_buf_queue, struct camera_frame, list);
			target_fid = min(target_fid, frame->user_fid);
			pr_debug("node id%d path%d user_fid %u\n",inode->node_id, hw_path_id, frame->user_fid);
		}
	}
	pr_debug("target_fid %u, cam_id = %d\n", target_fid, cam_id);

	return target_fid;
}

static int isp_yuv_scaler_node_offline_param_cfg(struct isp_yuv_scaler_node *inode, struct isp_pipe_info *pipe_info, struct camera_frame *pframe, struct scaler_tmp_param *tmp)
{
	struct camera_frame *out_frame = NULL;
	struct isp_scaler_uinfo *pipe_src = NULL;
	struct isp_scaler_path_uinfo *path_info = NULL;
	struct isp_scaler_port *port = NULL;
	int ret = 0, loop = 0, hw_path_id = 0;

	if(!inode || !pipe_info || !pframe || !tmp) {
		pr_err("fail to get param\n");
		return -EINVAL;
	}

	pipe_src = &inode->pipe_src;
	memcpy(pipe_src, &inode->uinfo, sizeof(inode->uinfo));

	pipe_src->in_fmt = pframe->in_fmt;
	pipe_src->src = pframe->in;
	pipe_src->crop = pframe->in_crop;
	pr_debug("pipe_src->in_fmt %d pipe_src->src.w %d pipe_src->src.h %d pipe_src->crop size_x %d pipe_src->crop size_y %d\n",pipe_src->in_fmt,pipe_src->src.w,pipe_src->src.h,pipe_src->crop.size_x,pipe_src->crop.size_y);
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			path_info = &inode->pipe_src.path_info[hw_path_id];
			path_info->dst = pframe->out[hw_path_id];
			path_info->in_trim = pframe->out_crop[hw_path_id];
			pr_debug("hw_path_id %d port->port_id %d path_info->dst w %d h %d in_trim size_x %d in_trim size_y %d path_info->out_fmt %d\n",hw_path_id,port->port_id,path_info->dst.w,path_info->dst.h,path_info->in_trim.size_x,path_info->in_trim.size_y,path_info->out_fmt);
		}
	}
	isp_yuv_scaler_node_pipeinfo_get(inode, pipe_info, pframe);
	ret = isp_yuv_scaler_slice_ctx_init(inode, pipe_info, &tmp->multi_slice);

	if (inode->pipe_src.uframe_sync)
		tmp->target_fid = isp_scaler_node_fid_across_context_get(inode,inode->attach_cam_id);

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			out_frame = isp_yuv_scaler_node_path_out_frame_get(inode, port, tmp, pframe);
			if (out_frame == NULL) {
				pr_err("fail to get out_frame\n");
				return -EINVAL;
			}

			out_frame->fid = pframe->fid;
			out_frame->sensor_time = pframe->sensor_time;
			out_frame->boot_sensor_time = pframe->boot_sensor_time;
			out_frame->link_from.port_id = port->port_id;

			if (inode->ch_id == CAM_CH_CAP)
				pr_info("isp scaler node %d is_reserved %d iova 0x%x, user_fid: %x mfd 0x%x\n",
					inode->node_id, out_frame->is_reserved,
					(uint32_t)out_frame->buf.iova, out_frame->user_fid,
					out_frame->buf.mfd);
			else
				pr_debug("isp scaler node %d, path%d is_reserved %d iova 0x%x, user_fid: %x mfd 0x%x fid: %d\n",
					inode->node_id, hw_path_id, out_frame->is_reserved,
					(uint32_t)out_frame->buf.iova, out_frame->user_fid,
					out_frame->buf.mfd, out_frame->fid);

			/* config store buffer */
			ret = isp_scaler_port_store_frm_set(pipe_info, port, out_frame);
			if (ret) {
				cam_buf_iommu_unmap(&out_frame->buf);
				cam_buf_ionbuf_put(&out_frame->buf);
				cam_queue_empty_frame_put(out_frame);
				pr_err("fail to set store buffer\n");
				return -EINVAL;
			}

			do {
				ret = cam_queue_enqueue(&port->result_queue, &out_frame->list);
				if (ret == 0)
					break;
				printk_ratelimited(KERN_INFO "wait for output queue. loop %d\n", loop);
				/* wait for previous frame output queue done */
				mdelay(1);
			} while (loop++ < 500);

			if (ret) {
				pr_err("fail to enqueue, hw %d, path %d\n",hw_path_id);
				/* ret frame to original queue */
				if (out_frame->is_reserved) {
					inode->resbuf_get_cb(RESERVED_BUF_SET_CB, out_frame, inode->resbuf_cb_data);
				} else {
					cam_buf_iommu_unmap(&out_frame->buf);
					cam_queue_enqueue(&port->out_buf_queue, &out_frame->list);
				}
				return -EINVAL;
			}
		}
	}
	return ret;
}

int isp_yuv_scaler_node_offline_param_set(struct isp_yuv_scaler_node *inode, struct isp_hw_context *pctx_hw, struct camera_frame *pframe)
{
	int hw_path_id = 0;
	struct isp_scaler_port *port = NULL;
	struct isp_pipe_info *pipe_in = NULL;
	struct isp_scaler_uinfo *pipe_src = NULL;

	if (!inode || !pctx_hw || !pframe) {
		pr_err("fail to get input ptr, node %p, pctx_hw %p, pframe %p tmp %p\n",
			inode, pctx_hw, pframe);
		return -EFAULT;
	}

	pipe_src = &inode->pipe_src;
	pipe_in = &pctx_hw->pipe_info;

	isp_hwctx_fetch_set(pctx_hw);

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			isp_hwctx_scaler_set(pctx_hw, hw_path_id, NULL);
			isp_hwctx_store_set(pctx_hw, hw_path_id);
		}
	}
	pr_debug("done\n");
	return 0;
}

static int isp_yuv_scaler_node_start_proc(void *node)
{
	struct isp_scaler_port *port = NULL;
	struct isp_yuv_scaler_node *inode = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_hw_context *pctx_hw = NULL;
	struct scaler_tmp_param tmp = {0};
	struct isp_hw_yuv_block_ctrl blk_ctrl;
	struct cam_hw_info *hw = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	int ret = 0, use_fmcu = 0, kick_fmcu = 0, hw_path_id = 0, loop = 0;

	inode = VOID_PTR_TO(node, struct isp_yuv_scaler_node);
	if (atomic_read(&inode->user_cnt) < 1) {
		pr_err("fail to init isp node\n");
		return EFAULT;
	}
	dev = inode->dev;
	hw = dev->isp_hw;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;

	if (inode->multi_slice || isp_yuv_scaler_node_slice_needed(inode))
		use_fmcu = FMCU_IS_NEED;

	pctx_hw = (struct isp_hw_context *)isp_scaler_node_context_bind(inode, use_fmcu, isp_yuv_scaler_node_postproc_irq);

	pframe = cam_queue_dequeue(&inode->in_queue, struct camera_frame, list);
	if (!pframe || !pctx_hw) {
		pr_err("fail to get param\n");
		ret = -EINVAL;
		goto input_err;
	}
	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. ctx %d\n");
		ret = -EINVAL;
		goto map_err;
	}

	loop = 0;
	do {
		ret = cam_queue_enqueue(&inode->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		/* wait for previous frame proccessed done.*/
		usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
		goto inq_overflow;
	}

	tmp.multi_slice = inode->multi_slice;
	tmp.valid_out_frame = -1;
	tmp.target_fid = CAMERA_RESERVE_FRAME_NUM;
	ret = isp_yuv_scaler_node_offline_param_cfg(inode, &pctx_hw->pipe_info, pframe, &tmp);
	if (ret) {
		pr_err("fail to cfg offline param.\n");
		ret = -EINVAL;
		goto dequeue;
	}

	ret = isp_yuv_scaler_node_offline_param_set(inode, pctx_hw, pframe);
	if (ret) {
		pr_err("fail to set offline param.\n");
		ret = -EINVAL;
		goto dequeue;
	}

	if (tmp.valid_out_frame == -1) {
		pr_info(" No available output buffer cfg id %d, hw %d,discard\n",inode->cfg_id, pctx_hw->hw_ctx_id);
		ret = -EINVAL;
		goto dequeue;
	}

	use_fmcu = 0;
	fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
	if (fmcu) {
		use_fmcu = tmp.multi_slice;
		if (use_fmcu)
			fmcu->ops->ctx_reset(fmcu);
	}

	if (tmp.multi_slice) {
		struct slice_cfg_input slc_cfg;
		memset(&slc_cfg, 0, sizeof(slc_cfg));
		list_for_each_entry(port, &inode->port_queue.head, list) {
			if (atomic_read(&port->user_cnt) >= 1) {
				hw_path_id = isp_scaler_port_id_switch(port->port_id);
				slc_cfg.frame_store[hw_path_id] = &pctx_hw->pipe_info.store[hw_path_id].store;
			}
		}
		slc_cfg.frame_fetch = &pctx_hw->pipe_info.fetch;
		pctx_hw->pipe_info.fetch_fbd_yuv.fetch_fbd_bypass = 1;
		slc_cfg.frame_fbd_yuv = &pctx_hw->pipe_info.fetch_fbd_yuv;
		slc_cfg.frame_in_size.w = inode->pipe_src.crop.size_x;
		slc_cfg.frame_in_size.h = inode->pipe_src.crop.size_y;
		slc_cfg.calc_dyn_ov.verison = hw->ip_isp->dyn_overlap_version;
		isp_yuv_scaler_slice_info_cfg(&slc_cfg, inode->slice_ctx);

		pr_debug("use fmcu support slices for ctx %d\n",inode->cfg_id);
		ret = isp_yuv_scaler_slice_fmcu_cmds_set((void *)fmcu, inode, pctx_hw);
		if (ret == 0)
			kick_fmcu = 1;
	}

	ret = wait_for_completion_interruptible_timeout(&inode->frm_done, ISP_CONTEXT_TIMEOUT);
	if (ret == -ERESTARTSYS) {
		pr_err("fail to interrupt, when isp wait\n");
		ret = -EFAULT;
		goto dequeue;
	} else if (ret == 0) {
		pr_err("fail to wait isp context %d, timeout.\n", inode->cfg_id);
		ret = -EFAULT;
		goto dequeue;
	}

	pctx_hw->iommu_status = (uint32_t)(-1);
	inode->started = 1;
	pctx_hw->fmcu_used = use_fmcu;

	/* start to prepare/kickoff cfg buffer. */
	if (likely(dev->wmode == ISP_CFG_MODE)) {
		pr_debug("cfg enter.");

		/* blkpm_lock to avoid user config block param across frame */
		mutex_lock(&inode->blkpm_lock);
		blk_ctrl.idx = inode->cfg_id;
		blk_ctrl.type = ISP_YUV_BLOCK_DISABLE;
		hw->isp_ioctl(hw, ISP_HW_CFG_YUV_BLOCK_CTRL_TYPE, &blk_ctrl);
		ret = cfg_desc->ops->hw_cfg(cfg_desc, inode->cfg_id, pctx_hw->hw_ctx_id, kick_fmcu);
		mutex_unlock(&inode->blkpm_lock);
		if (kick_fmcu) {
			pr_info("isp scaler node %d fid %d w %d h %d fmcu start\n", inode->cfg_id,pframe->fid, inode->pipe_src.crop.size_x, inode->pipe_src.crop.size_y);
			ret = fmcu->ops->hw_start(fmcu);
		} else {
			pr_debug("cfg start. fid %d\n", pframe->fid);
			hw->isp_ioctl(hw, ISP_HW_CFG_START_ISP, &pctx_hw->hw_ctx_id);
		}
	} else {
		if (kick_fmcu) {
			pr_info("fmcu start.\n");
			ret = fmcu->ops->hw_start(fmcu);
		} else {
			pr_debug("fetch start.\n");
			hw->isp_ioctl(hw, ISP_HW_CFG_FETCH_START, NULL);
		}
	}

	pr_debug("done.\n");
	return 0;
dequeue:
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) >= 1) {
			hw_path_id = isp_scaler_port_id_switch(port->port_id);
			pframe = cam_queue_dequeue_tail(&port->result_queue, struct camera_frame, list);
			if (pframe) {
				/* ret frame to original queue */
				if (pframe->is_reserved)
					inode->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, inode->resbuf_cb_data);
				else
					cam_queue_enqueue(&port->out_buf_queue, &pframe->list);
			}
		}
	}
	pframe = cam_queue_dequeue_tail(&inode->proc_queue, struct camera_frame, list);
inq_overflow:
	if (pframe)
		cam_buf_iommu_unmap(&pframe->buf);
map_err:
input_err:
	if (pctx_hw)
		isp_scaler_node_context_unbind(inode);
	return ret;
}

static void isp_scaler_node_src_frame_ret(void *param)
{
	struct camera_frame *frame = NULL;
	struct isp_yuv_scaler_node *inode = NULL;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	inode = (struct isp_yuv_scaler_node *)frame->priv_data;
	if (!inode) {
		pr_err("fail to get src_frame pctx.\n");
		return;
	}

	pr_debug("frame %p, ch_id %d, buf_fd %d\n", frame, frame->channel_id, frame->buf.mfd);
	isp_scaler_node_offline_pararm_free(frame->param_data);
	frame->param_data = NULL;
	if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
		cam_buf_iommu_unmap(&frame->buf);
	inode->data_cb_func(CAM_CB_ISP_RET_SRC_BUF, frame, inode->data_cb_handle);
}

void *isp_yuv_scaler_node_get (uint32_t node_id, struct isp_yuv_scaler_node_desc *param)
{
	int ret = 0;
	struct isp_yuv_scaler_node *node = NULL;
	struct cam_thread_info *thrd = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_scaler_path_uinfo *path_info = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	if (*param->node_dev == NULL) {
		node = kvzalloc(sizeof(struct isp_yuv_scaler_node), GFP_KERNEL);
		if (!node) {
			pr_err("fail to get valid isp yuv scaler node\n");
			return NULL;
		}
	} else {
		node = *param->node_dev;
		pr_info("isp yuv scaler node has been alloc %p\n", node);
		goto exit;
	}

	node->dev = (struct isp_pipe_dev *)param->dev;
	cfg_desc = (struct isp_cfg_ctx_desc *)node->dev->cfg_handle;
	node->cfg_id = atomic_inc_return(&cfg_desc->scaler_node_cnt);

	if (node->data_cb_func == NULL) {
		node->data_cb_func = param->data_cb_func;
		node->data_cb_handle = param->data_cb_handle;
	}
	node->resbuf_get_cb = param->resbuf_get_cb;
	node->resbuf_cb_data = param->resbuf_cb_data;

	mutex_init(&node->blkpm_lock);
	init_completion(&node->frm_done);
	complete(&node->frm_done);

	node->started = 0;
	node->multi_slice = 0;
	node->ch_id = param->ch_id;
	node->attach_cam_id = param->cam_id;
	node->node_id = node_id;
	node->node_type = param->node_type;
	node->is_bind = 0;

	path_info = &node->uinfo.path_info[param->hw_path_id];
	path_info->dst = param->output_size;
	path_info->out_fmt = param->out_fmt;
	path_info->regular_mode = param->regular_mode;
	path_info->data_endian = param->endian;

	cam_queue_init(&node->in_queue, ISP_IN_Q_LEN, isp_scaler_node_src_frame_ret);
	cam_queue_init(&node->proc_queue, ISP_PROC_Q_LEN, isp_scaler_node_src_frame_ret);
	cam_queue_init(&node->port_queue, PORT_ISP_MAX, NULL);
	cam_queue_init(&node->yuv_scaler_interrupt_queue, ISP_IRQ_Q_LEN, cam_queue_empty_interrupt_put);

	thrd = &node->thread;
	thrd->data_cb_func = node->data_cb_func;
	thrd->data_cb_handle = node->data_cb_handle;
	thrd->error_type = CAM_CB_ISP_DEV_ERR;
	sprintf(thrd->thread_name, "isp scaler node %d_offline", node->node_id);
	ret = camthread_create(node, thrd, isp_yuv_scaler_node_start_proc);
	if (ret) {
		pr_err("fail to get isp node nterruption_proc thread.\n");
		return NULL;
	}

	thrd = &node->yuv_scaler_interrupt_thread;
	thrd->data_cb_func = node->data_cb_func;
	thrd->data_cb_handle = node->data_cb_handle;
	thrd->error_type = CAM_CB_ISP_DEV_ERR;
	sprintf(thrd->thread_name, "isp scaler interrupt %d_interrupt", node->node_id);
	ret = camthread_create(node, thrd, isp_int_yuv_scaler_interruption_proc);
	if (ret) {
		pr_err("fail to get isp node nterruption_proc thread.\n");
		return NULL;
	}

	pr_info("isp scaler node cfg_id %d node %px, node_id %d\n",node->cfg_id, node, node_id);
	*param->node_dev = node;

exit:
	atomic_inc(&node->user_cnt);
	return node;
}

void isp_yuv_scaler_node_put (struct isp_yuv_scaler_node *node)
{
	int ret = 0;
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	if (node->started == 1) {
		pr_info("free context %d without users.\n", node->node_id);
		/* wait for last frame done */
		ret = wait_for_completion_interruptible_timeout(
			&node->frm_done, ISP_CONTEXT_TIMEOUT);
		if (ret == -ERESTARTSYS)
			pr_err("fail to interrupt, when isp wait\n");
		else if (ret == 0)
			pr_err("fail to wait ctx %d, timeout.\n", node->node_id);
		else
			pr_info("wait time %d\n", ret);

		node->started = 0;
		isp_scaler_node_context_unbind(node);
		cam_queue_clear(&node->in_queue, struct camera_frame, list);
		cam_queue_clear(&node->proc_queue, struct camera_frame, list);
		cam_queue_clear(&node->port_queue, struct isp_scaler_port, list);
	}

	if (atomic_dec_return(&node->user_cnt) == 0) {
		mutex_destroy(&node->blkpm_lock);
		camthread_stop(&node->thread);
		camthread_stop(&node->yuv_scaler_interrupt_thread);
		node->data_cb_func = NULL;
		node->resbuf_get_cb = NULL;
		if (node->slice_ctx)
			isp_slice_ctx_put(&node->slice_ctx);
		cam_queue_clear(&node->yuv_scaler_interrupt_queue, struct camera_interrupt, list);
		atomic_dec(&((struct isp_cfg_ctx_desc *)(node->dev->cfg_handle))->scaler_node_cnt);

		pr_info("isp yuv scaler node %d put success\n", node->node_id);
		kvfree(node);
		node = NULL;
	}
}
