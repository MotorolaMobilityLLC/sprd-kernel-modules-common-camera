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
#include "cam_statis.h"
#include "isp_fmcu.h"
#include "isp_drv.h"
#include "isp_slice.h"
#include "isp_cfg.h"
#include "isp_node.h"
#include "isp_int.h"
#include "cam_port.h"
#include "isp_pyr_rec.h"
#include "isp_gtm.h"
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_OFFLINE_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

static int ispnode_stream_state_get(struct isp_node *node,struct camera_frame *pframe)
{
	int ret = 0,i = 0, hw_path_id = 0;
	uint32_t normal_cnt = 0, postproc_cnt = 0;
	uint32_t maxw = 0, maxh = 0, scl_x = 0, scl_w = 0, scl_h = 0;
	uint32_t min_crop_w = 0xFFFFFFFF, min_crop_h = 0xFFFFFFFF;
	struct isp_stream_ctrl tmp_stream[5] = {0};
	struct isp_port *port = NULL;

	if (!node || !pframe) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	normal_cnt = 1;
	list_for_each_entry(port, &node->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
			maxw = MAX(maxw, port->size.w);
			maxh = MAX(maxh, port->size.h);
			min_crop_w = MIN(min_crop_w, port->trim.size_x);
			min_crop_h = MIN(min_crop_h, port->trim.size_y);
		}
	}
	if (!maxw || !maxh) {
		pr_err("fail to get valid max dst size\n");
		return -EFAULT;
	}

	scl_w = maxw / min_crop_w;
	if ((maxw % min_crop_w) != 0)
		scl_w ++;
	scl_h = maxh / min_crop_h;
	if ((maxh % min_crop_h) != 0)
		scl_h ++;
	scl_x = MAX(scl_w, scl_h);
	pr_debug("scl_x %d scl_w %d scl_h %d max_w %d max_h %d\n", scl_x,
		scl_w, scl_h, maxw, maxh);
	do {
		if (scl_x == ISP_SCALER_UP_MAX)
			break;
		scl_x = scl_x / ISP_SCALER_UP_MAX;
		if (scl_x)
			postproc_cnt++;
	} while (scl_x);

	/* If need postproc, first normal proc need to mark as postproc too*/
	if (postproc_cnt) {
		postproc_cnt++;
		normal_cnt--;
	}

	for (i = postproc_cnt - 1; i >= 0; i--) {
		maxw = maxw / ISP_SCALER_UP_MAX;
		maxw = ISP_ALIGN_W(maxw);
		maxh = maxh / ISP_SCALER_UP_MAX;
		maxh = ISP_ALIGN_H(maxh);

		if (i == 0) {
			list_for_each_entry(port, &node->port_queue.head, list)
			if (port->type == PORT_TRANSFER_IN) {
				tmp_stream[i].in_fmt = port->fmt;
				tmp_stream[i].in = port->size;
				tmp_stream[i].in_crop = port->trim;
			}
		} else {
			tmp_stream[i].in_fmt = CAM_YVU420_2FRAME;
			tmp_stream[i].in.w = maxw;
			tmp_stream[i].in.h = maxh;
			tmp_stream[i].in_crop.start_x = 0;
			tmp_stream[i].in_crop.start_y = 0;
			tmp_stream[i].in_crop.size_x = maxw;
			tmp_stream[i].in_crop.size_y = maxh;
		}

		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				/* This is for ensure the last postproc frame buf is OUT for user
				.* Then the frame before should be POST. Thus, only one post
				.* buffer is enough for all the isp postproc process */
				if ((postproc_cnt + i) % 2 == 1)
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_OUT;
				else
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_POSTPROC;
				if (hw_path_id != ISP_SPATH_CP && (i != postproc_cnt - 1))
					tmp_stream[i].buf_type[hw_path_id] = ISP_STREAM_BUF_RESERVED;
				if (i == (postproc_cnt - 1)) {
					tmp_stream[i].out[hw_path_id] = port->size;
					tmp_stream[i].out_crop[hw_path_id].start_x = 0;
					tmp_stream[i].out_crop[hw_path_id].start_y = 0;
					tmp_stream[i].out_crop[hw_path_id].size_x = maxw;
					tmp_stream[i].out_crop[hw_path_id].size_y = maxh;
				} else if (i == 0) {
					tmp_stream[i].out[hw_path_id].w = tmp_stream[i + 1].in.w;
					tmp_stream[i].out[hw_path_id].h = tmp_stream[i + 1].in.h;
					tmp_stream[i].out_crop[hw_path_id] = port->trim;
				} else {
					tmp_stream[i].out[hw_path_id].w = tmp_stream[i + 1].in.w;
					tmp_stream[i].out[hw_path_id].h = tmp_stream[i + 1].in.h;
					tmp_stream[i].out_crop[hw_path_id].start_x = 0;
					tmp_stream[i].out_crop[hw_path_id].start_y = 0;
					tmp_stream[i].out_crop[hw_path_id].size_x = maxw;
					tmp_stream[i].out_crop[hw_path_id].size_y = maxh;
				}
				pr_debug("isp %d i %d hw_path_id %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, i, hw_path_id, tmp_stream[i].out[hw_path_id].w, tmp_stream[i].out[hw_path_id].h,
					tmp_stream[i].out_crop[hw_path_id].start_x, tmp_stream[i].out_crop[hw_path_id].start_y,
					tmp_stream[i].out_crop[hw_path_id].size_x, tmp_stream[i].out_crop[hw_path_id].size_y);
			}
		}
		pr_debug("isp %d index %d in_size %d %d crop_szie %d %d %d %d\n",
			node->cfg_id, i, tmp_stream[i].in.w, tmp_stream[i].in.h,
			tmp_stream[i].in_crop.start_x, tmp_stream[i].in_crop.start_y,
			tmp_stream[i].in_crop.size_x, tmp_stream[i].in_crop.size_y);
	}

	/***one scaler out_size***/
	for (i = 0; i < normal_cnt; i++) {
		pframe->state = ISP_STREAM_NORMAL_PROC;
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
					pframe->buf_type = ISP_STREAM_BUF_OUT;
				pframe->out[hw_path_id] = port->size;
				pframe->out_crop[hw_path_id] = port->trim;
				pr_debug("isp %d hw_path_id %d normal_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, normal_cnt ,pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);
			}
		}
	}

	/***two scaler out_size***/
	if (postproc_cnt >= 2) {
		pframe->state = ISP_STREAM_POST_PROC;
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				pframe->buf_type = tmp_stream[0].buf_type[hw_path_id];
				if (hw_path_id != ISP_SPATH_CP)
					pframe->buf_type = ISP_STREAM_BUF_RESERVED;
				pframe->out[hw_path_id] = tmp_stream[0].out[hw_path_id];
				pframe->out_crop[hw_path_id] = tmp_stream[0].out_crop[hw_path_id];
				pr_debug("isp %d hw_path_id %d postproc_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, postproc_cnt, pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);

			}
		}
	}

	/***3dnr cap or 3dnr cap 10x out_size***/
	if (node->uinfo.mode_3dnr == MODE_3DNR_CAP) {
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) >= 1 && atomic_read(&port->is_work) > 0) {
				hw_path_id = isp_port_id_switch(port->port_id);
				if (postproc_cnt && node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					pframe->buf_type = ISP_STREAM_BUF_POSTPROC;
				else if (node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					pframe->buf_type = ISP_STREAM_BUF_OUT;
				else
					pframe->buf_type = ISP_STREAM_BUF_RESERVED;
				pframe->nr3_blend_cnt = node->nr3_blend_cnt;
				if (node->nr3_blend_cnt == (NR3_BLEND_CNT - 1))
					node->nr3_blend_cnt = 0;
				node->nr3_blend_cnt++;
				pframe->out[hw_path_id] = port->size;
				pframe->out_crop[hw_path_id] = port->trim;
				if (postproc_cnt) {
					pframe->out[hw_path_id] = tmp_stream[0].out[hw_path_id];
					pframe->out_crop[hw_path_id] = tmp_stream[0].out_crop[hw_path_id];
				}
				pr_debug("isp %d hw_path_id %d postproc_cnt %d node->nr3_blend_cnt %d out_size %d %d crop_szie %d %d %d %d\n",
					node->cfg_id, hw_path_id, postproc_cnt, node->nr3_blend_cnt, pframe->out[hw_path_id].w, pframe->out[hw_path_id].h,
					pframe->out_crop[hw_path_id].start_x, pframe->out_crop[hw_path_id].start_y,
					pframe->out_crop[hw_path_id].size_x, pframe->out_crop[hw_path_id].size_y);
			}
		}
	}
	return ret;
}

static uint32_t ispnode_slice_needed(struct isp_node *node)
{
	struct isp_port *port = NULL;

	list_for_each_entry(port, &node->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) > 0 && port->port_cfg_cb_func(NULL, ISP_PORT_SLICE_NEED, port))
			return 1;
	}
	return 0;
}

static int ispnode_postproc_frame_return(struct isp_node *inode)
{
	struct isp_port *port = NULL;
	ktime_t boot_time;
	timespec cur_ts = {0};
	struct isp_node_postproc_param post_param = {0};

	boot_time = ktime_get_boottime();
	ktime_get_ts(&cur_ts);
	post_param.boot_time = &boot_time;
	post_param.cur_ts = &cur_ts;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) > 0 && atomic_read(&port->is_work) > 0)
			port->port_cfg_cb_func(&post_param, ISP_PORT_IRQ_POSTPORC, port);
	}
	return 0;
}

static void ispnode_rgb_gtm_hist_done_process(struct isp_node *inode, uint32_t hw_idx, void *dev)
{
	struct isp_gtm_ctx_desc *gtm_ctx = NULL;
	int wait_fid = 0;
	struct camera_frame *frame = NULL;
	uint32_t *buf = NULL;
	uint32_t hist_total = 0;
	int ret = 0;

	gtm_ctx = (struct isp_gtm_ctx_desc *)inode->rgb_gtm_handle;
	if (!gtm_ctx) {
		pr_err("fail to gtm handle ptr\n");
		return;
	}

	if (gtm_ctx->calc_mode == GTM_SW_CALC) {
		frame  = cam_queue_dequeue(&inode->gtmhist_result_queue, struct camera_frame, list);
		if (!frame) {
			pr_debug("isp ctx_id[%d] gtmhist_result_queue buffer \n", gtm_ctx->ctx_id);
			return;
		} else {
			buf = (uint32_t *)frame->buf.addr_k;
			if (!buf) {
				cam_queue_enqueue(&inode->gtmhist_result_queue, &frame->list);
				return;
			} else {
				hist_total= gtm_ctx->src.w * gtm_ctx->src.h;
				ret = isp_hwctx_gtm_hist_result_get(frame, hw_idx, dev, hist_total, gtm_ctx->fid);
				if (ret) {
					cam_queue_enqueue(&inode->gtmhist_result_queue, &frame->list);
					return;
				}
				inode->data_cb_func(CAM_CB_ISP_STATIS_DONE, frame, inode->data_cb_handle);
			}
		}
	} else {
		gtm_ctx->gtm_ops.get_preview_hist_cal(gtm_ctx);
		/*for capture*/
		wait_fid = gtm_ctx->gtm_ops.sync_completion_get(gtm_ctx);
		if (wait_fid)
			gtm_ctx->gtm_ops.sync_completion_done(gtm_ctx);
	}
}

static int ispnode_postproc_irq(void *handle, uint32_t hw_idx, enum isp_postproc_type type)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_node *inode = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_ltm_ctx_desc *rgb_ltm = NULL;
	int completion = 0, i = 0, ret = 0;

	dev = (struct isp_pipe_dev *)handle;
	if (!handle || type >= POSTPROC_MAX) {
		pr_err("fail to get valid input handle %p, type %d\n",
			handle, type);
		return -EFAULT;
	}

	inode = (struct isp_node *)dev->hw_ctx[hw_idx].node;
	inode->in_irq_postproc = 1;
	if (atomic_read(&inode->user_cnt) < 1) {
		pr_debug("contex %d is stopped\n", inode->node_id);
		inode->in_irq_postproc = 0;
		return 0;
	}
	switch (type) {
	case POSTPROC_FRAME_DONE:
		if (inode->uinfo.enable_slowmotion == 0) {
			inode->dev->isp_ops->unbind(inode);
			complete(&inode->frm_done);
		}

		if (inode->is_fast_stop) {
			struct isp_port_cfg port_cfg = {0};
			struct isp_port *port = NULL;
			port_cfg.faststop = &inode->is_fast_stop;
			port_cfg.faststop_done = inode->fast_stop_done;
			port_cfg.param_share_queue = &inode->param_share_queue;
			port_cfg.out_buf_clear = 0;
			list_for_each_entry(port, &inode->port_queue.head, list) {
				if (atomic_read(&port->user_cnt) > 0 && atomic_read(&port->is_work) > 0)
					port->port_cfg_cb_func(&port_cfg, ISP_PORT_FAST_STOP, port);
			}
			break;
		}
		ispnode_postproc_frame_return(inode);
		break;
	case POSTPROC_SLOWMOTION_FRAMEDONE:
		inode->dev->isp_ops->unbind(inode);
		complete(&inode->frm_done);
		for (i = 0; i < inode->uinfo.slowmotion_count - 1; i++)
			ispnode_postproc_frame_return(inode);
		break;
	case POSTPROC_RGB_LTM_HISTS_DONE:
		rgb_ltm = (struct isp_ltm_ctx_desc *)inode->rgb_ltm_handle;
		if (!rgb_ltm) {
			ret = -EFAULT;
			goto exit;
		}

		rgb_ltm->ltm_ops.sync_ops.set_frmidx(rgb_ltm);
		completion = rgb_ltm->ltm_ops.sync_ops.get_completion(rgb_ltm);
		if (completion && (rgb_ltm->fid >= completion))
			completion = rgb_ltm->ltm_ops.sync_ops.do_completion(rgb_ltm);
		break;
	case POSTPROC_RGB_GTM_HISTS_DONE:
		ispnode_rgb_gtm_hist_done_process(inode, hw_idx, dev);
		break;
	case POSTPROC_HIST_CAL_DONE:
		/* only use isp hist in preview channel */
		if (inode->node_id != 0) {
			ret = -EFAULT;
			goto exit;
		}
		pframe = cam_queue_dequeue(&inode->hist2_result_queue, struct camera_frame, list);
		if (!pframe) {
			pr_debug("isp ctx_id[%d] hist2_result_queue unavailable\n", inode->node_id);
			ret = -EFAULT;
			goto exit;
		}
		isp_hwctx_hist2_frame_prepare(pframe, hw_idx, dev);
		inode->data_cb_func(CAM_CB_ISP_STATIS_DONE, pframe, inode->data_cb_handle);
		break;
	default:
		pr_err("fail to get type:%d", type);
	}
exit:
	inode->in_irq_postproc = 0;
	return ret;
}

static uint32_t ispnode_fid_across_context_get(struct isp_node *inode, enum camera_id cam_id)
{
	struct isp_port *port = NULL;
	uint32_t target_fid = 0, port_fid = 0;

	if (!inode)
		return CAMERA_RESERVE_FRAME_NUM;

	target_fid = CAMERA_RESERVE_FRAME_NUM;

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) > 0) {
			port->port_cfg_cb_func(&port_fid, ISP_PORT_UFRAME_FID_GET, port);
			target_fid = min(target_fid, port_fid);
			pr_debug("port%d user_fid %u\n", port->port_id, port_fid);
		}
	}
	pr_debug("target_fid %u, cam_id = %d\n", target_fid, cam_id);

	return target_fid;
}

static int ispnode_blkparam_adapt(struct isp_node *inode)
{
	struct isp_hw_k_blk_func sub_blk_func = {0};
	struct isp_port_blksize_desc size_desc = {0};
	struct isp_port *port = NULL;
	struct isp_k_block_param block_param = {0};

	size_desc.ch_id = inode->ch_id;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
			port->port_cfg_cb_func(&size_desc, ISP_PORT_BLKSIZE_GET, port);
	}

	inode->isp_k_param.blkparam_info.new_height = size_desc.new_height;
	inode->isp_k_param.blkparam_info.new_width = size_desc.new_width;
	inode->isp_k_param.blkparam_info.old_height = size_desc.old_height;
	inode->isp_k_param.blkparam_info.old_width = size_desc.old_width;
	inode->isp_k_param.sn_size = size_desc.sn_size;
	inode->isp_k_param.src_w = size_desc.src_size.w;
	inode->isp_k_param.src_h = size_desc.src_size.h;
	inode->isp_using_param->blkparam_info.new_height = size_desc.new_height;
	inode->isp_using_param->blkparam_info.new_width = size_desc.new_width;
	inode->isp_using_param->blkparam_info.old_height = size_desc.old_height;
	inode->isp_using_param->blkparam_info.old_width = size_desc.old_width;
	inode->isp_using_param->sn_size = size_desc.sn_size;
	inode->isp_using_param->src_w = size_desc.src_size.w;
	inode->isp_using_param->src_h = size_desc.src_size.h;

	block_param.cfg_id = inode->cfg_id;
	block_param.isp_k_param = &inode->isp_k_param;
	block_param.isp_using_param = inode->isp_using_param;
	pr_debug("ch_id: %d ctx_id:%d crop %d %d %d %d, size(%d %d) => (%d %d)\n", inode->ch_id, inode->node_id,
		size_desc.old_width, size_desc.old_height, size_desc.new_width, size_desc.new_height);

	sub_blk_func.index = ISP_K_BLK_NLM_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	sub_blk_func.index = ISP_K_BLK_IMBLANCE_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	sub_blk_func.index = ISP_K_BLK_YNR_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	sub_blk_func.index = ISP_K_BLK_CNR_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	sub_blk_func.index = ISP_K_BLK_POST_CNR_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	sub_blk_func.index = ISP_K_BLK_EDGE_UPDATE;
	inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_K_BLK_FUNC_GET, &sub_blk_func);
	if (sub_blk_func.k_blk_func)
		sub_blk_func.k_blk_func(&block_param);

	if (inode->uinfo.mode_3dnr != MODE_3DNR_OFF)
		isp_k_update_3dnr(inode->node_id, inode->isp_using_param,
			 size_desc.new_width, size_desc.old_width, size_desc.new_height, size_desc.old_height);

	return 0;
}

static int ispnode_offline_param_set(struct isp_node *inode, struct isp_hw_context *pctx_hw)
{
	struct isp_port *port = NULL;
	if (!inode || !pctx_hw) {
		pr_err("fail to get input ptr, pctx %p, pctx_hw %p, pframe %p tmp %p\n",
			inode, pctx_hw);
		return -EFAULT;
	}

	isp_hwctx_fetch_set(pctx_hw);
	if (inode->uinfo.fetch_path_sel == ISP_FETCH_PATH_FBD)
		isp_hwctx_fetch_fbd_set(pctx_hw);

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) > 0 && atomic_read(&port->is_work) > 0) {
			isp_hwctx_scaler_set(pctx_hw, isp_port_id_switch(port->port_id), NULL);
			isp_hwctx_store_set(pctx_hw, isp_port_id_switch(port->port_id));
		}
	}

	return 0;
}

static int ispnode_port_param_cfg(struct isp_node *inode, struct isp_hw_context *pctx_hw, struct isp_port_cfg *port_cfg)
{
	uint32_t ret = 0;
	struct isp_port *port = NULL;
	struct isp_hw_yuv_block_ctrl blk_ctrl = {0};

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
			ret |= port->port_cfg_cb_func(port_cfg, ISP_PORT_FRAME_CYCLE, port);
	}

	if (ret) {
		pr_err("fail fetch frame cycle");
		return -1;
	}

	isp_node_prepare_blk_param(inode, port_cfg->src_frame->fid, &port_cfg->src_frame->blkparam_info);
	inode->isp_using_param = port_cfg->src_frame->blkparam_info.param_block;
	if (port_cfg->src_frame->blkparam_info.update == 1) {
		blk_ctrl.idx = inode->cfg_id;
		blk_ctrl.blk_param = inode->isp_using_param;
		inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_SUBBLOCK_RECFG, &blk_ctrl);
	}

	port_cfg->target_fid = CAMERA_RESERVE_FRAME_NUM;
	port_cfg->rgb_ltm = (struct isp_ltm_ctx_desc *)inode->rgb_ltm_handle;
	port_cfg->pipe_info = &pctx_hw->pipe_info;
	port_cfg->cfg_id = inode->cfg_id;
	port_cfg->sec_mode = inode->dev->sec_mode;
	port_cfg->scaler_coeff_ex = inode->dev->isp_hw->ip_isp->scaler_coeff_ex;
	port_cfg->scaler_bypass_ctrl = inode->dev->isp_hw->ip_isp->scaler_bypass_ctrl;
	port_cfg->uinfo = &inode->uinfo;
	port_cfg->superzoom_frame = inode->postproc_buf;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		ret |= port->port_cfg_cb_func(port_cfg, ISP_PORT_PIPEINFO_GET, port);
	}
	if (inode->pipe_src.uframe_sync)
		port_cfg->target_fid = ispnode_fid_across_context_get(inode, inode->attach_cam_id);

	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) > 0 && atomic_read(&port->is_work) > 0)
			ret |= port->port_cfg_cb_func(port_cfg, ISP_PORT_FRAME_CYCLE, port);
	}
	ret |= ispnode_offline_param_set(inode, pctx_hw);

	return ret;
}

static int ispnode_3dnr_frame_process(struct isp_node *inode, struct isp_hw_fetch_info *fetch, struct isp_port_cfg *port_cfg)
{
	uint32_t mv_version = 0, blend_cnt = 0;
	struct isp_3dnr_ctx_desc *nr3_handle = NULL;

	if (inode == NULL || fetch == NULL || port_cfg == NULL) {
		pr_err("fail to get valid parameter pctx %p fetch %p pframe %p\n", inode, fetch, port_cfg);
		return 0;
	}

	pr_debug("fid %d, valid %d, x %d, y %d, w %u, h %u\n",
			port_cfg->src_frame->fid,port_cfg->src_frame->nr3_me.valid,
			port_cfg->src_frame->nr3_me.mv_x, port_cfg->src_frame->nr3_me.mv_y,
			port_cfg->src_frame->nr3_me.src_width, port_cfg->src_frame->nr3_me.src_height);

	mv_version = inode->dev->isp_hw->ip_isp->nr3_mv_alg_version;
	nr3_handle = (struct isp_3dnr_ctx_desc *)inode->nr3_handle;
	nr3_handle->pyr_rec_eb = port_cfg->src_frame->need_pyr_rec;

	if (inode->uinfo.mode_3dnr == MODE_3DNR_CAP) {
		blend_cnt = (port_cfg->src_frame->nr3_blend_cnt) % NR3_BLEND_CNT;
		nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_MODE, &inode->uinfo.mode_3dnr);
		nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_BLEND_CNT, &blend_cnt);
	}

	nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_MV_VERSION, &mv_version);
	nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_FBC_FBD_INFO, &inode->pipe_src.nr3_fbc_fbd);
	nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_SIZE_INFO, &port_cfg->src_crop);
	nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_MEMCTL_STORE_INFO, fetch);
	nr3_handle->ops.cfg_param(nr3_handle, ISP_3DNR_CFG_BLEND_INFO, inode->isp_using_param);
	nr3_handle->ops.pipe_proc(nr3_handle, &port_cfg->src_frame->nr3_me, inode->uinfo.mode_3dnr);

	return 0;
}

static int ispnode_ltm_frame_process(struct isp_node *inode, struct isp_port_cfg *port_cfg)
{
	int ret = 0;
	struct isp_ltm_ctx_desc *rgb_ltm = NULL;

	if (!inode || !port_cfg) {
		pr_err("fail to get valid parameter inode %p port_cfg %p\n",
			inode, port_cfg);
	}

	rgb_ltm = (struct isp_ltm_ctx_desc *)inode->rgb_ltm_handle;
	if (!rgb_ltm)
		return 0;

	if (inode->ch_id == CAM_CH_PRE && ispnode_slice_needed(inode)) {
		inode->pipe_src.mode_ltm = MODE_LTM_OFF;
		rgb_ltm->sync->pre_slice_ltm_bypass = 1;
	}

	rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_HIST_BYPASS, &port_cfg->src_frame->xtm_conflict.need_ltm_hist);
	rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_MAP_BYPASS, &port_cfg->src_frame->xtm_conflict.need_ltm_map);
	rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_MODE, &inode->pipe_src.mode_ltm);
	rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_FRAME_ID, &port_cfg->src_frame->fid);
	rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_SIZE_INFO, &port_cfg->src_crop);
	ret = rgb_ltm->ltm_ops.core_ops.pipe_proc(rgb_ltm, &inode->isp_using_param->ltm_rgb_info);
	if (ret == -1) {
		inode->pipe_src.mode_ltm = MODE_LTM_OFF;
		pr_err("fail to rgb LTM cfg frame, DISABLE\n");
	}

	return ret;
}

static int ispnode_gtm_frame_process(struct isp_node *inode, struct isp_port_cfg *port_cfg)
{
	int ret = 0;
	struct isp_gtm_ctx_desc *rgb_gtm = NULL;
	struct dcam_dev_raw_gtm_block_info *gtm_rgb_info = NULL;

	if (!inode || !port_cfg) {
		pr_err("fail to get valid inode %p port_cfg %p\n", inode, port_cfg);
		return -EINVAL;
	}

	rgb_gtm = (struct isp_gtm_ctx_desc *)inode->rgb_gtm_handle;

	if (!rgb_gtm)
		return 0;/*not support GTM in isp*/

	pr_debug("ctx_id %d, mode %d, fid %d\n", rgb_gtm->ctx_id, rgb_gtm->mode, port_cfg->src_frame->fid);

	rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_FRAME_ID, &port_cfg->src_frame->fid);
	rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_HIST_BYPASS, &port_cfg->src_frame->xtm_conflict.need_gtm_hist);
	rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_MAP_BYPASS, &port_cfg->src_frame->xtm_conflict.need_gtm_map);
	rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_MOD_EN, &port_cfg->src_frame->xtm_conflict.gtm_mod_en);
	rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_CALC_MODE, &inode->isp_using_param->gtm_calc_mode);
	gtm_rgb_info = &inode->isp_using_param->gtm_rgb_info;
	rgb_gtm->src.w = port_cfg->src_crop.size_x;
	rgb_gtm->src.h = port_cfg->src_crop.size_y;
	ret = rgb_gtm->gtm_ops.pipe_proc(rgb_gtm, gtm_rgb_info, &inode->isp_using_param->gtm_sw_map_info);
	if (ret) {
		inode->pipe_src.mode_gtm = MODE_GTM_OFF;
		pr_err("fail to rgb GTM process, GTM_OFF\n");
	}

	return ret;
}

static int ispnode_rec_frame_process(struct isp_node *inode, struct isp_hw_context *pctx_hw, struct isp_port_cfg *port_cfg)
{
	int ret = 0;
	struct isp_pipe_info *pipe_info = NULL;
	struct isp_pyr_rec_in cfg_in = {0};
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct isp_slice_context *slc_ctx = NULL;

	if (!inode || !pctx_hw || !port_cfg) {
		pr_err("fail to get valid parameter inode %p pipe_info %p port_cfg %p\n",
			inode, pctx_hw, port_cfg);
		return -EINVAL;
	}
	pipe_info = &pctx_hw->pipe_info;
	slc_ctx = (struct isp_slice_context *)pctx_hw->slice_ctx;
	rec_ctx = (struct isp_rec_ctx_desc *)inode->rec_handle;
	if (!rec_ctx)
		return 0;

	cfg_in.src = pipe_info->fetch.src;
	cfg_in.in_addr = pipe_info->fetch.addr;
	cfg_in.in_trim = pipe_info->fetch.in_trim;
	cfg_in.in_fmt = pipe_info->fetch.fetch_fmt;
	cfg_in.pyr_fmt = pipe_info->fetch.fetch_pyr_fmt;
	cfg_in.pyr_ynr_radius = inode->isp_using_param->ynr_radius;
	cfg_in.pyr_cnr_radius = inode->isp_using_param->cnr_radius;
	cfg_in.slice_overlap = &slc_ctx->slice_overlap;
	cfg_in.pyr_cnr = &inode->isp_using_param->cnr_info;
	cfg_in.pyr_ynr = &inode->isp_using_param->ynr_info_v3;
	cfg_in.out_addr = pipe_info->store[ISP_SPATH_CP].store.addr;

	if (port_cfg->src_frame->is_compressed == 1) {
		rec_ctx->fetch_path_sel = port_cfg->src_frame->is_compressed;
		rec_ctx->fetch_fbd = pipe_info->fetch_fbd_yuv;
		rec_ctx->fbcd_buffer_size = pipe_info->fetch_fbd_yuv.buffer_size;
	}

	rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_FMCU_HANDLE, pctx_hw->fmcu_handle);
	rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_LAYER_NUM, &inode->uinfo.pyr_layer_num);
	if (port_cfg->src_frame->need_pyr_rec) {
		rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_WORK_MODE, &inode->dev->wmode);
		rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_HW_CTX_IDX, &pctx_hw->hw_ctx_id);
		ret = rec_ctx->ops.pipe_proc(rec_ctx, &cfg_in);
		if (ret == -1)
			pr_err("fail to proc rec frame\n");
	} else
		rec_ctx->pyr_rec.reconstruct_bypass = 1;

	return ret;
}

static int ispnode_fmcu_slw_queue_set(struct isp_fmcu_ctx_desc *fmcu, struct isp_node *inode, struct isp_hw_context *pctx_hw, uint32_t vid_valid)
{
	int ret = 0;
	struct isp_port *port = NULL;
	struct isp_hw_slw_fmcu_cmds slw = {0};
	struct isp_port_cfg port_cfg = {0};
	if (!fmcu)
		return -EINVAL;

	port_cfg.slw = &slw;
	port_cfg.pipe_info = &pctx_hw->pipe_info;
	port_cfg.dev = inode->dev;
	port_cfg.vid_valid = vid_valid;
	port_cfg.uinfo = &inode->uinfo;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) > 0)
			ret |= port->port_cfg_cb_func(&port_cfg, ISP_PORT_SLOWMOTION, port);
	}

	slw.fmcu_handle = fmcu;
	slw.ctx_id = inode->cfg_id;
	slw.fetchaddr = pctx_hw->pipe_info.fetch.addr;
	slw.isp_store = pctx_hw->pipe_info.store;
	slw.is_compressed = port_cfg.src_frame->is_compressed;
	ret = inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_SLW_FMCU_CMDS, &slw);

	pr_debug("fmcu slw queue done!");
	return ret;
}

static int ispnode_copy_param(struct camera_frame *param_frame, struct isp_node *inode)
{
	int ret = 0, i = 0;
	struct cam_hw_info *ops = inode->dev->isp_hw;
	struct isp_cfg_block_param fucarg = {0};
	func_isp_cfg_block_param cfg_fun_ptr = NULL;

	for(i = ISP_BLOCK_BCHS; i < ISP_BLOCK_TOTAL; i++) {
		fucarg.index = i - ISP_BLOCK_BASE;
		ops->isp_ioctl(ops, ISP_HW_CFG_PARAM_BLOCK_FUNC_GET, &fucarg);
		if (fucarg.isp_param != NULL && fucarg.isp_param->cfg_block_func != NULL) {
			cfg_fun_ptr = fucarg.isp_param->cfg_block_func;
			ret = cfg_fun_ptr(&inode->isp_k_param, param_frame->blkparam_info.param_block);
		}
	}
	return ret;
}

static uint32_t ispnode_slw_need_vid_num(struct isp_node_uinfo *uinfo)
{
	uint32_t vid_valid_count = 0, res_num = 0, valid_cnt = 0;

	if (uinfo->stage_a_valid_count == 0)
		return (uinfo->slowmotion_count - 1);

	if (uinfo->frame_num.stage_a_frame_num > 0) {
		res_num = uinfo->frame_num.stage_a_frame_num;
		valid_cnt = uinfo->stage_a_valid_count;
	} else if (uinfo->frame_num.stage_b_frame_num > 0) {
		res_num = uinfo->frame_num.stage_b_frame_num;
		valid_cnt = uinfo->slowmotion_count;
	} else if(uinfo->frame_num.stage_c_frame_num > 0) {
		res_num = uinfo->frame_num.stage_c_frame_num;
		valid_cnt = uinfo->stage_a_valid_count;
	} else if (uinfo->stage_a_valid_count)
		return 0;

	if ((res_num / valid_cnt) >= 1)
		vid_valid_count = valid_cnt - 1;
	else
		vid_valid_count = res_num % valid_cnt;
	return vid_valid_count;
}

static int ispnode_hist_roi_update(struct cam_hw_info *hw, uint32_t cfg_id, struct isp_hw_fetch_info *fetch)
{
	int ret = 0;
	struct isp_dev_hist2_info hist2_info = {0};
	struct isp_hw_hist_roi hist_arg = {0};

	pr_debug("cfg_id %d, hist_roi w[%d] h[%d]\n",
		cfg_id, fetch->in_trim.size_x, fetch->in_trim.size_y);

	hist2_info.hist_roi.start_x = 0;
	hist2_info.hist_roi.start_y = 0;

	hist2_info.hist_roi.end_x = fetch->in_trim.size_x - 1;
	hist2_info.hist_roi.end_y = fetch->in_trim.size_y - 1;

	hist_arg.hist_roi = &hist2_info.hist_roi;
	hist_arg.ctx_id = cfg_id;
	hw->isp_ioctl(hw, ISP_HW_CFG_UPDATE_HIST_ROI, &hist_arg);

	return ret;
}

static int ispnode_start_proc(void *node)
{
	struct isp_node *inode = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;
	struct isp_hw_context *pctx_hw = NULL;
	uint32_t ret = 0, loop = 0, i = 0, hw_path_id = 0, slice_need = 0;
	struct isp_start_param start_param = {0};
	struct isp_port_cfg port_cfg = {0};
	struct isp_port *port = NULL;
	struct camera_frame *frame = NULL;

	inode = VOID_PTR_TO(node, struct isp_node);
	if (atomic_read(&inode->user_cnt) < 1) {
		pr_err("fail to init isp node\n");
		return EFAULT;
	}

	if (inode->is_fast_stop) {
		port_cfg.faststop = &inode->is_fast_stop;
		port_cfg.faststop_done = inode->fast_stop_done;
		port_cfg.param_share_queue = &inode->param_share_queue;
		port_cfg.out_buf_clear = 1;
		list_for_each_entry(port, &inode->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
				port->port_cfg_cb_func(&port_cfg, ISP_PORT_FAST_STOP, port);
		}
		return 0;
	}

	dev = inode->dev;
	hw = dev->isp_hw;
	slice_need = ispnode_slice_needed(inode);

	do {
		pctx_hw = (struct isp_hw_context *)dev->isp_ops->bind(inode, slice_need, ispnode_postproc_irq);
		if (!pctx_hw) {
			pr_info_ratelimited("ctx %d wait for hw. loop %d\n", inode->node_id, loop);
			usleep_range(600, 800);
		} else
			break;
	} while (loop++ < 5000);

	if (pctx_hw == NULL) {
		pr_err("fail to get pctx_hw");
		return EFAULT;
	}
	port_cfg.valid_out_frame = 0;
	port_cfg.hw_ctx_id = pctx_hw->hw_ctx_id;
	memcpy(&inode->pipe_src, &inode->uinfo, sizeof(inode->uinfo));
	ret = ispnode_port_param_cfg(inode, pctx_hw, &port_cfg);
	if (ret || !port_cfg.valid_out_frame) {
		pr_info("No available output buffer sw %d, hw %d\n", pctx_hw->node_id, pctx_hw->hw_ctx_id);
		goto exit;
	}
	if (inode->ch_id == CAM_CH_CAP || (port_cfg.src_frame && (port_cfg.src_frame->fid & 0x1f) == 0))
		pr_info(" inode type:%d node_id:%d, cfg_id %d, fid %d, ch_id %d, buf_fd %d iova 0x%08x\n",
			inode->node_type, inode->node_id, inode->cfg_id,
			port_cfg.src_frame->fid, port_cfg.src_frame->channel_id,
			port_cfg.src_frame->buf.mfd, port_cfg.src_frame->buf.iova);
	ispnode_hist_roi_update(inode->dev->isp_hw, inode->cfg_id, &pctx_hw->pipe_info.fetch);
	/*update NR param for crop/scaling image */
	ispnode_blkparam_adapt(inode);

	pctx_hw->isp_k_param = &inode->isp_k_param;
	pctx_hw->isp_using_param = inode->isp_using_param;
	pctx_hw->pyr_layer_num = inode->uinfo.pyr_layer_num;
	pctx_hw->wmode = dev->wmode;
	pctx_hw->valid_slc_num = 0;
	if (pctx_hw->fmcu_handle || slice_need) {
		ret = isp_hwctx_slice_ctx_init(pctx_hw, &pctx_hw->pipe_info);
		isp_hwctx_fmcu_reset(pctx_hw);
	}

	ispnode_3dnr_frame_process(inode, &pctx_hw->pipe_info.fetch, &port_cfg);
	ispnode_ltm_frame_process(inode, &port_cfg);
	ispnode_rec_frame_process(inode, pctx_hw, &port_cfg);
	ispnode_gtm_frame_process(inode, &port_cfg);
	if (pctx_hw->fmcu_handle || slice_need) {
		struct slice_cfg_input slc_cfg = {0};
		slc_cfg.pyr_rec_eb = port_cfg.src_frame->need_pyr_rec;
		slc_cfg.ltm_rgb_eb = inode->pipe_src.ltm_rgb;
		slc_cfg.rgb_ltm = (struct isp_ltm_ctx_desc *)inode->rgb_ltm_handle;
		slc_cfg.gtm_rgb_eb = inode->pipe_src.gtm_rgb;
		slc_cfg.rgb_gtm = (struct isp_gtm_ctx_desc *)inode->rgb_gtm_handle;
		slc_cfg.nr3_ctx = (struct isp_3dnr_ctx_desc *)inode->nr3_handle;
		pctx_hw->rec_handle = inode->rec_handle;
		pctx_hw->nr3_fbc_fbd = inode->pipe_src.nr3_fbc_fbd;
		isp_hwctx_slice_fmcu(pctx_hw, &slc_cfg);
	}

	if (inode->uinfo.enable_slowmotion) {
		uint32_t vid_valid_count = 0, i = 0;
		vid_valid_count = ispnode_slw_need_vid_num(&inode->uinfo);
		pr_debug("vid count %d, stage a frm_num %d, stage b frm_num %d, stage c frm_num %d, fps %d",
			vid_valid_count,inode->uinfo.frame_num.stage_a_frame_num,inode->uinfo.frame_num.stage_b_frame_num,
			inode->uinfo.frame_num.stage_c_frame_num, inode->uinfo.slowmotion_count);
		for (i = 0; i < inode->uinfo.slowmotion_count - 1; i++) {
			ret = ispnode_fmcu_slw_queue_set(pctx_hw->fmcu_handle, inode, pctx_hw, i < vid_valid_count);
			if (ret)
				pr_err("fail to set fmcu slw queue\n");
		}
	}
	ret = wait_for_completion_timeout(&inode->frm_done, ISP_CONTEXT_TIMEOUT);
	if (ret == 0) {
		pr_err("fail to wait isp node %d, timeout.\n", inode->node_id);
		ret = -EFAULT;
		goto exit;
	}

	loop = 0;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) > 0 && atomic_read(&port->is_work) > 0) {
			hw_path_id = isp_port_id_switch(port->port_id);
			if (port_cfg.need_post_proc[hw_path_id]) {
				do {
					frame = port_cfg.superzoom_frame;
					if (frame) {
						frame->state = port_cfg.src_frame->state;
						frame->in_fmt = CAM_YVU420_2FRAME;
						frame->in = port_cfg.src_frame->out[hw_path_id];
						frame->in_crop.size_x = port_cfg.src_frame->out[hw_path_id].w;
						frame->in_crop.size_y = port_cfg.src_frame->out[hw_path_id].h;
						frame->out_crop[hw_path_id].size_x = port_cfg.src_frame->out[hw_path_id].w;
						frame->out_crop[hw_path_id].size_y = port_cfg.src_frame->out[hw_path_id].h;
						frame->out[hw_path_id] = port->size;
						frame->link_from.node_type = inode->node_type;
						frame->link_from.port_id = port->port_id;
						frame->link_from.node_id = inode->node_id;
					}
					ret = cam_queue_enqueue(&port->result_queue, &frame->list);
					if (ret == 0)
						break;
					printk_ratelimited(KERN_INFO "wait for output queue. loop %d\n", loop);
					/* wait for previous frame output queue done */
					mdelay(1);
				} while (loop++ < 500);
			}
		}
	}

	pctx_hw->iommu_status = (uint32_t)(-1);
	pctx_hw->enable_slowmotion = inode->pipe_src.enable_slowmotion;
	pctx_hw->slowmotion_count = inode->pipe_src.slowmotion_count;
	start_param.blkpm_lock = &inode->blkpm_lock;
	if (!pctx_hw->fmcu_handle && slice_need) {
		if (port_cfg.in_fmt == CAM_YVU420_2FRAME)
			start_param.type = ISP_YUV_BLOCK_DISABLE;
		else
			start_param.type = ISP_YUV_BLOCK_CFG;
		start_param.isp_using_param = &inode->isp_using_param;
		start_param.param_share_queue = &inode->param_share_queue;
		start_param.src_frame = port_cfg.src_frame;
		ret = isp_hwctx_slices_proc(pctx_hw, inode->dev, &start_param);
	} else {
		if (port_cfg.in_fmt == IMG_PIX_FMT_NV21)
			start_param.type = ISP_YUV_BLOCK_DISABLE;
		else
			start_param.type = ISP_YUV_BLOCK_CFG;
		start_param.slw_state = &inode->pipe_src.slw_state;
		start_param.isp_using_param = &inode->isp_using_param;
		start_param.param_share_queue = &inode->param_share_queue;
		start_param.src_frame = port_cfg.src_frame;
		isp_hwctx_hw_start(pctx_hw, inode->dev, &start_param);
	}

	pr_debug("isp start done node:%x", inode);
	return 0;

exit:
	port_cfg.param_share_queue = &inode->param_share_queue;
	port_cfg.out_buf_clear = 0;
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (atomic_read(&port->user_cnt) > 0)
			port->port_cfg_cb_func(&port_cfg, ISP_PORT_START_ERROR, port);
	}

	if (inode->uinfo.enable_slowmotion) {
		for (i = 0; i < inode->uinfo.slowmotion_count - 1; i++) {
			port_cfg.param_share_queue = NULL;
			port_cfg.out_buf_clear = 1;
			list_for_each_entry(port, &inode->port_queue.head, list) {
				if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
					port->port_cfg_cb_func(&port_cfg, ISP_PORT_START_ERROR, port);
			}
		}
	}

	if (inode->is_fast_stop) {
		list_for_each_entry(port, &inode->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0) {
				if (cam_queue_cnt_get(&port->out_buf_queue) == 0 && cam_queue_cnt_get(&port->result_queue) == 0) {
					inode->is_fast_stop = 0;
					complete(inode->fast_stop_done);
				}
			}
		}
	}

	if (pctx_hw)
		dev->isp_ops->unbind(inode);
	inode->isp_using_param = NULL;
	pctx_hw->isp_using_param = NULL;

	return 0;
}

static int ispnode_blkparam_cfg(void *node, void *param)
{
	int ret = 0, i = 0;
	struct isp_node* inode = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_io_param *io_param = NULL;
	func_cam_cfg_param cfg_fun_ptr = NULL;
	struct cam_hw_info *ops = NULL;
	struct cam_hw_block_func_get fucarg = {0};
	struct isp_port_blksize_desc size_desc = {0};
	struct isp_port *port = NULL;

	inode = VOID_PTR_TO(node, struct isp_node);
	dev = inode->dev;
	ops = dev->isp_hw;
	if (!dev) {
		pr_info("fail to get dev");
		return -1;
	}
	if (!ops) {
		pr_info("fail to get ops");
		return -1;
	}
	io_param = VOID_PTR_TO(param, struct isp_io_param);

	mutex_lock(&dev->path_mutex);


	if (atomic_read(&inode->user_cnt) < 1) {
		pr_err("fail to use unable isp ctx %d.\n", inode->node_id);
		mutex_unlock(&dev->path_mutex);
		return -EINVAL;
	}

	if (inode->isp_receive_param == NULL && io_param->scene_id == PM_SCENE_PRE) {
		pr_warn("warning:not get recive handle, param out of range, isp%d blk %d\n", inode->cfg_id, io_param->sub_block);
		mutex_unlock(&dev->path_mutex);
		return 0;
	}
	/* lock to avoid block param across frame */
	mutex_lock(&inode->blkpm_lock);
	list_for_each_entry(port, &inode->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
			port->port_cfg_cb_func(&size_desc, ISP_PORT_BLKSIZE_GET, port);
	}
	inode->isp_k_param.src_w = size_desc.src_size.w;
	inode->isp_k_param.src_h = size_desc.src_size.h;

	if (io_param->sub_block == ISP_BLOCK_3DNR) {
		if (inode->uinfo.mode_3dnr != MODE_3DNR_OFF) {
			if (io_param->scene_id == PM_SCENE_PRE || io_param->scene_id == PM_SCENE_VID)
				ret = isp_k_cfg_3dnr(param, inode->isp_receive_param->blkparam_info.param_block);
			else
				ret = isp_k_cfg_3dnr(param, &inode->isp_k_param);
		}
	} else if (io_param->sub_block == ISP_BLOCK_NOISEFILTER) {
		if (io_param->scene_id == PM_SCENE_CAP)
			ret = isp_k_cfg_yuv_noisefilter(param, &inode->isp_k_param);
	} else {
		i = io_param->sub_block;
		fucarg.index = i;
		ops->cam_ioctl(ops, CAM_HW_GET_BLK_FUN, &fucarg);
		if (fucarg.cam_entry != NULL &&
			fucarg.cam_entry->sub_block == io_param->sub_block)
			cfg_fun_ptr = fucarg.cam_entry->cfg_func;

		if (cfg_fun_ptr == NULL) {
			pr_debug("isp block 0x%x is not supported.\n", io_param->sub_block);
			mutex_unlock(&inode->blkpm_lock);
			mutex_unlock(&dev->path_mutex);
			return 0;
		}
		if (io_param->scene_id == PM_SCENE_PRE)
			ret = cfg_fun_ptr(io_param, inode->isp_receive_param->blkparam_info.param_block);
		else
			ret = cfg_fun_ptr(io_param, &inode->isp_k_param);
	}

	mutex_unlock(&inode->blkpm_lock);
	mutex_unlock(&dev->path_mutex);

	return ret;
}

static uint32_t ispnode_insert_port(struct isp_node* node, void *param)
{
	struct isp_port *q_port = NULL;
	struct isp_port *new_port = NULL;
	uint32_t is_exist = 0;

	new_port = VOID_PTR_TO(param, struct isp_port);
	pr_debug("node type:%d,id:%d, port type:%d, port type:%d", node->node_type, node->node_id, new_port->type, new_port->port_id);

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

static int ispnode_param_buf_init(struct camera_frame *cfg_frame)
{
	int ret = 0;
	unsigned int iommu_enable = 0;
	size_t size = 0;

	/*alloc cfg context buffer*/
	if (cam_buf_iommu_status_get(CAM_IOMMUDEV_ISP) == 0) {
		pr_debug("isp iommu enable\n");
		iommu_enable = 1;
	} else {
		pr_debug("isp iommu disable\n");
		iommu_enable = 0;
	}
	size = sizeof(struct dcam_isp_k_block);
	ret = cam_buf_alloc(&cfg_frame->buf, size, 1);
	if (ret) {
		pr_err("fail to get cfg buffer\n");
		ret = -EFAULT;
	}

	return ret;
}

void isp_node_param_buf_destroy(void *param)
{
	struct camera_frame *frame = NULL;
	int ret = 0;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (frame->buf.addr_k) {
		ret = cam_buf_kunmap(&frame->buf);
		if(ret)
			pr_err("fail to unmap param node %px\n", frame);
	}
	ret = cam_buf_free(&frame->buf);
	if(ret)
		pr_err("fail to unmap param node %px\n", frame);
	cam_queue_empty_frame_put(frame);
	frame = NULL;
}

void isp_node_offline_pararm_free(void *param)
{
	struct isp_offline_param *cur, *prev;

	cur = (struct isp_offline_param *)param;
	while (cur) {
		prev = (struct isp_offline_param *)cur->prev;
		pr_info("free %p\n", cur);
		cam_buf_kernel_sys_vfree(cur);
		cur = prev;
	}
}

int isp_node_prepare_blk_param(struct isp_node *inode, uint32_t target_fid, struct blk_param_info *out)
{
	int ret = 0, loop = 0, param_last_fid = -1;
	uint32_t param_update = 0;
	struct camera_frame *param_pframe = NULL;
	struct camera_frame *last_param = NULL;

	if (out->param_block) {
		pr_info("already get param in dec process\n");
		return 0;
	}
	if (inode->ch_id == CAM_CH_CAP) {
		out->param_block = NULL;
		out->blk_param_node = NULL;
		out->update = 1;
		param_update = 1;
		goto capture_param;
	} else {
		out->param_block = &inode->isp_k_param;
		out->blk_param_node = NULL;
		goto preview_param;
	}

preview_param:
	do {
		mutex_lock(&inode->blkpm_q_lock);
		param_pframe = cam_queue_dequeue_peek(&inode->param_buf_queue, struct camera_frame, list);
		if (param_pframe) {
			pr_debug("inode =%d, param_pframe.id=%d,pframe.id=%d\n",
				inode->cfg_id, param_pframe->fid, target_fid);
			if (param_pframe->fid <= target_fid) {
				/*update old preview param into inode, cache latest capture param*/
				cam_queue_dequeue(&inode->param_buf_queue, struct camera_frame, list);
				mutex_unlock(&inode->blkpm_q_lock);
				if (param_pframe->blkparam_info.update) {
					pr_debug("isp%d update param %d\n", inode->cfg_id, param_pframe->fid);
					ispnode_copy_param(param_pframe, inode);
					param_update = 1;
				}
				param_last_fid = param_pframe->fid;
				cam_queue_recycle_blk_param(&inode->param_share_queue, param_pframe);
				if (param_last_fid == target_fid)
					break;
				param_pframe = NULL;
			} else {
				mutex_unlock(&inode->blkpm_q_lock);
				pr_warn("warning:param not match, isp%d, fid %d\n", inode->cfg_id, target_fid);
				if (param_last_fid != -1)
					pr_warn("use old param, param id %d, frame id %d\n", param_last_fid, target_fid);
				else
					pr_warn("no param update, frame id %d\n", target_fid);
				break;
			}
		} else
			mutex_unlock(&inode->blkpm_q_lock);
	} while (loop++ < inode->param_buf_queue.max);
	if (!out->param_block)
		out->param_block = &inode->isp_k_param;
	out->update = param_update;
	return param_update;

capture_param:
	do {
		mutex_lock(&inode->blkpm_q_lock);
		param_pframe = cam_queue_dequeue_peek(&inode->param_buf_queue, struct camera_frame, list);
		if (param_pframe) {
			pr_debug("inode =%d, param_pframe.id=%d,pframe.id=%d\n",
				inode->cfg_id,param_pframe->fid,target_fid);
			if (param_pframe->fid <= target_fid) {
				cam_queue_dequeue(&inode->param_buf_queue, struct camera_frame, list);
				mutex_unlock(&inode->blkpm_q_lock);
				/*return old param*/
				if (last_param)
					ret = cam_queue_recycle_blk_param(&inode->param_share_queue, last_param);
				/*cache latest capture param*/
				last_param = param_pframe;
				param_last_fid = param_pframe->fid;

				if (param_last_fid == target_fid)
					break;

				param_pframe = NULL;
			} else {
				mutex_unlock(&inode->blkpm_q_lock);
				pr_warn("warning:param not match, isp%d, fid %d\n", inode->cfg_id, target_fid);
				if (!last_param) {
					pr_warn("dont have old param, use latest param, frame %d\n", target_fid);
					out->param_block = &inode->isp_k_param;
					out->blk_param_node = NULL;
				}
				break;
			}
		} else
			mutex_unlock(&inode->blkpm_q_lock);
	} while (loop++ < inode->param_buf_queue.max);

	if (last_param) {
		out->param_block = last_param->blkparam_info.param_block;
		out->blk_param_node = last_param;
		if (param_last_fid != target_fid)
			pr_warn("use old param, param id %d, frame id %d\n", param_last_fid, target_fid);
	} else if (out->param_block == NULL) {
		pr_warn("isp%d not have param in q, use latest param, frame id %d\n", inode->cfg_id, target_fid);
		out->param_block = &inode->isp_k_param;
		out->blk_param_node = NULL;
	}
	out->update = param_update;
	return param_update;
}

uint32_t isp_node_config(void *node, enum isp_node_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_node *inode = NULL;
	struct isp_ltm_ctx_desc *rgb_ltm = NULL;
	struct isp_3dnr_ctx_desc *nr3_ctx = NULL;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct cam_hw_gtm_ltm_dis dis = {0};
	struct cam_hw_gtm_ltm_eb eb = {0};
	struct isp_hw_gtm_func *gtm_func = NULL;
	struct cfg_param_status *param_status = NULL;
	struct camera_frame *param_frame = NULL;
	struct camera_frame *frame = NULL;
	struct isp_port_cfg port_cfg = {0};
	struct isp_port *port = NULL;

	inode = VOID_PTR_TO(node, struct isp_node);
	switch (cmd) {
	case ISP_NODE_CFG_BLK_PATAM:
		ret = ispnode_blkparam_cfg(inode, param);
		break;
	case ISP_NODE_CFG_STATIS:
		if (atomic_read(&inode->user_cnt) == 0) {
			pr_info("isp ctx %d is not enable\n", inode->node_id);
			return 0;
		}
		ret = cam_statis_isp_buffer_cfg(inode->dev, inode, param);
		break;
	case ISP_NODE_CFG_LTM_BUF:
		rgb_ltm = (struct isp_ltm_ctx_desc *)inode->rgb_ltm_handle;
		if (!rgb_ltm)
			return 0;
		ret = rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_BUF, param);
		if (ret)
			pr_err("fail to set isp ctx %d rgb ltm buffers.\n", inode->node_id);
		break;
	case ISP_NODE_CFG_INSERT_PORT:
		ispnode_insert_port(inode, param);
		break;
	case ISP_NODE_CFG_CLEAR_PORT:
		break;
	case ISP_NODE_CFG_PORT_UFRAME:
		inode->uinfo.uframe_sync |= *(uint32_t *)param;
		break;
	case ISP_NODE_CFG_3DNR_MODE:
		inode->uinfo.mode_3dnr |= *(uint32_t *)param;
		break;
	case ISP_NODE_CFG_3DNR_BUF:
		nr3_ctx = (struct isp_3dnr_ctx_desc *)inode->nr3_handle;
		ret = nr3_ctx->ops.cfg_param(nr3_ctx, ISP_3DNR_CFG_BUF, param);
		if (ret) {
			pr_err("fail to set isp node %d nr3 buffers.\n", inode->node_id);
		}
		break;
	case ISP_NODE_CFG_REC_LEYER_NUM:
		inode->uinfo.pyr_layer_num = *(uint32_t *)param;
		rec_ctx = (struct isp_rec_ctx_desc *)inode->rec_handle;
		if (rec_ctx)
			rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_LAYER_NUM, &inode->uinfo.pyr_layer_num);
		break;
	case ISP_NODE_CFG_REC_BUF:
		rec_ctx = (struct isp_rec_ctx_desc *)inode->rec_handle;
		if (!rec_ctx)
			return 0;
		ret = rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_BUF, param);
		if (ret) {
			pr_err("fail to set isp node %d rec buffer.\n", inode->node_id);
		}
		break;
	case ISP_NODE_CFG_GTM:
		gtm_func = VOID_PTR_TO(param, struct isp_hw_gtm_func);
		inode->dev->isp_hw->isp_ioctl(inode->dev->isp_hw, ISP_HW_CFG_GTM_FUNC_GET, gtm_func);
		switch (gtm_func->index) {
		case ISP_K_GTM_LTM_DIS:
			dis.isp_idx = inode->cfg_id;
			gtm_func->k_blk_func(&dis);
			break;
		case ISP_K_GTM_STATUS_GET:
			gtm_func->cap_skip_num = gtm_func->k_blk_func(&inode->cfg_id);
			break;
		case ISP_K_GTM_LTM_EB:
			eb.isp_idx = inode->cfg_id;
			gtm_func->k_blk_func(&inode->cfg_id);
			break;
		default:;
		}
		break;
	case ISP_NODE_CFG_PARAM_SWITCH:
		param_status = VOID_PTR_TO(param, struct cfg_param_status);
		if (param_status->status == 0 && param_status->scene_id == PM_SCENE_PRE) {
			param_frame = cam_queue_empty_blk_param_get(&inode->param_share_queue);
			if (param_frame) {
				inode->isp_receive_param = param_frame;
			} else {
				pr_err("fail to recive param, scene %d fid %d\n", param_status->scene_id, param_status->frame_id);
				return -EFAULT;
			}
		}

		if (param_status->status) {
			if (param_status->scene_id == PM_SCENE_PRE) {
				param_frame = inode->isp_receive_param;
				if (!param_frame)
					return 0;
				/* give up current param and no longer receive new param after dcam_ctx unbind */
				if (!param_status->dcam_ctx_bind_state && param_status->frame_id > 0) {
					ret = cam_queue_recycle_blk_param(&inode->param_share_queue, param_frame);
					inode->isp_receive_param = NULL;
					return ret;
				}

				if (param_status->update)
					param_frame->blkparam_info.update = 1;
				else
					param_frame->blkparam_info.update = 0;
				param_frame->fid = param_status->frame_id;
				ret = cam_queue_enqueue(&inode->param_buf_queue, &param_frame->list);
				inode->isp_receive_param = NULL;
			} else {
				param_frame = cam_queue_empty_blk_param_get(&inode->param_share_queue);
				if (!param_frame) {
					mutex_lock(&inode->blkpm_q_lock);
					param_frame = cam_queue_dequeue(&inode->param_buf_queue, struct camera_frame, list);
					mutex_unlock(&inode->blkpm_q_lock);
					if (param_frame)
						pr_debug("isp deq param node %d\n", param_frame->fid);
				}

				if (param_frame) {
					memcpy(param_frame->blkparam_info.param_block, &inode->isp_k_param, sizeof(struct dcam_isp_k_block));
					if (param_status->update)
						param_frame->blkparam_info.update = 1;
					else
						param_frame->blkparam_info.update = 0;
					param_frame->fid = param_status->frame_id;
					ret = cam_queue_enqueue(&inode->param_buf_queue, &param_frame->list);
				} else
					pr_warn("isp%d cap lose param %d\n", inode->cfg_id, param_status->frame_id);
			}
		}
		break;
	case ISP_NODE_CFG_PARAM_Q_CLEAR:
		do {
			mutex_lock(&inode->blkpm_q_lock);
			frame = cam_queue_dequeue(&inode->param_buf_queue, struct camera_frame, list);
			mutex_unlock(&inode->blkpm_q_lock);
			if (frame)
				cam_queue_recycle_blk_param(&inode->param_share_queue, frame);
		} while (cam_queue_cnt_get(&inode->param_buf_queue) > 0);
		break;
	case ISP_NODE_CFG_FAST_STOP:
		inode->fast_stop_done = (struct completion *)param;
		port_cfg.faststop = &inode->is_fast_stop;
		port_cfg.faststop_done = inode->fast_stop_done;
		list_for_each_entry(port, &inode->port_queue.head, list) {
			if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0)
				port->port_cfg_cb_func(&port_cfg, ISP_PORT_FAST_STOP, port);
		}
		break;
	case ISP_NODE_CFG_SUPERZOOM_BUF:
		inode->postproc_buf = (struct camera_frame *)param;
		break;
	case ISP_NODE_CFG_RECYCLE_BLK_PARAM:
		cam_queue_blk_param_unbind(&inode->param_share_queue, param);
		break;
	default:
		pr_err("fail to support vaild cmd:%d", cmd);
		ret = -EFAULT;
		break;
	}
	return ret;
}

int isp_node_request_proc(struct isp_node *node, void *param)
{
	int ret = 0;
	struct camera_frame *pframe = NULL;
	struct isp_offline_param *in_param = NULL;
	struct isp_port *port = NULL;
	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}
	node = VOID_PTR_TO(node, struct isp_node);
	pframe = VOID_PTR_TO(param, struct camera_frame);
	in_param = (struct isp_offline_param *)pframe->param_data;
	if (in_param) {
		/*preview*/
		list_for_each_entry(port, &node->port_queue.head, list) {
			if (atomic_read(&port->user_cnt) >= 1)
				port->port_cfg_cb_func(pframe, ISP_PORT_SIZE_UPDATA, port);
		}
		isp_node_offline_pararm_free(in_param);
		pframe->param_data = NULL;
	}
	pr_debug("cam%d ctx %d, fid %d, ch_id %d, buf %d, 3dnr %d, w %d, h %d, pframe->is_reserved %d, compress_en %d\n",
		node->attach_cam_id, node->node_id, pframe->fid,
		pframe->channel_id, pframe->buf.mfd, node->uinfo.mode_3dnr,
		pframe->width, pframe->height, pframe->is_reserved, pframe->is_compressed);
	list_for_each_entry(port, &node->port_queue.head, list) {
		if (port->type == PORT_TRANSFER_IN && atomic_read(&port->user_cnt) > 0) {
			ret = isp_port_param_cfg(port, PORT_BUFFER_CFG_SET, pframe);
			if (ret) {
				pr_warn("warning: isp node proc in q may full\n");
				return ret;
			}
		}
	}

	ispnode_stream_state_get(node, pframe);
	if (node->uinfo.enable_slowmotion) {
		if (++(node->slw_frm_cnt) < node->uinfo.slowmotion_count)
			return ret;
	}
	if (atomic_read(&node->user_cnt) > 0)
		complete(&node->thread.thread_com);
	node->slw_frm_cnt = 0;
	return ret;
}

void *isp_node_get(uint32_t node_id, struct isp_node_desc *param)
{
	int ret = 0, i = 0, loop = 0;
	struct isp_node *node = NULL;
	struct cam_thread_info *thrd = NULL;
	struct isp_node_uinfo *uinfo = NULL;
	struct isp_ltm_ctx_desc *rgb_ltm = NULL;
	struct isp_gtm_ctx_desc *rgb_gtm = NULL;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct camera_frame *pframe_param = NULL;
	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	pr_info("node id %d node_dev %px\n", node_id, *param->node_dev);
	if (*param->node_dev == NULL) {
		node = cam_buf_kernel_sys_vzalloc(sizeof(struct isp_node));
		if (!node) {
			pr_err("fail to get valid isp node\n");
			return NULL;
		}
	} else {
		node = *param->node_dev;
		pr_info("isp offline node has been alloc %p\n", node);
		goto exit;
	}
	if (node->data_cb_func == NULL) {
		node->data_cb_func = param->data_cb_func;
		node->data_cb_handle = param->data_cb_handle;
	}
	node->dev = param->dev;
	ret = node->dev->isp_ops->ioctl(node->dev, ISP_IOCTL_INIT_NODE_HW, &node->cfg_id);
	mutex_init(&node->blkpm_lock);
	init_completion(&node->frm_done);
	node->isp_k_param.cfg_id = node->cfg_id;
	/* complete for first frame config */
	complete(&node->frm_done);
	init_isp_pm(&node->isp_k_param);

	node->attach_cam_id = param->cam_id;
	node->uinfo.enable_slowmotion = 0;
	cam_queue_init(&node->hist2_result_queue, ISP_HIST2_BUF_Q_LEN, cam_statis_isp_buf_destroy);
	cam_queue_init(&node->gtmhist_result_queue, ISP_GTMHIST_BUF_Q_LEN, cam_statis_isp_buf_destroy);

	cam_queue_init(&node->port_queue, PORT_ISP_MAX, NULL);
	if (param->is_high_fps)
		node->uinfo.enable_slowmotion = node->dev->isp_hw->ip_isp->slm_cfg_support;
	if (node->uinfo.enable_slowmotion) {
		node->slw_frm_cnt = 0;
		node->uinfo.enable_slowmotion = param->enable_slowmotion;
		node->uinfo.slowmotion_count = param->slowmotion_count;
	}
	pr_debug("cam%d isp slowmotion eb %d\n",node->attach_cam_id, node->uinfo.enable_slowmotion);

	node->resbuf_get_cb = param->resbuf_get_cb;
	node->resbuf_cb_data = param->resbuf_cb_data;
	node->is_bind = 0;
	uinfo = &node->uinfo;

	if (node->rgb_ltm_handle == NULL && node->dev->isp_hw->ip_isp->rgb_ltm_support) {
		node->rgb_ltm_handle = isp_ltm_rgb_ctx_get(node->cfg_id, node->attach_cam_id, node->dev->isp_hw);
		if (!node->rgb_ltm_handle) {
			pr_err("fail to get memory for ltm_rgb_ctx.\n");
			ret = -ENOMEM;
			goto rgb_ltm_err;
		}
		rgb_ltm = (struct isp_ltm_ctx_desc *)node->rgb_ltm_handle;
	}

	uinfo->mode_ltm = param->mode_ltm;
	uinfo->ltm_rgb = param->ltm_rgb;
	if (rgb_ltm) {
		rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_EB, &uinfo->ltm_rgb);
		rgb_ltm->ltm_ops.core_ops.cfg_param(rgb_ltm, ISP_LTM_CFG_MODE, &uinfo->mode_ltm);
		rgb_ltm->ltm_ops.sync_ops.set_status(rgb_ltm, 1);
	}

	if (node->rgb_gtm_handle == NULL && node->dev->isp_hw->ip_isp->rgb_gtm_support) {
		node->rgb_gtm_handle = ispgtm_rgb_ctx_get(node->cfg_id, node->attach_cam_id, node->dev->isp_hw);
		if (!node->rgb_gtm_handle) {
			pr_err("fail to get memory for gtm_rgb_ctx.\n");
			ret = -ENOMEM;
			goto rgb_gtm_err;
		}
		rgb_gtm = (struct isp_gtm_ctx_desc *)node->rgb_gtm_handle;
	}
	uinfo->gtm_rgb = param->gtm_rgb;
	uinfo->mode_gtm = param->mode_gtm;
	if (rgb_gtm) {
		rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_EB, &uinfo->gtm_rgb);
		rgb_gtm->gtm_ops.cfg_param(rgb_gtm, ISP_GTM_CFG_MODE, &uinfo->mode_gtm);
	}

	if (node->nr3_handle == NULL) {
		node->nr3_handle = isp_3dnr_ctx_get(node->cfg_id);
		if (!node->nr3_handle) {
			pr_err("fail to get memory for nr3_ctx.\n");
			ret = -ENOMEM;
			goto nr3_err;
		}
	}
	uinfo->fetch_path_sel = param->fetch_path_sel;
	uinfo->nr3_fbc_fbd = param->nr3_fbc_fbd;

	uinfo->mode_3dnr = param->mode_3dnr;
	node->nr3_blend_cnt = 0;

	if (node->rec_handle == NULL && node->dev->isp_hw->ip_isp->pyr_rec_support) {
		node->rec_handle = isp_pyr_rec_ctx_get(node->cfg_id, node->dev->isp_hw);
		if (!node->rec_handle) {
			pr_err("fail to get memory for rec_ctx.\n");
			ret = -ENOMEM;
			goto rec_err;
		}
		rec_ctx = (struct isp_rec_ctx_desc *)node->rec_handle;
	}

	uinfo->is_dewarping = 0;
	if (rec_ctx)
		rec_ctx->ops.cfg_param(rec_ctx, ISP_REC_CFG_DEWARPING_EB, &uinfo->is_dewarping);

	/* create isp offline thread */
	thrd = &node->thread;
	thrd->data_cb_func = node->data_cb_func;
	thrd->data_cb_handle = node->data_cb_handle;
	thrd->error_type = CAM_CB_ISP_DEV_ERR;
	sprintf(thrd->thread_name, "isp_node%d_offline", node_id);
	ret = camthread_create(node, thrd, ispnode_start_proc);
	if (ret) {
		pr_err("fail to get isp node start proc.\n");
		ret = -ENOMEM;
		goto thread_err;
	}

	thrd = &node->isp_interrupt_thread;
	thrd->data_cb_func = node->data_cb_func;
	thrd->data_cb_handle = node->data_cb_handle;
	thrd->error_type = CAM_CB_ISP_DEV_ERR;
	sprintf(thrd->thread_name, "isp_node%d_interrupt", node_id);
	ret = camthread_create(node, thrd, isp_int_interruption_proc);
	if (ret) {
		pr_err("fail to get isp node interruption_proc thread.\n");
		ret = -ENOMEM;
		goto thread_err;
	}
	mutex_init(&node->blkpm_q_lock);
	cam_queue_init(&node->param_share_queue, param->blkparam_node_num, isp_node_param_buf_destroy);
	cam_queue_init(&node->param_buf_queue, param->blkparam_node_num, isp_node_param_buf_destroy);
	cam_queue_init(&node->isp_interrupt_queue, ISP_IRQ_Q_LEN, cam_queue_empty_interrupt_put);
	for (i = 0; i < param->blkparam_node_num; i++) {
		pframe_param = cam_queue_empty_frame_get();
		ret = ispnode_param_buf_init(pframe_param);
		if (ret) {
			pr_err("fail to alloc memory.\n");
			cam_queue_empty_frame_put(pframe_param);
			ret = -ENOMEM;
			goto map_error;
		}

		ret = cam_queue_recycle_blk_param(&node->param_share_queue, pframe_param);
		if (ret) {
			pr_err("fail to enqueue shared buf: %d ch %d\n", i, node->ch_id);
			ret = -ENOMEM;
			goto map_error;
		}
	}

	isp_int_isp_irq_sw_cnt_reset(node->cfg_id);
	node->ch_id = param->ch_id;
	node->node_id = node_id;
	node->node_type = param->node_type;
	*param->node_dev = node;
	uinfo->slw_state = param->slw_state;

	pr_info("node id %d cur_ctx_id %d cfg_id %d\n", node_id, node->pctx_hw_id, node->cfg_id);

exit:
	if (param->is_high_fps) {
		node->uinfo.slw_960desc = param->fps960_info;
		if (node->uinfo.slw_960desc.slowmotion_stage_a_valid_num) {
			node->uinfo.frame_num.stage_a_frame_num = node->uinfo.slw_960desc.slowmotion_stage_a_num;
			node->uinfo.stage_a_valid_count = node->uinfo.slw_960desc.slowmotion_stage_a_valid_num;
			node->uinfo.frame_num.stage_b_frame_num = node->uinfo.slw_960desc.slowmotion_stage_b_num;
			node->uinfo.frame_num.stage_c_frame_num = node->uinfo.slw_960desc.slowmotion_stage_a_num;
		}
	}
	atomic_inc(&node->user_cnt);
	param->isp_node = node;
	return node;
map_error:
	loop = cam_queue_cnt_get(&node->param_share_queue);
	do {
		pframe_param = cam_queue_dequeue(&node->param_share_queue, struct camera_frame, list);
		if (pframe_param) {
			cam_buf_free(&pframe_param->buf);
			cam_queue_empty_frame_put(pframe_param);
		}
	} while (loop-- > 0);
thread_err:
	if (node->rec_handle && node->dev->isp_hw->ip_isp->pyr_rec_support) {
		isp_pyr_rec_ctx_put(node->rec_handle);
		node->rec_handle = NULL;
	}
rec_err:
	if (node->nr3_handle) {
		isp_3dnr_ctx_put(node->nr3_handle);
		node->nr3_handle = NULL;
	}
nr3_err:
	if (node->rgb_gtm_handle && node->dev->isp_hw->ip_isp->rgb_gtm_support) {
		isp_gtm_rgb_ctx_put(node->rgb_gtm_handle);
		node->rgb_gtm_handle = NULL;
	}
rgb_gtm_err:
	if (node->rgb_ltm_handle) {
		isp_ltm_rgb_ctx_put(node->rgb_ltm_handle);
		node->rgb_ltm_handle = NULL;
	}
rgb_ltm_err:
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
	return NULL;
}

void isp_node_put(struct isp_node *node)
{
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	if (atomic_dec_return(&node->user_cnt) == 0) {
		cam_queue_clear(&node->hist2_result_queue, struct camera_frame, list);

		if (node->rgb_gtm_handle && node->dev->isp_hw->ip_isp->rgb_gtm_support) {
			isp_gtm_rgb_ctx_put(node->rgb_gtm_handle);
			node->rgb_gtm_handle = NULL;
		}

		if (node->rgb_ltm_handle) {
			struct isp_ltm_ctx_desc *rgb_ltm = (struct isp_ltm_ctx_desc*)node->rgb_ltm_handle;
			rgb_ltm->ltm_ops.sync_ops.set_status(rgb_ltm, 0);
			isp_ltm_rgb_ctx_put(node->rgb_ltm_handle);
			node->rgb_ltm_handle = NULL;
		}
		if (node->nr3_handle) {
			isp_3dnr_ctx_put(node->nr3_handle);
			node->nr3_handle = NULL;
		}
		if (node->rec_handle && node->dev->isp_hw->ip_isp->pyr_rec_support) {
			isp_pyr_rec_ctx_put(node->rec_handle);
			node->rec_handle = NULL;
		}

		if (node->postproc_buf) {
			if (node->postproc_buf->buf.mapping_state & CAM_BUF_MAPPING_DEV)
				cam_buf_iommu_unmap(&node->postproc_buf->buf);
			node->postproc_buf = NULL;
			pr_debug("cfg_id %d, postproc out buffer unmap\n", node->cfg_id);
		}

		if (node->isp_receive_param) {
			cam_queue_recycle_blk_param(&node->param_share_queue, node->isp_receive_param);
			node->isp_receive_param = NULL;
		}
		pr_debug("share %d, buf %d\n", node->param_share_queue.cnt, node->param_buf_queue.cnt);
		mutex_lock(&node->blkpm_q_lock);
		cam_queue_clear(&node->param_share_queue, struct camera_frame, list);
		cam_queue_clear(&node->param_buf_queue, struct camera_frame, list);
		mutex_unlock(&node->blkpm_q_lock);

		cam_queue_clear(&node->hist2_result_queue, struct camera_frame, list);
		cam_queue_clear(&node->gtmhist_result_queue, struct camera_frame, list);
		cam_queue_clear(&node->isp_interrupt_queue, struct camera_interrupt, list);

		node->data_cb_func = NULL;
		isp_int_isp_irq_sw_cnt_trace(node->cfg_id);
		cam_statis_isp_buffer_unmap(node->dev, node);
		atomic_dec(&((struct isp_cfg_ctx_desc *)(node->dev->cfg_handle))->node_cnt);

		pr_info("isp offline node %d put success\n", node->node_id);
		cam_buf_kernel_sys_vfree(node);
		node = NULL;
	}
}

void isp_node_close(struct isp_node *node)
{
	int ret = 0, loop = 0;
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}
	if (node->thread.thread_task) {
		camthread_stop(&node->thread);
		/* wait for last frame done */
		ret = wait_for_completion_timeout(
			&node->frm_done, ISP_CONTEXT_TIMEOUT);
		if (ret == 0)
			pr_err("fail to wait node %d, timeout.\n", node->node_id);
		else
			pr_info("wait time %d ms\n", ret);
		camthread_stop(&node->isp_interrupt_thread);
		node->dev->isp_ops->unbind(node);
		/* make sure irq handler exit to avoid crash */
		while (node->in_irq_postproc && loop < 1000) {
			pr_debug("node %d in irq. wait %d", node->node_id, loop);
			loop++;
			udelay(500);
		};
		if (loop == 1000)
			pr_warn("warning: isp node close wait irq timeout\n");
		cam_queue_clear(&node->port_queue, struct isp_port, list);
	}
}
