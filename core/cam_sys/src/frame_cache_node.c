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

#include "cam_pipeline.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "FRAME_CACHE_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define FRAME_CACHE_BUF_NUM              50

enum frame_cache_type {
	FRAME_CACHE_NORMAL_TYPE,
	FRAME_CACHE_DUAL_SYNC_TYPE,
	FRAME_CACHE_MAX_TYPE,
};

static void framecache_frame_put(void *param)
{
	struct cam_frame *frame = NULL;
	struct frame_cache_node *node = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct cam_frame *)param;
	node = (struct frame_cache_node *)frame->common.priv_data;
	if (node && node->is_share_buf) {
		node->data_cb_func(CAM_CB_FRAME_CACHE_CLEAR_BUF, frame, node->data_cb_handle);
	} else
		cam_queue_empty_frame_put(frame);
}

static struct cam_frame *framecache_capframe_get(struct frame_cache_node *node,
		struct cam_frame *pframe)
{
	struct cam_frame *pftmp = NULL;

	pframe->common.priv_data = node;
	cam_queue_enqueue(&node->cache_buf_queue, &pframe->list);
	if (node->cache_buf_queue.cnt > node->cache_real_num && node->cache_real_num) {
		if (node->pre_raw_flag)
			pframe = cam_queue_dequeue_tail(&node->cache_buf_queue, struct cam_frame, list);
		else
			pframe = cam_queue_dequeue(&node->cache_buf_queue, struct cam_frame, list);
		node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pframe, node->data_cb_handle);
	}

	do {
		pftmp = cam_queue_dequeue(&node->cache_buf_queue, struct cam_frame, list);
		if (!pftmp)
			return NULL;
		if (node->opt_frame_fid) {
			if (node->pre_raw_flag == PRE_RAW_CACHE) {
				node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pftmp, node->data_cb_handle);
				return 0;
			}
			if (node->opt_frame_fid != pftmp->common.fid) {
				if (atomic_read(&node->opt_frame_done) == 0 && node->opt_frame_fid < pftmp->common.fid) {
					node->pre_raw_flag = PRE_RAW_CACHE;
					atomic_set(&node->opt_frame_done, 1);
					node->cap_frame_status(node->pre_raw_flag, node->dual_sync_cb_data);
					break;
				} else
					node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pftmp, node->data_cb_handle);
			} else {
				node->pre_raw_flag = PRE_RAW_CACHE;
				node->cap_frame_status(node->pre_raw_flag, node->dual_sync_cb_data);
				break;
			}
		} else {
			if (pftmp->common.boot_sensor_time < node->cap_param.cap_timestamp) {
				node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pftmp, node->data_cb_handle);
			} else {
				break;
			}
		}
	} while (pftmp);

	return pftmp;
}

static int framecache_normal_proc(struct frame_cache_node *node,
		struct cam_frame *pframe)
{
	int ret = 0;

	pr_debug("frame id %d cache_skip_num %d cache_num %d buf cnt %d\n", pframe->common.fid,
			node->cache_skip_num, node->cache_real_num, node->cache_buf_queue.cnt);
	if (node->cap_param.cap_type == DCAM_CAPTURE_STOP) {
		if (node->pre_raw_flag == PRE_RAW_DEAL) {
			node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pframe, node->data_cb_handle);
			return 0;
		}
		if (node->cur_cache_skip_num == node->cache_skip_num) {
			if (node->cur_cache_skip_num)
				node->cur_cache_skip_num --;
			else
				node->cur_cache_skip_num = node->cache_skip_num;
			pframe->common.priv_data = node;
			ret = cam_queue_enqueue(&node->cache_buf_queue, &pframe->list);
			if (node->cache_buf_queue.cnt > node->cache_real_num)
				pframe = cam_queue_dequeue(&node->cache_buf_queue, struct cam_frame, list);
			else
				return ret;
		} else {
			if (node->cur_cache_skip_num)
				node->cur_cache_skip_num --;
			else
				node->cur_cache_skip_num = node->cache_skip_num;
		}

		node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pframe, node->data_cb_handle);
	} else {
		pframe = framecache_capframe_get(node, pframe);
		if (!pframe)
			return 0;
		node->link_info = pframe->common.link_from;
		pframe->common.link_from.node_type = CAM_NODE_TYPE_FRAME_CACHE;
		pframe->common.link_from.port_id = PORT_FRAME_CACHE_OUT;
		node->data_cb_func(CAM_CB_FRAME_CACHE_DATA_DONE, pframe, node->data_cb_handle);
	}

	return ret;
}

static int framecache_dual_sync_proc(struct frame_cache_node *node,
		struct cam_frame *pframe)
{
	int ret = 0, current_frame_deal = 0;

	pr_debug("node %px frame id %d cache_num %d buf cnt %d cap cnt %d\n", node, pframe->common.fid,
			node->cache_real_num, node->cache_buf_queue.cnt);
	if (node->cap_param.cap_type == DCAM_CAPTURE_STOP) {
		if (node->dual_slave_frame_set) {
			ret = node->dual_slave_frame_set(pframe, node->dual_sync_cb_data);
			if (ret)
				return 0;
		}
	} else {
		/* callback to camcore find the dual sync frame */
		if (node->dual_sync_func) {
			pframe = node->dual_sync_func(pframe, node->dual_sync_cb_data, &current_frame_deal);
			if (current_frame_deal) {
				pr_debug("sync capture frame fid[%d], frame w %d, h %d s_time %lld c_time %lld\n",
					pframe->common.fid, pframe->common.width, pframe->common.height, pframe->common.boot_sensor_time, node->cap_param.cap_timestamp);

				node->link_info = pframe->common.link_from;
				pframe->common.link_from.node_type = CAM_NODE_TYPE_FRAME_CACHE;
				pframe->common.link_from.port_id = PORT_FRAME_CACHE_OUT;
				node->data_cb_func(CAM_CB_FRAME_CACHE_DATA_DONE, pframe, node->data_cb_handle);

				return ret;
			}
		}
	}

	pframe->common.priv_data = node;
	ret = cam_queue_enqueue(&node->cache_buf_queue, &pframe->list);
	if (node->cache_buf_queue.cnt > node->cache_real_num) {
		pframe = cam_queue_dequeue(&node->cache_buf_queue, struct cam_frame, list);
		node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pframe, node->data_cb_handle);
	}

	return ret;
}

int frame_cache_node_set_pre_raw_flag(void *handle, void *param)
{
	int ret = 0;
	struct frame_cache_node *node = NULL;
	uint32_t flag = 0;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	flag = *(uint32_t *)param;
	node = (struct frame_cache_node *)handle;
	node->pre_raw_flag = flag;

	return ret;
}

int frame_cache_node_get_cap_frame(void *handle, void *param)
{
	int ret = 0;
	struct frame_cache_node *node = NULL;
	uint32_t opt_frame_fid = 0;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	opt_frame_fid = *(uint32_t *)param;
	node = (struct frame_cache_node *)handle;
	node->opt_frame_fid = opt_frame_fid;
	atomic_set(&node->opt_frame_done, 0);

	return ret;
}

int frame_cache_outbuf_back(void *handle, void *param)
{
	int ret = 0;
	struct frame_cache_node *node = NULL;
	struct cam_frame *pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	node = (struct frame_cache_node *)handle;
	pframe = (struct cam_frame *)param;

	pframe->common.link_from = node->link_info;
	node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, pframe, node->data_cb_handle);

	return ret;
}

int frame_cache_cfg_param(void *handle, uint32_t cmd, void *param)
{
	int ret = 0;
	struct frame_cache_node *node = NULL;
	struct cam_capture_param *cap_param = NULL;
	struct cam_frame *frame = NULL;
	struct camera_queue **dual_sync_q = NULL;

	if (!handle) {
		pr_err("fail to get valid input ptr %px\n", handle);
		return -EFAULT;
	}

	node = (struct frame_cache_node *)handle;
	switch (cmd) {
	case CAM_NODE_CFG_CAP_PARAM:
		cap_param = (struct cam_capture_param *)param;
		node->cap_param.cap_type = cap_param->cap_type;
		node->cap_param.cap_cnt = cap_param->cap_cnt;
		node->cap_param.cap_timestamp = cap_param->cap_timestamp;
		node->cap_param.cap_user_crop = cap_param->cap_user_crop;
		pr_info("cap type %d, cnt %d, time %lld\n", node->cap_param.cap_type,
			atomic_read(&node->cap_param.cap_cnt), node->cap_param.cap_timestamp);
		break;
	case CAM_NODE_CLR_CACHE_BUF:
		do {
			frame = cam_queue_dequeue(&node->cache_buf_queue, struct cam_frame, list);
			if (frame == NULL)
				break;
			node->data_cb_func(CAM_CB_FRAME_CACHE_RET_SRC_BUF, frame, node->data_cb_handle);
		} while (frame);
		break;
	case CAM_NODE_DUAL_SYNC_BUF_GET:
		dual_sync_q = (struct camera_queue **)param;
		*dual_sync_q = &node->cache_buf_queue;
		break;
	default:
		pr_err("fail to support vaild cmd %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

int frame_cache_node_request_proc(struct frame_cache_node *node, void *param)
{
	int ret = 0;
	enum frame_cache_type type = FRAME_CACHE_NORMAL_TYPE;
	struct cam_frame *pframe = NULL;
	struct cam_pipeline *pipeline = NULL;
	struct cam_node *cam_node = NULL;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	pframe = (struct cam_frame *)param;
	cam_node = (struct cam_node *)node->data_cb_handle;
	pipeline = (struct cam_pipeline *)cam_node->data_cb_handle;

	if (pipeline->debug_log_switch)
		pr_info("pipeline_type %s, fid %d, ch_id %d, buf %x, w %d, h %d, pframe->common.is_reserved %d, compress_en %d\n",
			pipeline->pipeline_graph->name, pframe->common.fid, pframe->common.channel_id, pframe->common.buf.mfd,
			pframe->common.width, pframe->common.height, pframe->common.is_reserved, pframe->common.is_compressed);

	if (node->need_dual_sync)
		type = FRAME_CACHE_DUAL_SYNC_TYPE;

	switch (type) {
	case FRAME_CACHE_NORMAL_TYPE:
		ret = framecache_normal_proc(node, pframe);
		break;
	case FRAME_CACHE_DUAL_SYNC_TYPE:
		ret = framecache_dual_sync_proc(node, pframe);
		break;
	default :
		pr_err("fail to support cache type %d\n", type);
		ret = -EFAULT;
		break;
	}

	return ret;
}

void *frame_cache_node_get(uint32_t node_id, struct frame_cache_node_desc *param)
{
	struct frame_cache_node *node = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	node = cam_buf_kernel_sys_vzalloc(sizeof(struct frame_cache_node));
	if (!node) {
		pr_err("fail to get valid frame cache node\n");
		return NULL;
	}

	node->node_type = param->node_type;
	node->cache_real_num = param->cache_real_num;
	node->cache_skip_num = param->cache_skip_num;
	node->cur_cache_skip_num = param->cache_skip_num;
	node->is_share_buf = param->is_share_buf;
	node->need_dual_sync = param->need_dual_sync;
	node->buf_manager_handle = param->buf_manager_handle;
	node->cap_frame_status = param->cap_frame_status;
	node->pre_raw_flag = PRE_RAW_CACHE;
	node->opt_frame_fid = 0;
	if (node->data_cb_func == NULL) {
		node->data_cb_func = param->data_cb_func;
		node->data_cb_handle = param->data_cb_handle;
	}
	if (node->dual_sync_func == NULL) {
		node->dual_sync_func = param->dual_sync_func;
		node->dual_sync_cb_data = param->dual_sync_cb_data;
		node->dual_slave_frame_set = param->dual_slave_frame_set;
	}
	cam_queue_init(&node->cache_buf_queue, FRAME_CACHE_BUF_NUM, framecache_frame_put);
	pr_debug("cache real num %d\n", node->cache_real_num);

	return node;
}

void frame_cache_node_put(struct frame_cache_node *node)
{
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	cam_queue_clear(&node->cache_buf_queue, struct cam_frame, list);
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
}
