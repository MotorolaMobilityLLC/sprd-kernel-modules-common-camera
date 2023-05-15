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

#include "cam_copy_node.h"
#include "cam_pipeline.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "CAM_COPY_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

int cam_copy_node_buffer_cfg(void *handle, void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct cam_frame *pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	node = (struct cam_copy_node *)handle;
	node->copy_flag = CAM_ENABLE;
	pframe = (struct cam_frame *)param;
	ret = cam_buf_ionbuf_get(&pframe->common.buf);
	if (ret) {
		pr_err("fail to get ion copy buffer\n");
		return ret;
	}

	ret = CAM_QUEUE_ENQUEUE(&node->out_queue, &pframe->list);
	if (ret)
		pr_err("fail to enqueue copy buffer\n");

	return ret;
}

int cam_copy_node_buffers_alloc(void *handle, struct cam_buf_alloc_desc *param)
{
	int ret = 0;
	uint32_t i = 0, total = 0, size = 0, width = 0, height = 0, ch_id = 0;
	struct cam_copy_node *node = NULL;
	struct cam_frame *pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);;
		return -1;
	}

	node = (struct cam_copy_node *)handle;

	total = param->cam_copy_buf_alloc_num;
	if (total == 0)
		return ret;
	height = param->height;
	width = param->width;
	ch_id = param->ch_id;
	size = cal_sprd_size(width, height, CAM_RAW_PACK_10);

	pr_info("port:%s, cam%d, ch_id %d, buffer size: %u (%u x %u), num %d\n",
		cam_port_name_get(node->node_id), param->cam_idx, ch_id, size, width, height, total);

	for (i = 0; i < total; i++) {

		pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
		pframe->common.channel_id = param->ch_id;
		pframe->common.is_compressed = param->compress_en;
		pframe->common.width = width;
		pframe->common.height = height;
		ret = cam_buf_alloc(&pframe->common.buf, size, param->iommu_enable);
		if (ret) {
			pr_err("fail to alloc buf: %d ch %d\n",
				i, param->ch_id);
			cam_queue_empty_frame_put(pframe);
			continue;
		}

		cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_GET_IOVA, CAM_BUF_IOMMUDEV_DCAM);
		pframe->common.buf.bypass_iova_ops = CAM_ENABLE;
		ret = CAM_QUEUE_ENQUEUE(&node->out_queue, &pframe->list);
	}

	return ret;
}

int cam_copy_outbuf_back(void *handle, void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct cam_frame *pframe = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	node = (struct cam_copy_node *)handle;
	pframe = (struct cam_frame *)param;

	ret = CAM_QUEUE_ENQUEUE(&node->out_queue, &pframe->list);

	return ret;
}

int cam_copy_cfg_param(void *handle, void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct cam_capture_param *cap_param = NULL;

	if (!handle) {
		pr_err("fail to get valid input ptr %px\n", handle);
		return -EFAULT;
	}

	node = (struct cam_copy_node *)handle;
	cap_param = (struct cam_capture_param *)param;

	node->cap_param.cap_type = cap_param->cap_type;
	node->cap_param.cap_cnt = cap_param->cap_cnt;
	node->cap_param.cap_timestamp = cap_param->cap_timestamp;
	node->cap_param.cap_user_crop = cap_param->cap_user_crop;
	node->cap_param.zsl_num = cap_param->zsl_num;
	node->cap_param.frm_sel_mode = cap_param->frm_sel_mode;
	pr_info("cap type %d, cnt %d, time %lld, frm_sel_mode:%d, zsl_num:%d\n", node->cap_param.cap_type,
		atomic_read(&node->cap_param.cap_cnt), node->cap_param.cap_timestamp, cap_param->frm_sel_mode,
		cap_param->zsl_num);

	return ret;
}

int cam_copy_node_set_icap_scene(void *handle, void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	uint32_t icap_buffer_num = 0;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	icap_buffer_num = *(uint32_t *)param;
	node = (struct cam_copy_node *)handle;
	node->copy_flag = CAM_ENABLE;
	node->cache_num = icap_buffer_num ;
	node->scene_id = CAM_COPY_ICAP_SCENE;

	return ret;
}


int cam_copy_node_set_pre_raw_flag(void *handle, void *param)
{
	int ret = 0;
	uint32_t pre_raw_flag = 0;
	struct cam_copy_node *node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	pre_raw_flag = *(uint32_t *)param;
	node = (struct cam_copy_node *)handle;
	node->pre_raw_flag = pre_raw_flag;
	atomic_set(&node->opt_frame_done, 0);

	return ret;
}

int cam_copy_node_set_opt_scene(void *handle, void *param)
{
	int ret = 0;
	uint32_t opt_buffer_num = 0;
	struct cam_copy_node *node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	opt_buffer_num = *(uint32_t *)param;
	node = (struct cam_copy_node *)handle;
	node->copy_flag = CAM_ENABLE;
	node->opt_buffer_num = opt_buffer_num;
	node->scene_id = CAM_COPY_OPT_SCENE;

	return ret;
}

static int camcopy_node_copy_frame(struct cam_copy_node *node, int loop_num)
{
	int i = 0, ret = 0;
	struct cam_frame *pframe = NULL, *raw_frame = NULL;

	for (i = 0; i < loop_num; i++) {
		pframe = CAM_QUEUE_DEQUEUE(&node->in_queue, struct cam_frame, list);
		if (pframe == NULL) {
			pr_err("fail to get input frame for dump node %d\n", node->node_id);
			goto get_pframe_fail;
		}
		raw_frame = CAM_QUEUE_DEQUEUE(&node->out_queue, struct cam_frame, list);
		if (raw_frame == NULL) {
			pr_debug("raw path no get out queue\n");
			pframe->common.copy_en = 0;
			switch (node->scene_id) {
			case CAM_COPY_NORMAL_SCENE:
			case CAM_COPY_OPT_SCENE:
				node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
				break;
			case CAM_COPY_ICAP_SCENE:
				node->copy_cb_func(CAM_CB_ICAP_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
				break;
			default :
				pr_err("fail to support scene id %d\n", node->scene_id);
				return -EFAULT;
			}
		} else {
			pr_debug("start copy src frame data\n");
			if (pframe->common.buf.size > raw_frame->common.buf.size) {
				pr_err("fail to raw buff is small, frame buf size %d and raw buf size %d\n",
					pframe->common.buf.size, raw_frame->common.buf.size);
				goto raw_buffer_smaller_fail;
			}
			if (cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_GET_K_ADDR, CAM_BUF_IOMMUDEV_MAX)) {
				pr_err("fail to kmap temp buf\n");
				goto pframe_kmap_fail;
			}
			if (cam_buf_manager_buf_status_cfg(&raw_frame->common.buf, CAM_BUF_STATUS_GET_K_ADDR, CAM_BUF_IOMMUDEV_MAX)) {
				goto raw_frame_kmap_fail;
			}
			memcpy((char *)raw_frame->common.buf.addr_k, (char *)pframe->common.buf.addr_k, pframe->common.buf.size);
			/* use SOF time instead of ISP time for better accuracy */
			raw_frame->common.width = pframe->common.width;
			raw_frame->common.height = pframe->common.height;
			raw_frame->common.fid = pframe->common.fid;
			raw_frame->common.sensor_time.tv_sec = pframe->common.sensor_time.tv_sec;
			raw_frame->common.sensor_time.tv_usec = pframe->common.sensor_time.tv_usec;
			raw_frame->common.boot_sensor_time = pframe->common.boot_sensor_time;
			/* end copy, out src buf*/
			cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_PUT_K_ADDR, CAM_BUF_IOMMUDEV_MAX);
			cam_buf_manager_buf_status_cfg(&raw_frame->common.buf, CAM_BUF_STATUS_PUT_K_ADDR, CAM_BUF_IOMMUDEV_MAX);
			pframe->common.copy_en = 0;
			switch (node->scene_id) {
			case CAM_COPY_NORMAL_SCENE:
			case CAM_COPY_OPT_SCENE:
				node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
				raw_frame->common.evt = IMG_TX_DONE;
				raw_frame->common.irq_type = CAMERA_IRQ_IMG;
				raw_frame->common.priv_data = node;
				raw_frame->common.channel_id = CAM_CH_RAW;
				raw_frame->common.link_to.node_type = CAM_NODE_TYPE_USER;
				raw_frame->common.link_to.node_id = CAM_LINK_DEFAULT_NODE_ID;
				pr_debug("get out raw frame fd 0x%x raw fram fid %d\n", raw_frame->common.buf.mfd, raw_frame->common.fid);
				node->copy_cb_func(CAM_CB_DCAM_DATA_DONE, raw_frame, node->copy_cb_handle);
				break;
			case CAM_COPY_ICAP_SCENE:
				pr_debug("get out pframe fd 0x%x pframe fid %d\n", pframe->common.buf.mfd, pframe->common.fid);
				node->copy_cb_func(CAM_CB_ICAP_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
				raw_frame->common.link_from.node_type = CAM_NODE_TYPE_DATA_COPY;
				raw_frame->common.link_from.port_id = PORT_COPY_OUT;
				if (raw_frame->common.boot_sensor_time >= node->cap_param.cap_timestamp && atomic_read(&node->cap_param.cap_cnt) > 0) {
					ret = node->copy_cb_func(CAM_CB_DCAM_DATA_DONE, raw_frame, node->copy_cb_handle);
					atomic_dec(&node->cap_param.cap_cnt);
				} else {
					ret = CAM_QUEUE_ENQUEUE(&node->out_queue, &raw_frame->list);
				}
				break;
			default :
				pr_err("fail to support scene id %d\n", node->scene_id);
				return -EFAULT;
			}
		}
	}

	return ret;

raw_frame_kmap_fail:
	cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_PUT_K_ADDR, CAM_BUF_IOMMUDEV_MAX);
pframe_kmap_fail:
raw_buffer_smaller_fail:
	pframe->common.copy_en = 0;
	node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
	CAM_QUEUE_ENQUEUE(&node->out_queue, &raw_frame->list);
get_pframe_fail:

	return -EFAULT;
}

static int camcopy_node_frame_start(void *param)
{
	int ret = 0, loop_num = 1;
	struct cam_copy_node *node = NULL;
	struct cam_frame *pframe = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}
	node = (struct cam_copy_node *)param;

	switch (node->scene_id) {
	case CAM_COPY_NORMAL_SCENE:
		break;
	case CAM_COPY_OPT_SCENE:
		if (node->record_channel_id == CAM_CH_PRE && node->opt_buffer_num) {
			if (node->pre_raw_flag == PRE_RAW_CACHE) {
				if (node->in_queue.cnt > node->opt_buffer_num) {
					pframe = CAM_QUEUE_DEQUEUE(&node->in_queue, struct cam_frame, list);
					if (pframe) {
						pframe->common.copy_en = 0;
						ret = node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
						return ret;
					} else {
						pr_err("fail to dequeue frame for node %d\n", node->node_id);
						return -EFAULT;
					}
				}
				return ret;
			} else if (atomic_read(&node->opt_frame_done) == 0 && node->pre_raw_flag == PRE_RAW_DEAL) {
				if (node->in_queue.cnt < node->opt_buffer_num)
					break;
				atomic_set(&node->opt_frame_done, 1);
				loop_num = node->opt_buffer_num;
			} else {
				pframe = CAM_QUEUE_DEQUEUE_TAIL(&node->in_queue, struct cam_frame, list);
				if (pframe) {
					pframe->common.copy_en = 0;
					ret = node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
					return ret;
				} else {
					pr_err("fail to dequeue frame for node %d\n", node->node_id);
					return -EFAULT;
				}
			}
		}
		break;
	case CAM_COPY_ICAP_SCENE:
		if (node->cap_param.cap_type == DCAM_CAPTURE_STOP) {
			if (node->in_queue.cnt > node->cache_num) {
				pframe = CAM_QUEUE_DEQUEUE(&node->in_queue, struct cam_frame, list);
				if (pframe) {
					pframe->common.copy_en = 0;
					ret = node->copy_cb_func(CAM_CB_ICAP_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
					return ret;
				} else {
					pr_err("fail to dequeue frame for node %d\n", node->node_id);
					return -EFAULT;
				}
			}
		}
		break;
	default :
		pr_err("fail to support scene id %d\n", node->scene_id);
		return -EFAULT;
	}

	ret = camcopy_node_copy_frame(node, loop_num);

	return ret;
}

int cam_copy_node_request_proc(struct cam_copy_node *node, void *param)
{
	int ret = 0;
	struct cam_node_cfg_param *node_param = NULL;
	struct cam_frame *pframe = NULL;
	struct cam_pipeline *pipeline = NULL;
	struct cam_node *cam_node = NULL;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}
	node_param = (struct cam_node_cfg_param *)param;
	pframe = (struct cam_frame *)node_param->param;
	pframe->common.priv_data = node;

	cam_node = (struct cam_node *)node->copy_cb_handle;
	pipeline = (struct cam_pipeline *)cam_node->data_cb_handle;
	if (node->copy_flag) {
		if (pipeline->debug_log_switch)
			pr_info("pipeline_type %s, fid %d, ch_id %d, buf %x, w %d, h %d, pframe->common.is_reserved %d, compress_en %d\n",
				pipeline->pipeline_graph->name, pframe->common.fid, pframe->common.channel_id, pframe->common.buf.mfd,
				pframe->common.width, pframe->common.height, pframe->common.is_reserved, pframe->common.is_compressed);

		node->record_channel_id = pframe->common.channel_id;
		ret = CAM_QUEUE_ENQUEUE(&node->in_queue, &pframe->list);
		if (ret == 0)
			complete(&node->thread.thread_com);
		else
			pr_err("fail to enqueue cam copy %d frame\n", node->node_id);
	} else {
		pframe->common.copy_en = 0;
		node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
	}

	return ret;
}

void *cam_copy_node_get(uint32_t node_id, cam_data_cb cb_func, void *priv_data)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!cb_func || !priv_data) {
		pr_err("fail to get valid ptr %px %px\n", cb_func, priv_data);
		return NULL;
	}

	node = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_copy_node));
	if (!node) {
		pr_err("fail to get valid cam copy node\n");
		return NULL;
	}

	if (!node->copy_cb_func) {
		node->copy_cb_func = cb_func;
		node->copy_cb_handle = priv_data;
	}

	CAM_QUEUE_INIT(&node->in_queue, COPY_NODE_Q_LEN, cam_queue_empty_frame_put);
	CAM_QUEUE_INIT(&node->out_queue, COPY_NODE_Q_LEN, cam_queue_empty_frame_put);
	node->node_id = node_id;
	node->copy_flag = CAM_DISABLE;
	node->pre_raw_flag = PRE_RAW_CACHE;
	node->opt_buffer_num = 0;
	node->scene_id = CAM_COPY_NORMAL_SCENE;
	node->cache_num = 0;

	thrd = &node->thread;
	sprintf(thrd->thread_name, "cam_copy_node%d", node->node_id);
	ret = camthread_create(node, thrd, camcopy_node_frame_start);
	if (unlikely(ret != 0)) {
		pr_err("fail to create cam copy node%d thread\n", node->node_id);
		return NULL;
	}

	pr_info("cam copy node %d get\n", node->node_id);

	return node;
}

void cam_copy_node_put(struct cam_copy_node *node)
{
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	camthread_stop(&node->thread);
	CAM_QUEUE_CLEAN(&node->in_queue, struct cam_frame, list);
	CAM_QUEUE_CLEAN(&node->out_queue, struct cam_frame, list);
	node->copy_cb_func = NULL;
	node->copy_cb_handle = NULL;
	pr_info("cam copy node %d put\n", node->node_id);
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
}

