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

#include "cam_node.h"
#include "cam_copy_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "CAM_COPY_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__
#define COPY_ENABLE                   1
#define COPY_DISENABLE                0

static void camcopy_frame_put(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (frame->buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&frame->buf);
	else {
		if (frame->buf.mapping_state)
			cam_buf_kunmap(&frame->buf);
		cam_buf_free(&frame->buf);
	}
	cam_queue_empty_frame_put(frame);
}

int cam_copy_node_buffer_cfg(void *handle, void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct camera_frame *pframe = NULL;

	if(!handle || !param) {
		pr_err("fail to get valid inptr %p, %p\n", handle, param);
		return -EFAULT;
	}

	node = (struct cam_copy_node *)handle;
	node->copy_flag = COPY_ENABLE;
	pframe = (struct camera_frame *)param;
	ret = cam_queue_enqueue(&node->out_queue, &pframe->list);
	if (ret)
		pr_err("fail to enqueue copy buffer\n");

	return ret;
}

static int camcopy_node_frame_start(void *param)
{
	int ret = 0;
	struct cam_copy_node *node = NULL;
	struct camera_frame *pframe = NULL, *raw_frame = NULL;

	node = (struct cam_copy_node *)param;
	if (!node) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	pframe = cam_queue_dequeue(&node->in_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_err("fail to get input frame for dump node %d\n", node->node_id);
		return -EFAULT;
	}
	raw_frame = cam_queue_dequeue(&node->out_queue, struct camera_frame, list);
	if (raw_frame == NULL) {
		pr_debug("raw path no get out queue\n");
		pframe->copy_en = 0;
		node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);
	} else {
		pr_debug("start copy src queue data\n");
		if (cam_buf_kmap(&pframe->buf)) {
				pr_err("fail to kmap temp buf\n");
				return -EFAULT;
		}
		if (cam_buf_kmap(&raw_frame->buf)) {
				pr_err("fail to kmap raw buf\n");
				return -EFAULT;
		}
		memcpy((char *)raw_frame->buf.addr_k, (char *)pframe->buf.addr_k, pframe->buf.size);
		/* use SOF time instead of ISP time for better accuracy */
		raw_frame->width = pframe->width;
		raw_frame->height = pframe->height;
		raw_frame->fid = pframe->fid;
		raw_frame->sensor_time.tv_sec = pframe->sensor_time.tv_sec;
		raw_frame->sensor_time.tv_usec = pframe->sensor_time.tv_usec;
		raw_frame->boot_sensor_time = pframe->boot_sensor_time;
		/* end copy, out src buf*/
		cam_buf_kunmap(&pframe->buf);
		pframe->copy_en = 0;
		node->copy_cb_func(CAM_CB_COPY_SRC_BUFFER, pframe, node->copy_cb_handle);

		cam_buf_kunmap(&raw_frame->buf);
		raw_frame->evt = IMG_TX_DONE;
		raw_frame->irq_type = CAMERA_IRQ_IMG;
		raw_frame->priv_data = node;
		raw_frame->channel_id = CAM_CH_RAW;
		raw_frame->link_to.node_type = CAM_NODE_TYPE_USER;
		raw_frame->link_to.node_id = CAM_LINK_DEFAULT_NODE_ID;
		pr_debug("get out raw frame fd 0x%x raw fram fid %d\n", raw_frame->buf.mfd, raw_frame->fid);
		node->copy_cb_func(CAM_CB_DCAM_DATA_DONE, raw_frame, node->copy_cb_handle);
	}

	return ret;
}

int cam_copy_node_request_proc(struct cam_copy_node *node, void *param)
{
	int ret = 0;
	struct cam_node_cfg_param *node_param = NULL;
	struct camera_frame *pframe = NULL;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}
	node_param = (struct cam_node_cfg_param *)param;
	pframe = (struct camera_frame *)node_param->param;
	pframe->priv_data = node;

	if (node->copy_flag) {
		ret = cam_queue_enqueue(&node->in_queue, &pframe->list);
		if (ret == 0)
			complete(&node->thread.thread_com);
		else
			pr_err("fail to enqueue cam copy %d frame\n", node->node_id);
	} else {
		pframe->copy_en = 0;
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

	cam_queue_init(&node->in_queue, COPY_NODE_Q_LEN, camcopy_frame_put);
	cam_queue_init(&node->out_queue, COPY_NODE_Q_LEN, camcopy_frame_put);
	node->node_id = node_id;
	node->copy_flag = COPY_DISENABLE;

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
	cam_queue_clear(&node->in_queue, struct camera_frame, list);
	cam_queue_clear(&node->out_queue, struct camera_frame, list);
	node->copy_cb_func = NULL;
	node->copy_cb_handle = NULL;
	pr_info("cam copy node %d put\n", node->node_id);
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
}

