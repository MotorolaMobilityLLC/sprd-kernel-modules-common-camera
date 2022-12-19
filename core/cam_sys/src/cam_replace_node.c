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
#include "cam_replace_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "CAM_REPLACE_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define CAM_REPLACE_NODE_PATH "/data/ylog/"
#define BYTE_PER_ONCE                   4096

static void camreplace_frame_put(void *param)
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

static void camreplace_node_read_data_to_buf(uint8_t *buffer,
		ssize_t size, const char *file)
{
	ssize_t result = 0, total = 0, read = 0;
	struct file *wfp = NULL;

	wfp = cam_filp_open(file, O_RDONLY, 0666);
	if (IS_ERR_OR_NULL(wfp)) {
		pr_err("fail to open file %s\n", file);
		return;
	}
	pr_debug("read image buf=%p, size=%d\n", buffer, (uint32_t)size);
	do {
		read = (BYTE_PER_ONCE < size) ? BYTE_PER_ONCE : size;
		result = cam_kernel_read(wfp, buffer, read, &wfp->f_pos);
		pr_debug("read result: %d, size: %d, pos: %d\n",
		(uint32_t)result,  (uint32_t)size, (uint32_t)wfp->f_pos);

		if (result > 0) {
			size -= result;
			buffer += result;
		}
		total += result;
	} while ((result > 0) && (size > 0));
	cam_filp_close(wfp, NULL);

	pr_debug("read image done, total=%d\n", (uint32_t)total);
}

static void camreplace_node_frame_size_get(struct camera_frame *frame, struct cam_replace_msg *msg, int cur_layer)
{
	if (camcore_raw_fmt_get(frame->cam_fmt))
		msg->size = cal_sprd_raw_pitch(frame->width, frame->cam_fmt) * frame->height;
	else if (frame->cam_fmt == CAM_FULL_RGB14)
		msg->size = frame->width * frame->height;
	else {
		if (cur_layer == 0)
			msg->size = cal_sprd_yuv_pitch(frame->width, frame->cam_fmt, cam_is_pack(frame->cam_fmt)) * frame->height;
		else
			msg->size = cal_sprd_yuv_pitch(msg->align_w, frame->cam_fmt, cam_is_pack(frame->cam_fmt)) * msg->align_h;
	}

	pr_debug("replace frame buf_size %d cal_size %d fmt %s\n", frame->buf.size, msg->size, camport_fmt_name_get(frame->img_fmt));
}

static void camreplace_node_param_cfg(struct cam_replace_msg *msg, struct camera_frame *pframe)
{
	msg->offset = 0;
	msg->is_compressed = pframe->is_compressed;
	if (pframe->need_pyr_rec) {
		if (pframe->pyr_status == ONLINE_DEC_ON)
			msg->layer_num = DCAM_PYR_DEC_LAYER_NUM;
		else
			msg->layer_num = ISP_PYR_DEC_LAYER_NUM;
		while (isp_rec_small_layer_w(pframe->width, msg->layer_num) < MIN_PYR_WIDTH
			|| isp_rec_small_layer_h(pframe->height, msg->layer_num) < MIN_PYR_HEIGHT) {
			if (--msg->layer_num == 0)
				break;
		}
		msg->align_w = isp_rec_layer0_width(pframe->width, msg->layer_num);
		msg->align_h = isp_rec_layer0_heigh(pframe->height, msg->layer_num);
	} else
		msg->layer_num = 0;
}

static void camreplace_node_frame_file_read(struct camera_frame *frame,
		struct cam_replace_msg *msg, uint8_t *name, uint8_t *name1)
{
	unsigned long  addr = 0;

	if (cam_buf_kmap(&frame->buf)) {
		pr_err("fail to kmap replace buf\n");
		return;
	}

	addr = frame->buf.addr_k + msg->offset;
	if (frame->cam_fmt >= CAM_YUV_BASE && frame->cam_fmt <= CAM_YVU420_2FRAME_MIPI) {
		camreplace_node_read_data_to_buf((uint8_t *)addr, msg->size, name);
		addr += msg->size;
		msg->size = msg->size / 2;
		camreplace_node_read_data_to_buf((uint8_t *)addr, msg->size, name1);
		msg->offset += msg->size * 3;
	} else
		camreplace_node_read_data_to_buf((uint8_t *)addr, msg->size, name);

	cam_buf_kunmap(&frame->buf);
}

int camreplace_node_image_name_get(struct camera_frame *frame, struct cam_replace_msg *msg,
	uint8_t *file_name, uint8_t *file_name1, int cur_layer)
{
	int ret = 0;
	uint8_t tmp_str[40] = { '\0' };

	strcat(file_name, CAM_REPLACE_NODE_PATH);
	sprintf(tmp_str, "%d", cur_layer);
	strcat(file_name, tmp_str);
	strcat(file_name1, file_name);

	switch (g_dbg_replace_src) {
	case REPLACE_IMG_MIPIRAW:
		strcat(file_name, ".mipi_raw");
		break;
	case REPLACE_IMG_YUV:
		strcat(file_name, ".y");
		strcat(file_name1, ".uv");
		break;
	case REPLACE_IMG_RAW:
		strcat(file_name, ".raw");
		break;
	default:
		pr_err("fail to get valid replace type\n");
		return -EFAULT;
		break;
	}
	return ret;
}

static int camreplace_node_switch(struct cam_replace_node *node, struct camera_frame *pframe)
{
	int ret = 0;

	if (!node)
		return ret;

	/* Prev/video pipeline replace dcam online bin port need set g_dbg_replaceswitch,
			cap pipeline replace dcam online full bort need start capture. */
	if ((pframe->link_from.node_type == CAM_NODE_TYPE_DCAM_ONLINE)
		&& ((pframe->link_from.port_id == PORT_BIN_OUT && !g_dbg_replace_switch)
		|| (pframe->link_from.port_id == PORT_FULL_OUT && pframe->link_to.node_type == CAM_NODE_TYPE_DCAM_ONLINE)))
		ret = 0;
	else
		ret = pframe->replace_en;

	return ret;
}

static int camreplace_node_frame_start(void *param)
{
	int ret = 0, i = 0;
	struct cam_replace_msg msg  = {0};
	struct cam_replace_node *node = NULL;
	struct camera_frame *pframe = NULL;
	uint8_t file_name[256] = { '\0' };
	uint8_t file_name1[256] = { '\0' };

	node = (struct cam_replace_node *)param;
	if (!node) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	pframe = cam_queue_dequeue(&node->replace_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_err("fail to get input frame for replace node %d\n", node->node_id);
		return -EFAULT;
	}

	if (camreplace_node_switch(node, pframe)) {
		camreplace_node_param_cfg(&msg, pframe);
		for (i = 0; i <= msg.layer_num; i++) {
			if (i > 0) {
				msg.is_compressed = 0;
				msg.align_w = msg.align_w / 2;
				msg.align_h = msg.align_h / 2;
			}
			camreplace_node_image_name_get(pframe, &msg, file_name, file_name1, i);
			if (msg.is_compressed == 0) {
				camreplace_node_frame_size_get(pframe, &msg, i);
				camreplace_node_frame_file_read(pframe, &msg, file_name, file_name1);
			}
			memset(file_name, 0, sizeof(file_name));
			memset(file_name1, 0, sizeof(file_name1));
		}
	}
	pframe->replace_en = 0;
	pr_debug("cam replace node %d frame start\n", node->node_id);
	node->replace_cb_func(CAM_CB_REPLACE_DATA_DONE, pframe, node->replace_cb_handle);

	return ret;
}

int cam_replace_node_request_proc(struct cam_replace_node *node, void *param)
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
	ret = cam_queue_enqueue(&node->replace_queue, &pframe->list);
	if (ret == 0)
		complete(&node->thread.thread_com);
	else
		pr_err("fail to enqueue cam replace %d frame\n", node->node_id);

	return ret;
}

void cam_replace_node_set(void *param)
{
	uint32_t val = 0;
	int i = 0;
	uint8_t replace_cmd[REPLACE_CMD_MAX];

	if (!param) {
		pr_err("fail to get replace info\n");
		return;
	}

	val = *(uint32_t *)param;
	for (i = 0; i < 3; i++) {
		replace_cmd[i] = val % 10;
		val = val /10;
	}
	replace_cmd[REPLACE_CMD_PIPE_TYPE] = val;
	if (replace_cmd[REPLACE_CMD_EN]) {
		g_dbg_replace.replace_en = replace_cmd[REPLACE_CMD_EN];
		g_dbg_replace.replace_port_id = replace_cmd[REPLACE_CMD_PORT_ID];
		g_dbg_replace.replace_node_type = replace_cmd[REPLACE_CMD_NODE_TYPE];
		g_dbg_replace.replace_pipeline_type = replace_cmd[REPLACE_CMD_PIPE_TYPE];
		g_dbg_replace.replace_ongoing = 1;
		pr_info("replace pipeline %u node %u port id %u en %u\n",
			g_dbg_replace.replace_pipeline_type, g_dbg_replace.replace_node_type,
			g_dbg_replace.replace_port_id, g_dbg_replace.replace_en);
	} else {
		if ((g_dbg_replace.replace_pipeline_type == replace_cmd[REPLACE_CMD_PIPE_TYPE])
			&& (g_dbg_replace.replace_node_type == replace_cmd[REPLACE_CMD_NODE_TYPE])
			&& (g_dbg_replace.replace_port_id == replace_cmd[REPLACE_CMD_PORT_ID])) {
			g_dbg_replace.replace_en = replace_cmd[REPLACE_CMD_EN];
			pr_info("replace pipeline %u node %u port id %u en %u\n",
				g_dbg_replace.replace_pipeline_type, g_dbg_replace.replace_node_type,
				g_dbg_replace.replace_port_id, g_dbg_replace.replace_en);
		}
	}
}

void *cam_replace_node_get(uint32_t node_id, cam_data_cb cb_func, void *priv_data)
{
	int ret = 0;
	struct cam_replace_node *node = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!cb_func || !priv_data) {
		pr_err("fail to get valid ptr %px %px\n", cb_func, priv_data);
		return NULL;
	}

	node = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_replace_node));
	if (!node) {
		pr_err("fail to get valid cam replace node\n");
		return NULL;
	}

	if (!node->replace_cb_func) {
		node->replace_cb_func = cb_func;
		node->replace_cb_handle = priv_data;
	}

	cam_queue_init(&node->replace_queue, REPLACE_NODE_Q_LEN, camreplace_frame_put);
	init_completion(&node->replace_com);
	node->node_id = node_id;

	thrd = &node->thread;
	sprintf(thrd->thread_name, "cam_replace_node%d", node->node_id);
	ret = camthread_create(node, thrd, camreplace_node_frame_start);
	if (unlikely(ret != 0)) {
		pr_err("fail to create cam replace node%d thread\n", node->node_id);
		return NULL;
	}

	pr_info("cam replace node %d get\n", node->node_id);

	return node;
}

void cam_replace_node_put(struct cam_replace_node *node)
{
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	camthread_stop(&node->thread);
	cam_queue_clear(&node->replace_queue, struct camera_frame, list);
	node->replace_cb_func = NULL;
	node->replace_cb_handle = NULL;
	pr_info("cam replace node %d put\n", node->node_id);
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
}

