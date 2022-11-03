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

#include <linux/vmalloc.h>
#include <linux/delay.h>
#include "isp_port.h"
#include "cam_port.h"
#include "isp_interface.h"
#include "isp_dev.h"
#include "isp_node.h"
#include "isp_int.h"
#include "isp_drv.h"
#include "isp_slice.h"
#include "cam_node.h"
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define ISP_PATH_DECI_FAC_MAX       4
#define ISP_SC_COEFF_UP_MAX         ISP_SCALER_UP_MAX
#define ISP_SC_COEFF_DOWN_MAX       ISP_SCALER_UP_MAX

static void ispport_frame_ret(void *param)
{
	struct camera_frame * pframe = NULL;
	struct isp_port *port = NULL;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	pframe = (struct camera_frame *)param;
	port = (struct isp_port *)pframe->priv_data;
	if (!port) {
		pr_err("fail to get out_frame port.\n");
		return;
	}

	pr_debug("port type:%d frame %p, ch_id %d, buf_fd %d\n",
		port->type, pframe, pframe->channel_id, pframe->buf.mfd);

	if (pframe->is_reserved)
		port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
	else {
		if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&pframe->buf);
		switch (port->type) {
		case PORT_TRANSFER_IN:
			isp_node_offline_pararm_free(pframe->param_data);
			pframe->param_data = NULL;
			port->data_cb_func(CAM_CB_ISP_RET_SRC_BUF, pframe, port->data_cb_handle);
			break;
		case PORT_TRANSFER_OUT:
			port->data_cb_func(CAM_CB_ISP_RET_DST_BUF, pframe, port->data_cb_handle);
			break;
		default:
			pr_err("fail to get port type.\n");
		}
	}
}

static uint32_t ispport_base_cfg(struct isp_port *port, void *param)
{
	struct isp_path_base_desc *cfg_in = NULL;
	cfg_in = VOID_PTR_TO(param, struct isp_path_base_desc);

	port->fmt = cfg_in->out_fmt;
	port->data_endian = cfg_in->endian;
	port->size = cfg_in->output_size;
	if (cfg_in->is_work == 1)
		atomic_set(&port->is_work, 1);
	pr_info("port_id %d, out fmt %s, size:%d %d", port->port_id, camport_fmt_name_get(port->fmt), port->size.w, port->size.h);
	return 0;
}

static uint32_t ispport_size_check(struct isp_port *port, void *param)
{
	int invalid = 0;
	struct isp_size_desc *cfg_in = NULL;

	cfg_in = VOID_PTR_TO(param, struct isp_size_desc);
	if (port->type == PORT_TRANSFER_IN) {
		if (((cfg_in->trim.start_x + cfg_in->trim.size_x) > cfg_in->size.w) ||
			((cfg_in->trim.start_y + cfg_in->trim.size_y) > cfg_in->size.h))
			invalid |= 1;
		invalid |= ((cfg_in->trim.start_x & 1) | (cfg_in->trim.start_y & 1));
		if (invalid) {
			pr_err("fail to get valid ctx size. src %d %d, crop %d %d %d %d\n",
				cfg_in->size.w, cfg_in->size.h, cfg_in->trim.start_x, cfg_in->trim.start_y,
				cfg_in->trim.size_x, cfg_in->trim.size_y);
			return -EINVAL;
		}
	}
	return 0;
}

static uint32_t ispport_size_cfg(struct isp_port *port, void *param)
{
	struct isp_size_desc *cfg_in = NULL;

	cfg_in = VOID_PTR_TO(param, struct isp_size_desc);
	if (cfg_in->size.w > 0 && cfg_in->size.h > 0)
		port->size = cfg_in->size;
	if (cfg_in->trim.size_x > 0 && cfg_in->trim.size_y > 0)
		port->trim = cfg_in->trim;

	if (port->type == PORT_TRANSFER_IN) {
		port->ori_src = cfg_in->size;
		port->zoom_conflict_with_ltm = cfg_in->zoom_conflict_with_ltm;
	}

	pr_debug("port %d size %d %d trim %d %d %d %d\n", port->port_id,
		port->size.w, port->size.h, port->trim.start_x, port->trim.start_y, port->trim.size_x, port->trim.size_y);
	return 0;
}

static uint32_t ispport_buf_set(struct isp_port *port, void *param)
{
	uint32_t ret = 0;
	struct camera_frame *pframe = NULL;

	pframe = VOID_PTR_TO(param, struct camera_frame);
	if (port->type == PORT_TRANSFER_IN && pframe->pyr_status == OFFLINE_DEC_ON) {
		port->fetch_path_sel = ISP_FETCH_PATH_NORMAL;
		port->size.w = pframe->width;
		port->size.h = pframe->height;
		port->trim.start_x = 0;
		port->trim.start_y = 0;
		port->trim.size_x = pframe->width;
		port->trim.size_y = pframe->height;
	}
	/* close ltm map when zoom > 1x*/
	pr_debug("isp%d, conflict %d, ori size %dx%d, now size %dx%d\n", port->port_id, port->zoom_conflict_with_ltm,
		port->ori_src.w, port->ori_src.h, port->size.w, port->size.h);
	if (port->type == PORT_TRANSFER_IN && port->zoom_conflict_with_ltm && ((port->size.w != port->ori_src.w) || port->size.h != port->ori_src.h))
		pframe->xtm_conflict.need_ltm_map = 0;

	pframe->is_reserved = 0;
	pframe->priv_data = port;
	ret = cam_queue_enqueue(&port->out_buf_queue, &pframe->list);
	if (ret)
		pr_err("fail to enqueue output buffer,type:%d port_id %d.\n", port->type, port->port_id);
	return ret;
}

static uint32_t ispport_reserved_buf_set(struct isp_port *port, void *param)
{
	struct camera_frame *pframe = NULL;

	pframe = VOID_PTR_TO(param, struct camera_frame);
	port->reserved_buf_fd = pframe->buf.mfd;
	port->reserve_buf_size = pframe->buf.size;
	return 0;
}

static int ispport_size_update(struct isp_port *port, void *param)
{
	struct isp_offline_param *in_param = NULL;
	struct camera_frame *pframe = NULL;
	struct isp_size_desc cfg;
	uint32_t update[ISP_SPATH_NUM] = {
	ISP_PATH0_TRIM, ISP_PATH1_TRIM, ISP_PATH2_TRIM};
	struct img_trim path_trim;

	pframe = VOID_PTR_TO(param, struct camera_frame);
	in_param = (struct isp_offline_param*)pframe->param_data;
	if (atomic_read(&port->user_cnt) < 1)
		return 0;

	if (port->type == PORT_TRANSFER_IN) {
		if(in_param->valid & ISP_SRC_SIZE) {
			memcpy(&port->original, &in_param->src_info,
				sizeof(port->original));
			cfg.size = in_param->src_info.dst_size;
			cfg.trim.start_x = 0;
			cfg.trim.start_y = 0;
			cfg.trim.size_x = cfg.size.w;
			cfg.trim.size_y = cfg.size.h;
			port->size = cfg.size;
			port->trim = cfg.trim;
			pr_debug("isp sw %d update size: %d %d\n",
				port->port_id, cfg.size.w, cfg.size.h);
			if (pframe->blkparam_info.param_block) {
				pframe->blkparam_info.param_block->src_w = port->size.w;
				pframe->blkparam_info.param_block->src_h = port->size.h;
			}
		}
	}

	if (port->type == PORT_TRANSFER_OUT && (port->port_id == PORT_PRE_OUT || port->port_id == PORT_VID_OUT)) {
		if (in_param->valid & update[port->port_id])
			path_trim = in_param->trim_path[port->port_id];
		else if (in_param->valid & ISP_SRC_SIZE) {
			path_trim.start_x = path_trim.start_y = 0;
			path_trim.size_x = in_param->src_info.dst_size.w;
			path_trim.size_y = in_param->src_info.dst_size.h;
		} else
			return 0;

		port->trim = path_trim;
		pr_debug("update isp port%d trim %d %d %d %d\n",
			port->port_id, path_trim.start_x, path_trim.start_y,
			path_trim.size_x, path_trim.size_y);
	}
	return 0;
}

static int ispport_fetchport_postproc(struct isp_port *port, void *param)
{
	struct camera_frame *pframe = NULL;
	struct camera_frame *pframe_data = NULL;
	struct isp_node_postproc_param *post_param = NULL;

	post_param = VOID_PTR_TO(param, struct isp_node_postproc_param);
	pframe = cam_queue_dequeue(&port->result_queue, struct camera_frame, list);
	pframe_data = (struct camera_frame *)pframe->pframe_data;

	if (pframe) {
		post_param->zoom_ratio = pframe->zoom_ratio;
		post_param->total_zoom = pframe->total_zoom;
		/* return buffer to cam channel shared buffer queue. */
		if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&pframe->buf);
		port->data_cb_func(CAM_CB_ISP_RET_SRC_BUF, pframe, port->data_cb_handle);
		pr_debug("port %d, ch_id %d, fid:%d, mfd:%d, shard buffer cnt:%d\n",
			port->port_id, pframe->channel_id, pframe->fid, pframe->buf.mfd,
			port->result_queue.cnt);
	} else
		pr_err("fail to get src frame  sw_idx=%d  proc_queue.cnt:%d\n",
			port->port_id, port->result_queue.cnt);

	if (pframe_data != NULL) {
		if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&pframe_data->buf);
		port->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe_data, port->data_cb_handle);
		pr_debug("port_link_from %d, ch_id %d, fid:%d\n",
			pframe_data->link_from, pframe->channel_id, pframe->fid);
	}
	return 0;
}

static int ispport_storeport_postproc(struct isp_port *port, void *param)
{
	struct camera_frame *pframe = NULL;
	struct isp_node_postproc_param *post_param;
	struct camera_frame *pframe1 = NULL;

	post_param = VOID_PTR_TO(param, struct isp_node_postproc_param);
	pframe = cam_queue_dequeue(&port->result_queue, struct camera_frame, list);

	if (!pframe) {
		pr_err("fail to get frame from queue. port:%d, path:%d\n", port->port_id);
		return 0;
	}
	pframe->boot_time = *(post_param->boot_time);
	pframe->time.tv_sec = post_param->cur_ts->tv_sec;
	pframe->time.tv_usec = post_param->cur_ts->tv_nsec / NSEC_PER_USEC;
	pframe->zoom_ratio = post_param->zoom_ratio;
	pframe->total_zoom = post_param->total_zoom;

	pr_debug("port%d, ch_id %d, fid %d, mfd 0x%x, queue cnt:%d, is_reserved %d\n",
		port->port_id, pframe->channel_id, pframe->fid, pframe->buf.mfd,
		port->result_queue.cnt, pframe->is_reserved);
	pr_debug("time_sensor %03d.%6d, time_isp %03d.%06d\n",
		(uint32_t)pframe->sensor_time.tv_sec,
		(uint32_t)pframe->sensor_time.tv_usec,
		(uint32_t)pframe->time.tv_sec,
		(uint32_t)pframe->time.tv_usec);

	if (unlikely(pframe->is_reserved)) {
		port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
	} else if (pframe->state == ISP_STREAM_POST_PROC) {
		pframe1 = cam_queue_dequeue(&port->out_buf_queue, struct camera_frame, list);
		if (!pframe1) {
			pr_info("warning no frame get from queue\n");
			return 0;
		}
		pframe->pframe_data = pframe1;
		port->data_cb_func(CAM_CB_YUV_SCALER_DATA_DONE, pframe, port->data_cb_handle);
	} else {
		if (pframe->buf.mfd == port->reserved_buf_fd) {
			pframe->buf.size = port->reserve_buf_size;
			pr_info("pframe->buf.size = %d, path->reserve_buf_size = %d",
				(int)pframe->buf.size, (int)port->reserve_buf_size);
		}
		cam_buf_iommu_unmap(&pframe->buf);
		port->data_cb_func(CAM_CB_ISP_RET_DST_BUF, pframe, port->data_cb_handle);
	}
	return 0;
}

static int ispport_port_postproc(struct isp_port *port, void *param)
{
	if (port->type == PORT_TRANSFER_IN)
		ispport_fetchport_postproc(port, param);

	if (port->type == PORT_TRANSFER_OUT)
		ispport_storeport_postproc(port, param);
	return 0;
}

static bool ispport_fid_check(struct camera_frame *frame, void *data)
{
	uint32_t target_fid;

	if (!frame || !data)
		return false;

	target_fid = *(uint32_t *)data;

	pr_debug("target_fid = %d frame->user_fid = %d\n", target_fid, frame->user_fid);
	return frame->user_fid == CAMERA_RESERVE_FRAME_NUM
		|| frame->user_fid == target_fid;
}

static struct camera_frame *ispport_reserved_buf_get(reserved_buf_get_cb resbuf_get_cb, void *cb_data, void *path)
{
	struct camera_frame *frame = NULL;

	if (resbuf_get_cb)
		resbuf_get_cb(RESERVED_BUF_GET_CB, (void *)&frame, cb_data);
	if (frame != NULL) {
		frame->priv_data = path;
		if (cam_buf_iommu_single_page_map(&frame->buf, CAM_IOMMUDEV_ISP)) {
			pr_err("fail to iommu map\n");
			resbuf_get_cb(RESERVED_BUF_SET_CB, frame, cb_data);
			frame = NULL;
		}
	}
	return frame;
}

static struct camera_frame *ispport_out_frame_get(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	int ret = 0, hw_path_id = isp_port_id_switch(port->port_id);
	struct camera_frame *out_frame = NULL;
	uint32_t buf_type = 0;

	if (!port || !port_cfg) {
		pr_err("fail to get valid input pctx %p\n", port);
		return NULL;
	}

	if (port_cfg->src_frame) {
		buf_type = port_cfg->src_frame->buf_type;
		switch (buf_type) {
		case ISP_STREAM_BUF_OUT:
			goto normal_out_put;
		case ISP_STREAM_BUF_RESERVED:
			out_frame = ispport_reserved_buf_get(port->resbuf_get_cb, port->resbuf_cb_data, port);
			if (out_frame) {
				port_cfg->valid_out_frame = 1;
				pr_debug("reserved buffer %d %lx\n",out_frame->is_reserved, out_frame->buf.iova);
			}
			break;
		case ISP_STREAM_BUF_POSTPROC:
			out_frame = port_cfg->superzoom_frame;
			if (out_frame) {
				port_cfg->need_post_proc[hw_path_id] = 1;
				ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
				port_cfg->valid_out_frame = 1;
				pr_debug("postproc buf %px\n", out_frame->buf.iova);
				if (ret)
					out_frame = NULL;
			}
			break;
		default:
			pr_err("fail to support buf_type %d\n", buf_type);
			break;
		}
		goto exit;
	}

normal_out_put:

	if (port->uframe_sync && port_cfg->target_fid != CAMERA_RESERVE_FRAME_NUM)
		out_frame = cam_queue_dequeue_if(&port->out_buf_queue,
			ispport_fid_check, (void *)&port_cfg->target_fid);
	else
		out_frame = cam_queue_dequeue(&port->out_buf_queue,
			struct camera_frame, list);

	if (out_frame)
		port_cfg->valid_out_frame = 1;
	else
		out_frame = ispport_reserved_buf_get(port->resbuf_get_cb, port->resbuf_cb_data, port);
	if (out_frame != NULL) {
		if (out_frame->is_reserved == 0 &&
			(out_frame->buf.mapping_state & CAM_BUF_MAPPING_DEV) == 0) {
			if (out_frame->buf.mfd == port->reserved_buf_fd) {
				out_frame->buf.size = port->reserve_buf_size;
				pr_debug("out_frame->buf.size = %d, path->reserve_buf_size = %d",
					(int)out_frame->buf.size, (int)port->reserve_buf_size);
				ret = cam_buf_iommu_single_page_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
			} else
				ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);

			pr_debug("map output buffer %08x\n", (uint32_t)out_frame->buf.iova);
			if (ret) {
				cam_queue_enqueue(&port->out_buf_queue, &out_frame->list);
				out_frame = NULL;
				pr_err("fail to map isp iommu buf.\n");
			}
		}
	}

exit:
	return out_frame;
}

static uint32_t ispport_uframe_fid_get(struct isp_port* port, void *param)
{
	uint32_t *port_fid = NULL;
	struct camera_frame *frame = NULL;

	port_fid = VOID_PTR_TO(param, uint32_t);
	*port_fid = CAMERA_RESERVE_FRAME_NUM;
	if (port->uframe_sync) {
		frame = cam_queue_dequeue_peek(&port->out_buf_queue, struct camera_frame, list);
		if (!frame)
			return 0;
		*port_fid = frame->user_fid;
	}
	return 0;
}

static int ispport_fetch_frame_cycle(struct isp_port *port, void *param)
{
	struct camera_frame *pframe = NULL;
	struct camera_frame *pframe_data = NULL;
	struct isp_port_cfg *port_cfg = NULL;
	uint32_t ret = 0, loop = 0;

	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	pframe = cam_queue_dequeue(&port->out_buf_queue, struct camera_frame, list);
	pframe_data = (struct camera_frame *)pframe->pframe_data;
	if (pframe == NULL) {
		pr_err("fail to get frame (%px) for port %d\n", pframe, port->port_id);
		ret = -EINVAL;
		goto err;
	}

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. port_id %d\n", port->port_id);
		ret = -EINVAL;
		goto err;
	}

	if (pframe_data != NULL) {
		ret = cam_buf_iommu_map(&pframe_data->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map buf to ISP iommu. port_id %d\n", port->port_id);
			ret = -EINVAL;
			goto err;
		}
	}

	loop = 0;
	do {
		ret = cam_queue_enqueue(&port->result_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		/* wait for previous frame proccessed done.*/
		usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
		goto err;
	}
	port_cfg->src_frame = pframe;
	return 0;
err:
	ispport_frame_ret(pframe);
	port_cfg->src_frame = NULL;
	if (pframe_data)
		ispport_frame_ret(pframe_data);
	return ret;
}

static int ispport_store_frameproc(struct isp_port *port, struct camera_frame *out_frame, struct isp_port_cfg *port_cfg)
{
	int hw_path_id = isp_port_id_switch(port->port_id);
	uint32_t ret = 0, loop = 0;
	if (out_frame == NULL || !port_cfg->src_frame) {
		pr_err("fail to get out_frame,port id:%d,out queue cnt:%d\n", port->port_id, cam_queue_cnt_get(&port->out_buf_queue));
		return -EINVAL;
	}

	out_frame->fid = port_cfg->src_frame->fid;
	out_frame->sensor_time = port_cfg->src_frame->sensor_time;
	out_frame->boot_sensor_time = port_cfg->src_frame->boot_sensor_time;
	out_frame->link_from.port_id = port->port_id;

	pr_debug("port %d, is_reserved %d iova 0x%x, user_fid: %x mfd 0x%x fid: %d\n",
		port->port_id, out_frame->is_reserved,
		(uint32_t)out_frame->buf.iova, out_frame->user_fid,
		out_frame->buf.mfd, out_frame->fid);
	/* config store buffer */
	ret = isp_hwctx_store_frm_set(port_cfg->pipe_info, isp_port_id_switch(port->port_id), out_frame);
	/* If some error comes then do not start ISP */

	if (ret) {
		cam_buf_iommu_unmap(&out_frame->buf);
		cam_buf_ionbuf_put(&out_frame->buf);
		cam_queue_empty_frame_put(out_frame);
		pr_err("fail to set store buffer\n");
		return -EINVAL;
	}

	if (!port_cfg->need_post_proc[hw_path_id]) {
		do {
			ret = cam_queue_enqueue(&port->result_queue, &out_frame->list);
			if (ret == 0)
				break;
			printk_ratelimited(KERN_INFO "wait for output queue. loop %d\n", loop);
			/* wait for previous frame output queue done */
			mdelay(1);
		} while (loop++ < 500);
	}

	if (ret) {
		isp_int_isp_irq_cnt_trace(port_cfg->hw_ctx_id);
		pr_err("fail to enqueue, hw %d, port %d\n", port_cfg->hw_ctx_id, port->port_id);
		/* ret frame to original queue */
		if (out_frame->is_reserved) {
			port->resbuf_get_cb(RESERVED_BUF_SET_CB, out_frame, port->resbuf_cb_data);
			if (port_cfg->rgb_ltm)
				port_cfg->rgb_ltm->ltm_ops.sync_ops.clear_status(port_cfg->rgb_ltm);
		} else {
			cam_buf_iommu_unmap(&out_frame->buf);
			cam_queue_enqueue(
				&port->out_buf_queue, &out_frame->list);
		}
		return -EINVAL;
	}
	return 0;
}

static int ispport_store_frame_cycle(struct isp_port *port, void *param)
{
	int ret = 0;
	struct isp_port_cfg *port_cfg = NULL;
	struct camera_frame *out_frame = NULL;

	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	out_frame = ispport_out_frame_get(port, port_cfg);
	ret = ispport_store_frameproc(port, out_frame, port_cfg);
	return ret;
}

static inline void ispport_slw960_frame_num_update(struct slowmotion_960fps_info *slw_960desc, struct isp_node_slwm960fps_frame_num *frame_num)
{
	if (frame_num->stage_a_frame_num > 0)
		frame_num->stage_a_frame_num--;
	else if (frame_num->stage_b_frame_num > 0)
		frame_num->stage_b_frame_num--;
	else if (frame_num->stage_c_frame_num > 0)
		frame_num->stage_c_frame_num--;

	if (slw_960desc->slowmotion_stage_a_valid_num && frame_num->stage_c_frame_num == 0) {
		frame_num->stage_a_frame_num = slw_960desc->slowmotion_stage_a_num;
		frame_num->stage_b_frame_num = slw_960desc->slowmotion_stage_b_num;
		frame_num->stage_c_frame_num = slw_960desc->slowmotion_stage_a_num;
	}
}

static int ispport_frame_cycle(struct isp_port *port, void *param)
{
	int ret = 0;
	struct isp_port_cfg *port_cfg = NULL;
	uint32_t valid_out_frame = 0;
	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);

	if (port->type == PORT_TRANSFER_IN)
		ret = ispport_fetch_frame_cycle(port, param);

	if (port->type == PORT_TRANSFER_OUT) {
		switch (port->port_id) {
		case PORT_PRE_OUT:
		case PORT_CAP_OUT:
			ret = ispport_store_frame_cycle(port, param);
			break;
		case PORT_VID_OUT:
			valid_out_frame = port_cfg->valid_out_frame;
			port_cfg->valid_out_frame = 0;
			ret = ispport_store_frame_cycle(port, param);
			if (!ret && port_cfg->uinfo->stage_a_valid_count && port_cfg->valid_out_frame)
				ispport_slw960_frame_num_update(&port_cfg->uinfo->slw_960desc, &port_cfg->uinfo->frame_num);
			port_cfg->valid_out_frame |= valid_out_frame;
			break;
		default:
			pr_err("fail to get port_id:%d or type:%d\n", port->port_id, port->type);
		}
	}
	return ret;
}

static uint32_t ispport_deci_factor_get(uint32_t src_size, uint32_t dst_size)
{
	uint32_t factor = 0;

	if (0 == src_size || 0 == dst_size)
		return factor;

	/* factor: 0 - 1/2, 1 - 1/4, 2 - 1/8, 3 - 1/16 */
	for (factor = 0; factor < ISP_PATH_DECI_FAC_MAX; factor++) {
		if (src_size < (uint32_t) (dst_size * (1 << (factor + 1))))
			break;
	}

	return factor;
}

static int ispport_scaler_param_calc(struct img_trim *in_trim,
	struct img_size *out_size, struct yuv_scaler_info *scaler,
	struct img_deci_info *deci)
{
	int ret = 0;
	unsigned int tmp_dstsize = 0;
	unsigned int align_size = 0;
	unsigned int d_max = ISP_SC_COEFF_DOWN_MAX;
	unsigned int u_max = ISP_SC_COEFF_UP_MAX;
	unsigned int f_max = ISP_PATH_DECI_FAC_MAX;

	pr_debug("in_trim_size_x:%d, in_trim_size_y:%d, out_size_w:%d,out_size_h:%d\n",
		in_trim->size_x, in_trim->size_y, out_size->w, out_size->h);
	/* check input crop limit with max scale up output size(2 bit aligned) */
	if (in_trim->size_x > (out_size->w * d_max * (1 << f_max)) ||
		in_trim->size_y > (out_size->h * d_max * (1 << f_max)) ||
		in_trim->size_x < ISP_DIV_ALIGN_W(out_size->w, u_max) ||
		in_trim->size_y < ISP_DIV_ALIGN_H(out_size->h, u_max)) {
		pr_err("fail to get in_trim %d %d. out _size %d %d, fmax %d, u_max %d\n",
				in_trim->size_x, in_trim->size_y,
				out_size->w, out_size->h, f_max, d_max);
		ret = -EINVAL;
	} else {
		scaler->scaler_factor_in = in_trim->size_x;
		scaler->scaler_ver_factor_in = in_trim->size_y;
		if (in_trim->size_x > out_size->w * d_max) {
			tmp_dstsize = out_size->w * d_max;
			deci->deci_x = ispport_deci_factor_get(in_trim->size_x, tmp_dstsize);
			deci->deci_x_eb = 1;
			align_size = (1 << (deci->deci_x + 1)) * ISP_PIXEL_ALIGN_WIDTH;
			in_trim->size_x = (in_trim->size_x) & ~(align_size - 1);
			in_trim->start_x = (in_trim->start_x) & ~(align_size - 1);
			scaler->scaler_factor_in = in_trim->size_x >> (deci->deci_x + 1);
		} else {
			deci->deci_x = 1;
			deci->deci_x_eb = 0;
		}

		if (in_trim->size_y > out_size->h * d_max) {
			tmp_dstsize = out_size->h * d_max;
			deci->deci_y = ispport_deci_factor_get(in_trim->size_y, tmp_dstsize);
			deci->deci_y_eb = 1;
			align_size = (1 << (deci->deci_y + 1)) * ISP_PIXEL_ALIGN_HEIGHT;
			in_trim->size_y = (in_trim->size_y) & ~(align_size - 1);
			in_trim->start_y = (in_trim->start_y) & ~(align_size - 1);
			scaler->scaler_ver_factor_in = in_trim->size_y >> (deci->deci_y + 1);
		} else {
			deci->deci_y = 1;
			deci->deci_y_eb = 0;
		}
		pr_debug("end out_size  w %d, h %d\n",
			out_size->w, out_size->h);

		scaler->scaler_factor_out = out_size->w;
		scaler->scaler_ver_factor_out = out_size->h;
		scaler->scaler_out_width = out_size->w;
		scaler->scaler_out_height = out_size->h;
	}

	return ret;
}

static int ispport_scaler_get(struct isp_port *in_ptr,
	struct isp_hw_path_scaler *path, struct isp_port_cfg *port_cfg)
{
	int ret = 0, path_id = 0;
	uint32_t is_yuv422 = 0, scale2yuv420 = 0;
	struct yuv_scaler_info *scaler = NULL;

	if (!in_ptr || !path) {
		pr_err("fail to get valid input ptr %p, %p\n", in_ptr, path);
		return -EFAULT;
	}

	path_id = isp_port_id_switch(in_ptr->port_id);
	scaler = &path->scaler;
	if (in_ptr->fmt == CAM_UYVY_1FRAME)
		path->uv_sync_v = 1;
	else
		path->uv_sync_v = 0;
	if (in_ptr->fmt == CAM_FULL_RGB14)
		path->path_sel = 2;
	else
		path->path_sel = 0;
	path->frm_deci = 0;
	path->dst = port_cfg->src_frame->out[path_id];
	path->out_trim.start_x = 0;
	path->out_trim.start_y = 0;
	path->out_trim.size_x = port_cfg->src_frame->out[path_id].w;
	path->out_trim.size_y = port_cfg->src_frame->out[path_id].h;
	path->regular_info.regular_mode = in_ptr->regular_mode;
	ret = ispport_scaler_param_calc(&path->in_trim, &path->dst,
		&path->scaler, &path->deci);
	if (ret) {
		pr_err("fail to calc scaler param.\n");
		return ret;
	}

	if ((in_ptr->fmt == CAM_YUV422_2FRAME) || (in_ptr->fmt == CAM_YVU422_2FRAME))
		is_yuv422 = 1;

	if (((scaler->scaler_ver_factor_in == scaler->scaler_ver_factor_out)
		&& (scaler->scaler_factor_in == scaler->scaler_factor_out)
		&& (is_yuv422 || in_ptr->scaler_bypass_ctrl))
		|| in_ptr->fmt == CAM_FULL_RGB14) {
		scaler->scaler_bypass = 1;
	} else {
		scaler->scaler_bypass = 0;

		if (in_ptr->scaler_coeff_ex) {
			/*0:yuv422 to 422 ;1:yuv422 to 420 2:yuv420 to 420*/
			scaler->work_mode = 2;
			ret = cam_scaler_coeff_calc_ex(scaler);
		}  else {
			scale2yuv420 = is_yuv422 ? 0 : 1;
			ret = cam_scaler_coeff_calc(scaler, scale2yuv420);
		}

		if (ret) {
			pr_err("fail to calc scaler coeff.\n");
			return ret;
		}
	}
	scaler->odata_mode = is_yuv422 ? 0x00 : 0x01;

	return ret;
}

static int ispport_fetch_normal_get(void *cfg_in, void *cfg_out,
		struct camera_frame *frame)
{
	int ret = 0;
	uint32_t trim_offset[3] = { 0 };
	struct img_size *src = NULL;
	struct img_trim *intrim = NULL;
	struct isp_hw_fetch_info *fetch = NULL;
	struct isp_port *port = NULL;
	struct camera_frame *pframe_data = NULL;
	uint32_t mipi_word_num_start[16] = {
		0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {
		0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	if (!cfg_in || !cfg_out || !frame) {
		pr_err("fail to get valid input ptr, %p, %p\n", cfg_in, cfg_out);
		return -EFAULT;
	}

	port = VOID_PTR_TO(cfg_in, struct isp_port);
	fetch = (struct isp_hw_fetch_info *)cfg_out;
	src = &frame->in;
	intrim = &frame->in_crop;
	pframe_data = (struct camera_frame *)frame->pframe_data;
	fetch->src = *src;
	fetch->in_trim = *intrim;
	fetch->fetch_fmt = port->fmt;
	fetch->fetch_pyr_fmt = port->pyr_out_fmt;
	fetch->fetch_3dnr_fmt = port->store_3dnr_fmt;
	fetch->bayer_pattern = port->bayer_pattern;
	if (camcore_raw_fmt_get(port->fmt))
		fetch->dispatch_color = 0;
	else if (port->fmt == CAM_FULL_RGB10)
		fetch->dispatch_color = 1;
	else
		fetch->dispatch_color = 2;
	fetch->fetch_path_sel = port->fetch_path_sel;
	fetch->addr.addr_ch0 = frame->buf.iova;
	if (pframe_data != NULL)
		fetch->addr_dcam_out.addr_ch0 = pframe_data->buf.iova;
	else
		fetch->addr_dcam_out.addr_ch0 = 0;
	switch (fetch->fetch_fmt) {
	case CAM_YUV422_3FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w / 2;
		fetch->pitch.pitch_ch2 = src->w / 2;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 + intrim->start_x / 2;
		trim_offset[2] = intrim->start_y * fetch->pitch.pitch_ch2 + intrim->start_x / 2;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		fetch->addr.addr_ch2 = fetch->addr.addr_ch1 + fetch->pitch.pitch_ch1 * fetch->src.h;
		break;
	case CAM_YUYV_1FRAME:
	case CAM_UYVY_1FRAME:
	case CAM_YVYU_1FRAME:
	case CAM_VYUY_1FRAME:
		fetch->pitch.pitch_ch0 = src->w * 2;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		break;
	case CAM_RAW_HALFWORD_10:
	case CAM_RAW_14:
		fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(src->w, cam_pack_bits(port->fmt));
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		break;
	case CAM_YUV422_2FRAME:
	case CAM_YVU422_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 + intrim->start_x;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		if (pframe_data != NULL)
			fetch->addr_dcam_out.addr_ch1 = pframe_data->buf.iova + fetch->pitch.pitch_ch0 * fetch->src.h;
		break;
	case CAM_YUV420_2FRAME:
	case CAM_YVU420_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 / 2 + intrim->start_x;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		if (pframe_data != NULL)
			fetch->addr_dcam_out.addr_ch1 = pframe_data->buf.iova + fetch->pitch.pitch_ch0 * fetch->src.h;
		pr_debug("y_addr: %x, pitch:: %x\n", fetch->addr.addr_ch0, fetch->pitch.pitch_ch0);
		break;
	case CAM_YUV420_2FRAME_10:
	case CAM_YVU420_2FRAME_10:
		fetch->pitch.pitch_ch0 = (src->w * 16 + 127) / 128 * 128 / 8;
		fetch->pitch.pitch_ch1 = (src->w * 16 + 127) / 128 * 128 / 8;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 / 2 + intrim->start_x * 2;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		if (pframe_data != NULL)
			fetch->addr_dcam_out.addr_ch1 = pframe_data->buf.iova + fetch->pitch.pitch_ch0 * fetch->src.h;
		break;
	case CAM_YUV420_2FRAME_MIPI:
	case CAM_YVU420_2FRAME_MIPI:
	{
		uint32_t start_col = intrim->start_x;
		uint32_t start_row = intrim->start_y;
		uint32_t end_col =  intrim->start_x + intrim->size_x - 1;
		fetch->pitch.pitch_ch0 = (src->w * 10 + 127) / 128 * 128 / 8;
		fetch->pitch.pitch_ch1 = (src->w * 10 + 127) / 128 * 128 / 8;
		fetch->mipi_byte_rel_pos = intrim->start_x & 0xf;
		fetch->mipi_word_num = ((end_col + 1) >> 4) * 5
			+ mipi_word_num_end[(end_col + 1) & 0xF]
			- ((start_col + 1) >> 4) * 5
			- mipi_word_num_start[(start_col + 1) & 0xF] + 1;
		fetch->mipi_byte_rel_pos_uv = fetch->mipi_byte_rel_pos;
		fetch->mipi_word_num_uv = fetch->mipi_word_num;
		trim_offset[0] = start_row * fetch->pitch.pitch_ch0 + (start_col >> 2) * 5 +
			(start_col & 0x3);
		trim_offset[1] = (start_row >> 1) * fetch->pitch.pitch_ch1 + (start_col >> 2) * 5 +
			(start_col & 0x3);
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		if (pframe_data != NULL)
			fetch->addr_dcam_out.addr_ch1 = pframe_data->buf.iova + fetch->pitch.pitch_ch0 * fetch->src.h;
		break;
	}
	case CAM_FULL_RGB10:
		fetch->pitch.pitch_ch0 = src->w * 8;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 8;
		break;
	case CAM_RAW_PACK_10:
	{
		uint32_t mipi_byte_info = 0;
		uint32_t mipi_word_info = 0;
		uint32_t start_col = intrim->start_x;
		uint32_t start_row = intrim->start_y;
		uint32_t end_col =  intrim->start_x + intrim->size_x - 1;
		mipi_byte_info = start_col & 0xF;
		mipi_word_info = ((end_col + 1) >> 4) * 5
			+ mipi_word_num_end[(end_col + 1) & 0xF]
			- ((start_col + 1) >> 4) * 5
			- mipi_word_num_start[(start_col + 1) & 0xF] + 1;
		fetch->mipi_byte_rel_pos = mipi_byte_info;
		fetch->mipi_word_num = mipi_word_info;
		fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(src->w, 0);
		/* same as slice starts */
		trim_offset[0] = start_row * fetch->pitch.pitch_ch0 + (start_col >> 2) * 5 + (start_col & 0x3);
		break;
	}
	default:
		pr_err("fail to get fetch format: %s\n", camport_fmt_name_get(fetch->fetch_fmt));
		break;
	}

	fetch->addr_hw.addr_ch0 = fetch->addr.addr_ch0 + trim_offset[0];
	fetch->addr_hw.addr_ch1 = fetch->addr.addr_ch1 + trim_offset[1];
	fetch->addr_hw.addr_ch2 = fetch->addr.addr_ch2 + trim_offset[2];
	pr_debug("fetch fmt %s, y_addr: %x, u_addr: %x\n", camport_fmt_name_get(fetch->fetch_fmt), fetch->addr_hw.addr_ch0, fetch->addr_hw.addr_ch1);

	return ret;
}

static int ispport_fbdfetch_yuv_get(void *cfg_in, void *cfg_out, struct camera_frame *frame)
{
	int32_t tile_col = 0, tile_row = 0;
	struct isp_fbd_yuv_info *fbd_yuv = NULL;
	struct isp_port *port = NULL;
	struct dcam_compress_cal_para cal_fbc = {0};

	if (!cfg_in || !cfg_out || !frame) {
		pr_err("fail to get valid input ptr, %p, %p\n", cfg_in, cfg_out);
		return -EFAULT;
	}

	fbd_yuv = (struct isp_fbd_yuv_info *)cfg_out;
	port = VOID_PTR_TO(cfg_in, struct isp_port);

	fbd_yuv->fetch_fbd_bypass = 0;
	fbd_yuv->slice_size = frame->in;
	fbd_yuv->trim = frame->in_crop;
	tile_col = (fbd_yuv->slice_size.w + ISP_FBD_TILE_WIDTH - 1) / ISP_FBD_TILE_WIDTH;
	tile_row =(fbd_yuv->slice_size.h + ISP_FBD_TILE_HEIGHT - 1) / ISP_FBD_TILE_HEIGHT;

	fbd_yuv->tile_num_pitch = tile_col;
	fbd_yuv->slice_start_pxl_xpt = 0;
	fbd_yuv->slice_start_pxl_ypt = 0;

	cal_fbc.data_bits = cam_data_bits(port->fmt);
	cal_fbc.fbc_info = &frame->fbc_info;
	cal_fbc.in = frame->buf.iova;
	if (port->fmt == CAM_YVU420_2FRAME)
		cal_fbc.fmt = CAM_YVU420_2FRAME;
	else if (port->fmt == CAM_YUV420_2FRAME)
		cal_fbc.fmt = CAM_YUV420_2FRAME;
	cal_fbc.height = fbd_yuv->slice_size.h;
	cal_fbc.width = fbd_yuv->slice_size.w;
	cal_fbc.out = &fbd_yuv->hw_addr;
	dcam_if_cal_compressed_addr(&cal_fbc);
	fbd_yuv->buffer_size = cal_fbc.fbc_info->buffer_size;

	/* store start address for slice use */
	fbd_yuv->frame_header_base_addr = fbd_yuv->hw_addr.addr0;
	fbd_yuv->slice_start_header_addr = fbd_yuv->frame_header_base_addr +
			((fbd_yuv->slice_start_pxl_ypt / ISP_FBD_TILE_HEIGHT) * fbd_yuv->tile_num_pitch +
			fbd_yuv->slice_start_pxl_xpt / ISP_FBD_TILE_WIDTH) * 16;
	fbd_yuv->data_bits = cal_fbc.data_bits;

	pr_debug("fetch_fbd: iova:%x, fid %u 0x%x 0x%x, 0x%x, size %u %u, channel_id:%d, tile_col:%d\n",
		 frame->buf.iova, frame->fid, fbd_yuv->hw_addr.addr0,
		 fbd_yuv->hw_addr.addr1, fbd_yuv->hw_addr.addr2,
		fbd_yuv->slice_size.w, fbd_yuv->slice_size.h, frame->channel_id, fbd_yuv->tile_num_pitch);

	return 0;
}

static int ispport_store_normal_get(struct isp_port *in_ptr,
	struct isp_hw_path_store *store_info, struct isp_port_cfg *port_cfg)
{
	int ret = 0, path_id = 0;
	struct isp_store_info *store = NULL;

	if (!in_ptr || !store_info || !port_cfg || !port_cfg->src_frame) {
		pr_err("fail to get valid input ptr %p\n", in_ptr, store_info);
		return -EFAULT;
	}

	path_id = isp_port_id_switch(in_ptr->port_id);
	store = &store_info->store;
	store->color_fmt = in_ptr->fmt;
	store->bypass = 0;
	store->endian = in_ptr->data_endian;
	if (store->color_fmt == CAM_FULL_RGB14)
		store->speed_2x = 0;
	else
		store->speed_2x = 1;

	if (cam_data_bits(in_ptr->fmt)== CAM_10_BITS)
		store->need_bwd = 0;
	else
		store->need_bwd = 1;

	store->mirror_en = 0;
	store->max_len_sel = 0;
	store->shadow_clr_sel = 1;
	store->shadow_clr = 1;
	store->store_res = 1;
	store->rd_ctrl = 0;
	store->size.w = port_cfg->src_frame->out[path_id].w;
	store->size.h = port_cfg->src_frame->out[path_id].h;
	switch (store->color_fmt) {
	case CAM_UYVY_1FRAME:
		store->pitch.pitch_ch0 = store->size.w * 2;
		store->total_size = store->size.w * store->size.h * 2;
		break;
	case CAM_YUV422_2FRAME:
	case CAM_YVU422_2FRAME:
		store->pitch.pitch_ch0 = store->size.w;
		store->pitch.pitch_ch1 = store->size.w;
		store->total_size = store->size.w * store->size.h * 2;
		break;
	case CAM_YUV420_2FRAME:
	case CAM_YVU420_2FRAME:
		store->pitch.pitch_ch0 = store->size.w;
		store->pitch.pitch_ch1 = store->size.w;
		store->total_size = store->size.w * store->size.h * 3 / 2;
		break;
	case CAM_YUV420_2FRAME_10:
	case CAM_YVU420_2FRAME_10:
		store->pitch.pitch_ch0 = store->size.w * 2;
		store->pitch.pitch_ch1 = store->size.w * 2;
		store->total_size = store->size.w * store->size.h * 3 / 2;
		break;
	case CAM_YUV420_2FRAME_MIPI:
	case CAM_YVU420_2FRAME_MIPI:
		store->pitch.pitch_ch0 = store->size.w * 10 / 8;
		store->pitch.pitch_ch1 = store->size.w * 10 / 8 ;
		store->total_size = store->size.w * store->size.h * 3 / 2;
		break;
	case CAM_YUV422_3FRAME:
		store->pitch.pitch_ch0 = store->size.w;
		store->pitch.pitch_ch1 = store->size.w / 2;
		store->pitch.pitch_ch2 = store->size.w / 2;
		store->total_size = store->size.w * store->size.h * 2;
		break;
	case CAM_YUV420_3FRAME:
		store->pitch.pitch_ch0 = store->size.w;
		store->pitch.pitch_ch1 = store->size.w / 2;
		store->pitch.pitch_ch2 = store->size.w / 2;
		store->total_size = store->size.w * store->size.h * 3 / 2;
		break;
	case CAM_FULL_RGB14:
		store->pitch.pitch_ch0 = store->size.w * 8;
		break;
	default:
		pr_err("fail to get support store fmt: %s\n", camport_fmt_name_get(store->color_fmt));
		store->pitch.pitch_ch0 = 0;
		store->pitch.pitch_ch1 = 0;
		store->pitch.pitch_ch2 = 0;
		break;
	}

	return ret;
}

static int ispport_thumb_scaler_get(struct isp_port *in_ptr, struct isp_hw_thumbscaler_info *scalerInfo, struct isp_port_cfg *port_cfg)
{
	int ret = 0, path_id = 0;
	uint32_t deci_w = 0;
	uint32_t deci_h = 0;
	uint32_t trim_w, trim_h, temp_w, temp_h;
	uint32_t offset, shift, is_yuv422 = 0;
	struct img_size src, dst;
	uint32_t align_size = 0;

	if (!in_ptr || !scalerInfo ||  !port_cfg || !port_cfg->src_frame) {
		pr_err("fail to get valid input ptr %p\n", in_ptr, scalerInfo);
		return -EFAULT;
	}

	path_id = isp_port_id_switch(in_ptr->port_id);
	scalerInfo->scaler_bypass = 0;
	scalerInfo->frame_deci = 0;
	/* y factor & deci */
	src.w = in_ptr->trim.size_x;
	src.h = in_ptr->trim.size_y;
	dst = port_cfg->src_frame->out[path_id];
	ret = isp_drv_trim_deci_info_cal(src.w, dst.w, &temp_w, &deci_w);
	ret |= isp_drv_trim_deci_info_cal(src.h, dst.h, &temp_h, &deci_h);
	if (deci_w == 0 || deci_h == 0)
		return -EINVAL;
	if (ret) {
		pr_err("fail to set thumbscaler ydeci. src %d %d, dst %d %d\n",
					src.w, src.h, dst.w, dst.h);
		return ret;
	}

	scalerInfo->y_deci.deci_x = deci_w;
	scalerInfo->y_deci.deci_y = deci_h;
	if (deci_w > 1)
		scalerInfo->y_deci.deci_x_eb = 1;
	else
		scalerInfo->y_deci.deci_x_eb = 0;
	if (deci_h > 1)
		scalerInfo->y_deci.deci_y_eb = 1;
	else
		scalerInfo->y_deci.deci_y_eb = 0;
	align_size = deci_w * ISP_PIXEL_ALIGN_WIDTH;
	trim_w = (temp_w) & ~(align_size - 1);
	align_size = deci_h * ISP_PIXEL_ALIGN_HEIGHT;
	trim_h = (temp_h) & ~(align_size - 1);
	scalerInfo->y_factor_in.w = trim_w / deci_w;
	scalerInfo->y_factor_in.h = trim_h / deci_h;
	scalerInfo->y_factor_out = port_cfg->src_frame->out[path_id];

	if ((in_ptr->fmt == CAM_YUV422_2FRAME) || (in_ptr->fmt == CAM_YVU422_2FRAME))
		is_yuv422 = 1;

	/* uv factor & deci, input: yuv422(isp pipeline format) */
	shift = is_yuv422 ? 0 : 1;
	scalerInfo->uv_deci.deci_x = deci_w;
	scalerInfo->uv_deci.deci_y = deci_h;
	if (deci_w > 1)
		scalerInfo->uv_deci.deci_x_eb = 1;
	else
		scalerInfo->uv_deci.deci_x_eb = 0;
	if (deci_h > 1)
		scalerInfo->uv_deci.deci_y_eb = 1;
	else
		scalerInfo->uv_deci.deci_y_eb = 0;
	trim_w >>= 1;
	scalerInfo->uv_factor_in.w = trim_w / deci_w;
	scalerInfo->uv_factor_in.h = trim_h / deci_h;
	scalerInfo->uv_factor_out.w = dst.w / 2;
	scalerInfo->uv_factor_out.h = dst.h >> shift;

	scalerInfo->src0.w = in_ptr->trim.size_x;
	scalerInfo->src0.h = in_ptr->trim.size_y;

	/* y trim */
	trim_w = scalerInfo->y_factor_in.w * scalerInfo->y_deci.deci_x;
	offset = (in_ptr->trim.size_x - trim_w) / 2;
	scalerInfo->y_trim.start_x = in_ptr->trim.start_x + offset;
	scalerInfo->y_trim.size_x = trim_w;

	trim_h = scalerInfo->y_factor_in.h * scalerInfo->y_deci.deci_y;
	offset = (in_ptr->trim.size_y - trim_h) / 2;
	scalerInfo->y_trim.start_y = in_ptr->trim.start_y + offset;
	scalerInfo->y_trim.size_y = trim_h;

	scalerInfo->y_src_after_deci = scalerInfo->y_factor_in;
	scalerInfo->y_dst_after_scaler = scalerInfo->y_factor_out;

	/* uv trim */
	trim_w = scalerInfo->uv_factor_in.w * scalerInfo->uv_deci.deci_x;
	offset = (in_ptr->trim.size_x / 2 - trim_w) / 2;
	scalerInfo->uv_trim.start_x = in_ptr->trim.start_x / 2 + offset;
	scalerInfo->uv_trim.size_x = trim_w;

	trim_h = scalerInfo->uv_factor_in.h * scalerInfo->uv_deci.deci_y;
	offset = (in_ptr->trim.size_y - trim_h) / 2;
	scalerInfo->uv_trim.start_y = in_ptr->trim.start_y + offset;
	scalerInfo->uv_trim.size_y = trim_h;

	scalerInfo->uv_src_after_deci = scalerInfo->uv_factor_in;
	scalerInfo->uv_dst_after_scaler = scalerInfo->uv_factor_out;
	scalerInfo->odata_mode = is_yuv422 ? 0x00 : 0x01;

	scalerInfo->y_deci.deci_x = isp_drv_deci_factor_cal(scalerInfo->y_deci.deci_x);
	scalerInfo->y_deci.deci_y = isp_drv_deci_factor_cal(scalerInfo->y_deci.deci_y);
	scalerInfo->uv_deci.deci_x = isp_drv_deci_factor_cal(scalerInfo->uv_deci.deci_x);
	scalerInfo->uv_deci.deci_y = isp_drv_deci_factor_cal(scalerInfo->uv_deci.deci_y);

	/* N6pro thumbscaler calculation regulation */
	if (scalerInfo->thumbscl_cal_version == 1) {
		scalerInfo->y_init_phase.w = scalerInfo->y_dst_after_scaler.w / 2;
		scalerInfo->y_init_phase.h = scalerInfo->y_dst_after_scaler.h / 2;
		scalerInfo->uv_src_after_deci.w = scalerInfo->y_src_after_deci.w / 2;
		scalerInfo->uv_src_after_deci.h = scalerInfo->y_src_after_deci.h;
		scalerInfo->uv_dst_after_scaler.w = scalerInfo->y_dst_after_scaler.w / 2;
		scalerInfo->uv_dst_after_scaler.h = scalerInfo->y_dst_after_scaler.h / 2;
		scalerInfo->uv_trim.size_x = scalerInfo->y_trim.size_x / 2;
		scalerInfo->uv_trim.size_y = scalerInfo->y_trim.size_y / 2;
		scalerInfo->uv_init_phase.w = scalerInfo->uv_dst_after_scaler.w / 2;
		scalerInfo->uv_init_phase.h = scalerInfo->uv_dst_after_scaler.h / 2;
		scalerInfo->uv_factor_in.w = scalerInfo->y_factor_in.w / 2;
		scalerInfo->uv_factor_in.h = scalerInfo->y_factor_in.h / 2;
		scalerInfo->uv_factor_out.w = scalerInfo->y_factor_out.w / 2;
		scalerInfo->uv_factor_out.h = scalerInfo->y_factor_out.h / 2;
	}

	pr_debug("deciY %d %d, Yfactor (%d %d) => (%d %d) ytrim (%d %d %d %d)\n",
		scalerInfo->y_deci.deci_x, scalerInfo->y_deci.deci_y,
		scalerInfo->y_factor_in.w, scalerInfo->y_factor_in.h,
		scalerInfo->y_factor_out.w, scalerInfo->y_factor_out.h,
		scalerInfo->y_trim.start_x, scalerInfo->y_trim.start_y,
		scalerInfo->y_trim.size_x, scalerInfo->y_trim.size_y);
	pr_debug("deciU %d %d, Ufactor (%d %d) => (%d %d), Utrim (%d %d %d %d)\n",
		scalerInfo->uv_deci.deci_x, scalerInfo->uv_deci.deci_y,
		scalerInfo->uv_factor_in.w, scalerInfo->uv_factor_in.h,
		scalerInfo->uv_factor_out.w, scalerInfo->uv_factor_out.h,
		scalerInfo->uv_trim.start_x, scalerInfo->uv_trim.start_y,
		scalerInfo->uv_trim.size_x, scalerInfo->uv_trim.size_y);

	pr_debug("my frameY: %d %d %d %d\n",
		scalerInfo->y_src_after_deci.w, scalerInfo->y_src_after_deci.h,
		scalerInfo->y_dst_after_scaler.w,
		scalerInfo->y_dst_after_scaler.h);
	pr_debug("my frameU: %d %d %d %d\n",
		scalerInfo->uv_src_after_deci.w,
		scalerInfo->uv_src_after_deci.h,
		scalerInfo->uv_dst_after_scaler.w,
		scalerInfo->uv_dst_after_scaler.h);

	pr_debug("init_phase: Y(%d %d), UV(%d %d)\n",
		scalerInfo->y_init_phase.w, scalerInfo->y_init_phase.h,
		scalerInfo->uv_init_phase.w, scalerInfo->uv_init_phase.h);

	return ret;
}

static int ispport_fetch_pipeinfo_get(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	struct isp_pipe_info *pipe_in = NULL;
	uint32_t ret = 0;

	pipe_in = port_cfg->pipe_info;
	pipe_in->fetch.ctx_id = port_cfg->cfg_id;
	pipe_in->fetch.sec_mode = port_cfg->sec_mode;
	if (port->fetch_path_sel == ISP_FETCH_PATH_NORMAL)
		pipe_in->fetch_fbd_yuv.fetch_fbd_bypass = 1;
	ret = ispport_fetch_normal_get(port, &pipe_in->fetch, port_cfg->src_frame);
	if (ret) {
		pr_err("fail to get pipe fetch info\n");
		return -EFAULT;
	}
	port_cfg->src_size = port_cfg->src_frame->in;
	port_cfg->src_crop = port_cfg->src_frame->in_crop;
	return ret;
}

static int ispport_store_pipeinfo_get(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	struct isp_pipe_info *pipe_in = NULL;
	uint32_t ret = 0, path_id = isp_port_id_switch(port->port_id);

	pipe_in = port_cfg->pipe_info;
	pipe_in->store[path_id].ctx_id = port_cfg->cfg_id;
	pipe_in->store[path_id].spath_id = path_id;
	ret = ispport_store_normal_get(port, &pipe_in->store[path_id], port_cfg);
	if (ret) {
		pr_err("fail to get pipe store normal info\n");
		return -EFAULT;
	}

	pipe_in->scaler[path_id].ctx_id = port_cfg->cfg_id;
	pipe_in->scaler[path_id].spath_id = path_id;
	pipe_in->scaler[path_id].src.w = port_cfg->src_crop.size_x;
	pipe_in->scaler[path_id].src.h = port_cfg->src_crop.size_y;
	if (port_cfg->src_frame->pyr_status == OFFLINE_DEC_ON) {
		pipe_in->scaler[path_id].in_trim.start_x = 0;
		pipe_in->scaler[path_id].in_trim.start_y = 0;
		pipe_in->scaler[path_id].in_trim.size_x = port_cfg->src_crop.size_x;
		pipe_in->scaler[path_id].in_trim.size_y = port_cfg->src_crop.size_y;
	} else {
		pipe_in->scaler[path_id].in_trim = port_cfg->src_frame->out_crop[path_id];
	}

	port->scaler_coeff_ex = port_cfg->scaler_coeff_ex;
	port->scaler_bypass_ctrl = port_cfg->scaler_bypass_ctrl;
	ret = ispport_scaler_get(port, &pipe_in->scaler[path_id], port_cfg);
	if (ret) {
		pr_err("fail to get pipe path scaler info\n");
		return -EFAULT;
	}
	return ret;
}

static int ispport_thumb_pipeinfo_get(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	struct isp_pipe_info *pipe_in = NULL;
	uint32_t ret = 0, path_id = isp_port_id_switch(port->port_id);

	pipe_in = port_cfg->pipe_info;
	pipe_in->store[path_id].ctx_id = port_cfg->cfg_id;
	pipe_in->store[path_id].spath_id = path_id;
	ret = ispport_store_normal_get(port, &pipe_in->store[path_id], port_cfg);
	if (ret) {
		pr_err("fail to get pipe store normal info\n");
		return -EFAULT;
	}

	pipe_in->thumb_scaler.idx = port_cfg->cfg_id;
	if (port->hw->ip_isp->thumb_scaler_cal_version)
		pipe_in->thumb_scaler.thumbscl_cal_version = 1;
	ret = ispport_thumb_scaler_get(port, &pipe_in->thumb_scaler, port_cfg);
	if (ret) {
		pr_err("fail to get pipe thumb scaler info\n");
		return -EFAULT;
	}
	return 0;
}

static int ispport_pipeinfo_get(struct isp_port *port, void *param)
{
	struct isp_port_cfg *port_cfg = NULL;

	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	if (port->type == PORT_TRANSFER_IN) {
		if (port_cfg->src_frame->is_compressed) {
			port->fetch_path_sel = ISP_FETCH_PATH_FBD;
			port_cfg->pipe_info->fetch_fbd_yuv.ctx_id = port_cfg->cfg_id;
			ispport_fbdfetch_yuv_get(port, &port_cfg->pipe_info->fetch_fbd_yuv, port_cfg->src_frame);
		}
		ispport_fetch_pipeinfo_get(port, port_cfg);
		port_cfg->in_fmt = port->fmt;
	}

	if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->is_work) > 0) {
		switch (port->port_id) {
		case PORT_PRE_OUT:
		case PORT_VID_OUT:
		case PORT_CAP_OUT:
			ispport_store_pipeinfo_get(port, port_cfg);
			break;
		case PORT_THUMB_OUT:
			ispport_thumb_pipeinfo_get(port, port_cfg);
			break;
		default:
			pr_err("fail to get port_id:%d or type:%d\n", port->port_id, port->type);
		}
	}
	return 0;
}

static int ispport_port_slice_need(struct isp_port *port, void *param)
{
	if (port->type == PORT_TRANSFER_IN) {
		if (port->trim.size_x > g_camctrl.isp_linebuf_len)
			return 1;
	}
	if (port->type == PORT_TRANSFER_OUT) {
		if (port->size.w > g_camctrl.isp_linebuf_len)
			return 1;
	}
	return 0;
}

static int ispport_blksize_get(struct isp_port *port, void *param)
{
	struct isp_port_blksize_desc *size_desc = NULL;

	size_desc = VOID_PTR_TO(param, struct isp_port_blksize_desc);
	if (port->original.src_size.w > 0 && size_desc->ch_id != CAM_CH_CAP) {
		/* for input scaled image */
		size_desc->new_width = port->original.dst_size.w;
		size_desc->new_height = port->original.dst_size.h;
		size_desc->old_width = port->original.src_trim.size_x;
		size_desc->old_height = port->original.src_trim.size_y;
	} else {
		size_desc->old_width = port->trim.size_x;
		size_desc->old_height = port->trim.size_y;
		size_desc->new_width = size_desc->old_width;
		size_desc->new_height = size_desc->old_height;
	}
	size_desc->sn_size = port->sn_size;
	size_desc->src_size = port->size;
	return 0;
}

static int ispport_start_error(struct isp_port *port, void *param)
{
	struct camera_frame *pframe = NULL;
	struct camera_frame *pframe_data = NULL;
	struct isp_port_cfg *port_cfg = NULL;

	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	if (port->type == PORT_TRANSFER_IN) {
		if (port_cfg->out_buf_clear)
			pframe = cam_queue_dequeue(&port->out_buf_queue, struct camera_frame, list);
		else
			pframe = cam_queue_dequeue_tail(&port->result_queue, struct camera_frame, list);
		pframe_data = (struct camera_frame *)pframe->pframe_data;
		if (pframe) {
			if (pframe->is_reserved)
				port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
			else {
				if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
					cam_buf_iommu_unmap(&pframe->buf);
				isp_node_offline_pararm_free(pframe->param_data);
				pframe->param_data = NULL;
				if (port_cfg->param_share_queue)
					cam_queue_blk_param_unbind(port_cfg->param_share_queue, pframe);
				port->data_cb_func(CAM_CB_ISP_RET_SRC_BUF, pframe, port->data_cb_handle);
			}
		}
		if (pframe_data != NULL) {
			if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
				cam_buf_iommu_unmap(&pframe_data->buf);
			port->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe_data, port->data_cb_handle);
		}
	}

	if (port->type == PORT_TRANSFER_OUT) {
		pframe = cam_queue_dequeue_tail(&port->result_queue, struct camera_frame, list);
		if (pframe) {
			pr_debug("port %d frame %px, is reserve %d\n", port->port_id, pframe, pframe->is_reserved);
			/* ret frame to original queue */
			if (pframe->is_reserved)
				port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
			else
				cam_queue_enqueue(&port->out_buf_queue, &pframe->list);
		}
	}
	return 0;
}

static int ispport_video_slowmotion(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	struct camera_frame *out_frame = NULL;
	uint32_t hw_path_id = isp_port_id_switch(port->port_id);
	uint32_t ret = 0;
	if (port_cfg->vid_valid) {
		out_frame = cam_queue_dequeue(&port->out_buf_queue, struct camera_frame, list);
		pr_debug("vid use valid %px\n", out_frame);
	}

	if (out_frame == NULL)
		out_frame = ispport_reserved_buf_get(port->resbuf_get_cb, port->resbuf_cb_data, port);
	else if (port_cfg->uinfo->stage_a_valid_count)
		ispport_slw960_frame_num_update(&port_cfg->uinfo->slw_960desc, &port_cfg->uinfo->frame_num);

	if (out_frame->is_reserved == 0) {
		if (out_frame->buf.mfd == port->reserved_buf_fd) {
			out_frame->buf.size = port->reserve_buf_size;
			pr_debug("out_frame->buf.size = %d, path->reserve_buf_size = %d",
				(int)out_frame->buf.size, (int)port->reserve_buf_size);
			ret = cam_buf_iommu_single_page_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
		} else
			ret = cam_buf_iommu_map(&out_frame->buf, CAM_IOMMUDEV_ISP);
		pr_debug("map output buffer %08x\n", (uint32_t)out_frame->buf.iova);
		if (ret) {
			cam_queue_enqueue(&port->out_buf_queue, &out_frame->list);
			out_frame = NULL;
			pr_err("fail to map isp iommu buf.\n");
			return -EINVAL;
		}
	}
	ret = ispport_store_frameproc(port, out_frame, port_cfg);

	port_cfg->slw->store[hw_path_id] = port_cfg->pipe_info->store[hw_path_id].store;
	return 0;
}

static int ispport_preview_slowmotion(struct isp_port *port, struct isp_port_cfg *port_cfg)
{
	struct camera_frame *out_frame = NULL;
	uint32_t hw_path_id = isp_port_id_switch(port->port_id);
	uint32_t ret = 0;
	out_frame = ispport_reserved_buf_get(port->resbuf_get_cb, port->resbuf_cb_data, port);
	ret = ispport_store_frameproc(port, out_frame, port_cfg);
	port_cfg->slw->store[hw_path_id] = port_cfg->pipe_info->store[hw_path_id].store;
	return ret;
}

static int ispport_slowmotion(struct isp_port *port, void *param)
{
	struct isp_port_cfg *port_cfg = NULL;
	uint32_t ret = 0;
	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	if (port->type == PORT_TRANSFER_IN) {
		ret = ispport_fetch_frame_cycle(port, port_cfg);
		port_cfg->src_frame->width = port->size.w;
		port_cfg->src_frame->height = port->size.h;
		ret = isp_hwctx_fetch_frm_set(port_cfg->dev, &port_cfg->pipe_info->fetch, port_cfg->src_frame);
	}

	if (port->type == PORT_TRANSFER_OUT) {
		switch (port->port_id) {
		case PORT_PRE_OUT:
		case PORT_CAP_OUT:
			ret = ispport_preview_slowmotion(port, port_cfg);
			break;
		case PORT_VID_OUT:
			ret = ispport_video_slowmotion(port, port_cfg);
			break;
		default:
			pr_err("fail to get port_id:%d or type:%d\n", port->port_id, port->type);
		}
	}
	return ret;
}

static int ispport_fast_stop(struct isp_port *port, void *param)
{
	struct isp_port_cfg *port_cfg = NULL;
	struct camera_frame *pframe = NULL;
	uint32_t ret = 0;

	port_cfg = VOID_PTR_TO(param, struct isp_port_cfg);
	if (port->type == PORT_TRANSFER_IN) {
		if (*(port_cfg->faststop) == 1)
			ispport_start_error(port, port_cfg);

		if (cam_queue_cnt_get(&port->out_buf_queue) == 0 && cam_queue_cnt_get(&port->result_queue) == 0) {
			*(port_cfg->faststop) = 0;
			complete(port_cfg->faststop_done);
		} else
			*(port_cfg->faststop) = 1;
	}

	if (port->type == PORT_TRANSFER_OUT) {
		pframe = cam_queue_dequeue(&port->result_queue, struct camera_frame, list);

		if (!pframe) {
			pr_err("fail to get frame from queue. port:%d\n", port->port_id);
			return 0;
		}
		if (unlikely(pframe->is_reserved)) {
			port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
		} else if (pframe->state == ISP_STREAM_POST_PROC) {
			cam_buf_iommu_unmap(&pframe->buf);
		} else {
			if (pframe->buf.mfd == port->reserved_buf_fd) {
				pframe->buf.size = port->reserve_buf_size;
				pr_info("pframe->buf.size = %d, path->reserve_buf_size = %d",
					(int)pframe->buf.size, (int)port->reserve_buf_size);
			}
			cam_buf_iommu_unmap(&pframe->buf);
			cam_queue_enqueue_front(&port->out_buf_queue, &pframe->list);
		}
	}
	return ret;
}

static int ispport_cfg_callback(void *param, uint32_t cmd, void *handle)
{
	int ret = 0;
	struct isp_port *port = NULL;

	port = VOID_PTR_TO(handle, struct isp_port);
	switch (cmd) {
	case ISP_PORT_SIZE_UPDATA:
		ret = ispport_size_update(port, param);
		break;
	case ISP_PORT_IRQ_POSTPORC:
		ret = ispport_port_postproc(port, param);
		break;
	case ISP_PORT_SLICE_NEED:
		ret = ispport_port_slice_need(port, param);
		break;
	case ISP_PORT_UFRAME_FID_GET:
		ret = ispport_uframe_fid_get(port, param);
		break;
	case ISP_PORT_FRAME_CYCLE:
		ret = ispport_frame_cycle(port, param);
		break;
	case ISP_PORT_PIPEINFO_GET:
		ret = ispport_pipeinfo_get(port, param);
		break;
	case ISP_PORT_BLKSIZE_GET:
		ret = ispport_blksize_get(port, param);
		break;
	case ISP_PORT_START_ERROR:
		ret = ispport_start_error(port, param);
		break;
	case ISP_PORT_SLOWMOTION:
		ret = ispport_slowmotion(port, param);
		break;
	case ISP_PORT_FAST_STOP:
		ret = ispport_fast_stop(port, param);
		break;
	default:
		pr_err("fail to support port type %d\n", cmd);
		ret = -EFAULT;
		break;
	}
	return ret;
}

uint32_t isp_port_id_switch(uint32_t port_id)
{
	uint32_t hw_path_id = -1;
	switch (port_id) {
	case PORT_PRE_OUT:
		hw_path_id = ISP_SPATH_CP;
		break;
	case PORT_VID_OUT:
		hw_path_id = ISP_SPATH_VID;
		break;
	case PORT_CAP_OUT:
		hw_path_id = ISP_SPATH_CP;
		break;
	case PORT_THUMB_OUT:
		hw_path_id = ISP_SPATH_FD;
		break;
	default:
		pr_err("fail to get vaild port_id:%d", port_id);
	}
	return hw_path_id;
}

int isp_port_param_cfg(void *handle, enum cam_port_cfg_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_port *port = NULL;

	port = VOID_PTR_TO(handle, struct isp_port);
	switch (cmd) {
	case PORT_SIZE_CFG_SET:
		if (!ispport_size_check(port, param))
			ispport_size_cfg(port, param);
		break;
	case PORT_UFRAME_CFG:
		port->uframe_sync |= *(uint32_t *)param;
		break;
	case PORT_BUFFER_CFG_SET:
		ret = ispport_buf_set(port, param);
		break;
	case PORT_RESERVERD_BUF_CFG:
		ispport_reserved_buf_set(port, param);
		break;
	case PORT_CFG_BASE:
		ret = ispport_base_cfg(port, param);
		break;
	default:
		pr_err("fail to support port type %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

void *isp_port_get(uint32_t port_id, struct isp_port_desc *port_desc)
{
	struct isp_port *port = NULL;

	if (!port_desc) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	if (*port_desc->port_dev == NULL) {
		port = cam_buf_kernel_sys_vzalloc(sizeof(struct isp_port));
		if (!port) {
			pr_err("fail to get valid isp port %d\n", port_id);
			return NULL;
		}
	} else {
		port = *port_desc->port_dev;
		pr_debug("isp port has been alloc %p %d\n", port, port_id);
		goto exit;
	}

	if (port_desc->is_high_fps == 0) {
		cam_queue_init(&port->result_queue, ISP_RESULT_Q_LEN, ispport_frame_ret);
		cam_queue_init(&port->out_buf_queue, ISP_OUT_BUF_Q_LEN, ispport_frame_ret);
	} else {
		cam_queue_init(&port->result_queue, ISP_SLW_IN_Q_LEN, ispport_frame_ret);
		cam_queue_init(&port->out_buf_queue, ISP_OUT_BUF_Q_LEN, ispport_frame_ret);
	}

	port->resbuf_get_cb = port_desc->resbuf_get_cb;
	port->resbuf_cb_data = port_desc->resbuf_cb_data;
	port->data_cb_func = port_desc->data_cb_func;
	port->data_cb_handle = port_desc->data_cb_handle;
	port->port_cfg_cb_func = ispport_cfg_callback;
	port->type = port_desc->transfer_type;
	port->hw = port_desc->hw;
	if (port_desc->transfer_type == PORT_TRANSFER_IN) {
		port->fmt = port_desc->in_fmt;
		port->bayer_pattern = port_desc->bayer_pattern;
		port->pyr_out_fmt = port_desc->pyr_out_fmt;
		port->store_3dnr_fmt= port_desc->store_3dnr_fmt;
		port->sn_size = port_desc->sn_size;
		port->fetch_path_sel = port_desc->fetch_path_sel;
	}
	if (port_desc->transfer_type == PORT_TRANSFER_OUT) {
		port->fmt = port_desc->out_fmt;
		port->data_endian = port_desc->endian;
		port->regular_mode = port_desc->regular_mode;
		port->size = port_desc->output_size;
	}

	port->port_id = port_id;
	*port_desc->port_dev = port;
	pr_info("port id %d node_dev %px port_desc->depend_type %d\n", port_id, *port_desc->port_dev, port_desc->depend_type);

exit:
	port->data_cb_handle = port_desc->data_cb_handle;
	atomic_set(&port->is_work, 1);
	if (port_desc->depend_type == PORT_SLAVE)
		atomic_set(&port->is_work, 0);
	atomic_inc(&port->user_cnt);
	return port;
}

void isp_port_put(struct isp_port *port)
{
	if (!port) {
		pr_err("fail to get invalid port ptr\n");
		return;
	}

	if (atomic_dec_return(&port->user_cnt) == 0) {
		cam_queue_clear(&port->result_queue, struct camera_frame, list);
		cam_queue_clear(&port->out_buf_queue, struct camera_frame, list);
		port->resbuf_get_cb = NULL;
		port->data_cb_func = NULL;
		port->port_cfg_cb_func = NULL;
		cam_buf_kernel_sys_vfree(port);
		port = NULL;
		pr_info("isp portfree success\n");
	}

}
