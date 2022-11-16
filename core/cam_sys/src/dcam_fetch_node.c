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

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include "dcam_fetch_node.h"
#include "dcam_interface.h"
#include "dcam_core.h"
#include "cam_node.h"
#include "cam_debugger.h"
#include "dcam_slice.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_FETCH_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

static void dcamfetch_nr3_mv_get(struct dcam_hw_context *hw_ctx, struct camera_frame *frame)
{
	int i = 0;

	i = frame->fid % DCAM_NR3_MV_MAX;
	frame->nr3_me.mv_x = hw_ctx->nr3_mv_ctrl[i].mv_x;
	frame->nr3_me.mv_y = hw_ctx->nr3_mv_ctrl[i].mv_y;
	frame->nr3_me.project_mode = hw_ctx->nr3_mv_ctrl[i].project_mode;
	frame->nr3_me.sub_me_bypass = hw_ctx->nr3_mv_ctrl[i].sub_me_bypass;
	frame->nr3_me.src_height = hw_ctx->nr3_mv_ctrl[i].src_height;
	frame->nr3_me.src_width = hw_ctx->nr3_mv_ctrl[i].src_width;
	frame->nr3_me.valid = hw_ctx->nr3_mv_ctrl[i].valid;
	pr_debug("dcam%d fid %d, valid %d, x %d, y %d, w %u, h %u\n",
		hw_ctx->hw_ctx_id, frame->fid, frame->nr3_me.valid,
		frame->nr3_me.mv_x, frame->nr3_me.mv_y,
		frame->nr3_me.src_width, frame->nr3_me.src_height);
}

static struct camera_frame *dcamfetch_frame_prepare(struct dcam_online_node *node,
			struct dcam_online_port *dcam_port, struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	struct camera_frame *frame = NULL;
 	timespec *ts = NULL;
	uint32_t dev_fid = 0, path_id = 0;

	if (atomic_read(&dcam_port->set_frm_cnt) <= 1) {
		path_id = dcamonline_portid_convert_to_pathid(dcam_port->port_id);
		pr_warn("warning: %s cnt %d, deci %u, out %u, result %u\n",
			cam_port_name_get(dcam_port->port_id),
			atomic_read(&dcam_port->set_frm_cnt), hw_ctx->hw_path[path_id].frm_deci,
			cam_buf_manager_pool_cnt(&dcam_port->unprocess_pool),
			cam_buf_manager_pool_cnt(&dcam_port->result_pool));
		return NULL;
	}

	if (atomic_read(&node->state) != STATE_RUNNING) {
		pr_warn("warning: DCAM%u state %d %s\n", hw_ctx->hw_ctx_id,
			atomic_read(&node->state), cam_port_name_get(dcam_port->port_id));
		return NULL;
	}

	frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, NULL);
	if (!frame) {
		pr_err("fail to available output buffer DCAM%u %s\n",
			node->hw_ctx_id, cam_port_name_get(dcam_port->port_id));
		return NULL;
	}
	atomic_dec(&dcam_port->set_frm_cnt);

	if (unlikely(frame->is_reserved)) {
		pr_info_ratelimited("DCAM%u %s use reserved buffer\n",
			hw_ctx->hw_ctx_id, cam_port_name_get(dcam_port->port_id));
		dcam_online_port_reserved_buf_set(dcam_port, frame);
		return NULL;
	}

	/* assign same SOF time here for each port */
	dev_fid = frame->fid - hw_ctx->base_fid;
	ts = &node->frame_ts[tsid(dev_fid)];
	frame->sensor_time.tv_sec = ts->tv_sec;
	frame->sensor_time.tv_usec = ts->tv_nsec / NSEC_PER_USEC;
	frame->boot_sensor_time = node->frame_ts_boot[tsid(dev_fid)];
	if (dev_fid == 0)
		frame->frame_interval_time = 0;
	else
		frame->frame_interval_time = frame->boot_sensor_time - node->frame_ts_boot[tsid(dev_fid - 1)];

	pr_debug("DCAM%u %s: TX DONE, fid %u, %lld\n", hw_ctx->hw_ctx_id,
		cam_port_name_get(dcam_port->port_id), frame->fid, frame->frame_interval_time);

	if (!frame->boot_sensor_time) {
		pr_info("DCAM%u %s fid %u invalid 0 timestamp\n",
			hw_ctx->hw_ctx_id, cam_port_name_get(dcam_port->port_id), frame->fid);
		ret = dcamonline_port_buffer_cfg(dcam_port, frame);
		if (ret) {
			pr_err("fail to set dcam online port outbuf_queue\n");
		}
		frame = NULL;
	}

	return frame;
}

static void dcamfetch_frame_dispatch(void *param, void *handle)
{
	struct dcam_online_node *node = NULL;
	struct dcam_online_port *dcam_port = NULL;
	struct dcam_irq_proc *irq_proc = NULL;
	struct camera_frame *frame = NULL;
	uint32_t ret = 0;
	timespec cur_ts = {0};

	node = (struct dcam_online_node *)handle;
	irq_proc = (struct dcam_irq_proc *)param;
	frame = (struct camera_frame *)irq_proc->param;

	if (unlikely(!node || !frame || !is_port_id(irq_proc->dcam_port_id))) {
		pr_err("fail to get valid param %px %px %d\n", node, frame, irq_proc->dcam_port_id);
		return;
	}

	ktime_get_ts(&cur_ts);
	frame->time.tv_sec = cur_ts.tv_sec;
	frame->time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	frame->boot_time = ktime_get_boottime();
	pr_debug("DCAM%u port %d: time %06d.%06d\n", node->hw_ctx_id, irq_proc->dcam_port_id,
		(int)frame->time.tv_sec, (int)frame->time.tv_usec);

	dcam_port = dcam_online_node_port_get(node, irq_proc->dcam_port_id);
	if (dcam_port->isp_updata) {
		struct isp_offline_param *cur = NULL;
		pr_debug("next %p,  prev %p\n", frame->param_data, dcam_port->isp_updata);
		if (frame->param_data) {
			cur = (struct isp_offline_param *)frame->param_data;
			cur->prev = dcam_port->isp_updata;
		} else {
			frame->param_data = dcam_port->isp_updata;
		}
		dcam_port->isp_updata = NULL;
		pr_debug("cur %p\n", frame->param_data);
	}
	frame->link_from.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	frame->link_from.port_id = irq_proc->dcam_port_id;
	if (irq_proc->type == CAM_CB_DCAM_DATA_DONE) {
		ret = cam_buf_manager_buf_status_change(&frame->buf, CAM_BUF_WITH_ION, CAM_IOMMUDEV_DCAM);
		if (ret) {
			pr_err("fail ro unmap path:%d buffer.\n", irq_proc->dcam_port_id);
			ret = dcam_port->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, frame, dcam_port->data_cb_handle);
			if (ret) {
				pr_err("fail to enqueue path:%d frame, q_cnt:%d.\n", irq_proc->dcam_port_id, cam_buf_manager_pool_cnt(&dcam_port->unprocess_pool));
				cam_buf_destory(frame);
			}
			return;
		}
	}
	if (dcam_port->data_cb_func)
		dcam_port->data_cb_func(irq_proc->type, frame, dcam_port->data_cb_handle);
	else
		node->data_cb_func(irq_proc->type, frame, node->data_cb_handle);
}

static int dcamfetch_done_proc(void *param, void *handle, struct dcam_hw_context *hw_ctx)
{
	int ret = 0, cnt = 0, i = 0;
	struct dcam_fetch_node *node = NULL;
	struct dcam_online_node *online_node = NULL;
	struct dcam_irq_proc *irq_proc = NULL;
	struct camera_frame *frame = NULL;
	struct dcam_online_port *dcam_port = NULL;
	uint32_t mv_ready = 0;
	struct camera_buf_get_desc buf_desc = {0};
	struct dcam_offline_slice_info *slice_info = NULL;

	node = (struct dcam_fetch_node *)handle;
	irq_proc = (struct dcam_irq_proc *)param;
	online_node = &node->online_node;
	slice_info = &node->hw_ctx->slice_info;

	list_for_each_entry(dcam_port, &online_node->port_queue.head, list) {
		if (dcam_port->port_id != irq_proc->dcam_port_id)
			continue;
		switch (irq_proc->dcam_port_id) {
		case PORT_FULL_OUT:
			if (slice_info->slice_num > 0) {
				pr_debug("dcam%d fetch slice%d done.\n", online_node->hw_ctx_id,
						(slice_info->slice_num - slice_info->slice_count));
				complete(&node->slice_done);
				if (slice_info->slice_count > 0)
					slice_info->slice_count--;
				if (slice_info->slice_count > 0)
					return 0;
			}
			if (online_node->is_3dnr) {
				mv_ready = online_node->nr3_me.full_path_mv_ready;
				if (mv_ready == 0) {
					online_node->nr3_me.full_path_cnt++;
					return ret;
				} else if (mv_ready == 1) {
					frame = dcamfetch_frame_prepare(online_node, dcam_port, hw_ctx);
					if (frame == NULL)
						return ret;

					dcamfetch_nr3_mv_get(hw_ctx, frame);
					online_node->nr3_me.full_path_mv_ready = 0;
					online_node->nr3_me.full_path_cnt = 0;
					pr_debug("dcam %d,fid %d mv_x %d mv_y %d", online_node->hw_ctx_id, frame->fid, frame->nr3_me.mv_x, frame->nr3_me.mv_y);
					irq_proc->param = frame;
					irq_proc->type = CAM_CB_DCAM_DATA_DONE;
					dcamfetch_frame_dispatch(irq_proc, online_node);
				}
			} else {
				if ((frame = dcamfetch_frame_prepare(online_node, dcam_port, hw_ctx))) {
					if (online_node->is_4in1)
						frame->irq_type = CAMERA_IRQ_4IN1_DONE;
					irq_proc->param = frame;
					irq_proc->type = CAM_CB_DCAM_DATA_DONE;
					dcamfetch_frame_dispatch(irq_proc, online_node);
				}
			}
			frame = cam_queue_dequeue(&node->proc_queue, struct camera_frame, list);
			if (frame) {
				cam_buf_iommu_unmap(&frame->buf);
				if (online_node->data_cb_func)
					online_node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, frame, online_node->data_cb_handle);
			}
			complete(&node->frm_done);
			break;
		case PORT_BIN_OUT:
			if (!node->virtualsensor_cap_en && slice_info->slice_num > 0) {
				pr_debug("dcam%d online slice%d done.\n", online_node->hw_ctx_id,
						(slice_info->slice_num - slice_info->slice_count));
				complete(&node->slice_done);
				if (slice_info->slice_count > 0)
					slice_info->slice_count--;
				if (slice_info->slice_count > 0)
					return 0;
			}
			cnt = atomic_read(&dcam_port->set_frm_cnt);
			if (cnt <= online_node->slowmotion_count) {
				pr_warn("warning: DCAM%u BIN cnt %d, deci %u, out %u, result %u\n",
					hw_ctx->hw_ctx_id, cnt, hw_ctx->hw_path[DCAM_PATH_BIN].frm_deci,
					cam_buf_manager_pool_cnt(&dcam_port->unprocess_pool),
					cam_buf_manager_pool_cnt(&dcam_port->result_pool));
				return ret;
			}
			buf_desc.q_ops_cmd = CAM_QUEUE_DEQ_PEEK;
			frame = cam_buf_manager_buf_dequeue(&dcam_port->result_pool, &buf_desc);
			if (unlikely(!frame) || (frame->need_pyr_rec && !hw_ctx->dec_all_done))
				return 0;
			hw_ctx->dec_all_done = 0;
			hw_ctx->dec_layer0_done = 0;

			if (online_node->is_3dnr) {
				mv_ready = online_node->nr3_me.bin_path_mv_ready;
				if (mv_ready == 0) {
					online_node->nr3_me.bin_path_cnt++;
					return ret;
				} else if (mv_ready == 1) {
					frame = dcamfetch_frame_prepare(online_node, dcam_port, hw_ctx);
					if (frame == NULL) {
						return ret;
					}
					dcamfetch_nr3_mv_get(hw_ctx, frame);
					online_node->nr3_me.bin_path_mv_ready = 0;
					online_node->nr3_me.bin_path_cnt = 0;
					pr_debug("dcam %d,fid %d mv_x %d mv_y %d", online_node->hw_ctx_id, frame->fid, frame->nr3_me.mv_x, frame->nr3_me.mv_y);
					irq_proc->param = frame;
					irq_proc->type = CAM_CB_DCAM_DATA_DONE;
					dcamfetch_frame_dispatch(irq_proc, online_node);
				}
			} else {
				if ((frame = dcamfetch_frame_prepare(online_node, dcam_port, hw_ctx))) {
					irq_proc->param = frame;
					irq_proc->type = CAM_CB_DCAM_DATA_DONE;
					dcamfetch_frame_dispatch(irq_proc, online_node);
				}
			}

			i = 0;
			while (++i < online_node->slowmotion_count) {
				irq_proc->param = dcamfetch_frame_prepare(online_node, dcam_port, hw_ctx);
				irq_proc->type = CAM_CB_DCAM_DATA_DONE;
				dcamfetch_frame_dispatch(irq_proc, online_node);
			}
			if (!node->virtualsensor_cap_en) {
				frame = cam_queue_dequeue(&node->proc_queue, struct camera_frame, list);
				if (frame) {
					cam_buf_iommu_unmap(&frame->buf);
					if (online_node->data_cb_func)
						online_node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, frame, online_node->data_cb_handle);
				}
				complete(&node->frm_done);
			}
			break;
		default:
			pr_err("fail to get known node %d, port_id %s\n", online_node->node_id, cam_port_name_get(irq_proc->dcam_port_id));
			ret = -EFAULT;
			break;
		}
	}
	return ret;
}

static int dcamfetch_irq_proc(void *param, void *handle)
{
	int ret = 0;
	struct dcam_fetch_node *node = NULL;
	struct dcam_online_node *online_node = NULL;
	struct dcam_irq_proc *irq_proc = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	if (!handle || !param) {
		pr_err("fail to get valid param %px %px\n", handle, param);
		return -EFAULT;
	}

	node = (struct dcam_fetch_node *)handle;
	irq_proc = (struct dcam_irq_proc *)param;
	pr_debug("irq proc of %d", irq_proc->of);

	if (!node->hw_ctx) {
		pr_warn("warning: dcam_online_node hw_ctx %px\n", node->hw_ctx);
		return ret;
	}
	hw_ctx = node->hw_ctx;
	online_node = &node->online_node;

	switch (irq_proc->of) {
	case CAP_START_OF_FRAME:
	case CAP_END_OF_FRAME:
	case SN_START_OF_FRAME:
	case SN_END_OF_FRAME:
	case DCAM_INT_ERROR:
		dcam_online_node_irq_proc(param, online_node);
		break;
	case PREV_START_OF_FRAME:
		dcam_online_node_irq_proc(param, online_node);
		hw_ctx->frame_index++;
		break;
	case CAP_DATA_DONE:
		if (irq_proc->dcam_port_id == PORT_FULL_OUT
			|| irq_proc->dcam_port_id == PORT_BIN_OUT)
			dcamfetch_done_proc(param, handle, hw_ctx);
		else
			dcam_online_node_irq_proc(param, online_node);
		break;
	default:
		break;
	}
	pr_debug("irq proc done\n");
	return ret;
}

static int dcamfetch_frm_set(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;
	struct camera_frame *pframe = NULL;

	pframe = (struct camera_frame *)param;
	pframe->priv_data = node;
	ret = cam_queue_enqueue(&node->in_queue, &pframe->list);
	if (ret)
		return -1;
	complete(&node->thread.thread_com);

	return ret;
}

static void dcamfetch_src_frame_ret(void *param)
{
	struct camera_frame *frame = NULL;
	struct dcam_fetch_node *node = NULL;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	node = (struct dcam_fetch_node *)frame->priv_data;
	if (!node) {
		pr_err("fail to get valid src_frame node.\n");
		return;
	}

	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd);

	cam_buf_iommu_unmap(&frame->buf);
	node->online_node.data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, frame, node->online_node.data_cb_handle);
}

static struct camera_frame *dcamfetch_cycle_frame(struct dcam_fetch_node *node)
{
	int ret = 0, loop = 0;
	struct camera_frame *pframe = NULL;

	if (!node) {
		pr_err("fail to get valid dcam fetch node\n");
		return NULL;
	}

	pframe = cam_queue_dequeue(&node->in_queue, struct camera_frame, list);
	if (pframe == NULL) {
		pr_err("fail to get input frame (%p) for node %d\n", pframe, node->online_node.node_id);
		return NULL;
	}

	pr_debug("frame %p, dcam%d ch_id %d.  buf_fd %d\n", pframe,
			node->online_node.hw_ctx_id, pframe->channel_id, pframe->buf.mfd);
	pr_debug("size %d %d,  endian %d, pattern %d\n",
			pframe->width, pframe->height, pframe->endian, pframe->pattern);

	ret = cam_buf_iommu_map(&pframe->buf, CAM_IOMMUDEV_DCAM);
	if (ret) {
		pr_err("fail to map buf to DCAM iommu. ctx %d\n", node->online_node.node_id);
		goto map_err;
	}

	loop = 0;
	do {
		ret = cam_queue_enqueue(&node->proc_queue, &pframe->list);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		/* wait for previous frame proccessed done.*/
		usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to get proc queue, timeout.\n");
		ret = -EINVAL;
		goto inq_overflow;
	}
	return pframe;

inq_overflow:
	cam_buf_iommu_unmap(&pframe->buf);

map_err:
	node->online_node.data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, node->online_node.data_cb_handle);

	return NULL;
}

static int dcamfetch_param_get(struct dcam_fetch_node *node,
		struct camera_frame *pframe)
{
	int ret = 0;
	struct dcam_fetch_info *fetch = NULL;

	fetch = &node->fetch_info;
	fetch->endian = pframe->endian;
	fetch->pattern = pframe->pattern;
	fetch->size.w = pframe->width;
	fetch->size.h = pframe->height;
	fetch->trim.start_x = 0;
	fetch->trim.start_y = 0;
	fetch->trim.size_x = pframe->width;
	fetch->trim.size_y = pframe->height;
	fetch->addr.addr_ch0 = (uint32_t)pframe->buf.iova;

	node->online_node.hw_ctx->hw_fetch.idx = node->online_node.hw_ctx_id;
	node->online_node.hw_ctx->hw_fetch.virtualsensor_pre_sof = 1;
	node->online_node.hw_ctx->hw_fetch.fetch_info = &node->fetch_info;

	return ret;
}

static int dcamfetch_hw_frame_param_set(struct dcam_hw_context *hw_ctx)
{
	int ret = 0, i = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;

	for (i = 0; i < DCAM_PATH_MAX; i++) {
		if (!hw_ctx->hw_path[i].need_update)
			continue;
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_ADDR_SET, &hw_ctx->hw_path[i].hw_fbc_store);
		else
			hw->dcam_ioctl(hw, DCAM_HW_CFG_STORE_ADDR, &hw_ctx->hw_path[i].hw_store);

		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &hw_ctx->hw_path[i].hw_size);
		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &hw_ctx->hw_path[i].hw_start);
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_CTRL, &hw_ctx->hw_path[i].hw_fbc);
	}

	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_SET, &hw_ctx->hw_fetch);

	return ret;
}

static int dcamfetch_slice_proc(struct dcam_fetch_node *node)
{
	int i = 0, ret = 0;
	uint32_t force_ids = DCAM_CTRL_ALL;
	struct cam_hw_reg_trace trace = {0};
	struct dcam_hw_slice_fetch slicearg = {0};
	struct dcam_hw_force_copy copyarg = {0};
	struct dcam_fetch_info *fetch = NULL;
	struct dcam_offline_slice_info *slice = NULL;
	struct dcam_hw_context *hw_ctx = NULL;
	struct cam_hw_info *hw = NULL;

	fetch = &node->fetch_info;
	hw_ctx = node->online_node.hw_ctx;
	hw = hw_ctx->hw;
	slice = &hw_ctx->slice_info;

	for (i = 0; i < slice->slice_num; i++) {
		ret = wait_for_completion_interruptible_timeout(
				&node->slice_done, DCAM_OFFLINE_TIMEOUT);
		if (ret <= 0) {
			pr_err("fail to wait as dcam%d offline timeout. ret: %d\n", hw_ctx->hw_ctx_id, ret);
			trace.type = NORMAL_REG_TRACE;
			trace.idx = hw_ctx->hw_ctx_id;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
			return -EFAULT;
		}

		slice->cur_slice = &slice->slice_trim[i];
		pr_debug("slice%d/%d proc start\n", i, slice->slice_num);

		if (hw->ip_dcam[hw_ctx->hw_ctx_id]->offline_slice_support && slice->slice_num > 1) {
			slicearg.idx = hw_ctx->hw_ctx_id;
			slicearg.virtualsensor_pre_sof = 1;
			slicearg.path_id= hw->ip_dcam[1]->aux_dcam_path;
			slicearg.fetch = fetch;
			slicearg.cur_slice = slice->cur_slice;
			slicearg.dcam_slice_mode = slice->dcam_slice_mode;
			slicearg.slice_num = slice->slice_num;
			slicearg.slice_count = slice->slice_count;
			slicearg.st_pack = cam_is_pack(hw_ctx->hw_path[DCAM_PATH_FULL].hw_start.out_fmt);
			pr_debug("slc%d, (%d %d %d %d)\n", i,
				slice->slice_trim[i].start_x, slice->slice_trim[i].start_y,
				slice->slice_trim[i].size_x, slice->slice_trim[i].size_y);

			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLICE_FETCH_SET, &slicearg);
		}

		/* DCAM_CTRL_COEF will always set in dcam_init_lsc() */
		copyarg.id = force_ids;
		copyarg.idx = hw_ctx->hw_ctx_id;
		copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);
		udelay(500);

		if (i == 0) {
			trace.type = NORMAL_REG_TRACE;
			trace.idx = hw_ctx->hw_ctx_id;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
		}

		/* start fetch */
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_START, hw);
	}
	return 0;
}

static int dcamfetch_node_frame_start(void *param)
{
	int ret = 0;
	struct dcam_fetch_node *node = NULL;
	struct dcam_online_node *online_node = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_offline_slice_info *slice = NULL;
	uint32_t lbuf_width = 0, dev_fid = 0;

	node = (struct dcam_fetch_node *)param;
	online_node = &node->online_node;
	if (!node || !online_node) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	pr_debug("dcam fetch frame start %px\n", param);
	ret = wait_for_completion_interruptible_timeout(&node->frm_done, DCAM_OFFLINE_TIMEOUT);
	if (ret <= 0) {
		pframe = cam_queue_dequeue(&node->in_queue, struct camera_frame, list);
		if (!pframe) {
			pr_warn("warning: no frame from in_q node %px\n", node);
			return 0;
		}
		ret = -EFAULT;
		online_node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, online_node->data_cb_handle);
		return ret;
	}
	slice = &online_node->hw_ctx->slice_info;
	pframe = dcamfetch_cycle_frame(node);
	if (!pframe)
		goto fetch_input_err;

	node->hw_ctx->index_to_set = pframe->fid - node->hw_ctx->base_fid;
	dev_fid = node->hw_ctx->index_to_set;
	if (pframe->sensor_time.tv_sec || pframe->sensor_time.tv_usec) {
		online_node->frame_ts[tsid(dev_fid)].tv_sec = pframe->sensor_time.tv_sec;
		online_node->frame_ts[tsid(dev_fid)].tv_nsec = pframe->sensor_time.tv_usec * NSEC_PER_USEC;
		online_node->frame_ts_boot[tsid(dev_fid)] = pframe->boot_sensor_time;
	}
	ret = dcamfetch_param_get(node, pframe);
	if (ret) {
		pr_err("fail to get dcam online fetch param\n");
		goto fetch_return_buf;
	}

	/* fetch in node may put it into hw ctx */
	slice->fetch_fmt= node->fetch_info.fmt;
	dcamslice_needed_info_get(online_node->hw_ctx, &lbuf_width, pframe->width);
	dcam_slice_info_cal(slice, pframe, lbuf_width);
	dcamfetch_hw_frame_param_set(online_node->hw_ctx);

	ret = dcamfetch_slice_proc(node);
	pr_debug("done\n");

	return ret;

fetch_return_buf:
	pframe = cam_queue_dequeue(&node->proc_queue, struct camera_frame, list);
	cam_buf_iommu_unmap(&pframe->buf);
	online_node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, online_node->data_cb_handle);
fetch_input_err:
	complete(&node->slice_done);
	complete(&node->frm_done);
	return ret;
}

int dcam_fetch_node_blk_param_set(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;

	if (!node || !param) {
		pr_err("fail to get valid param %px, %px\n", node, param);
		return -EFAULT;
	}

	ret = dcam_online_node_blk_param_set(&node->online_node, param);
	return ret;

}

int dcam_fetch_node_state_get(void *handle)
{
	int ret = 0;
	struct dcam_fetch_node *node = (struct dcam_fetch_node *)handle;

	if (!handle) {
		pr_err("fail to get handle\n");
		return -EFAULT;
	}

	ret = dcam_online_node_state_get(&node->online_node);

	return ret;
}

int dcam_fetch_node_port_insert(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;

	ret = dcam_online_node_port_insert(&node->online_node, param);
	return ret;
}

int dcam_fetch_node_cfg_param(void *handle, uint32_t cmd, void *param)
{
	int ret = 0;
	struct dcam_fetch_node *node = NULL;

	if (!handle || !param) {
		pr_err("fail to get valid input ptr %px %px\n", handle, param);
		return -EFAULT;
	}

	node = (struct dcam_fetch_node *)handle;

	switch (cmd) {
	case CAM_NODE_CFG_BUF:
		ret = dcamfetch_frm_set(node, param);
		break;
	case CAM_NODE_CFG_STATIS:
	case CAM_NODE_RECT_GET:
		ret =dcam_online_cfg_param(&node->online_node, cmd, param);
		break;
	default:
		pr_err("fail to support vaild cmd %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

int dcam_fetch_share_buf(void *handle, void *param)
{
	int ret = 0;
	struct dcam_fetch_node *node = NULL;

	node = (struct dcam_fetch_node *)handle;
	ret = dcam_online_share_buf(&node->online_node, param);
	return ret;
}

int dcam_fetch_node_reset(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;

	node->hw_ctx->is_virtualsensor_proc = 0;
	ret = dcam_online_node_reset(&node->online_node, param);
	return ret;
}

int dcam_fetch_node_request_proc(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	ret = dcam_online_node_request_proc(&node->online_node, param);
	node->hw_ctx = node->online_node.hw_ctx;
	node->hw_ctx->dcam_irq_cb_func = dcamfetch_irq_proc;
	node->hw_ctx->dcam_irq_cb_handle = node;
	node->hw_ctx->is_virtualsensor_proc = 1;
	return ret;
}

int dcam_fetch_node_stop_proc(struct dcam_fetch_node *node, void *param)
{
	int ret = 0;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	node->hw_ctx->is_virtualsensor_proc = 0;
	ret = dcam_online_node_stop_proc(&node->online_node, param);

	return ret;
}

void *dcam_fetch_node_get(uint32_t node_id, struct dcam_fetch_node_desc *param)
{
	int ret = 0;
	struct dcam_fetch_node *node = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	if (node_id) {
		pr_err("dcam fetch node not support nonzero node id %d\n", node_id);
		return NULL;
	}

	pr_info("node id %d node_dev %px\n", node_id, *param->node_dev);
	if (*param->node_dev == NULL) {
		node = cam_buf_kernel_sys_vzalloc(sizeof(struct dcam_fetch_node));
		if (!node) {
			pr_err("fail to get valid dcam fetch node\n");
			return NULL;
		}
	} else {
		node = *param->node_dev;
		node->virtualsensor_cap_en = param->virtualsensor_cap_en;
		pr_info("dcam fetch node has been alloc %p\n", node);
		goto exit;
	}

	memset(&node->online_node.nr3_me, 0, sizeof(struct nr3_me_data));

	atomic_set(&node->online_node.state, STATE_IDLE);
	atomic_set(&node->online_node.pm_cnt, 0);
	atomic_set(&node->user_cnt, 0);
	atomic_set(&node->online_node.ch_enable_cnt, 0);
	node->online_node.slowmotion_count = param->online_node_desc->slowmotion_count;
	if (!node->online_node.slowmotion_count)
		node->online_node.slw_type = DCAM_SLW_OFF;
	else
		node->online_node.slw_type = DCAM_SLW_AP;
	node->online_node.is_4in1 = param->online_node_desc->is_4in1;
	node->online_node.is_3dnr = param->online_node_desc->enable_3dnr;
	node->online_node.nr3_frm = NULL;
	node->online_node.cap_info = param->online_node_desc->cap_info;
	node->online_node.is_pyr_rec = param->online_node_desc->is_pyr_rec;
	node->online_node.dcam_slice_mode_temp = param->online_node_desc->dcam_slice_mode_temp;
	node->online_node.dcam_idx = param->online_node_desc->dcam_idx;
	node->online_node.dev = param->online_node_desc->dev;
	node->online_node.hw_ctx_id = DCAM_HW_CONTEXT_MAX;
	node->online_node.raw_alg_type = param->online_node_desc->raw_alg_type;
	node->online_node.param_frame_sync = param->online_node_desc->param_frame_sync;
	node->online_node.is_ebd = param->online_node_desc->is_ebd;
	memcpy((void*)&node->online_node.ebd_param, (void*)&param->online_node_desc->ebd_param, sizeof(struct sprd_ebd_control));

	cam_queue_init(&node->online_node.port_queue, PORT_DCAM_OUT_MAX, NULL);
	node->online_node.node_id = node_id;
	node->online_node.node_type = param->online_node_desc->node_type;
	init_completion(&node->frm_done);
	complete(&node->frm_done);
	init_completion(&node->slice_done);
	complete(&node->slice_done);
	cam_queue_init(&node->in_queue, DCAM_FETCH_IN_Q_LEN, dcamfetch_src_frame_ret);
	cam_queue_init(&node->proc_queue, DCAM_FETCH_PROC_Q_LEN, dcamfetch_src_frame_ret);
	thrd = &node->thread;
	sprintf(thrd->thread_name, "dcam_fetch_node%d", node->online_node.node_id);
	ret = camthread_create(node, thrd, dcamfetch_node_frame_start);
	if (unlikely(ret != 0)) {
		pr_err("fail to create dcam offline node thread\n");
		return NULL;
	}
	if (node->online_node.data_cb_func == NULL) {
		node->online_node.data_cb_func = param->online_node_desc->data_cb_func;
		node->online_node.data_cb_handle = param->online_node_desc->data_cb_handle;
	}
	if (node->online_node.sharebuf_get_cb == NULL) {
		node->online_node.sharebuf_get_cb = param->online_node_desc->sharebuf_get_cb;
		node->online_node.sharebuf_cb_data = param->online_node_desc->sharebuf_cb_data;
	}
	if (node->online_node.resbuf_get_cb == NULL) {
		node->online_node.resbuf_get_cb = param->online_node_desc->resbuf_get_cb;
		node->online_node.resbuf_cb_data = param->online_node_desc->resbuf_cb_data;
	}
	if (node->online_node.port_cfg_cb_func == NULL) {
		node->online_node.port_cfg_cb_func = param->online_node_desc->port_cfg_cb_func;
		node->online_node.port_cfg_cb_handle = param->online_node_desc->port_cfg_cb_handle;
	}
	if (node->online_node.shutoff_cfg_cb_func == NULL) {
		node->online_node.shutoff_cfg_cb_func = param->online_node_desc->shutoff_cfg_cb_func;
		node->online_node.shutoff_cfg_cb_handle = param->online_node_desc->shutoff_cfg_cb_handle;
	}

	ret = dcam_online_node_pmctx_init(&node->online_node);
	if (ret)
		goto exit;

	dcam_online_node_pmctx_update(&node->online_node.blk_pm, param->online_node_desc->blk_pm);
	*param->node_dev = node;

exit:
	node->online_node.is_3dnr |= param->online_node_desc->enable_3dnr;
	atomic_inc(&node->user_cnt);
	pr_info("node id %d\n", node->online_node.node_id);
	return node;
}

void dcam_fetch_node_put(struct dcam_fetch_node *node)
{
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	cam_queue_clear(&node->online_node.port_queue, struct dcam_online_port, list);
	if (atomic_dec_return(&node->user_cnt) == 0) {
		camthread_stop(&node->thread);
		cam_queue_clear(&node->in_queue, struct camera_frame, list);
		cam_queue_clear(&node->proc_queue, struct camera_frame, list);
		memset(&node->online_node.nr3_me, 0, sizeof(struct nr3_me_data));
		if (atomic_read(&node->online_node.pm_cnt) > 0)
			dcam_online_node_pmctx_deinit(&node->online_node);
		node->online_node.data_cb_func = NULL;
		node->online_node.data_cb_handle = NULL;
		node->online_node.sharebuf_get_cb = NULL;
		node->online_node.sharebuf_cb_data = NULL;
		node->online_node.resbuf_get_cb = NULL;
		node->online_node.resbuf_cb_data = NULL;
		node->online_node.port_cfg_cb_func = NULL;
		node->online_node.port_cfg_cb_handle = NULL;
		atomic_set(&node->online_node.state, STATE_INIT);
		pr_info("dcam fetch node %d put success\n", node->online_node.node_id);
		cam_buf_kernel_sys_vfree(node);
		node = NULL;
	}
}
