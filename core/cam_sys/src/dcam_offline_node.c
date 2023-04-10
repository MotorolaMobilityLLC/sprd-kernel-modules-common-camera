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

#include "cam_offline_statis.h"
#include "cam_pipeline.h"
#include "cam_zoom.h"
#include "dcam_dummy.h"
#include "dcam_slice.h"

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "DCAM_OFFLINE_NODE: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define dcamoffline_statis_work_set(bypass, node, port_id) ( { \
	struct dcam_offline_port *__port = NULL; \
	if ((bypass) == 0) { \
		__port = dcam_offline_node_port_get(node, port_id); \
		if (__port) \
			atomic_set(&__port->is_work, 1); \
	}\
})

static int dcamoffline_hw_reset(struct dcam_hw_context *hw_ctx)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	hw_ctx->fid = 0;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_RESET, &hw_ctx->hw_ctx_id);
	if (ret)
		pr_err("fail to reset dcam%d\n", hw_ctx->hw_ctx_id);

	return ret;
}

static int dcamoffline_node_size_update(struct dcam_offline_node *node,
	struct dcam_offline_port *port, struct cam_frame *pframe)
{
	int ret = 0;
	struct cam_zoom_base zoom_base = {0};
	struct cam_zoom_index zoom_index = {0};

	memset(&pframe->common.zoom_data, 0 , sizeof(struct cam_zoom_frame));
	ret = port->zoom_cb_func(port->zoom_cb_handle, &pframe->common.zoom_data);
	if (!ret) {
		zoom_index.node_type = node->node_type;
		zoom_index.node_id = node->node_id;
		zoom_index.port_type = port->type;
		zoom_index.port_id = port->port_id;
		zoom_index.zoom_data = &pframe->common.zoom_data;
		ret = cam_zoom_frame_base_get(&zoom_base, &zoom_index);
		if (!ret) {
			dcam_offline_port_size_cfg(port, &zoom_base);
		}
	}

	return ret;
}

static int dcamoffline_ctx_unbind(struct dcam_offline_node *node)
{
	int ret = 0, i = 0;
	unsigned long flag = 0;
	struct dcam_dummy_param dummy_param = {0};

	if (!node) {
		pr_err("fail to get valid dcam offline node\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&node->dev->ctx_lock, flag);
	ret = node->dev->dcam_pipe_ops->unbind(node->dev, node, node->node_id);
	if (ret) {
		pr_err("fail to context_unbind\n");
		goto exit;
	}
	if (node->hw_ctx->dummy_slave) {
		dummy_param.hw_ctx_id = node->hw_ctx->hw_ctx_id;
		dummy_param.enable = DISABLE;
		node->hw_ctx->dummy_slave->dummy_ops->dummy_enable(node->hw_ctx->dummy_slave, &dummy_param);
	}
	node->hw_ctx->is_offline_proc = 0;
	node->hw_ctx->err_count = 0;
	node->hw_ctx->dcam_irq_cb_func = NULL;
	node->hw_ctx->dcam_irq_cb_handle = NULL;
	node->hw_ctx_id = DCAM_HW_CONTEXT_MAX;
	for (i = 0; i < DCAM_PATH_MAX; i++)
		node->hw_ctx->hw_path[i].need_update = 0;
	node->hw_ctx = NULL;
	node->pm_ctx.blk_pm.idx = DCAM_HW_CONTEXT_MAX;

exit:
	spin_unlock_irqrestore(&node->dev->ctx_lock, flag);
	return ret;
}

static int dcamoffline_node_ts_cal(struct dcam_offline_node *node)
{
	uint32_t sec = 0, usec = 0;
	timespec consume_ts = {0};
	timespec node_end_ts = {0};
	struct dcam_offline_slice_info *slice = NULL;
	struct dcam_hw_context *hw_ctx = NULL;

	hw_ctx = node->hw_ctx;
	slice = &hw_ctx->slice_info;
	os_adapt_time_get_ts(&slice->slice_start_ts[tsid(slice->slice_num - slice->slice_count)]);

	if (slice->slice_num - slice->slice_count == 0) {
		node_end_ts = slice->slice_start_ts[tsid(slice->slice_num - slice->slice_count)];
		consume_ts = os_adapt_time_timespec_sub(node_end_ts, node->start_ts);
		sec = consume_ts.tv_sec;
		usec = consume_ts.tv_nsec / NSEC_PER_USEC;
		if ((sec * USEC_PER_SEC + usec) > DCAMOFFLINE_NODE_TIME)
			pr_warn("Warning: dcamoffline node process too long. consume_time %d.%06d\n", sec, usec);

		PERFORMANCE_DEBUG("dcamoffline_node: cur_time %d.%06d, consume_time %d.%06d\n",
			node_end_ts.tv_sec, node_end_ts.tv_nsec / NSEC_PER_USEC,
			consume_ts.tv_sec, consume_ts.tv_nsec / NSEC_PER_USEC);
	}
	return 0;
}

 static int dcamoffline_hw_ts_cal(struct dcam_offline_node *node)
 {
	uint32_t  size = 0;
	int64_t sec = 0, usec = 0, time_ratio = 0;
	int cur_slice_count = 0;
 	timespec consume_ts = {0};
	timespec slice_end_ts = {0};
	timespec slice_start_ts = {0};
	struct dcam_offline_slice_info *slice = NULL;
	struct dcam_hw_context *hw_ctx = NULL;

	hw_ctx = node->hw_ctx;
	slice = &hw_ctx->slice_info;
	cur_slice_count = slice->slice_num - slice->slice_count;
	os_adapt_time_get_ts(&slice->slice_end_ts[tsid(cur_slice_count)]);

	slice_start_ts = slice->slice_start_ts[tsid(cur_slice_count)];
	slice_end_ts = slice->slice_end_ts[tsid(cur_slice_count)];

	consume_ts = os_adapt_time_timespec_sub(slice_end_ts, slice_start_ts);
 	sec = consume_ts.tv_sec;
 	usec = consume_ts.tv_nsec / NSEC_PER_USEC;

	size = slice->slice_trim[cur_slice_count].size_x * slice->slice_trim[cur_slice_count].size_y;
	time_ratio = div_s64(TIME_SIZE_RATIO * (sec * USEC_PER_SEC + usec), size);
	if (time_ratio > DCAMOFFLINE_HW_TIME_RATIO)
		pr_warn("Warning: slice%d process too long. consume_time %d.%06d.\n", cur_slice_count, sec, usec);

	PERFORMANCE_DEBUG("dcamoffline_hw: slice%d, cur_time %d.%06d, consume_time %d.%06d\n",
		cur_slice_count, slice_end_ts.tv_sec, slice_end_ts.tv_nsec / NSEC_PER_USEC,
 		consume_ts.tv_sec, consume_ts.tv_nsec / NSEC_PER_USEC);

	if (slice->slice_count - 1 == 0) {
		consume_ts = os_adapt_time_timespec_sub(slice_end_ts, slice->slice_start_ts[tsid(0)]);
		sec = consume_ts.tv_sec;
		usec = consume_ts.tv_nsec / NSEC_PER_USEC;
		size =  node->fetch.size.w * node->fetch.size.h;
		time_ratio = div_s64(TIME_SIZE_RATIO * (sec * USEC_PER_SEC + usec), size);
		if (time_ratio > DCAMOFFLINE_HW_TIME_RATIO)
			pr_warn("Warning: dcamoffline hw process too long. consume_time %d.%06d\n", sec, usec);

		PERFORMANCE_DEBUG("dcamoffline_hw: id %d, cur_time %d.%06d, consume_time %d.%06d\n",
			node->hw_ctx_id, slice_end_ts.tv_sec, slice_end_ts.tv_nsec / NSEC_PER_USEC,
			consume_ts.tv_sec, consume_ts.tv_nsec / NSEC_PER_USEC);
	}
 	return 0;
 }

static int dcamoffline_irq_proc(void *param, void *handle)
{
	int ret = 0, time_out = 0, port_id = 0, is_frm_port = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_frame *frame = NULL, *frame1 = NULL;
	struct dcam_offline_node *node = NULL;
	struct dcam_irq_proc_desc *irq_desc = NULL;
	struct dcam_offline_slice_info *slice_info = NULL;
	struct cam_node_cfg_param node_param = {0};
	struct camera_buf_get_desc buf_desc = {0};
	timespec *ts = NULL;

	if (!param || !handle) {
		pr_err("fail to get valid param %px %px\n", param, handle);
		return -EFAULT;
	}

	node = (struct dcam_offline_node *)handle;
	irq_desc = (struct dcam_irq_proc_desc *)param;

	slice_info = &node->hw_ctx->slice_info;
	hw = node->dev->hw;
	if (irq_desc->dcam_cb_type == CAM_CB_DCAM_DEV_ERR) {
		hw->dcam_ioctl(hw, DCAM_HW_CFG_RESET, &node->hw_ctx->hw_ctx_id);
		complete(&node->recovery_thread.thread_com);
		return 0;
	}
	port_id = irq_desc->dcam_port_id;
	is_frm_port = (port_id == PORT_OFFLINE_FULL_OUT || port_id == PORT_OFFLINE_BIN_OUT || port_id == PORT_OFFLINE_RAW_OUT);

	node->in_irq_proc = 1;
	pr_debug("dcam offline irq proc\n");
	if (slice_info->slice_num > 0 && is_frm_port) {
		pr_debug("dcam%d offline slice%d done.\n", node->hw_ctx_id,
			(slice_info->slice_num - slice_info->slice_count));
		if (irq_desc->dcam_cb_type == CAM_CB_DCAM_DATA_DONE)
			dcamoffline_hw_ts_cal(node);
		slice_info->slice_count--;
		time_out = hw->dcam_ioctl(hw, DCAM_HW_FETCH_STATUS_GET, &node->hw_ctx_id);
		if (time_out > CAM_DCAM_AXI_STOP_TIMEOUT)
			pr_warn("Warning:dcam fetch status is busy. timeout %d\n", time_out);

		complete(&node->slice_done);
		if (slice_info->slice_count > 0)
			return 0;
	}

	if (node->port_cfg_cb_func) {
		node_param.port_id = irq_desc->dcam_port_id;
		ret = node->port_cfg_cb_func(&node_param, PORT_CFG_BUFFER_GET, node->port_cfg_cb_handle);
		if (ret)
			pr_err("fail to get dcam offline port param\n");
		frame = (struct cam_frame *)(node_param.param);
		if (frame && is_frm_port)
			cam_buf_manager_buf_status_cfg(&frame->common.buf, CAM_BUF_STATUS_PUT_IOVA, CAM_BUF_IOMMUDEV_DCAM);
		if (frame && node->data_cb_func) {
			frame->common.link_from.node_type = node->node_type;
			frame->common.link_from.node_id = node->node_id;
			ts = &node->frame_ts[tsid(frame->common.fid)];
			frame->common.sensor_time.tv_sec = ts->tv_sec;
			frame->common.sensor_time.tv_usec = ts->tv_nsec / NSEC_PER_USEC;
			frame->common.boot_sensor_time = node->frame_ts_boot[tsid(frame->common.fid)];
			if (is_frm_port) {
				buf_desc.q_ops_cmd = CAM_QUEUE_DEQ_PEEK;
				frame1 = cam_buf_manager_buf_dequeue(&node->proc_pool, &buf_desc, node->buf_manager_handle);
				if (frame1)
					frame->common.zoom_data = frame1->common.zoom_data;
			}
			node->data_cb_func(irq_desc->dcam_cb_type, frame, node->data_cb_handle);
		}
	}

	if (is_frm_port) {
		time_out = hw->dcam_ioctl(hw, DCAM_HW_FETCH_STATUS_GET, &node->hw_ctx_id);
		if (time_out > CAM_DCAM_AXI_STOP_TIMEOUT)
			pr_warn("Warning:dcam fetch status is busy.\n");
		frame = cam_buf_manager_buf_dequeue(&node->proc_pool, NULL, node->buf_manager_handle);
		if (frame) {
			cam_buf_manager_buf_status_cfg(&frame->common.buf, CAM_BUF_STATUS_PUT_IOVA, CAM_BUF_IOMMUDEV_DCAM);
			if (node->data_cb_func)
				node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, frame, node->data_cb_handle);
		}
		ret = dcamoffline_hw_reset(node->hw_ctx);
		if (ret)
			pr_err("fail to reset dcam%d\n", node->hw_ctx_id);
		dcamoffline_ctx_unbind(node);
		complete(&node->frm_done);
	}
	node->in_irq_proc = 0;
	return ret;
}

static int dcamoffline_ctx_bind(struct dcam_offline_node *node)
{
	int ret = 0;
	uint32_t slw_cnt = 0, slw_type = DCAM_SLW_OFF;
	unsigned long flag = 0;

	if (!node) {
		pr_err("fail to get valid dcam offline node\n");
		return -EFAULT;
	}

	spin_lock_irqsave(&node->dev->ctx_lock, flag);
	ret = node->dev->dcam_pipe_ops->bind(node->dev, node, node->node_id,
			node->dcam_idx, slw_cnt, &node->hw_ctx_id, &slw_type);
	if (ret) {
		pr_warn("warning: bind ctx %d fail\n", node->node_id);
		goto exit;
	}

	node->hw_ctx = &node->dev->hw_ctx[node->hw_ctx_id];
	node->pm_ctx.blk_pm.idx = node->hw_ctx_id;
	node->hw_ctx->is_offline_proc = 1;
	node->hw_ctx->dcam_irq_cb_func = dcamoffline_irq_proc;
	node->hw_ctx->dcam_irq_cb_handle = node;
	node->hw_ctx->err_count = 1;

exit:
	spin_unlock_irqrestore(&node->dev->ctx_lock, flag);
	return ret;
}

static struct cam_frame *dcamoffline_cycle_frame(struct dcam_offline_node *node)
{
	int ret = 0, loop = 0, cnt = 0;
	struct cam_frame *pframe = NULL;
	struct dcam_offline_port *port = NULL;
	struct camera_buf_get_desc buf_desc = {0};

	if (!node) {
		pr_err("fail to get valid dcam offline node\n");
		return NULL;
	}

	pframe = cam_buf_manager_buf_dequeue(&node->in_pool, NULL, node->buf_manager_handle);
	if (pframe == NULL) {
		pr_err("fail to get input frame %p for node %d\n", pframe, node->node_id);
		return NULL;
	}

	pr_debug("frame %p, dcam%d ch_id %d, buf_fd %d, size %d %d\n", pframe, node->hw_ctx_id,
		pframe->common.channel_id, pframe->common.buf.mfd, pframe->common.width, pframe->common.height);

	CAM_QUEUE_FOR_EACH_ENTRY(port, &node->port_queue.head, list) {
		cnt = atomic_read(&port->is_work);
		if (port->type == PORT_TRANSFER_OUT && atomic_read(&port->user_cnt) > 0)
			dcamoffline_node_size_update(node, port, pframe);
	}

	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
	buf_desc.mmu_type = CAM_BUF_IOMMUDEV_DCAM;

	loop = 0;
	do {
		ret = cam_buf_manager_buf_enqueue(&node->proc_pool, pframe, &buf_desc, node->buf_manager_handle);
		if (ret == 0)
			break;
		pr_info_ratelimited("wait for proc queue. loop %d\n", loop);
		/* wait for previous frame proccessed done.*/
		os_adapt_time_usleep_range(600, 2000);
	} while (loop++ < 500);

	if (ret) {
		pr_err("fail to get proc queue, timeout.\n");
		ret = -EINVAL;
		goto inq_overflow;
	}
	return pframe;

inq_overflow:
	node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, node->data_cb_handle);

	return NULL;
}

static int dcamoffline_fetch_param_get(struct dcam_offline_node *node,
		struct cam_frame *pframe)
{
	int ret = 0;
	struct dcam_fetch_info *fetch = NULL;

	fetch = &node->fetch;
	fetch->size.w = pframe->common.width;
	fetch->size.h = pframe->common.height;
	fetch->trim.start_x = 0;
	fetch->trim.start_y = 0;
	fetch->trim.size_x = pframe->common.width;
	fetch->trim.size_y = pframe->common.height;
	fetch->addr.addr_ch0 = (uint32_t)pframe->common.buf.iova[CAM_BUF_IOMMUDEV_DCAM];

	node->hw_ctx->hw_fetch.idx = node->hw_ctx_id;
	node->hw_ctx->hw_fetch.fetch_info = &node->fetch;

	return ret;
}

static int dcamoffline_hw_statis_work_set(struct dcam_offline_node *node)
{
	int ret = 0;
	struct dcam_hw_context *hw_ctx = NULL;
	struct cam_hw_info *hw = NULL;
	struct dcam_isp_k_block *pm = NULL;
	struct dcam_offline_slice_info *slice_info = NULL;

	hw_ctx = node->hw_ctx;
	hw = hw_ctx->hw;
	pm = hw_ctx->blk_pm;
	slice_info = &node->hw_ctx->slice_info;

	if (slice_info->slice_num > 1)
		node->statis_en = 0;

	if (node->statis_en == 1) {
		dcamoffline_statis_work_set(pm->aem.bypass, node, PORT_OFFLINE_AEM_OUT);
		dcamoffline_statis_work_set(pm->afm.bypass, node, PORT_OFFLINE_AFM_OUT);
		dcamoffline_statis_work_set(pm->pdaf.bypass, node, PORT_OFFLINE_PDAF_OUT);
		dcamoffline_statis_work_set(pm->afl.afl_info.bypass, node, PORT_OFFLINE_AFL_OUT);
		dcamoffline_statis_work_set(pm->hist.bayerHist_info.hist_bypass, node, PORT_OFFLINE_BAYER_HIST_OUT);
		dcamoffline_statis_work_set(pm->lscm.bypass, node, PORT_OFFLINE_LSCM_OUT);
		if(hw->ip_isp->isphw_abt->frbg_hist_support)
			dcamoffline_statis_work_set(pm->hist_roi.hist_roi_info.bypass, node, PORT_OFFLINE_FRGB_HIST_OUT);
	} else {
		pm->aem.bypass = 1;
		pm->hist.bayerHist_info.hist_bypass = 1;
		pm->lscm.bypass = 1;
	}
	return ret;
}

static int dcamoffline_hw_frame_param_set(struct dcam_hw_context *hw_ctx)
{
	int ret = 0, i = 0;
	struct cam_hw_info *hw = NULL;

	hw = hw_ctx->hw;
	for (i = 0; i < DCAM_PATH_MAX; i++) {
		if (!hw_ctx->hw_path[i].need_update)
			continue;
		pr_info("dcam path id %d update, compress_en %d\n", i, hw_ctx->hw_path[i].hw_fbc.compress_en);
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_ADDR_SET, &hw_ctx->hw_path[i].hw_fbc_store);
		else
			hw->dcam_ioctl(hw, DCAM_HW_CFG_STORE_ADDR, &hw_ctx->hw_path[i].hw_store);

		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_SIZE_UPDATE, &hw_ctx->hw_path[i].hw_size);
		hw->dcam_ioctl(hw, DCAM_HW_CFG_PATH_START, &hw_ctx->hw_path[i].hw_start);
		if (hw_ctx->hw_path[i].hw_fbc.compress_en)
			hw->dcam_ioctl(hw, DCAM_HW_CFG_FBC_CTRL, &hw_ctx->hw_path[i].hw_fbc);
	}

	/*use for offline 4in1*/
	hw_ctx->binning.binning_4in1_en = 0;
	hw_ctx->binning.idx = hw_ctx->hw_ctx_id;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BINNING_4IN1_SET, &hw_ctx->binning);

	/* bypass all blks and then set all blks to current pm */
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETSTATIS, hw_ctx->blk_pm);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_BLOCKS_SETALL, hw_ctx->blk_pm);

	hw_ctx->fid++;
	ret = hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_SET, &hw_ctx->hw_fetch);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_RECORD_ADDR, hw_ctx);

	return ret;
}

static int dcamoffline_slice_proc(struct dcam_offline_node *node, struct dcam_isp_k_block *pm)
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

	fetch = &node->fetch;
	hw_ctx = node->hw_ctx;
	hw = hw_ctx->hw;
	slice = &hw_ctx->slice_info;

	for (i = 0; i < slice->slice_num; i++) {
		ret = wait_for_completion_interruptible_timeout(
				&node->slice_done, DCAM_OFFLINE_TIMEOUT);
		if (ret <= 0) {
			pr_err("fail to wait as dcam%d offline timeout. ret: %d\n", hw_ctx->hw_ctx_id, ret);
			trace.type = ABNORMAL_REG_TRACE;
			trace.idx = hw_ctx->hw_ctx_id;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
			return -EFAULT;
		}
		if (atomic_read(&node->status) == STATE_ERROR) {
			pr_info("warning dcam offline slice%d/%d\n", i, slice->slice_num);
			return 0;
		}
		slice->cur_slice = &slice->slice_trim[i];
		pr_info("slice%d/%d proc start\n", i, slice->slice_num);

		if (hw->ip_dcam[hw_ctx->hw_ctx_id]->dcamhw_abt->offline_slice_support && slice->slice_num > 1) {
			slicearg.idx = hw_ctx->hw_ctx_id;
			slicearg.path_id = hw->ip_dcam[hw_ctx->hw_ctx_id]->dcamhw_abt->aux_dcam_path;
			slicearg.fetch = fetch;
			slicearg.cur_slice = slice->cur_slice;
			slicearg.dcam_slice_mode = slice->dcam_slice_mode;
			slicearg.slice_num = slice->slice_num;
			slicearg.slice_count = slice->slice_count;
			slicearg.is_compress = hw_ctx->hw_path[DCAM_PATH_FULL].hw_fbc.compress_en;
			slicearg.st_pack = cam_is_pack(hw_ctx->hw_path[DCAM_PATH_FULL].hw_start.out_fmt);
			slicearg.store_trim = hw_ctx->hw_path[DCAM_PATH_FULL].hw_size.in_trim;
			pr_info("slc%d, (%d %d %d %d)\n", i,
				slice->slice_trim[i].start_x, slice->slice_trim[i].start_y,
				slice->slice_trim[i].size_x, slice->slice_trim[i].size_y);

			hw->dcam_ioctl(hw, DCAM_HW_CFG_SLICE_FETCH_SET, &slicearg);
		}

		mutex_lock(&pm->lsc.lsc_lock);
		if (i == 0)
			ret = dcam_init_lsc(pm, 0);
		else
			ret = dcam_init_lsc_slice(pm, 0);
		if (ret < 0) {
			pr_err("fail to init lsc\n");
			return ret;
		}
		mutex_unlock(&pm->lsc.lsc_lock);

		/* DCAM_CTRL_COEF will always set in dcam_init_lsc() */
		copyarg.id = force_ids;
		copyarg.idx = hw_ctx->hw_ctx_id;
		copyarg.glb_reg_lock = hw_ctx->glb_reg_lock;
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FORCE_COPY, &copyarg);
		os_adapt_time_udelay(500);

		if (i == 0) {
			trace.type = NORMAL_REG_TRACE;
			trace.idx = hw_ctx->hw_ctx_id;
			hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
		}

		dcamoffline_node_ts_cal(node);
		/* start fetch */
		hw->dcam_ioctl(hw, DCAM_HW_CFG_FETCH_START, hw);
	}
	return 0;
}

static struct dcam_isp_k_block *dcamoffline_frm_param_get(struct cam_frame *pframe, struct dcam_offline_node *node)
{
	uint32_t loop = 0;
	struct cam_frame *param_frm = NULL;

	mutex_lock(&node->blkpm_dcam_lock);
	do {
		param_frm = CAM_QUEUE_DEQUEUE(&node->blk_param_queue, struct cam_frame, list);
		if (param_frm) {
			if (param_frm->common.fid == pframe->common.fid) {
				param_frm->common.fid = 0xffff;
				CAM_QUEUE_ENQUEUE(&node->blk_param_queue, &param_frm->list);
				mutex_unlock(&node->blkpm_dcam_lock);
				return param_frm->common.blkparam_info.param_block;
			} else
				CAM_QUEUE_ENQUEUE(&node->blk_param_queue, &param_frm->list);
		} else
			pr_warn("warning:no frame in param queue:%d.\n", node->blk_param_queue.cnt);
	} while (loop++ < node->blk_param_queue.cnt);

	mutex_unlock(&node->blkpm_dcam_lock);
	pr_warn("Warning: not get param fm.\n");
	return NULL;
}

static int dcamoffline_node_frame_start(void *param)
{
	int ret = 0, loop = 0;
	struct dcam_offline_node *node = NULL;
	struct cam_frame *pframe = NULL;
	struct dcam_isp_k_block *pm = NULL;
	struct dcam_pm_context *pm_pctx = NULL;
	struct dcam_offline_slice_info *slice = NULL;
	uint32_t lbuf_width = 0;
	struct dcam_dummy_param dummy_param = {0};

	node = (struct dcam_offline_node *)param;
	if (!node) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	pr_info("dcam offline frame start %px\n", param);
	ret = wait_for_completion_interruptible_timeout(&node->frm_done, DCAM_OFFLINE_TIMEOUT);
	if (ret <= 0) {
		pr_err("fail to wait dcam offline node %px\n", node);
		pframe = cam_buf_manager_buf_dequeue(&node->in_pool, NULL, node->buf_manager_handle);
		if (!pframe) {
			pr_warn("warning: no frame from in_q node %px\n", node);
			return 0;
		}
		ret = -EFAULT;
		node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, node->data_cb_handle);
		return ret;
	}

	os_adapt_time_get_ts(&node->start_ts);
	PERFORMANCE_DEBUG("node_start_time %d.%06d\n", node->start_ts.tv_sec, node->start_ts.tv_nsec / NSEC_PER_USEC);

	loop = 0;
	do {
		ret = dcamoffline_ctx_bind(node);
		if (!ret)
			break;
		pr_info_ratelimited("node %d wait for hw loop %d\n", node->hw_ctx_id, loop);
		os_adapt_time_usleep_range(600, 800);
	} while (++loop < 5000);

	if ((loop == 5000) || (node->hw_ctx_id >= DCAM_HW_CONTEXT_MAX)) {
		pr_err("fail to get hw_ctx_id\n");
		return -1;
	}
	pr_debug("bind hw context %d.\n", node->hw_ctx_id);

	slice = &node->hw_ctx->slice_info;
	ret = dcamoffline_hw_reset(node->hw_ctx);
	if (ret)
		pr_err("fail to reset dcam%d\n", node->hw_ctx_id);

	pframe = dcamoffline_cycle_frame(node);
	if (!pframe)
		goto input_err;

	pm_pctx = &node->pm_ctx;
	pm = &pm_pctx->blk_pm;
	if (node->node_type == CAM_NODE_TYPE_DCAM_OFFLINE) {
		pm = dcamoffline_frm_param_get(pframe, node);
		if (pm)
			pm->idx = node->hw_ctx_id;
		else
			pm = &pm_pctx->blk_pm;
	}
	pm->dev = node->dev;

	if (pframe->common.sensor_time.tv_sec || pframe->common.sensor_time.tv_usec) {
		node->frame_ts[tsid(pframe->common.fid)].tv_sec = pframe->common.sensor_time.tv_sec;
		node->frame_ts[tsid(pframe->common.fid)].tv_nsec = pframe->common.sensor_time.tv_usec * NSEC_PER_USEC;
		node->frame_ts_boot[tsid(pframe->common.fid)] = pframe->common.boot_sensor_time;
	}
	node->hw_ctx->blk_pm = pm;
	ret = dcamoffline_fetch_param_get(node, pframe);
	if (ret) {
		pr_err("fail to get dcam offline fetch param\n");
		goto return_buf;
	}

	/* fetch in node may put it into hw ctx */
	slice->fetch_fmt= node->fetch.fmt;
	dcamslice_needed_info_get(node->hw_ctx, &lbuf_width, pframe->common.width);
	dcam_slice_info_cal(slice, pframe, lbuf_width);
	ret = dcamoffline_hw_statis_work_set(node);
	if (node->port_cfg_cb_func)
		ret = node->port_cfg_cb_func(node->hw_ctx, PORT_CFG_PARAM_GET, node->port_cfg_cb_handle);
	if (ret) {
		pr_err("fail to get dcam offline port param\n");
		goto return_buf;
	}

	dcamoffline_hw_frame_param_set(node->hw_ctx);
	atomic_set(&node->status, STATE_RUNNING);

	if (node->hw_ctx->dummy_slave) {
		dummy_param.enable = ENABLE;
		node->dev->dcam_pipe_ops->dummy_cfg(node->hw_ctx, &dummy_param);
	}

	ret = dcamoffline_slice_proc(node, pm);
	pr_debug("done\n");

	return ret;

return_buf:
	pframe = cam_buf_manager_buf_dequeue(&node->proc_pool, NULL, node->buf_manager_handle);
	if (pframe) {
		cam_buf_manager_buf_status_cfg(&pframe->common.buf, CAM_BUF_STATUS_PUT_IOVA, CAM_BUF_IOMMUDEV_DCAM);
		node->data_cb_func(CAM_CB_DCAM_RET_SRC_BUF, pframe, node->data_cb_handle);
	}
input_err:
	complete(&node->slice_done);
	complete(&node->frm_done);
	dcamoffline_ctx_unbind(node);
	return ret;
}

static int dcamoffline_pmctx_init(struct dcam_offline_node *node)
{
	int ret = 0;
	int iommu_enable = 0;
	struct dcam_pm_context *pm_ctx = NULL;
	struct dcam_isp_k_block *blk_pm = NULL;
	struct dcam_dev_lsc_param *pa = NULL;

	pm_ctx = &node->pm_ctx;
	blk_pm = &pm_ctx->blk_pm;
	pa = &blk_pm->lsc;
	memset(blk_pm, 0, sizeof(struct dcam_isp_k_block));
	if (cam_buf_iommu_status_get(CAM_BUF_IOMMUDEV_DCAM) == 0)
		iommu_enable = 1;
	ret = cam_buf_alloc(&blk_pm->lsc.buf, DCAM_LSC_BUF_SIZE, iommu_enable);
	if (ret)
		goto alloc_fail;

	ret = cam_buf_manager_buf_status_cfg(&blk_pm->lsc.buf, CAM_BUF_STATUS_GET_IOVA_K_ADDR, CAM_BUF_IOMMUDEV_DCAM);
	blk_pm->lsc.buf.bypass_iova_ops = ENABLE;
	if (ret)
		goto map_fail;

	blk_pm->offline = 1;
	blk_pm->idx = node->hw_ctx_id;
	blk_pm->dev = (void *)node->dev;

	atomic_set(&pm_ctx->user_cnt, 1);
	mutex_init(&blk_pm->lsc.lsc_lock);
	mutex_init(&blk_pm->param_lock);

	cam_block_dcam_init(blk_pm);

	pa->weight_tab_x = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
	if (pa->weight_tab_x == NULL) {
		ret = -ENOMEM;
		goto buf_fail;
	}
	pa->weight_tab_y = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
	if (pa->weight_tab_y == NULL) {
		ret = -ENOMEM;
		goto weight_tab_y_fail;
	}

	pa->weight_tab = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
	if (pa->weight_tab == NULL) {
		ret = -ENOMEM;
		goto weight_tab_fail;
	}

	return 0;

weight_tab_fail:
	cam_buf_kernel_sys_kfree(pa->weight_tab_y);
	pa->weight_tab_y = NULL;
weight_tab_y_fail:
	cam_buf_kernel_sys_kfree(pa->weight_tab_x);
	pa->weight_tab_x = NULL;
buf_fail:
	cam_buf_manager_buf_status_cfg(&blk_pm->lsc.buf, CAM_BUF_STATUS_PUT_IOVA_K_ADDR, CAM_BUF_IOMMUDEV_DCAM);
map_fail:
	cam_buf_free(&blk_pm->lsc.buf);
alloc_fail:
	pr_err("failed %d\n", ret);
	return ret;
}

static int dcamoffline_pmctx_deinit(struct dcam_offline_node *node)
{
	struct dcam_pm_context *pm_ctx = NULL;
	struct dcam_isp_k_block *blk_pm = NULL;
	struct dcam_dev_lsc_param *pa = NULL;

	pm_ctx = &node->pm_ctx;
	blk_pm = &pm_ctx->blk_pm;
	pa = &blk_pm->lsc;

	mutex_destroy(&blk_pm->param_lock);
	mutex_destroy(&blk_pm->lsc.lsc_lock);

	cam_buf_manager_buf_status_cfg(&blk_pm->lsc.buf, CAM_BUF_STATUS_MOVE_TO_ALLOC, CAM_BUF_IOMMUDEV_DCAM);
	cam_buf_free(&blk_pm->lsc.buf);
	if(blk_pm->lsc.weight_tab) {
		cam_buf_kernel_sys_kfree(blk_pm->lsc.weight_tab);
		blk_pm->lsc.weight_tab = NULL;
	}
	if(blk_pm->lsc.weight_tab_x) {
		cam_buf_kernel_sys_kfree(blk_pm->lsc.weight_tab_x);
		blk_pm->lsc.weight_tab_x = NULL;
	}
	if(blk_pm->lsc.weight_tab_y) {
		cam_buf_kernel_sys_kfree(blk_pm->lsc.weight_tab_y);
		blk_pm->lsc.weight_tab_y = NULL;
	}

	return 0;
}

static int dcamoffline_pmbuf_init(struct dcam_offline_node *node)
{
	int ret = 0;
	uint32_t i = 0, iommu_enable = 0;
	struct dcam_dev_lsc_param *pa = NULL;
	struct cam_frame *param_frm = NULL;

	if (!node) {
		pr_err("fail to get ptr handle:%p.\n", node);
		return -1;
	}

	for (i = 0; i < DCAM_OFFLINE_PARAM_Q_LEN; i++) {
		param_frm = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
		param_frm->common.blkparam_info.param_block = cam_buf_kernel_sys_vzalloc(sizeof(struct dcam_isp_k_block));
		param_frm->common.fid = 0xffff;
		if (param_frm->common.blkparam_info.param_block) {
			cam_block_dcam_init(param_frm->common.blkparam_info.param_block);
			if (cam_buf_iommu_status_get(CAM_BUF_IOMMUDEV_DCAM) == 0)
				iommu_enable = 1;

			ret = cam_buf_alloc(&param_frm->common.blkparam_info.param_block->lsc.buf, DCAM_LSC_BUF_SIZE, iommu_enable);
			if (ret)
				goto alloc_fail;
			pa = &param_frm->common.blkparam_info.param_block->lsc;
			mutex_init(&param_frm->common.blkparam_info.param_block->lsc.lsc_lock);
			mutex_init(&param_frm->common.blkparam_info.param_block->param_lock);
			ret = cam_buf_manager_buf_status_cfg(&param_frm->common.blkparam_info.param_block->lsc.buf, CAM_BUF_STATUS_GET_IOVA_K_ADDR, CAM_BUF_IOMMUDEV_DCAM);
			if (ret)
				goto map_fail;
			pa->weight_tab_x = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
			if (pa->weight_tab_x == NULL) {
				ret = -ENOMEM;
				goto weight_tab_x_fail;
			}
			pa->weight_tab_y = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
			if (pa->weight_tab_y == NULL) {
				ret = -ENOMEM;
				goto weight_tab_y_fail;
			}

			pa->weight_tab = cam_buf_kernel_sys_kzalloc(DCAM_LSC_WEIGHT_TAB_SIZE, GFP_KERNEL);
			if (pa->weight_tab == NULL) {
				ret = -ENOMEM;
				goto weight_tab_fail;
			}
			ret = CAM_QUEUE_ENQUEUE(&node->blk_param_queue, &param_frm->list);
			if (ret) {
				pr_warn("Warning:fail to enqueue empty queue.\n");
				cam_buf_manager_buf_status_cfg(&param_frm->common.blkparam_info.param_block->lsc.buf, CAM_BUF_STATUS_MOVE_TO_ALLOC, CAM_BUF_IOMMUDEV_MAX);
				cam_buf_free(&param_frm->common.blkparam_info.param_block->lsc.buf);
				cam_buf_kernel_sys_vfree(param_frm->common.blkparam_info.param_block);
				cam_queue_empty_frame_put(param_frm);
			}
		} else {
			cam_queue_empty_frame_put(param_frm);
			pr_warn("Warning: alloc dcam pm buf fail.\n");
			return -1;
		}
	}

	return 0;

weight_tab_fail:
	cam_buf_kernel_sys_kfree(pa->weight_tab_y);
	pa->weight_tab_y = NULL;
weight_tab_y_fail:
	cam_buf_kernel_sys_kfree(pa->weight_tab_x);
	pa->weight_tab_x = NULL;
weight_tab_x_fail:
	cam_buf_manager_buf_status_cfg(&param_frm->common.blkparam_info.param_block->lsc.buf, CAM_BUF_STATUS_MOVE_TO_ALLOC, CAM_BUF_IOMMUDEV_MAX);
map_fail:
	pr_err("fail to map lsc buf.\n");
	cam_buf_free(&param_frm->common.blkparam_info.param_block->lsc.buf);
alloc_fail:
	pr_err("fail to alloc lsc buf.\n");
	cam_buf_kernel_sys_vfree(param_frm->common.blkparam_info.param_block);
	cam_queue_empty_frame_put(param_frm);

	return -1;
}

static void dcamoffline_pmbuf_destroy(void *param)
{
	struct cam_frame *param_frm = NULL;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	param_frm = (struct cam_frame *)param;

	mutex_destroy(&param_frm->common.blkparam_info.param_block->param_lock);
	mutex_destroy(&param_frm->common.blkparam_info.param_block->lsc.lsc_lock);

	cam_buf_manager_buf_status_cfg(&param_frm->common.blkparam_info.param_block->lsc.buf, CAM_BUF_STATUS_MOVE_TO_ALLOC, CAM_BUF_IOMMUDEV_DCAM);
	cam_buf_free(&param_frm->common.blkparam_info.param_block->lsc.buf);
	if(param_frm->common.blkparam_info.param_block->lsc.weight_tab) {
		cam_buf_kernel_sys_kfree(param_frm->common.blkparam_info.param_block->lsc.weight_tab);
		param_frm->common.blkparam_info.param_block->lsc.weight_tab = NULL;
	}
	if(param_frm->common.blkparam_info.param_block->lsc.weight_tab_x) {
		cam_buf_kernel_sys_kfree(param_frm->common.blkparam_info.param_block->lsc.weight_tab_x);
		param_frm->common.blkparam_info.param_block->lsc.weight_tab_x = NULL;
	}
	if(param_frm->common.blkparam_info.param_block->lsc.weight_tab_y) {
		cam_buf_kernel_sys_kfree(param_frm->common.blkparam_info.param_block->lsc.weight_tab_y);
		param_frm->common.blkparam_info.param_block->lsc.weight_tab_y = NULL;
	}
	cam_buf_kernel_sys_vfree(param_frm->common.blkparam_info.param_block);
	param_frm->common.blkparam_info.param_block = NULL;
	cam_queue_empty_frame_put(param_frm);
}

static int dcamoffline_recovery(void *handle)
{
	struct cam_node_cfg_param node_param = {0};
	struct camera_buf_get_desc buf_desc = {0};
	struct dcam_offline_port *dcam_port = NULL;
	struct cam_frame *frame = NULL;
	struct dcam_offline_node *node = NULL;
	int ret = 0, is_frm_port = 0;

	node = VOID_PTR_TO(handle, struct dcam_offline_node);
	pr_info("offline node recovery start\n");

	atomic_set(&node->status, STATE_ERROR);
	if (node->hw_ctx->slice_info.slice_num > 0 && node->hw_ctx->slice_info.slice_count > 1)
		complete(&node->slice_done);
	complete(&node->slice_done);
	CAM_QUEUE_FOR_EACH_ENTRY(dcam_port, &node->port_queue.head, list) {
		node_param.port_id = dcam_port->port_id;
		ret = node->port_cfg_cb_func(&node_param, PORT_CFG_BUFFER_GET, node->port_cfg_cb_handle);
		if (ret)
			pr_err("fail to get dcam offline port param\n");
		frame = (struct cam_frame *)(node_param.param);
		is_frm_port = (dcam_port->port_id == PORT_OFFLINE_FULL_OUT || dcam_port->port_id == PORT_OFFLINE_BIN_OUT || dcam_port->port_id == PORT_OFFLINE_RAW_OUT);
		if (frame && is_frm_port)
			cam_buf_manager_buf_status_cfg(&frame->common.buf, CAM_BUF_STATUS_PUT_IOVA, CAM_BUF_IOMMUDEV_DCAM);
		buf_desc.buf_ops_cmd = CAM_BUF_STATUS_GET_IOVA;
		buf_desc.mmu_type = CAM_BUF_IOMMUDEV_DCAM;
		buf_desc.q_ops_cmd = CAM_QUEUE_FRONT;
		ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, frame, &buf_desc, node->buf_manager_handle);
	}

	frame = cam_buf_manager_buf_dequeue(&node->proc_pool, NULL, node->buf_manager_handle);
	if (frame)
		cam_buf_manager_buf_status_cfg(&frame->common.buf, CAM_BUF_STATUS_PUT_IOVA, CAM_BUF_IOMMUDEV_DCAM);
	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_MOVE_TO_ION;
	buf_desc.q_ops_cmd = CAM_QUEUE_FRONT;
	ret = cam_buf_manager_buf_enqueue(&node->in_pool, frame, &buf_desc, node->buf_manager_handle);
	dcamoffline_ctx_unbind(node);
	complete(&node->frm_done);
	if (ret == 0)
		complete(&node->thread.thread_com);
	pr_info("offline node recovery done\n");
	return 0;
}

int dcam_offline_node_port_insert(struct dcam_offline_node *node, void *param)
{
	struct dcam_offline_port *q_port = NULL;
	struct dcam_offline_port *new_port = NULL;
	uint32_t is_exist = 0;

	new_port = (struct dcam_offline_port *)param;

	CAM_QUEUE_FOR_EACH_ENTRY(q_port, &node->port_queue.head, list) {
		if (new_port == q_port) {
			is_exist = 1;
			break;
		}
	}

	if (!is_exist)
		CAM_QUEUE_ENQUEUE(&node->port_queue, &new_port->list);

	return 0;
}

struct dcam_offline_port *dcam_offline_node_port_get(struct dcam_offline_node *node, uint32_t port_id)
{
	struct dcam_offline_port *dcam_port = NULL;

	if (!node) {
		pr_err("fail to get valid dcam_online_node\n");
		return NULL;
	}

	CAM_QUEUE_FOR_EACH_ENTRY(dcam_port, &node->port_queue.head, list) {
		if (dcam_port->port_id == port_id)
			return dcam_port;
	}

	pr_err("fail to get dcam port %s\n", cam_port_dcam_offline_out_id_name_get(port_id));
	return NULL;
}

int dcam_offline_node_statis_cfg(struct dcam_offline_node *node, void *param)
{
	int ret = 0;
	struct dcam_statis_param *statis_param = NULL;

	statis_param = (struct dcam_statis_param *)param;

	switch (statis_param->statis_cmd) {
	case DCAM_IOCTL_CFG_STATIS_BUF:
		ret = cam_offline_statis_dcam_port_buffer_cfg(node, param);
		break;
	case DCAM_IOCTL_DEINIT_STATIS_Q:
		ret = cam_offline_statis_dcam_port_bufferq_deinit(node);
		break;
	default:
		pr_err("fail to get a known cmd: %d\n", statis_param->statis_cmd);
		ret = -EFAULT;
		break;
	}
	return ret;
}

int dcam_offline_node_blkpm_fid_set(struct dcam_offline_node *node, void *param)
{
	uint32_t ret = 0, fid = 0, loop = 0;
	struct cam_frame *param_frame = NULL;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	fid = *((uint32_t *)param);
	mutex_lock(&node->blkpm_dcam_lock);
	do {
		param_frame = CAM_QUEUE_DEQUEUE(&node->blk_param_queue, struct cam_frame, list);
		if (param_frame) {
			if (param_frame->common.fid == 0xffff) {
				param_frame->common.fid = fid;
				node->pm = param_frame->common.blkparam_info.param_block;
				CAM_QUEUE_ENQUEUE(&node->blk_param_queue, &param_frame->list);
				break;
			} else
				CAM_QUEUE_ENQUEUE(&node->blk_param_queue, &param_frame->list);
		} else {
			pr_warn("Warning:no frame in dcam blk empty queue:%d.\n", node->blk_param_queue.cnt);
			ret = -EFAULT;
			break;
		}
	} while (loop++ < node->blk_param_queue.cnt);

	mutex_unlock(&node->blkpm_dcam_lock);
	return ret;
}

void dcam_offline_node_param_ptr_reset(struct dcam_offline_node *node)
{
	if (!node) {
		pr_err("fail to get valid param %px.\n", node);
		return;
	}

	node->pm = NULL;
}

int dcam_offline_node_blk_param_set(struct dcam_offline_node *node, void *param)
{
	int ret = 0, index = 0;
	struct isp_io_param *blk_param = NULL;
	struct dcam_isp_k_block *pm = NULL;
	func_cam_cfg_param cfg_fun_ptr = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_block_func_get blk_func = {0};

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	blk_param = (struct isp_io_param *)param;
	pm = &node->pm_ctx.blk_pm;
	if (node->pm) {
		pm = node->pm;
		pm->offline = 1;
	}
	hw = node->dev->hw;

	index = blk_param->sub_block;
	blk_func.index = index;
	hw->cam_ioctl(hw, CAM_HW_GET_BLK_FUN, &blk_func);
	if (blk_func.cam_entry != NULL &&
		blk_func.cam_entry->sub_block == blk_param->sub_block) {
		cfg_fun_ptr = blk_func.cam_entry->cfg_func;
	} else {
		pr_err("fail to check param, io_param->sub_block = %d, error\n", blk_param->sub_block);
	}
	if (cfg_fun_ptr == NULL) {
		pr_debug("block %d not supported.\n", blk_param->sub_block);
		goto exit;
	}

	ret = cfg_fun_ptr(blk_param, pm);

exit:
	return ret;
}

int dcam_offline_node_request_proc(struct dcam_offline_node *node, void *param)
{
	int ret = 0;
	struct cam_node_cfg_param *node_param = NULL;
	struct cam_frame *src_frame = NULL;
	struct camera_buf_get_desc buf_desc = {0};
	struct cam_pipeline *pipeline = NULL;
	struct cam_node *cam_node = NULL;

	if (!node || !param) {
		pr_err("fail to get valid param %px %px\n", node, param);
		return -EFAULT;
	}

	node_param = (struct cam_node_cfg_param *)param;
	/* Temp change for cam buf manage change not ready, need change back later */
	src_frame = (struct cam_frame *)node_param->param;
	src_frame->common.priv_data = node;
	buf_desc.buf_ops_cmd = CAM_BUF_STATUS_MOVE_TO_ION;
	cam_node = (struct cam_node *)node->data_cb_handle;
	pipeline = (struct cam_pipeline *)cam_node->data_cb_handle;

	if (pipeline->debug_log_switch)
		pr_info("pipeline_type %s, fid %d, ch_id %d, buf %x, w %d, h %d, pframe->common.is_reserved %d, compress_en %d\n",
			pipeline->pipeline_graph->name, src_frame->common.fid, src_frame->common.channel_id, src_frame->common.buf.mfd,
			src_frame->common.width, src_frame->common.height, src_frame->common.is_reserved, src_frame->common.is_compressed);

	ret = cam_buf_manager_buf_enqueue(&node->in_pool, src_frame, &buf_desc, node->buf_manager_handle);
	if (ret == 0)
		complete(&node->thread.thread_com);
	else {
		if (src_frame->common.buf.type == CAM_BUF_USER)
			cam_buf_manager_buf_status_cfg(&src_frame->common.buf, CAM_BUF_STATUS_MOVE_TO_ALLOC, CAM_BUF_IOMMUDEV_MAX);
		cam_queue_empty_frame_put(src_frame);
		pr_err("fail to enqueue dcam offline frame\n");
	}
	return ret;
}

void dcam_offline_node_close(void *handle)
{
	int ret = 0;
	struct dcam_offline_node *node = NULL;

	if (!handle) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}
	node = (struct dcam_offline_node *)handle;
	if (node->thread.thread_task) {
		camthread_stop(&node->thread);
		camthread_stop(&node->recovery_thread);
		/* wait for last frame done */
		ret = wait_for_completion_timeout(&node->frm_done, ISP_CONTEXT_TIMEOUT);
		if (ret == 0)
			pr_err("fail to wait node %d, timeout.\n", node->node_id);
		else
			pr_info("wait time %d ms\n", ret);
		CAM_QUEUE_CLEAN(&node->port_queue, struct dcam_offline_port, list);
	}
}

void *dcam_offline_node_get(uint32_t node_id, struct dcam_offline_node_desc *param)
{
	int ret = 0;
	struct dcam_offline_node *node = NULL;
	struct cam_thread_info *thrd = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	pr_info("node id %d node_dev %px\n", node_id, *param->node_dev);
	if (*param->node_dev == NULL) {
		node = cam_buf_kernel_sys_vzalloc(sizeof(struct dcam_offline_node));
		if (!node) {
			pr_err("fail to get valid dcam offline node\n");
			return NULL;
		}
	} else {
		node = *param->node_dev;
		pr_info("dcam offline node has been alloc %p\n", node);
		goto exit;
	}

	node->buf_manager_handle = param->buf_manager_handle;
	init_completion(&node->frm_done);
	complete(&node->frm_done);
	init_completion(&node->slice_done);
	complete(&node->slice_done);
	mutex_init(&node->blkpm_dcam_lock);
	CAM_QUEUE_INIT(&node->blk_param_queue, DCAM_OFFLINE_PARAM_Q_LEN, dcamoffline_pmbuf_destroy);
	CAM_QUEUE_INIT(&node->port_queue, PORT_DCAM_OFFLINE_OUT_MAX, NULL);

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OFFLINE_IN_Q_LEN, node->buf_manager_handle);
	if (ret <= 0) {
		pr_err("fail to reg in pool for dcam offline node\n");
		cam_buf_kernel_sys_vfree(node);
		return NULL;
	}
	node->in_pool.private_pool_id = ret;

	ret = cam_buf_manager_pool_reg(NULL, DCAM_OFFLINE_PROC_Q_LEN, node->buf_manager_handle);
	if (ret <= 0) {
		pr_err("fail to reg proc pool for dcam offline node\n");
		cam_buf_manager_pool_unreg(&node->in_pool, node->buf_manager_handle);
		cam_buf_kernel_sys_vfree(node);
		return NULL;
	}
	node->proc_pool.private_pool_id = ret;
	pr_debug("reg pool %d %d\n", node->in_pool.private_pool_id, node->proc_pool.private_pool_id);

	node->dev = param->dev;
	node->dcam_idx = param->dcam_idx;
	node->fetch.fmt = param->fetch_fmt;
	node->fetch.pattern = param->pattern;
	node->fetch.endian = param->endian;
	pr_debug("pattern %d, endian %d\n", node->fetch.pattern, node->fetch.endian);
	node->node_id = node_id;
	node->node_type = param->node_type;
	node->hw_ctx_id = DCAM_HW_CONTEXT_MAX;
	node->statis_en = param->statis_en;
	if (!node->data_cb_func) {
		node->data_cb_func = param->data_cb_func;
		node->data_cb_handle = param->data_cb_handle;
	}
	if (!node->port_cfg_cb_func) {
		node->port_cfg_cb_func = param->port_cfg_cb_func;
		node->port_cfg_cb_handle = param->port_cfg_cb_handle;
	}

	if (node->node_type == CAM_NODE_TYPE_DCAM_OFFLINE) {
		ret = dcamoffline_pmbuf_init(node);
		if (ret)
			pr_warn("Warning:Not alloc pm buffer, take ctx recieve pm.\n");
	}
	ret = dcamoffline_pmctx_init(node);
	if (ret)
		goto exit;

	thrd = &node->thread;
	sprintf(thrd->thread_name, "dcam_offline_node%d", node->node_id);
	ret = camthread_create(node, thrd, dcamoffline_node_frame_start);
	if (unlikely(ret != 0)) {
		pr_err("fail to create dcam offline node thread\n");
		return NULL;
	}
	thrd = &node->recovery_thread;
	sprintf(thrd->thread_name, "dcam_offline_recovery%d", node->node_id);
	ret = camthread_create(node, thrd, dcamoffline_recovery);
	if (unlikely(ret != 0)) {
		pr_err("fail to create dcam offline node thread\n");
		return NULL;
	}
	*param->node_dev = node;

exit:
	atomic_inc(&node->user_cnt);
	pr_info("node id %d\n", node->node_id);
	return node;
}

void dcam_offline_node_put(struct dcam_offline_node *node)
{
	int loop = 0;
	if (!node) {
		pr_err("fail to get invalid node ptr\n");
		return;
	}

	if (atomic_dec_return(&node->user_cnt) != 0) {
		pr_debug("dcam offline user cnt is %d\n", atomic_read(&node->user_cnt));
		return;
	}

	while ((node->hw_ctx || node->in_irq_proc) && loop < 1000) {
		pr_debug("ctx % in irq. wait %d\n", node->node_id, loop);
		loop++;
		os_adapt_time_udelay(1000);
	};
	if (node->hw_ctx) {
		pr_warn("warning: offline node unbind or still in_irq_proc %d\n", node->in_irq_proc);
		dcamoffline_ctx_unbind(node);
	}
	dcamoffline_pmctx_deinit(node);
	mutex_destroy(&node->blkpm_dcam_lock);
	CAM_QUEUE_CLEAN(&node->blk_param_queue, struct cam_frame, list);

	cam_buf_manager_pool_unreg(&node->in_pool, node->buf_manager_handle);
	cam_buf_manager_pool_unreg(&node->proc_pool, node->buf_manager_handle);
	node->data_cb_func = NULL;
	node->data_cb_handle = NULL;
	node->port_cfg_cb_func = NULL;
	node->port_cfg_cb_handle = NULL;
	pr_info("dcam offline node %d put success\n", node->node_id);
	cam_buf_kernel_sys_vfree(node);
	node = NULL;
}
