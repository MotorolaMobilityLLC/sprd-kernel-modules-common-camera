/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>

#include "sprd_isp_hw.h"
#include "sprd_img.h"
#include <video/sprd_mmsys_pw_domain.h>
#include <sprd_mm.h>
#include <linux/sprd_ion.h>
#include  "cam_trusty.h"

#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"
#include "cam_block.h"
#include "cam_debugger.h"

#include "dcam_interface.h"

#include "isp_int.h"
#include "isp_reg.h"

#include "isp_interface.h"
#include "isp_core.h"
#include "isp_path.h"
#include "isp_slice.h"

#include "isp_cfg.h"
#include "isp_fmcu.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_CORE: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

unsigned long *isp_cfg_poll_addr[ISP_CONTEXT_SW_NUM];
static int sprd_isp_put_context(
	void *isp_handle, int ctx_id);
static int sprd_isp_put_path(
	void *isp_handle, int ctx_id, int path_id);
static int isp_slice_ctx_init(struct isp_pipe_context *pctx, uint32_t *multi_slice);
static DEFINE_MUTEX(isp_pipe_dev_mutex);
struct isp_pipe_dev *s_isp_dev;
uint32_t s_dbg_linebuf_len = ISP_LINE_BUFFER_W;
extern int s_dbg_work_mode;

static void free_offline_pararm(void *param)
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

void isp_unmap_frame(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}
	frame = (struct camera_frame *)param;
	cambuf_iommu_unmap(&frame->buf);
}

void isp_ret_out_frame(void *param)
{
	struct camera_frame *frame;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	path = (struct isp_path_desc *)frame->priv_data;

	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);

	if (frame->is_reserved)
		camera_enqueue(
			&path->reserved_buf_queue, frame);
	else {
		pctx = path->attach_ctx;
		cambuf_iommu_unmap(&frame->buf);
		pctx->isp_cb_func(
			ISP_CB_RET_DST_BUF,
			frame, pctx->cb_priv_data);
	}
}

void isp_ret_src_frame(void *param)
{
	struct camera_frame *frame;
	struct isp_pipe_context *pctx;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	pctx = (struct isp_pipe_context *)frame->priv_data;
	pr_debug("frame %p, ch_id %d, buf_fd %d\n",
		frame, frame->channel_id, frame->buf.mfd[0]);
	free_offline_pararm(frame->param_data);
	frame->param_data = NULL;
	cambuf_iommu_unmap(&frame->buf);
	pctx->isp_cb_func(
		ISP_CB_RET_SRC_BUF,
		frame, pctx->cb_priv_data);
}

void isp_destroy_reserved_buf(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (unlikely(frame->is_reserved == 0)) {
		pr_err("fail to get frame reserved buffer.\n");
		return;
	}
	/* is_reserved:
	 *  1:  basic mapping reserved buffer;
	 *  2:  copy of reserved buffer.
	 */
	if (frame->is_reserved == 1) {
		cambuf_iommu_unmap(&frame->buf);
		cambuf_put_ionbuf(&frame->buf);
	}
	put_empty_frame(frame);
}

void isp_destroy_statis_buf(void *param)
{
	struct camera_frame *frame;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	put_empty_frame(frame);
}

int isp_adapt_blkparam(struct isp_pipe_context *pctx)
{
	uint32_t new_width, old_width;
	uint32_t new_height, old_height;
	uint32_t crop_start_x, crop_start_y;
	uint32_t crop_end_x, crop_end_y;
	struct img_trim *src_trim;
	struct img_size *dst = &pctx->original.dst_size;

	if (pctx->original.src_size.w > 0) {
		/* for input scaled image */
		src_trim = &pctx->original.src_trim;
		new_width = dst->w;
		new_height = dst->h;
		old_width = src_trim->size_x;
		old_height = src_trim->size_y;
	} else {
		src_trim = &pctx->input_trim;
		old_width = src_trim->size_x;
		old_height = src_trim->size_y;
		new_width = old_width;
		new_height = old_height;
	}
	crop_start_x = src_trim->start_x;
	crop_start_y = src_trim->start_y;
	crop_end_x = src_trim->start_x + src_trim->size_x - 1;
	crop_end_y = src_trim->start_y + src_trim->size_y - 1;

	pr_debug("crop %d %d %d %d, size(%d %d) => (%d %d)\n",
		crop_start_x, crop_start_y, crop_end_x, crop_end_y,
		old_width, old_height, new_width, new_height);

	isp_k_update_nlm(pctx->ctx_id, &pctx->isp_k_param,
		new_width, old_width, new_height, old_height);
	isp_k_update_ynr(pctx->ctx_id, &pctx->isp_k_param,
		new_width, old_width, new_height, old_height);

	if (pctx->mode_3dnr != MODE_3DNR_OFF)
		isp_k_update_3dnr(pctx->ctx_id, &pctx->isp_k_param,
			 new_width, old_width, new_height, old_height);

	return 0;
}

static int isp_update_hist_roi(struct isp_pipe_context *pctx)
{
	int ret = 0;
	uint32_t val;
	struct isp_dev_hist2_info hist2_info;
	struct isp_fetch_info *fetch = &pctx->fetch;

	pr_debug("hist_roi w[%d] h[%d]\n", fetch->in_trim.size_x, fetch->in_trim.size_y);

	hist2_info.hist_roi.start_x = 0;
	hist2_info.hist_roi.start_y = 0;

	hist2_info.hist_roi.end_x = fetch->in_trim.size_x - 1;
	hist2_info.hist_roi.end_y = fetch->in_trim.size_y - 1;

	val = (hist2_info.hist_roi.start_y & 0xFFFF) | ((hist2_info.hist_roi.start_x & 0xFFFF) << 16);
	ISP_REG_WR(pctx->ctx_id, ISP_HIST2_ROI_S0, val);

	val = (hist2_info.hist_roi.end_y & 0xFFFF) | ((hist2_info.hist_roi.end_x & 0xFFFF) << 16);
	ISP_REG_WR(pctx->ctx_id, ISP_HIST2_ROI_E0, val);

	return ret;
}
static int isp_3dnr_process_frame_previous(struct isp_pipe_context *pctx,
					   struct camera_frame *pframe)
{
	if (!pctx || !pframe) {
		pr_err("fail to get valid parameter pctx %p pframe %p\n",
			pctx, pframe);
		return -EINVAL;
	}

	if (pctx->mode_3dnr == MODE_3DNR_OFF)
		return 0;

	/*  Check Zoom or not */
	if ((pctx->input_trim.size_x != pctx->nr3_ctx.width) ||
	    (pctx->input_trim.size_y != pctx->nr3_ctx.height)) {
		pr_debug("frame size changed, reset 3dnr blending\n");

		/*
		 * 1. reset blending count, so
		 *	cnt = 0, DONOT fetch ref
		 * 2. MUST before get output buffer, because
		 *      USING blend cnt to choose reserved buf or HAL buf
		 */
		pctx->nr3_ctx.blending_cnt = 0;
	}

	return 0;
}

static int isp_3dnr_process_frame(struct isp_pipe_context *pctx,
				  struct camera_frame *pframe)
{
	struct isp_3dnr_ctx_desc *nr3_ctx;

	struct dcam_frame_synchronizer *fsync = NULL;

	fsync = (struct dcam_frame_synchronizer *)pframe->sync_data;

	if (fsync) {
		pr_debug("id %u, valid %d, x %d, y %d, w %u, h %u\n",
			 fsync->index, fsync->nr3_me.valid,
			 fsync->nr3_me.mv_x, fsync->nr3_me.mv_y,
			 fsync->nr3_me.src_width, fsync->nr3_me.src_height);
	}

	nr3_ctx = &pctx->nr3_ctx;

	nr3_ctx->width  = pctx->input_trim.size_x;
	nr3_ctx->height = pctx->input_trim.size_y;

	pr_debug("input.w[%d], input.h[%d], trim.w[%d], trim.h[%d]\n",
		pctx->input_size.w, pctx->input_size.h,
		pctx->input_trim.size_x, pctx->input_trim.size_y);

	switch (pctx->mode_3dnr) {
	case MODE_3DNR_PRE:
		nr3_ctx->type = NR3_FUNC_PRE;

		if (fsync && fsync->nr3_me.valid) {
			nr3_ctx->mv.mv_x = fsync->nr3_me.mv_x;
			nr3_ctx->mv.mv_y = fsync->nr3_me.mv_y;
			nr3_ctx->mvinfo = &fsync->nr3_me;

			isp_3dnr_conversion_mv(nr3_ctx);
		} else {
			pr_err("fail to get binning path mv, set default 0\n");
			nr3_ctx->mv.mv_x = 0;
			nr3_ctx->mv.mv_y = 0;
			nr3_ctx->mvinfo = NULL;
		}

		isp_3dnr_gen_config(nr3_ctx);
		isp_3dnr_config_param(nr3_ctx,
				      &pctx->isp_k_param,
				      pctx->ctx_id,
				      NR3_FUNC_PRE);

		pr_debug("MODE_3DNR_PRE frame type: %d from dcam binning path mv[%d, %d]!\n.",
			 pframe->channel_id,
			 nr3_ctx->mv.mv_x,
			 nr3_ctx->mv.mv_y);
		break;

	case MODE_3DNR_CAP:
		nr3_ctx->type = NR3_FUNC_CAP;

		if (fsync && fsync->nr3_me.valid) {
			nr3_ctx->mv.mv_x = fsync->nr3_me.mv_x;
			nr3_ctx->mv.mv_y = fsync->nr3_me.mv_y;
			if (pctx->input_size.w != pctx->input_trim.size_x ||
				pctx->input_size.h != pctx->input_trim.size_y) {
				nr3_ctx->mvinfo = &fsync->nr3_me;
				isp_3dnr_conversion_mv(nr3_ctx);
			}
		} else {
			pr_err("fail to get full path mv, set default 0\n");
			nr3_ctx->mv.mv_x = 0;
			nr3_ctx->mv.mv_y = 0;
		}

		isp_3dnr_gen_config(nr3_ctx);
		isp_3dnr_config_param(nr3_ctx,
				      &pctx->isp_k_param,
				      pctx->ctx_id,
				      NR3_FUNC_CAP);

		pr_debug("MODE_3DNR_CAP frame type: %d from dcam binning path mv[%d, %d]!\n.",
			 pframe->channel_id,
			 nr3_ctx->mv.mv_x,
			 nr3_ctx->mv.mv_y);
		break;

	case MODE_3DNR_OFF:
		/* default: bypass 3dnr */
		nr3_ctx->type = NR3_FUNC_OFF;
		isp_3dnr_bypass_config(pctx->ctx_id);
		pr_debug("isp_offline_start_frame MODE_3DNR_OFF\n");
		break;
	default:
		/* default: bypass 3dnr */
		nr3_ctx->type = NR3_FUNC_OFF;
		isp_3dnr_bypass_config(pctx->ctx_id);
		pr_debug("isp_offline_start_frame default\n");
		break;
	}

	if (fsync)
		dcam_if_release_sync(fsync, pframe);

	return 0;
}

static int isp_ltm_process_frame_previous(struct isp_pipe_context *pctx,
					  struct camera_frame *pframe)
{
	if (!pctx || !pframe) {
		pr_err("fail to get valid parameter pctx %p pframe %p\n",
			pctx, pframe);
		return -EINVAL;
	}

	/*
	 * Only preview path care of frame size changed
	 * Because capture path, USING hist from preview path
	 */
	if (pctx->mode_ltm != MODE_LTM_PRE)
		return 0;

	/*  Check Zoom or not */
	if ((pctx->input_trim.size_x != pctx->ltm_ctx.frame_width) ||
	    (pctx->input_trim.size_y != pctx->ltm_ctx.frame_height)) {
		pr_debug("frame size changed, bypass ltm map\n");

		/* 1. hists from preview path always on
		 * 2. map will be off one time in preview case
		 */
		pctx->ltm_ctx.map.bypass = 1;
	} else {
		pctx->ltm_ctx.map.bypass = 0;
	}

	return 0;
}

static int isp_ltm_process_frame(struct isp_pipe_context *pctx,
				 struct camera_frame *pframe)
{
	int ret = 0;

	/* pre & cap */
	pctx->ltm_ctx.type = pctx->mode_ltm;
	pctx->ltm_ctx.fid	   = pframe->fid;
	pctx->ltm_ctx.frame_width  = pctx->input_trim.size_x;
	pctx->ltm_ctx.frame_height = pctx->input_trim.size_y;
	pctx->ltm_ctx.isp_pipe_ctx_id = pctx->ctx_id;

	/* pre & cap */
	ret = isp_ltm_gen_frame_config(&pctx->ltm_ctx);
	if (ret == -1) {
		pctx->mode_ltm = MODE_LTM_OFF;
		pr_err("fail to cfg LTM frame, DISABLE\n");
	}

	pr_debug("type[%d], fid[%d], frame_width[%d], frame_height[%d], isp_pipe_ctx_id[%d]\n",
		pctx->ltm_ctx.type,
		pctx->ltm_ctx.fid,
		pctx->ltm_ctx.frame_width,
		pctx->ltm_ctx.frame_height,
		pctx->ltm_ctx.isp_pipe_ctx_id);

	pr_debug("frame_height_stat[%d], frame_width_stat[%d],\
		tile_num_x_minus[%d], tile_num_y_minus[%d],\
		tile_width[%d], tile_height[%d]\n",
		pctx->ltm_ctx.frame_height_stat,
		pctx->ltm_ctx.frame_width_stat,
		pctx->ltm_ctx.hists.tile_num_x_minus,
		pctx->ltm_ctx.hists.tile_num_y_minus,
		pctx->ltm_ctx.hists.tile_width,
		pctx->ltm_ctx.hists.tile_height);

	return ret;
}

static int isp_afbc_store(struct isp_path_desc *path)
{
	uint32_t w_tile_num = 0, h_tile_num = 0;
	uint32_t pad_width = 0, pad_height = 0;
	uint32_t header_size = 0, tile_data_size = 0;
	uint32_t header_addr = 0;
	struct isp_afbc_store_info *afbc_store_info = NULL;

	if (!path) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	afbc_store_info = &path->afbc_store;
	afbc_store_info->size.w = path->dst.w;
	afbc_store_info->size.h = path->dst.h;

	pad_width = (afbc_store_info->size.w + AFBC_PADDING_W_YUV420 - 1)
		/ AFBC_PADDING_W_YUV420 * AFBC_PADDING_W_YUV420;
	pad_height = (afbc_store_info->size.h + AFBC_PADDING_H_YUV420 - 1)
		/ AFBC_PADDING_H_YUV420 * AFBC_PADDING_H_YUV420;

	w_tile_num = pad_width / AFBC_PADDING_W_YUV420;
	h_tile_num = pad_height / AFBC_PADDING_H_YUV420;

	header_size = w_tile_num * h_tile_num * AFBC_HEADER_SIZE;
	tile_data_size = w_tile_num * h_tile_num * AFBC_PAYLOAD_SIZE;
	header_addr = afbc_store_info->yheader;

	afbc_store_info->header_offset = (header_size + 1024 - 1) / 1024 * 1024;

	afbc_store_info->yheader = header_addr;

	afbc_store_info->yaddr = afbc_store_info->yheader +
		afbc_store_info->header_offset;

	afbc_store_info->tile_number_pitch = w_tile_num;

	pr_debug("afbc w_tile_num = %d, h_tile_num = %d\n",
		w_tile_num,h_tile_num);
	pr_debug("afbc header_offset = %x, yheader = %x, yaddr = %x\n",
		afbc_store_info->header_offset,
		afbc_store_info->yheader,
		afbc_store_info->yaddr);

	return 0;
}

static int isp_update_offline_param(
	struct isp_pipe_context *pctx,
	struct isp_offline_param *in_param)
{
	int ret = 0;
	int i;
	struct img_size *src_new = NULL;
	struct img_trim path_trim;
	struct isp_path_desc *path;
	struct isp_ctx_size_desc cfg;
	uint32_t update[ISP_SPATH_NUM] = {
			ISP_PATH0_TRIM,	ISP_PATH1_TRIM, ISP_PATH2_TRIM};

	if (in_param->valid & ISP_SRC_SIZE) {
		memcpy(&pctx->original, &in_param->src_info,
			sizeof(pctx->original));
		cfg.src = in_param->src_info.dst_size;
		cfg.crop.start_x = 0;
		cfg.crop.start_y = 0;
		cfg.crop.size_x = cfg.src.w;
		cfg.crop.size_y = cfg.src.h;
		ret = isp_cfg_ctx_size(pctx, &cfg);
		pr_debug("isp ctx %d update size: %d %d\n",
			pctx->ctx_id, cfg.src.w, cfg.src.h);
		src_new = &cfg.src;
	}

	/* update all path scaler trim0  */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if (in_param->valid & update[i]) {
			path_trim = in_param->trim_path[i];
		} else if (src_new) {
			path_trim.start_x = path_trim.start_y = 0;
			path_trim.size_x = src_new->w;
			path_trim.size_y = src_new->h;
		} else {
			continue;
		}
		ret = isp_cfg_path_size(path, &path_trim);
		pr_debug("update isp path%d trim %d %d %d %d\n",
			i, path_trim.start_x, path_trim.start_y,
			path_trim.size_x, path_trim.size_y);
	}
	pctx->updated = 1;
	return ret;
}

static int set_fmcu_slw_queue(
	struct isp_fmcu_ctx_desc *fmcu,
	struct isp_pipe_context *pctx)
{
	int ret = 0, i;
	uint32_t frame_id;
	struct isp_path_desc *path;
	struct camera_frame *pframe = NULL;
	struct camera_frame *out_frame = NULL;

	if (!fmcu)
		return -EINVAL;

	pframe = camera_dequeue(&pctx->in_queue);
	if (pframe == NULL) {
		pr_err("fail to get frame from input queue. cxt:%d\n", pctx->ctx_id);
		return -EINVAL;
	}

	ret = cambuf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n", pctx->ctx_id);
		ret = -EINVAL;
	}

	ret = camera_enqueue(&pctx->proc_queue, pframe);
	if (ret) {
		pr_err("fail to input frame queue, timeout.\n");
		ret = -EINVAL;
	}

	pframe->width = pctx->input_size.w;
	pframe->height = pctx->input_size.h;
	frame_id = pframe->fid;
	isp_path_set_fetch_frm(pctx, pframe);

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if (i == ISP_SPATH_VID)
			out_frame = camera_dequeue(&path->out_buf_queue);
		if (out_frame == NULL)
			out_frame = camera_dequeue(&path->reserved_buf_queue);

		if (out_frame == NULL) {
			pr_debug("fail to get available output buffer.\n");
			return -EINVAL;
		}
		out_frame->fid = frame_id;
		out_frame->sensor_time = pframe->sensor_time;
		out_frame->boot_sensor_time = pframe->boot_sensor_time;

		pr_debug("isp output buf, iova 0x%x, phy: 0x%x\n",
				(uint32_t)out_frame->buf.iova[0],
				(uint32_t)out_frame->buf.addr_k[0]);
		isp_path_set_store_frm(path, out_frame);
		if ((i < AFBC_PATH_NUM) && (path->afbc_store.bypass == 0)) {
			isp_path_set_afbc_store_frm(path, out_frame);
		}
		ret = camera_enqueue(&path->result_queue, out_frame);
		if (ret) {
			if (out_frame->is_reserved)
				camera_enqueue(&path->reserved_buf_queue,
						out_frame);
			else
				camera_enqueue(&path->out_buf_queue,
						out_frame);
			return -EINVAL;
		}
		atomic_inc(&path->store_cnt);
	}

	ret = isp_set_slw_fmcu_cmds((void *)fmcu, pctx);

	pr_debug("fmcu slw queue done!");
	return ret;
}


static int proc_slices(struct isp_pipe_context *pctx)
{
	int ret = 0;
	int hw_ctx_id = -1;
	uint32_t slice_id, cnt = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_cfg_ctx_desc *cfg_desc;

	pr_debug("enter. need_slice %d\n", pctx->multi_slice);
	hw_ctx_id = isp_get_hw_context_id(pctx);
	if (hw_ctx_id < 0) {
		pr_err("fail to get valid hw for ctx %d\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	pctx->is_last_slice = 0;
	for (slice_id = 0; slice_id < SLICE_NUM_MAX; slice_id++) {
		if (pctx->is_last_slice == 1)
			break;

		ret = isp_update_slice(pctx, pctx->ctx_id, slice_id);
		if (ret < 0)
			continue;

		cnt++;
		if (cnt == pctx->valid_slc_num)
			pctx->is_last_slice = 1;
		pr_debug("slice %d, valid %d, last %d\n", slice_id,
			pctx->valid_slc_num, pctx->is_last_slice);

		pctx->started = 1;
		ret = cfg_desc->ops->hw_cfg(
				cfg_desc, pctx->ctx_id, hw_ctx_id, 0);
		ret = cfg_desc->ops->hw_start(
					cfg_desc, hw_ctx_id);
		ret = wait_for_completion_interruptible_timeout(
					&pctx->slice_done,
					ISP_CONTEXT_TIMEOUT);
		if (ret == ERESTARTSYS) {
			pr_err("fail to interrupt, when isp wait\n");
			ret = -EFAULT;
			goto exit;
		} else if (ret == 0) {
			pr_err("fail to wait isp context %d, timeout.\n", pctx->ctx_id);
			ret = -EFAULT;
			goto exit;
		}
		pr_debug("slice %d done\n", slice_id);
	}
exit:
	return ret;
}

static uint32_t isp_get_fid_across_context(struct isp_pipe_dev *dev)
{
	struct isp_pipe_context *ctx;
	struct isp_path_desc *path;
	struct camera_frame *frame;
	uint32_t target_fid;
	int ctx_id, path_id;

	if (!dev)
		return CAMERA_RESERVE_FRAME_NUM;

	target_fid = CAMERA_RESERVE_FRAME_NUM;
	for (ctx_id = 0; ctx_id < ISP_CONTEXT_SW_NUM; ctx_id++) {
		ctx = &dev->ctx[ctx_id];
		if (!ctx || atomic_read(&ctx->user_cnt) < 1)
			continue;

		for (path_id = 0; path_id < ISP_SPATH_NUM; path_id++) {
			path = &ctx->isp_path[path_id];
			if (!path || atomic_read(&path->user_cnt) < 1
			    || !path->uframe_sync)
				continue;

			frame = camera_dequeue_peek(&path->out_buf_queue);
			if (!frame)
				continue;

			target_fid = min(target_fid, frame->user_fid);
			pr_debug("ISP%d path%d user_fid %u\n",
				 ctx_id, path_id, frame->user_fid);
		}
	}

	pr_debug("target_fid %u\n", target_fid);

	return target_fid;
}

static bool isp_check_fid(struct camera_frame *frame, void *data)
{
	uint32_t target_fid;

	if (!frame || !data)
		return false;

	target_fid = *(uint32_t *)data;

	return frame->user_fid == CAMERA_RESERVE_FRAME_NUM
		|| frame->user_fid == target_fid;
}

int isp_get_hw_context_id(struct isp_pipe_context *pctx)
{
	int i;
	int hw_ctx_id = -1;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_hw_context *pctx_hw;

	dev = pctx->dev;

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];

		if ((pctx->ctx_id == pctx_hw->sw_ctx_id)
			&& (pctx == pctx_hw->pctx)) {
			hw_ctx_id = pctx_hw->hw_ctx_id;
			break;
		}
	}
	pr_debug("get hw %d\n", hw_ctx_id);

	return hw_ctx_id;
}

int isp_get_sw_context_id(enum isp_context_hw_id hw_ctx_id, struct isp_pipe_dev *dev)
{
	int sw_id = -1;
	struct isp_pipe_hw_context *pctx_hw;

	if (hw_ctx_id < ISP_CONTEXT_HW_NUM) {
		pctx_hw = &dev->hw_ctx[hw_ctx_id];
		sw_id = pctx_hw->sw_ctx_id;
		if (sw_id >= ISP_CONTEXT_P0 &&
			sw_id < ISP_CONTEXT_SW_NUM &&
			pctx_hw->pctx == &dev->ctx[sw_id]) {
				pr_debug("get sw %d\n", sw_id);
				return sw_id;
			}
	}

	return -1;
}

int isp_context_bind(struct isp_pipe_context *pctx, int fmcu_need)
{
	int i = 0,  m = 0, loop;
	int hw_ctx_id = -1;
	unsigned long flag = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_hw_context *pctx_hw;

	dev = pctx->dev;
	spin_lock_irqsave(&dev->ctx_lock, flag);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if (pctx->ctx_id == pctx_hw->sw_ctx_id
			&& pctx_hw->pctx == pctx) {
			atomic_inc(&pctx_hw->user_cnt);
			pr_debug("sw %d & hw %d already binding, cnt=%d\n",
				pctx->ctx_id, i, atomic_read(&pctx_hw->user_cnt));
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return 0;
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

	if (hw_ctx_id == -1) {
		return -1;
	}

	pctx_hw->pctx = pctx;
	pctx_hw->sw_ctx_id = pctx->ctx_id;
	pr_debug("sw %d, hw %d %d, fmcu_need %d ptr 0x%lx\n",
		pctx->ctx_id, hw_ctx_id, pctx_hw->hw_ctx_id,
		fmcu_need, (unsigned long)pctx_hw->fmcu_handle);

	return 0;
}

int isp_context_unbind(struct isp_pipe_context *pctx)
{
	int i, cnt;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_hw_context *pctx_hw;
	unsigned long flag = 0;

	if (isp_get_hw_context_id(pctx) < 0) {
		pr_err("fail to binding sw ctx %d to any hw ctx\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	spin_lock_irqsave(&dev->ctx_lock, flag);

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw = &dev->hw_ctx[i];
		if ((pctx->ctx_id != pctx_hw->sw_ctx_id) ||
			(pctx != pctx_hw->pctx))
			continue;

		if (atomic_dec_return(&pctx_hw->user_cnt) == 0) {
			pr_debug("sw_id=%d, hw_id=%d unbind success\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id);
			pctx_hw->pctx = NULL;
			pctx_hw->sw_ctx_id = -1;
			goto exit;
		}

		cnt = atomic_read(&pctx_hw->user_cnt);
		if (cnt >= 1) {
			pr_debug("sw id=%d, hw_id=%d, cnt=%d\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id, cnt);
		} else {
			pr_debug("should not be here: sw id=%d, hw id=%d, cnt=%d\n",
				pctx->ctx_id, pctx_hw->hw_ctx_id, cnt);
			spin_unlock_irqrestore(&dev->ctx_lock, flag);
			return -EINVAL;
		}
	}

exit:
	spin_unlock_irqrestore(&dev->ctx_lock, flag);
	return 0;
}

static int isp_slice_needed(struct isp_pipe_context *pctx)
{
	int i;
	struct isp_path_desc *path;

	if (pctx->input_trim.size_x > g_camctrl.isp_linebuf_len)
		return 1;
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		if (path->dst.w > g_camctrl.isp_linebuf_len)
			return 1;
	}
	return 0;
}

static int isp_offline_start_frame(void *ctx)
{
	int ret = 0;
	int valid_out_frame = -1;
	int i = 0, loop = 0, kick_fmcu = 0, slc_by_ap = 0;
	int hw_ctx_id = -1;
	uint32_t use_fmcu = 0, multi_slice = 0;
	uint32_t frame_id, target_fid = CAMERA_RESERVE_FRAME_NUM;
	struct isp_pipe_dev *dev = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_frame *out_frame = NULL;
	struct isp_pipe_context *pctx = NULL;
	struct isp_pipe_hw_context *pctx_hw = NULL;
	struct isp_path_desc *path, *slave_path;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_offline_param *in_param = NULL;
	struct cam_hw_info *hw = NULL;

	pctx = (struct isp_pipe_context *)ctx;
	pr_debug("enter sw id %d, user_cnt=%d, ch_id=%d, cam_id=%d\n",
		pctx->ctx_id, atomic_read(&pctx->user_cnt),
		pctx->ch_id, pctx->attach_cam_id);

	if (atomic_read(&pctx->user_cnt) < 1) {
		pr_err("fail to init isp cxt %d.\n", pctx->ctx_id);
		return -EINVAL;
	}

	dev = pctx->dev;
	hw = dev->isp_hw;
	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;

	if (pctx->multi_slice | isp_slice_needed(pctx))
		use_fmcu = FMCU_IS_NEED;
	if (use_fmcu && (pctx->mode_3dnr != MODE_3DNR_OFF))
		use_fmcu |= FMCU_IS_MUST; // force fmcu used
	if (pctx->enable_slowmotion)
		use_fmcu |= FMCU_IS_MUST; // force fmcu used

	loop = 0;
	do {
		ret = isp_context_bind(pctx, use_fmcu);
		if (!ret) {
			hw_ctx_id = isp_get_hw_context_id(pctx);
			if (hw_ctx_id >= 0 && hw_ctx_id < ISP_CONTEXT_HW_NUM)
				pctx_hw = &dev->hw_ctx[hw_ctx_id];
			else
				pr_err("fail to get hw_ctx_id\n");
			break;
		}
		pr_info_ratelimited("ctx %d wait for hw. loop %d\n", pctx->ctx_id, loop);
		usleep_range(600, 2000);
	} while (loop++ < 5000);

	pframe = camera_dequeue(&pctx->in_queue);

	if (!pctx_hw || pframe == NULL) {
		pr_err("fail to get hw(%p) or input frame (%p) for ctx %d\n",
			pctx_hw, pframe, pctx->ctx_id);
		ret = 0;
		goto input_err;
	}

	if ((pframe->fid & 0x1f) == 0)
		pr_info("cam%d  ctx %d, fid %d, ch_id %d, buf_fd %d\n",
			pctx->attach_cam_id,  pctx->ctx_id,
			pframe->fid, pframe->channel_id, pframe->buf.mfd[0]);

	ret = cambuf_iommu_map(&pframe->buf, CAM_IOMMUDEV_ISP);
	if (ret) {
		pr_err("fail to map buf to ISP iommu. cxt %d\n", pctx->ctx_id);
		ret = -EINVAL;
		goto map_err;
	}

	frame_id = pframe->fid;
	loop = 0;
	do {
		ret = camera_enqueue(&pctx->proc_queue, pframe);
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

	/*
	 * param_mutex to avoid ctx/all paths param
	 * updated when set to register.
	 */
	mutex_lock(&pctx->param_mutex);

	in_param = (struct isp_offline_param *)pframe->param_data;
	if (in_param) {
		isp_update_offline_param(pctx, in_param);
		free_offline_pararm(in_param);
		pframe->param_data = NULL;
	}
	pframe->width = pctx->input_size.w;
	pframe->height = pctx->input_size.h;

	isp_update_hist_roi(pctx);

	/*update NR param for crop/scaling image */
	isp_adapt_blkparam(pctx);

	multi_slice = pctx->multi_slice;
	/* the context/path maybe init/updated after dev start. */
	if (pctx->updated) {
		ret = isp_slice_ctx_init(pctx, &multi_slice);
		hw->hw_ops.core_ops.isp_fetch_set(pctx);
	}

	/* config fetch address */
	isp_path_set_fetch_frm(pctx, pframe);

	/* Reset blending count if frame size change */
	isp_3dnr_process_frame_previous(pctx, pframe);

	if (pctx->uframe_sync)
		target_fid = isp_get_fid_across_context(dev);

	/* config all paths output */
	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;

		if ((i < AFBC_PATH_NUM) && (path->afbc_store.bypass == 0))
			ret = isp_afbc_store(path);

		if (pctx->updated)
			ret = isp_set_path(path);

		/* slave path output buffer binding to master buffer*/
		if (path->bind_type == ISP_PATH_SLAVE)
			continue;

		if ((pctx->mode_3dnr == MODE_3DNR_CAP) &&
		    (pctx->nr3_ctx.blending_cnt % 5 != 4)) {
			valid_out_frame = 1;
			out_frame = camera_dequeue(&path->reserved_buf_queue);
			pr_debug("3dnr frame [0 ~ 3], discard cnt[%d][%d]\n",
				 pctx->nr3_ctx.blending_cnt,
				 pctx->nr3_ctx.blending_cnt % 5);
		} else {
			if (path->uframe_sync
			    && target_fid != CAMERA_RESERVE_FRAME_NUM)
				out_frame =
					camera_dequeue_if(&path->out_buf_queue,
							  isp_check_fid,
							  (void *)&target_fid);
			else
				out_frame =
					camera_dequeue(&path->out_buf_queue);

			if (out_frame)
				valid_out_frame = 1;
			else
				out_frame =
					camera_dequeue(&path->reserved_buf_queue);
		}

		if (out_frame == NULL) {
			pr_debug("fail to get available output buffer.\n");
			ret = 0;
			goto unlock;
		}

		out_frame->fid = frame_id;
		out_frame->sensor_time = pframe->sensor_time;
		out_frame->boot_sensor_time = pframe->boot_sensor_time;

		/* config store buffer */
		pr_debug("isp output buf, iova 0x%x, phy: 0x%x user_fid: %x\n",
			 (uint32_t)out_frame->buf.iova[0],
			 (uint32_t)out_frame->buf.addr_k[0], out_frame->user_fid);
		ret = isp_path_set_store_frm(path, out_frame);
		/* If some error comes then do not start ISP */
		if (ret) {
			cambuf_iommu_unmap(&out_frame->buf);
			cambuf_put_ionbuf(&out_frame->buf);
			put_empty_frame(out_frame);
			ret = -EINVAL;
			goto unlock;
		}
		if ((i < AFBC_PATH_NUM) && (path->afbc_store.bypass == 0))
			isp_path_set_afbc_store_frm(path, out_frame);

		if (path->bind_type == ISP_PATH_MASTER) {
			struct camera_frame temp;
			/* fixed buffer offset here. HAL should use same offset calculation method */
			temp.buf.iova[0] = out_frame->buf.iova[0] + path->store.total_size;
			temp.buf.iova[1] = temp.buf.iova[2] = 0;
			slave_path = &pctx->isp_path[path->slave_path_id];
			isp_path_set_store_frm(slave_path, &temp);
		}
		/*
		 * context proc_queue frame number
		 * should be equal to path result queue.
		 * if ctx->proc_queue enqueue OK,
		 * path result_queue enqueue should be OK.
		 */
		loop = 0;
		do {
			ret = camera_enqueue(&path->result_queue, out_frame);
			if (ret == 0)
				break;
			printk_ratelimited(KERN_INFO "wait for output queue. loop %d\n", loop);
			/* wait for previous frame output queue done */
			mdelay(1);
		} while (loop++ < 500);

		if (ret) {
			trace_isp_irq_cnt(hw_ctx_id);
			pr_err("fail to enqueue, hw %d, path %d, store %d\n",
					hw_ctx_id, path->spath_id,
					atomic_read(&path->store_cnt));
			/* ret frame to original queue */
			if (out_frame->is_reserved)
				camera_enqueue(
					&path->reserved_buf_queue, out_frame);
			else
				camera_enqueue(
					&path->out_buf_queue, out_frame);
			ret = -EINVAL;
			goto unlock;
		}
		atomic_inc(&path->store_cnt);
	}

	if (valid_out_frame == -1) {
		pr_debug(" No available output buffer sw %d, hw %d,discard\n",
			pctx_hw->sw_ctx_id, pctx_hw->hw_ctx_id);
		goto unlock;
	}

	isp_ltm_process_frame_previous(pctx, pframe);
	isp_3dnr_process_frame(pctx, pframe);
	isp_ltm_process_frame(pctx, pframe);

	use_fmcu = 0;
	fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
	if (fmcu) {
		use_fmcu = (multi_slice | pctx->enable_slowmotion);
		if (use_fmcu)
			fmcu->ops->ctx_reset(fmcu);
	}

	if (multi_slice || pctx->enable_slowmotion) {
		struct slice_cfg_input slc_cfg;

		memset(&slc_cfg, 0, sizeof(slc_cfg));
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			if (atomic_read(&path->user_cnt) < 1)
				continue;
			slc_cfg.frame_store[i] = &path->store;
			if ((i < AFBC_PATH_NUM) && (path->afbc_store.bypass == 0))
				slc_cfg.frame_afbc_store[i] = &path->afbc_store;
		}
		slc_cfg.frame_fetch = &pctx->fetch;
		slc_cfg.frame_fbd_raw = &pctx->fbd_raw;
		isp_cfg_slice_fetch_info(&slc_cfg, pctx->slice_ctx);
		isp_cfg_slice_store_info(&slc_cfg, pctx->slice_ctx);
		if (path->afbc_store.bypass == 0)
			isp_cfg_slice_afbc_store_info(&slc_cfg, pctx->slice_ctx);

		/* 3DNR Capture case: Using CP Config */
		slc_cfg.frame_in_size.w = pctx->input_trim.size_x;
		slc_cfg.frame_in_size.h = pctx->input_trim.size_y;
		slc_cfg.nr3_ctx = &pctx->nr3_ctx;
		isp_cfg_slice_3dnr_info(&slc_cfg, pctx->slice_ctx);

		slc_cfg.ltm_ctx = &pctx->ltm_ctx;
		isp_cfg_slice_ltm_info(&slc_cfg, pctx->slice_ctx);

		slc_cfg.nofilter_ctx = &pctx->isp_k_param;
		isp_cfg_slice_noisefilter_info(&slc_cfg, pctx->slice_ctx);

		if (!use_fmcu) {
			pr_debug("use ap support slices for ctx %d hw %d\n",
				pctx->ctx_id, hw_ctx_id);
			slc_by_ap = 1;
		} else {
			pr_debug("use fmcu support slices for ctx %d hw %d\n",
				pctx->ctx_id, hw_ctx_id);
			ret = isp_set_slices_fmcu_cmds((void *)fmcu, pctx);
			if (ret == 0)
				kick_fmcu = 1;
		}
	}

	pctx->updated = 0;
	mutex_unlock(&pctx->param_mutex);

	if (pctx->enable_slowmotion) {
		for (i = 0; i < pctx->slowmotion_count - 1; i++) {
			ret = set_fmcu_slw_queue(fmcu, pctx);
			if (ret)
				pr_err("fail to set fmcu slw queue\n");
		}
	}

	ret = wait_for_completion_interruptible_timeout(
					&pctx->frm_done,
					ISP_CONTEXT_TIMEOUT);
	if (ret == ERESTARTSYS) {
		pr_err("fail to interrupt, when isp wait\n");
		ret = -EFAULT;
		goto dequeue;
	} else if (ret == 0) {
		pr_err("fail to wait isp context %d, timeout.\n", pctx->ctx_id);
		ret = -EFAULT;
		goto dequeue;
	}

	pctx->iommu_status = (uint32_t)(-1);
	pctx->started = 1;
	pctx->multi_slice = multi_slice;
	pctx_hw->fmcu_used = use_fmcu;

	if (slc_by_ap) {
		ret = proc_slices(pctx);
		goto done;
	}

	/* start to prepare/kickoff cfg buffer. */
	if (likely(dev->wmode == ISP_CFG_MODE)) {
		pr_debug("cfg enter.");

		/* blkpm_lock to avoid user config block param across frame */
		mutex_lock(&pctx->blkpm_lock);

		ret = cfg_desc->ops->hw_cfg(cfg_desc,
					pctx->ctx_id, hw_ctx_id, kick_fmcu);

		mutex_unlock(&pctx->blkpm_lock);

		if (kick_fmcu) {
			pr_info("fmcu start.");
			if (pctx->slw_state == CAM_SLOWMOTION_ON) {
				ret = fmcu->ops->cmd_ready(fmcu);
			} else {
				ret = fmcu->ops->hw_start(fmcu);
				if (!ret && pctx->enable_slowmotion)
					pctx->slw_state = CAM_SLOWMOTION_ON;
			}
		} else {
			pr_debug("cfg start. fid %d\n", frame_id);
			ret = cfg_desc->ops->hw_start(
					cfg_desc, hw_ctx_id);
		}
	} else {
		if (kick_fmcu) {
			pr_info("fmcu start.");
			ret = fmcu->ops->hw_start(fmcu);
		} else {
			pr_debug("fetch start.");
			ISP_HREG_WR(ISP_FETCH_START, 1);
		}
	}

done:
	pr_debug("done.\n");
	return 0;

unlock:
	mutex_unlock(&pctx->param_mutex);
dequeue:
	for (i = i - 1; i >= 0; i--) {
		path = &pctx->isp_path[i];
		if (atomic_read(&path->user_cnt) < 1)
			continue;
		pframe = camera_dequeue_tail(&path->result_queue);
		/* ret frame to original queue */
		if (pframe->is_reserved)
			camera_enqueue(
				&path->reserved_buf_queue, pframe);
		else
			camera_enqueue(
				&path->out_buf_queue, pframe);
		atomic_dec(&path->store_cnt);
	}

	pframe = camera_dequeue_tail(&pctx->proc_queue);
inq_overflow:
	cambuf_iommu_unmap(&pframe->buf);
map_err:
input_err:
	if (pframe) {
		free_offline_pararm(pframe->param_data);
		pframe->param_data = NULL;
		/* release sync data as if ISP has consumed */
		if (pframe->sync_data)
			dcam_if_release_sync(pframe->sync_data, pframe);
		/* return buffer to cam channel shared buffer queue. */
		pctx->isp_cb_func(ISP_CB_RET_SRC_BUF, pframe, pctx->cb_priv_data);
	}
	if (pctx_hw)
		isp_context_unbind(pctx);
	return ret;
}


static int isp_offline_thread_loop(void *arg)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;
	struct cam_thread_info *thrd;

	if (!arg) {
		pr_err("fail to get valid input ptr\n");
		return -1;
	}

	thrd = (struct cam_thread_info *)arg;
	pctx = (struct isp_pipe_context *)thrd->ctx_handle;
	dev = pctx->dev;

	while (1) {
		if (wait_for_completion_interruptible(
			&thrd->thread_com) == 0) {
			if (atomic_cmpxchg(
					&thrd->thread_stop, 1, 0) == 1) {
				pr_info("isp context %d thread stop.\n",
						pctx->ctx_id);
				break;
			}
			pr_debug("thread com done.\n");

			if (thrd->proc_func(pctx)) {
				pr_err("fail to start isp pipe to proc. exit thread\n");
				pctx->isp_cb_func(
					ISP_CB_DEV_ERR, dev,
					pctx->cb_priv_data);
				break;
			}
		} else {
			pr_debug("offline thread exit!");
			break;
		}
	}

	return 0;
}


static int isp_stop_offline_thread(void *param)
{
	int cnt = 0;
	int ret = 0;
	struct cam_thread_info *thrd;
	struct isp_pipe_context *pctx;

	thrd = (struct cam_thread_info *)param;
	pctx = (struct isp_pipe_context *)thrd->ctx_handle;

	if (thrd->thread_task) {
		atomic_set(&thrd->thread_stop, 1);
		complete(&thrd->thread_com);
		while (cnt < 2500) {
			cnt++;
			if (atomic_read(&thrd->thread_stop) == 0)
				break;
			udelay(1000);
		}
		thrd->thread_task = NULL;
		pr_info("offline thread stopped. wait %d ms\n", cnt);
	}

	/* wait for last frame done */
	ret = wait_for_completion_interruptible_timeout(
					&pctx->frm_done,
					ISP_CONTEXT_TIMEOUT);
	if (ret == -ERESTARTSYS)
		pr_err("fail to interrupt, when isp wait\n");
	else if (ret == 0)
		pr_err("fail to wait ctx %d, timeout.\n", pctx->ctx_id);
	else
		pr_info("wait time %d\n", ret);
	return 0;
}


static int isp_create_offline_thread(void *param)
{
	struct isp_pipe_context *pctx;
	struct cam_thread_info *thrd;
	char thread_name[32] = { 0 };

	pctx = (struct isp_pipe_context *)param;
	thrd = &pctx->thread;
	thrd->ctx_handle = pctx;
	thrd->proc_func = isp_offline_start_frame;
	atomic_set(&thrd->thread_stop, 0);
	init_completion(&thrd->thread_com);

	sprintf(thread_name, "isp_ctx%d_offline", pctx->ctx_id);
	thrd->thread_task = kthread_run(
						isp_offline_thread_loop,
					      thrd, thread_name);
	if (IS_ERR_OR_NULL(thrd->thread_task)) {
		pr_err("fail to start offline thread for isp ctx%d err %ld\n",
				pctx->ctx_id, PTR_ERR(thrd->thread_task));
		return -EFAULT;
	}

	pr_info("isp ctx %d offline thread created.\n", pctx->ctx_id);
	return 0;
}


static int isp_context_init(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, bind_fmcu;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_pipe_context *pctx;
	struct isp_pipe_hw_context *pctx_hw;
	struct cam_hw_info *hw = NULL;
	enum isp_context_id cid[ISP_CONTEXT_SW_NUM] = {
		ISP_CONTEXT_P0,
		ISP_CONTEXT_C0,
		ISP_CONTEXT_P1,
		ISP_CONTEXT_C1,
		ISP_CONTEXT_P2,
		ISP_CONTEXT_C2
	};

	pr_info("isp contexts init start!\n");
	memset(&dev->ctx[0], 0, sizeof(dev->ctx));

	for (i = 0; i < ISP_CONTEXT_SW_NUM; i++) {
		pctx = &dev->ctx[i];
		pctx->ctx_id = cid[i];
		pctx->dev = dev;
		pctx->attach_cam_id = CAM_ID_MAX;
		pctx->hw = dev->isp_hw;
		atomic_set(&pctx->user_cnt, 0);
		pr_debug("isp context %d init done!\n", cid[i]);
	}

	/* CFG module init */
	if (dev->wmode == ISP_AP_MODE) {

		pr_info("isp ap mode.\n");
		for (i = 0; i < ISP_CONTEXT_SW_NUM; i++)
			isp_cfg_poll_addr[i] = &s_isp_regbase[0];

	} else {
		cfg_desc = get_isp_cfg_ctx_desc();
		if (!cfg_desc || !cfg_desc->ops) {
			pr_err("fail to get isp cfg ctx %p.\n", cfg_desc);
			ret = -EINVAL;
			goto cfg_null;
		}
		cfg_desc->hw_ops = &dev->isp_hw->hw_ops;
		pr_debug("cfg_init start.\n");

		ret = cfg_desc->ops->ctx_init(cfg_desc);
		if (ret) {
			pr_err("fail to cfg ctx init.\n");
			goto ctx_fail;
		}
		pr_debug("cfg_init done.\n");

		ret = cfg_desc->ops->hw_init(cfg_desc);
		if (ret)
			goto hw_fail;
	}
	dev->cfg_handle = cfg_desc;

	dev->ltm_handle = isp_get_ltm_share_ctx_desc();

	pr_info("isp hw contexts init start!\n");
	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw  = &dev->hw_ctx[i];
		pctx_hw->hw_ctx_id = i;
		pctx_hw->sw_ctx_id = 0xffff;
		atomic_set(&pctx_hw->user_cnt, 0);

		hw = dev->isp_hw;
		hw->hw_ops.isp_irq_ops.irq_enable(hw->ip_isp, &i);
		hw->hw_ops.isp_irq_ops.irq_clear(hw->ip_isp, &i);

		bind_fmcu = 0;
		if (unlikely(dev->wmode == ISP_AP_MODE)) {
			/* for AP mode, multi-context is not supported */
			if (i != ISP_CONTEXT_HW_P0) {
				atomic_set(&pctx_hw->user_cnt, 1);
				continue;
			}
			bind_fmcu = 1;
		} else if (*(hw->ip_isp->ctx_fmcu_support + i)) {
			bind_fmcu = 1;
		}

		if (bind_fmcu) {
			fmcu = get_isp_fmcu_ctx_desc(hw);
			pr_debug("get fmcu %p\n", fmcu);

			if (fmcu && fmcu->ops) {
				ret = fmcu->ops->ctx_init(fmcu);
				if (ret) {
					pr_err("fail to init fmcu ctx\n");
					put_isp_fmcu_ctx_desc(fmcu);
				} else {
					pctx_hw->fmcu_handle = fmcu;
				}
			} else {
				pr_info("no more fmcu or ops\n");
			}
		}
		pr_info("isp hw context %d init done. fmcu %p\n",
				i, pctx_hw->fmcu_handle);

		fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
		if (fmcu) {
			uint32_t val[2];
			unsigned long reg_offset = 0;
			uint32_t reg_bits[4] = { 0x00, 0x02, 0x01, 0x03};

			reg_offset = (fmcu->fid == 0) ?
						ISP_COMMON_FMCU0_PATH_SEL :
						ISP_COMMON_FMCU1_PATH_SEL;
			if (reg_offset == 0) {
				pr_info("no fmcu sel register\n");
				continue;
			}

			ISP_HREG_MWR(reg_offset, BIT_1 | BIT_0, reg_bits[i]);
			pr_debug("FMCU%d reg_bits %d for hw_ctx %d\n", fmcu->fid, reg_bits[i], i);

			val[0] = ISP_HREG_RD(ISP_COMMON_FMCU0_PATH_SEL);
			val[1] = ISP_HREG_RD(ISP_COMMON_FMCU1_PATH_SEL);

			if ((val[0] & 3) == (val[1] & 3)) {
				pr_warn("isp fmcus select same context %d\n", val[0] & 3);
				if (fmcu->fid == 0) {
					/* force to set different value */
					reg_offset = ISP_COMMON_FMCU1_PATH_SEL;
					ISP_HREG_MWR(reg_offset, BIT_1 | BIT_0, reg_bits[(i + 1) & 3]);
					pr_debug("force FMCU1 regbits %d\n", reg_bits[(i + 1) & 3]);
				}
			}
		}
	}

	pr_debug("done!\n");
	return 0;

hw_fail:
	cfg_desc->ops->ctx_deinit(cfg_desc);
ctx_fail:
	put_isp_cfg_ctx_desc(cfg_desc);
cfg_null:
	return ret;
}

static int isp_context_deinit(struct isp_pipe_dev *dev)
{
	int ret = 0;
	int i, j;
	uint32_t path_id;
	struct isp_fmcu_ctx_desc *fmcu = NULL;
	struct isp_cfg_ctx_desc *cfg_desc = NULL;
	struct isp_pipe_context *pctx;
	struct isp_pipe_hw_context *pctx_hw;
	struct isp_path_desc *path;

	pr_debug("enter.\n");

	for (i = 0; i < ISP_CONTEXT_SW_NUM; i++) {
		pctx  = &dev->ctx[i];

		/* free all used path here if user did not call put_path  */
		for (j = 0; j < ISP_SPATH_NUM; j++) {
			path = &pctx->isp_path[j];
			path_id = path->spath_id;
			if (atomic_read(&path->user_cnt) > 0)
				sprd_isp_put_path(dev, pctx->ctx_id, path_id);
		}
		sprd_isp_put_context(dev, pctx->ctx_id);

		atomic_set(&pctx->user_cnt, 0);
		mutex_destroy(&pctx->param_mutex);
		mutex_destroy(&pctx->blkpm_lock);
	}

	for (i = 0; i < ISP_CONTEXT_HW_NUM; i++) {
		pctx_hw  = &dev->hw_ctx[i];
		fmcu = (struct isp_fmcu_ctx_desc *)pctx_hw->fmcu_handle;
		if (fmcu) {
			fmcu->ops->ctx_deinit(fmcu);
			put_isp_fmcu_ctx_desc(fmcu);
		}
		pctx_hw->fmcu_handle = NULL;
		pctx_hw->fmcu_used = 0;
		atomic_set(&pctx_hw->user_cnt, 0);
		trace_isp_irq_cnt(i);
	}
	pr_info("isp contexts deinit done!\n");

	cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
	if (cfg_desc) {
		cfg_desc->ops->ctx_deinit(cfg_desc);
		put_isp_cfg_ctx_desc(cfg_desc);
	}
	dev->cfg_handle = NULL;

	isp_put_ltm_share_ctx_desc(dev->ltm_handle);
	dev->ltm_handle = NULL;

	pr_debug("done.\n");
	return ret;
}


static int isp_slice_ctx_init(struct isp_pipe_context *pctx, uint32_t * multi_slice)
{
	int ret = 0;
	int j;
	uint32_t val;
	struct isp_path_desc *path;
	struct slice_cfg_input slc_cfg_in;

	*multi_slice = 0;
	if  ((isp_slice_needed(pctx) == 0) &&
		(pctx->enable_slowmotion == 0))
		return 0;

	if (pctx->slice_ctx == NULL) {
		pctx->slice_ctx = get_isp_slice_ctx();
		if (IS_ERR_OR_NULL(pctx->slice_ctx)) {
			pr_err("fail to get memory for slice_ctx.\n");
			pctx->slice_ctx = NULL;
			ret = -ENOMEM;
			goto exit;
		}
	}

	memset(&slc_cfg_in, 0, sizeof(struct slice_cfg_input));
	slc_cfg_in.frame_in_size.w = pctx->input_trim.size_x;
	slc_cfg_in.frame_in_size.h = pctx->input_trim.size_y;
	slc_cfg_in.frame_fetch = &pctx->fetch;
	slc_cfg_in.frame_fbd_raw = &pctx->fbd_raw;
	for (j = 0; j < ISP_SPATH_NUM; j++) {
		path = &pctx->isp_path[j];
		if (atomic_read(&path->user_cnt) <= 0)
			continue;
		slc_cfg_in.frame_out_size[j] = &path->dst;
		slc_cfg_in.frame_store[j] = &path->store;
		slc_cfg_in.frame_scaler[j] = &path->scaler;
		slc_cfg_in.frame_deci[j] = &path->deci;
		slc_cfg_in.frame_trim0[j] = &path->in_trim;
		slc_cfg_in.frame_trim1[j] = &path->out_trim;
		if ((j < AFBC_PATH_NUM) && (path->afbc_store.bypass == 0))
			slc_cfg_in.frame_afbc_store[j] = &path->afbc_store;
	}

	val = ISP_REG_RD(pctx->ctx_id, ISP_NLM_RADIAL_1D_DIST);
	slc_cfg_in.nlm_center_x = val & 0x3FFF;
	slc_cfg_in.nlm_center_y = (val >> 16) & 0x3FFF;

	val = ISP_REG_RD(pctx->ctx_id, ISP_YNR_CFG31);
	slc_cfg_in.ynr_center_x = val & 0xFFFF;
	slc_cfg_in.ynr_center_y = (val >> 16) & 0xfFFF;
	pr_debug("ctx %d,  nlm center %d %d, ynr center %d, %d\n",
		pctx->ctx_id, slc_cfg_in.nlm_center_x, slc_cfg_in.nlm_center_y,
		slc_cfg_in.ynr_center_x, slc_cfg_in.ynr_center_y);

	isp_cfg_slices(&slc_cfg_in, pctx->slice_ctx, &pctx->valid_slc_num);

	pr_debug("ctx %d valid_slc_num %d\n", pctx->ctx_id, pctx->valid_slc_num);
	if (pctx->valid_slc_num > 1)
		*multi_slice = 1;
exit:
	return ret;
}

/* offline process frame */
static int sprd_isp_proc_frame(void *isp_handle,
			void *param, int ctx_id)
{
	int ret = 0;
	struct camera_frame *pframe;
	static int slw_frm_cnt;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];
	pframe = (struct camera_frame *)param;
	pframe->priv_data = pctx;

	pr_debug("cam%d  ctx %d, fid %d, ch_id %d, buf  %d, 3dnr %d , w %d, h %d\n",
		pctx->attach_cam_id, ctx_id, pframe->fid,
		pframe->channel_id, pframe->buf.mfd[0], pctx->mode_3dnr,
		pframe->width, pframe->height);

	ret = camera_enqueue(&pctx->in_queue, pframe);
	if (ret == 0) {
		if (pctx->enable_slowmotion && ++slw_frm_cnt < pctx->slowmotion_count)
			return ret;
		complete(&pctx->thread.thread_com);
		slw_frm_cnt = 0;
	}

	return ret;
}

/*
 * Get a free context and initialize it.
 * Input param is possible max_size of image.
 * Return valid context id, or -1 for failure.
 */
static int sprd_isp_get_context(void *isp_handle, void *param)
{
	int ret = 0;
	int i;
	int sel_ctx_id = -1;
	enum  isp_context_id ctx_id;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev = NULL;
	struct isp_path_desc *path = NULL;
	struct cam_hw_info *hw = NULL;
	struct isp_cfg_ctx_desc *cfg_desc;
	struct isp_init_param *init_param;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	pr_debug("start.\n");

	dev = (struct isp_pipe_dev *)isp_handle;
	hw = dev->isp_hw;
	init_param = (struct isp_init_param *)param;

	mutex_lock(&dev->path_mutex);

	/* ISP cfg mode, get free context */
	for (i = 0; i < ISP_CONTEXT_SW_NUM; i++) {
		ctx_id = i;
		pctx = &dev->ctx[ctx_id];
		if (atomic_inc_return(&pctx->user_cnt) == 1) {
			sel_ctx_id = ctx_id;
			break;
		}
		atomic_dec(&pctx->user_cnt);
	}
	if (sel_ctx_id == -1)
		goto exit;

	if (dev->wmode == ISP_CFG_MODE) {
		cfg_desc = (struct isp_cfg_ctx_desc *)dev->cfg_handle;
		cfg_desc->ops->ctx_reset(cfg_desc, ctx_id);
	}

	for (i = 0; i < ISP_SPATH_NUM; i++) {
		path = &pctx->isp_path[i];
		path->spath_id = i;
		path->attach_ctx = pctx;
		path->q_init = 0;
		path->hw = hw;
		atomic_set(&path->user_cnt, 0);
	}

	mutex_init(&pctx->param_mutex);
	mutex_init(&pctx->blkpm_lock);
	init_completion(&pctx->frm_done);
	init_completion(&pctx->slice_done);
	/* complete for first frame config */
	complete(&pctx->frm_done);
	pctx->isp_k_param.nlm_info.bypass = 1;
	pctx->isp_k_param.ynr_param.ynr_info.bypass = 1;
	/* bypass fbd_raw by default */
	pctx->fbd_raw.fetch_fbd_bypass = 1;
	pctx->multi_slice = 0;
	pctx->started = 0;
	pctx->uframe_sync = 0;
	pctx->attach_cam_id = init_param->cam_id;

	pctx->enable_slowmotion = 0;
	if (init_param->is_high_fps)
		pctx->enable_slowmotion = hw->ip_isp->slm_cfg_support;;
	pr_info("cam%d enable ISP slowmotion %d\n",
		pctx->attach_cam_id, pctx->enable_slowmotion);

	pctx->isp_k_param.nlm_buf = vzalloc(sizeof(uint32_t) * ISP_VST_IVST_NUM2);
	if (pctx->isp_k_param.nlm_buf == NULL) {
		pr_err("fail to alloc nlm buf\n");
		return -ENOMEM;
	}
	ret = isp_create_offline_thread(pctx);
	if (unlikely(ret != 0)) {
		pr_err("fail to create offline thread for isp cxt:%d\n",
				pctx->ctx_id);
		goto thrd_err;
	}

	if (init_param->is_high_fps == 0) {
		camera_queue_init(&pctx->in_queue,
			ISP_IN_Q_LEN, 0, isp_ret_src_frame);
		camera_queue_init(&pctx->proc_queue,
			ISP_PROC_Q_LEN, 0, isp_ret_src_frame);
	} else {
		camera_queue_init(&pctx->in_queue,
			ISP_SLW_IN_Q_LEN, 0, isp_ret_src_frame);
		camera_queue_init(&pctx->proc_queue,
			ISP_SLW_PROC_Q_LEN, 0, isp_ret_src_frame);
	}
	camera_queue_init(&pctx->ltm_avail_queue, ISP_LTM_BUF_NUM,
						0, isp_unmap_frame);
	camera_queue_init(&pctx->ltm_wr_queue, ISP_LTM_BUF_NUM,
						0, isp_unmap_frame);
	camera_queue_init(&pctx->hist2_result_queue, 16,
						0, isp_destroy_statis_buf);

	hw->hw_ops.core_ops.default_para_set(hw, &pctx->ctx_id, ISP_CFG_PARA);
	reset_isp_irq_sw_cnt(pctx->ctx_id);

	goto exit;

thrd_err:
	atomic_dec(&pctx->user_cnt); /* free context */
	sel_ctx_id = -1;
exit:
	mutex_unlock(&dev->path_mutex);
	pr_info("done, ret context: %d\n", sel_ctx_id);
	return sel_ctx_id;
}

/*
 * Free a context and deinitialize it.
 * All paths of this context should be put before this.
 *
 * TODO:
 * we do not stop or reset ISP here because four contexts share it.
 * How to make sure current context process in ISP is done
 * before we clear buffer Q?
 * If ISP hw doesn't done buffer reading/writting,
 * we free buffer here may cause memory over-writting and perhaps system crash.
 * Delay buffer Q clear is a just solution to reduce this risk
 */
static int sprd_isp_put_context(void *isp_handle, int ctx_id)
{
	int ret = 0;
	int i;
	uint32_t loop = 0;
	struct isp_pipe_dev *dev;
	struct isp_pipe_context *pctx;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];

	if (pctx->isp_k_param.nlm_buf) {
		vfree(pctx->isp_k_param.nlm_buf);
		pctx->isp_k_param.nlm_buf = NULL;
	}

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) == 1)
		isp_stop_offline_thread(&pctx->thread);

	if (atomic_dec_return(&pctx->user_cnt) == 0) {
		pctx->started = 0;
		pr_info("free context %d without users.\n", pctx->ctx_id);

		/* make sure irq handler exit to avoid crash */
		while (pctx->in_irq_handler && (loop < 1000)) {
			pr_info("cxt % in irq. wait %d", pctx->ctx_id, loop);
			loop++;
			udelay(500);
		};

		if (pctx->slice_ctx)
			put_isp_slice_ctx(&pctx->slice_ctx);

		camera_queue_clear(&pctx->in_queue);
		camera_queue_clear(&pctx->proc_queue);
		camera_queue_clear(&pctx->hist2_result_queue);

#ifdef USING_LTM_Q
		camera_queue_clear(&pctx->ltm_avail_queue);
		camera_queue_clear(&pctx->ltm_wr_queue);
#else
		dev->ltm_handle->ops->set_status(0, ctx_id, pctx->mode_ltm);
		if (pctx->mode_ltm == MODE_LTM_PRE) {
			dev->ltm_handle->ops->complete_completion();
			for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
				if (pctx->ltm_buf[i]) {
					isp_unmap_frame(pctx->ltm_buf[i]);
					pctx->ltm_buf[i] = NULL;
				}
			}
		}
		if (pctx->mode_ltm == MODE_LTM_CAP) {
			for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
				if (pctx->ltm_buf[i])
					pctx->ltm_buf[i] = NULL;
			}
		}
#endif /* USING_LTM_Q */

		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			if (pctx->nr3_buf[i]) {
				isp_unmap_frame(pctx->nr3_buf[i]);
				pctx->nr3_buf[i] = NULL;
			}
		}

		/* clear path queue. */
		for (i = 0; i < ISP_SPATH_NUM; i++) {
			path = &pctx->isp_path[i];
			if (path->q_init == 0)
				continue;

			/* reserved buffer queue should be cleared at last. */
			camera_queue_clear(&path->result_queue);
			camera_queue_clear(&path->out_buf_queue);
			camera_queue_clear(&path->reserved_buf_queue);
		}

		pctx->isp_cb_func = NULL;
		pctx->cb_priv_data = NULL;
		trace_isp_irq_sw_cnt(pctx->ctx_id);
	} else {
		pr_err("fail to use free ctx %d.\n", ctx_id);
		atomic_set(&pctx->user_cnt, 0);
	}
	memset(pctx, 0, sizeof(struct isp_pipe_context));
	pctx->ctx_id = ctx_id;
	pctx->dev = dev;
	pctx->attach_cam_id = CAM_ID_MAX;
	pctx->hw = dev->isp_hw;

	mutex_unlock(&dev->path_mutex);
	pr_info("done, put ctx_id: %d\n", ctx_id);
	return ret;
}

/*
 * Enable path of sepicific path_id for specific context.
 * The context must be enable before get_path.
 */
static int sprd_isp_get_path(void *isp_handle, int ctx_id, int path_id)
{
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_debug("start.\n");
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM ||
		path_id < 0 || path_id >= ISP_SPATH_NUM) {
		pr_err("fail to get legal ctx_id %d, path_id %d\n", ctx_id, path_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) < 1) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to get path from free context.\n");
		return -EFAULT;
	}

	path = &pctx->isp_path[path_id];
	if (atomic_inc_return(&path->user_cnt) > 1) {
		mutex_unlock(&dev->path_mutex);
		atomic_dec(&path->user_cnt);
		pr_err("fail to get used path %d of cxt %d.\n",
			path_id, ctx_id);
		return -EFAULT;
	}

	mutex_unlock(&dev->path_mutex);

	path->frm_cnt = 0;
	atomic_set(&path->store_cnt, 0);

	if (path->q_init == 0) {
		if (pctx->enable_slowmotion)
			camera_queue_init(&path->result_queue,
				ISP_SLW_RESULT_Q_LEN, 0, isp_ret_out_frame);
		else
			camera_queue_init(&path->result_queue,
				ISP_RESULT_Q_LEN, 0, isp_ret_out_frame);
		camera_queue_init(&path->out_buf_queue, ISP_OUT_BUF_Q_LEN,
			0, isp_ret_out_frame);
		camera_queue_init(&path->reserved_buf_queue,
			ISP_RESERVE_BUF_Q_LEN, 0, isp_destroy_reserved_buf);
		path->q_init = 1;
	}

	pr_info("get path %d done.", path_id);
	return 0;
}

/*
 * Disable path of sepicific path_id for specific context.
 * The path and context should be enable before this.
 */
static int sprd_isp_put_path(void *isp_handle, int ctx_id, int path_id)
{
	int ret = 0;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_debug("start.\n");
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM ||
		path_id < 0 || path_id >= ISP_SPATH_NUM) {
		pr_err("fail to get legal ctx_id %d, path_id %d\n",
			ctx_id, path_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];

	mutex_lock(&dev->path_mutex);

	if (atomic_read(&pctx->user_cnt) < 1) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to free path for free context.\n");
		return -EFAULT;
	}

	path = &pctx->isp_path[path_id];

	if (atomic_read(&path->user_cnt) == 0) {
		mutex_unlock(&dev->path_mutex);
		pr_err("fail to use free isp cxt %d, path %d.\n",
					ctx_id, path_id);
		return -EFAULT;
	}

	if (atomic_dec_return(&path->user_cnt) != 0) {
		pr_warn("isp cxt %d, path %d has multi users.\n",
					ctx_id, path_id);
		atomic_set(&path->user_cnt, 0);
	}

	mutex_unlock(&dev->path_mutex);
	pr_info("done, put path_id: %d for ctx %d\n", path_id, ctx_id);
	return ret;
}

static int sprd_isp_cfg_path(void *isp_handle,
			enum isp_path_cfg_cmd cfg_cmd,
			int ctx_id, int path_id,
			void *param)
{
	int ret = 0;
	int i;
	struct isp_pipe_context *pctx;
	struct isp_pipe_dev *dev;
	struct isp_path_desc *path = NULL;
	struct isp_path_desc *slave_path;
	struct camera_frame *pframe = NULL;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}

	if (ctx_id >= ISP_CONTEXT_SW_NUM  || ctx_id < ISP_CONTEXT_P0) {
		pr_err("fail to get legal id ctx %d\n", ctx_id);
		return -EFAULT;
	}
	if (path_id >= ISP_SPATH_NUM) {
			pr_err("fail to use legal id path %d\n", path_id);
			return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];
	path = &pctx->isp_path[path_id];

	if ((cfg_cmd != ISP_PATH_CFG_CTX_BASE) &&
		(cfg_cmd != ISP_PATH_CFG_CTX_SIZE) &&
		(cfg_cmd != ISP_PATH_CFG_CTX_COMPRESSION)) {
		if (atomic_read(&path->user_cnt) == 0) {
			pr_err("fail to use free isp cxt %d, path %d.\n",
					ctx_id, path_id);
			return -EFAULT;
		}
	}

	switch (cfg_cmd) {
	case ISP_PATH_CFG_OUTPUT_RESERVED_BUF:
	case ISP_PATH_CFG_OUTPUT_BUF:
		pframe = (struct camera_frame *)param;
		ret = cambuf_iommu_map(
				&pframe->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map isp iommu buf.\n");
			ret = -EINVAL;
			goto exit;
		}
		pr_debug("cfg buf path %d, %p\n",
			path->spath_id, pframe);

		/* is_reserved:
		 *  1:  basic mapping reserved buffer;
		 *  2:  copy of reserved buffer.
		 */
		if (unlikely(cfg_cmd == ISP_PATH_CFG_OUTPUT_RESERVED_BUF)) {
			struct camera_frame *newfrm;

			pframe->is_reserved = 1;
			pframe->priv_data = path;
			pr_info("reserved buf\n");
			ret = camera_enqueue(
					&path->reserved_buf_queue, pframe);
			i = 1;
			while (i < ISP_RESERVE_BUF_Q_LEN) {
				newfrm = get_empty_frame();
				if (newfrm) {
					newfrm->is_reserved = 2;
					newfrm->priv_data = path;
					newfrm->user_fid = pframe->user_fid;
					memcpy(&newfrm->buf,
						&pframe->buf,
						sizeof(pframe->buf));
					camera_enqueue(
						&path->reserved_buf_queue,
						newfrm);
					i++;
				}
			}
		} else {
			pr_debug("output buf\n");
			pframe->is_reserved = 0;
			pframe->priv_data = path;
			ret = camera_enqueue(
					&path->out_buf_queue, pframe);
			if (ret) {
				pr_err("fail to enqueue output buffer, path %d.\n",
						path_id);
				cambuf_iommu_unmap(&pframe->buf);
				goto exit;
			}
		}
		break;

	case ISP_PATH_CFG_3DNR_BUF:
		pframe = (struct camera_frame *)param;
		ret = cambuf_iommu_map(
				&pframe->buf, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map isp iommu buf.\n");
			ret = -EINVAL;
			goto exit;
		}
		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			if (pctx->nr3_buf[i] == NULL) {
				pctx->nr3_buf[i] = pframe;
				pctx->nr3_ctx.buf_info[i] =
					&pctx->nr3_buf[i]->buf;
				pr_debug("3DNR CFGB[%d][0x%p][0x%p] = 0x%lx, 0x%lx\n",
					 i, pframe, pctx->nr3_buf[i],
					 pctx->nr3_ctx.buf_info[i]->iova[0],
					 pctx->nr3_buf[i]->buf.iova[0]);
				break;
			} else {
				pr_debug("3DNR CFGB[%d][0x%p][0x%p] failed\n",
				       i, pframe, pctx->nr3_buf[i]);
			}
		}
		if (i == 2) {
			pr_err("fail to set isp ctx %d nr3 buffers.\n", ctx_id);
			cambuf_iommu_unmap(&pframe->buf);
			goto exit;
		}
		break;

	case ISP_PATH_CFG_LTM_BUF:
		pframe = (struct camera_frame *)param;
		if (pctx->mode_ltm == MODE_LTM_PRE) {
			ret = cambuf_iommu_map(
				 &pframe->buf, CAM_IOMMUDEV_ISP);
			if (ret) {
				pr_err("fail to isp map iommu buf.\n");
				ret = -EINVAL;
				goto exit;
			}
		}
		for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
			if (pctx->ltm_buf[i] == NULL) {
				pctx->ltm_buf[i] = pframe;
				pctx->ltm_ctx.pbuf[i] = &pframe->buf;
				pr_debug("LTM CFGB[%d][0x%p][0x%p] = 0x%lx, 0x%lx\n",
					 i, pframe, pctx->ltm_buf[i],
					 pctx->ltm_ctx.pbuf[i]->iova[0],
					 pctx->ltm_buf[i]->buf.iova[0]);
				break;
			}
		}
		pr_debug("isp ctx [%d], ltm buf idx [%d], buf addr [0x%p]\n",
			pctx->ctx_id, i, pframe);
#ifdef USING_LTM_Q
		ret = camera_enqueue(&pctx->ltm_avail_queue, pframe);
		if (ret) {
			cambuf_iommu_unmap(&pframe->buf);
			pr_err("fail to cfg buf, isp ctx %d ltm mode %d.\n",
					pctx->ctx_id, pctx->mode_ltm);
		}
#endif /* USING_LTM_Q */
		break;

	case ISP_PATH_CFG_CTX_BASE:
		ret = isp_cfg_ctx_base(pctx, param);
		pctx->updated = 1;
		break;

	case ISP_PATH_CFG_CTX_SIZE:
		mutex_lock(&pctx->param_mutex);
		ret = isp_cfg_ctx_size(pctx, param);
		pctx->updated = 1;
		mutex_unlock(&pctx->param_mutex);
		break;

	case ISP_PATH_CFG_CTX_COMPRESSION:
		ret = isp_cfg_ctx_compression(pctx, param);
		break;

	case ISP_PATH_CFG_CTX_UFRAME_SYNC:
		ret = isp_cfg_ctx_uframe_sync(pctx, param);
		break;

	case ISP_PATH_CFG_PATH_BASE:
		ret = isp_cfg_path_base(path, param);
		pctx->updated = 1;
		break;

	case ISP_PATH_CFG_PATH_SIZE:
		mutex_lock(&pctx->param_mutex);
		ret = isp_cfg_path_size(path, param);
		if (path->bind_type == ISP_PATH_MASTER) {
			slave_path = &pctx->isp_path[path->slave_path_id];
			ret = isp_cfg_path_size(slave_path, param);
		}
		pctx->updated = 1;
		mutex_unlock(&pctx->param_mutex);
		break;

	case ISP_PATH_CFG_PATH_COMPRESSION:
		ret = isp_cfg_path_compression(path, param);
		break;

	case ISP_PATH_CFG_PATH_UFRAME_SYNC:
		ret = isp_cfg_path_uframe_sync(path, param);
		break;

	case ISP_PATH_CFG_3DNR_MODE:
		mutex_lock(&pctx->param_mutex);
		pctx->mode_3dnr = *(uint32_t *)param;
		mutex_unlock(&pctx->param_mutex);
		break;
	default:
		pr_warn("unsupported cmd: %d\n", cfg_cmd);
		break;
	}
exit:
	pr_debug("cfg path %d done. ret %d\n", path->spath_id, ret);
	return ret;
}

static int isp_cfg_statis_buffer(
		struct isp_statis_io_desc *io_desc)
{

	int ret = 0;
	struct camera_frame *pframe;
	struct camera_buf *ion_buf_isp = NULL;

	pr_debug("enter\n");

	if (io_desc->input->type == STATIS_INIT) {

		ion_buf_isp = kzalloc(sizeof(*ion_buf_isp), GFP_KERNEL);

		if (IS_ERR_OR_NULL(ion_buf_isp)) {
			pr_err("fail to alloc memory for isp statis buf.\n");
			ret = -ENOMEM;
			goto exit;
		}

		ion_buf_isp->mfd[0] = io_desc->input->u.init_data.mfd_isp;
		ion_buf_isp->size[0] = io_desc->input->u.init_data.isp_buf_size;
		ion_buf_isp->addr_vir[0] = (unsigned long)io_desc->input->isp_uaddr;
		ion_buf_isp->type = CAM_BUF_USER;

		ret = cambuf_get_ionbuf(ion_buf_isp);
		if (ret) {
			kfree(ion_buf_isp);
			ret = -EINVAL;
			goto exit;
		}

		ret = cambuf_iommu_map(ion_buf_isp, CAM_IOMMUDEV_ISP);
		if (ret) {
			pr_err("fail to map dcam statis buffer to iommu\n");
			cambuf_put_ionbuf(ion_buf_isp);
			kfree(ion_buf_isp);
			ret = -EINVAL;
			goto exit;
		}
		*io_desc->buf = ion_buf_isp;

		ret = cambuf_kmap(ion_buf_isp);
		if (ret) {
			pr_err("fail to kmap dcam statis buffer\n");
			cambuf_iommu_unmap(ion_buf_isp);
			cambuf_put_ionbuf(ion_buf_isp);
			kfree(ion_buf_isp);
			*io_desc->buf = NULL;
			ret = -EINVAL;
			goto exit;
		}

		pr_debug("isp_ iova[%08x] uaddr[%lx] kaddr[%lx] mfd[%d] size[%d] dmabuf[%p] ionbuf[%p]\n",
				(uint32_t)ion_buf_isp->iova[0],
				ion_buf_isp->addr_vir[0],
				ion_buf_isp->addr_k[0],
				ion_buf_isp->mfd[0],
				ion_buf_isp->size[0],
				ion_buf_isp->dmabuf_p[0],
				ion_buf_isp->ionbuf[0]);
	} else {

		pframe = get_empty_frame();
		pframe->irq_property = io_desc->input->type;
		pframe->buf.addr_vir[0] = (unsigned long)io_desc->input->uaddr;
		pframe->buf.addr_k[0] = (unsigned long)io_desc->input->kaddr;
		pframe->buf.iova[0] = io_desc->input->u.block_data.hw_addr;
		ret = camera_enqueue(io_desc->q, pframe);
		if (ret)
			pr_err("fail to enq\n");

		pr_debug("statis type %d, iova 0x%08x,  uaddr 0x%lx kaddr[0x%lx]\n",
			io_desc->input->type, (uint32_t)pframe->buf.iova[0],
			pframe->buf.addr_vir[0], pframe->buf.addr_k[0]);
	}
exit:
	return ret;

}


static int isp_init_statis_bufferq(
		struct isp_statis_io_desc *io_desc)
{
	int ret = 0;
	int j;
	size_t used_size = 0, total_size;
	size_t buf_size;
	unsigned long kaddr;
	unsigned long paddr;
	unsigned long uaddr;
	enum isp_statis_buf_type stats_type;
	struct camera_buf *ion_buf;
	struct camera_frame *pframe;

	pr_debug("enter\n");

	ion_buf = *io_desc->buf;
	if (ion_buf == NULL) {
		pr_info("isp no init statis buf.\n");
		return ret;
	}

	kaddr = ion_buf->addr_k[0];
	paddr = ion_buf->iova[0];
	uaddr = ion_buf->addr_vir[0];
	total_size = ion_buf->size[0];

	pr_debug("size %d  addr 0x%lx 0x%lx,  0x%08x\n", (int)total_size,
			kaddr, uaddr, (uint32_t)paddr);

	buf_size = STATIS_ISP_HIST2_BUF_SIZE;
	stats_type = STATIS_HIST2;
	for (j = 0; j < STATIS_ISP_HIST2_BUF_NUM; j++) {

		used_size += buf_size;
		if (used_size >= total_size)
			break;

		pframe = get_empty_frame();

		if (pframe) {
			pframe->irq_property = stats_type;
			pframe->buf.addr_vir[0] = uaddr;
			pframe->buf.addr_k[0] = kaddr;
			pframe->buf.iova[0] = paddr;
			pframe->buf.size[0] = buf_size;
			ret = camera_enqueue(io_desc->q, pframe);
			pr_debug("outputq[%p] qcnt[%d] qmax[%d]\n",
				io_desc->q, io_desc->q->cnt,
				io_desc->q->max);
			if (ret)
				put_empty_frame(pframe);
			else
				pr_debug("kaddr %p, vaddr %p\n", (void *)kaddr, (void *)uaddr);
		}

		uaddr += buf_size;
		kaddr += buf_size;
		paddr += buf_size;

		pr_debug("isp j[%d] uaddr[%lx] kaddr[%lx] paddr[%08x] stats_type[%d]\n",
			j, uaddr, kaddr, (uint32_t)paddr, stats_type);
	}

	pr_debug("done.\n");
	return ret;
}


static int isp_deinit_statis_buffer(
		struct isp_statis_io_desc *io_desc)
{
	int ret = 0;
	struct camera_buf *ion_buf = NULL;

	ion_buf = *io_desc->buf;

	if (!ion_buf) {
		pr_info("no statis buffer.\n");
		return 0;
	}

	cambuf_kunmap(ion_buf);
	cambuf_iommu_unmap(ion_buf);
	cambuf_put_ionbuf(ion_buf);
	pr_info("done %p\n", ion_buf);
	kfree(ion_buf);

	*io_desc->buf = NULL;

	return ret;
}

static int isp_cycle_hist2_frame(
		void *isp_handle,
		int ctx_id,
		struct isp_statis_io_desc *io_desc)
{
	struct camera_frame *frame = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx = NULL;
	struct cam_hw_info *hw = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];
	hw = dev->isp_hw;
	if (hw->hw_ops.core_ops.hist_enable_get(ctx_id))
		return 0;

	frame = camera_dequeue(io_desc->q);

	if (frame == NULL) {
		pr_err("fail to deq, outputq[%p] q_cnt[%d] q_max[%d]\n",
			(void*)io_desc->q,
			io_desc->q->cnt,
			io_desc->q->max);
		return -ENOMEM;
	}

	/* give hist statis frame_id */
	frame->fid = io_desc->fid;
	if (camera_enqueue(&pctx->hist2_result_queue, frame) < 0) {
		pr_err("fail to enq, ctx_id[%d] overflow \n",ctx_id);
		if (camera_enqueue(io_desc->q, frame) < 0)
			pr_err("fail to enq, ctx_id[%d] fatal\n",ctx_id);
		return -EPERM;
	}

	return 0;
}

static int sprd_isp_cfg_sec(struct isp_pipe_dev *dev, void *param)
{

	enum sprd_cam_sec_mode  *sec_mode = (enum sprd_cam_sec_mode *)param;

	dev->sec_mode =  *sec_mode;

	pr_debug("camca: isp sec_mode=%d\n", dev->sec_mode);

	return 0;
}

static int sprd_isp_ioctl(void *isp_handle, int ctx_id,
	enum isp_ioctrl_cmd cmd, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;

	switch (cmd) {

	case ISP_IOCTL_CFG_STATIS_BUF:
		ret = isp_cfg_statis_buffer(param);
		break;
	case ISP_IOCTL_INIT_STATIS_Q:
		ret = isp_init_statis_bufferq(param);
		break;
	case ISP_IOCTL_DEINIT_STATIS_BUF:
		ret = isp_deinit_statis_buffer(param);
		break;
	case ISP_IOCTL_CYCLE_HIST2_FRAME:
		ret = isp_cycle_hist2_frame(isp_handle, ctx_id, param);
		break;
	case ISP_IOCTL_CFG_SEC:
	    ret = sprd_isp_cfg_sec(dev, param);
		break;
	default:
		pr_err("fail to get known cmd: %d\n", cmd);
		ret = -EFAULT;
		break;
	}

	return ret;
}

static int sprd_isp_cfg_blkparam(
	void *isp_handle, int ctx_id, void *param)
{
	int ret = 0;
	int i;
	struct isp_pipe_context *pctx = NULL;
	struct isp_pipe_dev *dev = NULL;
	struct isp_cfg_entry *cfg_entry = NULL;
	struct isp_io_param *io_param = NULL;
	func_isp_cfg_param cfg_fun_ptr = NULL;
	struct cam_hw_core_ops *ops = NULL;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	ops = &dev->isp_hw->hw_ops.core_ops;
	pctx = &dev->ctx[ctx_id];
	io_param = (struct isp_io_param *)param;
	if (atomic_read(&pctx->user_cnt) < 1) {
		pr_err("fail to use unable isp ctx %d.\n", ctx_id);
		return -EINVAL;
	}

	/* lock to avoid block param across frame */
	mutex_lock(&pctx->blkpm_lock);

	if (io_param->sub_block == ISP_BLOCK_NLM) {
		ret = isp_k_cfg_nlm(param, &pctx->isp_k_param, ctx_id);
	} else if (io_param->sub_block == ISP_BLOCK_3DNR) {
		if (pctx->mode_3dnr != MODE_3DNR_OFF)
			ret = isp_k_cfg_3dnr(param, &pctx->isp_k_param, ctx_id);
	} else if (io_param->sub_block == ISP_BLOCK_YNR) {
		ret = isp_k_cfg_ynr(param, &pctx->isp_k_param, ctx_id);
	} else if (io_param->sub_block == ISP_BLOCK_NOISEFILTER) {
		ret = isp_k_cfg_yuv_noisefilter(param, &pctx->isp_k_param, ctx_id);
	} else {
		i = io_param->sub_block - ISP_BLOCK_BASE;
		cfg_entry = ops->block_func_get(i, ISP_BLOCK_TYPE);
		if (cfg_entry != NULL &&
			cfg_entry->sub_block == io_param->sub_block)
			cfg_fun_ptr = cfg_entry->cfg_func;

		if (cfg_fun_ptr == NULL) {
			pr_debug("isp block 0x%x is not supported.\n",
				io_param->sub_block);
			mutex_unlock(&pctx->blkpm_lock);
			return 0;
		}

		ret = cfg_entry->cfg_func(io_param, ctx_id);
	}

	mutex_unlock(&pctx->blkpm_lock);

	return ret;
}


static int sprd_isp_set_sb(void *isp_handle, int ctx_id,
		isp_dev_callback cb, void *priv_data)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_pipe_context *pctx;

	if (!isp_handle || !cb || !priv_data) {
		pr_err("fail to get valid input ptr, isp_handle %p, callback %p, priv_data %p\n",
			isp_handle, cb, priv_data);
		return -EFAULT;
	}
	if (ctx_id < 0 || ctx_id >= ISP_CONTEXT_SW_NUM) {
		pr_err("fail to get legal ctx_id %d\n", ctx_id);
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pctx = &dev->ctx[ctx_id];
	if (pctx->isp_cb_func == NULL) {
		pctx->isp_cb_func = cb;
		pctx->cb_priv_data = priv_data;
		pr_info("ctx: %d, cb %p, %p\n",
			ctx_id, cb, priv_data);
	}

	return ret;
}


static int sprd_isp_dev_open(void *isp_handle, void *param)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	pr_info("enter.\n");
	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	dev = (struct isp_pipe_dev *)isp_handle;
	hw = (struct cam_hw_info *)param;
	if (hw == NULL) {
		pr_err("fail to get valid hw\n");
		return -EFAULT;
	}

	if (atomic_inc_return(&dev->enable) == 1) {

		pr_info("isp dev init start.\n");

		/* line_buffer_len for debug */
		if (s_dbg_linebuf_len > 0 &&
			s_dbg_linebuf_len <= ISP_LINE_BUFFER_W)
			g_camctrl.isp_linebuf_len = s_dbg_linebuf_len;
		else
			g_camctrl.isp_linebuf_len = ISP_LINE_BUFFER_W;

		if (dev->sec_mode == SEC_SPACE_PRIORITY)
			dev->wmode = ISP_AP_MODE;
		else
			dev->wmode = s_dbg_work_mode;
		g_camctrl.isp_wmode = dev->wmode;

		pr_info("camca isp sec_mode=%d, work mode: %d,  line_buf_len: %d\n",
			dev->sec_mode, dev->wmode, g_camctrl.isp_linebuf_len);

		dev->isp_hw = param;
		mutex_init(&dev->path_mutex);
		spin_lock_init(&dev->ctx_lock);

		ret = isp_hw_init(dev);
		ret = isp_hw_start(hw, dev);
		ret = isp_context_init(dev);
		if (ret) {
			pr_err("fail to init isp context.\n");
			ret = -EFAULT;
			goto err_init;
		}
	}

	pr_info("open isp pipe dev done!\n");
	return 0;

err_init:
	isp_hw_stop(hw, dev);
	isp_hw_deinit(dev);
	atomic_dec(&dev->enable);
	pr_err("fail to open isp dev!\n");
	return ret;
}



int sprd_isp_dev_close(void *isp_handle)
{
	int ret = 0;
	struct isp_pipe_dev *dev = NULL;
	struct cam_hw_info *hw;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EINVAL;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	hw = dev->isp_hw;
	if (atomic_dec_return(&dev->enable) == 0) {

		ret = isp_hw_stop(hw, dev);
		ret = isp_context_deinit(dev);
		mutex_destroy(&dev->path_mutex);
		ret = isp_hw_deinit(dev);
	}

	pr_info("isp dev disable done\n");
	return ret;

}


static int sprd_isp_dev_reset(void *isp_handle, void *param)
{
	int ret = 0;

	if (!isp_handle || !param) {
		pr_err("fail to get valid input ptr, isp_handle %p, param %p\n",
			isp_handle, param);
		return -EFAULT;
	}
	return ret;
}

static struct isp_pipe_ops isp_ops = {
	.open = sprd_isp_dev_open,
	.close = sprd_isp_dev_close,
	.reset = sprd_isp_dev_reset,
	.get_context = sprd_isp_get_context,
	.put_context = sprd_isp_put_context,
	.get_path = sprd_isp_get_path,
	.put_path = sprd_isp_put_path,
	.cfg_path = sprd_isp_cfg_path,
	.ioctl = sprd_isp_ioctl,
	.cfg_blk_param = sprd_isp_cfg_blkparam,
	.proc_frame = sprd_isp_proc_frame,
	.set_callback = sprd_isp_set_sb,
};

struct isp_pipe_ops *get_isp_ops(void)
{
	return &isp_ops;
}

void *get_isp_pipe_dev(void)
{
	struct isp_pipe_dev *dev = NULL;

	mutex_lock(&isp_pipe_dev_mutex);

	if (s_isp_dev) {
		atomic_inc(&s_isp_dev->user_cnt);
		dev = s_isp_dev;
		pr_info("s_isp_dev is already exist=%p, user_cnt=%d",
			s_isp_dev, atomic_read(&s_isp_dev->user_cnt));
		goto exit;
	}

	dev = vzalloc(sizeof(struct isp_pipe_dev));
	if (!dev)
		goto exit;

	atomic_set(&dev->user_cnt, 1);
	atomic_set(&dev->enable, 0);
	s_isp_dev = dev;
	if (dev)
		pr_info("get isp pipe dev: %p\n", dev);
exit:
	mutex_unlock(&isp_pipe_dev_mutex);

	return dev;
}


int put_isp_pipe_dev(void *isp_handle)
{
	int ret = 0;
	int user_cnt, en_cnt;
	struct isp_pipe_dev *dev = NULL;

	if (!isp_handle) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	pr_info("put isp pipe dev:%p, s_isp_dev:%p,  users: %d\n",
		dev, s_isp_dev, atomic_read(&dev->user_cnt));

	mutex_lock(&isp_pipe_dev_mutex);

	if (dev != s_isp_dev) {
		mutex_unlock(&isp_pipe_dev_mutex);
		pr_err("fail to match dev: %p, %p\n",
					dev, s_isp_dev);
		return -EINVAL;
	}

	user_cnt = atomic_read(&dev->user_cnt);
	en_cnt = atomic_read(&dev->enable);
	if (user_cnt != (en_cnt + 1))
		pr_err("fail to match %d %d\n",
				user_cnt, en_cnt);

	if (atomic_dec_return(&dev->user_cnt) == 0) {
		pr_info("free isp pipe dev %p\n", dev);
		vfree(dev);
		dev = NULL;
		s_isp_dev = NULL;
	}
	mutex_unlock(&isp_pipe_dev_mutex);

	if (dev)
		pr_info("put isp pipe dev: %p\n", dev);

	return ret;
}

void isp_hwsim_extra(uint32_t idx) {
#if 0
	uint32_t bypass = 1;

	pr_info("ctx_id[%d]\n", idx);

	pr_info("gamma\n");
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0, 1);

	pr_info("all\n");

	ISP_REG_MWR(idx, ISP_NLM_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_VST_PARA, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_IVST_PARA, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_CMC10_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_GAMMA_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_HSV_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_PSTRZ_PARAM, BIT_0, bypass);
	ISP_REG_MWR(idx, ISP_CCE_PARAM, BIT_0, bypass);

	ISP_REG_MWR(idx, ISP_UVD_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_PRECDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YNR_CONTRL0, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST_CFG_READY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_HIST2_CFG_RDY, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CFAE_NEW_CFG0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_GRGB_CTRL, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YUV_NF_CTRL, BIT_0, 1);

	ISP_REG_MWR(idx, ISP_LTM_HIST_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_LTM_MAP_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_CDN_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_EE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_BCHS_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_POSTCDN_COMMON_CTRL, BIT_0|BIT_1, 0x3);
	ISP_REG_MWR(idx, ISP_YGAMMA_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_IIRCNR_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_YRANDOM_PARAM1, BIT_0, 1);

	/* 3DNR bypass */
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PARAM0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_BLEND_CONTROL0, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_STORE_PARAM, BIT_0, 1);
	ISP_REG_MWR(idx, ISP_3DNR_MEM_CTRL_PRE_PARAM0, BIT_0, 1);
#endif
}
