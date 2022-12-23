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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/sprd_iommu.h>
#include <linux/uaccess.h>

#include "cam_buf_manager.h"
#include "cam_debugger.h"
#include "cam_pipeline.h"
#include "cam_scene.h"
#include "cam_test.h"
#include "cam_trusty.h"
#include "cam_zoom.h"
#include "csi_api.h"
#include "dcam_core.h"
#include "dcam_hw_adpt.h"
#include "dcam_int.h"
#include "dcam_slice.h"
#include "flash_interface.h"
#include "isp_dev.h"
#include "isp_drv.h"
#include "isp_hw.h"
#include "isp_interface.h"
#include "isp_scaler_node.h"
#include "pyr_dec_node.h"
#include "sprd_img.h"
#include "sprd_sensor_drv.h"
#include <sprd_camsys_domain.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_CORE: %d %d %s : " fmt, current->pid, __LINE__, __func__

spinlock_t g_reg_wr_lock;
struct camera_queue *g_ion_buf_q;
struct camera_queue *g_empty_zoom_q;
struct camera_queue *g_empty_frm_q;
struct camera_queue *g_empty_interruption_q;

struct cam_global_ctrl g_camctrl = {
	ZOOM_BINNING2,
	0,
	ISP_MAX_LINE_WIDTH
};

static int camcore_dcam_online_buf_cfg(struct channel_context *ch,
		struct camera_frame *pframe, struct camera_module *module)
{
	int ret = 0;

	if (!ch || !pframe || !module) {
		pr_err("fail to get valid ptr %px %px\n", ch, pframe);
		return -EFAULT;
	}

	cam_queue_frame_flag_reset(pframe);
	ret = CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF, pframe);
	return ret;
}

static int camcore_capture_3dnr_set(struct camera_module *module,
		struct channel_context *ch)
{
	uint32_t mode_3dnr = 0;

	if ((!module) || (!ch))
		return -EFAULT;
	mode_3dnr = MODE_3DNR_OFF;
	if (ch->uinfo_3dnr) {
		if (ch->ch_id == CAM_CH_CAP)
			mode_3dnr = MODE_3DNR_CAP;
		else
			mode_3dnr = MODE_3DNR_PRE;
	}
	pr_debug("mode %d\n", mode_3dnr);
	CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_3DNR_MODE, &mode_3dnr);
	return 0;
}

static void camcore_k_frame_put(void *param)
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
		if (frame->buf.mapping_state & CAM_BUF_MAPPING_KERNEL)
			cam_buf_kunmap(&frame->buf);
		if (frame->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&frame->buf);
		cam_buf_free(&frame->buf);
	}
	cam_queue_empty_frame_put(frame);
}

static void camcore_ion_frame_put(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cam_buf_manager_buf_status_change(&frame->buf, CAM_BUF_ALLOC, CAM_IOMMUDEV_MAX);
	cam_queue_empty_frame_put(frame);
}

static void camcore_empty_frame_put(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cam_buf_manager_buf_status_change(&frame->buf, CAM_BUF_ALLOC, CAM_IOMMUDEV_MAX);
	cam_queue_empty_frame_put(frame);
}

static void camcore_empty_zoom_put(void *param)
{
	struct cam_zoom_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}

	frame = (struct cam_zoom_frame *)param;
	cam_queue_empty_zoom_put(frame);
}

static void camcore_camera_frame_release(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param)
		return;
	frame = (struct camera_frame *)param;
	cam_queue_empty_frame_put(frame);
}

static int camcore_resframe_set(struct camera_module *module)
{
	int ret = 0;
	struct channel_context *ch = NULL;
	uint32_t i = 0;
	uint32_t max_size = 0, out_size = 0, in_size = 0, res_size = 0;
	uint32_t src_w = 0, src_h = 0;
	struct camera_frame *pframe = NULL;
	struct dcam_compress_cal_para cal_fbc = {0};
	struct dcam_statis_param statis_param = {0};

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (ch->enable) {
			src_w = ch->ch_uinfo.src_size.w;
			src_h = ch->ch_uinfo.src_size.h;
			if (ch->ch_uinfo.dst_fmt != IMG_PIX_FMT_GREY)
				out_size = ch->ch_uinfo.dst_size.w *ch->ch_uinfo.dst_size.h * 3 / 2;
			else
				out_size = cal_sprd_pitch(ch->ch_uinfo.dst_size.w, ch->ch_uinfo.dcam_raw_fmt)
					* ch->ch_uinfo.dst_size.h;

			if (ch->compress_en) {
				cal_fbc.data_bits = cam_data_bits(ch->dcam_out_fmt);
				cal_fbc.fmt = ch->dcam_out_fmt;
				cal_fbc.height = src_h;
				cal_fbc.width = src_w;
				in_size = dcam_if_cal_compressed_size (&cal_fbc);
			} else if (ch->ch_uinfo.sn_fmt != IMG_PIX_FMT_GREY)
				in_size = src_w * src_h * 3 / 2;
			else if (camcore_raw_fmt_get(ch->dcam_out_fmt))
				in_size = cal_sprd_size(src_w, src_h, ch->ch_uinfo.dcam_raw_fmt);
			else
				in_size = cal_sprd_size(src_w, src_h, ch->dcam_out_fmt);

			if (module->cam_uinfo.is_pyr_rec && ch->ch_id != CAM_CH_CAP)
				in_size += dcam_if_cal_pyramid_size(src_w, src_h, cam_data_bits(ch->ch_uinfo.pyr_out_fmt), cam_is_pack(ch->ch_uinfo.pyr_out_fmt), 1, DCAM_PYR_DEC_LAYER_NUM);
			in_size = ALIGN(in_size, CAM_BUF_ALIGN_SIZE);

			max_size = max3(max_size, out_size, in_size);
			pr_debug("cam%d, ch %d, max_size = %d, %d, %d\n", module->idx, i, max_size, in_size, out_size);
		}
	}

	ch = &module->channel[CAM_CH_PRE];
	if (ch->enable && ch->pipeline_handle) {
		statis_param.statis_cmd = DCAM_IOCTL_INIT_STATIS_Q;
		res_size = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
	} else {
		ch = &module->channel[CAM_CH_CAP];
		if (ch->enable && module->cam_uinfo.is_longexp) {
			statis_param.statis_cmd = DCAM_IOCTL_INIT_STATIS_Q;
			res_size = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
		}
	}

	pframe = cam_queue_empty_frame_get();
	pframe->is_reserved = CAM_RESERVED_BUFFER_ORI;
	pframe->buf.type = CAM_BUF_USER;
	pframe->buf.mfd = module->reserved_buf_fd;
	pframe->buf.status = CAM_BUF_ALLOC;
	ret = cam_buf_manager_buf_status_change(&pframe->buf, CAM_BUF_WITH_ION, CAM_IOMMUDEV_MAX);
	if (ret) {
		pr_err("fail to get ionbuf on cam%d\n", module->idx);
		cam_queue_empty_frame_put(pframe);
		ret = -EFAULT;
	}
	pframe->buf.size = max(max_size, res_size);
	pframe->buf.status = CAM_BUF_WITH_ION;
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || ch->ch_id == CAM_CH_VIRTUAL)
			continue;
		if (ch->isp_port_id >= 0 && ch->ch_uinfo.dst_fmt != IMG_PIX_FMT_GREY) {
			pframe->channel_id = ch->ch_id;
			if (module->cam_uinfo.is_pyr_rec && ch->ch_id != CAM_CH_CAP)
				pframe->pyr_status = ONLINE_DEC_ON;
			ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPILINE_CFG_RESEVER_BUF, pframe);
			if (ret) {
				pr_err("fail to set output buffer for ch%d.\n", ch->ch_id);
				cam_buf_manager_buf_status_change(&pframe->buf, CAM_BUF_ALLOC, CAM_IOMMUDEV_MAX);
				cam_queue_empty_frame_put(pframe);
				ret = -EFAULT;
				break;
			}
		}
	}
	cam_scene_reserved_buf_cfg(RESERVED_BUF_SET_CB, pframe, module);
	module->res_frame = pframe;

	return ret;
}

static void camcore_compression_cal(struct camera_module *module)
{
	uint32_t nr3_compress_eb = 0, hw_ctx_id = 0;
	struct channel_context *ch_pre = NULL, *ch_cap = NULL, *ch_vid = NULL, *ch_raw = NULL;
	struct cam_hw_info *dcam_hw = NULL;

	dcam_hw = module->grp->hw_info;
	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	ch_vid = &module->channel[CAM_CH_VID];
	ch_raw = &module->channel[CAM_CH_RAW];

	/* Enable compression for DCAM path by default. Full path is prior to bin path. */
	ch_cap->compress_en = ch_cap->enable
		&& ch_cap->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY
		&& !ch_cap->ch_uinfo.is_high_fps
		&& !module->cam_uinfo.is_4in1
		&& dcam_hw->ip_dcam[hw_ctx_id]->dcam_full_fbc_support
		&& !(g_dbg_fbc_control & DEBUG_FBC_CRL_FULL);
	ch_pre->compress_en = ch_pre->enable
		&& ch_pre->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY
		&& !ch_pre->ch_uinfo.is_high_fps
		&& !module->cam_uinfo.is_4in1
		&& dcam_hw->ip_dcam[hw_ctx_id]->dcam_bin_fbc_support
		&& !(g_dbg_fbc_control & DEBUG_FBC_CRL_BIN);
	ch_raw->compress_en = ch_raw->enable
		&& ch_raw->ch_uinfo.sn_fmt == IMG_PIX_FMT_GREY
		&& !ch_raw->ch_uinfo.is_high_fps
		&& !module->cam_uinfo.is_4in1
		&& dcam_hw->ip_dcam[hw_ctx_id]->dcam_raw_fbc_support
		&& !(g_dbg_fbc_control & DEBUG_FBC_CRL_RAW);

	ch_cap->compress_offline = dcam_hw->ip_dcam[hw_ctx_id]->dcam_offline_fbc_support;
	ch_vid->compress_en = ch_pre->compress_en;
	ch_raw->compress_en = ch_cap->compress_en ? 0 : ch_raw->compress_en;

	/* Disable compression for 3DNR by default */
	nr3_compress_eb = module->isp_dev_handle->isp_hw->ip_isp->nr3_compress_support;
	if (ch_cap->uinfo_3dnr)
		ch_cap->compress_3dnr = nr3_compress_eb;
	if (ch_pre->uinfo_3dnr) {
		ch_pre->compress_3dnr = nr3_compress_eb;
		ch_vid->compress_3dnr = ch_pre->compress_3dnr;
	}
	ch_raw->compress_3dnr = 0;

	/* dcam not support fbc when dcam need fetch */
	if (module->cam_uinfo.dcam_slice_mode ||
		module->cam_uinfo.is_4in1 || module->cam_uinfo.is_raw_alg) {
		ch_cap->compress_en = 0;
		ch_raw->compress_en = 0;
	}

	pr_info("cam%d: cap %u, pre %u, vid %u, raw %u, offline %u, 3dnr %u %u %u.\n",
		module->idx,
		ch_cap->compress_en, ch_pre->compress_en,
		ch_vid->compress_en, ch_raw->compress_en,
		ch_cap->compress_offline,
		ch_cap->compress_3dnr,
		ch_pre->compress_3dnr,
		ch_vid->compress_3dnr);
}

static int camcore_pyr_info_config(
	struct camera_module *module, struct channel_context *channel)
{
	int ret = 0;
	uint32_t pyr_layer_num = 0;

	if (module->cam_uinfo.is_pyr_rec && channel->ch_id == CAM_CH_PRE) {
		channel->pyr_layer_num = DCAM_PYR_DEC_LAYER_NUM;
		pyr_layer_num = DCAM_PYR_DEC_LAYER_NUM;
	}

	if (module->cam_uinfo.is_pyr_dec && channel->ch_id == CAM_CH_CAP) {
		channel->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
		pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
	}

	if (channel->ch_id < CAM_CH_PRE_THM)
		CAM_PIPEINE_ISP_NODE_CFG(channel, CAM_PIPELINE_CFG_REC_LEYER_NUM, &pyr_layer_num);

	return ret;
}

static int camcore_buffer_channel_config(
	struct camera_module *module, struct channel_context *channel)
{
	struct channel_context *ch_vid = NULL;
	struct camera_uchannel *ch_uinfo = NULL;
	struct camera_frame *alloc_buf = NULL;
	struct channel_context *ch_pre = NULL;
	struct cam_hw_info *hw = NULL;

	if (!module || !channel) {
		pr_err("fail to get valid param %p %p\n", module, channel);
		return -EFAULT;
	}

	ch_uinfo = &channel->ch_uinfo;
	ch_pre = &module->channel[CAM_CH_PRE];
	ch_vid = &module->channel[CAM_CH_VID];
	hw = module->grp->hw_info;

	if (channel->ch_id != CAM_CH_RAW)
		camcore_pyr_info_config(module, channel);

	if ((channel->swap_size.w > 0) && (channel->ch_id != CAM_CH_RAW)) {
		cam_queue_init(&channel->share_buf_queue, CAM_SHARED_BUF_NUM, camcore_k_frame_put);

		/* alloc middle buffer for channel */
		mutex_lock(&module->buf_lock[channel->ch_id]);
		channel->alloc_start = 1;
		mutex_unlock(&module->buf_lock[channel->ch_id]);

		alloc_buf = cam_queue_empty_frame_get();
		alloc_buf->priv_data = (void *)channel;
		cam_queue_enqueue(&module->alloc_queue, &alloc_buf->list);
		complete(&module->buf_thrd.thread_com);
	}

	return 0;
}

static int camcore_buffer_path_cfg(struct camera_module *module, uint32_t index)
{
	int ret = 0;
	uint32_t j = 0;
	struct channel_context *ch = NULL;
	struct cam_hw_info *hw = NULL;

	if (!module) {
		pr_err("fail to get input ptr\n");
		return -EFAULT;
	}

	ch = &module->channel[index];
	hw = module->grp->hw_info;

	if (index == CAM_CH_PRE || index == CAM_CH_VID
		|| (index == CAM_CH_CAP && !module->channel[CAM_CH_PRE].enable && !module->cam_uinfo.is_4in1)) {
		ret = wait_for_completion_interruptible(&ch->stream_on_buf_com);
		if (ret != 0) {
			pr_err("fail to config channel/path param work %d\n", ret);
			goto exit;
		}
	}

	if (atomic_read(&ch->err_status) != 0) {
		pr_err("fail to get ch %d correct status\n", ch->ch_id);
		ret = -EFAULT;
		goto exit;
	}

	for (j = 0; j < ISP_NR3_BUF_NUM; j++) {
		if (ch->nr3_bufs[j] == NULL)
			continue;
		ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_3DNR_BUF, ch->nr3_bufs[j]);
		if (ret) {
			pr_err("fail to config isp 3DNR buffer\n");
			goto exit;
		}
	}

	if (hw->ip_dcam[0]->superzoom_support) {
		if (ch->postproc_buf) {
			ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_SUPERZOOM_BUF, ch->postproc_buf);
			if (ret) {
				pr_err("fail to config isp superzoom buffer\n");
				goto exit;
			}
		}
	}

	if (module->cam_uinfo.is_pyr_dec && ch->ch_id == CAM_CH_CAP) {
		if (ch->pyr_dec_buf) {
			ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPILINE_CFG_PYRDEC_BUF, ch->pyr_dec_buf);
			if (ret) {
				pr_err("fail to config isp pyr_dec buffer sw, path id\n");
				goto exit;
			}
		}
	}

	if ((module->cam_uinfo.is_pyr_rec && ch->ch_id != CAM_CH_CAP)
		|| (module->cam_uinfo.is_pyr_dec && ch->ch_id == CAM_CH_CAP)) {
		if (ch->pyr_rec_buf) {
			ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_REC_BUF, ch->pyr_rec_buf);
			if (ret) {
				pr_err("fail to config isp pyr_rec buffer sw\n");
				goto exit;
			}
		}
	}

exit:
	return ret;
}

static int camcore_buffer_ltm_cfg(struct camera_module *module, uint32_t index)
{
	int ret = 0;
	uint32_t j = 0;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL;
	struct channel_context *ch_cap = NULL;

	if (!module) {
		pr_err("fail to get input ptr\n");
		return -EFAULT;
	}

	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	/*non-zsl capture, or video path while preview path enable, do nothing*/
	if ((!ch_pre->enable && ch_cap->enable) || (index == CAM_CH_VID && ch_pre->enable))
		return 0;

	ch = &module->channel[index];

	if (module->cam_uinfo.is_rgb_ltm) {
		for (j = 0; j < ISP_LTM_BUF_NUM; j++) {
			ch->ltm_bufs[j] = ch_pre->ltm_bufs[j];
			if (ch->ltm_bufs[j] == NULL) {
				pr_err("fail to get rgb_buf ch->ltm_bufs[%d] NULL, index : %x\n", j, index);
				goto exit;
			}
			ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_LTM_BUF, ch->ltm_bufs[j]);

			if (ret) {
				pr_err("fail to config isp rgb LTM buffer\n");
				goto exit;
			}
		}
	}

exit:
	return ret;
}

static int camcore_faceid_secbuf(uint32_t sec, struct camera_buf *buf)
{
	int ret = 0;
	bool vaor_bp_en = 0;
	if (sec) {
		buf->buf_sec = 1;
		vaor_bp_en = true;
		ret = sprd_iommu_set_cam_bypass(vaor_bp_en);
		if (unlikely(ret)) {
			pr_err("fail to enable vaor bypass mode, ret %d\n", ret);
			ret = -EFAULT;
		}
	}

	return ret;
}

static inline int camcore_mulsharebuf_verif(struct channel_context *ch, struct camera_uinfo *info)
{
	return ch->ch_id == CAM_CH_CAP && info->need_share_buf && !info->dcam_slice_mode && !info->is_4in1;
}

static int camcore_buffers_alloc(void *param)
{
	int ret = 0;
	int hw_ctx_id = 0;
	int i = 0, total = 0, iommu_enable = 0;
	uint32_t width = 0, height = 0, block_size = 0;
	uint32_t postproc_w = 0, postproc_h = 0;
	uint32_t is_super_size = 0, sec_mode = 0;
	struct camera_module *module = NULL;
	struct camera_frame *pframe = NULL;
	struct channel_context *channel = NULL;
	struct channel_context *channel_vid = NULL;
	struct camera_debugger *debugger = NULL;
	struct cam_hw_info *hw = NULL;
	struct camera_frame *alloc_buf = NULL;
	struct camera_group *grp = NULL;
	struct camera_queue *cap_buf_q = NULL;
	struct camera_frame *pframe_dec = NULL;
	struct camera_frame *pframe_rec = NULL;
	struct cam_buf_alloc_desc alloc_param = {0};

	pr_info("enter.\n");

	module = (struct camera_module *)param;
	grp = module->grp;
	alloc_buf = cam_queue_dequeue(&module->alloc_queue,
		struct camera_frame, list);

	if (alloc_buf) {
		channel = (struct channel_context *)alloc_buf->priv_data;
		cam_queue_empty_frame_put(alloc_buf);
	} else {
		pr_err("fail to dequeue alloc_buf\n");
		return -1;
	}

	hw = module->grp->hw_info;
	iommu_enable = module->iommu_enable;
	channel_vid = &module->channel[CAM_CH_VID];
	sec_mode = module->grp->camsec_cfg.camsec_mode;

	total = cam_scene_buffers_alloc_num(channel, module);
	if (camcore_mulsharebuf_verif(channel, &module->cam_uinfo)) {
		cap_buf_q = &grp->mul_share_buf_q;
		width = grp->mul_sn_max_size.w;
		height = grp->mul_sn_max_size.h;
		grp->is_mul_buf_share = 1;
	} else {
		cap_buf_q = &channel->share_buf_queue;
		width = channel->swap_size.w;
		height = channel->swap_size.h;
	}

	alloc_param.ch_id = channel->ch_id;
	alloc_param.cam_idx = module->idx;
	alloc_param.width = width;
	alloc_param.height = height;
	alloc_param.sensor_img_ptn = module->cam_uinfo.sensor_if.img_ptn;
	alloc_param.is_pyr_rec = module->cam_uinfo.is_pyr_rec;
	alloc_param.is_pyr_dec = module->cam_uinfo.is_pyr_dec;
	alloc_param.sec_mode = sec_mode;
	alloc_param.iommu_enable = iommu_enable;
	alloc_param.compress_en = channel->compress_en;
	alloc_param.compress_offline = channel->compress_offline;
	alloc_param.share_buffer = camcore_mulsharebuf_verif(channel, &module->cam_uinfo);
	alloc_param.dcamonline_buf_alloc_num = total;
	alloc_param.dcamoffline_buf_alloc_num = 1;
	if (module->cam_uinfo.raw_alg_type == RAW_ALG_AI_SFNR)
		alloc_param.dcamoffline_buf_alloc_num = 0;
	alloc_param.stream_on_buf_com = &channel->stream_on_buf_com;
	if (channel->ch_id == CAM_CH_PRE || channel->ch_id == CAM_CH_VID) {
		if (channel->ch_uinfo.is_high_fps)
			alloc_param.stream_on_need_buf_num = channel->ch_uinfo.high_fps_skip_num;
		else
			alloc_param.stream_on_need_buf_num = 1;
	} else
		alloc_param.stream_on_need_buf_num = total;

	debugger = &module->grp->debugger;
	is_super_size = (module->cam_uinfo.dcam_slice_mode == CAM_OFFLINE_SLICE_HW
		&& width >= DCAM_HW_SLICE_WIDTH_MAX) ? 1 : 0;
	if (hw->ip_dcam[hw_ctx_id]->superzoom_support && !is_super_size) {
		postproc_w = channel->ch_uinfo.dst_size.w / ISP_SCALER_UP_MAX;
		postproc_h = channel->ch_uinfo.dst_size.h / ISP_SCALER_UP_MAX;
		if (channel->ch_id != CAM_CH_CAP && channel_vid->enable) {
			postproc_w = MAX(channel->ch_uinfo.dst_size.w,
				channel_vid->ch_uinfo.dst_size.w) / ISP_SCALER_UP_MAX;
			postproc_h = MAX(channel->ch_uinfo.dst_size.h,
				channel_vid->ch_uinfo.dst_size.h) / ISP_SCALER_UP_MAX;
		}

		block_size = ((postproc_w + 1) & (~1)) * postproc_h * 3 / 2;
		block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		if (!pframe) {
			pr_err("fail to superzoom no empty frame.\n");
			ret = -EINVAL;
			goto exit;
		}

		pframe->channel_id = channel->ch_id;
		if (sec_mode != SEC_UNABLE)
			pframe->buf.buf_sec = 1;

		ret = cam_buf_alloc(&pframe->buf, block_size, iommu_enable);
		if (ret) {
			pr_err("fail to alloc superzoom buf\n");
			cam_queue_empty_frame_put(pframe);
			atomic_inc(&channel->err_status);
			goto exit;
		}

		channel->postproc_buf = pframe;
		pr_info("hw_ctx_id %d, superzoom w %d, h %d, buf %p\n",
			hw_ctx_id, postproc_w, postproc_h, pframe);
	}

	pr_debug("channel->ch_id = %d, channel->type_3dnr = %d, channel->uinfo_3dnr = %d\n",
		channel->ch_id, channel->type_3dnr, channel->uinfo_3dnr);
	if ((channel->type_3dnr == CAM_3DNR_HW) &&
		(!((channel->uinfo_3dnr == 0) && (channel->ch_id == CAM_CH_PRE)))) {
		/* YUV420 for 3DNR ref*/
		if (channel->compress_3dnr)
			block_size = isp_3dnr_cal_compressed_size(width, height);
		else {
			if ((channel->dcam_out_fmt == CAM_YUV420_2FRAME)
				|| (channel->dcam_out_fmt == CAM_YVU420_2FRAME)
				|| (channel->dcam_out_fmt == CAM_YUV420_2FRAME_MIPI)) {
				block_size = ((width + 1) & (~1)) * height * 3;
				block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);
			} else {
				block_size = ((width + 1) & (~1)) * height * 3 / 2;
				block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);
			}
		}

		pr_info("ch %d 3dnr buffer size: %u.\n", channel->ch_id, block_size);
		for (i = 0; i < ISP_NR3_BUF_NUM; i++) {
			pframe = cam_queue_empty_frame_get();

			if (channel->ch_id == CAM_CH_PRE && sec_mode != SEC_UNABLE)
				pframe->buf.buf_sec = 1;

			ret = cam_buf_alloc(&pframe->buf, block_size, iommu_enable);
			if (ret) {
				pr_err("fail to alloc 3dnr buf: %d ch %d\n", i, channel->ch_id);
				cam_queue_empty_frame_put(pframe);
				atomic_inc(&channel->err_status);
				goto exit;
			}
			channel->nr3_bufs[i] = pframe;
		}
	}

	if (channel->mode_ltm != MODE_LTM_OFF) {
		/* todo: ltm buffer size needs to be refined.*/
		/* size = ((width + 1) & (~1)) * height * 3 / 2; */
		/*
		 * sizeof histo from 1 tile: 128 * 16 bit
		 * MAX tile num: 8 * 8
		 */
		block_size = 64 * 128 * 2;
		block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);

		pr_info("ch %d ltm buffer size: %u.\n", channel->ch_id, block_size);
		if (channel->ltm_rgb) {
			for (i = 0; i < ISP_LTM_BUF_NUM; i++) {
				if (channel->ch_id == CAM_CH_PRE) {
					pframe = cam_queue_empty_frame_get();

					if (channel->ch_id == CAM_CH_PRE
						&& sec_mode == SEC_TIME_PRIORITY)
						pframe->buf.buf_sec = 1;
					ret = cam_buf_alloc(&pframe->buf, block_size, iommu_enable);
					if (ret) {
						pr_err("fail to alloc ltm buf: %d ch %d\n", i, channel->ch_id);
						cam_queue_empty_frame_put(pframe);
						atomic_inc(&channel->err_status);
						goto exit;
					}
					channel->ltm_bufs[i] = pframe;
				}
			}
		}
	}

	if (channel->ch_id == CAM_CH_CAP && grp->is_mul_buf_share && (module->cam_uinfo.is_pyr_rec || module->cam_uinfo.is_pyr_dec))
		mutex_lock(&grp->pyr_mulshare_lock);

	if ((channel->ch_id == CAM_CH_CAP && grp->is_mul_buf_share
		&& (module->cam_uinfo.is_pyr_rec || module->cam_uinfo.is_pyr_dec)
		&& atomic_inc_return(&grp->mul_pyr_buf_alloced) > 1)) {
		pframe_rec = cam_queue_empty_frame_get();
		memcpy(pframe_rec, grp->mul_share_pyr_rec_buf, sizeof(struct camera_frame));
		channel->pyr_rec_buf = pframe_rec;
		pframe_dec = cam_queue_empty_frame_get();
		memcpy(pframe_dec, grp->mul_share_pyr_dec_buf, sizeof(struct camera_frame));
		channel->pyr_dec_buf = pframe_dec;
		goto mul_pyr_alloc_end;
	}

	if ((module->cam_uinfo.is_pyr_rec && channel->ch_id != CAM_CH_CAP)
		|| (module->cam_uinfo.is_pyr_dec && channel->ch_id == CAM_CH_CAP)) {
		width = isp_rec_layer0_width(width, channel->pyr_layer_num);
		height = isp_rec_layer0_heigh(height, channel->pyr_layer_num);
		/* rec temp buf max size is equal to layer1 size: w/2 * h/2 */
		block_size = dcam_if_cal_pyramid_size(width, height, cam_data_bits(channel->ch_uinfo.pyr_out_fmt), cam_is_pack(channel->ch_uinfo.pyr_out_fmt), 1, channel->pyr_layer_num - 1);
		block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		if (channel->ch_id == CAM_CH_PRE && sec_mode == SEC_TIME_PRIORITY)
			pframe->buf.buf_sec = 1;
		pframe->width = width;
		pframe->height = height;
		pframe->channel_id = channel->ch_id;
		ret = cam_buf_alloc(&pframe->buf, block_size, iommu_enable);
		if (ret) {
			pr_err("fail to alloc rec buf\n");
			cam_queue_empty_frame_put(pframe);
			atomic_inc(&channel->err_status);
			goto exit;
		}
		if (camcore_mulsharebuf_verif(channel, &module->cam_uinfo)) {
			grp->mul_share_pyr_rec_buf = pframe;
			pframe_rec = cam_queue_empty_frame_get();
			memcpy(pframe_rec, grp->mul_share_pyr_rec_buf, sizeof(struct camera_frame));
			channel->pyr_rec_buf = pframe_rec;
		} else
			channel->pyr_rec_buf = pframe;

		pr_debug("hw_ctx_id %d, pyr_rec w %d, h %d, buf %p\n",
				hw_ctx_id, width, height, pframe);
	}

	if (module->cam_uinfo.is_pyr_dec && channel->ch_id == CAM_CH_CAP) {
		block_size = dcam_if_cal_pyramid_size(width, height, cam_data_bits(channel->ch_uinfo.pyr_out_fmt), cam_is_pack(channel->ch_uinfo.pyr_out_fmt), 0, ISP_PYR_DEC_LAYER_NUM);
		block_size = ALIGN(block_size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		if (channel->ch_id == CAM_CH_PRE && sec_mode == SEC_TIME_PRIORITY)
			pframe->buf.buf_sec = 1;
		pframe->width = width;
		pframe->height = height;
		pframe->channel_id = channel->ch_id;
		pframe->data_src_dec = 1;
		pframe->is_compressed = 0;
		ret = cam_buf_alloc(&pframe->buf, block_size, iommu_enable);
		if (ret) {
			pr_err("fail to alloc dec buf\n");
			cam_queue_empty_frame_put(pframe);
			atomic_inc(&channel->err_status);
			goto exit;
		}
		if (camcore_mulsharebuf_verif(channel, &module->cam_uinfo)) {
			grp->mul_share_pyr_dec_buf = pframe;
			pframe_dec = cam_queue_empty_frame_get();
			memcpy(pframe_dec, grp->mul_share_pyr_dec_buf, sizeof(struct camera_frame));
			channel->pyr_dec_buf = pframe_dec;
		} else
			channel->pyr_dec_buf = pframe;

		pr_debug("hw_ctx_id %d, ch %d pyr_dec size %d, buf %p, w %d h %d\n",
			hw_ctx_id, channel->ch_id, block_size, pframe, width, height);
	}
mul_pyr_alloc_end:
	if (channel->ch_id == CAM_CH_CAP && grp->is_mul_buf_share && (module->cam_uinfo.is_pyr_rec || module->cam_uinfo.is_pyr_dec))
		mutex_unlock(&grp->pyr_mulshare_lock);

	if (channel->ch_id == CAM_CH_CAP && ((grp->is_mul_buf_share && atomic_inc_return(&grp->mul_buf_alloced) > 1) ||
		((module->cam_uinfo.param_frame_sync || module->cam_uinfo.raw_alg_type != RAW_ALG_AI_SFNR) && module->cam_uinfo.is_raw_alg)))
		goto exit;

	ret = cam_pipeline_buffer_alloc(channel->pipeline_handle, &alloc_param);
	if (ret) {
		pr_err("fail to alloc buffer for cam%d channel%d\n", module->idx, channel->ch_id);
		atomic_inc(&channel->err_status);
	}

exit:

	if (channel->ch_id != CAM_CH_PRE && module->channel[CAM_CH_PRE].enable) {
		ret = camcore_buffer_path_cfg(module, channel->ch_id);
		if (ret)
			pr_err("fail to cfg path buffer\n");
	}
	complete(&channel->alloc_com);
	channel->alloc_start = 0;
	pr_info("ch %d done. status %d\n", channel->ch_id, atomic_read(&channel->err_status));
	ret = cam_buf_mdbg_check();
	return ret;
}

static int camcore_dual_same_frame_get(struct camera_module *module, struct camera_frame *mframe)
{
	int i = 0, j = 0, ret = 0;
	int64_t t_sec = 0, t_usec = 0;
	struct camera_group *grp = NULL;
	struct camera_module *pmd[CAM_COUNT] = {0};
	struct camera_queue *q = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_module *slave_module = NULL;
	struct channel_context *ch = NULL;

	grp = module->grp;
	if (!grp)
		return -EFAULT;
	/* get the slave module */
	for (i = 0, j = 0; i < CAM_COUNT; i++) {
		pmd[j] = grp->module[i];
		if (!pmd[j])
			continue;
		if (pmd[j] != module) {
			slave_module = pmd[j];
			break;
		}
	}

	if (atomic_read(&slave_module->state) != CAM_RUNNING) {
		pr_info("slave sensor no running , current state: %d\n", atomic_read(&slave_module->state));
		return ret;
	}
	ch = &slave_module->channel[CAM_CH_CAP];
	ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_DUAL_SYNC_BUF_GET, &q);
	if (ret) {
		pr_err("fail to get buffer\n");
		return -EFAULT;
	}

	t_sec = mframe->sensor_time.tv_sec;
	t_usec = mframe->sensor_time.tv_usec;
	ret = cam_queue_same_frame_get(q, &pframe, t_sec, t_usec);
	if (ret) {
		slave_module->dual_frame = NULL;
		atomic_set(&slave_module->dual_select_frame_done, 1);
		atomic_set(&module->dual_select_frame_done, 1);
		return ret;
	}
	slave_module->dual_frame = pframe;
	atomic_set(&slave_module->dual_select_frame_done, 1);
	atomic_set(&module->dual_select_frame_done, 1);

	return 0;
}

static int camcore_dual_slave_frame_set(void *param, void *priv_data)
{
	struct camera_frame *pframe = NULL;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;
	int ret = CAM_FRAME_NO_DEAL;
	unsigned long flag = 0;

	pframe = (struct camera_frame *)param;
	module = (struct camera_module *)priv_data;
	grp = module->grp;

	spin_lock_irqsave(&grp->dual_deal_lock, flag);
	if (module->master_flag == 0 && atomic_read(&module->dual_select_frame_done) == 1 &&
		module->dual_frame == NULL && module->capture_type == CAM_CAPTURE_STOP) {
		module->dual_frame = pframe;
		ret = CAM_FRAME_DEAL;
	}
	spin_unlock_irqrestore(&grp->dual_deal_lock, flag);

	return ret;
}

static struct camera_frame *camcore_dual_frame_deal(void *param, void *priv_data, int *frame_flag)
{
	struct channel_context *channel = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;
	int ret = 0;
	unsigned long flag = 0;

	if (!param || !priv_data) {
		pr_err("fail to get valid ptr %px\n", param);
		return NULL;
	}

	pframe = (struct camera_frame *)param;
	module = (struct camera_module *)priv_data;
	channel = &module->channel[pframe->channel_id];
	grp = module->grp;

	spin_lock_irqsave(&grp->dual_deal_lock, flag);
	if (module->master_flag == 0) {
		if (atomic_read(&module->dual_select_frame_done) == 0)
			*frame_flag = CAM_FRAME_NO_DEAL;
		else {
			if (module->dual_frame) {
				camcore_dcam_online_buf_cfg(channel, pframe, module);
				pframe = module->dual_frame;
				module->dual_frame = NULL;
			}
			*frame_flag = CAM_FRAME_DEAL;
		}
	} else {
		if (pframe->boot_sensor_time < module->capture_times)
			*frame_flag = CAM_FRAME_NO_DEAL;
		else {
			if (atomic_read(&module->dual_select_frame_done) == 0) {
				ret = camcore_dual_same_frame_get(module , pframe);
				if (ret)
					pr_debug("No find match frame, adapt next slave frame\n");
			}
			*frame_flag = CAM_FRAME_DEAL;
		}
	}
	spin_unlock_irqrestore(&grp->dual_deal_lock, flag);

	return pframe;
}

static uint32_t camcore_frame_start_proc(struct camera_module *module, struct camera_frame *pframe, enum cam_node_type node_type)
{
	int ret = 0;
	struct channel_context *ch = NULL;
	struct cam_pipeline_cfg_param param = {0};

	ch = &module->channel[CAM_CH_CAP];
	param.node_type = node_type;
	param.node_param.param = pframe;
	param.node_param.port_id = ch->dcam_port_id;
	ret = ch->pipeline_handle->ops.streamon(ch->pipeline_handle, &param);

	return ret;
}

static void *cam_zoom_param_get_callback(uint32_t pipeline_type, void *priv_data)
{
	int i = 0;
	unsigned long flag = 0;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;
	struct cam_zoom_frame *pframe = NULL;

	if (!priv_data) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	module = (struct camera_module *)priv_data;
	for (i = 0; i < CAM_CH_MAX; i++) {
		channel = &module->channel[i];
		if (!channel->enable || !channel->pipeline_handle)
			continue;
		if (channel->pipeline_type == pipeline_type) {
			pr_debug("ch %d type %d\n", channel->ch_id, pipeline_type);
			pframe = cam_queue_dequeue(&channel->zoom_param_q, struct cam_zoom_frame, list);
			if (!pframe) {
				pframe = cam_queue_empty_zoom_get();
				spin_lock_irqsave(&channel->lastest_zoom_lock, flag);
				memcpy(pframe, &channel->latest_zoom_param, sizeof(struct cam_zoom_frame));
				spin_unlock_irqrestore(&channel->lastest_zoom_lock, flag);
			}
			break;
		}
	}

	return pframe;
}

static int camcore_pipeline_callback(enum cam_cb_type type, void *param, void *priv_data)
{
	int ret = 0;
	struct camera_frame *pframe = NULL;
	struct camera_module *module = NULL;
	struct channel_context *channel = NULL;
	struct cam_hw_info *hw = NULL;

	if (!param || !priv_data) {
		pr_err("fail to get valid param %p %p\n", param, priv_data);
		return -EFAULT;
	}

	module = (struct camera_module *)priv_data;
	hw = module->grp->hw_info;
	if (unlikely(type == CAM_CB_DCAM_DEV_ERR)) {
		pr_err("fail to fatal err may need recovery\n");
		if (hw->prj_id != QOGIRN6L)
			csi_api_reg_trace();
		if (*(uint32_t *)param) {
			pr_info("cam %d start recovery\n", module->idx);
			if (atomic_cmpxchg(&module->grp->recovery_state, CAM_RECOVERY_NONE, CAM_RECOVERY_RUNNING) == CAM_RECOVERY_NONE)
				complete(&module->grp->recovery_thrd.thread_com);
		}
		return 0;
	}

	if (unlikely(type == CAM_CB_ISP_DEV_ERR)) {
		pr_err("fail to fatal err may not do anything\n");
		return 0;
	}

	pframe = (struct camera_frame *)param;
	channel = &module->channel[pframe->channel_id];

	switch (type) {
	case CAM_CB_FRAME_CACHE_CLEAR_BUF:
		camcore_dcam_online_buf_cfg(channel, pframe, module);
		break;
	case CAM_CB_DCAM_DATA_DONE:
	case CAM_CB_DUMP_DATA_DONE:
	case CAM_CB_DCAM_CLEAR_BUF:
		if (atomic_read(&module->state) != CAM_RUNNING) {
			pr_info("stream off cmd %d put frame %px, state:%d\n", type, pframe, module->state);
			if (pframe->buf.type == CAM_BUF_KERNEL)
				cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
			else
				cam_queue_enqueue(&module->put_queue, &pframe->list);
			return ret;
		}

		pframe->evt = IMG_TX_DONE;
		if (pframe->irq_type == CAMERA_IRQ_4IN1_DONE)
			pframe->channel_id = CAM_CH_RAW;
		if (pframe->irq_type != CAMERA_IRQ_4IN1_DONE)
			pframe->irq_type = CAMERA_IRQ_IMG;
		if (pframe->irq_property == CAM_FRAME_ORIGINAL_RAW)
			pframe->irq_type = CAMERA_IRQ_RAW_IMG;
		if (pframe->irq_property == CAM_FRAME_PROCESS_RAW)
			pframe->irq_type = CAMERA_IRQ_RAW_BPC_IMG;
		pframe->priv_data = module;
		ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
		if (ret) {
			pr_err("fail to frm_queue overflow\n");
			cam_queue_enqueue(&module->put_queue, &pframe->list);
		} else
			complete(&module->frm_com);
		break;
	case CAM_CB_DCAM_STATIS_DONE:
	case CAM_CB_ISP_STATIS_DONE:
	case CAM_CB_DUMP_STATIS_DONE:
		pframe->evt = IMG_TX_DONE;
		pframe->irq_type = CAMERA_IRQ_STATIS;

		if ((pframe->irq_property != STATIS_PARAM) && (module->flash_skip_fid != 0))
			pframe->is_flash_status = module->is_flash_status;

		pr_debug("pframe->fid %d is_flash_status %d irq_property %d\n",pframe->fid, pframe->is_flash_status, pframe->irq_property);
		if (atomic_read(&module->state) == CAM_RUNNING) {
			pframe->priv_data = module;
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				pr_err("fail to frm_queue overflow\n");
				cam_queue_enqueue(&module->put_queue, &pframe->list);
			} else
				complete(&module->frm_com);
		} else {
			cam_queue_enqueue(&module->put_queue, &pframe->list);
		}
		break;
	case CAM_CB_DCAM_IRQ_EVENT:
		if (pframe->irq_property == IRQ_DCAM_SN_EOF) {
			cam_queue_empty_frame_put(pframe);
			break;
		}

		if (pframe->irq_property == IRQ_DCAM_SOF) {
			if ((module->flash_info.led0_ctrl && module->flash_info.led0_status < FLASH_STATUS_MAX) ||
				(module->flash_info.led1_ctrl && module->flash_info.led1_status < FLASH_STATUS_MAX)) {
				module->flash_core_handle->flash_core_ops->start_flash(module->flash_core_handle,
					&module->flash_info.set_param);
				if (module->flash_info.flash_last_status != module->flash_info.led0_status) {
					module->flash_skip_fid = pframe->fid;
					module->is_flash_status = module->flash_info.led0_status;
				} else
					pr_info("do not need skip");
				pr_info("skip_fram=%d\n", pframe->fid);
				module->flash_info.flash_last_status = module->flash_info.led0_status;
				module->flash_info.led0_ctrl = 0;
				module->flash_info.led1_ctrl = 0;
				module->flash_info.led0_status = 0;
				module->flash_info.led1_status = 0;
			}
		}

		if (atomic_read(&module->state) == CAM_RUNNING) {
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
			}
		} else {
			cam_queue_empty_frame_put(pframe);
		}
		break;
	case CAM_CB_ISP_RET_SRC_BUF:
	case CAM_CB_PYRDEC_RET_SRC_BUF:
		if ((atomic_read(&module->state) != CAM_RUNNING) || (channel->dcam_port_id < 0)) {
			pr_info("isp ret src frame %p\n", pframe);
			pframe->not_use_isp_reserved_buf = 0;
			cam_queue_enqueue(&channel->share_buf_queue, &pframe->list);
		} else {
			if (pframe->buf.type == CAM_BUF_USER) {
				pr_info("dcam src buf return mfd %d\n", pframe->buf.mfd);
				cam_buf_ionbuf_put(&pframe->buf);
				cam_queue_empty_frame_put(pframe);
			} else
				ret = camcore_dcam_online_buf_cfg(channel, pframe, module);
		}
		break;
	case CAM_CB_ISP_RET_DST_BUF:
		if (atomic_read(&module->state) == CAM_RUNNING) {
			pframe->priv_data = module;
			pframe->evt = IMG_TX_DONE;
			if (module->capture_type == CAM_CAPTURE_RAWPROC) {
				pr_info("raw proc return dst frame %px\n", pframe);
				cam_buf_ionbuf_put(&pframe->buf);
				pframe->irq_type = CAMERA_IRQ_DONE;
				pframe->irq_property = IRQ_RAW_PROC_DONE;
			} else {
				pframe->irq_type = CAMERA_IRQ_IMG;
			}
			ret = cam_queue_enqueue(&module->frm_queue, &pframe->list);
			if (ret) {
				cam_buf_ionbuf_put(&pframe->buf);
				cam_queue_empty_frame_put(pframe);
			} else {
				complete(&module->frm_com);
			}
		} else {
			cam_buf_ionbuf_put(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
		}
		break;
	case CAM_CB_DCAM_RET_SRC_BUF:
		pr_info("user buf return type:%d mfd %d\n", type, pframe->buf.mfd);
		cam_queue_enqueue(&module->put_queue, &pframe->list);
		break;
	case CAM_CB_ISP_SCALE_RET_ISP_BUF:
		pr_info("user buf return type:%d mfd %d\n", type, pframe->buf.mfd);
		cam_buf_ionbuf_put(&pframe->buf);
		cam_queue_empty_frame_put(pframe);
		break;
	default:
		pr_err("fail to get cb cmd: %d\n", type);
		break;
	}

	return ret;
}

static int camcore_vir_channel_config(
	struct camera_module *module, struct channel_context *channel)
{
	int ret = 0;
	struct channel_context *channel_prev = NULL;
	struct channel_context *channel_cap = NULL;
	struct camera_uchannel *ch_uinfo = NULL;
	struct isp_path_base_desc path_desc = {0};

	ch_uinfo = &channel->ch_uinfo;
	channel_prev = &module->channel[CAM_CH_PRE];
	channel_cap = &module->channel[CAM_CH_CAP];

	if (channel_prev->enable) {
		/*TODO: out_fmt need to follow hal fmt adjust*/
		path_desc.out_fmt = CAM_YUV420_2FRAME_MIPI;
		path_desc.endian = ENDIAN_LITTLE;
		path_desc.output_size.w = ch_uinfo->vir_channel[0].dst_size.w;
		path_desc.output_size.h = ch_uinfo->vir_channel[0].dst_size.h;
		path_desc.is_work = 1;
		ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(channel_prev, PORT_VID_OUT, CAM_PIPELINE_CFG_BASE, &path_desc);
		if (ret)
			pr_err("fail to cfg isp pre base.\n");
	}
	if (channel_cap->enable) {
		/*TODO: out_fmt need to follow hal fmt adjust*/
		path_desc.out_fmt = CAM_YUV420_2FRAME_MIPI;
		path_desc.endian = ENDIAN_LITTLE;
		path_desc.output_size.w = ch_uinfo->vir_channel[1].dst_size.w;
		path_desc.output_size.h = ch_uinfo->vir_channel[1].dst_size.h;
		path_desc.is_work = 1;
		ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(channel_cap, PORT_VID_OUT, CAM_PIPELINE_CFG_BASE, &path_desc);
		if (ret)
			pr_err("fail to cfg isp cap base.\n");
	}
	return 0;
}

static int camcore_dcamoffline_bpcraw_cfg(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	uint32_t dcam_path_id = 0;
	struct dcam_path_cfg_param ch_desc = {0};

	dcam_path_id = module->grp->hw_info->ip_dcam[0]->dcam_raw_path_id;
	ch_desc.input_size.w = channel->ch_uinfo.src_size.w;
	ch_desc.input_size.h = channel->ch_uinfo.src_size.h;
	ch_desc.input_trim.size_x = channel->ch_uinfo.src_size.w;
	ch_desc.input_trim.size_y = channel->ch_uinfo.src_size.h;
	ch_desc.output_size.w = ch_desc.input_trim.size_x;
	ch_desc.output_size.h = ch_desc.input_trim.size_y;
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(channel, dcamoffline_pathid_convert_to_portid(dcam_path_id),
		CAM_PIPELINE_CFG_SIZE, &ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW);
	return ret;
}

static int camcore_pyrdec_desc_get(struct camera_module *module,
		struct channel_context *channel, struct pyr_dec_node_desc *pyr_dec_desc)
{
	int ret = 0;
	uint32_t format = 0, temp_format = 0;
	struct cam_hw_info *hw = NULL;

	if (!module || !channel || !pyr_dec_desc) {
		pr_err("fail to get valid inptr %p %p %p\n", module, channel, pyr_dec_desc);
		return -EINVAL;
	}

	hw = module->grp->hw_info;
	format = module->cam_uinfo.sensor_if.img_fmt;
	pyr_dec_desc->hw = hw;
	pyr_dec_desc->layer_num = PYR_DEC_LAYER_NUM;
	pyr_dec_desc->node_idx = module->idx;
	pyr_dec_desc->dcam_slice_mode = module->cam_uinfo.dcam_slice_mode;
	pyr_dec_desc->is_4in1 = module->cam_uinfo.is_4in1;
	pyr_dec_desc->sn_size = module->cam_uinfo.sn_size;
	pyr_dec_desc->dev = module->isp_dev_handle;
	pyr_dec_desc->pyrdec_dev = module->isp_dev_handle->pyr_dec_handle;
	pyr_dec_desc->buf_cb_func = pyr_dec_node_outbuf_get;

	if (format == DCAM_CAP_MODE_YUV)
		pyr_dec_desc->in_fmt = cam_format_get(channel->ch_uinfo.dst_fmt);
	else
		pyr_dec_desc->in_fmt = channel->dcam_out_fmt;

	pyr_dec_desc->pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;
	/*NEED check if necessary*/
	temp_format = channel->ch_uinfo.dcam_raw_fmt;
	if (cam_block_valid_fmt_get(&channel->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0])) {
		if (hw->ip_dcam[0]->save_band_for_bigsize)
			temp_format = CAM_RAW_PACK_10;
	}
	if ((hw->ip_isp->fetch_raw_support == 0) || (format == DCAM_CAP_MODE_YUV))
		channel->ch_uinfo.dcam_out_fmt = pyr_dec_desc->in_fmt;
	else {
		channel->ch_uinfo.dcam_out_fmt = temp_format;
		pyr_dec_desc->in_fmt = temp_format;
	}
	return ret;
}

static int camcore_pyrdec_ctxid_cfg(struct channel_context *channel, void *isp_node)
{
	int ret = 0;
	ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(channel, CAM_PIPELINE_CFG_CTXID, isp_node);
	return ret;
}

static int camcore_framecache_desc_get(struct camera_module *module, struct frame_cache_node_desc *frame_cache_desc)
{
	int ret = 0;
	uint32_t real_cache_num = 0;

	real_cache_num = module->cam_uinfo.zsl_num;
	if (module->cam_uinfo.is_dual)
		real_cache_num = 3;
	frame_cache_desc->cache_real_num = real_cache_num;
	frame_cache_desc->cache_skip_num = module->cam_uinfo.zsk_skip_num;
	frame_cache_desc->is_share_buf = module->cam_uinfo.need_share_buf;
	frame_cache_desc->need_dual_sync = module->cam_uinfo.is_dual;
	frame_cache_desc->dual_sync_func = camcore_dual_frame_deal;
	frame_cache_desc->dual_slave_frame_set = camcore_dual_slave_frame_set;
	frame_cache_desc->dual_sync_cb_data = module;

	return ret;
}

static int camcore_pipeline_init(struct camera_module *module,
		struct channel_context *channel)
{
	int ret = 0;
	int dcam_port_id = 0, isp_port_id = 0;
	uint32_t pipeline_type = 0, pyrdec_support = 0;
	struct channel_context *channel_prev = NULL;
	struct channel_context *channel_cap = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_pipeline_desc pipeline_desc = {0};
	struct dcam_online_node_desc *dcam_online_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_bpcraw_desc = NULL;
	struct isp_node_desc *isp_node_description = NULL;
	struct frame_cache_node_desc *frame_cache_desc = NULL;
	struct pyr_dec_node_desc *pyr_dec_desc = NULL;
	struct isp_yuv_scaler_node_desc *isp_yuv_scaler_desc = NULL;
	struct dcam_fetch_node_desc *dcam_fetch_desc = NULL;

	hw = module->grp->hw_info;
	pyrdec_support = hw->ip_isp->pyr_dec_support;
	channel_prev = &module->channel[CAM_CH_PRE];
	channel->ch_uinfo.src_size.w = module->cam_uinfo.sn_rect.w;
	channel->ch_uinfo.src_size.h = module->cam_uinfo.sn_rect.h;

	switch (channel->ch_id) {
	case CAM_CH_PRE:
		if (module->cam_uinfo.sensor_if.img_fmt == DCAM_CAP_MODE_YUV)
			dcam_port_id = PORT_FULL_OUT;
		else
			dcam_port_id = PORT_BIN_OUT;

		isp_port_id = PORT_PRE_OUT;
		pipeline_type = CAM_PIPELINE_PREVIEW;
		break;
	case CAM_CH_VID:
		if (channel_prev->enable) {
			channel->dcam_port_id = channel_prev->dcam_port_id;
		} else {
			dcam_port_id = PORT_BIN_OUT;
			pr_info("vid channel enable without preview\n");
		}
		isp_port_id = PORT_VID_OUT;
		pipeline_type = CAM_PIPELINE_VIDEO;
		break;
	case CAM_CH_CAP:
		dcam_port_id = PORT_FULL_OUT;
		isp_port_id = PORT_CAP_OUT;
		pipeline_type = CAM_PIPELINE_CAPTURE;
		if (module->cam_uinfo.zsl_num != 0 || module->cam_uinfo.is_dual)
			pipeline_type = CAM_PIPELINE_ZSL_CAPTURE;
		if (module->cam_uinfo.is_4in1) {
			pipeline_type = CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV;
			dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);
			module->auto_3dnr = channel->uinfo_3dnr = 0;
		}
		if (module->cam_uinfo.is_raw_alg) {
			if (module->cam_uinfo.raw_alg_type == RAW_ALG_AI_SFNR) {
				pipeline_type = CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV;
				if (module->cam_uinfo.zsl_num != 0)
					pipeline_type = CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV;
				if (module->cam_uinfo.param_frame_sync) {
					dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);
					pipeline_type = CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV;
				}
			} else {
				dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);
				pipeline_type = CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV;
			}
		}
		if (module->cam_uinfo.dcam_slice_mode && !module->cam_uinfo.is_4in1) {
			pipeline_type = CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV;
			dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);
			module->auto_3dnr = channel->uinfo_3dnr = 0;
		}
		break;
	case CAM_CH_RAW:
		if (( module->grp->hw_info->prj_id == SHARKL5pro &&
			module->cam_uinfo.sn_rect.w >= DCAM_HW_SLICE_WIDTH_MAX)
			|| module->raw_callback)
			dcam_port_id = PORT_VCH2_OUT;
		else
			dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);

		channel_cap = &module->channel[CAM_CH_CAP];
		module->cam_uinfo.is_raw_alg = 0;
		module->cam_uinfo.raw_alg_type = 0;
		module->auto_3dnr = channel->uinfo_3dnr = 0;
		isp_port_id = -1;
		pipeline_type = CAM_PIPELINE_SENSOR_RAW;
		if (!channel_cap->enable)
			pr_info("ITS Only open preview pipeline,shoud open raw channel\n");
		else if (module->cam_uinfo.need_dcam_raw && module->grp->hw_info->ip_isp->fetch_raw_support)
			return 0;
		break;
	case CAM_CH_VIRTUAL:
		channel->isp_port_id = PORT_VID_OUT;
		camcore_vir_channel_config(module, channel);
		return 0;
		break;
	case CAM_CH_DCAM_VCH:
		dcam_port_id = PORT_VCH2_OUT;
		isp_port_id = -1;
		pipeline_type = CAM_PIPELINE_VCH_SENSOR_RAW;
		break;
	default:
		pr_err("fail to get channel id %d\n", channel->ch_id);
		return -EINVAL;
	}

	camcore_compression_cal(module);
	if ((hw->ip_isp->fetch_raw_support == 0 && channel->ch_id != CAM_CH_RAW)
		|| (module->cam_uinfo.sensor_if.img_fmt == DCAM_CAP_MODE_YUV))
		channel->dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;
	else
		channel->dcam_out_fmt = channel->ch_uinfo.dcam_raw_fmt;
	channel->isp_port_id = isp_port_id;
	channel->dcam_port_id = dcam_port_id;
	channel->aux_dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[DCAM_HW_CONTEXT_1]->aux_dcam_path);
	channel->aux_raw_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[DCAM_HW_CONTEXT_1]->dcam_raw_path_id);
	channel->ch_uinfo.pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;

	pipeline_desc.nodes_dev = &module->nodes_dev;
	pipeline_desc.zoom_cb_func = cam_zoom_param_get_callback;
	pipeline_desc.data_cb_func = camcore_pipeline_callback;
	pipeline_desc.data_cb_handle = module;
	pipeline_desc.pipeline_graph = &module->static_topology->pipeline_list[pipeline_type];

	if (cam_scene_pipeline_need_dcamonline(pipeline_type)) {
		dcam_online_desc = &pipeline_desc.dcam_online_desc;
		cam_scene_dcamonline_desc_get(module, channel, pipeline_type, dcam_online_desc);
		if (module->cam_uinfo.virtualsensor) {
			dcam_fetch_desc = &pipeline_desc.dcam_fetch_desc;
			dcam_fetch_desc->online_node_desc = dcam_online_desc;
			dcam_fetch_desc->virtualsensor = module->cam_uinfo.virtualsensor;
			if (dcam_fetch_desc->virtualsensor && channel->ch_id == CAM_CH_CAP)
				dcam_fetch_desc->virtualsensor_cap_en = 1;
		}
	}

	if (cam_scene_pipeline_need_dcamoffline(pipeline_type)) {
		dcam_offline_desc = &pipeline_desc.dcam_offline_desc;
		cam_scene_dcamoffline_desc_get(module, channel, pipeline_type, dcam_offline_desc);
	}

	if (cam_scene_pipeline_need_dcamoffline_bpcraw(pipeline_type)) {
		dcam_offline_bpcraw_desc = &pipeline_desc.dcam_offline_bpcraw_desc;
		cam_scene_dcamoffline_bpcraw_desc_get(module, channel, dcam_offline_bpcraw_desc);
	}

	if (cam_scene_pipeline_need_ispoffline(pipeline_type)) {
		isp_node_description = &pipeline_desc.isp_node_description;
		cam_scene_ispoffline_desc_get(module, channel, isp_node_description);
	}

	if (cam_scene_pipeline_need_framecache(pipeline_type)) {
		frame_cache_desc = &pipeline_desc.frame_cache_desc;
		camcore_framecache_desc_get(module, frame_cache_desc);
		channel->need_framecache = 1;
	}

	if (cam_scene_pipeline_need_pyrdec(pipeline_type, pyrdec_support)) {
		pyr_dec_desc = &pipeline_desc.pyr_dec_desc;
		camcore_pyrdec_desc_get(module, channel, pyr_dec_desc);
	}

	if (cam_scene_pipeline_need_yuv_scaler(pipeline_type)) {
		isp_yuv_scaler_desc = &pipeline_desc.isp_yuv_scaler_desc;
		isp_yuv_scaler_desc->hw_path_id = isp_port_id_switch(isp_port_id);
		cam_scene_isp_yuv_scaler_desc_get(module, channel, isp_yuv_scaler_desc);
	}

	if (module->cam_uinfo.need_share_buf && (channel->ch_id == CAM_CH_CAP)) {
		struct cam_buf_pool_id pool_id = {0};
		pool_id.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
		ret = cam_buf_manager_pool_reg(&pool_id, DCAM_OUT_BUF_Q_LEN);
		if(ret) {
			pr_err("fail to reg CAM_BUF_POOL_SHARE_FULL_PATH pool.\n");
			return ret;
		}
	}

	channel->pipeline_type = pipeline_type;
	channel->pipeline_handle = cam_pipeline_creat(&pipeline_desc);
	if (!channel->pipeline_handle) {
		pr_err("fail to get cam pipeline.\n");
		return -ENOMEM;
	}

	if (dcam_offline_bpcraw_desc)
		camcore_dcamoffline_bpcraw_cfg(module, channel);
	/*TEMP: config ispctxid to pyrdec node*/
	if (pyr_dec_desc && isp_node_description)
		camcore_pyrdec_ctxid_cfg(channel, isp_node_description->isp_node);

	pr_info("cam%d ch %d pipeline done. ret = %d pipeline_type %s\n", module->idx, channel->ch_id, ret, cam_pipeline_name_get(pipeline_type));
	return ret;
}

static void camcore_pipeline_deinit(struct camera_module *module,
		struct channel_context *channel)
{
	int j = 0, k = 0;
	struct cam_nodes_dev *nodes_dev = NULL;

	nodes_dev = &module->nodes_dev;
	cam_pipeline_destory(channel->pipeline_handle);

	nodes_dev->dcam_online_node_dev = NULL;
	nodes_dev->dcam_offline_node_dev = NULL;
	nodes_dev->dcam_offline_node_bpcraw_dev = NULL;
	nodes_dev->pyr_dec_node_dev = NULL;
	for (j = 0; j < ISP_YUV_SCALER_MAX_NODE_ID; j++)
		nodes_dev->isp_yuv_scaler_node_dev[j] = NULL;
	for (k = 0; k < ISP_NODE_MODE_MAX_ID; k++) {
		nodes_dev->isp_node_dev[k] = NULL;
		for (j = 0; j < PORT_ISP_IN_MAX; j++)
			nodes_dev->isp_in_port_dev[k][j] = NULL;
		for (j = 0; j < PORT_ISP_OUT_MAX; j++)
			nodes_dev->isp_out_port_dev[k][j] = NULL;
	}
	for (j = 0; j < PORT_DCAM_OUT_MAX; j++)
		nodes_dev->dcam_online_out_port_dev[j] = NULL;
	for (j = 0; j < PORT_DCAM_OUT_MAX; j++)
		nodes_dev->dcam_offline_out_port_dev[j] = NULL;
	for (j = 0; j < PORT_DCAM_OUT_MAX; j++)
		nodes_dev->dcam_offline_bpcraw_out_port_dev[j] = NULL;
	for (j = 0; j < PORT_ISP_YUV_SCALER_OUT_MAX; j++)
		nodes_dev->isp_scaler_out_port_dev[j] = NULL;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static void camcore_timer_callback(struct timer_list *t)
{
	struct camera_module *module = from_timer(module, t, cam_timer);
#else
static void camcore_timer_callback(unsigned long data)
{
	struct camera_module *module = (struct camera_module *)data;
#endif

	struct camera_frame *frame = NULL;
	int ret = 0;

	if (!module || atomic_read(&module->state) != CAM_RUNNING) {
		pr_err("fail to get valid module %p or error state\n", module);
		return;
	}

	if (atomic_read(&module->timeout_flag) == 1) {
		pr_err("fail to get frame data, CAM%d timeout.\n", module->idx);
		frame = cam_queue_empty_frame_get();
		if (module->capture_type == CAM_CAPTURE_RAWPROC) {
			module->capture_type = CAM_CAPTURE_RAWPROC_DONE;
			frame->evt = IMG_TX_DONE;
			frame->irq_type = CAMERA_IRQ_DONE;
			frame->irq_property = IRQ_RAW_PROC_TIMEOUT;
		} else {
			frame->evt = IMG_TIMEOUT;
			frame->irq_type = CAMERA_IRQ_IMG;
			frame->irq_property = IRQ_MAX_DONE;
		}
		ret = cam_queue_enqueue(&module->frm_queue, &frame->list);
		complete(&module->frm_com);
		if (ret)
			pr_err("fail to enqueue.\n");
	}
}

static void camcore_timer_init(struct timer_list *cam_timer,
		unsigned long data)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	timer_setup(cam_timer, camcore_timer_callback, 0);
#else
	setup_timer(cam_timer, camcore_timer_callback, data);
#endif
}

static int camcore_timer_start(struct timer_list *cam_timer,
		uint32_t time_val)
{
	int ret = 0;

	pr_debug("starting timer %ld\n", jiffies);
	ret = mod_timer(cam_timer, jiffies + msecs_to_jiffies(time_val));

	return ret;
}

static int camcore_raw_proc_done(struct camera_module *module)
{
	int ret = 0;
	struct channel_context *ch = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_nodes_dev *nodes_dev = NULL;
	struct dcam_statis_param statis_param = {0};

	pr_info("cam%d start\n", module->idx);
	module->capture_type = CAM_CAPTURE_STOP;
	atomic_set(&module->state, CAM_STREAM_OFF);
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;

	if (atomic_read(&module->timeout_flag) == 1)
		pr_err("fail to raw proc, timeout\n");

	nodes_dev = &module->nodes_dev;
	ch = &module->channel[CAM_CH_CAP];

	cam_kernel_timer_stop(&module->cam_timer);

	if (ch->enable) {
		statis_param.statis_cmd = DCAM_IOCTL_DEINIT_STATIS_Q;
		CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
	}

	ch->enable = 0;
	ch->dcam_port_id = -1;
	ch->isp_port_id = -1;
	ch->aux_dcam_port_id = -1;
	ch->ch_uinfo.dcam_raw_fmt = -1;
	ch->ch_uinfo.sensor_raw_fmt = -1;

	cam_queue_clear(&ch->share_buf_queue, struct camera_frame, list);
	module->cam_uinfo.dcam_slice_mode = CAM_SLICE_NONE;
	module->cam_uinfo.slice_num = 0;

	atomic_set(&module->state, CAM_IDLE);
	camcore_pipeline_deinit(module, ch);

	if (module->cam_uinfo.is_pyr_dec) {
		if (ch->pyr_dec_buf) {
			camcore_k_frame_put(ch->pyr_dec_buf);
			ch->pyr_dec_buf = NULL;
		}
		if (ch->pyr_rec_buf) {
			camcore_k_frame_put(ch->pyr_rec_buf);
			ch->pyr_rec_buf = NULL;
		}
	}

	pr_info("camera%d rawproc done.\n", module->idx);

	return ret;
}

static int camcore_raw_pre_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t pipeline_type = 0;
	uint32_t pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
	uint32_t pyrdec_support = 0;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct cam_pipeline_desc pipeline_desc = {0};
	struct dcam_offline_node_desc *dcam_offline_desc = NULL;
	struct isp_node_desc *isp_node_description = NULL;
	struct pyr_dec_node_desc *pyr_dec_desc = NULL;
	struct dcam_path_cfg_param ch_desc = {0};
	struct isp_size_desc size_cfg = {0};

	pr_info("cam%d in. module:%px\n", module->idx, module);

	module->capture_type = CAM_CAPTURE_RAWPROC;
	hw = module->grp->hw_info;
	pyrdec_support = hw->ip_isp->pyr_dec_support;
	ch = &module->channel[CAM_CH_CAP];
	ch->dcam_port_id = -1;
	ch->isp_port_id = PORT_CAP_OUT;
	ch->aux_dcam_port_id = -1;
	ch->ch_uinfo.dcam_raw_fmt = -1;
	ch->ch_uinfo.sensor_raw_fmt = -1;
	pipeline_type = CAM_PIPELINE_OFFLINE_RAW2YUV;
	dcam_offline_desc = &pipeline_desc.dcam_offline_desc;
	isp_node_description = &pipeline_desc.isp_node_description;
	pyr_dec_desc = &pipeline_desc.pyr_dec_desc;
	pipeline_desc.pipeline_graph = &module->static_topology->pipeline_list[pipeline_type];
	pipeline_desc.nodes_dev = &module->nodes_dev;
	pipeline_desc.data_cb_handle = module;
	pipeline_desc.data_cb_func = camcore_pipeline_callback;

	dcam_offline_desc->dev = module->dcam_dev_handle;
	if (module->grp->hw_info->prj_id == SHARKL3)
		dcam_offline_desc->dcam_idx = 0;
	else
		dcam_offline_desc->dcam_idx = module->dcam_idx;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->port_desc.src_sel = PROCESS_RAW_SRC_SEL;
	dcam_offline_desc->statis_en = 1;
	cam_block_valid_fmt_get(&module->raw_cap_fetch_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = module->raw_cap_fetch_fmt;
	cam_block_valid_fmt_get(&ch->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = ch->ch_uinfo.dcam_raw_fmt;
	if (module->cam_uinfo.dcam_slice_mode && hw->ip_dcam[0]->save_band_for_bigsize)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_PACK_10;
	if (hw->ip_isp->fetch_raw_support == 0)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;

	/* isp offline param desc */
	isp_node_description->in_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	isp_node_description->pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;
	isp_node_description->store_3dnr_fmt = hw->ip_dcam[0]->store_3dnr_fmt[0];
	isp_node_description->mode_3dnr = MODE_3DNR_OFF;
	isp_node_description->mode_ltm = MODE_LTM_OFF;
	isp_node_description->mode_gtm = MODE_GTM_OFF;
	isp_node_description->ch_id = ch->ch_id;
	isp_node_description->bayer_pattern = proc_info->src_pattern;
	isp_node_description->enable_slowmotion = 0;
	isp_node_description->is_high_fps = 0;
	isp_node_description->slw_state = CAM_SLOWMOTION_OFF;
	isp_node_description->cam_id = module->idx;
	isp_node_description->dev = module->isp_dev_handle;
	isp_node_description->port_desc.out_fmt = CAM_YVU420_2FRAME;
	isp_node_description->port_desc.endian = ENDIAN_LITTLE;
	isp_node_description->port_desc.output_size.w = proc_info->dst_size.width;
	isp_node_description->port_desc.output_size.h = proc_info->dst_size.height;
	isp_node_description->blkparam_node_num = 1;
	isp_node_description->port_desc.resbuf_get_cb = isp_node_description->resbuf_get_cb;
	isp_node_description->port_desc.resbuf_cb_data = isp_node_description->resbuf_cb_data;
	isp_node_description->port_desc.in_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	isp_node_description->port_desc.bayer_pattern = isp_node_description->bayer_pattern;
	isp_node_description->port_desc.pyr_out_fmt = isp_node_description->pyr_out_fmt;
	isp_node_description->port_desc.store_3dnr_fmt = isp_node_description->store_3dnr_fmt;
	isp_node_description->port_desc.sn_size = isp_node_description->sn_size;

	pyr_dec_desc->hw = hw;
	pyr_dec_desc->layer_num = PYR_DEC_LAYER_NUM;
	pyr_dec_desc->node_idx = module->idx;
	pyr_dec_desc->dcam_slice_mode = module->cam_uinfo.dcam_slice_mode;
	pyr_dec_desc->is_4in1 = module->cam_uinfo.is_4in1;
	pyr_dec_desc->is_rawcap = 1;
	pyr_dec_desc->sn_size = module->cam_uinfo.sn_size;
	pyr_dec_desc->dev = module->isp_dev_handle;
	pyr_dec_desc->pyrdec_dev = module->isp_dev_handle->pyr_dec_handle;
	pyr_dec_desc->buf_cb_func = pyr_dec_node_outbuf_get;
	pyr_dec_desc->in_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	pyr_dec_desc->pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;
	ch->pipeline_handle = cam_pipeline_creat(&pipeline_desc);
	if (!ch->pipeline_handle) {
		pr_err("fail to get cam pipeline.\n");
		return -ENOMEM;
	}
	ch->enable = 1;
	ch->dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[DCAM_HW_CONTEXT_0]->aux_dcam_path);

	/*TEMP: config ispctxid to pyrdec node*/
	if (cam_scene_pipeline_need_pyrdec(pipeline_type, pyrdec_support))
		camcore_pyrdec_ctxid_cfg(ch, isp_node_description->isp_node);

	ch_desc.input_size.w = proc_info->src_size.width;
	ch_desc.input_size.h = proc_info->src_size.height;
	ch_desc.input_trim.start_x = 0;
	ch_desc.input_trim.start_y = 0;
	ch_desc.input_trim.size_x = ch_desc.input_size.w;
	ch_desc.input_trim.size_y = ch_desc.input_size.h;
	ch->trim_dcam.size_x = ch_desc.input_size.w;
	ch->trim_dcam.size_y = ch_desc.input_size.h;
	ch_desc.output_size = ch_desc.input_size;
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, dcamoffline_pathid_convert_to_portid(hw->ip_dcam[DCAM_HW_CONTEXT_0]->aux_dcam_path),
		CAM_PIPELINE_CFG_SIZE, &ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE);

	size_cfg.size.w = proc_info->src_size.width;
	size_cfg.size.h = proc_info->src_size.height;
	size_cfg.trim.start_x = 0;
	size_cfg.trim.start_y = 0;
	size_cfg.trim.size_x = size_cfg.size.w;
	size_cfg.trim.size_y = size_cfg.size.h;
	ch->trim_isp.size_x = size_cfg.size.w;
	ch->trim_isp.size_y = size_cfg.size.h;
	ret = CAM_PIPEINE_ISP_IN_PORT_CFG(ch, PORT_ISP_OFFLINE_IN, CAM_PIPELINE_CFG_SIZE, &size_cfg);

	size_cfg.size.w = proc_info->dst_size.width;
	size_cfg.size.h = proc_info->dst_size.height;
	size_cfg.trim.start_x = 0;
	size_cfg.trim.start_y = 0;
	size_cfg.trim.size_x = proc_info->src_size.width;
	size_cfg.trim.size_y = proc_info->src_size.height;
	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_SIZE, &size_cfg);

	if (module->cam_uinfo.is_pyr_dec)
		CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_REC_LEYER_NUM, &pyr_layer_num);

	return ret;
}

static int camcore_raw_post_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t width = 0, height = 0, size = 0;
	uint32_t pack_bits = 0, dcam_out_bits = 0;
	struct channel_context *ch = NULL;
	struct camera_frame *src_frame = NULL;
	struct camera_frame *mid_frame = NULL;
	struct camera_frame *dst_frame = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_statis_param statis_param = {0};
	timespec cur_ts = {0};
	memset(&cur_ts, 0, sizeof(timespec));
	pr_info("start\n");

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	statis_param.statis_cmd = DCAM_IOCTL_INIT_STATIS_Q;
	CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);

	pr_info("src %d 0x%x, mid %d, 0x%x, dst %d, 0x%x\n",
		proc_info->fd_src, proc_info->src_offset,
		proc_info->fd_dst0, proc_info->dst0_offset,
		proc_info->fd_dst1, proc_info->dst1_offset);
	src_frame = cam_queue_empty_frame_get();
	src_frame->buf.type = CAM_BUF_USER;
	src_frame->link_from.node_type = CAM_NODE_TYPE_USER;
	src_frame->buf.mfd = proc_info->fd_src;
	src_frame->buf.offset[0] = proc_info->src_offset;
	src_frame->channel_id = ch->ch_id;
	src_frame->width = proc_info->src_size.width;
	src_frame->height = proc_info->src_size.height;
	src_frame->endian = proc_info->src_y_endian;
	src_frame->pattern = proc_info->src_pattern;
	ktime_get_ts(&cur_ts);
	src_frame->sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->boot_sensor_time = ktime_get_boottime();
	src_frame->buf.status = CAM_BUF_ALLOC;

	dst_frame = cam_queue_empty_frame_get();
	dst_frame->buf.type = CAM_BUF_USER;
	dst_frame->buf.mfd = proc_info->fd_dst1;
	dst_frame->buf.offset[0] = proc_info->dst1_offset;
	dst_frame->channel_id = ch->ch_id;
	dst_frame->img_fmt = ch->ch_uinfo.dst_fmt;
	dst_frame->sensor_time = src_frame->sensor_time;
	dst_frame->boot_sensor_time = src_frame->boot_sensor_time;
	dst_frame->buf.status = CAM_BUF_ALLOC;

	mid_frame = cam_queue_empty_frame_get();
	mid_frame->channel_id = ch->ch_id;
	mid_frame->buf.status = CAM_BUF_ALLOC;
	/* if user set this buffer, we use it for dcam output
	 * or else we will allocate one for it.
	 */
	if(ch->dcam_port_id == PORT_OFFLINE_FULL_OUT && module->cam_uinfo.is_4in1 == 1)
		pack_bits = CAM_RAW_PACK_10;
	else
		pack_bits = ch->ch_uinfo.dcam_raw_fmt;

	pr_info("dcam raw_proc_post pack_bits %d\n", pack_bits);
	if (proc_info->fd_dst0 > 0) {
		mid_frame->buf.type = CAM_BUF_USER;
		mid_frame->link_from.node_type = CAM_NODE_TYPE_USER;
		mid_frame->buf.mfd = proc_info->fd_dst0;
		mid_frame->buf.offset[0] = proc_info->dst0_offset;
	} else {
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;

		if (proc_info->src_format == IMG_PIX_FMT_GREY)
			size = cal_sprd_pitch(width, pack_bits) * height;
		else
			size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		ret = cam_buf_alloc(&mid_frame->buf, (size_t)size, module->iommu_enable);
		if (ret) {
			pr_err("fail to get mid_frame.\n");
			goto mid_fail;
		}
	}
	mid_frame->sensor_time = src_frame->sensor_time;
	mid_frame->boot_sensor_time = src_frame->boot_sensor_time;
	if (module->cam_uinfo.is_pyr_dec)
		mid_frame->need_pyr_dec = 1;

	cam_queue_frame_flag_reset(mid_frame);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF, mid_frame, CAM_NODE_TYPE_DCAM_OFFLINE);
	if (ret) {
		pr_err("fail to cfg dcam out buffer.\n");
		goto dcam_out_fail;
	}

	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, dst_frame);
	if (ret) {
		pr_err("fail to cfg isp out buffer.\n");
		goto dcam_out_fail;
	}

	pr_info("raw proc, src %px, mid %px, dst %px\n", src_frame, mid_frame, dst_frame);
	cam_queue_init(&ch->share_buf_queue, CAM_SHARED_BUF_NUM, camcore_k_frame_put);
	atomic_set(&module->state, CAM_RUNNING);

	if (module->cam_uinfo.is_pyr_dec) {
		/* dec out buf for raw capture */
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;
		dcam_out_bits = cam_data_bits(module->grp->hw_info->ip_dcam[0]->dcam_output_fmt[0]);
		size = dcam_if_cal_pyramid_size(width, height, dcam_out_bits, 1, 0, ISP_PYR_DEC_LAYER_NUM);
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		pframe->width = width;
		pframe->height = height;
		pframe->channel_id = ch->ch_id;
		pframe->data_src_dec = 1;
		ret = cam_buf_alloc(&pframe->buf, size, module->iommu_enable);
		if (ret) {
			pr_err("fail to alloc raw dec buf\n");
			cam_queue_empty_frame_put(pframe);
			goto dst_fail;
		}
		ch->pyr_dec_buf = pframe;
		pr_debug("pyr_dec size %d, buf %p, w %d h %d\n", size, pframe, width, height);
		ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPILINE_CFG_PYRDEC_BUF, ch->pyr_dec_buf);

		/* rec temp buf for raw capture */
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;
		width = isp_rec_layer0_width(width, ISP_PYR_DEC_LAYER_NUM);
		height = isp_rec_layer0_heigh(height, ISP_PYR_DEC_LAYER_NUM);
		dcam_out_bits = module->cam_uinfo.sensor_if.if_spec.mipi.bits_per_pxl;
		size = dcam_if_cal_pyramid_size(width, height, dcam_out_bits, 1, 1, ISP_PYR_DEC_LAYER_NUM - 1);
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		pframe = cam_queue_empty_frame_get();
		pframe->width = width;
		pframe->height = height;
		ret = cam_buf_alloc(&pframe->buf, size, module->iommu_enable);
		if (ret) {
			pr_err("fail to alloc rec buf\n");
			cam_queue_empty_frame_put(pframe);
			atomic_inc(&ch->err_status);
			goto dec_fail;
		}
		ch->pyr_rec_buf = pframe;
		pr_debug("pyr_rec w %d, h %d, buf %p\n", width, height, pframe);
		CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_REC_BUF, ch->pyr_rec_buf);
	}

	ret = camcore_frame_start_proc(module, src_frame, CAM_NODE_TYPE_DCAM_OFFLINE);
	if (ret) {
		pr_err("fail to start dcam/isp for raw proc\n");
		goto src_fail;

	}

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;

src_fail:
	if (ch->pyr_rec_buf) {
		if (ch->pyr_rec_buf->buf.dmabuf_p)
			cam_buf_free(&ch->pyr_rec_buf->buf);
		cam_queue_empty_frame_put(ch->pyr_rec_buf);
	}
dec_fail:
	if (ch->pyr_dec_buf) {
		if (ch->pyr_dec_buf->buf.dmabuf_p)
			cam_buf_free(&ch->pyr_dec_buf->buf);
		cam_queue_empty_frame_put(ch->pyr_dec_buf);
	}
dst_fail:
	cam_buf_ionbuf_put(&dst_frame->buf);
dcam_out_fail:
	if (mid_frame->buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&mid_frame->buf);
	else
		cam_buf_free(&mid_frame->buf);
mid_fail:
	cam_queue_empty_frame_put(mid_frame);
	cam_queue_empty_frame_put(dst_frame);
	cam_queue_empty_frame_put(src_frame);
	pr_err("fail to call post raw proc\n");

	return ret;
}

static int camcore_virtual_sensor_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	timespec cur_ts = {0};
	struct channel_context *ch = NULL;
	struct camera_frame *src_frame = NULL;

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_debug("cap channel is not enable \n");
		ch = &module->channel[CAM_CH_PRE];
		if (ch->enable == 0) {
			pr_err("fail to get channel enable state\n");
			return -EFAULT;
		}
	}

	src_frame = cam_queue_empty_frame_get();
	src_frame->buf.type = CAM_BUF_USER;
	src_frame->buf.mfd = proc_info->fd_src;
	src_frame->buf.offset[0] = proc_info->src_offset;
	src_frame->channel_id = ch->ch_id;
	src_frame->width = proc_info->src_size.width;
	src_frame->height = proc_info->src_size.height;
	src_frame->endian = proc_info->src_y_endian;
	src_frame->pattern = proc_info->src_pattern;
	src_frame->fid = module->simu_fid++;
	ktime_get_ts(&cur_ts);
	src_frame->sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->boot_sensor_time = ktime_get_boottime();
	ret = cam_buf_ionbuf_get(&src_frame->buf);
	if (ret)
		goto virtualsensor_src_fail;

	ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_BUF, src_frame);
	if (ret)
		goto virtualsensor_src_fail;

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;
virtualsensor_src_fail:
	cam_queue_empty_frame_put(src_frame);
	pr_err("fail to call virtual sensor raw proc\n");
	return ret;
}

static int camcore_csi_switch_disconnect(struct camera_module *module, uint32_t mode, uint32_t csi_connect_stat)
{
	int ret = 0, i = 0;
	struct channel_context *ch = NULL;
	uint32_t dcam_path_state = DCAM_PATH_PAUSE;
	struct dcam_csi_reset_param csi_p = {0};

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_warn("warning: module state: %d\n", atomic_read(&module->state));
		return 0;
	}

	ch = &module->channel[CAM_CH_PRE];
	if (!ch->pipeline_handle) {
		pr_err("fail to get pipeline handle %p\n", ch->pipeline_handle);
		return -1;
	}

	csi_p.mode = mode;
	csi_p.csi_connect_stat = csi_connect_stat;
	ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_RESET, &csi_p);

	for(i = 0; i < CAM_CH_MAX; i++) {
		if (module->channel[i].ch_id == CAM_CH_VIRTUAL)
			continue;
		if (module->channel[i].enable) {
			ret = CAM_PIPEINE_ISP_NODE_CFG(&module->channel[i], CAM_PIPRLINE_CFG_PARAM_Q_CLEAR, NULL);
			if (ret)
				pr_err("fail to recycle cam%d ch %d blk param node\n", module->idx, i);
		}
	}

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable) {
		if (module->cam_uinfo.need_share_buf)
			ret = CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(ch, PORT_FULL_OUT, CAM_PIPILINE_CFG_SHARE_BUF, &dcam_path_state);
		if (module->cam_uinfo.zsl_num != 0)
			ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_CLR_CACHE_BUF, NULL);
	}

	if (atomic_read(&module->timeout_flag) == 1)
		atomic_set(&module->timeout_flag, 0);

	return ret;
}

static int camcore_csi_switch_connect(struct camera_module *module, uint32_t mode, uint32_t csi_connect_stat)
{
	int ret = 0;
	struct cam_pipeline_cfg_param param = {0};
	struct channel_context *ch_pre = NULL;

	ch_pre = &module->channel[CAM_CH_PRE];
	param.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
	param.node_param.param = NULL;
	param.node_param.port_id = 0xffffffff;
	ret = ch_pre->pipeline_handle->ops.streamon(ch_pre->pipeline_handle, &param);
	if (ret < 0)
		pr_err("fail to start dcam dev, ret %d\n", ret);

	return ret;
}

static int camcore_shutoff_param_prepare(struct camera_module *module,
		struct cam_pipeline_shutoff_param *pipeline_shutoff)
{
	struct cam_hw_info *hw = NULL;
	struct cam_node_shutoff_ctrl *node_shutoff = NULL;
	uint32_t dcam_port_id = 0, shutoff_cnt = 0;

	hw = module->grp->hw_info;
	shutoff_cnt = atomic_read(&module->capture_frames_dcam);
	node_shutoff = &pipeline_shutoff->node_shutoff;

	if (module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode) {
		dcam_port_id = dcamonline_pathid_convert_to_portid(hw->ip_dcam[0]->dcam_raw_path_id);
		pipeline_shutoff->node_type = CAM_NODE_TYPE_DCAM_ONLINE;
		NODE_SHUTOFF_PARAM_INIT(pipeline_shutoff->node_shutoff);
		atomic_set(&node_shutoff->outport_shutoff[dcam_port_id].cap_cnt, shutoff_cnt);
		node_shutoff->outport_shutoff[dcam_port_id].port_id = dcam_port_id;
		node_shutoff->outport_shutoff[dcam_port_id].shutoff_scene = SHUTOFF_SINGLE_PORT_NONZSL;
		node_shutoff->outport_shutoff[dcam_port_id].shutoff_type = SHUTOFF_PAUSE;
		return 1;
	}

	if (module->capture_scene == CAPTURE_AI_SFNR && !module->cam_uinfo.param_frame_sync) {
		pipeline_shutoff->node_type = CAM_NODE_TYPE_DCAM_ONLINE;
		NODE_SHUTOFF_PARAM_INIT(pipeline_shutoff->node_shutoff);
		atomic_set(&node_shutoff->outport_shutoff[PORT_FULL_OUT].cap_cnt, shutoff_cnt + 1);
		node_shutoff->outport_shutoff[PORT_FULL_OUT].port_id = PORT_FULL_OUT;
		node_shutoff->outport_shutoff[PORT_FULL_OUT].shutoff_scene = SHUTOFF_MULTI_PORT_SWITCH;
		node_shutoff->outport_shutoff[PORT_FULL_OUT].shutoff_type = SHUTOFF_PAUSE;
		atomic_set(&node_shutoff->outport_shutoff[PORT_RAW_OUT].cap_cnt, shutoff_cnt + 1);
		node_shutoff->outport_shutoff[PORT_RAW_OUT].port_id = PORT_RAW_OUT;
		node_shutoff->outport_shutoff[PORT_RAW_OUT].shutoff_scene = SHUTOFF_MULTI_PORT_SWITCH;
		node_shutoff->outport_shutoff[PORT_RAW_OUT].shutoff_type = SHUTOFF_RESUME;
		return 1;
	}

	return 0;
}

static int camcore_module_init(struct camera_module *module)
{
	int ch = 0;
	struct channel_context *channel = NULL;

	pr_info("sprd_img: camera dev %d init start!\n", module->idx);

	atomic_set(&module->state, CAM_INIT);
	mutex_init(&module->ioctl_lock);
	mutex_init(&module->zoom_lock);
	init_completion(&module->frm_com);

	module->capture_type = CAM_CAPTURE_STOP;
	module->raw_cap_fetch_fmt = CAM_FORMAT_MAX;
	module->attach_sensor_id = SPRD_SENSOR_ID_MAX + 1;
	module->flash_core_handle = get_cam_flash_handle(module->idx);

	for (ch = 0; ch < CAM_CH_MAX; ch++) {
		channel = &module->channel[ch];
		channel->ch_id = ch;
		channel->dcam_port_id = -1;
		channel->isp_port_id = -1;
		init_dcam_pm(&channel->blk_pm);
		channel->blk_pm.idx = DCAM_HW_CONTEXT_MAX;
		mutex_init(&module->buf_lock[ch]);
		init_completion(&channel->alloc_com);
		init_completion(&channel->stream_on_buf_com);
		init_completion(&channel->fast_stop);
		spin_lock_init(&channel->lastest_zoom_lock);
	}

	camcore_timer_init(&module->cam_timer, (unsigned long)module);
	cam_queue_init(&module->put_queue, CAM_FRAME_Q_LEN, camcore_ion_frame_put);
	cam_queue_init(&module->frm_queue, CAM_FRAME_Q_LEN, camcore_empty_frame_put);
	cam_queue_init(&module->alloc_queue, CAM_ALLOC_Q_LEN, camcore_empty_frame_put);

	pr_info("module[%d] init OK %px!\n", module->idx, module);

	return 0;
}

static int camcore_module_deinit(struct camera_module *module)
{
	int ch = 0;
	struct channel_context *channel = NULL;

	put_cam_flash_handle(module->flash_core_handle);
	cam_queue_clear(&module->put_queue, struct camera_frame, list);
	cam_queue_clear(&module->frm_queue, struct camera_frame, list);
	cam_queue_clear(&module->alloc_queue, struct camera_frame, list);

	for (ch = 0; ch < CAM_CH_MAX; ch++) {
		channel = &module->channel[ch];
		mutex_destroy(&module->buf_lock[channel->ch_id]);
	}
	mutex_destroy(&module->zoom_lock);
	mutex_destroy(&module->ioctl_lock);
	return 0;
}

#define CAM_IOCTL_LAYER
#include "cam_ioctl.c"
#undef CAM_IOCTL_LAYER

static struct cam_ioctl_cmd ioctl_cmds_table[] = {
	[_IOC_NR(SPRD_IMG_IO_SET_MODE)]             = {SPRD_IMG_IO_SET_MODE,             camioctl_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_CAP_SKIP_NUM)]     = {SPRD_IMG_IO_SET_CAP_SKIP_NUM,     camioctl_cap_skip_num_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_SIZE)]      = {SPRD_IMG_IO_SET_SENSOR_SIZE,      camioctl_sensor_size_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_TRIM)]      = {SPRD_IMG_IO_SET_SENSOR_TRIM,      camioctl_sensor_trim_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FRM_ID_BASE)]      = {SPRD_IMG_IO_SET_FRM_ID_BASE,      camioctl_frame_id_base_set},
	[_IOC_NR(SPRD_IMG_IO_SET_CROP)]             = {SPRD_IMG_IO_SET_CROP,             camioctl_crop_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FLASH)]            = {SPRD_IMG_IO_SET_FLASH,            camioctl_flash_set},
	[_IOC_NR(SPRD_IMG_IO_SET_OUTPUT_SIZE)]      = {SPRD_IMG_IO_SET_OUTPUT_SIZE,      camioctl_output_size_set},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_IF)]        = {SPRD_IMG_IO_SET_SENSOR_IF,        camioctl_sensor_if_set},
	[_IOC_NR(SPRD_IMG_IO_SET_FRAME_ADDR)]       = {SPRD_IMG_IO_SET_FRAME_ADDR,       camioctl_frame_addr_set},
	[_IOC_NR(SPRD_IMG_IO_PATH_FRM_DECI)]        = {SPRD_IMG_IO_PATH_FRM_DECI,        camioctl_frm_deci_set},
	[_IOC_NR(SPRD_IMG_IO_PATH_PAUSE)]           = {SPRD_IMG_IO_PATH_PAUSE,           camioctl_path_pause},
	[_IOC_NR(SPRD_IMG_IO_PATH_RESUME)]          = {SPRD_IMG_IO_PATH_RESUME,          camioctl_path_resume},
	[_IOC_NR(SPRD_IMG_IO_STREAM_ON)]            = {SPRD_IMG_IO_STREAM_ON,            camioctl_stream_on},
	[_IOC_NR(SPRD_IMG_IO_STREAM_OFF)]           = {SPRD_IMG_IO_STREAM_OFF,           camioctl_stream_off},
	[_IOC_NR(SPRD_IMG_IO_STREAM_PAUSE)]         = {SPRD_IMG_IO_STREAM_PAUSE,         camioctl_stream_pause},
	[_IOC_NR(SPRD_IMG_IO_STREAM_RESUME)]        = {SPRD_IMG_IO_STREAM_RESUME,        camioctl_stream_resume},
	[_IOC_NR(SPRD_IMG_IO_GET_FMT)]              = {SPRD_IMG_IO_GET_FMT,              camioctl_fmt_get},
	[_IOC_NR(SPRD_IMG_IO_GET_CH_ID)]            = {SPRD_IMG_IO_GET_CH_ID,            camioctl_ch_id_get},
	[_IOC_NR(SPRD_IMG_IO_GET_TIME)]             = {SPRD_IMG_IO_GET_TIME,             camioctl_time_get},
	[_IOC_NR(SPRD_IMG_IO_CHECK_FMT)]            = {SPRD_IMG_IO_CHECK_FMT,            camioctl_fmt_check},
	[_IOC_NR(SPRD_IMG_IO_SET_SHRINK)]           = {SPRD_IMG_IO_SET_SHRINK,           camioctl_shrink_set},
	[_IOC_NR(SPRD_IMG_IO_CFG_FLASH)]            = {SPRD_IMG_IO_CFG_FLASH,            camioctl_flash_cfg},
	[_IOC_NR(SPRD_IMG_IO_GET_IOMMU_STATUS)]     = {SPRD_IMG_IO_GET_IOMMU_STATUS,     camioctl_iommu_status_get},
	[_IOC_NR(SPRD_IMG_IO_START_CAPTURE)]        = {SPRD_IMG_IO_START_CAPTURE,        camioctl_capture_start},
	[_IOC_NR(SPRD_IMG_IO_STOP_CAPTURE)]         = {SPRD_IMG_IO_STOP_CAPTURE,         camioctl_capture_stop},
	[_IOC_NR(SPRD_IMG_IO_DCAM_PATH_SIZE)]       = {SPRD_IMG_IO_DCAM_PATH_SIZE,       camioctl_dcam_path_size},
	[_IOC_NR(SPRD_IMG_IO_SET_SENSOR_MAX_SIZE)]  = {SPRD_IMG_IO_SET_SENSOR_MAX_SIZE,  camioctl_sensor_max_size_set},
	[_IOC_NR(SPRD_ISP_IO_SET_STATIS_BUF)]       = {SPRD_ISP_IO_SET_STATIS_BUF,       camioctl_statis_buf_set},
	[_IOC_NR(SPRD_ISP_IO_CFG_PARAM)]            = {SPRD_ISP_IO_CFG_PARAM,            camioctl_param_cfg},
	[_IOC_NR(SPRD_ISP_IO_RAW_CAP)]              = {SPRD_ISP_IO_RAW_CAP,              camioctl_raw_proc},
	[_IOC_NR(SPRD_IMG_IO_GET_DCAM_RES)]         = {SPRD_IMG_IO_GET_DCAM_RES,         camioctl_cam_res_get},
	[_IOC_NR(SPRD_IMG_IO_PUT_DCAM_RES)]         = {SPRD_IMG_IO_PUT_DCAM_RES,         camioctl_cam_res_put},
	[_IOC_NR(SPRD_IMG_IO_SET_FUNCTION_MODE)]    = {SPRD_IMG_IO_SET_FUNCTION_MODE,    camioctl_function_mode_set},
	[_IOC_NR(SPRD_IMG_IO_GET_FLASH_INFO)]       = {SPRD_IMG_IO_GET_FLASH_INFO,       camioctl_flash_get},
	[_IOC_NR(SPRD_IMG_IO_EBD_CONTROL)]          = {SPRD_IMG_IO_EBD_CONTROL,          camioctl_ebd_control},
	[_IOC_NR(SPRD_IMG_IO_SET_4IN1_ADDR)]        = {SPRD_IMG_IO_SET_4IN1_ADDR,        camioctl_4in1_raw_addr_set},
	[_IOC_NR(SPRD_IMG_IO_4IN1_POST_PROC)]       = {SPRD_IMG_IO_4IN1_POST_PROC,       camioctl_4in1_post_proc},
	[_IOC_NR(SPRD_IMG_IO_SET_CAM_SECURITY)]     = {SPRD_IMG_IO_SET_CAM_SECURITY,     camioctl_cam_security_set},
	[_IOC_NR(SPRD_IMG_IO_GET_PATH_RECT)]        = {SPRD_IMG_IO_GET_PATH_RECT,        camioctl_path_rect_get},
	[_IOC_NR(SPRD_IMG_IO_SET_3DNR_MODE)]        = {SPRD_IMG_IO_SET_3DNR_MODE,        camioctl_3dnr_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_AUTO_3DNR_MODE)]   = {SPRD_IMG_IO_SET_AUTO_3DNR_MODE,   camioctl_auto_3dnr_mode_set},
	[_IOC_NR(SPRD_IMG_IO_CAPABILITY)]           = {SPRD_IMG_IO_CAPABILITY,           camioctl_capability_get},
	[_IOC_NR(SPRD_IMG_IO_POST_FDR)]             = {SPRD_IMG_IO_POST_FDR,             camioctl_cam_post_proc},
	[_IOC_NR(SPRD_IMG_IO_CAM_TEST)]             = {SPRD_IMG_IO_CAM_TEST,             camioctl_cam_test},
	[_IOC_NR(SPRD_IMG_IO_DCAM_SWITCH)]          = {SPRD_IMG_IO_DCAM_SWITCH,          camioctl_csi_switch},
	[_IOC_NR(SPRD_IMG_IO_GET_SCALER_CAP)]       = {SPRD_IMG_IO_GET_SCALER_CAP,       camioctl_scaler_capability_get},
	[_IOC_NR(SPRD_IMG_IO_GET_DWARP_HW_CAP)]     = {SPRD_IMG_IO_GET_DWARP_HW_CAP,     camioctl_dewarp_hw_capability_get},
	[_IOC_NR(SPRD_IMG_IO_SET_DWARP_OTP)]        = {SPRD_IMG_IO_SET_DWARP_OTP,        camioctl_dewarp_otp_set},
	[_IOC_NR(SPRD_IMG_IO_SET_LONGEXP_CAP)]      = {SPRD_IMG_IO_SET_LONGEXP_CAP,      camioctl_longexp_mode_set},
	[_IOC_NR(SPRD_IMG_IO_SET_MUL_MAX_SN_SIZE)]  = {SPRD_IMG_IO_SET_MUL_MAX_SN_SIZE,  camioctl_mul_max_sensor_size_set},
	[_IOC_NR(SPRD_IMG_IO_SET_CAP_ZSL_INFO)]     = {SPRD_IMG_IO_SET_CAP_ZSL_INFO,     camioctl_cap_zsl_info_set},
	[_IOC_NR(SPRD_IMG_IO_SET_DCAM_RAW_FMT)]     = {SPRD_IMG_IO_SET_DCAM_RAW_FMT,     camioctl_dcam_raw_fmt_set},
	[_IOC_NR(SPRD_IMG_IO_SET_KEY)]              = {SPRD_IMG_IO_SET_KEY,              camioctl_key_set},
	[_IOC_NR(SPRD_IMG_IO_SET_960FPS_PARAM)]     = {SPRD_IMG_IO_SET_960FPS_PARAM,     camioctl_960fps_param_set},
	[_IOC_NR(SPRD_IMG_IO_CFG_PARAM_STATUS)]     = {SPRD_IMG_IO_CFG_PARAM_STATUS,     camioctl_cfg_param_start_end},
};

static long camcore_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	struct camera_module *module = NULL;
	struct cam_ioctl_cmd *ioctl_cmd_p = NULL;
	int nr = _IOC_NR(cmd);

	pr_debug("cam ioctl, cmd:0x%x, cmdnum %d\n", cmd, nr);

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (unlikely(!(nr >= 0 && nr < ARRAY_SIZE(ioctl_cmds_table)))) {
		pr_info("invalid cmd: 0x%xn", cmd);
		return -EINVAL;
	}

	ioctl_cmd_p = &ioctl_cmds_table[nr];
	if (unlikely((ioctl_cmd_p->cmd != cmd) || (ioctl_cmd_p->cmd_proc == NULL))) {
		pr_debug("unsupported cmd_k: 0x%x, cmd_u: 0x%x, nr: %d\n",
			ioctl_cmd_p->cmd, cmd, nr);
		return 0;
	}

	mutex_lock(&module->ioctl_lock);
	/* ioctl from user may cfg hw register when start recovery, need lock
	 * this operatate to avoid bus hang issue when axi reset in recovery.
	*/
	down_read(&module->grp->switch_recovery_lock);
	if (cmd == SPRD_IMG_IO_SET_KEY || module->private_key == 1) {
		ret = ioctl_cmd_p->cmd_proc(module, arg);
		if (ret) {
			pr_debug("fail to ioctl cmd:%x, nr:%d, func %ps\n",
				cmd, nr, ioctl_cmd_p->cmd_proc);
			goto exit;
		}
	} else
		pr_err("cam %d fail to get ioctl permission %d\n", module->idx, module->private_key);

	pr_debug("cam id:%d, %ps, done!\n", module->idx, ioctl_cmd_p->cmd_proc);
exit:
	up_read(&module->grp->switch_recovery_lock);
	mutex_unlock(&module->ioctl_lock);

	return ret;
}

#ifdef CONFIG_COMPAT
static long camcore_ioctl_compat(struct file *file,
	unsigned int cmd, unsigned long arg)
{

	long ret = 0L;
	struct camera_module *module = NULL;
	void __user *data32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	module->compat_flag = 1;
	pr_debug("cmd [0x%x][%d]\n", cmd, _IOC_NR(cmd));

	switch (cmd) {
	case COMPAT_SPRD_ISP_IO_CFG_PARAM:
		ret = file->f_op->unlocked_ioctl(file, SPRD_ISP_IO_CFG_PARAM,
			(unsigned long)data32);
		break;
	default:
		ret = file->f_op->unlocked_ioctl(file, cmd, (unsigned long)data32);
		break;
	}
	return ret;
}
#endif

static int camcore_recovery_proc(void *param)
{
	int ret = 0, i = 0, j = 0, recovery_id = 0;
	uint32_t timer = 0;
	struct camera_group *grp = NULL;
	struct camera_module *module = NULL;
	struct channel_context *ch = NULL;
	struct cam_hw_info *hw = NULL;
	uint32_t switch_mode = CAM_CSI_RECOVERY_SWITCH;
	struct cam_hw_lbuf_info cam_lbuf_info = {0};
	struct cam_pipeline_cfg_param pipe_param = {0};
	struct dcam_online_start_param start_param = {0};
	struct cam_pipeline_cfg_param cfg_param = {0};

	grp = (struct camera_group *)param;
	hw = grp->hw_info;

	down_write(&grp->switch_recovery_lock);
	/* all avaiable dcam csi switch disconnect & stop */
	for (i = 0; i < CAM_COUNT; i++) {
		module = grp->module[i];
		if (module && (atomic_read(&module->state) == CAM_RUNNING)){
			if (!module->dcam_ctx_bind_state) {
				pr_warn("warning: module %d has been disconnected and unbinded already\n", module->idx);
				continue;
			}
			for (j = 0; j < CAM_CH_MAX; j++) {
				ch = &module->channel[j];
				if (ch->enable && ch->pipeline_handle) {
					enum dcam_stop_cmd stop_cmd = DCAM_RECOVERY;
					pipe_param.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
					pipe_param.node_param.param = &stop_cmd;
					ret = ch->pipeline_handle->ops.streamoff(ch->pipeline_handle, &pipe_param);
					if (ret)
						pr_err("fail to stop module %d dcam online ret:%d\n", module->idx, ret);
				}
			}
			pr_info("modules %d recovery disconnect\n", module->idx);
			camcore_csi_switch_disconnect(module, switch_mode, DCAM_CSI_PAUSE);
		}
	}

	recovery_id = 0;
	hw->dcam_ioctl(hw, DCAM_HW_CFG_ALL_RESET, &recovery_id);
	cam_buf_iommu_restore(CAM_IOMMUDEV_DCAM);

	/* all avaiable dcam reconfig & start */
	for (i = 0; i < CAM_COUNT; i++) {
		module = grp->module[i];
		if (module && (atomic_read(&module->state) == CAM_RUNNING)){
			if (!module->dcam_ctx_bind_state)
				continue;
			pr_info("modules %d recovery connect\n", module->idx);
			/* dcam online node stream on */
			for (j = 0; j < CAM_CH_MAX; j++) {
				ch = &module->channel[j];
				if (ch->enable && ch->pipeline_handle) {
					cam_lbuf_info.is_4in1 = module->cam_uinfo.is_4in1;
					cam_lbuf_info.line_w = module->cam_uinfo.sn_rect.w;
					cam_lbuf_info.is_offline = 0;
					start_param.lbuf_param = &cam_lbuf_info;
					cfg_param.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
					cfg_param.node_param.param = &start_param;
					cfg_param.node_param.port_id = 0xffffffff;
					ret = ch->pipeline_handle->ops.streamon(ch->pipeline_handle, &cfg_param);
				}
			}
			atomic_set(&module->timeout_flag, 1);
			if (module->cam_uinfo.is_longexp)
				timer = CAMERA_LONGEXP_TIMEOUT;
			else
				timer = CAMERA_TIMEOUT;
			camcore_timer_start(&module->cam_timer, timer);
		}
	}
	atomic_set(&grp->recovery_state, CAM_RECOVERY_NONE);
	up_write(&grp->switch_recovery_lock);
	pr_info("cam recovery is finish\n");

	return ret;
}

static ssize_t camcore_read(struct file *file, char __user *u_data,
		size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	int i = 0;
	int superzoom_val = 0;
	struct sprd_img_read_op read_op = {0};
	struct camera_module *module = NULL;
	struct camera_frame *pframe = NULL;
	struct channel_context *pchannel = NULL;
	struct sprd_img_path_capability *cap = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_reg_trace trace = {0};

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_read_op)) {
		pr_err("fail to img read, cnt %zd read_op %d\n", cnt,
			(int32_t)sizeof(struct sprd_img_read_op));
		return -EIO;
	}

	if (copy_from_user(&read_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	pr_debug("cam %d read cmd %d\n", module->idx, read_op.cmd);

	switch (read_op.cmd) {
	case SPRD_IMG_GET_SCALE_CAP:
		hw = module->grp->hw_info;
		for (i = 0; i < DCAM_ID_MAX; i++) {
			if (hw->ip_dcam[i]->superzoom_support) {
				superzoom_val = 1;
				break;
			}
		}

		if (superzoom_val)
			read_op.parm.reserved[1] = 10;
		else
			read_op.parm.reserved[1] = 4;

		read_op.parm.reserved[0] = 4672;
		read_op.parm.reserved[2] = 4672;
		pr_debug("line threshold %d, sc factor %d, scaling %d.\n",
			read_op.parm.reserved[0],
			read_op.parm.reserved[1],
			read_op.parm.reserved[2]);
		break;
	case SPRD_IMG_GET_FRM_BUFFER:
rewait:
		memset(&read_op, 0, sizeof(struct sprd_img_read_op));
		while (1) {
			ret = wait_for_completion_interruptible(&module->frm_com);
			if (ret == 0) {
				break;
			} else if (ret == -ERESTARTSYS) {
				read_op.evt = IMG_SYS_BUSY;
				ret = 0;
				goto read_end;
			} else {
				pr_err("read frame buf, fail to down, %d\n",
					ret);
				return -EPERM;
			}
		}

		while (1) {
			pframe = cam_queue_dequeue(&module->put_queue,
				struct camera_frame, list);
			if (pframe) {
				cam_buf_manager_buf_status_change(&pframe->buf, CAM_BUF_ALLOC, CAM_IOMMUDEV_MAX);
				cam_queue_empty_frame_put(pframe);
			} else {
				break;
			}
		}

		pchannel = NULL;
		pframe = cam_queue_dequeue(&module->frm_queue, struct camera_frame, list);
		if (!pframe) {
			/* any exception happens or user trigger exit. */
			pr_info("No valid frame buffer. tx stop.\n");
			read_op.evt = IMG_TX_STOP;
		} else if (pframe->evt == IMG_TX_DONE) {
			atomic_set(&module->timeout_flag, 0);
			if ((pframe->irq_type == CAMERA_IRQ_4IN1_DONE)
				|| (pframe->irq_type == CAMERA_IRQ_IMG)
				|| (pframe->irq_type == CAMERA_IRQ_RAW_IMG)
				|| (pframe->irq_type == CAMERA_IRQ_RAW_BPC_IMG)) {
				cam_buf_ionbuf_put(&pframe->buf);
				pchannel = &module->channel[pframe->channel_id];
				if (pframe->buf.mfd == module->reserved_buf_fd) {
					pr_info("get output buffer with reserved frame fd %d, ch %d\n",
						module->reserved_buf_fd, pchannel->ch_id);
					cam_queue_empty_frame_put(pframe);
					goto rewait;
				}
				read_op.parm.frame.channel_id = pframe->channel_id;
				read_op.parm.frame.index = pchannel->frm_base_id;
				read_op.parm.frame.frm_base_id = pchannel->frm_base_id;
				read_op.parm.frame.img_fmt = pframe->img_fmt;
			}
			read_op.evt = pframe->evt;
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.irq_property = pframe->irq_property;
			read_op.parm.frame.length = pframe->width;
			read_op.parm.frame.height = pframe->height;
			read_op.parm.frame.real_index = pframe->fid;
			read_op.parm.frame.frame_id = pframe->fid;
			read_op.parm.frame.sec = pframe->sensor_time.tv_sec;
			read_op.parm.frame.usec = pframe->sensor_time.tv_usec;
			read_op.parm.frame.monoboottime = pframe->boot_sensor_time;
			read_op.parm.frame.yaddr_vir = (uint32_t)pframe->buf.addr_vir[0];
			read_op.parm.frame.uaddr_vir = (uint32_t)pframe->buf.addr_vir[1];
			read_op.parm.frame.vaddr_vir = (uint32_t)pframe->buf.addr_vir[2];
			read_op.parm.frame.mfd = pframe->buf.mfd;
			read_op.parm.frame.yaddr = pframe->buf.offset[0];
			read_op.parm.frame.uaddr = pframe->buf.offset[1];
			read_op.parm.frame.vaddr = pframe->buf.offset[2];
			read_op.parm.frame.is_flash_status = pframe->is_flash_status;

			if (pframe->irq_type == CAMERA_IRQ_RAW_IMG) {
				pr_info("FDR %d ch %d, evt %d, fid %d, buf_fd %d,  time  %06d.%06d\n",
					pframe->irq_type,  read_op.parm.frame.channel_id, read_op.evt,
					read_op.parm.frame.real_index, read_op.parm.frame.mfd,
					read_op.parm.frame.sec, read_op.parm.frame.usec);
			}
			/* for statis buffer address below. */
			read_op.parm.frame.addr_offset = pframe->buf.offset[0];
			read_op.parm.frame.zoom_ratio = pframe->zoom_ratio;
			read_op.parm.frame.total_zoom = pframe->total_zoom;
		} else {
			struct cam_hw_info *hw = module->grp->hw_info;
			struct dcam_online_node *dcam_online_node_dev = NULL;

			if (hw == NULL) {
				pr_err("fail to get hw ops.\n");
				return -EFAULT;
			}
			pr_err("fail to get correct event %d\n", pframe->evt);
			csi_api_reg_trace();
			if (module->nodes_dev.dcam_online_node_dev) {
				trace.type = ABNORMAL_REG_TRACE;
				dcam_online_node_dev = module->nodes_dev.dcam_online_node_dev;
				if (dcam_online_node_dev->hw_ctx_id != DCAM_HW_CONTEXT_MAX) {
					trace.idx = dcam_online_node_dev->hw_ctx_id;
					hw->isp_ioctl(hw, ISP_HW_CFG_REG_TRACE, &trace);
				}
			}
			read_op.evt = pframe->evt;
			read_op.parm.frame.irq_type = pframe->irq_type;
			read_op.parm.frame.irq_property = pframe->irq_property;
		}
		pr_debug("cam%d read frame, evt 0x%x irq %d, irq_property %d, ch 0x%x index %d mfd 0x%x\n",
			module->idx, read_op.evt, read_op.parm.frame.irq_type, read_op.parm.frame.irq_property, read_op.parm.frame.channel_id,
			read_op.parm.frame.real_index, read_op.parm.frame.mfd);

		if (pframe) {
			if (pframe->irq_type != CAMERA_IRQ_4IN1_DONE) {
				cam_queue_empty_frame_put(pframe);
				break;
			}
			/* 4in1 report frame for remosaic
			 * save frame for 4in1_post IOCTL
			 */
			ret = cam_queue_enqueue(&module->remosaic_queue, &pframe->list);
			if (!ret)
				break;
			/* fail, give back */
			cam_queue_empty_frame_put(pframe);
			ret = 0;
		}
		break;
	case SPRD_IMG_GET_PATH_CAP:
		pr_debug("get path capbility\n");
		cap = &read_op.parm.capability;
		memset(cap, 0, sizeof(struct sprd_img_path_capability));
		cap->support_3dnr_mode = 1;
		cap->support_4in1 = 1;
		cap->count = 6;
		cap->path_info[CAM_CH_RAW].support_yuv = 0;
		cap->path_info[CAM_CH_RAW].support_raw = 1;
		cap->path_info[CAM_CH_RAW].support_jpeg = 0;
		cap->path_info[CAM_CH_RAW].support_scaling = 0;
		cap->path_info[CAM_CH_RAW].support_trim = 1;
		cap->path_info[CAM_CH_RAW].is_scaleing_path = 0;
		cap->path_info[CAM_CH_PRE].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_PRE].support_yuv = 1;
		cap->path_info[CAM_CH_PRE].support_raw = 0;
		cap->path_info[CAM_CH_PRE].support_jpeg = 0;
		cap->path_info[CAM_CH_PRE].support_scaling = 1;
		cap->path_info[CAM_CH_PRE].support_trim = 1;
		cap->path_info[CAM_CH_PRE].is_scaleing_path = 0;
		cap->path_info[CAM_CH_CAP].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_CAP].support_yuv = 1;
		cap->path_info[CAM_CH_CAP].support_raw = 0;
		cap->path_info[CAM_CH_CAP].support_jpeg = 0;
		cap->path_info[CAM_CH_CAP].support_scaling = 1;
		cap->path_info[CAM_CH_CAP].support_trim = 1;
		cap->path_info[CAM_CH_CAP].is_scaleing_path = 0;
		cap->path_info[CAM_CH_VID].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_VID].support_yuv = 1;
		cap->path_info[CAM_CH_VID].support_raw = 0;
		cap->path_info[CAM_CH_VID].support_jpeg = 0;
		cap->path_info[CAM_CH_VID].support_scaling = 1;
		cap->path_info[CAM_CH_VID].support_trim = 1;
		cap->path_info[CAM_CH_VID].is_scaleing_path = 0;
		cap->path_info[CAM_CH_PRE_THM].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_PRE_THM].support_yuv = 1;
		cap->path_info[CAM_CH_PRE_THM].support_raw = 0;
		cap->path_info[CAM_CH_PRE_THM].support_jpeg = 0;
		cap->path_info[CAM_CH_PRE_THM].support_scaling = 1;
		cap->path_info[CAM_CH_PRE_THM].support_trim = 1;
		cap->path_info[CAM_CH_PRE_THM].is_scaleing_path = 0;
		cap->path_info[CAM_CH_CAP_THM].line_buf = ISP_WIDTH_MAX;
		cap->path_info[CAM_CH_CAP_THM].support_yuv = 1;
		cap->path_info[CAM_CH_CAP_THM].support_raw = 0;
		cap->path_info[CAM_CH_CAP_THM].support_jpeg = 0;
		cap->path_info[CAM_CH_CAP_THM].support_scaling = 1;
		cap->path_info[CAM_CH_CAP_THM].support_trim = 1;
		cap->path_info[CAM_CH_CAP_THM].is_scaleing_path = 0;
		break;
	case SPRD_IMG_GET_DCAM_RAW_CAP:
		hw = module->grp->hw_info;
		for (i = 0; i < DCAM_RAW_MAX; i++)
			read_op.parm.reserved[i] = hw->ip_dcam[0]->raw_fmt_support[i];
		break;
	default:
		pr_err("fail to get valid cmd\n");
		return -EINVAL;
	}

read_end:
	if (copy_to_user((void __user *)u_data, &read_op, cnt))
		ret = -EFAULT;

	if (ret)
		cnt = ret;

	return cnt;
}

static ssize_t camcore_write(struct file *file, const char __user *u_data,
		size_t cnt, loff_t *cnt_ret)
{
	int ret = 0;
	struct sprd_img_write_op write_op = {0};
	struct camera_module *module = NULL;

	module = (struct camera_module *)file->private_data;
	if (!module) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	if (cnt != sizeof(struct sprd_img_write_op)) {
		pr_err("fail to write, cnt %zd  write_op %d\n", cnt,
				(uint32_t)sizeof(struct sprd_img_write_op));
		return -EIO;
	}

	if (copy_from_user(&write_op, (void __user *)u_data, cnt)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}

	switch (write_op.cmd) {
	case SPRD_IMG_STOP_DCAM:
		pr_info("user stop camera %d\n", module->idx);
		complete(&module->frm_com);
		break;
	default:
		pr_err("fail to get write cmd %d\n", write_op.cmd);
		break;
	}

	ret = copy_to_user((void __user *)u_data, &write_op, cnt);
	if (ret) {
		pr_err("fail to get user info\n");
		cnt = ret;
		return -EFAULT;
	}

	return cnt;
}

static int camcore_open(struct inode *node, struct file *file)
{
	int ret = 0;
	unsigned long flag = 0;
	struct camera_module *module = NULL;
	struct camera_group *grp = NULL;
	struct miscdevice *md = file->private_data;
	uint32_t i = 0, idx = 0, count = 0;
	struct cam_thread_info *thrd = NULL;

	grp = md->this_device->platform_data;
	count = grp->dcam_count;

	if (count == 0 || count > CAM_COUNT) {
		pr_err("fail to get valid dts configured dcam count\n");
		return -ENODEV;
	}

	if (g_mem_dbg && g_mem_dbg->g_dbg_memory_leak_ctrl && atomic_read(&grp->camera_opened) == 0)
		cam_buf_memory_leak_queue_init();

	if (atomic_inc_return(&grp->camera_opened) > count) {
		pr_err("fail to open camera, all %d cameras opened already.", count);
		atomic_dec(&grp->camera_opened);
		return -EMFILE;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	ret = sprd_glb_mm_pw_on_cfg();
	pm_runtime_get_sync(&grp->hw_info->pdev->dev);
#endif

	pr_info("sprd_img: the camera opened count %d, camsec_mode %d\n",
		atomic_read(&grp->camera_opened), grp->camsec_cfg.camsec_mode);

	spin_lock_irqsave(&grp->module_lock, flag);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	__pm_stay_awake(grp->ws);
#endif
	for (i = 0, idx = count; i < count; i++) {
		if ((grp->module_used & (1 << i)) == 0) {
			if (grp->module[i] != NULL) {
				pr_err("fail to get null module, un-release camera module:  %p, idx %d\n",
					grp->module[i], i);
				spin_unlock_irqrestore(&grp->module_lock, flag);
				ret = -EMFILE;
				goto exit;
			}
			idx = i;
			grp->module_used |= (1 << i);
			break;
		}
	}

	spin_unlock_irqrestore(&grp->module_lock, flag);

	if (idx == count) {
		pr_err("fail to get available camera module.\n");
		ret = -EMFILE;
		goto exit;
	}

	pr_debug("alloc. size of module %x, group %x\n",
		(int)sizeof(struct camera_module),
		(int)sizeof(struct camera_group));

	module = cam_buf_kernel_sys_vzalloc(sizeof(struct camera_module));
	if (!module) {
		pr_err("fail to alloc camera module %d\n", idx);
		ret = -ENOMEM;
		goto alloc_fail;
	}

	module->idx = idx;
	module->static_topology = &grp->topology_info;
	ret = camcore_module_init(module);
	if (ret) {
		pr_err("fail to init camera module %d\n", idx);
		ret = -ENOMEM;
		goto init_fail;
	}

	ret = cam_buf_manager_init(idx);
	if (ret < 0) {
		pr_err("fail to init buf_manager\n");
		goto buf_manager_fail;
	} else
		module->reserved_pool_id = ret;

	if (atomic_read(&grp->camera_opened) == 1) {
		/* should check all needed interface here. */
		spin_lock_irqsave(&grp->module_lock, flag);
		rwlock_init(&grp->hw_info->soc_dcam->cam_ahb_lock);

		g_ion_buf_q = &grp->ion_buf_q;
		cam_queue_init(g_ion_buf_q, CAM_EMP_Q_LEN_MAX, cam_queue_ioninfo_free);

		g_empty_frm_q = &grp->empty_frm_q;
		cam_queue_init(g_empty_frm_q, CAM_EMP_Q_LEN_MAX, cam_queue_empty_frame_free);

		g_empty_interruption_q = &grp->empty_interruption_q;
		cam_queue_init(g_empty_interruption_q, CAM_INT_EMP_Q_LEN_MAX,
			cam_queue_empty_interrupt_free);

		g_empty_zoom_q = &grp->empty_zoom_q;
		cam_queue_init(g_empty_zoom_q, CAM_ZOOM_EMP_Q_LEN_MAX, cam_queue_empty_zoom_free);

		cam_queue_init(&grp->mul_share_buf_q, CAM_SHARED_BUF_NUM, camcore_k_frame_put);

		spin_unlock_irqrestore(&grp->module_lock, flag);

		/* create recovery thread */
		if (grp->hw_info->ip_dcam[DCAM_ID_0]->recovery_support) {
			thrd = &grp->recovery_thrd;
			sprintf(thrd->thread_name, "cam_recovery");
			ret = camthread_create(grp, thrd, camcore_recovery_proc);
			if (ret)
				pr_warn("warning: creat recovery thread fail\n");
		}
		pr_info("init frm_q %px\n", g_empty_frm_q);
	}

	module->idx = idx;
	module->grp = grp;
	grp->module[idx] = module;
	file->private_data = (void *)module;

	pr_info("sprd_img: open end! %d, %px, %px, grp %px\n",
		idx, module, grp->module[idx], grp);

	return 0;

buf_manager_fail:
	camcore_module_deinit(module);
init_fail:
	cam_buf_kernel_sys_vfree(module);

alloc_fail:
	spin_lock_irqsave(&grp->module_lock, flag);
	grp->module_used &= ~(1 << idx);
	grp->module[idx] = NULL;
	spin_unlock_irqrestore(&grp->module_lock, flag);

exit:
	atomic_dec(&grp->camera_opened);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	sprd_glb_mm_pw_off_cfg();
	pm_runtime_put_sync(&grp->hw_info->pdev->dev);
#endif

	pr_err("fail to open camera %d\n", ret);
	return ret;
}

static int camcore_release(struct inode *node, struct file *file)
{
	int ret = 0;
	int idx = 0;
	unsigned long flag = 0;
	struct camera_group *group = NULL;
	struct camera_module *module = NULL;

	pr_info("sprd_img: cam release start.\n");

	module = (struct camera_module *)file->private_data;
	if (!module || !module->grp) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}
	module->private_key = 0;
	group = module->grp;
	idx = module->idx;

	if (module->grp->camsec_cfg.camsec_mode  != SEC_UNABLE) {
		bool ret = 0;

		module->grp->camsec_cfg.camsec_mode = SEC_UNABLE;
		ret = cam_trusty_security_set(&module->grp->camsec_cfg,
			CAM_TRUSTY_EXIT);

		pr_info("camca :camsec_mode%d, ret %d\n",
			module->grp->camsec_cfg.camsec_mode, ret);
	}

	pr_info("cam %d, state %d\n", idx, atomic_read(&module->state));
	pr_info("used: %d, module %px, %px, grp %px\n",
		group->module_used, module, group->module[idx], group);

	spin_lock_irqsave(&group->module_lock, flag);
	if (((group->module_used & (1 << idx)) == 0) ||
		(group->module[idx] != module)) {
		pr_err("fail to release camera %d. used:%x, module:%px\n",
			idx, group->module_used, module);
		spin_unlock_irqrestore(&group->module_lock, flag);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&group->module_lock, flag);

	down_read(&group->switch_recovery_lock);
	ret = camioctl_stream_off(module, 0L);
	up_read(&group->switch_recovery_lock);

	camcore_module_deinit(module);

	if (atomic_read(&module->state) == CAM_IDLE) {
	/* function "camioctl_cam_res_put" couldn't be called, when HAL sever is killed.
	* cause camera device did not close completely.
	* So. we need clear camera device resoures when HAL process exited abnormally.
	*/
		module->attach_sensor_id = -1;

		camthread_stop(&module->zoom_thrd);
		camthread_stop(&module->buf_thrd);

		if (module->dcam_dev_handle) {
			pr_info("force close dcam %px\n", module->dcam_dev_handle);
			module->dcam_dev_handle->dcam_pipe_ops->close(module->dcam_dev_handle);
			dcam_core_pipe_dev_put(module->dcam_dev_handle);
			module->dcam_dev_handle = NULL;
		}

		if (module->isp_dev_handle) {
			pr_info("force close isp %px\n", module->isp_dev_handle);
			module->isp_dev_handle->isp_ops->close(module->isp_dev_handle);
			isp_core_pipe_dev_put(module->isp_dev_handle);
			module->isp_dev_handle = NULL;
		}
	}

	spin_lock_irqsave(&group->module_lock, flag);
	group->module_used &= ~(1 << idx);
	group->module[idx] = NULL;
	spin_unlock_irqrestore(&group->module_lock, flag);

	cam_buf_kernel_sys_vfree(module);
	file->private_data = NULL;

	if (atomic_dec_return(&group->camera_opened) == 0) {
		struct cam_buf_pool_id pool_id = {0};
		spin_lock_irqsave(&group->module_lock, flag);

		if (group->mul_share_pyr_rec_buf) {
			camcore_k_frame_put(group->mul_share_pyr_rec_buf);
			group->mul_share_pyr_rec_buf = NULL;
		}

		if (group->mul_share_pyr_dec_buf) {
			camcore_k_frame_put(group->mul_share_pyr_dec_buf);
			group->mul_share_pyr_dec_buf = NULL;
		}

		pool_id.tag_id = CAM_BUF_POOL_SHARE_FULL_PATH;
		cam_buf_manager_pool_unreg(&pool_id);
		pr_info("release %px\n", g_empty_frm_q);

		/* g_leak_debug_cnt should be 0 after clr, or else memory leak */
		cam_queue_clear(&group->mul_share_buf_q, struct camera_frame, list);
		cam_queue_clear(g_ion_buf_q, struct camera_ion_info, list);
		g_ion_buf_q = NULL;
		cam_queue_clear(g_empty_frm_q, struct camera_frame, list);
		g_empty_frm_q = NULL;
		cam_queue_clear(g_empty_interruption_q, struct camera_interrupt, list);
		g_empty_interruption_q = NULL;
		cam_queue_clear(g_empty_zoom_q, struct cam_zoom_frame, list);
		g_empty_zoom_q = NULL;

		ret = cam_buf_mdbg_check();
		atomic_set(&group->runner_nr, 0);
		atomic_set(&group->mul_buf_alloced, 0);
		atomic_set(&group->mul_pyr_buf_alloced, 0);
		group->is_mul_buf_share = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		__pm_relax(group->ws);
#endif
		spin_unlock_irqrestore(&group->module_lock, flag);
		camthread_stop(&group->recovery_thrd);
	}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	ret = sprd_glb_mm_pw_off_cfg();
	pm_runtime_put_sync(&group->hw_info->pdev->dev);
#endif
	cam_buf_manager_deinit(idx);

	if (g_mem_dbg && g_mem_dbg->g_dbg_memory_leak_ctrl && atomic_read(&group->camera_opened) == 0)
		cam_buf_memory_leak_queue_check();

	ret = cam_buf_mdbg_check();

	if (g_mem_dbg && g_mem_dbg->g_dbg_memory_leak_ctrl && atomic_read(&group->camera_opened) == 0)
		cam_buf_memory_leak_queue_deinit();

	pr_info("sprd_img: cam %d release end.\n", idx);

	return ret;
}

static const struct file_operations image_fops = {
	.open = camcore_open,
	.unlocked_ioctl = camcore_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = camcore_ioctl_compat,
#endif
	.release = camcore_release,
	.read = camcore_read,
	.write = camcore_write,
};

static struct miscdevice image_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = IMG_DEVICE_NAME,
	.fops = &image_fops,
};

static int camcore_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	pr_info("Start camera img probe\n");
	group = vzalloc(sizeof(struct camera_group));
	if (group == NULL) {
		pr_err("fail to alloc memory\n");
		return -ENOMEM;
	}

	ret = misc_register(&image_dev);
	if (ret) {
		pr_err("fail to register misc devices, ret %d\n", ret);
		vfree(group);
		return -EACCES;
	}

	image_dev.this_device->of_node = pdev->dev.of_node;
	image_dev.this_device->platform_data = (void *)group;
	group->md = &image_dev;
	group->pdev = pdev;
	group->hw_info = (struct cam_hw_info *)of_device_get_match_data(&pdev->dev);
	if (!group->hw_info) {
		pr_err("fail to get hw_info\n");
		goto probe_pw_fail;
	}
	atomic_set(&group->camera_opened, 0);
	atomic_set(&group->runner_nr, 0);
	atomic_set(&group->recovery_state, CAM_RECOVERY_NONE);
	spin_lock_init(&group->module_lock);
	mutex_init(&group->pyr_mulshare_lock);
	spin_lock_init(&group->dual_deal_lock);
	init_rwsem(&group->switch_recovery_lock);

	pr_info("sprd img probe pdev name %s\n", pdev->name);
	pr_info("sprd dcam dev name %s\n", dev_name(&pdev->dev));

	ret = dcam_drv_dt_parse(pdev, group->hw_info, &group->dcam_count);
	if (ret) {
		pr_err("fail to parse dcam dts\n");
		goto probe_pw_fail;
	}

	pr_info("sprd isp dev name %s\n", dev_name(&pdev->dev));
	ret = isp_drv_dt_parse(pdev->dev.of_node, group->hw_info);
	if (ret) {
		pr_err("fail to parse isp dts\n");
		goto probe_pw_fail;
	}

	if (group->hw_info && group->hw_info->soc_dcam->pdev)
		ret = cam_buf_iommudev_reg(
			&group->hw_info->soc_dcam->pdev->dev,
			CAM_IOMMUDEV_DCAM);

	if (group->hw_info && group->hw_info->soc_dcam_lite && group->hw_info->soc_dcam_lite->pdev)
		ret = cam_buf_iommudev_reg(
			&group->hw_info->soc_dcam_lite->pdev->dev,
			CAM_IOMMUDEV_DCAM_LITE);

	if (group->hw_info && group->hw_info->soc_isp->pdev)
		ret = cam_buf_iommudev_reg(
			&group->hw_info->soc_isp->pdev->dev,
			CAM_IOMMUDEV_ISP);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	group->ws = wakeup_source_create("Camdrv Wakeuplock");
	wakeup_source_add(group->ws);
#endif

	ret = cam_scene_static_linkages_get(&group->topology_info, group->hw_info);
	if (ret)
		pr_err("fail to get invalid static linkages\n");

	group->debugger.hw = group->hw_info;
	ret = cam_debugger_init(&group->debugger);
	if (ret)
		pr_err("fail to init cam debugfs\n");

	return 0;

probe_pw_fail:
	misc_deregister(&image_dev);
	vfree(group);

	return ret;
}

static int camcore_remove(struct platform_device *pdev)
{
	struct camera_group *group = NULL;

	if (!pdev) {
		pr_err("fail to get valid input ptr\n");
		return -EFAULT;
	}

	group = image_dev.this_device->platform_data;
	if (group) {
		cam_buf_iommudev_unreg(CAM_IOMMUDEV_DCAM);
		cam_buf_iommudev_unreg(CAM_IOMMUDEV_DCAM_LITE);
		cam_buf_iommudev_unreg(CAM_IOMMUDEV_ISP);
		cam_debugger_deinit();
		if (group->ca_conn)
			cam_trusty_disconnect();
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
		wakeup_source_remove(group->ws);
		wakeup_source_destroy(group->ws);
#endif
		vfree(group);
		image_dev.this_device->platform_data = NULL;
	}
	misc_deregister(&image_dev);

	return 0;
}

static const struct of_device_id sprd_cam_of_match[] = {
	#if defined (PROJ_SHARKL3)
	{ .compatible = "sprd,sharkl3-cam", .data = &sharkl3_hw_info},
	#elif defined (PROJ_SHARKL5)
	{ .compatible = "sprd,sharkl5-cam", .data = &sharkl5_hw_info},
	#elif defined (PROJ_SHARKL5PRO)
	{ .compatible = "sprd,sharkl5pro-cam", .data = &sharkl5pro_hw_info},
	#elif defined (PROJ_QOGIRL6)
	{ .compatible = "sprd,qogirl6-cam", .data = &qogirl6_hw_info},
	#elif defined (PROJ_QOGIRN6PRO)
	{ .compatible = "sprd,qogirn6pro-cam", .data = &qogirn6pro_hw_info},
	#elif defined (PROJ_QOGIRN6L)
	{ .compatible = "sprd,qogirn6l-cam", .data = &qogirn6l_hw_info},
	#endif
	{ },
};

static struct platform_driver sprd_img_driver = {
	.probe = camcore_probe,
	.remove = camcore_remove,
	.driver = {
		.name = IMG_DEVICE_NAME,
		.of_match_table = of_match_ptr(sprd_cam_of_match),
	},
};

module_platform_driver(sprd_img_driver);

MODULE_DESCRIPTION("SPRD CAM Driver");
MODULE_AUTHOR("Multimedia_Camera@SPRD");
MODULE_LICENSE("GPL");
