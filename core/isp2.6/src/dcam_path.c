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


#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <sprd_mm.h>

#include "cam_scaler.h"
#include "cam_hw.h"
#include "cam_types.h"
#include "cam_queue.h"
#include "cam_buf.h"
#include "cam_block.h"
#include "cam_debugger.h"

#include "dcam_interface.h"
#include "dcam_reg.h"
#include "dcam_int.h"
#include "dcam_core.h"
#include "dcam_path.h"
#include "dcam_hw_if.h"

/* Macro Definitions */
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_PATH: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__


/*
 * path name for debug output
 */
static const char *_DCAM_PATH_NAMES[DCAM_PATH_MAX] = {
	[DCAM_PATH_FULL] = "FULL",
	[DCAM_PATH_BIN] = "BIN",
	[DCAM_PATH_PDAF] = "PDAF",
	[DCAM_PATH_VCH2] = "VCH2",
	[DCAM_PATH_VCH3] = "VCH3",
	[DCAM_PATH_AEM] = "AEM",
	[DCAM_PATH_AFM] = "AFM",
	[DCAM_PATH_AFL] = "AFL",
	[DCAM_PATH_HIST] = "HIST",
	[DCAM_PATH_3DNR] = "3DNR",
	[DCAM_PATH_BPC] = "BPC",
};

/*
 * convert @path_id to path name
 */
const char *to_path_name(enum dcam_path_id path_id)
{
	return is_path_id(path_id) ? _DCAM_PATH_NAMES[path_id] : "(null)";
}


static const unsigned long dcam_store_addr[DCAM_PATH_MAX] = {
	DCAM_FULL_BASE_WADDR,
	DCAM_BIN_BASE_WADDR0,
	DCAM_PDAF_BASE_WADDR,
	DCAM_VCH2_BASE_WADDR,
	DCAM_VCH3_BASE_WADDR,
	DCAM_AEM_BASE_WADDR,
	ISP_AFM_BASE_WADDR,
	ISP_AFL_GLB_WADDR,
	DCAM_HIST_BASE_WADDR,
	ISP_NR3_WADDR,
	ISP_BPC_OUT_ADDR,
};
static const unsigned long dcam2_store_addr[DCAM_PATH_MAX] = {
	DCAM2_PATH0_BASE_WADDR,
	DCAM2_PATH1_BASE_WADDR,
	/* below:not cover usefull register */
	DCAM_PDAF_BASE_WADDR,
	DCAM_VCH2_BASE_WADDR,
	DCAM_VCH3_BASE_WADDR,
	DCAM_AEM_BASE_WADDR,
	ISP_AFM_BASE_WADDR,
	ISP_AFL_GLB_WADDR,
	DCAM_HIST_BASE_WADDR,
	ISP_NR3_WADDR,
	ISP_BPC_OUT_ADDR,
};

/*
 * TODO slow motion
 * remove unused address for AEM and HIST
 */
static const unsigned long slowmotion_store_addr[3][4] = {
	{
		DCAM_BIN_BASE_WADDR0,
		DCAM_BIN_BASE_WADDR1,
		DCAM_BIN_BASE_WADDR2,
		DCAM_BIN_BASE_WADDR3
	},
	{
		DCAM_AEM_BASE_WADDR,
		DCAM_AEM_BASE_WADDR1,
		DCAM_AEM_BASE_WADDR2,
		DCAM_AEM_BASE_WADDR3
	},
	{
		DCAM_HIST_BASE_WADDR,
		DCAM_HIST_BASE_WADDR1,
		DCAM_HIST_BASE_WADDR2,
		DCAM_HIST_BASE_WADDR3
	}
};

int dcam_cfg_path_base(void *dcam_handle,
		       struct dcam_path_desc *path, void *param)
{
	int ret = 0;
	uint32_t idx;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_cfg_param *ch_desc;

	if (!dcam_handle || !path || !param) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	ch_desc = (struct dcam_path_cfg_param *)param;
	idx = dev->idx;

	switch (path->path_id) {
	case DCAM_PATH_FULL:
		path->src_sel = ch_desc->is_raw ? 0 : 1;
	case DCAM_PATH_BIN:
		path->frm_deci = ch_desc->frm_deci;
		path->frm_skip = ch_desc->frm_skip;

		path->is_loose = ch_desc->is_loose;
		path->endian = ch_desc->endian;
		/*
		 * TODO:
		 * Better not binding dcam_if feature to BIN path, which is a
		 * architecture defect and not going to be fixed now.
		 */
		dev->slowmotion_count = ch_desc->slowmotion_count;
		dev->is_3dnr |= ch_desc->enable_3dnr;
		dev->raw_cap = ch_desc->raw_cap;
		break;

	case DCAM_PATH_VCH2:
		path->endian = ch_desc->endian;
		path->src_sel = ch_desc->is_raw ? 1 : 0;
		break;

	default:
		pr_err("unknown path %d\n", path->path_id);
		ret = -EFAULT;
		break;
	}
	return ret;
}

int dcam_cfg_path_size(void *dcam_handle,
				struct dcam_path_desc *path,
				void *param)
{
	int ret = 0;
	uint32_t idx;
	uint32_t invalid;
	struct img_size crop_size, dst_size;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_path_cfg_param *ch_desc;
	struct sprd_cam_hw_info *hw = NULL;
	uint32_t dcam_max_w = 0, dcam_max_h = 0;

	if (!dcam_handle || !path || !param) {
		pr_err("error input ptr.\n");
		return -EFAULT;
	}
	dev = (struct dcam_pipe_dev *)dcam_handle;
	hw = dev->hw;
	if (!hw) {
		pr_err("hw ptr is NULL.\n");
		return -EFAULT;
	}
	dcam_max_w = hw->path_max_width;
	dcam_max_h = hw->path_max_height;

	ch_desc = (struct dcam_path_cfg_param *)param;
	idx = dev->idx;

	switch (path->path_id) {
	case DCAM_PATH_FULL:
		spin_lock(&path->size_lock);
		if (path->size_update) {
			spin_unlock(&path->size_lock);
			return -EFAULT;
		}
		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		invalid |= (path->in_size.w > dcam_max_w);
		invalid |= (path->in_size.h > dcam_max_h);
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);
		if (invalid) {
			spin_unlock(&path->size_lock);
			pr_err("error size:%d %d, trim %d %d %d %d\n",
				path->in_size.w, path->in_size.h,
				path->in_trim.start_x, path->in_trim.start_y,
				path->in_trim.size_x,
				path->in_trim.size_y);
			return -EINVAL;
		}
		if ((path->in_size.w > path->in_trim.size_x) ||
			(path->in_size.h > path->in_trim.size_y)) {
			path->out_size.w = path->in_trim.size_x;
			path->out_size.h = path->in_trim.size_y;
		} else {
			path->out_size.w = path->in_size.w;
			path->out_size.h = path->in_size.h;
		}

		path->priv_size_data = ch_desc->priv_size_data;
		path->size_update = 1;
		spin_unlock(&path->size_lock);

		pr_info("cfg full path done. size %d %d %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("sel %d. trim %d %d %d %d\n", path->src_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	case DCAM_PATH_BIN:
		/* lock here to keep all size parameters updating is atomic.
		 * because the rds coeff caculation may be time-consuming,
		 * we should not disable irq here, or else may cause irq missed.
		 * Just trylock set_next_frame in irq handling to avoid deadlock
		 * If last updating has not been applied yet, will return error.
		 * error may happen if too frequent zoom ratio updateing,
		 * but should not happen for first time cfg before stream on,
		 * if error return, caller can discard updating
		 * or try cfg_size again after while.
		 */
		spin_lock(&path->size_lock);
		if (path->size_update) {
			if (atomic_read(&dev->state) != STATE_RUNNING)
				pr_info("Overwrite dcam path size before dcam start if any\n");
			else {
				spin_unlock(&path->size_lock);
				pr_info("Previous path updating pending\n");
				return -EFAULT;
			}
		}

		path->in_size = ch_desc->input_size;
		path->in_trim = ch_desc->input_trim;
		path->out_size = ch_desc->output_size;

		invalid = 0;
		invalid |= ((path->in_size.w == 0) || (path->in_size.h == 0));
		invalid |= (path->in_size.w > dcam_max_w);
		invalid |= (path->in_size.h > dcam_max_h);

		/* trim should not be out range of source */
		invalid |= ((path->in_trim.start_x +
				path->in_trim.size_x) > path->in_size.w);
		invalid |= ((path->in_trim.start_y +
				path->in_trim.size_y) > path->in_size.h);

		/* output size should not be larger than trim ROI */
		invalid |= path->in_trim.size_x < path->out_size.w;
		invalid |= path->in_trim.size_y < path->out_size.h;

		/* Down scaling should not be smaller then 1/4*/
		invalid |= path->in_trim.size_x >
				(path->out_size.w * DCAM_SCALE_DOWN_MAX);
		invalid |= path->in_trim.size_y >
				(path->out_size.h * DCAM_SCALE_DOWN_MAX);

		if (invalid) {
			spin_unlock(&path->size_lock);
			pr_err("error size:%d %d, trim %d %d %d %d, dst %d %d\n",
				path->in_size.w, path->in_size.h,
				path->in_trim.start_x, path->in_trim.start_y,
				path->in_trim.size_x, path->in_trim.size_y,
				path->out_size.w, path->out_size.h);
			return -EINVAL;
		}
		crop_size.w = path->in_trim.size_x;
		crop_size.h = path->in_trim.size_y;
		dst_size = path->out_size;

		if ((crop_size.w == dst_size.w) &&
			(crop_size.h == dst_size.h))
			path->scaler_sel = DCAM_SCALER_BYPASS;
		else if ((dst_size.w * 2 == crop_size.w) &&
			(dst_size.h * 2 == crop_size.h)) {
			pr_debug("1/2 binning used. src %d %d, dst %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_BINNING;
			path->bin_ratio = 0;
		} else if ((dst_size.w * 4 == crop_size.w) &&
			(dst_size.h * 4 == crop_size.h)) {
			pr_debug("1/4 binning used. src %d %d, dst %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_BINNING;
			path->bin_ratio = 1;
		} else {
			pr_debug("RDS used. in %d %d, out %d %d\n",
				crop_size.w, crop_size.h, dst_size.w,
				dst_size.h);
			path->scaler_sel = DCAM_SCALER_RAW_DOWNSISER;
			dcam_gen_rds_coeff((uint16_t)crop_size.w,
				(uint16_t)crop_size.h,
				(uint16_t)dst_size.w,
				(uint16_t)dst_size.h,
				(uint32_t *)path->rds_coeff_buf);
		}
		path->priv_size_data = ch_desc->priv_size_data;
		path->size_update = 1;
		/* if 3dnr path enable, need update when zoom */
		{
			struct dcam_path_desc *path_3dnr;

			path_3dnr = &dev->path[DCAM_PATH_3DNR];
			if (atomic_read(&path_3dnr->user_cnt) > 0)
				path_3dnr->size_update = 1;
		}
		spin_unlock(&path->size_lock);

		pr_info("cfg bin path done. size %d %d  dst %d %d\n",
			path->in_size.w, path->in_size.h,
			path->out_size.w, path->out_size.h);
		pr_info("scaler %d. trim %d %d %d %d\n", path->scaler_sel,
			path->in_trim.start_x, path->in_trim.start_y,
			path->in_trim.size_x, path->in_trim.size_y);
		break;

	default:
		pr_err("unknown path %d\n", path->path_id);
		ret = -EFAULT;
		break;
	}
	return ret;
}

/*
 * Set skip num for path @path_id so that we can update address accordingly in
 * CAP SOF interrupt.
 *
 * For slow motion mode.
 * On previous project, there's no slow motion support for AEM on DCAM. Instead,
 * we use @skip_num to generate AEM TX DONE every @skip_num frame. If @skip_num
 * is set by algorithm, we should configure AEM output address accordingly in
 * CAP SOF every @skip_num frame.
 */
int dcam_path_set_skip_num(struct dcam_pipe_dev *dev,
			   int path_id, uint32_t skip_num)
{
	struct dcam_path_desc *path = NULL;

	if (unlikely(!dev || !is_path_id(path_id)))
		return -EINVAL;

	if (atomic_read(&dev->state) == STATE_RUNNING) {
		pr_warn("DCAM%u %s set skip_num while running is forbidden\n",
			dev->idx, to_path_name(path_id));
		return -EINVAL;
	}

	path = &dev->path[path_id];
	path->frm_deci = skip_num;
	path->frm_deci_cnt = 0;

	pr_info("DCAM%u %s set skip_num %u\n",
		dev->idx, to_path_name(path_id), skip_num);

	return 0;
}

static inline struct camera_frame *
dcam_path_cycle_frame(struct dcam_pipe_dev *dev, struct dcam_path_desc *path)
{
	struct camera_frame *frame = NULL;

	frame = camera_dequeue(&path->out_buf_queue);
	if (frame == NULL)
		frame = camera_dequeue(&path->reserved_buf_queue);

	if (frame == NULL) {
		pr_debug("DCAM%u %s buffer unavailable\n",
		       dev->idx, to_path_name(path->path_id));
		return ERR_PTR(-ENOMEM);
	}

	if (camera_enqueue(&path->result_queue, frame) < 0) {
		if (frame->is_reserved)
			camera_enqueue(&path->reserved_buf_queue, frame);
		else
			camera_enqueue(&path->out_buf_queue, frame);

		pr_err("DCAM%u %s output queue overflow\n",
		       dev->idx, to_path_name(path->path_id));
		return ERR_PTR(-EPERM);
	}

	frame->fid = dev->index_to_set;
	frame->sync_data = NULL;

	return frame;
}

static inline void swap_frame_pointer(struct camera_frame **frame1,
				      struct camera_frame **frame2)
{
	struct camera_frame *frame;

	frame = *frame1;
	*frame1 = *frame2;
	*frame2 = frame;
}

int dcam_path_set_store_frm(void *dcam_handle,
			    struct dcam_path_desc *path,
			    struct dcam_sync_helper *helper)
{
	struct dcam_pipe_dev *dev = NULL;
	struct camera_frame *frame = NULL, *saved = NULL;
	struct dcam_if *dcam = NULL;
	uint32_t idx = 0, path_id = 0;
	unsigned long flags = 0, addr = 0;
	const int _bin = 0, _aem = 1, _hist = 2;
	int i = 0, ret = 0;

	if (unlikely(!dcam_handle || !path))
		return -EINVAL;

	dev = (struct dcam_pipe_dev *)dcam_handle;
	idx = dev->idx;
	path_id = path->path_id;
	dev->auto_cpy_id |= dcam_get_path_ctrl_id(path_id);

	pr_debug("DCAM%u %s enter\n", idx, to_path_name(path_id));

	frame = dcam_path_cycle_frame(dev, path);
	if (IS_ERR(frame))
		return PTR_ERR(frame);

	/* assign last buffer for AEM and HIST in slow motion */
	i = dev->slowmotion_count - 1;

	if (idx == DCAM_ID_2) {
		/* dcam2 path0 ~ full path */
		addr = dcam2_store_addr[path_id];
	} else if (dev->slowmotion_count && path_id == DCAM_PATH_AEM) {
		/* slow motion AEM */
		addr = slowmotion_store_addr[_aem][i];
		frame->fid += i;
	} else if (dev->slowmotion_count && path_id == DCAM_PATH_HIST) {
		/* slow motion HIST */
		addr = slowmotion_store_addr[_hist][i];
		frame->fid += i;
	} else {
		/* normal scene */
		addr = dcam_store_addr[path_id];
	}

	/* replace image data for debug */
	if (dev->replacer) {
		struct dcam_image_replacer *replacer = dev->replacer;

		if ((path_id < DCAM_IMAGE_REPLACER_PATH_MAX)
			&& replacer->enabled[path_id])
			saved = camera_dequeue(&path->reserved_buf_queue);
	}

	if (saved)
		swap_frame_pointer(&frame, &saved);
	if (frame->is_compressed) {
		struct compressed_addr compressed_addr;
		struct img_size *size = &path->out_size;

		dcam_if_cal_compressed_addr(size->w, size->h,
					    frame->buf.iova[0],
					    &compressed_addr);
		DCAM_REG_WR(idx, addr, compressed_addr.addr2);
		//DCAM_REG_WR(idx, DCAM_FBC_PAYLOAD_WADDR,
			    //compressed_addr.addr1);
	} else {
		DCAM_REG_WR(idx, addr, frame->buf.iova[0]);
	}
	if (saved)
		swap_frame_pointer(&frame, &saved);

	atomic_inc(&path->set_frm_cnt);
	if (path_id == DCAM_PATH_AFL)
		DCAM_REG_WR(idx, ISP_AFL_REGION_WADDR,
			frame->buf.iova[0] + STATIS_AFL_GBUF_SIZE);

	pr_debug("DCAM%u %s set frame: fid %u, count %d\n",
		 idx, to_path_name(path_id), frame->fid,
		 atomic_read(&path->set_frm_cnt));

	pr_debug("DCAM%u %s reg %08x, addr %08x\n", idx, to_path_name(path_id),
		 (uint32_t)addr, (uint32_t)frame->buf.iova[0]);

	/* bind frame sync data if it is not reserved buffer */
	if (helper && !frame->is_reserved && is_sync_enabled(dev, path_id)) {
		helper->enabled |= BIT(path_id);
		helper->sync.frames[path_id] = frame;
		frame->sync_data = &helper->sync;
	}

	if ((path_id == DCAM_PATH_FULL) || (path_id == DCAM_PATH_BIN) ||
		(path_id == DCAM_PATH_3DNR)) {
		/* use trylock here to avoid waiting if cfg_path_size
		 * is already lock
		 * because this function maybe called from irq handling.
		 * disable irq to make sure less time gap between size updating
		 * to register and frame buffer address updating to register
		 * make sure that corresponding param
		 * will applied for same frame.
		 * or else size may mismatch with frame.
		 */
		if (spin_trylock_irqsave(&path->size_lock, flags)) {
			if (path->size_update && !frame->is_reserved) {
				dcam_update_path_size(dev, path);
				frame->param_data = path->priv_size_data;
				path->size_update = 0;
				path->priv_size_data = NULL;
			}
			spin_unlock_irqrestore(&path->size_lock, flags);
		}
	}

	if (dev->is_pdaf && dev->pdaf_type == 3 && path_id == DCAM_PATH_PDAF) {
		/*
		 * PDAF type3, when need write addr to DCAM_PPE_RIGHT_WADDR
		 * PDAF type3, half buffer for right PD, TBD
		 */
		DCAM_REG_WR(idx, DCAM_PPE_RIGHT_WADDR,
			    frame->buf.iova[0] + STATIS_PDAF_BUF_SIZE / 2);
	}

	dcam = dcam_get_dcam_if(idx);
	if (dev->slowmotion_count && !dev->index_to_set &&
	    (path_id == DCAM_PATH_AEM || path_id == DCAM_PATH_HIST) &&
	    dcam && (dcam->slowmotion_path & BIT(path_id))) {
		/* configure reserved buffer for AEM and hist */
		frame = camera_dequeue(&path->reserved_buf_queue);
		if (!frame) {
			pr_debug("DCAM%u %s buffer unavailable\n",
				 idx, to_path_name(path_id));
			return -ENOMEM;
		}

		i = 0;
		while (i < dev->slowmotion_count - 1) {
			if (path_id == DCAM_PATH_AEM)
				addr = slowmotion_store_addr[_aem][i];
			else
				addr = slowmotion_store_addr[_hist][i];
			DCAM_REG_WR(idx, addr, frame->buf.iova[0]);

			pr_debug("DCAM%u %s set reserved frame\n", dev->idx,
				 to_path_name(path_id));
			i++;
		}


		/* put it back */
		camera_enqueue(&path->reserved_buf_queue, frame);
	} else if (dev->slowmotion_count && path_id == DCAM_PATH_BIN) {
		i = 1;
		while (i < dev->slowmotion_count) {
			frame = dcam_path_cycle_frame(dev, path);
			/* in init phase, return failure if error happens */
			if (IS_ERR(frame) && !dev->index_to_set) {
				ret = PTR_ERR(frame);
				goto enqueue_reserved;
			}

			/* in normal running, just stop configure */
			if (IS_ERR(frame))
				break;

			addr = slowmotion_store_addr[_bin][i];
			if (saved)
				swap_frame_pointer(&frame, &saved);
			DCAM_REG_WR(idx, addr, frame->buf.iova[0]);
			if (saved)
				swap_frame_pointer(&frame, &saved);
			atomic_inc(&path->set_frm_cnt);

			frame->fid = dev->index_to_set + i;

			pr_debug("DCAM%u BIN set frame: fid %u, count %d\n",
				 idx, frame->fid,
				 atomic_read(&path->set_frm_cnt));
			i++;
		}

		if (unlikely(i != dev->slowmotion_count))
			pr_warn("DCAM%u BIN %d frame missed\n",
				idx, dev->slowmotion_count - i);
	}

enqueue_reserved:
	if (saved)
		camera_enqueue(&path->reserved_buf_queue, saved);

	return ret;
}
