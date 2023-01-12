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
#include "isp_scaler_port.h"
#include "cam_port.h"
#include "isp_interface.h"
#include "isp_dev.h"
#include "isp_port.h"
#include "isp_drv.h"
#include "isp_scaler_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_PORT: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define ISP_DIV_ALIGN_W(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_WIDTH - 1))
#define ISP_DIV_ALIGN_H(_a, _b)    (((_a) / (_b)) & ~(ISP_PIXEL_ALIGN_HEIGHT - 1))
#define ISP_PATH_DECI_FAC_MAX       4
#define ISP_SC_COEFF_UP_MAX         ISP_SCALER_UP_MAX
#define ISP_SC_COEFF_DOWN_MAX       ISP_SCALER_UP_MAX
#define ISP_PIXEL_ALIGN_WIDTH      4
#define ISP_PIXEL_ALIGN_HEIGHT     2

struct camera_frame *ispscaler_port_reserved_buf_get(reserved_buf_get_cb resbuf_get_cb, void *cb_data, void *path)
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

bool ispscaler_port_fid_check(struct camera_frame *frame, void *data)
{
	uint32_t target_fid;

	if (!frame || !data)
		return false;

	target_fid = *(uint32_t *)data;

	pr_debug("target_fid = %d frame->user_fid = %d\n", target_fid, frame->user_fid);
	return frame->user_fid == CAMERA_RESERVE_FRAME_NUM
		|| frame->user_fid == target_fid;
}

int ispscaler_port_fetch_normal_get(void *cfg_in, void *cfg_out, struct camera_frame *frame)
{
	int ret = 0;
	uint32_t trim_offset[3] = { 0 };
	struct img_size *src = NULL;
	struct img_trim *intrim = NULL;
	struct isp_hw_fetch_info *fetch = NULL;
	struct isp_scaler_uinfo *pipe_src = NULL;
	uint32_t mipi_word_num_start[16] = {
		0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5};
	uint32_t mipi_word_num_end[16] = {
		0, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5};
	if (!cfg_in || !cfg_out || !frame) {
		pr_err("fail to get valid input ptr, %p, %p\n", cfg_in, cfg_out);
		return -EFAULT;
	}

	pipe_src = (struct isp_scaler_uinfo *)cfg_in;
	fetch = (struct isp_hw_fetch_info *)cfg_out;

	src = &pipe_src->src;
	intrim = &pipe_src->crop;
	fetch->src = *src;
	fetch->in_trim = *intrim;
	fetch->fetch_fmt = pipe_src->in_fmt;
	fetch->bayer_pattern = pipe_src->bayer_pattern;
	if (camcore_raw_fmt_get(pipe_src->in_fmt))
		fetch->dispatch_color = 0;
	else if (pipe_src->in_fmt == CAM_FULL_RGB10)
		fetch->dispatch_color = 1;
	else
		fetch->dispatch_color = 2;
	fetch->addr.addr_ch0 = frame->buf.iova;

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
		fetch->pitch.pitch_ch0 = cal_sprd_raw_pitch(src->w, cam_pack_bits(pipe_src->in_fmt));
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		break;
	case CAM_YUV422_2FRAME:
	case CAM_YVU422_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 + intrim->start_x;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		break;
	case CAM_YUV420_2FRAME:
	case CAM_YVU420_2FRAME:
		fetch->pitch.pitch_ch0 = src->w;
		fetch->pitch.pitch_ch1 = src->w;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 / 2 + intrim->start_x;
		fetch->addr.addr_ch1 = fetch->addr.addr_ch0 + fetch->pitch.pitch_ch0 * fetch->src.h;
		pr_debug("y_addr: %x, pitch:: %x\n", fetch->addr.addr_ch0, fetch->pitch.pitch_ch0);
		break;
	case CAM_YUV420_2FRAME_10:
	case CAM_YVU420_2FRAME_10:
		fetch->pitch.pitch_ch0 = (src->w * 16 + 127) / 128 * 128 / 8;
		fetch->pitch.pitch_ch1 = (src->w * 16 + 127) / 128 * 128 / 8;
		trim_offset[0] = intrim->start_y * fetch->pitch.pitch_ch0 + intrim->start_x * 2;
		trim_offset[1] = intrim->start_y * fetch->pitch.pitch_ch1 / 2 + intrim->start_x * 2;
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

int ispscaler_port_store_normal_get(void *cfg_in, struct isp_hw_path_store *store_info)
{
	int ret = 0;
	struct isp_store_info *store = NULL;
	struct isp_scaler_path_uinfo *in_ptr = NULL;

	if (!cfg_in || !store_info) {
		pr_err("fail to get valid input ptr %p\n", cfg_in, store_info);
		return -EFAULT;
	}
	in_ptr = (struct isp_scaler_path_uinfo *)cfg_in;

	store = &store_info->store;
	store->color_fmt = in_ptr->out_fmt;
	store->bypass = 0;
	store->endian = in_ptr->data_endian;
	if (store->color_fmt == CAM_FULL_RGB14)
		store->speed_2x = 0;
	else
		store->speed_2x = 1;

	if (cam_data_bits(in_ptr->out_fmt) == CAM_10_BITS)
		store->need_bwd = 0;
	else
		store->need_bwd = 1;

	store->mirror_en = 0;
	store->max_len_sel = 0;
	store->shadow_clr_sel = 1;
	store->shadow_clr = 1;
	store->store_res = 1;
	store->rd_ctrl = 0;
	store->size.w = in_ptr->dst.w;
	store->size.h = in_ptr->dst.h;
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

static uint32_t ispscaler_port_deci_factor_get(uint32_t src_size, uint32_t dst_size)
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

static int ispscaler_port_scaler_param_calc(struct img_trim *in_trim,
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
			deci->deci_x = ispscaler_port_deci_factor_get(in_trim->size_x, tmp_dstsize);
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
			deci->deci_y = ispscaler_port_deci_factor_get(in_trim->size_y, tmp_dstsize);
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

int ispscaler_port_scaler_get(void *cfg_in, struct isp_hw_path_scaler *path)
{
	int ret = 0;
	uint32_t is_yuv422 = 0, scale2yuv420 = 0;
	struct yuv_scaler_info *scaler = NULL;
	struct isp_scaler_path_uinfo *in_ptr = NULL;

	if (!cfg_in || !path) {
		pr_err("fail to get valid input ptr %p, %p\n", cfg_in, path);
		return -EFAULT;
	}
	in_ptr = (struct isp_scaler_path_uinfo *)cfg_in;

	scaler = &path->scaler;
	if (in_ptr->out_fmt == CAM_UYVY_1FRAME)
		path->uv_sync_v = 1;
	else
		path->uv_sync_v = 0;
	if (in_ptr->out_fmt == CAM_FULL_RGB14)
		path->path_sel = 2;
	else
		path->path_sel = 0;
	path->frm_deci = 0;
	path->dst = in_ptr->dst;
	path->out_trim.start_x = 0;
	path->out_trim.start_y = 0;
	path->out_trim.size_x = in_ptr->dst.w;
	path->out_trim.size_y = in_ptr->dst.h;
	path->regular_info.regular_mode = in_ptr->regular_mode;
	ret = ispscaler_port_scaler_param_calc(&path->in_trim, &path->dst,
		&path->scaler, &path->deci);
	if (ret) {
		pr_err("fail to calc scaler param.\n");
		return ret;
	}

	if ((in_ptr->out_fmt == CAM_YUV422_2FRAME) || (in_ptr->out_fmt == CAM_YVU422_2FRAME))
		is_yuv422 = 1;

	if (((scaler->scaler_ver_factor_in == scaler->scaler_ver_factor_out)
		&& (scaler->scaler_factor_in == scaler->scaler_factor_out)
		&& (is_yuv422 || in_ptr->scaler_bypass_ctrl))
		|| in_ptr->out_fmt == CAM_FULL_RGB14) {
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

int ispscaler_thumbport_scaler_get(void *cfg_in, struct isp_hw_thumbscaler_info *scalerInfo)
{
	int ret = 0;
	uint32_t deci_w = 0;
	uint32_t deci_h = 0;
	uint32_t trim_w, trim_h, temp_w, temp_h;
	uint32_t offset, shift, is_yuv422 = 0;
	struct img_size src, dst;
	uint32_t align_size = 0;
	struct isp_scaler_path_uinfo *in_ptr = NULL;

	if (!cfg_in || !scalerInfo) {
		pr_err("fail to get valid input ptr %p\n", cfg_in, scalerInfo);
		return -EFAULT;
	}
	in_ptr = (struct isp_scaler_path_uinfo *)cfg_in;

	scalerInfo->scaler_bypass = 0;
	scalerInfo->frame_deci = 0;
	/* y factor & deci */
	src.w = in_ptr->in_trim.size_x;
	src.h = in_ptr->in_trim.size_y;
	dst = in_ptr->dst;
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
	scalerInfo->y_factor_out = in_ptr->dst;

	if ((in_ptr->out_fmt == CAM_YUV422_2FRAME) || (in_ptr->out_fmt == CAM_YVU422_2FRAME))
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

	scalerInfo->src0.w = in_ptr->in_trim.size_x;
	scalerInfo->src0.h = in_ptr->in_trim.size_y;

	/* y trim */
	trim_w = scalerInfo->y_factor_in.w * scalerInfo->y_deci.deci_x;
	offset = (in_ptr->in_trim.size_x - trim_w) / 2;
	scalerInfo->y_trim.start_x = in_ptr->in_trim.start_x + offset;
	scalerInfo->y_trim.size_x = trim_w;

	trim_h = scalerInfo->y_factor_in.h * scalerInfo->y_deci.deci_y;
	offset = (in_ptr->in_trim.size_y - trim_h) / 2;
	scalerInfo->y_trim.start_y = in_ptr->in_trim.start_y + offset;
	scalerInfo->y_trim.size_y = trim_h;

	scalerInfo->y_src_after_deci = scalerInfo->y_factor_in;
	scalerInfo->y_dst_after_scaler = scalerInfo->y_factor_out;

	/* uv trim */
	trim_w = scalerInfo->uv_factor_in.w * scalerInfo->uv_deci.deci_x;
	offset = (in_ptr->in_trim.size_x / 2 - trim_w) / 2;
	scalerInfo->uv_trim.start_x = in_ptr->in_trim.start_x / 2 + offset;
	scalerInfo->uv_trim.size_x = trim_w;

	trim_h = scalerInfo->uv_factor_in.h * scalerInfo->uv_deci.deci_y;
	offset = (in_ptr->in_trim.size_y - trim_h) / 2;
	scalerInfo->uv_trim.start_y = in_ptr->in_trim.start_y + offset;
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

uint32_t isp_scaler_port_id_switch(uint32_t port_id)
{
	uint32_t hw_path_id = ISP_SPATH_NUM;
	switch (port_id) {
	case PORT_PRE_OUT:
	case PORT_CAP_OUT:
		hw_path_id = ISP_SPATH_CP;
		break;
	case PORT_VID_OUT:
		hw_path_id = ISP_SPATH_VID;
		break;
	case PORT_THUMB_OUT:
		hw_path_id = ISP_SPATH_FD;
		break;
	default:
		pr_err("port_id:%d\n",port_id);
	}
	return hw_path_id;
}

static void isp_scaler_port_frame_ret(void *param)
{
	struct camera_frame * pframe = NULL;
	struct isp_scaler_port *port = NULL;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	pframe = (struct camera_frame *)param;
	port = (struct isp_scaler_port *)pframe->priv_data;//temp
	if (!port) {
		pr_err("fail to get out_frame port.\n");
		return;
	}

	if (pframe->is_reserved)
		port->resbuf_get_cb(RESERVED_BUF_SET_CB, pframe, port->resbuf_cb_data);
	else {
		if (pframe->buf.mapping_state & CAM_BUF_MAPPING_DEV)
			cam_buf_iommu_unmap(&pframe->buf);
		port->data_cb_func(CAM_CB_ISP_RET_DST_BUF, pframe, port->data_cb_handle);
	}

}

void *isp_scaler_port_get(uint32_t port_id, struct isp_scaler_port_desc *param)
{
	struct isp_scaler_port *port = NULL;

	if (!param) {
		pr_err("fail to get valid param\n");
		return NULL;
	}

	pr_info("port id %d  node_dev %px\n", port_id, *param->port_dev);
	if (*param->port_dev == NULL) {
		port = cam_buf_kernel_sys_vzalloc(sizeof(struct isp_scaler_port));
		if (!port) {
			pr_err("fail to get valid isp port %d\n", port_id);
			return NULL;
		}
	} else {
		port = *param->port_dev;
		pr_info("isp port has been alloc %p %d\n", port, port_id);
		goto exit;
	}
	cam_queue_init(&port->result_queue, ISP_RESULT_Q_LEN, isp_scaler_port_frame_ret);
	cam_queue_init(&port->out_buf_queue, ISP_OUT_BUF_Q_LEN,isp_scaler_port_frame_ret);

	port->resbuf_get_cb = param->resbuf_get_cb;
	port->resbuf_cb_data = param->resbuf_cb_data;
	port->data_cb_handle = param->data_cb_handle;
	port->data_cb_func = param->data_cb_func;
	port->port_id = port_id;

	*param->port_dev = port;
exit:
	atomic_inc(&port->user_cnt);
	return port;
}

void isp_scaler_port_put(struct isp_scaler_port *port)
{
	if (!port) {
		pr_err("fail to get invalid port ptr\n");
		return;
	}

	cam_queue_clear(&port->out_buf_queue, struct camera_frame, list);
	cam_queue_clear(&port->result_queue, struct camera_frame, list);
	atomic_set(&port->user_cnt, 0);
	port->resbuf_get_cb = NULL;
	port->data_cb_func = NULL;
	cam_buf_kernel_sys_vfree(port);
	port = NULL;
	pr_info("isp port free success\n");
}

