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

#ifdef CAM_RAWCAP

int camrawcap_raw_pre_proc(struct camera_module *module,
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
	camcore_valid_fmt_get(&module->raw_cap_fetch_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = module->raw_cap_fetch_fmt;
	camcore_valid_fmt_get(&ch->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->raw_fmt_support[0]);
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

int camrawcap_raw_post_proc(struct camera_module *module,
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
			size = cal_sprd_raw_pitch(width, pack_bits) * height;
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
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF, mid_frame,
		CAM_NODE_TYPE_DCAM_OFFLINE);
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

int camrawcap_raw_proc_done(struct camera_module *module)
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

/* build channel/path in pre-processing */
int camrawcap_storeccm_frgb_pre_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t pipeline_type = 0;
	uint32_t pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct cam_pipeline_desc pipeline_desc = {0};
	struct dcam_offline_node_desc *dcam_offline_raw2frgb_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_frgb2yuv_desc = NULL;
	struct isp_node_desc *isp_node_description = NULL;
	struct pyr_dec_node_desc *pyr_dec_desc = NULL;
	struct dcam_path_cfg_param ch_desc = {0};
	struct isp_size_desc size_cfg = {0};

	pr_info("cam%d in. module:%px\n", module->idx, module);

	module->capture_type = CAM_CAPTURE_RAWPROC;
	hw = module->grp->hw_info;
	ch = &module->channel[CAM_CH_CAP];
	ch->dcam_port_id = -1;
	ch->isp_port_id = PORT_CAP_OUT;
	ch->aux_dcam_port_id = -1;
	ch->ch_uinfo.dcam_raw_fmt = -1;
	ch->ch_uinfo.sensor_raw_fmt = -1;
	pipeline_type = CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV;
	dcam_offline_raw2frgb_desc = &pipeline_desc.dcam_offline_raw2frgb_desc;
	dcam_offline_frgb2yuv_desc = &pipeline_desc.dcam_offline_frgb2yuv_desc;
	isp_node_description = &pipeline_desc.isp_node_description;
	pyr_dec_desc = &pipeline_desc.pyr_dec_desc;
	pipeline_desc.pipeline_graph = &module->static_topology->pipeline_list[pipeline_type];
	pipeline_desc.nodes_dev = &module->nodes_dev;
	pipeline_desc.data_cb_handle = module;
	pipeline_desc.data_cb_func = camcore_pipeline_callback;

	/* dcam offline raw2frgb desc */
	dcam_offline_raw2frgb_desc->dev = module->dcam_dev_handle;
	dcam_offline_raw2frgb_desc->dcam_idx = module->dcam_idx;
	dcam_offline_raw2frgb_desc->port_desc.endian = ENDIAN_LITTLE;
	camcore_valid_fmt_get(&module->raw_cap_fetch_fmt, hw->ip_dcam[0]->sensor_raw_fmt);
	dcam_offline_raw2frgb_desc->fetch_fmt = module->raw_cap_fetch_fmt;
	dcam_offline_raw2frgb_desc->port_desc.dcam_out_fmt = CAM_FULL_RGB14;

	/* dcam offline frgb2yuv desc */
	dcam_offline_frgb2yuv_desc->dev = module->dcam_dev_handle;
	dcam_offline_frgb2yuv_desc->dcam_idx = module->dcam_idx;
	dcam_offline_frgb2yuv_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_frgb2yuv_desc->fetch_fmt = CAM_FULL_RGB14;
	dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt = CAM_YUV420_2FRAME_MIPI;

	/* pyrdec param desc */
	pyr_dec_desc->hw = hw;
	pyr_dec_desc->dev = module->isp_dev_handle;
	pyr_dec_desc->pyrdec_dev = module->isp_dev_handle->pyr_dec_handle;
	pyr_dec_desc->buf_cb_func = pyr_dec_node_outbuf_get;
	pyr_dec_desc->layer_num = PYR_DEC_LAYER_NUM;
	pyr_dec_desc->node_idx = module->idx;
	pyr_dec_desc->sn_size = module->cam_uinfo.sn_size;
	pyr_dec_desc->in_fmt = dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt;
	pyr_dec_desc->pyr_out_fmt = hw->ip_dcam[0]->store_pyr_fmt;

	/* isp offline param desc */
	isp_node_description->in_fmt = dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt;
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
	isp_node_description->port_desc.in_fmt = dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt;
	isp_node_description->port_desc.bayer_pattern = isp_node_description->bayer_pattern;
	isp_node_description->port_desc.pyr_out_fmt = isp_node_description->pyr_out_fmt;
	isp_node_description->port_desc.store_3dnr_fmt = isp_node_description->store_3dnr_fmt;
	isp_node_description->port_desc.sn_size = isp_node_description->sn_size;

	ch->pipeline_handle = cam_pipeline_creat(&pipeline_desc);
	if (!ch->pipeline_handle) {
		pr_err("fail to get cam pipeline.\n");
		return -ENOMEM;
	}
	ch->enable = 1;
	ch->dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[DCAM_HW_CONTEXT_0]->aux_dcam_path);

	/* config ispctxid to pyrdec node */
	if (pyr_dec_desc && isp_node_description)
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
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_SIZE,
		&ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB);
	/* raw2frgb size info equal frgb2yuv */
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_SIZE,
		&ch_desc, CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV);

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

int camrawcap_storeccm_frgb_post_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t width = 0, height = 0, size = 0;
	uint32_t dcam_out_bits = 0;
	struct channel_context *ch = NULL;
	struct camera_frame *src_frame = NULL;
	struct camera_frame *mid_frame = NULL;
	struct camera_frame *dst_frame = NULL;
	struct camera_frame *mid_frgb_frame = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_pipe_dev *dev = NULL;
	timespec cur_ts = {0};
	memset(&cur_ts, 0, sizeof(timespec));
	pr_info("start\n");

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;

	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	pr_info("src %d 0x%x, mid %d 0x%x, dst %d 0x%x\n",
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
	ret = cam_buf_ionbuf_get(&src_frame->buf);
	if (ret)
		goto src_fail;

	dst_frame = cam_queue_empty_frame_get();
	dst_frame->buf.type = CAM_BUF_USER;
	dst_frame->buf.mfd = proc_info->fd_dst1;
	dst_frame->buf.offset[0] = proc_info->dst1_offset;
	dst_frame->channel_id = ch->ch_id;
	dst_frame->img_fmt = ch->ch_uinfo.dst_fmt;
	dst_frame->sensor_time = src_frame->sensor_time;
	dst_frame->boot_sensor_time = src_frame->boot_sensor_time;
	ret = cam_buf_ionbuf_get(&dst_frame->buf);
	if (ret)
		goto dst_fail;

	mid_frgb_frame = cam_queue_empty_frame_get();
	mid_frgb_frame->channel_id = ch->ch_id;

	width = proc_info->src_size.width;
	height = proc_info->src_size.height;
	size = width * height * FORMAT_FRGB_PITCH;
	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
	ret = cam_buf_alloc(&mid_frgb_frame->buf, (size_t)size, module->iommu_enable);
	if (ret)
		goto mid_frgb_fail;

	mid_frgb_frame->sensor_time = src_frame->sensor_time;
	mid_frgb_frame->boot_sensor_time = src_frame->boot_sensor_time;
	mid_frgb_frame->link_to.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV;
	mid_frgb_frame->link_to.node_id = DCAM_OFFLINE_NODE_ID;
	mid_frgb_frame->cam_fmt = CAM_FULL_RGB14;

	cam_queue_frame_flag_reset(mid_frgb_frame);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF,
		mid_frgb_frame, CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB);
	if (ret) {
		pr_err("fail to cfg dcam offline raw2frgb out buffer.\n");
		goto dcam_out_fail;
	}

	mid_frame = cam_queue_empty_frame_get();
	mid_frame->channel_id = ch->ch_id;
	/* if user set this buffer, we use it for dcam output
	 * or else we will allocate one for it.
	 */
	if (proc_info->fd_dst0 > 0) {
		mid_frame->buf.type = CAM_BUF_USER;
		mid_frame->link_from.node_type = CAM_NODE_TYPE_USER;
		mid_frame->buf.mfd = proc_info->fd_dst0;
		mid_frame->buf.offset[0] = proc_info->dst0_offset;
		ret = cam_buf_ionbuf_get(&mid_frame->buf);
		if (ret)
			goto mid_fail;
	} else {
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;
		size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		ret = cam_buf_alloc(&mid_frame->buf, (size_t)size, module->iommu_enable);
		if (ret)
			goto mid_fail;
	}
	mid_frame->sensor_time = src_frame->sensor_time;
	mid_frame->boot_sensor_time = src_frame->boot_sensor_time;
	mid_frame->link_to.node_type = CAM_NODE_TYPE_PYR_DEC;
	mid_frame->link_to.node_id = PYR_DEC_NODE_ID;
	if (module->cam_uinfo.is_pyr_dec)
		mid_frame->need_pyr_dec = 1;

	cam_queue_frame_flag_reset(mid_frame);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF,
		mid_frame, CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV);
	if (ret) {
		pr_err("fail to cfg dcam offline frgb2yuv out buffer.\n");
		goto mid_fail;
	}

	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, dst_frame);
	if (ret)
		pr_err("fail to cfg isp out buffer.\n");

	pr_info("frgb proc, src %px, mid %px, dst %px\n", src_frame, mid_frame, dst_frame);

	cam_queue_init(&ch->share_buf_queue, CAM_SHARED_BUF_NUM, camcore_k_frame_put);
	atomic_set(&module->state, CAM_RUNNING);

	if (module->cam_uinfo.is_pyr_dec) {
		/* dec out buf for frgb capture */
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
			goto dcam_out_fail;
		}
		ch->pyr_dec_buf = pframe;
		pr_info("pyr_dec size %d, buf %p, w %d h %d\n",
			size, pframe, width, height);
		ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPILINE_CFG_PYRDEC_BUF, ch->pyr_dec_buf);

		/* rec temp buf for frgb capture */
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
		pr_info("pyr_rec w %d, h %d, buf %p\n", width, height, pframe);
		CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_REC_BUF, ch->pyr_rec_buf);
	}

	ret = camcore_frame_start_proc(module, src_frame, CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB);
	if (ret)
		pr_err("fail to start dcam/isp for raw proc\n");

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;

dec_fail:
	if (ch->pyr_dec_buf) {
		if (ch->pyr_dec_buf->buf.dmabuf_p)
			cam_buf_free(&ch->pyr_dec_buf->buf);
		cam_queue_empty_frame_put(ch->pyr_dec_buf);
	}
mid_fail:
	cam_queue_empty_frame_put(mid_frame);
	cam_buf_ionbuf_put(&mid_frame->buf);
dcam_out_fail:
	cam_buf_free(&mid_frgb_frame->buf);
mid_frgb_fail:
	cam_queue_empty_frame_put(mid_frgb_frame);
	cam_buf_ionbuf_put(&dst_frame->buf);
dst_fail:
	cam_queue_empty_frame_put(dst_frame);
	cam_buf_ionbuf_put(&src_frame->buf);
src_fail:
	cam_queue_empty_frame_put(src_frame);
	pr_err("fail to call post raw proc\n");
	return ret;
}

int camrawcap_storeccm_frgb_proc_done(struct camera_module *module)
{
	int ret = 0, j = 0, i = 0;
	struct channel_context *ch = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_nodes_dev *nodes_dev = NULL;

	pr_info("cam%d start\n", module->idx);
	module->capture_type = CAM_CAPTURE_STOP;
	atomic_set(&module->state, CAM_STREAM_OFF);
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;

	if (atomic_read(&module->timeout_flag) == 1)
		pr_err("fail to raw proc, timeout\n");

	nodes_dev = &module->nodes_dev;
	ch = &module->channel[CAM_CH_CAP];

	cam_kernel_timer_stop(&module->cam_timer);

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
	pr_info("camera%d frgbproc done.\n", module->idx);

	cam_pipeline_destory(ch->pipeline_handle);
	nodes_dev->dcam_offline_node_dev = NULL;
	nodes_dev->dcam_offline_node_raw2frgb_dev = NULL;
	nodes_dev->dcam_offline_node_frgb2yuv_dev = NULL;
	for (i = 0; i < ISP_NODE_MODE_MAX_ID; i++) {
		nodes_dev->isp_node_dev[i] = NULL;
		for (j = 0; j < PORT_ISP_IN_MAX; j++)
			nodes_dev->isp_in_port_dev[i][j] = NULL;
		for (j = 0; j < PORT_ISP_OUT_MAX; j++)
			nodes_dev->isp_out_port_dev[i][j] = NULL;
	}
	for (j = 0; j < PORT_DCAM_OUT_MAX; j++) {
		nodes_dev->dcam_offline_out_port_dev[j] = NULL;
		nodes_dev->dcam_offline_raw2frgb_out_port_dev[j] = NULL;
		nodes_dev->dcam_offline_frgb2yuv_out_port_dev[j] = NULL;
	}

	return ret;
}
#endif
