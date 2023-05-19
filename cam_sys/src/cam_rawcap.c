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
	uint32_t rawpath_id = 0;
	uint32_t pipeline_type = 0, pyrdec_support = 0;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct cam_pipeline_desc *pipeline_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_desc = NULL;
	struct isp_node_desc *isp_node_description = NULL;
	struct pyr_dec_node_desc *pyr_dec_desc = NULL;

	pr_info("cam%d in. module:%px\n", module->idx, module);

	module->capture_type = CAM_CAPTURE_RAWPROC;
	hw = module->grp->hw_info;
	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);
	pyrdec_support = hw->ip_isp->isphw_abt->pyr_dec_support;
	ch = &module->channel[CAM_CH_CAP];
	ch->dcam_port_id = -1;
	ch->isp_port_id = PORT_CAP_OUT;
	ch->aux_dcam_port_id = -1;
	ch->ch_uinfo.dcam_raw_fmt = -1;
	ch->ch_uinfo.sensor_raw_fmt = -1;
	pipeline_type = CAM_PIPELINE_OFFLINE_RAW2YUV;
	pipeline_desc = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_pipeline_desc));
	if (!pipeline_desc) {
		pr_err("fail to alloc pipeline_des\n");
		return -ENOMEM;
	}
	dcam_offline_desc = &pipeline_desc->dcam_offline_desc;
	isp_node_description = &pipeline_desc->isp_node_description;
	pyr_dec_desc = &pipeline_desc->pyr_dec_desc;
	pipeline_desc->pipeline_graph = &module->static_topology->pipeline_list[pipeline_type];
	pipeline_desc->nodes_dev = &module->nodes_dev;
	pipeline_desc->data_cb_handle = module;
	pipeline_desc->buf_manager_handle =  module->grp->global_buf_manager;
	pipeline_desc->data_cb_func = camcore_pipeline_callback;
	pipeline_desc->zoom_cb_func = camcore_zoom_param_get_callback;

	dcam_offline_desc->dev = module->dcam_dev_handle;
	dcam_offline_desc->buf_manager_handle = pipeline_desc->buf_manager_handle;
	if (hw->prj_id == SHARKL3)
		dcam_offline_desc->csi_controller_idx = CSI_ID_0;
	else
		dcam_offline_desc->csi_controller_idx = module->csi_controller_idx;
	dcam_offline_desc->endian = proc_info->src_y_endian;
	dcam_offline_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_desc->pattern = proc_info->src_pattern;
	dcam_offline_desc->port_desc.src_sel = PROCESS_RAW_SRC_SEL;
	dcam_offline_desc->statis_en = 1;
	cam_valid_fmt_get((uint32_t*)&module->raw_cap_fetch_fmt, hw->ip_dcam[0]->dcamhw_abt->sensor_raw_fmt);
	dcam_offline_desc->fetch_fmt = module->raw_cap_fetch_fmt;
	cam_valid_fmt_get(&ch->ch_uinfo.dcam_raw_fmt, hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[0]);
	dcam_offline_desc->port_desc.dcam_out_fmt = ch->ch_uinfo.dcam_raw_fmt;
	if (module->cam_uinfo.dcam_slice_mode && hw->ip_dcam[0]->dcamhw_abt->save_band_for_bigsize)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_RAW_PACK_10;
	if (hw->ip_isp->isphw_abt->fetch_raw_support == 0)
		dcam_offline_desc->port_desc.dcam_out_fmt = CAM_YVU420_2FRAME_MIPI;

	/* isp offline param desc */
	isp_node_description->in_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	isp_node_description->pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
	isp_node_description->store_3dnr_fmt = hw->ip_dcam[0]->dcamhw_abt->store_3dnr_fmt[0];
	if (module->cam_uinfo.is_pyr_dec)
		isp_node_description->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
	else
		isp_node_description->pyr_layer_num = 0;
	isp_node_description->mode_3dnr = MODE_3DNR_OFF;
	isp_node_description->mode_ltm = MODE_LTM_OFF;
	isp_node_description->mode_gtm = MODE_GTM_OFF;
	isp_node_description->ch_id = ch->ch_id;
	isp_node_description->bayer_pattern = proc_info->src_pattern;
	isp_node_description->enable_slowmotion = 0;
	isp_node_description->is_high_fps = 0;
	isp_node_description->slw_state = CAM_SLOWMOTION_OFF;
	isp_node_description->cam_id = module->idx;
	isp_node_description->buf_manager_handle = pipeline_desc->buf_manager_handle;
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
	isp_node_description->port_desc.hw = hw;

	pyr_dec_desc->hw = hw;
	pyr_dec_desc->layer_num = PYR_DEC_LAYER_NUM;
	pyr_dec_desc->node_idx = module->idx;
	pyr_dec_desc->buf_manager_handle = pipeline_desc->buf_manager_handle;
	pyr_dec_desc->dcam_slice_mode = module->cam_uinfo.dcam_slice_mode;
	pyr_dec_desc->is_4in1 = module->cam_uinfo.is_4in1;
	pyr_dec_desc->is_rawcap = 1;
	pyr_dec_desc->sn_size = module->cam_uinfo.sn_size;
	pyr_dec_desc->dev = module->isp_dev_handle;
	pyr_dec_desc->pyrdec_dev = module->isp_dev_handle->pyr_dec_handle;
	pyr_dec_desc->in_fmt = dcam_offline_desc->port_desc.dcam_out_fmt;
	pyr_dec_desc->pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
	pyr_dec_desc->blkparam_buf_num = 1;
	ch->pipeline_handle = cam_pipeline_creat(pipeline_desc);
	if (!ch->pipeline_handle) {
		pr_err("fail to get cam pipeline.\n");
		ret = -ENOMEM;
		goto fail;
	}
	ch->enable = 1;
	ch->dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcamhw_abt->aux_dcam_path);
	ch->pipeline_type = pipeline_type;
	CAM_QUEUE_INIT(&ch->zoom_param_q, CAM_ZOOM_COEFF_Q_LEN, NULL);

	/*TEMP: config ispctxid to pyrdec node*/
	if (pipeline_desc->pipeline_graph->need_pyr_dec && pyrdec_support)
		camcore_pyrdec_ctxid_cfg(ch, isp_node_description->isp_node);

	ch->trim_dcam.size_x = proc_info->src_size.width;
	ch->trim_dcam.size_y = proc_info->src_size.height;
	ch->dst_dcam.w = ch->trim_dcam.size_x;
	ch->dst_dcam.h = ch->trim_dcam.size_y;
	ch->trim_isp.size_x = proc_info->src_size.width;
	ch->trim_isp.size_y = proc_info->src_size.height;
	ch->ch_uinfo.dst_size.w = proc_info->dst_size.width;
	ch->ch_uinfo.dst_size.h = proc_info->dst_size.height;
	ret = cam_zoom_channel_size_config(module, ch);

fail:
	cam_buf_kernel_sys_vfree(pipeline_desc);

	return ret;
}

int camrawcap_raw_post_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t width = 0, height = 0;
	uint32_t format = 0, size = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct channel_context *ch = NULL;
	struct isp_node *isp_node = NULL;
	struct pyr_dec_node *pyrdec_node = NULL;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct cam_frame *src_frame = NULL;
	struct cam_frame *mid_frame = NULL;
	struct cam_frame *dst_frame = NULL;
	struct cam_buf_alloc_desc alloc_param = {0};
	struct cam_node_cfg_param node_param = {0};
	timespec cur_ts = {0};
	memset(&cur_ts, 0, sizeof(timespec));
	pr_info("start\n");

	hw = module->grp->hw_info;
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	isp_node = module->nodes_dev.isp_node_dev[ISP_NODE_MODE_CAP_ID];

	pr_info("src %d 0x%x, mid %d, 0x%x, dst %d, 0x%x\n",
		proc_info->fd_src, proc_info->src_offset,
		proc_info->fd_dst0, proc_info->dst0_offset,
		proc_info->fd_dst1, proc_info->dst1_offset);
	src_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	src_frame->common.buf.type = CAM_BUF_USER;
	src_frame->common.link_from.node_type = CAM_NODE_TYPE_USER;
	src_frame->common.buf.mfd = proc_info->fd_src;
	src_frame->common.buf.offset[0] = proc_info->src_offset;
	src_frame->common.channel_id = ch->ch_id;
	src_frame->common.width = proc_info->src_size.width;
	src_frame->common.height = proc_info->src_size.height;
	os_adapt_time_get_ts(&cur_ts);
	src_frame->common.sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->common.sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->common.boot_sensor_time = os_adapt_time_get_boottime();
	src_frame->common.buf.status = CAM_BUF_ALLOC;

	dst_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	dst_frame->common.buf.type = CAM_BUF_USER;
	dst_frame->common.buf.mfd = proc_info->fd_dst1;
	dst_frame->common.buf.offset[0] = proc_info->dst1_offset;
	dst_frame->common.channel_id = ch->ch_id;
	dst_frame->common.img_fmt = ch->ch_uinfo.dst_fmt;
	dst_frame->common.sensor_time = src_frame->common.sensor_time;
	dst_frame->common.boot_sensor_time = src_frame->common.boot_sensor_time;
	dst_frame->common.buf.status = CAM_BUF_ALLOC;

	mid_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	mid_frame->common.channel_id = ch->ch_id;
	mid_frame->common.buf.status = CAM_BUF_ALLOC;
	/* if user set this buffer, we use it for dcam output
	 * or else we will allocate one for it.
	 */
	if(ch->dcam_port_id == PORT_OFFLINE_FULL_OUT && module->cam_uinfo.is_4in1 == 1)
		format = CAM_RAW_PACK_10;
	else
		format = ch->ch_uinfo.dcam_raw_fmt;

	pr_info("dcam raw_proc_post format %d\n", format);
	if (proc_info->fd_dst0 > 0) {
		mid_frame->common.buf.type = CAM_BUF_USER;
		mid_frame->common.link_from.node_type = CAM_NODE_TYPE_USER;
		mid_frame->common.buf.mfd = proc_info->fd_dst0;
		mid_frame->common.buf.offset[0] = proc_info->dst0_offset;
	} else {
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;

		if (proc_info->src_format == IMG_PIX_FMT_GREY)
			size = cal_sprd_pitch(width, format) * height;
		else
			size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		ret = cam_buf_alloc(&mid_frame->common.buf, (size_t)size, module->iommu_enable);
		if (ret) {
			pr_err("fail to get mid_frame.\n");
			goto mid_fail;
		}
	}
	mid_frame->common.sensor_time = src_frame->common.sensor_time;
	mid_frame->common.boot_sensor_time = src_frame->common.boot_sensor_time;
	CAM_QUEUE_FRAME_FLAG_RESET(&mid_frame->common);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF,
		mid_frame, CAM_NODE_TYPE_DCAM_OFFLINE);
	if (ret) {
		pr_err("fail to cfg dcam out buffer.\n");
		goto dcam_out_fail;
	}

	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, ISP_NODE_MODE_CAP_ID, dst_frame);
	if (ret) {
		pr_err("fail to cfg isp out buffer.\n");
		goto dcam_out_fail;
	}

	pr_info("raw proc, src %px, mid %px, dst %px\n", src_frame, mid_frame, dst_frame);
	atomic_set(&module->state, CAM_RUNNING);

	if (module->cam_uinfo.is_pyr_dec) {
		pyrdec_node = module->nodes_dev.pyr_dec_node_dev;
		rec_ctx = (struct isp_rec_ctx_desc *)isp_node->rec_handle;

		alloc_param.ch_id = ch->ch_id;
		alloc_param.width = proc_info->src_size.width;
		alloc_param.height = proc_info->src_size.height;
		alloc_param.iommu_enable = module->iommu_enable;
		alloc_param.is_pyr_dec = module->cam_uinfo.is_pyr_dec;
		alloc_param.pyr_out_fmt = hw->ip_dcam[0]->dcampath_abt[DCAM_BIN_OUT_PATH]->format[0];
		alloc_param.pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;

		node_param.port_id = PORT_DEC_OUT;
		node_param.param = &alloc_param;
		pyrdec_node->port_cfg_cb_func(&node_param, PORT_CFG_BUFFER_ALLOC, pyrdec_node->port_cfg_cb_handle);
		isp_pyr_rec_buffer_alloc(rec_ctx, &alloc_param, isp_node->buf_manager_handle);
	}

	ret = camcore_frame_start_proc(src_frame, CAM_NODE_TYPE_DCAM_OFFLINE, ch);
	if (ret) {
		pr_err("fail to start dcam/isp for raw proc\n");
		goto src_fail;

	}

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;

src_fail:
	cam_buf_ionbuf_put(&dst_frame->common.buf);
dcam_out_fail:
	if (mid_frame->common.buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&mid_frame->common.buf);
	else
		cam_buf_free(&mid_frame->common.buf);
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

	camcore_timer_deinit(&module->cam_timer);

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
	module->cam_uinfo.slice_num = 0;
	module->cam_uinfo.dcam_slice_mode = CAM_SLICE_NONE;

	atomic_set(&module->state, CAM_IDLE);
	camcore_pipeline_deinit(module, ch);

	pr_info("camera%d rawproc done.\n", module->idx);
	return ret;
}

/* build channel/path in pre-processing */
int camrawcap_storeccm_frgb_pre_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t pipeline_type = 0;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct cam_pipeline_desc *pipeline_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_raw2frgb_desc = NULL;
	struct dcam_offline_node_desc *dcam_offline_frgb2yuv_desc = NULL;
	struct isp_node_desc *isp_node_description = NULL;
	struct pyr_dec_node_desc *pyr_dec_desc = NULL;

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
	pipeline_desc = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_pipeline_desc));
	if (!pipeline_desc) {
		pr_err("fail to alloc pipeline_des\n");
		return -ENOMEM;
	}
	dcam_offline_raw2frgb_desc = &pipeline_desc->dcam_offline_raw2frgb_desc;
	dcam_offline_frgb2yuv_desc = &pipeline_desc->dcam_offline_frgb2yuv_desc;
	isp_node_description = &pipeline_desc->isp_node_description;
	pyr_dec_desc = &pipeline_desc->pyr_dec_desc;
	pipeline_desc->pipeline_graph = &module->static_topology->pipeline_list[pipeline_type];
	pipeline_desc->nodes_dev = &module->nodes_dev;
	pipeline_desc->data_cb_handle = module;
	pipeline_desc->buf_manager_handle =  module->grp->global_buf_manager;
	pipeline_desc->data_cb_func = camcore_pipeline_callback;

	/* dcam offline raw2frgb desc */
	dcam_offline_raw2frgb_desc->dev = module->dcam_dev_handle;
	dcam_offline_raw2frgb_desc->csi_controller_idx = module->csi_controller_idx;
	dcam_offline_raw2frgb_desc->endian = proc_info->src_y_endian;
	dcam_offline_raw2frgb_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_raw2frgb_desc->pattern = proc_info->src_pattern;
	cam_valid_fmt_get((uint32_t*)&module->raw_cap_fetch_fmt, hw->ip_dcam[0]->dcamhw_abt->sensor_raw_fmt);
	dcam_offline_raw2frgb_desc->fetch_fmt = module->raw_cap_fetch_fmt;
	dcam_offline_raw2frgb_desc->port_desc.dcam_out_fmt = CAM_FULL_RGB14;

	/* dcam offline frgb2yuv desc */
	dcam_offline_frgb2yuv_desc->dev = module->dcam_dev_handle;
	dcam_offline_frgb2yuv_desc->csi_controller_idx = module->csi_controller_idx;
	dcam_offline_frgb2yuv_desc->endian = proc_info->src_y_endian;
	dcam_offline_frgb2yuv_desc->port_desc.endian = ENDIAN_LITTLE;
	dcam_offline_frgb2yuv_desc->pattern = proc_info->src_pattern;
	dcam_offline_frgb2yuv_desc->fetch_fmt = CAM_FULL_RGB14;
	dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt = CAM_YVU420_2FRAME_MIPI;

	/* pyrdec param desc */
	pyr_dec_desc->hw = hw;
	pyr_dec_desc->dev = module->isp_dev_handle;
	pyr_dec_desc->pyrdec_dev = module->isp_dev_handle->pyr_dec_handle;
	pyr_dec_desc->layer_num = PYR_DEC_LAYER_NUM;
	pyr_dec_desc->node_idx = module->idx;
	pyr_dec_desc->sn_size = module->cam_uinfo.sn_size;
	pyr_dec_desc->in_fmt = dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt;
	pyr_dec_desc->pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
	pyr_dec_desc->blkparam_buf_num = 1;

	/* isp offline param desc */
	isp_node_description->in_fmt = dcam_offline_frgb2yuv_desc->port_desc.dcam_out_fmt;
	isp_node_description->pyr_out_fmt = hw->ip_dcam[0]->dcamhw_abt->store_pyr_fmt;
	isp_node_description->store_3dnr_fmt = hw->ip_dcam[0]->dcamhw_abt->store_3dnr_fmt[0];
	if (module->cam_uinfo.is_pyr_dec)
		isp_node_description->pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;
	else
		isp_node_description->pyr_layer_num = 0;
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
	isp_node_description->port_desc.hw = hw;

	ch->pipeline_handle = cam_pipeline_creat(pipeline_desc);
	if (!ch->pipeline_handle) {
		pr_err("fail to get cam pipeline.\n");
		ret = -ENOMEM;
		goto fail;
	}
	ch->enable = 1;
	ch->dcam_port_id = dcamoffline_pathid_convert_to_portid(hw->ip_dcam[0]->dcamhw_abt->aux_dcam_path);
	ch->pipeline_type = pipeline_type;
	CAM_QUEUE_INIT(&ch->zoom_param_q, CAM_ZOOM_COEFF_Q_LEN, NULL);

	/* config ispctxid to pyrdec node */
	if (pyr_dec_desc && isp_node_description)
		camcore_pyrdec_ctxid_cfg(ch, isp_node_description->isp_node);

	ch->trim_dcam.size_x = proc_info->src_size.width;
	ch->trim_dcam.size_y = proc_info->src_size.height;
	ch->dst_dcam.w = ch->trim_dcam.size_x;
	ch->dst_dcam.h = ch->trim_dcam.size_y;
	ch->trim_isp.size_x = proc_info->src_size.width;
	ch->trim_isp.size_y = proc_info->src_size.height;
	ch->ch_uinfo.dst_size.w = proc_info->dst_size.width;
	ch->ch_uinfo.dst_size.h = proc_info->dst_size.height;
	ret = cam_zoom_channel_size_config(module, ch);
fail:
	cam_buf_kernel_sys_vfree(pipeline_desc);

	return ret;
}

int camrawcap_storeccm_frgb_post_proc(struct camera_module *module,
		struct isp_raw_proc_info *proc_info)
{
	int ret = 0;
	uint32_t width = 0, height = 0, size = 0;
	struct cam_hw_info *hw = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct channel_context *ch = NULL;
	struct isp_node *isp_node = NULL;
	struct pyr_dec_node *pyrdec_node = NULL;
	struct isp_rec_ctx_desc *rec_ctx = NULL;
	struct cam_frame *src_frame = NULL;
	struct cam_frame *mid_frame = NULL;
	struct cam_frame *dst_frame = NULL;
	struct cam_frame *mid_frgb_frame = NULL;
	struct cam_buf_alloc_desc alloc_param = {0};
	struct cam_node_cfg_param node_param = {0};
	timespec cur_ts = {0};
	memset(&cur_ts, 0, sizeof(timespec));
	pr_info("start\n");

	hw = module->grp->hw_info;
	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	ch = &module->channel[CAM_CH_CAP];
	if (ch->enable == 0) {
		pr_err("fail to get channel enable state\n");
		return -EFAULT;
	}

	isp_node = module->nodes_dev.isp_node_dev[ISP_NODE_MODE_CAP_ID];

	pr_info("src %d 0x%x, mid %d 0x%x, dst %d 0x%x\n",
		proc_info->fd_src, proc_info->src_offset,
		proc_info->fd_dst0, proc_info->dst0_offset,
		proc_info->fd_dst1, proc_info->dst1_offset);
	src_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	src_frame->common.buf.type = CAM_BUF_USER;
	src_frame->common.link_from.node_type = CAM_NODE_TYPE_USER;
	src_frame->common.buf.mfd = proc_info->fd_src;
	src_frame->common.buf.offset[0] = proc_info->src_offset;
	src_frame->common.channel_id = ch->ch_id;
	src_frame->common.width = proc_info->src_size.width;
	src_frame->common.height = proc_info->src_size.height;
	os_adapt_time_get_ts(&cur_ts);
	src_frame->common.sensor_time.tv_sec = cur_ts.tv_sec;
	src_frame->common.sensor_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	src_frame->common.boot_sensor_time = os_adapt_time_get_boottime();
	ret = cam_buf_ionbuf_get(&src_frame->common.buf);
	if (ret)
		goto src_fail;

	dst_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	dst_frame->common.buf.type = CAM_BUF_USER;
	dst_frame->common.buf.mfd = proc_info->fd_dst1;
	dst_frame->common.buf.offset[0] = proc_info->dst1_offset;
	dst_frame->common.channel_id = ch->ch_id;
	dst_frame->common.img_fmt = ch->ch_uinfo.dst_fmt;
	dst_frame->common.sensor_time = src_frame->common.sensor_time;
	dst_frame->common.boot_sensor_time = src_frame->common.boot_sensor_time;
	ret = cam_buf_ionbuf_get(&dst_frame->common.buf);

	mid_frgb_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	mid_frgb_frame->common.channel_id = ch->ch_id;

	width = proc_info->src_size.width;
	height = proc_info->src_size.height;
	size = width * height * FORMAT_FRGB_PITCH;
	size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
	ret = cam_buf_alloc(&mid_frgb_frame->common.buf, (size_t)size, module->iommu_enable);

	mid_frgb_frame->common.sensor_time = src_frame->common.sensor_time;
	mid_frgb_frame->common.boot_sensor_time = src_frame->common.boot_sensor_time;
	mid_frgb_frame->common.link_to.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV;
	mid_frgb_frame->common.link_to.node_id = DCAM_OFFLINE_NODE_ID;
	mid_frgb_frame->common.cam_fmt = CAM_FULL_RGB14;

	CAM_QUEUE_FRAME_FLAG_RESET(&mid_frgb_frame->common);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF,
		mid_frgb_frame, CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB);
	if (ret) {
		pr_err("fail to cfg dcam offline raw2frgb out buffer.\n");
		goto dcam_out_fail;
	}

	mid_frame = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	mid_frame->common.channel_id = ch->ch_id;
	/* if user set this buffer, we use it for dcam output
	 * or else we will allocate one for it.
	 */
	if (proc_info->fd_dst0 > 0) {
		mid_frame->common.buf.type = CAM_BUF_USER;
		mid_frame->common.link_from.node_type = CAM_NODE_TYPE_USER;
		mid_frame->common.buf.mfd = proc_info->fd_dst0;
		mid_frame->common.buf.offset[0] = proc_info->dst0_offset;
		ret = cam_buf_ionbuf_get(&mid_frame->common.buf);
		if (ret)
			goto mid_fail;
	} else {
		width = proc_info->src_size.width;
		height = proc_info->src_size.height;
		size = width * height * 3;
		size = ALIGN(size, CAM_BUF_ALIGN_SIZE);
		ret = cam_buf_alloc(&mid_frame->common.buf, (size_t)size, module->iommu_enable);
		if (ret)
			goto mid_fail;
	}
	mid_frame->common.sensor_time = src_frame->common.sensor_time;
	mid_frame->common.boot_sensor_time = src_frame->common.boot_sensor_time;
	mid_frame->common.link_to.node_type = CAM_NODE_TYPE_PYR_DEC;
	mid_frame->common.link_to.node_id = PYR_DEC_NODE_ID;

	CAM_QUEUE_FRAME_FLAG_RESET(&mid_frame->common);
	ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF,
		mid_frame, CAM_NODE_TYPE_DCAM_OFFLINE_FRGB2YUV);
	if (ret) {
		pr_err("fail to cfg dcam offline frgb2yuv out buffer.\n");
		goto mid_fail;
	}

	ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, ISP_NODE_MODE_CAP_ID, dst_frame);
	if (ret)
		pr_err("fail to cfg isp out buffer.\n");

	pr_info("frgb proc, src %px, mid %px, dst %px\n", src_frame, mid_frame, dst_frame);

	atomic_set(&module->state, CAM_RUNNING);
	if (module->cam_uinfo.is_pyr_dec) {
		pyrdec_node = module->nodes_dev.pyr_dec_node_dev;
		rec_ctx = (struct isp_rec_ctx_desc *)isp_node->rec_handle;

		alloc_param.ch_id = ch->ch_id;
		alloc_param.width = proc_info->src_size.width;
		alloc_param.height = proc_info->src_size.height;
		alloc_param.iommu_enable = module->iommu_enable;
		alloc_param.is_pyr_dec = module->cam_uinfo.is_pyr_dec;
		alloc_param.pyr_out_fmt = hw->ip_dcam[0]->dcampath_abt[DCAM_BIN_OUT_PATH]->format[0];
		alloc_param.pyr_layer_num = ISP_PYR_DEC_LAYER_NUM;

		node_param.port_id = PORT_DEC_OUT;
		node_param.param = &alloc_param;
		pyrdec_node->port_cfg_cb_func(&node_param, PORT_CFG_BUFFER_ALLOC, pyrdec_node->port_cfg_cb_handle);
		isp_pyr_rec_buffer_alloc(rec_ctx, &alloc_param, isp_node->buf_manager_handle);
	}

	ret = camcore_frame_start_proc(src_frame, CAM_NODE_TYPE_DCAM_OFFLINE_RAW2FRGB, ch);
	if (ret)
		pr_err("fail to start dcam/isp for raw proc\n");

	atomic_set(&module->timeout_flag, 1);
	ret = camcore_timer_start(&module->cam_timer, CAMERA_TIMEOUT);

	return ret;

src_fail:
	cam_buf_ionbuf_put(&dst_frame->common.buf);
dcam_out_fail:
	if (mid_frame->common.buf.type == CAM_BUF_USER)
		cam_buf_ionbuf_put(&mid_frame->common.buf);
	else
		cam_buf_free(&mid_frame->common.buf);
mid_fail:
	cam_queue_empty_frame_put(mid_frame);
	cam_queue_empty_frame_put(dst_frame);
	cam_queue_empty_frame_put(src_frame);
	pr_err("fail to call post frgb proc\n");

	return ret;
}

int camrawcap_storeccm_frgb_proc_done(struct camera_module *module)
{
	int ret = 0;
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

	camcore_timer_deinit(&module->cam_timer);

	ch->enable = 0;
	ch->dcam_port_id = -1;
	ch->isp_port_id = -1;
	ch->aux_dcam_port_id = -1;
	ch->ch_uinfo.dcam_raw_fmt = -1;
	ch->ch_uinfo.sensor_raw_fmt = -1;

	module->cam_uinfo.dcam_slice_mode = CAM_SLICE_NONE;
	module->cam_uinfo.slice_num = 0;

	atomic_set(&module->state, CAM_IDLE);
	pr_info("camera%d frgbproc done.\n", module->idx);

	camcore_pipeline_deinit(module, ch);
	return ret;
}
#endif
