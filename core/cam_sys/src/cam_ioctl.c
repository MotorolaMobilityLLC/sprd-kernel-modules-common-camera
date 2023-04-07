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

#include <linux/compat.h>

#include "cam_buf_monitor.h"

#ifdef CAM_IOCTL_LAYER

extern g_dbg_raw2frgb_switch;

#define IS_XTM_CONFLICT_SCENE(scene) ({\
	uint32_t __s = scene;          \
	(__s == CAPTURE_FDR            \
	|| __s == CAPTURE_HW3DNR       \
	|| __s == CAPTURE_FLASH        \
	|| __s == CAPTURE_RAWALG       \
	|| __s == CAPTURE_AI_SFNR);    \
})

static int camioctl_time_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_time utime = {0};
	timespec ts = {0};

	memset(&ts, 0, sizeof(timespec));
	os_adapt_time_get_ts(&ts);
	utime.sec = (uint32_t)ts.tv_sec;
	utime.usec = (uint32_t)(ts.tv_nsec / NSEC_PER_USEC);
	pr_debug("get_time %d.%06d\n", utime.sec, utime.usec);

	ret = copy_to_user((void __user *)arg, &utime, sizeof(struct sprd_img_time));
	if (unlikely(ret)) {
		pr_err("fail to put user info, ret %d\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int camioctl_flash_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_set_flash set_param = {0};

	ret = copy_from_user((void *)&set_param,
		(void __user *)arg, sizeof(struct sprd_img_set_flash));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		goto exit;
	}

	module->flash_info.led0_ctrl = set_param.led0_ctrl;
	module->flash_info.led1_ctrl = set_param.led1_ctrl;
	module->flash_info.led0_status = set_param.led0_status;
	module->flash_info.led1_status = set_param.led1_status;
	module->flash_info.set_param = set_param;
	pr_info("led_ctrl %d %d led_status %d %d\n", set_param.led0_ctrl, set_param.led1_ctrl,
		set_param.led0_status, set_param.led1_status);

exit:
	return ret;
}

static int camioctl_flash_cfg(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_cfg_param cfg_parm = {0};

	ret = copy_from_user((void *) &cfg_parm,
		(void __user *)arg, sizeof(struct sprd_flash_cfg_param));
	if (ret) {
		pr_err("fail to get user info\n");
		ret = -EFAULT;
		goto exit;
	}

	ret = module->flash_core_handle->flash_core_ops->cfg_flash(module->flash_core_handle,
		(void *)&cfg_parm);

exit:
	return ret;
}

static int camioctl_flash_get(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_flash_capacity flash_info = {0};

	ret = module->flash_core_handle->flash_core_ops->get_flash(module->flash_core_handle,
		(void *)&flash_info);

	ret = copy_to_user((void __user *)arg, (void *)&flash_info,
		sizeof(struct sprd_flash_capacity));
	if (unlikely(ret)) {
		pr_err("fail to copy to user, ret %d\n", ret);
		ret = -EFAULT;
	}

	return ret;
}

static int camioctl_iommu_status_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	unsigned int iommu_enable = 0;
	struct device *dcam_dev = NULL;

	dcam_dev = &module->grp->pdev->dev;
	if (cam_buf_iommu_status_get(CAM_BUF_IOMMUDEV_DCAM) == 0)
		iommu_enable = 1;
	else
		iommu_enable = 0;
	module->iommu_enable = iommu_enable;

	ret = copy_to_user((void __user *)arg, &iommu_enable, sizeof(unsigned char));

	if (unlikely(ret)) {
		pr_err("fail to copy to user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_info("iommu_enable:%d\n", iommu_enable);
exit:
	return ret;
}

static int camioctl_statis_buf_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0, is_raw_cap = 0;
	struct channel_context *ch = NULL;
	struct isp_statis_buf_input statis_buf = {0};
	struct cam_hw_info *hw = module->dcam_dev_handle->hw;
	struct dcam_statis_param statis_param = {0};

	ret = copy_from_user((void *)&statis_buf,
		(void *)arg, sizeof(struct isp_statis_buf_input));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	if ((statis_buf.type == STATIS_INIT)
		&& (atomic_read(&module->state) != CAM_IDLE)
		&& (atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_err("fail to get init statis buf state: %d\n", atomic_read(&module->state));
		ret = -EFAULT;
		goto exit;
	}

	if ((statis_buf.type != STATIS_INIT) &&
		(statis_buf.type < STATIS_TYPE_MAX) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warning: should not configure statis buf for state %d\n",
			atomic_read(&module->state));
		goto exit;
	}
	ch = &module->channel[CAM_CH_PRE];
	is_raw_cap = (module->capture_type == CAM_CAPTURE_RAWPROC ? 1 : 0);
	switch (statis_buf.type) {
		case STATIS_INIT:
			if (ch->enable) {
				statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
				statis_param.param = &statis_buf;
				statis_param.buf_size = &module->reserved_size;
				ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
 			} else {
				ch = &module->channel[CAM_CH_CAP];
				if (ch->enable && module->cam_uinfo.is_longexp && !is_raw_cap) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					statis_param.buf_size = &module->reserved_size;
					ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				}
				if (ch->enable && is_raw_cap) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					statis_param.buf_size = &module->reserved_size;
					if (g_dbg_raw2frgb_switch == DEBUG_FRGB_MODE) {
						ret = CAM_PIPEINE_DCAM_OFFLINE_RAW2FRGB_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
						ret = CAM_PIPEINE_DCAM_OFFLINE_FRGB2YUV_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
					} else
						ret = CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				}
			}

			/* N6: frgb hist in dcam; others: yuv hist in isp */
			if (!hw->ip_isp->isphw_abt->frbg_hist_support || hw->ip_isp->isphw_abt->rgb_gtm_support) {
				ch = &module->channel[CAM_CH_PRE];
				if (ch->enable)
					ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, ISP_NODE_MODE_PRE_ID, &statis_buf);
			}
			break;
		case STATIS_AEM:
		case STATIS_AFM:
		case STATIS_AFL:
		case STATIS_HIST:
		case STATIS_PDAF:
		case STATIS_EBD:
		case STATIS_LSCM:
			pr_debug("cam%d type %d mfd %d\n", module->idx, statis_buf.type, statis_buf.mfd);
			if (ch->enable) {
				statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
				statis_param.param = &statis_buf;
				ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
			} else {
				ch = &module->channel[CAM_CH_CAP];
				if (ch->enable && module->cam_uinfo.is_longexp && !is_raw_cap) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				}
				if (ch->enable && is_raw_cap) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					ret = CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				}
			}
			break;
		case STATIS_GTMHIST:
			if (hw->ip_isp->isphw_abt->rgb_gtm_support > 0) {
				if (!ch->enable) {
					pr_debug("prev channel disable, can not set gtm buf\n");
					break;
				}
				/*gtm in isp*/
				ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, ISP_NODE_MODE_PRE_ID, &statis_buf);
			} else {
				/*gtm in dcam*/
				pr_debug("dcam gtm statis buf cfg\n");
				if (ch->enable) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				} else {
					ch = &module->channel[CAM_CH_CAP];
					if (ch->enable && module->cam_uinfo.is_longexp && !is_raw_cap) {
						statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
						statis_param.param = &statis_buf;
						ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
					}
					if (ch->enable && is_raw_cap) {
						statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
						statis_param.param = &statis_buf;
						ret = CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
					}
				}
			}
			break;
		case STATIS_HIST2:
			if (hw->ip_isp->isphw_abt->frbg_hist_support) {
				if (ch->enable) {
					statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
					statis_param.param = &statis_buf;
					ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
				} else {
					ch = &module->channel[CAM_CH_CAP];
					if (ch->enable && module->cam_uinfo.is_longexp && !is_raw_cap) {
						statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
						statis_param.param = &statis_buf;
						ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
					}
					if (ch->enable && is_raw_cap) {
						statis_param.statis_cmd = DCAM_IOCTL_CFG_STATIS_BUF;
						statis_param.param = &statis_buf;
						ret = CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
					}
				}
			} else {
				if (ch->enable)
					ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, ISP_NODE_MODE_PRE_ID, &statis_buf);
			}
			break;
		default:
			break;
	}

exit:
	return ret;
}

static int camioctl_param_cfg(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	int for_capture = 0, for_fdr = 0;
	uint32_t property_param = 0;
	uint32_t node_id = 0;
	struct channel_context *channel = NULL;
	struct isp_io_param param = {0};
	struct compat_isp_io_param __user *uparam = NULL;
	struct dcam_pipe_dev *dev = NULL;
	struct dcam_isp_k_block *pm = NULL;
	func_cam_cfg_param cfg_fun_ptr = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_cfg_block blk_type = {0};
	struct cam_hw_block_func_get blk_func = {0};

	if (module->compat_flag) {
		uparam = (struct compat_isp_io_param __user *)arg;
		ret |= get_user(param.scene_id, &uparam->scene_id);
		ret |= get_user(param.sub_block, &uparam->sub_block);
		ret |= get_user(param.property, &uparam->property);
		ret |= get_user(property_param, &uparam->property_param);
		param.property_param = compat_ptr(property_param);
	} else
		ret = copy_from_user((void *)&param, (void *)arg, sizeof(struct isp_io_param));

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	blk_type.sub_block = param.sub_block;
	blk_type.property = param.property;
	pr_debug("cfg param for sub_block %d\n", param.sub_block);

	if (atomic_read(&module->state) == CAM_INIT || param.property_param == NULL
		|| module->dcam_dev_handle == NULL || module->isp_dev_handle == NULL) {
		pr_err("fail to get user param. state %d\n", atomic_read(&module->state));
		ret = -EFAULT;
		goto exit;
	}

	dev = (struct dcam_pipe_dev *)module->dcam_dev_handle;
	hw = dev->hw;

	if ((param.scene_id == PM_SCENE_OFFLINE_CAP) ||
		(param.scene_id == PM_SCENE_OFFLINE_BPC) ||
		(param.scene_id == PM_SCENE_SFNR))
		for_fdr = 1;
	for_capture = (param.scene_id == PM_SCENE_CAP ? 1 : 0) | for_fdr;

	if (for_capture && (module->channel[CAM_CH_CAP].enable == 0)) {
		pr_warn("warning: ch scene_id[%d] ch_cap en[%d] ch_pre en[%d]\n",
		param.scene_id,
		module->channel[CAM_CH_CAP].enable,
		module->channel[CAM_CH_PRE].enable);

		return 0;
	}

	if (atomic_read(&module->state) == CAM_STREAM_OFF)
		return 0;

	if (param.scene_id == PM_SCENE_PRE) {
		channel = &module->channel[CAM_CH_PRE];
		if (!channel->enable && module->cam_uinfo.is_longexp)
			channel = &module->channel[CAM_CH_CAP];
	} else
		channel = &module->channel[CAM_CH_CAP];

	if ((module->capture_type == CAM_CAPTURE_RAWPROC) && (param.sub_block == DCAM_BLOCK_LSCM
		|| param.sub_block == DCAM_BLOCK_AEM || param.sub_block == DCAM_BLOCK_BAYERHIST)){
		channel = &module->channel[CAM_CH_CAP];
		for_capture = 1;
	}

	ret = hw->cam_ioctl(hw, CAM_HW_GET_BLOCK_TYPE, &blk_type);
	if (unlikely(ret)) {
		pr_err("fail to get block_type, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	if (!channel->enable || !channel->pipeline_handle) {
		pr_warn("warning: channel %d has not been enable %px scene %d subblock %d\n",
				channel->ch_id, channel->pipeline_handle, param.scene_id, param.sub_block);
		if (param.sub_block == DCAM_BLOCK_LSC)
			goto exit;
		if (!for_capture) {
			pm = &channel->blk_pm;
			blk_func.index = param.sub_block;
			if(hw->ip_isp->isphw_abt->rgb_gtm_support && (blk_func.index == DCAM_BLOCK_GTM))
				goto exit;
			if (blk_type.block_type == DCAM_BLOCK_TYPE) {
				hw->cam_ioctl(hw, CAM_HW_GET_BLK_FUN, &blk_func);
				if (blk_func.cam_entry != NULL &&
					blk_func.cam_entry->sub_block == param.sub_block) {
					cfg_fun_ptr = blk_func.cam_entry->cfg_func;
				} else { /* if not, some error */
					pr_err("fail to check param, io_param->sub_block = %d, error\n", param.sub_block);
				}
			}
			if (cfg_fun_ptr == NULL) {
				pr_debug("block %d not supported.\n", param.sub_block);
				goto exit;
			}
			cfg_fun_ptr(&param, pm);
		}
		ret = -EFAULT;
		goto exit;
	}
	node_id = (channel->ch_id == CAM_CH_CAP) ? ISP_NODE_MODE_CAP_ID : ISP_NODE_MODE_PRE_ID;
	switch (blk_type.block_type) {
	case DCAM_BLOCK_TYPE:
		if (blk_type.sub_block == ISP_BLOCK_RGB_GTM) {
			param.sub_block = ISP_BLOCK_RGB_GTM;
			if (channel->enable) {
				if (for_capture && (module->capture_type == CAM_CAPTURE_RAWPROC
					|| module->cam_uinfo.dcam_slice_mode || module->cam_uinfo.is_4in1))
					pr_warn("not support offline ctx now\n");
				else
					ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
			}
			goto exit;
		}
		if (for_capture && (module->capture_type == CAM_CAPTURE_RAWPROC
			|| module->cam_uinfo.dcam_slice_mode || module->cam_uinfo.is_4in1
			|| (param.scene_id == PM_SCENE_OFFLINE_BPC)
			|| (param.scene_id == PM_SCENE_OFFLINE_CAP)
			|| (param.scene_id == PM_SCENE_SFNR))) {
			if (g_dbg_raw2frgb_switch == DEBUG_FRGB_MODE) {
				ret = CAM_PIPEINE_DCAM_OFFLINE_RAW2FRGB_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
				ret = CAM_PIPEINE_DCAM_OFFLINE_FRGB2YUV_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
			} else
				ret = CAM_PIPEINE_DCAM_OFFLINE_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
		} else
			ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
		break;
	case ISP_BLOCK_TYPE:
		if (blk_type.sub_block == ISP_BLOCK_RGB_GTM)
			param.sub_block = ISP_BLOCK_RGB_GTM;
		ret = CAM_PIPEINE_ISP_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, node_id, &param);
		break;
	case DEC_BLOCK_TYPE:
		if (channel->enable) {
			if (param.scene_id == PM_SCENE_PRE)
				goto exit;
			ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(channel, CAM_PIPELINE_CFG_BLK_PARAM, &param);
		}
		break;
	default:
		break;
	}
exit:
	return ret;
}

static int camioctl_function_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_function_mode __user *uparam;
	struct cam_hw_info *hw = NULL;

	uparam = (struct sprd_img_function_mode __user *)arg;
	hw = module->grp->hw_info;
	ret |= get_user(module->cam_uinfo.is_4in1, &uparam->need_4in1);
	ret |= get_user(module->cam_uinfo.is_dual, &uparam->dual_cam);
	ret |= get_user(module->cam_uinfo.is_raw_alg, &uparam->is_raw_alg);
	ret |= get_user(module->cam_uinfo.algs_type, &uparam->raw_alg_type);
	ret |= get_user(module->cam_uinfo.param_frame_sync, &uparam->param_frame_sync);
	ret |= get_user(module->cam_uinfo.zoom_conflict_with_ltm, &uparam->zoom_conflict_with_ltm);
	ret |= get_user(module->cam_uinfo.need_dcam_raw, &uparam->need_dcam_raw);
	ret |= get_user(module->master_flag, &uparam->master_flag);
	ret |= get_user(module->cam_uinfo.virtualsensor, &uparam->virtualsensor);
	ret |= get_user(module->cam_uinfo.opt_buffer_num, &uparam->opt_buffer_num);
	if (unlikely(ret)) {
		pr_err("fail to get from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	module->cam_uinfo.is_rgb_ltm = hw->ip_isp->isphw_abt->rgb_ltm_support;
	module->cam_uinfo.is_rgb_gtm = hw->ip_isp->isphw_abt->rgb_gtm_support;
	if (g_pyr_dec_offline_bypass || module->channel[CAM_CH_CAP].ch_uinfo.is_high_fps
		|| module->cam_uinfo.sn_size.w > DCAM_64M_WIDTH)
		module->cam_uinfo.is_pyr_dec = 0;
	else
		module->cam_uinfo.is_pyr_dec = hw->ip_isp->isphw_abt->pyr_dec_support;

	if (g_pyr_dec_online_bypass || (module->cam_uinfo.sensor_if.img_fmt == DCAM_CAP_MODE_YUV))
		module->cam_uinfo.is_pyr_rec = 0;
	else
		module->cam_uinfo.is_pyr_rec = hw->ip_dcam[0]->dcamhw_abt->pyramid_support;

	pr_info("4in1:[%d], rgb_ltm[%d], gtm[%d], dual[%d], dec %d, raw_alg_type:%d, zoom_conflict_with_ltm %d, %d. dcam_raw %d, master_flag:%d,virtualsensor %d pre buffer num %d\n",
		module->cam_uinfo.is_4in1,module->cam_uinfo.is_rgb_ltm,
		module->cam_uinfo.is_rgb_gtm,
		module->cam_uinfo.is_dual, module->cam_uinfo.is_pyr_dec,
		module->cam_uinfo.algs_type,
		module->cam_uinfo.zoom_conflict_with_ltm,
		module->cam_uinfo.is_raw_alg,
		module->cam_uinfo.need_dcam_raw,
		module->master_flag,
		module->cam_uinfo.virtualsensor,
		module->cam_uinfo.opt_buffer_num);
exit:
	return ret;
}

static int camioctl_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;

	ret = copy_from_user(&module->cam_uinfo.capture_mode,
			(void __user *)arg, sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_info("mode %d\n", module->cam_uinfo.capture_mode);

exit:
	return ret;
}

static int camioctl_cam_security_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int sec_ret = 0;
	enum en_status vaor_bp_en = DISABLE;
	int ca_conn = 0;
	struct sprd_cam_sec_cfg uparam = {0};

	ret = copy_from_user(&uparam, (void __user *)arg, sizeof(struct sprd_cam_sec_cfg));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	if (!module->isp_dev_handle) {
		pr_err("fail to isp dev_hanlde is NULL\n");
		ret = -EFAULT;
		goto exit;
	}

	if (atomic_read(&module->state) == CAM_INIT) {
		pr_err("cam%d error state: %d\n", module->idx, atomic_read(&module->state));
		return -EFAULT;
	}

	pr_info("camca : conn = %d, security mode %d,  u camsec_mode=%d, u work_mode=%d\n",
		module->grp->ca_conn, module->grp->camsec_cfg.camsec_mode,
		uparam.camsec_mode, uparam.work_mode);

	if (uparam.camsec_mode == SEC_UNABLE) {
		vaor_bp_en = DISABLE;
	}  else {
		if (!module->grp->ca_conn) {
			ca_conn = cam_trusty_connect();
			if (!ca_conn) {
				pr_err("fail to init cam_trusty_connect\n");
				ret = -EINVAL;
				goto exit;
			}
			module->grp->ca_conn = ca_conn;
		}

		sec_ret = cam_trusty_security_set(&uparam, CAM_TRUSTY_ENTER);
		if (!sec_ret) {
			ret = -EINVAL;
			pr_err("fail to init cam security set\n");
			goto exit;
		}
		vaor_bp_en = ENABLE;
	}

	module->grp->camsec_cfg.work_mode = uparam.work_mode;
	module->grp->camsec_cfg.camsec_mode = uparam.camsec_mode;
	if (atomic_read(&module->isp_dev_handle->pd_clk_rdy) > 0)
		ret = sprd_iommu_set_cam_bypass(vaor_bp_en);

exit:
	return ret;
}

static int camioctl_sensor_if_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_sensor_if *dst = NULL;

	dst = &module->cam_uinfo.sensor_if;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(struct sprd_img_sensor_if));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_info("sensor_if %d %x %x, %d.....mipi %d %d %d %d\n",
		dst->if_type, dst->img_fmt, dst->img_ptn, dst->frm_deci,
		dst->if_spec.mipi.use_href, dst->if_spec.mipi.bits_per_pxl,
		dst->if_spec.mipi.is_loose, dst->if_spec.mipi.lane_num);

exit:
	return ret;
}

static int camioctl_sensor_size_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct img_size *dst = NULL;

	dst = &module->cam_uinfo.sn_size;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(struct img_size));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_info("cam%d, sensor_size %d %d\n", module->idx, dst->w, dst->h);
	module->cam_uinfo.dcam_slice_mode = dst->w > DCAM_24M_WIDTH ? CAM_OFFLINE_SLICE_HW : 0;

exit:
	return ret;
}

static int camioctl_sensor_trim_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_rect *dst = NULL;

	dst = &module->cam_uinfo.sn_rect;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(struct sprd_img_rect));
	if (unlikely(ret || (dst->w | dst->h) & (DCAM_MIPI_CAP_ALIGN - 1))) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_info("sensor_trim %d %d %d %d\n", dst->x, dst->y, dst->w, dst->h);
	/* make sure MIPI CAP size is 4 pixel aligned */

exit:
	return ret;
}

static int camioctl_sensor_max_size_set(
		struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct img_size *dst = NULL;

	dst = &module->cam_uinfo.sn_max_size;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(struct img_size));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_info("sensor_max_size %d %d\n", dst->w, dst->h);

exit:
	return ret;
}

static int camioctl_cap_skip_num_set(	struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t *dst = NULL;

	dst = &module->cam_uinfo.capture_skip;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	pr_debug("set cap skip frame %d\n", *dst);
	return 0;
}

static int camioctl_output_size_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t scene_mode = 0;
	uint32_t cap_type = 0;
	uint32_t sn_fmt, dst_fmt = 0;
	struct channel_context *channel = NULL;
	struct camera_uchannel *dst = NULL;
	struct sprd_img_parm __user *uparam = NULL;

	module->last_channel_id = CAM_CH_MAX;
	uparam = (struct sprd_img_parm __user *)arg;

	ret |= get_user(scene_mode, &uparam->scene_mode);
	ret |= get_user(cap_type, &uparam->need_isp_tool);
	ret |= get_user(sn_fmt, &uparam->sn_fmt);
	ret |= get_user(dst_fmt, &uparam->pixel_fmt);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		goto exit;
	}

	pr_info("cam%d, scene %d  cap_type %d, fmt %x %x\n",
		module->idx, scene_mode, cap_type, sn_fmt, dst_fmt);

	if (dst_fmt == IMG_PIX_FMT_GREY && cap_type != CAM_CAP_RAW_FULL && scene_mode != DCAM_SCENE_MODE_SENSOR_RAW)
		module->raw_callback = 1;
	else
		module->raw_callback = 0;
	pr_debug("raw_callback=%d\n", module->raw_callback);

	if ((scene_mode == DCAM_SCENE_MODE_SENSOR_RAW) &&
		(module->channel[CAM_CH_DCAM_VCH].enable == 0)) {
		channel = &module->channel[CAM_CH_DCAM_VCH];
		channel->enable = 1;
	} else if (((cap_type == CAM_CAP_RAW_FULL) || (dst_fmt == sn_fmt)) &&
		(module->channel[CAM_CH_RAW].enable == 0)) {
		channel = &module->channel[CAM_CH_RAW];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_PREVIEW) &&
		(module->channel[CAM_CH_PRE].enable == 0)) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if (((scene_mode == DCAM_SCENE_MODE_RECORDING) ||
		(scene_mode == DCAM_SCENE_MODE_CAPTURE_CALLBACK)) &&
		(module->channel[CAM_CH_VID].enable == 0)) {
		channel = &module->channel[CAM_CH_VID];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_CAPTURE) &&
		(module->channel[CAM_CH_CAP].enable == 0)) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_CAPTURE_THUMB) &&
		(module->channel[CAM_CH_CAP_THM].enable == 0)) {
		channel = &module->channel[CAM_CH_CAP_THM];
		channel->enable = 1;
	} else if ((scene_mode == DCAM_SCENE_MODE_VIRTUAL) &&
		(module->channel[CAM_CH_VIRTUAL].enable == 0)) {
		channel = &module->channel[CAM_CH_VIRTUAL];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_PRE].enable == 0) {
		channel = &module->channel[CAM_CH_PRE];
		channel->enable = 1;
	} else if (module->channel[CAM_CH_CAP].enable == 0) {
		channel = &module->channel[CAM_CH_CAP];
		channel->enable = 1;
	}

	if (channel == NULL) {
		pr_err("fail to get valid channel\n");
		ret = -EINVAL;
		goto exit;
	}

	module->last_channel_id = channel->ch_id;
	channel->dcam_port_id = -1;
	channel->aux_dcam_port_id = -1;
	channel->isp_port_id = -1;

	dst = &channel->ch_uinfo;
	dst->dcam_raw_fmt = -1;
	dst->sensor_raw_fmt = -1;
	dst->sn_fmt = sn_fmt;
	dst->dst_fmt = dst_fmt;
	ret |= get_user(dst->is_high_fps, &uparam->is_high_fps);
	ret |= get_user(dst->high_fps_skip_num, &uparam->high_fps_skip_num);

	ret |= copy_from_user(&dst->zoom_ratio_base,
			&uparam->base_rect, sizeof(struct sprd_img_rect));
	ret |= copy_from_user(&dst->src_crop,
			&uparam->crop_rect, sizeof(struct sprd_img_rect));
	ret |= copy_from_user(&dst->dst_size,
			&uparam->dst_size, sizeof(struct img_size));
	ret |= copy_from_user(&channel->ch_uinfo.dst_size_tmp,
			&uparam->dst_size, sizeof(struct img_size));
	ret |= get_user(dst->frame_sync_close, &uparam->reserved[3]);

	if (cap_type == CAM_CAP_RAW_FULL && dst->is_high_fps)
		dst->is_high_fps = 0;

	if (!dst->is_high_fps)
		dst->high_fps_skip_num = 0;

	if (dst->high_fps_skip_num == 1) {
		pr_err("fail to get valid high fps %u\n", dst->high_fps_skip_num);
		ret = -EINVAL;
	}

	if (channel->ch_id == CAM_CH_VIRTUAL) {
		ret |= get_user(dst->vir_channel[0].dump_isp_out_fmt, &uparam->vir_ch_info[0].dump_isp_out_fmt);
		ret |= copy_from_user(&dst->vir_channel[0].dst_size,
				&uparam->vir_ch_info[0].dst_size, sizeof(struct img_size));
		ret |= get_user(dst->vir_channel[1].dump_isp_out_fmt, &uparam->vir_ch_info[1].dump_isp_out_fmt);
		ret |= copy_from_user(&dst->vir_channel[1].dst_size,
				&uparam->vir_ch_info[1].dst_size, sizeof(struct img_size));
		pr_debug("cam_virtual_pre: dst %d %d\n", dst->vir_channel[0].dst_size.w, dst->vir_channel[0].dst_size.h);
		pr_debug("cam_virtual_cap: dst %d %d\n", dst->vir_channel[1].dst_size.w, dst->vir_channel[1].dst_size.h);
	}
	/* for AF zoom_ratio cal*/
	cam_zoom_crop_size_align(module, &dst->zoom_ratio_base, channel->ch_id);

	pr_info("cam_channel: ch_id %d high fps %u %u\n",
		channel->ch_id, dst->is_high_fps, dst->high_fps_skip_num);
	pr_info("cam_channel: zoom ratio base %d %d %d %d dst %d %d\n",
		dst->zoom_ratio_base.x,dst->zoom_ratio_base.y,
		dst->zoom_ratio_base.w,dst->zoom_ratio_base.h,
		dst->dst_size.w, dst->dst_size.h);

exit:
	return ret;
}

static int camioctl_ch_id_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;

	if ((atomic_read(&module->state) != CAM_IDLE) &&
		(atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_err("fail to get correct state, state %d\n",
			atomic_read(&module->state));
		return -EFAULT;
	}

	if (module->last_channel_id >= CAM_CH_MAX) {
		ret = -EINVAL;
		goto exit;
	}
	pr_info("cam_channel: get ch id: %d\n", module->last_channel_id);

	ret = copy_to_user((void __user *)arg, &module->last_channel_id, sizeof(uint32_t));
	if (unlikely(ret))
		pr_err("fail to copy to user. ret %d\n", ret);

exit:
	atomic_set(&module->state, CAM_CFG_CH);
	return ret;
}

static int camioctl_dcam_path_size(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_dcam_path_size param = {0};

	ret = copy_from_user(&param, (void __user *)arg, sizeof(struct sprd_dcam_path_size));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	param.dcam_out_w = param.dcam_in_w;
	param.dcam_out_h = param.dcam_in_h;

	if (atomic_read(&module->timeout_flag) == 1)
		pr_info("cam%d, in %d  %d. pre %d %d, vid %d, %d, out %d %d\n",
			module->idx, param.dcam_in_w, param.dcam_in_h,
			param.pre_dst_w, param.pre_dst_h,
			param.vid_dst_w, param.vid_dst_h,
			param.dcam_out_w, param.dcam_out_h);

	ret = copy_to_user((void __user *)arg, &param, sizeof(struct sprd_dcam_path_size));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
	}

exit:
	return ret;
}

static int camioctl_shrink_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id = 0;
	struct sprd_img_parm __user *uparam = NULL;

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_debug("skip\n");
		return ret;
	}

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	if (ret == 0 && channel_id < CAM_CH_MAX) {
		ret = copy_from_user(
			&module->channel[channel_id].ch_uinfo.regular_desc,
			(void __user *)&uparam->regular_desc,
			sizeof(struct dcam_regular_desc));
	}

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

 static int camioctl_ebd_control(struct camera_module *module,
		unsigned long arg)
{
	/* TBD: embedline */
	return 0;
}

static int camioctl_crop_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id = 0;
	struct channel_context *ch = NULL, *ch_vid = NULL;
	struct sprd_img_rect crop = {0};
	struct sprd_img_rect total_crop = {0};
	struct cam_frame *first_param = NULL;
	struct cam_frame *zoom_param = NULL;
	struct sprd_img_parm __user *uparam = NULL;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warning: module state: %d\n", atomic_read(&module->state));
		return 0;
	}

	uparam = (struct sprd_img_parm __user *)arg;

	ret = get_user(channel_id, &uparam->channel_id);
	if (ret || (channel_id >= CAM_CH_MAX)) {
		pr_err("fail to set crop, ret %d, ch %d\n", ret, channel_id);
		ret = -EINVAL;
		goto exit;
	}

	ch = &module->channel[channel_id];
	ch_vid = &module->channel[CAM_CH_VID];
	if (!ch->enable) {
		pr_err("fail to set crop, ret %d, ch %d\n", ret, channel_id);
		ret = -EINVAL;
		goto exit;
	}

	ret = copy_from_user(&crop, (void __user *)&uparam->crop_rect, sizeof(struct sprd_img_rect));
	if (unlikely(ret)) {
		pr_err("fail to copy crop from user, ret %d\n", ret);
		goto exit;
	}

	ret = copy_from_user(&total_crop, (void __user *)&uparam->total_rect, sizeof(struct sprd_img_rect));
	if (unlikely(ret)) {
		pr_err("fail to copy total_crop from user, ret %d\n", ret);
		goto exit;
	}

	pr_info("cam%d 4in1[%d],set ch%d crop %d %d %d %d.\n",
		module->idx, module->cam_uinfo.is_4in1, channel_id,
		crop.x, crop.y, crop.w, crop.h);
	pr_debug("cam%d total,set ch%d crop %d %d %d %d.\n",
		module->idx, channel_id,
		total_crop.x, total_crop.y, total_crop.w, total_crop.h);
	cam_zoom_crop_size_align(module, &crop, channel_id);

	/* CAM_RUNNING: for zoom update
	 * only crop rect size can be re-configured during zoom
	 * and it is forbidden during capture.
	 */
	if (atomic_read(&module->state) == CAM_RUNNING) {
		if ((module->capture_type != CAM_CAPTURE_STOP) &&
			module->channel[CAM_CH_CAP].enable &&
			(module->channel[CAM_CH_CAP].type_3dnr != CAM_3DNR_OFF)) {
			pr_err("fail to zoom during 3DNR capture\n");
			goto exit;
		}
		zoom_param = cam_queue_empty_frame_get(CAM_FRAME_NODE_ZOOM);
		zoom_param->user_zoom.zoom_crop = crop;
		zoom_param->user_zoom.total_zoom_crop = total_crop;
		ch->latest_user_crop = crop;
		if (CAM_QUEUE_ENQUEUE(&ch->zoom_user_crop_q, &zoom_param->list)) {
			/* if zoom queue overflow, discard first one node in queue*/
			pr_warn("warning: ch %d zoom q overflow cnt:%d, state:%d, max:%d\n", channel_id, ch->zoom_user_crop_q.cnt,
				ch->zoom_user_crop_q.state, ch->zoom_user_crop_q.max);
			first_param = CAM_QUEUE_DEQUEUE(&ch->zoom_user_crop_q, struct cam_frame, list);
			cam_queue_empty_frame_put(first_param);
			if (CAM_QUEUE_ENQUEUE(&ch->zoom_user_crop_q, &zoom_param->list))
				goto exit;
		}
		if (ch_vid->enable && channel_id == CAM_CH_PRE) {
			pr_debug("ch_vid->enable %d channel_id %d\n", ch_vid->enable, channel_id);
			goto exit;
		}
		complete(&module->zoom_thrd.thread_com);
	} else {
		ch->ch_uinfo.src_crop = crop;
		ch->ch_uinfo.total_src_crop = total_crop;
		ch->latest_user_crop = crop;
	}

exit:
	return ret;
}

/* TBD: This function need merge into drv capability to HAL */
static int camioctl_fmt_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_get_fmt fmt_desc = {0};

	ret = copy_from_user(&fmt_desc, (void __user *)arg,
		sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to copy from user ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	if (unlikely(fmt_desc.index >= ARRAY_SIZE(output_img_fmt))) {
		pr_err("fail to get valid index > arrar size\n");
		ret = -EINVAL;
		goto exit;
	}

	fmt_desc.fmt = output_img_fmt[fmt_desc.index];

	ret = copy_to_user((void __user *)arg,
		&fmt_desc, sizeof(struct sprd_img_get_fmt));
	if (unlikely(ret)) {
		pr_err("fail to put user info, GET_FMT, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
exit:
	return ret;
}

static int camioctl_fmt_check(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	enum cam_ch_id channel_id = 0;
	struct sprd_img_parm __user *uparam = NULL;
	struct channel_context *channel = NULL;

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_debug("skip\n");
		return ret;
	}

	pr_debug("check fmt\n");
	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(channel_id, &uparam->channel_id);
	if (ret) {
		pr_err("fail to get channel_id\n");
		return -EFAULT;
	}

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_err("fail to get module state: %d\n", atomic_read(&module->state));
		return -EFAULT;
	}

	channel = &module->channel[channel_id];

	if (atomic_read(&module->state) == CAM_CFG_CH) {
		pr_info("chk_fmt ch %d\n", channel_id);
		ret = camcore_pipeline_init(module, channel);
		if (ret) {
			/* todo: error handling. */
			pr_err("fail to init pipeline %d\n", channel->ch_id);
			goto exit;
		}
	}

exit:
	return ret;
}

/*
 * Now isp preview path skip num control use reserved buf.
 * This frm deci no need to use, if we want to power consumption in
 * slow motion scene, dynamic control for isp preview path may needed,
 * but in this case, high fps & skip num info is enought, so this ioctl
 * can delete when opt with HAL.
 */
static int camioctl_frm_deci_set(struct camera_module *module,
		unsigned long arg)
{
	return 0;
}

static int camioctl_frame_addr_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i = 0, sec = 0;
	uint32_t channel_id = 0, buffer_count = 0;
	uint32_t is_reserved_buf = 0, pixel_fmt = 0, node_id = 0;
	uint32_t dump_type[IMG_PATH_BUFFER_COUNT] = {0};
	struct cam_hw_info *hw = NULL;
	struct cam_frame *pframe = NULL;
	struct sprd_img_parm __user *uparam = NULL;
	struct channel_context *ch = NULL, *ch_pre = NULL, *ch_cap = NULL;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warning: only for state CFG_CH or RUNNING\n");
		return 0;
	}

	hw = module->grp->hw_info;
	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(channel_id, &uparam->channel_id);
	ret |= get_user(buffer_count, &uparam->buffer_count);
	ret |= get_user(is_reserved_buf, &uparam->is_reserved_buf);
	ret |= get_user(pixel_fmt, &uparam->pixel_fmt);

	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	if ((channel_id >= CAM_CH_MAX) || (buffer_count == 0) ||
		(module->channel[channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d. buf cnt %d\n",
			channel_id, buffer_count);
		return -EFAULT;
	}

	pr_debug("ch %d, buffer_count %d is_reserved_buf %d\n", channel_id, buffer_count, is_reserved_buf);

	ch = &module->channel[channel_id];
	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];

	for (i = 0; i < buffer_count; i++) {
		if (is_reserved_buf) {
			get_user(module->reserved_buf_fd, &uparam->fd_array[i]);
			continue;
		}

		pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
		pframe->common.buf.type = CAM_BUF_USER;
		pframe->common.channel_id = ch->ch_id;
		pframe->common.img_fmt = ch->ch_uinfo.dst_fmt;
		pframe->common.buf.status = CAM_BUF_ALLOC;
		ret |= get_user(pframe->common.buf.mfd, &uparam->fd_array[i]);
		ret |= get_user(pframe->common.buf.offset[0], &uparam->frame_addr_array[i].y);
		ret |= get_user(pframe->common.buf.offset[1], &uparam->frame_addr_array[i].u);
		ret |= get_user(pframe->common.buf.offset[2], &uparam->frame_addr_array[i].v);
		ret |= get_user(pframe->common.user_fid, &uparam->user_fid);
		ret |= get_user(pframe->common.buf.addr_vir[0], &uparam->frame_addr_vir_array[i].y);
		ret |= get_user(pframe->common.buf.addr_vir[1], &uparam->frame_addr_vir_array[i].u);
		ret |= get_user(pframe->common.buf.addr_vir[2], &uparam->frame_addr_vir_array[i].v);
		ret |= get_user(dump_type[i], &uparam->vir_ch_info[0].fd_array[i]);
		ret |= get_user(sec, &uparam->img_statis_info.sec);

		if (unlikely(ret)) {
			pr_err("fail to copy from user, ret %d\n", ret);
			ret = -EFAULT;
			break;
		}

		ret = camcore_faceid_secbuf(sec, &pframe->common.buf, module->grp->hw_info);
		if (unlikely(ret))
			break;

		pr_debug("cam%d ch %d, mfd 0x%x, off 0x%lx 0x%lx 0x%lx, reserved %d user_fid[%d]\n",
			module->idx, pframe->common.channel_id, pframe->common.buf.mfd,
			pframe->common.buf.offset[0], pframe->common.buf.offset[1],
			pframe->common.buf.offset[2], is_reserved_buf,
			pframe->common.user_fid);

		if (channel_id == CAM_CH_CAP)
			pr_info("ch %d, mfd 0x%x, off 0x%lx 0x%lx 0x%lx, size 0x%x, reserved %d, buffer_cnt %d\n",
				pframe->common.channel_id, pframe->common.buf.mfd,
				pframe->common.buf.offset[0],pframe->common.buf.offset[1], pframe->common.buf.offset[2],
				(uint32_t)pframe->common.buf.size, is_reserved_buf, buffer_count);

		if (ch->isp_port_id >= 0 && pixel_fmt != IMG_PIX_FMT_GREY) {
			if (channel_id == CAM_CH_VIRTUAL) {
				if ((dump_type[i] == DUMP_CH_PRE) && (ch_pre->enable)) {
					pframe->common.height = ch->ch_uinfo.vir_channel[0].dst_size.h;
					ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch_pre, PORT_VID_OUT, CAM_PIPELINE_CFG_BUF, ISP_NODE_MODE_PRE_ID, pframe);
				} else if ((dump_type[i] == DUMP_CH_CAP) && (ch_cap->enable)) {
					pframe->common.height = ch->ch_uinfo.vir_channel[1].dst_size.h;
					ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch_cap, PORT_VID_OUT, CAM_PIPELINE_CFG_BUF, ISP_NODE_MODE_CAP_ID, pframe);
				}
			} else	{
				node_id = (ch->ch_id == CAM_CH_CAP) ? ISP_NODE_MODE_CAP_ID : ISP_NODE_MODE_PRE_ID;
				ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, node_id, pframe);
			}
		} else {
			if ((ch->ch_id == CAM_CH_CAP) && pixel_fmt == IMG_PIX_FMT_GREY)
				pframe->common.img_fmt = pixel_fmt;
			if (module->cam_uinfo.is_raw_alg && channel_id != CAM_CH_DCAM_VCH) {
				pframe->common.irq_property = CAM_FRAME_ORIGINAL_RAW;
				ret = CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(ch, dcamonline_pathid_convert_to_portid(DCAM_PATH_RAW),
					CAM_PIPELINE_CFG_BUF, pframe);
			} else if (ch->ch_id == CAM_CH_RAW && module->cam_uinfo.need_dcam_raw && hw->ip_isp->isphw_abt->fetch_raw_support && ch_cap->enable){
				if (ch->ch_uinfo.dst_size.w == ch_cap->ch_uinfo.dst_size.w) {
					pr_debug("cap path copy\n");
					ret = CAM_PIPEINE_DATA_COPY_NODE_CFG(ch_cap, CAM_PIPELINE_CFG_BUF, pframe);
				} else {
					pr_debug("pre path copy\n");
					ret = CAM_PIPEINE_DATA_COPY_NODE_CFG(ch_pre, CAM_PIPELINE_CFG_BUF, pframe);
				}
			} else
				ret = CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(ch, ch->dcam_port_id, CAM_PIPELINE_CFG_BUF, pframe);
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n", ch->ch_id);
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}
	}

	return ret;
}

static int camioctl_frame_id_base_set(	struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t channel_id = 0, frame_id_base = 0;
	struct channel_context *ch = NULL;
	struct sprd_img_parm __user *uparam = NULL;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warning: only for state CFG_CH or RUNNING\n");
		return 0;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret = get_user(channel_id, &uparam->channel_id);
	ret |= get_user(frame_id_base, &uparam->frame_base_id);
	if (ret) {
		pr_err("fail to get from user. ret %d\n", ret);
		return -EFAULT;
	}
	if ((channel_id >= CAM_CH_MAX) ||
		(module->channel[channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d\n", channel_id);
		return -EFAULT;
	}

	ch = &module->channel[channel_id];
	ch->frm_base_id = frame_id_base;

	return ret;
}

static int camioctl_stream_off(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t i = 0, j = 0;
	uint32_t running = 0;
	struct channel_context *ch = NULL;
	struct cam_hw_info *hw = NULL;
	struct cam_buf_pool_id pool_id = {0};
	struct cam_pipeline_cfg_param pipe_param = {0};
	struct dcam_statis_param statis_param = {0};
	enum dcam_stop_cmd stop_cmd = DCAM_STOP;

	if ((atomic_read(&module->state) != CAM_RUNNING) &&
		(atomic_read(&module->state) != CAM_CFG_CH)) {
		pr_info("cam%d state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	if (atomic_read(&module->state) == CAM_RUNNING)
		running = 1;

	pr_info("cam %d stream off. state: %d\n",module->idx, atomic_read(&module->state));

	camcore_timer_deinit(&module->cam_timer);
	hw = module->grp->hw_info;
	ch = &module->channel[CAM_CH_CAP];
	if (ch) {
		mutex_lock(&module->buf_lock[ch->ch_id]);
		if (ch->enable && ch->alloc_start) {
			wait_for_completion(&ch->alloc_com);
			pr_debug("alloc buffer done.\n");
			ch->alloc_start = 0;
		}
		mutex_unlock(&module->buf_lock[ch->ch_id]);
	}

	atomic_set(&module->state, CAM_STREAM_OFF);
	module->capture_type = CAM_CAPTURE_STOP;

	if (running) {
		for (i = 0; i < CAM_CH_MAX; i++) {
			ch = &module->channel[i];
			if (ch->enable && ch->pipeline_handle) {
				pipe_param.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
				pipe_param.node_param.param = &stop_cmd;
				ret = ch->pipeline_handle->ops.streamoff(ch->pipeline_handle, &pipe_param);
				if (ret)
					pr_err("fail to stop module %d dcam online ret:%d\n", module->idx, ret);
			}
		}
		if (module->dual_frame) {
			cam_queue_empty_frame_put(module->dual_frame);
			module->dual_frame = NULL;
		}
	}

	mutex_lock(&module->zoom_lock);
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable)
			continue;
		CAM_QUEUE_CLEAN(&ch->zoom_user_crop_q, struct cam_frame, list);
		CAM_QUEUE_CLEAN(&ch->zoom_param_q, struct cam_frame, list);
		if ((ch->ch_id == CAM_CH_PRE) || (ch->ch_id == CAM_CH_CAP)) {
			mutex_lock(&module->buf_lock[ch->ch_id]);
			if (ch->alloc_start) {
				wait_for_completion(&ch->alloc_com);
				pr_debug("alloc buffer done.\n");
				ch->alloc_start = 0;
			}
			mutex_unlock(&module->buf_lock[ch->ch_id]);
		}
		if (ch->ch_id == CAM_CH_PRE || (ch->ch_id == CAM_CH_CAP
			&& module->cam_uinfo.is_longexp)) {
			statis_param.statis_cmd = DCAM_IOCTL_DEINIT_STATIS_Q;
			statis_param.param = NULL;
			ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_STATIS_BUF, &statis_param);
			if (ret)
				pr_err("fail to deinit statis q %d\n", ret);
		}
		camcore_pipeline_deinit(module, ch);
	}

	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		memset(ch, 0, sizeof(struct channel_context));
		camcore_channel_default_param_set(ch, i);
	}
	mutex_unlock(&module->zoom_lock);

	if (running) {
		/* wait for read thread take all events in frm_queue,
		 * frm_queue max len is CAM_FRAME_Q_LEN
		 * then we loop for this counter.
		 * if read thread has exited unexpectedly,
		 * queue_clear() will clear frm_queue as well
		 */
		j = CAM_FRAME_Q_LEN;
		while (j--) {
			i = CAM_QUEUE_CNT_GET(&module->frm_queue);
			if (i == 0)
				break;
			pr_info("camera%d wait for read %d %d\n", module->idx, i, j);
			os_adapt_time_msleep(20);
		}
		/* default 0, hal set 1 when needed */
		module->auto_3dnr = 0;
	}

	pool_id.tag_id = 0;
	pool_id.private_pool_id = 0;
	pool_id.reserved_pool_id = module->reserved_pool_id;
	cam_buf_manager_buf_clear(&pool_id, (void *)module->grp->global_buf_manager);
	pool_id.tag_id = CAM_BUF_POOL_ABNORAM_RECYCLE;
	cam_buf_manager_buf_clear(&pool_id, (void *)module->grp->global_buf_manager);
	if (running && atomic_dec_return(&module->grp->runner_nr) == 0) {
		module->isp_dev_handle->isp_ops->reset(module->isp_dev_handle, hw);
		cam_queue_frame_array_check(CAM_FRAME_CHECK_QUEUE_OUT);
	}
	atomic_set(&module->state, CAM_IDLE);

	ret = cam_buf_monitor_mdbg_check();
	pr_info("cam %d stream off done.\n", module->idx);

	return ret;
}

static int camioctl_stream_on(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	uint32_t i = 0;
	uint32_t uframe_sync = 0;
	uint32_t timer = 0;
	uint32_t node_id = 0;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL;
	struct dcam_online_start_param start_param = {0};
	struct cam_hw_lbuf_info cam_lbuf_info = {0};
	struct cam_pipeline_cfg_param param = {0};

	if (atomic_read(&module->state) != CAM_CFG_CH) {
		pr_info("cam%d error state: %d\n", module->idx,
			atomic_read(&module->state));
		return -EFAULT;
	}

	atomic_set(&module->state, CAM_STREAM_ON);
	module->flash_skip_fid = 0;
	module->is_flash_status = 0;
	module->simu_fid = 0;
	ret = cam_zoom_channels_size_init(module);
	cam_zoom_channel_size_calc(module);

	ch_pre = &module->channel[CAM_CH_PRE];
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || !ch->pipeline_handle)
			continue;
		if (i == CAM_CH_VID && ch_pre->enable)
			continue;
		CAM_QUEUE_INIT(&ch->zoom_param_q, CAM_ZOOM_COEFF_Q_LEN, NULL);
		camcore_buffer_channel_config(module, ch);
		cam_zoom_channel_size_config(module, ch);
	}

	if (module->cam_uinfo.opt_buffer_num)
		CAM_PIPEINE_DATA_COPY_NODE_CFG(ch_pre, CAM_PIPELINE_CFG_OPT_SCENE_SWITCH, &module->cam_uinfo.opt_buffer_num);
	ret = camcore_resframe_set(module);
	if (ret) {
		pr_err("fail to set resframe\n");
		goto exit;
	}
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (!ch->enable || (ch->ch_id == CAM_CH_RAW) || (ch->ch_id == CAM_CH_VIRTUAL) || (ch->ch_id == CAM_CH_DCAM_VCH))
			continue;

		uframe_sync = ch->ch_id != CAM_CH_CAP;
		if (ch->ch_uinfo.frame_sync_close)
			uframe_sync = 0;

		node_id = (ch->ch_id == CAM_CH_CAP) ? ISP_NODE_MODE_CAP_ID : ISP_NODE_MODE_PRE_ID;
		CAM_PIPEINE_ISP_IN_PORT_CFG(ch, PORT_ISP_OFFLINE_IN, CAM_PIPELINE_CFG_UFRAME, node_id, &uframe_sync);
		CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_UFRAME, node_id, &uframe_sync);

		CAM_QUEUE_INIT(&ch->zoom_user_crop_q, CAM_ZOOM_COEFF_Q_LEN, NULL);

		if (i == CAM_CH_PRE || (i == CAM_CH_VID && !module->channel[CAM_CH_PRE].enable) || (i == CAM_CH_CAP && !module->channel[CAM_CH_PRE].enable)) {
			ret = camcore_buffer_path_cfg(module, i);
			if (ret) {
				pr_err("fail to cfg path buffer\n");
				goto exit;
			}
		}
	}
	module->dual_frame = NULL;

	/* dcam online node stream on */
	for (i = 0; i < CAM_CH_MAX; i++) {
		ch = &module->channel[i];
		if (ch->enable && ch->pipeline_handle) {
			cam_lbuf_info.is_4in1 = module->cam_uinfo.is_4in1;
			cam_lbuf_info.line_w = module->cam_uinfo.sn_rect.w;
			cam_lbuf_info.is_offline = 0;
			start_param.lbuf_param = &cam_lbuf_info;
			param.node_type = CAM_NODE_TYPE_DCAM_ONLINE;
			param.node_param.param = &start_param;
			param.node_param.port_id = 0xffffffff;
			ret = ch->pipeline_handle->ops.streamon(ch->pipeline_handle, &param);
			if (ret < 0) {
				pr_err("fail to start dcam dev, ret %d\n", ret);
				goto exit;
			}
		}
	}
	module->dcam_ctx_bind_state = 1;
	atomic_set(&module->timeout_flag, 1);
	module->timeout_num = 0;

	if (module->cam_uinfo.is_longexp)
		timer = CAMERA_LONGEXP_TIMEOUT;
	else
		timer = CAMERA_TIMEOUT;
	ret = camcore_timer_start(&module->cam_timer, timer);
	atomic_set(&module->state, CAM_RUNNING);
	atomic_inc(&module->grp->runner_nr);

	pr_info("stream on done cam%d csi %d.\n", module->idx, module->dcam_idx);
	cam_buf_monitor_mdbg_check();
	return 0;

exit:
	atomic_set(&module->state, CAM_CFG_CH);
	pr_info("stream on failed\n");

	camioctl_stream_off(module, 0L);

	return ret;
}

static int camioctl_cam_res_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	int dcam_idx, retry = 1;
	struct sprd_img_res res = {0};
	struct camera_group *grp = module->grp;
	void *dcam = NULL;
	void *isp = NULL;
	struct cam_thread_info *thrd = NULL;

	ret = copy_from_user(&res, (void __user *)arg, sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	if (atomic_read(&module->state) != CAM_INIT) {
		pr_err("fail to get cam%d state: %d\n",
			module->idx, atomic_read(&module->state));
		return -EFAULT;
	}

	thrd = &module->zoom_thrd;
	sprintf(thrd->thread_name, "cam%d_zoom", module->idx);
	ret = camthread_create(module, thrd, cam_zoom_start_proc);
	if (ret)
		goto stop_thrd;

	thrd = &module->buf_thrd;
	sprintf(thrd->thread_name, "cam%d_alloc_buf", module->idx);
	ret = camthread_create(module, thrd, camcore_buffers_alloc);
	if (ret)
		goto stop_thrd;

	pr_info("cam%d get res sensor id %d\n", module->idx, res.sensor_id);

	dcam_idx = -1;
	if (res.sensor_id < SPRD_SENSOR_ID_MAX) {
		dcam_idx = sprd_sensor_find_dcam_id(res.sensor_id);
		pr_debug("get csi id %d\n", dcam_idx);
	}

check:
	if (!is_csi_id(dcam_idx) || (dcam_idx < 0)) {
		pr_err("fail to get dcam id for sensor: %d\n", res.sensor_id);
		ret = -EFAULT;
		goto stop_thrd;
	}

	dcam = module->dcam_dev_handle;
	if (dcam == NULL) {
		dcam = dcam_core_pipe_dev_get(grp->hw_info, (void *)&grp->s_dcam_dev);
		if (IS_ERR_OR_NULL(dcam)) {
			if (retry) {
				dcam_idx++;
				goto check;
			}
			pr_err("fail to get dcam%d\n", dcam_idx);
			ret = -EINVAL;
			goto no_dcam;
		}
		module->dcam_dev_handle = dcam;
		module->dcam_idx = dcam_idx;
		module->attach_sensor_id = res.sensor_id;
		res.flag = 1;
	}

	ret = module->dcam_dev_handle->dcam_pipe_ops->open(dcam);
	if (ret) {
		ret = -EINVAL;
		goto dcam_fail;
	}

	/* full tee faceid need assign for ioctl_security after ioctl_res_get*/
	//module->grp->camsec_cfg.camsec_mode = SEC_TIME_PRIORITY;

	isp = module->isp_dev_handle;
	if (isp == NULL) {
		isp = isp_core_pipe_dev_get(grp->hw_info, (void *)&grp->s_isp_dev);
		if (IS_ERR_OR_NULL(isp)) {
			pr_err("fail to get isp\n");
			module->isp_dev_handle = NULL;
			ret = -EINVAL;
			goto no_isp;
		}
		module->isp_dev_handle = isp;
	}

	ret = module->isp_dev_handle->isp_ops->ioctl(module->isp_dev_handle,
		ISP_IOCTL_CFG_SEC, &module->grp->camsec_cfg.camsec_mode);
	if (ret) {
		pr_err("fail to set isp sec %d.\n", module->grp->camsec_cfg.camsec_mode);
		goto wq_fail;
	}

	ret = module->isp_dev_handle->isp_ops->open(isp, grp->hw_info);
	if (ret) {
		pr_err("fail to enable isp module.\n");
		ret = -EINVAL;
		goto isp_fail;
	}

	pr_info("sensor %d w %u h %u, cam %d, csi_idx %d, res.flag %x\n",
		res.sensor_id, res.width, res.height, module->idx, module->dcam_idx, res.flag);

	ret = copy_to_user((void __user *)arg, &res, sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_to_user\n");
		ret = -EFAULT;
		goto copy_fail;
	}
	atomic_set(&module->state, CAM_IDLE);
	pr_info("cam%d get res done\n", module->idx);
	return 0;

copy_fail:
wq_fail:
	module->isp_dev_handle->isp_ops->close(isp);

isp_fail:
	isp_core_pipe_dev_put(isp, (void *)grp->s_isp_dev);
	module->isp_dev_handle = NULL;

no_isp:
	module->dcam_dev_handle->dcam_pipe_ops->close(dcam);

dcam_fail:
	dcam_core_pipe_dev_put(dcam, (void *)grp->s_dcam_dev);
	module->dcam_dev_handle = NULL;
no_dcam:
	pr_err("fail to get camera res for sensor: %d\n", res.sensor_id);

stop_thrd:
	camthread_stop(&module->zoom_thrd);
	camthread_stop(&module->buf_thrd);

	return ret;
}

static int camioctl_cam_res_put(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_res res = {0};

	ret = copy_from_user(&res, (void __user *)arg, sizeof(struct sprd_img_res));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	pr_info("cam%d put res state: %d\n", module->idx, atomic_read(&module->state));
	ret = camioctl_stream_off(module, arg);

	if (atomic_read(&module->state) != CAM_IDLE) {
		pr_info("cam%d error state: %d\n", module->idx, atomic_read(&module->state));
		return -EFAULT;
	}

	if (module->attach_sensor_id != res.sensor_id) {
		pr_warn("warning: mismatch sensor id: %d, %d for cam %d\n",
			module->attach_sensor_id, res.sensor_id, module->idx);
	}

	module->attach_sensor_id = -1;

	camthread_stop(&module->zoom_thrd);
	camthread_stop(&module->buf_thrd);

	if (module->dcam_dev_handle) {
		module->dcam_dev_handle->dcam_pipe_ops->close(module->dcam_dev_handle);
		dcam_core_pipe_dev_put(module->dcam_dev_handle, (void *)module->grp->s_dcam_dev);
		module->dcam_dev_handle = NULL;
	}
	if (module->isp_dev_handle) {
		module->isp_dev_handle->isp_ops->close(module->isp_dev_handle);
		isp_core_pipe_dev_put(module->isp_dev_handle, (void *)module->grp->s_isp_dev);
		module->isp_dev_handle = NULL;
	}

	atomic_set(&module->state, CAM_INIT);

	pr_debug("sensor %d w %u h %u, cam [%d]\n", res.sensor_id, res.width, res.height, module->idx);
	pr_info("put camera res for sensor id %d done.", res.sensor_id, res.flag);
	ret = copy_to_user((void __user *)arg, &res, sizeof(struct sprd_img_res));
	if (ret)
		pr_err("fail to copy_to_user!\n");

	return ret;
}

static int camioctl_pre_raw_flag_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	uint32_t param = 0;
	struct channel_context *ch_pre = NULL;
	struct channel_context *ch_cap = NULL;

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EFAULT;
	}

	ch_pre = &module->channel[CAM_CH_PRE];
	ch_cap = &module->channel[CAM_CH_CAP];
	param = PRE_RAW_DEAL;

	ret = CAM_PIPEINE_DATA_COPY_NODE_CFG(ch_pre, CAM_PIPELINE_CFG_PRE_RAW_FLAG, &param);
	if (ret) {
		pr_err("fail to notify copy node pre status\n");
		return -EFAULT;
	}
	if (module->cam_uinfo.zsl_num) {
		ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch_cap, CAM_PIPELINE_CFG_PRE_RAW_FLAG, &param);
		if (ret) {
			pr_err("fail to notify cache node pre status\n");
			return -EFAULT;
		}
	}
	pr_info("cam%d pre raw flag set\n", module->idx);

	return ret;
}

static int camioctl_capture_start(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0, need_cfg_shutoff = 0;
	struct sprd_img_capture_param param = {0};
	ktime_t start_time = 0;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct channel_context *ch_pre = NULL;
	struct cam_capture_param cap_param = {0};
	struct cam_pipeline_shutoff_param pipeline_shutoff = {0};

	ret = copy_from_user(&param, (void __user *)arg,
			sizeof(struct sprd_img_capture_param));
	if (ret) {
		pr_err("fail to copy user info\n");
		return -EFAULT;
	}

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_info("cam%d state: %d\n", module->idx, atomic_read(&module->state));
		return ret;
	}

	hw = module->grp->hw_info;
	start_time = os_adapt_time_get_boottime();
	module->capture_times = start_time;
	module->opt_frame_fid = param.opt_frame_fid;
	ch = &module->channel[CAM_CH_CAP];
	if (!ch->enable)
		goto exit;
	if (module->opt_frame_fid && module->cam_uinfo.zsl_num) {
		cap_param.cap_opt_frame_scene = 1;
		CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_GET_CAP_FRAME, &module->opt_frame_fid);
	}
	if(ch) {
		mutex_lock(&module->buf_lock[ch->ch_id]);
		if (ch->alloc_start) {
			ret = wait_for_completion_interruptible(&ch->alloc_com);
			if (ret != 0)
				pr_err("fail to config channel/path param work %d\n", ret);
			pr_debug("alloc buffer done.\n");
			ch->alloc_start = 0;
		}
		mutex_unlock(&module->buf_lock[ch->ch_id]);
	}

	module->cap_scene = param.cap_scene;
	if (IS_XTM_CONFLICT_SCENE(module->cap_scene)) {
		uint32_t eb = 0;
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_XTM_EN, &eb);
	}

	atomic_set(&module->cap_skip_frames, -1);

	/* recognize the capture scene */
	switch (param.type) {
	case DCAM_CAPTURE_START_FROM_NEXT_SOF:
		module->capture_type = CAM_CAPTURE_START_FROM_NEXT_SOF;

		atomic_set(&module->capture_frames_dcam, param.cap_cnt);
		atomic_set(&module->cap_total_frames, param.cap_cnt);
		if ((module->cap_scene == CAPTURE_SW3DNR ||
			module->cap_scene == CAPTURE_FDR ||
			module->cap_scene == CAPTURE_RAWALG ||
			module->cap_scene == CAPTURE_AI_SFNR ||
			module->cap_scene == CAPTURE_HDR) &&
			(param.timestamp != 0))
			module->capture_times = param.timestamp;
		else
			module->capture_times = start_time;
		break;
	case DCAM_CAPTURE_START_WITH_TIMESTAMP:
		module->capture_type = CAM_CAPTURE_START_WITH_TIMESTAMP;
		/* dual camera master control slave frame */
		if (param.cap_cnt != 0)
			atomic_set(&module->capture_frames_dcam, param.cap_cnt);
		else
			atomic_set(&module->capture_frames_dcam, CAP_NUM_COMMON);
		module->capture_times = param.timestamp;
		break;
	case DCAM_CAPTURE_START:
		module->capture_type = CAM_CAPTURE_START;
		module->capture_times = param.timestamp;
		atomic_set(&module->capture_frames_dcam, CAP_NUM_COMMON);
		if (ch->need_framecache)
			ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_CFG_CLR_CACHE_BUF, NULL);
		break;
	case DCAM_CAPTURE_START_3DNR:
		module->capture_type = CAM_CAPTURE_START_3DNR;
		atomic_set(&module->capture_frames_dcam, -1);
		break;
	case DCAM_CAPTURE_STOP:
		module->capture_type = CAM_CAPTURE_STOP;
		atomic_set(&module->capture_frames_dcam, -1);
		break;
	default:
		break;
	}

	need_cfg_shutoff = camcore_shutoff_param_prepare(module, &pipeline_shutoff);
	if (need_cfg_shutoff)
		CAM_PIPEINE_SHUTOFF_PARAM_CFG(ch, pipeline_shutoff);

	cap_param.cap_type = module->capture_type;
	cap_param.cap_cnt = module->capture_frames_dcam;
	cap_param.cap_timestamp = module->capture_times;
	cap_param.cap_scene = module->cap_scene;
	cap_param.cap_user_crop = ch->latest_user_crop;
	cap_param.skip_first_num = param.skip_first_num;

	if (module->cam_uinfo.is_4in1 || module->cam_uinfo.dcam_slice_mode || module->cam_uinfo.is_longexp)
		cap_param.need_skip_scene = 1;
	if ((!module->cam_uinfo.dcam_slice_mode && module->cam_uinfo.zsl_num != 0
		&& !module->cam_uinfo.is_4in1 && (module->cap_scene != CAPTURE_AI_SFNR)) || module->cam_uinfo.is_dual)
		ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);
	else
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);

	ch_pre = &module->channel[CAM_CH_PRE];
	if (ch_pre->enable)
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch_pre, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);

	pr_info("cam %d start capture U_type %d, scene %d, cnt %d, time %lld, capture num:%d, opt_frame_fid %d\n",
		module->idx, param.type, module->cap_scene, param.cap_cnt,
		module->capture_times, module->capture_frames_dcam, param.opt_frame_fid);

exit:
	return ret;
}

static int camioctl_capture_stop(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;
	struct channel_context *ch = NULL;
	struct cam_capture_param cap_param = {0};
	struct channel_context *ch_pre = NULL;

	if (module->capture_type == CAM_CAPTURE_STOP) {
		pr_info("cam%d alreay capture stopped\n", module->idx);
		return 0;
	}

	ch = &module->channel[CAM_CH_CAP];
	if (!ch->enable)
		goto exit;
	module->capture_type = CAM_CAPTURE_STOP;
	module->raw_cap_fetch_fmt = CAM_FORMAT_MAX;
	atomic_set(&module->dual_select_frame_done, 0);

	pr_info("cam %d stop capture.\n", module->idx);
	hw = module->grp->hw_info;
	if (IS_XTM_CONFLICT_SCENE(module->cap_scene)) {
		uint32_t eb = 1;
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_XTM_EN, &eb);
	}

	module->cap_scene = CAPTURE_COMMON;
	module->is_flash_status = 0;

	if (module->cam_uinfo.zsl_num) {
		cap_param.cap_opt_frame_scene = 0;
		module->opt_frame_fid = 0;
		CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_GET_CAP_FRAME, &module->opt_frame_fid);
	}

	cap_param.cap_type = module->capture_type;
	cap_param.cap_scene = module->cap_scene;
	if ((!module->cam_uinfo.dcam_slice_mode && module->cam_uinfo.zsl_num != 0
		&& !module->cam_uinfo.is_4in1) || module->cam_uinfo.is_dual)
		ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);
	else
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);

	ch_pre = &module->channel[CAM_CH_PRE];
	if (ch_pre->enable)
		ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch_pre, CAM_PIPELINE_CFG_CAP_PARAM, &cap_param);

	if (hw->ip_isp->isphw_abt->pyr_dec_support) {
		CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPELINE_CFG_FAST_STOP, &ch->fast_stop);
		ret = wait_for_completion_interruptible_timeout(&ch->fast_stop, FAST_STOP_TIME_OUT);
		if (ret <= 0)
			pr_err("fail to fast stop pyrdec ret:%d\n", ret);
	}
	CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_FAST_STOP, ISP_NODE_MODE_CAP_ID, &ch->fast_stop);
	ret = wait_for_completion_interruptible_timeout(&ch->fast_stop, FAST_STOP_TIME_OUT);
	if (ret <= 0)
		pr_err("fail to fast stop isp ret:%d\n", ret);
	if (ch->pipeline_handle->pipeline_graph->need_yuv_scaler) {
		CAM_PIPEINE_ISP_SCALER_NODE_CFG(ch, CAM_PIPELINE_CFG_FAST_STOP, ISP_YUV_SCALER_CAP_NODE_ID, &ch->fast_stop);
		ret = wait_for_completion_interruptible_timeout(&ch->fast_stop, FAST_STOP_TIME_OUT);
		if (ret <= 0)
			pr_err("fail to fast stop isp ret:%d\n", ret);
	}
exit:
	return 0;
}

static int camioctl_raw_proc(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct isp_raw_proc_info proc_info = {0};

	ret = copy_from_user(&proc_info, (void __user *)arg,
				sizeof(struct isp_raw_proc_info));
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	if (!module->dcam_dev_handle || !module->isp_dev_handle) {
		pr_err("fail to init hw resource.\n");
		return -EFAULT;
	}

	switch (proc_info.cmd) {
	case RAW_PROC_PRE:
		if (g_dbg_raw2frgb_switch == DEBUG_RAWCAP_MODE)
			ret = camrawcap_raw_pre_proc(module, &proc_info);
		else
			ret = camrawcap_storeccm_frgb_pre_proc(module, &proc_info);
		break;
	case RAW_PROC_POST:
		if (g_dbg_raw2frgb_switch == DEBUG_RAWCAP_MODE)
			ret = camrawcap_raw_post_proc(module, &proc_info);
		else
			ret = camrawcap_storeccm_frgb_post_proc(module, &proc_info);
		break;
	case RAW_PROC_DONE:
		if (g_dbg_raw2frgb_switch == DEBUG_RAWCAP_MODE)
			ret = camrawcap_raw_proc_done(module);
		else
			ret = camrawcap_storeccm_frgb_proc_done(module);
		break;
	case VIRTUAL_SENSOR_PROC:
		ret = camcore_virtual_sensor_proc(module, &proc_info);
		break;
	default:
		pr_err("fail to get correct cmd %d\n", proc_info.cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int camioctl_cam_post_proc(struct camera_module *module, unsigned long arg)
{
	int ret = 0, i = 0;
	uint32_t scene_mode = 0, mode_3dnr = 0, index = 0, reserved[4] = {0}, in_fmt = 0;
	struct channel_context *ch = NULL;
	struct sprd_img_size dst_size = {0};
	struct cam_zoom_index zoom_index = {0};
	struct sprd_img_parm __user *uparam = NULL;
	struct cam_postproc_blkpm postproc_blkpm = {0};
	struct cam_pipeline_cfg_param param = {0};
	struct cam_frame *pfrm_dcam = NULL, *pfrm_isp = NULL;
	struct cam_frame *pframe = NULL, *pfrm[3] = {NULL, NULL, NULL};

	if (atomic_read(&module->state) != CAM_RUNNING) {
		pr_warn("warning: only for state RUNNING\n");
		return -EFAULT;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(index, &uparam->index);
	ret |= get_user(scene_mode, &uparam->scene_mode);
	ret |= get_user(reserved[0], &uparam->reserved[0]);
	ret |= get_user(reserved[1], &uparam->reserved[1]);
	ret |= get_user(reserved[2], &uparam->reserved[2]);
	ret |= get_user(reserved[3], &uparam->reserved[3]);
	ret |= get_user(dst_size.w, &uparam->dst_size.w);
	ret |= get_user(dst_size.h, &uparam->dst_size.h);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		goto exit;
	}
	if (scene_mode == CAM_POSTPROC_VID_NOISE_RD)
		ch = &module->channel[CAM_CH_PRE];
	else
		ch = &module->channel[CAM_CH_CAP];
	for (i = 0; i < 3; i++) {
		pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
		ret |= get_user(pframe->common.buf.mfd, &uparam->fd_array[i]);
		if (pframe->common.buf.mfd == 0) {
			cam_queue_empty_frame_put(pframe);
			continue;
		}
		pframe->common.fid = index;
		pframe->common.sensor_time.tv_sec = reserved[0];
		pframe->common.sensor_time.tv_usec = reserved[1];
		pframe->common.boot_sensor_time = reserved[3];
		pframe->common.boot_sensor_time = (pframe->common.boot_sensor_time << 32) | reserved[2];
		pframe->common.buf.type = CAM_BUF_USER;
		pframe->common.link_from.node_type = CAM_NODE_TYPE_USER;
		pframe->common.link_from.node_id = CAM_LINK_DEFAULT_NODE_ID;
		if (scene_mode == CAM_POSTPROC_VID_NOISE_RD) {
			pframe->common.width = ch->ch_uinfo.dst_size.w;
			pframe->common.height  = ch->ch_uinfo.dst_size.h;
		} else {
			pframe->common.width = ch->ch_uinfo.src_size.w;
			pframe->common.height  = ch->ch_uinfo.src_size.h;
		}
		ret |= get_user(pframe->common.buf.addr_vir[0], &uparam->frame_addr_vir_array[i].y);
		ret |= get_user(pframe->common.buf.addr_vir[1], &uparam->frame_addr_vir_array[i].u);
		ret |= get_user(pframe->common.buf.addr_vir[2], &uparam->frame_addr_vir_array[i].v);
		pframe->common.channel_id = ch->ch_id;
		pframe->common.buf.status = CAM_BUF_ALLOC;
		pfrm[i] = pframe;
	}

	if (scene_mode == CAM_POSTPROC_VID_NOISE_RD) {
		zoom_index.node_id = ISP_NODE_MODE_CAP_ID;
		zoom_index.node_type = CAM_NODE_TYPE_ISP_OFFLINE;
		zoom_index.port_type = PORT_TRANSFER_IN;
		zoom_index.port_id = PORT_ISP_OFFLINE_IN;
		ret = camcore_postproc_zoom_param_get(module, ch, &pfrm[0]->common.zoom_data, &zoom_index);
		/* Tmp change, it will be changed when hal porting dr func after dispatch change back a14*/
		/*ret |= get_user(postproc_blkpm.blk_property, &uparam->blk_param);
		if (ret) {
			pr_err("fail to get postproc blk param addr\n");
			goto blkpm_cfg_err;
		}*/

		postproc_blkpm.fid = index;
		ret = CAM_PIPEINE_NR_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_POSTPROC_PARAM, &postproc_blkpm);
		if (ret) {
			pr_err("fail to cfg isp postproc param.\n");
			goto blkpm_cfg_err;
		}
		ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPELINE_CFG_POSTPROC_PARAM, &postproc_blkpm);
		if (ret) {
			pr_err("fail to cfg dec postproc param.\n");
			goto blkpm_cfg_err;
		}
		mode_3dnr = MODE_3DNR_OFF;
		ret = CAM_PIPEINE_NR_ISP_NODE_CFG(ch, CAM_PIPELINE_CFG_3DNR_MODE, &mode_3dnr);
		if (ret) {
			pr_err("fail to cfg isp 3dnr mode.\n");
			goto blkpm_cfg_err;
		}

		pfrm_isp = pfrm[1];
		pfrm_isp->common.proc_mode = CAM_POSTPROC_SERIAL;
		ret = CAM_PIPEINE_NR_ISP_OUT_PORT_CFG(ch, PORT_VID_OUT, CAM_PIPELINE_CFG_BUF, pfrm_isp);
		if (ret) {
			pr_err("fail to cfg isp out buffer.\n");
			goto blkpm_cfg_err;
		}

		in_fmt = CAM_YVU420_2FRAME;
		ret = CAM_PIPEINE_PYR_DEC_NODE_CFG(ch, CAM_PIPELINE_CFG_BASE, &in_fmt);
		if (ret) {
			pr_err("fail to cfg dec in fmt.\n");
			goto buf_cfg_err;
		}
		param.node_type = CAM_NODE_TYPE_PYR_DEC;
	} else {
		pfrm_dcam = pfrm[1];
		pfrm_isp = pfrm[2];
		if (scene_mode == CAM_POSTPROC_CAP_PROC_RAW) {
			param.node_type = CAM_NODE_TYPE_DCAM_OFFLINE_BPC_RAW;
			pfrm[0]->common.link_to.port_id = ch->aux_raw_port_id;
			pfrm_dcam->common.irq_property = CAM_FRAME_PROCESS_RAW;
			param.node_param.port_id = ch->aux_raw_port_id;
		} else {
			param.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
			pfrm[0]->common.link_to.port_id = ch->aux_dcam_port_id;
			pfrm_dcam->common.irq_property = CAM_FRAME_FDRH;
			param.node_param.port_id = ch->aux_dcam_port_id;

			if (dst_size.w && dst_size.h) {
				pfrm_isp->common.width = dst_size.w;
				pfrm_isp->common.height = dst_size.h;
			} else {
				pfrm_isp->common.width = ch->ch_uinfo.dst_size_tmp.w;
				pfrm_isp->common.height = ch->ch_uinfo.dst_size_tmp.h;
			}

			ch->ch_uinfo.dst_size.w = pfrm_isp->common.width;
			ch->ch_uinfo.dst_size.h = pfrm_isp->common.height;
			ret = cam_zoom_channel_size_config(module, ch);

			ret = CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF, ISP_NODE_MODE_CAP_ID, pfrm_isp);
			if (ret) {
				pr_err("fail to cfg isp out buffer.\n");
				goto size_cfg_err;
			}
		}

		ret = CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, param.node_param.port_id, CAM_PIPELINE_CFG_BUF, pfrm_dcam, param.node_type);
		if (ret) {
			pr_err("fail to cfg dcam out buffer.\n");
			goto buf_cfg_err;
		}
	}

	ret = camcore_frame_start_proc(module, pfrm[0], param.node_type, ch);
	if (ret) {
		pr_err("fail to start pipeline or cfg dcam out buffer.\n");
		goto start_proc_err;
	}

	pr_info("scene %d, frm fd (%d 0x%x), (%d 0x%x)\n",
		scene_mode, pfrm[0]->common.buf.mfd, pfrm[0]->common.buf.offset[0],
		pfrm[1]->common.buf.mfd, pfrm[1]->common.buf.offset[0]);

	if (scene_mode == CAM_POSTPROC_VID_NOISE_RD) {
		ret = wait_for_completion_timeout(&module->postproc_done, DCAM_OFFLINE_TIMEOUT);
		if (ret <= 0) {
			pr_err("fail to wait isp offline node done\n");
			ret = -EFAULT;
		        goto buf_cfg_err;
		}
	}

	return 0;

start_proc_err:
	if (scene_mode != CAM_POSTPROC_VID_NOISE_RD)
		CAM_PIPEINE_DCAM_OFFLINE_OUT_PORT_CFG(ch, param.node_param.port_id, CAM_PIPELINE_CFG_BUF_CLR, &param, param.node_type);
buf_cfg_err:
	if (scene_mode == CAM_POSTPROC_VID_NOISE_RD)
		CAM_PIPEINE_NR_ISP_OUT_PORT_CFG(ch, PORT_VID_OUT, CAM_PIPELINE_CFG_BUF_CLR, &param);
	else
		CAM_PIPEINE_ISP_OUT_PORT_CFG(ch, ch->isp_port_id, CAM_PIPELINE_CFG_BUF_CLR, ISP_NODE_MODE_CAP_ID, &param);
size_cfg_err:
blkpm_cfg_err:
	for (i = 0; i < 3; i++) {
		if (pfrm[i])
			cam_queue_empty_frame_put(pfrm[i]);
	}
exit:
	return ret;
}

/* set addr for 4in1 raw which need remosaic */
static int camioctl_4in1_raw_addr_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0, i = 0;
	uint32_t channel_id = 0, buffer_count = 0, is_reserved_buf = 0;
	enum dcam_id idx = 0;
	struct sprd_img_parm __user *uparam = NULL;
	struct channel_context *ch = NULL;
	struct cam_frame *pframe = NULL;

	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(channel_id, &uparam->channel_id);
	ret |= get_user(buffer_count, &uparam->buffer_count);
	ret |= get_user(is_reserved_buf, &uparam->is_reserved_buf);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}
	idx = module->idx;
	pr_debug("idx %d, channel_id %d, enable %d\n", idx,
		channel_id, module->channel[channel_id].enable);
	if ((channel_id >= CAM_CH_MAX) ||
		(buffer_count == 0) ||
		(module->channel[channel_id].enable == 0)) {
		pr_err("fail to get valid channel id %d. buf cnt %d\n",
			channel_id, buffer_count);
		return -EFAULT;
	}
	pr_info("ch %d, buffer_count %d\n", channel_id, buffer_count);

	ch = &module->channel[channel_id];

	for (i = 0; i < buffer_count; i++) {
		pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
		pframe->common.buf.type = CAM_BUF_USER;
		ret |= get_user(pframe->common.buf.mfd,&uparam->fd_array[i]);
		ret |= get_user(pframe->common.buf.addr_vir[0],&uparam->frame_addr_vir_array[i].y);
		ret |= get_user(pframe->common.buf.addr_vir[1],&uparam->frame_addr_vir_array[i].u);
		ret |= get_user(pframe->common.buf.addr_vir[2],&uparam->frame_addr_vir_array[i].v);
		pframe->common.channel_id = ch->ch_id;
		pframe->common.width = ch->ch_uinfo.src_size.w;
		pframe->common.height = ch->ch_uinfo.src_size.h;
		pr_debug("mfd %d, reserved %d\n", pframe->common.buf.mfd, is_reserved_buf);
		ret = cam_buf_ionbuf_get(&pframe->common.buf);
		if (ret) {
			pr_err("fail to get ion buffer\n");
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}

		if (is_reserved_buf) {
			module->reserved_buf_fd = pframe->common.buf.mfd;
			pframe->common.is_reserved = CAM_RESERVED_BUFFER_ORI;
			pframe->common.buf.size = cal_sprd_size(ch->ch_uinfo.src_size.w, ch->ch_uinfo.src_size.h, ch->ch_uinfo.dcam_raw_fmt);
			cam_scene_reserved_buf_cfg(RESERVED_BUF_SET_CB, pframe, module);
			module->res_frame = pframe;
		} else {
			ret = camcore_dcam_online_buf_cfg(ch, pframe, module);
		}

		if (ret) {
			pr_err("fail to set output buffer for ch%d.\n", ch->ch_id);
			cam_queue_empty_frame_put(pframe);
			ret = -EFAULT;
			break;
		}
	}
	pr_info("exit, ret = %d\n", ret);

	return ret;
}

/* buffer set to kernel after remosaic */
static int camioctl_4in1_post_proc(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0, iommu_enable = 0;
	uint32_t index = 0, reserved1 = 0, reserved2 = 0, fd_array = 0;
	struct sprd_img_parm __user *uparam = NULL;
	struct channel_context *channel = NULL;
	struct cam_frame *pframe = NULL;
	ktime_t sensor_time = {0};

	if ((atomic_read(&module->state) != CAM_RUNNING)) {
		pr_info("cam%d state: %d\n", module->idx, atomic_read(&module->state));
		return -EFAULT;
	}

	uparam = (struct sprd_img_parm __user *)arg;
	ret |= get_user(reserved1, &uparam->reserved[1]);
	ret |= get_user(reserved2, &uparam->reserved[2]);
	ret |= get_user(fd_array, &uparam->fd_array[0]);
	ret |= get_user(index, &uparam->index);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	pr_info("Enter\n");
	/* about image data address
	 * fetch: from HAL, in param
	 * dcam out: alloc by kernel, share buf
	 */
	channel = &module->channel[CAM_CH_CAP];
	iommu_enable = module->iommu_enable;
	/* timestamp, reserved	([2]<<32)|[1]
	 * Attention: Only send back one time, maybe need some
	 * change when hal use another time
	 */
	sensor_time = reserved2;
	sensor_time <<= 32;
	sensor_time |= reserved1;
	pframe = cam_queue_empty_frame_get(CAM_FRAME_GENERAL);
	pframe->common.boot_sensor_time = sensor_time;
	pframe->common.sensor_time = os_adapt_time_ktime_to_timeval(os_adapt_time_sub(
		pframe->common.boot_sensor_time, os_adapt_time_sub(
		os_adapt_time_get_boottime(), os_adapt_time_get_monotonic())));
	pframe->common.fid = index;

	pframe->common.width = channel->ch_uinfo.src_size.w;
	pframe->common.height = channel->ch_uinfo.src_size.h;
	pframe->common.buf.type = CAM_BUF_USER;
	pframe->common.buf.status = CAM_BUF_ALLOC;
	pframe->common.buf.mfd = fd_array;
	ret |= get_user(pframe->common.buf.addr_vir[0],&uparam->frame_addr_vir_array[0].y);
	ret |= get_user(pframe->common.buf.addr_vir[1],&uparam->frame_addr_vir_array[0].u);
	ret |= get_user(pframe->common.buf.addr_vir[2],&uparam->frame_addr_vir_array[0].v);
	pframe->common.channel_id = channel->ch_id;

	pr_info("frame[%d] fd %d addr_vir[0]=0x%lx iova=0x%lx\n",
		pframe->common.fid, pframe->common.buf.mfd, pframe->common.buf.addr_vir[0],
		pframe->common.buf.iova);

	ret = camcore_frame_start_proc(module, pframe, CAM_NODE_TYPE_DCAM_OFFLINE, channel);
	if (ret) {
		pr_err("fail to start dcam for raw proc\n");
		cam_queue_empty_frame_put(pframe);
		return -EFAULT;
	}

	return ret;
}

/* set which channel use hw 3dnr
 * if auto 3dnr, will be set when previewing
 */
static int camioctl_3dnr_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_3dnr_mode parm = {0};
	struct channel_context *ch = NULL;
	uint32_t ch_id = 0;

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg, sizeof(parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}
	ch_id = parm.channel_id;
	if (ch_id >= CAM_CH_MAX) {
		pr_err("fail to get channel id %d\n", ch_id);
		return -EFAULT;
	}
	ch = &module->channel[ch_id];
	pr_info("ch_id %d, need_3dnr %d\n", ch_id, parm.need_3dnr);
	/* dynamic set when auto 3dnr */
	if (module->auto_3dnr) {
		if (ch->uinfo_3dnr == parm.need_3dnr)
			return ret;

		ch->uinfo_3dnr = parm.need_3dnr;
		camcore_capture_3dnr_set(module, ch);
	} else {
		ch->uinfo_3dnr = parm.need_3dnr;
	}

	return ret;
}

static int camioctl_auto_3dnr_mode_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_auto_3dnr_mode parm = {0};

	if (!module) {
		pr_err("fail to get valid param\n");
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg, sizeof(parm));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}
	module->auto_3dnr = parm.auto_3dnr_enable;
	pr_info("auto_3dnr %d\n", module->auto_3dnr);

	return ret;
}

static int camioctl_capability_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct img_size size = {0};

	if (!module) {
		pr_err("fail to get valid param\n");
		ret = -EINVAL;
		goto exit;
	}

	size.w = ISP_WIDTH_MAX;
	size.h = ISP_HEIGHT_MAX;
	ret = copy_to_user((void __user *)arg, &size, sizeof(struct img_size));
	if (unlikely(ret)) {
		pr_err("fail to get capability, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

exit:
	return ret;
}

static int camioctl_scaler_capability_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	uint32_t isp_scaler_up_max = ISP_SCALER_UP_MAX;

	if (!module) {
		pr_err("fail to get valid param\n");
		ret = -EINVAL;
		goto exit;
	}

	ret = copy_to_user((void __user *)arg, &isp_scaler_up_max, sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to get SCALER capability, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}
	pr_debug("isp_scaler_up_max: %d\n", isp_scaler_up_max);

exit:
	return ret;
}

static int camioctl_stream_pause(struct camera_module *module,
		unsigned long arg)
{
	/* TBD: Need to check if this function necessary */
	return 0;
}

static int camioctl_stream_resume(struct camera_module *module,
		unsigned long arg)
{
	/* TBD: Need to check if this function necessary */
	return 0;
}

static int camioctl_dewarp_otp_set(struct camera_module *module, unsigned long arg)
{
	/* TBD: Need to check if this function necessary */
	return 0;
}

static int camioctl_dewarp_hw_capability_get(struct camera_module *module,
		unsigned long arg)
{
	/* TBD: Need to check if this function necessary */
	return 0;
}

static int camioctl_path_rect_get(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_img_path_rect parm = {0};
	struct channel_context *ch = NULL;

	if ((atomic_read(&module->state) != CAM_CFG_CH) &&
		(atomic_read(&module->state) != CAM_RUNNING)) {
		pr_warn("warning: module state: %d\n", atomic_read(&module->state));
		return 0;
	}

	ch = &module->channel[CAM_CH_PRE];
	if (!ch->enable) {
		pr_debug("ch preview not enable\n");
		return 0;
	}
	if (!module || !ch->pipeline_handle) {
		pr_err("fail to get valid param %px, %px\n", module, ch->pipeline_handle);
		return -EINVAL;
	}

	memset((void *)&parm, 0, sizeof(parm));
	ret = copy_from_user(&parm, (void __user *)arg,
		sizeof(struct sprd_img_path_rect));
	if (ret) {
		pr_err("fail to get user info ret %d\n", ret);
		return -EFAULT;
	}

	ret = CAM_PIPEINE_DCAM_ONLINE_NODE_CFG(ch, CAM_PIPELINE_CFG_RECT_GET, &parm);
	pr_debug("TRIM rect info x %d y %d w %d h %d\n",
		parm.trim_valid_rect.x, parm.trim_valid_rect.y,
		parm.trim_valid_rect.w, parm.trim_valid_rect.h);
	pr_debug("AE rect info x %d y %d w %d h %d\n",
		parm.ae_valid_rect.x, parm.ae_valid_rect.y,
		parm.ae_valid_rect.w, parm.ae_valid_rect.h);
	pr_debug("AF rect info x %d y %d w %d h %d\n",
		parm.af_valid_rect.x, parm.af_valid_rect.y,
		parm.af_valid_rect.w, parm.af_valid_rect.h);

	ret = copy_to_user((void __user *)arg, &parm, sizeof(struct sprd_img_path_rect));
	if (ret) {
		ret = -EFAULT;
		pr_err("fail to copy to user\n");
		return ret;
	}

	return ret;
}

static int camioctl_csi_switch(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	uint32_t switch_mode = CAM_CSI_NORMAL_SWITCH;
	uint32_t csi_connect_stat = DCAM_CSI_IDLE;
	uint32_t csi_connect = 0;

	ret = copy_from_user(&csi_connect, (void __user *)arg, sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to get capability, ret %d\n", ret);
		return -EFAULT;
	}
	module->dcam_ctx_bind_state = csi_connect;

	switch (csi_connect) {
		case 0:
			csi_connect_stat = DCAM_CSI_PAUSE;
			ret = camcore_csi_switch_disconnect(module, switch_mode, csi_connect_stat);
			break;
		case 1:
			csi_connect_stat = DCAM_CSI_RESUME;
			ret = camcore_csi_switch_connect(module, switch_mode, csi_connect_stat);
			break;
		default:
			pr_err("fail to get vailed csi_connect: %d\n", csi_connect);
			break;
	}

	return ret;
}

static int camioctl_path_pause(struct camera_module *module,
	unsigned long arg)
{
	int ret = 0;
	uint32_t dcam_path_state = DCAM_PATH_PAUSE;
	struct dcam_online_port *port = NULL;
	struct channel_context *ch = NULL;
	unsigned long flag = 0;

	pr_debug("pause cam%d with csi_id %d\n", module->idx, module->dcam_idx);
	module->path_state = dcam_path_state;
	ch = &module->channel[CAM_CH_CAP];
	if (atomic_read(&module->state) == CAM_RUNNING) {
		if (ch->enable) {
			ret = CAM_PIPEINE_ISP_NODE_CFG(ch, CAM_PIPRLINE_CFG_PARAM_Q_CLEAR, ISP_NODE_MODE_CAP_ID, NULL);
			if (ret)
				pr_err("fail to recycle cam%d  blk param node\n", module->idx);
			if (module->cam_uinfo.need_share_buf) {
				port = module->nodes_dev.dcam_online_out_port_dev[PORT_FULL_OUT];
				spin_lock_irqsave(&port->state_lock, flag);
				port->port_state = dcam_path_state;
				port->state_update = 1;
				spin_unlock_irqrestore(&port->state_lock, flag);
				ret = CAM_PIPEINE_DCAM_ONLINE_OUT_PORT_CFG(ch, PORT_FULL_OUT, CAM_PIPILINE_CFG_SHARE_BUF, &dcam_path_state);
			}

			if (module->cam_uinfo.zsl_num != 0 || module->cam_uinfo.is_dual)
				ret = CAM_PIPEINE_FRAME_CACHE_NODE_CFG(ch, CAM_PIPELINE_CFG_CLR_CACHE_BUF, NULL);
		}
	}

	return ret;
}

static int camioctl_path_resume(struct camera_module *module,
	unsigned long arg)
{
	int ret = 0;
	uint32_t dcam_path_state = DCAM_PATH_RESUME;
	struct dcam_online_port *port = NULL;
	unsigned long flag = 0;

	pr_debug("resume cam%d with csi_id %d\n", module->idx, module->dcam_idx);
	module->path_state = dcam_path_state;
	if (atomic_read(&module->state) == CAM_RUNNING) {
		if (module->channel[CAM_CH_CAP].enable) {
			port = module->nodes_dev.dcam_online_out_port_dev[PORT_FULL_OUT];
			spin_lock_irqsave(&port->state_lock, flag);
			port->port_state = dcam_path_state;
			port->state_update = 1;
			spin_unlock_irqrestore(&port->state_lock, flag);
		}
	}

	return ret;
}

static int camioctl_cam_test(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct cam_hw_info *hw = NULL;

	hw = module->grp->hw_info;
	ret = camt_test(hw, arg);

	return ret;
}

static int camioctl_longexp_mode_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_img_longexp_mode param = {0};

	if (!module) {
		pr_err("fail to get input ptr %p\n", module);
		return -EINVAL;
	}

	ret = copy_from_user(&param, (void __user *)arg, sizeof(param));
	if (ret) {
		pr_err("fail to get longexp mode %d\n", ret);
		return -EFAULT;
	}

	module->cam_uinfo.is_longexp = param.need_longexp;
	pr_debug("is_longexp %d\n", param.need_longexp);

	return ret;
}

static int camioctl_mul_max_sensor_size_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct img_size *dst = NULL;
	struct camera_group *grp = NULL;

	grp = module->grp;
	dst = &grp->mul_sn_max_size;

	ret = copy_from_user(dst, (void __user *)arg, sizeof(struct img_size));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_debug("mul sensor_size %d %d\n", dst->w, dst->h);

exit:
	return ret;
}

static int camioctl_cap_zsl_info_set(struct camera_module *module,
		unsigned long arg)
{
	int ret = 0;
	struct sprd_cap_zsl_param param = {0};

	ret = copy_from_user(&param, (void __user *)arg, sizeof(struct sprd_cap_zsl_param));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		ret = -EFAULT;
		goto exit;
	}

	pr_debug("cam %d, zsl num %d zsl skip num %d is share buf %d\n", module->idx, param.zsl_num,
		param.zsk_skip_num, param.need_share_buf);

	module->cam_uinfo.zsl_num = param.zsl_num;
	module->cam_uinfo.zsk_skip_num = param.zsk_skip_num;
	module->cam_uinfo.need_share_buf = param.need_share_buf;

exit:
	return ret;
}

static int camioctl_dcam_raw_fmt_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_dcam_raw_fmt param = {0};
	struct cam_hw_info *hw = NULL;
	struct channel_context *channel = NULL;
	uint32_t dcam_raw_update = 0, sensor_raw_update = 0;
	uint32_t i = 0, rawpath_id = 0;

	ret = copy_from_user(&param, (void __user *)arg, sizeof(struct sprd_dcam_raw_fmt));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	hw = module->grp->hw_info;
	if (param.ch_id > CAM_CH_CAP && param.ch_id != CAM_CH_VIRTUAL && param.ch_id != CAM_CH_DCAM_VCH) {
		pr_err("fail to check param, ch%d\n", param.ch_id);
		return -EFAULT;
	}
	if ((param.dcam_raw_fmt >= CAM_FORMAT_MAX) && (param.sensor_raw_fmt >= CAM_FORMAT_MAX)) {
		pr_err("fail to check param, sensor raw fmt %d, dcam raw fmt %d, ch%d\n",
			param.sensor_raw_fmt, param.dcam_raw_fmt, param.ch_id);
		return -EFAULT;
	}

	channel = &module->channel[param.ch_id];
	if (channel->enable == 0) {
		pr_err("ch%d not enable\n", param.ch_id);
		return -EFAULT;
	}

	rawpath_id = camcore_dcampath_id_convert(hw->ip_dcam[0]->dcamhw_abt->dcam_raw_path_id);
	for (i = 0; i < CAM_FORMAT_MAX; i++) {
		if (hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[i] == CAM_FORMAT_MAX)
			break;
		if (param.dcam_raw_fmt == hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[i]) {
			channel->ch_uinfo.dcam_raw_fmt = param.dcam_raw_fmt;
			/* when 4in1 hal according to remosic
			 * converts sensor raw fmt to dcam raw fmt
			 * dcam offline fetch fmt use dcam raw fmt config by hal
			 */
			if (module->cam_uinfo.is_4in1)
				module->raw_cap_fetch_fmt = param.dcam_raw_fmt;
			dcam_raw_update = 1;
		}
	}

	for (i = 0; i < CAM_FORMAT_MAX; i++) {
		if (hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[i] == CAM_FORMAT_MAX)
			break;
		if (param.sensor_raw_fmt == hw->ip_dcam[0]->dcampath_abt[rawpath_id]->format[i]) {
			channel->ch_uinfo.sensor_raw_fmt = param.sensor_raw_fmt;
			if (!module->cam_uinfo.is_4in1)
				module->raw_cap_fetch_fmt = param.sensor_raw_fmt;
			sensor_raw_update = 1;
		}
	}

	if (dcam_raw_update)
		pr_info("cam%d, ch%d set dcam raw fmt %s\n", module->idx, param.ch_id, camport_fmt_name_get(channel->ch_uinfo.dcam_raw_fmt));
	if (sensor_raw_update)
		pr_info("cam%d, ch%d set sensor raw fmt %s\n", module->idx, param.ch_id, camport_fmt_name_get(channel->ch_uinfo.sensor_raw_fmt));
 	if ((!dcam_raw_update) && (!sensor_raw_update)) {
		pr_err("fail to support raw fmt, not support, %s %s\n", camport_fmt_name_get(param.sensor_raw_fmt), camport_fmt_name_get(param.dcam_raw_fmt));
		return -EFAULT;
	} else
		return 0;

}

static int camioctl_key_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	uint32_t param = 0;

	ret = copy_from_user(&param, (void __user *)arg, sizeof(uint32_t));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}
	if (param == CAM_IOCTL_PRIVATE_KEY)
		module->private_key = 1;
	pr_info("cam%d get ioctl permission %d\n", module->idx, module->private_key);
	return 0;
}

static int camioctl_960fps_param_set(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	struct sprd_slowmotion_960fps_param param = {0};
	struct channel_context *channel = NULL;

	ret = copy_from_user(&param, (void __user *)arg, sizeof(struct sprd_slowmotion_960fps_param));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	if (param.ch_id > CAM_CH_CAP && param.ch_id != CAM_CH_VIRTUAL) {
		pr_err("fail to check param, ch%d\n", param.ch_id);
		return -EFAULT;
	}
	channel = &module->channel[param.ch_id];
	if (channel->enable == 0) {
		pr_err("ch%d not enable\n", param.ch_id);
		return -EFAULT;
	}
	channel->ch_uinfo.slowmotion_stage_a_num = param.stage_a_frm_num;
	channel->ch_uinfo.slowmotion_stage_a_valid_num = param.stage_a_valid_frm_num;
	channel->ch_uinfo.slowmotion_stage_b_num= param.stage_b_frm_num;
	pr_info("cam%d ch%d, (%d/8 %d num), (8/8 %d num)\n",
		module->idx, param.ch_id, channel->ch_uinfo.slowmotion_stage_a_valid_num,
		channel->ch_uinfo.slowmotion_stage_a_num, channel->ch_uinfo.slowmotion_stage_b_num);
	return 0;
}

static int camioctl_cfg_param_start_end(struct camera_module *module, unsigned long arg)
{
	int ret = 0;
	uint32_t for_capture = 0;
	struct sprd_cfg_param_status param = {0};
	struct channel_context *channel = NULL;
	struct cam_pipeline_cfg_param pipe_param = {0};
	struct cfg_param_status cfg_param = {0};

	if (atomic_read(&module->state) != CAM_CFG_CH &&
		atomic_read(&module->state) != CAM_STREAM_ON &&
		atomic_read(&module->state) != CAM_RUNNING)
		return ret;

	ret = copy_from_user(&param, (void __user *)arg, sizeof(struct sprd_cfg_param_status));
	if (unlikely(ret)) {
		pr_err("fail to copy from user, ret %d\n", ret);
		return -EFAULT;
	}

	for_capture = param.scene_id == PM_SCENE_CAP ? 1 : 0;
	cfg_param.status = param.status;
	cfg_param.frame_id = param.frame_id;
	cfg_param.scene_id = param.scene_id;
	cfg_param.update = param.update;
	cfg_param.dcam_ctx_bind_state = module->dcam_ctx_bind_state;

	if (for_capture && (module->channel[CAM_CH_CAP].enable == 0)) {
		pr_warn("warn: cam%d cap channel not enable\n");
		return 0;
	}

	pr_debug("cam%d status %d, fid %d, scene %d, update %d\n", module->idx, param.status, param.frame_id,
		param.scene_id, param.update);
	if (param.status == 0) {
		if (module->isp_dev_handle->cfg_handle) {
			if (param.scene_id == PM_SCENE_PRE) {
				channel = &module->channel[CAM_CH_PRE];
				if (channel->enable == 0)
					return 0;
				CAM_PIPEINE_ISP_NODE_CFG(channel, CAM_PIPELINE_CFG_PARAM_SWITCH, ISP_NODE_MODE_PRE_ID, &cfg_param);
			}
		}
		if (param.scene_id == PM_SCENE_OFFLINE_CAP) {
			channel = &module->channel[CAM_CH_CAP];
			if (channel->enable == 0)
				return 0;
			pipe_param.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
			pipe_param.node_param.param = &param.frame_id;
			pipe_param.node_id = DCAM_OFFLINE_NODE_ID;
			ret = channel->pipeline_handle->ops.cfg_param(channel->pipeline_handle,
					CAM_PIPELINE_CFG_PARAM_FID, &pipe_param);
		}
	}

	if (param.status) {
		if (module->isp_dev_handle->cfg_handle) {
			if (param.scene_id == PM_SCENE_PRE) {
				channel = &module->channel[CAM_CH_PRE];
				if (channel->enable == 0)
					return 0;
				CAM_PIPEINE_ISP_NODE_CFG(channel, CAM_PIPELINE_CFG_PARAM_SWITCH, ISP_NODE_MODE_PRE_ID, &cfg_param);
			} else {
				channel = &module->channel[CAM_CH_CAP];
				if (channel->enable == 0)
					return 0;
				CAM_PIPEINE_ISP_NODE_CFG(channel, CAM_PIPELINE_CFG_PARAM_SWITCH, ISP_NODE_MODE_CAP_ID, &cfg_param);
			}
		}
		if (module->isp_dev_handle->pyr_dec_handle) {
			if (param.scene_id == PM_SCENE_CAP) {
				channel = &module->channel[CAM_CH_CAP];
				if (channel->enable == 0)
					return 0;
				CAM_PIPEINE_PYR_DEC_NODE_CFG(channel, CAM_PIPELINE_CFG_PARAM_SWITCH, &cfg_param);
			}
		}
		if (param.scene_id == PM_SCENE_OFFLINE_CAP) {
			channel = &module->channel[CAM_CH_CAP];
			if (channel->enable == 0)
				return 0;
			pipe_param.node_type = CAM_NODE_TYPE_DCAM_OFFLINE;
			pipe_param.node_param.param = &param.frame_id;
			pipe_param.node_id = DCAM_OFFLINE_NODE_ID;
			ret = channel->pipeline_handle->ops.cfg_param(channel->pipeline_handle,
					CAM_PIPELINE_CFG_RESET_PARAM_PTR, &pipe_param);
		}
	}
	return ret;
}
#endif
