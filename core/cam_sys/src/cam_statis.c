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

#include <linux/slab.h>

#include "cam_statis.h"
#include "isp_dev.h"
#include "isp_node.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_STATIS: %d %d %s : " fmt, current->pid, __LINE__, __func__

/* VCH2 maybe used for raw picture output
 * If yes, PDAF should not output data though VCH2
 * todo: avoid conflict between raw/pdaf type3
 */
struct statis_port_buf_info s_statis_port_info_all[] = {
	{PORT_PDAF_OUT,      0,  0, STATIS_PDAF},
	{PORT_VCH2_OUT,      0,  0, STATIS_EBD},
	{PORT_AEM_OUT,       0,  0, STATIS_AEM},
	{PORT_AFM_OUT,       0,  0, STATIS_AFM},
	{PORT_AFL_OUT,       0,  0, STATIS_AFL},
	{PORT_BAYER_HIST_OUT,      0,  0, STATIS_HIST},
	{PORT_FRGB_HIST_OUT, 0,  0, STATIS_HIST2},
	{PORT_LSCM_OUT,      0,  0, STATIS_LSCM},
	{PORT_GTM_HIST_OUT,  0,  0, STATIS_GTMHIST},
};

static enum cam_port_dcam_online_out_id camstatis_dcam_type_to_port_id(enum isp_statis_buf_type type)
{
	switch (type) {
	case STATIS_AEM:
		return PORT_AEM_OUT;
	case STATIS_AFM:
		return PORT_AFM_OUT;
	case STATIS_AFL:
		return PORT_AFL_OUT;
	case STATIS_HIST:
		return PORT_BAYER_HIST_OUT;
	case STATIS_HIST2:
		return PORT_FRGB_HIST_OUT;
	case STATIS_PDAF:
		return PORT_PDAF_OUT;
	case STATIS_EBD:
		return PORT_VCH2_OUT;
	case STATIS_LSCM:
		return PORT_LSCM_OUT;
	case STATIS_GTMHIST:
		return PORT_GTM_HIST_OUT;
	default:
		return PORT_DCAM_OUT_MAX;
	}
}

int cam_statis_dcam_port_bufferq_deinit(void *dcam_handle)
{
	int i = 0, j = 0;
	int32_t mfd = 0;
	enum cam_port_dcam_online_out_id port_id = PORT_DCAM_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct dcam_online_node *dcam_node = NULL;
	struct dcam_online_port *dcam_port = NULL;

	dcam_node = (struct dcam_online_node *)dcam_handle;
	for (i = 0; i < ARRAY_SIZE(s_statis_port_info_all); i++) {
		port_id = s_statis_port_info_all[i].port_id;
		stats_type = s_statis_port_info_all[i].buf_type;
		dcam_port = dcam_online_node_get_port(dcam_node, port_id);
		if (!stats_type || !dcam_port)
			continue;
		if (port_id == PORT_VCH2_OUT && dcam_port->raw_src)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dcam_node->statis_buf_array[stats_type][j];
			mfd = ion_buf->mfd;
			if (mfd <= 0)
				continue;

			pr_debug("stats %d,  j %d,  mfd %d, offset %d\n",
				stats_type, j, mfd, ion_buf->offset[0]);
			cam_buf_manager_buf_status_change(ion_buf, CAM_BUF_ALLOC, CAM_IOMMUDEV_DCAM);
			ion_buf->iova = 0UL;
			ion_buf->addr_k = 0UL;
		}
	}

	pr_info("done\n");
	return 0;
}

int cam_statis_dcam_port_bufferq_init(void *dcam_handle)
{
	int ret = 0, i = 0, j = 0;
	enum cam_port_dcam_online_out_id port_id = PORT_DCAM_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	uint32_t res_buf_size = 0;
	struct dcam_online_node *dcam_node = NULL;
	struct dcam_online_port *dcam_port = NULL;

	pr_debug("enter\n");
	dcam_node = (struct dcam_online_node *)dcam_handle;

	for (i = 0; i < ARRAY_SIZE(s_statis_port_info_all); i++) {
		port_id = s_statis_port_info_all[i].port_id;
		stats_type = s_statis_port_info_all[i].buf_type;
		dcam_port = dcam_online_node_get_port(dcam_node, port_id);
		if (!stats_type || !dcam_port)
			continue;

		if (port_id == PORT_VCH2_OUT && dcam_port->raw_src)
			continue;
		if (port_id == PORT_GTM_HIST_OUT && dcam_node->dev->hw->ip_isp->rgb_gtm_support)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dcam_node->statis_buf_array[stats_type][j];
			if (ion_buf->mfd <= 0)
				continue;

			if (stats_type != STATIS_PDAF)
				ret = cam_buf_manager_buf_status_change(ion_buf, CAM_BUF_WITH_IOVA_K_ADDR, CAM_IOMMUDEV_DCAM);
			else
				ret = cam_buf_manager_buf_status_change(ion_buf, CAM_BUF_WITH_IOVA, CAM_IOMMUDEV_DCAM);
			if (ret)
				continue;

			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = 0;
			pframe->irq_property = stats_type;
			pframe->buf = *ion_buf;
			pframe->buf.type = CAM_BUF_NONE;

			ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, pframe, NULL);
			if (ret) {
				pr_info("dcam%d statis %d overflow\n", dcam_node->hw_ctx_id, stats_type);
				cam_queue_empty_frame_put(pframe);
			}

			pr_debug("dcam%d statis %d buf %d kaddr 0x%lx iova 0x%08x, size %x\n",
				dcam_node->hw_ctx_id, stats_type, ion_buf->mfd,
				ion_buf->addr_k, (uint32_t)ion_buf->iova , ion_buf->size);
			if (ion_buf->size > res_buf_size)
				res_buf_size = (uint32_t)ion_buf->size;
		}
	}

	pr_info("done.\n");
	return res_buf_size;
}

int cam_statis_dcam_port_buffer_cfg(
		void *dcam_handle,
		struct isp_statis_buf_input *input)
{
	int ret = 0, i = 0, j = 0;
	int32_t mfd = 0;
	uint32_t offset = 0;
	enum cam_port_dcam_online_out_id port_id = PORT_DCAM_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_online_node *dcam_node = NULL;
	struct dcam_online_port *dcam_port = NULL;

	if (!dcam_handle) {
		pr_err("fail to dcam_online_node is NULL\n");
		return -1;
	}

	dcam_node = (struct dcam_online_node *)dcam_handle;
	if (input->type == STATIS_INIT) {
		memset(&dcam_node->statis_buf_array[0][0], 0, sizeof(dcam_node->statis_buf_array));
		for (i = 0; i < ARRAY_SIZE(s_statis_port_info_all); i++) {
			port_id = s_statis_port_info_all[i].port_id;
			stats_type = s_statis_port_info_all[i].buf_type;
			dcam_port = dcam_online_node_get_port(dcam_node, port_id);
			if (!stats_type || !dcam_port)
				continue;

			if (port_id == PORT_VCH2_OUT && dcam_port->raw_src)
				continue;
			if (port_id == PORT_GTM_HIST_OUT && dcam_node->dev->hw->ip_isp->rgb_gtm_support)
				continue;

			for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
				mfd = input->mfd_array[stats_type][j];

				pr_debug("i %d, type %d, mfd %d, offset %d\n",
					i, stats_type, mfd, input->offset_array[stats_type][j]);

				if (mfd <= 0)
					continue;

				ion_buf = &dcam_node->statis_buf_array[stats_type][j];
				ion_buf->mfd = mfd;
				ion_buf->offset[0] = input->offset_array[stats_type][j];
				ion_buf->type = CAM_BUF_USER;
				if (mfd > 0)
					ion_buf->status = CAM_BUF_ALLOC;

				pr_debug("stats %d, mfd %d, off %d\n",
					stats_type, mfd, ion_buf->offset[0]);
			}
		}
		pr_info("done\n");

	} else {
		port_id = camstatis_dcam_type_to_port_id(input->type);
		if (port_id == PORT_DCAM_OUT_MAX) {
			pr_err("fail to get a valid statis type: %d\n", input->type);
			ret = -EINVAL;
			goto exit;
		}

		dcam_port = dcam_online_node_get_port(dcam_node, port_id);
		if (port_id == PORT_VCH2_OUT && dcam_port->raw_src)
			goto exit;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			mfd = dcam_node->statis_buf_array[input->type][j].mfd;
			offset = dcam_node->statis_buf_array[input->type][j].offset[0];
			if ((mfd > 0) && (mfd == input->mfd)
				&& (offset == input->offset)) {
				ion_buf = &dcam_node->statis_buf_array[input->type][j];
				break;
			}
		}

		if (ion_buf == NULL) {
			pr_err("fail to get statis buf %d, type %d\n",
					input->type, input->mfd);
			ret = -EINVAL;
			goto exit;
		}

		pframe = cam_queue_empty_frame_get();
		pframe->irq_property = input->type;
		pframe->buf = *ion_buf;
		pframe->buf.type = CAM_BUF_NONE;

		ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, pframe, NULL);
		pr_debug("statis %d, mfd %d, off %d, iova 0x%08x,  kaddr 0x%lx\n",
			input->type, mfd, offset,
			(uint32_t)pframe->buf.iova, pframe->buf.addr_k);

		if (ret)
			cam_queue_empty_frame_put(pframe);
	}
exit:
	return ret;
}

int cam_statis_dcam_port_buffer_skip_cfg(void *dcam_handle, struct camera_frame *pframe)
{
	int ret = 0;
	enum cam_port_dcam_online_out_id port_id = PORT_DCAM_OUT_MAX;
	struct dcam_online_node *dcam_node = NULL;
	struct dcam_online_port *dcam_port = NULL;

	dcam_node = (struct dcam_online_node *)dcam_handle;
	port_id = camstatis_dcam_type_to_port_id(pframe->irq_property);
	dcam_port = dcam_online_node_get_port(dcam_node, port_id);

	if (port_id == PORT_DCAM_OUT_MAX || !dcam_port) {
		pr_err("invalid statis type: %d, dcam_port %p\n", pframe->irq_property, dcam_port);
		ret = -EINVAL;
		goto exit;
	}

	ret = cam_buf_manager_buf_enqueue(&dcam_port->unprocess_pool, pframe, NULL);
exit:
	return ret;
}

static int cam_statis_isp_q_init(void *isp_handle, void *node,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int j = 0;
	int32_t mfd = 0;
	enum isp_statis_buf_type stats_type;
	struct isp_pipe_dev *dev = NULL;
	struct isp_node *inode = NULL;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_queue *statis_q = NULL;

	dev = (struct isp_pipe_dev *)isp_handle;
	inode = (struct isp_node *)node;

	memset(&inode->statis_buf_array[0][0], 0, sizeof(inode->statis_buf_array));

	for (stats_type = STATIS_HIST2; stats_type <= STATIS_GTMHIST; stats_type++) {
		if ((stats_type == STATIS_GTMHIST) && (inode->dev->isp_hw->ip_isp->rgb_gtm_support == 0))
			continue;
		if (stats_type == STATIS_HIST2)
			statis_q = &inode->hist2_result_queue;
		else if (stats_type == STATIS_GTMHIST)
			statis_q = &inode->gtmhist_result_queue;
		else
			continue;
		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			mfd = input->mfd_array[stats_type][j];
			if (mfd <= 0)
				continue;
			ion_buf = &inode->statis_buf_array[stats_type][j];
			ion_buf->mfd = mfd;
			ion_buf->offset[0] = input->offset_array[stats_type][j];
			ion_buf->type = CAM_BUF_USER;
			ret = cam_buf_ionbuf_get(ion_buf);
			if (ret) {
				memset(ion_buf, 0, sizeof(struct camera_buf));
				continue;
			}

			ret = cam_buf_kmap(ion_buf);
			if (ret) {
				cam_buf_ionbuf_put(ion_buf);
				memset(ion_buf, 0, sizeof(struct camera_buf));
				continue;
			}

			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = inode->ch_id;
			pframe->irq_property = stats_type;
			pframe->buf = *ion_buf;

			ret = cam_queue_enqueue(statis_q, &pframe->list);
			if (ret) {
				pr_info("statis %d overflow\n", stats_type);
				cam_queue_empty_frame_put(pframe);
			}

			pr_debug("buf_num %d, buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
				j, ion_buf->mfd, ion_buf->offset[0],
				ion_buf->addr_k, (uint32_t)ion_buf->iova);
		}
	}

	return ret;
}

int cam_statis_isp_buffer_cfg(void *isp_handle, void *node,
		struct isp_statis_buf_input *input)
{
	int ret = 0;
	int j = 0;
	int32_t mfd = 0;
	uint32_t offset = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_node *inode = NULL;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	struct camera_queue *statis_q = NULL;

	if (!isp_handle || !node || !input) {
		pr_info("isp_handle=%p, cxt_id=%d\n", isp_handle, node);
		return 0;
	}

	dev = (struct isp_pipe_dev *)isp_handle;
	inode = VOID_PTR_TO(node, struct isp_node);

	if (atomic_read(&inode->user_cnt) == 0) {
		pr_info("isp ctx %d is not enable\n", inode->node_id);
		return 0;
	}

	if (input->type == STATIS_INIT) {
		cam_statis_isp_q_init(isp_handle, inode, input);
		pr_debug("init done\n");
		return 0;
	}
	if ((input->type == STATIS_GTMHIST) && (inode->dev->isp_hw->ip_isp->rgb_gtm_support == 0))
		return 0;

	if (input->type == STATIS_HIST2)
		statis_q = &inode->hist2_result_queue;
	else if (input->type == STATIS_GTMHIST)
		statis_q = &inode->gtmhist_result_queue;
	else {
		pr_warn("statis type %d not support\n");
		return 0;
	}

	for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
		mfd = inode->statis_buf_array[input->type][j].mfd;
		offset = inode->statis_buf_array[input->type][j].offset[0];
		if ((mfd > 0) && (mfd == input->mfd)
			&& (offset == input->offset)) {
			ion_buf = &inode->statis_buf_array[input->type][j];
			break;
		}
	}

	if (ion_buf == NULL) {
		pr_err("fail to get statis buf %d, type %d\n",
				input->type, input->mfd);
		ret = -EINVAL;
		return ret;
	}

	pframe = cam_queue_empty_frame_get();
	pframe->channel_id = inode->ch_id;
	pframe->irq_property = input->type;
	pframe->buf = *ion_buf;
	ret = cam_queue_enqueue(statis_q, &pframe->list);
	if (ret) {
		pr_info("statis %d overflow\n", input->type);
		cam_queue_empty_frame_put(pframe);
	}
	pr_debug("buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
		ion_buf->mfd, ion_buf->offset[0],
		ion_buf->addr_k, (uint32_t)ion_buf->iova);
	return 0;
}

int cam_statis_isp_buffer_unmap(void *isp_handle, void *node)
{
	int ret = 0;
	int j = 0;
	struct isp_pipe_dev *dev = NULL;
	struct isp_node *inode = NULL;
	struct camera_buf *ion_buf = NULL;
	enum isp_statis_buf_type stats_type = {0};

	dev = (struct isp_pipe_dev *)isp_handle;
	inode = VOID_PTR_TO(node, struct isp_node);

	pr_debug("enter\n");

	for (stats_type = STATIS_HIST2; stats_type <= STATIS_GTMHIST; stats_type++) {
		if ((stats_type == STATIS_GTMHIST) && (inode->dev->isp_hw->ip_isp->rgb_gtm_support == 0))
			continue;
		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &inode->statis_buf_array[stats_type][j];
			if (ion_buf->mfd <= 0) {
				memset(ion_buf, 0, sizeof(struct camera_buf));
				continue;
			}

			pr_info("ctx %d free buf %d, off %d, kaddr 0x%lx iova 0x%08x\n",
				inode->node_id, ion_buf->mfd, ion_buf->offset[0],
				ion_buf->addr_k, (uint32_t)ion_buf->iova);

			cam_buf_kunmap(ion_buf);
			cam_buf_ionbuf_put(ion_buf);
			memset(ion_buf, 0, sizeof(struct camera_buf));
		}
	}
	pr_debug("done.\n");
	return ret;
}

void cam_statis_isp_buf_destroy(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get input ptr.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	cam_queue_empty_frame_put(frame);
}
