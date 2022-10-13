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

#include "cam_offline_statis.h"
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
struct offlinestatis_port_buf_info s_offlinestatis_port_info_all[] = {
	{PORT_OFFLINE_PDAF_OUT,      0,  0, STATIS_PDAF},
	{PORT_OFFLINE_AEM_OUT,       0,  0, STATIS_AEM},
	{PORT_OFFLINE_AFM_OUT,       0,  0, STATIS_AFM},
	{PORT_OFFLINE_AFL_OUT,       0,  0, STATIS_AFL},
	{PORT_OFFLINE_BAYER_HIST_OUT,0,  0, STATIS_HIST},
	{PORT_OFFLINE_FRGB_HIST_OUT, 0,  0, STATIS_HIST2},
	{PORT_OFFLINE_LSCM_OUT,      0,  0, STATIS_LSCM},
	{PORT_OFFLINE_GTM_HIST_OUT,  0,  0, STATIS_GTMHIST},
};

static enum cam_port_dcam_offline_out_id camoffstatis_dcam_type_to_port_id(enum isp_statis_buf_type type)
{
	switch (type) {
	case STATIS_AEM:
		return PORT_OFFLINE_AEM_OUT;
	case STATIS_AFM:
		return PORT_OFFLINE_AFM_OUT;
	case STATIS_AFL:
		return PORT_OFFLINE_AFL_OUT;
	case STATIS_HIST:
		return PORT_OFFLINE_BAYER_HIST_OUT;
	case STATIS_HIST2:
		return PORT_OFFLINE_FRGB_HIST_OUT;
	case STATIS_PDAF:
		return PORT_OFFLINE_PDAF_OUT;
	case STATIS_LSCM:
		return PORT_OFFLINE_LSCM_OUT;
	case STATIS_GTMHIST:
		return PORT_OFFLINE_GTM_HIST_OUT;
	default:
		return PORT_DCAM_OFFLINE_OUT_MAX;
	}
}

static void camoffstatis_dcam_buf_destroy(void *param)
{
	struct camera_frame *frame = NULL;

	if (!param) {
		pr_err("fail to get valid param.\n");
		return;
	}

	frame = (struct camera_frame *)param;
	if (frame->is_reserved) {
		cam_buf_iommu_unmap(&frame->buf);
		if (frame->is_reserved == CAM_RESERVED_BUFFER_ORI)
			cam_buf_ionbuf_put(&frame->buf);
	}

	cam_queue_empty_frame_put(frame);
}

static int camoffstatis_dcam_port_buffer_unmap(void *dcam_handle)
{
	int i = 0, j = 0;
	int32_t mfd = 0;
	enum cam_port_dcam_offline_out_id port_id = PORT_DCAM_OFFLINE_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct dcam_offline_node *dcam_node = NULL;
	struct dcam_offline_port *dcam_port = NULL;

	dcam_node = (struct dcam_offline_node *)dcam_handle;
	for (i = 0; i < ARRAY_SIZE(s_offlinestatis_port_info_all); i++) {
		port_id = s_offlinestatis_port_info_all[i].port_id;
		stats_type = s_offlinestatis_port_info_all[i].buf_type;
		dcam_port = dcam_offline_node_port_get(dcam_node, port_id);
		if (!stats_type || !dcam_port)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dcam_node->statis_buf_array[stats_type][j];
			mfd = ion_buf->mfd;
			if (mfd <= 0)
				continue;

			pr_debug("stats %d,  j %d,  mfd %d, offset %d\n",
				stats_type, j, mfd, ion_buf->offset[0]);
			if (ion_buf->mapping_state & CAM_BUF_MAPPING_KERNEL)
				cam_buf_kunmap(ion_buf);
			if (ion_buf->mapping_state & CAM_BUF_MAPPING_DEV)
				cam_buf_iommu_unmap(ion_buf);
			cam_buf_ionbuf_put(ion_buf);
			ion_buf->iova = 0UL;
			ion_buf->addr_k = 0UL;
		}
	}

	pr_info("done\n");
	return 0;
}

int camoffline_statis_dcam_port_bufferq_init(void *dcam_handle)
{
	int ret = 0, i = 0, j = 0;
	enum cam_port_dcam_offline_out_id port_id = PORT_DCAM_OFFLINE_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	uint32_t res_buf_size = 0;
	struct dcam_offline_node *dcam_node = NULL;
	struct dcam_offline_port *dcam_port = NULL;

	pr_debug("enter\n");
	dcam_node = (struct dcam_offline_node *)dcam_handle;
	for (i = 0; i < ARRAY_SIZE(s_offlinestatis_port_info_all); i++) {
		port_id = s_offlinestatis_port_info_all[i].port_id;
		stats_type = s_offlinestatis_port_info_all[i].buf_type;
		dcam_port = dcam_offline_node_port_get(dcam_node, port_id);
		if (!stats_type || !dcam_port)
			continue;

		cam_queue_init(&dcam_port->out_buf_queue, DCAM_OUT_BUF_Q_LEN, camoffstatis_dcam_buf_destroy);
		cam_queue_init(&dcam_port->result_queue, DCAM_RESULT_Q_LEN, camoffstatis_dcam_buf_destroy);
	}

	for (i = 0; i < ARRAY_SIZE(s_offlinestatis_port_info_all); i++) {

		port_id = s_offlinestatis_port_info_all[i].port_id;
		stats_type = s_offlinestatis_port_info_all[i].buf_type;
		dcam_port = dcam_offline_node_port_get(dcam_node, port_id);
		if (!stats_type || !dcam_port)
			continue;

		for (j = 0; j < STATIS_BUF_NUM_MAX; j++) {
			ion_buf = &dcam_node->statis_buf_array[stats_type][j];
			if (ion_buf->mfd <= 0)
				continue;

			ret = cam_buf_ionbuf_get(ion_buf);
			if (ret) {
				continue;
			}

			ret = cam_buf_iommu_map(ion_buf, CAM_IOMMUDEV_DCAM);
			if (ret) {
				cam_buf_ionbuf_put(ion_buf);
				continue;
			}

			if (stats_type != STATIS_PDAF) {
				ret = cam_buf_kmap(ion_buf);
				if (ret) {
					pr_err("fail to kmap statis buf %d\n", ion_buf->mfd);
					ion_buf->addr_k = 0UL;
				}
			}

			pframe = cam_queue_empty_frame_get();
			pframe->channel_id = 0;
			pframe->irq_property = stats_type;
			pframe->buf = *ion_buf;

			ret = cam_queue_enqueue(&dcam_port->out_buf_queue, &pframe->list);
			if (ret) {
				pr_info("dcam%d statis %d overflow\n", dcam_node->hw_ctx_id, stats_type);
				cam_queue_empty_frame_put(pframe);
			}

			pr_debug("dcam%d statis %d buf %d kaddr 0x%lx iova 0x%08x\n",
				dcam_node->hw_ctx_id, stats_type, ion_buf->mfd,
				ion_buf->addr_k, (uint32_t)ion_buf->iova);
			if (ion_buf->size > res_buf_size)
				res_buf_size = (uint32_t)ion_buf->size;
		}
	}

	pr_info("done.\n");
	return res_buf_size;
}

int camoffline_statis_dcam_port_bufferq_deinit(void *dcam_handle)
{
	int ret = 0, i = 0;
	enum cam_port_dcam_offline_out_id port_id = PORT_DCAM_OFFLINE_OUT_MAX;
	struct dcam_offline_node *dcam_node = NULL;
	struct dcam_offline_port *dcam_port = NULL;

	pr_info("enter\n");
	dcam_node = (struct dcam_offline_node *)dcam_handle;
	for (i = 0; i < ARRAY_SIZE(s_offlinestatis_port_info_all); i++) {
		port_id = s_offlinestatis_port_info_all[i].port_id;
		dcam_port = dcam_offline_node_port_get(dcam_node, port_id);

		pr_debug("port_id[%d] i[%d]\n", port_id, i);
		if (!dcam_port || port_id == PORT_DCAM_OUT_MAX)
			continue;

		atomic_set(&dcam_port->is_work, 0);
		cam_queue_clear(&dcam_port->out_buf_queue, struct camera_frame, list);
		cam_queue_clear(&dcam_port->result_queue, struct camera_frame, list);
	}

	camoffstatis_dcam_port_buffer_unmap(dcam_handle);

	pr_info("done.\n");
	return ret;
}

int camoffline_statis_dcam_port_buffer_cfg(
		void *dcam_handle,
		struct isp_statis_buf_input *input)
{
	int ret = 0, i = 0, j = 0;
	int32_t mfd = 0;
	uint32_t offset = 0;
	enum cam_port_dcam_offline_out_id port_id = PORT_DCAM_OFFLINE_OUT_MAX;
	enum isp_statis_buf_type stats_type = STATIS_INIT;
	struct camera_buf *ion_buf = NULL;
	struct camera_frame *pframe = NULL;
	struct dcam_offline_node *dcam_node = NULL;
	struct dcam_offline_port *dcam_port = NULL;

	if (!dcam_handle) {
		pr_err("fail to dcam_online_node is NULL\n");
		return -1;
	}

	dcam_node = (struct dcam_offline_node *)dcam_handle;
	if (input->type == STATIS_INIT) {
		memset(&dcam_node->statis_buf_array[0][0], 0, sizeof(dcam_node->statis_buf_array));
		for (i = 0; i < ARRAY_SIZE(s_offlinestatis_port_info_all); i++) {
			port_id = s_offlinestatis_port_info_all[i].port_id;
			stats_type = s_offlinestatis_port_info_all[i].buf_type;
			dcam_port = dcam_offline_node_port_get(dcam_node, port_id);
			if (!stats_type || !dcam_port)
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

				pr_debug("stats %d, mfd %d, off %d\n",
					stats_type, mfd, ion_buf->offset[0]);
			}
		}
		pr_info("done\n");

	} else {
		port_id = camoffstatis_dcam_type_to_port_id(input->type);
		if (port_id == PORT_DCAM_OUT_MAX) {
			pr_err("fail to get a valid statis type: %d\n", input->type);
			ret = -EINVAL;
			goto exit;
		}

		dcam_port = dcam_offline_node_port_get(dcam_node, port_id);

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
		ret = cam_queue_enqueue(&dcam_port->out_buf_queue, &pframe->list);
		pr_debug("statis %d, mfd %d, off %d, iova 0x%08x,  kaddr 0x%lx\n",
			input->type, mfd, offset,
			(uint32_t)pframe->buf.iova, pframe->buf.addr_k);

		if (ret)
			cam_queue_empty_frame_put(pframe);
	}
exit:
	return ret;
}

