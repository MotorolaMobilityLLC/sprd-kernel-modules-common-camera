/*
 * Copyright (C) 2016 Spreadtrum Communications Inc.
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

#include <linux/err.h>
#include <linux/sprd_ion.h>
#include <linux/sprd_iommu.h>
#include <linux/vmalloc.h>

#include "isp_buf.h"
#include "isp_statis_buf.h"
#include "ion.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ISP_BUF (%d): %d %s: "\
	fmt, current->pid, __LINE__, __func__

#define STATIS_QUEUE_LENGTH		8
#define ISP_IMG_QUEUE_LEN		8

int isp_frame_enqueue(struct isp_frm_queue *queue,
	struct camera_frame *frame)
{
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -1;
	}

	spin_lock_irqsave(&queue->lock, flags);
	pr_debug("queue->valid_cnt %d, r_index %d, w_index %d\n",
		queue->valid_cnt, queue->r_index, queue->w_index);
	if (queue->valid_cnt >= isp_frm_queue_len) {
		pr_err("fail to enqueue %s, frm_type:%d\n",
		       queue->owner, frame->type);
		spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	memcpy(&queue->frame[queue->w_index], frame, sizeof(*frame));
	queue->w_index++;
	if (queue->w_index  == isp_frm_queue_len)
		queue->w_index = 0;

	queue->valid_cnt++;
	pr_debug("en queue, %d, %d, 0x%x, 0x%x\n",
		   (0xF & frame->fid),
		   queue->valid_cnt, frame->yaddr, frame->uaddr);
	spin_unlock_irqrestore(&queue->lock, flags);
	return 0;
}

int isp_frame_dequeue(struct isp_frm_queue *queue,
	struct camera_frame *frame)
{
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -1;
	}

	spin_lock_irqsave(&queue->lock, flags);
	pr_debug("queue->valid_cnt %d, r_index %d, w_index %d\n",
		queue->valid_cnt, queue->r_index, queue->w_index);

	if (queue->valid_cnt == 0) {
		pr_debug("fail to dequeue %s, frm_type:%d, cb %pS\n",
		       queue->owner, frame->type, __builtin_return_address(0));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	memcpy(frame, &queue->frame[queue->r_index], sizeof(*frame));
	queue->valid_cnt--;
	queue->r_index++;

	if (queue->r_index == isp_frm_queue_len)
		queue->r_index = 0;

	pr_debug("de queue, %d, %d\n",
		   (0xF & (frame)->fid), queue->valid_cnt);
	spin_unlock_irqrestore(&queue->lock, flags);
	return 0;
}

void isp_frm_queue_clear(struct isp_frm_queue *queue)
{
	if (ISP_ADDR_INVALID(queue)) {
		pr_err("fail to clear que,invalid heap %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct isp_frm_queue));
	spin_lock_init(&queue->lock);
}

int isp_queue_init(struct isp_queue *queue)
{
	if (queue == NULL)
		return -EINVAL;

	memset(queue, 0x00, sizeof(*queue));
	queue->write = &queue->node[0];
	queue->read  = &queue->node[0];

	return 0;
}

int32_t isp_queue_read(struct isp_queue *queue, struct isp_node *node)
{
	if (NULL == queue || NULL == node) {
		pr_err("fail to get queue node %p %p\n",
			queue, node);
		return -1;
	}

	if (queue->read != queue->write) {
		*node = *queue->read++;
		if (queue->read > &queue->node[ISP_QUEUE_LENGTH-1])
			queue->read = &queue->node[0];
	}

	return 0;
}

void isp_buf_queue_init(struct isp_buf_queue *queue)
{
	if (ISP_ADDR_INVALID(queue)) {
		pr_err("fail to get queue %p\n", queue);
		return;
	}

	memset((void *)queue, 0, sizeof(struct isp_buf_queue));
	spin_lock_init(&queue->lock);
}

int isp_buf_queue_peek(struct isp_buf_queue *queue,
		       struct camera_frame *frame)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);

	if (queue->valid_cnt == 0) {
		pr_debug("warning, read wait new node write cb %pS\n",
				__builtin_return_address(0));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -EAGAIN;
	}

	memcpy(frame, &queue->frame[queue->r_index], sizeof(*frame));

	pr_debug("valid_cnt:%d read frame buf ufid %u\n",
		 queue->valid_cnt, frame->user_fid);
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int isp_buf_queue_read_if(struct isp_buf_queue *queue,
			  bool (*filter)(struct camera_frame *, void *),
			  void *data,
			  struct camera_frame *frame)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	pr_debug("queue->valid_cnt %d, r_index %d, w_index %d, cb:%pS\n",
		queue->valid_cnt, queue->r_index, queue->w_index,
		__builtin_return_address(0));

	if (queue->valid_cnt == 0) {
		pr_debug("warning, read wait new node write cb %pS\n",
				__builtin_return_address(0));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -EAGAIN;
	}

	memcpy(frame, &queue->frame[queue->r_index], sizeof(*frame));

	if (!filter(frame, data)) {
		ret = -EINVAL;
		goto unlock;
	}

	queue->valid_cnt--;
	queue->r_index++;

	if (queue->r_index == DCAM_FRM_CNT_MAX)
		queue->r_index = 0;

	pr_debug("read buf queue type %x fid %d valid_cnt %d\n",
			frame->type, frame->fid, queue->valid_cnt);

unlock:
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int isp_buf_queue_read(struct isp_buf_queue *queue,
	struct camera_frame *frame)
{
	int ret = ISP_RTN_SUCCESS;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	pr_debug("queue->valid_cnt %d, r_index %d, w_index %d, cb:%pS\n",
		queue->valid_cnt, queue->r_index, queue->w_index,
		__builtin_return_address(0));

	if (queue->valid_cnt == 0) {
		pr_debug("warning, read wait new node write cb %pS\n",
				__builtin_return_address(0));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -EAGAIN;
	}

	memcpy(frame, &queue->frame[queue->r_index], sizeof(*frame));
	queue->valid_cnt--;
	queue->r_index++;

	if (queue->r_index == DCAM_FRM_CNT_MAX)
		queue->r_index = 0;

	pr_debug("read buf queue type %x fid %d valid_cnt %d\n",
			frame->type, frame->fid, queue->valid_cnt);
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int isp_buf_queue_write(struct isp_buf_queue *queue,
	struct camera_frame *frame)
{
	int ret = 0;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(frame)) {
		pr_err("fail to get valid parm %p,%p\n", queue, frame);
		return -EINVAL;
	}

	spin_lock_irqsave(&queue->lock, flags);
	pr_debug("queue->valid_cnt %d, r_index %d, w_index %d, cb:%pS\n",
		queue->valid_cnt, queue->r_index, queue->w_index,
		 __builtin_return_address(0));
	if (queue->valid_cnt >= DCAM_FRM_CNT_MAX) {
		pr_warn("warning, queue is full, type %d, y_addr 0x%x, cb %pS\n",
			frame->type, frame->yaddr,
			 __builtin_return_address(0));
		spin_unlock_irqrestore(&queue->lock, flags);
		return -1;
	}

	memcpy(&queue->frame[queue->w_index], frame, sizeof(*frame));
	queue->w_index++;
	if (queue->w_index  == DCAM_FRM_CNT_MAX)
		queue->w_index = 0;

	queue->valid_cnt++;
	pr_debug("write buf queue type:%x fid:%d index:%x, cnt %d\n",
		 frame->type, frame->fid, queue->w_index, queue->valid_cnt);
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

static int __isp_coeff_queue_init(struct isp_sc_coeff_queue *queue)
{
	unsigned int i = 0;
	int ret = 0;
	struct isp_sc_coeff *t_sc_coeff;
	size_t t_sc_coeff_split_size;

	if (ISP_ADDR_INVALID(queue)) {
		pr_err("fail to get valid que %p\n", queue);
		return -EINVAL;
	}

	t_sc_coeff_split_size = sizeof(struct isp_sc_coeff) / 2;
	for (i = 0; i < ISP_SC_COEFF_BUF_COUNT; i++) {
		t_sc_coeff = &queue->coeff[i];
		/* TODO: this is just to fool sparse..., actually sparse has
		 * got a point, sc code is not good. Come back to this later.
		 */
		memset((void *)t_sc_coeff, 0, t_sc_coeff_split_size);
		memset((void *)((char *)t_sc_coeff + t_sc_coeff_split_size), 0,
			t_sc_coeff_split_size);
	}
	queue->write = &queue->coeff[0];
	queue->read = &queue->coeff[0];
	queue->w_index = 0;
	queue->r_index = 0;
	spin_lock_init(&queue->lock);
	return ret;
}

int isp_coeff_queue_init(struct isp_sc_array *scl_array)
{
	int ret;

	if (unlikely(scl_array == NULL)) {
		pr_err("fail to get scl_array\n");
		return -EINVAL;
	}

	ret = __isp_coeff_queue_init(&scl_array->pre_queue);
	if (unlikely(ret))
		return ret;

	ret = __isp_coeff_queue_init(&scl_array->vid_queue);
	if (unlikely(ret))
		return ret;

	ret = __isp_coeff_queue_init(&scl_array->cap_queue);
	if (unlikely(ret))
		return ret;

	return ret;
}


int isp_coeff_get_new_node(struct isp_sc_coeff_queue *queue,
			   struct isp_sc_coeff **coeff, int type)
{
	int ret = 0;
	unsigned long flags;
	struct isp_sc_coeff *ori;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(coeff)) {
		pr_err("fail to get valid parm %p %p\n", queue, coeff);
		return -EINVAL;
	}

	pr_debug("get new node\n");

	spin_lock_irqsave(&queue->lock, flags);

	*coeff = queue->write;
	ori = queue->write;
	if (type) {
		queue->write++;
		queue->w_index++;
		if (queue->write > &queue->coeff[ISP_SC_COEFF_BUF_COUNT - 1]) {
			queue->write = &queue->coeff[0];
			queue->w_index = 0;
		}
		if (queue->write == queue->read) {
			queue->write = ori;
			pr_warn("warning, queue is full\n");
			ret = -EAGAIN;
		}
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

int isp_coeff_get_valid_node(struct isp_sc_coeff_queue *queue,
			     struct isp_sc_coeff **coeff, int type)
{
	int ret = 0;
	unsigned long flags;

	if (ISP_ADDR_INVALID(queue) || ISP_ADDR_INVALID(coeff)) {
		pr_err("fail to valid parm %p %p\n", queue, coeff);
		return -EINVAL;
	}

	pr_debug("read buf queue\n");

	spin_lock_irqsave(&queue->lock, flags);

	if (queue->read != queue->write) {
		*coeff = queue->read;
		if (type) {
			queue->read++;
			queue->r_index++;
			if (queue->read
			    > &queue->coeff[ISP_SC_COEFF_BUF_COUNT - 1]) {
				queue->read = &queue->coeff[0];
				queue->r_index = 0;
			}
		}
	} else {
		ret = -EAGAIN;
		pr_debug("warning, queue is null\n");
	}
	spin_unlock_irqrestore(&queue->lock, flags);

	return ret;
}

void isp_frm_clear(struct isp_pipe_dev *dev, enum isp_path_index path_index)
{
	struct camera_frame frame, *res_frame;
	struct isp_path_desc *path;
	struct isp_module *module = NULL;
	struct isp_statis_module *statis_module = NULL;
	unsigned int rtn;

	if (!dev)
		return;

	module = &dev->module_info;
	statis_module = &dev->statis_module_info;
	if (ISP_PATH_IDX_PRE & path_index) {
		path = &module->isp_path[ISP_SCL_PRE];
		while (!isp_frame_dequeue(&path->frame_queue, &frame)) {
			if (pfiommu_free_addr(&frame.pfinfo))
				pr_err("fail to free pre path frame queue buf\n");
			memset(&frame, 0, sizeof(struct camera_frame));
		}

		isp_frm_queue_clear(&path->frame_queue);
		isp_buf_queue_init(&path->buf_queue);
		res_frame = &module->path_reserved_frame[ISP_SCL_PRE];
		if (res_frame->pfinfo.mfd[0] != 0){
			rtn = pfiommu_free_addr(&res_frame->pfinfo);
			if (rtn)
				pr_err("fail to free pre path reserved buf\n");
		}
		memset((void *)res_frame, 0, sizeof(struct camera_frame));

	}

	if (ISP_PATH_IDX_VID & path_index) {
		path = &module->isp_path[ISP_SCL_VID];
		while (!isp_frame_dequeue(&path->frame_queue, &frame)) {
			if (pfiommu_free_addr(&frame.pfinfo))
				pr_err("fail to free vid path frame queue buf\n");
			memset(&frame, 0, sizeof(struct camera_frame));
		}

		isp_frm_queue_clear(&path->frame_queue);
		isp_buf_queue_init(&path->buf_queue);
		res_frame = &module->path_reserved_frame[ISP_SCL_VID];
		rtn = pfiommu_free_addr(&res_frame->pfinfo);
		if (rtn)
			pr_err("fail to free vid path reserved buf\n");
		memset((void *)res_frame, 0, sizeof(struct camera_frame));
	}

	if (ISP_PATH_IDX_CAP & path_index) {
		path = &module->isp_path[ISP_SCL_CAP];
		while (!isp_frame_dequeue(&path->frame_queue, &frame)) {
			if (pfiommu_free_addr(&frame.pfinfo))
				pr_err("fail to free cap path frame queue buf\n");;
			memset(&frame, 0, sizeof(struct camera_frame));
		}

		isp_frm_queue_clear(&path->frame_queue);
		isp_buf_queue_init(&path->buf_queue);
		res_frame = &module->path_reserved_frame[ISP_SCL_CAP];
		rtn = pfiommu_free_addr(&res_frame->pfinfo);
		if (rtn)
			pr_err("fail to free cap path reserved buf\n");
		memset((void *)res_frame, 0, sizeof(struct camera_frame));
	}
}

/*
 * buffer handling for offline transfer
 */
struct offline_buf_desc *isp_offline_sel_buf(
	struct isp_offline_desc *off_desc, uint8_t off_type)
{
	if (off_type == ISP_OFF_BUF_BIN)
		return &off_desc->buf_desc_bin;
	else if (off_type == ISP_OFF_BUF_FULL)
		return &off_desc->buf_desc_full;

	pr_err("fail to get Input buf type\n");

	return NULL;
}

int get_off_frm_q_len(struct isp_offline_desc *off_desc,
		     unsigned int *len)
{
	struct isp_pipe_dev *dev = NULL;
	struct isp_module *module = NULL;
	unsigned int tmp_len;

	module = container_of(off_desc, struct isp_module, off_desc);
	if (IS_ERR_OR_NULL(module))
		goto exit;
	dev = container_of(module, struct isp_pipe_dev, module_info);
	if (IS_ERR_OR_NULL(dev))
		goto exit;

	if (is_dual_cam)
		tmp_len = isp_frm_queue_len;
	else if (dev->is_hdr)
		tmp_len = isp_frm_queue_len - 1;
	else
		tmp_len = isp_frm_queue_len - 2;

	*len = tmp_len;
	pr_debug("off_frm_q_len:%d\n", tmp_len);

	return 0;

exit:
	pr_info("error: exit\n");
	return -EFAULT;
}

int isp_offline_init_buf(struct isp_offline_desc *off_desc,
	uint8_t off_type, bool queue_only)
{
	int ret = 0;
	unsigned int num = 0;
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	unsigned int frm_q_len;

	if (!off_desc) {
		pr_err("fail to get Input ptr is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	isp_buf_queue_init(&buf_desc->tmp_buf_queue);
	isp_frm_queue_clear(&buf_desc->frame_queue);
	isp_frm_queue_clear(&buf_desc->zsl_queue);
	buf_desc->output_frame_count = 0;
	buf_desc->frame_queue.owner = (off_type == ISP_OFF_BUF_BIN ?
				       "off_bin_frm_q" : "off_full_frm_q");
	buf_desc->zsl_queue.owner = (off_type == ISP_OFF_BUF_BIN ?
				     "off_bin_zsl_q" : "off_full_zsl_q");

	if (!queue_only) {
		for (num = 0; num < frm_q_len; num++) {
			ion_buf = &buf_desc->ion_buf[num];
			ion_buf->dmabuf_p = NULL;
			ion_buf->buf = NULL;
		}
	}
	return ret;
}

int isp_offline_get_buf(struct isp_offline_desc *off_desc,
	uint8_t off_type)
{
	int ret = 0;
	unsigned long phys_addr = 0;
	unsigned int num = 0;
	char name[32];
	size_t buf_len;
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	struct camera_frame frame = {0};
	unsigned int frm_q_len;
	int heap_type;

	if (!off_desc) {
		pr_err("fail to get offline buf is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	pr_debug("+\n");

	/* TODO: consider to handle both kinds of buffer */
	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	if (!buf_desc->buf_len) {
		pr_err("fail to get buf size error\n");
		return -EFAULT;
	}

	for (num = 0; num < frm_q_len; num++) {
		ion_buf = &buf_desc->ion_buf[num];
		if (ion_buf->dmabuf_p != NULL) {
			pr_info("offline buffer %d has been allocated (%s).\n",
				num,
				((off_type == ISP_OFF_BUF_BIN) ? "bin_path" :
				"full_path"));
			if (sprd_iommu_attach_device(&s_isp_pdev->dev) != 0) {
			#if 0
				frame.yaddr = ion_buf->addr.yaddr;
			#endif
				frame.yaddr_vir = ion_buf->addr.yaddr_vir;
				isp_buf_queue_write(
					&buf_desc->tmp_buf_queue,
					&frame);
			}
			continue;
		}

		sprintf(name, "sprd-cam-off-%c-%d",
			off_type == ISP_OFF_BUF_BIN ? 'p' : 'c', num);

		heap_type = sprd_iommu_attach_device(&s_isp_pdev->dev) ?
			ION_HEAP_ID_MASK_MM :
			ION_HEAP_ID_MASK_SYSTEM;

		ion_buf->dmabuf_p = ion_new_alloc(buf_desc->buf_len,
				heap_type, 0);
		if (IS_ERR_OR_NULL(ion_buf->dmabuf_p)) {
			pr_err("fail to alloc ion buf size = 0x%x %ld\n",
				(int)buf_desc->buf_len,
				(unsigned long)ion_buf->dmabuf_p);
			goto failed;
		}

		ret = sprd_ion_get_buffer(-1,
				ion_buf->dmabuf_p,
				&ion_buf->buf,
				&ion_buf->buf_size);
		if (ret) {
			pr_err("fail to get ion buf for kernel buffer %p\n",
				ion_buf->dmabuf_p);
			goto failed;
		}

		pr_debug("dmabuf_p[%p], size 0x%x, heap %d\n",
			ion_buf->dmabuf_p,
			(int)buf_desc->buf_len,
			heap_type);

		if (sprd_iommu_attach_device(&s_isp_pdev->dev)) {
			ret = sprd_ion_get_phys_addr_by_db(ion_buf->dmabuf_p,
					&phys_addr,
					&buf_len);
			if (ret) {
				pr_err("fail to get phys addr\n");
				goto failed;
			}

			pr_info("off_b rvd: 0x%lx 0x%x type: %x\n",
				phys_addr, buf_len, off_type);

			frame.fid = num;
			frame.type = (off_type == ISP_OFF_BUF_FULL) ? 0xf : 0xb;
			frame.yaddr_vir = frame.uaddr_vir =
				phys_addr - MM_ION_OFFSET;
			ion_buf->addr.yaddr_vir = ion_buf->addr.uaddr_vir =
				frame.yaddr_vir;
			ret = isp_buf_queue_write(&buf_desc->tmp_buf_queue,
						  &frame);
			if (ret)
				goto failed;

			buf_desc->output_frame_count++;
		}
	}

	pr_debug("-\n");
	return 0;

failed:
	/* release all allocated buffers immediately */
	ret = isp_offline_put_buf(off_desc, off_type);

	pr_err("fail to get offline buffer\n");
	return -EFAULT;
}

int isp_offline_put_buf(struct isp_offline_desc *off_desc,
	uint8_t off_type)
{
	unsigned int num = 0;
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	unsigned int frm_q_len;

	if (!off_desc) {
		pr_err("fail to get offline buf is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	for (num = 0; num < frm_q_len; num++) {
		ion_buf = &buf_desc->ion_buf[num];
		if (IS_ERR_OR_NULL(ion_buf->dmabuf_p))
			continue;

		ion_free(ion_buf->dmabuf_p);
		ion_buf->dmabuf_p = NULL;
		ion_buf->buf = NULL;
		ion_buf->buf_size = 0;
	}

	return 0;
}

int isp_offline_buf_iommu_map(struct isp_offline_desc *off_desc,
	uint8_t off_type)
{
	int ret = 0;
	unsigned int num = 0;
	struct camera_frame frame = {0};
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	struct sprd_iommu_map_data iommu_data;
	unsigned int frm_q_len;

	if (!off_desc) {
		pr_err("fail to get offline buf is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	if (sprd_iommu_attach_device(&s_isp_pdev->dev) != 0)
		return 0;

	for (num = 0; num < frm_q_len; num++) {
		ion_buf = &buf_desc->ion_buf[num];
		if (ion_buf == NULL)
			return -EPERM;

		memset(&frame, 0x0, sizeof(struct camera_frame));

		/* for isp */
		memset(&iommu_data, 0x0, sizeof(iommu_data));
		iommu_data.buf = ion_buf->buf;
		iommu_data.iova_size = ion_buf->buf_size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		ret = sprd_iommu_map(&s_isp_pdev->dev, &iommu_data);
		if (ret) {
			pr_err("fail to get buffer iova\n");
			return -EFAULT;
		}

		frame.yaddr_vir = iommu_data.iova_addr;
		ion_buf->addr.yaddr_vir = frame.yaddr_vir;

		/* for dcam */
		memset(&iommu_data, 0x0, sizeof(iommu_data));
		iommu_data.buf = ion_buf->buf;
		iommu_data.iova_size = ion_buf->buf_size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		ret = sprd_iommu_map(&s_dcam_pdev->dev, &iommu_data);
		if (ret) {
			pr_err("fail to get buffer iova\n");
			return -EFAULT;
		}

		frame.uaddr_vir = iommu_data.iova_addr;
		ion_buf->addr.uaddr_vir = frame.uaddr_vir;

		pr_debug("isp iova 0x%08x, dcam iova 0x%08x\n",
			ion_buf->addr.yaddr_vir, ion_buf->addr.uaddr_vir);

		frame.pfinfo.table[0] = iommu_data.table;
		frame.pfinfo.size[0] = iommu_data.iova_size;
		frame.fid = num;
		frame.type = (off_type == ISP_OFF_BUF_FULL) ? 0xf : 0xb;

		ret = isp_buf_queue_write(&buf_desc->tmp_buf_queue, &frame);
		if (ret)
			pr_err("fail to write queue\n");

		buf_desc->output_frame_count++;
	}

	pr_debug("-\n");

	return ret;
}

int isp_offline_buf_iommu_unmap(struct isp_offline_desc *off_desc,
	uint8_t off_type)
{
	int ret = 0;
	unsigned int num = 0;
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	struct sprd_iommu_unmap_data iommu_data = {0};
	unsigned int frm_q_len;

	if (sprd_iommu_attach_device(&s_isp_pdev->dev) != 0)
		return ret;

	if (!off_desc) {
		pr_err("fail to get offline buf is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));
	for (num = 0; num < frm_q_len; num++) {
		ion_buf = &buf_desc->ion_buf[num];
		if (ion_buf->dmabuf_p == NULL)
			return -EPERM;

		/* for isp */
		if (ion_buf->addr.yaddr_vir) {
			iommu_data.iova_addr = ion_buf->addr.yaddr_vir;
			iommu_data.iova_size = ion_buf->buf_size;
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.table = NULL;
			iommu_data.buf = NULL;
			ret = sprd_iommu_unmap(&s_isp_pdev->dev, &iommu_data);
			if (ret) {
				pr_err("fail to unmap isp iommu addr\n");
				return -EFAULT;
			}
			ion_buf->addr.yaddr_vir = 0;
		}
	}
	pr_debug("-\n");

	return ret;
}

/* A temporary function to fix unmap hang during dual-cam close */
int isp_offline_buf_iommu_unmap_external(struct isp_offline_desc *off_desc,
	uint8_t off_type)
{
	int ret = 0;
	unsigned int num = 0;
	struct offline_buf_desc *buf_desc = NULL;
	struct offline_ion_buf *ion_buf = NULL;
	struct sprd_iommu_unmap_data iommu_data = {0};
	unsigned int frm_q_len;

	if (sprd_iommu_attach_device(&s_dcam_pdev->dev) != 0)
		return ret;

	if (!off_desc) {
		pr_err("fail to get offline buff is NULL\n");
		return -EFAULT;
	}

	if (get_off_frm_q_len(off_desc, &frm_q_len))
		return -EFAULT;

	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	memset(&iommu_data, 0, sizeof(struct sprd_iommu_unmap_data));
	for (num = 0; num < frm_q_len; num++) {
		ion_buf = &buf_desc->ion_buf[num];
		if (ion_buf->dmabuf_p == NULL)
			return -EPERM;

		/* for dcam */
		if (ion_buf->addr.uaddr_vir) {
			iommu_data.iova_addr = ion_buf->addr.uaddr_vir;
			iommu_data.iova_size = ion_buf->buf_size;
			iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
			iommu_data.table = NULL;
			iommu_data.buf = NULL;
			ret = sprd_iommu_unmap(&s_dcam_pdev->dev, &iommu_data);
			if (ret) {
				pr_err("fail to unmap dcam iommu addr\n");
				return -EFAULT;
			}
			ion_buf->addr.uaddr_vir = 0;
		}
	}

	pr_debug("-\n");

	return ret;
}

int isp_offline_set_next_frm(struct isp_module *module,
	uint8_t off_type, struct camera_frame *out_frame)
{
	int ret = 0;
	struct isp_offline_desc *off_desc = NULL;
	struct offline_buf_desc *buf_desc = NULL;
	struct isp_frm_queue *p_heap = NULL;
	struct isp_buf_queue *p_buf_queue = NULL;
	struct camera_frame frame;
	unsigned int output_frame_count = 0;
	enum isp_id iid = ISP_ID_0;
	enum isp_scl_id path_id;
	struct isp_pipe_dev *dev = NULL;

	if (!module) {
		pr_err("fail to get module ptr is NULL\n");
		return -EFAULT;
	}

	dev = container_of(module, struct isp_pipe_dev, module_info);
	iid = ISP_GET_IID(dev->com_idx);

	if (off_type == ISP_OFF_BUF_FULL)
		path_id = ISP_SCL_CAP;
	else
		path_id = ISP_SCL_PRE;

	off_desc = &module->off_desc;
	buf_desc = isp_offline_sel_buf(off_desc, off_type);
	if (!buf_desc)
		return -EFAULT;

	p_heap = &buf_desc->frame_queue;
	p_buf_queue = &buf_desc->tmp_buf_queue;
	output_frame_count = buf_desc->output_frame_count;

	if (isp_buf_queue_read(p_buf_queue, &frame) == 0 &&
			(frame.yaddr_vir != 0)) {
		buf_desc->output_frame_count--;
		frame.t = out_frame->t;
		frame.fid = out_frame->fid;
		*out_frame = frame;
		frame.width = module->isp_path[path_id].in_size.w;
		frame.height = module->isp_path[path_id].in_size.h;
		pr_debug("offline %s buf frame count: %d, cb:%pS\n",
			 off_type == ISP_OFF_BUF_BIN ? "bin_path" : "full_path",
			 buf_desc->output_frame_count,
			 __builtin_return_address(0));
	} else {
		off_desc->read_buf_err = 1;
		pr_err("fail to read isp buf que off_type %d\n", off_type);
		return -EFAULT;
	}

	if (isp_frame_enqueue(p_heap, &frame) == 0)
		pr_debug("success to enq frame off_type %d\n", off_type);
	else
		ret = ISP_RTN_PATH_FRAME_LOCKED;

	return -ret;
}

/*
 * get a relative large buffer for H/W and S/W use
 */
int isp_gen_buf_alloc(struct isp_buf_info *buf_info)
{
	int ret = 0;
	char name[16+ISP_BUF_SHORT_NAME_LEN+1];
	int iommu_enabled = 0;
	int heap_type;

	if (!buf_info) {
		pr_err("fail to get valid buffer info\n");
		return -EPERM;
	}

	iommu_enabled = !sprd_iommu_attach_device(&s_isp_pdev->dev);

	/* length of "sprd-cam-buf-" <= 16 */
	snprintf(name, 16+ISP_BUF_SHORT_NAME_LEN,
		"sprd-cam-buf-%s", buf_info->name);

	heap_type = sprd_iommu_attach_device(&s_isp_pdev->dev) ?
			ION_HEAP_ID_MASK_MM :
			ION_HEAP_ID_MASK_SYSTEM;

	buf_info->dmabuf_p = ion_new_alloc(buf_info->size,
			heap_type, 0);
	if (IS_ERR_OR_NULL(buf_info->dmabuf_p)) {
		pr_err("fail to alloc ion buf size = 0x%x %ld\n",
			(int)buf_info->size,
			(unsigned long)buf_info->dmabuf_p);
		return -ENOMEM;
	}

	ret = sprd_ion_get_buffer(-1,
			buf_info->dmabuf_p,
			&buf_info->buf,
			&buf_info->buf_size);
	if (ret) {
		pr_err("fail to get ion buf for kernel buffer %p\n",
			buf_info->dmabuf_p);
		ret = -EFAULT;
		goto failed;
	}

	pr_debug("dmabuf_p[%p], size 0x%x, heap %d\n",
		buf_info->dmabuf_p,
		(int)buf_info->size,
		heap_type);

	if (sprd_iommu_attach_device(&s_isp_pdev->dev)== 0) {
		buf_info->sw_addr = sprd_ion_map_kernel(buf_info->dmabuf_p, 0);
		if (IS_ERR_OR_NULL(buf_info->sw_addr)) {
			pr_err("fail to map kernel virtual address\n");
			ret = -EFAULT;
			goto map_failed;
		}
	} else {
		unsigned long phys_addr;
		size_t size;

		ret = sprd_ion_get_phys_addr_by_db(buf_info->dmabuf_p,
				&phys_addr,
				&size);
		if (ret) {
			pr_err("fail to get phys addr\n");
			ret = -EFAULT;
			goto failed;
		} else {
			buf_info->hw_addr = (void *)(phys_addr - MM_ION_OFFSET);
		}

		buf_info->sw_addr = sprd_ion_map_kernel(buf_info->dmabuf_p, 0);
		if (IS_ERR_OR_NULL(buf_info->sw_addr)) {
			pr_err("fail to map kernel virtual address\n");
			ret = -EFAULT;
			goto map_failed;
		}
	}

	return 0;

map_failed:
	sprd_ion_unmap_kernel(buf_info->dmabuf_p, 0);

failed:
	ion_free(buf_info->dmabuf_p);
	buf_info->dmabuf_p = NULL;
	buf_info->buf = NULL;
	buf_info->buf_size = 0;

	return ret;
}

int isp_gen_buf_free(struct isp_buf_info *buf_info)
{
	if (buf_info == NULL || buf_info->dmabuf_p == NULL) {
		pr_err("fail to get valid buffer info\n");
		return -EINVAL;
	}

	sprd_ion_unmap_kernel(buf_info->dmabuf_p, 0);
	buf_info->sw_addr = NULL;

	ion_free(buf_info->dmabuf_p);
	buf_info->dmabuf_p = NULL;
	buf_info->buf = NULL;
	buf_info->buf_size = 0;

	return 0;
}

int isp_gen_buf_hw_map(struct isp_buf_info *buf_info)
{
	int ret = 0;
	struct sprd_iommu_map_data iommu_data;

	if (sprd_iommu_attach_device(&s_isp_pdev->dev) != 0)
		return 0;

	if (!buf_info) {
		pr_err("fail to get valid buffer info\n");
		return -EINVAL;
	}

	if (!buf_info->hw_addr) {
		memset(&iommu_data, 0x0, sizeof(iommu_data));
		iommu_data.buf = buf_info->buf;
		iommu_data.iova_size = buf_info->buf_size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		ret = sprd_iommu_map(&s_isp_pdev->dev, &iommu_data);
		if (ret) {
			pr_err("fail to get buffer iova\n");
			return -EFAULT;
		}
		buf_info->hw_addr = (void *)iommu_data.iova_addr;
	} else
		pr_warn("WARNING: buffer addr already mapped, %p!\n",
			buf_info->hw_addr);

	return 0;
}

int isp_gen_buf_hw_unmap(struct isp_buf_info *buf_info)
{
	int ret = 0;
	struct sprd_iommu_unmap_data iommu_data;

	if (sprd_iommu_attach_device(&s_isp_pdev->dev) != 0)
		return ret;

	if (!buf_info) {
		pr_err("fail to get buffer info\n");
		return -EFAULT;
	}

	if (buf_info->hw_addr) {
		memset(&iommu_data, 0x0, sizeof(iommu_data));
		iommu_data.iova_addr = (unsigned long)buf_info->hw_addr;
		iommu_data.iova_size = buf_info->buf_size;
		iommu_data.ch_type = SPRD_IOMMU_FM_CH_RW;
		iommu_data.table = NULL;
		iommu_data.buf = NULL;
		ret = sprd_iommu_unmap(&s_isp_pdev->dev, &iommu_data);
		if (ret) {
			pr_err("fail to unmap iova\n");
			return -EFAULT;
		}
		buf_info->hw_addr = NULL;
	} else
		pr_warn("WARNING: buffer addr not mapped yet!\n");

	return 0;
}

/**
 * To recycle the specified cnt of buf(s) from a src_q to dst_q;
 *
 * @buf_desc:		pointer to offline_buf_desc
 * @dst_q:		recycle buf(s) to
 * @src_q:		recycle buf(s) from
 * @recycle_buf_cnt:	the buf count to recycle from src_q
 */
int isp_buf_recycle(struct offline_buf_desc *buf_desc,
		    struct isp_buf_queue *dst_q,
		    struct isp_frm_queue *src_q,
		    unsigned int recycle_buf_cnt)
{
	int ret = 0;
	struct camera_frame frame;
	int i;

	if (unlikely(IS_ERR_OR_NULL(dst_q) || IS_ERR_OR_NULL(src_q))) {
		ret = -EFAULT;
		goto failed;
	}
	if (unlikely(recycle_buf_cnt > src_q->valid_cnt))
		recycle_buf_cnt = src_q->valid_cnt;

	for (i = 0; i < recycle_buf_cnt; i++) {
		if (isp_frame_dequeue(src_q, &frame)) {
			pr_err("fail to deque frame\n");
			ret = -EPERM;
			break;
		}

		if (isp_buf_queue_write(dst_q, &frame)) {
			pr_err("fail to write buffer\n");
			ret = -EPERM;
			break;
		}

		buf_desc->output_frame_count++;
		pr_debug("recyled frame id %d, buf frm cnt:%d\n",
			 frame.fid, buf_desc->output_frame_count);
	}

	return ret;

failed:
	pr_err("fail to recycle buf, args error!\n");
	return ret;
}

void *isp_buf_get_kaddr(int fd)
{
	struct dma_buf *dmabuf_p;
	void *kaddr = NULL;

	if (fd <= 0)
		return 0;

	dmabuf_p = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf_p)) {
		pr_err("fail to get dma buf %p\n", dmabuf_p);
		return 0;
	}

	kaddr = sprd_ion_map_kernel(dmabuf_p, 0);
	if (IS_ERR_OR_NULL(kaddr)) {
		pr_err("fail to map kernel vir_addr\n");
	}

	dma_buf_put(dmabuf_p);

	return kaddr;
}
