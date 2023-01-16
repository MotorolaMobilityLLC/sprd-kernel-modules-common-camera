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

#include <linux/err.h>
#include <linux/vmalloc.h>

#include "cam_buf_manager.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_BUF_MANAGER: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define IS_PRIVATE_ID_VALID(val) (((val) > 0) && ((val) < PRIVATE_POOL_NUM_MAX))

#define IS_TAG_ID_VALID(val) (((val) > 0) && ((val) < CAM_BUF_POOL_TAG_ID_NUM))

#define IS_RESERVED_ID_VALID(val) (((val) > 0) && ((val) < CAM_COUNT_MAX))

static struct cam_buf_manager *global_buf_manager = NULL;

static struct camera_queue *cambufmanager_pool_handle_get(struct cam_buf_pool_id *pool_id)
{
	struct camera_queue *ret = NULL;

	if (!pool_id) {
		pr_err("fail to get pool id\n");
		return ret;
	}

	if (IS_PRIVATE_ID_VALID(pool_id->private_pool_idx) &&
		global_buf_manager->private_buf_pool[pool_id->private_pool_idx]) {
		ret = global_buf_manager->private_buf_pool[pool_id->private_pool_idx];
	} else if (IS_TAG_ID_VALID(pool_id->tag_id) &&
		global_buf_manager->tag_pool[pool_id->tag_id]) {
		ret = global_buf_manager->tag_pool[pool_id->tag_id];
	} else if (IS_RESERVED_ID_VALID(pool_id->reserved_pool_id) &&
		global_buf_manager->reserve_buf_pool[pool_id->reserved_pool_id]) {
		ret = global_buf_manager->reserve_buf_pool[pool_id->reserved_pool_id];
	} else {
		pr_debug("get valid pool id tag%d private%d reserved id %d\n",
			pool_id->tag_id, pool_id->private_pool_idx, pool_id->reserved_pool_id);
	}
	return ret;
}

static uint32_t s_status_switch_table[CAM_BUF_STATUS_NUM][CAM_BUF_STATUS_NUM] = {
	[CAM_BUF_ALLOC][CAM_BUF_WITH_ION] = CAM_BUF_STATUS_ALLOC_2_ION,
	[CAM_BUF_ALLOC][CAM_BUF_WITH_IOVA] = CAM_BUF_STATUS_ALLOC_2_IOVA,
	[CAM_BUF_ALLOC][CAM_BUF_WITH_SINGLE_PAGE_IOVA] = CAM_BUF_STATUS_ALLOC_2_SINGLE_PAGE_MAP,
	[CAM_BUF_ALLOC][CAM_BUF_WITH_K_ADDR] = CAM_BUF_STATUS_ALLOC_2_KMAP,
	[CAM_BUF_ALLOC][CAM_BUF_WITH_IOVA_K_ADDR] = CAM_BUF_STATUS_ALLOC_2_IOVA_K,

	[CAM_BUF_WITH_ION][CAM_BUF_ALLOC] = CAM_BUF_STATUS_ION_2_ALLOC,
	[CAM_BUF_WITH_ION][CAM_BUF_WITH_IOVA] = CAM_BUF_STATUS_ION_2_IOVA,
	[CAM_BUF_WITH_ION][CAM_BUF_WITH_SINGLE_PAGE_IOVA] = CAM_BUF_STATUS_ION_2_SINGLE_PAGE_MAP,
	[CAM_BUF_WITH_ION][CAM_BUF_WITH_K_ADDR] = CAM_BUF_STATUS_ION_2_KMAP,
	[CAM_BUF_WITH_ION][CAM_BUF_WITH_IOVA_K_ADDR] = CAM_BUF_STATUS_ION_2_IOVA_K,

	[CAM_BUF_WITH_IOVA][CAM_BUF_ALLOC] = CAM_BUF_STATUS_IOVA_2_ALLOC,
	[CAM_BUF_WITH_IOVA][CAM_BUF_WITH_ION] = CAM_BUF_STATUS_IOVA_2_ION,
	[CAM_BUF_WITH_IOVA][CAM_BUF_WITH_K_ADDR] = CAM_BUF_STATUS_IOVA_2_KMAP,
	[CAM_BUF_WITH_IOVA][CAM_BUF_WITH_IOVA_K_ADDR] = CAM_BUF_STATUS_IOVA_2_IOVA_K,

	[CAM_BUF_WITH_SINGLE_PAGE_IOVA][CAM_BUF_ALLOC] = CAM_BUF_STATUS_SINGLE_PAGE_MAP_2_ALLOC,
	[CAM_BUF_WITH_SINGLE_PAGE_IOVA][CAM_BUF_WITH_ION] = CAM_BUF_STATUS_SINGLE_PAGE_MAP_2_ION,

	[CAM_BUF_WITH_K_ADDR][CAM_BUF_ALLOC] = CAM_BUF_STATUS_KMAP_2_ALLOC,
	[CAM_BUF_WITH_K_ADDR][CAM_BUF_WITH_ION] = CAM_BUF_STATUS_KMAP_2_ION,
	[CAM_BUF_WITH_K_ADDR][CAM_BUF_WITH_IOVA] = CAM_BUF_STATUS_KMAP_2_IOVA,
	[CAM_BUF_WITH_K_ADDR][CAM_BUF_WITH_IOVA_K_ADDR] = CAM_BUF_STATUS_KMAP_2_IOVA_K,

	[CAM_BUF_WITH_IOVA_K_ADDR][CAM_BUF_ALLOC] = CAM_BUF_STATUS_K_IOVA_2_ALLOC,
	[CAM_BUF_WITH_IOVA_K_ADDR][CAM_BUF_WITH_ION] = CAM_BUF_STATUS_K_IOVA_2_ION,
	[CAM_BUF_WITH_IOVA_K_ADDR][CAM_BUF_WITH_IOVA] = CAM_BUF_STATUS_K_IOVA_2_IOVA,
	[CAM_BUF_WITH_IOVA_K_ADDR][CAM_BUF_WITH_K_ADDR] = CAM_BUF_STATUS_K_IOVA_2_K,
};

static int inline cambufmanager_alloc_2_ion(struct camera_buf *buf)
{
	if ((buf->type == CAM_BUF_USER) && (!buf->dmabuf_p))
		return cam_buf_ionbuf_get(buf);
	else
		buf->status = CAM_BUF_WITH_ION;
	return 0;
}

static int inline cambufmanager_alloc_2_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	int ret = 0;
	if ((buf->type == CAM_BUF_USER) && (!buf->dmabuf_p))
		ret = cam_buf_ionbuf_get(buf);
	if (ret)
		return ret;

	ret = cam_buf_iommu_map(buf, type);
	if (ret && (buf->type == CAM_BUF_USER))
		cam_buf_ionbuf_put(buf);
	return ret;
}

static int inline cambufmanager_alloc_2_single_page_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	int ret = 0;
	if ((buf->type == CAM_BUF_USER) && (!buf->dmabuf_p))
		ret = cam_buf_ionbuf_get(buf);
	if (ret)
		return ret;
	ret = cam_buf_iommu_single_page_map(buf, type);
	if (ret && (buf->type == CAM_BUF_USER))
		cam_buf_ionbuf_put(buf);
	return ret;
}

static int inline cambufmanager_alloc_2_kmap(struct camera_buf *buf)
{
	int ret = 0;
	if ((buf->type == CAM_BUF_USER) && (!buf->dmabuf_p))
		ret = cam_buf_ionbuf_get(buf);
	if (ret)
		return ret;
	ret = cam_buf_kmap(buf);
	if (ret && (buf->type == CAM_BUF_USER))
		cam_buf_ionbuf_put(buf);
	return ret;
}

static int inline cambufmanager_alloc_2_k_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	int ret = 0;

	if ((buf->type == CAM_BUF_USER) && (!buf->dmabuf_p))
		ret = cam_buf_ionbuf_get(buf);
	if (ret)
		return ret;
	ret = cam_buf_iommu_map(buf, type);
	if (ret && (buf->type == CAM_BUF_USER)) {
		cam_buf_ionbuf_put(buf);
		return ret;
	}
	ret = cam_buf_kmap(buf);
	if (ret ) {
		if (buf->type == CAM_BUF_USER)
			cam_buf_ionbuf_put(buf);
		cam_buf_iommu_unmap(buf, type);
	}
	return ret;
}

static int inline cambufmanager_ion_2_alloc(struct camera_buf *buf)
{
	if ((buf->type == CAM_BUF_USER) && buf->dmabuf_p)
		return cam_buf_ionbuf_put(buf);
	else
		buf->status = CAM_BUF_ALLOC;
	return 0;
}

static int inline cambufmanager_ion_2_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	return cam_buf_iommu_map(buf, type);
}

static int inline cambufmanager_ion_2_single_page_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	return cam_buf_iommu_single_page_map(buf, type);
}

static int inline cambufmanager_ion_2_kmap(struct camera_buf *buf)
{
	return cam_buf_kmap(buf);
}

static int inline cambufmanager_ion_2_k_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	int ret = 0;

	ret = cam_buf_iommu_map(buf, type);
	if (ret)
		return ret;
	ret = cam_buf_kmap(buf);
	if (ret )
		cam_buf_iommu_unmap(buf, type);

	return ret;
}

static int inline cambufmanager_iova_2_alloc(struct camera_buf *buf)
{
	int ret = 0;
	enum cam_buf_iommudev_type i = 0;

	for (i = CAM_BUF_IOMMUDEV_ISP; i < CAM_BUF_IOMMUDEV_MAX; i++)
		if (buf->mapping_state & (1 << i))
			ret |= cam_buf_iommu_unmap(buf, i);
	if (buf->type == CAM_BUF_USER)
		ret |= cam_buf_ionbuf_put(buf);
	else
		buf->status = CAM_BUF_ALLOC;
	return ret;
}

static int inline cambufmanager_iova_2_ion(struct camera_buf *buf)
{
	int ret = 0;
	enum cam_buf_iommudev_type i = 0;

	for (i = CAM_BUF_IOMMUDEV_ISP; i < CAM_BUF_IOMMUDEV_MAX; i++)
		if (buf->mapping_state & (1 << i))
			ret |= cam_buf_iommu_unmap(buf, i);
	return ret;
}

static int inline cambufmanager_iova_2_kmap(struct camera_buf *buf)
{
	return cam_buf_kmap(buf);
}

static int inline cambufmanager_iova_2_k_iova(struct camera_buf *buf)
{
	return cam_buf_kmap(buf);
}

static int inline cambufmanager_kmap_2_alloc(struct camera_buf *buf)
{
	int ret = 0;
	ret = cam_buf_kunmap(buf);
	if (buf->type == CAM_BUF_USER)
		ret |= cam_buf_ionbuf_put(buf);
	else
		buf->status = CAM_BUF_ALLOC;
	return ret;
}

static int inline cambufmanager_kmap_2_ion(struct camera_buf *buf)
{
	return cam_buf_kunmap(buf);
}

static int inline cambufmanager_kmap_2_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	int ret = 0;

	ret = cam_buf_kunmap(buf);
	ret |= cam_buf_iommu_map(buf, type);
	return ret;
}

static int inline cambufmanager_k_2_k_iova(struct camera_buf *buf, enum cam_buf_iommudev_type type)
{
	return cam_buf_iommu_map(buf, type);
}

static int inline cambufmanager_k_iova_2_iova(struct camera_buf *buf)
{
	return cam_buf_kunmap(buf);
}

static int inline cambufmanager_k_iova_2_k(struct camera_buf *buf)
{
	int ret = 0;
	enum cam_buf_iommudev_type i = 0;

	for (i = CAM_BUF_IOMMUDEV_ISP; i < CAM_BUF_IOMMUDEV_MAX; i++)
		if (buf->mapping_state & (1 << i))
			ret |= cam_buf_iommu_unmap(buf, i);
	return ret;
}

static int inline cambufmanager_k_iova_2_ion(struct camera_buf *buf)
{
	int ret = 0;
	enum cam_buf_iommudev_type i = 0;

	for (i = CAM_BUF_IOMMUDEV_ISP; i < CAM_BUF_IOMMUDEV_MAX; i++)
		if (buf->mapping_state & (1 << i))
			ret |= cam_buf_iommu_unmap(buf, i);
	ret |= cam_buf_kunmap(buf);
	return ret;
}

static int inline cambufmanager_k_iova_2_alloc(struct camera_buf *buf)
{
	int ret = 0;
	enum cam_buf_iommudev_type i = 0;

	for (i = CAM_BUF_IOMMUDEV_ISP; i < CAM_BUF_IOMMUDEV_MAX; i++)
		if (buf->mapping_state & (1 << i))
			ret |= cam_buf_iommu_unmap(buf, i);
	ret |= cam_buf_kunmap(buf);
	if (buf->type == CAM_BUF_USER)
		ret |= cam_buf_ionbuf_put(buf);
	else
		buf->status = CAM_BUF_ALLOC;
	return ret;
}

static int inline cambufmanager_buf_status_change(struct camera_buf *buf,
		enum cam_buf_status dst_status,
		enum cam_buf_iommudev_type type)
{
	int ret = 0;
	enum cambufmanager_status_cmd switch_cmd;
	if ((buf->status == dst_status) || (buf->type == CAM_BUF_NONE))
		return 0;
	switch_cmd = s_status_switch_table[buf->status][dst_status];
	switch (switch_cmd) {
	case CAM_BUF_STATUS_ALLOC_2_ION:
		ret = cambufmanager_alloc_2_ion(buf);
		break;
	case CAM_BUF_STATUS_ALLOC_2_IOVA:
		ret = cambufmanager_alloc_2_iova(buf, type);
		break;
	case CAM_BUF_STATUS_ALLOC_2_SINGLE_PAGE_MAP:
		ret = cambufmanager_alloc_2_single_page_iova(buf, type);
		break;
	case CAM_BUF_STATUS_ALLOC_2_KMAP:
		ret = cambufmanager_alloc_2_kmap(buf);
		break;
	case CAM_BUF_STATUS_ALLOC_2_IOVA_K:
		ret = cambufmanager_alloc_2_k_iova(buf, type);
		break;
	case CAM_BUF_STATUS_ION_2_ALLOC:
		ret = cambufmanager_ion_2_alloc(buf);
		break;
	case CAM_BUF_STATUS_ION_2_IOVA:
		ret = cambufmanager_ion_2_iova(buf, type);
		break;
	case CAM_BUF_STATUS_ION_2_SINGLE_PAGE_MAP:
		ret = cambufmanager_ion_2_single_page_iova(buf, type);
		break;
	case CAM_BUF_STATUS_ION_2_KMAP:
		ret = cambufmanager_ion_2_kmap(buf);
		break;
	case CAM_BUF_STATUS_ION_2_IOVA_K:
		ret = cambufmanager_ion_2_k_iova(buf, type);
		break;
	case CAM_BUF_STATUS_IOVA_2_ALLOC:
		ret = cambufmanager_iova_2_alloc(buf);
		break;
	case CAM_BUF_STATUS_IOVA_2_ION:
		ret = cambufmanager_iova_2_ion(buf);
		break;
	case CAM_BUF_STATUS_IOVA_2_KMAP:
		ret = cambufmanager_iova_2_kmap(buf);
		break;
	case CAM_BUF_STATUS_IOVA_2_IOVA_K:
		ret = cambufmanager_iova_2_k_iova(buf);
		break;
	case CAM_BUF_STATUS_SINGLE_PAGE_MAP_2_ALLOC:
		ret = cambufmanager_iova_2_alloc(buf);
		break;
	case CAM_BUF_STATUS_SINGLE_PAGE_MAP_2_ION:
		ret = cambufmanager_iova_2_ion(buf);
		break;
	case CAM_BUF_STATUS_KMAP_2_ALLOC:
		ret = cambufmanager_kmap_2_alloc(buf);
		break;
	case CAM_BUF_STATUS_KMAP_2_ION:
		ret = cambufmanager_kmap_2_ion(buf);
		break;
	case CAM_BUF_STATUS_KMAP_2_IOVA:
		ret = cambufmanager_kmap_2_iova(buf, type);
		break;
	case CAM_BUF_STATUS_KMAP_2_IOVA_K:
		ret = cambufmanager_k_2_k_iova(buf, type);
		break;
	case CAM_BUF_STATUS_K_IOVA_2_ALLOC:
		ret = cambufmanager_k_iova_2_alloc(buf);
		break;
	case CAM_BUF_STATUS_K_IOVA_2_ION:
		ret = cambufmanager_k_iova_2_ion(buf);
		break;
	case CAM_BUF_STATUS_K_IOVA_2_IOVA:
		ret = cambufmanager_k_iova_2_iova(buf);
		break;
	case CAM_BUF_STATUS_K_IOVA_2_K:
		ret = cambufmanager_k_iova_2_k(buf);
		break;
	default:
		pr_err("fail to get status %d -> %d\n", buf->status, dst_status);
	}
	return ret;
}

static void cambufmanager_reserve_q_cnt_check(struct camera_queue *reserve_heap)
{
	struct camera_frame *newfrm = NULL;
	struct camera_frame *ori = NULL;
	int j = 0;

	if (reserve_heap->cnt > 1)
		return;

	ori = cam_queue_dequeue_peek(reserve_heap, struct camera_frame, list);
	if (!ori)
		return;
	while (j < CAM_RESERVE_BUF_Q_LEN) {
		newfrm = cam_queue_empty_frame_get();
		if (newfrm) {
			newfrm->is_reserved = CAM_RESERVED_BUFFER_COPY;
			newfrm->channel_id = ori->channel_id;
			newfrm->pyr_status = ori->pyr_status;
			newfrm->user_fid = ori->user_fid;
			memcpy(&newfrm->buf, &ori->buf, sizeof(struct camera_buf));
			newfrm->buf.type = CAM_BUF_NONE;
			cam_queue_enqueue(reserve_heap, &newfrm->list);
			j++;
		}
	}
	pr_info("reserved pool %px, cnt %d\n", reserve_heap, reserve_heap->cnt);
}

int cam_buf_manager_buf_status_cfg(struct camera_buf *buf, enum cambufmanager_status_ops_cmd cmd, enum cam_buf_iommudev_type type)
{
	int ret = 0;
	enum cam_buf_status dst_status = 0;
	enum cam_buf_status src_status = 0;
	uint32_t map_dev = 0;

	if (!buf || (buf->status == CAM_BUF_EMPTY)) {
		pr_debug("fail to get buffer, frame %px\n", buf);
		return 0;
	}

	src_status = buf->status;

	if (cmd >= CAM_BUF_STATUS_OPS_CMD_NUM || !cmd)
		return 0;
	if (buf->type == CAM_BUF_NONE)
		return 0;
	if (buf->bypass_iova_ops &&
		(cmd == CAM_BUF_STATUS_GET_IOVA ||
		cmd == CAM_BUF_STATUS_GET_SINGLE_PAGE_IOVA ||
		cmd == CAM_BUF_STATUS_PUT_IOVA))
		return 0;

	switch (cmd) {
	case CAM_BUF_STATUS_MOVE_TO_ALLOC:
		dst_status = CAM_BUF_ALLOC;
		break;
	case CAM_BUF_STATUS_MOVE_TO_ION:
		dst_status = CAM_BUF_WITH_ION;
		break;
	case CAM_BUF_STATUS_GET_IOVA:
		dst_status =  (src_status & 0xfc) | CAM_BUF_WITH_IOVA;
		if (src_status == dst_status && (buf->mapping_state & (1 << type)) == 0)
			ret = cam_buf_iommu_map(buf, type);
		break;
	case CAM_BUF_STATUS_GET_SINGLE_PAGE_IOVA:
		dst_status = (src_status & 0xfc) | CAM_BUF_WITH_SINGLE_PAGE_IOVA;
		if (src_status == dst_status && (buf->mapping_state & (1 << type)) == 0)
			ret = cam_buf_iommu_single_page_map(buf, type);
		break;
	case CAM_BUF_STATUS_GET_K_ADDR:
		dst_status = (src_status & 0xfc) | CAM_BUF_WITH_K_ADDR;
		break;
	case CAM_BUF_STATUS_PUT_IOVA:
		dst_status = src_status;
		if ((buf->mapping_state & (1 << type)) == 0)
			break;
		map_dev = buf->mapping_state & (~(1 << type));
		if (map_dev != CAM_BUF_MAPPING_NULL) {
			ret = cam_buf_iommu_unmap(buf, type);
			break;
		}
		dst_status = src_status & (~CAM_BUF_WITH_IOVA);
		dst_status = dst_status & (~CAM_BUF_WITH_SINGLE_PAGE_IOVA);
		if (!dst_status)
			dst_status = CAM_BUF_WITH_ION;
		break;
	case CAM_BUF_STATUS_PUT_K_ADDR:
		dst_status = src_status & (~CAM_BUF_WITH_K_ADDR);
		if (!dst_status)
			dst_status = CAM_BUF_WITH_ION;
		break;
	case CAM_BUF_STATUS_GET_IOVA_K_ADDR:
		dst_status = CAM_BUF_WITH_IOVA_K_ADDR;
		break;
	case CAM_BUF_STATUS_PUT_IOVA_K_ADDR:
		dst_status = CAM_BUF_WITH_ION;
		break;
	default:
		pr_warn("not support cmd %d\n", cmd);
		return 0;
	}

	if (ret)
		return ret;

	if (src_status == dst_status)
		return 0;

	ret = cambufmanager_buf_status_change(buf, dst_status, type);
	if ((buf->status != dst_status) || ret)
		pr_err("fail to change status, buf %px status %d not right, %d -> %d, cmd %d, cb: %pS\n", buf, buf->status, src_status, dst_status, cmd, __builtin_return_address(0));
	return ret;
}

struct camera_frame *cam_buf_manager_buf_dequeue(struct cam_buf_pool_id *pool_id, struct camera_buf_get_desc *buf_desc)
{
	int ret = 0;
	struct camera_queue *heap = NULL;
	struct camera_queue *resverve_heap = NULL;
	struct cam_buf_pool_id pool_id_normal = {0};
	struct camera_frame *tmp = NULL;
	struct camera_queue *src = NULL;
	uint32_t cmd = 0;

	if (!pool_id)
		return NULL;

	pool_id_normal = *pool_id;
	pool_id_normal.reserved_pool_id = 0;
	heap = cambufmanager_pool_handle_get(&pool_id_normal);

	if (IS_RESERVED_ID_VALID(pool_id->reserved_pool_id) &&
		global_buf_manager->reserve_buf_pool[pool_id->reserved_pool_id])
		resverve_heap = global_buf_manager->reserve_buf_pool[pool_id->reserved_pool_id];

	if (!heap && !resverve_heap) {
		pr_err("fail to get valid pool id\n");
		goto exit;
	}

	if (buf_desc)
		cmd = buf_desc->q_ops_cmd;
	if (heap) {
		switch (cmd) {
		case CAM_QUEUE_DEL_TAIL:
			tmp = cam_queue_del_tail(heap, struct camera_frame, list);
			src = heap;
			break;
		case CAM_QUEUE_DEQ_PEEK:
			tmp = cam_queue_dequeue_peek(heap, struct camera_frame, list);
			break;
		case CAM_QUEUE_TAIL_PEEK:
			tmp = cam_queue_tail_peek(heap, struct camera_frame, list);
			break;
		case CAM_QUEUE_TAIL:
			tmp = cam_queue_dequeue_tail(heap, struct camera_frame, list);
			src = heap;
			break;
		case CAM_QUEUE_IF:
			tmp = cam_queue_dequeue_if(heap, buf_desc->filter, (void *)&buf_desc->target_fid);
			src = heap;
			break;
		default:
			tmp = cam_queue_dequeue(heap, struct camera_frame, list);
			src = heap;
		}
	}

	if (!tmp && resverve_heap) {/* && pool_id->tag_id != isp_blk_param*/
		cambufmanager_reserve_q_cnt_check(resverve_heap);
		tmp = cam_queue_dequeue(resverve_heap, struct camera_frame, list);
		src = resverve_heap;
		pr_debug("res pool %d %px deq, frame %px, cnt %d, cb:%pS\n\n", pool_id->reserved_pool_id, resverve_heap, tmp, resverve_heap->cnt, __builtin_return_address(0));
	} else
		pr_debug("pri %d, tag %d, %px, frame %px, cnt %d\n", pool_id->private_pool_idx, pool_id->tag_id, heap, tmp, heap->cnt);

	if (!buf_desc || !tmp)
		goto exit;

	if (buf_desc->buf_ops_cmd == CAM_BUF_STATUS_GET_IOVA)
		buf_desc->buf_ops_cmd = CAM_BUF_GET_IOVA_METHOD(tmp);
	ret = cam_buf_manager_buf_status_cfg(&tmp->buf, buf_desc->buf_ops_cmd, buf_desc->mmu_type);
	if (ret) {
		pr_err("fail to cfg buf status, (p %d, t %d, r %d), ori %d, cmd %d\n", pool_id->private_pool_idx, pool_id->tag_id, pool_id->reserved_pool_id, tmp->buf.status, buf_desc->buf_ops_cmd);
		if (src)
			cam_queue_enqueue(src, &tmp->list);
		return NULL;
	}

exit:
	if (!tmp)
		pr_debug("(p %d, t %d, r %d) deq null\n", pool_id->private_pool_idx, pool_id->tag_id, pool_id->reserved_pool_id);
	return tmp;
}

int cam_buf_manager_buf_enqueue(struct cam_buf_pool_id *pool_id,
	struct camera_frame *pframe, struct camera_buf_get_desc *buf_desc)
{
	struct camera_queue *heap = NULL;
	int ret = 0;

	if (!pframe) {
		pr_err("fail to get frame\n");
		return -1;
	}
	heap = cambufmanager_pool_handle_get(pool_id);
	if (!heap) {
		pr_err("fail to get valid pool id\n");
		return -1;
	}

	if (buf_desc && buf_desc->buf_ops_cmd == CAM_BUF_STATUS_GET_IOVA)
		buf_desc->buf_ops_cmd = CAM_BUF_GET_IOVA_METHOD(pframe);

	if (buf_desc)
		ret = cam_buf_manager_buf_status_cfg(&pframe->buf, buf_desc->buf_ops_cmd, buf_desc->mmu_type);
	if (ret) {
		pr_err("fail to cfg buffer\n");
		return -1;
	}

	if (buf_desc && buf_desc->q_ops_cmd == CAM_QUEUE_FRONT)
		ret = cam_queue_enqueue_head(heap, &pframe->list);
	else
		ret = cam_queue_enqueue(heap, &pframe->list);

	if (ret) {
		pr_err("fail to enq, (p %d, t %d, r%d) frame %px\n", pool_id->private_pool_idx, pool_id->tag_id, pool_id->reserved_pool_id, pframe);
		return -1;
	}

	pr_debug("pool p%d t%d r%d, buf cnt %d, cb:%pS\n",
		pool_id->private_pool_idx, pool_id->tag_id, pool_id->reserved_pool_id,
		heap->cnt, __builtin_return_address(0));
	return 0;
}

void cam_buf_manager_buf_clear(struct cam_buf_pool_id *pool_id)
{
	struct camera_frame *pframe = NULL;
	struct camera_queue *heap = NULL;

	heap = cambufmanager_pool_handle_get(pool_id);
	if (!heap) {
		pr_err("fail to get valid pool id\n");
		return;
	}

	pr_debug("pool r%d p%d t%d, cnt %d clear\n", pool_id->reserved_pool_id, pool_id->private_pool_idx, pool_id->tag_id, heap->cnt);
	do {
		pframe = cam_queue_dequeue(heap, struct camera_frame, list);
		if (pframe) {
			if (pframe->is_reserved == CAM_RESERVED_BUFFER_COPY)
				cambufmanager_buf_status_change(&pframe->buf, CAM_BUF_WITH_ION, CAM_BUF_IOMMUDEV_MAX);
			else
				cambufmanager_buf_status_change(&pframe->buf, CAM_BUF_ALLOC, CAM_BUF_IOMMUDEV_MAX);
			if (pframe->buf.type == CAM_BUF_KERNEL)
				cam_buf_free(&pframe->buf);
			cam_queue_empty_frame_put(pframe);
		}
	} while (pframe);
}

int cam_buf_manager_pool_reg(struct cam_buf_pool_id *pool_id, uint32_t length)
{
	int idx = 0;
	struct camera_queue *tmp_p = NULL;

	mutex_lock(&global_buf_manager->pool_lock);
	if (pool_id && IS_TAG_ID_VALID(pool_id->tag_id)) {
		if (global_buf_manager->tag_pool[pool_id->tag_id]) {
			pr_info("already reg tag pool %d\n", pool_id->tag_id);
			mutex_unlock(&global_buf_manager->pool_lock);
			return 0;
		} else {
			tmp_p = cam_buf_kernel_sys_vzalloc(sizeof(struct camera_queue));
			if (IS_ERR_OR_NULL(tmp_p)) {
				pr_err("fail to alloc pool\n");
				mutex_unlock(&global_buf_manager->pool_lock);
				return -1;
			}
			global_buf_manager->tag_pool[pool_id->tag_id] = tmp_p;
			pr_debug("reg tag pool %d %px, cd:%pS\n", pool_id->tag_id, tmp_p, __builtin_return_address(0));
			mutex_unlock(&global_buf_manager->pool_lock);
			cam_queue_init(tmp_p, length, NULL);
			return 0;
		}
	} else {
		for (idx = 1; idx < PRIVATE_POOL_NUM_MAX; idx ++)
			if (!global_buf_manager->private_buf_pool[idx])
				break;
		if(idx == PRIVATE_POOL_NUM_MAX) {
			pr_err("fail to get pool idx, full\n");
			mutex_unlock(&global_buf_manager->pool_lock);
			return -1;
		} else{
			tmp_p = cam_buf_kernel_sys_vzalloc(sizeof(struct camera_queue));
			if (IS_ERR_OR_NULL(tmp_p)) {
				pr_err("fail to alloc pool\n");
				mutex_unlock(&global_buf_manager->pool_lock);
				return -1;
			}
			global_buf_manager->private_buf_pool[idx] = tmp_p;
			pr_debug("reg pool %d %px, cd:%pS\n", idx, tmp_p, __builtin_return_address(0));
			mutex_unlock(&global_buf_manager->pool_lock);
			cam_queue_init(tmp_p, length, NULL);
			return idx;
		}
	}
}

int cam_buf_manager_pool_unreg(struct cam_buf_pool_id *pool_id)
{
	uint32_t unreg_private_pool = 0, unreg_tag_pool = 0;
	struct camera_queue *tmp_p = NULL;

	if (IS_PRIVATE_ID_VALID(pool_id->private_pool_idx)) {
		tmp_p = global_buf_manager->private_buf_pool[pool_id->private_pool_idx];
		if (!tmp_p)
			pr_warn("pool already unreg pri %d\n", pool_id->private_pool_idx);
		unreg_private_pool = 1;
	}

	if (IS_TAG_ID_VALID(pool_id->tag_id)) {
		tmp_p = global_buf_manager->tag_pool[pool_id->tag_id];
		if (!tmp_p) {
			pr_warn("tag pool %d already unreg\n", pool_id->tag_id);
			return 0;
		}
		unreg_tag_pool = 1;
	}

	if (tmp_p) {
		pr_debug("pool p:%d t:%d unreg\n", pool_id->private_pool_idx, pool_id->tag_id);
		cam_buf_manager_buf_clear(pool_id);
		cam_buf_kernel_sys_vfree(tmp_p);
	}

	if (unreg_tag_pool)
		global_buf_manager->tag_pool[pool_id->tag_id] = NULL;
	else if (unreg_private_pool)
		global_buf_manager->private_buf_pool[pool_id->private_pool_idx] = NULL;

	return 0;
}

int cam_buf_manager_pool_cnt(struct cam_buf_pool_id *pool_id)
{
	struct camera_queue *heap = NULL;

	heap = cambufmanager_pool_handle_get(pool_id);
	if (!heap) {
		pr_err("fail to get valid pool id\n");
		return -1;
	}

	return heap->cnt;
}

int cam_buf_manager_init(uint32_t cam_id)
{
	struct camera_queue *tmp_q = NULL;

	if (cam_id >= CAM_COUNT_MAX) {
		pr_err("fail to support cam%d\n", cam_id);
		return -1;
	}

	if (!global_buf_manager) {
		global_buf_manager = cam_buf_kernel_sys_vzalloc(sizeof(struct cam_buf_manager));
		if (IS_ERR_OR_NULL(global_buf_manager)) {
			global_buf_manager = NULL;
			pr_err("fail to create cam buf manager\n");
			return -1;
		};

		atomic_set(&global_buf_manager->user_cnt, 0);
		mutex_init(&global_buf_manager->pool_lock);
	}

	atomic_inc(&global_buf_manager->user_cnt);
	tmp_q = cam_buf_kernel_sys_vzalloc(sizeof(struct camera_queue));
	if (IS_ERR_OR_NULL(tmp_q)) {
		pr_err("fail to alloc reserved q, cam%d\n", cam_id);
		mutex_destroy(&global_buf_manager->pool_lock);
		cam_buf_kernel_sys_vfree(global_buf_manager);
		global_buf_manager = NULL;
		return -1;
	}

	cam_queue_init(tmp_q, RESERVE_BUF_Q_LEN, NULL);
	global_buf_manager->reserve_buf_pool[cam_id + 1] = tmp_q;
	pr_info("reg reserved pool %d, %px\n", cam_id, tmp_q);

	return (cam_id + 1);
}

int cam_buf_manager_deinit(uint32_t cam_id)
{
	struct camera_queue *tmp_q = NULL;
	struct cam_buf_pool_id pool_id = {0};

	if (cam_id >= CAM_COUNT_MAX || !global_buf_manager) {
		pr_err("fail to support cam%d\n", cam_id);
		return -1;
	}

	pool_id.reserved_pool_id = cam_id + 1;
	tmp_q = global_buf_manager->reserve_buf_pool[cam_id + 1];
	if (tmp_q) {
		cam_buf_manager_buf_clear(&pool_id);
		cam_buf_kernel_sys_vfree(tmp_q);
		global_buf_manager->reserve_buf_pool[cam_id + 1] = NULL;
	}
	pr_info("cam%d buf manager deinit, unreg %px\n", cam_id, tmp_q);

	if (atomic_dec_return(&global_buf_manager->user_cnt) == 0) {
		uint32_t i = 0;

		for (i = 0; i < CAM_COUNT_MAX; i++) {
			tmp_q = global_buf_manager->reserve_buf_pool[i];
			if (!tmp_q)
				continue;
			pool_id.reserved_pool_id = i;
			cam_buf_manager_buf_clear(&pool_id);
			cam_buf_kernel_sys_vfree(tmp_q);
			global_buf_manager->reserve_buf_pool[i] = NULL;
		}

		for (i = 0; i < PRIVATE_POOL_NUM_MAX; i++) {
			tmp_q = global_buf_manager->private_buf_pool[i];
			if (!tmp_q)
				continue;
			pool_id.private_pool_idx = i;
			cam_buf_manager_buf_clear(&pool_id);
			cam_buf_kernel_sys_vfree(tmp_q);
			global_buf_manager->private_buf_pool[i] = NULL;
		}

		for (i = 0; i < CAM_BUF_POOL_TAG_ID_NUM; i++) {
			tmp_q = global_buf_manager->tag_pool[i];
			if (!tmp_q)
				continue;
			pool_id.tag_id = i;
			cam_buf_manager_buf_clear(&pool_id);
			cam_buf_kernel_sys_vfree(tmp_q);
			global_buf_manager->tag_pool[i] = NULL;
		}
		mutex_destroy(&global_buf_manager->pool_lock);
		cam_buf_kernel_sys_vfree(global_buf_manager);
		global_buf_manager = NULL;
		pr_info("global_buf_manager deinit\n");
	}

	return 0;
}
