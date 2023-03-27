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

#include "cam_buf_monitor.h"
#include "cam_queue.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_BUF_MONITOR: %d %d %s : " fmt, current->pid, __LINE__, __func__

static struct cam_mem_dbg_info s_mem_dbg;
struct cam_mem_dbg_info *g_mem_dbg = &s_mem_dbg;
static struct camera_queue s_buf_map_calc_queue;
static struct camera_queue s_memory_alloc_queue;
static struct camera_queue s_empty_memory_alloc_queue;

static void cambufmonitor_empty_memory_alloc_queue_get(void)
{
	int ret = 0, i = 0;
	struct cam_buf_alloc_info *alloc_info = NULL;

	for (i = 0; i < CAM_BUF_EMPTY_MEMORY_ALLOC_Q_LEN; i++) {
		alloc_info = vzalloc(sizeof(struct cam_buf_alloc_info));
		if (alloc_info == NULL) {
			pr_err("fail to vzalloc memalloc_info\n");
			continue;
		}
		ret = CAM_QUEUE_ENQUEUE(g_mem_dbg->empty_memory_alloc_queue, &alloc_info->list);
		if (ret) {
			pr_err("fail to enque empty_memory_alloc_queue\n");
			vfree(alloc_info);
			alloc_info = NULL;
		}
	}
}

static void cambufmonitor_memory_alloc_queue_destory(void *param)
{
	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}
	vfree(param);
}

static void cambufmonitor_map_calc_queue_destory(void *param)
{
	if (!param) {
		pr_err("fail to get valid param\n");
		return;
	}
	cam_buf_kernel_sys_kfree(param);
}

void cam_buf_monitor_mapinfo_dequeue(struct camera_buf *buf_info)
{
	unsigned long flags = 0;
	struct cam_buf_map_info *map_info = NULL, *map_info_next = NULL;

	spin_lock_irqsave(&g_mem_dbg->buf_map_calc_queue->lock, flags);
	CAM_QUEUE_FOR_EACH_ENTRY_SAFE(map_info, map_info_next, &g_mem_dbg->buf_map_calc_queue->head, list) {
		if (!map_info)
			continue;
		if (map_info->buf_info == buf_info) {
			if (map_info->kmap_counts || map_info->dcam_iommumap_counts || map_info->isp_iommumap_counts)
				pr_err("fail to unmap buf_info :%p, kmap_counts %d, dcam_iommumap_counts %d, isp_iommumap_counts %d, buf_time %06d.%06d, map_time %06d.%06d\n",
					buf_info, map_info->kmap_counts, map_info->dcam_iommumap_counts, map_info->isp_iommumap_counts,
					(int)map_info->buf_time.tv_sec, (int)map_info->buf_time.tv_usec, (int)map_info->map_time.tv_sec, (int)map_info->map_time.tv_usec);
			CAM_QUEUE_LIST_DEL(&map_info->list);
			g_mem_dbg->buf_map_calc_queue->cnt--;
			cam_buf_kernel_sys_kfree(map_info);
			map_info = NULL;
			pr_debug("buf_info %p\n", buf_info);
			break;
		}
	}
	spin_unlock_irqrestore(&g_mem_dbg->buf_map_calc_queue->lock, flags);
}

int cam_buf_monitor_mapinfo_enqueue(struct camera_buf *buf_info, enum cam_buf_source buf_source)
{
	int ret = 0;
	unsigned long flags = 0;
	timespec cur_ts = {0};
	struct cam_buf_map_info *map_info = NULL;
	/*
	 * If memcopy buf_info is enqueued to buf_map_calc_queue,
	 * it will not be dequeued.
	 * buf_alloc and ion_buf_get may enqueue same buf_info to buf_map_calc_queue,
	 * so dequeue first.
	 */
	if (buf_source == CAM_BUF_SOURCE_ALLOC_GET) {
		cam_buf_monitor_mapinfo_dequeue(buf_info);
	} else {
		spin_lock_irqsave(&g_mem_dbg->buf_map_calc_queue->lock, flags);
		CAM_QUEUE_FOR_EACH_ENTRY(map_info, &g_mem_dbg->buf_map_calc_queue->head, list) {
			if (!map_info)
				continue;
			if (map_info->buf_info == buf_info) {
				spin_unlock_irqrestore(&g_mem_dbg->buf_map_calc_queue->lock, flags);
				return -1;
			}
		}
		spin_unlock_irqrestore(&g_mem_dbg->buf_map_calc_queue->lock, flags);
	}

	map_info = cam_buf_kernel_sys_kzalloc(sizeof(struct cam_buf_map_info), GFP_ATOMIC);
	if (map_info == NULL) {
		pr_err("fail to alloc map_info\n");
		return -1;
	}

	if (buf_source == CAM_BUF_SOURCE_MEMCPY)
		map_info->buf_memcyp = 1;
	else
		map_info->buf_memcyp = 0;
	map_info->buf_info = buf_info;

	os_adapt_time_get_ts(&cur_ts);
	map_info->buf_time.tv_sec = cur_ts.tv_sec;
	map_info->buf_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
	ret = CAM_QUEUE_ENQUEUE(g_mem_dbg->buf_map_calc_queue, &map_info->list);
	if (ret) {
		pr_err("fail to enque g_cam_buf_queue\n");
		cam_buf_kernel_sys_kfree(map_info);
		map_info = NULL;
	}

	return ret;
}

void cam_buf_monitor_map_counts_calc(struct camera_buf *buf_info, enum cam_buf_map_type type)
{
	unsigned long flags = 0;
	timespec cur_ts = {0};
	struct cam_buf_map_info *map_info = NULL;

	spin_lock_irqsave(&g_mem_dbg->buf_map_calc_queue->lock, flags);
	CAM_QUEUE_FOR_EACH_ENTRY(map_info, &g_mem_dbg->buf_map_calc_queue->head, list) {
		if (!map_info)
			continue;
		if (map_info->buf_info == buf_info) {
			os_adapt_time_get_ts(&cur_ts);
			map_info->map_time.tv_sec = cur_ts.tv_sec;
			map_info->map_time.tv_usec = cur_ts.tv_nsec / NSEC_PER_USEC;
			switch (type) {
			case CAM_BUF_DCAM_IOMMUMAP:
				map_info->dcam_iommumap_counts++;
				break;
			case CAM_BUF_DCAM_IOMMUUNMAP:
				map_info->dcam_iommumap_counts--;
				break;
			case CAM_BUF_ISP_IOMMUMAP:
				map_info->isp_iommumap_counts++;
				break;
			case CAM_BUF_ISP_IOMMUUNMAP:
				map_info->isp_iommumap_counts--;
				break;
			case CAM_BUF_KMAP:
				map_info->kmap_counts++;
				break;
			case CAM_BUF_KUNMAP:
				map_info->kmap_counts--;
				break;
			default :
				pr_err("fail to support type %d\n", type);
				break;
			}
		}
	}
	spin_unlock_irqrestore(&g_mem_dbg->buf_map_calc_queue->lock, flags);
}

int cam_buf_monitor_mdbg_check(void)
{
	int val[9] = {0};

	val[0] = atomic_read(&g_mem_dbg->ion_alloc_cnt);
	val[1] = atomic_read(&g_mem_dbg->ion_kmap_cnt);
	val[2] = atomic_read(&g_mem_dbg->ion_dma_cnt);
	val[3] = atomic_read(&g_mem_dbg->empty_frm_cnt);
	val[4] = atomic_read(&g_mem_dbg->iommu_map_cnt[0]);
	val[5] = atomic_read(&g_mem_dbg->iommu_map_cnt[1]);
	val[6] = atomic_read(&g_mem_dbg->mem_kzalloc_cnt);
	val[7] = atomic_read(&g_mem_dbg->mem_vzalloc_cnt);
	val[8] = atomic_read(&g_mem_dbg->ion_alloc_size);

	pr_info("mdbg info: k_alloc_cnt: %d, k_map_cnt: %d, u_ion_cnt: %d, frm_cnt: %d, isp_map_cnt: %d, "
			"dcam_map_cnt: %d kzalloc_cnt: %d vzalloc_cnt: %d k_alloc_size: %d Bytes\n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
	return 0;
}

void cam_buf_monitor_memory_queue_init(void)
{
	g_mem_dbg->buf_map_calc_queue = &s_buf_map_calc_queue;
	g_mem_dbg->memory_alloc_queue = &s_memory_alloc_queue;
	g_mem_dbg->empty_memory_alloc_queue = &s_empty_memory_alloc_queue;
	CAM_QUEUE_INIT(g_mem_dbg->buf_map_calc_queue, CAM_BUF_MAP_CALC_Q_LEN, cambufmonitor_map_calc_queue_destory);
	CAM_QUEUE_INIT(g_mem_dbg->memory_alloc_queue, CAM_BUF_MEMORY_ALLOC_Q_LEN, cambufmonitor_memory_alloc_queue_destory);
	CAM_QUEUE_INIT(g_mem_dbg->empty_memory_alloc_queue, CAM_BUF_EMPTY_MEMORY_ALLOC_Q_LEN, cambufmonitor_memory_alloc_queue_destory);
	cambufmonitor_empty_memory_alloc_queue_get();
}

void cam_buf_monitor_memory_queue_deinit(void)
{
	CAM_QUEUE_CLEAN(g_mem_dbg->buf_map_calc_queue, struct cam_buf_map_info, list);
	g_mem_dbg->buf_map_calc_queue = NULL;
	CAM_QUEUE_CLEAN(g_mem_dbg->memory_alloc_queue, struct cam_buf_alloc_info, list);
	g_mem_dbg->memory_alloc_queue = NULL;
	CAM_QUEUE_CLEAN(g_mem_dbg->empty_memory_alloc_queue, struct cam_buf_alloc_info, list);
	g_mem_dbg->empty_memory_alloc_queue = NULL;
}

void cam_buf_monitor_memory_queue_check(void)
{
	struct cam_buf_map_info *map_info = NULL, *map_info_next = NULL;
	struct cam_buf_alloc_info *alloc_info = NULL;

	CAM_QUEUE_FOR_EACH_ENTRY_SAFE(map_info, map_info_next, &g_mem_dbg->buf_map_calc_queue->head, list) {
		if (!map_info)
			continue;
		if (map_info->buf_memcyp) {
			if (map_info->kmap_counts || map_info->dcam_iommumap_counts || map_info->isp_iommumap_counts)
				pr_err("fail to unmap buf_info :%p, kmap_counts %d, dcam_iommumap_counts %d, isp_iommumap_counts %d, buf_time %06d.%06d, map_time %06d.%06d\n",
					map_info->buf_info, map_info->kmap_counts, map_info->dcam_iommumap_counts, map_info->isp_iommumap_counts,
					(int)map_info->buf_time.tv_sec, (int)map_info->buf_time.tv_usec, (int)map_info->map_time.tv_sec, (int)map_info->map_time.tv_usec);
			CAM_QUEUE_LIST_DEL(&map_info->list);
			g_mem_dbg->buf_map_calc_queue->cnt--;
			cam_buf_kernel_sys_kfree(map_info);
			map_info = NULL;
		} else {
			pr_err("fail to buf_free buf_info: %p, buf_time %06d.%06d, map_time %06d.%06d\n",
				map_info->buf_info, (int)map_info->buf_time.tv_sec, (int)map_info->buf_time.tv_usec, (int)map_info->map_time.tv_sec, (int)map_info->map_time.tv_usec);
		}
	}

	CAM_QUEUE_FOR_EACH_ENTRY(alloc_info, &g_mem_dbg->memory_alloc_queue->head, list) {
		if (!map_info)
			continue;
		pr_err("fail to free memory addr: %p, alloc_time %06d.%06d\n",
			alloc_info->addr, (int)alloc_info->time.tv_sec, (int)alloc_info->time.tv_usec);
	}
}

