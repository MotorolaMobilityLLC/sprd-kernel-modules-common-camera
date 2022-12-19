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

#ifndef _CAM_BUF_H_
#define _CAM_BUF_H_

#include <linux/types.h>
#include <linux/device.h>
#include <linux/sprd_iommu.h>

#include "cam_kernel_adapt.h"

#define CAM_BUF_CAHCED                     (1 << 31)
#define CAM_BUF_MAP_CALC_Q_LEN              4000
#define CAM_BUF_MEMORY_ALLOC_Q_LEN          4000
#define CAM_BUF_EMPTY_MEMORY_ALLOC_Q_LEN    4000

enum cam_buf_type {
	CAM_BUF_NONE,
	CAM_BUF_USER,
	CAM_BUF_KERNEL,
};

enum {
	CAM_BUF_MAPPING_NULL = 0,
	CAM_BUF_MAPPING_DEV = (1 << 1),
	CAM_BUF_MAPPING_KERNEL = (1 << 2),
};

enum cam_iommudev_type {
	CAM_IOMMUDEV_ISP,
	CAM_IOMMUDEV_DCAM,
	CAM_IOMMUDEV_DCAM_LITE,
	CAM_IOMMUDEV_FD,
	CAM_IOMMUDEV_MAX,
};

/* BIT 0: CAM_BUF_ALLOC
 * BIT 1: CAM_BUF_WITH_ION
 * BIT 2: CAM_BUF_WITH_IOVA
 * BIT 3: CAM_BUF_WITH_SINGLE_PAGE_IOVA
 * BIT 4: CAM_BUF_WITH_K_ADDR
 * =================buf status move step==============
 * CAM_BUF_EMPTY    ->CAM_BUF_ALLOC                      ->CAM_BUF_WITH_ION
 * CAM_BUF_WITH_ION ->CAM_BUF_WITH_IOVA                  -> CAM_BUF_WITH_IOVA_K_ADDR
 *                  ->CAM_BUF_WITH_SINGLE_PAGE_IOVA
 *                  ->CAM_BUF_WITH_K_ADDR                -> CAM_BUF_WITH_IOVA_K_ADDR
 *                  ->CAM_BUF_WITH_IOVA_K_ADDR
 * CAM_BUF_WITH_IOVA_K_ADDR = CAM_BUF_WITH_IOVA | CAM_BUF_WITH_K_ADDR
*/

enum cam_buf_status {
	CAM_BUF_EMPTY = 0,
	CAM_BUF_ALLOC = 0x1,
	CAM_BUF_WITH_ION = 0x2,
	CAM_BUF_WITH_IOVA = 0x4,
	CAM_BUF_WITH_SINGLE_PAGE_IOVA = 0x8,
	CAM_BUF_WITH_K_ADDR = 0x10,
	CAM_BUF_WITH_IOVA_K_ADDR = 0x14,
	CAM_BUF_STATUS_NUM,
};

struct cam_buf_memalloc_info {
	void *addr;
	timeval time;
	struct list_head list;
};

struct cam_buf_map_info {
	uint32_t buf_memcyp;
	uint32_t kmap_counts;
	uint32_t dcam_iommumap_counts;
	uint32_t isp_iommumap_counts;
	timeval buf_time;
	timeval map_time;
	struct camera_buf *buf_info;
	struct list_head list;
};

enum cam_buf_source {
	CAM_BUF_SOURCE_ALLOC_GET,
	CAM_BUF_SOURCE_MEMCPY,
	CAM_BUF_SOURCE_MAX,
};

enum cam_buf_map_type {
	CAM_BUF_DCAM_IOMMUMAP,
	CAM_BUF_DCAM_IOMMUUNMAP,
	CAM_BUF_ISP_IOMMUMAP,
	CAM_BUF_ISP_IOMMUUNMAP,
	CAM_BUF_KMAP,
	CAM_BUF_KUNMAP,
	CAM_BUF_MAP_MAX,
};

struct camera_buf {
	bool buf_sec;
	/* user buffer info */
	uint32_t mfd;
	struct dma_buf *dmabuf_p;
	void *ionbuf;/* for iommu map */
	uint32_t offset[3];
	size_t size;
	unsigned long addr_vir[3];
	unsigned long addr_k;
	unsigned long iova;
	struct device *dev;/* mapped device */
	enum cam_buf_type type;
	uint32_t mapping_state;
	struct dma_buf_attachment *attachment[CAM_IOMMUDEV_MAX];
	struct sg_table *table[CAM_IOMMUDEV_MAX];
	uint32_t status;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	struct dma_buf_map map;/* for k515 dambuf */
#endif
};

struct camera_ion_info {
	struct list_head list;
	struct camera_buf ionbuf_copy;
};

int cam_buf_iommudev_reg(struct device *dev,
	enum cam_iommudev_type type);
int cam_buf_iommudev_unreg(enum cam_iommudev_type type);
int cam_buf_iommu_status_get(enum cam_iommudev_type type);

int cam_buf_ionbuf_get(struct camera_buf *buf_info);
int cam_buf_ionbuf_put(struct camera_buf *buf_info);
int cam_buf_iommu_single_page_map(struct camera_buf *buf_info,
	enum cam_iommudev_type type);
int cam_buf_iommu_restore(enum cam_iommudev_type type);
int cam_buf_iommu_map(struct camera_buf *buf_info,
	enum cam_iommudev_type type);
int cam_buf_iommu_unmap(struct camera_buf *buf_info);

int cam_buf_kmap(struct camera_buf *buf_info);
int cam_buf_kunmap(struct camera_buf *buf_info);

int  cam_buf_alloc(struct camera_buf *buf_info, size_t size,
		unsigned int iommu_enable);
int cam_buf_free(struct camera_buf *buf_info);

void *cam_buf_kernel_sys_kzalloc(unsigned long size, gfp_t flags);
void cam_buf_kernel_sys_kfree(const void *mem);
void *cam_buf_kernel_sys_vzalloc(unsigned long size);
void cam_buf_kernel_sys_vfree(const void *mem);
void cam_buf_memory_leak_queue_deinit(void);
void cam_buf_memory_leak_queue_init(void);
void cam_buf_memory_leak_queue_check(void);

int cam_buf_mdbg_check(void);
#endif/* _CAM_BUF_H_ */
