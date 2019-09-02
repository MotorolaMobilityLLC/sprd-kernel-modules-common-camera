/*
 * XRP: Linux device driver for Xtensa Remote Processing
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#include <linux/version.h>
#include <linux/atomic.h>
#include <linux/acpi.h>
#include <linux/completion.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
#include <linux/dma-mapping.h>
#else
#include <linux/dma-direct.h>
#endif
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/highmem.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include "xrp_cma_alloc.h"
#include "xrp_firmware.h"
#include "xrp_hw.h"
#include "xrp_internal.h"
#include "xrp_kernel_defs.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_private_alloc.h"
#ifdef USE_SPRD_MODE
//#define USE_SPRD_MODE
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include "ion.h"
#include "vdsp_smem.h"
#include "vdsp_ipi_drv.h"
#include "vdsp_trusty.h"
#include "xrp_faceid.h"
/*add temp for set 512M as default clock*/
#include <linux/clk.h>

#ifdef USE_LOAD_LIB
#include "xrp_library_loader.h"
#include "xt_library_loader.h"
//#define LIBRARY_LOAD_UNLOAD_NSID        "lib_load"
//#define LIBRARY_LOAD_LOAD_FLAG      0
//#define LIBRARY_LOAD_UNLOAD_FLAG    1
#define LIBRARY_CMD_PIL_INFO_OFFSET   40
#define LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE 44

#define XRP_EXAMPLE_V3_NSID_INITIALIZER \
{0x73, 0x79, 0x73, 0x74, 0x65, 0x6d, 0x20, 0x63, \
	0x6d, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define LIBRARY_LOAD_UNLOAD_NSID (unsigned char [])XRP_EXAMPLE_V3_NSID_INITIALIZER

#endif

#define VDSP_FIRMWIRE_SIZE    (1024*1024*14)
#define VDSP_FACEID_FIRMWIRE_SIZE    (1024*1024*16)
#endif

#define DRIVER_NAME "xrp"
#define XRP_DEFAULT_TIMEOUT 100

#ifndef __io_virt
#define __io_virt(a) ((void __force *)(a))
#endif

struct xrp_alien_mapping {
	unsigned long vaddr;
	unsigned long size;
	phys_addr_t paddr;
	void *allocation;
	enum {
		ALIEN_GUP,
		ALIEN_PFN_MAP,
		ALIEN_COPY,
	} type;
};

struct xrp_mapping {
	enum {
		XRP_MAPPING_NONE,
		XRP_MAPPING_NATIVE,
		XRP_MAPPING_ALIEN,
		XRP_MAPPING_KERNEL = 0x4,
	} type;
	union {
		struct {
			struct xrp_allocation *xrp_allocation;
			unsigned long vaddr;
		} native;
		struct xrp_alien_mapping alien_mapping;
	};
};

struct xvp_file {
	struct xvp *xvp;
	spinlock_t busy_list_lock;
	struct xrp_allocation *busy_list;
};

struct xrp_known_file {
	void *filp;
	struct hlist_node node;
};
static struct mutex map_lock;
static struct semaphore log_start;

static int firmware_command_timeout = XRP_DEFAULT_TIMEOUT;
module_param(firmware_command_timeout, int, 0644);
MODULE_PARM_DESC(firmware_command_timeout, "Firmware command timeout in seconds.");

static int firmware_reboot = 1;
module_param(firmware_reboot, int, 0644);
MODULE_PARM_DESC(firmware_reboot, "Reboot firmware on command timeout.");

enum {
	LOOPBACK_NORMAL,	/* normal work mode */
	LOOPBACK_NOIO,		/* don't communicate with FW, but still load it and control DSP */
	LOOPBACK_NOMMIO,	/* don't comminicate with FW or use DSP MMIO, but still load the FW */
	LOOPBACK_NOFIRMWARE,	/* don't communicate with FW or use DSP MMIO, don't load the FW */
};
static int loopback = 0;
module_param(loopback, int, 0644);
MODULE_PARM_DESC(loopback, "Don't use actual DSP, perform everything locally.");

static DEFINE_HASHTABLE(xrp_known_files, 10);
static DEFINE_SPINLOCK(xrp_known_files_lock);

static DEFINE_IDA(xvp_nodeid);

static int xrp_boot_firmware(struct xvp *xvp);
static inline void xvp_dsp_setdvfs(struct xvp *xvp , uint32_t index);
static inline void xvp_dsp_enable_dvfs(struct xvp *xvp);
static inline void xvp_dsp_disable_dvfs(struct xvp *xvp);

void sprd_log_sem_init(void)
{
	sema_init(&log_start, 0);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_init);

void sprd_log_sem_up(void)
{
	up(&log_start);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_up);

void sprd_log_sem_down(void)
{
	down(&log_start);
}
EXPORT_SYMBOL_GPL(sprd_log_sem_down);

static bool xrp_cacheable(struct xvp *xvp, unsigned long pfn,
			  unsigned long n_pages)
{
	if (xvp->hw_ops->cacheable) {
		return xvp->hw_ops->cacheable(xvp->hw_arg, pfn, n_pages);
	} else {
		unsigned long i;

		for (i = 0; i < n_pages; ++i)
			if (!pfn_valid(pfn + i))
				return false;
		return true;
	}
}

#ifndef USE_SPRD_MODE
static int xrp_dma_direction(unsigned flags)
{
	static const enum dma_data_direction xrp_dma_direction[] = {
		[0] = DMA_NONE,
		[XRP_FLAG_READ] = DMA_TO_DEVICE,
		[XRP_FLAG_WRITE] = DMA_FROM_DEVICE,
		[XRP_FLAG_READ_WRITE] = DMA_BIDIRECTIONAL,
	};
	return xrp_dma_direction[flags & XRP_FLAG_READ_WRITE];
}
#endif

#ifndef USE_SPRD_MODE
static void xrp_default_dma_sync_for_device(struct xvp *xvp,
					    phys_addr_t phys,
					    unsigned long size,
					    unsigned long flags)
{
	dma_sync_single_for_device(xvp->dev, phys_to_dma(xvp->dev, phys), size,
				   xrp_dma_direction(flags));
}
#endif

#ifndef USE_SPRD_MODE
static void xrp_dma_sync_for_device(struct xvp *xvp,
				    unsigned long virt,
				    phys_addr_t phys,
				    unsigned long size,
				    unsigned long flags)
{
	if (xvp->hw_ops->dma_sync_for_device)
		xvp->hw_ops->dma_sync_for_device(xvp->hw_arg,
						 (void *)virt, phys, size,
						 flags);
	else
		xrp_default_dma_sync_for_device(xvp, phys, size, flags);
}
#endif

#ifndef USE_SPRD_MODE
static void xrp_default_dma_sync_for_cpu(struct xvp *xvp,
					 phys_addr_t phys,
					 unsigned long size,
					 unsigned long flags)
{
	dma_sync_single_for_cpu(xvp->dev, phys_to_dma(xvp->dev, phys), size,
				xrp_dma_direction(flags));
}
#endif

#ifndef USE_SPRD_MODE
static void xrp_dma_sync_for_cpu(struct xvp *xvp,
				 unsigned long virt,
				 phys_addr_t phys,
				 unsigned long size,
				 unsigned long flags)
{
	if (xvp->hw_ops->dma_sync_for_cpu)
		xvp->hw_ops->dma_sync_for_cpu(xvp->hw_arg,
					      (void *)virt, phys, size,
					      flags);
	else
		xrp_default_dma_sync_for_cpu(xvp, phys, size, flags);
}
#endif

static inline void xrp_comm_write32(volatile void __iomem *addr, u32 v)
{
	__raw_writel(v, addr);
}

static inline u32 xrp_comm_read32(volatile void __iomem *addr)
{
	return __raw_readl(addr);
}

static inline void __iomem *xrp_comm_put_tlv(void __iomem **addr,
					     uint32_t type,
					     uint32_t length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	xrp_comm_write32(&tlv->type, type);
	xrp_comm_write32(&tlv->length, length);
	*addr = tlv->value + ((length + 3) / 4);
	return tlv->value;
}

static inline void __iomem *xrp_comm_get_tlv(void __iomem **addr,
					     uint32_t *type,
					     uint32_t *length)
{
	struct xrp_dsp_tlv __iomem *tlv = *addr;

	*type = xrp_comm_read32(&tlv->type);
	*length = xrp_comm_read32(&tlv->length);
	*addr = tlv->value + ((*length + 3) / 4);
	return tlv->value;
}

static inline void xrp_comm_write(volatile void __iomem *addr, const void *p,
				  size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v = 0;

	while (sz32) {
		memcpy(&v, p, sizeof(v));
		__raw_writel(v, addr);
		p += 4;
		addr += 4;
		sz32 -= 4;
	}
	sz &= 3;
	if (sz) {
		v = 0;
		memcpy(&v, p, sz);
		__raw_writel(v, addr);
	}
}

static inline void xrp_comm_read(volatile void __iomem *addr, void *p,
				 size_t sz)
{
	size_t sz32 = sz & ~3;
	u32 v = 0;

	while (sz32) {
		v = __raw_readl(addr);
		memcpy(p, &v, sizeof(v));
		p += 4;
		addr += 4;
		sz32 -= 4;
	}
	sz &= 3;
	if (sz) {
		v = __raw_readl(addr);
		memcpy(p, &v, sz);
	}
}

static inline void xrp_send_device_irq(struct xvp *xvp)
{
	if (xvp->hw_ops->send_irq)
		xvp->hw_ops->send_irq(xvp->hw_arg);
}

static inline bool xrp_panic_check(struct xvp *xvp)
{
	if (xvp->hw_ops->panic_check)
		return xvp->hw_ops->panic_check(xvp->hw_arg);
	else
		return false;
}

static void xrp_add_known_file(struct file *filp)
{
	struct xrp_known_file *p = kmalloc(sizeof(*p), GFP_KERNEL);

	if (!p)
		return;

	p->filp = filp;
	spin_lock(&xrp_known_files_lock);
	hash_add(xrp_known_files, &p->node, (unsigned long)filp);
	spin_unlock(&xrp_known_files_lock);
}

static void xrp_remove_known_file(struct file *filp)
{
	struct xrp_known_file *p;
	struct xrp_known_file *pf = NULL;

	spin_lock(&xrp_known_files_lock);
	hash_for_each_possible(xrp_known_files, p, node, (unsigned long)filp) {
		if (p->filp == filp) {
			hash_del(&p->node);
			pf = p;
			break;
		}
	}
	spin_unlock(&xrp_known_files_lock);
	if (pf)
		kfree(pf);
}
#ifndef USE_SPRD_MODE
static bool xrp_is_known_file(struct file *filp)
{
	bool ret = false;
	struct xrp_known_file *p;

	spin_lock(&xrp_known_files_lock);
	hash_for_each_possible(xrp_known_files, p, node, (unsigned long)filp) {
		if (p->filp == filp) {
			ret = true;
			break;
		}
	}
	spin_unlock(&xrp_known_files_lock);
	return ret;
}
#endif

static void xrp_sync_v2(struct xvp *xvp,
			void *hw_sync_data, size_t sz)
{
	struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
	void __iomem *addr = shared_sync->hw_sync_data;

	xrp_comm_write(xrp_comm_put_tlv(&addr,
					XRP_DSP_SYNC_TYPE_HW_SPEC_DATA, sz),
		       hw_sync_data, sz);
	if (xvp->n_queues > 1) {
		struct xrp_dsp_sync_v2 __iomem *queue_sync;
		unsigned i;

		xrp_comm_write(xrp_comm_put_tlv(&addr,
						XRP_DSP_SYNC_TYPE_HW_QUEUES,
						xvp->n_queues * sizeof(u32)),
			       xvp->queue_priority,
			       xvp->n_queues * sizeof(u32));
		for (i = 1; i < xvp->n_queues; ++i) {
			queue_sync = xvp->queue[i].comm;
			xrp_comm_write32(&queue_sync->sync,
					 XRP_DSP_SYNC_IDLE);
		}
	}
	xrp_comm_put_tlv(&addr, XRP_DSP_SYNC_TYPE_LAST, 0);
}

static int xrp_sync_complete_v2(struct xvp *xvp, size_t sz)
{
	struct xrp_dsp_sync_v2 __iomem *shared_sync = xvp->comm;
	void __iomem *addr = shared_sync->hw_sync_data;
	u32 type, len;

	xrp_comm_get_tlv(&addr, &type, &len);
	if (len != sz) {
		dev_err(xvp->dev,
			"HW spec data size modified by the DSP\n");
		return -EINVAL;
	}
	if (!(type & XRP_DSP_SYNC_TYPE_ACCEPT))
		dev_info(xvp->dev,
			 "HW spec data not recognized by the DSP\n");

	if (xvp->n_queues > 1) {
		void __iomem *p = xrp_comm_get_tlv(&addr, &type, &len);

		if (len != xvp->n_queues * sizeof(u32)) {
			dev_err(xvp->dev,
				"Queue priority size modified by the DSP\n");
			return -EINVAL;
		}
		if (type & XRP_DSP_SYNC_TYPE_ACCEPT) {
			xrp_comm_read(p, xvp->queue_priority,
				      xvp->n_queues * sizeof(u32));
		} else {
			dev_info(xvp->dev,
				 "Queue priority data not recognized by the DSP\n");
			xvp->n_queues = 1;
		}
	}
	return 0;
}
static int xrp_faceid_run(struct xvp *xvp,__u32 yuv_addr,__u32 in_height,__u32 in_width)
{
	unsigned long deadline = jiffies + 100 * HZ;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	u32 v;

	struct faceid_hw_sync_data faceid_data;
	faceid_data.fd_p_coffe_addr = xvp->faceid_pool.ion_fd_weights_p.addr_p[0];
	faceid_data.fd_r_coffe_addr = xvp->faceid_pool.ion_fd_weights_r.addr_p[0];
	faceid_data.fd_o_coffe_addr = xvp->faceid_pool.ion_fd_weights_o.addr_p[0];
	faceid_data.fp_coffe_addr = xvp->faceid_pool.ion_fp_weights.addr_p[0];
	faceid_data.flv_coffe_addr = xvp->faceid_pool.ion_flv_weights.addr_p[0];
	faceid_data.fv_coffe_addr = xvp->faceid_pool.ion_fv_weights.addr_p[0];
	faceid_data.mem_pool_addr = xvp->faceid_pool.ion_fd_mem_pool.addr_p[0];
	faceid_data.yuv_addr = yuv_addr;
	faceid_data.frame_height = in_height;
	faceid_data.frame_width = in_width;
	printk("fd_p %X,fd_r %X,fd_o %X,\n",faceid_data.fd_p_coffe_addr,faceid_data.fd_r_coffe_addr,faceid_data.fd_o_coffe_addr);
	printk("fp %X,flv %X,fv %X\n",faceid_data.fp_coffe_addr,faceid_data.flv_coffe_addr,faceid_data.fv_coffe_addr);
	printk("mem pool %X\n",faceid_data.mem_pool_addr);

	mb();
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_HOST_TO_DSP);
	mb();
	xrp_comm_write(&shared_sync->hw_sync_data, &faceid_data, sizeof(struct faceid_hw_sync_data));

	mb();
	xrp_send_device_irq(xvp);

	do {
		mb();
		v = xrp_comm_read32(&shared_sync->sync);
		if (v == XRP_DSP_SYNC_DSP_TO_HOST)
		{
			__u32 ret = xrp_comm_read32(&shared_sync->hw_sync_data);
			printk("vdsp faceid ret %X\n",ret);
			break;
		}
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));

err:
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
	return 0;
}
static int xrp_synchronize(struct xvp *xvp)
{
	size_t sz;
	void *hw_sync_data;
	unsigned long deadline = jiffies + firmware_command_timeout * HZ;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	int ret;
	u32 v, v1;

	/*
	 * TODO
	 * BAD METHOD
	 * Just Using sz temp for transfer share memory address
	 */
	if (xvp->vdsp_mem_desc->cb_func[CB_MSG])
		sz = (size_t)xvp->vdsp_mem_desc->cb_func[CB_MSG](
					xvp->vdsp_mem_desc->cb_args[CB_MSG]);
	else
		pr_err("get smsg share memory address failed\n");

	pr_err("get smsg share memory address: 0x%lx\n", (unsigned long)sz);
	hw_sync_data = xvp->hw_ops->get_hw_sync_data(xvp->hw_arg, &sz);
	if (!hw_sync_data) {
		ret = -ENOMEM;
		goto err;
	}
	ret = -ENODEV;
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_START);
	mb();
	do {
		v = xrp_comm_read32(&shared_sync->sync);
		printk("yzl add %s v:%d\n" , __func__ , v);
		if (v != XRP_DSP_SYNC_START)
			break;
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));

	switch (v) {
	case XRP_DSP_SYNC_DSP_READY_V1:
		if (xvp->n_queues > 1) {
			dev_info(xvp->dev,
				 "Queue priority data not recognized by the DSP\n");
			xvp->n_queues = 1;
		}
		xrp_comm_write(&shared_sync->hw_sync_data, hw_sync_data, sz);
		break;
	case XRP_DSP_SYNC_DSP_READY_V2:
		xrp_sync_v2(xvp, hw_sync_data, sz);
		break;
	case XRP_DSP_SYNC_START:
		printk("yzl add %s , DSP is not ready for synchronization\n" , __func__);
		dev_err(xvp->dev, "DSP is not ready for synchronization\n");
		goto err;
	default:
		dev_err(xvp->dev,
			"DSP response to XRP_DSP_SYNC_START is not recognized\n");
		goto err;
	}

	mb();
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_HOST_TO_DSP);

	do {
		mb();
		v1 = xrp_comm_read32(&shared_sync->sync);
		if (v1 == XRP_DSP_SYNC_DSP_TO_HOST)
			break;
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));

	if (v1 != XRP_DSP_SYNC_DSP_TO_HOST) {
		dev_err(xvp->dev,
			"DSP haven't confirmed initialization data reception\n");
		goto err;
	}

	if (v == XRP_DSP_SYNC_DSP_READY_V2) {
		ret = xrp_sync_complete_v2(xvp, sz);
		if (ret < 0)
			goto err;
	}

	xrp_send_device_irq(xvp);

	if (xvp->host_irq_mode) {
		int res = wait_for_completion_timeout(&xvp->queue[0].completion,
						      firmware_command_timeout * HZ);

		ret = -ENODEV;
		if (xrp_panic_check(xvp))
			goto err;
		if (res == 0) {
			dev_err(xvp->dev,
				"host IRQ mode is requested, but DSP couldn't deliver IRQ during synchronization\n");
			goto err;
		}
	}
	ret = 0;
err:
	kfree(hw_sync_data);
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
	return ret;
}

static bool xrp_cmd_complete(struct xrp_comm *xvp)
{
	struct xrp_dsp_cmd __iomem *cmd = xvp->comm;
	u32 flags = xrp_comm_read32(&cmd->flags);

	rmb();
	return (flags & (XRP_DSP_CMD_FLAG_REQUEST_VALID |
			 XRP_DSP_CMD_FLAG_RESPONSE_VALID)) ==
		(XRP_DSP_CMD_FLAG_REQUEST_VALID |
		 XRP_DSP_CMD_FLAG_RESPONSE_VALID);
}

irqreturn_t xrp_irq_handler(int irq, struct xvp *xvp)
{
	unsigned i, n = 0;

	dev_dbg(xvp->dev, "%s\n", __func__);
	if (!xvp->comm)
		return IRQ_NONE;

	for (i = 0; i < xvp->n_queues; ++i) {
		if (xrp_cmd_complete(xvp->queue + i)) {
			dev_dbg(xvp->dev, "  completing queue %d\n", i);
			complete(&xvp->queue[i].completion);
			++n;
		}
	}
	return n ? IRQ_HANDLED : IRQ_NONE;
}
EXPORT_SYMBOL(xrp_irq_handler);

static inline void xvp_file_lock(struct xvp_file *xvp_file)
{
	spin_lock(&xvp_file->busy_list_lock);
}

static inline void xvp_file_unlock(struct xvp_file *xvp_file)
{
	spin_unlock(&xvp_file->busy_list_lock);
}

static void xrp_allocation_queue(struct xvp_file *xvp_file,
				 struct xrp_allocation *xrp_allocation)
{
	xvp_file_lock(xvp_file);

	xrp_allocation->next = xvp_file->busy_list;
	xvp_file->busy_list = xrp_allocation;

	xvp_file_unlock(xvp_file);
}

static struct xrp_allocation *xrp_allocation_dequeue(struct xvp_file *xvp_file,
						     phys_addr_t paddr, u32 size)
{
	struct xrp_allocation **pcur;
	struct xrp_allocation *cur;

	xvp_file_lock(xvp_file);

	for (pcur = &xvp_file->busy_list; (cur = *pcur); pcur = &((*pcur)->next)) {
		printk("%s: %pap / %pap x %d\n", __func__, &paddr, &cur->start, cur->size);
		if (paddr >= cur->start && paddr + size - cur->start <= cur->size) {
			*pcur = cur->next;
			break;
		}
	}

	xvp_file_unlock(xvp_file);
	return cur;
}

static long xrp_ioctl_alloc(struct file *filp,
			    struct xrp_ioctl_alloc __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xrp_allocation *xrp_allocation;
	unsigned long vaddr;
	struct xrp_ioctl_alloc xrp_ioctl_alloc;
	long err;

	printk("%s: %p\n", __func__, p);
	if (copy_from_user(&xrp_ioctl_alloc, p, sizeof(*p)))
		return -EFAULT;

	printk("%s: size = %d, align = %x\n", __func__,
	       xrp_ioctl_alloc.size, xrp_ioctl_alloc.align);

	err = xrp_allocate(xvp_file->xvp->pool,
			   xrp_ioctl_alloc.size,
			   xrp_ioctl_alloc.align,
			   &xrp_allocation);
	if (err)
		return err;

	xrp_allocation_queue(xvp_file, xrp_allocation);

	vaddr = vm_mmap(filp, 0, xrp_allocation->size,
			PROT_READ | PROT_WRITE, MAP_SHARED,
			xrp_allocation_offset(xrp_allocation));

	xrp_ioctl_alloc.addr = vaddr;

	if (copy_to_user(p, &xrp_ioctl_alloc, sizeof(*p))) {
		vm_munmap(vaddr, xrp_ioctl_alloc.size);
		return -EFAULT;
	}
	return 0;
}
#ifndef USE_SPRD_MODE
static void xrp_put_pages(phys_addr_t phys, unsigned long n_pages)
{
	struct page *page;
	unsigned long i;

	page = pfn_to_page(__phys_to_pfn(phys));
	for (i = 0; i < n_pages; ++i)
		put_page(page + i);
}
#endif

#ifndef USE_SPRD_MODE
static void xrp_alien_mapping_destroy(struct xrp_alien_mapping *alien_mapping)
{
	switch (alien_mapping->type) {
	case ALIEN_GUP:
		xrp_put_pages(alien_mapping->paddr,
			      PFN_UP(alien_mapping->vaddr +
				     alien_mapping->size) -
			      PFN_DOWN(alien_mapping->vaddr));
		break;
	case ALIEN_COPY:
		xrp_allocation_put(alien_mapping->allocation);
		break;
	default:
		break;
	}
}
#endif

#ifndef USE_SPRD_MODE
static long xvp_pfn_virt_to_phys(struct xvp_file *xvp_file,
				 struct vm_area_struct *vma,
				 unsigned long vaddr, unsigned long size,
				 phys_addr_t *paddr,
				 struct xrp_alien_mapping *mapping)
{
	int ret;
	unsigned long i;
	unsigned long nr_pages = PFN_UP(vaddr + size) - PFN_DOWN(vaddr);
	unsigned long pfn;
	const struct xrp_address_map_entry *address_map;

	ret = follow_pfn(vma, vaddr, &pfn);
	if (ret)
		return ret;

	*paddr = __pfn_to_phys(pfn) + (vaddr & ~PAGE_MASK);
	address_map = xrp_get_address_mapping(&xvp_file->xvp->address_map,
					      *paddr);
	if (!address_map) {
		printk("%s: untranslatable addr: %pap\n", __func__, paddr);
		return -EINVAL;
	}

	for (i = 1; i < nr_pages; ++i) {
		unsigned long next_pfn;
		phys_addr_t next_phys;

		ret = follow_pfn(vma, vaddr + (i << PAGE_SHIFT), &next_pfn);
		if (ret)
			return ret;
		if (next_pfn != pfn + 1) {
			printk("%s: non-contiguous physical memory\n",
			       __func__);
			return -EINVAL;
		}
		next_phys = __pfn_to_phys(next_pfn);
		if (xrp_compare_address(next_phys, address_map)) {
			printk("%s: untranslatable addr: %pap\n",
			       __func__, &next_phys);
			return -EINVAL;
		}
		pfn = next_pfn;
	}
	*mapping = (struct xrp_alien_mapping){
		.vaddr = vaddr,
		.size = size,
		.paddr = *paddr,
		.type = ALIEN_PFN_MAP,
	};
	printk("%s: success, paddr: %pap\n", __func__, paddr);
	return 0;
}
#endif

#ifndef USE_SPRD_MODE
static long xvp_gup_virt_to_phys(struct xvp_file *xvp_file,
				 unsigned long vaddr, unsigned long size,
				 phys_addr_t *paddr,
				 struct xrp_alien_mapping *mapping)
{
	int ret;
	int i;
	int nr_pages;
	struct page **page;
	const struct xrp_address_map_entry *address_map;

	if (PFN_UP(vaddr + size) - PFN_DOWN(vaddr) > INT_MAX)
		return -EINVAL;

	nr_pages = PFN_UP(vaddr + size) - PFN_DOWN(vaddr);
	page = kmalloc(nr_pages * sizeof(void *), GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	ret = get_user_pages_fast(vaddr, nr_pages, 1, page);
	if (ret < 0)
		goto out;

	if (ret < nr_pages) {
		printk("%s: asked for %d pages, but got only %d\n",
		       __func__, nr_pages, ret);
		nr_pages = ret;
		ret = -EINVAL;
		goto out_put;
	}

	address_map = xrp_get_address_mapping(&xvp_file->xvp->address_map,
					      page_to_phys(page[0]));
	if (!address_map) {
		phys_addr_t addr = page_to_phys(page[0]);
		printk("%s: untranslatable addr: %pap\n",
		       __func__, &addr);
		ret = -EINVAL;
		goto out_put;
	}

	for (i = 1; i < nr_pages; ++i) {
		phys_addr_t addr;

		if (page[i] != page[i - 1] + 1) {
			printk("%s: non-contiguous physical memory\n",
			       __func__);
			ret = -EINVAL;
			goto out_put;
		}
		addr = page_to_phys(page[i]);
		if (xrp_compare_address(addr, address_map)) {
			printk("%s: untranslatable addr: %pap\n",
			       __func__, &addr);
			ret = -EINVAL;
			goto out_put;
		}
	}

	*paddr = __pfn_to_phys(page_to_pfn(page[0])) + (vaddr & ~PAGE_MASK);
	*mapping = (struct xrp_alien_mapping){
		.vaddr = vaddr,
		.size = size,
		.paddr = *paddr,
		.type = ALIEN_GUP,
	};
	ret = 0;
	printk("%s: success, paddr: %pap\n", __func__, paddr);

out_put:
	if (ret < 0)
		for (i = 0; i < nr_pages; ++i)
			put_page(page[i]);
out:
	kfree(page);
	return ret;
}
#endif

#ifndef USE_SPRD_MODE
static long _xrp_copy_user_phys(struct xvp *xvp,
				unsigned long vaddr, unsigned long size,
				phys_addr_t paddr, unsigned long flags,
				bool to_phys)
{
	if (pfn_valid(__phys_to_pfn(paddr))) {
		struct page *page = pfn_to_page(__phys_to_pfn(paddr));
		size_t page_offs = paddr & ~PAGE_MASK;
		size_t offs;

		if (!to_phys)
			xrp_default_dma_sync_for_cpu(xvp, paddr, size, flags);
		for (offs = 0; offs < size; ++page) {
			void *p = kmap(page);
			size_t sz = PAGE_SIZE - page_offs;
			size_t copy_sz = sz;
			unsigned long rc;

			if (!p)
				return -ENOMEM;

			if (size - offs < copy_sz)
				copy_sz = size - offs;

			if (to_phys)
				rc = copy_from_user(p + page_offs,
						    (void __user *)(vaddr + offs),
						    copy_sz);
			else
				rc = copy_to_user((void __user *)(vaddr + offs),
						  p + page_offs, copy_sz);

			page_offs = 0;
			offs += copy_sz;

			kunmap(page);
			if (rc)
				return -EFAULT;
		}
		if (to_phys)
			xrp_default_dma_sync_for_device(xvp, paddr, size, flags);
	} else {
		void __iomem *p = ioremap(paddr, size);
		unsigned long rc;

		if (!p) {
			dev_err(xvp->dev,
				"couldn't ioremap %pap x 0x%08x\n",
				&paddr, (u32)size);
			return -EINVAL;
		}
		if (to_phys)
			rc = copy_from_user(__io_virt(p),
					    (void __user *)vaddr, size);
		else
			rc = copy_to_user((void __user *)vaddr,
					  __io_virt(p), size);
		iounmap(p);
		if (rc)
			return -EFAULT;
	}
	return 0;
}
#endif

#ifndef USE_SPRD_MODE
static long xrp_copy_user_to_phys(struct xvp *xvp,
				  unsigned long vaddr, unsigned long size,
				  phys_addr_t paddr, unsigned long flags)
{
	return _xrp_copy_user_phys(xvp, vaddr, size, paddr, flags, true);
}
#endif

#ifndef USE_SPRD_MODE
static long xrp_copy_user_from_phys(struct xvp *xvp,
				    unsigned long vaddr, unsigned long size,
				    phys_addr_t paddr, unsigned long flags)
{
	return _xrp_copy_user_phys(xvp, vaddr, size, paddr, flags, false);
}
#endif

#ifndef USE_SPRD_MODE
static long xvp_copy_virt_to_phys(struct xvp_file *xvp_file,
				  unsigned long flags,
				  unsigned long vaddr, unsigned long size,
				  phys_addr_t *paddr,
				  struct xrp_alien_mapping *mapping)
{
	phys_addr_t phys;
	unsigned long align = clamp(vaddr & -vaddr, 16ul, PAGE_SIZE);
	unsigned long offset = vaddr & (align - 1);
	struct xrp_allocation *allocation;
	long rc;

	rc = xrp_allocate(xvp_file->xvp->pool,
			  size + align, align, &allocation);
	if (rc < 0)
		return rc;

	phys = (allocation->start & -align) | offset;
	if (phys < allocation->start)
		phys += align;

	if (flags & XRP_FLAG_READ) {
		if (xrp_copy_user_to_phys(xvp_file->xvp,
					  vaddr, size, phys, flags)) {
			xrp_allocation_put(allocation);
			return -EFAULT;
		}
	}

	*paddr = phys;
	*mapping = (struct xrp_alien_mapping){
		.vaddr = vaddr,
		.size = size,
		.paddr = *paddr,
		.allocation = allocation,
		.type = ALIEN_COPY,
	};
	printk("%s: copying to pa: %pap\n", __func__, paddr);

	return 0;
}
#endif

#ifndef USE_SPRD_MODE
static unsigned xvp_get_region_vma_count(unsigned long virt,
					 unsigned long size,
					 struct vm_area_struct *vma)
{
	unsigned i;
	struct mm_struct *mm = current->mm;

	if (virt + size < virt)
		return 0;
	if (vma->vm_start > virt)
		return 0;
	if (vma->vm_start <= virt &&
	    virt + size <= vma->vm_end)
		return 1;
	for (i = 2; ; ++i) {
		struct vm_area_struct *next_vma = find_vma(mm, vma->vm_end);

		if (!next_vma)
			return 0;
		if (next_vma->vm_start != vma->vm_end)
			return 0;
		vma = next_vma;
		if (virt + size <= vma->vm_end)
			return i;
	}
	return 0;
}
#endif

#ifndef USE_SPRD_MODE
static long xrp_share_kernel(struct file *filp,
			     unsigned long virt, unsigned long size,
			     unsigned long flags, phys_addr_t *paddr,
			     struct xrp_mapping *mapping)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	phys_addr_t phys = __pa(virt);
	long err = 0;

	printk("%s: sharing kernel-only buffer: %pap\n", __func__, &phys);
	if (xrp_translate_to_dsp(&xvp->address_map, phys) ==
	    XRP_NO_TRANSLATION) {
		mm_segment_t oldfs = get_fs();

		printk("%s: untranslatable addr, making shadow copy\n",
		       __func__);
		set_fs(KERNEL_DS);
		err = xvp_copy_virt_to_phys(xvp_file, flags,
					    virt, size, paddr,
					    &mapping->alien_mapping);
		set_fs(oldfs);
		mapping->type = XRP_MAPPING_ALIEN | XRP_MAPPING_KERNEL;
	} else {
		mapping->type = XRP_MAPPING_KERNEL;
		*paddr = phys;

		xrp_default_dma_sync_for_device(xvp, phys, size, flags);
	}
	printk("%s: mapping = %p, mapping->type = %d\n",
	       __func__, mapping, mapping->type);
	return err;
}
#endif

#ifndef USE_SPRD_MODE
static bool vma_needs_cache_ops(struct vm_area_struct *vma)
{
	pgprot_t prot = vma->vm_page_prot;

	return pgprot_val(prot) != pgprot_val(pgprot_noncached(prot)) &&
		pgprot_val(prot) != pgprot_val(pgprot_writecombine(prot));
}
#endif

/* Share blocks of memory, from host to IVP or back.
 *
 * When sharing to IVP return physical addresses in paddr.
 * Areas allocated from the driver can always be shared in both directions.
 * Contiguous 3rd party allocations need to be shared to IVP before they can
 * be shared back.
 */
#ifndef USE_SPRD_MODE
static long __xrp_share_block(struct file *filp,
			      unsigned long virt, unsigned long size,
			      unsigned long flags, phys_addr_t *paddr,
			      struct xrp_mapping *mapping)
{
	phys_addr_t phys = ~0ul;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma = find_vma(mm, virt);
	bool do_cache = true;
	long rc = -EINVAL;

	if (!vma) {
		printk("%s: no vma for vaddr/size = 0x%08lx/0x%08lx\n",
		       __func__, virt, size);
		return -EINVAL;
	}
	/*
	 * Region requested for sharing should be within single VMA.
	 * That's true for the majority of cases, but sometimes (e.g.
	 * sharing buffer in the beginning of .bss which shares a
	 * file-mapped page with .data, followed by anonymous page)
	 * region will cross multiple VMAs. Support it in the simplest
	 * way possible: start with get_user_pages and use shadow copy
	 * if that fails.
	 */
	switch (xvp_get_region_vma_count(virt, size, vma)) {
	case 0:
		printk("%s: bad vma for vaddr/size = 0x%08lx/0x%08lx\n",
		       __func__, virt, size);
		printk("%s: vma->vm_start = 0x%08lx, vma->vm_end = 0x%08lx\n",
		       __func__, vma->vm_start, vma->vm_end);
		return -EINVAL;
	case 1:
		break;
	default:
		printk("%s: multiple vmas cover vaddr/size = 0x%08lx/0x%08lx\n",
		       __func__, virt, size);
		vma = NULL;
		break;
	}
	/*
	 * And it need to be allocated from the same file descriptor, or
	 * at least from a file descriptor managed by the XRP.
	 */
	if (vma &&
	    (vma->vm_file == filp || xrp_is_known_file(vma->vm_file))) {
		struct xvp_file *vm_file = vma->vm_file->private_data;
		struct xrp_allocation *xrp_allocation = vma->vm_private_data;

		phys = vm_file->xvp->pmem + (vma->vm_pgoff << PAGE_SHIFT) +
			virt - vma->vm_start;
		printk("%s: XRP allocation at 0x%08lx, paddr: %pap\n",
		       __func__, virt, &phys);
		/*
		 * If it was allocated from a different XRP file it may belong
		 * to a different device and not be directly accessible.
		 * Check if it is.
		 */
		if (vma->vm_file != filp) {
			const struct xrp_address_map_entry *address_map =
				xrp_get_address_mapping(&xvp->address_map,
							phys);

			if (!address_map ||
			    xrp_compare_address(phys + size - 1, address_map))
				printk("%s: untranslatable addr: %pap\n",
				       __func__, &phys);
			else
				rc = 0;

		} else {
			rc = 0;
		}

		if (rc == 0) {
			mapping->type = XRP_MAPPING_NATIVE;
			mapping->native.xrp_allocation = xrp_allocation;
			mapping->native.vaddr = virt;
			xrp_allocation_get(xrp_allocation);
			do_cache = vma_needs_cache_ops(vma);
		}
	}
	if (rc < 0) {
		struct xrp_alien_mapping *alien_mapping =
			&mapping->alien_mapping;
		unsigned long n_pages = PFN_UP(virt + size) - PFN_DOWN(virt);

		/* Otherwise this is alien allocation. */
		printk("%s: non-XVP allocation at 0x%08lx\n",
		       __func__, virt);

		/*
		 * A range can only be mapped directly if it is either
		 * uncached or HW-specific cache operations can handle it.
		 */
		if (vma && vma->vm_flags & (VM_IO | VM_PFNMAP)) {
			rc = xvp_pfn_virt_to_phys(xvp_file, vma,
						  virt, size,
						  &phys,
						  alien_mapping);
			if (rc == 0 && vma_needs_cache_ops(vma) &&
			    !xrp_cacheable(xvp, PFN_DOWN(phys), n_pages)) {
				printk("%s: needs unsupported cache mgmt\n",
				       __func__);
				rc = -EINVAL;
			}
		} else {
			up_read(&mm->mmap_sem);
			rc = xvp_gup_virt_to_phys(xvp_file, virt,
						  size, &phys,
						  alien_mapping);
			if (rc == 0 &&
			    (!vma || vma_needs_cache_ops(vma)) &&
			    !xrp_cacheable(xvp, PFN_DOWN(phys), n_pages)) {
				printk("%s: needs unsupported cache mgmt\n",
				       __func__);
				xrp_put_pages(phys, n_pages);
				rc = -EINVAL;
			}
			down_read(&mm->mmap_sem);
		}
		if (rc == 0 && vma && !vma_needs_cache_ops(vma))
			do_cache = false;

		/*
		 * If we couldn't share try to make a shadow copy.
		 */
		if (rc < 0) {
			rc = xvp_copy_virt_to_phys(xvp_file, flags,
						   virt, size, &phys,
						   alien_mapping);
			do_cache = false;
		}

		/* We couldn't share it. Fail the request. */
		if (rc < 0) {
			printk("%s: couldn't map virt to phys\n",
			       __func__);
			return -EINVAL;
		}

		phys = alien_mapping->paddr +
			virt - alien_mapping->vaddr;

		mapping->type = XRP_MAPPING_ALIEN;
	}

	*paddr = phys;
	printk("%s: mapping = %p, mapping->type = %d\n",
	       __func__, mapping, mapping->type);

	if (do_cache)
		xrp_dma_sync_for_device(xvp,
					virt, phys, size,
					flags);
	return 0;
}
#endif

#ifndef USE_SPRD_MODE
static long xrp_writeback_alien_mapping(struct xvp_file *xvp_file,
					struct xrp_alien_mapping *alien_mapping,
					unsigned long flags)
{
	struct page *page;
	size_t nr_pages;
	size_t i;
	long ret = 0;

	switch (alien_mapping->type) {
	case ALIEN_GUP:
		xrp_dma_sync_for_cpu(xvp_file->xvp,
				     alien_mapping->vaddr,
				     alien_mapping->paddr,
				     alien_mapping->size,
				     flags);
		printk("%s: dirtying alien GUP @va = %p, pa = %pap\n",
		       __func__, (void __user *)alien_mapping->vaddr,
		       &alien_mapping->paddr);
		page = pfn_to_page(__phys_to_pfn(alien_mapping->paddr));
		nr_pages = PFN_UP(alien_mapping->vaddr + alien_mapping->size) -
			PFN_DOWN(alien_mapping->vaddr);
		for (i = 0; i < nr_pages; ++i)
			SetPageDirty(page + i);
		break;

	case ALIEN_COPY:
		printk("%s: synchronizing alien copy @pa = %pap back to %p\n",
		       __func__, &alien_mapping->paddr,
		       (void __user *)alien_mapping->vaddr);
		if (xrp_copy_user_from_phys(xvp_file->xvp,
					    alien_mapping->vaddr,
					    alien_mapping->size,
					    alien_mapping->paddr,
					    flags))
			ret = -EINVAL;
		break;

	default:
		break;
	}
	return ret;
}
#endif

/*
 *
 */
#ifndef USE_SPRD_MODE
static long __xrp_unshare_block(struct file *filp, struct xrp_mapping *mapping,
				unsigned long flags)
{
	long ret = 0;
	mm_segment_t oldfs = get_fs();

	if (mapping->type & XRP_MAPPING_KERNEL)
		set_fs(KERNEL_DS);

	switch (mapping->type & ~XRP_MAPPING_KERNEL) {
	case XRP_MAPPING_NATIVE:
		if (flags & XRP_FLAG_WRITE) {
			struct xvp_file *xvp_file = filp->private_data;

			xrp_dma_sync_for_cpu(xvp_file->xvp,
					     mapping->native.vaddr,
					     mapping->native.xrp_allocation->start,
					     mapping->native.xrp_allocation->size,
					     flags);

		}
		xrp_allocation_put(mapping->native.xrp_allocation);
		break;

	case XRP_MAPPING_ALIEN:
		if (flags & XRP_FLAG_WRITE)
			ret = xrp_writeback_alien_mapping(filp->private_data,
							  &mapping->alien_mapping,
							  flags);

		xrp_alien_mapping_destroy(&mapping->alien_mapping);
		break;

	case XRP_MAPPING_KERNEL:
		break;

	default:
		break;
	}

	if (mapping->type & XRP_MAPPING_KERNEL)
		set_fs(oldfs);

	mapping->type = XRP_MAPPING_NONE;

	return ret;
}
#endif

static long xrp_ioctl_free(struct file *filp,
			   struct xrp_ioctl_alloc __user *p)
{
	struct mm_struct *mm = current->mm;
	struct xrp_ioctl_alloc xrp_ioctl_alloc;
	struct vm_area_struct *vma;
	unsigned long start;

	printk("%s: %p\n", __func__, p);
	if (copy_from_user(&xrp_ioctl_alloc, p, sizeof(*p)))
		return -EFAULT;

	start = xrp_ioctl_alloc.addr;
	printk("%s: virt_addr = 0x%08lx\n", __func__, start);

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);

	if (vma && vma->vm_file == filp &&
	    vma->vm_start <= start && start < vma->vm_end) {
		size_t size;

		start = vma->vm_start;
		size = vma->vm_end - vma->vm_start;
		up_read(&mm->mmap_sem);
		printk("%s: 0x%lx x %zu\n", __func__, start, size);
		return vm_munmap(start, size);
	}
	printk("%s: no vma/bad vma for vaddr = 0x%08lx\n", __func__, start);
	up_read(&mm->mmap_sem);

	return -EINVAL;
}

static long xvp_complete_cmd_irq(struct xvp *xvp, struct xrp_comm *comm,
				 bool (*cmd_complete)(struct xrp_comm *p))
{
	long timeout = firmware_command_timeout * HZ;

	pr_info("xvp_complete_cmd_irq %ld\n", timeout);
	if (cmd_complete(comm))
		return 0;
	if (xrp_panic_check(xvp)) {
		pr_info("xvp_complete_cmd_irq panic 1\n");
		return -EBUSY;
	}
	do {
		timeout = wait_for_completion_interruptible_timeout(&comm->completion,
								    timeout);
		if (cmd_complete(comm))
			return 0;
		if (xrp_panic_check(xvp)) {
			pr_info("xvp_complete_cmd_irq panic 2\n");
			return -EBUSY;
		}
	} while (timeout > 0);

	if (timeout == 0) {
		pr_info("xvp_complete_cmd_irq timeout\n");
		return -EBUSY;
	}
	return timeout;
}

static long xvp_complete_cmd_poll(struct xvp *xvp, struct xrp_comm *comm,
				  bool (*cmd_complete)(struct xrp_comm *p))
{
	unsigned long deadline = jiffies + firmware_command_timeout * HZ;

	do {
		if (cmd_complete(comm))
			return 0;
		if (xrp_panic_check(xvp))
			return -EBUSY;
		schedule();
	} while (time_before(jiffies, deadline));

	return -EBUSY;
}

struct xrp_request {
	struct xrp_ioctl_queue ioctl_queue;
	size_t n_buffers;
	struct xrp_mapping *buffer_mapping;
	struct xrp_dsp_buffer *dsp_buffer;
	phys_addr_t in_data_phys;
	phys_addr_t out_data_phys;
	phys_addr_t dsp_buffer_phys;
	struct ion_buf ion_in_buf;
	struct ion_buf ion_out_buf;
	struct ion_buf *ion_dsp_pool;
	struct ion_buf *dsp_buf;
	union {
		struct xrp_mapping in_data_mapping;
		u8 in_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping out_data_mapping;
		u8 out_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping dsp_buffer_mapping;
		struct xrp_dsp_buffer buffer_data[XRP_DSP_CMD_INLINE_BUFFER_COUNT];
	};
	u8 nsid[XRP_DSP_CMD_NAMESPACE_ID_SIZE];
};

#ifdef USE_SPRD_MODE

static int sprd_unmap_request(struct file *filp, struct xrp_request *rq)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	size_t n_buffers = rq->n_buffers;
	size_t i;
	long ret = 0;
	printk("yzl add %s enter current:%p\n" , __func__, get_current());
	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		printk("yzl add %s before sprd_vdsp_iommu_unmap indata current:%p\n" , __func__, get_current());
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc,
							      &rq->ion_in_buf,
							      /*IOMMU_MSTD*/IOMMU_ALL);
		printk("yzl add %s after sprd_vdsp_iommu_unmap indata current:%p\n" , __func__, get_current());
	}
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc,
							      &rq->ion_out_buf,
							      /*IOMMU_MSTD*/IOMMU_ALL);
	} else {
		if (copy_to_user((void __user *)(unsigned long)rq->ioctl_queue.out_data_addr,
				 rq->out_data,
				 rq->ioctl_queue.out_data_size)) {
			printk("%s: out_data could not be copied\n",
			       __func__);
			ret = -EFAULT;
		}
	}

	if (n_buffers) {
		printk("yzl add %s after sprd_vdsp_iommu_unmap buffer step0 current:%p\n" , __func__, get_current());
		ret = xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, rq->dsp_buf);
		printk("yzl add %s after sprd_vdsp_iommu_unmap buffer step1 current:%p\n" , __func__, get_current());
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, rq->dsp_buf, /*IOMMU_MSTD*/IOMMU_ALL);
		printk("yzl add %s after sprd_vdsp_iommu_unmap buffer step2 current:%p\n" , __func__, get_current());
		ret = xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, rq->dsp_buf);
		printk("yzl add %s after sprd_vdsp_iommu_unmap buffer step3 current:%p\n" , __func__, get_current());
		kfree(rq->dsp_buf);
		for (i = 0; i < n_buffers; ++i) {
			ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc,
								      &rq->ion_dsp_pool[i],
								      /*IOMMU_MSTD*/IOMMU_ALL);
			if (ret < 0) {
				printk("%s: buffer %zd could not be unshared , current:%p\n",
				       __func__, i , get_current());
			}
		}
		kfree(rq->ion_dsp_pool);

		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			kfree(rq->dsp_buffer);
		}
		rq->n_buffers = 0;
	}
	printk("yzl add %s exit current:%p\n" , __func__, get_current());
	return ret;
}

static int sprd_map_request(struct file *filp, struct xrp_request *rq)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_ioctl_buffer __user *buffer;
	int n_buffers = rq->ioctl_queue.buffer_size /
		sizeof(struct xrp_ioctl_buffer);
	struct ion_buf *p_in_buf = &rq->ion_in_buf;
	struct ion_buf *p_out_buf = &rq->ion_out_buf;
	struct ion_buf *p_dsp_buf;
	int i;
	long ret = 0;

	memset((void *)&rq->ion_in_buf, 0x00, sizeof(struct ion_buf));
	memset((void *)&rq->ion_out_buf, 0x00, sizeof(struct ion_buf));

	if ((rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID) &&
	    copy_from_user(rq->nsid,
			   (void __user *)(unsigned long)rq->ioctl_queue.nsid_addr,
			   sizeof(rq->nsid))) {
		printk("%s: nsid could not be copied\n ", __func__);
		return -EINVAL;
	}

	rq->n_buffers = n_buffers;
	if (n_buffers) {
		rq->dsp_buf =
			kmalloc(sizeof(*rq->dsp_buf),
				GFP_KERNEL);
		if (!rq->dsp_buf) {
			pr_err("fail to alloc buffer.\n");
			return -ENOMEM;
		}
		xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						  rq->dsp_buf,
						  ION_HEAP_ID_MASK_SYSTEM,
						  n_buffers * sizeof(*rq->dsp_buffer));
		if (!rq->dsp_buf) {
			return -ENOMEM;
		}
		xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, rq->dsp_buf);
		rq->dsp_buf->dev = xvp->dev;
		xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, rq->dsp_buf, /*IOMMU_MSTD*/IOMMU_ALL);
		rq->dsp_buffer_phys = (phys_addr_t)rq->dsp_buf->iova[0];

		rq->ion_dsp_pool =
			kmalloc(n_buffers * sizeof(*rq->ion_dsp_pool),
				GFP_KERNEL);
		if (!rq->ion_dsp_pool) {
			return -ENOMEM;
		}
		memset((void *)rq->ion_dsp_pool,
		       0x00, n_buffers * sizeof(*rq->ion_dsp_pool));
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			rq->dsp_buffer =
				kmalloc(n_buffers * sizeof(*rq->dsp_buffer),
					GFP_KERNEL);
			if (!rq->dsp_buffer) {
				return -ENOMEM;
			}
		} else {
			rq->dsp_buffer = rq->buffer_data;
		}
	}

	//in_data addr
	if ((rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)) {
		if(rq->ioctl_queue.in_data_fd < 0) {
			pr_err("in data fd is:%d\n" , rq->ioctl_queue.in_data_fd);
			return -EFAULT;
		}
		p_in_buf->mfd[0] = rq->ioctl_queue.in_data_fd;
		p_in_buf->dev = xvp->dev;
		printk("yzl addd %s before get_ionbuf fd:%d,%d , fd1:%d\n" , __func__ , (int)p_in_buf->mfd[0] , (int)rq->ioctl_queue.in_data_fd,
		       p_in_buf->mfd[1]);
		ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(xvp->vdsp_mem_desc, p_in_buf);
		if (ret) {
			pr_err("fail to get in_ion_buf\n");
			return -EFAULT;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, p_in_buf, /*IOMMU_MSTD*/IOMMU_ALL);
		if (ret) {
			pr_err("fail to get in_data addr!!!!\n");
			return -EFAULT;
		}
		rq->in_data_phys = (uint32_t)p_in_buf->iova[0];
		printk("rq->in_data_addr=0x%x\n",(uint32_t)rq->in_data_phys);
	} else {
		if (copy_from_user(rq->in_data,
				   (void __user *)(unsigned long)rq->ioctl_queue.in_data_addr,
				   rq->ioctl_queue.in_data_size)) {
			printk("%s: in_data could not be copied\n",
			       __func__);
			return -EFAULT;
		}
	}

	//out_data addr
	if ((rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
	    &&(rq->ioctl_queue.out_data_fd >= 0)) {
		p_out_buf->mfd[0] = rq->ioctl_queue.out_data_fd;
		p_out_buf->dev = xvp->dev;
		printk("yzl addd %s before get_ionbuf output fd:%d,%d , fd1:%d\n" , __func__ , (int)p_out_buf->mfd[0] , (int)rq->ioctl_queue.out_data_fd,
		       p_out_buf->mfd[1]);
		ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(xvp->vdsp_mem_desc, p_out_buf);
		if (ret) {
			pr_err("fail to get  ion_out_buf\n");
			ret = -EFAULT;
			goto share_err;
		}
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, p_out_buf, /*IOMMU_MSTD*/IOMMU_ALL);
		if (ret) {
			pr_err("fail to get out_data addr!!!!\n");
			ret = -EFAULT;
			goto share_err;
		}
		rq->out_data_phys = (uint32_t)p_out_buf->iova[0];
		printk("out_data_phys=0x%x\n",(uint32_t)rq->out_data_phys);
	}

	//bufer addr
	if (n_buffers) {
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;

			for (i = 0; i < n_buffers; ++i) {
				struct xrp_ioctl_buffer ioctl_buffer;

				if (copy_from_user(&ioctl_buffer, buffer + i,
						   sizeof(ioctl_buffer))) {
					ret = -EFAULT;
					goto share_err;
				}
				if (ioctl_buffer.fd >= 0) {
					p_dsp_buf = &rq->ion_dsp_pool[i];
					p_dsp_buf->mfd[0] = ioctl_buffer.fd;
					p_dsp_buf->dev = xvp->dev;
					ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(xvp->vdsp_mem_desc, p_dsp_buf);
					if (ret) {
						printk("fail to get  ion_des_buf\n");
						ret = -EFAULT;
						goto share_err;
					}
					ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, p_dsp_buf, /*IOMMU_MSTD*/IOMMU_ALL);
					if (ret) {
						printk("fail to get dsp addr[%d]!!!!\n", i);
						ret = -EFAULT;
						goto share_err;
					}
					rq->dsp_buffer[i] = (struct xrp_dsp_buffer){
						.flags = ioctl_buffer.flags,
						.size = ioctl_buffer.size,
						.addr = (uint32_t)p_dsp_buf->iova[0],
						.fd = ioctl_buffer.fd,
					};
				}
			}
		}else {
			struct xrp_ioctl_buffer ioctl_buffer;
			buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;
			if (copy_from_user(&ioctl_buffer, buffer,
					   sizeof(ioctl_buffer))) {
				ret = -EFAULT;
				goto share_err;
			}

			if (ioctl_buffer.fd >= 0) {
				p_dsp_buf = rq->ion_dsp_pool;
				p_dsp_buf->mfd[0] = ioctl_buffer.fd;
				p_dsp_buf->dev = xvp->dev;
				ret = xvp->vdsp_mem_desc->ops->mem_get_ionbuf(xvp->vdsp_mem_desc, p_dsp_buf);
				if (ret) {
					printk("fail to get  ion_des_buf\n");
					ret = -EFAULT;
					goto share_err;
				}
				ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, p_dsp_buf, /*IOMMU_MSTD*/IOMMU_ALL);
				if (ret) {
					printk("fail to get dsp addr!!!!\n");
					ret = -EFAULT;
					goto share_err;
				}
				printk("yzl add %s , before one buffer vdsp addr:%x , size:%d, fd:%d,flags:%d\n",
				       __func__ , rq->dsp_buffer->addr , rq->dsp_buffer->size , rq->dsp_buffer->fd , rq->dsp_buffer->flags);
				rq->dsp_buffer->addr =  (uint32_t)p_dsp_buf->iova[0];
				rq->dsp_buffer->fd = ioctl_buffer.fd;
				rq->dsp_buffer->size = ioctl_buffer.size;
				rq->dsp_buffer->flags = ioctl_buffer.flags;
				printk("yzl add %s , one buffer vdsp addr:%x , size:%d, fd:%d,flags:%d\n",
				       __func__ , rq->dsp_buffer->addr , rq->dsp_buffer->size , rq->dsp_buffer->fd , rq->dsp_buffer->flags);
			}
		}
		memcpy((void *)rq->dsp_buf->addr_k[0],
		       (void *)rq->dsp_buffer, n_buffers * sizeof(*rq->dsp_buffer));
	}
share_err:
	if (ret < 0)
		sprd_unmap_request(filp, rq);
	return ret;
}

static void sprd_fill_hw_request(struct xrp_dsp_cmd __iomem *cmd,
				struct xrp_request *rq)
{
	xrp_comm_write32(&cmd->in_data_size, rq->ioctl_queue.in_data_size);
	xrp_comm_write32(&cmd->out_data_size, rq->ioctl_queue.out_data_size);
	xrp_comm_write32(&cmd->buffer_size,
			 rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->in_data_addr,
				 rq->in_data_phys);
	else
		xrp_comm_write(&cmd->in_data, rq->in_data,
			       rq->ioctl_queue.in_data_size);

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->out_data_addr,
				 rq->out_data_phys);

	if (rq->n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_write32(&cmd->buffer_addr,
				 rq->dsp_buffer_phys);
	else
		xrp_comm_write(&cmd->buffer_data, rq->dsp_buffer,
			       rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID)
		xrp_comm_write(&cmd->nsid, rq->nsid, sizeof(rq->nsid));

#ifdef DEBUG
	{
		struct xrp_dsp_cmd dsp_cmd;
		xrp_comm_read(cmd, &dsp_cmd, sizeof(dsp_cmd));
		printk("%s: cmd for DSP: %p: %*ph\n",
		       __func__, cmd,
		       (int)sizeof(dsp_cmd), &dsp_cmd);
	}
#endif

	wmb();
	/* update flags */
	xrp_comm_write32(&cmd->flags,
			 (rq->ioctl_queue.flags & ~XRP_DSP_CMD_FLAG_RESPONSE_VALID) |
			 XRP_DSP_CMD_FLAG_REQUEST_VALID);
}

static long sprd_complete_hw_request(struct xrp_dsp_cmd __iomem *cmd,
				     struct xrp_request *rq)
{
	u32 flags = xrp_comm_read32(&cmd->flags);

	if (rq->ioctl_queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_read(&cmd->out_data, rq->out_data,
			      rq->ioctl_queue.out_data_size);
	if (rq->n_buffers <= XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_read(&cmd->buffer_data, rq->dsp_buffer,
			      rq->n_buffers * sizeof(struct xrp_dsp_buffer));
	xrp_comm_write32(&cmd->flags, 0);

	return (flags & XRP_DSP_CMD_FLAG_RESPONSE_DELIVERY_FAIL) ? -ENXIO : 0;
}
#else
static void xrp_unmap_request_nowb(struct file *filp, struct xrp_request *rq)
{
	size_t n_buffers = rq->n_buffers;
	size_t i;

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		__xrp_unshare_block(filp, &rq->in_data_mapping, 0);
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		__xrp_unshare_block(filp, &rq->out_data_mapping, 0);
	for (i = 0; i < n_buffers; ++i)
		__xrp_unshare_block(filp, rq->buffer_mapping + i, 0);
	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		__xrp_unshare_block(filp, &rq->dsp_buffer_mapping, 0);

	if (n_buffers) {
		kfree(rq->buffer_mapping);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			kfree(rq->dsp_buffer);
		}
	}
}

static long xrp_unmap_request(struct file *filp, struct xrp_request *rq)
{
	size_t n_buffers = rq->n_buffers;
	size_t i;
	long ret = 0;
	long rc;

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		__xrp_unshare_block(filp, &rq->in_data_mapping, XRP_FLAG_READ);
	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		rc = __xrp_unshare_block(filp, &rq->out_data_mapping,
					 XRP_FLAG_WRITE);

		if (rc < 0) {
			printk("%s: out_data could not be unshared\n",
			       __func__);
			ret = rc;
		}
	} else {
		if (copy_to_user((void __user *)(unsigned long)rq->ioctl_queue.out_data_addr,
				 rq->out_data,
				 rq->ioctl_queue.out_data_size)) {
			printk("%s: out_data could not be copied\n",
				 __func__);
			ret = -EFAULT;
		}
	}

	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		__xrp_unshare_block(filp, &rq->dsp_buffer_mapping,
				    XRP_FLAG_READ_WRITE);

	for (i = 0; i < n_buffers; ++i) {
		rc = __xrp_unshare_block(filp, rq->buffer_mapping + i,
					 rq->dsp_buffer[i].flags);
		if (rc < 0) {
			printk("%s: buffer %zd could not be unshared\n",
				 __func__, i);
			ret = rc;
		}
	}

	if (n_buffers) {
		kfree(rq->buffer_mapping);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			kfree(rq->dsp_buffer);
		}
		rq->n_buffers = 0;
	}

	return ret;
}

static long xrp_map_request(struct file *filp, struct xrp_request *rq,
			    struct mm_struct *mm)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_ioctl_buffer __user *buffer;
	size_t n_buffers = rq->ioctl_queue.buffer_size /
		sizeof(struct xrp_ioctl_buffer);

	size_t i;
	long ret = 0;

	if ((rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID) &&
	    copy_from_user(rq->nsid,
			   (void __user *)(unsigned long)rq->ioctl_queue.nsid_addr,
			   sizeof(rq->nsid))) {
		printk("%s: nsid could not be copied\n ", __func__);
		return -EINVAL;
	}
	rq->n_buffers = n_buffers;
	if (n_buffers) {
		rq->buffer_mapping =
			kzalloc(n_buffers * sizeof(*rq->buffer_mapping),
				GFP_KERNEL);
		if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
			rq->dsp_buffer =
				kmalloc(n_buffers * sizeof(*rq->dsp_buffer),
					GFP_KERNEL);
			if (!rq->dsp_buffer) {
				kfree(rq->buffer_mapping);
				return -ENOMEM;
			}
		} else {
			rq->dsp_buffer = rq->buffer_data;
		}
	}

	down_read(&mm->mmap_sem);

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		ret = __xrp_share_block(filp, rq->ioctl_queue.in_data_addr,
					rq->ioctl_queue.in_data_size,
					XRP_FLAG_READ, &rq->in_data_phys,
					&rq->in_data_mapping);
		if(ret < 0) {
			printk("%s: in_data could not be shared\n",
			       __func__);
			goto share_err;
		}
	} else {
		if (copy_from_user(rq->in_data,
				   (void __user *)(unsigned long)rq->ioctl_queue.in_data_addr,
				   rq->ioctl_queue.in_data_size)) {
			printk("%s: in_data could not be copied\n",
			       __func__);
			ret = -EFAULT;
			goto share_err;
		}
	}

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
		ret = __xrp_share_block(filp, rq->ioctl_queue.out_data_addr,
					rq->ioctl_queue.out_data_size,
					XRP_FLAG_WRITE, &rq->out_data_phys,
					&rq->out_data_mapping);
		if (ret < 0) {
			printk("%s: out_data could not be shared\n",
				 __func__);
			goto share_err;
		}
	}

	buffer = (void __user *)(unsigned long)rq->ioctl_queue.buffer_addr;

	for (i = 0; i < n_buffers; ++i) {
		struct xrp_ioctl_buffer ioctl_buffer;
		phys_addr_t buffer_phys = ~0ul;

		if (copy_from_user(&ioctl_buffer, buffer + i,
				   sizeof(ioctl_buffer))) {
			ret = -EFAULT;
			goto share_err;
		}
		if (ioctl_buffer.flags & XRP_FLAG_READ_WRITE) {
			ret = __xrp_share_block(filp, ioctl_buffer.addr,
						ioctl_buffer.size,
						ioctl_buffer.flags,
						&buffer_phys,
						rq->buffer_mapping + i);
			if (ret < 0) {
				printk("%s: buffer %zd could not be shared\n",
				       __func__, i);
				goto share_err;
			}
		}

		rq->dsp_buffer[i] = (struct xrp_dsp_buffer){
			.flags = ioctl_buffer.flags,
			.size = ioctl_buffer.size,
			.addr = xrp_translate_to_dsp(&xvp->address_map,
						     buffer_phys),
		};
	}

	if (n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT) {
		ret = xrp_share_kernel(filp, (unsigned long)rq->dsp_buffer,
				       n_buffers * sizeof(*rq->dsp_buffer),
				       XRP_FLAG_READ_WRITE, &rq->dsp_buffer_phys,
				       &rq->dsp_buffer_mapping);
		if(ret < 0) {
			printk("%s: buffer descriptors could not be shared\n",
			       __func__);
			goto share_err;
		}
	}
share_err:
	up_read(&mm->mmap_sem);
	if (ret < 0)
		xrp_unmap_request_nowb(filp, rq);
	return ret;
}

static void xrp_fill_hw_request(struct xrp_dsp_cmd __iomem *cmd,
				struct xrp_request *rq,
				const struct xrp_address_map *map)
{
	xrp_comm_write32(&cmd->in_data_size, rq->ioctl_queue.in_data_size);
	xrp_comm_write32(&cmd->out_data_size, rq->ioctl_queue.out_data_size);
	xrp_comm_write32(&cmd->buffer_size,
			 rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.in_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->in_data_addr,
				 xrp_translate_to_dsp(map, rq->in_data_phys));
	else
		xrp_comm_write(&cmd->in_data, rq->in_data,
			       rq->ioctl_queue.in_data_size);

	if (rq->ioctl_queue.out_data_size > XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_write32(&cmd->out_data_addr,
				 xrp_translate_to_dsp(map, rq->out_data_phys));

	if (rq->n_buffers > XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_write32(&cmd->buffer_addr,
				 xrp_translate_to_dsp(map, rq->dsp_buffer_phys));
	else
		xrp_comm_write(&cmd->buffer_data, rq->dsp_buffer,
			       rq->n_buffers * sizeof(struct xrp_dsp_buffer));

	if (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_NSID)
		xrp_comm_write(&cmd->nsid, rq->nsid, sizeof(rq->nsid));

#ifdef DEBUG
	{
		struct xrp_dsp_cmd dsp_cmd;
		xrp_comm_read(cmd, &dsp_cmd, sizeof(dsp_cmd));
		printk("%s: cmd for DSP: %p: %*ph\n",
		       __func__, cmd,
		       (int)sizeof(dsp_cmd), &dsp_cmd);
	}
#endif

	wmb();
	/* update flags */
	xrp_comm_write32(&cmd->flags,
			 (rq->ioctl_queue.flags & ~XRP_DSP_CMD_FLAG_RESPONSE_VALID) |
			 XRP_DSP_CMD_FLAG_REQUEST_VALID);
}

static long xrp_complete_hw_request(struct xrp_dsp_cmd __iomem *cmd,
				    struct xrp_request *rq)
{
	u32 flags = xrp_comm_read32(&cmd->flags);

	if (rq->ioctl_queue.out_data_size <= XRP_DSP_CMD_INLINE_DATA_SIZE)
		xrp_comm_read(&cmd->out_data, rq->out_data,
			      rq->ioctl_queue.out_data_size);
	if (rq->n_buffers <= XRP_DSP_CMD_INLINE_BUFFER_COUNT)
		xrp_comm_read(&cmd->buffer_data, rq->dsp_buffer,
			      rq->n_buffers * sizeof(struct xrp_dsp_buffer));
	xrp_comm_write32(&cmd->flags, 0);

	return (flags & XRP_DSP_CMD_FLAG_RESPONSE_DELIVERY_FAIL) ? -ENXIO : 0;
}
#endif

#ifdef USE_LOAD_LIB
xt_ptr xt_lib_memcpy(xt_ptr dest, const void * src, unsigned int n, void *user)
{
	memcpy(user , src , n);
	return 0;
}
xt_ptr xt_lib_memset(xt_ptr s, int c, unsigned int n, void *user)
{
	memset(user , c , n);
	return 0;
}
static int32_t xrp_library_checkprocessing(struct xvp *xvp , const char *libname)
{
	int i;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(libinfo->lib_state == XRP_LIBRARY_PROCESSING_CMD) {
			if(strcmp(libinfo->libname , libname) == 0) {
				return 1;
			}
		}
	}
	return 0;
}
static struct loadlib_info *xrp_library_getlibinfo(struct xvp *xvp , const char *libname)
{
	int i;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(0 == strcmp(libinfo->libname , libname)) {
			break;
		}
	}
	/*find , and decrease*/
	if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		return libinfo;
	}
	else
		return NULL;
}

#if 0
static int xrp_load_pi_to_buffer(const char *filename , void **out)
{
	loff_t pos = 0;
	void *libstart = NULL;
	struct kstat stat;
	__u32 libsize;
	mm_segment_t fs;
	int err;
	struct file *fp = NULL;
	fs = get_fs();
	set_fs(KERNEL_DS);
	err = vfs_stat(filename , &stat);
	if(err != 0) {
		printk("yzl add %s vfs_stat failed:%d" , __func__ , err);
		goto __load_error__;
	}
	libsize = stat.size;
	fp = filp_open(filename ,O_RDONLY ,0);
	if(IS_ERR(fp)) {
		printk("yzl %s add open fp failed\n" , __func__);
		goto __load_error__;
	}
	printk("yzl add %s , libsize:%d\n" , __func__  , libsize);
	libstart = vmalloc(libsize);
	if(NULL == libstart)
	{
		printk("yzl add %s malloc lib mem failed\n" , __func__);
		filp_close(fp, NULL);
		goto __load_error__;
	}
	vfs_read(fp , libstart , libsize , &pos);
	filp_close(fp , NULL);
	set_fs(fs);
	/*yzl add load lib bin*/
	printk("yzl add %s after load lib to libstart:%p\n" ,__func__ , libstart);
	*out = libstart;
	return 0;
__load_error__:
	set_fs(fs);
	return -1;
}
#endif

#if 0
static long xrp_library_unload_internal(struct xvp *xvp , const char* libname)
{

}
#endif
static int32_t xrp_library_load_internal(struct xvp *xvp , const char* buffer , const char *libname)
{
	unsigned int size;
	int32_t ret = 0;
	struct loadlib_info *new_element;
	unsigned int result;
	struct ion_buf *lib_ion_mem = NULL;
	struct ion_buf *libinfo_ion_mem = NULL;
	void *libback_buffer = NULL;
	void *kvaddr=NULL;
	void *kvaddr_libinfo = NULL;
	phys_addr_t kpaddr , kpaddr_libinfo;
	//sprintf(totallibname , "/vendor/firmware/%s.bin" , libname);
	/*load library to ddr*/
	size = xtlib_pi_library_size((xtlib_packaged_library *)buffer);
	/*alloc ion buffer later*/
	lib_ion_mem = vmalloc(sizeof(struct ion_buf));
	libinfo_ion_mem = vmalloc(sizeof(struct ion_buf));
	if((NULL == lib_ion_mem) || (NULL == libinfo_ion_mem)) {
		printk("yzl add %s vmalloc lib_ion_mem failed libionmem:%p , libinfoionmem:%p\n" , __func__ , lib_ion_mem , libinfo_ion_mem);
		if(NULL != lib_ion_mem)
			vfree(lib_ion_mem);
		if(NULL != libinfo_ion_mem)
			vfree(libinfo_ion_mem);
		return -ENOMEM;
	}
	printk("yzl add %s library size:%d\n" , __func__ , size);
	/*alloc lib ion buffer*/
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						lib_ion_mem,
						ION_HEAP_ID_MASK_SYSTEM,
						size);
	if(ret != 0) {
		printk("yzl add %s alloc lib_ion_mem failed\n" , __func__);
		vfree(lib_ion_mem);
		vfree(libinfo_ion_mem);
		return -ENOMEM;
	}
	lib_ion_mem->dev = xvp->dev;
	/*alloc libinfo ion buffer*/
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						libinfo_ion_mem,
						ION_HEAP_ID_MASK_SYSTEM,
						sizeof(xtlib_pil_info));
	if(ret != 0) {
		printk("yzl add %s alloc libinfo_ion_mem failed\n" , __func__);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
		vfree(lib_ion_mem);
		vfree(libinfo_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
		return -ENOMEM;
	}
	libinfo_ion_mem->dev = xvp->dev;
	xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, lib_ion_mem);
	kvaddr = (void*)lib_ion_mem->addr_k[0];
	xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, lib_ion_mem , IOMMU_ALL);
	kpaddr = lib_ion_mem->iova[0];

	xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
	kvaddr_libinfo = (void*)libinfo_ion_mem->addr_k[0];
	xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, libinfo_ion_mem , /*IOMMU_MSTD*/IOMMU_ALL);
	kpaddr_libinfo = libinfo_ion_mem->iova[0];
	printk("yzl add %s , load pi library buffer:%p , kpaddr:%lx, kvaddr:%p , kpaddr_libinfo:%lx , kvaddr_libinfo:%p\n",
	       __func__ , buffer , (unsigned long) kpaddr , kvaddr , (unsigned long) kpaddr_libinfo , kvaddr_libinfo);
	result = xtlib_host_load_pi_library((xtlib_packaged_library*)buffer , kpaddr , (xtlib_pil_info*)kvaddr_libinfo , xt_lib_memcpy , xt_lib_memset , kvaddr);
	if(result == 0)
	{
		/*free ion buffer*/
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_ion_mem, IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo_ion_mem,/*IOMMU_MSTD*/IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo_ion_mem);
		vfree(lib_ion_mem);
		vfree(libinfo_ion_mem);
		/**/
		printk("yzl add %s xtlib_host_load_pi_library failed %d\n" , __func__ , result);
		return -1;
	}
	libback_buffer = vmalloc(size);
	if(libback_buffer == NULL) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_ion_mem, IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo_ion_mem,/*IOMMU_MSTD*/IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo_ion_mem);
		vfree(lib_ion_mem);
		vfree(libinfo_ion_mem);
		printk("yzl add %s vmalloc back buffer is NULL\n" , __func__);
		return -1;
	}
	/*back up the code for reboot*/
	memcpy(libback_buffer , kvaddr , size);
	new_element = (struct loadlib_info*)libinfo_alloc_element();
	if(new_element == NULL) {
		/*free ion buffer*/
		printk("yzl add %s libinfo_alloc_element failed\n" , __func__);
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, lib_ion_mem, IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo_ion_mem,/*IOMMU_MSTD*/IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, lib_ion_mem);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo_ion_mem);
		vfree(lib_ion_mem);
		vfree(libinfo_ion_mem);
		vfree(libback_buffer);
		return -1;
	} else {
		sprintf(new_element->libname , "%s" , libname);
		/*may be change later*/
		new_element->address_start = 0;
		new_element->length = size;
		new_element->load_count = 0;
		new_element->ionhandle = lib_ion_mem;
		new_element->ion_phy = (phys_addr_t)kpaddr;
		new_element->ion_kaddr = kvaddr;
		new_element->pil_ionhandle = libinfo_ion_mem;
		new_element->pil_info = kpaddr_libinfo;
		new_element->pil_info_kaddr = kvaddr_libinfo;
		new_element->lib_state = XRP_LIBRARY_LOADING;
		new_element->code_back_kaddr = libback_buffer;
	}
	libinfo_list_add(&xvp->load_lib.lib_list , new_element , libinfo_list_size(&xvp->load_lib.lib_list));
	return 0;
}
static enum load_unload_flag xrp_check_load_unload(struct xvp *xvp , struct xrp_request *rq)
{
	__u32 indata_size;
	enum load_unload_flag load_flag = 0;
	__u8 *tempbuffer = NULL;
	void *tempsrc = NULL;
	indata_size = rq->ioctl_queue.in_data_size;
	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID)) {
		if(indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		} else {
			tempsrc = rq->in_data;
		}
		tempbuffer = vmalloc(indata_size);
		printk("yzl add %s before copy_from_user src:%p , dst:%p\n" , __func__ , tempsrc , tempbuffer);
		if(copy_from_user(tempbuffer , tempsrc , indata_size)) {
			vfree(tempbuffer);
			return -EFAULT;
		}
		load_flag = *tempbuffer;
		vfree(tempbuffer);
		printk("yzl add %s , load flag:%d\n" , __func__ , load_flag);
		return load_flag;
	} else
		return XRP_NOT_LOAD_UNLOAD;
}
#ifndef USE_RE_REGISTER_LIB
static int32_t xrp_library_setall_missstate(struct xvp *xvp)
{
	int i;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		libinfo->lib_state = XRP_LIBRARY_MISSED_STATE;
	}
	return 0;
}
#endif
static int32_t xrp_library_decrease(struct xvp *xvp , const char *libname)
{
	int i;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(0 == strcmp(libinfo->libname , libname)) {
			break;
		}
	}
	/*find , and decrease*/
	if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		libinfo->load_count--;
	} else
		return -EINVAL;
	return libinfo->load_count;

}
static int32_t xrp_library_increase(struct xvp *xvp , const char *libname)
{
	int i;
        struct loadlib_info *libinfo = NULL;
        for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
                libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
                if(0 == strcmp(libinfo->libname , libname)) {
                        break;
                }
        }
        /*find , and decrease*/
        if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
                libinfo->load_count++;
        } else
                return -EINVAL;
        return libinfo->load_count;
}
#if 0
static int xrp_check_library_loaded(struct xvp *xvp , const char *libname)
{
	int i;
	int ret = 0;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(0 == strcmp(libinfo->libname , libname))
			break;
	}
	/**/
	if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		libinfo->load_count++;
		ret = 0; /*loaded*/
	}
	else
		ret = 1;
	return ret;
	
}
#endif
static int32_t xrp_library_decrelease(struct xvp *xvp , const char *libname)
{
        int i;
        int ret;
        struct loadlib_info *libinfo = NULL;

        /*decrease load_count*/
        for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
                libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
                if(0 == strcmp(libinfo->libname , libname))
                        break;
        }
        /**/
        if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		if(libinfo->load_count != 0)
                	libinfo->load_count--;
                printk("yzl add %s load_count:%d\n" , __func__ , libinfo->load_count);
#if 0
		/*unmap iommu*/
		/*free ion buffer later*/
		dma_free_attrs(xvp->dev , libinfo->length , libinfo->ionhandle , phys_to_dma(xvp->dev, libinfo->ion_phy),DMA_ATTR_NO_KERNEL_MAPPING);
		dma_free_attrs(xvp->dev ,sizeof(xtlib_pil_info) , libinfo->pil_ionhandle , phys_to_dma(xvp->dev , libinfo->pil_info) , DMA_ATTR_NO_KERNEL_MAPPING);
		/*remove this lib element*/
                libinfo_list_remove(&(xvp->load_lib.lib_list) , i);
#else
		if(libinfo->load_count == 0) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->ionhandle);
		xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->ionhandle , IOMMU_ALL);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->ionhandle);
		vfree(libinfo->ionhandle);

		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
        xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle, /*IOMMU_MSTD*/IOMMU_ALL);
        xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
        vfree(libinfo->pil_ionhandle);
		if(libinfo->code_back_kaddr != NULL)
			vfree(libinfo->code_back_kaddr);
		/*remove this lib element*/
                libinfo_list_remove(&(xvp->load_lib.lib_list) , i);
		} else {
			printk("yzl add %s release failed loadcount:%d , libname:%s\n" , __func__ , libinfo->load_count , libname);
		}
#endif
                ret = 0; /*loaded*/
        } else {
                printk("yzl add %s not find lib:%s , may be some error\n" , __func__ , libname);
                ret = 1;
        }
        return ret;
}
static int32_t xrp_library_getloadunload_libname(struct xvp *xvp , struct xrp_request *rq , char *outlibname)
{
	__u32 indata_size;
	int32_t ret = 0;
	void *tempsrc = NULL;
	__u8 *tempbuffer = NULL;
	indata_size = rq->ioctl_queue.in_data_size;

	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID)) {
		/*check libname*/
		if(indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		} else {
			tempsrc = (void*)(rq->in_data);
		}
		tempbuffer = vmalloc(indata_size);
		printk("yzl add %s before copy_from_user src:%p , dst:%p\n" , __func__ , tempsrc , tempbuffer);
		if(copy_from_user(tempbuffer , tempsrc , indata_size)) {
			printk("yzl add %s , copy from user failed\n" , __func__);
			ret = -EINVAL;
		}
		/*input_vir first byte is load or unload*/
		sprintf(outlibname , "%s" , tempbuffer+1);
		vfree(tempbuffer);
	} else {
		ret = -EINVAL;
	}
	return ret;
}
static int32_t xrp_library_kmap_ionbuf(struct ion_buf *ionbuf , __u8 **buffer , struct dma_buf **dmabuf)
{
	struct dma_buf *dma_buf;
	int32_t mapcount = 0;
	*buffer = NULL;
	if((ionbuf == NULL) || (ionbuf->mfd[0] < 0)) {
		printk("yzl add %s , ion dsp pool is NULL return error\n" , __func__);
		return -EINVAL;
	}
	dma_buf = dma_buf_get(ionbuf->mfd[0]);
	if(IS_ERR_OR_NULL(dma_buf)) {
		printk("yzl add %s , dma_buf_get fd:%d\n" , __func__ , ionbuf->mfd[0]);
		return -EINVAL;
	}
	/*workaround need to process later*/
	while((*buffer == NULL) && (mapcount < 5)) {
		*buffer =sprd_ion_map_kernel(dma_buf, 0);
		mapcount++;
	}
	*dmabuf = dma_buf;
	/*buffer index 0 is input lib buffer*/
	printk("yzl add %s , mfd:%d , dev:%p , map kernel vaddr is:%p , mapcount:%d\n" , __func__ , ionbuf->mfd[0] , ionbuf->dev , *buffer  , mapcount);
	if(NULL == *buffer)
		return -EFAULT;
	return 0;
}
static int32_t xrp_library_kunmap_ionbuf(struct dma_buf *dmabuf)
{
	sprd_ion_unmap_kernel(dmabuf , 0);
	dma_buf_put(dmabuf);
	return 0;
}
static int32_t xrp_library_unload(struct xvp *xvp , struct xrp_request *rq , char *libname)
{
	int ret = 0;
	__u32 indata_size;
	__u8 *inputbuffer = NULL;
	struct dma_buf* dmabuf = NULL;
	struct loadlib_info *libinfo = NULL;
	indata_size = rq->ioctl_queue.in_data_size;
	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID) && (indata_size >= LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE)) {
		libinfo = xrp_library_getlibinfo(xvp , libname);
		if(libinfo != NULL) {
			ret = xrp_library_kmap_ionbuf(&rq->ion_in_buf,&inputbuffer ,&dmabuf);
			if(ret != 0) {
				printk("yzl add %s xrp_library_kmap_ionbuf failed ret:%d\n" , __func__, ret);
				return -EFAULT;
			}
			*((unsigned int*)((__u8 *)inputbuffer + 40)) = libinfo->pil_info;
			printk("yzl add %s  p:%p , p+40 value:%x , pil_info:%x\n" , __func__ , inputbuffer , *((unsigned int*)((__u8 *)inputbuffer + 40)) ,libinfo->pil_info);
			xrp_library_kunmap_ionbuf(dmabuf);
			wmb();
		} else {
			printk("yzl add %s , xrp_library_getlibinfo failed\n" , __func__);
			ret = -EINVAL;
		}
	} else {
		printk("yzl add %s nisid is not unload\n" , __func__);
		ret = -EINVAL;
	}
	return ret;
}
/* return value 0 is need load, 1 is loaded already*/
static int32_t xrp_library_load(struct xvp *xvp , struct xrp_request *rq , char *outlibname)
{
	__u32 indata_size;
	__u8 load_flag = 0;
	int32_t ret = 0;
	int i;
	void *tempsrc = NULL;
	__u8 *tempbuffer = NULL;
	struct loadlib_info *libinfo = NULL;
	__u8 *input_ptr = NULL;
	__u8 *libbuffer = NULL;
	__u8 *inputbuffer = NULL;
	char libname[64];
	struct dma_buf* dmabuf = NULL;
	indata_size = rq->ioctl_queue.in_data_size;
	/*check whether loaded*/

	/*check whether load cmd*/
	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID)) {
		/*check libname*/
		if(indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
			tempsrc = (void*)(rq->ioctl_queue.in_data_addr);
		} else {
			tempsrc = (void*)(rq->in_data);
		}
		tempbuffer = vmalloc(indata_size);
		printk("yzl add %s before copy_from_user src:%p , dst:%p\n" , __func__ , tempsrc , tempbuffer);
		if(copy_from_user(tempbuffer , tempsrc , indata_size)) {
			printk("yzl add %s , copy from user failed\n" , __func__);
			vfree(tempbuffer);
			return -EFAULT;
		}
		input_ptr = tempbuffer;
		/*input_vir first byte is load or unload*/
		load_flag = *input_ptr;
		printk("yzl add %s , load_flag:%d\n" , __func__ , load_flag);
		if(XRP_LOAD_LIB_FLAG == load_flag) {
			/*load*/
			sprintf(libname , "%s" , input_ptr+1);
			/*check whether loaded*/
			for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
				libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
				if(0 == strcmp(libinfo->libname , libname))
					break;
			}
			/**/
			if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
				libinfo->load_count++;
				printk("yzl add %s , lib:%s loaded not need reload\n" , __func__ , libname);
				ret = 1; /*loaded*/
			} else {
				/*not loaded alloc libinfo node ,load internal*/
				ret = xrp_library_kmap_ionbuf(rq->ion_dsp_pool , &libbuffer , &dmabuf);
				if(ret != 0) {
					printk("yzl add %s xrp_library_parase_libbuffer failed\n" , __func__);
					vfree(tempbuffer);
					return -EINVAL;
				}
				ret = xrp_library_load_internal(xvp , libbuffer , libname);
				if(ret != 0) {
					printk("yzl add %s xrp_library_load_internal ret:%d\n" , __func__ ,ret);
					xrp_library_kunmap_ionbuf(dmabuf);
					vfree(tempbuffer);
					ret = -ENOMEM;
					return ret;
				}
				xrp_library_kunmap_ionbuf(dmabuf);
				/*re edit rq for register libname , input data: input[0] load unload flag
				  input[1] ~input[32] --- libname , input[LIBRARY_CMD_PIL_INFO_OFFSET]~input[43] ---- libinfo addr*/
				libinfo = xrp_library_getlibinfo(xvp , libname);
				if(NULL == libinfo) {
					printk("yzl add %s xrp_library_getlibinfo NULL\n" , __func__);
					xrp_library_decrelease(xvp , libname);
					vfree(tempbuffer);
					ret = -ENOMEM;
					return ret;
				} else {
					*((uint32_t*)(input_ptr+LIBRARY_CMD_PIL_INFO_OFFSET)) = libinfo->pil_info;
					printk("yzl add %s load input param nsid:%s , loadflag:%d , libname:%s , pil_info:%x , indata_size:%d\n" , __func__ ,rq->nsid , 
					       load_flag , libname , libinfo->pil_info , indata_size);
					sprintf(outlibname , "%s" , libname);
				}
				ret = xrp_library_kmap_ionbuf(&rq->ion_in_buf,&inputbuffer ,&dmabuf);
				if(ret != 0) {
					printk("yzl add %s xrp_library_kmap_ionbuf input buffer failed\n" , __func__);
					vfree(tempbuffer);
					xrp_library_decrelease(xvp , libname);
					return -EFAULT;
				}
				memcpy(inputbuffer , tempbuffer , indata_size);
				xrp_library_kunmap_ionbuf(dmabuf);
				wmb();
			}
		} else {
			printk("yzl add %s not load flag\n" , __func__);
			ret = -EINVAL;
		}
#if 0
		if (pfn_valid(__phys_to_pfn(rq->in_data_phys))) {
			struct page *page = pfn_to_page(__phys_to_pfn(rq->in_data_phys));
			void *p = kmap(page);
			if(p){
				memcpy(p , tempbuffer , indata_size);
				printk("yzl add %s after memcpy p:%p\n" , __func__ , p);
				kunmap(page);
			}
		}
		dma_sync_single_for_device(xvp->dev, rq->in_data_phys , indata_size , DMA_TO_DEVICE);

#endif
		vfree(tempbuffer);
		return ret;
	} else
		return 0;
}
#if 1
static int32_t xrp_library_release_all(struct xvp *xvp)
{
	int i;
	int ret = 0;
	struct loadlib_info *libinfo = NULL;
	mutex_lock(&xvp->load_lib.libload_mutex);
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		/*unmap iommu and release ion buffer*/
#if 0
		dma_free_attrs(xvp->dev, PAGE_SIZE, libinfo->ionhandle,
			       phys_to_dma(xvp->dev, libinfo->ion_phy), 0);
		dma_free_attrs(xvp->dev, PAGE_SIZE, libinfo->pil_ionhandle , 
			       phys_to_dma(xvp->dev, libinfo->pil_info) , 0);

#else
		if((libinfo != NULL) /*&& (libinfo->load_count <=0)*/) {
			xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->ionhandle);
			xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->ionhandle , IOMMU_ALL);
			xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->ionhandle);
			vfree(libinfo->ionhandle);

			xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
			xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, libinfo->pil_ionhandle , /*IOMMU_MSTD*/IOMMU_ALL);
			xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, libinfo->pil_ionhandle);
			vfree(libinfo->pil_ionhandle);
			if(libinfo->code_back_kaddr != NULL)
				vfree(libinfo->code_back_kaddr);
			libinfo_list_remove(&(xvp->load_lib.lib_list) , i);
			printk("yzl add %s , release ion handle, pil handle current:%p\n" , __func__ , get_current());
		}

#endif
	}
	mutex_unlock(&xvp->load_lib.libload_mutex);
#if 0
	if(xvp->load_lib.lib_list.number != 0) {
		ret = -EINVAL;
	}
#endif
	return ret;
}
#endif
#ifdef USE_RE_REGISTER_LIB
static int32_t xrp_register_libs(struct xvp *xvp , struct xrp_comm *comm)
{
	int i;
	int ret =0;
	int realret = 0;
	__u8 *input_ptr = NULL;
	struct loadlib_info *libinfo = NULL;
	struct ion_buf input_ion_buf;
	phys_addr_t vdsp_addr;
	struct xrp_request rq;
	rq.ioctl_queue.flags = XRP_QUEUE_FLAG_NSID;
	rq.ioctl_queue.in_data_size = LIBRARY_CMD_LOAD_UNLOAD_INPUTSIZE;
	rq.ioctl_queue.out_data_size = 0;
	rq.ioctl_queue.buffer_size = 0;
	rq.ioctl_queue.in_data_addr = 0;
	rq.ioctl_queue.out_data_addr = 0;
	rq.ioctl_queue.buffer_addr = 0;
	rq.ioctl_queue.nsid_addr = 0;
	rq.n_buffers = 0;
	rq.buffer_mapping = NULL;
	rq.dsp_buffer = NULL;
	/*may change later*/
	rq.in_data_phys = 0;
	/*alloc input cmd ion buffer*/
	ret = sprd_vdsp_ion_alloc(&input_ion_buf , ION_HEAP_ID_MASK_SYSTEM , rq.ioctl_queue.in_data_size);
        if(ret != 0) {
                printk("yzl add %s alloc input_ion_buf failed\n" , __func__);
                return -ENOMEM;
        }
        input_ion_buf.dev = xvp->dev;
	sprd_vdsp_kmap(&input_ion_buf);
        input_ptr = (void*)input_ion_buf.addr_k[0];
        sprd_vdsp_iommu_map(&input_ion_buf , IOMMU_ALL);
        vdsp_addr = input_ion_buf.iova[0];
	rq.in_data_phys = vdsp_addr;
	memset(rq.nsid , 0 , sizeof(rq.nsid));
	sprintf(rq.nsid , "%s" , LIBRARY_LOAD_UNLOAD_NSID);
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(libinfo->load_count > 0) {
			/*re send register cmd*/
			if(libinfo->code_back_kaddr == NULL) {
				realret = -1;
				break;
			}
			memcpy(libinfo->code_back_kaddr , libinfo->ion_kaddr , libinfo->length);
			input_ptr[0] = XRP_LOAD_LIB_FLAG;
			sprintf(input_ptr+1 , "%s" , libinfo->libname);
			*((unsigned int*)(input_ptr + LIBRARY_CMD_PIL_INFO_OFFSET)) = libinfo->pil_info;
			sprd_fill_hw_request(comm->comm , &rq);
			xrp_send_device_irq(xvp);
			if(xvp->host_irq_mode) {
				ret = xvp_complete_cmd_irq(xvp , comm , xrp_cmd_complete);
			} else {
				ret = xvp_complete_cmd_poll(xvp, comm ,
					xrp_cmd_complete);
			}
			xrp_panic_check(xvp);
			if(0 == ret) {
				ret = sprd_complete_hw_request(comm->comm , &rq);
			}
			if(ret != 0) {
				/*set invalid*/
				printk("yzl add %s re load lib:%s failed\n" , __func__ , libinfo->libname);
				libinfo->lib_state = XRP_LIBRARY_IDLE;
				realret = -1;
			}
			else {
				libinfo->lib_state = XRP_LIBRARY_LOADED;
				printk("yzl add %s re load lib:%s ok\n" , __func__ ,libinfo->libname);
			}
		}
        }
	sprd_vdsp_iommu_unmap(&input_ion_buf , IOMMU_ALL);
	sprd_vdsp_kunmap(&input_ion_buf);
	sprd_vdsp_ion_free(&input_ion_buf);
	return realret;
}
#endif
#endif

#ifdef USE_LOAD_LIB
/*return value 0 is ok, other value is fail or no need process continue*/
static int32_t xrp_pre_process_request(struct xvp *xvp , struct xrp_request *rq , enum load_unload_flag loadflag, char *libname)
{
	int32_t lib_result , load_count;
	//char libname[32];
	struct loadlib_info *libinfo = NULL;
	if(loadflag == XRP_LOAD_LIB_FLAG) {
		lib_result = xrp_library_load(xvp , rq , libname);
		if(0 != lib_result) {
			/*has loaded needn't reload*/
			if(lib_result != 1) {
				printk("yzl add %s , result:%d, current:%p\n" , __func__ , lib_result , get_current());
				return -EFAULT;
			} else {
				printk("yzl add %s , already loaded needn't reload current:%p\n" , __func__ , get_current());
				return -ENXIO;
			}
		} else {
			/*re-edit the rq for register*/
			printk("yzl add %s , loadflag:%d current:%p\n" , __func__ , loadflag , get_current());
			return 0;
		}
	} else if(loadflag == XRP_UNLOAD_LIB_FLAG) {
		lib_result = xrp_library_getloadunload_libname(xvp , rq , libname);
		if(lib_result == 0) {
			if(0 != xrp_library_checkprocessing(xvp , libname)) {
				printk("yzl add %s xrp_library_checkprocessing the same lib is processing invalid operation\n" , __func__);
				return -EINVAL;
			}
			load_count = xrp_library_decrease(xvp  , libname);
			libinfo = xrp_library_getlibinfo(xvp , libname);
			/*if not need unload , maybe count is not zero , return after unmap*/
			if(load_count > 0) {
				printk("yzl add %s , needn't unload because load count is not zero\n" , __func__);
				return -ENXIO;
			} else if(0 == load_count){
#ifndef USE_RE_REGISTER_LIB
				if(libinfo != NULL) {
					if(libinfo->lib_state == XRP_LIBRARY_MISSED_STATE) {
						printk("yzl add %s xrp_library_unload libname:%s is missed state, release here\n" , __func__ , libname);
						xrp_library_decrelease(xvp , libname);
						return -ENXIO;
					}
				}
#endif
				/*if need unload may be modify libinfo addr, only follow the default send cmd*/
				lib_result = xrp_library_unload(xvp , rq , libname);
				if(lib_result != 0) {
					printk("yzl add %s xrp_library_unload failed:%d\n" , __func__ , lib_result);
					return -EINVAL;
				}
			} else {
				printk("yzl add %s error decrease count error:%d , libname:%s\n" , __func__ , load_count , libname);
				return -ENXIO;
			}
			if(libinfo != NULL)
				libinfo->lib_state = XRP_LIBRARY_UNLOADING;
			printk("yzl add %s loadflag:%d , libname:%s , current:%p\n" , __func__ , loadflag , libname , get_current());
			return 0;
		} else {
			printk("yzl add %s , XRP_UNLOAD_LIB_FLAG , get libname error , libname:%s , loadflag:%d ,current:%p\n" , __func__ ,
				libname , loadflag , get_current());
			return -EINVAL;
		}
	}
	else {
		/*check whether libname unloading state, if unloading return*/
		libinfo = xrp_library_getlibinfo(xvp , rq->nsid);
		printk("yzl add %s xrp_library_getlibinfo libinfo:%p\n" , __func__ , libinfo);
		if(libinfo != NULL) {
			if(libinfo->lib_state != XRP_LIBRARY_LOADED) {
				printk("yzl add %s lib:%s , libstate is:%d not XRP_LIBRARY_LOADED , so return inval\n" , __func__,rq->nsid , libinfo->lib_state);
				return -EINVAL;
			}
			/*set processing libname*/
			libinfo->lib_state = XRP_LIBRARY_PROCESSING_CMD;
		} else {
			printk("yzl add %s not loaded libinfo\n" , __func__);
		}
		printk("yzl add %s not loaded libinfo , libname:%s , current:%p\n" , __func__ , rq->nsid , get_current());
		return 0;
	}
}
static int post_process_request(struct xvp *xvp , struct xrp_request *rq , const char* libname , enum load_unload_flag load_flag , int32_t resultflag)
{
	struct loadlib_info *libinfo = NULL;
	int32_t ret = 0;
	if(load_flag == XRP_LOAD_LIB_FLAG) {
		if(0 == resultflag){
			xrp_library_increase(xvp , libname);
			libinfo = xrp_library_getlibinfo(xvp , libname);
			if(NULL != libinfo) {
				libinfo->lib_state = XRP_LIBRARY_LOADED;
				printk("yzl add %s , set lib:%s , libstate XRP_LIBRARY_LOADED\n" , __func__ , libname);
			}
		} else {
			/*load failedd release here*/
			xrp_library_decrelease(xvp , libname);
			printk("yzl add %s , set lib:%s , load failed xrp_library_decrelease\n" , __func__ , libname);
			ret = -EFAULT;
		}
	} else if(load_flag == XRP_UNLOAD_LIB_FLAG) {
		if(0 == resultflag) {
			libinfo = xrp_library_getlibinfo(xvp , libname);
			if(NULL != libinfo) {
				libinfo->lib_state = XRP_LIBRARY_IDLE;
			}
			printk("yzl add %s , set lib:%s , libstate XRP_LIBRARY_IDLE libinfo:%p\n" , __func__ , libname , libinfo);
		} else {
			printk("yzl add %s , set lib:%s , unload failed\n" , __func__ , libname);
			ret = -EFAULT;
		}
	} else {
		/*remove processing lib*/
		libinfo = xrp_library_getlibinfo(xvp , rq->nsid);
		printk("yzl add %s xrp_library_getlibinfo libinfo:%p\n" , __func__ , libinfo);
		if(libinfo != NULL) {
			if(libinfo->lib_state != XRP_LIBRARY_PROCESSING_CMD) {
				printk("yzl add %s lib:%s processing cmd , but not XRP_LIBRARY_PROCESSING_CMD state\n",
					__func__ , rq->nsid);
				ret = -EINVAL;
			}
			/*set processing libname*/
			libinfo->lib_state = XRP_LIBRARY_LOADED;
		} else {
			;
		}
		/*set processing lib state*/
		printk("yzl add %s lib:%s , process cmd over\n" , __func__ , rq->nsid);
	}
	return ret;
}
#endif

static long xrp_ioctl_submit_sync(struct file *filp,
				  struct xrp_ioctl_queue __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_comm *queue = xvp->queue;
	struct xrp_request xrp_rq, *rq = &xrp_rq;
	long ret = 0;
	bool went_off = false;
#ifdef USE_LOAD_LIB
	enum load_unload_flag load_flag;
	char libname[32];
	bool rebootflag = 0;
	/*int32_t load_count;*/
	//struct loadlib_info *libinfo = NULL;
	int32_t lib_result = 0;
#endif
	if (copy_from_user(&rq->ioctl_queue, p, sizeof(*p)))
		return -EFAULT;

	if (rq->ioctl_queue.flags & ~XRP_QUEUE_VALID_FLAGS) {
		dev_dbg(xvp->dev, "%s: invalid flags 0x%08x\n",
			__func__, rq->ioctl_queue.flags);
		return -EINVAL;
	}
	
	if (xvp->n_queues > 1) {
		unsigned n = (rq->ioctl_queue.flags & XRP_QUEUE_FLAG_PRIO) >>
			XRP_QUEUE_FLAG_PRIO_SHIFT;

		if (n >= xvp->n_queues)
			n = xvp->n_queues - 1;
		queue = xvp->queue_ordered[n];
		dev_dbg(xvp->dev, "%s: priority: %d -> %d\n",
			__func__, n, queue->priority);
		printk("yzl add %s , queue index:%d\n" , __func__ , n);
	}

#ifdef USE_SPRD_MODE
 ret = sprd_map_request(filp, rq);
#else
	ret = xrp_map_request(filp, rq, current->mm);
#endif
	if (ret < 0)
		return ret;

	if (loopback < LOOPBACK_NOIO) {
		int reboot_cycle;
retry:
		mutex_lock(&queue->lock);
		reboot_cycle = atomic_read(&xvp->reboot_cycle);
		if (reboot_cycle != atomic_read(&xvp->reboot_cycle_complete)) {
			mutex_unlock(&queue->lock);
			goto retry;
		}

		if (xvp->off) {
			ret = -ENODEV;
		} else {
			printk("yzl add %s , queue_comm:%p , current:%p\n" , __func__ , queue->comm , get_current());
#ifdef USE_SPRD_MODE
#ifdef USE_LOAD_LIB
			/*check whether libload command and if it is, do load*/
			load_flag = xrp_check_load_unload(xvp , rq);
#if 0
			if(load_flag == XRP_LOAD_LIB_FLAG) {
				mutex_lock(&(xvp->load_lib.libload_mutex));
				lib_result = xrp_library_load(xvp , rq , libname);
				if(0 != lib_result) {
					/*has loaded needn't reload*/
					mutex_unlock(&queue->lock);
					mutex_unlock(&(xvp->load_lib.libload_mutex));
					ret = sprd_unmap_request(filp, rq);
					if(lib_result != 1) {
						printk("yzl add %s , xrp_library_load result:%d\n" , __func__ , lib_result);
						return lib_result;
					} else {
						printk("yzl add %s , xrp_library_load already loaded needn't reload\n" , __func__);
						return ret;
					}
				} else {
					/*re-edit the rq for register*/
				}
			} else if(load_flag == XRP_UNLOAD_LIB_FLAG) {
				mutex_lock(&(xvp->load_lib.libload_mutex));
				/*check whether processing libname is the same with this unload one, if same
                                        return*/
				lib_result = xrp_library_getloadunload_libname(xvp , rq , libname);
				if(lib_result == 0) {
					if(0 != xrp_library_checkprocessing(xvp , libname)) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
						ret = sprd_unmap_request(filp, rq);
						printk("yzl add %s xrp_library_checkprocessing the same lib is processing invalid operation\n" , __func__);
						return -EINVAL;
					}
				
					load_count = xrp_library_decrease(xvp  , libname);
					/*if not need unload , maybe count is not zero , return after unmap*/
					if(load_count > 0) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
                				ret = sprd_unmap_request(filp, rq);
						printk("yzl add %s , needn't unload because load count is not zero\n" , __func__);
						return -EINVAL;
					} else if(0 == load_count){
						/*if need unload may be modify libinfo addr, only follow the default send cmd*/
						lib_result = xrp_library_unload(xvp , rq , libname);
						if(lib_result != 0) {
							mutex_unlock(&queue->lock);
							mutex_unlock(&(xvp->load_lib.libload_mutex));
							ret = sprd_unmap_request(filp, rq);
							printk("yzl add %s xrp_library_unload failed:%d\n" , __func__ , lib_result);
							return -EINVAL;
						}
					} else
						printk("yzl add %s error decrease count error:%d , libname:%s" , __func__ , load_count , libname);
					libinfo = xrp_library_getlibinfo(xvp , libname);
					libinfo->lib_state = XRP_LIBRARY_UNLOADING;
				} else {
					mutex_unlock(&queue->lock);
					mutex_unlock(&(xvp->load_lib.libload_mutex));
					ret = sprd_unmap_request(filp, rq);
					printk("yzl add %s , XRP_UNLOAD_LIB_FLAG , get libname error\n" , __func__);
				}
			} else {
				mutex_lock(&(xvp->load_lib.libload_mutex));
				/*check whether libname unloading state, if unloading return*/
				libinfo = xrp_library_getlibinfo(xvp , rq->nsid);
				printk("yzl add %s xrp_library_getlibinfo libinfo:%p\n" , __func__ , libinfo);
				if(libinfo != NULL) {
					if(libinfo->lib_state != XRP_LIBRARY_LOADED) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
						ret = sprd_unmap_request(filp , rq);
						printk("yzl add %s lib:%s , libstate is:%d not XRP_LIBRARY_LOADED , so return inval\n" , __func__,
							rq->nsid , libinfo->lib_state);
						return -EINVAL;
					}
					/*set processing libname*/
					libinfo->lib_state = XRP_LIBRARY_PROCESSING_CMD;
				} else {
					;
				}
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			}
#else
			mutex_lock(&(xvp->load_lib.libload_mutex));
			lib_result = xrp_pre_process_request(xvp , rq , load_flag , libname);
			if(lib_result != 0) {
				mutex_unlock(&queue->lock);
				mutex_unlock(&(xvp->load_lib.libload_mutex));
				ret = sprd_unmap_request(filp , rq);
				if(lib_result == -ENXIO) {
					return 0;
				} else {
					return -EFAULT;
				}
			}else if((load_flag != XRP_UNLOAD_LIB_FLAG)&&(load_flag != XRP_LOAD_LIB_FLAG)) {
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			}
				
#endif
#endif
			sprd_fill_hw_request(queue->comm, rq);
#else
			xrp_fill_hw_request(queue->comm, rq, &xvp->address_map);
#endif
			xrp_send_device_irq(xvp);

			if (xvp->host_irq_mode) {
				ret = xvp_complete_cmd_irq(xvp, queue,
							   xrp_cmd_complete);
			} else {
				ret = xvp_complete_cmd_poll(xvp, queue,
							    xrp_cmd_complete);
			}

			xrp_panic_check(xvp);
#if 0
			/*only for test need removed later*/
			if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG))
			{
				static int index = 0;
				index ++;
				if(index % 4 == 0)
					ret = -EBUSY;
				
			}
#endif
			/* copy back inline data */
			if (ret == 0) {
#ifdef USE_SPRD_MODE
				ret = sprd_complete_hw_request(queue->comm, rq);
#else
				ret = xrp_complete_hw_request(queue->comm, rq);
#endif
#ifdef USE_LOAD_LIB
				if(load_flag == XRP_LOAD_LIB_FLAG) {
					/*check wheter ok flag if not ok , release ion mem allocated for lib code
						release lib info struce*/
					if(ret < 0) {
						printk("yzl add %s xrp_complete_hw_request vdsp side failed load lib\n" , __func__);
						lib_result = xrp_library_decrelease(xvp , libname);
						
					}
				} else if(load_flag == XRP_UNLOAD_LIB_FLAG) {
					if(ret < 0) {
						printk("yzl add %s xrp_complete_hw_request vdsp side failed unload lib\n" , __func__);
					} else {
						/*unload lib , release lib info ion buffer allocated for code & data*/
						lib_result = xrp_library_decrelease(xvp , libname);
					}
                        	}
#endif
			} else if (ret == -EBUSY && firmware_reboot &&
				   atomic_inc_return(&xvp->reboot_cycle) ==
				   reboot_cycle + 1) {
				int rc;
				unsigned i;
				dev_dbg(xvp->dev,
					"%s: restarting firmware...\n",
					 __func__);
				printk("yzl add %s enter reboot flow\n" , __func__);
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_lock(&xvp->queue[i].lock);
				rc = xrp_boot_firmware(xvp);
#ifdef USE_LOAD_LIB
#ifdef USE_RE_REGISTER_LIB
				if(rc == 0) {
					if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG)) {
						mutex_lock(&(xvp->load_lib.libload_mutex));
					}
					/*re-register all */
					lib_result = xrp_register_libs(xvp , queue);
					rc = lib_result;
					printk("yzl add %s xrp_register_libs result:%d\n" , __func__ , lib_result);
					if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG)) {	
						mutex_unlock(&(xvp->load_lib.libload_mutex));
					}
				}
#else
				if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG)) {
					mutex_lock(&(xvp->load_lib.libload_mutex));
				}
				xrp_library_setall_missstate(xvp);
				if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG)) {
					mutex_unlock(&(xvp->load_lib.libload_mutex));
				}
#endif
#endif

				atomic_set(&xvp->reboot_cycle_complete,
					   atomic_read(&xvp->reboot_cycle));
				for (i = 0; i < xvp->n_queues; ++i)
					if (xvp->queue + i != queue)
						mutex_unlock(&xvp->queue[i].lock);
				if (rc < 0) {
					ret = rc;
					went_off = xvp->off;
				}
				rebootflag = 1;
			}
#ifdef USE_LOAD_LIB
#if 0
			if((load_flag == XRP_LOAD_LIB_FLAG)) {
				if(0 == ret){
					xrp_library_increase(xvp , libname);
					libinfo = xrp_library_getlibinfo(xvp , libname);
					if(NULL != libinfo) {
						libinfo->lib_state = XRP_LIBRARY_LOADED;
						printk("yzl add %s , set lib:%s , libstate XRP_LIBRARY_LOADED\n" , __func__ , libname);
					}
				} else {
					/*load failedd release here*/
					xrp_library_decrelease(xvp , libname);
				}
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			} else if(load_flag == XRP_UNLOAD_LIB_FLAG) {
				if(0 == ret) {
					libinfo = xrp_library_getlibinfo(xvp , libname);
					if(NULL != libinfo) {
						libinfo->lib_state = XRP_LIBRARY_IDLE;
					}
					printk("yzl add %s , set lib:%s , libstate XRP_LIBRARY_IDLE libinfo:%p\n" , __func__ , libname , libinfo);
				}
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			} else {
				mutex_lock(&(xvp->load_lib.libload_mutex));
				/*remove processing lib*/
				libinfo = xrp_library_getlibinfo(xvp , rq->nsid);
				printk("yzl add %s xrp_library_getlibinfo libinfo:%p\n" , __func__ , libinfo);
				if(libinfo != NULL) {
					if(libinfo->lib_state != XRP_LIBRARY_PROCESSING_CMD) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
						ret = sprd_unmap_request(filp , rq);
						printk("yzl add %s lib:%s processing cmd , but not XRP_LIBRARY_PROCESSING_CMD state\n",
						       __func__ , rq->nsid);
						return -EINVAL;
					}
					/*set processing libname*/
					libinfo->lib_state = XRP_LIBRARY_LOADED;
				} else {
					;	
				}
				/*set processing lib state*/
				printk("yzl add %s lib:%s , process cmd over\n" , __func__ , rq->nsid);
				mutex_unlock(&(xvp->load_lib.libload_mutex));
			}
#else
			if(0 == rebootflag) {
				if((load_flag != XRP_LOAD_LIB_FLAG) && (load_flag != XRP_UNLOAD_LIB_FLAG)) {
					mutex_lock(&(xvp->load_lib.libload_mutex));
				}
				lib_result = post_process_request(xvp , rq , libname , load_flag , ret);
				if((load_flag == XRP_LOAD_LIB_FLAG) || (load_flag == XRP_UNLOAD_LIB_FLAG)) {
					mutex_unlock(&(xvp->load_lib.libload_mutex));
				}
				else {
					if(lib_result != 0) {
						mutex_unlock(&queue->lock);
						mutex_unlock(&(xvp->load_lib.libload_mutex));
						sprd_unmap_request(filp , rq);
						return -EFAULT;
					}
					else {
						mutex_unlock(&(xvp->load_lib.libload_mutex));
					}
				}
			}
#endif

#endif
		}
		mutex_unlock(&queue->lock);
	}

	if (ret == 0)
#ifdef USE_SPRD_MODE
		ret = sprd_unmap_request(filp, rq);
#else
		ret = xrp_unmap_request(filp, rq);
#endif
	else if (!went_off)
#ifdef USE_SPRD_MODE
		sprd_unmap_request(filp, rq);
#else
		xrp_unmap_request_nowb(filp, rq);
#endif
	/*
	 * Otherwise (if the DSP went off) all mapped buffers are leaked here.
	 * There seems to be no way to recover them as we don't know what's
	 * going on with the DSP; the DSP may still be reading and writing
	 * this memory.
	 */
	printk("yzl add %s , return ret:%ld\n" , __func__ , ret);
	return ret;
}
static long xrp_ioctl_faceid_cmd(struct file *filp, struct xrp_faceid_ctrl __user *arg)
{
	struct xrp_faceid_ctrl faceid;
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;


	if (copy_from_user(&faceid, arg, sizeof(struct xrp_faceid_ctrl)))
		return -EFAULT;

	printk("faceid input addr %x,height %d, width %d\n",faceid.in_data_addr,faceid.in_height,faceid.in_width);

	xrp_faceid_run(xvp,faceid.in_data_addr,faceid.in_height,faceid.in_width);

	return 0;
}
static long xrp_ioctl_set_dvfs(struct file *filp , struct xrp_dvfs_ctrl __user *arg)
{
	struct xrp_dvfs_ctrl dvfs;
	struct xvp_file *xvp_file = filp->private_data;
        struct xvp *xvp = xvp_file->xvp;
	if (copy_from_user(&dvfs, arg, sizeof(struct xrp_dvfs_ctrl)))
                return -EFAULT;
	if(0 == dvfs.en_ctl_flag) {
		xvp_dsp_setdvfs(xvp , dvfs.index);
	} else {
		if(dvfs.enable)
			xvp_dsp_enable_dvfs(xvp);
		else
			xvp_dsp_disable_dvfs(xvp);
	}
	return 0;
} 
static long xvp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long retval;

	printk("%s: %x\n", __func__, cmd);

	switch(cmd){
	case XRP_IOCTL_ALLOC:
		retval = xrp_ioctl_alloc(filp,
					 (struct xrp_ioctl_alloc __user *)arg);
		break;

	case XRP_IOCTL_FREE:
		retval = xrp_ioctl_free(filp,
					(struct xrp_ioctl_alloc __user *)arg);
		break;

	case XRP_IOCTL_QUEUE:
	case XRP_IOCTL_QUEUE_NS:
		retval = xrp_ioctl_submit_sync(filp,
					       (struct xrp_ioctl_queue __user *)arg);
		break;
	case XRP_IOCTL_SET_DVFS:
		retval = xrp_ioctl_set_dvfs(filp , (struct xrp_dvfs_ctrl __user *)arg);
		break;
	case XRP_IOCTL_FACEID_CMD:
		retval = xrp_ioctl_faceid_cmd(filp , (struct xrp_faceid_ctrl __user *)arg);
		break;
	default:
		retval = -EINVAL;
		break;
	}
	return retval;
}

static void xvp_vm_open(struct vm_area_struct *vma)
{
	printk("%s\n", __func__);
	xrp_allocation_get(vma->vm_private_data);
}

static void xvp_vm_close(struct vm_area_struct *vma)
{
	printk("%s\n", __func__);
	xrp_allocation_put(vma->vm_private_data);
}

static const struct vm_operations_struct xvp_vm_ops = {
	.open = xvp_vm_open,
	.close = xvp_vm_close,
};

static int xvp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int err;
	struct xvp_file *xvp_file = filp->private_data;
	unsigned long pfn = vma->vm_pgoff + PFN_DOWN(xvp_file->xvp->pmem);
	struct xrp_allocation *xrp_allocation;

	printk("%s\n", __func__);
	xrp_allocation = xrp_allocation_dequeue(filp->private_data,
						pfn << PAGE_SHIFT,
						vma->vm_end - vma->vm_start);
	if (xrp_allocation) {
		struct xvp *xvp = xvp_file->xvp;
		pgprot_t prot = vma->vm_page_prot;

		if (!xrp_cacheable(xvp, pfn,
				   PFN_DOWN(vma->vm_end - vma->vm_start))) {
			prot = pgprot_writecombine(prot);
			vma->vm_page_prot = prot;
		}

		err = remap_pfn_range(vma, vma->vm_start, pfn,
				      vma->vm_end - vma->vm_start,
				      prot);

		vma->vm_private_data = xrp_allocation;
		vma->vm_ops = &xvp_vm_ops;
	} else {
		err = -EINVAL;
	}

	return err;
}
#ifdef USE_SPRD_MODE
static int32_t sprd_alloc_commbuffer(struct xvp *xvp)
{
	int ret;
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						&xvp->ion_comm,
						ION_HEAP_ID_MASK_SYSTEM,
						PAGE_SIZE);
	if(0 != ret) {
		printk("yzl add %s alloc comm buffer failed\n" , __func__);
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, &xvp->ion_comm);
	if(0 != ret) {
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_comm);
		return -EFAULT;
	}
	xvp->comm = (void*)xvp->ion_comm.addr_k[0];
	xvp->ion_comm.dev = xvp->dev;
	printk("yzl add %s xvp comm vaddr:%p\n" , __func__ , xvp->comm);
	return 0;
}
static int32_t sprd_free_commbuffer(struct xvp *xvp)
{
	printk("yzl add %s xvp comm:%p\n" , __func__ , xvp->comm);
	if(xvp->comm) {
                xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_comm);
                xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_comm);
                xvp->comm = NULL;
        }
	return 0;
}
static int32_t sprd_alloc_extrabuffer(struct xvp *xvp)
{
	int ret;
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						&xvp->ion_firmware,
						ION_HEAP_ID_MASK_SYSTEM,
						VDSP_FIRMWIRE_SIZE);
	printk("yzl add %s , after ion_alloc firmware buffer ret:%d\n" , __func__ ,ret);
	if(ret != 0)
		return -ENOMEM;
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, &xvp->ion_firmware);
	printk("yzl add after kmap ion_firmware\n");
	xvp->firmware_viraddr = (void*)xvp->ion_firmware.addr_k[0];
	//        xvp->firmware_phys = phys;
	xvp->ion_firmware.dev = xvp->dev;
	if(ret){
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->firmware_viraddr = NULL;
		return -EFAULT;
	}


	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						&xvp->ion_dram_back,
						ION_HEAP_ID_MASK_SYSTEM,
						PAGE_SIZE);
	printk("yzl add after ion_alloc ion_dram_back ret:%d\n" ,ret);
	if(ret) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->firmware_viraddr = NULL;
		return -ENOMEM;
	}
	//      xvp->dram_phys = phys;
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, &xvp->ion_dram_back);
	printk("yzl add after kmap ion_dram_back\n");
	xvp->dram_viraddr = (void*) xvp->ion_dram_back.addr_k[0];
	xvp->ion_dram_back.dev = xvp->dev;

	if(ret) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_dram_back);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_dram_back);
		xvp->firmware_viraddr = NULL;
		xvp->dram_viraddr = NULL;
		return -EFAULT;
	}
	printk("yzl add %s firmware vaddr:0x%p , dram back vaddr:0x%p\n", __func__ , xvp->firmware_viraddr,
	       xvp->dram_viraddr);
	return ret;
}

static int sprd_free_extrabuffer(struct xvp *xvp)
{
	printk("yzl add %s ,enter firmwareaddr:%p , drram addr:%p\n" , __func__ , xvp->firmware_viraddr,xvp->dram_viraddr);
	if(xvp->firmware_viraddr) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_firmware);
		xvp->firmware_viraddr = NULL;
	}
	if(xvp->dram_viraddr) {
		xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_dram_back);
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_dram_back);
		xvp->dram_viraddr = NULL;
	}
	return 0;
}
static int32_t sprd_iommu_map_commbuffer(struct xvp *xvp)
{
	int ret = -EFAULT;
	if(xvp->comm == NULL) {
		printk("yzl add %s map comm addr is %p\n" , __func__,xvp->comm);
		return ret;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, &xvp->ion_comm , /*IOMMU_MSTD*/IOMMU_ALL);
	if(ret) {
		printk("yzl add %s map ion_comm fialed\n" , __func__);
		return ret;
	}
	xvp->dsp_comm_addr = xvp->ion_comm.iova[0];
	return ret;
}
static int32_t sprd_iommu_unmap_commbuffer(struct xvp *xvp)
{
	int ret;
	if(xvp->comm) {
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, &xvp->ion_comm , /*IOMMU_MSTD*/IOMMU_ALL);
		if(ret) {
			printk("yzl add %s unmap ion_comm fialed\n" , __func__);
			return -EFAULT;
		}
	} else {
		printk("yzl add %s xvp->comm is NULL\n" , __func__);
		return -EINVAL;
	}
	 printk("yzl add %s xvp->comm:%p\n" , __func__ ,
               xvp->comm);
	return 0;
}
static int sprd_iommu_map_extrabuffer(struct xvp *xvp)
{
	int ret = -EFAULT;
	if((xvp->firmware_viraddr == NULL) || (xvp->dram_viraddr == NULL)) {
		printk("yzl add %s map some addr is NULL firmware:%p , dram:%p\n" , __func__,
		       xvp->firmware_viraddr , xvp->dram_viraddr);
		return ret;
	}
	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, &xvp->ion_firmware , IOMMU_ALL);
		if(ret) {
			printk("yzl add %s map firmware fialed\n" , __func__);
			return ret;
		}
		xvp->dsp_firmware_addr = xvp->ion_firmware.iova[0];
	}


	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, &xvp->ion_dram_back , /*IOMMU_MSTD*/IOMMU_ALL);
		if(ret) {
			xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, &xvp->ion_firmware , IOMMU_ALL);
			printk("yzl add %s map ion_dram_back fialed\n" , __func__);
			return ret;
		}
		xvp->dsp_dram_addr = xvp->ion_dram_back.iova[0];
	}
	printk("yzl add %s , dsp firmware addr:%lx , dsp dram addr:%lx\n" , __func__,
	       (unsigned long)xvp->dsp_firmware_addr , (unsigned long)xvp->dsp_dram_addr);
	return ret;
}
static int sprd_iommu_unmap_extrabuffer(struct xvp *xvp)
{
	int ret = -EFAULT;
	int ret1 = 0;
	if((xvp->firmware_viraddr == NULL) || (xvp->dram_viraddr == NULL)) {
		printk("yzl add %s unmap some addr is NULL firmware:%p , dram:%p\n" , __func__,
                       xvp->firmware_viraddr , xvp->dram_viraddr);
		return ret;
	}
	if(xvp->firmware_viraddr) {
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, &xvp->ion_firmware , IOMMU_ALL);
		if(ret) {
			ret1 = -EFAULT;
			printk("yzl add %s unmap firmware fialed\n" , __func__);
		}
	}
	if(xvp->dram_viraddr) {
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, &xvp->ion_dram_back , /*IOMMU_MSTD*/IOMMU_ALL);
		if(ret) {
			ret1 = -EFAULT;
			printk("yzl add %s map ion_dram_back fialed\n" , __func__);
		}
	}
	printk("yzl add %s unmap extrabuffer frimwarddr:%p , dram addr:%p, ret:%d, ret1:%d\n" , __func__ ,
	       xvp->firmware_viraddr ,xvp->dram_viraddr, ret , ret1);
	return ((ret!=0)||(ret1!=0)) ? -EFAULT : 0;
}
static int sprd_alloc_faceid_fwbuffer(struct xvp *xvp)
{
	int ret;
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						&xvp->ion_faceid_fw,
						ION_HEAP_ID_MASK_VDSP,/*todo vdsp head id*/
						VDSP_FACEID_FIRMWIRE_SIZE);
	if(0 != ret) {
		printk("yzl add %s  failed,ret %d\n" , __func__,ret);
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, &xvp->ion_faceid_fw);
	if(0 != ret) {
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_faceid_fw);
		return -EFAULT;
	}
	xvp->faceid_fw_viraddr = (void*)xvp->ion_faceid_fw.addr_k[0];
	xvp->ion_faceid_fw.dev = xvp->dev;
	printk("yzl add %s vaddr:%p\n" , __func__ , xvp->faceid_fw_viraddr);
	return 0;

}

static int sprd_free_faceid_fwbuffer(struct xvp *xvp)
{
	printk("yzl add %s xvp faceid fw:%p\n" , __func__ , xvp->faceid_fw_viraddr);
	if(xvp->faceid_fw_viraddr) {
                xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, &xvp->ion_faceid_fw);
                xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, &xvp->ion_faceid_fw);
                xvp->faceid_fw_viraddr = NULL;
	}
	return 0;
}
static int sprd_iommu_map_faceid_fwbuffer(struct xvp *xvp)
{
	int ret = -EFAULT;
	if(xvp->faceid_fw_viraddr == NULL) {
		printk("yzl add map faceid fw addr is NULL \n");
		return ret;
	}
	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, &xvp->ion_faceid_fw , IOMMU_ALL);
		if(ret) {
			printk("yzl add %s map faceid fialed\n" , __func__);
			return ret;
		}
		xvp->dsp_firmware_addr = xvp->ion_faceid_fw.iova[0];
	}
	printk("yzl add %s:%p --> %lx\n" , __func__,xvp->faceid_fw_viraddr,(unsigned long)xvp->dsp_firmware_addr);
	return ret;
}
static int sprd_iommu_unmap_faceid_fwbuffer(struct xvp *xvp)
{
	int ret = -EFAULT;
	int ret1 = 0;

	if(xvp->faceid_fw_viraddr == NULL) {
		printk("yzl add unmap faceid fw addr is NULL\n");
		return ret;
	}
	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, &xvp->ion_firmware , IOMMU_ALL);
		if(ret) {
			ret1 = -EFAULT;
			printk("yzl add %s unmap faceid fw fialed\n" , __func__);
		}
	}

	printk("yzl add %s unmap faceid fw :%p , ret:%d, ret1:%d\n" , __func__ ,
	       xvp->faceid_fw_viraddr , ret , ret1);
	return ((ret!=0)||(ret1!=0)) ? -EFAULT : 0;
}

int sprd_faceid_init(struct xvp *xvp)
{
	int ret = 0;

	ret = sprd_alloc_faceid_fwbuffer(xvp);
	if(ret < 0)
		return ret;

	sprd_faceid_request_weights(xvp);
    return 0;
}
int sprd_faceid_deinit(struct xvp *xvp)
{
	sprd_free_faceid_fwbuffer(xvp);
	sprd_faceid_release_weights(xvp);
	return 0;
}
#endif
static int xvp_open(struct inode *inode, struct file *filp)
{
	struct xvp *xvp = container_of(filp->private_data,
				       struct xvp, miscdev);
	struct xvp_file *xvp_file;
	int rc;

	printk("%s , xvp is:%p , flags:%x, fmode:%x\n", __func__ , xvp , filp->f_flags , filp->f_mode);
		if(filp->f_flags & O_RDWR)
	{
		xvp->secmode = true;
		xvp->tee_con = vdsp_ca_connect();
		//if(!xvp->tee_con)
			//return -EACCES;
	}
	rc = pm_runtime_get_sync(xvp->dev);
	if (rc < 0)
		return rc;

	xvp_file = devm_kzalloc(xvp->dev, sizeof(*xvp_file), GFP_KERNEL);
	if (!xvp_file) {
		pm_runtime_put_sync(xvp->dev);
		return -ENOMEM;
	}

	xvp_file->xvp = xvp;
	spin_lock_init(&xvp_file->busy_list_lock);
	filp->private_data = xvp_file;
	xrp_add_known_file(filp);
#ifdef USE_LOAD_LIB
	xvp->open_count ++;
#endif

	return 0;
}

static int xvp_close(struct inode *inode, struct file *filp)
{
	struct xvp_file *xvp_file = filp->private_data;
	int ret = 0;
	printk("%s\n", __func__);

	if(xvp_file->xvp->secmode && xvp_file->xvp->tee_con)
	{
		struct vdsp_msg msg;
		msg.vdsp_type = TA_CADENCE_VQ6;
		msg.msg_cmd = TA_FACEID_EXIT_SEC_MODE;
		vdsp_set_sec_mode(&msg);

		vdsp_ca_disconnect();
		xvp_file->xvp->secmode = false;
		xvp_file->xvp->tee_con = false;
	}

	xrp_remove_known_file(filp);
	pm_runtime_put_sync(xvp_file->xvp->dev);
#ifdef USE_LOAD_LIB
	xvp_file->xvp->open_count--;
	printk("yzl add %s , open_count is:%d\n" , __func__ , xvp_file->xvp->open_count);
	if(0 == xvp_file->xvp->open_count) {
		/*release xvp load_lib info*/
		ret = xrp_library_release_all(xvp_file->xvp);
	}
#endif
	devm_kfree(xvp_file->xvp->dev, xvp_file);
	printk("yzl add %s , ret:%d\n" , __func__ , ret);
	return ret;
}

static inline int xvp_enable_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->enable)
		return xvp->hw_ops->enable(xvp->hw_arg);
	else
		return 0;
}

static inline void xvp_disable_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->disable)
		xvp->hw_ops->disable(xvp->hw_arg);
}

static inline void xrp_reset_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->reset)
		xvp->hw_ops->reset(xvp->hw_arg);
}

static inline void xrp_halt_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->halt)
		xvp->hw_ops->halt(xvp->hw_arg);
}

static inline void xrp_release_dsp(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->release)
		xvp->hw_ops->release(xvp->hw_arg);
}
static inline void xvp_dsp_enable_dvfs(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->enable_dvfs)
		xvp->hw_ops->enable_dvfs(xvp->hw_arg);
}
static inline void xvp_dsp_disable_dvfs(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->disable_dvfs)
		xvp->hw_ops->disable_dvfs(xvp->hw_arg);
}
static inline void xvp_dsp_setdvfs(struct xvp *xvp , uint32_t index)
{
	if (loopback < LOOPBACK_NOMMIO &&
	    xvp->hw_ops->setdvfs)
		xvp->hw_ops->setdvfs(xvp->hw_arg , index);
}
static inline void xvp_parse_qos(struct xvp *xvp , struct device_node *ofnode)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->parse_qos)
		xvp->hw_ops->parse_qos(xvp->hw_arg , ofnode);
}
static inline void xvp_set_qos(struct xvp *xvp)
{
	if (loopback < LOOPBACK_NOMMIO &&
		xvp->hw_ops->parse_qos)
		xvp->hw_ops->set_qos(xvp->hw_arg);
}
static int xrp_runtime_resume_normal(struct xvp *xvp)
{
	unsigned i;
	int ret = 0;
#ifdef USE_SPRD_MODE
	int ret1 = 0;
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
	printk("yzl add %s enter\n" , __func__);
#endif
	for (i = 0; i < xvp->n_queues; ++i)
		mutex_lock(&xvp->queue[i].lock);

	if (xvp->off)
		goto out;
	ret = xvp_enable_dsp(xvp);
	if (ret < 0) {
		dev_err(xvp->dev, "couldn't enable DSP\n");
		goto out;
	}
#ifdef USE_SPRD_MODE
	ret = sprd_alloc_extrabuffer(xvp);
	if(ret != 0) {
		printk("yzl add %s sprd_alloc_extrabuffer failed\n" , __func__);
		goto out;
	}
	ret = sprd_iommu_map_extrabuffer(xvp);
	if(ret != 0) {
		printk("yzl add %s sprd_iommu_map_extrabuffer failed\n" , __func__);
		sprd_free_extrabuffer(xvp);
		goto out;
	}
	ret = sprd_iommu_map_commbuffer(xvp);
	if(ret != 0) {
		sprd_iommu_unmap_extrabuffer(xvp);
		sprd_free_extrabuffer(xvp);
		printk("yzl add %s sprd_iommu_map_commbuffer failed\n" , __func__);
		goto out;
	}
	ipidesc = get_vdsp_ipi_ctx_desc();
	if(ipidesc) {
		printk("yzl add %s ipi init called\n" , __func__);
		ipidesc->ops->ctx_init(ipidesc);
	}
#endif
	/*set qos*/
	xvp_set_qos(xvp);

	ret = xrp_boot_firmware(xvp);
	if (ret < 0) {
#ifdef USE_SPRD_MODE
		ret1 = sprd_iommu_unmap_extrabuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s sprd_iommu_unmap_extrabuffer failed\n" , __func__);
		}
		ret1 = sprd_free_extrabuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s sprd_free_extrabuffer failed boot firmware failed\n" , __func__);
		}
		ret1 = sprd_iommu_unmap_commbuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s sprd_iommu_unmap_commbuffer failed boot firmware failed\n" , __func__);
		}
#endif
		xvp_disable_dsp(xvp);
	}

out:
	for (i = 0; i < xvp->n_queues; ++i)
		mutex_unlock(&xvp->queue[i].lock);

	return ret;
}
static int xrp_runtime_resume_secure(struct xvp *xvp)
{
	unsigned i;
	int ret = 0;
	int ret1 = 0;
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
	printk("yzl add %s enter\n" , __func__);

	for (i = 0; i < xvp->n_queues; ++i)
		mutex_lock(&xvp->queue[i].lock);

	if (xvp->off)
		goto out;
	ret = xvp_enable_dsp(xvp);
	if (ret < 0) {
		dev_err(xvp->dev, "couldn't enable DSP\n");
		goto out;
	}
#ifdef USE_SPRD_MODE
	ret = sprd_iommu_map_faceid_fwbuffer(xvp);
	if(ret != 0) {
		printk("yzl add %s map faceid fw failed\n" , __func__);
		goto out;
	}
	ret = sprd_iommu_map_commbuffer(xvp);
	if(ret != 0) {
		sprd_iommu_unmap_faceid_fwbuffer(xvp);
		printk("yzl add %s map comm buffer failed\n" , __func__);
		goto out;
	}
	/*
	ret = sprd_iommu_map_faceid_weights(xvp);
	if(ret != 0) {
		sprd_iommu_unmap_faceid_fwbuffer(xvp);
		sprd_iommu_unmap_commbuffer(xvp);
		printk("yzl add %s map weights failed\n" , __func__);
		goto out;
	}*/
	ipidesc = get_vdsp_ipi_ctx_desc();
	if(ipidesc) {
		printk("yzl add %s ipi init called\n" , __func__);
		ipidesc->ops->ctx_init(ipidesc);
	}
#endif
	/*set qos*/
	xvp_set_qos(xvp);
	ret = xrp_boot_firmware(xvp);
	if (ret < 0) {
#ifdef USE_SPRD_MODE
		ret1 = sprd_iommu_unmap_faceid_fwbuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s unmap faceid fw failed\n" , __func__);
		}
		ret1 = sprd_iommu_unmap_commbuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s unmap comm buffer failed\n" , __func__);
		}
		/*
		ret1 = sprd_iommu_unmap_faceid_weights(xvp);
		if(ret1 != 0) {
			printk("yzl add %s unmap weights failed\n" , __func__);
		}*/
#endif
		xvp_disable_dsp(xvp);
	}

out:
	for (i = 0; i < xvp->n_queues; ++i)
		mutex_unlock(&xvp->queue[i].lock);

	return ret;
}
static int xrp_boot_firmware(struct xvp *xvp)
{
	int ret;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	static char faceid_fw[16] = "faceid_fw.bin";

	if(xvp->secmode && xvp->tee_con)
	{
		struct vdsp_msg msg;
		msg.vdsp_type = TA_CADENCE_VQ6;
		msg.msg_cmd = TA_FACEID_EXIT_SEC_MODE;
		vdsp_set_sec_mode(&msg);
	}
	else
	{
		printk("Exit secure mode fail\n");
	}
	xrp_halt_dsp(xvp);
	xrp_reset_dsp(xvp);
	if(xvp->secmode)
	{
		printk("yzl add %s, faceid_fw_viraddr %p , loopback:%d\n" , __func__ , xvp->faceid_fw_viraddr,loopback);
		if (xvp->faceid_fw_viraddr) {
			if (loopback < LOOPBACK_NOFIRMWARE) {
				ret = xrp_faceid_request_firmware(xvp,faceid_fw);
				if (ret < 0)
					return ret;
			}

			if (loopback < LOOPBACK_NOIO) {
				xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
				mb();
			}
		}

	}
	else
	{
		printk("yzl add %s firmware name:%s , loopback:%d\n" , __func__ , xvp->firmware_name , loopback);
		if (xvp->firmware_name) {
			if (loopback < LOOPBACK_NOFIRMWARE) {
				ret = xrp_request_firmware(xvp);
				if (ret < 0)
					return ret;
			}

			if (loopback < LOOPBACK_NOIO) {
				xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
				mb();
			}
		}
	}
	if(xvp->secmode && xvp->tee_con)
	{
		struct vdsp_msg msg;
		msg.vdsp_type = TA_CADENCE_VQ6;
		msg.msg_cmd = TA_FACEID_ENTER_SEC_MODE;
		vdsp_set_sec_mode(&msg);
	}
	else
	{
		printk("Enter secure mode fail\n");
	}
	xrp_release_dsp(xvp);

	if (loopback < LOOPBACK_NOIO) {
		ret = xrp_synchronize(xvp);
		if (ret < 0) {
			xrp_halt_dsp(xvp);
			dev_err(xvp->dev,
				"%s: couldn't synchronize with the DSP core\n",
				__func__);
			dev_err(xvp->dev,
				"XRP device will not use the DSP until the driver is rebound to this device\n");
			xvp->off = true;
			return ret;
		}
	}
	return 0;
}

static const struct file_operations xvp_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = xvp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = xvp_ioctl,
#endif
	.mmap = xvp_mmap,
	.open = xvp_open,
	.release = xvp_close,
};

int xrp_runtime_suspend(struct device *dev)
{
#ifdef USE_SPRD_MODE
	int ret;
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
#endif
	struct xvp *xvp = dev_get_drvdata(dev);

	xrp_halt_dsp(xvp);
#ifdef USE_SPRD_MODE
	if(xvp->secmode)
	{
		ret = sprd_iommu_unmap_faceid_fwbuffer(xvp);
		if(ret != 0) {
			printk("yzl add %s unmap faceid fw failed\n" , __func__);
		}
		/*
		ret = sprd_iommu_unmap_faceid_weights(xvp);
		if(ret != 0) {
			printk("yzl add %s unmap faceid weights failed\n" , __func__);
		}*/
	}
	else
	{
		ret = sprd_iommu_unmap_extrabuffer(xvp);
		if(ret != 0) {
			printk("yzl add %s sprd_iommu_unmap_extrabuffer failed\n" , __func__);
		}
		ret = sprd_free_extrabuffer(xvp);
		if(ret != 0) {
			printk("yzl add %s sprd_free_extrabuffer failed\n" , __func__);
		}
	}
	ret = sprd_iommu_unmap_commbuffer(xvp);
	if(ret != 0) {
		printk("yzl add %s sprd_iommu_unmap_commbuffer failed\n" , __func__);
	}
#endif
	xvp_disable_dsp(xvp);
#ifdef USE_SPRD_MODE
	ipidesc = get_vdsp_ipi_ctx_desc();
	if(ipidesc) {
                printk("yzl add %s ipi deinit called\n" , __func__);
                ipidesc->ops->ctx_deinit(ipidesc);
        }
#endif
	return 0;
}
EXPORT_SYMBOL(xrp_runtime_suspend);

int xrp_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct xvp *xvp = dev_get_drvdata(dev);

	if(xvp->secmode)
		ret = xrp_runtime_resume_secure(xvp);
	else
		ret = xrp_runtime_resume_normal(xvp);

	if(!ret)
		sprd_log_sem_up();

	return ret;
}
EXPORT_SYMBOL(xrp_runtime_resume);

static int xrp_init_regs_v0(struct platform_device *pdev, struct xvp *xvp)
{
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem)
		return -ENODEV;

	xvp->comm_phys = mem->start;
	xvp->comm = devm_ioremap_resource(&pdev->dev, mem);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!mem)
		return -ENODEV;

	xvp->pmem = mem->start;
	xvp->shared_size = resource_size(mem);
	return xrp_init_private_pool(&xvp->pool, xvp->pmem,
				     xvp->shared_size);
}

static int xrp_init_regs_v1(struct platform_device *pdev, struct xvp *xvp)
{
	struct resource *mem;
	struct resource r;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;

	if (resource_size(mem) < 2 * PAGE_SIZE) {
		dev_err(xvp->dev,
			"%s: shared memory size is too small\n",
			__func__);
		return -ENOMEM;
	}

	xvp->comm_phys = mem->start;
	xvp->pmem = mem->start + PAGE_SIZE;
	xvp->shared_size = resource_size(mem) - PAGE_SIZE;

	r = *mem;
	r.end = r.start + PAGE_SIZE;
	xvp->comm = devm_ioremap_resource(&pdev->dev, &r);
	return xrp_init_private_pool(&xvp->pool, xvp->pmem,
				     xvp->shared_size);
}

static int xrp_init_regs_sprd(struct platform_device *pdev, struct xvp *xvp)
{
	int ret;
	ret = sprd_alloc_commbuffer(xvp);
	if(ret != 0) {
		printk("yzl add %s sprd_alloc_commbuffer failed\n" , __func__);
		return -EFAULT;
	}
	return 0;//xrp_init_cma_pool(&xvp->pool, xvp->dev);
}
static int xrp_init_regs_cma(struct platform_device *pdev, struct xvp *xvp)
{
	dma_addr_t comm_phys;
	if (of_reserved_mem_device_init(xvp->dev) < 0)
		return -ENODEV;
	xvp->comm = dma_alloc_attrs(xvp->dev, PAGE_SIZE, &comm_phys,
				GFP_KERNEL, 0);
	if (!xvp->comm)
		return -ENOMEM;
	xvp->comm_phys = dma_to_phys(xvp->dev, comm_phys);
	printk("yzl add %s after dma_to_phys comm_phys:%lx\n" , __func__ , (unsigned long)xvp->comm_phys);

	return xrp_init_cma_pool(&xvp->pool, xvp->dev);

}
static int compare_queue_priority(const void *a, const void *b)
{
	const void * const *ppa = a;
	const void * const *ppb = b;
	const struct xrp_comm *pa = *ppa, *pb = *ppb;

	if (pa->priority == pb->priority)
		return 0;
	else
		return pa->priority < pb->priority ? -1 : 1;
}

static long xrp_init_common(struct platform_device *pdev,
			    enum xrp_init_flags init_flags,
			    const struct xrp_hw_ops *hw_ops, void *hw_arg,
			    int (*xrp_init_regs)(struct platform_device *pdev,
						 struct xvp *xvp))
{
	long ret;
	char nodename[sizeof("xvp") + 3 * sizeof(int)];
	struct xvp *xvp;
	int nodeid;
	unsigned i;

	xvp = devm_kzalloc(&pdev->dev, sizeof(*xvp), GFP_KERNEL);
	if (!xvp) {
		ret = -ENOMEM;
		goto err;
	}
	sprd_log_sem_init();
#ifdef USE_LOAD_LIB
	mutex_init(&(xvp->load_lib.libload_mutex));
	mutex_init(&map_lock);
	xvp->open_count = 0;
#endif
	xvp->dev = &pdev->dev;
	xvp->hw_ops = hw_ops;
	xvp->hw_arg = hw_arg;
	xvp->secmode = false;
	xvp->tee_con = false;

	xvp->vdsp_mem_desc = get_vdsp_mem_ctx_desc(xvp->dev);
	if (!xvp->vdsp_mem_desc) {
		pr_err("fail to get mem desc\n");
		ret = -1;
		goto err;
	}
	mutex_init(&(xvp->vdsp_mem_desc->iommu_lock));

	if (init_flags & XRP_INIT_USE_HOST_IRQ)
		xvp->host_irq_mode = true;
	platform_set_drvdata(pdev, xvp);

	ret = xrp_init_regs(pdev, xvp);
	if (ret < 0)
		goto err;

	printk("%s: comm = %pap/%p\n", __func__, &xvp->comm_phys, xvp->comm);
	printk("%s: xvp->pmem = %pap\n", __func__, &xvp->pmem);

	ret = xrp_init_address_map(xvp->dev, &xvp->address_map);
	if (ret < 0)
		goto err_free_pool;

	ret = device_property_read_u32_array(xvp->dev, "queue-priority",
					     NULL, 0);
	if (ret > 0) {
		xvp->n_queues = ret;
		xvp->queue_priority = devm_kmalloc(&pdev->dev,
						   ret * sizeof(u32),
						   GFP_KERNEL);
		if (xvp->queue_priority == NULL)
			goto err_free_pool;
		ret = device_property_read_u32_array(xvp->dev,
						     "queue-priority",
						     xvp->queue_priority,
						     xvp->n_queues);
		if (ret < 0)
			goto err_free_pool;
		dev_dbg(xvp->dev,
			"multiqueue (%d) configuration, queue priorities:\n",
			xvp->n_queues);
		for (i = 0; i < xvp->n_queues; ++i)
			dev_dbg(xvp->dev, "  %d\n", xvp->queue_priority[i]);
	} else {
		xvp->n_queues = 1;
	}
	xvp->queue = devm_kmalloc(&pdev->dev,
				  xvp->n_queues * sizeof(*xvp->queue),
				  GFP_KERNEL);
	xvp->queue_ordered = devm_kmalloc(&pdev->dev,
					  xvp->n_queues * sizeof(*xvp->queue_ordered),
					  GFP_KERNEL);
	if (xvp->queue == NULL ||
	    xvp->queue_ordered == NULL)
		goto err_free_pool;

	for (i = 0; i < xvp->n_queues; ++i) {
		mutex_init(&xvp->queue[i].lock);
		xvp->queue[i].comm = xvp->comm + XRP_DSP_CMD_STRIDE * i;
		init_completion(&xvp->queue[i].completion);
		if (xvp->queue_priority)
			xvp->queue[i].priority = xvp->queue_priority[i];
		xvp->queue_ordered[i] = xvp->queue + i;
		printk("yzl add %s , queue i:%d, comm:%p\n" , __func__ , i ,xvp->queue[i].comm);
	}
	sort(xvp->queue_ordered, xvp->n_queues, sizeof(*xvp->queue_ordered),
	     compare_queue_priority, NULL);
	if (xvp->n_queues > 1) {
		dev_dbg(xvp->dev, "SW -> HW queue priority mapping:\n");
		for (i = 0; i < xvp->n_queues; ++i) {
			dev_dbg(xvp->dev, "  %d -> %d\n",
				i, xvp->queue_ordered[i]->priority);
		}
	}

	ret = device_property_read_string(xvp->dev, "firmware-name",
					  &xvp->firmware_name);
	if (ret == -EINVAL || ret == -ENODATA) {
		dev_dbg(xvp->dev,
			"no firmware-name property, not loading firmware");
	} else if (ret < 0) {
		dev_err(xvp->dev, "invalid firmware name (%ld)", ret);
		goto err_free_map;
	}
	/*qos parse*/
	xvp_parse_qos(xvp , pdev->dev.of_node);
	pm_runtime_enable(xvp->dev);
	if (!pm_runtime_enabled(xvp->dev)) {
		ret = xrp_runtime_resume(xvp->dev);
		if (ret)
			goto err_pm_disable;
	}

	nodeid = ida_simple_get(&xvp_nodeid, 0, 0, GFP_KERNEL);
	if (nodeid < 0) {
		ret = nodeid;
		goto err_pm_disable;
	}
	xvp->nodeid = nodeid;
	sprintf(nodename, "vdsp%u", nodeid);

/*workaround now , may be remove later because 936 m is default freq, we may use 512m for stability temperary*/
	{
		struct clk *vdspclk , *vdspparentclk;
		vdspclk = devm_clk_get(xvp->dev , "vdsp_clk_set");
		vdspparentclk = devm_clk_get(xvp->dev , "vdsp_clk_512");
		if((vdspclk != NULL) && (vdspparentclk != NULL)) {
			printk("yzl add %s before set vdsp clk\n" , __func__);
			clk_set_parent(vdspclk , vdspparentclk);
			clk_prepare_enable(vdspclk);
		}
	}
/*workaround end*/

	xvp->miscdev = (struct miscdevice){
		.minor = MISC_DYNAMIC_MINOR,
		.name = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
		.nodename = devm_kstrdup(&pdev->dev, nodename, GFP_KERNEL),
		.fops = &xvp_fops,
	};
	sprd_faceid_init(xvp);
	ret = misc_register(&xvp->miscdev);
	if (ret < 0)
		goto err_free_id;
	return PTR_ERR(xvp);
err_free_id:
	ida_simple_remove(&xvp_nodeid, nodeid);
err_pm_disable:
	pm_runtime_disable(xvp->dev);
err_free_map:
	xrp_free_address_map(&xvp->address_map);
err_free_pool:
#ifdef USE_SPRD_MODE
	if(xvp->pool != NULL)
#endif
	xrp_free_pool(xvp->pool);
	if (xvp->comm_phys && !xvp->pmem) {
		dma_free_attrs(xvp->dev, PAGE_SIZE, xvp->comm,
			       phys_to_dma(xvp->dev, xvp->comm_phys), 0);
	}
err:
	dev_err(&pdev->dev, "%s: ret = %ld\n", __func__, ret);
	return ret;
}

typedef long xrp_init_function(struct platform_device *pdev,
			       enum xrp_init_flags flags,
			       const struct xrp_hw_ops *hw_ops, void *hw_arg);

xrp_init_function xrp_init;
long xrp_init(struct platform_device *pdev, enum xrp_init_flags flags,
	      const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_v0);
}
EXPORT_SYMBOL(xrp_init);

xrp_init_function xrp_init_v1;
long xrp_init_v1(struct platform_device *pdev, enum xrp_init_flags flags,
		 const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_v1);
}
EXPORT_SYMBOL(xrp_init_v1);

xrp_init_function xrp_init_cma;
long xrp_init_cma(struct platform_device *pdev, enum xrp_init_flags flags,
		  const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
	return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_cma);
}
EXPORT_SYMBOL(xrp_init_cma);

#ifdef USE_SPRD_MODE
xrp_init_function xrp_init_sprd;
long xrp_init_sprd(struct platform_device *pdev, enum xrp_init_flags flags,
                  const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
        return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_sprd);
}
EXPORT_SYMBOL(xrp_init_sprd);
#endif

int xrp_deinit(struct platform_device *pdev)
{
#ifdef USE_SPRD_MODE
	struct xvp *xvp = platform_get_drvdata(pdev);

	pm_runtime_disable(xvp->dev);
	if (!pm_runtime_status_suspended(xvp->dev))
		xrp_runtime_suspend(xvp->dev);

	misc_deregister(&xvp->miscdev);
	release_firmware(xvp->firmware);
	sprd_free_commbuffer(xvp);
	sprd_faceid_deinit(xvp);
	xrp_free_address_map(&xvp->address_map);
	ida_simple_remove(&xvp_nodeid, xvp->nodeid);
	return 0;
#else
	struct xvp *xvp = platform_get_drvdata(pdev);

	pm_runtime_disable(xvp->dev);
	if (!pm_runtime_status_suspended(xvp->dev))
		xrp_runtime_suspend(xvp->dev);

	misc_deregister(&xvp->miscdev);
	release_firmware(xvp->firmware);
#ifdef USE_SPRD_MODE
	if(xvp->pool != NULL)
#endif
	xrp_free_pool(xvp->pool);
	if (xvp->comm_phys && !xvp->pmem) {
		dma_free_attrs(xvp->dev, PAGE_SIZE, xvp->comm,
			       phys_to_dma(xvp->dev, xvp->comm_phys), 0);
	}
	xrp_free_address_map(&xvp->address_map);
	ida_simple_remove(&xvp_nodeid, xvp->nodeid);
	return 0;
#endif
}
EXPORT_SYMBOL(xrp_deinit);

int xrp_deinit_hw(struct platform_device *pdev, void **hw_arg)
{
	if (hw_arg) {
		struct xvp *xvp = platform_get_drvdata(pdev);
		*hw_arg = xvp->hw_arg;
	}
	return xrp_deinit(pdev);
}
EXPORT_SYMBOL(xrp_deinit_hw);
#if 0
static void *get_hw_sync_data(void *hw_arg, size_t *sz)
{
	void *p = kzalloc(64, GFP_KERNEL);

	*sz = 64;
	return p;
}

static const struct xrp_hw_ops hw_ops = {
	.get_hw_sync_data = get_hw_sync_data,
};
#ifdef CONFIG_OF
static const struct of_device_id xrp_of_match[] = {
	{
		.compatible = "cdns,xrp",
		.data = xrp_init,
	}, {
		.compatible = "cdns,xrp,v1",
		.data = xrp_init_v1,
	}, {
		.compatible = "cdns,xrp,cma",
		.data = xrp_init_cma,
	},
#ifdef USE_SPRD_MODE 
	{
		.compatible = "cdns,xrp,sprd",
		.data = xrp_init_sprd,
	},
#endif
	{},
};
MODULE_DEVICE_TABLE(of, xrp_of_match);
#endif
#ifdef CONFIG_ACPI
static const struct acpi_device_id xrp_acpi_match[] = {
	{ "CXRP0001", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, xrp_acpi_match);
#endif
static int xrp_probe(struct platform_device *pdev)
{
	long ret = -EINVAL;

#ifdef CONFIG_OF
	{
		const struct of_device_id *match;
		xrp_init_function *init;

	        match = of_match_device(xrp_of_match, &pdev->dev);
		init = match->data;
		ret = init(pdev, 0, &hw_ops, NULL);
		return IS_ERR_VALUE(ret) ? ret : 0;
	}
#endif
#ifdef CONFIG_ACPI
	ret = xrp_init_v1(pdev, 0, &hw_ops, NULL);
	if (!IS_ERR_VALUE(ret)) {
		struct xrp_address_map_entry *entry;
		struct xvp *xvp = ERR_PTR(ret);

		ret = 0;
		/*
		 * On ACPI system DSP can currently only access
		 * its own shared memory.
		 */
		entry = xrp_get_address_mapping(&xvp->address_map,
						xvp->comm_phys);
		if (entry) {
			entry->src_addr = xvp->comm_phys;
			entry->dst_addr = (u32)xvp->comm_phys;
			entry->size = (u32)xvp->shared_size + PAGE_SIZE;
		} else {
			dev_err(xvp->dev,
				"%s: couldn't find mapping for shared memory\n",
				__func__);
			ret = -EINVAL;
		}
	}
#endif
	return ret;
}

static int xrp_remove(struct platform_device *pdev)
{
	return xrp_deinit(pdev);
}

static const struct dev_pm_ops xrp_pm_ops = {
	SET_RUNTIME_PM_OPS(xrp_runtime_suspend,
			   xrp_runtime_resume, NULL)
};

static struct platform_driver xrp_driver = {
	.probe   = xrp_probe,
	.remove  = xrp_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(xrp_of_match),
		.acpi_match_table = ACPI_PTR(xrp_acpi_match),
		.pm = &xrp_pm_ops,
	},
};

module_platform_driver(xrp_driver);
#endif
MODULE_AUTHOR("Takayuki Sugawara");
MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: Linux device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
