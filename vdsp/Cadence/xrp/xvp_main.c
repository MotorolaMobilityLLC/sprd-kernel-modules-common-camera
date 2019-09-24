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
#include "xrp_faceid_firmware.h"
#include "xrp_hw.h"
#include "xrp_internal.h"
#include "xrp_kernel_defs.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_private_alloc.h"
#include "xvp_main.h"
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>
#include "ion.h"
#include "vdsp_smem.h"
#include "vdsp_ipi_drv.h"
#include "vdsp_trusty.h"
#include "xrp_faceid.h"
/*add temp for set 512M as default clock*/
#include <linux/clk.h>


#define VDSP_FIRMWIRE_SIZE    (1024*1024*14)


#define DRIVER_NAME "xrp"
#define XRP_DEFAULT_TIMEOUT 100

#ifndef __io_virt
#define __io_virt(a) ((void __force *)(a))
#endif

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
static int xrp_faceid_run(struct xvp *xvp,struct xrp_faceid_ctrl *faceid)
{
	unsigned long deadline = jiffies + 5 * HZ;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;
	u32 v;
	__u32 ret = -1,ret2;

	struct faceid_hw_sync_data faceid_data;
	faceid_data.fd_p_coffe_addr = xvp->faceid_pool.ion_fd_weights_p.addr_p[0];
	faceid_data.fd_r_coffe_addr = xvp->faceid_pool.ion_fd_weights_r.addr_p[0];
	faceid_data.fd_o_coffe_addr = xvp->faceid_pool.ion_fd_weights_o.addr_p[0];
	faceid_data.fp_coffe_addr = xvp->faceid_pool.ion_fp_weights.addr_p[0];
	faceid_data.flv_coffe_addr = xvp->faceid_pool.ion_flv_weights.addr_p[0];
	faceid_data.fv_coffe_addr = xvp->faceid_pool.ion_fv_weights.addr_p[0];
	faceid_data.mem_pool_addr = xvp->faceid_pool.ion_fd_mem_pool.addr_p[0];
	faceid_data.yuv_addr = faceid->in_data_addr;
	faceid_data.frame_height = faceid->in_height;
	faceid_data.frame_width = faceid->in_width;
	faceid_data.liveness = faceid->in_liveness;
	faceid_data.transfer_addr = xvp->faceid_pool.ion_face_transfer.addr_p[0];

	ret2 = sprd_iommu_map_faceid_result(xvp,faceid->out_fd);
	if(ret2 < 0)
	{
		printk("iommu map faceid result fail.\n");
		goto err;
	}

	faceid_data.out_addr = xvp->faceid_pool.ion_face_info.iova[0];
	pr_info("fd_p %X,fd_r %X,fd_o %X,\n",faceid_data.fd_p_coffe_addr,faceid_data.fd_r_coffe_addr,faceid_data.fd_o_coffe_addr);
	pr_info("fp %X,flv %X,fv %X\n",faceid_data.fp_coffe_addr,faceid_data.flv_coffe_addr,faceid_data.fv_coffe_addr);
	pr_info("mem pool %X, face info %X, transfer %x, liveness %d\n",faceid_data.mem_pool_addr,faceid_data.out_addr,faceid_data.transfer_addr,faceid_data.liveness);


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
			mb();
			ret = xrp_comm_read32(&shared_sync->hw_sync_data);
			printk("vdsp faceid ret %X\n",ret);
			break;
		}
		if (xrp_panic_check(xvp))
			goto err;
		schedule();
	} while (time_before(jiffies, deadline));

err:
	xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
	sprd_iommu_ummap_faceid_result(xvp);
	faceid->out_result = ret;
	
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
		//printk("yzl add %s v:%d\n" , __func__ , v);
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



static long xrp_ioctl_submit_sync(struct file *filp,
				  struct xrp_ioctl_queue __user *p)
{
	struct xvp_file *xvp_file = filp->private_data;
	struct xvp *xvp = xvp_file->xvp;
	struct xrp_comm *queue = xvp->queue;
	struct xrp_request xrp_rq, *rq = &xrp_rq;
	long ret = 0;
	bool went_off = false;
	enum load_unload_flag load_flag;
	char libname[32];
	bool rebootflag = 0;
	/*int32_t load_count;*/
	//struct loadlib_info *libinfo = NULL;
	int32_t lib_result = 0;
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

	ret = sprd_map_request(filp, rq);
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
			/*check whether libload command and if it is, do load*/
			load_flag = xrp_check_load_unload(xvp , rq);
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

			sprd_fill_hw_request(queue->comm, rq);
			xrp_send_device_irq(xvp);

			if (xvp->host_irq_mode) {
				ret = xvp_complete_cmd_irq(xvp, queue,
							   xrp_cmd_complete);
			} else {
				ret = xvp_complete_cmd_poll(xvp, queue,
							    xrp_cmd_complete);
			}

			xrp_panic_check(xvp);
			/* copy back inline data */
			if (ret == 0) {
				ret = sprd_complete_hw_request(queue->comm, rq);
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
		}
		mutex_unlock(&queue->lock);
	}

	if (ret == 0)
		ret = sprd_unmap_request(filp, rq);
	else if (!went_off)
		sprd_unmap_request(filp, rq);
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

	pr_info("faceid input addr %x,height %d, width %d, liveness %d, out_fd %d\n",faceid.in_data_addr,
								faceid.in_height,faceid.in_width,faceid.in_liveness, faceid.out_fd);
	xrp_faceid_run(xvp,&faceid);

	if (copy_to_user(arg, &faceid, sizeof(struct xrp_faceid_ctrl)))
		return -EFAULT;

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
	xvp->open_count ++;

	return 0;
}

static int xvp_close(struct inode *inode, struct file *filp)
{
	struct xvp_file *xvp_file = filp->private_data;
	int ret = 0;
	printk("%s\n", __func__);

	xrp_remove_known_file(filp);
	pm_runtime_put_sync(xvp_file->xvp->dev);
	xvp_file->xvp->open_count--;
	printk("yzl add %s , open_count is:%d\n" , __func__ , xvp_file->xvp->open_count);
	if(0 == xvp_file->xvp->open_count) {
		/*release xvp load_lib info*/
		ret = xrp_library_release_all(xvp_file->xvp);
	}
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
static int xrp_boot_faceid_firmware(struct xvp *xvp)
{
	int ret;
	struct xrp_dsp_sync_v1 __iomem *shared_sync = xvp->comm;

	if(xvp->tee_con)
	{
		struct vdsp_msg msg;
		msg.vdsp_type = TA_CADENCE_VQ6;
		msg.msg_cmd = TA_FACEID_EXIT_SEC_MODE;
		//vdsp_set_sec_mode(&msg);
	}
	else
	{
		pr_err("Exit secure mode fail\n");
	}
	xrp_halt_dsp(xvp);
	xrp_reset_dsp(xvp);

	pr_info("%s, faceid_fw_viraddr %p , loopback:%d\n" , __func__ , xvp->firmware2_viraddr,loopback);
	if (xvp->firmware2_viraddr) {
		if (loopback < LOOPBACK_NOFIRMWARE) {
			ret = sprd_load_faceid_firmware(xvp);
			if (ret < 0)
				return ret;
		}

		if (loopback < LOOPBACK_NOIO) {
			xrp_comm_write32(&shared_sync->sync, XRP_DSP_SYNC_IDLE);
			mb();
		}
	}

	if(xvp->tee_con)
	{
		struct vdsp_msg msg;
		msg.vdsp_type = TA_CADENCE_VQ6;
		msg.msg_cmd = TA_FACEID_ENTER_SEC_MODE;
		//vdsp_set_sec_mode(&msg);
	}
	else
	{
		pr_err("Enter secure mode fail\n");
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
static int xrp_runtime_resume_normal(struct xvp *xvp)
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
	/*set qos*/
	xvp_set_qos(xvp);

	ret = xrp_boot_firmware(xvp);
	if (ret < 0) {
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
	pr_info("%s enter\n" , __func__);

	for (i = 0; i < xvp->n_queues; ++i)
		mutex_lock(&xvp->queue[i].lock);

	if (xvp->off)
		goto out;
	ret = xvp_enable_dsp(xvp);
	if (ret < 0) {
		dev_err(xvp->dev, "couldn't enable DSP\n");
		goto out;
	}
	ret = sprd_iommu_map_faceid_fwbuffer(xvp);
	if(ret != 0) {
		pr_err("%s map faceid fw failed\n" , __func__);
		goto out;
	}
	ret = sprd_iommu_map_commbuffer(xvp);
	if(ret != 0) {
		sprd_iommu_unmap_faceid_fwbuffer(xvp);
		pr_err("%s map comm buffer failed\n" , __func__);
		goto out;
	}
	ipidesc = get_vdsp_ipi_ctx_desc();
	if(ipidesc) {
		pr_info("%s ipi init called\n" , __func__);
		ipidesc->ops->ctx_init(ipidesc);
	}
	/*set qos*/
	xvp_set_qos(xvp);
	ret = xrp_boot_faceid_firmware(xvp);
	if (ret < 0) {
		ret1 = sprd_iommu_unmap_faceid_fwbuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s unmap faceid fw failed\n" , __func__);
		}
		ret1 = sprd_iommu_unmap_commbuffer(xvp);
		if(ret1 != 0) {
			printk("yzl add %s unmap comm buffer failed\n" , __func__);
		}
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

	xrp_halt_dsp(xvp);
	xrp_reset_dsp(xvp);

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
	int ret;
	struct vdsp_ipi_ctx_desc *ipidesc = NULL;
	struct xvp *xvp = dev_get_drvdata(dev);

	xrp_halt_dsp(xvp);
	if(xvp->secmode)
	{
		ret = sprd_iommu_unmap_faceid_fwbuffer(xvp);
		if(ret != 0) {
			pr_err("yzl add %s unmap faceid fw failed\n" , __func__);
		}
	}
	else
	{
		ret = sprd_iommu_unmap_extrabuffer(xvp);
		if(ret != 0) {
			pr_err("yzl add %s sprd_iommu_unmap_extrabuffer failed\n" , __func__);
		}
		ret = sprd_free_extrabuffer(xvp);
		if(ret != 0) {
			pr_err("yzl add %s sprd_free_extrabuffer failed\n" , __func__);
		}
	}
	ret = sprd_iommu_unmap_commbuffer(xvp);
	if(ret != 0) {
		pr_err("yzl add %s sprd_iommu_unmap_commbuffer failed\n" , __func__);
	}
	xvp_disable_dsp(xvp);
	ipidesc = get_vdsp_ipi_ctx_desc();
	if(ipidesc) {
                printk("yzl add %s ipi deinit called\n" , __func__);
                ipidesc->ops->ctx_deinit(ipidesc);
	}
	if(xvp->secmode)
	{
		xvp->secmode = false;
		if(xvp->tee_con)
		{
			struct vdsp_msg msg;
			msg.vdsp_type = TA_CADENCE_VQ6;
			msg.msg_cmd = TA_FACEID_EXIT_SEC_MODE;
			vdsp_set_sec_mode(&msg);
			vdsp_ca_disconnect();
			xvp->tee_con = false;
		}
	}
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
	mutex_init(&(xvp->load_lib.libload_mutex));
	mutex_init(&map_lock);
	xvp->open_count = 0;
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
	if(xvp->pool != NULL)
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

xrp_init_function xrp_init_sprd;
long xrp_init_sprd(struct platform_device *pdev, enum xrp_init_flags flags,
                  const struct xrp_hw_ops *hw_ops, void *hw_arg)
{
        return xrp_init_common(pdev, flags, hw_ops, hw_arg, xrp_init_regs_sprd);
}
EXPORT_SYMBOL(xrp_init_sprd);

int xrp_deinit(struct platform_device *pdev)
{
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


MODULE_AUTHOR("Takayuki Sugawara");
MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: Linux device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
