

/*
 * xrp_hw_simple: Simple xtensa/arm low-level XRP driver
 *
 * Copyright (c) 2017 Cadence Design Systems, Inc.
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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include "xrp_kernel_defs.h"
#include "xrp_hw.h"
#include "xrp_hw_simple_dsp_interface.h"
#include "vdsp_ipi_drv.h"
#include <linux/clk.h>
#include "xrp_internal.h"
#include "sprd_dvfs_vdsp.h"
#include "vdsp_dvfs_sharkl5pro.h"

#define DRIVER_NAME "sharkl5pro-vdsp"

#define XRP_REG_RESET		(0x04)
#define XRP_REG_RUNSTALL	(0x3084)
#define XRP_REG_LP_CTL          (0x3090)
#define XRP_REG_QOS_THRESHOLD   (0xBC)
#define XRP_REG_QOS_3           (0xD4)
#define XRP_REG_QOS_SEL3        (0xD8)
#define BIT(nr) (1UL << (nr))


#define APAHB_HREG_MWR(reg, msk, val) \
		(REG_WR((reg), \
		((val) & (msk)) | \
		(REG_RD((reg)) & \
		(~(msk)))))

#define APAHB_HREG_OWR(reg, val) \
		(REG_WR((reg), \
		(REG_RD(reg) | (val))))
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "xrp_hw_simple: %d %d %s : "\
        fmt, current->pid, __LINE__, __func__


struct xrp_qos_info {
        uint8_t ar_qos_vdsp_msti;
        uint8_t ar_qos_vdsp_mstd;
        uint8_t aw_qos_vdsp_mstd;
        uint8_t ar_qos_vdsp_idma;
        uint8_t aw_qos_vdsp_idma;
        uint8_t ar_qos_vdma;
        uint8_t aw_qos_vdma;
        uint8_t ar_qos_threshold;
        uint8_t aw_qos_threshold;
};

struct xrp_hw_simple {
	struct xvp *xrp;
	phys_addr_t ahb_phys;
	void __iomem *ahb;
	phys_addr_t clk_phys;
	void __iomem *clk;
	phys_addr_t ipi_phys;
	void __iomem *ipi;
	phys_addr_t pmu_phys;
	void __iomem *pmu;
	phys_addr_t dvfs_phys;
	void __iomem *dvfs;
	/* how IRQ is used to notify the device of incoming data */
	enum xrp_irq_mode device_irq_mode;
	/*
	 * offset of device IRQ register in MMIO region (device side)
	 * bit number
	 * device IRQ#
	 */
	u32 device_irq[3];
	/* offset of devuce IRQ register in MMIO region (host side) */
	u32 device_irq_host_offset;
	/* how IRQ is used to notify the host of incoming data */
	enum xrp_irq_mode host_irq_mode;
	/*
	 * offset of IRQ register (device side)
	 * bit number
	 */
	u32 host_irq[2];

	u32 client_irq;

	struct vdsp_ipi_ctx_desc *vdsp_ipi_desc;
	struct xrp_qos_info qos;
};
static void parse_qos(void *hw_arg , void *of_node)
{
	struct device_node *qos_node = NULL;
	struct xrp_hw_simple *hw = (struct xrp_hw_simple*) hw_arg;
	if((NULL == hw_arg) || (NULL == of_node)) {
		pr_err("yzl add %s hw_arg:%lx , of_node:%lx\n" , __func__ , (unsigned long)hw_arg , (unsigned long)of_node);
		return;
	}
	qos_node = of_parse_phandle(of_node, "vdsp-qos", 0);
	if(qos_node) {
		if (of_property_read_u8(qos_node, "arqos-vdsp-msti", &hw->qos.ar_qos_vdsp_msti)) {
			hw->qos.ar_qos_vdsp_msti = 6;
		}
		if (of_property_read_u8(qos_node, "awqos-vdsp-mstd", &hw->qos.aw_qos_vdsp_mstd)) {
			hw->qos.aw_qos_vdsp_mstd = 6;
		}
		if (of_property_read_u8(qos_node, "arqos-vdsp-mstd", &hw->qos.ar_qos_vdsp_mstd)) {
			hw->qos.ar_qos_vdsp_mstd = 6;
		}
		if (of_property_read_u8(qos_node, "arqos-vdsp-idma", &hw->qos.ar_qos_vdsp_idma)) {
			hw->qos.ar_qos_vdsp_idma = 1;
		}
		if (of_property_read_u8(qos_node, "awqos-vdsp-idma", &hw->qos.aw_qos_vdsp_idma)) {
			hw->qos.aw_qos_vdsp_idma = 1;
		}
		if (of_property_read_u8(qos_node, "arqos-vdma", &hw->qos.ar_qos_vdma)) {
			hw->qos.ar_qos_vdma = 1;
		}
		if (of_property_read_u8(qos_node, "awqos-vdma", &hw->qos.aw_qos_vdma)) {
			hw->qos.aw_qos_vdma = 1;
		}
		if (of_property_read_u8(qos_node, "arqos-threshold", &hw->qos.ar_qos_threshold)) {
			hw->qos.ar_qos_threshold = 0x0f;
		}
		if (of_property_read_u8(qos_node, "awqos-threshold", &hw->qos.aw_qos_threshold)) {
			hw->qos.aw_qos_threshold = 0x0f;
		}
	}
	else {
		hw->qos.ar_qos_vdsp_msti = 6;
		hw->qos.ar_qos_vdsp_mstd = 6;
		hw->qos.aw_qos_vdsp_mstd = 6;
		hw->qos.ar_qos_vdsp_idma = 1;
		hw->qos.aw_qos_vdsp_idma = 1;
		hw->qos.ar_qos_vdma = 1;
		hw->qos.aw_qos_vdma = 1;
		hw->qos.ar_qos_threshold = 0x0f;
		hw->qos.aw_qos_threshold = 0x0f;
	}
	return;
}

static void set_qos(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	/*set qos threshold*/
	APAHB_HREG_MWR(hw->ahb + XRP_REG_QOS_THRESHOLD , (0xf<<28 | 0xf << 24) , ((hw->qos.ar_qos_threshold << 28) | (hw->qos.aw_qos_threshold << 24)));
	/*set qos 3*/
	APAHB_HREG_MWR(hw->ahb + XRP_REG_QOS_3 , 0xf0ffffff , ((hw->qos.ar_qos_vdsp_msti << 28) | (hw->qos.ar_qos_vdsp_mstd << 20)
				| (hw->qos.aw_qos_vdsp_mstd << 16) | (hw->qos.ar_qos_vdsp_idma << 12) | (hw->qos.aw_qos_vdsp_idma << 8)
				| (hw->qos.ar_qos_vdma << 4) | (hw->qos.aw_qos_vdma)));
	/*set qos sel 3*/
	APAHB_HREG_MWR(hw->ahb + XRP_REG_QOS_SEL3 , 0x7f , 0x7f);
	return;
}

static inline void reg_write32_setbit(struct xrp_hw_simple *hw, void *addr, u32 v)
{
        if (hw->ahb)
	{
		volatile u32 value;
		//pr_info("yzl add reg_write32_setbit , reg:%lx\n" , (unsigned long)(hw->ahb + addr));
		value = __raw_readl(addr);
		value = v | value;
                __raw_writel(value, addr);
		//pr_info("yzl add %s read reg:%lx , value:%x\n" , __func__ , (unsigned long)(hw->ahb + addr) , *((unsigned int*)(hw->ahb + addr)));
	}
}
static inline void reg_write32_clearbit(struct xrp_hw_simple *hw, void *addr, u32 v)
{
        if (hw->ahb)
        {
                volatile u32 value;
		//pr_info("yzl add reg_write32_clearbit\n");
                value = __raw_readl(addr);
                value = v & value;
                __raw_writel(value, addr);
		//pr_info("yzl add %s read reg:%lx , value:%x\n" , __func__ , (unsigned long)(hw->ahb + addr) , *((unsigned int*)(hw->ahb + addr)));
        }
}

static inline void reg_write32(struct xrp_hw_simple *hw, void *addr, u32 v)
{
	__raw_writel(v, addr);
}

static inline u32 reg_read32(struct xrp_hw_simple *hw, void *addr)
{
	return __raw_readl(addr);
}

static void *get_hw_sync_data(void *hw_arg, size_t *sz)
{
	static const u32 irq_mode[] = {
		[XRP_IRQ_NONE] = XRP_DSP_SYNC_IRQ_MODE_NONE,
		[XRP_IRQ_LEVEL] = XRP_DSP_SYNC_IRQ_MODE_LEVEL,
		[XRP_IRQ_EDGE] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
		[XRP_IRQ_EDGE_SW] = XRP_DSP_SYNC_IRQ_MODE_EDGE,
	};
	struct xrp_hw_simple *hw = hw_arg;
	struct xrp_hw_simple_sync_data *hw_sync_data =
		kmalloc(sizeof(*hw_sync_data), GFP_KERNEL);

	if (!hw_sync_data)
		return NULL;
	pr_info("yzl add get_hw_sync_data ahb_phys:%x , host_irq_mode:%d,host_irqoffset:%d , host_irq_bit:%d , hw->device_irq_mode:%d , device_irq_mode:%d,device_irq_offset:%d , device_irq_bit:%d , device_irq:%d, smsg addr:0x%lx\n",
	       (unsigned int)hw->ahb_phys , hw->host_irq_mode , hw->host_irq[0] , hw->host_irq[1],hw->device_irq_mode,
	       irq_mode[hw->device_irq_mode] , hw->device_irq[0] ,  hw->device_irq[1],
	       hw->device_irq[2], (unsigned long)*sz);
	*hw_sync_data = (struct xrp_hw_simple_sync_data){
			.device_mmio_base = hw->ipi_phys,
			.host_irq_mode = hw->host_irq_mode,
			.host_irq_offset = hw->host_irq[0],
			.host_irq_bit = hw->host_irq[1],
			.device_irq_mode = irq_mode[hw->device_irq_mode],
			.device_irq_offset = hw->device_irq[0],
			.device_irq_bit = hw->device_irq[1],
			.device_irq = hw->device_irq[2],
			.vdsp_smsg_addr = (unsigned int)*sz,
	};
	pr_info("vdsp_smsg_addr 0x%x \n", hw_sync_data->vdsp_smsg_addr);
	*sz = sizeof(*hw_sync_data);
	return hw_sync_data;
}

static void reset(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	pr_info("yzl add hw_simple %s arg:%p ,offset:%x , value:0\n" , __func__ , hw->ahb , XRP_REG_RESET);
	reg_write32_setbit(hw_arg, hw->ahb+XRP_REG_RESET, (0x3<<9));
	reg_write32_clearbit(hw_arg,hw->ahb+XRP_REG_RESET , ~(0x3<<9));
}

static void halt(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
        pr_info("yzl add hw_simple %s arg:%p ,offset:%x , value:1\n" , __func__ , hw->ahb , XRP_REG_RUNSTALL);
	reg_write32_setbit(hw_arg, hw->ahb+XRP_REG_RUNSTALL, 1<<2);
}

static void release(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	pr_info("yzl add hw_simple %s arg:%p ,offset:%x , value:0\n" , __func__ , hw->ahb , XRP_REG_RUNSTALL);
	reg_write32_clearbit(hw_arg, hw->ahb+XRP_REG_RUNSTALL, ~(1<<2));
}
static int enable(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	unsigned int rdata;
	pr_info("yzl add hw_simple %s arg:%p ,offset:0x7e4 , value:0x204004 , offsett:0xb0, value:0\n" , __func__ , hw->pmu);
	/*pd_ap_vdsp_force_shutdown bit */
	reg_write32(hw_arg, hw->pmu+0x07e4, 0x204004);
	/*vdsp_stop_en*/
	reg_write32_clearbit(hw_arg, hw->ahb+XRP_REG_LP_CTL , ~(1<<2));
	reg_write32_setbit(hw_arg, hw->ahb+XRP_REG_LP_CTL, 1<<3);
	/*isppll open for 936M*/
	reg_write32_setbit(hw_arg, hw->pmu+0x8c, 0x1);
	/* loop PD_AD_VDSP_STATE*/
	rdata = reg_read32(hw_arg, hw->pmu+0xbc);
	rdata &= 0xFF000000;
	while (rdata) {
		rdata = reg_read32(hw_arg, hw->pmu+0xbc);
		rdata &= 0xFF000000;
	}
	/* IPI enable */
	reg_write32_setbit(hw_arg, hw->ahb, (1<<6));
	/* vdma enable */
	reg_write32_setbit(hw_arg, hw->ahb, (1<<3));
	/*vdsp_all_int_mask = 0*/
	reg_write32_clearbit(hw_arg, hw->ahb+0x3094, ~(1<<13));
	return 0;
}
static void disable(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	pr_info("yzl add hw_simple %s arg:%p ,offset:%x , value:0\n" , __func__ , hw->ahb , XRP_REG_LP_CTL);
	reg_write32_setbit(hw_arg, hw->ahb+XRP_REG_LP_CTL , 1<<2);
	reg_write32_setbit(hw_arg, hw->ahb+XRP_REG_RESET , 0x3<<9);
	
}
static void enable_dvfs(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	pr_info("yzl add hw_simple %s arg:%p ,offset:%x , value:0\n" , __func__ , hw->dvfs , 0x8);
	reg_write32_setbit(hw_arg , hw->dvfs + 0x8 , 1<<2);
}
static void disable_dvfs(void *hw_arg)
{
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	reg_write32_clearbit(hw_arg , hw->dvfs +0x8 , ~(1<<2));
}
static uint32_t translate_dvfsindex_to_freq(uint32_t index)
{
	switch(index) {
	case 0:
		return SHARKL5PRO_VDSP_CLK256M;
	case 1:
		return SHARKL5PRO_VDSP_CLK384M;
	case 2:
		return SHARKL5PRO_VDSP_CLK512M;
	case 3:
		return SHARKL5PRO_VDSP_CLK614M4;
	case 4:
		return SHARKL5PRO_VDSP_CLK768M;
	case 5:
		return SHARKL5PRO_VDSP_CLK936M;
	default:
		return SHARKL5PRO_VDSP_CLK256M;
	}
}
static void setdvfs(void *hw_arg , uint32_t index)
{
#if 0
	struct xrp_hw_simple *hw = (struct xrp_hw_simple *)hw_arg;
	pr_info("yzl add hw_simple %s arg:%p , value:0\n" , __func__ , hw->dvfs);
	reg_write32(hw_arg , hw->dvfs + 0x114, index);
#else
	uint32_t freq;
	freq = translate_dvfsindex_to_freq(index);
	pr_info("yzl add %s before vdsp_dvfs_notifier_call_chain freq:%d , index:%d\n" , __func__ , freq ,index);
	vdsp_dvfs_notifier_call_chain(&freq);
	return;
#endif
}
static void send_irq(void *hw_arg)
{
	struct xrp_hw_simple *hw = hw_arg;

	hw->vdsp_ipi_desc->ops->irq_send(hw->device_irq[1]);
#if 0
	switch (hw->device_irq_mode) {
	case XRP_IRQ_EDGE_SW:
		reg_write32(hw, hw->device_irq_host_offset,
			    BIT(hw->device_irq[1]));
		while ((reg_read32(hw, hw->device_irq_host_offset) &
			BIT(hw->device_irq[1])))
			mb();
		break;
	case XRP_IRQ_EDGE:
		reg_write32(hw, hw->device_irq_host_offset, 0);
		/* fallthrough */
	case XRP_IRQ_LEVEL:
		wmb();
		pr_info("yzl add hw_simple %s before reg_write32 device_irq_host_offset:%x,hw->device_irq1:%x\n" ,
		       __func__ ,
		       (unsigned int)hw->device_irq_host_offset ,
		       (unsigned int)(BIT(hw->device_irq[1])));

		reg_write32_setbit(hw,
				   hw->device_irq_host_offset + hw->ipi,
				   /*0xF0*/ BIT(hw->device_irq[1]));
		break;
	default:
		break;
	}
#endif
}
static void ack_irq(void *hw_arg)
{
	struct xrp_hw_simple *hw = hw_arg;
	//pr_info("yzl add %s host_irq_mode:%d,hw->host_irq0:%x\n" , __func__ , hw->host_irq_mode , hw->host_irq[0]);
	if (hw->host_irq_mode == XRP_IRQ_LEVEL) {
	//	reg_write32_setbit(hw, hw->host_irq[0] + hw->ipi, hw->client_irq & 0xF);

		pr_info("Changdou clr = 0x%x\n", reg_read32(hw, hw->device_irq_host_offset + hw->ipi) & 0xF);
	}
}


static irqreturn_t xrp_hw_irq_handler(int irq, void *dev_id)
{
	irqreturn_t ret = 0;
	struct xrp_hw_simple *hw = dev_id;

	pr_info("Changdou clr = 0x%x\n", reg_read32(hw, hw->ipi) & 0xF);
	ret = xrp_irq_handler(irq, hw->xrp);

	if (ret == IRQ_HANDLED)
		ack_irq(hw);

	return ret;
}


#if defined(__XTENSA__)
static bool cacheable(void *hw_arg, unsigned long pfn, unsigned long n_pages)
{
	return true;
}

static void dma_sync_for_device(void *hw_arg,
				void *vaddr, phys_addr_t paddr,
				unsigned long sz, unsigned flags)
{
	switch (flags) {
	case XRP_FLAG_READ:
		__flush_dcache_range((unsigned long)vaddr, sz);
		break;

	case XRP_FLAG_READ_WRITE:
		__flush_dcache_range((unsigned long)vaddr, sz);
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;

	case XRP_FLAG_WRITE:
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;
	}
}

static void dma_sync_for_cpu(void *hw_arg,
			     void *vaddr, phys_addr_t paddr,
			     unsigned long sz, unsigned flags)
{
	switch (flags) {
	case XRP_FLAG_READ_WRITE:
	case XRP_FLAG_WRITE:
		__invalidate_dcache_range((unsigned long)vaddr, sz);
		break;
	}
}

#elif defined(__arm__)
static bool cacheable(void *hw_arg, unsigned long pfn, unsigned long n_pages)
{
	return true;
}

static void dma_sync_for_device(void *hw_arg,
				void *vaddr, phys_addr_t paddr,
				unsigned long sz, unsigned flags)
{
	switch (flags) {
	case XRP_FLAG_READ:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_clean_range(paddr, paddr + sz);
		break;

	case XRP_FLAG_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_inv_range(paddr, paddr + sz);
		break;

	case XRP_FLAG_READ_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_flush_range(paddr, paddr + sz);
		break;
	}
}

static void dma_sync_for_cpu(void *hw_arg,
			     void *vaddr, phys_addr_t paddr,
			     unsigned long sz, unsigned flags)
{
	switch (flags) {
	case XRP_FLAG_WRITE:
	case XRP_FLAG_READ_WRITE:
		__cpuc_flush_dcache_area(vaddr, sz);
		outer_inv_range(paddr, paddr + sz);
		break;
	}
}
#endif
static void memcpy_hw_function(void __iomem *dst, const void *src, size_t sz)
{
	memcpy(dst , src , sz);
	return;
}
static void memset_hw_function(void __iomem *dst, int c, size_t sz)
{
	memset(dst , c , sz);
	return;
}
static const struct xrp_hw_ops hw_ops = {
	.halt = halt,
	.release = release,
	.reset = reset,

	.get_hw_sync_data = get_hw_sync_data,

	.send_irq = send_irq,

#if defined(__XTENSA__) || defined(__arm__)
	.cacheable = cacheable,
	.dma_sync_for_device = dma_sync_for_device,
	.dma_sync_for_cpu = dma_sync_for_cpu,
#endif
	.memcpy_tohw = memcpy_hw_function,
	.memset_hw = memset_hw_function,
	.enable = enable,
	.disable = disable,
	.enable_dvfs = enable_dvfs,
	.disable_dvfs = disable_dvfs,
	.setdvfs = setdvfs,
	.parse_qos = parse_qos,
	.set_qos = set_qos,
};

static long init_hw(struct platform_device *pdev, struct xrp_hw_simple *hw,
		    int mem_idx, enum xrp_init_flags *init_flags)
{
	struct resource *mem;
	int irq;
	long ret;

	/*ahb */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, mem_idx);
	pr_info("yzl add init_hw  ahb memstart:%lx , mem:%p , mem_idx:%d\n" , (unsigned long)mem->start , mem , mem_idx);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}
	hw->ahb_phys = mem->start;
	hw->ahb = devm_ioremap_resource(&pdev->dev, mem);
	pr_info("yzl %s: ahb = %pap/%p\n",
		 __func__, &mem->start, hw->ahb);
	/*ipi */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, mem_idx+1);
	pr_info("yzl add init_hw ipi memstart:%lx , mem:%p , mem_idx:%d\n" , (unsigned long)mem->start , mem , mem_idx+1);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}
	hw->ipi_phys = mem->start;
	hw->ipi = devm_ioremap_resource(&pdev->dev, mem);
	pr_info("yzl %s: ipi = %pap/%p\n",
		 __func__, &mem->start, hw->ipi);
	/*pmu 0x327e0000*/
	mem = platform_get_resource(pdev, IORESOURCE_MEM, mem_idx+2);
	pr_info("yzl add init_hw pmu memstart:%lx, mem:%p , mem_idx:%d\n" ,(unsigned long)mem->start , mem , mem_idx+2);
        if (!mem) {
                ret = -ENODEV;
                goto err;
        }
	hw->pmu = devm_ioremap_resource(&pdev->dev, mem);
	pr_info("yzl %s: pmu = %pap/%p\n",
                 __func__, &mem->start, hw->pmu);
	/*dvfs */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, mem_idx+3);
	pr_info("yzl add init_hw dvfs memstart:%lx , mem:%p , mem_idx:%d\n" , (unsigned long)mem->start , mem , mem_idx+4);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}
	hw->dvfs_phys = mem->start;
	hw->dvfs = devm_ioremap_resource(&pdev->dev, mem);
	pr_info("yzl %s: dvfs = %pap/%p\n",
		 __func__, &mem->start, hw->dvfs);

	ret = of_property_read_u32_array(pdev->dev.of_node,
					 "device-irq",
					 hw->device_irq,
					 ARRAY_SIZE(hw->device_irq));
	if (ret == 0) {
		u32 device_irq_host_offset;

		ret = of_property_read_u32(pdev->dev.of_node,
					   "device-irq-host-offset",
					   &device_irq_host_offset);
		if (ret == 0) {
			hw->device_irq_host_offset = device_irq_host_offset;
		} else {
			hw->device_irq_host_offset = hw->device_irq[0];
			ret = 0;
		}
	}
	if (ret == 0) {
		u32 device_irq_mode;

		ret = of_property_read_u32(pdev->dev.of_node,
					   "device-irq-mode",
					   &device_irq_mode);
		if (device_irq_mode < XRP_IRQ_MAX)
			hw->device_irq_mode = device_irq_mode;
		else
			ret = -ENOENT;
	}
	if (ret == 0) {
		dev_dbg(&pdev->dev,
			"%s: device IRQ MMIO host offset = 0x%08x, offset = 0x%08x, bit = %d, device IRQ = %d, IRQ mode = %d",
			__func__, hw->device_irq_host_offset,
			hw->device_irq[0], hw->device_irq[1],
			hw->device_irq[2], hw->device_irq_mode);
	} else {
		dev_info(&pdev->dev,
			 "using polling mode on the device side\n");
	}

	ret = of_property_read_u32_array(pdev->dev.of_node, "host-irq",
					 hw->host_irq,
					 ARRAY_SIZE(hw->host_irq));
	if (ret == 0) {
		u32 host_irq_mode;

		ret = of_property_read_u32(pdev->dev.of_node,
					   "host-irq-mode",
					   &host_irq_mode);
		if (host_irq_mode < XRP_IRQ_MAX)
			hw->host_irq_mode = host_irq_mode;
		else
			ret = -ENOENT;
	}

	if (ret == 0 && hw->host_irq_mode != XRP_IRQ_NONE)
		irq = platform_get_irq(pdev, 0);
	else
		irq = -1;

	pr_info("yzl add %s , irq is:%d , ret:%ld , host_irq_mode:%d\n" , __func__ , irq , ret , hw->host_irq_mode);
	if (irq >= 0) {
		dev_dbg(&pdev->dev, "%s: host IRQ = %d, ",
			__func__, irq);
		pr_info("yzl add %s: host IRQ = %d, \n",
                        __func__, irq);

		hw->vdsp_ipi_desc = get_vdsp_ipi_ctx_desc();
		if (hw->vdsp_ipi_desc) {
			hw->vdsp_ipi_desc->base_addr = hw->ahb_phys;
			hw->vdsp_ipi_desc->vir_addr = hw->ahb;
			hw->vdsp_ipi_desc->ipi_addr = hw->ipi;
			hw->vdsp_ipi_desc->irq_mode = hw->host_irq_mode;
			ret = devm_request_irq(&pdev->dev,
					       irq,
					       hw->vdsp_ipi_desc->ops->irq_handler,
					       IRQF_SHARED,
					       pdev->name,
					       hw);

			if (ret < 0) {
				dev_err(&pdev->dev, "request_irq %d failed\n", irq);
				goto err;
			}

			hw->vdsp_ipi_desc->ops->irq_register(0, xrp_hw_irq_handler, hw);
			hw->vdsp_ipi_desc->ops->irq_register(1, xrp_hw_irq_handler, hw);

			*init_flags |= XRP_INIT_USE_HOST_IRQ;
		}
	} else {
		dev_info(&pdev->dev, "using polling mode on the host side\n");
	}
	ret = 0;
err:
	return ret;
}

static long init(struct platform_device *pdev, struct xrp_hw_simple *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;

	ret = init_hw(pdev, hw, 0, &init_flags);
	if (ret < 0)
		return ret;

	return xrp_init(pdev, init_flags, &hw_ops, hw);
}

static long init_v1(struct platform_device *pdev, struct xrp_hw_simple *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;

	ret = init_hw(pdev, hw, 1, &init_flags);
	if (ret < 0)
		return ret;

	return xrp_init_v1(pdev, init_flags, &hw_ops, hw);
}

static long init_cma(struct platform_device *pdev, struct xrp_hw_simple *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;

	ret = init_hw(pdev, hw, 0, &init_flags);
	if (ret < 0)
		return ret;

	return xrp_init_cma(pdev, init_flags, &hw_ops, hw);
}
static long init_sprd(struct platform_device *pdev, struct xrp_hw_simple *hw)
{
	long ret;
	enum xrp_init_flags init_flags = 0;
	ret = init_hw(pdev, hw, 0, &init_flags);
	if (ret < 0)
		return ret;
	return xrp_init_sprd(pdev, init_flags, &hw_ops, hw);
}
#ifdef CONFIG_OF
static const struct of_device_id xrp_hw_simple_match[] = {
	{
		.compatible = "cdns,xrp-hw-simple",
		.data = init,
	}, {
		.compatible = "cdns,xrp-hw-simple,v1",
		.data = init_v1,
	}, {
		.compatible = "cdns,xrp-hw-simple,cma",
		.data = init_cma,
	},
	{
		.compatible = "sprd,sharkl5pro-vdsp",
		.data = init_sprd,
	},
	 {},
};
MODULE_DEVICE_TABLE(of, xrp_hw_simple_match);
#endif

static int xrp_hw_simple_probe(struct platform_device *pdev)
{
	struct xrp_hw_simple *hw =
		devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	const struct of_device_id *match;
	long (*init)(struct platform_device *pdev, struct xrp_hw_simple *hw);
	long ret;

	if (!hw)
		return -ENOMEM;

	match = of_match_device(of_match_ptr(xrp_hw_simple_match),
				&pdev->dev);
	if (!match)
		return -ENODEV;

	init = match->data;
	ret = init(pdev, hw);
	if (IS_ERR_VALUE(ret)) {
		xrp_deinit(pdev);
		return ret;
	} else {
		hw->xrp = ERR_PTR(ret);
		return 0;
	}

}

static int xrp_hw_simple_remove(struct platform_device *pdev)
{
	return xrp_deinit(pdev);
}

static const struct dev_pm_ops xrp_hw_simple_pm_ops = {
	SET_RUNTIME_PM_OPS(xrp_runtime_suspend,
			   xrp_runtime_resume, NULL)
};

static struct platform_driver xrp_hw_simple_driver = {
	.probe   = xrp_hw_simple_probe,
	.remove  = xrp_hw_simple_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(xrp_hw_simple_match),
		.pm = &xrp_hw_simple_pm_ops,
	},
};

module_platform_driver(xrp_hw_simple_driver);

MODULE_AUTHOR("Max Filippov");
MODULE_DESCRIPTION("XRP: low level device driver for Xtensa Remote Processing");
MODULE_LICENSE("Dual MIT/GPL");
