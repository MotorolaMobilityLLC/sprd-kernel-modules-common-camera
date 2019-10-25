/*
 * Copyright (C) 2017-2018 Spreadtrum Communications Inc.
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


#include <linux/delay.h>
#include "vdsp_ipi_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "VDSP_IPI %d: %d %s:" \
	fmt, current->pid, __LINE__, __func__

static irqreturn_t irq_handler(int irq, void *arg);
static int vdsp_ipi_reg_irq_handle(int idx, irq_handler_t handler, void *arg);
static int vdsp_ipi_unreg_irq_handle(int idx);
static int vdsp_ipi_send_irq(int idx);
static int vdsp_ipi_ctx_init(struct vdsp_ipi_ctx_desc * ctx);
static int vdsp_ipi_ctx_deinit(struct vdsp_ipi_ctx_desc * ctx);

struct vdsp_ipi_ops ipi_ops = {
	.ctx_init =vdsp_ipi_ctx_init,
	.ctx_deinit = vdsp_ipi_ctx_deinit,
	.irq_handler = irq_handler,
	.irq_register = vdsp_ipi_reg_irq_handle,
	.irq_unregister = vdsp_ipi_unreg_irq_handle,
	.irq_send = vdsp_ipi_send_irq,
//	.irq_clear = ,
};

static struct vdsp_ipi_ctx_desc s_ipi_desc = {
	.ops = &ipi_ops,
};


//static irqreturn_t xrp_hw_irq_handler(int , void *);
static irq_handler_t ipi_isr_handler[IPI_IDX_MAX] = {
//	[IPI_IDX_0] = xrp_hw_irq_handler,
//	[IPI_IDX_1] = xrp_hw_irq_handler,
//	[IPI_IDX_2] = NULL,
//	[IPI_IDX_3] = NULL,
};

static void *ipi_isr_param[IPI_IDX_MAX] = {
	[IPI_IDX_0] = NULL,
	[IPI_IDX_1] = NULL,
	[IPI_IDX_2] = NULL,
	[IPI_IDX_3] = NULL,
};

static int vdsp_ipi_reg_irq_handle(int idx,
		irq_handler_t handler, void *param)
{
	pr_info("vdsp register handler[%d] = 0x%p, param = 0x%p\n",
		idx, handler, param);
	ipi_isr_handler[idx] = handler;
	ipi_isr_param[idx] = param;

	return 0;
}

static int vdsp_ipi_unreg_irq_handle(int idx)
{
	ipi_isr_handler[idx] = NULL;
	ipi_isr_param[idx] = NULL;

	return 0;
}

static int vdsp_ipi_send_irq(int idx)
{
	switch (s_ipi_desc.irq_mode) {
	case XRP_IRQ_EDGE_SW:
#if 0
		reg_write32(hw, hw->device_irq_host_offset,
			    BIT(hw->device_irq[1]));
		while ((reg_read32(hw, hw->device_irq_host_offset) &
			BIT(hw->device_irq[1])))
			mb();
#endif
		break;
	case XRP_IRQ_EDGE:
#if 0
		reg_write32(hw, hw->device_irq_host_offset, 0);
#endif
		/* fallthrough */
	case XRP_IRQ_LEVEL:
		wmb();
		pr_info("device_irq_host_offset,hw->device_irq:%x\n" , idx);
#if 0
		reg_write32_setbit(hw,
				   hw->device_irq_host_offset + hw->ipi,
				   0x1 << idx);
		volatile u32 value;
		value = __raw_readl(hw->regs + addr);
		value = v | value;
                __raw_writel(value, hw->regs + addr);
#endif
		if (s_ipi_desc.ipi_addr) {
			pr_info("s_ipi_desc.ipi_addr 0x%p\n", s_ipi_desc.ipi_addr);
		IPI_HREG_OWR(s_ipi_desc.ipi_addr, 0x1 << idx);
		} else
			pr_info("s_ipi_desc.vir_addr 0x%p\n", s_ipi_desc.vir_addr);
		break;
	default:
		break;
	}

	return 0;
}

static irqreturn_t irq_handler(int irq, void *arg)
{
	irqreturn_t ret = 0;
	int irq_val = 0;
	int irq_mask = 0;
	int i = 0;

	irq_val = IPI_HREG_RD(s_ipi_desc.ipi_addr) & 0xF;
	irq_mask = irq_val;
	pr_info("irq_handler reg val = 0x%x\n", irq_mask);

	/* clear the interrupt */
	IPI_HREG_OWR((s_ipi_desc.ipi_addr + 8), irq_val & 0xF);

	/* dispatch the interrupt */
	for (i = 0; i < IPI_IDX_MAX; i++) {
		if (irq_mask & (1 << i)) {
			pr_info("irq_handler ipi %d triggle\n", i);
			if (ipi_isr_handler[i]) {
				ret = ipi_isr_handler[i](irq, ipi_isr_param[i]);
			} else {
				pr_info("no irq function\n");
			}
		}
		irq_val  &= ~(1 << i);
		if (!irq_val)
			break;
	}

	return ret;
}

static int vdsp_ipi_ctx_init(struct vdsp_ipi_ctx_desc * ctx)
{
	IPI_HREG_OWR(ctx->vir_addr, 0x1 << 6);
	udelay(1);
	IPI_HREG_WR((ctx->vir_addr + 0x3094), 0xd85f);
	IPI_HREG_OWR((ctx->ipi_addr +8), 0xFF);

	return 0;
}


static int vdsp_ipi_ctx_deinit(struct vdsp_ipi_ctx_desc * ctx)
{
	IPI_HREG_OWR((ctx->ipi_addr +8), 0xFF);
	IPI_HREG_WR(ctx->vir_addr , (IPI_HREG_RD(ctx->vir_addr) & (~(0x1<<6))));
	IPI_HREG_WR((ctx->vir_addr + 0x3094), 0x1ffff);
	return 0;
}


struct vdsp_ipi_ctx_desc *get_vdsp_ipi_ctx_desc(void)
{
	return &s_ipi_desc;
}
EXPORT_SYMBOL_GPL(get_vdsp_ipi_ctx_desc);
