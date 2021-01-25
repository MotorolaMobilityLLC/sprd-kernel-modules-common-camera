/*
 * Copyright (C) 2020-2021 UNISOC Communications Inc.
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include "dcam_int.h"
#include "dcam_path.h"
#include "dcam_reg.h"
#include <sprd_mm.h>

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "DCAM_DRV: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

unsigned long g_dcam_regbase[DCAM_ID_MAX];
unsigned long g_dcam_aximbase[DCAM_ID_MAX];
unsigned long g_dcam_phys_base[DCAM_ID_MAX];
unsigned long g_dcam_mmubase;
unsigned long g_dcam_fmcubase;

/*
 * Initialize dcam_if hardware, power/clk/int should be prepared after this call
 * returns. It also brings the dcam_pipe_dev from INIT state to IDLE state.
 */
int dcam_drv_hw_init(void *arg)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (unlikely(!arg)) {
		pr_err("fail to get invalid arg\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)arg;
	hw = dev->hw;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	ret = sprd_cam_pw_on();
	ret = sprd_cam_domain_eb();
#endif
	/* prepare clk */
	hw->dcam_ioctl(hw, DCAM_HW_CFG_PW_ON, NULL);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_ENABLE_CLK, NULL);
	return ret;
}

/*
 * De-initialize dcam_if hardware thus power/clk/int resource can be released.
 * Registers will be inaccessible and dcam_pipe_dev will enter INIT state from
 * IDLE state.
 */
int dcam_drv_hw_deinit(void *arg)
{
	int ret = 0;
	struct dcam_pipe_dev *dev = NULL;
	struct cam_hw_info *hw = NULL;

	if (unlikely(!arg)) {
		pr_err("fail to get invalid arg\n");
		return -EINVAL;
	}

	dev = (struct dcam_pipe_dev *)arg;
	hw = dev->hw;
	/* unprepare clk and other resource */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	hw->dcam_ioctl(hw, DCAM_HW_CFG_DISABLE_CLK, NULL);
	hw->dcam_ioctl(hw, DCAM_HW_CFG_PW_OFF, NULL);
	ret = sprd_cam_domain_disable();
	ret = sprd_cam_pw_off();
#endif
	return ret;
}

int dcam_drv_dt_parse(struct platform_device *pdev,
			struct cam_hw_info *hw_info,
			uint32_t *dcam_count)
{
	struct cam_hw_soc_info *soc_dcam = NULL;
	struct cam_hw_ip_info *ip_dcam = NULL;
	struct device_node *dn = NULL;
	struct device_node *qos_node = NULL;
	struct device_node *iommu_node = NULL;
	struct regmap *ahb_map = NULL;
	void __iomem *reg_base = NULL;
	struct resource reg_res = {0}, irq_res = {0};
	uint32_t count = 0, prj_id = 0;
	uint32_t dcam_max_w = 0, dcam_max_h = 0;
	int i = 0, irq = 0;
	uint32_t args[2];
	char dcam_name[20];
	int ret = 0;

	pr_info("start dcam dts parse\n");

	if (!pdev || !hw_info) {
		pr_err("fail to get pdev %p hw info %p\n", pdev, hw_info);
		return -EINVAL;
	}

	dn = pdev->dev.of_node;
	if (unlikely(!dn)) {
		pr_err("fail to get a valid device node\n");
		return -EINVAL;
	}

	ahb_map = syscon_regmap_lookup_by_phandle(dn, "sprd,cam-ahb-syscon");
	if (IS_ERR_OR_NULL(ahb_map)) {
		pr_err("fail to get sprd,cam-ahb-syscon\n");
		return PTR_ERR(ahb_map);
	}

	if (of_property_read_u32(dn, "sprd,dcam-count", &count)) {
		pr_err("fail to parse the property of sprd,dcam-count\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dn, "sprd,project-id", &prj_id))
		pr_info("fail to parse the property of sprd,projectj-id\n");

	/* bounded kernel device node */
	hw_info->pdev = pdev;
	hw_info->prj_id = (enum cam_prj_id) prj_id;

	dcam_max_w = DCAM_PATH_WMAX;
	dcam_max_h = DCAM_PATH_HMAX;
	if (prj_id == ROC1) {
		dcam_max_w = DCAM_PATH_WMAX_ROC1;
		dcam_max_h = DCAM_PATH_HMAX_ROC1;
	}

	if (count > DCAM_ID_MAX) {
		pr_err("fail to get a valid dcam count, count: %u\n", count);
		return -EINVAL;
	}

	pr_info("dev: %s, full name: %s, cam_ahb_gpr: %px, count: %u, DCAM dcam_max.w.h %u %u\n",
		pdev->name, dn->full_name, ahb_map, count, dcam_max_w, dcam_max_h);

	iommu_node = of_parse_phandle(dn, "iommus", 0);
	if (iommu_node) {
		reg_base = of_iomap(iommu_node, 0);
		if (!reg_base)
			pr_err("fail to map DCAM IOMMU base\n");
		else
			g_dcam_mmubase = (unsigned long)reg_base;
	}
	pr_info("DCAM IOMMU Base  0x%lx\n", g_dcam_mmubase);

	/* Start dcam soc related dt parse */
	soc_dcam = hw_info->soc_dcam;
	soc_dcam->pdev = pdev;
	/* AHB bus register mapping */
	soc_dcam->cam_ahb_gpr = ahb_map;
	/* qos dt parse */
	qos_node = of_parse_phandle(dn, "dcam_qos", 0);
	if (qos_node) {
		uint8_t val;

		if (of_property_read_u8(qos_node, "awqos-high", &val)) {
			pr_warn("isp awqos-high reading fail.\n");
			val = 0xD;
		}
		soc_dcam->awqos_high = (uint32_t)val;

		if (of_property_read_u8(qos_node, "awqos-low", &val)) {
			pr_warn("isp awqos-low reading fail.\n");
			val = 0xA;
		}
		soc_dcam->awqos_low = (uint32_t)val;

		if (of_property_read_u8(qos_node, "arqos", &val)) {
			pr_warn("isp arqos-high reading fail.\n");
			val = 0xA;
		}
		soc_dcam->arqos_high = val;
		soc_dcam->arqos_low = val;

		pr_info("get dcam qos node. r: %d %d w: %d %d\n",
			soc_dcam->arqos_high, soc_dcam->arqos_low,
			soc_dcam->awqos_high, soc_dcam->awqos_low);
	} else {
		soc_dcam->awqos_high = 0xD;
		soc_dcam->awqos_low = 0xA;
		soc_dcam->arqos_high = 0xA;
		soc_dcam->arqos_low = 0xA;
	}

	/* read dcam clk */
#ifndef CAM_ON_HAPS

	ret = hw_info->cam_ioctl(hw_info, CAM_HW_GET_DCAM_DTS_CLK, dn);
	if (ret)
		goto err_iounmap;

#endif

	for (i = 0; i < count; i++) {
		ip_dcam = hw_info->ip_dcam[i];
		/* DCAM index */
		ip_dcam->idx = i;
		/* Assign project ID, DCAM Max Height & Width Info */
		ip_dcam->max_width = dcam_max_w;
		ip_dcam->max_height = dcam_max_h;

		/* irq */
		irq = of_irq_to_resource(dn, i, &irq_res);
		if (irq <= 0) {
			pr_err("fail to get DCAM%d irq, error: %d\n", i, irq);
			goto err_iounmap;
		}
		ip_dcam->irq_no = (uint32_t) irq;

		/* DCAM register mapping */
		if (of_address_to_resource(dn, i, &reg_res)) {
			pr_err("fail to get DCAM%d phy addr\n", i);
			goto err_iounmap;
		}
		ip_dcam->phy_base = (unsigned long) reg_res.start;
		g_dcam_phys_base[i] = ip_dcam->phy_base;
		pr_debug("dcam phys reg base is %lx\n", g_dcam_phys_base[0]);
		reg_base = ioremap(reg_res.start, reg_res.end - reg_res.start + 1);
		if (!reg_base) {
			pr_err("fail to map DCAM%d reg base\n", i);
			goto err_iounmap;
		}
		ip_dcam->reg_base = (unsigned long) reg_base;
		g_dcam_regbase[i] = (unsigned long)reg_base;

		pr_info("DCAM%d reg: %s 0x%lx %lx, irq: %s %u\n", i,
			reg_res.name, ip_dcam->phy_base, ip_dcam->reg_base,
			irq_res.name, ip_dcam->irq_no);

		sprintf(dcam_name, "dcam%d_reset", i);
		if (!cam_syscon_get_args_by_name(dn, dcam_name,
			ARRAY_SIZE(args), args)) {
			ip_dcam->syscon.rst = args[0];
			ip_dcam->syscon.rst_mask = args[1];
		} else {
			pr_err("fail to get dcam%d reset syscon\n", i);
			goto err_iounmap;
		}
	}

	ret = hw_info->cam_ioctl(hw_info, CAM_HW_GET_ALL_RST, dn);
	if (ret)
		return -1;

	ret = hw_info->cam_ioctl(hw_info, CAM_HW_GET_AXI_BASE, dn);
	if (ret)
		goto err_iounmap;
	*dcam_count = count;

	return 0;

err_iounmap:
	for (i = i - 1; i >= 0; i--)
		iounmap((void __iomem *)(hw_info->ip_dcam[0]->reg_base));
	iounmap((void __iomem *)g_dcam_mmubase);
	g_dcam_mmubase = 0;

	return -ENXIO;
}
