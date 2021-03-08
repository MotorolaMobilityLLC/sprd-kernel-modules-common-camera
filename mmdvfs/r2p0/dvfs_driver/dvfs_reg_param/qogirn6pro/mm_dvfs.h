/*
 * Copyright (C) 2019 Unisoc Communications Inc.
 *
 * This file is dual-licensed: you can use it either under the terms
 * of the GPL or the X11 license, at your option. Note that this dual
 * licensing only applies to this file, and not this project as a
 * whole.
 *
 ********************************************************************
 * Auto generated c code from ASIC Documentation, PLEASE DONOT EDIT *
 ********************************************************************
 */

#ifndef __MM_DVFS_REG_H___
#define __MM_DVFS_REG_H___

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/of.h>
//#include <video/sprd_mm.h>
#include <sprd_mm.h>
#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/sprd_iommu.h>
#include <linux/sprd_ion.h>

#include "mm_dvfs_reg.h"
#include "top_dvfs_reg.h"

enum {
	VOLT60 = 1, //0.60
	VOLT65, //0.65
	VOLT70, //0.7
	VOLT75, //0.75
};

enum {
	CPP_CLK1280 = 128000000, //128
	CPP_CLK1920 = 192000000, //192
	CPP_CLK2560 = 256000000, //256
	CPP_CLK3072 = 307200000, //307.2
	CPP_CLK3840 = 384000000, //384
};

enum {
	ISP_CLK1536 = 153600000, //153.6
	ISP_CLK2560 = 256000000, //256
	ISP_CLK3072 = 307200000, //307.2
	ISP_CLK4096 = 409600000, //409.6
	ISP_CLK5120 = 512000000, //512
};

enum {
	DEPTH_CLK1280 = 128000000, //128
	DEPTH_CLK1920 = 192000000, //192
	DEPTH_CLK2560 = 256000000, //256
	DEPTH_CLK3072 = 307200000, //307.2
	DEPTH_CLK3840 = 384000000, //384
};

enum {
	JPG_CLK1536 = 153600000, //153.6
	JPG_CLK2560 = 256000000, //256
	JPG_CLK3072 = 307200000, //307.2
	JPG_CLK4096 = 409600000, //409.6
	JPG_CLK5120 = 512000000, //512
};

enum {
	FD_CLK1280 = 128000000, //128
	FD_CLK1920 = 192000000, //192
	FD_CLK2560 = 256000000, //256
	FD_CLK3072 = 307200000, //307.2
	FD_CLK3840 = 384000000, //384
};

enum {
	MM_MTX_DATA_CLK1536 = 153600000, //153.6
	MM_MTX_DATA_CLK2560 = 256000000, //256
	MM_MTX_DATA_CLK3072 = 307200000, //307.2
	MM_MTX_DATA_CLK4096 = 409600000, //409.6
	MM_MTX_DATA_CLK5120 = 512000000, //512
};

enum {
	DCAM0_1_CLK1536 = 153600000, //153.6
	DCAM0_1_CLK2560 = 256000000, //2560
	DCAM0_1_CLK3072 = 307200000, //307.2
	DCAM0_1_CLK4096 = 409600000, //409.6
	DCAM0_1_CLK5120 = 512000000, //512
};

enum {
	DCAM0_1_AXI_CLK1536 = 153600000, //153.6
	DCAM0_1_AXI_CLK2560 = 256000000, //2560
	DCAM0_1_AXI_CLK3072 = 307200000, //307.2
	DCAM0_1_AXI_CLK4096 = 409600000, //409.6
	DCAM0_1_AXI_CLK5120 = 512000000, //512
};

enum {
	DCAM2_3_CLK960  = 96000000,  //96
	DCAM2_3_CLK1280 = 128000000, //128
	DCAM2_3_CLK1536 = 153600000, //153.6
	DCAM2_3_CLK1920 = 192000000, //192
	DCAM2_3_CLK2560 = 256000000, //256
};

enum {
	DCAM2_3_AXI_CLK960  = 96000000,  //96
	DCAM2_3_AXI_CLK1280 = 128000000, //128
	DCAM2_3_AXI_CLK1536 = 153600000, //153.6
	DCAM2_3_AXI_CLK1920 = 192000000, //192
	DCAM2_3_AXI_CLK2560 = 256000000, //256
};

enum {
	DCAM_MTX_CLK1536 = 153600000, //153.6
	DCAM_MTX_CLK3072 = 307200000, //307.2
	DCAM_MTX_CLK4096 = 409600000, //409.6
	DCAM_MTX_CLK5120 = 512000000, //512
};

enum {
	CPP_CLK_INDEX_1280 = 0, //128
	CPP_CLK_INDEX_1920 = 1, //192
	CPP_CLK_INDEX_2560 = 2, //256
	CPP_CLK_INDEX_3072 = 3, //307.2
	CPP_CLK_INDEX_3840 = 4, //384
};

enum {
	ISP_CLK_INDEX_1536 = 0, //153.6
	ISP_CLK_INDEX_2560 = 1, //256
	ISP_CLK_INDEX_3072 = 2, //307.2
	ISP_CLK_INDEX_4096 = 3, //409.6
	ISP_CLK_INDEX_5120 = 4, //512
};

enum {
	JPG_CLK_INDEX_1536 = 0, //153.6
	JPG_CLK_INDEX_2560 = 1, //256
	JPG_CLK_INDEX_3072 = 2, //307.2
	JPG_CLK_INDEX_4096 = 3, //409.6
	JPG_CLK_INDEX_5120 = 4, //512
};

enum {
	FD_CLK_INDEX_1280 = 0, //128
	FD_CLK_INDEX_1920 = 1, //192
	FD_CLK_INDEX_2560 = 2, //256
	FD_CLK_INDEX_3072 = 3, //307.2
	FD_CLK_INDEX_3840 = 4, //384
};

enum {
	DEPTH_CLK_INDEX_1280 = 0, //128
	DEPTH_CLK_INDEX_1920 = 1, //192
	DEPTH_CLK_INDEX_2560 = 2, //256
	DEPTH_CLK_INDEX_3072 = 3, //307.2
	DEPTH_CLK_INDEX_3840 = 4, //384
};

enum {
	MM_MTX_DATA_CLK_INDEX_1536 = 0, //153.6
	MM_MTX_DATA_CLK_INDEX_2560 = 1, //256
	MM_MTX_DATA_CLK_INDEX_3072 = 2, //307.2
	MM_MTX_DATA_CLK_INDEX_4096 = 3, //409.6
	MM_MTX_DATA_CLK_INDEX_5120 = 4, //512
};

enum {
	DCAM0_1_CLK_INDEX_1536 = 0, //153.6
	DCAM0_1_CLK_INDEX_2560 = 1, //2560
	DCAM0_1_CLK_INDEX_3072 = 2, //307.2
	DCAM0_1_CLK_INDEX_4096 = 3, //409.6
	DCAM0_1_CLK_INDEX_5120 = 4, //512
};

enum {
	DCAM0_1_AXI_CLK_INDEX_1536 = 0, //153.6
	DCAM0_1_AXI_CLK_INDEX_2560 = 1, //2560
	DCAM0_1_AXI_CLK_INDEX_3072 = 2, //307.2
	DCAM0_1_AXI_CLK_INDEX_4096 = 3, //409.6
	DCAM0_1_AXI_CLK_INDEX_5120 = 4, //512
};

enum {
	DCAM2_3_CLK_INDEX_960  = 0,  //96
	DCAM2_3_CLK_INDEX_1280 = 1, //128
	DCAM2_3_CLK_INDEX_1536 = 2, //153.6
	DCAM2_3_CLK_INDEX_1920 = 3, //192
	DCAM2_3_CLK_INDEX_2560 = 4, //256
};

enum {
	DCAM2_3_AXI_CLK_INDEX_960  = 0,  //96
	DCAM2_3_AXI_CLK_INDEX_1280 = 1, //128
	DCAM2_3_AXI_CLK_INDEX_1536 = 2, //153.6
	DCAM2_3_AXI_CLK_INDEX_1920 = 3, //192
	DCAM2_3_AXI_CLK_INDEX_2560 = 4, //256
};

enum {
	DCAM_MTX_CLK_INDEX_1536 = 0, //153.6
	DCAM_MTX_CLK_INDEX_3072 = 1, //307.2
	DCAM_MTX_CLK_INDEX_4096 = 2, //409.6
	DCAM_MTX_CLK_INDEX_5120 = 3, //512
};

enum {
	JPG_CLK = 0,
	VDSP_MTX_DATA_CLK = 1,
	ISP_CLK = 2,
	DCAM0_1_AXI_CLK = 3,
	MM_MTX_DATA_CLK = 4,
};
#define DVFS_REG_RD(reg)       (REG_RD(REGS_MM_DVFS_AHB_BASE+reg))
#define DVFS_REG_WR(reg, val)  (REG_WR((REGS_MM_DVFS_AHB_BASE+reg), (val)))

#define DVFS_REG_RD_AHB(reg)       (REG_RD(REGS_MM_AHB_BASE+reg))
#define DVFS_REG_WR_AHB(reg, val)  (REG_WR((REGS_MM_AHB_BASE+reg), (val)))

#define DVFS_REG_RD_ABS(reg)       (REG_RD(REGS_MM_TOP_DVFS_BASE+reg))
#define DVFS_REG_WR_ABS(reg, val)  (REG_WR((REGS_MM_TOP_DVFS_BASE+reg), (val)))

#define DVFS_REG_RD_ON(reg)       (REG_RD(REGS_MM_POWER_ON_BASE+reg))
#define DVFS_REG_WR_ON(reg, val)  (REG_WR((REGS_MM_POWER_ON_BASE+reg), (val)))
//0x327d0000
#define DVFS_REG_RD_POWER(reg)       (REG_RD(REGS_MM_POWER_BASE+reg))
#define DVFS_REG_WR_POWER(reg, val)  (REG_WR((REGS_MM_POWER_BASE+reg), (val)))

#define DVFS_REG_WR_OFF(ipbase, reg, val) (REG_WR(ipbase + reg, val))
#define DVFS_REG_RD_OFF(ipbase, reg)      (REG_RD(ipbase+reg))

#define DVFS_REG_MWR(reg, msk, val)  DVFS_REG_WR(reg, \
		((val)&(msk))|(DVFS_REG_RD(reg)&(~(msk))))
#define DVFS_REG_OWR(reg, val)  \
		DVFS_REG_WR(reg, (DVFS_REG_RD(reg)|val))

#endif /* __MM_DVFS_REG_H___ */
