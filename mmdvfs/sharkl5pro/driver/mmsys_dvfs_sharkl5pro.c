#include "mm_dvfs.h"
#include "mmsys_dvfs.h"
#include "sharkl5pro_mm_dvfs_coffe.h"

static void mm_dvfs_auto_gate_sel(u32  on)
{
	u32 reg_temp = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, on);
	mutex_lock(&mmsys_glob_reg_lock);
	reg_temp = DVFS_REG_RD(
		REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL);
	if (on)
		DVFS_REG_WR(REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL,
			reg_temp | BIT_CGM_MM_DVFS_AUTO_GATE_SEL);
	else
		DVFS_REG_WR(REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL,
			reg_temp & (~BIT_CGM_MM_DVFS_AUTO_GATE_SEL));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static void mm_dvfs_force_en(u32  on)
{
	u32 reg_temp = 0;

	mutex_lock(&mmsys_glob_reg_lock);
	reg_temp = DVFS_REG_RD(REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL);
	if (on)
		DVFS_REG_WR(REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL,
			reg_temp | BIT_CGM_MM_DVFS_FORCE_EN);
	else
		DVFS_REG_WR(REG_MM_DVFS_AHB_CGM_MM_DVFS_CLK_GATE_CTRL,
			reg_temp & (~BIT_CGM_MM_DVFS_FORCE_EN));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static void mm_sw_dvfs_onoff(u32 on)
{
	u32 sw_ctrl_reg = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, on);
	mutex_lock(&mmsys_glob_reg_lock);
	sw_ctrl_reg = DVFS_REG_RD_ABS(REG_TOP_DVFS_APB_SUBSYS_SW_DVFS_EN_CFG);
	if (on)
		DVFS_REG_WR_ABS(REG_TOP_DVFS_APB_SUBSYS_SW_DVFS_EN_CFG,
			sw_ctrl_reg | BIT_MM_SYS_SW_DVFS_EN);
	else
		DVFS_REG_WR_ABS(REG_TOP_DVFS_APB_SUBSYS_SW_DVFS_EN_CFG,
			sw_ctrl_reg & (~BIT_MM_SYS_SW_DVFS_EN));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static void mmdcdc_sw_dvfs_onoff(u32 on)
{
	u32 sw_ctrl_reg = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, on);
	mutex_lock(&mmsys_glob_reg_lock);
	sw_ctrl_reg = DVFS_REG_RD_ABS(REG_TOP_DVFS_APB_DCDC_MM_SW_DVFS_CTRL);
	if (on)
		DVFS_REG_WR_ABS(REG_TOP_DVFS_APB_DCDC_MM_SW_DVFS_CTRL,
		sw_ctrl_reg  | BIT_DCDC_MM_SW_TUNE_EN);
	else
		DVFS_REG_WR_ABS(REG_TOP_DVFS_APB_DCDC_MM_SW_DVFS_CTRL,
		sw_ctrl_reg & (~BIT_DCDC_MM_SW_TUNE_EN));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static void mmsys_sw_cgb_enable(u32  on)
{
	u32 temp_reg = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, on);
	mutex_lock(&mmsys_glob_reg_lock);
	temp_reg = DVFS_REG_RD_AHB(REG_MM_AHB_AHB_EB);
	if (on)
		DVFS_REG_WR_AHB(REG_MM_AHB_AHB_EB, temp_reg | BIT_MM_CKG_EB);
	else
		DVFS_REG_WR_AHB(REG_MM_AHB_AHB_EB, temp_reg & (~BIT_MM_CKG_EB));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static void mm_dvfs_hold_onoff(u32 on)
{
	u32 holdon_reg = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, on);
	mutex_lock(&mmsys_glob_reg_lock);
	holdon_reg = DVFS_REG_RD(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL);
	if (on)
		DVFS_REG_WR(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL,
		holdon_reg | BIT_MM_DVFS_HOLD);
	else
		DVFS_REG_WR(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL,
		holdon_reg & (~BIT_MM_DVFS_HOLD));
	mutex_unlock(&mmsys_glob_reg_lock);
}

static int set_mmsys_sw_dvfs_en(struct devfreq *devfreq,
		unsigned int sw_dvfs_eb)
{
	DVFS_TRACE("dvfs ops: %s\n", __func__);
	mm_sw_dvfs_onoff(sw_dvfs_eb);

	return 1;
}

static int set_mmsys_dvfs_hold_en(struct devfreq *devfreq,
		unsigned int hold_en)
{
	u32 sw_ctrl_reg = 0;

	DVFS_TRACE("dvfs ops: %s , %d\n", __func__, hold_en);
	sw_ctrl_reg = DVFS_REG_RD(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL);
	if (hold_en)
		DVFS_REG_WR(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL,
		sw_ctrl_reg | BIT_MM_DVFS_HOLD);
	else
		DVFS_REG_WR(REG_MM_DVFS_AHB_MM_DVFS_HOLD_CTRL,
		sw_ctrl_reg & (~BIT_MM_DVFS_HOLD));

	return 1;
}

static int set_mmsys_dvfs_clk_gate_ctrl(struct devfreq *devfreq,
		unsigned int clk_gate)
{
	DVFS_TRACE("dvfs ops: %s\n", __func__);
	mm_dvfs_auto_gate_sel(clk_gate);
	
	return 1;
}

static int set_mmsys_dvfs_wait_window(struct devfreq *devfreq,
		unsigned int wait_window)
{
	DVFS_TRACE("dvfs ops: %s\n", __func__);
	DVFS_REG_WR(REG_MM_DVFS_AHB_MM_DVFS_WAIT_WINDOW_CFG, wait_window);

	return 1;
}

static int set_mmsys_dvfs_min_volt(struct devfreq *devfreq,
		unsigned int min_volt)
{
	u32 reg_temp = 0;

	DVFS_TRACE("dvfs ops: %s\n", __func__);
	reg_temp = DVFS_REG_RD(REG_MM_DVFS_AHB_MM_MIN_VOLTAGE_CFG);
	DVFS_REG_WR(REG_MM_DVFS_AHB_MM_MIN_VOLTAGE_CFG,
			reg_temp | (min_volt & 0x7));

	return 1;
}

static int mmsys_init(struct devfreq *devfreq)
{
	struct mmsys_dvfs *mmsys;

	mmsys = dev_get_drvdata(devfreq->dev.parent);
	DVFS_TRACE("dvfs ops: 01 mmsys %s\n", __func__);
	/* sys clk enable */
	mmsys->mmsys_dvfs_para.sys_sw_cgb_enable = 1;
	mmsys_sw_cgb_enable(mmsys->mmsys_dvfs_para.sys_sw_cgb_enable);
	/* mm hw dvfs */
	mmdcdc_sw_dvfs_onoff(0);
	mm_sw_dvfs_onoff(0);
	mm_dvfs_force_en(1);
	mm_dvfs_hold_onoff(0);
	mmsys->mmsys_dvfs_para.sys_dvfs_force_en = 1;
	mmsys->mmsys_dvfs_para.sys_dvfs_wait_window =
		MMSYS_DVFS_WAIT_UP_WINDOW << 16 |
		MMSYS_DVFS_WAIT_DOWN_WINDOW;
	set_mmsys_dvfs_wait_window(devfreq,
		mmsys->mmsys_dvfs_para.sys_dvfs_wait_window);
	
	return 1;
}


static void power_on_nb(struct devfreq *devfreq)
{
	DVFS_TRACE("dvfs ops: mmsys %s\n", __func__);
	mmsys_init(devfreq);
}

static void  mmsys_power_ctrl(struct devfreq *devfreq, unsigned int  on)
{
	DVFS_REG_WR_POWER(0x00, 0x3208004);
	DVFS_REG_WR_ON(0x00, 0x3c280);
	DVFS_TRACE("dvfs ops: %s\n", __func__);
	if (on)
		mmsys_notify_call_chain(MMSYS_POWER_ON);
	else
		mmsys_notify_call_chain(MMSYS_POWER_OFF);
}

struct ip_dvfs_ops mmsys_dvfs_ops = {
	.name = "MMSYS_DVFS_OPS",
	.available = 1,

	.ip_dvfs_init = mmsys_init,
	.mmsys_dvfs_clk_gate_ctrl = set_mmsys_dvfs_clk_gate_ctrl,
	.mmsys_sw_dvfs_en = set_mmsys_sw_dvfs_en,
	.mmsys_dvfs_hold_en = set_mmsys_dvfs_hold_en,
	.mmsys_dvfs_wait_window = set_mmsys_dvfs_wait_window,
	.mmsys_dvfs_min_volt = set_mmsys_dvfs_min_volt,
	.mmsys_power_ctrl = mmsys_power_ctrl,
	.power_on_nb = power_on_nb,
};

