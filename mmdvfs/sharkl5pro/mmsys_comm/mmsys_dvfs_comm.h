/*
 * Copyright (C) 2019 Unisoc Communications Inc.
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

#ifndef _MMSYS_COMM_DVFS_H_
#define _MMSYS_COMM_DVFS_H_

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>

//#include <video/sprd_mm.h>
#include <sprd_mm.h>
#include <asm/memory.h>
//#include <uapi/video/sprd_mmsys_pw_domain.h>
#include <governor.h>

#define TRUE				1
#define FALSE				0

#define MAX_DVFS_INDEX (8)

enum mmsys_module {
	DVFS_DCAM = 0,
	DVFS_CPP,
	DVFS_ISP,
	DVFS_FD,
	DVFS_JPEG,
	DVFS_DCAMAXI,
	DVFS_MTX,
	DVFS_MODULE_MAX,
};

enum set_freq_type {
	DVFS_IDLE = 0,
	DVFS_WORK,
};

enum mm_dvfs_state {
	DVFS_OFF = 0,
	DVFS_ON,
};

enum mmsys_notifier_type {
	MMSYS_POWER_ON = 1,
	MMSYS_POWER_OFF,
	MMSYS_POWER_ONOFF,
	MMSYS_POWER_MAX,
};

struct ip_dvfs_coffe {
	uint32_t gfree_wait_delay;
	uint32_t freq_upd_hdsk_en;
	uint32_t freq_upd_delay_en;
	uint32_t freq_upd_en_byp;
	uint32_t sw_trig_en;
	uint32_t auto_tune;
	uint32_t work_index_def;
	uint32_t idle_index_def;
	uint32_t idle_switch_en;
};

struct ip_dvfs_map_cfg {

	uint32_t  map_index;
	unsigned long  reg_add;
	uint32_t  volt;
	uint32_t  clk_freq;
	uint32_t  clk;
	uint32_t  fdiv_denom;
	uint32_t  fdiv_num;
	uint32_t  axi_index;
	uint32_t  mtx_index;
};

struct ip_dvfs_status {
	uint32_t  ip_req_volt;
	uint32_t  ip_req_clk;
	uint32_t  current_sys_volt;
	uint32_t  current_ip_clk;
	uint32_t  mmsys_cgm_cfg_debug_info;
	uint32_t  mmsys_volt_debug_info;
	uint32_t is_work;
};

struct mmsys_dvfs_para {
	uint32_t  sys_sw_dvfs_en;
	uint32_t  sys_dvfs_hold_en;
	uint32_t  sys_dvfs_clk_gate_ctrl;
	uint32_t  sys_dvfs_wait_window;
	uint32_t  sys_dvfs_min_volt;
	uint32_t  sys_dvfs_force_en;
	uint32_t  sys_sw_cgb_enable;
};

struct ip_dvfs_para {
	enum mmsys_module ip_type;
	uint32_t  u_dvfs_en;
	uint32_t  u_auto_tune_en;
	uint32_t  u_work_freq;
	uint32_t  u_idle_freq;
	uint32_t  u_work_index;
	uint32_t  u_idle_index;
	uint32_t  u_fix_volt;

	struct ip_dvfs_status ip_status;
	struct ip_dvfs_coffe ip_coffe;
	struct ip_dvfs_map_cfg ip_dvfs_map[8];
};

struct mmreg_map {
	unsigned long mmdvfs_ahb_regbase;
	unsigned long mmdvfs_top_regbase;
	unsigned long mm_ahb_regbase;
	unsigned long mm_power_regbase;
	unsigned long mm_power_on_regbase;
};

struct ip_dvfs_ops {
	struct list_head node;
	const char name[DEVFREQ_NAME_LEN];
	const unsigned int available;
	int (*ip_dvfs_init)(struct devfreq *devfreq);
	int (*ip_hw_dvfs_en)(struct devfreq *devfreq, unsigned int dvfs_eb);
	int (*ip_auto_tune_en)(struct devfreq *devfreq, unsigned long dvfs_eb);
	int (*get_ip_auto_tune_en)(struct devfreq *devfreq,
		unsigned long *dvfs_eb);
	int (*set_work_freq)(struct devfreq *devfreq, unsigned long work_freq);
	int (*set_idle_freq)(struct devfreq *devfreq, unsigned long idle_freq);
	int (*get_ip_dvfs_table)(struct devfreq *devfreq,
		struct ip_dvfs_map_cfg  *dvfs_table);
	int (*set_ip_dvfs_table)(struct devfreq *devfreq,
		struct ip_dvfs_map_cfg  *dvfs_table);
	int (*get_ip_dvfs_coffe)(struct devfreq *devfreq,
		struct ip_dvfs_coffe *dvfs_coffe);
	int (*set_ip_dvfs_coffe)(struct devfreq *devfreq,
		struct ip_dvfs_coffe *dvfs_coffe);
	int (*get_ip_status)(struct devfreq *devfreq,
		struct ip_dvfs_status *ip_status);
	void (*power_on_nb)(struct devfreq *devfreq);
	void (*power_off_nb)(struct devfreq *devfreq);
	void  (*set_ip_gfree_wait_delay)(unsigned int wind_para);
	void  (*set_ip_freq_upd_en_byp)(unsigned int on);
	void  (*set_ip_freq_upd_delay_en)(unsigned int on);
	void  (*set_ip_freq_upd_hdsk_en)(unsigned int on);
	void  (*set_ip_dvfs_swtrig_en)(unsigned int en);
	void  (*set_ip_dfs_idle_disable)(unsigned int on);
	void  (*set_ip_dvfs_work_index)(struct devfreq *devfreq,
		unsigned int index);
	void  (*get_ip_dvfs_work_index)(struct devfreq *devfreq,
		unsigned int *index);
	void  (*set_ip_dvfs_idle_index)(struct devfreq *devfreq,
		unsigned int index);
	void  (*get_ip_dvfs_idle_index)(struct devfreq *devfreq,
		unsigned int *index);
	int (*update_target_freq)(struct devfreq *devfreq, unsigned long volt,
		unsigned long w_freq, unsigned long i_freq);
	int (*event_handler)(struct devfreq *devfreq, unsigned int event,
		void *data);
	int (*set_fix_dvfs_value)(struct devfreq *devfreq, unsigned long freq);
	int  (*mmsys_sw_dvfs_en)(struct devfreq *devfreq, unsigned int sw_en);
	int  (*mmsys_dvfs_hold_en)(struct devfreq *devfreq, unsigned int
			hold_en);
	int  (*mmsys_dvfs_clk_gate_ctrl)(struct devfreq *devfreq,
			unsigned int clk_gate);
	int (*mmsys_dvfs_wait_window)(struct devfreq *devfreq,
			unsigned int wait_window);
	int (*mmsys_dvfs_min_volt)(struct devfreq *devfreq,
			unsigned int min_volt);
	void (*mmsys_power_ctrl)(struct devfreq *devfreq, unsigned int on);
	int (*top_current_volt)(struct devfreq *devfreq,
			unsigned int *top_volt);
	int (*mm_current_volt)(struct devfreq *devfreq, unsigned int *mm_volt);
	int (*get_ip_work_or_idle)(unsigned int *is_work);
};

extern  struct ip_dvfs_ops *get_ip_dvfs_ops(const char *name);
extern  int register_ip_dvfs_ops(struct ip_dvfs_ops *dvfs_ops);
extern  int unregister_ip_dvfs_ops(struct ip_dvfs_ops *dvfs_ops);
extern int mmsys_adjust_target_freq(enum  mmsys_module md_type,
	unsigned long volt, unsigned long freq, unsigned int en_sysgov);
extern int mmsys_register_notifier(struct notifier_block *nb);
extern int mmsys_unregister_notifier(struct notifier_block *nb);
extern int mmsys_notify_call_chain(enum mmsys_notifier_type  notifier_type);
extern int mmsys_set_fix_dvfs_value(unsigned long  fix_volt);
extern int top_mm_dvfs_current_volt(struct devfreq *devfreq);
extern int mm_dvfs_read_current_volt(struct devfreq *devfreq);
extern int mmsys_dvfs_pw_on(void);
extern int mmsys_dvfs_pw_off(void);
extern struct mutex mmsys_glob_reg_lock;
extern struct mmreg_map g_mmreg_map;
extern void dvfs_test_case(u32 tc_id);

#define DVFS_DEBUG
#ifdef DVFS_DEBUG
#define DVFS_TRACE pr_info
#else
#define DVFS_TRACE pr_debug
#endif

#endif
