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

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <os_adapt_common.h>
#include <sprd_mm.h>

#include "cam_debugger.h"
#include "dcam_reg.h"
#include "isp_cfg.h"
#include "isp_reg.h"
#include "cam_pipeline.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "CAM_DEBUGGER: %d %d %s : "\
	fmt, current->pid, __LINE__, __func__

#define WORK_MODE_SLEN      2
#define LBUF_LEN_SLEN       8
#define DEBUG_CONTROL_KEY   "*#781#*"

uint32_t g_dcam_bypass[DCAM_HW_CONTEXT_MAX] = {0};
struct cam_dbg_dump g_dbg_dump[DUMP_NUM_MAX];
struct cam_dbg_replace g_dbg_replace;
uint64_t g_isp_bypass[ISP_CONTEXT_SW_NUM] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int g_dbg_iommu_mode = IOMMU_AUTO;
int g_dbg_set_iommu_mode = IOMMU_AUTO;
uint32_t g_pyr_dec_online_bypass = 0;
uint32_t g_pyr_dec_offline_bypass = 0;
uint32_t g_dbg_replace_src = REPLACE_IMG_YUV;
uint32_t g_dbg_replace_switch = 0;
uint32_t g_dbg_dumpswitch = 0;
uint32_t g_dbg_dumpcount = 0;
uint32_t g_dbg_recovery = DEBUG_DCAM_RECOVERY_IDLE;
uint32_t g_dbg_fbc_control = 0;
uint32_t contr_cap_eof = 1;
uint32_t g_dbg_debug_control = 0;
uint32_t g_dbg_raw2frgb_switch = DEBUG_RAWCAP_MODE;
uint32_t g_pipeline_type = CAM_PIPELINE_TYPE_MAX;
extern uint32_t s_dbg_linebuf_len;
static struct dentry *s_p_dentry;
static struct debug_cmd g_dbg_readinfo = {NULL, NULL, NULL, HW_CTX_MAX, IP_MAX};

static char zoom_mode_strings[4][8] = {
	"bypass", "bin2", "bin4", "scaler"
};

static uint8_t iommu_mode_string[4][32] = {
	"IOMMU_AUTO",
	"IOMMU_OFF",
	"IOMMU_ON_RESERVED",
	"IOMMU_ON"
};

static void camdebugger_dcam_bypass_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	struct cam_hw_bypass_data data = {0};
	uint32_t type = 0;
	uint32_t bypass_cnt = 0;
	uint32_t i = 0;
	int bypass_all = 0;
	uint32_t val = 0;

	if (!hw->s_dcam_dev_debug) {
		pr_err(" fail to enable dcam Hardware\n");
		return;
	}

	if (idx == HW_CTX_MAX) {
		pr_err("fail to get right hw_ctx\n");
		return;
	}

	val = simple_strtol(param, NULL, 0);
	val = val != 0 ? 1 : 0;
	/* find */
	if (strcmp(name, "dcam_all") == 0)
		bypass_all = 1;

	type = DCAM_BYPASS_TYPE;
	bypass_cnt = hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_COUNT_GET, &type);
	data.type = DCAM_BYPASS_TYPE;

	for (i = 0; i < bypass_cnt; i++) {
		data.i = i;
		hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_DATA_GET, &data);
		if (data.tag == NULL)
			continue;
		if (data.tag->p == NULL)
			continue;
		if (strcmp(data.tag->p, name) == 0 || bypass_all){
			printk("set dcam%d addr 0x%x, bit %d val %d\n",
				idx, data.tag->addr, data.tag->bpos, val);
			g_dcam_bypass[idx] &= (~(1 << i));
			g_dcam_bypass[idx] |= (val << i);
			os_adapt_time_msleep(20); /* If PM writing,wait little time */
			DCAM_REG_MWR(idx, data.tag->addr, 1 << data.tag->bpos,
				val << data.tag->bpos);
			/* afl need rgb2y work */
			if (strcmp(name, "afl") == 0)
				DCAM_REG_MWR(idx, ISP_AFL_PARAM0, BIT_0, val);
			if (!bypass_all)
				break;
		}
	}
	/* not opreate */
	if ((!bypass_all) && i >= bypass_cnt)
		pr_info("Not operate, dcam%d,name:%s val:%d\n", idx, name, val);
}

static void camdebugger_dcam_bypass_read(struct seq_file *s, uint32_t idx)
{
	uint32_t addr = 0, val = 0;
	struct cam_hw_bypass_data data = {0};
	uint32_t type = 0;
	uint32_t bypass_cnt = 0;
	uint32_t i = 0;
	struct cam_hw_info *hw = NULL;

	if (idx == HW_CTX_MAX) {
		pr_err("fail to get right hw_ctx\n");
		return;
	}

	hw = (struct cam_hw_info *)s->private;
	data.type = DCAM_BYPASS_TYPE;
	type = DCAM_BYPASS_TYPE;
	bypass_cnt = hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_COUNT_GET, &type);
	seq_printf(s, "-----dcam%d-----\n", idx);
	if (!hw->s_dcam_dev_debug) {
		seq_puts(s, "Hardware not enable\n");
	} else {
		for (i = 0; i < bypass_cnt; i++) {
			data.i = i;
			hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_DATA_GET, &data);
			if (data.tag == NULL)
				continue;
			if (data.tag->p == NULL)
				continue;
			addr = data.tag->addr;
			val = DCAM_REG_RD(idx, addr) & (1 << data.tag->bpos);
			if (val)
				seq_printf(s, "%s:bit%d=1 bypass\n",
					data.tag->p, data.tag->bpos);
			else
				seq_printf(s, "%s:bit%d=0  work\n",
					data.tag->p, data.tag->bpos);
		}
		seq_puts(s, "\ndcam_all:1:bypass_dcam0 #to bypass all\n\n");
		seq_puts(s, "Example: make 4in1 module bypass(1) or work(0)\n");
		seq_puts(s, "bypass: echo 4in1:1:bypass_dcam0 > debug_node\n");
		seq_puts(s, "work: echo 4in1:0:bypass_dcam0 > debug_node\n\n");
	}
}

/* read dcamx register once */
static void camdebugger_dcam_reg_read(struct seq_file *s, uint32_t idx)
{
	uint32_t addr = 0;
	struct cam_hw_info *hw = NULL;
	const uint32_t addr_end[] = {0x400, 0x400, 0x110, 0x110, 0x100, 0x100};

	hw = (struct cam_hw_info *)s->private;
	if (!hw->s_dcam_dev_debug) {
		seq_puts(s, "Hardware not enable\n");
		return;
	}

	if (idx >= 3 && idx < 5) {
		seq_printf(s, "-----dcam axi%d and fetch----\n", idx - 3);
		for (addr = 0; addr < addr_end[idx]; addr += 4)
			seq_printf(s, "0x%04x: 0x%08x\n",
				addr, REG_RD(g_dcam_aximbase[idx - 3] + addr));

		seq_puts(s, "--------------------\n");
	} else {
		seq_printf(s, "-----dcam%d----------\n", idx);
		for (addr = 0; addr < addr_end[idx]; addr += 4)
			seq_printf(s, "0x%04x: 0x%08x\n",
				addr, DCAM_REG_RD(idx, addr));

		seq_puts(s, "--------------------\n");
	}
}

static void camdebugger_zoom_mode_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	int val = 0;

	val = simple_strtol(param, NULL, 0);
	if (val == 0)
		g_camctrl.dcam_zoom_mode = ZOOM_DEBUG_DEFAULT;
	else if (val == 1)
		g_camctrl.dcam_zoom_mode = ZOOM_DEBUG_BINNING2;
	else if (val == 2)
		g_camctrl.dcam_zoom_mode = ZOOM_DEBUG_BINNING4;
	else if (val == 3)
		g_camctrl.dcam_zoom_mode = ZOOM_DEBUG_SCALER;
	else
		pr_err("fail to get valid zoom mode: %d\n", val);

	if (g_camctrl.dcam_zoom_mode >= ZOOM_DEBUG_DEFAULT)
		pr_info("set zoom mode %d(%s)\n", g_camctrl.dcam_zoom_mode - ZOOM_DEBUG_DEFAULT,
			zoom_mode_strings[g_camctrl.dcam_zoom_mode - ZOOM_DEBUG_DEFAULT]);
	else
		pr_info("set zoom mode %d(%s)\n", g_camctrl.dcam_zoom_mode,
			zoom_mode_strings[g_camctrl.dcam_zoom_mode]);
}

static void camdebugger_zoom_mode_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "0: bypass, 1: bin2, 2: bin4, 3: scaler";

	if (g_camctrl.dcam_zoom_mode >= ZOOM_DEBUG_DEFAULT)
		seq_printf(s, "%d(%s)\n%s\n", g_camctrl.dcam_zoom_mode - ZOOM_DEBUG_DEFAULT,
			zoom_mode_strings[g_camctrl.dcam_zoom_mode - ZOOM_DEBUG_DEFAULT], desc);
	else
		seq_printf(s, "%d(%s)\n%s\n", g_camctrl.dcam_zoom_mode,
			zoom_mode_strings[g_camctrl.dcam_zoom_mode], desc);
}

static void camdebugger_fbc_control_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_fbc_control = simple_strtol(param, NULL, 0);
	pr_info("set fbc_control %u\n", g_dbg_fbc_control);
}

static void camdebugger_fbc_control_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "bit 0:bin 1:full 2:raw\n";

	seq_printf(s, "%u\n\n%s\n", g_dbg_fbc_control, desc);
	seq_printf(s, "\nUsage:\n");
	seq_printf(s, "         echo fbc_ctrl:val >debug_node\n");
	seq_printf(s, "The different bits represent fbc switch in different path\n");
	seq_printf(s, "\nExample:\n");
	seq_printf(s, "         echo fbc_ctrl:6 >debug_node   // bit 110, bypass full and raw path fbc\n");
	seq_printf(s, "         echo fbc_ctrl:3 >debug_node   // bit 011, bypass bin and full path fbc\n");
}

static void camdebugger_recovery_bypass_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_recovery = simple_strtol(param, NULL, 0);
	pr_info("set recovery %u\n", g_dbg_recovery);
}

static void camdebugger_recovery_bypass_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "0: idle, 1: bypass 2:enable recovery\n";
	seq_printf(s, "%u\n\n%s\n", g_dbg_recovery, desc);
}

static void camdebugger_replace_control_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	uint32_t val = 0;

	val = simple_strtol(param, NULL, 0);
	pr_info("set replace_control %u\n", val);

	cam_replace_node_set(&val);
}

static void camdebugger_replace_control_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "\nUsage:\n");
	seq_printf(s, "  The different bits represent replace pipeline, node, port, state\n");
	seq_printf(s, "  Replace image form dump, need change image name \n");
	seq_printf(s, "      1.setenforce 0\n");
	seq_printf(s, "      2.echo replace_ctrl:val >debug_node\n");
	seq_printf(s, "      3.echo replace_file_fmt:replace img_fmt >debug_node\n");
	seq_printf(s, "  val = (pipeline_type << 3)|(node_type << 2)|(port_id << 1)|(state << 0)\n");
	seq_printf(s, "  replace img_fmt = character string\n");
	seq_printf(s, "\nExample:\n");
	seq_printf(s, "  replace capture_pipeline dcam_online_node full_port enable:\n");
	seq_printf(s, "      setenforce 0\n");
	seq_printf(s, "      echo replace_ctrl:2021 >debug_node\n");
	seq_printf(s, "      echo replace_file_fmt:yuv >debug_node\n");
	seq_printf(s, "  2->cap pipeline, 0->dcam online node, 2->full port, 1->work\n");
	seq_printf(s, "\nReplace Info:\n");
	seq_printf(s, "      Pipeline_type:        Preview = 0, Video = 1, Capture = 2, Zsl_Capture = 3...\n");
	seq_printf(s, "      Node_type:            Dcam_online = 0, Dcam_offline = 1, Offline_dec = 6...\n");
	seq_printf(s, "      Dcam_online port_id:  Bin = 1, Full = 2...\n");
	seq_printf(s, "      State:                Bypass = 0, Work = 1.\n");
	seq_printf(s, "      replace img_fmt:      yuv, mipiraw, raw...\n");
	seq_printf(s, "\nPS:\n");
	seq_printf(s, "      Replace PREV/VID pipeline need input replace_switch\n");
	seq_printf(s, "      Replace CAP pipeline need start capture\n");
	seq_printf(s, "      Reopen camera after input replace cmd\n");
}

static void camdebugger_replace_file_fmt_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	uint32_t val = 0;

	if (strcmp(param, "yuv") == 0)
		val = REPLACE_IMG_YUV;
	else if (strcmp(param, "mipi") == 0)
		val = REPLACE_IMG_MIPIRAW;
	else if (strcmp(param, "raw") == 0)
		val = REPLACE_IMG_RAW;

	g_dbg_replace_src = val;
	pr_info("set replace src image type %u\n", g_dbg_replace_src);
}

static void camdebugger_replace_switch_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_replace_switch = simple_strtol(param, NULL, 0);
	pr_info("set replaceswitch %u\n", g_dbg_replace_switch);
}

static void camdebugger_replace_switch_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "PREV/VIDEO pipeline dump 0: disable, 1: enable\n";
	seq_printf(s, "%u\n\n%s\n", g_dbg_replace_switch, desc);
}

static void camdebugger_dump_switch_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_dumpswitch = simple_strtol(param, NULL, 0);
	pr_info("set dumpswitch %u\n", g_dbg_dumpswitch);
}

static void camdebugger_dump_switch_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "g_dbg_dumpswitch %u\n", g_dbg_dumpswitch);
	seq_printf(s, "PREV/VIDEO pipeline dump 0: disable, 1: enable\n");
}

static void camdebugger_dump_control_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	uint32_t val = 0;

	val = simple_strtol(param, NULL, 16);
	pr_info("set dump_control %x\n", val);

	cam_dump_node_set(&val);
}

static void camdebugger_dump_control_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "\nUsage:\n");
	seq_printf(s, "  The different bits represent dump pipeline, node, port, state\n");
	seq_printf(s, "      1.setenforce 0\n");
	seq_printf(s, "      2.echo dump_ctrl:val >debug_node\n");
	seq_printf(s, "  val = (pipeline_type << 3)|(node_type << 2)|(port_id << 1)|(state << 0)\n");
	seq_printf(s, "\nExample:\n");
	seq_printf(s, "  Dump capture_pipeline dcam_online_node full_port enable:\n");
	seq_printf(s, "      setenforce 0\n");
	seq_printf(s, "      echo dump_ctrl:2021 >debug_node\n");
	seq_printf(s, "  2->cap pipeline, 0->dcam online node, 2->full port, 1->work\n");
	seq_printf(s, "\nDump Info:\n");
	seq_printf(s, "      Pipeline_type:        Preview = 0, Video = 1, Capture = 2, Zsl_Capture = 3, Online_normal2yuv_or_raw2user2yuv = b...\n");
	seq_printf(s, "      Node_type:            Dcam_online = 0, Dcam_offline = 1, Offline_dec = 8...\n");
	seq_printf(s, "      Dcam_online port_id:  Bin = 1, Full = 2...\n");
	seq_printf(s, "      State:                Bypass = 0, Work = 1.\n");
	seq_printf(s, "\nPS:\n");
	seq_printf(s, "      Dump PREV/VID pipeline need input dump_switch\n");
	seq_printf(s, "      Dump CAP pipeline need start capture\n");
	seq_printf(s, "      Reopen camera after input dump cmd\n");
}

static void camdebugger_dump_count_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_dumpcount = simple_strtol(param, NULL, 0);
	pr_info("set dump_count %u\n", g_dbg_dumpcount);
}

static void camdebugger_pyr_dec_bypass_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_pyr_dec_online_bypass = simple_strtol(param, NULL, 0);
	pr_info("set pyr_dec_online_bypass %u\n", g_pyr_dec_online_bypass);
}

static void camdebugger_pyr_dec_bypass_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "0: work, 1: bybass";
	char *str = "Example: echo pyr_dec_bypass:1 >debug_node";
	seq_printf(s, "%u\n\n%s\n\n%s\n\n", g_pyr_dec_online_bypass, desc ,str);
}

static void camdebugger_dcamint_eof_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	contr_cap_eof = simple_strtol(param, NULL, 0);
	pr_info("set contr_cap_eof value %d\n", contr_cap_eof);
}

static void camdebugger_dcamint_eof_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "cap_eof: %d\n", contr_cap_eof);
}

static void camdebugger_rawcap_frgb_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_dbg_raw2frgb_switch = simple_strtol(param, NULL, 0);
	pr_info("set raw2frgb_switch %u\n", g_dbg_raw2frgb_switch);
}

static void camdebugger_rawcap_frgb_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "0: rawcap mode, 1: frgb mode";
	char *str = "Example: echo rawcap_frgb_switch:1 >debug_node";
	seq_printf(s, "%u\n\n%s\n\n%s\n\n", g_dbg_raw2frgb_switch, desc ,str);
}

static void camdebugger_pipeline_log_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_pipeline_type = simple_strtol(param, NULL, 0);
	pr_info("set pipeline log type %d\n", g_pipeline_type);
}

static void camdebugger_pipeline_log_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "pipeline num %u\n", g_pipeline_type);
	seq_printf(s, "\n Example: open Preview Pipeline Log\n");
	seq_printf(s, "\n echo debug_log_switch:0 >debug_node\n");
	seq_printf(s, "\n Pipeline Type ID\n");
	seq_printf(s, " 0: CAM_PIPELINE_PREVIEW\n");
	seq_printf(s, " 1: CAM_PIPELINE_VIDEO\n");
	seq_printf(s, " 2: CAM_PIPELINE_CAPTURE\n");
	seq_printf(s, " 3: CAM_PIPELINE_ZSL_CAPTURE\n");
	seq_printf(s, " 4: CAM_PIPELINE_SENSOR_RAW\n");
	seq_printf(s, " 5: CAM_PIPELINE_SCALER_YUV\n");
	seq_printf(s, " 6: CAM_PIPELINE_OFFLINE_RAW2YUV\n");
	seq_printf(s, " 7: CAM_PIPELINE_ONLINERAW_2_OFFLINEYUV\n");
	seq_printf(s, " 8: CAM_PIPELINE_ONLINERAW_2_USER_2_OFFLINEYUV\n");
	seq_printf(s, " 9: CAM_PIPELINE_ONLINERAW_2_COPY_2_USER_2_OFFLINEYUV\n");
	seq_printf(s, " 10: CAM_PIPELINE_ONLINEBPCRAW_2_USER_2_OFFLINEYUV\n");
	seq_printf(s, " 11: CAM_PIPELINE_ONLINERAW_2_USER_2_BPCRAW_2_USER_2_OFFLINEYUV\n");
	seq_printf(s, " 12: CAM_PIPELINE_ONLINE_NORMAL2YUV_OR_RAW2USER2YUV\n");
	seq_printf(s, " 13: CAM_PIPELINE_ONLINE_NORMALZSLCAPTURE_OR_RAW2USER2YUV\n");
	seq_printf(s, " 14: CAM_PIPELINE_OFFLINE_RAW2FRGB_OFFLINE_FRGB2YUV\n");
	seq_printf(s, " 15: CAM_PIPELINE_ONLINEYUV_2_USER_2_OFFLINEYUV_2_NR\n");
	seq_printf(s, " 16: Close Pipeline Log\n");
}

static void camdebugger_isp_bypass_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	struct cam_hw_bypass_data data = {0};
	struct isp_hw_ltm_3dnr_param parm = {0};
	uint32_t type ={0};
	uint32_t bypass_cnt = 0;
	uint32_t val = 0;
	uint32_t bypass_all = 0;
	uint64_t bypass_val = 1;
	uint32_t i = 0;

	if (!hw->s_isp_dev_debug) { /* isp not working */
		pr_err(" fail to enable isp Hardware\n");
		return;
	}

	if (idx == HW_CTX_MAX) {
		pr_err("fail to get right hw_ctx\n");
		return;
	}

	val = simple_strtol(param, NULL, 0);
	val = val != 0 ? 1 : 0;
	/* find */
	if (strcmp(name, "isp_all") == 0) {
		bypass_all = 1;
	} else { /* check special: ltm, nr3 */
		if (strcmp(name, "ltm") == 0) {
			parm.idx = idx;
			parm.val = val;
			hw->isp_ioctl(hw, ISP_HW_CFG_LTM_PARAM, &parm);
			bypass_val = (bypass_val << _EISP_LTM);
			g_isp_bypass[idx] &= (~bypass_val);
			if (val)
				g_isp_bypass[idx] |= bypass_val;
			return;
		} else if (strcmp(name, "nr3") == 0 ||
			strcmp(name, "3dnr") == 0) {
			bypass_val = (bypass_val << _EISP_NR3);
			g_isp_bypass[idx] &= (~bypass_val);
			if (val)
				g_isp_bypass[idx] |= bypass_val;
			parm.idx = idx;
			parm.val = val;
			hw->isp_ioctl(hw, ISP_HW_CFG_3DNR_PARAM, &parm);
			/* ISP_HREG_MWR(ISP_FBC_3DNR_STORE_PARAM, BIT_0, val); */
			return;
		}
	}
	type = ISP_BYPASS_TYPE;
	bypass_cnt = hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_COUNT_GET, &type);
	data.type = ISP_BYPASS_TYPE;
	for (i = 0; i < bypass_cnt; i++) {
		data.i = i;
		hw->isp_ioctl(hw, ISP_HW_CFG_BYPASS_DATA_GET, &data);
		if (data.tag == NULL) {
			continue;
		}
		if (data.tag->p == NULL)
			continue;
		if (strcmp(data.tag->p, name) == 0 || bypass_all) {
			pr_debug("set isp addr 0x%x, bit %d val %d\n",
				data.tag->addr, data.tag->bpos, val);
			if (i < _EISP_TOTAL) {
				bypass_val = val;
				bypass_val = bypass_val << i;
				g_isp_bypass[idx] &= (~bypass_val);
				g_isp_bypass[idx] |= bypass_val;
			}
			if (bypass_all && (data.tag->all == 0))
				continue;
			ISP_REG_MWR(idx, data.tag->addr, 1 << data.tag->bpos,
				val << data.tag->bpos);

			if (!bypass_all)
				break;
		}
	}
		/* not opreate */
	if ((!bypass_all) && i >= bypass_cnt)
		pr_info("Not operate, isp%d,name:%s val:%d\n", idx, name, val);
}

static void camdebugger_isp_bypass_read(struct seq_file *s, uint32_t idx)
{
	uint32_t addr = 0, val = 0;
	struct cam_hw_info *hw = NULL;
	struct cam_hw_bypass_data data = {0};
	uint32_t type = 0;
	uint32_t bypass_cnt = 0;
	uint32_t i = 0;

	if (!s) {
		pr_err("fail to get valid input para\n");
		return;
	}
	hw = (struct cam_hw_info *)s->private;

	if (idx == HW_CTX_MAX) {
		pr_err("fail to get right hw_ctx\n");
		return;
	}

	if (!hw->s_isp_dev_debug) { /* isp not working */
		seq_printf(s, "isp hardware not working, can't read\n");
		return;
	}
	seq_printf(s, "===========isp context %d=============\n", idx);
	type = ISP_BYPASS_TYPE;
	bypass_cnt = hw ->isp_ioctl(hw , ISP_HW_CFG_BYPASS_COUNT_GET, &type);
	data.type = ISP_BYPASS_TYPE;
	for (i = 0; i < bypass_cnt; i++) {
		data.i = i;
		hw ->isp_ioctl(hw , ISP_HW_CFG_BYPASS_DATA_GET, &data);
		if (data.tag == NULL) {
			continue;
		}
		if (data.tag->p == NULL)
			continue;

		addr = data.tag->addr;
		val = ISP_REG_RD(idx, addr) & (1 << data.tag->bpos);
		if (val)
			seq_printf(s, "%s:%d  bypass\n", data.tag->p, val);
		else
			seq_printf(s, "%s:%d  work\n", data.tag->p, val);
	}
	seq_puts(s, "\n isp_all:1:bypass_pre0 //bypass all except preview path\n");
	seq_puts(s, "\nltm:1:bypass_pre0 //(ltm-hist,ltm-map)\n");
	seq_puts(s, "\nnr3:1:bypass_pre0 //or 3dnr:1:bypass_pre0(all of 3dnr block)\n");
}

static void camdebugger_reg_buf_read(struct seq_file *s, uint32_t idx)
{
	if (idx == HW_CTX_MAX) {
		pr_err("fail to get right hw_ctx\n");
		return;
	}

	isp_cfg_ctx_reg_buf_debug_show((void *)s, idx);
}

static void camdebugger_iommu_mode_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	int val = 0;

	val = simple_strtol(param, NULL, 0);
	if (val == 0)
		g_dbg_set_iommu_mode = IOMMU_AUTO;
	else if (val == 1)
		g_dbg_set_iommu_mode = IOMMU_OFF;
	else if (val == 2)
		g_dbg_set_iommu_mode = IOMMU_ON_RESERVED;
	else if (val == 3)
		g_dbg_set_iommu_mode = IOMMU_ON;
	else
		pr_err("fail to get valid work mode: %d\n", val);

	pr_info("set_iommu_mode : %d(%s)\n",
		g_dbg_set_iommu_mode,
		iommu_mode_string[g_dbg_set_iommu_mode&3]);
}

static void camdebugger_iommu_mode_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "cur: %d(%s), next: %d(%s)\n",
		g_dbg_iommu_mode,
		iommu_mode_string[g_dbg_iommu_mode&3],
		g_dbg_set_iommu_mode,
		iommu_mode_string[g_dbg_set_iommu_mode&3]);
}

static void camdebugger_lbuf_len_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	s_dbg_linebuf_len = simple_strtol(param, NULL, 0);
	pr_info("set line buf len %d.  %s\n", s_dbg_linebuf_len, param);
}

static void camdebugger_lbuf_len_read(struct seq_file *s, uint32_t idx)
{
	seq_printf(s, "%d\n", s_dbg_linebuf_len);
}

static void camdebugger_isp_pyr_dec_bypass_write(struct cam_hw_info *hw, char *name, char *param, uint32_t idx)
{
	g_pyr_dec_offline_bypass = simple_strtol(param, NULL, 0);
	pr_info("set pyr_dec_offline_bypass %u\n", g_pyr_dec_offline_bypass);
}

static void camdebugger_isp_pyr_dec_bypass_read(struct seq_file *s, uint32_t idx)
{
	const char *desc = "0: work, 1: bybass";
	char *str = "Example: echo 1 > offline_dec_bypass";
	seq_printf(s, "%u\n\n%s\n\n%s\n\n", g_pyr_dec_offline_bypass, desc ,str);
}

static int camdebugger_strtok(char *buf, char *name, char *val,
	char *tag, int name_size, int val_size, int tag_size)
{
	uint32_t j = 0;
	uint32_t k = 0;
	uint32_t i = 0;

	for (i = 0; i < name_size - 1; i++) {
		if ('\0' == buf[i] || ' ' == buf[i] || ':' == buf[i] || '\n' == buf[i])
			break;
		name[i] = buf[i];
	}
	name[i] = '\0';
	i++;

	for (j = 0; j < val_size - 1; i++, j++) {
		if ('\0' == buf[i] || ' ' == buf[i] || ':' == buf[i] || '\n' == buf[i])
			break;
		val[j] = buf[i];
	}

	if(j == 0)
		return -1;
	val[j] = '\0';
	i++;

	for (k = 0; k < tag_size - 1; i++, k++) {
		if ('\0' == buf[i] || ' ' == buf[i] || ':' == buf[i] || '\n' == buf[i])
			break;
		tag[k] = buf[i];
	}
	tag[k] = '\0';

	return 0;
}

static struct debug_cmd debug_cmd_tab[] = {
	{"bypass_dcam0",       NULL,                                 camdebugger_dcam_bypass_read,        HW_CTX_0,   IP_DCAM},
	{"bypass_dcam1",       NULL,                                 camdebugger_dcam_bypass_read,        HW_CTX_1,   IP_DCAM},
	{"reg_dcam0",          NULL,                                 camdebugger_dcam_reg_read,           HW_CTX_0,   IP_DCAM},
	{"reg_dcam1",          NULL,                                 camdebugger_dcam_reg_read,           HW_CTX_1,   IP_DCAM},
	{"reg_dcam2",          NULL,                                 camdebugger_dcam_reg_read,           HW_CTX_2,   IP_DCAM},
	{"reg_fetch",          NULL,                                 camdebugger_dcam_reg_read,           HW_CTX_3,   IP_DCAM},
	{"pre0_buf",           NULL,                                 camdebugger_reg_buf_read,            HW_CTX_0,   IP_ISP},
	{"cap0_buf",           NULL,                                 camdebugger_reg_buf_read,            HW_CTX_1,   IP_ISP},
	{"pre1_buf",           NULL,                                 camdebugger_reg_buf_read,            HW_CTX_2,   IP_ISP},
	{"cap1_buf",           NULL,                                 camdebugger_reg_buf_read,            HW_CTX_3,   IP_ISP},
	{"bypass_pre0",        NULL,                                 camdebugger_isp_bypass_read,         HW_CTX_0,   IP_ISP},
	{"bypass_cap0",        NULL,                                 camdebugger_isp_bypass_read,         HW_CTX_1,   IP_ISP},
	{"bypass_pre1",        NULL,                                 camdebugger_isp_bypass_read,         HW_CTX_2,   IP_ISP},
	{"bypass_cap1",        NULL,                                 camdebugger_isp_bypass_read,         HW_CTX_3,   IP_ISP},
	{"zoom_mode",          camdebugger_zoom_mode_write,          camdebugger_zoom_mode_read,          HW_CTX_MAX, IP_MAX},
	{"fbc_ctrl",           camdebugger_fbc_control_write,        camdebugger_fbc_control_read,        HW_CTX_MAX, IP_MAX},
	{"recovery_bypass",    camdebugger_recovery_bypass_write,    camdebugger_recovery_bypass_read,    HW_CTX_MAX, IP_MAX},
	{"replace_ctrl",       camdebugger_replace_control_write,    camdebugger_replace_control_read,    HW_CTX_MAX, IP_MAX},
	{"replace_file_fmt",   camdebugger_replace_file_fmt_write,   NULL,                                HW_CTX_MAX, IP_MAX},
	{"replace_switch",     camdebugger_replace_switch_write,     camdebugger_replace_switch_read,     HW_CTX_MAX, IP_MAX},
	{"dump_switch",        camdebugger_dump_switch_write,        camdebugger_dump_switch_read,        HW_CTX_MAX, IP_MAX},
	{"dump_ctrl",          camdebugger_dump_control_write,       camdebugger_dump_control_read,       HW_CTX_MAX, IP_MAX},
	{"dump_count",         camdebugger_dump_count_write,         NULL,                                HW_CTX_MAX, IP_MAX},
	{"pyr_dec_bypass",     camdebugger_pyr_dec_bypass_write,     camdebugger_pyr_dec_bypass_read,     HW_CTX_MAX, IP_MAX},
	{"dcam_cap_eof",       camdebugger_dcamint_eof_write,        camdebugger_dcamint_eof_read,        HW_CTX_MAX, IP_MAX},
	{"rawcap_frgb",        camdebugger_rawcap_frgb_write,        camdebugger_rawcap_frgb_read,        HW_CTX_MAX, IP_MAX},
	{"debug_log_switch",   camdebugger_pipeline_log_write,       camdebugger_pipeline_log_read,       HW_CTX_MAX, IP_MAX},
	{"iommu_mode",         camdebugger_iommu_mode_write,         camdebugger_iommu_mode_read,         HW_CTX_MAX, IP_MAX},
	{"line_buf_len",       camdebugger_lbuf_len_write,           camdebugger_lbuf_len_read,           HW_CTX_MAX, IP_MAX},
	{"offline_dec_bypass", camdebugger_isp_pyr_dec_bypass_write, camdebugger_isp_pyr_dec_bypass_read, HW_CTX_MAX, IP_MAX},
};

static struct debug_cmd debug_fun_get(char *name, char *tag)
{
	uint32_t total_num = 0;
	uint32_t i = 0;
	struct debug_cmd debug_info = {NULL, NULL, NULL, HW_CTX_MAX, IP_MAX};

	total_num = sizeof(debug_cmd_tab) / sizeof(struct debug_cmd);
	for (i = 0; i < total_num; i++) {
		if (strcmp(name, debug_cmd_tab[i].name) == 0) {
			debug_info.write_fun = debug_cmd_tab[i].write_fun;
			debug_info.read_fun = debug_cmd_tab[i].read_fun;
			debug_info.hw_ctx = debug_cmd_tab[i].hw_ctx;
			debug_info.ip_info = debug_cmd_tab[i].ip_info;
			break;
		}
		else {
			if (tag && strcmp(tag, debug_cmd_tab[i].name) == 0) {
				debug_info.hw_ctx = debug_cmd_tab[i].hw_ctx;
				debug_info.ip_info = debug_cmd_tab[i].ip_info;
				debug_info.read_fun = debug_cmd_tab[i].read_fun;
			}
		}
	}

	pr_info("idx %d ,ip_info %d\n", debug_info.hw_ctx, debug_info.ip_info);
	return debug_info;
}

static ssize_t camdebugger_input_write(struct file *filp,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	struct seq_file *p = (struct seq_file *)filp->private_data;
	struct cam_hw_info *hw = (struct cam_hw_info *)p->private;
	struct debug_cmd debug_info = {0};
	char buf[256];
	uint32_t i = 0;
	char name[20] = {0};
	char val[16] = {0};
	char tag[16] = {0};
	int ret = 0;

	memset(buf, 0x00, sizeof(buf));
	i = count;
	if (i >= sizeof(buf))
		i = sizeof(buf) - 1; /* last one for \0 */

	CAM_DEBUGGER_IF_GET_CORRECT_KEY(g_dbg_debug_control);

	if (copy_from_user(buf, buffer, i)) {
		pr_err("fail to get user info\n");
		return -EFAULT;
	}
	buf[i] = '\0';

	ret = camdebugger_strtok(buf, name, val, tag,
		sizeof(name), sizeof(val), sizeof(tag));
	if (ret < 0) {
		g_dbg_readinfo = debug_fun_get(name, NULL);
		return count;
	}
	debug_info = debug_fun_get(name, tag);
	if (!debug_info.write_fun) {
		if (debug_info.ip_info == IP_DCAM)
			debug_info.write_fun = camdebugger_dcam_bypass_write;
		else if (debug_info.ip_info == IP_ISP)
			debug_info.write_fun = camdebugger_isp_bypass_write;
		else {
			pr_info("please input right cmd\n");
			return count;
		}
	}
	debug_info.write_fun(hw, name, val, debug_info.hw_ctx);
	g_dbg_readinfo = debug_info;

	return count;
}

static int camdebugger_input_read(struct seq_file *s, void *unused)
{
	CAM_DEBUGGER_IF_GET_CORRECT_KEY(g_dbg_debug_control);

	if (g_dbg_readinfo.read_fun)
		g_dbg_readinfo.read_fun(s, g_dbg_readinfo.hw_ctx);
	else {
		seq_printf(s, "\n bypass|work sub_block \n");
		seq_printf(s, " echo sub_block:1(bypass)|0(work):target >debug_node \n");
		seq_printf(s, " target - DCAM:(bypass_dcam0|bypass_dcam1) ISP:(bypass_pre0|bypass_cap0|bypass_pre1|bypass_cap1) \n");
		seq_printf(s, " Example: bypass dcam0 blc\n");
		seq_printf(s, " echo blc:1:bypass_dcam0 >debug_node\n");
		seq_printf(s, "\n check sub_block bypass status\n");
		seq_printf(s, " 1.echo target >debug_node \n");
		seq_printf(s, " 2.cat debug_node \n");
		seq_printf(s, "\n check dcam register value\n");
		seq_printf(s, " 1.echo reg_dcam0|reg_dcam1|reg_dcam2|reg_fetch >debug_node \n");
		seq_printf(s, " 2.cat debug_node \n");
		seq_printf(s, "\n check isp cfg_buf value\n");
		seq_printf(s, " 1.echo pre0_buf|cap0_buf|pre1_buf|cap1_buf >debug_node \n");
		seq_printf(s, " 2.cat debug_node \n");
		seq_printf(s, "\n other debug_cmd \n");
		seq_printf(s, " zoom_mode|fbc_ctrl|recovery_bypass|replace_ctrl|replace_file_fmt|replace_switch \n");
		seq_printf(s, " dump_switch|dump_ctrl|dump_count|pyr_dec_bypass|dcam_cap_eof|mem_leak_ctrl \n");
		seq_printf(s, " rawcap_frgb|debug_log_switch|iommu_mode|line_buf_len|offline_dec_bypass \n");
		seq_printf(s, "\n Example: use zoom_mode\n");
		seq_printf(s, " 1.write echo zoom_mode:1 >debug_node \n");
		seq_printf(s, " 2.read  cat debug_node \n");
		seq_printf(s, "\n for specific operation methods of each debug function, please\n");
		seq_printf(s, " 1.echo debug_cmd >debug_node \n");
		seq_printf(s, " 2.cat debug_node \n");
	}

	return 0;
}

static int camdebugger_input_open(struct inode *inode, struct file *file)
{
	return single_open(file, camdebugger_input_read, inode->i_private);
}

static const struct file_operations debug_node_ops = {
	.owner = THIS_MODULE,
	.open = camdebugger_input_open,
	.read = seq_read,
	.write = camdebugger_input_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t camdebugger_debug_control_write(struct file *filp,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;
	char msg[8] = {0};

	if (count > 8 || count < 1)
		return -EINVAL;
	ret = copy_from_user(msg, (void __user *)buffer, count);
	if (ret) {
		pr_err("fail to copy_from_user\n");
		return -EFAULT;
	}

	msg[count - 1] = '\0';
	if (strcmp(msg, DEBUG_CONTROL_KEY) == 0) {
		g_dbg_debug_control = 1;
	} else {
		pr_err("fail to get correct key\n");
		return -EFAULT;
	}

	pr_info("set debug_control %u\n", g_dbg_debug_control);
	return count;
}

static const struct file_operations debug_control_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = camdebugger_debug_control_write,
};

static int camdebugger_cam_init(struct cam_hw_info *hw)
{
	/* folder in /sys/kernel/debug/ */
	const char tb_folder[] = {"sprd_cam"};
	struct dentry *pd = NULL;
	int ret = 0;

	if (!hw) {
		pr_err("fail to get valid para\n");
		return -EFAULT;
	}

	s_p_dentry = debugfs_create_dir(tb_folder, NULL);
	pd = s_p_dentry;
	if (pd == NULL)
		return -ENOMEM;
	/* sub block bypass */
	if (!debugfs_create_file("debug_ctrl", 0664, pd, NULL, &debug_control_ops))
		ret |= BIT(1);
	if (!debugfs_create_file("debug_node", 0664, pd, hw, &debug_node_ops))
		ret |= BIT(2);
	if (ret)
		ret = -ENOMEM;
	pr_info("cam debugfs init ok\n");

	return ret;
}

static int camdebugger_cam_deinit(void)
{
	if (s_p_dentry)
		debugfs_remove_recursive(s_p_dentry);
	s_p_dentry = NULL;
	return 0;
}

int cam_debugger_init(struct cam_hw_info *hw)
{
	int ret = 0;

	ret = camdebugger_cam_init(hw);
	if (ret)
		pr_err("fail to init cam debugfs\n");

	return ret;
}

int cam_debugger_deinit(void)
{
	camdebugger_cam_deinit();

	return 0;
}

