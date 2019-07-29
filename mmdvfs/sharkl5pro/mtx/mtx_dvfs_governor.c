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

#include "mtx_dvfs.h"

static ssize_t get_dvfs_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_dvfs_en);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_dvfs_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  dvfs_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &dvfs_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	DVFS_TRACE("%s: err=%d, dvfs_en=%lu\n", __func__, err, dvfs_en);

	mtx->mtx_dvfs_para.u_dvfs_en = dvfs_en;
	//mtx->dvfs_enable = dvfs_en;
	if (mtx->dvfs_ops != NULL && mtx->dvfs_ops->ip_hw_dvfs_en != NULL && (
		err != 0)) {

		mtx->dvfs_ops->ip_hw_dvfs_en(devfreq,
		mtx->mtx_dvfs_para.u_dvfs_en);
	} else
		pr_err("%s: ip  ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t get_auto_tune_en_show(struct device *dev,
					struct device_attribute *attr, char *
						buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_auto_tune_en);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_auto_tune_en_store(struct device *dev,
	struct device_attribute *attr,  const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  auto_tune_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &auto_tune_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.u_auto_tune_en = auto_tune_en;

	if (mtx->dvfs_ops  !=  NULL &&     mtx->dvfs_ops->ip_auto_tune_en !=
		NULL) {

		mtx->dvfs_ops->ip_auto_tune_en(devfreq,
		mtx->mtx_dvfs_para.u_auto_tune_en);
	} else
		pr_err("%s: ip  ops null\n", __func__);


	err = count;

	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t get_work_freq_show(struct device *dev, struct device_attribute *
	attr,
		char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);

	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_work_freq);
	else
		err = sprintf(buf, "undefined\n");

	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_work_freq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long user_freq;
	int err;


	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	err = sscanf(buf, "%lu\n", &user_freq);
	DVFS_TRACE("%s:err=%d,count=%d", __func__, err, (int)count);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	DVFS_TRACE("%s: dvfs freq %lu", __func__, user_freq);
	if (devfreq->max_freq && user_freq > devfreq->max_freq) {
		pr_err("Change Freq to max\n");
		user_freq = devfreq->max_freq;
	}
	if (devfreq->min_freq && user_freq < devfreq->min_freq) {
		pr_err("Change Freq to min\n");
		user_freq = devfreq->min_freq;
	}

	mtx->mtx_dvfs_para.u_work_freq = user_freq;
	DVFS_TRACE("Work:%d ,idle:%d\n",
		mtx->mtx_dvfs_para.u_work_freq, mtx->mtx_dvfs_para.u_idle_freq);
	err = update_devfreq(devfreq);
	if (err == 0)
		err = count;

	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t get_idle_freq_show(struct device *dev, struct device_attribute *
	attr,
			char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_idle_freq);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_idle_freq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long idle_freq;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	err = sscanf(buf, "%lu\n", &idle_freq);
	if (err == 0) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	DVFS_TRACE("%s: ip ops\n", __func__);
	if (devfreq->max_freq && idle_freq > devfreq->max_freq) {
		pr_err("Change Freq to max\n");
		idle_freq = devfreq->max_freq;
	}
	if (devfreq->min_freq && idle_freq < devfreq->min_freq) {
		pr_err("Change Freq to min\n");
		idle_freq = devfreq->min_freq;
	}
	mtx->mtx_dvfs_para.u_idle_freq = idle_freq;

	err = update_devfreq(devfreq);
	if (err == 0)
		err = count;

	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t get_work_index_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_work_index);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_work_index_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  work_index;
	int err;
	unsigned int is_work;
	struct ip_dvfs_map_cfg dvfs_table[8];

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &work_index);

	if (err != 1 || work_index >= MAX_DVFS_INDEX) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	DVFS_TRACE("%s: count=%d\n", __func__, (int)count);
	DVFS_TRACE("%s: ip ops null,work_index= %lu\n", __func__, work_index);

	if (mtx->dvfs_ops != NULL &&
		mtx->dvfs_ops->set_ip_dvfs_work_index != NULL && (err != 0)) {

		mtx->mtx_dvfs_para.u_work_index = work_index;
		mtx->dvfs_ops->set_ip_dvfs_work_index(devfreq,
			mtx->mtx_dvfs_para.u_work_index);
		err = mtx->dvfs_ops->get_ip_dvfs_table(devfreq, dvfs_table);
		mtx->mtx_dvfs_para.u_work_freq =
			dvfs_table[work_index].clk_freq;
		DVFS_TRACE("u_work_freq:%d\n", mtx->mtx_dvfs_para.u_work_freq);
	} else {
		pr_err("%s: ip  ops null\n", __func__);
		mutex_unlock(&devfreq->lock);
		return err;
	}
	DVFS_TRACE("Update mtx cur frequence1\n");
	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->get_ip_work_or_idle != NULL)
		mtx->dvfs_ops->get_ip_work_or_idle(&is_work);
	if (is_work)
		mtx->freq = mtx->mtx_dvfs_para.u_work_freq;
	else
		mtx->freq = mtx->mtx_dvfs_para.u_idle_freq;
	mutex_unlock(&devfreq->lock);


	return err;
}


static ssize_t get_idle_index_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_idle_index);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_idle_index_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  idle_index;
	unsigned int is_work;
	struct ip_dvfs_map_cfg dvfs_table[8];
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &idle_index);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.u_idle_index = idle_index;

	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->set_ip_dvfs_idle_index  != NULL && (err != 0)) {
		mtx->dvfs_ops->set_ip_dvfs_idle_index(devfreq,
			mtx->mtx_dvfs_para.u_idle_index);
		err = mtx->dvfs_ops->get_ip_dvfs_table(devfreq, dvfs_table);
		mtx->mtx_dvfs_para.u_idle_freq =
			dvfs_table[idle_index].clk_freq;
		DVFS_TRACE("u_idle_freq:%d\n", mtx->mtx_dvfs_para.u_idle_freq);
	} else {
		pr_err("%s: ip  ops null\n", __func__);
		err = count;
		mutex_unlock(&devfreq->lock);
		return err;
	}
	DVFS_TRACE("Update mtx cur frequence1\n");
	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->get_ip_work_or_idle != NULL)
		mtx->dvfs_ops->get_ip_work_or_idle(&is_work);
	if (is_work)
		mtx->freq = mtx->mtx_dvfs_para.u_work_freq;
	else
		mtx->freq = mtx->mtx_dvfs_para.u_idle_freq;
	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_fix_dvfs_show(struct device *dev, struct device_attribute *
	attr,
		char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL)
		err = sprintf(buf, "%d\n", mtx->mtx_dvfs_para.u_fix_volt);
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_fix_dvfs_store(struct device *dev, struct device_attribute *
	attr,
					const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  fix_volt;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &fix_volt);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.u_fix_volt = fix_volt;

	if (mtx->dvfs_ops  !=  NULL &&   mtx->dvfs_ops->set_fix_dvfs_value !=
		NULL) {

		mtx->dvfs_ops->set_fix_dvfs_value(devfreq,
		mtx->mtx_dvfs_para.u_fix_volt);
	} else
		pr_err("%s: ip  ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_dvfs_coffe_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int len = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);

	len = sprintf(buf, "IP_dvfs_coffe_show\n");

	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.auto_tune);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_delay_en);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_en_byp);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_hdsk_en);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.gfree_wait_delay);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.idle_index_def);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.idle_index_def);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.work_index_def);
	len += sprintf(buf+len, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.sw_trig_en);

	mutex_unlock(&devfreq->lock);

	return len;

}


static ssize_t set_dvfs_coffe_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	struct ip_dvfs_coffe dvfs_coffe;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	err = sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d\n",
	&dvfs_coffe.gfree_wait_delay,
	&dvfs_coffe.freq_upd_hdsk_en, &dvfs_coffe.freq_upd_delay_en,
	&dvfs_coffe.freq_upd_en_byp, &dvfs_coffe.sw_trig_en,
	&dvfs_coffe.auto_tune, &dvfs_coffe.work_index_def,
	&dvfs_coffe.idle_index_def);
	mtx->mtx_dvfs_para.ip_coffe = dvfs_coffe;
	if (err != 8) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}

	if (mtx->dvfs_ops  !=  NULL &&   mtx->dvfs_ops->set_ip_dvfs_coffe !=
		NULL) {
		err = mtx->dvfs_ops->set_ip_dvfs_coffe(devfreq,  &dvfs_coffe);
	} else
		pr_err("%s: ip  ops null\n", __func__);


	err = count;

	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t get_ip_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	struct ip_dvfs_status ip_status;
	ssize_t len = 0;
	int ret = 0;
	unsigned int top_volt = 0, mm_volt = 0;


	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);
	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->get_ip_status != NULL) {
		ret = mtx->dvfs_ops->get_ip_status(devfreq, &ip_status);
	} else {
		pr_err("%s: dvfs_read_ops is null\n", __func__);
		len = sprintf(buf, "get_ip_status_show failed\n");
		goto err_exit;
	}
	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->top_current_volt != NULL) {
		ret = mtx->dvfs_ops->top_current_volt(devfreq, &top_volt);
	} else {
		pr_err("%s: dvfs_read_top_volt is null\n", __func__);
		len = sprintf(buf, "get_ip_status_show failed\n");
		goto err_exit;
	}
	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->mm_current_volt != NULL) {
		ret = mtx->dvfs_ops->mm_current_volt(devfreq, &mm_volt);
	} else {
		pr_err("%s: dvfs_read_top_volt is null\n", __func__);
		len = sprintf(buf, "get_ip_status_show failed\n");
		goto err_exit;
	}
	len = sprintf(buf, "mtx_dvfs_read_clk=%d\n", ip_status.current_ip_clk);
	len += sprintf(buf+len, "mtx_dvfs_read_volt=%d\n",
		ip_status.current_sys_volt);
	len += sprintf(buf+len, "mtx_dvfs_read_top_volt=%d\n", top_volt);
	len += sprintf(buf+len, "mtx_dvfs_read_mm_volt=%d\n", mm_volt);

err_exit:
	mutex_unlock(&devfreq->lock);

	return len;
}

static ssize_t get_dvfs_table_info_show(struct device *dev,
	struct device_attribute *attr,   char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	struct ip_dvfs_map_cfg dvfs_table[8];
	ssize_t len = 0;
	int err = 0, i = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);

	if (mtx->dvfs_ops != NULL &&
		mtx->dvfs_ops->get_ip_dvfs_table != NULL) {
		err = mtx->dvfs_ops->get_ip_dvfs_table(devfreq, dvfs_table);

	} else
		pr_err("%s: ip ops null\n", __func__);
	for (i = 0; i < MAX_DVFS_INDEX; i++) {
		len += sprintf(buf+len, "map_index=%d\n",
			dvfs_table[i].map_index);
		len += sprintf(buf+len, "reg_add=%lu\n", dvfs_table[i].reg_add);
		len += sprintf(buf+len, "volt=%d\n", dvfs_table[i].volt);
		len += sprintf(buf+len, "clk=%d\n", dvfs_table[i].clk);
		len += sprintf(buf+len, "fdiv_denom=%d\n",
			dvfs_table[i].fdiv_denom);
		len += sprintf(buf+len, "fdiv_num=%d\n",
			dvfs_table[i].fdiv_num);
		len += sprintf(buf+len, "axi_index=%d\n",
			dvfs_table[i].axi_index);
		len += sprintf(buf+len, "mtx_index=%d\n",
			dvfs_table[i].mtx_index);
	}


	mutex_unlock(&devfreq->lock);

	return len;
}

static ssize_t set_dvfs_table_info_store(struct device *dev,
		struct device_attribute *attr,   const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	struct ip_dvfs_map_cfg dvfs_table;
	uint32_t  map_index;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
		/* To do */
	err = sscanf(buf, " %d,%lu,%d,%d,%d,%d,%d,%d,\n",
	&dvfs_table.map_index, &dvfs_table.reg_add
	, &dvfs_table.volt, &dvfs_table.clk, &dvfs_table.fdiv_denom,
	&dvfs_table.fdiv_num, &dvfs_table.axi_index, &dvfs_table.mtx_index);
	if (err != 8) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}

	map_index = dvfs_table.map_index;
	mtx->mtx_dvfs_para.ip_dvfs_map[map_index] = dvfs_table;

	if (mtx->dvfs_ops  !=  NULL &&   mtx->dvfs_ops->set_ip_dvfs_table !=
		NULL) {

		err = mtx->dvfs_ops->set_ip_dvfs_table(devfreq,  &dvfs_table);
	} else
		pr_err("%s: ip ops null\n", __func__);


	err = count;

	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t get_gfree_wait_delay_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.gfree_wait_delay);
	} else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_gfree_wait_delay_store(struct device *dev,
		struct device_attribute *attr,   const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  gfree_wait_delay;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, " %lu\n", &gfree_wait_delay);
	DVFS_TRACE("%s:err=%d,gfree_wait_delay=%lu,count=%d",
	__func__, err, gfree_wait_delay, (int)count);

	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.gfree_wait_delay = gfree_wait_delay;

	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->set_ip_gfree_wait_delay != NULL) {

		mtx->dvfs_ops->set_ip_gfree_wait_delay(
		mtx->mtx_dvfs_para.ip_coffe.gfree_wait_delay);
	} else
		pr_err("%s: ip ops null\n", __func__);

	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_freq_upd_hdsk_en_store(struct device *dev,
		struct device_attribute *attr,    const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  freq_upd_hdsk_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &freq_upd_hdsk_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.freq_upd_hdsk_en = freq_upd_hdsk_en;

	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->set_ip_freq_upd_hdsk_en != NULL) {

		mtx->dvfs_ops->set_ip_freq_upd_hdsk_en(
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_hdsk_en);
	} else
		pr_err("%s: ip ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_freq_upd_hdsk_en_show(struct device *dev,
	struct device_attribute *attr,  char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_hdsk_en);
		}
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_freq_upd_delay_en_store(struct device *dev,
		struct device_attribute *attr,   const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  freq_upd_delay_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, " %lu\n", &freq_upd_delay_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.freq_upd_delay_en = freq_upd_delay_en;

	if (mtx->dvfs_ops  !=  NULL
		&&  mtx->dvfs_ops->set_ip_freq_upd_delay_en != NULL) {

		mtx->dvfs_ops->set_ip_freq_upd_delay_en(
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_delay_en);
	} else
		pr_err("%s: ip ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_freq_upd_delay_en_show(struct device *dev,
		struct device_attribute *attr,    char *buf)

{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_delay_en);
		}
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_freq_upd_en_byp_store(struct device *dev,
		struct device_attribute *attr,  const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  freq_upd_en_byp;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, " %lu\n", &freq_upd_en_byp);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.freq_upd_en_byp = freq_upd_en_byp;

	if (mtx->dvfs_ops  !=  NULL &&
		mtx->dvfs_ops->set_ip_freq_upd_en_byp != NULL) {

		mtx->dvfs_ops->set_ip_freq_upd_en_byp(
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_en_byp);
	} else
		pr_err("%s: ip ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_freq_upd_en_byp_show(struct device *dev,
		struct device_attribute *attr,  char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.freq_upd_en_byp);
		}
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}

static ssize_t set_sw_trig_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long  sw_trig_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &sw_trig_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.freq_upd_en_byp = sw_trig_en;

	if (mtx->dvfs_ops  !=  NULL && mtx->dvfs_ops->set_ip_dvfs_swtrig_en !=
		NULL)
		mtx->dvfs_ops->set_ip_dvfs_swtrig_en(sw_trig_en);
	else
		pr_err("%s: ip ops null\n", __func__);


	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}


static ssize_t get_sw_trig_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.sw_trig_en);
		}
	else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;

}
static ssize_t get_idle_switch_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	int err = 0;

	mtx = dev_get_drvdata(devfreq->dev.parent);
	mutex_lock(&devfreq->lock);
	if (mtx != NULL) {
		err = sprintf(buf, "%d\n",
		mtx->mtx_dvfs_para.ip_coffe.idle_switch_en);
	} else
		err = sprintf(buf, "undefined\n");
	mutex_unlock(&devfreq->lock);

	return err;
}

static ssize_t set_idle_switch_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct devfreq *devfreq = to_devfreq(dev);
	struct mtx_dvfs *mtx;
	unsigned long idle_switch_en;
	int err;

	mtx = dev_get_drvdata(devfreq->dev.parent);

	mutex_lock(&devfreq->lock);

	err = sscanf(buf, "%lu\n", &idle_switch_en);
	if (err != 1) {
		mutex_unlock(&devfreq->lock);
		return -EINVAL;
	}
	mtx->mtx_dvfs_para.ip_coffe.idle_switch_en = idle_switch_en;
	if (mtx->dvfs_ops  !=  NULL && mtx->dvfs_ops->set_ip_dfs_idle_disable !=
		NULL)
		mtx->dvfs_ops->set_ip_dfs_idle_disable(idle_switch_en);
	else
		pr_err("%s: ip ops null\n", __func__);

	err = count;
	mutex_unlock(&devfreq->lock);

	return err;
}
/*sys for gov_entries*/
static DEVICE_ATTR(set_hw_dvfs_en, 0644, get_dvfs_enable_show,
			set_dvfs_enable_store);
static DEVICE_ATTR(set_auto_tune_en, 0644, get_auto_tune_en_show,
			set_auto_tune_en_store);
static DEVICE_ATTR(set_dvfs_coffe, 0644, get_dvfs_coffe_show,
	set_dvfs_coffe_store);
static DEVICE_ATTR(get_ip_status, 0644, get_ip_status_show,
	NULL);
static DEVICE_ATTR(set_work_freq, 0644, get_work_freq_show,
	set_work_freq_store);
static DEVICE_ATTR(set_idle_freq, 0644, get_idle_freq_show,
	set_idle_freq_store);
static DEVICE_ATTR(set_work_index, 0644, get_work_index_show,
			set_work_index_store);
static DEVICE_ATTR(set_idle_index, 0644, get_idle_index_show,
			set_idle_index_store);
static DEVICE_ATTR(set_fix_dvfs_value, 0644, get_fix_dvfs_show,
	set_fix_dvfs_store);
static DEVICE_ATTR(get_dvfs_table_info, 0644, get_dvfs_table_info_show,
			set_dvfs_table_info_store);
/*sys for coeff_entries*/
static DEVICE_ATTR(set_gfree_wait_delay, 0644, get_gfree_wait_delay_show,
			set_gfree_wait_delay_store);
static DEVICE_ATTR(set_freq_upd_hdsk_en, 0644, get_freq_upd_hdsk_en_show,
			set_freq_upd_hdsk_en_store);
static DEVICE_ATTR(set_freq_upd_delay_en, 0644, get_freq_upd_delay_en_show,
			set_freq_upd_delay_en_store);
static DEVICE_ATTR(set_freq_upd_en_byp, 0644, get_freq_upd_en_byp_show,
			set_freq_upd_en_byp_store);
static DEVICE_ATTR(set_sw_trig_en, 0644, get_sw_trig_en_show,
			set_sw_trig_en_store);
static DEVICE_ATTR(set_idle_switch_en, 0644, get_idle_switch_en_show,
			set_idle_switch_en_store);


static struct attribute *dev_entries[] = {

	&dev_attr_set_hw_dvfs_en.attr,
	&dev_attr_set_auto_tune_en.attr,
	&dev_attr_set_dvfs_coffe.attr,
	&dev_attr_get_ip_status.attr,
	&dev_attr_set_work_freq.attr,
	&dev_attr_set_idle_freq.attr,
	&dev_attr_set_work_index.attr,
	&dev_attr_set_idle_index.attr,
	&dev_attr_set_fix_dvfs_value.attr,
	&dev_attr_get_dvfs_table_info.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.name    = "mtx_governor",
	.attrs    = dev_entries,
};

static struct attribute *coeff_entries[] = {

	&dev_attr_set_gfree_wait_delay.attr,
	&dev_attr_set_freq_upd_hdsk_en.attr,
	&dev_attr_set_freq_upd_delay_en.attr,
	&dev_attr_set_freq_upd_en_byp.attr,

	&dev_attr_set_sw_trig_en.attr,
	&dev_attr_set_auto_tune_en.attr,
	&dev_attr_set_idle_switch_en.attr,
	NULL,
};

static struct attribute_group coeff_attr_group = {
	.name    = "mtx_coeff",
	.attrs    = coeff_entries,
};

static void userspace_exit(struct devfreq *devfreq)
{
	/*
	 * Remove the sysfs entry, unless this is being called after
	 * device_del(), which should have done this already via kobject_del().
	 */
	if (devfreq->dev.kobj.sd) {
		sysfs_remove_group(&devfreq->dev.kobj, &dev_attr_group);
		sysfs_remove_group(&devfreq->dev.kobj, &coeff_attr_group);
	}
}

static int userspace_init(struct devfreq *devfreq)
{
	int err = 0;

	struct mtx_dvfs *mtx = dev_get_drvdata(devfreq->dev.parent);

	mtx->dvfs_ops = get_ip_dvfs_ops("MTX_DVFS_OPS");

	err = sysfs_create_group(&devfreq->dev.kobj, &dev_attr_group);
	err = sysfs_create_group(&devfreq->dev.kobj, &coeff_attr_group);

	return err;
}

static int mtx_dvfs_gov_get_target(struct devfreq *devfreq,
			unsigned long *freq)
{
	struct mtx_dvfs *mtx = dev_get_drvdata(devfreq->dev.parent);

	DVFS_TRACE("devfreq_governor-->get_target_freq\n");

	if (mtx->dvfs_enable) {
		unsigned long adjusted_freq = *freq;



		if (devfreq->max_freq && adjusted_freq > devfreq->max_freq)
			adjusted_freq = devfreq->max_freq;

		if (devfreq->min_freq && adjusted_freq < devfreq->min_freq)
			adjusted_freq = devfreq->min_freq;
	*freq = adjusted_freq;
	} else
		*freq = devfreq->max_freq; /* No user freq specified yet */
	DVFS_TRACE("dvfs *freq %lu", *freq);
	return 0;
}

static int mtx_dvfs_gov_event_handler(struct devfreq *devfreq,
			unsigned int event, void *data)
{
	int ret = 0;

	DVFS_TRACE("devfreq_governor-->event_handler(%d)\n", event);
	switch (event) {
	case DEVFREQ_GOV_START:
		ret = userspace_init(devfreq);
		break;
	case DEVFREQ_GOV_STOP:
		userspace_exit(devfreq);
		break;
	default:
		break;
	}

	return ret;
}

struct devfreq_governor mtx_dvfs_gov = {
	.name = "mtx_dvfs",
	.get_target_freq = mtx_dvfs_gov_get_target,
	.event_handler = mtx_dvfs_gov_event_handler,
};

