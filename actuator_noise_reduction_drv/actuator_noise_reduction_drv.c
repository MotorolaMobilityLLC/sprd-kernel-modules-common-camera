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

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/i2c.h>
#include "sprd_sensor_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "ACTUATOR_DRV:: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define ACTUATOR_DEVICE_NAME       "actuator_control"

#define AF_I2C_SLAVE_ADDR 0x18

/* Structure Definitions */
struct actuator_device {
	struct miscdevice md;
	const struct actuator_driver_ops *ops;
	void *driver_data;
};

/* Static Variables Definitions */
static struct platform_device *actuator_pdev;

static void actuator_power_on(void)
{
	printk("actuator_power_on\n");
	sprd_sensor_set_voltage_by_gpio(SPRD_SENSOR_MAIN_ID_E, SPRD_SENSOR_VDD_2800MV_VAL, SPRD_SENSOR_MOT_GPIO_TAG_E);
	sprd_sensor_set_voltage(SPRD_SENSOR_MAIN_ID_E, SPRD_SENSOR_VDD_2800MV_VAL, SENSOR_REGULATOR_CAMMOT_ID_E);
}

static void actuator_power_off(void)
{
	printk("actuator_power_off\n");
	sprd_sensor_set_voltage_by_gpio(SPRD_SENSOR_MAIN_ID_E, 0, SPRD_SENSOR_MOT_GPIO_TAG_E);
	sprd_sensor_set_voltage(SPRD_SENSOR_MAIN_ID_E, 0, SENSOR_REGULATOR_CAMMOT_ID_E);
}

static int actuator_i2c_write(u8 *pwrite_data,u16 write_length, struct i2c_client* actuator_client)
{
	int i4RetValue = 0;
	struct i2c_msg msgs[1];

	msgs[0].addr  = AF_I2C_SLAVE_ADDR >> 1;
	msgs[0].flags = 0;
	msgs[0].len   = write_length;
	msgs[0].buf   = pwrite_data;

	i4RetValue = i2c_transfer(actuator_client->adapter, msgs, 1);
	printk("i4RetValue = %d in actuator_i2c_write\n", i4RetValue);
	if (i4RetValue != 1) {
		pr_err("I2C send failed!!\n");
		return -1;
	}
	return 0;
}

static void actuator_set_pos(void)
{
	struct i2c_client* actuator_client = get_actuator_i2c_client();
	int32_t code = 0x00; /* DAC = 00 */

	char pusendcmd0[2] = {0x02, 0x02}; /* Ringing Setting On */
	char pusendcmd1[2] = {0x06, 0x40}; /* SAC mode selection */
	char pusendcmd2[2] = {0x07, 0x6F}; /* Tvib Setting */

	char dac0[2] = {0x03, ((code & 0x300) >> 8)};
	char dac1[2] = {0x04, (code & 0xff)};

	actuator_i2c_write(pusendcmd0, 2, actuator_client);
	msleep(1);
	actuator_i2c_write(pusendcmd1, 2, actuator_client);
	msleep(1);
	actuator_i2c_write(pusendcmd2, 2, actuator_client);
	msleep(5);
	actuator_i2c_write(dac0, 2, actuator_client);
	actuator_i2c_write(dac1, 2, actuator_client);

	actuator_client = NULL;
}


static int actuator_noise_reduction_open(struct inode *node, struct file *file)
{
	struct actuator_device *actuator_dev;
	struct miscdevice *md = file->private_data;

	printk("actuator_noise_reduction_open\n");
	if (!md)
		return -EFAULT;

	actuator_dev = md->this_device->platform_data;
	file->private_data = (void *)actuator_dev;

	actuator_power_on();
	msleep(5);
	actuator_set_pos();

	return 0;
}

static int actuator_noise_reduction_release(struct inode *node, struct file *file)
{
	printk("actuator_noise_reduction_release\n");
	file->private_data = NULL;

	return 0;
}

static long actuator_noise_reduction_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct actuator_device* actuator_dev;

	printk("actuator_noise_reduction_ioctl start");
	actuator_dev = file->private_data;
	if (!actuator_dev)
		return -EFAULT;

	actuator_power_off();

	return 0;
}

static const struct file_operations actuator_fops = {
	.owner = THIS_MODULE,
	.open = actuator_noise_reduction_open,
	.unlocked_ioctl = actuator_noise_reduction_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = actuator_noise_reduction_ioctl,
#endif
	.release = actuator_noise_reduction_release,
};

static int actuator_noise_reduction_probe(struct platform_device *pdev)
{
	int ret;
	struct actuator_device *actuator_dev;
	
	actuator_dev = devm_kzalloc(&pdev->dev, sizeof(*actuator_dev), GFP_KERNEL);
	if (!actuator_dev)
		return -ENOMEM;

	actuator_dev->md.minor = MISC_DYNAMIC_MINOR;
	actuator_dev->md.name = ACTUATOR_DEVICE_NAME;
	actuator_dev->md.fops = &actuator_fops;
	actuator_dev->md.parent = NULL;
	ret = misc_register(&actuator_dev->md);
	if (ret) {
		pr_err("failed to register misc devices\n");
		goto exit;
	}

	actuator_dev->md.this_device->platform_data = (void *)actuator_dev;
	platform_set_drvdata(pdev, (void *)actuator_dev);

	return 0;

exit:
	pr_err("failed to probe actuator driver\n");
	return ret;
}

static int actuator_noise_reduction_remove(struct platform_device *pdev)
{
	struct actuator_device *actuator_dev = platform_get_drvdata(pdev);

	misc_deregister(&actuator_dev->md);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver actuator_noise_reduction_drvier = {
	.probe = actuator_noise_reduction_probe,
	.remove = actuator_noise_reduction_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = ACTUATOR_DEVICE_NAME,
	},
};

static int __init actuator_noise_reduction_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&actuator_noise_reduction_drvier);
	if (ret != 0) {
		pr_err("failed to register actuator driver\n");
		goto exit;
	}

	actuator_pdev = platform_device_register_simple(ACTUATOR_DEVICE_NAME, -1, NULL, 0);
	if (IS_ERR_OR_NULL(actuator_pdev)) {
		ret = PTR_ERR(actuator_pdev);
		platform_driver_unregister(&actuator_noise_reduction_drvier);
		goto exit;
	}

exit:
	return ret;
}

static void __exit actuator_noise_reduction_exit(void)
{
	platform_device_unregister(actuator_pdev);
	platform_driver_unregister(&actuator_noise_reduction_drvier);
}

module_init(actuator_noise_reduction_init);
module_exit(actuator_noise_reduction_exit);
MODULE_DESCRIPTION("Actuator Control Driver");
MODULE_LICENSE("GPL");
