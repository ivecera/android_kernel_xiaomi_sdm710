/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
** File:
**     tas2562-misc.c
**
** Description:
**     misc driver for Texas Instruments TAS2562 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include "tas2562.h"
#include "tas2562-misc.h"
#include <linux/dma-mapping.h>

static int g_logEnable = 1;
static struct tas2562_priv *g_tas2562;

static int tas2562_file_open(struct inode *inode, struct file *file)
{
	struct tas2562_priv *tas_priv = g_tas2562;

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)tas_priv;

	if (g_logEnable)
		dev_info(tas_priv->dev, "%s\n", __func__);
	return 0;
}

static int tas2562_file_release(struct inode *inode, struct file *file)
{
	struct tas2562_priv *tas_priv = (struct tas2562_priv *)file->private_data;

	if (g_logEnable)
		dev_info(tas_priv->dev, "%s\n", __func__);

	file->private_data = (void *)NULL;
	module_put(THIS_MODULE);

	return 0;
}

static ssize_t tas2562_file_read(struct file *file, char *dest, size_t count, loff_t *ppos)
{
	struct tas2562_priv *tas_priv = (struct tas2562_priv *)file->private_data;
	int ret = 0;
	unsigned int value = 0;
	unsigned char *buf = NULL;

	mutex_lock(&tas_priv->file_lock);

	switch (tas_priv->debug_cmd) {
	case TIAUDIO_CMD_REG_READ:
	{
		if (g_logEnable)
			dev_info(tas_priv->dev,
				"TIAUDIO_CMD_REG_READ: current_reg = 0x%x, count=%d\n",
				tas_priv->cur_reg, (int)count);
		if (count == 1) {
			ret = tas_priv->read(tas_priv, tas_priv->cur_reg, &value);
			if (ret < 0) {
				dev_err(tas_priv->dev, "dev read fail %d\n", ret);
				break;
			}

			if (g_logEnable)
				dev_info(tas_priv->dev, "TIAUDIO_CMD_REG_READ: nValue=0x%x, value=0x%x\n",
					 value, (u8)value);
			ret = copy_to_user(dest, &value, 1);
			if (ret != 0) {
				/* Failed to copy all the data, exit */
				dev_err(tas_priv->dev, "copy to user fail %d\n", ret);
			}
		} else if (count > 1) {
			buf = kzalloc(count, GFP_KERNEL);
			if (buf != NULL) {
				ret = tas_priv->bulk_read(tas_priv, tas_priv->cur_reg, buf, count);
				if (ret < 0) {
					dev_err(tas_priv->dev, "dev bulk read fail %d\n", ret);
				} else {
					ret = copy_to_user(dest, buf, count);
					if (ret != 0) {
						/* Failed to copy all the data, exit */
						dev_err(tas_priv->dev, "copy to user fail %d\n", ret);
					}
				}

				kfree(buf);
			} else {
				dev_err(tas_priv->dev, "read no mem\n");
			}
		}
	}
	break;
	}
	tas_priv->debug_cmd = 0;

	mutex_unlock(&tas_priv->file_lock);
	return count;
}

static ssize_t tas2562_file_write(struct file *file, const char *src, size_t count, loff_t *ppos)
{
	struct tas2562_priv *tas_priv = (struct tas2562_priv *)file->private_data;
	int ret = 0;
	unsigned char *buf = NULL;
	unsigned int reg = 0;
	unsigned int len = 0;

	mutex_lock(&tas_priv->file_lock);

	buf = kzalloc(count, GFP_KERNEL);
	if (buf == NULL) {
		dev_err(tas_priv->dev, "write no mem\n");
		goto err;
	}

	ret = copy_from_user(buf, src, count);
	if (ret != 0) {
		dev_err(tas_priv->dev, "copy_from_user failed.\n");
		goto err;
	}

	tas_priv->debug_cmd = buf[0];
	switch (tas_priv->debug_cmd) {
	case TIAUDIO_CMD_REG_WITE:
	if (count > 5) {
		reg = ((unsigned int)buf[1] << 24) +
			((unsigned int)buf[2] << 16) +
			((unsigned int)buf[3] << 8) +
			(unsigned int)buf[4];
		len = count - 5;
		if (len == 1) {
			ret = tas_priv->write(tas_priv, reg, buf[5]);
			if (g_logEnable)
				dev_info(tas_priv->dev,
					"TIAUDIO_CMD_REG_WITE, Reg=0x%x, Val=0x%x\n",
					reg, buf[5]);
		} else {
			ret = tas_priv->bulk_write(tas_priv, reg, &buf[5], len);
		}
	} else {
		dev_err(tas_priv->dev, "%s, write len fail, count=%d.\n",
			__func__, (int)count);
	}
	tas_priv->debug_cmd = 0;
	break;

	case TIAUDIO_CMD_REG_READ:
	if (count == 5) {
		tas_priv->cur_reg = ((unsigned int)buf[1] << 24) +
			((unsigned int)buf[2] << 16) +
			((unsigned int)buf[3] << 8) +
			(unsigned int)buf[4];
		if (g_logEnable) {
			dev_info(tas_priv->dev,
				"TIAUDIO_CMD_REG_READ, whole=0x%x\n",
				tas_priv->cur_reg);
		}
	} else {
		dev_err(tas_priv->dev, "read len fail.\n");
	}
	break;
	}
err:
	if (buf != NULL)
		kfree(buf);

	mutex_unlock(&tas_priv->file_lock);

	return count;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = tas2562_file_read,
	.write = tas2562_file_write,
	.unlocked_ioctl = NULL,
	.open = tas2562_file_open,
	.release = tas2562_file_release,
};

#define MODULE_NAME	"tas2562"
static struct miscdevice tas2562_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MODULE_NAME,
	.fops = &fops,
};

int tas2562_register_misc(struct tas2562_priv *tas_priv)
{
	int ret = 0;

	g_tas2562 = tas_priv;
	ret = misc_register(&tas2562_misc);
	if (ret) {
		dev_err(tas_priv->dev, "TAS2562 misc fail: %d\n", ret);
	}

	dev_info(tas_priv->dev, "%s, leave\n", __func__);

	return ret;
}

int tas2562_deregister_misc(struct tas2562_priv *tas_priv)
{
	misc_deregister(&tas2562_misc);
	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2562 Misc Smart Amplifier driver");
MODULE_LICENSE("GPL v2");
