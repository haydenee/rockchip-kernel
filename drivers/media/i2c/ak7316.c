// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include "linux/debugfs.h"
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME "ak7316"
#define AK7316_I2C_SLAVE_ADDR 0x18

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define AK7316_NAME				"ak7316"
#define AK7316_MAX_FOCUS_POS			1023
#define AK7316_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AK7316_FOCUS_STEPS			1
#define AK7316_SET_POSITION_ADDR		0x00

#define AK7316_CMD_DELAY			0xff
#define AK7316_CTRL_DELAY_US			7000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AK7316_MOVE_STEPS			100
#define AK7316_MOVE_DELAY_US			5000

#define AK7316_VIN_MIN			(2800000)
#define AK7316_VIN_MAX			(2800000)
#define AK7316_VDD_MIN			(1804000)
#define AK7316_VDD_MAX			(1804000)
#define AK7316_CMD_DELAY_US		(5000)
static bool auto_clip_enabled;
static bool pid_parm_setting_done = false;

static u16 g_origin_pos = AK7316_MAX_FOCUS_POS / 2;
static u16 g_last_pos = AK7316_MAX_FOCUS_POS / 2;

/* ak7316 device structure */
struct ak7316_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	// struct regulator *vin;
	// struct regulator *vdd;
	// struct pinctrl *vcamaf_pinctrl;
	// struct pinctrl_state *vcamaf_on;
	// struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;

	/* debugfs */
    struct dentry *debugfs_dir;
    u16 debug_position;
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static inline struct ak7316_device *to_ak7316_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7316_device, ctrls);
}

static inline struct ak7316_device *sd_to_ak7316_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7316_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


static int ak7316_set_position(struct ak7316_device *ak7316, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);

	return i2c_smbus_write_word_data(client, AK7316_SET_POSITION_ADDR,
					 swab16(val << 6));
}

static int ak7316_goto_last_pos(struct ak7316_device *ak7316)
{
	int ret, val = g_origin_pos, diff_dac, nStep_count, i;

	diff_dac = g_last_pos - val;
	if (diff_dac == 0) {
		return 0;
	}
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
			AK7316_MOVE_STEPS;
	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (AK7316_MOVE_STEPS*(-1)) : AK7316_MOVE_STEPS);
		ret = ak7316_set_position(ak7316, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d", __func__, ret);
		}
		usleep_range(AK7316_MOVE_DELAY_US, AK7316_MOVE_DELAY_US + 1000);
	}

	return 0;
}

static int ak7316_shaking_lock_on(struct ak7316_device *ak7316)
{
	int ret = -1;

	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);

	usleep_range(AK7316_CMD_DELAY_US, AK7316_CMD_DELAY_US + 100);
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x40);
	usleep_range(AK7316_CMD_DELAY_US, AK7316_CMD_DELAY_US + 100);
	if (ret) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
	}

	ret = i2c_smbus_read_byte_data(client, 0x09);
	if (ret < 0) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
	} else {
		if (ret != 0x80) {
			ret = i2c_smbus_write_byte_data(client, 0xAE, 0x3B);
			ret = i2c_smbus_write_byte_data(client, 0x09, 0x80);
			ret = i2c_smbus_write_byte_data(client, 0x03, 0x01);
			if (ret) {
				LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
				return ret;
			}
			msleep(110);
		}
		auto_clip_enabled = true;
	}

	ret = i2c_smbus_write_byte_data(client, 0x02, 0x20);
	if (ret) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
	}
	return ret;
}

static int ak7316_shaking_lock_off(struct ak7316_device *ak7316)
{
	int ret = -1;

	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);

	if (auto_clip_enabled) {
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x40);
		usleep_range(AK7316_CMD_DELAY_US, AK7316_CMD_DELAY_US + 100);
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x40);
		usleep_range(AK7316_CMD_DELAY_US, AK7316_CMD_DELAY_US + 100);
		if (ret) {
			LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
			return ret;
		}
	}

	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
	if (ret) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
	}

	return ret;
}

static int ak7316_shaking_lock(struct ak7316_device *ak7316, u16 val)
{
	if (val) {
		return ak7316_shaking_lock_on(ak7316);
	} else {
		return ak7316_shaking_lock_off(ak7316);
	}
}

static int ak7316_release(struct ak7316_device *ak7316)
{
	int ret, val;
	int diff_dac = 0;
	int nStep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);

	diff_dac = AK7316_ORIGIN_FOCUS_POS - ak7316->focus->val;

	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		AK7316_MOVE_STEPS;

	val = ak7316->focus->val;

	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (AK7316_MOVE_STEPS*(-1)) : AK7316_MOVE_STEPS);

		ret = ak7316_set_position(ak7316, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(AK7316_MOVE_DELAY_US,
			     AK7316_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = ak7316_set_position(ak7316, AK7316_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	ak7316->active = false;

	LOG_INF("-\n");

	return 0;
}

static int ak7316_init(struct ak7316_device *ak7316)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);
	int ret = 0, retry = 5;

	LOG_INF("+\n");

	// client->addr = AK7316_I2C_SLAVE_ADDR >> 1;
	//ret = i2c_smbus_read_byte_data(client, 0x02);

	LOG_INF("Check HW version: %x\n", ret);

	auto_clip_enabled = false;

	ret = i2c_smbus_write_byte_data(client, 0x02, 0x40); // standy
	usleep_range(2800, 3000); // wait over 2.71ms

	// double check Reg(0x16) == 0x20
	if (!pid_parm_setting_done) {
		LOG_INF("update pid arg");
		do {
			ret = i2c_smbus_write_byte_data(client, 0x16, 0x20);
		} while(ret < 0 && (--retry > 0));
	}

	ret = i2c_smbus_read_word_data(client, 0x84);
	if (ret < 0) {
		LOG_INF("%s I2C failure: %d", __func__, ret);
		g_origin_pos = AK7316_MAX_FOCUS_POS / 2;
	} else {
		g_origin_pos = swab16(ret & 0xFFFF) >> 6;
	}
	if (g_origin_pos > AK7316_MAX_FOCUS_POS) {
		g_origin_pos = AK7316_MAX_FOCUS_POS;
	}
	ak7316_set_position(ak7316, g_origin_pos);

	/* 00:active mode , 10:Standby mode , x1:Sleep mode */
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
	ak7316->active = true;
	ak7316->debug_position = g_origin_pos;
	ak7316_goto_last_pos(ak7316);

	LOG_INF("-\n");

	return 0;
}

/* Power handling */
static int ak7316_power_off(struct ak7316_device *ak7316)
{
	int ret, ret2;

	LOG_INF("+\n");

	ret = ak7316_release(ak7316);
	if (ret)
		LOG_INF("ak7316 release failed!\n");

	// ret = regulator_disable(ak7316->vin);
	// ret2 = regulator_disable(ak7316->vdd);
	// if (ret || ret2)
	// 	return ret ? ret : ret2;

	// if (ak7316->vcamaf_pinctrl && ak7316->vcamaf_off)
	// 	ret = pinctrl_select_state(ak7316->vcamaf_pinctrl,
	// 				ak7316->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int ak7316_power_on(struct ak7316_device *ak7316)
{
	int ret, min, max;

	LOG_INF("+\n");

	// min = AK7316_VIN_MIN;
	// max = AK7316_VIN_MAX;
	// ret = regulator_set_voltage(ak7316->vin, min, max);
	// if (ret) {
	// 	LOG_INF("regulator_set_voltage(vin), min(%d - %d), ret(%d)(fail)\n",
	// 		min, max, ret);
	// }
	// ret = regulator_enable(ak7316->vin);
	// if (ret < 0) {
	// 	LOG_INF("regulator_enable(vin), ret(%d)(fail)\n", ret);
	// 	return ret;
	// }

	// min = AK7316_VDD_MIN;
	// max = AK7316_VDD_MAX;
	// ret = regulator_set_voltage(ak7316->vdd, min, max);
	// if (ret) {
	// 	LOG_INF("regulator_set_voltage(vin), min(%d - %d), ret(%d)(fail)\n",
	// 		min, max, ret);
	// }
	// ret = regulator_enable(ak7316->vdd);
	// if (ret < 0) {
	// 	LOG_INF("regulator_enable(VDD),ret(%d)(fail)\n", ret);
	// 	regulator_disable(ak7316->vin);
	// 	return ret;
	// }

	// if (ak7316->vcamaf_pinctrl && ak7316->vcamaf_on)
	// 	ret = pinctrl_select_state(ak7316->vcamaf_pinctrl,
	// 				ak7316->vcamaf_on);

	// if (ret < 0)
	// 	return ret;

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(AK7316_CTRL_DELAY_US, AK7316_CTRL_DELAY_US + 100);

	ret = ak7316_init(ak7316);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	// regulator_disable(ak7316->vin);
	// regulator_disable(ak7316->vdd);
	// if (ak7316->vcamaf_pinctrl && ak7316->vcamaf_off) {
	// 	pinctrl_select_state(ak7316->vcamaf_pinctrl,
	// 			ak7316->vcamaf_off);
	// }

	return ret;
}

static int ak7316_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct ak7316_device *ak7316 = to_ak7316_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = ak7316_set_position(ak7316, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		g_last_pos = ctrl->val;
	} else if (ctrl->id == V4L2_CID_FOCUS_AUTO) {
		LOG_INF("lock(%d)\n", ctrl->val);
		ret = ak7316_shaking_lock(ak7316, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops ak7316_vcm_ctrl_ops = {
	.s_ctrl = ak7316_set_ctrl,
};

static int ak7316_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct ak7316_device *ak7316 = sd_to_ak7316_vcm(sd);

	ret = ak7316_power_on(ak7316);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int ak7316_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ak7316_device *ak7316 = sd_to_ak7316_vcm(sd);

	ak7316_power_off(ak7316);
	return 0;
}

static long ak7316_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct ak7316_device *ak7316 = sd_to_ak7316_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);
	LOG_INF("+\n");
	client->addr = AK7316_I2C_SLAVE_ADDR >> 1;

	switch (cmd) {
	case VCM_IOC_POWER_ON:
	{
		// customized area
		/* 00:active mode , 10:Standby mode , x1:Sleep mode */
		if (ak7316->active)
			return 0;
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
		if (ret) {
			LOG_INF("I2C failure!!!\n");
		} else {
			ak7316->active = true;
			LOG_INF("stand by mode, power on !!!!!!!!!!!!\n");
		}
	}
	break;
	case VCM_IOC_POWER_OFF:
	{
		// customized area
		if (!ak7316->active)
			return 0;
		ret = i2c_smbus_write_byte_data(client, 0x02, 0x40);
		if (ret) {
			LOG_INF("I2C failure!!!\n");
		} else {
			ak7316->active = false;
			LOG_INF("stand by mode, power off !!!!!!!!!!!!\n");
		}
	}
	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_internal_ops ak7316_int_ops = {
	.open = ak7316_open,
	.close = ak7316_close,
};

static struct v4l2_subdev_core_ops ak7316_ops_core = {
	.ioctl = ak7316_ops_core_ioctl,
};

static const struct v4l2_subdev_ops ak7316_ops = {
	.core = &ak7316_ops_core,
 };

static void ak7316_subdev_cleanup(struct ak7316_device *ak7316)
{
	v4l2_async_unregister_subdev(&ak7316->sd);
	v4l2_ctrl_handler_free(&ak7316->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&ak7316->sd.entity);
#endif
}

static int ak7316_init_controls(struct ak7316_device *ak7316)
{
	struct v4l2_ctrl_handler *hdl = &ak7316->ctrls;
	const struct v4l2_ctrl_ops *ops = &ak7316_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	ak7316->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, AK7316_MAX_FOCUS_POS, AK7316_FOCUS_STEPS, 0);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_AUTO, 0, 0, 0, 0);

	if (hdl->error)
		return hdl->error;

	ak7316->sd.ctrl_handler = hdl;

	return 0;
}

// static int ak7316_power_on2(struct ak7316_device *ak7316)
// {
// 	int ret, min, max;

// 	LOG_INF("+");

// 	min = AK7316_VIN_MIN;
// 	max = AK7316_VIN_MAX;
// 	ret = regulator_set_voltage(ak7316->vin, min, max);
// 	if (ret) {
// 		LOG_INF("regulator_set_voltage(vin), min(%d - %d), ret(%d)(fail)\n",
// 			min, max, ret);
// 	}
// 	ret = regulator_enable(ak7316->vin);
// 	if (ret < 0) {
// 		LOG_INF("regulator_enable(vin), ret(%d)(fail)\n", ret);
// 		goto fail2;
// 	}
// 	min = AK7316_VDD_MIN;
// 	max = AK7316_VDD_MAX;
// 	ret = regulator_set_voltage(ak7316->vdd, min, max);
// 	if (ret) {
// 		LOG_INF("regulator_set_voltage(vin), min(%d - %d), ret(%d)(fail)\n",
// 			min, max, ret);
// 	}
// 	ret = regulator_enable(ak7316->vdd);
// 	if (ret < 0) {
// 		LOG_INF("regulator_enable(VDD),ret(%d)(fail)\n", ret);
// 		goto fail2;
// 	}

// 	LOG_INF("-\n");

// 	return 0;

// fail2:
// 	regulator_disable(ak7316->vin);
// 	regulator_disable(ak7316->vdd);

// 	return ret;
// }

// static int ak7316_power_off2(struct ak7316_device *ak7316)
// {
// 	int ret, ret2;

// 	LOG_INF("+\n");

// 	ret = regulator_disable(ak7316->vin);
// 	ret2 = regulator_disable(ak7316->vdd);
// 	if (ret || ret2)
// 		return ret ? ret : ret2;

// 	LOG_INF("-\n");

// 	return ret;
// }

static int ak7316_set_setting(struct ak7316_device *ak7316)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);
	u8 addr = 0x16, value = 0x20; // W(0x16) = 0x20
	int ret = 0;

	LOG_INF("+");

	// power on
	// ret = ak7316_power_on2(ak7316);
	// if (ret) {
	// 	LOG_INF("%s:%d power on failure: %d", __func__, __LINE__, ret);
	// 	return ret;
	// }
	// Confirm hardware requirements and adjust/remove the delay.
	usleep_range(AK7316_CTRL_DELAY_US, AK7316_CTRL_DELAY_US + 100);
	// standby
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x40); // standy
	if (ret) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
		goto out;
	}
	usleep_range(2800, 3000); // wait over 2.71ms

	ret = i2c_smbus_read_byte_data(client, addr);
	if ((ret > 0) && ((ret & 0xFF) == value)) {
		LOG_INF("no need to update pid");
		pid_parm_setting_done = true;
		ret = 0;
		goto out;
	}
	// change to setting mode
	ret = i2c_smbus_write_byte_data(client, 0xAE, 0x3B);
	// pid parm setting
	ret = i2c_smbus_write_byte_data(client, addr, value);
	// strobe instruction
	ret = i2c_smbus_write_byte_data(client, 0x03, 0x02);
	if (ret < 0) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
		goto out;
	}
	msleep(180); // store delay 180ms
	// release setting mode
	ret = i2c_smbus_write_byte_data(client, 0xAE, 0x00);
	if (ret < 0) {
		LOG_INF("%s:%d I2C failure: %d", __func__, __LINE__, ret);
		goto out;
	}
	ret = i2c_smbus_read_byte_data(client, addr);
	if ((ret > 0) && ((ret & 0xFF) == value)) {
		LOG_INF("pid update success");
		pid_parm_setting_done = true;
		ret = 0;
	}

	LOG_INF("-");

out:
// 	ak7316_power_off2(ak7316);
	return ret;
}

static int ak7316_ensure_active(struct ak7316_device *ak7316)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);
	int ret = 0;

	if (ak7316->active)
		return 0;

	LOG_INF("Device not active, activating...\n");
	
	/* 00:active mode , 10:Standby mode , x1:Sleep mode */
	ret = i2c_smbus_write_byte_data(client, 0x02, 0x00);
	if (ret) {
		LOG_INF("Failed to activate device: %d\n", ret);
		return ret;
	}
	
	ak7316->active = true;
	LOG_INF("Device activated successfully\n");
	
	return 0;
}


static ssize_t ak7316_debugfs_position_read(struct file *file, char __user *buf,
                         size_t count, loff_t *ppos)
{
    struct ak7316_device *ak7316 = file->private_data;
    char tmp[32];
    int len;

    len = snprintf(tmp, sizeof(tmp), "%u\n", ak7316->debug_position);
    return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static ssize_t ak7316_debugfs_position_write(struct file *file,
                          const char __user *buf,
                          size_t count, loff_t *ppos)
{
    struct ak7316_device *ak7316 = file->private_data;
    unsigned long position;
    int ret;

    ret = kstrtoul_from_user(buf, count, 10, &position);
    if (ret)
        return ret;

    if (position > AK7316_MAX_FOCUS_POS) {
        LOG_INF("Position %lu exceeds maximum %u\n", position, AK7316_MAX_FOCUS_POS);
        return -EINVAL;
    }

    ret = ak7316_ensure_active(ak7316);
    if (ret) {
        LOG_INF("Failed to ensure device is active: %d\n", ret);
        return ret;
    }

    ret = ak7316_set_position(ak7316, (u16)position);
    if (ret) {
        LOG_INF("Failed to set position %lu: %d\n", position, ret);
        return ret;
    }

    ak7316->debug_position = (u16)position;
    g_last_pos = (u16)position;
    
    LOG_INF("Set absolute position to %u\n", (u16)position);
    return count;
}

static const struct file_operations ak7316_debugfs_position_fops = {
    .open = simple_open,
    .read = ak7316_debugfs_position_read,
    .write = ak7316_debugfs_position_write,
    .llseek = default_llseek,
};

static ssize_t ak7316_debugfs_offset_write(struct file *file,
                       const char __user *buf,
                       size_t count, loff_t *ppos)
{
    struct ak7316_device *ak7316 = file->private_data;
    long offset;
    int ret;
    u16 new_position;

    ret = kstrtol_from_user(buf, count, 10, &offset);
    if (ret)
        return ret;

    ret = ak7316_ensure_active(ak7316);
    if (ret) {
        LOG_INF("Failed to ensure device is active: %d\n", ret);
        return ret;
    }

    /* Calculate new position based on current position + offset */
    if (offset >= 0) {
        if (ak7316->debug_position + offset > AK7316_MAX_FOCUS_POS) {
            LOG_INF("Offset %ld would exceed maximum position\n", offset);
            return -EINVAL;
        }
        new_position = ak7316->debug_position + (u16)offset;
    } else {
        if (ak7316->debug_position < (u16)(-offset)) {
            LOG_INF("Offset %ld would go below minimum position\n", offset);
            return -EINVAL;
        }
        new_position = ak7316->debug_position - (u16)(-offset);
    }

    ret = ak7316_set_position(ak7316, new_position);
    if (ret) {
        LOG_INF("Failed to apply offset %ld: %d\n", offset, ret);
        return ret;
    }

    ak7316->debug_position = new_position;
    g_last_pos = new_position;
    
    LOG_INF("Applied offset %ld, new position: %u\n", offset, new_position);
    return count;
}

static const struct file_operations ak7316_debugfs_offset_fops = {
    .open = simple_open,
    .write = ak7316_debugfs_offset_write,
    .llseek = default_llseek,
};

static ssize_t ak7316_debugfs_status_read(struct file *file, char __user *buf,
                      size_t count, loff_t *ppos)
{
    struct ak7316_device *ak7316 = file->private_data;
    char tmp[256];
    int len;

    len = snprintf(tmp, sizeof(tmp), 
               "Device Active: %s\n"
               "Current Position: %u\n"
               "Last Position: %u\n"
               "Origin Position: %u\n"
               "Max Position: %u\n"
               "Auto Clip Enabled: %s\n"
               "PID Setting Done: %s\n",
               ak7316->active ? "Yes" : "No",
               ak7316->debug_position,
               g_last_pos,
               g_origin_pos,
               AK7316_MAX_FOCUS_POS,
               auto_clip_enabled ? "Yes" : "No",
               pid_parm_setting_done ? "Yes" : "No");

    return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static const struct file_operations ak7316_debugfs_status_fops = {
    .open = simple_open,
    .read = ak7316_debugfs_status_read,
    .llseek = default_llseek,
};

static int ak7316_debugfs_init(struct ak7316_device *ak7316)
{
    struct i2c_client *client = v4l2_get_subdevdata(&ak7316->sd);
    char dir_name[32];

    snprintf(dir_name, sizeof(dir_name), "ak7316_%s", dev_name(&client->dev));
    
    ak7316->debugfs_dir = debugfs_create_dir(dir_name, NULL);
    if (IS_ERR_OR_NULL(ak7316->debugfs_dir)) {
        LOG_INF("Failed to create debugfs directory\n");
        return -ENOMEM;
    }

    debugfs_create_file("position", 0644, ak7316->debugfs_dir, ak7316,
                &ak7316_debugfs_position_fops);
    
    debugfs_create_file("offset", 0200, ak7316->debugfs_dir, ak7316,
                &ak7316_debugfs_offset_fops);
    
    debugfs_create_file("status", 0444, ak7316->debugfs_dir, ak7316,
                &ak7316_debugfs_status_fops);

    ak7316->debug_position = g_origin_pos;
    
    LOG_INF("Debugfs interface created at /sys/kernel/debug/%s\n", dir_name);
    return 0;
}

static void ak7316_debugfs_cleanup(struct ak7316_device *ak7316)
{
    debugfs_remove_recursive(ak7316->debugfs_dir);
    ak7316->debugfs_dir = NULL;
}

static int ak7316_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ak7316_device *ak7316;
	int ret;

	LOG_INF("+\n");

	ak7316 = devm_kzalloc(dev, sizeof(*ak7316), GFP_KERNEL);
	if (!ak7316)
		return -ENOMEM;

	// ak7316->vin = devm_regulator_get(dev, "vin");
	// if (IS_ERR(ak7316->vin)) {
	// 	ret = PTR_ERR(ak7316->vin);
	// 	if (ret != -EPROBE_DEFER)
	// 		LOG_INF("cannot get vin regulator\n");
	// 	return ret;
	// }

	// ak7316->vdd = devm_regulator_get(dev, "vdd");
	// if (IS_ERR(ak7316->vdd)) {
	// 	ret = PTR_ERR(ak7316->vdd);
	// 	if (ret != -EPROBE_DEFER)
	// 		LOG_INF("cannot get vdd regulator\n");
	// 	return ret;
	// }

	// ak7316->vcamaf_pinctrl = devm_pinctrl_get(dev);
	// if (IS_ERR(ak7316->vcamaf_pinctrl)) {
	// 	ret = PTR_ERR(ak7316->vcamaf_pinctrl);
	// 	ak7316->vcamaf_pinctrl = NULL;
	// 	LOG_INF("cannot get pinctrl\n");
	// } else {
	// 	ak7316->vcamaf_on = pinctrl_lookup_state(
	// 		ak7316->vcamaf_pinctrl, "vcamaf_on");

	// 	if (IS_ERR(ak7316->vcamaf_on)) {
	// 		ret = PTR_ERR(ak7316->vcamaf_on);
	// 		ak7316->vcamaf_on = NULL;
	// 		LOG_INF("cannot get vcamaf_on pinctrl\n");
	// 	}

	// 	ak7316->vcamaf_off = pinctrl_lookup_state(
	// 		ak7316->vcamaf_pinctrl, "vcamaf_off");

	// 	if (IS_ERR(ak7316->vcamaf_off)) {
	// 		ret = PTR_ERR(ak7316->vcamaf_off);
	// 		ak7316->vcamaf_off = NULL;
	// 		LOG_INF("cannot get vcamaf_off pinctrl\n");
	// 	}
	// }

	v4l2_i2c_subdev_init(&ak7316->sd, client, &ak7316_ops);
	ak7316->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7316->sd.internal_ops = &ak7316_int_ops;

	ret = ak7316_init_controls(ak7316);
	if (ret)
	{
		dev_err(dev, "Failed to init controls: %d\n", ret);
		goto err_cleanup;
	}

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&ak7316->sd.entity, 0, NULL);
	if (ret < 0)
	{
		dev_err(dev, "Failed to init media entity pads: %d\n", ret);
		goto err_cleanup;
	}
	else
	{
		dev_info(dev, "Media entity pads initialized successfully\n");
	}

	ak7316->sd.entity.function = MEDIA_ENT_F_LENS;
#else
	dev_err(dev, "Media controller support is not enabled\n");
#endif

	ret = v4l2_async_register_subdev(&ak7316->sd);
	if (ret < 0)
	{
		dev_err(dev, "Failed to register subdev: %d\n", ret);
		goto err_cleanup;
	}
	else
	{
    	dev_info(dev, "Successfully registered subdev: %s\n", ak7316->sd.name);
	}

	pid_parm_setting_done = false;
	if (ak7316_set_setting(ak7316)) {
		LOG_INF("ak7316_set_setting failed");
	}

	ret = ak7316_debugfs_init(ak7316);
    if (ret) {
        LOG_INF("Failed to initialize debugfs: %d\n", ret);
        /* Continue even if debugfs fails */
    }


	LOG_INF("-\n");

	return 0;

err_cleanup:
	ak7316_subdev_cleanup(ak7316);
	return ret;
}

static void ak7316_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7316_device *ak7316 = sd_to_ak7316_vcm(sd);

	LOG_INF("+\n");
	ak7316_debugfs_cleanup(ak7316);
	ak7316_subdev_cleanup(ak7316);

	LOG_INF("-\n");

}

static const struct i2c_device_id ak7316_id_table[] = {
	{ AK7316_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ak7316_id_table);

static const struct of_device_id ak7316_of_table[] = {
	{ .compatible = "asahi-kasei,ak7316" },
	{ },
};
MODULE_DEVICE_TABLE(of, ak7316_of_table);

static struct i2c_driver ak7316_i2c_driver = {
	.driver = {
		.name = AK7316_NAME,
		.of_match_table = ak7316_of_table,
	},
	.probe_new  = ak7316_probe,
	.remove = ak7316_remove,
	.id_table = ak7316_id_table,
};

module_i2c_driver(ak7316_i2c_driver);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("AK7316 VCM driver");
MODULE_LICENSE("GPL v2");
