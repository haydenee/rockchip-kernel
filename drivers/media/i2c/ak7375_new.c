// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define DRIVER_NAME "ak7375"

#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define AK7375_MAX_FOCUS_POS	4095
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AK7375_FOCUS_STEPS	1
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AK7375_CTRL_STEPS	64
#define AK7375_CTRL_DELAY_US	1000

#define AK7375_REG_POSITION	0x0
#define AK7375_REG_CONT		0x2
#define AK7375_MODE_ACTIVE	0x0
#define AK7375_MODE_STANDBY	0x40

/* ak7375 device structure */
struct ak7375_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	/* active or standby mode */
	bool active;

	/* debugfs */
	struct dentry *debugfs_dir;
	u16 debug_position;
};

static inline struct ak7375_device *to_ak7375_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7375_device, ctrls_vcm);
}

static inline struct ak7375_device *sd_to_ak7375_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7375_device, sd);
}

static int ak7375_i2c_write(struct ak7375_device *ak7375,
	u8 addr, u16 data, u8 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7375->sd);
	u8 buf[3];
	int ret;

	if (size != 1 && size != 2)
		return -EINVAL;
	buf[0] = addr;
	buf[size] = data & 0xff;
	if (size == 2)
		buf[1] = (data >> 8) & 0xff;
	ret = i2c_master_send(client, (const char *)buf, size + 1);
	if (ret < 0)
		return ret;
	if (ret != size + 1)
		return -EIO;

	return 0;
}

static int ak7375_ensure_active(struct ak7375_device *ak7375_dev)
{
	int ret;

	if (ak7375_dev->active)
		return 0;

	LOG_INF("Device not active, activating...\n");

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT, AK7375_MODE_ACTIVE, 1);
	if (ret) {
		LOG_INF("Failed to activate device: %d\n", ret);
		return ret;
	}

	ak7375_dev->active = true;
	LOG_INF("Device activated successfully\n");

	return 0;
}

static int ak7375_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ak7375_device *dev_vcm = to_ak7375_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return ak7375_i2c_write(dev_vcm, AK7375_REG_POSITION,
					ctrl->val << 4, 2);

	return -EINVAL;
}

static const struct v4l2_ctrl_ops ak7375_vcm_ctrl_ops = {
	.s_ctrl = ak7375_set_ctrl,
};

static int ak7375_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int ak7375_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops ak7375_int_ops = {
	.open = ak7375_open,
	.close = ak7375_close,
};

static const struct v4l2_subdev_ops ak7375_ops = { };

static void ak7375_subdev_cleanup(struct ak7375_device *ak7375_dev)
{
	v4l2_async_unregister_subdev(&ak7375_dev->sd);
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
}

static int ak7375_init_controls(struct ak7375_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &ak7375_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dev_vcm->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
		0, AK7375_MAX_FOCUS_POS, AK7375_FOCUS_STEPS, 0);

	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
	dev_vcm->sd.ctrl_handler = hdl;

	return hdl->error;
}

static ssize_t ak7375_debugfs_position_read(struct file *file, char __user *buf,
					     size_t count, loff_t *ppos)
{
	struct ak7375_device *ak7375_dev = file->private_data;
	char tmp[32];
	int len;

	len = snprintf(tmp, sizeof(tmp), "%u\n", ak7375_dev->debug_position);
	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static ssize_t ak7375_debugfs_position_write(struct file *file,
					      const char __user *buf,
					      size_t count, loff_t *ppos)
{
	struct ak7375_device *ak7375_dev = file->private_data;
	unsigned long position;
	int ret;

	ret = kstrtoul_from_user(buf, count, 10, &position);
	if (ret)
		return ret;

	if (position > AK7375_MAX_FOCUS_POS) {
		LOG_INF("Position %lu exceeds maximum %u\n", position, AK7375_MAX_FOCUS_POS);
		return -EINVAL;
	}

	ret = ak7375_ensure_active(ak7375_dev);
	if (ret) {
		LOG_INF("Failed to ensure device is active: %d\n", ret);
		return ret;
	}

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION, (u16)position << 4, 2);
	if (ret) {
		LOG_INF("Failed to set position %lu: %d\n", position, ret);
		return ret;
	}

	ak7375_dev->debug_position = (u16)position;
	if (ak7375_dev->focus)
		ak7375_dev->focus->val = (u16)position;

	LOG_INF("Set absolute position to %u\n", (u16)position);
	return count;
}

static const struct file_operations ak7375_debugfs_position_fops = {
	.open = simple_open,
	.read = ak7375_debugfs_position_read,
	.write = ak7375_debugfs_position_write,
	.llseek = default_llseek,
};

static ssize_t ak7375_debugfs_offset_write(struct file *file,
					   const char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct ak7375_device *ak7375_dev = file->private_data;
	long offset;
	int ret;
	u16 new_position;

	ret = kstrtol_from_user(buf, count, 10, &offset);
	if (ret)
		return ret;

	ret = ak7375_ensure_active(ak7375_dev);
	if (ret) {
		LOG_INF("Failed to ensure device is active: %d\n", ret);
		return ret;
	}

	/* Calculate new position based on current position + offset */
	if (offset >= 0) {
		if (ak7375_dev->debug_position + offset > AK7375_MAX_FOCUS_POS) {
			LOG_INF("Offset %ld would exceed maximum position\n", offset);
			return -EINVAL;
		}
		new_position = ak7375_dev->debug_position + (u16)offset;
	} else {
		if (ak7375_dev->debug_position < (u16)(-offset)) {
			LOG_INF("Offset %ld would go below minimum position\n", offset);
			return -EINVAL;
		}
		new_position = ak7375_dev->debug_position - (u16)(-offset);
	}

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION, new_position << 4, 2);
	if (ret) {
		LOG_INF("Failed to apply offset %ld: %d\n", offset, ret);
		return ret;
	}

	ak7375_dev->debug_position = new_position;
	if (ak7375_dev->focus)
		ak7375_dev->focus->val = new_position;

	LOG_INF("Applied offset %ld, new position: %u\n", offset, new_position);
	return count;
}

static const struct file_operations ak7375_debugfs_offset_fops = {
	.open = simple_open,
	.write = ak7375_debugfs_offset_write,
	.llseek = default_llseek,
};

static ssize_t ak7375_debugfs_status_read(struct file *file, char __user *buf,
					  size_t count, loff_t *ppos)
{
	struct ak7375_device *ak7375_dev = file->private_data;
	char tmp[256];
	int len;

	len = snprintf(tmp, sizeof(tmp),
		       "Device Active: %s\n"
		       "Current Position: %u\n"
		       "Focus Control Value: %u\n"
		       "Max Position: %u\n"
		       "Control Steps: %u\n"
		       "Focus Steps: %u\n",
		       ak7375_dev->active ? "Yes" : "No",
		       ak7375_dev->debug_position,
		       ak7375_dev->focus ? ak7375_dev->focus->val : 0,
		       AK7375_MAX_FOCUS_POS,
		       AK7375_CTRL_STEPS,
		       AK7375_FOCUS_STEPS);

	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static const struct file_operations ak7375_debugfs_status_fops = {
	.open = simple_open,
	.read = ak7375_debugfs_status_read,
	.llseek = default_llseek,
};

static int ak7375_debugfs_init(struct ak7375_device *ak7375_dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7375_dev->sd);
	char dir_name[32];

	snprintf(dir_name, sizeof(dir_name), "ak7375_%s", dev_name(&client->dev));

	ak7375_dev->debugfs_dir = debugfs_create_dir(dir_name, NULL);
	if (IS_ERR_OR_NULL(ak7375_dev->debugfs_dir)) {
		LOG_INF("Failed to create debugfs directory\n");
		return -ENOMEM;
	}

	debugfs_create_file("position", 0644, ak7375_dev->debugfs_dir, ak7375_dev,
			    &ak7375_debugfs_position_fops);

	debugfs_create_file("offset", 0200, ak7375_dev->debugfs_dir, ak7375_dev,
			    &ak7375_debugfs_offset_fops);

	debugfs_create_file("status", 0444, ak7375_dev->debugfs_dir, ak7375_dev,
			    &ak7375_debugfs_status_fops);

	/* Initialize debug position to current focus value or 0 */
	ak7375_dev->debug_position = ak7375_dev->focus ? ak7375_dev->focus->val : 0;

	LOG_INF("Debugfs interface created at /sys/kernel/debug/%s\n", dir_name);
	return 0;
}

static void ak7375_debugfs_cleanup(struct ak7375_device *ak7375_dev)
{
	debugfs_remove_recursive(ak7375_dev->debugfs_dir);
	ak7375_dev->debugfs_dir = NULL;
}

static int ak7375_probe(struct i2c_client *client)
{
	struct ak7375_device *ak7375_dev;
	int ret;

	ak7375_dev = devm_kzalloc(&client->dev, sizeof(*ak7375_dev),
				  GFP_KERNEL);
	if (!ak7375_dev)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ak7375_dev->sd, client, &ak7375_ops);
	ak7375_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7375_dev->sd.internal_ops = &ak7375_int_ops;
	ak7375_dev->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = ak7375_init_controls(ak7375_dev);
	if (ret)
		goto err_cleanup;

	ret = media_entity_pads_init(&ak7375_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	ret = v4l2_async_register_subdev(&ak7375_dev->sd);
	if (ret < 0)
		goto err_cleanup;

	/* Initialize debugfs */
	ret = ak7375_debugfs_init(ak7375_dev);
	if (ret) {
		LOG_INF("Failed to initialize debugfs: %d\n", ret);
		/* Continue even if debugfs fails */
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

err_cleanup:
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);

	return ret;
}

static void ak7375_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);

	ak7375_debugfs_cleanup(ak7375_dev);
	ak7375_subdev_cleanup(ak7375_dev);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

/*
 * This function sets the vcm position, so it consumes least current
 * The lens position is gradually moved in units of AK7375_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused ak7375_vcm_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (!ak7375_dev->active)
		return 0;

	for (val = ak7375_dev->focus->val & ~(AK7375_CTRL_STEPS - 1);
	     val >= 0; val -= AK7375_CTRL_STEPS) {
		ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION,
				       val << 4, 2);
		if (ret)
			dev_err_once(dev, "%s I2C failure: %d\n",
				     __func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
			       AK7375_MODE_STANDBY, 1);
	if (ret)
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);

	ak7375_dev->active = false;

	return 0;
}

/*
 * This function sets the vcm position to the value set by the user
 * through v4l2_ctrl_ops s_ctrl handler
 * The lens position is gradually moved in units of AK7375_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused ak7375_vcm_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (ak7375_dev->active)
		return 0;

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
		AK7375_MODE_ACTIVE, 1);
	if (ret) {
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);
		return ret;
	}

	for (val = ak7375_dev->focus->val % AK7375_CTRL_STEPS;
	     val <= ak7375_dev->focus->val;
	     val += AK7375_CTRL_STEPS) {
		ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_POSITION,
				       val << 4, 2);
		if (ret)
			dev_err_ratelimited(dev, "%s I2C failure: %d\n",
						__func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ak7375_dev->active = true;

	return 0;
}

static const struct of_device_id ak7375_of_table[] = {
	{ .compatible = "asahi-kasei,ak7375" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ak7375_of_table);

static const struct dev_pm_ops ak7375_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume)
	SET_RUNTIME_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume, NULL)
};

static struct i2c_driver ak7375_i2c_driver = {
	.driver = {
		.name = "ak7375",
		.pm = &ak7375_pm_ops,
		.of_match_table = ak7375_of_table,
	},
	.probe_new = ak7375_probe,
	.remove = ak7375_remove,
};
module_i2c_driver(ak7375_i2c_driver);

MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_DESCRIPTION("AK7375 VCM driver");
MODULE_LICENSE("GPL v2");
