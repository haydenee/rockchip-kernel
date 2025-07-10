// SPDX-License-Identifier: GPL-2.0
/*
 * imx858 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 * V0.0X01.0X00 init version.
 */

//#define DEBUG
#include "media/v4l2-mediabus.h"
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/rk-preisp.h>
#include "otp_eeprom.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define IMX858_MIPI_FREQ_356M			356000000
#define IMX858_MIPI_FREQ_384M			384000000
#define IMX858_MIPI_FREQ_750M			750000000
#define IMX858_MIPI_FREQ_1250M			1250000000

#define IMX858_LANES			3

#define PIXEL_RATE_WITH_1250M_10BIT	((u64)IMX858_MIPI_FREQ_356M * 2  * 4 / 10)
#define PIXEL_RATE_WITH_1250M_12BIT	((u64)IMX858_MIPI_FREQ_356M * 2  * 4 / 12)

#define IMX858_XVCLK_FREQ		24000000

#define CHIP_ID				0x0858
#define IMX858_REG_CHIP_ID_H		0x0016
#define IMX858_REG_CHIP_ID_L		0x0017

#define IMX858_REG_CTRL_MODE		0x0100 //ok for 989
#define IMX858_MODE_SW_STANDBY		0x0
#define IMX858_MODE_STREAMING		0x1

#define IMX858_REG_EXPOSURE_H		0x0202 //ok for 989, coarse integration time.
#define IMX858_REG_EXPOSURE_L		0x0203
#define IMX858_EXPOSURE_MIN		2
#define IMX858_EXPOSURE_STEP		1
#define IMX858_VTS_MAX			0xffff

#define IMX858_REG_GAIN_H		0x0204 //ok for 989, analog gain value for long exposure frame
#define IMX858_REG_GAIN_L		0x0205
#define IMX858_GAIN_MIN			0x10
#define IMX858_GAIN_MAX			0x400
#define IMX858_GAIN_STEP		1
#define IMX858_GAIN_DEFAULT		0x10

//#define IMX858_REG_TEST_PATTERN_H	0x0600
#define IMX858_REG_TEST_PATTERN	0x0601
#define IMX858_TEST_PATTERN_ENABLE	0x1
#define IMX858_TEST_PATTERN_DISABLE	0x0

#define IMX858_REG_VTS_H		0x0340 //ok for 989, length of frame 
#define IMX858_REG_VTS_L		0x0341

#define IMX858_FLIP_MIRROR_REG		0x0101 //ok for 989, orientation for vertical
#define IMX858_MIRROR_BIT_MASK		BIT(0)
#define IMX858_FLIP_BIT_MASK		BIT(1)

#define IMX858_FETCH_EXP_H(VAL)		(((VAL) >> 8) & 0xFF)
#define IMX858_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define IMX858_FETCH_AGAIN_H(VAL)		(((VAL) >> 8) & 0x03)
#define IMX858_FETCH_AGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX858_FETCH_DGAIN_H(VAL)		(((VAL) >> 8) & 0x0F)
#define IMX858_FETCH_DGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX858_FETCH_RHS1_H(VAL)	(((VAL) >> 16) & 0x0F)
#define IMX858_FETCH_RHS1_M(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX858_FETCH_RHS1_L(VAL)	((VAL) & 0xFF)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define IMX858_REG_VALUE_08BIT		1
#define IMX858_REG_VALUE_16BIT		2
#define IMX858_REG_VALUE_24BIT		3

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define IMX858_NAME			"imx858"

static const char * const imx858_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX858_NUM_SUPPLIES ARRAY_SIZE(imx858_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct other_data {
	u32 width;
	u32 height;
	u32 bus_fmt;
	u32 data_type;
	u32 data_bit;
};

struct imx858_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *global_reg_list;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mipi_freq_idx;
	const struct other_data *spd;
	const struct other_data *ebd;
	u32 vc[PAD_MAX];
};

struct imx858 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX858_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct imx858_mode *cur_mode;
	u32			cfg_num;
	u32			cur_pixel_rate;
	u32			cur_link_freq;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	bool			has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	u8			flip;
	struct otp_info		*otp;
	u32			spd_id;
	u32			ebd_id;
	struct v4l2_fwnode_endpoint bus_cfg;
};

#define to_imx858(sd) container_of(sd, struct imx858, subdev)

static const struct other_data imx858_spd = { //modified to 989, for pdaf 2048x768 L and R 
	.width = 4096,
	.height = 768,
	.bus_fmt = MEDIA_BUS_FMT_SPD_2X8,
	.data_type = 0x30,
	.data_bit = 10,
};
static const struct other_data imx858_ebd = {
	.width = 4096,
	.height = 2,
	.data_type = 0x12,
	.bus_fmt = MEDIA_BUS_FMT_EBD_1X8,
};


static const struct regval imx858_init_regs[] = { //modified to 858!
// External Clock Setting
	{0x0136, 0x18},
	{0x0137, 0x00},
// Global Setting1
	{0xF800, 0xF4},
	{0xF801, 0xF4},
	{0xF802, 0x29},
	{0xF803, 0xFA},
	{0xF804, 0x55},
	{0xF805, 0xD8},
	{0xF806, 0x35},
	{0xF807, 0x00},
	{0xF808, 0x17},
	{0xF809, 0xFA},
	{0xF80A, 0x84},
	{0xF80B, 0xBE},
	{0xF80C, 0x55},
	{0xF80D, 0xDC},
	{0xF80E, 0x35},
	{0xF80F, 0x00},
	{0xF810, 0x1A},
	{0xF811, 0x80},
	{0xF812, 0xAA},
	{0xF813, 0xCA},
	{0xF884, 0x4F},
	{0xF885, 0x06},
	{0xF886, 0xAC},
	{0xF887, 0x27},
	{0xF888, 0xAC},
	{0xF889, 0x1D},
	{0xF88A, 0xAC},
	{0xF88B, 0x14},
	{0xF88C, 0xFA},
	{0xF88D, 0xE8},
	{0xF88E, 0xF3},
	{0xF88F, 0x92},
	{0xF890, 0xD0},
	{0xF891, 0x20},
	{0xF892, 0x40},
	{0xF893, 0x4F},
	{0xF894, 0x1E},
	{0xF895, 0x05},
	{0xF896, 0xD4},
	{0xF897, 0x18},
	{0xF898, 0x08},
	{0xF899, 0x00},
	{0xF89A, 0xF6},
	{0xF89B, 0x40},
	{0xF89C, 0x06},
	{0xF89D, 0xD3},
	{0xF89E, 0xBE},
	{0xF89F, 0x06},
	{0xF8A0, 0xF0},
	{0xF8A1, 0x40},
	{0xF8A2, 0x06},
	{0xF8A3, 0xD4},
	{0xF8A4, 0x29},
	{0xF8A5, 0x25},
	{0xF8A6, 0x90},
	{0xF8A7, 0x2C},
	{0xF8A8, 0x80},
	{0xF8A9, 0x22},
	{0xF8AA, 0xF6},
	{0xF8AB, 0x41},
	{0xF8AC, 0x05},
	{0xF8AD, 0x1C},
	{0xF8AE, 0xB8},
	{0xF8AF, 0x8B},
	{0xF8B0, 0xD0},
	{0xF8B1, 0x18},
	{0xF8B2, 0x28},
	{0xF8B3, 0x7C},
	{0xF8B4, 0xF0},
	{0xF8B5, 0x02},
	{0xF8B6, 0x02},
	{0xF8B7, 0x74},
	{0xF8B8, 0xF6},
	{0xF8B9, 0x40},
	{0xF8BA, 0x07},
	{0xF8BB, 0x10},
	{0xF8BC, 0x0F},
	{0xF8BD, 0x20},
	{0xF8BE, 0xFA},
	{0xF8BF, 0xE8},
	{0xF8C0, 0x07},
	{0xF8C1, 0x06},
	{0xF8C2, 0x80},
	{0xF8C3, 0x04},
	{0xF8C4, 0x5F},
	{0xF8C5, 0xF0},
	{0xF8C6, 0x29},
	{0xF8C7, 0x25},
	{0xF8C8, 0x90},
	{0xF8C9, 0x0A},
	{0xF8CA, 0x50},
	{0xF8CB, 0x31},
	{0xF8CC, 0xFA},
	{0xF8CD, 0xE8},
	{0xF8CE, 0x06},
	{0xF8CF, 0xF8},
	{0xF8D0, 0x80},
	{0xF8D1, 0x04},
	{0xF8D2, 0x09},
	{0xF8D3, 0x50},
	{0xF8D4, 0xF4},
	{0xF8D5, 0x40},
	{0xF8D6, 0x06},
	{0xF8D7, 0xD8},
	{0xF8D8, 0x50},
	{0xF8D9, 0xA1},
	{0xF8DA, 0xFA},
	{0xF8DB, 0xE8},
	{0xF8DC, 0x06},
	{0xF8DD, 0xEA},
	{0xF8DE, 0xFA},
	{0xF8DF, 0xE8},
	{0xF8E0, 0x06},
	{0xF8E1, 0x1A},
	{0xF8E2, 0xA8},
	{0xF8E3, 0x14},
	{0xF8E4, 0xA8},
	{0xF8E5, 0x1D},
	{0xF8E6, 0xA8},
	{0xF8E7, 0x27},
	{0xF8E8, 0xA0},
	{0xF8E9, 0x09},
	{0xF8EA, 0x00},
	{0xF8EB, 0x00},
	{0x4513, 0x01},
	{0x4331, 0x01},
// PHY_VIF Setting
	{0x3304, 0x00},
// Register version
	{0x33F0, 0x02},
	{0x33F1, 0x09},
// Signaling mode setting
	{0x0111, 0x03},
// MIPI Global Timing control Setting
	{0x0808, 0x02},
// Global Setting2
	{0x7795, 0x00},
	{0x7796, 0x00},
	{0x8315, 0x0F},
	{0x1200, 0x02},
	{0x1201, 0x02},
	{0x130B, 0x00},
	{0x1340, 0x00},
	{0x3BC0, 0xBF},
	{0x3BC4, 0xBF},
	{0x3BC8, 0xBF},
	{0x3BCC, 0xBF},
	{0x558F, 0x00},
	{0x61E8, 0x50},
	{0x61E9, 0x00},
	{0x61EA, 0x50},
	{0x61EB, 0x00},
	{0x7755, 0x09},
	{0x775B, 0x01},
	{0x7D5D, 0x19},
	{0x7D5E, 0x19},
	{0x7D5F, 0x19},
	{0x7D60, 0x19},
	{0x7D61, 0x19},
	{0x7D62, 0x19},
	{0x7D64, 0x19},
	{0x7D65, 0x19},
	{0x7D66, 0x19},
	{0x7D67, 0x19},
	{0x7D68, 0x19},
	{0x7D69, 0x19},
	{0x7D6B, 0x19},
	{0x7D6C, 0x19},
	{0x7D6D, 0x19},
	{0x7D6E, 0x19},
	{0x7D6F, 0x19},
	{0x7D70, 0x19},
	{0x7D72, 0x19},
	{0x7D73, 0x19},
	{0x7D74, 0x19},
	{0x7D75, 0x19},
	{0x7D76, 0x19},
	{0x7D77, 0x19},
	{0x7D79, 0x19},
	{0x7D7A, 0x19},
	{0x7D7B, 0x19},
	{0x7D7C, 0x19},
	{0x7D7D, 0x19},
	{0x7D7F, 0x19},
	{0x7D80, 0x19},
	{0x7D81, 0x19},
	{0x7D82, 0x19},
	{0x7D83, 0x19},
	{0x90E7, 0x01},
	{0x920C, 0x90},
	{0x920E, 0x53},
	{0x920F, 0x0C},
	{0x9210, 0xA0},
	{0x9212, 0xDD},
	{0x9213, 0xDA},
	{0x9214, 0xA0},
	{0x9216, 0xEB},
	{0x9217, 0x96},
	{0x9218, 0xA0},
	{0x921A, 0xDD},
	{0x921B, 0xD7},
	{0x5E2E, 0x00},
	{0x5E2F, 0x32},
	{0x5E32, 0x08},
	{0x5E33, 0xCD},
	{0x5E64, 0x00},
	{0x5E65, 0x32},
	{0x5E68, 0x0B},
	{0x5E69, 0x97},
	{0x7220, 0xFF},
	{0x7221, 0xFF},
	{0x7222, 0xFF},
	{0x7223, 0xFF},
	{0x7A28, 0x2D},
	{0x7A29, 0x30},
	{0x7A2A, 0x30},
	{0x7A2B, 0x0E},
	{0x7A2C, 0x10},
	{0x7A2D, 0x10},
	{0x7A2E, 0x0E},
	{0x7A2F, 0x0F},
	{0x7A30, 0x0F},
	{0x7A31, 0x10},
	{0x7A32, 0x10},
	{0x7A33, 0x10},
	{0x7A34, 0x0E},
	{0x7A35, 0x12},
	{0x7A36, 0x15},
	{0x7A3A, 0x2D},
	{0x7A3B, 0x30},
	{0x7A3C, 0x31},
	{0x7A3D, 0x2B},
	{0x7A3E, 0x2D},
	{0x7A3F, 0x2E},
	{0x7A40, 0x2E},
	{0x7A41, 0x2F},
	{0x7A42, 0x2F},
	{0x7A43, 0x2E},
	{0x7A44, 0x2F},
	{0x7A45, 0x2E},
	{0x7A46, 0x2F},
	{0x7A47, 0x31},
	{0x7A48, 0x34},
	{0x7A4C, 0x2F},
	{0x7A4D, 0x31},
	{0x7A4E, 0x31},
	{0x7A4F, 0x2D},
	{0x7A50, 0x2F},
	{0x7A51, 0x31},
	{0x7A52, 0x2F},
	{0x7A53, 0x31},
	{0x7A54, 0x31},
	{0x7A55, 0x2F},
	{0x7A56, 0x30},
	{0x7A57, 0x30},
	{0x7A58, 0x30},
	{0x7A59, 0x31},
	{0x7A5A, 0x36},
	{0x7A5B, 0x31},
	{0x7A5C, 0x33},
	{0x7A5E, 0x2F},
	{0x7A5F, 0x33},
	{0x7A60, 0x32},
	{0x7A61, 0x2D},
	{0x7A62, 0x30},
	{0x7A63, 0x31},
	{0x7A64, 0x30},
	{0x7A65, 0x30},
	{0x7A66, 0x30},
	{0x7A67, 0x30},
	{0x7A68, 0x31},
	{0x7A69, 0x31},
	{0x7A6A, 0x30},
	{0x7A6B, 0x30},
	{0x7A6C, 0x37},
	{0x7A6D, 0x32},
	{0x7A6E, 0x33},
	{0x7A70, 0x2F},
	{0x7A71, 0x30},
	{0x7A72, 0x31},
	{0x7A73, 0x31},
	{0x7A74, 0x32},
	{0x7A75, 0x32},
	{0x7A76, 0x31},
	{0x7A77, 0x31},
	{0x7A78, 0x32},
	{0x7A79, 0x32},
	{0x7A7A, 0x31},
	{0x7A7B, 0x33},
	{0x7A7C, 0x33},
	{0x7A7D, 0x34},
	{0x7A7F, 0x2F},
	{0x7A80, 0x31},
	{0x7A81, 0x32},
	{0x7A82, 0x31},
	{0x7A83, 0x31},
	{0x7A84, 0x31},
	{0x7A85, 0x31},
	{0x7A86, 0x32},
	{0x7A87, 0x31},
	{0x7A88, 0x31},
	{0x7A89, 0x32},
	{0x7A8A, 0x34},
	{0x7A8B, 0x33},
	{0x7A8C, 0x35},
	{0x7A90, 0x02},
	{0x7A92, 0x01},
	{0x7A95, 0x01},
	{0x7A98, 0x03},
	{0x7AA2, 0x02},
	{0x7AA5, 0x05},
	{0x7AAA, 0x08},
	{0x7AAB, 0x02},
	{0x7AB4, 0x18},
	{0x7AB7, 0x06},
	{0x7ABC, 0x03},
	{0x7ABD, 0x02},
	{0x7ACE, 0x06},
	{0x7ACF, 0x07},
	{0x7AEC, 0x01},
	{0x7B27, 0x09},
	{0x7B28, 0x08},
	{0x7B39, 0x06},
	{0x7B3A, 0x07},
	{0x7B48, 0x07},
	{0x7B49, 0x09},
	{0x7B57, 0x05},
	{0x7B58, 0x06},
	{0x7C18, 0x2D},
	{0x7C1E, 0x2D},
	{0x7C22, 0x23},
	{0x7C23, 0x1E},
	{0x90B4, 0x0B},
	{0x90B5, 0x2C},
	{0x90B8, 0x0C},
	{0x90B9, 0x3C},
	{0x9739, 0x00},
	{0x973A, 0x13},
	{0x973B, 0x04},
	{0x973D, 0x00},
	{0x973E, 0x1C},
	{0x973F, 0xF4},
	{0x9741, 0x00},
	{0x9742, 0x32},
	{0x9743, 0x48},
	{0xA2C3, 0x18},
	{0xA2F5, 0x04},
	{0xA722, 0x00},
	{0xDDA9, 0x4E},
	{0x9674, 0x21},
	{0x9675, 0x5C},
	{0x96AF, 0x01},
	{0xAD01, 0x0A},
	{0xAD02, 0x0A},
	{0xAD0E, 0x02},

	{REG_NULL, 0x00},
};

static const struct regval imx858_linear_10bit_4096x3072_30fps_pd_on[] = { //modified to 858!
// MIPI output setting
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x02},
	{0x3239, 0x00},
// Line Length PCK Setting
	{0x0342, 0x1D},
	{0x0343, 0x4C},
	{0x3850, 0x03},
	{0x3851, 0x34},
// Frame Length Lines Setting
	{0x0340, 0x0F},
	{0x0341, 0x3E},
// ROI Setting
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0x20},
	{0x0348, 0x1F},
	{0x0349, 0xFF},
	{0x034A, 0x16},
	{0x034B, 0xDF},
// Mode Setting
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x00},
	{0x3005, 0x02},
	{0x3006, 0x02},
	{0x3140, 0x0A},
	{0x3144, 0x00},
	{0x3148, 0x04},
	{0x31C0, 0x41},
	{0x31C1, 0x41},
	{0x3205, 0x00},
// Digital Crop & Scaling
    {0x0408,0x00},
    {0x0409,0x00},
    {0x040A,0x00},
    {0x040B,0x00},
    {0x040C,0x10},
    {0x040D,0x00},
    {0x040E,0x0C},
    {0x040F,0x00},
// Output Size Setting
    {0x034C,0x10},
    {0x034D,0x00},
    {0x034E,0x0C},
    {0x034F,0x00},
// Clock Setting
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xB7},
	{0x030B, 0x02},
	{0x030D, 0x0C},
	{0x030E, 0x05},
	{0x030F, 0xCF},
// Other Setting
	{0x3104, 0x01},
	{0x324C, 0x01},
	{0x3803, 0x01},
	{0x3804, 0x01},
	{0x3805, 0x01},
	{0x3806, 0x01},
	{0x38A0, 0x00},
	{0x38A1, 0x51},
	{0x38A2, 0x00},
	{0x38A3, 0x51},
	{0x38A4, 0x00},
	{0x38A5, 0x51},
	{0x38A8, 0x01},
	{0x38A9, 0x56},
	{0x38AA, 0x01},
	{0x38AB, 0x56},
	{0x38AC, 0x01},
	{0x38AD, 0x56},
	{0x38D0, 0x02},
	{0x38D1, 0xB8},
	{0x38D2, 0x07},
	{0x38D3, 0xB4},
	{0x38E0, 0x00},
	{0x38E1, 0x00},
	{0x38E2, 0x00},
	{0x38E3, 0x00},
	{0x38E4, 0x00},
	{0x38E5, 0x00},
	{0x38E6, 0x00},
	{0x38E7, 0x00},
	{0x3B00, 0x00},
	{0x3B01, 0x00},
	{0x3B04, 0x00},
	{0x3B05, 0x00},
	{0x3B06, 0x00},
	{0x3B07, 0x00},
	{0x3B0A, 0x00},
	{0x3B0B, 0x00},
	{0x9674, 0x07},
	{0x9675, 0x4B},
// Integration Setting/
	{0x0202, 0x03},
	{0x0203, 0xE8},
// Gain Setting
	{0x0204, 0x01},
	{0x0205, 0x34},
	{0x020E, 0x01},
	{0x020F, 0x00},
// DOL Setting,
	{0x3190, 0x00},
// Integration setting2
	{0x0224, 0x01},
	{0x0225, 0xF4},
// Gain Setting,
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
// PHASE PIX VCID Setting
	{0x30A4, 0x00},
	{0x30A6, 0x00},
	{0x30C6, 0x01},
	{0x30C8, 0x01},
	{0x30F2, 0x01},
	{0x30F3, 0x01},
// PHASE PIX data type Setting
	{0x30A5, 0x30},
	{0x30A7, 0x30},
	{0x30C7, 0x30},
	{0x30C9, 0x30},
// MIPI Global Timing Setting
	{0x084E, 0x00},
	{0x084F, 0x0D},
	{0x0850, 0x00},
	{0x0851, 0x0B},
	{0x0852, 0x00},
	{0x0853, 0x17},
	{0x0854, 0x00},
	{0x0855, 0x29},
	{0x0858, 0x00},
	{0x0859, 0x1F},

	{REG_NULL, 0x00},
};
static const struct imx858_mode supported_modes[] = {
	{
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0f00,
		.hts_def = 0x1d4c,//7500
		.vts_def = 0x0f3e,//3902
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		.global_reg_list = imx858_init_regs,
		.reg_list = imx858_linear_10bit_4096x3072_30fps_pd_on,
		.spd = &imx858_spd,
		.ebd = &imx858_ebd,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 3,
		.vc[PAD0] = 0,
	},
};

static const s64 link_freq_items[] = {
	IMX858_MIPI_FREQ_356M,
	IMX858_MIPI_FREQ_384M,
	IMX858_MIPI_FREQ_750M,
	IMX858_MIPI_FREQ_1250M,
};
static const char * const imx858_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"100% color bars",
	"Fade to grey color bars",
	"PN9"
};

/* Read registers up to 4 at a time */
static int imx858_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val) //ok for all
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret, i;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			break;
	}
	if (ret != ARRAY_SIZE(msgs) && i == 3)
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

/* Write registers up to 4 at a time */
static int imx858_write_reg(struct i2c_client *client, u16 reg,
			    int len, u32 val) //ok for all
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

    // read back to verify
	if (len == IMX858_REG_VALUE_08BIT) {
		u32 read_val;
		int ret = imx858_read_reg(client, reg, len, &read_val);
		if (ret < 0)
			return ret;

		if (read_val != val) {
			dev_err(&client->dev,
				"Failed to write register 0x%04x: "
				"expected 0x%02x, read 0x%02x\n",
				reg, val & 0xff, read_val & 0xff);
		}
	}else {
        dev_warn(&client->dev,
         "Unverified write to register 0x%04x: "
         "expected 0x%08x\n", reg, val);
    }

	return 0;
}

static int imx858_write_array(struct i2c_client *client,
			      const struct regval *regs)//ok for all
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = imx858_write_reg(client, regs[i].addr,
					       IMX858_REG_VALUE_08BIT,
					       regs[i].val);

	return ret;
}



static int imx858_get_reso_dist(const struct imx858_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)//ok for all
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct imx858_mode *
imx858_find_best_fit(struct imx858 *imx858, struct v4l2_subdev_format *fmt)//ok for all
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx858->cfg_num; i++) {
		dist = imx858_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx858_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	const struct imx858_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&imx858->mutex);

	mode = imx858_find_best_fit(imx858, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx858->mutex);
		return -ENOTTY;
#endif
	} else {
		imx858->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx858->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx858->vblank, vblank_def,
					 IMX858_VTS_MAX - mode->height,
					 1, vblank_def);

		__v4l2_ctrl_s_ctrl(imx858->vblank, vblank_def);
		__v4l2_ctrl_s_ctrl(imx858->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] * 2 * IMX858_LANES / 10 ;
		__v4l2_ctrl_s_ctrl_int64(imx858->pixel_rate,
					 pixel_rate);
	}

	dev_info(&imx858->client->dev, "%s: mode->mipi_freq_idx(%d)",
		 __func__, mode->mipi_freq_idx);

	mutex_unlock(&imx858->mutex);

	return 0;
}

static int imx858_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	const struct imx858_mode *mode = imx858->cur_mode;

	mutex_lock(&imx858->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&imx858->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		// if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
		// 	fmt->reserved[0] = mode->vc[fmt->pad];
		// else
		// 	fmt->reserved[0] = mode->vc[PAD0];
		if (fmt->pad == imx858->spd_id && mode->spd) {
			fmt->format.width = mode->spd->width;
			fmt->format.height = mode->spd->height;
			fmt->format.code = mode->spd->bus_fmt;
			//Set the vc channel to be consistent with the valid data
			fmt->reserved[0] = 0;
		} else if (fmt->pad == imx858->ebd_id && mode->ebd) {
			fmt->format.width = mode->ebd->width;
			fmt->format.height = mode->ebd->height;
			fmt->format.code = mode->ebd->bus_fmt;
			//Set the vc channel to be consistent with the valid data
			fmt->reserved[0] = 0;
		}
	}
	mutex_unlock(&imx858->mutex);

	return 0;
}

static int imx858_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = imx858->cur_mode->bus_fmt;

	return 0;
}

static int imx858_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);

	if (fse->index >= imx858->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx858_enable_test_pattern(struct imx858 *imx858, u32 pattern)//ok for 989
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | IMX858_TEST_PATTERN_ENABLE;
	else
		val = IMX858_TEST_PATTERN_DISABLE;

	return imx858_write_reg(imx858->client,
				IMX858_REG_TEST_PATTERN,
				IMX858_REG_VALUE_08BIT,
				val);
}

static int imx858_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	const struct imx858_mode *mode = imx858->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int imx858_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	
	config->type = imx858->bus_cfg.bus_type;
	config->bus.mipi_csi2 = imx858->bus_cfg.bus.mipi_csi2;

	return 0;
}

static void imx858_get_otp(struct otp_info *otp,
			       struct rkmodule_inf *inf)// TODO: modify for imx858!!!
{
	u32 i, j;
	u32 w, h;

	/* awb */
	if (otp->awb_data.flag) {
		inf->awb.flag = 1;
		inf->awb.r_value = otp->awb_data.r_ratio;
		inf->awb.b_value = otp->awb_data.b_ratio;
		inf->awb.gr_value = otp->awb_data.g_ratio;
		inf->awb.gb_value = 0x0;

		inf->awb.golden_r_value = otp->awb_data.r_golden;
		inf->awb.golden_b_value = otp->awb_data.b_golden;
		inf->awb.golden_gr_value = otp->awb_data.g_golden;
		inf->awb.golden_gb_value = 0x0;
	}

	/* lsc */
	if (otp->lsc_data.flag) {
		inf->lsc.flag = 1;
		inf->lsc.width = otp->basic_data.size.width;
		inf->lsc.height = otp->basic_data.size.height;
		inf->lsc.table_size = otp->lsc_data.table_size;

		for (i = 0; i < 289; i++) {
			inf->lsc.lsc_r[i] = (otp->lsc_data.data[i * 2] << 8) |
					     otp->lsc_data.data[i * 2 + 1];
			inf->lsc.lsc_gr[i] = (otp->lsc_data.data[i * 2 + 578] << 8) |
					      otp->lsc_data.data[i * 2 + 579];
			inf->lsc.lsc_gb[i] = (otp->lsc_data.data[i * 2 + 1156] << 8) |
					      otp->lsc_data.data[i * 2 + 1157];
			inf->lsc.lsc_b[i] = (otp->lsc_data.data[i * 2 + 1734] << 8) |
					     otp->lsc_data.data[i * 2 + 1735];
		}
	}

	/* pdaf */
	if (otp->pdaf_data.flag) {
		inf->pdaf.flag = 1;
		inf->pdaf.gainmap_width = otp->pdaf_data.gainmap_width;
		inf->pdaf.gainmap_height = otp->pdaf_data.gainmap_height;
		inf->pdaf.pd_offset = otp->pdaf_data.pd_offset;
		inf->pdaf.dcc_mode = otp->pdaf_data.dcc_mode;
		inf->pdaf.dcc_dir = otp->pdaf_data.dcc_dir;
		inf->pdaf.dccmap_width = otp->pdaf_data.dccmap_width;
		inf->pdaf.dccmap_height = otp->pdaf_data.dccmap_height;
		w = otp->pdaf_data.gainmap_width;
		h = otp->pdaf_data.gainmap_height;
		for (i = 0; i < h; i++) {
			for (j = 0; j < w; j++) {
				inf->pdaf.gainmap[i * w + j] =
					(otp->pdaf_data.gainmap[(i * w + j) * 2] << 8) |
					otp->pdaf_data.gainmap[(i * w + j) * 2 + 1];
			}
		}
		w = otp->pdaf_data.dccmap_width;
		h = otp->pdaf_data.dccmap_height;
		for (i = 0; i < h; i++) {
			for (j = 0; j < w; j++) {
				inf->pdaf.dccmap[i * w + j] =
					(otp->pdaf_data.dccmap[(i * w + j) * 2] << 8) |
					otp->pdaf_data.dccmap[(i * w + j) * 2 + 1];
			}
		}
	}

	/* af */
	if (otp->af_data.flag) {
		inf->af.flag = 1;
		inf->af.dir_cnt = 1;
		inf->af.af_otp[0].vcm_start = otp->af_data.af_inf;
		inf->af.af_otp[0].vcm_end = otp->af_data.af_macro;
		inf->af.af_otp[0].vcm_dir = 0;
	}

}

static void imx858_get_module_inf(struct imx858 *imx858,
				  struct rkmodule_inf *inf)
{
	struct otp_info *otp = imx858->otp;

	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, IMX858_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, imx858->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, imx858->len_name, sizeof(inf->base.lens));
	if (otp)
		imx858_get_otp(otp, inf);

}

static int imx858_get_channel_info(struct imx858 *imx858, struct rkmodule_channel_info *ch_info)
{
	const struct imx858_mode *mode = imx858->cur_mode;

	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

	if (ch_info->index == imx858->spd_id && mode->spd) {
		ch_info->vc = 1;
		ch_info->width = mode->spd->width;
		ch_info->height = mode->spd->height;
		ch_info->bus_fmt = mode->spd->bus_fmt;
		ch_info->data_type = mode->spd->data_type;
		ch_info->data_bit = mode->spd->data_bit;
	} else {
		ch_info->vc = imx858->cur_mode->vc[ch_info->index];
		ch_info->width = imx858->cur_mode->width;
		ch_info->height = imx858->cur_mode->height;
		ch_info->bus_fmt = imx858->cur_mode->bus_fmt;
	}
	return 0;
}

static long imx858_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)//FIXME:
{
	struct imx858 *imx858 = to_imx858(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_GET_MODULE_INFO:
		imx858_get_module_inf(imx858, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = imx858->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx858->cur_mode->width;
		h = imx858->cur_mode->height;
		for (i = 0; i < imx858->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				imx858->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == imx858->cfg_num) {
			dev_err(&imx858->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = imx858->cur_mode->hts_def -
			    imx858->cur_mode->width;
			h = imx858->cur_mode->vts_def -
			    imx858->cur_mode->height;
			__v4l2_ctrl_modify_range(imx858->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(imx858->vblank, h,
						 IMX858_VTS_MAX -
						 imx858->cur_mode->height,
						 1, h);

			if (imx858->cur_mode->bus_fmt ==
			    MEDIA_BUS_FMT_SRGGB10_1X10) {
				imx858->cur_link_freq = 0;
				imx858->cur_pixel_rate =
				PIXEL_RATE_WITH_1250M_10BIT;
			} else if (imx858->cur_mode->bus_fmt ==
				   MEDIA_BUS_FMT_SRGGB12_1X12) {
				imx858->cur_link_freq = 0;
				imx858->cur_pixel_rate =
				PIXEL_RATE_WITH_1250M_12BIT;
			}

			__v4l2_ctrl_s_ctrl_int64(imx858->pixel_rate,
						 imx858->cur_pixel_rate);
			__v4l2_ctrl_s_ctrl(imx858->link_freq,
					   imx858->cur_link_freq);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx858_write_reg(imx858->client, IMX858_REG_CTRL_MODE,
				IMX858_REG_VALUE_08BIT, IMX858_MODE_STREAMING);
		else
			ret = imx858_write_reg(imx858->client, IMX858_REG_CTRL_MODE,
				IMX858_REG_VALUE_08BIT, IMX858_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = imx858_get_channel_info(imx858, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx858_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx858_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = imx858_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx858_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}
		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = imx858_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}
		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = imx858_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx858_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}
		ret = imx858_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int imx858_set_flip(struct imx858 *imx858)//FIXME: Don't know 989's register
{
	int ret = 0;
	u32 val = 0;

	ret = imx858_read_reg(imx858->client, IMX858_FLIP_MIRROR_REG,
			      IMX858_REG_VALUE_08BIT, &val);
	if (imx858->flip & IMX858_MIRROR_BIT_MASK)
		val |= IMX858_MIRROR_BIT_MASK;
	else
		val &= ~IMX858_MIRROR_BIT_MASK;
	if (imx858->flip & IMX858_FLIP_BIT_MASK)
		val |= IMX858_FLIP_BIT_MASK;
	else
		val &= ~IMX858_FLIP_BIT_MASK;
	ret |= imx858_write_reg(imx858->client, IMX858_FLIP_MIRROR_REG,
				IMX858_REG_VALUE_08BIT, val);

	return ret;
}

static int __imx858_start_stream(struct imx858 *imx858)// really apply the mode
{
	int ret;

	ret = imx858_write_array(imx858->client, imx858->cur_mode->global_reg_list);
	if (ret)
	{
		dev_err(&imx858->client->dev,
			"Failed to write global registers\n");
		return ret;
	}

	ret = imx858_write_array(imx858->client, imx858->cur_mode->reg_list);
	if (ret)
	{
		dev_err(&imx858->client->dev,
			"Failed to write mode registers\n");
		return ret;
	}
	imx858->cur_vts = imx858->cur_mode->vts_def;
	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&imx858->ctrl_handler);
	if (ret)
		return ret;
	if (imx858->has_init_exp && imx858->cur_mode->hdr_mode != NO_HDR) {
		ret = imx858_ioctl(&imx858->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&imx858->init_hdrae_exp);
		if (ret) {
			dev_err(&imx858->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	imx858_set_flip(imx858);

	return imx858_write_reg(imx858->client, IMX858_REG_CTRL_MODE,
				IMX858_REG_VALUE_08BIT, IMX858_MODE_STREAMING);
}

static int __imx858_stop_stream(struct imx858 *imx858)
{
	return imx858_write_reg(imx858->client, IMX858_REG_CTRL_MODE,
				IMX858_REG_VALUE_08BIT, IMX858_MODE_SW_STANDBY);
}

static int imx858_s_stream(struct v4l2_subdev *sd, int on)//FIXME:
{
	struct imx858 *imx858 = to_imx858(sd);
	struct i2c_client *client = imx858->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				imx858->cur_mode->width,
				imx858->cur_mode->height,
		DIV_ROUND_CLOSEST(imx858->cur_mode->max_fps.denominator,
				  imx858->cur_mode->max_fps.numerator));

	mutex_lock(&imx858->mutex);
	on = !!on;
	if (on == imx858->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx858_start_stream(imx858);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx858_stop_stream(imx858);
		pm_runtime_put(&client->dev);
	}

	imx858->streaming = on;
	//log
	dev_info(&client->dev, "%s: %s\n", __func__,
			on ? "streaming on" : "streaming off");

unlock_and_return:
	mutex_unlock(&imx858->mutex);

	return ret;
}

static int imx858_s_power(struct v4l2_subdev *sd, int on)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	struct i2c_client *client = imx858->client;
	int ret = 0;

	mutex_lock(&imx858->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx858->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx858->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx858->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx858->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx858_cal_delay(u32 cycles)//ok for all
{
	return DIV_ROUND_UP(cycles, IMX858_XVCLK_FREQ / 1000 / 1000);
}

static int __imx858_power_on(struct imx858 *imx858)//ok for all
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx858->client->dev;

	if(!imx858->xvclk) {
		dev_err(dev, "xvclk is not ready, skip xvclk. you will have to provide clock yourself\n");
	}
	else
	{
		ret = clk_set_rate(imx858->xvclk, IMX858_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
			return ret;
		}
		if (clk_get_rate(imx858->xvclk) != IMX858_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
		ret = clk_prepare_enable(imx858->xvclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable xvclk\n");
			return ret;
		}
	}

	if (!IS_ERR(imx858->reset_gpio))
		gpiod_set_value_cansleep(imx858->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX858_NUM_SUPPLIES, imx858->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx858->reset_gpio))
		gpiod_set_value_cansleep(imx858->reset_gpio, 1);

	/* need wait 8ms to set register */
	usleep_range(8000, 10000);

	if (!IS_ERR(imx858->pwdn_gpio))
		gpiod_set_value_cansleep(imx858->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx858_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
    if (!IS_ERR(imx858->xvclk))
	{
        clk_disable_unprepare(imx858->xvclk);
	}

	return ret;
}

static void __imx858_power_off(struct imx858 *imx858)//ok for all
{

	if (!IS_ERR(imx858->pwdn_gpio))
		gpiod_set_value_cansleep(imx858->pwdn_gpio, 0);
	clk_disable_unprepare(imx858->xvclk);
	if (!IS_ERR(imx858->reset_gpio))
		gpiod_set_value_cansleep(imx858->reset_gpio, 0);
	regulator_bulk_disable(IMX858_NUM_SUPPLIES, imx858->supplies);
}

static int imx858_runtime_resume(struct device *dev)//ok for all
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx858 *imx858 = to_imx858(sd);

	return __imx858_power_on(imx858);
}

static int imx858_runtime_suspend(struct device *dev)//ok for all
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx858 *imx858 = to_imx858(sd);

	__imx858_power_off(imx858);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx858_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct imx858_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx858->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx858->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx858_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)//ok for all
{
	struct imx858 *imx858 = to_imx858(sd);

	if (fie->index >= imx858->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops imx858_pm_ops = {
	SET_RUNTIME_PM_OPS(imx858_runtime_suspend,
			   imx858_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx858_internal_ops = {
	.open = imx858_open,
};
#endif

static const struct v4l2_subdev_core_ops imx858_core_ops = {
	.s_power = imx858_s_power,
	.ioctl = imx858_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx858_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx858_video_ops = {
	.s_stream = imx858_s_stream,
	.g_frame_interval = imx858_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx858_pad_ops = {
	.enum_mbus_code = imx858_enum_mbus_code,
	.enum_frame_size = imx858_enum_frame_sizes,
	.enum_frame_interval = imx858_enum_frame_interval,
	.get_fmt = imx858_get_fmt,
	.set_fmt = imx858_set_fmt,
	.get_mbus_config = imx858_g_mbus_config,
};

static const struct v4l2_subdev_ops imx858_subdev_ops = {
	.core	= &imx858_core_ops,
	.video	= &imx858_video_ops,
	.pad	= &imx858_pad_ops,
};

static int imx858_set_gain_reg(struct imx858 *imx858, u32 a_gain) {
    int ret = 0;
    u32 gain_reg = 0;
    gain_reg = (16384 - (16384*16 / a_gain));
    ret = imx858_write_reg(imx858->client,
        IMX858_REG_GAIN_H,
        IMX858_REG_VALUE_08BIT,
        IMX858_FETCH_AGAIN_H(gain_reg));
    ret |= imx858_write_reg(imx858->client,
        IMX858_REG_GAIN_L,
        IMX858_REG_VALUE_08BIT,
        IMX858_FETCH_AGAIN_L(gain_reg));
    return ret;
}

static int imx858_set_ctrl(struct v4l2_ctrl *ctrl)//FIXME: check 989's exposure and gain registers
{
	struct imx858 *imx858 = container_of(ctrl->handler,
					     struct imx858, ctrl_handler);
	struct i2c_client *client = imx858->client;
	s64 max;
	int ret = 0;
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx858->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx858->exposure,
					 imx858->exposure->minimum, max,
					 imx858->exposure->step,
					 imx858->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx858_write_reg(imx858->client,
				       IMX858_REG_EXPOSURE_H,
				       IMX858_REG_VALUE_08BIT,
				       IMX858_FETCH_EXP_H(ctrl->val));
		ret |= imx858_write_reg(imx858->client,
					IMX858_REG_EXPOSURE_L,
					IMX858_REG_VALUE_08BIT,
					IMX858_FETCH_EXP_L(ctrl->val));
		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx858_set_gain_reg(imx858, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx858_write_reg(imx858->client,
				       IMX858_REG_VTS_H,
				       IMX858_REG_VALUE_08BIT,
				       (ctrl->val + imx858->cur_mode->height)
				       >> 8);
		ret |= imx858_write_reg(imx858->client,
					IMX858_REG_VTS_L,
					IMX858_REG_VALUE_08BIT,
					(ctrl->val + imx858->cur_mode->height)
					& 0xff);
		imx858->cur_vts = ctrl->val + imx858->cur_mode->height;

		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			imx858->flip |= IMX858_MIRROR_BIT_MASK;
		else
			imx858->flip &= ~IMX858_MIRROR_BIT_MASK;
		dev_dbg(&client->dev, "set hflip 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			imx858->flip |= IMX858_FLIP_BIT_MASK;
		else
			imx858->flip &= ~IMX858_FLIP_BIT_MASK;
		dev_dbg(&client->dev, "set vflip 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "set testpattern 0x%x\n",
			ctrl->val);
		ret = imx858_enable_test_pattern(imx858, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx858_ctrl_ops = {
	.s_ctrl = imx858_set_ctrl,
};

static int imx858_initialize_controls(struct imx858 *imx858)//ok for 989
{
	const struct imx858_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx858->ctrl_handler;
	mode = imx858->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &imx858->mutex;

	imx858->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_items) - 1, 0,
				link_freq_items);

	if (imx858->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB10_1X10) {
		imx858->cur_link_freq = 0;
		imx858->cur_pixel_rate = PIXEL_RATE_WITH_1250M_10BIT;
	} else if (imx858->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB12_1X12) {
		imx858->cur_link_freq = 0;
		imx858->cur_pixel_rate = PIXEL_RATE_WITH_1250M_12BIT;
	}

	imx858->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE,
					       0, PIXEL_RATE_WITH_1250M_10BIT,
					       1, imx858->cur_pixel_rate);
	v4l2_ctrl_s_ctrl(imx858->link_freq,
			   imx858->cur_link_freq);

	h_blank = mode->hts_def - mode->width;
	imx858->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (imx858->hblank)
		imx858->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx858->vblank = v4l2_ctrl_new_std(handler, &imx858_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   IMX858_VTS_MAX - mode->height,
					   1, vblank_def);
	imx858->cur_vts = mode->vts_def;
	exposure_max = mode->vts_def - 4;
	imx858->exposure = v4l2_ctrl_new_std(handler, &imx858_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX858_EXPOSURE_MIN,
					     exposure_max,
					     IMX858_EXPOSURE_STEP,
					     mode->exp_def);
	imx858->anal_gain = v4l2_ctrl_new_std(handler, &imx858_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      IMX858_GAIN_MIN,
					      IMX858_GAIN_MAX,
					      IMX858_GAIN_STEP,
					      IMX858_GAIN_DEFAULT);
	imx858->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &imx858_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(imx858_test_pattern_menu) - 1,
				0, 0, imx858_test_pattern_menu);

	imx858->h_flip = v4l2_ctrl_new_std(handler, &imx858_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx858->v_flip = v4l2_ctrl_new_std(handler, &imx858_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx858->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx858->client->dev,
			"Failed to init controls(  %d  )\n", ret);
		goto err_free_handler;
	}

	imx858->subdev.ctrl_handler = handler;
	imx858->has_init_exp = false;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx858_check_sensor_id(struct imx858 *imx858,
				  struct i2c_client *client)//ok for 989
{
	struct device *dev = &imx858->client->dev;
	u16 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = imx858_read_reg(client, IMX858_REG_CHIP_ID_H,
			      IMX858_REG_VALUE_08BIT, &reg_H);
	ret |= imx858_read_reg(client, IMX858_REG_CHIP_ID_L,
			       IMX858_REG_VALUE_08BIT, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (!(reg_H == (CHIP_ID >> 8) || reg_L == (CHIP_ID & 0xff))) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected imx858 %04x sensor\n", id);
	return 0;
}

static int imx858_configure_regulators(struct imx858 *imx858)//ok for all
{
	unsigned int i;

	for (i = 0; i < IMX858_NUM_SUPPLIES; i++)
		imx858->supplies[i].supply = imx858_supply_names[i];

	return devm_regulator_bulk_get(&imx858->client->dev,
				       IMX858_NUM_SUPPLIES,
				       imx858->supplies);
}

static int imx858_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx858 *imx858;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;
	struct device_node *eeprom_ctrl_node;
	struct i2c_client *eeprom_ctrl_client;
	struct v4l2_subdev *eeprom_ctrl;
	struct otp_info *otp_ptr;
	struct device_node *ep;


	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	imx858 = devm_kzalloc(dev, sizeof(*imx858), GFP_KERNEL);
	if (!imx858)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx858->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx858->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx858->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx858->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	imx858->client = client;
	imx858->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < imx858->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			imx858->cur_mode = &supported_modes[i];
			break;
		}
	}

	if (i == imx858->cfg_num)
		imx858->cur_mode = &supported_modes[0];

	imx858->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx858->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		// return -EINVAL;
	}

	imx858->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx858->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx858->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx858->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = of_property_read_u32(node,
				   "rockchip,spd-id",
				   &imx858->spd_id);
	if (ret != 0) {
		imx858->spd_id = PAD_MAX;
		dev_err(dev,
			"failed get spd_id, will not to use spd\n");
	}

	ret = of_property_read_u32(node,
				   "rockchip,ebd-id",
				   &imx858->ebd_id);
	if (ret != 0) {
		imx858->ebd_id = PAD_MAX;
		dev_err(dev,
			"failed get ebd_id, will not to use ebd\n");
	}

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep),
					&imx858->bus_cfg);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		of_node_put(ep);
		return ret;
	}

	if (imx858->bus_cfg.bus_type != V4L2_MBUS_CSI2_CPHY) {
		dev_err(dev, "bus type %d is not supported, only V4L2_MBUS_CSI2_CPHY is supported\n",
			imx858->bus_cfg.bus_type);
		of_node_put(ep);
		return -EINVAL;
	}
	else {
		dev_dbg(dev, "bus type V4L2_MBUS_CSI2_CPHY is supported\n");
	}


	ret = imx858_configure_regulators(imx858);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&imx858->mutex);

	sd = &imx858->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx858_subdev_ops);

	ret = imx858_initialize_controls(imx858);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx858_power_on(imx858);
	if (ret)
		goto err_free_handler;

	ret = imx858_check_sensor_id(imx858, client);
	if (ret)
		goto err_power_off;
	eeprom_ctrl_node = of_parse_phandle(node, "eeprom-ctrl", 0);
	if (eeprom_ctrl_node) {
		eeprom_ctrl_client =
			of_find_i2c_device_by_node(eeprom_ctrl_node);
		of_node_put(eeprom_ctrl_node);
		if (IS_ERR_OR_NULL(eeprom_ctrl_client)) {
			dev_err(dev, "can not get node\n");
			goto continue_probe;
		}
		eeprom_ctrl = i2c_get_clientdata(eeprom_ctrl_client);
		if (IS_ERR_OR_NULL(eeprom_ctrl)) {
			dev_err(dev, "can not get eeprom i2c client\n");
		} else {
			otp_ptr = devm_kzalloc(dev, sizeof(*otp_ptr), GFP_KERNEL);
			if (!otp_ptr)
				return -ENOMEM;
			ret = v4l2_subdev_call(eeprom_ctrl,
				core, ioctl, 0, otp_ptr);
			if (!ret) {
				imx858->otp = otp_ptr;
			} else {
				imx858->otp = NULL;
				devm_kfree(dev, otp_ptr);
			}
		}
	}
continue_probe:

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx858_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx858->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx858->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx858->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx858->module_index, facing,
		 IMX858_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx858_power_off(imx858);
err_free_handler:
	v4l2_ctrl_handler_free(&imx858->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx858->mutex);

	return ret;
}

static void imx858_remove(struct i2c_client *client)//ok for all
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx858 *imx858 = to_imx858(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx858->ctrl_handler);
	mutex_destroy(&imx858->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx858_power_off(imx858);
	pm_runtime_set_suspended(&client->dev);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx858_of_match[] = {
	{ .compatible = "sony,imx858" },
	{},
};
MODULE_DEVICE_TABLE(of, imx858_of_match);
#endif

static const struct i2c_device_id imx858_match_id[] = {
	{ "sony,imx858", 0 },
	{ },
};

static struct i2c_driver imx858_i2c_driver = {
	.driver = {
		.name = IMX858_NAME,
		.pm = &imx858_pm_ops,
		.of_match_table = of_match_ptr(imx858_of_match),
	},
	.probe		= &imx858_probe,
	.remove		= &imx858_remove,
	.id_table	= imx858_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx858_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx858_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx858 sensor driver");
MODULE_LICENSE("GPL");
