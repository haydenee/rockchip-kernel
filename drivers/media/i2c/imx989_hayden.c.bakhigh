// SPDX-License-Identifier: GPL-2.0
/*
 * imx989 driver
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

#define IMX989_MIPI_FREQ_1250M			450000000

#define IMX989_LANES			3

#define PIXEL_RATE_WITH_1250M_10BIT	((u64)IMX989_MIPI_FREQ_1250M * 2  * 3 / 10)
#define PIXEL_RATE_WITH_1250M_12BIT	((u64)IMX989_MIPI_FREQ_1250M * 2  * 3 / 12)

#define IMX989_XVCLK_FREQ		19200000

#define CHIP_ID				0x0989
#define IMX989_REG_CHIP_ID_H		0x0016
#define IMX989_REG_CHIP_ID_L		0x0017

#define IMX989_REG_CTRL_MODE		0x0100 //ok for 989
#define IMX989_MODE_SW_STANDBY		0x0
#define IMX989_MODE_STREAMING		0x1

#define IMX989_REG_EXPOSURE_H		0x0202 //ok for 989, coarse integration time.
#define IMX989_REG_EXPOSURE_L		0x0203
#define IMX989_EXPOSURE_MIN		2
#define IMX989_EXPOSURE_STEP		1
#define IMX989_VTS_MAX			0xffff

#define IMX989_REG_GAIN_H		0x0204 //ok for 989, analog gain value for long exposure frame
#define IMX989_REG_GAIN_L		0x0205
#define IMX989_GAIN_MIN			0x10
#define IMX989_GAIN_MAX			0x400
#define IMX989_GAIN_STEP		1
#define IMX989_GAIN_DEFAULT		0x10

//#define IMX989_REG_TEST_PATTERN_H	0x0600
#define IMX989_REG_TEST_PATTERN	0x0601
#define IMX989_TEST_PATTERN_ENABLE	0x1
#define IMX989_TEST_PATTERN_DISABLE	0x0

#define IMX989_REG_VTS_H		0x0340 //ok for 989, length of frame 
#define IMX989_REG_VTS_L		0x0341

#define IMX989_FLIP_MIRROR_REG		0x0101 //ok for 989, orientation for vertical
#define IMX989_MIRROR_BIT_MASK		BIT(0)
#define IMX989_FLIP_BIT_MASK		BIT(1)

#define IMX989_FETCH_EXP_H(VAL)		(((VAL) >> 8) & 0xFF)
#define IMX989_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define IMX989_FETCH_AGAIN_H(VAL)		(((VAL) >> 8) & 0x03)
#define IMX989_FETCH_AGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX989_FETCH_DGAIN_H(VAL)		(((VAL) >> 8) & 0x0F)
#define IMX989_FETCH_DGAIN_L(VAL)		((VAL) & 0xFF)

#define IMX989_FETCH_RHS1_H(VAL)	(((VAL) >> 16) & 0x0F)
#define IMX989_FETCH_RHS1_M(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX989_FETCH_RHS1_L(VAL)	((VAL) & 0xFF)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define IMX989_REG_VALUE_08BIT		1
#define IMX989_REG_VALUE_16BIT		2
#define IMX989_REG_VALUE_24BIT		3

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define IMX989_NAME			"imx989"

static const char * const imx989_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX989_NUM_SUPPLIES ARRAY_SIZE(imx989_supply_names)

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

struct imx989_mode {
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

struct imx989 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX989_NUM_SUPPLIES];

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
	const struct imx989_mode *cur_mode;
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

#define to_imx989(sd) container_of(sd, struct imx989, subdev)

static const struct other_data imx989_spd = { //modified to 989, for pdaf 2048x768 L and R 
	.width = 4096,
	.height = 768,
	.bus_fmt = MEDIA_BUS_FMT_SPD_2X8,
	.data_type = 0x30,
	.data_bit = 10,
};
static const struct other_data imx989_ebd = {
	.width = 4096,
	.height = 2,
	.data_type = 0x12,
	.bus_fmt = MEDIA_BUS_FMT_EBD_1X8,
};


static const struct regval imx989_init_regs[] = { //modified to 989!
	// External Clock Setting
	{0x0136, 0x13},
	{0x0137, 0x33},
	// PHY_VIF Setting
	{0x3304, 0x00},
	// Register version
	{0x33F0, 0x07},
	{0x33F1, 0x08},
	// Signaling mode setting: cphy
	{0x0111, 0x03},
	// Global Setting
	{0x316E, 0x00},
	{0x3379, 0x00},
	{0x3820, 0xBB},
	{0x3821, 0xEE},
	{0x3822, 0x01},
	{0x3823, 0x03},
	{0x3824, 0x07},
	{0x39D1, 0x00},
	{0x86A9, 0x60},
	{0x9002, 0x08},
	{0x9003, 0x08},
	{0x9004, 0x10},
	{0x90E4, 0x08},
	{0x90E5, 0x08},
	{0x90E6, 0x10},
	{0x90E7, 0x01},
	{0x9208, 0x31},
	{0x9209, 0x31},
	{0x920A, 0x42},
	{0x920B, 0x73},
	{0x923E, 0x4A},
	{0x923F, 0x84},
	{0x9240, 0x6F},
	{0x9241, 0xF2},
	{0x9242, 0x6F},
	{0x9243, 0xF3},
	{0x9244, 0x6F},
	{0x9245, 0xF7},
	{0x9246, 0x6F},
	{0x9247, 0xFE},
	{0x9248, 0x6F},
	{0x9278, 0x89},
	{0x9279, 0x98},
	{0x927A, 0x88},
	{0x927B, 0x8C},
	{0x927C, 0x88},
	{0x927D, 0x96},
	{0x927E, 0x88},
	{0x927F, 0x93},
	{0xBCAF, 0x01},
	{0xBD4E, 0xCD},
	{0xBD4F, 0xC0},
	{0xBD52, 0xD3},
	{0xBD56, 0xD1},
	{0xBD57, 0x40},
	{0xBD62, 0xCF},
	{0xBD63, 0x20},
	{0xBD66, 0xCF},
	{0xBD67, 0x20},
	{0xBD6A, 0x9D},
	{0xBD6B, 0x60},
	{0xBD6E, 0x3F},
	{0xBD6F, 0xE0},
	{0xBD76, 0xCF},
	{0xBD77, 0x20},
	{0xBD7A, 0x9D},
	{0xBD7B, 0x60},
	{0xBD7E, 0xD0},
	{0xBD7F, 0x60},
	{0xBD82, 0xD2},
	{0xBD83, 0x40},
	{0xBD86, 0xB1},
	{0xBD87, 0xC0},
	{0xBD92, 0xD0},
	{0xBD93, 0x60},
	// Global Setting 2
	{0x7533, 0x01},
	{0xBA80, 0x01},
	{0xBA9F, 0x0F},
	{0xBAA6, 0x0D},
	{0xBAAC, 0x0B},
	{0xBAC0, 0x09},
	{0xBAC6, 0x05},
	{0xBAFF, 0x32},
	{0xBB0D, 0x3C},
	{0xBB19, 0x44},
	{0xBB41, 0x4C},
	{0xBB4D, 0x5B},
	{0xBBAF, 0x73},
	{0xBBBD, 0x7D},
	{0xBBC9, 0x85},
	{0xBBF1, 0x8D},
	{0xBBFD, 0x9C},
	{0xBC65, 0x5D},
	{0xBC89, 0x6D},
	{0xD1E0, 0x0F},
	{0xD1E2, 0x0B},
	{0xD1E5, 0x05},
	{0xD205, 0x32},
	{0xD20B, 0x44},
	{0xD217, 0x5B},
	{0xD231, 0x73},
	{0xD237, 0x85},
	{0xD243, 0x9C},
	{0xD25D, 0x53},
	{0xD263, 0x65},
	{0xD26F, 0x7C},
	{0xAAE4, 0xFF},
	{0xAAE5, 0xFF},
	{0xAAEC, 0x01},
	{0xAAED, 0xE3},
	{0xAAF4, 0xFF},
	{0xAAF5, 0xFF},
	{0xAAFC, 0x01},
	{0xAAFD, 0xF1},
	{0xAB04, 0xFF},
	{0xAB05, 0xFF},
	{0xAB0C, 0x02},
	{0xAB0D, 0x02},
	{0xAB14, 0xFF},
	{0xAB15, 0xFF},
	{0xAB1C, 0x02},
	{0xAB1D, 0x02},
	{0xAB24, 0xFF},
	{0xAB25, 0xFF},
	{0xAB34, 0xFF},
	{0xAB35, 0xFF},
	{0xAB44, 0xFF},
	{0xAB45, 0xFF},
	{0xAB4C, 0x01},
	{0xAB4D, 0xF8},
	{0xAB54, 0xFF},
	{0xAB55, 0xFF},
	{0xAB5C, 0x01},
	{0xAB5D, 0xE9},
	{0xAB64, 0xFF},
	{0xAB65, 0xFF},
	{0xAB6C, 0x02},
	{0xAB6D, 0x0C},
	{0xAB74, 0xFF},
	{0xAB75, 0xFF},
	{0xAB7C, 0x02},
	{0xAB7D, 0x02},
	{0xAB84, 0xFF},
	{0xAB85, 0xFF},
	{0xAB8C, 0x02},
	{0xAB8D, 0x02},
	{0xAB94, 0xFF},
	{0xAB95, 0xFF},
	{0xAB9C, 0x01},
	{0xAB9D, 0xF8},
	{0xABA4, 0xFF},
	{0xABA5, 0xFF},
	{0xABAC, 0x02},
	{0xABAD, 0x0D},
	{0x7533, 0x00},
	// Global Setting 3
	{0x7533, 0x01},
	{0xBED4, 0x03},
	{0xBED5, 0xE8},
	{0x7533, 0x00},
	// Global Setting 4
	{0x7533, 0x01},
	{0xB00A, 0x0D},
	{0xB024, 0x0D},
	{0xB2DB, 0x05},
	{0xB34B, 0x12},
	{0xB35B, 0x50},
	{0xB360, 0x5C},
	{0xB37F, 0x16},
	{0xB39B, 0x0C},
	{0xB3B7, 0x07},
	{0xB3D3, 0x11},
	{0xB3EF, 0x7F},
	{0xB40B, 0x7F},
	{0xB41B, 0x7F},
	{0xB420, 0x7F},
	{0xB7E9, 0x32},
	{0xB821, 0x4B},
	{0xB859, 0x3C},
	{0xB891, 0x3C},
	{0xB8C9, 0x50},
	{0xB96F, 0x2D},
	{0xB99B, 0x2D},
	{0xBAEE, 0x07},
	{0xBB1B, 0x24},
	{0xBB35, 0x24},
	{0xBB4F, 0x24},
	{0xBBCB, 0x65},
	{0xBBE5, 0x65},
	{0xBBFF, 0x65},
	{0xD154, 0x20},
	{0xD155, 0x2D},
	{0xD15D, 0x3A},
	{0xD15F, 0x7F},
	{0xD161, 0x7F},
	{0xD162, 0x7F},
	{0xD163, 0x7F},
	{0xD1D6, 0x2D},
	{0xD1D9, 0x2D},
	{0xD1DC, 0x2D},
	{0xD1EB, 0x07},
	{0xD20D, 0x24},
	{0xD213, 0x24},
	{0xD219, 0x24},
	{0xD239, 0x65},
	{0xD23F, 0x65},
	{0xD245, 0x65},
	{0xD265, 0x45},
	{0xD26B, 0x45},
	{0xD271, 0x45},
	{0x7533, 0x00},
	// Global Setting 5
	{0x7533, 0x01},
	{0x97C8, 0xFF},
	{0x97C9, 0xFF},
	{0xB305, 0x05},
	{0xB321, 0x0D},
	{0xB33D, 0x10},
	{0xB354, 0x50},
	{0xB38D, 0x7F},
	{0xB3A9, 0x75},
	{0xB3C5, 0x7F},
	{0xB3E1, 0x7F},
	{0xB3FD, 0x7F},
	{0xB414, 0x7F},
	{0xC92C, 0x67},
	{0xC92D, 0x67},
	{0xC92E, 0x6A},
	{0xC934, 0x01},
	{0xC935, 0xFF},
	{0xC936, 0x6B},
	{0xC937, 0x9E},
	{0xC938, 0xA0},
	{0xC939, 0xF0},
	{0xC93C, 0x00},
	{0xC93D, 0x05},
	{0xC94C, 0x00},
	{0xC94E, 0x00},
	{0xC94F, 0xFF},
	{0xC950, 0x00},
	{0xC951, 0x95},
	{0xACFC, 0xAA},
	{0xACFE, 0xA3},
	{0xB2C4, 0x15},
	{0xB2CF, 0x0B},
	{0xB2E0, 0x24},
	{0xB307, 0x05},
	{0xB318, 0x19},
	{0xB323, 0x0F},
	{0xB33F, 0x20},
	{0xB345, 0x0C},
	{0xB350, 0x23},
	{0xB355, 0x54},
	{0xB358, 0x46},
	{0xB361, 0x5B},
	{0xB364, 0x50},
	{0xB384, 0x7F},
	{0xB38F, 0x7F},
	{0xB395, 0x7F},
	{0xB3A0, 0x7F},
	{0xB3AB, 0x7F},
	{0xB3B1, 0x7F},
	{0xB3C7, 0x7F},
	{0xB3CD, 0x7F},
	{0xB3D8, 0x7F},
	{0xB3E3, 0x7F},
	{0xB3E9, 0x7F},
	{0xB3FF, 0x7F},
	{0xB405, 0x7F},
	{0xB410, 0x7F},
	{0xB415, 0x7F},
	{0xB418, 0x7F},
	{0xB421, 0x7F},
	{0xB424, 0x7F},
	{0xB7ED, 0x28},
	{0xB80F, 0x23},
	{0xB85D, 0x32},
	{0xB869, 0x3C},
	{0x7533, 0x00},
	// Image Quality
	{0x9D87, 0x37},
	{0x9D89, 0x37},
	{0x9D99, 0x40},
	{0x9D9B, 0x40},
	{0xA705, 0x2D},
	{0xA70B, 0x2D},
	{0xA711, 0x2D},
	{0xA717, 0x2D},
	{0xA735, 0x51},
	{0xA73B, 0x51},
	{0xA7B7, 0x23},
	{0xA7B9, 0x23},
	{0xA7BD, 0x23},
	{0xA7BF, 0x23},
	{0xA7C3, 0x23},
	{0xA7C5, 0x23},
	{0xA7C9, 0x23},
	{0xA7CB, 0x23},
	{0xA805, 0x51},
	{0xA80B, 0x51},
	{0xF402, 0x01},
	{0xF403, 0x01},
	{0xF412, 0x00},

	{REG_NULL, 0x00},
};

static const struct regval imx989_linear_10bit_4096x3072_30fps_pd_on[] = { //modified to 989!
	// MIPI output setting
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x02},// 3-trio
	// Line Length PCK Setting
	{0x0342, 0x2B},
	{0x0343, 0xA0},
	{0x3152, 0x00},
	// Frame Length Lines Setting
	{0x0340, 0x2F},
	{0x0341, 0x70},
	// ROI Setting
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x1F},
	{0x0349, 0xFF},
	{0x034A, 0x17},
	{0x034B, 0xFF},
	// Mode Setting
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x08},
	{0x3004, 0x03},
	{0x31A4, 0x00},
	{0x31A8, 0x04},
	{0x31D0, 0x41},
	{0x31D1, 0x41},
	{0x321C, 0x00},
	// Digital Crop & Scaling
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},
	{0x040D, 0x00},
	{0x040E, 0x0C},
	{0x040F, 0x00},
	// Output Size Setting 4096x3072
	{0x034C, 0x10},
	{0x034D, 0x00},
	{0x034E, 0x0C},
	{0x034F, 0x00},
	// Clock Setting
	{0x0301, 0x08},
	{0x0303, 0x02},
	{0x0305, 0x03},
	{0x0306, 0x01},
	{0x0307, 0x3E},
	{0x030B, 0x04},
	{0x030D, 0x02},
	{0x030E, 0x01},
	{0x030F, 0x77},
	// Other Setting
	{0x312D, 0x00},
	{0x312E, 0x00},
	{0x312F, 0x00},
	{0x3205, 0x00},
	{0x3206, 0x00},
	{0x3805, 0x01},
	{0x381F, 0x00},
	{0x383D, 0x01},
	{0x383E, 0x01},
	{0x383F, 0x01},
	{0x3890, 0x00},
	{0x3891, 0xF8},
	{0x3894, 0x00},
	{0x3895, 0xF4},
	{0x3896, 0x00},
	{0x3897, 0xA0},
	{0x389A, 0x00},
	{0x389B, 0xA8},
	{0x38A0, 0x00},
	{0x38A1, 0x00},
	{0x38A2, 0x00},
	{0x38A3, 0x00},
	{0x38B8, 0x00},
	{0x38B9, 0xF0},
	{0x38BC, 0x27},
	{0x38BD, 0x27},
	{0x38BE, 0x27},
	{0x38BF, 0x27},
	{0x38C0, 0x00},
	{0x38C1, 0x00},
	{0x38D0, 0x00},
	{0x38D1, 0x00},
	{0x38D6, 0x00},
	{0x38D7, 0x00},
	{0x38DA, 0x00},
	{0x38DB, 0x00},
	{0x3A34, 0x00},
	{0x3A35, 0xE6},
	{0x3A36, 0x00},
	{0x3A37, 0xE6},
	{0x3A48, 0x00},
	{0x3A49, 0xE6},
	{0x3A4A, 0x00},
	{0x3A4B, 0xE6},
	{0x82B6, 0x00},
	{0x82B7, 0x1A},
	{0x82BA, 0x00},
	{0x82BB, 0x1A},
	{0x7533, 0x01},
	{0xABB4, 0xFF},
	{0xABB5, 0xFF},
	{0xABBC, 0x02},
	{0xABBD, 0x02},
	{0xABC4, 0xFF},
	{0xABC5, 0xFF},
	{0xABCC, 0x02},
	{0xABCD, 0x02},
	{0xBA7E, 0x03},
	{0xBA81, 0x05},
	{0xB001, 0x04},
	{0xD101, 0x04},
	{0x7533, 0x00},
	// Integration Setting
	{0x0202, 0x2F},
	{0x0203, 0x40},
	{0x0224, 0x01},
	{0x0225, 0xF4},
	{0x3162, 0x01},
	{0x3163, 0xF4},
	{0x3168, 0x01},
	{0x3169, 0xF4},
	// Gain Setting
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0218, 0x01},
	{0x0219, 0x00},
	{0x3164, 0x00},
	{0x3165, 0x00},
	{0x3166, 0x01},
	{0x3167, 0x00},
	{0x316A, 0x00},
	{0x316B, 0x00},
	{0x316C, 0x01},
	{0x316D, 0x00},
	// HDR mode Setting
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0222, 0x01},
	{0x3161, 0x00},
	{0x320B, 0x01},
	// DCGHDR Setting
	{0x3170, 0x00},
	{0x3171, 0x00},
	{0x3172, 0x04},
	{0x7533, 0x01},
	{0xB804, 0x00},
	{0xB805, 0x8C},
	{0xB83C, 0x00},
	{0xB83D, 0x96},
	{0xB874, 0x00},
	{0xB875, 0xA0},
	{0x7533, 0x00},
	// PHASE PIX Setting
	{0x3104, 0x01},
	{0x3131, 0x00},
	{0x3132, 0x1C},
	{0x3133, 0x80},
	{0x31BF, 0x01},
	{0xB598, 0x00},
	{0xB599, 0x00},
	// DOL Setting
	{0x3180, 0x00},
	{0x3181, 0x00},
	{0x3188, 0x0A},
	{0x3189, 0x0A},
	{0x318A, 0x0A},
	{0x318B, 0x0A},
	{0x318C, 0x0A},
	{0x318D, 0x0A},
	{0x39D0, 0x00},
	// EAE-Bracketing Setting
	{0x0E00, 0x00},
	{0x0E01, 0x00},
	{0x0E02, 0x00},
	{0x0E03, 0x00},
	{0x0E04, 0xFF},
	{0x0E05, 0x0F},
	{0x0E07, 0x01},
	{0x0E10, 0x00},
	{0x0E11, 0x00},
	{0x0E12, 0x00},
	{0x0E13, 0x00},
	{0x0E14, 0x00},
	{0x0E15, 0x00},
	{0x0E17, 0x00},
	{0x0E18, 0x00},
	{0x0E19, 0x00},
	{0x0E1A, 0x00},
	{0x0E1B, 0x00},
	{0x0E1C, 0x00},
	{0x0E1D, 0x00},
	{0x0E1E, 0x00},
	{0x0E1F, 0x00},
	{0x0E22, 0x00},
	{0x0E23, 0x00},
	{0x0E26, 0x00},
	{0x0E27, 0x00},
	{0x0E28, 0x00},
	{0x0E29, 0x00},
	{0x0E2A, 0x00},
	{0x0E2B, 0x00},
	{0x0E2C, 0x00},
	{0x0E2D, 0x00},
	{0x0E2E, 0x00},
	{0x0E2F, 0x00},
	{0x0E30, 0x00},
	{0x0E31, 0x00},
	{0x0E32, 0x00},
	{0x0E33, 0x00},
	{0x0E40, 0x00},
	{0x0E41, 0x00},
	{0x0E42, 0x00},
	{0x0E43, 0x00},
	{0x0E44, 0x00},
	{0x0E45, 0x00},
	{0x0E47, 0x00},
	{0x0E48, 0x00},
	{0x0E49, 0x00},
	{0x0E4A, 0x00},
	{0x0E4B, 0x00},
	{0x0E4C, 0x00},
	{0x0E4D, 0x00},
	{0x0E4E, 0x00},
	{0x0E4F, 0x00},
	{0x0E52, 0x00},
	{0x0E53, 0x00},
	{0x0E56, 0x00},
	{0x0E57, 0x00},
	{0x0E58, 0x00},
	{0x0E59, 0x00},
	{0x0E5A, 0x00},
	{0x0E5B, 0x00},
	{0x0E5C, 0x00},
	{0x0E5D, 0x00},
	{0x0E5E, 0x00},
	{0x0E5F, 0x00},
	{0x0E60, 0x00},
	{0x0E61, 0x00},
	{0x0E62, 0x00},
	{0x0E63, 0x00},
	{0x0E70, 0x00},
	{0x0E71, 0x00},
	{0x0E72, 0x00},
	{0x0E73, 0x00},
	{0x0E74, 0x00},
	{0x0E75, 0x00},
	{0x0E77, 0x00},
	{0x0E78, 0x00},
	{0x0E79, 0x00},
	{0x0E7A, 0x00},
	{0x0E7B, 0x00},
	{0x0E7C, 0x00},
	{0x0E7D, 0x00},
	{0x0E7E, 0x00},
	{0x0E7F, 0x00},
	{0x0E82, 0x00},
	{0x0E83, 0x00},
	{0x0E86, 0x00},
	{0x0E87, 0x00},
	{0x0E88, 0x00},
	{0x0E89, 0x00},
	{0x0E8A, 0x00},
	{0x0E8B, 0x00},
	{0x0E8C, 0x00},
	{0x0E8D, 0x00},
	{0x0E8E, 0x00},
	{0x0E8F, 0x00},
	{0x0E90, 0x00},
	{0x0E91, 0x00},
	{0x0E92, 0x00},
	{0x0E93, 0x00},
	{0x3240, 0x00},
	{0x3241, 0x00},
	{0x3248, 0x00},
	// Data Identifier
	{0x3087, 0x30},
	// Global Timing MIPI (3567 Msps/trio)
	{0x0808, 0x00},
	// {0x084E, 0x00},
	// {0x084F, 0x1F},
	// {0x0850, 0x00},
	// {0x0851, 0x19},
	// {0x0852, 0x00},
	// {0x0853, 0x33},
	// {0x0854, 0x00},
	// {0x0855, 0x29},
	// {0x0858, 0x00},
	// {0x0859, 0x1F},

	{REG_NULL, 0x00},
};
static const struct imx989_mode supported_modes[] = {
	{
		.width = 4096,
		.height = 3072,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x2000,
		.hts_def = 0x2ba0,//11168
		.vts_def = 0x2f70,//12144
		.bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
		.global_reg_list = imx989_init_regs,
		.reg_list = imx989_linear_10bit_4096x3072_30fps_pd_on,
		.spd = &imx989_spd,
		.ebd = &imx989_ebd,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 0,
		.vc[PAD0] = 0,
	},
};

static const s64 link_freq_items[] = {
	IMX989_MIPI_FREQ_1250M,
};
static const char * const imx989_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"100% color bars",
	"Fade to grey color bars",
	"PN9"
};

/* Read registers up to 4 at a time */
static int imx989_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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
static int imx989_write_reg(struct i2c_client *client, u16 reg,
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

	// readback
	if (len == IMX989_REG_VALUE_08BIT) {
		u32 read_val;
		int ret = imx989_read_reg(client, reg, len, &read_val);
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

static int imx989_write_array(struct i2c_client *client,
			      const struct regval *regs)//ok for all
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = imx989_write_reg(client, regs[i].addr,
					       IMX989_REG_VALUE_08BIT,
					       regs[i].val);

	dev_err(&client->dev, "%s: i=%d, ret=%d\n", __func__, i, ret);
	return ret;
}



static int imx989_get_reso_dist(const struct imx989_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)//ok for all
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct imx989_mode *
imx989_find_best_fit(struct imx989 *imx989, struct v4l2_subdev_format *fmt)//ok for all
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx989->cfg_num; i++) {
		dist = imx989_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx989_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	const struct imx989_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&imx989->mutex);

	mode = imx989_find_best_fit(imx989, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx989->mutex);
		return -ENOTTY;
#endif
	} else {
		imx989->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx989->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx989->vblank, vblank_def,
					 IMX989_VTS_MAX - mode->height,
					 1, vblank_def);

		__v4l2_ctrl_s_ctrl(imx989->vblank, vblank_def);
		__v4l2_ctrl_s_ctrl(imx989->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] * 2 * IMX989_LANES / 10 ;
		__v4l2_ctrl_s_ctrl_int64(imx989->pixel_rate,
					 pixel_rate);
	}

	dev_info(&imx989->client->dev, "%s: mode->mipi_freq_idx(%d)",
		 __func__, mode->mipi_freq_idx);

	mutex_unlock(&imx989->mutex);

	return 0;
}

static int imx989_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	const struct imx989_mode *mode = imx989->cur_mode;

	mutex_lock(&imx989->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
#else
		mutex_unlock(&imx989->mutex);
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
		if (fmt->pad == imx989->spd_id && mode->spd) {
			fmt->format.width = mode->spd->width;
			fmt->format.height = mode->spd->height;
			fmt->format.code = mode->spd->bus_fmt;
			//Set the vc channel to be consistent with the valid data
			fmt->reserved[0] = 0;
		} else if (fmt->pad == imx989->ebd_id && mode->ebd) {
			fmt->format.width = mode->ebd->width;
			fmt->format.height = mode->ebd->height;
			fmt->format.code = mode->ebd->bus_fmt;
			//Set the vc channel to be consistent with the valid data
			fmt->reserved[0] = 0;
		}
	}
	mutex_unlock(&imx989->mutex);

	return 0;
}

static int imx989_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = imx989->cur_mode->bus_fmt;

	return 0;
}

static int imx989_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);

	if (fse->index >= imx989->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx989_enable_test_pattern(struct imx989 *imx989, u32 pattern)//ok for 989
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | IMX989_TEST_PATTERN_ENABLE;
	else
		val = IMX989_TEST_PATTERN_DISABLE;

	return imx989_write_reg(imx989->client,
				IMX989_REG_TEST_PATTERN,
				IMX989_REG_VALUE_08BIT,
				val);
}

static int imx989_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	const struct imx989_mode *mode = imx989->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

static int imx989_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	
	// config->type = imx989->bus_cfg.bus_type;
	config->type = V4L2_MBUS_CSI2_CPHY;
	config->bus.mipi_csi2 = imx989->bus_cfg.bus.mipi_csi2;
	dev_info(&imx989->client->dev,
	       "[HAYDEN] %s: pad_id=%d, type=%d, num_data_lanes=%d\n",
	       __func__, pad_id, config->type,
	       config->bus.mipi_csi2.num_data_lanes);
	return 0;
}

static void imx989_get_otp(struct otp_info *otp,
			       struct rkmodule_inf *inf)// TODO: modify for imx989!!!
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

static void imx989_get_module_inf(struct imx989 *imx989,
				  struct rkmodule_inf *inf)
{
	struct otp_info *otp = imx989->otp;

	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, IMX989_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, imx989->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, imx989->len_name, sizeof(inf->base.lens));
	if (otp)
		imx989_get_otp(otp, inf);

}

static int imx989_get_channel_info(struct imx989 *imx989, struct rkmodule_channel_info *ch_info)
{
	const struct imx989_mode *mode = imx989->cur_mode;

	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

	if (ch_info->index == imx989->spd_id && mode->spd) {
		ch_info->vc = 1;
		ch_info->width = mode->spd->width;
		ch_info->height = mode->spd->height;
		ch_info->bus_fmt = mode->spd->bus_fmt;
		ch_info->data_type = mode->spd->data_type;
		ch_info->data_bit = mode->spd->data_bit;
	} else {
		ch_info->vc = imx989->cur_mode->vc[ch_info->index];
		ch_info->width = imx989->cur_mode->width;
		ch_info->height = imx989->cur_mode->height;
		ch_info->bus_fmt = imx989->cur_mode->bus_fmt;
	}
	return 0;
}

static long imx989_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)//FIXME:
{
	struct imx989 *imx989 = to_imx989(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	u32 i, h, w;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_GET_MODULE_INFO:
		imx989_get_module_inf(imx989, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = imx989->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx989->cur_mode->width;
		h = imx989->cur_mode->height;
		for (i = 0; i < imx989->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				imx989->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == imx989->cfg_num) {
			dev_err(&imx989->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = imx989->cur_mode->hts_def -
			    imx989->cur_mode->width;
			h = imx989->cur_mode->vts_def -
			    imx989->cur_mode->height;
			__v4l2_ctrl_modify_range(imx989->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(imx989->vblank, h,
						 IMX989_VTS_MAX -
						 imx989->cur_mode->height,
						 1, h);

			if (imx989->cur_mode->bus_fmt ==
			    MEDIA_BUS_FMT_SRGGB10_1X10) {
				imx989->cur_link_freq = 0;
				imx989->cur_pixel_rate =
				PIXEL_RATE_WITH_1250M_10BIT;
			} else if (imx989->cur_mode->bus_fmt ==
				   MEDIA_BUS_FMT_SRGGB12_1X12) {
				imx989->cur_link_freq = 0;
				imx989->cur_pixel_rate =
				PIXEL_RATE_WITH_1250M_12BIT;
			}

			__v4l2_ctrl_s_ctrl_int64(imx989->pixel_rate,
						 imx989->cur_pixel_rate);
			__v4l2_ctrl_s_ctrl(imx989->link_freq,
					   imx989->cur_link_freq);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx989_write_reg(imx989->client, IMX989_REG_CTRL_MODE,
				IMX989_REG_VALUE_08BIT, IMX989_MODE_STREAMING);
		else
			ret = imx989_write_reg(imx989->client, IMX989_REG_CTRL_MODE,
				IMX989_REG_VALUE_08BIT, IMX989_MODE_SW_STANDBY);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = imx989_get_channel_info(imx989, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx989_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = imx989_ioctl(sd, cmd, inf);
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
			ret = imx989_ioctl(sd, cmd, cfg);
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

		ret = imx989_ioctl(sd, cmd, hdr);
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
			ret = imx989_ioctl(sd, cmd, hdr);
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
			ret = imx989_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = imx989_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}
		ret = imx989_ioctl(sd, cmd, ch_info);
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

static int imx989_set_flip(struct imx989 *imx989)//FIXME: Don't know 989's register
{
	int ret = 0;
	u32 val = 0;

	ret = imx989_read_reg(imx989->client, IMX989_FLIP_MIRROR_REG,
			      IMX989_REG_VALUE_08BIT, &val);
	if (imx989->flip & IMX989_MIRROR_BIT_MASK)
		val |= IMX989_MIRROR_BIT_MASK;
	else
		val &= ~IMX989_MIRROR_BIT_MASK;
	if (imx989->flip & IMX989_FLIP_BIT_MASK)
		val |= IMX989_FLIP_BIT_MASK;
	else
		val &= ~IMX989_FLIP_BIT_MASK;
	ret |= imx989_write_reg(imx989->client, IMX989_FLIP_MIRROR_REG,
				IMX989_REG_VALUE_08BIT, val);

	return ret;
}

static int __imx989_start_stream(struct imx989 *imx989)// really apply the mode
{
	int ret;

	ret = imx989_write_array(imx989->client, imx989->cur_mode->global_reg_list);
	if (ret)
	{
		dev_err(&imx989->client->dev,
			"Failed to write global registers\n");
		return ret;
	}

	ret = imx989_write_array(imx989->client, imx989->cur_mode->reg_list);
	if (ret)
	{
		dev_err(&imx989->client->dev,
			"Failed to write mode registers\n");
		return ret;
	}
	imx989->cur_vts = imx989->cur_mode->vts_def;
	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&imx989->ctrl_handler);
	if (ret)
		return ret;
	if (imx989->has_init_exp && imx989->cur_mode->hdr_mode != NO_HDR) {
		ret = imx989_ioctl(&imx989->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&imx989->init_hdrae_exp);
		if (ret) {
			dev_err(&imx989->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	imx989_set_flip(imx989);

	dev_err(&imx989->client->dev,
		"%s: %dx%d@%d, hts: %d, vts: %d, exp: %d\n",
		__func__, imx989->cur_mode->width,
		imx989->cur_mode->height,
		DIV_ROUND_CLOSEST(imx989->cur_mode->max_fps.denominator,
				  imx989->cur_mode->max_fps.numerator),
		imx989->cur_mode->hts_def, imx989->cur_vts,
		imx989->cur_mode->exp_def);

	return imx989_write_reg(imx989->client, IMX989_REG_CTRL_MODE,
				IMX989_REG_VALUE_08BIT, IMX989_MODE_STREAMING);
}

static int __imx989_stop_stream(struct imx989 *imx989)
{
	return imx989_write_reg(imx989->client, IMX989_REG_CTRL_MODE,
				IMX989_REG_VALUE_08BIT, IMX989_MODE_SW_STANDBY);
}

static int imx989_s_stream(struct v4l2_subdev *sd, int on)//FIXME:
{
	struct imx989 *imx989 = to_imx989(sd);
	struct i2c_client *client = imx989->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				imx989->cur_mode->width,
				imx989->cur_mode->height,
		DIV_ROUND_CLOSEST(imx989->cur_mode->max_fps.denominator,
				  imx989->cur_mode->max_fps.numerator));

	mutex_lock(&imx989->mutex);
	on = !!on;
	if (on == imx989->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx989_start_stream(imx989);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx989_stop_stream(imx989);
		pm_runtime_put(&client->dev);
	}

	imx989->streaming = on;
	//log
	dev_info(&client->dev, "%s: %s\n", __func__,
			on ? "streaming on" : "streaming off");

unlock_and_return:
	mutex_unlock(&imx989->mutex);

	return ret;
}

static int imx989_s_power(struct v4l2_subdev *sd, int on)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	struct i2c_client *client = imx989->client;
	int ret = 0;

	mutex_lock(&imx989->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx989->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx989->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx989->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx989->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx989_cal_delay(u32 cycles)//ok for all
{
	return DIV_ROUND_UP(cycles, IMX989_XVCLK_FREQ / 1000 / 1000);
}

static int __imx989_power_on(struct imx989 *imx989)//ok for all
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx989->client->dev;

	if(!imx989->xvclk) {
		dev_err(dev, "xvclk is not ready, skip xvclk. you will have to provide clock yourself\n");
	}
	else
	{
		ret = clk_set_rate(imx989->xvclk, IMX989_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (19.2MHz)\n");
			return ret;
		}
		if (clk_get_rate(imx989->xvclk) != IMX989_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 19.2MHz\n");
		ret = clk_prepare_enable(imx989->xvclk);
		if (ret < 0) {
			dev_err(dev, "Failed to enable xvclk\n");
			return ret;
		}
	}

	if (!IS_ERR(imx989->reset_gpio))
		gpiod_set_value_cansleep(imx989->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX989_NUM_SUPPLIES, imx989->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx989->reset_gpio))
		gpiod_set_value_cansleep(imx989->reset_gpio, 1);

	/* need wait 8ms to set register */
	usleep_range(8000, 10000);

	if (!IS_ERR(imx989->pwdn_gpio))
		gpiod_set_value_cansleep(imx989->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx989_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
    if (!IS_ERR(imx989->xvclk))
	{
        clk_disable_unprepare(imx989->xvclk);
	}

	return ret;
}

static void __imx989_power_off(struct imx989 *imx989)//ok for all
{

	if (!IS_ERR(imx989->pwdn_gpio))
		gpiod_set_value_cansleep(imx989->pwdn_gpio, 0);
	clk_disable_unprepare(imx989->xvclk);
	if (!IS_ERR(imx989->reset_gpio))
		gpiod_set_value_cansleep(imx989->reset_gpio, 0);
	regulator_bulk_disable(IMX989_NUM_SUPPLIES, imx989->supplies);
}

static int imx989_runtime_resume(struct device *dev)//ok for all
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx989 *imx989 = to_imx989(sd);

	return __imx989_power_on(imx989);
}

static int imx989_runtime_suspend(struct device *dev)//ok for all
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx989 *imx989 = to_imx989(sd);

	__imx989_power_off(imx989);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx989_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->state, 0);
	const struct imx989_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx989->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx989->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx989_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)//ok for all
{
	struct imx989 *imx989 = to_imx989(sd);

	if (fie->index >= imx989->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops imx989_pm_ops = {
	SET_RUNTIME_PM_OPS(imx989_runtime_suspend,
			   imx989_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx989_internal_ops = {
	.open = imx989_open,
};
#endif

static const struct v4l2_subdev_core_ops imx989_core_ops = {
	.s_power = imx989_s_power,
	.ioctl = imx989_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx989_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx989_video_ops = {
	.s_stream = imx989_s_stream,
	.g_frame_interval = imx989_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx989_pad_ops = {
	.enum_mbus_code = imx989_enum_mbus_code,
	.enum_frame_size = imx989_enum_frame_sizes,
	.enum_frame_interval = imx989_enum_frame_interval,
	.get_fmt = imx989_get_fmt,
	.set_fmt = imx989_set_fmt,
	.get_mbus_config = imx989_g_mbus_config,
};

static const struct v4l2_subdev_ops imx989_subdev_ops = {
	.core	= &imx989_core_ops,
	.video	= &imx989_video_ops,
	.pad	= &imx989_pad_ops,
};

static int imx989_set_gain_reg(struct imx989 *imx989, u32 a_gain) {
    int ret = 0;
    u32 gain_reg = 0;
    gain_reg = (16384 - (16384*16 / a_gain));
    ret = imx989_write_reg(imx989->client,
        IMX989_REG_GAIN_H,
        IMX989_REG_VALUE_08BIT,
        IMX989_FETCH_AGAIN_H(gain_reg));
    ret |= imx989_write_reg(imx989->client,
        IMX989_REG_GAIN_L,
        IMX989_REG_VALUE_08BIT,
        IMX989_FETCH_AGAIN_L(gain_reg));
    return ret;
}

static int imx989_set_ctrl(struct v4l2_ctrl *ctrl)//FIXME: check 989's exposure and gain registers
{
	struct imx989 *imx989 = container_of(ctrl->handler,
					     struct imx989, ctrl_handler);
	struct i2c_client *client = imx989->client;
	s64 max;
	int ret = 0;
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx989->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx989->exposure,
					 imx989->exposure->minimum, max,
					 imx989->exposure->step,
					 imx989->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx989_write_reg(imx989->client,
				       IMX989_REG_EXPOSURE_H,
				       IMX989_REG_VALUE_08BIT,
				       IMX989_FETCH_EXP_H(ctrl->val));
		ret |= imx989_write_reg(imx989->client,
					IMX989_REG_EXPOSURE_L,
					IMX989_REG_VALUE_08BIT,
					IMX989_FETCH_EXP_L(ctrl->val));
		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx989_set_gain_reg(imx989, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx989_write_reg(imx989->client,
				       IMX989_REG_VTS_H,
				       IMX989_REG_VALUE_08BIT,
				       (ctrl->val + imx989->cur_mode->height)
				       >> 8);
		ret |= imx989_write_reg(imx989->client,
					IMX989_REG_VTS_L,
					IMX989_REG_VALUE_08BIT,
					(ctrl->val + imx989->cur_mode->height)
					& 0xff);
		imx989->cur_vts = ctrl->val + imx989->cur_mode->height;

		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			imx989->flip |= IMX989_MIRROR_BIT_MASK;
		else
			imx989->flip &= ~IMX989_MIRROR_BIT_MASK;
		dev_dbg(&client->dev, "set hflip 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			imx989->flip |= IMX989_FLIP_BIT_MASK;
		else
			imx989->flip &= ~IMX989_FLIP_BIT_MASK;
		dev_dbg(&client->dev, "set vflip 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		dev_dbg(&client->dev, "set testpattern 0x%x\n",
			ctrl->val);
		ret = imx989_enable_test_pattern(imx989, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx989_ctrl_ops = {
	.s_ctrl = imx989_set_ctrl,
};

static int imx989_initialize_controls(struct imx989 *imx989)//ok for 989
{
	const struct imx989_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx989->ctrl_handler;
	mode = imx989->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &imx989->mutex;

	imx989->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_items) - 1, 0,
				link_freq_items);

	if (imx989->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB10_1X10) {
		imx989->cur_link_freq = 0;
		imx989->cur_pixel_rate = PIXEL_RATE_WITH_1250M_10BIT;
	} else if (imx989->cur_mode->bus_fmt == MEDIA_BUS_FMT_SRGGB12_1X12) {
		imx989->cur_link_freq = 0;
		imx989->cur_pixel_rate = PIXEL_RATE_WITH_1250M_12BIT;
	}

	imx989->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE,
					       0, PIXEL_RATE_WITH_1250M_10BIT,
					       1, imx989->cur_pixel_rate);
	v4l2_ctrl_s_ctrl(imx989->link_freq,
			   imx989->cur_link_freq);

	h_blank = mode->hts_def - mode->width;
	imx989->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (imx989->hblank)
		imx989->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx989->vblank = v4l2_ctrl_new_std(handler, &imx989_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   IMX989_VTS_MAX - mode->height,
					   1, vblank_def);
	imx989->cur_vts = mode->vts_def;
	exposure_max = mode->vts_def - 4;
	imx989->exposure = v4l2_ctrl_new_std(handler, &imx989_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX989_EXPOSURE_MIN,
					     exposure_max,
					     IMX989_EXPOSURE_STEP,
					     mode->exp_def);
	imx989->anal_gain = v4l2_ctrl_new_std(handler, &imx989_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      IMX989_GAIN_MIN,
					      IMX989_GAIN_MAX,
					      IMX989_GAIN_STEP,
					      IMX989_GAIN_DEFAULT);
	imx989->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &imx989_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(imx989_test_pattern_menu) - 1,
				0, 0, imx989_test_pattern_menu);

	imx989->h_flip = v4l2_ctrl_new_std(handler, &imx989_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx989->v_flip = v4l2_ctrl_new_std(handler, &imx989_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx989->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx989->client->dev,
			"Failed to init controls(  %d  )\n", ret);
		goto err_free_handler;
	}

	imx989->subdev.ctrl_handler = handler;
	imx989->has_init_exp = false;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx989_check_sensor_id(struct imx989 *imx989,
				  struct i2c_client *client)//ok for 989
{
	struct device *dev = &imx989->client->dev;
	u16 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = imx989_read_reg(client, IMX989_REG_CHIP_ID_H,
			      IMX989_REG_VALUE_08BIT, &reg_H);
	ret |= imx989_read_reg(client, IMX989_REG_CHIP_ID_L,
			       IMX989_REG_VALUE_08BIT, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (!(reg_H == (CHIP_ID >> 8) || reg_L == (CHIP_ID & 0xff))) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected imx989 %04x sensor\n", id);
	return 0;
}

static int imx989_configure_regulators(struct imx989 *imx989)//ok for all
{
	unsigned int i;

	for (i = 0; i < IMX989_NUM_SUPPLIES; i++)
		imx989->supplies[i].supply = imx989_supply_names[i];

	return devm_regulator_bulk_get(&imx989->client->dev,
				       IMX989_NUM_SUPPLIES,
				       imx989->supplies);
}

static int imx989_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx989 *imx989;
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

	imx989 = devm_kzalloc(dev, sizeof(*imx989), GFP_KERNEL);
	if (!imx989)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx989->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx989->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx989->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx989->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	imx989->client = client;
	imx989->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < imx989->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			imx989->cur_mode = &supported_modes[i];
			break;
		}
	}

	if (i == imx989->cfg_num)
		imx989->cur_mode = &supported_modes[0];

	imx989->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx989->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		// return -EINVAL;
	}

	imx989->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx989->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx989->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx989->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = of_property_read_u32(node,
				   "rockchip,spd-id",
				   &imx989->spd_id);
	if (ret != 0) {
		imx989->spd_id = PAD_MAX;
		dev_err(dev,
			"failed get spd_id, will not to use spd\n");
	}

	ret = of_property_read_u32(node,
				   "rockchip,ebd-id",
				   &imx989->ebd_id);
	if (ret != 0) {
		imx989->ebd_id = PAD_MAX;
		dev_err(dev,
			"failed get ebd_id, will not to use ebd\n");
	}

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep),
					&imx989->bus_cfg);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		of_node_put(ep);
		return ret;
	}

	if (imx989->bus_cfg.bus_type != V4L2_MBUS_CSI2_CPHY) {
		dev_err(dev, "bus type %d is not supported, only V4L2_MBUS_CSI2_CPHY is supported\n",
			imx989->bus_cfg.bus_type);
		of_node_put(ep);
		return -EINVAL;
	}
	else {
		dev_dbg(dev, "bus type V4L2_MBUS_CSI2_CPHY is supported\n");
	}


	ret = imx989_configure_regulators(imx989);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&imx989->mutex);

	sd = &imx989->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx989_subdev_ops);

	ret = imx989_initialize_controls(imx989);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx989_power_on(imx989);
	if (ret)
		goto err_free_handler;

	ret = imx989_check_sensor_id(imx989, client);
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
				imx989->otp = otp_ptr;
			} else {
				imx989->otp = NULL;
				devm_kfree(dev, otp_ptr);
			}
		}
	}
continue_probe:

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx989_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx989->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx989->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx989->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx989->module_index, facing,
		 IMX989_NAME, dev_name(sd->dev));
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
	__imx989_power_off(imx989);
err_free_handler:
	v4l2_ctrl_handler_free(&imx989->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx989->mutex);

	return ret;
}

static void imx989_remove(struct i2c_client *client)//ok for all
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx989 *imx989 = to_imx989(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx989->ctrl_handler);
	mutex_destroy(&imx989->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx989_power_off(imx989);
	pm_runtime_set_suspended(&client->dev);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx989_of_match[] = {
	{ .compatible = "sony,imx989" },
	{},
};
MODULE_DEVICE_TABLE(of, imx989_of_match);
#endif

static const struct i2c_device_id imx989_match_id[] = {
	{ "sony,imx989", 0 },
	{ },
};

static struct i2c_driver imx989_i2c_driver = {
	.driver = {
		.name = IMX989_NAME,
		.pm = &imx989_pm_ops,
		.of_match_table = of_match_ptr(imx989_of_match),
	},
	.probe		= &imx989_probe,
	.remove		= &imx989_remove,
	.id_table	= imx989_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx989_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx989_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx989 sensor driver");
MODULE_LICENSE("GPL");
