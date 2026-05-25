/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_RK_LTC_H
#define _UAPI_LINUX_RK_LTC_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define RK_LTC_DEFAULT_SAMPLE_RATE	792000U

enum rk_ltc_mode {
	RK_LTC_MODE_NORMAL = 0,
	RK_LTC_MODE_IDLE_LOW = 1,
	RK_LTC_MODE_IDLE_HIGH = 2,
	RK_LTC_MODE_SQUARE_1KHZ = 3,
	RK_LTC_MODE_ALT_SAMPLES = 4,
	RK_LTC_MODE_RAW_FRAME = 5,
};

#define RK_LTC_FLAG_DROP_FRAME		(1U << 0)
#define RK_LTC_FLAG_COLOR_FRAME		(1U << 1)
#define RK_LTC_FLAG_BGF0		(1U << 2)
#define RK_LTC_FLAG_BGF1		(1U << 3)
#define RK_LTC_FLAG_BGF2		(1U << 4)

struct rk_ltc_timecode {
	__u8 hours;
	__u8 minutes;
	__u8 seconds;
	__u8 frames;
	__u8 reserved[4];
};

struct rk_ltc_config {
	__u32 fps_num;
	__u32 fps_den;
	__u32 mode;
	__u32 flags;
	__u32 sample_rate;
	__u8 user_bits[8];
	/*
	 * Raw frame diagnostic mode uses bit i from raw_frame[i / 8] bit
	 * (i % 8), for i in [0, 79].
	 */
	__u8 raw_frame[10];
	__u32 reserved[12];
};

struct rk_ltc_status {
	__u32 running;
	__u32 mode;
	__u32 fps_num;
	__u32 fps_den;
	__u32 drop_frame;
	__u32 high_frame_rate;
	__u32 sample_rate;
	__u32 actual_sample_rate;
	__u32 divider;
	__u32 dma_fill_bytes;
	struct rk_ltc_timecode timecode;
	__u64 codewords;
	__u64 samples;
	__u64 dma_periods;
	__u64 underruns;
	__u64 irq_count;
	__u64 decoder_locks;
	__u64 decoder_errors;
	__u32 reserved[16];
};

#define RK_LTC_IOC_MAGIC		'L'
#define RK_LTC_IOC_START		_IO(RK_LTC_IOC_MAGIC, 0x00)
#define RK_LTC_IOC_STOP			_IO(RK_LTC_IOC_MAGIC, 0x01)
#define RK_LTC_IOC_SET_CONFIG		_IOW(RK_LTC_IOC_MAGIC, 0x02, struct rk_ltc_config)
#define RK_LTC_IOC_GET_CONFIG		_IOR(RK_LTC_IOC_MAGIC, 0x03, struct rk_ltc_config)
#define RK_LTC_IOC_SET_TIMECODE		_IOW(RK_LTC_IOC_MAGIC, 0x04, struct rk_ltc_timecode)
#define RK_LTC_IOC_GET_TIMECODE		_IOR(RK_LTC_IOC_MAGIC, 0x05, struct rk_ltc_timecode)
#define RK_LTC_IOC_GET_STATUS		_IOR(RK_LTC_IOC_MAGIC, 0x06, struct rk_ltc_status)

#endif /* _UAPI_LINUX_RK_LTC_H */
