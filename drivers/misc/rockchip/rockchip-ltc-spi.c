// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip RK3588 SPI MOSI LTC generator.
 *
 * This driver owns one Rockchip SPI controller directly and uses its TX DMA
 * request to stream one-bit samples on MOSI. The SPI bit clock is the LTC
 * sample clock; DMA bytes are packed MSB first.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <uapi/linux/rk_ltc.h>

#define DRIVER_NAME				"rockchip-ltc-spi"

#define RK_LTC_RING_BYTES			(64 * 1024)
#define RK_LTC_PERIOD_BYTES			4096
#define RK_LTC_DEFAULT_SPI_PARENT_HZ		198000000U
#define RK_LTC_PHASE_SHIFT			16
#define RK_LTC_PHASE_ONE			(1ULL << RK_LTC_PHASE_SHIFT)
#define RK_LTC_DEFAULT_MAX_SLEW_PPM		1000
#define RK_LTC_MAX_SLEW_PPM			100000
#define RK_LTC_DEFAULT_LOCK_THRESHOLD_MS	5
#define RK_LTC_DEFAULT_PLL_KP_PPB_PER_MS	700000
#define RK_LTC_DEFAULT_PLL_KI_PPB_PER_MS_S	150000
#define RK_LTC_MAX_PLL_GAIN			1000000
#define RK_LTC_PLL_ERROR_CLAMP_NS		(10LL * NSEC_PER_SEC)
#define RK_LTC_NSEC_PER_MSEC			1000000LL
#define RK_LTC_NSEC_PER_USEC			1000LL
#define RK_LTC_OBSERVATION_MAX_NS			100000ULL
#define RK_LTC_SPI_FIFO_BYTES			64U

/* Rockchip SPI register offsets */
#define ROCKCHIP_SPI_CTRLR0			0x0000
#define ROCKCHIP_SPI_CTRLR1			0x0004
#define ROCKCHIP_SPI_SSIENR			0x0008
#define ROCKCHIP_SPI_SER			0x000c
#define ROCKCHIP_SPI_BAUDR			0x0010
#define ROCKCHIP_SPI_TXFTLR			0x0014
#define ROCKCHIP_SPI_RXFTLR			0x0018
#define ROCKCHIP_SPI_TXFLR			0x001c
#define ROCKCHIP_SPI_SR			0x0024
#define ROCKCHIP_SPI_IMR			0x002c
#define ROCKCHIP_SPI_ISR			0x0030
#define ROCKCHIP_SPI_ICR			0x0038
#define ROCKCHIP_SPI_DMACR			0x003c
#define ROCKCHIP_SPI_DMATDLR			0x0040
#define ROCKCHIP_SPI_TXDR			0x0400

#define CR0_DFS_OFFSET				0
#define CR0_DFS_8BIT				0x1
#define CR0_CSM_OFFSET				8
#define CR0_CSM_KEEP				0x0
#define CR0_SSD_OFFSET				10
#define CR0_SSD_ONE				0x1
#define CR0_EM_OFFSET				11
#define CR0_EM_BIG				0x1
#define CR0_FBM_OFFSET				12
#define CR0_FBM_MSB				0x0
#define CR0_BHT_OFFSET				13
#define CR0_BHT_8BIT				0x1
#define CR0_FRF_OFFSET				16
#define CR0_FRF_SPI				0x0
#define CR0_XFM_OFFSET				18
#define CR0_XFM_TO				0x1
#define CR0_OPM_OFFSET				20
#define CR0_OPM_MASTER				0x0

#define BAUDR_SCKDV_MIN			2U
#define BAUDR_SCKDV_MAX			65534U

#define SR_BUSY				BIT(0)
#define SR_TF_EMPTY				BIT(2)

#define INT_TF_OVERFLOW			BIT(1)
#define ICR_ALL				BIT(0)

#define TF_DMA_EN				BIT(1)

struct rk_ltc_rate {
	u32 fps_num;
	u32 fps_den;
	u32 codeword_num;
	u32 codeword_den;
	u32 addr_fps;
	bool high_frame_rate;
};

struct rk_ltc_gen {
	struct rk_ltc_timecode tc;
	u8 bits[80];
	u32 bit_index;
	u32 half_index;
	u32 half_remaining;
	u8 level;
	u64 half_rem;
	u64 diag_rem;
	u32 diag_remaining;
};

struct rk_ltc {
	struct device *dev;
	void __iomem *regs;
	phys_addr_t regs_phys;
	int irq;

	struct clk *spiclk;
	struct clk *apb_pclk;
	u32 sample_rate;
	u32 actual_sample_rate;
	u32 divider;

	struct dma_chan *tx_chan;
	dma_addr_t dma_handle;
	u8 *ring;
	u32 ring_bytes;
	u32 period_bytes;
	u32 periods;
	u32 period_pos;
	dma_cookie_t dma_cookie;

	struct miscdevice miscdev;
	struct dentry *debugfs_dir;

	struct mutex lock;
	spinlock_t state_lock;
	bool running;

	struct rk_ltc_config cfg;
	struct rk_ltc_rate rate;
	struct rk_ltc_timecode tc;
	struct rk_ltc_timecode current_tc;
	struct rk_ltc_timecode *period_tc;
	struct rk_ltc_gen gen;
	struct rk_ltc_sync_config sync_cfg;
	u32 sync_state;
	s32 sync_correction_ppb;
	s32 sync_integral_ppb;
	s32 sync_integral_rem;
	s64 sync_phase_error_ns;
	/* One reference/cursor snapshot per refill, shared by all its codewords. */
	bool sync_observation_valid;
	s64 sync_reference_ns;
	u64 sync_output_samples;
	u64 sync_observation_span_ns;
	u64 sync_observation_rejects;
	u32 sync_fifo_bytes;
	u32 sync_lock_streak;
	/* Keep a fixed midnight epoch across running fractional-rate day wraps. */
	bool sync_epoch_valid;
	s64 sync_epoch_ns;

	u64 codewords_generated;
	u64 samples_generated;
	u64 dma_periods;
	u64 underruns;
	u64 irq_count;
	u64 decoder_locks;
	u64 decoder_errors;
	u64 sync_updates;
	u64 sync_locked_updates;
	u64 sync_jump_count;
	s64 sync_last_jump_error_ns;
};

static const u8 rk_ltc_sync_bits[16] = {
	0, 0, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 0, 1,
};

static inline void rk_ltc_spi_enable(struct rk_ltc *ltc, bool enable)
{
	writel_relaxed(enable ? 1 : 0, ltc->regs + ROCKCHIP_SPI_SSIENR);
}

static bool rk_ltc_match_rate(u32 fps_num, u32 fps_den, u32 num, u32 den)
{
	return fps_num == num && fps_den == den;
}

static int rk_ltc_select_rate(const struct rk_ltc_config *cfg,
			      struct rk_ltc_rate *rate)
{
	u32 num = cfg->fps_num;
	u32 den = cfg->fps_den ? cfg->fps_den : 1;

	memset(rate, 0, sizeof(*rate));
	rate->fps_num = num;
	rate->fps_den = den;

	if (rk_ltc_match_rate(num, den, 24, 1)) {
		rate->codeword_num = 24;
		rate->codeword_den = 1;
		rate->addr_fps = 24;
	} else if (rk_ltc_match_rate(num, den, 25, 1)) {
		rate->codeword_num = 25;
		rate->codeword_den = 1;
		rate->addr_fps = 25;
	} else if (rk_ltc_match_rate(num, den, 30, 1)) {
		rate->codeword_num = 30;
		rate->codeword_den = 1;
		rate->addr_fps = 30;
	} else if (rk_ltc_match_rate(num, den, 50, 1)) {
		rate->codeword_num = 25;
		rate->codeword_den = 1;
		rate->addr_fps = 25;
		rate->high_frame_rate = true;
	} else if (rk_ltc_match_rate(num, den, 60, 1)) {
		rate->codeword_num = 30;
		rate->codeword_den = 1;
		rate->addr_fps = 30;
		rate->high_frame_rate = true;
	} else if (rk_ltc_match_rate(num, den, 30000, 1001)) {
		rate->codeword_num = 30000;
		rate->codeword_den = 1001;
		rate->addr_fps = 30;
	} else if (rk_ltc_match_rate(num, den, 60000, 1001)) {
		rate->codeword_num = 30000;
		rate->codeword_den = 1001;
		rate->addr_fps = 30;
		rate->high_frame_rate = true;
	} else if (rk_ltc_match_rate(num, den, 24000, 1001)) {
		rate->codeword_num = 24000;
		rate->codeword_den = 1001;
		rate->addr_fps = 24;
	} else {
		return -EINVAL;
	}

	if ((cfg->flags & RK_LTC_FLAG_DROP_FRAME) &&
	    !rk_ltc_match_rate(num, den, 30000, 1001) &&
	    !rk_ltc_match_rate(num, den, 60000, 1001))
		return -EINVAL;

	return 0;
}

static bool rk_ltc_time_valid(const struct rk_ltc_timecode *tc,
			      const struct rk_ltc_rate *rate, bool drop)
{
	if (tc->hours > 23 || tc->minutes > 59 || tc->seconds > 59)
		return false;
	if (tc->frames >= rate->addr_fps)
		return false;
	if (drop && tc->seconds == 0 && (tc->minutes % 10) && tc->frames < 2)
		return false;

	return true;
}

static void rk_ltc_clamp_time(struct rk_ltc_timecode *tc,
			      const struct rk_ltc_rate *rate, bool drop)
{
	tc->hours %= 24;
	tc->minutes %= 60;
	tc->seconds %= 60;
	if (tc->frames >= rate->addr_fps)
		tc->frames = 0;
	if (drop && tc->seconds == 0 && (tc->minutes % 10) && tc->frames < 2)
		tc->frames = 2;
}

static void rk_ltc_increment_time(struct rk_ltc_timecode *tc,
				  const struct rk_ltc_rate *rate, bool drop)
{
	tc->frames++;
	if (tc->frames < rate->addr_fps)
		return;

	tc->frames = 0;
	tc->seconds++;
	if (tc->seconds < 60)
		return;

	tc->seconds = 0;
	tc->minutes++;
	if (tc->minutes >= 60) {
		tc->minutes = 0;
		tc->hours = (tc->hours + 1) % 24;
	}

	if (drop && (tc->minutes % 10))
		tc->frames = 2;
}

static u64 rk_ltc_frames_per_day(const struct rk_ltc_rate *rate, bool drop)
{
	if (drop)
		return 24ULL * 60 * 60 * 30 - 2ULL * (24 * 60 - 24 * 6);

	return 24ULL * 60 * 60 * rate->addr_fps;
}

static u64 rk_ltc_tc_to_linear(const struct rk_ltc_timecode *tc,
			       const struct rk_ltc_rate *rate, bool drop)
{
	u64 minutes, frames;

	frames = ((u64)tc->hours * 3600 + (u64)tc->minutes * 60 +
		  tc->seconds) * rate->addr_fps + tc->frames;
	if (!drop)
		return frames;

	minutes = (u64)tc->hours * 60 + tc->minutes;

	return frames - 2 * (minutes - minutes / 10);
}

static void rk_ltc_linear_to_tc(u64 frame, const struct rk_ltc_rate *rate,
				bool drop, struct rk_ltc_timecode *tc)
{
	u64 label, ten_min, rem, dropped;

	frame %= rk_ltc_frames_per_day(rate, drop);
	if (drop) {
		ten_min = div64_u64_rem(frame, 17982, &rem);
		dropped = 18 * ten_min;
		if (rem >= 2)
			dropped += 2 * div64_u64(rem - 2, 1798);
		label = frame + dropped;
	} else {
		label = frame;
	}

	tc->frames = do_div(label, rate->addr_fps);
	tc->seconds = do_div(label, 60);
	tc->minutes = do_div(label, 60);
	tc->hours = label % 24;
	memset(tc->reserved, 0, sizeof(tc->reserved));
}

static s64 rk_ltc_mod_day_ns(s64 ns)
{
	const s64 day_ns = 24LL * 60 * 60 * NSEC_PER_SEC;

	ns = div64_s64(ns, day_ns) ? ns % day_ns : ns;
	if (ns < 0)
		ns += day_ns;

	return ns;
}

static int rk_ltc_read_sync_time(const struct rk_ltc_sync_config *cfg,
				 s64 *ns)
{
	struct timespec64 ts;
	int ret;

	switch (cfg->source) {
	case RK_LTC_SYNC_SOURCE_REALTIME:
		ktime_get_real_ts64(&ts);
		break;
	case RK_LTC_SYNC_SOURCE_TAI:
		ktime_get_clocktai_ts64(&ts);
		break;
	case RK_LTC_SYNC_SOURCE_PTP_TAI:
		if (cfg->ptp_index < 0) {
			ktime_get_clocktai_ts64(&ts);
			break;
		}
#if IS_REACHABLE(CONFIG_PTP_1588_CLOCK)
		ret = ptp_clock_gettime_by_index(cfg->ptp_index, &ts);
		if (ret)
			return ret;
		break;
#else
		return -EOPNOTSUPP;
#endif
	case RK_LTC_SYNC_SOURCE_MONOTONIC:
		ktime_get_ts64(&ts);
		break;
	default:
		return -EINVAL;
	}

	*ns = timespec64_to_ns(&ts);

	return 0;
}

/*
 * Extend a cyclic DMA cursor relative to the completed-period counter. A
 * delayed/missed callback must not masquerade as nearly one extra ring of
 * playback. The counter has already advanced for this callback.
 */
static int rk_ltc_output_position(struct rk_ltc *ltc, u32 residue,
				  u32 fifo_bytes, bool busy, u64 *samples)
{
	u64 completed = ltc->dma_periods * ltc->period_bytes;
	u32 cursor, base, ahead;
	u64 bytes;

	if (residue > ltc->ring_bytes || fifo_bytes > RK_LTC_SPI_FIFO_BYTES ||
	    !busy || !ltc->period_bytes || !ltc->ring_bytes)
		return -EAGAIN;

	cursor = (ltc->ring_bytes - residue) % ltc->ring_bytes;
	base = completed % ltc->ring_bytes;
	ahead = (cursor + ltc->ring_bytes - base) % ltc->ring_bytes;
	if (ahead >= ltc->period_bytes)
		return -EAGAIN;

	bytes = completed + ahead;
	if (bytes <= fifo_bytes)
		return -EAGAIN;

	/* TXFLR excludes the active shifter; estimate its remaining half byte. */
	*samples = (bytes - fifo_bytes) * 8 - 4;
	return 0;
}

static int rk_ltc_observe_playback_locked(struct rk_ltc *ltc)
{
	struct dma_tx_state before, after;
	enum dma_status status;
	u64 begin_ns, end_ns, samples;
	s64 ref_ns;
	u32 fifo, sr;
	int ret, retry;

	ltc->sync_observation_valid = false;
	if (!ltc->running || ltc->sync_cfg.source == RK_LTC_SYNC_SOURCE_NONE ||
	    ltc->cfg.mode != RK_LTC_MODE_NORMAL)
		return -EAGAIN;

	for (retry = 0; retry < 3; retry++) {
		begin_ns = ktime_get_raw_ns();
		status = dmaengine_tx_status(ltc->tx_chan, ltc->dma_cookie,
					     &before);
		if (status != DMA_IN_PROGRESS)
			break;
		ret = rk_ltc_read_sync_time(&ltc->sync_cfg, &ref_ns);
		if (ret)
			break;
		fifo = readl_relaxed(ltc->regs + ROCKCHIP_SPI_TXFLR);
		sr = readl_relaxed(ltc->regs + ROCKCHIP_SPI_SR);
		status = dmaengine_tx_status(ltc->tx_chan, ltc->dma_cookie,
					     &after);
		end_ns = ktime_get_raw_ns();
		if (status != DMA_IN_PROGRESS ||
		    end_ns - begin_ns > RK_LTC_OBSERVATION_MAX_NS ||
		    before.residue != after.residue)
			continue;
		ret = rk_ltc_output_position(ltc, after.residue, fifo,
					     sr & SR_BUSY, &samples);
		if (ret)
			continue;

		/*
		 * PL330's SAR may include one prefetched 16-byte burst not yet
		 * in TXFLR. Together with the shifter and snapshot span this
		 * bounds the observation error; it is not sample-exact timing.
		 */
		ltc->sync_reference_ns = ref_ns;
		ltc->sync_output_samples = samples;
		ltc->sync_fifo_bytes = fifo;
		ltc->sync_observation_span_ns = end_ns - begin_ns;
		ltc->sync_observation_valid = true;
		return 0;
	}

	ltc->sync_observation_rejects++;
	ltc->sync_state = RK_LTC_SYNC_STATE_HOLD;
	ltc->sync_lock_streak = 0;
	return -EAGAIN;
}

static int rk_ltc_boundary_reference(struct rk_ltc *ltc, s64 *ref_ns)
{
	u64 queued, delay_ns;

	if (!ltc->sync_observation_valid || !ltc->actual_sample_rate ||
	    ltc->samples_generated < ltc->sync_output_samples)
		return -EAGAIN;
	queued = ltc->samples_generated - ltc->sync_output_samples;
	if (queued > ((u64)ltc->ring_bytes + RK_LTC_SPI_FIFO_BYTES + 1) * 8)
		return -EAGAIN;
	delay_ns = div64_u64(queued * NSEC_PER_SEC, ltc->actual_sample_rate);
	if (check_add_overflow(ltc->sync_reference_ns, (s64)delay_ns, ref_ns))
		return -ERANGE;
	return 0;
}

static u64 rk_ltc_frame_frac_q16(u64 frame_rem, u64 frame_divisor)
{
	return div64_u64(frame_rem * RK_LTC_PHASE_ONE, frame_divisor);
}

static u64 rk_ltc_expected_phase_q16(struct rk_ltc *ltc, s64 ref_ns,
				     struct rk_ltc_timecode *tc_out)
{
	bool drop = ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME;
	u64 denom = (u64)ltc->rate.codeword_den * NSEC_PER_SEC;
	u64 rem, frac;
	s64 sec, scaled, frame, linear, sec_rem;
	s32 nsec;
	s64 period = rk_ltc_frames_per_day(&ltc->rate, drop);

	if (!ltc->sync_epoch_valid) {
		ltc->sync_epoch_ns = ref_ns - rk_ltc_mod_day_ns(ref_ns);
		ltc->sync_epoch_valid = true;
	}

	/* Split first: an epoch-sized nanosecond value times 30000 overflows. */
	sec = div_s64_rem(ref_ns, NSEC_PER_SEC, &nsec);
	sec -= div_s64(ltc->sync_epoch_ns, NSEC_PER_SEC);
	if (nsec < 0) {
		sec--;
		nsec += NSEC_PER_SEC;
	}
	scaled = sec * ltc->rate.codeword_num;
	frame = div_s64(scaled, ltc->rate.codeword_den);
	sec_rem = scaled - frame * ltc->rate.codeword_den;
	if (sec_rem < 0) {
		frame--;
		sec_rem += ltc->rate.codeword_den;
	}
	frame += (s64)div64_u64_rem((u64)sec_rem * NSEC_PER_SEC +
				   (u64)nsec * ltc->rate.codeword_num, denom, &rem);
	linear = frame % period;
	if (linear < 0)
		linear += period;
	frac = rk_ltc_frame_frac_q16(rem, denom);
	if (tc_out)
		rk_ltc_linear_to_tc(linear, &ltc->rate, drop, tc_out);

	return (linear << RK_LTC_PHASE_SHIFT) + frac;
}

static s64 rk_ltc_phase_diff_q16(u64 expected, u64 current_phase, u64 period)
{
	s64 diff = (s64)expected - (s64)current_phase;
	s64 half = (s64)period / 2;

	if (diff > half)
		diff -= period;
	else if (diff < -half)
		diff += period;

	return diff;
}

static s64 rk_ltc_phase_q16_to_ns(struct rk_ltc *ltc, s64 phase)
{
	u64 ns_per_frame;

	ns_per_frame = div64_u64((u64)ltc->rate.codeword_den * NSEC_PER_SEC,
				 ltc->rate.codeword_num);

	return (s64)mul_s64_u64_shr(phase, ns_per_frame,
				    RK_LTC_PHASE_SHIFT);
}

static s64 rk_ltc_abs64(s64 value)
{
	return value < 0 ? -value : value;
}

static void rk_ltc_phase_q16_to_nearest_tc(struct rk_ltc *ltc, u64 phase,
					   struct rk_ltc_timecode *tc)
{
	bool drop = ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME;
	u64 frame, period;

	period = rk_ltc_frames_per_day(&ltc->rate, drop);
	frame = div64_u64(phase + (RK_LTC_PHASE_ONE / 2),
			  RK_LTC_PHASE_ONE);
	frame %= period;
	rk_ltc_linear_to_tc(frame, &ltc->rate, drop, tc);
}

static s64 rk_ltc_clamp_pll_error(s64 phase_ns)
{
	return clamp_t(s64, phase_ns, -RK_LTC_PLL_ERROR_CLAMP_NS,
		       RK_LTC_PLL_ERROR_CLAMP_NS);
}

static int rk_ltc_validate_sync_config(struct rk_ltc_sync_config *cfg)
{
	switch (cfg->source) {
	case RK_LTC_SYNC_SOURCE_NONE:
	case RK_LTC_SYNC_SOURCE_REALTIME:
	case RK_LTC_SYNC_SOURCE_TAI:
	case RK_LTC_SYNC_SOURCE_MONOTONIC:
	case RK_LTC_SYNC_SOURCE_PTP_TAI:
		break;
	default:
		return -EINVAL;
	}

	if (cfg->ptp_index < -1)
		return -EINVAL;
	if (cfg->ptp_index >= 0) {
		if (cfg->source != RK_LTC_SYNC_SOURCE_PTP_TAI)
			return -EINVAL;
#if !IS_REACHABLE(CONFIG_PTP_1588_CLOCK)
		return -EOPNOTSUPP;
#endif
	}

	if (!cfg->max_slew_ppm)
		cfg->max_slew_ppm = RK_LTC_DEFAULT_MAX_SLEW_PPM;
	if (cfg->max_slew_ppm < 0 ||
	    cfg->max_slew_ppm > RK_LTC_MAX_SLEW_PPM)
		return -EINVAL;
	if (!cfg->lock_threshold_ms)
		cfg->lock_threshold_ms = RK_LTC_DEFAULT_LOCK_THRESHOLD_MS;
	if (cfg->lock_threshold_ms < 0)
		return -EINVAL;
	if (cfg->jump_threshold_ms < 0)
		return -EINVAL;
	if (!cfg->pll_kp_ppb_per_ms)
		cfg->pll_kp_ppb_per_ms = RK_LTC_DEFAULT_PLL_KP_PPB_PER_MS;
	if (!cfg->pll_ki_ppb_per_ms_s)
		cfg->pll_ki_ppb_per_ms_s =
			RK_LTC_DEFAULT_PLL_KI_PPB_PER_MS_S;
	if (cfg->pll_kp_ppb_per_ms < 0 ||
	    cfg->pll_kp_ppb_per_ms > RK_LTC_MAX_PLL_GAIN ||
	    cfg->pll_ki_ppb_per_ms_s < 0 ||
	    cfg->pll_ki_ppb_per_ms_s > RK_LTC_MAX_PLL_GAIN ||
	    cfg->pll_deadband_us < 0)
		return -EINVAL;
	if (cfg->source != RK_LTC_SYNC_SOURCE_NONE)
		cfg->flags |= RK_LTC_SYNC_FLAG_JAM_ON_START;

	return 0;
}

static void rk_ltc_sync_reset_locked(struct rk_ltc *ltc)
{
	ltc->sync_correction_ppb = 0;
	ltc->sync_integral_ppb = 0;
	ltc->sync_integral_rem = 0;
	ltc->sync_phase_error_ns = 0;
	ltc->sync_updates = 0;
	ltc->sync_locked_updates = 0;
	ltc->sync_jump_count = 0;
	ltc->sync_last_jump_error_ns = 0;
	ltc->sync_observation_valid = false;
	ltc->sync_observation_rejects = 0;
	ltc->sync_lock_streak = 0;
	ltc->sync_state = ltc->sync_cfg.source == RK_LTC_SYNC_SOURCE_NONE ?
			  RK_LTC_SYNC_STATE_OFF : RK_LTC_SYNC_STATE_SLEWING;
}

static s32 rk_ltc_sync_pll_step_locked(struct rk_ltc *ltc, s64 phase_ns)
{
	const struct rk_ltc_sync_config *cfg = &ltc->sync_cfg;
	s64 error_ns = rk_ltc_clamp_pll_error(phase_ns);
	s64 error_us, integrated;
	s64 p_ppb, i_delta_ppb, i_ppb, i_unclamped, correction;
	s64 max_ppb = (s64)cfg->max_slew_ppm * 1000;
	s32 remainder;

	if (cfg->pll_deadband_us &&
	    rk_ltc_abs64(error_ns) <=
	    (s64)cfg->pll_deadband_us * RK_LTC_NSEC_PER_USEC)
		error_ns = 0;

	error_us = div_s64(error_ns, RK_LTC_NSEC_PER_USEC);
	p_ppb = div_s64(error_us * cfg->pll_kp_ppb_per_ms, 1000);

	/*
	 * Integrate with the exact nominal codeword interval, retaining the
	 * sub-ppb remainder. Rounding error_us * dt to whole us*s first would
	 * silently disable integral correction below ~30 us at 30 fps.
	 * The 10 s error clamp and gain/rate limits keep the numerator in s64.
	 */
	integrated = error_us * cfg->pll_ki_ppb_per_ms_s *
		     ltc->rate.codeword_den + ltc->sync_integral_rem;
	i_delta_ppb = div_s64_rem(integrated,
				  1000 * ltc->rate.codeword_num, &remainder);
	i_unclamped = (s64)ltc->sync_integral_ppb + i_delta_ppb;
	i_ppb = clamp_t(s64, i_unclamped, -max_ppb, max_ppb);
	if (i_ppb != i_unclamped)
		remainder = 0;
	correction = clamp_t(s64, p_ppb + i_ppb, -max_ppb, max_ppb);

	if (correction != p_ppb + i_ppb) {
		i_ppb = clamp_t(s64, correction - p_ppb, -max_ppb, max_ppb);
		remainder = 0;
	}

	ltc->sync_integral_ppb = i_ppb;
	ltc->sync_integral_rem = remainder;
	ltc->sync_correction_ppb = correction;

	return correction;
}

static void rk_ltc_sync_update_locked(struct rk_ltc *ltc)
{
	const struct rk_ltc_sync_config *cfg = &ltc->sync_cfg;
	u64 expected, current_phase, period;
	s64 ref_ns, phase, phase_ns;
	s64 abs_phase_ns;
	bool locked;
	int ret;

	if (cfg->source == RK_LTC_SYNC_SOURCE_NONE) {
		ltc->sync_state = RK_LTC_SYNC_STATE_OFF;
		ltc->sync_correction_ppb = 0;
		ltc->sync_integral_ppb = 0;
		ltc->sync_integral_rem = 0;
		ltc->sync_phase_error_ns = 0;
		return;
	}

	if (ltc->cfg.mode != RK_LTC_MODE_NORMAL) {
		ltc->sync_state = RK_LTC_SYNC_STATE_HOLD;
		ltc->sync_correction_ppb = 0;
		return;
	}

	/* Prefill has no playback position and must not drive the servo. */
	if (!ltc->running)
		return;

	ret = rk_ltc_boundary_reference(ltc, &ref_ns);
	if (ret) {
		ltc->sync_state = RK_LTC_SYNC_STATE_HOLD;
		ltc->sync_lock_streak = 0;
		return;
	}

	if (!ltc->sync_epoch_valid) {
		ltc->sync_epoch_ns = ref_ns - rk_ltc_mod_day_ns(ref_ns);
		ltc->sync_epoch_valid = true;
	}
	if (check_add_overflow(ref_ns, cfg->offset_ns, &ref_ns)) {
		ltc->sync_state = RK_LTC_SYNC_STATE_HOLD;
		ltc->sync_lock_streak = 0;
		return;
	}
	expected = rk_ltc_expected_phase_q16(ltc, ref_ns, NULL);
	current_phase = rk_ltc_tc_to_linear(&ltc->gen.tc, &ltc->rate,
					    ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
	current_phase <<= RK_LTC_PHASE_SHIFT;
	period = rk_ltc_frames_per_day(&ltc->rate,
				       ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
	period <<= RK_LTC_PHASE_SHIFT;

	phase = rk_ltc_phase_diff_q16(expected, current_phase, period);
	phase_ns = rk_ltc_phase_q16_to_ns(ltc, phase);

	abs_phase_ns = rk_ltc_abs64(phase_ns);
	if (cfg->jump_threshold_ms &&
	    abs_phase_ns >
	    (s64)cfg->jump_threshold_ms * RK_LTC_NSEC_PER_MSEC) {
		rk_ltc_phase_q16_to_nearest_tc(ltc, expected, &ltc->gen.tc);
		current_phase = rk_ltc_tc_to_linear(&ltc->gen.tc, &ltc->rate,
				ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
		current_phase <<= RK_LTC_PHASE_SHIFT;
		phase = rk_ltc_phase_diff_q16(expected, current_phase,
					      period);
		ltc->sync_jump_count++;
		ltc->sync_last_jump_error_ns = phase_ns;
		ltc->sync_integral_ppb = 0;
		ltc->sync_integral_rem = 0;
		ltc->sync_lock_streak = 0;
	}

	phase_ns = rk_ltc_phase_q16_to_ns(ltc, phase);
	abs_phase_ns = rk_ltc_abs64(phase_ns);
	rk_ltc_sync_pll_step_locked(ltc, phase_ns);
	locked = abs_phase_ns <=
		 (s64)cfg->lock_threshold_ms * RK_LTC_NSEC_PER_MSEC;
	if (locked)
		ltc->sync_lock_streak = min_t(u32, ltc->sync_lock_streak + 1,
			DIV_ROUND_UP(ltc->rate.codeword_num, ltc->rate.codeword_den));
	else
		ltc->sync_lock_streak = 0;

	ltc->sync_phase_error_ns = phase_ns;
	ltc->sync_state = ltc->sync_lock_streak >=
		DIV_ROUND_UP(ltc->rate.codeword_num, ltc->rate.codeword_den) ?
		RK_LTC_SYNC_STATE_LOCKED : RK_LTC_SYNC_STATE_SLEWING;
	ltc->sync_updates++;
	if (locked)
		ltc->sync_locked_updates++;
}

static int rk_ltc_sync_jam_start_locked(struct rk_ltc *ltc)
{
	struct rk_ltc_timecode tc;
	u64 phase;
	s64 ref_ns;
	int ret;

	if (ltc->sync_cfg.source == RK_LTC_SYNC_SOURCE_NONE ||
	    !(ltc->sync_cfg.flags & RK_LTC_SYNC_FLAG_JAM_ON_START) ||
	    ltc->cfg.mode != RK_LTC_MODE_NORMAL)
		return 0;

	ret = rk_ltc_read_sync_time(&ltc->sync_cfg, &ref_ns);
	if (ret)
		return ret;

	ltc->sync_epoch_ns = ref_ns - rk_ltc_mod_day_ns(ref_ns);
	ltc->sync_epoch_valid = true;
	if (check_add_overflow(ref_ns, ltc->sync_cfg.offset_ns, &ref_ns))
		return -ERANGE;
	phase = rk_ltc_expected_phase_q16(ltc, ref_ns, NULL);
	rk_ltc_phase_q16_to_nearest_tc(ltc, phase, &tc);
	ltc->tc = tc;
	ltc->current_tc = tc;

	return 0;
}

static int rk_ltc_sync_set_running_time_locked(struct rk_ltc *ltc,
					       const struct rk_ltc_timecode *tc)
{
	unsigned long flags;
	s64 ref_ns, offset;
	u64 expected, target, period;
	bool drop = ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME;
	int ret;

	if (ltc->sync_cfg.source == RK_LTC_SYNC_SOURCE_NONE)
		return -EBUSY;

	spin_lock_irqsave(&ltc->state_lock, flags);
	ret = rk_ltc_observe_playback_locked(ltc);
	if (!ret)
		ret = rk_ltc_boundary_reference(ltc, &ref_ns);
	if (ret)
		goto out;
	expected = rk_ltc_expected_phase_q16(ltc, ref_ns, NULL);
	target = rk_ltc_tc_to_linear(tc, &ltc->rate, drop) << RK_LTC_PHASE_SHIFT;
	period = rk_ltc_frames_per_day(&ltc->rate, drop) << RK_LTC_PHASE_SHIFT;
	offset = rk_ltc_phase_q16_to_ns(ltc,
			rk_ltc_phase_diff_q16(target, expected, period));
	ltc->sync_cfg.offset_ns = offset;
	ltc->sync_lock_streak = 0;
	ltc->sync_state = RK_LTC_SYNC_STATE_SLEWING;
out:
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	return ret;
}

static void rk_ltc_put_bcd(u8 *bits, unsigned int start,
			   unsigned int count, unsigned int value)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		bits[start + i] = (value >> i) & 1;
}

static unsigned int rk_ltc_count_zeros(const u8 *bits, unsigned int skip)
{
	unsigned int i, zeros = 0;

	for (i = 0; i < 64; i++) {
		if (i == skip)
			continue;
		if (!bits[i])
			zeros++;
	}

	return zeros;
}

static void rk_ltc_build_frame_bits(struct rk_ltc *ltc)
{
	struct rk_ltc_gen *gen = &ltc->gen;
	const struct rk_ltc_timecode *tc = &gen->tc;
	const u32 flags = ltc->cfg.flags;
	unsigned int i, fmt, pc_bit, bgf0_bit, bgf1_bit, bgf2_bit;
	unsigned int zeros;
	u8 *bits = gen->bits;

	if (ltc->cfg.mode == RK_LTC_MODE_RAW_FRAME) {
		for (i = 0; i < 80; i++)
			bits[i] = !!(ltc->cfg.raw_frame[i / 8] & BIT(i % 8));
		return;
	}

	memset(bits, 0, 80);

	rk_ltc_put_bcd(bits, 0, 4, tc->frames % 10);
	rk_ltc_put_bcd(bits, 8, 2, tc->frames / 10);
	rk_ltc_put_bcd(bits, 16, 4, tc->seconds % 10);
	rk_ltc_put_bcd(bits, 24, 3, tc->seconds / 10);
	rk_ltc_put_bcd(bits, 32, 4, tc->minutes % 10);
	rk_ltc_put_bcd(bits, 40, 3, tc->minutes / 10);
	rk_ltc_put_bcd(bits, 48, 4, tc->hours % 10);
	rk_ltc_put_bcd(bits, 56, 2, tc->hours / 10);

	for (i = 0; i < 4; i++) {
		bits[4 + i] = !!(ltc->cfg.user_bits[0] & BIT(i));
		bits[12 + i] = !!(ltc->cfg.user_bits[1] & BIT(i));
		bits[20 + i] = !!(ltc->cfg.user_bits[2] & BIT(i));
		bits[28 + i] = !!(ltc->cfg.user_bits[3] & BIT(i));
		bits[36 + i] = !!(ltc->cfg.user_bits[4] & BIT(i));
		bits[44 + i] = !!(ltc->cfg.user_bits[5] & BIT(i));
		bits[52 + i] = !!(ltc->cfg.user_bits[6] & BIT(i));
		bits[60 + i] = !!(ltc->cfg.user_bits[7] & BIT(i));
	}

	fmt = ltc->rate.addr_fps;
	if (flags & RK_LTC_FLAG_DROP_FRAME)
		bits[10] = 1;
	if ((flags & RK_LTC_FLAG_COLOR_FRAME) && fmt != 24)
		bits[11] = 1;

	if (fmt == 25) {
		pc_bit = 59;
		bgf0_bit = 27;
		bgf1_bit = 58;
		bgf2_bit = 43;
	} else {
		pc_bit = 27;
		bgf0_bit = 43;
		bgf1_bit = 58;
		bgf2_bit = 59;
	}

	bits[bgf0_bit] = !!(flags & RK_LTC_FLAG_BGF0);
	bits[bgf1_bit] = !!(flags & RK_LTC_FLAG_BGF1);
	bits[bgf2_bit] = !!(flags & RK_LTC_FLAG_BGF2);
	bits[pc_bit] = 0;

	zeros = rk_ltc_count_zeros(bits, pc_bit);
	bits[pc_bit] = (zeros & 1) ? 1 : 0;

	for (i = 0; i < 16; i++)
		bits[64 + i] = rk_ltc_sync_bits[i];
}

static u32 rk_ltc_next_bmc_half(struct rk_ltc *ltc)
{
	struct rk_ltc_gen *gen = &ltc->gen;
	u64 dividend, divisor, rem;
	u64 len;
	s64 scale_s64 = 1000000000LL + ltc->sync_correction_ppb;
	u64 scale = scale_s64 > 0 ? (u64)scale_s64 : 1;

	dividend = (u64)ltc->actual_sample_rate * ltc->rate.codeword_den *
		   1000000000ULL;
	divisor = 160ULL * ltc->rate.codeword_num * scale;
	if (gen->half_rem >= divisor) {
		u64 old_rem;

		div64_u64_rem(gen->half_rem, divisor, &old_rem);
		gen->half_rem = old_rem;
	}
	dividend += gen->half_rem;
	len = div64_u64_rem(dividend, divisor, &rem);
	gen->half_rem = rem;

	return max_t(u32, (u32)len, 1);
}

static u32 rk_ltc_next_square_half(struct rk_ltc *ltc)
{
	struct rk_ltc_gen *gen = &ltc->gen;
	u64 dividend, rem;
	u64 len;

	dividend = (u64)ltc->actual_sample_rate + gen->diag_rem;
	len = div64_u64_rem(dividend, 2000, &rem);
	gen->diag_rem = rem;

	return max_t(u32, (u32)len, 1);
}

static u8 rk_ltc_next_bmc_sample(struct rk_ltc *ltc)
{
	struct rk_ltc_gen *gen = &ltc->gen;
	u8 bit;

	while (!gen->half_remaining) {
		if (!gen->half_index) {
			if (gen->bit_index == 80) {
				gen->bit_index = 0;
				ltc->codewords_generated++;
				if (ltc->cfg.mode != RK_LTC_MODE_RAW_FRAME)
					rk_ltc_increment_time(&gen->tc, &ltc->rate,
						ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
			}
			if (!gen->bit_index) {
				if (ltc->cfg.mode != RK_LTC_MODE_RAW_FRAME)
					rk_ltc_sync_update_locked(ltc);
				rk_ltc_build_frame_bits(ltc);
			}

			gen->level ^= 1;
			gen->half_remaining = rk_ltc_next_bmc_half(ltc);
			gen->half_index = 1;
		} else {
			if (gen->bits[gen->bit_index])
				gen->level ^= 1;

			gen->half_remaining = rk_ltc_next_bmc_half(ltc);
			gen->half_index = 0;
			gen->bit_index++;
		}
	}

	bit = gen->level;
	gen->half_remaining--;
	ltc->samples_generated++;

	return bit;
}

static u8 rk_ltc_next_diag_sample(struct rk_ltc *ltc)
{
	struct rk_ltc_gen *gen = &ltc->gen;
	u8 bit;

	switch (ltc->cfg.mode) {
	case RK_LTC_MODE_IDLE_LOW:
		ltc->samples_generated++;
		return 0;
	case RK_LTC_MODE_IDLE_HIGH:
		ltc->samples_generated++;
		return 1;
	case RK_LTC_MODE_ALT_SAMPLES:
		bit = ltc->samples_generated & 1;
		ltc->samples_generated++;
		return bit;
	case RK_LTC_MODE_SQUARE_1KHZ:
		if (!gen->diag_remaining) {
			if (ltc->samples_generated)
				gen->level ^= 1;
			gen->diag_remaining = rk_ltc_next_square_half(ltc);
		}
		bit = gen->level;
		gen->diag_remaining--;
		ltc->samples_generated++;
		return bit;
	default:
		return rk_ltc_next_bmc_sample(ltc);
	}
}

static u8 rk_ltc_next_byte(struct rk_ltc *ltc)
{
	u8 val = 0;
	unsigned int i;

	for (i = 0; i < 8; i++) {
		if (rk_ltc_next_diag_sample(ltc))
			val |= BIT(7 - i);
	}

	return val;
}

static void rk_ltc_fill_period_locked(struct rk_ltc *ltc, u32 period)
{
	u8 *dst = ltc->ring + period * ltc->period_bytes;
	u32 i;

	ltc->period_tc[period] = ltc->gen.tc;
	for (i = 0; i < ltc->period_bytes; i++)
		dst[i] = rk_ltc_next_byte(ltc);
}

static void rk_ltc_gen_reset_locked(struct rk_ltc *ltc)
{
	memset(&ltc->gen, 0, sizeof(ltc->gen));
	ltc->gen.tc = ltc->tc;
	ltc->current_tc = ltc->tc;
	ltc->codewords_generated = 0;
	ltc->samples_generated = 0;
	ltc->dma_periods = 0;
	ltc->underruns = 0;
	ltc->period_pos = 0;
	rk_ltc_sync_reset_locked(ltc);
}

static irqreturn_t rk_ltc_irq(int irq, void *data)
{
	struct rk_ltc *ltc = data;
	u32 isr = readl_relaxed(ltc->regs + ROCKCHIP_SPI_ISR);
	unsigned long flags;

	if (!isr)
		return IRQ_NONE;

	spin_lock_irqsave(&ltc->state_lock, flags);
	ltc->irq_count++;
	if (isr & INT_TF_OVERFLOW)
		ltc->underruns++;
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	writel_relaxed(0xffffffff, ltc->regs + ROCKCHIP_SPI_ICR);

	return IRQ_HANDLED;
}

static void rk_ltc_dma_callback(void *data)
{
	struct rk_ltc *ltc = data;
	unsigned long flags;
	u32 completed, next;

	spin_lock_irqsave(&ltc->state_lock, flags);
	if (!ltc->running) {
		spin_unlock_irqrestore(&ltc->state_lock, flags);
		return;
	}

	completed = ltc->period_pos;
	next = (completed + 1) % ltc->periods;
	ltc->dma_periods++;
	ltc->current_tc = ltc->period_tc[next];
	rk_ltc_observe_playback_locked(ltc);
	rk_ltc_fill_period_locked(ltc, completed);
	ltc->period_pos = next;

	if (readl_relaxed(ltc->regs + ROCKCHIP_SPI_TXFLR) == 0 &&
	    !(readl_relaxed(ltc->regs + ROCKCHIP_SPI_SR) & SR_BUSY))
		ltc->underruns++;

	spin_unlock_irqrestore(&ltc->state_lock, flags);
}

static int rk_ltc_config_spi_locked(struct rk_ltc *ltc)
{
	struct dma_slave_config txconf = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr = ltc->regs_phys + ROCKCHIP_SPI_TXDR,
		.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE,
		.dst_maxburst = 16,
	};
	u32 cr0, div;
	unsigned long parent;
	int ret;

	ret = clk_set_rate(ltc->spiclk,
			   (unsigned long)ltc->sample_rate * 250UL);
	if (ret)
		dev_warn(ltc->dev, "failed to request SPI parent clock: %d\n", ret);

	parent = clk_get_rate(ltc->spiclk);
	div = 2 * DIV_ROUND_UP(parent, 2 * ltc->sample_rate);
	if (div < BAUDR_SCKDV_MIN || div > BAUDR_SCKDV_MAX)
		return -EINVAL;

	ltc->divider = div;
	ltc->actual_sample_rate = parent / div;

	ret = dmaengine_slave_config(ltc->tx_chan, &txconf);
	if (ret)
		return ret;

	rk_ltc_spi_enable(ltc, false);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_DMACR);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_IMR);
	writel_relaxed(0xffffffff, ltc->regs + ROCKCHIP_SPI_ICR);

	cr0 = CR0_FRF_SPI << CR0_FRF_OFFSET |
	      CR0_BHT_8BIT << CR0_BHT_OFFSET |
	      CR0_SSD_ONE << CR0_SSD_OFFSET |
	      CR0_EM_BIG << CR0_EM_OFFSET |
	      CR0_FBM_MSB << CR0_FBM_OFFSET |
	      CR0_XFM_TO << CR0_XFM_OFFSET |
	      CR0_OPM_MASTER << CR0_OPM_OFFSET |
	      CR0_CSM_KEEP << CR0_CSM_OFFSET |
	      CR0_DFS_8BIT << CR0_DFS_OFFSET;

	writel_relaxed(cr0, ltc->regs + ROCKCHIP_SPI_CTRLR0);
	writel_relaxed(0xffff, ltc->regs + ROCKCHIP_SPI_CTRLR1);
	writel_relaxed(div, ltc->regs + ROCKCHIP_SPI_BAUDR);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_TXFTLR);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_RXFTLR);
	writel_relaxed(31, ltc->regs + ROCKCHIP_SPI_DMATDLR);
	writel_relaxed(INT_TF_OVERFLOW, ltc->regs + ROCKCHIP_SPI_IMR);
	writel_relaxed(TF_DMA_EN, ltc->regs + ROCKCHIP_SPI_DMACR);
	writel_relaxed(1, ltc->regs + ROCKCHIP_SPI_SER);

	return 0;
}

static int rk_ltc_start_locked(struct rk_ltc *ltc)
{
	struct dma_async_tx_descriptor *desc;
	unsigned long flags;
	u32 i;
	int ret;

	if (ltc->running)
		return 0;

	ret = rk_ltc_config_spi_locked(ltc);
	if (ret)
		return ret;

	ltc->sync_epoch_valid = false;
	ret = rk_ltc_sync_jam_start_locked(ltc);
	if (ret)
		return ret;

	spin_lock_irqsave(&ltc->state_lock, flags);
	rk_ltc_gen_reset_locked(ltc);
	for (i = 0; i < ltc->periods; i++)
		rk_ltc_fill_period_locked(ltc, i);
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	desc = dmaengine_prep_dma_cyclic(ltc->tx_chan, ltc->dma_handle,
					 ltc->ring_bytes, ltc->period_bytes,
					 DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc)
		return -EIO;

	desc->callback = rk_ltc_dma_callback;
	desc->callback_param = ltc;
	ltc->dma_cookie = dmaengine_submit(desc);
	if (dma_submit_error(ltc->dma_cookie))
		return dma_submit_error(ltc->dma_cookie);

	spin_lock_irqsave(&ltc->state_lock, flags);
	ltc->running = true;
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	rk_ltc_spi_enable(ltc, true);
	dma_async_issue_pending(ltc->tx_chan);

	return 0;
}

static void rk_ltc_stop_locked(struct rk_ltc *ltc)
{
	unsigned long flags;

	if (!ltc->running)
		return;

	spin_lock_irqsave(&ltc->state_lock, flags);
	ltc->running = false;
	ltc->tc = ltc->current_tc;
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	rk_ltc_spi_enable(ltc, false);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_DMACR);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_IMR);
	writel_relaxed(0, ltc->regs + ROCKCHIP_SPI_SER);
	writel_relaxed(0xffffffff, ltc->regs + ROCKCHIP_SPI_ICR);
	dmaengine_terminate_sync(ltc->tx_chan);
}

static int rk_ltc_apply_config_locked(struct rk_ltc *ltc,
				      const struct rk_ltc_config *new_cfg)
{
	struct rk_ltc_config cfg = *new_cfg;
	struct rk_ltc_rate rate;
	int ret;

	if (!cfg.sample_rate)
		cfg.sample_rate = RK_LTC_DEFAULT_SAMPLE_RATE;
	if (!cfg.fps_den)
		cfg.fps_den = 1;
	if (cfg.mode > RK_LTC_MODE_RAW_FRAME)
		return -EINVAL;

	ret = rk_ltc_select_rate(&cfg, &rate);
	if (ret)
		return ret;

	if (ltc->running)
		return -EBUSY;

	ltc->cfg = cfg;
	ltc->rate = rate;
	ltc->sample_rate = cfg.sample_rate;
	rk_ltc_clamp_time(&ltc->tc, &ltc->rate,
			  ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
	ltc->current_tc = ltc->tc;

	return 0;
}

static void rk_ltc_fill_status(struct rk_ltc *ltc, struct rk_ltc_status *st)
{
	struct dma_tx_state state;
	enum dma_status dma_status;
	unsigned long flags;
	bool running;

	memset(st, 0, sizeof(*st));

	spin_lock_irqsave(&ltc->state_lock, flags);
	running = ltc->running;
	st->running = running;
	st->mode = ltc->cfg.mode;
	st->fps_num = ltc->cfg.fps_num;
	st->fps_den = ltc->cfg.fps_den;
	st->drop_frame = !!(ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME);
	st->high_frame_rate = ltc->rate.high_frame_rate;
	st->sample_rate = ltc->sample_rate;
	st->actual_sample_rate = ltc->actual_sample_rate;
	st->divider = ltc->divider;
	st->timecode = ltc->current_tc;
	st->codewords = ltc->codewords_generated;
	st->samples = ltc->samples_generated;
	st->dma_periods = ltc->dma_periods;
	st->underruns = ltc->underruns;
	st->irq_count = ltc->irq_count;
	st->decoder_locks = ltc->decoder_locks;
	st->decoder_errors = ltc->decoder_errors;
	st->sync_source = ltc->sync_cfg.source;
	st->sync_state = ltc->sync_state;
	st->sync_correction_ppb = ltc->sync_correction_ppb;
	st->sync_correction_ppm = div_s64(ltc->sync_correction_ppb, 1000);
	st->sync_integral_ppb = ltc->sync_integral_ppb;
	st->sync_max_slew_ppm = ltc->sync_cfg.max_slew_ppm;
	st->sync_phase_error_ns = ltc->sync_phase_error_ns;
	st->sync_updates = ltc->sync_updates;
	st->sync_locked_updates = ltc->sync_locked_updates;
	st->sync_offset_ns = ltc->sync_cfg.offset_ns;
	st->sync_ptp_index = ltc->sync_cfg.ptp_index;
	st->sync_jump_count = ltc->sync_jump_count;
	st->sync_last_jump_error_ns = ltc->sync_last_jump_error_ns;
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	if (running) {
		dma_status = dmaengine_tx_status(ltc->tx_chan, ltc->dma_cookie,
						 &state);
		if (dma_status != DMA_ERROR && state.residue <= ltc->ring_bytes)
			st->dma_fill_bytes = ltc->ring_bytes - state.residue;
	}
}

static int rk_ltc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct rk_ltc *ltc = container_of(miscdev, struct rk_ltc, miscdev);

	file->private_data = ltc;

	return 0;
}

static long rk_ltc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk_ltc *ltc = file->private_data;
	struct rk_ltc_config cfg;
	struct rk_ltc_sync_config sync_cfg;
	struct rk_ltc_timecode tc;
	struct rk_ltc_status st;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&ltc->lock);

	switch (cmd) {
	case RK_LTC_IOC_START:
		ret = rk_ltc_start_locked(ltc);
		break;
	case RK_LTC_IOC_STOP:
		rk_ltc_stop_locked(ltc);
		break;
	case RK_LTC_IOC_SET_CONFIG:
		if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg))) {
			ret = -EFAULT;
			break;
		}
		ret = rk_ltc_apply_config_locked(ltc, &cfg);
		break;
	case RK_LTC_IOC_GET_CONFIG:
		cfg = ltc->cfg;
		if (copy_to_user((void __user *)arg, &cfg, sizeof(cfg)))
			ret = -EFAULT;
		break;
	case RK_LTC_IOC_SET_TIMECODE:
	{
		if (copy_from_user(&tc, (void __user *)arg, sizeof(tc))) {
			ret = -EFAULT;
			break;
		}
		if (!rk_ltc_time_valid(&tc, &ltc->rate,
				       ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME)) {
			ret = -EINVAL;
			break;
		}
		if (ltc->running)
			ret = rk_ltc_sync_set_running_time_locked(ltc, &tc);
		else {
			ltc->tc = tc;
			ltc->current_tc = tc;
		}
		break;
	}
	case RK_LTC_IOC_GET_TIMECODE:
		tc = ltc->current_tc;
		if (copy_to_user((void __user *)arg, &tc, sizeof(tc)))
			ret = -EFAULT;
		break;
	case RK_LTC_IOC_GET_STATUS:
		rk_ltc_fill_status(ltc, &st);
		if (copy_to_user((void __user *)arg, &st, sizeof(st)))
			ret = -EFAULT;
		break;
	case RK_LTC_IOC_SET_SYNC:
		if (copy_from_user(&sync_cfg, (void __user *)arg,
				   sizeof(sync_cfg))) {
			ret = -EFAULT;
			break;
		}
		ret = rk_ltc_validate_sync_config(&sync_cfg);
		if (ret)
			break;
		spin_lock_irqsave(&ltc->state_lock, flags);
		if (ltc->sync_cfg.source != sync_cfg.source ||
		    ltc->sync_cfg.ptp_index != sync_cfg.ptp_index)
			ltc->sync_epoch_valid = false;
		ltc->sync_cfg = sync_cfg;
		rk_ltc_sync_reset_locked(ltc);
		spin_unlock_irqrestore(&ltc->state_lock, flags);
		break;
	case RK_LTC_IOC_GET_SYNC:
		spin_lock_irqsave(&ltc->state_lock, flags);
		sync_cfg = ltc->sync_cfg;
		spin_unlock_irqrestore(&ltc->state_lock, flags);
		if (copy_to_user((void __user *)arg, &sync_cfg,
				 sizeof(sync_cfg)))
			ret = -EFAULT;
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&ltc->lock);

	return ret;
}

static const struct file_operations rk_ltc_fops = {
	.owner = THIS_MODULE,
	.open = rk_ltc_open,
	.unlocked_ioctl = rk_ltc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = rk_ltc_ioctl,
#endif
	.llseek = no_llseek,
};

static const char *rk_ltc_mode_name(u32 mode)
{
	switch (mode) {
	case RK_LTC_MODE_NORMAL:
		return "normal";
	case RK_LTC_MODE_IDLE_LOW:
		return "idle-low";
	case RK_LTC_MODE_IDLE_HIGH:
		return "idle-high";
	case RK_LTC_MODE_SQUARE_1KHZ:
		return "square-1khz";
	case RK_LTC_MODE_ALT_SAMPLES:
		return "alt-samples";
	case RK_LTC_MODE_RAW_FRAME:
		return "raw-frame";
	default:
		return "unknown";
	}
}

static const char *rk_ltc_sync_source_name(u32 source)
{
	switch (source) {
	case RK_LTC_SYNC_SOURCE_NONE:
		return "none";
	case RK_LTC_SYNC_SOURCE_REALTIME:
		return "realtime";
	case RK_LTC_SYNC_SOURCE_TAI:
		return "tai";
	case RK_LTC_SYNC_SOURCE_MONOTONIC:
		return "monotonic";
	case RK_LTC_SYNC_SOURCE_PTP_TAI:
		return "ptp-tai";
	default:
		return "unknown";
	}
}

static const char *rk_ltc_sync_state_name(u32 state)
{
	switch (state) {
	case RK_LTC_SYNC_STATE_OFF:
		return "off";
	case RK_LTC_SYNC_STATE_LOCKED:
		return "locked";
	case RK_LTC_SYNC_STATE_SLEWING:
		return "slewing";
	case RK_LTC_SYNC_STATE_HOLD:
		return "hold";
	default:
		return "unknown";
	}
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_status st;

	rk_ltc_fill_status(ltc, &st);

	return sysfs_emit(buf,
			  "running=%u mode=%s rate=%u/%u sample=%u actual=%u divider=%u tc=%02u:%02u:%02u:%02u periods=%llu underruns=%llu sync=%s state=%s phase_ns=%lld correction_ppb=%d jumps=%llu\n",
			  st.running, rk_ltc_mode_name(st.mode), st.fps_num,
			  st.fps_den, st.sample_rate, st.actual_sample_rate,
			  st.divider, st.timecode.hours, st.timecode.minutes,
			  st.timecode.seconds, st.timecode.frames,
			  st.dma_periods, st.underruns,
			  rk_ltc_sync_source_name(st.sync_source),
			  rk_ltc_sync_state_name(st.sync_state),
			  st.sync_phase_error_ns, st.sync_correction_ppb,
			  st.sync_jump_count);
}
static DEVICE_ATTR_RO(status);

static ssize_t running_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ltc->running);
}

static ssize_t running_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&ltc->lock);
	if (enable)
		ret = rk_ltc_start_locked(ltc);
	else
		rk_ltc_stop_locked(ltc);
	mutex_unlock(&ltc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(running);

static ssize_t mode_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", rk_ltc_mode_name(ltc->cfg.mode));
}

static int rk_ltc_parse_mode(const char *buf, u32 *mode)
{
	if (sysfs_streq(buf, "normal"))
		*mode = RK_LTC_MODE_NORMAL;
	else if (sysfs_streq(buf, "idle-low"))
		*mode = RK_LTC_MODE_IDLE_LOW;
	else if (sysfs_streq(buf, "idle-high"))
		*mode = RK_LTC_MODE_IDLE_HIGH;
	else if (sysfs_streq(buf, "square-1khz") || sysfs_streq(buf, "square"))
		*mode = RK_LTC_MODE_SQUARE_1KHZ;
	else if (sysfs_streq(buf, "alt-samples") || sysfs_streq(buf, "alt"))
		*mode = RK_LTC_MODE_ALT_SAMPLES;
	else if (sysfs_streq(buf, "raw-frame") || sysfs_streq(buf, "raw"))
		*mode = RK_LTC_MODE_RAW_FRAME;
	else
		return -EINVAL;

	return 0;
}

static ssize_t mode_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_config cfg;
	u32 mode;
	int ret;

	ret = rk_ltc_parse_mode(buf, &mode);
	if (ret)
		return ret;

	mutex_lock(&ltc->lock);
	cfg = ltc->cfg;
	cfg.mode = mode;
	ret = rk_ltc_apply_config_locked(ltc, &cfg);
	mutex_unlock(&ltc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(mode);

static ssize_t timecode_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_status st;

	rk_ltc_fill_status(ltc, &st);

	return sysfs_emit(buf, "%02u:%02u:%02u:%02u\n",
			  st.timecode.hours, st.timecode.minutes,
			  st.timecode.seconds, st.timecode.frames);
}

static ssize_t timecode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_timecode tc = {};
	unsigned int h, m, s, f;
	int ret;

	if (sscanf(buf, "%u:%u:%u%*[:;]%u", &h, &m, &s, &f) != 4)
		return -EINVAL;

	tc.hours = h;
	tc.minutes = m;
	tc.seconds = s;
	tc.frames = f;

	mutex_lock(&ltc->lock);
	if (!rk_ltc_time_valid(&tc, &ltc->rate,
			       ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME)) {
		ret = -EINVAL;
	} else if (ltc->running) {
		ret = rk_ltc_sync_set_running_time_locked(ltc, &tc);
	} else {
		ltc->tc = tc;
		ltc->current_tc = tc;
		ret = 0;
	}
	mutex_unlock(&ltc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(timecode);

static ssize_t rate_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u/%u\n", ltc->cfg.fps_num, ltc->cfg.fps_den);
}

static ssize_t rate_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_config cfg;
	unsigned int num, den = 1;
	int ret;

	if (sscanf(buf, "%u/%u", &num, &den) < 1)
		return -EINVAL;

	mutex_lock(&ltc->lock);
	cfg = ltc->cfg;
	cfg.fps_num = num;
	cfg.fps_den = den;
	ret = rk_ltc_apply_config_locked(ltc, &cfg);
	mutex_unlock(&ltc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(rate);

static ssize_t drop_frame_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n",
			  !!(ltc->cfg.flags & RK_LTC_FLAG_DROP_FRAME));
}

static ssize_t drop_frame_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);
	struct rk_ltc_config cfg;
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&ltc->lock);
	cfg = ltc->cfg;
	if (enable)
		cfg.flags |= RK_LTC_FLAG_DROP_FRAME;
	else
		cfg.flags &= ~RK_LTC_FLAG_DROP_FRAME;
	ret = rk_ltc_apply_config_locked(ltc, &cfg);
	mutex_unlock(&ltc->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(drop_frame);

static ssize_t underruns_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%llu\n", ltc->underruns);
}
static DEVICE_ATTR_RO(underruns);

static ssize_t actual_clock_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ltc->actual_sample_rate);
}
static DEVICE_ATTR_RO(actual_clock);

static ssize_t divider_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct rk_ltc *ltc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", ltc->divider);
}
static DEVICE_ATTR_RO(divider);

static struct attribute *rk_ltc_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_running.attr,
	&dev_attr_mode.attr,
	&dev_attr_timecode.attr,
	&dev_attr_rate.attr,
	&dev_attr_drop_frame.attr,
	&dev_attr_underruns.attr,
	&dev_attr_actual_clock.attr,
	&dev_attr_divider.attr,
	NULL,
};

static const struct attribute_group rk_ltc_attr_group = {
	.attrs = rk_ltc_attrs,
};

static int rk_ltc_debugfs_show(struct seq_file *s, void *unused)
{
	struct rk_ltc *ltc = s->private;
	struct rk_ltc_status st;
	unsigned long flags;
	u64 output_samples, span_ns, rejects;
	s64 reference_ns, epoch_ns;
	u32 fifo_bytes, lock_streak;
	bool valid;

	rk_ltc_fill_status(ltc, &st);
	spin_lock_irqsave(&ltc->state_lock, flags);
	valid = ltc->sync_observation_valid;
	reference_ns = ltc->sync_reference_ns;
	output_samples = ltc->sync_output_samples;
	span_ns = ltc->sync_observation_span_ns;
	rejects = ltc->sync_observation_rejects;
	fifo_bytes = ltc->sync_fifo_bytes;
	lock_streak = ltc->sync_lock_streak;
	epoch_ns = ltc->sync_epoch_ns;
	spin_unlock_irqrestore(&ltc->state_lock, flags);
	seq_printf(s, "running: %u\n", st.running);
	seq_printf(s, "mode: %s\n", rk_ltc_mode_name(st.mode));
	seq_printf(s, "rate: %u/%u\n", st.fps_num, st.fps_den);
	seq_printf(s, "drop_frame: %u\n", st.drop_frame);
	seq_printf(s, "high_frame_rate: %u\n", st.high_frame_rate);
	seq_printf(s, "sample_rate: %u\n", st.sample_rate);
	seq_printf(s, "actual_sample_rate: %u\n", st.actual_sample_rate);
	seq_printf(s, "divider: %u\n", st.divider);
	seq_printf(s, "dma_fill_bytes: %u\n", st.dma_fill_bytes);
	seq_printf(s, "timecode: %02u:%02u:%02u:%02u\n",
		   st.timecode.hours, st.timecode.minutes,
		   st.timecode.seconds, st.timecode.frames);
	seq_printf(s, "codewords_generated: %llu\n", st.codewords);
	seq_printf(s, "samples_generated: %llu\n", st.samples);
	seq_printf(s, "dma_periods: %llu\n", st.dma_periods);
	seq_printf(s, "underruns: %llu\n", st.underruns);
	seq_printf(s, "irq_count: %llu\n", st.irq_count);
	seq_printf(s, "decoder_locks: %llu\n", st.decoder_locks);
	seq_printf(s, "decoder_errors: %llu\n", st.decoder_errors);
	seq_printf(s, "sync_source: %s\n",
		   rk_ltc_sync_source_name(st.sync_source));
	seq_printf(s, "sync_state: %s\n",
		   rk_ltc_sync_state_name(st.sync_state));
	seq_printf(s, "sync_phase_error_ns: %lld\n",
		   st.sync_phase_error_ns);
	seq_printf(s, "sync_correction_ppm: %d\n",
		   st.sync_correction_ppm);
	seq_printf(s, "sync_correction_ppb: %d\n",
		   st.sync_correction_ppb);
	seq_printf(s, "sync_integral_ppb: %d\n",
		   st.sync_integral_ppb);
	seq_printf(s, "sync_max_slew_ppm: %d\n",
		   st.sync_max_slew_ppm);
	seq_printf(s, "sync_offset_ns: %lld\n",
		   st.sync_offset_ns);
	seq_printf(s, "sync_updates: %llu\n", st.sync_updates);
	seq_printf(s, "sync_locked_updates: %llu\n",
		   st.sync_locked_updates);
	seq_printf(s, "sync_ptp_index: %d\n", st.sync_ptp_index);
	seq_printf(s, "sync_jump_count: %llu\n", st.sync_jump_count);
	seq_printf(s, "sync_last_jump_error_ns: %lld\n",
		   st.sync_last_jump_error_ns);
	seq_printf(s, "sync_observation_valid: %u\n", valid);
	seq_printf(s, "sync_reference_ns: %lld\n", reference_ns);
	seq_printf(s, "sync_output_samples: %llu\n", output_samples);
	seq_printf(s, "sync_observation_span_ns: %llu\n", span_ns);
	seq_printf(s, "sync_observation_rejects: %llu\n", rejects);
	seq_printf(s, "sync_fifo_bytes: %u\n", fifo_bytes);
	seq_printf(s, "sync_lock_streak: %u\n", lock_streak);
	seq_printf(s, "sync_epoch_ns: %lld\n", epoch_ns);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rk_ltc_debugfs);

static int rk_ltc_probe(struct platform_device *pdev)
{
	struct rk_ltc *ltc;
	struct resource *res;
	struct rk_ltc_config cfg = {
		.fps_num = 30,
		.fps_den = 1,
		.mode = RK_LTC_MODE_IDLE_LOW,
		.sample_rate = RK_LTC_DEFAULT_SAMPLE_RATE,
	};
	struct rk_ltc_sync_config sync_cfg = {
		.source = RK_LTC_SYNC_SOURCE_NONE,
		.flags = RK_LTC_SYNC_FLAG_JAM_ON_START,
		.max_slew_ppm = RK_LTC_DEFAULT_MAX_SLEW_PPM,
		.lock_threshold_ms = RK_LTC_DEFAULT_LOCK_THRESHOLD_MS,
		.ptp_index = -1,
		.pll_kp_ppb_per_ms = RK_LTC_DEFAULT_PLL_KP_PPB_PER_MS,
		.pll_ki_ppb_per_ms_s = RK_LTC_DEFAULT_PLL_KI_PPB_PER_MS_S,
	};
	unsigned long flags;
	int ret;

	ltc = devm_kzalloc(&pdev->dev, sizeof(*ltc), GFP_KERNEL);
	if (!ltc)
		return -ENOMEM;

	ltc->dev = &pdev->dev;
	mutex_init(&ltc->lock);
	spin_lock_init(&ltc->state_lock);
	platform_set_drvdata(pdev, ltc);
	ltc->sync_cfg = sync_cfg;
	ltc->sync_state = RK_LTC_SYNC_STATE_OFF;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	ltc->regs_phys = res->start;
	ltc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ltc->regs))
		return PTR_ERR(ltc->regs);

	ltc->irq = platform_get_irq(pdev, 0);
	if (ltc->irq < 0)
		return ltc->irq;

	ltc->spiclk = devm_clk_get(&pdev->dev, "spiclk");
	if (IS_ERR(ltc->spiclk))
		return PTR_ERR(ltc->spiclk);

	ltc->apb_pclk = devm_clk_get(&pdev->dev, "apb_pclk");
	if (IS_ERR(ltc->apb_pclk))
		return PTR_ERR(ltc->apb_pclk);

	of_property_read_u32(pdev->dev.of_node, "rockchip,ltc-sample-rate",
			     &cfg.sample_rate);

	ret = clk_set_rate(ltc->spiclk, RK_LTC_DEFAULT_SPI_PARENT_HZ);
	if (ret)
		dev_warn(&pdev->dev, "failed to set SPI clock to 198 MHz: %d\n", ret);

	ret = clk_prepare_enable(ltc->apb_pclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ltc->spiclk);
	if (ret)
		goto err_disable_apb;

	ret = devm_request_irq(&pdev->dev, ltc->irq, rk_ltc_irq, 0,
			       dev_name(&pdev->dev), ltc);
	if (ret)
		goto err_disable_clks;

	ltc->tx_chan = dma_request_chan(&pdev->dev, "tx");
	if (IS_ERR(ltc->tx_chan)) {
		ret = PTR_ERR(ltc->tx_chan);
		goto err_disable_clks;
	}

	ltc->ring_bytes = RK_LTC_RING_BYTES;
	ltc->period_bytes = RK_LTC_PERIOD_BYTES;
	ltc->periods = ltc->ring_bytes / ltc->period_bytes;
	ltc->ring = dma_alloc_coherent(&pdev->dev, ltc->ring_bytes,
				       &ltc->dma_handle, GFP_KERNEL);
	if (!ltc->ring) {
		ret = -ENOMEM;
		goto err_release_dma;
	}

	ltc->period_tc = devm_kcalloc(&pdev->dev, ltc->periods,
				      sizeof(*ltc->period_tc), GFP_KERNEL);
	if (!ltc->period_tc) {
		ret = -ENOMEM;
		goto err_free_ring;
	}

	ret = rk_ltc_apply_config_locked(ltc, &cfg);
	if (ret)
		goto err_free_ring;

	spin_lock_irqsave(&ltc->state_lock, flags);
	rk_ltc_gen_reset_locked(ltc);
	spin_unlock_irqrestore(&ltc->state_lock, flags);

	ltc->miscdev.minor = MISC_DYNAMIC_MINOR;
	ltc->miscdev.name = "ltc0";
	ltc->miscdev.fops = &rk_ltc_fops;
	ltc->miscdev.parent = &pdev->dev;

	ret = misc_register(&ltc->miscdev);
	if (ret)
		goto err_free_ring;

	ret = sysfs_create_group(&pdev->dev.kobj, &rk_ltc_attr_group);
	if (ret)
		goto err_misc;

	ltc->debugfs_dir = debugfs_create_dir("rk_ltc", NULL);
	debugfs_create_file("ltc0_status", 0444, ltc->debugfs_dir, ltc,
			    &rk_ltc_debugfs_fops);

	dev_info(&pdev->dev, "registered /dev/ltc0, sample %u Hz\n",
		 ltc->sample_rate);

	return 0;

err_misc:
	misc_deregister(&ltc->miscdev);
err_free_ring:
	dma_free_coherent(&pdev->dev, ltc->ring_bytes, ltc->ring,
			  ltc->dma_handle);
err_release_dma:
	dma_release_channel(ltc->tx_chan);
err_disable_clks:
	clk_disable_unprepare(ltc->spiclk);
err_disable_apb:
	clk_disable_unprepare(ltc->apb_pclk);

	return ret;
}

static int rk_ltc_remove(struct platform_device *pdev)
{
	struct rk_ltc *ltc = platform_get_drvdata(pdev);

	mutex_lock(&ltc->lock);
	rk_ltc_stop_locked(ltc);
	mutex_unlock(&ltc->lock);

	debugfs_remove_recursive(ltc->debugfs_dir);
	sysfs_remove_group(&pdev->dev.kobj, &rk_ltc_attr_group);
	misc_deregister(&ltc->miscdev);
	dma_free_coherent(&pdev->dev, ltc->ring_bytes, ltc->ring,
			  ltc->dma_handle);
	dma_release_channel(ltc->tx_chan);
	clk_disable_unprepare(ltc->spiclk);
	clk_disable_unprepare(ltc->apb_pclk);

	return 0;
}

static const struct of_device_id rk_ltc_of_match[] = {
	{ .compatible = "rockchip,rk3588-ltc-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, rk_ltc_of_match);

static struct platform_driver rk_ltc_driver = {
	.probe = rk_ltc_probe,
	.remove = rk_ltc_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rk_ltc_of_match,
	},
};
module_platform_driver(rk_ltc_driver);

MODULE_AUTHOR("Codex");
MODULE_DESCRIPTION("Rockchip RK3588 SPI MOSI LTC generator");
MODULE_LICENSE("GPL");
