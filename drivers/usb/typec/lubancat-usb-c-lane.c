// SPDX-License-Identifier: GPL-2.0
/*
 * LubanCat-5 USB-C SuperSpeed lane orientation manager
 *
 * A standards-valid Type-C orientation is authoritative. A local FUSB302 CC
 * sample or USB2 activity may start a search, but only SuperSpeed enumeration
 * confirms a guessed lane mapping.
 */

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-rockchip-inno-usb2.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/usb.h>
#include <linux/usb/fusb302.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue.h>

#define LC_CC_OPEN_INTERVAL_MS	20
#define LC_CC_PRESENT_INTERVAL_MS	50
#define LC_SCAN_INTERVAL_MS	50
#define LC_LINK_CONFIRM_MS	350
#define LC_SCAN_TIMEOUT_MS	5000
#define LC_ROLE_TIMEOUT_MS	1000
#define LC_DETACH_GUARD_MS	250

enum lc_state {
	LC_IDLE,
	LC_PRESET,
	LC_REARM,
	LC_WAIT,
	LC_SCAN,
	LC_RESTORE,
	LC_APPLY,
};

struct lc_topology {
	bool ss_root;
	bool direct;
	bool configured_usb2;
	bool configured_ss;
};

struct lc_snapshot {
	struct device *controller;
	struct lc_topology topology;
};

struct lc_op {
	enum lc_state state;
	u64 epoch;
	u64 event_seq;
	enum typec_orientation target;
};

struct lc_lane {
	struct device *dev;
	struct device *controller;
	struct device *cc_dev;
	struct phy *usb2_phy;
	struct typec_switch *sw;
	struct typec_switch_dev *input_sw;
	struct mutex lock; /* protects state below */
	struct mutex switch_lock; /* serializes downstream calls */
	struct mutex activity_lock; /* serializes USB2 detector control */
	struct delayed_work work;
	struct delayed_work cc_work;
	struct notifier_block usb_nb;
	struct notifier_block usb2_nb;

	enum lc_state state;
	enum typec_orientation orientation, last_good, input_orientation;
	enum typec_orientation scan_hint, cc_orientation;
	struct lc_topology topology;
	unsigned long deadline, scan_deadline, guard_deadline, role_deadline;
	u64 epoch, event_seq, guard_seq;
	bool current_valid, attach_pending, activity_armed;
	bool startup_scan;
	bool guard_ready, guard_watch, guard_fresh;
	bool cc_faulted, scan_hint_pending, host_unavailable, root_returned;
	bool suspended, teardown;
};

static bool lc_bus_matches(struct device *controller, const struct usb_bus *bus)
{
	struct device *dev;

	if (!bus || !bus->controller)
		return false;
	for (dev = bus->controller; dev; dev = dev->parent)
		if (dev == controller)
			return true;
	return false;
}

static bool lc_root(const struct usb_device *udev)
{
	return udev && udev->bus &&
	       (udev == udev->bus->root_hub || !udev->parent || !udev->level);
}

static bool lc_direct(const struct usb_device *udev)
{
	return udev && udev->bus && udev->parent == udev->bus->root_hub &&
	       udev->level == 1 && udev->portnum == 1;
}

static int lc_snapshot_dev(struct usb_device *udev, void *data)
{
	struct lc_snapshot *s = data;
	enum usb_device_state state;

	if (!udev || !lc_bus_matches(s->controller, udev->bus))
		return 0;
	state = READ_ONCE(udev->state);
	if (lc_root(udev)) {
		if (state != USB_STATE_NOTATTACHED &&
		    READ_ONCE(udev->speed) >= USB_SPEED_SUPER)
			s->topology.ss_root = true;
		return 0;
	}
	if (!lc_direct(udev))
		return 0;
	if (state == USB_STATE_NOTATTACHED)
		return 0;
	s->topology.direct = true;
	if (READ_ONCE(udev->actconfig) &&
	    (state == USB_STATE_CONFIGURED || state == USB_STATE_SUSPENDED)) {
		if (READ_ONCE(udev->speed) >= USB_SPEED_SUPER)
			s->topology.configured_ss = true;
		else
			s->topology.configured_usb2 = true;
	}
	return 0;
}

/* State helpers require lane->lock. */
static bool lc_has_configured_child(const struct lc_topology *topology)
{
	return topology->configured_usb2 || topology->configured_ss;
}

static bool lc_phy_is_host_mode(enum phy_mode mode)
{
	return mode >= PHY_MODE_USB_HOST && mode <= PHY_MODE_USB_HOST_SS;
}

static bool lc_host_mode_visible(const struct lc_lane *lane, enum phy_mode mode)
{
	/* Board policy may change DWC3 role without updating the USB2 PHY role. */
	return lc_phy_is_host_mode(mode) || lane->topology.ss_root;
}

static bool lc_host_policy_enabled(const struct lc_lane *lane)
{
	enum phy_mode mode = phy_get_mode(lane->usb2_phy);

	if (lane->host_unavailable)
		return false;
	/* Preserve one bounded preset for a device attached before DWC3 probes. */
	return lc_host_mode_visible(lane, mode) ||
	       (mode == PHY_MODE_INVALID && lane->startup_scan &&
		(lane->scan_hint_pending || time_before(jiffies, lane->deadline)));
}

static bool lc_can_bootstrap_cc(const struct lc_lane *lane, enum phy_mode mode)
{
	return mode == PHY_MODE_INVALID && lane->startup_scan &&
	       lane->cc_orientation == TYPEC_ORIENTATION_NONE &&
	       time_before(jiffies, lane->deadline);
}

static bool lc_role_transition_pending(const struct lc_lane *lane,
				       unsigned long now)
{
	bool host_mode;

	if (!lane->host_unavailable || !lane->role_deadline ||
	    time_after_eq(now, lane->role_deadline))
		return false;
	host_mode = lc_phy_is_host_mode(phy_get_mode(lane->usb2_phy));

	return (lane->root_returned && !host_mode) ||
	       (!lane->root_returned && host_mode);
}

static bool lc_can_execute(const struct lc_lane *lane, enum lc_state state,
			   unsigned long now)
{
	enum phy_mode mode = phy_get_mode(lane->usb2_phy);

	return (state == LC_RESTORE &&
		!lc_role_transition_pending(lane, now)) ||
	       (!lane->host_unavailable &&
		(lc_host_mode_visible(lane, mode) ||
		 (state == LC_PRESET && mode == PHY_MODE_INVALID &&
		  lane->startup_scan)));
}

static bool lc_should_poll_cc(const struct lc_lane *lane)
{
	enum phy_mode mode = phy_get_mode(lane->usb2_phy);

	return !lane->host_unavailable &&
	       (lc_host_mode_visible(lane, mode) ||
		lc_can_bootstrap_cc(lane, mode)) &&
	       lane->cc_dev && !lane->cc_faulted &&
	       !lane->suspended && !lane->teardown &&
	       lane->input_orientation == TYPEC_ORIENTATION_NONE &&
	       !lc_has_configured_child(&lane->topology);
}

static void lc_queue(struct lc_lane *lane, unsigned long delay)
{
	if (!lane->suspended && !lane->teardown)
		mod_delayed_work(system_wq, &lane->work, delay);
}

static void lc_queue_cc(struct lc_lane *lane, unsigned long delay)
{
	if (lc_should_poll_cc(lane))
		mod_delayed_work(system_freezable_wq, &lane->cc_work, delay);
}

static void lc_update_topology(struct lc_lane *lane,
			       const struct lc_topology *topology)
{
	bool restart_cc = lc_has_configured_child(&lane->topology) &&
			  !lc_has_configured_child(topology);

	lane->topology = *topology;
	if (restart_cc)
		lc_queue_cc(lane, 0);
}

static void lc_invalidate(struct lc_lane *lane, enum lc_state state)
{
	lane->state = state;
	lane->epoch++;
}

static void lc_clear_scan(struct lc_lane *lane)
{
	lane->startup_scan = false;
	lane->scan_deadline = 0;
	lane->attach_pending = false;
	lane->scan_hint_pending = false;
}

static void lc_clear_guard(struct lc_lane *lane)
{
	lane->guard_deadline = 0;
	lane->guard_ready = false;
	lane->guard_watch = false;
	lane->guard_fresh = false;
	lane->guard_seq++;
}

static void lc_stop_host_policy(struct lc_lane *lane)
{
	lane->activity_armed = false;
	lane->cc_orientation = TYPEC_ORIENTATION_NONE;
	memset(&lane->topology, 0, sizeof(lane->topology));
	lc_clear_scan(lane);
	lc_clear_guard(lane);
}

static bool lc_guard_active(const struct lc_lane *lane)
{
	return lane->guard_deadline;
}

static u64 lc_start_guard(struct lc_lane *lane, bool detector_cleared,
			  bool watch_activity)
{
	if (!lc_guard_active(lane))
		lane->guard_fresh = false;
	lane->guard_deadline = jiffies +
			       msecs_to_jiffies(LC_DETACH_GUARD_MS);
	lane->guard_seq++;
	lane->guard_ready = detector_cleared;
	/* A later root/bus teardown supersedes direct-detach evidence. */
	lane->guard_watch = watch_activity;
	lane->activity_armed = false;
	lc_queue(lane, detector_cleared ? 0 :
		 msecs_to_jiffies(LC_DETACH_GUARD_MS));

	return lane->guard_seq;
}

static bool lc_needs_restore(struct lc_lane *lane)
{
	return lane->input_orientation == TYPEC_ORIENTATION_NONE &&
	       (!lane->current_valid ||
		(!lane->host_unavailable &&
		 lane->orientation != lane->last_good));
}

static bool lc_has_scan_intent(struct lc_lane *lane)
{
	return lane->scan_deadline || lane->attach_pending;
}

static enum typec_orientation lc_restore_target(struct lc_lane *lane)
{
	return lane->input_orientation != TYPEC_ORIENTATION_NONE ?
		lane->input_orientation : lane->last_good;
}

static void lc_restore(struct lc_lane *lane)
{
	lc_clear_scan(lane);
	lc_invalidate(lane, LC_RESTORE);
	lc_queue(lane, 0);
}

static void lc_defer_scan(struct lc_lane *lane, unsigned int delay_ms)
{
	unsigned long now = jiffies;
	unsigned long delay;

	if (!lane->scan_deadline ||
	    time_after_eq(now, lane->scan_deadline)) {
		lc_restore(lane);
		return;
	}
	delay = min(msecs_to_jiffies(delay_ms),
		    lane->scan_deadline - now);
	lane->deadline = now + delay;
	lc_invalidate(lane, LC_WAIT);
	lc_queue(lane, delay);
}

static void lc_resume_scan(struct lc_lane *lane)
{
	if (!lane->scan_deadline ||
	    time_after_eq(jiffies, lane->scan_deadline)) {
		lc_restore(lane);
		return;
	}

	lane->attach_pending = false;
	if (lane->scan_hint_pending) {
		/* Re-apply equal hints too: NONE may have disabled BVALID. */
		lc_invalidate(lane, LC_REARM);
		lc_queue(lane, 0);
		return;
	}
	if (lane->current_valid) {
		lc_defer_scan(lane, LC_SCAN_INTERVAL_MS);
	} else {
		lc_invalidate(lane, LC_REARM);
		lc_queue(lane, 0);
	}
}

static void lc_begin_hot_scan(struct lc_lane *lane)
{
	unsigned long now = jiffies;
	bool fresh = lc_guard_active(lane) && lane->guard_watch &&
		     !lane->guard_fresh;

	if (lane->host_unavailable ||
	    !lc_host_mode_visible(lane, phy_get_mode(lane->usb2_phy)) ||
	    lane->input_orientation != TYPEC_ORIENTATION_NONE)
		return;
	if (fresh)
		lane->guard_fresh = true;
	if (fresh && lane->guard_ready)
		lane->guard_deadline = jiffies;
	if (lane->scan_deadline &&
	    time_after_eq(now, lane->scan_deadline)) {
		lc_clear_scan(lane);
	}
	/* A normal hot generation already owns the only physical activity. */
	if (lane->scan_deadline && !lane->startup_scan &&
	    !lc_guard_active(lane) && lane->state != LC_APPLY &&
	    !lane->scan_hint_pending)
		return;
	if (fresh || lane->startup_scan || !lane->scan_deadline)
		lane->scan_deadline = now +
				      msecs_to_jiffies(LC_SCAN_TIMEOUT_MS);
	lane->startup_scan = false;
	lane->attach_pending = true;

	if (lc_guard_active(lane) || lane->state == LC_APPLY ||
	    !lane->topology.ss_root) {
		lc_queue(lane, 0);
		return;
	}
	lc_resume_scan(lane);
}

static void lc_cc_work(struct work_struct *work)
{
	struct lc_lane *lane = container_of(work, struct lc_lane, cc_work.work);
	enum typec_orientation orientation = TYPEC_ORIENTATION_NONE;
	enum phy_mode mode;
	unsigned int interval = LC_CC_OPEN_INTERVAL_MS;
	bool host_root;
	u64 guard_seq;
	int ret;

	mutex_lock(&lane->lock);
	if (!lc_host_policy_enabled(lane)) {
		lc_queue(lane, 0);
		goto out;
	}
	if (!lc_should_poll_cc(lane))
		goto out;
	if (lane->cc_orientation != TYPEC_ORIENTATION_NONE)
		interval = LC_CC_PRESENT_INTERVAL_MS;
	host_root = lane->topology.ss_root;
	guard_seq = lane->guard_seq;
	mutex_unlock(&lane->lock);

	mode = phy_get_mode(lane->usb2_phy);
	if (!lc_phy_is_host_mode(mode) && !host_root &&
	    mode != PHY_MODE_INVALID) {
		mutex_lock(&lane->lock);
		lc_queue(lane, 0);
		goto out;
	}
	ret = fusb302_sink_cc_sample(lane->cc_dev, &orientation);

	mutex_lock(&lane->lock);
	if (lane->suspended || lane->teardown || lane->cc_faulted)
		goto out;

	if (ret == -EUCLEAN) {
		lane->cc_faulted = true;
		dev_err(lane->dev, "FUSB302 CC sampling restore failed\n");
		goto out;
	}
	if (ret == -EOPNOTSUPP) {
		lane->cc_faulted = true;
		dev_err(lane->dev,
			"FUSB302 CC sampling requires a fixed Sink port\n");
		goto out;
	}
	if (!lc_should_poll_cc(lane)) {
		if (!lc_host_policy_enabled(lane))
			lc_queue(lane, 0);
		goto out;
	}
	if (guard_seq != lane->guard_seq)
		goto requeue;
	if (ret) {
		if (ret == -EPERM)
			interval = LC_CC_PRESENT_INTERVAL_MS;
		if (ret != -EAGAIN && ret != -ENODEV && ret != -EPERM)
			dev_warn_ratelimited(lane->dev,
					     "FUSB302 CC sampling failed: %d\n", ret);
		goto requeue;
	}

	if (orientation == TYPEC_ORIENTATION_NONE) {
		lane->cc_orientation = TYPEC_ORIENTATION_NONE;
		interval = LC_CC_OPEN_INTERVAL_MS;
		goto requeue;
	}
	if (orientation != TYPEC_ORIENTATION_NORMAL &&
	    orientation != TYPEC_ORIENTATION_REVERSE) {
		dev_warn_ratelimited(lane->dev,
				     "FUSB302 returned invalid CC orientation: %d\n",
				     orientation);
		goto requeue;
	}

	interval = LC_CC_PRESENT_INTERVAL_MS;
	if (orientation == lane->cc_orientation)
		goto requeue;
	lane->cc_orientation = orientation;
	lane->event_seq++;
	/* New CC attach evidence supersedes a physical-detach guard. */
	if (lc_guard_active(lane) && lane->guard_watch) {
		lane->guard_fresh = true;
		if (lane->guard_ready)
			lc_clear_guard(lane);
	}
	if (lane->scan_deadline &&
	    time_after_eq(jiffies, lane->scan_deadline))
		lc_clear_scan(lane);
	lane->scan_hint = orientation;
	lane->scan_hint_pending = true;
	if (lane->topology.ss_root) {
		lc_begin_hot_scan(lane);
	} else {
		if (!lane->scan_deadline)
			lane->startup_scan = true;
		lane->attach_pending = true;
		lc_queue(lane, 0);
	}

requeue:
	lc_queue_cc(lane, msecs_to_jiffies(interval));
	if (!lc_host_policy_enabled(lane))
		lc_queue(lane, 0);
out:
	mutex_unlock(&lane->lock);
}

static bool lc_complete_hot_scan(struct lc_lane *lane, int ret)
{
	if (!lane->attach_pending || lane->startup_scan ||
	    !lane->scan_deadline)
		return false;

	lane->attach_pending = false;
	if (!ret || ret == -EAGAIN)
		lc_resume_scan(lane);
	else if (ret == -EBUSY)
		lc_defer_scan(lane, LC_LINK_CONFIRM_MS);
	else
		lc_restore(lane);

	return true;
}

static bool lc_can_keep_activity(const struct lc_lane *lane)
{
	return !lane->host_unavailable &&
	       lc_host_mode_visible(lane, phy_get_mode(lane->usb2_phy)) &&
	       lane->current_valid &&
	       !lane->suspended && !lane->teardown &&
	       (!lane->topology.direct || lc_guard_active(lane)) &&
	       !(lane->attach_pending && lane->scan_deadline) &&
	       lane->input_orientation == TYPEC_ORIENTATION_NONE;
}

static bool lc_mark_activity_arm(struct lc_lane *lane)
{
	if (lane->activity_armed ||
	    (!lc_guard_active(lane) && lane->state != LC_IDLE) ||
	    (lc_guard_active(lane) &&
	     (!lane->guard_ready || !lane->guard_watch || lane->guard_fresh)) ||
	    !lc_can_keep_activity(lane))
		return false;

	lane->activity_armed = true;
	return true;
}

static void lc_mark_activity_disarm(struct lc_lane *lane)
{
	lane->activity_armed = false;
}

static void __lc_disarm_activity(struct lc_lane *lane)
{
	int ret;

	ret = rockchip_usb2phy_disarm_host_ls_activity(lane->usb2_phy);
	if (ret && ret != -EAGAIN && ret != -ENOENT)
		dev_warn_ratelimited(lane->dev,
				     "failed to disarm USB2 activity: %d\n", ret);
}

static void lc_disarm_activity(struct lc_lane *lane)
{
	bool disarm;

	mutex_lock(&lane->activity_lock);
	mutex_lock(&lane->lock);
	disarm = !lane->activity_armed;
	mutex_unlock(&lane->lock);
	if (disarm)
		__lc_disarm_activity(lane);
	mutex_unlock(&lane->activity_lock);
}

static void lc_arm_activity(struct lc_lane *lane)
{
	bool disarm = false, retry = false;
	int ret;

	mutex_lock(&lane->activity_lock);
	mutex_lock(&lane->lock);
	if (!lc_can_keep_activity(lane)) {
		lane->activity_armed = false;
		mutex_unlock(&lane->lock);
		__lc_disarm_activity(lane);
		mutex_unlock(&lane->activity_lock);
		return;
	}
	mutex_unlock(&lane->lock);
	ret = rockchip_usb2phy_arm_host_ls_activity(lane->usb2_phy);
	mutex_lock(&lane->lock);
	if (!ret) {
		/* arm() may synchronously deliver a pending activity callback. */
		disarm = !lane->activity_armed || !lc_can_keep_activity(lane);
		if (disarm)
			lane->activity_armed = false;
	} else if (lane->activity_armed) {
		lane->activity_armed = false;
		retry = ret == -EAGAIN && !lane->suspended && !lane->teardown;
		if (retry)
			lc_queue(lane, msecs_to_jiffies(LC_SCAN_INTERVAL_MS));
	}
	mutex_unlock(&lane->lock);
	if (disarm)
		__lc_disarm_activity(lane);
	if (ret && !retry && ret != -ENOENT && ret != -ESHUTDOWN)
		dev_warn_ratelimited(lane->dev,
				     "failed to arm USB2 activity: %d\n", ret);
	mutex_unlock(&lane->activity_lock);
}

static void lc_finish_guard(struct lc_lane *lane, u64 guard_seq)
{
	bool arm = false;

	mutex_lock(&lane->lock);
	if (!lane->teardown && !lane->suspended &&
	    lane->guard_seq == guard_seq && lc_guard_active(lane)) {
		lane->guard_ready = true;
		if (lane->guard_watch && lane->guard_fresh)
			lane->guard_deadline = jiffies;
		arm = lc_mark_activity_arm(lane);
		lc_queue(lane, 0);
	}
	mutex_unlock(&lane->lock);
	if (arm)
		lc_arm_activity(lane);
}

static void lc_prepare_apply(struct lc_lane *lane, struct lc_op *op)
{
	if ((lane->state == LC_REARM || lane->state == LC_SCAN) &&
	    (!lane->scan_deadline ||
	     time_after_eq(jiffies, lane->scan_deadline))) {
		lc_restore(lane);
		return;
	}

	if (lane->state == LC_PRESET) {
		op->state = LC_PRESET;
		op->target = lane->scan_hint;
	} else if (lane->state == LC_REARM) {
		op->state = LC_REARM;
		op->target = lane->scan_hint_pending ? lane->scan_hint :
			     (lane->current_valid ? lane->orientation :
						    lane->last_good);
	} else if (lane->state == LC_SCAN) {
		op->state = LC_SCAN;
		op->target = lane->scan_hint_pending ? lane->scan_hint :
			(lane->current_valid ?
			(lane->orientation == TYPEC_ORIENTATION_NORMAL ?
				 TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL) :
			lane->last_good);
	} else if (lane->state == LC_RESTORE) {
		op->state = LC_RESTORE;
		op->target = lc_restore_target(lane);
	} else {
		return;
	}
	lc_invalidate(lane, LC_APPLY);
	op->epoch = lane->epoch;
	op->event_seq = lane->event_seq;
}

static void lc_execute(struct lc_lane *lane, struct lc_op *op)
{
	bool arm, events_changed, valid, executed = false;
	unsigned long now;
	int ret = 0;

	mutex_lock(&lane->switch_lock);
	mutex_lock(&lane->lock);
	now = jiffies;
	valid = lc_can_execute(lane, op->state, now) &&
		!lane->teardown && !lane->suspended &&
		!lc_guard_active(lane) &&
		lane->epoch == op->epoch &&
		lane->event_seq == op->event_seq;
	mutex_unlock(&lane->lock);
	if (valid) {
		executed = true;
		ret = typec_switch_set(lane->sw, op->target);
	}
	mutex_lock(&lane->lock);
	now = jiffies;
	if (executed && !ret) {
		if (op->target == TYPEC_ORIENTATION_NONE) {
			lane->current_valid = false;
		} else {
			lane->orientation = op->target;
			lane->current_valid = true;
			if (lane->scan_hint_pending && op->target == lane->scan_hint)
				lane->scan_hint_pending = false;
		}
	}
	valid = executed && lc_can_execute(lane, op->state, now) &&
		!lane->teardown && !lane->suspended &&
		lane->epoch == op->epoch;
	if (!valid) {
		if (!executed && op->state == LC_RESTORE &&
		    lc_role_transition_pending(lane, now)) {
			lc_invalidate(lane, LC_RESTORE);
			lc_queue(lane,
				 min(msecs_to_jiffies(LC_SCAN_INTERVAL_MS),
				     lane->role_deadline - now));
			goto complete;
		}
		if (lane->epoch == op->epoch && lane->state == LC_APPLY) {
			if (executed ||
			    (lane->attach_pending && lane->scan_deadline &&
			     !lane->startup_scan))
				lc_invalidate(lane, LC_IDLE);
			else
				lc_invalidate(lane, op->state);
		}
		lc_queue(lane, 0);
		goto complete;
	}
	events_changed = lane->event_seq != op->event_seq;

	/* The validated RK3588 provider leaves its lane mux unchanged for NONE. */
	if (ret && ret != -EBUSY && ret != -EAGAIN)
		dev_warn_ratelimited(lane->dev, "lane switch failed: %d\n", ret);

	if (lc_complete_hot_scan(lane, ret))
		goto reconciled;

	switch (op->state) {
	case LC_PRESET:
		if (ret == -EAGAIN && time_before(jiffies, lane->deadline)) {
			lc_invalidate(lane, LC_PRESET);
			lc_queue(lane, msecs_to_jiffies(LC_SCAN_INTERVAL_MS));
		} else {
			if (ret && lane->scan_hint_pending &&
			    lane->scan_hint == op->target)
				lane->scan_hint_pending = false;
			lc_invalidate(lane, LC_IDLE);
		}
		break;
	case LC_REARM:
	case LC_SCAN:
		if (!ret) {
			lc_defer_scan(lane, LC_SCAN_INTERVAL_MS);
		} else if (ret == -EBUSY) {
			lc_defer_scan(lane, LC_LINK_CONFIRM_MS);
		} else if (ret == -EAGAIN) {
			if (lane->scan_deadline &&
			    time_before(now, lane->scan_deadline)) {
				lc_invalidate(lane, op->state);
				lc_queue(lane,
					 msecs_to_jiffies(LC_SCAN_INTERVAL_MS));
			} else {
				lc_restore(lane);
			}
		} else {
			lc_restore(lane);
		}
		break;
	case LC_RESTORE:
		if (ret == -EAGAIN) {
			lc_invalidate(lane, LC_RESTORE);
			lc_queue(lane, msecs_to_jiffies(LC_SCAN_INTERVAL_MS));
		} else {
			lc_invalidate(lane, LC_IDLE);
			if (!ret)
				lc_queue(lane, 0);
		}
		break;
	case LC_IDLE:
	case LC_WAIT:
	case LC_APPLY:
		break;
	}

reconciled:
	if (events_changed ||
	    (lane->attach_pending && lane->state == LC_IDLE))
		lc_queue(lane, 0);
complete:
	arm = lc_mark_activity_arm(lane);
	mutex_unlock(&lane->lock);
	mutex_unlock(&lane->switch_lock);
	if (arm)
		lc_arm_activity(lane);
}

static int lc_input_set(struct typec_switch_dev *sw,
			enum typec_orientation orientation)
{
	struct lc_lane *lane = typec_switch_get_drvdata(sw);
	bool resume_scan, state_changed;
	u64 apply_epoch, event_seq;
	int ret;

	if (orientation != TYPEC_ORIENTATION_NONE &&
	    orientation != TYPEC_ORIENTATION_NORMAL &&
	    orientation != TYPEC_ORIENTATION_REVERSE)
		return -EINVAL;
	mutex_lock(&lane->switch_lock);
	mutex_lock(&lane->lock);
	if (lane->teardown) {
		ret = -ESHUTDOWN;
		goto unlock;
	}
	if (lane->suspended) {
		ret = -EAGAIN;
		goto unlock;
	}
	resume_scan = lane->input_orientation == TYPEC_ORIENTATION_NONE &&
			      lc_has_scan_intent(lane) &&
			      !lc_has_configured_child(&lane->topology);
	event_seq = lane->event_seq;
	lane->activity_armed = false;
	lc_invalidate(lane, LC_APPLY);
	apply_epoch = lane->epoch;
	mutex_unlock(&lane->lock);

	lc_disarm_activity(lane);
	ret = typec_switch_set(lane->sw, orientation);
	mutex_lock(&lane->lock);
	state_changed = lane->epoch != apply_epoch;
	if (!ret) {
		lane->input_orientation = orientation;
		if (orientation != TYPEC_ORIENTATION_NONE) {
			lane->orientation = orientation;
			lane->current_valid = true;
			lc_clear_scan(lane);
		} else {
			lane->cc_orientation = TYPEC_ORIENTATION_NONE;
			lane->current_valid = false;
		}
	}
	if (!lane->suspended && !lane->teardown) {
		if (!ret && orientation != TYPEC_ORIENTATION_NONE) {
			lc_invalidate(lane, LC_IDLE);
		} else if (!ret && orientation == TYPEC_ORIENTATION_NONE) {
			if (state_changed) {
				lc_queue(lane, 0);
			} else {
				lc_invalidate(lane, LC_IDLE);
				lc_start_guard(lane, true, true);
				lane->attach_pending |= resume_scan;
			}
		} else if (ret && state_changed) {
			lc_queue(lane, 0);
		} else if (ret && resume_scan) {
			lc_invalidate(lane, LC_IDLE);
			lc_start_guard(lane, true, true);
			lane->attach_pending = true;
		} else if (lc_needs_restore(lane)) {
			lc_restore(lane);
		} else {
			lc_clear_scan(lane);
			lc_invalidate(lane, LC_IDLE);
		}
	}
	if (lane->event_seq != event_seq)
		lc_queue(lane, 0);
	if (!ret && orientation == TYPEC_ORIENTATION_NONE)
		lc_queue_cc(lane, 0);
unlock:
	mutex_unlock(&lane->lock);
	mutex_unlock(&lane->switch_lock);
	return ret;
}

static int lc_usb_notify(struct notifier_block *nb,
			 unsigned long action, void *data)
{
	struct lc_lane *lane = container_of(nb, struct lc_lane, usb_nb);
	struct usb_device *udev;
	bool clear_activity = false, direct = false, disconnected = false;
	bool root = false;
	u64 guard_seq = 0;

	if (action == USB_BUS_ADD || action == USB_BUS_REMOVE) {
		if (!lc_bus_matches(lane->controller, data))
			return NOTIFY_DONE;
	} else if (action == USB_DEVICE_ADD || action == USB_DEVICE_REMOVE) {
		udev = data;
		if (!udev || !lc_bus_matches(lane->controller, udev->bus))
			return NOTIFY_DONE;
		root = lc_root(udev);
		direct = lc_direct(udev);
		if (!root && !direct)
			return NOTIFY_DONE;
		disconnected = action == USB_DEVICE_REMOVE &&
			       READ_ONCE(udev->state) == USB_STATE_NOTATTACHED;
	} else {
		return NOTIFY_DONE;
	}

	mutex_lock(&lane->lock);
	if (lane->teardown)
		goto out;
	lane->event_seq++;
	if ((action == USB_BUS_ADD ||
	     (action == USB_DEVICE_ADD && root)) &&
	    lane->host_unavailable) {
		lane->root_returned = true;
		lane->role_deadline = jiffies +
				      msecs_to_jiffies(LC_ROLE_TIMEOUT_MS);
	}
	if (action == USB_BUS_REMOVE ||
	    (action == USB_DEVICE_REMOVE && root)) {
		lane->host_unavailable = true;
		lane->root_returned = false;
		lane->role_deadline = jiffies +
				      msecs_to_jiffies(LC_ROLE_TIMEOUT_MS);
		lc_stop_host_policy(lane);
		lc_invalidate(lane, LC_IDLE);
		clear_activity = true;
	} else if (action == USB_DEVICE_REMOVE && direct) {
		/* REMOVE is authoritative even while usb_for_each_dev is stale. */
		lane->topology.direct = false;
		lane->topology.configured_usb2 = false;
		lane->topology.configured_ss = false;
		lane->activity_armed = false;
		if (disconnected)
			lane->cc_orientation = TYPEC_ORIENTATION_NONE;
		clear_activity = true;
		if (lane->input_orientation == TYPEC_ORIENTATION_NONE)
			guard_seq = lc_start_guard(lane, false, disconnected);
	} else if (action == USB_DEVICE_ADD && direct &&
		   READ_ONCE(udev->speed) < USB_SPEED_SUPER) {
		/* Recover an attach edge consumed before the peer became ready. */
		lane->activity_armed = false;
		clear_activity = true;
		lc_begin_hot_scan(lane);
	}
	if (clear_activity || action == USB_BUS_ADD ||
	    (action == USB_DEVICE_ADD && root))
		lc_queue_cc(lane, 0);
	lc_queue(lane, 0);
out:
	mutex_unlock(&lane->lock);
	if (clear_activity)
		lc_disarm_activity(lane);
	if (guard_seq)
		lc_finish_guard(lane, guard_seq);
	return NOTIFY_OK;
}

static int lc_usb2_notify(struct notifier_block *nb,
			  unsigned long action, void *data)
{
	struct lc_lane *lane = container_of(nb, struct lc_lane, usb2_nb);
	bool activity = action & ROCKCHIP_USB2PHY_HOST_LS_ACTIVITY;
	bool was_armed;

	(void)data;
	if (!activity)
		return NOTIFY_DONE;
	mutex_lock(&lane->lock);
	was_armed = lane->activity_armed;
	lane->activity_armed = false;
	if (lane->teardown)
		goto out;
	if (lane->host_unavailable ||
	    !lc_host_mode_visible(lane, phy_get_mode(lane->usb2_phy)))
		goto out;
	/* Publish arm before accepting either IRQ or synchronous replay. */
	if (!was_armed)
		goto out;
	if (lane->suspended) {
		if (lane->input_orientation == TYPEC_ORIENTATION_NONE) {
			lane->startup_scan = false;
			lane->attach_pending = true;
		}
		goto out;
	}
	lane->event_seq++;
	if (lane->input_orientation == TYPEC_ORIENTATION_NONE)
		lc_begin_hot_scan(lane);
	lc_queue(lane, 0);
out:
	mutex_unlock(&lane->lock);
	return NOTIFY_OK;
}

static void lc_work(struct work_struct *work)
{
	struct lc_lane *lane = container_of(work, struct lc_lane, work.work);
	struct lc_snapshot s = { .controller = lane->controller };
	struct lc_op op = { };
	unsigned long now;
	u64 seq;
	bool arm = false, detached, disarm = false, host_resumed;
	bool host_root_new, role_expired, role_pending, sync_activity = false;

	mutex_lock(&lane->lock);
	if (lane->suspended || lane->teardown) {
		mutex_unlock(&lane->lock);
		return;
	}
	seq = lane->event_seq;
	mutex_unlock(&lane->lock);
	usb_for_each_dev(&s, lc_snapshot_dev);

	mutex_lock(&lane->lock);
	if (lane->suspended || lane->teardown)
		goto out;
	if (seq != lane->event_seq) {
		lc_queue(lane, 0);
		goto out;
	}
	/* The in-flight switch call owns APPLY and handles the next snapshot. */
	if (lane->state == LC_APPLY)
		goto out;
	now = jiffies;
	role_expired = lane->role_deadline &&
		time_after_eq(now, lane->role_deadline);
	host_resumed = lane->host_unavailable && lane->root_returned &&
		s.topology.ss_root &&
		(lc_phy_is_host_mode(phy_get_mode(lane->usb2_phy)) ||
		 role_expired);
	host_root_new = !lane->host_unavailable && !lane->topology.ss_root &&
		s.topology.ss_root;
	if (host_root_new)
		lane->topology.ss_root = true;
	if (host_resumed) {
		lane->topology.ss_root = true;
		lane->host_unavailable = false;
		lane->root_returned = false;
		lane->role_deadline = 0;
	}
	if (!lc_host_policy_enabled(lane)) {
		lc_stop_host_policy(lane);
		lane->topology.ss_root = s.topology.ss_root;
		disarm = true;
		role_pending = lc_role_transition_pending(lane, now);
		if (role_pending)
			lc_queue(lane,
				 min(msecs_to_jiffies(LC_SCAN_INTERVAL_MS),
				     lane->role_deadline - now));
		else if (lane->role_deadline)
			lane->role_deadline = 0;
		if (lc_needs_restore(lane)) {
			if (role_pending)
				goto out;
			lc_invalidate(lane, LC_RESTORE);
			goto prepare;
		}
		if (lane->state != LC_IDLE)
			lc_invalidate(lane, LC_IDLE);
		goto out;
	}
	if (host_resumed || lane->role_deadline || host_root_new) {
		lane->role_deadline = 0;
		lc_queue_cc(lane, 0);
	}

	sync_activity = host_root_new &&
			lane->activity_armed &&
			lane->input_orientation == TYPEC_ORIENTATION_NONE &&
			!lc_has_configured_child(&s.topology);
	if (lc_guard_active(lane)) {
		if (!lane->guard_ready)
			goto out;
		if (time_before(now, lane->guard_deadline)) {
			lc_queue(lane, lane->guard_deadline - now);
			goto out;
		}
		if (lane->guard_watch && !s.topology.ss_root)
			goto out;
		if (lane->guard_watch && sync_activity) {
			lc_update_topology(lane, &s.topology);
			lc_queue(lane, 0);
			goto out;
		}
		detached = lane->guard_watch && lane->topology.ss_root &&
			   s.topology.ss_root && !s.topology.direct &&
			   !lane->guard_fresh;
		lc_clear_guard(lane);
		if (detached) {
			lc_update_topology(lane, &s.topology);
			if (lc_needs_restore(lane)) {
				lc_restore(lane);
				goto prepare;
			}
			lc_clear_scan(lane);
			if (lane->state != LC_IDLE)
				lc_invalidate(lane, LC_IDLE);
			goto out;
		}
	}
	lc_update_topology(lane, &s.topology);
	if (sync_activity) {
		/* Re-sample activity after the controller has initialized the PHY. */
		lc_queue(lane, 0);
		goto out;
	}

	if (!s.topology.ss_root) {
		/* Preselect a measured CC hint before the controller trains U3. */
		if (lane->startup_scan && lane->attach_pending &&
		    lane->scan_hint_pending) {
			if (lane->state != LC_PRESET) {
				lane->deadline = now +
					msecs_to_jiffies(LC_SCAN_TIMEOUT_MS);
				lc_invalidate(lane, LC_PRESET);
			}
			goto prepare;
		}
		if (lane->scan_deadline) {
			if (time_before(now, lane->scan_deadline)) {
				lc_queue(lane, lane->scan_deadline - now);
				goto out;
			}
			if (lc_needs_restore(lane)) {
				lc_restore(lane);
				goto prepare;
			}
			lc_clear_scan(lane);
			if (lane->state != LC_IDLE)
				lc_invalidate(lane, LC_IDLE);
			goto out;
		}
		if (lane->attach_pending) {
			if (lane->state != LC_IDLE)
				lc_invalidate(lane, LC_IDLE);
			goto out;
		}
		if (lc_needs_restore(lane)) {
			if (lane->state != LC_RESTORE)
				lc_invalidate(lane, LC_RESTORE);
			goto prepare;
		}
		if (lane->state != LC_IDLE)
			lc_invalidate(lane, LC_IDLE);
		goto out;
	}

	if (s.topology.configured_ss) {
		lc_mark_activity_disarm(lane);
		disarm = true;
		lc_clear_scan(lane);
		if (lane->state != LC_IDLE)
			lc_invalidate(lane, LC_IDLE);
		if (lane->current_valid)
			lane->last_good = lane->orientation;
		goto out;
	}
	/*
	 * USB2 ADD publishes its fallback deadline before this snapshot.  An
	 * established child without an active generation remains quiescent.
	 */
	if (s.topology.configured_usb2 && !lane->scan_deadline) {
		lc_mark_activity_disarm(lane);
		disarm = true;
		lc_clear_scan(lane);
		if (lc_needs_restore(lane))
			lc_invalidate(lane, LC_RESTORE);
		else if (lane->state != LC_IDLE)
			lc_invalidate(lane, LC_IDLE);
		goto prepare;
	}
	if (lane->input_orientation != TYPEC_ORIENTATION_NONE) {
		lc_mark_activity_disarm(lane);
		disarm = true;
		lc_clear_scan(lane);
		if (lane->state != LC_IDLE)
			lc_invalidate(lane, LC_IDLE);
		goto out;
	}
	if (!lane->current_valid && !lane->scan_deadline) {
		lc_restore(lane);
		goto prepare;
	}
	if (s.topology.direct) {
		lc_mark_activity_disarm(lane);
		disarm = true;
	}
	if (lane->attach_pending && lane->scan_deadline &&
	    !lane->startup_scan)
		lc_resume_scan(lane);

	switch (lane->state) {
	case LC_WAIT:
		if (time_before(now, lane->deadline)) {
			lc_queue(lane, lane->deadline - now);
			goto out;
		}
		if (lane->scan_deadline &&
		    time_before(now, lane->scan_deadline)) {
			lc_invalidate(lane, LC_SCAN);
		} else {
			lc_restore(lane);
		}
		break;
	case LC_IDLE:
		if (lane->attach_pending) {
			if (!lane->scan_deadline)
				lane->scan_deadline = now +
					msecs_to_jiffies(LC_SCAN_TIMEOUT_MS);
			lc_resume_scan(lane);
		}
		break;
	case LC_PRESET:
	case LC_REARM:
	case LC_SCAN:
	case LC_RESTORE:
	case LC_APPLY:
		break;
	}

prepare:
	lc_prepare_apply(lane, &op);
	if (op.state == LC_IDLE)
		goto out;
	mutex_unlock(&lane->lock);
	if (disarm)
		lc_disarm_activity(lane);
	lc_execute(lane, &op);
	return;
out:
	if (!sync_activity)
		arm = lc_mark_activity_arm(lane);
	mutex_unlock(&lane->lock);
	if (disarm)
		lc_disarm_activity(lane);
	if (sync_activity || arm)
		lc_arm_activity(lane);
}

static int lc_link_supplier(struct device *consumer, struct device *supplier)
{
	struct device_link *link;

	link = device_link_add(consumer, supplier, DL_FLAG_AUTOPROBE_CONSUMER);
	if (!link)
		return -EINVAL;
	return READ_ONCE(link->status) == DL_STATE_CONSUMER_PROBE ?
		0 : -EPROBE_DEFER;
}

static int lc_link_controller(struct device *controller, struct device *manager)
{
	return device_link_add(controller, manager,
			       DL_FLAG_AUTOPROBE_CONSUMER) ? 0 : -EINVAL;
}

static struct platform_device *lc_find_pdev(struct device_node *np,
					    const char *compatible)
{
	struct platform_device *pdev;

	if (!np || !of_device_is_available(np) ||
	    !of_device_is_compatible(np, compatible))
		return ERR_PTR(-EINVAL);
	pdev = of_find_device_by_node(np);
	return pdev ? pdev : ERR_PTR(-EPROBE_DEFER);
}

static struct platform_device *lc_find_controller(struct device_node *np)
{
	struct platform_device *pdev;
	struct device_node *parent;

	if (!np || !of_device_is_available(np) ||
	    !of_device_is_compatible(np, "snps,dwc3"))
		return ERR_PTR(-EINVAL);
	parent = of_get_parent(np);
	if (!parent || !of_device_is_available(parent) ||
	    !of_device_is_compatible(parent, "rockchip,rk3588-dwc3")) {
		of_node_put(parent);
		return ERR_PTR(-EINVAL);
	}
	pdev = of_find_device_by_node(parent);
	of_node_put(parent);

	return pdev ? pdev : ERR_PTR(-EPROBE_DEFER);
}

static int lc_link_input_consumer(struct device *dev, struct device **cc_dev)
{
	struct device_node *consumer_np, *connector;
	struct device *consumer;
	struct device_link *link;
	bool fusb302;
	int ret = 0;

	connector = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!connector ||
	    !of_device_is_compatible(connector, "usb-c-connector")) {
		of_node_put(connector);
		return -EINVAL;
	}
	consumer_np = of_get_parent(connector);
	of_node_put(connector);
	if (!consumer_np || !of_device_is_available(consumer_np)) {
		of_node_put(consumer_np);
		return -EINVAL;
	}
	fusb302 = of_device_is_compatible(consumer_np, "fcs,fusb302");
	consumer = get_dev_from_fwnode(of_fwnode_handle(consumer_np));
	of_node_put(consumer_np);
	if (!consumer)
		return -EPROBE_DEFER;
	link = device_link_add(consumer, dev, DL_FLAG_AUTOPROBE_CONSUMER);
	/*
	 * An existing fw_devlink cycle link does not inherit AUTOPROBE.
	 * Seed deferred probing while this supplier is still probing.
	 */
	if (link && !(READ_ONCE(link->flags) & DL_FLAG_AUTOPROBE_CONSUMER))
		ret = device_attach(consumer);
	if (ret < 0 && ret != -EPROBE_DEFER)
		goto put_consumer;
	if (!link) {
		ret = -EINVAL;
		goto put_consumer;
	}
	if (fusb302) {
		*cc_dev = consumer;
		return 0;
	}
	ret = 0;

put_consumer:
	put_device(consumer);
	return ret;
}

static void lc_quiesce(struct lc_lane *lane, bool restore)
{
	enum typec_orientation target;
	int ret;

	mutex_lock(&lane->lock);
	if (lane->teardown) {
		mutex_unlock(&lane->lock);
		return;
	}
	lane->teardown = true;
	lane->suspended = true;
	lane->activity_armed = false;
	lc_clear_guard(lane);
	lc_invalidate(lane, LC_IDLE);
	mutex_unlock(&lane->lock);
	if (lane->input_sw) {
		typec_switch_unregister(lane->input_sw);
		lane->input_sw = NULL;
	}
	usb_unregister_notify(&lane->usb_nb);
	cancel_delayed_work_sync(&lane->cc_work);
	cancel_delayed_work_sync(&lane->work);
	lc_disarm_activity(lane);
	ret =
		rockchip_usb2phy_unregister_host_ls_activity_notifier(lane->usb2_phy,
								      &lane->usb2_nb);
	if (ret)
		dev_warn(lane->dev, "failed to remove USB2 notifier: %d\n", ret);

	mutex_lock(&lane->switch_lock);
	if (restore) {
		mutex_lock(&lane->lock);
		target = lane->input_orientation != TYPEC_ORIENTATION_NONE ?
			lane->input_orientation : lane->last_good;
		mutex_unlock(&lane->lock);
		ret = typec_switch_set(lane->sw, target);
		if (ret && ret != -EBUSY)
			dev_warn(lane->dev, "failed to restore lanes: %d\n", ret);
	}
	mutex_unlock(&lane->switch_lock);
}

static int lc_suspend(struct device *dev)
{
	struct lc_lane *lane = dev_get_drvdata(dev);
	bool disarm = true, resume_scan;

	mutex_lock(&lane->lock);
	if (!lane->teardown) {
		resume_scan = lane->input_orientation == TYPEC_ORIENTATION_NONE &&
			      lc_has_scan_intent(lane) &&
			      !lc_has_configured_child(&lane->topology);
		lane->attach_pending |= resume_scan;
		lane->scan_deadline = 0;
		lane->cc_orientation = TYPEC_ORIENTATION_NONE;
		lane->scan_hint_pending = false;
		if (resume_scan)
			lane->activity_armed = false;
		lc_clear_guard(lane);
		lane->suspended = true;
		lc_invalidate(lane, LC_IDLE);
	}
	mutex_unlock(&lane->lock);
	cancel_delayed_work_sync(&lane->cc_work);
	cancel_delayed_work_sync(&lane->work);
	mutex_lock(&lane->switch_lock);
	mutex_unlock(&lane->switch_lock);
	mutex_lock(&lane->activity_lock);
	mutex_unlock(&lane->activity_lock);
	mutex_lock(&lane->lock);
	if (!lane->teardown)
		disarm = !lane->activity_armed;
	mutex_unlock(&lane->lock);
	if (disarm)
		lc_disarm_activity(lane);
	return 0;
}

static int lc_resume(struct device *dev)
{
	struct lc_lane *lane = dev_get_drvdata(dev);
	bool sync_activity = false;

	mutex_lock(&lane->lock);
	if (!lane->teardown) {
		lc_clear_guard(lane);
		memset(&lane->topology, 0, sizeof(lane->topology));
		lane->suspended = false;
		lc_invalidate(lane, LC_IDLE);
		lc_queue(lane, 0);
		lc_queue_cc(lane, 0);
		sync_activity = lane->activity_armed;
	}
	mutex_unlock(&lane->lock);
	if (sync_activity)
		lc_arm_activity(lane);
	flush_delayed_work(&lane->work);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(lc_pm_ops, lc_suspend, lc_resume);

static int lc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct typec_switch_desc desc = { };
	struct platform_device *controller, *supplier;
	struct device_node *np;
	struct lc_lane *lane;
	bool arm, has_input;
	int ret;

	lane = devm_kzalloc(dev, sizeof(*lane), GFP_KERNEL);
	if (!lane)
		return -ENOMEM;
	lane->dev = dev;
	lane->orientation = TYPEC_ORIENTATION_NORMAL;
	lane->last_good = TYPEC_ORIENTATION_NORMAL;
	lane->input_orientation = TYPEC_ORIENTATION_NONE;
	/* Seed one bounded no-CC search for the first SuperSpeed root. */
	lane->attach_pending = true;
	lane->startup_scan = true;
	lane->deadline = jiffies + msecs_to_jiffies(LC_SCAN_TIMEOUT_MS);
	lane->suspended = true;
	mutex_init(&lane->lock);
	mutex_init(&lane->switch_lock);
	mutex_init(&lane->activity_lock);
	INIT_DELAYED_WORK(&lane->work, lc_work);
	INIT_DELAYED_WORK(&lane->cc_work, lc_cc_work);
	platform_set_drvdata(pdev, lane);
	has_input = device_property_present(dev, "orientation-switch");

	np = of_parse_phandle(dev->of_node, "embedfire,usb-controller", 0);
	controller = lc_find_controller(np);
	of_node_put(np);
	if (IS_ERR(controller))
		return dev_err_probe(dev, PTR_ERR(controller),
				     "USB controller unavailable\n");
	lane->controller = &controller->dev;
	ret = lc_link_controller(lane->controller, dev);
	if (ret) {
		ret = dev_err_probe(dev, ret,
				    "failed to order USB controller probe\n");
		goto put_controller;
	}

	lane->usb2_phy = devm_phy_get(dev, "usb2");
	if (IS_ERR(lane->usb2_phy)) {
		ret = dev_err_probe(dev, PTR_ERR(lane->usb2_phy),
				    "failed to get USB2 PHY\n");
		goto put_controller;
	}
	ret = lc_link_supplier(dev, lane->usb2_phy->dev.parent);
	if (ret) {
		ret = dev_err_probe(dev, ret, "USB2 provider unavailable\n");
		goto put_controller;
	}

	np = of_graph_get_remote_node(dev->of_node, 0, -1);
	supplier = lc_find_pdev(np, "rockchip,rk3588-usbdp-phy");
	of_node_put(np);
	if (IS_ERR(supplier)) {
		ret = dev_err_probe(dev, PTR_ERR(supplier),
				    "orientation provider unavailable\n");
		goto put_controller;
	}
	ret = lc_link_supplier(dev, &supplier->dev);
	put_device(&supplier->dev);
	if (ret) {
		ret = dev_err_probe(dev, ret, "orientation provider not bound\n");
		goto put_controller;
	}
	if (has_input) {
		ret = lc_link_input_consumer(dev, &lane->cc_dev);
		if (ret) {
			ret = dev_err_probe(dev, ret,
					    "orientation consumer unavailable\n");
			goto put_controller;
		}
	}

	/* Resolve downstream before registering this node as an input switch. */
	lane->sw = fwnode_typec_switch_get(dev_fwnode(dev));
	if (IS_ERR_OR_NULL(lane->sw)) {
		ret = lane->sw ? PTR_ERR(lane->sw) : -ENODEV;
		ret = dev_err_probe(dev, ret, "orientation switch unavailable\n");
		goto put_cc;
	}
	ret = typec_switch_set(lane->sw, TYPEC_ORIENTATION_NORMAL);
	if (!ret) {
		lane->current_valid = true;
	} else if (ret == -EBUSY) {
		lc_invalidate(lane, LC_RESTORE);
	} else {
		ret = dev_err_probe(dev, ret, "failed to initialize lanes\n");
		goto put_switch;
	}

	lane->usb2_nb.notifier_call = lc_usb2_notify;
	ret =
		rockchip_usb2phy_register_host_ls_activity_notifier(lane->usb2_phy,
								    &lane->usb2_nb);
	if (ret) {
		ret = dev_err_probe(dev, ret, "failed to monitor USB2 activity\n");
		goto put_switch;
	}
	lane->usb_nb.notifier_call = lc_usb_notify;
	usb_register_notify(&lane->usb_nb);
	if (has_input) {
		desc.fwnode = dev_fwnode(dev);
		desc.drvdata = lane;
		desc.set = lc_input_set;
		lane->input_sw = typec_switch_register(dev, &desc);
		if (IS_ERR(lane->input_sw)) {
			ret = dev_err_probe(dev, PTR_ERR(lane->input_sw),
					    "failed to register orientation input\n");
			lane->input_sw = NULL;
			goto quiesce;
		}
	}
	mutex_lock(&lane->lock);
	lc_clear_guard(lane);
	lane->suspended = false;
	arm = lc_mark_activity_arm(lane);
	lc_queue(lane, 0);
	lc_queue_cc(lane, 0);
	mutex_unlock(&lane->lock);
	if (arm)
		lc_arm_activity(lane);
	return 0;

quiesce:
	lc_quiesce(lane, false);
put_switch:
	typec_switch_put(lane->sw);
put_cc:
	if (lane->cc_dev)
		put_device(lane->cc_dev);
put_controller:
	put_device(lane->controller);
	return ret;
}

static int lc_remove(struct platform_device *pdev)
{
	struct lc_lane *lane = platform_get_drvdata(pdev);

	lc_quiesce(lane, true);
	typec_switch_put(lane->sw);
	if (lane->cc_dev)
		put_device(lane->cc_dev);
	put_device(lane->controller);
	return 0;
}

static void lc_shutdown(struct platform_device *pdev)
{
	lc_quiesce(platform_get_drvdata(pdev), false);
}

static const struct of_device_id lc_of_match[] = {
	{ .compatible = "embedfire,rk3588-lubancat-5-usb-c-lane-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc_of_match);

static struct platform_driver lc_driver = {
	.probe = lc_probe,
	.remove = lc_remove,
	.shutdown = lc_shutdown,
	.driver = {
		.name = "lubancat-usb-c-lane",
		.of_match_table = lc_of_match,
		.pm = pm_sleep_ptr(&lc_pm_ops),
	},
};

static int __init lc_init(void)
{
	return platform_driver_register(&lc_driver);
}
subsys_initcall(lc_init);

static void __exit lc_exit(void)
{
	platform_driver_unregister(&lc_driver);
}
module_exit(lc_exit);

MODULE_AUTHOR("EmbedFire");
MODULE_DESCRIPTION("LubanCat-5 USB-C SuperSpeed lane orientation manager");
MODULE_LICENSE("GPL");
