/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_USB_FUSB302_H
#define _LINUX_USB_FUSB302_H

#include <linux/usb/typec.h>

struct device;

/**
 * fusb302_sink_cc_sample() - sample local CC polarity on a fixed-Sink port
 * @dev: bound fcs,fusb302 device; the caller must hold a device reference
 * @orientation: local CC orientation, or %TYPEC_ORIENTATION_NONE if absent
 *
 * Temporarily measures both CC pins without sourcing VBUS. On success, it
 * restores the complete FUSB302 register state before returning. This reports
 * only the local plug orientation; callers must not treat it as far-end
 * orientation.
 *
 * Return: 0 for a valid sample, -EAGAIN when the current PM or TCPM state
 * cannot be sampled, -EUCLEAN after any register-restore failure, or another
 * negative errno for an access or configuration error.
 */
int fusb302_sink_cc_sample(struct device *dev,
			   enum typec_orientation *orientation);

#endif /* _LINUX_USB_FUSB302_H */
