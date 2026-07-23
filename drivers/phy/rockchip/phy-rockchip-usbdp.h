/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _PHY_ROCKCHIP_USBDP_H
#define _PHY_ROCKCHIP_USBDP_H

#include <linux/types.h>

/* Private PHY_MODE_USB_HOST submodes used by the Rockchip USB2 companion. */
enum rockchip_udphy_usb_lane_scan_event {
	ROCKCHIP_UDPHY_USB_LANE_SCAN_NONE,
	ROCKCHIP_UDPHY_USB_LANE_SCAN_ATTACH_BEGIN,
	ROCKCHIP_UDPHY_USB_LANE_SCAN_ATTACH_END,
	ROCKCHIP_UDPHY_USB_LANE_SCAN_LINK_CONFIRMED,
};

/* Keep the packed private submode non-negative on 32-bit int platforms. */
#define ROCKCHIP_UDPHY_USB_LANE_SCAN_EVENT_MASK		0xffU
#define ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_SHIFT	8
#define ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_MASK	0x7fffffU

static inline int
rockchip_udphy_usb_lane_scan_submode(enum rockchip_udphy_usb_lane_scan_event event,
				    u32 generation)
{
	return event |
	       ((generation & ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_MASK) <<
		ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_SHIFT);
}

static inline enum rockchip_udphy_usb_lane_scan_event
rockchip_udphy_usb_lane_scan_event(int submode)
{
	return submode & ROCKCHIP_UDPHY_USB_LANE_SCAN_EVENT_MASK;
}

static inline u32 rockchip_udphy_usb_lane_scan_generation(int submode)
{
	return ((u32)submode >>
		ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_SHIFT) &
	       ROCKCHIP_UDPHY_USB_LANE_SCAN_GENERATION_MASK;
}

#endif
