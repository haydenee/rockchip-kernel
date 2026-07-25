/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PHY_ROCKCHIP_INNO_USB2_H
#define __PHY_ROCKCHIP_INNO_USB2_H

struct notifier_block;
struct phy;

#define ROCKCHIP_USB2PHY_HOST_LS_ACTIVITY	1

/*
 * The blocking notifier takes exclusive ownership of an OTG port's linestate
 * detector. Registration disables the detector without discarding an earlier
 * activity indication. arm() is idempotent and may report that indication
 * synchronously; otherwise exactly one activity disables the detector before
 * invoking the notifier with data == @phy. disarm() synchronizes callbacks and
 * discards a pending indication; unregister performs the same implicit disarm
 * and requires the notifier used at registration. All functions return zero on
 * success or a negative errno. Callbacks may sleep, but must not invoke these
 * control functions or unregister themselves synchronously. Since arm() may
 * invoke the callback before returning, callers must not hold a lock acquired
 * by the callback.
 */
int rockchip_usb2phy_register_host_ls_activity_notifier(struct phy *phy,
							struct notifier_block *nb);
int rockchip_usb2phy_unregister_host_ls_activity_notifier(struct phy *phy,
							  struct notifier_block *nb);
int rockchip_usb2phy_arm_host_ls_activity(struct phy *phy);
int rockchip_usb2phy_disarm_host_ls_activity(struct phy *phy);

#endif /* __PHY_ROCKCHIP_INNO_USB2_H */
