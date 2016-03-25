/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BCM_AMAC_ETHTOOL_H__
#define __BCM_AMAC_ETHTOOL_H__
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/netlink.h>
#include <linux/if.h>
#include <linux/ethtool.h>


int bcm_ethtool_get_settings(struct net_device *ndev,
	struct ethtool_cmd *cmd);
int bcm_ethtool_set_settings(struct net_device *ndev,
	struct ethtool_cmd *cmd);
void bcm_ethtool_get_drvinfo(struct net_device *ndev,
	struct ethtool_drvinfo *info);
int bcm_ethtool_nway_reset(struct net_device *ndev);
void bcm_ethtool_get_strings(struct net_device *ndev,
	u32 stringset, u8 *buf);
int bcm_ethtool_get_sset_count(struct net_device *ndev, int sset);
void bcm_ethtool_get_stats(struct net_device *ndev,
		struct ethtool_stats *estats, u64 *tmp_stats);
void bcm_ethtool_get_ringparam(struct net_device *ndev,
		struct ethtool_ringparam *ering);
void bcm_ethtool_get_pauseparam(struct net_device *ndev,
				    struct ethtool_pauseparam *epause);

#endif /*__BCM_AMAC_ETHTOOL_H__*/
