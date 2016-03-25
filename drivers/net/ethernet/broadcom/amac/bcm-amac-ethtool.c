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

#include "bcm-amac-ethtool.h"
#include "bcm-amac-enet.h"
#include "bcm-sf2-regs.h"


#define MOD_NAME		"AMAC"
#define MOD_VERSION		"0.9.0"

#define ETH_GSTRING_LEN    32

#define REG_64_BITS         64
#define REG_32_BITS         32


struct ethtool_keys {
	const char string[ETH_GSTRING_LEN];
};


/* Derrived from 'struct bcm_ethtool_mib' */
static const struct ethtool_keys ethtool_stats_keys[] = {
	{ "                    Port" },
	{ "                TxOctets" },
	{ "              TxDropPkts" },
	{ "         TxBroadcastPkts" },
	{ "         TxMulticastPkts" },
	{ "           TxUnicastPkts" },
	{ "            TxCollisions" },
	{ "       TxSingleCollision" },
	{ "     TxMultipleCollision" },
	{ "      TxDeferredTransmit" },
	{ "         TxLateCollision" },
	{ "    TxExcessiveCollision" },
	{ "           TxFrameInDisc" },
	{ "             TxPausePkts" },
	{ "                RxOctets" },
	{ "         RxUndersizePkts" },
	{ "             RxPausePkts" },
	{ "          RxPkts64Octets" },
	{ "     RxPkts65To127Octets" },
	{ "    RxPkts128To255Octets" },
	{ "    RxPkts256To511Octets" },
	{ "   RxPkts512To1023Octets" },
	{ "RxPkts1024ToMaxPktOctets" },
	{ "          RxOversizePkts" },
	{ "               RxJabbers" },
	{ "       RxAlignmentErrors" },
	{ "             RxFcsErrors" },
	{ "            RxGoodOctets" },
	{ "              RxDropPkts" },
	{ "           RxUnicastPkts" },
	{ "         RxMulticastPkts" },
	{ "         RxBroadcastPkts" },
	{ "             RxSaChanges" },
	{ "             RxFragments" },
	{ "         RxJumboPktCount" },
	{ "           RxSymbolError" },
	{ "               RxDiscard" }
};

struct bcm_ethtool_mib {
	/* Warning: do not change the names of the following members
	 * as they are used in macro expansion to formulate
	 * corresponding data routines.
	 */
	u64 port_num;
	u64 tx_octets;
	u64 tx_drop_pkts;
	u64 tx_broadcast_pkts;
	u64 tx_multicast_pkts;
	u64 tx_unicast_pkts;
	u64 tx_collisions;
	u64 tx_single_collision;
	u64 tx_multiple_collision;
	u64 tx_deferred_txmit;
	u64 tx_late_collision;
	u64 tx_excessive_collision;
	u64 tx_frame_in_disc;
	u64 tx_pause_pkts;
	u64 rx_octets;
	u64 rx_undersize_pkts;
	u64 rx_pause_pkts;
	u64 rx_pkts64octets;
	u64 rx_pkts65to127octets;
	u64 rx_pkts128to255octets;
	u64 rx_pkts256to511octets;
	u64 rx_pkts512to1023octets;
	u64 rx_pkts1024tomaxpktoctets;
	u64 rx_oversize_pkts;
	u64 rx_jabbers;
	u64 rx_alignment_errors;
	u64 rx_fcs_errors;
	u64 rx_good_octets;
	u64 rx_drop_pkts;
	u64 rx_unicast_pkts;
	u64 rx_multicast_pkts;
	u64 rx_broadcast_pkts;
	u64 rx_sa_changes;
	u64 rx_fragments;
	u64 rx_jumbo_pktcount;
	u64 rx_symbol_error;
	u64 rx_discard;
};


int bcm_ethtool_get_settings(struct net_device *ndev,
	struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct phy_device *phydev;
	int i;

	/* Since there are multiple PHY's, return the LAN settings */
	for (i = 0; i < privp->port.count; i++)
		if (privp->port.info[i].type == AMAC_PORT_TYPE_LAN)
			break;

	phydev = privp->port.info[i].phydev;

	if (phydev == NULL)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

int bcm_ethtool_set_settings(struct net_device *ndev,
	struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct phy_device *phydev;
	int i;

	/* Since there are multiple PHY's, return the LAN settings */
	for (i = 0; i < privp->port.count; i++)
		if (privp->port.info[i].type == AMAC_PORT_TYPE_LAN)
			break;

	phydev = privp->port.info[i].phydev;

	if (phydev == NULL)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);

}

void bcm_ethtool_get_drvinfo(struct net_device *ndev,
	struct ethtool_drvinfo *info)
{
	strcpy(info->driver, MOD_NAME);
	strcpy(info->version, MOD_VERSION);
	strcpy(info->fw_version, "N/A");
	strcpy(info->bus_info, "mdio");
}

int bcm_ethtool_nway_reset(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc = -ENODEV;
	int i;

	if (!netif_running(ndev))
		return -EAGAIN;

	/* Restart aneg for all the PHY's */
	for (i = 0; i < privp->port.count; i++) {
		if (privp->port.info[i].phydev)
			rc += phy_start_aneg(
				privp->port.info[i].phydev);
	}
	return rc;
}

void bcm_ethtool_get_strings(struct net_device *ndev,
	u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &ethtool_stats_keys,
			sizeof(ethtool_stats_keys));
		break;

	default:
		WARN_ON(1);	/* we need a WARN() */
		break;
	}
}

int bcm_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	switch (sset) {
	case ETH_SS_STATS:
		/* The stats are supported only in switch mode */
		if (privp->switchmode)
			return ARRAY_SIZE(ethtool_stats_keys);
		else
			return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static void ethtool_get_port_stats(struct bcm_amac_priv *privp,
	struct bcm_ethtool_mib *mib, u8 port)
{
	u8 mib_page;

	if (port == 0)
		mib_page = PAGE_MIB_PORT0;
	else if (port == 1)
		mib_page = PAGE_MIB_PORT1;
	else
		mib_page = PAGE_MIB_IMP;

	mib->port_num = port;

	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_OCTETS,
			&mib->tx_octets, REG_64_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_DROP_PKTS,
			&mib->tx_drop_pkts, REG_64_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_BROADCAST_PKTS,
			&mib->tx_broadcast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_MULTICAST_PKTS,
			&mib->tx_multicast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_UNICAST_PKTS,
			&mib->tx_unicast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_COLLISIONS,
			&mib->tx_collisions, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_SINGLE_COLLISION,
			&mib->tx_single_collision, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_MULTIPLE_COLLISION,
			&mib->tx_multiple_collision, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_DEFERRED_TXMIT,
			&mib->tx_deferred_txmit, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_LATE_COLLISION,
			&mib->tx_late_collision, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_EXCESSIVE_COLLISION,
			&mib->tx_excessive_collision, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_FRAME_IN_DISC,
			&mib->tx_frame_in_disc, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_TX_PAUSE_PKTS,
			&mib->tx_pause_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_OCTETS,
			&mib->rx_octets, REG_64_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_UNDERSIZE_PKTS,
			&mib->rx_undersize_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PAUSE_PKTS,
			&mib->rx_pause_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS64OCTETS,
			&mib->rx_pkts64octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS65TO127OCTETS,
			&mib->rx_pkts65to127octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS128TO255OCTETS,
			&mib->rx_pkts128to255octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS256TO511OCTETS,
			&mib->rx_pkts256to511octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS512TO1023OCTETS,
			&mib->rx_pkts512to1023octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_PKTS1024TOMAXPKTOCTETS,
			&mib->rx_pkts1024tomaxpktoctets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_OVERSIZE_PKTS,
			&mib->rx_oversize_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_JABBERS,
			&mib->rx_jabbers, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_ALIGNMENT_ERRORS,
			&mib->rx_alignment_errors, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_FCS_ERRORS,
			&mib->rx_fcs_errors, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_GOOD_OCTETS,
			&mib->rx_good_octets, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_DROP_PKTS,
			&mib->rx_drop_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_UNICAST_PKTS,
			&mib->rx_unicast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_MULTICAST_PKTS,
			&mib->rx_multicast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_BROADCAST_PKTS,
			&mib->rx_broadcast_pkts, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_SA_CHANGES,
			&mib->rx_sa_changes, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_FRAGMENTS,
			&mib->rx_fragments, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_JUMBO_PKTCOUNT,
			&mib->rx_jumbo_pktcount, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_SYMBOL_ERROR,
			&mib->rx_symbol_error, REG_32_BITS);
	privp->esw.ops->read_reg(&privp->esw,
			mib_page, REG_RX_DISCARD,
			&mib->rx_discard, REG_32_BITS);
}

void bcm_ethtool_get_stats(struct net_device *ndev,
		struct ethtool_stats *estats, u64 *tmp_stats)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int i;

	/* Since there are multiple PHY's, return the LAN settings */
	for (i = 0; i < privp->port.count; i++)
		if (privp->port.info[i].type == AMAC_PORT_TYPE_LAN) {
			/* Retrieve and return the MiB of the LAN port */
			ethtool_get_port_stats(privp,
				(struct bcm_ethtool_mib *)tmp_stats,
				privp->port.info[i].num);
			break;
		}
}

void bcm_ethtool_get_ringparam(struct net_device *ndev,
		struct ethtool_ringparam *ering)
{
	(void)ndev;

	ering->rx_max_pending = DMA_RX_DESC_NUM;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->tx_max_pending = DMA_TX_MAX_QUEUE_LEN;

	ering->rx_pending = DMA_RX_DESC_NUM;
	ering->rx_mini_pending = 0;
	ering->rx_jumbo_pending = 0;
	ering->tx_pending = DMA_TX_MAX_QUEUE_LEN;
}

void bcm_ethtool_get_pauseparam(struct net_device *ndev,
				    struct ethtool_pauseparam *epause)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int i;

	/* Since there are multiple PHY's, return the LAN settings */
	for (i = 0; i < privp->port.count; i++)
		if (privp->port.info[i].type == AMAC_PORT_TYPE_LAN)
			break;

	epause->autoneg = privp->port.info[i].phy_info.aneg;
	epause->rx_pause = privp->port.info[i].phy_info.pause;
	epause->tx_pause = privp->port.info[i].phy_info.pause;
}



