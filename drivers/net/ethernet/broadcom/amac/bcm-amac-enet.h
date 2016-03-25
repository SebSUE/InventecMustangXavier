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

#ifndef __BCM_AMAC_ENET_H__
#define __BCM_AMAC_ENET_H__

#include <linux/phy.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock_types.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/kfifo.h>
#include <net/sock.h>


#define AMAC_MAX_PORTS 2

#define RX_BUF_SIZE		2048 /* MAX RX buffer size */
#define DMA_RX_DESC_NUM	256 /* Number of rx dma descriptors */
#define RXALIGN			16 /* Alignment for SKB */
/* 802.3as defines max packet size to be 2000 bytes, size is rounded up
 * to be multiple of 32 to be cache aligned
 */
#define DMA_RX_BUF_LEN	2016

#define DMA_TX_MAX_CHAIN_LEN	128 /* Limit TX DMA chain len */
/* Must be power of two because of the use of kfifo */
#define DMA_TX_MAX_QUEUE_LEN	(DMA_TX_MAX_CHAIN_LEN * 4)
/* Two descriptors per packet, one each for: config data and payload */
#define DMA_TX_DESC_NUM		(DMA_TX_MAX_QUEUE_LEN*2)

#define MIN_FRAME_LEN		64 /* ETH_ZLEN + FCS */

/* PORT Settings */
#define AMAC_PORT_SPEED_1G		SPEED_1000
#define AMAC_PORT_SPEED_100M	SPEED_100
#define AMAC_PORT_SPEED_10M		SPEED_10

#define AMAC_PORT_DEFAULT_SPEED		AMAC_PORT_SPEED_1G
#define AMAC_PORT_DEFAULT_ANEG		1
#define AMAC_PORT_DEFAULT_DUPLEX	1
#define AMAC_PORT_PAUSE_ENABLE		1
#define AMAC_PORT_PAUSE_DISABLE		0

/* Wake on LAN */
#define AMAC_WOL_ENABLE		1
#define AMAC_WOL_DISABLE	0

#define AMAC_WARM_RESET_SEQ 0x1111

/* Ethernet Statistics
 * Internally maintained data structure with detailed stats
 */
struct sysctl_ethstats {
	u64 rx_bytes;
	u64 rx_dropped_pkts;
	u64 rx_resyncs;
	u64 rx_wraparounds;
	u64 rx_syncchecked;
	u64 rx_syncdroppedpkts;
	u64 rx_noskb;
	u64 rx_broadcast;
	u64 rx_multicast;
	u64 rx_unicast;
	u64 tx_broadcast;
	u64 tx_multicast;
	u64 tx_unicast;
	u64 tx_dropped_pkts;
};

/* DMA Descriptor */
struct dma64_desc {
	u32 ctrl1;    /* misc control bits */
	u32 ctrl2;    /* buffer count and address extension */
	u32 addrlow;  /* mem addr of data buffer, bits 31:0 */
	u32 addrhigh; /* mem addr of data buffer, bits 63:32 */
};

/* DMA configuration data structure
 * These are used to configure the DMA block
 */
struct dma_cfg {
	void *descp;    /* Descriptor pointer */
	dma_addr_t addr;/* Descriptor bus addr */
	u32 rx_ptr;     /* Address to update rx_pointer register */
	u32 ring_len;   /* Total number of descriptors */
	u32 alloc_size; /* Total memory alloc in bytes */
	u32 index;      /* Current descriptor index */
};

/* SKB node data structure */
struct skb_list_node {
	struct sk_buff *skb;
	int len;
};

/* DMA private data for both RX and TX */
struct dma_priv {
	struct dma_cfg rx;
	struct dma_cfg tx;
	struct kfifo txfifo;
	u32 tx_max_pkts; /* max number of packets */
	u32 tx_curr;     /* current packet index */
	struct skb_list_node *tx_skb_list; /* list of skb given to hw for tx */
	struct skb_list_node *rx_skb_list; /* list of skb given to hw for rx */
};

/* Ports will have different configuration based on the type.
 * Default type is LAN
 */
enum port_type {
	AMAC_PORT_TYPE_LAN = 0,
	AMAC_PORT_TYPE_PC = 1,
	AMAC_PORT_TYPE_MAX = 2
};

/* Tag support */
enum tag_support {
	AMAC_TAG_NONE = 0,
	AMAC_TAG_BRCM = 1,
};

/* PHY data structure */
struct phy_priv {
	u32 aneg; /* auto negotiation */
	u32 speed; /* port speed */
	u32 duplex; /* port duplex */
	u32 pause; /* pause status */
	u32 link; /* link status */
};

/* Structure contains all Port related settings
 * Settings are read from Device tree or during PHY setup
 */
struct port_info {
	u32 num; /* Port number */
	enum port_type type;
	u32 phy_id; /* Associated PHY id */
	struct phy_device *phydev; /* Connected PHY dev */
	struct phy_priv phy_info; /* PHY info (from phy driver) */
	struct phy_priv phy_def; /* PHY default settings (DT) */
	u32 wol; /* Set if WOL state is active */
};

/* Ethernet Port data structure */
struct port_data {
	struct port_info info[AMAC_MAX_PORTS]; /* Port details */
	u32 count; /* Number of ports */
	u32 wol_en; /* WOL status, if any ports are in WOL */
	struct mutex wol_lock;
};

/* AMAC registers */
struct bcm_sf2_reg_base {
	void __iomem *amac_core;
	void __iomem *amac_io_ctrl;
	void __iomem *amac_idm_reset;
	void __iomem *icfg_regs;
	void __iomem *switch_global_cfg;
	void __iomem *crmu_io_pad_ctrl;
};

/* Structure contains all the hardware info */
struct bcm_sf2_hw {
	struct bcm_sf2_reg_base reg; /* iomapped register addresses */
	u32 intr_num; /* Interrupt number */
};

/* Device access/config oprands */
struct esw_ops {
	int (*read_reg)(void *esw, u8 page, u8 offset,
			void *val, int len);
	int (*write_reg)(void *esw, u8 page, u8 offset,
			 void *val, int len);
};

/* Private state per RoboSwitch */
struct esw_info {
	void __iomem *srab_base;
	struct esw_ops *ops; /* device ops */
	u32 devid32; /* Device id for the switch (32bits) */
	u32 corerev; /* Core rev of internal switch */
	spinlock_t lock;
};

enum amac_reboot_reason {
	AMAC_REBOOT_COLD = 0,
	AMAC_REBOOT_WARM = 1
};

/* AMAC Driver's private data structure.
 * Contains data for the driver instance. This structure is used
 * throughout the driver to derrive status of various blocks,
 * settings, stats, hw registers etc.
 */
struct bcm_amac_priv {
	struct napi_struct napi ____cacheline_aligned;
	struct tasklet_struct tx_tasklet;

	struct net_device *ndev; /* net device reference */
	struct platform_device *pdev; /* platform device reference */

	struct port_data port; /* Port and PHY Info */

	struct mii_bus *mii_bus; /* MII/MDIO Bus */
	u32 mdio_irq[PHY_MAX_ADDR];

	struct dma_priv dma; /* DMA info */
	u32 dmactrlflags; /* DMA Flags */

	/* the netlink socket to send link status change notification */
	struct sock *nl_sk;
	/* the sequence number of the link status change notification */
	u32 nl_seq;

	struct bcm_sf2_hw hw; /* Hardware info */

	struct sockaddr cur_etheraddr; /* local ethernet address */

	/* ESW info */
	u32 switchmode; /* 1 - switch mode, 0 - switch bypass mode */
	struct esw_info esw;

	/* BRCM tag supported
	 * Using BRCM tag enables special switch forwarding capabilities.
	 */
	enum tag_support brcm_tag;

	struct sysctl_ethstats eth_stats;

	spinlock_t lock; /* used by netdev api's */
	u32 msg_enable; /* message filter bit mask */
// ---------- [ADK] ------------
	struct device_node *phy_dn;
	struct device_node *mdio_dn;
// ---------------------------

	enum amac_reboot_reason reboot;
};


void bcm_amac_enet_send_link_stat(struct bcm_amac_priv *privp,
	unsigned int port_idx, struct phy_device *phydev, unsigned int link);

#endif /*__BCM_AMAC_ENET_H__*/
