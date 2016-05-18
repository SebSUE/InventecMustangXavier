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

#include <linux/err.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/sysctl.h>
#include <linux/io.h>
#include <linux/ctype.h>
#include <linux/netlink.h>
#include <linux/of.h>
#include <linux/if.h>
#include <linux/device.h>
#include <net/sock.h>
#include <asm/dma.h>

#include <linux/bcm_esw_ioctl.h>
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"
#include "bcm-amac-pkt.h"
#include "bcm-amac-ethtool.h"
#include "bcm-robo.h"

#define TX_TIMEOUT (5 * HZ)
#define QUOTA NAPI_POLL_WEIGHT /* 64 */

/* check if the buffer contains a BROADCAST DST MAC address */
#define is_broadcast(bufp)	((bufp[0] == 0xFF) && \
				(bufp[1] == 0xFF) && \
				(bufp[2] == 0xFF) && \
				(bufp[3] == 0xFF) && \
				(bufp[4] == 0xFF) && \
				(bufp[5] == 0xFF))

/* check if the buffer contains a MULTICAST DST MAC address */
#define is_multicast(bufp)	(bufp[0] & 0x01)

/* Netlink */
#define NETLINK_MAX_PAYLOAD 256
#define KERNEL_PID 0
/* to mcast group 1<<0 */
#define DST_GROUP 1
#define LINK_UP_STR \
	"Link UP for %s Port (Port=%d, PHY=%d) at %d Mbps, %s duplex\n"
#define LINK_DOWN_STR \
	"Link DOWN for %s Port (Port=%d, PHY=%d)\n"


struct bcm_amac_cmd_line_param {
	unsigned int lswap;
	char mac_addr[20];
	enum amac_reboot_reason reboot;
};

static struct bcm_amac_cmd_line_param cmdline_params = {
	0,
	"00:00:00:00:00:00",
	AMAC_REBOOT_COLD
};


static void bcm_amac_enet_shutdown(struct platform_device *pdev);
static int bcm_amac_notify_reboot(struct notifier_block *nb,
	unsigned long event, void *ptr);
static int bcm_amac_enet_suspend(struct device *dev);
static int bcm_amac_enet_resume(struct device *dev);
static int bcm_amac_enet_remove(struct platform_device *pdev);
static int bcm_amac_enet_start(struct bcm_amac_priv *privp);
static int bcm_amac_enet_probe(struct platform_device *pdev);
static int bcm_amac_enet_do_ioctl(struct net_device *dev,
				  struct ifreq *ifr, int cmd);

static int bcm_amac_get_dt_data(struct platform_device *pdev,
	struct bcm_amac_priv *privp);

static int bcm_amac_enet_open(struct net_device *dev);
static int amac_enet_set_mac(struct net_device *dev, void *addr);

static void amac_enet_parse_mac_addr(char *macstr, char *arsedmac);

static void amac_tx_task(unsigned long data);

static int bcm_amac_enet_close(struct net_device *dev);
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *dev);
static void bcm_amac_enet_tx_timeout(struct net_device *dev);

static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota);
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *dev);
static void amac_clear_stats(struct sysctl_ethstats *stats);

static const struct net_device_ops bcm_amac_enet_ops = {
	.ndo_open = bcm_amac_enet_open,
	.ndo_stop = bcm_amac_enet_close,
	.ndo_start_xmit = bcm_amac_enet_hard_xmit,
	.ndo_tx_timeout = bcm_amac_enet_tx_timeout,
	.ndo_get_stats = bcm_amac_enet_get_stats,
	.ndo_set_rx_mode = bcm_amac_set_rx_mode,
	.ndo_do_ioctl = bcm_amac_enet_do_ioctl,
	.ndo_set_mac_address = amac_enet_set_mac,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu = eth_change_mtu
};

static const struct ethtool_ops bcm_amac_ethtool_ops = {
	.get_settings      = bcm_ethtool_get_settings,
	.set_settings      = bcm_ethtool_set_settings,
	.get_drvinfo       = bcm_ethtool_get_drvinfo,
	.nway_reset        = bcm_ethtool_nway_reset,
	.get_link          = ethtool_op_get_link,
	.get_ringparam     = bcm_ethtool_get_ringparam,
	.get_pauseparam    = bcm_ethtool_get_pauseparam,
	.get_strings       = bcm_ethtool_get_strings,
	.get_ethtool_stats = bcm_ethtool_get_stats,
	.get_sset_count    = bcm_ethtool_get_sset_count,
};

static struct notifier_block bcm_amac_enet_reboot_notifier = {
	.notifier_call = bcm_amac_notify_reboot,
	.priority      = 0
};


/**
 * bcm_amac_enet_get_stats() - Get device status
 * @dev: net device pointer
 *
 * Returns: network statistics
 */
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

/**
 * amac_enet_parse_mac_addr() - parse the mac address
 * @macstr - string to be parsed
 * @parsedmac - parsed mac address in hex
 */
static void amac_enet_parse_mac_addr(char *macstr, char *parsedmac)
{
	int i, j;
	unsigned char result, value;

	for (i = 0; i < ETH_ALEN; i++) {
		result = 0;

		if (i != (ETH_ALEN - 1) && *(macstr + 2) != ':')
			return;

		for (j = 0; j < 2; j++) {
			if (!isxdigit(*macstr))
				return;
			value = isdigit(*macstr) ?
					*macstr - '0' :
					toupper(*macstr) - 'A' + 10;
			if (value > 16)
				return;
			result = result * 16 + value;
			macstr++;
		}
		macstr++;
		parsedmac[i] = result;
	}
}

/**
 * bcm_amac_enet_start() - Initialize core, phy,mac address etc.
 * @privp: device info pointer
 *
 * @Returns: 0 or error
 */
static int bcm_amac_enet_start(struct bcm_amac_priv *privp)
{
	int rc;
	struct sockaddr parsed_mac;

	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		dev_err(&privp->pdev->dev,
			"Failed to configure MAC\n");
		return rc;
	}

	/* disable DMA */
	rc = bcm_amac_enable_rx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Couldn't disable RX DMA\n");
		return rc;
	}

	rc = bcm_amac_enable_tx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Couldn't disable TX DMA\n");
		return rc;
	}

	/* Initialize the PHY's */
	rc = bcm_amac_gphy_init(privp->ndev);
	if (rc != 0) {
		dev_err(&privp->pdev->dev, "%s: PHY Init failed\n", __func__);
		return rc;
	}
	/* parse cmd line and set mac address */
	amac_enet_parse_mac_addr(cmdline_params.mac_addr, parsed_mac.sa_data);
	if (!is_valid_ether_addr(parsed_mac.sa_data)) {
		dev_err(&privp->pdev->dev,
			"bcm-amac: Invalid ethaddr - %s\n",
			cmdline_params.mac_addr);
		return rc;
	}
	amac_enet_set_mac(privp->ndev, (void *)&parsed_mac);

	/* Register GMAC Interrupt */
	rc = devm_request_irq(&privp->pdev->dev, privp->hw.intr_num,
			bcm_amac_isr, IRQF_SHARED, "amac_enet", privp);
	if (rc) {
		netdev_err(privp->ndev,
			"IRQ request failed, irq=%i, err=%i\n",
			privp->hw.intr_num, rc);
		return rc;
	}

	return 0;
}

static int bcm_amac_enet_stop(struct bcm_amac_priv *privp)
{

	return 1;
}

/**
 * amac_tx_task() - Packet transmission routing.
 * @data: device info pointer
 *
 * This api is registered with the tx tasklet. The task performs the
 * transmission of packets if the tx is free
 *
 * Returns: none
 */
static void amac_tx_task(unsigned long data)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)data;

	/* If TX DMA is busy return */
	if (1 == bcm_amac_get_tx_flag())
		return;

	bcm_amac_tx_clean(privp);
	bcm_amac_tx_send_packet(privp);
}

/**
 * bcm_amac_enet_rx_poll() - Packet reception routine
 * @napi: napi structure pointer
 * @quota: quota info
 *
 * This NAPI RX routine gets the received packets and sends it to the upper
 * layers. IT strips off any Broadcom Tags found in the packet. It also
 * updates the packet stats.
 * It enables the RX interrupt based on the quota.
 *
 * Returns: quota used
 */
static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota)
{
	struct bcm_amac_priv *privp =
		container_of(napi, struct bcm_amac_priv, napi);
	int used = 0;
	struct sk_buff *skb;
	int len;
	char *bufp;
	static int last_used;
	static int prev_used[8] = {0};
	static int prev_idx;

	if (privp == NULL)
		return -EINVAL;

	while (used < quota) {

		/* Check and retrieve rx packets */
		len = bcm_amac_dma_get_rx_data(privp, &skb);
		if (len > 0) {

			/* Strip the BRCM Tag if enabled */
			if (privp->brcm_tag)
				len -= bcm_amac_pkt_rm_brcm_tag(&skb);

			/* Check frame length and discard if invalid*/
			if (unlikely(len < MIN_FRAME_LEN)) {
				netdev_err(privp->ndev,
					"bad frame: len=%i\n", len);

				dev_kfree_skb_any(skb);

				/* Update error stats */
				privp->ndev->stats.rx_dropped++;
				privp->eth_stats.rx_dropped_pkts++;
				continue;
			}

			/* Process the packet */
			bufp = skb->data;

			/* Update remainder of socket buffer information */
			skb_put(skb, len);

			skb->dev = privp->ndev;
			skb->protocol = eth_type_trans(skb, privp->ndev);
			skb->ip_summed = CHECKSUM_NONE;

			/* Update Stats */
			privp->ndev->stats.rx_bytes += len;
			privp->ndev->stats.rx_packets++;
			if (is_broadcast(bufp))
				privp->eth_stats.rx_broadcast++;
			else if (is_multicast(bufp))
				privp->eth_stats.rx_multicast++;
			else
				privp->eth_stats.rx_unicast++;

			/* Pass the packet up for processing */
			netif_receive_skb(skb);

			used++;
		} else if (len == 0)
			break; /* no frames to process */
		else if (len == -EBADMSG) {
			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->eth_stats.rx_dropped_pkts++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					"rx frame length err, used=%d\n", used);

			continue;
		} else {
			/* Error retriving frame */

			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->eth_stats.rx_noskb++;
			privp->eth_stats.rx_dropped_pkts++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					"rx skb alloc err, used=%d\n", used);

			/* Don't try to read any more frames */
			/* just drop out of the loop */
			break;
		}
	}

	/* If quota not fully consumed, exit polling mode */
	if (likely(used < quota)) {
		napi_complete(napi);

		/* Enable RX Interrupt */
		bcm_amac_enable_rx_intr(privp, true);
	}

	last_used = used;
	prev_used[prev_idx++%8] = used;

	return used;
}

/**
 * bcm_amac_enet_open() - Ethernet interface open routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is opened.
 * This stats the DMA's, enables the RX (NAPI), powers up the PHY,
 * starts up the TX queue etc.
 *
 * Returns: '0' for success or the error number
 */
static int bcm_amac_enet_open(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;

	/* Allocate a TX fifo to stash to hold skb pointers */
	rc = kfifo_alloc(&privp->dma.txfifo,
			DMA_TX_MAX_QUEUE_LEN*sizeof(void *), GFP_KERNEL);
	if (rc) {
		netdev_err(ndev,
			"cannot alloc tx fifo, err=%i\n", rc);
		return rc;
	}

	/* Enable napi before rx interrupts */
	napi_enable(&privp->napi);

	/* Start DMA */
	rc = bcm_amac_dma_start(privp);
	if (rc) {
		netdev_err(ndev, "Failed to start DMA\n");
		goto err_free_kfifo;
	}

	bcm_amac_core_enable(privp, 1);

	/* Power up the PHY */
	bcm_amac_gphy_powerup(privp);

	netif_wake_queue(ndev);

	return rc;

err_free_kfifo:
	kfifo_free(&privp->dma.txfifo);
	netdev_err(ndev, "%s, open failed!\n", __func__);

	return rc;
}

/**
 * bcm_amac_enet_close() - Ethernet interface close routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is closed or disabled.
 * This stops the DMA, disables interrupts, switches off the PHY, disables
 * NAPI routine etc.
 *
 * Returns: '0'
 */
static int bcm_amac_enet_close(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	netif_stop_queue(ndev);

	/* Shutdown PHY(s) */
	bcm_amac_gphy_shutdown(privp);

	/* Disable RX NAPI */
	napi_disable(&privp->napi);

	/* Stop DMA */
	bcm_amac_dma_stop(privp);

	netif_carrier_off(ndev);

	kfifo_free(&privp->dma.txfifo);
	return 0;
}

/**
 * amac_enet_set_mac() - Sets up the mac address
 * @dev: network device pointer
 * @addr: mac address
 *
 * Assigns the mac address to the interface. This also adds the mac address
 * to the ARL entry if the switch is enabled.
 *
 * Returns: '0' or error
 */
static int amac_enet_set_mac(struct net_device *ndev, void *addr)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	rc = eth_mac_addr(ndev, addr);
	if (rc) {
		dev_err(&privp->pdev->dev, "cannot setup MAC err=%i\n", rc);
		return rc;
	}

	memcpy(privp->cur_etheraddr.sa_data, ndev->dev_addr, 6);

	return bcm_amac_set_mac(privp, privp->cur_etheraddr.sa_data);
}

/**
 * bcm_amac_enet_hard_xmit() - hard transmit routine
 * @skb: skb buffer pointer with data to be transmitted
 * @ndev: network device pointer.
 *
 * The hard transmit routine is called by the upper layers to transmit data.
 * The data is part of the skb pointer. The interface adds broadcom tags if
 * enabled, inserts the skb into the interanl transmit queue and schedules
 * the transmit task to run.
 *
 * Returns: NETDEV_TX_OK
 */
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct bcm_amac_priv *privp;
	int len;
	int rc;

	rc = NETDEV_TX_OK;
	privp = netdev_priv(ndev);

	/* BRCM Header support for Switch mode (if enabled)
	 * Handle the BRCM tag before adding the packet
	 * to the queue.
	 *
	 * Switch-by-pass mode is handled while reading the
	 * DT information and tag will have been disabled.
	 */
	if (privp->brcm_tag) {
		if (bcm_amac_pkt_add_bcm_tag(&skb)) {
			/* Error in adding the tag
			 * Still returning OK, but dropping
			 * the packet.
			 */
			rc = NETDEV_TX_OK;
			goto err_enet_hard_xmit;
		}
	}

	/* Insert skb pointer into fifo */
	len = kfifo_in_locked(&privp->dma.txfifo, (unsigned char *)&skb,
						sizeof(skb), &privp->lock);
	if (unlikely(len != sizeof(skb))) {
		/* Not enough space, which shouldn't happen since the queue
		 * should have been stopped already.
		 */
		netif_stop_queue(ndev);
		netdev_info(privp->ndev,
			"xmit called with no tx desc avail!");

		ndev->stats.tx_fifo_errors++;

		rc = NETDEV_TX_OK;
		goto err_enet_hard_xmit;
	}

	tasklet_schedule(&privp->tx_tasklet);

	/* Update stats */
	if (is_broadcast(skb->data))
		privp->eth_stats.tx_broadcast++;
	else if (is_multicast(skb->data))
		privp->eth_stats.tx_multicast++;
	else
		privp->eth_stats.tx_unicast++;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	return rc;

err_enet_hard_xmit:
	kfree_skb(skb);

	/* Update stats */
	ndev->stats.tx_dropped++;
	privp->eth_stats.tx_dropped_pkts++;

	return rc;
}

/**
 * bcm_amac_enet_tx_timeout() - Transmit timeout routine
 * @ndev - network device pointer
 */
static void bcm_amac_enet_tx_timeout(struct net_device *ndev)
{
	struct bcm_amac_priv *privp;

	privp = netdev_priv(ndev);

	netdev_warn(ndev, "tx timeout\n");

	ndev->stats.tx_errors++;
	ndev->trans_start = jiffies; /* prevent tx timeout */

	netif_wake_queue(ndev);
}

static int __init bcm_amac_setup_lswap(char *s)
{
	int rc;

	if ((NULL == s) || (0 == strlen(s)))
		return 0;

	pr_info("bcm-amac: setting ethlaneswap: %s\n", s);
	rc = (unsigned int)kstrtoint(s, 10, &cmdline_params.lswap);

	if (cmdline_params.lswap > 1) {
		pr_err("bcm-amac: Invalid lswap (%s) specified, defaulting to 0\n",
			s);
		cmdline_params.lswap = 0;
	}

	bcm_amac_gphy_set_lswap(cmdline_params.lswap);

	return 1;
}
__setup("lswap=", bcm_amac_setup_lswap);

static int __init bcm_amac_setup_ethaddr(char *s)
{
	bool rc;

	if ((NULL == s) || (0 == strlen(s))) {
		pr_err("bcm-amac: No ethaddr specified\n");
		return 0;
	}

        pr_info("bcm-amac: setting ethaddr: %s\n", s);
        strcpy(cmdline_params.mac_addr, s);

	return 1;
}
__setup("ethaddr=", bcm_amac_setup_ethaddr);

/**
 * bcm_amac_enet_do_ioctl() - ioctl support in the driver
 * @ndev: network device
 * @ifr: ioctl data pointer
 * @cmd: ioctl cmd
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_do_ioctl(struct net_device *ndev,
				  struct ifreq *ifr, int cmd)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct esw_info *esw = &(privp->esw);

	switch (cmd) {
	case SIOCESW_REG_READ:
	{
		struct esw_reg_data reg;
		if (copy_from_user(&reg, ifr->ifr_data, sizeof(reg))) {
			rc = -EFAULT;
			break;
		}

		/* Clear the data */
		reg.data = 0;
		rc = esw->ops->read_reg(esw, reg.page, reg.offset,
					&(reg.data), reg.len);

		if (rc == 0)
			if (copy_to_user(ifr->ifr_data, &reg, sizeof(reg)))
				rc = -EFAULT;
	}
	break;

	case SIOCESW_REG_WRITE:
	{
		struct esw_reg_data reg;
		if (copy_from_user(&reg, ifr->ifr_data, sizeof(reg))) {
			rc = -EFAULT;
			break;
		}
		rc = esw->ops->write_reg(esw, reg.page, reg.offset,
					 &(reg.data), reg.len);
	}
	break;

	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
	{
		struct mii_ioctl_data *mii_data = if_mii(ifr);
		u32 i;

		if (!netif_running(ndev))
			return -EINVAL;

		for (i = 0; i < privp->port.count; i++)
			if (mii_data->phy_id == privp->port.info[i].phy_id)
				return phy_mii_ioctl(
				 privp->mii_bus->phy_map[mii_data->phy_id],
				 ifr, cmd);

		return -ENODEV;
	}
	break;

	case SIOCESW_ENTER_WOL:
	{
		struct esw_wol_data wol_info;
		int i;

		/* We don't support WOL in switch by pass mode */
		if (!privp->switchmode) {
			netdev_warn(ndev,
				"ioctl err: wol not supported in switch bypass mode\n");
			return -EOPNOTSUPP;
		}

		rc = copy_from_user(&wol_info,
				ifr->ifr_data,
				sizeof(wol_info));
		if (rc) {
			netdev_warn(ndev, "ioctl enter wol err %d\n", rc);
			rc = -EFAULT;
			break;
		}

		rc = -ENODEV;
		/* Check if port is valid */
		for (i = 0; i <= privp->port.count; i++)
			if (wol_info.port == privp->port.info[i].num)
				rc = 0;

		if (rc) {
			netdev_warn(ndev,
				"ioctl enter wol, invalid port %d\n",
				wol_info.port);
			break;
		}

		/* Enter WOL */
		bcm_amac_gphy_enter_wol(privp,
			wol_info.port,
			wol_info.port_speed);
	}
	break;

	case SIOCESW_EXIT_WOL:
		/* We don't support WOL in switch by pass mode */
		if (!privp->switchmode) {
			netdev_warn(ndev,
				"ioctl err: wol not supported in switch bypass mode\n");
			return -EOPNOTSUPP;
		}

		/* Exit WOL mode */
		bcm_amac_gphy_exit_wol(privp);
		rc = 0;
		break;

	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
}

static void amac_enet_send_netlink_msg(struct bcm_amac_priv *privp,
	char *msg)
{
	unsigned int len;
	unsigned int size;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int err;

	/* Prepare and send broadcast message via netlink */
	if (!privp->nl_sk)
		return;

	len = (strlen(msg) + 1);
	size = NLMSG_SPACE(len);

	skb = alloc_skb(size, GFP_KERNEL);
	if (!skb)
		return;

	nlh = __nlmsg_put(skb, KERNEL_PID,
				privp->nl_seq,
				NLMSG_DONE,
				size - sizeof(*nlh),
				0);
	privp->nl_seq++;

	memcpy(NLMSG_DATA(nlh), msg, len);
	NETLINK_CB(skb).portid = KERNEL_PID;  /* from kernel */
	NETLINK_CB(skb).dst_group = DST_GROUP;

	/* multicast the message to all listening processes
	 * Note: error handling is done inside 'netlink_broadcast',
	 * so no need to call free skb in case of an error
	 */
	err = netlink_broadcast(privp->nl_sk, skb,
			KERNEL_PID, DST_GROUP, GFP_KERNEL);
	if (err == ENOBUFS)
		netdev_alert(privp->ndev,
			"netlink_broadcast() failed, No buffer\n");
}

/**
 * bcm_amac_enet_send_link_stat() - send netlink message to user space
 * @privp: driver info pointer
 * @port_idx: index in of the port in privp
 * @phydev: phy device to report the link change
 * @link: current link status
 *
 * The function creates and sends a custome netlink message to the user
 * space application to indicate a change in the n/w link status, speed
 * or duplex.
 * The API is called from the link change handler.
 */
void bcm_amac_enet_send_link_stat(struct bcm_amac_priv *privp,
	unsigned int port_idx, struct phy_device *phydev, unsigned int link)
{
	char msg[NETLINK_MAX_PAYLOAD];

	if (link)
		snprintf(msg, NETLINK_MAX_PAYLOAD,
			LINK_UP_STR,
			((privp->port.info[port_idx].type == 0) ? "LAN" : "PC"),
			privp->port.info[port_idx].num,
			phydev->addr,
			phydev->speed,
			phydev->duplex ? "full" : "half");
	else
		snprintf(msg, NETLINK_MAX_PAYLOAD,
			LINK_DOWN_STR,
			((privp->port.info[port_idx].type == 0) ? "LAN" : "PC"),
			privp->port.info[port_idx].num,
			phydev->addr);

	netif_info(privp, link, privp->ndev, msg);

	amac_enet_send_netlink_msg(privp, msg);
}

static void amac_enet_pwrstat_update(struct bcm_amac_priv *privp,
	int suspend)
{
	char msg[NETLINK_MAX_PAYLOAD];

	if (suspend)
		snprintf(msg, NETLINK_MAX_PAYLOAD,
			"AMAC Driver: Suspend\n");
	else
		snprintf(msg, NETLINK_MAX_PAYLOAD,
			"AMAC Driver: Resume\n");

	amac_enet_send_netlink_msg(privp, msg);
}

/**
 * bcm_amac_get_dt_data() - Retrieve data from the device tree
 * @pdev: platform device data structure
 * @privp: driver privagte data structure
 *
 * Returns: '0' or error
 */
static int
bcm_amac_get_dt_data(struct platform_device *pdev, struct bcm_amac_priv *privp)
{
	struct resource *iomem;
	u32 switchmode = 0, port_val, tag_support;
	u32 port_num;
	struct device_node *child_bus_node;
	int rc;

	/* GMAC Core register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "core_base");
	privp->hw.reg.amac_core = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_core)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_core failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_core);
	}

	/* AMAC IO Ctrl register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "amac_io_ctrl");
	privp->hw.reg.amac_io_ctrl = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_io_ctrl)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_io_ctrl failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_io_ctrl);
	}

	/* AMAC IDM RESET register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "amac_idm_reset");
	privp->hw.reg.amac_idm_reset = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_idm_reset)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_idm_reset failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_idm_reset);
	}

	/* ICFG register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "icfg");
	privp->hw.reg.icfg_regs = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.icfg_regs)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of icfg_regs failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.icfg_regs);
	}

	/* SWITCH GLOBAL CONFIG register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "switch_global_base");
	privp->hw.reg.switch_global_cfg =
		devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.switch_global_cfg)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of switch_global_cfg reg failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.switch_global_cfg);
	}

	/* CRMU IO PAD CTRL register */
	iomem = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "crmu_io_pad_ctrl");
	privp->hw.reg.crmu_io_pad_ctrl =
		devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.crmu_io_pad_ctrl)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of crmu_io_pad_ctrl failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.crmu_io_pad_ctrl);
	}

	/* Read Interrupt */
	privp->hw.intr_num = platform_get_irq(pdev, 0);
	if (privp->hw.intr_num == 0) {
		dev_err(&privp->pdev->dev,
			"%s: gmac0 interrupt not specified\n",
			__func__);
		return -EINVAL;
	}

	/* Read switch mode */
	rc = of_property_read_u32(pdev->dev.of_node,
		"switchmode", &switchmode);
	if ((rc != 0) || (switchmode > 1)) {
		dev_err(&privp->pdev->dev,
			"%s: Default Switch mode: enabled\n",
			__func__);
		switchmode = 1;
	}
	privp->switchmode = switchmode;

	/* Read SRAB base address if switch is enabled */
	if (privp->switchmode) {
		/* SRAB base address */
		iomem = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "srab_base");
		privp->esw.srab_base = devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->esw.srab_base)) {
			dev_err(&privp->pdev->dev,
				"%s: ioremap of SRAB BASE failed\n",
				__func__);
			return PTR_ERR(privp->esw.srab_base);
		}
	}

	/* Read tag support if switch is enabled */
	if (privp->switchmode) {
		rc = of_property_read_u32(pdev->dev.of_node,
			"tag_support", &tag_support);
		if (rc != 0)
			privp->brcm_tag = AMAC_TAG_NONE;
		else
			privp->brcm_tag = (enum tag_support)tag_support;
	} else
		privp->brcm_tag = AMAC_TAG_NONE; /* Switch by pass mode */


	/* Read Port Info */
	privp->port.count = 0;
	rc = of_get_child_count(pdev->dev.of_node);
	if ((rc < 1) || (rc > 2)) {
		dev_err(&privp->pdev->dev,
			"%s: Invalid port(s) defined\n", __func__);
		return -EINVAL;
	}

	for_each_available_child_of_node(pdev->dev.of_node, child_bus_node) {

		if (privp->port.count > AMAC_MAX_PORTS) {
			dev_err(&privp->pdev->dev,
				"%s: too many ports specified\n", __func__);
			return -EINVAL;
		}

		port_num = privp->port.count;

		/* Read Port number */
		rc = of_property_read_u32(child_bus_node, "port-id", &port_val);
		if (rc != 0) {
			dev_err(&privp->pdev->dev,
				"%s: invalid port-id\n", __func__);
			return -EINVAL;
		}
		privp->port.info[port_num].num = port_val;

		/* Read Port type */
		rc = of_property_read_u32(child_bus_node,
				"port-type", &port_val);
		if ((rc != 0) || (port_val > (u32)AMAC_PORT_TYPE_MAX)) {
			dev_err(&privp->pdev->dev,
				"%s: invalid port-type\n", __func__);
			return -EINVAL;
		}
		privp->port.info[port_num].type =
			(enum port_type)port_val;

		/* Read PHY ID for the port */
		rc = of_property_read_u32(child_bus_node, "phy-id", &port_val);
		if (rc != 0) {
			dev_err(&privp->pdev->dev,
				"%s: invalid phy-id\n", __func__);
			return -EINVAL;
		}
		privp->port.info[port_num].phy_id =
			port_val;

		rc = of_property_read_u32(child_bus_node, "speed", &port_val);
		if (rc != 0) {
			dev_info(&privp->pdev->dev,
				"amac-enet: using default speed (1G) for port %d\n",
				privp->port.info[port_num].num);
			privp->port.info[port_num].phy_def.speed =
				AMAC_PORT_DEFAULT_SPEED;
		} else {
			if ((port_val == SPEED_1000) || (port_val == SPEED_100))
				privp->port.info[port_num].phy_def.speed =
					port_val;
			else {
				dev_err(&privp->pdev->dev,
					"%s: Invalid eth port speed specified\n",
					__func__);
				return -EINVAL;
			}
		}

		/* Configure default port params */
		privp->port.info[privp->port.count].phy_def.aneg =
			AMAC_PORT_DEFAULT_ANEG;
		privp->port.info[privp->port.count].phy_def.duplex =
			AMAC_PORT_DEFAULT_DUPLEX;
		privp->port.info[privp->port.count].phy_def.link = 0;
		privp->port.info[privp->port.count].phy_info.link = 0;

		/* PAUSE Frames are disabled for LAN ports */
		if (privp->port.info[privp->port.count].type ==
			AMAC_PORT_TYPE_LAN)
			privp->port.info[privp->port.count].phy_def.pause =
				AMAC_PORT_PAUSE_DISABLE;
		else
			privp->port.info[privp->port.count].phy_def.pause =
				AMAC_PORT_PAUSE_ENABLE;

		privp->port.count++;
	}

	return 0;
}

/**
 * amac_clear_stats() - Clear sysfs statistics
 * @stats: sys stat variable pointer
 */
static void amac_clear_stats(struct sysctl_ethstats *stats)
{
	stats->rx_bytes = 0;
	stats->rx_dropped_pkts = 0;
	stats->rx_resyncs = 0;
	stats->rx_wraparounds = 0;
	stats->rx_syncchecked = 0;
	stats->rx_syncdroppedpkts = 0;
	stats->rx_noskb = 0;
	stats->rx_broadcast = 0;
	stats->rx_multicast = 0;
	stats->rx_unicast = 0;
	stats->tx_broadcast = 0;
	stats->tx_multicast = 0;
	stats->tx_unicast = 0;
	stats->tx_dropped_pkts = 0;
}

/**
 * bcm_amac_enet_probe() - driver probe function
 * @pdev: platform device pointer
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct bcm_amac_priv *privp;
	int rc;

	if (!pdev) {
		pr_err("%s: platform device is NULL\n", __func__);
		return -EINVAL;
	}

	/* Check for DT node */
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev,
			"%s: platform data / DT not available\n", __func__);
		return -EINVAL;
	}

	/* Initialize driver resource */
	ndev = alloc_etherdev(sizeof(struct bcm_amac_priv));
	if (ndev == NULL) {
		dev_err(&pdev->dev,
			"%s: Failed to allocate device\n", __func__);
		return -ENOMEM;
	}

	privp = netdev_priv(ndev);
	memset(privp, 0, sizeof(struct bcm_amac_priv));
	privp->pdev = pdev;
	privp->ndev = ndev;

	/* Read DT data */
	rc = bcm_amac_get_dt_data(pdev, privp);
	if (rc != 0) {
		dev_err(&pdev->dev,
			"%s: Failed to get platform data\n", __func__);
		goto amac_err_plat_data;
	}

	/* Clear sysfs stats */
	amac_clear_stats(&privp->eth_stats);

	spin_lock_init(&privp->lock);
	mutex_init(&privp->port.wol_lock);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ndev->netdev_ops = &bcm_amac_enet_ops;
	ndev->watchdog_timeo = TX_TIMEOUT;
	ndev->ethtool_ops = &bcm_amac_ethtool_ops;

	netif_napi_add(ndev, &privp->napi, bcm_amac_enet_rx_poll, QUOTA);

	/* increase ndev->hard_header_len to account for the
	 * Broadcom Header and the possible customized VLAN
	 * tag
	 */
	ndev->hard_header_len += sizeof(u32) + VLAN_HLEN;
	ndev->features &= ~(NETIF_F_SG | NETIF_F_FRAGLIST);
	ndev->tx_queue_len = DMA_TX_MAX_QUEUE_LEN;

	/* Get the reset reason, this is to decide the level
	 * of 'init' to be performed.
	 * If switch is not enabled, it is always cold reboot.
	 */
	if (privp->switchmode)
		privp->reboot = bcm_esw_get_reboot(privp);
	else
		privp->reboot = AMAC_REBOOT_COLD;

	/* Start ethernet block */
	rc = bcm_amac_enet_start(privp);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Failed to start ethernet block\n", __func__);
		goto amac_err_plat_data;
	}

	/* Reset the reboot reason, so the suspend/resume
	 * path is not affected
	 */
	privp->reboot = AMAC_REBOOT_COLD;

	/* Clear stats */
	memset(&ndev->stats, 0, sizeof(ndev->stats));

	tasklet_init(&privp->tx_tasklet, amac_tx_task, (unsigned long)privp);

	rc = register_netdev(ndev);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: netdev register failed\n", __func__);
		goto amac_err_stop_eth;
	}

	rc = register_reboot_notifier(&bcm_amac_enet_reboot_notifier);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: register_reboot_notifier failed\n", __func__);
		goto amac_err_unregister;
	}

	/* create netlink socket to send link notifications */
	privp->nl_sk = netlink_kernel_create(&init_net,
		NETLINK_USERSOCK, (struct netlink_kernel_cfg *)NULL);

	netdev_info(ndev, "NETLINK_USERSOCK create: %s!\n",
		privp->nl_sk ? "ok" : "failed");

	return 0;

amac_err_unregister:
	unregister_netdev(ndev);

amac_err_stop_eth:
	bcm_amac_enet_stop(privp);

amac_err_plat_data:
	/* unregister_netdevice(ndev); */
	free_netdev(ndev);

	return rc;
}

/**
 * bcm_amac_enet_shutdown() - driver probe function
 * @pdev: platform device pointer
 *
 * Returns: none
 */
static void bcm_amac_enet_shutdown(struct platform_device *pdev)
{
	(void)pdev;
}

/**
 * bcm_amac_notify_reboot() - reboot handler
 * @nb: notifier block pointer
 * @event: type of reboot
 * @ptr: unused
 *
 * Returns: none
 */
static int bcm_amac_notify_reboot(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct bcm_amac_priv *privp;
	struct net_device *ndev;

	(void)nb;
	(void)ptr;

	switch (event) {
	case SYS_DOWN:
	case SYS_HALT:
	case SYS_POWER_OFF:
		/* Find the network device to get the driver data.
		 * The reboot notifier doesn't provide the driver priv
		 * data. So this is the only way of getting the driver's
		 * priv data.
		 * In Cygnus we have only 'eth0', the logic will need
		 * to be re-visited in SoC's with multiple MAC
		 * blocks (multiple net interfaces)
		 */
		ndev = dev_get_by_name(&init_net, "eth0");
		if (!ndev)
			return NOTIFY_DONE;

		privp = netdev_priv(ndev);
		bcm_amac_gphy_shutdown(privp);
		break;

	/* Don't do anything in case of warm reboot */
	case SYS_RESTART_WARM:
	default:
		break;
	}

	return NOTIFY_DONE;
}

/**
 * bcm_amac_enet_remove() - interface remove callback
 * @pdev: platform data structure pointer
 *
 * Returns: 0
 */
static int bcm_amac_enet_remove(struct platform_device *pdev)
{
	struct bcm_amac_priv *privp;

	privp = netdev_priv((struct net_device *)&pdev->dev);

	if (privp->nl_sk) {
		netlink_kernel_release(privp->nl_sk);
		privp->nl_sk = NULL;
		netdev_info(privp->ndev, "netlink released\n");
	}

	unregister_reboot_notifier(&bcm_amac_enet_reboot_notifier);

	if (privp->ndev)
		free_netdev(privp->ndev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/**
 * bcm_amac_enet_suspend() - interface suspend callback
 * @dev: device data structure pointer
 *
 * Suspends the Ethernet interface by disabling dma, interrupt etc.
 *
 * Returns: 0
 */
static int bcm_amac_enet_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	netdev_info(ndev, "Suspending AMAC driver\n");

	amac_enet_pwrstat_update(privp, 1);

	if (netif_running(ndev)) {
		/* Stop TX queues */
		netif_stop_queue(ndev);

		/* Wait for TX FIFO to drain */
		while (kfifo_len(&privp->dma.txfifo) != 0)
			;

		/* Stop the DMA */
		bcm_amac_dma_stop(privp);

		/* Disable RX NAPI */
		napi_disable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_detach(ndev);
		netif_tx_unlock(ndev);
	}

	/* Stop PHY's */
	bcm_amac_gphy_stop_phy(privp);

	return 0;
}

/**
 * bcm_amac_enet_resume() - interface resume callback
 * @dev: device data structure pointer
 *
 * Resumes the Ethernet interface from sleep or deepsleep. Restores device
 * settings, dma, interrupts.
 *
 * Returns: 0 or error number
 */
static int bcm_amac_enet_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;

	netdev_info(ndev, "Resuming AMAC driver\n");

	/* Since we dont have h/w support for register retention,
	 * initialize everything.
	 */
	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		netdev_err(ndev, "core init failed\n");
		return rc;
	}

	/* Add the mac to ARL */
	bcm_amac_set_mac(privp, privp->cur_etheraddr.sa_data);

	if (netif_running(ndev)) {
		/* Start DMA */
		rc = bcm_amac_dma_start(privp);
		if (rc) {
			netdev_err(ndev, "Failed to start DMA\n");
			goto err_amac_resume;
		}

		bcm_amac_core_enable(privp, 1);

		/* Power up the PHY */
		bcm_amac_gphy_resume(privp);

		napi_enable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_attach(ndev);
		netif_tx_unlock(ndev);

		netif_start_queue(ndev);
	}

	amac_enet_pwrstat_update(privp, 0);

	return 0;

err_amac_resume:
	/* Stop PHY's */
	bcm_amac_gphy_stop_phy(privp);

	return rc;
}

static const struct dev_pm_ops amac_enet_pm_ops = {
	.suspend = bcm_amac_enet_suspend,
	.resume = bcm_amac_enet_resume
};
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id bcm_amac_of_enet_match[] = {
	{.compatible = "brcm,amac-enet",},
	{},
};

MODULE_DEVICE_TABLE(of, bcm_amac_of_enet_match);

static struct platform_driver bcm_amac_enet_driver = {
	.driver = {
		.name  = "amac-enet",
		.of_match_table = bcm_amac_of_enet_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &amac_enet_pm_ops,
#endif
	},
	.probe		= bcm_amac_enet_probe,
	.remove		= bcm_amac_enet_remove,
	.shutdown	= bcm_amac_enet_shutdown,
};

module_platform_driver(bcm_amac_enet_driver)

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom AMAC Ethernet Driver");
MODULE_LICENSE("GPL v2");
