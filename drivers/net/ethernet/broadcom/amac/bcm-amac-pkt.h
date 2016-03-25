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

#ifndef __BCM_AMAC_PKT_H__
#define __BCM_AMAC_PKT_H__

#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/etherdevice.h>
#include <net/sock.h>


/* BRCM Header / TAG is is a special field used to dictate the
 * packet forwarding in the switch. It is 4 bytes and is inserted
 * between the MAC address and the Ethertype in the packet.
 *
 * Below are the tag formats
 * To CPU:
 * [31 - 29]  [28 - 24]  [23 - 16]  [15 - 8]  [7 - 5]  [4 - 0]
 * OPCODE=0    Reserved  Class ID   Reason    TC       port_id
 *
 * From CPU:
 * To let the switch make the forwarding decision
 * [31 - 29]  [28 - 26]  [25 - 24]  [23 - 0]
 * OPCODE=0    TC        TE
 *
 * To specify the forwarding rules.
 * [31 - 29]  [28 - 26]  [25 - 24]  [23]  [22 - 0]
 * OPCODE=1    TC        TE         TS    DST_MAP
 *
 * OPCODE - Indicates CPU is dictating the packet forwarding
 * TC - Traffic class
 * TE - tag enforcement
 * DST_MAP = Port bit map Bits[5:0] = Port [5:0],
 *           Bit 8 = IMP port
 * port_id - The port the packet has been received.
 * Reason - indicates the reason why the packet is forwarded
 *          Bit [0] indicates mirroring
 *          Bit [1] indicates SA learning
 *          Bit [2] indicates switching
 *          Bit [3] indicates protocol termination
 *          Bit [4] indicates protocol snooping
 *          Bit [5] indicates flooding/exception processing
 *          Bit [6] and bit [7] are reserved
 */
#define BRCM_TAG_LEN           4

/* If an app wants to dictate packet forwarding, it needs to
 * send the frame with the BRCM header pre-defined.
 *  BRCM_TAG[0]:  ETHHW_BRCM_TYPE_MSB
 *  BRCM_TAG[1]:  ETHHW_BRCM_TYPE_LSB
 *  BRCM_TAG[2]:  tc[4:2], te[1:0]
 *  BRCM_TAG[3]:  port_mask
 */
#define ETH_BRCM_TAG_HDR       0x8874

#define ETH_TC_MASK            0x1C
#define ETH_TC_SHIFT           2
#define ETH_TC_MAX             7

#define ETH_TC_DEFAULT         0x1C /* (ETH_TC_MAX << ETH_TC_SHIFT) */

#define BCM_OPCODE_SHIFT       0x05
#define BCM_SET_OPCODE         0x20 /* ( 1 << BCM_OPCODE_SHIFT) */

#define PKT_ETH_ADDR_LEN        12 /* (2 * ETH_ALEN) */

#define BRCM_TAG_OFFSET_CUSTOM  PKT_ETH_ADDR_LEN
#define BRCM_TAG_OFFSET         8  /* (2 * ETH_ALEN) - BRCM_TAG_LEN = 12 - 4 */

/* Position of last byte of the src mac address in the packet */
#define SRC_MAC_ADDR_LAST_BYTE  11

#define ETH_P_LLDP              0x88CC
#define ETH_P_EAP               0x888E

#define ETHER_TYPE_LOC          PKT_ETH_ADDR_LEN
#define ETHER_TYPE_LOC_BRCM_TAG (PKT_ETH_ADDR_LEN + BRCM_TAG_LEN)

/* check if the buffer contains an EAP DST MAC address */
#define IS_CDP(bufp)	((bufp[0] == 0x01) && (bufp[1] == 0x00) && \
				(bufp[2] == 0x0c) && (bufp[3] == 0xcc) && \
				(bufp[4] == 0xcc) && (bufp[5] == 0xcc))


/**
 * bcm_amac_pkt_add_bcm_tag() - Add the BRCM tag
 * @skbp - Pointer to the SKB pointer to be modified
 *
 * Returns - 0 or error code
 */
static inline int bcm_amac_pkt_add_bcm_tag(struct sk_buff **skbp)
{
	u16 *tag_type;
	u8 *bcm_tag;

	tag_type = (u16 *)&((*skbp)->data[PKT_ETH_ADDR_LEN]);
	/* Check if packets contains the BRCM header/tag
	 * These are special packets, re-format them.
	 */
	if (htons(*tag_type) != ETH_BRCM_TAG_HDR) {
		/* Tag is inserted between the Src MAC and EType */
		if (skb_cow_head(*skbp, BRCM_TAG_LEN) < 0)
			return -ENOMEM;

		/* Copy the MACs */
		memmove((*skbp)->data - BRCM_TAG_LEN,
			(*skbp)->data,
			2 * ETH_ALEN);

		bcm_tag = (*skbp)->data + BRCM_TAG_OFFSET;

		/* Setup BRCM TAG
		 *  - tag enforcement is depreciated
		 *  - Let the switch make the decision
		 *  - use max for tc
		 */
		bcm_tag[0] = ETH_TC_DEFAULT; /* OPCODE = 0, TC = 0x07, TE = 0 */
		bcm_tag[1] = 0;
		bcm_tag[2] = 0;
		bcm_tag[3] = 0;

		skb_push(*skbp, BRCM_TAG_LEN);

	} else {
		/* Special pre-tagged packets, they need to be reformated
		 *  *bufp in:
		 *     byte  0    5 6    11  12     15   16  19
		 *            [DA]   [SA]    [BRCM_TAG]  [ETHER TYPE]
		 *
		 *  Custom L2 protocol type detected, raw socket has
		 *  inserted egress specific information, but type needs
		 *  to be formatted slightly for ASIC.
		 *  The portMask, tc, and te arguments will be overridden
		 *  since the RAW_HDR already contains this information
		 *  This is how the tag has been filled.
		 *
		 *  BRCM_TAG[0]:  ETHHW_BRCM_TYPE_MSB
		 *  BRCM_TAG[1]:  ETHHW_BRCM_TYPE_LSB
		 *  BRCM_TAG[2]:  tc[4:2], te[1:0]
		 *  BRCM_TAG[3]:  port_mask
		 */
		bcm_tag = (*skbp)->data + BRCM_TAG_OFFSET_CUSTOM;

		/* If port mask is to be used then BRCM_OPCODE
		 * needs to be set.
		 */
		if (bcm_tag[3])
			bcm_tag[0] = BCM_SET_OPCODE | bcm_tag[2];
		else
			bcm_tag[0] = bcm_tag[2];

		bcm_tag[1] = 0;
		bcm_tag[2] = 0;

	}

	return 0;
}

/**
 * bcm_amac_pkt_rm_brcm_tag() - Remove the BRCM tag
 * @skbp: Pointer to the SKB pointer to be modified
 *
 * Returns: number of bytes removed
 */
static inline int bcm_amac_pkt_rm_brcm_tag(struct sk_buff **skbp)
{
	u8 *src, *dest;
	int i;

	/* Move the Ethernet DA and SA */
	src = (*skbp)->data + SRC_MAC_ADDR_LAST_BYTE;
	dest = src + BRCM_TAG_LEN;

	for (i = 0; i < PKT_ETH_ADDR_LEN; i++)
		*dest-- = *src--;

	/* Adjust SKB */
	skb_reserve(*skbp, BRCM_TAG_LEN);

	return BRCM_TAG_LEN;
}

#endif /*__BCM_AMAC_PKT_H__*/
