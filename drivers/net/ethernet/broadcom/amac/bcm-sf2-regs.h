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

#ifndef __BCM_SF2_REGS_H__
#define __BCM_SF2_REGS_H__

#include <linux/types.h>

/* Page numbers */
#define PAGE_CTRL      0x00 /* Control page */
#define PAGE_STATUS    0x01 /* Status page */
#define PAGE_MMR       0x02 /* 5397 Management/Mirroring page */
#define PAGE_MPORT     0x04 /* MPORT page */
#define PAGE_ARLACCS   0x05 /* ARL Access page */
#define PAGE_MIB_PORT0 0x20 /* Port 0 MiB page */
#define PAGE_MIB_PORT1 0x21 /* Port 1 MiB page */
#define PAGE_MIB_IMP   0x28 /* Port 8 MiB page */
#define PAGE_VLAN      0x34 /* VLAN page */
#define PAGE_BPM       0x73 /* BPM page */
#define PAGE_IOCTRL    0xe4 /* IO Port Informantion Control page */
#define PAGE_EEE       0x92 /* EEE Control Registers */

/* Command and status register of the SRAB */
#define CFG_F_SRA_RST_MASK (1 << 2)

/* Switch interface controls */
#define CFG_F_SW_INIT_DONE_MASK (1 << 6)

#define REG_SRAB_CMDSTAT   0x2C
#define REG_SRAB_WDH       0x30
#define REG_SRAB_WDL       0x34
#define REG_SRAB_RDH       0x38
#define REG_SRAB_RDL       0x3C

#define SRAB_READY_MASK    0x1
#define SRAB_WRITE_MASK    0x2

#define SRAB_PAGE_SHIFT    24
#define SRAB_OFFSET_SHIFT  16

#define SRAB_MAX_RETRY     100

#define SWITCH_RX_PAUSE_CAP_OFFSET 9

/* Control page registers */
#define REG_CTRL_PORT0      0x00 /* Port 0 traffic control register */
#define REG_CTRL_PORT1      0x01 /* Port 1 traffic control register */
#define REG_CTRL_PORT5      0x05 /* Port 5 traffic control register */
#define REG_CTRL_IMP        0x08 /* IMP port traffic control register */
#define REG_CTRL_MODE       0x0B /* Switch Mode register */
#define REG_CTRL_MIIPO      0x0E /* 5325: MII Port Override register */
#define REG_CTRL_PWRDOWN    0x0F /* 5325: Power Down Mode register */
#define REG_CTRL_SWCTL      0x22 /* Switch control register */
#define REG_CTRL_PPORT      0x24 /* Protected port register */
#define REG_CTRL_P5RGMIICTL 0x2A /* Port5 RGMII Control register */
#define REG_CTRL_MDIO_DA    0x6f /* MDIO Direct Access Enable register */

#define REG_MGMT_CFG        0x00 /* Global Management Configuration */
#define REG_BRCM_HDR        0x03 /* BRCM Header Control */
#define REG_DEVICE_ID       0x30 /* 539x Device id: */
#define REG_VERSION_ID      0x40

/* MPORT page registers */
#define REG_MPORT_CTRL         0x0E /* mport control */
#define REG_MPORT_ADDR0        0x10 /* Control 0 address register */
#define REG_MPORT_VCTR0        0x18 /* Control 0 vector resigter */
#define NEXT_MPORT_REG_OFFSET  0x10 /* Offset to next MPORT reg */
#define MPORT_E_TYPE_SHIFT     48

/* ARLACCS page registers */
#define REG_ARLA_RWCTL              0x00 /* ARL Read/Write Control Register */
#define REG_ARLA_MAC                0x02 /* MAC Address Index Register */
#define REG_ARLA_VID                0x08 /* VID Index Register */
#define REG_ARLA_MACVID_ENTRY0      0x10 /* ARL MAC/VID Entry 0 Register */
#define REG_ARLA_FWD_ENTRY0         0x18 /* ARL FWD Entry 0 Register */
#define NEXT_ARLA_ENTRY_OFFSET      0x10 /* Offset to next ARLA Entry reg */
#define REG_ARLA_MACVID_VID_OFFSET  48

#define REG_ARLA_FWDENTRY_VALID     0x00010000
#define REG_ARLA_FWDENTRY_STATIC    0x00008000
#define REG_ARLA_FWDENTRY_AGE       0x00004000
#define REG_ARLA_FWDENTRY_TC_SHIFT  11

#define REG_ARLA_RWCTRL_START_DONE  0x80
#define REG_ARLA_RWCTRL_READ        0x01
#define REG_ARLA_RWCTRL_WRITE       0x00

#define REG_ARL_TIMEOUT             1000

/* Max number of ARL entry entries in one ARL bucket */
#define REG_ARLA_ENTRY_MAX 4

/* VLAN page registers */
#define REG_VLAN_CTRL5     0x06 /* VLAN Control 5 register */

/* IO Port Informantion Control page registers*/
#define REG_IOCTRL_PORT5   0x0a /* Port 5 traffic control register */
#define REG_IOCTRL_PORTIMP 0x10 /* Port 8 traffic control register */

/* IMP control bits */
#define IMP_C_RX_UCST_EN 16
#define IMP_C_RX_MCST_EN 8
#define IMP_C_RX_BCST_EN 4
#define IMP_C_TX_DIS     2
#define IMP_C_RX_DIS     0

/* MiB page registers */
#define REG_TX_OCTETS                 0x00
#define REG_TX_DROP_PKTS              0x08
#define REG_TX_BROADCAST_PKTS         0x10
#define REG_TX_MULTICAST_PKTS         0x14
#define REG_TX_UNICAST_PKTS           0x18
#define REG_TX_COLLISIONS             0x1c
#define REG_TX_SINGLE_COLLISION       0x20
#define REG_TX_MULTIPLE_COLLISION     0x24
#define REG_TX_DEFERRED_TXMIT         0x28
#define REG_TX_LATE_COLLISION         0x2c
#define REG_TX_EXCESSIVE_COLLISION    0x30
#define REG_TX_FRAME_IN_DISC          0x34
#define REG_TX_PAUSE_PKTS             0x38
#define REG_RX_OCTETS                 0x50
#define REG_RX_UNDERSIZE_PKTS         0x58
#define REG_RX_PAUSE_PKTS             0x5c
#define REG_RX_PKTS64OCTETS           0x60
#define REG_RX_PKTS65TO127OCTETS      0x64
#define REG_RX_PKTS128TO255OCTETS     0x68
#define REG_RX_PKTS256TO511OCTETS     0x6c
#define REG_RX_PKTS512TO1023OCTETS    0x70
#define REG_RX_PKTS1024TOMAXPKTOCTETS 0x74
#define REG_RX_OVERSIZE_PKTS          0x78
#define REG_RX_JABBERS                0x7c
#define REG_RX_ALIGNMENT_ERRORS       0x80
#define REG_RX_FCS_ERRORS             0x84
#define REG_RX_GOOD_OCTETS            0x88
#define REG_RX_DROP_PKTS              0x90
#define REG_RX_UNICAST_PKTS           0x94
#define REG_RX_MULTICAST_PKTS         0x98
#define REG_RX_BROADCAST_PKTS         0x9c
#define REG_RX_SA_CHANGES             0xa0
#define REG_RX_FRAGMENTS              0xa4
#define REG_RX_JUMBO_PKTCOUNT         0xa8
#define REG_RX_SYMBOL_ERROR           0xac
#define REG_RX_DISCARD                0xc0

/* BPM Register */
#define REG_BPM_SPARE0                0x60

/* EEE Control Registers */
#define REG_EEE_EN_CTRL               0x00

#endif /*__BCM_SF2_REGS_H__ */
