/*
 * Copyright 2014 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 */


/* ---- Include Files ---------------------------------------------------- */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/sysctl.h>
#include <linux/io.h>
#include <linux/ctype.h>

#include "bcm-amac-regs.h"
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"
#include "bcm-robo.h"
#include "bcm-amac-dbg.h"




/* Page numbers */
#define PAGE_CTRL	0x00	/* Control page */
#define PAGE_MPORT	0x04    /* MPORT page */

/* MPORT page registers */
#define REG_MPORT_CTRL         0x0E /* mport control */
#define REG_MPORT_ADDR0        0x10 /* Control 0 address register */
#define REG_MPORT_VCTR0        0x18 /* Control 0 vector resigter */
#define NEXT_MPORT_REG_OFFSET  0x10 /* Offset to next MPORT reg */
#define NEXT_MPORT_REG_OFFSET  0x10 /* Offset to next MPORT reg */





struct amac_reg_bit_field {
	int val;
	char desc[10];
};

struct amac_reg_dbg_info {
	int hi_bit; /* Higher bit */
	int lo_bit; /* -1 if only 1 bit */
	char info[30];
	struct amac_reg_bit_field bit_field[3];
};


/*************************************************/
/* GMAC CORE */
struct amac_reg_dbg_info gmac0_intmask[] = {
	{27, -1, "XMTINTEN_3", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{26, -1, "XMTINTEN_2", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{25, -1, "XMTINTEN_1", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{24, -1, "XMTINTEN_0", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{16, -1, "RCVINTEN", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_intstat[] = {
	{27, -1, "XMTINT_3", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{26, -1, "XMTINT_2", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{25, -1, "XMTINT_1", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{24, -1, "XMTINT_0", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{16, -1, "RCVINT", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

/*************************************************/
/* RX DMA */
struct amac_reg_dbg_info gmac0_rcvctrl[] = {
	{25, 24, "PREFETCHTHRESH", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{23, 22, "PREFETCHCTL", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{20, 18, "BURSTLEN", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{17, 16, "ADDREXT", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{13, -1, "SELECTBUFFERACTIVEINDEX", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{12, -1, "WAITFORCOMPLETE", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{11, -1, "RXPARITYCHECKDISABLE", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{10, -1, "OFLOWCONTINUE", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 9, -1, "SEPRXHDRDESCEN", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 7,  1, "RCVOFFSET", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 0, -1, "REVEN", {{1, "ENABLED"}, {0, "DISABLED"}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_rcvptr[] = {
	{12, 0, "RCVPTR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_rcvlowaddr[] = {
	{31, 4, "RCVADDR_LOW", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_rcvstatus0[] = {
	{31, 28, "RCVSTATE", {{D64_RS0_RS_IDLE, "IDLE"},
	{D64_RS0_RS_STOPPED, "STOPPED"}, {D64_RS0_RS_SUSP, "SUSPENDED"} } },
	{12, 0, "CURRDSCR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_rcvstatus1[] = {
	{31, 28, "RCVERR", {{D64_RS1_RE_DPO, "DESCP_PROT"},
		{D64_RS1_RE_DTE, "DATA_TX"}, {D64_RS1_RE_DESRE, "DESC_RD"} } },
	{12, 0,  "ACTIVEDESCR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};


/*************************************************/
/* TX DMA */
struct amac_reg_dbg_info gmac0_txctrl[] = {
	{25, 24, "PREFETCHTHRESH", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{23, 22, "PREFETCHCTL", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{20, 18, "BURSTLEN", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{17, 16, "ADDREXT", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{13, -1, "SELECTBUFFERACTIVEINDEX", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{11, -1, "TXPARITYCHECKDISABLE", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 7, 6,  "MULTIPLEOUTSTANDINGREADS", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 5, -1, "BURSTALIGNEN", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 2, -1, "DMA_LOOPBACK_MODE", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 1, -1, "TXSUSPEND", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{ 0, -1, "XMTEN", {{1, "ENABLED"}, {0, "DISABLED"}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_txptr[] = {
	{12, 0, "LASTDSCR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_txlowaddr[] = {
	{31, 4, "XMTADDR_LOW", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_txstatus0[] = {
	{31, 28, "XMTSTATE", {{D64_XS0_XS_IDLE, "IDLE"},
		{D64_XS0_XS_STOPPED, "STOPPED"}, {D64_XS0_XS_SUSP, "SUSP"} } },
	{12, 0, "CURRDSCR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_txstatus1[] = {
	{31, 28, "XMTERR", {{D64_XS1_XE_DPE, "DESC_PROT"},
		{D64_XS1_XE_DESRE, "DESC_RD"}, {D64_XS1_XE_DTE, "DATA_ERR"} } },
	{12, 0,  "ACTIVEDESCR", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};

struct amac_reg_dbg_info gmac0_txgdpkts[] = {
	{31, 0, "TX_GD_PKTS", {{-1, ""}, {-1, ""}, {-1, ""} } },
	{0xFF, -1, "", {{-1, ""}, {-1, ""}, {-1, ""} } }
};


static void dbg_display_reg(void __iomem *ddr,
	struct amac_reg_dbg_info *reg_info, char *reg_name);
static uint32_t dbg_get_bit_mask(uint32_t hi_bit, uint32_t lo_bit);
static void dbg_disp_stats(struct sysctl_ethstats *statp);




static uint32_t dbg_get_bit_mask(uint32_t hi_bit, uint32_t lo_bit)
{
	uint32_t i;
	uint32_t bit_mask = 0;

	for (i = lo_bit; i <= hi_bit; i++)
		bit_mask |= 1 << i;

	return bit_mask;
}



static void dbg_display_reg(void __iomem *addr,
	struct amac_reg_dbg_info *reg_info,
	char *reg_name)
{
	unsigned int reg_val = 0;
	unsigned int reg_mask, tmpval, reg_shift, tmpshift_val;
	uint32_t i = 0;
	uint32_t j = 0;
	char endbitstr[4] = "";
	char bitinfo[15] = "";


	reg_val = readl(addr);

	pr_info("\nReg: %s = 0x%x\n", reg_name, reg_val);

	while ((reg_info[i].hi_bit != 0xFF) && i < 32) {
		memset(endbitstr, 0, 4);
		memset(bitinfo, 0, 15);
		if (reg_info[i].lo_bit == -1) {
			reg_mask = (uint32_t) (1 << reg_info[i].hi_bit)
				& 0xffffffff;
			reg_shift = reg_info[i].hi_bit;
		} else {
			reg_shift = reg_info[i].lo_bit;
			reg_mask = dbg_get_bit_mask(reg_info[i].hi_bit,
				reg_info[i].lo_bit);
			sprintf(endbitstr, ":%2d", reg_info[i].lo_bit);
		}

		tmpval = tmpshift_val = (uint32_t)(reg_val & reg_mask);

		tmpshift_val = (uint32_t)(tmpshift_val >> reg_shift);

		for (j = 0; j < 3; j++) {
			if (reg_info[i].bit_field[j].val == -1) {
				j = 10; /* break */
			} else if (tmpval ==
				(uint32_t)reg_info[i].bit_field[j].val) {
				snprintf(bitinfo,
					15,
					"[%s]",
					reg_info[i].bit_field[j].desc);
				j = 10; /* break */
			}
		}

		pr_info(" [bit %2d%3s] %30s = 0x%x %s\n",
			reg_info[i].hi_bit,
			endbitstr,
			reg_info[i].info,
			tmpshift_val,
			bitinfo);
		i++;
	}
}

static void dbg_dump_descp(struct dma_cfg *dma_info)
{
	pr_info("addr=0x%x\n", dma_info->addr);
	pr_info("ring_len=0x%x\n", dma_info->ring_len);
	pr_info("alloc_size=0x%x\n", dma_info->alloc_size);
	pr_info("index=0x%x\n", dma_info->index);

	pr_info("First Descriptor Addr=0x%x\n",
		(unsigned int)dma_info->descp);
	pr_info(" ctrl1   =0x%x\n",
		((struct dma64_desc *)dma_info->descp)->ctrl1);
	pr_info(" ctrl2   =0x%x\n",
		((struct dma64_desc *)dma_info->descp)->ctrl2);
	pr_info(" addrlow =0x%x\n",
		((struct dma64_desc *)dma_info->descp)->addrlow);
	pr_info(" addrhigh=0x%x\n",
		((struct dma64_desc *)dma_info->descp)->addrhigh);
}


static void dbg_disp_stats(struct sysctl_ethstats *statp)
{
	pr_info("\nEthernet Stats\n");
	pr_info(" rx_bytes           = 0x%x\n",
		(unsigned int)statp->rx_bytes);
	pr_info(" rx_dropped_pkts    = 0x%x\n",
		(unsigned int)statp->rx_dropped_pkts);
	pr_info(" rx_dropped_pkts    = 0x%x\n",
		(unsigned int)statp->rx_dropped_pkts);
	pr_info(" rx_resyncs         = 0x%x\n",
		(unsigned int)statp->rx_resyncs);
	pr_info(" rx_wraparounds     = 0x%x\n",
		(unsigned int)statp->rx_wraparounds);
	pr_info(" rx_syncchecked     = 0x%x\n",
		(unsigned int)statp->rx_syncchecked);
	pr_info(" rx_syncdroppedpkts = 0x%x\n",
		(unsigned int)statp->rx_syncdroppedpkts);
	pr_info(" rx_noskb           = 0x%x\n",
		(unsigned int)statp->rx_noskb);
	pr_info(" rx_broadcast       = 0x%x\n",
		(unsigned int)statp->rx_broadcast);
	pr_info(" rx_multicast       = 0x%x\n",
		(unsigned int)statp->rx_multicast);
	pr_info(" rx_unicast         = 0x%x\n",
		(unsigned int)statp->rx_unicast);
	pr_info(" tx_broadcast       = 0x%x\n",
		(unsigned int)statp->tx_broadcast);
	pr_info(" tx_multicast       = 0x%x\n",
		(unsigned int)statp->tx_multicast);
	pr_info(" tx_unicast         = 0x%x\n\n",
		(unsigned int)statp->tx_unicast);
}


static void dbg_disp_port_info(struct port_data *port)
{
	int i;

	pr_info("Total Ports: %d\n", port->count);
	for (i = 0; i < port->count; i++) {
		pr_info("Port[%i] id: %d\n", i,
			port->info[i].num);
		pr_info("Port[%i] type: %s\n", i,
			(port->info[i].type == AMAC_PORT_TYPE_LAN) ?
			"LAN" : "PC");
		pr_info("Port[%i] phy_id: %d\n", i,
			port->info[i].phy_id);
		pr_info("Port[%i] aneg: %d\n", i,
			port->info[i].phy_info.aneg);
		pr_info("Port[%i] link speed: %d\n", i,
			port->info[i].phy_info.speed);
		pr_info("Port[%i] link duplex: %d\n", i,
			port->info[i].phy_info.duplex);
		pr_info("Port[%i] link pause: %d\n\n", i,
			port->info[i].phy_info.pause);
	}
}


/**
 * Display the Ethernet frame header info
 * @buff - Ethernet Fram
 * @info - String to be printed to identify the display msg
 */
void bcm_amac_dbg_frame_hdr(char *buff, char *info, int tag_offset)
{
	pr_info("\nFRAME: %s\n", info);

	pr_info(
	"Dst=%x:%x:%x:%x:%x:%x, Src=%x:%x:%x:%x:%x:%x, EType=0x%x%x%x%x\n",
	/* 0 - 5= Dest mac */
	buff[0], buff[1], buff[2],
	buff[3], buff[4], buff[5],
	/* 6 - 11= Src MAC */
	buff[6], buff[7], buff[8],
	buff[9], buff[10], buff[11],

	/* 12 - 13= Ethertype */
	(buff[12 + tag_offset] >> 4) & 0x0F, buff[12 + tag_offset] & 0x0F,
	(buff[13 + tag_offset] >> 4) & 0x0F, buff[13 + tag_offset] & 0x0F);

	if (tag_offset >= 4)
		pr_info("TAG[0]: %x%x %x%x %x%x %x%x hex\n",
			(buff[12] >> 4) & 0x0F, buff[12] & 0x0F,
			(buff[13] >> 4) & 0x0F, buff[13] & 0x0F,
			(buff[14] >> 4) & 0x0F, buff[14] & 0x0F,
			(buff[15] >> 4) & 0x0F, buff[15] & 0x0F);

	if (tag_offset == 8)
		pr_info("TAG[1]: %x%x %x%x %x%x %x%x hex\n",
			(buff[16] >> 4) & 0x0F, buff[16] & 0x0F,
			(buff[17] >> 4) & 0x0F, buff[17] & 0x0F,
			(buff[18] >> 4) & 0x0F, buff[18] & 0x0F,
			(buff[19] >> 4) & 0x0F, buff[19] & 0x0F);

}



void dbg_disp_arl_entry_info(struct bcm_amac_priv *privp)
{
	int i;
	uint64_t mport_addr;
	uint32_t mport_vector = 0;
	uint16_t mport_ctrl = 0;
	struct esw_info *robo = &(privp->esw);

	robo->ops->read_reg(robo, PAGE_MPORT, REG_MPORT_CTRL, &mport_ctrl, 16);
	pr_info("REG_MPORT_CTRL: 0x%x\n", mport_ctrl);

	for (i = 0; i <= MPORT_LAST; i++) {
		robo->ops->read_reg(robo, PAGE_MPORT,
			(REG_MPORT_ADDR0 + (i * NEXT_MPORT_REG_OFFSET)),
			&mport_addr, 64);

		robo->ops->read_reg(robo, PAGE_MPORT,
			(REG_MPORT_VCTR0 + (i * NEXT_MPORT_REG_OFFSET)),
			&mport_vector, 32);

		pr_info("REG_MPORT_ADDR[%d]: 0x%x%x\n", i,
			(unsigned int)((mport_addr >> 32) & 0xFFFFFFFF),
			(unsigned int)(mport_addr & 0xFFFFFFFF));
		pr_info("REG_MPORT_VECTOR[%d]: 0x%x\n\n", i, mport_vector);
	}
}

/**
 * API is used to display debug information.
 * @privp - device private data
 * @block - which info to display
 */
void bcm_amac_dbg_display(struct bcm_amac_priv *privp, int block)
{
	switch (block) {
	case AMAC_DBG_GMAC0:
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_INT_MASK_ADDR,
			&gmac0_intmask[0],
			"GMAC0_INTMASK");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_INT_STATUS_ADDR,
			&gmac0_intstat[0],
			"GMAC0_INTSTATUS");
		break;

	case AMAC_DBG_TX_DMA:
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_OFFSET,
			&gmac0_txctrl[0],
			"GMAC0_TXCONTROL");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_TX_PTR_OFFSET,
			&gmac0_txptr[0],
			"GMAC0_TXPTR");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_OFFSET,
			&gmac0_txlowaddr[0],
			"GMAC0_TXADDR_LOW");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_TX_STATUS0_OFFSET,
			&gmac0_txstatus0[0],
			"GMAC0_TXSTATUS0");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_TX_STATUS1_OFFSET,
			&gmac0_txstatus1[0],
			"GMAC0_TXSTATUS1");
		dbg_display_reg(
			privp->hw.reg.amac_core + 0x308,
			&gmac0_txgdpkts[0],
			"GMAC0_TX_GD_PKTS");
		break;

	case AMAC_DBG_RX_DMA:
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_OFFSET,
			&gmac0_rcvctrl[0],
			"GMAC0_RCVCONTROL");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_OFFSET,
			&gmac0_rcvptr[0],
			"GMAC0_RCVPTR");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_LO_OFFSET,
			&gmac0_rcvlowaddr[0],
			"GMAC0_RCVADDR_LOW");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS0_OFFSET,
			&gmac0_rcvstatus0[0],
			"GMAC0_RCVSTATUS0");
		dbg_display_reg(
			privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS1_OFFSET,
			&gmac0_rcvstatus1[0],
			"GMAC0_RCVSTATUS1");
		break;


	case AMAC_DBG_TX_DUMP_DESC:
		pr_info("\nDUMPING TX DMA DESCRIPTOR\n");
		dbg_dump_descp(&privp->dma.tx);
	break;

	case AMAC_DBG_DISP_STATS:
		dbg_disp_stats(&privp->eth_stats);
	break;

	case AMAC_DBG_DISP_PORT_INFO:
		dbg_disp_port_info(&privp->port);
		break;

	case AMAC_DBG_ARL_ENTRY:
		dbg_disp_arl_entry_info(privp);
		break;


	case AMAC_DBG_RX_PHY0:
	case AMAC_DBG_RX_PHY1:
	default:
		pr_info("DBG CMD NOT SUPPORTED\n");
	}
}



