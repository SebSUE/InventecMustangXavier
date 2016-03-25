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

#include <linux/io.h>
#include "bcm-sf2-regs.h"
#include "bcm-amac-enet.h"
#include "bcm-robo.h"


#define PORT_ALL        0x103
#define LLDP_E_TYPE     0x88CC

/* Convert MAC addr pointer to 64 bits */
#define MACTOU64(mac)	(((uint64_t)((mac)[0]) << 40) + \
	((uint64_t)((mac)[1]) << 32) + \
	((uint64_t)((mac)[2]) << 24) + \
	((uint64_t)((mac)[3]) << 16) + \
	((uint64_t)((mac)[4]) << 8) + \
	((uint64_t)((mac)[5])))

static int bcm_robo_attach(struct esw_info *robo);
static int bcm_robo_enable_switch(struct bcm_amac_priv *privp);


static int srab_interface_reset(struct esw_info *robo)
{
	int i, ret = 0;
	u32 val32;

	/* Wait for switch initialization complete */
	for (i = SRAB_MAX_RETRY * 10; i > 0; i--) {
		val32 = readl(robo->srab_base + REG_SRAB_CMDSTAT);
		if ((val32 & CFG_F_SW_INIT_DONE_MASK))
			break;
	}

	/* timed out */
	if (!i) {
		pr_err("srab_interface_reset: timeout sw_init_done");
		ret = -1;
	}

	/* Set the SRAU reset bit */
	writel(CFG_F_SRA_RST_MASK, robo->srab_base + REG_SRAB_CMDSTAT);

	/* Wait for it to auto-clear */
	for (i = SRAB_MAX_RETRY * 10; i > 0; i--) {
		val32 = readl(robo->srab_base + REG_SRAB_CMDSTAT);
		if ((val32 & CFG_F_SRA_RST_MASK) == 0)
			break;
	}

	/* timed out */
	if (!i) {
		pr_err("srab_interface_reset: timeout sra_rst");
		ret |= -2;
	}

	return ret;
}

static int check_srab_ready(void *esw)
{
	struct esw_info *robo = (struct esw_info *)esw;
	u32 result;
	int i;

	/* Wait for command complete */
	for (i = SRAB_MAX_RETRY; i > 0; i--) {
		result = ioread32(robo->srab_base + REG_SRAB_CMDSTAT);
		if ((result & SRAB_READY_MASK) == 0)
			break;
		udelay(1);
	}

	/* timed out */
	if (i == 0) {
		pr_err("srab_request: timeout");
		/* TODO srab_interface_reset(robo); */
		return -1;
	}

	return 0;
}

static int esw_read_reg(void *esw, u8 page, u8 offset,
			void *val, int len)
{
	struct esw_info *robo = (struct esw_info *)esw;
	u32 cmd = 0;
	u32 data[2];
	u32 *p = (u32 *)val;

	spin_lock(&robo->lock);

	if (check_srab_ready(esw))
		goto esw_read_reg_busy;

	/* Issue the read command */
	cmd = ((page << SRAB_PAGE_SHIFT)
	       | (offset << SRAB_OFFSET_SHIFT)
	       | SRAB_READY_MASK);
	iowrite32(cmd, robo->srab_base + REG_SRAB_CMDSTAT);

	/* Wait for command complete */
	if (check_srab_ready(esw))
		goto esw_read_reg_busy;

	spin_unlock(&robo->lock);

	data[0] = ioread32(robo->srab_base + REG_SRAB_RDL);
	data[1] = ioread32(robo->srab_base + REG_SRAB_RDH);

	switch (len) {
	case 64:
		p[0] = data[0];
		p[1] = data[1];
		break;
	case 32:
		p[0] = data[0];
		break;
	case 16:
		data[0] &= 0xFFFF;
		*((u16 *)p) = (u16)data[0];
		break;
	case 8:
		data[0] &= 0xFF;
		*((u8 *)p) = (u8)data[0];
		break;
	default:
		pr_err("ESW register read error (wrong length %d bits)\n",
		       len);
		return -EINVAL;
	}

	return 0;

esw_read_reg_busy:
	spin_unlock(&robo->lock);
	return -EBUSY;
}

static int esw_write_reg(void *esw, u8 page, u8 offset,
			 void *val, int len)
{
	struct esw_info *robo = (struct esw_info *)esw;
	u32 cmd = 0;
	u32 data_high = 0;
	u32 data_low = 0;
	u32 *p;

	switch (len) {
	case 64:
		p = (u32 *)val;
		data_high = *(p + 1);
		data_low = *p;
		break;
	case 32:
		data_low = *((u32 *)val);
		break;
	case 16:
		data_low = *((u16 *)val);
		break;
	case 8:
		data_low = *((u8 *)val);
		break;
	default:
		pr_err("ESW register write error (wrong length %d bits)\n",
		       len);
		return -EINVAL;
	}

	spin_lock(&robo->lock);

	if (check_srab_ready(esw))
		goto esw_write_reg_busy;

	iowrite32(data_high, robo->srab_base + REG_SRAB_WDH);
	iowrite32(data_low, robo->srab_base + REG_SRAB_WDL);

	/* Issue the write command */
	cmd = ((page << SRAB_PAGE_SHIFT)
	       | (offset << SRAB_OFFSET_SHIFT)
	       | SRAB_READY_MASK | SRAB_WRITE_MASK);
	iowrite32(cmd, robo->srab_base + REG_SRAB_CMDSTAT);

	/* Wait for command complete */
	if (check_srab_ready(esw))
		goto esw_write_reg_busy;

	spin_unlock(&robo->lock);

	return 0;

esw_write_reg_busy:
	spin_unlock(&robo->lock);
	return -EBUSY;
}

/* SRAB interface functions */
static struct esw_ops srab = {
	esw_read_reg,
	esw_write_reg,
};

/* High level switch configuration functions. */

/* Get access to the RoboSwitch */
static int bcm_robo_attach(struct esw_info *robo)
{
	int rc;

	robo->ops = &srab;

	srab_interface_reset(robo);
	esw_read_reg(robo, PAGE_MMR, REG_VERSION_ID, &robo->corerev, 8);

	rc = esw_read_reg(robo, PAGE_MMR, REG_DEVICE_ID, &robo->devid32, 32);
	pr_debug("devid read %ssuccesfully via srab: 0x%x\n",
		 rc ? "un" : "", robo->devid32);

	if ((rc != 0) || (robo->devid32 == 0)) {
		pr_err("error reading devid\n");
		goto error;
	}
	pr_debug("devid32: 0x%x\n", robo->devid32);

	return 0;

error:
	return -1;
}


static void bcm_robo_pause_frame_init(struct bcm_amac_priv *privp)
{
	struct esw_info *robo = &privp->esw;
	u32 reg_val;
	int i;

	robo->ops->read_reg(robo,
		PAGE_IOCTRL,
		0x18,
		&reg_val,
		32);

	reg_val |= 0x800000; /* Enable override */
	for (i = 0; i < privp->port.count; i++) {
		if (privp->port.info[i].type == AMAC_PORT_TYPE_LAN) {
			/* Disable PAUSE frames for LAN ports */
			reg_val &= ~(1 << privp->port.info[i].num); /* TX */
			reg_val &= ~(1 << (privp->port.info[i].num +
				SWITCH_RX_PAUSE_CAP_OFFSET));
		} else {
			/* Keep PAUSE frames enabled for other ports */
			reg_val |= (1 << privp->port.info[i].num); /* TX */
			reg_val |= (1 << (privp->port.info[i].num +
				SWITCH_RX_PAUSE_CAP_OFFSET));
		}
	}

	robo->ops->write_reg(robo,
		PAGE_IOCTRL,
		0x18,
		&reg_val,
		32);

}

/* Enable switching/forwarding */
static int bcm_robo_enable_switch(struct bcm_amac_priv *privp)
{
	struct esw_info *robo = &privp->esw;
	int ret = 0;
	u8 val8;
	u16 val16;

	/* Setup PAUSE capability of ports */
	bcm_robo_pause_frame_init(privp);

	/* Switch Mode register (Page 0, Address 0x0B) */
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, 8);

	/* Set managed mode */
	val8 |= 1;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, 8);

	/* Enable forwarding */
	val8 |= (1 << 1);
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, 8);

	/* Read back */
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, 8);
	if (!(val8 & (1 << 1))) {
		pr_err("robo_enable_switch: enabling forwarding failed\n");
		ret = -1;
	}

	/* No spanning tree for external ports */
	val8 = 0;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PORT0, &val8, 8);
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PORT1, &val8, 8);

	/* make sure external ports are not in protected mode
	 * (Page 0, Address 0x24)
	 */
	val16 = 0;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PPORT, &val16, 16);

	/* Over ride IMP(Port8) status to make it link by default */
	robo->ops->read_reg(robo, PAGE_IOCTRL, REG_IOCTRL_PORTIMP, &val16, 16);
	/* 2G_ENABLED:
	 * Page :0x00
	 * ( Offset: 0xe ) IMP Port States Override Register
	 * [6]: GMII SPEED UP 2G
	 */
	val16 |= 0xf1;	/* Make Link pass and override it. */
	robo->ops->write_reg(robo, PAGE_IOCTRL, REG_IOCTRL_PORTIMP, &val16, 16);

	/* IMP(Port8) config BRCM tag */
	val8 = 0;
	robo->ops->read_reg(robo, PAGE_MMR, REG_BRCM_HDR, &val8, 8);
	if (privp->brcm_tag)
		val8 |= 0x01;
	else
		val8 &= 0xfe;
	robo->ops->write_reg(robo, PAGE_MMR, REG_BRCM_HDR, &val8, 8);


	/* IMP(Port8) Enable receive bcast packets */
	val8 = 0;
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, 8);
	val8 |= 0x4;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, 8);

	/* IMP(Port8) IMP port Enable */
	val8 = 0;
	robo->ops->read_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, 8);
	val8 |= 0x80;
	robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, 8);

	/* Port5 (external PHY) configuration */
	/* Port N GMII Port States Override Register
	 * (Page 0x00 , address Offset: 0x0e , 0x58-0x5d and 0x5f )
	 * SPEED/ DUPLEX_MODE/ LINK_STS
	 */
	robo->ops->read_reg(robo, PAGE_IOCTRL, REG_IOCTRL_PORT5, &val16, 16);
	val16 |= 0x71;  /* Make Link pass and override it. */
	robo->ops->write_reg(robo, PAGE_IOCTRL, REG_IOCTRL_PORT5, &val16, 16);
	/* enable Port 5 RGMII delay mode */
	robo->ops->read_reg(robo, PAGE_IOCTRL, REG_CTRL_P5RGMIICTL, &val8, 8);
	val8 |= 0x02;
	robo->ops->write_reg(robo, PAGE_IOCTRL, REG_CTRL_P5RGMIICTL, &val8, 8);
	/* No spanning tree for Port5 */
	val8 = 0;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PORT5, &val8, 8);

	/* Enable LLDP */
	bcm_esw_set_mport_entry(privp, NULL, LLDP_E_TYPE, MPORT_RESERVED);

	val16 = 0x103; /* Enable 'EEE' on Port 0, 1 & 8*/
	robo->ops->write_reg(robo, PAGE_EEE, REG_EEE_EN_CTRL, &val16, 16);

	return ret;
}

static void robo_reset_mib(struct esw_info *robo)
{
	u8 val8;

	robo->ops->read_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, 8);
	/* set clear mib bit */
	val8 |= 0x01;
	robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, 8);
	/* clear clear mib bit */
	val8 &= 0xfe;
	robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, 8);
}

int bcm_esw_init(void *p)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)p;
	struct esw_info *robo = &(privp->esw);
	u8 val8;

	spin_lock_init(&robo->lock);

	/* Attach to the switch */
	if (bcm_robo_attach(robo)) {
		pr_err("Robo switch attach failed\n");
		return -1;
	}

	/* enable ESW direct access to EGPHY */
	/* MDIO Directo Access Enable Register (Page 0x0, Address 00x6f) */
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MDIO_DA, &val8, 8);
	val8 |= 01;		/* enable MDIO direct access */
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MDIO_DA, &val8, 8);

	/* Enable switching/forwarding */
	if (bcm_robo_enable_switch(privp)) {
		pr_err("chipattach: robo_enable_switch failed\n");
		return -1;
	}
	robo_reset_mib(robo);

	return 0;
}



/* Enables or disables multicast mode
 *
 * @privp - driver data pointer
 * @enable - enable or disable multicast
 */
void bcm_esw_enable_multicast(struct bcm_amac_priv *privp, int enable)
{
	struct esw_info *robo = &(privp->esw);
	u8 val8 = 0;

	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, 8);

	/* Enable or disable only if required */
	if (enable && (!(val8 & IMP_C_RX_MCST_EN)))
		val8 |= IMP_C_RX_MCST_EN;
	else if (!enable && ((val8 & IMP_C_RX_MCST_EN)))
		val8 &= (u8)~IMP_C_RX_MCST_EN;
	else
		return;

	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, 8);
}




/* Clears an MPORT entry
 *
 * @privp - driver data pointer
 * @mport - Entry to be cleared
 */
static void esw_clear_mport_entry(struct bcm_amac_priv *privp, u8 mport)
{
	struct esw_info *robo = &(privp->esw);
	u32 reg_offset = mport * NEXT_MPORT_REG_OFFSET;
	uint64_t mport_addr = 0;
	u32 mport_vector = 0;
	u32 mport_ctrl = 0;

	/* Clear Multiport Address reg */
	robo->ops->write_reg(robo, PAGE_MPORT,
		(REG_MPORT_ADDR0 + reg_offset),
		&mport_addr, 64);

	/* Clear Multiport vector reg */
	robo->ops->write_reg(robo, PAGE_MPORT,
		(REG_MPORT_VCTR0 + reg_offset),
		&mport_vector, 32);

	/* Read MPORT Ctrl reg */
	robo->ops->read_reg(robo, PAGE_MPORT, REG_MPORT_CTRL, &mport_ctrl, 16);
	mport_ctrl &= (u32)~(3 << (2 * mport)); /* Clear MPORT control */

	robo->ops->write_reg(robo, PAGE_MPORT, REG_MPORT_CTRL, &mport_ctrl, 16);
}



/* Adds or removes a MPORT entry
 * MPORT entries can be filtered based on MAC, ethertype or both
 * If neither is specified, the API will remove the ARL entry.
 *
 * @privp - driver data pointer
 * @mac - multicast mac address to be added
 * @ethertype - multicast ethertype to be added
 * @mport - mport to add/remove ARL entry
 */
void bcm_esw_set_mport_entry(struct bcm_amac_priv *privp, char *mac,
	u16 ethertype, u8 mport)
{
	struct esw_info *robo = &(privp->esw);
	u32 reg_offset;
	uint64_t mport_addr = 0;
	u32 mport_vector = 0;
	u16 mport_ctrl = 0;

	if (mport > MPORT_LAST)
		return;

	if ((!ethertype) && (mac == NULL)) {
		/* Clear the mport entry */
		esw_clear_mport_entry(privp, mport);
		return;
	}

	/* Set new ARL entry based on either ethertype, mac or both */

	/* Read MPORT Ctrl reg */
	robo->ops->read_reg(robo, PAGE_MPORT, REG_MPORT_CTRL, &mport_ctrl, 16);

	if (mac != NULL) {
		mport_addr = MACTOU64(mac); /* Preserve endianess */
		mport_vector = PORT_ALL;
		/* Filter based on mac addr */
		mport_ctrl |= (2 << (2 * mport));
	}

	if (ethertype) {
		mport_addr |= ((uint64_t)ethertype) << MPORT_E_TYPE_SHIFT;
		mport_vector |= PORT_ALL;
		/* Filter based on ether type */
		mport_ctrl |= (1 << (2 * mport));
	}

	reg_offset = mport * NEXT_MPORT_REG_OFFSET;

	/* Write to Multiport Address reg
	 * Writes either the mac address or the ethernet type or both
	 */
	robo->ops->write_reg(robo, PAGE_MPORT,
		(REG_MPORT_ADDR0 + reg_offset),
		&mport_addr, 64);

	/* Configure Multiport vector reg
	 * Decides which port to forward
	 */
	robo->ops->write_reg(robo, PAGE_MPORT,
		(REG_MPORT_VCTR0 + reg_offset),
		&mport_vector, 32);

	/* Enable the multiport in mport control
	 * Configures the filter type
	 */
	robo->ops->write_reg(robo, PAGE_MPORT, REG_MPORT_CTRL, &mport_ctrl, 16);

}


/* Clears all MPORT entries except the 'reserved' one
 *
 * @privp - driver data pointer
 */
void bcm_esw_clear_all_mport_entry(struct bcm_amac_priv *privp)
{
	int i;

	/* Clear all except the reserved */
	for (i = MPORT_START; i <= MPORT_LAST; i++)
		esw_clear_mport_entry(privp, i);
}


/* Wait for ARL access to complete
 *
 * @privp - device info data structure pointer
 *
 * Returns: '0' for success '1' if timed out
 */
static inline int esw_arl_access_done(struct bcm_amac_priv *privp)
{
	struct esw_info *robo = &(privp->esw);
	int timeout = 0;
	u8 val8;

	robo->ops->read_reg(robo, PAGE_ARLACCS,
			REG_ARLA_RWCTL,
			&val8, 8);

	while (val8 & REG_ARLA_RWCTRL_START_DONE) {

		if (timeout >= REG_ARL_TIMEOUT)
			return -EBUSY;

		udelay(10);

		timeout += 10;

		robo->ops->read_reg(robo, PAGE_ARLACCS,
				REG_ARLA_RWCTL,
				&val8, 8);
	}

	return 0;
}



/* Adds a ARL entry
 *
 * @privp - driver data pointer
 * @mac - multicast mac address to be added
 * @vid - VLAN id (usually '0')
 * @age - ARL Age
 * @portmask - Port mask
 * @entry_static - If entry is static or not (true/false)
 *
 * Returns: '0' for success or else the error code
 */
int bcm_esw_set_arl_entry(struct bcm_amac_priv *privp, char *macp,
	int vid, int age, int portmask, bool entry_static)
{
	struct esw_info *robo = &(privp->esw);
	uint64_t macaddr = 0, mac_read = 0, macvid = 0;
	int i = -1;
	u32 reg;
	u8 rw_flags;
	/* Dynamic ARL entry index of one ARL bucket */
	int last_dyn_arl_index = -1;
	int rc = -1;

	if (macp != NULL)
		macaddr = MACTOU64(macp); /* Preserve endianess */
	else
		return -EINVAL;

	macvid = macaddr | (((uint64_t)vid) << REG_ARLA_MACVID_VID_OFFSET);

	/* Setup MAC and VID for retreiving matching entries */
	robo->ops->write_reg(robo, PAGE_ARLACCS, REG_ARLA_MAC, &macaddr, 64);
	robo->ops->write_reg(robo, PAGE_ARLACCS, REG_ARLA_VID, &vid, 16);
	/* Read all matched entry to entry registers */
	rw_flags = REG_ARLA_RWCTRL_START_DONE | REG_ARLA_RWCTRL_READ;
	robo->ops->write_reg(robo, PAGE_ARLACCS, REG_ARLA_RWCTL, &rw_flags, 8);

	rc = esw_arl_access_done(privp);
	if (rc) {
		pr_err("%s: read ARL timeout!\n", __func__);
		return rc;
	}

	/* Search for matched and/or dynamic entries */
	for (i = 0; i < REG_ARLA_ENTRY_MAX; i++) {
		robo->ops->read_reg(robo,
				    PAGE_ARLACCS,
				    (REG_ARLA_FWD_ENTRY0 +
				     (i * NEXT_ARLA_ENTRY_OFFSET)),
				    &reg,
				    32);
		if (!(reg & REG_ARLA_FWDENTRY_STATIC)) {
			/* Mark the last index of the dynamic ARL entry.
			 * This entry will be overwritten when all 4
			 * are used.
			 */
			last_dyn_arl_index = i;
		}
		if (reg & REG_ARLA_FWDENTRY_VALID) {
			/* For valid entry, Check if the MAC+VID matches */
			robo->ops->read_reg(robo,
					    PAGE_ARLACCS,
					    (REG_ARLA_MACVID_ENTRY0 +
					     (i * NEXT_ARLA_ENTRY_OFFSET)),
					    &mac_read,
					    64);

			if (mac_read == macvid)
				/* index i is the matched entry and
				 * will be overwritten
				 */
				break;
		} else
			/* index i is the invalid entry, will be overwritten */
			break;
	}

	/* No matched entry is found */
	if (i == REG_ARLA_ENTRY_MAX) {
		if (last_dyn_arl_index == -1)
			/* No dynamic entry, no room for new one */
			return -ENOMEM;
		else
			/* Dynamic entry will be overwritten */
			i = last_dyn_arl_index;
	}

	/* Write forward entry */
	reg |= REG_ARLA_FWDENTRY_VALID;
	reg |= (REG_ARLA_FWDENTRY_AGE |
		(age << REG_ARLA_FWDENTRY_TC_SHIFT) |
		portmask);
	if (entry_static)
		reg |= REG_ARLA_FWDENTRY_STATIC;

	robo->ops->write_reg(robo,
			     PAGE_ARLACCS,
			     (REG_ARLA_FWD_ENTRY0 +
			      (i * NEXT_ARLA_ENTRY_OFFSET)),
			     &reg,
			     32);

	/* Write MAC+VID */
	robo->ops->write_reg(robo,
			     PAGE_ARLACCS,
			     (REG_ARLA_MACVID_ENTRY0 +
			      (i * NEXT_ARLA_ENTRY_OFFSET)),
			     &macvid,
			     64);

	/* Update the ARL table Entry */
	robo->ops->write_reg(robo, PAGE_ARLACCS,
			     REG_ARLA_VID, &vid, 16);
	robo->ops->write_reg(robo, PAGE_ARLACCS,
			     REG_ARLA_MAC,
			     &macaddr, 64);

	rw_flags = REG_ARLA_RWCTRL_START_DONE |
			REG_ARLA_RWCTRL_WRITE;
	robo->ops->write_reg(robo, PAGE_ARLACCS,
			     REG_ARLA_RWCTL,
			     &rw_flags, 8);

	rc = esw_arl_access_done(privp);

	return rc;
}


/* Clears all static ARL entries
 *
 * @privp - driver data pointer
 */
void bcm_esw_clear_all_arl_entry(struct bcm_amac_priv *privp)
{
	struct esw_info *robo = &(privp->esw);
	uint64_t mac_read = 0;
	int i;
	u32 reg;
	u8 rw_flags;
	u16 vid = 0;

	for (i = 0; i < REG_ARLA_ENTRY_MAX; i++) {
		robo->ops->read_reg(robo,
			PAGE_ARLACCS,
			(REG_ARLA_FWD_ENTRY0 +
				(i * NEXT_ARLA_ENTRY_OFFSET)),
			&reg,
			32);
		if (reg & REG_ARLA_FWDENTRY_STATIC) {

			/* Make this entry invalid and not static */
			robo->ops->read_reg(robo,
				PAGE_ARLACCS,
				(REG_ARLA_MACVID_ENTRY0 +
					(i * NEXT_ARLA_ENTRY_OFFSET)),
				&mac_read,
				64);

			reg &= ~REG_ARLA_FWDENTRY_VALID;
			reg &= ~REG_ARLA_FWDENTRY_STATIC;
			reg &= ~REG_ARLA_FWDENTRY_AGE;

			robo->ops->write_reg(robo,
				PAGE_ARLACCS,
				(REG_ARLA_FWD_ENTRY0 +
					(i * NEXT_ARLA_ENTRY_OFFSET)),
				&reg,
				32);

			/* Write MAC */
			robo->ops->write_reg(robo,
				PAGE_ARLACCS,
				(REG_ARLA_MACVID_ENTRY0 +
					(i * NEXT_ARLA_ENTRY_OFFSET)),
				&mac_read,
				64);

			/* Update the ARL table Entry */
			robo->ops->write_reg(robo, PAGE_ARLACCS,
				REG_ARLA_VID, &vid, 16);
			robo->ops->write_reg(robo, PAGE_ARLACCS,
				REG_ARLA_MAC,
				&mac_read, 64);

			rw_flags = REG_ARLA_RWCTRL_START_DONE |
				REG_ARLA_RWCTRL_WRITE;
			robo->ops->write_reg(robo, PAGE_ARLACCS,
				REG_ARLA_RWCTL,
				&rw_flags, 8);

			esw_arl_access_done(privp);
		}
	}
}

enum amac_reboot_reason bcm_esw_get_reboot(struct bcm_amac_priv *privp)
{
	struct esw_info *robo = &(privp->esw);
	u32 val;
	enum amac_reboot_reason rst_reason = AMAC_REBOOT_COLD;
	int rc;

	rc = esw_read_reg(robo,
			PAGE_BPM,
			REG_BPM_SPARE0,
			&val,
			32);

	if (!rc && (val == AMAC_WARM_RESET_SEQ))
		rst_reason = AMAC_REBOOT_WARM;

	return rst_reason;
}

