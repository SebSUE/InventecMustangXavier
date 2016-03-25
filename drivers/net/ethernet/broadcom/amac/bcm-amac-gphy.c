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

#include <linux/types.h>
#include <linux/mdio.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/brcmphy.h>
#include <linux/phy/iproc_mdio_phy.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>

#include "bcm-amac-regs.h"
#include "bcm-amac-enet.h"
#include "bcm-amac-core.h"

#define MII_CTRL_BUSY_TIMEOUT 1000
#define MII_CTRL_BUSY_BIT_MASK (1 << CHIPCMNG_MII_MGMT_CTRL__BSY)

#define BCM_GPHY_ENABLE 1
#define BCM_GPHY_DISABLE 0

#define GPHY_MDCDIV 0x1a

#define ADVERTISE_100M (ADVERTISE_100BASE4 | ADVERTISE_100FULL | \
				ADVERTISE_100HALF)

static unsigned int g_lnswp;

static void amac_gphy_handle_link_change(struct net_device *dev);
static int amac_gphy_mii_probe(struct net_device *dev);
static int amac_gphy_mdio_read(struct mii_bus *bp, int phy_id, int reg);
static int amac_gphy_mdio_write(struct mii_bus *bp, int phy_id, int reg,
	u16 val);
static void amac_gphy_lswap(struct mii_bus *bp, u32 phyaddr);
static int amac_gphy_mii_reset(struct mii_bus *bp);


static int amac_gphy_advertise_100M(struct phy_device *phydev, bool enable)
{
	int adv;
	int rc = 0;

	adv = phy_read(phydev, MII_ADVERTISE);
	if (adv < 0)
		return adv;

	if (enable) {
		/* Enable 100M advertisement */
		if (!(adv & ADVERTISE_100M)) {
			/* 100BaseT4 is not supported in Cygnus,
			 * so don't enable it
			 */
			adv |= ADVERTISE_100FULL;
			adv |= ADVERTISE_100HALF;
			rc = phy_write(phydev, MII_ADVERTISE, adv);
		}
		phydev->supported |= (SUPPORTED_100baseT_Half);
		phydev->supported |= (SUPPORTED_100baseT_Full);
	} else {
		/* Disable 100M advertisement */
		if (adv & ADVERTISE_100M) {
			adv &= ~ADVERTISE_100M;
			rc = phy_write(phydev, MII_ADVERTISE, adv);
		}
		phydev->supported &= ~(SUPPORTED_100baseT_Half);
		phydev->supported &= ~(SUPPORTED_100baseT_Full);
	}

	return rc;
}

static int amac_gphy_advertise_1G(struct phy_device *phydev, bool enable)
{
	int adv;
	int rc = 0;

	adv = phy_read(phydev, MII_CTRL1000);
	if (adv < 0)
		return adv;

	if (enable) {
		/* Enable 1000M (1G) advertisement */
		if (!(adv & (ADVERTISE_1000FULL | ADVERTISE_1000HALF))) {
			adv |= ADVERTISE_1000FULL;
			adv |= ADVERTISE_1000HALF;
			rc = phy_write(phydev, MII_CTRL1000, adv);
		}
		phydev->supported |= (SUPPORTED_1000baseT_Half);
		phydev->supported |= (SUPPORTED_1000baseT_Full);
	} else {
		/* Disable 1000M (1G) advertisement */
		if (adv & (ADVERTISE_1000FULL | ADVERTISE_1000HALF)) {
			adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);
			rc = phy_write(phydev, MII_CTRL1000, adv);
		}
		phydev->supported &= ~(SUPPORTED_1000baseT_Half);
		phydev->supported &= ~(SUPPORTED_1000baseT_Full);
	}

	return rc;
}

/**
 * amac_gphy_handle_link_change() - Handles link change
 * @ndev - network device pointer
 */
static void amac_gphy_handle_link_change(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct phy_device *phydev;
	struct phy_priv *phypriv;
	int i;

	for (i = 0; i < privp->port.count; i++) {
		phydev = privp->port.info[i].phydev;
		phypriv = &privp->port.info[i].phy_info;

		/* Act on link, speed, duplex status changes */
		if ((phydev->link !=  phypriv->link) ||
			(phydev->speed !=  phypriv->speed) ||
			(phydev->duplex !=  phypriv->duplex)) {

			/* Update the new status */
			phypriv->link = phydev->link;
			phypriv->speed = phydev->speed;
			phypriv->duplex = phydev->duplex;

			/* send netlink update */
			bcm_amac_enet_send_link_stat(privp,
				i,
				phydev,
				phydev->link);
		}
	}
}

/**
 * amac_gphy_mii_probe() - Probes and connects the PHY's
 * @ndev - network device pointer
 *
 * Probes the MDIO bus for GPHY's, attaches them and starts
 * the auto-negotiation.
 *
 * Returns: '0' or error
 */
static int amac_gphy_mii_probe(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	struct phy_device *phy_dev = NULL;
	struct phy_device *phy_dev_mii = NULL;
	struct port_info *port_priv = NULL;
	unsigned int phy_idx, port_idx;
	int rc = 0;
	u32 valid_phy = privp->mii_bus->phy_mask;

printk("[ADK] %s: entered, mii_bus@%p, valid_phy=0x%x\n", __func__, (void *)privp->mii_bus, privp->mii_bus->phy_mask);

	/* Scan the bus, attach and start the PHY(s) */
	for (port_idx = 0; port_idx < privp->port.count; port_idx++) {
		port_priv = &privp->port.info[port_idx];
		port_priv->phydev = (struct phy_device *)NULL;

		/* Find the PHY device */
		for (phy_idx = 0; phy_idx < PHY_MAX_ADDR; phy_idx++) {

			/* Skip if phydev is not valid */
			if (valid_phy & (1 << phy_idx))
				continue;

			phy_dev_mii = privp->mii_bus->phy_map[phy_idx];

			if (port_priv->phy_id == phy_dev_mii->addr) {
printk("[ADK] %s/%d\n", __func__, __LINE__);
				/* Reset phy curr status */
				port_priv->phy_info.aneg =
					port_priv->phy_def.aneg;
				port_priv->phy_info.pause =
					port_priv->phy_def.pause;
				/* The below would get updated on link change */
				port_priv->phy_info.speed = 0;
				port_priv->phy_info.duplex = 0;
				port_priv->phy_info.link = 0;

				/* Attach the PHY to the MAC */
				phy_dev = phy_connect(ndev,
					dev_name(&phy_dev_mii->dev),
					amac_gphy_handle_link_change,
					PHY_INTERFACE_MODE_MII);

				if (IS_ERR(phy_dev)) {
					dev_err(&privp->pdev->dev,
						"Cannot connect to PHY: %d\n",
						phy_dev->addr);
					rc = -ENODEV;
					goto err_amac_phy_init;
				}

				/* Store PHY device */
				port_priv->phydev = phy_dev;

				/* Apply settings */
				phy_dev->autoneg = port_priv->phy_def.aneg;
				phy_dev->speed = port_priv->phy_def.speed;
				phy_dev->duplex = port_priv->phy_def.duplex;
				phy_dev->pause = port_priv->phy_def.pause;
				phy_dev->dev_flags = PHY_BRCM_DIS_PWR_UP_ON_RES;

				/* Disable 1G advertisement for 10/100M ports */
				if ((phy_dev->speed == AMAC_PORT_SPEED_100M) ||
					(phy_dev->speed == AMAC_PORT_SPEED_10M))
					amac_gphy_advertise_1G(phy_dev, false);

				/* Disable 100M advertisement for 10M ports */
				if (phy_dev->speed == AMAC_PORT_SPEED_10M)
					amac_gphy_advertise_100M(phy_dev,
						false);

				if (privp->reboot == AMAC_REBOOT_COLD) {
					rc = phy_start_aneg(phy_dev);
					if (rc < 0) {
						dev_err(&privp->pdev->dev,
							"Cannot start PHY: %d\n",
							phy_dev->addr);
						rc = -ENODEV;
						goto err_amac_phy_init;
					}
				} else {
					phy_dev->state = PHY_AN;
					phy_dev->link_timeout = PHY_AN_TIMEOUT;
				}
				dev_info(&privp->pdev->dev,
					"Initialized PHY: %s\n",
					dev_name(&phy_dev->dev));

				break; /* Configure next port */
			}
		}
	}

	return 0;

err_amac_phy_init:
	/* Disconnect all the PHY's */
	for (port_idx = 0; port_idx < privp->port.count; port_idx++) {
		port_priv = &privp->port.info[port_idx];

		if (port_priv->phydev)
			phy_disconnect(port_priv->phydev);
	}

	return rc;
}

/**
 * amac_gphy_mdio_read() - Reads data from the PHY register
 * @bp: mdio bus pointer
 * @phy_id: phy identifier
 * @reg: PHY's register address
 *
 * Reads data from the PHY using the MDIO bus.
 * This is passed to the PHY Driver
 *
 * Returns: register's value or error
 */
static int amac_gphy_mdio_read(struct mii_bus *bp, int phy_id, int reg)
{
	int rc;
	u16 val = 0;

/* [ADK]
	rc = iproc_mdio_read(GPHY_MDCDIV, phy_id, reg, &val);
	if (rc)
		return rc;
*/
	rc = mdiobus_read(bp, phy_id, reg);
	val = rc&0xffff;
	return val;
}

/**
 * amac_gphy_mdio_write() - Write data to the PHY register
 * @bp: mdio bus pointer
 * @phy_id: phy identifier
 * @reg: PHY's register address
 * @val: value to write
 *
 * Writes data to the PHY using the MDIO bus
 * This is passed to the PHY Driver
 *
 * Returns: '0' or error
 */
static int
amac_gphy_mdio_write(struct mii_bus *bp, int phy_id, int reg, u16 val)
{
	
// [ADK]	return iproc_mdio_write(GPHY_MDCDIV, phy_id, reg, val);
	mdiobus_write(bp, phy_id, reg, val);
	return 0;
}

/**
 * amac_gphy_lswap() - PHY laneswapping.
 * @bp: mdio bus pointer
 * @phyaddr: phy id
 */
static void amac_gphy_lswap(struct mii_bus *bp, u32 phyaddr)
{
	u16 val;
	int rc;

	/* We support only Port 0 and Port 1 */
	if (phyaddr > 1)
		return;

	(bp->write)(bp, phyaddr, GPHY_EXP_SELECT_REG, 0x0F09);
	rc = (bp->read)(bp, phyaddr, GPHY_EXP_DATA_REG);
	/* Ignoring the read err (if any), worst case the
	 * register gets overwritten
	 */

	if (phyaddr == 0)
		val = 0x5193;
	else
		val = 0x11C9;

	if (val != (u16)rc) {
		/* Apply the laneswap setting */
		(bp->write)(bp, phyaddr, GPHY_EXP_SELECT_REG, 0x0F09);
		(bp->write)(bp, phyaddr, GPHY_EXP_DATA_REG, val);
	}
}

/**
 * amac_gphy_mii_reset() - MDIO callback to reset the MDIO interface
 * @bp: mdio bus pointer
 *
 * Reads the 'ethlaneswap' seting and performs laneswapping.
 *
 * Returns: '0' or error
 */
static int amac_gphy_mii_reset(struct mii_bus *bp)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)bp->priv;
	int i;

	/* Apply laneswapping if required */
	if (g_lnswp)
		for (i = 0; i < privp->port.count; i++)
			amac_gphy_lswap(bp,
				privp->port.info[i].phy_id);

	return 0;
}

/**
 * bcm_amac_gphy_init() - Initialize the MDIO bus and PHY
 * @ndev: network device pointer
 *
 * Initialize the MII/MDIO interface and starts the probe
 * to find and connect the PHY's
 *
 * Returns: '0' or error
 */
int bcm_amac_gphy_init(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int err = 0, i;

#if 1
	char *compat = "brcm,iproc-mdio";
	struct device_node *dn = privp->pdev->dev.of_node;
	struct device *kdev = &privp->pdev->dev;
/* ----------- [ADK] --------- dump DTS ----------
	struct property *pp;
	char *cp;

	struct device_node *_dn = of_find_all_nodes(NULL);
	while (_dn) {
		
		if ((pp = of_find_property(_dn, "compatible", NULL)) != NULL) {
			cp = of_prop_next_string(pp, NULL);
			printk("[ADK]\t%s: name=[%s] compatible=[%s]\n", __func__, _dn->name, cp);
		}
		_dn = of_find_all_nodes(_dn);
	}
---------------------------------------- */

	privp->mdio_dn = of_find_compatible_node(NULL, NULL, compat);
	if (!privp->mdio_dn) {
		dev_err(kdev, "unable to find MDIO bus node\n");
		return -ENODEV;
	}
printk("[ADK] %s: got MDIO bus node..\n", __func__);

	privp->mii_bus = of_mdio_find_bus(privp->mdio_dn);
	if (!privp->mii_bus) {
		dev_err(kdev, "unable to find MDIO bus \n");
		return -EPROBE_DEFER;
	}

printk("[ADK] %s: got MDIO bus@%p\n", __func__, (void *)privp->mii_bus);

#if 0
	privp->mii_bus = mdiobus_alloc();
	if (privp->mii_bus == NULL) {
		dev_err(&privp->pdev->dev,
			"MII BUS Alloc Failed in %s\n", __func__);
		return -ENOMEM;
	}

	/* Initialize mdio bus structure */
	snprintf(privp->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x", "bcmgphy", 0);
	privp->mii_bus->name	= "bcm_gphy mdio bus";
	privp->mii_bus->priv	= (void *)privp;
	privp->mii_bus->parent	= (struct device *)&privp->pdev->dev;
	privp->mii_bus->read	= &amac_gphy_mdio_read;
	privp->mii_bus->write	= &amac_gphy_mdio_write;
	privp->mii_bus->reset	= &amac_gphy_mii_reset;
	privp->mii_bus->irq		= &privp->mdio_irq[0];
	privp->mii_bus->phy_mask = (u32)0xfffffff0;

	for (i = 0; i < 32; i++)
		privp->mii_bus->irq[i] = PHY_POLL;

	err = of_mdiobus_register(privp->mii_bus, privp->mdio_dn);
	if (err) {
		dev_err(kdev, "failed to register MDIO bus\n");
		return err;
	}

printk("[ADK] %s: register MDIO bus ..\n", __func__);

	/* Fetch the PHY phandle */
	privp->phy_dn = of_parse_phandle(dn, "phy-handle", 0);
	if (!privp->phy_dn) {
		dev_err(kdev, "failed to obtain a PHY ..\n");
		goto err_bus;
	}

printk("[ADK] %s: phy-handle@%p\n", __func__, (void *)privp->phy_dn);
#endif

#else
	err = mdiobus_register(privp->mii_bus);
	if (err) {
		dev_err(&privp->pdev->dev,
			"mdiobus_register Failed !! in %s\n", __func__);
		goto err_register;
	}
#endif

	err = amac_gphy_mii_probe(privp->ndev);
	if (err)
		goto err_bus;

	return 0;

err_bus:
	mdiobus_unregister(privp->mii_bus);
err_register:
	mdiobus_free(privp->mii_bus);

	return err;
}

/**
 * bcm_amac_gphy_enable() - Enable/disable the PHY
 * @privp: driver local data structure pointer
 * @phy: phy id
 * @enable: enable/disable the phy
 */
void bcm_amac_gphy_enable(struct bcm_amac_priv *privp, int phy, int enable)
{
	u32 val;

	/* Read current value */
	val = (privp->mii_bus->read)(privp->mii_bus, phy, GPHY_MII_CTRL_REG);

	/* Set or Clear the Power and Reset bits */
	if (enable)
		val &= (~(GPHY_MII_CTRL_REG_RST_MASK |
			GPHY_MII_CTRL_REG_PWR_MASK));
	else
		val |= (GPHY_MII_CTRL_REG_PWR_MASK);

	/* Write to MDIO bus */
	(privp->mii_bus->write)(privp->mii_bus,
		phy,
		GPHY_MII_CTRL_REG,
		val);
}

/**
 * bcm_amac_gphy_powerup() - Power up the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_powerup(struct bcm_amac_priv *privp)
{
	int port_idx;

	/* Power up all the PHY(s) */
	for (port_idx = 0; port_idx < privp->port.count; port_idx++)
		bcm_amac_gphy_enable(privp,
			privp->port.info[port_idx].phy_id,
			1);

	/* Apply the laneswap setting */
	amac_gphy_mii_reset(privp->mii_bus);
}

/**
 * bcm_amac_gphy_shutdown() - Reset and power down the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_shutdown(struct bcm_amac_priv *privp)
{
	int i;

	/* Reset and power down all the PHY */
	for (i = 0; i < privp->port.count; i++)
		bcm_amac_gphy_enable(privp, privp->port.info[i].phy_id, 0);
}

/**
 * bcm_amac_gphy_set_lswap() - Set the laneswap setting
 * @val: '0' - disable, '1'- enabled
 */
void bcm_amac_gphy_set_lswap(unsigned int val)
{
	g_lnswp = val;
}

/**
 * amac_gphy_restart_aneg() - Force auto negotiation restart for the PHY
 * @phydev: phy device pointer
 */
static void amac_gphy_restart_aneg(struct phy_device *phydev)
{
	int ctl;

	ctl = phy_read(phydev, MII_BMCR);
	if (ctl < 0)
		return;

	ctl |= (BMCR_ANENABLE | BMCR_ANRESTART);

	/* Don't isolate the PHY if we're negotiating */
	ctl &= ~(BMCR_ISOLATE);

	(void)phy_write(phydev, MII_BMCR, ctl);
}

/**
 * amac_ghy_change_speed() - Change the PHY speed
 * @phydev: phy device pointer
 * @speed: speed (10, 100, 1000)
 */
static int amac_ghy_change_speed(struct phy_device *phydev, u32 speed)
{
	int rc;

	/* Apply settings */
	phydev->speed = speed;

	if (phydev->speed != AMAC_PORT_SPEED_1G) {
		/* for 10M and 100M speeds, disable 1G advertisement */
		rc = amac_gphy_advertise_1G(phydev, false);
		if (rc)
			return rc;

		if (phydev->speed == AMAC_PORT_SPEED_10M) {
			/* for 10M speeds, disable 100M advertisement */
			rc = amac_gphy_advertise_100M(phydev, false);
			if (rc)
				return rc;
		} else if (phydev->speed == AMAC_PORT_SPEED_100M) {
			/* for 100M speeds, enable 100M advertisement
			 * (it could have been previously disabled)
			 */
			rc = amac_gphy_advertise_100M(phydev, true);
			if (rc)
				return rc;
		}
	} else {
		/* Add 100M support if it was disabled */
		rc = amac_gphy_advertise_100M(phydev, true);
		if (rc)
			return rc;

		/* Add 1G support back */
		rc = amac_gphy_advertise_1G(phydev, true);
		if (rc)
			return rc;
	}

	amac_gphy_restart_aneg(phydev);

	return rc;
}

/**
 * bcm_amac_gphy_enter_wol() - Enter Wake on Lan mode
 * @privp: driver local data structure pointer
 * @wol_port: the port to be used for wol
 * @speed: speed of the wol port
 *
 * Enter the WOL (Wake On LAN) mode.
 * In WOL mode only one port is enabled. Others are powered down.
 * Typically the port that is enabled is configured at a lower speed
 * to save power.
 */
void bcm_amac_gphy_enter_wol(struct bcm_amac_priv *privp,
	u8 wol_port, u32 speed)
{
	u8 i;
	int wol_port_idx = -1;

	/* If WOL was previously entered, check to see if the port has
	 * changed. Although this is an unlikely scenario, this requires
	 * exiting the wol to work.
	 *
	 * For example if the WOL is enabled with Port 0 and then the
	 * WOL 'enter' is called again for Port 1, this will work only
	 * after calling wol exit before using Port 1 as the WOL port.
	 *
	 * So to handle this scenario, we call the wol exit here.
	 */
	if (privp->port.wol_en)
		for (i = 0; i < privp->port.count; i++)
			if (privp->port.info[i].wol)
				if (wol_port != privp->port.info[i].num) {
					/* Change the speed back to default */
					bcm_amac_gphy_exit_wol(privp);
					break;
				}

	/* locking to prevent race with exit wol */
	mutex_lock(&privp->port.wol_lock);

	/* Validate the requested WOL port */
	for (i = 0; i < privp->port.count; i++)
		/* Find the requested port and enter WOL only if the
		 * port speed is different or WOL is disabled.
		 */
		if ((wol_port == privp->port.info[i].num) &&
			((privp->port.info[i].phy_info.speed != speed) ||
			(privp->port.info[i].wol == AMAC_WOL_DISABLE)))
				wol_port_idx = i;

	if (wol_port_idx < 0)
		goto err_wol_enter;

	/* Power down all the other PHY's */
	for (i = 0; i < privp->port.count; i++)
		if (i != wol_port_idx) {
			bcm_amac_gphy_enable(privp,
				privp->port.info[i].phy_id,
				BCM_GPHY_DISABLE);
			privp->port.info[i].wol = AMAC_WOL_DISABLE;
		}

	/* Restart ANEG for the wol port with new speed */
	amac_ghy_change_speed(
		privp->port.info[wol_port_idx].phydev,
		speed);

	privp->port.wol_en = AMAC_WOL_ENABLE;
	privp->port.info[wol_port_idx].wol = AMAC_WOL_ENABLE;

err_wol_enter:
	mutex_unlock(&privp->port.wol_lock);
}

/**
 * bcm_amac_gphy_exit_wol() - Exit WoL mode
 * @privp: driver local data structure pointer
 *
 * Exit the WOL (Wake On LAN) mode.
 * Finds and restores the wol port to default speed. Enables all other
 * ports.
 */
void bcm_amac_gphy_exit_wol(struct bcm_amac_priv *privp)
{
	int wol_port = -1;
	int i;

	/* locking to prevent race with enter wol */
	mutex_lock(&privp->port.wol_lock);

	/* Find the port that is enabled for WOL */
	for (i = 0; i < privp->port.count; i++)
		if (privp->port.info[i].wol)
			wol_port = i;

	if (wol_port < 0)
		goto err_wol_unlock;

	/* Restart ANEG for the wol port to default speed */
	amac_ghy_change_speed(privp->port.info[wol_port].phydev,
		privp->port.info[wol_port].phy_def.speed);

	privp->port.wol_en = AMAC_WOL_DISABLE;
	privp->port.info[wol_port].wol = AMAC_WOL_DISABLE;

	/* Power up the other PHY's */
	for (i = 0; i < privp->port.count; i++)
		if (i != wol_port)
			bcm_amac_gphy_enable(privp,
				privp->port.info[i].phy_id,
				BCM_GPHY_ENABLE);

err_wol_unlock:
	mutex_unlock(&privp->port.wol_lock);
}

/**
 * bcm_amac_gphy_stop_phy() - stop the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_stop_phy(struct bcm_amac_priv *privp)
{
	int i;

	for (i = 0; i < privp->port.count; i++)
		phy_stop(privp->port.info[i].phydev);
}

/**
 * bcm_amac_gphy_resume() - resume the PHY's
 * @privp: driver local data structure pointer
 */
void bcm_amac_gphy_resume(struct bcm_amac_priv *privp)
{
	int i;
	struct phy_device *phy_dev = NULL;
	struct port_info *port_priv = NULL;

	mutex_lock(&privp->port.wol_lock);

	for (i = 0; i < privp->port.count; i++) {
		if (privp->port.info[i].phydev) {

			phy_dev = privp->port.info[i].phydev;
			port_priv = &privp->port.info[i];

			/* Check for WoL config */
			if (AMAC_WOL_ENABLE == privp->port.wol_en) {
				if (AMAC_WOL_DISABLE == port_priv->wol) {
					bcm_amac_gphy_enable(privp,
						port_priv->phy_id,
						BCM_GPHY_DISABLE);
					continue;
				}
			}

			/* Disable 1G advertisement for 10/100M ports */
			if ((phy_dev->speed == AMAC_PORT_SPEED_100M) ||
				(phy_dev->speed == AMAC_PORT_SPEED_10M))
				amac_gphy_advertise_1G(phy_dev, false);

			/* Disable 100M advertisement for 10M ports */
			if (phy_dev->speed == AMAC_PORT_SPEED_10M)
				amac_gphy_advertise_100M(phy_dev,
					false);

			phy_start(phy_dev);

			bcm_amac_gphy_enable(privp,
						port_priv->phy_id,
						BCM_GPHY_ENABLE);
		}
	}

	mutex_unlock(&privp->port.wol_lock);

	/* Apply the laneswap setting */
	amac_gphy_mii_reset(privp->mii_bus);
}

