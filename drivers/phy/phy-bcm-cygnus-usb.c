/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>

#define CDRU_USBPHY_CLK_RST_SEL_OFFSET			0x11b4
#define CDRU_USBPHY2_HOST_DEV_SEL_OFFSET		0x11b8
#define CDRU_SPARE_REG_0_OFFSET				0x1238
#define CRMU_USB_PHY_AON_CTRL_OFFSET			0x00028
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET		0x1210
#define CDRU_USBPHY_P0_STATUS_OFFSET			0x11D0
#define CDRU_USBPHY_P1_STATUS_OFFSET			0x11E8
#define CDRU_USBPHY_P2_STATUS_OFFSET			0x1200

#define CDRU_USBPHY_USBPHY_ILDO_ON_FLAG			1
#define CDRU_USBPHY_USBPHY_PLL_LOCK			0

#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE	0
#define PHY2_DEV_HOST_CTRL_SEL_DEVICE			0
#define PHY2_DEV_HOST_CTRL_SEL_HOST			1
#define PHY2_DEV_HOST_CTRL_SEL_IDLE			2
#define CDRU_USBPHY_P2_STATUS__USBPHY_ILDO_ON_FLAG	1
#define CDRU_USBPHY_P2_STATUS__USBPHY_PLL_LOCK		0
#define CRMU_USBPHY_P0_AFE_CORERDY_VDDC			1
#define CRMU_USBPHY_P0_RESETB				2
#define CRMU_USBPHY_P1_AFE_CORERDY_VDDC			9
#define CRMU_USBPHY_P1_RESETB				10
#define CRMU_USBPHY_P2_AFE_CORERDY_VDDC			17
#define CRMU_USBPHY_P2_RESETB				18

#define USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET		0x0408
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable	0
#define USB2_IDM_IDM_RESET_CONTROL_OFFSET		0x0800
#define USB2_IDM_IDM_RESET_CONTROL__RESET		0

#define PLL_LOCK_RETRY_COUNT				1000
#define MAX_REGULATOR_NAME_LEN				25

#define DUAL_ROLE_PHY					2

static int status_reg[] = {CDRU_USBPHY_P0_STATUS_OFFSET,
				CDRU_USBPHY_P1_STATUS_OFFSET,
				CDRU_USBPHY_P2_STATUS_OFFSET};

struct bcm_phy_instance;

struct bcm_phy_driver {
	void __iomem *usbphy_regs;
	void __iomem *usb2h_idm_regs;
	void __iomem *usb2d_idm_regs;
	spinlock_t lock;
	int num_phys, idm_host_enabled;
	struct bcm_phy_instance *instances;
	struct extcon_specific_cable_nb extcon_dev;
	struct extcon_specific_cable_nb extcon_host;
	struct notifier_block host_nb;
	struct notifier_block dev_nb;
	struct work_struct conn_work;
	bool dual_role_enable;
};

struct bcm_phy_instance {
	struct bcm_phy_driver *driver;
	struct phy *generic_phy;
	int port;
	int new_state; /* 1 - Host , 0 - device, 2 - idle*/
	int current_state; /* 1 - Host , 0 - device, 2 - idle*/
	int power; /* 1 -powered_on 0 -powered off */
	struct regulator *vbus_supply;
};

enum {
	EXTCON_CABLE_USB = 0,
	EXTCON_CABLE_USB_HOST,
};

static inline int bcm_phy_cdru_usbphy_status_wait(u32 usb_reg, int reg_bit,
					  struct bcm_phy_driver *phy_driver)
{
	/* Wait for the PLL lock status */
	int retry = PLL_LOCK_RETRY_COUNT;
	u32 reg_val;

	do {
		udelay(1);
		reg_val = readl(phy_driver->usbphy_regs +
				usb_reg);
		if (reg_val & (1 << reg_bit))
			return 0;
	} while (--retry > 0);

	return -EBUSY;

}

static struct phy *bcm_usb_phy_xlate(struct device *dev,
				     struct of_phandle_args *args)
{
	struct bcm_phy_driver *phy_driver = dev_get_drvdata(dev);

	if (!phy_driver)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args[0] >= phy_driver->num_phys))
		return ERR_PTR(-ENODEV);

	if (WARN_ON(args->args[1] < 0 || args->args[1] > 1))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args_count < 2))
		return ERR_PTR(-EINVAL);

	phy_driver->instances[args->args[0]].port = args->args[0];
	if (phy_driver->dual_role_enable && args->args[0] == 2)
		goto ret_p2;
	phy_driver->instances[args->args[0]].new_state = args->args[1];

ret_p2:
	return phy_driver->instances[args->args[0]].generic_phy;
}

static int bcm_phy_init(struct phy *generic_phy)
{
	u32 reg_val;
	unsigned long flags;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	spin_lock_irqsave(&phy_driver->lock, flags);

	/* Only PORT 2 is capabale of being device and host
	 * Default setting is device, check if it is set to host */
	if (instance_ptr->port == 2) {
		if (instance_ptr->new_state == PHY2_DEV_HOST_CTRL_SEL_HOST)
			writel(PHY2_DEV_HOST_CTRL_SEL_HOST,
				phy_driver->usbphy_regs +
				CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
		else {
			/* Disable suspend/resume signals to device controller
			when a port is in device mode  */
			writel(PHY2_DEV_HOST_CTRL_SEL_DEVICE,
				phy_driver->usbphy_regs +
				CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
			reg_val = readl(phy_driver->usbphy_regs +
				      CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
			reg_val |=
			  (1 << CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE);
			writel(reg_val, phy_driver->usbphy_regs +
			       CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
		}
	}

	spin_unlock_irqrestore(&phy_driver->lock, flags);
	return 0;
}

static int bcm_phy_shutdown(struct phy *generic_phy)
{

	u32 reg_val, powered_on_phy;
	int i, power_off_flag = 1;
	unsigned long flags;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;
	u32 current_connect = instance_ptr->current_state;
	u32 extcon_event = instance_ptr->new_state;

	/*
	 * Shutdown regulator only if transition from HOST->IDLE
	 */
	if (instance_ptr->vbus_supply &&
		current_connect == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		regulator_disable(instance_ptr->vbus_supply);
		if (instance_ptr->port == 2 &&
			phy_driver->dual_role_enable)
			goto shutdown;
	}


	spin_lock_irqsave(&phy_driver->lock, flags);

	/* power down the phy */
	reg_val = readl(phy_driver->usbphy_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0) {
		reg_val &= ~(1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P0_RESETB);
	} else if (instance_ptr->port == 1) {
		reg_val &= ~(1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P1_RESETB);
	} else if (instance_ptr->port == 2) {
		reg_val &= ~(1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P2_RESETB);
	}
	writel(reg_val, phy_driver->usbphy_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	instance_ptr->power = 0;

	/*
	 * if the transitions is from DEVICE->IDLE  and it is being shutdown,
	 * turn off the clocks to the usb device controller
	 */
	if (instance_ptr->port == 2 &&
		current_connect == PHY2_DEV_HOST_CTRL_SEL_DEVICE) {
		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2d_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
	} else {

		/*
		 * Transitions is from HOST->IDLE.
		 * If the phy being shutdown provides clock and reset to
		 * the host controller, change it do a different powered on phy
		 * If all phys are powered off, shut of the host controller
		 */
		reg_val = readl(phy_driver->usbphy_regs +
				CDRU_USBPHY_CLK_RST_SEL_OFFSET);
		powered_on_phy = reg_val;
		if (reg_val == instance_ptr->port) {
			for (i = 0; i < phy_driver->num_phys; i++) {
				if (phy_driver->instances[i].power == 1 &&
				phy_driver->instances[i].new_state == 1) {
					power_off_flag = 0;
					powered_on_phy = i;
				}
			}
		}

		if (power_off_flag) {
			/* Put the host controller into reset
			state and disable clock */
			reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
			reg_val &=
			  ~(1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
			writel(reg_val, phy_driver->usb2h_idm_regs +
					USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

			reg_val = readl(phy_driver->usb2h_idm_regs +
					 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
			reg_val |= (1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
			writel(reg_val, phy_driver->usb2h_idm_regs +
					USB2_IDM_IDM_RESET_CONTROL_OFFSET);
			phy_driver->idm_host_enabled = 0;
		} else
			writel(powered_on_phy, phy_driver->usbphy_regs +
				   CDRU_USBPHY_CLK_RST_SEL_OFFSET);
	}

	spin_unlock_irqrestore(&phy_driver->lock, flags);

shutdown:
	instance_ptr->current_state = extcon_event;
	return 0;
}

static int bcm_phy_poweron(struct phy *generic_phy)
{
	int ret, clock_reset_flag = 1;
	unsigned long flags;
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;
	u32 extcon_event = instance_ptr->new_state;

	/*
	 * Switch on the regulator only if in HOST mode
	 */
	if (instance_ptr->vbus_supply &&
			extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		ret = regulator_enable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
				"failed to enable regulator\n");
			return ret;
		}
	}

	spin_lock_irqsave(&phy_driver->lock, flags);

	/* Bring the AFE block out of reset to start powering up the PHY */
	reg_val = readl(phy_driver->usbphy_regs + CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0)
		reg_val |= (1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 1)
		reg_val |= (1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 2)
		reg_val |= (1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	writel(reg_val, phy_driver->usbphy_regs + CRMU_USB_PHY_AON_CTRL_OFFSET);

	/* Check for power on and PLL lock */
	ret = bcm_phy_cdru_usbphy_status_wait(status_reg[instance_ptr->port],
		   CDRU_USBPHY_USBPHY_ILDO_ON_FLAG, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_ILDO_ON_FLAG on port %d",
			instance_ptr->port);
		goto err_shutdown;
	}
	ret = bcm_phy_cdru_usbphy_status_wait(status_reg[instance_ptr->port],
		CDRU_USBPHY_USBPHY_PLL_LOCK, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_PLL_LOCK on port %d",
			instance_ptr->port);
		goto err_shutdown;
	}

	instance_ptr->power = 1;

	/* Check if the port 2 is configured for device */
	if (instance_ptr->port == 2 &&
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_DEVICE) {

		/* Enable clock to USB device and get
		 * the USB device out of reset */
		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);

	} else {
		reg_val = readl(phy_driver->usbphy_regs +
				CDRU_USBPHY_CLK_RST_SEL_OFFSET);

		/* Check if the phy that is configured
		 * to provide clock and reset is powered on*/
		if (reg_val >= 0 && reg_val < phy_driver->num_phys) {
			if (phy_driver->instances[reg_val].power == 1)
				clock_reset_flag = 0;
		}

		/* if not set the current phy */
		if (clock_reset_flag) {
			reg_val = instance_ptr->port;
			writel(reg_val, phy_driver->usbphy_regs +
			       CDRU_USBPHY_CLK_RST_SEL_OFFSET);
		}
	}

	if (phy_driver->idm_host_enabled != 1 &&
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		/* Enable clock to USB and get the USB out of reset */
		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = 1;
	}

	instance_ptr->current_state = extcon_event;
	spin_unlock_irqrestore(&phy_driver->lock, flags);
	return 0;

err_shutdown:
	spin_unlock_irqrestore(&phy_driver->lock, flags);
	bcm_phy_shutdown(generic_phy);
	return ret;
}

static void connect_work(struct work_struct *work)
{
	struct bcm_phy_driver *phy_driver =
		container_of(work, struct bcm_phy_driver, conn_work);
	struct bcm_phy_instance instance_ptr =
		phy_driver->instances[DUAL_ROLE_PHY];
	u32 extcon_event = instance_ptr.new_state;

	if (extcon_event == PHY2_DEV_HOST_CTRL_SEL_DEVICE ||
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		bcm_phy_init(instance_ptr.generic_phy);
		bcm_phy_poweron(instance_ptr.generic_phy);
	} else if (extcon_event == PHY2_DEV_HOST_CTRL_SEL_IDLE)
		bcm_phy_shutdown(instance_ptr.generic_phy);
}

static int usbp2_dev_connect_notifier(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	unsigned int conn_type;
	struct bcm_phy_driver *phy_driver =
			container_of(self, struct bcm_phy_driver, dev_nb);
	conn_type = event;

	if (conn_type) {
		phy_driver->instances[DUAL_ROLE_PHY].new_state =
			PHY2_DEV_HOST_CTRL_SEL_DEVICE;
		schedule_work(&phy_driver->conn_work);
	} else {
		phy_driver->instances[DUAL_ROLE_PHY].new_state =
			PHY2_DEV_HOST_CTRL_SEL_IDLE;
		schedule_work(&phy_driver->conn_work);
	}
	return NOTIFY_DONE;
}

static int usbp2_host_connect_notifier(struct notifier_block *self,
			unsigned long event, void *ptr)
{
	unsigned int conn_type;
	struct bcm_phy_driver *phy_driver =
			container_of(self, struct bcm_phy_driver, host_nb);
	conn_type = event;
	if (conn_type) {
		phy_driver->instances[DUAL_ROLE_PHY].new_state =
			PHY2_DEV_HOST_CTRL_SEL_HOST;
		schedule_work(&phy_driver->conn_work);
	} else {
		phy_driver->instances[DUAL_ROLE_PHY].new_state =
			PHY2_DEV_HOST_CTRL_SEL_IDLE;
		schedule_work(&phy_driver->conn_work);
	}

	return NOTIFY_DONE;
}

static int phy_usb_register_extcon(struct bcm_phy_driver *phy_driver,
				struct device *dev)
{
	int ret = 0;
	struct extcon_dev *edev;
	phy_driver->host_nb.notifier_call = usbp2_host_connect_notifier;
	phy_driver->dev_nb.notifier_call = usbp2_dev_connect_notifier;

	if (of_property_read_bool(dev->of_node, "extcon")) {
		edev = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(edev)) {
			dev_err(dev, "couldn't get extcon device\n");
			return -EPROBE_DEFER;
		}

		/*
		 * Register for the USB device mode connection
		 * state change notification
		 */
		ret = extcon_register_interest(&phy_driver->extcon_dev,
			edev->name, 
			"USB",
			&phy_driver->dev_nb);
		if (ret < 0) {
			pr_info("(%d) Cannot register extcon_dev for %s.\n", __LINE__, edev->name);
			ret = -EINVAL;
			goto ret_err;
		}

		/*
		 * Register for the USB host mode connection
		 * state change notification
		 */
		ret = extcon_register_interest(&phy_driver->extcon_host,
				edev->name,
				"USB-HOST",
				&phy_driver->host_nb);
		if (ret < 0) {
			pr_info("(%d) Cannot register extcon_dev for %s.\n", __LINE__, edev->name);
			ret = -EINVAL;
			goto ret_err;
		}

		/*
		 * Check the current state
		 * of device cable connection
		 */
		ret = extcon_get_cable_state(edev, "USB");
		if (ret < 0) {
			pr_info("Cannot get extcon_dev state for %s.\n", edev->name);
			ret = -EINVAL;
			goto ret_err;
		} else if (ret) {
			phy_driver->instances[DUAL_ROLE_PHY].new_state =
				PHY2_DEV_HOST_CTRL_SEL_DEVICE;
		}

		/*
		 * Check the host cable connect state
		 */
		ret = extcon_get_cable_state(edev, "USB-HOST");
		if (ret < 0) {
			pr_info("Cannot get extcon_dev state for %s.\n", edev->name);
			ret = -EINVAL;
			goto ret_err;
		} else if (ret) {
			phy_driver->instances[DUAL_ROLE_PHY].new_state =
				PHY2_DEV_HOST_CTRL_SEL_HOST;
		}
	}
ret_err:
	return ret;
}

static struct phy_ops ops = {
	.init		= bcm_phy_init,
	.power_on	= bcm_phy_poweron,
	.power_off	= bcm_phy_shutdown,
};

static const struct of_device_id bcm_phy_dt_ids[] = {
	{ .compatible = "brcm,cygnus-usb-phy", },
	{ }
};

static int bcm_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource res;
	struct bcm_phy_driver *phy_driver;
	struct phy_provider *phy_provider;
	int ret, num_phys, i;
	u32 reg_val;

	/* get number of phy cells */
	ret = of_property_read_u32(dev->of_node, "num-phys", &num_phys);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource num-phys\n");
		return ret;
	}

	/* allocate memory for each phy instance */
	phy_driver = devm_kzalloc(dev, sizeof(struct bcm_phy_driver),
				  GFP_KERNEL);
	if (!phy_driver)
		return -ENOMEM;

	phy_driver->instances = devm_kcalloc(dev, num_phys,
					     sizeof(struct bcm_phy_instance),
					     GFP_KERNEL);
	phy_driver->num_phys = num_phys;

	phy_driver->dual_role_enable = of_property_read_bool(dev->of_node,
				"enable-dual-role");

	spin_lock_init(&phy_driver->lock);

	ret = of_address_to_resource(dev->of_node, 0, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usbphy_regs\n");
		return ret;
	}
	phy_driver->usbphy_regs  = devm_ioremap_nocache(dev, res.start,
						resource_size(&res));
	if (!phy_driver->usbphy_regs) {
		dev_err(dev, "Failed to remap usbphy_regs\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(dev->of_node, 1, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usb2h_idm_regs\n");
		return ret;
	}
	phy_driver->usb2h_idm_regs = devm_ioremap_nocache(dev, res.start,
						  resource_size(&res));
	if (!phy_driver->usb2h_idm_regs) {
		dev_err(dev, "Failed to remap usb2h_idm_regs\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(dev->of_node, 2, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usb2d_idm_regs\n");
		return ret;
	}
	phy_driver->usb2d_idm_regs = devm_ioremap_nocache(dev, res.start,
						   resource_size(&res));
	if (!phy_driver->usb2d_idm_regs) {
		dev_err(dev, "Failed to remap usb2d_idm_regs\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, phy_driver);
	phy_driver->idm_host_enabled = 0;

	/* Shutdown all ports. They can be powered up as
	 * required */
	reg_val = readl(phy_driver->usbphy_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	reg_val &= ~(1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P0_RESETB);
	reg_val &= ~(1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P1_RESETB);
	reg_val &= ~(1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P2_RESETB);
	writel(reg_val, phy_driver->usbphy_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	for (i = 0; i < phy_driver->num_phys; i++) {
		char *vbus_name;
		struct bcm_phy_instance *instance_ptr =
					&phy_driver->instances[i];

		vbus_name = devm_kzalloc(dev, MAX_REGULATOR_NAME_LEN,
					 GFP_KERNEL);
		if (!vbus_name)
			return -ENOMEM;

		/* regulator use is optional */
		sprintf(vbus_name, "vbus-p%d", i);
		instance_ptr->vbus_supply = devm_regulator_get(dev, vbus_name);
		if (IS_ERR(instance_ptr->vbus_supply))
			instance_ptr->vbus_supply = NULL;
		devm_kfree(dev, vbus_name);

		instance_ptr->generic_phy = devm_phy_create(dev, NULL, &ops);

		if (IS_ERR(instance_ptr->generic_phy)) {
			dev_err(dev, "Failed to create usb phy %d", i);
			return PTR_ERR(instance_ptr->generic_phy);
		}
		instance_ptr->driver = phy_driver;
		phy_set_drvdata(instance_ptr->generic_phy, instance_ptr);
	}

	phy_provider = devm_of_phy_provider_register(dev,
					bcm_usb_phy_xlate);

	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		return PTR_ERR(phy_provider);
	}

	INIT_WORK(&phy_driver->conn_work, connect_work);
	platform_set_drvdata(pdev, phy_driver);

	/*
	 * If dual role is enabled, registering
	 * for extcon notifiers
	 */
	if (phy_driver->dual_role_enable) {
		ret = phy_usb_register_extcon(phy_driver, dev);
		if (ret < 0)
			return ret;
	}

	return 0;

}

MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver bcm_phy_driver = {
	.probe = bcm_phy_probe,
	.driver = {
		.name = "bcm-cygnus-usbphy",
		.of_match_table = of_match_ptr(bcm_phy_dt_ids),
	},
};
module_platform_driver(bcm_phy_driver);

MODULE_ALIAS("platform:bcm-cygnus-usbphy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus USB PHY driver");
MODULE_LICENSE("GPL V2");
