/*
 * Copyright (C) 2015 Broadcom Corporation
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
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/extcon.h>

#define USB2_SEL_DEVICE			0
#define USB2_SEL_HOST			1
#define USB2_SEL_IDLE			2

#define USB_CONNECTED			1
#define USB_DISCONNECTED		0

struct bcm_usb_data {
	struct extcon_dev *edev;
	struct gpio_desc *vbus_gpio;
	struct gpio_desc *usbid_gpio;
#ifdef CONFIG_ARCH_BCM_INVENTEC
	struct gpio_desc *vbus_enable_gpio;
#endif
	spinlock_t lock;
	unsigned int connect_mode;
#ifdef CONFIG_ARCH_BCM_INVENTEC
	unsigned int enable_high;
#endif
};


static unsigned int const usbp2_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
#ifdef CONFIG_ARCH_BCM_INVENTEC
	EXTCON_DOCK,
#endif
	EXTCON_NONE,
};

static irqreturn_t usbp2_vbus_id_isr(int irq, void *data)
{
	struct platform_device *pdev = (struct platform_device *)data;
	struct bcm_usb_data *usb_data;
	unsigned int vbus_presence, usb_id, conn_mode;
	usb_data = (struct bcm_usb_data *)platform_get_drvdata(pdev);

	/*
	 * Get the vbus and id gpio values
	 * ID = 0 ==> Host
	 * ID = 1 and VBUS = 1 ==> Device
	 * ID = 1 and VBUS = 0 ==> No connection(idle)
	 */
	vbus_presence = gpiod_get_raw_value(usb_data->vbus_gpio);
	usb_id = gpiod_get_raw_value(usb_data->usbid_gpio);
	conn_mode = usb_data->connect_mode;

	switch (usb_data->connect_mode) {
	case USB2_SEL_IDLE:
	if (usb_id == 1) {
		if (vbus_presence == 1) {
			conn_mode = USB2_SEL_DEVICE;
			extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, false);
			extcon_set_cable_state_(usb_data->edev, EXTCON_USB, true);
		} else
			conn_mode = USB2_SEL_IDLE;
	} else {
		conn_mode = USB2_SEL_HOST;
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, true);
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB, false);
	}
	break;

	case USB2_SEL_DEVICE:
	if (vbus_presence == 0) {
		conn_mode = USB2_SEL_IDLE;
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, false);
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB, false);
	} else
		conn_mode = USB2_SEL_DEVICE;
	break;

	case USB2_SEL_HOST:
	if (usb_id == 1) {
		conn_mode = USB2_SEL_IDLE;
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, false);
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB, false);
	} else
		conn_mode = USB2_SEL_HOST;
	break;

	default: 
		{
			struct device *dev = &pdev->dev;
			dev_err(dev, "Invalid case\n");
		}
	}

	usb_data->connect_mode = conn_mode;

	return IRQ_HANDLED;
}

static void bcm_usb_init_state(struct bcm_usb_data *usb_data)
{
	unsigned int vbus_presence, usb_id;
#ifdef CONFIG_ARCH_BCM_INVENTEC
printk("[ADK]\t%s() entered ..\n", __func__);

	if (usb_data->enable_high) {
printk("[ADK]\t%s()/%d \n", __func__, __LINE__);
		gpiod_set_value_cansleep(usb_data->vbus_enable_gpio, 1);
	}
#endif
	/*
	 * Check the initial state of vbus and id
	 * This is required if cable is kept connected
	 * when booting
	 */
	vbus_presence = gpiod_get_raw_value(usb_data->vbus_gpio);
	usb_id = gpiod_get_raw_value(usb_data->usbid_gpio);

	if (usb_id == 0) {
		usb_data->connect_mode = USB2_SEL_HOST;
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB, false);
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, true);
		
	} else if (vbus_presence == 1) {
		usb_data->connect_mode = USB2_SEL_DEVICE;
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB_HOST, false);
		extcon_set_cable_state_(usb_data->edev, EXTCON_USB, true);
	} else
		usb_data->connect_mode = USB2_SEL_IDLE;
}

static int bcm_extcon_usb_driver_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm_usb_data *usb_data;
	unsigned int vbus_intr, id_intr;
	int ret= -EINVAL;
	
printk("[ADK] %s() entered ..\n", __func__);

	usb_data = devm_kzalloc(dev, sizeof(struct bcm_usb_data),
				  GFP_KERNEL);
	if (!usb_data)
		return -ENOMEM;

printk("[ADK] %s()/%d \n", __func__, __LINE__);

	usb_data->edev = devm_extcon_dev_allocate(dev, usbp2_cable);
	if (IS_ERR(usb_data->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}
printk("[ADK] %s()/%d \n", __func__, __LINE__);

	ret = devm_extcon_dev_register(dev, usb_data->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		return ret;
	}
printk("[ADK] %s()/%d \n", __func__, __LINE__);

	spin_lock_init(&usb_data->lock);

	/* GPIO 145 - vbus, IN */
	usb_data->vbus_gpio = devm_gpiod_get(&pdev->dev, "vbus", GPIOD_IN);
	if (IS_ERR(usb_data->vbus_gpio)) {
		dev_err(dev, "failed to get VBUS GPIO\n");
		return PTR_ERR(usb_data->vbus_gpio);
	}
	vbus_intr = gpiod_to_irq(usb_data->vbus_gpio);
printk("[ADK] %s()/%d \n", __func__, __LINE__);



	/* GPIO 146 - usb id, IN */
	usb_data->usbid_gpio = devm_gpiod_get(&pdev->dev, "id", GPIOD_IN);
	if (IS_ERR(usb_data->usbid_gpio)) {
		dev_err(dev, "failed to get ID GPIO\n");
		return PTR_ERR(usb_data->usbid_gpio);
	}

	id_intr = gpiod_to_irq(usb_data->usbid_gpio);
printk("[ADK] %s()/%d \n", __func__, __LINE__);

#ifdef CONFIG_ARCH_BCM_INVENTEC
	/* GPIO 7 - vbus enable, OUT */
	if (of_property_read_bool(pdev->dev.of_node, "enable-active-high")) {
		usb_data->enable_high = true;
		usb_data->vbus_enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_HIGH);
	}else{
		usb_data->vbus_enable_gpio = devm_gpiod_get(&pdev->dev, "enable", GPIOD_OUT_LOW);
	}
	if (IS_ERR(usb_data->vbus_enable_gpio)) {
		dev_err(dev, "failed to get VBUS ENABLE GPIO\n");
		return PTR_ERR(usb_data->vbus_enable_gpio);
	}
printk("[ADK] %s()/%d, enable_high=%d\n", __func__, __LINE__, usb_data->enable_high);

#endif

	platform_set_drvdata(pdev, usb_data);

	/* set the initial vbus and id values */
	bcm_usb_init_state(usb_data);

	ret = devm_request_threaded_irq(dev, vbus_intr, NULL,
			usbp2_vbus_id_isr,
			IRQ_TYPE_EDGE_BOTH | IRQF_ONESHOT,
			"vbus_presence", pdev);
	if (ret)
		dev_err(dev, "VBUS_PRESENCE irq %d request failed\n",
			vbus_intr);

	ret = devm_request_threaded_irq(dev, id_intr, NULL,
			usbp2_vbus_id_isr,
			IRQ_TYPE_EDGE_BOTH | IRQF_ONESHOT,
			"usb_id", pdev);
	if (ret) {
		dev_err(dev, "USB_ID irq %d request failed\n", id_intr);
	}
printk("[ADK] %s() finished OK\n", __func__);
	return 0;
}

static int bcm_extcon_usb_driver_remove(struct platform_device *pdev)
{
	struct bcm_usb_data *usb_data = platform_get_drvdata(pdev);
	extcon_dev_unregister(usb_data->edev);
	return 0;
}

static const struct of_device_id extcon_usb_dt_ids[] = {
	{ .compatible = "brcm,extcon_cygnus_usb", },
	{ }
};

MODULE_DEVICE_TABLE(of, extcon_usb_dt_ids);

static struct platform_driver bcm_extcon_usb_cygnus_driver = {
	.probe = bcm_extcon_usb_driver_probe,
	.remove = bcm_extcon_usb_driver_remove,
	.driver = {
		.name = "bcm-extcon-cygnus-usb",
		.of_match_table = of_match_ptr(extcon_usb_dt_ids),
	},
};
module_platform_driver(bcm_extcon_usb_cygnus_driver);

MODULE_ALIAS("platform:bcm-extcon-cygnus-usb");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus Extcon driver");
MODULE_LICENSE("GPL V2");
