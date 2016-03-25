/******************************************************************************
* Copyright 2014 Broadcom Corporation.  All rights reserved.
*
*      @file   /kernel/drivers/misc/bcmbt_rfkill.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
******************************************************************************/

/*
 * Broadcom Bluetooth rfkill power control via GPIO.  The GPIOs are
 * configured through Device Tree
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/device.h>
#include "../gpio/gpiolib.h"


#define MAX_REGULATOR_NAME_LEN  25

static struct device *bcmbt_device;
static struct class *bcmbt_class;
static int bcmbt_dev_wake_gpio;
static int bcmbt_host_wake_gpio;

static ssize_t show_dev_wake_value(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", bcmbt_dev_wake_gpio);
}

static ssize_t store_dev_wake_value(struct device *device,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	int value;
	if (kstrtoint(buf, 10, &value) != 0)
		dev_err(device, "Unable to read value\n");
	else
		bcmbt_dev_wake_gpio = value;
	return count;
}

static ssize_t store_host_wake_value(struct device *device,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	int value;
	if (kstrtoint(buf, 10, &value) != 0)
		dev_err(device, "Unable to read value\n");
	else
		bcmbt_host_wake_gpio = value;
	return count;
}

static ssize_t show_host_wake_value(struct device *device,
				    struct device_attribute *attr,
				    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", bcmbt_host_wake_gpio);
}

static struct device_attribute device_attrs[] = {
	__ATTR(dev_wake,
	       S_IRUGO|S_IWUSR,
	       show_dev_wake_value,
	       store_dev_wake_value),
	__ATTR(host_wake,
	       S_IRUGO|S_IWUSR,
	       show_host_wake_value,
	       store_host_wake_value),
};

struct bcm_rfkill_instance {
	struct regulator *bt_reg;
};


struct bcmbt_rfkill_platform_data {
	struct device *dev;
	struct rfkill *rfkill;
	int num_rf_regs;
	struct bcm_rfkill_instance *instances;
	struct gpio_desc *dev_wake_gpio;
	struct gpio_desc *host_wake_gpio;
	unsigned int	host_wake_intr;

};

static int bcmbt_regulator_get(struct bcmbt_rfkill_platform_data *pdata)
{
	int i;
	char *reg_name;
	struct bcm_rfkill_instance *instance_ptr;

	for (i = 0; i < pdata->num_rf_regs; i++) {
		reg_name = devm_kzalloc(pdata->dev, MAX_REGULATOR_NAME_LEN,
					 GFP_KERNEL);
		if (!reg_name)
			return -ENOMEM;

		sprintf(reg_name, "bt-rf-reg%d", i);
		instance_ptr = &pdata->instances[i];
		instance_ptr->bt_reg = devm_regulator_get(pdata->dev, reg_name);
		if (IS_ERR(instance_ptr->bt_reg))
			instance_ptr->bt_reg = NULL;
		devm_kfree(pdata->dev, reg_name);
	}

	return 0;
}

static int bcmbt_regulator_enable(struct bcmbt_rfkill_platform_data *pdata)
{
	int i, err;

	for (i = 0; i < pdata->num_rf_regs; i++) {
		if (pdata->instances[i].bt_reg) {
			err = regulator_enable(pdata->instances[i].bt_reg);
			if (err) {
				dev_err(pdata->dev,
					"failed to enable regulator\n");
				return err;
			}
		}
	}

	return 0;
}

static int bcmbt_regulator_disable(struct bcmbt_rfkill_platform_data *pdata)
{
	int i, err;

	for (i = 0; i < pdata->num_rf_regs; i++) {
		if (pdata->instances[i].bt_reg &&
		    regulator_is_enabled(pdata->instances[i].bt_reg)) {
			err = regulator_disable(pdata->instances[i].bt_reg);
			if (err) {
				dev_err(pdata->dev,
					"failed to disable regulator\n");
				return err;
			}
		}
	}
	return 0;
}


static int bcmbt_rfkill_set_power(void *data, bool blocked)
{

	struct bcmbt_rfkill_platform_data *pdata = (struct bcmbt_rfkill_platform_data *)data;

	printk("[ADK] %s entered,  blocked=%d \n", __func__, blocked);

	if (blocked) {
		gpiod_set_value_cansleep(pdata->dev_wake_gpio, 0);
	}else
		gpiod_set_value_cansleep(pdata->dev_wake_gpio, 1);

	rfkill_set_states(pdata->rfkill, blocked, blocked);
	

#if 0
	if (blocked == false) {
		/* Transmitter ON (Unblocked) */
		bcmbt_regulator_enable(data);
		pr_info("bcm_bt_rfkill_setpower: unblocked\n");
	} else {
		/* Transmitter OFF (Blocked) */
		bcmbt_regulator_disable(data);
		pr_info("bcm_bt_rfkill_setpower: blocked\n");
	}
#endif	
	return 0;
}

static void bcmbt_rfkill_query(struct rfkill *rfkill, void *data)
{
	struct bcmbt_rfkill_platform_data *pdata = (struct bcmbt_rfkill_platform_data *)data;
	int result = 0;

	printk("[ADK] %s entered,  dev_wake_gpio= [%s]\n", __func__, pdata->dev_wake_gpio->name);


	rfkill_set_sw_state(pdata->rfkill, !result);
}

static irqreturn_t bcmbt_rfkill_host_wake_isr(int irq, void *data)
{
	struct bcmbt_rfkill_platform_data *pdata = (struct bcmbt_rfkill_platform_data *)data;

	printk("[ADK] %s entered,  dev_wake_gpio= [%s]\n", __func__, pdata->host_wake_gpio->name);

	return IRQ_HANDLED;
}

static const struct rfkill_ops bcmbt_rfkill_ops = {
	.set_block = bcmbt_rfkill_set_power,
//	.query = bcmbt_rfkill_query,
};

static int bcmbt_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct bcmbt_rfkill_platform_data *pdata;
	struct device *dev = &pdev->dev;
	int num_rf_regs;
	int gpio_value;
	struct device_node *np;
	dev_t devt;

printk("[ADK] %s entered\n", __func__);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		return -ENOMEM;

	/* get max number of regulators */
	rc = of_property_read_u32(dev->of_node, "num-rf-regs", &num_rf_regs);
	if (rc) {
		dev_err(dev, "Failed to get device tree resource num-rf-reg\n");
		return rc;
	}
	pdata->instances = devm_kcalloc(dev, num_rf_regs,
					     sizeof(struct bcm_rfkill_instance),
					     GFP_KERNEL);
	pdata->num_rf_regs = num_rf_regs;
	pdata->dev = dev;

#if 0
	rc = bcmbt_regulator_get(pdata);
	if (rc) {
		dev_err(dev, "Failed to get regulator\n");
		return rc;
	}
#endif	
printk("[ADK] %s/%d\n", __func__, __LINE__);

	pdata->rfkill =
/* [ADK]
	    rfkill_alloc("bcmbt", dev, RFKILL_TYPE_BLUETOOTH,
			 &bcmbt_rfkill_ops, pdata);
*/
	    rfkill_alloc(dev_name(dev), dev, RFKILL_TYPE_BLUETOOTH,
			 &bcmbt_rfkill_ops, pdata);
printk("[ADK] %s/%d  create [%s]\n", __func__, __LINE__, dev_name(dev));

	if (unlikely(!pdata->rfkill))
		return -ENOMEM;

	/* Keep BT Blocked by default as per above init */
	rfkill_init_sw_state(pdata->rfkill, true);
printk("[ADK] %s/%d\n", __func__, __LINE__);

	rc = rfkill_register(pdata->rfkill);
printk("[ADK] %s/%d, rc = %d\n", __func__, __LINE__, rc);

	if (unlikely(rc)) {
		rfkill_destroy(pdata->rfkill);
		return rc;
	}

	dev->platform_data = pdata;

#if 0
// [ADK]  01/15/2016  -- removed ..
	/* Create SYS entries for wake GPIOs */
	bcmbt_class = class_create(THIS_MODULE, "bluetooth");
	if (IS_ERR(bcmbt_class)) {
		dev_err(dev, "Can't create BT SYS class entry; errno = %ld\n",
			PTR_ERR(bcmbt_class));
		bcmbt_class = NULL;
		rfkill_destroy(pdata->rfkill);
		return PTR_ERR(bcmbt_class);
	}
printk("[ADK] %s/%d\n", __func__, __LINE__);

	devt = MKDEV(0, 0);
	bcmbt_device = device_create(bcmbt_class, NULL, devt, NULL,
				     "wake_gpios");
	if (IS_ERR(bcmbt_device)) {
		dev_err(dev, "Can't create BT SYS device entry; errno = %ld\n",
			PTR_ERR(bcmbt_device));
		bcmbt_device = NULL;
		rfkill_destroy(pdata->rfkill);
		class_destroy(bcmbt_class);
		return PTR_ERR(bcmbt_device);
	}
printk("[ADK] %s/%d\n", __func__, __LINE__);

	rc = device_create_file(bcmbt_device, &device_attrs[0]);
	if (rc != 0) {
		rfkill_destroy(pdata->rfkill);
		device_destroy(bcmbt_class, devt);
		class_destroy(bcmbt_class);
		return rc;
	}
printk("[ADK] %s/%d\n", __func__, __LINE__);

	rc = device_create_file(bcmbt_device, &device_attrs[1]);
	if (rc != 0) {
		rfkill_destroy(pdata->rfkill);
		device_remove_file(bcmbt_device, &device_attrs[0]);
		device_destroy(bcmbt_class, devt);
		class_destroy(bcmbt_class);
		return rc;
	}
#endif

printk("[ADK] %s/%d\n", __func__, __LINE__);

	pdata->dev_wake_gpio = devm_gpiod_get(&pdev->dev, "bt-dev-wake", GPIOD_OUT_LOW);
	if (IS_ERR(pdata->dev_wake_gpio)) {
		dev_err(dev, "failed to get dev wake GPIO\n");
//		return PTR_ERR(pdata->dev_wake_gpio);
	}

	pdata->host_wake_gpio = devm_gpiod_get(&pdev->dev, "bt-host-wake", GPIOD_IN);
	if (IS_ERR(pdata->host_wake_gpio)) {
		dev_err(dev, "failed to get host wake GPIO\n");
//		return PTR_ERR(pdata->dev_wake_gpio);
	}else{
		pdata->host_wake_intr = gpiod_to_irq(pdata->host_wake_gpio);
printk("[ADK] %s/%d host wake IRQ %d\n", __func__, __LINE__, pdata->host_wake_intr);
		rc = devm_request_threaded_irq(pdata->dev, pdata->host_wake_intr, NULL,
			bcmbt_rfkill_host_wake_isr,
			IRQ_TYPE_EDGE_BOTH | IRQF_ONESHOT,
			"bcmbt_host_wake", pdata);
		if (rc) {
			dev_err(dev, "Host wake irq %d request failed\n", pdata->host_wake_intr);
		}
	}

#if 0
	/* read device tree to get bluetooth device and host wake GPIOs */
	np = pdev->dev.of_node;
	gpio_value = of_get_named_gpio(np, "bt-dev-wake-gpio", 0);
	if (gpio_value < 0)
		dev_warn(dev, "bt-dev-wake-gpio missing\n");
	else
		bcmbt_dev_wake_gpio = gpio_value;

	gpio_value = of_get_named_gpio(np, "bt-host-wake-gpio", 0);
	if (gpio_value < 0)
		dev_warn(dev, "bt-host-wake-gpio missing\n");
	else
		bcmbt_host_wake_gpio = gpio_value;
#endif
printk("[ADK] %s() finished OK\n", __func__);
	return 0;
}

static int bcmbt_rfkill_remove(struct platform_device *pdev)
{
	struct bcmbt_rfkill_platform_data *pdata = pdev->dev.platform_data;

	bcmbt_regulator_disable(pdata);

	rfkill_unregister(pdata->rfkill);
	rfkill_destroy(pdata->rfkill);

	if (pdata->dev_wake_gpio) 
		devm_gpiod_put(pdata->dev, pdata->dev_wake_gpio);
	
	/* Add appropriate destructor here */
//	device_remove_file(bcmbt_device, &device_attrs[0]);
//	device_remove_file(bcmbt_device, &device_attrs[1]);

	if (pdata->dev_wake_gpio) 
		devm_gpiod_put(pdata->dev, pdata->dev_wake_gpio);

	return 0;
}

static const struct of_device_id bcmbt_rfkill_of_match[] = {
	{.compatible = "brcm,bcmbt-rfkill", },
	{ },
};
MODULE_DEVICE_TABLE(of, bcmbt_rfkill_of_match);

static struct platform_driver bcmbt_rfkill_platform_driver = {
	.driver = {
		.name = "bcmbt-rfkill",
		.owner = THIS_MODULE,
		.of_match_table = bcmbt_rfkill_of_match,
	},
	.probe = bcmbt_rfkill_probe,
	.remove = bcmbt_rfkill_remove,
};

module_platform_driver(bcmbt_rfkill_platform_driver);

MODULE_DESCRIPTION("bcmbt-rfkill");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");
