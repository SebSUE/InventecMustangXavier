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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_cygnus_mailbox.h>
#include <linux/delay.h>

#define IPROC_CRMU_MAILBOX0_OFFSET       0x0
#define IPROC_CRMU_MAILBOX1_OFFSET       0x4

#define CRMU_IPROC_MAILBOX0_OFFSET       0x8
#define CRMU_IPROC_MAILBOX1_OFFSET       0xc

#define IPROC_INTR_STATUS                0x34
#define IPROC_MAILBOX_INTR_SHIFT         0
#define IPROC_MAILBOX_INTR_MASK          0x1

#define IPROC_INTR_CLEAR                 0x3c
#define IPROC_MAILBOX_INTR_CLR_SHIFT     0

/* Domains that interrupts get forwarded to. */
enum mbox_domain {
	AON_GPIO_DOMAIN = 0,
};

/* Interrupt types from M0 processor. */
enum mcu_intr_status {
	AON_GPIO_INTR = 2,
};

struct cygnus_mbox {
	struct device         *dev;
	void __iomem          *base;
	spinlock_t            lock;
	struct irq_domain     *irq_domain;
	struct mbox_controller controller;
	u32                   num_chans;
	int                   mbox_irq;
};

static struct lock_class_key mbox_lock_class;

static void cygnus_mbox_irq_handler(struct irq_desc *desc)
{
	struct cygnus_mbox *mbox = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long status;
	u32 cmd, param;
	int virq;

	chained_irq_enter(chip, desc);

	/* Determine type of interrupt. */
	status = readl(mbox->base + IPROC_INTR_STATUS);
	status = (status >> IPROC_MAILBOX_INTR_SHIFT) &
		IPROC_MAILBOX_INTR_MASK;

	/* Process mailbox interrupts. */
	if (status) {
		writel(1 << IPROC_MAILBOX_INTR_CLR_SHIFT,
			mbox->base + IPROC_INTR_CLEAR);

		cmd = readl(mbox->base + CRMU_IPROC_MAILBOX0_OFFSET);
		param = readl(mbox->base + CRMU_IPROC_MAILBOX1_OFFSET);

		dev_dbg(mbox->dev,
			"Received message from M0: cmd 0x%x param 0x%x\n",
			cmd, param);

		/* Process AON GPIO interrupt - forward to GPIO handler. */
		if (cmd == M0_IPC_HOST_CMD_AON_GPIO_EVENT) {
			virq = irq_find_mapping(mbox->irq_domain,
				AON_GPIO_DOMAIN);
			generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

static int cygnus_mbox_send_data_m0_imp(struct cygnus_mbox *mbox,
	struct cygnus_mbox_msg *msg, int max_retries, int poll_period_us)
{
	unsigned long flags;
	u32 val;
	int err = 0;
	int retries;

	spin_lock_irqsave(&mbox->lock, flags);

	dev_dbg(mbox->dev, "Send msg to M0: cmd=0x%x, param=0x%x, wait_ack=%d\n",
		msg->cmd, msg->param, msg->wait_ack);

	writel(msg->cmd, mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
	writel(msg->param, mbox->base + IPROC_CRMU_MAILBOX1_OFFSET);

	if (msg->wait_ack) {
		err = msg->reply_code = -ETIMEDOUT;
		for (retries = 0; retries < max_retries; retries++) {
			val = readl(mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
			if (val & M0_IPC_CMD_DONE_MASK) {
				/*
				 * M0 replied - save reply code and
				 * clear error.
				 */
				msg->reply_code = (val &
					M0_IPC_CMD_REPLY_MASK) >>
					M0_IPC_CMD_REPLY_SHIFT;
				err = 0;
				break;
			}
			udelay(poll_period_us);
		}
	}

	spin_unlock_irqrestore(&mbox->lock, flags);

	return err;
}

static void cygnus_mbox_aon_gpio_forwarding_enable(struct cygnus_mbox *mbox,
	bool en)
{
	struct cygnus_mbox_msg msg;
	msg.cmd = M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE;
	msg.param = en ? 1 : 0;
	msg.wait_ack = true;

	cygnus_mbox_send_data_m0_imp(mbox, &msg, 5, 200);
}

static void cygnus_mbox_irq_unmask(struct irq_data *d)
{
	struct cygnus_mbox *cygnus_mbox = irq_data_get_irq_chip_data(d);

	cygnus_mbox_aon_gpio_forwarding_enable(cygnus_mbox, true);
}

static void cygnus_mbox_irq_mask(struct irq_data *d)
{
	/* Do nothing - Mask callback is not required, since upon GPIO event,
	 * M0 disables GPIO forwarding to A9. Hence, GPIO forwarding is already
	 * disabled  when in mbox irq handler, and no other mbox events from M0
	 * to A9 are expected until GPIO forwarding is enabled following
	 * cygnus_mbox_irq_unmask()
	 */
}

static struct irq_chip cygnus_mbox_irq_chip = {
	.name = "bcm-cygnus-mbox",
	.irq_mask = cygnus_mbox_irq_mask,
	.irq_unmask = cygnus_mbox_irq_unmask,
};

static int cygnus_mbox_irq_map(struct irq_domain *d, unsigned int irq,
	irq_hw_number_t hwirq)
{
	int ret;

	ret = irq_set_chip_data(irq, d->host_data);
	if (ret < 0)
		return ret;
	irq_set_lockdep_class(irq, &mbox_lock_class);
	irq_set_chip_and_handler(irq, &cygnus_mbox_irq_chip,
			handle_simple_irq);
//	set_irq_flags(irq, IRQF_VALID);
        irq_clear_status_flags(irq, IRQ_NOREQUEST);
	return 0;
}

static void cygnus_mbox_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static struct irq_domain_ops cygnus_mbox_irq_ops = {
	.map = cygnus_mbox_irq_map,
	.unmap = cygnus_mbox_irq_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static const struct of_device_id cygnus_mbox_of_match[] = {
	{ .compatible = "brcm,cygnus-mailbox" },
	{ }
};
MODULE_DEVICE_TABLE(of, cygnus_mbox_of_match);

/*
 * Sends a message to M0. The mailbox framework prevents multiple accesses to
 * the same channel but there is only one h/w "channel". This driver allows
 * multiple clients to create channels to the controller but must serialize
 * access to the mailbox registers used to communicate with the M0.
 */
int cygnus_mbox_send_data_m0(struct mbox_chan *chan, void *data)
{
	struct cygnus_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct cygnus_mbox_msg *msg = (struct cygnus_mbox_msg *)data;
	int err = 0;
	const int poll_period_us = 5;
	int max_retries;

	if (!msg)
		return -EINVAL;

	/* At least 1 attempt for misconfigured clients. */
	if (chan->cl->tx_tout == 0)
		max_retries = 1;
	else
		max_retries = (chan->cl->tx_tout * 1000) / poll_period_us;

	err = cygnus_mbox_send_data_m0_imp(mbox, msg, max_retries,
		poll_period_us);

	return err;
}

int cygnus_mbox_startup(struct mbox_chan *chan)
{
	/* Do nothing. */
	return 0;
}

void cygnus_mbox_shutdown(struct mbox_chan *chan)
{
	/* Do nothing. */
}

static struct mbox_chan_ops cygnus_mbox_ops = {
	.send_data    = cygnus_mbox_send_data_m0,
	.startup      = cygnus_mbox_startup,
	.shutdown     = cygnus_mbox_shutdown,
};

static int __init cygnus_mbox_probe(struct platform_device *pdev)
{
	int virq;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct cygnus_mbox *cygnus_mbox;
	int err;
	struct device_node *node;
	struct property *prop;

	dev_info(&pdev->dev, "Initializing cygnus mailbox controller\n");

	cygnus_mbox = devm_kzalloc(dev, sizeof(*cygnus_mbox), GFP_KERNEL);
	if (!cygnus_mbox)
		return -ENOMEM;

	cygnus_mbox->dev = dev;
	spin_lock_init(&cygnus_mbox->lock);

	platform_set_drvdata(pdev, cygnus_mbox);

	/* Count number of "mboxes" properties to determine # channels. */
	for_each_of_allnodes(node) {
	    for (prop = node->properties; prop; prop = prop->next)
			if (of_prop_cmp(prop->name, "mboxes") == 0)
				cygnus_mbox->num_chans++;
	}

	/* Allocate mailbox channels. */
	if (cygnus_mbox->num_chans > 0) {
		struct mbox_chan *chans = devm_kzalloc(&pdev->dev,
			sizeof(*chans) * cygnus_mbox->num_chans, GFP_KERNEL);

		if (!chans)
			return -ENOMEM;

		/* Initialize mailbox controller. */
		cygnus_mbox->controller.dev = cygnus_mbox->dev;
		cygnus_mbox->controller.num_chans = cygnus_mbox->num_chans;
		cygnus_mbox->controller.chans = chans;
		cygnus_mbox->controller.ops = &cygnus_mbox_ops;
		cygnus_mbox->controller.txdone_irq = false;
		cygnus_mbox->controller.txdone_poll = false;
		err = mbox_controller_register(&cygnus_mbox->controller);
		if (err) {
			dev_err(&pdev->dev, "Register mailbox failed\n");
			return err;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cygnus_mbox->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygnus_mbox->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(cygnus_mbox->base);
	}

	cygnus_mbox->mbox_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!cygnus_mbox->mbox_irq) {
		dev_err(&pdev->dev, "irq_of_parse_and_map failed\n");
		return -ENODEV;
	}

	cygnus_mbox->irq_domain = irq_domain_add_linear(dev->of_node, 1,
		&cygnus_mbox_irq_ops, cygnus_mbox);
	if (!cygnus_mbox->irq_domain) {
		dev_err(&pdev->dev, "unable to allocate IRQ domain\n");
		return -ENXIO;
	}

	/* Map irq for AON GPIO interrupt handling into this domain. */
	virq = irq_create_mapping(cygnus_mbox->irq_domain, AON_GPIO_DOMAIN);
	if (!virq) {
		dev_err(&pdev->dev, "failed mapping irq into domain\n");
		return -ENXIO;
	}
	dev_dbg(&pdev->dev, "irq for aon gpio domain: %d\n", virq);

	irq_set_handler_data(cygnus_mbox->mbox_irq, cygnus_mbox);
	irq_set_chained_handler(cygnus_mbox->mbox_irq, cygnus_mbox_irq_handler);

	return 0;
}

static int cygnus_mbox_remove(struct platform_device *pdev)
{
	struct cygnus_mbox *mbox = platform_get_drvdata(pdev);

	if (mbox->num_chans > 0)
		mbox_controller_unregister(&mbox->controller);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cygnus_mbox_suspend(struct device *dev)
{
	struct cygnus_mbox *mbox = dev_get_drvdata(dev);

	dev_info(dev,
		"Suspending mailbox controller: disabling GPIO forwarding\n");
	cygnus_mbox_aon_gpio_forwarding_enable(mbox, false);
	synchronize_irq(mbox->mbox_irq);

	return 0;
}

static int cygnus_mbox_resume(struct device *dev)
{
	struct cygnus_mbox *mbox = dev_get_drvdata(dev);

	dev_info(dev,
		"Resuming mailbox controller: enabling AON GPIO forwarding\n");
	cygnus_mbox_aon_gpio_forwarding_enable(mbox, true);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(cygnus_mbox_pm_ops, cygnus_mbox_suspend,
	cygnus_mbox_resume);

struct platform_driver cygnus_mbox_driver = {
	.driver = {
		.name = "brcm,cygnus-mailbox",
		.of_match_table = cygnus_mbox_of_match,
		.pm = &cygnus_mbox_pm_ops,
	},
	.remove = cygnus_mbox_remove,
};

static int __init iproc_mbox_init(void)
{
	return platform_driver_probe(&cygnus_mbox_driver, cygnus_mbox_probe);
}
arch_initcall(iproc_mbox_init);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus Mailbox Driver");
MODULE_LICENSE("GPL v2");
