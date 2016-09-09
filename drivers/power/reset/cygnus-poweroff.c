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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_cygnus_mailbox.h>

#include <asm/system_misc.h>

struct cygnus_reset {
	struct device       *dev;
	struct mbox_client  client;
	struct mbox_chan    *mbox_chan;
};
static struct cygnus_reset cygnus_reset = { NULL };


static inline void cygnus_reset_print_mbox_err(int mbox_err, int reply_code)
{
	if (mbox_err < 0)
		dev_err(cygnus_reset.dev,
			"mbox_send_message failed: %d\n", mbox_err);
	else if (reply_code)
		dev_err(cygnus_reset.dev,
			"M0 command failed: 0x%x\n", reply_code);
}

/*
 * Perform cold (L0) reset.
 */
static inline void cygnus_cold_reset(void)
{
	struct cygnus_mbox_msg msg;
	int err;

	msg.cmd = M0_IPC_M0_CMD_IPROC_RESET;
	msg.param = M0_IPC_M0_CMD_IPROC_RESET_PARAM;
	msg.wait_ack = true;
	err = mbox_send_message(cygnus_reset.mbox_chan, &msg);
	cygnus_reset_print_mbox_err(err, msg.reply_code);
	mbox_client_txdone(cygnus_reset.mbox_chan, 0);
}

/*
 * Perform warm reset (keep ethernet switch active throught the reset)
 */
static inline void cygnus_warm_reset(void)
{
	struct cygnus_mbox_msg msg;
	int err;

	msg.cmd = M0_IPC_M0_CMD_WARM_RESET;
	msg.param = 0;
	msg.wait_ack = false;
	err = mbox_send_message(cygnus_reset.mbox_chan, &msg);
	cygnus_reset_print_mbox_err(err, msg.reply_code);
	mbox_client_txdone(cygnus_reset.mbox_chan, 0);
}

/*
 * Notifies M0 to power off.
 */
static inline void cygnus_power_off(void)
{
	struct cygnus_mbox_msg msg;
	int err;

	msg.cmd = M0_IPC_M0_CMD_ENTER_OFF;
	msg.param = 0;
	msg.wait_ack = false;
	err = mbox_send_message(cygnus_reset.mbox_chan, &msg);
	cygnus_reset_print_mbox_err(err, msg.reply_code);
	mbox_client_txdone(cygnus_reset.mbox_chan, 0);
}

/*
 * Handles rebooting CPU.
 */
static void cygnus_reboot(enum reboot_mode mode, const char *cmd)
{
	if (mode == REBOOT_COLD)
		cygnus_cold_reset();
	else if (mode == REBOOT_WARM)
		cygnus_warm_reset();
}

static int cygnus_reset_probe(struct platform_device *pdev)
{
	/* Save device for reboot/power_off. */
	if (!cygnus_reset.dev) {
		cygnus_reset.dev = &pdev->dev;
	} else {
		dev_warn(&pdev->dev, "cygnus restart dev already set. not setting\n");
		return 0;
	}

	/* Request mailbox channel. */
	cygnus_reset.client.dev          = &pdev->dev;
	cygnus_reset.client.rx_callback  = NULL;
	cygnus_reset.client.tx_done      = NULL;
	cygnus_reset.client.tx_block     = false;
	cygnus_reset.client.tx_tout      = 1;
	cygnus_reset.client.knows_txdone = true;
	cygnus_reset.mbox_chan = mbox_request_channel(&cygnus_reset.client, 0);
	if (IS_ERR(cygnus_reset.mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		return PTR_ERR(cygnus_reset.mbox_chan);
	}

	/* Set the machine restart handler. */
	arm_pm_restart = cygnus_reboot;

	/* Set the power off handler. */
	pm_power_off = cygnus_power_off;

	return 0;
}

static int cygnus_reset_remove(struct platform_device *pdev)
{
	mbox_free_channel(cygnus_reset.mbox_chan);

	return 0;
}

static const struct of_device_id cygnus_reset_of_match[] = {
	{ .compatible = "brcm,cygnus-reset", },
	{}
};
MODULE_DEVICE_TABLE(of, cygnus_reset_of_match);

struct platform_driver cygnus_reset_driver = {
	.driver = {
		.name = "brcm,cygnus-reset",
		.of_match_table = cygnus_reset_of_match,
	},
	.probe = cygnus_reset_probe,
	.remove = cygnus_reset_remove,
};
module_platform_driver(cygnus_reset_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus Reset Driver");
MODULE_LICENSE("GPL v2");
