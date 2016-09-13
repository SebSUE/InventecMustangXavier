/*
 * Copyright 2014 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/sysfs.h>
#include <linux/cpu_pm.h>
#include <linux/delay.h>
#include <linux/kmod.h>

#include "iproc_pm.h"



/* ////////////////////////////////////////////////////////////////////////////////////////// */
static int iproc_pm_valid(suspend_state_t pm_state)
{
	return (pm_state == PM_SUSPEND_STANDBY) || (pm_state == PM_SUSPEND_MEM);
}

static int iproc_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:	/*  this is mapped to SoC SLEEP */
		dev_info(iproc_pm->dev, "Entering PM_SUSPEND_STANDBY\n");
		ret = iproc_soc_enter_sleep(IPROC_PM_STATE_SLEEP);
		dev_info(iproc_pm->dev, "Leaving PM_SUSPEND_STANDBY\n");
		if (ret == 0)
			iproc_soc_enter_run();
		break;
	case PM_SUSPEND_MEM:	/*  this is mapped to SoC DEEPSLEEP */
		dev_info(iproc_pm->dev, "Entering PM_SUSPEND_MEM\n");
		ret = iproc_soc_enter_sleep(IPROC_PM_STATE_DEEPSLEEP);
		dev_info(iproc_pm->dev, "Leaving PM_SUSPEND_MEM\n");
		if(ret == 0)
			iproc_soc_enter_run();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void iproc_pm_finish(void)
{
	return;
}

/* ////////////////////////////////////////////////////////////////////////////// */
static struct platform_suspend_ops iproc_pm_ops = {
	.valid = iproc_pm_valid,
	.enter = iproc_pm_enter,
	.finish = iproc_pm_finish,
};

int iproc_suspend_init(void)
{
	dev_info(iproc_pm->dev, "Enable suspend support for this machine!\n");

	suspend_set_ops(&iproc_pm_ops);

	return 0;
}

void iproc_suspend_exit(void)
{
	dev_info(iproc_pm->dev, "Disable suspend support for this machine!\n");

	suspend_set_ops(NULL);

	return;
}
