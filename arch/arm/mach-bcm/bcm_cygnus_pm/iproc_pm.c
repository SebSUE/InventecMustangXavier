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
#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system_misc.h>
#include <asm/mach/time.h>

#include "iproc_pm.h"
#include "iproc_pm_device.h"


#define ADD_IOMEM_RES(Name, Startreg, Endreg) \
	{\
		.name  = #Name,\
		.start = Startreg,\
		.end   = (Endreg+3),\
		.flags = IORESOURCE_MEM,\
	}

#define ADD_IOMEM_REG(Name, Startreg) \
	{\
		.name  = #Name,\
		.start = Startreg,\
		.end   = (Startreg+3),\
		.flags = IORESOURCE_MEM,\
	}

struct bcm_iproc_pm *iproc_pm;

struct resource m0_idram_res =
ADD_IOMEM_RES(M0_IDRAM, CRUM_M0_IDRAM_START, (CRUM_M0_IDRAM_END - 3));

void *__iomem m0_idram_vbase = NULL;	/* m0_idram_res mapped vaddr */

static struct resource iproc_pm_resources[] = {
	ADD_IOMEM_RES(m0_isr_ctl, 0x03018000, 0x3019ffc),
	ADD_IOMEM_RES(crmu_dru, CRMU_XTAL_CHANNEL_CONTROL, BSTI_COMMAND),
	ADD_IOMEM_RES(chip_dru, CRMU_GENPLL_CONTROL0, CDRU_USBPHY_P2_CTRL_0),
	ADD_IOMEM_RES(dmu, CRMU_STRAP_DATA, CRMU_SOTP_NEUTRALIZE_ENABLE),
	ADD_IOMEM_RES(cru, CRU_control, CRU_timer),
	ADD_IOMEM_RES(ihost_clk, IHOST_PROC_CLK_WR_ACCESS,
		      IHOST_PROC_RST_A9_CORE_SOFT_RSTN),
	ADD_IOMEM_RES(scu, IHOST_SCU_CONTROL, IHOST_SCU_SECURE_ACCESS),
	ADD_IOMEM_RES(gic_cpu, IHOST_GICCPU_CONTROL, IHOST_GICCPU_CPU_IDENT),
	ADD_IOMEM_RES(scu_gtimer, IHOST_GTIM_GLOB_LOW, IHOST_GTIM_GLOB_INCR),
	ADD_IOMEM_RES(gic_dist, IHOST_GICDIST_enable_s,
		      IHOST_GICDIST_priority_level0),
	ADD_IOMEM_RES(ihost_l2c, IHOST_L2C_CACHE_ID, IHOST_L2C_PWR_CTRL),
};

#define IPROC_RESOURCES_NUM        ARRAY_SIZE(iproc_pm_resources)

static iproc_resources_t IPROC_PM_RES[IPROC_RESOURCES_NUM];

static enum iproc_power_status current_pm_state = IPROC_PM_STATE_END;

static char *iproc_pm_states_str[] = {
	[IPROC_PM_STATE_RUN] = "RUN",
	[IPROC_PM_STATE_SLEEP] = "SLEEP",
	[IPROC_PM_STATE_DEEPSLEEP] = "DEEPSLEEP",
	[IPROC_PM_STATE_END] = "UNKNOW",
};

struct mbox_chan *cygnus_mbox_chan;

static ssize_t aon_gpio_mask_skip_sleep_if_high_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%lx",
		iproc_pm->aon_gpio_mask_skip_sleep_if_high);
}
static ssize_t aon_gpio_mask_skip_sleep_if_low_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "0x%lx", iproc_pm->aon_gpio_mask_skip_sleep_if_low);
}
static ssize_t aon_gpio_mask_skip_sleep_if_x_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count,
	unsigned long *var)
{
	int ret = 0;
	struct m0_ipc_m0_cmd_aon_gpio_wakeup_cfg *cmd;

	ret = kstrtoul(buf, 0, var);
	if (ret) {
		dev_err(dev, "Invalid value of AON GPIO mask\n");
		return ret;
	}

	/* Send the configuration IPC message to M0 */
	cmd = kmalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	cmd->mask_skip_sleep_if_low = iproc_pm->aon_gpio_mask_skip_sleep_if_low;
	cmd->mask_skip_sleep_if_high =
		iproc_pm->aon_gpio_mask_skip_sleep_if_high;
	dev_dbg(dev,
		"Configuring AON gpio wakeup: aon_gpio_mask_skip_sleep_if_high 0x%lx, aon_gpio_mask_skip_sleep_if_low 0x%lx\n",
		iproc_pm->aon_gpio_mask_skip_sleep_if_high,
		iproc_pm->aon_gpio_mask_skip_sleep_if_low);
	ret = iproc_mbox_send_msg_with_struct(M0_IPC_M0_CMD_AON_GPIO_WAKEUP_CFG,
		cmd, sizeof(*cmd), true);

	kfree(cmd);

	if (ret) {
		dev_err(dev, "Error configuring aon gpio wakeup\n");
		return -EINVAL;
	}
	return count;
}
static ssize_t aon_gpio_mask_skip_sleep_if_high_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	return aon_gpio_mask_skip_sleep_if_x_store(dev, attr, buf, count,
		&iproc_pm->aon_gpio_mask_skip_sleep_if_high);
}
static ssize_t aon_gpio_mask_skip_sleep_if_low_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	return aon_gpio_mask_skip_sleep_if_x_store(dev, attr, buf, count,
		&iproc_pm->aon_gpio_mask_skip_sleep_if_low);
}

static DEVICE_ATTR(aon_gpio_mask_skip_sleep_if_high, S_IWUSR | S_IRUGO,
	aon_gpio_mask_skip_sleep_if_high_show,
	aon_gpio_mask_skip_sleep_if_high_store);
static DEVICE_ATTR(aon_gpio_mask_skip_sleep_if_low, S_IWUSR | S_IRUGO,
	aon_gpio_mask_skip_sleep_if_low_show,
	aon_gpio_mask_skip_sleep_if_low_store);

static struct attribute *dev_attrs[] = {
	&dev_attr_aon_gpio_mask_skip_sleep_if_high.attr,
	&dev_attr_aon_gpio_mask_skip_sleep_if_low.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.attrs = dev_attrs,
};

static uint32_t iproc_pm_reg32_get_vaddr(const uint32_t reg)
{
	uint32_t i;
	uint32_t address = 0;
	iproc_resources_t *res = NULL;

	for (i = 0; i < IPROC_RESOURCES_NUM; i++) {
		res = &IPROC_PM_RES[i];

		if (!res->res || !res->vaddr)
			continue;

		if (res->res->flags != IORESOURCE_MEM)
			continue;

		if (reg >= res->res->start && reg <= res->res->end) {
			address =
			    ((uint32_t) res->vaddr + (reg - res->res->start));
			break;
		}
	}

	return address;
}

uint32_t iproc_reg32_read(const uint32_t addr)
{
	uint32_t val;
	uint32_t address = iproc_pm_reg32_get_vaddr(addr);

	if (!address) {
		dev_err(iproc_pm->dev,
			"Invalid reg addr: 0x%08x, caller: [%pS]\n", addr,
			  __builtin_return_address(0));
		return 0;
	}
	val = ioread32((const volatile void*)address);

	return val;
}

void iproc_reg32_write(const uint32_t addr, const uint32_t value)
{
	uint32_t address = iproc_pm_reg32_get_vaddr(addr);

	if (!address) {
		dev_err(iproc_pm->dev,
			"Invalid reg addr: 0x%08x, caller: [%pS]\n", addr,
			__builtin_return_address(0));
		return;
	}
	iowrite32(value, (volatile void*)address);
}

/* Set bitno, other bits unchange */
void iproc_reg32_setbit(const uint32_t addr, const uint32_t bitno)
{
	iproc_reg32_write(addr, iproc_reg32_read(addr) | (1U << bitno));
}

/* Clear bitno, other bits unchange */
void iproc_reg32_clrbit(const uint32_t addr, const uint32_t bitno)
{
	iproc_reg32_write(addr, iproc_reg32_read(addr) & (~(1U << bitno)));
}

/*
 * Initialize one IORESOURCE_MEM resource, return the iomapped vaddr.
 */
static uint32_t *iproc_resource_init_one(struct platform_device *pdev,
					 const char *name)
{
	void *__iomem reg_base;
	struct resource *res = NULL;

	if (!pdev || !name) {
		dev_err(&pdev->dev, "Invalid arg!\n");
		return NULL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (res == NULL) {
		dev_err(&pdev->dev,
			"Failed to get resource by name `%s'!\n", name);
		return NULL;
	}

	if (res->start > res->end) {
		dev_err(&pdev->dev,
			"Invalid resource `%s'[0x%08x~0x%08x]!\n", name,
			res->start, res->end);
		return NULL;
	}

	reg_base = ioremap_nocache(res->start, resource_size(res));
	if (reg_base == NULL) {
		dev_err(&pdev->dev,
			"Failed to do ioremap for resource `%s'!\n", name);
		return NULL;
	}

	return (uint32_t *) reg_base;
}

static int iproc_pm_resource_init(struct platform_device *pdev)
{
	int i = 0, j = 0, ret = 0;
	uint32_t *__iomem mapped_addr;
	struct resource *r = NULL;
	iproc_resources_t *res = NULL;

	/* IDRAM resource init */
	r = &m0_idram_res;

	m0_idram_vbase = ioremap(r->start, resource_size(r));
	BUG_ON(IS_ERR_OR_NULL(m0_idram_vbase));
	dev_info(&pdev->dev,
		"Resource %02d[%10s: 0x%08x~0x%08x/0x%08x] mapped to 0x%p\n",
		i, r->name, r->start, r->end, resource_size(r),
		m0_idram_vbase);

	/* Common resources init */
	for (i = 0; i < IPROC_RESOURCES_NUM; i++) {
		res = &IPROC_PM_RES[i];
		r = &iproc_pm_resources[i];

		res->res = r;
		res->vaddr = 0;

		if (r->flags != IORESOURCE_MEM)
			continue;

		mapped_addr = iproc_resource_init_one(pdev, r->name);

		if (!mapped_addr) {
			ret = -ENXIO;
			break;
		}

		res->vaddr = mapped_addr;

		dev_info(iproc_pm->dev,
			"Resource %02d[%10s: 0x%08x~0x%08x/0x%08x] mapped to 0x%p\n",
			i, res->res->name, res->res->start, res->res->end,
			resource_size(res->res), res->vaddr);

		j++;
	}

	dev_info(iproc_pm->dev, "%d MEM resources initialized\n", ++j);

	return ret;
}

static int iproc_pm_resource_exit(void)
{
	int i = 0;
	iproc_resources_t *res = NULL;

	dev_info(iproc_pm->dev, "Free resources!\n");

	/* Free mailbox channel. */
	mbox_free_channel(cygnus_mbox_chan);

	/* Common resources release */
	for (i = 0; i < IPROC_RESOURCES_NUM; i++) {
		res = &IPROC_PM_RES[i];

		if (res->res && res->res->flags == IORESOURCE_MEM
		    && res->vaddr) {
			iounmap(res->vaddr);

			dev_info(iproc_pm->dev,
				"  Resource %d freed, vaddr 0x%p.\n", i,
				res->vaddr);
		}
	}

	/* IDRAM resource release */
	iounmap(m0_idram_vbase);

	return 0;
}

/* ////////////////////////////////////////////////////////////////////////// */
int iproc_pm_save_device_regs(struct iproc_pm_device_regs *pm_dev)
{
	int i = 0;
	struct iproc_pm_reg *save_reg = NULL;

	if (!pm_dev->read || !pm_dev->write) {
		dev_err(iproc_pm->dev, "No read/write method!\n");
		return -EINVAL;
	}

	dev_info(iproc_pm->dev, "%s save regs:\n", pm_dev->devname);

	for (i = 0; i < pm_dev->regs_num; i++) {
		save_reg = *(pm_dev->regs) + i;
		save_reg->regval = pm_dev->read((uint32_t) (save_reg->regaddr));
		dev_info(iproc_pm->dev, "    [0x%p]=0x%08lx\n",
			save_reg->regaddr, save_reg->regval);
	}

	return 0;
}

EXPORT_SYMBOL(iproc_pm_save_device_regs);

int iproc_pm_restore_device_regs(struct iproc_pm_device_regs *pm_dev)
{
	int i = 0;
	struct iproc_pm_reg *save_reg = NULL;

	if (!pm_dev->read || !pm_dev->write) {
		dev_err(iproc_pm->dev, "No read/write method!\n");
		return -EINVAL;
	}

	dev_info(iproc_pm->dev, "%s restore regs:\n", pm_dev->devname);

	for (i = 0; i < pm_dev->regs_num; i++) {
		save_reg = *(pm_dev->regs) + i;

		dev_info(iproc_pm->dev,
			"    [0x%p]=0x%08lx\n", save_reg->regaddr,
			save_reg->regval);
		pm_dev->write((uint32_t) (save_reg->regaddr),
			      (uint32_t) (save_reg->regval));
	}

	return 0;
}

EXPORT_SYMBOL(iproc_pm_restore_device_regs);

enum iproc_power_status iproc_get_current_pm_state(void)
{
	return current_pm_state;
}

void iproc_set_current_pm_state(enum iproc_power_status new_state)
{
	dev_info(iproc_pm->dev, "Set curret pm state to %s\n",
		  iproc_get_pm_state_str_by_id(new_state));

	current_pm_state = new_state;
}

char *iproc_get_pm_state_str_by_id(enum iproc_power_status stateid)
{
	if (stateid <= IPROC_PM_STATE_END)
		return iproc_pm_states_str[stateid];
	else
		return NULL;
}

/* Find out if regulator is disabled in the given PM state.
 * AON regulator node in the device tree is matched against state_name parameter
 *
 * Return true if regulator should be externally disabled in the given PM state,
 *  otherwise return false
 */
static bool iproc_pm_parse_aon_regulator_is_disabled_in_state(
	struct device_node *regulator_node,
	const char *state_name)
{
	struct device_node *state_node = of_find_node_by_name(regulator_node,
		state_name);
	if (state_node &&
		of_property_read_bool(state_node,
			"regulator-external-off-in-suspend"))
		return true;

	return false;
}

static int iproc_pm_parse_aon_regulator_cfg(struct platform_device *pdev,
	struct device_node *regulator_node)
{
	int ret = 0;
	bool enable_active_high = false;
	u32 startup_delay = 0;
	u32 disabled_in_pm_states_mask = 0;
	u32 gpio;
	struct m0_ipc_m0_cmd_power_rail_regulator_cfg *cmd;
	struct of_phandle_args args;

	/* Parse aon regulator device tree node */

	/* Get AON GPIO - mandatory property */
	ret = of_parse_phandle_with_args(regulator_node, "gpio", "#gpio-cells",
		0, &args);
	if (ret) {
		dev_err(&pdev->dev,
			"Error parsing PM aon regulators config: gpio property of PM regulator is required (node %s)\n",
			of_node_full_name(regulator_node));
		return ret;
	}
	gpio = args.args[0];
	of_node_put(args.np);

	/* Get is-active-high property*/
	if (of_property_read_bool(regulator_node, "enable-active-high"))
		enable_active_high = true;

	/* Get ramp delay property */
	of_property_read_u32(regulator_node, "startup-delay-us",
		&startup_delay);

	/* Find out if regulator should be enabled/disabled in different
	 * PM states
	 */
	if (iproc_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-off"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_OFF);

	if (iproc_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-standby"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_SLEEP);

	if (iproc_pm_parse_aon_regulator_is_disabled_in_state(regulator_node,
		"regulator-state-mem"))
		disabled_in_pm_states_mask |= (1 << M0_IPC_PM_STATE_DEEPSLEEP);

	/* Send the configuration IPC message to M0 */
	cmd = kmalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	cmd->gpio = gpio;
	cmd->enable_active_high = enable_active_high;
	cmd->startup_delay_usec = startup_delay;
	cmd->disabled_in_pm_states_mask = disabled_in_pm_states_mask;
	dev_info(&pdev->dev, "Configuring AON regulator: aon gpio %u, active %s, startup delay usec %u, disabled PM states mask 0x%x\n",
		gpio,
		(enable_active_high ? "high" : " low"),
		startup_delay,
		disabled_in_pm_states_mask);
	ret = iproc_mbox_send_msg_with_struct(
		M0_IPC_M0_CMD_POWER_RAIL_REGULATOR_CFG, cmd, sizeof(*cmd),
		true);
	if (ret)
		dev_err(&pdev->dev,
			"Error configuring PM aon regulator (node %s)\n",
			of_node_full_name(regulator_node));

	kfree(cmd);

	return ret;
}

static int iproc_pm_parse_aon_regulators_cfg(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *regulators_node;
	struct device_node *regulator_node;

	regulators_node = of_find_node_by_name(np, "aon_regulators");
	if (!regulators_node)
		return 0;

	for_each_available_child_of_node(regulators_node, regulator_node) {
		ret = iproc_pm_parse_aon_regulator_cfg(pdev, regulator_node);
		if (ret)
			return ret;
	}

	return 0;
}

static int iproc_pm_probe(struct platform_device *pdev)
{
	int err = -1;

	dev_info(&pdev->dev, "Loading iProc PM driver\n");

	iproc_pm = devm_kzalloc(&pdev->dev, sizeof(*iproc_pm),
		GFP_KERNEL);
	if (!iproc_pm) {
		dev_err(&pdev->dev, "Failed to alloc iproc_pm struct.\n");
		err = -ENOMEM;
		goto err;
	}

	iproc_pm->dev = &pdev->dev;

	/* Register sysfs hooks */
	err = sysfs_create_group(&pdev->dev.kobj, &dev_attr_group);
	if (err) {
		dev_err(&pdev->dev, "Failed to create sysfs attributes\n");
		err = -ENOMEM;
		goto err;
	}

	/* Get mailbox channel. */
	iproc_pm->mbox_client.dev          = &pdev->dev;
	iproc_pm->mbox_client.rx_callback  = NULL;
	iproc_pm->mbox_client.tx_done      = NULL;
	iproc_pm->mbox_client.tx_block     = false;
	iproc_pm->mbox_client.tx_tout      = 2;
	iproc_pm->mbox_client.knows_txdone = true;
	cygnus_mbox_chan = mbox_request_channel(&iproc_pm->mbox_client, 0);
	if (IS_ERR(cygnus_mbox_chan)) {
		dev_err(&pdev->dev, "unable to get mbox channel\n");
		err = PTR_ERR(cygnus_mbox_chan);
		goto err;
	}

	pdev->resource = iproc_pm_resources,
	pdev->num_resources = IPROC_RESOURCES_NUM;

	err = iproc_pm_resource_init(pdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to init resources!\n");
		goto err_free_mbox;
	}

	err = iproc_pm_m0_init();
	if (err) {
		dev_err(&pdev->dev, "Fail to initailize M0!");
		goto err_free_res;
	}

	err = iproc_pm_parse_aon_regulators_cfg(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"Failed to configure aon regulators in device tree\n");
		goto err_free_res;
	}

	iproc_dump_resume_entry();

	if (iproc_suspend_init() < 0) {
		dev_err(&pdev->dev, "Fail to init Linux Suspend!");
		goto err_free_res;
	}
	dev_info(&pdev->dev, "Cygnus Power Management supported\n");

	return err;

 err_free_res:
	iproc_pm_resource_exit();
 err_free_mbox:
	mbox_free_channel(cygnus_mbox_chan);
 err:
	return err;
}

static int iproc_pm_remove(struct platform_device *pdev)
{
	dev_info(iproc_pm->dev, "Unloading PM driver.\n");

	iproc_suspend_exit();
	iproc_pm_resource_exit();
	arm_pm_idle = NULL;

	return 0;
}

static const struct of_device_id iproc_pm_of_match[] = {
	{.compatible = "brcm,cygnus-pm", },
	{},
};
MODULE_DEVICE_TABLE(of, iproc_pm_of_match);

static struct platform_driver iproc_pm_platform_driver = {
	.driver = {
		.name = "cygnus-pm",
		.owner = THIS_MODULE,
		.of_match_table = iproc_pm_of_match,
	},
	.probe = iproc_pm_probe,
	.remove = iproc_pm_remove,
};
module_platform_driver(iproc_pm_platform_driver);

MODULE_VERSION(DRV_VER);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("An Luo, <an.luo@broadcom.com>");
MODULE_DESCRIPTION("Cygnus SoC Power Management Driver");
