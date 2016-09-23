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
#include <linux/sizes.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <asm/outercache.h>
#include <asm/io.h>
#include <asm/bug.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/mach/bcm_iproc_smc.h>
#include <linux/bcm_cygnus_mailbox.h>
#include "iproc_pm.h"
#include "iproc_pm_device.h"

/* Define to allow loading M0 binary from kernel for development purposes */
/* #define IPROC_PM_LOAD_M0_BIN */

extern void iproc_pm_cpu_resume(void);	/* cpu_v7_do_resume() wrapper */
extern void iproc_pm_disable_L1_D_cache(void);
extern void iproc_pm_enable_L1_D_cache(void);
extern void iproc_pm_flush_disable_L1_D_cache(void);

void iproc_dump_gic_regs(void);

extern struct mbox_chan *cygnus_mbox_chan;

static int iproc_pm_wfi_fallthru_report_bug(void)
{
	printk(KERN_ERR "ERROR entering power saving state:\n"
		"A9 exited WFI on sleep/deepsleep entry path\n");
	iproc_dump_gic_regs();
	BUG_ON(1);
	return 0;
}


static const struct of_device_id smc_node[] = {
	{.compatible = "brcm,iproc-smc"},
	{},
};

static inline bool iproc_in_secure_mode(void)
{
	/*
	 * If the SMC device node exists, it means that we're
	 * running in non-secure mode.  i.e.,
	 *
	 * SMC exists -> processor running in NON-SECURE mode
	 * SMC doesn't exist -> processor running in SECURE mode
	 */
	return !of_find_matching_node(NULL, smc_node);
}

/*Note: the reg order is important! */
static iproc_pm_reg_t iproc_common_saved_regs[] = {
	/* SCU */
	IPROC_SAVEREG(IHOST_SCU_CONTROL),
	IPROC_SAVEREG(IHOST_SCU_CONFIG),
	IPROC_SAVEREG(IHOST_SCU_FILTER_START),
	IPROC_SAVEREG(IHOST_SCU_FILTER_END),
	IPROC_SAVEREG(IHOST_SCU_ACCESS_CONTROL),
	IPROC_SAVEREG(IHOST_SCU_SECURE_ACCESS),
	/* GIC */
	IPROC_SAVEREG(IHOST_GICCPU_CONTROL),
	IPROC_SAVEREG(IHOST_GICCPU_PRIORITY_MASK),
};

static iproc_pm_device_regs_t iproc_pm_dev_common = {
	.devname = "SoC_single",
	.regs = &iproc_common_saved_regs,
	.regs_num = ARRAY_SIZE(iproc_common_saved_regs),
	.read = iproc_reg32_read,
	.write = iproc_reg32_write,
};

static void iproc_pm_flush_disable_l2_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	if (iproc_in_secure_mode()) {
		dev_info(iproc_pm->dev, "Flush & disable L2 cache!\n");
		dsb();
		isb();

		/* Disable L2 cache */
		outer_disable();
	} else {
		/* SMC to disable L2 cache and save its configuration */
		bcm_iproc_smc(SSAPI_DISABLE_L2_CACHE, 0, 0, 0, 0);
	}
#endif
}

static void iproc_pm_enable_l2_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	if (iproc_in_secure_mode()) {
		dev_info(iproc_pm->dev,
			"Resume L2 cache.\n");
		outer_resume();
	} else {
		/* SMC to enable L2 cache and restore its configuration */
		bcm_iproc_smc(SSAPI_ENABLE_L2_CACHE, 0, ~0UL, 0, 0);
	}
#endif
}

/*
 * For M0 IPC commands with struct as a paramater, sends message to M0 using
 *  mailbox framework, while taking care of cache-coherency of the struct
 * @cmd The command to send.
 * @param Virtual pointer to struct to be sent as command parameter
 * @size Size of the struct to be sent
 * @wait_ack true to wait for M0 to send a reply to this command, false
 *   to return immediately and not wait for a reply.
 * @return A negative value if sending the message failed, otherwise the reply
 *   code from the M0 indicating success or failure of executing the command: 0
 *   indicates success.
 */
int iproc_mbox_send_msg_with_struct(uint32_t cmd, void *param,
	size_t size, bool wait_ack)
{
	int ret;
	dma_addr_t dma_handle;

	dma_handle = dma_map_single(iproc_pm->dev, param, size,
		DMA_BIDIRECTIONAL);
	if (dma_mapping_error(iproc_pm->dev, dma_handle)) {
		dev_err(iproc_pm->dev,
			"Failed to dma map param of command 0x%x\n", cmd);
		return -ENOMEM;
	}

	ret = iproc_mbox_send_msg(cmd, dma_handle, wait_ack);

	dma_unmap_single(iproc_pm->dev, dma_handle, size, DMA_BIDIRECTIONAL);

	return ret;
}

/*
 * Sends message to M0 using mailbox framework.
 * @cmd The command to send.
 * @param The parameter corresponding to the command or 0 if n/a.
 * @wait_ack true to wait for M0 to send a reply to this command, false
 *   to return immediately and not wait for a reply.
 * @return A negative value if sending the message failed, otherwise the reply
 *   code from the M0 indicating success or failure of executing the command: 0
 *   indicates success.
 */
int iproc_mbox_send_msg(uint32_t cmd, uint32_t param, bool wait_ack)
{
	int ret;
	struct cygnus_mbox_msg msg;

	msg.cmd = cmd;
	msg.param = param;
	msg.wait_ack = wait_ack;
	ret = mbox_send_message(cygnus_mbox_chan, &msg);
	mbox_client_txdone(cygnus_mbox_chan, 0);

	return ret < 0 ? ret : msg.reply_code;
}

/*
  * Write the mailbox control code, then trigger the mailbox interrupt on M0
  */
int iproc_send_pm_mode_entry_msg(enum iproc_power_status state)
{
	uint32_t cmd;

	switch (state) {
	case IPROC_PM_STATE_SLEEP:
		cmd = M0_IPC_M0_CMD_ENTER_SLEEP;
		break;
	case IPROC_PM_STATE_DEEPSLEEP:
		cmd = M0_IPC_M0_CMD_ENTER_DEEPSLEEP;
		break;
	default:
		dev_err(iproc_pm->dev, "Not supported PM state: %d!\n", state);
		return -EINVAL;
	}

	iproc_mbox_send_msg(cmd, (uint32_t)virt_to_phys(iproc_pm_cpu_resume),
		false);

	return 0;
}

static int iproc_set_scu_status(enum iproc_power_status state)
{
	uint32_t pm_val = 0;

	switch (state) {
	case IPROC_PM_STATE_RUN:
		pm_val = 0xfffffffc;
		break;
	case IPROC_PM_STATE_SLEEP:
	case IPROC_PM_STATE_DEEPSLEEP:
		pm_val = 0xffffffff;
		break;
	default:
		dev_err(iproc_pm->dev, "Not supported PM state: %d!\n", state);
		return -EINVAL;
	}

	dev_info(iproc_pm->dev, "Set SCU power status to 0x%08x\n", pm_val);
	iproc_reg32_write(IHOST_SCU_POWER_STATUS, pm_val);

	return 0;
}

/*
  * write 2'bxx to SCU CPU Power Status Reg
  */
static int iproc_set_pwrctrlo_state(enum iproc_power_status state)
{
	int pm_val = 0;
	uint32_t PWRCTRLO = CRMU_IHOST_POWER_CONFIG;

	switch (state) {
	case IPROC_PM_STATE_RUN:
		pm_val = 0x0;
		break;
	case IPROC_PM_STATE_SLEEP:
		pm_val = 0x2;
		break;
	case IPROC_PM_STATE_DEEPSLEEP:
		pm_val = 0x3;
		break;
	default:
		dev_err(iproc_pm->dev, "Not supported PM state: %d!\n", state);
		return -EINVAL;
	}

	dev_info(iproc_pm->dev, "Set PWRCTRLO to %d(%s)\n", pm_val,
		  iproc_get_pm_state_str_by_id(state));

	iproc_reg32_write(PWRCTRLO, pm_val);

	return 0;
}

/*
  * Enable/disable iproc_gtimer, used by iproc_clocksource_init()
  */
static void iproc_set_system_timer(int en)
{
	if (en)	{
		/* restore */
		iproc_reg32_setbit(IHOST_GTIM_GLOB_CTRL,
				   IHOST_GTIM_GLOB_CTRL__Timer_en_G);
		enable_percpu_irq(27, 0);
		dev_info(iproc_pm->dev, "System timer enabled\n");
	} else {
		/* save */
		disable_percpu_irq(27);
		iproc_reg32_clrbit(IHOST_GTIM_GLOB_CTRL,
				   IHOST_GTIM_GLOB_CTRL__Timer_en_G);
		dev_info(iproc_pm->dev, "System timer disabled\n");
	}
}

#ifdef IPROC_PM_LOAD_M0_BIN
__asm__("\n\t"
	"	.global M0_BIN_START\n\t"
	"M0_BIN_START:\n\t"
	"	.incbin \"arch/arm/mach-bcm/bcm_cygnus_pm/M0_image.bin\"\n\t"
	"	.global M0_BIN_END\n\t" "M0_BIN_END:\n\t" "\n\t");

static int iproc_install_m0_bin(void)
{
	/* M0 binary image structure: <header> <data>
	 *
	 * Header structure:
	 *      <magic-start> <num-sections>
	 *		{<src-offset> <src-size> <dst-addr>}* <magic-end>
	 *
	 * M0 data (<data>) consists of several sections of code/data, to be
	 *      installed (copied) into M0 memories.
	 * Header (<header>) gives information about sections contained in
	 *	<data>.
	 *
	 * The installer code iterates over sections in M0 binary.
	 * For each section, it copies the section into M0 memory.
	 *
	 * The header contains:
	 *      - <magic-start> - 32-bit magic number to mark header start
	 *      - <num-sections> - number of sections in <data>
	 *      - <num-section> tuples. Each tuple describes a section.
	 *              A tuple contains three 32-bit words.
	 *      - <magic-end> - 32-bit magic numbers to mark header end
	 *
	 * Each section is describes by a tuple, consisting of three 32-bit
	 *	words:
	 *      - offset of section within M0 binary (relative to beginning of
	 *		<data>)
	 *      - section size (in bytes) in M0 binary
	 *      - target address (in M0 memory). Section is copied to this
	 *		location.
	 *
	 *      All fields are 32-bit unsigned integers in little endian format.
	 *      All sizes are assumed to be 32-bit aligned
	 */
#define M0_BIN_HEADER_MAGIC_START 0xfa587D01
#define M0_BIN_HEADER_MAGIC_END 0xf3e06a85
	uint32_t *bin_p = (uint32_t *) &M0_BIN_START;
	uint32_t data_len;
	uint32_t *data_p;
	uint32_t bin_len = &M0_BIN_END - &M0_BIN_START;
	uint32_t bin_data_offset;
	uint32_t num_sections = 0;

	dev_info(iproc_pm->dev, "Installing M0 image %p...\n", bin_p);
	/*
	 * pre-parse header to determine data start offset, and
	 * validate header start and end magic numbers
	 */
	/* prefix */
	if ((bin_len < 4) || (*bin_p != M0_BIN_HEADER_MAGIC_START))
		goto err;
	dev_info(iproc_pm->dev, "...Magic prefix OK\n");
	/* number of sections */
	if (bin_len < 8)
		goto err;
	num_sections = *(bin_p + 1);
	dev_info(iproc_pm->dev, "...Number of sections: %u\n", num_sections);

	/*
	 * 4-byte prefix; 4-byte num-of-sections field;
	 * sections (3 4-byte words each); 4-byte suffix
	 */
	bin_data_offset = 4 * (1 + 1 + 3 * num_sections + 1);
	/* suffix */
	if (bin_len < bin_data_offset)
		goto err;
	if (*(bin_p + bin_data_offset / 4 - 1) != M0_BIN_HEADER_MAGIC_END)
		goto err;
	dev_info(iproc_pm->dev, "...Magic suffix OK\n");
	data_len = bin_len - bin_data_offset;
	dev_info(iproc_pm->dev,
		"Bin len %d, data_len %d, bin_data_offset %d\n", bin_len,
		data_len, bin_data_offset);
	data_p = bin_p + bin_data_offset / 4;

	/* skip magin field and number-of-sections field */
	bin_p += 2;
	/* loop over sections */
	for (; num_sections > 0; num_sections--) {
		uint32_t src_offset = bin_p[0];
		uint32_t src_size = bin_p[1];
		uint32_t dst_addr_phys = bin_p[2];
		void *iomem_res;
		bin_p += 3;
		dev_info(iproc_pm->dev,
			"...Installing section: src_offset 0x%8.8x, src size %u dst_addr 0x%8.8x\n",
			src_offset, src_size, dst_addr_phys);
		if (src_offset + src_size > data_len) {
			dev_err(iproc_pm->dev,
				"Section exceeds data len 0x%x %d %d\n",
				src_offset, src_size, data_len);
			goto err;
		}

		iomem_res = ioremap_nocache(dst_addr_phys, src_size);
		if (iomem_res == NULL) {
			dev_err(iproc_pm->dev,
				"Failed to request_mem_region for section src_offset 0x%8.8x, src size %u, dst_addr 0x%8.8x\n",
				src_offset, src_size, dst_addr_phys);
			return -1;
		}
		/* copy from source to target section */
		memcpy_toio(iomem_res, data_p + src_offset / 4, src_size);
		iounmap(iomem_res);
	}
	return 0;
 err:
	dev_err(iproc_pm->dev, "M0 binary - wrong header format");
	return -1;
}
#endif

static int iproc_pm_m0_print_version(void)
{
	int ret;
	struct m0_ipc_m0_cmd_get_version *get_version;

	get_version = kmalloc(sizeof(*get_version),
		GFP_KERNEL);

	if (!get_version) {
		dev_err(iproc_pm->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = iproc_mbox_send_msg_with_struct(M0_IPC_M0_CMD_GET_VERSION,
		get_version, sizeof(*get_version), true);
	if (ret)
		dev_err(iproc_pm->dev, "Failed to get M0 version");
	else
		dev_info(iproc_pm->dev, "%s", get_version->version_string);

	kfree(get_version);
	return ret;
}

int iproc_pm_m0_init(void)
{
	int ret = 0;

#ifdef IPROC_PM_LOAD_M0_BIN
	ret = iproc_install_m0_bin();
	if (ret) {
		dev_err(iproc_pm->dev, "Failed to load M0 binary\n");
		goto  exit;
	}

	ret = iproc_mbox_send_msg(M0_IPC_M0_CMD_INIT, 0, true);
	if (ret) {
		dev_err(iproc_pm->dev, "Failed to init M0\n");
		goto exit;
	} else
		dev_info(iproc_pm->dev, "Cygnus M0 initialized\n");
#endif

	ret = iproc_mbox_send_msg(M0_IPC_M0_CMD_NOP, 0, true);
	if (ret) {
		dev_err(iproc_pm->dev, "Failed to communicate with M0\n");
		goto exit;
	}
	dev_info(iproc_pm->dev, "Cygnus M0 detected\n");

	iproc_pm_m0_print_version();

exit:
	return ret;
}

/* ////////////////////////////////////////////////////////////////////////// */
static void iproc_pm_save_user_regs(void)
{
	iproc_pm_save_device_regs(&iproc_pm_dev_common);
}

static void iproc_pm_restore_user_regs(void)
{
	iproc_pm_restore_device_regs(&iproc_pm_dev_common);
}

/* ////////////////////////////////////////////////////////////////////////// */
/*
  * This is the last step to enter new power mode
  */
static int iproc_prepare_enter_target_pm_mode(unsigned long state)
{
	if (state == IPROC_PM_STATE_RUN)
		return 0;

	iproc_set_pwrctrlo_state(state);

	iproc_set_scu_status(IPROC_PM_STATE_SLEEP);

	iproc_reg32_clrbit(CRU_status, CRU_status__detected_wfi);

	dsb();
	isb();

	return 0;
}

void iproc_dump_resume_entry(void)
{
	uint32_t reg_phy_addr;
	uint32_t *addr;

	reg_phy_addr = (uint32_t) iproc_pm_cpu_resume;
	addr = (uint32_t *) reg_phy_addr;
	dev_info(iproc_pm->dev, "iproc_pm_cpu_resume() vaddr = 0x%08x\n",
		reg_phy_addr);
	dev_info(iproc_pm->dev, "iproc_pm_cpu_resume() paddr = 0x%08x\n",
		  virt_to_phys((void *)reg_phy_addr));
}

/*common operation after wakeup*/
static void iproc_pm_common_task_after_wakeup(void)
{

	iproc_set_scu_status(IPROC_PM_STATE_RUN);

	iproc_set_system_timer(IPROC_ENABLED);
}

static int iproc_pm_finish_switch(long unsigned int sleep_state)
{
	phys_addr_t resume_addr;

	if ((sleep_state != IPROC_PM_STATE_SLEEP) &&
		(sleep_state != IPROC_PM_STATE_DEEPSLEEP)) {
		return 0;
	}

	/*
	 * On a successful sleep & wake, the CPU will eventually
	 * continue from after the cpu_suspend() call.
	 */
	if (iproc_in_secure_mode()) {

		/* Request MCU for a shutdown */
		iproc_send_pm_mode_entry_msg(sleep_state);

		/* Kernel to execute WFI */
		cpu_do_idle();
	} else {
		/*
		 * Use secure API to request entry into sleep state
		 */
		resume_addr = virt_to_phys(iproc_pm_cpu_resume);
		if (sleep_state == IPROC_PM_STATE_DEEPSLEEP)
			bcm_iproc_smc(
				SSAPI_SLEEP_DEEP,
				resume_addr, 0, 0, 0);
		else
			bcm_iproc_smc(
				SSAPI_SLEEP_DORMANT,
				resume_addr, 0, 0, 0);
	}

	/*
	 * If the CPU gets to this point via the normal execution path,
	 * we have a wfi drop-through situation!!!
	 */
	iproc_pm_wfi_fallthru_report_bug();

	return 0;
}

/* Linux Suspend to Ram or Standby */
static int __iproc_enter_sleep(enum iproc_power_status state)
{
	unsigned long saved_irq_flags;

	iproc_dump_resume_entry();

	iproc_pm_save_user_regs();

	iproc_set_system_timer(IPROC_DISABLED);

	local_irq_save(saved_irq_flags);
	local_irq_disable();

	local_flush_tlb_all();

	/* Save core regs, then suspending, wakeup in the __cpu_suspend() */
	iproc_prepare_enter_target_pm_mode(state);

	flush_cache_all();
	iproc_pm_flush_disable_l2_cache();

	dev_info(iproc_pm->dev, "Suspending ...\n");

	cpu_suspend(state, iproc_pm_finish_switch);

	dev_info(iproc_pm->dev, "... waking up\n");

	iproc_pm_enable_l2_cache();

	local_irq_restore(saved_irq_flags);

	iproc_pm_common_task_after_wakeup();

	iproc_pm_restore_user_regs();

	return 0;
}

int iproc_soc_enter_run(void)
{
	/* Set pwrctlo to RUN */
	iproc_set_pwrctrlo_state(IPROC_PM_STATE_RUN);

	return 0;
}

int iproc_soc_enter_sleep(enum iproc_power_status state)
{
	if ((state != IPROC_PM_STATE_SLEEP) &&
	    (state != IPROC_PM_STATE_DEEPSLEEP))
		return -EINVAL;

	dev_info(iproc_pm->dev, "%s\n",
		 iproc_in_secure_mode() ? "Secure Mode" : "Non-Secure Mode");
	dev_info(iproc_pm->dev, "Entering %s\n",
		 iproc_get_pm_state_str_by_id(state));
	__iproc_enter_sleep(state);
	return 0;
}

void iproc_dump_gic_regs(void)
{
	dev_info(iproc_pm->dev, "GIC interrupt enable set registers\n");
	dev_info(iproc_pm->dev, "    GICDIST_enable_set0:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set0));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set1:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set1));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set2:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set2));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set3:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set3));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set4:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set4));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set5:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set5));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set6:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set6));
	dev_info(iproc_pm->dev, "    GICDIST_enable_set7:    0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_enable_set7));
	dev_info(iproc_pm->dev, "GIC interrupt pending set registers\n");
	dev_info(iproc_pm->dev, "    GICDIST_pending_set0:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set0));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set1:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set1));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set2:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set2));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set3:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set3));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set4:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set4));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set5:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set5));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set6:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set6));
	dev_info(iproc_pm->dev, "    GICDIST_pending_set7:   0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_pending_set7));
	dev_info(iproc_pm->dev, "GIC interrupt status registers\n");
	dev_info(iproc_pm->dev, "    GICDIST_active_status0: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status0));
	dev_info(iproc_pm->dev, "    GICDIST_active_status1: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status1));
	dev_info(iproc_pm->dev, "    GICDIST_active_status2: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status2));
	dev_info(iproc_pm->dev, "    GICDIST_active_status3: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status3));
	dev_info(iproc_pm->dev, "    GICDIST_active_status4: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status4));
	dev_info(iproc_pm->dev, "    GICDIST_active_status5: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status5));
	dev_info(iproc_pm->dev, "    GICDIST_active_status6: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status6));
	dev_info(iproc_pm->dev, "    GICDIST_active_status7: 0x%08x\n",
		iproc_reg32_read(IHOST_GICDIST_active_status7));
}
