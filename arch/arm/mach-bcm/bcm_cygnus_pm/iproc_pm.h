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
#ifndef __IPROC_PM_H
#define __IPROC_PM_H

#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_cygnus_mailbox.h>

#include "socregs.h"

#define iproc_bit_mask(n)              ((unsigned int)1<<n)
#define parse_integer(x)       simple_strtoul(x, NULL, 10)
#define parse_integer_hex(x)   simple_strtoul(x, NULL, 16)

/*Module info*/
#define SOC_NAME          "Cygnus"
#define DRV_NAME          "PM"
#define DRV_VER           "1.0"
#define DRV_INFO_PREFIX   "iProc-PM: "
#define DRV_ERR_PREFIX    "iProc-PM Err: "

enum iproc_power_status {
	IPROC_PM_STATE_RUN = 0,
	IPROC_PM_STATE_SLEEP,
	IPROC_PM_STATE_DEEPSLEEP,
	IPROC_PM_STATE_END
};

#define CRUM_M0_IDRAM_START	0x03010000
#define CRUM_M0_IDRAM_END	0x03017FFF
#define CRUM_M0_IDRAM_LEN	(CRUM_M0_IDRAM_END - CRUM_M0_IDRAM_START + 1)

struct bcm_iproc_pm {
	struct device *dev;
	struct mbox_client mbox_client;
	/*
	 * mask of AON GPIOs that should force immediate wakeup if high
	 * on sleep entry
	 */
	unsigned long aon_gpio_mask_skip_sleep_if_high;
	/*
	 * mask of AON GPIOs that should force immediate wakeup if low
	 * on sleep entry
	 */
	unsigned long aon_gpio_mask_skip_sleep_if_low;
};

typedef struct IPROC_Resources {
	struct resource *res;
	uint32_t *__iomem vaddr;
} iproc_resources_t;

typedef enum iproc_enable_disable {
	IPROC_DISABLED = 0,
	IPROC_ENABLED = 1,
} iproc_enable_disable_t;

extern unsigned char M0_BIN_START;
extern unsigned char M0_BIN_END;

/* Module functions */
#ifdef iproc_bitmask
#error "iproc_bitmask has re-defined!"
#else
/*  len=1~32 */
#define iproc_bitmask(len)	((len == 32) ? (0xffffffff) : ((1U<<len)-1))
#define iproc_bitmask_shifted(len, offset)	(iproc_bitmask(len)<<offset)
#endif

extern void *__iomem m0_idram_vbase;	/* m0_idram_res mapped vaddr */
extern struct resource m0_idram_res;
extern struct bcm_iproc_pm *iproc_pm;

uint32_t iproc_reg32_read(const uint32_t addr);
void iproc_reg32_write(const uint32_t addr, const uint32_t value);
void iproc_reg32_setbit(const uint32_t addr, const uint32_t bitno);
void iproc_reg32_clrbit(const uint32_t addr, const uint32_t bitno);

enum iproc_power_status iproc_get_current_pm_state(void);
void iproc_set_current_pm_state(enum iproc_power_status state);
char *iproc_get_pm_state_str_by_id(enum iproc_power_status state);
int iproc_mbox_send_msg(uint32_t cmd, uint32_t param, bool wait_ack);
int iproc_mbox_send_msg_with_struct(uint32_t cmd, void *param,
	size_t size, bool wait_ack);

int iproc_pm_m0_init(void);

void iproc_dump_resume_entry(void);

int iproc_soc_enter_sleep(enum iproc_power_status state);
int iproc_suspend_init(void);
void iproc_suspend_exit(void);
int iproc_soc_enter_run(void);

#endif				/* _IPROC_PM_H */
