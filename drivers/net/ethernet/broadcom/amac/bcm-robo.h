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

#ifndef _bcm_robo_h_
#define _bcm_robo_h_

#include <linux/types.h>

#include "bcm-amac-enet.h"


/* MPORT values. For adding ARL entries */
#define MPORT_RESERVED 0
#define MPORT_START    1
#define MPORT_LAST      5
#define MPORT_TOTAL    (MPORT_LAST - MPORT_START + 1)

#define PORT_MASK_ALL   0x103

#define PORT_INTERNAL   8

int bcm_esw_init(void *privp);
void bcm_esw_enable_multicast(struct bcm_amac_priv *privp, int enable);
void bcm_esw_set_mport_entry(struct bcm_amac_priv *privp, char *mac,
	u16 ethertype, u8 mport);
void bcm_esw_clear_all_mport_entry(struct bcm_amac_priv *privp);

int bcm_esw_set_arl_entry(struct bcm_amac_priv *privp, char *macp,
	int vid, int age, int portmask, bool entry_static);

void bcm_esw_clear_all_arl_entry(struct bcm_amac_priv *privp);
enum amac_reboot_reason bcm_esw_get_reboot(struct bcm_amac_priv *privp);

#endif /* _bcm_robo_h_ */
