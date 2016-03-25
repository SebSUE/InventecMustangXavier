/*****************************************************************************
* Copyright 2014 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

#ifndef __ETH_ESW_IOCTL_H
#define __ETH_ESW_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

struct esw_reg_data {
	/* Switch access using 64-bit only */
	uint64_t data;
	/* Actually register length, in bits */
	size_t len;
	uint8_t page;
	uint8_t offset;
};


/* Data structure for entering Wake-up On Lan (WOL) mode
 * Only the port specified in the data structure will be enabled
 * during the WOL mode. Other Ports will be disabled.
 */
struct esw_wol_data {
	unsigned int port; /* The port to be left enabled */
	unsigned int port_speed; /* speed of the port in WOL mode */
};

#define SIOCESW_REG_READ         (SIOCDEVPRIVATE + 1)
#define SIOCESW_REG_WRITE        (SIOCDEVPRIVATE + 2)
#define SIOCESW_ENTER_WOL        (SIOCDEVPRIVATE + 3)
#define SIOCESW_EXIT_WOL         (SIOCDEVPRIVATE + 4)

#endif /* __ETH_ESW_IOCTL_H */
