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
#ifndef _BCM_CYGNUS_MAILBOX_H_
#define _BCM_CYGNUS_MAILBOX_H_

#include <linux/bcm_cygnus_m0_ipc.h>

/*
 * A message to send to the M0 processor.
 * @cmd Command to send.
 * @param Parameter corresponding to command.
 * @wait_ack true if mbox_send_message() should wait for a reply from the M0,
 *   false if the M0 doesn't reply. This depends on the message being sent.
 * @reply_code The response code from the M0 for the command sent (wait_ack was
 *   set to true).
 */
struct cygnus_mbox_msg {
	uint32_t  cmd;
	uint32_t  param;
	bool      wait_ack;
	uint32_t  reply_code;
};

#endif
