/*
 * Early serial output macro for Broadcom IPROC SoC
 *
 * Copyright (C) 2014 Broadcom
 * Copyright (C) 1994-1999 Russell King
 * Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#if defined(CONFIG_ARCH_BCM_NSP) && defined(CONFIG_ARCH_BCM_CYGNUS)
#error "DEBUG_LL incompatible between ARCH_BCM_NSP and ARCH_BCM_CYGNUS"
#endif

#if defined(CONFIG_ARCH_BCM_NSP)
#define DEBUG_UART_PHYS	0x18000300
#define DEBUG_UART_VIRT	0xf1000300
#define DEBUG_UART_SH	0
#elif defined(CONFIG_ARCH_BCM_CYGNUS)
#define DEBUG_UART_PHYS	0x18023000
#define DEBUG_UART_VIRT	0xf1023000
#define DEBUG_UART_SH	2
#else
#error "Select uart for DEBUG_LL"
#endif

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =DEBUG_UART_PHYS 	@ MMU off, Physical
	ldr	\rv, =DEBUG_UART_VIRT 	@ MMU on, Virtual
	.endm

#define UART_SHIFT	DEBUG_UART_SH
#include <asm/hardware/debug-8250.S>
