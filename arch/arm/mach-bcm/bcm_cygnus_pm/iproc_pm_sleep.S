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
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "socregs.h"

	.align

/******************************************
 * only invalidate L1 caches, no flush
 ******************************************/
ENTRY(iproc_v7_invalidate_l1)
	mov     r0, #0
	mcr     p15, 2, r0, c0, c0, 0
	mrc     p15, 1, r0, c0, c0, 0

	ldr     r1, =0x7fff
	and     r2, r1, r0, lsr #13

	ldr     r1, =0x3ff

	and     r3, r1, r0, lsr #3	@ NumWays - 1
	add     r2, r2, #1		@ NumSets

	and     r0, r0, #0x7
	add     r0, r0, #4		@ SetShift

	clz     r1, r3			@ WayShift
	add     r4, r3, #1		@ NumWays
1:	sub     r2, r2, #1		@ NumSets--
	mov     r3, r4			@ Temp = NumWays
2:	subs    r3, r3, #1		@ Temp--
	mov     r5, r3, lsl r1
	mov     r6, r2, lsl r0
	orr     r5, r5, r6	@ Reg = (Temp<<WayShift)|(NumSets<<SetShift)
	mcr     p15, 0, r5, c7, c6, 2
	bgt     2b
	cmp     r2, #0
	bgt     1b
	dsb
	isb
	mov     pc, lr
ENDPROC(iproc_v7_invalidate_l1)

/******************************************
 * enable L1 D cache
 ******************************************/
ENTRY(iproc_pm_enable_L1_D_cache)
	dsb
	isb
	mrc  p15, 0, r0, c1, c0, 0
	orr  r0, r0, #(0x1 << 2)
	mcr  p15, 0, r0, c1, c0, 0
	isb
	mov pc, lr
ENDPROC(iproc_pm_enable_L1_D_cache)

/******************************************
 * disable L1 D cache
 ******************************************/
ENTRY(iproc_pm_disable_L1_D_cache)
	dsb
	isb
	mrc  p15, 0, r0, c1, c0, 0
	bic  r0, r0, #(0x1 << 2)    @ Disable D Cache
//	bic  r0, r0, #(0x1 << 12)   @ Disable I Cache
	mcr  p15, 0, r0, c1, c0, 0
	isb
	mov pc, lr
ENDPROC(iproc_pm_disable_L1_D_cache)

/******************************************
 * flush & disable L1 D cache
 ******************************************/
ENTRY(iproc_pm_flush_disable_L1_D_cache)
	mov  r12, lr
	dsb
	isb
	mov  lr, pc
	ldr  pc, =v7_flush_kern_cache_all  @ for long jump
//	bl   v7_flush_kern_cache_all       @ include v7_flush_dcache_all

#if 0
	dsb
	mov	 r0, #0		         	@ set up for MCR
	mcr	 p15, 0, r0, c8, c7, 0	@ invalidate TLBs, TLBIALL
//	mcr	 p15, 0, r0, c7, c5, 0	@ invalidate icache
//	mcr	 p15, 0, r0, c7, c5, 6	@ invalidate BP array
	isb
#endif

	bl   iproc_pm_disable_L1_D_cache
	mov  pc, r12
ENDPROC(iproc_pm_flush_disable_L1_D_cache)


/******************************************
 * Kernel resume entry
 ******************************************/
ENTRY(iproc_pm_cpu_resume)

	/* disable IRQ, FIQ, put into svc mode */
	msr cpsr_c, #PSR_I_BIT | PSR_F_BIT | SVC_MODE
	bl iproc_v7_invalidate_l1
	bl iproc_pm_disable_L1_D_cache
	b   cpu_resume
ENDPROC(iproc_pm_cpu_resume)
