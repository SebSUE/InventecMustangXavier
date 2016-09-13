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
#ifndef __IPROC_PM_DEVICE_H
#define __IPROC_PM_DEVICE_H

typedef struct iproc_pm_reg {
	void __iomem *regaddr;
	unsigned long regval;
} iproc_pm_reg_t;

typedef struct iproc_pm_device_regs {
	char *devname;
	 iproc_pm_reg_t(*regs)[];
	int regs_num;
	unsigned int (*read) (const unsigned int);
	void (*write) (const unsigned int, const unsigned int);
} iproc_pm_device_regs_t;

typedef struct iproc_pm_batch_regs {
	char *batch_name;
	void __iomem *startaddr;
	void __iomem *endaddr;
	unsigned int *saveaddr;
} iproc_pm_batch_regs_t;

typedef struct iproc_pm_device_batch_regs {
	char *name;
	 iproc_pm_batch_regs_t(*batch_regs)[];
	int batch_num;
	unsigned int (*read) (const unsigned int);
	void (*write) (const unsigned int, const unsigned int);
} iproc_pm_device_batch_regs_t;

#ifndef IPROC_SAVEREG
#define IPROC_SAVEREG(x) \
	{.regaddr = (void __iomem *)(x), .regval = 0}
#define IPROC_SAVE_BATCHREG(name, start, end) \
	{.batch_name = name, .startaddr = (void __iomem *)(start), .endaddr = (void __iomem *)(end), .saveaddr = 0}
#endif

extern int iproc_pm_save_device_regs(struct iproc_pm_device_regs *pm_dev);
extern int iproc_pm_restore_device_regs(struct iproc_pm_device_regs *pm_dev);
extern int iproc_pm_save_device_batch_regs(struct iproc_pm_device_batch_regs
					   *pm_dev);
extern int iproc_pm_restore_device_batch_regs(struct iproc_pm_device_batch_regs
					      *pm_dev);

#endif				/* _IPROC_PM_DEVICE_H */
