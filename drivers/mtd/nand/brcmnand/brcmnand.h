/*
 * Copyright © 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BRCMNAND_H__
#define __BRCMNAND_H__

#include <linux/types.h>
#include <linux/io.h>

struct platform_device;
struct dev_pm_ops;

#define NAND_ECC_MIPS_UNCORR_REG		0x18
#define NAND_ECC_MIPS_CORR_REG		0x1c


struct brcmnand_soc {
	bool (*ctlrdy_ack)(struct brcmnand_soc *soc);
	void (*ctlrdy_set_enabled)(struct brcmnand_soc *soc, bool en);
	void (*prepare_data_bus)(struct brcmnand_soc *soc, bool prepare);
	u32 (*read_ecc_mips_reg)(struct brcmnand_soc *soc, u32 reg);
	void (*write_ecc_mips_reg)(struct brcmnand_soc *soc, u32 reg, u32 value);
	
};

static inline void brcmnand_soc_data_bus_prepare(struct brcmnand_soc *soc)
{
	if (soc && soc->prepare_data_bus)
		soc->prepare_data_bus(soc, true);
}

static inline void brcmnand_soc_data_bus_unprepare(struct brcmnand_soc *soc)
{
	if (soc && soc->prepare_data_bus)
		soc->prepare_data_bus(soc, false);
}

static inline u32 brcmnand_soc_ecc_uncorr(struct brcmnand_soc *soc)
{
	if (soc && soc->read_ecc_mips_reg)
		return soc->read_ecc_mips_reg(soc, NAND_ECC_MIPS_UNCORR_REG);
	return 1;  // Error emulation??
}

static inline u32 brcmnand_soc_ecc_corr(struct brcmnand_soc *soc)
{
	if (soc && soc->read_ecc_mips_reg)
		return soc->read_ecc_mips_reg(soc, NAND_ECC_MIPS_CORR_REG);
	return 1; // Error emulation??
}

static inline void brcmnand_soc_uncorr_ack(struct brcmnand_soc *soc)
{
	if (soc && soc->write_ecc_mips_reg)
		soc->write_ecc_mips_reg(soc, NAND_ECC_MIPS_UNCORR_REG, 1);
}

static inline void brcmnand_soc_corr_ack(struct brcmnand_soc *soc)
{
	if (soc && soc->write_ecc_mips_reg)
		soc->write_ecc_mips_reg(soc, NAND_ECC_MIPS_CORR_REG, 1);
}

static inline u32 brcmnand_readl(void __iomem *addr)
{
	/*
	 * MIPS endianness is configured by boot strap, which also reverses all
	 * bus endianness (i.e., big-endian CPU + big endian bus ==> native
	 * endian I/O).
	 *
	 * Other architectures (e.g., ARM) either do not support big endian, or
	 * else leave I/O in little endian mode.
	 */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		return __raw_readl(addr);
	else
		return readl_relaxed(addr);
}

static inline void brcmnand_writel(u32 val, void __iomem *addr)
{
	/* See brcmnand_readl() comments */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		__raw_writel(val, addr);
	else
		writel_relaxed(val, addr);
}

int brcmnand_probe(struct platform_device *pdev, struct brcmnand_soc *soc);
int brcmnand_remove(struct platform_device *pdev);

extern const struct dev_pm_ops brcmnand_pm_ops;

#endif /* __BRCMNAND_H__ */
