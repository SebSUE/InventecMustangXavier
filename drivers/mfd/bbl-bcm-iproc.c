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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/jiffies.h>
#include <linux/ioctl.h>
#include <linux/bitops.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>

/*
 * High and Low temperature set for
 * TMON tamper detection
 * ADC 0x2A7 = 85C
 * ADC 0x36B = -10C
 */
#define TEMPERATURE_HIGH		0x2A7
#define TEMPERATURE_LOW			0x36B
/*
 * To enable-disbale all tamper sources,
 * int SRC and SRC1 registers
 */
#define ALL_SRC				0xffffffff
#define ALL_SRC1			0x3f
/* Bit to reset mesh logic */
#define EMESH_CONFIG			BIT(25)
/* Bit to clear emesh fault detector logic */
#define EMESH_CLR			BIT(10)
/*
 * n[8:1] are outputs, p[8:1] are inputs,
 * and dynamic mode enabled.
 */
#define EMESH_CONFIG_ENABLE_DYMMODE	0x180ffff
/*
 * Enable pull up on Emesh output
 * and select emesh filter period
 */
#define EMESH_CONFIG1			0x3e8
#define EMESH_CONFIG1_FILTER		(0x14<<2)
/* Enable internal mesh and dynamic mode */
#define MESH_CONFIG_TAPS_FAULT		0xE0
/* Enable Glitch filter on emesh pins */
#define EMESH_GLITCH_FILTER		(0x1fe << 18)
/*
 * Tamper input enable for digital
 * tamper input pairs
 */
#define TAMPERIN_INP_EN			0x1
/* Glitch filter for PN tamper */
#define GLITCH_FILTER_EN		BIT(18)
/* Power down fmon, do auto calibration */
#define FMON_CNG_AUTOCALIB		0x02000c30
/* Calibration data bit start */
#define FMON_CAL_REG			0x10
/* FMON calibration stat */
#define FMON_CALIB_STAT			0xA
/* To power on-off fmon */
#define FMON_CNG1_POWER_ON		0xA4
/*
 * Fmon wait time for calibrationa and
 * power on
 */
#define FMON_WAIT			50
/* Bit to reset FMON */
#define FMON_CNG1_OUT_RESET		BIT(3)
/* Bit to power in tmon */
#define TMON_CNG_PWRDN			BIT(1)
/* Offset for high and low temperature setting */
#define TMON_DELTA_LIMIT		0x1
#define TMON_MAX_LIMIT			0xB
/* Pad config bits for PN tamper */
#define PAD_CONFIG_UNATT_MODE		BIT(25)
#define PAD_CONFIG_IND			BIT(24)
/* Tmaper and interrupt bits for P0N0 */
#define TAMPER_SRC_P0N0			BIT(0)
#define INTR_P0N0			BIT(3)
/* EMESH clear delay */
#define EMESH_CLR_UDELAY		10
/* Reset cycle delay */
#define RESET_CYCLE_UDELAY		200
/* SPRU source select delay */
#define SPRU_SRC_SEL_UDELAY		1
/* RTC Seconds shift right */
#define RTC_SEC_SHIFT_RT		7

/* Tamper sources in SRC and SRC1 */
#define TAMPER_N0			BIT(0)
#define TAMPER_N1			BIT(1)
#define TAMPER_P0			BIT(9)
#define TAMPER_P1			BIT(10)
#define TAMPER_EMESH_F			BIT(18)
#define TAMPER_EMESH_O			BIT(19)
#define TAMPER_IMESH			BIT(20)
#define TAMPER_TMON_LOW			BIT(21)
#define TAMPER_TMON_HIGH		BIT(22)
#define TAMPER_FMON_LOW			BIT(23)
#define TAMPER_FMON_HIGH		BIT(24)
#define TAMPER_VOL			BIT(25)
#define TAMPER_1_VOL			BIT(2)
#define TAMPER_COIN_CELL		BIT(3)

/* Mask */
#define FMON_CALIB_MASK			0xff
#define TMON_TEMP_MASK			0xfffffffc
#define SPRU_BBL_REG_MASK		0x3FFF
#define RTC_SEC_MASK			0x3FFFFF
#define RTC_TM_TOSEC_MASK		0xFFFF
#define ADC_VAL				0x3ff

/* BBL SPRU Write/Read */
#define BBL_REG_ACC_DONE		BIT(0)
#define BBL_IND_SOFT_RST_N		BIT(10)
#define BBL_REG_WR_CMD			BIT(11)
#define BBL_REG_RD_CMD			BIT(12)

#define BBL_RAW_TAMPER_INTR_EN		BIT(8)
#define BBL_TWARN_INTR_EN		BIT(7)
#define BBL_TAMPERN_INTR_EN		BIT(6)
#define BBL_TAMPERIN_P_INTR_EN0		BIT(5)
#define BBL_TAMPERIN_P_INTR_EN1		BIT(4)
#define BBL_TAMPERIN_N_INTR_EN0		BIT(3)
#define BBL_TAMPERIN_N_INTR_EN1		BIT(2)

#define BBL_REG_TAMPER_INTR    (BBL_RAW_TAMPER_INTR_EN |\
		BBL_TAMPERN_INTR_EN |\
		BBL_TAMPERIN_P_INTR_EN0 |\
		BBL_TAMPERIN_P_INTR_EN1 |\
		BBL_TAMPERIN_N_INTR_EN0 |\
		BBL_TAMPERIN_N_INTR_EN1)

/* BBL register offset */
#define REG_BBL_RTC_PER			0x00
#define REG_BBL_RTC_MATCH		0x04
#define REG_BBL_RTC_DIV			0x08
#define REG_BBL_RTC_SECOND		0x0C
#define REG_BBL_INTERRUPT_EN		0x10
#define REG_BBL_INTERRUPT_STAT		0x14
#define REG_BBL_INTERRUPT_CLR		0x18
#define REG_BBL_CONTROL			0x1C
#define REG_BBL_TAMPER_TIMESTAMP	0x20
#define REG_BBL_TAMPER_SRC_ENABLE	0x24
#define REG_BBL_TAMPER_SRC_STAT		0x28
#define REG_BBL_TAMPER_SRC_CLEAR	0x2C
#define REG_BBL_TAMPER_SRC_ENABLE_1	0x30
#define REG_BBL_TAMPER_SRC_STAT_1	0x34
#define REG_BBL_TAMPER_SRC_CLEAR_1	0x38
#define REG_BBL_GLITCH_CFG		0x3C
#define REG_BBL_EN_TAMPERIN		0x40
#define REG_BBL_EMESH_CONFIG		0x44
#define REG_BBL_EMESH_CONFIG_1		0x48
#define REG_BBL_MESH_CONFIG		0x4C
#define REG_BBL_LFSR_SEED		0x50
#define REG_BBL_INPUT_STATUS		0x54
#define REG_BBL_CONFIG			0x58
#define REG_BBL_CONFIG_1		0x5C
#define REG_BBL_TMON_CONFIG		0x60
#define REG_BBL_TMON_CONFIG_1		0x64
#define REG_BBL_STAT			0x68
#define REG_FMON_CNG			0x6C
#define REG_FMON_CNG_1			0x70
#define REG_FMON_CNG_2			0x74
#define REG_FMON_CNG_3			0x78
#define REG_FMON_CNT_VAL		0x7C
#define REG_PAD_CONFIG			0x80
#define REG_TAMPER_INPUT_PULL_UP	0x84
#define REG_PAD_PULL_HYST		0x88
#define REG_DBG_CONFIG			0x8C
#define REG_XTAL_CONFIG			0x90
#define REG_LDO_CONFIG1			0x94
#define REG_LDO_CONFIG2			0x98
#define REG_LDO_CONFIG3			0x9C
#define REG_LDO_STATUS			0xA0
#define REG_TAMPER_INP_TIMEBASE		0xA4
#define REG_FILTER_THREHOLD_CONFIG1	0xA8
#define REG_FILTER_THREHOLD_CONFIG2	0xAC
#define REG_BBL_RSVD			0xB0
#define REG_BBL_WR_BLOCK		0x1CC

#define IREG_BBL_RTC_PER	0x00000000
#define IREG_BBL_RTC_MATCH	0x00000004
#define IREG_BBL_RTC_DIV	0x00000008
#define IREG_BBL_RTC_SECOND	0x0000000C
#define IREG_BBL_INTERRUPT_EN	0x00000010
#define IREG_BBL_INTERRUPT_STAT 0x00000014
#define IREG_BBL_INTERRUPT_CLR  0x00000018
#define IREG_BBL_CONTROL	0x0000001C

/* Period supported by BBL */
#define BBL_PER_125ms	0x00000001
#define BBL_PER_250ms	0x00000002
#define BBL_PER_500ms	0x00000004
#define BBL_PER_1s	0x00000008
#define BBL_PER_2s	0x00000010
#define BBL_PER_4s	0x00000020
#define BBL_PER_8s	0x00000040
#define BBL_PER_16s	0x00000080
#define BBL_PER_32s	0x00000100
#define BBL_PER_64s	0x00000200
#define BBL_PER_128s	0x00000400
#define BBL_PER_256s	0x00000800

#define CRMU_AUTH_CODE_PWD	0x12345678
#define CRMU_AUTH_CODE_PWD_RST	0x99999999
#define CRMU_AUTH_CODE_PWD_CLR	0x0

#define RTC_REG_ACC_DONE	BIT(0)
#define RTC_REG_RTC_STOP	BIT(0)
#define RTC_REG_PERIO_INTR	BIT(0)
#define RTC_REG_ALARM_INTR	BIT(1)
#define RTC_IND_SOFT_RST_N	BIT(10)
#define RTC_REG_WR_CMD		BIT(11)
#define RTC_REG_RD_CMD		BIT(12)
#define CRMU_ISO_PDBBL		BIT(16)
#define CRMU_ISO_PDBBL_TAMPER	BIT(24)
#define CRMU_ISO_CELL_CONTROL_OFFSET		0x0C
#define CRMU_SPRU_SOURCE_SEL_STAT_OFFSET	0x14
#define CRMU_BBL_AUTH_CODE_OFFSET		0x0
#define CRMU_BBL_AUTH_CHECK_OFFSET		0x4

/*
 * Timeout when waiting on register
 * reads or writes
 */
#define REG_TIMEOUT_MICROSECONDS 250

/*
 * SPRU Source Select status
 * 0 - SPRU is powered by AON power
 * 1 - SPRU is powerd by battery
 */
#define CRMU_SPRU_SOURCE_SEL_AON 0

/* SEC1280 Bank Register defines */
#define BBL_SEC0_MEM	0x200
#define BBL_SEC39_MEM	0x29C

/* Time to wait for BBL done status */
#define BBL_STATUS_TIMEOUT 1000

/* SFU update functionality */
#define SFUPDATE (1)

#if SFUPDATE
#define SFUPDATE_REG BBL_SEC0_MEM  /* secmem register used to store info*/
#define SFUPDATE_CNT_MASK (0x3fffffff)
#define SFUPDATE_CNT_SHIFT (0)
#define SFUPDATE_CNT_GET(x) (((x) & SFUPDATE_CNT_MASK) >> SFUPDATE_CNT_SHIFT)
#define SFUPDATE_CNT_SET(val, reg) ((reg & ~SFUPDATE_CNT_MASK) | (val << SFUPDATE_CNT_SHIFT))
#define SFUPDATE_FAIL_MASK (0x40000000)
#define SFUPDATE_FAIL_SHIFT (30)
#define SFUPDATE_FAIL_GET(x) (((x) & SFUPDATE_FAIL_MASK) >> SFUPDATE_FAIL_SHIFT)
#define SFUPDATE_FAIL_SET(val, reg) ((reg & ~SFUPDATE_FAIL_MASK) | (val << SFUPDATE_FAIL_SHIFT))
#define SFUPDATE_UPDATE_MASK (0x80000000)
#define SFUPDATE_UPDATE_SHIFT (31)
#define SFUPDATE_UPDATE_GET(x) (((x) & SFUPDATE_UPDATE_MASK) >> SFUPDATE_UPDATE_SHIFT)
#define SFUPDATE_UPDATE_SET(val, reg) ((reg & ~SFUPDATE_UPDATE_MASK) | (val << SFUPDATE_UPDATE_SHIFT))
#endif

struct bbl_regs {
	u32 REG_SPRU_BBL_WDATA;
	u32 REG_SPRU_BBL_CMD;
	u32 REG_SPRU_BBL_STATUS;
	u32 REG_SPRU_BBL_RDATA;
};

struct bcm_iproc_bbl {
	struct device *dev;
	struct rtc_device  *rtc;
	struct bbl_regs *regs;
	struct regmap *crmu_reg_pwr_good;
	struct regmap *bbl_auth;
	spinlock_t         lock;
	u32 bbl_tamper_irq;
	u32 periodic_irq;
	bool tamper_enable;
	bool rtc_enable;
	unsigned int bbl_secmem_address;
	unsigned int bbl_secmem_value;
};

static int wait_acc_done(struct bcm_iproc_bbl *iproc_bbl)
{
	int ret;
	unsigned long timeout = BBL_STATUS_TIMEOUT;

	ret = readl(&iproc_bbl->regs->REG_SPRU_BBL_STATUS);
	while (!(ret & BBL_REG_ACC_DONE)) {
		ret = readl(&iproc_bbl->regs->REG_SPRU_BBL_STATUS);
		timeout--;
		udelay(10);
		if (timeout == 0)
			return -EIO;
	}
	return 0;
}

static int bbl_reg_write(u32 reg_addr, u32 value,
	struct bcm_iproc_bbl *iproc_bbl)
{
	int ret;
	u32 cmd;

	writel(value, &iproc_bbl->regs->REG_SPRU_BBL_WDATA);
	cmd = (reg_addr & SPRU_BBL_REG_MASK) |
		BBL_REG_WR_CMD|BBL_IND_SOFT_RST_N; /* Write command */
	writel(cmd, &iproc_bbl->regs->REG_SPRU_BBL_CMD);
	ret = wait_acc_done(iproc_bbl);
	if (ret < 0)
		dev_err(iproc_bbl->dev, "BBL: reg write to 0x%x failed!",
			reg_addr);

	return ret;
}

static u32 bbl_reg_read(u32 reg_addr,
	u32 *data, struct bcm_iproc_bbl *iproc_bbl)
{
	int ret;
	u32 cmd;

	cmd = (reg_addr & SPRU_BBL_REG_MASK) |
		BBL_REG_RD_CMD|BBL_IND_SOFT_RST_N; /* Read command */
	writel(cmd, &iproc_bbl->regs->REG_SPRU_BBL_CMD);
	ret = wait_acc_done(iproc_bbl);
	if (ret < 0)
		dev_err(iproc_bbl->dev, "BBL: reg read to 0x%x failed!",
			reg_addr);
	else
		*data = readl(&iproc_bbl->regs->REG_SPRU_BBL_RDATA);
	return ret;
}

static int iproc_rtc_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 v;
	unsigned long flags;
	int ret;
	spin_lock_irqsave(&iproc_bbl->lock, flags);

	ret = bbl_reg_read(IREG_BBL_INTERRUPT_EN, &v, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Disable alarm&periodic interrupt */
	v &= ~(RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR);

	ret = bbl_reg_write(IREG_BBL_INTERRUPT_EN, v, iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_write(IREG_BBL_INTERRUPT_CLR,
		RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Set periodic timer as 1 second */
	ret = bbl_reg_write(IREG_BBL_RTC_PER, BBL_PER_1s, iproc_bbl);
	if (ret < 0)
		goto err;

	v |= RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR;
	/* enable RTC periodic interrupt */
	ret = bbl_reg_write(IREG_BBL_INTERRUPT_EN, v, iproc_bbl);
err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	return ret;

}

static int iproc_rtc_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 v;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&iproc_bbl->lock, flags);

	ret = bbl_reg_read(IREG_BBL_INTERRUPT_EN, &v, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Disable alarm and periodic interrupts */
	v &= ~(RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR);

	ret = bbl_reg_write(IREG_BBL_INTERRUPT_EN, v, iproc_bbl);

err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret == 0) {
		synchronize_irq(iproc_bbl->periodic_irq);
		synchronize_irq(iproc_bbl->bbl_tamper_irq);
	}
	return ret;
}

static int bbl_temp_sensor_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32  data;
	int ret;

	/*
	 * Set max and min temp limits
	 * LOW TEMP = -10C ==> 0x36B
	 * HIGH TEMP = 85C ==> 0x2A7
	 */
	ret = bbl_reg_read(REG_BBL_TMON_CONFIG, &data, iproc_bbl);
	if (data < 0)
		goto err;
	data &= TMON_CNG_PWRDN;
	data = data | (TEMPERATURE_LOW << TMON_MAX_LIMIT) |
			(TEMPERATURE_HIGH << TMON_DELTA_LIMIT);
	ret = bbl_reg_write(REG_BBL_TMON_CONFIG, data, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Power on tmon after tempertature limit is set */
	ret = bbl_reg_write(REG_BBL_TMON_CONFIG, data & ~(TMON_CNG_PWRDN),
		iproc_bbl);
	if (ret < 0)
		goto err;
	/*
	 * Enable both high and low temp
	 * monitors
	 */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data | TAMPER_TMON_LOW | TAMPER_TMON_HIGH, iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_read(REG_BBL_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "Current ADC reading = %03x\n",
		data & ADC_VAL);
err:
	return ret;
}

static int bbl_temp_sensor_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* power down temp monitor */
	ret = bbl_reg_read(REG_BBL_TMON_CONFIG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TMON_CONFIG, data | TMON_CNG_PWRDN,
			iproc_bbl);
	if (ret < 0)
		goto err;
	/* Disable tamper sources */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data & ~(TAMPER_TMON_LOW | TAMPER_TMON_HIGH), iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "Temperature monitor disabled\n");
err:
	return ret;

}

static int bbl_freq_monitor_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* run FMON auto calibration */
	ret = bbl_reg_write(REG_FMON_CNG,
		FMON_CNG_AUTOCALIB, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Power down fmon */
	ret = bbl_reg_write(REG_FMON_CNG_1,
		FMON_CNG1_POWER_ON, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Delay for auto calibration to complete */
	mdelay(FMON_WAIT);
	/* Reset FMON */
	ret = bbl_reg_write(REG_FMON_CNG_1,
		(FMON_CNG1_POWER_ON | FMON_CNG1_OUT_RESET), iproc_bbl);
	if (ret < 0)
		goto err;
	/* Delay for fmon reset */
	mdelay(FMON_WAIT);

	/* get FMON calibration value data[17:10] */
	ret = bbl_reg_read(REG_BBL_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	data = ((data >> FMON_CALIB_STAT) & FMON_CALIB_MASK);
	data = FMON_CNG_AUTOCALIB | (data << FMON_CAL_REG);
	/* Set calibration data for FMON configuration */
	ret = bbl_reg_write(REG_FMON_CNG, data, iproc_bbl);
	if (ret < 0)
		goto err;

	/* enable FMON as a tamper source */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		(data | TAMPER_FMON_LOW | TAMPER_FMON_HIGH), iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "Frequency monitor calibrated and enabled\n");
err:
	return ret;
}

static int bbl_freq_monitor_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* disable fmon as a tamper source */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data & ~(TAMPER_FMON_LOW | TAMPER_FMON_HIGH), iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "Frequency monitor disabled\n");
err:
	return ret;
}

static int bbl_voltage_monitor_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/*
	 * Enable voltage tamper detect,
	 * and coin cell go below 2.0V detect
	 */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data | (TAMPER_VOL), iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE_1, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE_1,
		data | TAMPER_1_VOL | TAMPER_COIN_CELL, iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "Voltage monitors enabled: Coincell drops below 2.0V");
err:
	return ret;
}

static int bbl_voltage_monitor_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/*
	 * Disable voltage tamper detection,
	 * and coin cell go below 2.0V detection
	 */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data & ~(TAMPER_VOL), iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE_1, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE_1,
		data & ~(TAMPER_1_VOL | TAMPER_COIN_CELL), iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "Voltage monitors disabled\n");
err:
	return ret;
}

static int bbl_tamper_pn_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* Enable Glitch filter */
	ret = bbl_reg_read(REG_BBL_GLITCH_CFG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_GLITCH_CFG, data | GLITCH_FILTER_EN,
			iproc_bbl);
	if (ret < 0)
		goto err;

	/* Enables the PN pair of bits as Digital Tamper Inputs */
	ret = bbl_reg_read(REG_BBL_EN_TAMPERIN, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_EN_TAMPERIN,
		data | TAMPERIN_INP_EN, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Enable TamperN and TamperP */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		(data | TAMPER_N0 |
		TAMPER_P0), iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "TAMPER_P0/P1 and TAMPER_N0/N1 activated\n");
err:
	return ret;
}

static int bbl_tamper_pn_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* Disable TamperN and TamperP */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		data & ~(TAMPER_N0 | TAMPER_P0),
		iproc_bbl);
	if (ret < 0)
		goto err;
	/* Disable the PN pair of bits as Digital Tamper Inputs */
	ret = bbl_reg_read(REG_BBL_EN_TAMPERIN, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_EN_TAMPERIN,
		data | ~(TAMPERIN_INP_EN), iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "TAMPER_P0/P1 and TAMPER_N0/N1 disabled\n");
err:
	return ret;
}

static int bbl_external_mesh_enable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/*
	 * reset mesh logic
	 * Writing 1 followed by 0 to reset
	 */
	ret = bbl_reg_read(REG_BBL_EMESH_CONFIG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
		data | (EMESH_CONFIG), iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
		data & (~(EMESH_CONFIG)), iproc_bbl);
	if (ret < 0)
		goto err;
	/* Enable internal mesh and dynamic mode */
	ret = bbl_reg_read(REG_BBL_MESH_CONFIG, &data, iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_write(REG_BBL_MESH_CONFIG,
			data | MESH_CONFIG_TAPS_FAULT,
			iproc_bbl);
	if (ret < 0)
		goto err;
	/*
	 * Enable Tamper outputs in emesh mode.
	 * n[8:1] are outputs, p[8:1] are inputs,
	 * and dynamic mode enabled.
	 */
	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
		EMESH_CONFIG_ENABLE_DYMMODE, iproc_bbl);
	if (ret < 0)
		goto err;

	/*
	 * Enable pull up on Emesh output
	 * and select emesh filter period
	 */
	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG_1,  EMESH_CONFIG1, iproc_bbl);
	if (ret < 0)
		goto err;

	/* Enable Glitch filter on emesh pins */
	ret = bbl_reg_read(REG_BBL_GLITCH_CFG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_GLITCH_CFG, data | EMESH_GLITCH_FILTER,
			iproc_bbl);
	if (ret < 0)
		goto err;

	/* Clear any exiting tampers */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR, data, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Enable mesh tamper detection */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		(data | TAMPER_EMESH_F | TAMPER_EMESH_O | TAMPER_IMESH),
		iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "emesh enabled!\n");
err:
	return ret;
}

static int bbl_external_mesh_disable(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	/* disable as tamper sources */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_ENABLE,
		(data &  ~(TAMPER_EMESH_F | TAMPER_EMESH_O | TAMPER_IMESH)),
		iproc_bbl);
	if (ret < 0)
		goto err;
	/* Disable imternal mesh and dynamic mode */
	ret = bbl_reg_write(REG_BBL_MESH_CONFIG, ~(EMESH_CONFIG_ENABLE_DYMMODE),
		iproc_bbl);
	if (ret < 0)
		goto err;

	/* reset mesh - writing '1; followed by a '0' */
	ret = bbl_reg_read(REG_BBL_EMESH_CONFIG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
		data | EMESH_CONFIG, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
		data & ~(EMESH_CONFIG), iproc_bbl);
	if (ret < 0)
		goto err;

	dev_info(iproc_bbl->dev, "External Mesh Grid and Internal mesh grid disabled\n");
err:
	return ret;
}

static int bbl_monitor_tampers(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data;
	int ret;

	dev_info(iproc_bbl->dev, "*********************************\n");
	dev_info(iproc_bbl->dev, "Tamper Monitor Running\n");
	dev_info(iproc_bbl->dev, "*********************************\n");
	ret = bbl_reg_read(REG_BBL_RTC_SECOND, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "RTC SECONDS            :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_INPUT_STATUS, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "RAW TAMPER/MESH STATUS :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_STAT         :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT_1, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_STAT_1         :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_TIMESTAMP, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER TIMESTAMP       :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_ENABLE      :%08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_ENABLE_1, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_ENABLE_1    :%08x\n", data);
	dev_info(iproc_bbl->dev, "*********************************\n");

err:
	return ret;
}

static int bbl_clear_tampers(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 data, data1;
	int ret;

	/* Check the tamper status */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_STAT  = %08x\n", data);
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT_1, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "TAMPER_SRC_STAT1 = %08x\n", data);
	/* Clear EMESH logic fault detector if set */
	if (data & ~(TAMPER_EMESH_F | TAMPER_EMESH_O)) {
		ret = bbl_reg_read(REG_BBL_EMESH_CONFIG, &data,
			iproc_bbl);
		if (ret < 0)
			goto err;
		ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
			data | (EMESH_CLR), iproc_bbl);
		if (ret < 0)
			goto err;
		udelay(EMESH_CLR_UDELAY);
		ret = bbl_reg_read(REG_BBL_EMESH_CONFIG, &data,
			iproc_bbl);
		if (ret < 0)
			goto err;
		ret = bbl_reg_write(REG_BBL_EMESH_CONFIG,
			data & (~(EMESH_CLR)), iproc_bbl);
		if (ret < 0)
			goto err;
	}
	/*
	 * Clear all sources
	 * To clear writing '1' to register
	 * followed by a '0'
	 */
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR,
		ALL_SRC, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR_1,
		ALL_SRC1, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR, 0, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR_1, 0, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Check if any tamper is still set */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data,
		iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT_1, &data1, iproc_bbl);
	if (ret < 0)
		goto err;
	data = data | data1;
	if (data > 0)
		dev_info(iproc_bbl->dev, "Active tampers still exist\n");
err:
	return ret;
}
static u32 bbl_show_rtc_secs(struct bcm_iproc_bbl *iproc_bbl)
{
	u32 rtc_secs;
	int ret;
	ret = bbl_reg_read(REG_BBL_RTC_SECOND, &rtc_secs, iproc_bbl);
	if (ret < 0)
		return ret;
	dev_info(iproc_bbl->dev, "RTC_SECOND = %08x", rtc_secs);
	return rtc_secs;
}

static ssize_t store_secmem_address(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	iproc_bbl->bbl_secmem_address = val;
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);

	if (iproc_bbl->bbl_secmem_address > BBL_SEC39_MEM)
		goto err;
	return strnlen(buf, count);
err:
	pr_err("BBL SecureMemory address %x is invalid\n", val);
	return -1;
}

static DEVICE_ATTR(bbl_secmem_address, (S_IWUSR|S_IWGRP),
		NULL, store_secmem_address);

static ssize_t store_secmem_write(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	iproc_bbl->bbl_secmem_value = val;
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);

	ret = bbl_reg_write(iproc_bbl->bbl_secmem_address,
			iproc_bbl->bbl_secmem_value, iproc_bbl);
	if (ret < 0)
		goto err;
	return strnlen(buf, count);
err:
	pr_err("BBL Secure Memory write of value :%x to address 0x%x failed\n",
		iproc_bbl->bbl_secmem_value, iproc_bbl->bbl_secmem_address);
	return ret;
}

static DEVICE_ATTR(bbl_secmem_write, (S_IWUSR|S_IWGRP),
			NULL, store_secmem_write);

static ssize_t show_bbl_secmem_dump(struct device *d,
			struct device_attribute *attr, char *buf)
{
	char *pos = buf;
	struct bcm_iproc_bbl *iproc_bbl;
	int i = 0, ret;
	unsigned int memaddr = 0;
	unsigned int data;
	iproc_bbl = dev_get_drvdata(d);

	while (memaddr <= BBL_SEC39_MEM) {
		memaddr = BBL_SEC0_MEM + i;
		ret = bbl_reg_read(memaddr, &data, iproc_bbl);
		if (ret < 0) {
			pr_err("BBL_SEC%d_MEM failed!\n", i);
			break;
		}
		i = i + 4;
		pos += sprintf(pos, "%x\n", data);
	}
	return pos - buf;
}

static DEVICE_ATTR(bbl_secmem_dump, S_IRUGO, show_bbl_secmem_dump, NULL);

static irqreturn_t bbl_sec_irq(int irq, void *pdev_data)
{
	u32 irq_flg;
	u32 irq_en;
	struct bcm_iproc_bbl *iproc_bbl;
	struct platform_device *pdev = (struct platform_device *)pdev_data;
	u32 data;
	unsigned long events = 0;
	int ret;

	iproc_bbl = (struct bcm_iproc_bbl *)platform_get_drvdata(pdev);

	spin_lock(&iproc_bbl->lock);
	ret = bbl_reg_read(REG_BBL_INTERRUPT_STAT, &irq_flg, iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_read(REG_BBL_INTERRUPT_EN, &irq_en, iproc_bbl);
	if (ret < 0)
		goto err;

	if (irq_en & irq_flg & RTC_REG_PERIO_INTR) {
		/* Clear periodic interrupt status */
		ret = bbl_reg_write(IREG_BBL_INTERRUPT_CLR, RTC_REG_PERIO_INTR,
			iproc_bbl);
		if (ret < 0)
			goto err;
		events |= RTC_IRQF | RTC_PF;
	}

	/* Check if any tamper has occured */
	if (irq_en & irq_flg & BBL_REG_TAMPER_INTR) {
		/* Check the tamper status */
		ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data, iproc_bbl);
		if (ret < 0)
			goto err;
		dev_err(iproc_bbl->dev, "BBL_TAMPER_SRC = %x\n", data);
		/* Disabling irq if tamper detected once */
		if (data)
			disable_irq_nosync(irq);
		ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT_1, &data, iproc_bbl);
		if (ret < 0)
			goto err;
		dev_err(iproc_bbl->dev, "BBL_TAMPER_SRC_1 = %x\n", data);
		/* Disabling irq if tamper detected once */
		if (data)
			disable_irq_nosync(irq);
	} else if (irq_en & irq_flg & RTC_REG_ALARM_INTR) {
		/* Clear alarm interrupt status */
		ret = bbl_reg_write(IREG_BBL_INTERRUPT_CLR, RTC_REG_ALARM_INTR,
			iproc_bbl);
		if (ret < 0)
			goto err;
		events |= RTC_IRQF | RTC_AF;

		ret = bbl_reg_read(IREG_BBL_INTERRUPT_EN, &data, iproc_bbl);
		if (ret < 0)
			goto err;
		data &= ~RTC_REG_ALARM_INTR;
		 /* Disable Alarm interrupt */
		ret = bbl_reg_write(IREG_BBL_INTERRUPT_EN, data, iproc_bbl);
		if (ret < 0)
			goto err;
	}
err:
	if (ret < 0)
		dev_err(iproc_bbl->dev, "bbl reg read/write failed\n");
	if (events && iproc_bbl->rtc_enable)
		rtc_update_irq(iproc_bbl->rtc, 1, events);
	spin_unlock(&iproc_bbl->lock);
	return IRQ_HANDLED;
}

static int iproc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
	u32 seconds;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	iproc_bbl = dev_get_drvdata(dev);

	spin_lock_irqsave(&iproc_bbl->lock, flags);

	ret = bbl_reg_read(IREG_BBL_RTC_SECOND, &seconds,
			iproc_bbl);
	if (ret < 0) {
		dev_err(dev, "RTC: iproc_rtc_read_time failed");
		goto err;
	}
	rtc_time_to_tm(seconds, tm);

	ret = rtc_valid_tm(tm);
err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	return ret;
}

static int iproc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags, t;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	iproc_bbl = dev_get_drvdata(dev);

	rtc_tm_to_time(tm, &t);
	spin_lock_irqsave(&iproc_bbl->lock, flags);

	/* bbl_rtc_stop = 1, RTC halt */
	ret = bbl_reg_write(IREG_BBL_CONTROL, RTC_REG_RTC_STOP, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Update DIV */
	ret = bbl_reg_write(IREG_BBL_RTC_DIV, 0, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Update second */
	ret = bbl_reg_write(IREG_BBL_RTC_SECOND, t, iproc_bbl);
	if (ret < 0)
		goto err;
	/* bbl_rtc_stop = 0, RTC release */
	ret = bbl_reg_write(IREG_BBL_CONTROL, ~RTC_REG_RTC_STOP, iproc_bbl);
	if (ret < 0)
		goto err;
err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		dev_err(dev, "RTC: iproc_rtc_set_time failed");

	return ret;
}

static int iproc_rtc_alarm_irq_enable(struct device *dev,
					u32 enabled)
{
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;
	u32 v;

	iproc_bbl = dev_get_drvdata(dev);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	ret = bbl_reg_read(IREG_BBL_INTERRUPT_EN, &v, iproc_bbl);
	if (ret < 0)
		goto err;
	/* Enable-Disable rtc alarm interrupt */
	if (enabled)
		v |= RTC_REG_ALARM_INTR;
	else
		v &= ~RTC_REG_ALARM_INTR;

	ret = bbl_reg_write(IREG_BBL_INTERRUPT_EN, v, iproc_bbl);
	if (ret < 0)
		goto err;
err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		dev_err(dev, "RTC: iproc_rtc_alarm_irq_enable failed");
	return ret;
}

static int iproc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long flags;
	u32 v, seconds;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	iproc_bbl = dev_get_drvdata(dev);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	ret = bbl_reg_read(IREG_BBL_RTC_MATCH, &seconds,
		iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_read(IREG_BBL_RTC_SECOND, &v, iproc_bbl);
	if (ret < 0)
		goto err;

	v &= ~RTC_SEC_MASK;
	seconds = (seconds << RTC_SEC_SHIFT_RT);
	seconds |= v;
	rtc_time_to_tm(seconds, &alm->time);
	ret = bbl_reg_read(IREG_BBL_INTERRUPT_EN, &v, iproc_bbl);
	if (ret < 0)
		goto err;

	v &= RTC_REG_ALARM_INTR;
	alm->pending = !v;
	alm->enabled = alm->pending && device_may_wakeup(dev);
err:
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		dev_err(dev, "RTC: iproc_rtc_read_alarm failed");
	return ret;
}

static int iproc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long flags;
	unsigned long seconds;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	iproc_bbl = dev_get_drvdata(dev);
	/*
	 * Setting RTC match to the time
	 * for alarm interrupt
	 */
	spin_lock_irqsave(&iproc_bbl->lock, flags);
	rtc_tm_to_time(&alm->time, &seconds);
	seconds = ((seconds&(RTC_TM_TOSEC_MASK<<7))>>7) & RTC_TM_TOSEC_MASK;
	ret = bbl_reg_write(IREG_BBL_RTC_MATCH, seconds, iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		dev_err(dev, "RTC: iproc_rtc_set_alarm failed");
	return ret;
}

static struct rtc_class_ops iproc_rtc_ops = {
	.read_time			= iproc_rtc_read_time,
	.set_time			= iproc_rtc_set_time,
	.alarm_irq_enable	= iproc_rtc_alarm_irq_enable,
	.read_alarm			= iproc_rtc_read_alarm,
	.set_alarm			= iproc_rtc_set_alarm,
};

static ssize_t show_rtc(struct device *d,
		struct device_attribute *attr,  char *buf)
{
	int val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;

	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	val = bbl_show_rtc_secs(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (val < 0)
		goto err;
	return sprintf(buf, "%d\n", val);
err:
	dev_err(d, "Show RTC failed\n");
	return val;
}
static DEVICE_ATTR(bbl_rtc_sec, (S_IRUSR|S_IRGRP), show_rtc, NULL);

static ssize_t store_tamper(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_monitor_tampers(iproc_bbl);
	else
		ret = bbl_clear_tampers(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;
	return strnlen(buf, count);
err:
	dev_err(d, "Tamper monitor/clear failed\n");
	return ret;
}
static DEVICE_ATTR(bbl_tamper_mon_clr, (S_IWUSR|S_IWGRP), NULL, store_tamper);

static ssize_t store_emesh(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_external_mesh_enable(iproc_bbl);
	else
		ret = bbl_external_mesh_disable(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;
	return strnlen(buf, count);
err:
	dev_err(d, "BBL mesh enable/disable failed\n");
	return ret;

}
static DEVICE_ATTR(bbl_emesh_en_dis, (S_IWUSR|S_IWGRP), NULL, store_emesh);

static ssize_t store_pn(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_tamper_pn_enable(iproc_bbl);
	else
		ret = bbl_tamper_pn_disable(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;
	return strnlen(buf, count);
err:
	dev_err(d, "BBL PN enable/disable failed\n");
	return ret;
}
static DEVICE_ATTR(bbl_pn_en_dis, (S_IWUSR|S_IWGRP), NULL, store_pn);

static ssize_t store_vmon(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_voltage_monitor_enable(iproc_bbl);
	else
		ret = bbl_voltage_monitor_disable(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;

	return strnlen(buf, count);
err:
	dev_err(d, "BBL Vmon enable/disable failed\n");
	return ret;
}
static DEVICE_ATTR(bbl_vmon_en_dis, (S_IWUSR|S_IWGRP), NULL, store_vmon);

static ssize_t store_tmon(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_temp_sensor_enable(iproc_bbl);
	else
		ret = bbl_temp_sensor_disable(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;
	return strnlen(buf, count);
err:
	dev_err(d, "BBL Vmon enable/disable failed\n");
	return ret;
}
static DEVICE_ATTR(bbl_tmon_en_dis, (S_IWUSR|S_IWGRP), NULL, store_tmon);

static ssize_t store_fmon(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	unsigned long flags;
	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	sscanf(buf, "%d", &val);
	iproc_bbl = dev_get_drvdata(d);

	spin_lock_irqsave(&iproc_bbl->lock, flags);
	if (val)
		ret = bbl_freq_monitor_enable(iproc_bbl);
	else
		ret = bbl_freq_monitor_disable(iproc_bbl);
	spin_unlock_irqrestore(&iproc_bbl->lock, flags);
	if (ret < 0)
		goto err;

	return strnlen(buf, count);
err:
	dev_err(d, "BBL Vmon enable/disable failed\n");
	return ret;
}
static DEVICE_ATTR(bbl_fmon_en_dis, (S_IWUSR|S_IWGRP), NULL, store_fmon);

#if SFUPDATE

static unsigned int sfupdate_read_reg(struct device *dev) {

	struct bcm_iproc_bbl *iproc_bbl;
	unsigned int data = 0;
	int ret = -1;

	iproc_bbl = dev_get_drvdata(dev);
	if (NULL == iproc_bbl) {
		pr_err("Get iproc_bbl faield!\n");
		return 0;
	}
	ret = bbl_reg_read(SFUPDATE_REG, &data, iproc_bbl);
	if (ret < 0) {
		pr_err("Get reg failed!\n");
		return 0;
	}
	return data;
}

static int sfupdate_write_reg(struct device *dev, unsigned int val) {

	struct bcm_iproc_bbl *iproc_bbl;
	int ret;

	iproc_bbl = dev_get_drvdata(dev);
	if (NULL == iproc_bbl) {
		pr_err("Get iproc_bbl faield!\n");
		return 0;
	}

	ret = bbl_reg_write(SFUPDATE_REG, val, iproc_bbl);
	if (ret < 0) {
		pr_err("BBL Secure Memory write of value :%x to address 0x%x failed\n",
			val, SFUPDATE_REG);
	}
	return ret;
}

/* Counter file */
static ssize_t counter_show(struct device *dev,
		 struct device_attribute *attr,
		 char *buf)
{
	unsigned int data;
	ssize_t ret;
	data = sfupdate_read_reg(dev);
	ret = sprintf(buf, "%u", SFUPDATE_CNT_GET(data));
	return ret;
}

static ssize_t counter_store(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	unsigned int reg;
	sscanf(buf, "%d", &val);
	// todo: CHECK VAL
	// todo: LOCK
	reg = sfupdate_read_reg(d);
	sfupdate_write_reg(d, SFUPDATE_CNT_SET(val, reg));
	// todo : UNLOCK
	return strnlen(buf, count);
}

static DEVICE_ATTR(counter, (S_IWUSR | S_IRUGO), counter_show, counter_store);

/* Fail file */
static ssize_t fail_show(struct device *dev,
		 struct device_attribute *attr,
		 char *buf)
{
	unsigned int data;
	ssize_t ret;
	data = sfupdate_read_reg(dev);
	ret = sprintf(buf, "%u", SFUPDATE_FAIL_GET(data));
	return ret;
}

static ssize_t fail_store(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	unsigned int reg;
	sscanf(buf, "%d", &val);
	//todo: CHECK VAL
	//todo: LOCK
	reg = sfupdate_read_reg(d);
	sfupdate_write_reg(d, SFUPDATE_FAIL_SET(val, reg));
	//todo: UNLOCK
	return strnlen(buf, count);
}

static DEVICE_ATTR(fail, (S_IWUSR | S_IRUGO), fail_show, fail_store);

/* Fail file */
static ssize_t update_show(struct device *dev,
		 struct device_attribute *attr,
		 char *buf)
{
	unsigned int data;
	ssize_t ret;
	data = sfupdate_read_reg(dev);
	ret = sprintf(buf, "%u", SFUPDATE_UPDATE_GET(data));
	return ret;
}

static ssize_t update_store(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	unsigned int reg;
	sscanf(buf, "%d", &val);
	//todo: CHECK VAL
	//todo: LOCK
	reg = sfupdate_read_reg(d);
	sfupdate_write_reg(d, SFUPDATE_UPDATE_SET(val, reg));
	//todo: UNLOCK
	return strnlen(buf, count);
}

static DEVICE_ATTR(update, (S_IWUSR | S_IRUGO), update_show, update_store);
static struct attribute *sfupdate_dev_attrs[] = {
	&dev_attr_counter.attr,
	&dev_attr_fail.attr,
	&dev_attr_update.attr,
	NULL
};

ATTRIBUTE_GROUPS(sfupdate_dev);

static struct miscdevice sfupdate_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sfupdate",
	.fops = NULL,

	.groups = sfupdate_dev_groups
};
#endif

static int bbl_init(struct bcm_iproc_bbl *iproc_bbl)
{
	int ret;
	u32 reg_val, data, timeout = REG_TIMEOUT_MICROSECONDS;

	/*
	 * Check SPRU Source Select status
	 * 0 - SPRU is powered by AON power
	 * 1 - SPRU is powerd by battery
	 */
	do {
		ret = regmap_read(iproc_bbl->crmu_reg_pwr_good,
			CRMU_SPRU_SOURCE_SEL_STAT_OFFSET, &reg_val);
		if (ret != 0) {
			dev_err(iproc_bbl->dev, "\nFailed to read CRMU_SPRU_SOURCE_SEL_STAT: %d"
				, ret);
			goto err;
		}
		if (--timeout == 0) {
			dev_info(iproc_bbl->dev, "RTC: BBL AON power not available\n");
			return -ENODEV;
		}
		udelay(SPRU_SRC_SEL_UDELAY);
	} while (reg_val != CRMU_SPRU_SOURCE_SEL_AON);

	/* Wait for reset cycle */
	writel(0, &iproc_bbl->regs->REG_SPRU_BBL_CMD);
	udelay(RESET_CYCLE_UDELAY);
	writel(BBL_IND_SOFT_RST_N, &iproc_bbl->regs->REG_SPRU_BBL_CMD);

	/* remove BBL related isolation from CRMU */
	ret = regmap_update_bits(iproc_bbl->crmu_reg_pwr_good,
			CRMU_ISO_CELL_CONTROL_OFFSET,
			(CRMU_ISO_PDBBL | CRMU_ISO_PDBBL_TAMPER),
			0);
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to update CRMU_ISO_PDBBL: %d",
			ret);
		goto err;
	}

	/* program CRMU auth_code resister */
	ret = regmap_write(iproc_bbl->bbl_auth,
			CRMU_BBL_AUTH_CODE_OFFSET, CRMU_AUTH_CODE_PWD);
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to write CRMU_BBL_AUTH_CODE: %d",
			ret);
		goto err;
	}
	/*
	 * program CRMU auth_code_check register
	 * auth_code must equal to auth_code_check
	 */
	ret = regmap_write(iproc_bbl->bbl_auth,
		CRMU_BBL_AUTH_CHECK_OFFSET, CRMU_AUTH_CODE_PWD);
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to write CRMU_BBL_AUTH_CHECK: %d",
			ret);
		goto err;
	}

	/*
	 * Need to set ind_unattended_mode (bit (25)
	 * and ind (bit 24) of PAD_CONFIG to zero
	 * in Cygnus B0, as the POR value is kept "1" to reduce
	 * the power consumption.
	 */
	ret = bbl_reg_read(REG_PAD_CONFIG, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_PAD_CONFIG,
		data & ~(PAD_CONFIG_UNATT_MODE | PAD_CONFIG_IND),
		iproc_bbl);
	if (ret < 0)
		goto err;
	/*
	 * Clearing the tamper stat and
	 * interrupt stat.
	 * To clear the bit, need to write a '1'
	 * to register and then clear the same.
	 */
	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_CLEAR, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR,
		data | TAMPER_SRC_P0N0, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_TAMPER_SRC_CLEAR,
		data & ~(TAMPER_SRC_P0N0), iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_INTERRUPT_CLR, INTR_P0N0, iproc_bbl);
	if (ret < 0)
		goto err;
	ret = bbl_reg_write(REG_BBL_INTERRUPT_CLR, ~(INTR_P0N0),
			iproc_bbl);
	if (ret < 0)
		goto err;

	ret = bbl_reg_read(REG_BBL_TAMPER_SRC_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "BBL_TAMPER_SRC_STAT=%x\n", data);
	ret = bbl_reg_read(REG_BBL_INTERRUPT_STAT, &data, iproc_bbl);
	if (ret < 0)
		goto err;
	dev_info(iproc_bbl->dev, "BBL_INTERRUPT_STAT=%x\n", data);

err:
	return ret;
}

static int bbl_exit(struct bcm_iproc_bbl *iproc_bbl)
{
	int ret;

	/* remove BBL related isolation from CRMU */
	ret = regmap_update_bits(iproc_bbl->crmu_reg_pwr_good,
			CRMU_ISO_CELL_CONTROL_OFFSET,
			(CRMU_ISO_PDBBL | CRMU_ISO_PDBBL_TAMPER),
			(CRMU_ISO_PDBBL | CRMU_ISO_PDBBL_TAMPER));
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to update CRMU_ISO_PDBBL: %d",
			ret);
		goto err;
	}

	/*
	 * Change the AUTH CODE register so it does not match
	 * the AUTH_CHECK register
	 */
	ret = regmap_write(iproc_bbl->bbl_auth,
			CRMU_BBL_AUTH_CODE_OFFSET, CRMU_AUTH_CODE_PWD_CLR);
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to write CRMU_BBL_AUTH_CODE: %d",
			ret);
		goto err;
	}

	ret = regmap_write(iproc_bbl->bbl_auth,
		CRMU_BBL_AUTH_CHECK_OFFSET, CRMU_AUTH_CODE_PWD_RST);
	if (ret != 0) {
		dev_err(iproc_bbl->dev, "\nFailed to write CRMU_BBL_AUTH_CHECK: %d",
			ret);
		goto err;
	}
err:
	return ret;
}


static struct attribute *bbl_dev_attrs[] = {
	&dev_attr_bbl_fmon_en_dis.attr,
	&dev_attr_bbl_tmon_en_dis.attr,
	&dev_attr_bbl_vmon_en_dis.attr,
	&dev_attr_bbl_pn_en_dis.attr,
	&dev_attr_bbl_emesh_en_dis.attr,
	&dev_attr_bbl_tamper_mon_clr.attr,
	&dev_attr_bbl_secmem_dump.attr,
	&dev_attr_bbl_secmem_address.attr,
	&dev_attr_bbl_secmem_write.attr,
	NULL
};

static struct attribute *bbl_rtc_dev_attrs[] = {
	&dev_attr_bbl_rtc_sec.attr,
	NULL
};

static struct attribute_group bbl_attr_group = {
	.name = "brcm-iproc-bbl",
	.attrs = bbl_dev_attrs,
};

static struct attribute_group bbl_rtc_attr_group = {
	.name = "brcm-iproc-bbl-rtc",
	.attrs = bbl_rtc_dev_attrs,
};

static const struct of_device_id iproc_bbl_of_match[] = {
	{.compatible = "brcm,iproc-bbl",},
	{ }
};

static int iproc_bbl_probe(struct platform_device *pdev)
{
	struct device_node *dev_of = pdev->dev.of_node;
	struct resource *res;
	struct bcm_iproc_bbl *iproc_bbl;
	u32 data;
	int ret;

	iproc_bbl = kzalloc(sizeof(*iproc_bbl), GFP_KERNEL);
	spin_lock_init(&iproc_bbl->lock);

	iproc_bbl->dev = &pdev->dev;

	iproc_bbl->bbl_tamper_irq = platform_get_irq(pdev, 0);
	if (iproc_bbl->bbl_tamper_irq < 0) {
		dev_err(&pdev->dev, "BBL interrupt not defined\n");
		ret = -ENODEV;
		goto fail;
	}

	iproc_bbl->periodic_irq = platform_get_irq(pdev, 1);
	if (iproc_bbl->periodic_irq < 0) {
		dev_err(&pdev->dev, "RTC periodic interrupt not defined\n");
		ret = -ENODEV;
		goto fail;
	}

	iproc_bbl->tamper_enable = of_property_read_bool(dev_of,
			"tamper-enable");

	iproc_bbl->rtc_enable = of_property_read_bool(dev_of, "rtc-enable");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "Unableto get resource\n");
		ret = -ENODEV;
		goto fail;
	}

	iproc_bbl->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!iproc_bbl->regs) {
		dev_err(&pdev->dev, "iomap of BBL MEM resource failed\n");
		ret = -ENOMEM;
		goto fail;
	}

	iproc_bbl->crmu_reg_pwr_good =
		syscon_regmap_lookup_by_phandle(dev_of, "crmu_pwr_good_syscon");
	if (IS_ERR(iproc_bbl->crmu_reg_pwr_good)) {
		dev_err(&pdev->dev,
		    "syscfg handle crmu_reg_pwr_good missing\n");
		ret = PTR_ERR(iproc_bbl->crmu_reg_pwr_good);
		goto fail;
	}

	iproc_bbl->bbl_auth = syscon_regmap_lookup_by_phandle(dev_of,
						    "bbl_auth_syscon");
	if (IS_ERR(iproc_bbl->bbl_auth)) {
		dev_err(&pdev->dev, "syscfg handle bbl_auth_syscon missing\n");
		ret = PTR_ERR(iproc_bbl->bbl_auth);
		goto fail;
	}

	platform_set_drvdata(pdev, iproc_bbl);

	ret = bbl_init(iproc_bbl);
	if (ret < 0)
		goto fail;

	if (iproc_bbl->rtc_enable) {
		iproc_bbl->rtc = rtc_device_register(pdev->name, &pdev->dev,
					&iproc_rtc_ops, THIS_MODULE);
		if (IS_ERR(iproc_bbl->rtc)) {
			ret = PTR_ERR(iproc_bbl->rtc);
			goto fail_bbl;
		}
		ret = sysfs_create_group(&pdev->dev.kobj, &bbl_rtc_attr_group);
		if (ret < 0)
			goto fail_rtc_unreg;

	}

	if (iproc_bbl->tamper_enable) {
		ret = sysfs_create_group(&pdev->dev.kobj, &bbl_attr_group);
		if (ret < 0)
			goto fail_rtc_unreg;
		ret = bbl_reg_read(REG_BBL_INTERRUPT_EN, &data,
			iproc_bbl);
		if (ret < 0)
			goto fail_sysfs;
		data |= BBL_REG_TAMPER_INTR;/* Enable BBL tamper interrupt */
		ret = bbl_reg_write(REG_BBL_INTERRUPT_EN, data, iproc_bbl);
		if (ret < 0)
			goto fail_sysfs;
	}

	ret = devm_request_irq(&pdev->dev, iproc_bbl->bbl_tamper_irq,
		bbl_sec_irq, IRQF_NO_SUSPEND,
		"iproc_bbl_tamper", pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register iproc BBL tamper interrupt\n");
		goto fail_sysfs;
	}

	ret = devm_request_irq(&pdev->dev, iproc_bbl->periodic_irq,
		bbl_sec_irq, IRQF_SHARED,
		"iproc_periodic_rtc", pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register iproc BBL periodic interrupt\n");
		goto fail_sysfs;
	}
	if (iproc_bbl->rtc_enable) {
		ret = iproc_rtc_enable(iproc_bbl);
		if (ret < 0) {
			dev_err(&pdev->dev, "RTC: Enable failed");
			goto fail_sysfs;
		}
		device_init_wakeup(&pdev->dev, 1);
	}
#if SFUPDATE
	if (iproc_bbl->tamper_enable) {
		ret = misc_register(&sfupdate_dev);
		/* Store iproc_bbl for misc driver */
		dev_set_drvdata(sfupdate_dev.this_device, iproc_bbl);
		if (ret) {
			printk(KERN_ERR
				"Unable to register \"sfupdate\" misc device\n");
			goto fail_sysfs;
		}
	}
#endif

	return 0;
fail_sysfs:
	if (iproc_bbl->tamper_enable)
		sysfs_remove_group(&pdev->dev.kobj, &bbl_attr_group);
fail_rtc_unreg:
	if (iproc_bbl->rtc_enable) {
		sysfs_remove_group(&pdev->dev.kobj, &bbl_rtc_attr_group);
		rtc_device_unregister(iproc_bbl->rtc);
	}
fail_bbl:
	bbl_exit(iproc_bbl);
fail:
	return ret;
}

static int iproc_bbl_remove(struct platform_device *pdev)
{
	int ret;
	struct bcm_iproc_bbl *iproc_bbl;
	iproc_bbl = platform_get_drvdata(pdev);

	if (iproc_bbl->tamper_enable) {
#if SFUPDATE
		misc_deregister(&sfupdate_dev);
#endif
		sysfs_remove_group(&pdev->dev.kobj, &bbl_attr_group);
	}
	if (iproc_bbl->rtc_enable) {
		device_init_wakeup(&pdev->dev, 0);
		ret = iproc_rtc_disable(iproc_bbl);
		if (ret < 0)
			dev_err(&pdev->dev, "RTC: Disable failed");
		rtc_device_unregister(iproc_bbl->rtc);
	}
	bbl_exit(iproc_bbl);
	kfree(iproc_bbl);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int iproc_rtc_suspend(struct device *dev)
{
	int ret;
	struct bcm_iproc_bbl *iproc_bbl;

	iproc_bbl = dev_get_drvdata(dev);

	ret = iproc_rtc_disable(iproc_bbl);
	if (ret < 0)
		dev_err(dev, "RTC: Disable failed");

	return ret;
}

static int iproc_rtc_resume(struct device *dev)
{
	int ret;
	struct bcm_iproc_bbl *iproc_bbl;

	iproc_bbl = dev_get_drvdata(dev);

	ret = iproc_rtc_enable(iproc_bbl);
	if (ret < 0)
		dev_err(dev, "RTC: Enable failed");

	return ret;
}

static const struct dev_pm_ops iproc_rtc_pm_ops = {
	.suspend = iproc_rtc_suspend,
	.resume = iproc_rtc_resume
};

#define IPROC_RTC_PM_OPS	(&iproc_rtc_pm_ops)
#else
#define IPROC_RTC_PM_OPS	NULL
#endif

static struct platform_driver iproc_bbl_driver = {
	.probe		= iproc_bbl_probe,
	.remove		= iproc_bbl_remove,
	.driver		= {
		.name = "iproc-bbl",
#ifdef CONFIG_PM_SLEEP
		.pm = IPROC_RTC_PM_OPS,
#endif
		.of_match_table = iproc_bbl_of_match
	},
};

module_platform_driver(iproc_bbl_driver);


MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC BBL Driver");
MODULE_LICENSE("GPL v2");



