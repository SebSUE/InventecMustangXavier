/*
 * Copyright (C) 2014-2015 Broadcom Corporation
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "cygnus-ssp.h"

#define I2S0  0
#define I2S1  1
#define I2S2  2
#define SPDIF 3

#define CYGNUS_TDM_RATE \
		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define PLL_NDIV_FRACT_MAX  (BIT(20)-1)   /* 20 bits max */

#define CAPTURE_FCI_ID_BASE 0x180

/* Used with stream_on field to indicate which streams are active */
#define  PLAYBACK_STREAM_MASK   BIT(0)
#define  CAPTURE_STREAM_MASK    BIT(1)

/* Begin register offset defines */
#define AUD_MISC_SEROUT_OE_REG_BASE  0x01c
#define AUD_MISC_SEROUT_SPDIF_OE  12
#define AUD_MISC_SEROUT_MCLK_OE    3
#define AUD_MISC_SEROUT_LRCK_OE    2
#define AUD_MISC_SEROUT_SCLK_OE    1
#define AUD_MISC_SEROUT_SDAT_OE    0

/* AUD_FMM_BF_CTRL_xxx regs */
#define BF_DST_CFG0_OFFSET  0x100
#define BF_DST_CFG1_OFFSET  0x104
#define BF_DST_CFG2_OFFSET  0x108

#define BF_DST_CTRL0_OFFSET 0x130
#define BF_DST_CTRL1_OFFSET 0x134
#define BF_DST_CTRL2_OFFSET 0x138

#define BF_SRC_CFG0_OFFSET  0x148
#define BF_SRC_CFG1_OFFSET  0x14c
#define BF_SRC_CFG2_OFFSET  0x150
#define BF_SRC_CFG3_OFFSET  0x154

#define BF_SRC_CTRL0_OFFSET 0x1c0
#define BF_SRC_CTRL1_OFFSET 0x1c4
#define BF_SRC_CTRL2_OFFSET 0x1c8
#define BF_SRC_CTRL3_OFFSET 0x1cc

#define BF_SRC_GRP0_OFFSET  0x1fc
#define BF_SRC_GRP1_OFFSET  0x200
#define BF_SRC_GRP2_OFFSET  0x204
#define BF_SRC_GRP3_OFFSET  0x208

#define BF_SRC_GRP_EN_OFFSET        0x320
#define BF_SRC_GRP_FLOWON_OFFSET    0x324
#define BF_SRC_GRP_SYNC_DIS_OFFSET  0x328

/* AUD_FMM_IOP_OUT_I2S_xxx regs */
#define OUT_I2S_0_STREAM_CFG_OFFSET 0xa00
#define OUT_I2S_0_CFG_OFFSET        0xa04
#define OUT_I2S_0_MCLK_CFG_OFFSET   0xa0c

#define OUT_I2S_1_STREAM_CFG_OFFSET 0xa40
#define OUT_I2S_1_CFG_OFFSET        0xa44
#define OUT_I2S_1_MCLK_CFG_OFFSET   0xa4c

#define OUT_I2S_2_STREAM_CFG_OFFSET 0xa80
#define OUT_I2S_2_CFG_OFFSET        0xa84
#define OUT_I2S_2_MCLK_CFG_OFFSET   0xa8c

/* AUD_FMM_IOP_OUT_SPDIF_xxx regs */
#define SPDIF_STREAM_CFG_OFFSET  0xac0
#define SPDIF_CTRL_OFFSET        0xac4
#define SPDIF_FORMAT_CFG_OFFSET  0xad8
#define SPDIF_MCLK_CFG_OFFSET    0xadc

/* AUD_FMM_IOP_PLL_0_xxx regs */
#define IOP_PLL_0_MACRO_OFFSET    0xb00
#define IOP_PLL_0_MDIV_Ch0_OFFSET 0xb14
#define IOP_PLL_0_MDIV_Ch1_OFFSET 0xb18
#define IOP_PLL_0_MDIV_Ch2_OFFSET 0xb1c

#define IOP_PLL_0_ACTIVE_MDIV_Ch0_OFFSET 0xb30
#define IOP_PLL_0_ACTIVE_MDIV_Ch1_OFFSET 0xb34
#define IOP_PLL_0_ACTIVE_MDIV_Ch2_OFFSET 0xb38

/* AUD_FMM_IOP_xxx regs */
#define IOP_PLL_0_CONTROL_OFFSET     0xb04
#define IOP_PLL_0_USER_NDIV_OFFSET   0xb08
#define IOP_PLL_0_ACTIVE_NDIV_OFFSET 0xb20
#define IOP_PLL_0_RESET_OFFSET       0xb5c

/* AUD_FMM_IOP_IN_I2S_xxx regs */
#define IN_I2S_0_STREAM_CFG_OFFSET 0xc00
#define IN_I2S_0_CFG_OFFSET        0xc04
#define IN_I2S_1_STREAM_CFG_OFFSET 0xc40
#define IN_I2S_1_CFG_OFFSET        0xc44
#define IN_I2S_2_STREAM_CFG_OFFSET 0xc80
#define IN_I2S_2_CFG_OFFSET        0xc84

/* AUD_FMM_IOP_MISC_xxx regs */
#define IOP_SW_INIT_LOGIC          0xdc0

/* End register offset defines */


/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_0_REG */
#define I2S_OUT_MCLKRATE_SHIFT 16

/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_REG */
#define I2S_OUT_PLLCLKSEL_SHIFT  0

/* AUD_FMM_IOP_OUT_I2S_x_STREAM_CFG */
#define I2S_OUT_STREAM_ENA  31
#define I2S_OUT_STREAM_CFG_GROUP_ID  20
#define I2S_OUT_STREAM_CFG_CHANNEL_GROUPING  24

/* AUD_FMM_IOP_IN_I2S_x_CAP */
#define I2S_IN_STREAM_CFG_CAP_ENA   31
#define I2S_IN_STREAM_CFG_0_GROUP_ID 4

/* AUD_FMM_IOP_OUT_I2S_x_I2S_CFG_REG */
#define I2S_OUT_CFGX_CLK_ENA         0
#define I2S_OUT_CFGX_DATA_ENABLE     1
#define I2S_OUT_CFGX_LRCK_POLARITY   4
#define I2S_OUT_CFGX_SCLK_POLARITY   5
#define I2S_OUT_CFGX_DATA_ALIGNMENT  6
#define I2S_OUT_CFGX_BITS_PER_SAMPLE 8
#define I2S_OUT_CFGX_BIT_PER_SAMPLE_MASK 0x1F
#define I2S_OUT_CFGX_BITS_PER_SLOT  13
#define I2S_OUT_CFGX_VALID_SLOT     14
#define I2S_OUT_CFGX_FSYNC_WIDTH    18
#define I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32 26
#define I2S_OUT_CFGX_SLAVE_MODE     30
#define I2S_OUT_CFGX_TDM_MODE       31

#define I2S_IN_CFGX_DATA_ALIGNMENT   6
#define I2S_IN_CFGX_BITS_PER_SAMPLE  8
#define I2S_IN_CFGX_BIT_PER_SAMPLE_MASK 0x1F
#define I2S_IN_CFGX_BITS_PER_SLOT   13
#define I2S_IN_CFGX_VALID_SLOT      14
#define I2S_IN_CFGX_SLAVE_MODE      30
#define I2S_IN_CFGX_TDM_MODE        31

/* AUD_FMM_BF_CTRL_SOURCECH_CFGx_REG */
#define BF_SRC_CFGX_SFIFO_ENA              0
#define BF_SRC_CFGX_BUFFER_PAIR_ENABLE     1
#define BF_SRC_CFGX_SAMPLE_CH_MODE         2
#define BF_SRC_CFGX_SFIFO_SZ_DOUBLE        5
#define BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY  10
#define BF_SRC_CFGX_SAMPLE_REPEAT_ENABLE  11
#define BF_SRC_CFGX_BIT_RES               20
#define BF_SRC_CFGX_PROCESS_SEQ_ID_VALID  31
#define BF_SRC_CFGX_BITRES_MASK           0x1F

/* AUD_FMM_BF_CTRL_SOURCECH_CTRLx_REG */
#define BF_SOURCECH_CTRL_PLAY_RUN   0

/* AUD_FMM_BF_CTRL_DESTCH_CFGx_REG */
#define BF_DST_CFGX_CAP_ENA              0
#define BF_DST_CFGX_BUFFER_PAIR_ENABLE   1
#define BF_DST_CFGX_DFIFO_SZ_DOUBLE      2
#define BF_DST_CFGX_NOT_PAUSE_WHEN_FULL 11
#define BF_DST_CFGX_FCI_ID              12
#define BF_DST_CFGX_CAP_MODE            24
#define BF_DST_CFGX_PROC_SEQ_ID_VALID   31
#define BF_DST_CFGX_BITRES_MASK         0x1F

/* AUD_FMM_IOP_OUT_SPDIF_xxx */
#define SPDIF_0_OUT_DITHER_ENA     3
#define SPDIF_0_OUT_STREAM_ENA    31

/* AUD_FMM_IOP_PLL_0_USER */
#define IOP_PLL_0_USER_NDIV_FRAC   10

/* AUD_FMM_IOP_PLL_0_ACTIVE */
#define IOP_PLL_0_ACTIVE_NDIV_FRAC 10

#define IOP_LOGIC_RESET_IN_OFFSET(x) (x + 7) /* Capture ports offset by 7 */

#define INIT_SSP_REGS(num) { \
		.i2s_stream_cfg = OUT_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cap_stream_cfg = IN_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cfg = OUT_I2S_ ##num## _CFG_OFFSET, \
		.i2s_cap_cfg = IN_I2S_ ##num## _CFG_OFFSET, \
		.i2s_mclk_cfg = OUT_I2S_ ##num## _MCLK_CFG_OFFSET, \
		.bf_destch_ctrl = BF_DST_CTRL ##num## _OFFSET, \
		.bf_destch_cfg = BF_DST_CFG ##num## _OFFSET, \
		.bf_sourcech_ctrl = BF_SRC_CTRL ##num## _OFFSET, \
		.bf_sourcech_cfg = BF_SRC_CFG ##num## _OFFSET \
}

static int group_id[CYGNUS_MAX_PLAYBACK_PORTS] = {0, 1, 2, 3};

/*
 * Choose one of the following valid mclk rates:
 * -----------------------------------------
 * macro pll_ch0   pll_ch1    pll_ch2
 * ----- -------  ----------  ----------
 * 0   4,096,000   8,192,000  16,384,000
 * 1   5,644,800  11,289,600  22,579,200
 * 2   6,144,000  12,288,000  24,576,000
 * 3  12,288,000  24,576,000  49,152,000
 * 4  22,579,200  45,158,400  90,316,800
 * 5  24,576,000  49,152,000  98,304,000
 * -----------------------------------------
 *
 * Use this table to look up the "macro" setting for the audio pll block.
 * There are 6 macro settings that produce some common mclk frequencies.
 * The pll has 3 output channels (1x, 2x, and 4x).
 */
struct pll_macro_entry {
	u32 mclk;
	u32 macro;
	u32 pll_ch_num;
};

static const struct pll_macro_entry pll_predef_mclk[] = {
	{ 4096000, 0, 0},
	{ 8192000, 0, 1},
	{16384000, 0, 2},

	{ 5644800, 1, 0},
	{11289600, 1, 1},
	{22579200, 1, 2},

	{ 6144000, 2, 0},
	{12288000, 2, 1},
	{24576000, 2, 2},

	{12288000, 3, 0},
	{24576000, 3, 1},
	{49152000, 3, 2},

	{22579200, 4, 0},
	{45158400, 4, 1},
	{90316800, 4, 2},

	{24576000, 5, 0},
	{49152000, 5, 1},
	{98304000, 5, 2},
};

/* List of valid frame sizes for tdm mode */
static const int ssp_valid_tdm_framesize[] = {32, 64, 128, 256, 512};

/*
 * Use this relationship to derive the sampling rate (lrclk)
 * lrclk = (mclk) / ((2*mclk_to_sclk_ratio) * (32 * SCLK))).
 *
 * Use mclk, macro and pll_ch from the table above
 *
 * Valid SCLK = 0/1/2/4/8/12
 *
 * mclk_to_sclk_ratio = number of MCLK per SCLK. Division is twice the
 * value programmed in this field.
 * Valid mclk_to_sclk_ratio = 1 through to 15
 *
 * eg: To set lrclk = 48khz, set mclk = 12288000, mclk_to_sclk_ratio = 2,
 * SCLK = 64
 */
struct _ssp_clk_coeff {
	u32 mclk;
	u32 sclk_rate;
	u32 rate;
	u32 mclk_rate;
};

static const struct _ssp_clk_coeff ssp_clk_coeff[] = {
	{ 4096000,  32,  16000, 4},
	{ 4096000,  32,  32000, 2},
	{ 4096000,  64,   8000, 4},
	{ 4096000,  64,  16000, 2},
	{ 4096000,  64,  32000, 1},
	{ 4096000, 128,   8000, 2},
	{ 4096000, 128,  16000, 1},
	{ 4096000, 256,   8000, 1},

	{ 6144000,  32,  16000, 6},
	{ 6144000,  32,  32000, 3},
	{ 6144000,  32,  48000, 2},
	{ 6144000,  32,  96000, 1},
	{ 6144000,  64,   8000, 6},
	{ 6144000,  64,  16000, 3},
	{ 6144000,  64,  48000, 1},
	{ 6144000, 128,   8000, 3},

	{ 8192000,  32,  32000, 4},
	{ 8192000,  64,  16000, 4},
	{ 8192000,  64,  32000, 2},
	{ 8192000, 128,   8000, 4},
	{ 8192000, 128,  16000, 2},
	{ 8192000, 128,  32000, 1},
	{ 8192000, 256,   8000, 2},
	{ 8192000, 256,  16000, 1},
	{ 8192000, 512,   8000, 1},

	{12288000,  32,  32000, 6},
	{12288000,  32,  48000, 4},
	{12288000,  32,  96000, 2},
	{12288000,  32, 192000, 1},
	{12288000,  64,  16000, 6},
	{12288000,  64,  32000, 3},
	{12288000,  64,  48000, 2},
	{12288000,  64,  96000, 1},
	{12288000, 128,   8000, 6},
	{12288000, 128,  16000, 3},
	{12288000, 128,  48000, 1},
	{12288000, 256,   8000, 3},

	{16384000,  64,  32000, 4},
	{16384000, 128,  16000, 4},
	{16384000, 128,  32000, 2},
	{16384000, 256,   8000, 4},
	{16384000, 256,  16000, 2},
	{16384000, 256,  32000, 1},
	{16384000, 512,   8000, 2},
	{16384000, 512,  16000, 1},

	{24576000,  32,  96000, 4},
	{24576000,  32, 192000, 2},
	{24576000,  64,  32000, 6},
	{24576000,  64,  48000, 4},
	{24576000,  64,  96000, 2},
	{24576000,  64, 192000, 1},
	{24576000, 128,  16000, 6},
	{24576000, 128,  32000, 3},
	{24576000, 128,  48000, 2},
	{24576000, 256,   8000, 6},
	{24576000, 256,  16000, 3},
	{24576000, 256,  48000, 1},
	{24576000, 512,   8000, 3},

	{49152000,  32, 192000, 4},
	{49152000,  64,  96000, 4},
	{49152000,  64, 192000, 2},
	{49152000, 128,  32000, 6},
	{49152000, 128,  48000, 4},
	{49152000, 128,  96000, 2},
	{49152000, 128, 192000, 1},
	{49152000, 256,  16000, 6},
	{49152000, 256,  32000, 3},
	{49152000, 256,  48000, 2},
	{49152000, 256,  96000, 1},
	{49152000, 512,   8000, 6},
	{49152000, 512,  16000, 3},
	{49152000, 512,  48000, 1},

	{ 5644800,  32,  22050, 4},
	{ 5644800,  32,  44100, 2},
	{ 5644800,  32,  88200, 1},
	{ 5644800,  64,  11025, 4},
	{ 5644800,  64,  22050, 2},
	{ 5644800,  64,  44100, 1},

	{11289600,  32,  44100, 4},
	{11289600,  32,  88200, 2},
	{11289600,  32, 176400, 1},
	{11289600,  64,  22050, 4},
	{11289600,  64,  44100, 2},
	{11289600,  64,  88200, 1},
	{11289600, 128,  11025, 4},
	{11289600, 128,  22050, 2},
	{11289600, 128,  44100, 1},

	{22579200,  32,  88200, 4},
	{22579200,  32, 176400, 2},
	{22579200,  64,  44100, 4},
	{22579200,  64,  88200, 2},
	{22579200,  64, 176400, 1},
	{22579200, 128,  22050, 4},
	{22579200, 128,  44100, 2},
	{22579200, 128,  88200, 1},
	{22579200, 256,  11025, 4},
	{22579200, 256,  22050, 2},
	{22579200, 256,  44100, 1},

	{45158400,  32, 176400, 4},
	{45158400,  64,  88200, 4},
	{45158400,  64, 176400, 2},
	{45158400, 128,  44100, 4},
	{45158400, 128,  88200, 2},
	{45158400, 128, 176400, 1},
	{45158400, 256,  22050, 4},
	{45158400, 256,  44100, 2},
	{45158400, 256,  88200, 1},
	{45158400, 512,  11025, 4},
	{45158400, 512,  22050, 2},
	{45158400, 512,  44100, 1},
};

static int update_ssp_cfg(struct cygnus_aio_port *aio);

static struct cygnus_aio_port *cygnus_dai_get_portinfo(struct snd_soc_dai *dai)
{
	struct cygnus_audio *cygaud = snd_soc_dai_get_drvdata(dai);

	return &cygaud->portinfo[dai->id];
}

int cygnus_enable_i2s_32fs(struct snd_soc_dai *cpu_dai, bool enable)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	aio->enable_i2s_32fs = enable;

	return 0;
}

int cygnus_fix_multichan_tdm_shift_tx(struct snd_soc_dai *cpu_dai, bool enable)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	aio->fix_multichan_tdm_shift_tx = enable;

	return 0;
}

int cygnus_fix_multichan_tdm_shift_slave_rx(struct snd_soc_dai *cpu_dai,
		bool enable, int gpio)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	dev_dbg(aio->dev, "%s set gpio for port %d gpio %d\n", __func__,
		aio->portnum, gpio);

	aio->fix_multichan_tdm_shift_slave_rx = enable;
	if (enable)
		aio->gpio_latchctrl = gpio;
	else
		aio->gpio_latchctrl = -1;

	return 0;
}

static void audio_pll0_init(void __iomem *audio_io)
{
	/* Set clock channel post divider ratio to 0x24 */
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch0_OFFSET);
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch1_OFFSET);
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch2_OFFSET);
	/* Disable and enable digital and analog PLL */
	writel(0x3, audio_io + IOP_PLL_0_RESET_OFFSET);
	writel(0x0, audio_io + IOP_PLL_0_RESET_OFFSET);
}

static int audio_ssp_init_portregs(struct cygnus_aio_port *aio)
{
	u32 value, fci_id;

	if ((aio->portnum == I2S0) || (aio->portnum == I2S1)
			|| (aio->portnum == I2S2)) {
		value = readl(aio->audio + aio->regs.i2s_stream_cfg);
		value &= ~0xff003ff;

		/* Set Group ID */
		writel(group_id[0], aio->audio + BF_SRC_GRP0_OFFSET);
		writel(group_id[1], aio->audio + BF_SRC_GRP1_OFFSET);
		writel(group_id[2], aio->audio + BF_SRC_GRP2_OFFSET);
		writel(group_id[3], aio->audio + BF_SRC_GRP3_OFFSET);

		/* Configure the AUD_FMM_IOP_OUT_I2S_x_STREAM_CFG reg */
		value |= group_id[aio->portnum] << I2S_OUT_STREAM_CFG_GROUP_ID;
		value |= aio->portnum; /* FCI ID is the port num */
		value |= aio->channel_grouping <<
			I2S_OUT_STREAM_CFG_CHANNEL_GROUPING;
		writel(value, aio->audio + aio->regs.i2s_stream_cfg);

		/* Configure the AUD_FMM_BF_CTRL_SOURCECH_CFGX reg */
		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		value &= ~BIT(BF_SRC_CFGX_SAMPLE_REPEAT_ENABLE);
		value |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		value |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

		/* Configure the AUD_FMM_IOP_IN_I2S_x_CAP_STREAM_CFG_0 reg */
		value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
		value &= ~0xf0;
		value |= group_id[aio->portnum] << I2S_IN_STREAM_CFG_0_GROUP_ID;
		writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);

		/* Configure the AUD_FMM_BF_CTRL_DESTCH_CFGX_REG_BASE reg */
		fci_id = CAPTURE_FCI_ID_BASE + aio->portnum;

		value = readl(aio->audio + aio->regs.bf_destch_cfg);
		value |= BIT(BF_DST_CFGX_DFIFO_SZ_DOUBLE);
		value &= ~BIT(BF_DST_CFGX_NOT_PAUSE_WHEN_FULL);
		value |= (fci_id << BF_DST_CFGX_FCI_ID);
		value |= BIT(BF_DST_CFGX_PROC_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_destch_cfg);

		/* Enable the transmit pin for this port */
		value = readl(aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);
		value &= ~BIT((aio->portnum * 4) + AUD_MISC_SEROUT_SDAT_OE);
		writel(value, aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);

	} else if (aio->portnum == SPDIF) {
		writel(group_id[3], aio->audio + BF_SRC_GRP3_OFFSET);

		value = readl(aio->audio + SPDIF_CTRL_OFFSET);
		value |= BIT(SPDIF_0_OUT_DITHER_ENA);
		writel(value, aio->audio + SPDIF_CTRL_OFFSET);

		/* Enable and set the FCI ID for the SPDIF channel */
		value = readl(aio->audio + SPDIF_STREAM_CFG_OFFSET);
		value &= ~0x3ff;
		value |= aio->portnum; /* FCI ID is the port num */
		value |= BIT(SPDIF_0_OUT_STREAM_ENA);
		writel(value, aio->audio + SPDIF_STREAM_CFG_OFFSET);

		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		value |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		value |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

		/* Enable the spdif output pin */
		value = readl(aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);
		value &= ~BIT(AUD_MISC_SEROUT_SPDIF_OE);
		writel(value, aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);

	} else {
		dev_err(aio->dev, "Port not supported\n");
		return -EINVAL;
	}

	return 0;
}

static void audio_ssp_out_reset_toggle(struct cygnus_aio_port *aio)
{
	u32 value;

	/* IOP SW INIT on OUT_I2S_x */
	value = readl(aio->audio + IOP_SW_INIT_LOGIC);
	value |= BIT(aio->portnum);
	writel(value, aio->audio + IOP_SW_INIT_LOGIC);
	/* Need 1 bit clk tick for INIT_LOGIC to activate */
	udelay(10);
	value &= ~BIT(aio->portnum);
	writel(value, aio->audio + IOP_SW_INIT_LOGIC);

	/* Small delay to allow INIT_LOGIC to to activate 1 bit clk */
	udelay(10);
}

static void audio_ssp_in_enable(struct cygnus_aio_port *aio)
{
	u32 value;

	value = readl(aio->audio + aio->regs.bf_destch_cfg);
	value |= BIT(BF_DST_CFGX_CAP_ENA);
	writel(value, aio->audio + aio->regs.bf_destch_cfg);

	writel(0x1, aio->audio + aio->regs.bf_destch_ctrl);

	/*
	 * DATA_ENABLE need to be set even if doing capture.
	 * Subsequent Tx will fail if this is not done.
	 */
	if (aio->streams_on == 0) {
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value |= BIT(I2S_OUT_CFGX_CLK_ENA);
		value |= BIT(I2S_OUT_CFGX_DATA_ENABLE);
		writel(value, aio->audio + aio->regs.i2s_cfg);
	}

	value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
	value |= BIT(I2S_IN_STREAM_CFG_CAP_ENA);
	writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);

	/* Enable input portion of block */
	udelay(10);
	value = readl(aio->audio + IOP_SW_INIT_LOGIC);
	value &= ~BIT(IOP_LOGIC_RESET_IN_OFFSET(aio->portnum));
	writel(value, aio->audio + IOP_SW_INIT_LOGIC);

	/* Enable signals to pass thru */
	if (aio->fix_multichan_tdm_shift_slave_rx) {
		if (gpio_is_valid(aio->gpio_latchctrl))
			gpio_set_value(aio->gpio_latchctrl, 1);
	}

	aio->streams_on |= CAPTURE_STREAM_MASK;
}

static void audio_ssp_in_disable(struct cygnus_aio_port *aio)
{
	u32 value;

	value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
	value &= ~BIT(I2S_IN_STREAM_CFG_CAP_ENA);
	writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);

	aio->streams_on &= ~CAPTURE_STREAM_MASK;

	writel(0x0, aio->audio + aio->regs.bf_destch_ctrl);

	value = readl(aio->audio + aio->regs.bf_destch_cfg);
	value &= ~BIT(BF_DST_CFGX_CAP_ENA);
	writel(value, aio->audio + aio->regs.bf_destch_cfg);

	if (aio->fix_multichan_tdm_shift_slave_rx) {
		if (gpio_is_valid(aio->gpio_latchctrl)) {
			/* Block signals from passing through */
			gpio_set_value(aio->gpio_latchctrl, 0);
			udelay(20);
		}
	}

	/*
	 * Put input portion of port in reset.
	 * Clears residual data (32 bits) from internal formatter buffer
	 * BIT_CLOCK must be present for this to take effect
	 */
	value = readl(aio->audio + IOP_SW_INIT_LOGIC);
	value |= BIT(IOP_LOGIC_RESET_IN_OFFSET(aio->portnum));
	writel(value, aio->audio + IOP_SW_INIT_LOGIC);

	/* If both playback and capture are off */
	if (aio->streams_on == 0) {

		/*
		 * This is to handle the case where reset toggle was not done
		 * for master Tx while Rx was running
		 */
		audio_ssp_out_reset_toggle(aio);

		/*
		 * Add small delay before turning off clock
		 * Need 1 bit clock tick for INIT_LOGIC to activate
		 */
		udelay(10);
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value &= ~BIT(I2S_OUT_CFGX_CLK_ENA);
		value &= ~BIT(I2S_OUT_CFGX_DATA_ENABLE);
		writel(value, aio->audio + aio->regs.i2s_cfg);
	}
}

static int audio_ssp_out_enable(struct cygnus_aio_port *aio)
{
	u32 value;

	if (aio->portnum < SPDIF) {
		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value |= BIT(BF_SRC_CFGX_SFIFO_ENA);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

		writel(BIT(BF_SOURCECH_CTRL_PLAY_RUN),
			aio->audio + aio->regs.bf_sourcech_ctrl);

		value = readl(aio->audio + aio->regs.i2s_stream_cfg);
		value |= BIT(I2S_OUT_STREAM_ENA);
		writel(value, aio->audio + aio->regs.i2s_stream_cfg);

		value = readl(aio->audio + aio->regs.i2s_cfg);
		value |= BIT(I2S_OUT_CFGX_CLK_ENA);
		value |= BIT(I2S_OUT_CFGX_DATA_ENABLE);
		writel(value, aio->audio + aio->regs.i2s_cfg);

		/* Bring out of reset */
		value = readl(aio->audio + IOP_SW_INIT_LOGIC);
		value &= ~BIT(aio->portnum);
		writel(value, aio->audio + IOP_SW_INIT_LOGIC);


		aio->streams_on |= PLAYBACK_STREAM_MASK;
	} else if (aio->portnum == SPDIF) {
		value = readl(aio->audio + SPDIF_FORMAT_CFG_OFFSET);
		value |= 0x3;
		writel(value, aio->audio + SPDIF_FORMAT_CFG_OFFSET);

		writel(BIT(BF_SOURCECH_CTRL_PLAY_RUN),
			aio->audio + aio->regs.bf_sourcech_ctrl);

		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value |= BIT(BF_SRC_CFGX_SFIFO_ENA);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
	} else {
		dev_err(aio->dev, "Port not supported %d\n", aio->portnum);
		return -EINVAL;
	}

	return 0;
}

static void audio_ssp_out_shutdown_seq(struct cygnus_aio_port *aio,
					bool do_reset)
{
	u32 value;

	/* set group_sync_dis = 1 */
	value = readl(aio->audio + BF_SRC_GRP_SYNC_DIS_OFFSET);
	value |= BIT(aio->portnum);
	writel(value, aio->audio + BF_SRC_GRP_SYNC_DIS_OFFSET);

	writel(0, aio->audio + aio->regs.bf_sourcech_ctrl);

	value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
	value &= ~BIT(BF_SRC_CFGX_SFIFO_ENA);
	writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

	/* set group_sync_dis = 0 */
	value = readl(aio->audio + BF_SRC_GRP_SYNC_DIS_OFFSET);
	value &= ~BIT(aio->portnum);
	writel(value, aio->audio + BF_SRC_GRP_SYNC_DIS_OFFSET);

	if (do_reset)
		audio_ssp_out_reset_toggle(aio);

	/* If both playback and capture are off */
	if (aio->streams_on == 0) {
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value &= ~BIT(I2S_OUT_CFGX_DATA_ENABLE);
		value &= ~BIT(I2S_OUT_CFGX_CLK_ENA);
		writel(value, aio->audio + aio->regs.i2s_cfg);
	}
}

static int audio_ssp_out_disable(struct cygnus_aio_port *aio)
{
	u32 value;

	if (aio->portnum < SPDIF) {
		aio->streams_on &= ~PLAYBACK_STREAM_MASK;

		if ((aio->slave == 0) && (aio->mode == CYGNUS_SSPMODE_I2S))
			audio_ssp_out_shutdown_seq(aio, false);
		else
			audio_ssp_out_shutdown_seq(aio, true);
	} else if (aio->portnum == SPDIF) {
		value = readl(aio->audio + SPDIF_FORMAT_CFG_OFFSET);
		value &= ~0x3;
		writel(value, aio->audio + SPDIF_FORMAT_CFG_OFFSET);
		writel(0, aio->audio + aio->regs.bf_sourcech_ctrl);

		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~BIT(BF_SRC_CFGX_SFIFO_ENA);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
	} else {
		dev_err(aio->dev, "Port not supported %d\n", aio->portnum);
		return -EINVAL;
	}

	return 0;
}

static int pll_configure_mclk(void __iomem *audio_base, u32 mclk,
	struct device *dev)
{
	int i = 0;
	bool found = false;
	const struct pll_macro_entry *p_entry;

	for (i = 0; i < ARRAY_SIZE(pll_predef_mclk); i++) {
		p_entry = &pll_predef_mclk[i];

		if (p_entry->mclk == mclk) {
			found = true;
			break;
		}
	}
	if (!found) {
		dev_err(dev,
			"%s No valid mclk freq (%u) found!\n", __func__, mclk);
		return -EINVAL;
	}

	writel(p_entry->macro, audio_base + IOP_PLL_0_MACRO_OFFSET);
	dev_dbg(dev, "PLL Macro = %d\n", p_entry->macro);

	return p_entry->pll_ch_num;
}

static int cygnus_ssp_set_clocks(struct cygnus_aio_port *aio)
{
	u32 value;
	bool found = false;
	const struct _ssp_clk_coeff *p_entry = NULL;
	unsigned int bits_per_frame;
	unsigned int i;

	if ((!aio->lrclk) || (!aio->slots_per_frame) || (!aio->slot_width)) {
		dev_err(aio->dev, "First set up port through hw_params()\n");
		return -EINVAL;
	}

	bits_per_frame = aio->slots_per_frame * aio->slot_width;

	for (i = 0; i < ARRAY_SIZE(ssp_clk_coeff); i++) {
		p_entry = &ssp_clk_coeff[i];
		if ((p_entry->rate == aio->lrclk) &&
				(p_entry->sclk_rate == bits_per_frame) &&
				(p_entry->mclk == aio->mclk)) {
			found = true;
			break;
		}
	}
	if (!found) {
		dev_err(aio->dev,
			"No valid match found in ssp_clk_coeff array\n");
		dev_err(aio->dev, "%u %u %u\n",
			aio->lrclk, bits_per_frame, aio->mclk);
		return -EINVAL;
	}

	/* Set sclk rate */
	if (aio->portnum != SPDIF) {
		u32 mask = 0xF;
		u32 sclk;

		sclk = bits_per_frame;
		if (sclk == 512)
			sclk = 0;

		/* sclks_per_1fs_div = sclk cycles/32 */
		sclk /= 32;

		/* Set number of bitclks per frame */
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value &= ~(mask << I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32);
		value |= sclk << I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32;
		writel(value, aio->audio + aio->regs.i2s_cfg);
		dev_dbg(aio->dev, "SCLKS_PER_1FS_DIV32 = 0x%x\n", value);
	}

	/* Set MCLK_RATE ssp port (spdif and ssp are the same) */
	value = readl(aio->audio + aio->regs.i2s_mclk_cfg);
	value &= ~(0xF << I2S_OUT_MCLKRATE_SHIFT);
	value |= (p_entry->mclk_rate << I2S_OUT_MCLKRATE_SHIFT);
	writel(value, aio->audio + aio->regs.i2s_mclk_cfg);

	dev_dbg(aio->dev, "mclk cfg reg = 0x%x\n", value);
	dev_dbg(aio->dev, "bits per frame = %d, mclk = %d Hz, lrclk = %d Hz\n",
			bits_per_frame, aio->mclk, aio->lrclk);
	return 0;
}

static int cygnus_ssp_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);
	int rate, bitres, bits_per_sample;
	u32 value;
	u32 mask;
	int ret = 0;

	printk("[ADK] %s port = %d\n", __func__, aio->portnum);
	printk("[ADK]\tparams_channels %d\n", params_channels(params));
	printk("[ADK]\trate %d\n", params_rate(params));
	printk("[ADK]\tformat %d\n", params_format(params));

	rate = params_rate(params);

	switch (aio->mode) {
	case CYGNUS_SSPMODE_TDM:
		/* It's expected that set_dai_tdm_slot has been called */
		if ((rate == 192000) && (params_channels(params) > 4)) {
			dev_err(aio->dev, "Cannot run %d channels at %dHz\n",
				params_channels(params), rate);
			return -EINVAL;
		}
		break;
	case CYGNUS_SSPMODE_I2S:
		if (params_channels(params) != 2) {
			dev_err(aio->dev, "i2s mode must use 2 channels\n");
			return -EINVAL;
		}

		aio->active_slots = 2;
		aio->slots_per_frame = 2;
		if (params_format(params) == SNDRV_PCM_FORMAT_S16_LE)
			if (aio->enable_i2s_32fs)
				aio->slot_width = 16;
			else
				aio->slot_width = 32; /* Use 64Fs */
		else if (params_format(params) == SNDRV_PCM_FORMAT_S32_LE)
			aio->slot_width = 32;
		else
			return -EINVAL;

		break;
	default:
		dev_err(aio->dev, "%s unknown mode\n", __func__);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Configure channels as mono or stereo */
		if (params_channels(params) == 1) {
			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value |= BIT(BF_SRC_CFGX_SAMPLE_CH_MODE);
			value &= ~BIT(BF_SRC_CFGX_BUFFER_PAIR_ENABLE);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		} else {
			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value &= ~BIT(BF_SRC_CFGX_SAMPLE_CH_MODE);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		}

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			bitres = 8;
			bits_per_sample = 8;
			break;

		case SNDRV_PCM_FORMAT_S16_LE:
			bitres = 16;
			bits_per_sample = 16;
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
			bitres = 0; /* 32 bit mode is coded as 0 */
			bits_per_sample = 24; /* Only 24 valid bits */
			break;

		default:
			return -EINVAL;
		}

		mask = BF_SRC_CFGX_BITRES_MASK;
		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~(mask << BF_SRC_CFGX_BIT_RES);
		value |= (bitres << BF_SRC_CFGX_BIT_RES);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

		/* Only needed for LSB mode, ignored for MSB */
		mask = I2S_OUT_CFGX_BIT_PER_SAMPLE_MASK;
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value &= ~(mask << I2S_OUT_CFGX_BITS_PER_SAMPLE);
		value |= (bits_per_sample << I2S_OUT_CFGX_BITS_PER_SAMPLE);
		writel(value, aio->audio + aio->regs.i2s_cfg);
	} else {

		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			bits_per_sample = 16;

			value = readl(aio->audio + aio->regs.bf_destch_cfg);
			value |= BIT(BF_DST_CFGX_CAP_MODE);
			writel(value, aio->audio + aio->regs.bf_destch_cfg);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
			bits_per_sample = 24; /* Only 24 valid bits */

			value = readl(aio->audio + aio->regs.bf_destch_cfg);
			value &= ~BIT(BF_DST_CFGX_CAP_MODE);
			writel(value, aio->audio + aio->regs.bf_destch_cfg);
			break;

		default:
			return -EINVAL;
		}

		/* Used for both LSB and MSB modes */
		mask = I2S_IN_CFGX_BIT_PER_SAMPLE_MASK;
		value = readl(aio->audio + aio->regs.i2s_cap_cfg);
		value &= ~(mask << I2S_IN_CFGX_BITS_PER_SAMPLE);
		value |= (bits_per_sample << I2S_IN_CFGX_BITS_PER_SAMPLE);
		writel(value, aio->audio + aio->regs.i2s_cap_cfg);
	}

	/* Put output port into reset prior to configuring.
	 * Only do this if output is not active because this action would
	 * mess up the transfer. This code helps with the situation when
	 * configuring playback coming out of idle state. Keeping the port in
	 * reset during configuration helps with 16-bit slave tx problem.
	 */
	if (aio->streams_on == 0) {
		if (aio->slave) {
			value = readl(aio->audio + IOP_SW_INIT_LOGIC);
			value |= BIT(aio->portnum);
			writel(value, aio->audio + IOP_SW_INIT_LOGIC);
		}
		update_ssp_cfg(aio);

		aio->lrclk = rate;

		if (!aio->slave)
			ret = cygnus_ssp_set_clocks(aio);
	}

	return ret;
}

/*
 * This function sets the mclk frequency for pll clock
 */
static int cygnus_ssp_set_sysclk(struct snd_soc_dai *dai,
			int clk_id, unsigned int freq, int dir)
{
	int sel;
	u32 value;
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	dev_dbg(aio->dev, "%s Enter port = %d\n", __func__, aio->portnum);
	sel = pll_configure_mclk(aio->audio, freq, aio->dev);
	if (sel < 0) {
		dev_err(aio->dev, "%s Setting mclk failed.\n", __func__);
		return -EINVAL;
	}

	aio->mclk = freq;

	dev_dbg(aio->dev, "%s Setting MCLKSEL to %d\n", __func__, sel);
	value = readl(aio->audio + aio->regs.i2s_mclk_cfg);
	value &= ~(0xF << I2S_OUT_PLLCLKSEL_SHIFT);
	value |= (sel << I2S_OUT_PLLCLKSEL_SHIFT);
	writel(value, aio->audio + aio->regs.i2s_mclk_cfg);

	/* Clear bit for active */
	value = readl(aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);
	value &= ~BIT(AUD_MISC_SEROUT_MCLK_OE + (aio->portnum * 4));
	writel(value, aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);

	return 0;
}

static int cygnus_ssp_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	snd_soc_dai_set_dma_data(dai, substream, aio);

	return 0;
}

int cygnus_ssp_set_custom_fsync_width(struct snd_soc_dai *cpu_dai, int len)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	if ((len > 0) && (len < 256)) {
		aio->fsync_width = len;
		return 0;
	} else {
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(cygnus_ssp_set_custom_fsync_width);

static int cygnus_ssp_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	printk("[ADK] %s Enter  portnum=%d,  fmt: %x\n", __func__, aio->portnum, fmt);

	if (aio->portnum == SPDIF)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aio->slave = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aio->slave = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aio->fs_delay = 1;
		aio->mode = CYGNUS_SSPMODE_I2S;
		break;

	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		/* DSP_A = data after FS, DSP_B = data during FS */
		if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A)
			aio->fs_delay = 1;
		else {
			if (aio->slave) {
				dev_err(aio->dev,
				"%s DSP_B mode not supported while slave.\n",
					__func__);
				return -EINVAL;
			}
			aio->fs_delay = 0;
		}
		aio->mode = CYGNUS_SSPMODE_TDM;
printk("[ADK] %s port=%d: set to TDM mode\n", __func__, aio->portnum);
		break;

	default:
		return -EINVAL;
	}

	if (aio->mode != CYGNUS_SSPMODE_TDM)
	{
	// [ADK] Add for Mustang ..
	/* We must be i2s master to invert any clock */
		if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
			if (aio->slave || (aio->mode == CYGNUS_SSPMODE_TDM)) {
				dev_err(aio->dev,
				"%s Can only invert clocks in i2s master mode\n",
					__func__);
				return -EINVAL;
			}
		}
	}
	
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		if (aio->mode != CYGNUS_SSPMODE_TDM)
			aio->invert_bclk = true;
		else
			aio->invert_bclk = false;
		aio->invert_fs = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		aio->invert_bclk = false;
		aio->invert_fs = true;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		aio->invert_bclk = true;
		aio->invert_fs = true;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		aio->invert_bclk = false;
		aio->invert_fs = false;
		break;
	default:
		return -EINVAL;
	}

	if ((0 <= aio->portnum) && (aio->portnum <= 2)) {
		u32 val;
		u32 mask;

		val = readl(aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);

		/*
		 * Configure the word clk and bit clk as output or tristate
		 * Each port has 4 bits for controlling its pins.
		 * Shift the mask based upon port number.
		 */
		mask = BIT(AUD_MISC_SEROUT_LRCK_OE)
			| BIT(AUD_MISC_SEROUT_SCLK_OE);
		mask = mask << (aio->portnum * 4);
		if (aio->slave)
			val |= mask;   /* Set bit for tri-state */
		else
			val &= ~mask;  /* Clear bit for active */

		dev_dbg(aio->dev, "%s  Set OE bits 0x%x\n", __func__, val);
		writel(val, aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);
	}

	return 0;
}

static int cygnus_ssp_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	dev_dbg(aio->dev,
		"%s cmd %d at port = %d\n", __func__, cmd, aio->portnum);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			audio_ssp_out_enable(aio);
		else
			audio_ssp_in_enable(aio);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			audio_ssp_out_disable(aio);
		else
			audio_ssp_in_disable(aio);

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int cygnus_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);
	unsigned int active_slots;
	unsigned int bits_per_frame;
	bool found = false;
	unsigned int i;

	printk("[ADK] %s Enter, tx/rx=0x%x/0x%x, slots=%d, slot_width=%d\n", __func__, 
		tx_mask, rx_mask, slots, slot_width);

	if (tx_mask != rx_mask) {
		dev_err(aio->dev, "%s tx_mask must equal rx_mask\n", __func__);
		return -EINVAL;
	}

	active_slots = hweight32(tx_mask);

	if ((active_slots == 0) || (active_slots > 16))
		return -EINVAL;

	/* Slot value must be even */
	if (active_slots % 2)
		return -EINVAL;

	if ((slot_width != 16) && (slot_width != 32))
		return -EINVAL;

	bits_per_frame = slots * slot_width;

	for (i = 0; i < ARRAY_SIZE(ssp_valid_tdm_framesize); i++) {
		if (ssp_valid_tdm_framesize[i] == bits_per_frame) {
			found = true;
			break;
		}
	}

	if (!found) {
		dev_err(aio->dev, "%s In TDM mode, frame bits INVALID (%d)\n",
			__func__, bits_per_frame);
		return -EINVAL;
	}

	aio->active_slots = active_slots;
	aio->slot_width = slot_width;
	aio->slots_per_frame = slots;

	dev_dbg(aio->dev, "%s active_slots %u, bits per frame %d\n",
			__func__, aio->active_slots, bits_per_frame);
	return 0;
}

/*
 * Bit    Update  Notes
 * 31     Yes     TDM Mode        (1 = TDM, 0 = i2s)
 * 30     Yes     Slave Mode      (1 = Slave, 0 = Master)
 * 29:26  No      Sclks per frame
 * 25:18  Yes     FS Width
 * 17:14  No      Valid Slots
 * 13     No      Bits            (1 = 16 bits, 0 = 32 bits)
 * 12:08  No      Bits per samp
 * 07     Yes     Justifcation    (1 = LSB, 0 = MSB)
 * 06     Yes     Alignment       (1 = Delay 1 clk, 0 = no delay
 * 05     Yes     SCLK polarity   (1 = Rising, 0 = Falling)
 * 04     Yes     LRCLK Polarity  (1 = High for left, 0 = Low for left)
 * 03:02  Yes     Reserved - write as zero
 * 01     No      Data Enable
 * 00     No      CLK Enable
 */
#define I2S_OUT_CFG_REG_UPDATE_MASK   0x3C03FF03  /* set bit = do not modify */

/* Input cfg is same as output, but the FS width is not a valid field */
#define I2S_IN_CFG_REG_UPDATE_MASK  (I2S_OUT_CFG_REG_UPDATE_MASK | 0x03FC0000)

static int update_ssp_cfg(struct cygnus_aio_port *aio)
{
	u32 valid_slots;       /* reg val to program */
	int bits_per_slot_cmn;
	int bits_per_slot_in;
	int bits_per_slot_out;

	u32 ssp_newcfg;
	u32 ssp_curcfg;
	u32 ssp_outcfg;
	u32 ssp_incfg;
	u32 fsync_width;

	/* We encode 16 slots as 0 in the reg */
	valid_slots = aio->active_slots;
	if (aio->active_slots == 16)
		valid_slots = 0;

	/* Slot Width is either 16 or 32 */
	bits_per_slot_cmn = 0;     /* Default to 32 bits */
	if (aio->slot_width <= 16)
		bits_per_slot_cmn = 1;

	bits_per_slot_in = bits_per_slot_cmn;
	bits_per_slot_out = bits_per_slot_cmn;

	if (aio->portnum == SPDIF)
		return -EINVAL;

	ssp_newcfg = 0;

	if (aio->mode == CYGNUS_SSPMODE_TDM) {
		ssp_newcfg |= BIT(I2S_OUT_CFGX_TDM_MODE);
		if (aio->fsync_width == -1)
			fsync_width = 1;
		else
			fsync_width = aio->fsync_width;

		ssp_newcfg |= (fsync_width << I2S_OUT_CFGX_FSYNC_WIDTH);
	}

	if (aio->slave)
		ssp_newcfg |= BIT(I2S_OUT_CFGX_SLAVE_MODE);
	else
		ssp_newcfg &= ~BIT(I2S_OUT_CFGX_SLAVE_MODE);

	if (aio->mode == CYGNUS_SSPMODE_I2S) {
		ssp_newcfg |= BIT(I2S_OUT_CFGX_DATA_ALIGNMENT);
		ssp_newcfg |= BIT(I2S_OUT_CFGX_FSYNC_WIDTH);
	} else {
		if (aio->fs_delay == 0)
			ssp_newcfg &= ~BIT(I2S_OUT_CFGX_DATA_ALIGNMENT);
		else
			ssp_newcfg |= BIT(I2S_OUT_CFGX_DATA_ALIGNMENT);
	}

	if (aio->invert_bclk)
		ssp_newcfg |= BIT(I2S_OUT_CFGX_SCLK_POLARITY);

	if (aio->invert_fs)
		ssp_newcfg |= BIT(I2S_OUT_CFGX_LRCK_POLARITY);

	/*
	 * SSP in cfg.
	 * Retain bits we do not want to update, then OR in new bits
	 * Always set slave mode for Rx formatter.
	 * The Rx formatter's Slave Mode bit controls if it uses its own
	 * internal clock or the clock signal that comes from the Slave Mode
	 * bit set in the Tx formatter (which would be the Tx Formatters
	 * internal clock or signal from external pin).
	 */
	ssp_curcfg = readl(aio->audio + aio->regs.i2s_cap_cfg);
	ssp_incfg = (ssp_curcfg & I2S_IN_CFG_REG_UPDATE_MASK) | ssp_newcfg;
	ssp_incfg |= BIT(I2S_OUT_CFGX_SLAVE_MODE);

	ssp_incfg &= ~(0xF << I2S_OUT_CFGX_VALID_SLOT);
	ssp_incfg |= (valid_slots << I2S_OUT_CFGX_VALID_SLOT);
	ssp_incfg &= ~BIT(I2S_OUT_CFGX_BITS_PER_SLOT);
	ssp_incfg |= (bits_per_slot_in << I2S_OUT_CFGX_BITS_PER_SLOT);

	writel(ssp_incfg, aio->audio + aio->regs.i2s_cap_cfg);

	/*
	 * SSP out cfg.
	 * Retain bits we do not want to update, then OR in new bits
	 */
	ssp_curcfg = readl(aio->audio + aio->regs.i2s_cfg);
	ssp_outcfg = (ssp_curcfg & I2S_OUT_CFG_REG_UPDATE_MASK) | ssp_newcfg;

	ssp_outcfg &= ~(0xF << I2S_OUT_CFGX_VALID_SLOT);
	ssp_outcfg |= (valid_slots << I2S_OUT_CFGX_VALID_SLOT);
	ssp_outcfg &= ~BIT(I2S_OUT_CFGX_BITS_PER_SLOT);
	ssp_outcfg |= (bits_per_slot_out << I2S_OUT_CFGX_BITS_PER_SLOT);

	writel(ssp_outcfg, aio->audio + aio->regs.i2s_cfg);

	return 0;
}

#ifdef CONFIG_PM
static int cygnus_ssp_suspend(struct snd_soc_dai *cpu_dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	audio_ssp_out_disable(aio);
	audio_ssp_in_disable(aio);
	return 0;
}
static int cygnus_ssp_resume(struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define cygnus_ssp_suspend NULL
#define cygnus_ssp_resume  NULL
#endif

static void pll_fract_tweak_set(void __iomem *audio_base, u32 value)
{
	/*
	 * Read ACTIVE PLL registers for current values
	 * Write new values to the USER PLL registers
	 * Transition PLL Control to update the active PLL registers with user
	 * PLL registers
	 */
	u32 ndiv, mdiv0, mdiv1, mdiv2;

	ndiv = readl(audio_base + IOP_PLL_0_ACTIVE_NDIV_OFFSET);
	mdiv0 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch0_OFFSET);
	mdiv1 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch1_OFFSET);
	mdiv2 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch2_OFFSET);

	ndiv &= 0x3ff;
	ndiv |= value << IOP_PLL_0_USER_NDIV_FRAC;

	writel(7, audio_base + IOP_PLL_0_MACRO_OFFSET);
	writel(0, audio_base + IOP_PLL_0_CONTROL_OFFSET);
	writel(mdiv0, audio_base + IOP_PLL_0_MDIV_Ch0_OFFSET);
	writel(mdiv1, audio_base + IOP_PLL_0_MDIV_Ch1_OFFSET);
	writel(mdiv2, audio_base + IOP_PLL_0_MDIV_Ch2_OFFSET);
	writel(ndiv, audio_base + IOP_PLL_0_USER_NDIV_OFFSET);
	writel(1, audio_base + IOP_PLL_0_CONTROL_OFFSET);
}

static u32 pll_fract_tweak_get(void __iomem *audio_base)
{
	u32 ndiv_fract, ndiv_int, value;

	value = readl(audio_base + IOP_PLL_0_USER_NDIV_OFFSET);
	ndiv_fract = value >> IOP_PLL_0_USER_NDIV_FRAC;
	ndiv_int = value & 0x3FF;

	value = readl(audio_base + IOP_PLL_0_ACTIVE_NDIV_OFFSET);
	ndiv_fract = value >> IOP_PLL_0_ACTIVE_NDIV_FRAC;
	ndiv_int = value & 0x3FF;

	return ndiv_fract;
}

/*
 * pll_tweak_get - read the pll fractional setting.
 *   kcontrol: The control for the speaker gain.
 *   ucontrol: The value that needs to be updated.
 */
static int pll_tweak_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	ucontrol->value.integer.value[0] = pll_fract_tweak_get(aio->audio);

	return 0;
}

/*
 * pll_tweak_put - set the pll fractional setting.
 *   kcontrol: The control for the pll tweak.
 *   ucontrol: The value that needs to be set.
 */
static int pll_tweak_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);
	int value;

	value = ucontrol->value.integer.value[0];
	if (value > PLL_NDIV_FRACT_MAX)
		return -EINVAL;

	pll_fract_tweak_set(aio->audio, value);

	return 0;
}

static const struct snd_kcontrol_new pll_tweak_controls[] = {
	SOC_SINGLE_EXT("PLL Tweak", 0, 0, PLL_NDIV_FRACT_MAX, 0,
	pll_tweak_get, pll_tweak_put),
};

int cygnus_ssp_add_pll_tweak_controls(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	return snd_soc_add_dai_controls(cpu_dai,
				pll_tweak_controls,
				ARRAY_SIZE(pll_tweak_controls));
}
EXPORT_SYMBOL_GPL(cygnus_ssp_add_pll_tweak_controls);


static const struct snd_soc_dai_ops cygnus_ssp_dai_ops = {
	.startup	= cygnus_ssp_startup,
	.trigger	= cygnus_ssp_trigger,
	.hw_params	= cygnus_ssp_hw_params,
	.set_fmt	= cygnus_ssp_set_fmt,
	.set_sysclk	= cygnus_ssp_set_sysclk,
	.set_tdm_slot	= cygnus_set_dai_tdm_slot,
};


#define INIT_CPU_DAI(num) { \
	.name = "cygnus-ssp" #num, \
	.playback = { \
		.channels_min = 1, \
		.channels_max = 16, \
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000, \
		.formats = SNDRV_PCM_FMTBIT_S8 | \
				SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S24_LE | \
				SNDRV_PCM_FMTBIT_S32_LE, \
	}, \
	.capture = { \
		.channels_min = 2, \
		.channels_max = 16, \
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_88200 | \
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			SNDRV_PCM_RATE_192000, \
		.formats =  SNDRV_PCM_FMTBIT_S16_LE | \
					SNDRV_PCM_FMTBIT_S24_LE | \
					SNDRV_PCM_FMTBIT_S32_LE, \
	}, \
	.ops = &cygnus_ssp_dai_ops, \
	.suspend = cygnus_ssp_suspend, \
	.resume = cygnus_ssp_resume, \
}

static struct snd_soc_dai_driver cygnus_ssp_dai_info[] = {
	INIT_CPU_DAI(0),
	INIT_CPU_DAI(1),
	INIT_CPU_DAI(2),
};

static struct snd_soc_dai_driver cygnus_spdif_dai_info = {
	.name = "cygnus-spdif",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &cygnus_ssp_dai_ops,
	.suspend = cygnus_ssp_suspend,
	.resume = cygnus_ssp_resume,
};

static struct snd_soc_dai_driver cygnus_ssp_dai[CYGNUS_MAX_PORTS];

static const struct snd_soc_component_driver cygnus_ssp_component = {
	.name		= "cygnus-audio",
};

static const struct of_device_id cygnus_ssp_of_match[] = {
	{ .compatible = "brcm,cygnus-audio" },
	{},
};

/*
 * Return < 0 if error
 * Return 0 if disabled
 * Return 1 if enabled and node is parsed successfully
 */
static int parse_ssp_child_node(struct platform_device *pdev,
				struct device_node *dn,
				struct cygnus_audio *cygaud,
				struct snd_soc_dai_driver *p_dai)
{
	struct cygnus_aio_port *aio;
	const char *channel_grp;
	struct cygnus_ssp_regs ssp_regs[3];
	u32 rawval;
	int portnum = -1;

	if (of_property_read_u32(dn, "reg", &rawval)) {
		pr_err("Could not find reg\n");
		return -EINVAL;
	}

	switch (rawval) {
	case 0:
		portnum = I2S0;
		break;
	case 1:
		portnum = I2S1;
		break;
	case 2:
		portnum = I2S2;
		break;
	case 3:
		portnum = SPDIF;
		break;
	default:
		pr_err("Bad value for reg %u\n", rawval);
		return -EINVAL;
	}

	if (of_property_read_string(dn, "channel-group", &channel_grp) != 0) {
		dev_err(&pdev->dev, "Missing channel_group property\n");
		return -EINVAL;
	}

	aio = &cygaud->portinfo[portnum];

	aio->audio = cygaud->audio;
	aio->portnum = portnum;
	aio->fsync_width = -1;

	aio->gpio_latchctrl = -1;
	aio->fix_multichan_tdm_shift_slave_rx = false;
	aio->fix_multichan_tdm_shift_tx = false;
	aio->enable_i2s_32fs = false;

	if ((portnum == I2S0) || (portnum == I2S1) || (portnum == I2S2)) {
		ssp_regs[I2S0] = (struct cygnus_ssp_regs) INIT_SSP_REGS(0);
		ssp_regs[I2S1] = (struct cygnus_ssp_regs) INIT_SSP_REGS(1);
		ssp_regs[I2S2] = (struct cygnus_ssp_regs) INIT_SSP_REGS(2);

		aio->regs = ssp_regs[portnum];

		*p_dai = cygnus_ssp_dai_info[portnum];
		aio->mode = CYGNUS_SSPMODE_UNKNOWN;

	} else { /* SPDIF case */
		aio->regs.bf_sourcech_cfg = BF_SRC_CFG3_OFFSET;
		aio->regs.bf_sourcech_ctrl = BF_SRC_CTRL3_OFFSET;
		aio->regs.i2s_mclk_cfg = SPDIF_MCLK_CFG_OFFSET;
		aio->regs.i2s_stream_cfg = SPDIF_STREAM_CFG_OFFSET;

		*p_dai = cygnus_spdif_dai_info;

		/* For the purposes of this code SPDIF can be I2S mode */
		aio->mode = CYGNUS_SSPMODE_I2S;
	}

	/* Handle the channel grouping */
	if (portnum == I2S0) {
		if (strstr(channel_grp, "2_0")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "3_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x3;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == I2S1) {
		if (strstr(channel_grp, "2_0")) {
			group_id[portnum] = 1;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "3_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x3;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == I2S2) {
		if (strstr(channel_grp, "2_0")) {
			group_id[I2S2] = 2;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[I2S2] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == SPDIF) {
		group_id[SPDIF] = 3;
		aio->channel_grouping = 0x1;
	}

	dev_dbg(&pdev->dev, "%s portnum = %d\n", __func__, aio->portnum);
	aio->streams_on = 0;
	aio->dev = &pdev->dev;

	audio_ssp_init_portregs(aio);
	return 0;
}

static int cygnus_ssp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child_node;
	struct resource *res = pdev->resource;
	struct cygnus_audio *cygaud;
	int err = -EINVAL;
	int node_count;
	int active_port_count;

	cygaud = devm_kzalloc(dev, sizeof(struct cygnus_audio), GFP_KERNEL);
	if (!cygaud)
		return -ENOMEM;

	dev_set_drvdata(dev, cygaud);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cygaud->audio = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygaud->audio))
		return PTR_ERR(cygaud->audio);

	audio_pll0_init(cygaud->audio);

	/* Tri-state all controlable pins until we know that we need them */
	writel(0x001FFF, cygaud->audio + AUD_MISC_SEROUT_OE_REG_BASE);

	node_count = of_get_child_count(pdev->dev.of_node);
	if ((node_count < 1) || (node_count > CYGNUS_MAX_PORTS)) {
		dev_err(dev, "child nodes is %d.  Must be between 1 and %d\n",
			node_count, CYGNUS_MAX_PORTS);
		return -EINVAL;
	}

	active_port_count = 0;
	for_each_available_child_of_node(pdev->dev.of_node, child_node) {
		err = parse_ssp_child_node(pdev, child_node, cygaud,
					&cygnus_ssp_dai[active_port_count]);

		/* negative is err, 0 is active and good, 1 is disabled */
		if (err < 0)
			return err;
		else if (err == 0) {
			dev_dbg(dev, "Activating DAI: %s\n",
				cygnus_ssp_dai[active_port_count].name);
			active_port_count++;
		}
	}

	cygaud->dev = dev;

	dev_dbg(dev, "Registering %d DAIs\n", active_port_count);
	err = snd_soc_register_component(dev, &cygnus_ssp_component,
				cygnus_ssp_dai, active_port_count);
	if (err) {
		dev_err(dev, "snd_soc_register_dai failed\n");
		return err;
	}

	cygaud->irq_num = platform_get_irq(pdev, 0);
	if (cygaud->irq_num <= 0) {
		dev_err(dev, "platform_get_irq failed\n");
		return cygaud->irq_num;
	}

	err = cygnus_soc_platform_register(dev, cygaud);
	if (err) {
		dev_err(dev, "platform reg error %d\n", err);
		return err;
	}

	return 0;
}

static int cygnus_ssp_remove(struct platform_device *pdev)
{
	cygnus_soc_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static struct platform_driver cygnus_ssp_driver = {
	.probe		= cygnus_ssp_probe,
	.remove		= cygnus_ssp_remove,
	.driver		= {
		.name	= "cygnus-ssp",
		.of_match_table = cygnus_ssp_of_match,
	},
};

module_platform_driver(cygnus_ssp_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Cygnus ASoC SSP Interface");
