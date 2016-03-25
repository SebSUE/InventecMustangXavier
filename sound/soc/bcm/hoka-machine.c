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
 *
 *
 * This is the ASoC machine file for the Hoka boards.
 * It works for the BCM58305 based Hoka board.
 * The board consists of a Cygnus SoC:
 */
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>

#include "cygnus-ssp.h"

#include "../codecs/tlv320aic3x.h"

/* Max string length of our dt property names */
#define PROP_LEN_MAX 40

#define BCMVP_UNUSED_GPIO	(-ENOENT)

#define SVK_SSPMODE_I2S  0
#define SVK_SSPMODE_TDM  1

#define SVK_TDM_SLOT_WIDTH_DEFAULT 256
#define SVK_TDM_FSYNC_WIDTH_DEFAULT 1

#define SVK_BT_LINK_NUM 2

/* Device tree entry names */
#define PROP_NAME_BT_BCLK  "bcm,cygnus-bt-bclk"

struct cygnus_sspcfg_info {
	int clksrc;
	int sspmode;
	int codec_slave_mode;
	int bit_per_frame;
	int fsync_width;
};

enum cygnussvk_dai_fn_code {
	DAI_SET_SYSCLK,
	DAI_SET_FMT,
	DAI_SET_HW,
	DAI_PREPARE,
	CODEC_POWER_MNG,
	DAPM_STREAM_EVENT,
};

static int chnls8_mode = 0;

struct cygnussvk_dai_fn {
	enum cygnussvk_dai_fn_code  fn;	// what we need to do ..
	unsigned int  args[8];  // arguments ..  
};

typedef int (*cygnussvk_codec_fn)(struct snd_soc_codec *c, struct cygnussvk_dai_fn *d);
typedef int (*cygnussvk_component_fn)(struct snd_soc_dai *dai, struct cygnussvk_dai_fn *d);
int snd_soc_for_every_codec(cygnussvk_codec_fn, struct cygnussvk_dai_fn *todo);
int snd_soc_for_every_dai(struct snd_soc_component *component, cygnussvk_component_fn _fn, struct cygnussvk_dai_fn *todo);
int aic3x_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
int aic3x_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level level);
int aic3x_get_bus(struct snd_soc_codec *codec); 


static const struct snd_soc_dapm_route aic3106_audio_map_u18[] = {
	/* Routings for outputs */
	{"Headset Spk-U18", NULL, "HPLOUT-U18"},
	{"Handset Spk-U18", NULL, "HPROUT-U18"},
	{"Handsfree Spk-U18", NULL, "MONO_LOUT-U18"},
	{"External Spk-U18", NULL, "LLOUT-U18"},
	{"External Spk-U18", NULL, "RLOUT-U18"},

	/* Routings for inputs */
	{"LINE1L-U18", NULL, "Mic Bias-U18"},
	{"LINE2L-U18", NULL, "Mic Bias-U18"},
	{"LINE2R-U18", NULL, "Mic Bias-U18"},

	{"Mic Bias-U18", NULL, "Handset Mic-U18"},
	{"Mic Bias-U18", NULL, "Headset Mic-U18"},
	{"Mic Bias-U18", NULL, "Handsfree Mic-U18"},

	{"MIC3R-U18", NULL, "External Mic-U18"},
};
static const struct snd_soc_dapm_route aic3106_audio_map_u19[] = {
	/* Routings for outputs */
	{"Headset Spk-U19", NULL, "HPLOUT-U19"},
	{"Handset Spk-U19", NULL, "HPROUT-U19"},
	{"Handsfree Spk-U19", NULL, "MONO_LOUT-U19"},
	{"External Spk-U19", NULL, "LLOUT-U19"},
	{"External Spk-U19", NULL, "RLOUT-U19"},

	/* Routings for inputs */
	{"LINE1L-U19", NULL, "Mic Bias-U19"},
	{"LINE2L-U19", NULL, "Mic Bias-U19"},
	{"LINE2R", NULL, "Mic Bias"},

	{"Mic Bias-U19", NULL, "Handset Mic-U19"},
	{"Mic Bias-U19", NULL, "Headset Mic-U19"},
	{"Mic Bias-U19", NULL, "Handsfree Mic-U19"},

	{"MIC3R-U19", NULL, "External Mic-U19"},
};
static const struct snd_soc_dapm_route aic3106_audio_map_u1a[] = {
	/* Routings for outputs */
	{"Headset Spk-U1a", NULL, "HPLOUT-U1a"},
	{"Handset Spk-U1a", NULL, "HPROUT-U1a"},
	{"Handsfree Spk-U1a", NULL, "MONO_LOUT-U1a"},
	{"External Spk-U1a", NULL, "LLOUT-U1a"},
	{"External Spk-U1a", NULL, "RLOUT-U1a"},

	/* Routings for inputs */
	{"LINE1L-U1a", NULL, "Mic Bias-U1a"},
	{"LINE2L-U1a", NULL, "Mic Bias-U1a"},
	{"LINE2R-U1a", NULL, "Mic Bias-U1a"},

	{"Mic Bias-U1a", NULL, "Handset Mic-U1a"},
	{"Mic Bias-U1a", NULL, "Headset Mic-U1a"},
	{"Mic Bias-U1a", NULL, "Handsfree Mic-U1a"},

	{"MIC3R-U1a", NULL, "External Mic-U1a"},
};
static const struct snd_soc_dapm_route aic3106_audio_map_u1b[] = {
	/* Routings for outputs */
	{"Headset Spk-U1b", NULL, "HPLOUT-U1b"},
	{"Handset Spk-U1b", NULL, "HPROUT-U1b"},
	{"Handsfree Spk-U1b", NULL, "MONO_LOUT-U1b"},
	{"External Spk-U1b", NULL, "LLOUT-U1b"},
	{"External Spk-U1b", NULL, "RLOUT-U1b"},

	/* Routings for inputs */
	{"LINE1L-U1b", NULL, "Mic Bias-U1b"},
	{"LINE2L-U1b", NULL, "Mic Bias-U1b"},
	{"LINE2R-U1b", NULL, "Mic Bias-U1b"},

	{"Mic Bias-U1b", NULL, "Handset Mic-U1b"},
	{"Mic Bias-U1b", NULL, "Headset Mic-U1b"},
	{"Mic Bias-U1b", NULL, "Handsfree Mic-U1b"},

	{"MIC3R-U1b", NULL, "External Mic-U1b"},
};

static const struct snd_soc_dapm_widget aic3106_dapm_widgets_u18[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk-U18", NULL),
	SND_SOC_DAPM_HP("Handset Spk-U18", NULL),
//	SND_SOC_DAPM_LINE("Handsfree Spk-U18", handsfree_spk_event),
//	SND_SOC_DAPM_SPK("External Spk-U18", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic-U18", NULL),
	SND_SOC_DAPM_MIC("Headset Mic-U18", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic-U18", NULL),
	SND_SOC_DAPM_LINE("External Mic-U18", NULL),   /* 3.5 mm jack */
};
static const struct snd_soc_dapm_widget aic3106_dapm_widgets_u19[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk-U19", NULL),
	SND_SOC_DAPM_HP("Handset Spk-U19", NULL),
//	SND_SOC_DAPM_LINE("Handsfree Spk-U19", handsfree_spk_event),
//	SND_SOC_DAPM_SPK("External Spk-U19", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic-U19", NULL),
	SND_SOC_DAPM_MIC("Headset Mic-U19", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic-U19", NULL),
	SND_SOC_DAPM_LINE("External Mic-U19", NULL),   /* 3.5 mm jack */
};
static const struct snd_soc_dapm_widget aic3106_dapm_widgets_u1a[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk-U1a", NULL),
	SND_SOC_DAPM_HP("Handset Spk-U1a", NULL),
//	SND_SOC_DAPM_LINE("Handsfree Spk-U1a", handsfree_spk_event),
//	SND_SOC_DAPM_SPK("External Spk-U1a", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic-U1a", NULL),
	SND_SOC_DAPM_MIC("Headset Mic-U1a", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic-U1a", NULL),
	SND_SOC_DAPM_LINE("External Mic-U1a", NULL),   /* 3.5 mm jack */
};
static const struct snd_soc_dapm_widget aic3106_dapm_widgets_u1b[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk-U1b", NULL),
	SND_SOC_DAPM_HP("Handset Spk-U1b", NULL),
//	SND_SOC_DAPM_LINE("Handsfree Spk-U1b", handsfree_spk_event),
//	SND_SOC_DAPM_SPK("External Spk-U1b", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic-U1b", NULL),
	SND_SOC_DAPM_MIC("Headset Mic-U1b", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic-U1b", NULL),
	SND_SOC_DAPM_LINE("External Mic-U1b", NULL),   /* 3.5 mm jack */
};

/* digital audio interface glue - connects codec <--> CPU
 * Just use some very generic names for "name" and "stream_name".
 * Ideally we would use more meaningful names, but because we have made
 * this machine module so configurable it make is easier to just use
 * generic names.
 */
static struct snd_soc_dai_link cygnussvk_dai_links[] = {
{
	.name = "cygsvkdev0",
	.stream_name = "cygsvkdev0_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
{
	.name = "cygsvkdev1",
	.stream_name = "cygsvkdev1_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
{
	.name = "cygsvkdev2",
	.stream_name = "cygsvkdev2_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
{
	.name = "cygsvkdev3",
	.stream_name = "cygsvkdev3_stream",
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
},
};

struct cygnussvk_data {
	struct cygnus_sspcfg_info sspcfg_info[CYGNUS_MAX_PLAYBACK_PORTS];
	u32 bt_bclk;
};

int cygnussvk_get_dai_id(struct snd_soc_dai_link *dai_link) 
{
	int idx;

	for (idx=0; idx < 4; idx++) {
//printk("[ADK] %s\t\t[%d]@%p\n", __func__, idx, (void *)(&cygnussvk_dai_links[idx]));
		if ((unsigned int)dai_link == (unsigned int)(&cygnussvk_dai_links[idx])) 
			return idx;
	}

	return (-1);
}

int component_fn(struct snd_soc_dai *dai, struct cygnussvk_dai_fn *todo)
{
//	printk("[ADK]\t%s: dai@%p, dai->name=[%s]\n", __func__, dai, dai->name);

	switch (todo->fn) {
		case DAI_SET_SYSCLK: 
			snd_soc_dai_set_sysclk(dai, CLKIN_MCLK, todo->args[0] /*mclk_freq */, SND_SOC_CLOCK_IN);
			break;
		case DAI_SET_FMT:
			snd_soc_dai_set_fmt(dai, todo->args[0] /*format */);
			break;
		case DAI_SET_HW:
			{
				struct snd_pcm_substream *substream = (struct snd_pcm_substream *)todo->args[0];
				struct snd_pcm_hw_params codec_params = *((struct snd_pcm_hw_params *)todo->args[1]);
				soc_dai_hw_params(substream, &codec_params, dai);
			}
			break;
		case DAI_PREPARE:
			{
				struct snd_pcm_substream *substream = (struct snd_pcm_substream *)todo->args[0];
				aic3x_prepare(substream, dai);
			}
			break;
	}
	return 0;
}

int codec_fn(struct snd_soc_codec *codec, struct cygnussvk_dai_fn *todo)
{
	cygnussvk_component_fn _fn = component_fn;
	int bus;

	if (!strncmp(codec->component.name, "tlv320aic3x", 11)) {
		bus = aic3x_get_bus(codec);
//		printk("[ADK] %s: codec@%p, bus=%d, component.name=[%s], fn=%d\n", __func__, codec, bus, codec->component.name, todo->fn);
		/* exept 0x18 and dummy .. on bus I2C 0 or 1 ..*/
		if (
	     	( bus == 0 && (
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.0-0019") ||
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.0-001a") ||
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.0-001b")))
	     	||	
	     	(bus  == 1 && (
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.1-0019") ||
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.1-001a") ||
	     	!strcmp(codec->component.name, "tlv320aic3x-codec.1-001b")))
	   	)
		{
			if (todo->fn == CODEC_POWER_MNG) {
				aic3x_set_bias_level(codec, todo->args[0]);
			}else	
			if (todo->fn == DAPM_STREAM_EVENT) {
				snd_soc_update_bits(codec, todo->args[0], todo->args[1], todo->args[2]); 	
			}else
				snd_soc_for_every_dai(&codec->component, _fn, todo);
		}
	}
//	else printk("[ADK] %s: component.name=[%s] skipped ..\n", __func__, codec->component.name);
	return 0;
}

int cygnussvk_powerup_aic3x(struct snd_soc_codec *codec, enum snd_soc_bias_level level) 
{

	if (
		!strcmp(codec->component.name, "tlv320aic3x-codec.0-0018") ||
		!strcmp(codec->component.name, "tlv320aic3x-codec.1-0018")
	) 
	{
		if (chnls8_mode) {
			cygnussvk_codec_fn _fn = codec_fn;
			struct cygnussvk_dai_fn todo;
			
			todo.fn = CODEC_POWER_MNG;
			todo.args[0] = level;
			snd_soc_for_every_codec(_fn, &todo);
		}
	}
	return 0;
}

static int cygnussvk_hw_params_spdif(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int mclk_freq = 0;
	int ret = 0, id;

//	dev_dbg(dev, "%s Enter\n", __func__);

	sspcfg = &card_data->sspcfg_info[3];

	switch (params_rate(params)) {
	case  8000:
	case 16000:
		mclk_freq = 6144000;
		break;
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;
	case 192000:
		mclk_freq = 24576000;
		break;
	case 11025:
	case 22050:
	case 44100:
		mclk_freq = 5644800;
		break;
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cygnussvk_ops_spdif = {
	.hw_params = cygnussvk_hw_params_spdif,
};

static int cygnussvk_hw_params_bluetooth(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int channels = 0;
	unsigned int width = 0;
	unsigned int format = 0;
	unsigned int bit_per_frame = 0;
	unsigned int mclk_freq = 12288000;
	int i;
	int slots;
	unsigned int mask = 0;
	int ret = 0;

//	printk("[ADK] Enter %s\n", __func__);
	sspcfg = &card_data->sspcfg_info[SVK_BT_LINK_NUM];

	/*
	 * Bluetooth wants a constant bps, so half the number of bits per
	 * frame when we do wideband (16 kHz)
	 */
	switch (params_rate(params)) {
	case 8000:  /* NBS */
		bit_per_frame = sspcfg->bit_per_frame;
		break;
	case 16000: /* WBS */
		bit_per_frame = sspcfg->bit_per_frame/2;
		break;
	default:
//		dev_err(dev, "%s Unsupported freq %d\n", __func__, params_rate(params));
		printk("[ADK] %s Unsupported freq %d\n", __func__, params_rate(params));
		return -EINVAL;
	}

//	printk("[ADK] %s Frame width = %d\n", __func__, bit_per_frame);

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	/* Set codec for Cygnus SSP for TDM mode master */
//	printk("[ADK] Configure system for TDM transfer\n");
	format = SND_SOC_DAIFMT_CBS_CFS
			| SND_SOC_DAIFMT_DSP_A
			| SND_SOC_DAIFMT_IB_NF;
	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		return ret;
	}

	channels = params_channels(params);
	width = snd_pcm_format_physical_width(params_format(params));

	/* Set a bit for each valid slot */
	mask = 0;
	for (i = 0; i < channels; i++)
		mask |= BIT(i);

	slots = bit_per_frame / width;
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask, slots, width);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_tdm_slot\n", __func__);
		return ret;
	}
//	printk("[ADK] %s set_tdm_slot:  mask 0x%x  slots %d  slot width %d\n", __func__, mask, slots, width);
	return 0;
}

static struct snd_soc_ops cygnussvk_ops_bluetooth = {
	.hw_params = cygnussvk_hw_params_bluetooth,
};

static int cygnussvk_hw_params_testport_common(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, int linknum)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int format = 0;
	unsigned int mclk_freq = 0;
	int ret = 0;

//	printk("[ADK] Enter %s\n", __func__);

	sspcfg = &card_data->sspcfg_info[linknum];

	switch (params_rate(params)) {
	case  8000:
	case 16000:
		mclk_freq = 6144000;
		break;
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;
	case 192000:
		mclk_freq = 24576000;
		break;
	case 11025:
	case 22050:
	case 44100:
		mclk_freq = 5644800;
		break;
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	/* Most systems will never need to do this get_mode, because the port
	 * will only every need to run in one mode.
	 * It is convientent on our verification system to initialize the board
	 * in different modes for verification puposes.
	 */
	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		printk("[ADK] Configure system for TDM transfer\n");
		if (sspcfg->codec_slave_mode == 1) {
			/* Set codec as I2S slave */
			printk("[ADK] %s Set dummy as master.\n", __func__);
			format = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_DSP_B
				| SND_SOC_DAIFMT_NB_NF;
//[ADK]				| SND_SOC_DAIFMT_IB_NF;
		} else {
			printk("[ADK] %s Set dummy as slave.\n", __func__);
			format = SND_SOC_DAIFMT_CBM_CFM
				| SND_SOC_DAIFMT_DSP_B
				| SND_SOC_DAIFMT_IB_NF;
		}

		/* change the frame sync width if it is not default */
		if (sspcfg->fsync_width != SVK_TDM_FSYNC_WIDTH_DEFAULT) {
			ret = cygnus_ssp_set_custom_fsync_width(cpu_dai,
				sspcfg->fsync_width);
			if (ret < 0) {
//				dev_err(dev, "%s Failed cygnus_ssp_set_custom_fsync_width cpu_dai %d\n", __func__, sspcfg->fsync_width);
				printk("[ADK] %s Failed cygnus_ssp_set_custom_fsync_width cpu_dai %d\n", __func__, sspcfg->fsync_width);
				return ret;
			}
		}
	} else {
		printk("[ADK] Configure system for I2S transfer\n");
		if (sspcfg->codec_slave_mode == 1) {
			/* Set codec as I2S slave */
			printk("[ADK] %s Set dummy as master.\n", __func__);
			format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
		} else {
			printk("[ADK] %s Set dummy as slave.\n", __func__);
			format = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
		}
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		return ret;
	}

	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
		unsigned int bit_per_frame = sspcfg->bit_per_frame;
		int slots;
		int i;

		channels = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

		mask = 0;
		for (i = 0; i < channels; i++)
			mask |= BIT(i);

		slots = bit_per_frame / width;

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask,
					slots, width);
		if (ret < 0) {
//			dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n", __func__);
			printk("[ADK] %s Failed snd_soc_dai_set_tdm_slot\n", __func__);
			return ret;
		}
	}
	return 0;
}

static int cygnussvk_hw_params_testport0(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return cygnussvk_hw_params_testport_common(substream, params, 0);
}

static int cygnussvk_hw_params_testport1(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return cygnussvk_hw_params_testport_common(substream, params, 1);
}

static int cygnussvk_hw_params_testport2(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return cygnussvk_hw_params_testport_common(substream, params, 2);
}

static struct snd_soc_ops cygnussvk_ops_testport0 = {
	.hw_params = cygnussvk_hw_params_testport0,
};

static struct snd_soc_ops cygnussvk_ops_testport1 = {
	.hw_params = cygnussvk_hw_params_testport1,
};

static struct snd_soc_ops cygnussvk_ops_testport2 = {
	.hw_params = cygnussvk_hw_params_testport2,
};

int aic3x_get_id(struct snd_soc_codec *codec);

static int cygnussvk_init_aic3106(struct snd_soc_pcm_runtime *rtd)
{

	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	int aic3x_id = aic3x_get_id(codec);
//printk("[ADK] %s entered, id=%d\n", __func__, aic3x_id);

#if 0
	switch(aic3x_id) {
 		case 0:
			snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets_u18,
				ARRAY_SIZE(aic3106_dapm_widgets_u18));

			snd_soc_dapm_add_routes(dapm, aic3106_audio_map_u18,
				ARRAY_SIZE(aic3106_audio_map_u18));
			snd_soc_dapm_nc_pin(dapm, "MIC3L-U18");

		break;
 		case 1:
			snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets_u19,
				ARRAY_SIZE(aic3106_dapm_widgets_u19));

			snd_soc_dapm_add_routes(dapm, aic3106_audio_map_u19,
				ARRAY_SIZE(aic3106_audio_map_u19));
			snd_soc_dapm_nc_pin(dapm, "MIC3L-U19");
		break;
 		case 2:
			snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets_u1a,
				ARRAY_SIZE(aic3106_dapm_widgets_u1a));

			snd_soc_dapm_add_routes(dapm, aic3106_audio_map_u1a,
				ARRAY_SIZE(aic3106_audio_map_u1a));
			snd_soc_dapm_nc_pin(dapm, "MIC3L-U1a");
		break;
 		case 3:
			snd_soc_dapm_new_controls(dapm, aic3106_dapm_widgets_u1b,
				ARRAY_SIZE(aic3106_dapm_widgets_u1b));

			snd_soc_dapm_add_routes(dapm, aic3106_audio_map_u1b,
				ARRAY_SIZE(aic3106_audio_map_u1b));
			snd_soc_dapm_nc_pin(dapm, "MIC3L-U1b");
		break;
	}

//	snd_soc_dapm_nc_pin(dapm, "LINE1R");
//	snd_soc_dapm_nc_pin(dapm, "MIC3L");
#endif //0

	return 0;
}


static int cygnussvk_hw_params_aic3x(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_codec *codec = rtd->codec;
	struct cygnussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int format = 0;
	unsigned int mclk_freq = 0;
	int ret = 0, linknum; 



	linknum = aic3x_get_id(codec);
//	printk("[ADK] %s Entered, dai: id=%d, codec_dai@%p, name=[%s]\n", __func__, linknum, codec_dai, codec_dai->name);

	if (linknum < 0 )
		return -EINVAL;

	sspcfg = &card_data->sspcfg_info[linknum];

//	printk("[ADK] %s Enter, link=%d\n", __func__, linknum);

	switch (params_rate(params)) {
	case  8000:
	case 16000:
		mclk_freq = 6144000;
		break;
	case 32000:
	case 48000:
		mclk_freq = 12288000;
		break;
	case 96000:
	case 192000:
		mclk_freq = 24576000;
		break;
	case 11025:
	case 22050:
	case 44100:
		mclk_freq = 5644800;
		break;
	case  88200:
	case 176400:
		mclk_freq = 22579200;
		break;
	default:
		return -EINVAL;
	}

	// ret = snd_soc_dai_set_sysclk(codec_dai, /* [ADK] CLKIN_MCLK */CLKIN_GPIO2, mclk_freq, SND_SOC_CLOCK_IN);
	if ( linknum == 0) {
		ret = snd_soc_dai_set_sysclk(codec_dai, /* [ADK] CLKIN_MCLK */CLKIN_GPIO2, 	mclk_freq, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			printk("[ADK] %s/%d: Failed snd_soc_dai_set_sysclk\n", __func__, __LINE__);
			return ret;
		}
		if (params_channels(params) == 8) {
			cygnussvk_codec_fn _fn = codec_fn;
			struct cygnussvk_dai_fn todo;
			
			todo.fn = DAI_SET_SYSCLK;
			todo.args[0] = mclk_freq;
			snd_soc_for_every_codec(_fn, &todo);
		}
	}else
		ret = snd_soc_dai_set_sysclk(codec_dai, CLKIN_MCLK, mclk_freq, SND_SOC_CLOCK_IN);

	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, CYGNUS_SSP_CLKSRC_PLL,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_sysclk\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

#if 0
[ADK]
	/* Set codec as I2S slave */
	printk("[ADK] %s Set AIC3106 as slave.\n", __func__);
	format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
#endif
	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
//		printk("[ADK] %s: Configure system for TDM transfer\n", __func__);
		if (sspcfg->codec_slave_mode == 1) {
			/* Set codec as I2S slave */
//			printk("[ADK] %s Set SSP as master.\n", __func__);
			format = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_DSP_B
				| SND_SOC_DAIFMT_IB_NF;
// ---> I try that, but it fails later ..				| SND_SOC_DAIFMT_NB_NF;
		} else {
			printk("[ADK] %s Set SSP as slave.\n", __func__);
			format = SND_SOC_DAIFMT_CBM_CFM
				| SND_SOC_DAIFMT_DSP_B
				| SND_SOC_DAIFMT_IB_NF;
		}

		/* change the frame sync width if it is not default */
		if (sspcfg->fsync_width != SVK_TDM_FSYNC_WIDTH_DEFAULT) {
			ret = cygnus_ssp_set_custom_fsync_width(cpu_dai,
				sspcfg->fsync_width);
			if (ret < 0) {
//				dev_err(dev, "%s Failed cygnus_ssp_set_custom_fsync_width cpu_dai %d\n", __func__, sspcfg->fsync_width);
				printk("[ADK] %s Failed cygnus_ssp_set_custom_fsync_width cpu_dai %d\n", __func__, sspcfg->fsync_width);
				return ret;
			}
		}
	} else {
//		printk("[ADK] %s: Configure system for I2S transfer\n", __func__);
		if (sspcfg->codec_slave_mode == 1) {
			/* Set codec as I2S slave */
//			printk("[ADK] %s Set dummy as master.\n", __func__);
			format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
		} else {
//			printk("[ADK] %s Set dummy as slave.\n", __func__);
			format = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
		}
	}

	ret = snd_soc_dai_set_fmt(codec_dai, format);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_fmt codec_dai\n", 	__func__);
		printk("[ADK] %s Failed snd_soc_dai_set_fmt codec_dai\n", 	__func__);
		return ret;
	}

	if (linknum == 0) {
		if (params_channels(params) == 8) {
			cygnussvk_codec_fn _fn = codec_fn;
			struct cygnussvk_dai_fn todo;
			
			todo.fn = DAI_SET_FMT;
			todo.args[0] = format;
			snd_soc_for_every_codec(_fn, &todo);
		}
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
//		dev_err(dev, "%s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		return ret;
	}

	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
		unsigned int bit_per_frame = sspcfg->bit_per_frame;
		int slots;
		int i;

		channels = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

//		printk("[ADK]\t%s: dai->name=[%s]\n", __func__, codec_dai->name);
//		printk("[ADK] %s TDM: bits per frame=%d, channels=%d, width=%d\n", __func__, bit_per_frame, channels, width);

		mask = 0;
		for (i = 0; i < channels; i++)
			mask |= BIT(i);

/* [ADK] */		mask <<= (linknum*2);  // set a different TDM offset for TLV320AIC3x parts
		slots = bit_per_frame / width;

//		printk("[ADK] %s TDM: slots=%d, mask=0x%x\n", __func__, slots, mask);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask,
					slots, width);
		if (ret < 0) {
//			dev_err(dev, "%s Failed snd_soc_dai_set_tdm_slot\n", __func__);
			printk("[ADK] %s Failed snd_soc_dai_set_tdm_slot\n", __func__);
			return ret;
		}
	}
//	printk("[ADK] %s finished OK for [%s]\n", __func__, codec_dai->name);
	return 0;
}

static int cygnussvk_startup_aic3x(struct snd_pcm_substream *substream)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
//	struct cygnussvk_data *card_data = snd_soc_card_get_drvdata(soc_card);

//	printk("[ADK] %s Enter, chnls=%d\n", __func__, substream->runtime->channels);
	chnls8_mode = 0;
	if (substream->runtime->channels == 0)  chnls8_mode = 1;


#if 0
[ADK]
	/* Set the GPIO multiplex to select appropriate daughter card */
	if (card_data->ti3106cfg.smartcardslot == 0) {
		if (BCMVP_UNUSED_GPIO != card_data->ti3106cfg.gpio_bank_sel0)
			gpio_set_value(card_data->ti3106cfg.gpio_bank_sel0, 0);
		if (BCMVP_UNUSED_GPIO != card_data->ti3106cfg.gpio_bank_sel1)
			gpio_set_value(card_data->ti3106cfg.gpio_bank_sel1, 1);
	} else {
		if (BCMVP_UNUSED_GPIO != card_data->ti3106cfg.gpio_bank_sel0)
			gpio_set_value(card_data->ti3106cfg.gpio_bank_sel0, 1);
		if (BCMVP_UNUSED_GPIO != card_data->ti3106cfg.gpio_bank_sel1)
			gpio_set_value(card_data->ti3106cfg.gpio_bank_sel1, 0);
	}
#endif 	
	return 0;
}

static struct snd_soc_ops cygnussvk_ops_aic3x = {
	.startup = cygnussvk_startup_aic3x,
	.hw_params = cygnussvk_hw_params_aic3x,
};



/* Audio machine driver */
static struct snd_soc_card cygnussvk_card = {
	.name = "bcm-cygnus-hoka",
	.owner = THIS_MODULE,
	.dai_link = cygnussvk_dai_links,
	.num_links = ARRAY_SIZE(cygnussvk_dai_links),
};

static int get_dt_info(struct platform_device *pdev,
	struct cygnussvk_data *card_data, int linknum)
{
	struct device_node *np = pdev->dev.of_node;
	char prop_name[PROP_LEN_MAX];
	int ret = 0;
	const char *clk;
	struct snd_soc_dai_link *dai_link;
	const char *port_mode;
	struct cygnus_sspcfg_info *sspcfg;

	dai_link = &cygnussvk_dai_links[linknum];
	sspcfg = &card_data->sspcfg_info[linknum];

	/* Is there a codec attached to this link?
	 * If not use a dummy codec.
	 */
	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-codec", linknum);
	dai_link->codec_of_node = of_parse_phandle(np, prop_name, 0);
	if (dai_link->codec_of_node == NULL) {
		if (linknum == 0) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &cygnussvk_ops_testport0;
		} else if (linknum == 1) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &cygnussvk_ops_testport1;
		} else if (linknum == 2) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &cygnussvk_ops_testport1;
		} else if (linknum == 3) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &cygnussvk_ops_testport1;
		}
	} else {
/* --------[ADK]
		dev_err(&pdev->dev, "These links should be dummy for now\n");
		goto err_exit;
----- */
//printk("[ADK] %s linknum=%d\n", __func__, linknum);
		if (linknum == 0) {
			dai_link->codec_dai_name = "tlv320aic3x-hifi";
			dai_link->ops = &cygnussvk_ops_aic3x;
			dai_link->init = cygnussvk_init_aic3106;
		} else if (linknum == 1) {
			dai_link->codec_dai_name = "tlv320aic3x-hifi";
			dai_link->ops = &cygnussvk_ops_aic3x;
			dai_link->init = cygnussvk_init_aic3106;
		} else if (linknum == 2) {
			dai_link->codec_dai_name = "tlv320aic3x-hifi";
			dai_link->ops = &cygnussvk_ops_aic3x;
			dai_link->init = cygnussvk_init_aic3106;
		} else if (linknum == 3) {
			dai_link->codec_dai_name = "tlv320aic3x-hifi";
			dai_link->ops = &cygnussvk_ops_aic3x;
			dai_link->init = cygnussvk_init_aic3106;
		}

	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-slave", linknum);
	if (of_property_read_u32(np, prop_name, &sspcfg->codec_slave_mode)) {
		dev_dbg(&pdev->dev,
			"%s property not set. Set default (codec is slave)\n",
			prop_name);
		sspcfg->codec_slave_mode = 1;
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-ssp", linknum);
	if (of_property_read_string(np, prop_name,
					&dai_link->cpu_dai_name) != 0) {
		dev_err(&pdev->dev, "Could not find %s", prop_name);
		ret = -EINVAL;
		goto err_exit;
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-clk", linknum);
	if (of_property_read_string(np, prop_name, &clk) != 0) {
		dev_dbg(&pdev->dev,
			"%s property not set. Set default (pll)\n", prop_name);
		sspcfg->clksrc = CYGNUS_SSP_CLKSRC_PLL;
	} else {
		 if (strstr(clk, "pll"))
			sspcfg->clksrc = CYGNUS_SSP_CLKSRC_PLL;
		else if (strstr(clk, "nco_0"))
			sspcfg->clksrc = CYGNUS_SSP_CLKSRC_NCO_0;
		else if (strstr(clk, "nco_1"))
			sspcfg->clksrc = CYGNUS_SSP_CLKSRC_NCO_1;
		else {
			dev_err(&pdev->dev, "Invalid value for %s", prop_name);
			ret = -EINVAL;
			goto err_exit;
		}
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-mode", linknum);
	if (of_property_read_string(np, prop_name, &port_mode) != 0) {
		dev_dbg(&pdev->dev,
			"%s property not set. Set default (i2s)\n", prop_name);
		sspcfg->sspmode = SVK_SSPMODE_I2S;
	} else {
		if (strstr(port_mode, "i2s"))
			sspcfg->sspmode = SVK_SSPMODE_I2S;
		else if (strstr(port_mode, "tdm"))
			sspcfg->sspmode = SVK_SSPMODE_TDM;
		else {
			dev_err(&pdev->dev, "Invalid value for %s", prop_name);
			ret = -EINVAL;
			goto err_exit;
		}
	}
#if 0
[ADK]
	if (linknum == SVK_BT_LINK_NUM) {
		/* switch the op for bluetooth depending
		   on i2s or pcm is used */
		if (sspcfg->sspmode == SVK_SSPMODE_I2S) {
				dev_dbg(&pdev->dev,
					"Switch bluetooth link to use i2s op\n");
				dai_link->ops = &cygnussvk_ops_testport2;
		} else
				dai_link->ops = &cygnussvk_ops_bluetooth;
	}
#endif
	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-frame-width",
		linknum);
	if (of_property_read_u32(np, prop_name, &sspcfg->bit_per_frame)) {
		dev_dbg(&pdev->dev,
			"%s property not set. Set default (%d)\n",
			prop_name, SVK_TDM_SLOT_WIDTH_DEFAULT);
		sspcfg->bit_per_frame = SVK_TDM_SLOT_WIDTH_DEFAULT;
	}

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-fsync-width",
		linknum);
	if (of_property_read_u32(np, prop_name, &sspcfg->fsync_width)) {
		dev_dbg(&pdev->dev,
			"%s property not set. Set default (%d)\n",
			prop_name, SVK_TDM_FSYNC_WIDTH_DEFAULT);
		sspcfg->fsync_width = SVK_TDM_FSYNC_WIDTH_DEFAULT;
	}

	/* Ensure sync width is smaller than frame width */
	if (sspcfg->fsync_width >= sspcfg->bit_per_frame) {
			dev_err(&pdev->dev, "sync (%d) is larger than frame (%d)\n",
				sspcfg->fsync_width, sspcfg->bit_per_frame);
			ret = -EINVAL;
			goto err_exit;
	}

err_exit:
	return ret;
}

static int cygnussvk_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &cygnussvk_card;
	struct cygnussvk_data *card_data;
	const struct device_node *pofn;
	int ret = 0;

	dev_dbg(&pdev->dev, "Enter %s\n", __func__);

	card->dev = &pdev->dev;

	card_data = devm_kzalloc(&pdev->dev, sizeof(*card_data), GFP_KERNEL);
	if (card_data == NULL)
		return -ENOMEM;

	/* Get codec and ssp info for Link 0 */
	ret = get_dt_info(pdev, card_data, 0);
	if (ret < 0)
		goto err_exit;

	/* Get codec and ssp info for Link 1 */
	ret = get_dt_info(pdev, card_data, 1);
	if (ret < 0)
		goto err_exit;

	/* Get codec and ssp info for Link 2 */
	ret = get_dt_info(pdev, card_data, 2);
	if (ret < 0)
		goto err_exit;

	/* Get codec and ssp info for fourth link */
	ret = get_dt_info(pdev, card_data, 3);
	if (ret < 0)
		goto err_exit;

	/* Use the same platform driver for both links */
	pofn = of_parse_phandle(np, "bcm,cygnus-pcm", 0);
	if (pofn == NULL) {
		dev_err(&pdev->dev,
			"Property bcm,cygnus-pcm missing or invalid\n");
		ret = -EINVAL;
		goto err_exit;
	}

	/* Use the same platform driver for both links */
	cygnussvk_dai_links[0].platform_of_node = pofn;
	cygnussvk_dai_links[1].platform_of_node = pofn;
	cygnussvk_dai_links[2].platform_of_node = pofn;
	cygnussvk_dai_links[3].platform_of_node = pofn;

	snd_soc_card_set_drvdata(card, card_data);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		goto err_exit;
	}

err_exit:
	return ret;
}

static int cygnussvk_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s Enter\n", __func__);

	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id cygnus_mach_of_match[] = {
	{.compatible = "bcm,cygnus-hoka-machine", },
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_mach_of_match);

static struct platform_driver bcm_cygnus_driver = {
	.driver = {
		.name = "bcm-cygnus-hoka-machine",
		.of_match_table = cygnus_mach_of_match,
	},
	.probe  = cygnussvk_probe,
	.remove = cygnussvk_remove,
};

module_platform_driver(bcm_cygnus_driver);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("ALSA SoC for Cygnus APs");
MODULE_LICENSE("GPL v2");
