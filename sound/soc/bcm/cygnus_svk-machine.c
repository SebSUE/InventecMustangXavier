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
 * This is the ASoC machine file for the Cygnus SVK boards.
 * It works for both the 958300k ("Combo board"), and the 911360k board.
 * The board consists of a Cygnus SoC and a TI TLV320AIC3106 codec mounted on
 * card slot 0.  There is also an onboard WM8750 codec.
 */
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/list.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>

#include "cygnus-ssp.h"

#include "../codecs/tlv320aic3x.h"
#include "../codecs/wm8750.h"

/* Max string length of our dt property names */
#define PROP_LEN_MAX 40

#define BCMVP_UNUSED_GPIO	(-ENOENT)

#define SVK_SSPMODE_I2S  0
#define SVK_SSPMODE_TDM  1

struct cygnus_sspcfg_info {
	int clksrc;
	int sspmode;
	int codec_slave_mode;
};


struct cygnussvk_ti3106_daughtercard_info {
	int gpio_bank_sel0;
	int gpio_bank_sel1;
	int gpio_ext_headset_amp_en;
	int gpio_handsfree_amp_en;
	int smartcardslot;
};

struct cygnussvk_data {

	struct cygnussvk_ti3106_daughtercard_info  ti3106cfg;
	struct cygnus_sspcfg_info sspcfg_info[CYGNUS_MAX_PLAYBACK_PORTS];
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

/* [ADK]
	.name = "SPDIF",
	.stream_name = "SPDIF",
	.codec_dai_name = "snd-soc-dummy-dai",
	.codec_name = "snd-soc-dummy",

	.ops = &cygnussvk_ops_spdif,
	.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
*/	
},
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
	printk("[ADK]\t%s: dai@%p, dai->name=[%s]\n", __func__, dai, dai->name);

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

	printk("[ADK] %s: codec@%p, component.name=[%s], fn=%d\n", __func__, codec, codec->component.name, todo->fn);
	/* exept 0x18 and dummy ..*/
	if (!strcmp(codec->component.name, "tlv320aic3x-codec.0-0019") ||
	     !strcmp(codec->component.name, "tlv320aic3x-codec.0-001a") ||
	     !strcmp(codec->component.name, "tlv320aic3x-codec.0-001b")	)
		{
			if (todo->fn == CODEC_POWER_MNG) {
				aic3x_set_bias_level(codec, todo->args[0]);
			}else	
			if (todo->fn == DAPM_STREAM_EVENT) {
				snd_soc_update_bits(codec, todo->args[0], todo->args[1], todo->args[2]); 	
			}else
				snd_soc_for_every_dai(&codec->component, _fn, todo);
		}
	return 0;
}

int cygnussvk_powerup_aic3x(struct snd_soc_codec *codec, enum snd_soc_bias_level level) 
{

	if (!strcmp(codec->component.name, "tlv320aic3x-codec.0-0018")) {
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
	
static int cygnussvk_hw_params_aic3x(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int format = 0;
	unsigned int mclk_freq = 0;
	int ret = 0, id;

	id = cygnussvk_get_dai_id(rtd->dai_link); 
//	dev_dbg(dev, "%s Enter\n", __func__);
	printk("[ADK] %s Entered, dai: id=%d, codec_dai@%p, name=[%s]\n", __func__, id, codec_dai, codec_dai->name);

	if (id < 0 )
		return -EINVAL;

	sspcfg = &card_data->sspcfg_info[ id];

	switch (params_rate(params)) {
	case  8000:
/* [ADK]	case 16000: */
		mclk_freq = 6144000;
		break;
	case 16000:
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000; 
/* [ADK] 	mclk_freq = 24576000;	  */
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

	if ( id == 0) {
		ret = snd_soc_dai_set_sysclk(codec_dai, /* [ADK] CLKIN_MCLK */CLKIN_GPIO2,
				mclk_freq, SND_SOC_CLOCK_IN);
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
		printk("[ADK] %s/%d: Failed snd_soc_dai_set_sysclk\n", __func__, __LINE__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc, mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	if (sspcfg->codec_slave_mode == 1) {
		/* Set codec as a slave */
		printk("[ADK] %s Set AIC3x as slave.\n", __func__);
		if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
			printk("[ADK] %s/%d: Configure system for TDM transfer\n", __func__, __LINE__);
/* [ADK] for 3.10.70-10.1  cygnus-ssp driver ..
			format = SND_SOC_DAIFMT_CBS_CFS
				| SND_SOC_DAIFMT_DSP_B
				| SND_SOC_DAIFMT_IB_NF;
------------------------------------- */
			format = SND_SOC_DAIFMT_CBS_CFM
				| SND_SOC_DAIFMT_DSP_A
				| SND_SOC_DAIFMT_NB_NF;
		} else {
			printk("[ADK] %s/%d: Configure system for I2S transfer\n", __func__, __LINE__);
			format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
		}
	} else {
		printk("[ADK] %s Set AIC3x as master.\n", __func__);
		format = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, format);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_fmt codec_dai\n", __func__);
		return ret;
	}
	if (id == 0) {
		if (params_channels(params) == 8) {
			cygnussvk_codec_fn _fn = codec_fn;
			struct cygnussvk_dai_fn todo;
			
			todo.fn = DAI_SET_FMT;
			todo.args[0] = format;
			snd_soc_for_every_codec(_fn, &todo);
		}
	}
	
	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		ret = snd_soc_dai_set_fmt(cpu_dai, format);
		if (ret < 0) {
			printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
			return ret;
		}
	}
	
	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
//		unsigned int bit_per_frame = 256;
		unsigned int bit_per_frame = 128;
		int slots;
		int i;

		channels = params_channels(params);
		width = snd_pcm_format_physical_width(params_format(params));

		printk("[ADK] %s TDM: channels=%d, width=%d\n", __func__, channels, width);

		mask = 0;
		for (i = 0; i < channels; i++)
			mask |= BIT(i);
		
/* [ADK] */		mask <<= (id *2);  // set a different TDM offset for TLV320AIC3x parts
		slots = bit_per_frame / width;

		printk("[ADK] %s TDM: slots=%d, mask=0x%08x\n", __func__, slots, mask);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask, slots, width);
		if (ret < 0) {
			printk("[ADK] %s Failed snd_soc_dai_set_tdm_slot\n", __func__);
			return ret;
		}
	}else{

		ret = snd_soc_dai_set_fmt(cpu_dai, format);
		if (ret < 0) {
			printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
			return ret;
		}
	}
	printk("[ADK] %s finished\n", __func__);
	return 0;
}

static int cygnussvk_startup_aic3x(struct snd_pcm_substream *substream)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
//	struct cygnussvk_data *card_data = 	snd_soc_card_get_drvdata(soc_card);

	printk("[ADK] %s Enter, chnls=%d\n", __func__, substream->runtime->channels);
	chnls8_mode = 0;
	if (substream->runtime->channels == 0)  chnls8_mode = 1;
#if 0
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

static int handsfree_spk_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);

	/* Set the gpio to enable or disable the op amp that will drive
	 * the handsfree jack
	 */
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(card_data->ti3106cfg.gpio_handsfree_amp_en, 1);
	else
		gpio_set_value(card_data->ti3106cfg.gpio_handsfree_amp_en, 0);

	return 0;
}

static int ext_spk_evt_aic3x(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm_context = w->dapm;
	struct snd_soc_card *soc_card = dapm_context->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);

	/* Set the gpio to enable or disable the op amp that will drive
	 * the 3.5mm stereo jack
	 */
	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value(card_data->ti3106cfg.gpio_ext_headset_amp_en, 1);
	else
		gpio_set_value(card_data->ti3106cfg.gpio_ext_headset_amp_en, 0);

	return 0;
}


static int cygnussvk_hw_params_wm8750(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
//	struct device *dev = substream->pcm->dev;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *soc_card = rtd->card;
	struct cygnussvk_data *card_data =
		snd_soc_card_get_drvdata(soc_card);
	struct cygnus_sspcfg_info *sspcfg;
	unsigned int format = 0;
	int ret = 0;
	unsigned int mclk_freq = 0;

	printk("[ADK] Enter %s\n", __func__);

	sspcfg = &card_data->sspcfg_info[2];

	switch (params_rate(params)) {
	/* These will not work. Cygnus and WM8750 do not have a common MCLK */
	case 8000:
	case 11025:
		printk("[ADK] %s: %d Hz will not work because there is not a frequency suitable for both Cygnus and the WM8750.\n",
			__func__, params_rate(params));
		return -EINVAL;

	case 16000:
	case 32000:
	case 48000:
	case 96000:
		mclk_freq = 12288000;
		break;

	case 22050:
	case 44100:
	case 88200:
		mclk_freq = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8750_SYSCLK,
				mclk_freq, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, sspcfg->clksrc,
				mclk_freq, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	if (sspcfg->codec_slave_mode == 1) {
		/* Set codec as I2S slave */
		printk("[ADK] %s Set WM8750 as slave.\n", __func__); 
		format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
	} else {
		printk("[ADK] %s Set WM8750 as master.\n", __func__);
		format = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, format);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_fmt codec_dai\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cygnussvk_ops_wm8750 = {
	.hw_params = cygnussvk_hw_params_wm8750,
};

/* Machine DAPM */
static const struct snd_soc_dapm_widget aic3106_dapm_widgets[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk", NULL),
	SND_SOC_DAPM_HP("Handset Spk", NULL),
	SND_SOC_DAPM_LINE("Handsfree Spk", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic", NULL),
	SND_SOC_DAPM_LINE("External Mic", NULL),   /* 3.5 mm jack */
};

static const struct snd_soc_dapm_widget aic3106_dapm_widgets_u18[] = {
	/* Outputs */
	SND_SOC_DAPM_HP("Headset Spk-U18", NULL),
	SND_SOC_DAPM_HP("Handset Spk-U18", NULL),
	SND_SOC_DAPM_LINE("Handsfree Spk-U18", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk-U18", ext_spk_evt_aic3x), /* 3.5 mm jack */

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
	SND_SOC_DAPM_LINE("Handsfree Spk-U19", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk-U19", ext_spk_evt_aic3x), /* 3.5 mm jack */

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
	SND_SOC_DAPM_LINE("Handsfree Spk-U1a", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk-U1a", ext_spk_evt_aic3x), /* 3.5 mm jack */

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
	SND_SOC_DAPM_LINE("Handsfree Spk-U1b", handsfree_spk_event),
	SND_SOC_DAPM_SPK("External Spk-U1b", ext_spk_evt_aic3x), /* 3.5 mm jack */

	/* Inputs */
	SND_SOC_DAPM_MIC("Handset Mic-U1b", NULL),
	SND_SOC_DAPM_MIC("Headset Mic-U1b", NULL),
	SND_SOC_DAPM_MIC("Handsfree Mic-U1b", NULL),
	SND_SOC_DAPM_LINE("External Mic-U1b", NULL),   /* 3.5 mm jack */
};

static const struct snd_soc_dapm_widget wm8750_dapm_widgets[] = {

	SND_SOC_DAPM_SPK("Wolfson Headphone", NULL),
	SND_SOC_DAPM_MIC("Wolfson Mic", NULL),
};

/* machine audio map (connections to the codec pins on tlv320aic3x)
 *============================================================================
 * Outputs
 * Codec Pin	Schematic Name		Physical connector
 * ------------------------------------------------------
 * HPLOUT	MONO_HEADSET_SPK_P      (J501) Headset RJ11 Jack
 * HPLCOM	MONO_HEADSET_SPK_N
 * -------------------------------------------------------------------------
 * HPROUT	HANDSET_SPK_N		(J403) Handset RJ11 Jack
 * HPRCOM	HANDSET_SPK_P
 * -------------------------------------------------------------------------
 * MONO_LOP	HANDFREE_SPK_N		(J401) Handsfree Speaker
 * MONO_LOM	HANDFREE_SPK_P		via AMP (U401) (needs enable via GPIO)
 * -------------------------------------------------------------------------
 * LEFT_LOP	STEREO_HEADSET_SPKL_P
 * LEFT_LOM	STEREO_HEADSET_SPKL_N
 * ----------------------------------	(J502) 3.5mm Stereo Headset Jack
 * RIGHT_LOP	STEREO_HEADSET_SPKR_P	via AMP (U501) (needs enable via GPIO)
 * RIGHT_LOM	STEREO_HEADSET_SPKR_N
 *
 *============================================================================
 * Inputs
 * Codec Pin	Schematic Name		Physical connector
 * ------------------------------------------------------
 * LINE1LP	HNDFREE_MIC_P		(J402) Currently unpopulate on Voip DM
 * LINE1LM	HNDFREE_MIC_N
 * ------------------------------------------------------
 * LINE1RP	No connect
 * LINE1RM	No connect
 * ------------------------------------------------------
 * LINE2LP	HANDSET_MIC_P		(J403) Handset RJ11 Jack
 * LINE2LM	HANDSET_MIC_N
 * ------------------------------------------------------
 * LINE2RP	MONO_HEADSET_MIC_P	(J501) Headset RJ11 Jack
 * LINE2RM	MONO_HEADSET_MIC_N
 * ------------------------------------------------------
 * MIC3L	No connect
 * MIC3R	STEREO_HEADSET_MIC_P	(J502) 3.5mm Stereo Headset Jack
 *
 * Daughter module connector
 *	Pin 10: External (3.5mm jack) Headset AMP enable
 *	Pin 12: Codec Reset
 *	Pin 14: Handsfree AMP enable
 *	Pin 24  MCLK
 *	Pin 26  SDIN  (from codec to cygnus)
 *	Pin 28  SDOUT (from cygnus to codec)
 *	Pin 30  WS
 *	Pin 32  BITCLK
 *
 * GPIO information
 * Three GPIOs from Cygnus are routed to the codec.  The GPIOs pass through
 * a multiplexer (IDTQS3253), allowing them to be used for other devices.
 * Another 2 GPIOs (AON_GPIO[0,1] from Cygnus are used to set the multiplexor.
 *
 * On Combo SVK (958300k)
 * ---------------------
 * "Smartcard 0" (J2398) - uses Cygnus' i2s0
 * Pin 10 - GPIO 0  }
 * Pin 12 - GPIO 1  } via IDTQS3253 mulitpx (AON_GPIO1=1 AON_GPIO0=0) SC0
 * Pin 14 - GPIO 2  }
 *
 * "Smartcard 1" (J2399) - uses Cygnus' i2s1
 * Pin 10 - GPIO 0  }
 * Pin 12 - GPIO 1  } via IDTQS3253 mulitpx (AON_GPIO1=0 AON_GPIO0=1) SC1
 * Pin 14 - GPIO 2  }
 *
 * On Voip SVK (911360k)
 * ---------------------
 * "Smartcard 0" (J2398) - uses Cygnus' i2s1 (mislabeled on schematic)
 * Pin 10 - GPIO 5  }
 * Pin 12 - GPIO 6  } via IDTQS3253 mulitpx (AON_GPIO1=1 AON_GPIO0=0) SC0
 * Pin 14 - GPIO 7  }
 *
 * "Smartcard 1" (J2399) - Does not work for codec
 */
static const struct snd_soc_dapm_route aic3106_audio_map[] = {
	/* Routings for outputs */
	{"Headset Spk", NULL, "HPLOUT"},
	{"Handset Spk", NULL, "HPROUT"},
	{"Handsfree Spk", NULL, "MONO_LOUT"},
	{"External Spk", NULL, "LLOUT"},
	{"External Spk", NULL, "RLOUT"},

	/* Routings for inputs */
	{"LINE1L", NULL, "Mic Bias"},
	{"LINE2L", NULL, "Mic Bias"},
	{"LINE2R", NULL, "Mic Bias"},

	{"Mic Bias", NULL, "Handset Mic"},
	{"Mic Bias", NULL, "Headset Mic"},
	{"Mic Bias", NULL, "Handsfree Mic"},

	{"MIC3R", NULL, "External Mic"},
};

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

/* Routings for WM8750 */
static const struct snd_soc_dapm_route wm8750_audio_map[] = {
	{"Wolfson Headphone", NULL, "LOUT1"},
	{"Wolfson Headphone", NULL, "ROUT1"},

	{"LINPUT1", NULL, "Mic Bias"},
	{"RINPUT1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Wolfson Mic"},
};

int aic3x_get_id(struct snd_soc_codec *codec);

static int cygnussvk_init_aic3106(struct snd_soc_pcm_runtime *rtd)
{

	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	int aic3x_id = aic3x_get_id(codec);
printk("[ADK] %s entered\n", __func__);

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

static int cygnussvk_init_wm8750(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_dapm_new_controls(dapm, wm8750_dapm_widgets,
				ARRAY_SIZE(wm8750_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, wm8750_audio_map,
				ARRAY_SIZE(wm8750_audio_map));

	/* For testing and as an example add in the ssp kcontrols that allow
	 * us to tweak the pll.  This could be done on any port, but this
	 * port has a convenient header to observe the signals.
	 */
	cygnus_ssp_add_pll_tweak_controls(rtd);

	snd_soc_dapm_nc_pin(dapm, "LOUT2");
	snd_soc_dapm_nc_pin(dapm, "ROUT2");

	snd_soc_dapm_nc_pin(dapm, "OUT3");
	snd_soc_dapm_nc_pin(dapm, "MONO1");

	snd_soc_dapm_nc_pin(dapm, "LINPUT2");
	snd_soc_dapm_nc_pin(dapm, "RINPUT2");
	snd_soc_dapm_nc_pin(dapm, "LINPUT3");
	snd_soc_dapm_nc_pin(dapm, "RINPUT3");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "LINPUT1");
	snd_soc_dapm_enable_pin(dapm, "RINPUT1");
	snd_soc_dapm_enable_pin(dapm, "LOUT1");
	snd_soc_dapm_enable_pin(dapm, "ROUT1");

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
	int ret = 0;

	printk("[ADK] Enter %s\n", __func__);

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
		printk("[ADK] %s Failed snd_soc_dai_set_sysclk\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops cygnussvk_ops_spdif = {
	.hw_params = cygnussvk_hw_params_spdif,
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

	printk("[ADK] Enter %s\n", __func__);

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
		format = SND_SOC_DAIFMT_CBS_CFS
			| SND_SOC_DAIFMT_DSP_B
			| SND_SOC_DAIFMT_IB_NF;
	} else {
		printk("[ADK] Configure system for I2S transfer\n");
		format = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, format);
	if (ret < 0) {
		printk("[ADK] %s Failed snd_soc_dai_set_fmt cpu_dai\n", __func__);
		return ret;
	}

	if (sspcfg->sspmode == SVK_SSPMODE_TDM) {
		unsigned int channels = 0;
		unsigned int width = 0;
		unsigned int mask;
		unsigned int bit_per_frame = 256;
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


/* Audio machine driver */
static struct snd_soc_card cygnussvk_card = {
	.name = "bcm-cygnussvk",
	.owner = THIS_MODULE,
	.dai_link = cygnussvk_dai_links,
//	.num_links = 2,
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

printk("[ADK] %s entered, link#%d\n", __func__, linknum);

	dai_link = &cygnussvk_dai_links[linknum];
	sspcfg = &card_data->sspcfg_info[linknum];

	/* Is there a codec attached to this link?
	 * If not use a dummy codec.
	 */
	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-codec", linknum);
	dai_link->codec_of_node = of_parse_phandle(np, prop_name, 0);
	if (dai_link->codec_of_node == NULL) {
printk("[ADK] %s/%d\n", __func__, __LINE__);
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
			dai_link->ops = &cygnussvk_ops_testport2;
		} else if (linknum == 3) {
			dai_link->codec_dai_name = "snd-soc-dummy-dai";
			dai_link->codec_name = "snd-soc-dummy";
			dai_link->ops = &cygnussvk_ops_testport2;
		}
	} else {
printk("[ADK] %s/%d\n", __func__, __LINE__);
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

/* [ADK]
			dai_link->codec_dai_name = "wm8750-hifi";
			dai_link->ops = &cygnussvk_ops_wm8750;
			dai_link->init = cygnussvk_init_wm8750;
*/			
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

	printk("[ADK] %s set codec[%d] slave=%d\n", __func__, linknum, sspcfg->codec_slave_mode);

	snprintf(prop_name, PROP_LEN_MAX, "bcm,cygnus-link%d-ssp", linknum);
	if (of_property_read_string(np, prop_name, &dai_link->cpu_dai_name) != 0) {
		if (strcmp(dai_link->codec_dai_name, "snd-soc-dummy-dai")) {
			dev_err(&pdev->dev, "Could not find %s", prop_name);
			ret = -EINVAL;
			goto err_exit;
		}
	}

	printk("[ADK] %s set codec[%d]  cpu_dai_name=[%s]\n", __func__, linknum, dai_link->cpu_dai_name);

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

	printk("[ADK] %s set codec[%d] clksrc=%d\n", __func__, linknum, sspcfg->clksrc);

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
	printk("[ADK] %s set codec[%d] mode=%d\n", __func__, linknum, sspcfg->sspmode);

err_exit:
	return ret;
}

/* Parse the DT entries spefically for the TI3106 daughter card */
static int get_ti3106_daughtercard_dt_into(struct platform_device *pdev,
					struct cygnussvk_data *card_data)
{
	struct cygnussvk_ti3106_daughtercard_info  *cfg_3106;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	cfg_3106 = &card_data->ti3106cfg;

	ret = of_property_read_u32(np, "bcm,scslot", &cfg_3106->smartcardslot);
	if (ret) {
		dev_err(&pdev->dev, "Could not find the sc slot num\n");
		return ret;
	}

	/* Get the gpio number from the device tree for the gpio used
	 * to control bank selection
	 */
	ret = of_get_named_gpio(np, "bcm,gpio-bank-sel0", 0);
	if ((ret < 0) && (ret != BCMVP_UNUSED_GPIO)) {
		dev_err(&pdev->dev, "DT err with gpio-bank-sel0\n");
		return ret;
	}
	cfg_3106->gpio_bank_sel0 = ret;

	if (BCMVP_UNUSED_GPIO != cfg_3106->gpio_bank_sel0) {
		ret = devm_gpio_request_one(&pdev->dev,
				cfg_3106->gpio_bank_sel0,
				GPIOF_OUT_INIT_LOW,
				"gpio_bank_sel0");
		if (ret) {
			dev_err(&pdev->dev, "request failed gpio_bank_sel0\n");
			return ret;
		}
	}

	ret = of_get_named_gpio(np, "bcm,gpio-bank-sel1", 0);
	if ((ret < 0) && (ret != BCMVP_UNUSED_GPIO)) {
		dev_err(&pdev->dev, "DT err with gpio-bank-sel1\n");
		return ret;
	}
	cfg_3106->gpio_bank_sel1 = ret;

	if (BCMVP_UNUSED_GPIO != cfg_3106->gpio_bank_sel1) {
		ret = devm_gpio_request_one(&pdev->dev,
					cfg_3106->gpio_bank_sel1,
					GPIOF_OUT_INIT_HIGH,
					"gpio_bank_sel1");
		if (ret) {
			dev_err(&pdev->dev, "request failed gpio_bank_sel1\n");
			return ret;
		}
	}

	/* Get the gpio number from the device tree for the gpio used
	 * to control the external headset amplifier.
	 */
	ret = of_get_named_gpio(np, "bcm,gpio-ext-headset-amp-en", 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "DT err with gpio-ext-headset-amp-en\n");
		return ret;
	}
	cfg_3106->gpio_ext_headset_amp_en = ret;

	ret = devm_gpio_request_one(&pdev->dev,
					cfg_3106->gpio_ext_headset_amp_en,
					GPIOF_OUT_INIT_LOW,
					"ext-headset-amp-en");
	if (ret) {
		dev_err(&pdev->dev, "request failed ext-headset-amp-en\n");
		return ret;
	}

	/* Get the gpio number from the device tree for the gpio used
	 * to control the handsfree amplifier.
	 */
	ret = of_get_named_gpio(np, "bcm,gpio-handsfree-amp-en", 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "DT err with gpio-handsfree-amp-en\n");
		return ret;
	}
	cfg_3106->gpio_handsfree_amp_en = ret;

	ret = devm_gpio_request_one(&pdev->dev,
					cfg_3106->gpio_handsfree_amp_en,
					GPIOF_OUT_INIT_LOW,
					"handsfree-amp-en");
	if (ret) {
		dev_err(&pdev->dev, "request failed gpio_handsfree_amp_en\n");
		return ret;
	}

	dev_dbg(&pdev->dev, "gpio_bank_sel0 %d\n", cfg_3106->gpio_bank_sel0);
	dev_dbg(&pdev->dev, "gpio_bank_sel1 %d\n", cfg_3106->gpio_bank_sel1);
	dev_dbg(&pdev->dev, "gpio_ext_headset_amp_en %d\n",
		cfg_3106->gpio_ext_headset_amp_en);
	dev_dbg(&pdev->dev, "gpio_handsfree_amp_en %d\n",
		cfg_3106->gpio_handsfree_amp_en);

	return 0;
}

void cygnus_pinmux_raw_dump(void); 

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

/* --------- [ADK] ------------
 * [ADK]  12/09/2015     Late will use  ti320aic3x codec ..
 
	/* Its possible there is a TI3106 on link #0 and/or #1 * /
	if ((cygnussvk_dai_links[0].codec_of_node != NULL) ||
			(cygnussvk_dai_links[1].codec_of_node != NULL)) {
		ret = get_ti3106_daughtercard_dt_into(pdev, card_data);
		if (ret < 0)
			goto err_exit;
	} else {
		dev_dbg(&pdev->dev, "TI3106 card not configured for board.\n");
	}
----------------------------- */

	snd_soc_card_set_drvdata(card, card_data);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		goto err_exit;
	}

	cygnus_pinmux_raw_dump();
	
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
	{.compatible = "bcm,cygnussvk-machine", },
	{ },
};
MODULE_DEVICE_TABLE(of, cygnus_mach_of_match);

static struct platform_driver bcm_cygnus_driver = {
	.driver = {
		.name = "bcm-cygnussvk-machine",
		.of_match_table = cygnus_mach_of_match,
	},
	.probe  = cygnussvk_probe,
	.remove = cygnussvk_remove,
};

module_platform_driver(bcm_cygnus_driver);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("ALSA SoC for Cygnus APs");
MODULE_LICENSE("GPL v2");
