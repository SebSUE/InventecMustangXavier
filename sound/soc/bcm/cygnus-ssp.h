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
#ifndef __CYGNUS_SSP_H__
#define __CYGNUS_SSP_H__

#include "cygnus-pcm.h"

#define CYGNUS_TDM_DAI_MAX_SLOTS 16

#define CYGNUS_MAX_PLAYBACK_PORTS 4
#define CYGNUS_MAX_CAPTURE_PORTS 3
#define CYGNUS_MAX_PORTS  CYGNUS_MAX_PLAYBACK_PORTS

#define CYGNUS_SSP_FRAMEBITS_DIV 1

#define CYGNUS_SSPMODE_I2S 0
#define CYGNUS_SSPMODE_TDM 1
#define CYGNUS_SSPMODE_UNKNOWN -1

#define CYGNUS_SSP_CLKSRC_PLL      0
#define CYGNUS_SSP_CLKSRC_NCO_0    1
#define CYGNUS_SSP_CLKSRC_NCO_1    2


struct cygnus_ssp_regs {
	u32 i2s_stream_cfg;
	u32 i2s_cfg;
	u32 i2s_cap_stream_cfg;
	u32 i2s_cap_cfg;
	u32 i2s_mclk_cfg;

	u32 bf_destch_ctrl;
	u32 bf_destch_cfg;
	u32 bf_sourcech_ctrl;
	u32 bf_sourcech_cfg;
};

struct cygnus_aio_port {
	struct device *dev;

	int portnum;
	int mode;
	bool slave;       /* 0 = master mode,  1 = slave mode */
	int streams_on;   /* will be 0 if both capture and play are off */
	int channel_grouping;
	int fsync_width;
	unsigned int fs_delay;
	bool invert_bclk;
	bool invert_fs;


	u32 mclk;
	u32 lrclk;

	unsigned int slot_width;
	unsigned int slots_per_frame;
	unsigned int active_slots;

	bool enable_i2s_32fs;
	bool fix_multichan_tdm_shift_tx;
	bool fix_multichan_tdm_shift_slave_rx;
	int gpio_latchctrl;

	void __iomem *audio;

	struct cygnus_ssp_regs regs;

	struct ringbuf_regs play_rb_regs;
	struct ringbuf_regs capture_rb_regs;

	struct snd_pcm_substream *play_stream;
	struct snd_pcm_substream *capture_stream;
};


struct cygnus_audio {
	struct cygnus_aio_port  portinfo[CYGNUS_MAX_PORTS];

	int irq_num;
	void __iomem *audio;
	struct device *dev;
};

extern int cygnus_ssp_get_mode(struct snd_soc_dai *cpu_dai);
extern int cygnus_ssp_add_pll_tweak_controls(struct snd_soc_pcm_runtime *rtd);
extern int cygnus_ssp_set_custom_fsync_width(struct snd_soc_dai *cpu_dai,
						int len);
extern int cygnus_soc_platform_register(struct device *dev,
					struct cygnus_audio *cygaud);
extern int cygnus_soc_platform_unregister(struct device *dev);
extern int cygnus_ssp_set_custom_fsync_width(struct snd_soc_dai *cpu_dai,
	int len);
extern int cygnus_fix_multichan_tdm_shift_tx(struct snd_soc_dai *cpu_dai,
	bool enable);
extern int cygnus_fix_multichan_tdm_shift_slave_rx(struct snd_soc_dai *cpu_dai,
	bool enable, int gpio);
int cygnus_enable_i2s_32fs(struct snd_soc_dai *cpu_dai, bool enable);


#endif
