/*
 * Intel Broxton-P I2S Machine Driver for IVI - HU
 *
 * Copyright (C) 2014-2015, Intel Corporation. All rights reserved.
 *
 * Modified from:
 *   Intel BXT P RT298 Machine driver
 *   Intel Skylake I2S Machine driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>

#include "../../codecs/rt298.h"

static struct snd_soc_jack broxton_headset;
/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin broxton_headset_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct snd_kcontrol_new broxton_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

static const struct snd_soc_dapm_widget broxton_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
};

static const struct snd_soc_dapm_route broxton_rt298_map[] = {
	{"Speaker", NULL, "Dummy Playback"},
	{"Dummy Capture", NULL, "DMIC2"},

	{ "Dummy Playback", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "codec0_out"},

	{ "bt_ssp0_in", NULL, "ssp0 Rx" },
	{ "ssp0 Rx", NULL, "Dummy Capture" },
};

static int broxton_rt298_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret = 0;
	char *gpio_addr, *mclk_addr;
	u32 gpio_value = 0x44000800;
	u32 mclk_value = 0x44000400;

	ret = snd_soc_card_jack_new(rtd->card, "Headset",
		SND_JACK_HEADSET | SND_JACK_BTN_0,
		&broxton_headset,
		broxton_headset_pins, ARRAY_SIZE(broxton_headset_pins));
	if (ret)
		return ret;

	return 0;

}

static int broxton_ssp0_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	/* SSP0 operates with a BT Transceiver */
	return 0;
}

static int broxton_ssp5_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
				SNDRV_PCM_HW_PARAM_CHANNELS);

	/* The ADSP will covert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP5 to 16 bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				SNDRV_PCM_HW_PARAM_FIRST_MASK],
				SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

static int broxton_rt298_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, RT298_SCLK_S_PLL, 19200000,
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk configuration\n");
		return ret;
	}

	return ret;
}

static struct snd_soc_ops broxton_rt298_ops = {
	.hw_params = broxton_rt298_hw_params,
};

/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_rt298_dais[] = {
	/* Front End DAI links */
	{
		.name = "Bxt Audio Port 1",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	{
		.name = "Bxt Audio Reference cap",
		.stream_name = "BT Capture",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.init = NULL,
		.dpcm_capture = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	/* Back End DAI links */
	{
		/* SSP0 - Codec - for HDMI MCH */
		.name = "SSP0-Codec",
		.be_id = 0,
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:00:0e.0",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.init = NULL,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = broxton_ssp0_fixup,
		.dpcm_capture = 1,
		.dpcm_playback = 1,
	},
	{
		/* SSP5 - Codec */
		.name = "SSP5-Codec",
		.be_id = 4,
		.cpu_dai_name = "SSP5 Pin",
		.platform_name = "0000:00:0e.0",
		.no_pcm = 1,
		.codec_name = "i2c-INT343A:00",
		.codec_dai_name = "rt298-aif1",
		.init = broxton_rt298_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = broxton_ssp5_fixup,
		.ops = &broxton_rt298_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
};

/* broxton audio machine driver for SPT + RT298S */
static struct snd_soc_card broxton_rt298 = {
	.name = "broxton-rt298",
	.owner = THIS_MODULE,
	.dai_link = broxton_rt298_dais,
	.num_links = ARRAY_SIZE(broxton_rt298_dais),
	.controls = broxton_controls,
	.num_controls = ARRAY_SIZE(broxton_controls),
	.dapm_widgets = broxton_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_widgets),
	.dapm_routes = broxton_rt298_map,
	.num_dapm_routes = ARRAY_SIZE(broxton_rt298_map),
	.fully_routed = true,
};

static int broxton_audio_probe(struct platform_device *pdev)
{
	broxton_rt298.dev = &pdev->dev;

	return snd_soc_register_card(&broxton_rt298);
}

static int broxton_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&broxton_rt298);
	return 0;
}

static struct platform_driver broxton_audio = {
	.probe = broxton_audio_probe,
	.remove = broxton_audio_remove,
	.driver = {
		.name = "bxt_alc298s_i2s",
	},
};

module_platform_driver(broxton_audio)

/* Module information */
MODULE_AUTHOR("Pardha Saradhi K <pardha.saradhi.kesapragada@intel.com>");
MODULE_AUTHOR("Ramesh Babu <Ramesh.Babu@intel.com>");
MODULE_AUTHOR("Senthilnathan Veppur <senthilnathanx.veppur@intel.com>");
MODULE_DESCRIPTION("Intel SST Audio for Broxton-P IVI HU");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_alc298s_i2s");
