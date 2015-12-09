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

static int broxton_ssp0_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	/* SSP0 operates with a BT Transceiver */
	return 0;
}

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
};

/* broxton audio machine driver for SPT + RT298S */
static struct snd_soc_card broxton_rt298 = {
	.name = "broxton-ivi-hu",
	.owner = THIS_MODULE,
	.dai_link = broxton_rt298_dais,
	.num_links = ARRAY_SIZE(broxton_rt298_dais),
	.controls = NULL,
	.num_controls = 0,
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
		.name = "bxt_ivi_hu_i2s",
	},
};

module_platform_driver(broxton_audio)

/* Module information */
MODULE_AUTHOR("Pardha Saradhi K <pardha.saradhi.kesapragada@intel.com>");
MODULE_AUTHOR("Ramesh Babu <Ramesh.Babu@intel.com>");
MODULE_AUTHOR("Senthilnathan Veppur <senthilnathanx.veppur@intel.com>");
MODULE_DESCRIPTION("Intel SST Audio for Broxton-P IVI HU");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_ivi_hu_i2s");
