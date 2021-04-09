// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "ma35d1-audio.h"
#include "../codecs/nau8822.h"

static int ma35d1_audio_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	unsigned int clk = 0;
	unsigned int sample_rate = params_rate(params);

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	clk = 256 * sample_rate;
	cpu_dai->channels = params_channels(params);
	cpu_dai->rate = params_rate(params);
	cpu_dai->sample_bits = params_width(params);

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_CLK_MCLK, clk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret =snd_soc_dai_set_sysclk(cpu_dai, MA35D1_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = snd_soc_dai_set_sysclk(cpu_dai, MA35D1_AUDIO_SAMPLECLK, sample_rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops ma35d1_audio_ops = {
	.hw_params = ma35d1_audio_hw_params,
};

SND_SOC_DAILINK_DEFS(hifi,
                     DAILINK_COMP_ARRAY(COMP_CPU("40480000.i2s")),
                     DAILINK_COMP_ARRAY(COMP_CODEC("nau8822.0-001a", "nau8822-hifi")),
                     DAILINK_COMP_ARRAY(COMP_PLATFORM("i2s_pcm")));

static struct snd_soc_dai_link ma35d1evb_i2s_dai = {
	.name               =   "IIS",
	.stream_name        =   "IIS HiFi",
	.ops                =   &ma35d1_audio_ops,
	SND_SOC_DAILINK_REG(hifi),
};

static struct snd_soc_card ma35d1evb_audio_machine = {
	.name       =   "ma35d1_IIS",
	.owner      =   THIS_MODULE,
	.dai_link   =   &ma35d1evb_i2s_dai,
	.num_links  =   1,
};

static int ma35d1_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &ma35d1evb_audio_machine;
	int ret;

	card->dev = &pdev->dev;

	if (np) {
		ma35d1evb_i2s_dai.cpus->of_node = of_parse_phandle(np, "i2s-controller", 0);
		if (!ma35d1evb_i2s_dai.cpus->of_node) {
			dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
			ret = -EINVAL;
		}

	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int ma35d1_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id ma35d1_audio_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-audio" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_audio_of_match);

static struct   platform_driver ma35d1_audio_driver = {
	.driver = {
		.name = "ma35d1-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_audio_of_match),
	},
	.probe = ma35d1_audio_probe,
	.remove = ma35d1_audio_remove,
};

module_platform_driver(ma35d1_audio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MA35D1 Series ASoC audio support");
