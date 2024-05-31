// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Nuvoton technology corporation.
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
#include <linux/clk.h>

#include "ma35h0-audio.h"
#include "../codecs/nau8822.h"

static int ma35h0_audio_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	unsigned int fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ma35h0_audio *ma35h0_audio = dev_get_drvdata(cpu_dai->dev);
	unsigned int i2s_clk, cpu_mclk;
	int ret;

	unsigned int sample_rate = params_rate(params);

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;

	cpu_dai->channels = params_channels(params);
	cpu_dai->rate = params_rate(params);
	cpu_dai->sample_bits = params_width(params);

	i2s_clk = clk_get_rate(ma35h0_audio->clk);
	cpu_mclk = i2s_clk/(2*8);

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_CLK_PLL,
				      cpu_mclk, SND_SOC_CLOCK_IN);
	if (ret < 0 )
		return ret;

	ret = snd_soc_dai_set_pll(codec_dai, 0, 0,
				  cpu_mclk, 256 * params_rate(params));
	if (ret < 0 )
		return ret;

	/* set prescaler division for sample rate */
	ret = snd_soc_dai_set_sysclk(cpu_dai, MA35H0_AUDIO_CLKDIV, sample_rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = snd_soc_dai_set_sysclk(cpu_dai, MA35H0_AUDIO_SAMPLECLK, sample_rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops ma35h0_audio_ops = {
	.hw_params = ma35h0_audio_hw_params,
};

SND_SOC_DAILINK_DEFS(hifi,
			DAILINK_COMP_ARRAY(COMP_CPU("40480000.i2s")),
			DAILINK_COMP_ARRAY(COMP_CODEC("nau8822.0-001a", "nau8822-hifi")),
			DAILINK_COMP_ARRAY(COMP_PLATFORM("i2s_pcm")));

static struct snd_soc_dai_link ma35h0evb_i2s_dai = {
	.name               =   "IIS",
	.stream_name        =   "IIS HiFi",
	.ops                =   &ma35h0_audio_ops,
	SND_SOC_DAILINK_REG(hifi),
};

static struct snd_soc_card ma35h0evb_audio_machine = {
	.name       =   "ma35h0_IIS",
	.owner      =   THIS_MODULE,
	.dai_link   =   &ma35h0evb_i2s_dai,
	.num_links  =   1,
};

static int ma35h0_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &ma35h0evb_audio_machine;
	int ret;

	card->dev = &pdev->dev;

	if (np) {
		ma35h0evb_i2s_dai.cpus->of_node = of_parse_phandle(np, "i2s-controller", 0);
		if (!ma35h0evb_i2s_dai.cpus->of_node) {
			dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
			ret = -EINVAL;
		}

	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int ma35h0_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

static const struct of_device_id ma35h0_audio_of_match[] = {
	{ .compatible = "nuvoton,ma35h0-audio" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35h0_audio_of_match);

static struct   platform_driver ma35h0_audio_driver = {
	.driver = {
		.name = "ma35h0-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35h0_audio_of_match),
	},
	.probe = ma35h0_audio_probe,
	.remove = ma35h0_audio_remove,
};

module_platform_driver(ma35h0_audio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MA35H0 Series ASoC audio support");

