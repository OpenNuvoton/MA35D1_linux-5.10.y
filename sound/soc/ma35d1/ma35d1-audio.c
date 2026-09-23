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
#include <linux/clk.h>

#include "ma35d1-audio.h"
#include "../codecs/nau8822.h"

/*
 * TAC5X1X_CLK_MCLK matches TAC5X1X_CLK_MCLK from ../codecs/tac5x1x.h.
 * Defined locally so this driver does not need to depend on / include
 * the TAC5212 codec driver header, which may not be present in builds
 * that only use NAU8822. If the TAC5212 codec driver is added later,
 * this value must stay in sync with its definition.
 */
#define TAC5X1X_CLK_MCLK	0

static int ma35d1_audio_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	unsigned int fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(cpu_dai->dev);
	unsigned int cpu_mclk;
	int ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;

	if (of_device_is_compatible(codec_dai->dev->of_node, "st,mp34dt01m"))
		return 0;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

	cpu_mclk = ma35d1_audio->actual_mclk_out;
	if (!cpu_mclk) {
		dev_err(cpu_dai->dev,
			"hw_params: invalid actual_mclk_out=0 requested_mclk=%u\n",
			ma35d1_audio->mclk_out);
		return -EINVAL;
	}

	/* set the codec system clock */
	if (of_device_is_compatible(codec_dai->dev->of_node, "ti,tac5212"))
		ret = snd_soc_dai_set_sysclk(codec_dai, TAC5X1X_CLK_MCLK,
					     cpu_mclk, SND_SOC_CLOCK_IN);
	else
		ret = snd_soc_dai_set_sysclk(codec_dai, NAU8822_CLK_MCLK,
					     cpu_mclk, SND_SOC_CLOCK_IN);
	if (ret < 0 )
		return ret;

	return 0;
}

static struct snd_soc_ops ma35d1_audio_ops = {
	.hw_params = ma35d1_audio_hw_params,
};

struct ma35d1_audio_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
};

static int ma35d1_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ma35d1_audio_card *priv;
	const char *model;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->card.dev = &pdev->dev;
	priv->card.owner = THIS_MODULE;
	priv->card.dai_link = &priv->dai_link;
	priv->card.num_links = 1;

	if (!of_property_read_string(np, "model", &model))
		priv->card.name = model;
	else
		priv->card.name = "ma35d1_IIS";

	priv->dai_link.name = "IIS";
	priv->dai_link.stream_name = "IIS HiFi";
	priv->dai_link.ops = &ma35d1_audio_ops;
	priv->dai_link.cpus = &priv->cpu;
	priv->dai_link.num_cpus = 1;
	priv->dai_link.codecs = &priv->codec;
	priv->dai_link.num_codecs = 1;
	priv->dai_link.platforms = &priv->platform;
	priv->dai_link.num_platforms = 1;

	priv->cpu.of_node = of_parse_phandle(np, "i2s-controller", 0);
	if (!priv->cpu.of_node) {
		dev_err(&pdev->dev, "Property 'i2s-controller' missing or invalid\n");
		return -EINVAL;
	}

	priv->platform.of_node = of_parse_phandle(np, "i2s-platform", 0);
	if (!priv->platform.of_node)
		priv->platform.name = "i2s_pcm";

	priv->codec.of_node = of_parse_phandle(np, "audio-codec", 0);
	if (priv->codec.of_node) {
		if (of_device_is_compatible(priv->codec.of_node, "st,mp34dt01m"))
			priv->codec.dai_name = "mp34dt01m-pdm";
		else if (of_device_is_compatible(priv->codec.of_node, "ti,tac5212"))
			priv->codec.dai_name = "tac5x1x-hifi";
		else
			priv->codec.dai_name = "nau8822-hifi";
	} else {
		priv->codec.name = "nau8822.0-001a";
		priv->codec.dai_name = "nau8822-hifi";
	}

	ret = devm_snd_soc_register_card(&pdev->dev, &priv->card);
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

