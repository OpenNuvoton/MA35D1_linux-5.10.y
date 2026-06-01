// SPDX-License-Identifier: GPL-2.0
/*
 * MP34DT01-M PDM microphone ASoC codec driver.
 *
 * The device outputs raw PDM on the serial data pin. This driver exposes the
 * stream to ALSA without doing PDM-to-PCM conversion; userspace is expected to
 * decode the captured bitstream.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

static struct snd_soc_dai_driver mp34dt01m_dai = {
	.name = "mp34dt01m-pdm",
	.capture = {
		.stream_name = "PDM Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct snd_soc_component_driver mp34dt01m_component = {
	.name = "mp34dt01m",
};

static int mp34dt01m_probe(struct platform_device *pdev)
{
	return devm_snd_soc_register_component(&pdev->dev,
					       &mp34dt01m_component,
					       &mp34dt01m_dai, 1);
}

static const struct of_device_id mp34dt01m_of_match[] = {
	{ .compatible = "st,mp34dt01m" },
	{}
};
MODULE_DEVICE_TABLE(of, mp34dt01m_of_match);

static struct platform_driver mp34dt01m_driver = {
	.driver = {
		.name = "mp34dt01m",
		.of_match_table = mp34dt01m_of_match,
	},
	.probe = mp34dt01m_probe,
};
module_platform_driver(mp34dt01m_driver);

MODULE_DESCRIPTION("MP34DT01-M raw PDM microphone codec driver");
MODULE_LICENSE("GPL");
