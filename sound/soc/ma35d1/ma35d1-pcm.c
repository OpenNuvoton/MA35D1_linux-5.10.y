// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/platform_data/dma-imx.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>

#include "ma35d1-audio.h"

#define MA35D1_DMABUF_SIZE	(64 * 1024)

static const struct snd_pcm_hardware ma35d1_pcm_hardware = {
	.info           = SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER |
	SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_MMAP_VALID |
	SNDRV_PCM_INFO_PAUSE |
	SNDRV_PCM_INFO_RESUME,
	.formats        = SNDRV_PCM_FMTBIT_S16_LE |
	SNDRV_PCM_FMTBIT_S24_LE,
	.rates          = SNDRV_PCM_RATE_32000 |
	SNDRV_PCM_RATE_44100 |
	SNDRV_PCM_RATE_48000,
	.channels_min       = 1,
	.channels_max       = 2,
	.buffer_bytes_max   = 64*1024,
	.period_bytes_min   = 4*1024,
	.period_bytes_max   = 8*1024,
	.periods_min        = 1,
	.periods_max        = 1024,
};

static const struct snd_dmaengine_pcm_config ma35d1_dmaengine_pcm_config = {
	.pcm_hardware = &ma35d1_pcm_hardware,
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.prealloc_buffer_size = 32 * 1024,
};

static int ma35d1_soc_platform_probe(struct platform_device *pdev)
{
	return devm_snd_dmaengine_pcm_register(&pdev->dev, &ma35d1_dmaengine_pcm_config,
						SND_DMAENGINE_PCM_FLAG_COMPAT);
}

static int ma35d1_soc_platform_remove(struct platform_device *pdev)
{
	snd_dmaengine_pcm_unregister(&pdev->dev);
	return 0;
}

static const struct of_device_id ma35d1_audio_pcm_of_match[] = {
	{   .compatible = "nuvoton,ma35d1-audio-pcm"    },
	{   },
};
MODULE_DEVICE_TABLE(of, ma35d1_audio_pcm_of_match);

static struct platform_driver ma35d1_pcm_driver = {
	.driver = {
		.name = "ma35d1-audio-pcm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_audio_pcm_of_match),
	},

	.probe = ma35d1_soc_platform_probe,
	.remove = ma35d1_soc_platform_remove,
};

module_platform_driver(ma35d1_pcm_driver);

MODULE_DESCRIPTION("ma35d1 Audio DMA module");
MODULE_LICENSE("GPL");

