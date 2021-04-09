// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Nuvoton technology corporation.
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

extern struct ma35d1_audio *ma35d1_i2s_data;

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

static int ma35d1_dma_hw_free(struct snd_pcm_substream *substream)
{
	struct ma35d1_audio *ma35d1_audio = ma35d1_i2s_data;

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (ma35d1_audio->dma_chan[IN])
		dma_release_channel(ma35d1_audio->dma_chan[IN]);

	if (ma35d1_audio->dma_chan[OUT])
		dma_release_channel(ma35d1_audio->dma_chan[OUT]);

	ma35d1_audio->dma_chan[IN] = NULL;
	ma35d1_audio->dma_chan[OUT] = NULL;

	return 0;
}

static int ma35d1_dma_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ma35d1_audio *ma35d1_audio = ma35d1_i2s_data;

	runtime->private_data = ma35d1_audio;

	snd_pcm_hw_constraint_integer(substream->runtime,
	                              SNDRV_PCM_HW_PARAM_PERIODS);
	snd_soc_set_runtime_hwparams(substream, &ma35d1_pcm_hardware);

	return 0;
}

static int ma35d1_dma_shutdown(struct snd_pcm_substream *substream)
{
	return 0;
}

static const struct snd_pcm_ops ma35d1_asrc_dma_pcm_ops = {
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_free	= ma35d1_dma_hw_free,
	.open		= ma35d1_dma_startup,
	.close		= ma35d1_dma_shutdown,
};

static int ma35d1_asrc_dma_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm_substream *substream;
	struct snd_pcm *pcm = rtd->pcm;
	int ret, i;

	ret = dma_coerce_mask_and_coherent(card->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(card->dev, "failed to set DMA mask\n");
		return ret;
	}

	for (i = SNDRV_PCM_STREAM_PLAYBACK; i <= SNDRV_PCM_STREAM_LAST; i++) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;

		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, pcm->card->dev,
		                          MA35D1_DMABUF_SIZE, &substream->dma_buffer);
		if (ret) {
			dev_err(card->dev, "failed to allocate DMA buffer\n");
			goto err;
		}
	}

	return 0;

err:
	if (--i == 0 && pcm->streams[i].substream)
		snd_dma_free_pages(&pcm->streams[i].substream->dma_buffer);

	return ret;
}

static void ma35d1_asrc_dma_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	int i;

	for (i = SNDRV_PCM_STREAM_PLAYBACK; i <= SNDRV_PCM_STREAM_LAST; i++) {
		substream = pcm->streams[i].substream;
		if (!substream)
			continue;

		snd_dma_free_pages(&substream->dma_buffer);
		substream->dma_buffer.area = NULL;
		substream->dma_buffer.addr = 0;
	}
}


struct snd_soc_component_driver ma35d1_asrc_component = {
	.name		= DRV_NAME,
	.ops		= &ma35d1_asrc_dma_pcm_ops,
	.pcm_new	= ma35d1_asrc_dma_pcm_new,
	.pcm_free	= ma35d1_asrc_dma_pcm_free,
};
EXPORT_SYMBOL_GPL(ma35d1_asrc_component);


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
