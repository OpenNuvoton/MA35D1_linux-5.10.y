// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dai.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/of.h>

#include "ma35d1-audio.h"

static DEFINE_MUTEX(i2s_mutex);
struct ma35d1_audio *ma35d1_i2s_data;

static int ma35d1_i2s_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params,
                                struct snd_soc_dai *dai)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(dai->dev);
	unsigned long val = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL0);
	unsigned long ctl1;

	switch (params_width(params)) {
	case 8:
		val = (val & ~DATWIDTH) | DATWIDTH_8;
		val = (val & ~CHWIDTH) | CHWIDTH_8;
		break;

	case 16:
		val = (val & ~DATWIDTH) | DATWIDTH_16;
		val = (val & ~CHWIDTH) | CHWIDTH_16;
		ctl1 = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL1);
		ctl1 = ctl1 | PBWIDTH_16;	//set Peripheral Bus Data Width to 16 bit
		AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL1, ctl1);
		break;

	case 24:
		val = (val & ~DATWIDTH) | DATWIDTH_24;
		val = (val & ~CHWIDTH) | CHWIDTH_24;
		break;

	case 32:
		val = (val & ~DATWIDTH) | DATWIDTH_32;
		val = (val & ~CHWIDTH) | CHWIDTH_32;
		break;

	default:
		return -EINVAL;
	}
	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);
	return 0;
}

static int ma35d1_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(cpu_dai->dev);
	unsigned long val = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL0);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_MSB:
		val |= ORDER; //MSB
		break;
	case SND_SOC_DAIFMT_I2S:
		val &= ~ORDER; //LSB
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		val &= ~SLAVE; //Master
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= SLAVE; //Slave
		break;
	default:
		return -EINVAL;
	}

	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

	return 0;
}

static int ma35d1_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id, unsigned int freq, int dir)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(cpu_dai->dev);
	unsigned int i2s_clk, bitrate, mclkdiv, bclkdiv;

	i2s_clk = clk_get_rate(ma35d1_audio->clk);

	bitrate = freq * 2U * 16U;
	bclkdiv = ((((i2s_clk * 10UL / bitrate) >> 1U) + 5UL) / 10UL) - 1U;


	mclkdiv = (i2s_clk / 12000000) >> 1;

	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CLKDIV, (bclkdiv << 8) | mclkdiv);

	return 0;

}

static int ma35d1_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(dai->dev);
	int ret = 0;
	unsigned long val;

	val = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL0);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		val |= I2S_EN;
		val |= MCLKEN;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val |= TX_EN | TXPDMAEN;
		} else {
			val |= RX_EN | RXPDMAEN;
		}
		AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		val &= ~I2S_EN;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			val &= ~(TX_EN | TXPDMAEN);
		} else {
			val &= ~(RX_EN | RXPDMAEN);
		}

		AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ma35d1_i2s_probe(struct snd_soc_dai *dai)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(dai->dev);

	mutex_lock(&i2s_mutex);

	/* Init DMA data */
	ma35d1_audio->dma_params_rx.addr = ma35d1_audio->phyaddr + I2S_RXFIFO;
	ma35d1_audio->dma_params_rx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	ma35d1_audio->dma_params_rx.slave_id = ma35d1_audio->pdma_reqsel_rx;
	ma35d1_audio->dma_params_tx.addr = ma35d1_audio->phyaddr + I2S_TXFIFO;
	ma35d1_audio->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	ma35d1_audio->dma_params_tx.slave_id = ma35d1_audio->pdma_reqsel_tx;

	snd_soc_dai_init_dma_data(dai, &ma35d1_audio->dma_params_tx,
				&ma35d1_audio->dma_params_rx);

	snd_soc_dai_set_drvdata(dai, ma35d1_audio);

	/* Set Audio_JKEN pin */
	ma35d1_audio->pwdn_gpio = devm_gpiod_get_optional(dai->dev, "powerdown",
							GPIOD_OUT_HIGH);
	if (IS_ERR(ma35d1_audio->pwdn_gpio))
		return PTR_ERR(ma35d1_audio->pwdn_gpio);

	gpiod_set_value_cansleep(ma35d1_audio->pwdn_gpio, 0);


	mutex_unlock(&i2s_mutex);

	return 0;
}

static int ma35d1_i2s_remove(struct snd_soc_dai *dai)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(dai->dev);
	clk_disable(ma35d1_audio->clk);

	return 0;
}

static struct snd_soc_dai_ops ma35d1_i2s_dai_ops = {
	.trigger    = ma35d1_i2s_trigger,
	.hw_params  = ma35d1_i2s_hw_params,
	.set_fmt    = ma35d1_i2s_set_fmt,
	.set_sysclk = ma35d1_i2s_set_sysclk,
};

struct snd_soc_dai_driver ma35d1_i2s_dai = {
	.name		= "i2s_pcm",
	.probe          = ma35d1_i2s_probe,
	.remove         = ma35d1_i2s_remove,
	.playback = {
		.rates      = SNDRV_PCM_RATE_8000_48000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min   = 1,
		.channels_max   = 2,
	},
	.capture = {
		.rates      = SNDRV_PCM_RATE_8000_48000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min   = 1,
		.channels_max   = 2,
	},
	.ops = &ma35d1_i2s_dai_ops,
};

static const struct snd_soc_component_driver ma35d1_i2s_component = {
	.name       = "ma35d1-i2s",
};


static int ma35d1_i2s_drvprobe(struct platform_device *pdev)
{
	struct ma35d1_audio *ma35d1_audio;
	u32	val32[4];
	u32	dma_tx_num, dma_rx_num;

	int ret;

	if (ma35d1_i2s_data)
		return -EBUSY;

	ma35d1_audio = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_audio), GFP_KERNEL);
	if (!ma35d1_audio)
		return -ENOMEM;

	spin_lock_init(&ma35d1_audio->lock);
	spin_lock_init(&ma35d1_audio->irqlock);

	if (of_property_read_u32_array(pdev->dev.of_node, "reg", val32, 4) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	ma35d1_audio->phyaddr = val32[1];

	if (of_property_read_u32(pdev->dev.of_node, "pdma_reqsel_tx", &dma_tx_num) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	ma35d1_audio->pdma_reqsel_tx = dma_tx_num;
	pr_debug("ma35d1_audio->pdma_reqsel_tx = 0x%lx\n",(ulong)ma35d1_audio->pdma_reqsel_tx);

	if (of_property_read_u32(pdev->dev.of_node, "pdma_reqsel_rx", &dma_rx_num) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	ma35d1_audio->pdma_reqsel_rx = dma_rx_num;
	pr_debug("ma35d1_audio->pdma_reqsel_rx = 0x%lx\n",(ulong)ma35d1_audio->pdma_reqsel_rx);

	ma35d1_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ma35d1_audio->res) {
		dev_err(&pdev->dev, "platform_get_resource error\n");
		ret = -ENODEV;
		goto out1;
	}

	ma35d1_audio->mmio = devm_ioremap_resource(&pdev->dev, ma35d1_audio->res);

	ma35d1_audio->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ma35d1_audio->clk)) {
		dev_err(&pdev->dev, "clk_get error\n");
		ret = PTR_ERR(ma35d1_audio->clk);
		goto out2;
	}
	clk_prepare_enable(ma35d1_audio->clk);

	ma35d1_audio->irq_num = platform_get_irq(pdev, 0);
	if (!ma35d1_audio->irq_num) {
		dev_err(&pdev->dev, "platform_get_irq error\n");
		ret = -EBUSY;
		goto out3;
	}

	ma35d1_i2s_data = ma35d1_audio;

	dev_set_drvdata(&pdev->dev, ma35d1_audio);

	ret = devm_snd_soc_register_component(&pdev->dev, &ma35d1_asrc_component,
	                                      &ma35d1_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC DAI\n");
		goto out3;
	}

	return 0;

out3:
	clk_put(ma35d1_audio->clk);
out2:
	iounmap(ma35d1_audio->mmio);
	release_mem_region(ma35d1_audio->res->start, resource_size(ma35d1_audio->res));
out1:
	kfree(ma35d1_audio);

	return ret;
}

static const struct of_device_id ma35d1_audio_i2s_of_match[] = {
	{   .compatible = "nuvoton,ma35d1-audio-i2s"    },
	{   },
};
MODULE_DEVICE_TABLE(of, ma35d1_audio_i2s_of_match);

static int ma35d1_i2s_drvremove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	clk_put(ma35d1_i2s_data->clk);
	iounmap(ma35d1_i2s_data->mmio);
	release_mem_region(ma35d1_i2s_data->res->start, resource_size(ma35d1_i2s_data->res));

	kfree(ma35d1_i2s_data);
	ma35d1_i2s_data = NULL;

	return 0;
}

static struct platform_driver ma35d1_i2s_driver = {
	.driver = {
		.name   = "ma35d1-audio-i2s",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_audio_i2s_of_match),
	},
	.probe      = ma35d1_i2s_drvprobe,
	.remove     = ma35d1_i2s_drvremove,
};

module_platform_driver(ma35d1_i2s_driver);

MODULE_DESCRIPTION("MA35D1 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-i2s");
