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

#define MA35D1_APLL_RATE_44K1_FAMILY	180633600UL
#define MA35D1_APLL_RATE_48K_FAMILY	196608000UL

#define MA35D1_APLL_FAMILY_NONE	0U
#define MA35D1_APLL_FAMILY_44K1	1U
#define MA35D1_APLL_FAMILY_48K	2U

static int ma35d1_i2s_rate_to_apll(unsigned int sample_rate,
				    unsigned int *family,
				    unsigned long *target_rate)
{
	switch (sample_rate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		*family = MA35D1_APLL_FAMILY_44K1;
		*target_rate = MA35D1_APLL_RATE_44K1_FAMILY;
		return 0;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 96000:
	case 192000:
		*family = MA35D1_APLL_FAMILY_48K;
		*target_rate = MA35D1_APLL_RATE_48K_FAMILY;
		return 0;
	default:
		return -EINVAL;
	}
}

/*
 * The MA35D1 VSI-PLL clk driver's .set_rate() always leaves the PD
 * (power-down) bit set after reprogramming M/N/P; only .prepare() clears
 * it again. Since the clk framework does NOT call .prepare() during a
 * plain clk_set_rate() on an already-enabled clock, we must explicitly
 * unprepare (disable) the APLL and the I2S gate clock that depends on it
 * before changing the rate, then re-prepare (enable) the APLL afterwards
 * so its .prepare() callback clears the PD bit again, and finally
 * re-enable the I2S gate clock.
 */
static int ma35d1_i2s_configure_apll(struct device *dev,
				      struct ma35d1_audio *ma35d1_audio,
				      unsigned int sample_rate,
				      unsigned int *family_out,
				      unsigned int *i2s_clk)
{
	unsigned long target_rate, old_rate;
	unsigned int family;
	int ret;

	ret = ma35d1_i2s_rate_to_apll(sample_rate, &family, &target_rate);
	if (ret) {
		dev_err(dev, "hw_params: unsupported sample rate for APLL selection: %u\n",
			sample_rate);
		return ret;
	}

	mutex_lock(&ma35d1_audio->apll_lock);
	if (ma35d1_audio->active_streams &&
	    ma35d1_audio->active_apll_family != family) {
		dev_err(dev,
			"hw_params: cannot switch APLL family while %u stream(s) active on a different family\n",
			ma35d1_audio->active_streams);
		mutex_unlock(&ma35d1_audio->apll_lock);
		return -EBUSY;
	}

	if (ma35d1_audio->active_streams) {
		*family_out = family;
		mutex_unlock(&ma35d1_audio->apll_lock);
		*i2s_clk = clk_get_rate(ma35d1_audio->clk);
		return 0;
	}

	old_rate = clk_get_rate(ma35d1_audio->apll_clk);
	if (old_rate != target_rate) {
		clk_disable_unprepare(ma35d1_audio->clk);
		clk_disable_unprepare(ma35d1_audio->apll_clk);

		ret = clk_set_rate(ma35d1_audio->apll_clk, target_rate);
		if (ret) {
			dev_err(dev, "failed to set APLL rate from %lu to %lu Hz: %d\n",
				old_rate, target_rate, ret);
			clk_prepare_enable(ma35d1_audio->apll_clk);
			clk_prepare_enable(ma35d1_audio->clk);
			mutex_unlock(&ma35d1_audio->apll_lock);
			return ret;
		}

		ret = clk_prepare_enable(ma35d1_audio->apll_clk);
		if (ret) {
			dev_err(dev, "failed to re-enable APLL after rate change to %lu Hz: %d\n",
				target_rate, ret);
			clk_prepare_enable(ma35d1_audio->clk);
			mutex_unlock(&ma35d1_audio->apll_lock);
			return ret;
		}

		ret = clk_prepare_enable(ma35d1_audio->clk);
		if (ret) {
			dev_err(dev, "failed to re-enable I2S gate after APLL rate change: %d\n",
				ret);
			clk_disable_unprepare(ma35d1_audio->apll_clk);
			mutex_unlock(&ma35d1_audio->apll_lock);
			return ret;
		}
	}

	*i2s_clk = clk_get_rate(ma35d1_audio->clk);
	*family_out = family;
	mutex_unlock(&ma35d1_audio->apll_lock);
	return 0;
}

static unsigned int ma35d1_i2s_calc_actual_mclk(struct ma35d1_audio *ma35d1_audio)
{
	unsigned int i2s_clk, mclkdiv;

	if (!ma35d1_audio->mclk_out)
		return 0;

	i2s_clk = clk_get_rate(ma35d1_audio->clk);
	mclkdiv = DIV_ROUND_CLOSEST(i2s_clk, 2U * ma35d1_audio->mclk_out);
	if (!mclkdiv || mclkdiv > 0x7f)
		return 0;

	return i2s_clk / (2 * mclkdiv);
}

static int ma35d1_i2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	unsigned int i2s_clk, bitrate, mclkdiv, bclkdiv;
	unsigned int sample_width, slot_width, slots, dma_width, ctl1;
	unsigned int apll_family;
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(dai->dev);
	unsigned channels = params_channels(params);
	unsigned sample_rate = params_rate(params);
	unsigned long val = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL0);
	int ret;

	sample_width = params_width(params);
	slot_width = params_physical_width(params);
	if (sample_width == 24 || sample_width == 32)
		slot_width = 32;
	else if (slot_width < sample_width)
		slot_width = sample_width;

	ret = ma35d1_i2s_configure_apll(dai->dev, ma35d1_audio, sample_rate,
					 &apll_family, &i2s_clk);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    ma35d1_audio->pdm_decimation) {
		bitrate = sample_rate * ma35d1_audio->pdm_decimation;
		slots = 1;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
		   ma35d1_audio->raw_pdm_mono && channels == 1) {
		bitrate = sample_rate * sample_width;
		slots = 1;
	} else {
		/* Standard I2S keeps left/right slots even for mono ALSA streams. */
		slots = 2;
		bitrate = sample_rate * slots * slot_width;
	}

	bclkdiv = DIV_ROUND_CLOSEST(i2s_clk, 2U * bitrate);
	if (!bclkdiv || bclkdiv > 0x400)
		return -EINVAL;
	bclkdiv -= 1;

	mclkdiv = DIV_ROUND_CLOSEST(i2s_clk, 2U * ma35d1_audio->mclk_out);
	if (!mclkdiv || mclkdiv > 0x7f)
		return -EINVAL;

	ma35d1_audio->actual_mclk_out = i2s_clk / (2 * mclkdiv);
	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CLKDIV, (bclkdiv << 8) | mclkdiv);

	switch (sample_width) {
	case 8:
		val = (val & ~DATWIDTH) | DATWIDTH_8;
		dma_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 16:
		val = (val & ~DATWIDTH) | DATWIDTH_16;
		dma_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 24:
		val = (val & ~DATWIDTH) | DATWIDTH_24;
		dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	case 32:
		val = (val & ~DATWIDTH) | DATWIDTH_32;
		dma_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}

	switch (slot_width) {
	case 8:
		val = (val & ~CHWIDTH) | CHWIDTH_8;
		break;
	case 16:
		val = (val & ~CHWIDTH) | CHWIDTH_16;
		break;
	case 24:
		val = (val & ~CHWIDTH) | CHWIDTH_24;
		break;
	case 32:
		val = (val & ~CHWIDTH) | CHWIDTH_32;
		break;
	default:
		return -EINVAL;
	}

	ctl1 = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL1);
	ctl1 &= ~(PBWIDTH | PB16ORD);
	ctl1 |= (sample_width == 16) ? PBWIDTH_16 : PBWIDTH_32;
	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL1, ctl1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ma35d1_audio->dma_params_tx.addr_width = dma_width;
	else
		ma35d1_audio->dma_params_rx.addr_width = dma_width;

	if (ma35d1_audio->raw_pdm_mono &&
		substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		val = FORMAT_MSB | ORDER;
	} else {
		if (channels == 1)
			val |= MONO;
		else
			val &= ~MONO;

		if (sample_width == 24) {
			/* S24_LE stores 24 valid bits in the low bits of a 32-bit little-endian container. */
			val &= ~ORDER;
		}
	}

	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

	mutex_lock(&ma35d1_audio->apll_lock);
	ma35d1_audio->stream_apll_family[substream->stream] = apll_family;
	mutex_unlock(&ma35d1_audio->apll_lock);

	return 0;
}

static int ma35d1_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(cpu_dai->dev);
	unsigned long val = AUDIO_READ(ma35d1_audio->mmio + I2S_CTL0);

	val &= ~FORMAT;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val |= FORMAT_I2S;
		break;
	case SND_SOC_DAIFMT_MSB:
		val |= FORMAT_MSB;
		break;
	case SND_SOC_DAIFMT_LSB:
		val |= FORMAT_LSB;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= SLAVE; //Slave
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val &= ~SLAVE; //Master
		break;
	default:
		return -EINVAL;
	}

	ma35d1_audio->actual_mclk_out = ma35d1_i2s_calc_actual_mclk(ma35d1_audio);

	AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

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
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val |= I2S_EN;
		val |= MCLKEN;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= TX_EN | TXPDMAEN;
		else
			val |= RX_EN | RXPDMAEN;

		mutex_lock(&ma35d1_audio->apll_lock);
		if (!ma35d1_audio->stream_active[substream->stream]) {
			ma35d1_audio->stream_active[substream->stream] = true;
			ma35d1_audio->active_streams++;
			ma35d1_audio->active_apll_family =
				ma35d1_audio->stream_apll_family[substream->stream];
		}
		mutex_unlock(&ma35d1_audio->apll_lock);

		AUDIO_WRITE(ma35d1_audio->mmio + I2S_CTL0, val);

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val &= ~I2S_EN;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val &= ~(TX_EN | TXPDMAEN);
		else
			val &= ~(RX_EN | RXPDMAEN);

		mutex_lock(&ma35d1_audio->apll_lock);
		if (ma35d1_audio->stream_active[substream->stream]) {
			ma35d1_audio->stream_active[substream->stream] = false;
			if (ma35d1_audio->active_streams)
				ma35d1_audio->active_streams--;
			if (!ma35d1_audio->active_streams)
				ma35d1_audio->active_apll_family =
					MA35D1_APLL_FAMILY_NONE;
		}
		mutex_unlock(&ma35d1_audio->apll_lock);

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
	ma35d1_audio->dma_params_rx.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	ma35d1_audio->pcfg_rx.reqsel = ma35d1_audio->pdma_reqsel_rx;
	ma35d1_audio->dma_params_rx.peripheral_config = &ma35d1_audio->pcfg_rx;
	ma35d1_audio->dma_params_rx.peripheral_size = sizeof(ma35d1_audio->pcfg_rx);
	ma35d1_audio->dma_params_tx.addr = ma35d1_audio->phyaddr + I2S_TXFIFO;
	ma35d1_audio->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	ma35d1_audio->pcfg_tx.reqsel = ma35d1_audio->pdma_reqsel_tx;
	ma35d1_audio->dma_params_tx.peripheral_config = &ma35d1_audio->pcfg_tx;
	ma35d1_audio->dma_params_tx.peripheral_size = sizeof(ma35d1_audio->pcfg_tx);

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
};

struct snd_soc_dai_driver ma35d1_i2s_dai = {
	.name		= "i2s_pcm",
	.probe          = ma35d1_i2s_probe,
	.remove         = ma35d1_i2s_remove,
	.playback = {
		.rates      = SNDRV_PCM_RATE_8000_192000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE |
			      SNDRV_PCM_FMTBIT_S24_LE |
			      SNDRV_PCM_FMTBIT_S32_LE,
		.channels_min   = 1,
		.channels_max   = 2,
	},
	.capture = {
		.rates      = SNDRV_PCM_RATE_8000_192000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE |
			      SNDRV_PCM_FMTBIT_S24_LE |
			      SNDRV_PCM_FMTBIT_S32_LE,
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
	u32	val32[4], mclk_out;
	u32	dma_tx_num, dma_rx_num;

	int ret;


	ma35d1_audio = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_audio), GFP_KERNEL);
	if (!ma35d1_audio)
		return -ENOMEM;

	spin_lock_init(&ma35d1_audio->lock);
	spin_lock_init(&ma35d1_audio->irqlock);
	mutex_init(&ma35d1_audio->apll_lock);

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
	pr_debug("ma35d1_audio->pdma_reqsel_tx = 0x%lx\n", (ulong)ma35d1_audio->pdma_reqsel_tx);

	if (of_property_read_u32(pdev->dev.of_node, "pdma_reqsel_rx", &dma_rx_num) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	ma35d1_audio->pdma_reqsel_rx = dma_rx_num;
	pr_debug("ma35d1_audio->pdma_reqsel_rx = 0x%lx\n", (ulong)ma35d1_audio->pdma_reqsel_rx);

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

	ma35d1_audio->apll_clk = devm_clk_get(&pdev->dev, "apll");
	if (IS_ERR(ma35d1_audio->apll_clk)) {
		dev_err(&pdev->dev, "failed to get APLL clock\n");
		ret = PTR_ERR(ma35d1_audio->apll_clk);
		goto out2;
	}

	clk_prepare_enable(ma35d1_audio->clk);

	ma35d1_audio->irq_num = platform_get_irq(pdev, 0);
	if (!ma35d1_audio->irq_num) {
		dev_err(&pdev->dev, "platform_get_irq error\n");
		ret = -EBUSY;
		goto out3;
	}

	if (of_property_read_u32(pdev->dev.of_node, "mclk_out", &mclk_out) != 0)
		ma35d1_audio->mclk_out = 12000000;
	else
		ma35d1_audio->mclk_out = mclk_out;

	of_property_read_u32(pdev->dev.of_node, "pdm-decimation",
			     &ma35d1_audio->pdm_decimation);
	if (ma35d1_audio->pdm_decimation &&
	    ma35d1_audio->pdm_decimation != 64 &&
	    ma35d1_audio->pdm_decimation != 128)
		return -EINVAL;

	ma35d1_audio->raw_pdm_mono = of_property_read_bool(pdev->dev.of_node,
							   "nuvoton,raw-pdm-mono");

	dev_set_drvdata(&pdev->dev, ma35d1_audio);

	ret = devm_snd_soc_register_component(&pdev->dev, &ma35d1_i2s_component,
						&ma35d1_i2s_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC DAI\n");
		goto out3;
	}

#ifdef CONFIG_SND_SIMPLE_CARD
	ret = ma35d1_dma_pcm_register(&pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
#endif
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
	struct ma35d1_audio *ma35d1_audio = dev_get_drvdata(&pdev->dev);

#ifdef CONFIG_SND_SIMPLE_CARD
	ma35d1_dma_pcm_unregister(&pdev->dev);
#endif
	snd_soc_unregister_component(&pdev->dev);

	clk_put(ma35d1_audio->clk);
	iounmap(ma35d1_audio->mmio);
	release_mem_region(ma35d1_audio->res->start, resource_size(ma35d1_audio->res));

	kfree(ma35d1_audio);

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

