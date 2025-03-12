/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#ifndef _MA35D1_AUDIO_H
#define _MA35D1_AUDIO_H

#include <linux/gpio/consumer.h>
#include <sound/dmaengine_pcm.h>
#include <linux/platform_data/dma-ma35d1.h>
#include <linux/io.h>

#define IN	0
#define OUT	1

/* Audio Control Registers */
#define I2S_CTL0		0x00
#define I2S_CLKDIV		0x04
#define I2S_IEN			0x08
#define I2S_STATUS0		0x0C
#define I2S_TXFIFO		0x10
#define I2S_RXFIFO		0x14
#define I2S_CTL1		0x20
#define I2S_STATUS1		0x24

/* bit definition of I2S_CTL0 register */
#define TDMCHNUM		0xC0000000
#define CHWIDTH			0x30000000
#define CHWIDTH_8		0x00000000
#define CHWIDTH_16		0x10000000
#define CHWIDTH_24		0x20000000
#define CHWIDTH_32		0x30000000
#define PCMSYNC			0x8000000
#define FORMAT			0x7000000
#define RXLCH			0x800000
#define RXPDMAEN		0x200000
#define TXPDMAEN		0x100000
#define RXFBCLR			0x80000
#define TXFBCLR			0x40000
#define MCLKEN			0x8000
#define SLAVE			0x0100
#define ORDER			0x0080
#define MONO			0x40
#define DATWIDTH_32		0x30
#define DATWIDTH_24		0x20
#define DATWIDTH_16		0x10
#define DATWIDTH_8		0x00
#define DATWIDTH		0x0030
#define MUTE			0x0008
#define RX_EN			0x0004
#define TX_EN			0x0002
#define I2S_EN			0x0001


/* bit definition of I2S_CTL1 register */
#define PB16ORD			0x2000000
#define PBWIDTH_16		0x1000000
#define PBWIDTH_32		0x0000000
#define RXTH			0x70000
#define TXTH			0x00700
#define CH7ZCEN			0x00080
#define CH6ZCEN			0x00040
#define CH5ZCEN			0x00020
#define CH4ZCEN			0x00010
#define CH3ZCEN			0x00008
#define CH2ZCEN			0x00004
#define CH1ZCEN			0x00002
#define CH0ZCEN			0x00001

/* bit definition of I2S_CLKDIV register */
#define MCLKDIV			0x3FF00
#define BCLKDIV			0x7F

/* bit definition of I2S_IEN register */
#define CH7ZCIEN		0x800000
#define CH6ZCIEN		0x400000
#define CH5ZCIEN		0x200000
#define CH4ZCIEN		0x100000
#define CH3ZCIEN		0x80000
#define CH2ZCIEN		0x40000
#define CH1ZCIEN		0x20000
#define CH0ZCIEN		0x10000
#define TXTHIEN			0x400
#define TXOVFIEN		0x200
#define TXUDFIEN		0x100
#define RXTHIEN			0x04
#define RXOVFIEN		0x02
#define RXUDFIEN		0x01

/* bit definition of I2S_STATUS0 register */
#define TXBUSY			0x200000
#define TXEMPTY			0x100000
#define TXFULL			0x80000
#define TXTHIF			0x40000
#define TXOVIF			0x20000
#define TXUDIF			0x10000
#define RXEMPTY			0x1000
#define RXFULL			0x800
#define RXTHIF			0x400
#define RXOVIF			0x200
#define RXUDIF			0x100
#define DATACH			0x38
#define I2STXINT		0x04
#define I2SRXINT		0x02
#define I2SINT			0x01

/* bit definition of I2S_STATUS1 register */
#define RXCNT			0x1F0000
#define TXCNT			0x1F00
#define CH7ZCIF			0x80
#define CH6ZCIF			0x40
#define CH5ZCIF			0x20
#define CH4ZCIF			0x10
#define CH3ZCIF			0x8
#define CH2ZCIF			0x4
#define CH1ZCIF			0x2
#define CH0ZCIF			0x1

#define MA35D1_AUDIO_SAMPLECLK  0x00
#define MA35D1_AUDIO_CLKDIV     0x01

#define CODEC_READY     0x10
#define RESET_PRSR      0x00
#define AUDIO_WRITE(addr, val)  __raw_writel(val, addr)
#define AUDIO_READ(addr)    __raw_readl(addr)

struct ma35d1_audio {
	void __iomem *mmio;
	spinlock_t irqlock, lock;
	dma_addr_t dma_addr[2];
	unsigned long buffersize[2];
	unsigned long irq_num;
	struct snd_pcm_substream *substream[2];
	struct resource *res;
	struct clk *clk;
	struct device *dev;
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
	unsigned int pos;
	struct dma_async_tx_descriptor *desc[2];
	struct dma_chan *dma_chan[2];
	struct snd_dmaengine_dai_dma_data *dma_data;
	unsigned int pdma_reqsel_tx;
	unsigned int pdma_reqsel_rx;
	struct ma35d1_peripheral pcfg_tx;
	struct ma35d1_peripheral pcfg_rx;
	unsigned int phyaddr;
	unsigned int mclk_out;
	struct gpio_desc *pwdn_gpio;

};

extern struct ma35d1_audio *ma35d1_i2s_data;

int ma35d1_dma_create(struct ma35d1_audio *ma35d1_audio);
int ma35d1_dma_destroy(struct ma35d1_audio *ma35d1_audio);

#define DRV_NAME "ma35d1-dai"

#endif /*end _MA35D1_AUDIO_H */
