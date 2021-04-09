// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <linux/spi/spi.h>

#include <asm/irq.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/platform_data/dma-ma35d1.h>

#define PCLK_FREQ	189000000
#define QSPI0_BASEADDR	0x40680000
#define QSPI1_BASEADDR	0x40690000

/* spi registers offset */
#define REG_CTL		0x00
#define REG_CLKDIV	0x04
#define REG_SSCTL	0x08
#define REG_PDMACTL	0x0C
#define REG_FIFOCTL	0x10
#define REG_STATUS	0x14
#define REG_TX		0x20
#define REG_RX		0x30

/* spi register bit */
#define QUADIOEN	(0x01 << 22)
#define DATDIR		(0x01 << 20)
#define UNITIEN		(0x01 << 17)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
#define LSB		(0x01 << 13)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 3)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	0x02
#define SPIEN		0x01
#define DWIDTH_MASK	0x1F00
#define DWIDTH_POS	8
#define BYTE_REORDER	0x80000
#define TXPDMAEN	0x01
#define RXPDMAEN	0x02
#define RXRST		0x01
#define TXRST		0x02
#define RXFBCLR		0x100
#define TXFBCLR		0x200
#define BUSY		0x01
#define RXEMPTY		0x100
#define RXFULL		0x200
#define SPIENSTS	0x8000
#define TXEMPTY		0x10000
#define TXFULL		0x20000
#define FIFOCLR		0x400000
#define TXRXRST		0x800000

/* define for PDMA */

#define ALIGNMENT_4	4
#define USE_PDMA_LEN	16

struct ma35d1_ip_dma {
	struct dma_chan			*chan_rx;
	struct dma_chan			*chan_tx;
	struct scatterlist		sgrx;
	struct scatterlist		sgtx;
	struct dma_async_tx_descriptor	*rxdesc;
	struct dma_async_tx_descriptor	*txdesc;
	struct dma_slave_config		slave_config;
};

struct ma35d1_qspi_info {
	unsigned int num_cs;
	unsigned int lsb;
	unsigned int txneg;
	unsigned int rxneg;
	unsigned int divider;
	unsigned int sleep;
	unsigned int txbitlen;
	unsigned int clkpol;
	int bus_num;
	unsigned int hz;
	unsigned int quad;
	unsigned int use_pdma;
	unsigned int pdma_reqsel_tx;
	unsigned int pdma_reqsel_rx;
};

struct ma35d1_qspi {
	struct spi_bitbang		bitbang;
	struct completion		done;
	void __iomem			*regs;
	int				irq;
	unsigned int			len;
	unsigned int			count;
	const void			*tx;
	void				*rx;
	struct clk			*clk;
	struct spi_master		*master;
	struct device			*dev;
	struct ma35d1_qspi_info	*pdata;
	spinlock_t			lock;
	struct resource			*res;
	struct ma35d1_dma_done		dma_slave_done;
	struct ma35d1_ip_dma		dma;
	volatile int			slave_done_state;
	struct wait_queue_head		slave_done;
	unsigned int			phyaddr;
};

static inline struct ma35d1_qspi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void ma35d1_slave_dma_callback(void *arg)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)arg;
	struct ma35d1_dma_done *done = &hw->dma_slave_done;

	done->done = true;
	hw->slave_done_state = 1;
	wake_up_interruptible(&hw->slave_done);

	return;
}

static inline void ma35d1_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)to_hw(spi);
	unsigned int val;
	unsigned int cs = spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_SSCTL);

	if (!cs)
		val &= ~SELECTLEV;
	else
		val |= SELECTLEV;

	if(spi->chip_select == 0) {
		if (!ssr)
			val &= ~SELECTSLAVE0;
		else
			val |= SELECTSLAVE0;
	} else {
		if (!ssr)
			val &= ~SELECTSLAVE1;
		else
			val |= SELECTSLAVE1;
	}

	while (__raw_readl(hw->regs + REG_STATUS) & BUSY); //wait busy

	__raw_writel(val, hw->regs + REG_SSCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_qspi_chipsel(struct spi_device *spi, int value)
{
	switch (value) {
	case BITBANG_CS_INACTIVE:
		ma35d1_slave_select(spi, 0);
		break;

	case BITBANG_CS_ACTIVE:
		ma35d1_slave_select(spi, 1);
		break;
	}
}

static inline void ma35d1_setup_txbitlen(struct ma35d1_qspi *hw,
        unsigned int txbitlen)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);
	val &= ~0x1f00;
	if(txbitlen != 32)
		val |= (txbitlen << 8);

	__raw_writel(val, hw->regs + REG_CTL);

}

static inline unsigned int hw_tx(struct ma35d1_qspi *hw, unsigned int count)
{
	const unsigned char *tx_byte = hw->tx;
	const unsigned short *tx_short = hw->tx;
	const unsigned int *tx_int = hw->tx;
	unsigned int bwp = hw->pdata->txbitlen;

	if(bwp <= 8)
		return tx_byte ? tx_byte[count] : 0;
	else if(bwp <= 16)
		return tx_short ? tx_short[count] : 0;
	else
		return tx_int ? tx_int[count] : 0;
}

static inline void hw_rx(struct ma35d1_qspi *hw, unsigned int data, int count)
{
	unsigned char *rx_byte = hw->rx;
	unsigned short *rx_short = hw->rx;
	unsigned int *rx_int = hw->rx;
	int bwp = hw->pdata->txbitlen;

	if(bwp <= 8)
		rx_byte[count] = data;
	else if(bwp <= 16)
		rx_short[count] = data;
	else
		rx_int[count] = data;
}

static int ma35d1_qspi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)to_hw(spi);
	unsigned int	i,j;
	unsigned int	non_align_len = 0;

	struct ma35d1_ip_dma *pdma = &hw->dma;
	dma_cookie_t	cookie;

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST), hw->regs + REG_FIFOCTL);
	while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST) {}

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;

	/* Use PDMA or not from device tree */
	if (hw->pdata->use_pdma) {

		/* short length transfer by CPU */
		if (t->len < USE_PDMA_LEN) {
			if (t->rx_nbits & SPI_NBITS_QUAD) {
				__raw_writel(((__raw_readl(hw->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
				             hw->regs + REG_CTL); //Enable Quad mode, direction input
			}

			if (hw->rx) {
				j = 0;

				for(i = 0; i < t->len; ) {
					if(((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
						__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
						i++;
					}
					if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}
				}
				while(j < t->len) {
					if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}
				}
			} else {
				for(i = 0; i < t->len; i++) {
					while (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == TXFULL)); //TXFULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
				}
			}

			__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~(QUADIOEN | DATDIR)),
			             hw->regs + REG_CTL); //Restore to single mode, direction input

		} else { //Long length transfer by PDMA

			/* Since MA35D1 PDMA cannot transfer data at non-alignment address */
			/* Transfer the non-alignment byte by CPU */
			if ((unsigned long)t->rx_buf & (ALIGNMENT_4 - 1)) {
				j = 0;
				non_align_len = (ALIGNMENT_4 - ((unsigned long)t->rx_buf & (ALIGNMENT_4 - 1)));

				for (i = 0; (i < t->len) && (i < non_align_len); ) {
					if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
						__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
						i++;
					}
					if (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}

				}
				while (j < non_align_len) {
					if (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}
				}
			} else if ((unsigned long)t->tx_buf & (ALIGNMENT_4 - 1)) {
				non_align_len = (ALIGNMENT_4 - ((unsigned long)t->tx_buf & (ALIGNMENT_4 - 1)));
				for (i = 0; (i < t->len) && (i < non_align_len); )
					if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) != TXFULL)) { //TX NOT FULL
						__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
						i++;
					}
			}

			/* Transfer the rest data starting at alignment address by PDMA */
			if (t->rx_buf) {

				if(t->rx_nbits & SPI_NBITS_QUAD)
					__raw_writel(((__raw_readl(hw->regs + REG_CTL) | QUADIOEN) & ~(DATDIR)),
					             hw->regs + REG_CTL); //Enable Quad mode, direction input


				/* prepare the RX dma transfer */
				sg_init_table(&pdma->sgrx, 1);
				pdma->slave_config.src_addr = (hw->phyaddr + REG_RX);
				pdma->slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
				pdma->sgrx.dma_length = (t->len - non_align_len);

				pdma->slave_config.direction = DMA_DEV_TO_MEM;
				pdma->slave_config.slave_id = hw->pdata->pdma_reqsel_rx;
				dmaengine_slave_config(pdma->chan_rx, &(pdma->slave_config));

				/* Map t->rx_buf to physical address including non-alignment part */
				pdma->sgrx.dma_address = dma_map_single(hw->dev,
				                                        (void *)(t->rx_buf),
				                                        t->len, DMA_FROM_DEVICE);
				if (dma_mapping_error(hw->dev, pdma->sgrx.dma_address)) {
					dev_err(hw->dev, "tx dma map error\n");
				}

				/* Adjust physical address that skip non-alignment part */
				pdma->sgrx.dma_address += non_align_len;

				pdma->rxdesc = dmaengine_prep_slave_sg(pdma->chan_rx,
				                                       &pdma->sgrx,
				                                       1,
				                                       DMA_FROM_DEVICE,
				                                       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
				if (!pdma->rxdesc) {
					dev_err(hw->dev, "pdma->rxdesc=NULL\n");
					BUG();
				}
				hw->dma_slave_done.done = false;
				pdma->rxdesc->callback = ma35d1_slave_dma_callback;
				pdma->rxdesc->callback_param = hw;

				cookie = pdma->rxdesc->tx_submit(pdma->rxdesc);
				if (dma_submit_error(cookie)) {
					dev_err(hw->dev, "rx cookie=%d\n",cookie);
					BUG();
				}
			} /* t->rx_buf */

			if (t->tx_buf) {
				/* prepare the TX dma transfer */
				sg_init_table(&pdma->sgtx, 1);
				pdma->slave_config.dst_addr = (hw->phyaddr + REG_TX);
				pdma->slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
				pdma->sgtx.dma_length = t->len - non_align_len;

				pdma->slave_config.direction = DMA_MEM_TO_DEV;
				pdma->slave_config.slave_id = hw->pdata->pdma_reqsel_tx;
				dmaengine_slave_config(pdma->chan_tx, &(pdma->slave_config));

				/* Map t->tx_buf to physical address including non-alignment part */
				pdma->sgtx.dma_address = dma_map_single(hw->dev,
				                                        (void *)t->tx_buf,
				                                        t->len, DMA_TO_DEVICE);
				if (dma_mapping_error(hw->dev, pdma->sgtx.dma_address)) {
					dev_err(hw->dev, "tx dma map error\n");
				}
				/* Adjust physical address that skip non-alignment part */
				pdma->sgtx.dma_address += non_align_len;

				dma_sync_single_for_device(hw->dev, pdma->sgtx.dma_address - non_align_len, t->len, DMA_TO_DEVICE);

				pdma->txdesc = dmaengine_prep_slave_sg(pdma->chan_tx,
				                                       &pdma->sgtx,
				                                       1,
				                                       DMA_TO_DEVICE,
				                                       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
				if (!pdma->txdesc) {
					dev_err(hw->dev, "pdma->txdex=NULL\n");
					BUG();
				}

				pdma->txdesc->callback = ma35d1_slave_dma_callback;
				pdma->txdesc->callback_param = hw;


				cookie = pdma->txdesc->tx_submit(pdma->txdesc);
				if (dma_submit_error(cookie)) {
					dev_err(hw->dev, "tx cookie=%d\n",cookie);
					BUG();
				}
			} /* t->tx_buf */

			/* TODO: TC5814 limitation, reset SPI TXFIFO write pointer and
			 * RXFIFO read pointer before enabling TX/RX PDMA */
			if (non_align_len) {
				__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXFBCLR | RXFBCLR), hw->regs + REG_FIFOCTL);
				while (__raw_readl(hw->regs + REG_STATUS) & FIFOCLR) {}
			}

			/* Trigger PDMA */
			if (t->rx_buf)
				__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) | (RXPDMAEN),
				             hw->regs + REG_PDMACTL); //Enable SPIx RX PDMA
			else
				__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) | (TXPDMAEN),
				             hw->regs + REG_PDMACTL); //Enable SPIx TX PDMA

			wait_event_interruptible(hw->slave_done, (hw->slave_done_state != 0));

			hw->slave_done_state=0;

			while (__raw_readl(hw->regs + REG_STATUS) & BUSY); //wait busy

			__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~SPIEN), hw->regs + REG_CTL); //Disable SPIEN
			__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) & ~(TXPDMAEN | RXPDMAEN),
			             hw->regs + REG_PDMACTL); //Disable SPIx TX/RX PDMA
			__raw_writel(((__raw_readl(hw->regs + REG_CTL) & ~(DWIDTH_MASK | BYTE_REORDER))
			              | (8 << DWIDTH_POS)), hw->regs + REG_CTL); //8 bits, no byte reorder
			__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~(QUADIOEN | DATDIR)),
			             hw->regs + REG_CTL); //Disable Quad mode, direction input
			__raw_writel((__raw_readl(hw->regs + REG_CTL) | SPIEN), hw->regs + REG_CTL); //Enable SPIEN


			/* unmap buffers if mapped above, restore map address that includes non-alignment part */
			if (t->rx_buf)
				dma_unmap_single(hw->dev, pdma->sgrx.dma_address - non_align_len, t->len,
				                 DMA_FROM_DEVICE);
			if (t->tx_buf)
				dma_unmap_single(hw->dev, pdma->sgtx.dma_address - non_align_len, t->len,
				                 DMA_TO_DEVICE);
		}

	} else { /* hw->pdata->use_pdma = 0 */

		if(t->rx_nbits & SPI_NBITS_QUAD) {
			__raw_writel(((__raw_readl(hw->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
			             hw->regs + REG_CTL);//Enable Quad mode, direction input
		}

		if (hw->rx) {
			j = 0;

			for(i = 0; i < t->len; ) {
				if(((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
					i++;
				} else {
					cond_resched();
				}
				if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
					hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
					j++;
				} else {
					cond_resched();
				}
			}
			while(j < t->len) {
				if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
					hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
					j++;
				} else {
					cond_resched();
				}
			}
		} else {
			for(i = 0; i < t->len; ) {
				if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
					i++;
				} else {
					cond_resched();
				}
			}
		}

		__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~(QUADIOEN | DATDIR)),
		             hw->regs + REG_CTL); //Restore to single mode, direction input

	}
	return t->len;
}


static inline void ma35d1_set_clock_polarity(struct ma35d1_qspi *hw, unsigned int polarity)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void ma35d1_tx_rx_edge(struct ma35d1_qspi *hw, unsigned int tx_edge, unsigned int rx_edge)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (tx_edge)
		val |= TXNEG;
	else
		val &= ~TXNEG;

	if (rx_edge)
		val |= RXNEG;
	else
		val &= ~RXNEG;

	__raw_writel(val, hw->regs + REG_CTL);

}

static inline void ma35d1_send_first(struct ma35d1_qspi *hw, unsigned int lsb)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void ma35d1_set_sleep(struct ma35d1_qspi *hw, unsigned int sleep)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	val &= ~(0x0f << 4);

	if (sleep)
		val |= (sleep << 4);

	__raw_writel(val, hw->regs + REG_CTL);
}


static inline void ma35d1_set_divider(struct ma35d1_qspi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_CLKDIV);
}

static int ma35d1_qspi_update_state(struct spi_device *spi,
                                     struct spi_transfer *t)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)to_hw(spi);
	unsigned int clk;
	unsigned int div;
	unsigned int bpw;
	unsigned int hz;
	unsigned char spimode;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if(hw->pdata->txbitlen != bpw)
		hw->pdata->txbitlen = bpw;

	if(hw->pdata->hz != hz) {
#if 0 /* TODO: turn on this after design fix */
		clk = clk_get_rate(hw->clk);
#else
		clk = PCLK_FREQ;
#endif
		div = DIV_ROUND_UP(clk, hz) - 1;
		hw->pdata->hz = hz;
		hw->pdata->divider = div;
	}

	//Mode 0: CPOL=0, CPHA=0; active high
	//Mode 1: CPOL=0, CPHA=1; active low
	//Mode 2: CPOL=1, CPHA=0; active low
	//Mode 3: CPOL=1, CPHA=1; active high
	if (spi->mode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	spimode = spi->mode & 0xff; //remove dual/quad bit

	if ((spimode == SPI_MODE_0) || (spimode == SPI_MODE_3)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	if (spi->mode & SPI_LSB_FIRST)
		hw->pdata->lsb = 1;
	else
		hw->pdata->lsb = 0;

	return 0;
}

static int ma35d1_qspi_setupxfer(struct spi_device *spi,
                                  struct spi_transfer *t)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)to_hw(spi);
	unsigned long flags;
	int ret;

	ret = ma35d1_qspi_update_state(spi, t);
	if (ret)
		return ret;

	spin_lock_irqsave(&hw->lock, flags);

	ma35d1_setup_txbitlen(hw, hw->pdata->txbitlen);
	ma35d1_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	ma35d1_set_clock_polarity(hw, hw->pdata->clkpol);
	ma35d1_send_first(hw, hw->pdata->lsb);
	ma35d1_set_divider(hw);

	spin_unlock_irqrestore(&hw->lock, flags);

	return 0;
}

static int ma35d1_qspi_setup(struct spi_device *spi)
{
	struct ma35d1_qspi *hw = (struct ma35d1_qspi *)to_hw(spi);
	int ret;

	ret = ma35d1_qspi_update_state(spi, NULL);
	if (ret)
		return ret;

	mutex_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		ma35d1_set_divider(hw);
		ma35d1_slave_select(spi, 0);
	}
	mutex_unlock(&hw->bitbang.lock);

	return 0;
}

static void ma35d1_init_spi(struct ma35d1_qspi *hw)
{
	unsigned long flags;

	spin_lock_init(&hw->lock);

	spin_lock_irqsave(&hw->lock, flags);

	ma35d1_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	ma35d1_send_first(hw, hw->pdata->lsb);
	ma35d1_set_sleep(hw, hw->pdata->sleep);
	ma35d1_setup_txbitlen(hw, hw->pdata->txbitlen);
	ma35d1_set_clock_polarity(hw, hw->pdata->clkpol);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static struct ma35d1_qspi_info *ma35d1_qspi_parse_dt(struct device *dev)
{
	struct ma35d1_qspi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci) {
		dev_err(dev, "memory allocation for spi_info failed\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(dev->of_node, "num_cs", &temp)) {
		dev_warn(dev, "can't get num_cs from dt\n");
		sci->num_cs = 2;
	} else {
		sci->num_cs = temp;
	}

	if (of_property_read_u32(dev->of_node, "lsb", &temp)) {
		dev_warn(dev, "can't get lsb from dt\n");
		sci->lsb = 0;
	} else {
		sci->lsb = temp;
	}

	if (of_property_read_u32(dev->of_node, "sleep", &temp)) {
		dev_warn(dev, "can't get sleep from dt\n");
		sci->sleep = 0;
	} else {
		sci->sleep = temp;
	}

	if (of_property_read_u32(dev->of_node, "txbitlen", &temp)) {
		dev_warn(dev, "can't get txbitlen from dt\n");
		sci->txbitlen = 8;
	} else {
		sci->txbitlen = temp;
	}

	if (of_property_read_u32(dev->of_node, "bus_num", &temp)) {
		dev_warn(dev, "can't get bus_num from dt\n");
		sci->bus_num = 0;
	} else {
		sci->bus_num = temp;
	}

	if (of_property_read_u32(dev->of_node, "quad", &temp)) {
		dev_warn(dev, "can't get quad from dt\n");
		sci->quad = 0;
	} else {
		sci->quad = temp;
	}

	if (of_property_read_u32(dev->of_node, "use_pdma", &temp)) {
		dev_warn(dev, "can't get use_pdma from dt\n");
		sci->use_pdma = 0;
	} else {
		sci->use_pdma = temp;
	}

	if (of_property_read_u32(dev->of_node, "pdma_reqsel_tx", &temp)) {
		dev_warn(dev, "can't get pdma_reqsel_tx from dt\n");
		sci->pdma_reqsel_tx = 0;
	} else {
		sci->pdma_reqsel_tx = temp;
	}

	if (of_property_read_u32(dev->of_node, "pdma_reqsel_rx", &temp)) {
		dev_warn(dev, "can't get pdma_reqsel_rx from dt\n");
		sci->pdma_reqsel_rx = 0;
	} else {
		sci->pdma_reqsel_rx = temp;
	}

	return sci;
}

static int ma35d1_qspi_probe(struct platform_device *pdev)
{
	struct ma35d1_ip_dma *pdma;
	dma_cap_mask_t mask;
	struct ma35d1_qspi *hw;
	struct spi_master *master;
	u32   val32[4];
	int err = 0;
	int status = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ma35d1_qspi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	hw->master = spi_master_get(master);
	hw->pdata = ma35d1_qspi_parse_dt(&pdev->dev);
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	if (hw->pdata->use_pdma) {
		/* Zero out the capability mask then initialize it for a slave channel that is
		 * private.
		 */
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		dma_cap_set(DMA_PRIVATE, mask);

		/* Initailize wiat queue head as  __WAIT_QUEUE_HEAD_INITIALIZER() */
		hw->slave_done.lock           = __SPIN_LOCK_UNLOCKED(hw->slave_done.lock);
		hw->slave_done.head.next       = &(hw->slave_done).head;
		hw->slave_done.head.prev       = &(hw->slave_done).head;

		/* Request the DMA channel from the DMA engine and then use the device from
		 * the channel for the proxy channel also.
		 */
		pdma = &hw->dma;
		pdma->chan_rx = dma_request_channel(mask, NULL, NULL);
		if (!pdma->chan_rx) {
			dev_err(&pdev->dev, "RX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("RX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_rx));

		pdma->chan_tx = dma_request_channel(mask, NULL, NULL);
		if (!pdma->chan_tx) {
			dev_err(&pdev->dev, "TX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("TX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_tx));
	}

	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);
	if (hw->pdata->quad == 0)
		master->mode_bits	= (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_CS_HIGH
		                       | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);
	else
		master->mode_bits	= (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_CS_HIGH
		                       | SPI_RX_QUAD | SPI_TX_QUAD | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);

	master->dev.of_node        = pdev->dev.of_node;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = ma35d1_qspi_setupxfer;
	hw->bitbang.chipselect     = ma35d1_qspi_chipsel;
	hw->bitbang.txrx_bufs      = ma35d1_qspi_txrx;
	hw->bitbang.master->setup  = ma35d1_qspi_setup;

	if (of_property_read_u32_array(pdev->dev.of_node, "reg", val32, 4) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	hw->phyaddr = val32[1];
	pr_debug("hw->phyaddr = 0x%lx\n",(ulong)hw->phyaddr);

	hw->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (hw->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

	hw->regs = devm_ioremap_resource(&pdev->dev, hw->res);
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_pdata;
	}

	hw->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "unable to get SYS clock, err=%d\n",
		        status);
		goto err_clk;
	}
	clk_prepare_enable(hw->clk);

	ma35d1_init_spi(hw);

	__raw_writel(__raw_readl(hw->regs + REG_CTL) | SPIEN, hw->regs + REG_CTL); /* enable SPI */
	while ((__raw_readl(hw->regs + REG_STATUS) & (1<<15)) == 0); //SPIENSTS

	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:
	clk_disable(hw->clk);
	clk_put(hw->clk);
err_clk:
	free_irq(hw->irq, hw);
err_pdata:
	spi_master_put(hw->master);

err_nomem:
	return err;
}

static int ma35d1_qspi_remove(struct platform_device *dev)
{
	struct ma35d1_qspi *hw = platform_get_drvdata(dev);

	free_irq(hw->irq, hw);
	platform_set_drvdata(dev, NULL);
	spi_bitbang_stop(&hw->bitbang);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM
static int ma35d1_qspi_suspend(struct device *dev)
{
	struct ma35d1_qspi *hw = dev_get_drvdata(dev);

	while (__raw_readl(hw->regs + REG_STATUS) & 1) //wait busy
		msleep(1);

	// disable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~0x20000), hw->regs + REG_CTL);

	return 0;
}

static int ma35d1_qspi_resume(struct device *dev)
{
	struct ma35d1_qspi *hw = dev_get_drvdata(dev);

	// enable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) | 0x20000), hw->regs + REG_CTL);

	return 0;
}

static const struct dev_pm_ops ma35d1_qspi_pmops = {
	.suspend    = ma35d1_qspi_suspend,
	.resume     = ma35d1_qspi_resume,
};

#define MA35D1_QSPI_PMOPS (&ma35d1_qspi_pmops)

#else
#define MA35D1_QSPI_PMOPS NULL
#endif

static const struct of_device_id ma35d1_qspi_of_match[] = {
	{   .compatible = "nuvoton,ma35d1-qspi" },
	{	},
};
MODULE_DEVICE_TABLE(of, ma35d1_qspi_of_match);

static struct platform_driver ma35d1_qspi_driver = {
	.probe      = ma35d1_qspi_probe,
	.remove     = ma35d1_qspi_remove,
	.driver     = {
		.name   = "ma35d1-qspi",
		.owner  = THIS_MODULE,
		.pm	= MA35D1_QSPI_PMOPS,
		.of_match_table = of_match_ptr(ma35d1_qspi_of_match),
	},
};
module_platform_driver(ma35d1_qspi_driver);

MODULE_DESCRIPTION("ma35d1 qspi driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-qspi");
