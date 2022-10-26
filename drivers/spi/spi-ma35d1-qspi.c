// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#include <linux/interrupt.h>
#include <linux/of.h>
#include <asm/irq.h>
#include <linux/platform_data/dma-ma35d1.h>

#define QSPI0_BASEADDR  0x40680000
#define QSPI1_BASEADDR  0x40690000

/* spi registers offset */
#define REG_CTL         0x00
#define REG_CLKDIV      0x04
#define REG_SSCTL       0x08
#define REG_PDMACTL     0x0C
#define REG_FIFOCTL     0x10
#define REG_STATUS      0x14
#define REG_TX          0x20
#define REG_RX          0x30
#define REG_INTERNAL    0x48

/* spi register bit */
#define QUADIOEN        (0x01 << 22)
#define DATDIR          (0x01 << 20)
#define UNITIEN         (0x01 << 17)
#define TXNEG           (0x01 << 2)
#define RXNEG           (0x01 << 1)
#define LSB             (0x01 << 13)
#define SELECTLEV       (0x01 << 2)
#define SELECTPOL       (0x01 << 3)
#define SELECTSLAVE0    0x01
#define SELECTSLAVE1    0x02
#define SPIEN           0x01
#define DWIDTH_MASK     0x1F00
#define DWIDTH_POS      8
#define BYTE_REORDER    0x80000
#define TXPDMAEN        0x01
#define RXPDMAEN        0x02
#define RXRST           0x01
#define TXRST           0x02
#define RXFBCLR         0x100
#define TXFBCLR         0x200
#define BUSY            0x01
#define RXEMPTY         0x100
#define RXFULL          0x200
#define SPIENSTS        0x8000
#define TXEMPTY         0x10000
#define TXFULL          0x20000
#define FIFOCLR         0x400000
#define TXRXRST         0x800000

#define OP_BUSW_1	0
#define OP_BUSW_2	1
#define OP_BUSW_4	2

/* define for PDMA */
#define ALIGNMENT_4     4
#define USE_PDMA_LEN    100

struct ma35d1_ip_dma {
	struct dma_chan                 *chan_rx;
	struct dma_chan                 *chan_tx;
	struct scatterlist              sgrx;
	struct scatterlist              sgtx;
	struct dma_async_tx_descriptor  *rxdesc;
	struct dma_async_tx_descriptor  *txdesc;
	struct dma_slave_config         slave_config;
};

struct nuvoton_qspi_info {
	unsigned int num_cs;
	unsigned int lsb;
	unsigned int txneg;
	unsigned int rxneg;
	unsigned int divider;
	unsigned int sleep;
	unsigned int txbitlen;
	unsigned int clkpol;
	int bus_num;
	unsigned int spimode;
	unsigned int hz;
	unsigned int quad;
	unsigned int use_pdma;
	unsigned int mrxphase;
	unsigned int pdma_reqsel_tx;
	unsigned int pdma_reqsel_rx;
};

struct nuvoton_spi {
	struct completion               done;
	void __iomem                    *regs;
	int                             irq;
	unsigned int                    len;
	unsigned int                    count;
	const void                      *tx;
	void                            *rx;
	struct clk                      *clk;
	struct spi_master               *master;
	struct device                   *dev;
	struct nuvoton_qspi_info	*pdata;
	spinlock_t                      lock;
	struct resource                 *res;
	struct ma35d1_dma_done          dma_slave_done;
	struct ma35d1_ip_dma            dma;
	volatile int                    slave_done_state;
	struct wait_queue_head          slave_done;
	unsigned int                    phyaddr;
	unsigned int			cur_speed_hz;
};

static inline struct nuvoton_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void ma35d1_slave_dma_callback(void *arg)
{
	struct nuvoton_spi *hw = (struct nuvoton_spi *)arg;
	struct ma35d1_dma_done *done = &hw->dma_slave_done;

	done->done = true;
	hw->slave_done_state = 1;
	wake_up_interruptible(&hw->slave_done);

	return;
}

static int nuvoton_spi_clk_enable(struct nuvoton_spi *nuvoton)
{
	int ret;

	ret = clk_prepare_enable(nuvoton->clk);

	return ret;
}

static void nuvoton_spi_clk_disable(struct nuvoton_spi *nuvoton)
{
	clk_disable_unprepare(nuvoton->clk);
}

static inline void nuvoton_set_divider(struct nuvoton_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_CLKDIV);
}

static int nuvoton_spi_clk_setup(struct nuvoton_spi *hw, unsigned long freq)
{
	unsigned int clk;
	unsigned int div;

	clk = clk_get_rate(hw->clk);
	div = DIV_ROUND_UP(clk, freq) - 1;
	hw->pdata->hz = freq;
	hw->pdata->divider = div;

	nuvoton_set_divider(hw);

	return 0;
}

static inline void nuvoton_setup_txbitlen(struct nuvoton_spi *hw,
        unsigned int txbitlen)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);
	val &= ~0x1f00;
	if(txbitlen != 32)
		val |= (txbitlen << 8);

	__raw_writel(val, hw->regs + REG_CTL);

}

static inline void nuvoton_set_clock_polarity(struct nuvoton_spi *hw, unsigned int polarity)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CTL);
}

static int nuvoton_spi_set_freq(struct nuvoton_spi *nuvoton, unsigned long freq)
{
	int ret;

	if (nuvoton->cur_speed_hz == freq)
		return 0;

	ret = nuvoton_spi_clk_setup(nuvoton, freq);
	if (ret)
		return ret;

	nuvoton->cur_speed_hz = freq;

	return 0;
}

static inline void nuvoton_tx_rx_edge(struct nuvoton_spi *hw, unsigned int tx_edge, unsigned int rx_edge)
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

static inline void nuvoton_send_first(struct nuvoton_spi *hw, unsigned int lsb)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void nuvoton_set_sleep(struct nuvoton_spi *hw, unsigned int sleep)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	val &= ~(0x0f << 4);

	if (sleep)
		val |= (sleep << 4);

	__raw_writel(val, hw->regs + REG_CTL);
}

static int nuvoton_spi_update_state(struct spi_device *spi)
{
	struct nuvoton_spi *hw = (struct nuvoton_spi *)to_hw(spi);
	unsigned char spimode;

	spi->mode = hw->pdata->spimode;

	if (hw->pdata->quad)
		spi->mode |= (SPI_TX_QUAD | SPI_RX_QUAD);

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

static int nuvoton_spi_setupxfer(struct spi_device *spi)
{
	struct nuvoton_spi *hw = (struct nuvoton_spi *)to_hw(spi);
	unsigned long flags;
	int ret;

	ret = nuvoton_spi_update_state(spi);
	if (ret)
		return ret;

	spin_lock_irqsave(&hw->lock, flags);

	nuvoton_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuvoton_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	nuvoton_set_clock_polarity(hw, hw->pdata->clkpol);
	nuvoton_send_first(hw, hw->pdata->lsb);
	nuvoton_set_divider(hw);

	spin_unlock_irqrestore(&hw->lock, flags);

	return 0;
}

static void nuvoton_spi_hw_init(struct nuvoton_spi *hw)
{
	unsigned long flags;

	spin_lock_init(&hw->lock);

	spin_lock_irqsave(&hw->lock, flags);

	if (hw->pdata->spimode & SPI_CPOL)
		hw->pdata->clkpol = 1;
	else
		hw->pdata->clkpol = 0;

	if ((hw->pdata->spimode == SPI_MODE_0) || (hw->pdata->spimode == SPI_MODE_3)) {
		hw->pdata->txneg = 1;
		hw->pdata->rxneg = 0;
	} else {
		hw->pdata->txneg = 0;
		hw->pdata->rxneg = 1;
	}

	nuvoton_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	nuvoton_send_first(hw, hw->pdata->lsb);
	nuvoton_set_sleep(hw, hw->pdata->sleep);
	nuvoton_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuvoton_set_clock_polarity(hw, hw->pdata->clkpol);

	__raw_writel((__raw_readl(hw->regs + REG_INTERNAL) & ~0xF000) | (hw->pdata->mrxphase << 12),
			hw->regs + REG_INTERNAL); /* MRxPhase(QSPI_INTERNAL[15:12] */

	__raw_writel(__raw_readl(hw->regs + REG_CTL) | SPIEN, hw->regs + REG_CTL); /* enable SPI */
	while ((__raw_readl(hw->regs + REG_STATUS) & (1<<15)) == 0); //SPIENSTS

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST), hw->regs + REG_FIFOCTL);
	while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST) {}

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline unsigned int hw_tx(struct nuvoton_spi *hw, unsigned int count)
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

static inline void hw_rx(struct nuvoton_spi *hw, unsigned int data, int count)
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

static int nuvoton_spi_data_xfer(struct nuvoton_spi *hw, const void *txbuf,
                                 void *rxbuf, unsigned int len)
{
	unsigned int    i,j;
	unsigned int    non_align_len = 0;

	struct ma35d1_ip_dma *pdma = &hw->dma;
	struct ma35d1_peripheral pcfg;

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXFBCLR | RXFBCLR), hw->regs + REG_FIFOCTL);
	while (__raw_readl(hw->regs + REG_STATUS) & FIFOCLR) {}

	hw->tx = txbuf;
	hw->rx = rxbuf;

	/* Use PDMA or not from device tree */
	if (hw->pdata->use_pdma) {

		/* short length transfer by CPU */
		if ( len < USE_PDMA_LEN) {
			if (hw->rx) {
				j = 0;

				for(i = 0; i < len; ) {
					if(((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
						__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
						i++;
					}
					if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}
				}
				while(j < len) {
					if(((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
						hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
						j++;
					}
				}
			} else {
				for(i = 0; i < len; i++) {
					while (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == TXFULL)); //TXFULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
				}
			}

			while (__raw_readl(hw->regs + REG_STATUS) & BUSY); //wait busy

		} else { //Long length transfer by PDMA

			/* Since MA35D1 PDMA cannot transfer data at non-alignment address */
			/* Transfer the non-alignment byte by CPU */
			if ((unsigned long)rxbuf & (ALIGNMENT_4 - 1)) {
				j = 0;
				non_align_len = (ALIGNMENT_4 - ((unsigned long)rxbuf & (ALIGNMENT_4 - 1)));

				for (i = 0; (i < len) && (i < non_align_len); ) {
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
			} else if ((unsigned long)txbuf & (ALIGNMENT_4 - 1)) {
				non_align_len = (ALIGNMENT_4 - ((unsigned long)txbuf & (ALIGNMENT_4 - 1)));
				for (i = 0; (i < len) && (i < non_align_len); )
					if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) != TXFULL)) { //TX NOT FULL
						__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
						i++;
					}
			}

			while (__raw_readl(hw->regs + REG_STATUS) & BUSY); //wait busy

			/* Transfer the rest data starting at alignment address by PDMA */
			if (rxbuf) {

				/* prepare the RX dma transfer */
				sg_init_table(&pdma->sgrx, 1);
				pdma->slave_config.src_addr = (hw->phyaddr + REG_RX);
				pdma->slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
				pdma->sgrx.dma_length = (len - non_align_len) >> 2; /* divide 4 */
				pdma->slave_config.direction = DMA_DEV_TO_MEM;
				pcfg.reqsel = hw->pdata->pdma_reqsel_rx;
				pdma->slave_config.peripheral_config = &pcfg;
				pdma->slave_config.peripheral_size = sizeof(pcfg);
				dmaengine_slave_config(pdma->chan_rx, &(pdma->slave_config));


				/* Map rxbuf to physical address including non-alignment part */
				pdma->sgrx.dma_address = dma_map_single(hw->dev,
				                                        (void *)(rxbuf),
				                                        len, DMA_FROM_DEVICE);
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
				dmaengine_submit(pdma->rxdesc);
				dma_async_issue_pending(pdma->chan_rx);
			} /* rxbuf */

			if (txbuf) {
				/* prepare the TX dma transfer */
				sg_init_table(&pdma->sgtx, 1);
				pdma->slave_config.dst_addr = (hw->phyaddr + REG_TX);
				pdma->slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
				pdma->sgtx.dma_length = len - non_align_len;
				pdma->slave_config.direction = DMA_MEM_TO_DEV;
				pcfg.reqsel = hw->pdata->pdma_reqsel_tx;
				pdma->slave_config.peripheral_config = &pcfg;
				pdma->slave_config.peripheral_size = sizeof(pcfg);
				dmaengine_slave_config(pdma->chan_tx, &(pdma->slave_config));

				/* Map t->tx_buf to physical address including non-alignment part */
				pdma->sgtx.dma_address = dma_map_single(hw->dev,
				                                        (void *)txbuf,
				                                        len, DMA_TO_DEVICE);
				if (dma_mapping_error(hw->dev, pdma->sgtx.dma_address)) {
					dev_err(hw->dev, "tx dma map error\n");
				}
				/* Adjust physical address that skip non-alignment part */
				pdma->sgtx.dma_address += non_align_len;
				dma_sync_single_for_device(hw->dev, pdma->sgtx.dma_address - non_align_len, len, DMA_TO_DEVICE);

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
				dmaengine_submit(pdma->txdesc);
				dma_async_issue_pending(pdma->chan_tx);
			} /* txbuf */

			/* MA35D1 QSPI/SPI limitation, reset SPI TXFIFO write pointer and
			 * RXFIFO read pointer before enabling TX/RX PDMA */
			if (non_align_len) {
				__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXFBCLR | RXFBCLR), hw->regs + REG_FIFOCTL);
				while (__raw_readl(hw->regs + REG_STATUS) & FIFOCLR) {}
			}

			/* Trigger PDMA */
			if (rxbuf) {
				__raw_writel(((__raw_readl(hw->regs + REG_CTL) & ~(DWIDTH_MASK))
				              | BYTE_REORDER), hw->regs + REG_CTL); //32 bits, byte reorder
				__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) | (RXPDMAEN),
				             hw->regs + REG_PDMACTL); //Enable SPIx RX PDMA
			} else
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
			if (rxbuf)
				dma_unmap_single(hw->dev, pdma->sgrx.dma_address - non_align_len, len,
				                 DMA_FROM_DEVICE);
			if (txbuf)
				dma_unmap_single(hw->dev, pdma->sgtx.dma_address - non_align_len, len,
				                 DMA_TO_DEVICE);
		}


	} else { /* hw->pdata->use_pdma = 0 */

		if (hw->rx) {
			j = 0;

			for (i = 0; i < len; ) {
				if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
					i++;
				} else {
					cond_resched();
				}
				if (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
					hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
					j++;
				} else {
					cond_resched();
				}
			}
			while (j < i) {
				if (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == 0)) { //RX NOT EMPTY
					hw_rx(hw, __raw_readl(hw->regs + REG_RX), j);
					j++;
				} else {
					cond_resched();
				}
			}

		} else { /* hw->tx */
			for (i = 0; i < len; ) {
				if (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == 0)) { //TX NOT FULL
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
					i++;
				} else {
					cond_resched();
				}
			}
		}

		while (__raw_readl(hw->regs + REG_STATUS) & BUSY); //wait busy
	}

	return 0;
}

static bool nuvoton_spi_mem_supports_op(struct spi_mem *mem,
                                        const struct spi_mem_op *op)
{
	if (op->data.buswidth > 4 || op->addr.buswidth > 4 ||
	    op->dummy.buswidth > 4 || op->cmd.buswidth > 4)
		return false;

	if (op->data.nbytes && op->dummy.nbytes &&
	    op->data.buswidth != op->dummy.buswidth)
		return false;

	if (op->addr.nbytes > 7)
		return false;

	return true;
}

static void nuvoton_spi_set_cs(struct spi_device *spi, bool lvl)
{
	struct nuvoton_spi *nuvoton = spi_master_get_devdata(spi->master);
	unsigned int val;

	val = __raw_readl(nuvoton->regs + REG_SSCTL);

	if(spi->chip_select == 0) {
		if (!lvl)
			val |= SELECTSLAVE0;
		else
			val &= ~SELECTSLAVE0;
	} else {
		if (!lvl)
			val |= SELECTSLAVE1;
		else
			val &= ~SELECTSLAVE1;
	}

	while (__raw_readl(nuvoton->regs + REG_STATUS) & BUSY); //wait busy

	__raw_writel(val, nuvoton->regs + REG_SSCTL);
}

static struct nuvoton_qspi_info *nuvoton_spi_parse_dt(struct device *dev)
{
	struct nuvoton_qspi_info *sci;
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

	if (of_property_read_u32(dev->of_node, "spimode", &temp)) {
		dev_warn(dev, "can't get spimode from dt\n");
		sci->spimode = 0;
	} else {
		sci->spimode = temp;
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

	if (of_property_read_u32(dev->of_node, "mrxphase", &temp)) {
		dev_warn(dev, "can't get mrxphase from dt\n");
		sci->mrxphase = 0;
	} else {
		sci->mrxphase = temp;
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



static int nuvoton_spi_mem_exec_op(struct spi_mem *mem,
                                   const struct spi_mem_op *op)
{
	struct nuvoton_spi *nuvoton = spi_master_get_devdata(mem->spi->master);
	int i, ret;
	u8 addr[8];

	ret = nuvoton_spi_set_freq(nuvoton, mem->spi->max_speed_hz);
	if (ret)
		return ret;

	nuvoton_spi_setupxfer(mem->spi);

	nuvoton_spi_set_cs(mem->spi, 0); //Activate CS

	ret = nuvoton_spi_data_xfer(nuvoton, &op->cmd.opcode, NULL, 1);
	if (ret)
		goto out;

	if (op->addr.buswidth == 4) {
		__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN | DATDIR),
		             nuvoton->regs + REG_CTL); //Enable Quad mode, direction output
	}

	for (i = 0; i < op->addr.nbytes; i++)
		addr[i] = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

	ret = nuvoton_spi_data_xfer(nuvoton, addr, NULL, op->addr.nbytes);
	if (ret)
		goto out;

	if (op->dummy.buswidth == 4) {
		__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN | DATDIR),
		             nuvoton->regs + REG_CTL); //Enable Quad mode, direction output
	}

	ret = nuvoton_spi_data_xfer(nuvoton, NULL, NULL, op->dummy.nbytes);
	if (ret)
		goto out;

	if (op->data.buswidth == 4) {
		if (op->data.dir == SPI_MEM_DATA_OUT)
			__raw_writel(((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN) | DATDIR),
			             nuvoton->regs + REG_CTL);//Enable Quad mode, direction output
		else if (op->data.dir == SPI_MEM_DATA_IN)
			__raw_writel(((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
			             nuvoton->regs + REG_CTL);//Enable Quad mode, direction input
	}

	ret = nuvoton_spi_data_xfer(nuvoton,
	                            op->data.dir == SPI_MEM_DATA_OUT ?
	                            op->data.buf.out : NULL,
	                            op->data.dir == SPI_MEM_DATA_IN ?
	                            op->data.buf.in : NULL,
	                            op->data.nbytes);

	__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) & ~(QUADIOEN | DATDIR)),
	             nuvoton->regs + REG_CTL); //Restore to single mode, direction input
out:
	nuvoton_spi_set_cs(mem->spi, 1); //Deactivate CS

	return ret;
}

static const struct spi_controller_mem_ops nuvoton_spi_mem_ops = {
	.supports_op = nuvoton_spi_mem_supports_op,
	.exec_op = nuvoton_spi_mem_exec_op,
};

static int nuvoton_spi_transfer_one(struct spi_master *master,
                                    struct spi_device *spi,
                                    struct spi_transfer *t)
{
	struct nuvoton_spi *nuvoton = spi_master_get_devdata(master);
	unsigned int busw = OP_BUSW_1;
	int ret;

	if (t->rx_buf && t->tx_buf) {
		if (((spi->mode & SPI_TX_QUAD) &&
		     !(spi->mode & SPI_RX_QUAD)) ||
		    ((spi->mode & SPI_TX_DUAL) &&
		     !(spi->mode & SPI_RX_DUAL)))
			return -ENOTSUPP;
	}

	ret = nuvoton_spi_set_freq(nuvoton, t->speed_hz);
	if (ret)
		return ret;

	if (t->tx_buf) {
		if (spi->mode & SPI_TX_QUAD)
			busw = OP_BUSW_4;
		else if (spi->mode & SPI_TX_DUAL)
			busw = OP_BUSW_2;
	} else if (t->rx_buf) {
		if (spi->mode & SPI_RX_QUAD)
			busw = OP_BUSW_4;
		else if (spi->mode & SPI_RX_DUAL)
			busw = OP_BUSW_2;
	}

	if (busw == OP_BUSW_4) {
		__raw_writel(((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
		             nuvoton->regs + REG_CTL);//Enable Quad mode, direction input
	}

	ret = nuvoton_spi_data_xfer(nuvoton, t->tx_buf, t->rx_buf, t->len);
	if (ret)
		return ret;

	__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) & ~(QUADIOEN | DATDIR)),
	             nuvoton->regs + REG_CTL); //Restore to single mode, direction input

	spi_finalize_current_transfer(master);

	return 0;
}

static int __maybe_unused nuvoton_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct nuvoton_spi *nuvoton = spi_master_get_devdata(master);

	nuvoton_spi_clk_disable(nuvoton);
	clk_disable_unprepare(nuvoton->clk);

	return 0;
}

static int __maybe_unused nuvoton_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct nuvoton_spi *nuvoton = spi_master_get_devdata(master);
	int ret;

	ret = clk_prepare_enable(nuvoton->clk);
	if (ret) {
		dev_err(dev, "Cannot enable ps_clock.\n");
		return ret;
	}

	return nuvoton_spi_clk_enable(nuvoton);
}

static const struct dev_pm_ops nuvoton_spi_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(nuvoton_spi_runtime_suspend,
	                   nuvoton_spi_runtime_resume, NULL)
};

static int nuvoton_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct nuvoton_spi *nuvoton;
	int ret;
	struct ma35d1_ip_dma *pdma;
	dma_cap_mask_t mask;
	u32   val32[4];
	int err = 0;
	int status = 0;

	master = devm_spi_alloc_master(&pdev->dev, sizeof(struct nuvoton_spi));
	if (!master)
		return -ENOMEM;

	nuvoton = spi_master_get_devdata(master);
	nuvoton->master = spi_master_get(master);
	nuvoton->pdata = nuvoton_spi_parse_dt(&pdev->dev);
	nuvoton->dev = &pdev->dev;

	if (nuvoton->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	if (nuvoton->pdata->use_pdma) {
		/* Zero out the capability mask then initialize it for a slave channel that is
		 * private.
		 */
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		dma_cap_set(DMA_PRIVATE, mask);

		/* Initailize wiat queue head as  __WAIT_QUEUE_HEAD_INITIALIZER() */
		nuvoton->slave_done.lock           = __SPIN_LOCK_UNLOCKED(nuvoton->slave_done.lock);
		nuvoton->slave_done.head.next       = &(nuvoton->slave_done).head;
		nuvoton->slave_done.head.prev       = &(nuvoton->slave_done).head;

		/* Request the DMA channel from the DMA engine and then use the device from
		 * the channel for the proxy channel also.
		 */
		pdma = &nuvoton->dma;
		pdma->chan_rx = dma_request_slave_channel(&pdev->dev,"rx");
		if (!pdma->chan_rx) {
			dev_err(&pdev->dev, "RX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("RX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_rx));

		pdma->chan_tx = dma_request_slave_channel(&pdev->dev,"tx");
		if (!pdma->chan_tx) {
			dev_err(&pdev->dev, "TX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("TX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_tx));
	}

	platform_set_drvdata(pdev, nuvoton);
	init_completion(&nuvoton->done);


	master->dev.of_node = pdev->dev.of_node;

	nuvoton->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(nuvoton->clk)) {
		dev_err(&pdev->dev, "unable to get SYS clock, err=%d\n",
		        status);
		goto err_clk;
	}
	clk_prepare_enable(nuvoton->clk);

	if (of_property_read_u32_array(pdev->dev.of_node, "reg", val32, 4) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	nuvoton->phyaddr = val32[1];
	pr_debug("nuvoton->phyaddr = 0x%lx\n",(ulong)nuvoton->phyaddr);

	nuvoton->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (nuvoton->res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_pdata;
	}

	nuvoton->regs = devm_ioremap_resource(&pdev->dev, nuvoton->res);
	nuvoton->irq = platform_get_irq(pdev, 0);
	if (nuvoton->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_pdata;
	}

	master->num_chipselect = 2;
	master->mem_ops = &nuvoton_spi_mem_ops;

	master->set_cs = nuvoton_spi_set_cs;
	master->transfer_one = nuvoton_spi_transfer_one;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = SPI_CPOL | SPI_CPHA |
	                    SPI_RX_DUAL | SPI_TX_DUAL |
	                    SPI_RX_QUAD | SPI_TX_QUAD;

	nuvoton_spi_hw_init(nuvoton);

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "spi_register_master failed\n");
		pm_runtime_disable(&pdev->dev);
	}

	return ret;

err_clk:
	spi_master_put(nuvoton->master);
err_pdata:
	clk_disable(nuvoton->clk);
	clk_put(nuvoton->clk);

	return err;
}

static int nuvoton_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	spi_unregister_master(master);

	return 0;
}

static const struct of_device_id nuvoton_qspi_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-qspi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nuvoton_qspi_of_match);

static struct platform_driver nuvoton_qspi_driver = {
	.probe = nuvoton_spi_probe,
	.remove = nuvoton_spi_remove,
	.driver = {
		.name = "ma35d1-qspi",
		.of_match_table = nuvoton_qspi_of_match,
		.pm = &nuvoton_spi_dev_pm_ops,
	},
};
module_platform_driver(nuvoton_qspi_driver);

MODULE_DESCRIPTION("nuvoton qspi driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuvoton-qspi");
