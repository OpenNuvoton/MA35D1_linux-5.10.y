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
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/wait.h>

#include <linux/interrupt.h>
#include <linux/of.h>
#include <asm/irq.h>
#include <linux/platform_data/dma-ma35d1.h>

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
#define UNITIF          0x02
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

#define SPI_GENERAL_TIMEOUT_MS	1000

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
	unsigned int pdma_reqsel_tx;
	unsigned int pdma_reqsel_rx;
	unsigned int mrxphase;
};

struct nuvoton_spi {
	struct completion               txdone;
	struct completion               rxdone;
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
	/* Serialize SPI-mem operations; PDMA waits may sleep. */
	struct mutex                    lock;
	struct resource                 *res;
	struct ma35d1_dma_done          dma_slave_rxdone;
	struct ma35d1_dma_done          dma_slave_txdone;
	struct ma35d1_ip_dma            dma;
	int                             slave_txdone_state;
	int                             slave_rxdone_state;
	struct wait_queue_head          slave_txdone;
	struct wait_queue_head          slave_rxdone;
	unsigned int                    phyaddr;
	unsigned int			cur_speed_hz;
	unsigned int			last_xfer_len;
};

static inline struct nuvoton_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void ma35d1_slave_txdma_callback(void *arg)
{
	struct nuvoton_spi *hw = (struct nuvoton_spi *)arg;
	struct ma35d1_dma_done *done = &hw->dma_slave_txdone;

	done->done = true;
	WRITE_ONCE(hw->slave_txdone_state, 1);
	wake_up(&hw->slave_txdone);
}

static void ma35d1_slave_rxdma_callback(void *arg)
{
	struct nuvoton_spi *hw = (struct nuvoton_spi *)arg;
	struct ma35d1_dma_done *done = &hw->dma_slave_rxdone;

	done->done = true;
	WRITE_ONCE(hw->slave_rxdone_state, 1);
	wake_up(&hw->slave_rxdone);
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
	if (txbitlen != 32)
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
	int ret;

	ret = nuvoton_spi_update_state(spi);
	if (ret)
		return ret;

	nuvoton_setup_txbitlen(hw, hw->pdata->txbitlen);
	nuvoton_tx_rx_edge(hw, hw->pdata->txneg, hw->pdata->rxneg);
	nuvoton_set_clock_polarity(hw, hw->pdata->clkpol);
	nuvoton_send_first(hw, hw->pdata->lsb);
	nuvoton_set_divider(hw);

	return 0;
}

static void nuvoton_spi_hw_init(struct nuvoton_spi *hw)
{
	mutex_init(&hw->lock);
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
	while ((__raw_readl(hw->regs + REG_STATUS) & SPIENSTS) == 0)
		;

	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST), hw->regs + REG_FIFOCTL);
	while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST)
		;

}

static inline unsigned int hw_tx(struct nuvoton_spi *hw, unsigned int count)
{
	const unsigned char *tx_byte = hw->tx;
	const unsigned short *tx_short = hw->tx;
	const unsigned int *tx_int = hw->tx;
	unsigned int bwp = hw->pdata->txbitlen;

	if (bwp <= 8)
		return tx_byte ? tx_byte[count] : 0;
	else if (bwp <= 16)
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

	if (bwp <= 8)
		rx_byte[count] = data;
	else if (bwp <= 16)
		rx_short[count] = data;
	else
		rx_int[count] = data;
}

/* The callback's state may also carry a negative controller-specific error. */
static int nuvoton_spi_dma_wait(struct nuvoton_spi *hw,
			       wait_queue_head_t *queue, int *state,
			       const char *direction)
{
	long timeout;
	int status;

	timeout = wait_event_killable_timeout(*queue,
					   READ_ONCE(*state) != 0,
					   msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS));
	if (!timeout) {
		dev_err(hw->dev, "QSPI %s DMA timeout: STATUS=0x%08x, len=%u\n",
			direction, __raw_readl(hw->regs + REG_STATUS),
			hw->last_xfer_len);
		return -ETIMEDOUT;
	}

	if (timeout < 0) {
		dev_err(hw->dev, "QSPI %s DMA wait interrupted: %ld\n",
			direction, timeout);
		return (int)timeout;
	}

	status = READ_ONCE(*state);
	if (status < 0) {
		dev_err(hw->dev, "QSPI %s DMA completion error: %d\n",
			direction, status);
		return status;
	}

	return 0;
}

/* Return to the baseline 8-bit PIO transfer configuration. */
static void nuvoton_spi_pdma_restore(struct nuvoton_spi *hw)
{
	u32 ctl;

	ctl = __raw_readl(hw->regs + REG_CTL);
	__raw_writel(ctl & ~SPIEN, hw->regs + REG_CTL);
	__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) &
		     ~(TXPDMAEN | RXPDMAEN), hw->regs + REG_PDMACTL);

	ctl = __raw_readl(hw->regs + REG_CTL);
	ctl &= ~(DWIDTH_MASK | BYTE_REORDER | QUADIOEN | DATDIR);
	ctl |= 8 << DWIDTH_POS;
	__raw_writel(ctl, hw->regs + REG_CTL);
	__raw_writel(ctl | SPIEN, hw->regs + REG_CTL);
}

static int nuvoton_spi_data_xfer(struct nuvoton_spi *hw, const void *txbuf,
				void *rxbuf, unsigned int len)
{
	struct ma35d1_ip_dma *pdma = &hw->dma;
	struct ma35d1_peripheral pcfg = { 0 };
	unsigned long end;
	unsigned int  i;

	hw->last_xfer_len = len;
	__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXFBCLR | RXFBCLR), hw->regs + REG_FIFOCTL);
	end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
	while (__raw_readl(hw->regs + REG_STATUS) & FIFOCLR) {
		if (time_after(jiffies, end)) {
			printk("SPI FIFOCLR timeout: %d\n", __LINE__);
			return -ETIMEDOUT;
		}
	}

	hw->tx = txbuf;
	hw->rx = rxbuf;
	/* Use PDMA or not from device tree */
	if (hw->pdata->use_pdma) {

		/* short length transfer by CPU */
		if (len < USE_PDMA_LEN) {
			end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
			if (hw->rx) {
				for (i = 0; i < len; i++) {
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
					while (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == RXEMPTY)) {
						if (time_after(jiffies, end)) {
							printk("SPI RXEMPTY timeout: %d\n", __LINE__);
							return -ETIMEDOUT;
						}
					}
					hw_rx(hw, __raw_readl(hw->regs + REG_RX), i);
				}
			} else {
				for (i = 0; i < len; i++) {
					while (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == TXFULL)) {
						if (time_after(jiffies, end)) {
							printk("SPI TXFULL timeout: %d\n", __LINE__);
							return -ETIMEDOUT;
						}
					}
					__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
				}
			}

			while (__raw_readl(hw->regs + REG_STATUS) & BUSY) {
				if (time_after(jiffies, end)) {
					printk("SPI BUSY timeout: %d\n", __LINE__);
					return -ETIMEDOUT;
				}
			}

		} else { /* Long length transfer by PDMA */
			const void *dma_txbuf = txbuf;
			void *dummy_txbuf = NULL;
			dma_cookie_t cookie;
			bool tx_mapped = false, rx_mapped = false;
			bool tx_started = false, rx_started = false;
			bool width_4;
			int pdma_en = 0;
			int tx_ret = 0, rx_ret = 0;
			int term_ret;
			int ret = 0;

			/* RX needs dummy TX data to generate SPI clocks. */
			if (!dma_txbuf) {
				dummy_txbuf = kzalloc(len, GFP_KERNEL);
				if (!dummy_txbuf)
					return -ENOMEM;
				dma_txbuf = dummy_txbuf;
			}

			end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
			__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) |
				     (TXFBCLR | RXFBCLR), hw->regs + REG_FIFOCTL);
			while (__raw_readl(hw->regs + REG_STATUS) & FIFOCLR) {
				if (time_after_eq(jiffies, end)) {
					dev_err(hw->dev,
						"QSPI PDMA FIFO clear timeout: STATUS=0x%08x, len=%u\n",
						__raw_readl(hw->regs + REG_STATUS), len);
					ret = -ETIMEDOUT;
					goto pdma_cleanup;
				}
			}

			width_4 = !(len & 3) &&
				  !((unsigned long)dma_txbuf & 3) &&
				  (!rxbuf || !((unsigned long)rxbuf & 3));

			/* Set completion states before issuing either DMA descriptor. */
			WRITE_ONCE(hw->slave_txdone_state, 0);
			WRITE_ONCE(hw->slave_rxdone_state, 0);
			hw->dma_slave_txdone.done = false;
			hw->dma_slave_rxdone.done = false;

			if (rxbuf) {
				sg_init_table(&pdma->sgrx, 1);
				pdma->slave_config.src_addr = hw->phyaddr + REG_RX;
				pdma->slave_config.src_addr_width = width_4 ?
					DMA_SLAVE_BUSWIDTH_4_BYTES :
					DMA_SLAVE_BUSWIDTH_1_BYTE;
				/*
				 * The Linux 5.10 PDMA provider expects transfer COUNT
				 * here, not the standard DMAengine byte length. Keep this
				 * legacy convention until both drivers change together.
				 */
				pdma->sgrx.dma_length = width_4 ? len >> 2 : len;

				pdma->slave_config.direction = DMA_DEV_TO_MEM;
				pcfg.reqsel = hw->pdata->pdma_reqsel_rx;
				pdma->slave_config.peripheral_config = &pcfg;
				pdma->slave_config.peripheral_size = sizeof(pcfg);
				ret = dmaengine_slave_config(pdma->chan_rx,
							     &pdma->slave_config);
				if (ret) {
					dev_err(hw->dev, "QSPI RX DMA config failed: %d\n", ret);
					goto pdma_cleanup;
				}

				pdma->sgrx.dma_address = dma_map_single(hw->dev, rxbuf,
								   len, DMA_FROM_DEVICE);
				if (dma_mapping_error(hw->dev, pdma->sgrx.dma_address)) {
					dev_err(hw->dev, "QSPI RX DMA mapping failed\n");
					ret = -ENOMEM;
					goto pdma_cleanup;
				}
				rx_mapped = true;

				pdma->rxdesc = dmaengine_prep_slave_sg(pdma->chan_rx,
									 &pdma->sgrx, 1,
									 DMA_DEV_TO_MEM,
									 DMA_PREP_INTERRUPT |
									 DMA_CTRL_ACK);
				if (!pdma->rxdesc) {
					dev_err(hw->dev, "QSPI RX DMA descriptor failed\n");
					ret = -EIO;
					goto pdma_cleanup;
				}

				pdma->rxdesc->callback = ma35d1_slave_rxdma_callback;
				pdma->rxdesc->callback_param = hw;
				cookie = dmaengine_submit(pdma->rxdesc);
				ret = dma_submit_error(cookie);
				if (ret) {
					dev_err(hw->dev, "QSPI RX DMA submit failed: %d\n", ret);
					goto pdma_cleanup;
				}
				rx_started = true;
				dma_async_issue_pending(pdma->chan_rx);
				pdma_en |= RXPDMAEN;
			}

			if (dma_txbuf) {
				sg_init_table(&pdma->sgtx, 1);
				pdma->slave_config.dst_addr = hw->phyaddr + REG_TX;
				pdma->slave_config.dst_addr_width = width_4 ?
					DMA_SLAVE_BUSWIDTH_4_BYTES :
					DMA_SLAVE_BUSWIDTH_1_BYTE;
				pdma->sgtx.dma_length = width_4 ? len >> 2 : len;

				pdma->slave_config.direction = DMA_MEM_TO_DEV;
				pcfg.reqsel = hw->pdata->pdma_reqsel_tx;
				pdma->slave_config.peripheral_config = &pcfg;
				pdma->slave_config.peripheral_size = sizeof(pcfg);
				ret = dmaengine_slave_config(pdma->chan_tx,
							     &pdma->slave_config);
				if (ret) {
					dev_err(hw->dev, "QSPI TX DMA config failed: %d\n", ret);
					goto pdma_cleanup;
				}

				pdma->sgtx.dma_address = dma_map_single(hw->dev,
									   (void *)dma_txbuf,
									   len, DMA_TO_DEVICE);
				if (dma_mapping_error(hw->dev, pdma->sgtx.dma_address)) {
					dev_err(hw->dev, "QSPI TX DMA mapping failed\n");
					ret = -ENOMEM;
					goto pdma_cleanup;
				}
				tx_mapped = true;

				pdma->txdesc = dmaengine_prep_slave_sg(pdma->chan_tx,
									 &pdma->sgtx, 1,
									 DMA_MEM_TO_DEV,
									 DMA_PREP_INTERRUPT |
									 DMA_CTRL_ACK);
				if (!pdma->txdesc) {
					dev_err(hw->dev, "QSPI TX DMA descriptor failed\n");
					ret = -EIO;
					goto pdma_cleanup;
				}

				pdma->txdesc->callback = ma35d1_slave_txdma_callback;
				pdma->txdesc->callback_param = hw;
				cookie = dmaengine_submit(pdma->txdesc);
				ret = dma_submit_error(cookie);
				if (ret) {
					dev_err(hw->dev, "QSPI TX DMA submit failed: %d\n", ret);
					goto pdma_cleanup;
				}
				tx_started = true;
				dma_async_issue_pending(pdma->chan_tx);
				pdma_en |= TXPDMAEN;
			}

			if (width_4)
				__raw_writel((__raw_readl(hw->regs + REG_CTL) &
					      ~DWIDTH_MASK) | BYTE_REORDER,
					     hw->regs + REG_CTL);

			__raw_writel(UNITIF, hw->regs + REG_STATUS);
			__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) | pdma_en,
				     hw->regs + REG_PDMACTL);

			if (tx_started)
				tx_ret = nuvoton_spi_dma_wait(hw, &hw->slave_txdone,
							    &hw->slave_txdone_state, "TX");

			/* Fatal-signal interruption should begin DMA recovery promptly. */
			if (rx_started && tx_ret != -ERESTARTSYS && tx_ret != -EINTR)
				rx_ret = nuvoton_spi_dma_wait(hw, &hw->slave_rxdone,
							    &hw->slave_rxdone_state, "RX");

			/* Keep TX/RX errors independent: successful RX must not mask TX. */
			if (tx_ret)
				ret = tx_ret;
			else if (rx_ret)
				ret = rx_ret;

			if (!ret) {
				end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
				while (__raw_readl(hw->regs + REG_STATUS) & BUSY) {
					if (time_after_eq(jiffies, end)) {
						dev_err(hw->dev,
							"QSPI PDMA BUSY timeout: STATUS=0x%08x, len=%u\n",
							__raw_readl(hw->regs + REG_STATUS), len);
						ret = -ETIMEDOUT;
						break;
					}
				}
			}

pdma_cleanup:
			/* Stop QSPI requests before terminating incomplete PDMA work. */
			__raw_writel(__raw_readl(hw->regs + REG_PDMACTL) &
				     ~(TXPDMAEN | RXPDMAEN), hw->regs + REG_PDMACTL);
			if (ret) {
				if (tx_started) {
					term_ret = dmaengine_terminate_sync(pdma->chan_tx);
					if (term_ret)
						dev_err(hw->dev, "QSPI TX DMA terminate failed: %d\n",
							term_ret);
				}
				if (rx_started) {
					term_ret = dmaengine_terminate_sync(pdma->chan_rx);
					if (term_ret)
						dev_err(hw->dev, "QSPI RX DMA terminate failed: %d\n",
							term_ret);
				}
			}

			nuvoton_spi_pdma_restore(hw);
			if (rx_mapped)
				dma_unmap_single(hw->dev, pdma->sgrx.dma_address,
							 len, DMA_FROM_DEVICE);
			if (tx_mapped)
				dma_unmap_single(hw->dev, pdma->sgtx.dma_address,
							 len, DMA_TO_DEVICE);
			kfree(dummy_txbuf);
			return ret;

		}

	} else { /* hw->pdata->use_pdma = 0 */

		end = jiffies + msecs_to_jiffies(SPI_GENERAL_TIMEOUT_MS);
		if (hw->rx) {
			for (i = 0; i < len; i++) {
				__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
				while (((__raw_readl(hw->regs + REG_STATUS) & RXEMPTY) == RXEMPTY)) {
					if (time_after(jiffies, end)) {
						printk("SPI RXEMPTY timeout: %d\n", __LINE__);
						return -ETIMEDOUT;
					}
				}
				hw_rx(hw, __raw_readl(hw->regs + REG_RX), i);
			}
		} else {
			for (i = 0; i < len; i++) {
				while (((__raw_readl(hw->regs + REG_STATUS) & TXFULL) == TXFULL)) {
					if (time_after(jiffies, end)) {
						printk("SPI TXFULL timeout: %d\n", __LINE__);
						return -ETIMEDOUT;
					}
				}
				__raw_writel(hw_tx(hw, i), hw->regs + REG_TX);
			}
		}

		while (__raw_readl(hw->regs + REG_STATUS) & BUSY) {
			if (time_after(jiffies, end)) {
				printk("SPI BUSY timeout: %d\n", __LINE__);
				return -ETIMEDOUT;
			}
		}
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
	unsigned long end;
	unsigned int val;

	val = __raw_readl(nuvoton->regs + REG_SSCTL);

	if (spi->chip_select == 0) {
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

	end = jiffies + msecs_to_jiffies(3000);
	while (__raw_readl(nuvoton->regs + REG_STATUS) & BUSY) {
		if (time_after(jiffies, end)) {
			dev_err(nuvoton->dev,
				"QSPI CS BUSY timeout: STATUS=0x%08x, last transfer len=%u, cs=%u, level=%d\n",
				__raw_readl(nuvoton->regs + REG_STATUS),
				nuvoton->last_xfer_len, spi->chip_select, lvl);
			return;
		}
	}

	__raw_writel(val, nuvoton->regs + REG_SSCTL);
}

static struct nuvoton_qspi_info *nuvoton_spi_parse_dt(struct device *dev)
{
	struct nuvoton_qspi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci)
		return ERR_PTR(-ENOMEM);

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

	mutex_lock(&nuvoton->lock);
	ret = nuvoton_spi_set_freq(nuvoton, mem->spi->max_speed_hz);
	if (ret) {
		printk("nuvoton_spi_set_freq failed!\n");
		goto out;
	}
	nuvoton_spi_setupxfer(mem->spi);

	nuvoton_spi_set_cs(mem->spi, 0); //Activate CS

	ret = nuvoton_spi_data_xfer(nuvoton, &op->cmd.opcode, NULL, 1);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

	if (op->addr.buswidth == 4)
		__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN | DATDIR),
				nuvoton->regs + REG_CTL); //Enable Quad mode, direction output

	for (i = 0; i < op->addr.nbytes; i++)
		addr[i] = op->addr.val >> (8 * (op->addr.nbytes - i - 1));

	ret = nuvoton_spi_data_xfer(nuvoton, addr, NULL, op->addr.nbytes);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

	if (op->dummy.buswidth == 4) {
		__raw_writel((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN | DATDIR),
		nuvoton->regs + REG_CTL); //Enable Quad mode, direction output
	}

	ret = nuvoton_spi_data_xfer(nuvoton, NULL, NULL, op->dummy.nbytes);
	if (ret) {
		printk("nuvoton_spi_data_xfer failed!! %d\n", __LINE__);
		goto out;
	}

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

out:
	/* Restore normal bus width even when command/address/dummy fails. */
	__raw_writel(__raw_readl(nuvoton->regs + REG_CTL) &
		     ~(QUADIOEN | DATDIR), nuvoton->regs + REG_CTL);
	nuvoton_spi_set_cs(mem->spi, 1); //Deactivate CS

	mutex_unlock(&nuvoton->lock);
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

	if (busw == OP_BUSW_4)
		__raw_writel(((__raw_readl(nuvoton->regs + REG_CTL) | QUADIOEN) & ~DATDIR),
				nuvoton->regs + REG_CTL);//Enable Quad mode, direction input

	ret = nuvoton_spi_set_freq(nuvoton, t->speed_hz);
	if (ret)
		return ret;

	ret = nuvoton_spi_data_xfer(nuvoton, t->tx_buf, t->rx_buf, t->len);
	if (ret)
		return ret;

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

		/* Initailize wait queue head as  __WAIT_QUEUE_HEAD_INITIALIZER() */
		init_waitqueue_head(&nuvoton->slave_txdone);
		init_waitqueue_head(&nuvoton->slave_rxdone);

		/* Request the DMA channel from the DMA engine and then use the device from
		 * the channel for the proxy channel also.
		 */
		pdma = &nuvoton->dma;
		pdma->chan_rx = dma_request_slave_channel(&pdev->dev, "rx");
		if (!pdma->chan_rx) {
			dev_err(&pdev->dev, "RX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("RX %s: %s module removed\n", __func__, dma_chan_name(pdma->chan_rx));

		pdma->chan_tx = dma_request_slave_channel(&pdev->dev, "tx");
		if (!pdma->chan_tx) {
			dev_err(&pdev->dev, "TX DMA channel request error\n");
			err = -ENOENT;
			goto err_pdata;
		}
		pr_debug("TX %s: %s module removed\n", __func__, dma_chan_name(pdma->chan_tx));
	}

	platform_set_drvdata(pdev, nuvoton);
	init_completion(&nuvoton->txdone);
	init_completion(&nuvoton->rxdone);


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
	pr_debug("nuvoton->phyaddr = 0x%lx\n", (ulong)nuvoton->phyaddr);

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
