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

#define PCLK_FREQ	189000000

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
#define SLAVE		(0x01 << 18)
#define UNITIEN		(0x01 << 17)
#define TXNEG		(0x01 << 2)
#define RXNEG		(0x01 << 1)
#define LSB		(0x01 << 13)
#define SELECTLEV	(0x01 << 2)
#define SELECTPOL	(0x01 << 3)
#define SELECTSLAVE0	0x01
#define SELECTSLAVE1	0x02
#define SPIEN		0x01
#define RXRST		0x01
#define TXRST		0x02
#define BUSY		0x01
#define RXEMPTY		0x100
#define RXFULL		0x200
#define SPIENSTS	0x8000
#define TXEMPTY		0x10000
#define TXFULL		0x20000
#define TXRXRST		0x800000
#define TXTHIEN		(0x01 << 3)
#define RXTHIEN		(0x01 << 2)
#define SSINAIEN	(0x01 << 13)
#define SSACTIF		(0x01 << 2)
#define SSINAIF		(0x01 << 3)
#define RXTHIF		(0x01 << 10)
#define TXTHIF		(0x01 << 18)
#define TXUFIF		(0x01 << 19)

static volatile int slave_done_state=0;
static DECLARE_WAIT_QUEUE_HEAD(slave_done);

static int SPI_SlaveDataLen = 256;
static int SPI_SlaveData[256];
static int TransmittedCnt = 0;
static int InTransmitted = 0;

struct ma35d1_spi_info {
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
};

struct ma35d1_spi {
	struct spi_bitbang      bitbang;
	void __iomem            *regs;
	int                     irq;
	unsigned int            len;
	unsigned int            count;
	const void              *tx;
	void                    *rx;
	struct clk              *clk;
	struct spi_master       *master;
	struct device           *dev;
	struct ma35d1_spi_info  *pdata;
	spinlock_t              lock;
	struct resource         *res;
};

static inline struct ma35d1_spi0 *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static inline void ma35d1_slave_select(struct spi_device *spi, unsigned int ssr)
{
	struct ma35d1_spi *hw = (struct ma35d1_spi *)to_hw(spi);
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

static inline void ma35d1_spi_chipsel(struct spi_device *spi, int value)
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

static inline void ma35d1_setup_txbitlen(struct ma35d1_spi *hw,
        unsigned int txbitlen)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);
	val &= ~0x1f00;
	if(txbitlen != 32)
		val |= (txbitlen << 8);

	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void ma35d1_set_clock_polarity(struct ma35d1_spi *hw, unsigned int polarity)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (polarity)
		val |= SELECTPOL;
	else
		val &= ~SELECTPOL;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void ma35d1_tx_rx_edge(struct ma35d1_spi *hw, unsigned int tx_edge, unsigned int rx_edge)
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

static inline void ma35d1_send_first(struct ma35d1_spi *hw, unsigned int lsb)
{
	unsigned int val;

	val = __raw_readl(hw->regs + REG_CTL);

	if (lsb)
		val |= LSB;
	else
		val &= ~LSB;
	__raw_writel(val, hw->regs + REG_CTL);
}

static inline void ma35d1_set_sleep(struct ma35d1_spi *hw, unsigned int sleep)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	val &= ~(0x0f << 4);

	if (sleep)
		val |= (sleep << 4);

	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}


static inline void ma35d1_set_divider(struct ma35d1_spi *hw)
{
	__raw_writel(hw->pdata->divider, hw->regs + REG_CLKDIV);
}

static inline void ma35d1_enable_slave(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_CTL);

	val |= SLAVE;

	__raw_writel(val, hw->regs + REG_CTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_enable_rxth_int(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val = (val & ~0x7000000) | 0x0000000; /* set RXTH = 0 */
	val |= RXTHIEN; /* enable RXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_disable_rxth_int(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val &= ~RXTHIEN; /* disable RXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_enable_txth_int(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val = (val & ~0x70000000) | 0x30000000; /* set TXTH = 3 */
	val |= TXTHIEN; /* enable TXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_disable_txth_int(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_FIFOCTL);

	val &= ~TXTHIEN; /* disable TXTHIEN */

	__raw_writel(val, hw->regs + REG_FIFOCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static inline void ma35d1_enable_ssinact_int(struct ma35d1_spi *hw)
{
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);

	val = __raw_readl(hw->regs + REG_SSCTL);

	val |= SSINAIEN; /* enable SSINAIEN */

	__raw_writel(val, hw->regs + REG_SSCTL);

	spin_unlock_irqrestore(&hw->lock, flags);
}

static irqreturn_t ma35d1_spi_irq(int irq, void *dev)
{
	struct ma35d1_spi *hw = dev;
	unsigned int status;

	status = __raw_readl(hw->regs + REG_STATUS);
	__raw_writel(status, hw->regs + REG_STATUS);

	if (status & RXTHIF) {
		if (InTransmitted == 0) {
			ma35d1_disable_rxth_int(hw);
			slave_done_state = 1;
			wake_up_interruptible(&slave_done);
			InTransmitted = 1;
		}
	}
	if (status & TXTHIF) {
		while(!(__raw_readl(hw->regs + REG_STATUS) & TXFULL)) {//TXFULL
			__raw_writel(SPI_SlaveData[TransmittedCnt++], hw->regs + REG_TX);
			if (TransmittedCnt >= SPI_SlaveDataLen) {
				ma35d1_disable_txth_int(hw);
				InTransmitted = 0;
				TransmittedCnt = 0;
				break;
			}
		}
	}
	if (status & SSINAIF) {
		/* Check if transmition complete */
		if (InTransmitted == 1) {
			pr_debug("Master pull SS high, but slave TX doesn't complete!\n");
			__raw_writel(__raw_readl(hw->regs + REG_FIFOCTL) | (TXRST | RXRST),
			             hw->regs + REG_FIFOCTL); //TXRX reset
			while (__raw_readl(hw->regs + REG_STATUS) & TXRXRST); //TXRXRST
			ma35d1_enable_rxth_int(hw);
			InTransmitted = 0;
			TransmittedCnt = 0;
		}
	}

	return IRQ_HANDLED;
}

static int ma35d1_spi_update_state(struct spi_device *spi,
                                    struct spi_transfer *t)
{
	struct ma35d1_spi *hw = (struct ma35d1_spi *)to_hw(spi);
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

static int ma35d1_spi_setupxfer(struct spi_device *spi,
                                 struct spi_transfer *t)
{
	struct ma35d1_spi *hw = (struct ma35d1_spi *)to_hw(spi);
	unsigned long flags;
	int ret;

	ret = ma35d1_spi_update_state(spi, t);
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

static int ma35d1_spi_setup(struct spi_device *spi)
{
	struct ma35d1_spi *hw = (struct ma35d1_spi *)to_hw(spi);
	int ret;

	ret = ma35d1_spi_update_state(spi, NULL);
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

static void ma35d1_init_spi(struct ma35d1_spi *hw)
{
	spin_lock_init(&hw->lock);

	ma35d1_enable_slave(hw);
	ma35d1_enable_rxth_int(hw);
	ma35d1_enable_ssinact_int(hw);
}

static struct ma35d1_spi_info *ma35d1_spi_parse_dt(struct device *dev)
{
	struct ma35d1_spi_info *sci;
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

	if (of_property_read_u32(dev->of_node, "divider", &temp)) {
		dev_warn(dev, "can't get divider from dt\n");
		sci->divider = 3;
	} else {
		sci->divider = temp;
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

	return sci;
}

/* In Thread, only prepare data and enable TXTHIEN.
   The data will be transmitted in IRQ
 */
static int SPI_Slave_Thread_TXRX(struct ma35d1_spi *hw)
{
	unsigned char rx;
	int i;

	while(1) {

		wait_event_interruptible(slave_done, (slave_done_state != 0));
		rx = __raw_readl(hw->regs + REG_RX);
		pr_debug("Receive [0x%x] \n", rx);

		switch (rx) {
		case 0x9f:
			for (i = 0; i < SPI_SlaveDataLen; i++)
				SPI_SlaveData[i] = i;

			ma35d1_enable_txth_int(hw);
			break;
		default:
			break;
		}

		InTransmitted = 0;
		slave_done_state = 0;
		ma35d1_enable_rxth_int(hw);
	}

	return 0;
}

static int ma35d1_spi_probe(struct platform_device *pdev)
{
	struct ma35d1_spi *hw;
	struct spi_master *master;
	int err = 0;
	int status = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ma35d1_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	hw->master = spi_master_get(master);
	hw->pdata = ma35d1_spi_parse_dt(&pdev->dev);
	hw->dev = &pdev->dev;

	if (hw->pdata == NULL) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		err = -ENOENT;
		goto err_pdata;
	}

	platform_set_drvdata(pdev, hw);
	master->mode_bits	= (SPI_MODE_0 | SPI_TX_DUAL | SPI_RX_DUAL | SPI_CS_HIGH
	                       | SPI_LSB_FIRST | SPI_CPHA | SPI_CPOL);

	master->dev.of_node        = pdev->dev.of_node;
	master->num_chipselect     = hw->pdata->num_cs;
	master->bus_num            = hw->pdata->bus_num;
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = ma35d1_spi_setupxfer;
	hw->bitbang.chipselect     = ma35d1_spi_chipsel;
	hw->bitbang.master->setup  = ma35d1_spi_setup;

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

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_pdata;
	}

	err = request_irq(hw->irq, ma35d1_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
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
	while ((__raw_readl(hw->regs + REG_STATUS) & SPIENSTS) == 0); //SPIENSTS

	kthread_run((int (*)(void *))SPI_Slave_Thread_TXRX, hw, "SPI_SLAVE_Thread_TXRX");

	return 0;

err_clk:
	free_irq(hw->irq, hw);
err_pdata:
	spi_master_put(hw->master);

err_nomem:
	return err;
}

static int ma35d1_spi_remove(struct platform_device *dev)
{
	struct ma35d1_spi *hw = platform_get_drvdata(dev);

	free_irq(hw->irq, hw);
	platform_set_drvdata(dev, NULL);
	spi_bitbang_stop(&hw->bitbang);

	clk_disable(hw->clk);
	clk_put(hw->clk);

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM
static int ma35d1_spi_suspend(struct device *dev)
{
	struct ma35d1_spi *hw = dev_get_drvdata(dev);

	while (__raw_readl(hw->regs + REG_STATUS) & 1) //wait busy
		msleep(1);

	// disable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) & ~(UNITIEN)), hw->regs + REG_CTL);

	return 0;
}

static int ma35d1_spi_resume(struct device *dev)
{
	struct ma35d1_spi *hw = dev_get_drvdata(dev);

	// enable interrupt
	__raw_writel((__raw_readl(hw->regs + REG_CTL) | UNITIEN), hw->regs + REG_CTL);

	return 0;
}

static const struct dev_pm_ops ma35d1_spi_pmops = {
	.suspend    = ma35d1_spi_suspend,
	.resume     = ma35d1_spi_resume,
};

#define MA35D1_SPI_PMOPS (&ma35d1_spi_pmops)

#else
#define MA35D1_SPI_PMOPS NULL
#endif

static const struct of_device_id ma35d1_spi_slave_of_match[] = {
	{   .compatible = "nuvoton,ma35d1-spi-slave" },
	{	},
};
MODULE_DEVICE_TABLE(of, ma35d1_spi_slave_of_match);

static struct platform_driver ma35d1_spi_slave_driver = {
	.probe      = ma35d1_spi_probe,
	.remove     = ma35d1_spi_remove,
	.driver     = {
		.name   = "ma35d1-spi-slave",
		.owner  = THIS_MODULE,
		.pm	= MA35D1_SPI_PMOPS,
		.of_match_table = of_match_ptr(ma35d1_spi_slave_of_match),
	},
};
module_platform_driver(ma35d1_spi_slave_driver);

MODULE_DESCRIPTION("ma35d1 spi slave driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-spi-slave");
