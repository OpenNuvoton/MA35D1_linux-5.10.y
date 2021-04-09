// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>

// Allocat 4 channels for Tx/Rx, and 4 for general interrupt
#define DATA_CHANS		4	// chan 0~3
#define GINT_CHANS		4	// chan 4~7
#define STS_CHAN		1	// chan 8
// Each Tx/Rx channel can transfer 4 words
#define DATA_COUNT		4

/* Register offset */
#define WKCTL			0x0000
#define INTEN			0x0004
#define INTSTS			0x0008
#define CPSTS			0x0040
#define GINTTRG			0x0080
#define TXCTL			0x00C0
#define TXSTS			0x00C4
#define RXCTL			0x00C8
#define RXSTS			0x00CC
#define TM0			0x0100
#define TX_OFST			0x10
#define RM0			0x0200
#define RX_OFST			0x10

/* Register bit field */
#define WKCTK_GI_WKEN		0x00000F00
#define WKCTK_RXTX_WKEN		0x0F0F0000
#define INTEN_CPSTS_IEN		0x00000007
#define INTEN_GI0_INTEN		0x00000100
#define INTEN_RXTX0_INTEN	0x01010000
#define INTSTS_CPSTSIF		0x00000007
#define INTSTS_GI0IF		0x00000100
#define INTSTS_TX0IF		0x00010000
#define INTSTS_RX0IF		0x01000000
#define GINTTRG_TRGGI0		0x00000001
#define TXCTL_CH0SND		0x00000001
#define TXSTS_CH0RDY		0x00000001
#define RXCTL_CH0ACK		0x00000001
#define RXSTS_CH0RDY		0x00000001

struct nvt_wh {
	// Used by all mailbox type to record channel number
	u32			ch;
	// Following element used by GI to set txdone only
	u32			queue;
	struct hrtimer		ack_hrt;
	struct mbox_chan	*chan;
};


struct nvt_priv {
	struct device		*dev;
	void __iomem		*base;
	struct clk		*clk;
	int			irq;
	bool			wake;
	struct mbox_controller	mbox;
};




static irqreturn_t wormhole_isr(int irq, void *data)
{
	struct nvt_priv *priv = (struct nvt_priv *)data;
	// Only process enabled interrupt
	u32 intsts = ioread32(priv->base + INTSTS) & ioread32(priv->base + INTEN);
	u32 ch, d[4] = {0};

	// Process Tx and Rx
	for (ch = 0; ch < DATA_CHANS ; ch++) {
		if (intsts & (INTSTS_TX0IF << ch))
			mbox_chan_txdone(&priv->mbox.chans[ch], 0);

		if (intsts & (INTSTS_RX0IF << ch)) {
			void __iomem *base = priv->base + RM0 + RX_OFST * ch;
			int i;

			for (i = 0; i < DATA_COUNT; i++)
				d[i] = ioread32(base + sizeof(u32) * i);
			mbox_chan_received_data(&priv->mbox.chans[ch], (void *)d);
			iowrite32(RXCTL_CH0ACK << ch, priv->base + RXCTL);	// ACK
		}
	}

	// Process GI
	for (ch = 0; ch < GINT_CHANS ; ch++) {
		if (intsts & (INTSTS_GI0IF << ch))
			mbox_chan_received_data(&priv->mbox.chans[ch + DATA_CHANS], NULL);
	}

	if (intsts & INTSTS_CPSTSIF) {
		d[0] = ioread32(priv->base + CPSTS);
		iowrite32(d[0], priv->base + CPSTS);
		mbox_chan_received_data(&priv->mbox.chans[DATA_CHANS + GINT_CHANS], (void *)d);
	}
	// Clear interrupt
	iowrite32(intsts, priv->base + INTSTS);

	return IRQ_HANDLED;
}

static int wormhole_send_data(struct mbox_chan *chan, void *data)
{
	struct nvt_priv *priv = container_of(chan->mbox, struct nvt_priv, mbox);
	struct nvt_wh *wh = (struct nvt_wh *)chan->con_priv;

	if (wh->ch < DATA_CHANS) {
		u32 i;

		if (ioread32(priv->base + TXSTS) & (TXSTS_CH0RDY << wh->ch)) {
			u32 *d = (u32 *)data;
			void __iomem *base = priv->base + TM0 + TX_OFST * wh->ch;

			for (i = 0; i < DATA_COUNT; i++)
				iowrite32(*d++, base + sizeof(u32) * i);
			iowrite32(TXCTL_CH0SND << wh->ch, priv->base + TXCTL);
		} else
			return -EBUSY;
	} else if (wh->ch < DATA_CHANS + GINT_CHANS) {
		// Don't have data to send, simply trigger GI event
		iowrite32(GINTTRG_TRGGI0 << (wh->ch - DATA_CHANS), priv->base + GINTTRG);
		// Since there's no ACK done for GI channels, create a timer to set txdone.
		wh->queue++;
		hrtimer_start(&wh->ack_hrt, ms_to_ktime(1), HRTIMER_MODE_REL);

	} else {
		return -EIO;	// Do not support Tx function on status channel
	}
	return 0;
}

static int wormhole_startup(struct mbox_chan *chan)
{
	struct nvt_priv *priv = container_of(chan->mbox, struct nvt_priv, mbox);
	struct nvt_wh *wh = (struct nvt_wh *)chan->con_priv;

	if (wh->ch < DATA_CHANS) {
		iowrite32(ioread32(priv->base + INTSTS) | ((INTSTS_TX0IF | INTSTS_RX0IF) << wh->ch),
			  priv->base + INTSTS);
		iowrite32(ioread32(priv->base + INTEN) | (INTEN_RXTX0_INTEN << wh->ch),
			  priv->base + INTEN);
	} else if (wh->ch < DATA_CHANS + GINT_CHANS) {
		iowrite32(ioread32(priv->base + INTSTS) | (INTSTS_GI0IF << wh->ch),
			  priv->base + INTSTS);
		iowrite32(ioread32(priv->base + INTEN) | (INTEN_GI0_INTEN << (wh->ch - DATA_CHANS)),
			  priv->base + INTEN);
	} else {
		iowrite32(INTSTS_CPSTSIF, priv->base + INTSTS);
		iowrite32(ioread32(priv->base + CPSTS), priv->base + CPSTS);
		iowrite32(ioread32(priv->base + INTEN) | INTEN_CPSTS_IEN, priv->base + INTEN);
	}
	return 0;
}

static void wormhole_shutdown(struct mbox_chan *chan)
{
	struct nvt_priv *priv = container_of(chan->mbox, struct nvt_priv, mbox);
	struct nvt_wh *wh = (struct nvt_wh *)chan->con_priv;

	if (wh->ch < DATA_CHANS) {
		iowrite32(ioread32(priv->base + INTEN) & ~(INTEN_RXTX0_INTEN << wh->ch),
			  priv->base + INTEN);
	} else if (wh->ch < DATA_CHANS + GINT_CHANS) {
		iowrite32(ioread32(priv->base + INTEN) & ~(INTEN_GI0_INTEN << (wh->ch - DATA_CHANS)),
			  priv->base + INTEN);
	} else {
		iowrite32(ioread32(priv->base + INTEN) & ~INTEN_CPSTS_IEN, priv->base + INTEN);
	}
}


static bool wormhole_peek_data(struct mbox_chan *chan)
{
	struct nvt_priv *priv = container_of(chan->mbox, struct nvt_priv, mbox);
	struct nvt_wh *wh = (struct nvt_wh *)chan->con_priv;

	if (wh->ch < DATA_CHANS) {
		// Check Rx ready flag
		if (ioread32(priv->base + RXSTS) & (RXSTS_CH0RDY << wh->ch))
			return true;
	}

	return false; // No data in Rx or general interrupt channel
}


static const struct mbox_chan_ops wormhole_ops = {
	.send_data = wormhole_send_data,
	.startup = wormhole_startup,
	.shutdown = wormhole_shutdown,
	.peek_data = wormhole_peek_data,
};

static enum hrtimer_restart txdone_hrtimer(struct hrtimer *hrtimer)
{
	struct nvt_wh *wh = container_of(hrtimer, struct nvt_wh, ack_hrt);
	struct mbox_chan *chan = wh->chan;

	mbox_chan_txdone(chan, 0);

	// Return true for GI channel, or TX channel ready
	if (--wh->queue == 0)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(hrtimer, ms_to_ktime(1));
	return HRTIMER_RESTART;
}


static int wormhole_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct nvt_priv *priv;
	struct nvt_wh *wh;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	priv->clk = devm_clk_get(&pdev->dev, "wh0_gate");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, priv->irq, wormhole_isr, 0, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "Failed to request irq\n");
		return ret;
	}
	priv->mbox.dev = dev;
	priv->mbox.ops = &wormhole_ops;
	priv->mbox.txdone_irq = true;
	priv->mbox.num_chans = DATA_CHANS + GINT_CHANS + STS_CHAN;
	priv->mbox.chans = devm_kcalloc(dev, priv->mbox.num_chans,
					     sizeof(*priv->mbox.chans),
					     GFP_KERNEL);
	if (!priv->mbox.chans) {
		clk_disable_unprepare(priv->clk);
		return -ENOMEM;
	}

	/* Enable wakeup from by wormhole interrupt?*/
	if (of_property_read_bool(node, "enable-wakeup")) {
		device_init_wakeup(dev, true);
		priv->wake = true;
	} else {
		device_init_wakeup(dev, false);
		priv->wake = false;
	}

	for (i = 0; i < priv->mbox.num_chans; i++) {

		wh = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);

		if (!wh) {
			clk_disable_unprepare(priv->clk);
			return -ENOMEM;
		}
		wh->ch = i;
		wh->queue = 0;
		hrtimer_init(&wh->ack_hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		wh->ack_hrt.function = txdone_hrtimer;
		wh->chan = &priv->mbox.chans[i];
		priv->mbox.chans[i].con_priv = wh;
	}
	platform_set_drvdata(pdev, priv);

	return devm_mbox_controller_register(&pdev->dev, &priv->mbox);

}

static int wormhole_remove(struct platform_device *pdev)
{
	struct nvt_priv *priv = platform_get_drvdata(pdev);
	struct nvt_wh *wh;
	int i;

	device_init_wakeup(&pdev->dev, false);
	clk_disable_unprepare(priv->clk);
	for (i = 0; i < GINT_CHANS; i++) {
		wh = (struct nvt_wh *)priv->mbox.chans[i].con_priv;
		hrtimer_cancel(&wh->ack_hrt);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void nvt_wormhole_set_irq_wake(struct device *dev, bool enable)
{
	struct nvt_priv *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		irq_set_irq_wake(priv->irq, enable);
}

static int wormhole_suspend(struct device *dev)
{
	struct nvt_priv *priv = dev_get_drvdata(dev);

	if (priv->wake) {
		// Set Tx/Rx & GI as wake up source
		iowrite32(WKCTK_GI_WKEN | WKCTK_RXTX_WKEN, priv->base + WKCTL);
		nvt_wormhole_set_irq_wake(dev, true);
	}

	return 0;
}

static int wormhole_resume(struct device *dev)
{
	struct nvt_priv *priv = dev_get_drvdata(dev);

	if (priv->wake) {
		// Clear wake up source
		nvt_wormhole_set_irq_wake(dev, false);
		iowrite32(0, priv->base + WKCTL);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(nvt_wormhole_pm_ops,
			 wormhole_suspend, wormhole_resume);


static const struct of_device_id nvt_wormhole_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-wormhole" },
	{ },
};
MODULE_DEVICE_TABLE(of, nvt_wormhole_dt_ids);

static struct platform_driver nvt_wormhole_driver = {
	.probe		= wormhole_probe,
	.remove		= wormhole_remove,
	.driver = {
		.name	= "ma35d1-wormhole",
		.of_match_table = nvt_wormhole_dt_ids,
		.pm = &nvt_wormhole_pm_ops,
	},
};
module_platform_driver(nvt_wormhole_driver);

MODULE_DESCRIPTION("Wormhole driver for Nuvoton MA35D1");
MODULE_LICENSE("GPL v2");
