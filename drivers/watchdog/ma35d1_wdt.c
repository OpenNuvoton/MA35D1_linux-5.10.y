// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2020 Nuvoton technology corporation.
 *
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/of.h>

#include <linux/mfd/ma35d1-sys.h>
#include <dt-bindings/reset/nuvoton,ma35d1-reset.h>

#define REG_WDT_CTL		0x00		/* Watchdog Control Register */
#define REG_WDT_ALTCTL		0x04		/* WDT Alternative Control Register */
#define REG_WDT_RSTCNT		0x08		/* WDT Reset Counter Register */

#define TOUTSEL			(0x0f << 8)     /* wdt interval selection */
#define WDTEN			(0x01 << 7)	/* wdt enable*/
#define INTEN			(0x01 << 6)
#define WKF			(0x01 << 5)
#define WKEN			(0x01 << 4)
#define IF			(0x01 << 3)
#define RSTF			(0x01 << 2)	/* wdt reset flag */
#define RSTEN			(0x01 << 1)	/* wdt reset enable */
/*
 * Assumming 32k crystal is configured as the watchdog clock source,
 * the time out interval can be calculated via following formula:
 * TOUTSEL		real time interval (formula)
 * 0x05		(2^ 14 * (32k crystal freq))seconds = 0.53 sec
 * 0x06		(2^ 16 * (32k crystal freq))seconds = 2.03 sec
 * 0x07		(2^ 18 * (32k crystal freq))seconds = 8.03 sec
 */
#define WDT_HW_TIMEOUT		0x05

#define RESET_COUNTER		0x00005AA5

static int heartbeat;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeats in seconds. (default = "
	__MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct ma35d1_wdt {
	struct watchdog_device	wdog;
	struct device		*dev;
	void __iomem		*base;
	struct clk		*clk;
	struct clk		*eclk;
	u32			wkupen;
	u32			irq;
};

static struct ma35d1_wdt *ma35d1_wdt;


static int ma35d1wdt_ping(struct watchdog_device *wdd)
{
	__raw_writel(RESET_COUNTER, ma35d1_wdt->base+REG_WDT_RSTCNT);
	return 0;
}


static int ma35d1wdt_start(struct watchdog_device *wdd)
{
	unsigned int val = RSTEN | WDTEN;
	unsigned long flags;


	if (ma35d1_wdt->wkupen == 1) {

		val |= INTEN;
		val |= WKEN;

		if (wdd->timeout < 2)
			val |= 0x5 << 8;
		else if (wdd->timeout < 8)
			val |= 0x6 << 8;
		else
			val |= 0x7 << 8;

	} else {

		if (wdd->timeout < 2)
			val |= 0x5 << 8;
		else if (wdd->timeout < 11)
			val |= 0x6 << 8;
		else
			val |= 0x7 << 8;
	}

	local_irq_save(flags);
	ma35d1_reg_unlock();
	__raw_writel(val, ma35d1_wdt->base+REG_WDT_CTL);
	ma35d1_reg_lock();
	local_irq_restore(flags);
	__raw_writel(RESET_COUNTER, ma35d1_wdt->base+REG_WDT_RSTCNT);

	return 0;
}

static int ma35d1wdt_stop(struct watchdog_device *wdd)
{
	unsigned long flags;

	pr_warn("Stopping WDT is probably not a good idea\n");

	local_irq_save(flags);
	ma35d1_reg_unlock();
	__raw_writel(0, ma35d1_wdt->base+REG_WDT_CTL);
	ma35d1_reg_lock();
	local_irq_restore(flags);
	return 0;
}


static int ma35d1wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	unsigned int val;
	unsigned long flags;

	val = __raw_readl(ma35d1_wdt->base+REG_WDT_CTL);
	val &= ~TOUTSEL;

	if (ma35d1_wdt->wkupen == 1) {

		if (timeout < 2)
			val |= 0x5 << 8;
		else if (timeout < 8)
			val |= 0x6 << 8;
		else
			val |= 0x7 << 8;

	} else {

		if (timeout < 2)
			val |= 0x5 << 8;   /* 0.512s */
		else if (timeout < 5)
			val |= 0x6 << 8;   /* 2.048s */
		else if (timeout < 10)
			val |= 0x7 << 8;   /* 8.192s */
		else
			val |= 0x8 << 8;   /* 32.768s */
	}

	local_irq_save(flags);
	ma35d1_reg_unlock();
	__raw_writel(val, ma35d1_wdt->base+REG_WDT_CTL);
	ma35d1_reg_lock();
	local_irq_restore(flags);

	return 0;
}

static const struct watchdog_info ma35d1wdt_info = {
	.identity	= "ma35d1 watchdog",
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static struct watchdog_ops ma35d1wdt_ops = {
	.owner = THIS_MODULE,
	.start = ma35d1wdt_start,
	.stop = ma35d1wdt_stop,
	.ping = ma35d1wdt_ping,
	.set_timeout = ma35d1wdt_set_timeout,
};

static struct watchdog_device ma35d1_wdd = {
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info = &ma35d1wdt_info,
	.ops = &ma35d1wdt_ops,
};

static irqreturn_t ma35d1_wdt_interrupt(int irq, void *dev_id)
{
	__raw_writel(RESET_COUNTER, ma35d1_wdt->base+REG_WDT_RSTCNT);

	ma35d1_reg_unlock();
	if (__raw_readl(ma35d1_wdt->base+REG_WDT_CTL) & IF)
		__raw_writel(__raw_readl(ma35d1_wdt->base+REG_WDT_CTL) | IF, ma35d1_wdt->base+REG_WDT_CTL);

	ma35d1_reg_lock();

	return IRQ_HANDLED;
};

static int ma35d1wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	const char *clkmux, *clkgate;

	ma35d1_wdt = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_wdt), GFP_KERNEL);
	if (!ma35d1_wdt)
		return -ENOMEM;

	ma35d1_wdt->dev = &pdev->dev;

	ma35d1_wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ma35d1_wdt->base))
		return PTR_ERR(ma35d1_wdt->base);

	of_property_read_string(pdev->dev.of_node, "clock-enable", &clkgate);
	ma35d1_wdt->clk = devm_clk_get(&pdev->dev, clkgate);
	if (IS_ERR(ma35d1_wdt->clk)) {
		if (PTR_ERR(ma35d1_wdt->clk) != -ENOENT) {
			dev_err(&pdev->dev, "failed to get clock gate clock\n");
			return PTR_ERR(ma35d1_wdt->clk);
		}
		ma35d1_wdt->clk = NULL;
	}

	ret = clk_prepare_enable(ma35d1_wdt->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	of_property_read_string(pdev->dev.of_node, "clock-names", &clkmux);
	ma35d1_wdt->eclk = devm_clk_get(&pdev->dev, clkmux);
	if (IS_ERR(ma35d1_wdt->eclk)) {
		if (PTR_ERR(ma35d1_wdt->eclk) != -ENOENT)
			return PTR_ERR(ma35d1_wdt->eclk);

		ma35d1_wdt->eclk = NULL;
	}

	ma35d1_wdt->irq = platform_get_irq(pdev, 0);

	of_property_read_u32(pdev->dev.of_node, "wakeup-enable", &ma35d1_wdt->wkupen);

	/* Initialize struct watchdog_device. */
	ma35d1_wdd = ma35d1_wdt->wdog;
	ma35d1_wdd.parent = &pdev->dev;
	ma35d1_wdd.info = &ma35d1wdt_info;
	ma35d1_wdd.ops = &ma35d1wdt_ops;

	if (ma35d1_wdt->wkupen == 1) {

		heartbeat = 8;
		ma35d1_wdd.timeout = 8;	// default time out = 2 sec
		ma35d1_wdd.min_timeout = 1;	// min time out = 1 sec
		ma35d1_wdd.max_timeout = 8;	// max time out = 8 sec
	} else {

		heartbeat = 11;
		ma35d1_wdd.timeout = 11;	// default time out = 11.2 sec
		ma35d1_wdd.min_timeout = 1;	// min time out = 1.4 sec
		ma35d1_wdd.max_timeout = 33;	// max time out = 32.768 sec
	}
	watchdog_set_drvdata(&ma35d1_wdd, ma35d1_wdt);
	watchdog_set_nowayout(&ma35d1_wdd, nowayout);
	watchdog_init_timeout(&ma35d1_wdd, heartbeat, &pdev->dev);

	ret = devm_watchdog_register_device(ma35d1_wdt->dev, &ma35d1_wdd);
	if (ret) {
		dev_err(&pdev->dev, "err register watchdog device\n");
		clk_disable(ma35d1_wdt->clk);
		return ret;
	}

	platform_set_drvdata(pdev, ma35d1_wdt);

	return 0;
}

static int ma35d1wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&ma35d1_wdd);

	clk_disable(ma35d1_wdt->eclk);
	clk_disable(ma35d1_wdt->clk);

	return 0;
}

static void ma35d1wdt_shutdown(struct platform_device *pdev)
{
	ma35d1wdt_stop(&ma35d1_wdd);
}

static u32 reg_save;
static int __maybe_unused ma35d1wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	if (ma35d1_wdt->wkupen == 1) {

		unsigned long flags;

		reg_save = __raw_readl(ma35d1_wdt->base+REG_WDT_CTL);

		local_irq_save(flags);
		ma35d1_reg_unlock();
		__raw_writel(__raw_readl(ma35d1_wdt->base+REG_WDT_CTL) & ~RSTEN, ma35d1_wdt->base+REG_WDT_CTL); //Disable WDT reset
		ma35d1_reg_lock();
		local_irq_restore(flags);

		if (devm_request_irq(&dev->dev, ma35d1_wdt->irq, ma35d1_wdt_interrupt, 0, dev_name(&dev->dev), ma35d1_wdt))
			return -EBUSY;

		enable_irq_wake(ma35d1_wdt->irq);
	}

	return 0;
}

static int __maybe_unused ma35d1wdt_resume(struct platform_device *dev)
{
	if (ma35d1_wdt->wkupen == 1) {
		unsigned long flags;

		local_irq_save(flags);
		ma35d1_reg_unlock();
		__raw_writel(reg_save, ma35d1_wdt->base+REG_WDT_CTL);
		ma35d1_reg_lock();
		local_irq_restore(flags);

		disable_irq_wake(ma35d1_wdt->irq);
		free_irq(ma35d1_wdt->irq, NULL);
	}
	return 0;
}


static const struct of_device_id ma35d1_wdt_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_wdt_dt_ids);

static struct platform_driver ma35d1wdt_driver = {
	.probe		= ma35d1wdt_probe,
	.remove		= ma35d1wdt_remove,
	.shutdown	= ma35d1wdt_shutdown,
	.suspend	= ma35d1wdt_suspend,
	.resume		= ma35d1wdt_resume,
	.driver		= {
		.name	= "ma35d1-wdt",
		.of_match_table = of_match_ptr(ma35d1_wdt_dt_ids),
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ma35d1wdt_driver);

MODULE_DESCRIPTION("MA35D1 Watchdog Timer Driver");
MODULE_LICENSE("GPL");

