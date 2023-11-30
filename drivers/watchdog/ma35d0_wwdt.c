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


#define REG_WWDT_RLD		0x00		/* WWDT Reload Counter Register */
#define REG_WWDT_CR		0x04		/* WWDT Control Register */
#define REG_WWDT_SR		0x08		/* WWDT Status Register */
#define REG_WWDT_CVR		0x0C		/* WWDT Counter Value Register */

#define RELOAD_WORD	0x00005AA5

/*
 *  Select WWDT clock source from PCLK/4096.
 *  Here we set compare window to 32, and prescale to 1024 after init.
 *
 *  So WWDT time out every 2048 * 32 * (4096/125000000) = 2.1 second,
 *  And software has another 2.1 second window period to reload
 *  WWDT counter by writing RELOAD_WORD to REG_WWDT_RLD register.
 */
#define WWDT_CONFIG	0x00200F01

struct ma35d0_wwdt {
	struct watchdog_device	wdog;
	struct device		*dev;
	void __iomem		*base;
};

static struct ma35d0_wwdt *ma35d0_wwdt;

static int ma35d0wwdt_ping(struct watchdog_device *wdd)
{
	__raw_writel(RELOAD_WORD, ma35d0_wwdt->base+REG_WWDT_RLD);
	return 0;
}

static int ma35d0wwdt_start(struct watchdog_device *wdd)
{
	__raw_writel(WWDT_CONFIG, ma35d0_wwdt->base+REG_WWDT_CR);
	return 0;
}

/*
 *  This function always return error, we define it here simply because stop() is mandatory operation.
 *  Due to the fact that WWDT register can only be programmed once, so there is NO WAY OUT!!!
 */
static int ma35d0wwdt_stop(struct watchdog_device *wdd)
{

	return -EBUSY;
}

static unsigned int ma35d0wwdt_get_timeleft(struct watchdog_device *wdd)
{
	unsigned int time_left;

	time_left = __raw_readl(ma35d0_wwdt->base+REG_WWDT_CVR) / 32;

	return time_left;
}

static const struct watchdog_info ma35d0wwdt_info = {
	.identity	= "ma35d0 window watchdog",
	.options	= WDIOF_KEEPALIVEPING,
};

static struct watchdog_ops ma35d0wwdt_ops = {
	.owner = THIS_MODULE,
	.start = ma35d0wwdt_start,
	.stop = ma35d0wwdt_stop,
	.ping = ma35d0wwdt_ping,
	.get_timeleft = ma35d0wwdt_get_timeleft,
};

static struct watchdog_device ma35d0_wdd = {
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
	.info = &ma35d0wwdt_info,
	.ops = &ma35d0wwdt_ops,
	.timeout = 2,
};


static int ma35d0wwdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	const char *clkmux, *clkgate;
	struct clk *clk, *clksrc;

	ma35d0_wwdt = devm_kzalloc(&pdev->dev, sizeof(struct ma35d0_wwdt), GFP_KERNEL);
	if (!ma35d0_wwdt)
		return -ENOMEM;

	ma35d0_wwdt->dev = &pdev->dev;

	ma35d0_wwdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ma35d0_wwdt->base))
		return PTR_ERR(ma35d0_wwdt->base);

	of_property_read_string(pdev->dev.of_node, "clock-enable", &clkgate);
	clk = devm_clk_get(&pdev->dev, clkgate);
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -ENOENT) {
			dev_err(&pdev->dev, "failed to get clock gate clock\n");
			return PTR_ERR(clk);
		}
		clk = NULL;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return ret;
	}

	of_property_read_string(pdev->dev.of_node, "clock-names", &clkmux);
	clksrc = devm_clk_get(&pdev->dev, clkmux);
	if (IS_ERR(clksrc)) {
		if (PTR_ERR(clksrc) != -ENOENT) {
			dev_err(&pdev->dev, "failed to get clock source\n");
			return PTR_ERR(clksrc);
		}
		clksrc = NULL;
	}

	/* Initialize struct watchdog_device. */
	ma35d0_wdd = ma35d0_wwdt->wdog;
	ma35d0_wdd.parent = &pdev->dev;
	ma35d0_wdd.info = &ma35d0wwdt_info;
	ma35d0_wdd.ops = &ma35d0wwdt_ops;

	ret = devm_watchdog_register_device(ma35d0_wwdt->dev, &ma35d0_wdd);
	if (ret) {
		dev_err(&pdev->dev, "err register window watchdog device\n");
		return ret;
	}

	platform_set_drvdata(pdev, ma35d0_wwdt);
	return 0;

}

static int ma35d0wwdt_remove(struct platform_device *pdev)
{

	watchdog_unregister_device(&ma35d0_wdd);
	// There's no way out~~~~
	/* You can check-out any time you like
	 * But you can never leave!
	 */

	return 0;
}

#ifdef CONFIG_PM
static int ma35d0wwdt_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int ma35d0wwdt_resume(struct platform_device *dev)
{
	return 0;
}

#else
#define ma35d0wwdt_suspend NULL
#define ma35d0wwdt_resume  NULL
#endif /* CONFIG_PM */

static const struct of_device_id ma35d0_wwdt_of_match[] = {
	{ .compatible = "nuvoton,ma35d0-wwdt" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d0_wwdt_of_match);


static struct platform_driver ma35d0wwdt_driver = {
	.probe	= ma35d0wwdt_probe,
	.remove = ma35d0wwdt_remove,
	.suspend = ma35d0wwdt_suspend,
	.resume = ma35d0wwdt_resume,
	.driver = {
		.name	= "ma35d0-wwdt",
		.of_match_table = of_match_ptr(ma35d0_wwdt_of_match),
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ma35d0wwdt_driver);

MODULE_DESCRIPTION("MA35D0 Window Watchdog Timer Driver");
MODULE_LICENSE("GPL");

