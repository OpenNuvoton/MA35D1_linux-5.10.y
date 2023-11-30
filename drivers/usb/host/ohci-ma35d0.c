// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/usb/host/ohci-ma35d0.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb/isp1301.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d0-sys.h>

#include "ohci.h"


#define DRIVER_DESC "Nuvoton MA35D0 OHCI driver"

static const char hcd_name[] = "ohci-ma35d0";
static struct hc_driver __read_mostly ohci_ma35d0_hc_driver;


/* interface and function clocks */
#define hcd_to_ma35d0_ohci_priv(h) \
	((struct ma35d0_ohci_priv *)hcd_to_ohci(h)->priv)

struct ma35d0_ohci_priv {
	int	id;
	struct regmap *sysregmap;
	struct clk *clk;
};

static int ohci_hcd_ma35d0_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd = 0;
	const struct hc_driver *driver = &ohci_ma35d0_hc_driver;
	struct ma35d0_ohci_priv *ma35d0_ohci;
	struct resource *res;
	int ret = 0, irq;

	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dev_dbg(&pdev->dev, "%s: " DRIVER_DESC " (nxp)\n", hcd_name);
	if (usb_disabled()) {
		dev_err(&pdev->dev, "USB is disabled\n");
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Failed to allocate HC buffer\n");
		ret = -ENOMEM;
		goto fail_hcd;
	}

	ma35d0_ohci = hcd_to_ma35d0_ohci_priv(hcd);

	ma35d0_ohci->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ma35d0_ohci->clk)) {
		ret = PTR_ERR(ma35d0_ohci->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", ret);
		return -ENOENT;
	}
	ret = clk_prepare_enable(ma35d0_ohci->clk);
	if (ret)
		return -ENOENT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto fail_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = -ENXIO;
		goto fail_resource;
	}

	platform_set_drvdata(pdev, hcd);

	dev_info(&pdev->dev, "at 0x%p, irq %d\n", hcd->regs, hcd->irq);
	ret = usb_add_hcd(hcd, irq, 0);
	if (ret == 0) {
		device_wakeup_enable(hcd->self.controller);
		return ret;
	}

fail_resource:
	usb_put_hcd(hcd);
fail_hcd:
	clk_disable_unprepare(ma35d0_ohci->clk);
	return ret;
}

static int ohci_hcd_ma35d0_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ma35d0_ohci_priv *ma35d0_ohci = hcd_to_ma35d0_ohci_priv(hcd);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	clk_disable_unprepare(ma35d0_ohci->clk);

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:usb-ohci");

#ifdef CONFIG_OF
static const struct of_device_id ohci_hcd_ma35d0_match[] = {
	{ .compatible = "nuvoton,ma35d0-ohci" },
	{},
};
MODULE_DEVICE_TABLE(of, ohci_hcd_ma35d0_match);
#endif

static struct platform_driver ohci_hcd_ma35d0_driver = {
	.driver = {
		.name = "usb-ohci",
		.of_match_table = of_match_ptr(ohci_hcd_ma35d0_match),
	},
	.probe = ohci_hcd_ma35d0_probe,
	.remove = ohci_hcd_ma35d0_remove,
};

static int __init ohci_ma35d0_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ohci_init_driver(&ohci_ma35d0_hc_driver, NULL);
	return platform_driver_register(&ohci_hcd_ma35d0_driver);
}
module_init(ohci_ma35d0_init);

static void __exit ohci_ma35d0_cleanup(void)
{
	platform_driver_unregister(&ohci_hcd_ma35d0_driver);
}
module_exit(ohci_ma35d0_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:ma35d0-ohci");
MODULE_LICENSE("GPL v2");
