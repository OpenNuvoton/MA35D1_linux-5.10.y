// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/usb/host/ehci-ma35d1.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d1-sys.h>

#include "ehci.h"

#define DRIVER_DESC "Nuvoton MA35D1 EHCI driver"

static const char hcd_name[] = "ehci-ma35d1";

/* interface and function clocks */
#define hcd_to_ma35d1_ehci_priv(h) \
	((struct ma35d1_ehci_priv *)hcd_to_ehci(h)->priv)

struct ma35d1_ehci_priv {
	int	id;
	struct regmap *sysregmap;
	struct clk *clk;
};

static struct hc_driver __read_mostly ehci_ma35d1_hc_driver;

static const struct ehci_driver_overrides ehci_ma35d1_drv_overrides __initconst = {
	.extra_priv_size = sizeof(struct ma35d1_ehci_priv),
};

static void ma35d1_start_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ma35d1_ehci_priv *ma35d1_ehci = hcd_to_ma35d1_ehci_priv(hcd);
	u32   reg;

	ma35d1_ehci->sysregmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					"nuvoton,sys");

	/* USBPMISCR; HSUSBH0 & HSUSBH1 PHY */
	regmap_write(ma35d1_ehci->sysregmap, REG_SYS_USBPMISCR, 0x20002);

	/* set UHOVRCURH(SYS_MISCFCR0[12]) 1 => USBH Host over-current detect is high-active */
	/*                                 0 => USBH Host over-current detect is low-active  */
	regmap_read(ma35d1_ehci->sysregmap, REG_SYS_MISCFCR0, &reg);
	regmap_write(ma35d1_ehci->sysregmap, REG_SYS_MISCFCR0, (reg & ~(1<<12)));
}

static void ma35d1_stop_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ma35d1_ehci_priv *ma35d1_ehci = hcd_to_ma35d1_ehci_priv(hcd);

	dev_dbg(&pdev->dev, "stop\n");
	clk_disable(ma35d1_ehci->clk);
}


/*-------------------------------------------------------------------------*/

static int ehci_ma35d1_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &ehci_ma35d1_hc_driver;
	struct resource *res;
	struct ehci_hcd *ehci;
	struct ma35d1_ehci_priv *ma35d1_ehci;
	int irq;
	int retval;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("Initializing MA35D1 EHCI...\n");

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		retval = -ENODEV;
		goto fail_create_hcd;
	}

	/* Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	retval = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (retval)
		goto fail_create_hcd;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}
	ma35d1_ehci = hcd_to_ma35d1_ehci_priv(hcd);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		retval = PTR_ERR(hcd->regs);
		goto fail_request_resource;
	}

	ma35d1_ehci->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ma35d1_ehci->clk)) {
		retval = PTR_ERR(ma35d1_ehci->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", retval);
		return -ENOENT;
	}
	retval = clk_prepare_enable(ma35d1_ehci->clk);
	if (retval)
		return -ENOENT;

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	ehci = hcd_to_ehci(hcd);
	/* registers start at offset 0x0 */
	ehci->caps = hcd->regs;

	ma35d1_start_ehci(pdev);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;
	device_wakeup_enable(hcd->self.controller);

	// printk(KERN_INFO "PORT0 = 0x%x\n", __raw_readl(hcd->regs+0x54));

	return retval;

fail_add_hcd:
	ma35d1_stop_ehci(pdev);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n",
		dev_name(&pdev->dev), retval);

	return retval;
}

static int ehci_ma35d1_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	ma35d1_stop_ehci(pdev);

	return 0;
}

static int __maybe_unused ehci_ma35d1_drv_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ma35d1_ehci_priv *ma35d1_ehci = hcd_to_ma35d1_ehci_priv(hcd);
	int ret;

	ret = ehci_suspend(hcd, false);
	if (ret)
		return ret;

	clk_disable(ma35d1_ehci->clk);
	return 0;
}

static int __maybe_unused ehci_ma35d1_drv_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ma35d1_ehci_priv *ma35d1_ehci = hcd_to_ma35d1_ehci_priv(hcd);

	clk_enable(ma35d1_ehci->clk);
	ehci_resume(hcd, false);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ma35d1_ehci_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-ehci0" },
	{ .compatible = "nuvoton,ma35d1-ehci1" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ma35d1_ehci_dt_ids);
#endif

static SIMPLE_DEV_PM_OPS(ehci_ma35d1_pm_ops, ehci_ma35d1_drv_suspend,
					ehci_ma35d1_drv_resume);

static struct platform_driver ehci_ma35d1_driver = {
	.probe		= ehci_ma35d1_drv_probe,
	.remove		= ehci_ma35d1_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "ma35d1-ehci",
		.pm	= &ehci_ma35d1_pm_ops,
		.of_match_table	= of_match_ptr(ma35d1_ehci_dt_ids),
	},
};

static int __init ehci_ma35d1_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	ehci_init_driver(&ehci_ma35d1_hc_driver, &ehci_ma35d1_drv_overrides);
	return platform_driver_register(&ehci_ma35d1_driver);
}
module_init(ehci_ma35d1_init);

static void __exit ehci_ma35d1_cleanup(void)
{
	platform_driver_unregister(&ehci_ma35d1_driver);
}
module_exit(ehci_ma35d1_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:ma35d1-ehci");
MODULE_LICENSE("GPL v2");
