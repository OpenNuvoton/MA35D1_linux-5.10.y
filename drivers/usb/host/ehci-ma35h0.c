// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/usb/host/ehci-ma35h0.c
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
#include <linux/mfd/ma35h0-sys.h>

#include "ehci.h"

#define DRIVER_DESC "Nuvoton MA35H0 EHCI driver"

static const char hcd_name[] = "ehci-ma35h0";

/* interface and function clocks */
#define hcd_to_ma35h0_ehci_priv(h) \
	((struct ma35h0_ehci_priv *)hcd_to_ehci(h)->priv)

struct ma35h0_ehci_priv {
	int id;
	struct regmap *sysregmap;
	struct clk *clk;
	int oc_active_level;
};

static struct hc_driver __read_mostly ehci_ma35h0_hc_driver;

static const struct ehci_driver_overrides ehci_ma35h0_drv_overrides __initconst = {
	.extra_priv_size = sizeof(struct ma35h0_ehci_priv),
};

static void ma35h0_start_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ma35h0_ehci_priv *ma35h0_ehci = hcd_to_ma35h0_ehci_priv(hcd);
	u32   reg, timeout = (500 / 20);

	/* USBPMISCR; HSUSBH0 & HSUSBH1 PHY */
	regmap_read(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, &reg);
	if ((reg & 0x20302) != 0x20302) {
		reg = (reg & ~0x30003) | 0x30003;
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, reg);
		msleep(20);
		reg = (reg & ~0x30003) | 0x20002;
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, reg);
		do {
			msleep(20);
			regmap_read(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, &reg);
		} while (((reg & 0x20302) != 0x20302) && (timeout-- > 0));
	}
	dev_dbg(&pdev->dev, "REG_SYS_USBPMISCR = 0x%x, timeout = %d\n", reg, timeout);

	/* set UHOVRCURH(SYS_MISCFCR0[12]) 1 => USBH Host over-current detect is high-active */
	/*                                 0 => USBH Host over-current detect is low-active  */
	regmap_read(ma35h0_ehci->sysregmap, REG_SYS_MISCFCR0, &reg);
	if (ma35h0_ehci->oc_active_level)
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_MISCFCR0, (reg | (1<<12)));
	else
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_MISCFCR0, (reg & ~(1<<12)));
}

static void ma35h0_stop_ehci(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ma35h0_ehci_priv *ma35h0_ehci = hcd_to_ma35h0_ehci_priv(hcd);

	dev_dbg(&pdev->dev, "stop\n");
	clk_disable(ma35h0_ehci->clk);
}


/*-------------------------------------------------------------------------*/

static int ehci_ma35h0_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &ehci_ma35h0_hc_driver;
	struct resource *res;
	struct ehci_hcd *ehci;
	struct ma35h0_ehci_priv *ma35h0_ehci;
	u32 rcalcode, reg;
	int irq;
	int retval;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("Initializing MA35H0 EHCI...\n");

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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		retval = PTR_ERR(hcd->regs);
		goto fail_request_resource;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	ma35h0_ehci = hcd_to_ma35h0_ehci_priv(hcd);

	ma35h0_ehci->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ma35h0_ehci->clk)) {
		retval = PTR_ERR(ma35h0_ehci->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", retval);
		retval = -ENOENT;
		goto fail_request_resource;
	}

	retval = clk_prepare_enable(ma35h0_ehci->clk);
	if (retval) {
		dev_err(&pdev->dev, "failed to enable usb host clk: %d\n", retval);
		retval = -ENOENT;
		goto fail_request_resource;
	}

	ma35h0_ehci->sysregmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				 "nuvoton,sys");
	if (!ma35h0_ehci->sysregmap) {
		dev_err(&pdev->dev, "failed to map sysreg!\n");
		retval = -ENOENT;
		goto fail_request_resource;
	}

	if (of_property_read_u32(pdev->dev.of_node, "oc-active-level", &(ma35h0_ehci->oc_active_level))) {
		ma35h0_ehci->oc_active_level = 0;
		dev_warn(&pdev->dev, "EHCI oc-active-level not found!!\n");
	}

	if (!of_property_read_u32(pdev->dev.of_node, "nuvoton,rcalcode0", &rcalcode)) {
		regmap_read(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, &reg);
		reg = (reg & 0xffff0fff) | ((rcalcode & 0xf) << 12);
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, reg);
	}

	if (!of_property_read_u32(pdev->dev.of_node, "nuvoton,rcalcode1", &rcalcode)) {
		regmap_read(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, &reg);
		reg = (reg & 0x0fffffff) | ((rcalcode & 0xf) << 28);
		regmap_write(ma35h0_ehci->sysregmap, REG_SYS_USBPMISCR, reg);
	}

	ma35h0_start_ehci(pdev);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;
	device_wakeup_enable(hcd->self.controller);

	return retval;

fail_add_hcd:
	ma35h0_stop_ehci(pdev);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n",
		dev_name(&pdev->dev), retval);

	return retval;
}

static int ehci_ma35h0_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	ma35h0_stop_ehci(pdev);

	return 0;
}

static int __maybe_unused ehci_ma35h0_drv_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ma35h0_ehci_priv *ma35h0_ehci = hcd_to_ma35h0_ehci_priv(hcd);
	int ret;

	ret = ehci_suspend(hcd, false);
	if (ret)
		return ret;

	clk_disable(ma35h0_ehci->clk);
	return 0;
}

static int __maybe_unused ehci_ma35h0_drv_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct ma35h0_ehci_priv *ma35h0_ehci = hcd_to_ma35h0_ehci_priv(hcd);

	clk_enable(ma35h0_ehci->clk);
	ehci_resume(hcd, false);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ma35h0_ehci_dt_ids[] = {
	{ .compatible = "nuvoton,ma35h0-ehci" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ma35h0_ehci_dt_ids);
#endif

static SIMPLE_DEV_PM_OPS(ehci_ma35h0_pm_ops, ehci_ma35h0_drv_suspend,
					ehci_ma35h0_drv_resume);

static struct platform_driver ehci_ma35h0_driver = {
	.probe		= ehci_ma35h0_drv_probe,
	.remove		= ehci_ma35h0_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name	= "ma35h0-ehci",
		.pm	= &ehci_ma35h0_pm_ops,
		.of_match_table	= of_match_ptr(ma35h0_ehci_dt_ids),
	},
};

static int __init ehci_ma35h0_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);
	ehci_init_driver(&ehci_ma35h0_hc_driver, &ehci_ma35h0_drv_overrides);
	return platform_driver_register(&ehci_ma35h0_driver);
}
module_init(ehci_ma35h0_init);

static void __exit ehci_ma35h0_cleanup(void)
{
	platform_driver_unregister(&ehci_ma35h0_driver);
}
module_exit(ehci_ma35h0_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:ma35h0-ehci");
MODULE_LICENSE("GPL v2");
