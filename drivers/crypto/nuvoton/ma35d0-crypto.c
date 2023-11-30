// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/ma35d0-crypto.c
 *
 * Copyright (c) 2023 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "ma35d0-crypto.h"

static struct nu_crypto_dev *_ma35d0_crypto_device;

static irqreturn_t ma35d0_crypto_irq(int irq, void *data)
{
	struct nu_crypto_dev  *nu_cryp_dev =
					(struct nu_crypto_dev *)data;
	struct nu_aes_dev  *aes_dd = &nu_cryp_dev->aes_dd;
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	u32  status, aes_sts, ret = IRQ_NONE;

	status = readl_relaxed(nu_cryp_dev->reg_base + INTSTS);
	if (status & (INTSTS_AESIF | INTSTS_AESEIF)) {
		aes_sts = readl_relaxed(nu_cryp_dev->reg_base + AES_STS);
		if (aes_sts & (AES_STS_KSERR | AES_STS_BUSERR))
			pr_err("AES H/W error: AES_STS = 0x%x!\n", aes_sts);
		writel_relaxed(INTSTS_AESIF | INTSTS_AESEIF,
				nu_cryp_dev->reg_base + INTSTS);
		tasklet_schedule(&aes_dd->done_task);
		ret = IRQ_HANDLED;
	}

	if (status & (INTSTS_HMACIF | INTSTS_HMACEIF)) {
		writel_relaxed(INTSTS_HMACIF | INTSTS_HMACEIF,
				nu_cryp_dev->reg_base + INTSTS);
		tasklet_schedule(&sha_dd->done_task);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static int ma35d0_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nu_crypto_dev *nu_cryp_dev;
	struct resource *res;
	int   irq;
	const char  *str;
	int   err;

	/* MA35D0 crypto engine clock should be enabled by firmware. */

	nu_cryp_dev = devm_kzalloc(dev, sizeof(*nu_cryp_dev), GFP_KERNEL);
	if (!nu_cryp_dev)
		return -ENOMEM;

	dev_set_drvdata(dev, nu_cryp_dev);
	_ma35d0_crypto_device = nu_cryp_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nu_cryp_dev->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(nu_cryp_dev->reg_base)) {
		err = PTR_ERR(nu_cryp_dev->reg_base);
		return err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "Failed to get Crypto irq!\n");
		return -ENODEV;
	}
	err = devm_request_irq(dev, irq, ma35d0_crypto_irq, IRQF_SHARED,
			       "ma35d0-crypto", nu_cryp_dev);
	if (err) {
		dev_err(dev, "Failed to request IRQ%d: err: %d.\n", irq, err);
		return err;
	}

	err = ma35d0_prng_probe(dev, nu_cryp_dev->reg_base, &nu_cryp_dev->prng);
	if (err)
		dev_err(dev, "failed to init PRNG!\n");

	err = ma35d0_aes_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init AES!\n");

	err = ma35d0_sha_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init SHA!\n");

#ifdef CONFIG_CRYPTO_ECDH
	nu_cryp_dev->ecc_ioctl = false;
	if (!of_property_read_string(dev->of_node, "ecc_ioctl", &str)) {
		if (!strcmp("yes", str))
			nu_cryp_dev->ecc_ioctl = true;
	}
	err = ma35d0_ecc_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init ECC!\n");
#endif

#ifdef CONFIG_CRYPTO_RSA
	nu_cryp_dev->rsa_ioctl = false;
	if (!of_property_read_string(dev->of_node, "rsa_ioctl", &str)) {
		if (!strcmp("yes", str))
			nu_cryp_dev->rsa_ioctl = true;
	}
	err = ma35d0_rsa_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init RSA!\n");
#endif
	return 0;
}

static int ma35d0_crypto_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nu_crypto_dev *nu_cryp_dev;

	nu_cryp_dev = dev_get_drvdata(dev);
	if (!nu_cryp_dev)
		return -ENODEV;

	ma35d0_prng_remove(&pdev->dev, nu_cryp_dev->prng);
	ma35d0_aes_remove(&pdev->dev, nu_cryp_dev);
	ma35d0_sha_remove(&pdev->dev, nu_cryp_dev);

#ifdef CONFIG_CRYPTO_ECDH
	ma35d0_ecc_remove(&pdev->dev, nu_cryp_dev);
#endif

#ifdef CONFIG_CRYPTO_RSA
	ma35d0_rsa_remove(&pdev->dev, nu_cryp_dev);
#endif
	devm_kfree(dev, nu_cryp_dev);
	return 0;
}

static const struct of_device_id ma35d0_crypto_of_match[] = {
	{ .compatible = "nuvoton,ma35d0-crypto" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d0_crypto_of_match);


static struct platform_driver ma35d0_crypto_driver = {
	.probe  = ma35d0_crypto_probe,
	.remove = ma35d0_crypto_remove,
	.driver = {
		.name   = "ma35d0-crypto",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d0_crypto_of_match),
	},
};

module_platform_driver(ma35d0_crypto_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("Nuvoton Cryptographic Accerlerator");
MODULE_LICENSE("GPL");
