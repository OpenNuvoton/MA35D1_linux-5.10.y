// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-crypto.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
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
#include <linux/tee_drv.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "nuvoton-crypto.h"

static struct nuvoton_crypto_dev  *nvt_crypto_dev;

#ifdef CONFIG_OPTEE
static int optee_crypto_probe(struct device *dev)
{
	struct tee_client_device *tee_cdev = to_tee_client_device(dev);
	struct tee_context  *ctx;
	u32	session_id;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_param param[4];
	int  ret;

	if (nvt_crypto_dev == NULL)
		return -ENODEV;

	nvt_crypto_dev->tee_cdev = tee_cdev;

	/*
	 * Open context with TEE driver
	 */
	ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
	if (IS_ERR(ctx)) {
		dev_err(dev, "tee_client_open_context failed.\n");
		return -ENODEV;
	}

	/*
	 * Open session with Crypto Trusted App
	 */
	memcpy(sess_arg.uuid, tee_cdev->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;
	ret = tee_client_open_session(ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		tee_client_close_context(ctx);
		return -EINVAL;
	}
	session_id = sess_arg.session;

	/*
	 * Invoke PTA_CMD_CRYPTO_INIT function of Trusted App
	 */
	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));
	inv_arg.func = PTA_CMD_CRYPTO_INIT;
	inv_arg.session = session_id;
	inv_arg.num_params = 4;
	ret = tee_client_invoke_func(ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PTA_CMD_CRYPTO_INIT invoke err: %x\n",
			inv_arg.ret);
		ret = -EINVAL;
	}
	tee_client_close_session(ctx, session_id);
	tee_client_close_context(ctx);
	return ret;
}

static int optee_crypto_remove(struct device *dev)
{
	return 0;
}

static const struct tee_client_device_id optee_crypto_id_table[] = {
	{UUID_INIT(0x61d3c750, 0x9e72, 0x46b6,
		   0x85, 0x7c, 0x46, 0xfa, 0x51, 0x27, 0x32, 0xac)},
	{}
};

MODULE_DEVICE_TABLE(tee, optee_crypto_id_table);

static struct tee_client_driver optee_crypto_driver = {
	.id_table	= optee_crypto_id_table,
	.driver		= {
		.name		= "optee-nvt-crypto",
		.bus		= &tee_bus_type,
		.probe		= optee_crypto_probe,
		.remove		= optee_crypto_remove,
	},
};
#endif  /* CONFIG_OPTEE */

static irqreturn_t nuvoton_crypto_irq(int irq, void *data)
{
	struct nuvoton_crypto_dev  *nu_cryp_dev =
					(struct nuvoton_crypto_dev *)data;
	struct nu_aes_dev  *aes_dd = &nu_cryp_dev->aes_dd;
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	u32  status, ret = IRQ_NONE;

	status = readl_relaxed(nu_cryp_dev->reg_base + INTSTS);

	if (status & (INTSTS_AESIF | INTSTS_AESEIF)) {
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

static int nuvoton_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nuvoton_crypto_dev  *nu_cryp_dev;
	struct resource *res;
#ifdef CONFIG_OPTEE
	const char  *optee_sel;
#endif
	int   irq;
	int   err;

	/* MA35D1 crypto engine clock should be enabled by firmware. */

	nu_cryp_dev = devm_kzalloc(&pdev->dev, sizeof(*nu_cryp_dev),
				GFP_KERNEL);
	if (!nu_cryp_dev)
		return -ENOMEM;
	memset(nu_cryp_dev, 0, sizeof(*nu_cryp_dev));

	nvt_crypto_dev = nu_cryp_dev;

	platform_set_drvdata(pdev, nu_cryp_dev);

	nu_cryp_dev->use_optee = false;

#ifdef CONFIG_OPTEE
	if (!of_property_read_string(dev->of_node, "optee_nuvoton",
	    &optee_sel)) {
		if (!strcmp("yes", optee_sel))
			nu_cryp_dev->use_optee = true;
	}
#endif
	if (nu_cryp_dev->use_optee) {
#ifdef CONFIG_OPTEE
		pr_info("Register MA35D1 Crypto optee client driver.\n");
		err = driver_register(&optee_crypto_driver.driver);
		if (err) {
			pr_err("Failed to register crypto optee driver!\n");
			return err;
		}
#endif
	} else {
		/*
		 *  Get register base
		 */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		nu_cryp_dev->reg_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(nu_cryp_dev->reg_base)) {
			err = PTR_ERR(nu_cryp_dev->reg_base);
			return err;
		}

		/*
		 *  Get irq number and install irq handler
		 */
		irq = platform_get_irq(pdev, 0);
		if (irq <= 0) {
			dev_err(dev, "Failed to get Crypto irq!\n");
			return -ENODEV;
		}
		err = devm_request_irq(dev, irq, nuvoton_crypto_irq,
				       IRQF_SHARED, "nuvoton-crypto",
				       nu_cryp_dev);
		if (err) {
			dev_err(dev, "Failed to request IRQ%d: err: %d.\n",
				irq, err);
			return err;
		}
	}

#ifndef CONFIG_HW_RANDOM_MA35D1
	if (nu_cryp_dev->use_optee == false) {
		err = nuvoton_prng_probe(dev, nu_cryp_dev->reg_base,
					 &nu_cryp_dev->prng);
		if (err)
			dev_err(dev, "failed to init PRNG!\n");
	} else {
		pr_info("MA35D1 PRNG not used when optee enabled.\n");
	}
#else
	pr_warn("MA35D1 PRNG is not enabled when MA35D1 TRNG is used.\n");
#endif

	err = nuvoton_aes_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init AES!\n");

	err = nuvoton_sha_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init SHA!\n");

#ifdef CONFIG_CRYPTO_ECDH
	err = nuvoton_ecc_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init ECC!\n");
#endif

#ifdef CONFIG_CRYPTO_RSA
	err = nuvoton_rsa_probe(dev, nu_cryp_dev);
	if (err)
		dev_err(dev, "failed to init RSA!\n");
#endif
	return 0;
}

static int nuvoton_crypto_remove(struct platform_device *pdev)
{
	struct nuvoton_crypto_dev *nu_cryp_dev;

	nu_cryp_dev = platform_get_drvdata(pdev);
	if (!nu_cryp_dev)
		return -ENODEV;

#ifndef CONFIG_HW_RANDOM_MA35D1
	nuvoton_prng_remove(&pdev->dev, nu_cryp_dev->prng);
#endif
	nuvoton_aes_remove(&pdev->dev, nu_cryp_dev);
	nuvoton_sha_remove(&pdev->dev, nu_cryp_dev);

#ifdef CONFIG_CRYPTO_ECDH
	nuvoton_ecc_remove(&pdev->dev, nu_cryp_dev);
#endif

#ifdef CONFIG_CRYPTO_RSA
	nuvoton_rsa_remove(&pdev->dev, nu_cryp_dev);
#endif

	return 0;
}

static const struct of_device_id nuvoton_crypto_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-crypto" },
	{},
};
MODULE_DEVICE_TABLE(of, nuvoton_crypto_of_match);


static struct platform_driver nuvoton_crypto_driver = {
	.probe  = nuvoton_crypto_probe,
	.remove = nuvoton_crypto_remove,
	.driver = {
		.name   = "nuvoton-crypto",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuvoton_crypto_of_match),
	},
};

module_platform_driver(nuvoton_crypto_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("Nuvoton Cryptographic Accerlerator");
MODULE_LICENSE("GPL");
