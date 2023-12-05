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
#include <linux/tee_drv.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "ma35d0-crypto.h"

static struct nu_crypto_dev *_ma35d0_crypto_device;

#ifdef CONFIG_OPTEE
static int optee_crypto_probe(struct device *dev)
{
	struct nu_crypto_dev *nu_cryp_dev;
	struct tee_client_device *tee_cdev; // = to_tee_client_device(dev);
	struct tee_context  *ctx;
	u32	session_id;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_ioctl_open_session_arg sess_arg;
	struct tee_param param[4];
	int  ret;

	nu_cryp_dev = _ma35d0_crypto_device;

	tee_cdev = to_tee_client_device(dev);
	nu_cryp_dev->tee_cdev = tee_cdev;

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

int ma35d0_crypto_optee_init(struct nu_crypto_dev *nu_cryp_dev)
{
	int err;

	if (nu_cryp_dev->tee_cdev != NULL)
		return 0; /* already inited */

	pr_info("Register MA35D0 Crypto optee client driver.\n");
	err = driver_register(&optee_crypto_driver.driver);
	if (err) {
		pr_err("Failed to register crypto optee driver!\n");
		return err;
	}
	return 0;
}

#endif  /* CONFIG_OPTEE */

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

	nu_cryp_dev->use_optee = false;
#ifdef CONFIG_OPTEE
	if (!of_property_read_string(dev->of_node, "optee_nuvoton", &str)) {
		if (!strcmp("yes", str))
			nu_cryp_dev->use_optee = true;
	}
#endif
	if (!nu_cryp_dev->use_optee) {
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
	}

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
