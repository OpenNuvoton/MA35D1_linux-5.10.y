// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-prng.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/tee_drv.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>

#include "nuvoton-crypto.h"

#define PRNG_TIMEOUT     100

struct nuvoton_prng {
	struct device	*dev;
	void __iomem	*reg_base;
	struct hwrng	rng;
};

static inline void nu_write_reg(struct nuvoton_prng *prng, u32 val, u32 reg)
{
	writel_relaxed(val, prng->reg_base + reg);
}

static inline u32 nu_read_reg(struct nuvoton_prng *prng, u32 reg)
{
	return readl_relaxed(prng->reg_base + reg);
}


static inline int nuvoton_prng_wait_busy_clear(struct nuvoton_prng *prng)
{
	while (nu_read_reg(prng, PRNG_CTL) & PRNG_CTL_BUSY) {
		if (time_after(jiffies, jiffies +
			msecs_to_jiffies(PRNG_TIMEOUT)))
			return -EBUSY;
	}
	return 0;
}

static int nuvoton_prng_init(struct hwrng *rng)
{
	struct nuvoton_prng *prng = (struct nuvoton_prng *)rng->priv;

	// dump_trng_reg(prng);

	nu_write_reg(prng, jiffies, PRNG_SEED);

	nu_write_reg(prng, PRNG_CTL_SEEDRLD | PRNG_CTL_START, PRNG_CTL);

	return nuvoton_prng_wait_busy_clear(prng);
}

static int nuvoton_prng_read(struct hwrng *rng, void *buf,
			     size_t max, bool wait)
{
	struct nuvoton_prng *prng = (struct nuvoton_prng *)rng->priv;
	u32	*data = buf;
	int	i, err, retval;

	retval = 0;

	while (max >= 4) {
		err = nuvoton_prng_wait_busy_clear(prng);
		if (err != 0)
			return err;

		nu_write_reg(prng, PRNG_CTL_START, PRNG_CTL);

		err = nuvoton_prng_wait_busy_clear(prng);
		if (err != 0)
			return err;

		for (i = 0; i < 4; i++) {
			if (max < 4)
				break;
			*data = nu_read_reg(prng, PRNG_KEY(i));
			pr_debug("%08x ", *data);
			data++;
			max -= 4;
			retval += 4;
		}
	}
	return retval;
}


int nuvoton_prng_probe(struct device *dev, void __iomem *reg_base,
		       unsigned long *data)
{
	struct nuvoton_prng  *prng;
	int  err;

	prng = devm_kzalloc(dev, sizeof(*prng), GFP_KERNEL);
	if (!prng)
		return -ENOMEM;

	prng->dev = dev;
	prng->reg_base = reg_base;

	prng->rng.name = "nuvoton-prng";
	prng->rng.init = nuvoton_prng_init;
	prng->rng.read = nuvoton_prng_read;
	prng->rng.priv = (unsigned long)prng;

	*data = (unsigned long)prng;

	err = devm_hwrng_register(dev, &prng->rng);
	if (err)
		return err;

	pr_info("MA35D1 PRNG inited.\n");
	return 0;
}

int nuvoton_prng_remove(struct device *dev, unsigned long data)
{
	devm_kfree(dev, (void *)data);
	return 0;
}

