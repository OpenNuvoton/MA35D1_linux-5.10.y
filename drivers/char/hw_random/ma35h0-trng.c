// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019 Linaro Ltd.
 * Copyright (C) 2023, Nuvoton Technology Corporation
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/of_platform.h>

//#define USE_GEN_NONCE

/*--------------------------------------------------------------*/
/*  MA35H0 TRNG registers                                      */
/*--------------------------------------------------------------*/
#define CTRL			0x000
#define CTRL_CMD_OFFSET			(0)
#define CTRL_CMD_MASK			(0xf << 0)

#define MODE			0x004
#define MODE_SEC_ALG			(0x1 << 0)
#define MODE_PRED_RESET			(0x1 << 3)
#define MODE_ADDIN_PRESENT		(0x1 << 4)
#define MODE_KAT_VEC_OFFSET		(5)
#define MODE_KAT_VEC_MASK		(0x3 << 5)
#define MODE_KAT_SEL_OFFSET		(7)
#define MODE_KAT_SEL_MASK		(0x3 << 7)

#define SMODE			0x008
#define SMODE_NONCE			(0x1 << 0)
#define SMODE_MISSION_MODE		(0x1 << 1)
#define SMODE_MAX_REJECTS_OFFSET	(2)
#define SMODE_MAX_REJECTS_MASK		(0xff << 2)
#define SMODE_INDIV_HT_DISABLE_OFFSET	(16)
#define SMODE_INDIV_HT_DISABLE_MASK	(0xff << 16)
#define SMODE_NOISE_COLLECT		(0x1 << 31)

#define STAT			0x00C
#define STAT_LAST_CMD_OFFSET		(0)
#define STAT_LAST_CMD_MASK		(0xf << 0)
#define STAT_SEC_ALG			(0x1 << 4)
#define STAT_NONCE_MODE			(0x1 << 5)
#define STAT_MISSION_MODE		(0x1 << 6)
#define STAT_DRBG_STATE_OFFSET		(7)
#define STAT_DRBG_STATE_MASK		(0x3 << 7)
#define STAT_STARTUP_TEST_STUCK		(0x1 << 9)
#define STAT_STARTUP_TEST_IN_PROG	(0x1 << 10)
#define STAT_BUSY			(0x1 << 31)

#define IE			0x010
#define IE_ZEROIZED			(0x1 << 0)
#define IE_KAT_COMPLETED		(0x1 << 1)
#define IE_NOISE_RDY			(0x1 << 2)
#define IE_ALARMS			(0x1 << 3)
#define IE_DONE				(0x1 << 4)
#define IE_GLBL				(0x1 << 31)

#define ISTAT			0x014
#define ISTAT_ZEROIZED			(0x1 << 0)
#define ISTAT_KAT_COMPLETED		(0x1 << 1)
#define ISTAT_NOISE_RDY			(0x1 << 2)
#define ISTAT_ALARMS			(0x1 << 3)
#define ISTAT_DONE			(0x1 << 4)

#define ALARMS			0x018
#define ALARMS_FAILED_TEST_ID_OFFSET	(0)
#define ALARMS_FAILED_TEST_ID_MASK	(0xf << 0)
#define ALARMS_ILLEGAL_CMD_SEQ		(0x1 << 4)
#define ALARMS_FAILED_SEED_ST_HT	(0x1 << 5)

#define COREKIT_REL		0x01C
#define COREKIT_REL_REL_NUM_OFFSET	(0)
#define COREKIT_REL_REL_NUM_MASK	(0xffff << 0)
#define COREKIT_REL_EXT_VER_OFFSET	(16)
#define COREKIT_REL_EXT_VER_MASK	(0xff << 16)
#define COREKIT_REL_EXT_ENUM_OFFSET	(28)
#define COREKIT_REL_EXT_ENUM_MASK	(0xf << 28)

#define FEATURES		0x020
#define FEATURES_SECURE_RST_STATE	(0x1 << 0)
#define FEATURES_DIAG_LEVEL_ST_HLT_OFFSET (1)
#define FEATURES_DIAG_LEVEL_ST_HLT_MASK	(0x7 << 1)
#define FEATURES_DIAG_LEVEL_CLP800_OFFSET (4)
#define FEATURES_DIAG_LEVEL_CLP800_MASK	(0x7 << 4)
#define FEATURES_DIAG_LEVEL_NS		(0x1 << 7)
#define FEATURES_PS_PRESENT		(0x1 << 8)
#define FEATURES_AES_256		(0x1 << 9)
#define RAND(x)			(0x024 + ((x) * 0x04))
#define RAND_WCNT			4
#define NPA_DATA(x)		(0x034 + ((x) * 0x04))
#define NPA_DATA_WCNT			16
#define SEED(x)			(0x074 + ((x) * 0x04))
#define SEED_WCNT			12
#define TIME_TO_SEED		0x0d0
#define BUILD_CFG0		0x0f0
#define BUILD_CFG1		0x0f4

struct ma35h0_trng {
	struct device		*dev;
	void __iomem		*reg_base;
	struct hwrng		rng;
};

/*
 *  CTL CMD[3:0]  commands
 */
#define TCMD_NOP		0x0 /* Execute a NOP */
#define TCMD_GEN_NOISE		0x1 /* Generate ful-entropy seed from noise  */
#define TCMD_GEN_NONCE		0x2 /* Generate seed from host written nonce */
#define TCMD_CREATE_STATE	0x3 /* Move DRBG to create state  */
#define TCMD_RENEW_STATE	0x4 /* Move DRBG to renew state   */
#define TCMD_REFRESH_ADDIN	0x5 /* Move DRBG to refresh addin */
#define TCMD_GEN_RANDOM		0x6 /* Generate a random number   */
#define TCMD_ADVANCE_STATE	0x7 /* Advance DRBG state         */
#define TCMD_RUN_KAT		0x8 /* Run KAT on DRBG or entropy source */
#define TCMD_ZEROIZE		0xf /* Zeroize                    */

#define TRNG_TIMEOUT		(2000)

/*-------------------------------------------------------------------*/
/*  hwrng driver for MA35H0 TRNG                                    */
/*-------------------------------------------------------------------*/

static inline void nu_write_reg(struct ma35h0_trng *tdev, u32 val, u32 reg)
{
	writel_relaxed(val, tdev->reg_base + reg);
}

static inline u32 nu_read_reg(struct ma35h0_trng *tdev, u32 reg)
{
	return readl_relaxed(tdev->reg_base + reg);
}

static inline int ma35h0_trng_wait_busy_clear(struct ma35h0_trng *tdev)
{
	unsigned long  timeout = jiffies + msecs_to_jiffies(TRNG_TIMEOUT);

	while (nu_read_reg(tdev, STAT) & STAT_BUSY) {
		if (time_after(jiffies, timeout))
			return -EBUSY;
	}
	return 0;
}

static int ma35h0_trng_issue_command(struct ma35h0_trng *tdev, int cmd)
{
	unsigned long  timeout;
	int	err;

	err = ma35h0_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	nu_write_reg(tdev, (nu_read_reg(tdev, CTRL) & ~CTRL_CMD_MASK) |
			(cmd << CTRL_CMD_OFFSET), CTRL);

	timeout = jiffies + msecs_to_jiffies(TRNG_TIMEOUT);
	while (!(nu_read_reg(tdev, ISTAT) & ISTAT_DONE)) {
		if (time_after(jiffies, timeout)) {
			pr_debug("TRNG command %d timeout! ISTAT = 0x%x, SMODE = 0x%x.\n",
				 cmd, nu_read_reg(tdev, ISTAT),
				 nu_read_reg(tdev, SMODE));
			return -EBUSY;
		}
	}
	return 0;
}

#ifdef USE_GEN_NONCE
static int ma35h0_trng_gen_nonce(struct ma35h0_trng *tdev, uint32_t *nonce)
{
	int   i, j, loop, err;

	nu_write_reg(tdev, nu_read_reg(tdev, SMODE) | SMODE_NONCE, SMODE);

	if (nu_read_reg(tdev, MODE) & MODE_SEC_ALG)
		loop = 3;
	else
		loop = 2;

	for (i = 0; i < loop; i++) {

		err = ma35h0_trng_wait_busy_clear(tdev);
		if (err != 0)
			return err;

		for (j = 0; j < 16; j++)
			nu_write_reg(tdev, nonce[j], NPA_DATA(j));

		err = ma35h0_trng_issue_command(tdev, TCMD_GEN_NONCE);
		if (err != 0)
			return err;
	}
	return 0;
}
#else
static int ma35h0_trng_gen_noise(struct ma35h0_trng *tdev)
{
	int	err;

	err = ma35h0_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	err = ma35h0_trng_issue_command(tdev, TCMD_GEN_NOISE);
	if (err != 0)
		return err;

	return 0;
}
#endif

static int ma35h0_trng_create_state(struct ma35h0_trng *tdev)
{
	int	err;

	err = ma35h0_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	err = ma35h0_trng_issue_command(tdev, TCMD_CREATE_STATE);
	if (err != 0)
		return err;

	return 0;
}


static int ma35h0_trng_init(struct hwrng *rng)
{
	struct ma35h0_trng *tdev = (struct ma35h0_trng *)rng->priv;

#ifdef USE_GEN_NONCE
	u32 nonce[16] = {0x11111111, 0x22222222, 0x33333333, 0x44444444,
			 0x55555555, 0x66666666, 0x77777777, 0x88888888,
			 0x99999999, 0xaaaaaaaa, 0xbbbbbbbb, 0xcccccccc,
			 0x12345678, 0x87654321, 0x10293847, 0x65473820 };
#endif
	int	err;

	err = ma35h0_trng_wait_busy_clear(tdev);
	if (err) {
		dev_err(tdev->dev, "TRNG startup busy state timeout!\n");
		return err;
	}

	if (nu_read_reg(tdev, STAT) & (STAT_STARTUP_TEST_STUCK |
	    STAT_STARTUP_TEST_IN_PROG)) {
		dev_err(tdev->dev, "TRNG startup in progress state!\n");
		return -EBUSY;
	}

	/* SELECT_ALG_AES_256 */
	nu_write_reg(tdev, nu_read_reg(tdev, MODE) | MODE_SEC_ALG, MODE);

#ifdef USE_GEN_NONCE
	pr_debug("TRNG GEN_NONCE...\n");
	err = ma35h0_trng_gen_nonce(tdev, nonce);
#else
	pr_debug("TRNG GEN_NOISE...\n");
	err = ma35h0_trng_gen_noise(tdev);
#endif
	if (err != 0)
		return err;

	err = ma35h0_trng_create_state(tdev);
	if (err != 0)
		return err;

	return 0;
}

static int ma35h0_trng_read(struct hwrng *rng, void *buf,
			     size_t max, bool wait)
{
	struct ma35h0_trng *tdev = (struct ma35h0_trng *)rng->priv;
	u32	*data = buf;
	int	i, err, retval;

	retval = 0;

	while (max >= 4) {
		err = ma35h0_trng_wait_busy_clear(tdev);
		if (err != 0)
			return err;

		err = ma35h0_trng_issue_command(tdev, TCMD_GEN_RANDOM);
		if (err != 0)
			return err;

		for (i = 0; i < 4; i++) {
			if (max < 4)
				break;
			*data = nu_read_reg(tdev, RAND(i));
			pr_debug("%08x ", *data);
			data++;
			max -= 4;
			retval += 4;
		}
	}
	return retval;
}

static int ma35h0_trng_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct ma35h0_trng	*tdev;
	struct resource		*res;
	int			err;

	tdev = devm_kzalloc(dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, tdev);

	/*
	 *  Get register base
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tdev->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(tdev->reg_base)) {
		err = PTR_ERR(tdev->reg_base);
		goto res_err;
	}

	tdev->dev = dev;
	tdev->rng.name = pdev->name;
	tdev->rng.init = ma35h0_trng_init;
	tdev->rng.read = ma35h0_trng_read;
	tdev->rng.priv = (unsigned long)tdev;

	err = devm_hwrng_register(dev, &tdev->rng);
	if (err)
		goto res_err;

	platform_set_drvdata(pdev, tdev);

	pr_info("MA35H0 TRNG inited.\n");
	return 0;

res_err:
	devm_kfree(dev, tdev);
	return err;
}

static int ma35h0_trng_remove(struct platform_device *pdev)
{
	devm_kfree(&pdev->dev, platform_get_drvdata(pdev));
	return 0;
}


static const struct of_device_id ma35h0_trng_dt_ids[] = {
	{ .compatible = "nuvoton,ma35h0-trng" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ma35h0_trng_dt_ids);

static struct platform_driver ma35h0_trng_driver = {
	.probe		= ma35h0_trng_probe,
	.remove		= ma35h0_trng_remove,
	.driver		= {
		.name		= "nuvoton-trng",
		.of_match_table = ma35h0_trng_dt_ids,
	},
};

module_platform_driver(ma35h0_trng_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Nuvoton MA35H0 TRNG driver");
