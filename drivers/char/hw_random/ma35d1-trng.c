// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018-2019 Linaro Ltd.
 * Copyright (C) 2020, Nuvoton Technology Corporation
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
#include <linux/tee_drv.h>

//#define USE_GEN_NONCE

/*--------------------------------------------------------------*/
/*  MA35D1 TRNG registers                                      */
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

struct ma35d1_trng {
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

#ifdef CONFIG_OPTEE

#define SHARE_MEM_SIZE		4096

/*-------------------------------------------------------------------*/
/*  OP-TEE client driver for MA35D1 secure TRNG                     */
/*-------------------------------------------------------------------*/
/*
 * PTA_CMD_TRNG_INIT - Initialize TRNG hardware
 *
 * param[0] (out value) - value.a: TRNG STAT register
 *                        value.b: TRNG ISTAT register
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_TRNG_BUSY - TRNG hardware busy
 * TEE_ERROR_TRNG_GEN_NOISE - Failed to generate noise or nounce
 * TEE_ERROR_TRNG_COMMAND - TRNG command failed
 */
#define PTA_CMD_TRNG_INIT		0x1

/*
 * PTA_CMD_TRNG_READ - Get TRNG data
 *
 * param[0] (inout memref) - TRNG data buffer memory reference
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
#define PTA_CMD_TRNG_READ		0x2

#define RND_DATA_SHM_SZ		(4 * 1024)

/**
 * struct optee_rng_private - OP-TEE private data of MA35D1 TRNG
 * @dev:		OP-TEE based RNG device.
 * @ctx:		OP-TEE context handler.
 * @session_id:		RNG TA session identifier.
 * @rdata_shm_pool:	Memory pool shared with OP-TEE MA35D1 TRNG
 * @optee_rng:		Linux hwrng device data
 */
struct optee_rng_private {
	struct device *dev;
	struct tee_context *ctx;
	u32 session_id;
	struct tee_shm *rdata_shm_pool;
	struct hwrng optee_rng;
};

#define to_optee_rng_private(r) \
		container_of(r, struct optee_rng_private, optee_rng)

static size_t get_optee_rng_data(struct optee_rng_private *pvt_data,
				 void *buf, size_t req_size)
{
	int ret = 0;
	u8 *rng_data = NULL;
	size_t rng_size = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_TRNG_READ function of Trusted App */
	inv_arg.func = PTA_CMD_TRNG_READ;
	inv_arg.session = pvt_data->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[0].u.memref.shm = pvt_data->rdata_shm_pool;
	param[0].u.memref.size = req_size;
	param[0].u.memref.shm_offs = 0;

	ret = tee_client_invoke_func(pvt_data->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(pvt_data->dev, "PTA_CMD_TRNG_READ invoke err: %x\n",
			inv_arg.ret);
		return 0;
	}

	rng_data = tee_shm_get_va(pvt_data->rdata_shm_pool, 0);
	if (IS_ERR(rng_data)) {
		dev_err(pvt_data->dev, "tee_shm_get_va failed\n");
		return 0;
	}

	rng_size = param[0].u.memref.size;
	memcpy(buf, rng_data, rng_size);

	return rng_size;
}

static int optee_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct optee_rng_private *pvt_data = to_optee_rng_private(rng);
	size_t read = 0, rng_size = 0;
	u8 *data = buf;

	if (max > RND_DATA_SHM_SZ)
		max = RND_DATA_SHM_SZ;

	if (max % 4)
		return -EINVAL;

	while (read == 0) {
		rng_size = get_optee_rng_data(pvt_data, data, (max - read));

		data += rng_size;
		read += rng_size;
	}
	return read;
}

static int optee_rng_init(struct hwrng *rng)
{
	struct optee_rng_private *pvt_data = to_optee_rng_private(rng);
	struct tee_shm *rdata_shm_pool = NULL;

	rdata_shm_pool = tee_shm_alloc(pvt_data->ctx, RND_DATA_SHM_SZ,
					 TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(rdata_shm_pool)) {
		dev_err(pvt_data->dev, "tee_shm_alloc failed\n");
		return PTR_ERR(rdata_shm_pool);
	}

	pvt_data->rdata_shm_pool = rdata_shm_pool;

	return 0;
}

static void optee_rng_cleanup(struct hwrng *rng)
{
	struct optee_rng_private *pvt_data = to_optee_rng_private(rng);

	tee_shm_free(pvt_data->rdata_shm_pool);
}

static struct optee_rng_private pvt_data = {
	.optee_rng = {
		.name		= "optee-nvt-trng",
		.init		= optee_rng_init,
		.cleanup	= optee_rng_cleanup,
		.read		= optee_rng_read,
	}
};

static int ma35d1_optee_trng_init(struct device *dev)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_TRNG_INIT function of Trusted App */
	inv_arg.func = PTA_CMD_TRNG_INIT;
	inv_arg.session = pvt_data.session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(pvt_data.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(dev, "PTA_CMD_TRNG_INIT invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	pr_info("optee get TRNG STAT=0x%llx, ISTAT=0x%llx\n",
			param[0].u.value.a, param[0].u.value.b);

	return 0;
}

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

static int optee_rng_probe(struct device *dev)
{
	struct tee_client_device *rng_device = to_tee_client_device(dev);
	int ret = 0, err = -ENODEV;
	struct tee_ioctl_open_session_arg sess_arg;

	memset(&sess_arg, 0, sizeof(sess_arg));

	/* Open context with TEE driver */
	pvt_data.ctx = tee_client_open_context(NULL, optee_ctx_match, NULL,
					       NULL);
	if (IS_ERR(pvt_data.ctx))
		return -ENODEV;

	/* Open session with hwrng Trusted App */
	memcpy(sess_arg.uuid, rng_device->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	ret = tee_client_open_session(pvt_data.ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "TRNG tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	pvt_data.session_id = sess_arg.session;

	err = ma35d1_optee_trng_init(dev);
	if (err)
		goto out_sess;

	err = hwrng_register(&pvt_data.optee_rng);
	if (err) {
		dev_err(dev, "optee hwrng registration failed (%d)\n", err);
		goto out_sess;
	}

	pvt_data.dev = dev;

	return 0;

out_sess:
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
out_ctx:
	tee_client_close_context(pvt_data.ctx);

	return err;
}

static int optee_rng_remove(struct device *dev)
{
	hwrng_unregister(&pvt_data.optee_rng);
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
	tee_client_close_context(pvt_data.ctx);

	return 0;
}

static const struct tee_client_device_id optee_rng_id_table[] = {
	{UUID_INIT(0x9f831ffa, 0x1823, 0x4ee9,
		   0x8f, 0xb2, 0x41, 0x57, 0x1f, 0x64, 0x32, 0xe1)},
	{}
};

MODULE_DEVICE_TABLE(tee, optee_rng_id_table);

static struct tee_client_driver optee_rng_driver = {
	.id_table	= optee_rng_id_table,
	.driver		= {
		.name		= "optee-nvt-trng",
		.bus		= &tee_bus_type,
		.probe		= optee_rng_probe,
		.remove		= optee_rng_remove,
	},
};
#endif  /* CONFIG_OPTEE */

/*-------------------------------------------------------------------*/
/*  hwrng driver for MA35D1 TRNG                                    */
/*-------------------------------------------------------------------*/

static inline void nu_write_reg(struct ma35d1_trng *tdev, u32 val, u32 reg)
{
	writel_relaxed(val, tdev->reg_base + reg);
}

static inline u32 nu_read_reg(struct ma35d1_trng *tdev, u32 reg)
{
	return readl_relaxed(tdev->reg_base + reg);
}

static inline int ma35d1_trng_wait_busy_clear(struct ma35d1_trng *tdev)
{
	while (nu_read_reg(tdev, STAT) & STAT_BUSY) {
		if (time_after(jiffies, jiffies +
		    msecs_to_jiffies(TRNG_TIMEOUT)))
			return -EBUSY;
	}
	return 0;
}

static int ma35d1_trng_issue_command(struct ma35d1_trng *tdev, int cmd)
{
	int	err;

	err = ma35d1_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	nu_write_reg(tdev, (nu_read_reg(tdev, CTRL) & ~CTRL_CMD_MASK) |
			(cmd << CTRL_CMD_OFFSET), CTRL);

	while (!(nu_read_reg(tdev, ISTAT) & ISTAT_DONE)) {
		if (time_after(jiffies, jiffies +
		    msecs_to_jiffies(TRNG_TIMEOUT))) {
			pr_debug("TRNG command %d timeout! ISTAT = 0x%x, SMODE = 0x%x.\n",
				 cmd, nu_read_reg(tdev, ISTAT),
				 nu_read_reg(tdev, SMODE));
			return -EBUSY;
		}
	}
	return 0;
}

#ifdef USE_GEN_NONCE
static int ma35d1_trng_gen_nonce(struct ma35d1_trng *tdev, uint32_t *nonce)
{
	int   i, j, loop, err;

	nu_write_reg(tdev, nu_read_reg(tdev, SMODE) | SMODE_NONCE, SMODE);

	if (nu_read_reg(tdev, MODE) & MODE_SEC_ALG)
		loop = 3;
	else
		loop = 2;

	for (i = 0; i < loop; i++) {

		err = ma35d1_trng_wait_busy_clear(tdev);
		if (err != 0)
			return err;

		for (j = 0; j < 16; j++)
			nu_write_reg(tdev, nonce[j], NPA_DATA(j));

		err = ma35d1_trng_issue_command(tdev, TCMD_GEN_NONCE);
		if (err != 0)
			return err;
	}
	return 0;
}
#else
static int ma35d1_trng_gen_noise(struct ma35d1_trng *tdev)
{
	int	err;

	err = ma35d1_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	err = ma35d1_trng_issue_command(tdev, TCMD_GEN_NOISE);
	if (err != 0)
		return err;

	return 0;
}
#endif


static int ma35d1_trng_create_state(struct ma35d1_trng *tdev)
{
	int	err;

	err = ma35d1_trng_wait_busy_clear(tdev);
	if (err != 0)
		return err;

	err = ma35d1_trng_issue_command(tdev, TCMD_CREATE_STATE);
	if (err != 0)
		return err;

	return 0;
}


static int ma35d1_trng_init(struct hwrng *rng)
{
	struct ma35d1_trng *tdev = (struct ma35d1_trng *)rng->priv;

#ifdef USE_GEN_NONCE
	u32 nonce[16] = {0x11111111, 0x22222222, 0x33333333, 0x44444444,
			 0x55555555, 0x66666666, 0x77777777, 0x88888888,
			 0x99999999, 0xaaaaaaaa, 0xbbbbbbbb, 0xcccccccc,
			 0x12345678, 0x87654321, 0x10293847, 0x65473820 };
#endif
	int	err;

	err = ma35d1_trng_wait_busy_clear(tdev);
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
	err = ma35d1_trng_gen_nonce(tdev, nonce);
#else
	pr_debug("TRNG GEN_NOISE...\n");
	err = ma35d1_trng_gen_noise(tdev);
#endif
	if (err != 0)
		return err;

	err = ma35d1_trng_create_state(tdev);
	if (err != 0)
		return err;

	return 0;
}

static int ma35d1_trng_read(struct hwrng *rng, void *buf,
			     size_t max, bool wait)
{
	struct ma35d1_trng *tdev = (struct ma35d1_trng *)rng->priv;
	u32	*data = buf;
	int	i, err, retval;

	retval = 0;

	while (max >= 4) {
		err = ma35d1_trng_wait_busy_clear(tdev);
		if (err != 0)
			return err;

		err = ma35d1_trng_issue_command(tdev, TCMD_GEN_RANDOM);
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

static int ma35d1_trng_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct ma35d1_trng	*tdev;
	struct resource		*res;
	int			err;

#ifdef CONFIG_OPTEE
	const char  *optee_sel;
	bool  optee_nuvoton = false;

	if (!of_property_read_string(dev->of_node, "optee_nuvoton",
	    &optee_sel)) {
		if (!strcmp("yes", optee_sel))
			optee_nuvoton = true;
	}
	if (optee_nuvoton) {
		pr_info("Register MA35D1 TRNG optee client driver.\n");
		return driver_register(&optee_rng_driver.driver);
	}
#endif
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
	tdev->rng.init = ma35d1_trng_init;
	tdev->rng.read = ma35d1_trng_read;
	tdev->rng.priv = (unsigned long)tdev;

	err = devm_hwrng_register(dev, &tdev->rng);
	if (err)
		goto res_err;

	platform_set_drvdata(pdev, tdev);

	pr_info("MA35D1 TRNG inited.\n");
	return 0;

res_err:
	devm_kfree(dev, tdev);
	return err;
}

static int ma35d1_trng_remove(struct platform_device *pdev)
{
	devm_kfree(&pdev->dev, platform_get_drvdata(pdev));
	return 0;
}


static const struct of_device_id ma35d1_trng_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-trng" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ma35d1_trng_dt_ids);

static struct platform_driver ma35d1_trng_driver = {
	.probe		= ma35d1_trng_probe,
	.remove		= ma35d1_trng_remove,
	.driver		= {
		.name		= "nuvoton-trng",
		.of_match_table = ma35d1_trng_dt_ids,
	},
};

module_platform_driver(ma35d1_trng_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Nuvoton MA35D1 TRNG driver");
