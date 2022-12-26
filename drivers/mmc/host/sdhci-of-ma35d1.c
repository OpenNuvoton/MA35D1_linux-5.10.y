// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Nuvoton MA35d1 SDHCI
 *
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/mmc/mmc.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/ma35d1-sys.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include "sdhci-pltfm.h"

#define MSHC_CTRL 0x508
#define  CMD_CONFLICT_CHECK 0x1
#define MBIU_CTRL 0x510
#define  BURST_INCR_MASK 0xF
#define  BURST_INCR16_EN 0x8
#define  BURST_INCR8_EN 0x4
#define  BURST_INCR4_EN 0x2
#define  UNDEFL_INCR_EN 0x1
#define SDH1VSTB (1<<17)

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

struct ma35d1_priv {
	struct clk	*bus_clk;
	struct regmap	*regmap;
	struct pinctrl	*pinctrl;
	struct pinctrl_state	*pinctrl_state_3v3;
	struct pinctrl_state	*pinctrl_state_1v8;
	struct reset_control    *rst;
};

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void ma35d1_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static int ma35d1_sdhci_init_pinctrl(struct device *dev,
					 struct ma35d1_priv *ma35d1_priv)
{
	ma35d1_priv->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ma35d1_priv->pinctrl)) {
		dev_dbg(dev, "No pinctrl info, err: %ld\n",
			PTR_ERR(ma35d1_priv->pinctrl));
		return -1;
	}

	ma35d1_priv->pinctrl_state_3v3 =
		pinctrl_lookup_state(ma35d1_priv->pinctrl, "sdhci_3V3");
	if (IS_ERR(ma35d1_priv->pinctrl_state_3v3)) {
		dev_warn(dev, "Missing 3.3V pad state\n");
		return -1;
	}

	ma35d1_priv->pinctrl_state_1v8 =
		pinctrl_lookup_state(ma35d1_priv->pinctrl, "sdhci_1V8");
	if (IS_ERR(ma35d1_priv->pinctrl_state_1v8)) {
		dev_warn(dev, "Missing 1.8V pad state, err: %ld\n",
			 PTR_ERR(ma35d1_priv->pinctrl_state_1v8));
		return -1;
	}
	return 0;
}

static int ma35d1_sdhci_set_padctrl(struct sdhci_host *host, int voltage)
{
	int ret;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct ma35d1_priv *ma35d1_priv = sdhci_pltfm_priv(pltfm_host);

	/* Only signal voltage PADS */
	if (host->quirks2 & SDHCI_QUIRK2_NO_1_8_V)
		return 0;

	if (voltage == MMC_SIGNAL_VOLTAGE_180) {
		ret = pinctrl_select_state(ma35d1_priv->pinctrl,
					ma35d1_priv->pinctrl_state_1v8);
		if (ret < 0)
			dev_err(mmc_dev(host->mmc),
				"setting 1.8V failed, ret: %d\n", ret);
	} else {
		ret = pinctrl_select_state(ma35d1_priv->pinctrl,
					ma35d1_priv->pinctrl_state_3v3);
		if (ret < 0)
			dev_err(mmc_dev(host->mmc),
				"setting 3.3V failed, ret: %d\n", ret);
	}

	return 0;
}

static void ma35d1_voltage_switch(struct sdhci_host *host)
{
       /*
	* According to Section 3.6.1 signal voltage switch procedure in
	* SD Host Controller Simplified Spec. 4.20, steps 6~8 are as
	* follows:
	* (6) Set 1.8V Signal Enable in the Host Control 2 register.
	* (7) Wait 5ms. 1.8V voltage regulator shall be stable within this
	*     period.
	* (8) If 1.8V Signal Enable is cleared by Host Controller, go to
	*     step (12).
	*
	* Wait 50ms after set 1.8V signal enable in Host Control 2 register.
	*/
	usleep_range(50000, 55000);
}

static int ma35d1_start_signal_voltage_switch(struct mmc_host *mmc,
							struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int ret = 0;

	ret = ma35d1_sdhci_set_padctrl(host, ios->signal_voltage);
	if (ret < 0)
		return ret;

	ret = sdhci_start_signal_voltage_switch(mmc, ios);
	return ret;
}

void ma35d1_set_clock(struct sdhci_host *host, unsigned int clock)
{
	/* If clock > 100MHz, Need to set CMD_CONFIG_CHECK = 0 */
	if (clock > 100000000)
		sdhci_writew(host, sdhci_readw(host, MSHC_CTRL)&~CMD_CONFLICT_CHECK, MSHC_CTRL);
	else
		sdhci_writew(host, sdhci_readw(host, MSHC_CTRL)|CMD_CONFLICT_CHECK, MSHC_CTRL);
	sdhci_set_clock(host, clock);
}

#define REG_RESTORE_NUM 7
int ma35d1_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct ma35d1_priv *priv = sdhci_pltfm_priv(pltfm_host);

	if (!IS_ERR(priv->rst)) {
		int idx;
		u32 value[REG_RESTORE_NUM];
		int reg[REG_RESTORE_NUM] = {
			SDHCI_CLOCK_CONTROL,
			SDHCI_BLOCK_SIZE,
			SDHCI_INT_ENABLE,
			SDHCI_SIGNAL_ENABLE,
			SDHCI_AUTO_CMD_STATUS,
			SDHCI_HOST_CONTROL,
			SDHCI_TIMEOUT_CONTROL,
		};

		for (idx = 0; idx < (REG_RESTORE_NUM-1); idx++)
			value[idx] = sdhci_readl(host, reg[idx]);
		value[(REG_RESTORE_NUM-1)] = sdhci_readb(host, reg[(REG_RESTORE_NUM-1)]);

		reset_control_assert(priv->rst);
		reset_control_deassert(priv->rst);

		for (idx = 0; idx < (REG_RESTORE_NUM-1); idx++)
			sdhci_writel(host, value[idx], reg[idx]);
		sdhci_writeb(host, value[(REG_RESTORE_NUM-1)], reg[(REG_RESTORE_NUM-1)]);

		sdhci_writew(host, sdhci_readw(host, MSHC_CTRL)&~CMD_CONFLICT_CHECK, MSHC_CTRL);
		sdhci_writew(host, (sdhci_readw(host, MBIU_CTRL)&~BURST_INCR_MASK)|
				(BURST_INCR8_EN|BURST_INCR16_EN), MBIU_CTRL);
	}

	return sdhci_execute_tuning(mmc, opcode);
}
static const struct sdhci_ops sdhci_ma35d1_ops = {
	.set_clock		= ma35d1_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= sdhci_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.reset			= sdhci_reset,
	.adma_write_desc	= ma35d1_adma_write_desc,
	.voltage_switch		= ma35d1_voltage_switch,
};

static const struct sdhci_pltfm_data sdhci_ma35d1_pdata = {
	.ops = &sdhci_ma35d1_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static void ma35d1_shutdown(struct platform_device *pdev)
{
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get_optional(&pdev->dev, "power", GPIOD_OUT_LOW);
	ma35d1_reg_unlock();
	if (gpio) {
		gpiod_set_value_cansleep(gpio, 1);
		udelay(1);
	}
	ma35d1_reg_lock();
}


static int ma35d1_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_host *host;
	struct ma35d1_priv *priv;
	int err;
	u32 extra;

	host = sdhci_pltfm_init(pdev, &sdhci_ma35d1_pdata,
				sizeof(struct ma35d1_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
		clk_prepare_enable(priv->bus_clk);

	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	priv->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->rst))
		dev_err(&pdev->dev, "Error: Missing sdhci controller reset\n");
	else {
		reset_control_assert(priv->rst);
		reset_control_deassert(priv->rst);
	}


	sdhci_get_of_property(pdev);

	if (!(host->quirks2 & SDHCI_QUIRK2_NO_1_8_V)) {
		int reg;

		priv->regmap = syscon_regmap_lookup_by_phandle(
				pdev->dev.of_node, "nuvoton,ma35d1-sys");

		/* Set SDH1 voltage stable for 1.8V  */
		regmap_read(priv->regmap, REG_SYS_MISCFCR0, &reg);
		reg |= SDH1VSTB;
		regmap_write(priv->regmap, REG_SYS_MISCFCR0, reg);

		err = ma35d1_sdhci_init_pinctrl(&pdev->dev, priv);
		if (err == 0) {
			host->mmc_host_ops.start_signal_voltage_switch =
				ma35d1_start_signal_voltage_switch;
			ma35d1_sdhci_set_padctrl(host, MMC_SIGNAL_VOLTAGE_330);
		}
	}

	host->mmc_host_ops.execute_tuning = ma35d1_execute_tuning;
	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	/* Enable INCR16 and INCR8 */
	sdhci_writew(host, (sdhci_readw(host, MBIU_CTRL)&~BURST_INCR_MASK)|(BURST_INCR8_EN|BURST_INCR16_EN), MBIU_CTRL);

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int ma35d1_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct ma35d1_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

static const struct of_device_id sdhci_ma35d1_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d1-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_ma35d1_dt_ids);

static struct platform_driver sdhci_ma35d1_driver = {
	.driver	= {
		.name	= "sdhci-ma35d1",
		.of_match_table = sdhci_ma35d1_dt_ids,
	},
	.probe	= ma35d1_probe,
	.remove = ma35d1_remove,
	.shutdown = ma35d1_shutdown,
};
module_platform_driver(sdhci_ma35d1_driver);

MODULE_DESCRIPTION("SDHCI platform driver for Nuvoton ma35d1");
MODULE_LICENSE("GPL v2");
