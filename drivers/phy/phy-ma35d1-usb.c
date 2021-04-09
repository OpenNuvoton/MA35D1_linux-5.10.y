// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2020 Nuvoton technology corporation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d1-sys.h>


struct ma35d1_usb_phy {
	struct device *dev;
	struct regmap *sysreg;
	struct clk *clk;
};


static int ma35d1_usb_phy_power_on(struct phy *phy)
{
	struct ma35d1_usb_phy *p_phy = phy_get_drvdata(phy);
	unsigned long timeout;
	int ret;

	ret = clk_prepare_enable(p_phy->clk);
	if (ret < 0) {
		dev_err(p_phy->dev, "Failed to enable PHY clock: %d\n", ret);
		return ret;
	}

	regmap_update_bits(p_phy->sysreg, REG_SYS_USBPMISCR, 0x7, 0x3);

	timeout = jiffies + msecs_to_jiffies(200);
	while (time_before(jiffies, timeout)) {
		unsigned int val;

		regmap_read(p_phy->sysreg, REG_SYS_USBPMISCR, &val);
		if ((val & 0x400) == 0)
			break;
		usleep_range(1000, 1500);
	}

	regmap_update_bits(p_phy->sysreg, REG_SYS_USBPMISCR, 0x7, 0x2);

	timeout = jiffies + msecs_to_jiffies(200);
	while (time_before(jiffies, timeout)) {
		unsigned int val;

		regmap_read(p_phy->sysreg, REG_SYS_USBPMISCR, &val);
		if (val & 0x400)
			return 0;
		usleep_range(1000, 1500);
	}

	dev_err(p_phy->dev, "Timed out waiting for PHY to power on\n");
	ret = -ETIMEDOUT;

	clk_disable_unprepare(p_phy->clk);
	return ret;
}

static int ma35d1_usb_phy_power_off(struct phy *phy)
{
	struct ma35d1_usb_phy *p_phy = phy_get_drvdata(phy);

	clk_disable_unprepare(p_phy->clk);

	return 0;
}

static const struct phy_ops ma35d1_usb_phy_ops = {
	.power_on = ma35d1_usb_phy_power_on,
	.power_off = ma35d1_usb_phy_power_off,
	.owner = THIS_MODULE,
};

static int ma35d1_usb_phy_probe(struct platform_device *pdev)
{
	struct ma35d1_usb_phy *p_phy;
	struct phy_provider *provider;
	struct phy *phy;
	const char *clkgate;

	p_phy = devm_kzalloc(&pdev->dev, sizeof(*p_phy), GFP_KERNEL);
	if (!p_phy)
		return -ENOMEM;
	p_phy->dev = &pdev->dev;
	platform_set_drvdata(pdev, p_phy);

	p_phy->sysreg = syscon_regmap_lookup_by_phandle(p_phy->dev->of_node, "nuvoton,sys");
	if (IS_ERR(p_phy->sysreg)) {
		dev_err(p_phy->dev, "Failed to get SYS registers: %ld\n",
			PTR_ERR(p_phy->sysreg));
		return PTR_ERR(p_phy->sysreg);
	}

	/* enable clock */
	of_property_read_string(p_phy->dev->of_node, "clock-enable", &clkgate);
	p_phy->clk = devm_clk_get(p_phy->dev, clkgate);
	if (IS_ERR(p_phy->clk)) {
		dev_err(p_phy->dev, "Failed to get usb_phy clock\n");
			return PTR_ERR(p_phy->clk);
	}

	phy = devm_phy_create(p_phy->dev, NULL, &ma35d1_usb_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(p_phy->dev, "Failed to create PHY: %ld\n",
			PTR_ERR(phy));
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, p_phy);

	provider = devm_of_phy_provider_register(p_phy->dev, of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(p_phy->dev, "Failed to register PHY provider: %ld\n",
			PTR_ERR(provider));
		return PTR_ERR(provider);
	}

	return 0;
}

static const struct of_device_id ma35d1_usb_phy_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-usb-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, ma35d1_usb_phy_of_match);

static struct platform_driver ma35d1_usb_phy_driver = {
	.probe		= ma35d1_usb_phy_probe,
	.driver		= {
		.name	= "ma35d1-usb-phy",
		.of_match_table = ma35d1_usb_phy_of_match,
	},
};
module_platform_driver(ma35d1_usb_phy_driver);

MODULE_DESCRIPTION("Nuvoton ma35d1 USB2.0 PHY driver");
MODULE_LICENSE("GPL v2");
