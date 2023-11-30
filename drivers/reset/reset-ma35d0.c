// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/mfd/ma35d0-sys.h>
#include <dt-bindings/reset/nuvoton,ma35d0-reset.h>
#include <linux/reboot.h>

#define RST_PRE_REG	32

struct ma35d0_reset_data {
	struct reset_controller_dev	rcdev;
	struct regmap			*regmap;
};

struct ma35d0_reboot_data {
	struct notifier_block		restart_handler;
	struct regmap			*regmap;
};

static int ma35d0_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	struct ma35d0_reboot_data *data =
			container_of(this, struct ma35d0_reboot_data,
					restart_handler);
	ma35d0_reg_unlock();
	regmap_write(data->regmap, REG_SYS_IPRST0, 1 << MA35D0_RESET_CHIP);
	ma35d0_reg_lock();

	while (1)
		cpu_do_idle();

	return NOTIFY_DONE;
}


static int ma35d0_reset_update(struct reset_controller_dev *rcdev,
			      unsigned long id, bool assert)
{
	struct ma35d0_reset_data *data = container_of(rcdev, struct ma35d0_reset_data, rcdev);
	int reg;
	int offset = (id / RST_PRE_REG) * 4;

	ma35d0_reg_unlock();
	regmap_read(data->regmap, REG_SYS_IPRST0 + offset, &reg);
	if (assert)
		reg |= 1 << (id % RST_PRE_REG);
	else
		reg &= ~(1 << (id % RST_PRE_REG));

	regmap_write(data->regmap, REG_SYS_IPRST0 + offset, reg);

	ma35d0_reg_lock();
	return 0;
}

static int ma35d0_reset_assert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	return ma35d0_reset_update(rcdev, id, true);
}

static int ma35d0_reset_deassert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return ma35d0_reset_update(rcdev, id, false);
}

static int ma35d0_reset_status(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct ma35d0_reset_data *data = container_of(rcdev, struct ma35d0_reset_data, rcdev);
	int reg;
	int offset = id / RST_PRE_REG;

	regmap_read(data->regmap, REG_SYS_IPRST0 + offset, &reg);

	return !!(reg & BIT(id % RST_PRE_REG));

}

static const struct reset_control_ops ma35d0_reset_ops = {
	.assert		= ma35d0_reset_assert,
	.deassert	= ma35d0_reset_deassert,
	.status		= ma35d0_reset_status,
};

static const struct of_device_id ma35d0_reset_dt_ids[] = {
	{ .compatible = "nuvoton,ma35d0-reset"},
	{},
};

static int ma35d0_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ma35d0_reset_data *reset_data;
	struct ma35d0_reboot_data *reboot_data;
	int err;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Device tree node not found\n");
		return -EINVAL;
	}

	reset_data = devm_kzalloc(dev, sizeof(*reset_data), GFP_KERNEL);
	if (!reset_data)
		return -ENOMEM;

	reboot_data = devm_kzalloc(dev, sizeof(*reboot_data), GFP_KERNEL);
	if (!reboot_data)
		return -ENOMEM;

	reset_data->regmap  = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
								"nuvoton,ma35d0-sys");
	if (IS_ERR(reset_data->regmap)) {
		dev_err(&pdev->dev, "Failed to get SYS register base\n");
		return -ENODEV;
	}

	reboot_data->regmap  = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
								"nuvoton,ma35d0-sys");
	if (IS_ERR(reboot_data->regmap)) {
		dev_err(&pdev->dev, "Failed to get SYS register base\n");
		return -ENODEV;
	}

	reset_data->rcdev.owner = THIS_MODULE;
	reset_data->rcdev.nr_resets = MA35D0_RESET_COUNT;
	reset_data->rcdev.ops = &ma35d0_reset_ops;
	reset_data->rcdev.of_node = dev->of_node;

	reboot_data->restart_handler.notifier_call = ma35d0_restart_handler;
	reboot_data->restart_handler.priority = 192;

	err = register_restart_handler(&reboot_data->restart_handler);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n", err);
		return -ENODEV;
	}

	return devm_reset_controller_register(dev, &reset_data->rcdev);
}

static struct platform_driver ma35d0_reset_driver = {
	.probe	= ma35d0_reset_probe,
	.driver = {
		.name		= "ma35d0-reset",
		.of_match_table	= ma35d0_reset_dt_ids,
	},
};

builtin_platform_driver(ma35d0_reset_driver);
