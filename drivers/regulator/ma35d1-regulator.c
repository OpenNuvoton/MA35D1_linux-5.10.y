// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Nuvoton MA35d1 regulator
 *
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 */


#define DRVNAME "ma35d1-regulator"
#define pr_fmt(fmt) DRVNAME ": " fmt


#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <soc/nuvoton/ma35d1_sip.h>
#include <linux/slab.h>

struct ma35d1_info {
	u8	id;	/*  PMIC resource ID */
};

static const unsigned int ma35d1_volt_table[] = {
	1800000,
	3300000,
};

static const unsigned int ma35d1_pmic_table[] = {
	VOL_1_80,
	VOL_3_30,
};

static int ma35d1_get_voltage_sel(struct regulator_dev *regdev)
{
	struct arm_smccc_res res;
	struct ma35d1_info *info = rdev_get_drvdata(regdev);
	int idx;

	arm_smccc_smc(MA35D1_SIP_PMIC,
			info->id,
			0, 0, 0, 0, 0, 0, &res);

	for (idx = 0; idx < ARRAY_SIZE(ma35d1_pmic_table); idx++)
		if (res.a0 == ma35d1_pmic_table[idx])
			break;

	return idx;
}

static int ma35d1_set_voltage_sel(struct regulator_dev *regdev,
					unsigned int sel)
{
	struct arm_smccc_res res;
	struct ma35d1_info *info = rdev_get_drvdata(regdev);

	arm_smccc_smc(MA35D1_SIP_PMIC,
		info->id,
		ma35d1_pmic_table[sel],
		0, 0, 0, 0, 0, &res);

	return 0;
}

static const struct regulator_ops ma35d1_regulator_ops = {
	.get_voltage_sel = ma35d1_get_voltage_sel,
	.set_voltage_sel = ma35d1_set_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
};

static int ma35d1_regulator_probe(struct platform_device *pdev)
{
	struct ma35d1_info *info;
	struct regulator_desc *desc;
	struct regulator_init_data *init_data;
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Device tree node not found\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	info->id = MA35D1_SIP_PMIC_SD;
	desc->n_voltages = ARRAY_SIZE(ma35d1_volt_table);
	desc->volt_table = ma35d1_volt_table;
	desc->name = dev_name(&pdev->dev);
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;

	init_data = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node,
					       desc);
	if (!init_data)
		return -EINVAL;

	desc->ops = &ma35d1_regulator_ops;
	config.dev = &pdev->dev;
	config.init_data = init_data;
	config.of_node = pdev->dev.of_node;
	config.driver_data = info;

	rdev = devm_regulator_register(&pdev->dev, desc, &config);

	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	platform_set_drvdata(pdev, rdev);

	return 0;
}

static const struct of_device_id ma35d1_regulator_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-volt", },
	{ }
};
MODULE_DEVICE_TABLE(of, ma35d1_regulator_of_match);

static struct platform_driver ma35d1_regulator_driver = {
	.probe = ma35d1_regulator_probe,
	.driver	= {
		.name = DRVNAME,
		.of_match_table = ma35d1_regulator_of_match,
	},
};

module_platform_driver(ma35d1_regulator_driver);

MODULE_DESCRIPTION("Nuvootn ma35d1 regulator");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ma35d1-regulator");

