// SPDX-License-Identifier: GPL-2.0-only
/*
 * MA35D1 system control driver
 *
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/ma35d1-sys.h>

struct reg_protect {
	int rec;
	struct regmap *regmap;
	spinlock_t lock;
} rp;


void ma35d1_reg_unlock(void)
{
	unsigned long flags;
	unsigned int reg;

	spin_lock_irqsave(&rp.lock, flags);

	rp.rec++;
	while (1) {
		regmap_read(rp.regmap, REG_SYS_RLKTZNS, &reg);
		if (reg == 1)
			break;

		regmap_write(rp.regmap, REG_SYS_RLKTZNS, 0x59);
		regmap_write(rp.regmap, REG_SYS_RLKTZNS, 0x16);
		regmap_write(rp.regmap, REG_SYS_RLKTZNS, 0x88);
	}

	spin_unlock_irqrestore(&rp.lock, flags);

}
EXPORT_SYMBOL(ma35d1_reg_unlock);

void ma35d1_reg_lock(void)
{
	unsigned long flags;

	spin_lock_irqsave(&rp.lock, flags);
	if (--rp.rec == 0)
		regmap_write(rp.regmap, REG_SYS_RLKTZNS, 1);

	spin_unlock_irqrestore(&rp.lock, flags);

}
EXPORT_SYMBOL(ma35d1_reg_lock);

static int ma35d1_sysctrl_probe(struct platform_device *pdev)
{
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "Device tree node not found\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, &rp);

	rp.regmap  = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "nuvoton,ma35d1-sys");
	if (IS_ERR(rp.regmap)) {
		dev_err(&pdev->dev, "Failed to get SYS register base\n");
		return -ENODEV;
	}
	spin_lock_init(&rp.lock);
	rp.rec = 0;

	return 0;
}

static int ma35d1_sysctrl_remove(struct platform_device *pdev)
{


	return 0;
}

static const struct of_device_id ma35d1_sysctrl_match[] = {
	{ .compatible = "nuvoton,ma35d1-sysctrl", },
	{}
};

static struct platform_driver ma35d1_sysctrl_driver = {
	.driver = {
		.name = "ma35d1-sysctrl",
		.of_match_table = ma35d1_sysctrl_match,
	},
	.probe = ma35d1_sysctrl_probe,
	.remove = ma35d1_sysctrl_remove,
};

builtin_platform_driver(ma35d1_sysctrl_driver);
