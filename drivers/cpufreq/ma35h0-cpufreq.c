// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <soc/nuvoton/ma35h0_sip.h>

#define TRANSITION_LATENCY	(10 * 1000)	/* 10 us */

static struct cpufreq_frequency_table ma35h0_freq_table[] = {
	{0, 0x000006A2, 650000},
	{0, 0x00001396, 600000},
	{0, 0x0000137D, 500000},
	{0, 0x0000237D, 250000},
	{0, 0x0000337D, 125000},
	{0, 0, CPUFREQ_TABLE_END},
};

static struct regmap *clk_regmap;
static struct platform_device *cpufreq_pdev;

static int ma35h0_cpufreq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	unsigned int freq;
	struct arm_smccc_res res;

	freq = ma35h0_freq_table[index].frequency / 1000;
	arm_smccc_smc(MA35H0_SIP_CPU_CLK, freq, 0, 0, 0, 0, 0, 0, &res);
	// printk("%s - index=%d, set CPU to %d MHz, CAP-PLL should be 0x%x\n",
	//	  __func__, index, freq, ma35h0_freq_table[index].driver_data);
	return 0;
}

static unsigned int ma35h0_cpufreq_get(unsigned int cpu)
{
	u32 capll, idx, freq = 650000;

	regmap_read(clk_regmap, 0x60, &capll);
	// printk("%s - capll is 0x%x\n", __func__, capll);

	for (idx = 0; idx < ARRAY_SIZE(ma35h0_freq_table); idx++) {
		if (ma35h0_freq_table[idx].frequency == CPUFREQ_TABLE_END)
			break;
		if (capll == ma35h0_freq_table[idx].driver_data) {
			freq = ma35h0_freq_table[idx].frequency;
			break;
		}
	}
	return freq;
}

static int ma35h0_cpufreq_init(struct cpufreq_policy *policy)
{
	cpufreq_generic_init(policy, ma35h0_freq_table, TRANSITION_LATENCY);
	policy->max = 650000;
	policy->min = 125000;
	return 0;
}

static int ma35h0_cpufreq_exit(struct cpufreq_policy *policy)
{
	return 0;
}

static struct cpufreq_driver ma35h0_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_NO_AUTO_DYNAMIC_SWITCHING,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = ma35h0_cpufreq_set_target,
	.get = ma35h0_cpufreq_get,
	.init = ma35h0_cpufreq_init,
	.exit = ma35h0_cpufreq_exit,
	.name = "ma35h0-cpufreq",
	.attr = cpufreq_generic_attr,
};

static int ma35h0_cpufreq_probe(struct platform_device *pdev)
{
	int ret;

	ret = cpufreq_register_driver(&ma35h0_cpufreq_driver);
	if (ret)
		dev_err(&pdev->dev, "failed to register ma35h0 cpufreq driver\n");

	return ret;
}

static struct platform_driver ma35h0_cpufreq_platdrv = {
	.driver = {
		.name	= "ma35h0-cpufreq",
	},
	.probe		= ma35h0_cpufreq_probe,
};

/* List of machines supported by this driver */
static const struct of_device_id ma35h0_cpufreq_machines[] __initconst = {
	{ .compatible = "nuvoton,ma35h0", },
	{ }
};
MODULE_DEVICE_TABLE(of, ma35h0_cpufreq_machines);

static int __init ma35h0_cpufreq_driver_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;
	int err;

	np = of_find_node_by_path("/");
	if (!np)
		return -ENODEV;

	match = of_match_node(ma35h0_cpufreq_machines, np);
	of_node_put(np);
	if (!match) {
		pr_debug("Machine is not compatible with ma35h0-cpufreq\n");
		return -ENODEV;
	}

	clk_regmap = syscon_regmap_lookup_by_compatible("nuvoton,ma35h0-clk");
	if (!clk_regmap)
		pr_debug("Failed to get nuvoton,ma35h0-clk regmap!\n");

	err = platform_driver_register(&ma35h0_cpufreq_platdrv);
	if (err)
		return err;

	cpufreq_pdev = platform_device_register_simple("ma35h0-cpufreq", -1, NULL, 0);
	if (IS_ERR(cpufreq_pdev)) {
		pr_err("failed to register ma35h0-cpufreq platform device\n");
		platform_driver_unregister(&ma35h0_cpufreq_platdrv);
		return PTR_ERR(cpufreq_pdev);
	}
	return 0;
}
module_init(ma35h0_cpufreq_driver_init)

static void __exit ma35h0_cpufreq_driver_exit(void)
{
	platform_device_unregister(cpufreq_pdev);
	platform_driver_unregister(&ma35h0_cpufreq_platdrv);
}
module_exit(ma35h0_cpufreq_driver_exit)

MODULE_DESCRIPTION("Nuvoton MA35H0 CPUFreq driver");
MODULE_LICENSE("GPL v2");
