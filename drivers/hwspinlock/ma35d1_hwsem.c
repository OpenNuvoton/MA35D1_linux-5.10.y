// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hwspinlock.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "hwspinlock_internal.h"

/*
 * Since the MUTEX_KEY is fixed in this driver, hswpinlocks should only be
 * used to synchronise operations between the CA35 and CM4.
 */


#define NVT_MUTEX_KEY		0x5500
#define NVT_MUTEX_FREE		0
#define NVT_MUTEX_COREID	0x1 // 0x1: CA35, 0x2: CM4
#define NVT_MUTEX_CORE_MASK	0xF
#define NVT_MUTEX_NUM_LOCKS	8
#define NVT_MUTEX_REG_OFFSET	8

struct nvt_hwsem {
	struct clk *clk;
	struct hwspinlock_device bank;
};

static int nvt_hwsem_trylock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;
	u32 status;

	writel(NVT_MUTEX_KEY, lock_addr);
	status = readl(lock_addr);

	return status == (NVT_MUTEX_KEY | NVT_MUTEX_COREID);
}

static void nvt_hwsem_unlock(struct hwspinlock *lock)
{
	void __iomem *lock_addr = lock->priv;

	writel(NVT_MUTEX_KEY, lock_addr);
}

static void nvt_hwsem_relax(struct hwspinlock *lock)
{
	ndelay(50);
}

static const struct hwspinlock_ops nvt_hwsem_ops = {
	.trylock	= nvt_hwsem_trylock,
	.unlock		= nvt_hwsem_unlock,
	.relax		= nvt_hwsem_relax,
};

static int nvt_hwsem_probe(struct platform_device *pdev)
{
	struct nvt_hwsem *hw;
	void __iomem *io_base;
	size_t array_size;
	int i, ret;

	io_base = devm_platform_ioremap_resource(pdev, 0);
	if (!io_base)
		return -ENOMEM;

	array_size = NVT_MUTEX_NUM_LOCKS * sizeof(struct hwspinlock);
	hw = devm_kzalloc(&pdev->dev, sizeof(*hw) + array_size, GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	hw->clk = devm_clk_get(&pdev->dev, "hws_gate");
	if (IS_ERR(hw->clk))
		return PTR_ERR(hw->clk);

	ret = clk_prepare_enable(hw->clk);
	if (ret)
		return ret;

	// SEMx begins at offset 0x20, so add 8 words here.
	for (i = 0; i < NVT_MUTEX_NUM_LOCKS; i++)
		hw->bank.lock[i].priv = io_base + (i + NVT_MUTEX_REG_OFFSET) * sizeof(u32);

	platform_set_drvdata(pdev, hw);
	pm_runtime_enable(&pdev->dev);

	ret = hwspin_lock_register(&hw->bank, &pdev->dev, &nvt_hwsem_ops,
				   0, NVT_MUTEX_NUM_LOCKS);

	if (ret)
		pm_runtime_disable(&pdev->dev);

	return ret;
}

static int nvt_hwsem_remove(struct platform_device *pdev)
{
	struct nvt_hwsem *hw = platform_get_drvdata(pdev);
	int ret;

	ret = hwspin_lock_unregister(&hw->bank);
	if (ret)
		dev_err(&pdev->dev, "%s failed: %d\n", __func__, ret);

	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(hw->clk);

	return 0;
}

static int __maybe_unused nvt_hwsem_runtime_suspend(struct device *dev)
{
	struct nvt_hwsem *hw = dev_get_drvdata(dev);

	clk_disable_unprepare(hw->clk);
	return 0;
}

static int __maybe_unused nvt_hwsem_runtime_resume(struct device *dev)
{
	struct nvt_hwsem *hw = dev_get_drvdata(dev);

	clk_prepare_enable(hw->clk);
	return 0;
}

static const struct dev_pm_ops nvt_hwsem_pm_ops = {
	SET_RUNTIME_PM_OPS(nvt_hwsem_runtime_suspend,
			   nvt_hwsem_runtime_resume,
			   NULL)
};

static const struct of_device_id nvt_hwsem_ids[] = {
	{ .compatible = "nuvoton,ma35d1-hwsem", },
	{},
};
MODULE_DEVICE_TABLE(of, nvt_hwsem_ids);

static struct platform_driver nvt_hwsem_driver = {
	.probe		= nvt_hwsem_probe,
	.remove		= nvt_hwsem_remove,
	.driver		= {
		.name	= "ma35d1-hwsem",
		.of_match_table = nvt_hwsem_ids,
		.pm	= &nvt_hwsem_pm_ops,
	},
};

static int __init nvt_hwsem_init(void)
{
	return platform_driver_register(&nvt_hwsem_driver);
}
postcore_initcall(nvt_hwsem_init);

static void __exit nvt_hwsem_exit(void)
{
	platform_driver_unregister(&nvt_hwsem_driver);
}
module_exit(nvt_hwsem_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hardware semaphore driver for MA35D1 MPU");

