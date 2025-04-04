// SPDX-License-Identifier: GPL-2.0
// IOMapped CAN bus driver for Bosch M_CAN controller
// Copyright (C) 2014 Freescale Semiconductor, Inc.
//	Dong Aisheng <b29396@freescale.com>
//
// Copyright (C) 2018-19 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/platform_device.h>

#include "m_can.h"

#define REG_READ_TIME 20

static u32 iomap_read_reg(struct m_can_classdev *cdev, int reg)
{
	struct m_can_plat_priv *priv = cdev->device_data;

	if(priv->version == 0) {
		u32 u32ReadReg = 0x0;
		u32 u32ReadReg_1 = 0x0;
		u32 u32TimeOutCnt = 0x0;
		unsigned long flags;

		spin_lock_irqsave(&priv->lock, flags);

		u32ReadReg = readl(priv->base + reg);

		do{
			u32ReadReg_1 = readl(priv->base + reg);

			if(u32ReadReg == u32ReadReg_1) {
				u32TimeOutCnt++;
			}
			else {
				u32ReadReg = u32ReadReg_1;
				u32TimeOutCnt = 0;
			}

		}while(u32TimeOutCnt < REG_READ_TIME);

		spin_unlock_irqrestore(&priv->lock, flags);

		return u32ReadReg;
	}
	else
		return readl(priv->base + reg);
}


#if 0
static u32 iomap_read_reg(struct m_can_classdev *cdev, int reg)
{
	struct m_can_plat_priv *priv = cdev->device_data;

	return readl(priv->base + reg);
}
#endif

static u32 iomap_read_fifo(struct m_can_classdev *cdev, int offset)
{
	struct m_can_plat_priv *priv = cdev->device_data;

	return readl(priv->mram_base + offset);
}

static int iomap_write_reg(struct m_can_classdev *cdev, int reg, int val)
{
	struct m_can_plat_priv *priv = cdev->device_data;

	writel(val, priv->base + reg);

	return 0;
}

static int iomap_write_fifo(struct m_can_classdev *cdev, int offset, int val)
{
	struct m_can_plat_priv *priv = cdev->device_data;

	writel(val, priv->mram_base + offset);

	return 0;
}

static struct m_can_ops m_can_plat_ops = {
	.read_reg = iomap_read_reg,
	.write_reg = iomap_write_reg,
	.write_fifo = iomap_write_fifo,
	.read_fifo = iomap_read_fifo,
};

static int m_can_plat_probe(struct platform_device *pdev)
{
	struct m_can_classdev *mcan_class;
	struct m_can_plat_priv *priv;
	struct resource *res;
	void __iomem *addr;
	void __iomem *mram_addr;
	int irq, ret = 0;
	void __iomem *reg;

	mcan_class = m_can_class_allocate_dev(&pdev->dev);
	if (!mcan_class)
		return -ENOMEM;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto probe_fail;
	}

	mcan_class->device_data = priv;

	ret = m_can_class_get_clocks(mcan_class);
	if (ret)
		goto probe_fail;

	spin_lock_init(&priv->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "m_can");
	addr = devm_ioremap_resource(&pdev->dev, res);
	irq = platform_get_irq_byname(pdev, "int0");
	if (IS_ERR(addr) || irq < 0) {
		ret = -EINVAL;
		goto probe_fail;
	}

	/* message ram could be shared */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "message_ram");
	if (!res) {
		ret = -ENODEV;
		goto probe_fail;
	}

	mram_addr = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mram_addr) {
		ret = -ENOMEM;
		goto probe_fail;
	}

	priv->base = addr;
	priv->mram_base = mram_addr;

	mcan_class->net->irq = irq;
	mcan_class->pm_clock_support = 1;
	mcan_class->can.clock.freq = clk_get_rate(mcan_class->cclk);
	mcan_class->dev = &pdev->dev;

	mcan_class->ops = &m_can_plat_ops;

	mcan_class->is_peripheral = false;

	reg = ioremap(0x404601f0, 0x10);
	if (!reg)
		printk("Error: couldn't map product number and revision register !!\n");
	else
		ret = readl(reg) & 0xf000000;

	if(ret == 0)
		priv->version = 0;
	else
		priv->version = 1;

	iounmap(reg);

	platform_set_drvdata(pdev, mcan_class->net);

	m_can_init_ram(mcan_class);

	return m_can_class_register(mcan_class);

probe_fail:
	m_can_class_free_dev(mcan_class->net);
	return ret;
}

static __maybe_unused int m_can_suspend(struct device *dev)
{
	return m_can_class_suspend(dev);
}

static __maybe_unused int m_can_resume(struct device *dev)
{
	return m_can_class_resume(dev);
}

static int m_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct m_can_classdev *mcan_class = netdev_priv(dev);

	m_can_class_unregister(mcan_class);

	m_can_class_free_dev(mcan_class->net);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int __maybe_unused m_can_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *mcan_class = netdev_priv(ndev);

	clk_disable_unprepare(mcan_class->cclk);
	clk_disable_unprepare(mcan_class->hclk);

	return 0;
}

static int __maybe_unused m_can_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *mcan_class = netdev_priv(ndev);
	int err;

	err = clk_prepare_enable(mcan_class->hclk);
	if (err)
		return err;

	err = clk_prepare_enable(mcan_class->cclk);
	if (err)
		clk_disable_unprepare(mcan_class->hclk);

	return err;
}

static const struct dev_pm_ops m_can_pmops = {
	SET_RUNTIME_PM_OPS(m_can_runtime_suspend,
			   m_can_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(m_can_suspend, m_can_resume)
};

static const struct of_device_id m_can_of_table[] = {
	{ .compatible = "bosch,m_can", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, m_can_of_table);

static struct platform_driver m_can_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = m_can_of_table,
		.pm     = &m_can_pmops,
	},
	.probe = m_can_plat_probe,
	.remove = m_can_plat_remove,
};

module_platform_driver(m_can_plat_driver);

MODULE_AUTHOR("Dong Aisheng <b29396@freescale.com>");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("M_CAN driver for IO Mapped Bosch controllers");
