// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/err.h>
#include <linux/io.h>

/* Keypad Interface Control Registers */
#define KPI_CONF		0x00
#define KPI_3KCONF		0x04
#define KPI_STATUS		0x08
#define KPI_RSTC		0x0C
#define KPI_KEST		0x10
#define KPI_KPE0		0x18
#define KPI_KPE1		0x1C
#define KPI_KRE0		0x20
#define KPI_KRE1		0x24
#define KPI_PRESCALDIV	0x28

/* KPI_CONF */
#define KROW		(0x70000000) /* Keypad Matrix ROW number */
#define KCOL		(0x07000000) /* Keypad Matrix COL Number */
#define DB_EN		(0x00200000) /* Scan In Signal De-bounce Enable */
#define DB_CLKSEL	(0x000F0000) /* De-bounce sampling cycle selection */
#define PRESCALE	(0x0000FF00) /* Row Scan Cycle Pre-scale Value */
#define INPU		(0x00000040) /* key Scan In Pull-UP Enable Register */
#define WAKEUP		(0x00000020) /* Lower Power Wakeup Enable */
#define ODEN		(0x00000010) /* Open Drain Enable */
#define INTEN		(0x00000008) /* Key Interrupt Enable Control */
#define RKINTEN		(0x00000004) /* Release Key Interrupt Enable */
#define PKINTEN		(0x00000002) /* Press Key Interrupt Enable Control */
#define ENKP		(0x00000001) /* Keypad Scan Enable */

/* KPI_STATUS */
#define RROW7		(0x00800000) /* Release key row coordinate */
#define RROW6		(0x00400000)
#define RROW5		(0x00200000)
#define RROW4		(0x00100000)
#define RROW3		(0x00080000)
#define RROW2		(0x00040000)
#define RROW1		(0x00020000)
#define RROW0		(0x00010000)
#define PROW7		(0x00008000) /* Press key row coordinate */
#define PROW6		(0x00004000)
#define PROW5		(0x00002000)
#define PROW4		(0x00001000)
#define PROW3		(0x00000800)
#define PROW2		(0x00000400)
#define PROW1		(0x00000200)
#define PROW0		(0x00000100)
#define PKEY_INT	(0x00000010) /* Press key interrupt */
#define RKEY_INT	(0x00000008) /* Release key interrupt */
#define KEY_INT		(0x00000004) /* Key Interrupt */
#define RST_3KEY	(0x00000002) /* 3-Keys Reset Flag */
#define PDWAKE		(0x00000001) /* Power Down Wakeup Flag */

#define PROW		(0x00000f00) /* Press Key Row Coordinate */

#define KPI_PRESCALE	(8)
#define DEBOUNCE_BIT	(16)

#define ma35d0_NUM_ROWS		8
#define ma35d0_NUM_COLS		8


struct ma35d0_keypad {
	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;
	int irq;
	unsigned int kpi_row;
	unsigned int kpi_col;
	unsigned int debounce_val;
	unsigned int pre_scale;
	unsigned int pre_scale_divider;
	unsigned int mask_col;
};



void ma35d0_keypad_mfp_set(struct platform_device *pdev)
{

}

static void ma35d0_keypad_scan_matrix(struct ma35d0_keypad *keypad,
										unsigned int status)
{
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int i, j;
	unsigned int row_add = 0;
	unsigned int code;
	unsigned int key;
	unsigned int press_key;
	unsigned long KeyEvent[4];
	unsigned int row_shift = get_count_order(keypad->kpi_col);
	unsigned short *keymap = input_dev->keycode;

	KeyEvent[0] = __raw_readl(keypad->mmio_base + KPI_KPE0);
	KeyEvent[1] = __raw_readl(keypad->mmio_base + KPI_KPE1);
	KeyEvent[2] = __raw_readl(keypad->mmio_base + KPI_KRE0);
	KeyEvent[3] = __raw_readl(keypad->mmio_base + KPI_KRE1);

	__raw_writel(KeyEvent[0], (keypad->mmio_base + KPI_KPE0));
	__raw_writel(KeyEvent[1], (keypad->mmio_base + KPI_KPE1));
	__raw_writel(KeyEvent[2], (keypad->mmio_base + KPI_KRE0));
	__raw_writel(KeyEvent[3], (keypad->mmio_base + KPI_KRE1));

	KeyEvent[0] = KeyEvent[0] & keypad->mask_col;
	KeyEvent[1] = KeyEvent[1] & keypad->mask_col;
	KeyEvent[2] = KeyEvent[2] & keypad->mask_col;
	KeyEvent[3] = KeyEvent[3] & keypad->mask_col;

	for (j = 0; j < 4; j++) {
		if (KeyEvent[j] != 0) {
			if ((j == 1) || (j == 3))
				row_add = 4;
			else
				row_add = 0;

			if (j < 2)
				press_key = 1;
			else
				press_key = 0;

			for (i = 0; i < 32; i++) {
				if (KeyEvent[j] & (1<<i)) {
					code = MATRIX_SCAN_CODE(((i/8) + row_add),
											(i % 8), row_shift);
					key = keymap[code];

					input_event(input_dev, EV_MSC, MSC_SCAN, code);
					input_report_key(input_dev, key, press_key);
				}
			}
		}
	}
}

static irqreturn_t ma35d0_keypad_irq_handler(int irq, void *dev_id)
{
	struct ma35d0_keypad *keypad = dev_id;
	unsigned int  kstatus;

	kstatus = __raw_readl(keypad->mmio_base + KPI_STATUS);

	if (kstatus & (PKEY_INT|RKEY_INT)) {
		ma35d0_keypad_scan_matrix(keypad, kstatus);

		input_sync(keypad->input_dev);
	} else {
		if (kstatus & PDWAKE)
			__raw_writel(PDWAKE, (keypad->mmio_base + KPI_STATUS));
	}

	return IRQ_HANDLED;
}

static int ma35d0_keypad_open(struct input_dev *dev)
{
	struct ma35d0_keypad *keypad = input_get_drvdata(dev);
	unsigned int val, config;

	val = INPU | RKINTEN | PKINTEN | INTEN | ENKP;

	val |= (((keypad->kpi_row) - 1) << 28) |
			(((keypad->kpi_col) - 1) << 24);

	if (keypad->debounce_val > 0)
		config = ((keypad->pre_scale - 1) << KPI_PRESCALE) |
				(keypad->debounce_val << DEBOUNCE_BIT) | DB_EN;
	else
		config = ((keypad->pre_scale - 1) << KPI_PRESCALE);

	val |= config;

	__raw_writel(val, keypad->mmio_base + KPI_CONF);

	__raw_writel((keypad->pre_scale_divider - 1),
				keypad->mmio_base + KPI_PRESCALDIV);

	return 0;
}

static void ma35d0_keypad_close(struct input_dev *dev)
{
	struct ma35d0_keypad *keypad = input_get_drvdata(dev);

	/* Disable clock unit */
	clk_disable(keypad->clk);
}

static int ma35d0_keypad_probe(struct platform_device *pdev)
{
	struct ma35d0_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq;
	int error = 0;
	u32 i;
	struct clk *clk;

	keypad = devm_kzalloc(&pdev->dev, sizeof(*keypad), GFP_KERNEL);

	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		error = -ENOMEM;
		goto failed_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	keypad->mmio_base = ioremap(res->start, resource_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_res;
	}

	keypad->input_dev = input_dev;
	keypad->irq = platform_get_irq(pdev, 0);

	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = ma35d0_keypad_open;
	input_dev->close = ma35d0_keypad_close;
	input_dev->dev.parent = &pdev->dev;

	/* Enable clock */
	clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(clk)) {
		error = PTR_ERR(clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", error);
		return -ENOENT;
	}
	error = clk_prepare_enable(clk);
	if (error)
		return -ENOENT;

	error = matrix_keypad_parse_properties(&pdev->dev,
					&(keypad->kpi_row), &(keypad->kpi_col));
	if (error) {
		dev_err(&pdev->dev, "failed to parse kp params\n");
		return error;
	}

	error = matrix_keypad_build_keymap(NULL, NULL, keypad->kpi_row,
									keypad->kpi_col, NULL, input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to build keymap\n");
		goto failed_put_clk;
	}

	if (keypad->kpi_col == 0x0) keypad->mask_col = 0x0;
	else {
		keypad->mask_col = 1;
		for (i = 0; i < keypad->kpi_col; i++) {
			keypad->mask_col = keypad->mask_col *2;
		}
		keypad->mask_col = keypad->mask_col - 1;
		keypad->mask_col = (keypad->mask_col) | (keypad->mask_col << 8) |
						(keypad->mask_col << 16) | (keypad->mask_col << 24);
	}

	error = request_irq(keypad->irq, ma35d0_keypad_irq_handler,
			    IRQF_NO_SUSPEND, pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_put_clk;
	}

	of_property_read_u32(pdev->dev.of_node, "debounce-period",
									&(keypad->debounce_val));

	of_property_read_u32(pdev->dev.of_node, "pre-scale",
									&(keypad->pre_scale));

	of_property_read_u32(pdev->dev.of_node, "pre-scalediv",
									&(keypad->pre_scale_divider));

	__set_bit(EV_REP, input_dev->evbit);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
	input_set_drvdata(input_dev, keypad);

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	platform_set_drvdata(pdev, keypad);
	return 0;

failed_free_irq:
	free_irq(irq, pdev);
failed_put_clk:
	clk_put(keypad->clk);

	iounmap(keypad->mmio_base);
failed_free_res:
	release_mem_region(res->start, resource_size(res));
failed_free:
	input_free_device(input_dev);
	kfree(keypad);

	dev_err(&pdev->dev, "probe failed\n");

	return error;
}

static int ma35d0_keypad_remove(struct platform_device *pdev)
{
	struct ma35d0_keypad *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	free_irq(keypad->irq, pdev);

	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);

	iounmap(keypad->mmio_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);

	return 0;
}

static int ma35d0_keypad_suspend(struct platform_device *pdev,
									pm_message_t state)
{
	struct ma35d0_keypad *keypad = platform_get_drvdata(pdev);

	__raw_writel(__raw_readl(keypad->mmio_base + KPI_CONF) |
					WAKEUP, keypad->mmio_base + KPI_CONF);
	enable_irq_wake(keypad->irq);

	return 0;
}

static int ma35d0_keypad_resume(struct platform_device *pdev)
{
	struct ma35d0_keypad *keypad = platform_get_drvdata(pdev);

	__raw_writel(__raw_readl(keypad->mmio_base + KPI_CONF) & ~(WAKEUP),
					keypad->mmio_base + KPI_CONF);
	disable_irq_wake(keypad->irq);

	return 0;
}

static const struct of_device_id ma35d0_kpi_of_match[] = {
	{ .compatible = "nuvoton,ma35d0-kpi"},
	{},
};
MODULE_DEVICE_TABLE(of, ma35d0_kpi_of_match);

static struct platform_driver ma35d0_keypad_driver = {
	.probe		= ma35d0_keypad_probe,
	.remove		= ma35d0_keypad_remove,
	.suspend	= ma35d0_keypad_suspend,
	.resume		= ma35d0_keypad_resume,
	.driver		= {
		.name	= "ma35d0-kpi",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d0_kpi_of_match),
	},
};
module_platform_driver(ma35d0_keypad_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("ma35d0 keypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d0-keypad");
