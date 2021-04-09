// SPDX-License-Identifier: GPL-2.0
/*
 * MA35D1 EADC driver
 *
 * Copyright (c) 2020 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/of.h>

#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/clk.h>

/* ma35d1 eadc registers offset */
#define DAT0	0x00
#define DAT1	0x04
#define DAT2	0x08
#define DAT3	0x0C
#define DAT4	0x10
#define DAT5	0x14
#define DAT6	0x18
#define DAT7	0x1C
#define DAT8	0x20
#define CTL	0x50
#define SWTRG	0x54
#define SCTL0	0x80
#define SCTL1	0x84
#define SCTL2	0x88
#define SCTL3	0x8C
#define SCTL4	0x90
#define SCTL5	0x94
#define SCTL6	0x98
#define SCTL7	0x9C
#define INTSRC0	0xD0
#define INTSRC1	0xD4
#define INTSRC2	0xD8
#define INTSRC3	0xDC
#define STATUS0	0xF0
#define STATUS2	0xF8
#define STATUS3	0xFC
#define PWRM	0x110
#define SELSMP0	0x140
#define SELSMP1	0x144
#define REFADJCTL	0x150

#define ADCEN		1
#define PWRUPRDY	1
#define ADCIEN0		4
#define CHSELMSK	0xF
#define DATMSK		0xFFF

#define MA35D1_ADC_TIMEOUT	(msecs_to_jiffies(1000))

#define ADC_CHANNEL(_index, _id) {			\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = _index,				\
	.address = _index,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.datasheet_name = _id,				\
	.scan_index = _index,				\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = 12,				\
		.storagebits = 16,			\
		.shift = 0,				\
		.endianness = IIO_BE,			\
	},						\
}

struct ma35d1_adc_device {
	struct clk	*clk;
	struct clk	*eclk;
	unsigned int	irq;
	void __iomem	*regs;
	struct completion	completion;
	struct iio_trigger	*trig;
};

static const struct iio_chan_spec ma35d1_adc_iio_channels[] = {
	ADC_CHANNEL(0, "adc0"),
	ADC_CHANNEL(1, "adc1"),
	ADC_CHANNEL(2, "adc2"),
	ADC_CHANNEL(3, "adc3"),
	ADC_CHANNEL(4, "adc4"),
	ADC_CHANNEL(5, "adc5"),
	ADC_CHANNEL(6, "adc6"),
	ADC_CHANNEL(7, "adc7"),

};

static irqreturn_t ma35d1_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int val;
	int channel;
	unsigned long timeout;

	channel = find_first_bit(indio_dev->active_scan_mask,
				indio_dev->masklength);

	// enable channel
	writel((readl(info->regs + SCTL0) & ~CHSELMSK) | channel, info->regs + SCTL0);

	// software trigger sample module 0
	writel(1, info->regs + SWTRG);

	timeout = wait_for_completion_interruptible_timeout
			(&info->completion, MA35D1_ADC_TIMEOUT);

	val = readl(info->regs + DAT0);

	iio_push_to_buffers(indio_dev, (void *)&val);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t ma35d1_adc_isr(int irq, void *dev_id)
{
	struct ma35d1_adc_device *info = (struct ma35d1_adc_device *)dev_id;

	if (readl(info->regs + STATUS2) & 1) {  //check ADIF0
		writel(1, info->regs + STATUS2); //clear ADIF0
		complete(&info->completion);
	}

	return IRQ_HANDLED;
}

static void ma35d1_adc_channels_remove(struct iio_dev *indio_dev)
{
	kfree(indio_dev->channels);
}

static void ma35d1_adc_buffer_remove(struct iio_dev *idev)
{
	iio_triggered_buffer_cleanup(idev);
}

static int ma35d1_adc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	unsigned long timeout;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	// enable channel
	writel((readl(info->regs + SCTL0) & ~CHSELMSK)
		| chan->channel, info->regs + SCTL0);

	// software trigger sample module 0
	writel(1, info->regs + SWTRG);

	timeout = wait_for_completion_interruptible_timeout
			(&info->completion, MA35D1_ADC_TIMEOUT);

	*val = readl(info->regs + DAT0) & DATMSK;

	mutex_unlock(&indio_dev->mlock);

	if (timeout == 0)
		return -ETIMEDOUT;

	return IIO_VAL_INT;
}

static int ma35d1_ring_preenable(struct iio_dev *indio_dev)
{
	return 0;
}

static const struct iio_buffer_setup_ops ma35d1_ring_setup_ops = {
	.preenable = &ma35d1_ring_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static const struct iio_info ma35d1_adc_info = {
	.read_raw = &ma35d1_adc_read_raw,
};

static int ma35d1_adc_probe(struct platform_device *pdev)
{
	struct iio_dev	*indio_dev;
	struct ma35d1_adc_device *info = NULL;
	int ret = -ENODEV;
	struct resource *res;
	int irq;
	int err = 0;
	struct pinctrl *p;
	struct device *dev = &pdev->dev;
	const char *clkgate;
	u32 freq;

	indio_dev = iio_device_alloc(sizeof(struct ma35d1_adc_device));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		ret = -ENOMEM;
		goto err_ret;
	}

	info = iio_priv(indio_dev);

	/* map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_ret;
	}

	info->regs = ioremap(res->start, resource_size(res));
	if (info->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ret;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ma35d1_adc_info;
	indio_dev->num_channels = 8;
	indio_dev->channels = ma35d1_adc_iio_channels;
	indio_dev->masklength = indio_dev->num_channels - 1;

	/* find the clock and enable it */

	if (of_property_read_u32(pdev->dev.of_node, "eadc-frequency", &freq) < 0)
		panic("missing 'eadc-frequency' property");

	of_property_read_string(pdev->dev.of_node, "clock-enable", &clkgate);
	info->eclk = devm_clk_get(dev, clkgate);
	if (IS_ERR(info->eclk)) {
		if (PTR_ERR(info->eclk) != -ENOENT)
			return PTR_ERR(info->eclk);

		info->eclk = NULL;
	}

	err = clk_prepare_enable(info->eclk);
	if (err) {
		err = -ENOENT;
		goto err_ret;
	}

	clk_set_rate(info->eclk, freq);

	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve eadc pin by mode\n");
		err = PTR_ERR(p);
		goto err_ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = irq;
		goto err_ret;
	}

	info->irq = irq;

	init_completion(&info->completion);

	ret = request_irq(info->irq, ma35d1_adc_isr,
				0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n",
			info->irq);
		goto err_ret;
	}

	// enable ADCEN
	writel(readl(info->regs + CTL) | ADCEN, info->regs + CTL);

	// clear ADIF0
	writel(1, info->regs + STATUS2);
	// enable ADCIEN0
	writel(readl(info->regs + CTL) | ADCIEN0, info->regs + CTL);
	// enable ADINT0 for sample module 0
	writel(readl(info->regs + INTSRC0) | 1, info->regs + INTSRC0);
	// set reference voltage from external Vref pin
	writel(readl(info->regs + REFADJCTL) | 1, info->regs + REFADJCTL);

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					&ma35d1_trigger_handler, &ma35d1_ring_setup_ops);
	if (ret)
		goto err_free_channels;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't register MA35D1 EADC..\n");
		goto err_free_channels;
	}

	platform_set_drvdata(pdev, indio_dev);

	dev_dbg(&pdev->dev, "%s: ma35d1 EADC\n",
		indio_dev->name);

	// dummy conversion, since the first conversion is incorrect
	writel(1, info->regs + SWTRG);

	return 0;

err_free_channels:
	// disable ADCEN
	writel(readl(info->regs + CTL) & ~ADCEN, info->regs + CTL);
	ma35d1_adc_channels_remove(indio_dev);
err_ret:
	return ret;
}

static int ma35d1_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ma35d1_adc_device *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ma35d1_adc_channels_remove(indio_dev);

	iio_device_free(indio_dev);

	// disable ADCEN
	writel(readl(info->regs + CTL) & ~ADCEN, info->regs + CTL);

	clk_disable_unprepare(info->eclk);

	ma35d1_adc_buffer_remove(indio_dev);
	free_irq(info->irq, info);

	return 0;
}

#ifdef CONFIG_PM
static int ma35d1_adc_suspend(struct device *dev)
{
	return 0;
}

static int ma35d1_adc_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ma35d1_adc_pm_ops = {
	.suspend = ma35d1_adc_suspend,
	.resume = ma35d1_adc_resume,
};
#define MA35D1_ADC_PM_OPS (&ma35d1_adc_pm_ops)
#else
#define MA35D1_ADC_PM_OPS NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id ma35d1_eadc_of_match[] = {
	{	.compatible = "nuvoton,ma35d1-eadc" },
	{	},
};
MODULE_DEVICE_TABLE(of, ma35d1_eadc_of_match);
#endif

static struct platform_driver ma35d1_adc_driver = {
	.driver = {
		.name   = "ma35d1-eadc",
		.owner	= THIS_MODULE,
		.pm	= MA35D1_ADC_PM_OPS,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(ma35d1_eadc_of_match),
#endif
	},
	.probe	= ma35d1_adc_probe,
	.remove	= ma35d1_adc_remove,
};

module_platform_driver(ma35d1_adc_driver);

MODULE_DESCRIPTION("MA35D1 EADC controller driver");
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-eadc");
