// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MA35D1 EADC driver
 *
 * Copyright (c) 2022 Nuvoton Technology Corp.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* ma35d1 eadc registers offset */
#define DAT0    0x00
#define DAT1    0x04
#define DAT2    0x08
#define DAT3    0x0C
#define DAT4    0x10
#define DAT5    0x14
#define DAT6    0x18
#define DAT7    0x1C
#define DAT8    0x20
#define CTL     0x50
#define SWTRG   0x54
#define SCTL0   0x80
#define SCTL1   0x84
#define SCTL2   0x88
#define SCTL3   0x8C
#define SCTL4   0x90
#define SCTL5   0x94
#define SCTL6   0x98
#define SCTL7   0x9C
#define INTSRC0 0xD0
#define INTSRC1 0xD4
#define INTSRC2 0xD8
#define INTSRC3 0xDC
#define STATUS0 0xF0
#define STATUS2 0xF8
#define STATUS3 0xFC
#define PWRM    0x110
#define SELSMP0 0x140
#define SELSMP1 0x144
#define REFADJCTL       0x150

#define ADCEN           1
#define DIFFEN          0x100
#define PWRUPRDY        1
#define ADCIEN0         4
#define CHSELMSK        0xF
#define DATMSK          0xFFF
#define TRGSELMSK       0x3F0000
#define TRGDLYMSK       0xFF00
#define ADINT0TRG       0x20000
#define TRGSELPOS       16

#define ADC_TIMEOUT			msecs_to_jiffies(1000)
#define ADC_MAX_CHANNELS		8

struct ma35d1_adc_data {
	const struct iio_chan_spec	*channels;
	int				num_channels;
	unsigned long			clk_rate;
};

struct ma35d1_adc {
	void __iomem		*regs;
	struct clk		*clk;
	struct completion	completion;
	struct regulator	*vref;
	struct reset_control	*reset;
	const struct ma35d1_adc_data *data;
	u16			last_val;
	const struct iio_chan_spec *last_chan;
};

static void ma35d1_adc_power_down(struct ma35d1_adc *info)
{
	/* Clear irq & power down adc */
}

static int ma35d1_adc_conversion(struct ma35d1_adc *info,
				   struct iio_chan_spec const *chan)
{
	reinit_completion(&info->completion);

	info->last_chan = chan;

	// enable channel
        writel((readl(info->regs + SCTL0) & ~CHSELMSK)
                | chan->channel, info->regs + SCTL0);

        // check if the channel is designated as differential channel
        if (chan->differential)
                writel((readl(info->regs + CTL) | DIFFEN), info->regs + CTL);
        else
                writel((readl(info->regs + CTL) & ~DIFFEN), info->regs + CTL);

        // software trigger sample module 0
        writel(1, info->regs + SWTRG);

	if (!wait_for_completion_timeout(&info->completion, ADC_TIMEOUT))
		return -ETIMEDOUT;

	return 0;
}

static int ma35d1_adc_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    int *val, int *val2, long mask)
{
	struct ma35d1_adc *info = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		ret = ma35d1_adc_conversion(info, chan);
		if (ret) {
			ma35d1_adc_power_down(info);
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		*val = info->last_val;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(info->vref);
		if (ret < 0) {
			dev_err(&indio_dev->dev, "failed to get voltage\n");
			return ret;
		}

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static irqreturn_t ma35d1_adc_isr(int irq, void *dev_id)
{
	struct ma35d1_adc *info = dev_id;

	if (readl(info->regs + STATUS2) & 1) {  //check ADIF0
                writel(1, info->regs + STATUS2); //clear ADIF0
	}

	/* Read value */
	info->last_val = readl(info->regs + DAT0);
	info->last_val &= GENMASK(info->last_chan->scan_type.realbits - 1, 0);

	ma35d1_adc_power_down(info);

	complete(&info->completion);

	return IRQ_HANDLED;
}

static const struct iio_info ma35d1_adc_iio_info = {
	.read_raw = ma35d1_adc_read_raw,
};

#define SARADC_CHANNEL(_index, _id) {   			\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = _index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.datasheet_name = _id,					\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 12, 				\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
}

static const struct iio_chan_spec ma35d1_adc_iio_channels[] = {
	SARADC_CHANNEL(0, "adc0"),
	SARADC_CHANNEL(1, "adc1"),
	SARADC_CHANNEL(2, "adc2"),
	SARADC_CHANNEL(3, "adc3"),
	SARADC_CHANNEL(4, "adc4"),
	SARADC_CHANNEL(5, "adc5"),
	SARADC_CHANNEL(6, "adc6"),
	SARADC_CHANNEL(7, "adc7"),
};

static const struct ma35d1_adc_data ma35d1_adc_data = {
	.channels = ma35d1_adc_iio_channels,
	.num_channels = ARRAY_SIZE(ma35d1_adc_iio_channels),
	.clk_rate = 1000000,
};

static const struct of_device_id ma35d1_adc_match[] = {
	{
		.compatible = "nuvoton,ma35d1-eadc",
		.data = &ma35d1_adc_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_adc_match);

/*
 * Reset SARADC Controller.
 */
static void ma35d1_adc_reset_controller(struct reset_control *reset)
{
	reset_control_assert(reset);
	usleep_range(10, 20);
	reset_control_deassert(reset);
}

static void ma35d1_adc_clk_disable(void *data)
{
	struct ma35d1_adc *info = data;

	clk_disable_unprepare(info->clk);
}

#if 0
static void ma35d1_adc_regulator_disable(void *data)
{
	struct ma35d1_adc *info = data;

	regulator_disable(info->vref);
}
#endif

static irqreturn_t ma35d1_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *i_dev = pf->indio_dev;
	struct ma35d1_adc *info = iio_priv(i_dev);
	/*
	 * @values: each channel takes an u16 value
	 * @timestamp: will be 8-byte aligned automatically
	 */
	struct {
		u16 values[ADC_MAX_CHANNELS];
		int64_t timestamp;
	} data;
	int ret;
	int i, j = 0;

	mutex_lock(&i_dev->mlock);

	for_each_set_bit(i, i_dev->active_scan_mask, i_dev->masklength) {
		const struct iio_chan_spec *chan = &i_dev->channels[i];

		ret = ma35d1_adc_conversion(info, chan);
		if (ret) {
			ma35d1_adc_power_down(info);
			goto out;
		}

		data.values[j] = info->last_val;
		j++;
	}

	iio_push_to_buffers_with_timestamp(i_dev, &data, iio_get_time_ns(i_dev));
out:
	mutex_unlock(&i_dev->mlock);

	iio_trigger_notify_done(i_dev->trig);

	return IRQ_HANDLED;
}

static int ma35d1_adc_probe(struct platform_device *pdev)
{
	struct ma35d1_adc *info = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct iio_dev *indio_dev = NULL;
	struct resource	*mem;
	const struct of_device_id *match;
	int ret;
	int irq;

	if (!np)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed allocating iio device\n");
		return -ENOMEM;
	}
	info = iio_priv(indio_dev);

	match = of_match_device(ma35d1_adc_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to match device\n");
		return -ENODEV;
	}

	info->data = match->data;

	/* Sanity check for possible later IP variants with more channels */
	if (info->data->num_channels > ADC_MAX_CHANNELS) {
		dev_err(&pdev->dev, "max channels exceeded");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(info->regs))
		return PTR_ERR(info->regs);

#if 0
	/*
	 * The reset should be an optional property, as it should work
	 * with old devicetrees as well
	 */
	info->reset = devm_reset_control_get_exclusive(&pdev->dev,
						       "saradc-apb");
	if (IS_ERR(info->reset)) {
		ret = PTR_ERR(info->reset);
		if (ret != -ENOENT)
			return ret;

		dev_dbg(&pdev->dev, "no reset control found\n");
		info->reset = NULL;
	}
#endif

	init_completion(&info->completion);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, ma35d1_adc_isr,
			       0, dev_name(&pdev->dev), info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed requesting irq %d\n", irq);
		return ret;
	}

	info->clk = devm_clk_get(&pdev->dev, "eadc_gate");
	if (IS_ERR(info->clk)) {
		dev_err(&pdev->dev, "failed to get eadc clock\n");
		return PTR_ERR(info->clk);
	}

	info->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(info->vref)) {
		dev_err(&pdev->dev, "failed to get regulator, %ld\n",
			PTR_ERR(info->vref));
		return PTR_ERR(info->vref);
	}

	if (info->reset)
		ma35d1_adc_reset_controller(info->reset);

	/*
	 * Use a default value for the converter clock.
	 * This may become user-configurable in the future.
	 */
	ret = clk_set_rate(info->clk, info->data->clk_rate);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set eadc clk rate, %d\n", ret);
		return ret;
	}

#if 0
	ret = regulator_enable(info->vref);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable vref regulator\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       ma35d1_adc_regulator_disable, info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register devm action, %d\n",
			ret);
		return ret;
	}
#endif

	ret = clk_prepare_enable(info->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable converter clock\n");
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev,
				       ma35d1_adc_clk_disable, info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register devm action, %d\n",
			ret);
		return ret;
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

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &ma35d1_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = info->data->channels;
	indio_dev->num_channels = info->data->num_channels;
	ret = devm_iio_triggered_buffer_setup(&indio_dev->dev, indio_dev, NULL,
					      ma35d1_adc_trigger_handler,
					      NULL);
	if (ret)
		return ret;

	printk("%s: ma35d1 EADC adapter\n",
		indio_dev->name);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

#ifdef CONFIG_PM_SLEEP
static int ma35d1_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ma35d1_adc *info = iio_priv(indio_dev);

	clk_disable_unprepare(info->clk);
	regulator_disable(info->vref);

	return 0;
}

static int ma35d1_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ma35d1_adc *info = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(info->vref);
	if (ret)
		return ret;

	ret = clk_prepare_enable(info->clk);
	if (ret)
		clk_disable_unprepare(info->clk);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(ma35d1_adc_pm_ops,
			 ma35d1_adc_suspend, ma35d1_adc_resume);

static struct platform_driver ma35d1_adc_driver = {
	.probe		= ma35d1_adc_probe,
	.driver		= {
		.name	= "ma35d1-eadc",
		.of_match_table = ma35d1_adc_match,
		.pm	= &ma35d1_adc_pm_ops,
	},
};

module_platform_driver(ma35d1_adc_driver);

MODULE_DESCRIPTION("MA35D1 EADC driver");
MODULE_LICENSE("GPL v2");
