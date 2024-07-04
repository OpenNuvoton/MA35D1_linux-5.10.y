// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Nuvoton Technology Corp.
 */

#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/triggered_event.h>

/* ADC Control  */
#define REG_ADC_CTL             (0x000)

#define ADC_CTL_ADEN            (1 << 0)
#define ADC_CTL_MST             (1 << 8)
#define ADC_CTL_PEDEEN          (1 << 9)
#define ADC_CTL_WKTEN           (1 << 11)
#define ADC_CTL_WMSWCH          (1 << 16)

/* ADC Configure  */
#define REG_ADC_CONF            (0x004)

#define ADC_CONF_TEN            (1 << 0)
#define ADC_CONF_ZEN            (1 << 1)
#define ADC_CONF_NACEN          (1 << 2)
#define ADC_CONF_REFSEL_POS     (6)
#define ADC_CONF_REFSEL_MSK     (3 << 6)
#define ADC_CONF_REFSEL_AVDD    (3 << 6)
#define ADC_CONF_CHSEL_POS      (12)
#define ADC_CONF_CHSEL_MSK      (7 << 12)
#define ADC_CONF_TMAVDIS        (1 << 20)
#define ADC_CONF_ZMAVDIS        (1 << 21)
#define ADC_CONF_SPEED          (1 << 22)

/* ADC Interrupt Enable Register */
#define REG_ADC_IER             (0x008)

#define ADC_IER_MIEN            (1 << 0)
#define ADC_IER_PEDEIEN         (1 << 2)
#define ADC_IER_WKTIEN          (1 << 3)
#define ADC_IER_PEUEIEN         (1 << 6)

/* ADC Interrupt Status Register */
#define REG_ADC_ISR             (0x00C)

#define ADC_ISR_MF              (1 << 0)
#define ADC_ISR_PEDEF           (1 << 2)
#define ADC_ISR_PEUEF           (1 << 4)
#define ADC_ISR_TF              (1 << 8)
#define ADC_ISR_ZF              (1 << 9)
#define ADC_ISR_NACF            (1 << 10)
#define ADC_ISR_INTTC           (1 << 17)

/* ADC Normal Conversion Data */
#define REG_ADC_DATA            (0x028)

#define ADC_DEFAULT_CLK_RATE    100000000

#define ADC_CONV_TIMEOUT        (msecs_to_jiffies(100))
#define ADC_MENU_TIMEOUT        (msecs_to_jiffies(1000))

#define ADC_CH_NUM              8


/* device related private data */
struct ma35_iio_priv {
	void __iomem		*base;
	struct completion	conv_completion;
	struct completion   menu_completion;

	u32                 vref_mv;
	
	/* data buffer needs space for channel data and timestamp */
	struct {
		u16 chan[ADC_CH_NUM];
		u64 ts __aligned(8);
	} scan;
};

static irqreturn_t ma35_iio_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = (struct iio_dev *)private;
	struct ma35_iio_priv *priv = iio_priv(indio_dev);
	u32 isr;

	isr = __raw_readl(priv->base + REG_ADC_ISR);

	if (isr & ADC_ISR_NACF) {
		/* EOC: set by hardware, write 1 to clear it */
		complete(&priv->conv_completion);
	}

	if (isr & ADC_ISR_MF) {
		/* MST: set by hardware, write 1 to clear it */
		/* dev_info(&indio_dev->dev, "Function menu completed\n"); */
		complete(&priv->menu_completion);
	}

	__raw_writel(isr, priv->base + REG_ADC_ISR);

	return IRQ_HANDLED;
}

static int ma35_read(struct iio_dev *indio_dev, int channel)
{
	struct ma35_iio_priv *priv = iio_priv(indio_dev);
	long timeout;

	reinit_completion(&priv->conv_completion);

	/* Disabel touch detection, select AGND33 vs AVDD33 and channel */
	__raw_writel((__raw_readl(priv->base + REG_ADC_CONF) &
			~(ADC_CONF_TEN | ADC_CONF_ZEN |
			ADC_CONF_REFSEL_MSK | ADC_CONF_CHSEL_MSK)) |
			ADC_CONF_REFSEL_AVDD | (channel << ADC_CONF_CHSEL_POS)
			| ADC_CONF_NACEN, priv->base + REG_ADC_CONF);

	/* Config Interrupt */
	__raw_writel((__raw_readl(priv->base + REG_ADC_IER) &
		~ADC_IER_PEDEIEN) | ADC_IER_MIEN, priv->base + REG_ADC_IER);

	// Disable pen down detection enable MST
	__raw_writel((__raw_readl(priv->base + REG_ADC_CTL) &
		~ADC_CTL_PEDEEN) | ADC_CTL_MST, priv->base + REG_ADC_CTL);

	timeout = wait_for_completion_interruptible_timeout(&priv->conv_completion, ADC_CONV_TIMEOUT);
	if (timeout)
		return __raw_readl(priv->base + REG_ADC_DATA);
	else
		return -ENODATA;
}

static int ma35_adc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	struct ma35_iio_priv *priv = iio_priv(indio_dev);
	int raw_val, ret;

	if (mask == IIO_CHAN_INFO_RAW) {
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret < 0)
			return -EBUSY;

		raw_val = ma35_read(indio_dev, chan->channel);

		iio_device_release_direct_mode(indio_dev);
		
		if (raw_val < 0) {
			return -ENODATA;
		}

		*val = raw_val;
		return IIO_VAL_INT;
	}

	if (mask == IIO_CHAN_INFO_SCALE) {
		*val = priv->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

static irqreturn_t ma35_iio_buffer_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ma35_iio_priv *priv = iio_priv(indio_dev);
	int scan_index, val = 0, i = 0;
	unsigned long timeout;

	for_each_set_bit(scan_index, indio_dev->active_scan_mask, indio_dev->masklength) {
		const struct iio_chan_spec *scan_chan = &indio_dev->channels[scan_index];
	
		reinit_completion(&priv->menu_completion);
			
		val = ma35_read(indio_dev, scan_chan->channel);
		if (val == -ENODATA) {
			dev_err(&indio_dev->dev, "failed to get conversion data\n");
			return IRQ_NONE;
		}
	
		priv->scan.chan[i++] = val;
	
		timeout = wait_for_completion_interruptible_timeout(
			&priv->menu_completion, ADC_MENU_TIMEOUT);
		if (timeout <= 0) {
			dev_err(&indio_dev->dev, "failed to wait for mf completion\n");
			return IRQ_NONE;
		}
	}
	
	iio_push_to_buffers_with_timestamp(indio_dev, &priv->scan, 
		iio_get_time_ns(indio_dev));
	
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_info ma35_adc_info = {
	.read_raw = &ma35_adc_read_raw,
};

static int ma35_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ma35_iio_priv *priv;
	struct iio_chan_spec *chan_array, *timestamp;
	struct device_node *np = pdev->dev.of_node;
	struct clk *clock;
	u32 clock_rate, min_clock_freq, max_clock_freq, vref;
	int irq, ret, bit, idx = 0;
	unsigned long mask;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*priv));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocate an IIO device\n");
		return -ENOMEM;
	}

	priv = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return irq;
	
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	clock = devm_clk_get(&pdev->dev, "adc_gate");
	if (IS_ERR(clock))
		return PTR_ERR(clock);

	if (of_property_read_u32(np, "clock-rate", &clock_rate)) {
		dev_warn(&pdev->dev, "Missing clock-rate in the DT, set to 1MHz\n");
		clock_rate = ADC_DEFAULT_CLK_RATE;
	}

	ret = clk_prepare_enable(clock);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		return ret;
	}

	clk_set_rate(clock, clock_rate);

	if (of_property_read_u32(np, "nuvoton,min-clock-frequency", &min_clock_freq)) {
		dev_warn(&pdev->dev, "Missing min-clock-frequency\n");
	}

	if (of_property_read_u32(np, "nuvoton,max-clock-frequency", &max_clock_freq)) {
		dev_warn(&pdev->dev, "Missing max-clock-frequency\n");
	}

	if (of_property_read_u32(np, "nuvoton,adc-vref", &vref)) {
		dev_err(&pdev->dev, "Missing adc-vref property in the DT.\n");
		return -EINVAL;
	}
	priv->vref_mv = vref;

	/* Set ADC clock speed mode low or high */
	if (clock_rate < min_clock_freq)
		__raw_writel(__raw_readl(priv->base + REG_ADC_CONF) |
					ADC_CONF_SPEED, priv->base + REG_ADC_CONF);
	else
		__raw_writel(__raw_readl(priv->base + REG_ADC_CONF) &
					~ADC_CONF_SPEED, priv->base + REG_ADC_CONF);
	
	/* Clear all interrupt flags */
	__raw_writel(0, priv->base + REG_ADC_IER);

	/* Power on ADC */
	__raw_writel(ADC_CTL_ADEN, priv->base + REG_ADC_CTL);

	ret = devm_request_irq(&pdev->dev, irq, ma35_iio_irq_handler,
				0, dev_name(&pdev->dev), indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		return ret;
	}

	init_completion(&priv->conv_completion);
	init_completion(&priv->menu_completion);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ma35_adc_info;

	/* IIO consumers: 4-/5-wire resistive ADC touch */
	/* 4-wire: mask = 0xF */
	/* 5-wire: mask = 0x7 */
	mask =  0xFF;
	
	/* timestamp channel (+1) */
	indio_dev->num_channels = bitmap_weight(&mask, ADC_CH_NUM) + 1;

	chan_array = devm_kzalloc(&pdev->dev,
					((indio_dev->num_channels + 1) *
					sizeof(struct iio_chan_spec)),
					GFP_KERNEL);
	if (!chan_array) {
		dev_err(&pdev->dev, "Failed to allocate memory for channels\n");
		return -ENOMEM;
	}

	for_each_set_bit(bit, &mask, ADC_CH_NUM) {
		struct iio_chan_spec *chan = chan_array + idx;

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = bit;
		chan->scan_index = idx;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 16;
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		idx++;
	}

	timestamp = chan_array + idx;
	timestamp->type = IIO_TIMESTAMP;
	timestamp->channel = -1;
	timestamp->scan_index = idx;
	timestamp->scan_type.sign = 's';
	timestamp->scan_type.realbits = 64;
	timestamp->scan_type.storagebits = 64;

	indio_dev->channels = chan_array;

	ret = devm_iio_triggered_buffer_setup(&indio_dev->dev, indio_dev, 
				&iio_pollfunc_store_time, ma35_iio_buffer_trigger_handler, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to setup iio triggered buffer\n");
		return ret;
	}
	
	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register iio device\n");
		/* Power down the ADC */
		__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_ADEN,
			priv->base + REG_ADC_CTL);
		
		return ret;
	}

	return 0;
}

static int ma35_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ma35_iio_priv *priv = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	/* Power down the ADC */
	__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_ADEN,
		priv->base + REG_ADC_CTL);

	return 0;
}

static const struct of_device_id ma35_adc_match[] = {
	{ .compatible = "nuvoton,ma35-adc" },
	{ },
};
MODULE_DEVICE_TABLE(of, ma35_adc_match);

static struct platform_driver ma35_adc_driver = {
	.driver	= {
		.name		= "ma35-adc",
		.of_match_table	= ma35_adc_match,
	},
	.probe	= ma35_adc_probe,
	.remove	= ma35_adc_remove,
};
module_platform_driver(ma35_adc_driver);

MODULE_AUTHOR("Arthur Jiang <twjiang@nuvoton.com>");
MODULE_DESCRIPTION("MA35 IIO ADC driver");
MODULE_LICENSE("GPL v2");
