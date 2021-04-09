// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2020 Nuvoton Technology Corp.
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>


#define REG_ADC_CTL		(0x000)	/* ADC Control  */
#define REG_ADC_CONF		(0x004)	/* ADC Configure  */
#define REG_ADC_IER		(0x008)	/* ADC Interrupt Enable Register */
#define REG_ADC_ISR		(0x00C) /* ADC Interrupt Status Register */
#define REG_ADC_WKISR		(0x010) /* ADC Wake Up Interrupt Status Register */
#define REG_ADC_XYDATA		(0x020) /* ADC Touch X,Y Position Data  */
#define REG_ADC_ZDATA		(0x024) /* ADC Touch Z Pressure Data  */
#define REG_ADC_DATA		(0x028) /* ADC Normal Conversion Data  */

#define ADC_CTL_ADEN		(1 << 0)
#define ADC_CTL_MST		(1 << 8)
#define ADC_CTL_PEDEEN		(1 << 9)
#define ADC_CTL_WKTEN		(1 << 11)
#define ADC_CTL_WMSWCH		(1 << 16)

#define ADC_CONF_TEN		(1 << 0)
#define ADC_CONF_ZEN		(1 << 1)
#define ADC_CONF_NACEN		(1 << 2)
#define ADC_CONF_HSPEED		(1 << 22)

#define ADC_CONF_CHSEL_POS	12
#define ADC_CONF_CHSEL_MSK	(7 << 12)

#define ADC_CONF_REFSEL_POS	6
#define ADC_CONF_REFSEL_MSK	(3 << 6)
#define ADC_CONF_REFSEL_AVDD	(3 << 6)

#define ADC_IER_MIEN		(1 << 0)
#define ADC_IER_PEDEIEN		(1 << 2)
#define ADC_IER_WKTIEN		(1 << 3)
#define ADC_IER_PEUEIEN		(1 << 6)

#define ADC_ISR_MF		(1 << 0)
#define ADC_ISR_PEDEF		(1 << 2)
#define ADC_ISR_PEUEF		(1 << 4)
#define ADC_ISR_TF		(1 << 8)
#define ADC_ISR_ZF		(1 << 9)
#define ADC_ISR_NACF		(1 << 10)
#define ADC_ISR_INTTC		(1 << 17)

#define ADC_WKISR_WPEDEF	(1 << 1)

#define ADC_CH_NUM		8
#define ADC_RESOLUTION		12	// 12-bit data
#define ADC_DATA_MASK		0xFFF
#define ADC_DATA_SHIFT		16
#define ADC_CONV_TIMEOUT	(msecs_to_jiffies(100))
#define ADC_TS_SPS		(HZ / 100)
#define ADC_DEFAULT_CLK_RATE	100000000
#define ADC_DEFAULT_P_THRESHOLD	100

enum touch_state {
	TS_STOP,	/* TS stop */
	TS_CONVERT,	/* Converting X/Y */
	TS_PENDOWN,	/* Detecting pen down event */
	TS_PAUSE,	/* Relinquish TS function, let IIO use ADC channel */
	TS_WAKE,	/* Detect wakeup event */
};

struct ma35d1_adc {
	struct input_dev	*ts_dev;
	struct iio_dev		*indio_dev;

	void __iomem		*base;
	int			irq;
	struct clk		*clk;
	u32			clk_rate;

	spinlock_t		lock;

	enum touch_state	ts_state;
	enum touch_state	prev_ts_state;
	struct hrtimer		trigger_hrt;
	struct completion	completion;
	int			enable_ts;
	int			enable_iio;
	int			enable_ts_wk;
	int			ts_type;
	int			p_th;
};

static int report_touch(struct ma35d1_adc *priv);
static void detect_touch(struct ma35d1_adc *priv);
static void detect_pendown(struct ma35d1_adc *priv);


static irqreturn_t ma35d1_adc_interrupt(int irq, void *private)
{
	struct ma35d1_adc *priv = (struct ma35d1_adc *)private;
	u32 isr, wkisr;
	u32 chg_state = 0;

	isr = __raw_readl(priv->base + REG_ADC_ISR);
	wkisr = __raw_readl(priv->base + REG_ADC_WKISR);

	if (isr & ADC_ISR_PEDEF) {
		__raw_writel(ADC_ISR_PEDEF, priv->base + REG_ADC_ISR);
		if (priv->ts_state == TS_PENDOWN)
			chg_state = 1;
	}

	if (wkisr & ADC_WKISR_WPEDEF)
		__raw_writel(ADC_WKISR_WPEDEF, priv->base + REG_ADC_WKISR);

	if (chg_state)
		detect_touch(priv);

	if (isr & ADC_ISR_NACF) {
		complete(&priv->completion);
		__raw_writel(ADC_ISR_NACF, priv->base + REG_ADC_ISR);
	}

	if (isr & ADC_ISR_MF) {
		__raw_writel(ADC_ISR_MF, priv->base + REG_ADC_ISR);

		if ((isr & ADC_ISR_TF) && (isr & ADC_ISR_ZF))
			__raw_writel(ADC_ISR_TF | ADC_ISR_ZF,
					priv->base + REG_ADC_ISR);
			if (priv->ts_state == TS_CONVERT)
				report_touch(priv);
	}

	return IRQ_HANDLED;
}


// Touchscreen related function begins here

static enum hrtimer_restart trigger_hrtimer(struct hrtimer *hrtimer)
{
	struct ma35d1_adc *priv = container_of(hrtimer,
						struct ma35d1_adc,
						trigger_hrt);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) | ADC_CTL_MST,
			priv->base + REG_ADC_CTL);
	spin_unlock_irqrestore(&priv->lock, flags);

	return HRTIMER_NORESTART;
}

static void detect_touch(struct ma35d1_adc *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->ts_state = TS_CONVERT;
	/* Disable pen down detection*/
	__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) & ~(ADC_CTL_PEDEEN),
			priv->base + REG_ADC_CTL);
	/* Enable touch detection  */
	__raw_writel((__raw_readl(priv->base + REG_ADC_CONF) & ~ADC_CONF_NACEN) |
			ADC_CONF_TEN | ADC_CONF_ZEN,
			priv->base + REG_ADC_CONF);
	/* Config interrupt */
	__raw_writel((__raw_readl(priv->base + REG_ADC_IER) & ~ADC_IER_PEDEIEN) |
			ADC_IER_MIEN, priv->base + REG_ADC_IER);

	hrtimer_start(&priv->trigger_hrt,
			ms_to_ktime(ADC_TS_SPS),
			HRTIMER_MODE_REL);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void detect_pendown(struct ma35d1_adc *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->ts_state == TS_CONVERT)
		hrtimer_cancel(&priv->trigger_hrt);

	priv->ts_state = TS_PENDOWN;
	/* Disabel touch detection */
	__raw_writel(__raw_readl(priv->base + REG_ADC_CONF) &
			~(ADC_CONF_TEN | ADC_CONF_ZEN | ADC_CONF_NACEN),
			priv->base + REG_ADC_CONF);
	/* Enable pen down event */
	__raw_writel((__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_WKTEN) |
			ADC_CTL_PEDEEN, priv->base + REG_ADC_CTL);
	/* Config Interrupt */
	__raw_writel((__raw_readl(priv->base + REG_ADC_IER) &
			~(ADC_IER_MIEN | ADC_IER_WKTIEN)) | ADC_IER_PEDEIEN,
			priv->base + REG_ADC_IER);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void detect_wakeup(struct ma35d1_adc *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->ts_state == TS_CONVERT)
		hrtimer_cancel(&priv->trigger_hrt);

	priv->ts_state = TS_WAKE;
	/* Disabel touch detection */
	__raw_writel(__raw_readl(priv->base + REG_ADC_CONF) &
			~(ADC_CONF_TEN | ADC_CONF_ZEN | ADC_CONF_NACEN),
			priv->base + REG_ADC_CONF);
	/* Enable pen down event */
	__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) | ADC_CTL_PEDEEN | ADC_CTL_WKTEN,
			priv->base + REG_ADC_CTL);
	/* Config Interrupt */
	__raw_writel((__raw_readl(priv->base + REG_ADC_IER) & ~ADC_IER_MIEN) | ADC_IER_WKTIEN,
			priv->base + REG_ADC_IER);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static void stop_detect(struct ma35d1_adc *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->ts_state == TS_CONVERT)
		hrtimer_cancel(&priv->trigger_hrt);

	priv->ts_state = TS_STOP;
	/* Disabel touch detection */
	__raw_writel(__raw_readl(priv->base + REG_ADC_CONF) &
			~(ADC_CONF_TEN | ADC_CONF_ZEN | ADC_CONF_NACEN),
			priv->base + REG_ADC_CONF);
	/* Disable pen down detection*/
	__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_PEDEEN,
			priv->base + REG_ADC_CTL);
	/* Config Interrupt */
	__raw_writel(__raw_readl(priv->base + REG_ADC_IER) & ~ADC_IER_PEDEIEN,
			priv->base + REG_ADC_IER);

	spin_unlock_irqrestore(&priv->lock, flags);
}

static int report_touch(struct ma35d1_adc *priv)
{
	u32 x, y, z1, z2, p, reg;

	reg = __raw_readl(priv->base + REG_ADC_XYDATA);
	x = reg & ADC_DATA_MASK;
	y = reg >> ADC_DATA_SHIFT;
	reg = __raw_readl(priv->base + REG_ADC_ZDATA);
	z1 = reg & ADC_DATA_MASK;
	z2 = reg >> ADC_DATA_SHIFT;
	p = (x * (z2 - (z1 + 1))) / (z1 + 1);

	if (p < priv->p_th) {
		input_report_key(priv->ts_dev, BTN_TOUCH, 0);
		detect_pendown(priv);
	} else {
		input_report_key(priv->ts_dev, BTN_TOUCH, 1);
		input_report_abs(priv->ts_dev, ABS_X, x);
		input_report_abs(priv->ts_dev, ABS_Y, y);
		input_report_abs(priv->ts_dev, ABS_PRESSURE, p);

		hrtimer_start(&priv->trigger_hrt,
				ms_to_ktime(ADC_TS_SPS),
				HRTIMER_MODE_REL);
	}
	input_sync(priv->ts_dev);

	return 0;
}

static int ma35d1_ts_open(struct input_dev *dev)
{
	struct ma35d1_adc *priv = input_get_drvdata(dev);

	detect_pendown(priv);

	return 0;
}

static void ma35d1_ts_close(struct input_dev *dev)
{
	struct ma35d1_adc *priv = input_get_drvdata(dev);

	stop_detect(priv);
}

static int ma35d1_ts_register(struct platform_device *pdev)
{
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);
	struct input_dev *ts_dev;
	int ret;

	ts_dev = devm_input_allocate_device(&pdev->dev);
	if (!ts_dev)
		return -ENOMEM;
	ts_dev->name = "MA35D1 Touch Screen";
	ts_dev->phys = "ma35d1ts/event0";
	ts_dev->id.bustype = BUS_HOST;
	ts_dev->dev.parent = &pdev->dev;
	ts_dev->open = ma35d1_ts_open;
	ts_dev->close = ma35d1_ts_close;

	input_set_capability(ts_dev, EV_KEY, BTN_TOUCH);
	input_set_abs_params(ts_dev, ABS_X, 0, (1 << ADC_RESOLUTION) - 1, 0, 0);
	input_set_abs_params(ts_dev, ABS_Y, 0, (1 << ADC_RESOLUTION) - 1, 0, 0);
	input_set_abs_params(ts_dev, ABS_PRESSURE, 0, (1 << ADC_RESOLUTION) - 1, 0, 0);
	input_set_drvdata(ts_dev, priv);

	hrtimer_init(&priv->trigger_hrt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	priv->trigger_hrt.function = trigger_hrtimer;

	ret = input_register_device(ts_dev);
	if (ret) {
		input_free_device(ts_dev);
		return ret;
	}

	if (priv->ts_type == 5)
		__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) | ADC_CTL_WMSWCH,
			priv->base + REG_ADC_CTL);
	else
		__raw_writel(__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_WMSWCH,
			priv->base + REG_ADC_CTL);
	priv->ts_dev = ts_dev;
	return 0;
}

static void ma35d1_ts_unregister(struct platform_device *pdev)
{
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);

	input_unregister_device(priv->ts_dev);
}

// Touchscreen related function ends here

// IIO related function begins here

static void normal_convert(struct ma35d1_adc *priv, int ch)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (priv->ts_state != TS_PAUSE) {
		if (priv->ts_state == TS_CONVERT)
			hrtimer_cancel(&priv->trigger_hrt);

		priv->prev_ts_state = priv->ts_state;
		priv->ts_state = TS_PAUSE;
	}

	/* Disabel touch detection, select AGND33 vs AVDD33 and channel*/
	__raw_writel((__raw_readl(priv->base + REG_ADC_CONF) &
			~(ADC_CONF_TEN | ADC_CONF_ZEN | ADC_CONF_NACEN |
			ADC_CONF_REFSEL_MSK | ADC_CONF_CHSEL_MSK)) |
			ADC_CONF_REFSEL_AVDD | (ch << ADC_CONF_CHSEL_POS)
			| ADC_CONF_NACEN, priv->base + REG_ADC_CONF);

	/* Config Interrupt */
	__raw_writel((__raw_readl(priv->base + REG_ADC_IER) & ~ADC_IER_PEDEIEN) | ADC_IER_MIEN,
			priv->base + REG_ADC_IER);

	// Disable pen down detection enable MST
	__raw_writel((__raw_readl(priv->base + REG_ADC_CTL) & ~ADC_CTL_PEDEEN) | ADC_CTL_MST,
			priv->base + REG_ADC_CTL);

	spin_unlock_irqrestore(&priv->lock, flags);
}


static int ma35d1_adc_read_raw(struct iio_dev *idev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct ma35d1_adc *priv = iio_device_get_drvdata(idev);
	unsigned long timeout;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	reinit_completion(&priv->completion);
	normal_convert(priv, chan->channel);

	timeout = wait_for_completion_interruptible_timeout(&priv->completion,
								ADC_CONV_TIMEOUT);

	*val = __raw_readl(priv->base + REG_ADC_DATA);

	priv->ts_state = priv->prev_ts_state;

	if (priv->ts_state == TS_PENDOWN)
		detect_pendown(priv);
	else if (priv->ts_state == TS_CONVERT)
		detect_touch(priv);

	if (timeout == 0)
		return -ETIMEDOUT;

	return IIO_VAL_INT;
}


static const struct iio_info ma35d1_adc_info = {
	.read_raw = &ma35d1_adc_read_raw,
};

static int ma35d1_iio_register(struct platform_device *pdev)
{
	int ret, bit, idx = 0;
	struct iio_dev *indio_dev;
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);
	struct iio_chan_spec *chan_array;
	unsigned long mask;

	indio_dev = devm_iio_device_alloc(&pdev->dev, 0);
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		return -ENOMEM;
	}
	iio_device_set_drvdata(indio_dev, priv);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ma35d1_adc_info;

	/* If touchscreen is enable, then reserve the adc channels */
	if (priv->ts_type == 4)
		mask = 0xF;
	else if (priv->ts_type == 5)
		mask = 0x7;
	else
		mask = 0xFF;
	/* set up the channel mask to reserve touchscreen channels */

	indio_dev->num_channels = bitmap_weight(&mask, ADC_CH_NUM);

	chan_array = devm_kzalloc(&pdev->dev,
					(indio_dev->num_channels *
					sizeof(struct iio_chan_spec)),
					GFP_KERNEL);

	for_each_set_bit(bit, &mask, ADC_CH_NUM) {
		struct iio_chan_spec *chan = chan_array + idx;

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = bit;
		chan->scan_index = idx;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = ADC_RESOLUTION;
		chan->scan_type.storagebits = 16;
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		idx++;
	}

	indio_dev->channels = chan_array;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't register iio device\n");
		iio_device_free(priv->indio_dev);
		return ret;
	}
	priv->indio_dev = indio_dev;
	return 0;
}

static int ma35d1_iio_unregister(struct platform_device *pdev)
{
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);

	iio_device_free(priv->indio_dev);

	return 0;
}

// IIO related function ends here

static int ma35d1_adc_probe_dt(struct ma35d1_adc *priv,
			     struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const char *clkgate;

	if (!node)
		return -EINVAL;

	priv->enable_iio = of_property_read_bool(node, "enable-iio");
	priv->enable_ts = of_property_read_bool(node, "enable-ts");
	priv->enable_ts_wk = of_property_read_bool(node, "enable-wakeup");

	if (priv->enable_ts) {
		if (of_property_read_u32(node, "ts-type", &priv->ts_type)) {
			dev_warn(&pdev->dev,
				"Missing ts-type property in the DT, select 4-wire mode\n");
			priv->ts_type = 4;

		}
		if (of_property_read_u32(node, "ts-pressure-threshold", &priv->p_th)) {
			dev_info(&pdev->dev,
				"Use default pressure threshold select\n");
			priv->p_th = ADC_DEFAULT_P_THRESHOLD;

		}
	} else
		priv->ts_type = 0;

	of_property_read_string(node, "clock-enable", &clkgate);
	priv->clk = devm_clk_get(&pdev->dev, clkgate);
	if (IS_ERR(priv->clk)) {
		return PTR_ERR(priv->clk);
	}

	if (of_property_read_u32(node, "clock-rate", &priv->clk_rate)) {
		dev_warn(&pdev->dev,
			"Missing clock-rate in the DT, set to 1MHz\n");
		priv->clk_rate = ADC_DEFAULT_CLK_RATE;

	}

	return 0;
}

static int ma35d1_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ma35d1_adc *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = ma35d1_adc_probe_dt(priv, pdev);
	if (ret) {
		dev_err(&pdev->dev, "No platform data available.\n");
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		return ret;
	}

	clk_set_rate(priv->clk, priv->clk_rate);
	// Set to high speed mode if clock rate is higher than 3.2MHz
	if (priv->clk_rate > 3200000)
		__raw_writel(ADC_CONF_HSPEED, priv->base + REG_ADC_CONF);
	else
		__raw_writel(0, priv->base + REG_ADC_CONF);

	__raw_writel(ADC_CTL_ADEN, priv->base + REG_ADC_CTL);

	__raw_writel(0, priv->base + REG_ADC_IER);

	ret = devm_request_irq(&pdev->dev, priv->irq, ma35d1_adc_interrupt,
				0, dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "Failed to request irq\n");
		return ret;
	}

	spin_lock_init(&priv->lock);
	init_completion(&priv->completion);

	if (priv->enable_ts) {
		ret = ma35d1_ts_register(pdev);
		if (ret)
			goto error_disable_clk;
	}

	if (priv->enable_iio) {
		ret = ma35d1_iio_register(pdev);
		if (ret)
			goto error_disable_ts;
	}

	return 0;

error_disable_ts:
	if (priv->enable_ts)
		ma35d1_ts_unregister(pdev);

error_disable_clk:
	clk_disable_unprepare(priv->clk);

	return ret;
}

static int ma35d1_adc_remove(struct platform_device *pdev)
{
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);

	// Power off analog macro
	__raw_writel(0, priv->base + REG_ADC_CTL);

	if (priv->enable_iio)
		ma35d1_iio_unregister(pdev);
	if (priv->enable_ts)
		ma35d1_ts_unregister(pdev);

	clk_disable_unprepare(priv->clk);
	free_irq(priv->irq, pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ma35d1_adc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);

	if (priv->enable_ts_wk == 1 && priv->ts_state != TS_STOP) {
		detect_wakeup(priv);
		enable_irq_wake(priv->irq);
	}

	return 0;
}

static int ma35d1_adc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ma35d1_adc *priv = platform_get_drvdata(pdev);

	if (priv->enable_ts_wk == 1 && priv->ts_state != TS_STOP) {
		disable_irq_wake(priv->irq);
		detect_pendown(priv);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ma35d1_adc_pm_ops, ma35d1_adc_suspend, ma35d1_adc_resume);

static const struct of_device_id ma35d1_adc_match[] = {
	{ .compatible = "nuvoton,ma35d1-adc" },
	{},
};

static struct platform_driver ma35d1_adc_driver = {
	.probe = ma35d1_adc_probe,
	.remove = ma35d1_adc_remove,
	.driver = {
		.name = "ma35d1-adc",
		.of_match_table = ma35d1_adc_match,
		.pm = &ma35d1_adc_pm_ops,
	},
};

module_platform_driver(ma35d1_adc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nuvoton MA35D1 ADC Driver");
