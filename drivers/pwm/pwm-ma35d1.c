// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 Nuvoton Technology Corp.
 *
 * MA35D1 Series EPWM driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */


#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/pwm.h>


/* PWM Registers */

#define REG_PWM_CTL0            (0x00)
#define REG_PWM_CTL1            (0x04)
#define REG_PWM_CLKPSC01        (0x14)
#define REG_PWM_CLKPSC23        (0x18)
#define REG_PWM_CLKPSC45        (0x1C)
#define REG_PWM_CNTEN           (0x20)
#define REG_PWM_PERIOD0		(0x30)
#define REG_PWM_PERIOD1		(0x34)
#define REG_PWM_PERIOD2		(0x38)
#define REG_PWM_PERIOD3		(0x3C)
#define REG_PWM_PERIOD4		(0x40)
#define REG_PWM_PERIOD5		(0x44)
#define REG_PWM_CMPDAT0		(0x50)
#define REG_PWM_CMPDAT1		(0x54)
#define REG_PWM_CMPDAT2		(0x58)
#define REG_PWM_CMPDAT3		(0x5C)
#define REG_PWM_CMPDAT4		(0x60)
#define REG_PWM_CMPDAT5		(0x64)
#define REG_PWM_CNT0		(0x90)
#define REG_PWM_CNT1		(0x94)
#define REG_PWM_CNT2		(0x98)
#define REG_PWM_CNT3		(0x9C)
#define REG_PWM_WGCTL0		(0xB0)
#define REG_PWM_WGCTL1		(0xB4)
#define REG_PWM_POLCTL		(0xD4)
#define REG_PWM_POEN		(0xD8)
#define REG_PWM_CAPINEN		(0x200)
#define REG_PWM_CAPCTL		(0x204)
#define REG_PWM_CAPSTS		(0x208)
#define REG_PWM_RCAPDAT0	(0x20C)
#define REG_PWM_RCAPDAT1	(0x214)
#define REG_PWM_RCAPDAT2	(0x21C)
#define REG_PWM_RCAPDAT3	(0x224)
#define REG_PWM_FCAPDAT0	(0x210)
#define REG_PWM_FCAPDAT1	(0x218)
#define REG_PWM_FCAPDAT2	(0x220)
#define REG_PWM_FCAPDAT3	(0x228)
#define REG_PWM_CAPIEN		(0x250)
#define REG_PWM_CAPIF		(0x254)

#define WGCTL_MASK		0x3
#define WGCTL_HIGH		0x2
#define WGCTL_LOW		0x1

#define CAP_CLK_PSC		9

/*
 * Each capture input can be programmed to detect rising-edge, falling-edge,
 * either edge or neither egde.
 */
enum ma35d1_cpt_edge {
	CPT_EDGE_DISABLED,
	CPT_EDGE_RISING,
	CPT_EDGE_FALLING,
	CPT_EDGE_BOTH,
};

struct ma35d1_cpt_ddata {
	u32 snapshot[3];
	unsigned int index;
	struct mutex lock;
	wait_queue_head_t wait;
};

struct ma35d1_pwm_compat_data {
	unsigned int pwm_num_devs;
	unsigned int cpt_num_devs;
	unsigned int max_pwm_cnt;
	unsigned int max_prescale;
};

struct ma35d1_chip {
	struct platform_device	*pdev;
	struct clk		*clk;
	struct pwm_chip		chip;
	void __iomem		*regs;
	spinlock_t		lock;
};

#define to_ma35d1_chip(chip)	container_of(chip, struct ma35d1_chip, chip)

static void CalPeriodTime(struct pwm_chip *chip, uint32_t u32Ch, struct pwm_capture *result)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	uint16_t u32Count[4];
	uint32_t u32i;
	uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
	unsigned int effective_ticks;

	/* Clear Capture Falling Indicator (Time A) */
	__raw_writel(0x100 << u32Ch, ma35d1->regs + REG_PWM_CAPIF);

	/* Wait for Capture Falling Indicator  */
	while ((__raw_readl(ma35d1->regs + REG_PWM_CAPIF) & (0x100 << u32Ch)) == 0);

	/* Clear Capture Falling Indicator (Time B)*/
	__raw_writel(0x100 << u32Ch, ma35d1->regs + REG_PWM_CAPIF);

	u32i = 0;

	while(u32i < 4) {
		/* Wait for Capture Falling Indicator */
		while ((__raw_readl(ma35d1->regs + REG_PWM_CAPIF) & (0x101 << u32Ch)) != (0x101 << u32Ch));

		/* Clear Capture Falling and Rising Indicator */
		__raw_writel(0x101 << u32Ch, ma35d1->regs + REG_PWM_CAPIF);

		/* Get Capture Falling Latch Counter Data */
		u32Count[u32i++] = __raw_readl(ma35d1->regs + REG_PWM_FCAPDAT0 + (u32Ch * 8));

		/* Wait for Capture Rising Indicator */
		//while ((__raw_readl(ma35d1->regs + REG_PWM_CAPIF) & (0x101 << u32Ch)) != (0x101 << u32Ch));
		while ((__raw_readl(ma35d1->regs + REG_PWM_CAPIF) & (0x1 << u32Ch)) != (0x1 << u32Ch));

		/* Clear Capture Rising Indicator */
		__raw_writel(0x1 << u32Ch, ma35d1->regs + REG_PWM_CAPIF);

		/* Get Capture Rising Latch Counter Data */
		u32Count[u32i++] = __raw_readl(ma35d1->regs + REG_PWM_RCAPDAT0 + (u32Ch * 8));
	}

	//printk("Count[0] = %d, Count[1] = %d, Count[2] = %d, Count[3] = %d\n", u32Count[0], u32Count[1], u32Count[2], u32Count[3]);

	u16RisingTime = u32Count[1];

	u16FallingTime = u32Count[0];

	u16HighPeriod = u32Count[1] - u32Count[2];

	u16LowPeriod = 0x10000 - u32Count[1];

	u16TotalPeriod = 0x10000 - u32Count[2];

	//printk("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
	//      u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

	effective_ticks = clk_get_rate(ma35d1->clk);
	result->period = u16TotalPeriod * NSEC_PER_SEC / effective_ticks * (CAP_CLK_PSC + 1);
	result->duty_cycle = u16HighPeriod * NSEC_PER_SEC / effective_ticks * (CAP_CLK_PSC + 1);

}

static int ma35d1_pwm_capture(struct pwm_chip *chip, struct pwm_device *pwm,
                              struct pwm_capture *result, unsigned long timeout)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	int ch = (pwm->hwpwm + chip->base) % ma35d1->chip.npwm;
	unsigned long flags;

	struct ma35d1_cpt_ddata *ddata = pwm_get_chip_data(pwm);
	struct device *dev = ma35d1->chip.dev;

	int ret = 0;

	if (pwm->hwpwm >= 6 /*cdata->cpt_num_devs*/) {
		dev_err(dev, "device %u is not valid\n", pwm->hwpwm);
		return -EINVAL;
	}

	spin_lock_irqsave(&ma35d1->lock, flags);
	ddata->index = 0;

	/* Set PSC */
	if (ch < 2)
		__raw_writel(CAP_CLK_PSC, ma35d1->regs + REG_PWM_CLKPSC01);
	else if (ch < 4)
		__raw_writel(CAP_CLK_PSC, ma35d1->regs + REG_PWM_CLKPSC23);
	else if (ch < 6)
		__raw_writel(CAP_CLK_PSC, ma35d1->regs + REG_PWM_CLKPSC45);

	/* Set to down count type */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CTL1) & ~(3 << (ch << 1)))
	             | (1 << (ch << 1)), ma35d1->regs + REG_PWM_CTL1);

	/* Set period */
	__raw_writel(0xFFFF, ma35d1->regs + REG_PWM_PERIOD0 + (ch * 4));

	/* Enable CNTEN */
	__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_CNTEN)
	             | (1 << ch), ma35d1->regs + REG_PWM_CNTEN);

	/* Enable capture input enable */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CAPINEN) | (0x1 << ch)),
	             ma35d1->regs + REG_PWM_CAPINEN);
	/* Enable capture function */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CAPCTL) | (0x1 << ch)),
	             ma35d1->regs + REG_PWM_CAPCTL);
	/* Enable falling capture reload */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CAPCTL) | (0x1000000 << ch)),
	             ma35d1->regs + REG_PWM_CAPCTL);

	/* Wait until capture channel start to count */
	while (__raw_readl(ma35d1->regs + REG_PWM_CNT0 + (ch *4)) == 0);

	CalPeriodTime(chip, ch, result);

	/* Disable capture input enable */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CAPINEN) & ~(0x1 << ch)),
	             ma35d1->regs + REG_PWM_CAPINEN);
	/* Disable falling capture reload and capture function enable */
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CAPCTL) & ~(0x1000001 << ch)),
	             ma35d1->regs + REG_PWM_CAPCTL);

	spin_unlock_irqrestore(&ma35d1->lock, flags);

	return ret;
}

static int ma35d1_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	int ch = (pwm->hwpwm + chip->base) % ma35d1->chip.npwm;
	unsigned long flags;

	spin_lock_irqsave(&ma35d1->lock, flags);

	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_WGCTL0) & ~(WGCTL_MASK << (ch*2)))
	             | (WGCTL_HIGH << (ch*2)), ma35d1->regs + REG_PWM_WGCTL0);
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_WGCTL1) & ~(WGCTL_MASK << (ch*2)))
	             | (WGCTL_LOW << (ch*2)), ma35d1->regs + REG_PWM_WGCTL1);
	__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_POEN)
	             | (1 << ch), ma35d1->regs + REG_PWM_POEN);
	__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_CNTEN)
	             | (1 << ch), ma35d1->regs + REG_PWM_CNTEN);

	spin_unlock_irqrestore(&ma35d1->lock, flags);

	return 0;
}

static void ma35d1_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	int ch = (pwm->hwpwm + chip->base) % ma35d1->chip.npwm;
	unsigned long flags;

	spin_lock_irqsave(&ma35d1->lock, flags);

	__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_POEN)
	             & ~(1 << ch), ma35d1->regs + REG_PWM_POEN);
	__raw_writel((__raw_readl(ma35d1->regs + REG_PWM_CNTEN)
	              & ~(1 << ch)), ma35d1->regs + REG_PWM_CNTEN);

	spin_unlock_irqrestore(&ma35d1->lock, flags);
}

static int ma35d1_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
                                   enum pwm_polarity polarity)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	int ch = (pwm->hwpwm + chip->base) % ma35d1->chip.npwm;
	unsigned long flags;

	spin_lock_irqsave(&ma35d1->lock, flags);

	if (polarity == PWM_POLARITY_NORMAL)
		__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_POLCTL)
		             & ~(1 << ch), ma35d1->regs + REG_PWM_POLCTL);
	else
		__raw_writel(__raw_readl(ma35d1->regs + REG_PWM_POLCTL)
		             | (1 << ch), ma35d1->regs + REG_PWM_POLCTL);

	spin_unlock_irqrestore(&ma35d1->lock, flags);

	return 0;
}

static int ma35d1_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                             int duty_ns, int period_ns)
{
	struct ma35d1_chip *ma35d1 = to_ma35d1_chip(chip);
	unsigned long period, duty, prescale;
	unsigned long flags;
	int ch = (pwm->hwpwm + chip->base) % ma35d1->chip.npwm;

	/* Get PCLK, calculate valid parameter range. */
	prescale = clk_get_rate(ma35d1->clk) / 1000000 - 1;

	/* now pwm time unit is 1000ns. */
	period = (period_ns + 500) / 1000;
	duty = (duty_ns + 500) / 1000;

	/* don't want the minus 1 below change the value to -1 (0xFFFF) */
	if (period == 0)
		period = 1;
	if (duty == 0)
		duty = 1;

	spin_lock_irqsave(&ma35d1->lock, flags);

	/* Set prescale */
	if (ch < 2)
		__raw_writel(prescale, ma35d1->regs + REG_PWM_CLKPSC01);
	else if (ch < 4)
		__raw_writel(prescale, ma35d1->regs + REG_PWM_CLKPSC23);
	else
		__raw_writel(prescale, ma35d1->regs + REG_PWM_CLKPSC45);

	if (ch == 0) {
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD0);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT0);
	} else if (ch == 1) {
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD1);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT1);
	} else if (ch == 2) {
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD2);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT2);
	} else if (ch == 3) {
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD3);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT3);
	} else if (ch == 4) {
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD4);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT4);
	} else {/* ch 5 */
		__raw_writel(period - 1, ma35d1->regs + REG_PWM_PERIOD5);
		__raw_writel(duty - 1, ma35d1->regs + REG_PWM_CMPDAT5);
	}

	spin_unlock_irqrestore(&ma35d1->lock, flags);

	return 0;
}

static struct pwm_ops ma35d1_pwm_ops = {
	.capture = ma35d1_pwm_capture,
	.enable = ma35d1_pwm_enable,
	.disable = ma35d1_pwm_disable,
	.config = ma35d1_pwm_config,
	.set_polarity = ma35d1_pwm_set_polarity,
	.owner = THIS_MODULE,
};

static int ma35d1_pwm_probe(struct platform_device *pdev)
{
	struct ma35d1_chip *ma35d1;
	int err, ret;
	struct resource *r;
	unsigned int i;

	ma35d1 = devm_kzalloc(&pdev->dev, sizeof(*ma35d1), GFP_KERNEL);
	if (ma35d1 == NULL)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ma35d1->regs = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(ma35d1->regs))
		return PTR_ERR(ma35d1->regs);

	ma35d1->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ma35d1->clk)) {
		dev_err(&pdev->dev, "failed to get epwm clock\n");
		ret = PTR_ERR(ma35d1->clk);
		return -ENOENT;
	}
	err = clk_prepare_enable(ma35d1->clk);
	if (err)
		return -ENOENT;

	platform_set_drvdata(pdev, ma35d1);

	ma35d1->chip.dev = &pdev->dev;
	ma35d1->chip.ops = &ma35d1_pwm_ops;
	ma35d1->chip.base = -1;
	ma35d1->chip.npwm = 6;

	ret = pwmchip_add(&ma35d1->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register pwm\n");
		goto err;
	}

	for (i = 0; i < 6 /*cdata->cpt_num_devs*/; i++) {
		struct ma35d1_cpt_ddata *ddata;

		ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
		if (!ddata)
			return -ENOMEM;

		init_waitqueue_head(&ddata->wait);
		mutex_init(&ddata->lock);

		pwm_set_chip_data(&ma35d1->chip.pwms[i], ddata);
	}

	return 0;

err:
	clk_disable(ma35d1->clk);
	return ret;
}

static int ma35d1_pwm_remove(struct platform_device *pdev)
{
	struct ma35d1_chip *ma35d1 = platform_get_drvdata(pdev);

	clk_disable(ma35d1->clk);

	return pwmchip_remove(&ma35d1->chip);
}

#if defined(CONFIG_OF)
static const struct of_device_id ma35d1_epwm_of_match[] = {
	{   .compatible = "nuvoton,ma35d1-epwm" },
	{	},
};
MODULE_DEVICE_TABLE(of, ma35d1_epwm_of_match);
#endif

static struct platform_driver ma35d1_epwm_driver = {
	.driver		= {
		.name	= "ma35d1-epwm",
		.owner	= THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(ma35d1_epwm_of_match),
#endif
	},
	.probe		= ma35d1_pwm_probe,
	.remove		= ma35d1_pwm_remove,
};



module_platform_driver(ma35d1_epwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_ALIAS("platform:ma35d1-epwm");

