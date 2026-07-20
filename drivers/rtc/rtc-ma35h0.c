// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/bcd.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>

/* RTC Control Registers */
#define REG_RTC_INIT		0x00

#define REG_RTC_SINFASTS	0x04
#define REG_RTC_FREQADJ		0x08
#define REG_RTC_TIME		0x0c
#define REG_RTC_CAL		0x10
#define REG_RTC_CLKFMT		0x14
#define REG_RTC_WEEKDAY		0x18
#define REG_RTC_TALM		0x1c
#define REG_RTC_CALM		0x20
#define REG_RTC_LEAPYEAR	0x24
#define REG_RTC_INTEN		0x28
#define REG_RTC_INTSTS		0x2c
#define REG_RTC_TICK		0x30
#define REG_RTC_TAMSK		0x34
#define REG_RTC_SPRCTL		0x3c
#define REG_RTC_SPR(x)		(0x40 + ((x) * 4))
#define REG_RTC_CLKDCTL		0x140
#define REG_RTC_CDBR		0x144

#define INIT_ACTIVE		BIT(0)
#define RTC_INIT_MAGIC		0xa5eb1357

#define CLKFMT_24HEN		BIT(0)

/* RTC Time Register Masks */
#define TIME_SEC_MASK		GENMASK(6, 0)
#define TIME_MIN_MASK		GENMASK(14, 8)
#define TIME_HOUR_MASK		GENMASK(21, 16)

/* RTC Calendar Register Masks */
#define CAL_DAY_MASK		GENMASK(5, 0)
#define CAL_MON_MASK		GENMASK(12, 8)
#define CAL_YEAR_MASK		GENMASK(23, 16)

/* RTC Day of the Week Register Mask */
#define WEEKDAY_MASK		GENMASK(2, 0)

/* RTC Time Alarm Register Masks */
#define TALM_SEC_MASK		GENMASK(6, 0)
#define TALM_MIN_MASK		GENMASK(14, 8)
#define TALM_HOUR_MASK		GENMASK(21, 16)

/* RTC Calendar Alarm Register Masks */
#define CALM_DAY_MASK		GENMASK(5, 0)
#define CALM_MON_MASK		GENMASK(12, 8)
#define CALM_YEAR_MASK		GENMASK(23, 16)

#define INTEN_ALMIEN		BIT(0)
#define INTEN_TICKIEN		BIT(1)
#define INTEN_PSWIEN		BIT(2)
#define INTEN_KPRSIEN		BIT(3)
#define INTEN_CLKFIEN		BIT(24)
#define INTEN_CLKSTIEN		BIT(25)

#define INTSTS_ALMIF		BIT(0)
#define INTSTS_TICKIF		BIT(1)
#define INTSTS_PSWST		BIT(2)
#define INTSTS_KPRSST		BIT(3)
#define INTSTS_CLKFIF		BIT(24)
#define INTSTS_CLKSTIF		BIT(25)

#define TAMSK_MSEC		BIT(0)
#define TAMSK_MTENSEC		BIT(1)
#define TAMSK_MMIN		BIT(2)
#define TAMSK_MTENMIN		BIT(3)
#define TAMSK_MHR		BIT(4)
#define TAMSK_MTENHR		BIT(5)

#define CLKDCLT_LXTFDEN		BIT(0)

#define CDBR_FAILBD_MASK	GENMASK(23, 16)
#define CDBR_STOPBD_MASK	GENMASK(7, 0)

#define LXT_DET_FAILBD_DEFAULT	0x80
#define LXT_DET_STOPBD_DEFAULT	0x80

#define TIME_SEC_SHIFT		0
#define TIME_MIN_SHIFT		8
#define TIME_HOUR_SHIFT		16
#define CAL_DAY_SHIFT		0
#define CAL_MON_SHIFT		8
#define CAL_YEAR_SHIFT		16

#define TICK_1_SEC		0x0
#define TICK_1_2_SEC		0x1
#define TICK_1_4_SEC		0x2
#define TICK_1_8_SEC		0x3

#define SPRCTL_SPRRWEN		BIT(2)

struct ma35h0_rtc {
	int irq;
	void __iomem *base;
	struct rtc_device *rtc_dev;
	bool lxt_failed;
	bool lxt_detect_enabled;
};

struct ma35h0_bcd_time {
	int bcd_sec;
	int bcd_min;
	int bcd_hour;
	int bcd_mday;
	int bcd_mon;
	int bcd_year;
};

static u32 rtc_reg_read(struct ma35h0_rtc *rtc, int offset)
{
	return __raw_readl(rtc->base + offset);
}

static void rtc_reg_write(struct ma35h0_rtc *rtc, int offset, int value)
{
	__raw_writel(value, rtc->base + offset);
}

static irqreturn_t ma35h0_rtc_interrupt(int irq, void *_rtc)
{
	struct ma35h0_rtc *rtc = _rtc;
	u32 intsts;
	u32 events = 0;

	intsts = rtc_reg_read(rtc, REG_RTC_INTSTS);

	if (intsts & INTSTS_PSWST) {
		rtc_reg_write(rtc, REG_RTC_INTSTS, INTSTS_PSWST);

		rtc_reg_write(rtc, REG_RTC_INTEN,
			      rtc_reg_read(rtc, REG_RTC_INTEN) & ~INTEN_TICKIEN);
	}

	if (intsts & INTSTS_ALMIF) {
		rtc_reg_write(rtc, REG_RTC_INTSTS, INTSTS_ALMIF);
		events |= RTC_AF | RTC_IRQF;
	}

	if (intsts & INTSTS_TICKIF) {
		rtc_reg_write(rtc, REG_RTC_INTSTS, INTSTS_TICKIF);

		if (!rtc->lxt_failed)
			events |= RTC_UF | RTC_IRQF;
	}

	if (intsts & (INTSTS_CLKFIF | INTSTS_CLKSTIF)) {
		rtc->lxt_failed = true;

		rtc_reg_write(rtc, REG_RTC_INTEN, rtc_reg_read(rtc, REG_RTC_INTEN) &
			      ~(INTEN_CLKFIEN | INTEN_CLKSTIEN));

		rtc_reg_write(rtc, REG_RTC_INTSTS, (INTSTS_CLKFIF | INTSTS_CLKSTIF));
	}

	if (events)
		rtc_update_irq(rtc->rtc_dev, 1, events);

	return IRQ_HANDLED;
}

static int check_rtc_access_enable(struct ma35h0_rtc *rtc)
{
	if (!(rtc_reg_read(rtc, REG_RTC_INIT) & INIT_ACTIVE)) {
		rtc_reg_write(rtc, REG_RTC_INIT, RTC_INIT_MAGIC);
		mdelay(1);
		if (!(rtc_reg_read(rtc, REG_RTC_INIT) & INIT_ACTIVE)) {
			dev_err(rtc->rtc_dev->dev.parent, "RTC access is not enabled\n");
			return -EIO;
		}
	}
	return 0;
}

static int ma35h0_rtc_bcd2bin(u32 timereg, u32 calreg, u32 wdayreg, struct rtc_time *tm)
{
	tm->tm_mday = bcd2bin(FIELD_GET(CAL_DAY_MASK, calreg));

	/* RTC reports 1-12, struct rtc_time expects 0-11 */
	tm->tm_mon  = bcd2bin(FIELD_GET(CAL_MON_MASK, calreg)) - 1;

	/* RTC reports 0-99, struct rtc_time expects years since 1900 (e.g. 2000 = 100) */
	tm->tm_year = bcd2bin(FIELD_GET(CAL_YEAR_MASK, calreg)) + 100;

	tm->tm_sec  = bcd2bin(FIELD_GET(TIME_SEC_MASK, timereg));
	tm->tm_min  = bcd2bin(FIELD_GET(TIME_MIN_MASK, timereg));
	tm->tm_hour = bcd2bin(FIELD_GET(TIME_HOUR_MASK, timereg));

	tm->tm_wday = FIELD_GET(WEEKDAY_MASK, wdayreg);

	return rtc_valid_tm(tm);
}

static int ma35h0_rtc_alarm_bcd2bin(u32 timereg, u32 calreg, struct rtc_time *tm)
{
	tm->tm_mday = bcd2bin(FIELD_GET(CALM_DAY_MASK, calreg));
	tm->tm_mon  = bcd2bin(FIELD_GET(CALM_MON_MASK, calreg)) - 1;
	tm->tm_year = bcd2bin(FIELD_GET(CALM_YEAR_MASK, calreg)) + 100;

	tm->tm_sec  = bcd2bin(FIELD_GET(TALM_SEC_MASK, timereg));
	tm->tm_min  = bcd2bin(FIELD_GET(TALM_MIN_MASK, timereg));
	tm->tm_hour = bcd2bin(FIELD_GET(TALM_HOUR_MASK, timereg));

	return rtc_valid_tm(tm);
}

static void ma35h0_rtc_bin2bcd(struct device *dev, struct rtc_time *settm,
			       struct ma35h0_bcd_time *gettm)
{
	gettm->bcd_mday = FIELD_PREP(CAL_DAY_MASK, bin2bcd(settm->tm_mday));
	gettm->bcd_mon  = FIELD_PREP(CAL_MON_MASK, bin2bcd(settm->tm_mon + 1));

	if (settm->tm_year < 100) {
		dev_warn(dev, "The year is before 2000, setting raw value\n");
		gettm->bcd_year = FIELD_PREP(CAL_YEAR_MASK, bin2bcd(settm->tm_year));
	} else {
		gettm->bcd_year = FIELD_PREP(CAL_YEAR_MASK, bin2bcd(settm->tm_year - 100));
	}

	/* Time: Sec, Min, Hour */
	gettm->bcd_sec  = FIELD_PREP(TIME_SEC_MASK, bin2bcd(settm->tm_sec));
	gettm->bcd_min  = FIELD_PREP(TIME_MIN_MASK, bin2bcd(settm->tm_min));
	gettm->bcd_hour = FIELD_PREP(TIME_HOUR_MASK, bin2bcd(settm->tm_hour));
}

static int ma35h0_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);

	if (enabled) {
		rtc_reg_write(rtc, REG_RTC_INTEN,
			      (rtc_reg_read(rtc, REG_RTC_INTEN) | INTEN_ALMIEN));
	} else {
		rtc_reg_write(rtc, REG_RTC_INTEN,
			      (rtc_reg_read(rtc, REG_RTC_INTEN) & ~INTEN_ALMIEN));
	}

	return 0;
}

static int ma35h0_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);
	u32 timeval, clrval, wdayval;
	u32 intsts;

	intsts = rtc_reg_read(rtc, REG_RTC_INTSTS);

	if (rtc->lxt_failed)
		return -EIO;

	timeval = rtc_reg_read(rtc, REG_RTC_TIME);
	clrval  = rtc_reg_read(rtc, REG_RTC_CAL);
	wdayval = rtc_reg_read(rtc, REG_RTC_WEEKDAY);

	return ma35h0_rtc_bcd2bin(timeval, clrval, wdayval, tm);
}

static int ma35h0_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);
	struct ma35h0_bcd_time gettm;
	unsigned long val;
	int ret;

	if (rtc->lxt_failed)
		return -EIO;

	ma35h0_rtc_bin2bcd(dev, tm, &gettm);

	ret = check_rtc_access_enable(rtc);
	if (ret)
		return ret;

	val = gettm.bcd_mday | gettm.bcd_mon | gettm.bcd_year;
	rtc_reg_write(rtc, REG_RTC_CAL, val);

	val = gettm.bcd_sec | gettm.bcd_min | gettm.bcd_hour;
	rtc_reg_write(rtc, REG_RTC_TIME, val);

	val = tm->tm_wday;
	rtc_reg_write(rtc, REG_RTC_WEEKDAY, val);

	return 0;
}

static int ma35h0_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);
	u32 inten, intsts;
	u32 timeval, carval;

	timeval = rtc_reg_read(rtc, REG_RTC_TALM);
	carval  = rtc_reg_read(rtc, REG_RTC_CALM);

	inten = rtc_reg_read(rtc, REG_RTC_INTEN);
	intsts = rtc_reg_read(rtc, REG_RTC_INTSTS);

	alrm->enabled = !!(inten & INTEN_ALMIEN);
	alrm->pending = !!(intsts & INTSTS_ALMIF);

	return ma35h0_rtc_alarm_bcd2bin(timeval, carval, &alrm->time);
}

static int ma35h0_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);
	struct ma35h0_bcd_time tm;
	u32 cal_val, time_val;
	int ret;

	ret = check_rtc_access_enable(rtc);
	if (ret)
		return ret;

	ma35h0_rtc_bin2bcd(dev, &alrm->time, &tm);

	cal_val = tm.bcd_mday | tm.bcd_mon | tm.bcd_year;
	time_val = tm.bcd_sec | tm.bcd_min | tm.bcd_hour;

	rtc_reg_write(rtc, REG_RTC_INTSTS, INTSTS_ALMIF);
	rtc_reg_write(rtc, REG_RTC_CALM, cal_val);
	rtc_reg_write(rtc, REG_RTC_TALM, time_val);

	/* Disable all alarm masks to enable precise time and date matching */
	rtc_reg_write(rtc, REG_RTC_TAMSK, 0);

	if (alrm->enabled)
		ma35h0_alarm_irq_enable(dev, 1);

	return 0;
}

static int ma35h0_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct ma35h0_rtc *rtc = dev_get_drvdata(dev);
	unsigned int spare_data[16], i;
	int ret;

	ret = check_rtc_access_enable(rtc);
	if (ret)
		return ret;

	switch (cmd) {
	case RTC_GET_SPARE_DATA:

		rtc_reg_write(rtc, REG_RTC_SPRCTL, SPRCTL_SPRRWEN);

		for (i = 0; i < 16; i++)
			spare_data[i] =  rtc_reg_read(rtc, REG_RTC_SPR(i));

		if (copy_to_user((void *)arg, (void *)&spare_data[0], sizeof(spare_data)))
			return -EFAULT;
		break;

	case RTC_SET_SPARE_DATA:

		if (copy_from_user((void *)&spare_data[0], (void *)arg, sizeof(spare_data)))
			return -EFAULT;

		rtc_reg_write(rtc, REG_RTC_SPRCTL, SPRCTL_SPRRWEN);

		for (i = 0; i < 16; i++)
			rtc_reg_write(rtc, REG_RTC_SPR(i), spare_data[i]);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static const struct rtc_class_ops ma35h0_rtc_ops = {
	.read_time = ma35h0_rtc_read_time,
	.set_time = ma35h0_rtc_set_time,
	.read_alarm = ma35h0_rtc_read_alarm,
	.set_alarm = ma35h0_rtc_set_alarm,
	.alarm_irq_enable = ma35h0_alarm_irq_enable,
	.ioctl = ma35h0_ioctl,
};

static int ma35h0_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ma35h0_rtc *rtc;
	struct clk *clk;
	u32 val32;
	int err;

	rtc = devm_kzalloc(&pdev->dev, sizeof(struct ma35h0_rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->base))
		return PTR_ERR(rtc->base);

	clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		return -ENOENT;
	}
	err = clk_prepare_enable(clk);
	if (err)
		return -ENOENT;

	platform_set_drvdata(pdev, rtc);

	rtc_reg_write(rtc, REG_RTC_CLKFMT,
		      (rtc_reg_read(rtc, REG_RTC_CLKFMT) | CLKFMT_24HEN));

	rtc->irq = platform_get_irq(pdev, 0);

	if (devm_request_irq(&pdev->dev, rtc->irq, ma35h0_rtc_interrupt, 0,
			     "ma35h0rtc", rtc)) {
		dev_err(&pdev->dev, "ma35h0 RTC request irq failed\n");
		return -EBUSY;
	}

	rtc_reg_write(rtc, REG_RTC_TICK, TICK_1_SEC);

	if (of_property_read_u32_array(pdev->dev.of_node,
				       "lxt-detect-enable", &val32, 1) != 0) {
		dev_err(&pdev->dev, "can not get lxt-detect-enable flag !\n");
		return -EINVAL;
	}

	rtc->lxt_failed = false;

	if (val32 == 1) {
		u32 cdbr_val;

		rtc->lxt_detect_enabled = true;

		cdbr_val = FIELD_PREP(CDBR_FAILBD_MASK, LXT_DET_FAILBD_DEFAULT) |
			   FIELD_PREP(CDBR_STOPBD_MASK, LXT_DET_STOPBD_DEFAULT);
		rtc_reg_write(rtc, REG_RTC_CDBR, cdbr_val);

		rtc_reg_write(rtc, REG_RTC_CLKDCTL,
			      (rtc_reg_read(rtc, REG_RTC_CLKDCTL) | CLKDCLT_LXTFDEN));

		rtc_reg_write(rtc, REG_RTC_INTEN,
			      (rtc_reg_read(rtc, REG_RTC_INTEN) |
			      INTEN_TICKIEN | INTEN_CLKSTIEN | INTEN_CLKFIEN | INTEN_PSWIEN));
	} else {
		rtc->lxt_detect_enabled = false;

		rtc_reg_write(rtc, REG_RTC_CDBR, 0x0);

		rtc_reg_write(rtc, REG_RTC_CLKDCTL,
			      (rtc_reg_read(rtc, REG_RTC_CLKDCTL) & ~CLKDCLT_LXTFDEN));

		rtc_reg_write(rtc, REG_RTC_INTEN,
			      (rtc_reg_read(rtc, REG_RTC_INTEN) |
			      INTEN_TICKIEN | INTEN_PSWIEN));
	}

	device_init_wakeup(&pdev->dev, true);

	rtc->rtc_dev = devm_rtc_device_register(&pdev->dev, pdev->name,
						      &ma35h0_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "rtc device register failed\n");
		return PTR_ERR(rtc->rtc_dev);
	}

	return 0;
}

static int __exit ma35h0_rtc_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ma35h0_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ma35h0_rtc *rtc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(rtc->irq);

	rtc_reg_write(rtc, REG_RTC_INTEN,
		      (rtc_reg_read(rtc, REG_RTC_INTEN) & ~INTEN_TICKIEN));

	return 0;
}

static int ma35h0_rtc_resume(struct platform_device *pdev)
{
	struct ma35h0_rtc *rtc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(rtc->irq);

	rtc_reg_write(rtc, REG_RTC_INTEN,
		      (rtc_reg_read(rtc, REG_RTC_INTEN) | INTEN_TICKIEN));

	return 0;
}

static const struct of_device_id ma35h0_rtc_of_match[] = {
	{ .compatible = "nuvoton,ma35h0-rtc"},
	{},
};
MODULE_DEVICE_TABLE(of, ma35h0_rtc_of_match);


static struct platform_driver ma35h0_rtc_driver = {
	.remove     = __exit_p(ma35h0_rtc_remove),
	.suspend    = ma35h0_rtc_suspend,
	.resume     = ma35h0_rtc_resume,
	.probe      = ma35h0_rtc_probe,
	.driver		= {
		.name	= "ma35h0-rtc",
		.owner	= THIS_MODULE,
		.of_match_table = ma35h0_rtc_of_match,
	},
};

module_platform_driver(ma35h0_rtc_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("ma35h0 RTC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35h0-rtc");
