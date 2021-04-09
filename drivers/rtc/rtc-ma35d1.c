/*
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>


/* RTC Control Registers */
#define REG_RTC_INIR    0x00
#define REG_RTC_AER	    0x04
#define REG_RTC_FCR	    0x08
#define REG_RTC_TLR	    0x0C
#define REG_RTC_CLR	    0x10
#define REG_RTC_TSSR    0x14
#define REG_RTC_DWR	    0x18
#define REG_RTC_TAR	    0x1C
#define REG_RTC_CAR	    0x20
#define REG_RTC_LIR	    0x24
#define REG_RTC_RIER    0x28
#define REG_RTC_RIIR    0x2C
#define REG_RTC_TTR	    0x30
#define REG_RTC_PWRCTL  0x34


#define RTCSET          0x01
#define AERRWENB        0x10000
#define INIRRESET       0xa5eb1357
#define AERPOWERON      0xA965
#define AERPOWEROFF     0x0000
#define LEAPYEAR        0x0001
#define TICKENB         0x80
#define TICKINTENB      0x0002
#define ALARMINTENB     0x0001
#define MODE24          0x0001

struct clk *rtc_clk;

struct ma35d1_rtc {
	int irq_num;
	void __iomem *rtc_reg;
	struct rtc_device *rtcdev;
};

struct ma35d1_bcd_time {
	int bcd_sec;
	int bcd_min;
	int bcd_hour;
	int bcd_mday;
	int bcd_mon;
	int bcd_year;
};

static inline unsigned int rtc_reg_read(struct ma35d1_rtc *p, int offset)
{
	return(__raw_readl(p->rtc_reg + offset));
}

static inline void rtc_reg_write(struct ma35d1_rtc *p, int offset, int value)
{
	unsigned int writetimeout = 0x400;

	__raw_writel(value, p->rtc_reg + offset);

	// wait rtc register write finish
	while((__raw_readl(p->rtc_reg + REG_RTC_RIIR) & (1 << 31)) && writetimeout--)
		udelay(1);
}

static irqreturn_t ma35d1_rtc_interrupt(int irq, void *_rtc)
{
	struct ma35d1_rtc *rtc = _rtc;
	unsigned long events = 0, rtc_irq;

	rtc_irq = __raw_readl(rtc->rtc_reg + REG_RTC_RIIR);

	if (rtc_irq & ALARMINTENB) {
		rtc_reg_write(rtc, REG_RTC_RIIR, ALARMINTENB);
		events |= RTC_AF | RTC_IRQF;
	}

	if (rtc_irq & TICKINTENB) {
		rtc_reg_write(rtc, REG_RTC_RIIR, TICKINTENB);
		events |= RTC_UF | RTC_IRQF;
	}

	rtc_update_irq(rtc->rtcdev, 1, events);

	return IRQ_HANDLED;
}

static int *check_rtc_access_enable(struct ma35d1_rtc *ma35d1_rtc)
{
	rtc_reg_write(ma35d1_rtc, REG_RTC_INIR, INIRRESET);

	mdelay(1);

	rtc_reg_write(ma35d1_rtc, REG_RTC_AER, AERPOWERON);

	mdelay(1);

	return 0;
}

static int ma35d1_rtc_bcd2bin(unsigned int timereg,
				unsigned int calreg, unsigned int wdayreg, struct rtc_time *tm)
{
	tm->tm_mday	= bcd2bin(calreg >> 0);
	tm->tm_mon	= bcd2bin(calreg >> 8);
	tm->tm_mon	= tm->tm_mon - 1;

	tm->tm_year	= bcd2bin(calreg >> 16) + 100;

	tm->tm_sec	= bcd2bin(timereg >> 0);
	tm->tm_min	= bcd2bin(timereg >> 8);
	tm->tm_hour	= bcd2bin(timereg >> 16);

	tm->tm_wday = wdayreg;

	return rtc_valid_tm(tm);
}

static int ma35d1_rtc_alarm_bcd2bin(unsigned int timereg,
				unsigned int calreg, struct rtc_time *tm)
{
	tm->tm_mday	= bcd2bin(calreg >> 0);
	tm->tm_mon	= bcd2bin(calreg >> 8);
	tm->tm_mon	= tm->tm_mon - 1;

	tm->tm_year	= bcd2bin(calreg >> 16) + 100;

	tm->tm_sec	= bcd2bin(timereg >> 0);
	tm->tm_min	= bcd2bin(timereg >> 8);
	tm->tm_hour	= bcd2bin(timereg >> 16);

	return rtc_valid_tm(tm);
}

static void ma35d1_rtc_bin2bcd(struct device *dev, struct rtc_time *settm,
						struct ma35d1_bcd_time *gettm)
{
	gettm->bcd_mday = bin2bcd(settm->tm_mday) << 0;
	gettm->bcd_mon  = bin2bcd((settm->tm_mon + 1)) << 8;

	if (settm->tm_year < 100) {
		dev_warn(dev, "The year will be between 1970-1999, right?\n");
		gettm->bcd_year = bin2bcd(settm->tm_year) << 16;
	} else {
		gettm->bcd_year = bin2bcd(settm->tm_year - 100) << 16;
	}

	gettm->bcd_sec  = bin2bcd(settm->tm_sec) << 0;
	gettm->bcd_min  = bin2bcd(settm->tm_min) << 8;
	gettm->bcd_hour = bin2bcd(settm->tm_hour) << 16;
}

static int ma35d1_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);

	if (enabled)
		rtc_reg_write(rtc, REG_RTC_RIER, (__raw_readl(rtc->rtc_reg + REG_RTC_RIER)|(ALARMINTENB)));
	else
		rtc_reg_write(rtc, REG_RTC_RIER, (__raw_readl(rtc->rtc_reg + REG_RTC_RIER)&(~ALARMINTENB)));
	return 0;
}

static int ma35d1_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);
	unsigned int timeval, clrval, wdayval;

	timeval = __raw_readl(rtc->rtc_reg + REG_RTC_TLR);
	clrval  = __raw_readl(rtc->rtc_reg + REG_RTC_CLR);
	wdayval = __raw_readl(rtc->rtc_reg + REG_RTC_DWR);

	return ma35d1_rtc_bcd2bin(timeval, clrval, wdayval, tm);
}

static int ma35d1_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);
	struct ma35d1_bcd_time gettm;
	unsigned long val;
	int *err;

	ma35d1_rtc_bin2bcd(dev, tm, &gettm);

	err = check_rtc_access_enable(rtc);
	if (IS_ERR(err))
		return PTR_ERR(err);

	val = gettm.bcd_mday | gettm.bcd_mon | gettm.bcd_year;
	rtc_reg_write(rtc, REG_RTC_CLR, val);

	val = gettm.bcd_sec | gettm.bcd_min | gettm.bcd_hour;
	rtc_reg_write(rtc, REG_RTC_TLR, val);

	val = tm->tm_wday;
	rtc_reg_write(rtc, REG_RTC_DWR, val);

	return 0;
}

static int ma35d1_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);
	unsigned int timeval, carval;

	timeval = __raw_readl(rtc->rtc_reg + REG_RTC_TAR);
	carval  = __raw_readl(rtc->rtc_reg + REG_RTC_CAR);

	return ma35d1_rtc_alarm_bcd2bin(timeval, carval, &alrm->time);
}

static int ma35d1_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);
	struct ma35d1_bcd_time tm;
	unsigned long val;
	int *err;

	ma35d1_rtc_bin2bcd(dev, &alrm->time, &tm);

	err = check_rtc_access_enable(rtc);
	if (IS_ERR(err))
		return PTR_ERR(err);

	val = tm.bcd_mday | tm.bcd_mon | tm.bcd_year;
	val |= (1 << 31); // mask alarm week day
	rtc_reg_write(rtc, REG_RTC_CAR, val);

	val = tm.bcd_sec | tm.bcd_min | tm.bcd_hour;
	rtc_reg_write(rtc, REG_RTC_TAR, val);

	rtc_reg_write(rtc, REG_RTC_PWRCTL, (__raw_readl(rtc->rtc_reg + REG_RTC_PWRCTL) | (1 << 3)));

	return 0;
}

static int ma35d1_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct ma35d1_rtc *rtc = dev_get_drvdata(dev);
	unsigned int spare_data[16], i;
	int *err;

	switch(cmd)
	{
		case RTC_GET_SPARE_DATA:
			err = check_rtc_access_enable(rtc);
			if (IS_ERR(err))
				return PTR_ERR(err);

			for(i = 0; i < 16; i++)
			{
				spare_data[i] =  __raw_readl(rtc->rtc_reg + (0x40+(i*4)));
			}

			if(copy_to_user((void*)arg, (void *)&spare_data[0], sizeof(spare_data)))
			{
				return -EFAULT;
			}

		break;

		case RTC_SET_SPARE_DATA:
			if(copy_from_user((void *)&spare_data[0], (void*)arg, sizeof(spare_data)))
			{
				return -EFAULT;
			}

			err = check_rtc_access_enable(rtc);
			if (IS_ERR(err))
				return PTR_ERR(err);

			for(i = 0; i < 16; i++)
			{
				rtc_reg_write(rtc, (0x40+(i*4)), spare_data[i]);
			}

		break;

		default:
			return -ENOIOCTLCMD;
	}

	return 0;
}

static struct rtc_class_ops ma35d1_rtc_ops = {
	.read_time = ma35d1_rtc_read_time,
	.set_time = ma35d1_rtc_set_time,
	.read_alarm = ma35d1_rtc_read_alarm,
	.set_alarm = ma35d1_rtc_set_alarm,
	.alarm_irq_enable = ma35d1_alarm_irq_enable,
	.ioctl = ma35d1_ioctl,
};

static int ma35d1_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ma35d1_rtc *ma35d1_rtc;
	struct clk *clk;
	int err;

	ma35d1_rtc = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_rtc), GFP_KERNEL);
	if (!ma35d1_rtc) {
		dev_err(&pdev->dev, "kzalloc nuc900_rtc failed\n");
		return -ENOMEM;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ma35d1_rtc->rtc_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ma35d1_rtc->rtc_reg))
		return PTR_ERR(ma35d1_rtc->rtc_reg);

	clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		return -ENOENT;
	}
	err = clk_prepare_enable(clk);
	if (err)
		return -ENOENT;

	platform_set_drvdata(pdev, ma35d1_rtc);

	ma35d1_rtc->rtcdev = devm_rtc_device_register(&pdev->dev, pdev->name, &ma35d1_rtc_ops, THIS_MODULE);
	if (IS_ERR(ma35d1_rtc->rtcdev)) {
		dev_err(&pdev->dev, "rtc device register failed\n");
		return PTR_ERR(ma35d1_rtc->rtcdev);
	}

	rtc_reg_write(ma35d1_rtc, REG_RTC_TSSR, (__raw_readl(ma35d1_rtc->rtc_reg + REG_RTC_TSSR) | MODE24));

	ma35d1_rtc->irq_num = platform_get_irq(pdev, 0);

	if (devm_request_irq(&pdev->dev, ma35d1_rtc->irq_num,
			ma35d1_rtc_interrupt, IRQF_NO_SUSPEND, "ma35d1rtc", ma35d1_rtc)) {
		dev_err(&pdev->dev, "ma35d1 RTC request irq failed\n");
		return -EBUSY;
	}

	rtc_reg_write(ma35d1_rtc, REG_RTC_RIER, (__raw_readl(ma35d1_rtc->rtc_reg + REG_RTC_RIER) | TICKINTENB));

	device_init_wakeup(&pdev->dev, true);

	return 0;
}

static int __exit ma35d1_rtc_remove(struct platform_device *pdev)
{
	device_init_wakeup(&pdev->dev, 0);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int ma35d1_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ma35d1_rtc *ma35d1_rtc = platform_get_drvdata(pdev);
	
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(ma35d1_rtc->irq_num);

	rtc_reg_write(ma35d1_rtc, REG_RTC_RIER, (__raw_readl(ma35d1_rtc->rtc_reg + REG_RTC_RIER) &~ TICKINTENB));

	return 0;
}

static int ma35d1_rtc_resume(struct platform_device *pdev)
{
	struct ma35d1_rtc *ma35d1_rtc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(ma35d1_rtc->irq_num);

	rtc_reg_write(ma35d1_rtc, REG_RTC_RIER, (__raw_readl(ma35d1_rtc->rtc_reg + REG_RTC_RIER) | TICKINTENB));

	return 0;
}

static const struct of_device_id ma35d1_rtc_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-rtc"},
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_rtc_of_match);


static struct platform_driver ma35d1_rtc_driver = {
	.remove     = __exit_p(ma35d1_rtc_remove),
	.suspend    = ma35d1_rtc_suspend,
	.resume     = ma35d1_rtc_resume,
	.probe      = ma35d1_rtc_probe,
	.driver		= {
		.name	= "ma35d1-rtc",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_rtc_of_match),
	},
};

module_platform_driver(ma35d1_rtc_driver);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("ma35d1 RTC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-rtc");
