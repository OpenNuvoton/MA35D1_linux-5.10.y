// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35D1 TIMER
 *
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/pinctrl/consumer.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <uapi/misc/ma35d1_timer.h>
#include "regs-ma35d1-timer.h"

#define TIMER_CH				12
#define TIMER_OPMODE_NONE			0
#define TIMER_OPMODE_ONESHOT			1
#define TIMER_OPMODE_PERIODIC			2
#define TIMER_OPMODE_CONTINUOUS			3
#define TIMER_OPMODE_TOGGLE			4
#define TIMER_OPMODE_TRIGGER_COUNTING		5
#define TIMER_OPMODE_FREE_COUNTING		6
#define TIMER_OPMODE_EVENT_COUNTING		7

/* Register bit field */
#define TIMER_WK_EN				(0x1 << 23)
#define TIMER_CNT_IEN				(0x1 << 29)
#define TIMER_CNT_EN				(0x1 << 30)
#define TIMER_ONESHOT_MODE			(0x0 << 27)
#define TIMER_PERIODIC_MODE			(0x1 << 27)
#define TIMER_TOGGLE_MODE			(0x2 << 27)
#define TIMER_CONTINUOUS_MODE			(0x3 << 27)
#define TIMER_EVENT_COUNTING_MODE		(0x1 << 24)
#define TIMER_CAPTURE_EN			(0x1 << 3)
#define TIMER_CAPTURE_FREE_COUNTING		(0x0 << 4)
#define TIMER_CAPTURE_COUNTER_RESET		(0x1 << 4)
#define TIMER_CAPTURE_IEN			(0x1 << 5)

#define TIMER_COUNTER_RESET	(TIMER_CAPTURE_COUNTER_RESET | TIMER_CAPTURE_EN)
#define TIMER_FREE_COUNTING	(TIMER_CAPTURE_FREE_COUNTING | TIMER_CAPTURE_EN)

#define TIMER_PERIODIC		(TIMER_PERIODIC_MODE | TIMER_CNT_EN)
#define TIMER_TOGGLE		(TIMER_TOGGLE_MODE | TIMER_CNT_EN)

#define TIMER_EVENT_COUNTER	(TIMER_EVENT_COUNTING_MODE | TIMER_CNT_EN)

struct ma35d1_timer {
	spinlock_t lock;
	struct device *dev;
	struct clk *clk;
	struct clk *eclk;
	struct regmap *regmap;
	void __iomem *base;
	wait_queue_head_t wq;
	int minor;	// dynamic minor num, so we need this to distinguish between channels
	u32 cap;	// latest capture data
	u32 cnt;	// latest timer up-counter value
	int irq;	// interrupt number
	u8 ch;		// timer channel. 0~11
	u8 mode;	// Current OP mode. Counter, free counting, trigger counting...
	u8 occupied;	// device opened
	u8 update;	// new capture data available
	u8 clksel;
	u32 psc;
};


static struct ma35d1_timer *tmr[TIMER_CH];


static u8 gu8_ch;
static u32 gu32_cnt;

static irqreturn_t ma35d1_timer_interrupt(int irq, void *dev_id)
{
	struct ma35d1_timer *t = (struct ma35d1_timer *)dev_id;
	static int cnt = 0;
	static uint32_t t0, t1;
	unsigned long flag = 0;

	spin_lock(&t->lock);
	flag = readl_relaxed(t->base + REG_TIMER_INTSTS);
	if (flag & 0x2) {
		// Clear Timer Wake-up Interrupt Status
		writel_relaxed(0x2, t->base + REG_TIMER_INTSTS);
	}
	if (flag & 0x1) {
		t->cnt = gu32_cnt++;
		// Clear Timer Time-out Interrupt Status
		writel_relaxed(readl_relaxed(t->base + REG_TIMER_INTSTS) & 0x1,
				t->base + REG_TIMER_INTSTS);
		t->update = 1;
	}

	flag = readl_relaxed(t->base + REG_TIMER_EINTSTS);
	if (flag & 0x1) {

		if (t->mode == TIMER_OPMODE_FREE_COUNTING) {
			if (cnt == 0) {
				/* Gets the Timer capture data */
				t0 =  readl_relaxed(t->base + REG_TIMER_CAP);
				cnt++;

			} else if (cnt == 1) {
				/* Gets the Timer capture data */
				t1 =  readl_relaxed(t->base + REG_TIMER_CAP);
				cnt++;

				if (t0 > t1) {
					/* over run, drop this data and do nothing */

				} else {
					/* Display the measured input frequency */
					t->cap =  12000000 / (t1 - t0);
					t->update = 1;

				}
			} else {
				cnt = 0;
			}

		} else {
			t->cap = readl_relaxed(t->base + REG_TIMER_CAP);
			t->update = 1;
		}

		// Clear Timer capture Interrupt Status
		writel_relaxed(readl_relaxed(t->base + REG_TIMER_EINTSTS) & 0x1,
				t->base + REG_TIMER_EINTSTS);
	}

	wake_up_interruptible(&t->wq);
	spin_unlock(&t->lock);

	return IRQ_HANDLED;
}

static void timer_SwitchClkSrc(u8 u8clksel, struct ma35d1_timer *t)
{
	struct clk *clkmux;
	u8 ch;
	u32 val, tmpval;
	ulong tmrFreq;
	int ret;

	ch = t->ch;
	t->clksel = u8clksel;

	if(u8clksel == 1 || u8clksel == 5) {
		// timer clock is lxt 32.768k, set prescaler to 1 - 1.
		t->psc = 0;
		tmrFreq = 32768;
	}
	else if(u8clksel == 5) {
		// timer clock is lirc 32kHz, set prescaler to 1 - 1.
		t->psc = 0;
		tmrFreq = 32000;
	}
	else if(u8clksel == 0) {
		// timer clock is hxt 24MHz, set prescaler to 2 - 1.
		t->psc = (2-1);
		tmrFreq = 24000000;
	}
	else if(u8clksel == 7) {
		// timer clock is hirc 12MHz.
		t->psc = 0;
		tmrFreq = 12000000;
	}
	else {
		t->psc = (10-1);
		tmrFreq = 180000000;
	}

	if (t->ch <= 7) {
		regmap_read(t->regmap, 0x1c, &val);
		pr_debug("   tmr%d before clksel0:0x%08x  >>>\n", ch, val);
		tmpval = (u8clksel << (ch*4));
		regmap_write(t->regmap, 0x1c, tmpval | (val & ~(0x7 << (ch*4))));
		regmap_read(t->regmap, 0x1c, &val);
		pr_debug("  tmr%d after  clksel0: [ 0x%08x ]  >>>\n", ch, val);
	} else {
		regmap_read(t->regmap, 0x20, &val);
		pr_debug("  tmr%d before clksel0:0x%08x  >>>\n", ch, val);
		tmpval = (u8clksel << ((ch%8)*4));
		regmap_write(t->regmap, 0x20, tmpval | (val & ~(0x7 << ((ch%8)*4))));
		regmap_read(t->regmap, 0x20, &val);
		pr_debug("  tmr%d after  clksel0: [ 0x%08x ]  >>>\n", ch, val);
	}

	t->eclk = of_clk_get(t->dev->of_node, 0);
	if (IS_ERR(t->eclk)) {
		ret = PTR_ERR(t->eclk);
		dev_err(t->dev, "failed to get tmr eclk, ret %d\n", ret);
		return;
	}

	clk_set_rate(t->eclk, tmrFreq);

	clkmux = clk_get_parent(t->eclk);
	if (IS_ERR(clkmux)) {
		dev_err(t->dev, "failed to get tmr eclock\n");
		ret = PTR_ERR(clkmux);
		return;
	}

	ret = clk_set_parent(t->eclk, clkmux);
	if (ret < 0) {
		dev_err(t->dev, "failed to set parent %s for %s: %d\n",
			__clk_get_name(clkmux),
			__clk_get_name(t->eclk), ret);
		return;
	}
}

static void stop_timer(struct ma35d1_timer *t)
{
	unsigned long flag;

	pr_debug("L:%d %s\n", __LINE__, __FUNCTION__);

	spin_lock_irqsave(&t->lock, flag);

	// Stop timer
	writel_relaxed((readl_relaxed(t->base + REG_TIMER_CTL) & ~(1<<30)),
			t->base + REG_TIMER_CTL);
	// Disable interrupt
	writel_relaxed((readl_relaxed(t->base + REG_TIMER_CTL) & ~(1<<29)),
			t->base + REG_TIMER_CTL);
	// Clear interrupt flag if any
	writel_relaxed(0x3, t->base + REG_TIMER_INTSTS);
	writel_relaxed(0x1, t->base + REG_TIMER_EINTSTS);

	t->mode = TIMER_OPMODE_NONE;
	t->update = 0;
	spin_unlock_irqrestore(&t->lock, flag);
}

static ssize_t timer_read(struct file *filp, char __user *buf, size_t count,
			loff_t *f_pos)
{
	unsigned long flag;
	struct ma35d1_timer *t = (struct ma35d1_timer *)filp->private_data;
	int ret = 0;

	pr_debug("L:%d %s\n", __LINE__, __FUNCTION__);

	spin_lock_irqsave(&t->lock, flag);
	if (t->mode != TIMER_OPMODE_TRIGGER_COUNTING &&
		t->mode != TIMER_OPMODE_FREE_COUNTING   &&
		t->mode != TIMER_OPMODE_PERIODIC        &&
		t->mode != TIMER_OPMODE_EVENT_COUNTING) {
		ret = -EPERM;

		goto out;
	}

	if (t->update) {

		if (t->mode == TIMER_OPMODE_TRIGGER_COUNTING ||
			t->mode == TIMER_OPMODE_FREE_COUNTING) {

			if (copy_to_user(buf, &t->cap, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;// size of int.
		} else if (t->mode == TIMER_OPMODE_PERIODIC ||
				t->mode == TIMER_OPMODE_EVENT_COUNTING) {
			if (copy_to_user(buf, &t->cnt, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;// size of int.
		}
		t->update = 0;

		goto out;
	} else {
		spin_unlock_irqrestore(&t->lock, flag);
		wait_event_interruptible(t->wq, t->update != 0);
		if (t->mode == TIMER_OPMODE_TRIGGER_COUNTING ||
			t->mode == TIMER_OPMODE_FREE_COUNTING) {
			if (copy_to_user(buf, &t->cap, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;// size of int.
		} else if (t->mode == TIMER_OPMODE_PERIODIC ||
				t->mode == TIMER_OPMODE_EVENT_COUNTING) {
			if (copy_to_user(buf, &t->cnt, sizeof(unsigned int)))
				ret = -EFAULT;
			else
				ret = 4;// size of int.
		}
		t->update = 0;

		return ret;
	}

out:
	spin_unlock_irqrestore(&t->lock, flag);

	return ret;
}


static int timer_release(struct inode *inode, struct file *filp)
{
	struct ma35d1_timer *t = (struct ma35d1_timer *)filp->private_data;
	u8 ch = t->ch;
	unsigned long flag;

	pr_debug("L:%d %s\n", __LINE__, __FUNCTION__);

	stop_timer(t);

	// free irq
	free_irq(tmr[ch]->irq, tmr[ch]);
	// disable clk
	clk_disable_unprepare(tmr[ch]->eclk);
	clk_disable_unprepare(tmr[ch]->clk);

	spin_lock_irqsave(&tmr[ch]->lock, flag);
	tmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&tmr[ch]->lock, flag);
	filp->private_data = NULL;

	return 0;
}

static int timer_open(struct inode *inode, struct file *filp)
{
	int i, ret;
	u8 ch;
	unsigned long flag;
	struct clk *clkmux;
	int minor = iminor(inode);

#if 1
	for (i = 0; i < TIMER_CH; i++) {
		if(tmr[i]->minor == minor) {
			ch = i;
			break;
		}
	}
#else
	ch = 2;
#endif

	spin_lock_irqsave(&tmr[ch]->lock, flag);
	if (tmr[ch]->occupied) {
		spin_unlock_irqrestore(&tmr[ch]->lock, flag);
		pr_debug("-EBUSY error\n");
		return -EBUSY;
	}

	tmr[ch]->occupied = 1;
	spin_unlock_irqrestore(&tmr[ch]->lock, flag);

	if (request_irq(tmr[ch]->irq, ma35d1_timer_interrupt,
			IRQF_NO_SUSPEND, "ma35d1-timer", tmr[ch])) {
		pr_debug("register irq failed %d\n", tmr[ch]->irq);
		ret = -EAGAIN;
		goto out2;
	}
	filp->private_data = tmr[ch];

	clkmux = clk_get_parent(tmr[ch]->eclk);
	if (IS_ERR(clkmux)) {
		dev_err(tmr[ch]->dev, "failed to get tmr eclock\n");
		ret = PTR_ERR(clkmux);
		goto out1;
	}

	ret = clk_set_parent(tmr[ch]->eclk, clkmux);
	if (ret < 0) {
		dev_err(tmr[ch]->dev, "failed to set parent %s for %s: %d\n",
			__clk_get_name(clkmux),
			__clk_get_name(tmr[ch]->eclk), ret);
		goto out1;
	}

	clk_prepare(tmr[ch]->clk);
	clk_enable(tmr[ch]->clk);
	clk_prepare(tmr[ch]->eclk);
	clk_enable(tmr[ch]->eclk);

	return 0;

out1:
	free_irq(tmr[ch]->irq, tmr[ch]);
out2:
	spin_lock_irqsave(&tmr[ch]->lock, flag);
	tmr[ch]->occupied = 0;
	spin_unlock_irqrestore(&tmr[ch]->lock, flag);

	return ret;
}

static long timer_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flag;
	struct ma35d1_timer *t = (struct ma35d1_timer *)filp->private_data;
	unsigned int param;

	// stop timer before we do any change
	stop_timer(t);

	// init time-out counts
	gu32_cnt = 1;
	t->cnt = gu32_cnt;

	// check clock source
	pr_debug("\ttimer_ioctl_clksec: [ %d ]\n", t->clksel);
	timer_SwitchClkSrc(t->clksel, t);

	switch (cmd) {
	case TMR_IOC_CLKLXT:
	case TMR_IOC_CLKHXT:
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;
		// switch clock source
		timer_SwitchClkSrc(param, t);
		break;

	case TMR_IOC_STOP:
		//timer stopped
		break;

	case TMR_IOC_PERIODIC:
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		// compare register is 24-bit width
		if (param > 0xFFFFFF)
			return -EPERM;

		spin_lock_irqsave(&t->lock, flag);

		writel_relaxed(param, t->base + REG_TIMER_CMP);
		// enable timeout interrupt
		writel_relaxed(TIMER_PERIODIC | TIMER_CNT_IEN | t->psc, t->base + REG_TIMER_CTL);
		t->mode = TIMER_OPMODE_PERIODIC;
		spin_unlock_irqrestore(&t->lock, flag);

		break;


	case TMR_IOC_PERIODIC_FOR_WKUP:
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		// compare register is 24-bit width
		if (param > 0xFFFFFF)
			return -EPERM;

		// check clock, power down mode using 32kHz
		if (t->clksel != 0x1) {
			if (t->clksel != 0x5) {
				timer_SwitchClkSrc(1, t);
				pr_warn("Power down mode clock need to switch to 32k.\n");
				return -1;
			}
		}

		gu8_ch = t->ch;
		spin_lock_irqsave(&t->lock, flag);
		writel_relaxed(param, t->base + REG_TIMER_CMP);

		// enable timeout interrupt
		writel_relaxed(TIMER_PERIODIC | TIMER_CNT_IEN | TIMER_WK_EN, t->base + REG_TIMER_CTL);
		t->mode = TIMER_OPMODE_PERIODIC;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_TOGGLE:
		// get output duty in us
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;
		// divide by 2 because a duty cycle is high + low
		param >>= 1;
		// compare register is 24-bit width
		if (param > 0xFFFFFF)
			return -EPERM;

		spin_lock_irqsave(&t->lock, flag);

		writel_relaxed(param, t->base + REG_TIMER_CMP);
		writel_relaxed(TIMER_TOGGLE | t->psc, t->base + REG_TIMER_CTL);

		t->mode = TIMER_OPMODE_TOGGLE;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_EVENT_COUNTING:
		// get capture setting
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		spin_lock_irqsave(&t->lock, flag);
		writel_relaxed(param, t->base + REG_TIMER_CMP);
		writel_relaxed(TIMER_EVENT_COUNTER | TIMER_PERIODIC_MODE |
				TMR_EXTCNT_EDGE_FF | TIMER_CNT_IEN, t->base + REG_TIMER_CTL);

		t->mode = TIMER_OPMODE_EVENT_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_FREE_COUNTING:
		// get capture setting
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		spin_lock_irqsave(&t->lock, flag);
		writel_relaxed(t->psc | param | TIMER_PERIODIC, t->base + REG_TIMER_CTL);
		writel_relaxed(0xFFFFFF, t->base + REG_TIMER_CMP);
		// enable capture interrupt
		writel_relaxed(TIMER_CAPTURE_IEN | TIMER_FREE_COUNTING,
				t->base + REG_TIMER_EXTCTL);

		t->mode = TIMER_OPMODE_FREE_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);

		break;

	case TMR_IOC_TRIGGER_COUNTING:
		// get capture setting
		if (copy_from_user((void *)&param, (const void *)arg, sizeof(unsigned int)))
			return -EFAULT;

		spin_lock_irqsave(&t->lock, flag);
		writel_relaxed(t->psc | param | TIMER_PERIODIC, t->base + REG_TIMER_CTL);
		writel_relaxed(0xFFFFFF, t->base + REG_TIMER_CMP);
		// enable capture interrupt
		writel_relaxed(TIMER_CAPTURE_IEN | TIMER_COUNTER_RESET, t->base + REG_TIMER_EXTCTL);
		t->mode = TIMER_OPMODE_TRIGGER_COUNTING;
		spin_unlock_irqrestore(&t->lock, flag);
		break;

	default:
		return -ENOTTY;


	}
	return 0;
}

static unsigned int timer_poll(struct file *filp, poll_table *wait)
{
	struct ma35d1_timer *t = (struct ma35d1_timer *)filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &t->wq, wait);
	if (t->update)
		mask |= (POLLIN | POLLRDNORM);
	return mask;
}

static const struct file_operations timer_fops = {
	.owner		= THIS_MODULE,
	.open		= timer_open,
	.release	= timer_release,
	.read		= timer_read,
	.unlocked_ioctl	= timer_ioctl,
	.poll		= timer_poll,
};

static struct miscdevice timer_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer0",
		.fops = &timer_fops,
	},
	[1] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer1",
		.fops = &timer_fops,
	},
	[2] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer2",
		.fops = &timer_fops,
	},
	[3] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer3",
		.fops = &timer_fops,
	},
	[4] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer4",
		.fops = &timer_fops,
	},
	[5] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer5",
		.fops = &timer_fops,
	},
	[6] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer6",
		.fops = &timer_fops,
	},
	[7] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer7",
		.fops = &timer_fops,
	},
	[8] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer8",
		.fops = &timer_fops,
	},
	[9] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer9",
		.fops = &timer_fops,
	},
	[10] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer10",
		.fops = &timer_fops,
	},
	[11] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "timer11",
		.fops = &timer_fops,
	},

};

static int ma35d1_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const char *clkmux;
	u32   ch, val32[2], val;
	int ret;

	dev_info(dev, "Nuvoton MA35D1 Timer Driver");

	if (of_property_read_u32_array(pdev->dev.of_node, "port-number", val32,
					1) != 0) {
		pr_err("%s can not get port-number!\n", __func__);
		return -EINVAL;
	}

	ch = val32[0];
	tmr[ch] = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_timer), GFP_KERNEL);
	if (tmr[ch] == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for timer device\n");
		return -ENOMEM;
	}

	tmr[ch]->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmr[ch]->base))
		return PTR_ERR(tmr[ch]->base);

	tmr[ch]->dev = &pdev->dev;
	ret = misc_register(&timer_dev[ch]);
	if (ret)
		pr_err("misc registration failed\n");

	/* Enable clock */
	tmr[ch]->clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(tmr[ch]->clk)) {
		ret = PTR_ERR(tmr[ch]->clk);
		dev_err(&pdev->dev, "failed to get tmr%d_gate\n", ch);
		return -ENOENT;
	}
	ret = clk_prepare_enable(tmr[ch]->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tmr%d clk\n", ch);
		return -ENOENT;
	}

	tmr[ch]->eclk = devm_clk_get(dev, clkmux);
	if (IS_ERR(tmr[ch]->eclk)) {
		if (PTR_ERR(tmr[ch]->eclk) != -ENOENT)
			return PTR_ERR(tmr[ch]->eclk);

		tmr[ch]->eclk = NULL;
	}

	ret = clk_prepare_enable(tmr[ch]->eclk);
	if (ret) {
		dev_err(dev, "Failed to enable tmr%d_eclk\n", ch);
		return ret;
	}

	// Get Timer clock source
	node = of_parse_phandle(pdev->dev.of_node, "nuvoton,clk", 0);
	if (node) {
		tmr[ch]->regmap = syscon_node_to_regmap(node);
		if (IS_ERR(tmr[ch]->regmap))
			return PTR_ERR(tmr[ch]->regmap);

		if (ch <= 7) {
			regmap_read(tmr[ch]->regmap, 0x1c, &val);
			tmr[ch]->clksel = (val & (0x7 << (ch*4))) >> (ch*4);
		} else {
			regmap_read(tmr[ch]->regmap, 0x20, &val);
			tmr[ch]->clksel = (val & (0x7 << ((ch%8)*4))) >> ((ch%8)*4);
		}
	}

	tmr[ch]->minor = MINOR(timer_dev[ch].minor);
	tmr[ch]->ch = ch;
	spin_lock_init(&tmr[ch]->lock);

	tmr[ch]->irq = platform_get_irq(pdev, 0);
	if (tmr[ch]->irq < 0)
		return tmr[ch]->irq;

	init_waitqueue_head(&tmr[ch]->wq);
	platform_set_drvdata(pdev, tmr[ch]);

	return 0;
}

static int ma35d1_timer_remove(struct platform_device *pdev)
{
	struct ma35d1_timer *t = platform_get_drvdata(pdev);
	u8 ch = t->ch;

	misc_deregister(&timer_dev[ch]);

	return 0;
}

static int ma35d1_timer_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct ma35d1_timer *t = platform_get_drvdata(pdev);

	if (t->ch == gu8_ch) {
		writel_relaxed(readl_relaxed(t->base + REG_TIMER_CTL) | TIMER_PERIODIC |
				TIMER_CNT_IEN | TIMER_WK_EN, t->base + REG_TIMER_CTL);

		enable_irq_wake(t->irq);
	}

	return 0;
}

static int ma35d1_timer_resume(struct platform_device *pdev)
{
	struct ma35d1_timer *t = platform_get_drvdata(pdev);

	if (t->ch == gu8_ch) {
		writel_relaxed(readl_relaxed(t->base + REG_TIMER_CTL) | TIMER_PERIODIC |
				TIMER_CNT_IEN | TIMER_WK_EN, t->base + REG_TIMER_CTL);
		disable_irq_wake(t->irq);
	}

	return 0;
}

static const struct of_device_id ma35d1_tmr_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-timer" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_tmr_of_match);

static struct platform_driver ma35d1_tmr_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ma35d1-timer",
		.of_match_table = of_match_ptr(ma35d1_tmr_of_match),
	},
	.probe		= ma35d1_timer_probe,
	.remove		= ma35d1_timer_remove,
	.suspend	= ma35d1_timer_suspend,
	.resume		= ma35d1_timer_resume,
};

module_platform_driver(ma35d1_tmr_driver);

MODULE_ALIAS("platform:ma35d1-timer");
MODULE_DESCRIPTION("Timer driver for Nuvoton MA35D1");
MODULE_LICENSE("GPL");


