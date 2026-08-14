// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35D1 Clock Divider driver for ADC
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 *
 * Author: Chi-Fang Li <cfli0@nuvoton.com>
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/spinlock.h>

#include "clk-ma35d1.h"

#define div_mask(width)	((1 << (width)) - 1)

struct ma35d1_adc_clk_divider {
	struct clk_hw hw;
	void __iomem *reg;
	u8 shift;
	u8 width;
	spinlock_t *lock;
};

#define to_ma35d1_adc_clk_divider(_hw)	\
	container_of(_hw, struct ma35d1_adc_clk_divider, hw)

static unsigned long ma35d1_clkdiv_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);
	unsigned int val;

	val = readl_relaxed(dclk->reg) >> dclk->shift;
	val &= div_mask(dclk->width);

	return DIV_ROUND_CLOSEST_ULL((u64)parent_rate, 2 * (val + 1));
}

static long ma35d1_clkdiv_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);
	unsigned int val;

	if (!rate)
		return -EINVAL;

	val = DIV_ROUND_UP(*prate, 2 * rate);
	if (val == 0)
		val = 1;
	if (val > (unsigned int)div_mask(dclk->width) + 1)
		val = div_mask(dclk->width) + 1;

	return DIV_ROUND_CLOSEST_ULL((u64)*prate, 2 * val);
}

static int ma35d1_clkdiv_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);
	unsigned long flags = 0;
	unsigned int val;
	u32 data;

	if (!rate)
		return -EINVAL;

	val = DIV_ROUND_UP(parent_rate, 2 * rate);
	if (val == 0)
		val = 1;
	if (val > (unsigned int)div_mask(dclk->width) + 1)
		val = div_mask(dclk->width) + 1;

	spin_lock_irqsave(dclk->lock, flags);
	data = readl_relaxed(dclk->reg);
	data &= ~(div_mask(dclk->width) << dclk->shift);
	data |= (val - 1) << dclk->shift;
	writel_relaxed(data, dclk->reg);
	spin_unlock_irqrestore(dclk->lock, flags);

	return 0;
}

static const struct clk_ops ma35d1_adc_clkdiv_ops = {
	.recalc_rate = ma35d1_clkdiv_recalc_rate,
	.round_rate = ma35d1_clkdiv_round_rate,
	.set_rate = ma35d1_clkdiv_set_rate,
};

struct clk_hw *ma35d1_reg_adc_clkdiv(struct device *dev, const char *name,
				     const char *parent_name,
				     unsigned long flags, void __iomem *reg,
				     u8 shift, u8 width)
{
	struct ma35d1_adc_clk_divider *div;
	struct clk_init_data init = {};
	struct clk_hw *hw;
	int ret;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &ma35d1_adc_clkdiv_ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->lock = &ma35d1_lock;
	div->hw.init = &init;

	hw = &div->hw;
	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(div);
		return ERR_PTR(ret);
	}

	return hw;
}
