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
	u32 mask;
	const struct clk_div_table *table;
	spinlock_t *lock;
};

#define to_ma35d1_adc_clk_divider(_hw)	\
	container_of(_hw, struct ma35d1_adc_clk_divider, hw)

static unsigned long ma35d1_clkdiv_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	unsigned int val;
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);

	val = readl_relaxed(dclk->reg) >> dclk->shift;
	val &= div_mask(dclk->width);
	val += 1;

	return divider_recalc_rate(hw, parent_rate, val, dclk->table,
				   CLK_DIVIDER_ROUND_CLOSEST, dclk->width);
}

static long ma35d1_clkdiv_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *prate)
{
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);

	return divider_round_rate(hw, rate, prate, dclk->table,
				  dclk->width, CLK_DIVIDER_ROUND_CLOSEST);
}

static int ma35d1_clkdiv_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	int value;
	unsigned long flags = 0;
	u32 data;
	struct ma35d1_adc_clk_divider *dclk = to_ma35d1_adc_clk_divider(hw);

	value = divider_get_val(rate, parent_rate, dclk->table,
				dclk->width, CLK_DIVIDER_ROUND_CLOSEST);

	if (dclk->lock)
		spin_lock_irqsave(dclk->lock, flags);

	data = readl_relaxed(dclk->reg);
	data &= ~(div_mask(dclk->width) << dclk->shift);
	data |= (value - 1) << dclk->shift;
	data |= dclk->mask;

	writel_relaxed(data, dclk->reg);

	if (dclk->lock)
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
				     u8 shift, u8 width, u32 mask_bit)
{
	struct ma35d1_adc_clk_divider *div;
	struct clk_init_data init;
	struct clk_div_table *table;
	u32 max_div, min_div;
	struct clk_hw *hw;

	int ret;
	int i;

	/* allocate the divider */
	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	/* Init the divider table */
	max_div = div_mask(width) + 1;
	min_div = 1;

	table = kcalloc(max_div + 1, sizeof(*table), GFP_KERNEL);
	if (!table) {
		kfree(div);
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < max_div; i++) {
		table[i].val = (min_div + i);
		table[i].div = 2 * table[i].val;
	}
	table[max_div].val = 0;
	table[max_div].div = 0;

	init.name = name;
	init.ops = &ma35d1_adc_clkdiv_ops;
	init.flags |= flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct ma35d1_adc_clk_divider assignments */
	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->mask = mask_bit ? BIT(mask_bit) : 0;
	div->lock = &ma35d1_lock;
	div->hw.init = &init;
	div->table = table;

	/* Register the clock */
	hw = &div->hw;
	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(table);
		kfree(div);
		return ERR_PTR(ret);
	}

	return hw;
}
