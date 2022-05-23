/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 *
 * Author: Chi-Fang Li <cfli0@nuvoton.com>
 */

#ifndef CLK_NUVOTON_CLK_MA35D1_H
#define CLK_NUVOTON_CLK_MA35D1_H

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d1-sys.h>

enum ma35d1_pll_type {
	MA35D1_CAPLL,
	MA35D1_DDRPLL,
	MA35D1_APLL,
	MA35D1_EPLL,
	MA35D1_VPLL,
};

enum ma35d1_pll_mode {
	VSIPLL_INTEGER_MODE,
	VSIPLL_FRACTIONAL_MODE,
	VSIPLL_SS_MODE,
};

/* VSI-PLL CTL0~2 */
#define VSIPLL_CTL0		0x0
#define VSIPLL_CTL1		0x4
#define VSIPLL_CTL2		0x8

/* VSI-PLL Specification limits */
#define VSIPLL_FREF_MAX_FREQ	200000000UL
#define VSIPLL_FREF_MIN_FREQ	1000000UL

#define VSIPLL_FREFDIVM_MAX_FREQ	40000000UL
#define VSIPLL_FREFDIVM_MIN_FREQ0	1000000UL
#define VSIPLL_FREFDIVM_MIN_FREQ1	10000000UL

#define VSIPLL_FCLK_MAX_FREQ		2400000000UL
#define VSIPLL_FCLK_MIN_FREQ		600000000UL

#define VSIPLL_FCLKO_MAX_FREQ		2400000000UL
#define VSIPLL_FCLKO_MIN_FREQ		85700000UL

#define VSIPLL_SPREAD_RANGE		194
#define VSIPLL_MODULATION_FREQ		50000

/* Clock Control Registers Offset */
#define REG_CLK_PWRCTL          (0x00)
#define REG_CLK_SYSCLK0         (0x04)
#define REG_CLK_SYSCLK1         (0x08)
#define REG_CLK_APBCLK0         (0x0C)
#define REG_CLK_APBCLK1         (0x10)
#define REG_CLK_APBCLK2         (0x14)
#define REG_CLK_CLKSEL0         (0x18)
#define REG_CLK_CLKSEL1         (0x1C)
#define REG_CLK_CLKSEL2         (0x20)
#define REG_CLK_CLKSEL3         (0x24)
#define REG_CLK_CLKSEL4         (0x28)
#define REG_CLK_CLKDIV0         (0x2C)
#define REG_CLK_CLKDIV1         (0x30)
#define REG_CLK_CLKDIV2         (0x34)
#define REG_CLK_CLKDIV3         (0x38)
#define REG_CLK_CLKDIV4         (0x3C)
#define REG_CLK_CLKOCTL         (0x40)
#define REG_CLK_STATUS          (0x50)
#define REG_CLK_PLL0CTL0        (0x60)
#define REG_CLK_PLL2CTL0        (0x80)
#define REG_CLK_PLL2CTL1        (0x84)
#define REG_CLK_PLL2CTL2        (0x88)
#define REG_CLK_PLL3CTL0        (0x90)
#define REG_CLK_PLL3CTL1        (0x94)
#define REG_CLK_PLL3CTL2        (0x98)
#define REG_CLK_PLL4CTL0        (0xA0)
#define REG_CLK_PLL4CTL1        (0xA4)
#define REG_CLK_PLL4CTL2        (0xA8)
#define REG_CLK_PLL5CTL0        (0xB0)
#define REG_CLK_PLL5CTL1        (0xB4)
#define REG_CLK_PLL5CTL2        (0xB8)
#define REG_CLK_CLKDCTL         (0xC0)
#define REG_CLK_CLKDSTS         (0xC4)
#define REG_CLK_CDUPB           (0xC8)
#define REG_CLK_CDLOWB          (0xCC)
#define REG_CLK_CKFLTRCTL       (0xD0)
#define REG_CLK_TESTCLK         (0xF0)
#define REG_CLK_PLLCTL          (0x40)

/* Constant Definitions for Clock Controller */
#define SMICPLLCTL0_FBDIV_POS	(0)
#define SMICPLLCTL0_FBDIV_MSK	(0xfful << SMICPLLCTL0_FBDIV_POS)

#define SMICPLLCTL0_INDIV_POS	(8)
#define SMICPLLCTL0_INDIV_MSK	(0xful << SMICPLLCTL0_INDIV_POS)

#define SMICPLLCTL0_OUTDIV_POS	(12)
#define SMICPLLCTL0_OUTDIV_MSK	(0x3ul << SMICPLLCTL0_OUTDIV_POS)

#define SMICPLLCTL0_PD_POS	(16)
#define SMICPLLCTL0_PD_MSK	(0x1ul << SMICPLLCTL0_PD_POS)

#define SMICPLLCTL0_BP_POS	(17)
#define SMICPLLCTL0_BP_MSK	(0x1ul << SMICPLLCTL0_BP_POS)

#define VSIPLLCTL0_FBDIV_POS	(0)
#define VSIPLLCTL0_FBDIV_MSK	(0x7fful << VSIPLLCTL0_FBDIV_POS)

#define VSIPLLCTL0_INDIV_POS	(12)
#define VSIPLLCTL0_INDIV_MSK	(0x3ful << VSIPLLCTL0_INDIV_POS)

#define VSIPLLCTL0_MODE_POS	(18)
#define VSIPLLCTL0_MODE_MSK	(0x3ul << VSIPLLCTL0_MODE_POS)

#define VSIPLLCTL0_SSRATE_POS	(20)
#define VSIPLLCTL0_SSRATE_MSK	(0x7fful << VSIPLLCTL0_SSRATE_POS)

#define VSIPLLCTL1_PD_POS	(0)
#define VSIPLLCTL1_PD_MSK	(0x1ul << VSIPLLCTL1_PD_POS)

#define VSIPLLCTL1_BP_POS	(1)
#define VSIPLLCTL1_BP_MSK	(0x1ul << VSIPLLCTL1_BP_POS)

#define VSIPLLCTL1_OUTDIV_POS	(4)
#define VSIPLLCTL1_OUTDIV_MSK	(0x7ul << VSIPLLCTL1_OUTDIV_POS)

#define VSIPLLCTL1_FRAC_POS	(8)
#define VSIPLLCTL1_FRAC_MSK	(0xfffffful << VSIPLLCTL1_FRAC_POS)

#define VSIPLLCTL2_SLOPE_POS	(0)
#define VSIPLLCTL2_SLOPE_MSK	(0xfffffful << VSIPLLCTL2_SLOPE_POS)

struct clk_hw *ma35d1_reg_clk_pll(enum ma35d1_pll_type type, u8 u8mode,
				 const char *name, const char *parent,
				 unsigned long targetFreq,
				 void __iomem *base,
				 struct regmap *regmap);

struct clk_hw *ma35d1_reg_adc_clkdiv(struct device *dev,
				    const char *name,
				    const char *parent_name,
				    unsigned long flags,
				    void __iomem *reg, u8 shift,
				    u8 width, u32 mask_bit);

extern spinlock_t ma35d1_lock;

static inline struct clk_hw *ma35d1_clk_fixed(const char *name, int rate)
{
	return clk_hw_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk_hw *ma35d1_clk_mux(const char *name,
					    void __iomem *reg, u8 shift,
					    u8 width,
					    const char *const *parents,
					    int num_parents)
{
	return clk_hw_register_mux(NULL, name, parents, num_parents,
				   CLK_SET_RATE_NO_REPARENT, reg, shift,
				   width, 0, &ma35d1_lock);
}

static inline struct clk_hw *ma35d1_clk_divider(const char *name,
						const char *parent,
						void __iomem *reg, u8 shift,
						u8 width)
{
	return clk_hw_register_divider(NULL, name, parent, CLK_SET_RATE_PARENT,
				       reg, shift, width, 0, &ma35d1_lock);
}

static inline struct clk_hw *ma35d1_clk_divider_pow2(const char *name,
						     const char *parent,
						     void __iomem *reg,
						     u8 shift, u8 width)
{
	return clk_hw_register_divider(NULL, name, parent,
				       CLK_DIVIDER_POWER_OF_TWO, reg, shift,
				       width, 0, &ma35d1_lock);
}

static inline struct clk_hw *ma35d1_clk_divider_table(const char *name,
						      const char *parent,
						      void __iomem *reg,
						      u8 shift, u8 width,
						      const struct clk_div_table
						      *table)
{
	return clk_hw_register_divider_table(NULL, name, parent, 0,
					     reg, shift, width, 0, table,
					     &ma35d1_lock);
}

static inline struct clk_hw *ma35d1_clk_fixed_factor(const char *name,
						     const char *parent,
						     unsigned int mult,
						     unsigned int div)
{
	return clk_hw_register_fixed_factor(NULL, name, parent,
					    CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk_hw *ma35d1_clk_gate(const char *name,
					     const char *parent,
					     void __iomem *reg, u8 shift)
{
	return clk_hw_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT,
				    reg, shift, 0, &ma35d1_lock);
}

#endif

