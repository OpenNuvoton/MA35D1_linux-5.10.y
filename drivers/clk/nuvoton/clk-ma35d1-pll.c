/*
 * driver/clk/nuvoton/clk-ma35d1-pll.c
 *
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>

#include "clk-ma35d1.h"

struct clk_pll {
	struct clk_hw hw;
	u32 plltype;
	u8	mode;
	unsigned long clko;
	void __iomem *ctl0_base;
	void __iomem *ctl1_base;
	void __iomem *ctl2_base;
};

unsigned long CLK_CalPLLFreq_Mode0(unsigned long PllSrcClk, unsigned long u64PllFreq, u32 *u32Reg)
{
	u32 u32M, u32N, u32P;
	u32 u32Tmp, u32Min, u32MinN, u32MinM, u32MinP;

	unsigned long u64basFreq, u64PllClk;
	unsigned long u64Con1, u64Con2, u64Con3;

	/* Find best solution */
	u32Min = (u32) - 1;
	u32MinM = 0UL;
	u32MinN = 0UL;
	u32MinP = 0UL;
	u64basFreq = u64PllFreq;
	if((u64PllFreq <= 2400000000) && (u64PllFreq >= 85700000)) {
		for(u32M = 1UL; u32M < 64UL; u32M++) {
			u64Con1 = PllSrcClk/u32M;
			if(!((u64Con1 <= 40000000) && (u64Con1 >= 1000000)))  continue;

			for(u32N = 16UL; u32N < 2048UL; u32N++) {
				u64Con2 = u64Con1 * u32N;
				if(!((u64Con2 <= 2400000000) && (u64Con2 >= 600000000)))  continue;

				for(u32P = 1UL; u32P < 8UL; u32P++) {
					/* Break when get good results */
					if (u32Min == 0UL) {
						break;
					}

					u64Con3 = u64Con2 / u32P;
					if(!((u64Con3 <= 2400000000) && (u64Con3 >= 85700000)))	 continue;

					u32Tmp = (u64Con3 > u64PllFreq) ? u64Con3 - u64PllFreq : u64PllFreq - u64Con3;
					if(u32Tmp < u32Min) {
						u32Min = u32Tmp;
						u32MinM = u32M;
						u32MinN = u32N;
						u32MinP = u32P;

					} else {
						if (u64Con3 < u64PllFreq)  break;
					}
				}
			}
		}

		/* Enable and apply new PLL setting. */
		u32Reg[0] =	 (u32MinM << 12) | (u32MinN);
		u32Reg[1] =	 (u32MinP << 4);

		/* Actual PLL output clock frequency */
		u64PllClk = (PllSrcClk * u32MinN) / (u32MinP * (u32MinM));
	} else {
		u32Reg[0] = 0x30FA;
		u32Reg[1] = 0x20;

		/* Actual PLL output clock frequency */
		u64PllClk = 1000000000;
	}

	return u64PllClk;
}

unsigned long CLK_CalPLLFreq_Mode1(unsigned long PllSrcClk, unsigned long u64PllFreq, u32 *u32Reg)
{
	unsigned long u64X, u64N, u64M, u64P, u64tmp, u64tmpP, u64tmpM;
	unsigned long u64PllClk, u64FCLKO;
	u32 u32FRAC, i;

	// check condition 1
	if ((PllSrcClk > 200000000) || (PllSrcClk < 1000000)) {
		// Fref is incorrect, return fail
		return 0;
	}

	// check condition 4
	if(u64PllFreq < 85700000) {
		// Adjust u64FCLKO
		u64FCLKO = 0;

		for(i = 2; i < 100; i++) {
			u64tmp = (i * u64PllFreq);
			if (u64tmp > 85700000) {
				u64FCLKO = u64tmp;
				break;
			}
		}

		if (u64FCLKO == 0) return 0;
	} else if(u64PllFreq >= 2400000000) {
		u32Reg[0] = 0x30FA;
		u32Reg[1] = (0x2 << 4);
		u64PllClk = 1000000000;
		return u64PllClk;
	} else
		u64FCLKO = u64PllFreq;

	// Find P
	u64P = 0;
	for(i = 1; i < 8; i++) {
		u64tmpP =  i * u64FCLKO;
		// it should be condition 3
		if((u64tmpP <= 2400000000) && (u64tmpP >= 600000000)) {
			u64P = i;
			break;
		}
	}

	// No reasonable P is found, return fail.
	if (u64P == 0)	return 0;

	// Find M
	u64M = 0; // Initialize it, and use it to judge reasonable M is found or not
	for(i = 1; i < 64; i++) {
		u64tmpM = PllSrcClk / i;
		if((u64tmpM <= 40000000) && (u64tmpM >= 10000000)) { // condition 2
			u64M = i;
			break;
		}
	}

	if (u64M == 0) { // No reasonable M is found
		return 0;
	}

	u64tmp = (u64FCLKO * u64P * u64M * 1000)/ PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;

	u32Reg[0] = (u64M << 12) | (u64N);
	u32Reg[1] = (u64P << 4) | (u32FRAC << 8);

	/* Actual PLL output clock frequency */
	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

	return u64PllClk;
}

unsigned long CLK_CalPLLFreq_Mode2(unsigned long PllSrcClk, unsigned long u64PllFreq, u32 u32SR, u32 u32Fmod, u32 *u32Reg)
{

	unsigned long u64PllClk;

#if 0 // Spread Specrum mode PLL calculating

	unsigned long u64X, u64N, u64M, u64P, u64tmp, u64tmpP, u64tmpM;
	unsigned long u64PllClk, u64FCLKO;
	u32 u32FRAC, i;

	// check condition 1
	if ((PllSrcClk > 200000000) || (PllSrcClk < 1000000)) {
		// Fref is incorrect, return fail case
		return 0;
	}

	// check condition 4
	if(u64PllFreq < 85700000) {
		u64FCLKO = 0;
		for(i = 2; i < 8; i++) {
			u64tmp = (i * u64PllFreq);
			if (u64tmp > 85700000) {
				u64FCLKO = u64tmp;
			}
		}

		if (u64FCLKO == 0) return 0;
	} else if(u64PllFreq >= 2400000000) {
		u32Reg[0] = 0x30FA;
		u32Reg[1] = (0x2 << 4);
		u64PllClk = 1000000000;
		return u64PllClk;
	} else
		u64FCLKO = u64PllFreq;

	// Find P
	u64P = 0;
	for(i = 1; i < 8; i++) {
		u64tmpP =  i * u64FCLKO;
		if((u64tmpP <= 2400000000) && (u64tmpP >= 600000000)) {
			u64P = i;
			break;
		}
	}

	// No reasonable P is found, return fail.
	if (u64P == 0)	return 0;

	// Find M
	u64M = 0; // Initialize it, and use it to judge reasonable M is found or not
	for(i = 1; i < 64; i++) {
		u64tmpM = PllSrcClk / i;
		if((u64tmpM <= 40000000) && (u64tmpM >= 10000000)) { // condition 2
			u64M = i;
			break;
		}
	}

	if (u64M == 0) { // No reasonable M is found
		return 0;
	}

	u64tmp = (u64FCLKO * u64P * u64M * 1000)/ PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;

	u64SSRATE = ((PllSrcClk >> 1) / (u32Fmod*2)) -1;
	u64SLOPE = ((u64tmp * u32SR / u64SSRATE) << 24) / 100 / 1000;

	u32Reg[0] = (u64SSRATE << 20) | (u64M << 12) | (u64N);
	u32Reg[1] = (u64P << 4) | (u32FRAC << 8);
	u32Reg[2] = u64SLOPE;

	/* Actual PLL output clock frequency */
	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

#else

	// Workround :Slope bug(only 16bit)
	if(u64PllFreq <= 266000000) {
		u32Reg[0] = 0x07782085;
		u32Reg[1] = 0x60;
		u32Reg[2] = 0x58CF9;
		u64PllClk = 266000000;
	} else {
		u32Reg[0] = 0x12b81016;
		u32Reg[1] = 0x35532610;
		u32Reg[2] = 0x9208;
		u64PllClk = 533000000;
	}
#endif

	return u64PllClk;
}

unsigned long CLK_SetPLLFreq(struct clk_pll *pll, unsigned long PllSrcClk, unsigned long u64PllFreq)
{
	u32 u32Reg[3] = {0}, val_ctl0, val_ctl1, val_ctl2;
	unsigned long u64PllClk;

	val_ctl0 = __raw_readl(pll->ctl0_base);
	val_ctl1 = __raw_readl(pll->ctl1_base);
	val_ctl2 = __raw_readl(pll->ctl2_base);

	// PLL Operation mode setting
	val_ctl0 = (val_ctl0 & ~(0xc0000)) | (pll->mode << 18);

	if(pll->mode == 0) {
		u64PllClk = CLK_CalPLLFreq_Mode0(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = (val_ctl0 & ~(0x3f000) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4)) | u32Reg[1];
	} else if(pll->mode == 1) {
		u64PllClk = CLK_CalPLLFreq_Mode1(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = (val_ctl0 & ~(0x3f<< 12) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4) & ~(0xffffff << 8)) | u32Reg[1];
	} else { //pll->mode == 2
		u64PllClk = CLK_CalPLLFreq_Mode2(PllSrcClk, u64PllFreq, 50000, 194, u32Reg); //50khz, 1.94%
		val_ctl0 = (val_ctl0 & ~(0x7ff << 20) & ~(0x3f << 12) & ~(0x7ff)) | u32Reg[0];
		val_ctl1 = (val_ctl1 & ~(0x7 << 4) & ~(0xffffff << 8)) | u32Reg[1];
		val_ctl2 = u32Reg[2];
	}

	__raw_writel(val_ctl0, pll->ctl0_base);
	__raw_writel(val_ctl1, pll->ctl1_base);
	__raw_writel(val_ctl2, pll->ctl2_base);

	return u64PllClk;
}

unsigned long CLK_GetPLLFreq(struct clk_pll *pll, unsigned long PllSrcClk)
{
	u32 u32M, u32N, u32P, u32X, u32SR, u32FMOD;
	u32 val_ctl0, val_ctl1, val_ctl2;
	unsigned long u64PllClk, u64X;

	val_ctl0 = __raw_readl(pll->ctl0_base);
	val_ctl1 = __raw_readl(pll->ctl1_base);
	val_ctl2 = __raw_readl(pll->ctl2_base);

	if(val_ctl0 == 0)
		u64PllClk = CLK_SetPLLFreq(pll, PllSrcClk, pll->clko);
	else {

		if(pll->mode == 0) {
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;

			u32P = ((val_ctl1 & (0x70)) >> 4);

			/* Actual PLL output clock frequency */
			u64PllClk = (PllSrcClk * u32N) / (u32P * (u32M));

		} else if(pll->mode == 1) {
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;

			u32P = ((val_ctl1 & (0x70)) >> 4);
			u32X =	(val_ctl1 & 0xffffff00) >> 8;

			/* Actual PLL output clock frequency */
			u64X = (u64)u32X;
			u64X = (((u64X * 1000)+500) >> 24);
			u64PllClk = (PllSrcClk * ((u32N*1000) + u64X)) / 1000 / u32P / u32M;

		} else { //pll->mode == 2
			u32N = (val_ctl0 & (0x7ff));
			u32M = (val_ctl0 & (0x3f000)) >> 12;
			u32SR = (val_ctl0 & (0x7ff00000)) >> 20;

			u32P = ((val_ctl1 & (0x70)) >> 4);
			u32X =	(val_ctl1 & 0xffffff00) >> 8;

			u32FMOD = (val_ctl2 & (0x00ffffff));

			/* Actual PLL output clock frequency */
			u64X = (u64)u32X;
			u64X = ((u64X * 1000) >> 24);

			//u64PllClk = (PllSrcClk * ((u32N*1000) + u64X)) / 1000 / u32P / u32M;
			u64PllClk = pll->clko;
		}
	}

	return u64PllClk;
}

#define to_clk_pll(clk) (container_of(clk, struct clk_pll, clk))

static int clk_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	unsigned long ll;
	struct clk_pll *pll = to_clk_pll(hw);

	if(!((parent_rate <= 200000000) && (parent_rate >= 1000000))) {
		printk("XXX Pll clock source error.\n");
		return 0;
	}

	if (pll->plltype == MA35D1_CAPLL) {
		__raw_writel(0x30FA, pll->ctl0_base);
		__raw_writel(0x21, pll->ctl1_base);
		__raw_writel(0, pll->ctl2_base);
		ll = 1000000000;
	} else if (pll->plltype == MA35D1_SYSPLL) {
		__raw_writel(0x60e4, pll->ctl0_base);
		__raw_writel(0x51, pll->ctl1_base);
		__raw_writel(0, pll->ctl2_base);
		ll = 189000000;
	} else if (pll->plltype == MA35D1_DDRPLL) {
		__raw_writel(0x0F04102C, pll->ctl0_base);
		__raw_writel(0x6B851E41, pll->ctl1_base);
		__raw_writel(0x48A3, pll->ctl2_base);
		ll = 266000000;
	} else if (pll->plltype == MA35D1_EPLL) {
		__raw_writel(0x60fa, pll->ctl0_base);
		__raw_writel(0x21, pll->ctl1_base);
		__raw_writel(0, pll->ctl2_base);
		ll = 500000000;
	} else {
		ll = CLK_SetPLLFreq(pll, parent_rate, rate);
	}

	pll->clko = ll;
	return 0;
}


static unsigned long clk_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	unsigned long ll;
	struct clk_pll *pll = to_clk_pll(hw);

	if ((parent_rate > 200000000) || (parent_rate < 1000000)) {
		pr_debug("PLL source clock Error!!!\n");
		return 0;
	}

	if (pll->plltype == MA35D1_CAPLL)
		ll = 1000000000;
	else if (pll->plltype == MA35D1_SYSPLL)
		ll = 189000000;
	else if (pll->plltype == MA35D1_DDRPLL)
		ll = 266000000;
	else if (pll->plltype == MA35D1_EPLL)
		ll = 500000000;
	else
		ll = CLK_GetPLLFreq(pll, parent_rate);

	return ll;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	return rate;
}

static int clk_pll_enable(struct clk_hw *hw)
{

	struct clk_pll *pll = to_clk_pll(hw);
	u32 val;

	val = __raw_readl(pll->ctl1_base);
	val &= ~0x1;/* PD = 0, power down mode disable */
	__raw_writel(val, pll->ctl1_base);

	return 0;
}

static void clk_pll_disable(struct clk_hw *hw)
{

	struct clk_pll *pll = to_clk_pll(hw);
	u32 val;

	val = __raw_readl(pll->ctl1_base);
	val |= 0x1;/* PD = 1, power down mode enable */
	__raw_writel(val, pll->ctl1_base);
}

static struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pll_recalc_rate,
	.enable = clk_pll_enable,
	.disable = clk_pll_disable,
	.set_rate = clk_pll_set_rate,
	.round_rate = clk_pll_round_rate,
};

struct clk *ma35d1_clk_pll(enum ma35d1_pll_type type, u8 u8mode, const char *name, const char *parent,
                            unsigned long targetFreq, void __iomem *base)
{

	struct clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kmalloc(sizeof(*pll), GFP_KERNEL);

	if (!pll) {
		pr_debug("pll(%d)  kmalloc Fail!!!\n", type);
		return ERR_PTR(-ENOMEM);
	}

	pll->plltype = type;
	pll->mode = u8mode;
	pll->clko = targetFreq;
	pll->ctl0_base = base;
	pll->ctl1_base = pll->ctl0_base + 0x4;
	pll->ctl2_base = pll->ctl1_base + 0x4;

	init.name = name;
	init.ops = &clk_pll_ops;
	init.flags = 0;
	init.parent_names = &parent;
	init.num_parents = 1;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);

	if (IS_ERR(clk)) {
		pr_debug("clk_register Error!!!\n");
		kfree(pll);
	}

	return clk;
}
