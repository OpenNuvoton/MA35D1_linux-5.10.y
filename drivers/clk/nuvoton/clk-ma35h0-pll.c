// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35H0 Clock PLL driver
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 *
 * Author: Chi-Fang Li <cfli0@nuvoton.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/bitfield.h>

#include "clk-ma35h0.h"

#define to_ma35h0_clk_pll(clk) \
	(container_of(clk, struct ma35h0_clk_pll, clk))

#define PLL0CTL0_FBDIV_MSK		GENMASK(7, 0)
#define PLL0CTL0_INDIV_MSK		GENMASK(11, 8)
#define PLL0CTL0_OUTDIV_MSK		GENMASK(13, 12)
#define PLL0CTL0_PD_MSK			BIT(16)
#define PLL0CTL0_BP_MSK			BIT(17)
#define PLLXCTL0_FBDIV_MSK		GENMASK(10, 0)
#define PLLXCTL0_INDIV_MSK		GENMASK(17, 12)
#define PLLXCTL0_MODE_MSK		GENMASK(19, 18)
#define PLLXCTL0_SSRATE_MSK		GENMASK(30, 20)
#define PLLXCTL1_PD_MSK			BIT(0)
#define PLLXCTL1_BP_MSK			BIT(1)
#define PLLXCTL1_OUTDIV_MSK		GENMASK(6, 4)
#define PLLXCTL1_FRAC_MSK		GENMASK(31, 8)
#define PLLXCTL2_SLOPE_MSK		GENMASK(23, 0)

struct ma35h0_clk_pll {
	struct clk_hw hw;
	u8 type;
	u8 mode;
	unsigned long rate;
	void __iomem *ctl0_base;
	void __iomem *ctl1_base;
	void __iomem *ctl2_base;
	struct regmap *regmap;
};

struct vsipll_freq_conf_reg_tbl {
	unsigned long freq;
	u8 mode;
	u32 ctl0_reg;
	u32 ctl1_reg;
	u32 ctl2_reg;
};

static const struct vsipll_freq_conf_reg_tbl ma35h0pll_freq[] = {
	{ 1000000000, VSIPLL_INTEGER_MODE, 0x307d, 0x10, 0 },
	{ 884736000, VSIPLL_FRACTIONAL_MODE, 0x41024, 0xdd2f1b11, 0 },
	{ 533000000, VSIPLL_SS_MODE, 0x12b8102c, 0x6aaaab20, 0x12317 },
	{ }
};

static void CLK_UnLockReg(struct ma35h0_clk_pll *pll)
{
	int ret;

	/* Unlock PLL registers */
	do {
		regmap_write(pll->regmap, REG_SYS_RLKTZNS, 0x59);
		regmap_write(pll->regmap, REG_SYS_RLKTZNS, 0x16);
		regmap_write(pll->regmap, REG_SYS_RLKTZNS, 0x88);
		regmap_read(pll->regmap, REG_SYS_RLKTZNS, &ret);
	} while (ret == 0);
}

static void CLK_LockReg(struct ma35h0_clk_pll *pll)
{
	/* Lock PLL registers */
	regmap_write(pll->regmap, REG_SYS_RLKTZNS, 0x0);
}

/* SMIC PLL for CAPLL */
unsigned long CLK_GetPLLFreq_SMICPLL(struct ma35h0_clk_pll *pll,
		unsigned long PllSrcClk)
{
	u32 u32M, u32N, u32P, u32OutDiv;
	u32 val;
	unsigned long u64PllClk;
	u32 clk_div_table[] = { 1, 2, 4, 8};

	val = __raw_readl(pll->ctl0_base);

	u32N = FIELD_GET(PLL0CTL0_FBDIV_MSK, val);
	u32M = FIELD_GET(PLL0CTL0_INDIV_MSK, val);
	u32P = FIELD_GET(PLL0CTL0_OUTDIV_MSK, val);
	u32OutDiv = clk_div_table[u32P];

	if (val & PLL0CTL0_BP_MSK) {
		u64PllClk = PllSrcClk;
	} else {
		u64PllClk = PllSrcClk * u32N;
		do_div(u64PllClk, u32M * u32OutDiv);
	}

	return u64PllClk;
}

/* VSI-PLL: INTEGER_MODE */
unsigned long CLK_CalPLLFreq_Mode0(unsigned long PllSrcClk,
		unsigned long u64PllFreq, u32 *u32Reg)
{
	u32 u32TmpM, u32TmpN, u32TmpP;
	u32 u32RngMinN, u32RngMinM, u32RngMinP;
	u32 u32RngMaxN, u32RngMaxM, u32RngMaxP;
	u32 u32Tmp, u32Min, u32MinN, u32MinM, u32MinP;
	unsigned long u64PllClk;
	unsigned long u64Con1, u64Con2, u64Con3;

	u64PllClk = 0;
	u32Min = (u32) -1;

	if (!((u64PllFreq >= VSIPLL_FCLKO_MIN_FREQ)
	      && (u64PllFreq <= VSIPLL_FCLKO_MAX_FREQ))) {
		u32Reg[0] = ma35h0pll_freq[0].ctl0_reg;
		u32Reg[1] = ma35h0pll_freq[0].ctl1_reg;
		u64PllClk = ma35h0pll_freq[0].freq;

		return u64PllClk;
	}

	u32RngMinM = 1UL;
	u32RngMaxM = 63UL;

	u32RngMinM = ((PllSrcClk / VSIPLL_FREFDIVM_MAX_FREQ) > 1) ?
			(PllSrcClk / VSIPLL_FREFDIVM_MAX_FREQ) : 1;
	u32RngMaxM = ((PllSrcClk / VSIPLL_FREFDIVM_MIN_FREQ0) < u32RngMaxM) ?
			(PllSrcClk / VSIPLL_FREFDIVM_MIN_FREQ0) : u32RngMaxM;

	for (u32TmpM = u32RngMinM; u32TmpM < (u32RngMaxM + 1); u32TmpM++) {
		u64Con1 = PllSrcClk / u32TmpM;

		u32RngMinN = 16UL;
		u32RngMaxN = 2047UL;

		u32RngMinN = ((VSIPLL_FCLK_MIN_FREQ / u64Con1) > u32RngMinN) ?
				(VSIPLL_FCLK_MIN_FREQ / u64Con1) : u32RngMinN;
		u32RngMaxN = ((VSIPLL_FCLK_MAX_FREQ / u64Con1) < u32RngMaxN) ?
				(VSIPLL_FCLK_MAX_FREQ / u64Con1) : u32RngMaxN;

		for (u32TmpN = u32RngMinN; u32TmpN < (u32RngMaxN + 1); u32TmpN++) {
			u64Con2 = u64Con1 * u32TmpN;

			u32RngMinP = 1UL;
			u32RngMaxP = 7UL;

			u32RngMinP = ((u64Con2 / VSIPLL_FCLKO_MAX_FREQ) > 1) ? (u64Con2 /
					VSIPLL_FCLKO_MAX_FREQ) : 1;
			u32RngMaxP = ((u64Con2 / VSIPLL_FCLKO_MIN_FREQ) < u32RngMaxP) ?
					(u64Con2 / VSIPLL_FCLKO_MIN_FREQ) : u32RngMaxP;
			for (u32TmpP = u32RngMinP; u32TmpP < (u32RngMaxP + 1); u32TmpP++) {
				u64Con3 = u64Con2 / u32TmpP;
				if (u64Con3 > u64PllFreq)
					u32Tmp = u64Con3 - u64PllFreq;
				else
					u32Tmp = u64PllFreq - u64Con3;

				if (u32Tmp < u32Min) {
					u32Min = u32Tmp;
					u32MinM = u32TmpM;
					u32MinN = u32TmpN;
					u32MinP = u32TmpP;

					if (u32Min == 0UL) {
						u32Reg[0] = (u32MinM << 12) | (u32MinN);
						u32Reg[1] = (u32MinP << 4);
						return ((PllSrcClk * u32MinN) / (u32MinP * u32MinM));
					}
				}
			}
		}
	}

	u32Reg[0] = (u32MinM << 12) | (u32MinN);
	u32Reg[1] = (u32MinP << 4);
	u64PllClk = (PllSrcClk * u32MinN) / (u32MinP * u32MinM);
	return u64PllClk;
}

/* VSI-PLL: FRACTIONAL_MODE */
unsigned long CLK_CalPLLFreq_Mode1(unsigned long PllSrcClk,
				unsigned long u64PllFreq, u32 *u32Reg)
{
	unsigned long u64X, u64N, u64M, u64P, u64tmp;
	unsigned long u64PllClk, u64FCLKO;
	u32 u32FRAC;

	if (u64PllFreq > VSIPLL_FCLKO_MAX_FREQ) {
		u32Reg[0] = ma35h0pll_freq[1].ctl0_reg;
		u32Reg[1] = ma35h0pll_freq[1].ctl1_reg;
		u64PllClk = ma35h0pll_freq[1].freq;
		return u64PllClk;
	}

	if (u64PllFreq > (VSIPLL_FCLKO_MIN_FREQ/(100-1)))
		u64FCLKO = u64PllFreq * ((VSIPLL_FCLKO_MIN_FREQ / u64PllFreq) + ((
				VSIPLL_FCLKO_MIN_FREQ % u64PllFreq) ? 1 : 0));
	else {
		pr_err("Failed to set rate %ld\n", u64PllFreq);
		return 0;
	}

	u64P = (u64FCLKO >= VSIPLL_FCLK_MIN_FREQ) ? 1 : ((VSIPLL_FCLK_MIN_FREQ /
		u64FCLKO) + ((VSIPLL_FCLK_MIN_FREQ % u64FCLKO) ? 1 : 0));
	if ((PllSrcClk > (VSIPLL_FREFDIVM_MAX_FREQ * (64-1))) ||
	    (PllSrcClk < VSIPLL_FREFDIVM_MIN_FREQ1)) {
		pr_err("Failed to set rate %ld\n", u64PllFreq);
		return 0;
	}
	u64M = (PllSrcClk <= VSIPLL_FREFDIVM_MAX_FREQ) ? 1 : ((PllSrcClk /
		VSIPLL_FREFDIVM_MAX_FREQ) + ((PllSrcClk % VSIPLL_FREFDIVM_MAX_FREQ) ? 1 : 0));

	u64tmp = (u64FCLKO * u64P * u64M * 1000) / PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;
	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

	u32Reg[0] = (u64M << 12) | (u64N);
	u32Reg[1] = (u64P << 4) | (u32FRAC << 8);

	return u64PllClk;
}

/* VSI-PLL: SS_MODE */
unsigned long CLK_CalPLLFreq_Mode2(unsigned long PllSrcClk,
		unsigned long u64PllFreq, u32 u32SR, u32 u32Fmod, u32 *u32Reg)
{
	unsigned long u64X, u64N, u64M, u64P, u64tmp, u64tmpP, u64tmpM;
	unsigned long u64SSRATE, u64SLOPE, u64PllClk, u64FCLKO;
	u32 u32FRAC, i;

	if (u64PllFreq >= VSIPLL_FCLKO_MAX_FREQ) {
		u32Reg[0] = ma35h0pll_freq[2].ctl0_reg;
		u32Reg[1] = ma35h0pll_freq[2].ctl1_reg;
		u32Reg[2] = ma35h0pll_freq[2].ctl2_reg;
		u64PllClk = ma35h0pll_freq[2].freq;
		return u64PllClk;
	}

	if (u64PllFreq < VSIPLL_FCLKO_MIN_FREQ) {
		u64FCLKO = 0;
		for (i = 2; i < 8; i++) {
			u64tmp = (i * u64PllFreq);
			if (u64tmp > VSIPLL_FCLKO_MIN_FREQ)
				u64FCLKO = u64tmp;
		}
		if (u64FCLKO == 0) {
			pr_err("Failed to set rate %ld\n", u64PllFreq);
			return 0;
		}

	} else
		u64FCLKO = u64PllFreq;

	u64P = 0;
	for (i = 1; i < 8; i++) {
		u64tmpP = i * u64FCLKO;
		if ((u64tmpP <= VSIPLL_FCLK_MAX_FREQ) && (u64tmpP >= VSIPLL_FCLK_MIN_FREQ)) {
			u64P = i;
			break;
		}
	}

	if (u64P == 0)
		return 0;

	u64M = 0;
	for (i = 1; i < 64; i++) {
		u64tmpM = PllSrcClk / i;
		if ((u64tmpM <= VSIPLL_FREFDIVM_MAX_FREQ)
			&& (u64tmpM >= VSIPLL_FREFDIVM_MIN_FREQ1)) {
			u64M = i;
			break;
		}
	}

	if (u64M == 0)
		return 0;

	u64tmp = (u64FCLKO * u64P * u64M * 1000) / PllSrcClk;
	u64N = u64tmp / 1000;
	u64X = u64tmp % 1000;
	u32FRAC = ((u64X << 24) + 500) / 1000;

	u64SSRATE = ((PllSrcClk >> 1) / (u32Fmod * 2)) - 1;
	u64SLOPE = ((u64tmp * u32SR / u64SSRATE) << 24) / 100 / 1000;

	u64PllClk = (PllSrcClk * u64tmp) / u64P / u64M / 1000;

	u32Reg[0] = (u64SSRATE << VSIPLLCTL0_SSRATE_POS) | (u64M <<
			VSIPLLCTL0_INDIV_POS) | (u64N);
	u32Reg[1] = (u64P << VSIPLLCTL1_OUTDIV_POS) | (u32FRAC << VSIPLLCTL1_FRAC_POS);
	u32Reg[2] = u64SLOPE;

	return u64PllClk;
}

unsigned long CLK_SetPLLFreq(struct ma35h0_clk_pll *pll,
		unsigned long PllSrcClk, unsigned long u64PllFreq)
{
	u32 u32Reg[3] = { 0 }, val_ctl0, val_ctl1, val_ctl2;
	unsigned long u64PllClk;

	val_ctl0 = __raw_readl(pll->ctl0_base);
	val_ctl1 = __raw_readl(pll->ctl1_base);
	val_ctl2 = __raw_readl(pll->ctl2_base);

	switch (pll->mode) {
	case VSIPLL_INTEGER_MODE:
		u64PllClk = CLK_CalPLLFreq_Mode0(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = u32Reg[0] | (VSIPLL_INTEGER_MODE << VSIPLLCTL0_MODE_POS);
		break;
	case VSIPLL_FRACTIONAL_MODE:
		u64PllClk = CLK_CalPLLFreq_Mode1(PllSrcClk, u64PllFreq, u32Reg);
		val_ctl0 = u32Reg[0] | (VSIPLL_FRACTIONAL_MODE << VSIPLLCTL0_MODE_POS);
		break;
	case VSIPLL_SS_MODE:
		u64PllClk = CLK_CalPLLFreq_Mode2(PllSrcClk, u64PllFreq, VSIPLL_MODULATION_FREQ,
				VSIPLL_SPREAD_RANGE, u32Reg);
		val_ctl0 = u32Reg[0] | (VSIPLL_SS_MODE << VSIPLLCTL0_MODE_POS);
		break;
	}

	val_ctl1 = VSIPLLCTL1_PD_MSK | u32Reg[1];
	val_ctl2 = u32Reg[2];

	__raw_writel(val_ctl0, pll->ctl0_base);
	__raw_writel(val_ctl1, pll->ctl1_base);
	__raw_writel(val_ctl2, pll->ctl2_base);

	return u64PllClk;
}

unsigned long CLK_GetPLLFreq_VSIPLL(struct ma35h0_clk_pll *pll,
		unsigned long PllSrcClk)
{
	u32 u32M, u32N, u32P, u32X, u32SR, u32FMOD;
	u32 val_ctl0, val_ctl1, val_ctl2;
	unsigned long u64PllClk, u64X;

	val_ctl0 = __raw_readl(pll->ctl0_base);
	val_ctl1 = __raw_readl(pll->ctl1_base);
	val_ctl2 = __raw_readl(pll->ctl2_base);

	if (val_ctl1 & PLLXCTL1_BP_MSK) {
		u64PllClk = PllSrcClk;
		return u64PllClk;
	}

	if (pll->mode == VSIPLL_INTEGER_MODE) {
		u32N = FIELD_GET(PLLXCTL0_FBDIV_MSK, val_ctl0);
		u32M = FIELD_GET(PLLXCTL0_INDIV_MSK, val_ctl0);
		u32P = FIELD_GET(PLLXCTL1_OUTDIV_MSK, val_ctl1);

		u64PllClk = PllSrcClk * u32N;
		do_div(u64PllClk, u32M * u32P);

	} else if (pll->mode == VSIPLL_FRACTIONAL_MODE) {
		u32N = FIELD_GET(PLLXCTL0_FBDIV_MSK, val_ctl0);
		u32M = FIELD_GET(PLLXCTL0_INDIV_MSK, val_ctl0);
		u32P = FIELD_GET(PLLXCTL1_OUTDIV_MSK, val_ctl1);
		u32X = FIELD_GET(PLLXCTL1_FRAC_MSK, val_ctl1);

		u64X = (u64) u32X;
		u64X = (((u64X * 1000) + 500) >> 24);
		u64PllClk = (PllSrcClk * ((u32N * 1000) + u64X)) / 1000 / u32P / u32M;

	} else {
		u32N = FIELD_GET(PLLXCTL0_FBDIV_MSK, val_ctl0);
		u32M = FIELD_GET(PLLXCTL0_INDIV_MSK, val_ctl0);
		u32SR = FIELD_GET(PLLXCTL0_SSRATE_MSK, val_ctl0);
		u32P = FIELD_GET(PLLXCTL1_OUTDIV_MSK, val_ctl1);
		u32X = FIELD_GET(PLLXCTL1_FRAC_MSK, val_ctl1);
		u32FMOD = FIELD_GET(PLLXCTL2_SLOPE_MSK, val_ctl2);

		u64X = (u64) u32X;
		u64X = ((u64X * 1000) >> 24);

		u64PllClk = (PllSrcClk * ((u32N * 1000) + u64X)) / 1000 / u32P / u32M;
	}

	return u64PllClk;
}

static int ma35h0_clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct ma35h0_clk_pll *pll = to_ma35h0_clk_pll(hw);

	if ((parent_rate < VSIPLL_FREF_MIN_FREQ) || (parent_rate > VSIPLL_FREF_MAX_FREQ))
		return 0;

	if ((pll->type == MA35H0_CAPLL) || (pll->type == MA35H0_DDRPLL)) {
		pr_warn("Nuvoton MA35H0 CAPLL/DDRPLL Read Only.\n");
		return 0;
	}

	CLK_UnLockReg(pll);
	pll->rate = CLK_SetPLLFreq(pll, parent_rate, rate);
	CLK_LockReg(pll);

	return 0;
}

static unsigned long ma35h0_clk_pll_recalc_rate(struct clk_hw *hw,
			unsigned long parent_rate)
{
	unsigned long pllfreq;
	struct ma35h0_clk_pll *pll = to_ma35h0_clk_pll(hw);

	if ((parent_rate < VSIPLL_FREF_MIN_FREQ)
	    || (parent_rate > VSIPLL_FREF_MAX_FREQ))
		return 0;

	switch (pll->type) {
	case MA35H0_CAPLL:
		pllfreq = CLK_GetPLLFreq_SMICPLL(pll, parent_rate);
		break;
	case MA35H0_DDRPLL:
	case MA35H0_APLL:
	case MA35H0_EPLL:
	case MA35H0_VPLL:
		pllfreq = CLK_GetPLLFreq_VSIPLL(pll, parent_rate);
		break;
	}

	return pllfreq;
}

static long ma35h0_clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	return rate;
}

static int ma35h0_clk_pll_is_prepared(struct clk_hw *hw)
{
	struct ma35h0_clk_pll *pll = to_ma35h0_clk_pll(hw);

	u32 val = __raw_readl(pll->ctl1_base);

	return (val & VSIPLLCTL1_PD_MSK) ? 0 : 1;
}

static int ma35h0_clk_pll_prepare(struct clk_hw *hw)
{
	struct ma35h0_clk_pll *pll = to_ma35h0_clk_pll(hw);
	u32 val;

	if ((pll->type == MA35H0_CAPLL) || (pll->type == MA35H0_DDRPLL)) {
		pr_warn("Nuvoton MA35H0 CAPLL/DDRPLL Enable Only.\n");
		return 0;
	}

	CLK_UnLockReg(pll);
	val = __raw_readl(pll->ctl1_base);
	val &= ~VSIPLLCTL1_PD_MSK;
	__raw_writel(val, pll->ctl1_base);
	CLK_LockReg(pll);

	return 0;
}

static void ma35h0_clk_pll_unprepare(struct clk_hw *hw)
{
	struct ma35h0_clk_pll *pll = to_ma35h0_clk_pll(hw);
	u32 val;

	if ((pll->type == MA35H0_CAPLL) || (pll->type == MA35H0_DDRPLL))
		pr_warn("Nuvoton MA35H0 CAPLL/DDRPLL Cannot disable.\n");
	else {
		val = __raw_readl(pll->ctl1_base);
		val |= VSIPLLCTL1_PD_MSK;
		__raw_writel(val, pll->ctl1_base);
	}
}

static const struct clk_ops ma35h0_clk_pll_ops = {
	.is_prepared = ma35h0_clk_pll_is_prepared,
	.prepare = ma35h0_clk_pll_prepare,
	.unprepare = ma35h0_clk_pll_unprepare,
	.set_rate = ma35h0_clk_pll_set_rate,
	.recalc_rate = ma35h0_clk_pll_recalc_rate,
	.round_rate = ma35h0_clk_pll_round_rate,
};

struct clk_hw *ma35h0_reg_clk_pll(enum ma35h0_pll_type type,
				u8 u8mode, const char *name,
				const char *parent, unsigned long targetFreq,
				void __iomem *base, struct regmap *regmap)
{
	struct ma35h0_clk_pll *pll;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	pll = kmalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->type = type;
	pll->mode = u8mode;
	pll->rate = targetFreq;
	pll->ctl0_base = base + VSIPLL_CTL0;
	pll->ctl1_base = base + VSIPLL_CTL1;
	pll->ctl2_base = base + VSIPLL_CTL2;
	pll->regmap = regmap;

	init.name = name;
	init.flags = 0;
	init.parent_names = &parent;
	init.num_parents = 1;

	init.ops = &ma35h0_clk_pll_ops;
	pll->hw.init = &init;

	hw = &pll->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		pr_err("failed to register vsi-pll clock!!!\n");
		kfree(pll);
		return ERR_PTR(ret);
	}

	return hw;
}
