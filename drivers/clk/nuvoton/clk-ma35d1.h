/*
 *
 * Copyright (c) 2020 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __MACH_MA35D1_CLK_CCF_H
#define __MACH_MA35D1_CLK_CCF_H

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/clk-provider.h>

enum ma35d1_pll_type {
	MA35D1_CAPLL,
	MA35D1_SYSPLL,
	MA35D1_DDRPLL,
	MA35D1_APLL,
	MA35D1_EPLL,
	MA35D1_VPLL,
};


extern struct clk *ma35d1_clk_pll(enum ma35d1_pll_type type, u8 u8mode, const char *name, const char *parent,
                                   unsigned long targetFreq, void __iomem *base);
extern spinlock_t ma35d1_lock;


static inline struct clk *ma35d1_clk_fixed(const char *name, int rate)
{

	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *ma35d1_clk_mux(const char *name,
        void __iomem *reg,
        u8 shift,
        u8 width, const char **parents,
        int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents, 0, reg,
	                        shift, width, 0, &ma35d1_lock);
}

static inline struct clk *ma35d1_clk_divider(const char *name,
        const char *parent,
        void __iomem *reg, u8 shift,
        u8 width)
{
	return clk_register_divider(NULL, name, parent, 0,
	                            reg, shift, width, 0, &ma35d1_lock);
}

static inline struct clk *ma35d1_clk_fixed_factor(const char *name,
        const char *parent,
        unsigned int mult,
        unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
	                                 CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk *ma35d1_clk_gate(const char *name,
        const char *parent,
        void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
	                         shift, 0, &ma35d1_lock);
}

#endif
