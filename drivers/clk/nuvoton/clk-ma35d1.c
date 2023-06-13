// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35D1 Clock Controller driver
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 *
 * Author: Chi-Fang Li <cfli0@nuvoton.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <dt-bindings/clock/nuvoton,ma35d1-clk.h>
#include "clk-ma35d1.h"

DEFINE_SPINLOCK(ma35d1_lock);

static const char *const ca35clk_sel_clks[] = {
	"hxt", "capll", "ddrpll", "dummy"
};

static const char *const sysclk0_sel_clks[] = {
	"epll_div2", "syspll"
};

static const char *const sysclk1_sel_clks[] = {
	"hxt", "syspll"
};

static const char *const axiclk_sel_clks[] = {
	"capll_div2", "capll_div4"
};

static const char *const ccap_sel_clks[] = {
	"hxt", "vpll", "apll", "syspll"
};

static const char *const sdh_sel_clks[] = {
	"syspll", "apll", "dummy", "dummy"
};

static const char *const dcu_sel_clks[] = {
	"epll_div2", "syspll"
};

static const char *const gfx_sel_clks[] = {
	"epll", "syspll"
};

static const char *const dbg_sel_clks[] = {
	"hirc", "syspll"
};

static const char *const timer0_sel_clks[] = {
	"hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer1_sel_clks[] = {
	"hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer2_sel_clks[] = {
	"hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer3_sel_clks[] = {
	"hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer4_sel_clks[] = {
	"hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer5_sel_clks[] = {
	"hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer6_sel_clks[] = {
	"hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer7_sel_clks[] = {
	"hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer8_sel_clks[] = {
	"hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer9_sel_clks[] = {
	"hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer10_sel_clks[] = {
	"hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const timer11_sel_clks[] = {
	"hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc"
};

static const char *const uart_sel_clks[] = {
	"hxt", "sysclk1_div2", "dummy", "dummy"
};

static const char *const wdt0_sel_clks[] = {
	"dummy", "lxt", "pclk3_div4096", "lirc"
};

static const char *const wdt1_sel_clks[] = {
	"dummy", "lxt", "pclk3_div4096", "lirc"
};

static const char *const wdt2_sel_clks[] = {
	"dummy", "lxt", "pclk4_div4096", "lirc"
};

static const char *const wwdt0_sel_clks[] = {
	"dummy", "dummy", "pclk3_div4096", "lirc"
};

static const char *const wwdt1_sel_clks[] = {
	"dummy", "dummy", "pclk3_div4096", "lirc"
};

static const char *const wwdt2_sel_clks[] = {
	"dummy", "dummy", "pclk4_div4096", "lirc"
};

static const char *const spi0_sel_clks[] = {
	"pclk1", "apll", "dummy", "dummy"
};

static const char *const spi1_sel_clks[] = {
	"pclk2", "apll", "dummy", "dummy"
};

static const char *const spi2_sel_clks[] = {
	"pclk1", "apll", "dummy", "dummy"
};

static const char *const spi3_sel_clks[] = {
	"pclk2", "apll", "dummy", "dummy"
};

static const char *const qspi0_sel_clks[] = {
	"pclk0", "apll", "dummy", "dummy"
};

static const char *const qspi1_sel_clks[] = {
	"pclk0", "apll", "dummy", "dummy"
};

static const char *const i2s0_sel_clks[] = {
	"apll", "sysclk1_div2", "dummy", "dummy"
};

static const char *const i2s1_sel_clks[] = {
	"apll", "sysclk1_div2", "dummy", "dummy"
};

static const char *const can_sel_clks[] = {
	"apll", "vpll"
};

static const char *const cko_sel_clks[] = {
	"hxt", "lxt", "hirc", "lirc", "capll_div4", "syspll",
	"ddrpll", "epll_div2", "apll", "vpll", "dummy", "dummy",
	"dummy", "dummy", "dummy", "dummy"
};

static const char *const smc_sel_clks[] = {
	"hxt", "pclk4"
};

static const char *const kpi_sel_clks[] = {
	"hxt", "lxt"
};

static const struct clk_div_table ip_div_table[] = {
	{0, 2}, {1, 4}, {2, 6}, {3, 8}, {4, 10},
	{5, 12}, {6, 14}, {7, 16}, {0, 0},
};

static const struct clk_div_table eadc_div_table[] = {
	{0, 2}, {1, 4}, {2, 6}, {3, 8}, {4, 10},
	{5, 12}, {6, 14}, {7, 16}, {8, 18},
	{9, 20}, {10, 22}, {11, 24}, {12, 26},
	{13, 28}, {14, 30}, {15, 32}, {0, 0},
};

static struct clk_hw **hws;
static struct clk_hw_onecell_data *ma35d1_hw_data;

static int ma35d1_clocks_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *clk_node = dev->of_node;
	void __iomem *clk_base;
	struct regmap *regmap;
	u32 pllmode[5] = { 0, 0, 0, 0, 0 };
	u32 pllfreq[5] = { 0, 0, 0, 0, 0 };

	dev_info(&pdev->dev, "Nuvoton MA35D1 Clock Driver\n");
	ma35d1_hw_data = devm_kzalloc(&pdev->dev,
			struct_size(ma35d1_hw_data, hws, MA35D1_CLK_MAX_IDX),
			GFP_KERNEL);

	if (WARN_ON(!ma35d1_hw_data))
		return -ENOMEM;

	ma35d1_hw_data->num = MA35D1_CLK_MAX_IDX;
	hws = ma35d1_hw_data->hws;

	clk_node = of_find_compatible_node(NULL, NULL, "nuvoton,ma35d1-clk");
	clk_base = of_iomap(clk_node, 0);
	of_node_put(clk_node);
	if (!clk_base) {
		pr_err("%s: could not map region\n", __func__);
		return -ENOMEM;
	}

	regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "nuvoton,sys");

	/* Clock sources */
	hws[HXT] = ma35d1_clk_fixed("hxt", 24000000);
	hws[HXT_GATE] =
	ma35d1_clk_gate("hxt_gate", "hxt", clk_base + REG_CLK_PWRCTL, 0);
	hws[LXT] = ma35d1_clk_fixed("lxt", 32768);
	hws[LXT_GATE] =
	ma35d1_clk_gate("lxt_gate", "lxt", clk_base + REG_CLK_PWRCTL, 1);
	hws[HIRC] = ma35d1_clk_fixed("hirc", 12000000);
	hws[HIRC_GATE] =
	ma35d1_clk_gate("hirc_gate", "hirc", clk_base + REG_CLK_PWRCTL, 2);
	hws[LIRC] = ma35d1_clk_fixed("lirc", 32000);
	hws[LIRC_GATE] =
	ma35d1_clk_gate("lirc_gate", "lirc", clk_base + REG_CLK_PWRCTL, 3);

	/* PLL */
	of_property_read_u32_array(clk_node, "clock-pll-mode", pllmode,
		ARRAY_SIZE(pllmode));
	of_property_read_u32_array(clk_node, "assigned-clock-rates", pllfreq,
		ARRAY_SIZE(pllfreq));

	/* SMIC PLL */
	hws[CAPLL] = ma35d1_reg_clk_pll(MA35D1_CAPLL, pllmode[0], "capll", "hxt",
		pllfreq[0], clk_base + REG_CLK_PLL0CTL0, regmap);

	hws[SYSPLL] = ma35d1_clk_fixed("syspll", 180000000);

	/* VSI PLL */
	hws[DDRPLL] = ma35d1_reg_clk_pll(MA35D1_DDRPLL, pllmode[1], "ddrpll", "hxt",
		pllfreq[1], clk_base + REG_CLK_PLL2CTL0, regmap);

	hws[APLL] = ma35d1_reg_clk_pll(MA35D1_APLL, pllmode[2], "apll", "hxt",
		pllfreq[2], clk_base + REG_CLK_PLL3CTL0, regmap);

	hws[EPLL] = ma35d1_reg_clk_pll(MA35D1_EPLL, pllmode[3], "epll", "hxt",
		pllfreq[3], clk_base + REG_CLK_PLL4CTL0, regmap);

	hws[VPLL] = ma35d1_reg_clk_pll(MA35D1_VPLL, pllmode[4], "vpll", "hxt",
		pllfreq[4], clk_base + REG_CLK_PLL5CTL0, regmap);

	hws[EPLL_DIV2] = ma35d1_clk_fixed_factor("epll_div2", "epll", 1, 2);
	hws[EPLL_DIV4] = ma35d1_clk_fixed_factor("epll_div4", "epll", 1, 4);
	hws[EPLL_DIV8] = ma35d1_clk_fixed_factor("epll_div8", "epll", 1, 8);

	/* CA35 */
	hws[CA35CLK_MUX] = ma35d1_clk_mux("ca35clk_mux", clk_base + REG_CLK_CLKSEL0, 0,
		2, ca35clk_sel_clks, ARRAY_SIZE(ca35clk_sel_clks));

	/* AXI */
	hws[AXICLK_DIV2] = ma35d1_clk_fixed_factor("capll_div2", "ca35clk_mux", 1, 2);
	hws[AXICLK_DIV4] = ma35d1_clk_fixed_factor("capll_div4", "ca35clk_mux", 1, 4);
	hws[AXICLK_MUX] = ma35d1_clk_mux("axiclk_mux", clk_base + REG_CLK_CLKDIV0, 26,
		1, axiclk_sel_clks, ARRAY_SIZE(axiclk_sel_clks));

	/* SYSCLK0 & SYSCLK1 */
	hws[SYSCLK0_MUX] = ma35d1_clk_mux("sysclk0_mux", clk_base + REG_CLK_CLKSEL0, 2,
		1, sysclk0_sel_clks, ARRAY_SIZE(sysclk0_sel_clks));

	hws[SYSCLK1_MUX] = ma35d1_clk_mux("sysclk1_mux", clk_base + REG_CLK_CLKSEL0, 4,
		1, sysclk1_sel_clks, ARRAY_SIZE(sysclk1_sel_clks));
	hws[SYSCLK1_DIV2] = ma35d1_clk_fixed_factor("sysclk1_div2", "sysclk1_mux", 1,
		2);

	/* HCLK0~3 & PCLK0~4 */
	hws[HCLK0] = ma35d1_clk_fixed_factor("hclk0", "sysclk1_mux", 1, 1);
	hws[HCLK1] = ma35d1_clk_fixed_factor("hclk1", "sysclk1_mux", 1, 1);
	hws[HCLK2] = ma35d1_clk_fixed_factor("hclk2", "sysclk1_mux", 1, 1);
	hws[PCLK0] = ma35d1_clk_fixed_factor("pclk0", "sysclk1_mux", 1, 1);
	hws[PCLK1] = ma35d1_clk_fixed_factor("pclk1", "sysclk1_mux", 1, 1);
	hws[PCLK2] = ma35d1_clk_fixed_factor("pclk2", "sysclk1_mux", 1, 1);

	hws[HCLK3] = ma35d1_clk_fixed_factor("hclk3", "sysclk1_mux", 1, 2);
	hws[PCLK3] = ma35d1_clk_fixed_factor("pclk3", "sysclk1_mux", 1, 2);
	hws[PCLK4] = ma35d1_clk_fixed_factor("pclk4", "sysclk1_mux", 1, 2);

	hws[USBPHY0] = ma35d1_clk_fixed("usbphy0", 480000000);
	hws[USBPHY1] = ma35d1_clk_fixed("usbphy1", 480000000);

	/* DDR */
	hws[DDR0_GATE] = ma35d1_clk_gate("ddr0_gate", "ddrpll",
		clk_base + REG_CLK_SYSCLK0, 4);
	hws[DDR6_GATE] = ma35d1_clk_gate("ddr6_gate", "ddrpll",
		clk_base + REG_CLK_SYSCLK0, 5);

	/* CAN0 */
	hws[CAN0_MUX] = ma35d1_clk_mux("can0_mux", clk_base + REG_CLK_CLKSEL4, 16, 1,
		can_sel_clks, ARRAY_SIZE(can_sel_clks));
	hws[CAN0_DIV] = ma35d1_clk_divider_table("can0_div", "can0_mux",
		clk_base + REG_CLK_CLKDIV0, 0, 3, ip_div_table);
	hws[CAN0_GATE] = ma35d1_clk_gate("can0_gate", "can0_div",
		clk_base + REG_CLK_SYSCLK0, 8);

	/* CAN1 */
	hws[CAN1_MUX] = ma35d1_clk_mux("can1_mux", clk_base + REG_CLK_CLKSEL4, 17, 1,
		can_sel_clks, ARRAY_SIZE(can_sel_clks));
	hws[CAN1_DIV] = ma35d1_clk_divider_table("can1_div", "can1_mux",
		clk_base + REG_CLK_CLKDIV0, 4, 3, ip_div_table);
	hws[CAN1_GATE] = ma35d1_clk_gate("can1_gate", "can1_div",
		clk_base + REG_CLK_SYSCLK0, 9);

	/* CAN2 */
	hws[CAN2_MUX] = ma35d1_clk_mux("can2_mux", clk_base + REG_CLK_CLKSEL4, 18, 1,
		can_sel_clks, ARRAY_SIZE(can_sel_clks));
	hws[CAN2_DIV] = ma35d1_clk_divider_table("can2_div", "can2_mux",
		clk_base + REG_CLK_CLKDIV0, 8, 3, ip_div_table);
	hws[CAN2_GATE] = ma35d1_clk_gate("can2_gate", "can2_div",
		 clk_base + REG_CLK_SYSCLK0, 10);

	/* CAN3 */
	hws[CAN3_MUX] = ma35d1_clk_mux("can3_mux", clk_base + REG_CLK_CLKSEL4, 19, 1,
		can_sel_clks, ARRAY_SIZE(can_sel_clks));
	hws[CAN3_DIV] = ma35d1_clk_divider_table("can3_div", "can3_mux",
		clk_base + REG_CLK_CLKDIV0, 12, 3, ip_div_table);
	hws[CAN3_GATE] = ma35d1_clk_gate("can3_gate", "can3_div",
		clk_base + REG_CLK_SYSCLK0, 11);

	/* SDH0 */
	hws[SDH0_MUX] = ma35d1_clk_mux("sdh0_mux", clk_base + REG_CLK_CLKSEL0, 16, 2,
		sdh_sel_clks, ARRAY_SIZE(sdh_sel_clks));
	hws[SDH0_GATE] = ma35d1_clk_gate("sdh0_gate", "sdh0_mux",
		clk_base + REG_CLK_SYSCLK0, 16);

	/* SDH1 */
	hws[SDH1_MUX] = ma35d1_clk_mux("sdh1_mux", clk_base + REG_CLK_CLKSEL0, 18, 2,
		sdh_sel_clks, ARRAY_SIZE(sdh_sel_clks));
	hws[SDH1_GATE] = ma35d1_clk_gate("sdh1_gate", "sdh1_mux",
		clk_base + REG_CLK_SYSCLK0, 17);

	/* NAND */
	hws[NAND_GATE] = ma35d1_clk_gate("nand_gate", "hclk1",
		clk_base + REG_CLK_SYSCLK0, 18);

	/* USB */
	hws[USBD_GATE] = ma35d1_clk_gate("usbd_gate", "usbphy0",
		clk_base + REG_CLK_SYSCLK0, 19);
	hws[USBH_GATE] = ma35d1_clk_gate("usbh_gate", "usbphy0",
		clk_base + REG_CLK_SYSCLK0, 20);
	hws[HUSBH0_GATE] = ma35d1_clk_gate("husbh0_gate", "usbphy0",
		clk_base + REG_CLK_SYSCLK0, 21);
	hws[HUSBH1_GATE] = ma35d1_clk_gate("husbh1_gate", "usbphy0",
		clk_base + REG_CLK_SYSCLK0, 22);

	/* GFX */
	hws[GFX_MUX] = ma35d1_clk_mux("gfx_mux", clk_base + REG_CLK_CLKSEL0, 26, 1,
		gfx_sel_clks, ARRAY_SIZE(gfx_sel_clks));
	hws[GFX_GATE] = ma35d1_clk_gate("gfx_gate", "gfx_mux",
		clk_base + REG_CLK_SYSCLK0, 24);

	/* VC8K */
	hws[VC8K_GATE] = ma35d1_clk_gate("vc8k_gate", "sysclk0_mux",
		clk_base + REG_CLK_SYSCLK0, 25);

	/* DCU */
	hws[DCU_MUX] = ma35d1_clk_mux("dcu_mux", clk_base + REG_CLK_CLKSEL0, 24, 1,
		dcu_sel_clks, ARRAY_SIZE(dcu_sel_clks));
	hws[DCU_GATE] = ma35d1_clk_gate("dcu_gate", "dcu_mux",
		clk_base + REG_CLK_SYSCLK0, 26);

	/* DCUP */
	hws[DCUP_DIV] = ma35d1_clk_divider_table("dcup_div", "vpll",
		clk_base + REG_CLK_CLKDIV0, 16, 3, ip_div_table);

	/* EMAC0 */
	hws[EMAC0_GATE] = ma35d1_clk_gate("emac0_gate", "epll_div2",
		clk_base + REG_CLK_SYSCLK0, 27);

	/* EMAC1 */
	hws[EMAC1_GATE] = ma35d1_clk_gate("emac1_gate", "epll_div2",
		clk_base + REG_CLK_SYSCLK0, 28);

	/* CCAP0 */
	hws[CCAP0_MUX] = ma35d1_clk_mux("ccap0_mux", clk_base + REG_CLK_CLKSEL0, 12, 1,
		ccap_sel_clks, ARRAY_SIZE(ccap_sel_clks));
	hws[CCAP0_DIV] = ma35d1_clk_divider("ccap0_div", "ccap0_mux",
		clk_base + REG_CLK_CLKDIV1, 8, 4);
	hws[CCAP0_GATE] = ma35d1_clk_gate("ccap0_gate", "ccap0_div",
		clk_base + REG_CLK_SYSCLK0, 29);

	/* CCAP1 */
	hws[CCAP1_MUX] = ma35d1_clk_mux("ccap1_mux", clk_base + REG_CLK_CLKSEL0, 14, 1,
		ccap_sel_clks, ARRAY_SIZE(ccap_sel_clks));
	hws[CCAP1_DIV] = ma35d1_clk_divider("ccap1_div", "ccap1_mux",
		clk_base + REG_CLK_CLKDIV1, 12, 4);
	hws[CCAP1_GATE] = ma35d1_clk_gate("ccap1_gate", "ccap1_div",
		clk_base + REG_CLK_SYSCLK0, 30);

	/* PDMA0~3 */
	hws[PDMA0_GATE] = ma35d1_clk_gate("pdma0_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 0);
	hws[PDMA1_GATE] = ma35d1_clk_gate("pdma1_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 1);
	hws[PDMA2_GATE] = ma35d1_clk_gate("pdma2_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 2);
	hws[PDMA3_GATE] = ma35d1_clk_gate("pdma3_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 3);

	/* WH0~1 */
	hws[WH0_GATE] = ma35d1_clk_gate("wh0_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		4);
	hws[WH1_GATE] = ma35d1_clk_gate("wh1_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		5);

	/* HWS */
	hws[HWS_GATE] = ma35d1_clk_gate("hws_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		6);

	/* EBI */
	hws[EBI_GATE] = ma35d1_clk_gate("ebi_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		7);

	/* SRAM0~1 */
	hws[SRAM0_GATE] = ma35d1_clk_gate("sram0_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 8);
	hws[SRAM1_GATE] = ma35d1_clk_gate("sram1_gate", "hclk0",
		clk_base + REG_CLK_SYSCLK1, 9);

	/* ROM */
	hws[ROM_GATE] = ma35d1_clk_gate("rom_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		10);

	/* TRA */
	hws[TRA_GATE] = ma35d1_clk_gate("tra_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		11);

	/* DBG */
	hws[DBG_MUX] = ma35d1_clk_mux("dbg_mux", clk_base + REG_CLK_CLKSEL0, 27, 1,
		dbg_sel_clks, ARRAY_SIZE(dbg_sel_clks));
	hws[DBG_GATE] = ma35d1_clk_gate("dbg_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		12);

	/* CLKO */
	hws[CKO_MUX] = ma35d1_clk_mux("cko_mux", clk_base + REG_CLK_CLKSEL4, 24, 4,
		cko_sel_clks, ARRAY_SIZE(cko_sel_clks));
	hws[CKO_DIV] = ma35d1_clk_divider_pow2("cko_div", "cko_mux",
		clk_base + REG_CLK_CLKOCTL, 0, 4);
	hws[CKO_GATE] = ma35d1_clk_gate("cko_gate", "cko_div",
		clk_base + REG_CLK_SYSCLK1, 13);

	/* GTMR */
	hws[GTMR_GATE] = ma35d1_clk_gate("gtmr_gate", "hirc",
		clk_base + REG_CLK_SYSCLK1, 14);

	/* GPIO */
	hws[GPA_GATE] = ma35d1_clk_gate("gpa_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		16);
	hws[GPB_GATE] = ma35d1_clk_gate("gpb_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		17);
	hws[GPC_GATE] = ma35d1_clk_gate("gpc_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		18);
	hws[GPD_GATE] = ma35d1_clk_gate("gpd_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		19);
	hws[GPE_GATE] = ma35d1_clk_gate("gpe_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		20);
	hws[GPF_GATE] = ma35d1_clk_gate("gpf_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		21);
	hws[GPG_GATE] = ma35d1_clk_gate("gpg_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		22);
	hws[GPH_GATE] = ma35d1_clk_gate("gph_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		23);
	hws[GPI_GATE] = ma35d1_clk_gate("gpi_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		24);
	hws[GPJ_GATE] = ma35d1_clk_gate("gpj_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		25);
	hws[GPK_GATE] = ma35d1_clk_gate("gpk_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		26);
	hws[GPL_GATE] = ma35d1_clk_gate("gpl_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		27);
	hws[GPM_GATE] = ma35d1_clk_gate("gpm_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		28);
	hws[GPN_GATE] = ma35d1_clk_gate("gpn_gate", "hclk0", clk_base + REG_CLK_SYSCLK1,
		29);

	/* TIMER0~11 */
	hws[TMR0_MUX] = ma35d1_clk_mux("tmr0_mux", clk_base + REG_CLK_CLKSEL1, 0, 3,
		timer0_sel_clks, ARRAY_SIZE(timer0_sel_clks));
	hws[TMR0_GATE] = ma35d1_clk_gate("tmr0_gate", "tmr0_mux",
		clk_base + REG_CLK_APBCLK0, 0);
	hws[TMR1_MUX] = ma35d1_clk_mux("tmr1_mux", clk_base + REG_CLK_CLKSEL1, 4, 3,
		timer1_sel_clks, ARRAY_SIZE(timer1_sel_clks));
	hws[TMR1_GATE] = ma35d1_clk_gate("tmr1_gate", "tmr1_mux",
		clk_base + REG_CLK_APBCLK0, 1);
	hws[TMR2_MUX] = ma35d1_clk_mux("tmr2_mux", clk_base + REG_CLK_CLKSEL1, 8, 3,
		timer2_sel_clks, ARRAY_SIZE(timer2_sel_clks));
	hws[TMR2_GATE] = ma35d1_clk_gate("tmr2_gate", "tmr2_mux",
		clk_base + REG_CLK_APBCLK0, 2);
	hws[TMR3_MUX] = ma35d1_clk_mux("tmr3_mux", clk_base + REG_CLK_CLKSEL1, 12, 3,
		timer3_sel_clks, ARRAY_SIZE(timer3_sel_clks));
	hws[TMR3_GATE] = ma35d1_clk_gate("tmr3_gate", "tmr3_mux",
		clk_base + REG_CLK_APBCLK0, 3);
	hws[TMR4_MUX] = ma35d1_clk_mux("tmr4_mux", clk_base + REG_CLK_CLKSEL1, 16, 3,
		timer4_sel_clks, ARRAY_SIZE(timer4_sel_clks));
	hws[TMR4_GATE] = ma35d1_clk_gate("tmr4_gate", "tmr4_mux",
		clk_base + REG_CLK_APBCLK0, 4);
	hws[TMR5_MUX] = ma35d1_clk_mux("tmr5_mux", clk_base + REG_CLK_CLKSEL1, 20, 3,
		timer5_sel_clks, ARRAY_SIZE(timer5_sel_clks));
	hws[TMR5_GATE] = ma35d1_clk_gate("tmr5_gate", "tmr5_mux",
		clk_base + REG_CLK_APBCLK0, 5);
	hws[TMR6_MUX] = ma35d1_clk_mux("tmr6_mux", clk_base + REG_CLK_CLKSEL1, 24, 3,
		timer6_sel_clks, ARRAY_SIZE(timer6_sel_clks));
	hws[TMR6_GATE] = ma35d1_clk_gate("tmr6_gate", "tmr6_mux",
		clk_base + REG_CLK_APBCLK0, 6);
	hws[TMR7_MUX] = ma35d1_clk_mux("tmr7_mux", clk_base + REG_CLK_CLKSEL1, 28, 3,
		timer7_sel_clks, ARRAY_SIZE(timer7_sel_clks));
	hws[TMR7_GATE] = ma35d1_clk_gate("tmr7_gate", "tmr7_mux",
		clk_base + REG_CLK_APBCLK0, 7);
	hws[TMR8_MUX] = ma35d1_clk_mux("tmr8_mux", clk_base + REG_CLK_CLKSEL2, 0, 3,
		timer8_sel_clks, ARRAY_SIZE(timer8_sel_clks));
	hws[TMR8_GATE] = ma35d1_clk_gate("tmr8_gate", "tmr8_mux",
		clk_base + REG_CLK_APBCLK0, 8);
	hws[TMR9_MUX] = ma35d1_clk_mux("tmr9_mux", clk_base + REG_CLK_CLKSEL2, 4, 3,
		timer9_sel_clks, ARRAY_SIZE(timer9_sel_clks));
	hws[TMR9_GATE] = ma35d1_clk_gate("tmr9_gate", "tmr9_mux",
		clk_base + REG_CLK_APBCLK0, 9);
	hws[TMR10_MUX] = ma35d1_clk_mux("tmr10_mux", clk_base + REG_CLK_CLKSEL2, 8, 3,
		timer10_sel_clks, ARRAY_SIZE(timer10_sel_clks));
	hws[TMR10_GATE] = ma35d1_clk_gate("tmr10_gate", "tmr10_mux",
		clk_base + REG_CLK_APBCLK0, 10);
	hws[TMR11_MUX] = ma35d1_clk_mux("tmr11_mux", clk_base + REG_CLK_CLKSEL2, 12, 3,
		timer11_sel_clks, ARRAY_SIZE(timer11_sel_clks));
	hws[TMR11_GATE] = ma35d1_clk_gate("tmr11_gate", "tmr11_mux",
		clk_base + REG_CLK_APBCLK0, 11);

	/* UART0~16 */
	hws[UART0_MUX] = ma35d1_clk_mux("uart0_mux", clk_base + REG_CLK_CLKSEL2, 16, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART0_DIV] = ma35d1_clk_divider("uart0_div", "uart0_mux",
		clk_base + REG_CLK_CLKDIV1, 16, 4);
	hws[UART0_GATE] = ma35d1_clk_gate("uart0_gate", "uart0_div",
		clk_base + REG_CLK_APBCLK0, 12);
	hws[UART1_MUX] = ma35d1_clk_mux("uart1_mux", clk_base + REG_CLK_CLKSEL2, 18, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART1_DIV] = ma35d1_clk_divider("uart1_div", "uart1_mux",
		clk_base + REG_CLK_CLKDIV1, 20, 4);
	hws[UART1_GATE] = ma35d1_clk_gate("uart1_gate", "uart1_div",
		clk_base + REG_CLK_APBCLK0, 13);
	hws[UART2_MUX] = ma35d1_clk_mux("uart2_mux", clk_base + REG_CLK_CLKSEL2, 20, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART2_DIV] = ma35d1_clk_divider("uart2_div", "uart2_mux",
		clk_base + REG_CLK_CLKDIV1, 24, 4);
	hws[UART2_GATE] = ma35d1_clk_gate("uart2_gate", "uart2_div",
		clk_base + REG_CLK_APBCLK0, 14);
	hws[UART3_MUX] = ma35d1_clk_mux("uart3_mux", clk_base + REG_CLK_CLKSEL2, 22, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART3_DIV] = ma35d1_clk_divider("uart3_div", "uart3_mux",
		clk_base + REG_CLK_CLKDIV1, 28, 4);
	hws[UART3_GATE] = ma35d1_clk_gate("uart3_gate", "uart3_div",
		clk_base + REG_CLK_APBCLK0, 15);
	hws[UART4_MUX] = ma35d1_clk_mux("uart4_mux", clk_base + REG_CLK_CLKSEL2, 24, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART4_DIV] = ma35d1_clk_divider("uart4_div", "uart4_mux",
		clk_base + REG_CLK_CLKDIV2, 0, 4);
	hws[UART4_GATE] = ma35d1_clk_gate("uart4_gate", "uart4_div",
		clk_base + REG_CLK_APBCLK0, 16);
	hws[UART5_MUX] = ma35d1_clk_mux("uart5_mux", clk_base + REG_CLK_CLKSEL2, 26, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART5_DIV] = ma35d1_clk_divider("uart5_div", "uart5_mux",
		clk_base + REG_CLK_CLKDIV2, 4, 4);
	hws[UART5_GATE] = ma35d1_clk_gate("uart5_gate", "uart5_div",
		clk_base + REG_CLK_APBCLK0, 17);
	hws[UART6_MUX] = ma35d1_clk_mux("uart6_mux", clk_base + REG_CLK_CLKSEL2, 28, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART6_DIV] = ma35d1_clk_divider("uart6_div", "uart6_mux",
		clk_base + REG_CLK_CLKDIV2, 8, 4);
	hws[UART6_GATE] = ma35d1_clk_gate("uart6_gate", "uart6_div",
		clk_base + REG_CLK_APBCLK0, 18);
	hws[UART7_MUX] = ma35d1_clk_mux("uart7_mux", clk_base + REG_CLK_CLKSEL2, 30, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART7_DIV] = ma35d1_clk_divider("uart7_div", "uart7_mux",
		clk_base + REG_CLK_CLKDIV2, 12, 4);
	hws[UART7_GATE] = ma35d1_clk_gate("uart7_gate", "uart7_div",
		clk_base + REG_CLK_APBCLK0, 19);
	hws[UART8_MUX] = ma35d1_clk_mux("uart8_mux", clk_base + REG_CLK_CLKSEL3, 0, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART8_DIV] = ma35d1_clk_divider("uart8_div", "uart8_mux",
		clk_base + REG_CLK_CLKDIV2, 16, 4);
	hws[UART8_GATE] = ma35d1_clk_gate("uart8_gate", "uart8_div",
		clk_base + REG_CLK_APBCLK0, 20);
	hws[UART9_MUX] = ma35d1_clk_mux("uart9_mux", clk_base + REG_CLK_CLKSEL3, 2, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART9_DIV] = ma35d1_clk_divider("uart9_div", "uart9_mux",
		clk_base + REG_CLK_CLKDIV2, 20, 4);
	hws[UART9_GATE] = ma35d1_clk_gate("uart9_gate", "uart9_div",
		clk_base + REG_CLK_APBCLK0, 21);
	hws[UART10_MUX] = ma35d1_clk_mux("uart10_mux", clk_base + REG_CLK_CLKSEL3, 4, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART10_DIV] = ma35d1_clk_divider("uart10_div", "uart10_mux",
		clk_base + REG_CLK_CLKDIV2, 24, 4);
	hws[UART10_GATE] = ma35d1_clk_gate("uart10_gate", "uart10_div",
		clk_base + REG_CLK_APBCLK0, 22);
	hws[UART11_MUX] = ma35d1_clk_mux("uart11_mux", clk_base + REG_CLK_CLKSEL3, 6, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART11_DIV] = ma35d1_clk_divider("uart11_div", "uart11_mux",
		clk_base + REG_CLK_CLKDIV2, 28, 4);
	hws[UART11_GATE] = ma35d1_clk_gate("uart11_gate", "uart11_div",
		clk_base + REG_CLK_APBCLK0, 23);
	hws[UART12_MUX] = ma35d1_clk_mux("uart12_mux", clk_base + REG_CLK_CLKSEL3, 8, 2,
		uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART12_DIV] = ma35d1_clk_divider("uart12_div", "uart12_mux",
		clk_base + REG_CLK_CLKDIV3, 0, 4);
	hws[UART12_GATE] = ma35d1_clk_gate("uart12_gate", "uart12_div",
		clk_base + REG_CLK_APBCLK0, 24);
	hws[UART13_MUX] = ma35d1_clk_mux("uart13_mux", clk_base + REG_CLK_CLKSEL3, 10,
		 2, uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART13_DIV] = ma35d1_clk_divider("uart13_div", "uart13_mux",
		clk_base + REG_CLK_CLKDIV3, 4, 4);
	hws[UART13_GATE] = ma35d1_clk_gate("uart13_gate", "uart13_div",
		clk_base + REG_CLK_APBCLK0, 25);
	hws[UART14_MUX] = ma35d1_clk_mux("uart14_mux", clk_base + REG_CLK_CLKSEL3, 12,
		2, uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART14_DIV] = ma35d1_clk_divider("uart14_div", "uart14_mux",
		clk_base + REG_CLK_CLKDIV3, 8, 4);
	hws[UART14_GATE] = ma35d1_clk_gate("uart14_gate", "uart14_div",
		clk_base + REG_CLK_APBCLK0, 26);
	hws[UART15_MUX] = ma35d1_clk_mux("uart15_mux", clk_base + REG_CLK_CLKSEL3, 14,
		2, uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART15_DIV] = ma35d1_clk_divider("uart15_div", "uart15_mux",
		clk_base + REG_CLK_CLKDIV3, 12, 4);
	hws[UART15_GATE] = ma35d1_clk_gate("uart15_gate", "uart15_div",
		clk_base + REG_CLK_APBCLK0, 27);
	hws[UART16_MUX] = ma35d1_clk_mux("uart16_mux", clk_base + REG_CLK_CLKSEL3, 16,
		2, uart_sel_clks, ARRAY_SIZE(uart_sel_clks));
	hws[UART16_DIV] = ma35d1_clk_divider("uart16_div", "uart16_mux",
		clk_base + REG_CLK_CLKDIV3, 16, 4);
	hws[UART16_GATE] = ma35d1_clk_gate("uart16_gate", "uart16_div",
		clk_base + REG_CLK_APBCLK0, 28);

	/* RTC */
	hws[RTC_GATE] = ma35d1_clk_gate("rtc_gate", "lxt", clk_base + REG_CLK_APBCLK0,
		29);

	/* DDRP */
	hws[DDR_GATE] = ma35d1_clk_gate("ddr_gate", "ddrpll",
		clk_base + REG_CLK_APBCLK0, 30);

	/* KPI */
	hws[KPI_MUX] = ma35d1_clk_mux("kpi_mux", clk_base + REG_CLK_CLKSEL4, 30, 1,
		kpi_sel_clks, ARRAY_SIZE(kpi_sel_clks));
	hws[KPI_DIV] = ma35d1_clk_divider("kpi_div", "kpi_mux",
		clk_base + REG_CLK_CLKDIV4, 24, 8);
	hws[KPI_GATE] = ma35d1_clk_gate("kpi_gate", "kpi_div",
		clk_base + REG_CLK_APBCLK0, 31);

	/* I2C0~5 */
	hws[I2C0_GATE] = ma35d1_clk_gate("i2c0_gate", "pclk0",
		clk_base + REG_CLK_APBCLK1, 0);
	hws[I2C1_GATE] = ma35d1_clk_gate("i2c1_gate", "pclk1",
		clk_base + REG_CLK_APBCLK1, 1);
	hws[I2C2_GATE] = ma35d1_clk_gate("i2c2_gate", "pclk2",
		clk_base + REG_CLK_APBCLK1, 2);
	hws[I2C3_GATE] = ma35d1_clk_gate("i2c3_gate", "pclk0",
		clk_base + REG_CLK_APBCLK1, 3);
	hws[I2C4_GATE] = ma35d1_clk_gate("i2c4_gate", "pclk1",
		clk_base + REG_CLK_APBCLK1, 4);
	hws[I2C5_GATE] = ma35d1_clk_gate("i2c5_gate", "pclk2",
		clk_base + REG_CLK_APBCLK1, 5);

	/* QSPI0~1 */
	hws[QSPI0_MUX] = ma35d1_clk_mux("qspi0_mux", clk_base + REG_CLK_CLKSEL4, 8, 2,
		qspi0_sel_clks, ARRAY_SIZE(qspi0_sel_clks));
	hws[QSPI0_GATE] = ma35d1_clk_gate("qspi0_gate", "qspi0_mux",
		clk_base + REG_CLK_APBCLK1, 6);
	hws[QSPI1_MUX] = ma35d1_clk_mux("qspi1_mux", clk_base + REG_CLK_CLKSEL4, 10, 2,
		qspi1_sel_clks, ARRAY_SIZE(qspi1_sel_clks));
	hws[QSPI1_GATE] = ma35d1_clk_gate("qspi1_gate", "qspi1_mux",
		clk_base + REG_CLK_APBCLK1, 7);

	/* SMC0~1 */
	hws[SMC0_MUX] = ma35d1_clk_mux("smc0_mux", clk_base + REG_CLK_CLKSEL4, 28, 1,
		smc_sel_clks, ARRAY_SIZE(smc_sel_clks));
	hws[SMC0_DIV] = ma35d1_clk_divider("smc0_div", "smc0_mux",
		clk_base + REG_CLK_CLKDIV1, 0, 4);
	hws[SMC0_GATE] = ma35d1_clk_gate("smc0_gate", "smc0_div",
		clk_base + REG_CLK_APBCLK1, 12);

	hws[SMC1_MUX] = ma35d1_clk_mux("smc1_mux", clk_base + REG_CLK_CLKSEL4, 29, 1,
		smc_sel_clks, ARRAY_SIZE(smc_sel_clks));
	hws[SMC1_DIV] = ma35d1_clk_divider("smc1_div", "smc1_mux",
		clk_base + REG_CLK_CLKDIV1, 4, 4);
	hws[SMC1_GATE] = ma35d1_clk_gate("smc1_gate", "smc1_div",
		clk_base + REG_CLK_APBCLK1, 13);

	/* WDT0~2 */
	hws[WDT0_MUX] = ma35d1_clk_mux("wdt0_mux", clk_base + REG_CLK_CLKSEL3, 20, 2,
		wdt0_sel_clks, ARRAY_SIZE(wdt0_sel_clks));
	hws[WDT0_GATE] = ma35d1_clk_gate("wdt0_gate", "wdt0_mux",
		clk_base + REG_CLK_APBCLK1, 16);
	hws[WDT1_MUX] = ma35d1_clk_mux("wdt1_mux", clk_base + REG_CLK_CLKSEL3, 24, 2,
		wdt1_sel_clks, ARRAY_SIZE(wdt1_sel_clks));
	hws[WDT1_GATE] = ma35d1_clk_gate("wdt1_gate", "wdt1_mux",
		clk_base + REG_CLK_APBCLK1, 17);
	hws[WDT2_MUX] = ma35d1_clk_mux("wdt2_mux", clk_base + REG_CLK_CLKSEL3, 28, 2,
		wdt2_sel_clks, ARRAY_SIZE(wdt2_sel_clks));
	hws[WDT2_GATE] = ma35d1_clk_gate("wdt2_gate", "wdt2_mux",
		clk_base + REG_CLK_APBCLK1, 18);

	/* WWDT0~2 */
	hws[WWDT0_MUX] = ma35d1_clk_mux("wwdt0_mux", clk_base + REG_CLK_CLKSEL3, 22, 2,
		wwdt0_sel_clks, ARRAY_SIZE(wwdt0_sel_clks));
	hws[WWDT1_MUX] = ma35d1_clk_mux("wwdt1_mux", clk_base + REG_CLK_CLKSEL3, 26, 2,
		wwdt1_sel_clks, ARRAY_SIZE(wwdt1_sel_clks));
	hws[WWDT2_MUX] = ma35d1_clk_mux("wwdt2_mux", clk_base + REG_CLK_CLKSEL3, 30, 2,
		wwdt2_sel_clks, ARRAY_SIZE(wwdt2_sel_clks));

	/* EPWM0~2 */
	hws[EPWM0_GATE] = ma35d1_clk_gate("epwm0_gate", "pclk1",
		clk_base + REG_CLK_APBCLK1, 24);
	hws[EPWM1_GATE] = ma35d1_clk_gate("epwm1_gate", "pclk2",
		clk_base + REG_CLK_APBCLK1, 25);
	hws[EPWM2_GATE] = ma35d1_clk_gate("epwm2_gate", "pclk1",
		clk_base + REG_CLK_APBCLK1, 26);

	/* I2S0~1 */
	hws[I2S0_MUX] = ma35d1_clk_mux("i2s0_mux", clk_base + REG_CLK_CLKSEL4, 12, 2,
		i2s0_sel_clks, ARRAY_SIZE(i2s0_sel_clks));
	hws[I2S0_GATE] = ma35d1_clk_gate("i2s0_gate", "i2s0_mux",
		clk_base + REG_CLK_APBCLK2, 0);
	hws[I2S1_MUX] = ma35d1_clk_mux("i2s1_mux", clk_base + REG_CLK_CLKSEL4, 14, 2,
		i2s1_sel_clks, ARRAY_SIZE(i2s1_sel_clks));
	hws[I2S1_GATE] = ma35d1_clk_gate("i2s1_gate", "i2s1_mux",
		clk_base + REG_CLK_APBCLK2, 1);

	/* SSMCC */
	hws[SSMCC_GATE] = ma35d1_clk_gate("ssmcc_gate", "pclk3",
		clk_base + REG_CLK_APBCLK2, 2);

	/* SSPCC */
	hws[SSPCC_GATE] = ma35d1_clk_gate("sspcc_gate", "pclk3",
		clk_base + REG_CLK_APBCLK2, 3);

	/* SPI0~3 */
	hws[SPI0_MUX] = ma35d1_clk_mux("spi0_mux", clk_base + REG_CLK_CLKSEL4, 0, 2,
		spi0_sel_clks, ARRAY_SIZE(spi0_sel_clks));
	hws[SPI0_GATE] = ma35d1_clk_gate("spi0_gate", "spi0_mux",
		clk_base + REG_CLK_APBCLK2, 4);
	hws[SPI1_MUX] = ma35d1_clk_mux("spi1_mux", clk_base + REG_CLK_CLKSEL4, 2, 2,
		spi1_sel_clks, ARRAY_SIZE(spi1_sel_clks));
	hws[SPI1_GATE] = ma35d1_clk_gate("spi1_gate", "spi1_mux",
		clk_base + REG_CLK_APBCLK2, 5);
	hws[SPI2_MUX] = ma35d1_clk_mux("spi2_mux", clk_base + REG_CLK_CLKSEL4, 4, 2,
		spi2_sel_clks, ARRAY_SIZE(spi2_sel_clks));
	hws[SPI2_GATE] = ma35d1_clk_gate("spi2_gate", "spi2_mux",
		clk_base + REG_CLK_APBCLK2, 6);
	hws[SPI3_MUX] = ma35d1_clk_mux("spi3_mux", clk_base + REG_CLK_CLKSEL4, 6, 2,
		spi3_sel_clks, ARRAY_SIZE(spi3_sel_clks));
	hws[SPI3_GATE] = ma35d1_clk_gate("spi3_gate", "spi3_mux",
		clk_base + REG_CLK_APBCLK2, 7);

	/* ECAP0~2 */
	hws[ECAP0_GATE] = ma35d1_clk_gate("ecap0_gate", "pclk1",
		clk_base + REG_CLK_APBCLK2, 8);
	hws[ECAP1_GATE] = ma35d1_clk_gate("ecap1_gate", "pclk2",
		clk_base + REG_CLK_APBCLK2, 9);
	hws[ECAP2_GATE] = ma35d1_clk_gate("ecap2_gate", "pclk1",
		clk_base + REG_CLK_APBCLK2, 10);

	/* QEI0~2 */
	hws[QEI0_GATE] = ma35d1_clk_gate("qei0_gate", "pclk1",
		clk_base + REG_CLK_APBCLK2, 12);
	hws[QEI1_GATE] = ma35d1_clk_gate("qei1_gate", "pclk2",
		clk_base + REG_CLK_APBCLK2, 13);
	hws[QEI2_GATE] = ma35d1_clk_gate("qei2_gate", "pclk1",
		clk_base + REG_CLK_APBCLK2, 14);

	/* ADC */
	hws[ADC_DIV] = ma35d1_reg_adc_clkdiv(dev, "adc_div", "pclk0", 0,
		clk_base + REG_CLK_CLKDIV4, 4, 17, 0x1ffff);
	hws[ADC_GATE] = ma35d1_clk_gate("adc_gate", "adc_div",
		clk_base + REG_CLK_APBCLK2, 24);

	/* EADC */
	hws[EADC_DIV] = ma35d1_clk_divider_table("eadc_div", "pclk2",
		clk_base + REG_CLK_CLKDIV4, 0, 4, eadc_div_table);
	hws[EADC_GATE] = ma35d1_clk_gate("eadc_gate", "eadc_div",
		clk_base + REG_CLK_APBCLK2, 25);

	ret = of_clk_add_hw_provider(clk_node, of_clk_hw_onecell_get, ma35d1_hw_data);
	if (ret < 0) {
		dev_err(dev, "failed to register hws for MA35D1\n");
		iounmap(clk_base);
	}

	clk_prepare_enable(hws[VPLL]->clk);
	clk_prepare_enable(hws[DCU_GATE]->clk);

	return ret;
}

static const struct of_device_id ma35d1_clk_of_match[] = {
	{.compatible = "nuvoton,ma35d1-clk"},
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_clk_of_match);

static struct platform_driver ma35d1_clk_driver = {
	.probe = ma35d1_clocks_probe,
	.driver = {
		   .name = "ma35d1-clk",
		   .of_match_table = ma35d1_clk_of_match,
		   },
};

static int __init ma35d1_clocks_init(void)
{
	return platform_driver_register(&ma35d1_clk_driver);
}

postcore_initcall(ma35d1_clocks_init);

MODULE_AUTHOR("CFLi<CFLi0@nuvoton.com>");
MODULE_DESCRIPTION("NUVOTON MA35D1 Clock Driver");
MODULE_LICENSE("GPL v2");
