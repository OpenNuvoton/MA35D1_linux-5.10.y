/*
 * driver/clk/nuvoton/clk-ma35d1-ccf.c
 *
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 */

#include <dt-bindings/clock/ma35d1-clk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/platform_device.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d1-sys.h>

#include "clk-ma35d1.h"
#include "regs-ma35d1-clock.h"


DEFINE_SPINLOCK(ma35d1_lock);

//static const char *ca35clk_sel_clks[] = { "hxt", "capll", "epll", "apll", };
static const char *ca35clk_sel_clks[] = { "capll", "capll", "epll", "apll", };
static const char *sysclk0_sel_clks[] = { "epll_div2", "syspll"};
//static const char *sysclk1_sel_clks[] = { "hxt", "syspll", "apll", "apll", };
static const char *sysclk1_sel_clks[] = { "syspll", "syspll", "apll", "apll", };

static const char *axiclk_sel_clks[] = { "axiclk_div2", "axiclk_div4", };

static const char *ccap0_sel_clks[] = { "hxt", "vpll", "apll", "syspll", };
static const char *ccap1_sel_clks[] = { "hxt", "vpll", "apll", "syspll", };
static const char *sdh0_sel_clks[] = { "apll", "vpll", "syspll", "syspll", };
static const char *sdh1_sel_clks[] = { "apll", "vpll", "syspll", "syspll", };
static const char *dcu_sel_clks[] = { "epll_div2", "syspll", };
static const char *dcup_sel_clks[] = { "vpll", "apll", };
static const char *gfx_sel_clks[] = { "epll", "syspll", };
static const char *dbg_sel_clks[] = { "hirc", "syspll", };

static const char *timer0_sel_clks[] = { "hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer1_sel_clks[] = { "hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer2_sel_clks[] = { "hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer3_sel_clks[] = { "hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer4_sel_clks[] = { "hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer5_sel_clks[] = { "hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer6_sel_clks[] = { "hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer7_sel_clks[] = { "hxt", "lxt", "pclk0", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer8_sel_clks[] = { "hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer9_sel_clks[] = { "hxt", "lxt", "pclk1", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer10_sel_clks[] = { "hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc", };
static const char *timer11_sel_clks[] = { "hxt", "lxt", "pclk2", "dummy", "dummy", "lirc", "dummy", "hirc", };

static const char *uart0_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart1_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart2_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart3_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart4_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart5_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart6_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart7_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart8_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart9_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart10_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart11_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart12_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart13_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart14_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart15_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };
static const char *uart16_sel_clks[] = { "hxt", "sysclk1_div2", "lxt", "hirc", };

static const char *wdt0_sel_clks[] = { "dummy", "dummy", "pclk3_div4096", "lirc", };
static const char *wdt1_sel_clks[] = { "dummy", "dummy", "pclk3_div4096", "lirc", };
static const char *wdt2_sel_clks[] = { "dummy", "dummy", "pclk4_div4096", "lirc", };
static const char *wwdt0_sel_clks[] = { "dummy", "dummy", "pclk3_div4096", "lirc", };
static const char *wwdt1_sel_clks[] = { "dummy", "dummy", "pclk3_div4096", "lirc", };
static const char *wwdt2_sel_clks[] = { "dummy", "dummy", "pclk4_div4096", "lirc", };

static const char *spi0_sel_clks[] = { "hxt", "epll_div4", "pclk1", "hirc", };
static const char *spi1_sel_clks[] = { "hxt", "epll_div4", "pclk2", "hirc", };
static const char *spi2_sel_clks[] = { "hxt", "epll_div4", "pclk1", "hirc", };
static const char *spi3_sel_clks[] = { "hxt", "epll_div4", "pclk2", "hirc", };
static const char *qspi0_sel_clks[] = { "hxt", "epll_div4", "pclk0", "hirc", };
static const char *qspi1_sel_clks[] = { "hxt", "epll_div4", "pclk0", "hirc", };

static const char *i2s0_sel_clks[] = { "hxt", "apll", "pclk0", "hirc", };
static const char *i2s1_sel_clks[] = { "hxt", "apll", "pclk2", "hirc", };

static const char *can0_sel_clks[] = { "vpll", "apll", };
static const char *can1_sel_clks[] = { "vpll", "apll", };
static const char *can2_sel_clks[] = { "vpll", "apll", };
static const char *can3_sel_clks[] = { "vpll", "apll", };

static const char *cko_sel_clks[] = { "hxt", "lxt", "hirc", "lirc", "capll_div4", "syspll", "ddrpll", "epll", "apll", "vpll", "capll", "axi0_aclk", "sysclk0", "sysclk1", "pclk3", "pclk4", };

static const char *smc0_sel_clks[] = { "hxt", "pclk4", };
static const char *smc1_sel_clks[] = { "hxt", "pclk4", };

static const char *kpi_sel_clks[] = { "hxt", "lxt", };

#if 0
// ma35d1 clock source
/*	0	*/	hxt, hxt_gate, lxt, lxt_gate, hirc, hirc_gate, lirc, lirc_gate,
    /*	8	*/	capll, syspll, ddrpll, apll, epll, vpll, epll_div2, epll_div4,
    /*	16	*/	epll_div8, ca35clk_mux, axiclk_div2, axiclk_div4, axiclk_mux, sysclk0_mux, sysclk1_mux, sysclk1_div2,
    /*	24	*/	hclk0, hclk1, hclk2, pclk0, pclk1, pclk2, hclk3, pclk3,
    /*	32	*/	pclk4, usbphy0, usbphy1, ddr0_gate, ddr6_gate, can0_mux, can0_div, can0_gate,
    /*	40	*/	can1_mux, can1_div, can1_gate, can2_mux, can2_div, can2_gate, can3_mux, can3_div,
    /*	48	*/	can3_gate, sdh0_mux, sdh0_gate, sdh1_mux, sdh1_gate, nand_gate, usbd_gate, usbh_gate,
    /*	56	*/	husbh0_gate, husbh1_gate, gfx_mux, gfx_gate, vc8k_gate, dcu_mux, dcu_gate, dcup_mux,
    /*	64	*/	dcup_div, emac0_gate, emac1_gate, ccap0_mux, ccap0_div, ccap0_gate, ccap1_mux,ccap1_div,
    /*	72	*/	ccap1_gate, pdma0_gate, pdma1_gate, pdma2_gate, pdma3_gate, wh0_gate, wh1_gate, hws_gate,
    /*	80	*/	ebi_gate, sram0_gate, sram1_gate, rom_gate, tra_gate, dbg_mux, dbg_gate, cko_mux,
    /*	88	*/	cko_div, cko_gate, gtmr_gate, gpa_gate, gpb_gate, gpc_gate, gpd_gate, gpe_gate,
    /*	96	*/	gpf_gate, gpg_gate, gph_gate, gpi_gate, gpj_gate, gpk_gate, gpl_gate, gpm_gate,
    /*	104	*/	gpn_gate, tmr0_mux, tmr0_gate, tmr1_mux, tmr1_gate, tmr2_mux, tmr2_gate, tmr3_mux,
    /*	112	*/	tmr3_gate, tmr4_mux, tmr4_gate, tmr5_mux, tmr5_gate, tmr6_mux, tmr6_gate, tmr7_mux,
    /*	120	*/	tmr7_gate, tmr8_mux, tmr8_gate, tmr9_mux, tmr9_gate, tmr10_mux, tmr10_gate, tmr11_mux,
    /*	128	*/	tmr11_gate, uart0_mux, uart0_div, uart0_gate, uart1_mux, uart1_div, uart1_gate, uart2_mux,
    /*	136	*/	uart2_div, uart2_gate, uart3_mux, uart3_div, uart3_gate, uart4_mux, uart4_div, uart4_gate,
    /*	144	*/	uart5_mux, uart5_div, uart5_gate, uart6_mux, uart6_div, uart6_gate, uart7_mux, uart7_div,
    /*	152	*/	uart7_gate, uart8_mux, uart8_div, uart8_gate, uart9_mux, uart9_div, uart9_gate, uart10_mux,
    /*	160	*/	uart10_div, uart10_gate, uart11_mux, uart11_div, uart11_gate, uart12_mux, uart12_div, uart12_gate,
    /*	168	*/	uart13_mux, uart13_div, uart13_gate, uart14_mux, uart14_div, uart14_gate, uart15_mux, uart15_div,
    /*	176	*/	uart15_gate, uart16_mux, uart16_div, uart16_gate, rtc_gate, ddr_gate, kpi_mux, kpi_div,
    /*	184	*/	kpi_gate, i2c0_gate, i2c1_gate, i2c2_gate, i2c3_gate, i2c4_gate, i2c5_gate, qspi0_mux,
    /*	192	*/	qspi0_gate, qspi1_mux, qspi1_gate, smc0_mux, smc0_div, smc0_gate, smc1_mux, smc1_div,
    /*	200	*/	smc1_gate, wdt0_mux, wdt0_gate, wdt1_mux, wdt1_gate, wdt2_mux, wdt2_gate, wwdt0_mux,
    /*	208	*/	wwdt1_mux, wwdt2_mux, epwm0_gate, epwm1_gate, epwm2_gate, i2s0_mux, i2s0_gate, i2s1_mux,
    /*	216	*/	i2s1_gate, ssmcc_gate, sspcc_gate, spi0_mux, spi0_gate, spi1_mux, spi1_gate, spi2_mux,
    /*	224	*/	spi2_gate, spi3_mux, spi3_gate, ecap0_gate, ecap1_gate, ecap2_gate, qei0_gate, qei1_gate,
    /*	232	*/	qei2_gate, adc_div, adc_gate, eadc_div, eadc_gate, clk_max
#endif

static struct clk *clk[clk_max];
static struct clk_onecell_data clk_data;

static void __init ma35d1_init_clocks(struct device_node *clk_node)
{
	int i;
	void __iomem *clk_base;

	u32 pllmode[6] = { 0, 0, 0, 0, 0, 0 };
	u32 pllfreq[6] = { 0, 0, 0, 0, 0, 0 };

	clk_node = of_find_compatible_node(NULL, NULL, "nuvoton,ma35d1-clk");
	clk_base = of_iomap(clk_node, 0);

	if (!clk_base) {
		pr_err("%s: could not map region\n", __func__);
		return;
	}

	// source
	clk[hxt] = ma35d1_clk_fixed("hxt", 24000000);
	clk[hxt_gate] = ma35d1_clk_gate("hxt_gate", "hxt", clk_base+REG_CLK_PWRCTL, 0);
	clk[lxt] = ma35d1_clk_fixed("lxt", 32768);
	clk[lxt_gate] = ma35d1_clk_gate("lxt_gate", "lxt", clk_base+REG_CLK_PWRCTL, 1);
	clk[hirc] = ma35d1_clk_fixed("hirc", 12000000);
	clk[hirc_gate] = ma35d1_clk_gate("hirc_gate", "hirc", clk_base+REG_CLK_PWRCTL, 2);
	clk[lirc] = ma35d1_clk_fixed("lirc", 32000);
	clk[lirc_gate] = ma35d1_clk_gate("lirc_gate", "lirc", clk_base+REG_CLK_PWRCTL, 3);

	// PLL
	of_property_read_u32_array(clk_node, "clock-pll-mode", pllmode, ARRAY_SIZE(pllmode));
	of_property_read_u32_array(clk_node, "assigned-clock-rates", pllfreq, ARRAY_SIZE(pllfreq));
	pr_debug("     -> pll mode:%d %d %d %d %d %d\n", pllmode[0], pllmode[1], pllmode[2], pllmode[3], pllmode[4], pllmode[5]);
	pr_debug("     -> pll frequency:%d %d %d %d %d %d\n", pllfreq[0], pllfreq[1], pllfreq[2], pllfreq[3], pllfreq[4], pllfreq[5]);

	clk[capll]= ma35d1_clk_pll(MA35D1_CAPLL, pllmode[0], "capll", "hxt", pllfreq[0], clk_base+REG_CLK_PLL0CTL0);
	clk[syspll] = ma35d1_clk_pll(MA35D1_SYSPLL, pllmode[1], "syspll", "hxt", pllfreq[1], clk_base+REG_CLK_PLL1CTL0);
	clk[ddrpll] = ma35d1_clk_pll(MA35D1_DDRPLL, pllmode[2], "ddrpll", "hxt", pllfreq[2], clk_base+REG_CLK_PLL2CTL0);
	clk[apll]= ma35d1_clk_pll(MA35D1_APLL, pllmode[3], "apll", "hxt", pllfreq[3], clk_base+REG_CLK_PLL3CTL0);
	clk[epll]= ma35d1_clk_pll(MA35D1_EPLL, pllmode[4], "epll", "hxt", pllfreq[4], clk_base+REG_CLK_PLL4CTL0);
	clk[vpll]= ma35d1_clk_pll(MA35D1_VPLL, pllmode[5], "vpll", "hxt",  pllfreq[5], clk_base+REG_CLK_PLL5CTL0);

	clk[epll_div2] = ma35d1_clk_fixed_factor("epll_div2", "epll", 1, 2);// epll/2
	clk[epll_div4] = ma35d1_clk_fixed_factor("epll_div4", "epll", 1, 4);// epll/4
	clk[epll_div8] = ma35d1_clk_fixed_factor("epll_div8", "epll", 1, 8);// epll/8

	// CA35
	//__raw_writel(__raw_readl(clk_base+REG_CLK_CLKSEL0) | (3 << 0), clk_base+REG_CLK_CLKSEL0);// ca35clk from capll
	clk[ca35clk_mux] = ma35d1_clk_mux("ca35clk_mux", clk_base+REG_CLK_CLKSEL0, 0, 2, ca35clk_sel_clks, ARRAY_SIZE(ca35clk_sel_clks));
	pr_debug("CLK_CLKSEL0=0x%x, CLK_PLL0CTL1=0x%x\n",__raw_readl(clk_base+REG_CLK_CLKSEL0),__raw_readl(clk_base+REG_CLK_PLL0CTL1));

	//AXI
	clk[axiclk_div2] = ma35d1_clk_fixed_factor("capll_div2", "ca35clk_mux", 1, 2);// capll/2
	clk[axiclk_div4] = ma35d1_clk_fixed_factor("capll_div4", "ca35clk_mux", 1, 4);// capll/4
	clk[axiclk_mux] = ma35d1_clk_mux("axiclk_mux", clk_base+REG_CLK_CLKDIV0, 26, 1, axiclk_sel_clks, ARRAY_SIZE(axiclk_sel_clks));

	// SYSCLK0 & SYSCLK1
	clk[sysclk0_mux] = ma35d1_clk_mux("sysclk0_mux", clk_base+REG_CLK_CLKSEL0, 2, 1, sysclk0_sel_clks, ARRAY_SIZE(sysclk0_sel_clks));

	clk[sysclk1_mux] = ma35d1_clk_mux("sysclk1_mux", clk_base+REG_CLK_CLKSEL0, 4, 2, sysclk1_sel_clks, ARRAY_SIZE(sysclk1_sel_clks));
	clk[sysclk1_div2] = ma35d1_clk_fixed_factor("sysclk1_div2", "sysclk1_mux", 1, 2);// sysclk1/2

	// HCLK & PCLK
	clk[hclk0] = ma35d1_clk_fixed_factor("hclk0", "sysclk1_mux", 1, 1);// sysclk1/1
	clk[hclk1] = ma35d1_clk_fixed_factor("hclk1", "sysclk1_mux", 1, 1);// sysclk1/1
	clk[hclk2] = ma35d1_clk_fixed_factor("hclk2", "sysclk1_mux", 1, 1);// sysclk1/1
	clk[pclk0] = ma35d1_clk_fixed_factor("pclk0", "sysclk1_mux", 1, 1);// sysclk1/1
	clk[pclk1] = ma35d1_clk_fixed_factor("pclk1", "sysclk1_mux", 1, 1);// sysclk1/1
	clk[pclk2] = ma35d1_clk_fixed_factor("pclk2", "sysclk1_mux", 1, 1);// sysclk1/1

	clk[hclk3] = ma35d1_clk_fixed_factor("hclk3", "sysclk1_mux", 1, 2);// sysclk1/2
	clk[pclk3] = ma35d1_clk_fixed_factor("pclk3", "sysclk1_mux", 1, 2);// sysclk1/2
	clk[pclk4] = ma35d1_clk_fixed_factor("pclk4", "sysclk1_mux", 1, 2);// sysclk1/2

	clk[usbphy0] = ma35d1_clk_fixed("usbphy0", 480000000);
	clk[usbphy1] = ma35d1_clk_fixed("usbphy1", 480000000);

	// DDR
	clk[ddr0_gate] = ma35d1_clk_gate("ddr0_gate", "ddrpll", clk_base+REG_CLK_SYSCLK0, 4);
	clk[ddr6_gate] = ma35d1_clk_gate("ddr6_gate", "ddrpll", clk_base+REG_CLK_SYSCLK0, 5);

	// CAN0
	clk[can0_mux] = ma35d1_clk_mux("can0_mux", clk_base+REG_CLK_CLKSEL4, 16, 1, can0_sel_clks, ARRAY_SIZE(can0_sel_clks));
	clk[can0_div] = ma35d1_clk_divider("can0_div", "can0_mux", clk_base+REG_CLK_CLKDIV0, 0, 2);
	clk[can0_gate] = ma35d1_clk_gate("can0_gate", "can0_div", clk_base+REG_CLK_SYSCLK0, 8);

	// CAN1
	clk[can1_mux] = ma35d1_clk_mux("can1_mux", clk_base+REG_CLK_CLKSEL4, 17, 1, can1_sel_clks, ARRAY_SIZE(can1_sel_clks));
	clk[can1_div] = ma35d1_clk_divider("can1_div", "can1_mux", clk_base+REG_CLK_CLKDIV0, 2, 2);
	clk[can1_gate] = ma35d1_clk_gate("can1_gate", "can1_div", clk_base+REG_CLK_SYSCLK0, 9);

	// CAN2
	clk[can2_mux] = ma35d1_clk_mux("can2_mux", clk_base+REG_CLK_CLKSEL4, 18, 1, can2_sel_clks, ARRAY_SIZE(can2_sel_clks));
	clk[can2_div] = ma35d1_clk_divider("can2_div", "can2_mux", clk_base+REG_CLK_CLKDIV0, 4, 2);
	clk[can2_gate] = ma35d1_clk_gate("can2_gate", "can2_div", clk_base+REG_CLK_SYSCLK0, 10);

	// CAN3
	clk[can3_mux] = ma35d1_clk_mux("can3_mux", clk_base+REG_CLK_CLKSEL4, 19, 1, can3_sel_clks, ARRAY_SIZE(can3_sel_clks));
	clk[can3_div] = ma35d1_clk_divider("can3_div", "can3_mux", clk_base+REG_CLK_CLKDIV0, 6, 2);
	clk[can3_gate] = ma35d1_clk_gate("can3_gate", "can3_div", clk_base+REG_CLK_SYSCLK0, 11);

	// SDH0
	clk[sdh0_mux] = ma35d1_clk_mux("sdh0_mux", clk_base+REG_CLK_CLKSEL0, 16, 2, sdh0_sel_clks, ARRAY_SIZE(sdh0_sel_clks));
	clk[sdh0_gate] = ma35d1_clk_gate("sdh0_gate", "sdh0_mux", clk_base+REG_CLK_SYSCLK0, 16);

	// SDH1
	clk[sdh1_mux] = ma35d1_clk_mux("sdh1_mux", clk_base+REG_CLK_CLKSEL0, 18, 2, sdh1_sel_clks, ARRAY_SIZE(sdh1_sel_clks));
	clk[sdh1_gate] = ma35d1_clk_gate("sdh1_gate", "sdh1_mux", clk_base+REG_CLK_SYSCLK0, 17);

	// NAND
	clk[nand_gate] = ma35d1_clk_gate("nand_gate", "hclk1", clk_base+REG_CLK_SYSCLK0, 18);

	// USB todo
	clk[usbd_gate] = ma35d1_clk_gate("usbd_gate", "usbphy0", clk_base+REG_CLK_SYSCLK0, 19);//usbd clk from?
	clk[usbh_gate] = ma35d1_clk_gate("usbh_gate", "usbphy0", clk_base+REG_CLK_SYSCLK0, 20);//usbh clk from?
	clk[husbh0_gate] = ma35d1_clk_gate("husbh0_gate", "usbphy0", clk_base+REG_CLK_SYSCLK0, 21);//husbh0 clk from?
	clk[husbh1_gate] = ma35d1_clk_gate("husbh1_gate", "usbphy0", clk_base+REG_CLK_SYSCLK0, 22);//husbh1 clk from?

	// GFX
	clk[gfx_mux] = ma35d1_clk_mux("gfx_mux", clk_base+REG_CLK_CLKSEL0, 26, 1, gfx_sel_clks, ARRAY_SIZE(gfx_sel_clks));
	clk[gfx_gate] = ma35d1_clk_gate("gfx_gate", "gfx_mux", clk_base+REG_CLK_SYSCLK0, 24);

	// VC8K
	clk[vc8k_gate] = ma35d1_clk_gate("vc8k_gate", "sysclk0_mux", clk_base+REG_CLK_SYSCLK0, 25);

	// DCU
	clk[dcu_mux] = ma35d1_clk_mux("dcu_mux", clk_base+REG_CLK_CLKSEL0, 24, 1, dcu_sel_clks, ARRAY_SIZE(dcu_sel_clks));
	clk[dcu_gate] = ma35d1_clk_gate("dcu_gate", "dcu_mux", clk_base+REG_CLK_SYSCLK0, 26);

	// DCUP
	clk[dcup_mux] = ma35d1_clk_mux("dcup_mux", clk_base+REG_CLK_CLKSEL0, 25, 1, dcup_sel_clks, ARRAY_SIZE(dcup_sel_clks));
	//clk[dcup_div] = ma35d1_clk_divider("dcup_div", "dcup_mux", clk_base+REG_CLK_CLKDIV0, 24, 2);
	clk[dcup_div] = ma35d1_clk_fixed_factor("dcup_div", "dcup_mux", 1, 2);// vpll/2

	// EMAC0
	clk[emac0_gate] = ma35d1_clk_gate("emac0_gate", "epll_div2", clk_base+REG_CLK_SYSCLK0, 27);

	// EMAC1
	clk[emac1_gate] = ma35d1_clk_gate("emac1_gate", "epll_div2", clk_base+REG_CLK_SYSCLK0, 28);

	// CCAP0
	clk[ccap0_mux] = ma35d1_clk_mux("ccap0_mux", clk_base+REG_CLK_CLKSEL0, 12, 1, ccap0_sel_clks, ARRAY_SIZE(ccap0_sel_clks));
	clk[ccap0_div] = ma35d1_clk_divider("ccap0_div", "ccap0_mux", clk_base+REG_CLK_CLKDIV1, 8, 4);
	clk[ccap0_gate] = ma35d1_clk_gate("ccap0_gate", "ccap0_div", clk_base+REG_CLK_SYSCLK0, 29);

	// CCAP1
	clk[ccap1_mux] = ma35d1_clk_mux("ccap1_mux", clk_base+REG_CLK_CLKSEL0, 14, 1, ccap1_sel_clks, ARRAY_SIZE(ccap1_sel_clks));
	clk[ccap1_div] = ma35d1_clk_divider("ccap1_div", "ccap1_mux", clk_base+REG_CLK_CLKDIV1, 12, 4);
	clk[ccap1_gate] = ma35d1_clk_gate("ccap1_gate", "ccap1_div", clk_base+REG_CLK_SYSCLK0, 30);

	// PDMA0~3
	clk[pdma0_gate] = ma35d1_clk_gate("pdma0_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 0);
	clk[pdma1_gate] = ma35d1_clk_gate("pdma1_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 1);
	clk[pdma2_gate] = ma35d1_clk_gate("pdma2_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 2);
	clk[pdma3_gate] = ma35d1_clk_gate("pdma3_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 3);

	// WH0~1
	clk[wh0_gate] = ma35d1_clk_gate("wh0_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 4);
	clk[wh1_gate] = ma35d1_clk_gate("wh0_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 5);

	// HWS
	clk[hws_gate] = ma35d1_clk_gate("hws_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 6);

	// EBI
	clk[ebi_gate] = ma35d1_clk_gate("ebi_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 7);

	// SRAM0~1
	clk[sram0_gate] = ma35d1_clk_gate("sram0_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 8);
	clk[sram1_gate] = ma35d1_clk_gate("sram1_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 9);

	// ROM
	clk[rom_gate] = ma35d1_clk_gate("rom_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 10);

	// TRA
	clk[tra_gate] = ma35d1_clk_gate("tra_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 11);

	// DBG
	clk[dbg_mux] = ma35d1_clk_mux("dbg_mux", clk_base+REG_CLK_CLKSEL0, 27, 1, dbg_sel_clks, ARRAY_SIZE(dbg_sel_clks));
	clk[dbg_gate] = ma35d1_clk_gate("dbg_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 12);

	// CLKO
	clk[cko_mux] = ma35d1_clk_mux("cko_mux", clk_base+REG_CLK_CLKSEL4, 24, 4, cko_sel_clks, ARRAY_SIZE(cko_sel_clks));
	clk[cko_div] = ma35d1_clk_divider("cko_div", "cko_mux", clk_base+REG_CLK_CLKOCTL, 0, 4);
	clk[cko_gate] = ma35d1_clk_gate("cko_gate", "cko_div", clk_base+REG_CLK_SYSCLK1, 13);

	// GTMR todo
	clk[gtmr_gate] = ma35d1_clk_gate("gtmr_gate", "hirc", clk_base+REG_CLK_SYSCLK1, 14);// GTMR clock from?

	// GPIO
	clk[gpa_gate] = ma35d1_clk_gate("gpa_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 16);
	clk[gpb_gate] = ma35d1_clk_gate("gpb_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 17);
	clk[gpc_gate] = ma35d1_clk_gate("gpc_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 18);
	clk[gpd_gate] = ma35d1_clk_gate("gpd_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 19);
	clk[gpe_gate] = ma35d1_clk_gate("gpe_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 20);
	clk[gpf_gate] = ma35d1_clk_gate("gpf_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 21);
	clk[gpg_gate] = ma35d1_clk_gate("gpg_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 22);
	clk[gph_gate] = ma35d1_clk_gate("gph_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 23);
	clk[gpi_gate] = ma35d1_clk_gate("gpi_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 24);
	clk[gpj_gate] = ma35d1_clk_gate("gpj_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 25);
	clk[gpk_gate] = ma35d1_clk_gate("gpk_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 26);
	clk[gpl_gate] = ma35d1_clk_gate("gpl_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 27);
	clk[gpm_gate] = ma35d1_clk_gate("gpm_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 28);
	clk[gpn_gate] = ma35d1_clk_gate("gpn_gate", "hclk0", clk_base+REG_CLK_SYSCLK1, 29);

	// TIMER0~11
	clk[tmr0_mux] = ma35d1_clk_mux("tmr0_mux", clk_base+REG_CLK_CLKSEL1, 0, 3, timer0_sel_clks, ARRAY_SIZE(timer0_sel_clks));
	clk[tmr0_gate] = ma35d1_clk_gate("tmr0_gate", "tmr0_mux", clk_base+REG_CLK_APBCLK0, 0);
	clk[tmr1_mux] = ma35d1_clk_mux("tmr1_mux", clk_base+REG_CLK_CLKSEL1, 4, 3, timer1_sel_clks, ARRAY_SIZE(timer1_sel_clks));
	clk[tmr1_gate] = ma35d1_clk_gate("tmr1_gate", "tmr1_mux", clk_base+REG_CLK_APBCLK0, 1);
	clk[tmr2_mux] = ma35d1_clk_mux("tmr2_mux", clk_base+REG_CLK_CLKSEL1, 8, 3, timer2_sel_clks, ARRAY_SIZE(timer2_sel_clks));
	clk[tmr2_gate] = ma35d1_clk_gate("tmr2_gate", "tmr2_mux", clk_base+REG_CLK_APBCLK0, 2);
	clk[tmr3_mux] = ma35d1_clk_mux("tmr3_mux", clk_base+REG_CLK_CLKSEL1, 12, 3, timer3_sel_clks, ARRAY_SIZE(timer3_sel_clks));
	clk[tmr3_gate] = ma35d1_clk_gate("tmr3_gate", "tmr3_mux", clk_base+REG_CLK_APBCLK0, 3);
	clk[tmr4_mux] = ma35d1_clk_mux("tmr4_mux", clk_base+REG_CLK_CLKSEL1, 16, 3, timer4_sel_clks, ARRAY_SIZE(timer4_sel_clks));
	clk[tmr4_gate] = ma35d1_clk_gate("tmr4_gate", "tmr4_mux", clk_base+REG_CLK_APBCLK0, 4);
	clk[tmr5_mux] = ma35d1_clk_mux("tmr5_mux", clk_base+REG_CLK_CLKSEL1, 20, 3, timer5_sel_clks, ARRAY_SIZE(timer5_sel_clks));
	clk[tmr5_gate] = ma35d1_clk_gate("tmr5_gate", "tmr5_mux", clk_base+REG_CLK_APBCLK0, 5);
	clk[tmr6_mux] = ma35d1_clk_mux("tmr6_mux", clk_base+REG_CLK_CLKSEL1, 24, 3, timer6_sel_clks, ARRAY_SIZE(timer6_sel_clks));
	clk[tmr6_gate] = ma35d1_clk_gate("tmr6_gate", "tmr6_mux", clk_base+REG_CLK_APBCLK0, 6);
	clk[tmr7_mux] = ma35d1_clk_mux("tmr7_mux", clk_base+REG_CLK_CLKSEL1, 28, 3, timer7_sel_clks, ARRAY_SIZE(timer7_sel_clks));
	clk[tmr7_gate] = ma35d1_clk_gate("tmr7_gate", "tmr7_mux", clk_base+REG_CLK_APBCLK0, 7);
	clk[tmr8_mux] = ma35d1_clk_mux("tmr8_mux", clk_base+REG_CLK_CLKSEL2, 0, 3, timer8_sel_clks, ARRAY_SIZE(timer8_sel_clks));
	clk[tmr8_gate] = ma35d1_clk_gate("tmr8_gate", "tmr8_mux", clk_base+REG_CLK_APBCLK0, 8);
	clk[tmr9_mux] = ma35d1_clk_mux("tmr9_mux", clk_base+REG_CLK_CLKSEL2, 4, 3, timer9_sel_clks, ARRAY_SIZE(timer9_sel_clks));
	clk[tmr9_gate] = ma35d1_clk_gate("tmr9_gate", "tmr9_mux", clk_base+REG_CLK_APBCLK0, 9);
	clk[tmr10_mux] = ma35d1_clk_mux("tmr10_mux", clk_base+REG_CLK_CLKSEL2, 8, 3, timer10_sel_clks, ARRAY_SIZE(timer10_sel_clks));
	clk[tmr10_gate] = ma35d1_clk_gate("tmr10_gate", "tmr10_mux", clk_base+REG_CLK_APBCLK0, 10);
	clk[tmr11_mux] = ma35d1_clk_mux("tmr11_mux", clk_base+REG_CLK_CLKSEL2, 12, 3, timer11_sel_clks, ARRAY_SIZE(timer11_sel_clks));
	clk[tmr11_gate] = ma35d1_clk_gate("tmr11_gate", "tmr11_mux", clk_base+REG_CLK_APBCLK0, 11);

	// UART0~16
	clk[uart0_mux] = ma35d1_clk_mux("uart0_mux", clk_base+REG_CLK_CLKSEL2, 16, 2, uart0_sel_clks, ARRAY_SIZE(uart0_sel_clks));
	clk[uart0_div] = ma35d1_clk_divider("uart0_div", "uart0_mux", clk_base+REG_CLK_CLKDIV1, 16, 4);
	clk[uart0_gate] = ma35d1_clk_gate("uart0_gate", "uart0_div", clk_base+REG_CLK_APBCLK0, 12);
	clk[uart1_mux] = ma35d1_clk_mux("uart1_mux", clk_base+REG_CLK_CLKSEL2, 18, 2, uart1_sel_clks, ARRAY_SIZE(uart1_sel_clks));
	clk[uart1_div] = ma35d1_clk_divider("uart1_div", "uart1_mux", clk_base+REG_CLK_CLKDIV1, 20, 4);
	clk[uart1_gate] = ma35d1_clk_gate("uart1_gate", "uart1_div", clk_base+REG_CLK_APBCLK0, 13);
	clk[uart2_mux] = ma35d1_clk_mux("uart2_mux", clk_base+REG_CLK_CLKSEL2, 20, 2, uart2_sel_clks, ARRAY_SIZE(uart2_sel_clks));
	clk[uart2_div] = ma35d1_clk_divider("uart2_div", "uart2_mux", clk_base+REG_CLK_CLKDIV1, 24, 4);
	clk[uart2_gate] = ma35d1_clk_gate("uart2_gate", "uart2_div", clk_base+REG_CLK_APBCLK0, 14);
	clk[uart3_mux] = ma35d1_clk_mux("uart3_mux", clk_base+REG_CLK_CLKSEL2, 22, 2, uart3_sel_clks, ARRAY_SIZE(uart3_sel_clks));
	clk[uart3_div] = ma35d1_clk_divider("uart3_div", "uart3_mux", clk_base+REG_CLK_CLKDIV1, 28, 4);
	clk[uart3_gate] = ma35d1_clk_gate("uart3_gate", "uart3_div", clk_base+REG_CLK_APBCLK0, 15);
	clk[uart4_mux] = ma35d1_clk_mux("uart4_mux", clk_base+REG_CLK_CLKSEL2, 24, 2, uart4_sel_clks, ARRAY_SIZE(uart4_sel_clks));
	clk[uart4_div] = ma35d1_clk_divider("uart4_div", "uart4_mux", clk_base+REG_CLK_CLKDIV2, 0, 4);
	clk[uart4_gate] = ma35d1_clk_gate("uart4_gate", "uart4_div", clk_base+REG_CLK_APBCLK0, 16);
	clk[uart5_mux] = ma35d1_clk_mux("uart5_mux", clk_base+REG_CLK_CLKSEL2, 26, 2, uart5_sel_clks, ARRAY_SIZE(uart5_sel_clks));
	clk[uart5_div] = ma35d1_clk_divider("uart5_div", "uart5_mux", clk_base+REG_CLK_CLKDIV2, 4, 4);
	clk[uart5_gate] = ma35d1_clk_gate("uart5_gate", "uart5_div", clk_base+REG_CLK_APBCLK0, 17);
	clk[uart6_mux] = ma35d1_clk_mux("uart6_mux", clk_base+REG_CLK_CLKSEL2, 28, 2, uart6_sel_clks, ARRAY_SIZE(uart6_sel_clks));
	clk[uart6_div] = ma35d1_clk_divider("uart6_div", "uart6_mux", clk_base+REG_CLK_CLKDIV2, 8, 4);
	clk[uart6_gate] = ma35d1_clk_gate("uart6_gate", "uart6_div", clk_base+REG_CLK_APBCLK0, 18);
	clk[uart7_mux] = ma35d1_clk_mux("uart7_mux", clk_base+REG_CLK_CLKSEL2, 30, 2, uart7_sel_clks, ARRAY_SIZE(uart7_sel_clks));
	clk[uart7_div] = ma35d1_clk_divider("uart7_div", "uart7_mux", clk_base+REG_CLK_CLKDIV2, 12, 4);
	clk[uart7_gate] = ma35d1_clk_gate("uart7_gate", "uart7_div", clk_base+REG_CLK_APBCLK0, 19);
	clk[uart8_mux] = ma35d1_clk_mux("uart8_mux", clk_base+REG_CLK_CLKSEL3, 0, 2, uart8_sel_clks, ARRAY_SIZE(uart8_sel_clks));
	clk[uart8_div] = ma35d1_clk_divider("uart8_div", "uart8_mux", clk_base+REG_CLK_CLKDIV2, 16, 4);
	clk[uart8_gate] = ma35d1_clk_gate("uart8_gate", "uart8_div", clk_base+REG_CLK_APBCLK0, 20);
	clk[uart9_mux] = ma35d1_clk_mux("uart9_mux", clk_base+REG_CLK_CLKSEL3, 2, 2, uart9_sel_clks, ARRAY_SIZE(uart9_sel_clks));
	clk[uart9_div] = ma35d1_clk_divider("uart9_div", "uart9_mux", clk_base+REG_CLK_CLKDIV2, 20, 4);
	clk[uart9_gate] = ma35d1_clk_gate("uart9_gate", "uart9_div", clk_base+REG_CLK_APBCLK0, 21);
	clk[uart10_mux] = ma35d1_clk_mux("uart10_mux", clk_base+REG_CLK_CLKSEL3, 4, 2, uart10_sel_clks, ARRAY_SIZE(uart10_sel_clks));
	clk[uart10_div] = ma35d1_clk_divider("uart10_div", "uart10_mux", clk_base+REG_CLK_CLKDIV2, 24, 4);
	clk[uart10_gate] = ma35d1_clk_gate("uart10_gate", "uart10_div", clk_base+REG_CLK_APBCLK0, 22);
	clk[uart11_mux] = ma35d1_clk_mux("uart11_mux", clk_base+REG_CLK_CLKSEL3, 6, 2, uart11_sel_clks, ARRAY_SIZE(uart11_sel_clks));
	clk[uart11_div] = ma35d1_clk_divider("uart11_div", "uart11_mux", clk_base+REG_CLK_CLKDIV2, 28, 4);
	clk[uart11_gate] = ma35d1_clk_gate("uart11_gate", "uart11_div", clk_base+REG_CLK_APBCLK0, 23);
	clk[uart12_mux] = ma35d1_clk_mux("uart12_mux", clk_base+REG_CLK_CLKSEL3, 8, 2, uart12_sel_clks, ARRAY_SIZE(uart12_sel_clks));
	clk[uart12_div] = ma35d1_clk_divider("uart12_div", "uart12_mux", clk_base+REG_CLK_CLKDIV3, 0, 4);
	clk[uart12_gate] = ma35d1_clk_gate("uart12_gate", "uart12_div", clk_base+REG_CLK_APBCLK0, 24);
	clk[uart13_mux] = ma35d1_clk_mux("uart13_mux", clk_base+REG_CLK_CLKSEL3, 10, 2, uart13_sel_clks, ARRAY_SIZE(uart13_sel_clks));
	clk[uart13_div] = ma35d1_clk_divider("uart13_div", "uart13_mux", clk_base+REG_CLK_CLKDIV3, 4, 4);
	clk[uart13_gate] = ma35d1_clk_gate("uart13_gate", "uart13_div", clk_base+REG_CLK_APBCLK0, 25);
	clk[uart14_mux] = ma35d1_clk_mux("uart14_mux", clk_base+REG_CLK_CLKSEL3, 12, 2, uart14_sel_clks, ARRAY_SIZE(uart14_sel_clks));
	clk[uart14_div] = ma35d1_clk_divider("uart14_div", "uart14_mux", clk_base+REG_CLK_CLKDIV3, 8, 4);
	clk[uart14_gate] = ma35d1_clk_gate("uart14_gate", "uart14_div", clk_base+REG_CLK_APBCLK0, 26);
	clk[uart15_mux] = ma35d1_clk_mux("uart15_mux", clk_base+REG_CLK_CLKSEL3, 14, 2, uart15_sel_clks, ARRAY_SIZE(uart15_sel_clks));
	clk[uart15_div] = ma35d1_clk_divider("uart15_div", "uart15_mux", clk_base+REG_CLK_CLKDIV3, 12, 4);
	clk[uart15_gate] = ma35d1_clk_gate("uart15_gate", "uart15_div", clk_base+REG_CLK_APBCLK0, 27);
	clk[uart16_mux] = ma35d1_clk_mux("uart16_mux", clk_base+REG_CLK_CLKSEL3, 16, 2, uart16_sel_clks, ARRAY_SIZE(uart16_sel_clks));
	clk[uart16_div] = ma35d1_clk_divider("uart16_div", "uart16_mux", clk_base+REG_CLK_CLKDIV3, 16, 4);
	clk[uart16_gate] = ma35d1_clk_gate("uart16_gate", "uart16_div", clk_base+REG_CLK_APBCLK0, 28);

	// RTC
	clk[rtc_gate] = ma35d1_clk_gate("rtc_gate", "lxt", clk_base+REG_CLK_APBCLK0, 29);

	//DDRP
	clk[ddr_gate] = ma35d1_clk_gate("ddr_gate", "ddrpll", clk_base+REG_CLK_APBCLK0, 30);

	//KPI
	clk[kpi_mux] = ma35d1_clk_mux("kpi_mux", clk_base+REG_CLK_CLKSEL4, 30, 1, kpi_sel_clks, ARRAY_SIZE(kpi_sel_clks));
	clk[kpi_div] = ma35d1_clk_divider("kpi_div", "kpi_mux", clk_base+REG_CLK_CLKDIV4, 24, 8);
	clk[kpi_gate] = ma35d1_clk_gate("kpi_gate", "kpi_div", clk_base+REG_CLK_APBCLK0, 31);

	// I2C0~5
	clk[i2c0_gate] = ma35d1_clk_gate("i2c0_gate", "pclk0", clk_base+REG_CLK_APBCLK1, 0);
	clk[i2c1_gate] = ma35d1_clk_gate("i2c1_gate", "pclk1", clk_base+REG_CLK_APBCLK1, 1);
	clk[i2c2_gate] = ma35d1_clk_gate("i2c2_gate", "pclk2", clk_base+REG_CLK_APBCLK1, 2);
	clk[i2c3_gate] = ma35d1_clk_gate("i2c3_gate", "pclk0", clk_base+REG_CLK_APBCLK1, 3);
	clk[i2c4_gate] = ma35d1_clk_gate("i2c4_gate", "pclk1", clk_base+REG_CLK_APBCLK1, 4);
	clk[i2c5_gate] = ma35d1_clk_gate("i2c5_gate", "pclk2", clk_base+REG_CLK_APBCLK1, 5);

	// QSPI0~1
	clk[qspi0_mux] = ma35d1_clk_mux("qspi0_mux", clk_base+REG_CLK_CLKSEL4, 8, 2, qspi0_sel_clks, ARRAY_SIZE(qspi0_sel_clks));
	clk[qspi0_gate] = ma35d1_clk_gate("qspi0_gate", "qspi0_mux", clk_base+REG_CLK_APBCLK1, 6);
	clk[qspi1_mux] = ma35d1_clk_mux("qspi1_mux", clk_base+REG_CLK_CLKSEL4, 10, 2, qspi1_sel_clks, ARRAY_SIZE(qspi1_sel_clks));
	clk[qspi1_gate] = ma35d1_clk_gate("qspi1_gate", "qspi1_mux", clk_base+REG_CLK_APBCLK1, 7);

	//SMC0~1
	clk[smc0_mux] = ma35d1_clk_mux("smc0_mux", clk_base+REG_CLK_CLKSEL4, 28, 1, smc0_sel_clks, ARRAY_SIZE(smc0_sel_clks));
	clk[smc0_div] = ma35d1_clk_divider("smc0_div", "smc0_mux", clk_base+REG_CLK_CLKDIV1, 0, 4);
	clk[smc0_gate] = ma35d1_clk_gate("smc0_gate", "smc0_div", clk_base+REG_CLK_APBCLK1, 12);

	clk[smc1_mux] = ma35d1_clk_mux("smc1_mux", clk_base+REG_CLK_CLKSEL4, 29, 1, smc1_sel_clks, ARRAY_SIZE(smc1_sel_clks));
	clk[smc1_div] = ma35d1_clk_divider("smc1_div", "smc1_mux", clk_base+REG_CLK_CLKDIV1, 4, 4);
	clk[smc1_gate] = ma35d1_clk_gate("smc1_gate", "smc1_div", clk_base+REG_CLK_APBCLK1, 13);

	// WDT0~2
	clk[wdt0_mux] = ma35d1_clk_mux("wdt0_mux", clk_base+REG_CLK_CLKSEL3, 20, 2, wdt0_sel_clks, ARRAY_SIZE(wdt0_sel_clks));
	clk[wdt0_gate] = ma35d1_clk_gate("wdt0_gate", "wdt0_mux", clk_base+REG_CLK_APBCLK1, 16);
	clk[wdt1_mux] = ma35d1_clk_mux("wdt1_mux", clk_base+REG_CLK_CLKSEL3, 24, 2, wdt1_sel_clks, ARRAY_SIZE(wdt1_sel_clks));
	clk[wdt1_gate] = ma35d1_clk_gate("wdt1_gate", "wdt1_mux", clk_base+REG_CLK_APBCLK1, 17);
	clk[wdt2_mux] = ma35d1_clk_mux("wdt2_mux", clk_base+REG_CLK_CLKSEL3, 28, 2, wdt2_sel_clks, ARRAY_SIZE(wdt2_sel_clks));
	clk[wdt2_gate] = ma35d1_clk_gate("wdt2_gate", "wdt2_mux", clk_base+REG_CLK_APBCLK1, 18);

	// WWDT0~2
	clk[wwdt0_mux] = ma35d1_clk_mux("wwdt0_mux", clk_base+REG_CLK_CLKSEL3, 22, 2, wwdt0_sel_clks, ARRAY_SIZE(wwdt0_sel_clks));
	clk[wwdt1_mux] = ma35d1_clk_mux("wwdt1_mux", clk_base+REG_CLK_CLKSEL3, 26, 2, wwdt1_sel_clks, ARRAY_SIZE(wwdt1_sel_clks));
	clk[wwdt2_mux] = ma35d1_clk_mux("wwdt2_mux", clk_base+REG_CLK_CLKSEL3, 30, 2, wwdt2_sel_clks, ARRAY_SIZE(wwdt2_sel_clks));

	// EPWM0~2
	clk[epwm0_gate] = ma35d1_clk_gate("epwm0_gate", "pclk1", clk_base+REG_CLK_APBCLK1, 24);
	clk[epwm1_gate] = ma35d1_clk_gate("epwm1_gate", "pclk2", clk_base+REG_CLK_APBCLK1, 25);
	clk[epwm2_gate] = ma35d1_clk_gate("epwm2_gate", "pclk1", clk_base+REG_CLK_APBCLK1, 26);

	//I2S0~1
	clk[i2s0_mux] = ma35d1_clk_mux("i2s0_mux", clk_base+REG_CLK_CLKSEL4, 12, 2, i2s0_sel_clks, ARRAY_SIZE(i2s0_sel_clks));
	clk[i2s0_gate] = ma35d1_clk_gate("i2s0_gate", "i2s0_mux", clk_base+REG_CLK_APBCLK2, 0);
	clk[i2s1_mux] = ma35d1_clk_mux("i2s1_mux", clk_base+REG_CLK_CLKSEL4, 14, 2, i2s1_sel_clks, ARRAY_SIZE(i2s1_sel_clks));
	clk[i2s1_gate] = ma35d1_clk_gate("i2s1_gate", "i2s1_mux", clk_base+REG_CLK_APBCLK2, 1);

	// SSMCC
	clk[ssmcc_gate] = ma35d1_clk_gate("ssmcc_gate", "pclk3", clk_base+REG_CLK_APBCLK2, 2);

	// SSPCC
	clk[sspcc_gate] = ma35d1_clk_gate("sspcc_gate", "pclk3", clk_base+REG_CLK_APBCLK2, 3);

	// SPI0~3
	clk[spi0_mux] = ma35d1_clk_mux("spi0_mux", clk_base+REG_CLK_CLKSEL4, 0, 2, spi0_sel_clks, ARRAY_SIZE(spi0_sel_clks));
	clk[spi0_gate] = ma35d1_clk_gate("spi0_gate", "spi0_mux", clk_base+REG_CLK_APBCLK2, 4);
	clk[spi1_mux] = ma35d1_clk_mux("spi1_mux", clk_base+REG_CLK_CLKSEL4, 2, 2, spi1_sel_clks, ARRAY_SIZE(spi1_sel_clks));
	clk[spi1_gate] = ma35d1_clk_gate("spi1_gate", "spi1_mux", clk_base+REG_CLK_APBCLK2, 5);
	clk[spi2_mux] = ma35d1_clk_mux("spi2_mux", clk_base+REG_CLK_CLKSEL4, 4, 2, spi2_sel_clks, ARRAY_SIZE(spi2_sel_clks));
	clk[spi2_gate] = ma35d1_clk_gate("spi2_gate", "spi2_mux", clk_base+REG_CLK_APBCLK2, 6);
	clk[spi3_mux] = ma35d1_clk_mux("spi3_mux", clk_base+REG_CLK_CLKSEL4, 6, 2, spi3_sel_clks, ARRAY_SIZE(spi3_sel_clks));
	clk[spi3_gate] = ma35d1_clk_gate("spi3_gate", "spi3_mux", clk_base+REG_CLK_APBCLK2, 7);

	// ECAP0~2
	clk[ecap0_gate] = ma35d1_clk_gate("ecap0_gate", "pclk1", clk_base+REG_CLK_APBCLK2, 8);
	clk[ecap1_gate] = ma35d1_clk_gate("ecap1_gate", "pclk2", clk_base+REG_CLK_APBCLK2, 9);
	clk[ecap2_gate] = ma35d1_clk_gate("ecap2_gate", "pclk1", clk_base+REG_CLK_APBCLK2, 10);

	// QEI0~2
	clk[qei0_gate] = ma35d1_clk_gate("qei0_gate", "pclk1", clk_base+REG_CLK_APBCLK2, 12);
	clk[qei1_gate] = ma35d1_clk_gate("qei1_gate", "pclk2", clk_base+REG_CLK_APBCLK2, 13);
	clk[qei2_gate] = ma35d1_clk_gate("qei2_gate", "pclk1", clk_base+REG_CLK_APBCLK2, 14);

	// ADC
	clk[adc_div] = ma35d1_clk_divider("adc_div", "pclk0", clk_base+REG_CLK_CLKDIV4, 4, 17);
	clk[adc_gate] = ma35d1_clk_gate("adc_gate", "adc_div", clk_base+REG_CLK_APBCLK2, 24);

	// EADC
	clk[eadc_div] = ma35d1_clk_divider("eadc_div", "pclk0", clk_base+REG_CLK_CLKDIV4, 0, 4);
	clk[eadc_gate] = ma35d1_clk_gate("eadc_gate", "eadc_div", clk_base+REG_CLK_APBCLK2, 25);

#if 0
	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("ma35d1 clk %d: register failed with %ld\n", i, PTR_ERR(clk[i]));
#endif


	clk_register_clkdev(clk[hxt], "hxt", NULL);
	clk_register_clkdev(clk[hxt_gate], "hxt_gate", NULL);
	clk_register_clkdev(clk[lxt], "lxt", NULL);
	clk_register_clkdev(clk[lxt_gate], "lxt_gate", NULL);
	clk_register_clkdev(clk[hirc], "hirc", NULL);
	clk_register_clkdev(clk[hirc_gate], "hirc_gate", NULL);
	clk_register_clkdev(clk[lirc], "lirc", NULL);
	clk_register_clkdev(clk[lirc_gate], "lirc_gate", NULL);

	clk_register_clkdev(clk[capll], "capll", NULL);
	clk_register_clkdev(clk[syspll], "syspll", NULL);
	clk_register_clkdev(clk[ddrpll], "ddrpll", NULL);
	clk_register_clkdev(clk[apll], "apll", NULL);
	clk_register_clkdev(clk[epll], "epll", NULL);
	clk_register_clkdev(clk[vpll], "vpll", NULL);

	clk_register_clkdev(clk[epll_div2], "epll_div2", NULL);
	clk_register_clkdev(clk[epll_div4], "epll_div4", NULL);

	clk_register_clkdev(clk[ca35clk_mux], "ca35clk_mux", NULL);

	clk_register_clkdev(clk[axiclk_div2], "axiclk_div2", NULL);
	clk_register_clkdev(clk[axiclk_div4], "axiclk_div4", NULL);
	clk_register_clkdev(clk[axiclk_mux],  "axiclk_mux", NULL);

	clk_register_clkdev(clk[sysclk0_mux], "sysclk0_mux", NULL);
	clk_register_clkdev(clk[sysclk1_mux], "sysclk1_mux", NULL);
	clk_register_clkdev(clk[sysclk1_div2], "sysclk1_div2", NULL);

	clk_register_clkdev(clk[hclk0], "hclk0", NULL);
	clk_register_clkdev(clk[hclk1], "hclk1", NULL);
	clk_register_clkdev(clk[hclk2], "hclk2", NULL);
	clk_register_clkdev(clk[pclk0], "pclk0", NULL);
	clk_register_clkdev(clk[pclk1], "pclk1", NULL);
	clk_register_clkdev(clk[pclk2], "pclk2", NULL);

	clk_register_clkdev(clk[hclk3], "hclk3", NULL);
	clk_register_clkdev(clk[pclk3], "pclk3", NULL);
	clk_register_clkdev(clk[pclk4], "pclk4", NULL);

	clk_register_clkdev(clk[usbphy0], "usbphy0", NULL);
	clk_register_clkdev(clk[usbphy1], "usbphy1", NULL);

	clk_register_clkdev(clk[ddr0_gate], "ddr0_gate", NULL);
	clk_register_clkdev(clk[ddr6_gate], "ddr6_gate", NULL);

	clk_register_clkdev(clk[can0_mux], "can0_mux", NULL);
	clk_register_clkdev(clk[can0_div], "can0_div", NULL);
	clk_register_clkdev(clk[can0_gate],"can0_gate", NULL);

	clk_register_clkdev(clk[can1_mux], "can1_mux", NULL);
	clk_register_clkdev(clk[can1_div], "can1_div", NULL);
	clk_register_clkdev(clk[can1_gate],"can1_gate", NULL);

	clk_register_clkdev(clk[can2_mux], "can2_mux", NULL);
	clk_register_clkdev(clk[can2_div], "can2_div", NULL);
	clk_register_clkdev(clk[can2_gate],"can2_gate", NULL);

	clk_register_clkdev(clk[can3_mux], "can3_mux", NULL);
	clk_register_clkdev(clk[can3_div], "can3_div", NULL);
	clk_register_clkdev(clk[can3_gate],"can3_gate", NULL);

	clk_register_clkdev(clk[sdh0_mux], "sdh0_mux", NULL);
	clk_register_clkdev(clk[sdh0_gate], "sdh0_gate", NULL);

	clk_register_clkdev(clk[sdh1_mux], "sdh1_mux", NULL);
	clk_register_clkdev(clk[sdh1_gate],"sdh1_gate", NULL);

	clk_register_clkdev(clk[nand_gate], "nand_gate", NULL);

	clk_register_clkdev(clk[usbd_gate], "usbd_gate", NULL);
	clk_register_clkdev(clk[usbh_gate], "usbh_gate", NULL);
	clk_register_clkdev(clk[husbh0_gate], "husbh0_gate", NULL);
	clk_register_clkdev(clk[husbh1_gate], "husbh1_gate", NULL);

	clk_register_clkdev(clk[gfx_mux], "gfx_mux", NULL);
	clk_register_clkdev(clk[gfx_gate],"gfx_gate", NULL);

	clk_register_clkdev(clk[vc8k_gate], "vc8k_gate", NULL);

	clk_register_clkdev(clk[dcu_mux], "dcu_mux", NULL);
	clk_register_clkdev(clk[dcu_gate],"dcu_gate", NULL);

	clk_register_clkdev(clk[dcup_mux], "dcup_mux", NULL);
	clk_register_clkdev(clk[dcup_div], "dcup_div", NULL);

	clk_register_clkdev(clk[emac0_gate],"emac0_gate", NULL);
	clk_register_clkdev(clk[emac1_gate],"emac1_gate", NULL);

	clk_register_clkdev(clk[ccap0_mux], "ccap0_mux", NULL);
	clk_register_clkdev(clk[ccap0_div], "ccap0_div", NULL);
	clk_register_clkdev(clk[ccap0_gate],"ccap0_gate", NULL);
	clk_register_clkdev(clk[ccap1_mux], "ccap1_mux", NULL);
	clk_register_clkdev(clk[ccap1_div], "ccap1_div", NULL);
	clk_register_clkdev(clk[ccap1_gate],"ccap1_gate", NULL);

	clk_register_clkdev(clk[pdma0_gate], "pdma0_gate", NULL);
	clk_register_clkdev(clk[pdma1_gate], "pdma1_gate", NULL);
	clk_register_clkdev(clk[pdma2_gate], "pdma2_gate", NULL);
	clk_register_clkdev(clk[pdma3_gate], "pdma3_gate", NULL);

	clk_register_clkdev(clk[wh0_gate], "wh0_gate", NULL);
	clk_register_clkdev(clk[wh1_gate], "wh1_gate", NULL);

	clk_register_clkdev(clk[hws_gate], "hws_gate", NULL);

	clk_register_clkdev(clk[ebi_gate], "ebi_gate", NULL);

	clk_register_clkdev(clk[sram0_gate], "sram0_gate", NULL);
	clk_register_clkdev(clk[sram1_gate], "sram1_gate", NULL);

	clk_register_clkdev(clk[rom_gate], "rom_gate", NULL);

	clk_register_clkdev(clk[tra_gate], "tra_gate", NULL);

	clk_register_clkdev(clk[dbg_mux], "dbg_mux", NULL);
	clk_register_clkdev(clk[dbg_gate],"dbg_gate", NULL);

	clk_register_clkdev(clk[cko_mux], "cko_mux", NULL);
	clk_register_clkdev(clk[cko_div], "cko_div", NULL);
	clk_register_clkdev(clk[cko_gate],"cko_gate", NULL);

	clk_register_clkdev(clk[gtmr_gate], "gtmr_gate", NULL);

	clk_register_clkdev(clk[gpa_gate], "gpa_gate", NULL);
	clk_register_clkdev(clk[gpb_gate], "gpb_gate", NULL);
	clk_register_clkdev(clk[gpc_gate], "gpc_gate", NULL);
	clk_register_clkdev(clk[gpd_gate], "gpd_gate", NULL);
	clk_register_clkdev(clk[gpe_gate], "gpe_gate", NULL);
	clk_register_clkdev(clk[gpf_gate], "gpf_gate", NULL);
	clk_register_clkdev(clk[gpg_gate], "gpg_gate", NULL);
	clk_register_clkdev(clk[gph_gate], "gph_gate", NULL);
	clk_register_clkdev(clk[gpi_gate], "gpi_gate", NULL);
	clk_register_clkdev(clk[gpj_gate], "gpj_gate", NULL);
	clk_register_clkdev(clk[gpk_gate], "gpk_gate", NULL);
	clk_register_clkdev(clk[gpl_gate], "gpl_gate", NULL);
	clk_register_clkdev(clk[gpm_gate], "gpm_gate", NULL);
	clk_register_clkdev(clk[gpn_gate], "gpn_gate", NULL);

	clk_register_clkdev(clk[tmr0_mux], "tmr0_mux", NULL);
	clk_register_clkdev(clk[tmr0_gate],"tmr0_gate", NULL);
	clk_register_clkdev(clk[tmr1_mux], "tmr1_mux", NULL);
	clk_register_clkdev(clk[tmr1_gate],"tmr1_gate", NULL);
	clk_register_clkdev(clk[tmr2_mux], "tmr2_mux", NULL);
	clk_register_clkdev(clk[tmr2_gate],"tmr2_gate", NULL);
	clk_register_clkdev(clk[tmr3_mux], "tmr3_mux", NULL);
	clk_register_clkdev(clk[tmr3_gate],"tmr3_gate", NULL);
	clk_register_clkdev(clk[tmr4_mux], "tmr4_mux", NULL);
	clk_register_clkdev(clk[tmr4_gate],"tmr4_gate", NULL);
	clk_register_clkdev(clk[tmr5_mux], "tmr5_mux", NULL);
	clk_register_clkdev(clk[tmr5_gate],"tmr5_gate", NULL);
	clk_register_clkdev(clk[tmr6_mux], "tmr6_mux", NULL);
	clk_register_clkdev(clk[tmr6_gate],"tmr6_gate", NULL);
	clk_register_clkdev(clk[tmr7_mux], "tmr7_mux", NULL);
	clk_register_clkdev(clk[tmr7_gate],"tmr7_gate", NULL);
	clk_register_clkdev(clk[tmr8_mux], "tmr8_mux", NULL);
	clk_register_clkdev(clk[tmr8_gate],"tmr8_gate", NULL);
	clk_register_clkdev(clk[tmr9_mux], "tmr9_mux", NULL);
	clk_register_clkdev(clk[tmr9_gate],"tmr9_gate", NULL);
	clk_register_clkdev(clk[tmr10_mux], "tmr10_mux", NULL);
	clk_register_clkdev(clk[tmr10_gate],"tmr10_gate", NULL);
	clk_register_clkdev(clk[tmr11_mux], "tmr11_mux", NULL);
	clk_register_clkdev(clk[tmr11_gate],"tmr11_gate", NULL);

	clk_register_clkdev(clk[uart0_mux], "uart0_mux", NULL);
	clk_register_clkdev(clk[uart0_div], "uart0_div", NULL);
	clk_register_clkdev(clk[uart0_gate],"uart0_gate", NULL);
	clk_register_clkdev(clk[uart1_mux], "uart1_mux", NULL);
	clk_register_clkdev(clk[uart1_div], "uart1_div", NULL);
	clk_register_clkdev(clk[uart1_gate],"uart2_gate", NULL);
	clk_register_clkdev(clk[uart2_mux], "uart2_mux", NULL);
	clk_register_clkdev(clk[uart2_div], "uart2_div", NULL);
	clk_register_clkdev(clk[uart2_gate],"uart2_gate", NULL);
	clk_register_clkdev(clk[uart3_mux], "uart3_mux", NULL);
	clk_register_clkdev(clk[uart3_div], "uart3_div", NULL);
	clk_register_clkdev(clk[uart3_gate],"uart3_gate", NULL);
	clk_register_clkdev(clk[uart4_mux], "uart4_mux", NULL);
	clk_register_clkdev(clk[uart4_div], "uart4_div", NULL);
	clk_register_clkdev(clk[uart4_gate],"uart4_gate", NULL);
	clk_register_clkdev(clk[uart5_mux], "uart5_mux", NULL);
	clk_register_clkdev(clk[uart5_div], "uart5_div", NULL);
	clk_register_clkdev(clk[uart5_gate],"uart5_gate", NULL);
	clk_register_clkdev(clk[uart6_mux], "uart6_mux", NULL);
	clk_register_clkdev(clk[uart6_div], "uart6_div", NULL);
	clk_register_clkdev(clk[uart6_gate],"uart6_gate", NULL);
	clk_register_clkdev(clk[uart7_mux], "uart7_mux", NULL);
	clk_register_clkdev(clk[uart7_div], "uart7_div", NULL);
	clk_register_clkdev(clk[uart7_gate],"uart7_gate", NULL);
	clk_register_clkdev(clk[uart8_mux], "uart8_mux", NULL);
	clk_register_clkdev(clk[uart8_div], "uart8_div", NULL);
	clk_register_clkdev(clk[uart8_gate],"uart8_gate", NULL);
	clk_register_clkdev(clk[uart9_mux], "uart9_mux", NULL);
	clk_register_clkdev(clk[uart9_div], "uart9_div", NULL);
	clk_register_clkdev(clk[uart9_gate],"uart9_gate", NULL);
	clk_register_clkdev(clk[uart10_mux], "uart10_mux", NULL);
	clk_register_clkdev(clk[uart10_div], "uart10_div", NULL);
	clk_register_clkdev(clk[uart10_gate],"uart10_gate", NULL);
	clk_register_clkdev(clk[uart11_mux], "uart11_mux", NULL);
	clk_register_clkdev(clk[uart11_div], "uart11_div", NULL);
	clk_register_clkdev(clk[uart11_gate],"uart11_gate", NULL);
	clk_register_clkdev(clk[uart12_mux], "uart12_mux", NULL);
	clk_register_clkdev(clk[uart12_div], "uart12_div", NULL);
	clk_register_clkdev(clk[uart12_gate],"uart12_gate", NULL);
	clk_register_clkdev(clk[uart13_mux], "uart13_mux", NULL);
	clk_register_clkdev(clk[uart13_div], "uart13_div", NULL);
	clk_register_clkdev(clk[uart13_gate],"uart13_gate", NULL);
	clk_register_clkdev(clk[uart14_mux], "uart14_mux", NULL);
	clk_register_clkdev(clk[uart14_div], "uart14_div", NULL);
	clk_register_clkdev(clk[uart14_gate],"uart14_gate", NULL);
	clk_register_clkdev(clk[uart15_mux], "uart15_mux", NULL);
	clk_register_clkdev(clk[uart15_div], "uart15_div", NULL);
	clk_register_clkdev(clk[uart15_gate],"uart15_gate", NULL);
	clk_register_clkdev(clk[uart16_mux], "uart16_mux", NULL);
	clk_register_clkdev(clk[uart16_div], "uart16_div", NULL);
	clk_register_clkdev(clk[uart16_gate],"uart16_gate", NULL);

	clk_register_clkdev(clk[rtc_gate], "rtc_gate", NULL);

	clk_register_clkdev(clk[ddr_gate], "ddr_gate", NULL);

	clk_register_clkdev(clk[kpi_mux], "kpi_mux", NULL);
	clk_register_clkdev(clk[kpi_div], "kpi_div", NULL);
	clk_register_clkdev(clk[kpi_gate],"kpi_gate", NULL);

	clk_register_clkdev(clk[i2c0_gate], "i2c0_gate", NULL);
	clk_register_clkdev(clk[i2c1_gate], "i2c1_gate", NULL);
	clk_register_clkdev(clk[i2c2_gate], "i2c2_gate", NULL);
	clk_register_clkdev(clk[i2c3_gate], "i2c3_gate", NULL);
	clk_register_clkdev(clk[i2c4_gate], "i2c4_gate", NULL);
	clk_register_clkdev(clk[i2c5_gate], "i2c5_gate", NULL);

	clk_register_clkdev(clk[qspi0_mux], "qspi0_mux", NULL);
	clk_register_clkdev(clk[qspi0_gate],"qspi0_gate", NULL);
	clk_register_clkdev(clk[qspi1_mux], "qspi1_mux", NULL);
	clk_register_clkdev(clk[qspi1_gate],"qspi1_gate", NULL);

	clk_register_clkdev(clk[smc0_mux], "smc0_mux", NULL);
	clk_register_clkdev(clk[smc0_div], "smc0_div", NULL);
	clk_register_clkdev(clk[smc0_gate],"smc0_gate", NULL);
	clk_register_clkdev(clk[smc1_mux], "smc1_mux", NULL);
	clk_register_clkdev(clk[smc1_div], "smc1_div", NULL);
	clk_register_clkdev(clk[smc1_gate],"smc1_gate", NULL);

	clk_register_clkdev(clk[wdt0_mux], "wdt0_mux", NULL);
	clk_register_clkdev(clk[wdt0_gate],"wdt0_gate", NULL);
	clk_register_clkdev(clk[wdt1_mux], "wdt1_mux", NULL);
	clk_register_clkdev(clk[wdt1_gate],"wdt1_gate", NULL);
	clk_register_clkdev(clk[wdt2_mux], "wdt2_mux", NULL);
	clk_register_clkdev(clk[wdt2_gate],"wdt2_gate", NULL);

	clk_register_clkdev(clk[wwdt0_mux], "wwdt0_mux", NULL);
	clk_register_clkdev(clk[wwdt1_mux], "wwdt1_mux", NULL);
	clk_register_clkdev(clk[wwdt2_mux], "wwdt2_mux", NULL);

	clk_register_clkdev(clk[epwm0_gate], "epwm0_gate", NULL);
	clk_register_clkdev(clk[epwm1_gate], "epwm1_gate", NULL);
	clk_register_clkdev(clk[epwm2_gate], "epwm2_gate", NULL);

	clk_register_clkdev(clk[i2s0_mux], "i2s0_mux", NULL);
	clk_register_clkdev(clk[i2s0_gate],"i2s0_gate", NULL);
	clk_register_clkdev(clk[i2s1_mux], "i2s1_mux", NULL);
	clk_register_clkdev(clk[i2s1_gate],"i2s1_gate", NULL);

	clk_register_clkdev(clk[ssmcc_gate], "ssmcc_gate", NULL);

	clk_register_clkdev(clk[sspcc_gate], "sspcc_gate", NULL);

	clk_register_clkdev(clk[spi0_mux], "spi0_mux", NULL);
	clk_register_clkdev(clk[spi0_gate],"spi0_gate", NULL);
	clk_register_clkdev(clk[spi1_mux], "spi1_mux", NULL);
	clk_register_clkdev(clk[spi1_gate],"spi1_gate", NULL);
	clk_register_clkdev(clk[spi2_mux], "spi2_mux", NULL);
	clk_register_clkdev(clk[spi2_gate],"spi2_gate", NULL);
	clk_register_clkdev(clk[spi3_mux], "spi3_mux", NULL);
	clk_register_clkdev(clk[spi3_gate],"spi3_gate", NULL);

	clk_register_clkdev(clk[ecap0_gate], "ecap0_gate", NULL);
	clk_register_clkdev(clk[ecap1_gate], "ecap1_gate", NULL);
	clk_register_clkdev(clk[ecap2_gate], "ecap2_gate", NULL);

	clk_register_clkdev(clk[qei0_gate], "qei0_gate", NULL);
	clk_register_clkdev(clk[qei1_gate], "qei1_gate", NULL);
	clk_register_clkdev(clk[qei2_gate], "qei2_gate", NULL);

	clk_register_clkdev(clk[adc_div],  "adc_div", NULL);
	clk_register_clkdev(clk[adc_gate], "adc_gate", NULL);

	clk_register_clkdev(clk[eadc_div],  "eadc_div", NULL);
	clk_register_clkdev(clk[eadc_gate], "eadc_gate", NULL);

	clk_data.clks = clk;
	clk_data.clk_num = (clk_max-3);//ARRAY_SIZE(clk);  todo
	of_clk_add_provider(clk_node, of_clk_src_onecell_get, &clk_data);

	// enable some important clocks
	for (i = 0; i < can0_mux; i++)
		clk_prepare_enable(clk[i]);

	clk_set_rate(clk[capll], pllfreq[0]);
	clk_set_rate(clk[syspll], pllfreq[1]);
	clk_set_rate(clk[ddrpll], pllfreq[2]);
	clk_set_rate(clk[apll], pllfreq[3]);
	clk_set_rate(clk[epll], pllfreq[4]);
	clk_set_rate(clk[vpll], pllfreq[5]);

	pr_debug("apll = %ld, vpll =%ld\n", clk_get_rate(clk[apll]), clk_get_rate(clk[vpll]));

}

CLK_OF_DECLARE(ma35d1_init_clk, "nuvoton,ma35d1-clk", ma35d1_init_clocks);



