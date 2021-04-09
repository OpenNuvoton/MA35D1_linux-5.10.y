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

#ifndef __ASM_ARCH_REGS_CLOCK_H
#define __ASM_ARCH_REGS_CLOCK_H

/* Clock Control Registers  */
#define REG_CLK_PWRCTL      (0x00)    /* System Power-down Control Register */
#define REG_CLK_SYSCLK0     (0x04)    /* AXI and AHB Device Clock Enable Control Register 0 */
#define REG_CLK_SYSCLK1     (0x08)    /* AXI and AHB Device Clock Enable Control Register 1 */
#define REG_CLK_APBCLK0     (0x0C)    /* APB Devices Clock Enable Control Register 0 */
#define REG_CLK_APBCLK1     (0x10)    /* APB Devices Clock Enable Control Register 1 */
#define REG_CLK_APBCLK2     (0x14)    /* APB Devices Clock Enable Control Register 2 */
#define REG_CLK_CLKSEL0     (0x18)    /* Clock Source Select Control Register 0 */
#define REG_CLK_CLKSEL1     (0x1C)    /* Clock Source Select Control Register 1 */
#define REG_CLK_CLKSEL2     (0x20)    /* Clock Source Select Control Register 2 */
#define REG_CLK_CLKSEL3     (0x24)    /* Clock Source Select Control Register 3 */
#define REG_CLK_CLKSEL4     (0x28)    /* Clock Source Select Control Register 4 */
#define REG_CLK_CLKDIV0     (0x2C)    /* Clock Divider Number Register 0 */
#define REG_CLK_CLKDIV1     (0x30)    /* Clock Divider Number Register 1 */
#define REG_CLK_CLKDIV2     (0x34)    /* Clock Divider Number Register 2 */
#define REG_CLK_CLKDIV3     (0x38)    /* Clock Divider Number Register 3 */
#define REG_CLK_CLKDIV4     (0x3C)    /* Clock Divider Number Register 4 */
#define REG_CLK_CLKOCTL     (0x40)    /* Clock Output Control Register (Write Protect) */
#define REG_CLK_STATUS      (0x50)    /* Clock Status Monitor Register  */
#define REG_CLK_PLL0CTL0    (0x60)    /* CA-pLL Control Register 0(Write Protect) */
#define REG_CLK_PLL0CTL1    (0x64)    /* CA-pLL Control Register 1(Write Protect) */
#define REG_CLK_PLL0CTL2    (0x68)    /* CA-pLL Control Register 2(Write Protect) */
#define REG_CLK_PLL1CTL0    (0x70)    /* SYS-pLL Control Register 0(Write Protect) */
#define REG_CLK_PLL1CTL1    (0x74)    /* SYS-pLL Control Register 1(Write Protect) */
#define REG_CLK_PLL1CTL2    (0x78)    /* SYS-pLL Control Register 2(Write Protect) */
#define REG_CLK_PLL2CTL0    (0x80)    /* DDR-pLL Control Register 0(Write Protect) */
#define REG_CLK_PLL2CTL1    (0x84)    /* DDR-pLL Control Register 1(Write Protect) */
#define REG_CLK_PLL2CTL2    (0x88)    /* DDR-pLL Control Register 2(Write Protect) */
#define REG_CLK_PLL3CTL0    (0x90)    /* APLL Control Register 0(Write Protect) */
#define REG_CLK_PLL3CTL1    (0x94)    /* APLL Control Register 1(Write Protect) */
#define REG_CLK_PLL3CTL2    (0x98)    /* APLL Control Register 2(Write Protect) */
#define REG_CLK_PLL4CTL0    (0xA0)    /* EPLL Control Register 0(Write Protect) */
#define REG_CLK_PLL4CTL1    (0xA4)    /* EPLL Control Register 1(Write Protect) */
#define REG_CLK_PLL4CTL2    (0xA8)    /* EPLL Control Register 2(Write Protect) */
#define REG_CLK_PLL5CTL0    (0xB0)    /* VPLL Control Register 0(Write Protect) */
#define REG_CLK_PLL5CTL1    (0xB4)    /* VPLL Control Register 1(Write Protect) */
#define REG_CLK_PLL5CTL2    (0xB8)    /* VPLL Control Register 2(Write Protect) */
#define REG_CLK_CLKDCTL     (0xC0)    /* Clock Fail Detector Control Register(Write Protect) */
#define REG_CLK_CLKDSTS     (0xC4)    /* Clock Fail Detector Status Register(Write Protect) */
#define REG_CLK_CDUPB       (0xC8)    /* Clock Frequency Detector Upper Boundary Register(Write Protect) */
#define REG_CLK_CDLOWB      (0xCC)    /* Clock Frequency Detector Lower Boundary Register(Write Protect) */
#define REG_CLK_CKFLTRCTL   (0xD0)    /* Clock Filter Control Register (Write Protect) */
#define REG_CLK_TESTCLK     (0xF0)    /* Test Clock Control Register */


#endif /*  __ASM_ARCH_REGS_CLOCK_H */
