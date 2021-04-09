/*
 *
 * Copyright (c) 2020 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_TIMER_H
#define __ASM_ARCH_REGS_TIMER_H

#define REG_TIMER_CTL          (0x00)    /* Timer Control Register */
#define REG_TIMER_CMP          (0x04)    /* Timer Comparator Register */
#define REG_TIMER_INTSTS       (0x08)    /* Timer Interrupt Status Register */
#define REG_TIMER_CNT          (0x0C)    /* Timer Data Register */
#define REG_TIMER_CAP          (0x10)    /* Timer Capture Data Register */
#define REG_TIMER_EXTCTL       (0x14)    /* Timer External Control Register */
#define REG_TIMER_EINTSTS      (0x18)    /* Timer External Interrupt Status Register */
#define REG_TIMER_TRGCTL       (0x1C)    /* Timer Trigger Control Register */
#define REG_TIMER_ALTCTL       (0x20)    /* Timer Alternative Control Register */
#define REG_TIMER_PWMCTL       (0x40)    /* Timer PWM Control Register */
#define REG_TIMER_PWMCLKSRC    (0x44)    /* Timer PWM Counter Clock Source Register */
#define REG_TIMER_PWMCLKPSC    (0x48)    /* Timer PWM Counter Clock Pre-scale Register */
#define REG_TIMER_PWMCNTCLR    (0x4C)    /* Timer PWM Clear Counter Register */
#define REG_TIMER_PWMPERIOD    (0x50)    /* Timer PWM Period Register */
#define REG_TIMER_PWMCMPDAT    (0x54)    /* Timer PWM Comparator Register */
#define REG_TIMER_PWMDTCTL     (0x58)    /* Timer PWM Dead-time Control Register */
#define REG_TIMER_PWMCNT       (0x5C)    /* Timer PWM Counter Register */
#define REG_TIMER_PWMMSKEN     (0x60)    /* Timer PWM Output Mask Enable Register */
#define REG_TIMER_PWMMSK       (0x64)    /* Timer PWM Output Mask Data Control Register */
#define REG_TIMER_PWMBNF       (0x68)    /* Timer PWM Brake Pin Noise Filter Register */
#define REG_TIMER_PWMFAILBRK   (0x6C)    /* Timer PWM System Fail Brake Control Register */
#define REG_TIMER_PWMBRKCTL    (0x70)    /* Timer PWM Brake Control Register */
#define REG_TIMER_PWMPOLCTL    (0x74)    /* Timer PWM Pin Output Polar Control Register */
#define REG_TIMER_PWMPOEN      (0x78)    /* Timer PWM Pin Output Enable Register */
#define REG_TIMER_PWMSWBRK     (0x7C)    /* Timer PWM Software Trigger Brake Control Register */
#define REG_TIMER_PWMINTEN0    (0x80)    /* Timer PWM Interrupt Enable Register 0 */
#define REG_TIMER_PWMINTEN1    (0x84)    /* Timer PWM Interrupt Enable Register 1 */
#define REG_TIMER_PWMINTSTS0   (0x88)    /* Timer PWM Interrupt Status Register 0 */
#define REG_TIMER_PWMINTSTS1   (0x8C)    /* Timer PWM Interrupt Status Register 1 */
#define REG_TIMER_PWMEADCTS    (0x90)    /* Timer PWM EADC Trigger Source Select Register */
#define REG_TIMER_PWMSCTL      (0x94)    /* Timer PWM Synchronous Control Register */
#define REG_TIMER_PWMSTRG      (0x98)    /* Timer PWM Synchronous Trigger Register */
#define REG_TIMER_PWMSTATUS    (0x9C)    /* Timer PWM Status Register */
#define REG_TIMER_PWMPBUF      (0xA0)    /* Timer PWM Period Buffer Register */
#define REG_TIMER_PWMCMPBUF    (0xA4)    /* Timer PWM Comparator Buffer Register */

#endif /*  __ASM_ARCH_REGS_TIMER_H */
