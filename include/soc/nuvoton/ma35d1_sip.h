/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 */
#ifndef __SOC_MA35d1_SIP_H
#define __SOC_MA35d1_SIP_H

#define MA35D1_SIP_PMIC			0xC2000008
#define MA35D1_SIP_CPU_CLK		0xC2000009
#define MA35D1_SIP_EPLL			0xC200000A
#define MA35D1_SIP_LSPD			0xC200000B
#define MA35D1_SIP_CHIP_RESET		0xC200000D
#define MA35D1_SVC_VERSION		0xC200000F

#define MA35d1_SIP_EPLL_DIV_2		0x02
#define MA35d1_SIP_EPLL_DIV_4		0x04
#define MA35d1_SIP_EPLL_DIV_8		0x08
#define MA35d1_SIP_EPLL_RESTORE		0x0F

#define MA35d1_SIP_PMIC_CPU		0x01
#define MA35d1_SIP_PMIC_SD		0x02

enum {
	VOL_1_00 = 100,
	VOL_1_10 = 110,
	VOL_1_15 = 115,
	VOL_1_20 = 120,
	VOL_1_25 = 125,
	VOL_1_29 = 129,
	VOL_1_30 = 130,
	VOL_1_32 = 132,
	VOL_1_34 = 134,
	VOL_1_36 = 136,
	VOL_1_80 = 180,
	VOL_3_30 = 330
};

#endif
