/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2021 Nuvoton Technology Corp.
 *
 */
#ifndef __SOC_MA35d1_SIP_H
#define __SOC_MA35d1_SIP_H

#define MA35D1_SIP_PMIC	0xC2000008
#define SIP_CPU_CLK			0xC2000009
#define MA35D1_SVC_VERSION	0xC200000F

#define MA35d1_SIP_PMIC_CPU	0x01
#define MA35d1_SIP_PMIC_SD	0x02

enum {
        VOL_1_00 = 10,
        VOL_1_10,
        VOL_1_15,
        VOL_1_20,
        VOL_1_25,
        VOL_1_29,
        VOL_1_30,
        VOL_1_80,
        VOL_3_30
};

#endif
