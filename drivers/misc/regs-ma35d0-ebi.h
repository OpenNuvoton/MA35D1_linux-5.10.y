/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Header file for the MA35D0 EBI driver
 *
 * Copyright (C) 2020 Nuvoton Technology corporation
 */

#ifndef __REGS_MA35D0_EBI_H__
#define __REGS_MA35D0_EBI_H__

#define REG_EBI_CTL 	        (0x00)    /* External Bus Interface Bank0~2 Control Register */
#define REG_EBI_TCTL 	        (0x04)    /* External Bus Interface Bank0~2 Timing Control Register */

#define EBI_BANK0_BASE_ADDR     0x68000000UL /*!< EBI bank0 base address */
#define EBI_BANK1_BASE_ADDR     0x68100000UL /*!< EBI bank1 base address */
#define EBI_BANK2_BASE_ADDR     0x68200000UL /*!< EBI bank2 base address */

#define EBI_BANK0               0UL    /*!< EBI bank 0 */
#define EBI_BANK1               1UL    /*!< EBI bank 1 */
#define EBI_BANK2               2UL    /*!< EBI bank 2 */

#define EBI_BUSWIDTH_8BIT       0UL    /*!< EBI bus width is 8-bit 	*/
#define EBI_BUSWIDTH_16BIT      1UL    /*!< EBI bus width is 16-bit	 */

#define EBI_CS_ACTIVE_LOW       0UL    /*!< EBI CS active level is low  */
#define EBI_CS_ACTIVE_HIGH      1UL    /*!< EBI CS active level is high */

#define EBI_MCLKDIV_1           0x0UL /*!< EBI output clock(MCLK) is HCLK/1 */
#define EBI_MCLKDIV_2           0x1UL /*!< EBI output clock(MCLK) is HCLK/2 */
#define EBI_MCLKDIV_4           0x2UL /*!< EBI output clock(MCLK) is HCLK/4 */
#define EBI_MCLKDIV_8           0x3UL /*!< EBI output clock(MCLK) is HCLK/8 */
#define EBI_MCLKDIV_16          0x4UL /*!< EBI output clock(MCLK) is HCLK/16 */
#define EBI_MCLKDIV_32          0x5UL /*!< EBI output clock(MCLK) is HCLK/32 */
#define EBI_MCLKDIV_64          0x6UL /*!< EBI output clock(MCLK) is HCLK/64 */
#define EBI_MCLKDIV_128         0x7UL /*!< EBI output clock(MCLK) is HCLK/128 */

#define EBI_OPMODE_NORMAL       0x0UL /*!< EBI bus operate in normal mode */
#define EBI_OPMODE_ADSEPARATE   0x1UL /*!< EBI bus operate in AD Separate mode */
#define EBI_OPMODE_CACCESS      (EBI_CTL_CACCESS_Msk) /*!< EBI bus operate in Continuous Data Access mode */


#define EBI_CTL_EN_Pos                   (0)                                               /*!< EBI_T::CTL: EN Position                */
#define EBI_CTL_EN_Msk                   (0x1ul << EBI_CTL_EN_Pos)                         /*!< EBI_T::CTL: EN Mask                    */

#define EBI_CTL_DW16_Pos                 (1)                                               /*!< EBI_T::CTL: DW16 Position              */
#define EBI_CTL_DW16_Msk                 (0x1ul << EBI_CTL_DW16_Pos)                       /*!< EBI_T::CTL: DW16 Mask                  */

#define EBI_CTL_CSPOLINV_Pos             (2)                                               /*!< EBI_T::CTL: CSPOLINV Position          */
#define EBI_CTL_CSPOLINV_Msk             (0x1ul << EBI_CTL_CSPOLINV_Pos)                   /*!< EBI_T::CTL: CSPOLINV Mask              */

#define EBI_CTL_ADSEPEN_Pos              (3)                                               /*!< EBI_T::CTL: ADSEPEN Position           */
#define EBI_CTL_ADSEPEN_Msk              (0x1ul << EBI_CTL_ADSEPEN_Pos)                    /*!< EBI_T::CTL: ADSEPEN Mask               */

#define EBI_CTL_CACCESS_Pos              (4)                                               /*!< EBI_T::CTL: CACCESS Position           */
#define EBI_CTL_CACCESS_Msk              (0x1ul << EBI_CTL_CACCESS_Pos)                    /*!< EBI_T::CTL: CACCESS Mask               */

#define EBI_CTL_MCLKDIV_Pos              (8)                                               /*!< EBI_T::CTL: MCLKDIV Position           */
#define EBI_CTL_MCLKDIV_Msk              (0x7ul << EBI_CTL_MCLKDIV_Pos)                    /*!< EBI_T::CTL: MCLKDIV Mask               */

#define EBI_CTL_TALE_Pos                 (16)                                              /*!< EBI_T::CTL: TALE Position              */
#define EBI_CTL_TALE_Msk                 (0x7ul << EBI_CTL_TALE_Pos)                       /*!< EBI_T::CTL: TALE Mask                  */

#define EBI_CTL_WBUFEN_Pos               (24)                                              /*!< EBI_T::CTL: WBUFEN Position            */
#define EBI_CTL_WBUFEN_Msk               (0x1ul << EBI_CTL_WBUFEN_Pos)                     /*!< EBI_T::CTL: WBUFEN Mask                */

#define EBI_TCTL_TACC_Pos                (3)                                               /*!< EBI_T::TCTL: TACC Position             */
#define EBI_TCTL_TACC_Msk                (0x1ful << EBI_TCTL_TACC_Pos)                     /*!< EBI_T::TCTL: TACC Mask                 */

#define EBI_TCTL_TAHD_Pos                (8)                                               /*!< EBI_T::TCTL: TAHD Position             */
#define EBI_TCTL_TAHD_Msk                (0x7ul << EBI_TCTL_TAHD_Pos)                      /*!< EBI_T::TCTL: TAHD Mask                 */

#define EBI_TCTL_W2X_Pos                 (12)                                              /*!< EBI_T::TCTL: W2X Position              */
#define EBI_TCTL_W2X_Msk                 (0xful << EBI_TCTL_W2X_Pos)                       /*!< EBI_T::TCTL: W2X Mask                  */

#define EBI_TCTL_RAHDOFF_Pos             (22)                                              /*!< EBI_T::TCTL: RAHDOFF Position          */
#define EBI_TCTL_RAHDOFF_Msk             (0x1ul << EBI_TCTL_RAHDOFF_Pos)                   /*!< EBI_T::TCTL: RAHDOFF Mask              */

#define EBI_TCTL_WAHDOFF_Pos             (23)                                              /*!< EBI_T::TCTL: WAHDOFF Position          */
#define EBI_TCTL_WAHDOFF_Msk             (0x1ul << EBI_TCTL_WAHDOFF_Pos)                   /*!< EBI_T::TCTL: WAHDOFF Mask              */

#define EBI_TCTL_R2R_Pos                 (24)                                              /*!< EBI_T::TCTL: R2R Position              */
#define EBI_TCTL_R2R_Msk                 (0xful << EBI_TCTL_R2R_Pos)                       /*!< EBI_T::TCTL: R2R Mask                  */

#endif /*  __REGS_MA35D0_EBI_H__ */
