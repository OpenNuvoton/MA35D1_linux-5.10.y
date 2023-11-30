#ifndef __LINUX_MFD_MA35D0_SYS_H
#define __LINUX_MFD_MA35D0_SYS_H

#include <linux/clk.h>
#include <linux/regmap.h>

#define REG_SYS_PDID            (0x000)    /* Product and Device Identifier Register (TZNS) */
#define REG_SYS_PWRONOTP        (0x004)    /* Power-on Setting OTP Source Register (TZNS) */
#define REG_SYS_PWRONPIN        (0x008)    /* Power-on Setting Pin Source Register (TZNS) */
#define REG_SYS_RSTSTS          (0x010)    /* Reset Source Active Status Register (Shared) */
#define REG_SYS_MISCRFCR        (0x014)    /* Miscellaneous Reset Function Control Register */
#define REG_SYS_RSTDEBCTL       (0x018)    /* Reset Pin De-bounce Control Register */
#define REG_SYS_LVRDCR          (0x01C)    /* Low Voltage Reset & Detect Control Register */
#define REG_SYS_IPRST0          (0x020)    /* Reset Control Register 0 (Shared) */
#define REG_SYS_IPRST1          (0x024)    /* Reset Control Register 1 (Shared) */
#define REG_SYS_IPRST2          (0x028)    /* Reset Control Register 2 (Shared) */
#define REG_SYS_IPRST3          (0x02C)    /* Reset Control Register 3 (Shared) */
#define REG_SYS_PMUCR           (0x030)    /* Power Management Unit Control Register */
#define REG_SYS_DDRCQCSR        (0x034)    /* DDR Controller Q Channel Control and Status Register */
#define REG_SYS_PMUIEN          (0x038)    /* Power Management Unit Interrupt Enable Register */
#define REG_SYS_PMUSTS          (0x03C)    /* Power Management Unit Status Register */
#define REG_SYS_CA35WRBADR1     (0x040)    /* Cortex®-A35 Core 1 Warm-boot Address Register */
#define REG_SYS_CA35WRBPAR1     (0x044)    /* Cortex®-A35 Core 1 Warm-boot Parameter Register */
#define REG_SYS_CA35WRBADR2     (0x048)    /* Cortex®-A35 Core 2 Warm-boot Address Register */
#define REG_SYS_CA35WRBPAR2     (0x04C)    /* Cortex®-A35 Core 2 Warm-boot Parameter Register */
#define REG_SYS_USBPMISCR       (0x060)    /* USB PHY Miscellaneous Control Register (TZNS) */
#define REG_SYS_USBP0PCR        (0x064)    /* USB Port 0 PHY Control Register */
#define REG_SYS_USBP1PCR        (0x068)    /* USB Port 1 PHY Control Register (TZNS) */
#define REG_SYS_MISCFCR0        (0x070)    /* Miscellaneous Function Control Register 0 (TZNS) */
#define REG_SYS_MISCFCR1        (0x074)    /* Miscellaneous Function Control Register 1 (Shared) */
#define REG_SYS_MISCIER         (0x078)    /* Miscellaneous Interrupt Enable Register (TZNS) */
#define REG_SYS_MISCISR         (0x07C)    /* Miscellaneous Interrupt Status Register (TZNS) */
#define REG_SYS_GPA_MFPL        (0x080)    /* GPIOA Low Byte Multiple Function Control Register */
#define REG_SYS_GPA_MFPH        (0x084)    /* GPIOA High Byte Multiple Function Control Register */
#define REG_SYS_GPB_MFPL        (0x088)    /* GPIOB Low Byte Multiple Function Control Register */
#define REG_SYS_GPB_MFPH        (0x08C)    /* GPIOB High Byte Multiple Function Control Register */
#define REG_SYS_GPC_MFPL        (0x090)    /* GPIOC Low Byte Multiple Function Control Register */
#define REG_SYS_GPC_MFPH        (0x094)    /* GPIOC High Byte Multiple Function Control Register */
#define REG_SYS_GPD_MFPL        (0x098)    /* GPIOD Low Byte Multiple Function Control Register */
#define REG_SYS_GPD_MFPH        (0x09C)    /* GPIOD High Byte Multiple Function Control Register */
#define REG_SYS_GPE_MFPL        (0x0A0)    /* GPIOE Low Byte Multiple Function Control Register */
#define REG_SYS_GPE_MFPH        (0x0A4)    /* GPIOE High Byte Multiple Function Control Register */
#define REG_SYS_GPF_MFPL        (0x0A8)    /* GPIOF Low Byte Multiple Function Control Register */
#define REG_SYS_GPF_MFPH        (0x0AC)    /* GPIOF High Byte Multiple Function Control Register */
#define REG_SYS_GPG_MFPL        (0x0B0)    /* GPIOG Low Byte Multiple Function Control Register */
#define REG_SYS_GPG_MFPH        (0x0B4)    /* GPIOG High Byte Multiple Function Control Register */
#define REG_SYS_GPH_MFPL        (0x0B8)    /* GPIOH Low Byte Multiple Function Control Register */
#define REG_SYS_GPH_MFPH        (0x0BC)    /* GPIOH High Byte Multiple Function Control Register */
#define REG_SYS_GPI_MFPL        (0x0C0)    /* GPIOI Low Byte Multiple Function Control Register */
#define REG_SYS_GPI_MFPH        (0x0C4)    /* GPIOI High Byte Multiple Function Control Register */
#define REG_SYS_GPJ_MFPL        (0x0C8)    /* GPIOJ Low Byte Multiple Function Control Register */
#define REG_SYS_GPJ_MFPH        (0x0CC)    /* GPIOJ High Byte Multiple Function Control Register */
#define REG_SYS_GPK_MFPL        (0x0D0)    /* GPIOK Low Byte Multiple Function Control Register */
#define REG_SYS_GPK_MFPH        (0x0D4)    /* GPIOK High Byte Multiple Function Control Register */
#define REG_SYS_GPL_MFPL        (0x0D8)    /* GPIOL Low Byte Multiple Function Control Register */
#define REG_SYS_GPL_MFPH        (0x0DC)    /* GPIOL High Byte Multiple Function Control Register */
#define REG_SYS_GPM_MFPL        (0x0E0)    /* GPIOM Low Byte Multiple Function Control Register */
#define REG_SYS_GPM_MFPH        (0x0E4)    /* GPIOM High Byte Multiple Function Control Register */
#define REG_SYS_GPN_MFPL        (0x0E8)    /* GPION Low Byte Multiple Function Control Register */
#define REG_SYS_GPN_MFPH        (0x0EC)    /* GPION High Byte Multiple Function Control Register */
#define REG_SYS_HIRCFTRIM       (0x100)    /* HIRC Frequency Trim Value Register */
#define REG_SYS_TSENSRFCR       (0x104)    /* Temperature Sensor Function Control Register */
#define REG_SYS_GMAC0MISCR      (0x108)    /* GMAC 0 Miscellaneous Control Register (TZNS) */
#define REG_SYS_GMAC1MISCR      (0x10C)    /* GMAC 1 Miscellaneous Control Register (TZNS) */
#define REG_SYS_MACAD0LSR       (0x110)    /* MAC Address 0 Low Significant Word Register (TZNS) */
#define REG_SYS_MACAD0HSR       (0x114)    /* MAC Address 0 High Significant Word Register (TZNS) */
#define REG_SYS_MACAD1LSR       (0x118)    /* MAC Address 1 Low Significant Word Register (TZNS) */
#define REG_SYS_MACAD1HSR       (0x11C)    /* MAC Address 1 High Significant Word Register (TZNS) */
#define REG_SYS_CSDBGCTL        (0x120)    /* CoreSight Debug Control Register */
#define REG_SYS_GPAB_MFOS       (0x140)    /* GPIOA and GPIOB Multiple Function Output Mode Select Register */
#define REG_SYS_GPCD_MFOS       (0x144)    /* GPIOC and GPIOD Multiple Function Output Mode Select Register */
#define REG_SYS_GPEF_MFOS       (0x148)    /* GPIOE and GPIOF Multiple Function Output Mode Select Register */
#define REG_SYS_GPGH_MFOS       (0x14C)    /* GPIOG and GPIOH Multiple Function Output Mode Select Register */
#define REG_SYS_GPIJ_MFOS       (0x150)    /* GPIOI and GPIOJ Multiple Function Output Mode Select Register */
#define REG_SYS_GPKL_MFOS       (0x154)    /* GPIOK and GPIOL Multiple Function Output Mode Select Register */
#define REG_SYS_GPMN_MFOS       (0x158)    /* GPIOM and GPION Multiple Function Output Mode Select Register */
#define REG_SYS_UID0            (0x180)    /* Unique Identifier Word 0 Register (TZNS) */
#define REG_SYS_UID1            (0x184)    /* Unique Identifier Word 1 Register (TZNS) */
#define REG_SYS_UID2            (0x188)    /* Unique Identifier Word 2 Register (TZNS) */
#define REG_SYS_UCID0           (0x190)    /* Unique Customer Identifier Word 0 Register (TZNS) */
#define REG_SYS_UCID1           (0x194)    /* Unique Customer Identifier Word 1 Register (TZNS) */
#define REG_SYS_UCID2           (0x198)    /* Unique Customer Identifier Word 2 Register (TZNS) */
#define REG_SYS_RLKTZS          (0x1A0)    /* TZS Register Lock Control Register */
#define REG_SYS_RLKTZNS         (0x1A4)    /* TZNS Register Lock Control Register (TZNS) */
#define REG_SYS_RLKSUBM         (0x1A8)    /* SubM Register Lock Control Register (SubM) */
#define REG_SYS_DPLPASWD        (0x1B0)    /* Deployed Password Register */

void ma35d0_reg_lock(void);
void ma35d0_reg_unlock(void);

#endif /* __LINUX_MFD_MA35D0_SYS_H */
