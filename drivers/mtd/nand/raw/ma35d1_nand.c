// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2020 Nuvoton technology corporation.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/rawnand.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/ma35d1-sys.h>

/* NFI Registers */
#define REG_NAND_FB0		(0x000)	/* DMAC Control and Status Register */
#define REG_NAND_DMACCSR	(0x400)	/* DMAC Control and Status Register */
#define REG_NAND_DMACSAR	(0x408)	/* DMAC Transfer Starting Address Register */
#define REG_NAND_DMACBCR	(0x40C)	/* DMAC Transfer Byte Count Register */
#define REG_NAND_DMACIER	(0x410)	/* DMAC Interrupt Enable Register */
#define REG_NAND_DMACISR	(0x414)	/* DMAC Interrupt Status Register */

#define REG_NAND_FMICSR		(0x800)	/* Global Control and Status Register */
#define REG_NAND_FMIIER		(0x804)	/* Global Interrupt Control Register */
#define REG_NAND_FMIISR		(0x808)	/* Global Interrupt Status Register */

/* NAND-type Flash Registers */
#define REG_SMCSR		(0x8A0)	/* NAND Flash Control and Status Register */
#define REG_SMTCR		(0x8A4)	/* NAND Flash Timing Control Register */
#define REG_SMIER		(0x8A8)	/* NAND Flash Interrupt Control Register */
#define REG_SMISR		(0x8AC)	/* NAND Flash Interrupt Status Register */
#define REG_SMCMD		(0x8B0)	/* NAND Flash Command Port Register */
#define REG_SMADDR		(0x8B4)	/* NAND Flash Address Port Register */
#define REG_SMDATA		(0x8B8)	/* NAND Flash Data Port Register */
#define REG_SMREACTL		(0x8BC)	/* NAND Flash Smart-Media Redundant Area Control Register */
#define REG_NFECR		(0x8C0)	/* NAND Flash Extend Control Regsiter */
#define REG_SMECC_ST0		(0x8D0)	/* Smart-Media ECC Error Status 0 */
#define REG_SMECC_ST1		(0x8D4)	/* Smart-Media ECC Error Status 1 */
#define REG_SMECC_ST2		(0x8D8)	/* Smart-Media ECC Error Status 2 */
#define REG_SMECC_ST3		(0x8DC)	/* Smart-Media ECC Error Status 3 */

/* NAND-type Flash BCH Error Address Registers */
#define REG_BCH_ECC_ADDR0	(0x900)	/* BCH error byte address 0 */
#define REG_BCH_ECC_ADDR1	(0x904)	/* BCH error byte address 1 */
#define REG_BCH_ECC_ADDR2	(0x908)	/* BCH error byte address 2 */
#define REG_BCH_ECC_ADDR3	(0x90C)	/* BCH error byte address 3 */
#define REG_BCH_ECC_ADDR4	(0x910)	/* BCH error byte address 4 */
#define REG_BCH_ECC_ADDR5	(0x914)	/* BCH error byte address 5 */
#define REG_BCH_ECC_ADDR6	(0x918)	/* BCH error byte address 6 */
#define REG_BCH_ECC_ADDR7	(0x91C)	/* BCH error byte address 7 */
#define REG_BCH_ECC_ADDR8	(0x920)	/* BCH error byte address 8 */
#define REG_BCH_ECC_ADDR9	(0x924)	/* BCH error byte address 9 */
#define REG_BCH_ECC_ADDR10	(0x928)	/* BCH error byte address 10 */
#define REG_BCH_ECC_ADDR11	(0x92C)	/* BCH error byte address 11 */

/* NAND-type Flash BCH Error Data Registers */
#define REG_BCH_ECC_DATA0	(0x960)	/* BCH error byte data 0 */
#define REG_BCH_ECC_DATA1	(0x964)	/* BCH error byte data 1 */
#define REG_BCH_ECC_DATA2	(0x968)	/* BCH error byte data 2 */
#define REG_BCH_ECC_DATA3	(0x96C)	/* BCH error byte data 3 */
#define REG_BCH_ECC_DATA4	(0x970)	/* BCH error byte data 4 */
#define REG_BCH_ECC_DATA5	(0x974)	/* BCH error byte data 5 */

/* NAND-type Flash Redundant Area Registers */
#define REG_SMRA0		(0xA00)	/* Smart-Media Redundant Area Register */
#define REG_SMRA1		(0xA04)	/* Smart-Media Redundant Area Register */

/* FMI Global Control and Status Register(FMICSR) */
#define FMICSR_SWRST		(1)
#define FMICSR_NANDEN		(1<<3)

/* NAND-type Flash Interrupt Control Register(SMIER) */
#define FMIIER_DTAIE		(1)

/* NAND-type Flash Interrupt Status Register (SMISR) */
#define FMIISR_DTAIF		(1)
#define SMISR_RB0IF		(1<<10)

/* DMAC Control and Status Register (DMACCSR) */
#define DMACCSR_DMAC_EN		(1)
#define DMACCSR_SWRST		(1<<1)
#define DMACCSR_SGEN		(1<<3)
#define DMACCSR_FMIBUSY		(1<<9)

/* DMAC Interrupt Enable Register (DMACIER) */
#define DMACIER_TABORTIE	(1)
#define DMACIER_WEOTIE		(1<<1)

/* DMAC Interrupt Status Register (DMACISR) */
#define DMACISR_TABORTIF	(1)
#define DMACISR_WEOTIF		(1<<1)

/************************************************/
#define NAND_RB_TIMEOUT	1000

#define RESET_FMI   0x01
#define NAND_EN     0x08
#define READYBUSY   0x400

#define SWRST       0x01
#define PSIZE       (0x01 << 3)
#define DMARWEN     (0x03 << 1)
#define BUSWID      (0x01 << 4)
#define ECC4EN      (0x01 << 5)
#define WP          (0x01 << 24)
#define NANDCS      (0x01 << 25)
#define ENDADDR     (0x01 << 31)

#define BCH_T8      0x00100000
#define BCH_T12     0x00200000
#define BCH_T24     0x00040000

#define MA35D1_DRV_VERSION "20200921"
#define DEF_RESERVER_OOB_SIZE_FOR_MARKER 4

//#define NVT_NAND_DEBUG

struct ma35d1_nand_info {
	struct nand_controller	controller;
	struct device		*dev;
	void __iomem 		*base;

	struct mtd_info         mtd;
	struct nand_chip        chip;
	struct mtd_partition    *parts;     // mtd partition
	int                     nr_parts;   // mtd partition number
	struct platform_device  *pdev;
	struct clk              *clk;

	int                     eBCHAlgo;
	int                     m_i32SMRASize;

	unsigned char           *dma_buf;

};

static int ma35d1_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section || !ecc->total)
		return -ERANGE;

	oobregion->length = ecc->total;
	oobregion->offset = mtd->oobsize - oobregion->length;

	return 0;
}

static int ma35d1_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;

	if (section)
		return -ERANGE;

	oobregion->length = mtd->oobsize - ecc->total - 2;
	oobregion->offset = 2;

	return 0;
}

static const struct mtd_ooblayout_ops ma35d1_ooblayout_ops = {
	.ecc = ma35d1_ooblayout_ecc,
	.free = ma35d1_ooblayout_free,
};

typedef enum  {
	eBCH_NONE=0,
	eBCH_T8,
	eBCH_T12,
	eBCH_T24,
	eBCH_CNT
} E_BCHALGORITHM;


static const int g_i32BCHAlgoIdx[eBCH_CNT] = { BCH_T8, BCH_T8, BCH_T12, BCH_T24 };
static struct nand_ecclayout_user ma35d1_nand_oob;
static const int g_i32ParityNum[3][eBCH_CNT] = {
	{ 0,  60,  92,  90 },  // for 2K
	{ 0, 120, 184, 180 },  // for 4K
	{ 0, 240, 368, 360 },  // for 8K
};


/*
 * ma35d1_nand_hwecc_init - Initialize hardware ECC IP
 */
static void ma35d1_nand_hwecc_init(struct ma35d1_nand_info *nand)
{
	writel ( readl(nand->base+REG_SMCSR)|0x1, nand->base+REG_SMCSR);    // reset SM controller

	// Redundant area size
	writel( nand->m_i32SMRASize , nand->base+REG_SMREACTL );

	// Protect redundant 3 bytes
	// because we need to implement write_oob function to partial data to oob available area.
	// Please note we skip 4 bytes
	writel( readl(nand->base+REG_SMCSR) | 0x100, nand->base+REG_SMCSR);

	// To read/write the ECC parity codes automatically from/to NAND Flash after data area field written.
	writel( readl(nand->base+REG_SMCSR) | 0x10, nand->base+REG_SMCSR);

	if ( nand->eBCHAlgo == eBCH_NONE ) {
		// Disable H/W ECC / ECC parity check enable bit during read page
		writel( readl(nand->base+REG_SMCSR) & (~0x00800000), nand->base+REG_SMCSR);
	} else  {
		// Set BCH algorithm
		writel( (readl(nand->base+REG_SMCSR) & (~0x007C0000)) | g_i32BCHAlgoIdx[nand->eBCHAlgo], nand->base+REG_SMCSR);

		// Enable H/W ECC, ECC parity check enable bit during read page
		writel( readl(nand->base+REG_SMCSR) | 0x00800000, nand->base+REG_SMCSR);
	}
}

static void ma35d1_nand_initialize (struct ma35d1_nand_info *nand)
{
	// Enable SM_EN
	writel(FMICSR_NANDEN, nand->base+REG_NAND_FMICSR);

	// Enable SM_CS0
	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);

	// NAND Reset
	writel(readl(nand->base+REG_SMCSR) | 0x1, nand->base+REG_SMCSR);    // software reset
}

/*-----------------------------------------------------------------------------
 * Define some constants for BCH
 *---------------------------------------------------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code lenght for 512 bytes data pattern
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
// define the BCH parity code lenght for 1024 bytes data pattern
#define BCH_PARITY_LEN_T24 45


/*-----------------------------------------------------------------------------
 * Correct data by BCH alrogithm.
 *---------------------------------------------------------------------------*/
void fmiSM_CorrectData_BCH(struct ma35d1_nand_info *nand, u8 ucFieidIndex, u8 ucErrorCnt, u8* pDAddr)
{
	u32 uaData[24], uaAddr[24];
	u32 uaErrorData[6];
	u8  ii, jj;
	u32 uPageSize;
	u32 field_len, padding_len, parity_len;
	u32 total_field_num;
	u8  *smra_index;

	//--- assign some parameters for different BCH and page size
	switch (readl(nand->base+REG_SMCSR) & 0x007C0000)
	{
		case BCH_T24:
			field_len   = 1024;
			padding_len = BCH_PADDING_LEN_1024;
			parity_len  = BCH_PARITY_LEN_T24;
			break;
		case BCH_T12:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T12;
			break;
		case BCH_T8:
			field_len   = 512;
			padding_len = BCH_PADDING_LEN_512;
			parity_len  = BCH_PARITY_LEN_T8;
			break;
		default:
			pr_warn("NAND ERROR: %s(): invalid SMCR_BCH_TSEL = 0x%08X\n", __FUNCTION__, (u32)(readl(nand->base+REG_SMCSR) & 0x7C0000));
			return;
	}

	uPageSize = readl(nand->base+REG_SMCSR) & 0x00030000;
	switch (uPageSize)
	{
		case 0x30000:  total_field_num = 8192 / field_len; break;
		case 0x20000:  total_field_num = 4096 / field_len; break;
		case 0x10000:  total_field_num = 2048 / field_len; break;
		default:
			pr_warn("NAND ERROR: %s(): invalid SMCR_PSIZE = 0x%08X\n", __FUNCTION__, uPageSize);
			return;
	}

	//--- got valid BCH_ECC_DATAx and parse them to uaData[]
	// got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
	jj = ucErrorCnt/4;
	jj ++;
	if (jj > 6)
		jj = 6;     // there are 6 BCH_ECC_DATAx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaErrorData[ii] = readl(nand->base+REG_BCH_ECC_DATA0 + ii*4);
	}

	for(ii=0; ii<jj; ii++)
	{
		uaData[ii*4+0] = uaErrorData[ii] & 0xff;
		uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
		uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
		uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
	}

	//--- got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
	// got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
	jj = ucErrorCnt/2;
	jj ++;
	if (jj > 12)
		jj = 12;    // there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24

	for(ii=0; ii<jj; ii++)
	{
		uaAddr[ii*2+0] = readl(nand->base+REG_BCH_ECC_ADDR0 + ii*4) & 0x07ff;   // 11 bits for error address
		uaAddr[ii*2+1] = (readl(nand->base+REG_BCH_ECC_ADDR0 + ii*4)>>16) & 0x07ff;
	}

	//--- pointer to begin address of field that with data error
	pDAddr += (ucFieidIndex-1) * field_len;

	//--- correct each error bytes
	for(ii=0; ii<ucErrorCnt; ii++)
	{
		// for wrong data in field
		if (uaAddr[ii] < field_len)
		{
#ifdef NVT_NAND_DEBUG
			printk("BCH error corrected for data: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(pDAddr+uaAddr[ii]), (unsigned int)(*(pDAddr+uaAddr[ii])));
#endif
			*(pDAddr+uaAddr[ii]) ^= uaData[ii];

#ifdef NVT_NAND_DEBUG
			printk("[0x%02X]\n", *(pDAddr+uaAddr[ii]));
#endif
		}
		// for wrong first-3-bytes in redundancy area
		else if (uaAddr[ii] < (field_len+3))
		{
			uaAddr[ii] -= field_len;
			uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset

#ifdef NVT_NAND_DEBUG
			printk("BCH error corrected for 3 bytes: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)((u8 *)nand->base+REG_SMRA0 + uaAddr[ii]), (unsigned int)(*((u8 *)nand->base+REG_SMRA0 + uaAddr[ii])));
#endif
			*((u8 *)nand->base+REG_SMRA0 + uaAddr[ii]) ^= uaData[ii];

#ifdef NVT_NAND_DEBUG
			printk("[0x%02X]\n", *((u8 *)nand->base+REG_SMRA0+uaAddr[ii]));
#endif
		}
		// for wrong parity code in redundancy area
		else
		{
			// BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
			//                                   |<--     padding bytes      -->|
			// The BCH_ERR_ADDRx for last parity code always = field size + padding size.
			// So, the first parity code = field size + padding size - parity code length.
			// For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
			// That is, error byte address offset within field is
			uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

			// smra_index point to the first parity code of first field in register SMRA0~n
			smra_index = (u8 *)
						 (nand->base+REG_SMRA0 + (readl(nand->base+REG_SMREACTL) & 0x1ff) - // bottom of all parity code -
						  (parity_len * total_field_num)                             // byte count of all parity code
						 );

			// final address = first parity code of first field +
			//                 offset of fields +
			//                 offset within field

#ifdef NVT_NAND_DEBUG
			printk("BCH error corrected for parity: address 0x%08X, data [0x%02X] --> ",
				(unsigned int)(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]),
				(unsigned int)(*(smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii])));
#endif
			*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];

#ifdef NVT_NAND_DEBUG
			printk("[0x%02X]\n",
				*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]));
#endif
		}
	}   // end of for (ii<ucErrorCnt)
}

int fmiSMCorrectData (struct nand_chip *chip, unsigned long uDAddr )
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int uStatus, ii, jj, i32FieldNum=0;
	volatile int uErrorCnt = 0;
	volatile int uReportErrCnt = 0;

	if ( readl ( nand->base+REG_SMISR ) & 0x4 )
	{
		if ( ( readl(nand->base+REG_SMCSR) & 0x7C0000) == BCH_T24 )
			i32FieldNum = mtd->writesize / 1024;    // Block=1024 for BCH
		else
			i32FieldNum = mtd->writesize / 512;

		if ( i32FieldNum < 4 )
			i32FieldNum  = 1;
		else
			i32FieldNum /= 4;

		for ( jj=0; jj<i32FieldNum; jj++ )
		{
			uStatus = readl ( nand->base+REG_SMECC_ST0+jj*4 );
			if ( !uStatus )
				continue;

			for ( ii=1; ii<5; ii++ )
			{
				if ( !(uStatus & 0x03) ) { // No error
					uStatus >>= 8;
					continue;
				} else if ( (uStatus & 0x03)==0x01 ) { // Correctable error
					uErrorCnt = (uStatus >> 2) & 0x1F;
#ifdef NVT_NAND_DEBUG
					printk("Field (%d, %d) have %d error!!\n", jj, ii, uErrorCnt);
#endif
					fmiSM_CorrectData_BCH(nand, jj*4+ii, uErrorCnt, (char*)uDAddr);
					uReportErrCnt += uErrorCnt;
					break;
				} else // uncorrectable error or ECC error
				{
#ifdef NVT_NAND_DEBUG
					printk("SM uncorrectable error is encountered, 0x%4x !!\n", uStatus);
#endif
					return -1;
				}
				uStatus >>= 8;
			}
		} //jj
	}
	return uReportErrCnt;
}

static int ma35d1_nand_correct_data(struct nand_chip *chip, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	return 0;
}


void ma35d1_nand_enable_hwecc(struct nand_chip *chip, int mode)
{

}

/*
 * ma35d1_nand_dmac_init - Initialize dma controller
 */
static void ma35d1_nand_dmac_init( struct ma35d1_nand_info *nand )
{
	// DMAC enable
	writel( readl(nand->base+REG_NAND_DMACCSR) | 0x3, nand->base+REG_NAND_DMACCSR);
	writel( readl(nand->base+REG_NAND_DMACCSR) & (~0x2), nand->base+REG_NAND_DMACCSR);

	// Clear DMA finished flag
	writel( readl(nand->base+REG_SMISR) | 0x1, nand->base+REG_SMISR);

	// Disable Interrupt
	writel(readl(nand->base+REG_SMIER) & ~(0x1), nand->base+REG_SMIER);
}

/*
 * ma35d1_nand_dmac_fini - Finalize dma controller
 */
static void ma35d1_nand_dmac_fini(struct ma35d1_nand_info *nand)
{
	// Clear DMA finished flag
	writel(readl(nand->base+REG_SMISR) | 0x1, nand->base+REG_SMISR);
}

/*
 * ma35d1_nand_read_byte - read a byte from NAND controller into buffer
 */
static unsigned char ma35d1_nand_read_byte(struct nand_chip *chip)
{
	unsigned char ret;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	ret = (unsigned char)readl(nand->base+REG_SMDATA);
	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);

	return ret;
}


/*
 * ma35d1_nand_read_buf - read data from NAND controller into buffer
 */
static void ma35d1_nand_read_buf(struct nand_chip *chip, unsigned char *buf, int len)
{
	int i;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)readl(nand->base+REG_SMDATA);
	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
}
/*
 * ma35d1_nand_write_buf - write data from buffer into NAND controller
 */

static void ma35d1_nand_write_buf(struct nand_chip *chip, const unsigned char *buf, int len)
{
	int i;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	for (i = 0; i < len; i++)
		writel(buf[i], nand->base+REG_SMDATA);
	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
}

/*
 * ma35d1_nand_dma_transfer: configer and start dma transfer
 */
static inline int ma35d1_nand_dma_transfer(struct nand_chip *chip, const u_char *addr, unsigned int len, int is_write)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	dma_addr_t dma_addr;
	int ret;

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	// For save, wait DMAC to ready
	while ( readl(nand->base+REG_NAND_DMACCSR) & 0x200 );

	// Reinitial dmac
	ma35d1_nand_dmac_init(nand);

	// Enable target abort interrupt generation during DMA transfer.
	writel( 0x1, nand->base+REG_NAND_DMACIER);

	// Clear Ready/Busy 0 Rising edge detect flag
	writel(0x400, nand->base+REG_SMISR);

	writel( nand->m_i32SMRASize , nand->base+REG_SMREACTL );

	writel( readl(nand->base+REG_SMIER) & (~0x4), nand->base+REG_SMIER );
	writel ( 0x4, nand->base+REG_SMISR );

	// Enable SM_CS0
	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	/* setup and start DMA using dma_addr */

	if ( is_write ) {
		register char * ptr=nand->base+REG_SMRA0;
		// To mark this page as dirty.
		if ( ptr[3] == 0xFF )
			ptr[3] = 0;
		if ( ptr[2] == 0xFF )
			ptr[2] = 0;

		if ( addr )
			memcpy( (void*)nand->dma_buf, (void*)addr, len);

		// Fill dma_addr
		dma_addr = dma_map_single(nand->dev, (void *)nand->dma_buf, len, DMA_TO_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}

		writel((unsigned long)dma_addr, nand->base+REG_NAND_DMACSAR);
		//dma_sync_single_for_device(nand->dev, dma_addr, len, DMA_TO_DEVICE);

		writel ( readl(nand->base+REG_SMCSR) | 0x4, nand->base+REG_SMCSR );
		while ( !(readl(nand->base+REG_SMISR) & 0x1) );

	} else {
		// Blocking for reading
		// Enable DMA Read

		// Fill dma_addr
		dma_addr = dma_map_single(nand->dev, (void *)nand->dma_buf, len, DMA_FROM_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}

		writel((unsigned long)dma_addr, nand->base+REG_NAND_DMACSAR);

		writel ( readl(nand->base+REG_SMCSR) | 0x2, nand->base+REG_SMCSR);
		if ( readl(nand->base+REG_SMCSR) & 0x80 ) {
			do {
				int stat=0;
				if ( (stat=fmiSMCorrectData (chip,  (unsigned long)nand->dma_buf)) < 0 )
				{
					mtd->ecc_stats.failed++;
					writel ( 0x4, nand->base+REG_SMISR );
					writel ( 0x3, nand->base+REG_NAND_DMACCSR);          // reset DMAC
					writel ( readl(nand->base+REG_SMCSR)|0x1, nand->base+REG_SMCSR);    // reset SM controller
					break;
				}
				else if ( stat > 0 ) {
					mtd->ecc_stats.corrected += stat;   // Add corrected bit count
					writel ( 0x4, nand->base+REG_SMISR );
				}

			} while ( !(readl(nand->base+REG_SMISR) & 0x1) || (readl(nand->base+REG_SMISR) & 0x4) );
		} else
			while ( !(readl(nand->base+REG_SMISR) & 0x1) );

		//dma_sync_single_for_cpu(nand->dev, dma_addr, len, DMA_FROM_DEVICE);
		if ( addr )
			memcpy( (void*)addr, (void*)nand->dma_buf,  len );
	}

	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
	ma35d1_nand_dmac_fini(nand);
	if ( is_write )
		dma_unmap_single(nand->dev, dma_addr, len, DMA_TO_DEVICE);
	else
		dma_unmap_single(nand->dev, dma_addr, len, DMA_FROM_DEVICE);

	return 0;
}

static void ma35d1_read_buf_dma(struct nand_chip *chip, u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		ma35d1_nand_dma_transfer (chip, buf, len, 0x0);
	else {
		ma35d1_nand_read_buf(chip, buf, len);

#ifdef NVT_NAND_DEBUG
		{
		int i;
		printk("R OOB %d\n", len );
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
		printk("\n");
		}
#endif
	}
}

static void ma35d1_write_buf_dma(struct nand_chip *chip, const u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if ( len == mtd->writesize ) /* start transfer in DMA mode */
		ma35d1_nand_dma_transfer(chip, (u_char *)buf, len, 0x1);
	else
	{
#ifdef NVT_NAND_DEBUG
		int i;
		printk("W OOB %d\n", len);
		for ( i=0; i<len; i++ )
		{
			printk("%02X ", buf[i] );
			if ( i%32 == 31 )   printk("\n");
		}
#endif
		ma35d1_nand_write_buf(chip, buf, len);
	}
}


static int ma35d1_check_rb(struct ma35d1_nand_info *nand)
{
	unsigned int volatile val;

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	val = (readl(nand->base+REG_SMISR) & SMISR_RB0IF) ? 1 : 0;
	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);

	return val;
}

static int ma35d1_nand_devready(struct nand_chip *chip)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	int ready;

	ready = (ma35d1_check_rb(nand)) ? 1 : 0;

	return ready;
}

static void ma35d1_nand_command(struct nand_chip *chip, unsigned int command, int column, int page_addr)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	writel(0x400, nand->base+REG_SMISR);

	if ((command == NAND_CMD_STATUS) || (command == NAND_CMD_ERASE1) || (command == NAND_CMD_SEQIN)) {
		writel(0x1, nand->base+REG_NFECR); /* un-lock write protect */
	}

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	writel(command & 0xff, nand->base+REG_SMCMD);

	if (command == NAND_CMD_READID) {
		writel(ENDADDR|column, nand->base+REG_SMADDR);
		return;

	} else if (command == NAND_CMD_PARAM){
		writel(ENDADDR|column, nand->base+REG_SMADDR);

	} else {
		if (column != -1 || page_addr != -1) {
			if (column != -1) {
				writel(column&0xFF, nand->base+REG_SMADDR);
				if ( page_addr != -1 )
					writel(column >> 8, nand->base+REG_SMADDR);
				else
					writel((column >> 8) | ENDADDR, nand->base+REG_SMADDR);

			}

			if (page_addr != -1) {
				writel(page_addr&0xFF, nand->base+REG_SMADDR);

				if ( chip->options & NAND_ROW_ADDR_3) {
					writel((page_addr >> 8)&0xFF, nand->base+REG_SMADDR);
					writel(((page_addr >> 16)&0xFF)|ENDADDR, nand->base+REG_SMADDR);
				} else {
					writel(((page_addr >> 8)&0xFF)|ENDADDR, nand->base+REG_SMADDR);
				}
			}
		}
	}

	switch (command) {
	case NAND_CMD_ERASE1:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
		return;

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE2:
		writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
		break;

	case NAND_CMD_RESET:
		if ( chip->legacy.chip_delay )
			udelay(chip->legacy.chip_delay);

		writel(0x0, nand->base+REG_NFECR); /* lock write protect */
		writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
		break;

	case NAND_CMD_RNDOUT:
		writel(NAND_CMD_RNDOUTSTART, nand->base+REG_SMCMD);
		writel(0x0, nand->base+REG_NFECR); /* lock write protect */
		writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);
		return;

	case NAND_CMD_READ0:
		writel(NAND_CMD_READSTART, nand->base+REG_SMCMD);
		writel(0x0, nand->base+REG_NFECR); /* lock write protect */
		break;
	default:
		writel(0x0, nand->base+REG_NFECR); /* lock write protect */
		if (!chip->legacy.dev_ready) {
			if ( chip->legacy.chip_delay )
				udelay(chip->legacy.chip_delay);
			return;
		}
	}

	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	while (1)
	{
		if (ma35d1_check_rb(nand) == 1)
			break;
		if (time_after(jiffies, jiffies + msecs_to_jiffies(NAND_RB_TIMEOUT))) {
			pr_err("NAND is busy. Timeout! 0x%x\n", readl(nand->base+REG_SMISR));
			break;
		}
	}
	writel(readl(nand->base+REG_SMCSR)|0x02000000, nand->base+REG_SMCSR);

	if ((command == NAND_CMD_STATUS) || (command == NAND_CMD_ERASE1) || (command == NAND_CMD_SEQIN)) {
		writel(0x0, nand->base+REG_NFECR); /* lock write protect */
	}
}

/* select chip */
static void ma35d1_nand_select_chip(struct nand_chip *chip, int cs)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	writel(readl(nand->base+REG_SMCSR) & (~0x02000000), nand->base+REG_SMCSR);
	return;
}

/*
 * Calculate HW ECC
 * function called after a write
 */
static int ma35d1_nand_calculate_ecc(struct nand_chip *chip, const u_char *dat, u_char *ecc_code)
{
	return 0;
}

static int ma35d1_nand_write_page_hwecc(struct nand_chip *chip, const uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	uint8_t *ecc_calc = chip->ecc.calc_buf;
	register char * ptr = nand->base+REG_SMRA0;

	memset ( (void*)ptr, 0xFF, mtd->oobsize );
	memcpy ( (void*)ptr, (void*)chip->oob_poi,  mtd->oobsize - chip->ecc.total );

	ma35d1_nand_command(chip, NAND_CMD_SEQIN, 0, page);
	ma35d1_nand_dma_transfer(chip, buf, mtd->writesize , 0x1);
	ma35d1_nand_command(chip, NAND_CMD_PAGEPROG, -1, -1);

	// Copy parity code in SMRA to calc
	memcpy ( (void*)ecc_calc,  (void*)( nand->base+REG_SMRA0 + ( mtd->oobsize - chip->ecc.total ) ), chip->ecc.total );

	// Copy parity code in calc to oob_poi
	memcpy ( (void*)(chip->oob_poi+(mtd->oobsize-chip->ecc.total)), (void*)ecc_calc, chip->ecc.total);

	return 0;
}

static int ma35d1_nand_read_page_hwecc_oob_first(struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	uint8_t *p = buf;
	char * ptr=nand->base+REG_SMRA0;

	/* At first, read the OOB area  */
	ma35d1_nand_command(chip, NAND_CMD_READOOB, 0, page);
	ma35d1_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
	{
		memset((void*)p, 0xff, mtd->writesize);
	}
	else
	{
		// Third, read data from nand
		ma35d1_nand_command(chip, NAND_CMD_READ0, 0, page);
		ma35d1_nand_dma_transfer(chip, p, mtd->writesize, 0x0);

		// Fouth, restore OOB data from SMRA
		memcpy ( (void*)chip->oob_poi, (void*)ptr, mtd->oobsize );
	}

	return 0;
}

static void ma35d1_layout_oob_table (struct nand_ecclayout_user *pNandOOBTbl, int oobsize , int eccbytes )
{
	pNandOOBTbl->eccbytes = eccbytes;

	pNandOOBTbl->oobavail = oobsize - DEF_RESERVER_OOB_SIZE_FOR_MARKER - eccbytes ;

	pNandOOBTbl->oobfree[0].offset = DEF_RESERVER_OOB_SIZE_FOR_MARKER;  // Bad block marker size

	pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset ;
}

static int ma35d1_nand_read_oob_hwecc(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	char * ptr=nand->base+REG_SMRA0;

	ma35d1_nand_command(chip, NAND_CMD_READOOB, 0, page);

	ma35d1_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy ( (void*)ptr, (void*)chip->oob_poi, mtd->oobsize );

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
	{
		memset((void*)chip->oob_poi, 0xff, mtd->oobsize);
	}

	return 0;
}


static int ma35d1_nand_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *host = nand_get_controller_data(chip);
	struct device *dev = &host->pdev->dev;
	unsigned int reg;

	//Set PSize bits of SMCSR register to select NAND card page size
	reg = readl(host->base+REG_SMCSR) & (~0x30000);
	writel(reg | (mtd->writesize << 5), host->base+REG_SMCSR);

	if (chip->ecc.strength == 0) {
		host->eBCHAlgo = eBCH_NONE; /* No ECC */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 8) {
		host->eBCHAlgo = eBCH_T8; /* T8 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 12) {
		host->eBCHAlgo = eBCH_T12; /* T12 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 24) {
		host->eBCHAlgo = eBCH_T24; /* T24 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize, g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else {
		pr_warn("NAND Controller is not support this flash. (%d, %d)\n", mtd->writesize, mtd->oobsize);
	}

	host->m_i32SMRASize = mtd->oobsize;
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	chip->ecc.bytes = ma35d1_nand_oob.eccbytes / chip->ecc.steps;
	chip->ecc.total = ma35d1_nand_oob.eccbytes;
	mtd_set_ooblayout(mtd, &ma35d1_ooblayout_ops);

	pr_info("attach: page %d, SMRA size %d, %d\n", mtd->writesize, mtd->oobsize, ma35d1_nand_oob.eccbytes);

	/* add mtd-id. The string should same as uboot definition */
	mtd->name = "nand0";

	ma35d1_nand_hwecc_init(host);

	writel(0x0, host->base+REG_NFECR); /* lock write protect */

	host->dma_buf = devm_kzalloc(dev, mtd->writesize, GFP_KERNEL);
	if (!host->dma_buf)
		return -ENOMEM;

	return 0;
}

static const struct nand_controller_ops ma35d1_nand_controller_ops = {
	.attach_chip = ma35d1_nand_attach_chip,
};

static int ma35d1_nand_probe(struct platform_device *pdev)
{
	struct nand_chip *chip;
	struct ma35d1_nand_info *ma35d1_nand;
	struct mtd_info *mtd;
	const char *clkgate;
	int retval=0;

	ma35d1_nand = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_nand_info), GFP_KERNEL);
	if (!ma35d1_nand)
		return -ENOMEM;

	nand_controller_init(&ma35d1_nand->controller);

	ma35d1_nand->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ma35d1_nand->base))
		return PTR_ERR(ma35d1_nand->base);

	ma35d1_nand->pdev = pdev;
	ma35d1_nand->dev = &pdev->dev;
	chip = &ma35d1_nand->chip;
	mtd = nand_to_mtd(chip);
	nand_set_controller_data(chip, ma35d1_nand);
	nand_set_flash_node(chip, pdev->dev.of_node);

	mtd->priv   = chip;
	mtd->owner  = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;

	/* enable clock */
	of_property_read_string(pdev->dev.of_node, "clock-enable", &clkgate);
	ma35d1_nand->clk = devm_clk_get(&pdev->dev, clkgate);
	if (IS_ERR(ma35d1_nand->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
			return PTR_ERR(ma35d1_nand->clk);
	}

	retval = clk_prepare_enable(ma35d1_nand->clk);
	if (retval) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		retval = -ENXIO;
		goto fail;
	}

	ma35d1_nand->chip.controller = &ma35d1_nand->controller;

	chip->legacy.cmdfunc     = ma35d1_nand_command;
	chip->legacy.read_byte   = ma35d1_nand_read_byte;
	chip->legacy.select_chip = ma35d1_nand_select_chip;
	chip->legacy.read_buf    = ma35d1_read_buf_dma;
	chip->legacy.write_buf   = ma35d1_write_buf_dma;
	chip->legacy.chip_delay  = 25; /* us */

	// Check NAND device NBUSY0 pin
	chip->legacy.dev_ready     = ma35d1_nand_devready;

	// Read OOB data first, then HW read page
	chip->ecc.hwctl      = ma35d1_nand_enable_hwecc;
	chip->ecc.calculate  = ma35d1_nand_calculate_ecc;
	chip->ecc.correct    = ma35d1_nand_correct_data;
	chip->ecc.write_page = ma35d1_nand_write_page_hwecc;
	chip->ecc.read_page  = ma35d1_nand_read_page_hwecc_oob_first;
	chip->ecc.read_oob   = ma35d1_nand_read_oob_hwecc;
	chip->options |= (NAND_NO_SUBPAGE_WRITE | NAND_USE_BOUNCE_BUFFER);

	ma35d1_nand_initialize(ma35d1_nand);
	platform_set_drvdata(pdev, ma35d1_nand);

	ma35d1_nand->controller.ops = &ma35d1_nand_controller_ops;
	/* second phase scan */
	if (nand_scan(chip, 1))
		goto fail;

	if (mtd_device_register(mtd, ma35d1_nand->parts, ma35d1_nand->nr_parts))
		goto fail;

	pr_info("fmi-sm: registered successfully! mtdid=%s\n", mtd->name);
	return retval;

fail:
	nand_release(chip);
	devm_kfree(&pdev->dev, ma35d1_nand);
	return retval;
}

static int ma35d1_nand_remove(struct platform_device *pdev)
{
	struct ma35d1_nand_info *ma35d1_nand = platform_get_drvdata(pdev);

	clk_disable(ma35d1_nand->clk);
	clk_put(ma35d1_nand->clk);

	kfree(ma35d1_nand);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* PM Support */
#ifdef CONFIG_PM
static int ma35d1_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct ma35d1_nand_info *ma35d1_nand = platform_get_drvdata(pdev);

	// For save, wait DMAC to ready
	while ( readl(ma35d1_nand->base+REG_NAND_DMACCSR) & 0x200 );
	clk_disable(ma35d1_nand->clk);

	return 0;
}

static int ma35d1_nand_resume(struct platform_device *pdev)
{
	struct ma35d1_nand_info *ma35d1_nand = platform_get_drvdata(pdev);

	clk_enable(ma35d1_nand->clk);
	ma35d1_nand_hwecc_init(ma35d1_nand);
	ma35d1_nand_dmac_init(ma35d1_nand);


	return 0;
}

#else
#define ma35d1_nand_suspend NULL
#define ma35d1_nand_resume NULL
#endif

static const struct of_device_id ma35d1_fmi_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-nand" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_fmi_of_match);

static struct platform_driver ma35d1_nand_driver = {
		.driver = {
		.name   = "ma35d1-nand",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_fmi_of_match),
		},
		.probe      = ma35d1_nand_probe,
		.remove     = ma35d1_nand_remove,
		.suspend    = ma35d1_nand_suspend,
		.resume     = ma35d1_nand_resume,
};

static int __init ma35d1_nand_init(void)
{
	int ret;
	pr_info("ma35d1 mtd nand driver version: %s\n", MA35D1_DRV_VERSION );

	ret = platform_driver_register(&ma35d1_nand_driver);
	if (ret) {
		pr_warn("nand: failed to add device driver %s \n", ma35d1_nand_driver.driver.name);
		return ret;
	}

	return ret;
}

static void __exit ma35d1_nand_exit(void)
{
	platform_driver_unregister(&ma35d1_nand_driver);
	pr_info("nand: unregistered successfully! \n");
}

module_init(ma35d1_nand_init);
module_exit(ma35d1_nand_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("ma35d1 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-nand");

