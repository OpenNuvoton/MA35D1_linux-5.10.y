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
#define REG_NFI_BUFFER0	(0x000)	/* NFI Embedded Buffer Word 0 */
#define REG_NFI_DMACTL		(0x400)	/* NFI DMA Control and Status Register */
#define REG_NFI_DMASA		(0x408)	/* NFI DMA Transfer Starting Address Register */
#define REG_NFI_DMABCNT	(0x40C)	/* NFI DMA Transfer Byte Count Register */
#define REG_NFI_DMAINTEN	(0x410)	/* NFI DMA Interrupt Enable Control Register */
#define REG_NFI_DMAINTSTS	(0x414)	/* NFI DMA Interrupt Status Register */

#define REG_NFI_GCTL		(0x800)	/* NFI Global Control and Status Register */
#define REG_NFI_GINTEN		(0x804)	/* NFI Global Interrupt Control Register */
#define REG_NFI_GINTSTS	(0x808)	/* NFI Global Interrupt Status Register */

/* NAND-type Flash Registers */
#define REG_NFI_NANDCTL	(0x8A0)	/* NAND Flash Control Register */
#define REG_NFI_NANDTMCTL	(0x8A4)	/* NAND Flash Timing Control Register */
#define REG_NFI_NANDINTEN	(0x8A8)	/* NAND Flash Interrupt Enable Register */
#define REG_NFI_NANDINTSTS	(0x8AC)	/* NAND Flash Interrupt Status Register */
#define REG_NFI_NANDCMD	(0x8B0)	/* NAND Flash Command Port Register */
#define REG_NFI_NANDADDR	(0x8B4)	/* NAND Flash Address Port Register */
#define REG_NFI_NANDDATA	(0x8B8)	/* NAND Flash Data Port Register */
#define REG_NFI_NANDRACTL	(0x8BC)	/* NAND Flash Redundant Area Control Register */
#define REG_NFI_NANDECTL	(0x8C0)	/* NAND Flash Extend Control Register */
#define REG_NFI_NANDECCES0	(0x8D0)	/* NAND Flash ECC Error Status 0 Register */
#define REG_NFI_NANDECCES1	(0x8D4)	/* NAND Flash ECC Error Status 1 Register */
#define REG_NFI_NANDECCES2	(0x8D8)	/* NAND Flash ECC Error Status 2 Register */
#define REG_NFI_NANDECCES3	(0x8DC)	/* NAND Flash ECC Error Status 3 Register */

/* NAND-type Flash BCH Error Address Registers */
#define REG_NFI_NANDECCEA0	(0x900)	/* NAND Flash ECC Error Byte Address 0 Register */
#define REG_NFI_NANDECCEA1	(0x904)	/* NAND Flash ECC Error Byte Address 1 Register */
#define REG_NFI_NANDECCEA2	(0x908)	/* NAND Flash ECC Error Byte Address 2 Register */
#define REG_NFI_NANDECCEA3	(0x90C)	/* NAND Flash ECC Error Byte Address 3 Register */
#define REG_NFI_NANDECCEA4	(0x910)	/* NAND Flash ECC Error Byte Address 4 Register */
#define REG_NFI_NANDECCEA5	(0x914)	/* NAND Flash ECC Error Byte Address 5 Register */
#define REG_NFI_NANDECCEA6	(0x918)	/* NAND Flash ECC Error Byte Address 6 Register */
#define REG_NFI_NANDECCEA7	(0x91C)	/* NAND Flash ECC Error Byte Address 7 Register */
#define REG_NFI_NANDECCEA8	(0x920)	/* NAND Flash ECC Error Byte Address 8 Register */
#define REG_NFI_NANDECCEA9	(0x924)	/* NAND Flash ECC Error Byte Address 9 Register */
#define REG_NFI_NANDECCEA10	(0x928)	/* NAND Flash ECC Error Byte Address 10 Register */
#define REG_NFI_NANDECCEA11	(0x92C)	/* NAND Flash ECC Error Byte Address 11 Register */

/* NAND-type Flash BCH Error Data Registers */
#define REG_NFI_NANDECCED0	(0x960)	/* NAND Flash ECC Error Data Register 0 */
#define REG_NFI_NANDECCED1	(0x964)	/* NAND Flash ECC Error Data Register 1 */
#define REG_NFI_NANDECCED2	(0x968)	/* NAND Flash ECC Error Data Register 2 */
#define REG_NFI_NANDECCED3	(0x96C)	/* NAND Flash ECC Error Data Register 3 */
#define REG_NFI_NANDECCED4	(0x970)	/* NAND Flash ECC Error Data Register 4 */
#define REG_NFI_NANDECCED5	(0x974)	/* NAND Flash ECC Error Data Register 5 */

/* NAND-type Flash Redundant Area Registers */
#define REG_NFI_NANDRA0	(0xA00)	/* NAND Flash Redundant Area Word 0 */
#define REG_NFI_NANDRA1	(0xA04)	/* NAND Flash Redundant Area Word 1 */

/************************************************/
#define ENDADDR     (0x01 << 31)

#define BCH_T8      0x00100000
#define BCH_T12     0x00200000
#define BCH_T24     0x00040000

#define MA35D1_DRV_VERSION "20200921"
#define DEF_RESERVER_OOB_SIZE_FOR_MARKER 4

struct ma35d1_nand_info {
	struct nand_controller	controller;
	struct device		*dev;
	void __iomem		*base;

	struct mtd_info	mtd;
	struct nand_chip	chip;
	struct mtd_partition	*parts;	/* mtd partition */
	int			nr_parts;	/* mtd partition number */
	struct platform_device  *pdev;
	struct clk              *clk;

	int                     eBCHAlgo;
	int                     m_i32SMRASize;
	int			irq;
	struct completion       complete;

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

enum {
	eBCH_NONE = 0,
	eBCH_T8,
	eBCH_T12,
	eBCH_T24,
	eBCH_CNT
} E_BCHALGORITHM;


static const int g_i32BCHAlgoIdx[eBCH_CNT] = { BCH_T8, BCH_T8, BCH_T12, BCH_T24 };
static struct nand_ecclayout_user ma35d1_nand_oob;
static const int g_i32ParityNum[3][eBCH_CNT] = {
	{ 0,  60,  92,  90 },  /* for 2K */
	{ 0, 120, 184, 180 },  /* for 4K */
	{ 0, 240, 368, 360 },  /* for 8K */
};


/*
 * ma35d1_nand_hwecc_init - Initialize hardware ECC IP
 */
static void ma35d1_nand_hwecc_init(struct ma35d1_nand_info *nand)
{
	/* reset SM controller */
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x1, nand->base+REG_NFI_NANDCTL);

	/* Redundant area size */
	writel(nand->m_i32SMRASize, nand->base+REG_NFI_NANDRACTL);

	/* Protect redundant 3 bytes */
	/* because we need to implement write_oob function to partial data to oob available area. */
	/* Please note we skip 4 bytes */
	writel(readl(nand->base+REG_NFI_NANDCTL) | 0x100, nand->base+REG_NFI_NANDCTL);

	/* To read/write the ECC parity codes automatically from/to NAND Flash after data area field written. */
	writel(readl(nand->base+REG_NFI_NANDCTL) | 0x10, nand->base+REG_NFI_NANDCTL);

	if (nand->eBCHAlgo == eBCH_NONE) {
		/* Disable H/W ECC / ECC parity check enable bit during read page */
		writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x00800000), nand->base+REG_NFI_NANDCTL);
	} else  {
		/* Set BCH algorithm */
		writel((readl(nand->base+REG_NFI_NANDCTL) & (~0x007C0000)) | g_i32BCHAlgoIdx[nand->eBCHAlgo],
			nand->base+REG_NFI_NANDCTL);

		/* Enable H/W ECC, ECC parity check enable bit during read page */
		writel(readl(nand->base+REG_NFI_NANDCTL) | 0x00800000, nand->base+REG_NFI_NANDCTL);
	}
}

static void ma35d1_nand_initialize(struct ma35d1_nand_info *nand)
{
	/* Enable SM_EN */
	writel(0x8, nand->base+REG_NFI_GCTL);
}

/*-------------------------------*/
/* Define some constants for BCH */
/*-------------------------------*/
/* define the total padding bytes for 512/1024 data segment */
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
/* define the BCH parity code length for 512 bytes data pattern */
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
/* define the BCH parity code length for 1024 bytes data pattern */
#define BCH_PARITY_LEN_T24 45


/*--------------------------------*/
/* Correct data by BCH alrogithm. */
/*--------------------------------*/
void fmiSM_CorrectData_BCH(struct ma35d1_nand_info *nand, u8 ucFieidIndex,
				u8 ucErrorCnt, u8 *pDAddr)
{
	u32 uaData[24], uaAddr[24];
	u32 uaErrorData[6];
	u8  ii, jj;
	u32 uPageSize;
	u32 field_len, padding_len, parity_len;
	u32 total_field_num;
	u8  *smra_index;

	/*--- assign some parameters for different BCH and page size */
	switch (readl(nand->base+REG_NFI_NANDCTL) & 0x007C0000) {
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
		pr_warn("NAND ERROR: invalid SMCR_BCH_TSEL = 0x%08X\n",
			(u32)(readl(nand->base+REG_NFI_NANDCTL) & 0x7C0000));
		return;
	}

	uPageSize = readl(nand->base+REG_NFI_NANDCTL) & 0x00030000;
	switch (uPageSize) {
	case 0x30000:
		total_field_num = 8192 / field_len; break;
	case 0x20000:
		total_field_num = 4096 / field_len; break;
	case 0x10000:
		total_field_num = 2048 / field_len; break;
	default:
		pr_warn("NAND ERROR: invalid SMCR_PSIZE = 0x%08X\n", uPageSize);
		return;
	}

	/* got valid BCH_ECC_DATAx and parse them to uaData[] */
	/* got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes */
	jj = ucErrorCnt / 4;
	jj++;
	if (jj > 6)
		jj = 6;	/* there are 6 BCH_ECC_DATAx registers to support BCH T24 */

	for (ii = 0; ii < jj; ii++)
		uaErrorData[ii] = readl(nand->base+REG_NFI_NANDECCED0 + ii*4);

	for (ii = 0; ii < jj; ii++) {
		uaData[ii*4+0] = uaErrorData[ii] & 0xff;
		uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
		uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
		uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
	}

	/* got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[] */
	/* got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses */
	jj = ucErrorCnt / 2;
	jj++;
	if (jj > 12)
		jj = 12;	/* there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24 */

	for (ii = 0; ii < jj; ii++) {
		/* 11 bits for error address */
		uaAddr[ii*2+0] = readl(nand->base+REG_NFI_NANDECCEA0 + ii*4) & 0x07ff;
		uaAddr[ii*2+1] = (readl(nand->base+REG_NFI_NANDECCEA0 + ii*4)>>16) & 0x07ff;
	}

	/* pointer to begin address of field that with data error */
	pDAddr += (ucFieidIndex-1) * field_len;

	/* correct each error bytes */
	for (ii = 0; ii < ucErrorCnt; ii++) {
		/* for wrong data in field */
		if (uaAddr[ii] < field_len)
			*(pDAddr+uaAddr[ii]) ^= uaData[ii];

		/* for wrong first-3-bytes in redundancy area */
		else if (uaAddr[ii] < (field_len+3)) {
			uaAddr[ii] -= field_len;
			uaAddr[ii] += (parity_len*(ucFieidIndex-1));	/* field offset */

			*((u8 *)nand->base+REG_NFI_NANDRA0 + uaAddr[ii]) ^= uaData[ii];
		}
		/* for wrong parity code in redundancy area */
		else {
			/* BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]          */
			/*                                   |<--     padding bytes      -->|          */
			/* The BCH_ERR_ADDRx for last parity code always = field size + padding size.  */
			/* So, the first parity code = field size + padding size - parity code length. */
			/* For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.      */
			/* That is, error byte address offset within field is                          */
			uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

			/* smra_index point to the first parity code of first field in register SMRA0~n */
			smra_index = (u8 *)(nand->base+REG_NFI_NANDRA0 +
				(readl(nand->base+REG_NFI_NANDRACTL) & 0x1ff) -
				(parity_len * total_field_num));

			/* final address = first parity code of first field + */
			/*                 offset of fields +                 */
			/*                 offset within field                */

			*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];
		}
	}   /* end of for (ii < ucErrorCnt) */
}

int fmiSMCorrectData(struct nand_chip *chip, unsigned long uDAddr)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int uStatus, ii, jj, i32FieldNum = 0;
	int uErrorCnt = 0;
	int uReportErrCnt = 0;

	if ((readl(nand->base+REG_NFI_NANDCTL) & 0x7C0000) == BCH_T24)
		i32FieldNum = mtd->writesize / 1024;
	else
		i32FieldNum = mtd->writesize / 512;

	if (i32FieldNum < 4)
		i32FieldNum  = 1;
	else
		i32FieldNum /= 4;

	for (jj = 0; jj < i32FieldNum; jj++) {
		uStatus = readl(nand->base+REG_NFI_NANDECCES0+jj*4);
		if (!uStatus)
			continue;

		for (ii = 1; ii < 5; ii++) {
			if (!(uStatus & 0x03)) { /* No error */
				uStatus >>= 8;
				continue;

			} else if ((uStatus & 0x03) == 0x01) { /* Correctable error */
				uErrorCnt = (uStatus >> 2) & 0x1F;
				pr_warn("Field (%d, %d) have %d error!!\n", jj, ii, uErrorCnt);
				fmiSM_CorrectData_BCH(nand, jj*4+ii, uErrorCnt, (char *)uDAddr);
				uReportErrCnt += uErrorCnt;

			} else { /* uncorrectable error or ECC error */
				pr_warn("SM uncorrectable error is encountered, 0x%4x !!\n", uStatus);
				return -1;
			}
			uStatus >>= 8;
		}
	}
	return uReportErrCnt;
}

static int ma35d1_nand_correct_data(struct nand_chip *chip, u_char *dat,
					u_char *read_ecc, u_char *calc_ecc)
{
	return 0;
}


void ma35d1_nand_enable_hwecc(struct nand_chip *chip, int mode)
{

}

/*
 * ma35d1_nand_dmac_init - Initialize dma controller
 */
static void ma35d1_nand_dmac_init(struct ma35d1_nand_info *nand)
{
	/* DMAC enable */
	writel(readl(nand->base+REG_NFI_DMACTL) | 0x3, nand->base+REG_NFI_DMACTL);
	writel(readl(nand->base+REG_NFI_DMACTL) & (~0x2), nand->base+REG_NFI_DMACTL);

	/* Clear DMA finished flag */
	writel(readl(nand->base+REG_NFI_NANDINTSTS) | 0x5, nand->base+REG_NFI_NANDINTSTS);

	init_completion(&nand->complete);
	/* enable ecc and dma */
	writel(0x5, nand->base+REG_NFI_NANDINTEN);
}

/*
 * ma35d1_nand_read_byte - read a byte from NAND controller into buffer
 */
static unsigned char ma35d1_nand_read_byte(struct nand_chip *chip)
{
	unsigned char ret;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	ret = (unsigned char)readl(nand->base+REG_NFI_NANDDATA);
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);

	return ret;
}


/*
 * ma35d1_nand_read_buf - read data from NAND controller into buffer
 */
static void ma35d1_nand_read_buf(struct nand_chip *chip, unsigned char *buf, int len)
{
	int i;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)readl(nand->base+REG_NFI_NANDDATA);
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);
}
/*
 * ma35d1_nand_write_buf - write data from buffer into NAND controller
 */

static void ma35d1_nand_write_buf(struct nand_chip *chip, const unsigned char *buf, int len)
{
	int i;
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	for (i = 0; i < len; i++)
		writel(buf[i], nand->base+REG_NFI_NANDDATA);
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);
}

/*
 * ma35d1_nand_dma_transfer: configer and start dma transfer
 */
static inline int ma35d1_nand_dma_transfer(struct nand_chip *chip,
	const u_char *addr, unsigned int len, int is_write)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	dma_addr_t dma_addr;
	int ret;
	unsigned long timeo = jiffies + HZ/2;

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	/* For save, wait DMAC to ready */
	while (1) {
		if ((readl(nand->base+REG_NFI_DMACTL) & 0x200) == 0)
			break;
		if (time_after(jiffies, timeo))
			return -ETIMEDOUT;
	}

	/* Reinitial dmac */
	ma35d1_nand_dmac_init(nand);

	writel(nand->m_i32SMRASize, nand->base+REG_NFI_NANDRACTL);

	/* setup and start DMA using dma_addr */
	if (is_write) {
		register char *ptr = nand->base+REG_NFI_NANDRA0;
		/* To mark this page as dirty. */
		if (ptr[3] == 0xFF)
			ptr[3] = 0;
		if (ptr[2] == 0xFF)
			ptr[2] = 0;

		/* Fill dma_addr */
		dma_addr = dma_map_single(nand->dev, (void *)addr, len, DMA_TO_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}

		writel((unsigned long)dma_addr, nand->base+REG_NFI_DMASA);
		writel(readl(nand->base+REG_NFI_NANDCTL) | 0x4, nand->base+REG_NFI_NANDCTL);
		wait_for_completion_timeout(&nand->complete, msecs_to_jiffies(1000));

		dma_unmap_single(nand->dev, dma_addr, len, DMA_TO_DEVICE);
	} else {
		/* Fill dma_addr */
		dma_addr = dma_map_single(nand->dev, (void *)addr, len, DMA_FROM_DEVICE);
		ret = dma_mapping_error(nand->dev, dma_addr);
		if (ret) {
			dev_err(nand->dev, "dma mapping error\n");
			return -EINVAL;
		}
		nand->dma_buf = (unsigned char *) dma_addr;

		writel((unsigned long)dma_addr, nand->base+REG_NFI_DMASA);
		writel(readl(nand->base+REG_NFI_NANDCTL) | 0x2, nand->base+REG_NFI_NANDCTL);
		wait_for_completion_timeout(&nand->complete, msecs_to_jiffies(1000));

		dma_unmap_single(nand->dev, dma_addr, len, DMA_FROM_DEVICE);
	}

	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);

	return 0;
}

static void ma35d1_read_buf_dma(struct nand_chip *chip, u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if (len == mtd->writesize) /* start transfer in DMA mode */
		ma35d1_nand_dma_transfer(chip, buf, len, 0x0);
	else {
		ma35d1_nand_read_buf(chip, buf, len);
	}
}

static void ma35d1_write_buf_dma(struct nand_chip *chip, const u_char *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);

	if (len == mtd->writesize) /* start transfer in DMA mode */
		ma35d1_nand_dma_transfer(chip, (u_char *)buf, len, 0x1);
	else
		ma35d1_nand_write_buf(chip, buf, len);
}


static int ma35d1_nand_devready(struct nand_chip *chip)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	unsigned int val;

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	val = (readl(nand->base+REG_NFI_NANDINTSTS) & 0x40000) ? 1 : 0;
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);

	return val;
}

static int ma35d1_waitfunc(struct nand_chip *chip)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	unsigned long timeo = jiffies;
	int status = -1;

	timeo += msecs_to_jiffies(400);

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	while (time_before(jiffies, timeo)) {
		status = readl(nand->base+REG_NFI_NANDINTSTS);
		if (status & 0x400) {	/* check r/b# flag */
			writel(0x400, nand->base+REG_NFI_NANDINTSTS);
			status = 0;
			break;
		}
		cond_resched();
	}
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);
	return status;
}

static void ma35d1_nand_command(struct nand_chip *chip,
	unsigned int command, int column, int page_addr)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);

	writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	writel(0x400, nand->base+REG_NFI_NANDINTSTS);

	if (command == NAND_CMD_READOOB) {
		command = NAND_CMD_READ0;
		column += mtd->writesize;
	}

	switch (command) {

	case NAND_CMD_RESET:
		writel(command, nand->base+REG_NFI_NANDCMD);
		break;

	case NAND_CMD_READID:
		writel(command, nand->base+REG_NFI_NANDCMD);
		writel(ENDADDR|column, nand->base+REG_NFI_NANDADDR);
		break;

	case NAND_CMD_PARAM:
		writel(command, nand->base+REG_NFI_NANDCMD);
		writel(ENDADDR|column, nand->base+REG_NFI_NANDADDR);
		ma35d1_waitfunc(chip);
		break;

	case NAND_CMD_READ0:
		writel(0x0, nand->base+REG_NFI_NANDECTL); /* lock write protect */
		writel(command, nand->base+REG_NFI_NANDCMD);
		if (column != -1) {
			writel(column & 0xff, nand->base+REG_NFI_NANDADDR);
			writel((column >> 8) & 0xff, nand->base+REG_NFI_NANDADDR);
		}
		if (page_addr != -1) {
			writel(page_addr & 0xff, nand->base+REG_NFI_NANDADDR);
			if (chip->options & NAND_ROW_ADDR_3) {
				writel((page_addr >> 8) & 0xff, nand->base+REG_NFI_NANDADDR);
				writel(((page_addr >> 16) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
			} else {
				writel(((page_addr >> 8) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
			}
		}
		writel(NAND_CMD_READSTART, nand->base+REG_NFI_NANDCMD);
		ma35d1_waitfunc(chip);
		break;


	case NAND_CMD_ERASE1:
		writel(0x1, nand->base+REG_NFI_NANDECTL); /* un-lock write protect */
		writel(command, nand->base+REG_NFI_NANDCMD);
		writel(page_addr & 0xff, nand->base+REG_NFI_NANDADDR);
		if (chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, nand->base+REG_NFI_NANDADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
		} else {
			writel(((page_addr >> 8) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
		}
		break;


	case NAND_CMD_SEQIN:
		writel(0x1, nand->base+REG_NFI_NANDECTL); /* un-lock write protect */
		writel(command, nand->base+REG_NFI_NANDCMD);
		writel(column & 0xff, nand->base+REG_NFI_NANDADDR);
		writel(column >> 8, nand->base+REG_NFI_NANDADDR);
		writel(page_addr & 0xff, nand->base+REG_NFI_NANDADDR);
		if (chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, nand->base+REG_NFI_NANDADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
		} else {
			writel(((page_addr >> 8) & 0xff)|ENDADDR, nand->base+REG_NFI_NANDADDR);
		}
		break;

	case NAND_CMD_STATUS:
		writel(0x1, nand->base+REG_NFI_NANDECTL); /* un-lock write protect */
		writel(command, nand->base+REG_NFI_NANDCMD);
		break;

	default:
		writel(command, nand->base+REG_NFI_NANDCMD);
	}
	writel(readl(nand->base+REG_NFI_NANDCTL)|0x02000000, nand->base+REG_NFI_NANDCTL);
}

/* select chip */
static void ma35d1_nand_select_chip(struct nand_chip *chip, int cs)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);

	if (cs == 0)
		writel(readl(nand->base+REG_NFI_NANDCTL) & (~0x02000000), nand->base+REG_NFI_NANDCTL);
	else
		writel(readl(nand->base+REG_NFI_NANDCTL) | 0x02000000, nand->base+REG_NFI_NANDCTL);
}

/*
 * Calculate HW ECC
 * function called after a write
 */
static int ma35d1_nand_calculate_ecc(struct nand_chip *chip, const u_char *dat, u_char *ecc_code)
{
	return 0;
}

static int ma35d1_nand_write_page_hwecc(struct nand_chip *chip,
	const uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	uint8_t *ecc_calc = chip->ecc.calc_buf;
	register char *ptr = nand->base+REG_NFI_NANDRA0;

	memset((void *)ptr, 0xFF, mtd->oobsize);
	memcpy((void *)ptr, (void *)chip->oob_poi,  mtd->oobsize - chip->ecc.total);

	ma35d1_nand_command(chip, NAND_CMD_SEQIN, 0, page);
	ma35d1_nand_dma_transfer(chip, buf, mtd->writesize, 0x1);
	ma35d1_nand_command(chip, NAND_CMD_PAGEPROG, -1, -1);
	ma35d1_waitfunc(chip);

	/* Copy parity code in SMRA to calc */
	memcpy((void *)ecc_calc,
		(void *)(nand->base+REG_NFI_NANDRA0 + (mtd->oobsize-chip->ecc.total)), chip->ecc.total);

	/* Copy parity code in calc to oob_poi */
	memcpy((void *)(chip->oob_poi+(mtd->oobsize-chip->ecc.total)), (void *)ecc_calc, chip->ecc.total);

	return 0;
}

static int ma35d1_nand_read_page_hwecc_oob_first(struct nand_chip *chip,
	uint8_t *buf, int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	uint8_t *p = buf;
	char *ptr = nand->base+REG_NFI_NANDRA0;

	/* At first, read the OOB area  */
	ma35d1_nand_command(chip, NAND_CMD_READOOB, 0, page);
	ma35d1_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	/* Second, copy OOB data to SMRA for page read */
	memcpy((void *)ptr, (void *)chip->oob_poi, mtd->oobsize);

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
		memset((void *)p, 0xff, mtd->writesize);
	else {
		/* Third, read data from nand */
		ma35d1_nand_command(chip, NAND_CMD_READ0, 0, page);
		ma35d1_nand_dma_transfer(chip, p, mtd->writesize, 0x0);

		/* Fouth, restore OOB data from SMRA */
		memcpy((void *)chip->oob_poi, (void *)ptr, mtd->oobsize);
	}

	return 0;
}

static void ma35d1_layout_oob_table(struct nand_ecclayout_user *pNandOOBTbl,
					int oobsize, int eccbytes)
{
	pNandOOBTbl->eccbytes = eccbytes;

	pNandOOBTbl->oobavail = oobsize - DEF_RESERVER_OOB_SIZE_FOR_MARKER - eccbytes;

	pNandOOBTbl->oobfree[0].offset = DEF_RESERVER_OOB_SIZE_FOR_MARKER;  /* Bad block marker size */

	pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset;
}

static int ma35d1_nand_read_oob_hwecc(struct nand_chip *chip, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	char *ptr = nand->base+REG_NFI_NANDRA0;

	ma35d1_nand_command(chip, NAND_CMD_READOOB, 0, page);
	ma35d1_nand_read_buf(chip, chip->oob_poi, mtd->oobsize);

	/* Second, copy OOB data to SMRA for page read */
	memcpy((void *)ptr, (void *)chip->oob_poi, mtd->oobsize);

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
		memset((void *)chip->oob_poi, 0xff, mtd->oobsize);

	return 0;
}

static irqreturn_t ma35d1_nand_irq(int irq, struct ma35d1_nand_info *nand)
{
	struct mtd_info *mtd = nand_to_mtd(&nand->chip);
	unsigned int isr;
	int stat = 0;

	/* Clear interrupt flag */
	/* SM interrupt status */
	isr = readl(nand->base+REG_NFI_NANDINTSTS);
	if (isr & 0x01) {
		writel(0x1, nand->base+REG_NFI_NANDINTSTS);
		complete(&nand->complete);
	}
	if (isr & 0x04) {
		stat = fmiSMCorrectData(&nand->chip, (unsigned long)nand->dma_buf);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
			writel(0x3, nand->base+REG_NFI_DMACTL);          /* reset DMAC */
			writel(readl(nand->base+REG_NFI_NANDCTL)|0x1,
					nand->base+REG_NFI_NANDCTL);    /* reset SM controller */

		} else if (stat > 0) {
			mtd->ecc_stats.corrected += stat;   /* Add corrected bit count */
		}
		writel(0x4, nand->base+REG_NFI_NANDINTSTS);
	}

	return IRQ_HANDLED;
}


static int ma35d1_nand_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct ma35d1_nand_info *host = nand_get_controller_data(chip);
	unsigned int reg;

	/* Set PSize bits of SMCSR register to select NAND card page size */
	reg = readl(host->base+REG_NFI_NANDCTL) & (~0x30000);
	if (mtd->writesize == 2048)
		writel(reg | 0x10000, host->base+REG_NFI_NANDCTL);
	else if (mtd->writesize == 4096)
		writel(reg | 0x20000, host->base+REG_NFI_NANDCTL);
	else if (mtd->writesize == 8192)
		writel(reg | 0x30000, host->base+REG_NFI_NANDCTL);

	if (chip->ecc.strength == 0) {
		host->eBCHAlgo = eBCH_NONE; /* No ECC */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
				g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 8) {
		host->eBCHAlgo = eBCH_T8; /* T8 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
				g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 12) {
		host->eBCHAlgo = eBCH_T12; /* T12 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
				g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

	} else if (chip->ecc.strength <= 24) {
		host->eBCHAlgo = eBCH_T24; /* T24 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
				g_i32ParityNum[mtd->writesize>>12][host->eBCHAlgo]);

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

	writel(0x0, host->base+REG_NFI_NANDECTL); /* lock write protect */

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
	int retval = 0;

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
	chip->legacy.waitfunc    = ma35d1_waitfunc;
	chip->legacy.read_byte   = ma35d1_nand_read_byte;
	chip->legacy.select_chip = ma35d1_nand_select_chip;
	chip->legacy.read_buf    = ma35d1_read_buf_dma;
	chip->legacy.write_buf   = ma35d1_write_buf_dma;
	chip->legacy.chip_delay  = 25; /* us */

	/* Check NAND device NBUSY0 pin */
	chip->legacy.dev_ready     = ma35d1_nand_devready;

	/* Read OOB data first, then HW read page */
	chip->ecc.hwctl      = ma35d1_nand_enable_hwecc;
	chip->ecc.calculate  = ma35d1_nand_calculate_ecc;
	chip->ecc.correct    = ma35d1_nand_correct_data;
	chip->ecc.write_page = ma35d1_nand_write_page_hwecc;
	chip->ecc.read_page  = ma35d1_nand_read_page_hwecc_oob_first;
	chip->ecc.read_oob   = ma35d1_nand_read_oob_hwecc;
	chip->options |= (NAND_NO_SUBPAGE_WRITE | NAND_USES_DMA);

	ma35d1_nand_initialize(ma35d1_nand);
	platform_set_drvdata(pdev, ma35d1_nand);

	ma35d1_nand->controller.ops = &ma35d1_nand_controller_ops;

	ma35d1_nand->irq = platform_get_irq(pdev, 0);
	if (ma35d1_nand->irq < 0) {
		dev_err(&pdev->dev, "failed to get platform irq\n");
		retval = -EINVAL;
		goto fail;
	}

	if (request_irq(ma35d1_nand->irq, (irq_handler_t)&ma35d1_nand_irq,
			IRQF_TRIGGER_HIGH, "ma35d1-nand", ma35d1_nand)) {
		dev_err(&pdev->dev, "Error requesting NAND IRQ\n");
		retval = -ENXIO;
		goto fail;
	}

	if (nand_scan(chip, 1))
		goto fail;

	if (mtd_device_register(mtd, ma35d1_nand->parts, ma35d1_nand->nr_parts))
		goto fail;

	pr_info("fmi-sm: registered successfully! mtdid=%s\n", mtd->name);
	return retval;

fail:
	nand_cleanup(chip);
	devm_kfree(&pdev->dev, ma35d1_nand);
	return retval;
}

static int ma35d1_nand_remove(struct platform_device *pdev)
{
	struct ma35d1_nand_info *ma35d1_nand = platform_get_drvdata(pdev);
	struct nand_chip *chip = &ma35d1_nand->chip;
	int ret;

	ret = mtd_device_unregister(nand_to_mtd(chip));
	WARN_ON(ret);
	nand_cleanup(chip);

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
	unsigned long timeo = jiffies + HZ/2;

	/* wait DMAC to ready */
	while (1) {
		if ((readl(ma35d1_nand->base+REG_NFI_DMACTL) & 0x200) == 0)
			break;
		if (time_after(jiffies, timeo))
			return -ETIMEDOUT;
	}

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

	pr_info("ma35d1 mtd nand driver version: %s\n", MA35D1_DRV_VERSION);

	ret = platform_driver_register(&ma35d1_nand_driver);
	if (ret) {
		pr_warn("nand: failed to add device driver %s\n", ma35d1_nand_driver.driver.name);
		return ret;
	}

	return ret;
}

static void __exit ma35d1_nand_exit(void)
{
	platform_driver_unregister(&ma35d1_nand_driver);
	pr_info("nand: unregistered successfully!\n");
}

module_init(ma35d1_nand_init);
module_exit(ma35d1_nand_exit);

MODULE_AUTHOR("nuvoton");
MODULE_DESCRIPTION("ma35d1 nand driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-nand");

