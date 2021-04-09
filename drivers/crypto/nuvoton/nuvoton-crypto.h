/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Nuvoton Cryptographic Accelerator registers and data
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#ifndef __NUVOTON_CRYPTO_H__
#define __NUVOTON_CRYPTO_H__

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <linux/types.h>

#define INTEN			0x000
#define INTEN_AESIEN			(0x1 << 0)
#define INTEN_AESEIEN			(0x1 << 1)
#define INTEN_PRNGIEN			(0x1 << 16)
#define INTEN_ECCIEN			(0x1 << 22)
#define INTEN_ECCEIEN			(0x1 << 23)
#define INTEN_HMACIEN			(0x1 << 24)
#define INTEN_HMACEIEN			(0x1 << 25)
#define INTSTS			0x004
#define INTSTS_AESIF			(0x1 << 0)
#define INTSTS_AESEIF			(0x1 << 1)
#define INTSTS_PRNGIF			(0x1 << 16)
#define INTSTS_ECCIF			(0x1 << 22)
#define INTSTS_ECCEIF			(0x1 << 23)
#define INTSTS_HMACIF			(0x1 << 24)
#define INTSTS_HMACEIF			(0x1 << 25)

#define PRNG_CTL		0x008
#define PRNG_CTL_START			(0x1 << 0)
#define PRNG_CTL_SEEDRLD		(0x1 << 1)
#define PRNG_CTL_KEYSZ_OFFSET		(2)
#define PRNG_CTL_KEYSZ_MASK		(0xf << 2)
#define PRNG_CTL_BUSY			(0x1 << 8)
#define PRNG_CTL_SEEDSRC		(0x1 << 16)
#define PRNG_SEED		0x00C
#define PRNG_KEY(x)		(0x010 + ((x) * 0x04))

#define AES_FDBCK(x)		(0x050 + ((x) * 0x04))
#define AES_GCM_IVCNT(x)	(0x080 + ((x) * 0x04))
#define AES_GCM_ACNT(x)		(0x088 + ((x) * 0x04))
#define AES_GCM_PCNT(x)		(0x090 + ((x) * 0x04))
#define AES_FBADDR		0x0A0
#define AES_CTL			0x100
#define AES_CTL_START			(0x1 << 0)
#define AES_CTL_STOP			(0x1 << 1)
#define AES_CTL_KEYSZ_OFFSET		2
#define AES_CTL_KEYSZ_MASK		(0x3 << 2)
#define AES_CTL_DMALAST			(0x1 << 5)
#define AES_CTL_DMACSCAD		(0x1 << 6)
#define AES_CTL_DMAEN			(0x1 << 7)
#define AES_CTL_OPMODE_OFFSET		8
#define AES_CTL_OPMODE_MASK		(0xff << 8)
#define AES_CTL_ENCRPT			(0x1 << 16)
#define AES_CTL_SM4EN			(0x1 << 17)
#define AES_CTL_FBIN			(0x1 << 20)
#define AES_CTL_FBOUT			(0x1 << 21)
#define AES_CTL_OUTSWAP			(0x1 << 22)
#define AES_CTL_INSWAP			(0x1 << 23)
#define AES_CTL_KOUTSWAP		(0x1 << 24)
#define AES_CTL_KINSWAP			(0x1 << 25)
#define AES_STS			0x104
#define AES_STS_BUSY			(0x1 << 0)
#define AES_STS_INBUFEMPTY		(0x1 << 8)
#define AES_STS_INBUFFULL		(0x1 << 9)
#define AES_STS_INBUFERR		(0x1 << 10)
#define AES_STS_CNTERR			(0x1 << 12)
#define AES_STS_OUTBUFEMPTY		(0x1 << 16)
#define AES_STS_OUTBUFFULL		(0x1 << 17)
#define AES_STS_OUTBUFERR		(0x1 << 18)
#define AES_STS_BUSERR			(0x1 << 20)
#define AES_STS_KSERR			(0x1 << 21)
#define AES_DATIN		0x108
#define AES_DATOUT		0x10C
#define AES_KEY(x)		(0x110 + ((x) * 0x04))
#define AES_IV(x)		(0x130 + ((x) * 0x04))
#define AES_SADDR		0x140
#define AES_DADDR		0x144
#define AES_CNT			0x148

#define HMAC_CTL		0x300
#define HMAC_CTL_START			(0x1 << 0)
#define HMAC_CTL_STOP			(0x1 << 1)
#define HMAC_CTL_DMAFIRST		(0x1 << 4)
#define HMAC_CTL_DMALAST		(0x1 << 5)
#define HMAC_CTL_DMACSCAD		(0x1 << 6)
#define HMAC_CTL_DMAEN			(0x1 << 7)
#define HMAC_CTL_OPMODE_OFFSET		8
#define HMAC_CTL_OPMODE_MASK		(0x7 << 8)
#define HMAC_CTL_HMACEN			(0x1 << 11)
#define HMAC_CTL_SHA3EN			(0x1 << 12)
#define HMAC_CTL_SM3EN			(0x1 << 13)
#define HMAC_CTL_MD5EN			(0x1 << 14)
#define HMAC_CTL_FBIN			(0x1 << 20)
#define HMAC_CTL_FBOUT			(0x1 << 21)
#define HMAC_CTL_OUTSWAP		(0x1 << 22)
#define HMAC_CTL_INSWAP			(0x1 << 23)
#define HMAC_CTL_NEXTDGST		(0x1 << 24)
#define HMAC_CTL_FINISHDGST		(0x1 << 25)
#define HMAC_STS		0x304
#define HMAC_STS_BUSY			(0x1 << 0)
#define HMAC_STS_DMABUSY		(0x1 << 1)
#define HMAC_STS_SHAKEBUSY		(0x1 << 2)
#define HMAC_STS_DMAERR			(0x1 << 8)
#define HMAC_STS_KSERR			(0x1 << 9)
#define HMAC_STS_DATINREQ		(0x1 << 16)
#define HMAC_DGST(x)		(0x308 + ((x) * 0x04))
#define HMAC_KEYCNT		0x348
#define HMAC_SADDR		0x34C
#define HMAC_DMACNT		0x350
#define HMAC_DATIN		0x354
#define HMAC_FDBCK(x)		(0x358 + ((x) * 0x04))
#define HMAC_FDBCK_WCNT			88
#define HMAC_FBADDR		0x4FC
#define HMAC_SHAKEDGST(x)	(0x500 + ((x) * 0x04))
#define HMAC_SHAKEDGST_WCNT		42

#define ECC_CTL			0x800
#define ECC_CTL_START			(0x1 << 0)
#define ECC_CTL_STOP			(0x1 << 1)
#define ECC_CTL_ECDSAS			(0x1 << 4)
#define ECC_CTL_ECDSAR			(0x1 << 5)
#define ECC_CTL_DMAEN			(0x1 << 7)
#define ECC_CTL_FSEL			(0x1 << 8)
#define ECC_CTL_ECCOP_OFFSET		9
#define ECC_CTL_ECCOP_MASK		(0x3 << 9)
#define ECC_CTL_MODOP_OFFSET		11
#define ECC_CTL_MODOP_MASK		(0x3 << 9)
#define ECC_CTL_CSEL			(0x1 << 13)
#define ECC_CTL_SCAP			(0x1 << 14)
#define ECC_CTL_LDAP1			(0x1 << 16)
#define ECC_CTL_LDAP2			(0x1 << 17)
#define ECC_CTL_LDA			(0x1 << 18)
#define ECC_CTL_LDB			(0x1 << 19)
#define ECC_CTL_LDN			(0x1 << 20)
#define ECC_CTL_LDK			(0x1 << 21)
#define ECC_CTL_CURVEM_OFFSET		22
#define ECC_CTL_CURVEM_MASK		(0x3ff << 22)
#define ECC_STS			0x804
#define ECC_STS_BUSY			(0x1 << 0)
#define ECC_STS_DMABUSY			(0x1 << 1)
#define ECC_STS_BUSERR			(0x1 << 16)
#define ECC_STS_KSERR			(0x1 << 17)
#define ECC_X1			0x808
#define ECC_Y1			0x850
#define ECC_X2			0x898
#define ECC_Y2			0x8E0
#define ECC_A			0x928
#define ECC_B			0x970
#define ECC_N			0x9B8
#define ECC_K			0xA00
#define ECC_KEY_WCNT			18
#define ECC_SADDR		0xA48
#define ECC_DADDR		0xA4C
#define ECC_STARTREG		0xA50
#define ECC_WORDCNT		0xA54

#define RSA_CTL			0xB00
#define RSA_CTL_START			(0x1 << 0)
#define RSA_CTL_STOP			(0x1 << 1)
#define RSA_CTL_CRT			(0x1 << 2)
#define RSA_CTL_CRTBYP			(0x1 << 3)
#define RSA_CTL_KEYLENG_OFFSET		4
#define RSA_CTL_KEYLENG_MASK		(0x3 << 4)
#define RSA_CTL_SCAP			(0x1 << 8)
#define RSA_STS			0xB04
#define RSA_STS_BUSY			(0x1 << 0)
#define RSA_STS_DMABUSY			(0x1 << 1)
#define RSA_STS_BUSERR			(0x1 << 16)
#define RSA_STS_CTLERR			(0x1 << 17)
#define RSA_STS_KSERR			(0x1 << 18)
#define RSA_SADDR0		0xB08
#define RSA_SADDR1		0xB0C
#define RSA_SADDR2		0xB10
#define RSA_SADDR3		0xB14
#define RSA_SADDR4		0xB18
#define RSA_DADDR		0xB1C
#define RSA_MADDR0		0xB20
#define RSA_MADDR1		0xB24
#define RSA_MADDR2		0xB28
#define RSA_MADDR3		0xB2C
#define RSA_MADDR4		0xB30
#define RSA_MADDR5		0xB34
#define RSA_MADDR6		0xB38

#define PRNG_KSCTL		0xF00
#define PRNG_KSCTL_NUM_OFFSET		0
#define PRNG_KSCTL_NUM_MASK		(0x1f << 0)
#define PRNG_KSCTL_KEYSRC		(0x1 << 8)
#define PRNG_KSCTL_TRUST		(0x1 << 16)
#define PRNG_KSCTL_PRIV			(0x1 << 18)
#define PRNG_KSCTL_ECDH			(0x1 << 19)
#define PRNG_KSCTL_ECDSA		(0x1 << 20)
#define PRNG_KSCTL_WDST			(0x1 << 21)
#define PRNG_KSCTL_WSDST_OFFSET		22
#define PRNG_KSCTL_WSDST_MASK		(0x3 << 22)
#define PRNG_KSCTL_OWNER_OFFSET		24
#define PRNG_KSCTL_OWNER_MASK		(0x7 << 24)
#define PRNG_KSSTS		0xF04
#define PRNG_KSSTS_NUM_OFFSET		0
#define PRNG_KSSTS_NUM_MASK		(0x1f << 0)
#define PRNG_KSSTS_KCTLERR		(0x1 << 16)

#define AES_KSCTL		0xF10
#define AES_KSCTL_NUM_OFFSET		0
#define AES_KSCTL_NUM_MASK		(0x1f << 0)
#define AES_KSCTL_RSRC			(0x1 << 5)
#define AES_KSCTL_RSSRC_OFFSET		6
#define AES_KSCTL_RSSRC_MASK		(0x3 << 6)

#define HMAC_KSCTL		0xF30
#define HMAC_KSCTL_NUM_OFFSET		0
#define HMAC_KSCTL_NUM_MASK		(0x1f << 0)
#define HMAC_KSCTL_RSRC			(0x1 << 5)
#define HMAC_KSCTL_RSSRC_OFFSET		6
#define HMAC_KSCTL_RSSRC_MASK		(0x3 << 6)

#define ECC_KSCTL		0xF40
#define ECC_KSCTL_NUMK_OFFSET		0
#define ECC_KSCTL_NUMK_MASK		(0x1f << 0)
#define ECC_KSCTL_RSRCK			(0x1 << 5)
#define ECC_KSCTL_RSSRCK_OFFSET		6
#define ECC_KSCTL_RSSRCK_MASK		(0x3 << 6)
#define ECC_KSCTL_TRUST			(0x1 << 16)
#define ECC_KSCTL_PRIV			(0x1 << 18)
#define ECC_KSCTL_XY			(0x1 << 20)
#define ECC_KSCTL_WDST			(0x1 << 21)
#define ECC_KSCTL_WSDST_OFFSET		22
#define ECC_KSCTL_WSDST_MASK		(0x3 << 22)
#define ECC_KSCTL_OWNER_OFFSET		24
#define ECC_KSCTL_OWNER_MASK		(0x7 << 24)
#define ECC_KSSTS		0xF44
#define ECC_KSSTS_NUM_OFFSET		0
#define ECC_KSSTS_NUM_MASK		(0x1f << 0)
#define ECC_KSXY			0xF48
#define ECC_KSXY_NUMX_OFFSET		0
#define ECC_KSXY_NUMX_MASK		(0x1f << 0)
#define ECC_KSXY_RSRCXY			(0x1 << 5)
#define ECC_KSXY_RSSRCX_OFFSET		6
#define ECC_KSXY_RSSRCX_MASK		(0x3 << 6)
#define ECC_KSXY_NUMY_OFFSET		8
#define ECC_KSXY_NUMY_MASK		(0x1f << 8)
#define ECC_KSXY_RSSRCY_OFFSET		14
#define ECC_KSXY_RSSRCY_MASK		(0x3 << 14)

#define RSA_KSCTL		0xF50
#define RSA_KSCTL_NUM_OFFSET		0
#define RSA_KSCTL_NUM_MASK		(0x1f << 0)
#define RSA_KSCTL_RSRC			(0x1 << 5)
#define RSA_KSCTL_RSSRC_OFFSET		6
#define RSA_KSCTL_RSSRC_MASK		(0x3 << 6)
#define RSA_KSCTL_BKNUM_OFFSET		8
#define RSA_KSCTL_BKNUM_MASK		(0x1f << 8)
#define RSA_KSSTS0		0xF54
#define RSA_KSSTS0_NUM0_OFFSET		0
#define RSA_KSSTS0_NUM0_MASK		(0x1f << 0)
#define RSA_KSSTS0_NUM1_OFFSET		8
#define RSA_KSSTS0_NUM1_MASK		(0x1f << 8)
#define RSA_KSSTS0_NUM2_OFFSET		16
#define RSA_KSSTS0_NUM2_MASK		(0x1f << 16)
#define RSA_KSSTS0_NUM3_OFFSET		24
#define RSA_KSSTS0_NUM3_MASK		(0x1f << 24)
#define RSA_KSSTS1		0xF58
#define RSA_KSSTS1_NUM4_OFFSET		0
#define RSA_KSSTS1_NUM4_MASK		(0x1f << 0)
#define RSA_KSSTS1_NUM5_OFFSET		8
#define RSA_KSSTS1_NUM5_MASK		(0x1f << 8)
#define RSA_KSSTS1_NUM6_OFFSET		16
#define RSA_KSSTS1_NUM6_MASK		(0x1f << 16)
#define RSA_KSSTS1_NUM7_OFFSET		24
#define RSA_KSSTS1_NUM7_MASK		(0x1f << 24)

#define AES_KEYSZ_SEL_128       (0x0 << AES_CTL_KEYSZ_OFFSET)
#define AES_KEYSZ_SEL_192       (0x1 << AES_CTL_KEYSZ_OFFSET)
#define AES_KEYSZ_SEL_256       (0x2 << AES_CTL_KEYSZ_OFFSET)

#define AES_MODE_ECB            (0x00 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CBC            (0x01 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CFB            (0x02 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_OFB            (0x03 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CTR            (0x04 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CBC_CS1        (0x10 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CBC_CS2        (0x11 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CBC_CS3        (0x12 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_GCM            (0x20 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_GHASH          (0x21 << AES_CTL_OPMODE_OFFSET)
#define AES_MODE_CCM            (0x22 << AES_CTL_OPMODE_OFFSET)

#define SHA_OPMODE_SHA1		(0x0 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHA224	(0x5 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHA256	(0x4 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHA384	(0x7 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHA512	(0x6 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHAKE128	(0x0 << HMAC_CTL_OPMODE_OFFSET)
#define SHA_OPMODE_SHAKE256	(0x1 << HMAC_CTL_OPMODE_OFFSET)

#define ECCOP_POINT_MUL		(0x0 << ECC_CTL_ECCOP_OFFSET)
#define ECCOP_MODULE		(0x1 << ECC_CTL_ECCOP_OFFSET)
#define ECCOP_POINT_ADD		(0x2 << ECC_CTL_ECCOP_OFFSET)
#define ECCOP_POINT_DOUBLE	(0x3 << ECC_CTL_ECCOP_OFFSET)

#define MODOP_DIV		(0x0 << ECC_CTL_MODOP_OFFSET)
#define MODOP_MUL		(0x1 << ECC_CTL_MODOP_OFFSET)
#define MODOP_ADD		(0x2 << ECC_CTL_MODOP_OFFSET)
#define MODOP_SUB		(0x3 << ECC_CTL_MODOP_OFFSET)

#define AES_BUFF_SIZE		(PAGE_SIZE)
#define SHA_BUFF_SIZE		(PAGE_SIZE)
#define SHA_FDBCK_SIZE		(HMAC_FDBCK_WCNT * 4)

/*-------------------------------------------------------------------------*/
/*   AES                                                                   */
/*-------------------------------------------------------------------------*/

struct nu_aes_dev;

typedef int (*nu_aes_fn_t)(struct nu_aes_dev *, int);

struct nu_aes_base_ctx {
	struct nu_aes_dev *dd;
	nu_aes_fn_t  start;
	u32	   mode;
	int	   keylen;
	u32	   keysz_sel;
	u32	   aes_key[8];

	u8	   tag[16];
	int	   authsize;
	int	   assoclen;
	int	   text_len;
};

struct nu_aes_ctx {
	struct nu_aes_base_ctx	base;
};

struct nu_aes_dev {
	struct list_head	list;
	struct device		*dev;
	void __iomem		*reg_base;
	u32			flags;

	struct crypto_async_request	*areq;
	struct nu_aes_base_ctx	*ctx;

	nu_aes_fn_t		resume;

	spinlock_t		lock;
	struct crypto_queue	queue;

	struct tasklet_struct	done_task;
	struct tasklet_struct	queue_task;

	u8			inbuf[AES_BUFF_SIZE] __aligned(32);
	dma_addr_t		dma_inbuf;  /* AES input buffer DMA address  */
	u8			outbuf[AES_BUFF_SIZE] __aligned(32);
	dma_addr_t		dma_outbuf; /* AES output buffer DMA address */

	/*
	 *  for request handling
	 */
	int			req_len;
	int			dma_len;
	struct scatterlist	*in_sg;
	struct scatterlist	*out_sg;
	int			in_sg_off;
	int			out_sg_off;

	/*
	 * for optee client driver
	 */
	bool			use_optee;
	struct tee_client_device *tee_cdev;
	struct tee_context	*octx;
	u32			session_id;         /* optee session */
	struct tee_shm		*shm_pool;
	u32			*va_shm;
	u32			crypto_session_id;  /* crypto session */
};

/*-------------------------------------------------------------------------*/
/*   SHA                                                                   */
/*-------------------------------------------------------------------------*/

struct nu_sha_dev;

struct nu_sha_ctx {
	struct nu_sha_dev  *dd;
	u32	    hash_mode;
	int	    hmac_key_len;	   /* HMAC key length in bytes       */
	int         bufcnt;                /* byte count in buffer           */
	u8	    buffer[SHA_BUFF_SIZE] __aligned(32); /* data buffer      */
	dma_addr_t  dma_buff;		   /* DMA mapping address of buffer[]*/
	u8	    fdbck[SHA_FDBCK_SIZE] __aligned(32); /* feedback buffer  */
	dma_addr_t  dma_fdbck;		   /* DMA mapping address of fdbck[] */
};

struct nu_sha_reqctx {
	struct nu_sha_dev  *dd;
	u32	   flags;
	u32	   op;
	u32	   reg_ctl;		   /* SHA control register setting   */

	int	   digest_len;		   /* digest length in bytes	     */
	int	   block_size;             /* SHA block size		     */
	int	   dma_max_size;           /* Maximum DMA buffer size used   */

	struct scatterlist  *sg;
	u32        sg_off;                 /* offset in sg		     */
	u32        req_len;		   /* remaining data count of request*/
};

struct nu_sha_dev {
	struct list_head	list;
	struct device		*dev;
	void __iomem		*reg_base;
	u32			flags;

	spinlock_t		lock;

	struct crypto_queue	queue;
	struct ahash_request	*req;

	struct tasklet_struct	done_task;
	struct tasklet_struct	queue_task;

	/*
	 * for optee client driver
	 */
	bool			use_optee;
	struct tee_client_device *tee_cdev;
	struct tee_context	*octx;
	u32			session_id;  /* optee session */
	struct tee_shm		*shm_pool;
	u32			*va_shm;
	u32			crypto_session_id;  /* crypto session */
};


/*-------------------------------------------------------------------------*/
/*   ECC                                                                   */
/*-------------------------------------------------------------------------*/

#define NU_ECC_MAX_LEN		(256/8)
#define NU_ECC_MAX_DIGITS	(NU_ECC_MAX_LEN/8)

struct ecc_curve {
	int        optee_curve_id;     /* optee ECC PTA defined curve ID */
	char	   name[16];
	u32	   keylen;
	u8	   g_x[NU_ECC_MAX_LEN];
	u8	   g_y[NU_ECC_MAX_LEN];
	u8	   p[NU_ECC_MAX_LEN];
	u8	   n[NU_ECC_MAX_LEN];
	u8	   a[NU_ECC_MAX_LEN];
	u8	   b[NU_ECC_MAX_LEN];
};

struct nu_ecdh_ctx {
	struct nu_ecc_dev  *dd;
	int	   curve_id;
	const struct ecc_curve	*curve;
	int	   keylen;
	u8	   private_key[NU_ECC_MAX_LEN];
};


struct nu_ecc_dev {
	struct list_head	list;
	struct device		*dev;
	void __iomem		*reg_base;
	spinlock_t		lock;

	/*
	 * for optee client driver
	 */
	bool			use_optee;
	struct tee_client_device *tee_cdev;
	struct tee_context	*octx;
	u32			session_id;  /* optee session */
	struct tee_shm		*shm_pool;
	u32			*va_shm;
};


/*-------------------------------------------------------------------------*/
/*   RSA                                                                   */
/*-------------------------------------------------------------------------*/

#define NU_RSA_MAX_BIT_LEN	(4096)
#define NU_RSA_MAX_BYTE_LEN	(NU_RSA_MAX_BIT_LEN/8)
#define NU_RSA_MAX_WORD_LEN	(NU_RSA_MAX_BIT_LEN/32)

#define RSA_REG_RAM_SIZE	(NU_RSA_MAX_BYTE_LEN)
#define RSA_BUFF_SIZE		(RSA_REG_RAM_SIZE * 13)

#define M_OFF			(0)
#define N_OFF			(RSA_REG_RAM_SIZE)
#define E_OFF			(RSA_REG_RAM_SIZE*2)
#define D_OFF			(RSA_REG_RAM_SIZE*2)
#define P_OFF			(RSA_REG_RAM_SIZE*3)
#define Q_OFF			(RSA_REG_RAM_SIZE*4)
#define ANS_OFF			(RSA_REG_RAM_SIZE*5)
#define MADR0_OFF		(RSA_REG_RAM_SIZE*6)
#define MADR1_OFF		(RSA_REG_RAM_SIZE*7)
#define MADR2_OFF		(RSA_REG_RAM_SIZE*8)
#define MADR3_OFF		(RSA_REG_RAM_SIZE*9)
#define MADR4_OFF		(RSA_REG_RAM_SIZE*10)
#define MADR5_OFF		(RSA_REG_RAM_SIZE*11)
#define MADR6_OFF		(RSA_REG_RAM_SIZE*12)

struct nu_rsa_ctx {
	struct nu_rsa_dev  *dd;
	void __iomem  *reg_base;
	int	   rsa_bit_len;

	u8         buffer[RSA_BUFF_SIZE] __aligned(32);
	u8	   public_key[RSA_REG_RAM_SIZE];
	int	   public_key_size;
	u8	   private_key[RSA_REG_RAM_SIZE];
	int	   private_key_size;
	dma_addr_t dma_buff;
};

struct nu_rsa_dev {
	struct list_head	list;
	struct device		*dev;
	void __iomem		*reg_base;

	/*
	 * for optee client driver
	 */
	bool			use_optee;
	struct tee_client_device *tee_cdev;
	struct tee_context	*octx;
	u32			session_id;  /* optee session */
	struct tee_shm		*shm_pool;
	u32			*va_shm;
};

struct nuvoton_crypto_dev {
	struct device           *dev;
	bool                    use_optee;
	struct tee_client_device *tee_cdev;
	void __iomem            *reg_base;
	unsigned long           prng;
	struct nu_aes_dev       aes_dd;
	struct nu_sha_dev       sha_dd;
	struct nu_ecc_dev       ecc_dd;
	struct nu_rsa_dev       rsa_dd;
};

/*-------------------------------------------------------------------------*/
/*   OP-TEE                                                                */
/*-------------------------------------------------------------------------*/
#define CRYPTO_SHM_SIZE		(0x4000)

#define TEE_ERROR_CRYPTO_BUSY		0x00000001
#define TEE_ERROR_CRYPTO_FAIL		0x00000002
#define TEE_ERROR_CRYPTO_INVALID	0x00000003
#define TEE_ERROR_CRYPTO_TIMEOUT	0x00000004

/* Crypto session class */
#define C_CODE_AES			0x04
#define C_CODE_SHA			0x05

enum {
	CURVE_P_192  = 0x01,
	CURVE_P_224  = 0x02,
	CURVE_P_256  = 0x03,
	CURVE_P_384  = 0x04,
	CURVE_P_521  = 0x05,
	CURVE_K_163  = 0x11,
	CURVE_K_233  = 0x12,
	CURVE_K_283  = 0x13,
	CURVE_K_409  = 0x14,
	CURVE_K_571  = 0x15,
	CURVE_B_163  = 0x21,
	CURVE_B_233  = 0x22,
	CURVE_B_283  = 0x23,
	CURVE_B_409  = 0x24,
	CURVE_B_571  = 0x25,
	CURVE_KO_192 = 0x31,
	CURVE_KO_224 = 0x32,
	CURVE_KO_256 = 0x33,
	CURVE_BP_256 = 0x41,
	CURVE_BP_384 = 0x42,
	CURVE_BP_512 = 0x43,
	CURVE_SM2_256 = 0x50,
	CURVE_25519  = 0x51,
	CURVE_UNDEF,
};

/*
 * PTA_CMD_CRYPTO_INIT - Initialize Crypto Engine
 *
 * param[0] unused
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 * TEE_ERROR_CRYPTO_FAIL - Initialization failed
 */
#define PTA_CMD_CRYPTO_INIT		1

/*
 * PTA_CMD_CRYPTO_OPEN_SESSION - open a crypto session
 *
 * param[0] (in value)  - value.a: session class
 * param[1] (out value) - value.a: session ID
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - failed
 */
#define PTA_CMD_CRYPTO_OPEN_SESSION	2

/*
 * PTA_CMD_CRYPTO_CLOSE_SESSION - close an opened crypto session
 *
 * param[0] (in value)  - value.a: session class
 * param[1] (in value)  - value.a: session ID
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - failed
 */
#define PTA_CMD_CRYPTO_CLOSE_SESSION	3

/*
 * PTA_CMD_CRYPTO_AES_RUN - Run AES encrypt/decrypt
 *
 * param[0] (in value) - value.a: crypto session ID
 *                     - value.b: register AES_KSCTL
 * param[1] (inout memref) - memref.size: size of register map
 *                           memref.buffer: register map buffer
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - AES encrypt/decrypt operation failed
 */
#define PTA_CMD_CRYPTO_AES_RUN		5

/*
 * PTA_CMD_CRYPTO_SHA_START - Start a SHA session
 *
 * param[0] (in value) - value.a: session ID
 * param[1] (in value) - value.a: HMAC_CTL
 *                     - value.b: HMAC_KSCTL
 * param[2] (in value) - value.a: HMAC key length
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - SHA operation failed
 */
#define PTA_CMD_CRYPTO_SHA_START	8

/*
 * PTA_CMD_CRYPTO_SHA_UPDATE - Update SHA input data
 *
 * param[0] (in value) - value.a: session ID
 *                     - value.b: HMAC_KSCTL
 * param[1] (inout memref) - memref.size: size of register map
 *                           memref.buffer: register map buffer
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - SHA operation failed
 */
#define PTA_CMD_CRYPTO_SHA_UPDATE	9

/*
 * PTA_CMD_CRYPTO_SHA_FINAL - final update SHA input data and
 *                            get output digest
 *
 * param[0] (in value) - value.a: session ID
 *                     - value.b: digest byte length
 * param[1] (inout memref) - memref.size: size of register map
 *                           memref.buffer: register map buffer
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - SHA operation failed
 */
#define PTA_CMD_CRYPTO_SHA_FINAL	10

/*
 * PTA_CMD_CRYPTO_ECC_PMUL - Run ECC point multiplication
 *
 * param[0] (in value) - value.a: ECC curve ID
 * param[1] (inout memref) - memref.size: size of register map
 *                           memref.buffer: register map buffer
 * param[2] (in value) - value.a: shm offset of parameter block
 *                     - value.b: shm offset of output buffer
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - ECC operation failed
 */
#define PTA_CMD_CRYPTO_ECC_PMUL		15

/*
 * PTA_CMD_CRYPTO_RSA_RUN - Run RSA engine
 *
 * param[0] unused
 * param[1] (inout memref) - memref.size: size of register map
 *                           memref.buffer: register map buffer
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_CRYPTO_INVALID - Invalid input param
 * TEE_ERROR_CRYPTO_FAIL - RSA operation failed
 */
#define PTA_CMD_CRYPTO_RSA_RUN		20

static inline int optee_ctx_match(struct tee_ioctl_version_data *ver,
				  const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

extern int nuvoton_prng_probe(struct device *dev, void __iomem *reg_base,
				unsigned long *data);
extern int nuvoton_prng_remove(struct device *dev, unsigned long data);

extern int nuvoton_aes_probe(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);
extern int nuvoton_aes_remove(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);

extern int nuvoton_sha_probe(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);
extern int nuvoton_sha_remove(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);

extern int nuvoton_ecc_probe(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);
extern int nuvoton_ecc_remove(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);

extern int nuvoton_rsa_probe(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);
extern int nuvoton_rsa_remove(struct device *dev,
				struct nuvoton_crypto_dev *nu_cryp_dev);

/* functions in crypto/ecc.c */
extern int ecc_gen_privkey(unsigned int curve_id, unsigned int ndigits,
		u64 *privkey);
extern int ecc_is_key_valid(unsigned int curve_id, unsigned int ndigits,
		const u64 *private_key, unsigned int private_key_len);

#endif /* __NUVOTON_CRYPTO_H__ */
