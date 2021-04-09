/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Nuvoton MA35D1 Key Store
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#ifndef __MA35D1_KS_H__
#define __MA35D1_KS_H__

#include <linux/types.h>

#define KS_CTL			0x00
#define KS_CTL_START			(0x1 << 0)
#define KS_CTL_OPMODE_Pos		1
#define KS_CTL_OPMODE_Msk		(0x7 << 1)
#define KS_CTL_CONT			(0x1 << 7)
#define KS_CTL_INIT			(0x1 << 8)
#define KS_CTL_SILENT			(0x1 << 10)
#define KS_CTL_SCMB			(0x1 << 11)
#define KS_CTL_TCLR			(0x1 << 14)
#define KS_CTL_IEN			(0x1 << 15)
#define KS_METADATA		0x04
#define KS_STS			0x08
#define KS_STS_IF			(0x1 << 0)
#define KS_STS_EIF			(0x1 << 1)
#define KS_STS_BUSY			(0x1 << 2)
#define KS_STS_SRAMFULL			(0x1 << 3)
#define KS_STS_INITDONE			(0x1 << 7)
#define KS_STS_RAMINV_Pos	        8
#define KS_STS_RAMINV_Msk		(0xFFFFFF << 8)
#define KS_REMAIN		0x0C
#define KS_REMAIN_RRMNG_Pos		0
#define KS_REMAIN_RRMNG_Msk		(0x1FFF << 0)
#define KS_SCMBKEY(x)		(0x10 + ((x) * 0x04))
#define KS_KEY(x)		(0x20 + ((x) * 0x04))
#define KS_OTPSTS		0x40

#define KS_CLT_FUNC_MASK	((KS_CTL_IEN) | (KS_CTL_TCLR) | \
				 (KS_CTL_SCMB) | (KS_CTL_SILENT))

#define KS_OP_READ		(0x0 << KS_CTL_OPMODE_Pos)
#define KS_OP_WRITE		(0x1 << KS_CTL_OPMODE_Pos)
#define KS_OP_ERASE		(0x2 << KS_CTL_OPMODE_Pos)
#define KS_OP_ERASE_ALL		(0x3 << KS_CTL_OPMODE_Pos)
#define KS_OP_REVOKE		(0x4 << KS_CTL_OPMODE_Pos)
#define KS_OP_REMAN		(0x5 << KS_CTL_OPMODE_Pos)

#define KS_MAX_KEY_SIZE		(4096)

#define KS_SRAM_KEY_CNT		(32)
#define KS_OTP_KEY_CNT		(9)

#define KS_METADATA_SEC_Msk       (1<<0)
#define KS_METADATA_PRIV_Msk      (1<<1)
#define KS_METADATA_READABLE_Msk  (1<<2)
#define KS_METADATA_RVK_Msk       (1<<3)
#define KS_METADATA_SIZE_Pos      (8)
#define KS_METADATA_SIZE_Msk      (0x1f << KS_METADATA_SIZE_Pos)
#define KS_METADATA_OWNER_Pos     (16)
#define KS_METADATA_OWNER_Msk     (0x7 << KS_METADATA_OWNER_Pos)
#define KS_METADATA_NUMBER_Pos    (20)
#define KS_METADATA_NUMBER_Msk    (0x3f << KS_METADATA_NUMBER_Pos)
#define KS_METADATA_DST_Pos       (30)
#define KS_METADATA_DST_Msk       (0x3 << KS_METADATA_DST_Pos)

#define KS_OWNER_AES      (0ul)
#define KS_OWNER_HMAC     (1ul)
#define KS_OWNER_RSA_EXP  (2ul)
#define KS_OWNER_RSA_MID  (3ul)
#define KS_OWNER_ECC      (4ul)
#define KS_OWNER_CPU      (5ul)

#define KS_META_AES       (0ul << KS_METADATA_OWNER_Pos) /* AES only     */
#define KS_META_HMAC      (1ul << KS_METADATA_OWNER_Pos) /* HMAC only    */
#define KS_META_RSA_EXP   (2ul << KS_METADATA_OWNER_Pos) /* RSA_EXP only */
#define KS_META_RSA_MID   (3ul << KS_METADATA_OWNER_Pos) /* RSA_MID only */
#define KS_META_ECC       (4ul << KS_METADATA_OWNER_Pos) /* ECC only     */
#define KS_META_CPU       (5ul << KS_METADATA_OWNER_Pos) /* CPU readable */

#define KS_META_128    (0ul << KS_METADATA_SIZE_Pos)  /* Key size 128 bits  */
#define KS_META_163    (1ul << KS_METADATA_SIZE_Pos)  /* Key size 163 bits  */
#define KS_META_192    (2ul << KS_METADATA_SIZE_Pos)  /* Key size 192 bits  */
#define KS_META_224    (3ul << KS_METADATA_SIZE_Pos)  /* Key size 224 bits  */
#define KS_META_233    (4ul << KS_METADATA_SIZE_Pos)  /* Key size 233 bits  */
#define KS_META_255    (5ul << KS_METADATA_SIZE_Pos)  /* Key size 255 bits  */
#define KS_META_256    (6ul << KS_METADATA_SIZE_Pos)  /* Key size 256 bits  */
#define KS_META_283    (7ul << KS_METADATA_SIZE_Pos)  /* Key size 283 bits  */
#define KS_META_384    (8ul << KS_METADATA_SIZE_Pos)  /* Key size 384 bits  */
#define KS_META_409    (9ul << KS_METADATA_SIZE_Pos)  /* Key size 409 bits  */
#define KS_META_512    (10ul << KS_METADATA_SIZE_Pos) /* Key size 512 bits  */
#define KS_META_521    (11ul << KS_METADATA_SIZE_Pos) /* Key size 521 bits  */
#define KS_META_571    (12ul << KS_METADATA_SIZE_Pos) /* Key size 571 bits  */
#define KS_META_1024   (16ul << KS_METADATA_SIZE_Pos) /* Key size 1024 bits */
#define KS_META_1536   (17ul << KS_METADATA_SIZE_Pos) /* Key size 1024 bits */
#define KS_META_2048   (18ul << KS_METADATA_SIZE_Pos) /* Key size 2048 bits */
#define KS_META_3072   (19ul << KS_METADATA_SIZE_Pos) /* Key size 1024 bits */
#define KS_META_4096   (20ul << KS_METADATA_SIZE_Pos) /* Key size 4096 bits */

#define KS_META_READABLE          (1ul << 2)  /* key caan be read by CPU */

#define KS_TOMETAKEY(x)           (((x) << KS_METADATA_NUMBER_Pos) & \
				   KS_METADATA_NUMBER_Msk)
#define KS_TOKEYIDX(x)            (((x) & KS_METADATA_NUMBER_Msk) >> \
				   KS_METADATA_NUMBER_Pos)


#define MA35D1_KS_MAGIC         'K'

#define NU_KS_IOCTL_READ         _IOWR(MA35D1_KS_MAGIC, 1, \
				       struct ks_read_args *)
#define NU_KS_IOCTL_WRITE_SRAM   _IOWR(MA35D1_KS_MAGIC, 2, \
				       struct ks_write_args *)
#define NU_KS_IOCTL_WRITE_OTP    _IOWR(MA35D1_KS_MAGIC, 3, \
				       struct ks_write_args *)
#define NU_KS_IOCTL_ERASE        _IOWR(MA35D1_KS_MAGIC, 4, \
				       struct ks_kidx_args *)
#define NU_KS_IOCTL_ERASE_ALL    _IOWR(MA35D1_KS_MAGIC, 5, \
				       unsigned long)
#define NU_KS_IOCTL_REVOKE       _IOWR(MA35D1_KS_MAGIC, 6, \
				       struct ks_kidx_args *)
#define NU_KS_IOCTL_GET_REMAIN   _IOWR(MA35D1_KS_MAGIC, 7, \
				       unsigned long)


/* type of key */
#define KS_SRAM                  0x0
#define KS_OTP                   0x2


struct ks_read_args {
	unsigned int  type;
	int           key_idx;
	int           word_cnt;        /* word count of the key */
	unsigned int  key[128];
};



struct ks_write_args {
	int           key_idx;
	unsigned int  meta_data;
	unsigned int  key[128];
};

struct ks_kidx_args {
	unsigned int  type;
	unsigned int  key_idx;
};


#endif /* __MA35D1_KS_H__ */
