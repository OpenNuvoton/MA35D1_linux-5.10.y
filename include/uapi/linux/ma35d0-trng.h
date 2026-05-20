/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_MA35D0_TRNG_H
#define _UAPI_LINUX_MA35D0_TRNG_H

#include <linux/ioctl.h>

#define MA35D0_TRNG_IOC_MAGIC		0xb8

#define MA35D0_TRNG_KS_OWNER_AES	0
#define MA35D0_TRNG_KS_OWNER_HMAC	1
#define MA35D0_TRNG_KS_OWNER_ECC	4
#define MA35D0_TRNG_KS_OWNER_CPU	5

#define MA35D0_TRNG_KS_AES		MA35D0_TRNG_KS_OWNER_AES
#define MA35D0_TRNG_KS_HMAC		MA35D0_TRNG_KS_OWNER_HMAC
#define MA35D0_TRNG_KS_ECC		MA35D0_TRNG_KS_OWNER_ECC
#define MA35D0_TRNG_KS_CPU		MA35D0_TRNG_KS_OWNER_CPU

/*
 * Generate a random key to Key Store SRAM through the OP-TEE backed
 * MA35D0 TRNG device. The input integer is one MA35D0_TRNG_KS_OWNER_*
 * value and is overwritten with the returned SRAM key number on success.
 * Non-OP-TEE MA35D0 TRNG devices return -EOPNOTSUPP.
 */
#define MA35D0_TRNG_IOC_WRITE_KS \
	_IOWR(MA35D0_TRNG_IOC_MAGIC, 0x00, int)

#endif /* _UAPI_LINUX_MA35D0_TRNG_H */
