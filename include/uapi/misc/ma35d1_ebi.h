/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Nuvoton MA35D1 EBI
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 */
#ifndef _MA35D1_EBI_H_
#define _MA35D1_EBI_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define EBI_IOC_MAGIC		'e'
#define EBI_IOC_SET		_IOW(EBI_IOC_MAGIC, 0, unsigned int *)

struct ma35d1_set_ebi {
	unsigned int bank;
	unsigned int busmode;
	unsigned int CSActiveLevel;
	unsigned int base;
	unsigned int size;
	unsigned int width;
};

#endif /* _MA35D1_EBI_H_ */
