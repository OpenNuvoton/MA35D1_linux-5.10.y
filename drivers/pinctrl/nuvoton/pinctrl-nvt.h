/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2023 Nuvoton Technology Corp.
 *
 * Author: Shan-Chun Hung <schung@nuvoton.com>
 */

#ifndef __PINCTRL_NVT_H__
#define __PINCTRL_NVT_H__

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>

/**
 * struct nvt_mux_desc - hardware mux descriptor
 * @name: mux function name
 * @muxval: mux register bit value
 */
struct nvt_mux_desc {
	const char *name;
	u32 muxval;
};

/**
 * struct nvt_pin_data - hardware per-pin data
 * @offset: MFP register offset
 * @shift: MFP bit position
 * @muxes: available mux function names and corresponding register values
 */
struct nvt_pin_data {
	u32 offset;
	u32 shift;
	struct nvt_mux_desc *muxes;
};

/**
 * struct nvt_pinctrl_soc_info - all pin info of SoC
 * @pins: pin descriptions
 * @npins: total pin count
 * @get_pin_num: a function return pin # based on offset and shift
 */
struct nvt_pinctrl_soc_info {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;
	int (*get_pin_num)(int offset, int shift);
};

#define NVT_PIN(num, n, o, s,  ...) {		\
	.number = num,					\
	.name = #n,					\
	.drv_data = &(struct nvt_pin_data) {		\
		.offset = o,				\
		.shift = s,				\
		.muxes = (struct nvt_mux_desc[]) {	\
			 __VA_ARGS__, { } },		\
	},						\
}

#define NVT_MUX(_val, _name) {				\
	.name = _name,					\
	.muxval = _val,					\
}

int nvt_pinctrl_probe(struct platform_device *pdev, const struct nvt_pinctrl_soc_info *info);
int nvt_pinctrl_suspend(struct device *dev);
int nvt_pinctrl_resume(struct device *dev);

#endif /* __PINCTRL_NVT_H__ */
