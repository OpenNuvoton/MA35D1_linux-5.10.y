/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Header file for the MA35D1 DMA Controller driver
 *
 * Copyright (C) 2020 Nuvoton Technology Corp
 */

#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

struct ma35d1_dma_data {
        u32 timeout_prescaler;
        u32 timeout_counter;
        u32 en_sc;
};

struct ma35d1_dma_done {
	int		ch;
	u32		base_addr;
	bool            done;
	int             timeout;
	int             remain;
	void            *callback_param;
};


struct ma35d1_dma_config {
	u32 timeout_prescaler;
	u32 timeout_counter;
	u32 en_sc;
	u32 reqsel;
};

/**
 * struct ma35d1_dma_chan_data - platform specific data for a DMA channel
 * @name: name of the channel,
 *        used for getting the right clock for the channel
 * @base: mapped registers
 * @irq: interrupt number used by this channel
 */
struct ma35d1_dma_chan_data {
	const char			*name;
	void __iomem			*base;
	int				irq;
};

/**
 * struct ma35d1_dma_platform_data - platform data
 * for the dmaengine driver
 *
 * @channels: array of channels which are passed to the driver
 * @num_channels: number of channels in the array
 *
 * This structure is passed to the DMA engine driver via platform data. For
 * M2P channels, contract is that even channels are for TX and odd for RX.
 * There is no requirement for the M2M channels.
 */
struct ma35d1_dma_platform_data {
	struct ma35d1_dma_chan_data	*channels;
	size_t				num_channels;
	void __iomem			*base;
	unsigned int			irq;
};

#endif /* __ASM_ARCH_DMA_H */
