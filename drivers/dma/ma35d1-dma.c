// SPDX-License-Identifier: GPL-2.0+
//
// drivers/dma/ma35d1-dma.c
//
// This file contains a driver for the Nuvoton MA35D1 DMA engine
// found on MA35D1
//
// Copyright (C) 2020 Nuvoton Technology Corp.

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/platform_data/dma-ma35d1.h>
#include <linux/of.h>
#include "dmaengine.h"

/* PDMA registers */
#define DSCT_CTL 0x000
#define PDMA_TXTYPE_Pos	(2)
#define PDMA_TXTYPE_Msk	(0x1ul << PDMA_TXTYPE_Pos)
#define PDMA_SAINC_Pos	(8)
#define PDMA_SAINC_Msk	(0x3ul << PDMA_SAINC_Pos)
#define PDMA_DAINC_Pos	(10)
#define PDMA_DAINC_Msk	(0x3ul << PDMA_DAINC_Pos)
#define PDMA_TXWIDTH_Pos (12)
#define PDMA_TXWIDTH_Msk (0x3ul << PDMA_TXWIDTH_Pos)

#define PDMA_OP_Msk 0x3
#define PDMA_OP_STOP 0x0
#define PDMA_OP_BASIC 0x1
#define	PDMA_OP_SCATTER	0x2
#define PDMA_TXCNT_Pos 16
#define PDMA_TXCNT_Msk (0xFFFFul<<PDMA_TXCNT_Pos)

#define DSCT_SA 0x004
#define DSCT_DA	0x008
#define DSCT_NEXT 0x00c
#define PDMA_CHCTL 0x400
#define PDMA_PAUSE 0x404
#define PDMA_SWREQ 0x408
#define PDMA_INTEN 0x418
#define PDMA_INTSTS 0x41C
#define PDMA_TDSTS 0x424
#define PDMA_TOUTEN 0x434
#define PDMA_TOUTIEN 0x438
#define PDMA_TOC 0x440
#define PDMA_CHRST 0x460
#define PDMA_TOUTPSC 0x470
#define PDMA_TOUTPSC1 0x474
#define PDMA_REQSEL 0x480

#define EN_HW_SG
//#define EN_PDMA_DEBUG
#ifdef EN_PDMA_DEBUG
#define ENTRY()	pr_info("Enter...%s()\n", __func__)
#define LEAVE()	pr_info("Leave...%s()\n", __func__)
#else
#define ENTRY()
#define LEAVE()
#endif

#ifdef EN_PDMA_DEBUG
#define DMA_DEBUG(fmt, arg...) pr_info(fmt, ##arg)
#define DMA_DEBUG2(fmt, arg...) pr_info(fmt, ##arg)
#else
#define DMA_DEBUG(fmt, arg...)
#define DMA_DEBUG2(fmt, arg...)
#endif

#define DMA_CHANNELS		10
#define DMA_MAX_CHAN_DESCRIPTORS	32
#define DMA_MAX_CHAN_BYTES		0x10000

struct ma35d1_dma_engine;
static int ma35d1_dma_slave_config_write(struct dma_chan *chan,
					  enum dma_transfer_direction dir,
					  struct dma_slave_config *config);

struct ma35d1_sg {
	u32 ctl;
	u32 src;
	u32 dst;
	u32 next;
};

/**
 * struct ma35d1_dma_desc - ma35d1 specific transaction descriptor
 * @src_addr: source address of the transaction
 * @dst_addr: destination address of the transaction
 * @size: size of the transaction (in bytes)
 * @complete: this descriptor is completed
 * @txd: dmaengine API descriptor
 * @tx_list: list of linked descriptors
 * @node: link used for putting this into a channel queue
 */
struct ma35d1_dma_desc {
	u32 src_addr;
	u32 dst_addr;
	size_t size;
	bool complete;
	struct dma_async_tx_descriptor txd;
	struct list_head tx_list;
	struct list_head node;
	struct ma35d1_dma_config config;
	u32 ctl;
	u32 dir;
};

/**
 * struct ma35d1_dma_chan - an ma35d1 DMA channel
 * @chan: dmaengine API channel
 * @edma: pointer to to the engine device
 * @regs: memory mapped registers
 * @irq: interrupt number of the channel
 * @clk: clock used by this channel
 * @tasklet: channel specific tasklet used for callbacks
 * @lock: lock protecting the fields following
 * @flags: flags for the channel
 * @buffer: which buffer to use next (0/1)
 * @active: flattened chain of descriptors currently being processed
 * @queue: pending descriptors which are handled next
 * @free_list: list of free descriptors which can be used
 * @runtime_addr: This is set via .device_config before slave operation is
 *                prepared
 * @runtime_ctrl: runtime values for the control register.
 *
 * As ma35d1 DMA controller doesn't support real chained DMA descriptors
 * we will have slightly different scheme here: @active points to a head of
 * flattened DMA descriptor chain.
 *
 * @queue holds pending transactions. These are linked through the first
 * descriptor in the chain. When a descriptor is moved to the active queue,
 * the first and chained descriptors are flattened into a single list.
 *
 * @chan.private holds pointer to &struct ma35d1_dma_data which contains
 * necessary channel configuration information.
 * For memcpy channels this must
 * be %NULL.
 */
#ifdef EN_HW_SG
#define SG_LEN 64
#endif
struct ma35d1_dma_chan {
	struct device *dev;
	struct dma_chan chan;
	const struct ma35d1_dma_engine *edma;
	void __iomem *regs;
	int irq;
	u32 ch;
	struct clk *clk;
	struct tasklet_struct tasklet;
	/* protects the fields following */
	spinlock_t lock;
	unsigned long flags;
	/* Channel is configured for cyclic transfers */
#define MA35D1_DMA_IS_CYCLIC		1

	struct list_head active;
	struct list_head queue;
	struct list_head free_list;
	u32 runtime_addr;
	u32 runtime_ctrl;
	struct dma_slave_config slave_config;

#ifdef EN_HW_SG
	u64 sg_addr;
	u64 sg_base[SG_LEN];
#endif
};

/**
 * struct ma35d1_dma_engine - the ma35d1 DMA engine instance
 * @dma_dev: holds the dmaengine device
 * @m2m: is this an M2M or M2P device
 * @hw_setup: method which sets the channel up for operation
 * @hw_shutdown: shuts the channel down and flushes whatever is left
 * @hw_submit: pushes active descriptor(s) to the hardware
 * @hw_interrupt: handle the interrupt
 * @num_channels: number of channels for this instance
 * @channels: array of channels
 *
 */
struct ma35d1_dma_engine {
	struct ma35d1_dma_platform_data *pdata;
	struct dma_device dma_dev;

	void (*hw_synchronize)(struct ma35d1_dma_chan *edmac);
	void (*hw_shutdown)(struct ma35d1_dma_chan *edmac);
	void (*hw_submit)(struct ma35d1_dma_chan *edmac);
	int (*hw_interrupt)(struct ma35d1_dma_chan *edmac);
#define INTERRUPT_UNKNOWN	0
#define INTERRUPT_DONE		1
#define INTERRUPT_NEXT_BUFFER	2
#define INTERRUPT_TIMEOUT       3

	size_t num_channels;
	struct ma35d1_dma_chan channels[];
};

static void ma35d1_pdma_edmac_write(struct ma35d1_dma_chan *edmac,
				     unsigned int reg, u32 value)
{
	__raw_writel(value, edmac->regs + reg);
}

static u32 ma35d1_pdma_edmac_read(struct ma35d1_dma_chan *edmac,
				   unsigned int reg)
{
	return __raw_readl(edmac->regs + reg);
}

static inline struct device *chan2dev(struct ma35d1_dma_chan *edmac)
{
	return &edmac->chan.dev->device;
}

static struct ma35d1_dma_chan *to_ma35d1_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct ma35d1_dma_chan, chan);
}

void ma35d1_set_transfer_mode(struct ma35d1_dma_chan *edmac,
			       uint32_t u32Peripheral)
{
	ma35d1_pdma_edmac_write(edmac, PDMA_REQSEL + (4 * (edmac->ch / 4)),
		(ma35d1_pdma_edmac_read(edmac, PDMA_REQSEL +
		(4 * (edmac->ch / 4))) & ~(0xFF << ((edmac->ch & 0x3) * 8))) |
		(u32Peripheral << ((edmac->ch & 0x3) * 8)));
}

/**
 * ma35d1_dma_set_active - set new active descriptor chain
 * @edmac: channel
 * @desc: head of the new active descriptor chain
 *
 * Sets @desc to be the head of the new active descriptor chain.
 * This is the chain which is processed next.
 * The active list must be empty before calling
 * this function.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static void ma35d1_dma_set_active(struct ma35d1_dma_chan *edmac,
				   struct ma35d1_dma_desc *desc)
{
	ENTRY();
	BUG_ON(!list_empty(&edmac->active));
	list_add_tail(&desc->node, &edmac->active);

	/* Flatten the @desc->tx_list chain into @edmac->active list */
	while (!list_empty(&desc->tx_list)) {
		struct ma35d1_dma_desc *d = list_first_entry(&desc->tx_list,
							      struct
							      ma35d1_dma_desc,
							      node);
		/*
		 * We copy the callback parameters from the first descriptor
		 * to all the chained descriptors. This way we can call the
		 * callback without having to find out the first descriptor in
		 * the chain. Useful for cyclic transfers.
		 */
		d->txd.callback = desc->txd.callback;
		d->txd.callback_param = desc->txd.callback_param;
		list_move_tail(&d->node, &edmac->active);
	}

	LEAVE();
}

/* Called with @edmac->lock held and interrupts disabled */
static struct ma35d1_dma_desc *ma35d1_dma_get_active(
	struct ma35d1_dma_chan *edmac)
{
	ENTRY();
	return list_first_entry_or_null(&edmac->active,
					struct ma35d1_dma_desc, node);
}

/**
 * ma35d1_dma_advance_active - advances to the next active descriptor
 * @edmac: channel
 *
 * Function advances active descriptor to the next in the
 * edmac->active and returns true if
 * we still have descriptors in the chain to process.
 * Otherwise returns %false.
 *
 * When the channel is in cyclic mode always returns %true.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static bool ma35d1_dma_advance_active(struct ma35d1_dma_chan *edmac)
{
	struct ma35d1_dma_desc *desc;

	ENTRY();
	list_rotate_left(&edmac->active);

	if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags))
		return true;

	desc = ma35d1_dma_get_active(edmac);

	if (!desc)
		return false;

	/*
	 * If txd.cookie is set it means that we are back in the first
	 * descriptor in the chain and hence done with it.
	 */
	DMA_DEBUG("cookie=%d\n", desc->txd.cookie);
	LEAVE();
	return !desc->txd.cookie;
}

void ma35d1_dma_set_timeout(struct ma35d1_dma_chan *edmac,
			     u32 prescaler, u32 counter)
{
	struct ma35d1_dma_desc *desc;
	u32 value;

	ENTRY();
	desc = ma35d1_dma_get_active(edmac);

	if (!desc) {
		dev_warn(chan2dev(edmac), "PDMA: empty descriptor list\n");
		return;
	}

	if (prescaler == 0 && counter == 0) {
		/* Disable time-out funciton */
		value = ma35d1_pdma_edmac_read(edmac,
			PDMA_TOUTIEN) & ~(1 << edmac->ch);
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTIEN, value);

		value = ma35d1_pdma_edmac_read(edmac,
			PDMA_TOUTEN) & ~(1 << edmac->ch);
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTEN, value);
		return;
	}

	if (edmac->ch <= 7) {
		value = ma35d1_pdma_edmac_read(edmac,
			PDMA_TOUTPSC) & ~(0x7 << (4 * edmac->ch));
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTPSC, value);

		value = ma35d1_pdma_edmac_read(edmac,
			PDMA_TOUTPSC) | ((prescaler & 0x7) << (4 * edmac->ch));
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTPSC, value);

	} else {
		value = ma35d1_pdma_edmac_read(edmac, PDMA_TOUTPSC1)
		    & ~(0x7 << (4 * (edmac->ch - 8)));
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTPSC1, value);

		value = ma35d1_pdma_edmac_read(edmac, PDMA_TOUTPSC1)
		    | ((prescaler & 0x7) << (4 * (edmac->ch - 8)));
		ma35d1_pdma_edmac_write(edmac, PDMA_TOUTPSC1, value);
	}

	value = ma35d1_pdma_edmac_read(edmac,
		(4 * (edmac->ch / 2))) &
		~(0xffff << (16 * (edmac->ch % 2)));
	ma35d1_pdma_edmac_write(edmac,
		PDMA_TOC + (4 * (edmac->ch / 2)), value);

	value = ma35d1_pdma_edmac_read(edmac,
		(4 * (edmac->ch / 2))) |
		((counter & 0xffff) << (16 * (edmac->ch % 2)));
	ma35d1_pdma_edmac_write(edmac,
		PDMA_TOC + (4 * (edmac->ch / 2)), value);

	/* Enable time-out funciton */
	value = ma35d1_pdma_edmac_read(edmac, PDMA_TOUTEN) | (1 << edmac->ch);
	ma35d1_pdma_edmac_write(edmac, PDMA_TOUTEN, value);

	/* Enable time-out interrupt */
	value = ma35d1_pdma_edmac_read(edmac, PDMA_TOUTIEN) | (1 << edmac->ch);
	ma35d1_pdma_edmac_write(edmac, PDMA_TOUTIEN, value);
	LEAVE();
}

static void hw_shutdown(struct ma35d1_dma_chan *edmac)
{
	u32 value;

	ENTRY();
	value = ma35d1_pdma_edmac_read(edmac, PDMA_CHCTL) & ~(1 << edmac->ch);
	ma35d1_pdma_edmac_write(edmac, PDMA_CHCTL, value);
	LEAVE();
}

static void fill_desc(struct ma35d1_dma_chan *edmac)
{
	struct ma35d1_dma_desc *desc;
	u32 reg, value;
	int width = 1;

	ENTRY();
	desc = ma35d1_dma_get_active(edmac);

	if (!desc) {
		dev_warn(chan2dev(edmac), "PDMA: empty descriptor list\n");
		return;
	}
	DMA_DEBUG("edmac->runtime_ctrl=0x%08x\n", edmac->runtime_ctrl);
	DMA_DEBUG("PDMA ch%02d CTL=0x%08x\n", edmac->ch,
		  ma35d1_pdma_edmac_read(edmac, edmac->ch * 16));
	DMA_DEBUG("desc->ctl=0x%08x\n", desc->ctl);

	if ((ma35d1_pdma_edmac_read(edmac, edmac->ch * 16) & 0x3) != 0) {
		reg = ma35d1_pdma_edmac_read(edmac, PDMA_CHCTL);
		ma35d1_pdma_edmac_write(edmac, edmac->ch * 16, 0);

		value = ma35d1_pdma_edmac_read(edmac,
						PDMA_CHRST) | 1 << (edmac->ch);
		ma35d1_pdma_edmac_write(edmac, PDMA_CHRST, value);
		ma35d1_pdma_edmac_write(edmac, PDMA_CHCTL,
					 reg | (1 << edmac->ch));

	} else {
		ma35d1_pdma_edmac_write(edmac, edmac->ch * 16, 0);

		value = ma35d1_pdma_edmac_read(edmac,
						PDMA_CHCTL) | (1 << edmac->ch);
		ma35d1_pdma_edmac_write(edmac, PDMA_CHCTL, value);
	}

	value = ma35d1_pdma_edmac_read(edmac, PDMA_INTEN) | (1 << edmac->ch);
	ma35d1_pdma_edmac_write(edmac, PDMA_INTEN, value);
	ma35d1_set_transfer_mode(edmac, desc->config.reqsel);

	if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags))
	{
		if(desc->dir == DMA_MEM_TO_DEV)
				width = edmac->slave_config.dst_addr_width;
		else
				width = edmac->slave_config.src_addr_width;
	}

#ifdef EN_HW_SG
	if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags))
	{
		struct ma35d1_sg *p_sg;
		int cnt=1;
		u32 start_addr,end_addr;

		if(desc->dir == DMA_MEM_TO_DEV)
			start_addr =  desc->src_addr;
		else
			start_addr =  desc->dst_addr;
		ma35d1_pdma_edmac_write(edmac, (edmac->ch * 16) + 0xc, (u32)edmac->sg_addr);
		p_sg=(struct ma35d1_sg *)edmac->sg_base;

		do{
			p_sg->ctl = (edmac->runtime_ctrl | (((desc->size/width) - 1UL) << PDMA_TXCNT_Pos) | PDMA_OP_SCATTER);
			p_sg->src = desc->src_addr;
			p_sg->dst = desc->dst_addr;
			p_sg->next = (u32)edmac->sg_addr+(cnt*16);
			cnt++;
			DMA_DEBUG2("ctl 0x%08x, src 0x%08x, dst 0x%08x, next 0x%08x\n",p_sg->ctl,p_sg->src,p_sg->dst,p_sg->next);

			ma35d1_dma_advance_active(edmac);
			desc = ma35d1_dma_get_active(edmac);

			if(desc->dir == DMA_MEM_TO_DEV)
				end_addr =  desc->src_addr;
			else
				end_addr =  desc->dst_addr;
			p_sg++;
		}while(start_addr!=end_addr);
		(--p_sg)->next=(u32)edmac->sg_addr;
		dma_sync_single_for_cpu(edmac->dev, (u32)edmac->sg_addr, SG_LEN, DMA_FROM_DEVICE);
	}else{
#endif
		value = ma35d1_pdma_edmac_read(edmac,
			(edmac->ch * 16)) |
			(edmac->runtime_ctrl | ((desc->size/width - 1UL) << PDMA_TXCNT_Pos));
		ma35d1_pdma_edmac_write(edmac, (edmac->ch * 16), value);
		ma35d1_pdma_edmac_write(edmac, (edmac->ch * 16) + 4, desc->src_addr);
		ma35d1_pdma_edmac_write(edmac, (edmac->ch * 16) + 8, desc->dst_addr);
#ifdef EN_HW_SG
	}
#endif

	DMA_DEBUG2("===============pdma=============\n");
	DMA_DEBUG2("pdma->DSCT[%d].CTL=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, edmac->ch * 16));
	DMA_DEBUG2("pdma->DSCT[%d].SA=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, (edmac->ch * 16) + 4));
	DMA_DEBUG2("pdma->DSCT[%d].DA=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, (edmac->ch * 16) + 8));
	DMA_DEBUG2("pdma->CHCTL=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_CHCTL));
	DMA_DEBUG2("pdma->INTEN=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_INTEN));
	DMA_DEBUG2("pdma->INTSTS=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_INTSTS));
	DMA_DEBUG2("pdma->TDSTS=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_TDSTS));
	DMA_DEBUG2("pdma->REQSEL=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_REQSEL));
	DMA_DEBUG2("===============================\n");
	LEAVE();
}

static void hw_submit(struct ma35d1_dma_chan *edmac)
{
	struct ma35d1_dma_desc *desc;

	ENTRY();
	fill_desc(edmac);
	desc = ma35d1_dma_get_active(edmac);
	ma35d1_dma_set_timeout(edmac,
	desc->config.timeout_prescaler,desc->config.timeout_counter);

#ifdef EN_HW_SG
	if (!test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags))
#endif
		ma35d1_pdma_edmac_write(edmac, edmac->ch * 16,
			(ma35d1_pdma_edmac_read(edmac,
			edmac->ch * 16) & ~PDMA_OP_Msk) | PDMA_OP_BASIC);
#ifdef EN_HW_SG
	else{
		ma35d1_pdma_edmac_write(edmac, edmac->ch * 16,
			(ma35d1_pdma_edmac_read(edmac,
			edmac->ch * 16) & ~PDMA_OP_Msk) | PDMA_OP_SCATTER);
	}
#endif

	if (desc->config.reqsel == 0)
		ma35d1_pdma_edmac_write(edmac, PDMA_SWREQ, 1 << edmac->ch);

	DMA_DEBUG2("===============pdma=============\n");
	DMA_DEBUG2("pdma->DSCT[%d].CTL=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, edmac->ch * 16));
	DMA_DEBUG2("pdma->DSCT[%d].SA=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, (edmac->ch * 16) + 4));
	DMA_DEBUG2("pdma->DSCT[%d].DA=0x%08x\n", edmac->ch,
		   ma35d1_pdma_edmac_read(edmac, (edmac->ch * 16) + 8));
	DMA_DEBUG2("pdma->CHCTL=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_CHCTL));
	DMA_DEBUG2("pdma->INTEN=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_INTEN));
	DMA_DEBUG2("pdma->INTSTS=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_INTSTS));
	DMA_DEBUG2("pdma->TDSTS=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_TDSTS));
	DMA_DEBUG2("pdma->REQSEL=0x%08x\n",
		   ma35d1_pdma_edmac_read(edmac, PDMA_REQSEL));
	DMA_DEBUG2("===============================\n");
	LEAVE();
}

static int hw_interrupt(struct ma35d1_dma_chan *edmac)
{
	ENTRY();
	if (ma35d1_dma_advance_active(edmac)) {
		struct ma35d1_dma_desc *desc;

		fill_desc(edmac);
		desc = ma35d1_dma_get_active(edmac);
		ma35d1_pdma_edmac_write(edmac, edmac->ch * 16,
			(ma35d1_pdma_edmac_read(edmac,
			edmac->ch * 16) & ~PDMA_OP_Msk) | PDMA_OP_BASIC);

		if (desc->config.reqsel == 0)
			ma35d1_pdma_edmac_write(edmac, PDMA_SWREQ,
						 1 << (edmac->ch));

		return INTERRUPT_NEXT_BUFFER;
	}

	LEAVE();
	return INTERRUPT_DONE;
}

/*
 * DMA engine API implementation
 */

static struct ma35d1_dma_desc *ma35d1_dma_desc_get(
	struct ma35d1_dma_chan *edmac)
{
	struct ma35d1_dma_desc *desc, *_desc;
	struct ma35d1_dma_desc *ret = NULL;
	unsigned long flags;

	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	list_for_each_entry_safe(desc, _desc, &edmac->free_list, node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del_init(&desc->node);
			/* Re-initialize the descriptor */
			desc->src_addr = 0;
			desc->dst_addr = 0;
			desc->size = 0;
			desc->complete = false;
			desc->txd.cookie = 0;
			desc->txd.callback = NULL;
			desc->txd.callback_param = NULL;
			ret = desc;
			break;
		}
	}
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
	return ret;
}

static void ma35d1_dma_desc_put(struct ma35d1_dma_chan *edmac,
				 struct ma35d1_dma_desc *desc)
{
	if (desc) {
		unsigned long flags;

		spin_lock_irqsave(&edmac->lock, flags);
		list_splice_init(&desc->tx_list, &edmac->free_list);
		list_add(&desc->node, &edmac->free_list);
		spin_unlock_irqrestore(&edmac->lock, flags);
	}
}

/**
 * ma35d1_dma_advance_work - start processing the next pending transaction
 * @edmac: channel
 *
 * If we have pending transactions queued and we are currently idling, this
 * function takes the next queued transaction from the @edmac->queue and
 * pushes it to the hardware for execution.
 */
static void ma35d1_dma_advance_work(struct ma35d1_dma_chan *edmac)
{
	struct ma35d1_dma_desc *new;
	unsigned long flags;

	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);

	if (!list_empty(&edmac->active) || list_empty(&edmac->queue)) {
		spin_unlock_irqrestore(&edmac->lock, flags);
		return;
	}

	/* Take the next descriptor from the pending queue */
	new = list_first_entry(&edmac->queue, struct ma35d1_dma_desc, node);
	list_del_init(&new->node);
	ma35d1_dma_set_active(edmac, new);
	DMA_DEBUG2("hw_submit(edmac)\n");
	/* Push it to the hardware */
	edmac->edma->hw_submit(edmac);
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
}

static void ma35d1_dma_tasklet(unsigned long data)
{
	struct ma35d1_dma_chan *edmac = (struct ma35d1_dma_chan *)data;
	struct ma35d1_dma_desc *desc, *d;
	struct dmaengine_desc_callback cb;
	LIST_HEAD(list);

	ENTRY();
	memset(&cb, 0, sizeof(cb));
	spin_lock_irq(&edmac->lock);
	/*
	 * If dma_terminate_all() was called before we get to run, the active
	 * list has become empty. If that happens we aren't supposed to do
	 * anything more than call ma35d1_dma_advance_work().
	 */
	desc = ma35d1_dma_get_active(edmac);

	if (desc) {
		if (desc->complete) {
			DMA_DEBUG("desc->complete ==> OK\n");

			/* mark descriptor complete for non cyclic case only */
			if (!test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags)) {
				DMA_DEBUG("if (!test_bit(MA35D1_\n");
				dma_cookie_complete(&desc->txd);
			}

			list_splice_init(&edmac->active, &list);
		}

		DMA_DEBUG("dmaengine_desc_get_callback\n");
		dmaengine_desc_get_callback(&desc->txd, &cb);
	}

	spin_unlock_irq(&edmac->lock);
	/* Pick up the next descriptor from the queue */
	ma35d1_dma_advance_work(edmac);
	/* Now we can release all the chained descriptors */
	list_for_each_entry_safe(desc, d, &list, node) {
		dma_descriptor_unmap(&desc->txd);
		ma35d1_dma_desc_put(edmac, desc);
	}
	dmaengine_desc_callback_invoke(&cb, NULL);
	LEAVE();
}

void ma35d1_dma_edmac_interrupt(struct ma35d1_dma_chan *edmac,
	int status)
{
	struct ma35d1_dma_desc *desc = NULL;
	struct ma35d1_dma_done *done = NULL;

	ENTRY();
	desc = ma35d1_dma_get_active(edmac);

	if (!desc) {
		dev_warn(chan2dev(edmac),
			 "got interrupt while active list is empty\n");
		LEAVE();
		return;
	}

	if (status == INTERRUPT_TIMEOUT) {
		done = (struct ma35d1_dma_done *)desc->txd.callback_param;

		if (done != NULL) {
			done->done = 0;
			done->timeout = 1;
			done->remain = (ma35d1_pdma_edmac_read(edmac,
				edmac->ch * 16) &
				PDMA_TXCNT_Msk) >> PDMA_TXCNT_Pos;
		}

		tasklet_schedule(&edmac->tasklet);
		return;
	}

#ifdef EN_HW_SG
	if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags)){
			struct dmaengine_desc_callback cb;
			memset(&cb, 0, sizeof(cb));
			dmaengine_desc_get_callback(&desc->txd, &cb);
			dmaengine_desc_callback_invoke(&cb, NULL);
			ma35d1_dma_advance_active(edmac);
			return;
	}
#endif

	switch (edmac->edma->hw_interrupt(edmac)) {
	case INTERRUPT_DONE:
		DMA_DEBUG2("INTERRUPT_DONE\n");
		desc->complete = true;
		done = (struct ma35d1_dma_done *)desc->txd.callback_param;

		if (done != NULL) {
			done->done = 1;
			done->timeout = 0;
			done->remain = 0;
		}

		tasklet_schedule(&edmac->tasklet);
		break;

	case INTERRUPT_NEXT_BUFFER:
		DMA_DEBUG2("INTERRUPT_NEXT_BUFFER\n");

		if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags))
			tasklet_schedule(&edmac->tasklet);

		break;

	default:
		dev_warn(chan2dev(edmac), "unknown interrupt!\n");
		break;
	}

	LEAVE();
}

static irqreturn_t ma35d1_dma_interrupt(int irq, void *dev_id)
{
	int i;
	u32 value;
	struct ma35d1_dma_engine *edma = dev_id;
	unsigned int pdma_int_status =
	    ma35d1_pdma_edmac_read(&edma->channels[0],
				    PDMA_INTSTS);
	unsigned int pdma_status = ma35d1_pdma_edmac_read(&edma->channels[0],
							   PDMA_TDSTS);
	irqreturn_t ret = IRQ_HANDLED;

	ENTRY();
	DMA_DEBUG2("irqreturn_t\n");
	DMA_DEBUG2("PDMA->INTSTS=0x%08x,PDMA->TDSTS=0x%08x\n",
		   pdma_int_status, pdma_status);

	for (i = (edma->num_channels - 1); i >= 0; i--) {
		if (pdma_status & (1 << (edma->channels[i].ch))) {
			ma35d1_pdma_edmac_write(&edma->channels[0], PDMA_TDSTS,
						 (1 << (edma->channels[i].ch)));
			ma35d1_dma_edmac_interrupt(&edma->channels[i],
						    INTERRUPT_DONE);
		}

		if (pdma_int_status & (1 << (edma->channels[i].ch + 8))) {
			DMA_DEBUG2("PDMA INTERRUPT_TIMEOUT id=%d",
				   edma->channels[i].ch);
			ma35d1_dma_edmac_interrupt(&edma->channels[i],
						    INTERRUPT_TIMEOUT);
			value = (ma35d1_pdma_edmac_read
				 (&edma->channels[i],
				  PDMA_TOUTEN) & ~(1 << (edma->channels
							 [i].ch)));
			ma35d1_pdma_edmac_write(&edma->channels[i],
						 PDMA_TOUTEN, value);
			ma35d1_pdma_edmac_write(&edma->channels[i],
						 PDMA_INTSTS,
						 (1 <<
						  (edma->channels[i].ch + 8)));
		}
	}

	LEAVE();
	return ret;
}

/**
 * ma35d1_dma_tx_submit - set the prepared descriptor(s) to be executed
 * @tx: descriptor to be executed
 *
 * Function will execute given descriptor on the hardware or
 * if the hardware is busy, queue the descriptor to be executed later on.
 * Returns cookie which can be used to poll the status of the descriptor.
 */
static dma_cookie_t ma35d1_dma_tx_submit(
	struct dma_async_tx_descriptor *tx)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(tx->chan);
	struct ma35d1_dma_desc *desc;
	dma_cookie_t cookie;
	unsigned long flags;

	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	cookie = dma_cookie_assign(tx);
	DMA_DEBUG("cookie=%d\n", cookie);
	desc = container_of(tx, struct ma35d1_dma_desc, txd);

	/*
	 * If nothing is currently prosessed, we push this descriptor
	 * directly to the hardware. Otherwise we put the descriptor
	 * to the pending queue.
	 */
	if (list_empty(&edmac->active)) {
		ma35d1_dma_set_active(edmac, desc);
		edmac->edma->hw_submit(edmac);

	} else {
		list_add_tail(&desc->node, &edmac->queue);
	}

	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
	return cookie;
}

static int ma35d1_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	int i;

	ENTRY();
	spin_lock_irq(&edmac->lock);
	dma_cookie_init(&edmac->chan);
	spin_unlock_irq(&edmac->lock);

	for (i = 0; i < DMA_MAX_CHAN_DESCRIPTORS; i++) {
		struct ma35d1_dma_desc *desc;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc)
			break;
		INIT_LIST_HEAD(&desc->tx_list);
		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.tx_submit = ma35d1_dma_tx_submit;
		ma35d1_dma_desc_put(edmac, desc);
	}

	DMA_DEBUG("return %d\n", i);
	LEAVE();
	return i;
}

static void ma35d1_dma_free_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	struct ma35d1_dma_desc *desc, *d;
	unsigned long flags;
	LIST_HEAD(list);

	ENTRY();
	BUG_ON(!list_empty(&edmac->queue));
	spin_lock_irqsave(&edmac->lock, flags);
	edmac->edma->hw_shutdown(edmac);
	edmac->runtime_addr = 0;
	edmac->runtime_ctrl = 0;
	list_splice_init(&edmac->free_list, &list);
	spin_unlock_irqrestore(&edmac->lock, flags);
	list_for_each_entry_safe(desc, d, &list, node)
		kfree(desc);
	LEAVE();
}

static struct dma_async_tx_descriptor *ma35d1_dma_prep_dma_memcpy(
	struct dma_chan *chan,
	dma_addr_t dest,
	dma_addr_t src,
	size_t len,
	unsigned long flags)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	struct ma35d1_dma_desc *desc, *first;
	size_t bytes, offset;

	ENTRY();
	first = NULL;
	for (offset = 0; offset < len; offset += bytes) {
		desc = ma35d1_dma_desc_get(edmac);

		if (!desc) {
			dev_warn(chan2dev(edmac), "couldn't get descriptor\n");
			goto fail;
		}

		bytes = min_t(size_t, len - offset, DMA_MAX_CHAN_BYTES);
		desc->src_addr = src + offset;
		desc->dst_addr = dest + offset;
		desc->size = bytes;
		desc->config.reqsel = 0;
		desc->config.en_sc = 0;
		desc->dir = DMA_MEM_TO_MEM;
		edmac->runtime_ctrl = 0;

		if (!first)
			first = desc;

		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();
	return &first->txd;
fail:
	ma35d1_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

static struct dma_async_tx_descriptor *ma35d1_dma_prep_slave_sg(
	struct dma_chan *chan,
	struct scatterlist *sgl,
	unsigned int sg_len,
	enum dma_transfer_direction dir,
	unsigned long flags,
	void *context)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	struct ma35d1_dma_desc *desc, *first;
	struct scatterlist *sg;
	struct ma35d1_dma_data *data;
	int i;

	ENTRY();
	{
		struct ma35d1_dma_desc *d;
		LIST_HEAD(list);

		list_splice_init(&edmac->active, &list);
		/* Now we can release all the chained descriptors */
		list_for_each_entry_safe(desc, d, &list, node) {
			desc->txd.flags = DMA_CTRL_ACK;
			ma35d1_dma_desc_put(edmac, desc);
		}
	}

	data =(struct ma35d1_dma_data *)chan->private;
	if (test_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	ma35d1_dma_slave_config_write(chan, dir, &edmac->slave_config);
	first = NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		size_t len = sg_dma_len(sg);

		if (len > DMA_MAX_CHAN_BYTES) {
			dev_warn(chan2dev(edmac), "too big transfer size %zu\n",
				 len);
			goto fail;
		}

		desc = ma35d1_dma_desc_get(edmac);

		if (!desc) {
			dev_warn(chan2dev(edmac), "couldn't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = sg_dma_address(sg);
			desc->dst_addr = edmac->runtime_addr;
			desc->dir = DMA_MEM_TO_DEV;

		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = sg_dma_address(sg);
			desc->dir = DMA_DEV_TO_MEM;
		}

		desc->size = len;
		desc->config.reqsel = edmac->slave_config.slave_id;
		if(data!=NULL) {
			desc->config.timeout_counter=data->timeout_counter;
			desc->config.timeout_prescaler=data->timeout_prescaler;
		}else {
			desc->config.timeout_counter = 0;
			desc->config.timeout_prescaler = 0;
		}

		if (!first)
			first = desc;

		else
			list_add_tail(&desc->node, &first->tx_list);
	}
	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();
	return &first->txd;
fail:
	ma35d1_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

static struct dma_async_tx_descriptor *ma35d1_dma_prep_dma_cyclic(
	struct dma_chan *chan,
	dma_addr_t dma_addr,
	size_t buf_len,
	size_t period_len,
	enum dma_transfer_direction dir,
	unsigned long flags)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	struct ma35d1_dma_desc *desc, *first;
	size_t offset = 0;
	struct ma35d1_dma_data *data;

	ENTRY();
	if (test_and_set_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
			 "channel is already used for cyclic transfers\n");
		return NULL;
	}

	if (period_len > DMA_MAX_CHAN_BYTES) {
		dev_warn(chan2dev(edmac), "too big period length %zu\n",
			 period_len);
		return NULL;
	}

	ma35d1_dma_slave_config_write(chan, dir, &edmac->slave_config);
	/* Split the buffer into period size chunks */
	first = NULL;

	for (offset = 0; offset < buf_len; offset += period_len) {
		desc = ma35d1_dma_desc_get(edmac);

		if (!desc) {
			dev_warn(chan2dev(edmac), "couldn't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = dma_addr + offset;
			desc->dst_addr = edmac->runtime_addr;
			desc->dir = DMA_MEM_TO_DEV;

		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = dma_addr + offset;
			desc->dir = DMA_DEV_TO_MEM;
		}

		data =(struct ma35d1_dma_data *)chan->private;
		desc->size = period_len;

		desc->config.reqsel = edmac->slave_config.slave_id;
		if(data!=NULL) {
			desc->config.timeout_counter=data->timeout_counter;
			desc->config.timeout_prescaler=data->timeout_prescaler;
		}else {
			desc->config.timeout_counter = 0;
			desc->config.timeout_prescaler = 0;
		}

		if (!first)
			first = desc;

		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	LEAVE();
	return &first->txd;
fail:
	ma35d1_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

static void ma35d1_dma_synchronize(struct dma_chan *chan)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);

	ENTRY();
	if (edmac->edma->hw_synchronize)
		edmac->edma->hw_synchronize(edmac);

	LEAVE();
}

static int ma35d1_dma_terminate_all(struct dma_chan *chan)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	struct ma35d1_dma_desc *desc, *_d;
	unsigned long flags;
	LIST_HEAD(list);

	ENTRY();
	spin_lock_irqsave(&edmac->lock, flags);
	/* First we disable and flush the DMA channel */
	edmac->edma->hw_shutdown(edmac);
	clear_bit(MA35D1_DMA_IS_CYCLIC, &edmac->flags);
	list_splice_init(&edmac->active, &list);
	list_splice_init(&edmac->queue, &list);
	spin_unlock_irqrestore(&edmac->lock, flags);

	list_for_each_entry_safe(desc, _d, &list, node)
		ma35d1_dma_desc_put(edmac, desc);

	return 0;
}

static int ma35d1_dma_slave_config(struct dma_chan *chan,
				    struct dma_slave_config *config)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);

	ENTRY();
	memcpy(&edmac->slave_config, config, sizeof(*config));
	LEAVE();
	return 0;
}

static int ma35d1_dma_slave_config_write(struct dma_chan *chan,
					  enum dma_transfer_direction dir,
					  struct dma_slave_config *config)
{
	struct ma35d1_dma_chan *edmac = to_ma35d1_dma_chan(chan);
	enum dma_slave_buswidth width;
	unsigned long flags;
	u32 addr, ctrl;

	ENTRY();
	switch (dir) {
	case DMA_DEV_TO_MEM:
		ctrl = PDMA_SAINC_Msk | PDMA_TXTYPE_Msk;
		width = config->src_addr_width;
		addr = config->src_addr;
		break;

	case DMA_MEM_TO_DEV:
		ctrl = PDMA_DAINC_Msk | PDMA_TXTYPE_Msk;
		width = config->dst_addr_width;
		addr = config->dst_addr;
		break;

	default:
		return -EINVAL;
	}

	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl |= 0 << PDMA_TXWIDTH_Pos;
		break;

	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl |= 1 << PDMA_TXWIDTH_Pos;
		break;

	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl |= 2 << PDMA_TXWIDTH_Pos;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&edmac->lock, flags);
	edmac->runtime_addr = addr;
	edmac->runtime_ctrl = ctrl;
	spin_unlock_irqrestore(&edmac->lock, flags);
	LEAVE();
	return 0;
}

/**
 * ma35d1_dma_tx_status - check if a transaction is completed
 * @chan: channel
 * @cookie: transaction specific cookie
 * @state: state of the transaction is stored here if given
 *
 * This function can be used to query state of a given transaction.
 */
static enum dma_status ma35d1_dma_tx_status(struct dma_chan *chan,
					     dma_cookie_t cookie,
					     struct dma_tx_state *state)
{
	ENTRY();
	DMA_DEBUG("cookie=%d\n", cookie);
	LEAVE();
	return dma_cookie_status(chan, cookie, state);
}

/**
 * ma35d1_dma_issue_pending - push pending transactions to the hardware
 * @chan: channel
 *
 * When this function is called, all pending transactions are pushed to the
 * hardware and executed.
 */
static void ma35d1_dma_issue_pending(struct dma_chan *chan)
{
	ENTRY();
	ma35d1_dma_advance_work(to_ma35d1_dma_chan(chan));
	LEAVE();
}

static int ma35d1_dma_probe(struct platform_device *pdev)
{
	struct ma35d1_dma_platform_data *pdata;
	struct ma35d1_dma_engine *edma;
	struct dma_device *dma_dev;
	size_t edma_size;
	struct resource *res;
	int ret, i;
	struct clk *pdma_clk;
	int err;

	pdma_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(pdma_clk)) {
		err = PTR_ERR(pdma_clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		return -ENOENT;
	}
	err = clk_prepare_enable(pdma_clk);
	if (err)
		return -ENOENT;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

	if (ret)
		return ret;

	/* Allocate a new DMAC and its Channels */
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(pdata->base))
		return PTR_ERR(pdata->base);

	pdata->irq = platform_get_irq(pdev, 0);

	if (pdata->irq < 0)
		return pdata->irq;

	pdata->num_channels = DMA_CHANNELS;
	pdata->channels = devm_kcalloc(&pdev->dev,
		pdata->num_channels, sizeof(struct ma35d1_dma_chan_data),
		GFP_KERNEL);
	edma_size = pdata->num_channels * sizeof(struct ma35d1_dma_chan);
	edma = devm_kzalloc(&pdev->dev, sizeof(*edma) + edma_size, GFP_KERNEL);

	if (!edma) {
		DMA_DEBUG("MA35D1 GDMA -ENOMEM\n");
		return -ENOMEM;
	}

	dma_dev = &edma->dma_dev;
	edma->num_channels = pdata->num_channels;
	INIT_LIST_HEAD(&dma_dev->channels);

	for (i = 0; i < pdata->num_channels; i++) {
		struct ma35d1_dma_chan *edmac = &edma->channels[i];

		edmac->dev = &pdev->dev;
		edmac->ch = i;
		edmac->chan.device = dma_dev;
		edmac->regs = pdata->base;
		edmac->irq = pdata->irq;
		edmac->edma = edma;
		#ifdef EN_HW_SG
		edmac->sg_addr = (u32)dma_map_single(edmac->dev,(void *)(edmac->sg_base),SG_LEN, DMA_BIDIRECTIONAL);
		#endif
		spin_lock_init(&edmac->lock);
		INIT_LIST_HEAD(&edmac->active);
		INIT_LIST_HEAD(&edmac->queue);
		INIT_LIST_HEAD(&edmac->free_list);
		tasklet_init(&edmac->tasklet, ma35d1_dma_tasklet,
			     (unsigned long)edmac);
		list_add_tail(&edmac->chan.device_node, &dma_dev->channels);
	}

	ret = devm_request_irq(&pdev->dev, pdata->irq,
			       ma35d1_dma_interrupt, 0, pdev->name, edma);

	if (ret) {
		dev_warn(&pdev->dev, "Can't register IRQ for DMA\n");
		return ret;
	}

	dma_cap_zero(dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_dev->directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
                BIT(DMA_MEM_TO_MEM);
	dma_dev->dev = &pdev->dev;
	dma_dev->device_alloc_chan_resources = ma35d1_dma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = ma35d1_dma_free_chan_resources;
	dma_dev->device_prep_slave_sg = ma35d1_dma_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = ma35d1_dma_prep_dma_cyclic;
	dma_dev->device_config = ma35d1_dma_slave_config;
	dma_dev->device_synchronize = ma35d1_dma_synchronize;
	dma_dev->device_terminate_all = ma35d1_dma_terminate_all;
	dma_dev->device_issue_pending = ma35d1_dma_issue_pending;
	dma_dev->device_tx_status = ma35d1_dma_tx_status;
	dma_set_max_seg_size(dma_dev->dev, DMA_MAX_CHAN_BYTES);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_dev->device_prep_dma_memcpy = ma35d1_dma_prep_dma_memcpy;
	edma->hw_shutdown = hw_shutdown;
	edma->hw_submit = hw_submit;
	edma->hw_interrupt = hw_interrupt;
	edma->pdata = pdata;
	dma_cap_set(DMA_PRIVATE, dma_dev->cap_mask);
	platform_set_drvdata(pdev, edma);
	ret = dma_async_device_register(dma_dev);

	if (unlikely(ret))
		dev_info(dma_dev->dev, "ma35d1 DMA not ready\n");
	else
		dev_info(dma_dev->dev, "ma35d1 DMA ready\n");

	return ret;
}

static int ma35d1_dma_remove(struct platform_device *pdev)
{
	struct ma35d1_dma_engine *edma = platform_get_drvdata(pdev);
	struct dma_device *dma_dev;

	dma_dev = &edma->dma_dev;
	dma_async_device_unregister(dma_dev);
	return 0;
}

static const struct of_device_id ma35d1_dma_of_match[] = {
	{.compatible = "nuvoton,ma35d1-pdma"},
	{},
};

static struct platform_driver ma35d1_dma_driver = {
	.probe = ma35d1_dma_probe,
	.remove = ma35d1_dma_remove,
	.driver = {
		   .name = "ma35d1-dma",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ma35d1_dma_of_match),
		   },
};

module_platform_driver(ma35d1_dma_driver);

MODULE_DESCRIPTION("ma35d1 DMA driver");
MODULE_LICENSE("GPL v2");
