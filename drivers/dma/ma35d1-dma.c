// SPDX-License-Identifier: GPL-2.0+
/*
 * This file contains a driver for the Nuvoton MA35D1 DMA engine
 * found on MA35D1
 *
 * Copyright (C) 2022 Nuvoton Technology Corp.
 */

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/platform_data/dma-ma35d1.h>

#include "dmaengine.h"
#include "virt-dma.h"

/* PDMA registers */
#define PDMA_OFFSET_CHAN_SIZE		0x10
#define PDMA_DSCT_CTL			0x0
#define   PDMA_OP_STOP			0x0
#define   PDMA_OP_BASIC		0x1
#define   PDMA_OP_SCATTER		0x2
#define   PDMA_OP_MSK			0x3
#define   PDMA_TXTYPE			(1<<2)
#define   PDMA_TBINTDIS		(1<<7)
#define   PDMA_SAFIX			(3<<8)
#define   PDMA_DAFIX			(3<<10)
#define   PDMA_TXWIDTH_1_BYTE		(0<<12)
#define   PDMA_TXWIDTH_2_BYTES	(1<<12)
#define   PDMA_TXWIDTH_4_BYTES	(2<<12)
#define   PDMA_TXCNT(cnt)		((cnt)<<16)
#define   PDMA_GET_TXCNT(ctrl)	((ctrl>>16)&0xffff)
#define PDMA_DSCT_SA			0x004
#define PDMA_DSCT_DA			0x008
#define PDMA_DSCT_NEXT			0x00c
#define PDMA_CHCTL			0x400
#define PDMA_PAUSE			0x404
#define PDMA_SWREQ			0x408
#define PDMA_INTEN			0x418
#define PDMA_INTSTS			0x41C
#define PDMA_TDSTS			0x424
#define PDMA_TOUTEN			0x434
#define PDMA_TOUTIEN			0x438
#define PDMA_TOC			0x440
#define PDMA_CHRST			0x460
#define PDMA_TOUTPSC			0x470
#define PDMA_TOUTPSC1			0x474
#define PDMA_REQSEL			0x480

#define PDMA_MAX_CHANS			10
#define PDMA_MAX_CHAN_BYTES		0x10000

struct ma35d1_sg {
	u32 ctl;
	u32 src;
	u32 dst;
	u32 next;
};

struct ma35d1_desc {
	enum dma_transfer_direction dma_dir;
	bool cyclic;
	unsigned int sglen;
	struct virt_dma_desc vd;
	uint32_t ctl;
	struct ma35d1_peripheral pcfg;
	unsigned int sg_addr;
	struct ma35d1_sg sg[];
};

struct ma35d1_chan {
	struct device *dev;
	struct ma35d1_dmadev *edma;
	struct virt_dma_chan vc;
	void __iomem *base;
	struct ma35d1_desc *desc;
	struct dma_slave_config cfg;
	bool allocated;
	bool error;
	int ch_num;
	unsigned int remain;
	u32 runtime_addr;
	u32 runtime_ctrl;
	u32 runtime_width;
};

struct ma35d1_dmadev {
	struct dma_device ddev;
	struct ma35d1_chan channels[PDMA_MAX_CHANS];
	unsigned int irq;
	int nr_chans;

        /* To protect channel manipulation */
        spinlock_t lock;
};

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static inline struct ma35d1_chan *to_ma35d1_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct ma35d1_chan, vc.chan);
}

static inline struct ma35d1_desc *to_ma35d1_dma_desc(struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct ma35d1_desc, vd.tx);
}

static void ma35d1_dma_desc_free(struct virt_dma_desc *vd)
{
	kfree(container_of(vd, struct ma35d1_desc, vd));
}

static int ma35d1_terminate_all(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);
	u32 val;

	dev_dbg(chan2dev(chan), "%s: ch=%p\n", __func__, ch);

	spin_lock_irqsave(&ch->vc.lock, flags);

	if (ch->desc) {
		ma35d1_dma_desc_free(&ch->desc->vd);
		ch->desc = NULL;
	}

	spin_lock_irqsave(&ch->edma->lock,flags);
	val = readl(ch->base + PDMA_CHCTL) & ~(1 << ch->ch_num);
	writel(val, ch->base + PDMA_CHCTL);
	spin_unlock_irqrestore(&ch->edma->lock,flags);

	vchan_get_all_descriptors(&ch->vc, &head);
	vchan_dma_desc_free_list(&ch->vc, &head);
	spin_unlock_irqrestore(&ch->vc.lock, flags);

	return 0;
}

static void ma35d1_set_dma_timeout(struct ma35d1_chan *ch)
{
	struct ma35d1_desc *d = ch->desc;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&ch->edma->lock,flags);
	if (d->pcfg.timeout_prescaler == 0 && d->pcfg.timeout_counter == 0) {
		/* Disable time-out funciton */
		val = readl(ch->base + PDMA_TOUTIEN);
		val &= ~(1 << ch->ch_num);
		writel(val, ch->base + PDMA_TOUTIEN);

		val = readl(ch->base + PDMA_TOUTEN);
		val &= ~(1 << ch->ch_num);
		writel(val, ch->base + PDMA_TOUTEN);
		spin_unlock_irqrestore(&ch->edma->lock,flags);
		return;
	}

	if (ch->ch_num <= 7) {
		val = readl(ch->base + PDMA_TOUTPSC);
		val &= ~(0x7 << (4 * ch->ch_num));
		writel(val, ch->base + PDMA_TOUTPSC);

		val = readl(ch->base + PDMA_TOUTPSC);
		val |= ((d->pcfg.timeout_prescaler & 0x7) << (4 * ch->ch_num));
		writel(val, ch->base + PDMA_TOUTPSC);

	} else {
		val = readl(ch->base + PDMA_TOUTPSC1);
		val &= ~(0x7 << (4 * (ch->ch_num - 8)));
		writel(val, ch->base + PDMA_TOUTPSC1);

		val = readl(ch->base + PDMA_TOUTPSC1);
		val |=
		    ((d->pcfg.timeout_prescaler & 0x7) << (4 * (ch->ch_num - 8)));
		writel(val, ch->base + PDMA_TOUTPSC1);
	}

	val = readl(ch->base + PDMA_TOC + (4 * (ch->ch_num / 2)));
	val &= ~(0xffff << (16 * (ch->ch_num % 2)));
	writel(val, ch->base + PDMA_TOC + (4 * (ch->ch_num / 2)));

	val = readl(ch->base + PDMA_TOC + (4 * (ch->ch_num / 2)));
	val |=
	    ((d->pcfg.timeout_counter & 0xffff) << (16 * (ch->ch_num % 2)));
	writel(val, ch->base + PDMA_TOC + (4 * (ch->ch_num / 2)));

	/* Enable time-out funciton */
	val = readl(ch->base + PDMA_TOUTEN);
	val |= (1 << ch->ch_num);
	writel(val, ch->base + PDMA_TOUTEN);

	/* Enable time-out interrupt */
	val = readl(ch->base + PDMA_TOUTIEN);
	val |= (1 << ch->ch_num);
	writel(val, ch->base + PDMA_TOUTIEN);
	spin_unlock_irqrestore(&ch->edma->lock,flags);
}

static void ma35d1_set_transfer_params(struct ma35d1_chan *ch, int reqsel)
{
	u32 sel, reg;
	unsigned long flags;

	spin_lock_irqsave(&ch->edma->lock,flags);
	reg = PDMA_REQSEL + (4 * (ch->ch_num / 4));
	sel = readl(ch->base + reg);
	sel &= ~(0xFF << ((ch->ch_num & 0x3) * 8));
	sel |= (reqsel << ((ch->ch_num & 0x3) * 8));
	writel(sel, ch->base + reg);
	spin_unlock_irqrestore(&ch->edma->lock,flags);
}

static void ma35d1_set_channel_params(struct ma35d1_chan *ch)
{
	u32 reg, val;
	unsigned long flags;

	spin_lock_irqsave(&ch->edma->lock,flags);
	if ((readl(ch->base + ch->ch_num * 16) & 0x3) != 0) {
		reg = readl(ch->base + PDMA_CHCTL);
		writel(0, ch->base + ch->ch_num * 16);
		val = readl(ch->base + PDMA_CHRST) | 1 << (ch->ch_num);
		writel(val, ch->base + PDMA_CHRST);
		writel(reg | (1 << ch->ch_num), ch->base + PDMA_CHCTL);

	} else {
		writel(0, ch->base + ch->ch_num * 16);
		val = readl(ch->base + PDMA_CHCTL) | (1 << ch->ch_num);
		writel(val, ch->base + PDMA_CHCTL);
	}
	val = readl(ch->base + PDMA_INTEN) | (1 << ch->ch_num);
	writel(val, ch->base + PDMA_INTEN);
	spin_unlock_irqrestore(&ch->edma->lock,flags);
}

static int ma35d1_slave_config(struct dma_chan *chan,
			       struct dma_slave_config *cfg)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	enum dma_slave_buswidth width;
	u32 addr, ctrl;

	ch->cfg = *cfg;
	if (ch->cfg.direction == DMA_MEM_TO_DEV) {
		ctrl = PDMA_DAFIX | PDMA_TXTYPE;
		width = ch->cfg.dst_addr_width;
		addr = ch->cfg.dst_addr;
	} else {
		ctrl = PDMA_SAFIX | PDMA_TXTYPE;
		width = ch->cfg.src_addr_width;
		addr = ch->cfg.src_addr;
	}

	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl |= PDMA_TXWIDTH_1_BYTE;
		ch->runtime_width = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl |= PDMA_TXWIDTH_2_BYTES;
		ch->runtime_width = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl |= PDMA_TXWIDTH_4_BYTES;
		ch->runtime_width = 4;
		break;
	default:
		return -EINVAL;
	}
	ch->runtime_addr = addr;
	ch->runtime_ctrl = ctrl;
	return 0;
}

static struct dma_async_tx_descriptor *ma35d1_prep_dma_memcpy(
	struct dma_chan *chan,
	dma_addr_t dst,
	dma_addr_t src,
	size_t len,
	unsigned long tx_flags)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct ma35d1_desc *d;
	unsigned int sg_len;
	size_t bytes, tlen;
	int i;

	sg_len = 1 + len / PDMA_MAX_CHAN_BYTES;
	d = kzalloc(struct_size(d, sg, max_t(size_t, sg_len, 4)), GFP_ATOMIC);
	if (!d)
		return NULL;

	d->pcfg.reqsel = 0;
	ma35d1_set_channel_params(ch);
	ma35d1_set_transfer_params(ch, d->pcfg.reqsel);
	d->sg_addr =
	    dma_map_single(ch->dev, d->sg, sizeof(struct ma35d1_sg) * sg_len,
			   DMA_TO_DEVICE);
	writel(d->sg_addr,
	       ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) +
	       PDMA_DSCT_NEXT);

	for (i = 0; i < sg_len; i++) {
		bytes = min_t(size_t, len, PDMA_MAX_CHAN_BYTES);
		d->sg[i].src = src;
		d->sg[i].dst = dst;
		tlen = rounddown(len, bytes);
		d->sg[i].ctl = PDMA_TXCNT(tlen - 1UL) | PDMA_OP_SCATTER;
		src += tlen;
		dst += tlen;
		len -= tlen;
		d->sg[i].next = d->sg_addr + (16 * (i + 1) + 4);
	}
	dma_sync_single_for_cpu(ch->dev, d->sg_addr, sizeof(*d->sg),
				DMA_FROM_DEVICE);
	d->sglen = sg_len;
	d->sg[d->sglen - 1].ctl &= ~(PDMA_OP_MSK | PDMA_TBINTDIS);
	d->sg[d->sglen - 1].ctl |= PDMA_OP_BASIC;
	d->dma_dir = DMA_MEM_TO_MEM;
	d->cyclic = false;

	return vchan_tx_prep(&ch->vc, &d->vd, tx_flags);
}

static struct dma_async_tx_descriptor *ma35d1_prep_slave_sg(
	struct dma_chan *chan,
	struct scatterlist *sgl,
	unsigned int sg_len,
	enum dma_transfer_direction dir,
	unsigned long tx_flags,
	void *context)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct ma35d1_desc *d;
	struct scatterlist *sgent;
	unsigned int i;

	if (!is_slave_direction(dir)) {
		dev_err(chan2dev(chan), "%s: invalid DMA direction\n",
			__func__);
		return NULL;
	}

	d = kzalloc(struct_size(d, sg, sg_len), GFP_ATOMIC);
	if (!d)
		return NULL;

	if (ch->cfg.peripheral_size != sizeof(struct ma35d1_peripheral)) {
		dev_err(chan2dev(chan),
			"Invalid peripheral size %zu, expected %zu\n",
			ch->cfg.peripheral_size,
			sizeof(struct ma35d1_peripheral));
		return NULL;
	}

	memcpy(&d->pcfg, ch->cfg.peripheral_config, ch->cfg.peripheral_size);
	ma35d1_set_channel_params(ch);
	ma35d1_set_transfer_params(ch, d->pcfg.reqsel);
	d->sg_addr = (u32) dma_map_single(ch->dev,
					  (void *)(d->sg),
					  sizeof(struct ma35d1_sg) * sg_len,
					  DMA_BIDIRECTIONAL);
	writel(d->sg_addr,
	       ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) +
	       PDMA_DSCT_NEXT);
	for_each_sg(sgl, sgent, sg_len, i) {
		d->sg[i].ctl =
		    (ch->runtime_ctrl |
		     PDMA_TXCNT(sg_dma_len(sgent) -
				1UL) | PDMA_TBINTDIS | PDMA_OP_SCATTER);
		if (dir == DMA_MEM_TO_DEV) {
			d->sg[i].src = sg_dma_address(sgent);
			d->sg[i].dst = ch->runtime_addr;
			d->dma_dir = DMA_MEM_TO_DEV;

		} else {
			d->sg[i].src = ch->runtime_addr;
			d->sg[i].dst = sg_dma_address(sgent);
			d->dma_dir = DMA_DEV_TO_MEM;
		}
		d->sg[i].next = d->sg_addr + (PDMA_OFFSET_CHAN_SIZE * (i + 1));
	}
	d->sglen = sg_len;
	d->sg[d->sglen - 1].ctl &= ~(PDMA_OP_MSK | PDMA_TBINTDIS);
	d->sg[d->sglen - 1].ctl |= PDMA_OP_BASIC;
	dma_sync_single_for_cpu(ch->dev, d->sg_addr, sizeof(*d->sg) * d->sglen,
				DMA_FROM_DEVICE);
	d->cyclic = false;
	ch->error = 0;

	return vchan_tx_prep(&ch->vc, &d->vd, tx_flags);
}

static struct dma_async_tx_descriptor *ma35d1_prep_dma_cyclic(
	struct dma_chan *chan,
	dma_addr_t dma_addr,
	size_t buf_len,
	size_t period_len,
	enum dma_transfer_direction dir,
	unsigned long flags)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct ma35d1_desc *d;
	unsigned int i, sg_len;
	size_t offset = 0;

	if (period_len > PDMA_MAX_CHAN_BYTES) {
		dev_warn(chan2dev(chan), "too big period length %zu\n",
			 period_len);
		return NULL;
	}

	sg_len = buf_len / period_len;
	d = kzalloc(struct_size(d, sg, sg_len), GFP_ATOMIC);
	if (!d)
		return NULL;

	memcpy(&d->pcfg, ch->cfg.peripheral_config, ch->cfg.peripheral_size);
	ma35d1_set_channel_params(ch);
	ma35d1_set_transfer_params(ch, d->pcfg.reqsel);
	d->sg_addr = (u32) dma_map_single(ch->dev,
					  (void *)(d->sg),
					  sizeof(struct ma35d1_sg) * sg_len,
					  DMA_BIDIRECTIONAL);

	writel(d->sg_addr,
	       ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE) +
	       PDMA_DSCT_NEXT);
	/* Split the buffer into period size chunks */
	for (offset = 0, i = 0; offset < buf_len; offset += period_len, i++) {
		d->sg[i].ctl = (ch->runtime_ctrl |
		     PDMA_TXCNT((period_len/ch->runtime_width)-1) |
		     PDMA_OP_SCATTER);
		if (dir == DMA_MEM_TO_DEV) {
			d->sg[i].src = dma_addr + offset;
			d->sg[i].dst = ch->runtime_addr;
			d->dma_dir = DMA_MEM_TO_DEV;

		} else {
			d->sg[i].src = ch->runtime_addr;
			d->sg[i].dst = dma_addr + offset;
			d->dma_dir = DMA_DEV_TO_MEM;
		}
		d->sg[i].next = d->sg_addr + (PDMA_OFFSET_CHAN_SIZE * (i + 1));
	}
	d->sglen = sg_len;
	d->sg[d->sglen - 1].next = d->sg_addr;
	dma_sync_single_for_cpu(ch->dev, d->sg_addr, sizeof(*d->sg) * d->sglen,
				DMA_FROM_DEVICE);
	d->cyclic = true;
	ch->error = 0;
	return vchan_tx_prep(&ch->vc, &d->vd, flags);
}

static struct dma_chan *ma35d1_of_xlate(struct of_phandle_args *dma_spec,
					struct of_dma *of)
{
	struct ma35d1_dmadev *edma = of->of_dma_data;
	unsigned int request;

	if (dma_spec->args_count != 1)
		return NULL;

	request = dma_spec->args[0];
	if (request >= edma->nr_chans)
		return NULL;

	return dma_get_slave_channel(&(edma->channels[request].vc.chan));
}

static int ma35d1_alloc_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);

	dev_dbg(chan2dev(chan), "%s: allocating channel #%u\n",
		__func__, ch->ch_num);
	ch->allocated = 1;

	return 0;
}

static void ma35d1_free_chan_resources(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);

	vchan_free_chan_resources(&ch->vc);
	dev_dbg(chan2dev(chan), "%s: freeing channel #%u\n",
		__func__, ch->ch_num);
	ch->allocated = 0;
}

static void ma35d1_start_dma(struct ma35d1_chan *ch)
{
	struct ma35d1_desc *d = ch->desc;
	u32 ctrl;
	void __iomem *ch_base;

	ch_base = ch->base + (ch->ch_num * PDMA_OFFSET_CHAN_SIZE);
	if (d->sglen == 1) {
		writel(d->sg[0].ctl, ch_base + PDMA_DSCT_CTL);
		writel(d->sg[0].src, ch_base + PDMA_DSCT_SA);
		writel(d->sg[0].dst, ch_base + PDMA_DSCT_DA);
		ctrl = readl(ch_base + PDMA_DSCT_CTL);
		ctrl |= PDMA_OP_BASIC;
		writel(ctrl, ch_base + PDMA_DSCT_CTL);

		/* Memory to memory mode , when reqsel is equal to 0 */
		if (d->pcfg.reqsel == 0)
			writel((1 << ch->ch_num), ch->base + PDMA_SWREQ);
	} else {
		/* scatter gather mode */
		ctrl = readl(ch_base + PDMA_DSCT_CTL);
		ctrl = (ctrl & ~PDMA_OP_MSK) | PDMA_OP_SCATTER;
		writel(ctrl, ch_base + PDMA_DSCT_CTL);
	}
}

static void ma35d1_dma_start_sg(struct ma35d1_chan *ch)
{
	ma35d1_set_dma_timeout(ch);
	ma35d1_start_dma(ch);
}

static void ma35d1_dma_start_desc(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	struct virt_dma_desc *vd;

	vd = vchan_next_desc(&ch->vc);
	if (!vd) {
		ch->desc = NULL;
		return;
	}
	list_del(&vd->node);
	ch->desc = to_ma35d1_dma_desc(&vd->tx);
	ma35d1_dma_start_sg(ch);

}

static void ma35d1_issue_pending(struct dma_chan *chan)
{
	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&ch->vc.lock, flags);
	if (vchan_issue_pending(&ch->vc) && !ch->desc)
		ma35d1_dma_start_desc(chan);
	spin_unlock_irqrestore(&ch->vc.lock, flags);
}

static enum dma_status ma35d1_tx_status(struct dma_chan *chan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{

	struct ma35d1_chan *ch = to_ma35d1_dma_chan(chan);
	enum dma_status ret;
	unsigned long flags;

	spin_lock_irqsave(&ch->vc.lock, flags);
	ret = dma_cookie_status(chan, cookie, txstate);
	if (likely(ret != DMA_ERROR))
		dma_set_residue(txstate, ch->remain);
	spin_unlock_irqrestore(&ch->vc.lock, flags);

	return ret;
}

static irqreturn_t ma35d1_dma_interrupt(int irq, void *devid)
{
	int i;
	u32 val;
	struct ma35d1_dmadev *dmadev = devid;
	unsigned int intsts, tdsts;
	struct ma35d1_chan *ch = &dmadev->channels[dmadev->nr_chans - 1];

	intsts = readl(ch->base + PDMA_INTSTS);
	tdsts = readl(ch->base + PDMA_TDSTS);
	for (i = (dmadev->nr_chans - 1); i >= 0; i--, ch--) {
		/* Transfer done interrupt */
		if (tdsts & (1 << i)) {
			writel((1 << i), ch->base + PDMA_TDSTS);
			if (ch->desc) {
				ch->remain = 0;
				if (!ch->desc->cyclic) {
					vchan_cookie_complete(&ch->desc->vd);
					ma35d1_dma_start_desc(&ch->vc.chan);
				} else
					vchan_cyclic_callback(&ch->desc->vd);
			}
		}

		/* Timeout interrupt */
		if (intsts & (1 << (i + 8))) {
			val =
			    readl(ch->base +
				  ch->ch_num * PDMA_OFFSET_CHAN_SIZE);
			ch->remain = PDMA_GET_TXCNT(val);
			val = readl(ch->base + PDMA_TOUTEN);
			val &= ~(1 << i);
			writel(val, ch->base + PDMA_TOUTEN);
			writel(1 << (i + 8), ch->base + PDMA_INTSTS);
			if (ch->desc) {
				vchan_cookie_complete(&ch->desc->vd);
				ma35d1_dma_start_desc(&ch->vc.chan);
			}
		}

		/* Abort interrupt */
		if (intsts & 0x1) {
			writel(0x1, ch->base + PDMA_TDSTS);
			ch->error = 1;
		}
	}

	return IRQ_HANDLED;
}

static void ma35d1_dma_release(struct dma_device *dma_dev)
{
	struct ma35d1_dmadev *edma =
		container_of(dma_dev, struct ma35d1_dmadev, ddev);

	put_device(edma->ddev.dev);
	kfree(edma);
}

static int ma35d1_probe(struct platform_device *pdev)
{
	struct clk *pdma_clk;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;
	void __iomem *dma_base_addr;
	int ret, i, nr_chans;
	unsigned int irq;
	struct ma35d1_chan *ch;
	struct ma35d1_dmadev *edma;

	pdma_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(pdma_clk)) {
		ret = PTR_ERR(pdma_clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", ret);
		return -ENOENT;
	}
	ret = clk_prepare_enable(pdma_clk);
	if (ret)
		return -ENOENT;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	edma = devm_kzalloc(dev, sizeof(*edma), GFP_KERNEL);
	if (!edma)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(dev, "no IRQ resource\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "dma-channels", &nr_chans))
		return -EINVAL;
	if (nr_chans > PDMA_MAX_CHANS)
		nr_chans = PDMA_MAX_CHANS;
	edma->nr_chans = nr_chans;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dma_base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(dma_base_addr))
		return PTR_ERR(dma_base_addr);

	dma_cap_zero(edma->ddev.cap_mask);
	dma_cap_set(DMA_SLAVE, edma->ddev.cap_mask);
	dma_cap_set(DMA_PRIVATE, edma->ddev.cap_mask);
	dma_cap_set(DMA_MEMCPY, edma->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, edma->ddev.cap_mask);
	edma->ddev.device_prep_dma_memcpy = ma35d1_prep_dma_memcpy;
	edma->ddev.device_prep_slave_sg = ma35d1_prep_slave_sg;
	edma->ddev.device_prep_dma_cyclic = ma35d1_prep_dma_cyclic;
	edma->ddev.device_alloc_chan_resources = ma35d1_alloc_chan_resources;
	edma->ddev.device_free_chan_resources = ma35d1_free_chan_resources;
	edma->ddev.device_issue_pending = ma35d1_issue_pending;
	edma->ddev.device_tx_status = ma35d1_tx_status;
	edma->ddev.device_config = ma35d1_slave_config;
	edma->ddev.device_terminate_all = ma35d1_terminate_all;
	edma->ddev.device_release = ma35d1_dma_release;
	edma->ddev.dev = dev;
	edma->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
	    BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	edma->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
	    BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	edma->ddev.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
	    BIT(DMA_MEM_TO_MEM);

	spin_lock_init(&edma->lock);
	INIT_LIST_HEAD(&edma->ddev.channels);
	ch = &edma->channels[0];
	for (i = 0; i < nr_chans; i++, ch++) {
		ch->dev = &pdev->dev;
		ch->edma = edma;
		ch->ch_num = i;
		ch->base = dma_base_addr;
		ch->allocated = 0;

		ch->vc.desc_free = ma35d1_dma_desc_free;
		vchan_init(&ch->vc, &edma->ddev);
		dev_dbg(dev, "%s: chs[%d]: ch->ch_num=%u ch->base=%p\n",
			__func__, i, ch->ch_num, ch->base);
	}

	platform_set_drvdata(pdev, edma);

	ret = devm_request_irq(dev, irq, ma35d1_dma_interrupt, 0,
			       pdev->name, edma);
	if (ret) {
		dev_err(dev, "devm_request_irq failed\n");
		return ret;
	}
	edma->irq = irq;

	ret = dma_async_device_register(&edma->ddev);
	if (ret) {
		dev_err(dev, "dma_async_device_register failed\n");
		return ret;
	}

	ret = of_dma_controller_register(node, ma35d1_of_xlate, edma);
	if (ret) {
		dev_err(dev, "of_dma_controller_register failed\n");
		dma_async_device_unregister(&edma->ddev);
		return ret;
	}

	dev_dbg(dev, "%s: IRQ=%u\n", __func__, irq);
	return 0;
}

static int ma35d1_remove(struct platform_device *pdev)
{
	struct ma35d1_dmadev *m = platform_get_drvdata(pdev);

	devm_free_irq(&pdev->dev, m->irq, m);

	dma_async_device_unregister(&m->ddev);

	if (pdev->dev.of_node)
		of_dma_controller_free(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id ma35d1_dma_match[] = {
	{.compatible = "nuvoton,ma35d1-dma" },
	{ }
};

MODULE_DEVICE_TABLE(of, ma35d1_dma_match);

static struct platform_driver ma35d1_driver = {
	.probe = ma35d1_probe,
	.remove = ma35d1_remove,
	.driver = {
		   .name = "ma35d1-dma",
		   .of_match_table = ma35d1_dma_match,
		    },
};

static int ma35d1_init(void)
{
	return platform_driver_register(&ma35d1_driver);
}

subsys_initcall(ma35d1_init);

MODULE_AUTHOR("Shan-Chun Hung <schung@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton MA35D1 DMA controller driver");
MODULE_LICENSE("GPL v2");
