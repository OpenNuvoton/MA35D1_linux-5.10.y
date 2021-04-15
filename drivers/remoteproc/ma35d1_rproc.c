/*
 *  linux/drivers/remoteproc/ma35d1_rproc.c
 *
 *  MA35D1 remote processors driver
 *
 *
 *  Copyright (C) 2020 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"


enum ma35d1_rproc_messages {
	RPROC_ECHO_REPLY	= 0xFF000000,
};


struct ma35d1_rproc {
	struct device           *dev;
	struct rproc            *rproc;
	struct mbox_chan        *channel;
	struct mbox_client      mbox_client1;
	struct reset_control    *m4_rst;
	void   __iomem          *da_to_va_addr;
	u32    da_to_va_offset;
};

static void ma35d1_rx_callback(struct mbox_client *cl, void *msg)
{
	struct ma35d1_rproc *nproc = container_of(cl, struct ma35d1_rproc, mbox_client1);
	struct device *dev = nproc->rproc->dev.parent;
	const char *name = nproc->rproc->name;
	u32 *data = (u32 *)msg;

	switch (*data) {
		case RPROC_ECHO_REPLY:
			dev_info(dev, "received echo reply from %s\n", name);
		break;

		default:
			/* msg contains the index of the triggered vring */
			if (rproc_vq_interrupt(nproc->rproc, *data) == IRQ_NONE)
				dev_dbg(dev, "no message was found in vqid %d\n", *data);
	}
}

static int ma35d1_rproc_start(struct rproc *rproc)
{
	struct ma35d1_rproc *nproc = rproc->priv;
	struct device *dev = rproc->dev.parent;

	reset_control_deassert(nproc->m4_rst);

	nproc->mbox_client1.dev          = dev;
	nproc->mbox_client1.rx_callback  = ma35d1_rx_callback;
	nproc->mbox_client1.tx_done	 = NULL;
	nproc->mbox_client1.tx_block	 = false;
	nproc->mbox_client1.knows_txdone = false;
	nproc->mbox_client1.tx_tout	 = 1;

	nproc->channel = mbox_request_channel(&nproc->mbox_client1, 0);
	if (IS_ERR(nproc->channel)) {
		dev_err(dev, "mbox_request_channel failed: %ld\n", PTR_ERR(nproc->channel));
		return -EBUSY;
	}

	return 0;
}

static int ma35d1_rproc_stop(struct rproc *rproc)
{
	struct ma35d1_rproc *nproc = rproc->priv;

	nproc->da_to_va_offset = 0;

	reset_control_assert(nproc->m4_rst);

	if(nproc->channel)
		mbox_free_channel(nproc->channel);

	return 0;
}

static void ma35d1_rproc_kick(struct rproc *rproc, int vqid)
{
	struct ma35d1_rproc *nproc = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int ret;

	ret = mbox_send_message(nproc->channel, (void *)&vqid);
	if (ret < 0)
		dev_err(dev, "failed to send mailbox message, status = %d\n", ret);
}

static void *ma35d1_m4_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct ma35d1_rproc *nproc = rproc->priv;
	void *va = NULL;

	if (len <= 0)
		return NULL;

	va = (__force void *)(nproc->da_to_va_addr + nproc->da_to_va_offset);

	nproc->da_to_va_offset = nproc->da_to_va_offset + len;

	return va;
}

int ma35d1_rproc_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct ma35d1_rproc *nproc = rproc->priv;
	struct device *dev = &rproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0, j;
	const u8 *elf_data = fw->data;

	reset_control_assert(nproc->m4_rst);

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n", phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%x mem 0x%x\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		for(j = 0; j <= phdr->p_filesz; j++)
		{
			*((volatile unsigned char *)(ptr+j)) = *((volatile unsigned char *)(elf_data + phdr->p_offset+j));
		}

		/*
		 * Zero out remaining memory for this segment.
		 *
		 * This isn't strictly required since dma_alloc_coherent already
		 * did this for us. albeit harmless, we may consider removing
		 * this.
		 */
		for(j = 0; j <= (memsz - filesz); j++)
		{
			*((volatile unsigned char *)(ptr + filesz+j)) = 0x0;
		}
	}

	return ret;
}



static int ma35d1_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
  return 0;
}

static struct rproc_ops ma35d1_rproc_ops = {
	.start		= ma35d1_rproc_start,
	.stop		= ma35d1_rproc_stop,
	.kick		= ma35d1_rproc_kick,
	.da_to_va	= ma35d1_m4_rproc_da_to_va,
	.load           = ma35d1_rproc_elf_load_segments,
	.parse_fw	= ma35d1_rproc_parse_fw,
	.sanity_check	= rproc_elf_sanity_check,
};

static const struct of_device_id ma35d1_rproc_match[] = {
	{ .compatible = "nuvoton, ma35d1-rproc" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_rproc_match);

static int ma35d1_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct ma35d1_rproc *nproc;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	int ret;

	match = of_match_device(ma35d1_rproc_match, dev);
	if (!match) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	rproc = rproc_alloc(dev, np->name, &ma35d1_rproc_ops, NULL, sizeof(*nproc));
	if (!rproc)
		return -ENOMEM;

	rproc->auto_boot = false;

	nproc = rproc->priv;
	nproc->rproc = rproc;
	nproc->dev = dev;

	nproc->da_to_va_addr = devm_ioremap_nocache(&pdev->dev, 0x24000000, 0x20000);
	nproc->da_to_va_offset = 0;

	platform_set_drvdata(pdev, rproc);

	nproc->m4_rst = devm_reset_control_get(&pdev->dev, NULL); // error
	if (IS_ERR(nproc->m4_rst)) {
		dev_err(nproc->dev, "Error: Missing rproc controller reset\n");
	}

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	return 0;

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int ma35d1_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static struct platform_driver ma35d1_rproc_driver = {
	.probe = ma35d1_rproc_probe,
	.remove = ma35d1_rproc_remove,
	.driver = {
		.name = "ma35d1-rproc",
		.of_match_table = of_match_ptr(ma35d1_rproc_match),
	},
};
module_platform_driver(ma35d1_rproc_driver);

MODULE_DESCRIPTION("MA35D1 Remote Processor Control Driver");
MODULE_LICENSE("GPL v2");

