// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/of.h>

#include <uapi/misc/ma35d1_ebi.h>
#include "regs-ma35d1-ebi.h"

#define EBI_CH			3


struct ebi_dev {
	int minor;// dynamic minor num, so we need this to distinguish between channels
	struct clk *clk;
	unsigned long bank_addr[3];
	void __iomem  *reg_base;
	u8 bank;
	unsigned int busmode;
	unsigned int csactiveLevel;
	unsigned int datawidth;
	unsigned int MCLKDIV;
	unsigned int tALE;
	unsigned int tACC;
	unsigned int tAHD;
	unsigned int IDLE;
};

static struct ebi_dev *ebi[EBI_CH];

static inline void ma35d1_set_ebi_ctl(u8 u8Bank)
{
	void __iomem *EBIBaseAddr = ebi[u8Bank]->reg_base;
	unsigned int TIMIING_CTL, TIMIING_TCTL;

	if(ebi[u8Bank]->busmode == EBI_OPMODE_ADSEPARATE) {
		writel_relaxed(readl_relaxed(EBIBaseAddr) | EBI_CTL_ADSEPEN_Msk, EBIBaseAddr);
	} else {
		writel_relaxed(readl_relaxed(EBIBaseAddr) & ~EBI_CTL_ADSEPEN_Msk, EBIBaseAddr);
	}

	if(ebi[u8Bank]->csactiveLevel == EBI_CS_ACTIVE_LOW) {
		writel_relaxed(readl_relaxed(EBIBaseAddr) & ~EBI_CTL_CSPOLINV_Msk, EBIBaseAddr);
	} else {
		writel_relaxed(readl_relaxed(EBIBaseAddr) | EBI_CTL_CSPOLINV_Msk, EBIBaseAddr);
	}

	if(ebi[u8Bank]->datawidth == EBI_BUSWIDTH_8BIT) {
		writel_relaxed(readl_relaxed(EBIBaseAddr) & ~EBI_CTL_DW16_Msk, EBIBaseAddr);
	} else {
		writel_relaxed(readl_relaxed(EBIBaseAddr) | EBI_CTL_DW16_Msk, EBIBaseAddr);
	}

	TIMIING_CTL = (readl_relaxed(EBIBaseAddr) & ~(EBI_CTL_MCLKDIV_Msk | EBI_CTL_TALE_Msk)) |
			((ebi[u8Bank]->MCLKDIV) << EBI_CTL_MCLKDIV_Pos) |
			((ebi[u8Bank]->tALE) << EBI_CTL_TALE_Pos);

	writel_relaxed(TIMIING_CTL | EBI_CTL_EN_Msk, EBIBaseAddr);

	TIMIING_TCTL = ((ebi[u8Bank]->tACC) << EBI_TCTL_TACC_Pos) |
			((ebi[u8Bank]->tAHD) << EBI_TCTL_TAHD_Pos) |
			((ebi[u8Bank]->IDLE) << EBI_TCTL_W2X_Pos) |
			((ebi[u8Bank]->IDLE) << EBI_TCTL_R2R_Pos);
	writel_relaxed(TIMIING_TCTL, EBIBaseAddr+REG_EBI_TCTL);

}

static long ebi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ma35d1_set_ebi *pebi, sebi;
	struct ebi_dev *t = (struct ebi_dev *)filp->private_data;
	u8 ch;

	pebi=&sebi;

	switch(cmd) {
	case EBI_IOC_SET:
		if(copy_from_user((void *)&sebi, (const void *)arg,
		                  sizeof(struct ma35d1_set_ebi)))
			return -EFAULT;

		ch = pebi->bank;
		t->bank = ch;
		t->bank_addr[ch] = pebi->base;
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static int ebi_open(struct inode *inode, struct file *filp)
{
	u8 i, ch;

	for(i = 0; i < EBI_CH; i++) {
		if(ebi[i]->minor == iminor(inode)) {
			ch = i;
			break;
		}
	}

	filp->private_data = ebi[ch];

	return 0;
}

static int ebi_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static int ebi_mmap(struct file *filp, struct vm_area_struct * vma);
struct file_operations ebi_fops = {
	.owner		= THIS_MODULE,
	.open		= ebi_open,
	.release	= ebi_release,
	.mmap 		= ebi_mmap,
	.unlocked_ioctl	= ebi_ioctl,
};

static struct miscdevice ebi_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "ebi0",
		.fops = &ebi_fops,
	},
	[1] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "ebi1",
		.fops = &ebi_fops,
	},
	[2] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "ebi2",
		.fops = &ebi_fops,
	},
};


static int ebi_mmap(struct file *filp, struct vm_area_struct * vma)
{
	struct ebi_dev *t = (struct ebi_dev *)filp->private_data;
	unsigned long pageFrameNo = 0, size;

	pageFrameNo = __phys_to_pfn(t->bank_addr[t->bank]);

	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo,size, vma->vm_page_prot)) {
		printk(KERN_INFO "ma35d1_mem_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}

	return 0;
}

static int ma35d1_ebi_probe(struct platform_device *pdev)
{
	unsigned int ch;
	int ret;

	if (of_property_read_u32_array(pdev->dev.of_node, "bank", &ch,
	                               1) != 0) {
		dev_err(&pdev->dev, "can not get bank!\n");
		return -EINVAL;
	}

	ebi[ch] = devm_kzalloc(&pdev->dev, sizeof(struct ebi_dev), GFP_KERNEL);
	if (ebi[ch] == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for ebi device\n");
		return -ENOMEM;
	}

	ebi[ch]->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ebi[ch]->reg_base)) {
		dev_err(&pdev->dev, "could not map region\n");
		return PTR_ERR(ebi[ch]->reg_base);
	}

	ret = misc_register(&ebi_dev[ch]);
	if (ret) {
		dev_err(&pdev->dev, "misc registration failed\n");
		return ret;
	}

	ebi[ch]->clk = devm_clk_get(&pdev->dev, "ebi_gate");
	if (IS_ERR(ebi[ch]->clk)) {
		if (PTR_ERR(ebi[ch]->clk) != -ENOENT)
			return PTR_ERR(ebi[ch]->clk);

		ebi[ch]->clk = NULL;
	}

	ret = clk_prepare_enable(ebi[ch]->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clock\n");
		return ret;
	}

	ebi[ch]->minor = MINOR(ebi_dev[ch].minor);
	ebi[ch]->bank = ch;

	ret = of_property_read_u32_array(pdev->dev.of_node,"busmode",&ebi[ch]->busmode, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI 'busmode' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"csactiveLevel",&ebi[ch]->csactiveLevel, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI 'csactiveLevel' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"datawidth",&ebi[ch]->datawidth, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI 'datawidth' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"MCLKDIV",&ebi[ch]->MCLKDIV, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI Timing control 'MCLKDIV' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"tALE",&ebi[ch]->tALE, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI Timing control 'tALE' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"tACC",&ebi[ch]->tACC, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI Timing control 'tACC' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"tAHD",&ebi[ch]->tAHD, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI Timing control 'tAHD' cannot be read!\n");
		return ret;
	}
	ret = of_property_read_u32_array(pdev->dev.of_node,"IDLE",&ebi[ch]->IDLE, 1);
	if (ret) {
		dev_err(&pdev->dev, "EBI Timing control 'IDLE' cannot be read!\n");
		return ret;
	}

	ma35d1_set_ebi_ctl(ebi[ch]->bank);

	platform_set_drvdata(pdev, ebi[ch]);

	return 0;
}

static int ma35d1_ebi_remove(struct platform_device *pdev)
{
	struct ebi_dev *t = platform_get_drvdata(pdev);
	u8 ch = t->bank;

	clk_disable(ebi[ch]->clk);
	clk_put(ebi[ch]->clk);
	misc_deregister(&ebi_dev[ch]);

	return 0;
}

static const struct of_device_id ma35d1_ebi_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-ebi" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_ebi_of_match);

static struct platform_driver ma35d1_ebi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name = "ma35d1-ebi",
		.of_match_table = of_match_ptr(ma35d1_ebi_of_match),
	},
	.probe = ma35d1_ebi_probe,
	.remove = ma35d1_ebi_remove,
};

module_platform_driver(ma35d1_ebi_driver);

MODULE_ALIAS("platform:ma35d1-ebi");
MODULE_DESCRIPTION("EBI driver for Nuvoton MA35D1");
MODULE_LICENSE("GPL v2");
