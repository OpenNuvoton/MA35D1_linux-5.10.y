// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35H0 Key Store
 *
 * Copyright (C) 2018-2019 Linaro Ltd.
 * Copyright (c) 2023 Nuvoton technology corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/highmem.h>

#include <uapi/misc/ma35h0_ks.h>

#define MISCDEV_NAME		"ksdev"

#define KS_BUSY_TIMEOUT		1000

struct ma35h0_ks_dev {
	struct device *dev;
	struct miscdevice miscdev;
	wait_queue_head_t waitq;
	void __iomem  *reg_base;
	int  irq;
	int  int_err_sts;
};

static char  ks_miscdev_name[] = MISCDEV_NAME;

static uint16_t au8SRAMCntTbl[21] = {4, 6, 6, 7, 8, 8, 8, 9, 12, 13, 16, 17,
				     18, 0, 0, 0, 32, 48, 64, 96, 128};
static uint16_t au8OTPCntTbl[7] = {4, 6, 6, 7, 8, 8, 8};

/*-------------------------------------------------------------------*/
/*  Key Store driver for MA35H0 Key Store                           */
/*-------------------------------------------------------------------*/

static inline u32  nu_ks_read_reg(struct ma35h0_ks_dev *ks_dev, u32 offset)
{
	u32 value = readl_relaxed(ks_dev->reg_base + offset);

	dev_vdbg(ks_dev->dev, "reg read 0x%08x from <0x%x>\n", value, offset);
	return value;
}

static inline void nu_ks_write_reg(struct ma35h0_ks_dev *ks_dev,
				   u32 offset, u32 value)
{
	dev_vdbg(ks_dev->dev, "write 0x%08x into <0x%x>\n", value, offset);
	writel_relaxed(value, ks_dev->reg_base + offset);
}

static inline int ma35h0_ks_wait_busy_clear(struct ma35h0_ks_dev *ks_dev)
{
	unsigned long  timeout = jiffies + msecs_to_jiffies(KS_BUSY_TIMEOUT);

	while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		if (time_after(jiffies, timeout))
			return -EBUSY;
	}
	return 0;
}

static int ma35h0_ks_read(struct ma35h0_ks_dev *ks_dev, void __user *arg)
{
	struct ks_read_args  r_args;
	int         err, remain_cnt;
	u32         cont_msk;
	int         offset, i, cnt;

	err = copy_from_user(&r_args, arg, sizeof(r_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (r_args.type <<
			KS_METADATA_DST_Pos) | KS_TOMETAKEY(r_args.key_idx));

	/* Clear error flag */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF);
	offset = 0;
	cont_msk = 0;
	remain_cnt = r_args.word_cnt;

	do {

		/* Clear Status */
		nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

		/* Trigger to read the key */
		nu_ks_write_reg(ks_dev, KS_CTL, cont_msk | KS_OP_READ |
				KS_CTL_START | (nu_ks_read_reg(ks_dev,
				KS_CTL) & KS_CLT_FUNC_MASK));

		/* Waiting for key store processing */
		if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
			return -EBUSY;

		/* Read the key to key buffer */
		cnt = remain_cnt;
		if (cnt > 8)
			cnt = 8;
		for (i = 0; i < cnt; i++) {
			r_args.key[offset+i] = nu_ks_read_reg(ks_dev,
				KS_KEY(i));
		}

		cont_msk = KS_CTL_CONT;
		remain_cnt -= 8;
		offset += 8;

	} while (remain_cnt > 0);

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on reading keys!\n");
		return -EIO;
	}

	err = copy_to_user(arg, &r_args, sizeof(r_args));
	if (err)
		err = -EFAULT;

	return err;
}

static int ma35h0_ks_write_sram(struct ma35h0_ks_dev *ks_dev,
				 void __user *arg)
{
	struct ks_write_args  w_args;
	int         err, remain_cnt;
	u32         cont_msk;
	int         offset, i, cnt;

	err = copy_from_user(&w_args, arg, sizeof(w_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (KS_SRAM << KS_METADATA_DST_Pos) |
			w_args.meta_data);

	/* Get key size by indexing to size table */
	i = ((w_args.meta_data & KS_METADATA_SIZE_Msk) >>
		KS_METADATA_SIZE_Pos);
	remain_cnt = au8SRAMCntTbl[i];

	/* Invalid key length */
	if (remain_cnt == 0) {
		pr_err("Invalid key length!\n");
		return -EINVAL;
	}

	/* Clear error flag */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF);
	offset = 0;
	cont_msk = 0;
	do {
		/* Prepare the key to write */
		cnt = remain_cnt;
		if (cnt > 8)
			cnt = 8;
		for (i = 0; i < cnt; i++) {
			nu_ks_write_reg(ks_dev, KS_KEY(i),
					w_args.key[offset + i]);
		}

		/* Clear Status */
		nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

		/* Write the key */
		nu_ks_write_reg(ks_dev, KS_CTL,	cont_msk | KS_OP_WRITE |
				KS_CTL_START | (nu_ks_read_reg(ks_dev,
				KS_CTL) & KS_CLT_FUNC_MASK));

		cont_msk = KS_CTL_CONT;
		remain_cnt -= 8;
		offset += 8;

		/* Waiting for key store processing */
		if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
			return -EBUSY;

	} while (remain_cnt > 0);

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on writing SRAM keys!\n");
		return -EIO;
	}

	return KS_TOKEYIDX(nu_ks_read_reg(ks_dev, KS_METADATA));
}

static int ma35h0_ks_write_otp(struct ma35h0_ks_dev *ks_dev, void __user *arg)
{
	struct ks_write_args  w_args;
	int         err, remain_cnt;
	u32         cont_msk;
	int         sidx, offset, i, cnt;

	err = copy_from_user(&w_args, arg, sizeof(w_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (KS_OTP << KS_METADATA_DST_Pos) |
			w_args.meta_data | KS_TOMETAKEY(w_args.key_idx));

	/* Get size index */
	sidx = ((w_args.meta_data & KS_METADATA_SIZE_Msk) >>
		KS_METADATA_SIZE_Pos);

	/* OTP only support maximum 256 bits */
	if (sidx >= 7)
		return -1;

	remain_cnt = au8OTPCntTbl[sidx];

	/* Clear error flag */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF);
	offset = 0;
	cont_msk = 0;
	do  {
		/* Prepare the key to write */
		cnt = remain_cnt;
		if (cnt > 8)
			cnt = 8;
		for (i = 0; i < cnt; i++) {
			nu_ks_write_reg(ks_dev, KS_KEY(i),
					w_args.key[offset + i]);
		}

		/* Clear Status */
		nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

		/* Write the key */
		nu_ks_write_reg(ks_dev, KS_CTL,	cont_msk | KS_OP_WRITE |
				KS_CTL_START | (nu_ks_read_reg(ks_dev, KS_CTL)
				& KS_CLT_FUNC_MASK));

		cont_msk = KS_CTL_CONT;
		remain_cnt -= 8;
		offset += 8;

		/* Waiting for key store processing */
		if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
			return -EBUSY;

	} while (remain_cnt > 0);

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on writing OTP keys!\n");
		return -EIO;
	}
	return 0;
}

static int ma35h0_ks_erase(struct ma35h0_ks_dev *ks_dev, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (k_args.type <<
			KS_METADATA_DST_Pos) | KS_TOMETAKEY(k_args.key_idx));

	/* Clear Status */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

	/* Erase the key */
	nu_ks_write_reg(ks_dev, KS_CTL, KS_OP_ERASE | KS_CTL_START |
			(nu_ks_read_reg(ks_dev, KS_CTL) & KS_CLT_FUNC_MASK));

	/* Waiting for processing */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
		return -EBUSY;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on erasing a key!\n");
		return -EIO;
	}
	return 0;
}

static int ma35h0_ks_erase_all(struct ma35h0_ks_dev *ks_dev)
{
	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (KS_SRAM << KS_METADATA_DST_Pos));

	/* Clear Status */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

	/* Erase the key */
	nu_ks_write_reg(ks_dev, KS_CTL, KS_OP_ERASE_ALL | KS_CTL_START |
			(nu_ks_read_reg(ks_dev, KS_CTL) & KS_CLT_FUNC_MASK));

	/* Waiting for processing */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
		return -EBUSY;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on erase-all!\n");
		return -EIO;
	}
	return 0;
}

static int ma35h0_ks_revoke(struct ma35h0_ks_dev *ks_dev, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0) {
		pr_err("MA35H0 KS is busy!\n");
		return -EBUSY;
	}

	/* Specify the key address */
	nu_ks_write_reg(ks_dev, KS_METADATA, (k_args.type <<
			KS_METADATA_DST_Pos) | KS_TOMETAKEY(k_args.key_idx));

	/* Clear Status */
	nu_ks_write_reg(ks_dev, KS_STS, KS_STS_EIF | KS_STS_IF);

	/* Revoke the key */
	nu_ks_write_reg(ks_dev, KS_CTL,	KS_OP_REVOKE | KS_CTL_START |
			(nu_ks_read_reg(ks_dev, KS_CTL) & KS_CLT_FUNC_MASK));

	/* Waiting for processing */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
		return -EBUSY;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on revoking a key!\n");
		return -EIO;
	}
	return 0;
}

static int ma35h0_ks_remain(struct ma35h0_ks_dev *ks_dev)
{
	u32	reg_data, sram_remain;

	reg_data = nu_ks_read_reg(ks_dev, KS_REMAIN);
	sram_remain = (reg_data & KS_REMAIN_RRMNG_Msk) >> KS_REMAIN_RRMNG_Pos;

	return sram_remain;
}

static int ks_dev_open(struct inode *iptr, struct file *fptr)
{
	struct ma35h0_ks_dev *ks_dev;
	unsigned long  timeout;

	ks_dev = container_of(fptr->private_data, struct ma35h0_ks_dev,
			      miscdev);

	/* Start Key Store Initial */
	nu_ks_write_reg(ks_dev, KS_CTL, KS_CTL_INIT | KS_CTL_START);

	/* Waiting for KeyStore initilization done */
	timeout = jiffies + msecs_to_jiffies(KS_BUSY_TIMEOUT);
	while ((nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_INITDONE) == 0) {
		if (time_after(jiffies, timeout))
			return -EIO;
	}

	/* Waiting for processing */
	if (ma35h0_ks_wait_busy_clear(ks_dev) != 0)
		return -EBUSY;

	return 0;
}

static int ks_dev_release(struct inode *iptr, struct file *fptr)
{
	return 0;
}

static long ks_dev_ioctl(struct file *fptr, unsigned int cmd,
				 unsigned long data)
{
	struct ma35h0_ks_dev *ks_dev;
	char __user *argp = (char __user *)data;
	int rval = -EINVAL;

	ks_dev = container_of(fptr->private_data, struct ma35h0_ks_dev,
			      miscdev);

	if (_IOC_TYPE(cmd) != MA35H0_KS_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case NU_KS_IOCTL_READ:
		rval = ma35h0_ks_read(ks_dev, argp);
		break;
	case NU_KS_IOCTL_WRITE_SRAM:
		rval = ma35h0_ks_write_sram(ks_dev, argp);
		break;
	case NU_KS_IOCTL_WRITE_OTP:
		rval = ma35h0_ks_write_otp(ks_dev, argp);
		break;
	case NU_KS_IOCTL_ERASE:
		rval = ma35h0_ks_erase(ks_dev, argp);
		break;
	case NU_KS_IOCTL_ERASE_ALL:
		rval = ma35h0_ks_erase_all(ks_dev);
		break;
	case NU_KS_IOCTL_REVOKE:
		rval = ma35h0_ks_revoke(ks_dev, argp);
		break;
	case NU_KS_IOCTL_GET_REMAIN:
		rval = ma35h0_ks_remain(ks_dev);
		break;
	default:
		/* Should not get here */
		break;
	}
	return rval;
}

static const struct file_operations ma35h0_ks_fops = {
	.owner = THIS_MODULE,
	.open = ks_dev_open,
	.release = ks_dev_release,
	.unlocked_ioctl = ks_dev_ioctl,
	.compat_ioctl = ks_dev_ioctl,
};


static int ma35h0_ks_probe(struct platform_device *pdev)
{
	struct ma35h0_ks_dev	*ks_dev;
	struct device		*dev;
	struct resource		*res;
	int err;
	dev = &pdev->dev;

	ks_dev = devm_kzalloc(&pdev->dev, sizeof(*ks_dev), GFP_KERNEL);
	if (!ks_dev)
		return -ENOMEM;

	ks_dev->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ks_dev->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ks_dev->reg_base)) {
		err = PTR_ERR(ks_dev->reg_base);
		goto err_ks_dev;
	}

	ks_dev->irq = platform_get_irq(pdev, 0);
	if (ks_dev->irq < 0) {
		dev_dbg(dev, "platform_get_irq failed");
		goto err_ks_dev;
	}

	/* Save driver private data */
	platform_set_drvdata(pdev, ks_dev);

	ks_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	ks_dev->miscdev.name = ks_miscdev_name;
	ks_dev->miscdev.fops = &ma35h0_ks_fops;
	ks_dev->miscdev.parent = dev;
	err = misc_register(&ks_dev->miscdev);
	if (err) {
		dev_err(dev, "error:%d. Unable to register device", err);
		goto err_ks_dev;
	}
	return 0;

err_ks_dev:
	return err;
}

static int ma35h0_ks_remove(struct platform_device *pdev)
{
	struct ma35h0_ks_dev *ks_dev;

	ks_dev = platform_get_drvdata(pdev);
	misc_deregister(&ks_dev->miscdev);
	return 0;
}

static const struct of_device_id ma35h0_ks_of_match[] = {
	{
		.compatible = "nuvoton,ma35h0-ks",
	},
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, ma35h0_ks_of_match);

static struct platform_driver ma35h0_ks_driver = {
	.probe = ma35h0_ks_probe,
	.remove =  ma35h0_ks_remove,
	.driver = {
		.name = "nuvoton-ks",
		.of_match_table = ma35h0_ks_of_match,
	},
};

module_platform_driver(ma35h0_ks_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nuvoton, Inc");
MODULE_DESCRIPTION("Nuvoton MA35H0 Key Store Driver");
