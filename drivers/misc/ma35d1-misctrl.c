// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Nuvoton Technology Corp.
 */

#include <linux/arm-smccc.h>
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
#include <soc/nuvoton/ma35d1_sip.h>

#define SET_CPU_FREQ_500M	0x1005
#define SET_CPU_FREQ_800M	0x1008
#define SET_CPU_FREQ_1000M	0x1010
#define GET_PMIC_VOLT		0x1101
#define SET_PMIC_VOLT		0x1102
#define SET_EPLL_DIV_BY_2	0x1202
#define SET_EPLL_DIV_BY_4	0x1204
#define SET_EPLL_DIV_BY_8	0x1208
#define SET_EPLL_RESTORE	0x120F
#define SET_SYS_SPD_LOW		0x1301
#define SET_SYS_SPD_RESTORE	0x1302

struct ma35d1_misctrl {
	int minor;
	u32 mode;
};

static struct ma35d1_misctrl *misctrl;

static int ma35d1_misctrl_open(struct inode *inode, struct file *filp)
{
	filp->private_data = misctrl;

	return 0;
}

static long ma35d1_misctrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct arm_smccc_res res;

	switch(cmd) {
	case SET_CPU_FREQ_500M:
		arm_smccc_smc(MA35D1_SIP_CPU_CLK, 500, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_CPU_FREQ_800M:
		arm_smccc_smc(MA35D1_SIP_CPU_CLK, 800, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_CPU_FREQ_1000M:
		arm_smccc_smc(MA35D1_SIP_CPU_CLK, 1000, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case GET_PMIC_VOLT:
		arm_smccc_smc(MA35D1_SIP_PMIC, 0, 0, 0, 0, 0,
			      0, 0, &res);
		if (res.a0 != 0)
			return -EIO;
		return res.a1;

	case SET_PMIC_VOLT:
		arm_smccc_smc(MA35D1_SIP_PMIC, arg, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_EPLL_DIV_BY_2:
		arm_smccc_smc(MA35D1_SIP_EPLL, MA35d1_SIP_EPLL_DIV_2, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_EPLL_DIV_BY_4:
		arm_smccc_smc(MA35D1_SIP_EPLL, MA35d1_SIP_EPLL_DIV_4, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_EPLL_DIV_BY_8:
		arm_smccc_smc(MA35D1_SIP_EPLL, MA35d1_SIP_EPLL_DIV_8, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_EPLL_RESTORE:
		arm_smccc_smc(MA35D1_SIP_EPLL, MA35d1_SIP_EPLL_RESTORE, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_SYS_SPD_LOW:
		arm_smccc_smc(MA35D1_SIP_LSPD, 1, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	case SET_SYS_SPD_RESTORE:
		arm_smccc_smc(MA35D1_SIP_LSPD, 0, 0, 0, 0, 0,
			      0, 0, &res);
		break;

	default:
		return -EINVAL;
	}
	return res.a0;
}

struct file_operations misctrl_fops = {
	.owner  = THIS_MODULE,
	.open   = ma35d1_misctrl_open,
	.unlocked_ioctl = ma35d1_misctrl_ioctl,
};

static struct miscdevice misctrl_dev[] = {
	[0] = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = "ma35_misctrl",
		.fops  = &misctrl_fops,
	},
};

static int ma35d1_misctrl_probe(struct platform_device *pdev)
{
	int ret;

	misctrl = devm_kzalloc(&pdev->dev, sizeof(struct ma35d1_misctrl), GFP_KERNEL);
	if (misctrl == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory for ebi device\n");
		return -ENOMEM;
	}

	ret = misc_register(&misctrl_dev[0]);
	if (ret) {
		dev_err(&pdev->dev, "misc registration failed\n");
		return ret;
	}

	misctrl->minor = MINOR(misctrl_dev[0].minor);

	platform_set_drvdata(pdev, misctrl);

	return 0;
}

static int ma35d1_misctrl_remove(struct platform_device *pdev)
{
	misc_deregister(&misctrl_dev[0]);

	return 0;
}


static const struct of_device_id ma35d1_misctrl_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-misctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_misctrl_of_match);

static struct platform_driver ma35d1_misctrl_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name = "ma35d1-misctrl",
		.of_match_table = of_match_ptr(ma35d1_misctrl_of_match),
	},
	.probe   = ma35d1_misctrl_probe,
	.remove  = ma35d1_misctrl_remove,
};

module_platform_driver(ma35d1_misctrl_driver);

MODULE_ALIAS("platform:ma35d1-misctrl");
MODULE_DESCRIPTION("misctrl driver for Nuvoton MA35D1");
MODULE_LICENSE("GPL v2");



