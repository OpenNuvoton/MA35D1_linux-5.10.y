// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton MA35D1 Key Store
 *
 * Copyright (C) 2018-2019 Linaro Ltd.
 * Copyright (c) 2020 Nuvoton technology corporation.
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
#include <linux/tee_drv.h>

#include <uapi/misc/ma35d1_ks.h>

#define MISCDEV_NAME		"ksdev"

struct ma35d1_ks_dev {
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

#ifdef CONFIG_OPTEE
/*-------------------------------------------------------------------*/
/*  OP-TEE client driver for MA35D1 secure Key Store                */
/*-------------------------------------------------------------------*/
#define TEE_ERROR_KS_BUSY		0x00000001
#define TEE_ERROR_KS_FAIL		0x00000002
#define TEE_ERROR_KS_INVALID		0x00000003

/*
 * PTA_CMD_KS_INIT - Initialize Key Store
 *
 * param[0] unused
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 * TEE_ERROR_KS_FAIL - Initialization failed
 */
#define PTA_CMD_KS_INIT			0x1

/*
 * PTA_CMD_KS_READ - Read a Key Store key
 *
 * param[0] (in value) - value.a: 0: SRAM; 2: OTP
 *                       value.b: key number
 * param[1] (inout memref) - memref.size: word count of the key
 *                           memref.buffer: key buffer
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_INVALID - Incorrect input param
 * TEE_ERROR_KS_FAIL - Read operation failed
 */
#define PTA_CMD_KS_READ			0x2

/*
 * PTA_CMD_KS_WRITE - Write a Key Store key
 *
 * param[0] (in value) - value.a: 0: SRAM; 2: OTP
 *                       value.b: meta data
 * param[1] (inout memref) - memref.size: word count of the key
 *                           memref.buffer: key buffer
 * param[2] (out value) - value.a: key number
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_INVALID - Invalid parameter
 * TEE_ERROR_KS_FAIL - Write operation failed
 */
#define PTA_CMD_KS_WRITE		0x3

/*
 * PTA_CMD_KS_ERASE - Erase a Key Store key
 *
 * param[0] (in value) - value.a: 0: SRAM; 2: OTP
 *                       value.b: key number
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_INVALID - Incorrect input param
 * TEE_ERROR_KS_FAIL - Erase operation failed
 */
#define PTA_CMD_KS_ERASE		0x4

/*
 * PTA_CMD_KS_ERASE_ALL - Erase all Key Store SRAM keys
 *
 * param[0] unused
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_FAIL - Erase all operation failed
 */
#define PTA_CMD_KS_ERASE_ALL		0x5

/*
 * PTA_CMD_KS_REVOKE - Revoke a Key Store key
 *
 * param[0] (in value) - value.a: 0: SRAM; 2: OTP
 *                       value.b: key number
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_INVALID - Incorrect input param
 * TEE_ERROR_KS_FAIL - Revoke operation failed
 */
#define PTA_CMD_KS_REVOKE		0x6

/*
 * PTA_CMD_KS_REMAIN - Get the remaining size of Key Store SRAM
 *
 * param[0] (out value) - value.a: remaining size of SRAM
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_KS_FAIL - Get remain operation failed
 */
#define PTA_CMD_KS_REMAIN		0x7

#define KS_DATA_SHM_SZ			1024

/**
 * struct optee_ks_private - OP-TEE private data of MA35D1 Key Store
 * @dev:		OP-TEE based device.
 * @ctx:		OP-TEE context handler.
 * @session_id:		KS TA session identifier.
 * @ks_shm_pool:	Memory pool shared with OP-TEE MA35D1 KS
 * @miscdev:		Linux misc device data
 */
struct optee_ks_private {
	struct device *dev;
	struct tee_context *ctx;
	u32 session_id;
	struct tee_shm *ks_shm_pool;
	u8  *va_shm;
	struct miscdevice miscdev;
};

static struct optee_ks_private  pvt_data;

#define to_optee_ks_private(r) \
		container_of(r, struct optee_ks_private, miscdev)


static int optee_ks_read(struct optee_ks_private *ks_priv, void __user *arg)
{
	struct ks_read_args  r_args;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int   err;

	err = copy_from_user(&r_args, arg, sizeof(r_args));
	if (err)
		return -EFAULT;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_READ function of Trusted App */
	inv_arg.func = PTA_CMD_KS_READ;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

	param[0].u.value.a = r_args.type;
	param[0].u.value.b = r_args.key_idx;
	param[1].u.memref.shm = ks_priv->ks_shm_pool;
	param[1].u.memref.size = r_args.word_cnt;
	param[1].u.memref.shm_offs = 0;

	err = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_READ invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}

	memcpy((u8 *)r_args.key, ks_priv->va_shm, r_args.word_cnt * 4);
	return copy_to_user(arg, &r_args, sizeof(r_args));
}

static int optee_ks_write_sram(struct optee_ks_private *ks_priv,
			       void __user *arg)
{
	struct ks_write_args  w_args;
	int    sidx, key_wcnt;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int   err;

	err = copy_from_user(&w_args, arg, sizeof(w_args));
	if (err)
		return -EFAULT;

	/* Get size index */
	sidx = ((w_args.meta_data & KS_METADATA_SIZE_Msk) >>
		KS_METADATA_SIZE_Pos);

	key_wcnt = au8SRAMCntTbl[sidx];

	memcpy(ks_priv->va_shm, (u8 *)w_args.key, key_wcnt * 4);

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_WRITE function of Trusted App */
	inv_arg.func = PTA_CMD_KS_WRITE;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	param[0].u.value.a = KS_SRAM;
	param[0].u.value.b = w_args.meta_data | KS_TOMETAKEY(w_args.key_idx);
	param[1].u.memref.shm = ks_priv->ks_shm_pool;
	param[1].u.memref.size = key_wcnt;
	param[1].u.memref.shm_offs = 0;

	err = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_WRITE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return param[2].u.value.a;
}

static int optee_ks_write_otp(struct optee_ks_private *ks_priv,
			      void __user *arg)
{
	struct ks_write_args  w_args;
	int    sidx, key_wcnt;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int   err;

	err = copy_from_user(&w_args, arg, sizeof(w_args));
	if (err)
		return -EFAULT;

	/* Get size index */
	sidx = ((w_args.meta_data & KS_METADATA_SIZE_Msk) >>
		KS_METADATA_SIZE_Pos);

	/* OTP only support maximum 256 bits */
	if (sidx >= 7)
		return -EINVAL;

	key_wcnt = au8OTPCntTbl[sidx];

	memcpy(ks_priv->va_shm, (u8 *)w_args.key, key_wcnt * 4);

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_WRITE function of Trusted App */
	inv_arg.func = PTA_CMD_KS_WRITE;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	param[0].u.value.a = KS_OTP;
	param[0].u.value.b = w_args.meta_data | KS_TOMETAKEY(w_args.key_idx);
	param[1].u.memref.shm = ks_priv->ks_shm_pool;
	param[1].u.memref.size = key_wcnt;
	param[1].u.memref.shm_offs = 0;

	err = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_WRITE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return 0;
}

static int optee_ks_erase(struct optee_ks_private *ks_priv, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_ERASE function of Trusted App */
	inv_arg.func = PTA_CMD_KS_ERASE;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

	param[0].u.value.a = k_args.type;
	param[0].u.value.b = k_args.key_idx;

	ret = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_ERASE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return 0;
}

static int optee_ks_erase_all(struct optee_ks_private *ks_priv)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_ERASE_ALL function of Trusted App */
	inv_arg.func = PTA_CMD_KS_ERASE_ALL;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	ret = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_ERASE_ALL invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return 0;
}

static int optee_ks_revoke(struct optee_ks_private *ks_priv, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_REVOKE function of Trusted App */
	inv_arg.func = PTA_CMD_KS_REVOKE;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

	param[0].u.value.a = k_args.type;
	param[0].u.value.b = k_args.key_idx;

	ret = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_REVOKE invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return 0;
}

static int optee_ks_remain(struct optee_ks_private *ks_priv)
{
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_REMAIN function of Trusted App */
	inv_arg.func = PTA_CMD_KS_REMAIN;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_REMAIN invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return param[0].u.value.a;
}

static int optee_ks_dev_open(struct inode *iptr, struct file *fptr)
{
	struct optee_ks_private  *ks_priv;
	int ret = 0;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	ks_priv = container_of(fptr->private_data, struct optee_ks_private,
			       miscdev);

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_KS_INIT function of Trusted App */
	inv_arg.func = PTA_CMD_KS_INIT;
	inv_arg.session = ks_priv->session_id;
	inv_arg.num_params = 4;

	ret = tee_client_invoke_func(ks_priv->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(ks_priv->dev, "PTA_CMD_KS_INIT invoke err: %x\n",
			inv_arg.ret);
		return -EINVAL;
	}
	return 0;
}

static int optee_ks_dev_release(struct inode *iptr, struct file *fptr)
{
	return 0;
}

static long optee_ks_dev_ioctl(struct file *fptr, unsigned int cmd,
				 unsigned long data)
{
	struct optee_ks_private  *ks_priv;
	char __user *argp = (char __user *)data;
	int rval = -EINVAL;

	ks_priv = container_of(fptr->private_data, struct optee_ks_private,
			       miscdev);

	if (_IOC_TYPE(cmd) != MA35D1_KS_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case NU_KS_IOCTL_READ:
		rval = optee_ks_read(ks_priv, argp);
		break;
	case NU_KS_IOCTL_WRITE_SRAM:
		rval = optee_ks_write_sram(ks_priv, argp);
		break;
	case NU_KS_IOCTL_WRITE_OTP:
		rval = optee_ks_write_otp(ks_priv, argp);
		break;
	case NU_KS_IOCTL_ERASE:
		rval = optee_ks_erase(ks_priv, argp);
		break;
	case NU_KS_IOCTL_ERASE_ALL:
		rval = optee_ks_erase_all(ks_priv);
		break;
	case NU_KS_IOCTL_REVOKE:
		rval = optee_ks_revoke(ks_priv, argp);
		break;
	case NU_KS_IOCTL_GET_REMAIN:
		rval = optee_ks_remain(ks_priv);
		break;
	default:
		/* Should not get here */
		break;
	}
	return rval;
}

static const struct file_operations optee_ks_fops = {
	.owner = THIS_MODULE,
	.open = optee_ks_dev_open,
	.release = optee_ks_dev_release,
	.unlocked_ioctl = optee_ks_dev_ioctl,
	.compat_ioctl = optee_ks_dev_ioctl,
};

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (ver->impl_id == TEE_IMPL_ID_OPTEE)
		return 1;
	else
		return 0;
}

static int optee_ks_probe(struct device *dev)
{
	struct tee_client_device *ks_device = to_tee_client_device(dev);
	int ret = 0, err = -ENODEV;
	struct tee_ioctl_open_session_arg sess_arg;

	memset(&sess_arg, 0, sizeof(sess_arg));

	pvt_data.dev = dev;

	/*
	 * Open context with TEE driver
	 */
	pvt_data.ctx = tee_client_open_context(NULL, optee_ctx_match,
					       NULL, NULL);
	if (IS_ERR(pvt_data.ctx)) {
		dev_err(dev, "tee_client_open_context failed, err: %x\n",
			sess_arg.ret);
		return err;
	}

	/*
	 * Open session with KS Trusted App
	 */
	memcpy(sess_arg.uuid, ks_device->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	ret = tee_client_open_session(pvt_data.ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	pvt_data.session_id = sess_arg.session;

	/*
	 * Allocate KS buffer from OP-TEE share memory
	 */
	pvt_data.ks_shm_pool = tee_shm_alloc(pvt_data.ctx, KS_DATA_SHM_SZ,
					 TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(pvt_data.ks_shm_pool)) {
		dev_err(dev, "tee_shm_alloc failed\n");
		goto out_sess;
	}
	pvt_data.va_shm = tee_shm_get_va(pvt_data.ks_shm_pool, 0);
	if (IS_ERR(pvt_data.va_shm)) {
		dev_err(dev, "tee_shm_get_va failed\n");
		goto out_sess;
	}

	/*
	 * Register optee KS device as a misc device
	 */
	pvt_data.miscdev.minor = MISC_DYNAMIC_MINOR;
	pvt_data.miscdev.name = ks_miscdev_name;
	pvt_data.miscdev.fops = &optee_ks_fops;
	pvt_data.miscdev.parent = dev;
	err = misc_register(&pvt_data.miscdev);
	if (err) {
		dev_err(dev, "error:%d. Unable to register device", err);
		goto out_sess;
	}
	return 0;

out_sess:
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
out_ctx:
	tee_client_close_context(pvt_data.ctx);
	return err;
}

static int optee_ks_remove(struct device *dev)
{
	misc_deregister(&pvt_data.miscdev);
	tee_shm_free(pvt_data.ks_shm_pool);
	tee_client_close_session(pvt_data.ctx, pvt_data.session_id);
	tee_client_close_context(pvt_data.ctx);
	return 0;
}

static const struct tee_client_device_id optee_ks_id_table[] = {
	{UUID_INIT(0xaac83d50, 0xc303, 0x41ee,
		   0xb8, 0xf2, 0x70, 0x6c, 0x0b, 0x78, 0xe5, 0xad)},
	{}
};

MODULE_DEVICE_TABLE(tee, optee_ks_id_table);

static struct tee_client_driver optee_ks_driver = {
	.id_table	= optee_ks_id_table,
	.driver		= {
		.name		= "optee-nvt-ks",
		.bus		= &tee_bus_type,
		.probe		= optee_ks_probe,
		.remove		= optee_ks_remove,
	},
};
#endif

/*-------------------------------------------------------------------*/
/*  Key Store driver for MA35D1 Key Store                           */
/*-------------------------------------------------------------------*/

static inline u32  nu_ks_read_reg(struct ma35d1_ks_dev *ks_dev, u32 offset)
{
	u32 value = readl_relaxed(ks_dev->reg_base + offset);

	dev_vdbg(ks_dev->dev, "reg read 0x%08x from <0x%x>\n", value, offset);
	return value;
}

static inline void nu_ks_write_reg(struct ma35d1_ks_dev *ks_dev,
				   u32 offset, u32 value)
{
	dev_vdbg(ks_dev->dev, "write 0x%08x into <0x%x>\n", value, offset);
	writel_relaxed(value, ks_dev->reg_base + offset);
}


static int ma35d1_ks_read(struct ma35d1_ks_dev *ks_dev, void __user *arg)
{
	struct ks_read_args  r_args;
	int         err, remain_cnt;
	u32         cont_msk;
	int         offset, i, cnt;

	err = copy_from_user(&r_args, arg, sizeof(r_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
		while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
			;

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

static int ma35d1_ks_write_sram(struct ma35d1_ks_dev *ks_dev,
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
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
		while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
			;

	} while (remain_cnt > 0);

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on writing SRAM keys!\n");
		return -EIO;
	}

	return KS_TOKEYIDX(nu_ks_read_reg(ks_dev, KS_METADATA));
}

static int ma35d1_ks_write_otp(struct ma35d1_ks_dev *ks_dev, void __user *arg)
{
	struct ks_write_args  w_args;
	int         err, remain_cnt;
	u32         cont_msk;
	int         sidx, offset, i, cnt;

	err = copy_from_user(&w_args, arg, sizeof(w_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
		while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
			;

	} while (remain_cnt > 0);

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on writing OTP keys!\n");
		return -EIO;
	}
	return 0;
}

static int ma35d1_ks_erase(struct ma35d1_ks_dev *ks_dev, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
	while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
		;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on erasing a key!\n");
		return -EIO;
	}
	return 0;
}

static int ma35d1_ks_erase_all(struct ma35d1_ks_dev *ks_dev)
{
	/* Just return when key store is in busy */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
	while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
		;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on erase-all!\n");
		return -EIO;
	}
	return 0;
}

static int ma35d1_ks_revoke(struct ma35d1_ks_dev *ks_dev, void __user *arg)
{
	struct ks_kidx_args  k_args;
	int   err;

	err = copy_from_user(&k_args, arg, sizeof(k_args));
	if (err)
		return -EFAULT;

	/* Just return when key store is in busy */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY) {
		pr_err("MA35D1 KS is busy!\n");
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
	while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
		;

	/* Check error flag */
	if (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_EIF) {
		pr_err("KS EIF set on revoking a key!\n");
		return -EIO;
	}
	return 0;
}

static int ma35d1_ks_remain(struct ma35d1_ks_dev *ks_dev)
{
	u32	reg_data, sram_remain;

	reg_data = nu_ks_read_reg(ks_dev, KS_REMAIN);
	sram_remain = (reg_data & KS_REMAIN_RRMNG_Msk) >> KS_REMAIN_RRMNG_Pos;

	return sram_remain;
}

static int ks_dev_open(struct inode *iptr, struct file *fptr)
{
	struct ma35d1_ks_dev *ks_dev;

	ks_dev = container_of(fptr->private_data, struct ma35d1_ks_dev,
			      miscdev);

	/* Start Key Store Initial */
	nu_ks_write_reg(ks_dev, KS_CTL, KS_CTL_INIT | KS_CTL_START);

	/* Waiting for initilization */
	while ((nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_INITDONE) == 0)
		;

	/* Waiting for processing */
	while (nu_ks_read_reg(ks_dev, KS_STS) & KS_STS_BUSY)
		;

	return 0;
}

static int ks_dev_release(struct inode *iptr, struct file *fptr)
{
	return 0;
}

static long ks_dev_ioctl(struct file *fptr, unsigned int cmd,
				 unsigned long data)
{
	struct ma35d1_ks_dev *ks_dev;
	char __user *argp = (char __user *)data;
	int rval = -EINVAL;

	ks_dev = container_of(fptr->private_data, struct ma35d1_ks_dev,
			      miscdev);

	if (_IOC_TYPE(cmd) != MA35D1_KS_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case NU_KS_IOCTL_READ:
		rval = ma35d1_ks_read(ks_dev, argp);
		break;
	case NU_KS_IOCTL_WRITE_SRAM:
		rval = ma35d1_ks_write_sram(ks_dev, argp);
		break;
	case NU_KS_IOCTL_WRITE_OTP:
		rval = ma35d1_ks_write_otp(ks_dev, argp);
		break;
	case NU_KS_IOCTL_ERASE:
		rval = ma35d1_ks_erase(ks_dev, argp);
		break;
	case NU_KS_IOCTL_ERASE_ALL:
		rval = ma35d1_ks_erase_all(ks_dev);
		break;
	case NU_KS_IOCTL_REVOKE:
		rval = ma35d1_ks_revoke(ks_dev, argp);
		break;
	case NU_KS_IOCTL_GET_REMAIN:
		rval = ma35d1_ks_remain(ks_dev);
		break;
	default:
		/* Should not get here */
		break;
	}
	return rval;
}

static const struct file_operations ma35d1_ks_fops = {
	.owner = THIS_MODULE,
	.open = ks_dev_open,
	.release = ks_dev_release,
	.unlocked_ioctl = ks_dev_ioctl,
	.compat_ioctl = ks_dev_ioctl,
};


static int ma35d1_ks_probe(struct platform_device *pdev)
{
	struct ma35d1_ks_dev	*ks_dev;
	struct device		*dev;
	struct resource		*res;
	int err;
#ifdef CONFIG_OPTEE
	bool  optee_nuvoton = false;
	const char  *optee_sel;
#endif
	dev = &pdev->dev;

#ifdef CONFIG_OPTEE
	if (!of_property_read_string(dev->of_node, "optee_nuvoton",
	    &optee_sel)) {
		if (!strcmp("yes", optee_sel))
			optee_nuvoton = true;
	}
	if (optee_nuvoton) {
		pr_info("Register MA35D1 KS optee client driver.\n");
		return driver_register(&optee_ks_driver.driver);
	}
#endif

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
	ks_dev->miscdev.fops = &ma35d1_ks_fops;
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

static int ma35d1_ks_remove(struct platform_device *pdev)
{
	struct ma35d1_ks_dev *ks_dev;

	ks_dev = platform_get_drvdata(pdev);
	misc_deregister(&ks_dev->miscdev);
	return 0;
}

static const struct of_device_id ma35d1_ks_of_match[] = {
	{
		.compatible = "nuvoton,ma35d1-ks",
	},
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, ma35d1_ks_of_match);

static struct platform_driver ma35d1_ks_driver = {
	.probe = ma35d1_ks_probe,
	.remove =  ma35d1_ks_remove,
	.driver = {
		.name = "nuvoton-ks",
		.of_match_table = ma35d1_ks_of_match,
	},
};

module_platform_driver(ma35d1_ks_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nuvoton, Inc");
MODULE_DESCRIPTION("Nuvoton MA35D1 Key Store Driver");
