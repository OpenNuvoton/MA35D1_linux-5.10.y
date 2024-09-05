// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-aes.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 * Some ideas are from atmel-aes.c and mtk-aes.c driver.
 *
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/tee_drv.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <crypto/aes.h>
#include <crypto/gcm.h>
#include <crypto/internal/skcipher.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/internal/aead.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "nuvoton-crypto.h"

#define SUPPORT_SM4

#define AES_QUEUE_LENGTH	8
#define AES_FLAGS_BUSY		BIT(1)


static u8  g_zeros[16] = { 0 };

static int nuvoton_aes_dma_cascade(struct nu_aes_dev *dd, int err);

struct nu_aes_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

static struct nu_aes_drv  nu_aes = {
	.dev_list = LIST_HEAD_INIT(nu_aes.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(nu_aes.lock),
};


static struct nu_aes_dev *nuvoton_aes_find_dev(struct nu_aes_base_ctx *ctx)
{
	struct nu_aes_dev *aes_dd = NULL;
	struct nu_aes_dev *tmp;

	spin_lock_bh(&nu_aes.lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &nu_aes.dev_list, list) {
			aes_dd = tmp;
			break;
		}
		ctx->dd = aes_dd;
	} else {
		aes_dd = ctx->dd;
	}
	spin_unlock_bh(&nu_aes.lock);

	return aes_dd;
}


static inline void nu_write_reg(struct nu_aes_dev *aes_dd, u32 val, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true)
		aes_dd->va_shm[reg/4] = val;
	else
		writel_relaxed(val, aes_dd->reg_base + reg);
#else
	writel_relaxed(val, aes_dd->reg_base + reg);
#endif
}

static inline u32 nu_read_reg(struct nu_aes_dev *aes_dd, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true)
		return aes_dd->va_shm[reg/4];
	else
		return readl_relaxed(aes_dd->reg_base + reg);
#else
	return readl_relaxed(aes_dd->reg_base + reg);
#endif
}

static int nuvoton_aes_sg_to_buffer(struct nu_aes_dev *dd, u8 *bptr,
				    int max_cnt)
{
	int	in_cnt, copy_len;

	in_cnt = 0;
	while (dd->in_sg && (dd->req_len > 0) && (in_cnt < max_cnt)) {
		copy_len = min((int)dd->in_sg->length - dd->in_sg_off,
				dd->req_len);
		if (copy_len + in_cnt > max_cnt)
			copy_len = max_cnt - in_cnt;

		memcpy(bptr + in_cnt, (u8 *)sg_virt(dd->in_sg) +
			dd->in_sg_off, copy_len);

		in_cnt += copy_len;
		dd->req_len -= copy_len;
		dd->in_sg_off += copy_len;

		if (dd->in_sg_off >= dd->in_sg->length) {
			dd->in_sg = sg_next(dd->in_sg);
			dd->in_sg_off = 0;
		}
	}
	return in_cnt;
}

static int nuvoton_aes_buffer_to_sg(struct nu_aes_dev *dd, u8 *bptr,
				    int max_cnt)
{
	int	copy_len, out_cnt = 0;

	while ((max_cnt > 0) && (dd->out_sg != NULL)) {
		copy_len = min((int)dd->out_sg->length - dd->out_sg_off,
			max_cnt);
		memcpy((u8 *)sg_virt(dd->out_sg) + dd->out_sg_off, bptr +
			out_cnt, copy_len);

		max_cnt -= copy_len;
		dd->out_sg_off += copy_len;
		out_cnt += copy_len;

		if (dd->out_sg_off >= dd->out_sg->length) {
			dd->out_sg = sg_next(dd->out_sg);
			dd->out_sg_off = 0;
		}
	}
	return out_cnt;
}

static int nuvoton_aes_get_output(struct nu_aes_dev *dd)
{
	struct nu_aes_base_ctx  *ctx = dd->ctx;
	int     retval;

	if ((ctx->mode & AES_CTL_OPMODE_MASK) == AES_MODE_GCM) {
		nuvoton_aes_buffer_to_sg(dd, dd->inbuf, ctx->assoclen);

		retval = nuvoton_aes_buffer_to_sg(dd, dd->outbuf,
					ctx->text_len);
		if ((retval % 16) != 0) {
			/* seek to block aligned position */
			retval += (16 - (retval % 16));
		}
		/* attach auth tag to end of ciphertext */
		nuvoton_aes_buffer_to_sg(dd, dd->outbuf + retval,
					ctx->authsize);
	} else if ((ctx->mode & AES_CTL_OPMODE_MASK) == AES_MODE_CCM) {
		nuvoton_aes_buffer_to_sg(dd, dd->inbuf, ctx->assoclen);

		retval = nuvoton_aes_buffer_to_sg(dd, dd->outbuf,
				ctx->text_len);

		if ((retval % 16) != 0) {
			/* seek to block aligned position */
			retval += (16 - (retval % 16));
		}

		if (ctx->mode & AES_CTL_ENCRPT) {
			/* attach auth tag to end of ciphertext */
			nuvoton_aes_buffer_to_sg(dd, dd->outbuf + retval,
					ctx->authsize);
		} else {
			/* check the decrypt output auth tag */
			if (memcmp(ctx->tag, dd->outbuf + retval,
					ctx->authsize) != 0) {
				pr_debug("CCM tag is wrong!\n");
				return -EBADMSG;
			}
			pr_debug("CMM check passed. %d, %d\n",
				retval, ctx->authsize);
		}
	} else {
		retval = nuvoton_aes_buffer_to_sg(dd, dd->outbuf, dd->dma_len);
		dd->dma_len = 0;
	}
	return 0;
}

static int nuvoton_aes_complete(struct nu_aes_dev *dd, int err)
{
	struct skcipher_request *req = skcipher_request_cast(dd->areq);
	u32	*ivec;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
#endif
	int	i;

	err = nuvoton_aes_get_output(dd);

	if ((req->iv) &&
	    ((dd->ctx->mode & AES_CTL_OPMODE_MASK) != AES_MODE_ECB)) {
		ivec = (u32 *)req->iv;
		for (i = 0; i < 4; i++)
			ivec[i] = nu_read_reg(dd, AES_FDBCK(i));
	}

#ifdef CONFIG_OPTEE
	if (dd->nu_cdev->use_optee == true) {
		/*
		 * Close the crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_CLOSE_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_CLOSE_SESSION;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;
		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

		param[0].u.value.a = C_CODE_AES;
		param[1].u.value.a = dd->crypto_session_id;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_CLOSE_SESSION err: %x\n", inv_arg.ret);
			dd->flags &= ~AES_FLAGS_BUSY;
			dd->areq->complete(dd->areq, err);
			/* Handle new request */
			tasklet_schedule(&dd->queue_task);
			return -EINVAL;
		}
	}
#endif
	dd->flags &= ~AES_FLAGS_BUSY;
	dd->areq->complete(dd->areq, err);
	/* Handle new request */
	tasklet_schedule(&dd->queue_task);
	return 0;
}

static int nuvoton_aes_dma_run(struct nu_aes_dev *dd, u32 cascade)
{
	struct device	*dev = dd->dev;
	u32     dma_ctl;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int	err;
#endif

	dd->dma_len += nuvoton_aes_sg_to_buffer(dd, dd->inbuf + dd->dma_len,
			AES_BUFF_SIZE - dd->dma_len);

	if (!dd->in_sg || (dd->req_len == 0)) {
		dd->resume = nuvoton_aes_complete;	/* no more data */
		dma_ctl = cascade | AES_CTL_DMALAST;
	} else {
		dd->resume = nuvoton_aes_dma_cascade;
		dma_ctl = cascade;
	}

	dd->dma_inbuf = dma_map_single(dev, dd->inbuf, AES_BUFF_SIZE,
				       DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, dd->dma_inbuf))) {
		dev_err(dev, "AES inbuf map error\n");
		return -EINVAL;
	}

	dd->dma_outbuf = dma_map_single(dev, dd->outbuf, AES_BUFF_SIZE,
					DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, dd->dma_outbuf))) {
		dma_unmap_single(dev, dd->dma_inbuf, AES_BUFF_SIZE,
				 DMA_TO_DEVICE);
		dev_err(dev, "AES outbuf map error\n");
		return -EINVAL;
	}

	/*
	 *  Execute AES encrypt/decrypt
	 */
	pr_debug("AES start dma_len = %d\n", dd->dma_len);
	nu_write_reg(dd, dd->dma_len, AES_CNT);
	nu_write_reg(dd, dd->dma_inbuf, AES_SADDR);
	nu_write_reg(dd, dd->dma_outbuf, AES_DADDR);

	// dump_AES_registers(dd);

	/* start AES */
	nu_write_reg(dd, (nu_read_reg(dd, AES_CTL) | dma_ctl |
		     AES_CTL_START), AES_CTL);

#ifdef CONFIG_OPTEE
	if (dd->nu_cdev->use_optee == false)
		return -EINPROGRESS;

	/*--------------------------------------------------------------*/
	/*  Invoke OP-TEE Crypto PTA to run AES                         */
	/*--------------------------------------------------------------*/
	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_CRYPTO_AES_RUN function of Trusted App */
	inv_arg.func = PTA_CMD_CRYPTO_AES_RUN;
	inv_arg.session = dd->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

	param[0].u.value.a = dd->crypto_session_id;
	param[0].u.value.b = nu_read_reg(dd, AES_KSCTL);
	param[1].u.memref.shm = dd->shm_pool;
	param[1].u.memref.size = CRYPTO_SHM_SIZE;
	param[1].u.memref.shm_offs = 0;
	err = tee_client_invoke_func(dd->octx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_CMD_CRYPTO_AES_RUN err: %x\n", inv_arg.ret);
		tasklet_schedule(&dd->done_task);
		return -EINVAL;
	}
	tasklet_schedule(&dd->done_task);
#endif
	return -EINPROGRESS;

}

static int nuvoton_aes_dma_cascade(struct nu_aes_dev *dd, int err)
{
	/* write AES engine DMA output data to out_sg */
	nuvoton_aes_get_output(dd);

	/* cascade AES DMA to process remaining data */
	return nuvoton_aes_dma_run(dd, AES_CTL_DMACSCAD);
}

static int nuvoton_aes_dma_start(struct nu_aes_dev *dd, int err)
{
	struct skcipher_request *req = skcipher_request_cast(dd->areq);
	struct nu_aes_base_ctx  *ctx = dd->ctx;
	u32	*iv = (u32 *)req->iv;
	int	i;

	if ((req->cryptlen == 0) || (req->src == NULL) ||	(req->dst == NULL))
		return nuvoton_aes_complete(dd, 0);  /* no data */

	dd->req_len = req->cryptlen;
	dd->in_sg = req->src;
	dd->out_sg = req->dst;
	dd->in_sg_off = 0;
	dd->out_sg_off = 0;
	dd->dma_len = 0;

	if (ctx->keylen == AES_KS_KEYLEN) {
		/* configure AES Key from Key Store */
		u8  *key = (u8 *)ctx->aes_key;

		nu_write_reg(dd, (key[1] << 7) | AES_KSCTL_RSRC | key[2],
				AES_KSCTL);
	} else {
		/* program AES key */
		nu_write_reg(dd, 0, AES_KSCTL);
		for (i = 0; i < ctx->keylen / 4; i++)
			nu_write_reg(dd, ctx->aes_key[i], AES_KEY(i));
	}

	/* program AES IV */
	if (iv) {
		for (i = 0; i < 4; i++)
			nu_write_reg(dd, iv[i], AES_IV(i));
	} else {
		for (i = 0; i < 4; i++)
			nu_write_reg(dd, 0, AES_IV(i));
	}

	nu_write_reg(dd, nu_read_reg(dd, INTEN) | (INTEN_AESIEN |
			INTEN_AESEIEN), INTEN);
	nu_write_reg(dd, 0, AES_CTL);
	nu_write_reg(dd, (INTSTS_AESIF | INTSTS_AESEIF), INTSTS);

	nu_write_reg(dd, 0, AES_GCM_IVCNT(0));
	nu_write_reg(dd, 0, AES_GCM_ACNT(0));
	nu_write_reg(dd, 0, AES_GCM_PCNT(0));

	nu_write_reg(dd, (ctx->keysz_sel | ctx->mode | AES_CTL_INSWAP |
			AES_CTL_OUTSWAP | AES_CTL_KINSWAP | AES_CTL_KOUTSWAP |
			AES_CTL_DMAEN), AES_CTL);

	pr_debug("[%s] - mode: 0x%08x, AES_CTL = 0x%x\n", __func__,
			ctx->mode, nu_read_reg(dd, AES_CTL));

	return nuvoton_aes_dma_run(dd, 0);
}

static int nuvoton_aes_handle_queue(struct nu_aes_dev *dd,
				    struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	struct nu_aes_base_ctx  *ctx;
	unsigned long  flags;
	int  ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&dd->queue, new_areq);
	if (dd->flags & AES_FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	areq = crypto_dequeue_request(&dd->queue);
	if (areq)
		dd->flags |= AES_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx = crypto_tfm_ctx(areq->tfm);

	dd->areq = areq;
	dd->ctx = ctx;
	return ctx->start(dd, 0);
}

static int nuvoton_aes_crypt(struct skcipher_request *req, u32 mode)
{
	struct nu_aes_base_ctx	*ctx = crypto_skcipher_ctx(
				crypto_skcipher_reqtfm(req));
	struct nu_aes_dev	*aes_dd;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int  err;
#endif

	aes_dd = nuvoton_aes_find_dev(ctx);
	if (!aes_dd)
		return -ENODEV;

#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true) {
		/*
		 * Open a crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_OPEN_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_OPEN_SESSION;
		inv_arg.session = aes_dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
		param[0].u.value.a = C_CODE_AES;

		err = tee_client_invoke_func(aes_dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_OPEN_SESSION err: %x\n", inv_arg.ret);
			return -EINVAL;
		}
		aes_dd->crypto_session_id = param[1].u.value.a;
	}
#endif
	ctx->mode = mode;

	return nuvoton_aes_handle_queue(aes_dd, &req->base);
}

static int nuvoton_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				 unsigned int keylen)
{
	struct nu_aes_base_ctx  *ctx = crypto_skcipher_ctx(tfm);

	switch (keylen) {
	case AES_KEYSIZE_128:
		ctx->keysz_sel = AES_KEYSZ_SEL_128;
		break;

	case AES_KEYSIZE_192:
		ctx->keysz_sel = AES_KEYSZ_SEL_192;
		break;

	case AES_KEYSIZE_256:
		ctx->keysz_sel = AES_KEYSZ_SEL_256;
		break;

	case AES_KS_KEYLEN:
		/*
		 *  AES key is from H/W Key Store
		 *  key[0] select the key size
		 *         0: AES-128
		 *         1: AES-192
		 *         2: AES-256
		 *  key[1] select the Key Store storage type
		 *         0: Key Store SRAM
		 *         1: Key Store OTP
		 *  key[2] select the Key Store key number
		 */
		if ((key[0] > 2) || (key[1] > 1) || (key[2] > 31))
			return -EINVAL;
		ctx->keysz_sel = (key[0] << AES_CTL_KEYSZ_OFFSET);
		break;

	default:
		pr_err("[%s]: Unsupported keylen %d!\n", __func__, keylen);
		return -EINVAL;
	}
	ctx->keylen = keylen;
	memcpy(ctx->aes_key, key, keylen);
	return 0;
}

#ifdef CONFIG_OPTEE
static int  optee_aes_open(struct nu_aes_dev *dd)
{
	struct tee_ioctl_open_session_arg sess_arg;
	int   err;

	err = nuvoton_crypto_optee_init(dd->nu_cdev);
	if (err)
		return err;

	/*
	 * Open AES session with Crypto Trusted App
	 */
	memset(&sess_arg, 0, sizeof(sess_arg));
	memcpy(sess_arg.uuid, dd->nu_cdev->tee_cdev->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	err = tee_client_open_session(dd->octx, &sess_arg, NULL);
	if ((err < 0) || (sess_arg.ret != 0)) {
		pr_err("%s open session failed, err: %x\n", __func__, sess_arg.ret);
		return -EINVAL;
	}
	dd->session_id = sess_arg.session;

	return 0;
}

static void optee_aes_close(struct nu_aes_dev *dd)
{
	tee_client_close_session(dd->octx, dd->session_id);
}
#endif

static int nuvoton_aes_ecb_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_ECB | AES_CTL_ENCRPT);
}

static int nuvoton_aes_ecb_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_ECB);
}

static int nuvoton_aes_cbc_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CBC | AES_CTL_ENCRPT);
}

static int nuvoton_aes_cbc_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CBC);
}

static int nuvoton_aes_cfb_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CFB | AES_CTL_ENCRPT);
}

static int nuvoton_aes_cfb_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CFB);
}

static int nuvoton_aes_ofb_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_OFB | AES_CTL_ENCRPT);
}

static int nuvoton_aes_ofb_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_OFB);
}

static int nuvoton_aes_ctr_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CTR | AES_CTL_ENCRPT);
}

static int nuvoton_aes_ctr_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_MODE_CTR);
}

#ifdef SUPPORT_SM4
static int nuvoton_sm4_ecb_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN | AES_MODE_ECB |
					AES_CTL_ENCRPT);
}

static int nuvoton_sm4_ecb_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN | AES_MODE_ECB);
}

static int nuvoton_sm4_cbc_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN | AES_MODE_CBC |
					AES_CTL_ENCRPT);
}

static int nuvoton_sm4_cbc_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN | AES_MODE_CBC);
}

static int nuvoton_sm4_ctr_encrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN | AES_MODE_CTR |
					AES_CTL_ENCRPT);
}

static int nuvoton_sm4_ctr_decrypt(struct skcipher_request *req)
{
	return nuvoton_aes_crypt(req, AES_CTL_SM4EN|AES_MODE_CTR);
}
#endif

static int nuvoton_aes_cra_init(struct crypto_tfm *tfm)
{
	struct nu_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	struct nu_aes_dev  *aes_dd;

	// printk("AES: %s\n", tfm->__crt_alg->cra_driver_name);
	ctx->base.start = nuvoton_aes_dma_start;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (!aes_dd)
		return -ENODEV;

#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true)
		return optee_aes_open(aes_dd);
#endif
	return 0;
}

static void nuvoton_aes_cra_exit(struct crypto_tfm *tfm)
{
#ifdef CONFIG_OPTEE
	struct nu_aes_ctx *ctx = crypto_tfm_ctx(tfm);
	struct nu_aes_dev  *aes_dd;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (aes_dd) {
		if (aes_dd->nu_cdev->use_optee)
			optee_aes_close(aes_dd);
	}
#endif
}

static struct skcipher_alg nuvoton_aes_algs[] = {
{
	.base.cra_name		= "cbc(aes)",
	.base.cra_driver_name	= "nuvoton-cbc-aes",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_aes_cbc_encrypt,
	.decrypt		= nuvoton_aes_cbc_decrypt,
},
{
	.base.cra_name		= "ecb(aes)",
	.base.cra_driver_name	= "nuvoton-ecb-aes",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_aes_ecb_encrypt,
	.decrypt		= nuvoton_aes_ecb_decrypt,
},
{
	.base.cra_name		= "cfb(aes)",
	.base.cra_driver_name	= "nuvoton-cfb-aes",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_aes_cfb_encrypt,
	.decrypt		= nuvoton_aes_cfb_decrypt,
},
{
	.base.cra_name		= "ofb(aes)",
	.base.cra_driver_name	= "nuvoton-ofb-aes",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_aes_ofb_encrypt,
	.decrypt		= nuvoton_aes_ofb_decrypt,
},
{
	.base.cra_name		= "ctr(aes)",
	.base.cra_driver_name	= "nuvoton-ctr-aes",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= 1,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_aes_ctr_encrypt,
	.decrypt		= nuvoton_aes_ctr_decrypt,
},
#ifdef SUPPORT_SM4
{
	.base.cra_name		= "ecb(sm4)",
	.base.cra_driver_name	= "nuvoton-ecb-sm4",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_sm4_ecb_encrypt,
	.decrypt		= nuvoton_sm4_ecb_decrypt,
},
{
	.base.cra_name		= "cbc(sm4)",
	.base.cra_driver_name	= "nuvoton-cbc-sm4",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= AES_BLOCK_SIZE,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_sm4_cbc_encrypt,
	.decrypt		= nuvoton_sm4_cbc_decrypt,
},
{
	.base.cra_name		= "ctr(sm4)",
	.base.cra_driver_name	= "nuvoton-ctr-sm4",
	.base.cra_priority	= 400,
	.base.cra_flags		= CRYPTO_ALG_ASYNC,
	.base.cra_blocksize	= 1,
	.base.cra_ctxsize	= sizeof(struct nu_aes_ctx),
	.base.cra_alignmask	= 0xf,
	.base.cra_init		= nuvoton_aes_cra_init,
	.base.cra_exit		= nuvoton_aes_cra_exit,
	.base.cra_module	= THIS_MODULE,

	.min_keysize		= AES_MIN_KEY_SIZE,
	.max_keysize		= AES_MAX_KEY_SIZE,
	.ivsize			= AES_BLOCK_SIZE,
	.setkey			= nuvoton_aes_setkey,
	.encrypt		= nuvoton_sm4_ctr_encrypt,
	.decrypt		= nuvoton_sm4_ctr_decrypt,
},
#endif
};   /* nuvoton_aes_algs */

/*------------------------------------------------------------------------*/
/*                                                                        */
/*  AES GCM mode                                                          */
/*                                                                        */
/*------------------------------------------------------------------------*/

static int nuvoton_aes_gcm_dma_start(struct nu_aes_dev *dd, int err)
{
	struct aead_request *req = aead_request_cast(dd->areq);
	struct nu_aes_base_ctx  *ctx = dd->ctx;
	u32	key;
	int	i, len;

	pr_debug("[%s] - assoclen: %d, cryptlen: %d\n", __func__,
			req->assoclen, req->cryptlen);

	ctx->assoclen = req->assoclen;
	ctx->text_len = req->cryptlen;

	dd->req_len = req->assoclen + req->cryptlen;
	dd->in_sg = req->src;
	dd->out_sg = req->dst;
	dd->in_sg_off = 0;
	dd->out_sg_off = 0;
	dd->dma_len = 0;

	/* Initial vector is 96 bits */
	memcpy(&dd->inbuf[0], req->iv, 12);
	memcpy(&dd->inbuf[12], g_zeros, 4);  /* add padding 00 00 00 01 */
	dd->inbuf[15] = 0x1;
	dd->dma_len = 16;

	/*
	 *  Copy associate authenticated data
	 */
	if (req->assoclen) {
		dd->dma_len += nuvoton_aes_sg_to_buffer(dd, dd->inbuf +
				dd->dma_len, req->assoclen);

		/* padding to be AES block aligned */
		if ((req->assoclen % 16) != 0) {
			len = 16 - (req->assoclen % 16);
			memcpy(&dd->inbuf[dd->dma_len], g_zeros, len);
			dd->dma_len += len;
		}
	}

	/*
	 *  Copy text data
	 */
	if (req->cryptlen) {
		dd->dma_len += nuvoton_aes_sg_to_buffer(dd, dd->inbuf +
				dd->dma_len, req->cryptlen);

		/* padding to be AES block aligned */
		if ((req->cryptlen % 16) != 0) {
			len = 16 - (req->cryptlen % 16);
			memcpy(&dd->inbuf[dd->dma_len], g_zeros, len);
			dd->dma_len += len;
		}
	}

	if (ctx->keylen == AES_KS_KEYLEN) {
		/* configure AES Key from Key Store */
		u8  *key = (u8 *)ctx->aes_key;

		nu_write_reg(dd, (key[1] << 7) | AES_KSCTL_RSRC | key[2], AES_KSCTL);
	} else {
		/* program AES key */
		nu_write_reg(dd, 0, AES_KSCTL);
		for (i = 0; i < ctx->keylen/4; i++) {
			key = ctx->aes_key[i];
			key = ((key>>24)&0xff) | ((key>>8)&0xff00) |
				((key&0xff00)<<8) | (key<<24);
			nu_write_reg(dd, key, AES_KEY(i));
		}
	}

	/* clear AES IV registers */
	for (i = 0; i < 4; i++)
		nu_write_reg(dd, 0, AES_IV(i));

	nu_write_reg(dd, nu_read_reg(dd, INTEN) | (INTEN_AESIEN | INTEN_AESEIEN), INTEN);
	nu_write_reg(dd, 0, AES_CTL);
	nu_write_reg(dd, (INTSTS_AESIF | INTSTS_AESEIF), INTSTS);

	nu_write_reg(dd, 12, AES_GCM_IVCNT(0));
	nu_write_reg(dd, req->assoclen, AES_GCM_ACNT(0));
	nu_write_reg(dd, req->cryptlen, AES_GCM_PCNT(0));

	nu_write_reg(dd, (ctx->keysz_sel | ctx->mode | AES_CTL_INSWAP |
			AES_CTL_OUTSWAP | AES_CTL_KOUTSWAP | AES_CTL_DMAEN),
			AES_CTL);

	pr_debug("[%s] - mode: 0x%08x, AES_CTL = 0x%x\n", __func__,
			ctx->mode, nu_read_reg(dd, AES_CTL));

	return nuvoton_aes_dma_run(dd, 0);
}


static int nuvoton_aes_gcm_crypt(struct aead_request *req, u32 mode)
{
	struct nu_aes_base_ctx	*ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct nu_aes_dev	*aes_dd;

	aes_dd = nuvoton_aes_find_dev(ctx);
	if (!aes_dd)
		return -ENODEV;

	ctx->mode = AES_MODE_GCM | mode;

	return nuvoton_aes_handle_queue(aes_dd, &req->base);
}

static int nuvoton_aes_gcm_encrypt(struct aead_request *req)
{
	return nuvoton_aes_gcm_crypt(req, AES_CTL_ENCRPT);
}

static int nuvoton_aes_gcm_decrypt(struct aead_request *req)
{
	return nuvoton_aes_gcm_crypt(req, 0);
}

static int nuvoton_aes_gcm_setkey(struct crypto_aead *tfm,
				  const u8 *key, unsigned int keylen)
{
	struct nu_aes_base_ctx  *ctx = crypto_aead_ctx(tfm);

	switch (keylen) {
	case AES_KEYSIZE_128:
		ctx->keysz_sel = AES_KEYSZ_SEL_128;
		break;

	case AES_KEYSIZE_192:
		ctx->keysz_sel = AES_KEYSZ_SEL_192;
		break;

	case AES_KEYSIZE_256:
		ctx->keysz_sel = AES_KEYSZ_SEL_256;
		break;

	case AES_KS_KEYLEN:
		/*
		 *  AES key is from H/W Key Store
		 *  key[0] select the key size
		 *         0: AES-128
		 *         1: AES-192
		 *         2: AES-256
		 *  key[1] select the Key Store storage type
		 *         0: Key Store SRAM
		 *         1: Key Store OTP
		 *  key[2] select the Key Store key number
		 */
		if ((key[0] > 2) || (key[1] > 1) || (key[2] > 31))
			return -EINVAL;
		ctx->keysz_sel = (key[0] << AES_CTL_KEYSZ_OFFSET);
		break;

	default:
		pr_err("[%s]: Unsupported keylen %d!\n", __func__, keylen);
		return -EINVAL;
	}
	ctx->keylen = keylen;
	memcpy(ctx->aes_key, key, keylen);
	return 0;
}

static int nuvoton_aes_gcm_setauthsize(struct crypto_aead *aead, u32 authsize)
{
	struct nu_aes_base_ctx *ctx = crypto_aead_ctx(aead);

	ctx->authsize = authsize;
	return 0;
}

static int nuvoton_aes_gcm_init(struct crypto_aead *aead)
{
	struct nu_aes_ctx *ctx = crypto_aead_ctx(aead);
	struct nu_aes_dev  *aes_dd;

	ctx->base.start = nuvoton_aes_gcm_dma_start;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (!aes_dd)
		return -ENODEV;

#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true) {
		struct tee_ioctl_invoke_arg inv_arg;
		struct tee_param param[4];
		int  err;

		err = optee_aes_open(aes_dd);
		if (err != 0)
			return err;

		/*
		 * Open a crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_OPEN_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_OPEN_SESSION;
		inv_arg.session = aes_dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
		param[0].u.value.a = C_CODE_AES;

		err = tee_client_invoke_func(aes_dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_OPEN_SESSION err: %x\n", inv_arg.ret);
			optee_aes_close(aes_dd);
			return -EINVAL;
		}
		aes_dd->crypto_session_id = param[1].u.value.a;
	}
#endif
	return 0;
}

static void nuvoton_aes_gcm_exit(struct crypto_aead *aead)
{
#ifdef CONFIG_OPTEE
	struct nu_aes_ctx *ctx = crypto_aead_ctx(aead);
	struct nu_aes_dev  *aes_dd;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (aes_dd && (aes_dd->nu_cdev->use_optee)) {
		struct tee_ioctl_invoke_arg inv_arg;
		struct tee_param param[4];

		/*
		 * Close the crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_CLOSE_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_CLOSE_SESSION;
		inv_arg.session = aes_dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

		param[0].u.value.a = C_CODE_AES;
		param[1].u.value.a = aes_dd->crypto_session_id;

		tee_client_invoke_func(aes_dd->octx, &inv_arg, param);

		optee_aes_close(aes_dd);
	}
#endif
}

static struct aead_alg  nuvoton_aes_gcm_alg[] = {
{
	.setkey		= nuvoton_aes_gcm_setkey,
	.setauthsize	= nuvoton_aes_gcm_setauthsize,
	.encrypt	= nuvoton_aes_gcm_encrypt,
	.decrypt	= nuvoton_aes_gcm_decrypt,
	.init		= nuvoton_aes_gcm_init,
	.exit		= nuvoton_aes_gcm_exit,
	.ivsize		= GCM_AES_IV_SIZE,
	.maxauthsize	= AES_BLOCK_SIZE,
	.base = {
		.cra_name		= "gcm(aes)",
		.cra_driver_name	= "nuvoton-gcm-aes",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct nu_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
	},
},
};

/*-------------------------------------------------------------------------*/
/*                                                                         */
/*  AES CCM mode                                                           */
/*                                                                         */
/*-------------------------------------------------------------------------*/
static int nuvoton_aes_ccm_dma_start(struct nu_aes_dev *dd, int err)
{
	struct aead_request *req = aead_request_cast(dd->areq);
	struct nu_aes_base_ctx  *ctx = dd->ctx;
	u8	*b;
	int	i, q, nlen, plen, alen, len_left, use_len;
	int	p_blk_cnt;
	u8	ctr[16];

	pr_debug("[%s] - assoclen: %d, cryptlen: %d\n", __func__,
			req->assoclen, req->cryptlen);

	ctx->assoclen = req->assoclen;

	if (ctx->mode & AES_CTL_ENCRPT)
		ctx->text_len = req->cryptlen;
	else
		/* decrypt input contains auth tag */
		ctx->text_len = req->cryptlen - ctx->authsize;

	/* decrypt input contains auth tag */
	dd->req_len = req->assoclen + req->cryptlen;
	dd->in_sg = req->src;
	dd->out_sg = req->dst;
	dd->in_sg_off = 0;
	dd->out_sg_off = 0;
	dd->dma_len = 0;

	b = dd->inbuf;
	alen = ctx->assoclen;       /* length of associated data           */
	plen = ctx->text_len;       /* length of text                      */
	nlen = 16 - req->iv[0] - 2; /* nonce length                        */
	q = 15 - nlen;              /* n + q = 15; n is length of nounce   */

	/*-----------------------------------------------------------------*/
	/*  Block B0                                                       */
	/*       Formatting of the Control Information and the Nonce       */
	/*-----------------------------------------------------------------*/

	/* First block B_0:
	 * 0        .. 0        flags
	 * 1        .. iv_len   nonce (aka iv)
	 * iv_len+1 .. 15       length
	 *
	 * With flags as (bits):
	 * 7        0
	 * 6        add present?
	 * 5 .. 3   (t - 2) / 2
	 * 2 .. 0   q - 1
	 */
	b[0] = 0;
	b[0] |= ((alen > 0) ? 1 : 0) << 6;
	b[0] |= ((ctx->authsize - 2) / 2) << 3;
	b[0] |= q - 1;

	memcpy(b+1, req->iv+1, nlen);

	for (i = 0, len_left = plen; i < q; i++, len_left >>= 8)
		b[15-i] = (u8)(len_left & 0xFF);
	b += 16;

	/*-----------------------------------------------------------------*/
	/*    Formatting of the Associated Data                            */
	/*-----------------------------------------------------------------*/
	/*
	 * If there is associated data, update CBC-MAC with
	 * add_len, add, 0 (padding to a block boundary)
	 */
	if (alen > 0) {
		len_left = alen;
		memset(b, 0, 16);
		b[0] = (unsigned char)((alen >> 8) & 0xFF);
		b[1] = (unsigned char)(alen & 0xFF);

		use_len = (len_left < (16 - 2)) ? len_left : 16 - 2;
		nuvoton_aes_sg_to_buffer(dd, b + 2, use_len);

		len_left -= use_len;
		b += 16;

		while (len_left > 0) {
			use_len = len_left > 16 ? 16 : len_left;
			memset(b, 0, 16);
			nuvoton_aes_sg_to_buffer(dd, b, use_len);
			b += 16;
			len_left -= use_len;
		}
	}

	/*-----------------------------------------------------------------*/
	/*    Formatting of the payload                                    */
	/*-----------------------------------------------------------------*/
	p_blk_cnt = 0;
	if (plen > 0) {
		len_left = plen;
		while (len_left > 0) {
			/*
			 * fill payload by 16 bytes block
			 */
			use_len = len_left > 16 ? 16 : len_left;
			/* naturally padding zero if len_left < 16 */
			memset(b, 0, 16);
			nuvoton_aes_sg_to_buffer(dd, b, use_len);
			b += 16;
			len_left -= use_len;
			p_blk_cnt++;
		}
	}

	/*
	 *  In decrypt case, get the tag from tail of input cipher text.
	 *  We should compare this tag with the CCM decrypt output tag.
	 */
	if (!(ctx->mode & AES_CTL_ENCRPT)) {
		q = nuvoton_aes_sg_to_buffer(dd, ctx->tag,
				((ctx->authsize > 16) ? 16 : ctx->authsize));
		pr_debug("[%s] - CCM read tag return length: %d/%d\n",
				__func__, q, ctx->authsize);
	}

	dd->dma_len = b - dd->inbuf;

	/*-----------------------------------------------------------------*/
	/*  Block Ctr0                                                     */
	/*       Formatting of the first Counter Block                     */
	/*-----------------------------------------------------------------*/
	memset(ctr, 0, 16);
	q = 15 - nlen;     /* n + q = 15; n is length of nounce, i.e. Nlen */
	ctr[0] = (q - 1) & 0x7;
	memcpy(ctr+1, req->iv+1, nlen);

	/* program AES IV registers */
	for (i = 0; i < 4; i++)
		nu_write_reg(dd, *((u32 *)&ctr[i*4]), AES_IV(i));

	if (ctx->keylen == AES_KS_KEYLEN) {
		/* configure AES Key from Key Store */
		u8  *key = (u8 *)ctx->aes_key;

		nu_write_reg(dd, (key[1] << 7) | AES_KSCTL_RSRC | key[2], AES_KSCTL);
	} else {
		/* program AES key */
		nu_write_reg(dd, 0, AES_KSCTL);
		for (i = 0; i < ctx->keylen/4; i++)
			nu_write_reg(dd, ctx->aes_key[i], AES_KEY(i));
	}

	nu_write_reg(dd, nu_read_reg(dd, INTEN) | (INTEN_AESIEN | INTEN_AESEIEN), INTEN);
	nu_write_reg(dd, 0, AES_CTL);
	nu_write_reg(dd, (INTSTS_AESIF | INTSTS_AESEIF), INTSTS);

	nu_write_reg(dd, 0, AES_GCM_IVCNT(0));
	nu_write_reg(dd, (dd->dma_len - (p_blk_cnt * 16)), AES_GCM_ACNT(0));
	nu_write_reg(dd, ctx->text_len, AES_GCM_PCNT(0));

	nu_write_reg(dd, (ctx->keysz_sel | ctx->mode | AES_CTL_INSWAP |
			AES_CTL_OUTSWAP | AES_CTL_KINSWAP | AES_CTL_KOUTSWAP |
			AES_CTL_DMAEN), AES_CTL);

	pr_debug("[%s] - mode: 0x%08x, AES_CTL = 0x%x\n", __func__, ctx->mode,
		 nu_read_reg(dd, AES_CTL));

	return nuvoton_aes_dma_run(dd, 0);
}


static int nuvoton_aes_ccm_crypt(struct aead_request *req, u32 mode)
{
	struct nu_aes_base_ctx	*ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));
	struct nu_aes_dev	*aes_dd;

	aes_dd = nuvoton_aes_find_dev(ctx);
	if (!aes_dd)
		return -ENODEV;

	ctx->mode = AES_MODE_CCM | mode;

	return nuvoton_aes_handle_queue(aes_dd, &req->base);
}

static int nuvoton_aes_ccm_encrypt(struct aead_request *req)
{
	return nuvoton_aes_ccm_crypt(req, AES_CTL_ENCRPT);
}

static int nuvoton_aes_ccm_decrypt(struct aead_request *req)
{
	return nuvoton_aes_ccm_crypt(req, 0);
}

static int nuvoton_aes_ccm_setauthsize(struct crypto_aead *aead, u32 authsize)
{
	struct nu_aes_base_ctx *ctx = crypto_aead_ctx(aead);

	pr_debug("[%s] - authsize = %d\n", __func__, authsize);
	ctx->authsize = authsize;
	return 0;
}

static int nuvoton_aes_ccm_init(struct crypto_aead *aead)
{
	struct nu_aes_ctx *ctx = crypto_aead_ctx(aead);
	struct nu_aes_dev  *aes_dd;

	ctx->base.start = nuvoton_aes_ccm_dma_start;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (!aes_dd)
		return -ENODEV;

#ifdef CONFIG_OPTEE
	if (aes_dd->nu_cdev->use_optee == true) {
		struct tee_ioctl_invoke_arg inv_arg;
		struct tee_param param[4];
		int  err;

		err = optee_aes_open(aes_dd);
		if (err != 0)
			return err;

		/*
		 * Open a crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_OPEN_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_OPEN_SESSION;
		inv_arg.session = aes_dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
		param[0].u.value.a = C_CODE_AES;

		err = tee_client_invoke_func(aes_dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_OPEN_SESSION err: %x\n", inv_arg.ret);
			optee_aes_close(aes_dd);
			return -EINVAL;
		}
		aes_dd->crypto_session_id = param[1].u.value.a;
	}
#endif
	return 0;
}

static void nuvoton_aes_ccm_exit(struct crypto_aead *aead)
{
#ifdef CONFIG_OPTEE
	struct nu_aes_ctx *ctx = crypto_aead_ctx(aead);
	struct nu_aes_dev  *aes_dd;

	aes_dd = nuvoton_aes_find_dev(&ctx->base);
	if (aes_dd && (aes_dd->nu_cdev->use_optee)) {
		struct tee_ioctl_invoke_arg inv_arg;
		struct tee_param param[4];

		/*
		 * Close the crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_CLOSE_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_CLOSE_SESSION;
		inv_arg.session = aes_dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

		param[0].u.value.a = C_CODE_AES;
		param[1].u.value.a = aes_dd->crypto_session_id;

		tee_client_invoke_func(aes_dd->octx, &inv_arg, param);

		optee_aes_close(aes_dd);
	}
#endif
}

static struct aead_alg  nuvoton_aes_ccm_alg[] = {
{
	.setkey		= nuvoton_aes_gcm_setkey,  /* same routine as GCM */
	.setauthsize	= nuvoton_aes_ccm_setauthsize,
	.encrypt	= nuvoton_aes_ccm_encrypt,
	.decrypt	= nuvoton_aes_ccm_decrypt,
	.init		= nuvoton_aes_ccm_init,
	.exit		= nuvoton_aes_ccm_exit,
	.ivsize		= AES_BLOCK_SIZE,
	.maxauthsize	= AES_BLOCK_SIZE,
	.base = {
		.cra_name		= "ccm(aes)",
		.cra_driver_name	= "nuvoton-ccm-aes",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct nu_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
	},
},
};

static void nuvoton_aes_queue_task(unsigned long data)
{
	struct nu_aes_dev *dd = (struct nu_aes_dev *)data;

	nuvoton_aes_handle_queue(dd, NULL);
}

static void nuvoton_aes_done_task(unsigned long data)
{
	struct nu_aes_dev *dd = (struct nu_aes_dev *)data;

	dma_unmap_single(dd->dev, dd->dma_inbuf, AES_BUFF_SIZE, DMA_TO_DEVICE);
	dma_unmap_single(dd->dev, dd->dma_outbuf, AES_BUFF_SIZE, DMA_FROM_DEVICE);

	(void)dd->resume(dd, 0);
}

static int nuvoton_register_gcm_ccm(struct device *dev)
{
	int i, err;

	/*
	 *  Register AES GCM algorithms
	 */
	err = crypto_register_aeads(nuvoton_aes_gcm_alg, ARRAY_SIZE(nuvoton_aes_gcm_alg));
	if (err) {
		for (i = 0; i < ARRAY_SIZE(nuvoton_aes_algs); i++)
			crypto_unregister_skcipher(&nuvoton_aes_algs[i]);
		dev_err(dev, "Could not register nuvoton_aes_gcm_algs!\n");
		return err;
	}

	/*
	 *  Register AES CCM algorithms
	 */
	err = crypto_register_aeads(nuvoton_aes_ccm_alg, ARRAY_SIZE(nuvoton_aes_ccm_alg));
	if (err) {
		for (i = 0; i < ARRAY_SIZE(nuvoton_aes_algs); i++)
			crypto_unregister_skcipher(&nuvoton_aes_algs[i]);
		crypto_unregister_aeads(nuvoton_aes_gcm_alg, ARRAY_SIZE(nuvoton_aes_gcm_alg));
		dev_err(dev, "Could not register nuvoton_aes_ccm_algs!\n");
		return err;
	}
	return 0;
}

int nuvoton_aes_probe(struct device *dev,
		      struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_aes_dev *aes_dd = &nu_cryp_dev->aes_dd;
	int i, err;

	aes_dd->dev = dev;
	aes_dd->nu_cdev = nu_cryp_dev;
	aes_dd->reg_base = nu_cryp_dev->reg_base;
	aes_dd->octx = NULL;

#ifdef CONFIG_OPTEE
	if (nu_cryp_dev->use_optee == true) {
		err = nuvoton_crypto_optee_init(nu_cryp_dev);
		if (err)
			return err;

		/*
	 	 * Open AES context with TEE driver
	 	 */
		aes_dd->octx = tee_client_open_context(NULL, optee_ctx_match,
					       NULL, NULL);
		if (IS_ERR(aes_dd->octx)) {
			pr_err("%s open context failed!\n", __func__);
			return -EINVAL;
		}

		/*
	 	 * Allocate handshake buffer from OP-TEE share memory
	 	 */
		aes_dd->shm_pool = tee_shm_alloc(aes_dd->octx, CRYPTO_SHM_SIZE,
						 TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(aes_dd->shm_pool)) {
			pr_err("%s tee_shm_alloc failed\n", __func__);
			return -EINVAL;
		}

		aes_dd->va_shm = tee_shm_get_va(aes_dd->shm_pool, 0);
		if (IS_ERR(aes_dd->va_shm)) {
			tee_shm_free(aes_dd->shm_pool);
			pr_err("%s tee_shm_get_va failed\n", __func__);
			return -EINVAL;
		}
	}
#endif

	INIT_LIST_HEAD(&aes_dd->list);
	spin_lock_init(&aes_dd->lock);

	tasklet_init(&aes_dd->done_task, nuvoton_aes_done_task, (unsigned long)aes_dd);
	tasklet_init(&aes_dd->queue_task, nuvoton_aes_queue_task, (unsigned long)aes_dd);

	crypto_init_queue(&aes_dd->queue, 32);

	spin_lock(&nu_aes.lock);
	list_add_tail(&aes_dd->list, &nu_aes.dev_list);
	spin_unlock(&nu_aes.lock);


	/*
	 *  Register AES/SM4 algorithms
	 */
	for (i = 0; i < ARRAY_SIZE(nuvoton_aes_algs); i++) {
		err = crypto_register_skcipher(&nuvoton_aes_algs[i]);
		if (err) {
			dev_err(dev, "Could not register nuvoton_aes_algs!\n");
			err = -EINVAL;
			goto err_algs;
		}
	}

	if (nu_cryp_dev->use_optee == false) {
		err = nuvoton_register_gcm_ccm(dev);
		if (err)
			goto err_algs;
	}

	pr_info("MA35D1 Crypto AES engine enabled.\n");
	return 0;

err_algs:
	spin_lock(&nu_aes.lock);
	list_del(&aes_dd->list);
	spin_unlock(&nu_aes.lock);
	tasklet_kill(&aes_dd->done_task);
	tasklet_kill(&aes_dd->queue_task);
	return err;
}

int nuvoton_aes_remove(struct device *dev, struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_aes_dev  *aes_dd = &nu_cryp_dev->aes_dd;
	int i;

	if (aes_dd == NULL)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(nuvoton_aes_algs); i++)
		crypto_unregister_skcipher(&nuvoton_aes_algs[i]);

	crypto_unregister_aeads(nuvoton_aes_gcm_alg, ARRAY_SIZE(nuvoton_aes_gcm_alg));
	crypto_unregister_aeads(nuvoton_aes_ccm_alg, ARRAY_SIZE(nuvoton_aes_ccm_alg));

	spin_lock(&nu_aes.lock);
	list_del(&aes_dd->list);
	spin_unlock(&nu_aes.lock);

	tasklet_kill(&aes_dd->done_task);
	tasklet_kill(&aes_dd->queue_task);

#ifdef CONFIG_OPTEE
	tee_shm_free(aes_dd->shm_pool);
	tee_client_close_context(aes_dd->octx);
#endif
	return 0;
}


