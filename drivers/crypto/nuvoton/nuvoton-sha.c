// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-sha.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 * Some ideas are from oamp-sha.c and mtk-sha.c drivers.
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
#include <crypto/algapi.h>
#include <crypto/sha.h>
#include <crypto/sha3.h>
#include <crypto/sm3.h>
#include <crypto/md5.h>
#include <crypto/hmac.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "nuvoton-crypto.h"

/* SHA device flags */
#define DD_FLAGS_BUSY		BIT(0)
#define DD_FLAGS_DO_KEY		BIT(1)

/* SHA context flags */
#define SHA_FLAGS_FIRST		BIT(0)
#define SHA_FLAGS_KEY_BLK	BIT(1)
#define	SHA_FLAGS_FINUP		BIT(2)  /* is a final update request */
#define	SHA_FLAGS_FINAL		BIT(3)  /* is the final request */
#define	SHA_FLAGS_FINAL_DMA	BIT(4)  /* is last DMA of the final request */

struct nu_sha_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

static int  optee_sha_open(struct nu_sha_dev *dd);
static void optee_sha_close(struct nu_sha_dev *dd);

static struct nu_sha_drv nu_sha = {
	.dev_list = LIST_HEAD_INIT(nu_sha.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(nu_sha.lock),
};

static struct nu_sha_dev *nuvoton_sha_find_dev(struct nu_sha_ctx *tctx)
{
	struct nu_sha_dev *dd = NULL;
	struct nu_sha_dev *tmp;

	spin_lock_bh(&nu_sha.lock);
	if (!tctx->dd) {
		list_for_each_entry(tmp, &nu_sha.dev_list, list) {
			dd = tmp;
			break;
		}
		tctx->dd = dd;
	} else {
		dd = tctx->dd;
	}
	spin_unlock_bh(&nu_sha.lock);
	return dd;
}

static inline void nu_write_reg(struct nu_sha_dev *sha_dd, u32 val, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (sha_dd->nu_cdev->use_optee == true)
		sha_dd->va_shm[reg/4] = val;
	else
		writel_relaxed(val, sha_dd->reg_base + reg);
#else
	writel_relaxed(val, sha_dd->reg_base + reg);
#endif
}

static inline u32 nu_read_reg(struct nu_sha_dev *sha_dd, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (sha_dd->nu_cdev->use_optee == true)
		return sha_dd->va_shm[reg/4];
	else
		return readl_relaxed(sha_dd->reg_base + reg);
#else
	return readl_relaxed(sha_dd->reg_base + reg);
#endif
}

static int nuvoton_sha_dma_run(struct nu_sha_dev *dd, int is_key_block)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int  dma_cnt;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int  err;
#endif
	dma_cnt = 0;
	tctx->dma_buff = 0;
	if (is_key_block) {
		tctx->dma_buff = dma_map_single(dd->dev, tctx->keybuf,
				HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dd->dev,
				tctx->dma_buff))) {
			dev_err(dd->dev, "SHA keybuf dma map error\n");
			return -EINVAL;
		}
		dma_sync_single_for_cpu(dd->dev, tctx->dma_buff,
				 HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);
		dma_cnt = tctx->keybufcnt;
	} else {
		tctx->dma_buff = dma_map_single(dd->dev, tctx->buffer,
				    SHA_BUFF_SIZE, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dd->dev,
				tctx->dma_buff))) {
			dev_err(dd->dev, "SHA buffer dma map error\n");
			return -EINVAL;
		}
		dma_sync_single_for_cpu(dd->dev, tctx->dma_buff,
					SHA_BUFF_SIZE, DMA_TO_DEVICE);
		dma_cnt = tctx->bufcnt;
	}

	tctx->dma_fdbck = dma_map_single(dd->dev, tctx->fdbck,
					SHA_FDBCK_SIZE, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dd->dev, tctx->dma_fdbck))) {
		dev_err(dd->dev, "dma map bytes error\n");
		return -EINVAL;
	}

	dma_sync_single_for_cpu(dd->dev, tctx->dma_buff, dma_cnt,
					DMA_FROM_DEVICE);

	ctx->reg_ctl |= HMAC_CTL_INSWAP | HMAC_CTL_OUTSWAP | HMAC_CTL_FBOUT |
			HMAC_CTL_DMACSCAD | HMAC_CTL_DMAEN | HMAC_CTL_START;
	ctx->reg_ctl |= tctx->hash_mode;	/* HMAC/SHA3/SM3/MD5 */

	if (ctx->flags & SHA_FLAGS_FIRST) {
		ctx->reg_ctl |= HMAC_CTL_DMAFIRST;
	} else {
		ctx->reg_ctl &= ~HMAC_CTL_DMAFIRST;
		ctx->reg_ctl |= HMAC_CTL_FBIN;
	}

	if (ctx->flags & SHA_FLAGS_FINAL_DMA) {
		/* It's the final request and all data have in DMA buffer. */
		ctx->reg_ctl |= HMAC_CTL_DMALAST;
		if (ctx->flags & SHA_FLAGS_FIRST)
			ctx->reg_ctl &= ~HMAC_CTL_DMACSCAD;
	}

	if ((tctx->hash_mode & HMAC_CTL_SHA3EN) && (tctx->bufcnt == 0)) {
		/* workaround for MA35D1 SHA3 in case of DMACNT is 0 */
		ctx->reg_ctl |= HMAC_CTL_DMACSCAD;
	}

	pr_debug("Write HMAC_CTL = 0x%x, dma_cnt = %d, key_len = %d/%d\n",
		ctx->reg_ctl, dma_cnt, tctx->hmac_key_len, ctx->block_size);

	nu_write_reg(dd, 0, HMAC_KSCTL);

	nu_write_reg(dd, (INTSTS_HMACIF | INTSTS_HMACEIF), INTSTS);
	nu_write_reg(dd, nu_read_reg(dd, INTEN) |
			(INTEN_HMACIEN | INTEN_HMACEIEN), INTEN);

	nu_write_reg(dd, tctx->hmac_key_len, HMAC_KEYCNT);
	nu_write_reg(dd, dma_cnt,            HMAC_DMACNT);
	nu_write_reg(dd, tctx->dma_buff,     HMAC_SADDR);
	nu_write_reg(dd, tctx->dma_fdbck,    HMAC_FBADDR);
	nu_write_reg(dd, ctx->reg_ctl, HMAC_CTL);

#ifdef CONFIG_OPTEE
	if (dd->nu_cdev->use_optee == false)
		return -EINPROGRESS;

	/*--------------------------------------------------------------*/
	/*  Invoke OP-TEE Crypto PTA to run SHA                         */
	/*--------------------------------------------------------------*/

	if (ctx->flags & SHA_FLAGS_FIRST) {
		/*
		 * Open a crypto session
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_OPEN_SESSION function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_OPEN_SESSION;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;
		param[0].u.value.a = C_CODE_SHA;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_OPEN_SESSION err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}
		dd->crypto_session_id = param[1].u.value.a;

		/*
		 * Invoke PTA_CMD_CRYPTO_SHA_START
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_SHA_START function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_SHA_START;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

		param[0].u.value.a = dd->crypto_session_id;
		param[1].u.value.a = ctx->reg_ctl;
		param[1].u.value.b = 0;
		param[2].u.value.a = tctx->hmac_key_len;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_SHA_START err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}
	}

	/*
	 * Invoke PTA_CMD_CRYPTO_SHA_UPDATE/FINAL
	 */
	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	/* Invoke PTA_CMD_CRYPTO_SHA_UPDATE/FINAL function of Trusted App */
	if (ctx->flags & SHA_FLAGS_FINAL_DMA)
		inv_arg.func = PTA_CMD_CRYPTO_SHA_FINAL;
	else
		inv_arg.func = PTA_CMD_CRYPTO_SHA_UPDATE;
	inv_arg.session = dd->session_id;
	inv_arg.num_params = 4;

	/* Fill invoke cmd params */
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

	param[0].u.value.a = dd->crypto_session_id;
	param[0].u.value.b = ctx->digest_len;
	param[1].u.memref.shm = dd->shm_pool;
	param[1].u.memref.size = CRYPTO_SHM_SIZE;
	param[1].u.memref.shm_offs = 0;

	err = tee_client_invoke_func(dd->octx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_CMD_CRYPTO_SHA_UPDATE err: %x\n", inv_arg.ret);
		return -EINVAL;
	}

	tasklet_schedule(&dd->done_task);

	if (ctx->flags & SHA_FLAGS_FINAL_DMA) {
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

		param[0].u.value.a = C_CODE_SHA;
		param[1].u.value.a = dd->crypto_session_id;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_CLOSE_SESSION err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}
	}
#endif  /* CONFIG_OPTEE */
	return -EINPROGRESS;
}

/*
 *  The whole SHA operation is finished. Get the digest result from SHA engine.
 */
static void  nuvoton_sha_get_result(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	u32 *result = (u32 *)req->result;
	int i;

	/* Get the hash from the digest buffer */
	for (i = 0; i < ctx->digest_len/4; i++)
		result[i] = nu_read_reg(ctx->dd, HMAC_DGST(i));
	pr_debug("Digest: %08x %08x %08x %08x %08x\n", result[0], result[1],
			result[2], result[3], result[4]);
}

/*
 *  A request is completed(err is 0) or aborted(err < 0).
 */
static void nuvoton_sha_finish_req(struct nu_sha_reqctx *ctx, int err)
{
	struct nu_sha_dev	*dd = ctx->dd;
	struct ahash_request	*req = dd->req;

	/*
	 *  In case of error occurred or it's the completion of final request
	 */
	if ((ctx->flags & SHA_FLAGS_FINAL_DMA) && !err) {
		/* success and get output digest */
		nuvoton_sha_get_result(req);
	}
	req->base.complete(&req->base, err);

	dd->flags &= ~DD_FLAGS_BUSY;

	/* Handle new request */
	tasklet_schedule(&ctx->dd->queue_task);
}

static int nuvoton_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);
	bool	is_sha3 = false;
	int	klen, plen;

	struct hash_alg_common *halg = crypto_hash_alg_common(tfm);
	char	*cra_name = halg->base.cra_name;

	tctx->hash_mode = 0;
	if (strncmp(cra_name, "hmac", 4) == 0) {
		tctx->hash_mode = HMAC_CTL_HMACEN;
		if (strncmp(cra_name+5, "sha3-", 5) == 0) {
			tctx->hash_mode |= HMAC_CTL_SHA3EN;
			is_sha3 = true;
		}
		if (strncmp(cra_name+5, "sm3", 3) == 0)
			tctx->hash_mode |= HMAC_CTL_SM3EN;
		if (strncmp(cra_name+5, "md5", 3) == 0)
			tctx->hash_mode |= HMAC_CTL_MD5EN;
	} else if (strncmp(cra_name, "sha3-", 5) == 0) {
		is_sha3 = true;
		tctx->hash_mode = HMAC_CTL_SHA3EN;
	} else if (strncmp(cra_name, "sm3", 3) == 0) {
		tctx->hash_mode = HMAC_CTL_SM3EN;
	} else if (strncmp(cra_name, "md5", 3) == 0) {
		tctx->hash_mode = HMAC_CTL_MD5EN;
	} else {
		/* default, SHA mode */
	}

	pr_debug("[ %s ], 0x%x\n", halg->base.cra_name, tctx->hash_mode);
	ctx->dd = dd;
	ctx->flags = SHA_FLAGS_FIRST;
	ctx->reg_ctl = 0;
	ctx->digest_len = crypto_ahash_digestsize(tfm);

	switch (ctx->digest_len) {
	case SHA1_DIGEST_SIZE:
		ctx->reg_ctl |= SHA_OPMODE_SHA1;
		ctx->block_size = SHA1_BLOCK_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->reg_ctl |= SHA_OPMODE_SHA224;
		if (is_sha3 == true)
			ctx->block_size = SHA3_224_BLOCK_SIZE;
		else
			ctx->block_size = SHA224_BLOCK_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->reg_ctl |= SHA_OPMODE_SHA256;
		if (is_sha3 == true)
			ctx->block_size = SHA3_256_BLOCK_SIZE;
		else
			ctx->block_size = SHA256_BLOCK_SIZE;
		break;
	case SHA384_DIGEST_SIZE:
		ctx->reg_ctl |= SHA_OPMODE_SHA384;
		if (is_sha3 == true)
			ctx->block_size = SHA3_384_BLOCK_SIZE;
		else
			ctx->block_size = SHA384_BLOCK_SIZE;
		break;
	case SHA512_DIGEST_SIZE:
		ctx->reg_ctl |= SHA_OPMODE_SHA512;
		if (is_sha3 == true)
			ctx->block_size = SHA3_512_BLOCK_SIZE;
		else
			ctx->block_size = SHA512_BLOCK_SIZE;
		break;
	case MD5_DIGEST_SIZE:
		ctx->block_size = MD5_HMAC_BLOCK_SIZE;
		break;
	default:
		return -EINVAL;
	}
	ctx->dma_max_size = (SHA_BUFF_SIZE / ctx->block_size) *
				ctx->block_size;

	if (!(tctx->hash_mode & HMAC_CTL_HMACEN)) {
		tctx->hmac_key_len = 0;
		tctx->bufcnt = 0;
		return 0;
	}

	/* is HMAC, check key length */
	if (((tctx->hmac_key_len + ctx->block_size - 1) >
		HMAC_KEY_BUFF_SIZE) ||	(tctx->hmac_key_len == 0)) {
		pr_err("HMAC key length %d is not supported!\n",
				tctx->hmac_key_len);
		return -EINVAL;
	}

	ctx->flags |= SHA_FLAGS_KEY_BLK;
	klen = tctx->hmac_key_len;
	if ((klen % ctx->block_size) != 0) {
		/* Paading zeros to make key data be block aligned */
		plen = ctx->block_size - (klen % ctx->block_size);
		memset(&tctx->keybuf[tctx->keybufcnt], 0, plen);
		tctx->keybufcnt += plen;
	}
	return 0;
}

static void nuvoton_sha_sg_to_dma_buffer(struct ahash_request *req,
					 struct nu_sha_ctx *tctx,
					 struct nu_sha_reqctx *ctx)
{
	int	copy_len;

	while (ctx->sg && (ctx->req_len > 0) &&
		(tctx->bufcnt < ctx->dma_max_size)) {
		copy_len = min((int)ctx->sg->length - ctx->sg_off,
				ctx->req_len);
		if (ctx->dma_max_size - tctx->bufcnt < copy_len)
			copy_len = ctx->dma_max_size - tctx->bufcnt;

		memcpy(&tctx->buffer[tctx->bufcnt], (u8 *)sg_virt(ctx->sg)
				+ ctx->sg_off, copy_len);

		tctx->bufcnt += copy_len;
		ctx->req_len -= copy_len;
		ctx->sg_off += copy_len;

		if (ctx->sg_off >= ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			ctx->sg_off = 0;
		}
	}
}

static int nuvoton_sha_update_start(struct nu_sha_dev *dd)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int err = 0;

	if ((ctx->req_len > 0) &&  (tctx->bufcnt < ctx->dma_max_size))
		nuvoton_sha_sg_to_dma_buffer(dd->req, tctx, ctx);

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		if ((ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) &&
		    (tctx->bufcnt == 0) && (dd->req->nbytes == 0)) {
			pr_err("MA35D1 HMAC does not support 0 data length!\n");
			nuvoton_sha_finish_req(ctx, -EINVAL);
			return -EINVAL;
		}
		err = nuvoton_sha_dma_run(dd, 1);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			nuvoton_sha_finish_req(ctx, err);
		}
	} else if (tctx->bufcnt == ctx->dma_max_size) {
		/*
		 * DMA buffer is full, start DMA.
		 */

		/* Check if it's the final DMA */
		if ((ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) &&
		    (ctx->req_len == 0))
			ctx->flags |= SHA_FLAGS_FINAL_DMA;

		err = nuvoton_sha_dma_run(dd, 0);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			nuvoton_sha_finish_req(ctx, err);
		}
	} else if (ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) {
		/*
		 * This is the last block of the final update, or
		 * is the final request. It should be the last DMA.
		 * If key block was queued, process it first.
		 */

		ctx->flags |= SHA_FLAGS_FINAL_DMA;
		err = nuvoton_sha_dma_run(dd, 0);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			nuvoton_sha_finish_req(ctx, err);
		}
	} else {
		/*
		 * All data of this request were copy to DMA buffer.
		 * We can finish this request.
		 */
		nuvoton_sha_finish_req(ctx, 0);
		err = 0;
	}
	return err;
}

static int nuvoton_sha_handle_queue(struct nu_sha_dev *dd,
				    struct ahash_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct nu_sha_reqctx	*ctx;
	unsigned long		flags;
	int			ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&dd->queue, req);

	if ((dd->flags & DD_FLAGS_BUSY)) {
		/* SHA device is busy on a request */
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}

	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= DD_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	ctx = ahash_request_ctx(req);
	dd->req = req;
	return nuvoton_sha_update_start(dd);
}

static void nuvoton_sha_dma_complete(struct nu_sha_reqctx *ctx)
{
	struct nu_sha_dev	*dd = ctx->dd;
	struct nu_sha_ctx	*tctx = crypto_tfm_ctx(dd->req->base.tfm);

	ctx->flags &= ~SHA_FLAGS_FIRST;     /* clear FIRST flag anyway     */

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		ctx->flags &= ~SHA_FLAGS_KEY_BLK;
		nuvoton_sha_update_start(dd);
		return;
	}
	tctx->bufcnt = 0;	    /* reset DMA buffer count      */
	if (ctx->req_len == 0) {
		/* the current request H/W processing done */
		nuvoton_sha_finish_req(ctx, 0);
		return;
	}
	nuvoton_sha_update_start(dd);
}

static int nuvoton_sha_update(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(req->base.tfm);

	ctx->sg = req->src;
	ctx->sg_off = 0;
	ctx->req_len = req->nbytes;

	nuvoton_sha_sg_to_dma_buffer(req, tctx, ctx);
	if (tctx->bufcnt + ctx->req_len <= ctx->dma_max_size)
		return 0;
	return nuvoton_sha_handle_queue(ctx->dd, req);
}

static int nuvoton_sha_final(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	ctx->flags |= SHA_FLAGS_FINAL;
	return nuvoton_sha_handle_queue(ctx->dd, req);
}

static int nuvoton_sha_finup(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;

	ctx->flags |= SHA_FLAGS_FINUP;

	err1 = nuvoton_sha_update(req);
	if (err1 == -EINPROGRESS ||
	    (err1 == -EBUSY && (ahash_request_flags(req) &
				CRYPTO_TFM_REQ_MAY_BACKLOG)))
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if update() failed, except EINPROGRESS
	 */
	err2 = nuvoton_sha_final(req);

	return err1 ?: err2;
}

static int nuvoton_sha_digest(struct ahash_request *req)
{
	return nuvoton_sha_init(req) ?: nuvoton_sha_finup(req);
}

static int nuvoton_sha_setkey(struct crypto_ahash *tfm, const u8 *key,
			      u32 keylen)
{
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);

	if (keylen > HMAC_KEY_BUFF_SIZE)
		return -EINVAL;

	if (keylen > 0) {
		memcpy(tctx->keybuf, key, keylen);
		tctx->keybufcnt = keylen;
	}
	tctx->hmac_key_len = keylen;
	return 0;
}

static int nuvoton_sha_export(struct ahash_request *req, void *out)
{
	const struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int nuvoton_sha_import(struct ahash_request *req, const void *in)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static int nuvoton_sha_cra_init_alg(struct crypto_tfm *tfm,
				    const char *alg_base)
{
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(tfm);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);

	dd = nuvoton_sha_find_dev(tctx);
	if (!dd)
		return -ENODEV;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
			sizeof(struct nu_sha_reqctx));
	return 0;
}

static int nuvoton_sha_cra_init(struct crypto_tfm *tfm)
{
#ifdef CONFIG_OPTEE
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(tfm);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);

	dd = nuvoton_sha_find_dev(tctx);
	if (!dd)
		return -ENODEV;

	if (dd->nu_cdev->use_optee) {
		if (optee_sha_open(dd) != 0)
			return -ENODEV;
	}
#endif
	return nuvoton_sha_cra_init_alg(tfm, NULL);
}

static void nuvoton_sha_cra_exit(struct crypto_tfm *tfm)
{
#ifdef CONFIG_OPTEE
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(tfm);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);

	dd = nuvoton_sha_find_dev(tctx);
	if (dd) {
		if (dd->nu_cdev->use_optee)
			optee_sha_close(dd);
	}
#endif
}

static struct ahash_alg  nuvoton_sha_algs[] = {
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.	halg.digestsize	= SHA1_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha1",
		.cra_driver_name	= "nuvoton-sha1",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA1_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA224_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha224",
		.cra_driver_name	= "nuvoton-sha224",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA224_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA256_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha256",
		.cra_driver_name	= "nuvoton-sha256",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA256_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA384_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha384",
		.cra_driver_name	= "nuvoton-sha384",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA384_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA512_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha512",
		.cra_driver_name	= "nuvoton-sha512",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA512_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.setkey		= nuvoton_sha_setkey,
	.halg.digestsize	= SHA1_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(sha1)",
		.cra_driver_name	= "nuvoton-hmac-sha1",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA1_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.setkey		= nuvoton_sha_setkey,
	.halg.digestsize	= SHA224_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(sha224)",
		.cra_driver_name	= "nuvoton-hmac-sha224",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA224_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.setkey		= nuvoton_sha_setkey,
	.halg.digestsize	= SHA256_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(sha256)",
		.cra_driver_name	= "nuvoton-hmac-sha256",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA256_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.setkey		= nuvoton_sha_setkey,
	.halg.digestsize	= SHA384_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(sha384)",
		.cra_driver_name	= "nuvoton-hmac-sha384",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA384_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.setkey		= nuvoton_sha_setkey,
	.halg.digestsize	= SHA512_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "hmac(sha512)",
		.cra_driver_name	= "nuvoton-hmac-sha512",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA512_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SM3_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sm3",
		.cra_driver_name	= "nuvoton-sm3",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SM3_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= MD5_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "md5",
		.cra_driver_name	= "nuvoton-md5",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= MD5_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "md5",
		.cra_driver_name	= "nuvoton-md5",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
};

static struct ahash_alg  nuvoton_sha3_algs[] = {
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA3_224_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha3-224",
		.cra_driver_name	= "nuvoton-sha3-224",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA3_224_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA3_256_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha3-256",
		.cra_driver_name	= "nuvoton-sha3-256",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA3_256_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA3_384_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha3-384",
		.cra_driver_name	= "nuvoton-sha3-384",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA3_384_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
{
	.init		= nuvoton_sha_init,
	.update		= nuvoton_sha_update,
	.final		= nuvoton_sha_final,
	.finup		= nuvoton_sha_finup,
	.digest		= nuvoton_sha_digest,
	.export		= nuvoton_sha_export,
	.import		= nuvoton_sha_import,
	.halg.digestsize	= SHA3_512_DIGEST_SIZE,
	.halg.statesize = sizeof(struct nu_sha_reqctx),
	.halg.base	= {
		.cra_name		= "sha3-512",
		.cra_driver_name	= "nuvoton-sha3-512",
		.cra_priority		= 400,
		.cra_flags		= CRYPTO_ALG_ASYNC,
		.cra_blocksize		= SHA3_512_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct nu_sha_ctx),
		.cra_alignmask		= 0xf,
		.cra_module		= THIS_MODULE,
		.cra_init		= nuvoton_sha_cra_init,
		.cra_exit		= nuvoton_sha_cra_exit,
	}
},
};

static void nuvoton_sha_queue_task(unsigned long data)
{
	struct nu_sha_dev *dd = (struct nu_sha_dev *)data;

	nuvoton_sha_handle_queue(dd, NULL);
}

/*
 *  This task is triggerred by Crypto IRQ when a SHA DMA completed.
 */
static void nuvoton_sha_done_task(unsigned long data)
{
	struct nu_sha_dev *dd = (struct nu_sha_dev *)data;
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int   map_size;

	if (ctx->flags & SHA_FLAGS_KEY_BLK)
		map_size = HMAC_KEY_BUFF_SIZE;
	else
		map_size = SHA_BUFF_SIZE;

	dma_unmap_single(dd->dev, tctx->dma_fdbck, SHA_FDBCK_SIZE,
			DMA_BIDIRECTIONAL);

	if (tctx->dma_buff != 0)
		dma_unmap_single(dd->dev, tctx->dma_buff, map_size,
			DMA_TO_DEVICE);

	nuvoton_sha_dma_complete(ctx);
}

#ifdef CONFIG_OPTEE
static int  optee_sha_open(struct nu_sha_dev *dd)
{
	struct tee_ioctl_open_session_arg sess_arg;
	int   err;

	err = nuvoton_crypto_optee_init(dd->nu_cdev);
	if (err)
		return err;
	/*
	 * Open SHA context with TEE driver
	 */
	dd->octx = tee_client_open_context(NULL, optee_ctx_match,
					       NULL, NULL);
	if (IS_ERR(dd->octx)) {
		pr_err("%s open context failed, err: %x\n", __func__,
			sess_arg.ret);
		return err;
	}

	/*
	 * Open SHA session with Crypto Trusted App
	 */
	memset(&sess_arg, 0, sizeof(sess_arg));
	memcpy(sess_arg.uuid, dd->nu_cdev->tee_cdev->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
	sess_arg.num_params = 0;

	err = tee_client_open_session(dd->octx, &sess_arg, NULL);
	if ((err < 0) || (sess_arg.ret != 0)) {
		pr_err("%s open session failed, err: %x\n", __func__,
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	dd->session_id = sess_arg.session;

	/*
	 * Allocate handshake buffer from OP-TEE share memory
	 */
	dd->shm_pool = tee_shm_alloc(dd->octx, CRYPTO_SHM_SIZE,
				TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
	if (IS_ERR(dd->shm_pool)) {
		pr_err("%s tee_shm_alloc failed\n", __func__);
		goto out_sess;
	}

	dd->va_shm = tee_shm_get_va(dd->shm_pool, 0);
	if (IS_ERR(dd->va_shm)) {
		tee_shm_free(dd->shm_pool);
		pr_err("%s tee_shm_get_va failed\n", __func__);
		goto out_sess;
	}
	return 0;

out_sess:
	tee_client_close_session(dd->octx, dd->session_id);
out_ctx:
	tee_client_close_context(dd->octx);
	return err;
}

static void optee_sha_close(struct nu_sha_dev *dd)
{
	tee_shm_free(dd->shm_pool);
	tee_client_close_session(dd->octx, dd->session_id);
	tee_client_close_context(dd->octx);
	dd->octx = NULL;
}
#endif

int nuvoton_sha_probe(struct device *dev,
		      struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int   i, err = 0;

	sha_dd->dev = dev;
	sha_dd->nu_cdev = nu_cryp_dev;
	sha_dd->reg_base = nu_cryp_dev->reg_base;
	sha_dd->octx = NULL;

	INIT_LIST_HEAD(&sha_dd->list);
	spin_lock_init(&sha_dd->lock);

	tasklet_init(&sha_dd->done_task, nuvoton_sha_done_task,
			(unsigned long)sha_dd);
	tasklet_init(&sha_dd->queue_task, nuvoton_sha_queue_task,
			(unsigned long)sha_dd);

	crypto_init_queue(&sha_dd->queue, 32);

	spin_lock(&nu_sha.lock);
	list_add_tail(&sha_dd->list, &nu_sha.dev_list);
	spin_unlock(&nu_sha.lock);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++) {
		err = crypto_register_ahash(&nuvoton_sha_algs[i]);
		if (err)
			goto err_register;
	}

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha3_algs); i++) {
		err = crypto_register_ahash(&nuvoton_sha3_algs[i]);
		if (err)
			goto err_register;
	}

	pr_info("MA35D1 Crypto SHA engine enabled.\n");
	return 0;

err_register:
	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->queue_task);
	tasklet_kill(&sha_dd->done_task);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha3_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha3_algs[i]);

	dev_err(dev, "SHA initialization failed. %d\n", err);

	return err;
}

int nuvoton_sha_remove(struct device *dev,
		       struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int	i;

	if (sha_dd == NULL)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha3_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha3_algs[i]);

	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->done_task);
	tasklet_kill(&sha_dd->queue_task);

#ifdef CONFIG_OPTEE
	if (nu_cryp_dev->use_optee)
		optee_sha_close(sha_dd);
#endif
	return 0;
}


