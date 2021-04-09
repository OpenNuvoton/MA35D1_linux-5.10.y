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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/tee_drv.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
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

/* SHA flags */
#define SHA_FLAGS_BUSY		BIT(0)
#define	SHA_FLAGS_FIRST		BIT(1)
#define	SHA_FLAGS_FINAL		BIT(2)
#define	SHA_FLAGS_FINUP		BIT(3)
#define SHA_FLAGS_ERROR		BIT(4)

#define SHA_OP_UPDATE		1
#define SHA_OP_FINAL		2

struct nu_sha_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

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
	if (sha_dd->use_optee == true)
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
	if (sha_dd->use_optee == true)
		return sha_dd->va_shm[reg/4];
	else
		return readl_relaxed(sha_dd->reg_base + reg);
#else
	return readl_relaxed(sha_dd->reg_base + reg);
#endif
}

static int nuvoton_sha_dma_run(struct nu_sha_dev *dd, bool is_hamc_key_block)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	int  err;
#endif

	if (tctx->bufcnt > 0) {
		tctx->dma_buff = dma_map_single(dd->dev, tctx->buffer,
					SHA_BUFF_SIZE, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dd->dev, tctx->dma_buff))) {
			dev_err(dd->dev, "SHA buffer dma map error\n");
			return -EINVAL;
		}
		dma_sync_single_for_cpu(dd->dev, tctx->dma_buff,
					SHA_BUFF_SIZE, DMA_TO_DEVICE);
	} else {
		tctx->dma_buff = 0;
	}


	tctx->dma_fdbck = dma_map_single(dd->dev, tctx->fdbck,
					SHA_FDBCK_SIZE, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dd->dev, tctx->dma_fdbck))) {
		dev_err(dd->dev, "dma map bytes error\n");
		return -EINVAL;
	}

	ctx->reg_ctl |= HMAC_CTL_INSWAP | HMAC_CTL_OUTSWAP | HMAC_CTL_FBIN |
			HMAC_CTL_FBOUT | HMAC_CTL_DMAEN | HMAC_CTL_DMACSCAD |
			HMAC_CTL_START;

	ctx->reg_ctl |= tctx->hash_mode;	/* HMAC/SHA3/SM3/MD5 */

	if (ctx->flags & SHA_FLAGS_FIRST) {
		ctx->reg_ctl &= ~HMAC_CTL_FBIN;
		ctx->reg_ctl |= HMAC_CTL_DMAFIRST;
	} else {
		ctx->reg_ctl &= ~HMAC_CTL_DMAFIRST;
	}

	if (ctx->flags & SHA_FLAGS_FINAL) {
		/* It's the final request and all data have in DMA buffer. */
		ctx->reg_ctl |= HMAC_CTL_DMALAST;
	}

	pr_debug("Write HMAC_CTL = 0x%x, bufcnt = %d, key_len = %d\n",
			ctx->reg_ctl, tctx->bufcnt, tctx->hmac_key_len);

	nu_write_reg(dd, 0, HMAC_KSCTL);

	nu_write_reg(dd, (INTSTS_HMACIF | INTSTS_HMACEIF), INTSTS);
	nu_write_reg(dd, nu_read_reg(dd, INTEN) |
			(INTEN_HMACIEN | INTEN_HMACEIEN), INTEN);

	nu_write_reg(dd, tctx->hmac_key_len, HMAC_KEYCNT);
	nu_write_reg(dd, tctx->bufcnt,       HMAC_DMACNT);
	nu_write_reg(dd, tctx->dma_buff,     HMAC_SADDR);
	nu_write_reg(dd, tctx->dma_fdbck,    HMAC_FBADDR);
	nu_write_reg(dd, ctx->reg_ctl,       HMAC_CTL);

#ifdef CONFIG_OPTEE
	if (dd->use_optee == false)
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
	if (ctx->flags & SHA_FLAGS_FINAL)
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

	if (is_hamc_key_block == true) {
		ctx->flags &= ~SHA_FLAGS_FIRST;
		tctx->bufcnt = 0;
		dma_unmap_single(dd->dev, tctx->dma_fdbck, SHA_FDBCK_SIZE,
					DMA_BIDIRECTIONAL);
		dma_unmap_single(dd->dev, tctx->dma_buff, SHA_BUFF_SIZE,
					DMA_TO_DEVICE);
	} else
		tasklet_schedule(&dd->done_task);

	if (ctx->flags & SHA_FLAGS_FINAL) {
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

static int nuvoton_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);
	int	alen;
	bool	is_sha3 = false;
	int	err;

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

	/*
	 * The following code is for HMAC enabled case.
	 */
	if (unlikely(tctx->hmac_key_len == 0)) {
		pr_err("HMAC enabled but HAMC key len is zero!\n");
		return -EINVAL;
	}

	/* Adjust HMAC key to be block aligned */
	if ((tctx->hmac_key_len % ctx->block_size) != 0) {
		alen = tctx->hmac_key_len + (ctx->block_size -
			(tctx->hmac_key_len % ctx->block_size));
		memset(&(tctx->buffer[tctx->hmac_key_len]), 0, (alen -
			tctx->hmac_key_len));
		tctx->bufcnt = alen;

		/* Execute HMAC-SHA on the HMAC key block */
		err = nuvoton_sha_dma_run(ctx->dd, true);
		if (err != -EINPROGRESS)
			return err;

#ifdef CONFIG_OPTEE
		if (dd->use_optee == true)
			return 0;
#endif
		/*
		 *  Wait until SHA engine busy cleared or timeout
		 */
		while (nu_read_reg(ctx->dd, HMAC_STS) & HMAC_STS_BUSY) {
			if (time_after(jiffies, jiffies +
					msecs_to_jiffies(200))) {
				nu_write_reg(dd, HMAC_CTL_STOP, HMAC_CTL);
				dev_err(dd->dev, "HAMC key block DMA time-out!\n");
				return -EBUSY;
			}
		}
	}
	return 0;
}

static void nuvoton_sha_sg_to_dma_buffer(struct nu_sha_ctx *tctx,
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
	if ((err == 0) && (ctx->op == SHA_OP_FINAL)) {
		/* success and get output digest */
		nuvoton_sha_get_result(req);
	}

	req->base.complete(&req->base, err);

	dd->flags &= ~SHA_FLAGS_BUSY;	/* SHA device complete a request. */

	/* Handle new request */
	tasklet_schedule(&ctx->dd->queue_task);
}

static int nuvoton_sha_update_start(struct nu_sha_dev *dd)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int	err = 0;

	if (ctx->req_len > 0)
		nuvoton_sha_sg_to_dma_buffer(tctx, ctx);

	if ((ctx->op == SHA_OP_FINAL) && (ctx->req_len == 0))
		ctx->flags |= SHA_FLAGS_FINAL;	/* note the final DMA */

	if ((ctx->flags & SHA_FLAGS_FINAL) ||
	     (tctx->bufcnt == ctx->dma_max_size)) {
		/*
		 * Start SHA DMA if this is the final block or
		 * DMA buffer is full. The final DMA can be zero length.
		 */
		err = nuvoton_sha_dma_run(dd, false);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			nuvoton_sha_finish_req(ctx, err);
		}
	} else {
		/*
		 * All data of this request were copy to DMA buffer.
		 * We can finish this request.
		 */
		if (unlikely(ctx->op == SHA_OP_FINAL)) {
			nuvoton_sha_finish_req(ctx, -EIO);
			return -EIO;
		}
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

	if (dd->flags & SHA_FLAGS_BUSY) {
		/* SHA device is busy on a request */
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}

	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= SHA_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	ctx = ahash_request_ctx(req);

	dd->req = req;

	ret = nuvoton_sha_update_start(dd);
	if (ret != -EINPROGRESS) {
		nuvoton_sha_finish_req(ctx, ret);
		return 0;
	}
	return ret;
}

static void nuvoton_sha_dma_complete(struct nu_sha_reqctx *ctx)
{
	struct nu_sha_dev	*dd = ctx->dd;
	struct nu_sha_ctx	*tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int	err = 0;

	tctx->bufcnt = 0;	/* clear DMA buffer count */
	ctx->flags &= ~SHA_FLAGS_FIRST;

	if ((tctx->hash_mode & HMAC_CTL_HMACEN) &&
		(ctx->flags & SHA_FLAGS_FIRST)) {
		ctx->flags &= ~SHA_FLAGS_FIRST;
		return;
	}
	if (ctx->req_len == 0) {
		/* the current request H/W processing done */
		nuvoton_sha_finish_req(ctx, 0);
		return;
	}

	err = nuvoton_sha_update_start(dd);
	if (err != -EINPROGRESS)
		nuvoton_sha_finish_req(ctx, 0);
}

static int nuvoton_sha_update(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(req->base.tfm);

	ctx->op = SHA_OP_UPDATE;
	ctx->sg = req->src;
	ctx->sg_off = 0;
	ctx->req_len = req->nbytes;

	if ((tctx->bufcnt + ctx->req_len <= ctx->dma_max_size) &&
	    !(ctx->flags & SHA_FLAGS_FINUP)) {
		nuvoton_sha_sg_to_dma_buffer(tctx, ctx);
		return 0;
	}
	return nuvoton_sha_handle_queue(ctx->dd, req);
}

static int nuvoton_sha_final(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	ctx->flags |= SHA_FLAGS_FINUP;
	ctx->op = SHA_OP_FINAL;
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
#if 0
static int nuvoton_sha_setkey(struct crypto_ahash *tfm, const u8 *key,
			      u32 keylen)
{
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);

	if (keylen > SHA_BUFF_SIZE)
		return -EINVAL;

	if (keylen > 0) {
		memcpy(tctx->buffer, key, keylen);
		tctx->bufcnt = keylen;
	}
	tctx->hmac_key_len = keylen;
	return 0;
}
#endif
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
	return nuvoton_sha_cra_init_alg(tfm, NULL);
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
	.halg.digestsize	= SHA1_DIGEST_SIZE,
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
	}
},
#if 0
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
	}
},
#endif
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

	dma_unmap_single(dd->dev, tctx->dma_fdbck, SHA_FDBCK_SIZE,
			DMA_BIDIRECTIONAL);
	if (tctx->dma_buff != 0)
		dma_unmap_single(dd->dev, tctx->dma_buff, SHA_BUFF_SIZE,
			DMA_TO_DEVICE);

	nuvoton_sha_dma_complete(ctx);
}

#ifdef CONFIG_OPTEE
static int  optee_sha_open(struct nu_sha_dev *dd)
{
	struct tee_ioctl_open_session_arg sess_arg;
	int   err;

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
	memcpy(sess_arg.uuid, dd->tee_cdev->id.uuid.b, TEE_IOCTL_UUID_LEN);
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
}
#endif

int nuvoton_sha_probe(struct device *dev,
		      struct nuvoton_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int   i, j, err = 0;

	sha_dd->dev = dev;
	sha_dd->reg_base = nu_cryp_dev->reg_base;
	sha_dd->use_optee = false;
#ifdef CONFIG_OPTEE
	sha_dd->use_optee = nu_cryp_dev->use_optee;
	sha_dd->tee_cdev = nu_cryp_dev->tee_cdev;

	if (sha_dd->use_optee) {
		if (optee_sha_open(sha_dd) != 0)
			return -ENODEV;
	}
#endif


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
	pr_info("MA35D1 Crypto SHA engine enabled.\n");
	return 0;

err_register:
	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->queue_task);
	tasklet_kill(&sha_dd->done_task);

	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);

	dev_err(dev, "SHA initialization failed. %d\n", err);

	return err;
}

int nuvoton_sha_remove(struct device *dev,
		       struct nuvoton_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int	i;

	if (sha_dd == NULL)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);

	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->done_task);
	tasklet_kill(&sha_dd->queue_task);

#ifdef CONFIG_OPTEE
	if (sha_dd->use_optee)
		optee_sha_close(sha_dd);
#endif
	return 0;
}


