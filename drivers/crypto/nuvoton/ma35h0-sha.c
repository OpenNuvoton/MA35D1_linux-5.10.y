// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/ma35h0-sha.c
 *
 * Copyright (c) 2023 Nuvoton technology corporation.
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

#include "ma35h0-crypto.h"

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

static struct nu_sha_drv nu_sha = {
	.dev_list = LIST_HEAD_INIT(nu_sha.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(nu_sha.lock),
};

static struct nu_sha_dev *ma35h0_sha_find_dev(struct nu_sha_ctx *tctx)
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
	writel_relaxed(val, sha_dd->reg_base + reg);
}

static inline u32 nu_read_reg(struct nu_sha_dev *sha_dd, u32 reg)
{
	return readl_relaxed(sha_dd->reg_base + reg);
}

static int ma35h0_sha_dma_run(struct nu_sha_dev *dd, int is_key_block)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int  dma_cnt;

	dma_cnt = 0;
	tctx->dma_buff = 0;
	if (is_key_block) {
		tctx->dma_buff = dma_map_single(dd->dev, tctx->keybuf,
						HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dd->dev, tctx->dma_buff))) {
			dev_err(dd->dev, "SHA keybuf dma map error\n");
			return -EINVAL;
		}
		dma_sync_single_for_cpu(dd->dev, tctx->dma_buff,
					HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);
		dma_cnt = tctx->keybufcnt;
	} else {
		tctx->dma_buff = dma_map_single(dd->dev, tctx->buffer,
						SHA_BUFF_SIZE, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dd->dev, tctx->dma_buff))) {
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
		/* workaround for MA35H0 SHA3 in case of DMACNT is 0 */
		ctx->reg_ctl |= HMAC_CTL_DMACSCAD;
	}

	pr_debug("Write HMAC_CTL = 0x%x, dma_cnt = %d, key_len = %d/%d\n",
		ctx->reg_ctl, dma_cnt, tctx->hmac_key_len, ctx->block_size);

	nu_write_reg(dd, 0, HMAC_KSCTL);

	nu_write_reg(dd, (INTSTS_HMACIF | INTSTS_HMACEIF), INTSTS);
	nu_write_reg(dd, nu_read_reg(dd, INTEN) | (INTEN_HMACIEN | INTEN_HMACEIEN), INTEN);

	nu_write_reg(dd, tctx->hmac_key_len, HMAC_KEYCNT);
	nu_write_reg(dd, dma_cnt, HMAC_DMACNT);
	nu_write_reg(dd, tctx->dma_buff, HMAC_SADDR);
	nu_write_reg(dd, tctx->dma_fdbck, HMAC_FBADDR);
	nu_write_reg(dd, ctx->reg_ctl, HMAC_CTL);

	return -EINPROGRESS;
}

/*
 *  The whole SHA operation is finished. Get the digest result from SHA engine.
 */
static void  ma35h0_sha_get_result(struct ahash_request *req)
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
static void ma35h0_sha_finish_req(struct nu_sha_reqctx *ctx, int err)
{
	struct nu_sha_dev *dd = ctx->dd;
	struct ahash_request *req = dd->req;

	/*
	 *  In case of error occurred or it's the completion of final request
	 */
	if ((ctx->flags & SHA_FLAGS_FINAL_DMA) && !err) {
		/* success and get output digest */
		ma35h0_sha_get_result(req);
	}
	req->base.complete(&req->base, err);

	dd->flags &= ~DD_FLAGS_BUSY;

	/* Handle new request */
	tasklet_schedule(&ctx->dd->queue_task);
}

static int ma35h0_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_dev *dd = ma35h0_sha_find_dev(tctx);
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
	ctx->dma_max_size = (SHA_BUFF_SIZE / ctx->block_size) * ctx->block_size;

	if (!(tctx->hash_mode & HMAC_CTL_HMACEN)) {
		tctx->hmac_key_len = 0;
		tctx->bufcnt = 0;
		return 0;
	}

	/* is HMAC, check key length */
	if (((tctx->hmac_key_len + ctx->block_size - 1) >
		HMAC_KEY_BUFF_SIZE) ||	(tctx->hmac_key_len == 0)) {
		pr_err("HMAC key length %d is not supported!\n", tctx->hmac_key_len);
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

static void ma35h0_sha_sg_to_dma_buffer(struct ahash_request *req,
					 struct nu_sha_ctx *tctx,
					 struct nu_sha_reqctx *ctx)
{
	int	copy_len;

	while (ctx->sg && (ctx->req_len > 0) &&
		(tctx->bufcnt < ctx->dma_max_size)) {
		copy_len = min((int)ctx->sg->length - ctx->sg_off, ctx->req_len);

		if (ctx->dma_max_size - tctx->bufcnt < copy_len)
			copy_len = ctx->dma_max_size - tctx->bufcnt;

		memcpy(&tctx->buffer[tctx->bufcnt], (u8 *)sg_virt(ctx->sg) +
			ctx->sg_off, copy_len);

		tctx->bufcnt += copy_len;
		ctx->req_len -= copy_len;
		ctx->sg_off += copy_len;

		if (ctx->sg_off >= ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			ctx->sg_off = 0;
		}
	}
}

static int ma35h0_sha_update_start(struct nu_sha_dev *dd)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int err = 0;

	if ((ctx->req_len > 0) &&  (tctx->bufcnt < ctx->dma_max_size))
		ma35h0_sha_sg_to_dma_buffer(dd->req, tctx, ctx);

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		if ((ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) &&
		    (tctx->bufcnt == 0) && (dd->req->nbytes == 0)) {
			pr_err("MA35H0 HMAC does not support 0 data length!\n");
			ma35h0_sha_finish_req(ctx, -EINVAL);
			return -EINVAL;
		}
		err = ma35h0_sha_dma_run(dd, 1);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			ma35h0_sha_finish_req(ctx, err);
		}
	} else if (tctx->bufcnt == ctx->dma_max_size) {
		/*
		 * DMA buffer is full, start DMA.
		 */

		/* Check if it's the final DMA */
		if ((ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) && (ctx->req_len == 0))
			ctx->flags |= SHA_FLAGS_FINAL_DMA;

		err = ma35h0_sha_dma_run(dd, 0);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			ma35h0_sha_finish_req(ctx, err);
		}
	} else if (ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) {
		/*
		 * This is the last block of the final update, or
		 * is the final request. It should be the last DMA.
		 * If key block was queued, process it first.
		 */

		ctx->flags |= SHA_FLAGS_FINAL_DMA;
		err = ma35h0_sha_dma_run(dd, 0);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			ma35h0_sha_finish_req(ctx, err);
		}
	} else {
		/*
		 * All data of this request were copy to DMA buffer.
		 * We can finish this request.
		 */
		ma35h0_sha_finish_req(ctx, 0);
		err = 0;
	}
	return err;
}

static int ma35h0_sha_handle_queue(struct nu_sha_dev *dd,
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
	return ma35h0_sha_update_start(dd);
}

static void ma35h0_sha_dma_complete(struct nu_sha_reqctx *ctx)
{
	struct nu_sha_dev	*dd = ctx->dd;
	struct nu_sha_ctx	*tctx = crypto_tfm_ctx(dd->req->base.tfm);

	ctx->flags &= ~SHA_FLAGS_FIRST;     /* clear FIRST flag anyway     */

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		ctx->flags &= ~SHA_FLAGS_KEY_BLK;
		ma35h0_sha_update_start(dd);
		return;
	}
	tctx->bufcnt = 0;	    /* reset DMA buffer count      */
	if (ctx->req_len == 0) {
		/* the current request H/W processing done */
		ma35h0_sha_finish_req(ctx, 0);
		return;
	}
	ma35h0_sha_update_start(dd);
}

static int ma35h0_sha_update(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(req->base.tfm);

	ctx->sg = req->src;
	ctx->sg_off = 0;
	ctx->req_len = req->nbytes;

	ma35h0_sha_sg_to_dma_buffer(req, tctx, ctx);
	if (tctx->bufcnt + ctx->req_len <= ctx->dma_max_size)
		return 0;
	return ma35h0_sha_handle_queue(ctx->dd, req);
}

static int ma35h0_sha_final(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	ctx->flags |= SHA_FLAGS_FINAL;
	return ma35h0_sha_handle_queue(ctx->dd, req);
}

static int ma35h0_sha_finup(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;

	ctx->flags |= SHA_FLAGS_FINUP;

	err1 = ma35h0_sha_update(req);
	if (err1 == -EINPROGRESS ||
	    (err1 == -EBUSY && (ahash_request_flags(req) &
				CRYPTO_TFM_REQ_MAY_BACKLOG)))
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if update() failed, except EINPROGRESS
	 */
	err2 = ma35h0_sha_final(req);

	return err1 ?: err2;
}

static int ma35h0_sha_digest(struct ahash_request *req)
{
	return ma35h0_sha_init(req) ?: ma35h0_sha_finup(req);
}

static int ma35h0_sha_setkey(struct crypto_ahash *tfm, const u8 *key, u32 keylen)
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

static int ma35h0_sha_export(struct ahash_request *req, void *out)
{
	const struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int ma35h0_sha_import(struct ahash_request *req, const void *in)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static int ma35h0_sha_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(tfm);
	struct nu_sha_dev *dd = ma35h0_sha_find_dev(tctx);

	dd = ma35h0_sha_find_dev(tctx);
	if (!dd)
		return -ENODEV;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm), sizeof(struct nu_sha_reqctx));
	return 0;
}

static int ma35h0_sha_cra_init(struct crypto_tfm *tfm)
{
	return ma35h0_sha_cra_init_alg(tfm, NULL);
}

static void ma35h0_sha_cra_exit(struct crypto_tfm *tfm)
{
}

static struct ahash_alg  ma35h0_sha_algs[] = {
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
	.setkey		= ma35h0_sha_setkey,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
	.setkey		= ma35h0_sha_setkey,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
	.setkey		= ma35h0_sha_setkey,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
	.setkey		= ma35h0_sha_setkey,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
	.setkey		= ma35h0_sha_setkey,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
};

static struct ahash_alg  ma35h0_sha3_algs[] = {
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
{
	.init		= ma35h0_sha_init,
	.update		= ma35h0_sha_update,
	.final		= ma35h0_sha_final,
	.finup		= ma35h0_sha_finup,
	.digest		= ma35h0_sha_digest,
	.export		= ma35h0_sha_export,
	.import		= ma35h0_sha_import,
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
		.cra_init		= ma35h0_sha_cra_init,
		.cra_exit		= ma35h0_sha_cra_exit,
	}
},
};

static void ma35h0_sha_queue_task(unsigned long data)
{
	struct nu_sha_dev *dd = (struct nu_sha_dev *)data;

	ma35h0_sha_handle_queue(dd, NULL);
}

/*
 *  This task is triggerred by Crypto IRQ when a SHA DMA completed.
 */
static void ma35h0_sha_done_task(unsigned long data)
{
	struct nu_sha_dev *dd = (struct nu_sha_dev *)data;
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	int   map_size;

	if (ctx->flags & SHA_FLAGS_KEY_BLK)
		map_size = HMAC_KEY_BUFF_SIZE;
	else
		map_size = SHA_BUFF_SIZE;

	dma_unmap_single(dd->dev, tctx->dma_fdbck, SHA_FDBCK_SIZE, DMA_BIDIRECTIONAL);

	if (tctx->dma_buff != 0)
		dma_unmap_single(dd->dev, tctx->dma_buff, map_size, DMA_TO_DEVICE);

	ma35h0_sha_dma_complete(ctx);
}

int ma35h0_sha_probe(struct device *dev, struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int   i, err = 0;

	sha_dd->dev = dev;
	sha_dd->nu_cdev = nu_cryp_dev;
	sha_dd->reg_base = nu_cryp_dev->reg_base;

	INIT_LIST_HEAD(&sha_dd->list);
	spin_lock_init(&sha_dd->lock);

	tasklet_init(&sha_dd->done_task, ma35h0_sha_done_task, (unsigned long)sha_dd);
	tasklet_init(&sha_dd->queue_task, ma35h0_sha_queue_task, (unsigned long)sha_dd);

	crypto_init_queue(&sha_dd->queue, 32);

	spin_lock(&nu_sha.lock);
	list_add_tail(&sha_dd->list, &nu_sha.dev_list);
	spin_unlock(&nu_sha.lock);

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha_algs); i++) {
		err = crypto_register_ahash(&ma35h0_sha_algs[i]);
		if (err)
			goto err_register;
	}

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha3_algs); i++) {
		err = crypto_register_ahash(&ma35h0_sha3_algs[i]);
		if (err)
			goto err_register;
	}

	pr_info("MA35H0 Crypto SHA engine enabled.\n");
	return 0;

err_register:
	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->queue_task);
	tasklet_kill(&sha_dd->done_task);

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha_algs); i++)
		crypto_unregister_ahash(&ma35h0_sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha3_algs); i++)
		crypto_unregister_ahash(&ma35h0_sha3_algs[i]);

	dev_err(dev, "SHA initialization failed. %d\n", err);

	return err;
}

int ma35h0_sha_remove(struct device *dev,
		       struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev  *sha_dd = &nu_cryp_dev->sha_dd;
	int	i;

	if (sha_dd == NULL)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha_algs); i++)
		crypto_unregister_ahash(&ma35h0_sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(ma35h0_sha3_algs); i++)
		crypto_unregister_ahash(&ma35h0_sha3_algs[i]);

	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);

	tasklet_kill(&sha_dd->done_task);
	tasklet_kill(&sha_dd->queue_task);

	return 0;
}
