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
#include <asm/unaligned.h>

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
#define SHA_FLAGS_TEE_SESSION	BIT(5)  /* owned by this initialized stream */

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

/* Keep IDs independently of request memory for teardown of idle streams. */
struct nu_sha_tee_session {
	struct list_head list;
	u32 sid;
	struct nu_sha_reqctx *ctx;
	bool close_failed;
};

static void nuvoton_sha_schedule_queue(struct nu_sha_dev *dd)
{
	if (dd->nu_cdev->use_optee)
		queue_work(dd->tee_wq, &dd->tee_queue_work);
	else
		tasklet_schedule(&dd->queue_task);
}

static bool nuvoton_sha_tee_session_open(struct nu_sha_dev *dd,
					 struct nu_sha_reqctx *ctx)
{
	struct nu_sha_tee_session *session;

	if (!(ctx->flags & SHA_FLAGS_TEE_SESSION))
		return false;
	list_for_each_entry(session, &dd->tee_sessions, list) {
		if (session->ctx == ctx && session->sid == ctx->tsi_sid &&
		    !session->close_failed)
			return true;
	}
	return false;
}

static int nuvoton_sha_tee_close(struct nu_sha_dev *dd, u32 sid)
{
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg arg = { };
	struct tee_param param[4] = { };
	struct nu_sha_tee_session *session, *tmp;
	int err;

	list_for_each_entry(session, &dd->tee_sessions, list) {
		if (session->sid == sid && session->close_failed)
			return -EIO;
	}
	arg.func = PTA_CMD_CRYPTO_CLOSE_SESSION;
	arg.session = dd->session_id;
	arg.num_params = ARRAY_SIZE(param);
	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = C_CODE_SHA;
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].u.value.a = sid;
	err = tee_client_invoke_func(dd->octx, &arg, param);
	if (err < 0 || arg.ret)
		dev_err(dd->dev, "SHA close sid=%u: transport=%d PTA=%#x\n",
			sid, err, arg.ret);
	dev_dbg(dd->dev, "SHA close sid=%u\n", sid);
	list_for_each_entry_safe(session, tmp, &dd->tee_sessions, list) {
		if (session->sid == sid) {
			if (err < 0 || arg.ret) {
				session->close_failed = true;
				WRITE_ONCE(nuvoton_crypto_optee_faulted, true);
				dev_err(dd->dev,
					"OP-TEE AES/SHA disabled; reboot required\n");
				break;
			}
			list_del(&session->list);
			kfree(session);
			break;
		}
	}
	return err < 0 ? err : (arg.ret ? -EIO : 0);
#else
	return -EOPNOTSUPP;
#endif
}

static void nuvoton_sha_tee_close_all(struct nu_sha_dev *dd)
{
	struct nu_sha_tee_session *session;

	while (!list_empty(&dd->tee_sessions)) {
		session = list_first_entry(&dd->tee_sessions,
					   struct nu_sha_tee_session, list);
		if (session->close_failed ||
		    nuvoton_sha_tee_close(dd, session->sid)) {
			dev_err(dd->dev, "Unreleased SHA session %u; reboot required\n",
				session->sid);
			list_del(&session->list);
			kfree(session);
		}
	}
}

#ifdef CONFIG_OPTEE
/* Called only by the ordered worker, before opening a new stream. */
static int nuvoton_sha_tee_retire(struct nu_sha_dev *dd,
				 struct nu_sha_reqctx *ctx)
{
	struct nu_sha_tee_session *session, *tmp;
	int err;

	list_for_each_entry_safe(session, tmp, &dd->tee_sessions, list) {
		if (session->ctx != ctx)
			continue;
		err = nuvoton_sha_tee_close(dd, session->sid);
		if (err)
			return err;
	}
	return 0;
}

#endif

static void nuvoton_sha_tee_unmap(struct nu_sha_dev *dd)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	int size = ctx->flags & SHA_FLAGS_KEY_BLK ?
		   HMAC_KEY_BUFF_SIZE : SHA_BUFF_SIZE;

	if (!dd->tee_dma_mapped)
		return;
	dma_unmap_single(dd->dev, ctx->dma_fdbck, SHA_FDBCK_SIZE,
			 DMA_BIDIRECTIONAL);
	dma_unmap_single(dd->dev, ctx->dma_buff, size, DMA_TO_DEVICE);
	dd->tee_dma_mapped = false;
}

static int nuvoton_sha_dma_run(struct nu_sha_dev *dd, int is_key_block)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	struct nu_sha_ctx *tctx = crypto_tfm_ctx(dd->req->base.tfm);
	u32 hash_mode = dd->nu_cdev->use_optee ? ctx->op : tctx->hash_mode;
	int  dma_cnt;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	struct nu_sha_tee_session *session;
	int err;
#endif

	if (dd->nu_cdev->use_optee)
		dd->tee_dma_mapped = false;
	dma_cnt = 0;
	ctx->dma_buff = 0;
	if (is_key_block) {
		ctx->dma_buff = dma_map_single(dd->dev, tctx->keybuf,
					       HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dd->dev, ctx->dma_buff))) {
			dev_err(dd->dev, "SHA keybuf dma map error\n");
			return -EINVAL;
		}
		if (!dd->nu_cdev->use_optee)
			dma_sync_single_for_cpu(dd->dev, ctx->dma_buff,
						HMAC_KEY_BUFF_SIZE, DMA_TO_DEVICE);
		dma_cnt = tctx->keybufcnt;
	} else {
		ctx->dma_buff = dma_map_single(dd->dev, ctx->buffer,
					       SHA_BUFF_SIZE, DMA_TO_DEVICE);

		if (unlikely(dma_mapping_error(dd->dev, ctx->dma_buff))) {
			dev_err(dd->dev, "SHA buffer dma map error\n");
			return -EINVAL;
		}
		if (!dd->nu_cdev->use_optee)
			dma_sync_single_for_cpu(dd->dev, ctx->dma_buff,
						SHA_BUFF_SIZE, DMA_TO_DEVICE);
		dma_cnt = ctx->bufcnt;
	}

	ctx->dma_fdbck = dma_map_single(dd->dev, ctx->fdbck,
					SHA_FDBCK_SIZE, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dd->dev, ctx->dma_fdbck))) {
		dev_err(dd->dev, "dma map bytes error\n");
		if (dd->nu_cdev->use_optee)
			dma_unmap_single(dd->dev, ctx->dma_buff,
					 is_key_block ? HMAC_KEY_BUFF_SIZE :
					 SHA_BUFF_SIZE, DMA_TO_DEVICE);
		return -EINVAL;
	}
	if (dd->nu_cdev->use_optee)
		dd->tee_dma_mapped = true;
	else
		dma_sync_single_for_cpu(dd->dev, ctx->dma_buff, dma_cnt,
					DMA_FROM_DEVICE);

	ctx->reg_ctl |= HMAC_CTL_INSWAP | HMAC_CTL_OUTSWAP | HMAC_CTL_FBOUT |
			HMAC_CTL_DMACSCAD | HMAC_CTL_DMAEN | HMAC_CTL_START;
	ctx->reg_ctl |= hash_mode;	/* HMAC/SHA3/SM3/MD5 */

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

	if ((hash_mode & HMAC_CTL_SHA3EN) && (ctx->bufcnt == 0)) {
		/* workaround for MA35D1 SHA3 in case of DMACNT is 0 */
		ctx->reg_ctl |= HMAC_CTL_DMACSCAD;
	}

	pr_debug("Write HMAC_CTL = 0x%x, dma_cnt = %d, key_len = %d/%d\n", ctx->reg_ctl,
		 dma_cnt, tctx->hmac_key_len, ctx->block_size);

	nu_write_reg(dd, 0, HMAC_KSCTL);

	nu_write_reg(dd, (INTSTS_HMACIF | INTSTS_HMACEIF), INTSTS);
	nu_write_reg(dd, nu_read_reg(dd, INTEN) |
			(INTEN_HMACIEN | INTEN_HMACEIEN), INTEN);

	nu_write_reg(dd, tctx->hmac_key_len, HMAC_KEYCNT);
	nu_write_reg(dd, dma_cnt, HMAC_DMACNT);
	nu_write_reg(dd, ctx->dma_buff, HMAC_SADDR);
	nu_write_reg(dd, ctx->dma_fdbck, HMAC_FBADDR);
	nu_write_reg(dd, ctx->reg_ctl, HMAC_CTL);

#ifdef CONFIG_OPTEE
	if (dd->nu_cdev->use_optee == false)
		return -EINPROGRESS;

	/*--------------------------------------------------------------*/
	/*  Invoke OP-TEE Crypto PTA to run SHA                         */
	/*--------------------------------------------------------------*/

	if (ctx->flags & SHA_FLAGS_FIRST) {
		err = nuvoton_sha_tee_retire(dd, ctx);
		if (err)
			goto tee_error;
		session = kzalloc(sizeof(*session), GFP_KERNEL);
		if (!session) {
			err = -ENOMEM;
			goto tee_error;
		}
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
			if (err < 0)
				WRITE_ONCE(nuvoton_crypto_optee_faulted, true);
			kfree(session);
			err = err < 0 ? err : -EIO;
			goto tee_error;
		}
		ctx->tsi_sid = param[1].u.value.a;
		session->sid = ctx->tsi_sid;
		session->ctx = ctx;
		list_add_tail(&session->list, &dd->tee_sessions);
		ctx->flags |= SHA_FLAGS_TEE_SESSION;
		dev_dbg(dd->dev, "SHA open req=%p sid=%u\n", dd->req, ctx->tsi_sid);

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

		param[0].u.value.a = ctx->tsi_sid;
		param[1].u.value.a = ctx->reg_ctl;
		param[1].u.value.b = 0;
		param[2].u.value.a = tctx->hmac_key_len;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_SHA_START err: %x. %d\n",
				inv_arg.ret, tctx->hmac_key_len);
			err = err < 0 ? err : -EIO;
			goto tee_error;
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

	param[0].u.value.a = ctx->tsi_sid;
	param[0].u.value.b = ctx->digest_len;
	param[1].u.memref.shm = dd->shm_pool;
	param[1].u.memref.size = CRYPTO_SHM_SIZE;
	param[1].u.memref.shm_offs = 0;

	dev_dbg(dd->dev, "SHA run req=%p sid=%u cmd=%u\n",
		dd->req, ctx->tsi_sid, inv_arg.func);
	err = tee_client_invoke_func(dd->octx, &inv_arg, param);
	if ((err < 0) || (inv_arg.ret != 0)) {
		dev_err(dd->dev, "SHA run sid=%u: transport=%d PTA=%#x\n",
			ctx->tsi_sid, err, inv_arg.ret);
		err = err < 0 ? err : -EIO;
		goto tee_error;
	}

	if (ctx->flags & SHA_FLAGS_FINAL_DMA) {
		err = nuvoton_sha_tee_close(dd, ctx->tsi_sid);
		ctx->flags &= ~SHA_FLAGS_TEE_SESSION;
		if (err)
			goto tee_error;
	}

	queue_work(dd->tee_wq, &dd->tee_done_work);
	return -EINPROGRESS;

tee_error:
	nuvoton_sha_tee_unmap(dd);
	return err;
#endif /* CONFIG_OPTEE */
	return -EINPROGRESS;
}

/*
 *  The whole SHA operation is finished. Get the digest result from SHA engine.
 */
static void  nuvoton_sha_get_result(struct ahash_request *req)
{
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	u32 *result;
	int i;

	if (ctx->dd->nu_cdev->use_optee) {
		for (i = 0; i < ctx->digest_len / sizeof(u32); i++)
			put_unaligned(nu_read_reg(ctx->dd, HMAC_DGST(i)),
				      (u32 *)(req->result + i * sizeof(u32)));
		return;
	}

	/* Preserve the direct-hardware result and debug path. */
	result = (u32 *)req->result;
	/* Get the hash from the digest buffer */
	for (i = 0; i < ctx->digest_len/4; i++)
		result[i] = nu_read_reg(ctx->dd, HMAC_DGST(i));
	pr_debug("Digest: %08x %08x %08x %08x %08x\n",
		 result[0], result[1], result[2], result[3], result[4]);
}

/*
 *  A request is completed(err is 0) or aborted(err < 0).
 */
static void nuvoton_sha_finish_req(struct nu_sha_reqctx *ctx, int err)
{
	struct nu_sha_dev	*dd = ctx->dd;
	struct ahash_request	*req = dd->req;
	unsigned long flags;

	if (!dd->nu_cdev->use_optee) {
		/* Preserve the original direct-hardware completion path. */
		if (ctx->flags & SHA_FLAGS_FINAL_DMA) {
			if (!err)
				nuvoton_sha_get_result(req);
			kfree(ctx->buffer);
			ctx->buffer = NULL;
			ctx->bufcnt = 0;
		}
		req->base.complete(&req->base, err);
		dd->flags &= ~DD_FLAGS_BUSY;
		tasklet_schedule(&dd->queue_task);
		return;
	}

	if (err && nuvoton_sha_tee_session_open(dd, ctx))
		nuvoton_sha_tee_close(dd, ctx->tsi_sid);
	if (err)
		ctx->flags &= ~SHA_FLAGS_TEE_SESSION;

	/*
	 * An OP-TEE error aborts the stream, while a successful non-final
	 * update keeps the buffer and TSI session for the following update.
	 */
	if (err || (ctx->flags & SHA_FLAGS_FINAL_DMA)) {
		if (!err)
			nuvoton_sha_get_result(req);
		kfree(ctx->buffer);
		ctx->buffer = NULL;
		ctx->bufcnt = 0;
	}
	spin_lock_irqsave(&dd->lock, flags);
	dd->req = NULL;
	dd->flags &= ~DD_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);
	req->base.complete(&req->base, err);

	/* The callback may have freed the request. */
	nuvoton_sha_schedule_queue(dd);
}

static int nuvoton_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);
	struct nu_sha_dev *dd = nuvoton_sha_find_dev(tctx);
	bool	is_sha3 = false;
	int	klen, plen;
	u32	hash_mode = 0;

	struct hash_alg_common *halg = crypto_hash_alg_common(tfm);
	char	*cra_name = halg->base.cra_name;

	if (dd && dd->nu_cdev->use_optee &&
	    READ_ONCE(nuvoton_crypto_optee_faulted))
		return -EIO;
	if (strncmp(cra_name, "hmac", 4) == 0) {
		hash_mode = HMAC_CTL_HMACEN;
		if (strncmp(cra_name+5, "sha3-", 5) == 0) {
			hash_mode |= HMAC_CTL_SHA3EN;
			is_sha3 = true;
		}
		if (strncmp(cra_name+5, "sm3", 3) == 0)
			hash_mode |= HMAC_CTL_SM3EN;
		if (strncmp(cra_name+5, "md5", 3) == 0)
			hash_mode |= HMAC_CTL_MD5EN;
	} else if (strncmp(cra_name, "sha3-", 5) == 0) {
		is_sha3 = true;
		hash_mode = HMAC_CTL_SHA3EN;
	} else if (strncmp(cra_name, "sm3", 3) == 0) {
		hash_mode = HMAC_CTL_SM3EN;
	} else if (strncmp(cra_name, "md5", 3) == 0) {
		hash_mode = HMAC_CTL_MD5EN;
	} else {
		/* default, SHA mode */
	}

	if (dd->nu_cdev->use_optee)
		ctx->op = hash_mode;
	else
		tctx->hash_mode = hash_mode;
	pr_debug("[ %s ], 0x%x\n", halg->base.cra_name, hash_mode);
	ctx->dd = dd;
	if (dd->nu_cdev->use_optee) {
		ctx->sg = NULL;
		ctx->sg_off = 0;
		ctx->req_len = 0;
		ctx->tsi_sid = 0;
	}
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

	if (dd->nu_cdev->use_optee)
		ctx->buffer = kmalloc(SHA_BUFF_SIZE, GFP_DMA |
			(ahash_request_flags(req) & CRYPTO_TFM_REQ_MAY_SLEEP ?
			 GFP_KERNEL : GFP_ATOMIC));
	else
		ctx->buffer = kmalloc(SHA_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	if (!ctx->buffer)
		return -ENOMEM;

	ctx->bufcnt = 0;
	ctx->dma_max_size = (SHA_BUFF_SIZE / ctx->block_size) * ctx->block_size;

	if (!(hash_mode & HMAC_CTL_HMACEN)) {
		if (!dd->nu_cdev->use_optee)
			tctx->hmac_key_len = 0;
		ctx->bufcnt = 0;
		return 0;
	}

	/* is HMAC, check key length */
	if (((tctx->hmac_key_len + ctx->block_size - 1) >
		HMAC_KEY_BUFF_SIZE) ||	(tctx->hmac_key_len == 0)) {
		pr_err("HMAC key length %d is not supported!\n",
				tctx->hmac_key_len);
		if (dd->nu_cdev->use_optee) {
			kfree(ctx->buffer);
			ctx->buffer = NULL;
		}
		return -EINVAL;
	}

	ctx->flags |= SHA_FLAGS_KEY_BLK;
	if (dd->nu_cdev->use_optee)
		return 0;

	/* Preserve direct-hardware request-time key padding. */
	klen = tctx->hmac_key_len;
	if ((klen % ctx->block_size) != 0) {
		/* Paading zeros to make key data be block aligned */
		plen = ctx->block_size - (klen % ctx->block_size);
		memset(&tctx->keybuf[tctx->keybufcnt], 0, plen);
		tctx->keybufcnt += plen;
	}
	return 0;
}

static void nuvoton_sha_sg_to_dma_buffer(struct ahash_request *req, struct nu_sha_reqctx *ctx)
{
	int	copy_len;

	while (ctx->sg && (ctx->req_len > 0) &&
		(ctx->bufcnt < ctx->dma_max_size)) {
		copy_len = min((int)ctx->sg->length - ctx->sg_off,
				ctx->req_len);
		if (ctx->dma_max_size - ctx->bufcnt < copy_len)
			copy_len = ctx->dma_max_size - ctx->bufcnt;

		memcpy(&ctx->buffer[ctx->bufcnt], (u8 *)sg_virt(ctx->sg)
				+ ctx->sg_off, copy_len);

		ctx->bufcnt += copy_len;
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
	int err = 0;

	if (dd->nu_cdev->use_optee &&
	    (READ_ONCE(nuvoton_crypto_optee_faulted) ||
	     (!(ctx->flags & SHA_FLAGS_FIRST) &&
	      !nuvoton_sha_tee_session_open(dd, ctx)))) {
		nuvoton_sha_finish_req(ctx, -EIO);
		return -EIO;
	}
	if ((ctx->req_len > 0) &&  (ctx->bufcnt < ctx->dma_max_size))
		nuvoton_sha_sg_to_dma_buffer(dd->req, ctx);

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		if ((ctx->flags & (SHA_FLAGS_FINUP | SHA_FLAGS_FINAL)) &&
		    (ctx->bufcnt == 0) && (dd->req->nbytes == 0)) {
			pr_err("MA35D1 HMAC does not support 0 data length!\n");
			nuvoton_sha_finish_req(ctx, -EINVAL);
			return -EINVAL;
		}
		err = nuvoton_sha_dma_run(dd, 1);
		if (err != -EINPROGRESS) {
			/* DMA trigger failed, abort! */
			nuvoton_sha_finish_req(ctx, err);
		}
	} else if (ctx->bufcnt == ctx->dma_max_size) {
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
	unsigned long		flags;
	int			ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req) {
		if (dd->nu_cdev->use_optee && dd->stopping) {
			spin_unlock_irqrestore(&dd->lock, flags);
			return -ESHUTDOWN;
		}
		ret = ahash_enqueue_request(&dd->queue, req);
		if (dd->nu_cdev->use_optee) {
			queue_work(dd->tee_wq, &dd->tee_queue_work);
			spin_unlock_irqrestore(&dd->lock, flags);
			return ret;
		}
	}

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
	dd->req = req;

	return nuvoton_sha_update_start(dd);
}

static void nuvoton_sha_dma_complete(struct nu_sha_reqctx *ctx)
{
	struct nu_sha_dev *dd = ctx->dd;

	ctx->flags &= ~SHA_FLAGS_FIRST;     /* clear FIRST flag anyway     */

	if (ctx->flags & SHA_FLAGS_KEY_BLK) {
		ctx->flags &= ~SHA_FLAGS_KEY_BLK;
		nuvoton_sha_update_start(dd);
		return;
	}
	ctx->bufcnt = 0;	    /* reset DMA buffer count      */
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

	ctx->sg = req->src;
	ctx->sg_off = 0;
	ctx->req_len = req->nbytes;

	nuvoton_sha_sg_to_dma_buffer(req, ctx);
	if (ctx->bufcnt + ctx->req_len <= ctx->dma_max_size)
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

	/* A rejected OP-TEE update was not queued; do not submit it again. */
	if (err1 && ctx->dd->nu_cdev->use_optee)
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
	unsigned int block_size;
	unsigned int padded_len;

	if (tctx->dd && tctx->dd->nu_cdev->use_optee) {
		block_size = crypto_ahash_blocksize(tfm);
		if (keylen > HMAC_KEY_BUFF_SIZE ||
		    (keylen && keylen + block_size - 1 > HMAC_KEY_BUFF_SIZE))
			return -EINVAL;

		padded_len = round_up(keylen, block_size);
		if (keylen)
			memcpy(tctx->keybuf, key, keylen);
		memset(tctx->keybuf + keylen, 0, padded_len - keylen);
		tctx->hmac_key_len = keylen;
		tctx->keybufcnt = padded_len;
		return 0;
	}

	/* Preserve the original direct-hardware setkey path. */
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
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	const struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	if (tctx->dd && tctx->dd->nu_cdev->use_optee)
		return -EOPNOTSUPP;

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int nuvoton_sha_import(struct ahash_request *req, const void *in)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct nu_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct nu_sha_reqctx *ctx = ahash_request_ctx(req);

	if (tctx->dd && tctx->dd->nu_cdev->use_optee)
		return -EOPNOTSUPP;

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

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm), sizeof(struct nu_sha_reqctx));
	return 0;
}

static int nuvoton_sha_cra_init(struct crypto_tfm *tfm)
{
	return nuvoton_sha_cra_init_alg(tfm, NULL);
}

static void nuvoton_sha_cra_exit(struct crypto_tfm *tfm)
{
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
	int map_size;

	if (dd->nu_cdev->use_optee) {
		nuvoton_sha_tee_unmap(dd);
		nuvoton_sha_dma_complete(ctx);
		return;
	}

	/* Preserve the original direct-hardware DMA completion path. */
	if (ctx->flags & SHA_FLAGS_KEY_BLK)
		map_size = HMAC_KEY_BUFF_SIZE;
	else
		map_size = SHA_BUFF_SIZE;
	dma_unmap_single(dd->dev, ctx->dma_fdbck, SHA_FDBCK_SIZE,
			 DMA_BIDIRECTIONAL);
	if (ctx->dma_buff != 0)
		dma_unmap_single(dd->dev, ctx->dma_buff, map_size, DMA_TO_DEVICE);
	nuvoton_sha_dma_complete(ctx);
}

static void nuvoton_sha_tee_queue_work(struct work_struct *work)
{
	struct nu_sha_dev *dd = container_of(work, struct nu_sha_dev,
					   tee_queue_work);

	nuvoton_sha_handle_queue(dd, NULL);
}

static void nuvoton_sha_tee_done_work(struct work_struct *work)
{
	struct nu_sha_dev *dd = container_of(work, struct nu_sha_dev,
					   tee_done_work);

	nuvoton_sha_done_task((unsigned long)dd);
}

static void nuvoton_sha_tee_stop(struct nu_sha_dev *dd)
{
	unsigned long flags;

	spin_lock_irqsave(&dd->lock, flags);
	dd->stopping = true;
	spin_unlock_irqrestore(&dd->lock, flags);
	if (dd->tee_wq) {
		destroy_workqueue(dd->tee_wq);
		dd->tee_wq = NULL;
		nuvoton_sha_tee_close_all(dd);
	}
	tasklet_kill(&dd->done_task);
	tasklet_kill(&dd->queue_task);
}

static int nuvoton_sha_probe_direct(struct device *dev,
				    struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev *sha_dd = &nu_cryp_dev->sha_dd;
	int i, err = 0;

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

static int nuvoton_sha_remove_direct(struct device *dev,
				     struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev *sha_dd = &nu_cryp_dev->sha_dd;
	int i;

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
	return 0;
}

int nuvoton_sha_probe(struct device *dev, struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev *sha_dd = &nu_cryp_dev->sha_dd;
	int i, j, err;

	if (!nu_cryp_dev->use_optee)
		return nuvoton_sha_probe_direct(dev, nu_cryp_dev);

	sha_dd->dev = dev;
	sha_dd->nu_cdev = nu_cryp_dev;
	sha_dd->reg_base = nu_cryp_dev->reg_base;
	sha_dd->octx = NULL;
	INIT_LIST_HEAD(&sha_dd->tee_sessions);

#ifdef CONFIG_OPTEE
	if (nu_cryp_dev->use_optee) {
		struct tee_ioctl_open_session_arg sess_arg = { };

		err = nuvoton_crypto_optee_init(nu_cryp_dev);
		if (err)
			return err;
		if (!nu_cryp_dev->tee_cdev)
			return -EPROBE_DEFER;

		sha_dd->octx = tee_client_open_context(NULL, optee_ctx_match,
						     NULL, NULL);
		if (IS_ERR(sha_dd->octx))
			return PTR_ERR(sha_dd->octx);

		memcpy(sess_arg.uuid, nu_cryp_dev->tee_cdev->id.uuid.b,
		       TEE_IOCTL_UUID_LEN);
		sess_arg.clnt_login = TEE_IOCTL_LOGIN_PUBLIC;
		err = tee_client_open_session(sha_dd->octx, &sess_arg, NULL);
		if (err < 0 || sess_arg.ret) {
			err = err < 0 ? err : -EIO;
			goto out_ctx;
		}
		sha_dd->session_id = sess_arg.session;
		sha_dd->shm_pool = tee_shm_alloc(sha_dd->octx, CRYPTO_SHM_SIZE,
					       TEE_SHM_MAPPED | TEE_SHM_DMA_BUF);
		if (IS_ERR(sha_dd->shm_pool)) {
			err = PTR_ERR(sha_dd->shm_pool);
			goto out_sess;
		}
		sha_dd->va_shm = tee_shm_get_va(sha_dd->shm_pool, 0);
		if (IS_ERR(sha_dd->va_shm)) {
			err = PTR_ERR(sha_dd->va_shm);
			goto out_shm;
		}
	}
#endif

	INIT_LIST_HEAD(&sha_dd->list);
	spin_lock_init(&sha_dd->lock);
	tasklet_init(&sha_dd->done_task, nuvoton_sha_done_task,
		     (unsigned long)sha_dd);
	tasklet_init(&sha_dd->queue_task, nuvoton_sha_queue_task,
		     (unsigned long)sha_dd);
	INIT_WORK(&sha_dd->tee_queue_work, nuvoton_sha_tee_queue_work);
	INIT_WORK(&sha_dd->tee_done_work, nuvoton_sha_tee_done_work);
	crypto_init_queue(&sha_dd->queue, 32);
	if (nu_cryp_dev->use_optee) {
		sha_dd->tee_wq = alloc_ordered_workqueue("ma35-sha", WQ_MEM_RECLAIM);
		if (!sha_dd->tee_wq) {
			err = -ENOMEM;
			goto err_tee;
		}
	}
	spin_lock(&nu_sha.lock);
	list_add_tail(&sha_dd->list, &nu_sha.dev_list);
	spin_unlock(&nu_sha.lock);

	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++) {
		err = crypto_register_ahash(&nuvoton_sha_algs[i]);
		if (err)
			goto err_algs;
	}
	for (j = 0; j < ARRAY_SIZE(nuvoton_sha3_algs); j++) {
		err = crypto_register_ahash(&nuvoton_sha3_algs[j]);
		if (err)
			goto err_sha3;
	}
	sha_dd->registered = true;
	pr_info("MA35D1 Crypto SHA engine enabled.\n");
	return 0;

err_sha3:
	while (j--)
		crypto_unregister_ahash(&nuvoton_sha3_algs[j]);
err_algs:
	while (i--)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);
	nuvoton_sha_tee_stop(sha_dd);
	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);
err_tee:
#ifdef CONFIG_OPTEE
out_shm:
	if (nu_cryp_dev->use_optee)
		tee_shm_free(sha_dd->shm_pool);
out_sess:
	if (nu_cryp_dev->use_optee)
		tee_client_close_session(sha_dd->octx, sha_dd->session_id);
out_ctx:
	if (nu_cryp_dev->use_optee)
		tee_client_close_context(sha_dd->octx);
#endif
	return err;
}

int nuvoton_sha_remove(struct device *dev, struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_sha_dev *sha_dd = &nu_cryp_dev->sha_dd;
	int i;

	if (!nu_cryp_dev->use_optee)
		return nuvoton_sha_remove_direct(dev, nu_cryp_dev);

	if (!sha_dd->registered)
		return 0;

	nuvoton_sha_tee_stop(sha_dd);
	for (i = 0; i < ARRAY_SIZE(nuvoton_sha_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha_algs[i]);
	for (i = 0; i < ARRAY_SIZE(nuvoton_sha3_algs); i++)
		crypto_unregister_ahash(&nuvoton_sha3_algs[i]);

	spin_lock(&nu_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&nu_sha.lock);
#ifdef CONFIG_OPTEE
	if (nu_cryp_dev->use_optee) {
		tee_shm_free(sha_dd->shm_pool);
		tee_client_close_session(sha_dd->octx, sha_dd->session_id);
		tee_client_close_context(sha_dd->octx);
	}
#endif
	sha_dd->registered = false;
	return 0;
}
