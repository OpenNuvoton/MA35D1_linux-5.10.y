// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-ecc.c
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
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
#include <crypto/internal/kpp.h>
#include <crypto/kpp.h>
#include <crypto/ecdh.h>

#include <linux/io.h>
#include <linux/clk.h>

#include "nuvoton-crypto.h"

/* macro defined in crypto/ecc.h */
#define ECC_CURVE_NIST_P192_DIGITS  3
#define ECC_CURVE_NIST_P256_DIGITS  4

static struct ecc_curve curve_p192 = {
	CURVE_P_192,
	"P-192",
	24,
	"\x18\x8d\xa8\x0e\xb0\x30\x90\xf6\x7c\xbf\x20\xeb\x43\xa1\x88\x00\xf4\xff\x0a\xfd\x82\xff\x10\x12",
	"\x07\x19\x2b\x95\xff\xc8\xda\x78\x63\x10\x11\xed\x6b\x24\xcd\xd5\x73\xf9\x77\xa1\x1e\x79\x48\x11",
	"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",
	"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x99\xDE\xF8\x36\x14\x6B\xC9\xB1\xB4\xD2\x28\x31",
	"\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC",
	"\x64\x21\x05\x19\xe5\x9c\x80\xe7\x0f\xa7\xe9\xab\x72\x24\x30\x49\xfe\xb8\xde\xec\xc1\x46\xb9\xb1",
};

static struct ecc_curve curve_p256 = {
	CURVE_P_256,
	"P-256",
	32,
	"\x6b\x17\xd1\xf2\xe1\x2c\x42\x47\xf8\xbc\xe6\xe5\x63\xa4\x40\xf2\x77"
	"\x03\x7d\x81\x2d\xeb\x33\xa0\xf4\xa1\x39\x45\xd8\x98\xc2\x96",
	"\x4f\xe3\x42\xe2\xfe\x1a\x7f\x9b\x8e\xe7\xeb\x4a\x7c\x0f\x9e\x16\x2b"
	"\xce\x33\x57\x6b\x31\x5e\xce\xcb\xb6\x40\x68\x37\xbf\x51\xf5",
	"\xFF\xFF\xFF\xFF\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00"
	"\x00\x00\x00\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF",
	"\xFF\xFF\xFF\xFF\x00\x00\x00\x00\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xBC"
	"\xE6\xFA\xAD\xA7\x17\x9E\x84\xF3\xB9\xCA\xC2\xFC\x63\x25\x51",
	"\xFF\xFF\xFF\xFF\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00"
	"\x00\x00\x00\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC",
	"\x5a\xc6\x35\xd8\xaa\x3a\x93\xe7\xb3\xeb\xbd\x55\x76\x98\x86\xbc\x65"
	"\x1d\x06\xb0\xcc\x53\xb0\xf6\x3b\xce\x3c\x3e\x27\xd2\x60\x4b",
};

struct nu_ecc_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

static struct nu_ecc_drv nu_ecc = {
	.dev_list = LIST_HEAD_INIT(nu_ecc.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(nu_ecc.lock),
};

static struct nu_ecc_dev *nuvoton_ecc_find_dev(struct nu_ecdh_ctx *tctx)
{
	struct nu_ecc_dev *dd = NULL;
	struct nu_ecc_dev *tmp;

	spin_lock_bh(&nu_ecc.lock);
	if (!tctx->dd) {
		list_for_each_entry(tmp, &nu_ecc.dev_list, list) {
			dd = tmp;
			break;
		}
		tctx->dd = dd;
	} else {
		dd = tctx->dd;
	}
	spin_unlock_bh(&nu_ecc.lock);
	return dd;
}

static inline void nu_write_reg(struct nu_ecc_dev *ecc_dd, u32 val, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (ecc_dd->use_optee == true)
		ecc_dd->va_shm[reg / 4] = val;
	else
		writel_relaxed(val, ecc_dd->reg_base + reg);
#else
	writel_relaxed(val, ecc_dd->reg_base + reg);
#endif
}

static inline u32 nu_read_reg(struct nu_ecc_dev *ecc_dd, u32 reg)
{
#ifdef CONFIG_OPTEE
	if (ecc_dd->use_optee == true)
		return ecc_dd->va_shm[reg / 4];
	else
		return readl_relaxed(ecc_dd->reg_base + reg);
#else
	return readl_relaxed(ecc_dd->reg_base + reg);
#endif
}

static inline void nu_write_ecc_key(struct nu_ecdh_ctx *ctx, u8 *key, u32 reg)
{
	struct nu_ecc_dev *dd = ctx->dd;
	u32	val32;
	int	i, idx;

	idx = (ctx->keylen/4)-1;
	for (i = 0; i < ctx->keylen; i += 4) {
		val32 = (key[i]<<24) | (key[i+1]<<16) |
			(key[i+2]<<8) | (key[i+3]);
		nu_write_reg(dd, val32, reg + idx*4);
		idx--;
	}
}

static inline void nu_read_ecc_key(struct nu_ecdh_ctx *ctx, u32 reg, u8 *key)
{
	struct nu_ecc_dev *dd = ctx->dd;
	u32	val32;
	int	i, idx;

	idx = (ctx->keylen/4)-1;
	for (i = 0; i < ctx->keylen; i += 4) {
		val32 = nu_read_reg(dd, reg + idx*4);
		idx--;
		key[i]   = (val32>>24) & 0xff;
		key[i+1] = (val32>>16) & 0xff;
		key[i+2] = (val32>>8) & 0xff;
		key[i+3] = val32 & 0xff;
	}
}

#ifdef CONFIG_OPTEE
static const char hex_char_tbl[] = "0123456789abcdef";

static void ecc_key_to_str(u8 *key, char *kstr, int keylen)
{
	int   i;

	for (i = 0; i < keylen; i++) {
		*kstr++ = hex_char_tbl[(*key >> 4) & 0xf];
		*kstr++ = hex_char_tbl[*key & 0xf];
		key++;
	}
	*kstr++ = 0;
	*kstr = 0;
}
#endif

static inline int get_nibble_value(char c)
{
	if ((c >= '0') && (c <= '9'))
		c = c - '0';

	if ((c >= 'a') && (c <= 'f'))
		c = c - 'a' + (char)10;

	if ((c >= 'A') && (c <= 'F'))
		c = c - 'A' + (char)10;

	return (int)c;
}

#ifdef FOR_ECC_TRY
static void ecc_str_to_key(char *kstr, u8 *key, int klen)
{
	for ( ; klen > 0; klen--, key++) {
		*key = get_nibble_value(*kstr);
		kstr++;
		*key = (*key << 4) + get_nibble_value(*kstr);
		kstr++;
	}
}
#endif

static inline int nuvoton_wait_ecc_complete(struct nu_ecc_dev *dd, int timeout)
{
	while (nu_read_reg(dd, ECC_STS) & (ECC_STS_BUSY | ECC_STS_DMABUSY)) {

		if (time_after(jiffies, jiffies + msecs_to_jiffies(timeout))) {
			nu_write_reg(dd, ECC_CTL_STOP, ECC_CTL);
			pr_err("ECC hardware timeout! 0x%x\n",
				nu_read_reg(dd, ECC_STS));
			return -EBUSY;
		}

		if (nu_read_reg(dd, ECC_STS) &
			(ECC_STS_BUSERR | ECC_STS_KSERR)) {
			pr_err("ECC hardware error! 0x%x\n",
				nu_read_reg(dd, ECC_STS));
			return -EINVAL;
		}
	}
	return 0;
}

static int nuvoton_ecc_init_curve(int curve_id, struct nu_ecdh_ctx *ctx)
{
	struct ecc_curve  *curve;

	if (curve_id == ECC_CURVE_NIST_P192)
		curve = &curve_p192;
	else if (curve_id == ECC_CURVE_NIST_P256)
		curve = &curve_p256;
	else {
		pr_err("[%s] - invalid curve ID!\n", __func__);
		return -EINVAL;
	}

	ctx->curve_id = curve_id;
	ctx->curve = curve;
	ctx->keylen = curve->keylen;

	nu_write_ecc_key(ctx, curve->g_x, ECC_X1);
	nu_write_ecc_key(ctx, curve->g_y, ECC_Y1);
	nu_write_ecc_key(ctx, curve->a, ECC_A);
	nu_write_ecc_key(ctx, curve->b, ECC_B);
	nu_write_ecc_key(ctx, curve->p, ECC_N);

	return 0;
}

static int nuvoton_ecc_point_mult(struct nu_ecdh_ctx *ctx, u8 *private_key,
					u8 *public_key, u8 *result)
{
	struct nu_ecc_dev  *dd = ctx->dd;
#ifdef CONFIG_OPTEE
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
#endif
	int	err;

	if (public_key != NULL) {
		nu_write_ecc_key(ctx, public_key, ECC_X1);
		nu_write_ecc_key(ctx, public_key + ctx->keylen, ECC_Y1);

	} else {
		/* use G point */
	}

	nu_write_reg(dd, 0, ECC_KSCTL);
	nu_write_reg(dd, 0, ECC_KSXY);

	nu_write_ecc_key(ctx, private_key, ECC_K);

	nu_write_reg(dd, ((ctx->keylen*8) << ECC_CTL_CURVEM_OFFSET) |
		ECC_CTL_FSEL | ECCOP_POINT_MUL | ECC_CTL_START, ECC_CTL);

	if (dd->use_optee == false) {
		err = nuvoton_wait_ecc_complete(dd, 1000);
		if (err)
			return err;
	} else {
#ifdef CONFIG_OPTEE
		/*
		 *  Invoke OP-TEE Crypto PTA to run ECC
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_ECC_PMUL function of PTA */
		inv_arg.func = PTA_CMD_CRYPTO_ECC_PMUL;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;
		param[2].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;

		param[0].u.value.a = ctx->curve->optee_curve_id;
		param[1].u.memref.shm = dd->shm_pool;
		param[1].u.memref.size = CRYPTO_SHM_SIZE;
		param[1].u.memref.shm_offs = 0;
		param[2].u.value.a = 0x1000;
		param[2].u.value.b = 0x2000;

		memset(&dd->va_shm[0x1000 / 4], 0, 0x2000);
		if (public_key != NULL) {
			ecc_key_to_str(public_key,
				(char *)&dd->va_shm[0x1000 / 4], ctx->keylen);
			ecc_key_to_str(public_key + ctx->keylen,
				(char *)&dd->va_shm[0x1240 / 4], ctx->keylen);
		} else {
			ecc_key_to_str((u8 *)ctx->curve->g_x,
				(char *)&dd->va_shm[0x1000 / 4], ctx->keylen);
			ecc_key_to_str((u8 *)ctx->curve->g_y,
				(char *)&dd->va_shm[0x1240 / 4], ctx->keylen);
		}
		ecc_key_to_str(private_key,
				(char *)&dd->va_shm[0x1480 / 4], ctx->keylen);

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_ECC_PMUL err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}

		//ecc_str_to_key((char *)&dd->va_shm[0x2000 / 4],
		//		 (u8 *)&dd->va_shm[0x1000 / 4], ctx->keylen);
		//nu_read_ecc_key(ctx, 0x1000, result);

		//ecc_str_to_key((char *)&dd->va_shm[0x2240 / 4],
		//		 (u8 *)&dd->va_shm[0x1000 / 4], ctx->keylen);
		//nu_read_ecc_key(ctx, 0x1000, result + ctx->keylen);
#endif
	}
	nu_read_ecc_key(ctx, ECC_X1, result);
	nu_read_ecc_key(ctx, ECC_Y1, result + ctx->keylen);
	return 0;
}

static int nuvoton_ecdh_set_secret(struct crypto_kpp *tfm, const void *buf,
				   unsigned int len)
{
	struct nu_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	struct nu_ecc_dev *dd = nuvoton_ecc_find_dev(ctx);
	struct ecdh	params;
	unsigned int	ndigits;
	int		err;

	ctx->dd = dd;

	if (crypto_ecdh_decode_key(buf, len, &params) < 0) {
		pr_err("crypto_ecdh_decode_key failed!\n");
		return -EINVAL;
	}

	if (params.curve_id == ECC_CURVE_NIST_P192)
		ndigits = ECC_CURVE_NIST_P192_DIGITS;
	else if (params.curve_id == ECC_CURVE_NIST_P256)
		ndigits = ECC_CURVE_NIST_P256_DIGITS;
	else {
		pr_err("%s - Invalid curve id!\n", __func__);
		return -EINVAL;
	}

	err = nuvoton_ecc_init_curve(params.curve_id, ctx);
	if (err)
		return err;

	if (!params.key || !params.key_size)
		return ecc_gen_privkey(params.curve_id, ndigits,
				       (u64 *)ctx->private_key);

	if (ecc_is_key_valid(params.curve_id, ndigits,
			     (const u64 *)params.key, params.key_size) < 0) {
		pr_err("[%s] - input key is invalid!\n", __func__);
		return -EINVAL;
	}

	memcpy(ctx->private_key, params.key, params.key_size);
	return 0;
}

static int nuvoton_ecdh_compute_value(struct kpp_request *req)
{
	struct crypto_kpp *tfm = crypto_kpp_reqtfm(req);
	struct nu_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);
	struct nu_ecc_dev *dd = ctx->dd;
	u8	public_key[NU_ECC_MAX_LEN * 2];
	u8	shared_secret[NU_ECC_MAX_LEN * 2];
	void	*buf;
	int	copied, nbytes, public_key_sz;
	int	ret;

	ctx->dd = dd;

	nuvoton_ecc_init_curve(ctx->curve_id, ctx);

	nbytes = ctx->keylen;

	/* Public part is a point thus it has both coordinates */
	public_key_sz = 2 * nbytes;

	if (req->src) {
		/* must have exactly two points to be on the curve */
		if (public_key_sz != req->src_len)
			return -EINVAL;

		copied = sg_copy_to_buffer(req->src,
					   sg_nents_for_len(req->src,
							    public_key_sz),
					   public_key, public_key_sz);
		if (copied != public_key_sz)
			return -EINVAL;

		ret = nuvoton_ecc_point_mult(ctx, ctx->private_key,
						public_key, shared_secret);

		buf = shared_secret;
	} else {
		/* Make public key by private key and G */
		ret = nuvoton_ecc_point_mult(ctx, ctx->private_key,
						NULL, public_key);
		buf = public_key;
		nbytes = public_key_sz;
	}

	if (ret < 0)
		return ret;

	/* might want less than we've got */
	nbytes = min_t(size_t, nbytes, req->dst_len);
	copied = sg_copy_from_buffer(req->dst,
				sg_nents_for_len(req->dst, nbytes),
				buf, nbytes);
	if (copied != nbytes)
		ret = -EINVAL;

	return ret;
}

static unsigned int nuvoton_ecdh_max_size(struct crypto_kpp *tfm)
{
	struct nu_ecdh_ctx *ctx = kpp_tfm_ctx(tfm);

	/* Public key is made of two coordinates, add one to the left shift */
	return (ctx->keylen * 2);
}

static struct kpp_alg nuvoton_ecdh = {
	.set_secret = nuvoton_ecdh_set_secret,
	.generate_public_key = nuvoton_ecdh_compute_value,
	.compute_shared_secret = nuvoton_ecdh_compute_value,
	.max_size = nuvoton_ecdh_max_size,
	.base = {
		.cra_flags = CRYPTO_ALG_NEED_FALLBACK,
		.cra_name = "ecdh",
		.cra_driver_name = "nuvoton-ecdh",
		.cra_priority = 400,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct nu_ecdh_ctx),
	},
};

#ifdef CONFIG_OPTEE
static int  optee_ecc_open(struct nu_ecc_dev *dd)
{
	struct tee_ioctl_open_session_arg sess_arg;
	int   err;

	/*
	 * Open RSA context with TEE driver
	 */
	dd->octx = tee_client_open_context(NULL, optee_ctx_match,
					       NULL, NULL);
	if (IS_ERR(dd->octx)) {
		pr_err("%s open context failed, err: %x\n", __func__,
			sess_arg.ret);
		return err;
	}

	/*
	 * Open ECC session with Crypto Trusted App
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

static void optee_ecc_close(struct nu_ecc_dev *dd)
{
	tee_shm_free(dd->shm_pool);
	tee_client_close_session(dd->octx, dd->session_id);
	tee_client_close_context(dd->octx);
}
#endif

int nuvoton_ecc_probe(struct device *dev,
		      struct nuvoton_crypto_dev *nu_cryp_dev)
{
	struct nu_ecc_dev  *ecc_dd = &nu_cryp_dev->ecc_dd;
	int   i, err = 0;

	ecc_dd->dev = dev;
	ecc_dd->reg_base = nu_cryp_dev->reg_base;
	ecc_dd->use_optee = false;

#ifdef CONFIG_OPTEE
	ecc_dd->use_optee = nu_cryp_dev->use_optee;
	ecc_dd->tee_cdev = nu_cryp_dev->tee_cdev;
	if (ecc_dd->use_optee) {
		if (optee_ecc_open(ecc_dd) != 0)
			return -ENODEV;
	}
#endif

	/* clear  all ECC curve parameter registers */
	for (i = 0; i < ECC_KEY_WCNT; i++) {
		nu_write_reg(ecc_dd, 0, ECC_X1);
		nu_write_reg(ecc_dd, 0, ECC_Y1);
		nu_write_reg(ecc_dd, 0, ECC_X2);
		nu_write_reg(ecc_dd, 0, ECC_Y2);
		nu_write_reg(ecc_dd, 0, ECC_A);
		nu_write_reg(ecc_dd, 0, ECC_B);
		nu_write_reg(ecc_dd, 0, ECC_N);
		nu_write_reg(ecc_dd, 0, ECC_K);
	}

	INIT_LIST_HEAD(&ecc_dd->list);
	spin_lock_init(&ecc_dd->lock);

	spin_lock(&nu_ecc.lock);
	list_add_tail(&ecc_dd->list, &nu_ecc.dev_list);
	spin_unlock(&nu_ecc.lock);

	err = crypto_register_kpp(&nuvoton_ecdh);
	if (err) {
		dev_err(dev, "register ecdh failed\n");
		goto err_out;
	}

	pr_info("MA35D1 Crypto ECC engine enabled.\n");
	return 0;

err_out:
	spin_lock(&nu_ecc.lock);
	list_del(&ecc_dd->list);
	spin_unlock(&nu_ecc.lock);

	dev_err(dev, "MA35D1 ECC initialization failed. %d\n", err);
	return err;
}

int nuvoton_ecc_remove(struct device *dev,
		       struct nuvoton_crypto_dev *nu_cryp_dev)
{
	struct nu_ecc_dev  *ecc_dd = &nu_cryp_dev->ecc_dd;

	if (ecc_dd == NULL)
		return -ENODEV;

	spin_lock(&nu_ecc.lock);
	list_del(&ecc_dd->list);
	spin_unlock(&nu_ecc.lock);

#ifdef CONFIG_OPTEE
	if (ecc_dd->use_optee)
		optee_ecc_close(ecc_dd);
#endif
	return 0;
}


