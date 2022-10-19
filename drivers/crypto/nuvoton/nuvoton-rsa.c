// SPDX-License-Identifier: GPL-2.0
/*
 * linux/driver/crypto/nuvoton/nuvoton-rsac
 *
 * Copyright (c) 2020 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/tee_drv.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <linux/scatterlist.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/algapi.h>
#include <crypto/akcipher.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>

#include "nuvoton-crypto.h"

static int  optee_rsa_open(struct nu_rsa_dev *dd);
static struct nu_rsa_dev  *__rsa_dd;

struct nu_rsa_drv {
	struct list_head dev_list;
	/* Device list lock */
	spinlock_t lock;
};

static struct nu_rsa_drv  nu_rsa = {
	.dev_list = LIST_HEAD_INIT(nu_rsa.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(nu_rsa.lock),
};

static struct nu_rsa_dev *nuvoton_rsa_find_dev(struct nu_rsa_ctx *ctx)
{
	struct nu_rsa_dev *dd = NULL;
	struct nu_rsa_dev *tmp;

	spin_lock_bh(&nu_rsa.lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &nu_rsa.dev_list, list) {
			dd = tmp;
			break;
		}
		ctx->dd = dd;
	} else {
		dd = ctx->dd;
	}
	spin_unlock_bh(&nu_rsa.lock);
	return dd;
}

static inline void nu_write_reg(struct nu_rsa_dev *rsa_dd, u32 val, u32 reg)
{
	if (rsa_dd->nu_cdev->use_optee == true)
		rsa_dd->va_shm[reg/4] = val;
	else
		writel_relaxed(val, rsa_dd->reg_base + reg);
}

static inline u32 nu_read_reg(struct nu_rsa_dev *rsa_dd, u32 reg)
{
	if (rsa_dd->nu_cdev->use_optee == true)
		return rsa_dd->va_shm[reg/4];
	else
		return readl_relaxed(rsa_dd->reg_base + reg);
}

static int check_key_length(u8 *input, int keysz)
{
	u8	*key;

	for (key = input; (*key == 0x00); key++)
		keysz--;
	return keysz * 8;
}

static int check_ras_length(struct rsa_key *raw_key)
{
	int  rsa_len;

	rsa_len = check_key_length((u8 *)raw_key->n, raw_key->n_sz);

	if ((rsa_len != 1024) && (rsa_len != 2048) &&
		(rsa_len != 3072) && (rsa_len != 4096)) {
		pr_err("rsa length %d is not supported!\n", rsa_len);
		return -EINVAL;
	}
	return rsa_len;
}

int nu_rsa_hex_to_reg(u8 *input, int keylen, int rsa_bit_len, u32 *reg)
{
	int	rsa_byte_len = rsa_bit_len/8;
	u8	*key;
	int	idx;
	u8	buff[NU_RSA_MAX_BYTE_LEN];

	memset(buff, 0, sizeof(buff));

	/* remove leading 0x00's */
	for (key = input; ((*key == 0x00) && (keylen > 0)); key++)
		keylen--;

	if (keylen > rsa_byte_len)
		return -EINVAL;

	memcpy(&buff[rsa_byte_len - keylen], key, keylen);

	for (idx = rsa_byte_len - 1; idx > 0; idx -= 4) {
		*reg++ = (buff[idx - 3] << 24) | (buff[idx - 2] << 16) |
				(buff[idx - 1] << 8) | buff[idx];
	}
	return 0;
}

void nu_rsa_reg_to_hex(u32 *reg, int rsa_bit_len, u8 *key, int *key_sz)
{
	int	i, idx;

	*key_sz = 0;
	for (idx = rsa_bit_len/32 - 1 ; idx >= 0; idx--) {
		for (i = 3; i >= 0; i--) {
			*key++ = (reg[idx] >> (i*8)) & 0xff;
			(*key_sz)++;
			if (*key_sz >= NU_RSA_MAX_BYTE_LEN)
				return;
		}
	}
}

static int nuvoton_rsa_sg_to_buffer(struct scatterlist *sgl,
				    u8 *buff, int max_len)
{
	int	total = 0, copy_len;

	while (sgl && (total < max_len)) {
		copy_len = min((int)sgl->length, max_len - total);
		memcpy(buff + total, (u8 *)sg_virt(sgl), copy_len);
		total += copy_len;
		sgl = sg_next(sgl);
	}
	return total;
}

static int nuvoton_rsa_buffer_to_sg(u8 *buff, int data_cnt,
				    struct scatterlist *sgl)
{
	int	total = 0, copy_len;

	while (sgl && (data_cnt > 0)) {
		copy_len = min_t(int, sgl->length, data_cnt);
		memcpy((u8 *)sg_virt(sgl), buff + total, copy_len);
		total += copy_len;
		data_cnt -= copy_len;
		sgl = sg_next(sgl);
	}
	return total;
}

static int nuvoton_rsa_enc(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct nu_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct nu_rsa_dev *dd = nuvoton_rsa_find_dev(ctx);
	u8	m[NU_RSA_MAX_BYTE_LEN];
	u32	keyleng;
	int	len, err;
	unsigned long   timeout;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if ((dd->nu_cdev->use_optee) && (dd->octx == NULL)) {
		if (optee_rsa_open(dd) != 0)
			return -ENODEV;
	}
	ctx->dd = dd;
	keyleng = ctx->rsa_bit_len/1024 - 1;

	memset(&ctx->buffer[E_OFF], 0, NU_RSA_MAX_BYTE_LEN);
	err = nu_rsa_hex_to_reg((u8 *)ctx->public_key,
				ctx->public_key_size, ctx->rsa_bit_len,
				(u32 *)&ctx->buffer[E_OFF]);
	if (err)
		return err;

	len = nuvoton_rsa_sg_to_buffer(req->src, m,
				min((int)req->src_len, NU_RSA_MAX_BYTE_LEN));

	pr_debug("[%s] - req_src byte length: %d\n", __func__, len);

	err = nu_rsa_hex_to_reg(m, len, ctx->rsa_bit_len,
				(u32 *)&ctx->buffer[M_OFF]);
	if (err)
		return err;

	ctx->dma_buff = dma_map_single(dd->dev, ctx->buffer,
				RSA_BUFF_SIZE, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dd->dev, ctx->dma_buff))) {
		dev_err(dd->dev, "RSA buffer dma map error\n");
		return -EINVAL;
	}

	nu_write_reg(dd, 0, RSA_KSCTL);
	nu_write_reg(dd, 0, RSA_KSSTS0);
	nu_write_reg(dd, 0, RSA_KSSTS1);

	nu_write_reg(dd, ctx->dma_buff + M_OFF,   RSA_SADDR0);
	nu_write_reg(dd, ctx->dma_buff + N_OFF,   RSA_SADDR1);
	nu_write_reg(dd, ctx->dma_buff + E_OFF,   RSA_SADDR2);
	nu_write_reg(dd, ctx->dma_buff + P_OFF,   RSA_SADDR3);
	nu_write_reg(dd, ctx->dma_buff + Q_OFF,   RSA_SADDR4);
	nu_write_reg(dd, ctx->dma_buff + ANS_OFF, RSA_DADDR);
	nu_write_reg(dd, ctx->dma_buff + MADR0_OFF, RSA_MADDR0);
	nu_write_reg(dd, ctx->dma_buff + MADR1_OFF, RSA_MADDR1);
	nu_write_reg(dd, ctx->dma_buff + MADR2_OFF, RSA_MADDR2);
	nu_write_reg(dd, ctx->dma_buff + MADR3_OFF, RSA_MADDR3);
	nu_write_reg(dd, ctx->dma_buff + MADR4_OFF, RSA_MADDR4);
	nu_write_reg(dd, ctx->dma_buff + MADR5_OFF, RSA_MADDR5);
	nu_write_reg(dd, ctx->dma_buff + MADR6_OFF, RSA_MADDR6);

	nu_write_reg(dd, (keyleng << RSA_CTL_KEYLENG_OFFSET) |
			RSA_CTL_START, RSA_CTL);

	if (dd->nu_cdev->use_optee == false) {
		timeout = jiffies + msecs_to_jiffies(2000);
		while (nu_read_reg(dd, RSA_STS) & RSA_STS_BUSY) {
			if (time_after(jiffies, timeout)) {
				pr_err("RSA encrypt time-out!\n");
				err = -EIO;
				break;
			}
		}
	} else {
		/*
		 *  Invoke OP-TEE Crypto PTA to run RSA
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_RSA_RUN function of Trusted App */
		inv_arg.func = PTA_CMD_CRYPTO_RSA_RUN;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

		param[1].u.memref.shm = dd->shm_pool;
		param[1].u.memref.size = CRYPTO_SHM_SIZE;
		param[1].u.memref.shm_offs = 0;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_RSA_RUN enc err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}
	}

	dma_unmap_single(dd->dev, ctx->dma_buff,
				RSA_BUFF_SIZE, DMA_FROM_DEVICE);

	memset(m, 0, sizeof(m));
	nu_rsa_reg_to_hex((u32 *)&ctx->buffer[ANS_OFF],
				ctx->rsa_bit_len, m, &len);

	nuvoton_rsa_buffer_to_sg(m, min_t(int, len, (int)req->dst_len),
			req->dst);
	return err;
}

static int nuvoton_rsa_dec(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct nu_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct nu_rsa_dev *dd = nuvoton_rsa_find_dev(ctx);
	u8	m[NU_RSA_MAX_BYTE_LEN];
	u32	keyleng;
	int	err, len;
	unsigned long   timeout;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if ((dd->nu_cdev->use_optee) && (dd->octx == NULL)) {
		if (optee_rsa_open(dd) != 0)
			return -ENODEV;
	}
	ctx->dd = dd;
	keyleng = (ctx->rsa_bit_len - 1024) / 1024;

	memset(&ctx->buffer[D_OFF], 0, NU_RSA_MAX_BYTE_LEN);
	err = nu_rsa_hex_to_reg((u8 *)ctx->private_key,
				ctx->private_key_size, ctx->rsa_bit_len,
				(u32 *)&ctx->buffer[D_OFF]);
	if (err)
		return err;

	len = nuvoton_rsa_sg_to_buffer(req->src, m,
				min((int)req->src_len, NU_RSA_MAX_BYTE_LEN));

	pr_debug("[%s] - req_src byte length: %d\n", __func__, len);

	memset(&ctx->buffer[M_OFF], 0, NU_RSA_MAX_BYTE_LEN);
	memset(&ctx->buffer[ANS_OFF], 0, NU_RSA_MAX_BYTE_LEN);

	err = nu_rsa_hex_to_reg(m, len, ctx->rsa_bit_len,
				(u32 *)&ctx->buffer[M_OFF]);
	if (err)
		return err;

	ctx->dma_buff = dma_map_single(dd->dev, ctx->buffer,
				RSA_BUFF_SIZE, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dd->dev, ctx->dma_buff))) {
		dev_err(dd->dev, "RSA buffer dma map error\n");
		return -EINVAL;
	}

	nu_write_reg(dd, ctx->dma_buff + M_OFF,     RSA_SADDR0);
	nu_write_reg(dd, ctx->dma_buff + N_OFF,     RSA_SADDR1);
	nu_write_reg(dd, ctx->dma_buff + E_OFF,     RSA_SADDR2);
	nu_write_reg(dd, ctx->dma_buff + P_OFF,     RSA_SADDR3);
	nu_write_reg(dd, ctx->dma_buff + Q_OFF,     RSA_SADDR4);
	nu_write_reg(dd, ctx->dma_buff + ANS_OFF,   RSA_DADDR);
	nu_write_reg(dd, ctx->dma_buff + MADR0_OFF, RSA_MADDR0);
	nu_write_reg(dd, ctx->dma_buff + MADR1_OFF, RSA_MADDR1);
	nu_write_reg(dd, ctx->dma_buff + MADR2_OFF, RSA_MADDR2);
	nu_write_reg(dd, ctx->dma_buff + MADR3_OFF, RSA_MADDR3);
	nu_write_reg(dd, ctx->dma_buff + MADR4_OFF, RSA_MADDR4);
	nu_write_reg(dd, ctx->dma_buff + MADR5_OFF, RSA_MADDR5);
	nu_write_reg(dd, ctx->dma_buff + MADR6_OFF, RSA_MADDR6);

	nu_write_reg(dd, (keyleng << RSA_CTL_KEYLENG_OFFSET) |
			RSA_CTL_START, RSA_CTL);

	if (dd->nu_cdev->use_optee == false) {
		timeout = jiffies + msecs_to_jiffies(2000);
		while (nu_read_reg(dd, RSA_STS) & RSA_STS_BUSY) {
			if (time_after(jiffies, timeout)) {
				pr_err("RSA decrypt time-out!\n");
				err = -EIO;
				break;
			}
		}
	} else {
		/*
		 *  Invoke OP-TEE Crypto PTA to run RSA
		 */
		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		/* Invoke PTA_CMD_CRYPTO_RSA_RUN function of Trusted App */
		inv_arg.func = PTA_CMD_CRYPTO_RSA_RUN;
		inv_arg.session = dd->session_id;
		inv_arg.num_params = 4;

		/* Fill invoke cmd params */
		param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

		param[1].u.memref.shm = dd->shm_pool;
		param[1].u.memref.size = CRYPTO_SHM_SIZE;
		param[1].u.memref.shm_offs = 0;

		err = tee_client_invoke_func(dd->octx, &inv_arg, param);
		if ((err < 0) || (inv_arg.ret != 0)) {
			pr_err("PTA_CMD_CRYPTO_RSA_RUN dec err: %x\n",
				inv_arg.ret);
			return -EINVAL;
		}
	}

	dma_unmap_single(dd->dev, ctx->dma_buff, RSA_BUFF_SIZE,
				DMA_BIDIRECTIONAL);

	memset(m, 0, sizeof(m));
	nu_rsa_reg_to_hex((u32 *)&ctx->buffer[ANS_OFF],
				ctx->rsa_bit_len, m, &len);

	nuvoton_rsa_buffer_to_sg(m, min_t(int, len, (int)req->dst_len),
				req->dst);
	return err;
}

static int nuvoton_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
			   unsigned int keylen)
{
	struct nu_rsa_ctx  *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key raw_key = {0};
	int	rsa_bit_len;
	int	err;

	pr_debug("[%s]\n", __func__);

	err = rsa_parse_pub_key(&raw_key, key, keylen);
	if (err)
		return err;

	pr_debug("Key size n,e,d,p,q: %d %d %d %d %d\n", (int)raw_key.n_sz,
		(int)raw_key.e_sz, (int)raw_key.d_sz, (int)raw_key.p_sz,
		(int)raw_key.q_sz);

	rsa_bit_len = check_ras_length(&raw_key);
	if (rsa_bit_len < 0)
		return rsa_bit_len;

	ctx->rsa_bit_len = rsa_bit_len;

	memset(&ctx->buffer[N_OFF], 0, NU_RSA_MAX_BYTE_LEN);
	err = nu_rsa_hex_to_reg((u8 *)raw_key.n, raw_key.n_sz, rsa_bit_len,
					(u32 *)&ctx->buffer[N_OFF]);
	if (err)
		return err;

	memcpy(ctx->public_key, (u8 *)raw_key.e, raw_key.e_sz);
	ctx->public_key_size = raw_key.e_sz;

	return 0;
}

static int nuvoton_rsa_set_priv_key(struct crypto_akcipher *tfm,
				    const void *key, unsigned int keylen)
{
	struct nu_rsa_ctx  *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key raw_key = {0};
	int	rsa_bit_len;
	int	err;

	pr_debug("[%s]\n", __func__);

	err = rsa_parse_priv_key(&raw_key, key, keylen);
	if (err)
		return err;

	pr_debug("Key size n,e,d,p,q: %d %d %d %d %d\n", (int)raw_key.n_sz,
		(int)raw_key.e_sz, (int)raw_key.d_sz, (int)raw_key.p_sz,
		(int)raw_key.q_sz);

	rsa_bit_len = check_ras_length(&raw_key);
	if (rsa_bit_len < 0)
		return rsa_bit_len;

	ctx->rsa_bit_len = rsa_bit_len;

	memset(&ctx->buffer[N_OFF], 0, NU_RSA_MAX_BYTE_LEN);
	err = nu_rsa_hex_to_reg((u8 *)raw_key.n, raw_key.n_sz, rsa_bit_len,
					(u32 *)&ctx->buffer[N_OFF]);
	if (err)
		return err;

	memcpy(ctx->private_key, (u8 *)raw_key.d, raw_key.d_sz);
	ctx->private_key_size = raw_key.d_sz;

	memcpy(ctx->public_key, (u8 *)raw_key.e, raw_key.e_sz);
	ctx->public_key_size = raw_key.e_sz;

	return 0;
}

static unsigned int nuvoton_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct nu_rsa_ctx  *ctx = akcipher_tfm_ctx(tfm);

	pr_debug("%s: %d\n", __func__, ctx->rsa_bit_len / 8);
	return (ctx->rsa_bit_len / 8);
}

static int nuvoton_rsa_init_tfm(struct crypto_akcipher *tfm)
{
	struct nu_rsa_ctx  *ctx = akcipher_tfm_ctx(tfm);

	memset(ctx->buffer, 0, sizeof(ctx->buffer));
	return 0;
}

static struct akcipher_alg  nuvoton_rsa = {
	.encrypt = nuvoton_rsa_enc,
	.decrypt = nuvoton_rsa_dec,
	.set_priv_key = nuvoton_rsa_set_priv_key,
	.set_pub_key = nuvoton_rsa_set_pub_key,
	.max_size = nuvoton_rsa_max_size,
	.init = nuvoton_rsa_init_tfm,
	.base = {
		.cra_name = "rsa",
		.cra_driver_name = "rsa-ma35d1",
		.cra_priority = 500,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct nu_rsa_ctx),
	},
};

static int  optee_rsa_open(struct nu_rsa_dev *dd)
{
	struct tee_ioctl_open_session_arg sess_arg;
	int   err;

	err = nuvoton_crypto_optee_init(dd->nu_cdev);
	if (err)
		return err;
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
	 * Open RSA session with Crypto Trusted App
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

static void optee_rsa_close(struct nu_rsa_dev *dd)
{
	tee_shm_free(dd->shm_pool);
	tee_client_close_session(dd->octx, dd->session_id);
	tee_client_close_context(dd->octx);
	dd->octx = NULL;
}

static int nvt_rsa_ioctl_set_register(struct nu_rsa_ctx *rsa_ctx,
				      int offs, unsigned long arg)
{
	int   ret, sz;
	u8    kbuf[520];

	memset(kbuf, 0, sizeof(kbuf));
	if (copy_from_user(kbuf, (u8 *)arg, 514))
		return -EFAULT;

	sz = (kbuf[0] << 8) | kbuf[1];
	if (sz > 512)
		return -EINVAL;

	memset(&rsa_ctx->buffer[offs], 0, NU_RSA_MAX_BYTE_LEN);
	ret = nu_rsa_hex_to_reg((u8 *)&kbuf[2], sz,
				rsa_ctx->rsa_bit_len,
				(u32 *)&rsa_ctx->buffer[offs]);
	return ret;
}

static long nvt_rsa_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	struct nu_rsa_dev  *dd;
	int	keyleng, len;
	u8	m[NU_RSA_MAX_BYTE_LEN];
	struct nu_rsa_ctx  *rsa_ctx = filp->private_data;
	unsigned long   timeout;
	int err;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!rsa_ctx)
		return -EIO;
	dd = rsa_ctx->dd;

	switch (cmd) {
	case RSA_IOC_SET_BITLEN:
		rsa_ctx->rsa_bit_len = arg;
		return 0;

	case RSA_IOC_SET_N:
		memset(&rsa_ctx->buffer[N_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		nvt_rsa_ioctl_set_register(rsa_ctx, N_OFF, arg);
		break;

	case RSA_IOC_SET_E:
		memset(&rsa_ctx->buffer[E_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		nvt_rsa_ioctl_set_register(rsa_ctx, E_OFF, arg);
		break;

	case RSA_IOC_SET_M:
		memset(&rsa_ctx->buffer[M_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		nvt_rsa_ioctl_set_register(rsa_ctx, M_OFF, arg);
		break;

	case RSA_IOC_SET_P:
		memset(&rsa_ctx->buffer[P_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		nvt_rsa_ioctl_set_register(rsa_ctx, P_OFF, arg);
		break;

	case RSA_IOC_SET_Q:
		memset(&rsa_ctx->buffer[Q_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		nvt_rsa_ioctl_set_register(rsa_ctx, Q_OFF, arg);
		break;

	case RSA_IOC_RUN:
		memset(&rsa_ctx->buffer[ANS_OFF], 0, NU_RSA_MAX_BYTE_LEN);
		rsa_ctx->dma_buff = dma_map_single(dd->dev, rsa_ctx->buffer,
				RSA_BUFF_SIZE, DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(dd->dev, rsa_ctx->dma_buff))) {
			dev_err(dd->dev, "RSA buffer DMA map error\n");
			return -EINVAL;
		}
		nu_write_reg(dd, rsa_ctx->dma_buff + M_OFF,     RSA_SADDR0);
		nu_write_reg(dd, rsa_ctx->dma_buff + N_OFF,     RSA_SADDR1);
		nu_write_reg(dd, rsa_ctx->dma_buff + E_OFF,     RSA_SADDR2);
		nu_write_reg(dd, rsa_ctx->dma_buff + P_OFF,     RSA_SADDR3);
		nu_write_reg(dd, rsa_ctx->dma_buff + Q_OFF,     RSA_SADDR4);
		nu_write_reg(dd, rsa_ctx->dma_buff + ANS_OFF,   RSA_DADDR);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR0_OFF, RSA_MADDR0);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR1_OFF, RSA_MADDR1);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR2_OFF, RSA_MADDR2);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR3_OFF, RSA_MADDR3);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR4_OFF, RSA_MADDR4);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR5_OFF, RSA_MADDR5);
		nu_write_reg(dd, rsa_ctx->dma_buff + MADR6_OFF, RSA_MADDR6);

		keyleng = rsa_ctx->rsa_bit_len/1024 - 1;
		nu_write_reg(dd, (keyleng << RSA_CTL_KEYLENG_OFFSET) |
				RSA_CTL_START, RSA_CTL);

		if (dd->nu_cdev->use_optee == false) {
			timeout = jiffies + msecs_to_jiffies(2000);
			while (nu_read_reg(dd, RSA_STS) & RSA_STS_BUSY) {
				if (time_after(jiffies, timeout)) {
					pr_err("RSA decrypt time-out!\n");
					return -EIO;
					break;
				}
			}
		} else {
			/*
		 	*  Invoke OP-TEE Crypto PTA to run RSA
		 	*/
			memset(&inv_arg, 0, sizeof(inv_arg));
			memset(&param, 0, sizeof(param));

			/* Invoke PTA_CMD_CRYPTO_RSA_RUN function of Trusted App */
			inv_arg.func = PTA_CMD_CRYPTO_RSA_RUN;
			inv_arg.session = dd->session_id;
			inv_arg.num_params = 4;

			/* Fill invoke cmd params */
			param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INOUT;

			param[1].u.memref.shm = dd->shm_pool;
			param[1].u.memref.size = CRYPTO_SHM_SIZE;
			param[1].u.memref.shm_offs = 0;

			memcpy(&(dd->va_shm[0x1000/4]), rsa_ctx->buffer, RSA_BUFF_SIZE);
			memset(&(dd->va_shm[0x3000/4]), 0, 0x200);
			err = tee_client_invoke_func(dd->octx, &inv_arg, param);
			if ((err < 0) || (inv_arg.ret != 0)) {
				pr_err("PTA_CMD_CRYPTO_RSA_RUN err: %x\n",
					inv_arg.ret);
				dma_unmap_single(dd->dev, rsa_ctx->dma_buff,
						 RSA_BUFF_SIZE, DMA_BIDIRECTIONAL);
				return -EINVAL;
			}
			memcpy((u32 *)&rsa_ctx->buffer[ANS_OFF], &(dd->va_shm[0x3000/4]), 0x200);
		}

		dma_unmap_single(dd->dev, rsa_ctx->dma_buff,
				 RSA_BUFF_SIZE, DMA_BIDIRECTIONAL);

		nu_rsa_reg_to_hex((u32 *)&rsa_ctx->buffer[ANS_OFF],
				  rsa_ctx->rsa_bit_len, m, &len);

		if (copy_to_user((char *)arg, m, len))
			return -EFAULT;
		break;

	default:
		return -ENOTTY;
	}
	return 0;
}

static int nvt_rsa_open(struct inode *inode, struct file *file)
{
	struct nu_rsa_ctx  *rsa_ctx;

	rsa_ctx = kzalloc(sizeof(*rsa_ctx), GFP_KERNEL);
	if (!rsa_ctx)
		return -ENOMEM;
	rsa_ctx->dd = __rsa_dd;
	file->private_data = rsa_ctx;
	if ((__rsa_dd->nu_cdev->use_optee) && (__rsa_dd->octx == NULL)) {
		if (optee_rsa_open(__rsa_dd) != 0)
			return -ENODEV;
	}
	return 0;
}

static int nvt_rsa_close(struct inode *inode, struct file *file)
{
	struct nu_rsa_ctx  *rsa_ctx;

	rsa_ctx = file->private_data;
	if (!rsa_ctx)
		return -EIO;
	kfree(rsa_ctx);
	return 0;
}

const struct file_operations nvt_rsa_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= nvt_rsa_ioctl,
	.open           = nvt_rsa_open,
	.release        = nvt_rsa_close,
};

static struct miscdevice nvt_rsa_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nuvoton-rsa",
	.fops		= &nvt_rsa_fops,
};

int nuvoton_rsa_probe(struct device *dev,
		      struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_rsa_dev  *rsa_dd = &nu_cryp_dev->rsa_dd;
	int   err = 0;

	__rsa_dd = rsa_dd;
	rsa_dd->dev = dev;
	rsa_dd->nu_cdev = nu_cryp_dev;
	rsa_dd->reg_base = nu_cryp_dev->reg_base;
	rsa_dd->octx = NULL;

	spin_lock(&nu_rsa.lock);
	list_add_tail(&rsa_dd->list, &nu_rsa.dev_list);
	spin_unlock(&nu_rsa.lock);

	if (nu_cryp_dev->rsa_ioctl == true) {
		misc_register(&nvt_rsa_dev);
	} else {
		err = crypto_register_akcipher(&nuvoton_rsa);
		if (err) {
			pr_err("[%s] - failed to register nuvoton_rsa! %d\n",
				__func__, err);
			goto err_out;
		}
	}
	pr_info("MA35D1 Crypto RSA engine enabled.\n");
	return 0;

err_out:
	spin_lock(&nu_rsa.lock);
	list_del(&rsa_dd->list);
	spin_unlock(&nu_rsa.lock);
	return err;
}

int nuvoton_rsa_remove(struct device *dev,
		       struct nu_crypto_dev *nu_cryp_dev)
{
	struct nu_rsa_dev  *rsa_dd = &nu_cryp_dev->rsa_dd;

	if (rsa_dd == NULL)
		return -ENODEV;

	if (nu_cryp_dev->rsa_ioctl == true)
		misc_deregister(&nvt_rsa_dev);
	else
		crypto_unregister_akcipher(&nuvoton_rsa);

	spin_lock(&nu_rsa.lock);
	list_del(&rsa_dd->list);
	spin_unlock(&nu_rsa.lock);

	if (nu_cryp_dev->use_optee)
		optee_rsa_close(rsa_dd);
	return 0;
}


