# SPDX-License-Identifier: GPL-2.0-only

ifeq ($(CONFIG_CRYPTO_DEV_NUVOTON),y)
	obj-$(CONFIG_CRYPTO_DEV_NUVOTON) += nuvoton-crypto.o
	obj-$(CONFIG_CRYPTO_ECDH) += nuvoton-ecc.o
	obj-$(CONFIG_CRYPTO_RSA) += nuvoton-rsa.o
	nuvoton-crypto-objs:= nuvoton-crypto.o nuvoton-prng.o nuvoton-aes.o nuvoton-sha.o
endif

ifeq ($(CONFIG_CRYPTO_DEV_MA35D0),y)
	obj-$(CONFIG_CRYPTO_DEV_MA35D0) += ma35d0-crypto.o
	obj-$(CONFIG_CRYPTO_ECDH) += ma35d0-ecc.o
	obj-$(CONFIG_CRYPTO_RSA) += ma35d0-rsa.o
	ma35d0-crypto-objs:= ma35d0-crypto.o ma35d0-prng.o ma35d0-aes.o ma35d0-sha.o
endif

ifeq ($(CONFIG_CRYPTO_DEV_MA35H0),y)
	obj-$(CONFIG_CRYPTO_DEV_MA35H0) += ma35h0-crypto.o
	obj-$(CONFIG_CRYPTO_ECDH) += ma35h0-ecc.o
	obj-$(CONFIG_CRYPTO_RSA) += ma35h0-rsa.o
	ma35h0-crypto-objs:= ma35h0-crypto.o ma35h0-prng.o ma35h0-aes.o ma35h0-sha.o
endif
