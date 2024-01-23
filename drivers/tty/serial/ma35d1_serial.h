/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *
 *  MA35D1 serial driver header file
 *
 *  Copyright (C) 2017 Nuvoton Technology Corp.
 *
 */

#ifndef __MA35D1_SERIAL_H__
#define __MA35D1_SERIAL_H__

#define UART_REG_RBR	0x00
#define UART_REG_THR	0x00

#define UART_REG_IER	0x04
#define RDA_IEN		0x00000001
#define THRE_IEN	0x00000002
#define RLS_IEN		0x00000004
#define RTO_IEN		0x00000010
#define BUFERR_IEN	0x00000020
#define WAKEUP_IEN	0x00000040
#define TIME_OUT_EN	0x00000800
#define TXPDMAEN	0x00004000
#define RXPDMAEN	0x00008000

#define UART_REG_FCR	0x08
#define RFR		0x00000002
#define TFR		0x00000004

#define UART_REG_LCR	0x0C
#define	NSB		0x00000004
#define PBE		0x00000008
#define EPE		0x00000010
#define SPE		0x00000020
#define BCB		0x00000040

#define UART_REG_MCR	0x10
#define UART_REG_MSR	0x14

#define UART_REG_FSR	0x18
#define RX_OVER_IF	0x00000001
#define TX_OVER_IF	0x01000000
#define PEF		0x00000010
#define FEF		0x00000020
#define BIF		0x00000040
#define RX_EMPTY	0x00004000
#define TX_EMPTY	0x00400000
#define TX_FULL		0x00800000
#define RX_FULL		0x00008000
#define TE_FLAG		0x10000000

#define UART_REG_ISR	0x1C
#define RDA_IF		0x00000001
#define THRE_IF		0x00000002
#define TOUT_IF		0x00000010
#define WAKEUP_IF	0x00000040
#define THRE_INT	0x00000200
#define HWRLS_IF	0x00040000
#define PTO_IF		0x00100000
#define HWBUFE_IF	0x00200000

#define UART_REG_TOR	0x20
#define UART_REG_BAUD	0x24

#define UART_REG_IRCR	0x28

#define UART_REG_ALT_CSR	0x2C

#define UART_FUN_SEL	0x30
#define FUN_SEL_UART	0x00000000
#define FUN_SEL_LIN		0x00000001
#define FUN_SEL_IrDA	0x00000002
#define FUN_SEL_RS485	0x00000003
#define FUN_SEL_Msk		0x00000007

#define UART_REG_WKCTL	0x40
#define UART_REG_WKSTS	0x44

#define PDMA_UART0_TX   4
#define PDMA_UART0_RX   5
#define PDMA_UART1_TX   6
#define PDMA_UART1_RX   7
#define PDMA_UART2_TX   8
#define PDMA_UART2_RX   9
#define PDMA_UART3_TX  10
#define PDMA_UART3_RX  11
#define PDMA_UART4_TX  12
#define PDMA_UART4_RX  13
#define PDMA_UART5_TX  14
#define PDMA_UART5_RX  15
#define PDMA_UART6_TX  16
#define PDMA_UART6_RX  17
#define PDMA_UART7_TX  18
#define PDMA_UART7_RX  19
#define PDMA_UART8_TX  20
#define PDMA_UART8_RX  21
#define PDMA_UART9_TX  22
#define PDMA_UART9_RX  23
#define PDMA_UART10_TX 24
#define PDMA_UART10_RX 25
#define PDMA_UART11_TX 26
#define PDMA_UART11_RX 27
#define PDMA_UART12_TX 28
#define PDMA_UART12_RX 29
#define PDMA_UART13_TX 30
#define PDMA_UART13_RX 31
#define PDMA_UART14_TX 32
#define PDMA_UART14_RX 33
#define PDMA_UART15_TX 34
#define PDMA_UART15_RX 35
#define PDMA_UART16_TX 36
#define PDMA_UART16_RX 37

struct ma35d1_mem_alloc {
	u64		size;
	u64		vir_addr;
	dma_addr_t		phy_addr;
};

struct ma35d1_ip_rx_dma {
	struct dma_chan         *chan_rx;
	struct scatterlist      sgrx[64];
	struct dma_async_tx_descriptor  *rxdesc;
	struct dma_slave_config slave_config;
	dma_cookie_t cookie;
};

struct ma35d1_ip_tx_dma {
	struct dma_chan         *chan_tx;
	struct scatterlist      sgtx[64];
	struct dma_async_tx_descriptor  *txdesc;
	struct dma_slave_config slave_config;
	dma_cookie_t cookie;
};

#endif
