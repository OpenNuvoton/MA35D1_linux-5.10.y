// SPDX-License-Identifier: GPL-2.0+
/*
 *  MA35D1 serial driver
 *
 *  Copyright (C) 2018 Nuvoton Technology Corp.
 *
 */

#if defined(CONFIG_SERIAL_MA35D1_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/serial.h>
#include <linux/platform_data/dma-ma35d1.h>
#include "ma35d1_serial.h"

#define UART_NR 17
#define UART_RX_BUF_SIZE 4096
#define UART_TX_MAX_BUF_SIZE 128

/* PDMA mode time-out */
#define Time_Out_Frame_Count 2
#define Time_Out_Low_Baudrate 115200

#define UPDMA_OFFSET_CHAN_SIZE		0x10
#define UPDMA_DSCT_CTL			0x0
#define   UPDMA_OP_STOP			0x0
#define   UPDMA_OP_BASIC		0x1
#define   UPDMA_OP_SCATTER		0x2
#define   UPDMA_OP_MSK			0x3
#define   UPDMA_TXTYPE			(1<<2)
#define   UPDMA_TBINTDIS		(1<<7)
#define   UPDMA_SAFIX			(3<<8)
#define   UPDMA_DAFIX			(3<<10)
#define   UPDMA_TXWIDTH_1_BYTE		(0<<12)
#define   UPDMA_TXWIDTH_2_BYTES	(1<<12)
#define   UPDMA_TXWIDTH_4_BYTES	(2<<12)
#define   UPDMA_TXCNT(cnt)		((cnt)<<16)
#define   UPDMA_GET_TXCNT(ctrl)	((ctrl>>16)&0xffff)
#define UPDMA_DSCT_SA			0x004
#define UPDMA_DSCT_DA			0x008
#define UPDMA_DSCT_NEXT			0x00c
#define UPDMA_CHCTL			0x400
#define UPDMA_PAUSE			0x404
#define UPDMA_SWREQ			0x408
#define UPDMA_INTEN			0x418
#define UPDMA_INTSTS			0x41C
#define UPDMA_TDSTS			0x424
#define UPDMA_TACTSTS			0x42C
#define UPDMA_TOUTEN			0x434
#define UPDMA_TOUTIEN			0x438
#define UPDMA_TOC			0x440
#define UPDMA_CHRST			0x460
#define UPDMA_TOUTPSC			0x470
#define UPDMA_TOUTPSC1			0x474
#define UPDMA_REQSEL0_3			0x480
#define UPDMA_REQSEL4_7			0x484
#define UPDMA_REQSEL8_11		0x488


unsigned char PUART_PDMA_TX_ID[UART_NR] = {
	PDMA_UART0_TX, PDMA_UART1_TX, PDMA_UART2_TX,
	PDMA_UART3_TX, PDMA_UART4_TX, PDMA_UART5_TX,
	PDMA_UART6_TX, PDMA_UART7_TX, PDMA_UART8_TX,
	PDMA_UART9_TX, PDMA_UART10_TX, PDMA_UART11_TX,
	PDMA_UART12_TX, PDMA_UART13_TX, PDMA_UART14_TX,
	PDMA_UART15_TX, PDMA_UART16_TX
};

unsigned char PUART_PDMA_RX_ID[UART_NR] = {
	PDMA_UART0_RX, PDMA_UART1_RX, PDMA_UART2_RX,
	PDMA_UART3_RX, PDMA_UART4_RX, PDMA_UART5_RX,
	PDMA_UART6_RX, PDMA_UART7_RX, PDMA_UART8_RX,
	PDMA_UART9_RX, PDMA_UART10_RX, PDMA_UART11_RX,
	PDMA_UART12_RX, PDMA_UART13_RX, PDMA_UART14_RX,
	PDMA_UART15_RX, PDMA_UART16_RX
};

static struct uart_driver ma35d1_puart_reg;

struct puart_ma35d1_port {
	struct uart_port    port;

	unsigned short      capabilities; /* port capabilities */
	unsigned char       ier;
	unsigned char       lcr;
	unsigned char       mcr;
	unsigned char       mcr_mask; /* mask of user bits */
	unsigned char       mcr_force; /* mask of forced bits */

	struct ma35d1_ip_rx_dma dma_rx;
	struct ma35d1_ip_tx_dma dma_tx;
	struct ma35d1_mem_alloc src_mem_p;
	struct ma35d1_mem_alloc dest_mem_p;
	struct ma35d1_dma_done   dma_slave_done;

	unsigned char PDMA_UARTx_TX;
	unsigned char PDMA_UARTx_RX;

	struct ma35d1_dma_done   dma_Rx_done;
	struct ma35d1_dma_done   dma_Tx_done;

	unsigned int tx_dma_len;

	unsigned char uart_pdma_enable_flag;
	unsigned char Tx_pdma_busy_flag;

	unsigned int pdma_time_out_prescaler;
	unsigned int pdma_time_out_count;
	unsigned int baud_rate;

	unsigned char pdma_port;
	unsigned char pdma_tx_ch;
	unsigned char pdma_rx_ch;

	resource_size_t pdma_iobase;
	void __iomem *pdma_membase;

	u32 pdma_rx_mem_size;
	u32 pdma_tx_mem_size;
	u64 pdma_rx_vir_addr;
	u64 pdma_rx_phy_addr;
	u64 pdma_tx_vir_addr;
	u64 pdma_tx_phy_addr;

	u64 pdma_rx_vir_addr1;
	u64 pdma_rx_vir_addr2;
	u64 pdma_rx_phy_addr1;
	u64 pdma_rx_phy_addr2;

	int pdma_irq;

	struct timer_list	rx_pdma_timer;
};

static struct puart_ma35d1_port puart_ma35d1_ports[UART_NR] = {0};


static inline void __stop_tx(struct puart_ma35d1_port *p);

static void ma35d1_PUART_prepare_RX_dma(struct puart_ma35d1_port *p);
static void ma35d1_PUART_prepare_TX_dma(struct puart_ma35d1_port *p);

static inline struct puart_ma35d1_port *
to_ma35d1_puart_port(struct uart_port *uart)
{
	return container_of(uart, struct puart_ma35d1_port, port);
}

static inline unsigned int pserial_in(struct puart_ma35d1_port *p,
									int offset)
{
	return __raw_readl(p->port.membase + offset);
}

static inline void pserial_out(struct puart_ma35d1_port *p, int offset,
								int value)
{
	__raw_writel(value, p->port.membase + offset);
}

static inline unsigned int pdma_read(struct puart_ma35d1_port *p,
									int offset)
{
	return __raw_readl(p->pdma_membase + offset);
}

static inline void pdma_write(struct puart_ma35d1_port *p, int offset,
								int value)
{
	__raw_writel(value, p->pdma_membase + offset);
}

static inline unsigned int pdma_ch_read(struct puart_ma35d1_port *p, unsigned char ch,
									int offset)
{
	return __raw_readl(p->pdma_membase + (0x10 * ch) + offset);
}

static inline void pdma_ch_write(struct puart_ma35d1_port *p, unsigned char ch, 
								int offset,	int value)
{
	__raw_writel(value, p->pdma_membase + (0x10 * ch) + offset);
}

void PDMA_SetTransferAddr(struct puart_ma35d1_port *p, unsigned char u32Ch, 
							uint32_t u32SrcAddr, uint32_t u32DstAddr)
{
	uint32_t u32Ch_Offset = (0x10 * u32Ch);

	pdma_write(p, UPDMA_DSCT_SA+u32Ch_Offset, u32SrcAddr);
	pdma_write(p, UPDMA_DSCT_DA+u32Ch_Offset, u32DstAddr);
}

void PDMA_SetTransferMode(struct puart_ma35d1_port *p,unsigned char u32Ch)
{
#if 1
	uint32_t u32Request = 0;
	uint32_t u32Port = p->port.line;
	uint32_t u32REQ_Reg_Tmp = 0;

	if(u32Ch == p->pdma_rx_ch)
		u32Request = 5 + (u32Port * 2);
	else if(u32Ch == p->pdma_tx_ch)
		u32Request = 4 + (u32Port * 2);

    switch(u32Ch) {
    case 0ul:
		u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL0_3);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff);
		pdma_write(p, UPDMA_REQSEL0_3, u32REQ_Reg_Tmp|(u32Request));
        break;
    case 1ul:
		u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL0_3);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 8);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp | (u32Request << 8);
		pdma_write(p, UPDMA_REQSEL0_3, u32REQ_Reg_Tmp);
        break;
    case 2ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL0_3);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 16);
		pdma_write(p, UPDMA_REQSEL0_3, u32REQ_Reg_Tmp|(u32Request << 16));;
        break;
    case 3ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL0_3);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 24);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp | (u32Request << 24);
		pdma_write(p, UPDMA_REQSEL0_3, u32REQ_Reg_Tmp);
        break;
    case 4ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL4_7);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff);
		pdma_write(p, UPDMA_REQSEL4_7, u32REQ_Reg_Tmp|(u32Request));
        break;
    case 5ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL4_7);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 8);
		pdma_write(p, UPDMA_REQSEL4_7, u32REQ_Reg_Tmp|(u32Request << 8));
        break;
    case 6ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL4_7);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 16);
		pdma_write(p, UPDMA_REQSEL4_7, u32REQ_Reg_Tmp|(u32Request << 16));
        break;
    case 7ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL4_7);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 24);
		pdma_write(p, UPDMA_REQSEL4_7, u32REQ_Reg_Tmp|(u32Request << 24));
        break;
    case 8ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL8_11);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff);
		pdma_write(p, UPDMA_REQSEL8_11, u32REQ_Reg_Tmp|(u32Request));
        break;
    case 9ul:
        u32REQ_Reg_Tmp = pdma_read(p, UPDMA_REQSEL8_11);
		u32REQ_Reg_Tmp = u32REQ_Reg_Tmp & ~(0xff << 8);
		pdma_write(p, UPDMA_REQSEL8_11, u32REQ_Reg_Tmp|(u32Request << 8));
        break;
    default:
		printk("\n Request incorrect channel !! \n");
        break;
    }

#endif
}

static void ma35d1_PUART_Rx_dma_callback(struct puart_ma35d1_port *p)
{
	struct tty_port    *tty_port = &p->port.state->port;
	u32 u32PDMA_STATUS = pdma_read(p, UPDMA_INTSTS);
	unsigned char rx_tmp_buf[65];
	uint32_t i = 0;
	int copied_count = 0;
	int count;

	/* Disable UART RX PDMA */
	pserial_out(p, UART_REG_IER, (pserial_in(p, UART_REG_IER) & ~RXPDMAEN));

	if(pdma_read(p, UPDMA_TDSTS) & (0x1 << p->pdma_rx_ch)) { 
		count = UART_RX_BUF_SIZE;
	} else if( u32PDMA_STATUS & (0x1 << (p->pdma_rx_ch + 8)) ){

		/* Stop PDMA timeout counter */
		pdma_write(p, UPDMA_TOUTEN, (pdma_read(p, UPDMA_TOUTEN) &~ (0x1 << p->pdma_rx_ch)) );

		/* clear time-out flag */
		pdma_write(p, UPDMA_INTSTS, (0x1 << (p->pdma_rx_ch + 8)));

		/* Disable UART RX PDMA */
		//pserial_out(p, UART_REG_IER, (pserial_in(p, UART_REG_IER) & ~RXPDMAEN));

		/* PDMA channel reset */
		pdma_write(p, UPDMA_CHRST, (0x1 << p->pdma_rx_ch));

		/* wait PDMA channel active finish */
		while( pdma_read(p, UPDMA_TACTSTS) & (0x1 << p->pdma_rx_ch));

		/* Wait PDMA channel rest finish */
		while(pdma_read(p, UPDMA_CHRST) & (0x1 << p->pdma_rx_ch));

		count = (pdma_ch_read(p, p->pdma_rx_ch, UPDMA_DSCT_CTL) & 0xffff0000) >> 16;
		count = UART_RX_BUF_SIZE - count - 1;
	}

	if (p->pdma_rx_phy_addr2 == p->pdma_rx_phy_addr) {
		p->pdma_rx_phy_addr = p->pdma_rx_phy_addr1;
		p->pdma_rx_vir_addr = p->pdma_rx_vir_addr1;

		i = 0;
		pserial_out(p, UART_REG_IER,
				(pserial_in(p, UART_REG_IER) & ~RXPDMAEN));
		while (!(pserial_in(p, UART_REG_FSR) & RX_EMPTY)) {
			rx_tmp_buf[i] = (unsigned char)pserial_in(p, UART_REG_RBR);
			i++;
			if (i > 64)
				dev_err(p->port.dev, "rx_tmp_buf bull\n");
		}

		ma35d1_PUART_prepare_RX_dma(p);
		/* Trigger Rx dma again */
		pserial_out(p, UART_REG_IER,
				(pserial_in(p, UART_REG_IER)|RXPDMAEN|TIME_OUT_EN));

		dma_sync_single_for_cpu(p->port.dev,
				p->pdma_rx_phy_addr2, UART_RX_BUF_SIZE, DMA_FROM_DEVICE);

		if (i > 0)
			memcpy((void *)(p->pdma_rx_vir_addr2 + count), rx_tmp_buf, i);

		copied_count = tty_insert_flip_string(tty_port,
					((unsigned char *)p->pdma_rx_vir_addr2), count + i);
	} else {
		p->pdma_rx_phy_addr = p->pdma_rx_phy_addr2;
		p->pdma_rx_vir_addr = p->pdma_rx_vir_addr2;

		i = 0;
		pserial_out(p, UART_REG_IER,
				(pserial_in(p, UART_REG_IER) & ~RXPDMAEN));
		while (!(pserial_in(p, UART_REG_FSR) & RX_EMPTY)) {
			rx_tmp_buf[i] = (unsigned char)pserial_in(p, UART_REG_RBR);
			i++;
			if (i > 64)
				dev_err(p->port.dev, "rx_tmp_buf bull\n");
		}

		ma35d1_PUART_prepare_RX_dma(p);
		/* Trigger Rx dma again */
		pserial_out(p, UART_REG_IER,
				(pserial_in(p, UART_REG_IER)|RXPDMAEN|TIME_OUT_EN));

		dma_sync_single_for_cpu(p->port.dev,
			p->pdma_rx_phy_addr1, UART_RX_BUF_SIZE, DMA_FROM_DEVICE);

		if (i > 0)
			memcpy((void *)(p->pdma_rx_vir_addr1 + count), rx_tmp_buf, i);

		copied_count = tty_insert_flip_string(tty_port,
					((unsigned char *)p->pdma_rx_vir_addr1), count + i);
	}

	if (copied_count != (count + i)) {
		dev_err(p->port.dev, "Rx overrun: dropping %d bytes\n",
				(count - copied_count));
	}

	p->port.icount.rx = p->port.icount.rx + copied_count;

	tty_flip_buffer_push(tty_port);
}

static void ma35d1_PUART_Tx_dma_callback(struct puart_ma35d1_port *p)
{
	struct circ_buf *xmit = &p->port.state->xmit;

	spin_lock(&p->port.lock);

	p->port.icount.tx += p->tx_dma_len;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&p->port);

	if (!uart_circ_empty(xmit) && !uart_tx_stopped(&p->port)) {
		p->Tx_pdma_busy_flag = 1;
		ma35d1_PUART_prepare_TX_dma(p);
		/* Trigger Tx dma again */
		pserial_out(p, UART_REG_IER,
				(pserial_in(p, UART_REG_IER) | TXPDMAEN));
	} else
		p->Tx_pdma_busy_flag = 0;

	spin_unlock(&p->port.lock);
}


void ma35d1_puart_cal_pdma_time_out(struct puart_ma35d1_port *p, unsigned int baud)
{
	unsigned int lcr;
	unsigned int pdma_time_out_base =
		300000000 * Time_Out_Frame_Count / 256;
	unsigned int time_out_prescaler = 0;
	unsigned int bit_length;
	unsigned int time_out;

	if (baud > Time_Out_Low_Baudrate) {
		p->pdma_time_out_count = 255 * 16;
		p->pdma_time_out_prescaler = 7 * 16;

		return;
	}

	bit_length = 2; /* 1 start + 1 stop bit */

	lcr = pserial_in(p, UART_REG_LCR);
	switch (lcr & 0x3) {
	case 0:
		bit_length = bit_length + 5;
	break;
	case 1:
		bit_length = bit_length + 6;
	break;
	case 2:
		bit_length = bit_length + 7;
	break;
	case 3:
		bit_length = bit_length + 8;
	break;
	}

	if (lcr & 0x4)
		bit_length += 1;

	if (lcr & 0x8) /* Parity bit */
		bit_length += 1;

	time_out = pdma_time_out_base * bit_length;
	time_out = (time_out / baud) + 1;
	time_out = time_out * 16;

	/* pdma max. time-out count is 65535 */
	while (time_out > 65535) {
		time_out = time_out / 2;
		time_out_prescaler++;
	}

	if (time_out == 0)
		time_out = 1;

	p->pdma_time_out_count = time_out;
	p->pdma_time_out_prescaler = time_out_prescaler;
}

static void ma35d1_PUART_prepare_RX_dma(struct puart_ma35d1_port *p)
{
	int ret;
	u32 u32DSCT_Value = 0;
	u32 u32Reg_Tmp = 0;

	pserial_out(p, UART_REG_IER, (pserial_in(p, UART_REG_IER) & ~RXPDMAEN));

	if(p->pdma_rx_mem_size == 0) {
		p->pdma_rx_mem_size = UART_RX_BUF_SIZE;
		p->pdma_rx_vir_addr = (u64)(kmalloc(((UART_RX_BUF_SIZE + 64) * 2), GFP_KERNEL));
		p->pdma_rx_vir_addr1 = p->pdma_rx_vir_addr;

		p->pdma_rx_phy_addr =
			dma_map_single(p->port.dev,
				(void *)p->pdma_rx_vir_addr,
				((UART_RX_BUF_SIZE + 64) * 2), DMA_FROM_DEVICE);

		ret = dma_mapping_error(p->port.dev, p->pdma_rx_phy_addr);
		if (ret)
			dev_err(p->port.dev, "Rx buffer mapping error.\n");

		p->pdma_rx_phy_addr1 = p->pdma_rx_phy_addr;

		p->pdma_rx_vir_addr2 =
			p->pdma_rx_vir_addr1 + UART_RX_BUF_SIZE + 64;
		p->pdma_rx_phy_addr2 =
			p->pdma_rx_phy_addr1 + UART_RX_BUF_SIZE + 64;
	}

	PDMA_SetTransferAddr(p, p->pdma_rx_ch, p->port.iobase, p->pdma_rx_phy_addr);
	PDMA_SetTransferMode(p, p->pdma_rx_ch);
	
	u32DSCT_Value = ((UART_RX_BUF_SIZE - 1) << 16) | 0x305;
	pdma_ch_write(p, p->pdma_rx_ch, UPDMA_DSCT_CTL, u32DSCT_Value);

	u32Reg_Tmp = pdma_read(p, UPDMA_TOC + (4 * (p->pdma_rx_ch/2)));
	u32Reg_Tmp &= ~(0xffff << (16 * (p->pdma_rx_ch%2)) );
	u32Reg_Tmp = u32Reg_Tmp | (p->pdma_time_out_count << (16 * (p->pdma_rx_ch%2)));
	pdma_write(p, UPDMA_TOC + (4 * (p->pdma_rx_ch/2)), u32Reg_Tmp);

	if(p->pdma_rx_ch < 8) {
		u32Reg_Tmp = pdma_read(p, UPDMA_TOUTPSC);
		u32Reg_Tmp &= ~(0x7 << (4*p->pdma_rx_ch));
		u32Reg_Tmp = u32Reg_Tmp | (p->pdma_time_out_prescaler << (4*p->pdma_rx_ch));	
	
		pdma_write(p, UPDMA_TOUTPSC , u32Reg_Tmp);
	}
	else {
		u32Reg_Tmp = pdma_read(p, UPDMA_TOUTPSC1);
		u32Reg_Tmp &= ~(0x7 << (4*p->pdma_rx_ch));
		u32Reg_Tmp = u32Reg_Tmp | (p->pdma_time_out_prescaler << (4*p->pdma_rx_ch));
		pdma_write(p, UPDMA_TOUTPSC1 , u32Reg_Tmp);
	}

	/* Disable PDMA timeout interrupt*/
	pdma_write(p, UPDMA_TOUTIEN, (pdma_read(p, UPDMA_TOUTIEN) &~ (0x1 << p->pdma_rx_ch)) );

    /* Enable PDMA channel */
	//pdma_write(p, UPDMA_CHCTL, pdma_read(p, UPDMA_CHCTL) | (0x1 << p->pdma_rx_ch));
	pdma_write(p, UPDMA_CHCTL, 0x3ff);

	/* Start PDMA timeout counter */
	pdma_write(p, UPDMA_TOUTEN, (pdma_read(p, UPDMA_TOUTEN) | (0x1 << p->pdma_rx_ch)) );

	/* Enable PDMA rx channel interrupt */
	pdma_write(p, UPDMA_INTEN, pdma_read(p, UPDMA_INTEN) | (0x1 << p->pdma_rx_ch));
}

static void ma35d1_PUART_prepare_TX_dma(struct puart_ma35d1_port *p)
{
	struct circ_buf *xmit = &p->port.state->xmit;
	int ret;
	u32 u32DSCT_Value = 0;

	if(p->pdma_tx_mem_size == 0) {
		p->pdma_tx_mem_size = UART_XMIT_SIZE;
		p->pdma_tx_vir_addr = (u64)(kmalloc(p->pdma_tx_mem_size, GFP_KERNEL));

		p->pdma_tx_phy_addr =
			dma_map_single(p->port.dev,
				(void *)p->pdma_tx_vir_addr,
				p->pdma_tx_mem_size, DMA_TO_DEVICE);
		ret = dma_mapping_error(p->port.dev, p->pdma_tx_phy_addr);
		if (ret)
			dev_err(p->port.dev, "Tx buffer mapping error.\n");
	}

	p->tx_dma_len = uart_circ_chars_pending(xmit);

	if(p->tx_dma_len >= UART_XMIT_SIZE) p->tx_dma_len = UART_XMIT_SIZE;

	if (xmit->tail < xmit->head)
		memcpy((unsigned char *)p->pdma_tx_vir_addr,
				&xmit->buf[xmit->tail], p->tx_dma_len);
	else {
		size_t first = UART_XMIT_SIZE - xmit->tail;
		size_t second = xmit->head;

		memcpy((unsigned char *)p->pdma_tx_vir_addr,
				&xmit->buf[xmit->tail], first);
		if (second)
			memcpy((unsigned char *)p->pdma_tx_vir_addr+first,
				&xmit->buf[0], second);
	}

	pserial_out(p, UART_REG_IER, (pserial_in(p, UART_REG_IER) & ~TXPDMAEN));

	dma_sync_single_for_device(p->port.dev,	p->pdma_tx_phy_addr, UART_XMIT_SIZE, DMA_TO_DEVICE);

	xmit->tail = (xmit->tail +  p->tx_dma_len) & (UART_XMIT_SIZE - 1);

	PDMA_SetTransferAddr(p, p->pdma_tx_ch, p->pdma_tx_phy_addr, p->port.iobase);
	PDMA_SetTransferMode(p, p->pdma_tx_ch);

	u32DSCT_Value = ((p->tx_dma_len - 1) << 16) | 0xC05;
	pdma_ch_write(p, p->pdma_tx_ch, UPDMA_DSCT_CTL, u32DSCT_Value);

	/* Enable PDMA channel */
	//pdma_write(p, UPDMA_CHCTL, pdma_read(p, UPDMA_CHCTL)|(0x1 << p->pdma_tx_ch));

	/* Enable PDMA tx channel interrupt */
	pdma_write(p, UPDMA_INTEN, pdma_read(p, UPDMA_INTEN) | (0x1 << p->pdma_tx_ch));
}


static inline void __stop_tx(struct puart_ma35d1_port *p)
{
	unsigned int ier;

	ier = pserial_in(p, UART_REG_IER);
	if (ier & THRE_IEN)
		pserial_out(p, UART_REG_IER, ier & ~THRE_IEN);
}

static void ma35d1_PUART_stop_tx(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;

	__stop_tx(up);
}

static void puart_transmit_chars(struct puart_ma35d1_port *up);

static void ma35d1_PUART_start_tx(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned int ier;
	struct circ_buf *xmit = &up->port.state->xmit;

	if (up->uart_pdma_enable_flag == 1) {
		if (up->Tx_pdma_busy_flag == 1)
			return;

		if (uart_circ_empty(xmit)) {
			__stop_tx(up);
			return;
		}

		up->Tx_pdma_busy_flag = 1;
		ma35d1_PUART_prepare_TX_dma(up);
		pserial_out(up, UART_REG_IER,
				(pserial_in(up, UART_REG_IER)|TXPDMAEN));
	} else {
		struct circ_buf *xmit = &up->port.state->xmit;

		ier = pserial_in(up, UART_REG_IER);
		pserial_out(up, UART_REG_IER, ier & ~THRE_IEN);
		if (uart_circ_chars_pending(xmit) <
				(16-((pserial_in(up, UART_REG_FSR)>>16)&0x3F)))
			puart_transmit_chars(up);
		pserial_out(up, UART_REG_IER, ier | THRE_IEN);
	}
}

static void ma35d1_PUART_stop_rx(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;

	pserial_out(up, UART_REG_IER, pserial_in(up, UART_REG_IER) & ~RDA_IEN);
}


static int max_count;

static void
puart_receive_chars(struct puart_ma35d1_port *up)
{
	unsigned char ch;
	unsigned int fsr;
	unsigned int isr;
	unsigned int dcnt;
	char flag;

	isr = pserial_in(up, UART_REG_ISR);
	fsr = pserial_in(up, UART_REG_FSR);

	while (!(fsr & RX_EMPTY)) {
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(fsr & (BIF | FEF | PEF | RX_OVER_IF))) {
			if (fsr & BIF) {
				pserial_out(up, UART_REG_FSR, BIF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (fsr & FEF) {
				pserial_out(up, UART_REG_FSR, FEF);
				up->port.icount.frame++;
			}

			if (fsr & PEF) {
				pserial_out(up, UART_REG_FSR, PEF);
				up->port.icount.parity++;
			}

			if (fsr & RX_OVER_IF) {
				pserial_out(up, UART_REG_FSR, RX_OVER_IF);
				up->port.icount.overrun++;
			}

			if (fsr & BIF)
				flag = TTY_BREAK;
			if (fsr & PEF)
				flag = TTY_PARITY;
			if (fsr & FEF)
				flag = TTY_FRAME;
		}

		ch = (unsigned char)pserial_in(up, UART_REG_RBR);

		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, fsr, RX_OVER_IF, ch, flag);
		max_count++;
		dcnt = (pserial_in(up, UART_REG_FSR) >> 8) & 0x3f;
		if (max_count > 1023) {
			spin_lock(&up->port.lock);
			tty_flip_buffer_push(&up->port.state->port);
			spin_unlock(&up->port.lock);
			max_count = 0;
			if ((isr & TOUT_IF) && (dcnt == 0))
				goto tout_end;
		}

		if (isr & RDA_IF) {
			if (dcnt == 1)
				return; /* have remaining data, don't reset max_count */
		}
		fsr = pserial_in(up, UART_REG_FSR);
	}

	spin_lock(&up->port.lock);
	tty_flip_buffer_push(&up->port.state->port);
	spin_unlock(&up->port.lock);
tout_end:
	max_count = 0;
}

static void puart_transmit_chars(struct puart_ma35d1_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count = 16 - ((pserial_in(up, UART_REG_FSR)>>16)&0xF);

	if (pserial_in(up, UART_REG_FSR) & TX_FULL)
		count = 0;

	if (up->port.x_char) {
		do {
		} while (pserial_in(up, UART_REG_FSR) & TX_FULL);
		pserial_out(up, UART_REG_THR, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}

	if (uart_tx_stopped(&up->port)) {
		ma35d1_PUART_stop_tx(&up->port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	while (count > 0) {
		do {
		} while (pserial_in(up, UART_REG_FSR) & TX_FULL);
		pserial_out(up, UART_REG_THR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		count--;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);

}

static unsigned int check_modem_status(struct puart_ma35d1_port *up)
{
	unsigned int status = 0;

	if (0)
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);

	return status;
}

static irqreturn_t ma35d1_PUART_interrupt(int irq, void *dev_id)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)dev_id;
	unsigned int isr, fsr;
	unsigned char ch[64];
	uint32_t i = 0;
	struct tty_port    *tty_port = &up->port.state->port;
	int read_fifo_count = 0;

	isr = pserial_in(up, UART_REG_ISR);
	fsr = pserial_in(up, UART_REG_FSR);

	if (up->uart_pdma_enable_flag == 1) {

		if (isr & PTO_IF) {
			pserial_out(up, UART_REG_IER,
					(pserial_in(up, UART_REG_IER) & ~RXPDMAEN));
			i = 0;
			while (!(pserial_in(up, UART_REG_FSR) & RX_EMPTY)) {
				ch[i] = (unsigned char)pserial_in(up, UART_REG_RBR);
				i++;
			}

			pserial_out(up, UART_REG_IER,
					(pserial_in(up, UART_REG_IER) | RXPDMAEN));

			if (i > 0) {
				read_fifo_count = tty_insert_flip_string(tty_port, ch, i);

				if (read_fifo_count != i)
					dev_err(up->port.dev, "Rx dropping %d bytes\n",
							(i - read_fifo_count));

				up->port.icount.rx += read_fifo_count;
				tty_flip_buffer_push(tty_port);
			}
		}

		if (fsr & (BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF))
			pserial_out(up, UART_REG_FSR,
					(BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF));

	} else {
		if (isr & (RDA_IF | TOUT_IF))
			puart_receive_chars(up);

		check_modem_status(up);

		if (isr & THRE_INT)
			puart_transmit_chars(up);

		if (fsr & (BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF))
			pserial_out(up, UART_REG_FSR,
					(BIF | FEF | PEF | RX_OVER_IF | TX_OVER_IF));
	}

	return IRQ_HANDLED;
}

static irqreturn_t ma35d1_PUART_PDMA_Int(int irq, void *dev_id)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)dev_id;
	uint32_t PDMA_TDSTS = 0;
	irqreturn_t ret = IRQ_NONE;

	PDMA_TDSTS = pdma_read(up, UPDMA_TDSTS);

	if(PDMA_TDSTS & (0x1 << up->pdma_rx_ch)) {

		/* Disable PDMA rx channel interrupt */
		pdma_write(up, UPDMA_INTEN, (pdma_read(up, UPDMA_INTEN) &~ (0x1 << up->pdma_rx_ch)) );

		/* Clear Interrupt Flag */
		//pdma_write(up, UPDMA_TDSTS, (0x1 << up->pdma_rx_ch));

		ma35d1_PUART_Rx_dma_callback(up);

		/* Clear Interrupt Flag */
		pdma_write(up, UPDMA_TDSTS, (0x1 << up->pdma_rx_ch));

		ret = IRQ_HANDLED;
	}

	if(PDMA_TDSTS & (0x1 << up->pdma_tx_ch)) {

		/* Disable PDMA tx channel interrupt */
		pdma_write(up, UPDMA_INTEN, (pdma_read(up, UPDMA_INTEN) &~ (0x1 << up->pdma_tx_ch)) );

		/* Clear Interrupt Flag */
		pdma_write(up, UPDMA_TDSTS, (0x1 << up->pdma_tx_ch));

		ma35d1_PUART_Tx_dma_callback(up);

		ret = IRQ_HANDLED;
	}

	return ret;
}

static unsigned int ma35d1_PUART_tx_empty(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned int fsr;

	fsr = pserial_in(up, UART_REG_FSR);

	return (fsr & (TE_FLAG | TX_EMPTY)) ==
				(TE_FLAG | TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int ma35d1_PUART_get_mctrl(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned int status;
	unsigned int ret = 0;

	status = pserial_in(up, UART_REG_MSR);

	if (!(status & 0x10))
		ret |= TIOCM_CTS;

	return ret;
}

static void ma35d1_PUART_set_mctrl(struct uart_port *port,
									unsigned int mctrl)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned int mcr = 0;
	unsigned int ier = 0;

	if (mctrl & TIOCM_RTS) {
		/* set RTS high level trigger */
		mcr = pserial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);
	}

	if (up->mcr & UART_MCR_AFE) {
		/* set RTS high level trigger */
		mcr = pserial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);

		/* enable CTS/RTS auto-flow control */
		pserial_out(up, UART_REG_IER,
				(pserial_in(up, UART_REG_IER) | (0x3000)));

		/* Set hardware flow control */
		up->port.flags |= UPF_HARD_FLOW;
	} else {
		/* disable CTS/RTS auto-flow control */
		ier = pserial_in(up, UART_REG_IER);
		ier &= ~(0x3000);
		pserial_out(up, UART_REG_IER, ier);

		/* un-set hardware flow control */
		up->port.flags &= ~UPF_HARD_FLOW;
	}

	/* set CTS high level trigger */
	pserial_out(up, UART_REG_MSR,
			(pserial_in(up, UART_REG_MSR) | (0x100)));
}

static void ma35d1_PUART_break_ctl(struct uart_port *port,
									int break_state)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned long flags;
	unsigned int lcr;

	spin_lock_irqsave(&up->port.lock, flags);
	lcr = pserial_in(up, UART_REG_LCR);
	if (break_state != 0)
		lcr |= BCB; /* set break */
	else
		lcr &= ~BCB; /* clr break */
	pserial_out(up, UART_REG_LCR, lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void ma35d1_PUART_timer(struct timer_list *t)
{
	struct puart_ma35d1_port *up = from_timer(up, t, rx_pdma_timer);
	u32 u32PDMA_INT;

	spin_lock(&up->port.lock);

	u32PDMA_INT = pdma_read(up, UPDMA_INTSTS);

	if( u32PDMA_INT & (0x1 << (up->pdma_rx_ch + 8)) ) {
		ma35d1_PUART_Rx_dma_callback(up);
	}

	mod_timer(&up->rx_pdma_timer, jiffies + HZ);

	spin_unlock(&up->port.lock);
}

static int ma35d1_PUART_startup(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	int retval;

	dma_cap_mask_t mask;

	if (up->uart_pdma_enable_flag == 1) {
		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);
		dma_cap_set(DMA_PRIVATE, mask);
	}

	/* Reset FIFO */
	pserial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

	/* Clear pending interrupts */
	pserial_out(up, UART_REG_ISR, 0xFFFFFFFF);

	retval = request_irq(port->irq, ma35d1_PUART_interrupt, 0, 
						"ma35d1_puart_serial", port);
	if (retval) {
		dev_err(up->port.dev, "request serial irq failed.\n");
		return retval;
	}

	retval = request_irq(up->pdma_irq, ma35d1_PUART_PDMA_Int, IRQF_SHARED, 
						"ma35d1_puart_pdma", port);
	if (retval) {
		dev_err(up->port.dev, "request pdma irq failed.\n");
		return retval;
	}

	/* Now, initialize the UART */
	/* FIFO trigger level 4 byte */
	/* RTS trigger level 8 bytes */
	pserial_out(up, UART_REG_FCR, pserial_in(up,
			UART_REG_FCR) | 0x10 | 0x20000);

	pserial_out(up, UART_REG_LCR, 0x7); /* 8 bit */
	pserial_out(up, UART_REG_TOR, 0x40);

	if (up->uart_pdma_enable_flag == 1)
		pserial_out(up, UART_REG_IER,
				RTO_IEN | RLS_IEN | BUFERR_IEN |
				TIME_OUT_EN | BUFERR_IEN);
	else
		pserial_out(up, UART_REG_IER,
				RTO_IEN | RDA_IEN | TIME_OUT_EN | BUFERR_IEN);

	if (up->uart_pdma_enable_flag == 1)
		up->baud_rate = 0;

	timer_setup(&up->rx_pdma_timer, ma35d1_PUART_timer, 0);
	mod_timer(&up->rx_pdma_timer, jiffies + HZ);

	return 0;
}

static void ma35d1_PUART_shutdown(struct uart_port *port)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;

	if (up->uart_pdma_enable_flag == 1) {
		if (up->dest_mem_p.size != 0)
			kfree((void *)up->dest_mem_p.vir_addr);

		if (up->src_mem_p.size != 0)
			kfree((void *)up->src_mem_p.vir_addr);

		up->Tx_pdma_busy_flag = 0;
		up->dest_mem_p.size = 0;
		up->src_mem_p.size = 0;
	}

	free_irq(port->irq, port);
	free_irq(up->pdma_irq, port);

	del_timer(&up->rx_pdma_timer);

	/* Disable interrupts from this port */
	pserial_out(up, UART_REG_IER, 0);
}

static unsigned int ma35d1_PUART_get_divisor(struct uart_port *port,
											unsigned int baud)
{
	unsigned int quot;

	quot = (port->uartclk / baud) - 2;
	return quot;
}

static void
ma35d1_PUART_set_termios(struct uart_port *port,
						struct ktermios *termios, struct ktermios *old)
{
	struct puart_ma35d1_port *up = (struct puart_ma35d1_port *)port;
	unsigned int lcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = 0;
		break;
	case CS6:
		lcr |= 1;
		break;
	case CS7:
		lcr |= 2;
		break;
	default:
	case CS8:
		lcr |= 3;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= NSB;
	if (termios->c_cflag & PARENB)
		lcr |= PBE;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPE;
	if (termios->c_cflag & CMSPAR)
		lcr |= SPE;

	baud = uart_get_baud_rate(port, termios, old, port->uartclk / 0xffff,
								port->uartclk / 11);

	quot = ma35d1_PUART_get_divisor(port, baud);

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = RX_OVER_IF;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= FEF | PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= BIF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= FEF | PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= BIF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= RX_OVER_IF;
	}

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE;
	else
		up->mcr &= ~UART_MCR_AFE;

	ma35d1_PUART_set_mctrl(&up->port, up->port.mctrl);
	pserial_out(up, UART_REG_BAUD, quot | 0x30000000);
	pserial_out(up, UART_REG_LCR, lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);

	if (up->uart_pdma_enable_flag == 1) {
		if (up->baud_rate != baud) {
			up->baud_rate = baud;
			ma35d1_puart_cal_pdma_time_out(up, baud);
			ma35d1_PUART_prepare_RX_dma(up);
			/* trigger pdma */
			pserial_out(up, UART_REG_IER,
					(pserial_in(up, UART_REG_IER)|RXPDMAEN));
		}
	}
}

static void ma35d1_PUART_release_port(struct uart_port *port)
{
	struct puart_ma35d1_port *p = (struct puart_ma35d1_port *)port;
	struct ma35d1_ip_rx_dma *pdma_rx = &(p->dma_rx);
	struct ma35d1_ip_tx_dma *pdma_tx = &(p->dma_tx);

	if (p->uart_pdma_enable_flag == 1) {
		dma_unmap_single(pdma_rx->chan_rx->device->dev,
			p->dest_mem_p.phy_addr, UART_RX_BUF_SIZE, DMA_FROM_DEVICE);
		dma_unmap_single(pdma_tx->chan_tx->device->dev,
			p->src_mem_p.phy_addr, p->src_mem_p.size, DMA_TO_DEVICE);
	}

	iounmap(port->membase);
	iounmap(p->pdma_membase);
	port->membase = NULL;
	p->pdma_membase = NULL;
}

static int ma35d1_PUART_request_port(struct uart_port *port)
{
	return 0;
}

static void ma35d1_PUART_config_port(struct uart_port *port, int flags)
{
	int ret;

	ret = ma35d1_PUART_request_port(port);
	if (ret < 0)
		return;
	port->type = PORT_MA35D1;
}

static int ma35d1_PUART_verify_port(struct uart_port *port,
									struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_MA35D1)
		return -EINVAL;
	return 0;
}

static const char *ma35d1_PUART_type(struct uart_port *port)
{
	return (port->type == PORT_MA35D1) ? "MA35D1" : NULL;
}

static const struct uart_ops ma35d1_PUART_ops = {
	.tx_empty     = ma35d1_PUART_tx_empty,
	.set_mctrl    = ma35d1_PUART_set_mctrl,
	.get_mctrl    = ma35d1_PUART_get_mctrl,
	.stop_tx      = ma35d1_PUART_stop_tx,
	.start_tx     = ma35d1_PUART_start_tx,
	.stop_rx      = ma35d1_PUART_stop_rx,
	.break_ctl    = ma35d1_PUART_break_ctl,
	.startup      = ma35d1_PUART_startup,
	.shutdown     = ma35d1_PUART_shutdown,
	.set_termios  = ma35d1_PUART_set_termios,
	.type         = ma35d1_PUART_type,
	.release_port = ma35d1_PUART_release_port,
	.request_port = ma35d1_PUART_request_port,
	.config_port  = ma35d1_PUART_config_port,
	.verify_port  = ma35d1_PUART_verify_port,
};

static const struct of_device_id ma35d1_puart_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-puart" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_puart_of_match);

static struct uart_driver ma35d1_puart_reg = {
	.owner        = THIS_MODULE,
	.driver_name  = "p_uart",
	.dev_name     = "ttyPS",
	.major        = TTY_MAJOR,
	.minor        = 105,
	.nr           = UART_NR,
};

void ma35d1_PUART_suspend_port(int line)
{
	uart_suspend_port(&ma35d1_puart_reg, &puart_ma35d1_ports[line].port);
}
EXPORT_SYMBOL(ma35d1_PUART_suspend_port);

void ma35d1_PUART_resume_port(int line)
{
	struct puart_ma35d1_port *up = &puart_ma35d1_ports[line];

	uart_resume_port(&ma35d1_puart_reg, &up->port);
}
EXPORT_SYMBOL(ma35d1_PUART_resume_port);

static int  get_puart_port_number(struct platform_device *pdev)
{
	u32   val32[2];

	if (of_property_read_u32_array(pdev->dev.of_node, "port-number",
													val32, 1) != 0) {
		dev_err(&pdev->dev, "can not get port-number!\n");
		return -EINVAL;
	}

	return val32[0];
}

static int ma35d1_PUART_probe(struct platform_device *pdev)
{
	struct resource *res_mem;
	struct puart_ma35d1_port *up;
	struct clk *clk;
	int ret, i;		
	int err;
	u32	val32[2];

	i = get_puart_port_number(pdev);
	if (i < 0)
		return -EINVAL;

	up = &puart_ma35d1_ports[i];

	up->port.line = i;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;

	up->port.iobase = res_mem->start;
	up->port.membase = ioremap(up->port.iobase, 0x10000);
	up->port.ops = &ma35d1_PUART_ops;

	spin_lock_init(&up->port.lock);

	clk = devm_clk_get(&pdev->dev, "puart_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no puart0 clock found\n");
		ret = -ENODEV;
	}

	err = clk_prepare_enable(clk);
	if (err) {
		dev_err(&pdev->dev, "could not enable puart0 clk\n");
		return ret;
	}

	if (up->port.line != 0)
		up->port.uartclk = clk_get_rate(clk);

	clk = devm_clk_get(&pdev->dev, "pdma_clk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no puart1 clock found\n");
		ret = -ENODEV;
	}

	err = clk_prepare_enable(clk);
	if (err) {
		dev_err(&pdev->dev, "could not enable puart1 clk\n");
		return ret;
	}

	if (of_property_read_u32_array(pdev->dev.of_node, "pdma-port",
													val32, 1) != 0) {
		dev_err(&pdev->dev, "can not get pdma-port !\n");
		return -EINVAL;
	}

	up->pdma_port = val32[0];

	if (of_property_read_u32_array(pdev->dev.of_node, "pdma-tx-ch",
													val32, 1) != 0) {
		dev_err(&pdev->dev, "can not get pdma-tx-ch !\n");
		return -EINVAL;
	}

	up->pdma_tx_ch = val32[0];

	if (of_property_read_u32_array(pdev->dev.of_node, "pdma-rx-ch",
													val32, 1) != 0) {
		dev_err(&pdev->dev, "can not get pdma-tx-ch !\n");
		return -EINVAL;
	}

	up->pdma_rx_ch = val32[0];

	if (up->pdma_port == 0)
		up->pdma_iobase = 0x40080000;
	else if (up->pdma_port == 1)
		up->pdma_iobase = 0x40090000;
	else if (up->pdma_port == 2)
		up->pdma_iobase = 0x400a0000;
	else if (up->pdma_port == 3)
		up->pdma_iobase = 0x400b0000;

	up->pdma_membase = ioremap(up->pdma_iobase, 0x10000);

	up->uart_pdma_enable_flag = 1;

	up->port.irq = platform_get_irq_byname(pdev, "puart_uart");
	if (up->port.irq < 0) {
		ret = -EINVAL;
		return ret;
	}

	up->pdma_irq = platform_get_irq_byname(pdev, "puart_dma");
	if (up->pdma_irq < 0) {
		ret = -EINVAL;
		return ret;
	}

	up->port.dev = &pdev->dev;
	up->port.flags = UPF_BOOT_AUTOCONF;

	ret = uart_add_one_port(&ma35d1_puart_reg, &up->port);

	platform_set_drvdata(pdev, up);

	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int ma35d1_PUART_remove(struct platform_device *dev)
{
	int i;
	struct uart_port *port = platform_get_drvdata(dev);

	free_irq(port->irq, port);

	for (i = 0; i < UART_NR; i++) {
		struct puart_ma35d1_port *up = &puart_ma35d1_ports[i];

		if (up->port.dev == &dev->dev)
			uart_remove_one_port(&ma35d1_puart_reg, &up->port);
	}
	return 0;
}

static int ma35d1_PUART_suspend(struct platform_device *dev,
								pm_message_t state)
{
	int i;

	i = get_puart_port_number(dev);
	if (i < 0)
		return i;


	return 0;
}

static int ma35d1_PUART_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver ma35d1_PUART_driver = {
	.probe      = ma35d1_PUART_probe,
	.remove     = ma35d1_PUART_remove,
	.suspend    = ma35d1_PUART_suspend,
	.resume     = ma35d1_PUART_resume,
	.driver     = {
		.name   = "ma35d1-puart",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ma35d1_puart_of_match),
	},
};

static int __init ma35d1_PUART_init(void)
{
	int ret;

	ret = uart_register_driver(&ma35d1_puart_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&ma35d1_PUART_driver);
	if (ret)
		uart_unregister_driver(&ma35d1_puart_reg);

	return ret;
}

static void __exit ma35d1_PUART_exit(void)
{
	platform_driver_unregister(&ma35d1_PUART_driver);
	uart_unregister_driver(&ma35d1_puart_reg);
}

module_init(ma35d1_PUART_init);
module_exit(ma35d1_PUART_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MA35D1 pdma serial driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);

