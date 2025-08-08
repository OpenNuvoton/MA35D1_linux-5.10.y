// SPDX-License-Identifier: GPL-2.0
/*
 * MA35D1 EADC driver
 *
 * Copyright (c) 2022 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/of.h>

#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/platform_data/dma-ma35d1.h>

#include <linux/clk.h>

/* ma35d1 eadc registers offset */
#define DAT0 0x00
#define DAT1 0x04
#define DAT2 0x08
#define DAT3 0x0C
#define DAT4 0x10
#define DAT5 0x14
#define DAT6 0x18
#define DAT7 0x1C
#define DAT8 0x20
#define CURDAT 0x4C
#define CTL 0x50
#define SWTRG 0x54
#define SCTL0 0x80
#define SCTL1 0x84
#define SCTL2 0x88
#define SCTL3 0x8C
#define SCTL4 0x90
#define SCTL5 0x94
#define SCTL6 0x98
#define SCTL7 0x9C
#define INTSRC0 0xD0
#define INTSRC1 0xD4
#define INTSRC2 0xD8
#define INTSRC3 0xDC
#define STATUS0 0xF0
#define STATUS2 0xF8
#define STATUS3 0xFC
#define PWRM 0x110
#define PDMACTL 0x130
#define SELSMP0 0x140
#define SELSMP1 0x144
#define REFADJCTL 0x150

#define ADCEN 1
#define DIFFEN 0x100
#define PWRUPRDY 1
#define ADCIEN0 4
#define CHSELMSK 0xF
#define DATMSK 0xFFF
#define TRGSELMSK 0x3F0000
#define TRGDLYMSK 0xFF00
#define ADINT0TRG 0x20000
#define TRGSELPOS 16

#define EADC_CH_MAX 9 /* max number of channels */
#define EADC_CH_SZ 10 /* max channel name size */
#define EADC_MAX_SP 16

#define MA35D1_ADC_TIMEOUT (msecs_to_jiffies(1000))
#define MA35D1_DMA_BUFFER_SIZE PAGE_SIZE

#define ADC_CHANNEL(_index, _id)                                               \
	{			\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = _index,				\
	.address = _index,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.datasheet_name = _id,				\
	.scan_index = _index,				\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = 12,				\
		.storagebits = 16,			\
		.shift = 0,				\
		.endianness = IIO_BE,			\
	},						\
}

struct ma35d1_ip_dma {
	struct dma_chan *chan_rx;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_slave_config slave_config;
	u8 *rx_buf;
	unsigned int rx_buf_sz;
	dma_addr_t rx_dma_buf;
};

struct ma35d1_adc_diff_channel {
	u32 vinp;
	u32 vinn;
};

struct ma35d1_adc_device {
	struct clk *clk;
	struct clk *eclk;
	unsigned int irq;
	void __iomem *regs;
	struct completion completion;
	struct iio_trigger *trig;
	u16 buffer[EADC_MAX_SP];
	unsigned int bufi;
	unsigned int num_conv;
	unsigned int scan_chancnt;
	char chan_name[EADC_CH_MAX][EADC_CH_SZ];
	unsigned int use_pdma;
	spinlock_t lock; /* interrupt lock */
	unsigned int pdma_reqsel_rx;
	struct ma35d1_ip_dma dma;
	unsigned int phyaddr;
};

static const struct iio_chan_spec ma35d1_adc_iio_channels[] = {
	ADC_CHANNEL(0, "adc0"), ADC_CHANNEL(1, "adc1"), ADC_CHANNEL(2, "adc2"),
	ADC_CHANNEL(3, "adc3"), ADC_CHANNEL(4, "adc4"), ADC_CHANNEL(5, "adc5"),
	ADC_CHANNEL(6, "adc6"), ADC_CHANNEL(7, "adc7"),

};

static irqreturn_t ma35d1_adc_isr(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int i;

	if (readl(info->regs + STATUS2) & 1) { /* check ADIF0 */
		writel(1, info->regs + STATUS2); /* clear ADIF0 */

		/* Reading DR also clears EOC status flag */
		/* Check if IIO buffer enabled */
		if (iio_buffer_enabled(indio_dev)) {
			if (!info->dma.chan_rx) {
				for (i = 0; i < info->scan_chancnt; i++) {
					info->buffer[info->bufi] =
						readl(info->regs + DAT0 +
						      (i << 2)) &
						DATMSK;
					info->bufi++;
				}
				if (info->bufi >= info->num_conv) {
					/* disable ADCIEN0 */
					writel(readl(info->regs + CTL) &
						       ~ADCIEN0,
					       info->regs + CTL);
					iio_trigger_poll(indio_dev->trig);
				}
			}
		} else {
			info->buffer[info->bufi] =
				readl(info->regs + DAT0) & DATMSK;
			complete(&info->completion);
		}
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

static void ma35d1_adc_channels_remove(struct iio_dev *indio_dev)
{
	kfree(indio_dev->channels);
}

static void ma35d1_adc_buffer_remove(struct iio_dev *idev)
{
	iio_triggered_buffer_cleanup(idev);
}

static void nuvoton_adc_chan_init_one(struct iio_dev *indio_dev,
				      struct iio_chan_spec *chan, u32 vinp,
				      u32 vinn, int scan_index,
				      bool differential)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	char *name = info->chan_name[vinp];

	chan->type = IIO_VOLTAGE;
	chan->channel = vinp;
	if (differential) {
		chan->differential = 1;
		chan->channel2 = vinn;
		snprintf(name, EADC_CH_SZ, "in%d-in%d", vinp, vinn);
	} else {
		snprintf(name, EADC_CH_SZ, "in%d", vinp);
	}
	chan->datasheet_name = name;
	chan->scan_index = scan_index;
	chan->indexed = 1;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
}

static int ma35d1_adc_chan_of_init(struct iio_dev *indio_dev)
{
	struct device_node *node = indio_dev->dev.of_node;
	struct ma35d1_adc_diff_channel diff[EADC_CH_MAX];
	struct property *prop;
	const __be32 *cur;
	struct iio_chan_spec *channels;
	int scan_index = 0, num_channels = 0, num_diff = 0, ret, i;
	u32 val;

	ret = of_property_count_u32_elems(node, "eadc-channels");
	if (ret > EADC_CH_MAX) {
		dev_err(&indio_dev->dev, "Bad eadc-channels?\n");
		return -EINVAL;
	} else if (ret > 0) {
		num_channels += ret;
	}

	ret = of_property_count_elems_of_size(node, "eadc-diff-channels",
					      sizeof(*diff));
	if (ret > EADC_CH_MAX) {
		dev_err(&indio_dev->dev, "Bad eadc-diff-channels?\n");
		return -EINVAL;
	} else if (ret > 0) {
		int size = ret * sizeof(*diff) / sizeof(u32);

		num_diff = ret;
		num_channels += ret;
		ret = of_property_read_u32_array(node, "eadc-diff-channels",
						 (u32 *)diff, size);
		if (ret)
			return ret;
	}

	if (!num_channels) {
		dev_err(&indio_dev->dev, "No channels configured\n");
		return -ENODATA;
	}

	channels = devm_kcalloc(&indio_dev->dev, num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	of_property_for_each_u32 (node, "eadc-channels", prop, cur, val) {
		if (val >= EADC_CH_MAX) {
			dev_err(&indio_dev->dev, "Invalid channel %d\n", val);
			return -EINVAL;
		}

		/* Channel can't be configured both as single-ended & diff */
		for (i = 0; i < num_diff; i++) {
			if (val == diff[i].vinp) {
				dev_err(&indio_dev->dev,
					"channel %d miss-configured\n", val);
				return -EINVAL;
			}
		}
		nuvoton_adc_chan_init_one(indio_dev, &channels[scan_index], val,
					  0, scan_index, false);
		scan_index++;
	}

	for (i = 0; i < num_diff; i++) {
		if (diff[i].vinp >= EADC_CH_MAX ||
		    diff[i].vinn >= EADC_CH_MAX) {
			dev_err(&indio_dev->dev, "Invalid channel in%d-in%d\n",
				diff[i].vinp, diff[i].vinn);
			return -EINVAL;
		}
		nuvoton_adc_chan_init_one(indio_dev, &channels[scan_index],
					  diff[i].vinp, diff[i].vinn,
					  scan_index, true);
		scan_index++;
	}

	indio_dev->num_channels = scan_index;
	indio_dev->channels = channels;

	return 0;
}

static int ma35d1_adc_dma_request(struct device *dev, struct iio_dev *indio_dev)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int ret;

	info->dma.chan_rx = dma_request_slave_channel(dev, "rx");
	if (IS_ERR(info->dma.chan_rx)) {
		ret = PTR_ERR(info->dma.chan_rx);
		if (ret != -ENODEV)
			return dev_err_probe(
				dev, ret, "DMA channel request failed with\n");

		pr_info("%s:dma_request_chan failed info->dma.chan_rx = NULL\n",
			__func__);
		/* DMA is optional: fall back to IRQ mode */
		info->dma.chan_rx = NULL;
		return 0;
	}

	info->dma.rx_buf_sz = 1024;
	info->dma.rx_buf =
		dma_alloc_coherent(info->dma.chan_rx->device->dev,
				   info->dma.rx_buf_sz, &info->dma.rx_dma_buf,
				   GFP_KERNEL);
	if (!info->dma.rx_buf) {
		pr_info("%s:dma_alloc_coherent failed dma_release_channel\n",
			__func__);
		ret = -ENOMEM;
		goto err_release;
	}
	info->dma.slave_config.direction = DMA_DEV_TO_MEM;
	info->dma.slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	info->dma.slave_config.src_addr = info->phyaddr + CURDAT;
	ret = dmaengine_slave_config(info->dma.chan_rx,
				     &(info->dma.slave_config));
	if (ret)
		goto err_free;

	return 0;

err_free:
	dma_free_coherent(info->dma.chan_rx->device->dev,
			  MA35D1_DMA_BUFFER_SIZE, info->dma.rx_buf,
			  info->dma.rx_dma_buf);
err_release:
	dma_release_channel(info->dma.chan_rx);

	return ret;
}

static int ma35d1_adc_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int *val,
			       int *val2, long mask)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	unsigned long timeout;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	reinit_completion(&info->completion);

	/* enable channel */
	writel((readl(info->regs + SCTL0) & ~CHSELMSK) | chan->channel,
	       info->regs + SCTL0);

	/* check if the channel is designated as differential channel */
	if (chan->differential)
		writel((readl(info->regs + CTL) | DIFFEN), info->regs + CTL);
	else
		writel((readl(info->regs + CTL) & ~DIFFEN), info->regs + CTL);

	/* software trigger sample module 0 */
	writel(1, info->regs + SWTRG);

	timeout = wait_for_completion_interruptible_timeout(&info->completion,
							    MA35D1_ADC_TIMEOUT);

	*val = readl(info->regs + DAT0) & DATMSK;
	/* *val = info->buffer[0]; */

	mutex_unlock(&indio_dev->mlock);

	if (timeout == 0)
		return -ETIMEDOUT;

	return IIO_VAL_INT;
}

static int ma35d1_adc_conf_scan_seq(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	u32 bit;
	int i = 0;

	for_each_set_bit (bit, scan_mask, indio_dev->masklength) {
		chan = indio_dev->channels + bit;
		/*
		 * Assign one channel per Sample module
		 */

		dev_dbg(&indio_dev->dev, "%s chan %d to Sample module %d\n",
			__func__, chan->channel, i);

		/* configure sample module trigger source to ADINT0 */
		writel((readl(info->regs + SCTL0 + (i << 2)) & ~TRGSELMSK) |
			       ADINT0TRG,
		       info->regs + SCTL0 + (i << 2));
		/* configure sample module channel select */
		writel((readl(info->regs + SCTL0 + (i << 2)) & ~CHSELMSK) |
			       chan->channel,
		       info->regs + SCTL0 + (i << 2));
		/* check if the channel is designated as differential channel */
		if (chan->differential)
			writel((readl(info->regs + CTL) | DIFFEN),
			       info->regs + CTL);
		else
			writel((readl(info->regs + CTL) & ~DIFFEN),
			       info->regs + CTL);

		i++;
		if (i > EADC_MAX_SP)
			return -EINVAL;
	}
	info->scan_chancnt = i;

	if (!i)
		return -EINVAL;

	return 0;
}

static int ma35d1_adc_update_scan_mode(struct iio_dev *indio_dev,
				       const unsigned long *scan_mask)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int ret;

	info->num_conv = bitmap_weight(scan_mask, indio_dev->masklength);

	ret = ma35d1_adc_conf_scan_seq(indio_dev, scan_mask);

	return ret;
}

static unsigned int ma35d1_adc_dma_residue(struct ma35d1_adc_device *info)
{
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(info->dma.chan_rx,
				     info->dma.chan_rx->cookie, &state);
	if (status == DMA_IN_PROGRESS) {
		/* Residue is size in bytes from end of buffer */
		unsigned int i = info->dma.rx_buf_sz - state.residue;
		unsigned int size;

		/* Return available bytes */
		if (i >= info->bufi)
			size = i - info->bufi;
		else
			size = info->dma.rx_buf_sz + i - info->bufi;

		return size;
	}

	return 0;
}

static irqreturn_t ma35d1_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int channel;
	int ret_push;

	if (!info->dma.chan_rx) {
		channel = find_first_bit(indio_dev->active_scan_mask,
					 indio_dev->masklength);

		/* reset buffer index */
		info->bufi = 0;
		ret_push = iio_push_to_buffers_with_timestamp(
			indio_dev, info->buffer, pf->timestamp);
		if (ret_push) {
			/* Set trigger source to software trigger (clear ADINT0 trigger) */
			writel((readl(info->regs + SCTL0) & ~TRGSELMSK),
			       info->regs + SCTL0);
		}
	} else {
		int residue = ma35d1_adc_dma_residue(info);
		int dma_available = info->dma.rx_buf_sz - residue;

		while (dma_available >= indio_dev->scan_bytes) {
			u16 *buffer = (u16 *)&info->dma.rx_buf[info->bufi];

			iio_push_to_buffers_with_timestamp(indio_dev, buffer,
							   pf->timestamp);

			info->bufi += indio_dev->scan_bytes;
			if (info->bufi >= info->dma.rx_buf_sz)
				info->bufi = 0;

			dma_available -= indio_dev->scan_bytes;
		}
	}

	iio_trigger_notify_done(indio_dev->trig);
	/* re-enable eoc irq */
	writel(readl(info->regs + CTL) | ADCIEN0, info->regs + CTL);
	return IRQ_HANDLED;
}

static void ma35d1_adc_dma_buffer_done(void *data)
{
	struct iio_dev *indio_dev = data;
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	int residue = ma35d1_adc_dma_residue(info);

	/*
	 * In DMA mode the trigger services of IIO are not used
	 * (e.g. no call to iio_trigger_poll).
	 * Calling irq handler associated to the hardware trigger is not
	 * relevant as the conversions have already been done. Data
	 * transfers are performed directly in DMA callback instead.
	 * This implementation avoids to call trigger irq handler that
	 * may sleep, in an atomic context (DMA irq handler context).
	 */
	//pr_info("%s\n",__func__);

	//dev_info(&indio_dev->dev, "%s bufi=%d\n", __func__, info->bufi);

	while (residue >= indio_dev->scan_bytes) {
		u16 *buffer = (u16 *)&info->dma.rx_buf[info->bufi];

		iio_push_to_buffers(indio_dev, buffer);

		residue -= indio_dev->scan_bytes;
		info->bufi += indio_dev->scan_bytes;
		if (info->bufi >= info->dma.rx_buf_sz)
			info->bufi = 0;
	}
}

static int ma35d1_adc_dma_start(struct iio_dev *indio_dev)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	struct ma35d1_peripheral pcfg;
	dma_cookie_t cookie;
	int ret;
	if (!info->dma.chan_rx)
		return 0;

	dmaengine_terminate_all(info->dma.chan_rx);
	pcfg.reqsel = info->pdma_reqsel_rx;
	info->dma.slave_config.peripheral_config = &pcfg;
	info->dma.slave_config.peripheral_size = sizeof(pcfg);
	dmaengine_slave_config(info->dma.chan_rx, &(info->dma.slave_config));

	/* Prepare a DMA cyclic transaction */
	info->dma.rxdesc = dmaengine_prep_dma_cyclic(
		info->dma.chan_rx, info->dma.rx_dma_buf, info->dma.rx_buf_sz,
		info->dma.rx_buf_sz / 2, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!info->dma.rxdesc)
		return -EBUSY;

	info->dma.rxdesc->callback = ma35d1_adc_dma_buffer_done;
	info->dma.rxdesc->callback_param = indio_dev;

	cookie = dmaengine_submit(info->dma.rxdesc);
	ret = dma_submit_error(cookie);

	if (ret) {
		dmaengine_terminate_sync(info->dma.chan_rx);
		return ret;
	}

	/* Issue pending DMA requests */
	dma_async_issue_pending(info->dma.chan_rx);
	writel(1, info->regs + SWTRG);
	return 0;
}

static int __ma35d1_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	int ret;
	int chan_idx;

	chan_idx = find_first_bit(indio_dev->active_scan_mask,
				  indio_dev->masklength);
	chan = &indio_dev->channels[chan_idx];
	/* Reset adc buffer index */
	info->bufi = 0;
	/* clear ADIF0 */
	writel(1, info->regs + STATUS2);

	if (info->dma.chan_rx) {
		/* Disable interrupt */
		writel(readl(info->regs + CTL) & ~ADCIEN0, info->regs + CTL);

		writel(readl(info->regs + INTSRC0) | 1, info->regs + INTSRC0);
		/* Enable DAT0 PDMA */
		writel(0x3FF, info->regs + PDMACTL);
		/* set reference voltage from external Vref pin */
		writel(readl(info->regs + REFADJCTL) | 1,
		       info->regs + REFADJCTL);
		/* set sampling time */
		writel(readl(info->regs + SELSMP0) | 3, info->regs + SELSMP0);
		/* set trigger delay count */
		writel(readl(info->regs + SCTL0) | TRGDLYMSK,
		       info->regs + SCTL0);
		/* Set trigger source to ADINT0 */
		writel((readl(info->regs + SCTL0) & ~TRGSELMSK) | ADINT0TRG,
		       info->regs + SCTL0);

		/* enable channel */
		writel((readl(info->regs + SCTL0) & ~CHSELMSK) | chan->channel,
		       info->regs + SCTL0);

		/* check if the channel is designated as differential channel */
		if (chan->differential)
			writel((readl(info->regs + CTL) | DIFFEN),
			       info->regs + CTL);
		else
			writel((readl(info->regs + CTL) & ~DIFFEN),
			       info->regs + CTL);

		ret = ma35d1_adc_dma_start(indio_dev);
		if (ret) {
			dev_err(&indio_dev->dev,
				"PDMA start failed, fallback to IRQ mode: %d\n",
				ret);
			info->dma.chan_rx = NULL;
		}

		//writel(1, info->regs + SWTRG);
	} else {
		pr_info("%s NO DMA MODE", __func__);
		/* enable ADCIEN0 */
		writel(readl(info->regs + CTL) | ADCIEN0, info->regs + CTL);
		/* enable ADINT0 for sample module 0 */
		writel(readl(info->regs + INTSRC0) | 1, info->regs + INTSRC0);
		/* set reference voltage from external Vref pin */
		writel(readl(info->regs + REFADJCTL) | 1,
		       info->regs + REFADJCTL);
		/* set sampling time */
		writel(readl(info->regs + SELSMP0) | 3, info->regs + SELSMP0);
		/* set trigger delay count */
		writel(readl(info->regs + SCTL0) | TRGDLYMSK,
		       info->regs + SCTL0);
		/* Set trigger source to ADINT0 */
		writel((readl(info->regs + SCTL0) & ~TRGSELMSK) | ADINT0TRG,
		       info->regs + SCTL0);

		/* software trigger sample module 0 */
		writel(1, info->regs + SWTRG);
	}

	return 0;
}

static int ma35d1_adc_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret;

	ret = __ma35d1_adc_buffer_postenable(indio_dev);

	return ret;
}

static void __ma35d1_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ma35d1_adc_device *info = iio_priv(indio_dev);

	if (info->dma.chan_rx)
		dmaengine_terminate_sync(info->dma.chan_rx);
	/* disable ADCIEN0 */
	writel(readl(info->regs + CTL) & ~ADCIEN0, info->regs + CTL);
	/* Clear trigger source to software trigger, not ADINT0 trigger */
	writel((readl(info->regs + SCTL0) & ~TRGSELMSK), info->regs + SCTL0);
}

static int ma35d1_adc_buffer_predisable(struct iio_dev *indio_dev)
{
	__ma35d1_adc_buffer_predisable(indio_dev);

	return 0;
}

static const struct iio_buffer_setup_ops ma35d1_ring_setup_ops = {
	.postenable = &ma35d1_adc_buffer_postenable,
	.predisable = &ma35d1_adc_buffer_predisable,
};

static const struct iio_info ma35d1_adc_info = {
	.read_raw = &ma35d1_adc_read_raw,
	.update_scan_mode = &ma35d1_adc_update_scan_mode,
};

static int ma35d1_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ma35d1_adc_device *info = NULL;
	irqreturn_t (*handler)(int irq, void *p) = NULL;
	int ret = -ENODEV;
	struct resource *res;
	int irq;
	int err = 0;
	const char *clkgate;
	u32 freq;
	u32 val32[4];

	indio_dev = devm_iio_device_alloc(&pdev->dev,
					  sizeof(struct ma35d1_adc_device));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	spin_lock_init(&info->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		return -ENOENT;
	}

	info->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(info->regs)) {
		dev_err(&pdev->dev, "cannot map IO\n");
		return PTR_ERR(info->regs);
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ma35d1_adc_info;
	indio_dev->num_channels = 8;
	indio_dev->channels = ma35d1_adc_iio_channels;
	indio_dev->masklength = indio_dev->num_channels - 1;

	if (of_property_read_u32(pdev->dev.of_node, "eadc-frequency", &freq)) {
		panic("missing 'eadc-frequency' property");
	}

	of_property_read_string(pdev->dev.of_node, "clock-enable", &clkgate);
	info->eclk = devm_clk_get(&pdev->dev, "eadc_gate");
	if (IS_ERR(info->eclk)) {
		if (PTR_ERR(info->eclk) != -ENOENT)
			return PTR_ERR(info->eclk);
		info->eclk = NULL;
	}

	err = clk_prepare_enable(info->eclk);
	if (err)
		return -ENOENT;

	clk_set_rate(info->eclk, freq);

	if (of_property_read_u32(pdev->dev.of_node, "use_pdma",
				 &info->use_pdma)) {
		info->use_pdma = 0;
		dev_warn(&pdev->dev, "can't get use_pdma from dt, default=0\n");
	}

	if (info->use_pdma) {
		if (of_property_read_u32_array(pdev->dev.of_node, "reg", val32,
					       4)) {
			dev_err(&pdev->dev, "can not get bank!\n");
			return -EINVAL;
		}

		info->phyaddr = val32[1];
		dev_err(&pdev->dev, "info->phyaddr = 0x%lx\n",
			(ulong)info->phyaddr);

		ret = of_property_read_u32(pdev->dev.of_node, "pdma_reqsel_rx",
					   &info->pdma_reqsel_rx);
		if (ret) {
			dev_err(&pdev->dev, "cannot get pdma_reqsel_rx\n");
			return ret;
		}
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = irq;
		goto err_clk_disable;
	}

	info->irq = irq;
	init_completion(&info->completion);

	ret = devm_request_irq(&pdev->dev, info->irq, ma35d1_adc_isr, 0,
			       dev_name(&pdev->dev), indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed requesting irq, irq = %d\n",
			info->irq);
		goto err_clk_disable;
	}

	ret = ma35d1_adc_chan_of_init(indio_dev);
	if (ret) {
		goto err_clk_disable;
	}

	if (info->use_pdma) {
		ret = ma35d1_adc_dma_request(&pdev->dev, indio_dev);
		if (ret)
			goto err_clk_disable;
	}
	writel(readl(info->regs + CTL) | ADCEN, info->regs + CTL);
	writel(1, info->regs + STATUS2);
	writel(readl(info->regs + CTL) | ADCIEN0, info->regs + CTL);
	writel(readl(info->regs + INTSRC0) | 1, info->regs + INTSRC0);
	writel(readl(info->regs + REFADJCTL) | 1, info->regs + REFADJCTL);
	writel(readl(info->regs + SELSMP0) | 3, info->regs + SELSMP0);

	if (!info->dma.chan_rx)
		handler = &ma35d1_trigger_handler;

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					 handler, &ma35d1_ring_setup_ops);
	if (ret)
		goto err_disable_adc;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register MA35D1 EADC..\n");
		goto err_cleanup_buffer;
	}

	platform_set_drvdata(pdev, indio_dev);
	dev_dbg(&pdev->dev, "%s: ma35d1 EADC\n", indio_dev->name);

	//writel(1, info->regs + SWTRG);

	return 0;

err_cleanup_buffer:
	iio_triggered_buffer_cleanup(indio_dev);
err_disable_adc:
	writel(readl(info->regs + CTL) & ~ADCEN, info->regs + CTL);
err_clk_disable:
	clk_disable_unprepare(info->eclk);
	return ret;
}

static int ma35d1_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct ma35d1_adc_device *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ma35d1_adc_channels_remove(indio_dev);

	iio_device_free(indio_dev);

	/* disable ADCEN */
	writel(readl(info->regs + CTL) & ~ADCEN, info->regs + CTL);

	clk_disable_unprepare(info->eclk);

	ma35d1_adc_buffer_remove(indio_dev);
	free_irq(info->irq, info);

	return 0;
}

#ifdef CONFIG_PM
static int ma35d1_adc_suspend(struct device *dev)
{
	return 0;
}

static int ma35d1_adc_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ma35d1_adc_pm_ops = {
	.suspend = ma35d1_adc_suspend,
	.resume = ma35d1_adc_resume,
};
#define MA35D1_ADC_PM_OPS (&ma35d1_adc_pm_ops)
#else
#define MA35D1_ADC_PM_OPS NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id ma35d1_eadc_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-eadc" },
	{},
};
MODULE_DEVICE_TABLE(of, ma35d1_eadc_of_match);
#endif

static struct platform_driver ma35d1_adc_driver = {
	.driver = {
		.name   = "ma35d1-eadc",
		.owner	= THIS_MODULE,
		.pm	= MA35D1_ADC_PM_OPS,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(ma35d1_eadc_of_match),
#endif
	},
	.probe	= ma35d1_adc_probe,
	.remove	= ma35d1_adc_remove,
};

module_platform_driver(ma35d1_adc_driver);

MODULE_DESCRIPTION("MA35D1 EADC controller driver");
MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ma35d1-eadc");
