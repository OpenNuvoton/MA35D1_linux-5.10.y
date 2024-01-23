/*
 *  linux/drivers/rpmsg/ma35d1_rpmsg.c
 *
 *  MA35D1 rpmsg driver
 *
 *
 *  Copyright (C) 2020 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/mailbox_client.h>
#include "rpmsg_internal.h"

#define MAX_SHARE_MEMERY_SIZE   16*1024

#define COMMAND_RECEIVE_MSG 0x60
#define COMMAND_RECEIVE_ACK 0x61
#define COMMAND_SEND_MSG 0x80
#define COMMAND_SEND_ACK 0x81

struct ma35d1_rpmsg_priv {
	struct device dev;
	const char *name;
	struct device_node *of_node;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	wait_queue_head_t tx_ack_event;
	u32 tx_ack_flag;
	u32 rx_ack_flag;
	bool enable_v2_arch;
	unsigned char __iomem *tx_mem_start_addr;
	unsigned char __iomem *rx_mem_start_addr;
	u32 tx_smem_size;
	u32 rx_smem_size;
	struct ma35d1_rpmsg_endpoint *ma35d1_ept;
};


struct ma35d1_rpmsg_device {
	struct rpmsg_device rpdev;
	struct ma35d1_rpmsg_priv *nvt_rpmag_priv;
};

struct ma35d1_rpmsg_endpoint {
	struct rpmsg_endpoint ept;
	struct ma35d1_rpmsg_priv *ma35d1_priv;
};

#define to_ma35d1_rpmsg_device(r)     container_of(r, struct ma35d1_rpmsg_device, rpdev)
#define to_ma35d1_rpmsg_priv(d)       container_of(d, struct ma35d1_rpmsg_priv, dev)
#define to_ma35d1_rpmsg_endpoint(e)   container_of(e, struct ma35d1_rpmsg_endpoint, ept)

static const struct rpmsg_endpoint_ops ma35d1_rpmsg_endpoint_ops;

static void ma35d1_rpmsg_recv_from_remote(struct mbox_client *cl, void *msg)
{
	struct ma35d1_rpmsg_priv *priv = container_of(cl, struct ma35d1_rpmsg_priv, mbox_client);
	struct rpmsg_endpoint *ept = &priv->ma35d1_ept->ept;
	struct device *dev = priv->dev.parent;
	int ret;
	int command[4];

	u32 *data = (u32 *)msg;

	if(ept == NULL) {
		dev_err(&priv->dev, "endpoint not set !! \n");
		return;
	}

	switch (data[0]) {
		case COMMAND_RECEIVE_MSG:
			ret = ept->cb(ept->rpdev, priv->rx_mem_start_addr, data[1], ept->priv, RPMSG_ADDR_ANY);
			if (ret < 0)
				dev_err(&priv->dev, "failed to get share memery data \n");

			if (priv->enable_v2_arch != true) {
				command[0] = COMMAND_RECEIVE_ACK;
				ret = mbox_send_message(priv->mbox_chan, (void *)&command[0]);
				if (ret < 0)
					dev_err(&priv->dev, "failed to send mailbox message, status = %d\n", ret);
			}

			priv->rx_ack_flag = 1;
			wake_up_interruptible_all(&priv->tx_ack_event);
		break;

		case COMMAND_SEND_ACK:
			priv->tx_ack_flag = 1;
			wake_up_interruptible_all(&priv->tx_ack_event);
		break;

		default:
			dev_dbg(dev, "no message was found %d\n", *data);
	}
}

static void ma35d1_rpmsg_ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint, refcount);

	kfree(to_ma35d1_rpmsg_endpoint(ept));
}

static void ma35d1_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	kref_put(&ept->refcount, ma35d1_rpmsg_ept_release);
}

static int ma35d1_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct ma35d1_rpmsg_endpoint *nept = to_ma35d1_rpmsg_endpoint(ept);
	struct ma35d1_rpmsg_priv *priv = nept->ma35d1_priv;
	int ret = 0;
	int command[4];

	memcpy(priv->tx_mem_start_addr, data, len);

	command[0] = COMMAND_SEND_MSG;
	command[1] = len;

	priv->tx_ack_flag = 0;
	ret = mbox_send_message(priv->mbox_chan, (void *)&command);
	if (ret < 0)
		dev_err(&priv->dev, "failed to send mailbox message, status = %d\n", ret);

	if (priv->enable_v2_arch != true)
		return mbox_flush(priv->mbox_chan, 50);
	else
		return ret;
}

static int ma35d1_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct ma35d1_rpmsg_endpoint *nept = to_ma35d1_rpmsg_endpoint(ept);
	struct ma35d1_rpmsg_priv *priv = nept->ma35d1_priv;
	int ret = 0;
	int command[4];

	memcpy(priv->tx_mem_start_addr, data, len);

	command[0] = COMMAND_SEND_MSG;
	command[1] = len;

	priv->tx_ack_flag = 0;
	ret = mbox_send_message(priv->mbox_chan, (void *)&command);
	if (ret < 0)
		dev_err(&priv->dev, "failed to send mailbox message, status = %d\n", ret);

	if (priv->enable_v2_arch != true)
		return mbox_flush(priv->mbox_chan, 50);
	else
		return ret;
}

static __poll_t ma35d1_rpmsg_poll(struct rpmsg_endpoint *ept, struct file *filp, poll_table *wait)
{
	struct ma35d1_rpmsg_endpoint *nept = to_ma35d1_rpmsg_endpoint(ept);
	struct ma35d1_rpmsg_priv *priv = nept->ma35d1_priv;
	__poll_t mask = 0;

	poll_wait(filp, &priv->tx_ack_event, wait);

	if(priv->tx_ack_flag == 1){
		mask |= EPOLLOUT;
		priv->tx_ack_flag = 0;
	}

	if(priv->rx_ack_flag == 1){
		mask |= EPOLLIN;
		priv->rx_ack_flag = 0;
	}

	return mask;
}

static int ma35d1_rpmsg_announce_create(struct rpmsg_device *rpdev)
{
	return 0;
}

static struct rpmsg_endpoint *ma35d1_rpmsg_create_ept(struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv, struct rpmsg_channel_info chinfo)
{
	struct ma35d1_rpmsg_endpoint *nept;
	struct ma35d1_rpmsg_device *ndev = to_ma35d1_rpmsg_device(rpdev);
	struct ma35d1_rpmsg_priv *ma35d1_priv = ndev->nvt_rpmag_priv;
	struct rpmsg_endpoint *ept;

	nept = kzalloc(sizeof(*nept), GFP_KERNEL);
	if (!nept)
		return NULL;

	ept = &nept->ept;

	kref_init(&ept->refcount);

	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = &ma35d1_rpmsg_endpoint_ops;

	ma35d1_priv->ma35d1_ept = nept;
	nept->ma35d1_priv = ma35d1_priv;

	return ept;
}


static void ma35d1_rpmsg_release(struct device *dev)
{
	struct ma35d1_rpmsg_priv *priv = to_ma35d1_rpmsg_priv(dev);

	kfree(priv);
}

static void ma35d1_rpmsg_release_device(struct device *dev)
{
	struct rpmsg_device *rpdev = to_rpmsg_device(dev);
	struct ma35d1_rpmsg_device *ndev = to_ma35d1_rpmsg_device(rpdev);

	kfree(ndev);
}

static const struct rpmsg_endpoint_ops ma35d1_rpmsg_endpoint_ops = {
	.destroy_ept = ma35d1_rpmsg_destroy_ept,
	.send = ma35d1_rpmsg_send,
	.trysend = ma35d1_rpmsg_trysend,
	.poll = ma35d1_rpmsg_poll,
};

static const struct rpmsg_device_ops ma35d1_rpmsg_device_ops = {
	.create_ept = ma35d1_rpmsg_create_ept,
	.announce_create = ma35d1_rpmsg_announce_create,
};

static void ma35d1_rpmsg_mbox_set(struct ma35d1_rpmsg_priv *priv)
{
	struct device *dev = priv->dev.parent;

	priv->mbox_client.dev            = dev;
	priv->mbox_client.rx_callback    = ma35d1_rpmsg_recv_from_remote;
	priv->mbox_client.tx_done        = NULL;
	priv->mbox_client.tx_block       = false;
	priv->mbox_client.knows_txdone   = false;
	if (priv->enable_v2_arch != true)
		priv->mbox_client.tx_tout    = 10;
	else
		priv->mbox_client.tx_tout    = 1;

	priv->mbox_chan = mbox_request_channel(&priv->mbox_client, 0);
	if (IS_ERR(priv->mbox_chan)) {
		dev_err(dev, "mbox_request_channel failed: %ld\n", PTR_ERR(priv->mbox_chan));
	}
}


struct ma35d1_rpmsg_priv *ma35d1_rpmsg_register(struct device *parent, struct device_node *node)
{
	struct ma35d1_rpmsg_priv *priv;
	struct ma35d1_rpmsg_device *ndev;
	struct device_node *node1;
	struct resource r;
	int ret;
	u32  share_mem_addr, share_mem_size, tx_share_size, rx_share_size;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_bool(node, "rpmsg-ddr-buf")) {
		node1 = of_parse_phandle(node, "memory-region", 0);
		if (node) {
			ret = of_address_to_resource(node1, 0, &r);
			if (ret == 0) {
				share_mem_addr = r.start;
				share_mem_size  = resource_size(&r);
				priv->tx_mem_start_addr = memremap(share_mem_addr, share_mem_size, MEMREMAP_WT);
				priv->rx_mem_start_addr = priv->tx_mem_start_addr + share_mem_size/2;
			} else {
				dev_err(&priv->dev, "fail to parse share memory region!\n");
				return ERR_PTR(ret);
			}
		}
		priv->enable_v2_arch = true;
	} else {
		priv->enable_v2_arch = of_property_read_bool(node, "rpmsg-v2-arch");

		if (of_property_read_u32_array(node, "share-mem-addr", &share_mem_addr, 1) != 0) {
			return ERR_PTR(-EINVAL);
		}
		if (of_property_read_u32_array(node, "tx-smem-size", &tx_share_size, 1) != 0) {
			return ERR_PTR(-EINVAL);
		}
		if (of_property_read_u32_array(node, "rx-smem-size", &rx_share_size, 1) != 0) {
			return ERR_PTR(-EINVAL);
		}

		if((tx_share_size + rx_share_size) > MAX_SHARE_MEMERY_SIZE){
			dev_err(&priv->dev, "share memery set error!\n");
			return ERR_PTR(-EINVAL);
		}

		priv->tx_mem_start_addr = ioremap(share_mem_addr, tx_share_size);
		priv->rx_mem_start_addr = ioremap((share_mem_addr + tx_share_size), rx_share_size);
		priv->tx_smem_size = tx_share_size;
		priv->rx_smem_size = rx_share_size;
	}

	priv->tx_ack_flag = 0;
	priv->rx_ack_flag = 0;
	priv->dev.parent = parent;
	priv->dev.release = ma35d1_rpmsg_release;
	priv->dev.of_node = node;

	dev_set_name(&priv->dev, "%s:%pOFn", dev_name(parent), node);

	init_waitqueue_head(&priv->tx_ack_event);

	ma35d1_rpmsg_mbox_set(priv);

	ret = device_register(&priv->dev);
	if (ret) {
		pr_err("failed to register rpmsg\n");
		put_device(&priv->dev);
		return ERR_PTR(ret);
	}

	ndev = kzalloc(sizeof(*ndev), GFP_KERNEL);
	if (!ndev)
		return ERR_PTR(-ENOMEM);

	ndev->nvt_rpmag_priv = priv;
	ndev->rpdev.ops = &ma35d1_rpmsg_device_ops;
	ndev->rpdev.dev.parent = &priv->dev;
	ndev->rpdev.dev.release = ma35d1_rpmsg_release_device;

	ret = rpmsg_chrdev_register_device(&ndev->rpdev);
	if (ret) {
		dev_err(&priv->dev, "failed to register rpmsg chrdev \n");
		goto unregister_dev;
	}

	return priv;

unregister_dev:
	if (!IS_ERR_OR_NULL(priv->mbox_chan))
		mbox_free_channel(priv->mbox_chan);

	device_unregister(&priv->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(ma35d1_rpmsg_register);


static int ma35d1_rpmsg_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;

	ma35d1_rpmsg_register(&pdev->dev, node);

	return 0;
}

static int ma35d1_rpmsg_remove_device(struct device *dev, void *data)
{
	device_unregister(dev);

	return 0;
}

static int ma35d1_rpmsg_remove_priv(struct device *dev, void *data)
{
	struct ma35d1_rpmsg_priv *priv = to_ma35d1_rpmsg_priv(dev);
	int ret;

	ret = device_for_each_child(&priv->dev, NULL, ma35d1_rpmsg_remove_device);
	if (ret)
		dev_warn(&priv->dev, "can't remove rpmsg device: %d\n", ret);

	mbox_free_channel(priv->mbox_chan);
	device_unregister(&priv->dev);

	return 0;
}

static int ma35d1_rpmsg_remove(struct platform_device *pdev)
{
	int ret;

	ret = device_for_each_child(&pdev->dev, NULL, ma35d1_rpmsg_remove_priv);
	if (ret)
		dev_warn(&pdev->dev, "can't remove rpmsg device: %d\n", ret);

	return ret;
}

static const struct of_device_id ma35d1_rpmsg_of_match[] = {
	{ .compatible = "nuvoton,ma35d1-rpmsg" },
	{}
};
MODULE_DEVICE_TABLE(of, ma35d1_rpmsg_of_match);

static struct platform_driver ma35d1_rpmsg_driver = {
	.probe = ma35d1_rpmsg_probe,
	.remove = ma35d1_rpmsg_remove,
	.driver = {
		.name = "ma35d1-rpmsg",
		.of_match_table = ma35d1_rpmsg_of_match,
	},
};
module_platform_driver(ma35d1_rpmsg_driver);

static void __exit ma35d1_rpmsg_dev_exit(void)
{
	platform_driver_unregister(&ma35d1_rpmsg_driver);
}
module_exit(ma35d1_rpmsg_dev_exit);


MODULE_AUTHOR("ma35d1, Inc.");
MODULE_DESCRIPTION("ma35d1");
MODULE_LICENSE("GPL v2");




