/*
 *  linux/drivers/rpmsg/ma35_amp.c
 *
 *  MA35 series AMP driver
 *
 *  Copyright (C) 2024 Nuvoton Technology Corp.
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
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/remoteproc.h>
#include "rpmsg_internal.h"

struct fw_rsc_vdev_ext {
	u32 type;
	struct fw_rsc_vdev vdev;
} __packed;

struct rsc_table_desc {
	u8  CMD;
	u8  STS;
	u16 len;
	u32 buf_offset;       // offset from tx or rx base
	u32 nxt_offset;       // offset from shmem_base
};

struct bdtask {
	struct list_head list;
	struct rpmsg_endpoint *ept;
};
static LIST_HEAD(bdtask_queue);

struct ma35_rpmsg_priv {
	struct device dev;
	struct device_node *node;
	/* Unused, reserved for IPI */
	const char *kick_dev_name;
	const char *kick_dev_bus_name;
	struct metal_device *kick_dev; // link to kick_device
	struct metal_io_region *kick_io; // metal_io_region of above

	/* Tx IPI */
	u32 ipi_txch;
	void __iomem *ipi_txbase;
	/* Rx IPI */
	u32 ipi_rxch;
	void __iomem *ipi_rxbase;
	int irq_num;

	// general
	int ready; // check remote driver is ready, aka first frame
	spinlock_t lock;
	struct task_struct *bdthread;
	wait_queue_head_t bd_event;
	struct delayed_work dwork;
	u32 resumeflow;

	// desc
	u32 desc_num;
	u32 desc_size;
	u32 ns_offset;
	u32 ns_size;
	u32 desc_rxbuf; // size of buf managed by 1 desc
	u32 desc_txbuf;

	// shmem
	unsigned char __iomem *shmem_base;
	unsigned char __iomem *desc_vring0; // 0 is rx for this endpoint
	unsigned char __iomem *desc_vring1; // 1 is tx for this endpoint
	unsigned char __iomem *shmem_rx_base;
	unsigned char __iomem *shmem_tx_base;
	u32 shmem_size; // local provided
	u32 shmem_rx_size;
	u32 shmem_tx_size;
	u32 vring0_offset;
	u32 vring1_offset;

	// tx
	uint8_t *buf_flag; // manage tx buffer usage
	// rx
	void   **kick_ept; // save pointer to each rx ept by id

};

struct ma35_rpmsg_ept_priv {
	struct rpmsg_endpoint *ept_parent; // save its parent
	int ept_type;
	struct bdtask *task; // memory its bdtask
	unsigned int available_len;
	spinlock_t lock; // an ept can be either tx or rx
	// tx
	unsigned int no_desc;
	unsigned int id; // desc id of the following
	void *pDesc; // pointer to tx desc link
	int remote_binded; // can be used to check remote is binded, send before check
	// rx
	void *bind_desc; // pointer to rx desc, can be used to check remote is created, scan until bind
	unsigned int bind_id; // index of kickept
	int /*atomic_t*/ kicked;
	wait_queue_head_t rx_event;
	uint8_t cmd; // save CMD
	void *rxns; // save rxns for passing
	struct task_struct *csthread;
	wait_queue_head_t cs_event;
};

struct ma35_rpmsg_device {
	struct rpmsg_device rpdev;
	struct ma35_rpmsg_priv *rpmsg_priv;
};

struct ma35_rpmsg_endpoint {
	struct rpmsg_endpoint ept;
	struct ma35_rpmsg_priv *rpmsg_priv;
	void *eptdev; // save pointer form char rpmsg endpoint
	struct sk_buff_head *rxqueue;
};

#define to_ma35_rpmsg_device(r)     container_of(r, struct ma35_rpmsg_device, rpdev)
#define to_ma35_rpmsg_endpoint(r)   container_of(r, struct ma35_rpmsg_endpoint, ept)
#define to_ma35_rpmsg_priv(r)       container_of(r, struct ma35_rpmsg_priv, dev)

#define rsc_table_rxdesc(priv, index)   ((struct rsc_table_desc *)(priv->desc_vring0 + priv->desc_size * index))
#define rsc_table_rxns(priv, index)     ((char *)(priv->desc_vring0 + priv->ns_offset + priv->desc_size * index))
#define rsc_table_txdesc(priv, index)   ((struct rsc_table_desc *)(priv->desc_vring1 + priv->desc_size * index))
#define rsc_table_txns(priv, index)     ((char *)(priv->desc_vring1 + priv->ns_offset + priv->desc_size * index))
#define shmem_rx_buf(priv, index)       ((unsigned char *)(priv->shmem_rx_base + priv->desc_rxbuf * index))
#define shmem_tx_buf(priv, index)       ((unsigned char *)(priv->shmem_tx_base + priv->desc_txbuf * index))

#define VRING_DESC_CMD_HEAD    0x1 // write
#define VRING_DESC_CMD_RELOAD  0x2
#define VRING_DESC_CMD_CLAIM   0x10 // scan, no need notification
#define VRING_DESC_CMD_CLOSE   0x20 // remote close it

#define VRING_DESC_STS_READING 0x1
#define VRING_DESC_STS_RELOAD  0x2
#define VRING_DESC_STS_ACK     0x4
#define VRING_DESC_STS_UNKNOWN 0x8
#define VRING_DESC_STS_BIND    0x10
#define VRING_DESC_STS_CLOSE   0x20
#define VRING_DESC_STS_ERR     0x80

#define DRIVER_VERSION         1
#define VIRTIO_ID_RPMSG_       7
#define NO_RESOURCE_ENTRIES    8

#define IPI_CMD_REQUEST        0x10
#define IPI_CMD_REPLY          0x11
#define IPI_CMD_MODIFY         0x20
#define IPI_CMD_UPDATE         0x22

#define IPI_REG_CTL            0x0
#define IPI_REG_CMP            0x4
#define IPI_REG_INTSTS         0x8

#define IPI_CTL_INTEN          (0x1 << 29)
#define IPI_CTL_CNTEN          (0x1 << 30)
#define IPI_INTSTS_TIF         (0x1 << 0)

#define EPT_TYPE_TX            0x01
#define EPT_TYPE_RX            0x10

static const struct rpmsg_endpoint_ops ma35_rpmsg_endpoint_ops;

static int ma35_rsc_table_parser(struct ma35_rpmsg_priv *priv, struct device_node *node);
static int check_rx_bind_ready(struct ma35_rpmsg_ept_priv *ept_priv);
static int ma35_rpmsg_receive(struct rpmsg_endpoint *ept, void *buf, int *len);
static u8 ma35_rpmsg_retrieve_status(struct rpmsg_endpoint *ept);
static int ma35_rpmsg_receive_status(struct rpmsg_endpoint *ept, u8 sts);
static int ma35_rpmsg_reconnect_ept(struct rpmsg_endpoint *ept);

irqreturn_t ma35_rpmsg_ipi_isr(int irq, void *data)
{
	struct ma35_rpmsg_priv *rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct resource_table *rsc_table;
	struct rsc_table_desc *desc;
	int i;
	u32 flag, len;
	unsigned long flags;

	rpmsg_priv = data;
	rsc_table = (struct resource_table *)rpmsg_priv->shmem_base;

	// check CMD
	if (rsc_table->reserved[0] == IPI_CMD_REPLY) {
		ma35_rsc_table_parser(rpmsg_priv, NULL);
		flag = readl_relaxed(rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS) & IPI_INTSTS_TIF;
		if(flag)
			writel_relaxed(flag, rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS);
		return IRQ_HANDLED;
	}

	// scan desc ch by ch
	for (i = 0; i < rpmsg_priv->desc_num; i++)
	{
		if(rsc_table_rxdesc(rpmsg_priv, i)->CMD & (VRING_DESC_CMD_HEAD | VRING_DESC_CMD_CLOSE))
		{
			ept_priv = (struct ma35_rpmsg_ept_priv *)rpmsg_priv->kick_ept[i];
			if(!check_rx_bind_ready(ept_priv))
			{
				desc = ept_priv->bind_desc;
				ept_priv->cmd = desc->CMD;
				desc->CMD = 0;
				desc->STS = VRING_DESC_STS_READING;
				if(ept_priv->cmd & VRING_DESC_CMD_HEAD) {
					// enqueue to skb
					if(ma35_rpmsg_receive(ept_priv->ept_parent, NULL, &len))
						continue;

				} else if(ept_priv->cmd & VRING_DESC_CMD_CLOSE) {
					ma35_rpmsg_reconnect_ept(ept_priv->ept_parent);
				}
				// trigger ept
				spin_lock_irqsave(&ept_priv->lock, flags);
				ept_priv->kicked = 1;
				spin_unlock_irqrestore(&ept_priv->lock, flags);

				wake_up_interruptible_all(&ept_priv->rx_event);
			}
		}
	}

	flag = readl_relaxed(rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS) & IPI_INTSTS_TIF;

	if(flag)
		writel_relaxed(flag, rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS);

	return IRQ_HANDLED;
}

/* send irq to remote */
static int ma35_rpmsg_notify_remote(struct ma35_rpmsg_priv *rpmsg_priv)
{
	writel_relaxed((IPI_CTL_INTEN | IPI_CTL_CNTEN), rpmsg_priv->ipi_txbase + IPI_REG_CTL);

	return 0;
}

static int ma35_rpmsg_ns_bind_remote(struct rpmsg_endpoint *ept, char *ns, bool wait)
{
	// poll for desc to search ns
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rpmsg_device *rpdev;
	struct rsc_table_desc *rxdesc;
	unsigned long flags;
	int i, ret = 0;

	if(!ept || !ns)
		return -1;

	ept_priv = ept->priv;
	rpdev = ept->rpdev;

NS_MATCHING:
	for(i = 0; i < rpmsg_priv->desc_num; i++)
	{
		if((rsc_table_rxdesc(rpmsg_priv, i)->CMD & VRING_DESC_CMD_CLAIM) == 0)
			continue;
		if(!memcmp(ns, rsc_table_rxns(rpmsg_priv, i), rpmsg_priv->ns_size))
		{
			spin_lock_irqsave(&rpmsg_priv->lock, flags);

			ept_priv->bind_desc = rsc_table_rxdesc(rpmsg_priv, i);
			ept_priv->bind_id = i;
			rpmsg_priv->kick_ept[ept_priv->bind_id] = ept_priv;

			rxdesc = ept_priv->bind_desc;
			rxdesc->STS = VRING_DESC_STS_BIND;
			while(rxdesc)
			{
				ret += rpmsg_priv->desc_rxbuf;
				rxdesc = (rxdesc->nxt_offset == 0) ? NULL : (struct rsc_table_desc *)(rpmsg_priv->shmem_base + rxdesc->nxt_offset);
			}

			spin_unlock_irqrestore(&rpmsg_priv->lock, flags);

			// bind success, print something
			dev_info(ma35_rpept->eptdev, "Binded with remote ept \"%s\" with size %d.\n", ns, ret);

			return ret;
		}
	}

	if (wait)
		goto NS_MATCHING;

	return -1; // no match
}

static int check_tx_bind_ready(struct ma35_rpmsg_ept_priv *ept_priv)
{
	struct rsc_table_desc *desc;

	if(!ept_priv->remote_binded)
		return -EACCES;

	desc = ept_priv->pDesc;

	if(desc->STS & (VRING_DESC_STS_READING | VRING_DESC_STS_ERR))
		return -EAGAIN;
	else if(desc->STS == VRING_DESC_STS_CLOSE)
	{
		ma35_rpmsg_reconnect_ept(ept_priv->ept_parent);

		return -EPERM;
	}

	return 0;
}

static int check_rx_bind_ready(struct ma35_rpmsg_ept_priv *ept_priv)
{
	if(!ept_priv)
		return -1;

	if(!ept_priv->bind_desc)
		return -1;

	return 0;
}

static int ma35_kill_ns_bind(struct rpmsg_endpoint *ept)
{
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct rsc_table_desc *rxdesc;

	ept_priv = ept->priv;
	if(check_rx_bind_ready(ept_priv) < 0)
		return -1;

	rxdesc = ept_priv->bind_desc;
	ept_priv->bind_desc = NULL;
	rpmsg_priv->kick_ept[ept_priv->bind_id] = NULL;
	ept_priv->bind_id = 0;

	rxdesc->STS = VRING_DESC_STS_CLOSE;

	return 0;
}

static int ma35_check_available_desc(struct rpmsg_endpoint *ept, int len)
{
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	int i, req, avail, id;
	unsigned long flags;
	struct rsc_table_desc head;
	struct rsc_table_desc *desc = &head;

	spin_lock_irqsave(&rpmsg_priv->lock, flags);

	for (i = 0, avail = 0; i < rpmsg_priv->desc_num; i++)
	{
		if(!rpmsg_priv->buf_flag[i])
			avail++;
	}

	req = len / rpmsg_priv->desc_txbuf;
	if((len % rpmsg_priv->desc_txbuf) != 0)
		req++;

	if(req > avail || len == 0)
		goto err; /* buffer insufficient */

	ept_priv->no_desc = req;
	ept_priv->available_len = len;

	id = -1;
	for (i = 0; (i < rpmsg_priv->desc_num) && (req > 0); i++)
	{
		if(!rpmsg_priv->buf_flag[i])
		{
			rpmsg_priv->buf_flag[i] = 1;
			desc->nxt_offset = rpmsg_priv->vring1_offset + rpmsg_priv->desc_size * i;
			desc = (struct rsc_table_desc *)(rpmsg_priv->shmem_base + desc->nxt_offset);
			req--;
			if(id < 0)
				id = i;
		}
	}
	if(id < 0)
		goto err;

	ept_priv->id = id;
	ept_priv->pDesc = (void *)(rpmsg_priv->shmem_base + head.nxt_offset);

	spin_unlock_irqrestore(&rpmsg_priv->lock, flags);

	return len;
err:
	spin_unlock_irqrestore(&rpmsg_priv->lock, flags);
	dev_err(&rpmsg_priv->dev, "No buffer available in shared memory pool.\n");
	return -1;
}

static int ma35_request_tx_desc(struct rpmsg_endpoint *ept, struct rpmsg_channel_info chinfo)
{
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct rsc_table_desc *desc;
	int ret, dst;

	ept_priv = ept->priv;
	dst = chinfo.dst;
	ret = ma35_check_available_desc(ept, dst);
	if (ret <= 0)
		return -1;

	// desc get, waiting to be binded by remote
	desc = ept_priv->pDesc;
	desc->CMD = VRING_DESC_CMD_CLAIM;
	desc->STS = 0;
	strncpy((char *)desc + rpmsg_priv->ns_offset, chinfo.name, rpmsg_priv->ns_size);

	return ret;
}

static int ma35_rpmsg_release_shmem(struct rpmsg_endpoint *ept)
{
	u32 i, offset;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc, *next;
	unsigned long flags;

	if(!ept || !ept_priv)
		return -1;

	desc = ept_priv->pDesc;

	spin_lock_irqsave(&ept_priv->lock, flags);

	while(desc)
	{
		next = (struct rsc_table_desc *)(rpmsg_priv->shmem_base + desc->nxt_offset);
		i = desc->buf_offset/rpmsg_priv->desc_txbuf;
		rpmsg_priv->buf_flag[i] = 0;
		offset = desc->nxt_offset;
		// resume
		memset(&desc->STS, 0, sizeof(struct rsc_table_desc) - 1 + rpmsg_priv->ns_size);
		desc->buf_offset = rpmsg_priv->desc_txbuf * i;
		desc = (offset == 0) ? NULL : next;
	}

	spin_unlock_irqrestore(&ept_priv->lock, flags);

	return 0;
}

static int ma35_rpmsg_bind_process(void *data)
{
	struct rpmsg_endpoint *ept = (struct rpmsg_endpoint *)data;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct rsc_table_desc *desc;
	int buf_size;

	if(!ept)
		return 0;

	ept_priv = ept->priv;

	if(ept_priv->ept_type == EPT_TYPE_TX) {
		// just check STS
		desc = ept_priv->pDesc;
		if(desc->STS & VRING_DESC_STS_BIND) {
			ept_priv->remote_binded = 1;
			ept_priv->task = NULL;
			dev_info(ma35_rpept->eptdev, "Remote binded to \"%s\".\n", (char *)desc + sizeof(struct rsc_table_desc));
			//break;
			return 1;
		}
	}
	else { // EPT_TYPE_RX
		// scan for rxns
		buf_size = ma35_rpmsg_ns_bind_remote(ept, ept_priv->rxns, false);
		if(buf_size > 0) {
			ept_priv->available_len = (unsigned int)buf_size;
			ept_priv->task = NULL;

			return 2;
		}
	}

	return 0;
}

static struct bdtask *ma35_enqueue_bdtask(struct rpmsg_endpoint *ept) {
    struct bdtask *task;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;

    task = kmalloc(sizeof(struct bdtask), GFP_KERNEL);
    if (!task) {
        return NULL;
    }

    task->ept = ept;
    INIT_LIST_HEAD(&task->list);
    list_add_tail(&task->list, &bdtask_queue);

	wake_up_interruptible(&rpmsg_priv->bd_event);

	return task;
}

static int ma35_binding_thread(void *data) {
    struct bdtask *task, *tmp;
	struct ma35_rpmsg_priv *rpmsg_priv = data;
	(void)data;

    while (!kthread_should_stop()) {
		if (list_empty(&bdtask_queue)) {
			wait_event_interruptible(rpmsg_priv->bd_event, !list_empty(&bdtask_queue) || kthread_should_stop());
			if(kthread_should_stop())
				break;
		}
		else {
			list_for_each_entry_safe(task, tmp, &bdtask_queue, list) {
				if (ma35_rpmsg_bind_process(task->ept)) {
					list_del(&task->list);
					kfree(task);
				}
			}
			yield();
		}
    }

    return 0;
}

static int ma35_checksts_thread(void *data) {
	struct rpmsg_endpoint *ept = (struct rpmsg_endpoint *)data;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;

	while (!kthread_should_stop()) {
		if (ma35_rpmsg_retrieve_status(ept) != VRING_DESC_STS_ERR) {
			wait_event_interruptible(ept_priv->cs_event,
				(ma35_rpmsg_retrieve_status(ept) == VRING_DESC_STS_ERR) || kthread_should_stop());
			if(kthread_should_stop())
				break;
		}
		else {
			if(skb_queue_empty(ma35_rpept->rxqueue))
				ma35_rpmsg_receive_status(ept, VRING_DESC_STS_ACK);
			yield();
		}
	}

	return 0;
}

static int ma35_rpmsg_reconnect_ept(struct rpmsg_endpoint *ept)
{
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc;

	if(ept_priv->task)
		return -1;

	if(ept_priv->ept_type == EPT_TYPE_TX)
	{
		// shared memory not released
		ept_priv->remote_binded = 0;
		desc = ept_priv->pDesc;
		desc->STS = 0;
		desc->CMD = VRING_DESC_CMD_CLAIM;
	}
	else // EPT_TYPE_RX
	{
		ma35_kill_ns_bind(ept);
	}

	ept_priv->task = ma35_enqueue_bdtask(ept);

	return 0;
}

static struct rpmsg_endpoint *ma35_rpmsg_create_ept(struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv, struct rpmsg_channel_info chinfo)
{
	struct ma35_rpmsg_device *ma35_rpdev = to_ma35_rpmsg_device(rpdev);
	struct ma35_rpmsg_priv *rpmsg_priv;
	struct ma35_rpmsg_endpoint *ma35_rpept;
	struct rpmsg_endpoint *ept;
	struct ma35_rpmsg_ept_priv *ept_priv;

	rpmsg_priv = ma35_rpdev->rpmsg_priv;

	if (!rpmsg_priv->ready || rpmsg_priv->resumeflow) {
		dev_err(&rpmsg_priv->dev, "Parse remote failed.\n");
		return NULL;
	}

	ma35_rpept = kzalloc(sizeof(*ma35_rpept), GFP_KERNEL);
	if (!ma35_rpept)
		goto free_ma35_rpept;

	ept_priv = kzalloc(sizeof(*ept_priv), GFP_KERNEL);
	if (!ept_priv)
		goto free_ept_priv;

	ma35_rpept->rpmsg_priv = rpmsg_priv;
	ma35_rpept->eptdev = priv;
	ma35_rpept->rxqueue = (struct sk_buff_head *)((char *)ma35_rpept->eptdev + qtoeptdev);
	ept = &ma35_rpept->ept;
	ept->priv = ept_priv;
	ept_priv->ept_parent = ept;
	spin_lock_init(&ept_priv->lock);

	if(chinfo.src == EPT_TYPE_TX) {
		// check for available shmem
		ept_priv->ept_type = EPT_TYPE_TX;
		if (ma35_request_tx_desc(ept, chinfo) < 0)
			goto free_ept_priv;
	} else if(chinfo.src == EPT_TYPE_RX) {
		// check ns from remote
		ept_priv->ept_type = EPT_TYPE_RX;
		ept_priv->rxns = kzalloc(sizeof(char) * RPMSG_NAME_SIZE, GFP_KERNEL);
		strncpy(ept_priv->rxns, chinfo.name, rpmsg_priv->ns_size);
		init_waitqueue_head(&ept_priv->rx_event); // place here to prevent poll fail
		init_waitqueue_head(&ept_priv->cs_event);
		ept_priv->csthread = kthread_run(ma35_checksts_thread, ept, chinfo.name);
	} else {
		goto free_ept_priv;
	}

	kref_init(&ept->refcount);
	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->ops = &ma35_rpmsg_endpoint_ops;

	/* Create one if no thread */
	if(!rpmsg_priv->bdthread) {
		init_waitqueue_head(&rpmsg_priv->bd_event);
		rpmsg_priv->bdthread = kthread_run(ma35_binding_thread, rpmsg_priv, "binding_thread");
		dev_dbg(&rpmsg_priv->dev, "Binding thread is created.\n");
		if(!rpmsg_priv->bdthread)
			goto free_ept_priv;
	}

	ept_priv->task = ma35_enqueue_bdtask(ept);
	if(!ept_priv->task)
		goto free_ept_priv;

	return ept;

free_ept_priv:
	kfree(ept_priv);

free_ma35_rpept:
	kfree(ma35_rpept);
	return NULL;
}

static void ma35_rpmsg_ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint, refcount);
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;

	if(ept_priv)
		kfree(ept_priv);

	kfree(to_ma35_rpmsg_endpoint(ept));
}

static void ma35_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;

	// tell remote ept close
	if(ept_priv->ept_type == EPT_TYPE_TX) {
		if(ept_priv->remote_binded) {
			ept_priv->remote_binded = 0;
			desc = ept_priv->pDesc;
			desc->CMD = VRING_DESC_CMD_CLOSE;
			ma35_rpmsg_notify_remote(rpmsg_priv);
		}
		// no need to wait, just leave the CMD
		ma35_rpmsg_release_shmem(ept);
	} else { // EPT_TYPE_RX
		if(ept_priv->bind_desc) {
			desc = ept_priv->bind_desc;
			if(desc) // NULL: no bind or already get close cmd
				desc->STS = VRING_DESC_STS_CLOSE;
		}
		if(ept_priv->csthread) {
			kthread_stop(ept_priv->csthread);
			wake_up_interruptible(&ept_priv->cs_event);
		}
		// trace back to unregister ept from device
		disable_irq(rpmsg_priv->irq_num);
		synchronize_irq(rpmsg_priv->irq_num);
		ept_priv->pDesc = ept_priv->bind_desc = NULL;
		rpmsg_priv->kick_ept[ept_priv->bind_id] = NULL;
		enable_irq(rpmsg_priv->irq_num);
		kfree(ept_priv->rxns);
	}

	// kill binding task
	if(ept_priv->task) {
		list_del(&ept_priv->task->list);
		kfree(ept_priv->task);
	}

	kref_put(&ept->refcount, ma35_rpmsg_ept_release);
}

int ma35_rpmsg_desc_reset(struct rpmsg_endpoint *ept)
{
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv;

	desc = ept_priv->pDesc;
	rpmsg_priv = ma35_rpept->rpmsg_priv;

	while(desc)
	{
		desc->CMD = desc->STS = desc->len = 0;
		desc = (desc->nxt_offset == 0) ? NULL : (struct rsc_table_desc *)(rpmsg_priv->shmem_base + desc->nxt_offset);
	}

	return 0;
}

static int ma35_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv;
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct rsc_table_desc *desc;
	unsigned long flags;
	int i, res;

	if(!ept || !ept->priv || !data || len < 0)
		return -EINVAL; // RPMSG_ERR_PARAM

	ept_priv = ept->priv;
	if(len > ept_priv->available_len)
		return -ENOMEM; // RPMSG_ERR_NO_MEM

	if(ept_priv->ept_type != EPT_TYPE_TX)
		return -EACCES;

	desc = ept_priv->pDesc;
	if(!desc)
		return -EBUSY; // RPMSG_ERR_NO_BUFF

	res = check_tx_bind_ready(ept_priv);
	if(res)
		return res;

	spin_lock_irqsave(&ept_priv->lock, flags);

	ma35_rpmsg_desc_reset(ept);

	rpmsg_priv = ma35_rpept->rpmsg_priv;

	for (i = 0, res = len; i < len; i += rpmsg_priv->desc_txbuf, res -= rpmsg_priv->desc_txbuf)
	{
		if(res <= rpmsg_priv->desc_txbuf)
		{
			memcpy((void *)(rpmsg_priv->shmem_tx_base + desc->buf_offset), (void *)((u8 *)data + i), res);
			desc->len = res;
		}
		else
		{
			memcpy((void *)(rpmsg_priv->shmem_tx_base + desc->buf_offset), (void *)((u8 *)data + i), rpmsg_priv->desc_txbuf);
			desc->len = rpmsg_priv->desc_txbuf;
			desc = (struct rsc_table_desc *)(rpmsg_priv->shmem_base + desc->nxt_offset);
		}
	}
	((struct rsc_table_desc *)ept_priv->pDesc)->CMD = VRING_DESC_CMD_HEAD;

	ma35_rpmsg_notify_remote(rpmsg_priv);

	spin_unlock_irqrestore(&ept_priv->lock, flags);

	return len;
}

static int ma35_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	return ma35_rpmsg_send(ept, data, len);
}

static u8 ma35_rpmsg_retrieve_status(struct rpmsg_endpoint *ept)
{
	struct ma35_rpmsg_ept_priv *ept_priv;
	struct rsc_table_desc *desc;

	if(!ept)
		return -1;

	ept_priv = ept->priv;

	if(!ept_priv)
		return -1;

	desc = ept_priv->bind_desc;

	if(!desc)
		return -1;

	return desc->STS;
}

static int ma35_rpmsg_receive_status(struct rpmsg_endpoint *ept, u8 sts)
{
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc;

	if(!ept)
		return -1;

	desc = ept_priv->bind_desc;
	desc->STS = sts;

	// If err, trigger check thread
	if(sts == VRING_DESC_STS_ERR)
		wake_up_interruptible(&ept_priv->cs_event);

	return 0;
}

static int ma35_rpmsg_receive(struct rpmsg_endpoint *ept, void *buf, int *len)
{
	struct ma35_rpmsg_ept_priv *ept_priv = ept->priv;
	struct rsc_table_desc *desc;
	struct ma35_rpmsg_endpoint *ma35_rpept = to_ma35_rpmsg_endpoint(ept);
	struct ma35_rpmsg_priv *rpmsg_priv = ma35_rpept->rpmsg_priv;
	int rxlen;

	// flow control on skb queue
	if(!skb_queue_empty(ma35_rpept->rxqueue)) {
		ma35_rpmsg_receive_status(ept, VRING_DESC_STS_ERR);
		return -ENOMEM;
	}

	desc = (struct rsc_table_desc *)ept_priv->bind_desc;
	rxlen = 0;
	while(desc && desc->len)
	{
		// enqueue via ept, appl read this recursively
		if(ept->cb(ept->rpdev, (void *)(rpmsg_priv->shmem_rx_base + desc->buf_offset), desc->len, ma35_rpept->eptdev, 0)) {
			ma35_rpmsg_receive_status(ept, VRING_DESC_STS_ERR);
			return -ENOMEM;
		}
		rxlen += desc->len;
		desc = (desc->nxt_offset == 0) ? NULL : (struct rsc_table_desc *)(rpmsg_priv->shmem_base + desc->nxt_offset);
	}
	*len = rxlen;

	ma35_rpmsg_receive_status(ept, VRING_DESC_STS_ACK);

	return 0;
}

static __poll_t ma35_rpmsg_poll(struct rpmsg_endpoint *ept, struct file *filp, poll_table *wait)
{
	struct ma35_rpmsg_ept_priv *ept_priv;
	__poll_t mask = 0;
	unsigned long flags;

	if(!ept)
		return -1;

	ept_priv = ept->priv;

	poll_wait(filp, &ept_priv->rx_event, wait);

	spin_lock_irqsave(&ept_priv->lock, flags);
	if(ept_priv->kicked)
	{
		if(ept_priv->cmd & VRING_DESC_CMD_HEAD)	{
			mask |= EPOLLIN;
		} else if(ept_priv->cmd & VRING_DESC_CMD_CLOSE) {
			mask |= EPOLLHUP; // remote shut down
		}

		ept_priv->kicked = 0; // interrupt is okay
	}
	spin_unlock_irqrestore(&ept_priv->lock, flags);

	return mask;
}

static void ma35_rpmsg_release(struct device *dev)
{
	struct ma35_rpmsg_priv *rpmsg_priv = to_ma35_rpmsg_priv(dev);

	kfree(rpmsg_priv);
}

static void ma35_rpmsg_release_device(struct device *dev)
{
	struct rpmsg_device *rpdev = to_rpmsg_device(dev);
	struct ma35_rpmsg_device *ma35_rpdev = to_ma35_rpmsg_device(rpdev);

	kfree(ma35_rpdev);
}

static const struct rpmsg_endpoint_ops ma35_rpmsg_endpoint_ops = {
	.destroy_ept = ma35_rpmsg_destroy_ept,
	.send = ma35_rpmsg_send,
	.trysend = ma35_rpmsg_trysend,
	.poll = ma35_rpmsg_poll,
};

static const struct rpmsg_device_ops ma35_rpmsg_device_ops = {
	.create_ept = ma35_rpmsg_create_ept,
};

static int ma35_desc_init(struct ma35_rpmsg_priv *priv)
{
	unsigned char *desc = priv->desc_vring1;
	u32 i;

	memset(desc, 0, priv->desc_num * priv->desc_size);
	for (i = 0; i < priv->desc_num; i++)
	{
		((struct rsc_table_desc *)(desc + i * priv->desc_size))->buf_offset = i * priv->desc_txbuf;
	}

	return 0;
}

static int ma35_rsc_table_update(struct ma35_rpmsg_priv *priv)
{
	// Reserved, add configuration here
	return 0;
}

static int ma35_rsc_table_parser(struct ma35_rpmsg_priv *priv, struct device_node *node)
{
	struct resource_table *rsc_table;
	struct fw_rsc_vdev_ext *rsc_vdev_ext;
	struct fw_rsc_vdev *rsc_vdev;
	struct fw_rsc_vdev_vring *vring0, *vring1;

	if(priv->ready)
		goto resu;

	rsc_table = (struct resource_table *)priv->shmem_base;
	if(rsc_table->ver != DRIVER_VERSION || rsc_table->num != 1) {
		dev_err(&priv->dev, "Nuvoton AMP version not match %d\n\n", rsc_table->ver);
		goto err;
	}

	rsc_vdev_ext = (struct fw_rsc_vdev_ext *)(priv->shmem_base + rsc_table->offset[0]);
	rsc_vdev = &rsc_vdev_ext->vdev;
	if(rsc_vdev->id != VIRTIO_ID_RPMSG_ || rsc_vdev->num_of_vrings != 2) {
		dev_err(&priv->dev, "vdev: %d %d\n", rsc_vdev->id, rsc_vdev->num_of_vrings);
		goto err;
	}
	vring0 = (struct fw_rsc_vdev_vring *)(priv->shmem_base + rsc_table->offset[1]);
	vring1 = (struct fw_rsc_vdev_vring *)(priv->shmem_base + rsc_table->offset[2]);
	if(vring0->notifyid != 1 || vring1->notifyid != 2) {
		dev_err(&priv->dev, "notifyid: %d %d\n", vring0->notifyid, vring1->notifyid);
		goto err;
	}
	priv->desc_num = vring0->num;
	priv->desc_size = (rsc_table->offset[4] - rsc_table->offset[3]) / priv->desc_num;
	priv->ns_offset = sizeof(struct rsc_table_desc);
	priv->ns_size = priv->desc_size - priv->ns_offset;
	priv->vring0_offset = rsc_table->offset[3];
	priv->vring1_offset = rsc_table->offset[4];
	priv->desc_vring0 = (unsigned char *)(priv->shmem_base + rsc_table->offset[3]);
	priv->desc_vring1 = (unsigned char *)(priv->shmem_base + rsc_table->offset[4]);
	priv->shmem_rx_base = priv->shmem_base + vring0->da;
	priv->shmem_rx_size = vring0->pa;
	priv->shmem_tx_base = priv->shmem_base + vring1->da;
	priv->shmem_tx_size = vring1->pa;
	priv->desc_rxbuf = priv->shmem_rx_size / priv->desc_num;
	priv->desc_txbuf = priv->shmem_tx_size / priv->desc_num;

	priv->buf_flag = kzalloc(priv->desc_num, GFP_KERNEL);
	priv->kick_ept = kzalloc(sizeof(void *) * priv->desc_num, GFP_KERNEL);
	if(!priv->buf_flag || !priv->kick_ept)
		goto err;
	if((priv->shmem_rx_size + priv->shmem_tx_size + vring0->da) > priv->shmem_size) {
		dev_err(&priv->dev, "Insufficient shared memory, please try increasing reserved-memory.\n");
		goto err;
	}
	else {
		dev_info(&priv->dev, "AMPv%d with Tx/Rx shmem %d/%d KiB, naming space %d letters.\n",
			rsc_table->ver, priv->shmem_tx_size >> 10, priv->shmem_rx_size >> 10, priv->ns_size);
	}

	spin_lock_init(&priv->lock);
	priv->ready = 1;
resu:
	ma35_rsc_table_update(priv);
	rsc_table->reserved[0] = 0;
	ma35_desc_init(priv);
	if (priv->resumeflow)
		priv->resumeflow = 0;

	return 0;
err:
	dev_err(&priv->dev, "Parameter error detected in resource table.\n");
	return -1;
}

static int ma35_ipi_register(struct ma35_rpmsg_priv *rpmsg_priv, struct device_node *node)
{
	struct device *dev = &rpmsg_priv->dev;
	struct device_node *ipi_np, *node1;
	struct resource r;
	struct resource_table *rsc_table;
	u64 base;
	u32 size, ch[2];
	int irq_num, ret;

	/* register rx IPI, default is timer8 */
	ipi_np = of_parse_phandle(node, "rxipi", 0);
	if (ipi_np) {
		ret = of_address_to_resource(ipi_np, 0, &r);
		if (ret == 0) {
			base = r.start;
			size = resource_size(&r);
		} else {
			goto out2;
		}
	} else {
		goto out2;
	}

	if (of_property_read_u32_array(ipi_np, "port-number", ch, 1) != 0) {
		pr_err("%s can not get port-number from timer!\n", __func__);
		return -EINVAL;
	}
	rpmsg_priv->ipi_rxch = ch[0];
	rpmsg_priv->ipi_rxbase = ioremap(base, size);

	/* clk init done in tmr driver */
	writel_relaxed(0x2, rpmsg_priv->ipi_rxbase + IPI_REG_CMP); // fastest
	writel_relaxed(IPI_CTL_INTEN, rpmsg_priv->ipi_rxbase + IPI_REG_CTL); // INTEN
	writel_relaxed(readl_relaxed(rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS) & IPI_INTSTS_TIF,
					rpmsg_priv->ipi_rxbase + IPI_REG_INTSTS); // clear
	/* request irq handler */
	irq_num = of_irq_get(ipi_np, 0);
	if (devm_request_irq(dev, irq_num, ma35_rpmsg_ipi_isr,
			IRQF_NO_SUSPEND, dev_name(dev), rpmsg_priv)) {
		pr_debug("register irq failed %d\n", irq_num);
		ret = -EAGAIN;
		goto out2;
	}
	rpmsg_priv->irq_num = irq_num;

	/* register tx IPI, default is timer9 */
	ipi_np = of_parse_phandle(node, "txipi", 0);
	if (ipi_np) {
		ret = of_address_to_resource(ipi_np, 0, &r);
		if (ret == 0) {
			base = r.start;
			size = resource_size(&r);
		} else {
			goto out1;
		}
	} else {
		goto out1;
	}

	if (of_property_read_u32_array(ipi_np, "port-number", ch, 1) != 0) {
		pr_err("%s can not get port-number from timer!\n", __func__);
		return -EINVAL;
	}
	rpmsg_priv->ipi_txch = ch[0];
	rpmsg_priv->ipi_txbase = ioremap(base, size);

	/* clk init done in tmr driver */
	writel_relaxed(0x2, rpmsg_priv->ipi_txbase + IPI_REG_CMP); // fastest
	writel_relaxed(IPI_CTL_INTEN, rpmsg_priv->ipi_txbase + IPI_REG_CTL); // INTEN
	writel_relaxed(readl_relaxed(rpmsg_priv->ipi_txbase + IPI_REG_INTSTS) & IPI_INTSTS_TIF,
					rpmsg_priv->ipi_txbase + IPI_REG_INTSTS); // clear
	/* Irq handler requested by remote */

	/* get shared momory region */
	node1 = of_parse_phandle(node, "memory-region", 0);
	if (node1) {
		ret = of_address_to_resource(node1, 0, &r);
		if (ret == 0) {
			base = r.start;
			size = resource_size(&r);
			rpmsg_priv->shmem_base = memremap(base, size, MEMREMAP_WB);
			rpmsg_priv->shmem_size = size;
		}
	} else {
		dev_err(dev, "Failed to parse share memory region.\n");
		goto out3;
	}

	rpmsg_priv->node = node;

	rsc_table = (struct resource_table *)rpmsg_priv->shmem_base;

	rsc_table->reserved[1] = IPI_CMD_REQUEST;
	ma35_rpmsg_notify_remote(rpmsg_priv);

	dev_info(dev, "IPI device registered.\n");

	return ret;

out1:
	dev_err(dev, "%s: can't parse \"txipi\" property\n", __func__);
	return -ENODEV;
out2:
	dev_err(dev, "%s: can't parse \"rxipi\" property\n", __func__);
	return -ENODEV;
out3:
	dev_err(dev, "%s: can't parse \"memory-region\" property\n", __func__);
	return -ENODEV;
}

static struct ma35_rpmsg_priv *ma35_rpmsg_register(struct device *parent, struct device_node *node)
{
	struct ma35_rpmsg_priv *rpmsg_priv;
	struct ma35_rpmsg_device *rpdev;
	int ret;

	rpmsg_priv = kzalloc(sizeof(*rpmsg_priv), GFP_KERNEL);
	if (!rpmsg_priv)
		return ERR_PTR(-ENOMEM);

	rpmsg_priv->dev.parent = parent;
	rpmsg_priv->dev.release = ma35_rpmsg_release;
	rpmsg_priv->dev.of_node = node;

	dev_set_name(&rpmsg_priv->dev, "ma35-amp");

	ret = device_register(&rpmsg_priv->dev);
	if (ret) {
		pr_err("failed to register rpmsg\n");
		put_device(&rpmsg_priv->dev);
		return ERR_PTR(ret);
	}

	rpdev = kzalloc(sizeof(*rpdev), GFP_KERNEL);
	if (!rpdev)
		return ERR_PTR(-ENOMEM);

	if(ma35_ipi_register(rpmsg_priv, node))
		goto unregister_dev;

	rpdev->rpmsg_priv = rpmsg_priv;
	rpdev->rpdev.ops = &ma35_rpmsg_device_ops;
	rpdev->rpdev.dev.parent = &rpmsg_priv->dev;
	rpdev->rpdev.dev.release = ma35_rpmsg_release_device;

	ret = rpmsg_chrdev_register_device(&rpdev->rpdev);
	if (ret) {
		dev_err(&rpmsg_priv->dev, "Failed to register rpmsg chrdev.\n");
		goto unregister_dev;
	}

	return rpmsg_priv;

unregister_dev:
	device_unregister(&rpmsg_priv->dev);
	kfree(rpmsg_priv);
	return ERR_PTR(ret);
}

static int ma35_rpmsg_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct ma35_rpmsg_priv *rpmsg_priv;

	rpmsg_priv = ma35_rpmsg_register(&pdev->dev, node);

	return 0;
}

static int ma35_rpmsg_remove_device(struct device *dev, void *data)
{
	device_unregister(dev);

	return 0;
}

static int ma35_rpmsg_remove_priv(struct device *dev, void *data)
{
	struct ma35_rpmsg_priv *priv = to_ma35_rpmsg_priv(dev);
	int ret;

	if(priv->bdthread) {
		kthread_stop(priv->bdthread);
		wake_up_interruptible(&priv->bd_event);
	}

	ret = device_for_each_child(&priv->dev, NULL, ma35_rpmsg_remove_device);
	if (ret)
		dev_warn(&priv->dev, "Can't remove rpmsg device: %d\n", ret);

	device_unregister(&priv->dev);

	return 0;
}

static int ma35_rpmsg_remove(struct platform_device *pdev)
{
	int ret;

	ret = device_for_each_child(&pdev->dev, NULL, ma35_rpmsg_remove_priv);
	if (ret)
		dev_warn(&pdev->dev, "Can't remove rpmsg device: %d\n", ret);

	return ret;
}

static int ma35_rpmsg_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static void periodic_work_handler(struct work_struct *work)
{
	struct ma35_rpmsg_priv *rpmsg_priv = container_of(to_delayed_work(work), struct ma35_rpmsg_priv, dwork);
	struct resource_table *rsc_table;

	rsc_table = (struct resource_table *)rpmsg_priv->shmem_base;
	if(rpmsg_priv->resumeflow) {
		rsc_table->reserved[1] = IPI_CMD_REQUEST;
		ma35_rpmsg_notify_remote(rpmsg_priv);
		schedule_delayed_work(&rpmsg_priv->dwork, msecs_to_jiffies(500));
	} else {
		cancel_delayed_work(&rpmsg_priv->dwork);
	}
}

static int ma35_rpmsg_resume_priv(struct device *dev, void *data)
{
	struct ma35_rpmsg_priv *rpmsg_priv;
	struct delayed_work *dwork;

	rpmsg_priv = to_ma35_rpmsg_priv(dev);
	if (rpmsg_priv && rpmsg_priv->ready) {
		rpmsg_priv->resumeflow = IPI_CMD_REQUEST; // start resume flow
		dwork = &rpmsg_priv->dwork;
		INIT_DELAYED_WORK(dwork, periodic_work_handler);
		schedule_delayed_work(dwork, msecs_to_jiffies(500)); // issue delay work
	}

	return 0;
}

static int ma35_rpmsg_resume(struct platform_device *pdev)
{
	int ret;

	ret = device_for_each_child(&pdev->dev, NULL, ma35_rpmsg_resume_priv);
	if (ret)
		dev_warn(&pdev->dev, "Reset remote core failed: %d\n", ret);

	return ret;
}

static const struct of_device_id ma35_amp_of_match[] = {
	{ .compatible = "nuvoton,ma35-amp" },
	{}
};
MODULE_DEVICE_TABLE(of, ma35_amp_of_match);

static struct platform_driver ma35_rpmsg_driver = {
	.probe = ma35_rpmsg_probe,
	.remove = ma35_rpmsg_remove,
	.suspend = ma35_rpmsg_suspend,
	.resume = ma35_rpmsg_resume,
	.driver = {
		.name = "ma35-amp",
		.of_match_table = ma35_amp_of_match,
	},
};
module_platform_driver(ma35_rpmsg_driver);

static void __exit ma35_rpmsg_dev_exit(void)
{
	platform_driver_unregister(&ma35_rpmsg_driver);
}
module_exit(ma35_rpmsg_dev_exit);


MODULE_DESCRIPTION("Nuvoton MA35 series AMP driver");
MODULE_LICENSE("GPL v2");
