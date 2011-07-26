/*
 * This program implements API to control NTB hardware.
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#ifndef NTBETHCOPIER_H
#define NTBETHCOPIER_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/kernel.h> /* printk() */
#include <linux/slab.h> /* kmalloc() */
#include <linux/errno.h>  /* error codes */
#include <linux/types.h>  /* size_t */
#include <linux/interrupt.h> /* mark_bh */
#include <linux/in.h>
#include <linux/netdevice.h>   /* struct device, and other headers */
#include <linux/etherdevice.h> /* eth_type_trans */
#include <linux/inetdevice.h>
#include <net/neighbour.h>
#include <linux/ip.h>          /* struct iphdr */
#include <linux/tcp.h>         /* struct tcphdr */
#include <linux/skbuff.h>
#include <linux/async_tx.h>
#include <linux/in6.h>
#include <asm/checksum.h>

#include "ntbethconf.h"

struct ntbeth_copier_info
{
// This field is used only if CB3 DMA engine is used to do the memcpy transfers.
	struct dma_chan * chan;
        int use_cb3_dma_engine;
};

int ntbeth_copier_init(struct ntbeth_copier_info *pcopier, int usecb3);
int ntbeth_copier_copy_to_skb(struct ntbeth_copier_info *pcoiper, char *pmsg, int length, struct sk_buff *skb, void (*pcallback)(void *), void *pref);
int ntbeth_ntb_doorbell_enable(struct ntbeth_copier_info *pcopier);
int ntbeth_copier_cleanup(struct ntbeth_copier_info *pcopier);
int ntbeth_copier_copy_from_skb(struct ntbeth_copier_info *pcopier, struct sk_buff *skb, char *pmsg, int length, void (*pcallback)(void *), void *pref);

#endif 
