/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2009 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * BSD LICENSE
 *
 * Copyright(c) 2009 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* This program implements network driver over NTB hardware.*/

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
// debug pkt info
       unsigned int dbgpkt_len;
       unsigned int dbgpkt_counter;
       struct ntbeth_ntbdev *ntbdev;
};

int ntbeth_copier_init(struct ntbeth_copier_info *pcopier, int usecb3, struct ntbeth_ntbdev *ntb_dev);
//int ntbeth_copier_copy_to_skb(struct ntbeth_copier_info *pcoiper, char *pmsg, int length, struct sk_buff *skb, void (*pcallback)(void *), void *pref);
int ntbeth_ntb_doorbell_enable(struct ntbeth_copier_info *pcopier);
int ntbeth_copier_cleanup(struct ntbeth_copier_info *pcopier);
//int ntbeth_copier_copy_from_skb(struct ntbeth_copier_info *pcopier, struct sk_buff *skb, char *pmsg, int length, void (*pcallback)(void *), void *pref);
void ntbeth_copier_memcpy(char *dest, char *src, int length);
int ntbeth_copier_copy_from_skb(struct ntbeth_copier_info *pcopier, struct sk_buff *skb, dma_addr_t *dma_addr,char *pmsg, int length, void (*pcallback)(void *), void *pref);
int ntbeth_copier_copy_to_skb(struct ntbeth_copier_info *pcopier, char *pmsg, int length, struct sk_buff *skb, dma_addr_t *dma_addr, void (*pcallback)(void *), void *pref);

#endif 
