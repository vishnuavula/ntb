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

/* This file implements network driver over NTB PCIE-LINK */
#ifndef NTBNET_H
#define NTBNET_H

#include "ntbethconf.h"

#define DEBUG_TX      2
#define DEBUG_RX      1

#define NTBETH_STATUS_LINKDOWN	(1<<3)
#define NTBETH_STATUS_LINKUP	(2<<3)

#define NTBETH_Q_NOT_FULL       0 // default
#define NTBETH_Q_FULL           2


#define NTBETH_REMOTE_MASK  0xFF00
#define NTBETH_LOCAL_MASK   0x00FF

#define NTBETH_REMOTE_PEER_UP    0x1100
#define NTBETH_REMOTE_PEER_DOWN  0x1000

#define NTBETH_LOCAL_PEER_UP    0x01
#define NTBETH_LOCAL_PEER_DOWN  0x00


struct ntbeth_priv
{
	struct net_device_stats stats;
	struct ntbeth_copier_info copier;
	struct ntbeth_ntbdev ntbdev;
	int status;
	spinlock_t lock;
	struct net_device *netdev;
	struct cq *rxcq; // reside locally
	struct cq *txcq; // reside locally but packet store is remote
	int peer_status;
	int local_drv_if_status;
	int remote_drv_if_status;
	int txq_status;
	int rx_pkt_count;
	int tx_pkt_count;
	int tx_timeout_count;
	int rx_dropped;
	dma_addr_t *rx_dma_addresses;
	dma_addr_t *tx_dma_addresses;
	int tx_pending_pkts;
	struct timer_list perf_tmr;
};

int ntbeth_open(struct net_device *dev);
int ntbeth_close(struct net_device *dev);
int ntbeth_tx(struct sk_buff *skb, struct net_device *dev);
int ntbeth_set_mac_address(struct net_device *netdev, void *p);
void ntbeth_set_multicast_list(struct net_device *netdev);
int ntbeth_change_mtu(struct net_device *dev, int new_mtu);
int ntbeth_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
void ntbeth_tx_timeout(struct net_device *netdev);
void ntbeth_rx_interrupt(void * pref);
void ntbeth_txack_interrupt(void *pref);
void ntbeth_lnkchg_interrupt(void *pref);
int ntbeth_init(struct net_device *dev);
void ntbeth_ping_interrupt(void * pref);
void ntbeth_ping_ack_interrupt(void * pref);
void ntbeth_decide_traffic_on_off(struct net_device *pdev);
void dump_memory(char *memoryloc, int size, char *fmtstr);
void update_peer_status(struct net_device *netdev, int peer_status);
void dump_info(struct ntbeth_priv *priv, int side, void *pkt, int len);
struct net_device_stats *ntbeth_stats(struct net_device *dev);

#endif
