/*
 * This program implements network driver over NTB hardware. 
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
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 */

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
//	int put;
//	int get;
        struct net_device *netdev;
        struct cq *rxcq; // reside locally
        struct cq *txcq; // reside locally but packet store is remote 
        int peer_status; 
        int local_drv_if_status;
        int remote_drv_if_status;
        int txq_status;
        int rx_pkt_count;
        int tx_pkt_count;
       // int tx_packet_sent_count;
      //  int tx_packet_ack_sent_count;
       // int tx_packet_rcvd_count;
        //int tx_packet_ack_rcvd_count;
        int tx_timeout_count;
        int rx_dropped;
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
void DumpMemory(char *memoryloc, int size, char *fmtstr);
void update_peer_status(struct net_device *netdev, int peer_status);
void dump_info(struct ntbeth_priv *priv, int side, void *pkt, int len);

#endif 
