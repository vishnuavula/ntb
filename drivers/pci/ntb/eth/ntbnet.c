/*
 * This program implements network nriver over NTB hardware.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/kernel.h> 
#include <linux/slab.h> 
#include <linux/errno.h>  
#include <linux/types.h>  
#include <linux/interrupt.h> 
#include <linux/in.h>
#include <linux/netdevice.h>   
#include <linux/etherdevice.h> 
#include <linux/inetdevice.h>
#include <net/neighbour.h>
#include <linux/ip.h>          
#include <linux/tcp.h>         
#include <linux/skbuff.h>
#include <linux/async_tx.h>
#include <linux/in6.h>
#include <asm/checksum.h>

#include "ntbethcq.h"
#include "ntbdev.h"
#include "ntbethcopier.h"
#include "ntbnet.h"

#define NTBETH_WATCHDOG_PERIOD (100*HZ);

// module parameter that tells if to use CB3 DMA hardware accelaration for memcpying packets from skb to remote NTB queue and from NTB queue to skb.

static int use_cb3_dma_engine = 0;
module_param(use_cb3_dma_engine,int, 0);

// module parameter that tells which doorbell to use to generate interrupt to the remote side after the local side transmitted a packet into to remote CQ. This is a way to exercise all the door bell bits (ofcourse mutually exclusive).

static int rx_int_doorbell_num = 0;
module_param(rx_int_doorbell_num,int, 0);

static int bar23_size = 0x100000;
module_param(bar23_size,int, 0);

static int bar45_size = 0x100000 ;
module_param(bar45_size,int, 0);

static int turnoff_tx = 0;
module_param(turnoff_tx,int, 0);

static int turnoff_pkt_count = 0;
module_param(turnoff_pkt_count,int, 0);

struct ntbeth_priv * ntbeth_device;

static const struct net_device_ops ntbeth_netdev_ops = {
        .ndo_open               = ntbeth_open,
        .ndo_stop               = ntbeth_close,
        .ndo_start_xmit         = ntbeth_tx,
        //.ndo_set_multicast_list = ntbeth_set_multicast_list,
        .ndo_set_mac_address    = ntbeth_set_mac_address,
        .ndo_change_mtu         = ntbeth_change_mtu,
        .ndo_do_ioctl           = ntbeth_do_ioctl,
        .ndo_tx_timeout         = ntbeth_tx_timeout,
};

void ntbeth_set_multicast_list(struct net_device *netdev)
{
      // because ntbeth is a point-to-point interface, no need to implement this.
        NTBETHDEBUG("Made it to ntbet_set_multicast_list\n");
        return ;
}

void ntbeth_tx_timeout(struct net_device *netdev)
{
       // for now we increment time out count and spit out an error message and proceed...
       struct ntbeth_priv *priv = netdev_priv(netdev);
       
       spin_lock_bh(&priv->lock);
       priv->tx_timeout_count++;    
       NTBETHDEBUG("NTBETH:ERROR: tx timed out%d\n",priv->tx_timeout_count);
       netdev->trans_start = jiffies;
       spin_unlock_bh(&priv->lock);
       return;
}

int ntbeth_open(struct net_device *dev)
{	
	struct ntbeth_priv * priv;
	priv = netdev_priv(dev);
	NTBETHDEBUG("Made it to ntbeth_open\n");
	memcpy(dev->dev_addr, NTBETH_MAC, ETH_ALEN);
        update_peer_status(dev, NTBETH_LOCAL_PEER_UP);
        ntbdev_send_ping_doorbell_interrupt(&priv->ntbdev);
        return 0;
}

int ntbeth_close(struct net_device *dev)
{
	struct ntbeth_priv * priv;
	NTBETHDEBUG("Made it to ntbeth_close\n");
	priv = netdev_priv(dev);
        update_peer_status(dev, NTBETH_LOCAL_PEER_DOWN);
        ntbdev_send_close_interrupt(&priv->ntbdev);
	return 0;
}

void rx_copy_callback(void *pref)
{
        int ret = 0;
        struct sk_buff *skb = (struct sk_buff *)pref;
	struct net_device *dev = (struct net_device *)skb->dev;
	struct ntbeth_priv *priv = netdev_priv(dev);
        NTBETHDEBUG("Made it to rx_copy_callback \n");
#ifndef USE_DBG_PKTS
        skb->protocol = eth_type_trans(skb, dev);
        priv->stats.rx_packets++;
        priv->stats.rx_bytes += skb->len;
        ret = netif_rx(skb);
        if(ret)
        {
          if(ret == NET_RX_DROP)
           printk("ntheth: RX PACKET DROPPED by Kernel\n");
          else
           printk("ntheth: RX PACKET has some problems in the Kernel\n");
        }
#else
  dev_kfree_skb(skb);
#endif 
        cq_update_get_ptr(priv->rxcq);
        // send tx ack now
     //   ntbdev_send_packet_transfer_ack_interrupt(&priv->ntbdev);
        return;
}	

void ntbeth_lnkchg_interrupt(void *pref)
{
	struct net_device *dev = (struct net_device *)pref;
	struct ntbeth_priv *priv = netdev_priv(dev);
        update_peer_status(dev, NTBETH_REMOTE_PEER_DOWN); 
        // send ping
        ntbdev_send_ping_doorbell_interrupt(&priv->ntbdev);
	return;
}

void ntbeth_ping_ack_interrupt(void *pref)
{
	struct net_device *dev = (struct net_device *)pref;
	//struct ntbeth_priv *priv = netdev_priv(dev);
        NTBETHDEBUG("Made it to ping ack interrupt routine\n");
        update_peer_status(dev, NTBETH_REMOTE_PEER_UP); 
	return;
}

void ntbeth_close_interrupt(void *pref)
{
	struct net_device *dev = (struct net_device *)pref;
	//struct ntbeth_priv *priv = netdev_priv(dev);
        update_peer_status(dev, NTBETH_REMOTE_PEER_DOWN); 
 // do not need to send close back
	return;
}

void ntbeth_ping_interrupt(void *pref)
{
	struct net_device *dev = (struct net_device *)pref;
	struct ntbeth_priv *priv = netdev_priv(dev);
        NTBETHDEBUG("Made it to ping interrupt routine\n");
        update_peer_status(dev, NTBETH_REMOTE_PEER_UP); 
        // send ping ack
        if((priv->peer_status & NTBETH_LOCAL_MASK) == NTBETH_LOCAL_PEER_UP)
        {
           ntbdev_send_ping_ack_doorbell_interrupt(&priv->ntbdev);
        }
	return;
}

void ntbeth_txack_interrupt(void *pref)
{
	//struct net_device *dev = (struct net_device *)pref;
	//struct ntbeth_priv *priv = netdev_priv(dev);
        NTBETHDEBUG("Made it to txack interrupt routine\n");
	return;
}

void ntbeth_rx_interrupt(void * pref)
{
	struct net_device *dev = (struct net_device *)pref;
	struct ntbeth_priv *priv = netdev_priv(dev);
        struct sk_buff *skb;
        char *data;
        unsigned int len;
        NTBETHDEBUG("ntbeth_rx_interrupt entered with devptr 0x%Lx\n",(unsigned long long)dev);
        NTBETHDEBUG("ntbeth_rx_interrupt entered with priv 0x%Lx\n",(unsigned long long)priv);
        spin_lock_bh(&priv->lock);
        // pull message from 
        while(!(is_cq_empty(priv->rxcq)))
        {
           data = cq_get_current_get_entry_loc(priv->rxcq);  
           if(data == 0)
           {
             printk(KERN_INFO "ntbeth: ntbeth_rx_interrupt received but rxq is empty\n");
             spin_unlock_bh(&priv->lock);
             return;
          }
          priv->rx_pkt_count++;
          len = *(unsigned int *)data; 
	  skb = dev_alloc_skb(len+2); // to make IP frame 16B aligned 
	  if (!skb) {
		if (printk_ratelimit())
			printk(KERN_INFO "ntbeth  rx: low on mem - packet dropped\n");
		priv->rx_dropped++;
                spin_unlock_bh(&priv->lock);
		return;
	  }
          skb->dev = dev;
          skb->ip_summed = CHECKSUM_UNNECESSARY;
          skb_reserve(skb, 2);
          // DumpMemory(data, len);
          ntbeth_copier_copy_to_skb(&priv->copier,data+4, len, skb, rx_copy_callback, skb);
         // dump_info(priv, DEBUG_RX, data, len);
        } 
        spin_unlock_bh(&priv->lock);
        return;
}

// we expect the callback completes in order of submission
void tx_copy_callback(void *pref)
{
  struct sk_buff *skb = (struct sk_buff *)pref;
  struct ntbeth_priv *priv = netdev_priv(skb->dev);
  NTBETHDEBUG("tx_copy_callback invoked\n");
  cq_update_put_ptr(priv->txcq); 
  // we also need to send interrupt to the remote node
  dev_kfree_skb(skb);
  ntbdev_send_packet_txed_interrupt(&priv->ntbdev);
  return;
}

/*
 * Transmit a packet (called by the kernel)
 */
int ntbeth_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct ntbeth_priv *priv = netdev_priv(dev);
        char *data;
        int len;
        NTBETHDEBUG("Made it to ntbeth_tx \n");
        // obtain queue entry in ntb CQ.
        spin_lock_bh(&priv->lock);
        len = skb->len;
        dev->trans_start = jiffies;

        // drop the packet if peer status is not  up
        if(!(priv->peer_status & NTBETH_REMOTE_PEER_UP))
        {
          NTBETHDEBUG(" packet dropped because remote peer is down: peer status 0x%x\n", priv->peer_status);
          dev_kfree_skb(skb);
          spin_unlock_bh(&priv->lock);
          return 0;
        }
#if 1
        if(turnoff_tx)
        {
          if(priv->tx_pkt_count == turnoff_pkt_count)
          {
           dev_kfree_skb(skb);
           spin_unlock_bh(&priv->lock);
           return 0;
          }
        }
#endif 
        if(is_cq_full(priv->txcq))
        {
          NTBETHDEBUG(" packet dropped because remote cq is full 0x%x\n", priv->peer_status);
          ntbdev_send_packet_txed_interrupt(&priv->ntbdev);
          dev_kfree_skb(skb);
          spin_unlock_bh(&priv->lock);
          return 0;
        }
        data = cq_get_current_put_entry_loc(priv->txcq);
        if(data == NULL)
        {
          printk(KERN_INFO "Something wrong cq is full\n");
          // no need to release skb
          // notify kernel that it should stop queue
          // just send one more remainder of availability of packets in the queue 
          ntbdev_send_packet_txed_interrupt(&priv->ntbdev);
          dev_kfree_skb(skb);
          spin_unlock_bh(&priv->lock);
          return 0;
        }
        priv->tx_pkt_count++;
        priv->stats.tx_packets++;
        priv->stats.tx_bytes += len;
        *(unsigned int *)data = len; 
        ntbeth_copier_copy_from_skb(&priv->copier,skb,data+4, len, tx_copy_callback, skb);
        //dump_info(priv, DEBUG_TX, data, len); 
        spin_unlock_bh(&priv->lock);
        return 0; 
}

/*
 * Return statistics to the caller
 */
struct net_device_stats *ntbeth_stats(struct net_device *dev)
{
	struct ntbeth_priv *priv = netdev_priv(dev);
	return &priv->stats;
}

int ntbeth_change_mtu(struct net_device *dev, int new_mtu)
{
	struct ntbeth_priv *priv = netdev_priv(dev);

	/* check ranges */
	if ((new_mtu < NTBETH_MIN_MTUSIZE) || (new_mtu > NTBETH_MAX_MTUSIZE))
		return -EINVAL;
        NTBETHDEBUG("Chaning Ntbeth MTU Size to %d\n", new_mtu);	
	spin_lock_bh(&priv->lock);
	dev->mtu = new_mtu;
	spin_unlock_bh(&priv->lock);
	return 0; /* success */
}

int ntbeth_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{

       // current we do not implement any custom ioctls so we leave this empty.
      // but in future we can implment some ioctls to dump driver/ntb structures
        //struct ntbeth_priv *priv = netdev_priv(netdev);
        NTBETHDEBUG("ntbeth_do_ioctl entered\n");
        return 0;
}

int ntbeth_init(struct net_device *dev)
{
        int err;
	struct ntbeth_priv *priv = netdev_priv(dev);
	NTBETHDEBUG("Made it to ntbeth_init\n");
	ether_setup(dev); /* assign some of the fields */
	
	/* Set up some of the private variables now */
	memset(priv, 0, sizeof(struct ntbeth_priv));
        NTBETHDEBUG("private ptr: 0x%Lx\n", (unsigned long long)priv);
	priv->status = 0;
	spin_lock_init(&priv->lock);
	netif_stop_queue(dev);
        update_peer_status(dev,NTBETH_LOCAL_PEER_DOWN);
        update_peer_status(dev,NTBETH_REMOTE_PEER_DOWN);
	dev->mtu		= NTBETH_MAX_MTUSIZE;
	dev->flags		|= IFF_NOARP;
//	dev->features		|= NETIF_F_NO_CSUM | NETIF_F_SG;
	dev->features		|= NETIF_F_NO_CSUM; 
        if((err = ntbeth_copier_init(&priv->copier, use_cb3_dma_engine)))
        {
          return (err);
        }
        NTBETHDEBUG("rx_int_doorbell num  %d\n", rx_int_doorbell_num);
        // initialize ntb device info structures.  
        if((err = ntbdev_init(&priv->ntbdev, bar23_size, bar45_size, rx_int_doorbell_num)))
        {
          return (err);
        }
        // Initialize CQ to recv message from remote side
        priv->rxcq = ntbdev_get_bar23_local_memory(&priv->ntbdev);
        priv->txcq = ntbdev_get_bar23_value(&priv->ntbdev);
        init_cq(priv->rxcq, cq_calculate_num_entries(bar23_size),NTBETH_RX_CQ,ntbdev_get_bar23_value(&priv->ntbdev)); 
        // obtain rxcq  ptr
        NTBETHDEBUG("RxCQ Ptr 0x%Lx\n", (unsigned long long)priv->rxcq);
        NTBETHDEBUG("TxCQ Ptr 0x%Lx\n", (unsigned long long)priv->txcq);
        NTBETHDEBUG("bar23size 0x%Lx\n", (unsigned long long)bar23_size);
        NTBETHDEBUG("bar45size 0x%Lx\n", (unsigned long long)bar45_size);
        cq_dump_debug_data(priv->rxcq,"INIT: ");
        cq_dump_debug_data(priv->txcq,"INIT: ");
        NTBETHDEBUG("tx_pkt_count %d\n", priv->tx_pkt_count);
        NTBETHDEBUG("rx_pkt_count %d\n", priv->rx_pkt_count);

        return 0;
}

int ntbeth_set_mac_address(struct net_device *netdev, void *p)
{
       // struct ntbeth_priv *priv = netdev_priv(netdev);
        NTBETHDEBUG("ntbeth_set_mac_address: Copied MAC Address to dev_addr\n");
        memcpy(netdev->dev_addr, p, ETH_ALEN);
        return 0;
}

void DumpMemory(char *memoryloc, int size, char *fmtstr)
{
  unsigned int *pBuf = (unsigned int *)memoryloc;
  int i;
  for(i=0; i < size/4; i++)
  {
  if (i%4 == 0) printk("\n%s0x%02x: ",fmtstr,i*4);
    printk("%s0x%08x ",fmtstr, pBuf[i]);
  }
  return;
}
void ntbeth_cleanup(void)
{
	unregister_netdev(ntbeth_device->netdev);
        ntbeth_copier_cleanup(&ntbeth_device->copier);
        // cleanup ntb device info structures.  
        ntbdev_cleanup(&ntbeth_device->ntbdev);
        // have to free private structure TBD???
	return;
}

int ntbeth_init_module(void)
{
        int err;
	struct net_device * netdev;
        struct ntbeth_priv *priv;
        if (!(netdev = alloc_etherdev(sizeof(struct ntbeth_priv)))) {
                        printk(KERN_ERR "Etherdev alloc failed, abort.\n");
                return -ENOMEM;
        }
        NTBETHDEBUG("netdev ptr 0x%Lx\n", (unsigned long long)netdev);
        netdev->netdev_ops = &ntbeth_netdev_ops;
        netdev->get_stats = ntbeth_stats;
        netdev->watchdog_timeo = NTBETH_WATCHDOG_PERIOD;
        strncpy(netdev->name, "ntb1",5);
        if((err = register_netdev(netdev)))
        {
          printk(KERN_ERR "Unable to register network device with Kernel\n");
          return err;
        }
        ntbeth_init(netdev);
        priv = netdev_priv(netdev);
        priv->netdev = netdev;
        ntbeth_device = priv;
        ntbdev_mask_doorbell_interrupts(&priv->ntbdev);
       // subscribe to rx interrupt callback
        ntbdev_subscribe_to_rx_int(&priv->ntbdev, ntbeth_rx_interrupt, netdev);
        ntbdev_subscribe_to_txack_int(&priv->ntbdev, ntbeth_txack_interrupt, netdev);
        ntbdev_subscribe_to_ping_int(&priv->ntbdev, ntbeth_ping_interrupt, netdev);
        ntbdev_subscribe_to_ping_ack_int(&priv->ntbdev, ntbeth_ping_ack_interrupt, netdev);
        ntbdev_subscribe_to_lnkchg_int(&priv->ntbdev, ntbeth_lnkchg_interrupt, netdev);
        ntbdev_subscribe_to_close_int(&priv->ntbdev, ntbeth_close_interrupt, netdev);
        ntbdev_unmask_doorbell_interrupts(&priv->ntbdev);
        NTBETHDEBUG(" ntbeth_init_module completed successfully\n");
        NTBETHDEBUG( "ntbeth_init_module completed successfully\n");
	return 0;
}

void update_peer_status(struct net_device *netdev, int peer_status)
{
   struct ntbeth_priv *priv = netdev_priv(netdev);
   spin_lock_bh(&priv->lock);
   // clear link status 
    switch(peer_status)
    {
      case NTBETH_REMOTE_PEER_UP: 
      case NTBETH_REMOTE_PEER_DOWN: 
      priv->peer_status = (~NTBETH_REMOTE_MASK & priv->peer_status)|peer_status;
      break;
      case NTBETH_LOCAL_PEER_UP: 
      case NTBETH_LOCAL_PEER_DOWN: 
      priv->peer_status = (~NTBETH_LOCAL_MASK & priv->peer_status)|peer_status;
      break;
    }
   if(((priv->peer_status &  NTBETH_LOCAL_MASK) == NTBETH_LOCAL_PEER_UP) && ((priv->peer_status & NTBETH_REMOTE_MASK) == NTBETH_REMOTE_PEER_UP))
   {
        NTBETHDEBUG("ntbeth: update_peer_status  both sides are up\n");
     netif_wake_queue(netdev);
   }
   else
   {
        NTBETHDEBUG("ntbeth: update_peer_status  at least one side is down\n");
     netif_stop_queue(netdev);
   }
   spin_unlock_bh(&priv->lock);
}

void dump_info(struct ntbeth_priv *priv, int side, void *pkt, int len)
{
  switch(side)
  {
       case DEBUG_RX:
       printk("\t\t\t\t\tRxed Pkt xCount %d w/len %d\n", priv->rx_pkt_count,len);
       cq_dump_debug_data(priv->rxcq, "\t\t\t\t\t");
	break;
       case DEBUG_TX:
       printk("Txed Pkt Count %d w/len %d\n", priv->tx_pkt_count, len);
       cq_dump_debug_data(priv->txcq, " ");
	break;
  }
}

MODULE_LICENSE("GPL");
module_init(ntbeth_init_module);
module_exit(ntbeth_cleanup);
