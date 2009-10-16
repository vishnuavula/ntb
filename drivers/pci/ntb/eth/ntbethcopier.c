/*
 * This program implements network driver onver NTB hardware. 
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

#include <linux/dmaengine.h>

#include "ntbethcopier.h"
#include "ntbdev.h"

int ntbeth_copier_init(struct ntbeth_copier_info *pcopier, int usecb3, struct ntbeth_ntbdev *ntbdev)
{
   dma_cap_mask_t dma_mask;
   pcopier->use_cb3_dma_engine = usecb3;
   pcopier->dbgpkt_len = 4;
   pcopier->dbgpkt_counter = 0;
   pcopier->ntbdev = ntbdev;
   if(pcopier->use_cb3_dma_engine)
   {
      dma_cap_zero(dma_mask);
      dma_cap_set(DMA_MEMCPY,dma_mask);
      dmaengine_get();
      pcopier->chan = dma_find_channel(DMA_MEMCPY);
      if(pcopier->chan == NULL)
      {
        printk("Copier initialization failed\n");
        return(NTBETH_FAIL);
      }
   }
  NTBETHDEBUG("Initialized Copier\n");
  return (NTBETH_SUCCESS);
}

int ntbeth_copier_cleanup(struct ntbeth_copier_info *pcopier)
{
  if(pcopier->use_cb3_dma_engine)
  {
    dmaengine_put();
    return (NTBETH_SUCCESS);
  }
  return NTBETH_SUCCESS;
}
void ntbeth_copier_memcpy(char *dest, char *src, int length)
{
  int num_longs = length/8;
  unsigned long long *psrc, *pdest;
  int i;
  psrc = (unsigned long long *)src;
  pdest = (unsigned long long *)dest;
  for(i=0; i < num_longs; i++)
  {
    pdest[i]  = psrc[i];
  }
  memcpy(pdest + i, psrc + i, length%8);
}
int ntbeth_copier_copy_to_skb(struct ntbeth_copier_info *pcopier, char *pmsg, int length, struct sk_buff *skb, dma_addr_t *dma_addr, void (*pcallback)(void *), void *pref)
{
   unsigned long long flags;
   dma_addr_t src, dest;
   struct dma_async_tx_descriptor *tx;
   dma_cookie_t cookie;

   if(pcopier->use_cb3_dma_engine)
   {
      if(ntbdev_get_bus_address_for_local_buffers(pcopier->ntbdev, pmsg, length, &src))
      {
        printk("Unable to obtain PCI address for the given src address\n");
        return NTBETH_FAIL;
      }
       //printk("obtain PCI address for the given src address\n");
      if(ntbdev_get_bus_address(NULL, skb_put(skb,length), length, &dest))
      {
        printk("Unable to obtain PCI address for the given dest address\n");
        return NTBETH_FAIL;
      }
      *dma_addr = dest;
      // printk("obtain PCI address for the given dest address\n");
      flags = DMA_CTRL_ACK | DMA_COMPL_SKIP_DEST_UNMAP | DMA_PREP_INTERRUPT;
      tx= pcopier->chan->device->device_prep_dma_memcpy(pcopier->chan, dest, src, length, flags);
      tx->callback = pcallback; 
      tx->callback_param = pref; 
      //printk("passed skb ptr to dma subsystem  0x%Lx\n",pref);
       
      cookie = tx->tx_submit(tx);
      if(dma_submit_error(cookie))
      {
         printk("Submit Error Returned cookied 0x%Lx\n",(unsigned long long) cookie);
         return NTBETH_FAIL;
      }
      dma_async_issue_pending(pcopier->chan); 
   }
   else
   {
      // copy message to sk buffer and call callback
       ntbeth_copier_memcpy(skb_put(skb, length), pmsg, length); 
#ifdef USE_DBG_PKTS
       printk("\t\t\t\t\tRxed Pkt Counter %d Lenth %d  Pat 0x%x\n", *(unsigned int *)pmsg, length, *(unsigned int *)(pmsg + 4));
#endif 
       NTBETHDEBUG("Copied to SKB from  RxCq\n");
       pcallback(pref);
    }
   return NTBETH_SUCCESS; 
}
int ntbeth_copier_copy_from_skb(struct ntbeth_copier_info *pcopier, struct sk_buff *skb, dma_addr_t *dma_addr,char *pmsg, int length, void (*pcallback)(void *), void *pref)
{
   unsigned long long flags;
   dma_addr_t src, dest;
   struct dma_async_tx_descriptor *tx;
   dma_cookie_t cookie;

   if(pcopier->use_cb3_dma_engine)
   {
      if(ntbdev_get_bus_address(NULL, skb->data, length, &src))
      {
        printk("Unable to obtain PCI address for the given src address\n");
        return NTBETH_FAIL;
      }
      *dma_addr = src;
//      printk(" obtained PCI address for the given src address\n");
      if(ntbdev_get_bus_address_for_remote_buffers(pcopier->ntbdev, pmsg, length, &dest))
      {
        printk("Unable to obtain PCI address for the given dest address\n");
        return NTBETH_FAIL;
      }
//      printk(" obtained PCI address for the given dest address\n");
      flags = DMA_CTRL_ACK | DMA_COMPL_SKIP_DEST_UNMAP | DMA_PREP_INTERRUPT;
#ifdef USE_DBG_PKTS
     printk("Copier Txing pkt %d\n", pcopier->dbgpkt_counter); 
     *(unsigned int *)((char *)pmsg-4) = pcopier->dbgpkt_len;
     *(unsigned int *)((char *)skb->data) = pcopier->dbgpkt_counter++;
     *(unsigned int *)((char *)skb->data+4) = 0xA5A5A5A5;
     length = pcopier->dbgpkt_len;
#endif
     // printk("Tx copying %d bytes\n", length);
      tx= pcopier->chan->device->device_prep_dma_memcpy(pcopier->chan, dest, src, length, flags);
     tx->callback = pcallback; 
     tx->callback_param = pref; 
     cookie = tx->tx_submit(tx);
     if(dma_submit_error(cookie))
     {
         printk("Submit Error Returned cookied 0x%Lx\n",(unsigned long long) cookie);
         return NTBETH_FAIL;
     }
     dma_async_issue_pending(pcopier->chan); 
   }
   else
   {
      // copy message to sk buffer and call callback
       NTBETHDEBUG("Copied from SKB to  txCq\n");
#ifdef USE_DBG_PKTS
   // instead of sending the IP packet we will send debug pkt
   // debug packet format is length(4),counter (4), payload pattern 
     printk("Copier Txing pkt %d\n", pcopier->dbgpkt_counter); 
     *(unsigned int *)((char *)pmsg-4) = pcopier->dbgpkt_len;
     *(unsigned int *)((char *)pmsg) = pcopier->dbgpkt_counter++;
     *(unsigned int *)((char *)pmsg+4) = 0xA5A5A5A5;
     
#else
       ntbeth_copier_memcpy(pmsg, skb->data, length); 
#endif 
      pcallback(pref);
   }
   return NTBETH_SUCCESS; 
}
