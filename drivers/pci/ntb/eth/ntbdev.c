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
#include "ntbdev.h"

struct ntbeth_ntbdev *gntbdev;

void ntbeth_bar23_callback(ntb_client_handle_t handle, int16_t doorbell_value, struct scratchpad_registers pad)
{
  struct ntbeth_ntbdev *pdev = gntbdev; 
     NTBETHDEBUG( "Entered ntbeth_bar23_callback db val 0x%x\n",doorbell_value);
     if(doorbell_value & (1<< pdev->rx_int_doorbell_num))
     {
       NTBETHDEBUG( "Invoking rx_interrupt callback\n");
       if(pdev->prx_int_callback)
       pdev->prx_int_callback(pdev->rx_int_callback_ref);
     }
     if(doorbell_value & (0x2 << pdev->rx_int_doorbell_num))
     {
       NTBETHDEBUG( "Invoking tx_ack_interrupt callback\n");
       pdev->ptx_ack_int_callback(pdev->tx_ack_int_callback_ref);
       
     }
     if(doorbell_value & (0x4 << pdev->rx_int_doorbell_num))
     {
       NTBETHDEBUG( "Invoking ping_interrupt callback\n");
       if(pdev->ping_int_callback)
         pdev->ping_int_callback(pdev->ping_int_callback_ref);
     }
     if(doorbell_value & (0x8 << pdev->rx_int_doorbell_num))
     {
       NTBETHDEBUG( "Invoking ping_ack_interrupt callback\n");
       if(pdev->ping_ack_int_callback)
         pdev->ping_ack_int_callback(pdev->ping_ack_int_callback_ref);
     }
     if(doorbell_value & (0x10 << pdev->rx_int_doorbell_num))
     {
       NTBETHDEBUG( "Invoking close int callback\n");
       if(pdev->close_int_callback)
         pdev->close_int_callback(pdev->ping_ack_int_callback_ref);
     }
}
void ntbeth_bar45_callback(ntb_client_handle_t handle, int16_t doorbell_value, struct scratchpad_registers pad)
{
 
     NTBETHDEBUG( "Invoked ntbeth_bar45_callback\n");
}

int ntbdev_init(struct ntbeth_ntbdev *pdev, int bar23_size, int bar45_size, int rx_int_doorbell_num)
{
  unsigned short intbits;
  unsigned int temp_value;
  pdev->barinfo[0].bar_size = bar23_size;
  pdev->barinfo[1].bar_size = bar45_size;
  pdev->rx_int_doorbell_num = rx_int_doorbell_num;

 gntbdev = pdev; // remember pointer to ntbdev here because in NTB callbacks we do not have a facility to get the context back. Thus we use global variable 
  // obtain handle to the ntb api functions
  ntb_get_api(&pdev->funcs);

  // obtain ntb device structure
  pdev->ntbdevice = pdev->funcs.ntb_get_device(PROC_0);

  // register for Bar23 
  pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle = pdev->funcs.ntb_register_client(NTB_BAR_23, ntbeth_bar23_callback,PROC_0,NULL); 
  if(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle == -EPERM)
  {
      return (NTBETH_FAIL);
  }
  // register for Bar45
  pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle = pdev->funcs.ntb_register_client(NTB_BAR_45, ntbeth_bar45_callback,PROC_0,NULL); 
  if(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle == -EPERM)
  {
      return (NTBETH_FAIL);
  }
  
  // Allocate memory and program  Sec Bar 23 Xlate register

  // convert to bus address
  if(ntbdev_alloc_dma_memory(&pdev->ntbdevice->dev->dev, &pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr,pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size, &pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr))
  {
        return (NTBETH_FAIL);
  } 

  // Program Secondary BAR23 Xlate Register  
  ntbdev_write_to_sec_bar23_xlate(pdev, (unsigned long long)pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr);
  // Allocate memory and program  Sec Bar 45 Xlate register
  if(ntbdev_alloc_dma_memory(&pdev->ntbdevice->dev->dev,&pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr,pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size, &pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr))
  {
        return (NTBETH_FAIL);
  } 
  // Program Secondary BAR45 Xlate Register 
  ntbdev_write_to_sec_bar45_xlate(pdev, (unsigned long long)pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr); 

 intbits = (0x1F << rx_int_doorbell_num); //lower bit 0 for rx_int,  bit 1 for tx_ack_int, bit 2 for ping bit 3 for ping_ack;

 intbits |= 0x4000; // WC flush ack interrupt
  
// set policy:
  //we are interested in rx_int_doorbell, txack door bell and link change interrupts only. we set the policy using bar23 client handle 

 pdev->funcs.ntb_set_policy(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, 0, intbits, 0, 0, 0);

// writing jiffies to scratchpad register to manual check of link
  temp_value = jiffies; 
  *(unsigned int *)((char *)pdev->ntbdevice->mm_regs + 0x100 ) = temp_value ;
  NTBETHDEBUG( "Jiffies value written to SPAD0 0x%x\n", temp_value);

  // we start with no door bell interrupts pending so clear the door bells bits 
  *(unsigned int *)((char *)pdev->ntbdevice->mm_regs + 0x60 ) = 0xffff ;
  NTBETHDEBUG( "door bell  value read  0x%x\n", *(unsigned int *)((char *)pdev->ntbdevice->mm_regs + 0x60 ) );
 
  temp_value = *(unsigned int *)((char *)pdev->ntbdevice->mm_regs + 0x80 );
 
  NTBETHDEBUG( "Jiffies value read from SPAD0 0x%x\n", temp_value);
  return 0;
}

void ntbdev_write_to_sec_bar45_xlate(struct ntbeth_ntbdev *pdev, unsigned long long memory_loc)
{
  *(unsigned long long *)((char *)pdev->ntbdevice->mm_regs + 0x38) = memory_loc;
}
void ntbdev_write_to_sec_bar23_xlate(struct ntbeth_ntbdev *pdev, unsigned long long memory_loc)
{

  *(unsigned long long *)((char *)pdev->ntbdevice->mm_regs + 0x30) = memory_loc;
}

int ntbdev_free_dma_memory(struct device *pdev, int size, void *virt_addr, dma_addr_t bus_address)
{
    dma_free_coherent(pdev, size, virt_addr, bus_address);
    return (NTBETH_SUCCESS);
}

int ntbdev_alloc_dma_memory(struct device *pdev, void **virt_addr, int size, dma_addr_t *bus_address)
{
   *virt_addr = (void *)dma_alloc_coherent(pdev, size, bus_address, GFP_KERNEL);
    if(*virt_addr == NULL)
    {
      NTBETHDEBUG("ntbdev_alloc_dma_memory: Failed to allocate %d bytes of memory\n", size);
      return (NTBETH_FAIL);
    }
    else
    {
      NTBETHDEBUG("ntbdev_alloc_dma_memory: successfully allocated %d bytes of memory\n", size);
      return (NTBETH_SUCCESS);
    }
}
// This routine needs to be modified for the alignment
int ntbdev_get_bus_address(struct device *pdev, void *virt_addr, int size, dma_addr_t *bus_address)
{
    *bus_address = dma_map_single(pdev, virt_addr, size, DMA_BIDIRECTIONAL); 
    if(*bus_address == 0)
    {
      return (NTBETH_FAIL);
    }
    else
    {
     NTBETHDEBUG(" Bus Address 0x%Lx\n", (unsigned long long)*bus_address);
     return (NTBETH_SUCCESS);
    }
}

int ntbdev_cleanup(struct ntbeth_ntbdev *pdev)
{
  // obtain handle to the ntb api functions
  ntb_get_api(&pdev->funcs);
  // register for Bar23 
 pdev->funcs.ntb_unregister_client(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle);
  // unregister for Bar45
 pdev->funcs.ntb_unregister_client(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle);
  // free bar23 memory 

  if(ntbdev_free_dma_memory(&pdev->ntbdevice->dev->dev,pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size, pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr, pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr))
  {
    printk("Unable to free dma memory\n");
  }
  if(ntbdev_free_dma_memory(&pdev->ntbdevice->dev->dev,pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size, pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr, pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr))
  {
    printk("Unable to free dma memory\n");
  }
  return 0;
}

// we send interrupt on the rx_int doorbell. but also send wc flush along with it, for validation purposes. we maintain count of rx_int_doorbells that we sent and we expect equal number of acks for the doorbell at rx_int_doorbell_num +1. And we also keep track of how many WC FLUSH are acked. we make sure that we only hit WC Flush if the bit is 0. Otherwise we do not hit WC Flush bit.
void ntbdev_send_packet_txed_interrupt(struct ntbeth_ntbdev *pdev)
{
  unsigned short doorbell_value;
  void *bar01_virt_addr;

  bar01_virt_addr =  pdev->ntbdevice->mm_regs;
  doorbell_value = 1;

  // set bit corresponding the door bell we are using 
   doorbell_value <<= pdev->rx_int_doorbell_num;
  
// writing to door bell can offloaded to CB3 DMA engine 
  // write to door bell 
  pdev->funcs.ntb_write_doorbell(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, doorbell_value);
  pdev->pktsent_count++;
#if 0
  // write to WCCNTRL register to flush out the cache
  if(!((*(unsigned int *)((char *)bar01_virt_addr + WCCNTRL_REG_OFFSET)) & 1))
  {
     // flush is not in progress so request new flush 
  *(unsigned int *)((char *)bar01_virt_addr + WCCNTRL_REG_OFFSET) = 1;
      pdev->wc_flush_req_count++; 
  } 
#endif
  NTBETHDEBUG("Sent txed  Interrupt to Remote\n");
}

int ntbdev_subscribe_to_lnkchg_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
    return 0;
} 
int ntbdev_subscribe_to_ping_ack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
  pdev->ping_ack_int_callback = pcallback;
  pdev->ping_ack_int_callback_ref = pref;
    return 0;
} 
int ntbdev_subscribe_to_close_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
  pdev->close_int_callback = pcallback;
  pdev->close_int_callback_ref = pref;
  return 0;
} 
int ntbdev_subscribe_to_ping_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
  pdev->ping_int_callback = pcallback;
  pdev->ping_int_callback_ref = pref;
    return 0;
} 
int ntbdev_subscribe_to_txack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
  
  pdev->ptx_ack_int_callback = pcallback;
  pdev->tx_ack_int_callback_ref = pref;
  return 0;
} 
int ntbdev_subscribe_to_rx_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
  pdev->prx_int_callback = pcallback;
  pdev->rx_int_callback_ref = pref;
  return 0;
} 
void* ntbdev_get_bar23_local_memory(struct ntbeth_ntbdev *pdev)
{
  return(pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr);
}
void* ntbdev_get_bar45_local_memory(struct ntbeth_ntbdev *pdev)
{

  return(pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr);
}
void* ntbdev_get_bar23_value(struct ntbeth_ntbdev *pdev)
{

  return(pdev->ntbdevice->pci_bar_23_virt);
}
void* ntbdev_get_bar45_value(struct ntbeth_ntbdev *pdev)
{

  return(pdev->ntbdevice->pci_bar_23_virt);
}

void ntbdev_send_packet_transfer_ack_interrupt(struct ntbeth_ntbdev *pdev)
{
  unsigned short doorbell_value;
  void *bar01_virt_addr;

  bar01_virt_addr =  pdev->ntbdevice->mm_regs;
  doorbell_value = 0x2;

  // set bit corresponding the door bell we are using  rxint db num + 1
   doorbell_value <<= (pdev->rx_int_doorbell_num);
  
// writing to door bell can offloaded to CB3 DMA engine 
  // write to door bell 
  pdev->funcs.ntb_write_doorbell(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, doorbell_value);
  NTBETHDEBUG("Sent tx_ack  Interrupt to Remote\n");
}

void ntbdev_send_ping_doorbell_interrupt(struct ntbeth_ntbdev *pdev)
{
  unsigned short doorbell_value;
  void *bar01_virt_addr;

  bar01_virt_addr =  pdev->ntbdevice->mm_regs;
  doorbell_value = 0x4;

  // set bit corresponding the door bell we are using 
   doorbell_value <<= pdev->rx_int_doorbell_num;
  
  // writing to door bell can be offloaded to CB3 DMA engine 
  // write to door bell 
  pdev->funcs.ntb_write_doorbell(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, doorbell_value);
  NTBETHDEBUG("Sent Ping Interrupt to Remote\n");
}
void ntbdev_send_ping_ack_doorbell_interrupt(struct ntbeth_ntbdev *pdev)
{
  unsigned short doorbell_value;
  void *bar01_virt_addr;

  bar01_virt_addr =  pdev->ntbdevice->mm_regs;
  doorbell_value = 0x8;

  // set bit corresponding the door bell we are using 
   doorbell_value <<= pdev->rx_int_doorbell_num;
  
  // writing to door bell can be offloaded to CB3 DMA engine 
  // write to door bell 
  pdev->funcs.ntb_write_doorbell(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, doorbell_value);
  NTBETHDEBUG("Sent Ping Ack Interrupt to Remote\n");
}
void ntbdev_send_close_interrupt(struct ntbeth_ntbdev *pdev)
{
  unsigned short doorbell_value;
  void *bar01_virt_addr;

  bar01_virt_addr =  pdev->ntbdevice->mm_regs;
  doorbell_value = 0x10;

  // set bit corresponding the door bell we are using 
   doorbell_value <<= pdev->rx_int_doorbell_num;
  
  // writing to door bell can be offloaded to CB3 DMA engine 
  // write to door bell 
  pdev->funcs.ntb_write_doorbell(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, doorbell_value);
  NTBETHDEBUG("Sent close  Interrupt to Remote\n");
}

void ntbdev_mask_doorbell_interrupts(struct ntbeth_ntbdev *pdev)
{
  *(unsigned short *)((char *)pdev->ntbdevice->mm_regs + 0x62 ) = 0xFFFF;
  printk("read door bell mask value 0x%x\n", *(unsigned short *)((char *)pdev->ntbdevice->mm_regs + 0x62 )); ;
}
void ntbdev_unmask_doorbell_interrupts(struct ntbeth_ntbdev *pdev)
{
  *(unsigned short *)((char *)pdev->ntbdevice->mm_regs + 0x62 ) = 0x0;
  printk("read door bell mask value 0x%x\n", *(unsigned short *)((char *)pdev->ntbdevice->mm_regs + 0x62 )); ;
}
