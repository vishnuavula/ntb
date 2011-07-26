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

#ifndef NTBDEV_H
#define NTBDEV_H

#include "../common/ntb_main.h"
#include "ntbethconf.h"

// some of the register offsets are defined here, though when NTB API is complete, these offsets may not be required

#define SBAR2XLAT_REG_OFFSET 0x30 
#define SBAR4XLAT_REG_OFFSET 0x38 
#define PDOORBELL_REG_OFFSET 0x60
#define PDBMSK_REG_OFFSET 0x62
#define SDOORBELL_REG_OFFSET 0x64
#define SDBMSK_REG_OFFSET 0x66
#define WCCNTRL_REG_OFFSET 0xE0
#define B2BDOORBELL_REG_OFFSET 0x140


#define NTBETH_BAR23INFO_INDEX 0
#define NTBETH_BAR45INFO_INDEX 1

struct ntbeth_ntb_bar_client_info
{
 ntb_client_handle_t handle;
 int bar_size;
 void *local_memory_virt_addr;
 dma_addr_t local_memory_dma_addr;
 struct ntb_client_data client_data;
};

// structure that maintains relational information between NTB API and NTBETH Driver. It also maintains some statistics counters that are used for debugging.
struct ntbeth_ntbdev
{
 struct ntb_api_export funcs;
 struct ntbeth_ntb_bar_client_info barinfo[2]; 
 int rx_int_doorbell_num;
 struct ntb_device *ntbdevice;
 int wc_flush_req_count;
 int wc_flush_ack_count;
 int pktsent_count;
 int pktack_count;
 void (*prx_int_callback)(void *);
 void *rx_int_callback_ref;
 void (*ptx_ack_int_callback)(void *);
 void *tx_ack_int_callback_ref;
 void (*ping_ack_int_callback)(void *);
 void *ping_ack_int_callback_ref;
 void (*ping_int_callback)(void *);
 void *ping_int_callback_ref;
 void (*close_int_callback)(void *);
 void *close_int_callback_ref;
 spinlock_t db_lock; // one has to obtain this lock for every door bell access, serializing access to door bell. also insert delay of 10 usec after every door bell
};

// door bell assignments are as follows relatively rx_int_doorbell position
// bit 0 -- tx_int  //  (local-> remote )indicates  packet is queued 
// bit 1 -- tx_ack_int  // (remote->local)indicates that recieved ack for the sent packet
// bit 2 -- ping int  // (local-->remote)   
// bit 3 -- ping_ack_int   // (remote -->local)
// bit 4 -- close_int   // (local -->remote)
 

// currently NTB API does not take a context pointer that will be returned in the callback when the door bell interrupts happen. Because of this we need to store the context in a global pointer that is accessible from the callback invoked in taskelet context.

extern struct ntbeth_ntbdev *gntbdev;


int ntbdev_init(struct ntbeth_ntbdev *pdev, int bar23_size, int bar45_size, int rx_int_doorbell_num);
int ntbdev_cleanup(struct ntbeth_ntbdev *pdev);


int ntbdev_subscribe_to_rx_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);
int ntbdev_subscribe_to_txack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);
int ntbdev_subscribe_to_lnkchg_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);
int ntbdev_subscribe_to_ping_ack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);
int ntbdev_subscribe_to_ping_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);
int ntbdev_subscribe_to_close_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref);


void ntbdev_write_to_sec_bar45_xlate(struct ntbeth_ntbdev *pdev, unsigned long long memory_loc);
void ntbdev_write_to_sec_bar23_xlate(struct ntbeth_ntbdev *pdev, unsigned long long memory_loc);

void* ntbdev_get_bar23_local_memory(struct ntbeth_ntbdev *pdev);
void* ntbdev_get_bar45_local_memory(struct ntbeth_ntbdev *pdev);
void* ntbdev_get_bar45_value(struct ntbeth_ntbdev *pdev);
void* ntbdev_get_bar23_value(struct ntbeth_ntbdev *pdev);

int ntbdev_get_bus_address(struct device *, void *virt_addr, int size, dma_addr_t *bus_address);

int ntbdev_free_dma_memory(struct device *pdev, int size, void *virt_addr, dma_addr_t bus_address);
int ntbdev_alloc_dma_memory(struct device *pdev, void **virt_addr, int size, dma_addr_t *bus_address);

void ntbdev_send_packet_txed_interrupt(struct ntbeth_ntbdev *pdev);
void ntbdev_send_packet_transfer_ack_interrupt(struct ntbeth_ntbdev *pdev);
void ntbdev_send_ping_ack_doorbell_interrupt(struct ntbeth_ntbdev *pdev);
void ntbdev_send_ping_doorbell_interrupt(struct ntbeth_ntbdev *pdev);
void ntbdev_send_close_interrupt(struct ntbeth_ntbdev *pdev);

void ntbdev_unmask_doorbell_interrupts(struct ntbeth_ntbdev *pdev);
void ntbdev_mask_doorbell_interrupts(struct ntbeth_ntbdev *pdev);
void ntbdev_send_doorbell(struct ntbeth_ntbdev *pdev, unsigned short doorbell_value);

#endif 
