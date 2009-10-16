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

/* This program implements network driver over NTB PCIE Link */

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
 dma_addr_t bar_pci_address;
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
int ntbdev_get_bus_address_for_local_buffers(struct ntbeth_ntbdev *pdev, void *virt_addr, int size, dma_addr_t *bus_address);
int ntbdev_get_bus_address_for_remote_buffers(struct ntbeth_ntbdev *pdev, void *virt_addr, int size, dma_addr_t *bus_address);


#endif 
