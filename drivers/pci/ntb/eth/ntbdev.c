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

#include "ntbdev.h"

struct ntbeth_ntbdev *gntbdev;

void ntbeth_bar23_callback(ntb_client_handle_t handle, int16_t doorbell_value, struct scratchpad_registers pad)
{
	struct ntbeth_ntbdev *pdev = gntbdev;
	NTBETHDEBUG( "Entered ntbeth_bar23_callback db val 0x%x\n",doorbell_value);
	if (doorbell_value & 0x8000) {
		NTBETHDEBUG( "Link Change Door bEll set \n");
	}
	if (doorbell_value & (1<< pdev->rx_int_doorbell_num)) {
		NTBETHDEBUG( "Invoking rx_interrupt callback\n");
		if(pdev->prx_int_callback)
			pdev->prx_int_callback(pdev->rx_int_callback_ref);
	}
	if (doorbell_value & (0x2 << pdev->rx_int_doorbell_num)) {
		NTBETHDEBUG( "Invoking tx_ack_interrupt callback\n");
		pdev->ptx_ack_int_callback(pdev->tx_ack_int_callback_ref);
	}
	if (doorbell_value & (0x4 << pdev->rx_int_doorbell_num)) {
		NTBETHDEBUG( "Invoking ping_interrupt callback\n");
		if(pdev->ping_int_callback)
			pdev->ping_int_callback(pdev->ping_int_callback_ref);
	}
	if (doorbell_value & (0x8 << pdev->rx_int_doorbell_num)) {
	NTBETHDEBUG( "Invoking ping_ack_interrupt callback\n");
	if(pdev->ping_ack_int_callback)
		pdev->ping_ack_int_callback(pdev->ping_ack_int_callback_ref);
	}
	if (doorbell_value & (0x10 << pdev->rx_int_doorbell_num)) {
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
	spin_lock_init(&pdev->db_lock);
	gntbdev = pdev; // remember pointer to ntbdev here because in NTB callbacks we do not have a facility to get the context back. Thus we use global variable
	ntb_get_api(&pdev->funcs);
	if(pdev->funcs.ntb_get_number_devices() < 1) {
		printk("ERROR: NO NTB-NTB DEVICES found\n");
		return (NTBETH_FAIL);
	}

	pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle = pdev->funcs.ntb_register_client(NTB_BAR_23, ntbeth_bar23_callback,NTBDEV_D3_F0,NULL);
	if(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle == -EPERM) {
		return (NTBETH_FAIL);
	}
	pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_pci_address =  pdev->funcs.ntb_get_bar_address(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, NTB_BAR_23);
	pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_virt_address =  ioremap(pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_pci_address, bar23_size);

	pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle = pdev->funcs.ntb_register_client(NTB_BAR_45, ntbeth_bar45_callback,NTBDEV_D3_F0,NULL);
	if(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle == -EPERM) {
		return (NTBETH_FAIL);
	}
	pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_pci_address =  pdev->funcs.ntb_get_bar_address(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle, NTB_BAR_45);
	pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_virt_address =  ioremap(pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_pci_address, bar45_size);
	if(pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_virt_address == NULL) {
		return(NTBETH_FAIL);
	}
	if(ntbdev_alloc_dma_memory(NULL, &pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr,pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size, &pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr))
		return (NTBETH_FAIL);
	pdev->funcs.ntb_write_translate(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr);
	if(ntbdev_alloc_dma_memory(NULL,&pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr,pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size, &pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr))
		return (NTBETH_FAIL);
	pdev->funcs.ntb_write_translate(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle, pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr);
	intbits = (0x1F << rx_int_doorbell_num);
	/*lower bit 0 for rx_int,  bit 1 for tx_ack_int, bit 2 for ping bit 3 for ping_ack, bit 4 for close */
	/* intbits |= 0x4000; // WC flush ack interrupt */

	/*we are interested in rx_int_doorbell, txack door bell and link change interrupts only.
	* we set the policy using bar23 client handle
	*/
	if(pdev->funcs.ntb_add_policy(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, 0, intbits, 0, 0))
		return (NTBETH_FAIL);
	temp_value = jiffies;
	pdev->funcs.ntb_write_scratch_pad_one(0,temp_value, pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle);
	NTBETHDEBUG( "Jiffies value written to SPAD0 0x%x\n", temp_value);
	pdev->funcs.ntb_read_scratch_pad_one(0,&temp_value, pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle);
	NTBETHDEBUG( "Jiffies value read from SPAD0 0x%x\n", temp_value);

	/* set BAR45 to point to remote side secondary BAR01 */
	/* this is required to send door bell interrupts directly to the remote side */
	pdev->funcs.ntb_write_remote_translate(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle, NTB_BAR_45,0);
	return NTBETH_SUCCESS;
}

int ntbdev_free_dma_memory(struct device *pdev, int size, void *virt_addr, dma_addr_t bus_address)
{
	dma_free_coherent(pdev, size, virt_addr, bus_address);
	return NTBETH_SUCCESS;
}

int ntbdev_alloc_dma_memory(struct device *pdev, void **virt_addr, int size, dma_addr_t *bus_address)
{
	*virt_addr = (void *)dma_alloc_coherent(pdev, size, bus_address, GFP_KERNEL);
	if (*virt_addr == NULL) {
		NTBETHDEBUG("ntbdev_alloc_dma_memory: Failed to allocate %d bytes of memory\n", size);
		return (NTBETH_FAIL);
	}
	else {
		NTBETHDEBUG("ntbdev_alloc_dma_memory: successfully allocated %d bytes of memory\n", size);
		return (NTBETH_SUCCESS);
	}
}

int ntbdev_get_bus_address(struct device *pdev, void *virt_addr, int size, dma_addr_t *bus_address)
{
	*bus_address = dma_map_single(pdev, virt_addr, size, DMA_BIDIRECTIONAL);
	if(*bus_address == 0) {
		return (NTBETH_FAIL);
	} else {
		NTBETHDEBUG(" Bus Address 0x%Lx\n", (unsigned long long)*bus_address);
		return (NTBETH_SUCCESS);
	}
}

int ntbdev_cleanup(struct ntbeth_ntbdev *pdev)
{
	ntb_get_api(&pdev->funcs);
	pdev->funcs.ntb_unregister_client(pdev->barinfo[NTBETH_BAR23INFO_INDEX].handle);
	pdev->funcs.ntb_unregister_client(pdev->barinfo[NTBETH_BAR45INFO_INDEX].handle);
	if(ntbdev_free_dma_memory(NULL,pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size, pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr, pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr)) {
	printk("Unable to free dma memory\n");
	}
	if(ntbdev_free_dma_memory(NULL,pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size, pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr, pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr)) {
	printk("Unable to free dma memory\n");
	}
	return 0;
}
void ntbdev_send_doorbell(struct ntbeth_ntbdev *pdev, unsigned short doorbell_value)
{
   *(unsigned short *)((char *)(pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_virt_address) + 0x60) = doorbell_value;
   return;
}
/* we send interrupt on the rx_int doorbell. but also send wc flush along with it,
* for validation purposes. we maintain count of rx_int_doorbells that we sent and
* we expect equal number of acks for the doorbell at rx_int_doorbell_num +1. And
* we also keep track of how many WC FLUSH are acked. we make sure that we only
* hit WC Flush if the bit is 0. Otherwise we do not hit WC Flush bit.
*/
void ntbdev_send_packet_txed_interrupt(struct ntbeth_ntbdev *pdev)
{
	unsigned short doorbell_value;
	doorbell_value = 1;

	/* set bit corresponding the door bell we are using  */
	doorbell_value <<= pdev->rx_int_doorbell_num;
  	ntbdev_send_doorbell(pdev, doorbell_value);
  	NTBETHDEBUG("Sent txed  Interrupt to Remote\n");
	return;
}

int ntbdev_subscribe_to_lnkchg_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
    return NTBETH_SUCCESS;
}
int ntbdev_subscribe_to_ping_ack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
	pdev->ping_ack_int_callback = pcallback;
	pdev->ping_ack_int_callback_ref = pref;
	return NTBETH_SUCCESS;
}
int ntbdev_subscribe_to_close_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
	pdev->close_int_callback = pcallback;
	pdev->close_int_callback_ref = pref;
	return NTBETH_SUCCESS;
}
int ntbdev_subscribe_to_ping_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
	pdev->ping_int_callback = pcallback;
	pdev->ping_int_callback_ref = pref;
	return NTBETH_SUCCESS;
}
int ntbdev_subscribe_to_txack_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
	pdev->ptx_ack_int_callback = pcallback;
	pdev->tx_ack_int_callback_ref = pref;
	return NTBETH_SUCCESS;
}
int ntbdev_subscribe_to_rx_int(struct ntbeth_ntbdev *pdev, void (*pcallback)(void *), void *pref)
{
	pdev->prx_int_callback = pcallback;
	pdev->rx_int_callback_ref = pref;
	return NTBETH_SUCCESS;
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
	return (pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_virt_address);
}

void* ntbdev_get_bar45_value(struct ntbeth_ntbdev *pdev)
{
	return (pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_virt_address);
}

void ntbdev_send_packet_transfer_ack_interrupt(struct ntbeth_ntbdev *pdev)
{
	unsigned short doorbell_value;
	doorbell_value = 0x2;
	/* set bit corresponding the door bell we are using  rxint db num + 1 */
	doorbell_value <<= (pdev->rx_int_doorbell_num);
	ntbdev_send_doorbell(pdev, doorbell_value);
	NTBETHDEBUG("Sent tx_ack  Interrupt to Remote\n");
	return;
}

void ntbdev_send_ping_doorbell_interrupt(struct ntbeth_ntbdev *pdev)
{
	unsigned short doorbell_value;
	doorbell_value = 0x4;
	/* set bit corresponding the door bell we are using  */
	doorbell_value <<= pdev->rx_int_doorbell_num;
	ntbdev_send_doorbell(pdev, doorbell_value);
	NTBETHDEBUG("Sent Ping Interrupt to Remote\n");
	return;
}
void ntbdev_send_ping_ack_doorbell_interrupt(struct ntbeth_ntbdev *pdev)
{
	unsigned short doorbell_value;
	doorbell_value = 0x8;
	doorbell_value <<= pdev->rx_int_doorbell_num;
	ntbdev_send_doorbell(pdev, doorbell_value);
	NTBETHDEBUG("Sent Ping Ack Interrupt to Remote\n");
}
void ntbdev_send_close_interrupt(struct ntbeth_ntbdev *pdev)
{
	unsigned short doorbell_value;
	doorbell_value = 0x10;
	doorbell_value <<= pdev->rx_int_doorbell_num;
	ntbdev_send_doorbell(pdev, doorbell_value);
	NTBETHDEBUG("Sent close  Interrupt to Remote\n");
}
int ntbdev_get_bus_address_for_local_buffers(struct ntbeth_ntbdev *pdev, void *virt_addr, int size, dma_addr_t *bus_address)
{
	unsigned long offset;
	void *base_virt = pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_virt_addr;
	dma_addr_t base_pci = pdev->barinfo[NTBETH_BAR23INFO_INDEX].local_memory_dma_addr;
	int bar_size = pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size;

	if (((unsigned long long)base_virt <= (unsigned long long)virt_addr) && ((bar_size + (char *) base_virt) > (char *)virt_addr)) {
	// given virt address is with in the range of bar23
		offset = virt_addr - base_virt;
		*bus_address =  (dma_addr_t)((char *)base_pci + offset);
	} else {
		base_virt = pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_virt_addr;
		base_pci = pdev->barinfo[NTBETH_BAR45INFO_INDEX].local_memory_dma_addr;
		bar_size = pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size;
		if (((unsigned long long)base_virt <= (unsigned long long)virt_addr) && ((bar_size + (char *) base_virt) > (char *)virt_addr)) {
		/* given virt address is with in the range of bar45 */
			offset = virt_addr -base_virt;
			*bus_address =  (dma_addr_t)((char *)base_pci + offset);
		} else {
			NTBETHDEBUG(" FAILED to get  Bus Address for local memory 0x%Lx\n", (unsigned long long)*bus_address);
			return (NTBETH_FAIL);
		}
	}
	NTBETHDEBUG(" Bus Address for local buffers 0x%Lx\n", (unsigned long long)*bus_address);
	return (NTBETH_SUCCESS);
}

// This routine needs to be modified for the alignment
int ntbdev_get_bus_address_for_remote_buffers(struct ntbeth_ntbdev *pdev, void *virt_addr, int size, dma_addr_t *bus_address)
{
	unsigned long offset;
	void *bar_base_virt = pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_virt_address;
	int bar_size = pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_size;
	dma_addr_t  bar_base_pci = pdev->barinfo[NTBETH_BAR23INFO_INDEX].bar_pci_address;
	if (((unsigned long long)bar_base_virt <= (unsigned long long)virt_addr) && ((bar_size + (char *) bar_base_virt) > (char *)virt_addr)) {
		// given virt address is with in the range of bar23
		offset = virt_addr - bar_base_virt;
		*bus_address =  (dma_addr_t)((char *)bar_base_pci + offset);
	} else {
		bar_base_virt = pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_virt_address;
		bar_size = pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_size;
		bar_base_pci = pdev->barinfo[NTBETH_BAR45INFO_INDEX].bar_pci_address;
		if (((unsigned long long)bar_base_virt <= (unsigned long long)virt_addr) && ((bar_size + (char *) bar_base_virt) > (char *)virt_addr)) {
			// given virt address is with in the range of the bar45
			offset = virt_addr - bar_base_virt;
			*bus_address = (dma_addr_t) ((char *)bar_base_pci + offset);
		} else  {
			NTBETHDEBUG(" FAILED to get  Bus Address for remote memory 0x%Lx\n", (unsigned long long)*bus_address);
			return (NTBETH_FAIL);
      		}
	}
	NTBETHDEBUG(" Bus Address for remote buffers 0x%Lx\n", (unsigned long long)*bus_address);
	return (NTBETH_SUCCESS);
}
