/*****************************************************************************
 * %LICENSE_DUAL%
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007,2008,2009 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007,2008,2009 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: Embedded.Release.L.0.5.1-2
 ****************************************************************************/


#include "ntb_main.h"


static uint32_t g_number_devices;
static struct ntb_device g_ntb[MAX_DEVICES];

uint16_t ntb_get_bdf(unsigned char bus, uint32_t devfn)
{
	uint16_t bdf = bus;
	bdf = bdf << SHIFT_8;
	bdf = bdf | devfn;

	return bdf;

}


/*****************************************************************************
 * Description
 * Increments number of devices enumerated.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: NONE
 ****************************************************************************/

void ntb_increment_number_devices(void)
{
	g_number_devices++;
}

/*****************************************************************************
 * Description
 * Increments number of devices enumerated.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: NONE
 ****************************************************************************/

void ntb_initialize_number_devices(void)
{
	g_number_devices = 0;
}
/*****************************************************************************
 * Description
 * Returns number of devices enumerated.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: >= 0
 ****************************************************************************/

uint32_t ntb_get_number_devices(void)
{
	return g_number_devices;
}

/*****************************************************************************
 * Description
 * Checks for validity of a handle against valid combinations.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: VALID successful, INVALID unsuccessful
 ****************************************************************************/
int32_t ntb_check_handle_validity(ntb_client_handle_t handle)
{

	int32_t i = 0;

	if (handle == NTB_UNUSED)
		return INVALID;

	for (i = 0 ; i < MAX_DEVICES; i++) {
		if (g_ntb[i].client_list.clients[NTB_CLIENT_23].handle
		== handle)
			return VALID;
		else if (g_ntb[i].client_list.clients[NTB_CLIENT_45].handle
				== handle)
			return VALID;
	}

	return INVALID;
}


/*****************************************************************************
 * Description
 * Returns client list per associated BUS DEVICE FUNCTION.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: client_list ptr
 ****************************************************************************/
struct ntb_clients *ntb_get_client_list_by_bdf(uint32_t bdf)
{
	uint32_t i = 0;
	while (i < MAX_DEVICES) {
		NTB_DEBUG_PRINT(
		("NTB: Client List BDF %x state %x\n",
		g_ntb[i].bdf, g_ntb[i].device_state));
		if (g_ntb[i].bdf == bdf &&
		g_ntb[i].device_state == ENUMERATED)
			return &g_ntb[i].client_list;
		i++;
	}

	return NULL;
}
/*****************************************************************************
 * Description
 * Returns client list per associated device index.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: client_list ptr
 ****************************************************************************/
struct ntb_clients *ntb_get_client_list(uint32_t index)
{
	if (index >= 0 && index < MAX_DEVICES) {
		if (g_ntb[index].device_state != NOT_ENUMERATED)
			return &g_ntb[index].client_list;
	}

	return NULL;
}


/*****************************************************************************
 * Description
 * Returns client list per associated handle.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: client_list ptr
 ****************************************************************************/
struct ntb_clients *ntb_get_client_list_by_handle(ntb_client_handle_t handle)
{
	int32_t check = ntb_check_handle_validity(handle);
	int32_t i = 0;
	int32_t bdf = handle & LOWER_16;
	if (VALID == check) {
		while (i >= 0 && i < MAX_DEVICES) {

			if (g_ntb[i].bdf == bdf) {
				NTB_DEBUG_PRINT(
				("NTB: Client List %p BDF %x\n",
				&g_ntb[i].client_list, bdf));
					return &g_ntb[i].client_list;
		}
			i++;
		}
	}

	return NULL;
}

/*****************************************************************************
 * Description
 * Returns ntb_device ptr per processor.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: ntb device ptr
 ****************************************************************************/
struct ntb_device *ntb_get_device(uint32_t index)
{
	if (index >= 0 || index < MAX_DEVICES) {
		NTB_DEBUG_PRINT(("NTB: GET DEVICE %p\n", &g_ntb[index]));
		return &g_ntb[index];
	}

	return NULL;
}




/*****************************************************************************
 * Description
 * Returns ntb_device ptr per handle.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: ntb device ptr
 ****************************************************************************/
struct ntb_device *ntb_get_device_by_handle(ntb_client_handle_t handle)
{

	int32_t i = 0;

	for (i = 0 ; i < MAX_DEVICES; i++) {
		NTB_DEBUG_PRINT(("NTB: Device %p\n", &g_ntb[i]));
		if (g_ntb[i].client_list.clients[NTB_CLIENT_23].handle
		== handle)
			return  &g_ntb[i];
		else if (g_ntb[i].client_list.clients[NTB_CLIENT_45].handle
				== handle)
			return  &g_ntb[i];
	}


	return NULL;
}


/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
uint64_t ntb_get_bar_address(ntb_client_handle_t handle,
enum ntb_bar_t bar)
{
	struct ntb_device *device = ntb_get_device_by_handle(handle);
	int32_t check = ntb_check_handle_validity(handle);

	if ((VALID == check) && (bar & handle) != 0) {
		if (device != NULL && (bar == NTB_BAR_23))
			return device->pci_bar[DEVICE_BAR_23];

		if (device != NULL && (bar == NTB_BAR_45))
			return device->pci_bar[DEVICE_BAR_45];
	}
	NTB_DEBUG_PRINT(
	("NTBC: ntb_get_bar_address - wrong handle/bar supplied \n"));
	/* Assumption that 0 is an invalid BAR address */
	return 0;
}

/*****************************************************************************
 * See ntb_main.h
 *****************************************************************************/
int64_t pow_exp(int32_t exponent)
{
	return (1 << exponent);
}

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
void ntb_get_limit_settings(struct pci_dev *dev, enum ntb_bar_t bar,
struct ntb_device *device, int32_t cmd)
{

#ifdef CONFIG_X86_32
	uint32_t base             = 0;
	uint32_t start            = 0;
	uint32_t end              = 0;
	uint32_t len              = 0;

#else
	uint64_t base             = 0;
	uint64_t start            = 0;
	uint64_t end              = 0;
	uint64_t len              = 0;
#endif
	uint8_t len_byte          = 0;

	if (cmd == PRIMARY_CONFIG) {

		NTB_DEBUG_PRINT(("NTB: Get Limit Settings \n"));

		if (bar == NTB_BAR_23) {

			base =
			start = pci_resource_start(dev, PCI_CONFIG_SPACE_23);
			end = pci_resource_end(dev, PCI_CONFIG_SPACE_23);
			len = end - start;

			base = base & ~SHIFT_LOWER;
			device->limit_base_23 = base;
			device->limit_max_23 = len;
#ifdef CONFIG_X86_32
			NTB_DEBUG_PRINT(("NTB: Base Setting %x\n",
			base));
#else
			NTB_DEBUG_PRINT(("NTB: Base Setting %Lx\n",
			base));
#endif

		} else if (bar == NTB_BAR_45) {
			base =
			start = pci_resource_start(dev, PCI_CONFIG_SPACE_45);
			end = pci_resource_end(dev, PCI_CONFIG_SPACE_45);
			len = end - start;

			base = base & ~SHIFT_LOWER;
			device->limit_base_45 = base;
			device->limit_max_45 = len;

#ifdef CONFIG_X86_32
			NTB_DEBUG_PRINT(("NTB: Base Setting %x\n",
			base));
#else
			NTB_DEBUG_PRINT(("NTB: Base Setting %Lx\n",
			base));
#endif
		}


	} else if (cmd == SECONDARY_CONFIG) {
		if (bar == NTB_BAR_23) {

			base = ntb_lib_read_64(device->mm_regs,
			NTB_SECONDARY_BASE_2_OFFSET);

			pci_read_config_byte(dev, NTB_SECONDARY_LIMIT_MAX_23,
			&len_byte);

			len = pow_exp(len_byte);

			base = base & ~SHIFT_LOWER;
			device->secondary_limit_base_23 = base;
			device->secondary_limit_max_23 = len;



		} else if (bar == NTB_BAR_45) {

			base = ntb_lib_read_64(device->mm_regs,
			NTB_SECONDARY_BASE_4_OFFSET);

			pci_read_config_byte(dev, NTB_SECONDARY_LIMIT_MAX_45,
			&len_byte);

			len = pow_exp(len_byte);

			base = base & ~SHIFT_LOWER;
			device->secondary_limit_base_45 = base;
			device->secondary_limit_max_45 = len;

		}

	}

#ifdef CONFIG_X86_32
	NTB_DEBUG_PRINT(("NTB: base %x\n", base));
	NTB_DEBUG_PRINT(("NTB: max limit %x\n", len));
#else
	NTB_DEBUG_PRINT(("NTB: base %Lx\n", base));
	NTB_DEBUG_PRINT(("NTB: max limit %Lx\n", len));
#endif

}

void ntb_write_limit_settings_to_scratchpad(struct ntb_device *device)
{

	uint64_t data_to_write[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int i = 0;

	data_to_write[i] = device->limit_base_23;
	i++;
	data_to_write[i] = device->limit_max_23;
	i++;
	data_to_write[i] = device->limit_base_45;
	i++;
	data_to_write[i] = device->limit_max_45;
	i++;
	data_to_write[i] = device->secondary_limit_base_23;
	i++;
	data_to_write[i] = device->secondary_limit_max_23;
	i++;
	data_to_write[i] = device->secondary_limit_base_45;
	i++;
	data_to_write[i] = device->secondary_limit_max_45;


	ntb_lib_read_32(device->mm_regs,
	device->semaphore_offset);

	for (i = 0; i < NTB_CONFIG_AND_SIZE_REGS; i++)
		ntb_lib_write_64(device->mm_regs,
		device->doorbell_offset + (i * sizeof(uint64_t)),
		data_to_write[i]);


	ntb_lib_write_32(device->mm_regs, device->semaphore_offset,
	SEMAPHORE_ONE_TO_CLEAR);

}
/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
int32_t ntb_write_remote_bar(ntb_client_handle_t handle, enum ntb_bar_t bar,
uint64_t address)
{
	struct ntb_device *device = NULL;

#ifdef CONFIG_X86_32
	uint32_t new_address = address;

#else
	uint64_t new_address = address;

#endif
	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("NTB: SET POLICY- NULL Device Returned\n"));
		return -EINVAL;
	}


	if (bar == NTB_BAR_01)
		ntb_lib_write_64(device->mm_regs,
		NTB_SECONDARY_BASE_0_OFFSET, new_address);
	else if (bar == NTB_BAR_23)
		ntb_lib_write_64(device->mm_regs,
		NTB_SECONDARY_BASE_2_OFFSET, new_address);
	else if (bar == NTB_BAR_45)
		ntb_lib_write_64(device->mm_regs,
		NTB_SECONDARY_BASE_4_OFFSET, new_address);
	else
		return -EINVAL;


	return SUCCESS;
}

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
uint64_t ntb_read_remote_bar(ntb_client_handle_t handle, enum ntb_bar_t bar)
{
	struct ntb_device *device = NULL;

#ifdef CONFIG_X86_32
	uint32_t address = 0;

#else
	uint64_t address = 0;

#endif
	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("NTB: SET POLICY- NULL Device Returned\n"));
		return -EINVAL;
	}

	if (bar == NTB_BAR_01)
		address = ntb_lib_read_64(device->mm_regs,
		NTB_SECONDARY_BASE_0_OFFSET);
	else if (bar == NTB_BAR_23)
		address = ntb_lib_read_64(device->mm_regs,
		NTB_SECONDARY_BASE_2_OFFSET);
	else if (bar == NTB_BAR_45)
		address = ntb_lib_read_64(device->mm_regs,
		NTB_SECONDARY_BASE_4_OFFSET);


	return address;
}
