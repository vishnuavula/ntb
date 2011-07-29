/*****************************************************************************
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007,2008,2009,2010 Intel Corporation. All rights reserved.
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
 *   Copyright(c) 2007,2008,2009, 2010 Intel Corporation. All rights reserved.
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
 *  version: Embedded.Release.L.1.0.0-401
 ****************************************************************************/


#include "ntb_main.h"

/*****************************************************************************
 * Abstract
 * Checks for validity of a handle against a given bar and returns the
 * handle's bar.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: > 0 valid bar, successful, -EPERM (unsuccessful)
 ****************************************************************************/
enum ntb_bar_t ntb_compare_handle_bar(ntb_client_handle_t handle)
{

	if ((handle & NTB_BAR_23) != 0)
		return NTB_BAR_23;
	else if ((handle & NTB_BAR_45) != 0)
		return NTB_BAR_45;

	return -EPERM;
}

/*****************************************************************************
 * Abstract
 * Determines if a given handle is allowed to reset snoop level.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: (VALID) successful or allowed, INVALID (unsuccessful)
 ****************************************************************************/
int32_t ntb_compare_handle_snoop_level(ntb_client_handle_t handle,
uint32_t value)
{

	if (handle & NTB_BAR_45) {
		if ((NTB_23_SEC_TO_PRI_FORCE_SNOOP & value) ||
		(NTB_23_SEC_TO_PRI_FORCE_NO_SNOOP  & value) ||
		(NTB_23_PRI_TO_SEC_FORCE_SNOOP & value) ||
		(NTB_23_PRI_TO_SEC_FORCE_NO_SNOOP & value))
			return INVALID;
	} else if (handle & NTB_BAR_23) {
		if ((NTB_45_SEC_TO_PRI_FORCE_SNOOP & value) ||
		(NTB_45_SEC_TO_PRI_FORCE_NO_SNOOP & value) ||
		(NTB_45_PRI_TO_SEC_FORCE_SNOOP & value) ||
		(NTB_45_PRI_TO_SEC_FORCE_NO_SNOOP & value))
			return INVALID;
	}
	return VALID;
}


/*****************************************************************************
 * Abstract
 * Registers a client, and returns a valid handle.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: (a valid client, positive integer value) successful,
 * (NTB_UNUSED) unsuccessful
 * -EPERM
 * -EINVAL invalid peramter
 ****************************************************************************/
ntb_client_handle_t ntb_register_client(enum ntb_bar_t bar,
ntb_callback_t callback,
uint16_t bdf,
struct ntb_client_data *client_data
)
{
	struct ntb_client *new_client	= NULL;
	ntb_client_handle_t handle	= NTB_UNUSED;
	struct ntb_clients *client_list = NULL;
	uint32_t prefix 		= 0;

	client_list = ntb_get_client_list_by_bdf(bdf);

	NTB_DEBUG_PRINT(("%s Entering ntb_register_client\n", PREFIX_STRING));
	NTB_DEBUG_PRINT(("%s BAR = %x BDF = %x\n", PREFIX_STRING, bar, bdf));

	if (client_list == NULL) {
		NTB_DEBUG_PRINT(("%s NO CLIENT LIST\n", PREFIX_STRING));
		return -EINVAL;
	}
	NTB_DEBUG_PRINT(("%s Client List = %p\n", PREFIX_STRING, client_list));

	spin_lock(&client_list->client_list_lock);

	if (bar == NTB_BAR_23) {
		if (client_list->clients[NTB_CLIENT_23].handle == NTB_UNUSED) {
			prefix = client_list->clients[NTB_CLIENT_23].bdf;
			handle = bar | prefix;
			client_list->clients[NTB_CLIENT_23].handle = handle;
			client_list->clients[NTB_CLIENT_23].callback =
			callback;
			new_client =
			&client_list->clients[NTB_CLIENT_23];
			client_list->number_used++;
			NTB_DEBUG_PRINT(
			("%s allocating 23 BAR HANDLE %x\n", PREFIX_STRING,
			handle));
		} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(("%s 23 BAR in use\n", PREFIX_STRING));
			return handle;
		}
	} else if (bar == NTB_BAR_45) {
		if (client_list->clients[NTB_CLIENT_45].handle == NTB_UNUSED) {
			prefix = client_list->clients[NTB_CLIENT_45].bdf;
			handle = bar | prefix;
			client_list->clients[NTB_CLIENT_45].handle = handle;
			client_list->clients[NTB_CLIENT_45].callback =
			callback;
			new_client =
			&client_list->clients[NTB_CLIENT_45];
			client_list->number_used++;
			NTB_DEBUG_PRINT(
			("%s allocating 45 BAR HANDLE %x\n", PREFIX_STRING,
			handle));
	} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(("%s 23 BAR in use\n", PREFIX_STRING));
			return handle;
		}
	} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(
			("%s BAD BAR value\n", PREFIX_STRING));
			return handle;
	}

	spin_unlock(&client_list->client_list_lock);

	if (new_client == NULL)
		return -EPERM;

	if (handle > NTB_UNUSED && client_data != NULL)
		ntb_write_client_data(handle, new_client, client_data);
	NTB_DEBUG_PRINT(
	("%s Exiting ntb_register_client\n", PREFIX_STRING));
	return handle;

}
/*****************************************************************************
 * Abstract
 * Writes client data associated with a request to register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: None
 ****************************************************************************/
void ntb_write_client_data(ntb_client_handle_t handle,
struct ntb_client *client,
struct ntb_client_data *client_data)
{

	if (client != NULL) {
		client->client_data.limit = client_data->limit;
		client->client_data.translate_address =
		client_data->translate_address;

		ntb_write_translate(handle, client_data->translate_address);
		ntb_write_limit(handle, client_data->limit);
	}

}
/*****************************************************************************
 * Abstract
 * Unregisters a client associated with a valid handle.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful
 * -EPERM
 * -EINVAL invalid peramter
 ****************************************************************************/
int32_t ntb_unregister_client(ntb_client_handle_t handle)
{
	struct ntb_client *client	= NULL;
	struct ntb_clients *client_list = NULL;
	struct ntb_device *device	= NULL;

	NTB_DEBUG_PRINT(
	("%s Entering ntb_unregister_client \n", PREFIX_STRING));
	device = ntb_get_device_by_handle(handle);

	if (device == NULL) {
		NTB_DEBUG_PRINT(
		("%s ERROR: DEVICE IS NULL\n", PREFIX_STRING));
		return -EINVAL;
	}

	client_list = ntb_get_client_list_by_handle(handle);

	if (client_list == NULL)
		return -EINVAL;


	spin_lock(&client_list->client_list_lock);

	if (client_list->clients[NTB_CLIENT_23].handle == handle) {
		client_list->number_used--;
		client = &client_list->clients[NTB_CLIENT_23];
		NTB_DEBUG_PRINT(
		("%s released 23 BAR number of BARs used: %d (decimal)\n",
		PREFIX_STRING,
		client_list->number_used));
	} else if (client_list->clients[NTB_CLIENT_45].handle == handle) {
		client_list->number_used--;
		client = &client_list->clients[NTB_CLIENT_45];
		NTB_DEBUG_PRINT(
		("%s released 45 BAR number of BARs used: %d (decimal)\n",
		PREFIX_STRING,
		client_list->number_used));
	} else {

		spin_unlock(&client_list->client_list_lock);
		return -EINVAL;
	}



	if (client != NULL) {
		NTB_DEBUG_PRINT(("%s releasing client data\n", PREFIX_STRING));
		if (client->handle == client_list->semaphore_owner)
			ntb_release_semaphore(client->handle);


		client->handle = NTB_UNUSED;
		client->callback = NULL;
		client->client_data.limit = 0;
		client->client_data.translate_address = 0;

		if ((handle & NTB_BAR_23) != 0)
			device->policy_bits_23 = 0;
		else if ((handle & NTB_BAR_45) != 0)
			device->policy_bits_45 = 0;
	}

	spin_unlock(&client_list->client_list_lock);

	NTB_DEBUG_PRINT(
	("%s Exiting ntb_unregister_client \n", PREFIX_STRING));

	return SUCCESS;

}

/*****************************************************************************
 * Abstract
 * Writes to doorbell register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful
 * -EINVAL invalid peramter
 ****************************************************************************/
int32_t ntb_write_doorbell(ntb_client_handle_t handle, uint16_t value)
{

	struct ntb_device *device = NULL;
	uint16_t doorbell	  = 0;

	NTB_DEBUG_PRINT(("%s Entering ntb_write_doorbell \n", PREFIX_STRING));

	device = ntb_get_device_by_handle(handle);

	if (device == NULL) {
		NTB_DEBUG_PRINT(
		("%s ERROR: DEVICE IS NULL\n", PREFIX_STRING));
		return -EINVAL;
	}

	NTB_DEBUG_PRINT(("%s doorbell offset %x value to write %x\n",
	PREFIX_STRING, device->doorbell_offset, value));

	ntb_lib_write_16(device->mm_regs,
	device->doorbell_offset,
	value);

	doorbell = ntb_lib_read_16(device->mm_regs,
			device->doorbell_offset);

	NTB_DEBUG_PRINT(("%s doorbell after write %x\n", PREFIX_STRING,
	doorbell));

	NTB_DEBUG_PRINT(("%s Exiting ntb_write_doorbell \n", PREFIX_STRING));

	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * Allows a client to set snoop level.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful
 * -EPERM
 * -EINVAL wrong parameter supplied
 ****************************************************************************/
int32_t ntb_set_snoop_level(ntb_client_handle_t handle, uint32_t value)
{
	struct ntb_device *device = NULL;
	uint32_t read_val = 0;

	device = ntb_get_device_by_handle(handle);
	NTB_DEBUG_PRINT(("%s Entering ntb_set_snoop_level \n", PREFIX_STRING));
	if (device == NULL)
		return -EINVAL;

	/*Check to determine if the client is allowed to set the value */
	if (ntb_compare_handle_snoop_level(handle, value) == VALID) {
		value &= NTB_45_SNOOP_MASK | NTB_23_SNOOP_MASK;
		read_val = ntb_lib_read_32(device->mm_regs, NTB_CNTL_OFFSET);
		if (handle & NTB_BAR_45)
			read_val &= ~NTB_45_SNOOP_MASK;
		if (handle & NTB_BAR_23)
			read_val &= ~NTB_23_SNOOP_MASK;

		read_val |= value;
		NTB_DEBUG_PRINT(("%s Set Snoop Level value: 0x%x\n",
		PREFIX_STRING, read_val));

		ntb_lib_write_32(device->mm_regs, NTB_CNTL_OFFSET, read_val);
	} else {
		NTB_DEBUG_PRINT(("%s INVALID SNOOP LEVEL \n", PREFIX_STRING));
		return -EPERM;
	}

	NTB_DEBUG_PRINT(("%s Exiting ntb_set_snoop_level \n", PREFIX_STRING));
	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * Writes address translation value.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful
 * -EINVAL wrong parameter supplied
 * -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_translate(ntb_client_handle_t handle,
uint64_t address)
{

	struct ntb_device *device = NULL;
	enum ntb_bar_t bar	  = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);
	NTB_DEBUG_PRINT(("%s Entering ntb_write_translate \n", PREFIX_STRING));

	NTB_DEBUG_PRINT(("%s translate address value: %Lx\n",
	PREFIX_STRING, address));

	if (device == NULL)
		return -EINVAL;

	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_23_translate_offset, address);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_translate \n",
		PREFIX_STRING));
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_45_translate_offset, address);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_translate \n",
		PREFIX_STRING));
		return SUCCESS;
	}



	return -EPERM;
}
/*****************************************************************************
 * Abstract
 * Writes limit value.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful
 * -EINVAL wrong parameter supplied
 * -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_limit(ntb_client_handle_t handle, uint64_t value)
{

	struct ntb_device *device = NULL;
	uint64_t limit_setting	  = 0;
	enum ntb_bar_t bar	  = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);
	NTB_DEBUG_PRINT(("%s Entering ntb_write_limit\n", PREFIX_STRING));


	if (device == NULL)
		return -EINVAL;

	if (handle & NTB_BAR_23)
		limit_setting = device->limit_max_23;
	else if (handle & NTB_BAR_45)
		limit_setting = device->limit_max_45;

	NTB_DEBUG_PRINT(("%s Value =  %Lx Limit Settings =  %Lx \n",
		PREFIX_STRING,
		value, limit_setting));

	if (limit_setting <= 0) {
		NTB_DEBUG_PRINT(("%s LIMIT <= 0 \n", PREFIX_STRING));
		return -EPERM;
	}

	if (value == 0 || value > limit_setting) {
		NTB_DEBUG_PRINT(("%s LIMIT OUT OF BOUNDS \n", PREFIX_STRING));

		return -EINVAL;
	}

	if ((value & ALIGNMENT_CHECK) != 0) {
		NTB_DEBUG_PRINT(("%s LIMIT UNALIGNED\n", PREFIX_STRING));
		return -EINVAL;
	}

	NTB_DEBUG_PRINT(("%s writing value = %Lx\n", PREFIX_STRING,
	value + device->limit_base_23));
	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_23_limit_offset, value
		+ device->limit_base_23);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_limit\n",
		PREFIX_STRING));
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_45_limit_offset, value
		+ device->limit_base_45);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_limit\n",
		PREFIX_STRING));
		return SUCCESS;
	}

	return -EPERM;

}

/*****************************************************************************
 * Abstract
 * Reads limit register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: >= 0 successful,
 * -EINVAL wrong parameter supplied
 ****************************************************************************/
uint64_t ntb_read_limit(ntb_client_handle_t handle)
{

	uint64_t limit		  = 0;
	struct ntb_device *device = NULL;
	enum ntb_bar_t bar	  = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Entering ntb_read_limit\n", PREFIX_STRING));

	if (bar == NTB_BAR_23) {
		limit = ntb_lib_read_64(device->mm_regs,
		device->bar_23_limit_offset);
		limit -= device->limit_base_23;

	} else if (bar == NTB_BAR_45) {
		limit = ntb_lib_read_64(device->mm_regs,
		device->bar_45_limit_offset);
		limit -= device->limit_base_45;
	}
	NTB_DEBUG_PRINT(("%s Limit = %Lx \n", PREFIX_STRING, limit));
	NTB_DEBUG_PRINT(("%s Exiting ntb_read_limit\n", PREFIX_STRING));
	return limit;

}

/*****************************************************************************
 * Abstract
 * Writes 0 to 16 scratch pad registers.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
 * -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *values, ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(
	("%s Entering ntb_write_scratch_pad_many\n", PREFIX_STRING));

	if (device == NULL) {
		NTB_DEBUG_PRINT(
		("%s DEVICE IS NULL\n", PREFIX_STRING));
		return -EINVAL;
	}

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO)) {
		NTB_DEBUG_PRINT(
		("%s SCRATCHPAD NUMBER OUT OF BOUNDS\n", PREFIX_STRING));
		return -EINVAL;
	}

	/* If in B2B mode, it doesn't matter if you own the sema4.*/
	if (device->scratchpad_offset_write == NTB_B2B_SCRATCHPAD_OFFSET) {
		ntb_lib_write_rep(device->mm_regs,
		device->scratchpad_offset_write, (void *)(values),
		how_many);
		NTB_DEBUG_PRINT(
		("%s Exiting ntb_write_scratch_pad_many\n", PREFIX_STRING));
		return SUCCESS;
	} else {
		if (device->client_list.semaphore_owner == handle) {
			ntb_lib_write_rep(device->mm_regs,
			device->scratchpad_offset_write, (void *)(values),
			how_many);
			NTB_DEBUG_PRINT(
			("%s Exiting ntb_write_scratch_pad_many\n",
			PREFIX_STRING));
			return SUCCESS;
		} else {
			return -EPERM;
		}
	}

	return -EPERM;
}

/*****************************************************************************
 * Abstract
 * Writes to one scratch pad register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
 * -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_scratch_pad_one(uint32_t index, uint32_t value,
ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_write_scratch_pad_one\n",
	PREFIX_STRING));

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	/* If in B2B mode, it doesn't matter if you own the sema4.*/
	if (device->scratchpad_offset_write == NTB_B2B_SCRATCHPAD_OFFSET) {
		ntb_lib_write_32(device->mm_regs,
		device->scratchpad_offset_write
		+ (index * sizeof(uint32_t)), value);
		NTB_DEBUG_PRINT(
		("%s Exiting ntb_write_scratch_pad_one\n",
		PREFIX_STRING));
		return SUCCESS;
	} else {
		if (device->client_list.semaphore_owner == handle) {
			ntb_lib_write_32(device->mm_regs,
				device->scratchpad_offset_write
				+ (index * sizeof(uint32_t)), value);
				NTB_DEBUG_PRINT(
				("%s Exiting ntb_write_scratch_pad_one\n",
				PREFIX_STRING));
				return SUCCESS;
		} else {
			return -EPERM;
		}
	}

	return -EPERM;

}


#ifdef B0_SI_SOLN_CL
/*****************************************************************************
 * Description: fills up a scratch pad with MSIX table info.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: No return
 ****************************************************************************/
void ntb_store_msix(struct pci_dev *dev, struct ntb_device *device,
struct scratchpad_registers *pad)
{
	int32_t i = 0;
	uint32_t table_offset = NTB_SEC_MSIX_MAP;

	for (i = 0; i < NTB_TOTAL_SCRATCHPAD_NO; i++) {
		pad->registers[i]  =
			ntb_lib_read_32(device->mm_regs,
			table_offset + (i * sizeof(uint32_t)));
		NTB_DEBUG_PRINT(("%s MSIX Register %x\n", PREFIX_STRING,
			pad->registers[i]));
	}
}

/*****************************************************************************
 * Abstract
 * Reads remote MSIX config data into a scratchpad like structure.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
 ****************************************************************************/
int32_t ntb_read_remote_msix(struct scratchpad_registers *pad,
ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_read_remote_msix\n",
		PREFIX_STRING));

	if (pad == NULL) {
		NTB_DEBUG_PRINT(
			("%s  ntb_read_remote_msix NULL scratchpad ptr\n",
			PREFIX_STRING));
		return -EINVAL;
	}

	if (device == NULL) {
		NTB_DEBUG_PRINT(("%s  ntb_read_remote_msix BAD handle\n",
		PREFIX_STRING));
		return -EINVAL;
	}

	ntb_store_msix(device->dev, device, pad);

	NTB_DEBUG_PRINT(("%s Exiting ntb_read_remote_msix\n",
		PREFIX_STRING));

	return SUCCESS;
}
#endif
/*****************************************************************************
 * Abstract
 * Reads multiple scratch pad registers.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
 ****************************************************************************/
int32_t ntb_read_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *pad, ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_read_scratch_pad_many\n",
	PREFIX_STRING));

	if (pad == NULL)
		return -EINVAL;

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO))
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	ntb_lib_read_rep(device->mm_regs,
	device->scratchpad_offset_read,
	(void *)(&pad->registers), how_many);

	NTB_DEBUG_PRINT(("%s pad[0] = %x pad [1] = %x\n", PREFIX_STRING,
	pad->registers[0], pad->registers[1]));

	NTB_DEBUG_PRINT(("%s Exiting ntb_read_scratch_pad_many\n",
	PREFIX_STRING));

	return SUCCESS;

}
/*****************************************************************************
 * Abstract
 * Reads one scratch pad register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied
 *****************************************************************************/
int32_t ntb_read_scratch_pad_one(uint32_t index, uint32_t *value,
ntb_client_handle_t handle)
{
	uint32_t offset = 0;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(("%s Entering ntb_read_scratch_pad_one\n",
		PREFIX_STRING));

	device = ntb_get_device_by_handle(handle);

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	if (value == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(
	("%s starting offset %x offset %x value %x\n", PREFIX_STRING,
	device->scratchpad_offset_read, offset, *value));

	offset = device->scratchpad_offset_read + (index * sizeof(int32_t));
	*value = ntb_lib_read_32(device->mm_regs, offset);

	NTB_DEBUG_PRINT(("%s Exiting ntb_read_scratch_pad_one\n",
	PREFIX_STRING));

	return SUCCESS;

}

/*****************************************************************************
 * Abstract
 * Gets stored link status.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: >= 0 successful,
 * -EINVAL wrong parameter supplied,
 * -EPERM unsuccessful
 *****************************************************************************/
int16_t ntb_get_link_status(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;
	int16_t link_status	  = 0;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_get_link_status\n",
			PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;

	/* Not allowed from this side of NTB*/
	if (device->link_status_offset == 0)
		return -EPERM;

	/* Read the link status register */
	pci_read_config_word(device->dev, device->link_status_offset,
			&link_status);

	NTB_DEBUG_PRINT(
	("%s  Link Status Offset: 0x%x\n", PREFIX_STRING,
	device->link_status_offset));
	NTB_DEBUG_PRINT(("%s Get Link Status: 0x%x\n",
	PREFIX_STRING,
	link_status));


	if (link_status & LINK_STATUS_ACTIVE) {
		device->link_status = LINK_UP;
		if ((device->dev_type == NTB_DEV_TYPE_CLASSIC) ||
		    (device->dev_type == NTB_DEV_TYPE_B2B)) {
			ntb_get_limit_settings(device->dev, NTB_BAR_23,
				device, SECONDARY_CONFIG);
			ntb_get_limit_settings(device->dev, NTB_BAR_45,
					device, SECONDARY_CONFIG);
		}
	} else
		device->link_status = LINK_DOWN;
	NTB_DEBUG_PRINT(("%s Exiting ntb_get_link_status\n",
				PREFIX_STRING));
	return device->link_status;

}
/*****************************************************************************
 * Abstract
 * Sets link status.
 *
 * Side Effects:
 * Assumptions:
 * Return Values:  >= 0 successful,
 * -EINVAL wrong parameter supplied,
 *****************************************************************************/
int32_t ntb_set_link_status(ntb_client_handle_t handle, enum link_t status)
{
	int32_t read_status	  = 0;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(("%s Entering ntb_set_link_status\n",
		PREFIX_STRING));

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	if ((status == SET_LINK_DOWN) || (status ==  SET_LINK_UP)) {
		read_status = ntb_lib_read_32(device->mm_regs,
			device->link_control_offset);

		read_status = ~read_status & status;

		NTB_DEBUG_PRINT(("%s Setting Link Status 0x%x\n",
		PREFIX_STRING,
		read_status));

		ntb_lib_write_32(device->mm_regs,
			device->link_control_offset, read_status);

		NTB_DEBUG_PRINT(("%s Exiting ntb_set_link_status\n",
				PREFIX_STRING));
		return SUCCESS;

	} else {
		return -EINVAL;
	}

}

/*****************************************************************************
 * Abstract
 * Obtains semaphore for a client.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
 * -EAGAIN semaphore already taken
 *****************************************************************************/
int32_t ntb_obtain_semaphore(ntb_client_handle_t handle)
{
	uint32_t value		  = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Entering ntb_obtain_semaphore\n",
			PREFIX_STRING));
	if (device->client_list.semaphore_owner == handle) {
		NTB_DEBUG_PRINT(("%s Already have semaphore\n",
		PREFIX_STRING));
		NTB_DEBUG_PRINT(("%s Exiting ntb_obtain_semaphore\n",
				PREFIX_STRING));
		return SUCCESS;
	}

	if (device->client_list.semaphore_owner == NTB_UNUSED) {
		value = ntb_lib_read_32(device->mm_regs,
		device->semaphore_offset);
		NTB_DEBUG_PRINT(("%s value %x\n", PREFIX_STRING, value));
		device->client_list.semaphore_owner = handle;
		NTB_DEBUG_PRINT(("%s semaphore_owner %x\n", PREFIX_STRING,
		device->client_list.semaphore_owner));
		NTB_DEBUG_PRINT(("%s Exiting ntb_obtain_semaphore\n",
		PREFIX_STRING));
		return SUCCESS;
	}


	NTB_DEBUG_PRINT(("%s NTB_OBTAIN_SEMA4 - TRY AGAIN\n", PREFIX_STRING));

	return -EAGAIN; /* try again */

}
/*****************************************************************************
 * Abstract
 * Releases semaphore for a client.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
*****************************************************************************/
int32_t ntb_release_semaphore(ntb_client_handle_t handle)
{

	struct ntb_device *device = NULL;
	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_release_semaphore\n",
				PREFIX_STRING));

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Owner = %x\n", PREFIX_STRING,
	device->client_list.semaphore_owner));
	if (device != NULL) {
		if (device->client_list.semaphore_owner == handle) {
			if (device->client_list.semaphore_owner
			!= NTB_UNUSED){
				NTB_DEBUG_PRINT(("%s releasing semafore\n",
				PREFIX_STRING));
				ntb_lib_write_32(device->mm_regs,
					device->semaphore_offset,
					SEMAPHORE_ONE_TO_CLEAR);
				device->client_list.semaphore_owner
					= NTB_UNUSED;
				NTB_DEBUG_PRINT((
				"%s Exiting ntb_release_semaphore\n",
				PREFIX_STRING));

				return SUCCESS;
			} else {
				NTB_DEBUG_PRINT(("%s WRONG SEMA4 OWNER\n",
				PREFIX_STRING));
				return -EINVAL;
			}
		} else {
			NTB_DEBUG_PRINT(("%s NOT OWNER\n", PREFIX_STRING));
			return -EINVAL;
		}
	}
	NTB_DEBUG_PRINT(("%s Exiting ntb_release_semaphore\n", PREFIX_STRING));
	return SUCCESS;
}

/*****************************************************************************
*****************************************************************************/

/*****************************************************************************
 * Abstract
 * adds a new doorbell policy.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
 * -EINVAL wrong parameter supplied,
*****************************************************************************/
int32_t ntb_add_policy(ntb_client_handle_t handle,
uint16_t heartbeat_bit, uint16_t bar_bits,
uint16_t power_event_bit,
uint16_t power_notification_bit)
{
	struct ntb_device *device = NULL;
	uint16_t add_on_policy	  = 0;
	int32_t ret		  = SUCCESS;

	NTB_DEBUG_PRINT(("%s Entering ntb_add_policy\n", PREFIX_STRING));

	add_on_policy = (heartbeat_bit |
	bar_bits | power_event_bit | power_notification_bit);

	if ((add_on_policy & DOORBELL_LINK_BIT) != 0)
		return -EPERM;

	if ((add_on_policy & DOORBELL_FLUSH_ACK) != 0)
		return -EPERM;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("%s DEVICE IS NULL\n", PREFIX_STRING));
		return -EINVAL;
	}

	if ((handle & NTB_BAR_23) != 0) {
		if (add_on_policy & device->policy_bits_45)
			return -EINVAL;
		else
			device->policy_bits_23 |= add_on_policy;
	} else if ((handle & NTB_BAR_45) != 0) {
		if (add_on_policy & device->policy_bits_23)
			return -EINVAL;
		else
			device->policy_bits_45 |= add_on_policy;
	} else
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Exiting ntb_add_policy\n", PREFIX_STRING));
	return ret;
}

/*****************************************************************************
 * Abstract
 * resets the doorbell policy for a particular client.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
 * -EINVAL wrong parameter supplied,
*****************************************************************************/
int32_t ntb_reset_policy(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(("%s Entering ntb_reset_policy\n", PREFIX_STRING));
	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("%s RESET POLICY- DEVICE IS NULL\n",
		PREFIX_STRING));
		return -EINVAL;
	}

	if ((handle & NTB_BAR_23) != 0)
		device->policy_bits_23 = 0;
	else if ((handle & NTB_BAR_45) != 0)
		device->policy_bits_45 = 0;
	else
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Exiting ntb_reset_policy\n", PREFIX_STRING));
	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * returns the policy, indicating which bits are already set.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied,
*****************************************************************************/
int16_t ntb_get_policy(ntb_client_handle_t handle)
{
	int16_t ret		  = 0;
	struct ntb_device *device = NULL;
	NTB_DEBUG_PRINT(("%s Entering ntb_get_policy\n", PREFIX_STRING));
	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("%s DEVICE IS NULL\n", PREFIX_STRING));
		return -EINVAL;
	}

	ret = device->policy_bits_23 | device->policy_bits_45;

	NTB_DEBUG_PRINT(("%s policy : 0x%x (%d)\n", PREFIX_STRING,
	ret, ret));
	NTB_DEBUG_PRINT(("%s Exiting ntb_get_policy\n", PREFIX_STRING));
	return ret;
}


/*****************************************************************************
 * Abstract
 * returns next available BDF/BAR combination.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
*****************************************************************************/
int32_t ntb_get_next_bdf(uint16_t *next_bdf,
uint32_t *next_bar)
{
	struct ntb_device *device = NULL;
	struct ntb_clients *client_list = NULL;
	int32_t i = 0;


	NTB_DEBUG_PRINT(("%s Entering ntb_get_next_bdf\n", PREFIX_STRING));

	if (next_bdf == NULL || next_bar == NULL)
		return -EINVAL;

	if (*next_bdf == 0) {
		device = ntb_get_device(i);
	} else {


		device = ntb_get_device(i);
		while (device->bdf != *next_bdf && i < MAX_DEVICES) {

			device = ntb_get_device(i);
			i++;
		}
		if (device == NULL) {
			*next_bar = 0;
			*next_bdf = 0;
			return -EPERM;
		}

		if (device->device_state != ENUMERATED) {
			*next_bar = 0;
			*next_bdf = 0;
			NTB_DEBUG_PRINT(("%s Exiting ntb_get_next_bdf\n",
			PREFIX_STRING));
			return SUCCESS;
		}

		if (next_bar != NULL) {
			client_list =
			ntb_get_client_list(device->device_index);

			if (client_list == NULL)
				return -EPERM;

			if (*next_bar == NTB_BAR_23) {
				if (client_list->clients[NTB_CLIENT_45].handle
					== NTB_UNUSED) {
					*next_bar = NTB_BAR_45;
					*next_bdf =
					client_list->
					clients[NTB_CLIENT_45].bdf;
					NTB_DEBUG_PRINT((
					"%s Exiting ntb_get_next_bdf\n",
					PREFIX_STRING));
					return SUCCESS;
				}

			} else {
				i++;
				device = ntb_get_device(i);
				if (device == NULL)
					return -EPERM;
			}
		}
	}

	while (i < MAX_DEVICES) {
		NTB_DEBUG_PRINT((
		"%s checking for BAR BDF combos \n",
		PREFIX_STRING));

		if (device == NULL) {
			*next_bar = 0;
			*next_bdf = 0;
			return -EPERM;
		}

		if (device->device_state != ENUMERATED) {
			*next_bar = 0;
			*next_bdf = 0;
			return SUCCESS;
		}

		client_list =
			ntb_get_client_list(device->device_index);

		if (client_list == NULL)
			return -EPERM;

		if (client_list->clients[NTB_CLIENT_23].handle
			== NTB_UNUSED) {
			*next_bar = NTB_BAR_23;
			*next_bdf =
			client_list->clients[NTB_CLIENT_23].bdf;
			NTB_DEBUG_PRINT(("%s NTB BAR 23 %x BDF %x %p %p\n",
			PREFIX_STRING,
			*next_bar, *next_bdf, next_bar, next_bdf));
			return SUCCESS;

		} else if (client_list->clients[NTB_CLIENT_45].handle
			== NTB_UNUSED) {
			*next_bar = NTB_BAR_45;
			*next_bdf =
			client_list->clients[NTB_CLIENT_45].bdf;
			NTB_DEBUG_PRINT(("%s NTB BAR 45 %x BDF %x \n",
			PREFIX_STRING,
			*next_bar, *next_bdf));
			return SUCCESS;
		}

		device = ntb_get_device(i);
		i++;
	}
	NTB_DEBUG_PRINT(
	("%s No more BDFs available \n", PREFIX_STRING));

	*next_bar = 0;
	*next_bdf = 0;
	return SUCCESS;

}

/*****************************************************************************
 * Abstract
 * returns number of BDF/BAR combinations unused.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_get_number_unused_bdfs(void)
{
	struct ntb_device *device = NULL;
	struct ntb_clients *client_list = NULL;
	int32_t i = 0;
	int32_t counter = 0;

	NTB_DEBUG_PRINT(("%s Entering ntb_get_number_unused_bdfs\n",
		PREFIX_STRING));
	while (i < MAX_DEVICES) {

		device = ntb_get_device(i);
		if (device == NULL) {
			NTB_DEBUG_PRINT(("%s DEVICE IS NULL\n",
			PREFIX_STRING));
			return -EPERM;
		}

		if (device->device_state == ENUMERATED) {
			client_list =
			ntb_get_client_list(device->device_index);

			if (client_list == NULL)
				return -EPERM;
			else {
				if (client_list->clients[NTB_CLIENT_23].handle
				== NTB_UNUSED)
					counter++;

				if (client_list->clients[NTB_CLIENT_45].handle
				== NTB_UNUSED)
					counter++;

			}

		}
		i++;
	}
	NTB_DEBUG_PRINT(("%s Number of BDFs available is %x\n", PREFIX_STRING,
	counter));
	NTB_DEBUG_PRINT(("%s Exiting ntb_get_number_unused_bdfs\n",
	PREFIX_STRING));
	return counter;
}

/*****************************************************************************
 * Abstract
 * writes wccntrl register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_write_wccntrl_bit(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;
	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_write_wccntrl_bit\n",
		PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;

	ntb_lib_write_32(device->mm_regs,
	NTB_WCCNTRL_OFFSET, NTB_WCCNTRL_WRITE);

	NTB_DEBUG_PRINT(("%s Exiting ntb_write_wccntrl_bit\n",
		PREFIX_STRING));

	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * returns wnctrl register value.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
uint32_t ntb_read_wccntrl_bit(ntb_client_handle_t handle)
{
	uint32_t value = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);
	NTB_DEBUG_PRINT(("%s Entering ntb_read_wccntrl_bit\n",
			PREFIX_STRING));
	if (device == NULL)
		return -EINVAL;

	value = ntb_lib_read_32(device->mm_regs,
	NTB_WCCNTRL_OFFSET);

	NTB_DEBUG_PRINT(("%s READ WCCNTRL value %x\n",	PREFIX_STRING,
			value));
	NTB_DEBUG_PRINT(("%s Exiting ntb_read_wccntrl_bit\n", PREFIX_STRING));

	return value;
}

/*****************************************************************************
 * Abstract
 * sets translate (primary) register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_write_remote_translate(ntb_client_handle_t handle,
enum ntb_bar_t bar,
uint64_t address)
{

	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("%s Entering ntb_write_remote_translate\n",
	PREFIX_STRING));

	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		NTB_PBAR_23_TRANSLATE_OFFSET, address);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_remote_translate\n",
			PREFIX_STRING));
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		NTB_PBAR_45_TRANSLATE_OFFSET, address);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_remote_translate\n",
			PREFIX_STRING));
		return SUCCESS;
	}

	return -EPERM;
}

/*****************************************************************************
 * Abstract
 * returns translate (primnary) value.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int64_t ntb_read_remote_translate(ntb_client_handle_t handle,
enum ntb_bar_t bar)
{
	int64_t translate = 0;
	struct ntb_device *device = NULL;
	int64_t ret = -EINVAL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_read_remote_translate\n",
		PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;



	if (bar == NTB_BAR_23) {
		translate = ntb_lib_read_64(device->mm_regs,
		NTB_PBAR_23_TRANSLATE_OFFSET);
		NTB_DEBUG_PRINT(("%s Exiting ntb_read_remote_translate\n",
				PREFIX_STRING));
		return translate;

	} else if (bar == NTB_BAR_45) {
		translate = ntb_lib_read_64(device->mm_regs,
		NTB_PBAR_45_TRANSLATE_OFFSET);
		NTB_DEBUG_PRINT(("%s Exiting ntb_read_remote_translate\n",
				PREFIX_STRING));
		return translate;

	}

	return ret;
}

/*****************************************************************************
 * Abstract
 * sets secondary side's mask values.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_write_remote_doorbell_mask(ntb_client_handle_t handle,
uint16_t mask)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_write_remote_doorbell_mask\n",
	PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;

	ntb_lib_write_16(device->mm_regs,
	DOORBELL_SECONDARY_MASK_OFFSET, mask);

	NTB_DEBUG_PRINT(("%s mask %x\n", PREFIX_STRING,
	mask));
	NTB_DEBUG_PRINT(("%s Exiting ntb_write_remote_doorbell_mask\n",
			PREFIX_STRING));
	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * returns secondary side's mask values.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_read_remote_doorbell_mask(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;
	int32_t mask = 0;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_read_remote_doorbell_mask\n",
		PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;

	mask = ntb_lib_read_16(device->mm_regs,
	DOORBELL_SECONDARY_MASK_OFFSET);

	NTB_DEBUG_PRINT(("%s mask %x\n", PREFIX_STRING,
	mask));

	NTB_DEBUG_PRINT(("%s Exiting ntb_read_remote_doorbell_mask\n",
			PREFIX_STRING));

	return mask;
}

/*****************************************************************************
 * Abstract
 * sets secondary limit register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int32_t ntb_write_remote_limit(ntb_client_handle_t handle,
enum ntb_bar_t bar,
uint64_t value)
{
	struct ntb_device *device = NULL;
	uint64_t limit_setting = 0;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_write_remote_limit\n",
			PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;


	if (handle & NTB_BAR_23)
		limit_setting = device->secondary_limit_max_23;
	else if (handle & NTB_BAR_45)
		limit_setting = device->secondary_limit_max_45;

	NTB_DEBUG_PRINT(("%s limt_setting =  %Lx \n", PREFIX_STRING,
	limit_setting));
	NTB_DEBUG_PRINT(("%s value = %Lx \n", PREFIX_STRING,
	value));

	if (limit_setting <= 0)
		return -EPERM;

	if (value == 0 || value > limit_setting) {
		NTB_DEBUG_PRINT(("%s LIMIT OUT OF BOUNDS\n",
		PREFIX_STRING));
		NTB_DEBUG_PRINT(("%s Value %Lx Limit Settings %Lx \n",
		PREFIX_STRING,
		value, limit_setting));
		return -EINVAL;
	}

	if ((value & ALIGNMENT_CHECK) != 0) {
		NTB_DEBUG_PRINT(("%s ALIGNMENT CHECK\n",
		PREFIX_STRING));
		NTB_DEBUG_PRINT(("%s Value %Lx Limit Settings %Lx \n",
		PREFIX_STRING,
		value, limit_setting));
		return -EINVAL;
	}

	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		NTB_SBAR_23_LIMIT_OFFSET, value
		+ device->secondary_limit_base_23);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_remote_limit\n",
		PREFIX_STRING));
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		NTB_SBAR_45_LIMIT_OFFSET, value
		+ device->secondary_limit_base_45);
		NTB_DEBUG_PRINT(("%s Exiting ntb_write_remote_limit\n",
		PREFIX_STRING));
		return SUCCESS;
	} else {
		return -EINVAL;
	}

	return -EPERM;

}

/*****************************************************************************
 * Abstract
 * returns secondary limit register value.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful,
 * -EPERM
 * -EINVAL wrong parameter supplied
*****************************************************************************/
int64_t ntb_read_remote_limit(ntb_client_handle_t handle,
enum ntb_bar_t bar)
{
	uint64_t limit = 0;
	struct ntb_device *device = NULL;
	int64_t ret = -EINVAL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("%s Entering ntb_read_remote_limit\n",
	PREFIX_STRING));

	if (device == NULL)
		return -EINVAL;


	if (bar == NTB_BAR_23) {
		limit = ntb_lib_read_64(device->mm_regs,
		NTB_SBAR_23_LIMIT_OFFSET);
		limit -= device->secondary_limit_base_23;
		NTB_DEBUG_PRINT(("%s Limit = %Lx\n",
		PREFIX_STRING, limit));
		NTB_DEBUG_PRINT(("%s Exiting ntb_read_remote_limit\n",
		PREFIX_STRING));
		return limit;

	} else if (bar == NTB_BAR_45) {
		limit = ntb_lib_read_64(device->mm_regs,
		NTB_SBAR_45_LIMIT_OFFSET);
		limit -= device->secondary_limit_base_45;
		NTB_DEBUG_PRINT(("%s Limit = %Lx\n",
		PREFIX_STRING, limit));
		NTB_DEBUG_PRINT(("%s Exiting read_remote_limit\n",
		PREFIX_STRING));
		return limit;
	}

	return ret;
}
