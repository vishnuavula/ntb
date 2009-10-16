/*****************************************************************************
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
	struct ntb_client *new_client   = NULL;
	ntb_client_handle_t handle      = NTB_UNUSED;
	struct ntb_clients *client_list = NULL;
	uint32_t prefix                 = 0;

	client_list = ntb_get_client_list_by_bdf(bdf);

	NTB_DEBUG_PRINT(
	("NTB: INSIDE NTB_REGISTER_CLIENT BAR %x BDF %x\n",
	bar, bdf));

	if (client_list == NULL) {
		NTB_DEBUG_PRINT(
		("NTB: NO CLIENT LIST\n"));
		return -EINVAL;
	}
	NTB_DEBUG_PRINT(
	("NTB: Client List %p\n", client_list));

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
			("NTB: ALLOC 23 BAR HANDLE %x\n", handle));
		} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(
			("NTB: IN USE 23 BAR\n"));
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
			("NTB: ALLOC 45 BAR\n"));
	} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(
			("NTB: IN USE 45 BAR\n"));
			return handle;
		}
	} else {
			handle = -EINVAL;
			spin_unlock(&client_list->client_list_lock);
			NTB_DEBUG_PRINT(
			("NTB: BAD BAR \n"));
			return handle;
	}

	spin_unlock(&client_list->client_list_lock);

	if (new_client == NULL)
		return -EPERM;

	if (handle > NTB_UNUSED && client_data != NULL)
		ntb_write_client_data(handle, new_client, client_data);

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
	struct ntb_client *client       = NULL;
	struct ntb_clients *client_list = NULL;
	struct ntb_device *device       = NULL;

	NTB_DEBUG_PRINT(
	("NTB: INSIDE NTB_UNREGISTER_CLIENT\n"));
	device = ntb_get_device_by_handle(handle);

	if (device == NULL) {
		NTB_DEBUG_PRINT(
			("NTB: NTB_UNREGISTER_CLIENT Error: Device NULL\n"));
		return -EINVAL;
	}
	NTB_DEBUG_PRINT(
	("NTB: DEVICE %p\n", device));

	client_list = ntb_get_client_list_by_handle(handle);

	if (client_list == NULL)
		return -EINVAL;
	NTB_DEBUG_PRINT(("NTB: CLIENT LIST %p\n", client_list));

	spin_lock(&client_list->client_list_lock);

	if (client_list->clients[NTB_CLIENT_23].handle == handle) {
		client_list->number_used--;
		client = &client_list->clients[NTB_CLIENT_23];
		NTB_DEBUG_PRINT(
		("NTB: RELEASED 23 BAR  number_used: %d (decimal)\n",
		client_list->number_used));
	} else if (client_list->clients[NTB_CLIENT_45].handle == handle) {
		client_list->number_used--;
		client = &client_list->clients[NTB_CLIENT_45];
		NTB_DEBUG_PRINT(
		("NTB: RELEASED 45 BAR  number_used: %d (decimal)\n",
		client_list->number_used));
	} else {

		spin_unlock(&client_list->client_list_lock);
		return -EINVAL;
	}



	if (client != NULL) {
		NTB_DEBUG_PRINT(("NTB: RELEASE CLIENT DATA\n"));
		if (client->handle == client_list->semaphore_owner)
			ntb_release_semaphore(client->handle);


		client->handle = NTB_UNUSED;
		client->callback = NULL;
		client->client_data.limit = 0;
		client->client_data.translate_address = 0;

		if ((handle & NTB_BAR_23) != 0) {
			device->policy_bits_23 = 0;
		} else if ((handle & NTB_BAR_45) != 0) {
			device->policy_bits_45 = 0;
		}
	}

	spin_unlock(&client_list->client_list_lock);

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
	uint16_t doorbell         = 0;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_DOORBELL \n"));

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: doorbell offset %x value to write %x\n",
	device->doorbell_offset, value));

	ntb_lib_write_16(device->mm_regs,
	device->doorbell_offset,
	value);

	doorbell = ntb_lib_read_16(device->mm_regs,
			device->doorbell_offset);

	NTB_DEBUG_PRINT(("NTB: doorbell after write %x\n", doorbell));

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
		NTB_DEBUG_PRINT(
			("NTB: Set Snoop Level value: 0x%x\n", read_val));

		ntb_lib_write_32(device->mm_regs, NTB_CNTL_OFFSET, read_val);
	} else {
		return -EPERM;
	}


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
	enum ntb_bar_t bar        = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("NTB: WRITE TRANSLATE ADDRESS VALUE offset %Lx\n",
		address));

	if (device == NULL)
		return -EINVAL;

	if (address != 0) {
		if (bar == NTB_BAR_23) {
			ntb_lib_write_64(device->mm_regs,
			device->bar_23_translate_offset, address);
			return SUCCESS;
		} else if (bar == NTB_BAR_45) {
			ntb_lib_write_64(device->mm_regs,
			device->bar_45_translate_offset, address);
			return SUCCESS;
		}

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
	uint64_t limit_setting    = 0;
	enum ntb_bar_t bar        = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_LIMIT dev %p \n",
	device));

	if (device == NULL)
		return -EINVAL;

	if (handle & NTB_BAR_23)
		limit_setting = device->limit_max_23;
	else if (handle & NTB_BAR_45)
		limit_setting = device->limit_max_45;

	if (limit_setting <= 0)
		return -EPERM;

	if (value == 0 || value > limit_setting) {
		NTB_DEBUG_PRINT(("NTB: Limit out of bounds \n"));
		NTB_DEBUG_PRINT(("NTB: Value %Lx Limit Settings %Lx \n",
		value, limit_setting));
		return -EINVAL;
	}

	if ((value & ALIGNMENT_CHECK) != 0) {
		NTB_DEBUG_PRINT(("NTB: Limit unaligned\n"));
		NTB_DEBUG_PRINT(("NTB: Value %Lx Limit Settings %Lx \n",
		value, limit_setting));
		return -EINVAL;
	}

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_LIMIT WRITING VALUE %Lx\n",
	value + device->limit_base_23));
	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_23_limit_offset, value
		+ device->limit_base_23);
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		device->bar_45_limit_offset, value
		+ device->limit_base_45);
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

	uint64_t limit            = 0;
	struct ntb_device *device = NULL;
	enum ntb_bar_t bar        = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_READ_LIMIT dev %p \n",
			device));
	NTB_DEBUG_PRINT(("NTB: READ LIMIT VALUE offset %x\n",
			device->bar_23_limit_offset));

	if (bar == NTB_BAR_23) {
		limit = ntb_lib_read_64(device->mm_regs,
		device->bar_23_limit_offset);
		limit -= device->limit_base_23;

	} else if (bar == NTB_BAR_45) {
		limit = ntb_lib_read_64(device->mm_regs,
		device->bar_45_limit_offset);
		limit -= device->limit_base_45;
	}

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
	("NTB: INSIDE NTB_WRITE_PAD_MANY\n"));

	if (device == NULL)
		return -EINVAL;

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO))
		return -EINVAL;

	/* If in B2B mode, it doesn't matter if you own the sema4.*/
	if (device->scratchpad_offset_write == NTB_B2B_SCRATCHPAD_OFFSET) {
		ntb_lib_write_rep(device->mm_regs,
		device->scratchpad_offset_write, (void *)(values),
		how_many);
		return SUCCESS;
	} else {
		if (device->client_list.semaphore_owner == handle) {
			ntb_lib_write_rep(device->mm_regs,
			device->scratchpad_offset_write, (void *)(values),
			how_many);
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

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_PAD_ONE\n"));

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	/* If in B2B mode, it doesn't matter if you own the sema4.*/
	if (device->scratchpad_offset_write == NTB_B2B_SCRATCHPAD_OFFSET) {
		ntb_lib_write_32(device->mm_regs,
		device->scratchpad_offset_write
		+ (index * sizeof(uint32_t)), value);
		return SUCCESS;
	} else {
		if (device->client_list.semaphore_owner == handle) {
			ntb_lib_write_32(device->mm_regs,
				device->scratchpad_offset_write
				+ (index * sizeof(uint32_t)), value);
				return SUCCESS;
		} else {
			return -EPERM;
		}
	}

	return -EPERM;

}

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

	if (pad == NULL)
		return -EINVAL;

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO))
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	ntb_lib_read_rep(device->mm_regs,
	device->scratchpad_offset_read,
	(void *)(&pad->registers), how_many);

	NTB_DEBUG_PRINT(("NTB: READ_SCRATCHPAD_MANY %x %x\n",
	pad->registers[0], pad->registers[1]));

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

	device = ntb_get_device_by_handle(handle);

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EINVAL;

	if (device == NULL)
		return -EINVAL;

	if (value == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(
	("NTB: READ_SCRATCHPAD_MANY starting offset %x offset %x value %x\n",
	device->scratchpad_offset_read, offset, *value));

	offset = device->scratchpad_offset_read + (index * sizeof(int32_t));
	*value = ntb_lib_read_32(device->mm_regs, offset);

	NTB_DEBUG_PRINT(
	("NTB: READ_SCRATCHPAD_MANY starting offset %x offset %x value %x\n",
	device->scratchpad_offset_read, offset, *value));

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
	int16_t link_status       = 0;

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	/* Not allowed from this side of NTB*/
	if (device->link_status_offset == 0)
		return -EPERM;

	/* Read the link status register */
	pci_read_config_word(device->dev, device->link_status_offset,
			&link_status);

	NTB_DEBUG_PRINT(
	("NTB: Get Link Status: LinkStatusOffset: 0x%x\n",
	device->link_status_offset));
	NTB_DEBUG_PRINT(("NTB: Get Link Status: LinkStatus: 0x%x\n",
	link_status));


	if (link_status & LINK_STATUS_ACTIVE)
		device->link_status = LINK_UP;
	else
		device->link_status = LINK_DOWN;

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
	int32_t read_status       = 0;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(("NTB: Set Link Status: LinkStatus: 0x%x\n",
	status));

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EINVAL;

	if ((status == SET_LINK_DOWN) || (status ==  SET_LINK_UP)) {
	read_status = ntb_lib_read_32(device->mm_regs,
		device->link_control_offset);

		NTB_DEBUG_PRINT(("NTB: Read Link Status: LinkStatus: 0x%x\n",
		read_status));

		read_status = ~read_status & status;

		NTB_DEBUG_PRINT(("NTB: Read Link Status: LinkStatus: 0x%x\n",
		read_status));

		NTB_DEBUG_PRINT(("NTB: Setting Link Status 0x%x\n",
		read_status));
		ntb_lib_write_32(device->mm_regs,
			device->link_control_offset, read_status);

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
	uint32_t value            = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_OBTAIN_SEMA4\n"));
	if (device->client_list.semaphore_owner == handle) {
		NTB_DEBUG_PRINT(("NTB: Already have semaphore\n"));
		return SUCCESS;
	}

	if (device->client_list.semaphore_owner == NTB_UNUSED) {
		value = ntb_lib_read_32(device->mm_regs,
		device->semaphore_offset);
		NTB_DEBUG_PRINT(("NTB: INSIDE NTB_OBTAIN_SEMA4 %x\n", value));
		device->client_list.semaphore_owner = handle;
		NTB_DEBUG_PRINT(("NTB: semaphore_owner %x\n",
		device->client_list.semaphore_owner));
		return SUCCESS;
	}


	NTB_DEBUG_PRINT(("NTB: NTB_OBTAIN_SEMA4 - TRY AGAIN\n"));

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

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_RELEASE_SEMA4\n"));

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: Owner %x\n",
	device->client_list.semaphore_owner));
	if (device != NULL) {
		if (device->client_list.semaphore_owner == handle) {
			if (device->client_list.semaphore_owner
			!= NTB_UNUSED){
				NTB_DEBUG_PRINT(("NTB: RELEASING_SEMA4\n"));
				ntb_lib_write_32(device->mm_regs,
					device->semaphore_offset,
					SEMAPHORE_ONE_TO_CLEAR);
				device->client_list.semaphore_owner
					= NTB_UNUSED;
				NTB_DEBUG_PRINT(("NTB: RELEASED SEMAPHORE\n"));
				return SUCCESS;
			} else {
				NTB_DEBUG_PRINT(("NTB: WRONG SEMA4 OWNER\n"));
				return -EINVAL;
			}
		} else {
			NTB_DEBUG_PRINT(("NTB: NOT OWNER\n"));
			return -EINVAL;
		}
	}
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
	uint16_t add_on_policy    = 0;
	int32_t ret               = SUCCESS;

	NTB_DEBUG_PRINT(("NTB: ADD POLICY: Entry\n"));

	add_on_policy = (heartbeat_bit |
	bar_bits | power_event_bit | power_notification_bit);

	if ((add_on_policy & DOORBELL_LINK_BIT) != 0)
		return -EPERM;

	if ((add_on_policy & DOORBELL_FLUSH_ACK) != 0)
		return -EPERM;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("NTB: ADD POLICY- NULL Device Returned\n"));
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

	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("NTB: RESET POLICY- NULL Device Returned\n"));
		return -EINVAL;
	}


	NTB_DEBUG_PRINT(("NTB: RESET POLICY\n"));
	if ((handle & NTB_BAR_23) != 0) {
		device->policy_bits_23 = 0;
	} else if ((handle & NTB_BAR_45) != 0) {
		device->policy_bits_45 = 0;
	} else {
		return -EINVAL;
	}

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
uint16_t ntb_get_policy(ntb_client_handle_t handle)
{
	int16_t ret               = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL) {
		NTB_DEBUG_PRINT(("NTB: GET POLICY- NULL Device Returned\n"));
		return -EINVAL;
	}

	ret = device->policy_bits_23 | device->policy_bits_45;

	NTB_DEBUG_PRINT(("NTB: POLICY RETURNED: 0x%x (%d)\n", ret, ret));

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


	NTB_DEBUG_PRINT(
	("NTB_GET_NEXT_BDF \n"));

	if (next_bdf == NULL || next_bar == NULL)
		return -EINVAL;

	if (*next_bdf == 0) {
		device = ntb_get_device(i);
		NTB_DEBUG_PRINT(
		("NTB FIRST BDF \n"));
	} else {

		NTB_DEBUG_PRINT(
		("NTB FINDING PLACE IN LIST\n"));
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
					NTB_DEBUG_PRINT(("NTB: BAR 45 \n"));
					*next_bar = NTB_BAR_45;
					*next_bdf =
					client_list->clients[NTB_CLIENT_45].bdf;
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
		NTB_DEBUG_PRINT(("NTB: CHECKING FOR BDF/BAR COMBOS \n"));

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
			NTB_DEBUG_PRINT(
			("NTB BAR 23 %x BDF %x %p %p\n",
			*next_bar, *next_bdf, next_bar, next_bdf));
			return SUCCESS;

		} else if (client_list->clients[NTB_CLIENT_45].handle
			== NTB_UNUSED) {
			*next_bar = NTB_BAR_45;
			*next_bdf =
			client_list->clients[NTB_CLIENT_45].bdf;
			NTB_DEBUG_PRINT(
			("NTB BAR 45 %x BDF %x \n", *next_bar, *next_bdf));
			return SUCCESS;
		}

		device = ntb_get_device(i);
		i++;
	}
	NTB_DEBUG_PRINT(
	("NTB: No more BDFs available \n"));

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

	while (i < MAX_DEVICES) {

		device = ntb_get_device(i);
		if (device == NULL) {
			NTB_DEBUG_PRINT(("NTB: device == null\n"));
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
	NTB_DEBUG_PRINT(("NTB: Number BDFs available %x\n", counter));
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

	if (device == NULL)
		return -EINVAL;

	ntb_lib_write_32(device->mm_regs,
	NTB_WCCNTRL_OFFSET, NTB_WCCNTRL_WRITE);

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

	if (device == NULL)
		return -EINVAL;

	value = ntb_lib_read_32(device->mm_regs,
	NTB_WCCNTRL_OFFSET);

	NTB_DEBUG_PRINT(("NTB: READ WCCNTRL value %x\n",
			value));

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

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_REMOTE_TRANSLATE dev %p \n",
	device));

	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		NTB_PBAR_23_TRANSLATE_OFFSET, address);
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		NTB_PBAR_45_TRANSLATE_OFFSET, address);
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

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_READ_REMOTE_TRANSLATE dev %p \n",
			device));

	if (bar == NTB_BAR_23) {
		translate = ntb_lib_read_64(device->mm_regs,
		NTB_PBAR_23_TRANSLATE_OFFSET);
		return translate;

	} else if (bar == NTB_BAR_45) {
		translate = ntb_lib_read_64(device->mm_regs,
		NTB_PBAR_45_TRANSLATE_OFFSET);
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
uint32_t mask)
{
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	ntb_lib_write_16(device->mm_regs,
	DOORBELL_SECONDARY_MASK_OFFSET, mask);

	NTB_DEBUG_PRINT(("NTB: WRITE SECONDARY MASK mask %x\n",
				mask));

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
uint16_t ntb_read_remote_doorbell_mask(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;
	uint16_t mask = 0;

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	mask = ntb_lib_read_16(device->mm_regs,
	DOORBELL_SECONDARY_MASK_OFFSET);

	NTB_DEBUG_PRINT(("NTB: READ SECONDARY MASK mask %x\n",
	mask));

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

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_SECONDARY_LIMIT dev %p \n",
	device));

	if (handle & NTB_BAR_23)
		limit_setting = device->secondary_limit_max_23;
	else if (handle & NTB_BAR_45)
		limit_setting = device->secondary_limit_max_45;

	if (limit_setting <= 0)
		return -EPERM;

	if (value == 0 || value > limit_setting) {
		NTB_DEBUG_PRINT(("NTB: Limit out of bounds\n"));
		NTB_DEBUG_PRINT(("NTB: Value %Lx Limit Settings %Lx \n",
		value, limit_setting));
		return -EINVAL;
	}

	if ((value & ALIGNMENT_CHECK) != 0) {
		NTB_DEBUG_PRINT(("NTB: Limit unaligned\n"));
		NTB_DEBUG_PRINT(("NTB: Value %Lx Limit Settings %Lx \n",
		value, limit_setting));
		return -EINVAL;
	}

	if (bar == NTB_BAR_23) {
		ntb_lib_write_64(device->mm_regs,
		NTB_SBAR_23_LIMIT_OFFSET, value
		+ device->secondary_limit_base_23);
		return SUCCESS;
	} else if (bar == NTB_BAR_45) {
		ntb_lib_write_64(device->mm_regs,
		NTB_SBAR_45_LIMIT_OFFSET, value
		+ device->secondary_limit_base_45);
		return SUCCESS;
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
	int64_t limit = 0;
	struct ntb_device *device = NULL;
	int64_t ret = -EINVAL;

	device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_READ_SECONDARY_LIMIT dev %p \n",
			device));

	if (bar == NTB_BAR_23) {
		limit = ntb_lib_read_64(device->mm_regs,
		NTB_SBAR_23_LIMIT_OFFSET);
		limit -= device->secondary_limit_base_23;
		return limit;

	} else if (bar == NTB_BAR_45) {
		limit = ntb_lib_read_64(device->mm_regs,
		NTB_SBAR_45_LIMIT_OFFSET);
		limit -= device->secondary_limit_base_45;
		return limit;
	}

	return ret;
}
