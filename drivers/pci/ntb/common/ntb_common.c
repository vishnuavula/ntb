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
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */


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
 * -EPERM no client list available for processor ID.
 ****************************************************************************/
ntb_client_handle_t ntb_register_client(enum ntb_bar_t bar,
ntb_callback_t callback, enum ntb_proc_id_t processor_id,
struct ntb_client_data *client_data)
{
	struct ntb_device *device = NULL;
	struct ntb_client *new_client = NULL;
	ntb_client_handle_t handle = NTB_UNUSED;
	struct ntb_clients *client_list = NULL;
	uint32_t prefix = 0;
	client_list = ntb_get_client_list(processor_id);

	NTB_DEBUG_PRINT(
	("NTB: INSIDE NTB_REGISTER_CLIENT\n"));

	if (client_list == NULL) {
		NTB_DEBUG_PRINT(
		("NTB: NO CLIENT LIST\n"));
		return -EPERM;
	}
	spin_lock(&client_list->client_list_lock);
	if (bar == NTB_BAR_23) {
		if (client_list->clients[NTB_CLIENT_23].handle == NTB_UNUSED) {
			prefix = client_list->clients[NTB_CLIENT_23].prefix;
			handle = bar | prefix;
			client_list->clients[NTB_CLIENT_23].handle = handle;
			client_list->clients[NTB_CLIENT_23].callback = callback;
			NTB_DEBUG_PRINT(("Callback Pointer: 0x%Lx\n",(unsigned long long)callback));
			new_client =
			&client_list->clients[NTB_CLIENT_23];
			client_list->number_used++;
			NTB_DEBUG_PRINT(
			("NTB: ALLOC 23 BAR\n"));
		}
	} else if (bar == NTB_BAR_45) {
		if (client_list->clients[NTB_CLIENT_45].handle == NTB_UNUSED) {
			prefix = client_list->clients[NTB_CLIENT_45].prefix;
			handle = bar | prefix;
			client_list->clients[NTB_CLIENT_45].handle = handle;
			client_list->clients[NTB_CLIENT_45].callback = callback;
			new_client =
			&client_list->clients[NTB_CLIENT_45];
			client_list->number_used++;
			NTB_DEBUG_PRINT(
			("NTB: ALLOC 45 BAR\n"));
		}
	}

	device = ntb_get_device_by_handle(handle);
	if (device != NULL) {
		NTB_DEBUG_PRINT(
			("NTB: INSIDE NTB_REGISTER_CLIENT\n"));
	} else {
		NTB_DEBUG_PRINT(("NTB: DEVICE == NULL \n"));
	}


	spin_unlock(&client_list->client_list_lock);

	if (new_client == NULL)
		return -EPERM;

	if (handle != NTB_UNUSED && client_data != NULL)
		ntb_write_client_data(new_client, client_data);

	return handle;

}
/*****************************************************************************
 * Abstract
 * Writes client data associated associated with a request to register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: None
 ****************************************************************************/
void ntb_write_client_data(struct ntb_client *client,
struct ntb_client_data *client_data)
{

	if (client != NULL) {
		client->client_data.limit = client_data->limit;
		client->client_data.translate_address =
		client_data->translate_address;
	}

}
/*****************************************************************************
 * Abstract
 * Unregisters a client associated with a valid handle.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, or < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_unregister_client(ntb_client_handle_t handle)
{
	struct ntb_client *client = NULL;
	struct ntb_clients *client_list = NULL;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(
	("NTB: INSIDE NTB_UNREGISTER_CLIENT\n"));
	device = ntb_get_device_by_handle(handle);

	if (device == NULL) {
		NTB_DEBUG_PRINT(
			("NTB: NTB_UNREGISTER_CLIENT Error: Device NULL\n"));
		return -EPERM;
	}

	client_list = ntb_get_client_list_by_handle(handle);
	if (client_list == NULL)
		return -EPERM;

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
	}

	/* Reset client values to defaults... */

	if (client != NULL) {
		NTB_DEBUG_PRINT(("NTB: RELEASE CLIENT DATA\n"));
		if (client->handle == client_list->semaphore_owner) {
			/* Resetting of sem4 necessary here */
			ntb_release_semaphore(client->handle);
		}
		if (client->handle == client_list->heartbeat_owner)
			client_list->heartbeat_owner = NTB_UNUSED;

		client->handle = NTB_UNUSED;
		client->callback = NULL;
		client->client_data.limit = 0;
		client->client_data.translate_address = 0;
	}

	spin_unlock(&client_list->client_list_lock);



	return SUCCESS;

}
/*****************************************************************************
 * Abstract
 * Writes to door bell register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, or < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_write_doorbell(ntb_client_handle_t handle, uint16_t value)
{

	struct ntb_device *device = NULL;
	int32_t check = 0;
	uint16_t doorbell = 0;

	printk("NTB: INSIDE NTB_WRITE_DOORBELL 0x%x \n", value);

	check = ntb_check_handle_validity(handle);
	device = ntb_get_device_by_handle(handle);


	if (device == NULL || check == INVALID)
		return -EPERM;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_DOORBELL device %p \n",
		device));
	NTB_DEBUG_PRINT(("NTB: doorbell offset %x value to write %x\n",
	device->doorbell_offset, value));

	ntb_lib_write_16(device->mm_regs,
	device->doorbell_offset,
	value);

	doorbell = ntb_lib_read_16(device->mm_regs,
			device->doorbell_offset);

	NTB_DEBUG_PRINT(("NTB: doorbell after write %x\n", doorbell));

	if (device != NULL) {
		NTB_DEBUG_PRINT(
		("NTB: INSIDE NTB_DOORBELL_CLIENT\n"));
	}

	return SUCCESS;
}

/*****************************************************************************
 * Abstract
 * Sets a client to be receive heart beat messages.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, < 0 unsuccessful.
 ****************************************************************************/
int32_t ntb_set_heartbeat(ntb_client_handle_t handle)
{
	struct ntb_clients *client_list = NULL;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(
	("NTB: INSIDE CLAIM HEARTBEAT OWNER\n"));

	client_list = ntb_get_client_list_by_handle(handle);
	device = ntb_get_device_by_handle(handle);

	if (device == NULL || client_list == NULL)
		return -EPERM;

	NTB_DEBUG_PRINT(("NTB: INSIDE CLAIM HEARTBEAT OWNER Link %x\n",
	device->link_status));

	device->link_status = ntb_get_link_status(handle);

	if (device->link_status == LINK_UP) {
		NTB_DEBUG_PRINT(("NTB: INSIDE CLAIM HEARTBEAT OWNER Link %x\n",
			device->link_status));
		if (client_list->heartbeat_owner == NTB_UNUSED)
			client_list->heartbeat_owner = handle;

		return SUCCESS;

	}

	return -EPERM;
}
/*****************************************************************************
 * Abstract
 * Release a client from receiving heart beat messages.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, < 0 unsuccessful.
 ****************************************************************************/
int32_t ntb_release_heartbeat(ntb_client_handle_t handle)
{
	struct ntb_clients *client_list = NULL;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(
	("NTB: INSIDE RELEASE HEARTBEAT OWNER\n"));

	client_list = ntb_get_client_list_by_handle(handle);
	device = ntb_get_device_by_handle(handle);

	if (device == NULL || client_list == NULL)
		return -EPERM;
	NTB_DEBUG_PRINT(
		("NTB: HEARTBEAT OWNER %x\n", client_list->heartbeat_owner));

	if (client_list->heartbeat_owner == handle) {
		client_list->heartbeat_owner = NTB_UNUSED;
		return SUCCESS;
	} else {
		return -EPERM;
	}


	return -EPERM;
}

/*****************************************************************************
 * Abstract
 * Allows a client to set snoop level.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_set_snoop_level(ntb_client_handle_t handle, uint32_t value)
{
	struct ntb_device *device = NULL;
	int32_t check;
	uint32_t read_val = 0;

	device = ntb_get_device_by_handle(handle);
	check = ntb_check_handle_validity(handle);

	if (device == NULL || check == INVALID)
		return -EPERM;

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
 * Return Values: 0 successful, < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_write_translate_address_value(ntb_client_handle_t handle,
uint64_t address)
{

	int32_t status = -EPERM;
	struct ntb_device *device = NULL;
	enum ntb_bar_t bar = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return status;
	NTB_DEBUG_PRINT(("NTB: WRITE TRANSLATE ADDRESS VALUE offset %Lx\n",
		address));

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

	return status;
}
/*****************************************************************************
 * Abstract
 * Writes limit value which is a size of a memory window plus the base address
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_limit(ntb_client_handle_t handle, uint64_t value)
{

	struct ntb_device *device = NULL;
	uint64_t limit_setting = 0;
	enum ntb_bar_t bar = ntb_compare_handle_bar(handle);
	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_LIMIT before any checks\n"));
	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_LIMIT dev %p \n",
	device));

	if (device == NULL)
		return -EPERM;

	if (handle & NTB_BAR_23)
		limit_setting = device->limit_max_23;
	else if (handle & NTB_BAR_45)
		limit_setting = device->limit_max_45;

	if (device != NULL) {
		NTB_DEBUG_PRINT(
		("NTB: INSIDE NTB_WRITE_LIMIT\n"));
	}

	if (limit_setting <= 0)
		return -EPERM;

	if (value == 0 || value > limit_setting) {
		NTB_DEBUG_PRINT(("NTB: Limit out of bounds \n"));
		return -EPERM;
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
 * Return Values: >= 0 successful, -EPERM unsuccessful
 ****************************************************************************/
uint64_t ntb_read_limit(ntb_client_handle_t handle)
{

	uint64_t limit = 0;
	struct ntb_device *device = NULL;
	enum ntb_bar_t bar = ntb_compare_handle_bar(handle);

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EPERM;
	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_READ_LIMIT dev %p \n",
			device));
	NTB_DEBUG_PRINT(("NTB: READ LIMIT VALUE offset %x\n",
			device->bar_23_limit_offset));

	if (device != NULL) {
		NTB_DEBUG_PRINT(
		("NTB: INSIDE NTB_READ_LIMIT\n"));

	}


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
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/
int32_t ntb_write_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *values, ntb_client_handle_t handle)
{

	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(
	("NTB: INSIDE NTB_WRITE_PAD_MANY\n"));
	if (device == NULL)
		return -EPERM;

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO))
		return -EPERM;

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
 * Return Values: >= 0,  < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_write_scratch_pad_one(uint32_t index, uint32_t value,
ntb_client_handle_t handle)
{

	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_WRITE_PAD_ONE\n"));

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EPERM;

	if (device == NULL)
		return -EPERM;
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
 * Return Values: >= 0,  < 0 unsuccessful
 ****************************************************************************/
int32_t ntb_read_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *pad, ntb_client_handle_t handle)
{

	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	if (pad == NULL)
		return -EPERM;

	if ((how_many == 0) || (how_many > NTB_TOTAL_SCRATCHPAD_NO))
		return -EPERM;

	if (device == NULL)
		return -EPERM;

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
 * Return Values: >= 0,  < 0 unsuccessful
 *****************************************************************************/
int32_t ntb_read_scratch_pad_one(uint32_t index, uint32_t *value,
ntb_client_handle_t handle)
{
	uint32_t offset = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);

	if (index >= NTB_TOTAL_SCRATCHPAD_NO)
		return -EPERM;

	if (device == NULL)
		return -EPERM;

	if (value == NULL)
		return -EPERM;


	offset = device->scratchpad_offset_read + (index * sizeof(int32_t));
	*value = ntb_lib_read_32(device->mm_regs, offset);


	return SUCCESS;

}

/*****************************************************************************
 * Abstract
 * Gets stored link status.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: > 0,  <= 0 unsuccessful
 *****************************************************************************/
int16_t ntb_get_link_status(ntb_client_handle_t handle)
{
	struct ntb_device *device = NULL;
	int16_t link_status = 0;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EPERM;

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
 * Reads limit register.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: None
 *****************************************************************************/
int32_t ntb_set_link_status(ntb_client_handle_t handle, enum link_t status)
{
	int32_t read_status = 0;
	struct ntb_device *device = NULL;

	NTB_DEBUG_PRINT(("NTB: Set Link Status: LinkStatus: 0x%x\n",
	status));

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EPERM;

	if ((status & ~LINK_ENABLE) != 0)
		return -EPERM;

	read_status = ntb_lib_read_32(device->mm_regs,
	device->link_control_offset);

	NTB_DEBUG_PRINT(("NTB: Read Link Status: LinkStatus: 0x%x\n",
	read_status));

	read_status &= ~LINK_ENABLE;
	read_status |= status;
	ntb_lib_write_32(device->mm_regs,
		device->link_control_offset, read_status);

		return SUCCESS;

}

/*****************************************************************************
 ****************************************************************************/
int32_t ntb_obtain_semaphore(ntb_client_handle_t handle)
{

	uint32_t value = 0;
	struct ntb_device *device = NULL;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EPERM;

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_OBTAIN_SEMA4\n"));
	value = ntb_lib_read_32(device->mm_regs, device->semaphore_offset);
	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_OBTAIN_SEMA4 %x\n", value));

	if (value == NTB_UNUSED) {
		device->client_list.semaphore_owner = handle;
		return SUCCESS;
	}

	return -EAGAIN; /* try again */

}
/*****************************************************************************
*****************************************************************************/
void ntb_release_semaphore(ntb_client_handle_t handle)
{

	struct ntb_device *device = NULL;
	device = ntb_get_device_by_handle(handle);
	/* && device->link_status == LINK_UP */

	NTB_DEBUG_PRINT(("NTB: INSIDE NTB_RELEASE_SEMA4\n"));

	if (device != NULL) {
		if (device->client_list.semaphore_owner == handle
		&& device->client_list.semaphore_owner
		!= NTB_UNUSED && device->link_status == LINK_UP) {
			ntb_lib_write_32(device->mm_regs,
			device->semaphore_offset, SEMAPHORE_ONE_TO_CLEAR);
			device->client_list.semaphore_owner = NTB_UNUSED;
		}
	}
}

/*****************************************************************************
*****************************************************************************/

int32_t ntb_get_bar01(ntb_client_handle_t handle, void **pvalue)
{
  return 0;
}
