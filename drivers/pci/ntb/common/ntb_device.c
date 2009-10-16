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

#include "ntb_main.h"

static uint32_t g_number_devices;
static struct ntb_device g_ntb[MAX_DEVICES];

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
	ntb_client_handle_t valid_handle_device[NUMBER_VALID_HANDLES] = {
		g_ntb[PROC_0].client_list.clients[NTB_CLIENT_23].handle,
		g_ntb[PROC_0].client_list.clients[NTB_CLIENT_45].handle,
		g_ntb[PROC_1].client_list.clients[NTB_CLIENT_23].handle,
		g_ntb[PROC_1].client_list.clients[NTB_CLIENT_45].handle,
	};

	if (handle == NTB_UNUSED)
		return INVALID;

	for (i = 0 ; i < NUMBER_VALID_HANDLES; i++)
		if (valid_handle_device[i] == handle)
			return VALID;

	return INVALID;
}



/*****************************************************************************
 * Description
 * Returns client list per associated processor.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: client_list ptr
 ****************************************************************************/
struct ntb_clients *ntb_get_client_list(enum ntb_proc_id_t id)
{
	if (id == PROC_0) {
		if (g_ntb[PROC_0].device_processor != NOT_ENUMERATED)
			return &g_ntb[PROC_0].client_list;
	} else if (id == PROC_1) {
		if (g_ntb[PROC_1].device_processor != NOT_ENUMERATED)
			return &g_ntb[PROC_1].client_list;
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
	if (VALID == check) {
		if (handle & NTB_CLIENT_P0) {
			if (g_ntb[PROC_0].device_processor != NOT_ENUMERATED)
				return &g_ntb[PROC_0].client_list;
		} else if (handle & NTB_CLIENT_P1) {
			if (g_ntb[PROC_1].device_processor != NOT_ENUMERATED)
				return &g_ntb[PROC_1].client_list;
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
struct ntb_device *ntb_get_device(enum ntb_proc_id_t id)
{
	if (id == PROC_0) {
		NTB_DEBUG_PRINT(("NTB: GET DEVICE %p\n", &g_ntb[PROC_0]));
		return &g_ntb[PROC_0];
	} else if (id == PROC_1) {
		NTB_DEBUG_PRINT(("NTB: GET DEVICE %p\n", &g_ntb[PROC_1]));
		return &g_ntb[PROC_1];
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
inline struct ntb_device *ntb_get_device_by_handle(ntb_client_handle_t handle)
{
	int32_t check = ntb_check_handle_validity(handle);
	if (check == VALID) {
		if (handle & NTB_CLIENT_P0) {
			if (g_ntb[PROC_0].device_processor != NOT_ENUMERATED)
				return &g_ntb[PROC_0];
		} else if (handle & NTB_CLIENT_P1) {
			if (g_ntb[PROC_1].device_processor != NOT_ENUMERATED)
				return &g_ntb[PROC_1];
		}
	}

	return NULL;
}


/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
int32_t ntb_set_policy(ntb_client_handle_t handle,
uint16_t heartbeat_bit, uint16_t bits_23, uint16_t bits_45,
uint16_t power_event_bit,
uint16_t power_notification_bit)
{
	struct ntb_device *device = NULL;
	struct ntb_clients *client_list = NULL;
	int16_t link_status = LINK_DOWN;

	device = ntb_get_device_by_handle(handle);
	if (device == NULL)
		return -EPERM;

	client_list = ntb_get_client_list(device->device_processor);
	if (client_list == NULL)
		return -EPERM;
	link_status = ntb_get_link_status(handle);
	if (link_status == LINK_UP) {
		if (device) {
			/* All bits are mutually exclusive, non-overlapping */
			if ((heartbeat_bit &
			bits_23 &
			bits_45 &
			power_event_bit &
			power_notification_bit) != 0)
				return -EPERM;

			device->heartbeat_bit = heartbeat_bit;
			device->bits_23 = bits_23;
			device->bits_45 = bits_45;
			device->client_pm_event_bit = power_event_bit;
			device->client_pm_acknowledgement_bit =
			power_notification_bit;

			NTB_DEBUG_PRINT(
			("NTB: SET POLICY: heart beat bit: 0x%x\n",
					device->heartbeat_bit));
			NTB_DEBUG_PRINT(("NTB: SET POLICY: bits_23: 0x%x\n",
					device->bits_23));
			NTB_DEBUG_PRINT(("NTB: SET POLICY: bits_45: 0x%x\n",
					device->bits_45));
			NTB_DEBUG_PRINT(
			("NTB: SET POLICY: PM Event bit: 0x%x\n",
					device->client_pm_event_bit));
			NTB_DEBUG_PRINT(
			("NTB: SET POLICY: PM Ack bit: 0x%x\n",
			device->client_pm_acknowledgement_bit));

			if (handle ==
			client_list->clients[NTB_CLIENT_23].handle)
				client_list->heartbeat_owner = NTB_CLIENT_23;
			else if (handle ==
			client_list->clients[NTB_CLIENT_45].handle)
				client_list->heartbeat_owner = NTB_CLIENT_45;

		}
	} else {
		NTB_DEBUG_PRINT(("NTB: SET POLICY: LINK STATUS: NOT UP\n"));
		return -EPERM;
	}

	return 0;
}

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
uint64_t ntb_get_bar_address(ntb_client_handle_t handle,
enum ntb_bar_t bar)
{
	struct ntb_device *device = ntb_get_device_by_handle(handle);

	if (device != NULL && (bar == NTB_BAR_23))
		return device->pci_bar[DEVICE_BAR_23];

	if (device != NULL && (bar == NTB_BAR_45))
		return device->pci_bar[DEVICE_BAR_45];

	return 0;
}

/*****************************************************************************
 * See ntb_main.h
 *****************************************************************************/
int64_t pow_exp(int32_t exponent)
{
	int64_t ret = 0;
	int32_t i = 0;
	int64_t operand = 2;

	if (exponent == 0)
		return 1;

	if (exponent == 1)
		return operand;

	if (exponent == 2) {
		ret = (operand * operand);
		return ret;
	}

	ret = operand;
	for (i = 0; i < (exponent - 1); i++)
		ret = ret * operand;

	return  ret;
}
/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
void ntb_get_limit_settings(struct pci_dev *dev, enum ntb_bar_t bar,
uint64_t *dev_limit_base, uint64_t *dev_max)
{
	uint32_t upper = 0;
	uint32_t lower = 0;
	uint64_t limit_setting = 0;
	uint8_t max_setting = 0;
	outb(0xDD, 0x80);

	NTB_DEBUG_PRINT(("NTB: DEV PTR %p\n", dev));

	outb(0xFF, 0x80);

	if (bar == NTB_BAR_23) {
		pci_read_config_dword(dev, BAR_TWO_OFFSET,
					&lower);
		pci_read_config_dword(dev, BAR_THREE_OFFSET,
					&upper);
		pci_read_config_byte(dev, NTB_LIMIT_MAX_23,
					&max_setting);


	} else if (bar == NTB_BAR_45) {
		pci_read_config_dword(dev, BAR_FOUR_OFFSET,
					&lower);
		pci_read_config_dword(dev, BAR_FIVE_OFFSET,
					&upper);
		pci_read_config_byte(dev, NTB_LIMIT_MAX_45,
					&max_setting);
	}
	NTB_DEBUG_PRINT(("NTB: Getting Limit Setting lower %x upper%x\n",
	lower, upper));
	limit_setting =  upper;
	limit_setting = limit_setting << BIT_SHIFT_32;
	limit_setting = limit_setting | lower;
	limit_setting = limit_setting & ~SHIFT_LOWER;

	NTB_DEBUG_PRINT(("NTB: max %x\n", max_setting));

#if 0
	if (bar == NTB_BAR_23) {
		pci_read_config_dword(dev, BAR_TWO_OFFSET,
			&limit_setting);
		pci_read_config_byte(dev, NTB_LIMIT_MAX_23,
			&max_setting);
	} else if (bar == NTB_BAR_45) {
		pci_read_config_dword(dev, BAR_FOUR_OFFSET,
			&limit_setting);
		pci_read_config_byte(dev, NTB_LIMIT_MAX_45,
			&max_setting);
	}
	limit_setting = limit_setting & ~SHIFT_LOWER;
#endif


	*dev_limit_base = limit_setting;
	*dev_max = pow_exp(max_setting);

	NTB_DEBUG_PRINT(("NTB: limit base %Lx\n", *dev_limit_base));
	NTB_DEBUG_PRINT(("NTB: max limit %Lx\n", *dev_max));


}
