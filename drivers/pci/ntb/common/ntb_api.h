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
******************************************************************************/

/* This file defines the API exposed by the NTB Driver */

#ifndef NTB_API_H_
#define NTB_API_H_


/* Snoop Levels */
#define NTB_23_SEC_TO_PRI_TLP                     0x00000000
#define NTB_23_SEC_TO_PRI_FORCE_SNOOP             0x00000004
#define NTB_23_SEC_TO_PRI_FORCE_NO_SNOOP          0x00000008

#define NTB_23_PRI_TO_SEC_TLP                     0x00000000
#define NTB_23_PRI_TO_SEC_FORCE_SNOOP             0x00000010
#define NTB_23_PRI_TO_SEC_FORCE_NO_SNOOP          0x00000020

#define NTB_45_SEC_TO_PRI_TLP                     0x00000000
#define NTB_45_SEC_TO_PRI_FORCE_SNOOP             0x00000040
#define NTB_45_SEC_TO_PRI_FORCE_NO_SNOOP          0x00000080

#define NTB_45_PRI_TO_SEC_TLP                     0x00000000
#define NTB_45_PRI_TO_SEC_FORCE_SNOOP             0x00000100
#define NTB_45_PRI_TO_SEC_FORCE_NO_SNOOP          0x00000200

/* The most we can have is two clients per NTB side, one for
 * each BAR. One client can possess both bars.
 */

#define INVALID_LIMIT_VALUE  -1
#define NTB_BUS_NUMBER_P0     0x00
#define NTB_UNUSED            0
#define NTB_HEARTBEAT_UNUSED -1
#define VALID                0
#define INVALID              -1
#define ACKNOWLEDGED         0

#define NTB_RESET_POLICY     0x01
#define NTB_ADD_POLICY       0x02
#define NTB_GET_POLICY       0x03

#define NTB_MSIX_MAX_VECTORS       4
#define MAX_DEVICES                2
#define NUMBER_VALID_HANDLES       4

#ifndef NTB_SCRATCHPAD_REGS_DEFINED
#define NTB_SCRATCHPAD_REGS_DEFINED

#define NTB_TOTAL_SCRATCHPAD_NO 16

/* struct to hold scratch pad registers */
struct scratchpad_registers {
	uint32_t registers[NTB_TOTAL_SCRATCHPAD_NO];
};

#endif /* NTB_SCRATCHPAD_REGS_DEFINED */

/* Link Related Defines */
#define LINK_RETRAIN       0x20
#define LINK_ENABLE        0x00
#define LINK_STATUS_ACTIVE 0x2000

/* Set Link Up NTBCNTL */
enum link_t {
	SET_LINK_DOWN = 0x02,
	SET_LINK_UP   = LINK_ENABLE

};
/* Status */
#define LINK_UP   0x01
#define LINK_DOWN 0x00

/**
 * enum ntb_bar_t - BAR types which are used by API for handle creation.
 * @NTB_BAR_01: Base Address 0/1 (only available in certain functions)
 * @NTB_BAR_23: Base Address 2/3
 * @NTB_BAR_45: Base Address 4/5
 *
 * These enum elements are used by the internal functions to generate
 * handles for obtaining access to BARS. Also, these values can be
 * passed into  functions to indicate which BAR is being requested.
 */
enum ntb_bar_t {
	NTB_BAR_01    = 0x00000000,
	NTB_BAR_23    = 0x00010000,
	NTB_BAR_45    = 0x00020000
};



/** COMMENT TBD
**/
typedef int ntb_client_handle_t;

/** COMMENT TBD
**/
typedef void (*ntb_callback_t)(
ntb_client_handle_t handle,
/**< client handle */
int16_t doorbell_value,
/**< ptr to a door bell value */
struct scratchpad_registers pad
/**< ptr to a scratch pad registers */);



struct ntb_semaphore {
	ntb_client_handle_t handle; /* who has the semaphore */
};

/**
 * struct ntb_client_data - initial values for NTB client.
 * @limit: limit value
 * @translate_address: address translation value
 * @power_event_acknowledgement_bit: bit recognized as acknowledgment
 *
 */
struct ntb_client_data {
	uint64_t limit;
	uint64_t translate_address;
};



#define NOT_ENUMERATED   -1
#define ENUMERATED       1
#define INDEX_0           0
#define INDEX_1           1



struct ntb_client {
	ntb_client_handle_t handle;
	ntb_callback_t callback;
	struct ntb_client_data client_data;
	uint16_t bdf;
};


/**
 * ntb_obtain_semaphore - client driver uses to obtain a semaphore.
 * @handle: handle granted by ntb_client_register
 */
int32_t ntb_obtain_semaphore(ntb_client_handle_t handle);

/**
 * ntb_release_semaphore - releases semaphore granted by ntb_obtain_semaphore.
 * @handle: handle granted by ntb_client_register
 *
 */
int32_t ntb_release_semaphore(ntb_client_handle_t handle);

/**
 * ntb_write_scratch_pad_many - writes to 0 to 16 scratch pad registers.
 * @how_many: how many 32-bit registers
 * @values: scratch pad registers' values
 * @handle: handle granted by ntb_client_register
 *
 * Client driver uses this function to write to scratch pad registers
 * after obtaining a semaphore.
 */
int32_t ntb_write_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *values,
ntb_client_handle_t handle);

/**
 * ntb_write_scratch_pad_one - writes one scratch pad register.
 * @index: index into the array of scratch pad registers (0-15)
 * @value: scratch pad register value
 * @handle: handle granted by ntb_client_register
 *
 * Client driver uses this function to write to scratch pad registers
 * after obtaining a semaphore.
 */
int32_t ntb_write_scratch_pad_one(uint32_t index, uint32_t value,
ntb_client_handle_t handle);

/**
 * ntb_write_limit - writes value of the limit.
 * @handle: handle granted by ntb_client_register
 * @value: scratch pad register value
 *
 * Base address is added to this value to arrive at the upper bound
 * address of a memory window. The NTB driver uses the bar and handle
 * values to determine if a client is permitted to modify values associated
 * with a particular BAR.
 */
int32_t ntb_write_limit(ntb_client_handle_t handle, uint64_t value);

/**
 * ntb_read_limit - reads value of the limit register.
 * @handle: handle granted by ntb_client_register
 *
 * Base address is added to this value to arrive at the upper bound
 * address of a memory window.
 */
uint64_t ntb_read_limit(ntb_client_handle_t handle);

/**
 * ntb_write_translate - writes translation address value.
 * @handle: handle granted by ntb_client_register
 * @address: physical address value for the translation address
 *
 * Base address is added to this value to arrive at the upper bound
 * address of a memory window.
 */
int32_t ntb_write_translate(ntb_client_handle_t handle,
uint64_t address);

/**
 * ntb_write_doorbell - writes value to door bell register.
 * @handle: handle granted by ntb_client_register
 * @value: value for the door bell
 *
 */
int32_t ntb_write_doorbell(ntb_client_handle_t handle,
uint16_t value
);

/**
 * ntb_read_scratch_pad_many - reads multiple scratch pad registers.
 * @how_many: number of registers to read
 * @pad: ptr to a pad struct to hold register values
 * @handle: handle granted by ntb_client_register
 *
 */
int32_t ntb_read_scratch_pad_many(uint32_t how_many,
struct scratchpad_registers *pad,
ntb_client_handle_t handle);

/**
 * ntb_read_scratch_pad_one - reads one scratch pad registers.
 * @index: number of registers to read
 * @value: ptr to a pad struct to hold register values
 * @handle: handle granted by ntb_client_register
 *
 */
int32_t ntb_read_scratch_pad_one(uint32_t index,
uint32_t *value,
ntb_client_handle_t handle
);

/**
 * ntb_register_client - registers a client driver with the NTB.
 * @bar: BAR(s) requested
 * @callback: pointer to a callback function of type ntb_callback_t
 * @bdf: bus,device, function (bus, 8 bits, device, 5 bits, function, 3 bits)
 * @client_data: sets limit register and translation addresses
 *
 * Returns a valid handle. If the BAR(s) requested are already taken,
 * the function will not return a valid BAR (0/-1 unsuccessful).
 */
ntb_client_handle_t ntb_register_client(enum ntb_bar_t bar,
ntb_callback_t callback,
uint16_t bdf,
struct ntb_client_data *client_data
);

/**
 * ntb_unegister_client - unregisters a client driver with the NTB.
 * @handle: handle granted by ntb_client_register
 *
 */
int32_t ntb_unregister_client(ntb_client_handle_t handle);

/**
 * ntb_set_snoop_level - sets the snoop level.
 * @handle: handle granted by ntb_client_register
 * @value: valid value for snoop level.
 *
 */
int32_t ntb_set_snoop_level(ntb_client_handle_t handle,
uint32_t value);

/**
 * ntb_get_number_devices - returns number of NTB devices enumerated.
 *
 */
uint32_t ntb_get_number_devices(void);

/**
 * ntb_get_link_status - returns the link status.
 * @handle: handle granted by ntb_client_register
 *
 */
int16_t ntb_get_link_status(ntb_client_handle_t handle);

/**
 * ntb_set_link_status - sets the link status.
 * @handle: handle granted by ntb_client_register
 * @status: link status value (SET_LINK_UP, SET_LINK_DOWN)
 *
 */
int32_t ntb_set_link_status(ntb_client_handle_t handle, enum link_t status);

/**
 * ntb_get_bar_address - returns the pci resource.
 * @handle: handle granted by ntb_client_register
 * @bar: bar address requested
 *
 */
uint64_t ntb_get_bar_address(ntb_client_handle_t handle,
enum ntb_bar_t bar);

/**
 * ntb_client_suspend - allows client to signal NTB it is ready to suspend.
 * @handle: handle granted by ntb_client_register.
 */
int32_t ntb_client_suspend(ntb_client_handle_t handle);

/* NEW APIS*/

/**
 * ntb_add_policy - adds a new doorbell policy.
 * @handle: handle granted by ntb_client_register
 * @heartbeat_bit: which bits will be used for heartbeat messages
 * @bar_bits: which bits will be used for the client's other protocol messages
 * @power_event_bit: PM policy, other side of NTB must ready itself for PM event
 * @power_notification_bit: PM policy, acknowledgement of PM even
 *
 * The use of doorbell bits is referred to as the 'policy'
 * of the doorbell. Each client can add policy by using the
 * the function parameters to indicate specific bits.
 */
int32_t ntb_add_policy(ntb_client_handle_t handle,
uint16_t heartbeat_bit, uint16_t bar_bits,
uint16_t power_event_bit,
uint16_t power_notification_bit);

/**
 * ntb_reset_policy - resets the doorbell policy for a particular client.
 * @handle: handle granted by ntb_client_register
 *
 * The use of doorbell bits is referred to as the 'policy'
 * of the doorbell. Each client can add policy by using the
 * the function parameters to indicate specific bits.
 */
int32_t ntb_reset_policy(ntb_client_handle_t handle);

/**
 * ntb_get_policy - returns the policy, indicating which bits are already set.
 * @handle: handle granted by ntb_client_register
 *
 * The use of doorbell bits is referred to as the 'policy'
 * of the doorbell. Each client can add policy by using the
 * the function parameters to indicate specific bits.
 */
uint16_t ntb_get_policy(ntb_client_handle_t handle);

/**
 * ntb_get_next_bdf - returns next available BDF/BAR combination.
 * @next_bdf: ptr to store next available BDF
 * @next_bar: ptr to next BAR associated with BDF
 *
 * Use this function within a loop. The parameter cmd should contain
 * a value of zero for the first iteration and a value of non-zero
 * for subsequent iterations.
 *
 * The loop should be stopped when a value of 0 is returned in
 * next_bar, as a value of 0 is not a BAR available for clients.
 *
 *
 */
int32_t ntb_get_next_bdf(uint16_t *next_bdf,
uint32_t *next_bar);
/**
 * ntb_get_number_unused_bdfs - number of BDF/BAR combinations available.
 *
 */
int32_t ntb_get_number_unused_bdfs(void);

/**
 * ntb_write_wccntrl_bit - writes to wccntrl register.
 * @handle: handle granted by ntb_client_register
 *
 * Only bit one can be set in this register (bit 0).
 */
int32_t ntb_write_wccntrl_bit(ntb_client_handle_t handle);

/**
 * ntb_read_wccntrl_bit - returns wnctrl register value.
 * @handle: handle granted by ntb_client_register
 *
 */
uint32_t ntb_read_wccntrl_bit(ntb_client_handle_t handle);

/**
 * ntb_write_remote_translate - sets secondary translate register.
 * @handle: handle granted by ntb_client_register
 * @bar: the secondary translate to write
 * @address: address for secondary translate
 *
 */
int32_t ntb_write_remote_translate(ntb_client_handle_t handle,
enum ntb_bar_t bar,
uint64_t address);

/**
 * ntb_read_remote_translate - returns translate (primnary) value.
 * @handle: handle granted by ntb_client_register
 * @bar: the secondary translate to read.
 */

int64_t ntb_read_remote_translate(ntb_client_handle_t handle,
enum ntb_bar_t bar);

/*
 * ASSUMPTION: These APIs are being used from the primary side
 * of the NTB classic and B2B drivers. These are not root port
 * side APIs.
 *
 */

/**
 * ntb_write_remote_doorbell_mask - sets secondary side's mask values.
 * @handle: handle granted by ntb_client_register
 * @mask: parameter for mask bits
 *
 */
int32_t ntb_write_remote_doorbell_mask(ntb_client_handle_t handle,
uint32_t mask);

/**
 * ntb_read_remote_doorbell_mask - returns secondary side's mask values.
 * @handle: handle granted by ntb_client_register
 *
 */
uint16_t ntb_read_remote_doorbell_mask(ntb_client_handle_t handle);

/**
 * ntb_write_remote_limit - sets secondary limit register.
 * @handle: handle granted by ntb_client_register
 * @bar: the secondary limit being requested
 * @value: value for secondary limit
 *
 */
int32_t ntb_write_remote_limit(ntb_client_handle_t handle,
enum ntb_bar_t bar,
uint64_t value);

/**
 * ntb_read_remote_limit - returns secondary limit register value.
 * @handle: handle granted by ntb_client_register
 * @bar: the secondary limit being read.
 */
int64_t ntb_read_remote_limit(ntb_client_handle_t handle,
enum ntb_bar_t bar);

/**
 * ntb_read_remote_bar - reads base address register.
 * @handle: handle granted by ntb_client_register
 * @bar: BAR to write
 *
 * Used from the primary side.
 */
uint64_t ntb_read_remote_bar(ntb_client_handle_t handle, enum ntb_bar_t bar);

/**
 * ntb_write_remote_bar- writes base address register.
 * @handle: handle granted by ntb_client_register
 * @bar: BAR to write
 * @address: address value
 *
 * Used from the primary side.
 */
int32_t ntb_write_remote_bar(ntb_client_handle_t handle, enum ntb_bar_t bar,
uint64_t address);


/*****************************************************************************
 *@ingroup NTB_CLIENT_API
 *Structure to hold functions exported to the client driver.
 *
 *****************************************************************************/
struct ntb_api_export {

	ntb_client_handle_t (*ntb_register_client)(enum ntb_bar_t bar,
	ntb_callback_t callback,
	uint16_t bdf,
	struct ntb_client_data *client_data);

	int32_t (*ntb_unregister_client)(ntb_client_handle_t handle);

	int32_t (*ntb_write_limit)(ntb_client_handle_t handle,
	uint64_t value);

	uint64_t (*ntb_read_limit)(ntb_client_handle_t handle);

	int32_t (*ntb_write_scratch_pad_many)(uint32_t how_many,
	struct scratchpad_registers *values,
	ntb_client_handle_t handle);

	int32_t (*ntb_write_scratch_pad_one)(uint32_t index, uint32_t value,
	ntb_client_handle_t handle);

	int32_t (*ntb_read_scratch_pad_many)(uint32_t how_many,
	struct scratchpad_registers *pad,
	ntb_client_handle_t handle);

	int32_t (*ntb_read_scratch_pad_one)(uint32_t index, uint32_t *value,
	ntb_client_handle_t handle);

	int32_t (*ntb_write_translate)(
	ntb_client_handle_t handle,
	uint64_t address);

	int32_t(*ntb_write_doorbell)(ntb_client_handle_t handle,
	uint16_t value);

	int32_t (*ntb_obtain_semaphore)(ntb_client_handle_t handle);

	int32_t(*ntb_release_semaphore)(ntb_client_handle_t handle);

	int32_t (*ntb_set_snoop_level)(ntb_client_handle_t handle,
	uint32_t value);

	uint32_t (*ntb_get_number_devices)(void);

	int16_t (*ntb_get_link_status)(int32_t handle);

	int32_t (*ntb_set_link_status)(int32_t handle, enum link_t status);

	uint64_t (*ntb_get_bar_address)(ntb_client_handle_t handle,
	enum ntb_bar_t bar);

	int32_t (*ntb_client_suspend)(ntb_client_handle_t handle);

	int32_t (*ntb_add_policy)(ntb_client_handle_t handle,
	uint16_t heartbeat_bit, uint16_t bar_bits,
	uint16_t power_event_bit,
	uint16_t power_notification_bit);

	int32_t (*ntb_reset_policy)(ntb_client_handle_t handle);

	uint16_t (*ntb_get_policy)(ntb_client_handle_t handle);

	int32_t (*ntb_get_next_bdf)(uint16_t *next_bdf,
	uint32_t *next_bar);

	int32_t (*ntb_get_number_unused_bdfs)(void);

	int32_t (*ntb_write_wccntrl_bit)(ntb_client_handle_t handle);

	uint32_t (*ntb_read_wccntrl_bit)(ntb_client_handle_t handle);

	int32_t (*ntb_write_remote_translate)(
	ntb_client_handle_t handle,
	enum ntb_bar_t bar,
	uint64_t address);

	int64_t (*ntb_read_remote_translate)(ntb_client_handle_t handle,
	enum ntb_bar_t bar);

};

int32_t ntb_get_api(struct ntb_api_export *api);


typedef int32_t (*ntb_get_api_t)(struct ntb_api_export *);





#endif /* NTB_API_H */
