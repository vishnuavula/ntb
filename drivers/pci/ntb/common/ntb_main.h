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

#ifndef NTB_MAIN_H_
#define NTB_MAIN_H_

#include "ntb_lib.h"
#include "ntb_api.h"

#define NTB_B2B_DEVICE_ID        0x3725
#define NTB_CLASSIC_DEVICE_ID    0x3726
#define NTB_ROOT_PORT_DEVICE_ID  0x3721
#define NTB_VENDOR_ID            0x8086

#define NTB_MSIX_NAME "NTB_MSIX"

#define BAR_TWO_OFFSET	     0x18
#define BAR_THREE_OFFSET     0x1C
#define BAR_FOUR_OFFSET	     0x20
#define BAR_FIVE_OFFSET      0x24
#define NO_CLIENTS           0x02

#define NTB_DELAY            0x1000
#define TIMEOUT              1000
#define PM_ACK_23            0x08
#define PM_ACK_45            0x80
#define PM_ACK_BOTH          0x800

#define BAR_NO              3
#define DEVICE_BAR_01       0
#define DEVICE_BAR_23       1
#define DEVICE_BAR_45       2
#define PCI_CONFIG_SPACE_01 0
#define PCI_CONFIG_SPACE_23 2
#define	PCI_CONFIG_SPACE_45 4

/* Flip from 1 to 0 to get rid of all the debug printing */
#ifdef NTB_DEBUG
//#define NTB_DEBUG_PRINT(args) printk args
#define NTB_DEBUG_PRINT(args)
#else
#define NTB_DEBUG_PRINT(args)
#endif

enum ntb_irq_handle_index_t {
	NTB_XXX_0     = 0,
	NTB_XXX_1     = 1,
	NTB_XXX_2     = 2,
	NTB_INTX_ONLY = 3
};

typedef irqreturn_t (*ntb_irq_handler_t)(int irq, void *data,
		struct pt_regs *regs);

#define NTB_ID_PROC0 0x78846600
#define NTB_ID_PROC1 0x78846601
#define MASK_36_BIT  0xFFFFFFF0

#define SHIFT_LOWER  0xFFF
#define UPPER_LIMIT  0x3FFFFFFF
#define FULL_LIMIT   0x3FFFFFFFFFFFF000

struct ntb_clients {
  uint32_t number_used; /*number of BARS used */
  int32_t heartbeat_owner;
  int32_t semaphore_owner;
  struct ntb_client clients[2];
  spinlock_t client_list_lock;
};

struct ntb_device {

	/* Has a device been found for this processor NOT_ENUMERATED,
	 * PROC_0, PROC1*/
	int32_t device_id;
	int32_t device_processor;

	uint64_t limit_base_23;
	uint64_t limit_base_45;
	uint64_t limit_max_23;
	uint64_t limit_max_45;

	/* MSIx related values */
	int16_t msixmsgctrl;
	struct pci_dev *dev;
	int16_t msix_entry_no;
	int16_t intx_entry_no;
	int16_t msi_entry_no;
	struct msix_entry msix_entries[NTB_MSIX_MAX_VECTORS];

	/* Virtual address for the memory mapped regs for BAR 0/1 */
	struct ntb_mm_regs *mm_regs;
	void *pci_bar_23_virt;
	void *pci_bar_45_virt;

	/* Physical addresses of BARs
	 * Also suspend and resume */
	uint64_t pci_bar[3];
	int64_t bar_23_translate;
	int64_t bar_45_translate;
	int64_t bar_23_limit;
	int64_t bar_45_limit;
	int32_t cntrl;

	/* List of clients associated with BARs */
	struct ntb_clients client_list;

	ntb_irq_handler_t handlers[4];

	/* Offsets */
	int32_t doorbell_offset;
	int32_t bar_23_translate_offset;
	int32_t bar_45_translate_offset;
	int32_t bar_23_limit_offset;
	int32_t bar_45_limit_offset;
	int32_t scratchpad_offset_write;
	int32_t scratchpad_offset_read;
	int32_t semaphore_offset;
	int32_t link_status_offset;
	int32_t link_control_offset;

	uint32_t link_status;
	uint16_t heartbeat_bit;
	uint16_t client_pm_event_bit;
	uint16_t client_pm_acknowledgement_bit;
	uint16_t bits_23;
	uint16_t bits_45;

	uint16_t client_pm_acknowledgement;
	uint32_t test_value;


};

void ntb_get_limit_settings(struct pci_dev *dev, enum ntb_bar_t bar,
uint64_t *dev_limit_setting, uint64_t *dev_max);

/* NTB Device Access Functions */
uint32_t ntb_get_number_devices(void);
void ntb_increment_number_devices(void);
void ntb_write_client_data(struct ntb_client *client,
struct ntb_client_data *client_data);
enum ntb_bar_t ntb_compare_handle_bar(ntb_client_handle_t handle);
int32_t ntb_compare_handle_snoop_level(ntb_client_handle_t handle,
uint32_t value);
struct ntb_clients *ntb_get_client_list(enum ntb_proc_id_t id);
struct ntb_clients *ntb_get_client_list_by_handle(ntb_client_handle_t handle);
struct ntb_device *ntb_get_device(enum ntb_proc_id_t id);
struct ntb_device *ntb_get_device_by_handle(ntb_client_handle_t handle);
int32_t ntb_check_handle_validity(ntb_client_handle_t handle);
void ntb_initialize_number_devices(void);

#endif /*NTB_MAIN_H_*/

