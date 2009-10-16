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
 *****************************************************************************/


#include "../common/ntb_main.h"

MODULE_LICENSE("Dual BSD/GPL");

/* PRIVATE */
static int ntb_init(void);
static void ntb_exit(void);
static void ntb_device_init(struct ntb_device *device,
uint16_t bdf, uint32_t device_index);
static int32_t ntb_probe(struct pci_dev *dev, const struct pci_device_id *id);
static void ntb_remove(struct pci_dev *dev);

#ifdef RH_5
static irqreturn_t ntb_irq_xxx(int irq, void *data, struct pt_regs *regs);
#else
static irqreturn_t ntb_irq_xxx(int irq, void *data);
#endif
static void callback_tasklet_func(unsigned long data);
static int32_t ntb_set_interrupts(struct ntb_device *device);
static void ntb_release_interrupts(struct ntb_device *device);

static int32_t ntb_suspend(struct pci_dev *dev, pm_message_t state);
static int32_t ntb_resume(struct pci_dev *dev);

static int32_t ntb_get_bar_addresses(struct ntb_device *device);
static void ntb_release_bar_addresses(struct ntb_device *device);


static struct pci_device_id pci_ids[] = { {
	PCI_DEVICE(NTB_VENDOR_ID, NTB_B2B_DEVICE_ID) }, { 0, },
};

static struct pci_driver ntb_pci_ops = {
	.name     = "NTBB2B",
	.id_table = pci_ids,
	.probe    = ntb_probe,
	.remove   = ntb_remove,
	.suspend  = ntb_suspend,
	.resume   = ntb_resume
};

static struct ntb_api_export ntb_api = {
	.ntb_register_client               = ntb_register_client,
	.ntb_unregister_client             = ntb_unregister_client,
	.ntb_write_limit                   = ntb_write_limit,
	.ntb_read_limit                    = ntb_read_limit,
	.ntb_write_scratch_pad_many        = ntb_write_scratch_pad_many,
	.ntb_write_scratch_pad_one         = ntb_write_scratch_pad_one,
	.ntb_read_scratch_pad_many         = ntb_read_scratch_pad_many,
	.ntb_read_scratch_pad_one          = ntb_read_scratch_pad_one,
	.ntb_write_translate               = ntb_write_translate,
	.ntb_write_doorbell                = ntb_write_doorbell,
	.ntb_obtain_semaphore              = ntb_obtain_semaphore,
	.ntb_release_semaphore             = ntb_release_semaphore,
	.ntb_set_snoop_level               = ntb_set_snoop_level,
	.ntb_get_number_devices            = ntb_get_number_devices,
	.ntb_get_link_status               = ntb_get_link_status,
	.ntb_set_link_status               = ntb_set_link_status,
	.ntb_get_bar_address               = ntb_get_bar_address,
	.ntb_client_suspend                = ntb_client_suspend,
	.ntb_add_policy                    = ntb_add_policy,
	.ntb_reset_policy                  = ntb_reset_policy,
	.ntb_get_policy                    = ntb_get_policy,
	.ntb_get_next_bdf                  = ntb_get_next_bdf,
	.ntb_get_number_unused_bdfs        = ntb_get_number_unused_bdfs,
	.ntb_write_wccntrl_bit             = ntb_write_wccntrl_bit,
	.ntb_read_wccntrl_bit              = ntb_read_wccntrl_bit,
	.ntb_write_remote_translate        = ntb_write_remote_translate,
	.ntb_read_remote_translate         = ntb_read_remote_translate

};

static spinlock_t lock_pm_event_check;   /*  lock for pm acknowledgment */
static spinlock_t lock_callback_tasklet; /* lock for door bell tasklet work */
static int16_t g_tasklet_data[MAX_DEVICES];
DECLARE_TASKLET(callback_tasklet, callback_tasklet_func,
(unsigned long) &g_tasklet_data);

int icounter = 1;

static char *g_ntb_name = "NTBB2B";


/*****************************************************************************
 * Description
 * Called during loading.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/
static int ntb_init(void)
{

	/*Initialization Phase */
	int32_t ret = SUCCESS;
	int32_t i = 0;
	struct ntb_device *device_proc = NULL;
	ntb_initialize_number_devices();

	NTB_DEBUG_PRINT(("NTB: Entering Init\n"));

	for (i = 0;  i < MAX_DEVICES; i++) {

		device_proc = ntb_get_device(i);
		if (device_proc == NULL) {
			NTB_DEBUG_PRINT(
			("NTB: Unable to retrieve NTB devices \n"));
			return -EPERM;
		}

		memset(device_proc, 0x00, sizeof(struct ntb_device));
		g_tasklet_data[i] = 0;

	}

	if (pci_register_driver(&ntb_pci_ops) < 0) {
		NTB_DEBUG_PRINT(("NTB: Error on PCI Registration \n"));
		return -EPERM;
	}


	return ret;

}

/*****************************************************************************
 * Description
 * Called during unloading, releases allocations and registrations.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 ****************************************************************************/
static void ntb_exit(void)
{

	struct ntb_device *device_proc0 = NULL;
	struct ntb_device *device_proc1 = NULL;

	device_proc0 = ntb_get_device(INDEX_0);
	device_proc1 = ntb_get_device(INDEX_1);

	NTB_DEBUG_PRINT(("NTB: Entering ntb_exit \n"));

	pci_unregister_driver(&ntb_pci_ops);

}

/*****************************************************************************
 * Description
 * Initializes an ntb_device.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 ****************************************************************************/
static void ntb_device_init(struct ntb_device *device,
uint16_t bdf, uint32_t device_index)
{
	int32_t i = 0;
	device->client_list.number_used     = NTB_UNUSED;
	device->client_list.semaphore_owner = NTB_UNUSED;

	device->device_state = ENUMERATED;
	device->bdf = bdf;

	/* Assign offset values */
	device->doorbell_offset         = NTB_B2B_DOORBELL_OFFSET;
	device->bar_23_translate_offset = NTB_SBAR_23_TRANSLATE_OFFSET;
	device->bar_45_translate_offset = NTB_SBAR_45_TRANSLATE_OFFSET;
	device->bar_23_limit_offset     = NTB_PBAR_23_LIMIT_OFFSET;
	device->bar_45_limit_offset     = NTB_PBAR_45_LIMIT_OFFSET;
	device->scratchpad_offset_write = NTB_B2B_SCRATCHPAD_OFFSET;
	device->scratchpad_offset_read  = NTB_SCRATCHPAD_OFFSET;
	device->link_control_offset     = NTB_CNTL_OFFSET;
	device->semaphore_offset        = NTB_SCRATCHPAD_SEM4_OFFSET;

	device->link_status_offset      = NTB_LINK_STATUS_OFFSET;

	for (i = 0; i < NO_CLIENTS; i++) {
		NTB_DEBUG_PRINT(
		("NTB: Client List %p\n", &device->client_list));
		device->client_list.clients[i].handle = NTB_UNUSED;
		device->client_list.clients[i].callback = NULL;
		device->client_list.clients[i].bdf = bdf;
		device->device_tag = NTB_IDENTIFIER + device_index;

	}
	/* default values */
	device->policy_bits_23 = 0;
	device->policy_bits_45 = 0;
	spin_lock_init(&device->client_list.client_list_lock);

}

/*****************************************************************************
 * Description
 * Called during loading when correct device ID is found.
 *
 * Side Effects:SOFTWARE_ONLY_ON
 * Assumptions:
 * Return Values: 0 successful, -ENODEV, -EPERM unsuccessful
 ****************************************************************************/
static int32_t ntb_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	/* Link enabled, sec side r/wr secondary regs, snoop set to default */
	uint32_t cntl_value = 0;
	uint32_t attempt = 0;
	uint16_t doorbell = 0;
	struct ntb_device *device = NULL;
	int32_t number_devices = ntb_get_number_devices();
	uint16_t bdf = 0;
	/*Test
	uint32_t memory_lcfgbus = 0xc804011c;
	uint32_t *virtual_lcfgbus = NULL; */

	NTB_DEBUG_PRINT(("NTB: PROBE FOUND DEVICE ID BUS %x DEVFN %x\n",
	dev->bus->number, dev->devfn));

	bdf = ntb_get_bdf(dev->bus->number, dev->devfn);

	NTB_DEBUG_PRINT(("NTB: PROBE FOUND DEVICE ID BDF %x\n",
	bdf));

	device = ntb_get_device(number_devices);
	device->device_id = NTB_B2B_DEVICE_ID;
	spin_lock_init(&lock_callback_tasklet);
	spin_lock_init(&lock_pm_event_check);

	if (device == NULL)
		return -ENODEV;

	device->device_index = number_devices;
	device->dev = dev;
	ntb_device_init(device, bdf, number_devices);

	NTB_DEBUG_PRINT(("NTB: SUCCESSFUL NTB LOAD\n"));
	if (pci_enable_device(dev))
		return -ENODEV;

	/* The bus master enable bit in PCI config space is set originally
	by BIOS but once this driver loads/unloads its not being set again
	this makes ord2 0xc0018004p on the ITP read 0406 rather than 0402 */
	pci_set_master(dev);

	if (ntb_get_bar_addresses(device) != 0) {
		NTB_DEBUG_PRINT(
		("NTB: BAR ACCESS FAILURE IN PROBE PROC %x\n",
		device->device_state));
		return -EPERM;
	}

	/* Valid only in back to back configuration */
	ntb_lib_write_16(device->mm_regs, BUS_MASTER_MEMORY_OFFSET,
	BUS_MASTER_MEMORY_ENABLE);


	doorbell = ntb_lib_read_16(device->mm_regs,
	NTB_PDOORBELL_OFFSET);

	NTB_DEBUG_PRINT(("NTB: Doorbell %i \n", doorbell));

	ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET,
	doorbell);

	NTB_DEBUG_PRINT(("NTB: Clear Doorbell \n"));

	attempt = ntb_set_interrupts(device);
	if (attempt != 0) {
		NTB_DEBUG_PRINT(
		("NTB: INTERRUPT ALLOCATION FAILURE IN PROBE\n"));
		return -EPERM;
	}

	/*ntb_lib_write_16(device->mm_regs, DOORBELL_PRIMARY_MASK_OFFSET,
	DOORBELL_MASK_VALUE); */

	ntb_lib_write_16(device->mm_regs, DOORBELL_PRIMARY_MASK_OFFSET,
	0);

	ntb_lib_write_32(device->mm_regs, device->link_control_offset,
	cntl_value);

	ntb_increment_number_devices();

	NTB_DEBUG_PRINT(("NTB: DEV PTR %p\n", dev));
	if (pci_enable_device(dev)) {
		NTB_DEBUG_PRINT(
		("NTB: UNABLE TO ENABLE DEVICE!\n"));
		return -ENODEV;
	}

	ntb_get_limit_settings(dev, NTB_BAR_23, device, PRIMARY_CONFIG);
	ntb_get_limit_settings(dev, NTB_BAR_45, device, PRIMARY_CONFIG);
	ntb_write_limit_settings_to_scratchpad(device);

	NTB_DEBUG_PRINT(("NTB: LIMIT BASE %Lx\n", device->limit_base_23));
	NTB_DEBUG_PRINT(("NTB: LIMIT MAX %Lx\n", device->limit_max_23));
	NTB_DEBUG_PRINT(("NTB: DEVICE PTR %p\n", device));
	NTB_DEBUG_PRINT(("NTB: DEV PTR %p\n", dev));
	NTB_DEBUG_PRINT(("NTB: DEV PROC ID %x\n", device->device_id));
	NTB_DEBUG_PRINT(("NTB: SUCCESSFUL NTB LOAD\n"));


	pci_set_drvdata(dev, device);

	return 0;

}

/*****************************************************************************
 * Description
 * Called during unloading.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 ****************************************************************************/
static void ntb_remove(struct pci_dev *dev)
{

	struct ntb_device *device = NULL;

	device = (struct ntb_device *)pci_get_drvdata(dev);

	NTB_DEBUG_PRINT(("NTB: REMOVE BDFs x"));
	if (device != NULL) {
		NTB_DEBUG_PRINT(("NTB: DEVICE %p\n", device));
			NTB_DEBUG_PRINT(("NTB: STATE %x\n",
			device->device_state));
			NTB_DEBUG_PRINT(("NTB: INT %x\n",
			device->intx_entry_no));
		NTB_DEBUG_PRINT(("NTB: DEV %p\n", device->dev));

		ntb_release_interrupts(device);
		ntb_release_bar_addresses(device);
	} else {
		NTB_DEBUG_PRINT(
		("NTB: DEVICE == NULL IN NTB_REMOVE\n"));
	}

	pci_disable_device(dev);
}
/*****************************************************************************
 * Description
 * Interrupt handler
 *
 * Side Effects:
 * Assumptions:
 * Return Values: IRQ_HANDLED successful, IRQ_NONE unsuccessful
 ****************************************************************************/
#ifdef RH_5
static irqreturn_t ntb_irq_xxx(int irq, void *data, struct pt_regs *regs)
#else
static irqreturn_t ntb_irq_xxx(int irq, void *data)
#endif
{
	uint32_t tag = 0;
	struct ntb_device *device = NULL;
	int16_t doorbell = 0;
	irqreturn_t handled = IRQ_NONE;


	/* The ntb_device signature
	 * must be at the top of the structure. */
	NTB_DEBUG_PRINT(("NTB: IRQ \n"));

	/* DEBUG
	icounter++;
	outb(icounter, 0x80);
	if(icounter == 9)
		icounter = 0;
	*/

	if (data != NULL) {
		outb(0x0A, 0x80);
		tag = *((int32_t *)data);
		NTB_DEBUG_PRINT(("NTB: ID %x \n", tag));
		if (tag >= NTB_IDENTIFIER &&
		tag <= NTB_IDENTIFIER + MAX_DEVICES)
			device = data;
		else
			return IRQ_NONE;

		/* Get the doorbell register */
		doorbell = ntb_lib_read_16(device->mm_regs,
		NTB_PDOORBELL_OFFSET);
		NTB_DEBUG_PRINT(("NTB:In Interrupts Doorbell %x \n",
		doorbell));
		NTB_DEBUG_PRINT(("NTB: device->device_tag %x \n",
		device->device_tag));

		outb(0x0B, 0x80);
		/* Clear the doorbell register*/
		ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET,
		doorbell);

		g_tasklet_data[device->device_index] |= doorbell;

		handled = IRQ_HANDLED;

		tasklet_schedule(&callback_tasklet);
		outb(0x0C, 0x80);
	}

	outb(0x0D, 0x80);
	return handled;
}

/*****************************************************************************
 * Description
 * Bottom half for processing callbacks to clients.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 ****************************************************************************/
static void callback_tasklet_func(unsigned long data)
{
	uint32_t offset = 0;
	unsigned long flags;
	struct scratchpad_registers pad;
	int32_t i = 0;
	int32_t k = 0;
	struct ntb_clients *client_list;
	int16_t doorbell_array[MAX_DEVICES] = { 0, 0 };
	struct ntb_device *device = NULL;
	ntb_callback_t callback23 = NULL;
	ntb_callback_t callback45 = NULL;
	uint16_t *n_data = (uint16_t *)data;
	uint16_t callback_event_23 = 0;
	uint16_t callback_event_45 = 0;

	NTB_DEBUG_PRINT(("NTB: callback_tasklet_func \n"));

	/* SPINLOCK NOTE: Sharing this data with the interrupt handler above.
	 * To prevent a HW interrupt from happening while it is being copied,
	 * use this style of spinlock.
	 */
	spin_lock_irqsave(&lock_callback_tasklet, flags);
	doorbell_array[INDEX_0] = n_data[INDEX_0];
	doorbell_array[INDEX_1] = n_data[INDEX_1];
	n_data[INDEX_0] = 0;
	n_data[INDEX_1] = 0;
	spin_unlock_irqrestore(&lock_callback_tasklet, flags);

	outb(0x0E, 0x80);


	for (i = INDEX_0; i < MAX_DEVICES; i++) {

		if (doorbell_array[i] == 0)
			continue;

		device = ntb_get_device(i);
		if (device == NULL)
			return;

		if (device->device_state == NOT_ENUMERATED)
			continue;

		client_list = &device->client_list;
		if (client_list == NULL)
			return;

		callback23 = client_list->clients[NTB_CLIENT_23].callback;
		callback45 = client_list->clients[NTB_CLIENT_45].callback;
		if (callback23 == NULL)
			NTB_DEBUG_PRINT(("NTB: Callback 23 == NULL \n"));
		if (callback45 == NULL)
			NTB_DEBUG_PRINT(("NTB: Callback 45 == NULL \n"));


		offset = device->scratchpad_offset_read;
		ntb_lib_read_rep(device->mm_regs, offset, &pad.registers,
				NTB_TOTAL_SCRATCHPAD_NO);


		NTB_DEBUG_PRINT(("NTB: Client List %p \n", client_list));
		NTB_DEBUG_PRINT(("NTB: Before doorbell check \n"));
		NTB_DEBUG_PRINT(("NTB: DOORBELL %x\n", doorbell_array[i]));
		for (k = 0; k < NTB_TOTAL_SCRATCHPAD_NO; k++)
			NTB_DEBUG_PRINT(("NTB: SPAD REG %x %x\n",
			k, pad.registers[k]));


		if ((doorbell_array[i] & device->policy_bits_23))
			callback_event_23 = CALLBACK_EVENT;


		if ((doorbell_array[i] & device->policy_bits_45))
			callback_event_45 = CALLBACK_EVENT;



		NTB_DEBUG_PRINT(("NTB: Before link check \n"));
		if (doorbell_array[i] & NTB_LINK_STATUS_CHANGE) {
			if (device->link_status == LINK_UP)
				device->link_status = LINK_DOWN;
			else
				device->link_status = LINK_UP;

			callback_event_23 = CALLBACK_EVENT;
			callback_event_45 = CALLBACK_EVENT;

		}

		if (doorbell_array[i] & NTB_WCCNTRL_BIT) {
			callback_event_23 = CALLBACK_EVENT;
			callback_event_45 = CALLBACK_EVENT;
		}
		spin_lock(&device->client_list.client_list_lock);
		if (callback23 != NULL && callback_event_23 == CALLBACK_EVENT)
			callback23(
			client_list->clients[NTB_CLIENT_23].handle,
			doorbell_array[i], pad);

		if (callback45 != NULL && callback_event_45 == CALLBACK_EVENT)
			callback45(
			client_list->clients[NTB_CLIENT_45].handle,
			doorbell_array[i], pad);
		spin_unlock(&device->client_list.client_list_lock);

	}

	NTB_DEBUG_PRINT(("NTB: END TASKLET \n"));
}

/*****************************************************************************
 * Description
 * Requests IRQs (MSIx, MSI, legacy INTx)
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/
static int32_t ntb_set_interrupts(struct ntb_device *device)
{
	int32_t enable_attempt = 0;
	uint32_t i = 0;
	uint32_t k = 0;
	uint32_t attempts = 0;
	uint16_t msix_entries = 0;
	uint32_t msi_value = 0;

	pci_read_config_word(device->dev, NTB_MSIXMSGCTRL_OFFSET,
			&device->msixmsgctrl);
	msix_entries = device->msixmsgctrl & NTB_MSIXMSGCTRL_ENTRIES_MASK;

	msix_entries++;
	if (msix_entries > NTB_MSIX_MAX_VECTORS)
		return EPERM;

	NTB_DEBUG_PRINT(("NTB: Entering ntb_setup_interrupts \n"));
	NTB_DEBUG_PRINT(("NTB: MSIX Vector Count %x \n", msix_entries));

	for (i = 0; i < msix_entries; i++)
		device->msix_entries[i].entry = i;

	if (pci_enable_msix(device->dev,
			device->msix_entries,
			ARRAY_SIZE(device->msix_entries)) == 0) {

		if (msix_entries == 0) {
			pci_disable_msix(device->dev);
			return -EPERM;
		}

		for (i = 0; i < msix_entries; i++) {
			enable_attempt = request_irq(
			device->msix_entries[i].vector,
			&ntb_irq_xxx,
			IRQF_SHARED,
			NTB_MSIX_NAME, device);
			if (enable_attempt != 0) {
				NTB_DEBUG_PRINT(
				("NTB: MSIX setup failed\n"));
				pci_disable_msix(device->dev);
				attempts = i;
				for (k = 0; k < attempts; k++)
					free_irq(
					device->msix_entries[i].vector,
					device->dev);
				return -EPERM;
			}
			NTB_DEBUG_PRINT(
				("NTB: MSIX INT %i setup successful\n", i));
			device->msix_entry_no = msix_entries;
		}

	} else if (pci_enable_msi(device->dev) == 0) {
		enable_attempt = 0;
		pci_read_config_dword(device->dev, NTB_MSI_OFFSET,
		&msi_value);

		NTB_DEBUG_PRINT(("NTB: pci_enable_msi passed\n"));
		enable_attempt = request_irq(device->dev->irq,
		&ntb_irq_xxx,
		IRQF_SHARED,
		NTB_MSIX_NAME, device);

		if (enable_attempt != 0) {
			NTB_DEBUG_PRINT(("NTB: MSI setup failed\n"));
			pci_disable_msi(device->dev);
			return -EPERM;
		}
		device->msi_entry_no = 1;
		NTB_DEBUG_PRINT(("NTB: MSI setup successful\n"));
	} else {
		enable_attempt = request_irq(device->dev->irq,
		&ntb_irq_xxx,
		IRQF_SHARED,
		NTB_MSIX_NAME, device);

		if (enable_attempt != 0) {
			NTB_DEBUG_PRINT(("NTB: INTX setup failed\n"));
			device->msix_entry_no = 0;
			device->intx_entry_no = 0;
			device->msi_entry_no = 0;
			return -EPERM;
		} else {
			device->intx_entry_no = 1;
			NTB_DEBUG_PRINT(("NTB: INTX setup successful %x\n",
				device->intx_entry_no));
		}
	}

	return 0;

}

/*****************************************************************************
 * Description
 * Releases IRQs (MSIx, MSI, legacy INTx)
 *
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 ****************************************************************************/
static void ntb_release_interrupts(struct ntb_device *device)
{
	int32_t i = 0;
	NTB_DEBUG_PRINT(("NTB: Int entries msix %x msi %x intx %x \n",
	device->msix_entry_no, device->msi_entry_no, device->intx_entry_no));

	if (device->msix_entry_no != 0) {
		NTB_DEBUG_PRINT(("NTB: Release MSIX Entries \n"));
		for (i = 0; i < device->msix_entry_no; i++)
			free_irq(device->msix_entries[i].vector, device);
		pci_disable_msix(device->dev);
	} else if (device->msi_entry_no != 0) {
		NTB_DEBUG_PRINT(("NTB: Release MSI Entries \n"));
		free_irq(device->dev->irq, device);
		pci_disable_msi(device->dev);
	} else if (device->intx_entry_no != 0) {
		NTB_DEBUG_PRINT(("NTB: Release Legacy Entries \n"));
		free_irq(device->dev->irq, device);
	}

}

/*****************************************************************************
 * Description
 * Power mgmt function suspend.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/

static int32_t ntb_suspend(struct pci_dev *dev, pm_message_t state)
{
	struct scratchpad_registers pad;
	struct ntb_device *device = pci_get_drvdata(dev);
	struct ntb_clients *client_list;
	int32_t timeout = 0;
	int16_t doorbell = 0;

	client_list = ntb_get_client_list(device->device_index);
	if (client_list == NULL)
		return -EPERM;

	doorbell = ntb_lib_read_16(device->mm_regs, device->doorbell_offset);
	spin_lock(&lock_pm_event_check);
	if (client_list->clients[NTB_CLIENT_23].handle != NTB_UNUSED) {
		if (client_list->clients[NTB_CLIENT_23].callback != NULL) {
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement | PM_ACK_23;
			client_list->clients[NTB_CLIENT_23].callback(
			client_list->clients[NTB_CLIENT_23].handle,
			doorbell, pad);
		}
	}

	if (client_list->clients[NTB_CLIENT_45].handle != NTB_UNUSED) {
		if (client_list->clients[NTB_CLIENT_45].callback != NULL) {
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement | PM_ACK_45;
			client_list->clients[NTB_CLIENT_45].callback(
			client_list->clients[NTB_CLIENT_45].handle,
			doorbell, pad);
		}
	}

	spin_unlock(&lock_pm_event_check);

	/* Store important reg values */
	device->bar_23_translate = ntb_lib_read_64(device->mm_regs,
	device->bar_23_translate_offset);
	device->bar_45_translate = ntb_lib_read_64(device->mm_regs,
	device->bar_45_translate_offset);
	device->bar_23_limit     = ntb_lib_read_64(device->mm_regs,
	device->bar_23_limit_offset);
	device->bar_45_limit     = ntb_lib_read_64(device->mm_regs,
	device->bar_45_limit_offset);
	device->cntrl            = ntb_lib_read_32(device->mm_regs,
	device->link_control_offset);

	while (device->client_pm_acknowledgement != ACKNOWLEDGED
	&& timeout != TIMEOUT) {
		/* wait for a while */
		udelay(NTB_DELAY);
		/* time out after TIMEOUT number of tries */
		timeout++;
	}

	ntb_release_interrupts(device);
	pci_save_state(dev);
	pci_disable_device(dev);



	return SUCCESS;
}
/*****************************************************************************
 * Description
 * Power mgmt function resume.
 *
 * Side Effects:
 * Assumptions:
 * Return Values: 0 successful, -EPERM unsuccessful
 ****************************************************************************/
static int32_t ntb_resume(struct pci_dev *dev)
{
	struct scratchpad_registers pad;
	struct ntb_device *device = NULL;
	struct ntb_clients *client_list = NULL;
	int16_t doorbell = 0;

	device = pci_get_drvdata(dev);
	if (device == NULL)
		return -EPERM;

	client_list = ntb_get_client_list(device->device_index);
	if (client_list == NULL)
		return -EPERM;

	doorbell = ntb_lib_read_16(device->mm_regs, device->doorbell_offset);
	/* Restore important reg values */
	ntb_lib_write_64(device->mm_regs,
	device->bar_23_translate_offset, device->bar_23_translate);
	ntb_lib_write_64(device->mm_regs,
	device->bar_45_translate_offset, device->bar_45_translate);
	ntb_lib_write_64(device->mm_regs,
	device->bar_23_limit_offset, device->bar_23_limit);
	ntb_lib_write_64(device->mm_regs,
	device->bar_45_limit_offset, device->bar_45_limit);
	ntb_lib_write_32(device->mm_regs,
	device->link_control_offset, device->cntrl);


	spin_lock(&lock_pm_event_check);
	if (client_list->clients[NTB_CLIENT_23].handle != NTB_UNUSED) {
		if (client_list->clients[NTB_CLIENT_23].callback != NULL) {
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement | PM_ACK_23;
			client_list->clients[NTB_CLIENT_23].callback(
			client_list->clients[NTB_CLIENT_45].handle,
			doorbell,
			pad);
		}
	}

	if (client_list->clients[NTB_CLIENT_45].handle != NTB_UNUSED) {
		if (client_list->clients[NTB_CLIENT_45].callback != NULL) {
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement | PM_ACK_45;
			client_list->clients[NTB_CLIENT_45].callback(
			client_list->clients[NTB_CLIENT_45].handle,
			doorbell,
			pad);
		}
	}

	spin_unlock(&lock_pm_event_check);

	if (pci_enable_device(dev))
		pci_restore_state(dev);
	else
		return -EPERM;

	if ((ntb_set_interrupts(device)) != 0)
		return -EPERM;

	return 0;
}

/*****************************************************************************
 * Description
 * Retrieves BARs with pci_resource_start API.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: 0 successful, -ENODEV unsuccessful
 ****************************************************************************/
static int32_t ntb_get_bar_addresses(struct ntb_device *device)
{

	int32_t i = 0;
	int32_t bars[BAR_NO] = { PCI_CONFIG_SPACE_01,
	PCI_CONFIG_SPACE_23,
	PCI_CONFIG_SPACE_45 };

	void *virtual_address;
	int req = 0;

	for (i = 0; i < BAR_NO; i++) {
		virtual_address = NULL;

		req = pci_request_region(device->dev, bars[i], g_ntb_name);
		if (req != 0) {
			NTB_DEBUG_PRINT(("NTB: FAILED TO OBTAIN %i\n", i));
			return -ENODEV;
		}

		device->pci_bar[i] = pci_resource_start(device->dev,
			bars[i]);

		NTB_DEBUG_PRINT(("NTB: BAR %Lx\n", device->pci_bar[i]));

		/*See README.txt for details about this ioremap call */
		virtual_address = ioremap(device->pci_bar[i],
			pci_resource_len(device->dev, bars[i]));

		if (virtual_address == NULL) {
			NTB_DEBUG_PRINT(("NTB: IOREMAP FAILED\n"));
			return -EPERM;

		} else {
			NTB_DEBUG_PRINT(("NTB: VIRTUAL ADDRESS %p\n",
				virtual_address));
		}

		if (i == DEVICE_BAR_01)
			device->mm_regs =
			(struct ntb_mm_regs *)virtual_address;
		else if (i == DEVICE_BAR_23)
			device->pci_bar_23_virt = virtual_address;
		else if (i == DEVICE_BAR_45)
			device->pci_bar_45_virt = virtual_address;

	}

	return 0;
}
/*****************************************************************************
 * Description
 * Releases BARs with pci_release_region.
 *
 * Side Effects:
 * Assumptions: PRIVATE
 * Return Values: NONE
 ****************************************************************************/
static void ntb_release_bar_addresses(struct ntb_device *device)
{
	int32_t i = 0;
	int32_t bars[BAR_NO] = { PCI_CONFIG_SPACE_01,
			PCI_CONFIG_SPACE_23,
			PCI_CONFIG_SPACE_45 };

	NTB_DEBUG_PRINT(("NTB: RELEASE BAR ADDRESSES\n"));

	if (device->mm_regs != NULL)
		iounmap(device->mm_regs);

	if (device->pci_bar_23_virt != NULL)
		iounmap(device->pci_bar_23_virt);

	if (device->pci_bar_45_virt != NULL)
		iounmap(device->pci_bar_45_virt);

	NTB_DEBUG_PRINT(("NTB: SUCCESSFUL IOUNMAP\n"));
	for (i = 0; i < BAR_NO; i++) {
		/* If configured for 32 bit, there are two BAR vals */
		if (device->pci_bar[i] != 0) {
			NTB_DEBUG_PRINT(("NTB: RELEASE BAR %x\n", bars[i]));
			pci_release_region(device->dev, bars[i]);

		}
	}

}

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
int32_t ntb_get_api(struct ntb_api_export *funcs)
{
	int32_t err = SUCCESS;
	if (funcs != NULL) {
		*funcs = ntb_api;
	} else {
		NTB_DEBUG_PRINT(
		("NTB: FAILED INITIALIZATION OF NTB FUNCTION TABLE\n"));
		err = FAILED;
	}
	return err;
}
EXPORT_SYMBOL(ntb_get_api);

/*****************************************************************************
 * See ntb_main.h
 ****************************************************************************/
int32_t ntb_client_suspend(ntb_client_handle_t handle)
{
	struct ntb_device *device = ntb_get_device_by_handle(handle);

	if (device == NULL)
		return -EINVAL;

	spin_lock(&lock_pm_event_check);
	if (handle & NTB_BAR_23)
		device->client_pm_acknowledgement =
		device->client_pm_acknowledgement & PM_ACK_45;
	if (handle & NTB_BAR_45)
		device->client_pm_acknowledgement =
		device->client_pm_acknowledgement & PM_ACK_23;
	spin_unlock(&lock_pm_event_check);
	return SUCCESS;


}

module_init(ntb_init);
module_exit(ntb_exit);

