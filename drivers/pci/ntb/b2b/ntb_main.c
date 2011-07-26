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
 */

#include "../common/ntb_main.h"

MODULE_LICENSE("Intel Proprietary");

/* PRIVATE */
static int ntb_init(void);
static void ntb_exit(void);
static void ntb_device_init(struct ntb_device *device,
enum ntb_handle_prefix_t prefix);
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
	.ntb_write_translate_address_value = ntb_write_translate_address_value,
	.ntb_write_doorbell                = ntb_write_doorbell,
	.ntb_obtain_semaphore              = ntb_obtain_semaphore,
	.ntb_release_semaphore             = ntb_release_semaphore,
	.ntb_set_snoop_level               = ntb_set_snoop_level,
	.ntb_get_number_devices            = ntb_get_number_devices,
	.ntb_set_policy                    = ntb_set_policy,
	.ntb_set_heartbeat                 = ntb_set_heartbeat,
	.ntb_release_heartbeat             = ntb_release_heartbeat,
	.ntb_get_link_status               = ntb_get_link_status,
	.ntb_set_link_status               = ntb_set_link_status,
	.ntb_get_bar_address               = ntb_get_bar_address,
	.ntb_client_suspend                = ntb_client_suspend,
	.ntb_get_device                = ntb_get_device
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
        int i;
	int32_t ret = SUCCESS;
	struct ntb_device *device_proc[MAX_DEVICES];
	ntb_initialize_number_devices();

	NTB_DEBUG_PRINT(("NTB: Entering Init\n"));
      
        for(i= PROC_0; i< MAX_DEVICES; i++) 
        {
           device_proc[i] = NULL;
	   device_proc[i] = ntb_get_device(i);
	   if ((device_proc[i] == NULL)) {
		NTB_DEBUG_PRINT(("NTB: Unable to retrieve NTB devices \n"));
		return -EPERM;
	   }
	   memset(device_proc[i], 0x00, sizeof(struct ntb_device));
	   device_proc[i]->device_processor = NOT_ENUMERATED;
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

        int i;
	struct ntb_device *device_proc[MAX_DEVICES];
          
        for(i=PROC_0; i < MAX_DEVICES; i++)
        {
            device_proc[i] = NULL;
	    device_proc[i] = ntb_get_device(i);
        }

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
enum ntb_handle_prefix_t prefix)
{
	int32_t i = 0;
	device->client_list.number_used     = NTB_UNUSED;
	device->client_list.heartbeat_owner = NTB_UNUSED;
	device->client_list.semaphore_owner = NTB_UNUSED;

	/* Assign offset values */
	device->doorbell_offset         = NTB_B2B_DOORBELL_OFFSET;
	device->bar_23_translate_offset = NTB_SBAR_23_TRANSLATE_OFFSET;
	device->bar_45_translate_offset = NTB_SBAR_45_TRANSLATE_OFFSET;
	device->bar_23_limit_offset     = NTB_PBAR_23_LIMIT_OFFSET;
	device->bar_45_limit_offset     = NTB_PBAR_45_LIMIT_OFFSET;
	device->scratchpad_offset_write = NTB_B2B_SCRATCHPAD_OFFSET;
	device->scratchpad_offset_read  = NTB_SCRATCHPAD_OFFSET;
	device->link_control_offset     = NTB_CNTL_OFFSET;

	device->link_status_offset = NTB_LINK_STATUS_OFFSET;

	for (i = 0; i < NO_CLIENTS; i++) {
		device->client_list.clients[i].handle = NTB_UNUSED;
		device->client_list.clients[i].callback = NULL;
		device->client_list.clients[i].proc_id
		= device->device_processor;
		device->client_list.clients[i].prefix = prefix;
		if (prefix == NTB_CLIENT_P0)
			device->device_id = NTB_ID_PROC0;
		else
			device->device_id = NTB_ID_PROC1;
	}
	/* default values */
	device->heartbeat_bit = NTB_HEARTBEAT;
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
	enum ntb_handle_prefix_t prefix;

	NTB_DEBUG_PRINT(("NTB: PROBE FOUND DEVICE ID\n"));

	if (dev->bus->number == NTB_BUS_NUMBER_P0) {
		device = ntb_get_device(PROC_0);
		spin_lock_init(&lock_callback_tasklet);
		spin_lock_init(&lock_pm_event_check);
		if (device == NULL)
			return -ENODEV;
		device->device_processor = PROC_0;
		prefix = NTB_CLIENT_P0;
	} else {
		device = ntb_get_device(PROC_1);
		if (device == NULL)
			return -ENODEV;
		device->device_processor = PROC_1;
		prefix = NTB_CLIENT_P1;

	}
	device->dev = dev;
	ntb_device_init(device, prefix);

	NTB_DEBUG_PRINT(("g_task_data 0x%Lx\n", (unsigned long long)g_tasklet_data));

	NTB_DEBUG_PRINT(("NTB: SUCCESSFUL DEVICE INIT \n"));
	if (pci_enable_device(dev))
		return -ENODEV;

	/* The bus master enable bit in PCI config space is set originally
	by BIOS but once this driver loads/unloads its not being set again
	this makes ord2 0xc0018004p on the ITP read 0406 rather than 0402 */
	pci_set_master(dev);

	if (ntb_get_bar_addresses(device) != 0) {
		NTB_DEBUG_PRINT(
		("NTB: BAR ACCESS FAILURE IN PROBE PROC %x\n",
		device->device_processor));
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

	ntb_lib_write_16(device->mm_regs, DOORBELL_MASK_OFFSET,
	DOORBELL_MASK_VALUE);

	ntb_lib_write_32(device->mm_regs, device->link_control_offset,
	cntl_value);

	ntb_increment_number_devices();

	NTB_DEBUG_PRINT(("NTB: DEV PTR %p\n", dev));
	if (pci_enable_device(dev)) {
		NTB_DEBUG_PRINT(
		("NTB: UNABLE TO ENABLE DEVICE!\n"));
		return -ENODEV;
	}

	ntb_get_limit_settings(dev, NTB_BAR_23, &(device->limit_base_23),
	&(device->limit_max_23));
	ntb_get_limit_settings(dev, NTB_BAR_45, &(device->limit_base_45),
		&(device->limit_max_45));

	NTB_DEBUG_PRINT(("NTB: Limit Base %Lx\n", device->limit_base_23));
	NTB_DEBUG_PRINT(("NTB: Limit Max %Lx\n", device->limit_max_23));
	NTB_DEBUG_PRINT(("NTB: DEVICE PTR %p\n", device));
	NTB_DEBUG_PRINT(("NTB: DEV PTR %p\n", dev));
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

	NTB_DEBUG_PRINT(("NTB: REMOVE\n"));

	if (dev->bus->number == NTB_BUS_NUMBER_P0)
		device = ntb_get_device(PROC_0);
	else
		device = ntb_get_device(PROC_1);

	if (device != NULL) {
		NTB_DEBUG_PRINT(("NTB: DEVICE %p\n", device));
		NTB_DEBUG_PRINT(("NTB: PROC %x\n", device->device_processor));
		NTB_DEBUG_PRINT(("NTB: INT %x\n", device->intx_entry_no));
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
	int32_t id = 0;
	struct ntb_device *device = NULL;
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
		id = *((int32_t *)data);
		NTB_DEBUG_PRINT(("NTB: ID %x \n", id));
		if (id == NTB_ID_PROC0 || id == NTB_ID_PROC1)
			device = data;
		else
			return IRQ_NONE;
                //mask all the interrupts, they will be unmasked in tasklet at the end
		ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET+2,
		0xffff);
		NTB_DEBUG_PRINT(("masked door bell interrupts %x \n",ntb_lib_read_16(device->mm_regs, NTB_PDOORBELL_OFFSET+2)));

		handled = IRQ_HANDLED;

		tasklet_schedule(&callback_tasklet);
	}

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
	struct scratchpad_registers pad;
	int32_t heartbeat = NTB_UNUSED;
	int16_t doorbell = 0;
	int32_t i = 0;
	struct ntb_clients *client_list;
	int16_t doorbell_array[MAX_DEVICES];
	struct ntb_device *device = NULL;
	ntb_callback_t callback23 = NULL;
	ntb_callback_t callback45 = NULL;

	NTB_DEBUG_PRINT(("NTB: callback_tasklet_func 0x%Lx \n", data));

	/* SPINLOCK NOTE: Sharing this data with the interrupt handler above.
	 * To prevent a HW interrupt from happening while it is being copied,
	 * use this style of spinlock.
	 */
        
	/* Heartbeat notifications need to be sent right away. */
	for (i = PROC_0; i < MAX_DEVICES; i++) {
		device = ntb_get_device(i);
		if (device == NULL)
			return;

		/* Get the doorbell register */
		doorbell = ntb_lib_read_16(device->mm_regs,
		NTB_PDOORBELL_OFFSET);
		NTB_DEBUG_PRINT(("NTB:In Interrupts Doorbell %x \n",
		doorbell));
		NTB_DEBUG_PRINT(("NTB:Pending In Interrupts Doorbell %x \n",doorbell));

		/* Clear the doorbell register*/
		ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET,
		doorbell);
	        doorbell_array[i] = doorbell;

		doorbell = ntb_lib_read_16(device->mm_regs,
		NTB_PDOORBELL_OFFSET);
		NTB_DEBUG_PRINT(("NTB:In Interrupts Doorbell after clearing %x \n",doorbell));
		NTB_DEBUG_PRINT(("NTB:cleared door bell\n")); 

		client_list = &device->client_list;
		if (client_list == NULL)
                {
                      //unmask db interrupts 
		NTB_DEBUG_PRINT(("NTB:client list null \n")); 
		      ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET+2,
		      0x0);
			return;
                }

		callback23 = client_list->clients[NTB_CLIENT_23].callback;
		callback45 = client_list->clients[NTB_CLIENT_45].callback;
		offset = device->scratchpad_offset_read;
		ntb_lib_read_rep(device->mm_regs, offset, &pad,
				NTB_TOTAL_SCRATCHPAD_NO);
		NTB_DEBUG_PRINT(("NTB: Before doorbell check \n"));
		if (doorbell_array[i] & device->heartbeat_bit) {
			/*Call back clients, with heartbeat message */
			heartbeat = client_list->heartbeat_owner;
			if (heartbeat != NTB_HEARTBEAT_UNUSED)
				client_list->clients[heartbeat].callback(
				heartbeat,
				doorbell_array[i], pad);
		}
		NTB_DEBUG_PRINT(("NTB: Before link check \n"));
		if (doorbell_array[i] & NTB_LINK_STATUS_CHANGE) {
			if (device->link_status == LINK_UP)
				device->link_status = LINK_DOWN;
			else
				device->link_status = LINK_UP;
			/* Everyone should be called back */
			if (callback23 != NULL)
                        {
				callback23(
				client_list->clients[NTB_CLIENT_23].handle,
				doorbell_array[i], pad);
                        }
			if (callback45 != NULL)
                        {
				callback45(
				client_list->clients[NTB_CLIENT_45].handle,
				doorbell_array[i], pad);
                         }

		}
                 NTB_DEBUG_PRINT(("NTB door bell array%d 0x%x  bits 0x%x\n", i, doorbell_array[i], device->bits_23));

		if ((doorbell_array[i] & device->bits_23)) {
		            NTB_DEBUG_PRINT(("NTB:checking callback23\n")); 
			if (callback23 != NULL)
                        {
		              NTB_DEBUG_PRINT(("NTB:calling callback23\n")); 
				callback23(
				client_list->clients[NTB_CLIENT_23].handle,
				doorbell_array[i], pad);
                       }

		}

		if ((doorbell_array[i] & device->bits_45)) {
			if (callback45 != NULL)
                        {
				callback45(
				client_list->clients[NTB_CLIENT_45].handle,
				doorbell_array[i], pad);
                         }
		}


		NTB_DEBUG_PRINT(("NTB:unmasking door bells\n")); 
                //unmask db interrupts 
		ntb_lib_write_16(device->mm_regs, NTB_PDOORBELL_OFFSET+2,
		0x0);
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
	uint32_t attempts = 0;
	uint16_t msix_entries = 0;
	uint16_t msix_enabled = 0;
	uint32_t msi_value = 0;

	pci_read_config_word(device->dev, NTB_MSIXMSGCTRL_OFFSET,
			&device->msixmsgctrl);
	msix_entries = device->msixmsgctrl & NTB_MSIXMSGCTRL_ENTRIES_MASK;
	msix_enabled = device->msixmsgctrl & NTB_MSIXMSGCTRL_ENABLED_MASK;

	msix_entries++;
	if (msix_entries > NTB_MSIX_MAX_VECTORS)
		return EPERM;

	NTB_DEBUG_PRINT(("NTB: Entering ntb_setup_interrupts \n"));

	if (msix_enabled > 0) {
		NTB_DEBUG_PRINT(
			("NTB: MSIX Vector Count %x \n", msix_entries));
		for (i = 0; i < msix_entries; i++)
			device->msix_entries[i].entry = i;

		enable_attempt = pci_enable_msix(device->dev,
				device->msix_entries,
				ARRAY_SIZE(device->msix_entries));
		if (enable_attempt == 0) {
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
					attempts = msix_entries;
					for (i = 0; i < attempts; i++)
						free_irq(
						device->msix_entries[i].vector,
						device->dev);
				}
				NTB_DEBUG_PRINT(
					("NTB: MSIX setup successful\n"));
				device->msix_entry_no = msix_entries;
			}

		} else {
			pci_disable_msix(device->dev);
		}
	}
        else if (pci_enable_msi(device->dev) == 0) {
		enable_attempt = 0;
		pci_read_config_dword(device->dev, NTB_MSI_OFFSET,
		&msi_value);
	//	device->dev->irq = msi_value & NTB_MSI_IRQ_MASK;

		enable_attempt = request_irq(device->dev->irq,
		&ntb_irq_xxx,
		IRQF_SHARED,
		NTB_MSIX_NAME, device);

		if (enable_attempt != 0) {
			NTB_DEBUG_PRINT(("NTB: MSI setup failed\n"));
			pci_disable_msi(device->dev);
		}
		device->msi_entry_no = 1;
		NTB_DEBUG_PRINT(("NTB: MSI setup successful\n"));
	}
        else {
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
		}
		device->intx_entry_no = 1;
		NTB_DEBUG_PRINT(("NTB: INTXx setup successful %x\n",
		device->intx_entry_no));
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

	client_list = ntb_get_client_list(device->device_processor);
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

	client_list = ntb_get_client_list(device->device_processor);
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
#ifdef CONFIG_X86_32
	uint64_t resource_start_lower = 0;
	uint64_t resource_start_upper = 0;
#endif
	int32_t i = 0;
	int32_t bars[BAR_NO] = { PCI_CONFIG_SPACE_01,
	PCI_CONFIG_SPACE_23,
	PCI_CONFIG_SPACE_45 };
	void *virtual_address;
	int req = 0;

	for (i = 0; i < BAR_NO; i++) {
		virtual_address = NULL;

#ifdef CONFIG_X86_32
		req = pci_request_region(device->dev, bars[i], g_ntb_name);
		if (req != 0) {
			NTB_DEBUG_PRINT(("NTB: FAILED TO OBTAIN %i\n", i));
			return -ENODEV;
		}

		req =
		pci_request_region(device->dev, bars[i] + 1, g_ntb_name);
		if (req != 0) {
			NTB_DEBUG_PRINT(("NTB: FAILED TO OBTAIN %i\n", i));
			return -ENODEV;
		}

		resource_start_lower = pci_resource_start(device->dev,
				bars[i]);
		NTB_DEBUG_PRINT(("NTB: lower %Lx\n", resource_start_lower));

		resource_start_upper = pci_resource_start(device->dev,
				bars[i] + 1);
		NTB_DEBUG_PRINT(("NTB: UPPER %Lx\n", resource_start_upper));
		/* An address was returned that is above the 36 bit limit */
		if ((resource_start_upper & MASK_36_BIT) != 0)
			return -EPERM;



		device->pci_bar[i] =
		resource_start_upper << BIT_SHIFT_32;
		device->pci_bar[i] =
		resource_start_lower | device->pci_bar[i];
		device->pci_bar[i] = resource_start_lower;
		NTB_DEBUG_PRINT(("NTB: Bar Stored as %Lx\n",
		device->pci_bar[i]));

#else
		req = pci_request_region(device->dev, bars[i], g_ntb_name);
		if (req != 0)
			return -ENODEV;

		device->pci_bar[i] = pci_resource_start(device->dev, bars[i]);

#endif /*CONFIG_X86_32 */

		/*See README.txt for details about this ioremap call */
		virtual_address = ioremap(device->pci_bar[i], pci_resource_len(
				device->dev, bars[i]));

		if (virtual_address == NULL)
			return -EPERM;
		else
			NTB_DEBUG_PRINT(("NTB: Virtual Address as %p\n",
					virtual_address));

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


	if (device->mm_regs != NULL) {
		/*free all pci-related resources */
		iounmap(device->mm_regs);
	}

	NTB_DEBUG_PRINT(("NTB: SUCCESSFUL IOUNMAP\n"));
	for (i = 0; i < BAR_NO; i++) {
		/* If configured for 32 bit, there are two BAR vals */
		if (device->pci_bar[i] != 0) {
#ifdef CONFIG_X86_32
			NTB_DEBUG_PRINT(("NTB: RELEASE BAR %x\n", bars[i]));
			pci_release_region(device->dev, bars[i]);
			pci_release_region(device->dev, bars[i] + 1);
#else
			pci_release_region(device->dev, bars[i]);

#endif
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
	int32_t check = ntb_check_handle_validity(handle);

	if (device == NULL)
		return -EPERM;

	if (VALID == check) {
		spin_lock(&lock_pm_event_check);
		if (handle & NTB_BAR_23)
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement & PM_ACK_45;
		if (handle & NTB_BAR_45)
			device->client_pm_acknowledgement =
			device->client_pm_acknowledgement & PM_ACK_23;
		spin_unlock(&lock_pm_event_check);
		return SUCCESS;
	} else {
		return -EPERM;
	}



}

module_init(ntb_init);
module_exit(ntb_exit);

