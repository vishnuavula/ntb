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


/*****************************************************************************
 * FILE CONTENTS: Data and types private to the driver. These APIs will not
 * be exposed to a client driver or user space application.
 ****************************************************************************/
#ifndef NTB_LIB_H_
#define NTB_LIB_H_

#include <linux/autoconf.h>
#include <linux/module.h>     /* SET_MODULE_OWNER */
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/kernel.h>     /* DMA_DEBUG_PRINTK_0() */
#include <linux/string.h>
#include <linux/init.h>
#include <linux/fs.h>         /* everything... */
#include <linux/errno.h>      /* error codes */
#include <linux/types.h>      /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>      /* O_ACCMODE */
#include <asm/system.h>       /* cli, *_flags */
#include <linux/uaccess.h>    /* copy_to_user */
#include <linux/io.h>         /* inb(), outb() */
#include <linux/kmod.h>
#include <linux/ioport.h>     /* request_region */
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/interrupt.h>  /* tasklets */
#include <linux/list.h>
#include <linux/mman.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/cdev.h>      /* For registering the file operations struct */
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/crypto.h>
#include <linux/spinlock_types.h>


#define MAX_BARS_USED      2
#define NTB_SEMAPHORE_USED 1
#define NTB_SEMAPHORE_FREE 0
#define FAILED             -1
#define SUCCESS            0

/* Snoop Mask that are private to the NTB Driver not exposed via the Client
Driver interface */
#define NTB_23_SNOOP_MASK	0x0000003C
#define NTB_45_SNOOP_MASK	0x000003C0

/*SERIOUS: This must be changed after a silicon REV because
there is a silicon bug that requires us to write all F's here for now.
#define SEMAPHORE_ONE_TO_CLEAR  0x01 */
#define SEMAPHORE_ONE_TO_CLEAR  0xffff

#define LOWER_32     0x00000000FFFFFFFF
#define BIT_SHIFT_32 0x20
#define OFFSET_4     0x4

/* NTB_CNTL SETTINGS */
#define NTB_SECONDARY_SPACE_LOCKED                0x00000001
#define NTB_SECONDARY_SPACE_UNLOCKED              0x00000000

#define NTB_LINK_ENABLED                          0x00000000
#define NTB_LINK_DISABLED                         0x00000002

#define NTB_MSIXMSGCTRL_OFFSET                    0x82
#define NTB_MSIXMSGCTRL_ENTRIES_MASK              0x7FF
#define NTB_MSIXMSGCTRL_ENABLED_MASK              0x8000
#define NTB_MSI_IRQ_MASK                          0x000000FF
#define NTB_MSI_OFFSET                            0x68

#define NTB_PMSIX_TABLE_DEFAULT_OFFSET            0x2000
#define NTB_SMSIX_TABLE_DEFAULT_OFFSET            0x4000

#define NTB_LIMIT_MAX_23                          0xD0
#define NTB_LIMIT_MAX_45                          0xD1

#define BUS_MASTER_MEMORY_OFFSET 0x504
#define BUS_MASTER_MEMORY_ENABLE 0x06
#define DOORBELL_MASK_OFFSET     0x62
#define DOORBELL_MASK_VALUE      0x8000

/* PCI Configuration Registers */

#define NTB_LINK_CONTROL_OFFSET        0x1A0
#define NTB_LINK_STATUS_OFFSET         0x1A2

#ifndef NTB_SCRATCHPAD_REGS_DEFINED
#define NTB_SCRATCHPAD_REGS_DEFINED

#define NTB_TOTAL_SCRATCHPAD_NO 16

/* struct to hold scratchpad registers */
struct scratchpad_registers {
	uint32_t registers[NTB_TOTAL_SCRATCHPAD_NO];
};

#endif /* NTB_SCRATCHPAD_REGS_DEFINED */

/* struct to hold B2B shadowed registers */
struct shadowed_area {
	struct scratchpad_registers b2b_scratchpad;
	uint32_t b2b_doorbell;
	uint64_t b2b_translate;
};

/* memory mapped registers  */

struct ntb_mm_regs {
	uint64_t ntb_primary_bar_23_limit;
	uint64_t ntb_primary_bar_45_limit;
	uint64_t ntb_primary_bar_23_translate;
	uint64_t ntb_primary_bar_45_translate;
	uint64_t ntb_secondary_bar_23_limit;
	uint64_t ntb_secondary_bar_45_limit;
	uint64_t ntb_secondary_bar_23_translate;
	uint64_t ntb_secondary_bar_45_translate;
	uint64_t ntb_secondary_base_0;
	uint64_t ntb_secondary_base_2;
	uint64_t ntb_secondary_base_4;

	uint32_t ntb_cntl;
	int16_t ntb_sbdf;
	int16_t ntb_reserved_sbdf;
	uint16_t ntb_pdoorbell;
	uint16_t ntb_pdbmask;
	uint16_t ntb_sdoorbell;
	uint16_t ntb_sdbmask;
	uint16_t reserved_region_one;
	uint16_t ntb_usememmiss;
	uint32_t reserved_region_two[4];
	struct scratchpad_registers scratchpad;
	uint32_t ntb_scratchpad_semaphore;

	/* Shadowed area separated by large reserve region */
	uint32_t reserved_region_three[15];

	struct shadowed_area shadow;
};

/* register offsets */
enum ntb_mmio_offsets {
	NTB_PBAR_23_LIMIT_OFFSET     = 0x00,
	NTB_PBAR_45_LIMIT_OFFSET     = 0x08,
	NTB_PBAR_23_TRANSLATE_OFFSET = 0x10,
	NTB_PBAR_45_TRANSLATE_OFFSET = 0x18,

	NTB_SBAR_23_LIMIT_OFFSET     = 0x20,
	NTB_SBAR_45_LIMIT_OFFSET     = 0x28,
	NTB_SBAR_23_TRANSLATE_OFFSET = 0x30,
	NTB_SBAR_45_TRANSLATE_OFFSET = 0x38,

	NTB_SECONDARY_BASE_0_OFFSET  = 0x40,
	NTB_SECONDARY_BASE_2_OFFSET  = 0x48,
	NTB_SECONDARY_BASE_4_OFFSET  = 0x50,

	NTB_CNTL_OFFSET              = 0x58,
	NTB_SBDF_OFFSET              = 0x5C,

	NTB_PDOORBELL_OFFSET         = 0x60,
	NTB_SDOORBELL_OFFSET         = 0x64,

	NTB_SCRATCHPAD_OFFSET        = 0x80,
	NTB_SCRATCHPAD_SEM4_OFFSET   = 0xC0,

	NTB_B2B_SCRATCHPAD_OFFSET    = 0x100,
	NTB_B2B_DOORBELL_OFFSET      = 0x140,
	NTB_B2B_TRANSLATE_OFFSET     = 0x144
};

/* default register values */
enum ntb_doorbell_default_values {
	NTB_HEARTBEAT            = 0x01,
	NTB_EVENT_NOTIFICATION   = 0x400,
	NTB_EVENT_ACKNOWLEDGMENT = 0x800,
	NTB_LINK_STATUS_CHANGE   = 0x8000,
};

/* Writes */
void ntb_lib_write_16(void *mm_regs, uint32_t offset, int16_t value);

void ntb_lib_write_32(void *mm_regs, uint32_t offset, uint32_t value);

void ntb_lib_write_64(void *mm_regs, uint64_t offset, uint64_t value);

void ntb_lib_write_rep(void *mm_regs, uint32_t offset, void *pad,
uint32_t count);

/* Reads */
uint16_t ntb_lib_read_16(void *mm_regs, uint32_t offset);

uint32_t ntb_lib_read_32(void *mm_regs, uint32_t offset);

uint64_t ntb_lib_read_64(void *mm_regs, uint64_t offset);

void ntb_lib_read_rep(void *mm_regs, uint32_t offset, void *pad,
uint32_t count);


#endif /*NTB_LIB_H_*/
