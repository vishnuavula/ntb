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



/*****************************************************************************
 * FILE CONTENTS: Functions private to the driver. These APIs will not
 * be exposed to a client driver or user space application.
 *****************************************************************************/
#include "ntb_lib.h"
#include "ntb_main.h"


/*****************************************************************************
 * Description: Writes to 16-bit registers.
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 */
void ntb_lib_write_16(void *mm_regs, uint32_t offset, int16_t value)
{
	if (offset >= NTB_PBAR_23_LIMIT_OFFSET) {
		/* value, iomem address */
		iowrite16(value, (char *)mm_regs + offset);
		NTB_DEBUG_PRINT(("NTBC WRITE16 offset  %p\n",
		((char *)mm_regs + offset)));
	}
}

/*****************************************************************************
 * Description: Writes to 32-bit registers.
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 */
void ntb_lib_write_32(void *mm_regs, uint32_t offset, uint32_t value)
{

	if (offset >= NTB_PBAR_23_LIMIT_OFFSET)
		/* value, iomem address */
		iowrite32(value, (uint8_t *)mm_regs + offset);
		//(unsigned int *) ((char *)mm_regs + offset) = value; 
}

/*****************************************************************************
 * Description: Writes to 64-bit registers.
 * Side Effects:
 * Assumptions:
 * Return Values: NONE
 */
void ntb_lib_write_64(void *mm_regs, uint64_t offset, uint64_t value)
{
	/* value, iomem address */
	uint32_t lower = value & LOWER_32;
	uint32_t upper = value >> BIT_SHIFT_32;

	NTB_DEBUG_PRINT(("NTBC WRITE64 VALUE  %Lx\n", value));
	if (offset >= NTB_PBAR_23_LIMIT_OFFSET) {

		iowrite32(lower, (uint8_t *)mm_regs + offset);
		iowrite32(upper, (uint8_t *)mm_regs + offset + OFFSET_4);

		NTB_DEBUG_PRINT(("NTBC WRITE64 MMREGS  %p\n",
				mm_regs));
	}

}

/*****************************************************************************
 * Description: Reads from 16-bit register.
 * Side Effects:
 * Assumptions:
 * Return Values: value of 16-bit register.
 */
uint16_t ntb_lib_read_16(void *mm_regs, uint32_t offset)
{
	if (offset >= NTB_PBAR_23_LIMIT_OFFSET)
		/* value, iomem address */
		return ioread16((uint8_t *)mm_regs + offset);
	else
		return -EPERM;
}

/*****************************************************************************
 * Description: Reads from any 32-bit register.
 * Reads registers.
 * Side Effects:
 * Assumptions:
 * Return Values: value of 32-bit register.
 */
uint32_t ntb_lib_read_32(void *mm_regs, uint32_t offset)
{
	if (offset >= NTB_PBAR_23_LIMIT_OFFSET)
		/* value, iomem address */
		return ioread32((uint8_t *)mm_regs + offset);
	else
		return -EPERM;
}

/*****************************************************************************
 * Description: Reads from 64-bit registers.
 * Side Effects:
 * Assumptions:
 * Return Values: value of 64-bit register.
 */
uint64_t ntb_lib_read_64(void *mm_regs, uint64_t offset)
{
	uint32_t lower = 0;
	int64_t reg_value = -EPERM;

	if (offset >= NTB_PBAR_23_LIMIT_OFFSET) {
		lower = ioread32((uint8_t *)mm_regs + offset);
		reg_value = ioread32((uint8_t *)mm_regs + offset + OFFSET_4);
		NTB_DEBUG_PRINT(("NTB: lib lower %x upper %Lx\n",
		lower, reg_value));
		reg_value = reg_value << BIT_SHIFT_32;
		reg_value = reg_value | lower;

	}

	return reg_value;
}

/*****************************************************************************
 * Description: Writes to consecutive scratch pad registers.
 * Side Effects:
 * Assumptions: Used mainly with the scratch pad registers. The var "count"
 * refers to how many 32 bit scratch pad registers we are writing.
 * Return Values: NONE
 */
void ntb_lib_write_rep(void *mm_regs, uint32_t offset,  void *pad,
		      uint32_t count)
{
	int i;
	uint32_t *spad;
	uint32_t *regs;

	spad = (uint32_t *)pad;
	regs = ((uint32_t *)mm_regs + (offset/sizeof(uint32_t)));

	if (offset >= NTB_PBAR_23_LIMIT_OFFSET)
		for (i = 0; i < count; i++)
			iowrite32(*spad++, regs++);


}



/*****************************************************************************
 * Description: Reads from consecutive scratch pad registers.
 * Side Effects:
 * Assumptions: Used mainly with the scratch pad registers. The var "count"
 * refers to how many 32 bit scratch pad registers we are reading.
 * Return Values: NONE
 */
void ntb_lib_read_rep(void *mm_regs, uint32_t offset, void *pad,
			uint32_t count)
{
	int i;
	uint32_t *spad;
	uint32_t *regs;
	spad = (uint32_t *)pad;
	regs = ((uint32_t *)mm_regs + (offset/sizeof(uint32_t)));

	if (offset >= NTB_PBAR_23_LIMIT_OFFSET)
		/* iomem address, buffer ptr to values, count  */
		for (i = 0; i < count; i++)
			*spad++ = ioread32(regs++);
}




