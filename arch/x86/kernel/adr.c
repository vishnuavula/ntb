/*
 * platform device for ADR protected memory
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
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <asm/e820.h>

static struct resource adr_res[ADR_MAX_REGIONS];

static struct platform_device adr_device = {
	.name = "adr",
	.id = 0,
	.resource = adr_res,
};

static __init int register_adr_device(void)
{
	int rc, i;

	for (i = 0; i < ADR_MAX_REGIONS; i++)
		if (adr_resource[i]) {
			adr_res[i] = *adr_resource[i];
			adr_device.num_resources++;
		}

	if (!adr_device.num_resources)
		return 0;

	rc = platform_device_register(&adr_device);
	if (rc == 0)
		dev_info(&adr_device.dev, "registered platform adr device\n");

	return rc;
}

device_initcall(register_adr_device);
