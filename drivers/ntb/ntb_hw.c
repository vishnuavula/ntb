/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2012-14 Intel Corporation. All rights reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   BSD LICENSE
 *
 *   Copyright(c) 2012-14 Intel Corporation. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copy
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
 * Intel PCIe NTB Linux driver
 *
 * Contact Information:
 * Jon Mason <jon.mason@intel.com>
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/random.h>
#include <linux/slab.h>
#include "ntb_hw.h"
#include "ntb_regs.h"

#define NTB_NAME	"Intel(R) PCI-E Non-Transparent Bridge Driver"
#define NTB_VER		"1.1"

MODULE_DESCRIPTION(NTB_NAME);
MODULE_VERSION(NTB_VER);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Intel Corporation");

static struct dentry *debugfs_dir;

/**
 * ntb_register_event_callback() - register event callback
 * @ndev: pointer to ntb_device instance
 * @func: callback function to register
 *
 * This function registers a callback for any HW driver events such as link
 * up/down, power management notices and etc.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_register_event_callback(struct ntb_device *ndev,
				void (*func)(void *handle,
					     enum ntb_hw_event event))
{
	if (ndev->event_cb)
		return -EINVAL;

	ndev->event_cb = func;

	return 0;
}

/**
 * ntb_unregister_event_callback() - unregisters the event callback
 * @ndev: pointer to ntb_device instance
 *
 * This function unregisters the existing callback from transport
 */
void ntb_unregister_event_callback(struct ntb_device *ndev)
{
	ndev->event_cb = NULL;
}

static void ntb_irq_work(unsigned long data)
{
	struct ntb_db_cb *db_cb = (struct ntb_db_cb *)data;
	int rc;

	rc = db_cb->callback(db_cb->data, db_cb->db_num);
	if (rc)
		tasklet_schedule(&db_cb->irq_work);
	else {
		struct ntb_device *ndev = db_cb->ndev;
		unsigned long mask;

		mask = readw(ndev->reg_ofs.ldb_mask);
		clear_bit(db_cb->db_num * ndev->bits_per_vector, &mask);
		writew(mask, ndev->reg_ofs.ldb_mask);
	}
}

/**
 * ntb_register_db_callback() - register a callback for doorbell interrupt
 * @ndev: pointer to ntb_device instance
 * @idx: doorbell index to register callback, zero based
 * @data: pointer to be returned to caller with every callback
 * @func: callback function to register
 *
 * This function registers a callback function for the doorbell interrupt
 * on the primary side. The function will unmask the doorbell as well to
 * allow interrupt.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_register_db_callback(struct ntb_device *ndev, unsigned int idx,
			     void *data, int (*func)(void *data, int db_num))
{
	unsigned long mask;

	if (idx >= ndev->max_cbs || ndev->db_cb[idx].callback) {
		dev_warn(&ndev->pdev->dev, "Invalid Index.\n");
		return -EINVAL;
	}

	ndev->db_cb[idx].callback = func;
	ndev->db_cb[idx].data = data;
	ndev->db_cb[idx].ndev = ndev;

	tasklet_init(&ndev->db_cb[idx].irq_work, ntb_irq_work,
		     (unsigned long) &ndev->db_cb[idx]);

	/* unmask interrupt */
	mask = readw(ndev->reg_ofs.ldb_mask);
	clear_bit(idx * ndev->bits_per_vector, &mask);
	writew(mask, ndev->reg_ofs.ldb_mask);

	return 0;
}

/**
 * ntb_unregister_db_callback() - unregister a callback for doorbell interrupt
 * @ndev: pointer to ntb_device instance
 * @idx: doorbell index to register callback, zero based
 *
 * This function unregisters a callback function for the doorbell interrupt
 * on the primary side. The function will also mask the said doorbell.
 */
void ntb_unregister_db_callback(struct ntb_device *ndev, unsigned int idx)
{
	unsigned long mask;

	if (idx >= ndev->max_cbs || !ndev->db_cb[idx].callback)
		return;

	mask = readw(ndev->reg_ofs.ldb_mask);
	set_bit(idx * ndev->bits_per_vector, &mask);
	writew(mask, ndev->reg_ofs.ldb_mask);

	tasklet_disable(&ndev->db_cb[idx].irq_work);

	ndev->db_cb[idx].callback = NULL;
}
EXPORT_SYMBOL_GPL(ntb_unregister_db_callback); //FIXME - is this necessary

/**
 * ntb_find_transport() - find the transport pointer
 * @transport: pointer to pci device
 *
 * Given the pci device pointer, return the transport pointer passed in when
 * the transport attached when it was inited.
 *
 * RETURNS: pointer to transport.
 */
void *ntb_find_transport(struct pci_dev *pdev)
{
	struct ntb_device *ndev = pci_get_drvdata(pdev);
	return ndev->ntb_transport;
}

/**
 * ntb_register_transport() - Register NTB transport with NTB HW driver
 * @transport: transport identifier
 *
 * This function allows a transport to reserve the hardware driver for
 * NTB usage.
 *
 * RETURNS: pointer to ntb_device, NULL on error.
 */
struct ntb_device *ntb_register_transport(struct pci_dev *pdev, void *transport)
{
	struct ntb_device *ndev = pci_get_drvdata(pdev);

	if (ndev->ntb_transport)
		return NULL;

	ndev->ntb_transport = transport;
	return ndev;
}

/**
 * ntb_unregister_transport() - Unregister the transport with the NTB HW driver
 * @ndev - ntb_device of the transport to be freed
 *
 * This function unregisters the transport from the HW driver and performs any
 * necessary cleanups.
 */
void ntb_unregister_transport(struct ntb_device *ndev)
{
	int i;

	if (!ndev->ntb_transport)
		return;

	for (i = 0; i < ndev->max_cbs; i++)
		ntb_unregister_db_callback(ndev, i);

	ntb_unregister_event_callback(ndev);
	ndev->ntb_transport = NULL;
}

/**
 * ntb_write_local_spad() - write to the secondary scratchpad register
 * @ndev: pointer to ntb_device instance
 * @idx: index to the scratchpad register, 0 based
 * @val: the data value to put into the register
 *
 * This function allows writing of a 32bit value to the indexed scratchpad
 * register. This writes over the data mirrored to the local scratchpad register
 * by the remote system.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_write_local_spad(struct ntb_device *ndev, unsigned int idx, u32 val)
{
	if (idx >= ndev->limits.max_spads)
		return -EINVAL;

	dev_dbg(&ndev->pdev->dev, "Writing %x to local scratch pad index %d\n",
		val, idx);
	writel(val, ndev->reg_ofs.spad_read + idx * 4);

	return 0;
}

/**
 * ntb_read_local_spad() - read from the primary scratchpad register
 * @ndev: pointer to ntb_device instance
 * @idx: index to scratchpad register, 0 based
 * @val: pointer to 32bit integer for storing the register value
 *
 * This function allows reading of the 32bit scratchpad register on
 * the primary (internal) side.  This allows the local system to read data
 * written and mirrored to the scratchpad register by the remote system.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_read_local_spad(struct ntb_device *ndev, unsigned int idx, u32 *val)
{
	if (idx >= ndev->limits.max_spads)
		return -EINVAL;

	*val = readl(ndev->reg_ofs.spad_write + idx * 4);
	dev_dbg(&ndev->pdev->dev,
		"Reading %x from local scratch pad index %d\n", *val, idx);

	return 0;
}

/**
 * ntb_write_remote_spad() - write to the secondary scratchpad register
 * @ndev: pointer to ntb_device instance
 * @idx: index to the scratchpad register, 0 based
 * @val: the data value to put into the register
 *
 * This function allows writing of a 32bit value to the indexed scratchpad
 * register. The register resides on the secondary (external) side.  This allows
 * the local system to write data to be mirrored to the remote systems
 * scratchpad register.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_write_remote_spad(struct ntb_device *ndev, unsigned int idx, u32 val)
{
	if (idx >= ndev->limits.max_spads)
		return -EINVAL;

	dev_dbg(&ndev->pdev->dev, "Writing %x to remote scratch pad index %d\n",
		val, idx);
	writel(val, ndev->reg_ofs.spad_write + idx * 4);

	return 0;
}

/**
 * ntb_read_remote_spad() - read from the primary scratchpad register
 * @ndev: pointer to ntb_device instance
 * @idx: index to scratchpad register, 0 based
 * @val: pointer to 32bit integer for storing the register value
 *
 * This function allows reading of the 32bit scratchpad register on
 * the primary (internal) side.  This alloows the local system to read the data
 * it wrote to be mirrored on the remote system.
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
int ntb_read_remote_spad(struct ntb_device *ndev, unsigned int idx, u32 *val)
{
	if (idx >= ndev->limits.max_spads)
		return -EINVAL;

	*val = readl(ndev->reg_ofs.spad_read + idx * 4);
	dev_dbg(&ndev->pdev->dev,
		"Reading %x from remote scratch pad index %d\n", *val, idx);

	return 0;
}

/**
 * ntb_get_mw_base() - get addr for the NTB memory window
 * @ndev: pointer to ntb_device instance
 * @mw: memory window number
 *
 * This function provides the base address of the memory window specified.
 *
 * RETURNS: address, or NULL on error.
 */
resource_size_t ntb_get_mw_base(struct ntb_device *ndev, unsigned int mw)
{
	if (mw >= ntb_max_mw(ndev))
		return 0;

	return pci_resource_start(ndev->pdev, MW_TO_BAR(mw));
}

/**
 * ntb_get_mw_vbase() - get virtual addr for the NTB memory window
 * @ndev: pointer to ntb_device instance
 * @mw: memory window number
 *
 * This function provides the base virtual address of the memory window
 * specified.
 *
 * RETURNS: pointer to virtual address, or NULL on error.
 */
void __iomem *ntb_get_mw_vbase(struct ntb_device *ndev, unsigned int mw)
{
	if (mw >= ntb_max_mw(ndev))
		return NULL;

	return ndev->mw[mw].vbase;
}

/**
 * ntb_get_mw_size() - return size of NTB memory window
 * @ndev: pointer to ntb_device instance
 * @mw: memory window number
 *
 * This function provides the physical size of the memory window specified
 *
 * RETURNS: the size of the memory window or zero on error
 */
u64 ntb_get_mw_size(struct ntb_device *ndev, unsigned int mw)
{
	if (mw >= ntb_max_mw(ndev))
		return 0;

	return ndev->mw[mw].bar_sz;
}

/**
 * ntb_set_mw_addr - set the memory window address
 * @ndev: pointer to ntb_device instance
 * @mw: memory window number
 * @addr: base address for data
 *
 * This function sets the base physical address of the memory window.  This
 * memory address is where data from the remote system will be transfered into
 * or out of depending on how the transport is configured.
 */
void ntb_set_mw_addr(struct ntb_device *ndev, unsigned int mw, u64 addr)
{
	if (mw >= ntb_max_mw(ndev))
		return;

	dev_dbg(&ndev->pdev->dev, "Writing addr %Lx to BAR %d\n", addr,
		MW_TO_BAR(mw));

	ndev->mw[mw].phys_addr = addr;

	switch (MW_TO_BAR(mw)) {
	case NTB_BAR_23:
		writeq(addr, ndev->reg_ofs.bar2_xlat);
		break;
	case NTB_BAR_45:
		writeq(addr, ndev->reg_ofs.bar4_xlat);
		break;
	}
}

/**
 * ntb_ring_doorbell() - Set the doorbell on the secondary/external side
 * @ndev: pointer to ntb_device instance
 * @db: doorbell to ring
 *
 * This function allows triggering of a doorbell on the secondary/external
 * side that will initiate an interrupt on the remote host
 *
 * RETURNS: An appropriate -ERRNO error value on error, or zero for success.
 */
void ntb_ring_doorbell(struct ntb_device *ndev, unsigned int db)
{
	dev_dbg(&ndev->pdev->dev, "%s: ringing doorbell %d\n", __func__, db);

	ndev->ring_doorbell(ndev, db);
}

static void ntb_setup_debugfs(struct ntb_device *ndev)
{
	if (!debugfs_initialized())
		return;

	if (!debugfs_dir)
		debugfs_dir = debugfs_create_dir(KBUILD_MODNAME, NULL);

	ndev->debugfs_dir = debugfs_create_dir(pci_name(ndev->pdev),
					       debugfs_dir);
}

static void ntb_free_debugfs(struct ntb_device *ndev)
{
	debugfs_remove_recursive(ndev->debugfs_dir);

	if (debugfs_dir && simple_empty(debugfs_dir)) {
		debugfs_remove_recursive(debugfs_dir);
		debugfs_dir = NULL;
	}
}

int ntb_setup_hw(struct ntb_device *ndev)
{
	int rc, i;

	/* The scratchpad registers keep the values between rmmod/insmod,
	 * blast them now
	 */
	for (i = 0; i < ndev->limits.max_spads; i++) {
		ntb_write_local_spad(ndev, i, 0);
		ntb_write_remote_spad(ndev, i, 0);
	}

	rc = ntb_transport_init(ndev->pdev);
	if (rc)
		goto err;

	ntb_setup_debugfs(ndev);

	return 0;

err:
	return rc;
}
EXPORT_SYMBOL_GPL(ntb_setup_hw);

void ntb_remove_hw(struct ntb_device *ndev)
{
	ntb_free_debugfs(ndev);
	ntb_transport_free(ndev->ntb_transport);
}
EXPORT_SYMBOL_GPL(ntb_remove_hw);
