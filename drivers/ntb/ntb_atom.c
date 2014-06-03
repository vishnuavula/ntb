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

#define BWD_LINK_RECOVERY_TIME	500

static const struct pci_device_id ntb_pci_tbl[] = {
	{PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_NTB_B2B_BWD)},
	{0}
};
MODULE_DEVICE_TABLE(pci, ntb_pci_tbl);

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
void ntb_atom_ring_doorbell(struct ntb_device *ndev, unsigned int db)
{
	dev_dbg(&ndev->pdev->dev, "%s: ringing doorbell %d\n", __func__, db);

	writeq((u64) 1 << db, ndev->reg_ofs.rdb);
}

static void atom_recover_link(struct ntb_device *ndev)
{
	u32 status;

	/* Driver resets the NTB ModPhy lanes - magic! */
	writeb(0xe0, ndev->reg_base + BWD_MODPHY_PCSREG6);
	writeb(0x40, ndev->reg_base + BWD_MODPHY_PCSREG4);
	writeb(0x60, ndev->reg_base + BWD_MODPHY_PCSREG4);
	writeb(0x60, ndev->reg_base + BWD_MODPHY_PCSREG6);

	/* Driver waits 100ms to allow the NTB ModPhy to settle */
	msleep(100);

	/* Clear AER Errors, write to clear */
	status = readl(ndev->reg_base + BWD_ERRCORSTS_OFFSET);
	dev_dbg(&ndev->pdev->dev, "ERRCORSTS = %x\n", status);
	status &= PCI_ERR_COR_REP_ROLL;
	writel(status, ndev->reg_base + BWD_ERRCORSTS_OFFSET);

	/* Clear unexpected electrical idle event in LTSSM, write to clear */
	status = readl(ndev->reg_base + BWD_LTSSMERRSTS0_OFFSET);
	dev_dbg(&ndev->pdev->dev, "LTSSMERRSTS0 = %x\n", status);
	status |= BWD_LTSSMERRSTS0_UNEXPECTEDEI;
	writel(status, ndev->reg_base + BWD_LTSSMERRSTS0_OFFSET);

	/* Clear DeSkew Buffer error, write to clear */
	status = readl(ndev->reg_base + BWD_DESKEWSTS_OFFSET);
	dev_dbg(&ndev->pdev->dev, "DESKEWSTS = %x\n", status);
	status |= BWD_DESKEWSTS_DBERR;
	writel(status, ndev->reg_base + BWD_DESKEWSTS_OFFSET);

	status = readl(ndev->reg_base + BWD_IBSTERRRCRVSTS0_OFFSET);
	dev_dbg(&ndev->pdev->dev, "IBSTERRRCRVSTS0 = %x\n", status);
	status &= BWD_IBIST_ERR_OFLOW;
	writel(status, ndev->reg_base + BWD_IBSTERRRCRVSTS0_OFFSET);

	/* Releases the NTB state machine to allow the link to retrain */
	status = readl(ndev->reg_base + BWD_LTSSMSTATEJMP_OFFSET);
	dev_dbg(&ndev->pdev->dev, "LTSSMSTATEJMP = %x\n", status);
	status &= ~BWD_LTSSMSTATEJMP_FORCEDETECT;
	writel(status, ndev->reg_base + BWD_LTSSMSTATEJMP_OFFSET);
}

static void ntb_link_event(struct ntb_device *ndev, int link_state)
{
	unsigned int event;

	if (ndev->link_status == link_state)
		return;

	if (link_state == NTB_LINK_UP) {
		u16 status;

		dev_info(&ndev->pdev->dev, "Link Up\n");
		ndev->link_status = NTB_LINK_UP;
		event = NTB_EVENT_HW_LINK_UP;

		status = readw(ndev->reg_ofs.lnk_stat);

		ndev->link_width = (status & NTB_LINK_WIDTH_MASK) >> 4;
		ndev->link_speed = (status & NTB_LINK_SPEED_MASK);
		dev_info(&ndev->pdev->dev, "Link Width %d, Link Speed %d\n",
			 ndev->link_width, ndev->link_speed);
	} else {
		dev_info(&ndev->pdev->dev, "Link Down\n");
		ndev->link_status = NTB_LINK_DOWN;
		event = NTB_EVENT_HW_LINK_DOWN;
		/* Don't modify link width/speed, we need it in link recovery */
	}

	/* notify the upper layer if we have an event change */
	if (ndev->event_cb)
		ndev->event_cb(ndev->ntb_transport, event);
}

static int ntb_link_status(struct ntb_device *ndev)
{
	int link_state;
	u32 ntb_cntl;

	ntb_cntl = readl(ndev->reg_ofs.lnk_cntl);
	if (ntb_cntl & BWD_CNTL_LINK_DOWN)
		link_state = NTB_LINK_DOWN;
	else
		link_state = NTB_LINK_UP;

	ntb_link_event(ndev, link_state);

	return 0;
}

static void atom_link_recovery(struct work_struct *work)
{
	struct ntb_device *ndev = container_of(work, struct ntb_device,
					       lr_timer.work);
	u32 status32;

	atom_recover_link(ndev);
	/* There is a potential race between the 2 NTB devices recovering at the
	 * same time.  If the times are the same, the link will not recover and
	 * the driver will be stuck in this loop forever.  Add a random interval
	 * to the recovery time to prevent this race.
	 */
	msleep(BWD_LINK_RECOVERY_TIME + prandom_u32() % BWD_LINK_RECOVERY_TIME);

	status32 = readl(ndev->reg_base + BWD_LTSSMSTATEJMP_OFFSET);
	if (status32 & BWD_LTSSMSTATEJMP_FORCEDETECT)
		goto retry;

	status32 = readl(ndev->reg_base + BWD_IBSTERRRCRVSTS0_OFFSET);
	if (status32 & BWD_IBIST_ERR_OFLOW)
		goto retry;

	status32 = readl(ndev->reg_ofs.lnk_cntl);
	if (!(status32 & BWD_CNTL_LINK_DOWN)) {
		unsigned char speed, width;
		u16 status16;

		status16 = readw(ndev->reg_ofs.lnk_stat);
		width = (status16 & NTB_LINK_WIDTH_MASK) >> 4;
		speed = (status16 & NTB_LINK_SPEED_MASK);
		if (ndev->link_width != width || ndev->link_speed != speed)
			goto retry;
	}

	schedule_delayed_work(&ndev->hb_timer, NTB_HB_TIMEOUT);
	return;

retry:
	schedule_delayed_work(&ndev->lr_timer, NTB_HB_TIMEOUT);
}

/* BWD doesn't have link status interrupt, poll on that platform */
static void atom_link_poll(struct work_struct *work)
{
	struct ntb_device *ndev = container_of(work, struct ntb_device,
					       hb_timer.work);
	unsigned long ts = jiffies;

	/* If we haven't gotten an interrupt in a while, check the BWD link
	 * status bit
	 */
	if (ts > ndev->last_ts + NTB_HB_TIMEOUT) {
		int rc = ntb_link_status(ndev);
		if (rc)
			dev_err(&ndev->pdev->dev,
				"Error determining link status\n");

		/* Check to see if a link error is the cause of the link down */
		if (ndev->link_status == NTB_LINK_DOWN) {
			u32 status32 = readl(ndev->reg_base +
					     BWD_LTSSMSTATEJMP_OFFSET);
			if (status32 & BWD_LTSSMSTATEJMP_FORCEDETECT) {
				schedule_delayed_work(&ndev->lr_timer, 0);
				return;
			}
		}
	}

	schedule_delayed_work(&ndev->hb_timer, NTB_HB_TIMEOUT);
}

static int ntb_atom_setup(struct ntb_device *ndev)
{
	int rc;
	u32 val;

	rc = pci_read_config_dword(ndev->pdev, NTB_PPD_OFFSET, &val);
	if (rc)
		return rc;

	switch ((val & BWD_PPD_CONN_TYPE) >> 8) {
	case NTB_CONN_B2B:
		ndev->conn_type = NTB_CONN_B2B;
		break;
	case NTB_CONN_RP:
	default:
		dev_err(&ndev->pdev->dev, "Unsupported NTB configuration\n");
		return -EINVAL;
	}

	if (val & BWD_PPD_DEV_TYPE)
		ndev->dev_type = NTB_DEV_DSD;
	else
		ndev->dev_type = NTB_DEV_USD;

	/* Initiate PCI-E link training */
	rc = pci_write_config_dword(ndev->pdev, NTB_PPD_OFFSET,
				    val | BWD_PPD_INIT_LINK);
	if (rc)
		return rc;

	ndev->reg_ofs.ldb = ndev->reg_base + BWD_PDOORBELL_OFFSET;
	ndev->reg_ofs.ldb_mask = ndev->reg_base + BWD_PDBMSK_OFFSET;
	ndev->reg_ofs.rdb = ndev->reg_base + BWD_B2B_DOORBELL_OFFSET;
	ndev->reg_ofs.bar2_xlat = ndev->reg_base + BWD_SBAR2XLAT_OFFSET;
	ndev->reg_ofs.bar4_xlat = ndev->reg_base + BWD_SBAR4XLAT_OFFSET;
	ndev->reg_ofs.lnk_cntl = ndev->reg_base + BWD_NTBCNTL_OFFSET;
	ndev->reg_ofs.lnk_stat = ndev->reg_base + BWD_LINK_STATUS_OFFSET;
	ndev->reg_ofs.spad_read = ndev->reg_base + BWD_SPAD_OFFSET;
	ndev->reg_ofs.spad_write = ndev->reg_base + BWD_B2B_SPAD_OFFSET;
	ndev->reg_ofs.spci_cmd = ndev->reg_base + BWD_PCICMD_OFFSET;
	ndev->limits.max_mw = BWD_MAX_MW;
	ndev->limits.max_spads = BWD_MAX_SPADS;
	ndev->limits.max_db_bits = BWD_MAX_DB_BITS;
	ndev->limits.msix_cnt = BWD_MSIX_CNT;
	ndev->bits_per_vector = BWD_DB_BITS_PER_VEC;

	/* Since atom doesn't have a link interrupt, setup a poll timer */
	INIT_DELAYED_WORK(&ndev->hb_timer, atom_link_poll);
	INIT_DELAYED_WORK(&ndev->lr_timer, atom_link_recovery);
	schedule_delayed_work(&ndev->hb_timer, NTB_HB_TIMEOUT);

	return 0;
}

static int ntb_device_setup(struct ntb_device *ndev)
{
	int rc;

	rc = ntb_atom_setup(ndev);
	if (rc)
		return rc;

	dev_info(&ndev->pdev->dev, "Device Type = %s\n",
		 ndev->dev_type == NTB_DEV_USD ? "USD/DSP" : "DSD/USP");

	if (ndev->conn_type == NTB_CONN_B2B)
		/* Enable Bus Master and Memory Space on the secondary side */
		writew(PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER,
		       ndev->reg_ofs.spci_cmd);

	return 0;
}

static void ntb_device_free(struct ntb_device *ndev)
{
	cancel_delayed_work_sync(&ndev->hb_timer);
	cancel_delayed_work_sync(&ndev->lr_timer);
}

static int ntb_create_callbacks(struct ntb_device *ndev)
{
	int i;

	/* Chicken-egg issue.  We won't know how many callbacks are necessary
	 * until we see how many MSI-X vectors we get, but these pointers need
	 * to be passed into the MSI-X register function.  So, we allocate the
	 * max, knowing that they might not all be used, to work around this.
	 */
	ndev->db_cb = kcalloc(ndev->limits.max_db_bits,
			      sizeof(struct ntb_db_cb),
			      GFP_KERNEL);
	if (!ndev->db_cb)
		return -ENOMEM;

	for (i = 0; i < ndev->limits.max_db_bits; i++) {
		ndev->db_cb[i].db_num = i;
		ndev->db_cb[i].ndev = ndev;
	}

	return 0;
}

static void ntb_free_callbacks(struct ntb_device *ndev)
{
	int i;

	for (i = 0; i < ndev->limits.max_db_bits; i++)
		ntb_unregister_db_callback(ndev, i);

	kfree(ndev->db_cb);
}

static irqreturn_t atom_callback_msix_irq(int irq, void *data)
{
	struct ntb_db_cb *db_cb = data;
	struct ntb_device *ndev = db_cb->ndev;
	unsigned long mask;

	dev_dbg(&ndev->pdev->dev, "MSI-X irq %d received for DB %d\n", irq,
		db_cb->db_num);

	mask = readw(ndev->reg_ofs.ldb_mask);
	set_bit(db_cb->db_num * ndev->bits_per_vector, &mask);
	writew(mask, ndev->reg_ofs.ldb_mask);

	tasklet_schedule(&db_cb->irq_work);

	/* No need to check for the specific HB irq, any interrupt means
	 * we're connected.
	 */
	ndev->last_ts = jiffies;

	writeq((u64) 1 << db_cb->db_num, ndev->reg_ofs.ldb);

	return IRQ_HANDLED;
}

static irqreturn_t ntb_interrupt(int irq, void *dev)
{
	struct ntb_device *ndev = dev;
	unsigned int i = 0;
	u64 ldb = readq(ndev->reg_ofs.ldb);

	dev_dbg(&ndev->pdev->dev, "irq %d - ldb = %Lx\n", irq, ldb);

	while (ldb) {
		i = __ffs(ldb);
		ldb &= ldb - 1;
		atom_callback_msix_irq(irq, &ndev->db_cb[i]);
	}

	return IRQ_HANDLED;
}

static int ntb_atom_setup_msix(struct ntb_device *ndev, int msix_entries)
{
	struct pci_dev *pdev = ndev->pdev;
	struct msix_entry *msix;
	int rc, i;

	msix_entries = pci_enable_msix_range(pdev, ndev->msix_entries,
					     1, msix_entries);
	if (msix_entries < 0)
		return msix_entries;

	for (i = 0; i < msix_entries; i++) {
		msix = &ndev->msix_entries[i];
		WARN_ON(!msix->vector);

		rc = request_irq(msix->vector, atom_callback_msix_irq, 0,
				 "ntb-callback-msix", &ndev->db_cb[i]);
		if (rc)
			goto err;
	}

	ndev->num_msix = msix_entries;
	ndev->max_cbs = msix_entries;

	return 0;

err:
	while (--i >= 0)
		free_irq(msix->vector, &ndev->db_cb[i]);

	pci_disable_msix(pdev);
	ndev->num_msix = 0;

	return rc;
}

static int ntb_setup_msix(struct ntb_device *ndev)
{
	struct pci_dev *pdev = ndev->pdev;
	int msix_entries;
	int rc, i;

	msix_entries = pci_msix_vec_count(pdev);
	if (msix_entries < 0) {
		rc = msix_entries;
		goto err;
	} else if (msix_entries > ndev->limits.msix_cnt) {
		rc = -EINVAL;
		goto err;
	}

	ndev->msix_entries = kmalloc(sizeof(struct msix_entry) * msix_entries,
				     GFP_KERNEL);
	if (!ndev->msix_entries) {
		rc = -ENOMEM;
		goto err;
	}

	for (i = 0; i < msix_entries; i++)
		ndev->msix_entries[i].entry = i;

	rc = ntb_atom_setup_msix(ndev, msix_entries);
	if (rc)
		goto err1;

	return 0;

err1:
	kfree(ndev->msix_entries);
err:
	dev_err(&pdev->dev, "Error allocating MSI-X interrupt\n");
	return rc;
}

static int ntb_setup_msi(struct ntb_device *ndev)
{
	struct pci_dev *pdev = ndev->pdev;
	int rc;

	rc = pci_enable_msi(pdev);
	if (rc)
		return rc;

	rc = request_irq(pdev->irq, ntb_interrupt, 0, "ntb-msi", ndev);
	if (rc) {
		pci_disable_msi(pdev);
		dev_err(&pdev->dev, "Error allocating MSI interrupt\n");
		return rc;
	}

	return 0;
}

static int ntb_setup_intx(struct ntb_device *ndev)
{
	struct pci_dev *pdev = ndev->pdev;
	int rc;

	pci_msi_off(pdev);

	/* Verify intx is enabled */
	pci_intx(pdev, 1);

	rc = request_irq(pdev->irq, ntb_interrupt, IRQF_SHARED, "ntb-intx",
			 ndev);
	if (rc)
		return rc;

	return 0;
}

static int ntb_setup_interrupts(struct ntb_device *ndev)
{
	int rc;

	writeq(~0, ndev->reg_ofs.ldb_mask);

	rc = ntb_setup_msix(ndev);
	if (!rc)
		goto done;

	ndev->bits_per_vector = 1;
	ndev->max_cbs = ndev->limits.max_db_bits;

	rc = ntb_setup_msi(ndev);
	if (!rc)
		goto done;

	rc = ntb_setup_intx(ndev);
	if (rc) {
		dev_err(&ndev->pdev->dev, "no usable interrupts\n");
		return rc;
	}

done:
	ndev->ring_doorbell = ntb_atom_ring_doorbell;
	return 0;
}

static void ntb_free_interrupts(struct ntb_device *ndev)
{
	struct pci_dev *pdev = ndev->pdev;

	/* mask interrupts */
	writeq(~0, ndev->reg_ofs.ldb_mask);

	if (ndev->num_msix) {
		struct msix_entry *msix;
		u32 i;

		for (i = 0; i < ndev->num_msix; i++) {
			msix = &ndev->msix_entries[i];
			free_irq(msix->vector, &ndev->db_cb[i]);
		}
		pci_disable_msix(pdev);
		kfree(ndev->msix_entries);
	} else {
		free_irq(pdev->irq, ndev);

		if (pci_dev_msi_enabled(pdev))
			pci_disable_msi(pdev);
	}
}

static void ntb_hw_link_up(struct ntb_device *ndev)
{
	u32 ntb_cntl;

	/* Let's bring the NTB link up */
	ntb_cntl = readl(ndev->reg_ofs.lnk_cntl);
	ntb_cntl &= ~(NTB_CNTL_LINK_DISABLE | NTB_CNTL_CFG_LOCK);
	ntb_cntl |= NTB_CNTL_P2S_BAR23_SNOOP | NTB_CNTL_S2P_BAR23_SNOOP;
	ntb_cntl |= NTB_CNTL_P2S_BAR45_SNOOP | NTB_CNTL_S2P_BAR45_SNOOP;
	writel(ntb_cntl, ndev->reg_ofs.lnk_cntl);
}

static void ntb_hw_link_down(struct ntb_device *ndev)
{
	u32 ntb_cntl;

	/* Bring NTB link down */
	ntb_cntl = readl(ndev->reg_ofs.lnk_cntl);
	ntb_cntl &= ~(NTB_CNTL_P2S_BAR23_SNOOP | NTB_CNTL_S2P_BAR23_SNOOP);
	ntb_cntl &= ~(NTB_CNTL_P2S_BAR45_SNOOP | NTB_CNTL_S2P_BAR45_SNOOP);
	ntb_cntl |= NTB_CNTL_LINK_DISABLE | NTB_CNTL_CFG_LOCK;
	writel(ntb_cntl, ndev->reg_ofs.lnk_cntl);
}

static int ntb_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ntb_device *ndev;
	int rc, i;

	ndev = kzalloc(sizeof(struct ntb_device), GFP_KERNEL);
	if (!ndev)
		return -ENOMEM;

	ndev->pdev = pdev;
	ndev->link_status = NTB_LINK_DOWN;
	pci_set_drvdata(pdev, ndev);

	rc = pci_enable_device(pdev);
	if (rc)
		goto err;

	pci_set_master(ndev->pdev);

	rc = pci_request_selected_regions(pdev, NTB_BAR_MASK, KBUILD_MODNAME);
	if (rc)
		goto err1;

	ndev->reg_base = pci_ioremap_bar(pdev, NTB_BAR_MMIO);
	if (!ndev->reg_base) {
		dev_warn(&pdev->dev, "Cannot remap BAR 0\n");
		rc = -EIO;
		goto err2;
	}

	for (i = 0; i < NTB_MAX_NUM_MW; i++) {
		ndev->mw[i].bar_sz = pci_resource_len(pdev, MW_TO_BAR(i));
		ndev->mw[i].vbase =
		    ioremap_wc(pci_resource_start(pdev, MW_TO_BAR(i)),
			       ndev->mw[i].bar_sz);
		dev_info(&pdev->dev, "MW %d size %llu\n", i,
			 (unsigned long long) ndev->mw[i].bar_sz);
		if (!ndev->mw[i].vbase) {
			dev_warn(&pdev->dev, "Cannot remap BAR %d\n",
				 MW_TO_BAR(i));
			rc = -EIO;
			goto err3;
		}
	}

	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (rc)
			goto err3;

		dev_warn(&pdev->dev, "Cannot DMA highmem\n");
	}

	rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
		if (rc)
			goto err3;

		dev_warn(&pdev->dev, "Cannot DMA consistent highmem\n");
	}

	rc = ntb_device_setup(ndev);
	if (rc)
		goto err3;

	rc = ntb_create_callbacks(ndev);
	if (rc)
		goto err4;

	rc = ntb_setup_interrupts(ndev);
	if (rc)
		goto err5;

	rc = ntb_setup_hw(ndev);
	if (rc)
		goto err6;

	ntb_hw_link_up(ndev);

	return 0;

err6:
	ntb_free_interrupts(ndev);
err5:
	ntb_free_callbacks(ndev);
err4:
	ntb_device_free(ndev);
err3:
	for (i--; i >= 0; i--)
		iounmap(ndev->mw[i].vbase);
	iounmap(ndev->reg_base);
err2:
	pci_release_selected_regions(pdev, NTB_BAR_MASK);
err1:
	pci_disable_device(pdev);
err:
	kfree(ndev);

	dev_err(&pdev->dev, "Error loading %s module\n", KBUILD_MODNAME);
	return rc;
}

static void ntb_pci_remove(struct pci_dev *pdev)
{
	struct ntb_device *ndev = pci_get_drvdata(pdev);
	int i;

	ntb_hw_link_down(ndev);
	ntb_remove_hw(ndev);

	ntb_free_interrupts(ndev);
	ntb_free_callbacks(ndev);
	ntb_device_free(ndev);

	for (i = 0; i < NTB_MAX_NUM_MW; i++)
		iounmap(ndev->mw[i].vbase);

	iounmap(ndev->reg_base);
	pci_release_selected_regions(pdev, NTB_BAR_MASK);
	pci_disable_device(pdev);
	kfree(ndev);
}

static struct pci_driver ntb_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = ntb_pci_tbl,
	.probe = ntb_pci_probe,
	.remove = ntb_pci_remove,
};
module_pci_driver(ntb_pci_driver);
