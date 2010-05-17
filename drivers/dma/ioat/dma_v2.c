/*
 * Intel I/OAT DMA Linux driver
 * Copyright(c) 2004 - 2009 Intel Corporation.
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
 */

/*
 * This driver supports an Intel I/OAT DMA engine (versions >= 2), which
 * does asynchronous data movement and checksumming operations.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/i7300_idle.h>
#include "dma.h"
#include "dma_v2.h"
#include "registers.h"
#include "hw.h"

int ioat_ring_alloc_order = 8;
module_param(ioat_ring_alloc_order, int, 0644);
MODULE_PARM_DESC(ioat_ring_alloc_order,
		 "ioat2+: allocate 2^n descriptors per channel"
		 " (default: 8 max: 16)");
static int ioat_ring_max_alloc_order = IOAT_MAX_ORDER;
module_param(ioat_ring_max_alloc_order, int, 0644);
MODULE_PARM_DESC(ioat_ring_max_alloc_order,
		 "ioat2+: upper limit for ring size (default: 16)");

void __ioat2_issue_pending(struct ioat2_dma_chan *ioat)
{
	struct ioat_chan_common *chan = &ioat->base;

	ioat->dmacount += ioat2_ring_pending(ioat);
	ioat2_set_issued(ioat, ioat->head);
	writew(ioat->dmacount, chan->reg_base + IOAT_CHAN_DMACOUNT_OFFSET);
	dev_dbg(to_dev(chan),
		"%s: head: %#x tail: %#x issued: %#x count: %#x\n",
		__func__, ioat->head, ioat->tail, ioat->issued, ioat->dmacount);
}

void ioat2_issue_pending(struct dma_chan *c)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);

	if (ioat2_ring_pending(ioat)) {
		spin_lock_bh(&ioat->prep_lock);
		__ioat2_issue_pending(ioat);
		spin_unlock_bh(&ioat->prep_lock);
	}
}

/**
 * ioat2_update_pending - log pending descriptors
 * @ioat: ioat2+ channel
 *
 * Check if the number of unsubmitted descriptors has exceeded the
 * watermark.  Called with prep_lock held
 */
static void ioat2_update_pending(struct ioat2_dma_chan *ioat)
{
	if (ioat2_ring_pending(ioat) > ioat_pending_level)
		__ioat2_issue_pending(ioat);
}

static void __ioat2_start_null_desc(struct ioat2_dma_chan *ioat)
{
	struct ioat2_ring_ent *desc;
	struct ioat_dma_descriptor *hw;

	if (ioat2_ring_space(ioat) < 1) {
		dev_err(to_dev(&ioat->base),
			"Unable to start null desc - ring full\n");
		return;
	}

	dev_dbg(to_dev(&ioat->base), "%s: head: %#x tail: %#x issued: %#x\n",
		__func__, ioat->head, ioat->tail, ioat->issued);
	desc = ioat2_get_ring_ent(ioat, ioat->head);

	hw = to_hw(desc);
	hw->ctl = 0;
	hw->ctl_f.null = 1;
	hw->ctl_f.int_en = 1;
	hw->ctl_f.compl_write = 1;
	/* set size to non-zero value (channel returns error when size is 0) */
	hw->size = NULL_DESC_BUFFER_SIZE;
	hw->src_addr = 0;
	hw->dst_addr = 0;
	async_tx_ack(&desc->txd);
	ioat2_set_chainaddr(ioat, desc->txd.phys);
	__dump_desc_dbg(&ioat->base, to_hw(desc), &desc->txd);
	wmb();
	ioat2_inc_head(ioat, 1);
	__ioat2_issue_pending(ioat);
}

static void ioat2_start_null_desc(struct ioat2_dma_chan *ioat)
{
	spin_lock_bh(&ioat->prep_lock);
	__ioat2_start_null_desc(ioat);
	spin_unlock_bh(&ioat->prep_lock);
}

static void __cleanup(struct ioat2_dma_chan *ioat, unsigned long phys_complete)
{
	struct ioat_chan_common *chan = &ioat->base;
	struct dma_async_tx_descriptor *tx;
	struct ioat2_ring_ent *desc;
	bool seen_current = false;
	u16 active;
	int idx = ioat->tail, i;

	dev_dbg(to_dev(chan), "%s: head: %#x tail: %#x issued: %#x\n",
		__func__, ioat->head, ioat->tail, ioat->issued);

	active = ioat2_ring_active(ioat);
	for (i = 0; i < active && !seen_current; i++) {
		smp_read_barrier_depends();
		prefetch(ioat2_get_ring_ent(ioat, idx + i + 1));
		desc = ioat2_get_ring_ent(ioat, idx + i);
		tx = &desc->txd;
		__dump_desc_dbg(&ioat->base, to_hw(desc), &desc->txd);
		if (tx->cookie) {
			ioat_dma_unmap(chan, tx->flags, desc->len, to_hw(desc));
			chan->completed_cookie = tx->cookie;
			tx->cookie = 0;
			if (tx->callback) {
				tx->callback(tx->callback_param);
				tx->callback = NULL;
			}
		}

		if (tx->phys == phys_complete)
			seen_current = true;
	}
	smp_mb(); /* finish all descriptor reads before incrementing tail */
	ioat2_inc_tail(ioat, i);
	BUG_ON(active && !seen_current); /* no active descs have written a completion? */

	chan->last_completion = phys_complete;
	if (active - i == 0) {
		dev_dbg(to_dev(chan), "%s: cancel completion timeout\n",
			__func__);
		clear_bit(IOAT_COMPLETION_PENDING, &chan->state);
		mod_timer(&chan->timer, jiffies + IDLE_TIMEOUT);
	}
}

/**
 * ioat2_cleanup - clean finished descriptors (advance tail pointer)
 * @chan: ioat channel to be cleaned up
 */
static void ioat2_cleanup(struct ioat2_dma_chan *ioat)
{
	struct ioat_chan_common *chan = &ioat->base;
	unsigned long phys_complete;

	spin_lock_bh(&chan->cleanup_lock);
	if (ioat_cleanup_preamble(chan, &phys_complete))
		__cleanup(ioat, phys_complete);
	spin_unlock_bh(&chan->cleanup_lock);
}

void ioat2_cleanup_event(unsigned long data)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan((void *) data);

	ioat2_cleanup(ioat);
	writew(IOAT_CHANCTRL_RUN, ioat->base.reg_base + IOAT_CHANCTRL_OFFSET);
}

void __ioat2_restart_chan(struct ioat2_dma_chan *ioat)
{
	struct ioat_chan_common *chan = &ioat->base;

	/* set the tail to be re-issued */
	ioat2_set_issued(ioat, ioat->tail);
	ioat->dmacount = 0;
	set_bit(IOAT_COMPLETION_PENDING, &chan->state);
	mod_timer(&chan->timer, jiffies + COMPLETION_TIMEOUT);

	dev_dbg(to_dev(chan),
		"%s: head: %#x tail: %#x issued: %#x count: %#x\n",
		__func__, ioat->head, ioat->tail, ioat->issued, ioat->dmacount);

	if (ioat2_ring_pending(ioat)) {
		struct ioat2_ring_ent *desc;

		desc = ioat2_get_ring_ent(ioat, ioat->tail);
		ioat2_set_chainaddr(ioat, desc->txd.phys);
		__ioat2_issue_pending(ioat);
	} else
		__ioat2_start_null_desc(ioat);
}

int ioat2_quiesce(struct ioat_chan_common *chan, unsigned long tmo)
{
	unsigned long end = jiffies + tmo;
	int err = 0;
	u32 status;

	status = ioat_chansts(chan);
	if (is_ioat_active(status) || is_ioat_idle(status))
		ioat_suspend(chan);
	while (is_ioat_active(status) || is_ioat_idle(status)) {
		if (tmo && time_after(jiffies, end)) {
			err = -ETIMEDOUT;
			break;
		}
		status = ioat_chansts(chan);
		cpu_relax();
	}

	return err;
}

int ioat2_reset_sync(struct ioat_chan_common *chan, unsigned long tmo)
{
	unsigned long end = jiffies + tmo;
	int err = 0;

	ioat_reset(chan);
	while (ioat_reset_pending(chan)) {
		if (end && time_after(jiffies, end)) {
			err = -ETIMEDOUT;
			break;
		}
		cpu_relax();
	}

	return err;
}

static void ioat2_restart_channel(struct ioat2_dma_chan *ioat)
{
	struct ioat_chan_common *chan = &ioat->base;
	unsigned long phys_complete;

	ioat2_quiesce(chan, 0);
	if (ioat_cleanup_preamble(chan, &phys_complete))
		__cleanup(ioat, phys_complete);

	__ioat2_restart_chan(ioat);
}

void ioat2_timer_event(unsigned long data)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan((void *) data);
	struct ioat_chan_common *chan = &ioat->base;

	if (test_bit(IOAT_COMPLETION_PENDING, &chan->state)) {
		unsigned long phys_complete;
		u64 status;

		status = ioat_chansts(chan);

		/* when halted due to errors check for channel
		 * programming errors before advancing the completion state
		 */
		if (is_ioat_halted(status)) {
			u32 chanerr;

			chanerr = readl(chan->reg_base + IOAT_CHANERR_OFFSET);
			dev_err(to_dev(chan), "%s: Channel halted (%x)\n",
				__func__, chanerr);
			BUG_ON(is_ioat_bug(chanerr));
		}

		/* if we haven't made progress and we have already
		 * acknowledged a pending completion once, then be more
		 * forceful with a restart
		 */
		spin_lock_bh(&chan->cleanup_lock);
		if (ioat_cleanup_preamble(chan, &phys_complete)) {
			__cleanup(ioat, phys_complete);
		} else if (test_bit(IOAT_COMPLETION_ACK, &chan->state)) {
			spin_lock_bh(&ioat->prep_lock);
			ioat2_restart_channel(ioat);
			spin_unlock_bh(&ioat->prep_lock);
		} else {
			set_bit(IOAT_COMPLETION_ACK, &chan->state);
			mod_timer(&chan->timer, jiffies + COMPLETION_TIMEOUT);
		}
		spin_unlock_bh(&chan->cleanup_lock);
	} else {
		u16 active;

		/* if the ring is idle, empty, and oversized try to step
		 * down the size
		 */
		spin_lock_bh(&chan->cleanup_lock);
		spin_lock_bh(&ioat->prep_lock);
		active = ioat2_ring_active(ioat);
		if (active == 0 && ioat->alloc_order > ioat_get_alloc_order())
			reshape_ring(ioat, ioat->alloc_order-1);
		spin_unlock_bh(&ioat->prep_lock);
		spin_unlock_bh(&chan->cleanup_lock);

		/* keep shrinking until we get back to our minimum
		 * default size
		 */
		if (ioat->alloc_order > ioat_get_alloc_order())
			mod_timer(&chan->timer, jiffies + IDLE_TIMEOUT);
	}
}

static int ioat2_reset_hw(struct ioat_chan_common *chan)
{
	/* throw away whatever the channel was doing and get it initialized */
	u32 chanerr;

	ioat2_quiesce(chan, msecs_to_jiffies(100));

	chanerr = readl(chan->reg_base + IOAT_CHANERR_OFFSET);
	writel(chanerr, chan->reg_base + IOAT_CHANERR_OFFSET);

	return ioat2_reset_sync(chan, msecs_to_jiffies(200));
}

/**
 * ioat2_enumerate_channels - find and initialize the device's channels
 * @device: the device to be enumerated
 */
int ioat2_enumerate_channels(struct ioatdma_device *device)
{
	struct ioat2_dma_chan *ioat;
	struct device *dev = &device->pdev->dev;
	struct dma_device *dma = &device->common;
	u8 xfercap_log;
	int i;

	INIT_LIST_HEAD(&dma->channels);
	dma->chancnt = readb(device->reg_base + IOAT_CHANCNT_OFFSET);
	dma->chancnt &= 0x1f; /* bits [4:0] valid */
	if (dma->chancnt > ARRAY_SIZE(device->idx)) {
		dev_warn(dev, "(%d) exceeds max supported channels (%zu)\n",
			 dma->chancnt, ARRAY_SIZE(device->idx));
		dma->chancnt = ARRAY_SIZE(device->idx);
	}
	xfercap_log = readb(device->reg_base + IOAT_XFERCAP_OFFSET);
	xfercap_log &= 0x1f; /* bits [4:0] valid */
	if (xfercap_log == 0)
		return 0;
	dev_dbg(dev, "%s: xfercap = %d\n", __func__, 1 << xfercap_log);

	/* FIXME which i/oat version is i7300? */
#ifdef CONFIG_I7300_IDLE_IOAT_CHANNEL
	if (i7300_idle_platform_probe(NULL, NULL, 1) == 0)
		dma->chancnt--;
#endif
	for (i = 0; i < dma->chancnt; i++) {
		ioat = devm_kzalloc(dev, sizeof(*ioat), GFP_KERNEL);
		if (!ioat)
			break;

		ioat_init_channel(device, &ioat->base, i);
		ioat->xfercap_log = xfercap_log;
		spin_lock_init(&ioat->prep_lock);
		if (device->reset_hw(&ioat->base)) {
			i = 0;
			break;
		}
	}
	dma->chancnt = i;
	return i;
}

static dma_cookie_t ioat2_tx_submit_unlock(struct dma_async_tx_descriptor *tx)
{
	struct dma_chan *c = tx->chan;
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);
	struct ioat_chan_common *chan = &ioat->base;
	dma_cookie_t cookie = c->cookie;

	cookie++;
	if (cookie < 0)
		cookie = 1;
	tx->cookie = cookie;
	c->cookie = cookie;
	dev_dbg(to_dev(&ioat->base), "%s: cookie: %d\n", __func__, cookie);

	if (!test_and_set_bit(IOAT_COMPLETION_PENDING, &chan->state))
		mod_timer(&chan->timer, jiffies + COMPLETION_TIMEOUT);

	/* make descriptor updates visible before advancing ioat->head,
	 * this is purposefully not smp_wmb() since we are also
	 * publishing the descriptor updates to a dma device
	 */
	wmb();

	ioat2_inc_head(ioat, ioat->produce);

	ioat2_update_pending(ioat);
	spin_unlock_bh(&ioat->prep_lock);

	return cookie;
}

static void ioat2_init_page(struct ioat2_dma_chan *ioat, struct ioat2_ring_page *page)
{
	struct ioat_chan_common *chan = &ioat->base;
	struct dma_chan *c = &chan->common;
	struct device *dev = to_dev(chan);
	dma_addr_t dma;
	int i;

	BUILD_BUG_ON(sizeof(page->hw) != PAGE_SIZE);

	dma = dma_map_single(dev, page->hw, sizeof(page->hw), DMA_TO_DEVICE);

	for (i = 0; i < IOAT_DESCS_PER_PAGE; i++) {
		struct ioat2_ring_ent *desc = &page->sw[i];

		dma_async_tx_descriptor_init(&desc->txd, c);
		desc->txd.tx_submit = ioat2_tx_submit_unlock;
		desc->txd.phys = dma;
		dma += sizeof(page->hw[0]);
	}
}

static void ioat2_free_page(struct ioat2_dma_chan *ioat, struct ioat2_ring_page *page)
{
	struct ioat_chan_common *chan = &ioat->base;
	struct device *dev = to_dev(chan);
	struct ioat2_ring_ent *desc;

	if (!page)
		return;

	desc = &page->sw[0];
	dma_unmap_single(dev, desc->txd.phys, sizeof(page->hw), DMA_TO_DEVICE);
	kfree(page);
}

static int ioat2_init_ring(struct ioat2_dma_chan *ioat)
{
	struct ioat_chan_common *chan = &ioat->base;
	struct ioat2_ring_page *page;
	struct ioat2_ring_dir *dir;
	int descs = 1 << ioat_ring_alloc_order;
	int i, p;

	dir = kzalloc(sizeof(*dir), GFP_KERNEL);
	if (!dir)
		return -ENOMEM;
	for (p = 0; p < descs / IOAT_DESCS_PER_PAGE; p++) {
		page = kzalloc(sizeof(*page), GFP_KERNEL);
		if (!page) {
			while (p--)
				ioat2_free_page(ioat, dir->page[p]);
			kfree(dir);
			return -ENOMEM;
		}
		ioat2_init_page(ioat, page);
		dir->page[p] = page;
	}

	spin_lock_bh(&chan->cleanup_lock);
	spin_lock_bh(&ioat->prep_lock);
	ioat->dir[0] = dir;
	ioat2_set_head(ioat, 0);
	ioat2_set_issued(ioat, 0);
	ioat2_set_tail(ioat, 0);
	ioat->alloc_order = ioat_ring_alloc_order;

	/* link descs */
	for (i = 0; i < descs; i++) {
		struct ioat2_ring_ent *next = ioat2_get_ring_ent(ioat, i+1);
		struct ioat2_ring_ent *desc = ioat2_get_ring_ent(ioat, i);
		struct ioat_dma_descriptor *hw = to_hw(desc);

		hw->next = next->txd.phys;
	}
	spin_unlock_bh(&ioat->prep_lock);
	spin_unlock_bh(&chan->cleanup_lock);

	return 0;
}

/* ioat2_alloc_chan_resources - allocate/initialize ioat2 descriptor ring
 * @chan: channel to be initialized
 */
int ioat2_alloc_chan_resources(struct dma_chan *c)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);
	struct ioat_chan_common *chan = &ioat->base;
	int err;

	/* have we already been set up? */
	if (ioat->dir[0])
		return 1 << ioat->alloc_order;

	/* Setup register to interrupt and write completion status on error */
	writew(IOAT_CHANCTRL_RUN, chan->reg_base + IOAT_CHANCTRL_OFFSET);

	/* allocate a completion writeback area */
	/* doing 2 32bit writes to mmio since 1 64b write doesn't work */
	chan->completion = pci_pool_alloc(chan->device->completion_pool,
					  GFP_KERNEL, &chan->completion_dma);
	if (!chan->completion)
		return -ENOMEM;

	memset(chan->completion, 0, sizeof(*chan->completion));
	writel(((u64) chan->completion_dma) & 0x00000000FFFFFFFF,
	       chan->reg_base + IOAT_CHANCMP_OFFSET_LOW);
	writel(((u64) chan->completion_dma) >> 32,
	       chan->reg_base + IOAT_CHANCMP_OFFSET_HIGH);

	err = ioat2_init_ring(ioat);
	if (err)
		return err;

	tasklet_enable(&chan->cleanup_task);
	ioat2_start_null_desc(ioat);

	return 1 << ioat->alloc_order;
}

bool reshape_ring(struct ioat2_dma_chan *ioat, int order)
{
	/* reshape differs from normal ring allocation in that we want
	 * to allocate a new software ring while only
	 * extending/truncating the hardware ring
	 */
	struct ioat_chan_common *chan = &ioat->base;
	const u16 curr_size = ioat2_ring_size(ioat);
	const u16 active = ioat2_ring_active(ioat);
	const u16 new_size = 1 << order;
	struct ioat2_ring_ent *next_ent, *curr_ent;
	struct ioat_dma_descriptor *hw;
	struct ioat2_ring_dir *dir;
	int old_order = ioat->alloc_order;
	struct ioat2_ring_page *page;
	u16 pending;
	u16 i;

	if (order >= ioat_get_max_alloc_order())
		return false;

	/* double check that we have at least 1 free descriptor */
	if (active == curr_size)
		return false;

	/* when shrinking, verify that we can hold the current active
	 * set in the new ring
	 */
	if (active >= new_size)
		return false;

	pending = ioat2_ring_pending(ioat);
	ioat->alloc_order = order;
	dir = ioat->dir[0];

	/* we need to allocate additional pages if we are expanding */
	if (new_size > curr_size) {
		u16 total_pages = ioat2_page_index(new_size);
		u16 curr_pages = ioat2_page_index(curr_size);
		u16 tail_page = ioat2_page_index(ioat->tail);

		if (ioat->head < ioat->tail) {
			struct ioat2_ring_dir *new_dir;

			new_dir = kzalloc(sizeof(*dir), GFP_NOWAIT);
			if (!new_dir) {
				ioat->alloc_order = old_order;
				return -ENOMEM;
			}

			/*
			 * we re-align old ring with tail not wrapped.
			 * we don't need to worry about head < tail and
			 * head+tail are on the same page because we
			 * make sure there are at least 64+1 descs free
			 */

			for (i = 0; i < curr_pages; i++) {
				u16 curr_idx =
					(tail_page + i) & (curr_pages - 1);
				u16 new_idx = i & (total_pages - 1);

				new_dir->page[new_idx] = dir->page[curr_idx];
			}

			/* free the old dir and linke the new dir */
			kfree(ioat->dir[0]);
			dir = ioat->dir[0] = new_dir;

			/* fixup ioat->tail and ioat->head*/
			ioat2_set_tail(ioat,
				       ioat->tail & (IOAT_DESCS_PER_PAGE - 1));
			ioat2_set_head(ioat, ioat->tail + active);
			ioat2_set_issued(ioat, ioat->head - pending);
		}

		for(i = curr_pages; i < total_pages; i++) {
			page = kzalloc(sizeof(*page), GFP_NOWAIT);
			if (!page) {
				while (i--)
					ioat2_free_page(ioat, dir->page[i]);
				ioat->alloc_order = old_order;
				return -ENOMEM;
			}

			ioat2_init_page(ioat, page);
			dir->page[i] = page;
		}

		/* link descs */
		for (i = (curr_size - 1); i < new_size; i++) {
			next_ent = ioat2_get_ring_ent(ioat, i+1);
			curr_ent = ioat2_get_ring_ent(ioat, i);
			hw = to_hw(curr_ent);
			hw->next = next_ent->txd.phys;
		}
	} else { /* shrinking the ring */
		int start_pg, end_pg;
		u16 head_page = ioat2_page_index(ioat->head);

		/* at this point, DMA is idle. head = tail = issued */

		start_pg = ioat2_page_index(new_size);
		end_pg = ioat2_page_index(curr_size);

		if (head_page >= start_pg) {
			/* we save the page we want to flip */
			page = dir->page[0];

			/* we reassign the page where head is to the "last" page */
			dir->page[0] = dir->page[head_page];

			/* and we swap the original "last" page */
			dir->page[head_page] = page;

			/* fixup the last entry of flipped page */
			curr_ent = ioat2_get_ring_ent(ioat, IOAT_DESCS_PER_PAGE - 1);
			next_ent = ioat2_get_ring_ent(ioat, IOAT_DESCS_PER_PAGE);
			hw = to_hw(curr_ent);
			hw->next = next_ent->txd.phys;

			/* fixup ioat->tail and ioat->head*/
			ioat2_set_head(ioat, ioat->head & (IOAT_DESCS_PER_PAGE - 1));
			ioat2_set_tail(ioat, ioat->head);
			ioat2_set_issued(ioat, ioat->head);
		}

		/* fixup the last entry */
		curr_ent = ioat2_get_ring_ent(ioat, new_size - 1);
		next_ent = ioat2_get_ring_ent(ioat, 0);
		hw = to_hw(curr_ent);
		hw->next = next_ent->txd.phys;

		for (i = start_pg; i < end_pg; i++)
			ioat2_free_page(ioat, dir->page[i]);
	}

	dev_dbg(to_dev(chan), "%s: resized to %d descriptors\n",
		__func__, new_size);

	return true;
}

/**
 * ioat2_check_space_lock - verify space and grab ring producer lock
 * @ioat: ioat2,3 channel (ring) to operate on
 * @num_descs: allocation length
 */
int ioat2_check_space_lock(struct ioat2_dma_chan *ioat, int num_descs)
{
	struct ioat_chan_common *chan = &ioat->base;
	bool retry;

 retry:
	spin_lock_bh(&ioat->prep_lock);
	/* never allow the last descriptor to be consumed, we need at
	 * least one free at all times to allow for on-the-fly ring
	 * resizing.
	 */
	if (likely(ioat2_ring_space(ioat) > num_descs + 64)) {
		dev_dbg(to_dev(chan), "%s: num_descs: %d (%x:%x:%x)\n",
			__func__, num_descs, ioat->head, ioat->tail, ioat->issued);
		ioat->produce = num_descs;
		return 0;  /* with ioat->prep_lock held */
	}
	retry = test_and_set_bit(IOAT_RESHAPE_PENDING, &chan->state);
	spin_unlock_bh(&ioat->prep_lock);

	/* is another cpu already trying to expand the ring? */
	if (retry)
		goto retry;

	spin_lock_bh(&chan->cleanup_lock);
	spin_lock_bh(&ioat->prep_lock);
	retry = reshape_ring(ioat, ioat->alloc_order + 1);
	clear_bit(IOAT_RESHAPE_PENDING, &chan->state);
	spin_unlock_bh(&ioat->prep_lock);
	spin_unlock_bh(&chan->cleanup_lock);

	/* if we were able to expand the ring retry the allocation */
	if (retry)
		goto retry;

	if (printk_ratelimit())
		dev_dbg(to_dev(chan), "%s: ring full! num_descs: %d (%x:%x:%x)\n",
			__func__, num_descs, ioat->head, ioat->tail, ioat->issued);

	/* progress reclaim in the allocation failure case we may be
	 * called under bh_disabled so we need to trigger the timer
	 * event directly
	 */
	if (jiffies > chan->timer.expires && timer_pending(&chan->timer)) {
		struct ioatdma_device *device = chan->device;

		mod_timer(&chan->timer, jiffies + COMPLETION_TIMEOUT);
		device->timer_fn((unsigned long) &chan->common);
	}

	return -ENOMEM;
}

struct dma_async_tx_descriptor *
ioat2_dma_prep_memcpy_lock(struct dma_chan *c, dma_addr_t dma_dest,
			   dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);
	struct ioat_dma_descriptor *hw;
	struct ioat2_ring_ent *desc;
	dma_addr_t dst = dma_dest;
	dma_addr_t src = dma_src;
	size_t total_len = len;
	int num_descs, idx, i;

	num_descs = ioat2_xferlen_to_descs(ioat, len);
	if (likely(num_descs) && ioat2_check_space_lock(ioat, num_descs) == 0)
		idx = ioat->head;
	else
		return NULL;
	i = 0;
	do {
		size_t copy = min_t(size_t, len, 1 << ioat->xfercap_log);

		desc = ioat2_get_ring_ent(ioat, idx + i);
		hw = to_hw(desc);

		hw->size = copy;
		hw->ctl = 0;
		hw->src_addr = src;
		hw->dst_addr = dst;

		len -= copy;
		dst += copy;
		src += copy;
		__dump_desc_dbg(&ioat->base, to_hw(desc), &desc->txd);
	} while (++i < num_descs);

	desc->txd.flags = flags;
	desc->len = total_len;
	hw->ctl_f.int_en = !!(flags & DMA_PREP_INTERRUPT);
	hw->ctl_f.fence = !!(flags & DMA_PREP_FENCE);
	hw->ctl_f.compl_write = 1;
	__dump_desc_dbg(&ioat->base, to_hw(desc), &desc->txd);
	/* we leave the channel locked to ensure in order submission */

	return &desc->txd;
}

/**
 * ioat2_free_chan_resources - release all the descriptors
 * @chan: the channel to be cleaned
 */
void ioat2_free_chan_resources(struct dma_chan *c)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);
	struct ioat_chan_common *chan = &ioat->base;
	struct ioatdma_device *device = chan->device;
	struct ioat2_ring_ent *desc;
	const u16 total_descs = 1 << ioat->alloc_order;
	int descs;
	int i, p;

	/* Before freeing channel resources first check
	 * if they have been previously allocated for this channel.
	 */
	if (!ioat->dir[0])
		return;

	tasklet_disable(&chan->cleanup_task);
	del_timer_sync(&chan->timer);
	device->cleanup_fn((unsigned long) c);
	device->reset_hw(chan);

	spin_lock_bh(&chan->cleanup_lock);
	spin_lock_bh(&ioat->prep_lock);
	descs = ioat2_ring_space(ioat);

	dev_dbg(to_dev(chan), "freeing %d idle descriptors\n", descs);
	if (descs < total_descs) {
		dev_err(to_dev(chan), "Freeing %d in use descriptors!\n",
			total_descs - descs);
		for (i = 0; i < total_descs - descs; i++) {
			desc = ioat2_get_ring_ent(ioat, ioat->tail + i);
			__dump_desc_dbg(&ioat->base, to_hw(desc), &desc->txd);
		}
	}

	for (i = 0; i < ARRAY_SIZE(ioat->dir); i++) {
		struct ioat2_ring_dir *dir = ioat->dir[i];

		if (!dir)
			continue;
		for (p = 0; p < ARRAY_SIZE(dir->page); p++)
			ioat2_free_page(ioat, dir->page[p]);
		kfree(dir);
		ioat->dir[i] = NULL;
	}

	ioat->alloc_order = 0;
	pci_pool_free(device->completion_pool, chan->completion,
		      chan->completion_dma);
	spin_unlock_bh(&ioat->prep_lock);
	spin_unlock_bh(&chan->cleanup_lock);

	chan->last_completion = 0;
	chan->completion_dma = 0;
	ioat->dmacount = 0;
}

static ssize_t ring_size_show(struct dma_chan *c, char *page)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);

	return sprintf(page, "%d\n", (1 << ioat->alloc_order) & ~1);
}
static struct ioat_sysfs_entry ring_size_attr = __ATTR_RO(ring_size);

static ssize_t ring_active_show(struct dma_chan *c, char *page)
{
	struct ioat2_dma_chan *ioat = to_ioat2_chan(c);

	/* ...taken outside the lock, no need to be precise */
	return sprintf(page, "%d\n", ioat2_ring_active(ioat));
}
static struct ioat_sysfs_entry ring_active_attr = __ATTR_RO(ring_active);

static struct attribute *ioat2_attrs[] = {
	&ring_size_attr.attr,
	&ring_active_attr.attr,
	&ioat_cap_attr.attr,
	&ioat_version_attr.attr,
	NULL,
};

struct kobj_type ioat2_ktype = {
	.sysfs_ops = &ioat_sysfs_ops,
	.default_attrs = ioat2_attrs,
};

int __devinit ioat2_dma_probe(struct ioatdma_device *device, int dca)
{
	struct pci_dev *pdev = device->pdev;
	struct dma_device *dma;
	struct dma_chan *c;
	struct ioat_chan_common *chan;
	int err;

	/* ring allocation never to exceed an order-1 allocation */
	BUILD_BUG_ON(sizeof(struct ioat2_ring_dir) > PAGE_SIZE);
	BUILD_BUG_ON(sizeof(struct ioat2_ring_page) > PAGE_SIZE << 1);

	device->enumerate_channels = ioat2_enumerate_channels;
	device->reset_hw = ioat2_reset_hw;
	device->cleanup_fn = ioat2_cleanup_event;
	device->timer_fn = ioat2_timer_event;
	device->self_test = ioat_dma_self_test;
	dma = &device->common;
	dma->device_prep_dma_memcpy = ioat2_dma_prep_memcpy_lock;
	dma->device_issue_pending = ioat2_issue_pending;
	dma->device_alloc_chan_resources = ioat2_alloc_chan_resources;
	dma->device_free_chan_resources = ioat2_free_chan_resources;
	dma->device_is_tx_complete = ioat_is_dma_complete;

	err = ioat_probe(device);
	if (err)
		return err;
	ioat_set_tcp_copy_break(2048);

	list_for_each_entry(c, &dma->channels, device_node) {
		chan = to_chan_common(c);
		writel(IOAT_DCACTRL_CMPL_WRITE_ENABLE | IOAT_DMA_DCA_ANY_CPU,
		       chan->reg_base + IOAT_DCACTRL_OFFSET);
	}

	err = ioat_register(device);
	if (err)
		return err;

	ioat_kobject_add(device, &ioat2_ktype);

	if (dca)
		device->dca = ioat2_dca_init(pdev, device->reg_base);

	return err;
}
