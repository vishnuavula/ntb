/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2012 Intel Corporation. All rights reserved.
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
 *   BSD LICENSE
 *
 *   Copyright(c) 2012 Intel Corporation. All rights reserved.
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
 * Intel ADR backed RAM disk
 *
 */
#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/bio.h>
#include <linux/platform_device.h>
#include <linux/hdreg.h>

static sector_t hardsect_size = 4096/*512*/;
static atomic_t adrbd_idx;

struct adr_dev {
	struct platform_device *pdev;
	int dev_id;
	int adr_major;
	size_t size;
	sector_t sectors;
	void *adr_start;
	struct request_queue *queue;
	struct gendisk *gd;
};

#define ADEV_MINORS		16
#define MINOR_SHIFT		4
#define DEVNUM(kdevnum)	(MINOR(kdev_t_to_nr(kdevnum)) >> MINOR_SHIFT
#define KERNEL_SECTOR_SIZE	512
#define SECTOR_SHIFT		9

static void
adr_xfer(struct adr_dev *adev, unsigned long sector,
	 unsigned long nbytes, char *buffer, int write)
{
	unsigned long offset = sector * KERNEL_SECTOR_SIZE;

	if ((offset + nbytes) > adev->size) {
		dev_dbg(&adev->pdev->dev,
			"Beyond-end write (%ld %ld)\n", offset, nbytes);
		return;
	}

#ifdef DEBUG
#if 0	
	dev_dbg(&adev->pdev->dev,
		  "sector: %lu  nsect: %lu  offset: %lu  nbytes: %lu\n",
		  sector, nsect, offset, nbytes);
	dev_dbg(&adev->pdev->dev, "write %s buffer: %p\n",
		  (write ? "from" : "to"), buffer);
#endif
#endif
	if (write) {
		unsigned int tmp;

		__copy_from_user_inatomic_nocache(adev->adr_start + offset,
						  buffer, nbytes);

		/*
		 * we must flush (and invalidate) the cache because the
		 * memory is write back. And then we will "fence" the memory
		 * by reading a dword back to ensure it's there
		 */
		clflush_cache_range(adev->adr_start + offset, nbytes);
		tmp = *(unsigned int *)(adev->adr_start + offset + nbytes - sizeof(tmp));
	} else
		memcpy(buffer, adev->adr_start + offset, nbytes);
}

static void adr_make_request(struct request_queue *q, struct bio *bio)
{
	struct adr_dev *adev = q->queuedata;
	struct block_device *bdev = bio->bi_bdev;
	struct bio_vec *bvec;
	sector_t sector = bio->bi_sector;
	int i;
	int err = 0;

	if (sector + (bio->bi_size >> SECTOR_SHIFT) >
			get_capacity(bdev->bd_disk)) {
		err = -EIO;
		goto out;
	}

	bio_for_each_segment(bvec, bio, i) {
		unsigned int len = bvec->bv_len;
		char *buffer = __bio_kmap_atomic(bio, i, KM_USER0);
		adr_xfer(adev, sector, bvec->bv_len,
			 buffer, bio_data_dir(bio) == WRITE);
		sector += len >> SECTOR_SHIFT;
		__bio_kunmap_atomic(bio, KM_USER0);
	}

out:
	bio_endio(bio, err);
}

static int
adr_ioctl(struct block_device *bdev, fmode_t mode,
	  unsigned int cmd, unsigned long arg)
{
	if (cmd != BLKFLSBUF)
		return -ENOTTY;

	/* 
	 * we'll just say we did because we don't need to flush since we
	 * are flushing to DRAM every time we write anyhow
	 */
	return 0;
}

static struct block_device_operations adev_ops = {
	.owner		= THIS_MODULE,
	.ioctl		= adr_ioctl
};

static int adev_init(struct adr_dev *adev)
{
	/* do ADR initialization / verification? */

	adev->queue = blk_alloc_queue(GFP_KERNEL);
	if (!adev->queue)
		return -ENODEV;
	blk_queue_make_request(adev->queue, adr_make_request);
	blk_queue_max_hw_sectors(adev->queue, 1024);
	blk_queue_bounce_limit(adev->queue, BLK_BOUNCE_ANY);

	blk_queue_physical_block_size(adev->queue, hardsect_size);
	blk_queue_logical_block_size(adev->queue, hardsect_size);

	adev->queue->queuedata = adev;

	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, adev->queue);

	adev->gd = alloc_disk(ADEV_MINORS);
	if (!adev->gd) {
		dev_err(&adev->pdev->dev, "%s: alloc_disk failed\n", __func__);
		return -ENODEV;
	}

	adev->gd->major = adev->adr_major;
	adev->gd->first_minor = ADEV_MINORS * adev->dev_id;
	adev->gd->fops = &adev_ops;
	adev->gd->queue = adev->queue;
	adev->gd->private_data = adev;
	adev->gd->flags |= GENHD_FL_SUPPRESS_PARTITION_INFO;
	snprintf(adev->gd->disk_name, 32, "adrbd%u", adev->dev_id);
	set_capacity(adev->gd, adev->size >> 9);
	add_disk(adev->gd);

	return 0;
}

#if 0
static int set_pages_wc(struct page *page, int num_pages)
{
	unsigned long addr = (unsigned long) page_address(page);

	return set_memory_wc(addr, num_pages);
}
#endif

static int __devinit adrbd_probe(struct platform_device *pdev)
{

	struct adr_dev *adev;
	struct resource *res;
	struct device *dev = &pdev->dev;
	resource_size_t size;
	int err;
	unsigned long start_pfn, end_pfn, num_pages;

	adev = devm_kzalloc(&pdev->dev, sizeof(*adev), GFP_KERNEL);
	if (!adev) {
		dev_err(&pdev->dev, "Unable to allocate adev\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get platform resources\n");
		return -ENODEV;
	}

	size = resource_size(res);
	if (!devm_request_mem_region(dev, res->start, size, pdev->name)) {
		dev_err(&pdev->dev,
			"Failed to request mem region (%s) %#llx size %llu\n",
			pdev->name, res->start, size);
		return -EBUSY;
	}

	/* for consistency we don't want memory backed by CPU cache */
	start_pfn = res->start >> PAGE_SHIFT;
	end_pfn = (res->start + size - 1) >> PAGE_SHIFT;
	num_pages = end_pfn - start_pfn + 1;
	err = set_pages_wb(pfn_to_page(start_pfn), num_pages);
	if (err)
		return err;

	adev->adr_major = register_blkdev(0, "adrbd");
	if (adev->adr_major <= 0) {
		dev_warn(&pdev->dev, "Unable to acquire major number.\n");
		return -EBUSY;
	}

	adev->dev_id = atomic_inc_return(&adrbd_idx) - 1;
	adev->pdev = pdev;
	adev->size = size;
	adev->sectors = size / hardsect_size;
	adev->adr_start = page_address(pfn_to_page(start_pfn));
	platform_set_drvdata(pdev, adev);

	dev_dbg(&pdev->dev, "physical start: %#llx\n", res->start);
	dev_dbg(&pdev->dev, "size: %llu bytes\n", size);
	dev_dbg(&pdev->dev, "virt start: %p\n", adev->adr_start);

	err = adev_init(adev);
	if (err < 0) {
		dev_err(&pdev->dev, "ADRBD initialization failed\n");
		atomic_dec(&adrbd_idx);
		unregister_blkdev(adev->adr_major, "adrbd");
		return err;
	}

	dev_info(&pdev->dev, "ADRBD.%d (%d.%d) registered: %lu bytes.\n",
		 adev->dev_id, adev->adr_major,
		 adev->gd->first_minor, adev->size);

	return 0;
}

static int __devexit adrbd_remove(struct platform_device *pdev)
{
	struct adr_dev *adev = platform_get_drvdata(pdev);

	if (adev->gd) {
		del_gendisk(adev->gd);
		put_disk(adev->gd);
	}

	if (adev->queue)
		blk_put_queue(adev->queue);

	unregister_blkdev(adev->adr_major, "adrbd");
	atomic_dec(&adrbd_idx);
	kfree(adev);

	return 0;
}

static struct platform_driver adrbd_driver = {
	.probe		= adrbd_probe,
	.remove		= __devexit_p(adrbd_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "adr",
	},
};

static int __init adr_init(void)
{
	int res;

	atomic_set(&adrbd_idx, 0);
	res = platform_driver_register(&adrbd_driver);

	return res;
}
subsys_initcall(adr_init);

static void adr_exit(void)
{
	platform_driver_unregister(&adrbd_driver);
}
module_exit(adr_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("ADR backed RAM disk");
MODULE_AUTHOR("Intel Corporation");
