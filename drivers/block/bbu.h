/*
 * battery backed block-device cache
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
#include <linux/list.h>
#include <linux/bio.h>
#include <linux/types.h>
#include <linux/blkdev.h>
#include <linux/workqueue.h>

#define BBU_MAGIC 0x03bcbbbc
#define NR_HASH (PAGE_SIZE / sizeof(struct hlist_head))
#define HASH_MASK (NR_HASH - 1)
#define BBU_INVALID_PFN (-1ULL)

/**
 * bbu_region - describe a range of adr pages
 * @magic - magic identifier for a bbu region
 * @size - size in megabytes of the adr region (must be an even number)
 * @start_pfn - starting location in adr memory for this region
 * @name - handle for this region in the /sys/class/cache namespace
 * @uuid - for associating a block device to this bbu region
 * @blk_order - 2^blk_order pages per block
 * @pad - for future expansion
 * @reserved - for future expansion
 * @cheksum - sum of preceeding parameters
 *
 * Note: The first page of adr memory contains an array of bbu_region
 * descriptors, given each desriptor is 64-bytes we can describe a total
 * of 64 regions.
 */
struct bbu_region {
	u32 magic;
	u32 size;
	u64 start_pfn;
	char name[16];
	char uuid[16];
	u8 blk_order;
	u8 pad[3];
	u32 reserved[2];
	u32 checksum;
};
#define BBU_MAX_REGIONS (PAGE_SIZE / sizeof(struct bbu_region))
#define BBU_REGION_WORDS (sizeof(struct bbu_region) / sizeof(u32) - 1)
static inline unsigned long bbu_region_to_data_pages(struct bbu_region *region)
{
	/* megabytes to pages */
	return region->size << (20 - PAGE_SHIFT);
}

static inline unsigned long bbu_region_to_blks(struct bbu_region *region)
{
	return region->size << (20 - (PAGE_SHIFT + region->blk_order));
}

/* number of pages reserved for cache descriptors */
static inline unsigned long bbu_region_to_desc_pages(struct bbu_region *region)
{
	/* megabytes to blocks */
	size_t size;

	size = ALIGN(bbu_region_to_blks(region) * sizeof(u64), PAGE_SIZE);
	return size >> PAGE_SHIFT;
}

static inline unsigned long bbu_region_to_pages(struct bbu_region *region)
{
	return bbu_region_to_data_pages(region) +
	       bbu_region_to_desc_pages(region);
}

struct bbu_device {
	struct list_head node;
	struct platform_device *pdev;
	unsigned long start_pfn;
	void __iomem *ctrl;
	int num_pages;
	struct bbu_region *region;
	struct list_head caches;
	u64 failed_mask;
};


static inline int __first_failed(const struct bbu_device *bdev)
{
	unsigned long *mask = (unsigned long *) &bdev->failed_mask;

	return min_t(int, 64, find_first_bit(mask, 64));
}

static inline int __next_failed(int n, const struct bbu_device *bdev)
{
	unsigned long *mask = (unsigned long *) &bdev->failed_mask;

	return min_t(int, 64, find_next_bit(mask, 64, n+1));
}
#define for_each_failed_region(idx, bdev)   \
	for ((idx) = __first_failed(bdev); \
	     (idx) < 64;		    \
	     (idx) = __next_failed((idx), (bdev)))

/**
 * bbu_cache_dev - class device for bbu_cache_conf objects
 */
struct bbu_cache_dev {
	struct device device;
	struct bbu_cache_conf *conf;
	struct work_struct del_work;
};

enum bbu_cache_state {
	BBU_inactive,
	BBU_active,
	BBU_failed,
};

/**
 * bbu_cache_conf - live configuration data for a bbu region
 */
struct bbu_cache_conf {
	struct hlist_head *hashtbl;
	struct list_head inactive;
	struct list_head inactive_dirty;
	wait_queue_head_t wait_for_ent;
	int total_ents;
	int inactive_blocked; /* allow batch freeing of active ents */
	spinlock_t cache_lock;
	unsigned long desc_pfn; /* first page of cache entry descriptors */
	u64 __iomem *desc; /* page_address(desc_pfn) */
	enum bbu_cache_state state;
	struct list_head node;
	struct bbu_device *parent; /* platform device containing ADR resource */
	struct bbu_cache_dev *dev; /* sysfs device for this cache */
	int region_idx;
	atomic_t active; /* number of non-idle cache_ents */
	atomic_t dirty; /* number of dirty ents (idle or otherwise) */
	atomic_t writeback_active; /* number of inflight evictions */
	int blk_order; /* 1 << blk_order pages per cache block (bbu_io_ent) */
	int stripe_members; /* is_striped(bdev) ? num members : 1 */
	unsigned long stripe_sectors; /* is_striped(bdev) ? num sectors : 0 */
	struct request_queue *queue; /* backing device request_queue */
	make_request_fn *make_request; /* backing device make_request_fn */
	char name[16+5]; /* 16-char name + 'bbu/' prefix + null */
	struct kmem_cache *mem_cache; /* for allocating bbu_cache_ents */
	struct bbu_init_lists {
		struct list_head complete;
		struct list_head complete_dirty;
		struct list_head partial;
	} *init;
};

struct bbu_work {
	struct bbu_cache_conf *conf;
	struct work_struct work;
};

static inline struct bbu_region *bbu_conf_to_region(struct bbu_cache_conf *conf)
{
	return &conf->parent->region[conf->region_idx];
}

