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
 * bbu_blk_state - state of the data in a bbu cache block
 * @BBU_unassociated - free block
 * @BBU_replace_lock - overwriting a BBU_allocated block
 * @BBU_read_lock - inflight data from backing device
 * @BBU_sync - cache matches disk
 * @BBU_update_lock - merging new data into a BBU_sync block
 * @BBU_dirty - cache has latest data
 * @BBU_writeback_lock - dirty data inflight to disk
 */
enum bbu_blk_state {
	BBU_unassociated = 0,
	BBU_replace_lock = 1,
	BBU_read_lock = 2,
	BBU_sync = 3,
	BBU_update_lock = 4,
	BBU_dirty = 5,
	BBU_writeback_lock = 6,
};

static inline char *strstate(enum bbu_blk_state state)
{
	static char *str[] = { "unassociated", "replace", "read",
			       "sync", "update", "dirty", "writeback" };

	if (state >= 0 && state <= 6)
		return str[state];
	else
		return "invalid";
}

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
	struct list_head handle;
	struct bio *dirty_merge_bios;
	wait_queue_head_t wait_for_ent;
	wait_queue_head_t wait_for_overlap;
	wait_queue_head_t wait_for_writeback;
	int total_ents;
	int inactive_blocked; /* allow batch freeing of active ents */
	int barrier_active;
	spinlock_t cache_lock;
	unsigned long data_pfn; /* first page of data */
	u64 __iomem *desc; /* page_address(region->start_pfn) */
	enum bbu_cache_state state;
	struct list_head node;
	struct bbu_device *parent; /* platform device containing ADR resource */
	struct bbu_cache_dev *dev; /* sysfs device for this cache */
	struct block_device *bd; /* backing block device inode */
	int region_idx;
	atomic_t active; /* number of non-idle cache_ents */
	atomic_t dirty; /* number of dirty ents (idle or otherwise) */
	atomic_t writeback_active; /* number of inflight evictions */
	atomic_t active_bypass; /* in-flight cache bypass reads */
	int requesters; /* number of threads active in bbu_make_request */
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

/**
 * bbu_io_ent - infrastructure to manage reading/writing a cache block
 */
struct bbu_io_ent {
	struct bio *req;
	unsigned long pfn;
	enum bbu_blk_state state;
	struct bio *toread, *read, *towrite, *written;
	#define BLK_F_OVERWRITE 0
	#define BLK_F_DIRTY 1
	#define BLK_F_UPTODATE 2
	#define BLK_F_LOCKED 3
	#define BLK_F_ReadError 4
	#define BLK_F_Overlap 5
	#define BLK_F_Wantfill 6
	#define BLK_F_Wantdrain 7
	#define BLK_F_Wantread 8
	#define BLK_F_Wantwrite 9
	unsigned long flags;
};

/**
 * bbu_cache_ent - live
 */
struct bbu_cache_ent {
	struct hlist_node hash;
	struct list_head lru;
	unsigned long state;
	#define BBU_ENT_DIRTY 0 /* ent dirty */
	#define BBU_ENT_HANDLE 1 /* ent needs work before going inactive */
	#define BBU_ENT_WRITEBACK 2 /* writeback in flight */
	spinlock_t lock;
	atomic_t count;
	sector_t sector;
	struct bbu_cache_conf *conf;
	struct bbu_io_ent blk[1];
};

enum bbu_get_flags {
	BBU_GET_F_RECYCLE_OK = 1 << 0,
	BBU_GET_F_BLOCK_OK = 1 << 1,
};

static bool is_blk_active(struct bbu_io_ent *blk)
{
	switch (blk->state) {
	case BBU_replace_lock:
	case BBU_read_lock:
	case BBU_update_lock:
	case BBU_dirty:
		return true;
	default:
		return false;
	}
}

static inline struct bbu_region *bbu_conf_to_region(struct bbu_cache_conf *conf)
{
	return &conf->parent->region[conf->region_idx];
}

static inline unsigned long bbu_conf_to_blks(struct bbu_cache_conf *conf)
{
	struct bbu_region *region = bbu_conf_to_region(conf);

	return bbu_region_to_blks(region);
}

static inline int blk_shift(struct bbu_cache_conf *conf)
{
	return PAGE_SHIFT + conf->blk_order - 9;
}

static inline sector_t blk_sectors(struct bbu_cache_conf *conf)
{
	return 1ULL << blk_shift(conf);
}

/* the number of active + dirty ents must fall below this number when
 * evicting used ents due to cache pressure
 */
static inline int bbu_watermark(struct bbu_cache_conf *conf)
{
	return conf->total_ents * 3/4 | 1;
}

static inline sector_t blk_to_sector(struct bbu_cache_ent *ent, int blk_idx)
{
	struct bbu_cache_conf *conf = ent->conf;
	sector_t sector = ent->sector + conf->stripe_sectors * blk_idx;

	return sector;
}

static inline sector_t bbu_desc_to_sector(struct bbu_cache_conf *conf, u64 desc)
{
	return (desc >> blk_shift(conf)) << blk_shift(conf);
}

static inline enum bbu_blk_state bbu_desc_to_state(u64 desc)
{
	return desc & 0x7;
}

static inline struct hlist_head *bbu_hash(struct bbu_cache_conf *conf, sector_t sector)
{
	return &conf->hashtbl[(sector >> blk_shift(conf)) & HASH_MASK];
}

static inline struct device *conf_to_dev(struct bbu_cache_conf *conf)
{
	return &conf->parent->pdev->dev;
}

static inline void bbu_set_queuedata(struct request_queue *q, void *data)
{
	*((void **) q->queuedata) = data;
}

static inline void *bbu_get_queuedata(struct request_queue *q)
{
	return *((void **) q->queuedata);
}

static inline u64 read_desc(struct bbu_cache_conf *conf, struct bbu_io_ent *blk)
{
	return readq(conf->desc + (blk->pfn - conf->data_pfn));
}

static inline void write_desc(u64 v, struct bbu_cache_conf *conf, struct bbu_io_ent *blk)
{
	writeq(v, conf->desc + (blk->pfn - conf->data_pfn));
}
