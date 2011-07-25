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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/sort.h>
#include <linux/io.h>

#include <asm/cacheflush.h>

#include "bbu.h"

/* protects bbu_device_list, bbu_region configuration, and bbu_cache_dev
 * to bbu_cache_conf conversions
 */
static DEFINE_MUTEX(bbu_lock);

/* each bbu_device manages a set of bbu_region children */
static LIST_HEAD(bbu_device_list);

static int set_pages_wc(struct page *page, int num_pages)
{
	unsigned long addr = (unsigned long) page_address(page);

	return set_memory_wc(addr, num_pages);
}

static u32 calc_checksum(struct bbu_region *region)
{
	u32 checksum = 0;
	int i;

	for (i = 0; i < BBU_REGION_WORDS; i++)
		checksum += ((u32 *) region)[i];

	return checksum;
}

static struct bbu_region *validate_region(struct bbu_region *region)
{
	if (region->magic != BBU_MAGIC)
		return NULL;

	if (region->checksum != calc_checksum(region))
		return NULL;

	return region;
}

static struct bbu_cache_conf *
alloc_add_cache_conf(struct bbu_device *bdev, int idx)
{
	struct device *dev = &bdev->pdev->dev;
	struct bbu_region *region = &bdev->region[idx];
	unsigned long desc_pages = bbu_region_to_desc_pages(region);
	struct bbu_cache_conf *conf;

	conf = devm_kzalloc(dev, sizeof(*conf), GFP_KERNEL);
	if (conf) {
		conf->hashtbl = devm_kzalloc(dev, PAGE_SIZE, GFP_KERNEL);
		if (!conf->hashtbl) {
			devm_kfree(dev, conf);
			conf = NULL;
		}
	}

	if (!conf)
		dev_err(dev, "bbu/%.16s: failed to allocate resources\n",
			region->name);
	else {
		INIT_LIST_HEAD(&conf->inactive);
		INIT_LIST_HEAD(&conf->inactive_dirty);
		init_waitqueue_head(&conf->wait_for_ent);
		spin_lock_init(&conf->cache_lock);
		INIT_LIST_HEAD(&conf->node);
		conf->state = BBU_inactive;
		conf->parent = bdev;
		conf->region_idx = idx;
		atomic_set(&conf->active, 0);
		atomic_set(&conf->dirty, 0);
		atomic_set(&conf->writeback_active, 0);
		conf->desc_pfn = region->start_pfn;
		conf->desc = page_address(pfn_to_page(conf->desc_pfn));
		conf->blk_order = region->blk_order;
		snprintf(conf->name, sizeof(conf->name), "bbu/%.16s",
			 region->name);
		list_add(&conf->node, &bdev->caches);
		dev_info(dev, "%s: %dMB @ %llx\n",
			 conf->name, region->size,
			 (region->start_pfn + desc_pages) << PAGE_SHIFT);
	}

	return conf;
}

static void __devexit __bbu_remove(struct bbu_device *bdev, struct bbu_cache_conf *conf)
{
	/* unregister, unlink cache_dev */
	if (conf->dev) {
		device_unregister(&conf->dev->device);
		conf->dev->conf = NULL;
		conf->dev = NULL;
	}
}

static int __devexit bbu_remove(struct platform_device *dev)
{
	struct bbu_device *bdev, *b;
	struct bbu_cache_conf *conf, *c;

	mutex_lock(&bbu_lock);
	list_for_each_entry_safe(bdev, b, &bbu_device_list, node) {
		list_for_each_entry_safe(conf, c, &bdev->caches, node) {
			__bbu_remove(bdev, conf);
			list_del(&conf->node);
		}
		list_del(&bdev->node);
	}
	mutex_unlock(&bbu_lock);

	return 0;
}

static ssize_t
state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;

	conf = cdev->conf;
	if (conf) {
		switch (conf->state) {
		case BBU_inactive:
			len += snprintf(buf, PAGE_SIZE, "inactive\n");
			break;
		case BBU_active:
			len += snprintf(buf, PAGE_SIZE, "active\n");
			break;
		case BBU_failed:
			len += snprintf(buf, PAGE_SIZE, "failed\n");
			break;
		}
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	bool delete = sysfs_streq(buf, "delete");
	int err;

	if (!delete)
		return -EINVAL;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;

	conf = cdev->conf;
	if (conf && conf->state == BBU_inactive) {
		struct bbu_region *region = bbu_conf_to_region(conf);
		struct bbu_device *bdev = conf->parent;
		struct device *dev = &bdev->pdev->dev;

		cdev->conf = NULL;
		conf->dev = NULL;
		list_del(&conf->node);

		region->magic = 0;

		devm_kfree(dev, conf);
		schedule_work(&cdev->del_work);
		dev_info(dev, "%s: removed\n", conf->name);
		err = 0;
	} else if (conf && conf->state != BBU_inactive)
		err = -EBUSY;
	mutex_unlock(&bbu_lock);

	return err ? err : cnt;
}

static ssize_t
size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		struct bbu_region *region = bbu_conf_to_region(conf);

		len += snprintf(buf, PAGE_SIZE, "%d\n", region->size);
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
meta_pfn_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		struct bbu_region *region = bbu_conf_to_region(conf);

		len += snprintf(buf, PAGE_SIZE, "%#llx\n", region->start_pfn);
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
uuid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		struct bbu_region *region = bbu_conf_to_region(conf);

		uuid_to_string(buf, region->uuid, 1);
		err = strlen(buf);
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
uuid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int uuid[4];
	int err = parse_uuid(uuid, buf);

	if (err)
		return err;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	conf = cdev->conf;
	if (conf && conf->state == BBU_inactive) {
		struct bbu_region *region = bbu_conf_to_region(conf);

		memcpy(region->uuid, uuid, sizeof(uuid));
		region->checksum = calc_checksum(region);
	} else if (conf && conf->state != BBU_inactive)
		err = -EBUSY;
	else
		err = -ENODEV;
	mutex_unlock(&bbu_lock);

	return err ? err : cnt;
}

static ssize_t
order_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		struct bbu_region *region = bbu_conf_to_region(conf);

		len += snprintf(buf, PAGE_SIZE, "%d\n", region->blk_order);
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		len += snprintf(buf, PAGE_SIZE, "%d\n",
				atomic_read(&conf->active));
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
dirty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		len += snprintf(buf, PAGE_SIZE, "%d\n",
				atomic_read(&conf->dirty));
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
writeback_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		len += snprintf(buf, PAGE_SIZE, "%d\n",
				atomic_read(&conf->writeback_active));
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static ssize_t
entry_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);
	struct bbu_cache_conf *conf;
	int err;
	size_t len = 0;

	err = mutex_lock_interruptible(&bbu_lock);
	if (err)
		return err;
	err = -ENODEV;
	conf = cdev->conf;
	if (conf) {
		len += snprintf(buf, PAGE_SIZE, "%d\n", conf->total_ents);
		err = len;
	}
	mutex_unlock(&bbu_lock);

	return err;
}

static struct device_attribute bbu_attrs[] = {
	__ATTR(state, S_IRUGO|S_IWUSR, state_show, state_store),
	__ATTR(size, S_IRUGO, size_show, NULL),
	__ATTR(meta_pfn, S_IRUGO, meta_pfn_show, NULL),
	__ATTR(uuid, S_IRUGO|S_IWUSR, uuid_show, uuid_store),
	__ATTR(order, S_IRUGO, order_show, NULL),
	__ATTR(active, S_IRUGO, active_show, NULL),
	__ATTR(dirty, S_IRUGO, dirty_show, NULL),
	__ATTR(writeback, S_IRUGO, writeback_show, NULL),
	__ATTR(entry_count, S_IRUGO, entry_count_show, NULL),
	__ATTR_NULL
};

static void bbu_dev_release(struct device *dev)
{
	struct bbu_cache_dev *cdev = container_of(dev, typeof(*cdev), device);

	kfree(cdev);
}

static struct class bbu_class = {
	.name		= "bbu",
	.dev_attrs	= bbu_attrs,
	.dev_release	= bbu_dev_release,
};

static void del_cache_dev(struct work_struct *work)
{
	struct bbu_cache_dev *cdev = container_of(work, typeof(*cdev), del_work);

	device_unregister(&cdev->device);
}

static int register_cache(struct bbu_device *bdev, struct bbu_cache_conf *conf)
{
	struct bbu_region *region = bbu_conf_to_region(conf);
	struct device *dev = &bdev->pdev->dev;
	struct bbu_cache_dev *cdev;
	int err = -ENOMEM;

	/* can't use devm_kzalloc here as cdev might out live bdev and conf */
	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (cdev) {
		cdev->device.class = &bbu_class;
		cdev->device.parent = &bdev->pdev->dev;
		INIT_WORK(&cdev->del_work, del_cache_dev);
		err = dev_set_name(&cdev->device, "%.16s", region->name);
		if (!err) {
			err = device_register(&cdev->device);
			if (!err) {
				conf->dev = cdev;
				cdev->conf = conf;
			}
		}

		if (err)
			kfree(cdev);
	}

	if (err)
		dev_warn(dev, "%s: sysfs registration failed: %d\n",
			 conf->name, err);
	return err;
}

static void bbu_mark_failed_region(struct bbu_device *bdev, int idx)
{
	set_bit(idx, (unsigned long *) &bdev->failed_mask);
}

static int __devinit bbu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	resource_size_t size;
	struct bbu_device *bdev;
	unsigned long end_pfn;
	int err;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	size = resource_size(res);
	if (!devm_request_mem_region(dev, res->start, size, pdev->name))
		return -EBUSY;

	bdev = devm_kzalloc(dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;
	INIT_LIST_HEAD(&bdev->caches);
	bdev->pdev = pdev;

	/* for consistency we do not want bbu memory backed by the cpu cache */
	bdev->start_pfn = res->start >> PAGE_SHIFT;
	end_pfn = (res->start + size - 1) >> PAGE_SHIFT;
	bdev->num_pages = end_pfn - bdev->start_pfn + 1;
	err = set_pages_wc(pfn_to_page(bdev->start_pfn), bdev->num_pages);
	if (err)
		return err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		size = resource_size(res);
		if (devm_request_mem_region(dev, res->start, size, pdev->name))
			bdev->ctrl = devm_ioremap(dev, res->start, size);
	}

	if (!bdev->ctrl)
		dev_dbg(dev, "control interface not found\n");

	mutex_lock(&bbu_lock);
	list_add(&bdev->node, &bbu_device_list);
	bdev->region = page_address(pfn_to_page(bdev->start_pfn));
	for (i = 0; i < BBU_MAX_REGIONS; i++) {
		struct bbu_region *region = &bdev->region[i];
		struct bbu_cache_conf *conf;

		region = validate_region(region);
		if (!region)
			continue;

		conf = alloc_add_cache_conf(bdev, i);
		if (!conf) {
			bbu_mark_failed_region(bdev, i);
			continue;
		}
		register_cache(bdev, conf);
	}
	mutex_unlock(&bbu_lock);

	return 0;
}

static int region_cmp(const void *_a, const void *_b)
{
	const struct bbu_region *a = *(struct bbu_region **) _a;
	const struct bbu_region *b = *(struct bbu_region **) _b;

	if (a->start_pfn < b->start_pfn)
		return -1;
	if (a->start_pfn > b->start_pfn)
		return 1;
	return 0;
}

static int insert_region(struct bbu_device *bdev, struct bbu_region *new)
{
	struct bbu_region *active[BBU_MAX_REGIONS+1];
	struct bbu_region *region = NULL;
	struct bbu_region final_region;
	struct bbu_cache_conf *conf;
	unsigned long maxsize;
	unsigned long start;
	unsigned long pos;
	int alloc_idx = -1;
	int active_count = 0;
	int i;

	/* find a free region slot and collect the active regions to
	 * scan for free space
	 */
	for (i = 0; i < BBU_MAX_REGIONS; i++) {
		struct bbu_region *slot = &bdev->region[i];

		if (validate_region(slot) == NULL) {
			if (!region) {
				alloc_idx = i;
				region = slot;
			}
		} else {
			if (strncmp(new->name, slot->name, sizeof(slot->name)) == 0)
				return -EEXIST;
			active[active_count++] = slot;
		}
	}
	if (!region)
		return -ENOSPC;

	sort(active, active_count, sizeof(active[0]), region_cmp, NULL);

	memset(&final_region, 0, sizeof(final_region));
	sprintf(final_region.name, "final");
	final_region.start_pfn = bdev->start_pfn + bdev->num_pages;
	final_region.size = 0;
	active[active_count] = &final_region;

	/* find the position and size of the largest free region */
	i = 0;
	maxsize = 0;
	pos = bdev->start_pfn + 1;
	start = pos;
	do {
		unsigned long size;

		size = active[i]->start_pfn - pos;
		if (size >= maxsize) {
			maxsize = size;
			start = pos;
		}
		pos = active[i]->start_pfn + bbu_region_to_pages(active[i]);
		i++;
	} while (active[i-1]->size);

	if (bbu_region_to_pages(new) > maxsize)
		return -ENOSPC;

	/* set a default size */
	if (new->size == 0) {
		unsigned long max = maxsize >> (20 - PAGE_SHIFT);
		unsigned long min = 1;

		/* search for the largest size (in megabytes) we can
		 * describe with the available pages and the requested
		 * blk_order
		 */
		do {
			unsigned long mid = (min + max) / 2;

			new->size = mid;
			if (bbu_region_to_pages(new) <= maxsize)
				min = mid + 1;
			else
				max = mid;
		} while (max - min > 1 && min < max);

		new->size = min;
		if (bbu_region_to_pages(new) > maxsize)
			new->size = min - 1;

		if (new->size == 0)
			return -ENOSPC;
	}

	/* initialize cache block descriptors */
	memset(page_address(pfn_to_page(start)), 0,
	       PAGE_SIZE * bbu_region_to_desc_pages(new));

	/* make sure the cache descs are init'd before new is inserted */
	wmb();

	new->magic = BBU_MAGIC;
	new->start_pfn = start;
	new->checksum = calc_checksum(new);
	*region = *new;

	conf = alloc_add_cache_conf(bdev, alloc_idx);
	if (!conf) {
		region->magic = 0; /* invalidate new region */
		return -ENOMEM;
	}

	return register_cache(bdev, conf);
}

static int parse_add_region(const char *input, struct bbu_region *new)
{
	/* tmp - 16 characters for name, 5 characters for size (MB), 1
	 * character for order (2^n pages), two ':' separators and a null
	 * termintator
	 */
	char tmp[16+1+5+1+1+1];
	int input_len = strlen(input);
	unsigned long val;
	char *c1, *c2;
	int i;

	memset(tmp, 0, sizeof(tmp));
	memset(new, 0, sizeof(*new));

	/* chop trailing newline */
	if (input[input_len - 1] == '\n' && input_len - 1 < sizeof(tmp)) {
		strncpy(tmp, input, sizeof(tmp));
		if (input_len <= sizeof(tmp))
			tmp[input_len - 1] = 0;
	} else if (input_len < sizeof(tmp))
		strncpy(tmp, input, sizeof(tmp));
	else
		return -E2BIG;

	c1 = strchr(tmp, ':');
	/* limit names to 16 characters */
	if (c1 && c1 - tmp > 16)
		return -EINVAL;
	else if (!c1 && strlen(tmp) > 16)
		return -EINVAL;

	for (i = 0; i < (c1 ? c1 - tmp : strlen(tmp)); i++) {
		if (isalnum(tmp[i]))
			new->name[i] = tmp[i];
		else
			return -EINVAL;
	}
	if (strlen(new->name) == 0)
		return -EINVAL;

	if (!c1)
		return 0; /* default size, order */

	c1++;
	c2 = strchr(c1, ':');

	if (c2 && c2 - c1 > 5)
		return -EFBIG;
	else if (!c2 && strlen(c1) > 5)
		return -EFBIG;

	if (c1 == c2)
		*c1 = 0; /* default size */
	else if (c2) {
		*c2 = 0;
		if (strict_strtoul(c1, 10, &val) != 0)
			return -EINVAL;
		new->size = val;
	} else if (strlen(c1)) {
		if (strict_strtoul(c1, 10, &val) != 0)
			return -EINVAL;
		new->size = val;
		return 0;
	} else {
		/* default size, order */
		return 0;
	}
	c2++;

	if (*c2 == 0)
		return 0; /* default order */

	if (strlen(c2) > 1)
		return -EFBIG;

	if (strict_strtoul(c2, 10, &val) != 0)
		return -EINVAL;

	/* order is limited to a 1MB block size */
	if (val + PAGE_SHIFT > 20)
		return -EINVAL;

	new->blk_order = val;

	return 0;
}

static int add_region(const char *val, struct kernel_param *kp)
{
	int err;

	mutex_lock(&bbu_lock);
	if (list_empty(&bbu_device_list))
		err = -ENODEV;
	else {
		struct bbu_device *bdev;
		struct bbu_region new;

		err = parse_add_region(val, &new);
		if (err == 0) {
			/* do a first-fit search to insert the new region */
			list_for_each_entry(bdev, &bbu_device_list, node) {
				err = insert_region(bdev, &new);
				if (err == 0)
					break;
			}
		}
	}
	mutex_unlock(&bbu_lock);

	return err;
}
module_param_call(new_region, add_region, NULL, NULL, S_IWUSR);

static struct platform_driver bbu_driver = {
	.probe		= bbu_probe,
	.remove		= __devexit_p(bbu_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "adr",
	},
};

static int __devinit bbu_init(void)
{
	int err;

	err = class_register(&bbu_class);
	if (err)
		return err;

	err = platform_driver_register(&bbu_driver);
	if (err)
		class_unregister(&bbu_class);

	return err;
}
/* be ready before block device drivers call bbu_register() */
subsys_initcall(bbu_init);

static void __devexit bbu_exit(void)
{
	class_unregister(&bbu_class);
	platform_driver_unregister(&bbu_driver);
}
module_exit(bbu_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("bbu: battery backed block-device cache");
MODULE_LICENSE("GPL");
