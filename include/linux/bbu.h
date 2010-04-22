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
#include <linux/blkdev.h>

struct bbu_device_info {
	sector_t stripe_sectors;
	int stripe_members;
};

#if defined(CONFIG_BLK_BBU) || defined(CONFIG_BLK_BBU_MODULE)
extern make_request_fn *bbu_register(const char uuid[16],
				    struct gendisk *disk,
				    make_request_fn *make_request,
				    struct bbu_device_info *info);
extern int bbu_unregister(const char uuid[16], struct gendisk *disk);
#else
static inline make_request_fn *bbu_register(const char uuid[16],
					    struct gendisk *disk,
					    make_request_fn *make_request,
					    struct bbu_device_info *info)
{
	return ERR_PTR(-ENODEV);
}

static inline int bbu_unregister(const char uuid[16], struct gendisk *disk)
{
	return -ENODEV;
}
#endif
