/*
 * Unified UUID/GUID definition
 *
 * Copyright (C) 2009, Intel Corp.
 *	Huang Ying <ying.huang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uuid.h>
#include <linux/random.h>

static void __uuid_gen_common(__u8 b[16])
{
	int i;
	u32 r;

	for (i = 0; i < 4; i++) {
		r = random32();
		memcpy(b + i * 4, &r, 4);
	}
	/* reversion 0b10 */
	b[8] = (b[8] & 0x3F) | 0x80;
}

void uuid_le_gen(uuid_le *lu)
{
	__uuid_gen_common(lu->b);
	/* version 4 : random generation */
	lu->b[7] = (lu->b[7] & 0x0F) | 0x40;
}
EXPORT_SYMBOL_GPL(uuid_le_gen);

void uuid_be_gen(uuid_be *bu)
{
	__uuid_gen_common(bu->b);
	/* version 4 : random generation */
	bu->b[6] = (bu->b[6] & 0x0F) | 0x40;
}
EXPORT_SYMBOL_GPL(uuid_be_gen);

char *uuid_to_string(char *buf, const char uuid[16], int sysfs)
{
	int i, j;
	int id;
	char *c = buf;

	for (i = 0; i < 4; i++) {
		id = uuid[i];
		if (i)
			*c++ = '-';
		for (j = 3; j >= 0; j--) {
			sprintf(c, "%02x", (unsigned char) uuid[j+4*i]);
			c += 2;
		}
	}
	if (sysfs)
		*c++ = '\n';
	*c = 0;

	return buf;
}
EXPORT_SYMBOL(uuid_to_string);


/* parse_uuid - parse a 128 bit uuid into 4 integers
 * @uuid: integer array to store parsed value
 * @str: text formatted uuid
 *
 * The format is 32 hex nibbles with optional :.<space>-\n separators.
 * If the input is not exactly 32 hex digits, return -EINVAL else return 0.
 */
int parse_uuid(int uuid[4], const char *str)
{
	int hit = 0; /* number of hex digit */
	int i;
	char c;

	for (i = 0; i < 4; i++)
		uuid[i] = 0;

	while ((c = *str++)) {
		int n;

		if (c >= '0' && c <= '9')
			n = c-'0';
		else if (c >= 'a' && c <= 'f')
			n = 10 + c - 'a';
		else if (c >= 'a' && c <= 'f')
			n = 10 + c - 'a';
		else if (strchr(":. -\n", c))
			continue;
		else
			return -EINVAL;

		if (hit < 32) {
			uuid[hit / 8] <<= 4;
			uuid[hit / 8] += n;
		}
		hit++;
	}

	if (hit == 32)
		return 0;

	return -EINVAL;
}
EXPORT_SYMBOL(parse_uuid);
