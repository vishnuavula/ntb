/*
 * 128-bit uuid handling routines
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
#include <linux/module.h>

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

