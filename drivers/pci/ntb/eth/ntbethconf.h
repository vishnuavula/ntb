/*
 * This program implements API to control NTB hardware.
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

#ifndef NTBETHCONF_H
#define NTBETHCONF_H

//enable/disable debug logs
#define NTBETHDEBUG(fmt, args...) printk(KERN_INFO "ntbeth: " fmt, ## args)
//#define NTBETHDEBUG(fmt, args...) /* not debugging: nothing */

#define MAX_PACKET_BUF_LEN  1600
#define NTBETH_MAC              "\0NTBE0"
#define NTBETH_MAX_MTUSIZE     1500
#define NTBETH_MIN_MTUSIZE     68

#define NTBETH_SUCCESS 0
#define NTBETH_FAIL 1

#endif 
