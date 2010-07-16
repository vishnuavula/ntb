/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2009 Intel Corporation. All rights reserved.
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
 * BSD LICENSE
 *
 * Copyright(c) 2009 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* This file implements network driver over NTB PCIE-LINK */
#ifndef NTBETHCQ_H
#define NTBETHCQ_H


#ifdef __cplusplus
extern "C"
{
#endif

#include "ntbethconf.h"

#define NTBETH_RX_CQ   1
#define NTBETH_TX_CQ   2

/* Tyedefition for an element of ciruclar queue
 */
struct q_element
{
  unsigned int pktlen;
  unsigned char payload[MAX_PACKET_BUF_LEN] ;
};

/*
 * This structure defines the complete ciruclar Queue
 * local structure
 */
 struct cq
{
	unsigned int cq_type;   // 1 means RX CQ desc for CQ  maintained locally
	// 2 means TX CQ desc for CQ maintained remotely
	unsigned int cq_size;
	unsigned int put_index;
	unsigned int get_index;
	unsigned int avail_put_index;
	unsigned int avail_get_index;
	unsigned int signature;
	struct q_element qarray[1];
};

inline  unsigned int cq_avail_put_index(struct cq *cq);
inline unsigned int cq_avail_get_index(struct cq *cq);
inline  unsigned int cq_put_index(struct cq *cq);
inline unsigned int cq_get_index(struct cq *cq);

//Operations on the Circular Queue

// All operations are not guranteed to be re-entrant. Callers must make sure that appropriate synchronization techniques applied when used in multi-kernel control path scenarios.

// return value -1 is error zero is success
int init_cq(struct cq *cq,int size, unsigned int  cq_type);

// returns  1 is q is empty otherwise zero
inline int is_cq_empty(struct cq *pCq);

// returns  1 is q is full otherwise zero
inline int is_cq_full(struct cq *pCq);

//just returns the current put entry address, no changes to the q state, except avail_put ptr
inline void *cq_get_current_put_entry_loc(struct cq *cq);

//just returns the current get entry address, no changes to the q state   except avail_get ptr
inline void *cq_get_current_get_entry_loc(struct cq *cq);

// updates get ptr
inline int cq_update_get_ptr(struct cq *cq);

// updates put ptr
inline int cq_update_put_ptr(struct cq *cq);

inline int cq_validate(struct cq *cq);

// calculate number of entries
inline int cq_calculate_num_entries(int size);

//dumps cq data structure
inline void cq_dump_debug_data(struct cq *cq, char *fmtstr);

inline int cq_is_buf_ready(struct cq *cq);
inline int cq_is_buf_avail(struct cq *cq);
inline void * cq_get_buffer(struct cq *cq, int i);

#ifdef __cplusplus
}
#endif

#endif
