/*
 * This program implements network driver over NTB hardware.
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
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 */

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
int is_cq_empty(struct cq *pCq);

// returns  1 is q is full otherwise zero 
int is_cq_full(struct cq *pCq);

//just returns the current put entry address, no changes to the q state, except avail_put ptr  
void *cq_get_current_put_entry_loc(struct cq *cq);

//just returns the current get entry address, no changes to the q state   except avail_get ptr
void *cq_get_current_get_entry_loc(struct cq *cq);

// updates get ptr
int cq_update_get_ptr(struct cq *cq);

// updates put ptr
int cq_update_put_ptr(struct cq *cq);

int cq_validate(struct cq *cq);

// calculate number of entries 
int cq_calculate_num_entries(int size);

//dumps cq data structure 
void cq_dump_debug_data(struct cq *cq, char *fmtstr);

int cq_is_buf_ready(struct cq *cq);
int cq_is_buf_avail(struct cq *cq);
void * cq_get_buffer(struct cq *cq, int i);

#ifdef __cplusplus
}
#endif 

#endif
