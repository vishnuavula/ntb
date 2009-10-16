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
#ifndef NTBETHCQ_H
#define NTBETHCQ_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ntbethconf.h"

/* Tyedefition for an element of ciruclar queue
 */
struct q_element
{
  unsigned int pktlen;
  unsigned char payload[MAX_PACKET_BUF_LEN] ;
};

/*
 * This structure defines the complete ciruclar Queue
 */
 struct cq
{
  unsigned int uiCqSize;
  unsigned int uiPutIndex;
  unsigned int uiGetIndex;
  unsigned int uiPutWrapBit;
  unsigned int uiGetWrapBit;
  unsigned int uiAvailPutIndex;
  unsigned int uiAvailGetIndex;
  unsigned int uiAvailPutWrapBit;
  unsigned int uiAvailGetWrapBit;
  struct q_element sQArray[1];
};


//Operations on the Circular Queue

// All operations are not guranteed to be re-entrant. Callers must make sure that appropriate synchronization techniques applied when used in multi-kernel control path scenarios. 



// return value -1 is error zero is success
int init_cq(struct cq *pCq,int iSize);

// directly copy the given element into current cq put entry and update the put pointer.
int put_into_cq(struct cq *pCq, struct q_element* psEle);

// directly copy the current cq get entry to given element and update the get pointer.
int get_from_cq(struct cq *pCq, struct q_element *psEle);

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

// calculate number of entries 
int cq_calculate_num_entries(int size);

//dumps cq data structure 
void cq_dump_debug_data(struct cq *cq);

#ifdef __cplusplus
}
#endif 

#endif
