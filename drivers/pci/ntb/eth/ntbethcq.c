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
#include <linux/module.h>
#include "ntbethcq.h"


int init_cq(struct cq *pCq, int iCqSize)
{
  // intialize both put and get pointers to 0
  pCq->uiAvailPutWrapBit = pCq->uiAvailGetWrapBit = pCq->uiAvailPutIndex = pCq->uiAvailGetIndex = pCq->uiPutIndex = pCq->uiPutWrapBit = pCq->uiGetWrapBit = pCq->uiGetIndex = 0 ; 
  pCq->uiCqSize = iCqSize;
  return (0);
}

// return value -1 is error zero is success
int put_into_cq(struct cq *pCq, struct q_element *psEle)
{
  // check for full condition
   if((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit != pCq->uiPutWrapBit)) return (-1);
   pCq->sQArray[pCq->uiPutIndex++] = *psEle;
   if(pCq->uiPutIndex == pCq->uiCqSize)
   {
      pCq->uiPutIndex = 0;
      pCq->uiPutWrapBit = (~pCq->uiPutWrapBit & 0x1);
   }
  return 0;
}

int get_from_cq(struct cq *pCq, struct q_element *psEle)
{
  // check for empty condition
   if((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit == pCq->uiPutWrapBit)) return (-1);
   *psEle = pCq->sQArray[pCq->uiGetIndex++];
   if(pCq->uiGetIndex == pCq->uiCqSize)
   {
      pCq->uiGetIndex = 0;
      pCq->uiGetWrapBit = (~pCq->uiGetWrapBit & 0x1);
   }
  return 0;
}

// return value 1 is Qempty zero is not empty
int is_cq_empty(struct cq *pCq)
{
  // check for empty condition
   return((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit == pCq->uiPutWrapBit));  
}

// return value 1 is Qempty zero is not empty
int is_cq_full(struct cq *pCq)
{
  // check for empty condition
   return((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit != pCq->uiPutWrapBit));  
}

// this routine returns 0 if the q is full.
void *cq_get_current_put_entry_loc(struct cq *cq)
{
  void *loc;
   if((cq->uiAvailGetIndex == cq->uiAvailPutIndex) && (cq->uiAvailGetWrapBit != cq->uiAvailPutWrapBit))
   {
    printk (KERN_INFO ": ERROR: CQ full\n"); 
    return (0);
   }
   loc =  ((void *)&cq->sQArray[cq->uiAvailPutIndex++]);
   if(cq->uiAvailPutIndex == cq->uiCqSize)
   {
      cq->uiAvailPutIndex = 0;
      cq->uiAvailPutWrapBit = (~cq->uiAvailPutWrapBit & 0x1);
      // check for wrap bit here for full condition
   }
  return loc;
} 

int cq_update_put_ptr(struct cq *cq)
{
  cq->uiPutIndex++; 
  if((cq->uiPutIndex) == cq->uiCqSize)
  {
      cq->uiPutIndex = 0;
      cq->uiPutWrapBit = (~cq->uiPutWrapBit & 0x1);
  }
  return 0;
} 

void *cq_get_current_get_entry_loc(struct cq *cq)
{
   void *loc;
   if((cq->uiAvailGetIndex == cq->uiAvailPutIndex) && (cq->uiAvailGetWrapBit == cq->uiAvailPutWrapBit))
    {
      printk (KERN_INFO " ERROR: CQ Empty\n"); 
      return (0);
    }
   loc = &cq->sQArray[cq->uiAvailGetIndex++];
   if(cq->uiAvailGetIndex == cq->uiCqSize)
   {
      cq->uiAvailGetIndex = 0;
      cq->uiAvailGetWrapBit = (~cq->uiAvailGetWrapBit & 0x1);
   }
   return (loc);
} 

int cq_update_get_ptr(struct cq *cq)
{
  cq->uiGetIndex++; 
  if((cq->uiGetIndex) == cq->uiCqSize)
  {
      cq->uiGetIndex = 0;
      cq->uiGetWrapBit = (~cq->uiGetWrapBit & 0x1);
  }
  return 0;
} 

int cq_calculate_num_entries(int size)
{
   int cqsize = size - sizeof(struct cq) + sizeof(struct q_element);
   //printk(KERN_INFO " cqsize 0x%x, tCQsize 0x%x tCQElemetSize 0x%x size 0x%x\n",cqsize,sizeof(struct cq), sizeof(struct cq_element), size); 
   if(cqsize < 0)
   {
      return 0;
   }
   else
   {
       cqsize = (cqsize / sizeof(struct q_element));
   }
   return cqsize;
}

void cq_dump_debug_data(struct cq *cq)
{
  printk(KERN_INFO "cqPtr == 0x%Lx\n", (unsigned long long)cq); 
  printk(KERN_INFO "cqsize == 0x%x\n", cq->uiCqSize); 
  printk(KERN_INFO "PutIndex == 0x%x\n", cq->uiPutIndex); 
  printk(KERN_INFO "GetIndex == 0x%x\n", cq->uiGetIndex); 
  printk(KERN_INFO "PutWrapBit == 0x%x\n", cq->uiPutWrapBit); 
  printk(KERN_INFO "GetWrapBit == 0x%x\n", cq->uiGetWrapBit); 
  printk(KERN_INFO "AvailPutIndex == 0x%x\n", cq->uiAvailPutIndex); 
  printk(KERN_INFO "AvailGetIndex == 0x%x\n", cq->uiAvailGetIndex); 
  printk(KERN_INFO "AvailPutWrapBit == 0x%x\n", cq->uiAvailPutWrapBit); 
  printk(KERN_INFO "AvailGetWrapBit == 0x%x\n", cq->uiAvailGetWrapBit); 
  return ;
} 
