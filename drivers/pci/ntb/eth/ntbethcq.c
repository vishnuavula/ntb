/*
 * This program implements network driver onver NTB hardware. 
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

#include <linux/module.h>
#include "ntbethcq.h"


int init_cq(struct cq *pCq, int iCqSize, unsigned int uiCqType, void *pRemoteCq)
{
  pCq->uiCqType = uiCqType;
  pCq->pRemoteCq = pRemoteCq;
  // intialize both put and get pointers to 0
  pCq->uiAvailPutWrapBit = pCq->uiAvailGetWrapBit = pCq->uiAvailPutIndex = pCq->uiAvailGetIndex = pCq->uiPutIndex = pCq->uiPutWrapBit = pCq->uiGetWrapBit = pCq->uiGetIndex = 0 ; 
  pCq->uiCqSize = iCqSize;
  pCq->uiSignature = 0xA12B23C3;
  return (0);
}

// return value -1 is error zero is success
int put_into_cq(struct cq *pCq, struct q_element *psEle)
{
  // check for full condition
   if((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit != pCq->uiPutWrapBit)) return (-1);
   if((pCq->uiPutIndex+1) == pCq->uiCqSize)
   {
      pCq->uiPutIndex = 0;
      pCq->uiPutWrapBit = (~pCq->uiPutWrapBit & 0x1);
   }
   else
   {
      pCq->uiPutIndex = pCq->uiPutIndex + 1;
   }
   pCq->sQArray[pCq->uiPutIndex] = *psEle;
  return 0;
}

int get_from_cq(struct cq *pCq, struct q_element *psEle)
{
  // check for empty condition
   if((pCq->uiGetIndex == pCq->uiPutIndex) && (pCq->uiGetWrapBit == pCq->uiPutWrapBit)) return (-1);
   if((pCq->uiGetIndex + 1) == pCq->uiCqSize)
   {
      pCq->uiGetIndex = 0;
      pCq->uiGetWrapBit = (~pCq->uiGetWrapBit & 0x1);
   }
   else
   {
      pCq->uiGetIndex = pCq->uiGetIndex +1;
   }
   *psEle = pCq->sQArray[pCq->uiGetIndex];
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
   if(cq_validate(cq))
   {
      printk("NTBETHCQ: ERROR cq validation failed in cq_get_current_put_entry_loc\n"); 
      return 0;
   }
   if((cq->uiAvailGetIndex == cq->uiAvailPutIndex) && (cq->uiAvailGetWrapBit != cq->uiAvailPutWrapBit))
   {
    printk (KERN_INFO ": ERROR: CQ full\n"); 
    return (0);
   }
   // return pointer to the packet remote  buffer 
  // loc =  ((void *)&cq->pRemoteCq->sQArray[cq->uiAvailPutIndex]);
   loc =  ((void *)&cq->sQArray[cq->uiAvailPutIndex]);
   if((cq->uiAvailPutIndex+1) == cq->uiCqSize)
   {
      cq->uiAvailPutIndex = 0;
      cq->uiAvailPutWrapBit = ((~cq->uiAvailPutWrapBit) & 0x1);
      // check for wrap bit here for full condition
   }
   else
   {
      cq->uiAvailPutIndex = cq->uiAvailPutIndex + 1;
   }
  return loc;
} 

// return 1 if validation fails 0 if success
int cq_validate(struct cq *cq)
{
   if(cq->uiPutIndex > cq->uiCqSize)
      goto validate_fail;
   if(cq->uiAvailPutIndex > cq->uiCqSize)
      goto validate_fail;
   if(cq->uiAvailGetIndex > cq->uiCqSize)
      goto validate_fail;
   if(cq->uiGetIndex > cq->uiCqSize)
      goto validate_fail;

   if(cq->uiSignature != 0xA12B23C3)
      goto validate_fail;
   else
      goto validate_pass;

validate_fail:
 printk("NTBETH ERROR: CQ Validation  failed \n");
 cq_dump_debug_data(cq,"VALIDATION");
 return 1;
validate_pass:
 return 0;

}
int cq_update_put_ptr(struct cq *cq)
{
  if(cq_validate(cq))
  {
    printk("NTHETHCQ: cq_update_put_ptr cq validate failed\n");
    return 1;
  }
  if((cq->uiPutIndex+1) == cq->uiCqSize)
  {
      cq->uiPutIndex = 0;
      cq->uiPutWrapBit = (~cq->uiPutWrapBit & 0x1);
  }
  else
  {
    cq->uiPutIndex = cq->uiPutIndex + 1; 
  }
  // update put ptr in mirro cq
  NTBETHDEBUG("Updating  Put Ptr in Remote CQ\n");
  NTBETHDEBUG("Updating uiAvailPutIndex 0x%x\n",cq->uiAvailPutIndex);
  NTBETHDEBUG("Updating uiAvailPutWrapBit 0x%x\n",cq->uiAvailPutWrapBit);
  NTBETHDEBUG("Updating uiPutIndex 0x%x\n",cq->uiPutIndex);
  NTBETHDEBUG("Updating uiPutWrapBit 0x%x\n",cq->uiPutWrapBit);
#if 0
  cq->pRemoteCq->uiAvailPutIndex = cq->uiAvailPutIndex;
  cq->pRemoteCq->uiAvailPutWrapBit = cq->uiAvailPutWrapBit;
  cq->pRemoteCq->uiPutIndex = cq->uiPutIndex;
  cq->pRemoteCq->uiPutWrapBit = cq->uiPutWrapBit;
#endif
  
  return 0;
} 

void *cq_get_current_get_entry_loc(struct cq *cq)
{
   void *loc;
   if(cq_validate(cq))
   {
     printk("NTBETHCQ: ERROR : cq_get_current_get_entry_loc cq validation failed\n");
     return 0;
   } 
   if((cq->uiAvailGetIndex == cq->uiAvailPutIndex) && (cq->uiAvailGetWrapBit == cq->uiAvailPutWrapBit))
    {
      printk (KERN_INFO " ERROR: CQ Empty\n"); 
      return (0);
    }
   loc = &cq->sQArray[cq->uiAvailGetIndex];
   if((cq->uiAvailGetIndex+1) == cq->uiCqSize)
   {
      cq->uiAvailGetIndex = 0;
      cq->uiAvailGetWrapBit = ((~cq->uiAvailGetWrapBit) & 0x1);
   }
   else
   {
      cq->uiAvailGetIndex= cq->uiAvailGetIndex+1;
   }
   return (loc);
} 

int cq_update_get_ptr(struct cq *cq)
{
  if(cq_validate(cq))
  {
    printk("NTBETHCQ: before cq_update_get_ptr cq validate failed\n");
    return 1;
  }
  if((cq->uiGetIndex+1) == cq->uiCqSize)
  {
      cq->uiGetIndex = 0;
      cq->uiGetWrapBit = (~cq->uiGetWrapBit & 0x1);
  }
  else
  {
    cq->uiGetIndex = cq->uiGetIndex + 1; 
  }
  // update get ptrs in remote cq
  NTBETHDEBUG("Updating  Get Ptr in Remote CQ\n");
  NTBETHDEBUG("Updating uiAvailGetIndex 0x%x\n",cq->uiAvailGetIndex);
  NTBETHDEBUG("Updating uiAvailGetWrapBit 0x%x\n",cq->uiAvailGetWrapBit);
  NTBETHDEBUG("Updating uiGetIndex 0x%x\n",cq->uiGetIndex);
  NTBETHDEBUG("Updating uiGetWrapBit 0x%x\n",cq->uiGetWrapBit);
#if 0
  cq->pRemoteCq->uiAvailGetWrapBit = cq->uiAvailGetWrapBit;
  cq->pRemoteCq->uiAvailGetIndex = cq->uiAvailGetIndex;
  cq->pRemoteCq->uiGetWrapBit = cq->uiGetWrapBit;
  cq->pRemoteCq->uiGetIndex = cq->uiGetIndex;
#endif
  if(cq_validate(cq))
  {
    printk("NTBETHCQ: after cq_update_get_ptr cq validate failed\n");
    return 1;
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

void cq_dump_debug_data(struct cq *cq, char *fmtstr)
{
  printk("%scqPtr == 0x%Lx\n", fmtstr,(unsigned long long)cq); 
  printk("%scqsize == 0x%x\n",fmtstr, cq->uiCqSize); 
  printk("%sPutIndex == 0x%x\n",fmtstr, cq->uiPutIndex); 
  printk("%sGetIndex == 0x%x\n",fmtstr, cq->uiGetIndex); 
  printk("%sPutWrapBit == 0x%x\n",fmtstr, cq->uiPutWrapBit); 
  printk("%sGetWrapBit == 0x%x\n",fmtstr, cq->uiGetWrapBit); 
  printk("%sAvailPutIndex == 0x%x\n",fmtstr, cq->uiAvailPutIndex); 
  printk("%sAvailGetIndex == 0x%x\n",fmtstr, cq->uiAvailGetIndex); 
  printk("%sAvailPutWrapBit == 0x%x\n",fmtstr, cq->uiAvailPutWrapBit); 
  printk("%sAvailGetWrapBit == 0x%x\n", fmtstr,cq->uiAvailGetWrapBit); 
  printk("%sSignature == 0x%x\n", fmtstr,cq->uiSignature); 
  return ;
} 
