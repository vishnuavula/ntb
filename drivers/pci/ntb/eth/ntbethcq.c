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


int init_cq(struct cq *cq, int cq_size, unsigned int cq_type)
{
  cq->cq_type = cq_type;
  // intialize both put and get pointers to 0
  cq->avail_put_index = cq->avail_get_index = cq->put_index = cq->get_index = 0 ; 
  cq->cq_size = cq_size;
  cq->signature = 0xA12B23C3;
  return (0);
}

// return value 1 is Qempty zero is not empty
int is_cq_empty(struct cq *cq)
{
  // check for empty condition
   return(cq->get_index == cq->put_index) ;  
}

// return value 1 is Qempty zero is not empty
int is_cq_full(struct cq *cq)
{
  int tmp;
  tmp = (cq->put_index + 1) % cq->cq_size;
  return(cq->get_index == tmp);
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
   loc =  ((void *)&cq->qarray[cq->avail_put_index]);
   if((cq->avail_put_index+1) == cq->cq_size)
   {
      cq->avail_put_index = 0;
   }
   else
   {
      cq->avail_put_index = cq->avail_put_index + 1;
   }
   return loc;
} 

// return 1 if validation fails 0 if success
int cq_validate(struct cq *cq)
{
   if(cq->put_index > cq->cq_size)
      goto validate_fail;
   if(cq->avail_put_index > cq->cq_size)
      goto validate_fail;
   if(cq->avail_get_index > cq->cq_size)
      goto validate_fail;
   if(cq->get_index > cq->cq_size)
      goto validate_fail;

   if(cq->signature != 0xA12B23C3)
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
  if((cq->put_index+1) == cq->cq_size)
  {
      cq->put_index = 0;
  }
  else
  {
    cq->put_index = cq->put_index + 1; 
  }
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
   loc = &cq->qarray[cq->avail_get_index];
   if((cq->avail_get_index+1) == cq->cq_size)
   {
      cq->avail_get_index = 0;
   }
   else
   {
      cq->avail_get_index= cq->avail_get_index+1;
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
  if((cq->get_index+1) == cq->cq_size)
  {
      cq->get_index = 0;
  }
  else
  {
    cq->get_index = cq->get_index + 1; 
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
  printk("%scqsize == 0x%x\n",fmtstr, cq->cq_size); 
  printk("%sPutIndex == 0x%x\n",fmtstr, cq->put_index); 
  printk("%sGetIndex == 0x%x\n",fmtstr, cq->get_index); 
  printk("%sAvailPutIndex == 0x%x\n",fmtstr, cq->avail_put_index); 
  printk("%sAvailGetIndex == 0x%x\n",fmtstr, cq->avail_get_index); 
  printk("%sSignature == 0x%x\n", fmtstr,cq->signature); 
  return ;
} 
 // return  0 if no buffer
 //return  non-zero if  buffer
int cq_is_buf_ready(struct cq *cq)
{
    return(!(cq->avail_get_index == cq->put_index));
}
// return  0 if  full
 //return  non-zero if  enough space to copy
int cq_is_buf_avail(struct cq *cq)
{
    return(!(((cq->avail_put_index + 1) % cq->cq_size) == cq->get_index));
}

inline  unsigned int cq_avail_put_index(struct cq *cq)
{
   return (cq->avail_put_index);
}
inline unsigned int cq_avail_get_index(struct cq *cq)
{
   return (cq->avail_get_index);
}
inline  unsigned int cq_put_index(struct cq *cq)
{
   return (cq->put_index);
}
inline unsigned int cq_get_index(struct cq *cq)
{
   return (cq->get_index);
}
void * cq_get_buffer(struct cq *cq, int i)
{
   void *loc;
   loc =  ((void *)&cq->qarray[i]);
   return loc;
}
