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
#include <linux/module.h>
#include "ntbethcq.h"
	
int init_cq(struct cq *cq, int cq_size, unsigned int cq_type)
{
	cq->cq_type = cq_type;
	// intialize both put and get pointers to 0
	cq->avail_put_index = cq->avail_get_index = cq->put_index = cq->get_index = 0 ;
	cq->cq_size = cq_size;
	cq->signature = 0xA12B23C3;
	return NTBETH_SUCCESS;
}
	
	// return value 1 is Qempty zero is not empty
inline int is_cq_empty(struct cq *cq)
{
	// check for empty condition
	return(cq->get_index == cq->put_index) ;
}
	
	// return value 1 is Qempty zero is not empty
inline int is_cq_full(struct cq *cq)
{
	int tmp;
	tmp = (cq->put_index + 1) % cq->cq_size;
	return(cq->get_index == tmp);
}
	
	// this routine returns 0 if the q is full.
inline void *cq_get_current_put_entry_loc(struct cq *cq)
{
	void *loc;
	if(cq_validate(cq)) {
		printk("NTBETHCQ: ERROR cq validation failed in cq_get_current_put_entry_loc\n");
		return 0;
	}
	loc =  ((void *)&cq->qarray[cq->avail_put_index]);
	if((cq->avail_put_index+1) == cq->cq_size) {
		cq->avail_put_index = 0;
	} else {
		cq->avail_put_index = cq->avail_put_index + 1;
	}
	return loc;
}
	
	// return 1 if validation fails 0 if success
int cq_validate(struct cq *cq)
{
	if (cq->put_index > cq->cq_size)
		goto validate_fail;
	if (cq->avail_put_index > cq->cq_size)
		goto validate_fail;
	if (cq->avail_get_index > cq->cq_size)
		goto validate_fail;
	if (cq->get_index > cq->cq_size)
		goto validate_fail;
	if (cq->signature != 0xA12B23C3)
		goto validate_fail;
	else
		goto validate_pass;
	
	validate_fail:
		printk("NTBETH ERROR: CQ Validation  failed \n");
		cq_dump_debug_data(cq,"VALIDATION");
		return NTBETH_FAIL;
	validate_pass:
	return NTBETH_SUCCESS;
}
inline int cq_update_put_ptr(struct cq *cq)
{
	if (cq_validate(cq)) {
		printk("NTHETHCQ: cq_update_put_ptr cq validate failed\n");
		return 1;
	}
	if ((cq->put_index+1) == cq->cq_size)
		cq->put_index = 0;
	else
		cq->put_index = cq->put_index + 1;
	return NTBETH_SUCCESS;
}
	
inline void *cq_get_current_get_entry_loc(struct cq *cq)
{
	void *loc;
	if (cq_validate(cq)) {
		printk("NTBETHCQ: ERROR : cq_get_current_get_entry_loc cq validation failed\n");
		return 0;
	}
	loc = &cq->qarray[cq->avail_get_index];
	if ((cq->avail_get_index+1) == cq->cq_size)
		cq->avail_get_index = 0;
	else
		cq->avail_get_index= cq->avail_get_index+1;
	return loc;
}
	
inline int cq_update_get_ptr(struct cq *cq)
{
	if (cq_validate(cq)) {
		printk("NTBETHCQ: before cq_update_get_ptr cq validate failed\n");
		return 1;
	}
	if ((cq->get_index+1) == cq->cq_size)
		cq->get_index = 0;
	else
		cq->get_index = cq->get_index + 1;
	return 0;
}
	
inline int cq_calculate_num_entries(int size)
{
	int cqsize = size - sizeof(struct cq) + sizeof(struct q_element);
	//printk(KERN_INFO " cqsize 0x%x, tCQsize 0x%x tCQElemetSize 0x%x size 0x%x\n",cqsize,sizeof(struct cq), sizeof(struct cq_element), size);
	if(cqsize < 0)
		return 0;
	else
		cqsize = (cqsize / sizeof(struct q_element));
	return cqsize;
}
	
inline void cq_dump_debug_data(struct cq *cq, char *fmtstr)
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
inline int cq_is_buf_ready(struct cq *cq)
{
	return(!(cq->avail_get_index == cq->put_index));
}
	// return  0 if  full
	//return  non-zero if  enough space to copy
inline int cq_is_buf_avail(struct cq *cq)
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
inline void * cq_get_buffer(struct cq *cq, int i)
{
	void *loc;
	loc =  ((void *)&cq->qarray[i]);
	return loc;
}
