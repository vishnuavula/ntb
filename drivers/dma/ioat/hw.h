/*
 * Copyright(c) 2004 - 2009 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */
#ifndef _IOAT_HW_H_
#define _IOAT_HW_H_

/* PCI Configuration Space Values */
#define IOAT_PCI_VID            0x8086
#define IOAT_MMIO_BAR		0

/* CB device ID's */
#define IOAT_PCI_DID_5000       0x1A38
#define IOAT_PCI_DID_CNB        0x360B
#define IOAT_PCI_DID_SCNB       0x65FF
#define IOAT_PCI_DID_SNB        0x402F

#define IOAT_PCI_RID            0x00
#define IOAT_PCI_SVID           0x8086
#define IOAT_PCI_SID            0x8086
#define IOAT_VER_1_2            0x12    /* Version 1.2 */
#define IOAT_VER_2_0            0x20    /* Version 2.0 */
#define IOAT_VER_3_0            0x30    /* Version 3.0 */
#define IOAT_VER_3_2            0x32    /* Version 3.2 */
#define IOAT_VER_3_3            0x33    /* Version 3.3 */

int system_has_dca_enabled(struct pci_dev *pdev);

struct ioat_dma_descriptor {
	uint32_t	size;
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int src_snoop_dis:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int null:1;
			unsigned int src_brk:1;
			unsigned int dest_brk:1;
			unsigned int bundle:1;
			unsigned int dest_dca:1;
			unsigned int hint:1;
			unsigned int rsvd2:13;
			#define IOAT_OP_COPY 0x00
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_addr;
	uint64_t	dst_addr;
	uint64_t	next;
	uint64_t	rsv1;
	uint64_t	rsv2;
	/* store some driver data in an unused portion of the descriptor */
	union {
		uint64_t	user1;
		uint64_t	tx_cnt;
	};
	uint64_t	user2;
};

struct ioat_fill_descriptor {
	uint32_t	size;
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int rsvd:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int rsvd2:2;
			unsigned int dest_brk:1;
			unsigned int bundle:1;
			unsigned int rsvd4:15;
			#define IOAT_OP_FILL 0x01
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_data;
	uint64_t	dst_addr;
	uint64_t	next;
	uint64_t	rsv1;
	uint64_t	next_dst_addr;
	uint64_t	user1;
	uint64_t	user2;
};

struct ioat_mcast_descriptor {
	uint32_t	size;
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int src_snoop_dis:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int null:1;
			unsigned int rsvd1:2;
			unsigned int bundle:1;
			unsigned int rsvd2:1;
			unsigned int hint:1;
			unsigned int ndest:3;
			unsigned int rsvd3:10;
			#define IOAT_OP_MCAST 0x0a
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_addr;
	uint64_t	dst_addr_1;
	uint64_t	next;
	uint64_t	dst_addr_x[4];
};

struct ioat_xor_descriptor {
	uint32_t	size;
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int src_snoop_dis:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int src_cnt:3;
			unsigned int bundle:1;
			unsigned int dest_dca:1;
			unsigned int hint:1;
			unsigned int rsvd:13;
			#define IOAT_OP_XOR 0x87
			#define IOAT_OP_XOR_VAL 0x88
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_addr;
	uint64_t	dst_addr;
	uint64_t	next;
	uint64_t	src_addr2;
	uint64_t	src_addr3;
	uint64_t	src_addr4;
	uint64_t	src_addr5;
};

struct ioat_xor_ext_descriptor {
	uint64_t	src_addr6;
	uint64_t	src_addr7;
	uint64_t	src_addr8;
	uint64_t	next;
	uint64_t	rsvd[4];
};

struct ioat_pq_descriptor {
	union {
		uint32_t	size;
		uint32_t	dwbes;
		struct {
			unsigned int rsvd:25;
			unsigned int p_val_err:1;
			unsigned int q_val_err:1;
			unsigned int rsvd1:4;
			unsigned int wbes:1;
		} dwbes_f;
	};
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int src_snoop_dis:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int src_cnt:3;
			unsigned int bundle:1;
			unsigned int dest_dca:1;
			unsigned int hint:1;
			unsigned int p_disable:1;
			unsigned int q_disable:1;
			unsigned int rsvd2:2;
			unsigned int wb_en:1;
			unsigned int prl_en:1;
			unsigned int rsvd3:7;
			#define IOAT_OP_PQ 0x89
			#define IOAT_OP_PQ_VAL 0x8a
			#define IOAT_OP_PQ_16S 0xa0
			#define IOAT_OP_PQ_VAL_16S 0xa1
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_addr;
	uint64_t	p_addr;
	uint64_t	next;
	uint64_t	src_addr2;
	union {
		uint64_t	src_addr3;
		uint64_t	sed_addr;
	};
	uint8_t		coef[8];
	uint64_t	q_addr;
};

struct ioat_pq_ext_descriptor {
	uint64_t	src_addr4;
	uint64_t	src_addr5;
	uint64_t	src_addr6;
	uint64_t	next;
	uint64_t	src_addr7;
	uint64_t	src_addr8;
	uint64_t	rsvd[2];
};

struct ioat_pq_update_descriptor {
	uint32_t	size;
	union {
		uint32_t ctl;
		struct {
			unsigned int int_en:1;
			unsigned int src_snoop_dis:1;
			unsigned int dest_snoop_dis:1;
			unsigned int compl_write:1;
			unsigned int fence:1;
			unsigned int src_cnt:3;
			unsigned int bundle:1;
			unsigned int dest_dca:1;
			unsigned int hint:1;
			unsigned int p_disable:1;
			unsigned int q_disable:1;
			unsigned int rsvd:3;
			unsigned int coef:8;
			#define IOAT_OP_PQ_UP 0x8b
			unsigned int op:8;
		} ctl_f;
	};
	uint64_t	src_addr;
	uint64_t	p_addr;
	uint64_t	next;
	uint64_t	src_addr2;
	uint64_t	p_src;
	uint64_t	q_src;
	uint64_t	q_addr;
};

struct ioat_raw_descriptor {
	uint64_t	field[8];
};

struct ioat_pq16a_descriptor {
	uint8_t coef[8];
	uint64_t src_addr3;
	uint64_t src_addr4;
	uint64_t src_addr5;
	uint64_t src_addr6;
	uint64_t src_addr7;
	uint64_t src_addr8;
	uint64_t src_addr9;
};

struct ioat_pq16b_descriptor {
	uint64_t src_addr10;
	uint64_t src_addr11;
	uint64_t src_addr12;
	uint64_t src_addr13;
	uint64_t src_addr14;
	uint64_t src_addr15;
	uint64_t src_addr16;
	uint64_t rsvd;
};

union ioat_sed_pq_descriptor {
	struct ioat_pq16a_descriptor a;
	struct ioat_pq16b_descriptor b;
};

struct ioat_sed_raw_descriptor {
	uint64_t	a[8];
	uint64_t	b[8];
};

#endif
