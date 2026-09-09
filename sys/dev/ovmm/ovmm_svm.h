/*
 * Copyright (c) 2014 Mike Larkin <mlarkin@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * ovmm(4): a port of the OpenBSD vmm(4) hypervisor to FreeBSD.
 *
 * This header contains the AMD SVM (RVI) specific definitions that were
 * originally in OpenBSD's <machine/vmmvar.h>.  The Intel VMX and AMD
 * SEV/SEV-ES code paths of the original were not ported; ovmm(4) only
 * supports AMD processors with nested paging.
 */

#ifndef _DEV_OVMM_OVMM_SVM_H_
#define _DEV_OVMM_OVMM_SVM_H_

#include <sys/types.h>
#include <sys/ioccom.h>

#define VMM_HV_SIGNATURE	"OpenBSDVMM58"

/* SVM: Intercept codes (exit reasons) */
#define SVM_VMEXIT_CR0_READ			0x00
#define SVM_VMEXIT_CR1_READ			0x01
#define SVM_VMEXIT_CR2_READ			0x02
#define SVM_VMEXIT_CR3_READ			0x03
#define SVM_VMEXIT_CR4_READ			0x04
#define SVM_VMEXIT_CR5_READ			0x05
#define SVM_VMEXIT_CR6_READ			0x06
#define SVM_VMEXIT_CR7_READ			0x07
#define SVM_VMEXIT_CR8_READ			0x08
#define SVM_VMEXIT_CR9_READ			0x09
#define SVM_VMEXIT_CR10_READ			0x0A
#define SVM_VMEXIT_CR11_READ			0x0B
#define SVM_VMEXIT_CR12_READ			0x0C
#define SVM_VMEXIT_CR13_READ			0x0D
#define SVM_VMEXIT_CR14_READ			0x0E
#define SVM_VMEXIT_CR15_READ			0x0F
#define SVM_VMEXIT_CR0_WRITE			0x10
#define SVM_VMEXIT_CR1_WRITE			0x11
#define SVM_VMEXIT_CR2_WRITE			0x12
#define SVM_VMEXIT_CR3_WRITE			0x13
#define SVM_VMEXIT_CR4_WRITE			0x14
#define SVM_VMEXIT_CR5_WRITE			0x15
#define SVM_VMEXIT_CR6_WRITE			0x16
#define SVM_VMEXIT_CR7_WRITE			0x17
#define SVM_VMEXIT_CR8_WRITE			0x18
#define SVM_VMEXIT_CR9_WRITE			0x19
#define SVM_VMEXIT_CR10_WRITE			0x1A
#define SVM_VMEXIT_CR11_WRITE			0x1B
#define SVM_VMEXIT_CR12_WRITE			0x1C
#define SVM_VMEXIT_CR13_WRITE			0x1D
#define SVM_VMEXIT_CR14_WRITE			0x1E
#define SVM_VMEXIT_CR15_WRITE			0x1F
#define SVM_VMEXIT_DR0_READ			0x20
#define SVM_VMEXIT_DR1_READ			0x21
#define SVM_VMEXIT_DR2_READ			0x22
#define SVM_VMEXIT_DR3_READ			0x23
#define SVM_VMEXIT_DR4_READ			0x24
#define SVM_VMEXIT_DR5_READ			0x25
#define SVM_VMEXIT_DR6_READ			0x26
#define SVM_VMEXIT_DR7_READ			0x27
#define SVM_VMEXIT_DR8_READ			0x28
#define SVM_VMEXIT_DR9_READ			0x29
#define SVM_VMEXIT_DR10_READ			0x2A
#define SVM_VMEXIT_DR11_READ			0x2B
#define SVM_VMEXIT_DR12_READ			0x2C
#define SVM_VMEXIT_DR13_READ			0x2D
#define SVM_VMEXIT_DR14_READ			0x2E
#define SVM_VMEXIT_DR15_READ			0x2F
#define SVM_VMEXIT_DR0_WRITE			0x30
#define SVM_VMEXIT_DR1_WRITE			0x31
#define SVM_VMEXIT_DR2_WRITE			0x32
#define SVM_VMEXIT_DR3_WRITE			0x33
#define SVM_VMEXIT_DR4_WRITE			0x34
#define SVM_VMEXIT_DR5_WRITE			0x35
#define SVM_VMEXIT_DR6_WRITE			0x36
#define SVM_VMEXIT_DR7_WRITE			0x37
#define SVM_VMEXIT_DR8_WRITE			0x38
#define SVM_VMEXIT_DR9_WRITE			0x39
#define SVM_VMEXIT_DR10_WRITE			0x3A
#define SVM_VMEXIT_DR11_WRITE			0x3B
#define SVM_VMEXIT_DR12_WRITE			0x3C
#define SVM_VMEXIT_DR13_WRITE			0x3D
#define SVM_VMEXIT_DR14_WRITE			0x3E
#define SVM_VMEXIT_DR15_WRITE			0x3F
#define SVM_VMEXIT_EXCP0			0x40
#define SVM_VMEXIT_EXCP1			0x41
#define SVM_VMEXIT_EXCP2			0x42
#define SVM_VMEXIT_EXCP3			0x43
#define SVM_VMEXIT_EXCP4			0x44
#define SVM_VMEXIT_EXCP5			0x45
#define SVM_VMEXIT_EXCP6			0x46
#define SVM_VMEXIT_EXCP7			0x47
#define SVM_VMEXIT_EXCP8			0x48
#define SVM_VMEXIT_EXCP9			0x49
#define SVM_VMEXIT_EXCP10			0x4A
#define SVM_VMEXIT_EXCP11			0x4B
#define SVM_VMEXIT_EXCP12			0x4C
#define SVM_VMEXIT_EXCP13			0x4D
#define SVM_VMEXIT_EXCP14			0x4E
#define SVM_VMEXIT_EXCP15			0x4F
#define SVM_VMEXIT_EXCP16			0x50
#define SVM_VMEXIT_EXCP17			0x51
#define SVM_VMEXIT_EXCP18			0x52
#define SVM_VMEXIT_EXCP19			0x53
#define SVM_VMEXIT_EXCP20			0x54
#define SVM_VMEXIT_EXCP21			0x55
#define SVM_VMEXIT_EXCP22			0x56
#define SVM_VMEXIT_EXCP23			0x57
#define SVM_VMEXIT_EXCP24			0x58
#define SVM_VMEXIT_EXCP25			0x59
#define SVM_VMEXIT_EXCP26			0x5A
#define SVM_VMEXIT_EXCP27			0x5B
#define SVM_VMEXIT_EXCP28			0x5C
#define SVM_VMEXIT_EXCP29			0x5D
#define SVM_VMEXIT_EXCP30			0x5E
#define SVM_VMEXIT_EXCP31			0x5F
#define SVM_VMEXIT_INTR				0x60
#define SVM_VMEXIT_NMI				0x61
#define SVM_VMEXIT_SMI				0x62
#define SVM_VMEXIT_INIT				0x63
#define SVM_VMEXIT_VINTR			0x64
#define SVM_VMEXIT_CR0_SEL_WRITE		0x65
#define SVM_VMEXIT_IDTR_READ			0x66
#define SVM_VMEXIT_GDTR_READ			0x67
#define SVM_VMEXIT_LDTR_READ			0x68
#define SVM_VMEXIT_TR_READ			0x69
#define SVM_VMEXIT_IDTR_WRITE			0x6A
#define SVM_VMEXIT_GDTR_WRITE			0x6B
#define SVM_VMEXIT_LDTR_WRITE			0x6C
#define SVM_VMEXIT_TR_WRITE			0x6D
#define SVM_VMEXIT_RDTSC			0x6E
#define SVM_VMEXIT_RDPMC			0x6F
#define SVM_VMEXIT_PUSHF			0x70
#define SVM_VMEXIT_POPF				0x71
#define SVM_VMEXIT_CPUID			0x72
#define SVM_VMEXIT_RSM				0x73
#define SVM_VMEXIT_IRET				0x74
#define SVM_VMEXIT_SWINT			0x75
#define SVM_VMEXIT_INVD				0x76
#define SVM_VMEXIT_PAUSE			0x77
#define SVM_VMEXIT_HLT				0x78
#define SVM_VMEXIT_INVLPG			0x79
#define SVM_VMEXIT_INVLPGA			0x7A
#define SVM_VMEXIT_IOIO				0x7B
#define SVM_VMEXIT_MSR				0x7C
#define SVM_VMEXIT_TASK_SWITCH			0x7D
#define SVM_VMEXIT_FERR_FREEZE			0x7E
#define SVM_VMEXIT_SHUTDOWN			0x7F
#define SVM_VMEXIT_VMRUN			0x80
#define SVM_VMEXIT_VMMCALL			0x81
#define SVM_VMEXIT_VMLOAD			0x82
#define SVM_VMEXIT_VMSAVE			0x83
#define SVM_VMEXIT_STGI				0x84
#define SVM_VMEXIT_CLGI				0x85
#define SVM_VMEXIT_SKINIT			0x86
#define SVM_VMEXIT_RDTSCP			0x87
#define SVM_VMEXIT_ICEBP			0x88
#define SVM_VMEXIT_WBINVD			0x89
#define SVM_VMEXIT_MONITOR			0x8A
#define SVM_VMEXIT_MWAIT			0x8B
#define SVM_VMEXIT_MWAIT_CONDITIONAL		0x8C
#define SVM_VMEXIT_XSETBV			0x8D
#define SVM_VMEXIT_EFER_WRITE_TRAP		0x8F
#define SVM_VMEXIT_CR0_WRITE_TRAP		0x90
#define SVM_VMEXIT_CR1_WRITE_TRAP		0x91
#define SVM_VMEXIT_CR2_WRITE_TRAP		0x92
#define SVM_VMEXIT_CR3_WRITE_TRAP		0x93
#define SVM_VMEXIT_CR4_WRITE_TRAP		0x94
#define SVM_VMEXIT_CR5_WRITE_TRAP		0x95
#define SVM_VMEXIT_CR6_WRITE_TRAP		0x96
#define SVM_VMEXIT_CR7_WRITE_TRAP		0x97
#define SVM_VMEXIT_CR8_WRITE_TRAP		0x98
#define SVM_VMEXIT_CR9_WRITE_TRAP		0x99
#define SVM_VMEXIT_CR10_WRITE_TRAP		0x9A
#define SVM_VMEXIT_CR11_WRITE_TRAP		0x9B
#define SVM_VMEXIT_CR12_WRITE_TRAP		0x9C
#define SVM_VMEXIT_CR13_WRITE_TRAP		0x9D
#define SVM_VMEXIT_CR14_WRITE_TRAP		0x9E
#define SVM_VMEXIT_CR15_WRITE_TRAP		0x9F
#define SVM_VMEXIT_NPF				0x400
#define SVM_AVIC_INCOMPLETE_IPI			0x401
#define SVM_AVIC_NOACCEL			0x402
#define SVM_VMEXIT_VMGEXIT			0x403
#define SVM_VMEXIT_INVALID			-1

/* Exit reasons synthesised by the kernel for VMM_IOC_RUN */
#define VM_EXIT_TERMINATED			0xFFFE
#define VM_EXIT_NONE				0xFFFF

/*
 * Exception injection vectors (these correspond to the CPU exception types
 * defined in the SDM.)
 */
#define VMM_EX_DE	0	/* Divide Error #DE */
#define VMM_EX_DB	1	/* Debug Exception #DB */
#define VMM_EX_NMI	2	/* NMI */
#define VMM_EX_BP	3	/* Breakpoint #BP */
#define VMM_EX_OF	4	/* Overflow #OF */
#define VMM_EX_BR	5	/* Bound range exceeded #BR */
#define VMM_EX_UD	6	/* Undefined opcode #UD */
#define VMM_EX_NM	7	/* Device not available #NM */
#define VMM_EX_DF	8	/* Double fault #DF */
#define VMM_EX_CP	9	/* Coprocessor segment overrun (unused) */
#define VMM_EX_TS	10	/* Invalid TSS #TS */
#define VMM_EX_NP	11	/* Segment not present #NP */
#define VMM_EX_SS	12	/* Stack segment fault #SS */
#define VMM_EX_GP	13	/* General protection #GP */
#define VMM_EX_PF	14	/* Page fault #PF */
#define VMM_EX_MF	16	/* x87 FPU floating point error #MF */
#define VMM_EX_AC	17	/* Alignment check #AC */
#define VMM_EX_MC	18	/* Machine check #MC */
#define VMM_EX_XM	19	/* SIMD floating point exception #XM */
#define VMM_EX_VE	20	/* Virtualization exception #VE */

enum {
	VEI_DIR_OUT,
	VEI_DIR_IN
};

enum {
	VEE_FAULT_INVALID = 0,
	VEE_FAULT_HANDLED,
	VEE_FAULT_MMIO_ASSIST,
	VEE_FAULT_PROTECT,
};

enum {
	VMM_CPU_MODE_REAL,
	VMM_CPU_MODE_PROT,
	VMM_CPU_MODE_PROT32,
	VMM_CPU_MODE_COMPAT,
	VMM_CPU_MODE_LONG,
	VMM_CPU_MODE_UNKNOWN,
};

/*
 * vm exit data
 *  vm_exit_inout		: describes an IN/OUT exit
 */
struct vm_exit_inout {
	uint8_t			vei_size;	/* Size of access */
	uint8_t			vei_dir;	/* Direction */
	uint8_t			vei_rep;	/* REP prefix? */
	uint8_t			vei_string;	/* string variety? */
	uint8_t			vei_encoding;	/* operand encoding */
	uint16_t		vei_port;	/* port */
	uint32_t		vei_data;	/* data */
	uint8_t			vei_insn_len;	/* Count of instruction bytes */
};

/*
 *  vm_exit_eptviolation	: describes a nested page fault exit
 */
struct vm_exit_eptviolation {
	uint8_t		vee_fault_type;		/* type of vm exit */
	uint8_t		vee_insn_info;		/* bitfield */
#define VEE_LEN_VALID		0x1		/* vee_insn_len is valid */
#define VEE_BYTES_VALID		0x2		/* vee_insn_bytes is valid */
	uint8_t		vee_insn_len;		/* [VMX] instruction length */
	uint8_t		vee_insn_bytes[15];	/* [SVM] bytes at {R,E,}IP */
};

/*
 * struct vcpu_inject_event	: describes an exception or interrupt to inject.
 */
struct vcpu_inject_event {
	uint8_t		vie_vector;	/* Exception or interrupt vector. */
	uint32_t	vie_errorcode;	/* Optional error code. */
	uint8_t		vie_type;
#define VCPU_INJECT_NONE	0
#define VCPU_INJECT_INTR	1	/* External hardware interrupt. */
#define VCPU_INJECT_EX		2	/* HW or SW Exception */
#define VCPU_INJECT_NMI		3	/* Non-maskable Interrupt */
};

/*
 * struct vcpu_segment_info
 *
 * Describes a segment + selector set, used in constructing the initial vcpu
 * register content
 */
struct vcpu_segment_info {
	uint16_t	vsi_sel;
	uint32_t	vsi_limit;
	uint32_t	vsi_ar;
	uint64_t	vsi_base;
};

/* The GPRS are ordered to assist instruction decode. */
#define VCPU_REGS_RAX		0
#define VCPU_REGS_RCX		1
#define VCPU_REGS_RDX		2
#define VCPU_REGS_RBX		3
#define VCPU_REGS_RSP		4
#define VCPU_REGS_RBP		5
#define VCPU_REGS_RSI		6
#define VCPU_REGS_RDI		7
#define VCPU_REGS_R8		8
#define VCPU_REGS_R9		9
#define VCPU_REGS_R10		10
#define VCPU_REGS_R11		11
#define VCPU_REGS_R12		12
#define VCPU_REGS_R13		13
#define VCPU_REGS_R14		14
#define VCPU_REGS_R15		15
#define VCPU_REGS_RIP		16
#define VCPU_REGS_RFLAGS	17
#define VCPU_REGS_NGPRS		(VCPU_REGS_RFLAGS + 1)

#define VCPU_REGS_CR0		0
#define VCPU_REGS_CR2		1
#define VCPU_REGS_CR3		2
#define VCPU_REGS_CR4		3
#define VCPU_REGS_CR8		4
#define VCPU_REGS_XCR0		5
#define VCPU_REGS_PDPTE0 	6
#define VCPU_REGS_PDPTE1 	7
#define VCPU_REGS_PDPTE2 	8
#define VCPU_REGS_PDPTE3 	9
#define VCPU_REGS_NCRS		(VCPU_REGS_PDPTE3 + 1)

#define VCPU_REGS_ES		0
#define VCPU_REGS_CS		1
#define VCPU_REGS_SS		2
#define VCPU_REGS_DS		3
#define VCPU_REGS_FS		4
#define VCPU_REGS_GS		5
#define VCPU_REGS_LDTR		6
#define VCPU_REGS_TR		7
#define VCPU_REGS_NSREGS	(VCPU_REGS_TR + 1)

#define VCPU_REGS_EFER   	0
#define VCPU_REGS_STAR   	1
#define VCPU_REGS_LSTAR  	2
#define VCPU_REGS_CSTAR  	3
#define VCPU_REGS_SFMASK 	4
#define VCPU_REGS_KGSBASE	5
#define VCPU_REGS_MISC_ENABLE	6
#define VCPU_REGS_NMSRS		(VCPU_REGS_MISC_ENABLE + 1)

#define VCPU_REGS_DR0		0
#define VCPU_REGS_DR1		1
#define VCPU_REGS_DR2		2
#define VCPU_REGS_DR3		3
#define VCPU_REGS_DR6		4
#define VCPU_REGS_DR7		5
#define VCPU_REGS_NDRS		(VCPU_REGS_DR7 + 1)

struct vcpu_reg_state {
	uint64_t			vrs_gprs[VCPU_REGS_NGPRS];
	uint64_t			vrs_crs[VCPU_REGS_NCRS];
	uint64_t			vrs_msrs[VCPU_REGS_NMSRS];
	uint64_t			vrs_drs[VCPU_REGS_NDRS];
	struct vcpu_segment_info	vrs_sregs[VCPU_REGS_NSREGS];
	struct vcpu_segment_info	vrs_gdtr;
	struct vcpu_segment_info	vrs_idtr;
};

/*
 * struct vm_exit
 *
 * Contains VM exit information communicated to the userland monitor. This
 * information is gathered by ovmm(4) from the CPU on each exit that requires
 * help from userland.
 */
struct vm_exit {
	union {
		struct vm_exit_inout		vei;	/* IN/OUT exit */
		struct vm_exit_eptviolation	vee;	/* NPF exit */
	};

	struct vcpu_reg_state		vrs;
	int				cpl;
};

struct vm_intr_params {
	/* Input parameters to VMM_IOC_INTR */
	uint32_t		vip_vm_id;
	uint32_t		vip_vcpu_id;
	uint16_t		vip_intr;
};

#define VM_RWREGS_GPRS	0x1	/* read/write GPRs */
#define VM_RWREGS_SREGS	0x2	/* read/write segment registers */
#define VM_RWREGS_CRS	0x4	/* read/write CRs */
#define VM_RWREGS_MSRS	0x8	/* read/write MSRs */
#define VM_RWREGS_DRS	0x10	/* read/write DRs */
#define VM_RWREGS_ALL	(VM_RWREGS_GPRS | VM_RWREGS_SREGS | VM_RWREGS_CRS | \
    VM_RWREGS_MSRS | VM_RWREGS_DRS)

struct vm_rwregs_params {
	/*
	 * Input/output parameters to VMM_IOC_READREGS /
	 * VMM_IOC_WRITEREGS
	 */
	uint32_t		vrwp_vm_id;
	uint32_t		vrwp_vcpu_id;
	uint64_t		vrwp_mask;
	struct vcpu_reg_state	vrwp_regs;
};

/* IOCTL definitions */
#define VMM_IOC_INTR _IOW('V', 6, struct vm_intr_params) /* Intr pending */

#ifdef _KERNEL

#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/sx.h>

/* MSR bitmap manipulation macros */
#define SVM_MSRIDX(m)			((m) / 4)
#define SVM_MSRBIT_R(m)			(1 << (((m) % 4) * 2))
#define SVM_MSRBIT_W(m)			(1 << (((m) % 4) * 2 + 1))

enum {
	VMM_MODE_UNKNOWN,
	VMM_MODE_RVI
};

enum {
	VMM_MEM_TYPE_REGULAR,
	VMM_MEM_TYPE_MMIO,
	VMM_MEM_TYPE_UNKNOWN
};

/*
 * SVM feature bits from CPUID 0x8000000A %edx
 */
#define SVM_FEAT_NP		(1U << 0)	/* Nested paging (RVI) */
#define SVM_FEAT_LBRV		(1U << 1)
#define SVM_FEAT_SVML		(1U << 2)
#define SVM_FEAT_NRIPS		(1U << 3)	/* Next RIP save */
#define SVM_FEAT_TSCRATE	(1U << 4)
#define SVM_FEAT_VMCBCLEAN	(1U << 5)	/* VMCB clean bits */
#define SVM_FEAT_FLUSHBYASID	(1U << 6)
#define SVM_FEAT_DECODEASSIST	(1U << 7)

/*
 * VMCB TLB control values
 */
#define SVM_TLB_CONTROL_FLUSH_NONE	0
#define SVM_TLB_CONTROL_FLUSH_ALL	1
#define SVM_TLB_CONTROL_FLUSH_ASID	3
#define SVM_TLB_CONTROL_FLUSH_ASID_GLB	7

/*
 * VMCB clean bits
 */
#define SVM_CLEANBITS_I			(1 << 0)
#define SVM_CLEANBITS_IOPM		(1 << 1)
#define SVM_CLEANBITS_ASID		(1 << 2)
#define SVM_CLEANBITS_TPR		(1 << 3)
#define SVM_CLEANBITS_NP		(1 << 4)
#define SVM_CLEANBITS_CR		(1 << 5)
#define SVM_CLEANBITS_DR		(1 << 6)
#define SVM_CLEANBITS_DT		(1 << 7)
#define SVM_CLEANBITS_SEG		(1 << 8)
#define SVM_CLEANBITS_CR2		(1 << 9)
#define SVM_CLEANBITS_LBR		(1 << 10)
#define SVM_CLEANBITS_AVIC		(1 << 11)

#define SVM_CLEANBITS_ALL \
	(SVM_CLEANBITS_I | SVM_CLEANBITS_IOPM | SVM_CLEANBITS_ASID | \
	 SVM_CLEANBITS_TPR | SVM_CLEANBITS_NP | SVM_CLEANBITS_CR | \
	 SVM_CLEANBITS_DR | SVM_CLEANBITS_DT | SVM_CLEANBITS_SEG | \
	 SVM_CLEANBITS_CR2 | SVM_CLEANBITS_LBR | SVM_CLEANBITS_AVIC )

/* VMCB v_intr_misc: ignore TPR for virtual interrupt injection */
#define SVM_INTR_MISC_V_IGN_TPR		0x10

/*
 * VMCB intercept bits.  The first block is for v_cr_rw and v_dr_rw, the
 * second block is v_intercept1, the third block is v_intercept2.
 */
#define SVM_INTERCEPT_CR0_READ		(1UL << 0)
#define SVM_INTERCEPT_CR1_READ		(1UL << 1)
#define SVM_INTERCEPT_CR2_READ		(1UL << 2)
#define SVM_INTERCEPT_CR3_READ		(1UL << 2)
#define SVM_INTERCEPT_CR4_READ		(1UL << 4)
#define SVM_INTERCEPT_CR5_READ		(1UL << 5)
#define SVM_INTERCEPT_CR6_READ		(1UL << 6)
#define SVM_INTERCEPT_CR7_READ		(1UL << 7)
#define SVM_INTERCEPT_CR8_READ		(1UL << 8)
#define SVM_INTERCEPT_CR9_READ		(1UL << 9)
#define SVM_INTERCEPT_CR10_READ		(1UL << 10)
#define SVM_INTERCEPT_CR11_READ		(1UL << 11)
#define SVM_INTERCEPT_CR12_READ		(1UL << 12)
#define SVM_INTERCEPT_CR13_READ		(1UL << 13)
#define SVM_INTERCEPT_CR14_READ		(1UL << 14)
#define SVM_INTERCEPT_CR15_READ		(1UL << 15)
#define SVM_INTERCEPT_CR0_WRITE		(1UL << 16)
#define SVM_INTERCEPT_CR1_WRITE		(1UL << 17)
#define SVM_INTERCEPT_CR2_WRITE		(1UL << 18)
#define SVM_INTERCEPT_CR3_WRITE		(1UL << 19)
#define SVM_INTERCEPT_CR4_WRITE		(1UL << 20)
#define SVM_INTERCEPT_CR5_WRITE		(1UL << 21)
#define SVM_INTERCEPT_CR6_WRITE		(1UL << 22)
#define SVM_INTERCEPT_CR7_WRITE		(1UL << 23)
#define SVM_INTERCEPT_CR8_WRITE		(1UL << 24)
#define SVM_INTERCEPT_CR9_WRITE		(1UL << 25)
#define SVM_INTERCEPT_CR10_WRITE	(1UL << 26)
#define SVM_INTERCEPT_CR11_WRITE	(1UL << 27)
#define SVM_INTERCEPT_CR12_WRITE	(1UL << 28)
#define SVM_INTERCEPT_CR13_WRITE	(1UL << 29)
#define SVM_INTERCEPT_CR14_WRITE	(1UL << 30)
#define SVM_INTERCEPT_CR15_WRITE	(1UL << 31)
#define SVM_INTERCEPT_DR0_READ		(1UL << 0)
#define SVM_INTERCEPT_DR1_READ		(1UL << 1)
#define SVM_INTERCEPT_DR2_READ		(1UL << 2)
#define SVM_INTERCEPT_DR3_READ		(1UL << 2)
#define SVM_INTERCEPT_DR4_READ		(1UL << 4)
#define SVM_INTERCEPT_DR5_READ		(1UL << 5)
#define SVM_INTERCEPT_DR6_READ		(1UL << 6)
#define SVM_INTERCEPT_DR7_READ		(1UL << 7)
#define SVM_INTERCEPT_DR8_READ		(1UL << 8)
#define SVM_INTERCEPT_DR9_READ		(1UL << 9)
#define SVM_INTERCEPT_DR10_READ		(1UL << 10)
#define SVM_INTERCEPT_DR11_READ		(1UL << 11)
#define SVM_INTERCEPT_DR12_READ		(1UL << 12)
#define SVM_INTERCEPT_DR13_READ		(1UL << 13)
#define SVM_INTERCEPT_DR14_READ		(1UL << 14)
#define SVM_INTERCEPT_DR15_READ		(1UL << 15)
#define SVM_INTERCEPT_DR0_WRITE		(1UL << 16)
#define SVM_INTERCEPT_DR1_WRITE		(1UL << 17)
#define SVM_INTERCEPT_DR2_WRITE		(1UL << 18)
#define SVM_INTERCEPT_DR3_WRITE		(1UL << 19)
#define SVM_INTERCEPT_DR4_WRITE		(1UL << 20)
#define SVM_INTERCEPT_DR5_WRITE		(1UL << 21)
#define SVM_INTERCEPT_DR6_WRITE		(1UL << 22)
#define SVM_INTERCEPT_DR7_WRITE		(1UL << 23)
#define SVM_INTERCEPT_DR8_WRITE		(1UL << 24)
#define SVM_INTERCEPT_DR9_WRITE		(1UL << 25)
#define SVM_INTERCEPT_DR10_WRITE	(1UL << 26)
#define SVM_INTERCEPT_DR11_WRITE	(1UL << 27)
#define SVM_INTERCEPT_DR12_WRITE	(1UL << 28)
#define SVM_INTERCEPT_DR13_WRITE	(1UL << 29)
#define SVM_INTERCEPT_DR14_WRITE	(1UL << 30)
#define SVM_INTERCEPT_DR15_WRITE	(1UL << 31)
#define SVM_INTERCEPT_INTR		(1UL << 0)
#define SVM_INTERCEPT_NMI		(1UL << 1)
#define SVM_INTERCEPT_SMI		(1UL << 2)
#define SVM_INTERCEPT_INIT		(1UL << 3)
#define SVM_INTERCEPT_VINTR		(1UL << 4)
#define SVM_INTERCEPT_CR0_SEL_WRITE	(1UL << 5)
#define SVM_INTERCEPT_IDTR_READ		(1UL << 6)
#define SVM_INTERCEPT_GDTR_READ		(1UL << 7)
#define SVM_INTERCEPT_LDTR_READ		(1UL << 8)
#define SVM_INTERCEPT_TR_READ		(1UL << 9)
#define SVM_INTERCEPT_IDTR_WRITE	(1UL << 10)
#define SVM_INTERCEPT_GDTR_WRITE	(1UL << 11)
#define SVM_INTERCEPT_LDTR_WRITE	(1UL << 12)
#define SVM_INTERCEPT_TR_WRITE		(1UL << 13)
#define SVM_INTERCEPT_RDTSC		(1UL << 14)
#define SVM_INTERCEPT_RDPMC		(1UL << 15)
#define SVM_INTERCEPT_PUSHF		(1UL << 16)
#define SVM_INTERCEPT_POPF		(1UL << 17)
#define SVM_INTERCEPT_CPUID		(1UL << 18)
#define SVM_INTERCEPT_RSM		(1UL << 19)
#define SVM_INTERCEPT_IRET		(1UL << 20)
#define SVM_INTERCEPT_INTN		(1UL << 21)
#define SVM_INTERCEPT_INVD		(1UL << 22)
#define SVM_INTERCEPT_PAUSE		(1UL << 23)
#define SVM_INTERCEPT_HLT		(1UL << 24)
#define SVM_INTERCEPT_INVLPG		(1UL << 25)
#define SVM_INTERCEPT_INVLPGA		(1UL << 26)
#define SVM_INTERCEPT_INOUT		(1UL << 27)
#define SVM_INTERCEPT_MSR		(1UL << 28)
#define SVM_INTERCEPT_TASK_SWITCH	(1UL << 29)
#define SVM_INTERCEPT_FERR_FREEZE	(1UL << 30)
#define SVM_INTERCEPT_SHUTDOWN		(1UL << 31)
#define SVM_INTERCEPT_VMRUN		(1UL << 0)
#define SVM_INTERCEPT_VMMCALL		(1UL << 1)
#define SVM_INTERCEPT_VMLOAD		(1UL << 2)
#define SVM_INTERCEPT_VMSAVE		(1UL << 3)
#define SVM_INTERCEPT_STGI		(1UL << 4)
#define SVM_INTERCEPT_CLGI		(1UL << 5)
#define SVM_INTERCEPT_SKINIT		(1UL << 6)
#define SVM_INTERCEPT_RDTSCP		(1UL << 7)
#define SVM_INTERCEPT_ICEBP		(1UL << 8)
#define SVM_INTERCEPT_WBINVD		(1UL << 9)
#define SVM_INTERCEPT_MONITOR		(1UL << 10)
#define SVM_INTERCEPT_MWAIT_UNCOND	(1UL << 11)
#define SVM_INTERCEPT_MWAIT_COND	(1UL << 12)
#define SVM_INTERCEPT_XSETBV		(1UL << 13)
#define SVM_INTERCEPT_EFER_WRITE	(1UL << 15)
#define SVM_INTERCEPT_CR0_WRITE_POST	(1UL << 16)
#define SVM_INTERCEPT_CR1_WRITE_POST	(1UL << 17)
#define SVM_INTERCEPT_CR2_WRITE_POST	(1UL << 18)
#define SVM_INTERCEPT_CR3_WRITE_POST	(1UL << 19)
#define SVM_INTERCEPT_CR4_WRITE_POST	(1UL << 20)
#define SVM_INTERCEPT_CR5_WRITE_POST	(1UL << 21)
#define SVM_INTERCEPT_CR6_WRITE_POST	(1UL << 22)
#define SVM_INTERCEPT_CR7_WRITE_POST	(1UL << 23)

/* Forward declarations */
struct vm;
struct vm_create_params;

/*
 * Implementation-specific cpu state
 */
struct vmcb_segment {
	uint16_t 			vs_sel;			/* 000h */
	uint16_t 			vs_attr;		/* 002h */
	uint32_t			vs_lim;			/* 004h */
	uint64_t			vs_base;		/* 008h */
};

#define SVM_ENABLE_NP		(1ULL << 0)
#define SVM_ENABLE_SEV		(1ULL << 1)
#define SVM_SEVES_ENABLE	(1ULL << 2)
#define SMV_GUEST_INTR_MASK	(1ULL << 1)
#define SVM_LBRVIRT_ENABLE	(1ULL << 0)

struct vmcb {
	union {
		struct {
			uint32_t	v_cr_rw;		/* 000h */
			uint32_t	v_dr_rw;		/* 004h */
			uint32_t	v_excp;			/* 008h */
			uint32_t	v_intercept1;		/* 00Ch */
			uint32_t	v_intercept2;		/* 010h */
			uint8_t		v_pad1[0x28];		/* 014h-03Bh */
			uint16_t	v_pause_thr;		/* 03Ch */
			uint16_t	v_pause_ct;		/* 03Eh */
			uint64_t	v_iopm_pa;		/* 040h */
			uint64_t	v_msrpm_pa;		/* 048h */
			uint64_t	v_tsc_offset;		/* 050h */
			uint32_t	v_asid;			/* 058h */
			uint8_t		v_tlb_control;		/* 05Ch */
			uint8_t		v_pad2[0x3];		/* 05Dh-05Fh */
			uint8_t		v_tpr;			/* 060h */
			uint8_t		v_irq;			/* 061h */
			uint8_t		v_intr_misc;		/* 062h */
			uint8_t		v_intr_masking;		/* 063h */
			uint8_t		v_intr_vector;		/* 064h */
			uint8_t		v_pad3[0x3];		/* 065h-067h */
			uint64_t	v_intr_shadow;		/* 068h */
			uint64_t	v_exitcode;		/* 070h */
			uint64_t	v_exitinfo1;		/* 078h */
			uint64_t	v_exitinfo2;		/* 080h */
			uint64_t	v_exitintinfo;		/* 088h */
			uint64_t	v_np_enable;		/* 090h */
			uint64_t	v_avic_apic_bar;	/* 098h */
			uint64_t	v_ghcb_gpa;		/* 0A0h */
			uint64_t	v_eventinj;		/* 0A8h */
			uint64_t	v_n_cr3;		/* 0B0h */
			uint64_t	v_lbr_virt_enable;	/* 0B8h */
			uint64_t	v_vmcb_clean_bits;	/* 0C0h */
			uint64_t	v_nrip;			/* 0C8h */
			uint8_t		v_n_bytes_fetched;	/* 0D0h */
			uint8_t		v_guest_ins_bytes[0xf];	/* 0D1h-0DFh */
			uint64_t	v_avic_apic_back_page;	/* 0E0h */
			uint64_t	v_pad5;			/* 0E8h-0EFh */
			uint64_t	v_avic_logical_table;	/* 0F0h */
			uint64_t	v_avic_phys;		/* 0F8h */
			uint64_t	v_pad12;		/* 100h */
			uint64_t	v_vmsa_pa;		/* 108h */
		};
		uint8_t			vmcb_control[0x400];
	};

	union {
		struct {
			/* Offsets here are relative to start of VMCB SSA */
			struct vmcb_segment	v_es;		/* 000h */
			struct vmcb_segment	v_cs;		/* 010h */
			struct vmcb_segment	v_ss;		/* 020h */
			struct vmcb_segment	v_ds;		/* 030h */
			struct vmcb_segment	v_fs;		/* 040h */
			struct vmcb_segment	v_gs;		/* 050h */
			struct vmcb_segment	v_gdtr;		/* 060h */
			struct vmcb_segment	v_ldtr;		/* 070h */
			struct vmcb_segment	v_idtr;		/* 080h */
			struct vmcb_segment	v_tr;		/* 090h */
			uint8_t 		v_pad6[0x2B];	/* 0A0h-0CAh */
			uint8_t			v_cpl;		/* 0CBh */
			uint32_t		v_pad7;		/* 0CCh-0CFh */
			uint64_t		v_efer;		/* 0D0h */
			uint8_t			v_pad8[0x70];	/* 0D8h-147h */
			uint64_t		v_cr4;		/* 148h */
			uint64_t		v_cr3;		/* 150h */
			uint64_t		v_cr0;		/* 158h */
			uint64_t		v_dr7;		/* 160h */
			uint64_t		v_dr6;		/* 168h */
			uint64_t		v_rflags;	/* 170h */
			uint64_t		v_rip;		/* 178h */
			uint64_t		v_pad9[0xB];	/* 180h-1D7h */
			uint64_t		v_rsp;		/* 1D8h */
			uint64_t		v_pad10[0x3];	/* 1E0h-1F7h */
			uint64_t		v_rax;		/* 1F8h */
			uint64_t		v_star;		/* 200h */
			uint64_t		v_lstar;	/* 208h */
			uint64_t		v_cstar;	/* 210h */
			uint64_t		v_sfmask;	/* 218h */
			uint64_t		v_kgsbase;	/* 220h */
			uint64_t		v_sysenter_cs;	/* 228h */
			uint64_t		v_sysenter_esp;	/* 230h */
			uint64_t		v_sysenter_eip;	/* 238h */
			uint64_t		v_cr2;		/* 240h */
			uint64_t		v_pad11[0x4];	/* 248h-267h */
			uint64_t		v_g_pat;	/* 268h */
			uint64_t		v_dbgctl;	/* 270h */
			uint64_t		v_br_from;	/* 278h */
			uint64_t		v_br_to;	/* 280h */
			uint64_t		v_lastexcpfrom;	/* 288h */
			uint64_t		v_lastexcpto;	/* 290h */
		};

		uint8_t				vmcb_layout[PAGE_SIZE - 0x400];
	};
};

/*
 * Storage for guest registers not preserved in the VMCB and various exit
 * information.
 *
 * Note that svm_enter_guest depends on the layout of this struct for
 * field access (see ovmm_support.S).
 */
struct vcpu_gueststate {
	/* %rsi should be first */
	uint64_t	vg_rsi;			/* 0x00 */
	uint64_t	vg_rax;			/* 0x08 */
	uint64_t	vg_rbx;			/* 0x10 */
	uint64_t	vg_rcx;			/* 0x18 */
	uint64_t	vg_rdx;			/* 0x20 */
	uint64_t	vg_rdi;			/* 0x28 */
	uint64_t	vg_rbp;			/* 0x30 */
	uint64_t	vg_r8;			/* 0x38 */
	uint64_t	vg_r9;			/* 0x40 */
	uint64_t	vg_r10;			/* 0x48 */
	uint64_t	vg_r11;			/* 0x50 */
	uint64_t	vg_r12;			/* 0x58 */
	uint64_t	vg_r13;			/* 0x60 */
	uint64_t	vg_r14;			/* 0x68 */
	uint64_t	vg_r15;			/* 0x70 */
	uint64_t	vg_cr2;			/* 0x78 */
	uint64_t	vg_rip;			/* 0x80 */
	uint32_t	vg_exit_reason;		/* 0x88 */
	uint64_t	vg_rflags;		/* 0x90 */
	uint64_t	vg_xcr0;		/* 0x98 */
	/*
	 * Debug registers
	 * - %dr4/%dr5 are aliased to %dr6/%dr7 (or cause #DE)
	 * - %dr7 is saved automatically in the VMCB
	 */
	uint64_t	vg_dr0;			/* 0xa0 */
	uint64_t	vg_dr1;			/* 0xa8 */
	uint64_t	vg_dr2;			/* 0xb0 */
	uint64_t	vg_dr3;			/* 0xb8 */
	uint64_t	vg_dr6;			/* 0xc0 */
};

/*
 * Virtual CPU
 *
 * Methods used to vcpu struct members:
 *	a	atomic operations
 *	I	immutable after creation
 *	v	vcpu lock (vc_lock)
 *	V	vm struct's vcpu list lock (vm_vcpu_lock)
 */
struct vcpu {
	/* Guest FPU state, allocated with fpu_save_area_alloc(9). */
	struct savefpu *vc_g_fpu;		/* [v] */

	/* VMCB pointer */
	void *vc_control_va;			/* [I] */
	vm_paddr_t vc_control_pa;		/* [I] */

	/* MSR bitmap address (2 pages) */
	void *vc_msr_bitmap_va;			/* [I] */
	vm_paddr_t vc_msr_bitmap_pa;		/* [I] */

	/* I/O permission bitmap (3 pages) */
	void *vc_svm_ioio_va;			/* [I] */
	vm_paddr_t vc_svm_ioio_pa;		/* [I] */

	struct vm *vc_parent;			/* [I] */
	uint32_t vc_id;				/* [I] */
	uint16_t vc_vpid;			/* [I] ASID */
	volatile u_int vc_state;		/* [a] */
	SLIST_ENTRY(vcpu) vc_vcpu_link;		/* [V] */

	uint8_t vc_virt_mode;			/* [I] */

	struct sx vc_lock;
	volatile int vc_curcpu;			/* [a] host cpu or NOCPU */
	int vc_last_pcpu;			/* [v] */
	long vc_eptgen;				/* [v] nested pmap generation */
	struct vm_exit vc_exit;			/* [v] */

	uint16_t vc_intr;			/* [v] */
	uint8_t vc_irqready;			/* [v] */

	uint8_t vc_fpuinited;			/* [v] */
	struct vcpu_gueststate vc_gueststate;	/* [v] */

	struct vcpu_inject_event vc_inject;	/* [v] */

	uint32_t vc_pvclock_version;		/* [v] */
	uint64_t vc_pvclock_system_gpa;		/* [v] */
	uint32_t vc_pvclock_system_tsc_mul;	/* [v] */

	/* Shadowed MSRs */
	uint64_t vc_shadow_pat;			/* [v] */
};

SLIST_HEAD(vcpu_head, vcpu);

/* Machine dependent part of the softc. */
struct vmm_softc_md {
	uint32_t	svm_features;	/* [I] SVM_FEAT_* */
	uint32_t	svm_nasid;	/* [I] number of ASIDs */
	uint64_t	host_xcr0;	/* [I] */
	uint8_t		pkru_enabled;	/* [I] */
	void		*hsave_va;	/* [I] per-cpu host save areas */
	vm_paddr_t	hsave_pa;	/* [I] */
	size_t		hsave_size;	/* [I] */
	volatile int	vmm_on;		/* [I under sc_slock] SVME enabled? */
};

struct vm_run_params;
struct vm_rwregs_params;
struct vm_rwvmparams_params;
struct thread;

int	svm_enter_guest(uint64_t, struct vcpu_gueststate *);

int	ovmm_probe_machdep(struct vmm_softc_md *);
void	ovmm_deinit_machdep(struct vmm_softc_md *);
void	ovmm_start(void);
void	ovmm_stop(void);
int	ovmm_ioctl_machdep(u_long, caddr_t, int, struct thread *);
int	vm_impl_init(struct vm *);
void	vm_impl_deinit(struct vm *);
int	vcpu_init(struct vcpu *, struct vm_create_params *);
void	vcpu_deinit(struct vcpu *);
int	vm_rwregs(struct vm_rwregs_params *, int);
int	vm_rwvmparams(struct vm_rwvmparams_params *, int);
int	vcpu_reset_regs(struct vcpu *, struct vcpu_reg_state *);
int	vm_run(struct vm_run_params *);
int	vm_intr_pending(struct vm_intr_params *);
const char *svm_exit_reason_decode(uint32_t);

#endif /* _KERNEL */
#endif	/* ! _DEV_OVMM_OVMM_SVM_H_ */
