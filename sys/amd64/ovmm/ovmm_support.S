/*
 * Copyright (c) 2014-2015 Mike Larkin <mlarkin@openbsd.org>
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
 * ovmm(4) low level guest entry for AMD SVM, ported from OpenBSD's
 * vmm_support.S.  Differences from the original:
 *
 *  - The caller is responsible for CLGI/STGI; this function must be
 *    entered with GIF cleared.
 *  - The host TR is restored by the C code (ovmm_restore_host_tss()),
 *    the original manipulated the GDT directly here.
 *  - The return stack buffer is overwritten after #VMEXIT like bhyve does.
 */

#include <machine/asmacros.h>
#include <machine/specialreg.h>

	.text

/*
 * int svm_enter_guest(uint64_t vmcb_pa, struct vcpu_gueststate *gs)
 *
 * %rdi: physical address of the VMCB
 * %rsi: pointer to the guest register state
 *
 * Returns 0 on a normal #VMEXIT.  The stack layout below mirrors the
 * offsets of struct vcpu_gueststate (see ovmm_svm.h).
 */
ENTRY(svm_enter_guest)
	movq	%rdi, %r8
	pushfq

	/*
	 * Save (possibly) lazy-switched selectors
	 */
	movw	%es, %ax
	pushw	%ax
	movw	%ds, %ax
	pushw	%ax
	movw	%ss, %ax
	pushw	%ax

	movq	$MSR_FSBASE, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx
	pushw	%fs
	movq	$MSR_GSBASE, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx
	pushw	%gs
	movq	$MSR_KGSBASE, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx

	/*
	 * Save various MSRs
	 */
	movq	$MSR_STAR, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx

	movq	$MSR_LSTAR, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx

	movq	$MSR_CSTAR, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx

	movq	$MSR_SF_MASK, %rcx
	rdmsr
	pushq	%rax
	pushq	%rdx

	/* Preserve callee-preserved registers as per AMD64 ABI */
	pushq	%r15
	pushq	%r14
	pushq	%r13
	pushq	%r12
	pushq	%rbp
	pushq	%rbx
	pushq	%rsi		/* Guest Regs Pointer */

	/* Restore guest registers */
	movq	%r8, %rax	/* rax = vmcb pa */
	movq	0xa0(%rsi), %r8
	movq	%r8, %dr0
	movq	0xa8(%rsi), %r8
	movq	%r8, %dr1
	movq	0xb0(%rsi), %r8
	movq	%r8, %dr2
	movq	0xb8(%rsi), %r8
	movq	%r8, %dr3
	/* %dr6 is saved in the VMCB */
	movq	0x78(%rsi), %r8
	movq	%r8, %cr2
	movq	0x70(%rsi), %r15
	movq	0x68(%rsi), %r14
	movq	0x60(%rsi), %r13
	movq	0x58(%rsi), %r12
	movq	0x50(%rsi), %r11
	movq	0x48(%rsi), %r10
	movq	0x40(%rsi), %r9
	movq	0x38(%rsi), %r8
	movq	0x30(%rsi), %rbp
	movq	0x28(%rsi), %rdi
	movq	0x20(%rsi), %rdx
	movq	0x18(%rsi), %rcx
	movq	0x10(%rsi), %rbx
	/* %rax at 0x08(%rsi) is not needed in SVM */
	movq	0x00(%rsi), %rsi

	vmload	%rax
	vmrun	%rax
	vmsave	%rax

	/* Preserve guest registers not saved in VMCB */
	pushq	%rsi
	pushq	%rdi
	movq	0x10(%rsp), %rdi
	movq	0x8(%rsp), %rsi
	movq	%rsi, (%rdi)
	popq	%rdi
	popq	%rsi	/* discard */

	popq	%rsi
	/* %rax at 0x08(%rsi) is not needed in SVM */
	movq	%rbx, 0x10(%rsi)
	movq	%rcx, 0x18(%rsi)
	movq	%rdx, 0x20(%rsi)
	movq	%rdi, 0x28(%rsi)
	movq	%rbp, 0x30(%rsi)
	movq	%r8, 0x38(%rsi)
	movq	%r9, 0x40(%rsi)
	movq	%r10, 0x48(%rsi)
	movq	%r11, 0x50(%rsi)
	movq	%r12, 0x58(%rsi)
	movq	%r13, 0x60(%rsi)
	movq	%r14, 0x68(%rsi)
	movq	%r15, 0x70(%rsi)
	movq	%cr2, %rax
	movq	%rax, 0x78(%rsi)
	movq	%dr0, %rax
	movq	%rax, 0xa0(%rsi)
	movq	%dr1, %rax
	movq	%rax, 0xa8(%rsi)
	movq	%dr2, %rax
	movq	%rax, 0xb0(%rsi)
	movq	%dr3, %rax
	movq	%rax, 0xb8(%rsi)
	/* %dr6 is saved in the VMCB */

	/*
	 * To prevent malicious branch target predictions from
	 * affecting the host, overwrite all entries in the RSB upon
	 * exiting a guest.
	 */
	mov	$16, %ecx	/* 16 iterations, two calls per loop */
	mov	%rsp, %rax
0:	call	2f		/* create an RSB entry. */
1:	pause
	call	1b		/* capture rogue speculation. */
2:	call	2f		/* create an RSB entry. */
1:	pause
	call	1b		/* capture rogue speculation. */
2:	sub	$1, %ecx
	jnz	0b
	mov	%rax, %rsp

	/* %rdi = 0 means we took an exit */
	xorq	%rdi, %rdi

	popq	%rbx
	popq	%rbp
	popq	%r12
	popq	%r13
	popq	%r14
	popq	%r15

	/*
	 * Restore saved MSRs
	 */
	popq	%rdx
	popq	%rax
	movq	$MSR_SF_MASK, %rcx
	wrmsr

	popq	%rdx
	popq	%rax
	movq	$MSR_CSTAR, %rcx
	wrmsr

	popq	%rdx
	popq	%rax
	movq	$MSR_LSTAR, %rcx
	wrmsr

	popq	%rdx
	popq	%rax
	movq	$MSR_STAR, %rcx
	wrmsr

	/*
	 * popw %gs will reset gsbase to 0, so preserve it
	 * first. This is to accommodate possibly lazy-switched
	 * selectors from above
	 */
	cli
	popq	%rdx
	popq	%rax
	movq	$MSR_KGSBASE, %rcx
	wrmsr

	popw	%gs
	popq	%rdx
	popq	%rax
	movq	$MSR_GSBASE, %rcx
	wrmsr

	popw	%fs
	popq	%rdx
	popq	%rax
	movq	$MSR_FSBASE, %rcx
	wrmsr

	popw	%ax
	movw	%ax, %ss
	popw	%ax
	movw	%ax, %ds
	popw	%ax
	movw	%ax, %es

	xorq	%rax, %rax
	lldtw	%ax		/* Host LDT is always 0 */

	popfq

	movq	%rdi, %rax
	ret
END(svm_enter_guest)

	.section .note.GNU-stack,"",%progbits
