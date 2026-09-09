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
 * ovmm(4): AMD SVM machine dependent code, ported from OpenBSD's
 * sys/arch/amd64/amd64/vmm_machdep.c.
 *
 * Notable differences from the original:
 *
 *  - Nested page tables are a FreeBSD pmap of type PT_RVI inside a private
 *    vmspace.  Guest memory objects are mapped into that vmspace at their
 *    guest physical address, so a nested page fault is resolved with
 *    vm_fault(9) exactly like bhyve does.  The pmap's pm_eptgen generation
 *    counter tells us when the host invalidated guest mappings and the
 *    guest TLB must be flushed.
 *  - Guest FPU state lives in a savefpu allocated with fpu_save_area_alloc(9).
 *    It is loaded and saved around every VMRUN while GIF is clear.
 *  - SVM is enabled on all CPUs with smp_rendezvous(9) when the first VM
 *    is created and disabled again when the last VM goes away.
 *  - The host TSS is restored after #VMEXIT like bhyve does.
 *  - No VMX, no SEV/SEV-ES, no suspend/resume support.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/proc.h>
#include <sys/refcount.h>
#include <sys/signalvar.h>
#include <sys/smp.h>
#include <sys/smr.h>
#include <sys/pcpu.h>
#include <sys/time.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_page.h>
#include <vm/vm_extern.h>

#include <machine/cpufunc.h>
#include <machine/specialreg.h>
#include <machine/md_var.h>
#include <machine/fpu.h>
#include <machine/pcb.h>
#include <machine/segments.h>
#include <machine/smp.h>
#include <machine/vmparam.h>
#include <machine/clock.h>
#include <x86/apicvar.h>
#include <x86/cputypes.h>
#include <x86/pvclock.h>
#include <x86/kvm.h>

#include <dev/ovmm/ovmm.h>

/* MSRs not (yet) named in <machine/specialreg.h>. */
#define	OVMM_MSR_PSTATE0	0xc0010064	/* P-state 0 definition */
#define	OVMM_MSR_SEV_STATUS	0xc0010131	/* SEV status */
#define	OVMM_MSR_INT_PEN_MSG	0xc0010055	/* Interrupt pending message */
#define	OVMM_DE_CFG_SERIALIZE_LFENCE	0x2	/* LFENCE is serializing */

#define	OVMM_CPUID_STDEXT_PCOMMIT	0x00400000
#define	OVMM_XSAVE_XSAVEOPT		0x01
#define	OVMM_XSAVE_XGETBV1		0x04
#define	OVMM_KVM_FEATURE_NOP_IO_DELAY	0x00000002

/*
 * CPUID masks
 *
 * Clone host capabilities minus features that cannot be virtualised
 * (or that we do not want to expose).  See the OpenBSD vmmvar.h for the
 * rationale behind every bit.
 */
#define VMM_CPUIDECX_MASK ~(CPUID2_EST | CPUID2_TM2 | CPUID2_MON | \
    CPUID2_PDCM | CPUID2_VMX | CPUID2_DTES64 | \
    CPUID2_DS_CPL | CPUID2_SMX | CPUID2_CNXTID | \
    CPUID2_SDBG | CPUID2_XTPR | CPUID2_PCID | \
    CPUID2_DCA | CPUID2_X2APIC | CPUID2_TSCDLT)
#define VMM_ECPUIDECX_MASK ~(AMDID2_SVM | AMDID2_MWAITX)
#define VMM_CPUIDEDX_MASK ~(CPUID_ACPI | CPUID_TM | \
    CPUID_HTT | CPUID_DS | CPUID_APIC | \
    CPUID_PSN | CPUID_SS | CPUID_PBE | \
    CPUID_MTRR | CPUID_MCE | CPUID_MCA)
#define VMM_AMDSPEC_EBX_MASK ~(AMDFEID_IBPB | AMDFEID_IBRS | \
    AMDFEID_STIBP | AMDFEID_IBRS_ALWAYSON | AMDFEID_STIBP_ALWAYSON | \
    AMDFEID_PREFER_IBRS | AMDFEID_SSBD | AMDFEID_VIRT_SSBD | \
    AMDFEID_SSB_NO)
/* This mask is an include list for bits we want to expose */
#define VMM_APMI_EDX_INCLUDE_MASK (AMDPM_TSC_INVARIANT)
#define VMM_SEFF0EBX_MASK ~(CPUID_STDEXT_TSC_ADJUST | CPUID_STDEXT_SGX | \
    CPUID_STDEXT_HLE | CPUID_STDEXT_INVPCID | \
    CPUID_STDEXT_RTM | CPUID_STDEXT_PQM | CPUID_STDEXT_MPX | \
    OVMM_CPUID_STDEXT_PCOMMIT | CPUID_STDEXT_PROCTRACE | \
    CPUID_STDEXT_AVX512F | CPUID_STDEXT_AVX512DQ | \
    CPUID_STDEXT_AVX512IFMA | CPUID_STDEXT_AVX512PF | \
    CPUID_STDEXT_AVX512ER | CPUID_STDEXT_AVX512CD | \
    CPUID_STDEXT_AVX512BW | CPUID_STDEXT_AVX512VL)
/* ECX mask contains the bits to include */
#define VMM_SEFF0ECX_MASK (CPUID_STDEXT2_UMIP)
/* EDX mask contains the bits to include */
#define VMM_SEFF0EDX_MASK (CPUID_STDEXT3_MD_CLEAR)
/* Extended function flags - copy from host minus RDTSCP */
#define VMM_FEAT_EFLAGS_MASK ~(AMDID_RDTSCP)
#define VMM_CPUID4_CACHE_TOPOLOGY_MASK	0x3FF

static const char *vmm_hv_signature = VMM_HV_SIGNATURE;
static const char *kvm_hv_signature = "KVMKVMKVM\0\0\0";

DPCPU_DEFINE_STATIC(struct vcpu *, ovmm_last_vcpu);

/*
 * IPI vector used to force a running vcpu out of the guest, both by
 * vm_intr_pending() and by the pmap layer when it invalidates nested page
 * table entries (see pmap_invalidate_ept()).  Falls back to IPI_AST when no
 * vector can be allocated.
 */
static int ovmm_ipinum = IPI_AST;

static void
ovmm_justreturn(struct trapframe *tf __unused)
{
}

/* Make sure ovmm_support.S and struct vcpu_gueststate agree. */
CTASSERT(offsetof(struct vcpu_gueststate, vg_rsi) == 0x00);
CTASSERT(offsetof(struct vcpu_gueststate, vg_rbx) == 0x10);
CTASSERT(offsetof(struct vcpu_gueststate, vg_r15) == 0x70);
CTASSERT(offsetof(struct vcpu_gueststate, vg_cr2) == 0x78);
CTASSERT(offsetof(struct vcpu_gueststate, vg_dr0) == 0xa0);
CTASSERT(offsetof(struct vcpu_gueststate, vg_dr6) == 0xc0);
CTASSERT(sizeof(struct vmcb) == PAGE_SIZE);

static int vcpu_readregs_svm(struct vcpu *, uint64_t, struct vcpu_reg_state *);
static int vcpu_writeregs_svm(struct vcpu *, uint64_t,
    struct vcpu_reg_state *);
static int vcpu_reset_regs_svm(struct vcpu *, struct vcpu_reg_state *);
static int vcpu_init_svm(struct vcpu *, struct vm_create_params *);
static void vcpu_deinit_svm(struct vcpu *);
static int vcpu_run_svm(struct vcpu *, struct vm_run_params *);
static int svm_handle_exit(struct vcpu *);
static int svm_handle_msr(struct vcpu *);
static int svm_handle_xsetbv(struct vcpu *);
static int vmm_handle_xsetbv(struct vcpu *, uint64_t *);
static int vmm_handle_cpuid(struct vcpu *);
static int svm_handle_inout(struct vcpu *);
static int svm_handle_hlt(struct vcpu *);
static int svm_handle_np_fault(struct vcpu *);
static int svm_fault_page(struct vcpu *, vm_paddr_t);
static void vmm_inject_ud(struct vcpu *);
static void vmm_inject_gp(struct vcpu *);
static void vmm_inject_db(struct vcpu *);
static int vmm_get_guest_memtype(struct vm *, vm_paddr_t);
static int vmm_get_guest_cpu_cpl(struct vcpu *);
static int vmm_get_guest_cpu_mode(struct vcpu *);
static int vmm_alloc_asid(uint16_t *);
static void vmm_free_asid(uint16_t);
static void svm_setmsrbr(struct vcpu *, uint32_t);
static void svm_setmsrbw(struct vcpu *, uint32_t);
static void svm_setmsrbrw(struct vcpu *, uint32_t);
static void svm_set_clean(struct vcpu *, uint32_t);
static void svm_set_dirty(struct vcpu *, uint32_t);
static int vmm_gpa_is_valid(struct vcpu *, vm_paddr_t, size_t);
static void vmm_init_pvclock(struct vcpu *, vm_paddr_t);
static int vmm_update_pvclock(struct vcpu *);
static void vmm_pv_wall_clock(struct vcpu *, vm_paddr_t);
static int vmm_pat_is_valid(uint64_t);

static __inline void
ovmm_clgi(void)
{
	__asm __volatile("clgi");
}

static __inline void
ovmm_stgi(void)
{
	__asm __volatile("stgi");
}

static __inline int
ovmm_flush_by_asid(void)
{
	return ((vmm_softc->sc_md.svm_features & SVM_FEAT_FLUSHBYASID) != 0);
}

static __inline int
ovmm_decode_assist(void)
{
	return ((vmm_softc->sc_md.svm_features & SVM_FEAT_DECODEASSIST) != 0);
}

static __inline int
ovmm_vmcb_clean(void)
{
	return ((vmm_softc->sc_md.svm_features & SVM_FEAT_VMCBCLEAN) != 0);
}

static __inline pmap_t
ovmm_pmap(struct vm *vm)
{
	return (vmspace_pmap(vm->vm_vmspace));
}

/*
 * Allocate physically contiguous, page aligned, zeroed kernel memory.
 */
static void *
ovmm_contigalloc(size_t size, vm_paddr_t *pa)
{
	void *va;

	va = contigmalloc(size, M_OVMM, M_WAITOK | M_ZERO, 0, ~(vm_paddr_t)0,
	    PAGE_SIZE, 0);
	if (va == NULL)
		return (NULL);
	*pa = vtophys(va);
	return (va);
}

static void
ovmm_contigfree(void *va, size_t size)
{
	if (va != NULL)
		contigfree(va, size, M_OVMM);
}

/*
 * ovmm_probe_machdep
 *
 * Check that the CPU supports what we need and gather SVM capabilities.
 *
 * Return values:
 *  0: SVM with nested paging is available
 *  ENXIO: unsupported CPU or feature missing
 *  EBUSY: SVM is already enabled (another hypervisor, e.g. vmm(4), is active)
 *  ENOMEM: could not allocate host save areas
 */
int
ovmm_probe_machdep(struct vmm_softc_md *md)
{
	u_int regs[4];

	if (cpu_vendor_id != CPU_VENDOR_AMD && cpu_vendor_id != CPU_VENDOR_HYGON) {
		printf("ovmm: not an AMD processor\n");
		return (ENXIO);
	}

	/* Section 15.4 Enabling SVM from APM2. */
	if ((amd_feature2 & AMDID2_SVM) == 0) {
		printf("ovmm: SVM not available\n");
		return (ENXIO);
	}

	if (rdmsr(MSR_VM_CR) & VM_CR_SVMDIS) {
		printf("ovmm: SVM disabled by BIOS\n");
		return (ENXIO);
	}

	if (cpu_exthigh < 0x8000000a) {
		printf("ovmm: CPUID leaf 0x8000000a not available\n");
		return (ENXIO);
	}

	do_cpuid(0x8000000a, regs);
	md->svm_features = regs[3];
	md->svm_nasid = regs[1];

	if ((md->svm_features & SVM_FEAT_NP) == 0) {
		printf("ovmm: nested paging (RVI) not available\n");
		return (ENXIO);
	}

	if (md->svm_nasid < 2) {
		printf("ovmm: insufficient ASIDs (%u)\n", md->svm_nasid);
		return (ENXIO);
	}

	if (rdmsr(MSR_EFER) & EFER_SVM) {
		printf("ovmm: SVM already enabled, is vmm(4) loaded?\n");
		return (EBUSY);
	}

	md->host_xcr0 = (rcr4() & CR4_XSAVE) ? rxcr(0) : 0;
	md->pkru_enabled = (rcr4() & CR4_PKE) != 0;

	/* One host save area page per CPU, see vmm_start(). */
	md->hsave_size = (size_t)(mp_maxid + 1) * PAGE_SIZE;
	md->hsave_va = ovmm_contigalloc(md->hsave_size, &md->hsave_pa);
	if (md->hsave_va == NULL) {
		printf("ovmm: cannot allocate host save areas\n");
		return (ENOMEM);
	}

	md->vmm_on = 0;

	ovmm_ipinum = lapic_ipi_alloc(pti ? &IDTVEC(justreturn1_pti) :
	    &IDTVEC(justreturn), ovmm_justreturn);
	if (ovmm_ipinum < 0)
		ovmm_ipinum = IPI_AST;

	printf("ovmm: AMD SVM/RVI, %u ASIDs, features 0x%x%s%s%s%s\n",
	    md->svm_nasid, md->svm_features,
	    (md->svm_features & SVM_FEAT_NRIPS) ? " nrips" : "",
	    (md->svm_features & SVM_FEAT_VMCBCLEAN) ? " vmcbclean" : "",
	    (md->svm_features & SVM_FEAT_FLUSHBYASID) ? " flushbyasid" : "",
	    (md->svm_features & SVM_FEAT_DECODEASSIST) ? " decodeassist" : "");

	return (0);
}

void
ovmm_deinit_machdep(struct vmm_softc_md *md)
{
	KASSERT(md->vmm_on == 0, ("ovmm: SVM still enabled at unload"));
	if (ovmm_ipinum != IPI_AST) {
		lapic_ipi_free(ovmm_ipinum);
		ovmm_ipinum = IPI_AST;
	}
	ovmm_contigfree(md->hsave_va, md->hsave_size);
	md->hsave_va = NULL;
}

/*
 * ovmm_enable_cpu / ovmm_disable_cpu
 *
 * smp_rendezvous(9) callbacks that flip EFER.SVME on the current CPU and
 * point the CPU at its host save area.
 */
static void
ovmm_enable_cpu(void *arg)
{
	struct vmm_softc_md *md = arg;
	uint64_t efer;

	efer = rdmsr(MSR_EFER);
	efer |= EFER_SVM;
	wrmsr(MSR_EFER, efer);

	wrmsr(MSR_VM_HSAVE_PA, md->hsave_pa + (vm_paddr_t)curcpu * PAGE_SIZE);
}

static void
ovmm_disable_cpu(void *arg __unused)
{
	uint64_t efer;

	efer = rdmsr(MSR_EFER);
	efer &= ~EFER_SVM;
	wrmsr(MSR_EFER, efer);
}

/*
 * ovmm_start
 *
 * Starts SVM mode on all CPUs (if not already running).
 */
void
ovmm_start(void)
{
	struct vmm_softc_md *md = &vmm_softc->sc_md;

	sx_xlock(&vmm_softc->sc_slock);
	if (!md->vmm_on) {
		smp_rendezvous(NULL, ovmm_enable_cpu, NULL, md);
		md->vmm_on = 1;
	}
	sx_xunlock(&vmm_softc->sc_slock);
}

/*
 * ovmm_stop
 *
 * Stops SVM mode on all CPUs.
 */
void
ovmm_stop(void)
{
	struct vmm_softc_md *md = &vmm_softc->sc_md;

	sx_xlock(&vmm_softc->sc_slock);
	if (md->vmm_on) {
		smp_rendezvous(NULL, ovmm_disable_cpu, NULL, NULL);
		md->vmm_on = 0;
	}
	sx_xunlock(&vmm_softc->sc_slock);
}

/*
 * ovmm_ioctl_machdep
 *
 * Machine dependent ioctls (VMM_IOC_INTR).
 */
int
ovmm_ioctl_machdep(u_long cmd, caddr_t data, int flag, struct thread *td)
{
	int ret;

	switch (cmd) {
	case VMM_IOC_INTR:
		ret = vm_intr_pending((struct vm_intr_params *)data);
		break;
	default:
		DPRINTF("%s: unknown ioctl code 0x%lx\n", __func__, cmd);
		ret = ENOTTY;
	}

	return (ret);
}

/*
 * vm_intr_pending
 *
 * IOCTL handler routine for VMM_IOC_INTR messages, sent from the monitor
 * when an interrupt is pending and needs acknowledgment.
 *
 * If the VCPU is currently executing guest code on another CPU, an IPI
 * is sent to force a #VMEXIT so the interrupt can be delivered promptly.
 *
 * Return values:
 *  0: if successful
 *  ENOENT: if the VM/VCPU defined by 'vip' cannot be found
 */
int
vm_intr_pending(struct vm_intr_params *vip)
{
	struct vm *vm;
	struct vcpu *vcpu;
	int error, cpu, ret = 0;

	/* Find the desired VM */
	error = vm_find(vip->vip_vm_id, &vm);

	/* Not found? exit. */
	if (error != 0)
		return (error);

	vcpu = vm_find_vcpu(vm, vip->vip_vcpu_id);

	if (vcpu == NULL) {
		ret = ENOENT;
		goto out;
	}

	vcpu->vc_intr = vip->vip_intr;

	/* Pairs with the store/load ordering in vcpu_run_svm(). */
	atomic_thread_fence_seq_cst();

	cpu = atomic_load_int(&vcpu->vc_curcpu);
	if (cpu != NOCPU && cpu != curcpu)
		ipi_cpu(cpu, ovmm_ipinum);
out:
	vm_rele(vm);
	return (ret);
}

/*
 * vm_rwvmparams
 *
 * IOCTL handler to read/write the current vmm params like pvclock gpa,
 * pvclock version, etc.
 *
 * Parameters:
 *   vpp: Describes the VM and VCPU to get/set the params from
 *   dir: 0 for reading, 1 for writing
 *
 * Return values:
 *  0: if successful
 *  ENOENT: if the VM/VCPU defined by 'vpp' cannot be found
 */
int
vm_rwvmparams(struct vm_rwvmparams_params *vpp, int dir)
{
	struct vm *vm;
	struct vcpu *vcpu;
	int error, ret = 0;

	/* Find the desired VM */
	error = vm_find(vpp->vpp_vm_id, &vm);

	/* Not found? exit. */
	if (error != 0)
		return (error);

	vcpu = vm_find_vcpu(vm, vpp->vpp_vcpu_id);

	if (vcpu == NULL) {
		ret = ENOENT;
		goto out;
	}

	sx_xlock(&vcpu->vc_lock);
	if (dir == 0) {
		if (vpp->vpp_mask & VM_RWVMPARAMS_PVCLOCK_VERSION)
			vpp->vpp_pvclock_version = vcpu->vc_pvclock_version;
		if (vpp->vpp_mask & VM_RWVMPARAMS_PVCLOCK_SYSTEM_GPA)
			vpp->vpp_pvclock_system_gpa =
			    vcpu->vc_pvclock_system_gpa;
	} else {
		if (vpp->vpp_mask & VM_RWVMPARAMS_PVCLOCK_VERSION)
			vcpu->vc_pvclock_version = vpp->vpp_pvclock_version;
		if (vpp->vpp_mask & VM_RWVMPARAMS_PVCLOCK_SYSTEM_GPA) {
			vmm_init_pvclock(vcpu, vpp->vpp_pvclock_system_gpa);
		}
	}
	sx_xunlock(&vcpu->vc_lock);
out:
	vm_rele(vm);
	return (ret);
}

/*
 * vm_rwregs
 *
 * IOCTL handler to read/write the current register values of a guest VCPU.
 * The VCPU must not be running.
 *
 * Parameters:
 *   vrwp: Describes the VM and VCPU to get/set the registers from. The
 *    register values are returned here as well.
 *   dir: 0 for reading, 1 for writing
 *
 * Return values:
 *  0: if successful
 *  ENOENT: if the VM/VCPU defined by 'vrwp' cannot be found
 *  EINVAL: if an error occurred accessing the registers of the guest
 *  EPERM: if the vm cannot be accessed from the calling process
 */
int
vm_rwregs(struct vm_rwregs_params *vrwp, int dir)
{
	struct vm *vm;
	struct vcpu *vcpu;
	struct vcpu_reg_state *vrs = &vrwp->vrwp_regs;
	int error, ret = 0;

	/* Find the desired VM */
	error = vm_find(vrwp->vrwp_vm_id, &vm);

	/* Not found? exit. */
	if (error != 0)
		return (error);

	vcpu = vm_find_vcpu(vm, vrwp->vrwp_vcpu_id);

	if (vcpu == NULL) {
		ret = ENOENT;
		goto out;
	}

	sx_xlock(&vcpu->vc_lock);
	ret = (dir == 0) ?
	    vcpu_readregs_svm(vcpu, vrwp->vrwp_mask, vrs) :
	    vcpu_writeregs_svm(vcpu, vrwp->vrwp_mask, vrs);
	sx_xunlock(&vcpu->vc_lock);
out:
	vm_rele(vm);
	return (ret);
}

/*
 * ovmm_npt_pinit
 *
 * pmap init callback for the nested page table vmspace.
 */
static int
ovmm_npt_pinit(pmap_t pmap)
{
	int flags;

	/*
	 * The low bits of the pmap flags carry the IPI vector the pmap
	 * layer uses to interrupt CPUs running the guest.
	 */
	flags = (ovmm_ipinum & PMAP_NESTED_IPIMASK) | PMAP_PDE_SUPERPAGE;
	return (pmap_pinit_type(pmap, PT_RVI, flags));
}

/*
 * vm_impl_init
 *
 * VM address space initialization routine: create the vmspace that holds
 * the nested page tables and the guest memory mappings.
 *
 * Return values:
 *  0: the initialization was successful
 *  ENOMEM: the initialization failed (lack of resources)
 */
int
vm_impl_init(struct vm *vm)
{
	vm->vm_vmspace = vmspace_alloc(0, VMM_MAX_VM_MEM_SIZE, ovmm_npt_pinit);
	if (vm->vm_vmspace == NULL)
		return (ENOMEM);
	return (0);
}

void
vm_impl_deinit(struct vm *vm)
{
	if (vm->vm_vmspace != NULL) {
		vmspace_free(vm->vm_vmspace);
		vm->vm_vmspace = NULL;
	}
}

/*
 * vcpu_readregs_svm
 *
 * Reads 'vcpu's registers
 *
 * Parameters:
 *  vcpu: the vcpu to read register values from
 *  regmask: the types of registers to read
 *  vrs: output parameter where register values are stored
 *
 * Return values:
 *  0: if successful
 */
static int
vcpu_readregs_svm(struct vcpu *vcpu, uint64_t regmask,
    struct vcpu_reg_state *vrs)
{
	uint64_t *gprs = vrs->vrs_gprs;
	uint64_t *crs = vrs->vrs_crs;
	uint64_t *msrs = vrs->vrs_msrs;
	uint64_t *drs = vrs->vrs_drs;
	uint32_t attr;
	struct vcpu_segment_info *sregs = vrs->vrs_sregs;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	if (regmask & VM_RWREGS_GPRS) {
		gprs[VCPU_REGS_RAX] = vmcb->v_rax;
		gprs[VCPU_REGS_RBX] = vcpu->vc_gueststate.vg_rbx;
		gprs[VCPU_REGS_RCX] = vcpu->vc_gueststate.vg_rcx;
		gprs[VCPU_REGS_RDX] = vcpu->vc_gueststate.vg_rdx;
		gprs[VCPU_REGS_RSI] = vcpu->vc_gueststate.vg_rsi;
		gprs[VCPU_REGS_RDI] = vcpu->vc_gueststate.vg_rdi;
		gprs[VCPU_REGS_R8] = vcpu->vc_gueststate.vg_r8;
		gprs[VCPU_REGS_R9] = vcpu->vc_gueststate.vg_r9;
		gprs[VCPU_REGS_R10] = vcpu->vc_gueststate.vg_r10;
		gprs[VCPU_REGS_R11] = vcpu->vc_gueststate.vg_r11;
		gprs[VCPU_REGS_R12] = vcpu->vc_gueststate.vg_r12;
		gprs[VCPU_REGS_R13] = vcpu->vc_gueststate.vg_r13;
		gprs[VCPU_REGS_R14] = vcpu->vc_gueststate.vg_r14;
		gprs[VCPU_REGS_R15] = vcpu->vc_gueststate.vg_r15;
		gprs[VCPU_REGS_RBP] = vcpu->vc_gueststate.vg_rbp;
		gprs[VCPU_REGS_RIP] = vmcb->v_rip;
		gprs[VCPU_REGS_RSP] = vmcb->v_rsp;
		gprs[VCPU_REGS_RFLAGS] = vmcb->v_rflags;
	}

	if (regmask & VM_RWREGS_SREGS) {
#define	READ_SREG(reg, field) do {					\
	sregs[VCPU_REGS_##reg].vsi_sel = vmcb->field.vs_sel;		\
	sregs[VCPU_REGS_##reg].vsi_limit = vmcb->field.vs_lim;		\
	attr = vmcb->field.vs_attr;					\
	sregs[VCPU_REGS_##reg].vsi_ar = (attr & 0xff) | ((attr << 4) &	\
	    0xf000);							\
	sregs[VCPU_REGS_##reg].vsi_base = vmcb->field.vs_base;		\
} while (0)
		READ_SREG(CS, v_cs);
		READ_SREG(DS, v_ds);
		READ_SREG(ES, v_es);
		READ_SREG(FS, v_fs);
		READ_SREG(GS, v_gs);
		READ_SREG(SS, v_ss);
		READ_SREG(LDTR, v_ldtr);
		READ_SREG(TR, v_tr);
#undef READ_SREG

		vrs->vrs_gdtr.vsi_limit = vmcb->v_gdtr.vs_lim;
		vrs->vrs_gdtr.vsi_base = vmcb->v_gdtr.vs_base;
		vrs->vrs_idtr.vsi_limit = vmcb->v_idtr.vs_lim;
		vrs->vrs_idtr.vsi_base = vmcb->v_idtr.vs_base;
	}

	if (regmask & VM_RWREGS_CRS) {
		crs[VCPU_REGS_CR0] = vmcb->v_cr0;
		crs[VCPU_REGS_CR3] = vmcb->v_cr3;
		crs[VCPU_REGS_CR4] = vmcb->v_cr4;
		crs[VCPU_REGS_CR2] = vcpu->vc_gueststate.vg_cr2;
		crs[VCPU_REGS_XCR0] = vcpu->vc_gueststate.vg_xcr0;
	}

	if (regmask & VM_RWREGS_MSRS) {
		msrs[VCPU_REGS_EFER] = vmcb->v_efer;
		msrs[VCPU_REGS_STAR] = vmcb->v_star;
		msrs[VCPU_REGS_LSTAR] = vmcb->v_lstar;
		msrs[VCPU_REGS_CSTAR] = vmcb->v_cstar;
		msrs[VCPU_REGS_SFMASK] = vmcb->v_sfmask;
		msrs[VCPU_REGS_KGSBASE] = vmcb->v_kgsbase;
	}

	if (regmask & VM_RWREGS_DRS) {
		drs[VCPU_REGS_DR0] = vcpu->vc_gueststate.vg_dr0;
		drs[VCPU_REGS_DR1] = vcpu->vc_gueststate.vg_dr1;
		drs[VCPU_REGS_DR2] = vcpu->vc_gueststate.vg_dr2;
		drs[VCPU_REGS_DR3] = vcpu->vc_gueststate.vg_dr3;
		drs[VCPU_REGS_DR6] = vmcb->v_dr6;
		drs[VCPU_REGS_DR7] = vmcb->v_dr7;
	}

	return (0);
}

/*
 * vcpu_writeregs_svm
 *
 * Writes 'vcpu's registers
 *
 * Parameters:
 *  vcpu: the vcpu that has to get its registers written to
 *  regmask: the types of registers to write
 *  vrs: the register values to write
 *
 * Return values:
 *  0: if successful
 *  EINVAL an error writing registers occurred
 */
static int
vcpu_writeregs_svm(struct vcpu *vcpu, uint64_t regmask,
    struct vcpu_reg_state *vrs)
{
	uint64_t *gprs = vrs->vrs_gprs;
	uint64_t *crs = vrs->vrs_crs;
	uint16_t attr;
	uint64_t *msrs = vrs->vrs_msrs;
	uint64_t *drs = vrs->vrs_drs;
	struct vcpu_segment_info *sregs = vrs->vrs_sregs;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	if (regmask & VM_RWREGS_CRS) {
		/*
		 * The guest %xcr0 is loaded with xsetbv on every entry; make
		 * sure userland cannot hand us a value the CPU would reject.
		 */
		if (xsave_mask != 0 &&
		    ((crs[VCPU_REGS_XCR0] & ~xsave_mask) != 0 ||
		    (crs[VCPU_REGS_XCR0] & XFEATURE_ENABLED_X87) == 0))
			return (EINVAL);
	}

	if (regmask & VM_RWREGS_DRS) {
		if ((drs[VCPU_REGS_DR6] & 0xffffffff00000000ULL) != 0)
			return (EINVAL);
		if ((drs[VCPU_REGS_DR7] & 0xffffffff00000000ULL) != 0)
			return (EINVAL);
	}

	if (regmask & VM_RWREGS_GPRS) {
		vcpu->vc_gueststate.vg_rax = gprs[VCPU_REGS_RAX];
		vcpu->vc_gueststate.vg_rbx = gprs[VCPU_REGS_RBX];
		vcpu->vc_gueststate.vg_rcx = gprs[VCPU_REGS_RCX];
		vcpu->vc_gueststate.vg_rdx = gprs[VCPU_REGS_RDX];
		vcpu->vc_gueststate.vg_rsi = gprs[VCPU_REGS_RSI];
		vcpu->vc_gueststate.vg_rdi = gprs[VCPU_REGS_RDI];
		vcpu->vc_gueststate.vg_r8 = gprs[VCPU_REGS_R8];
		vcpu->vc_gueststate.vg_r9 = gprs[VCPU_REGS_R9];
		vcpu->vc_gueststate.vg_r10 = gprs[VCPU_REGS_R10];
		vcpu->vc_gueststate.vg_r11 = gprs[VCPU_REGS_R11];
		vcpu->vc_gueststate.vg_r12 = gprs[VCPU_REGS_R12];
		vcpu->vc_gueststate.vg_r13 = gprs[VCPU_REGS_R13];
		vcpu->vc_gueststate.vg_r14 = gprs[VCPU_REGS_R14];
		vcpu->vc_gueststate.vg_r15 = gprs[VCPU_REGS_R15];
		vcpu->vc_gueststate.vg_rbp = gprs[VCPU_REGS_RBP];
		vcpu->vc_gueststate.vg_rip = gprs[VCPU_REGS_RIP];

		vmcb->v_rax = gprs[VCPU_REGS_RAX];
		vmcb->v_rip = gprs[VCPU_REGS_RIP];
		vmcb->v_rsp = gprs[VCPU_REGS_RSP];
		vmcb->v_rflags = gprs[VCPU_REGS_RFLAGS];
	}

	if (regmask & VM_RWREGS_SREGS) {
#define	WRITE_SREG(reg, field) do {					\
	vmcb->field.vs_sel = sregs[VCPU_REGS_##reg].vsi_sel;		\
	vmcb->field.vs_lim = sregs[VCPU_REGS_##reg].vsi_limit;		\
	attr = sregs[VCPU_REGS_##reg].vsi_ar;				\
	vmcb->field.vs_attr = (attr & 0xff) | ((attr >> 4) & 0xf00);	\
	vmcb->field.vs_base = sregs[VCPU_REGS_##reg].vsi_base;		\
} while (0)
		WRITE_SREG(CS, v_cs);
		WRITE_SREG(DS, v_ds);
		WRITE_SREG(ES, v_es);
		WRITE_SREG(FS, v_fs);
		WRITE_SREG(GS, v_gs);
		WRITE_SREG(SS, v_ss);
		WRITE_SREG(LDTR, v_ldtr);
		WRITE_SREG(TR, v_tr);
#undef WRITE_SREG

		vmcb->v_gdtr.vs_lim = vrs->vrs_gdtr.vsi_limit;
		vmcb->v_gdtr.vs_base = vrs->vrs_gdtr.vsi_base;
		vmcb->v_idtr.vs_lim = vrs->vrs_idtr.vsi_limit;
		vmcb->v_idtr.vs_base = vrs->vrs_idtr.vsi_base;
	}

	if (regmask & VM_RWREGS_CRS) {
		vmcb->v_cr0 = crs[VCPU_REGS_CR0];
		vmcb->v_cr3 = crs[VCPU_REGS_CR3];
		vmcb->v_cr4 = crs[VCPU_REGS_CR4];
		vcpu->vc_gueststate.vg_cr2 = crs[VCPU_REGS_CR2];
		vcpu->vc_gueststate.vg_xcr0 = crs[VCPU_REGS_XCR0];
	}

	if (regmask & VM_RWREGS_MSRS) {
		vmcb->v_efer |= msrs[VCPU_REGS_EFER];
		vmcb->v_star = msrs[VCPU_REGS_STAR];
		vmcb->v_lstar = msrs[VCPU_REGS_LSTAR];
		vmcb->v_cstar = msrs[VCPU_REGS_CSTAR];
		vmcb->v_sfmask = msrs[VCPU_REGS_SFMASK];
		vmcb->v_kgsbase = msrs[VCPU_REGS_KGSBASE];
	}

	if (regmask & VM_RWREGS_DRS) {
		vcpu->vc_gueststate.vg_dr0 = drs[VCPU_REGS_DR0];
		vcpu->vc_gueststate.vg_dr1 = drs[VCPU_REGS_DR1];
		vcpu->vc_gueststate.vg_dr2 = drs[VCPU_REGS_DR2];
		vcpu->vc_gueststate.vg_dr3 = drs[VCPU_REGS_DR3];
		vmcb->v_dr6 = drs[VCPU_REGS_DR6];
		vmcb->v_dr7 = drs[VCPU_REGS_DR7];
	}

	/* Everything may have changed. */
	svm_set_dirty(vcpu, SVM_CLEANBITS_ALL);

	return (0);
}

/*
 * vcpu_reset_regs_svm
 *
 * Initializes 'vcpu's registers to supplied state
 *
 * Parameters:
 *  vcpu: the vcpu whose register state is to be initialized
 *  vrs: the register state to set
 *
 * Return values:
 *  0: registers init'ed successfully
 *  EINVAL: an error occurred setting register state
 */
static int
vcpu_reset_regs_svm(struct vcpu *vcpu, struct vcpu_reg_state *vrs)
{
	struct vmcb *vmcb;
	int ret;

	vmcb = (struct vmcb *)vcpu->vc_control_va;

	/*
	 * Intercept controls
	 *
	 * External Interrupt exiting (SVM_INTERCEPT_INTR)
	 * External NMI exiting (SVM_INTERCEPT_NMI)
	 * CPUID instruction (SVM_INTERCEPT_CPUID)
	 * HLT instruction (SVM_INTERCEPT_HLT)
	 * I/O instructions (SVM_INTERCEPT_INOUT)
	 * MSR access (SVM_INTERCEPT_MSR)
	 * shutdown events (SVM_INTERCEPT_SHUTDOWN)
	 * INVLPGA instruction (SVM_INTERCEPT_INVLPGA)
	 *
	 * VMRUN instruction (SVM_INTERCEPT_VMRUN)
	 * VMMCALL instruction (SVM_INTERCEPT_VMMCALL)
	 * VMLOAD instruction (SVM_INTERCEPT_VMLOAD)
	 * VMSAVE instruction (SVM_INTERCEPT_VMSAVE)
	 * STGI instruction (SVM_INTERCEPT_STGI)
	 * CLGI instruction (SVM_INTERCEPT_CLGI)
	 * SKINIT instruction (SVM_INTERCEPT_SKINIT)
	 * ICEBP instruction (SVM_INTERCEPT_ICEBP)
	 * MWAIT instruction (SVM_INTERCEPT_MWAIT_UNCOND)
	 * MWAIT instruction (SVM_INTERCEPT_MWAIT_COND)
	 * MONITOR instruction (SVM_INTERCEPT_MONITOR)
	 * RDTSCP instruction (SVM_INTERCEPT_RDTSCP)
	 * XSETBV instruction (SVM_INTERCEPT_XSETBV) (if available)
	 */
	vmcb->v_intercept1 = SVM_INTERCEPT_INTR | SVM_INTERCEPT_NMI |
	    SVM_INTERCEPT_CPUID | SVM_INTERCEPT_HLT | SVM_INTERCEPT_INOUT |
	    SVM_INTERCEPT_MSR | SVM_INTERCEPT_SHUTDOWN | SVM_INTERCEPT_INVLPGA;

	vmcb->v_intercept2 = SVM_INTERCEPT_VMRUN | SVM_INTERCEPT_VMMCALL |
	    SVM_INTERCEPT_VMLOAD | SVM_INTERCEPT_VMSAVE | SVM_INTERCEPT_STGI |
	    SVM_INTERCEPT_CLGI | SVM_INTERCEPT_SKINIT | SVM_INTERCEPT_ICEBP |
	    SVM_INTERCEPT_MWAIT_UNCOND | SVM_INTERCEPT_MONITOR |
	    SVM_INTERCEPT_MWAIT_COND | SVM_INTERCEPT_RDTSCP;

	if (xsave_mask)
		vmcb->v_intercept2 |= SVM_INTERCEPT_XSETBV;

	/* Setup I/O bitmap: intercept every port */
	memset((uint8_t *)vcpu->vc_svm_ioio_va, 0xFF, 3 * PAGE_SIZE);
	vmcb->v_iopm_pa = (uint64_t)(vcpu->vc_svm_ioio_pa);

	/* Setup MSR bitmap: intercept everything, then allow selected MSRs */
	memset((uint8_t *)vcpu->vc_msr_bitmap_va, 0xFF, 2 * PAGE_SIZE);
	vmcb->v_msrpm_pa = (uint64_t)(vcpu->vc_msr_bitmap_pa);
	svm_setmsrbrw(vcpu, MSR_IA32_FEATURE_CONTROL);
	svm_setmsrbrw(vcpu, MSR_SYSENTER_CS_MSR);
	svm_setmsrbrw(vcpu, MSR_SYSENTER_ESP_MSR);
	svm_setmsrbrw(vcpu, MSR_SYSENTER_EIP_MSR);
	svm_setmsrbrw(vcpu, MSR_STAR);
	svm_setmsrbrw(vcpu, MSR_LSTAR);
	svm_setmsrbrw(vcpu, MSR_CSTAR);
	svm_setmsrbrw(vcpu, MSR_SF_MASK);
	svm_setmsrbrw(vcpu, MSR_FSBASE);
	svm_setmsrbrw(vcpu, MSR_GSBASE);
	svm_setmsrbrw(vcpu, MSR_KGSBASE);

	/* allow reading SEV status */
	svm_setmsrbr(vcpu, OVMM_MSR_SEV_STATUS);

	/* EFER is R/O so we can ensure the guest always has SVME */
	svm_setmsrbr(vcpu, MSR_EFER);

	/* allow reading TSC */
	svm_setmsrbr(vcpu, MSR_TSC);

	/* allow reading HWCR and PSTATEDEF to determine TSC frequency */
	svm_setmsrbr(vcpu, MSR_HWCR);
	svm_setmsrbr(vcpu, OVMM_MSR_PSTATE0);

	/* Guest VCPU ASID */
	vmcb->v_asid = vcpu->vc_vpid;

	/* TLB Control - First time in, flush all*/
	vmcb->v_tlb_control = SVM_TLB_CONTROL_FLUSH_ALL;

	/* INTR masking */
	vmcb->v_intr_masking = 1;

	/* PAT */
	vmcb->v_g_pat = PAT_VALUE(0, PAT_WRITE_BACK) |
	    PAT_VALUE(1, PAT_WRITE_COMBINING) |
	    PAT_VALUE(2, PAT_UNCACHED) | PAT_VALUE(3, PAT_UNCACHEABLE) |
	    PAT_VALUE(4, PAT_WRITE_BACK) | PAT_VALUE(5, PAT_WRITE_COMBINING) |
	    PAT_VALUE(6, PAT_UNCACHED) | PAT_VALUE(7, PAT_UNCACHEABLE);

	/* NPT */
	vmcb->v_np_enable = SVM_ENABLE_NP;
	vmcb->v_n_cr3 = vtophys(ovmm_pmap(vcpu->vc_parent)->pm_pmltop);

	/* Enable SVME in EFER (must always be set) */
	vmcb->v_efer |= EFER_SVM;

	if ((ret = vcpu_writeregs_svm(vcpu, VM_RWREGS_ALL, vrs)) != 0)
		return ret;

	/* xcr0 power on default sets bit 0 (x87 state) */
	vcpu->vc_gueststate.vg_xcr0 = XFEATURE_ENABLED_X87 & xsave_mask;

	/* Force a full TLB flush and nested pmap resync on the next entry. */
	vcpu->vc_last_pcpu = NOCPU;
	vcpu->vc_eptgen = -1;

	return (0);
}

/*
 * svm_setmsrbr
 *
 * Allow read access to the specified msr on the supplied vcpu.
 *
 * Parameters:
 *  vcpu: the VCPU to allow access
 *  msr: the MSR number to allow access to
 */
static void
svm_setmsrbr(struct vcpu *vcpu, uint32_t msr)
{
	uint8_t *msrs;
	uint16_t idx;

	msrs = (uint8_t *)vcpu->vc_msr_bitmap_va;

	/*
	 * MSR Read bitmap layout:
	 * Pentium MSRs (0x0 - 0x1fff) @ 0x0
	 * Gen6 and Syscall MSRs (0xc0000000 - 0xc0001fff) @ 0x800
	 * Gen7 and Gen8 MSRs (0xc0010000 - 0xc0011fff) @ 0x1000
	 *
	 * Read enable bit is low order bit of 2-bit pair
	 * per MSR (eg, MSR 0x0 write bit is at bit 0 @ 0x0)
	 */
	if (msr <= 0x1fff) {
		idx = SVM_MSRIDX(msr);
		msrs[idx] &= ~(SVM_MSRBIT_R(msr));
	} else if (msr >= 0xc0000000 && msr <= 0xc0001fff) {
		idx = SVM_MSRIDX(msr - 0xc0000000) + 0x800;
		msrs[idx] &= ~(SVM_MSRBIT_R(msr - 0xc0000000));
	} else if (msr >= 0xc0010000 && msr <= 0xc0011fff) {
		idx = SVM_MSRIDX(msr - 0xc0010000) + 0x1000;
		msrs[idx] &= ~(SVM_MSRBIT_R(msr - 0xc0010000));
	} else {
		printf("%s: invalid msr 0x%x\n", __func__, msr);
		return;
	}
}

/*
 * svm_setmsrbw
 *
 * Allow write access to the specified msr on the supplied vcpu
 *
 * Parameters:
 *  vcpu: the VCPU to allow access
 *  msr: the MSR number to allow access to
 */
static void
svm_setmsrbw(struct vcpu *vcpu, uint32_t msr)
{
	uint8_t *msrs;
	uint16_t idx;

	msrs = (uint8_t *)vcpu->vc_msr_bitmap_va;

	/*
	 * MSR Write bitmap layout:
	 * Pentium MSRs (0x0 - 0x1fff) @ 0x0
	 * Gen6 and Syscall MSRs (0xc0000000 - 0xc0001fff) @ 0x800
	 * Gen7 and Gen8 MSRs (0xc0010000 - 0xc0011fff) @ 0x1000
	 *
	 * Write enable bit is high order bit of 2-bit pair
	 * per MSR (eg, MSR 0x0 write bit is at bit 1 @ 0x0)
	 */
	if (msr <= 0x1fff) {
		idx = SVM_MSRIDX(msr);
		msrs[idx] &= ~(SVM_MSRBIT_W(msr));
	} else if (msr >= 0xc0000000 && msr <= 0xc0001fff) {
		idx = SVM_MSRIDX(msr - 0xc0000000) + 0x800;
		msrs[idx] &= ~(SVM_MSRBIT_W(msr - 0xc0000000));
	} else if (msr >= 0xc0010000 && msr <= 0xc0011fff) {
		idx = SVM_MSRIDX(msr - 0xc0010000) + 0x1000;
		msrs[idx] &= ~(SVM_MSRBIT_W(msr - 0xc0010000));
	} else {
		printf("%s: invalid msr 0x%x\n", __func__, msr);
		return;
	}
}

/*
 * svm_setmsrbrw
 *
 * Allow read/write access to the specified msr on the supplied vcpu
 *
 * Parameters:
 *  vcpu: the VCPU to allow access
 *  msr: the MSR number to allow access to
 */
static void
svm_setmsrbrw(struct vcpu *vcpu, uint32_t msr)
{
	svm_setmsrbr(vcpu, msr);
	svm_setmsrbw(vcpu, msr);
}

/*
 * svm_set_clean
 *
 * Sets (mark as unmodified) the VMCB clean bit set in 'value'.
 * For example, to set the clean bit for the VMCB intercepts (bit position 0),
 * the caller provides 'SVM_CLEANBITS_I' (0x1) for the 'value' argument.
 * Multiple cleanbits can be provided in 'value' at the same time (eg,
 * "SVM_CLEANBITS_I | SVM_CLEANBITS_TPR").
 *
 * Note that this function does not clear any bits; to clear bits in the
 * vmcb cleanbits bitfield, use 'svm_set_dirty'.
 *
 * Parameters:
 *  vcpu: the VCPU whose VMCB clean value should be set
 *  value: the value(s) to enable in the cleanbits mask
 */
static void
svm_set_clean(struct vcpu *vcpu, uint32_t value)
{
	struct vmcb *vmcb;

	/* If no cleanbits support, do nothing */
	if (!ovmm_vmcb_clean())
		return;

	vmcb = (struct vmcb *)vcpu->vc_control_va;

	vmcb->v_vmcb_clean_bits |= value;
}

/*
 * svm_set_dirty
 *
 * Clears (mark as modified) the VMCB clean bit set in 'value'.
 * For example, to clear the bit for the VMCB intercepts (bit position 0)
 * the caller provides 'SVM_CLEANBITS_I' (0x1) for the 'value' argument.
 * Multiple dirty bits can be provided in 'value' at the same time (eg,
 * "SVM_CLEANBITS_I | SVM_CLEANBITS_TPR").
 *
 * Parameters:
 *  vcpu: the VCPU whose VMCB dirty value should be set
 *  value: the value(s) to dirty in the cleanbits mask
 */
static void
svm_set_dirty(struct vcpu *vcpu, uint32_t value)
{
	struct vmcb *vmcb;

	/* If no cleanbits support, do nothing */
	if (!ovmm_vmcb_clean())
		return;

	vmcb = (struct vmcb *)vcpu->vc_control_va;

	vmcb->v_vmcb_clean_bits &= ~value;
}

/*
 * vcpu_reset_regs
 *
 * Resets a vcpu's registers to the provided state
 *
 * Parameters:
 *  vcpu: the vcpu whose registers shall be reset
 *  vrs: the desired register state
 *
 * Return values:
 *  0: the vcpu's registers were successfully reset
 *  !0: the vcpu's registers could not be reset (see arch-specific reset
 *      function for various values that can be returned here)
 */
int
vcpu_reset_regs(struct vcpu *vcpu, struct vcpu_reg_state *vrs)
{
	return (vcpu_reset_regs_svm(vcpu, vrs));
}

/*
 * vcpu_init_svm
 *
 * AMD SVM specific VCPU initialization routine.
 *
 * This function allocates various per-VCPU memory regions, sets up initial
 * VCPU VMCB controls, and sets initial register values.
 *
 * Parameters:
 *  vcpu: the VCPU structure being initialized
 *  vcp: parameters provided by the monitor
 *
 * Return values:
 *  0: the VCPU was initialized successfully
 *  ENOMEM: insufficient resources
 */
static int
vcpu_init_svm(struct vcpu *vcpu, struct vm_create_params *vcp)
{
	int ret = 0;

	/* Allocate an ASID early to avoid the allocations if out of ASIDs. */
	if (vmm_alloc_asid(&vcpu->vc_vpid))
		return (ENOMEM);

	/* Allocate VMCB (1 page) */
	vcpu->vc_control_va = ovmm_contigalloc(PAGE_SIZE,
	    &vcpu->vc_control_pa);
	if (vcpu->vc_control_va == NULL) {
		ret = ENOMEM;
		goto exit;
	}

	DPRINTF("%s: VMCB va @ %p, pa @ 0x%lx\n", __func__,
	    vcpu->vc_control_va, (u_long)vcpu->vc_control_pa);

	/* Allocate MSR bitmap (2 pages) */
	vcpu->vc_msr_bitmap_va = ovmm_contigalloc(2 * PAGE_SIZE,
	    &vcpu->vc_msr_bitmap_pa);
	if (vcpu->vc_msr_bitmap_va == NULL) {
		ret = ENOMEM;
		goto exit;
	}

	DPRINTF("%s: MSR bitmap va @ %p, pa @ 0x%lx\n", __func__,
	    vcpu->vc_msr_bitmap_va, (u_long)vcpu->vc_msr_bitmap_pa);

	/* Allocate IOIO area (3 pages) */
	vcpu->vc_svm_ioio_va = ovmm_contigalloc(3 * PAGE_SIZE,
	    &vcpu->vc_svm_ioio_pa);
	if (vcpu->vc_svm_ioio_va == NULL) {
		ret = ENOMEM;
		goto exit;
	}

	DPRINTF("%s: IOIO va @ %p, pa @ 0x%lx\n", __func__,
	    vcpu->vc_svm_ioio_va, (u_long)vcpu->vc_svm_ioio_pa);

	/* Guest FPU state */
	vcpu->vc_g_fpu = fpu_save_area_alloc();
	if (vcpu->vc_g_fpu == NULL) {
		ret = ENOMEM;
		goto exit;
	}
	fpu_save_area_reset(vcpu->vc_g_fpu);

	/* Inform the monitor about ASID and C bit position (no SEV: 0). */
	vcp->vcp_poscbit = 0;
	vcp->vcp_asid[vcpu->vc_id] = vcpu->vc_vpid;

exit:
	if (ret)
		vcpu_deinit_svm(vcpu);

	return (ret);
}

/*
 * vcpu_init
 *
 * Calls the architecture-specific VCPU init routine
 */
int
vcpu_init(struct vcpu *vcpu, struct vm_create_params *vcp)
{
	vcpu->vc_virt_mode = vmm_softc->mode;
	vcpu->vc_state = VCPU_STATE_STOPPED;
	vcpu->vc_vpid = 0;
	vcpu->vc_pvclock_system_gpa = 0;
	vcpu->vc_last_pcpu = NOCPU;
	vcpu->vc_curcpu = NOCPU;
	vcpu->vc_eptgen = -1;
	sx_init(&vcpu->vc_lock, "ovmm vcpu");

	/* Shadow PAT MSR, starting with host's value. */
	vcpu->vc_shadow_pat = rdmsr(MSR_PAT);

	return (vcpu_init_svm(vcpu, vcp));
}

/*
 * vcpu_deinit_svm
 *
 * Deinitializes the vcpu described by 'vcpu'
 *
 * Parameters:
 *  vcpu: the vcpu to be deinited
 */
static void
vcpu_deinit_svm(struct vcpu *vcpu)
{
	if (vcpu->vc_control_va != NULL) {
		ovmm_contigfree(vcpu->vc_control_va, PAGE_SIZE);
		vcpu->vc_control_va = NULL;
	}
	if (vcpu->vc_msr_bitmap_va != NULL) {
		ovmm_contigfree(vcpu->vc_msr_bitmap_va, 2 * PAGE_SIZE);
		vcpu->vc_msr_bitmap_va = NULL;
	}
	if (vcpu->vc_svm_ioio_va != NULL) {
		ovmm_contigfree(vcpu->vc_svm_ioio_va, 3 * PAGE_SIZE);
		vcpu->vc_svm_ioio_va = NULL;
	}
	if (vcpu->vc_g_fpu != NULL) {
		fpu_save_area_free(vcpu->vc_g_fpu);
		vcpu->vc_g_fpu = NULL;
	}

	if (vcpu->vc_vpid != 0) {
		vmm_free_asid(vcpu->vc_vpid);
		vcpu->vc_vpid = 0;
	}
}

/*
 * vcpu_deinit
 *
 * Calls the architecture-specific VCPU deinit routine
 *
 * Parameters:
 *  vcpu: the vcpu to be deinited
 */
void
vcpu_deinit(struct vcpu *vcpu)
{
	vcpu_deinit_svm(vcpu);
	sx_destroy(&vcpu->vc_lock);
}

/*
 * vm_run
 *
 * Run the vm / vcpu specified by 'vrp'
 *
 * Parameters:
 *  vrp: structure defining the VM to run
 *
 * Return value:
 *  ENOENT: the VM defined in 'vrp' could not be located
 *  EBUSY: the VM defined in 'vrp' is already running
 *  EFAULT: error copying data from userspace (vmd) on return from previous
 *      exit.
 *  EAGAIN: help is needed from vmd(8) (device I/O or exit vmm(4) cannot
 *      handle in-kernel.)
 *  0: the run loop exited and no help is needed from vmd(8)
 */
int
vm_run(struct vm_run_params *vrp)
{
	struct vm *vm;
	struct vcpu *vcpu;
	int ret = 0, vcpu_rv = 0;

	/*
	 * Find desired VM
	 */
	ret = vm_find(vrp->vrp_vm_id, &vm);
	if (ret)
		return (ret);

	vcpu = vm_find_vcpu(vm, vrp->vrp_vcpu_id);
	if (vcpu == NULL) {
		ret = ENOENT;
		goto out;
	}

	ret = sx_xlock_sig(&vcpu->vc_lock);
	if (ret != 0)
		goto out;

	/*
	 * We may be returning from userland helping us from the last
	 * exit. Copy in the exit data from vmd. The exit data will be
	 * consumed before the next entry (this typically comprises
	 * VCPU register changes as the result of vmd(8)'s actions).
	 */
	ret = copyin(vrp->vrp_exit, &vcpu->vc_exit, sizeof(struct vm_exit));
	if (ret)
		goto out_unlock;

	/*
	 * Attempt to transition from VCPU_STATE_STOPPED -> VCPU_STATE_RUNNING.
	 * Failure to make the transition indicates the VCPU is busy.
	 */
	if (atomic_cmpset_int(&vcpu->vc_state, VCPU_STATE_STOPPED,
	    VCPU_STATE_RUNNING) == 0) {
		ret = EBUSY;
		goto out_unlock;
	}

	vcpu->vc_inject.vie_type = vrp->vrp_inject.vie_type;
	vcpu->vc_inject.vie_vector = vrp->vrp_inject.vie_vector;
	vcpu->vc_inject.vie_errorcode = vrp->vrp_inject.vie_errorcode;

	/* Run the VCPU specified in vrp */
	vcpu_rv = vcpu_run_svm(vcpu, vrp);

	if (vcpu_rv == 0 || vcpu_rv == EAGAIN) {
		/* vcpu requires userland assist or is yielding */
		vrp->vrp_exit_reason = (vcpu_rv == 0) ? VM_EXIT_NONE
		    : vcpu->vc_gueststate.vg_exit_reason;
		vrp->vrp_irqready = vcpu->vc_irqready;
		vcpu->vc_state = VCPU_STATE_STOPPED;
		ret = copyout(&vcpu->vc_exit, vrp->vrp_exit,
		    sizeof(struct vm_exit));
	} else {
		/* vcpu is in a terminal state */
		vrp->vrp_exit_reason = VM_EXIT_TERMINATED;
		vcpu->vc_state = VCPU_STATE_TERMINATED;
	}

out_unlock:
	sx_xunlock(&vcpu->vc_lock);
out:
	vm_rele(vm);
	return (ret);
}

/*
 * ovmm_restore_host_tss
 *
 * The TSS descriptor was in use prior to launching the guest so it
 * has been marked busy.  'ltr' requires the descriptor to be marked
 * available so change the type to "64-bit available TSS" first.
 */
static void
ovmm_restore_host_tss(void)
{
	struct system_segment_descriptor *tss_sd;

	tss_sd = PCPU_GET(tss);
	tss_sd->sd_type = SDT_SYSTSS;
	ltr(GSEL(GPROC0_SEL, SEL_KPL));
}

/*
 * ovmm_inject_event
 *
 * Program the VMCB event injection field from the pending vc_inject.
 *
 * Return values:
 *  0: ok (possibly nothing to inject)
 *  EINVAL: unsupported exception vector
 */
static int
ovmm_inject_event(struct vcpu *vcpu, struct vmcb *vmcb)
{
	/* Is there an interrupt pending injection? */
	if (vcpu->vc_inject.vie_type == VCPU_INJECT_INTR &&
	    vcpu->vc_irqready) {
		vmcb->v_eventinj = vcpu->vc_inject.vie_vector | (1U << 31);
		vcpu->vc_inject.vie_type = VCPU_INJECT_NONE;
	}

	/* Inject event if present */
	if (vcpu->vc_inject.vie_type == VCPU_INJECT_EX) {
		vmcb->v_eventinj = vcpu->vc_inject.vie_vector;

		/* Set the "Event Valid" flag for certain vectors */
		switch (vcpu->vc_inject.vie_vector) {
		case VMM_EX_BP:
		case VMM_EX_OF:
			/*
			 * Software exception.
			 * XXX check nRIP support.
			 */
			vmcb->v_eventinj |= (4ULL << 8);
			break;
		case VMM_EX_DB:
		case VMM_EX_UD:
			/* Hardware exception, no error code. */
			vmcb->v_eventinj |= (3ULL << 8);
			break;
		case VMM_EX_AC:
			vcpu->vc_inject.vie_errorcode = 0;
			/* fallthrough */
		case VMM_EX_DF:
		case VMM_EX_TS:
		case VMM_EX_NP:
		case VMM_EX_SS:
		case VMM_EX_GP:
		case VMM_EX_PF:
			/* Hardware exception. */
			vmcb->v_eventinj |= (3ULL << 8);

			if (vmcb->v_cr0 & CR0_PE) {
				/* Error code valid. */
				vmcb->v_eventinj |= (1ULL << 11);
				vmcb->v_eventinj |= (uint64_t)
				    vcpu->vc_inject.vie_errorcode << 32;
			}
			break;
		default:
			printf("%s: unsupported exception vector %u\n",
			    __func__, vcpu->vc_inject.vie_vector);
			return (EINVAL);
		}

		/* Event is valid. */
		vmcb->v_eventinj |= (1U << 31);
		vcpu->vc_inject.vie_type = VCPU_INJECT_NONE;
	}

	return (0);
}

/*
 * vcpu_run_svm
 *
 * SVM main loop used to run a VCPU.
 *
 * Parameters:
 *  vcpu: The VCPU to run
 *  vrp: run parameters
 *
 * Return values:
 *  0: The run loop exited and no help is needed from vmd
 *  EAGAIN: The run loop exited and help from vmd is needed
 *  EINVAL: an error occurred
 */
static int
vcpu_run_svm(struct vcpu *vcpu, struct vm_run_params *vrp)
{
	struct vmm_softc_md *md = &vmm_softc->sc_md;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;
	pmap_t pmap = ovmm_pmap(vcpu->vc_parent);
	uint64_t exit_reason;
	long eptgen;
	int ret = 0, cpu;
	uint16_t ldt_sel;
	bool flush;

	if (vrp->vrp_intr_pending)
		vcpu->vc_intr = 1;
	else
		vcpu->vc_intr = 0;

	/*
	 * If we are returning from userspace (vmd) because we exited
	 * last time, fix up any needed vcpu state first. Which state
	 * needs to be fixed up depends on what vmd populated in the
	 * exit data structure.
	 */
	switch (vcpu->vc_gueststate.vg_exit_reason) {
	case SVM_VMEXIT_IOIO:
		if (vcpu->vc_exit.vei.vei_dir == VEI_DIR_IN) {
			vcpu->vc_gueststate.vg_rax =
			    vcpu->vc_exit.vei.vei_data;
			vmcb->v_rax = vcpu->vc_gueststate.vg_rax;
		}
		vcpu->vc_gueststate.vg_rip =
		    vcpu->vc_exit.vrs.vrs_gprs[VCPU_REGS_RIP];
		vmcb->v_rip = vcpu->vc_gueststate.vg_rip;
		break;
	case SVM_VMEXIT_NPF:
		ret = vcpu_writeregs_svm(vcpu, VM_RWREGS_GPRS,
		    &vcpu->vc_exit.vrs);
		if (ret) {
			printf("%s: vm %d vcpu %d failed to update "
			    "registers\n", __func__,
			    vcpu->vc_parent->vm_id, vcpu->vc_id);
			return (EINVAL);
		}
		break;
	}
	memset(&vcpu->vc_exit, 0, sizeof(vcpu->vc_exit));

	while (ret == 0) {
		vmm_update_pvclock(vcpu);

		/* Handle vmd(8) injected interrupts and exceptions */
		ret = ovmm_inject_event(vcpu, vmcb);
		if (ret != 0)
			break;

		/*
		 * From here until stgi() nothing may sleep or be preempted:
		 * we are about to load guest state into the CPU.
		 */
		critical_enter();
		ovmm_clgi();
		cpu = curcpu;

		flush = false;
		if (cpu != vcpu->vc_last_pcpu) {
			/*
			 * We are launching for the first time, or we are
			 * resuming from a different pcpu, so the TLB may
			 * hold stale entries for our ASID and the VMCB
			 * state cache must be invalidated.
			 */
			flush = true;
			svm_set_dirty(vcpu, SVM_CLEANBITS_ALL);
			vcpu->vc_last_pcpu = cpu;
		}

		/*
		 * Tell the pmap layer we are using its page tables on this
		 * CPU (so that it IPIs us on invalidation) and check whether
		 * the host changed any guest mappings since our last entry.
		 */
		CPU_SET_ATOMIC(cpu, &pmap->pm_active);
		smr_enter(pmap->pm_eptsmr);
		eptgen = atomic_load_long(&pmap->pm_eptgen);
		if (vcpu->vc_eptgen != eptgen) {
			flush = true;
			vcpu->vc_eptgen = eptgen;
		}

		if (flush) {
			/*
			 * Flush TLB by guest ASID if feature
			 * available, flush entire TLB if not.
			 */
			if (ovmm_flush_by_asid())
				vmcb->v_tlb_control =
				    SVM_TLB_CONTROL_FLUSH_ASID;
			else
				vmcb->v_tlb_control =
				    SVM_TLB_CONTROL_FLUSH_ALL;
		}

		/*
		 * Publish the CPU we are about to run on.  vm_intr_pending()
		 * reads it (after setting vc_intr) to decide whether to IPI
		 * us out of the guest.  If an interrupt became pending in
		 * the meantime and the guest is ready for it, do not enter
		 * at all - unless we have an event to deliver right now.
		 */
		atomic_store_int(&vcpu->vc_curcpu, cpu);
		atomic_thread_fence_seq_cst();
		if (vcpu->vc_intr && vcpu->vc_irqready &&
		    (vmcb->v_eventinj & (1U << 31)) == 0) {
			atomic_store_int(&vcpu->vc_curcpu, NOCPU);
			smr_exit(pmap->pm_eptsmr);
			CPU_CLR_ATOMIC(cpu, &pmap->pm_active);
			ovmm_stgi();
			critical_exit();
			ret = EAGAIN;
			break;
		}

		/*
		 * #VMEXIT resumes the host with the guest LDTR, so
		 * save the current LDT selector so it can be restored
		 * after an exit.
		 */
		ldt_sel = sldt();

		/*
		 * Save the host (userland) FPU state and load the guest's.
		 */
		fpuexit(curthread);
		fpu_enable();
		fpurestore(vcpu->vc_g_fpu);
		if (rcr4() & CR4_XSAVE)
			load_xcr(0, vcpu->vc_gueststate.vg_xcr0);

		/*
		 * If we're resuming to a different VCPU and have IBPB,
		 * then use it to prevent cross-VM branch-target injection.
		 */
		if (DPCPU_GET(ovmm_last_vcpu) != vcpu &&
		    (amd_extended_feature_extensions & AMDFEID_IBPB)) {
			wrmsr(MSR_IA32_PRED_CMD, IA32_PRED_CMD_IBPB_BARRIER);
			DPCPU_SET(ovmm_last_vcpu, vcpu);
		}

		KASSERT(vmcb->v_intercept1 & SVM_INTERCEPT_INTR,
		    ("ovmm: INTR intercept disabled"));

		ret = svm_enter_guest(vcpu->vc_control_pa,
		    &vcpu->vc_gueststate);

		/*
		 * On exit, interrupts are disabled, and we are running with
		 * the guest FPU state still on the CPU. Save the FPU state
		 * and restore the host %xcr0 before re-enabling interrupts.
		 */
		if (rcr4() & CR4_XSAVE) {
			vcpu->vc_gueststate.vg_xcr0 = rxcr(0);
			load_xcr(0, md->host_xcr0);
		}
		fpusave(vcpu->vc_g_fpu);
		fpu_disable();

		/* Do not let a restrictive guest PKRU break copyout(9). */
		if (md->pkru_enabled)
			wrpkru(0);

		/*
		 * The host GDTR and IDTR are saved by VMRUN and restored
		 * automatically on #VMEXIT. However, the host TSS and LDT
		 * need to be restored explicitly.
		 */
		ovmm_restore_host_tss();
		lldt(ldt_sel);

		atomic_store_int(&vcpu->vc_curcpu, NOCPU);
		smr_exit(pmap->pm_eptsmr);
		CPU_CLR_ATOMIC(cpu, &pmap->pm_active);

		/*
		 * Enable interrupts now. Note that if the exit was due to INTR
		 * (external interrupt), the interrupt will be processed now.
		 */
		ovmm_stgi();
		critical_exit();

		vcpu->vc_gueststate.vg_rip = vmcb->v_rip;
		vmcb->v_tlb_control = SVM_TLB_CONTROL_FLUSH_NONE;
		svm_set_clean(vcpu, SVM_CLEANBITS_ALL);

		/* If we exited successfully ... */
		if (ret == 0) {
			exit_reason = vmcb->v_exitcode;
			vcpu->vc_gueststate.vg_exit_reason = exit_reason;
			vcpu->vc_gueststate.vg_rflags = vmcb->v_rflags;

			/*
			 * Handle the exit. This will alter "ret" to EAGAIN if
			 * the exit handler determines help from vmd is needed.
			 */
			ret = svm_handle_exit(vcpu);

			if (vcpu->vc_gueststate.vg_rflags & PSL_I)
				vcpu->vc_irqready = 1;
			else
				vcpu->vc_irqready = 0;

			/*
			 * If not ready for interrupts, but interrupts pending,
			 * enable interrupt window exiting.
			 */
			if (vcpu->vc_irqready == 0 && vcpu->vc_intr) {
				vmcb->v_intercept1 |= SVM_INTERCEPT_VINTR;
				vmcb->v_irq = 1;
				vmcb->v_intr_misc = SVM_INTR_MISC_V_IGN_TPR;
				vmcb->v_intr_vector = 0;
				svm_set_dirty(vcpu, SVM_CLEANBITS_TPR |
				    SVM_CLEANBITS_I);
			}

			/*
			 * Exit to vmd if we are terminating, failed to enter,
			 * or need help (device I/O)
			 */
			if (ret || vcpu_must_yield(vcpu))
				break;

			if (vcpu->vc_intr && vcpu->vc_irqready) {
				ret = EAGAIN;
				break;
			}
		}
	}

	/*
	 * We are heading back to userspace (vmd), either because we need help
	 * handling an exit, a guest interrupt is pending, or we failed in some
	 * way to enter the guest. Copy the guest registers to the exit struct
	 * and return to vmd.
	 */
	if (vcpu_readregs_svm(vcpu, VM_RWREGS_ALL, &vcpu->vc_exit.vrs))
		ret = EINVAL;

	return (ret);
}

/*
 * svm_handle_hlt
 *
 * Handle HLT exits
 *
 * Parameters
 *  vcpu: The VCPU that executed the HLT instruction
 *
 * Return Values:
 *  EIO: The guest halted with interrupts disabled
 *  EAGAIN: Normal return to vmd - vmd should halt scheduling this VCPU
 *   until a virtual interrupt is ready to inject
 */
static int
svm_handle_hlt(struct vcpu *vcpu)
{
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;
	uint64_t rflags = vmcb->v_rflags;

	/* All HLT insns are 1 byte */
	vcpu->vc_gueststate.vg_rip += 1;

	if (!(rflags & PSL_I)) {
		DPRINTF("%s: guest halted with interrupts disabled\n",
		    __func__);
		return (EIO);
	}

	return (EAGAIN);
}

/*
 * svm_handle_exit
 *
 * Handle exits from the VM by decoding the exit reason and calling various
 * subhandlers as needed.
 */
static int
svm_handle_exit(struct vcpu *vcpu)
{
	uint64_t exit_reason, rflags;
	int update_rip, ret = 0, guest_cpl;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	update_rip = 0;
	exit_reason = vcpu->vc_gueststate.vg_exit_reason;
	rflags = vcpu->vc_gueststate.vg_rflags;

	switch (exit_reason) {
	case SVM_VMEXIT_VINTR:
		if (!(rflags & PSL_I)) {
			DPRINTF("%s: impossible interrupt window exit "
			    "config\n", __func__);
			ret = EINVAL;
			break;
		}

		/*
		 * Guest is now ready for interrupts, so disable interrupt
		 * window exiting.
		 */
		vmcb->v_irq = 0;
		vmcb->v_intr_vector = 0;
		vmcb->v_intercept1 &= ~SVM_INTERCEPT_VINTR;
		svm_set_dirty(vcpu, SVM_CLEANBITS_TPR | SVM_CLEANBITS_I);

		update_rip = 0;
		break;
	case SVM_VMEXIT_INTR:
		update_rip = 0;
		break;
	case SVM_VMEXIT_SHUTDOWN:
		update_rip = 0;
		ret = EAGAIN;
		break;
	case SVM_VMEXIT_NPF:
		ret = svm_handle_np_fault(vcpu);
		break;
	case SVM_VMEXIT_CPUID:
		ret = vmm_handle_cpuid(vcpu);
		update_rip = 1;
		break;
	case SVM_VMEXIT_MSR:
		ret = svm_handle_msr(vcpu);
		update_rip = 1;
		break;
	case SVM_VMEXIT_XSETBV:
		ret = svm_handle_xsetbv(vcpu);
		update_rip = 1;
		break;
	case SVM_VMEXIT_IOIO:
		if (svm_handle_inout(vcpu) == 0)
			ret = EAGAIN;
		break;
	case SVM_VMEXIT_HLT:
		ret = svm_handle_hlt(vcpu);
		update_rip = 1;
		break;
	case SVM_VMEXIT_MWAIT:
	case SVM_VMEXIT_MWAIT_CONDITIONAL:
	case SVM_VMEXIT_MONITOR:
	case SVM_VMEXIT_VMRUN:
	case SVM_VMEXIT_VMLOAD:
	case SVM_VMEXIT_VMSAVE:
	case SVM_VMEXIT_STGI:
	case SVM_VMEXIT_CLGI:
	case SVM_VMEXIT_SKINIT:
	case SVM_VMEXIT_RDTSCP:
	case SVM_VMEXIT_ICEBP:
	case SVM_VMEXIT_INVLPGA:
		vmm_inject_ud(vcpu);
		update_rip = 0;
		break;
	case SVM_VMEXIT_VMMCALL:
		guest_cpl = vmm_get_guest_cpu_cpl(vcpu);
		if (guest_cpl == 0 &&
		    vcpu->vc_gueststate.vg_rax == HVCALL_FORCED_ABORT)
			return (EINVAL);
		DPRINTF("SVM_VMEXIT_VMMCALL at cpl=%d\n", guest_cpl);
		vmm_inject_ud(vcpu);
		update_rip = 0;
		break;
	default:
		DPRINTF("%s: unhandled exit 0x%lx (%s)\n", __func__,
		    (u_long)exit_reason,
		    svm_exit_reason_decode((uint32_t)exit_reason));
		return (EINVAL);
	}

	if (update_rip) {
		vmcb->v_rip = vcpu->vc_gueststate.vg_rip;

		if (rflags & PSL_T)
			vmm_inject_db(vcpu);
	}

	/* Enable SVME in EFER (must always be set) */
	vmcb->v_efer |= EFER_SVM;
	svm_set_dirty(vcpu, SVM_CLEANBITS_CR);

	return (ret);
}

/*
 * vmm_inject_gp
 *
 * Injects an #GP exception into the guest VCPU.
 *
 * Parameters:
 *  vcpu: vcpu to inject into
 */
static void
vmm_inject_gp(struct vcpu *vcpu)
{
	DPRINTF("%s: injecting #GP at guest %%rip 0x%lx\n", __func__,
	    (u_long)vcpu->vc_gueststate.vg_rip);
	vcpu->vc_inject.vie_vector = VMM_EX_GP;
	vcpu->vc_inject.vie_type = VCPU_INJECT_EX;
	vcpu->vc_inject.vie_errorcode = 0;
}

/*
 * vmm_inject_ud
 *
 * Injects an #UD exception into the guest VCPU.
 *
 * Parameters:
 *  vcpu: vcpu to inject into
 */
static void
vmm_inject_ud(struct vcpu *vcpu)
{
	DPRINTF("%s: injecting #UD at guest %%rip 0x%lx\n", __func__,
	    (u_long)vcpu->vc_gueststate.vg_rip);
	vcpu->vc_inject.vie_vector = VMM_EX_UD;
	vcpu->vc_inject.vie_type = VCPU_INJECT_EX;
	vcpu->vc_inject.vie_errorcode = 0;
}

/*
 * vmm_inject_db
 *
 * Injects a #DB exception into the guest VCPU.
 *
 * Parameters:
 *  vcpu: vcpu to inject into
 */
static void
vmm_inject_db(struct vcpu *vcpu)
{
	DPRINTF("%s: injecting #DB at guest %%rip 0x%lx\n", __func__,
	    (u_long)vcpu->vc_gueststate.vg_rip);
	vcpu->vc_inject.vie_vector = VMM_EX_DB;
	vcpu->vc_inject.vie_type = VCPU_INJECT_EX;
	vcpu->vc_inject.vie_errorcode = 0;
}

/*
 * vmm_get_guest_memtype
 *
 * Returns the type of memory 'gpa' refers to in the context of vm 'vm'
 */
static int
vmm_get_guest_memtype(struct vm *vm, vm_paddr_t gpa)
{
	int i;
	struct vm_mem_range *vmr;

	/* XXX Use binary search? */
	for (i = 0; i < vm->vm_nmemranges; i++) {
		vmr = &vm->vm_memranges[i];

		/*
		 * vm_memranges are ascending. gpa can no longer be in one of
		 * the memranges
		 */
		if (gpa < vmr->vmr_gpa)
			break;

		if (gpa < vmr->vmr_gpa + vmr->vmr_size) {
			if (vmr->vmr_type == VM_MEM_MMIO)
				return (VMM_MEM_TYPE_MMIO);
			return (VMM_MEM_TYPE_REGULAR);
		}
	}

	DPRINTF("guest memtype @ 0x%lx unknown\n", (u_long)gpa);
	return (VMM_MEM_TYPE_UNKNOWN);
}

/*
 * svm_fault_page
 *
 * Resolve a nested page fault at guest physical address 'gpa' by faulting
 * the backing page into the VM's nested address space.  The guest memory
 * objects are mapped into the nested vmspace at their guest physical
 * address, so vm_fault(9) does all the work (including installing the
 * translation in the RVI page tables).
 *
 * Return values:
 *  0: if successful
 *  EFAULT: the page could not be faulted in
 */
static int
svm_fault_page(struct vcpu *vcpu, vm_paddr_t gpa)
{
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;
	vm_map_t map = &vcpu->vc_parent->vm_vmspace->vm_map;
	vm_prot_t ftype;
	int rv;

	/*
	 * EXITINFO1 for #NPF has the same layout as a #PF error code:
	 * bit 1 = write access, bit 4 = instruction fetch.
	 */
	if (vmcb->v_exitinfo1 & 0x10)
		ftype = VM_PROT_EXECUTE;
	else if (vmcb->v_exitinfo1 & 0x2)
		ftype = VM_PROT_WRITE;
	else
		ftype = VM_PROT_READ;

	rv = vm_fault(map, trunc_page(gpa), ftype, VM_FAULT_NORMAL, NULL);
	if (rv != KERN_SUCCESS) {
		printf("%s: vm_fault failed %d gpa=0x%lx type=%d\n", __func__,
		    rv, (u_long)gpa, ftype);
		return (EFAULT);
	}

	return (0);
}

/*
 * svm_handle_np_fault
 *
 * High level nested paging handler for SVM. Verifies that a fault is for a
 * valid memory region, then faults a page, or aborts otherwise.
 */
static int
svm_handle_np_fault(struct vcpu *vcpu)
{
	uint64_t gpa;
	int gpa_memtype, ret = 0;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;
	struct vm_exit_eptviolation *vee = &vcpu->vc_exit.vee;

	memset(vee, 0, sizeof(*vee));

	gpa = vmcb->v_exitinfo2;

	gpa_memtype = vmm_get_guest_memtype(vcpu->vc_parent, gpa);
	switch (gpa_memtype) {
	case VMM_MEM_TYPE_REGULAR:
		vee->vee_fault_type = VEE_FAULT_HANDLED;
		ret = svm_fault_page(vcpu, gpa);
		break;
	case VMM_MEM_TYPE_MMIO:
		vee->vee_fault_type = VEE_FAULT_MMIO_ASSIST;
		if (ovmm_decode_assist()) {
			vee->vee_insn_len = vmcb->v_n_bytes_fetched;
			memcpy(&vee->vee_insn_bytes, vmcb->v_guest_ins_bytes,
			    sizeof(vee->vee_insn_bytes));
			vee->vee_insn_info |= VEE_BYTES_VALID;
		}
		ret = EAGAIN;
		break;
	default:
		printf("%s: unknown memory type %d for GPA 0x%lx\n",
		    __func__, gpa_memtype, (u_long)gpa);
		return (EINVAL);
	}

	return (ret);
}

/*
 * vmm_get_guest_cpu_cpl
 *
 * Determines current CPL of 'vcpu'. This is gathered directly from the
 * VMCB's 'cpl' field, as per the APM.
 *
 * Parameters:
 *  vcpu: guest VCPU for which CPL is to be checked
 *
 * Return Values:
 *  -1: the CPL could not be determined
 *  0-3 indicating the current CPL. For real mode operation, 0 is returned.
 */
static int
vmm_get_guest_cpu_cpl(struct vcpu *vcpu)
{
	int mode;
	struct vmcb *vmcb;

	mode = vmm_get_guest_cpu_mode(vcpu);

	if (mode == VMM_CPU_MODE_UNKNOWN)
		return (-1);

	if (mode == VMM_CPU_MODE_REAL)
		return (0);

	vmcb = (struct vmcb *)vcpu->vc_control_va;
	return (vmcb->v_cpl);
}

/*
 * vmm_get_guest_cpu_mode
 *
 * Determines current CPU mode of 'vcpu'.
 *
 * Parameters:
 *  vcpu: guest VCPU for which mode is to be checked
 *
 * Return Values:
 *  One of VMM_CPU_MODE_*, or VMM_CPU_MODE_UNKNOWN if the mode could not be
 *   ascertained.
 */
static int
vmm_get_guest_cpu_mode(struct vcpu *vcpu)
{
	uint64_t cr0, efer, cs_ar;
	uint8_t l, dib;
	struct vmcb *vmcb;

	vmcb = (struct vmcb *)vcpu->vc_control_va;
	cr0 = vmcb->v_cr0;
	efer = vmcb->v_efer;
	cs_ar = vmcb->v_cs.vs_attr;
	cs_ar = (cs_ar & 0xff) | ((cs_ar << 4) & 0xf000);

	l = (cs_ar & 0x2000) >> 13;
	dib = (cs_ar & 0x4000) >> 14;

	/* Check CR0.PE */
	if (!(cr0 & CR0_PE))
		return (VMM_CPU_MODE_REAL);

	/* Check EFER */
	if (efer & EFER_LMA) {
		/* Could be compat or long mode, check CS.L */
		if (l)
			return (VMM_CPU_MODE_LONG);
		else
			return (VMM_CPU_MODE_COMPAT);
	}

	/* Check prot vs prot32 */
	if (dib)
		return (VMM_CPU_MODE_PROT32);
	else
		return (VMM_CPU_MODE_PROT);
}

/*
 * svm_handle_inout
 *
 * Exit handler for IN/OUT instructions.
 *
 * Parameters:
 *  vcpu: The VCPU where the IN/OUT instruction occurred
 *
 * Return values:
 *  0: if successful
 */
static int
svm_handle_inout(struct vcpu *vcpu)
{
	uint64_t insn_length, exit_qual;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	insn_length = vmcb->v_exitinfo2 - vmcb->v_rip;
	exit_qual = vmcb->v_exitinfo1;

	/* Bit 0 - direction */
	if (exit_qual & 0x1)
		vcpu->vc_exit.vei.vei_dir = VEI_DIR_IN;
	else
		vcpu->vc_exit.vei.vei_dir = VEI_DIR_OUT;
	/* Bit 2 - string instruction? */
	vcpu->vc_exit.vei.vei_string = (exit_qual & 0x4) >> 2;
	/* Bit 3 - REP prefix? */
	vcpu->vc_exit.vei.vei_rep = (exit_qual & 0x8) >> 3;

	/* Bits 4:6 - size of exit */
	if (exit_qual & 0x10)
		vcpu->vc_exit.vei.vei_size = 1;
	else if (exit_qual & 0x20)
		vcpu->vc_exit.vei.vei_size = 2;
	else if (exit_qual & 0x40)
		vcpu->vc_exit.vei.vei_size = 4;

	/* Bit 16:31 - port */
	vcpu->vc_exit.vei.vei_port = (exit_qual & 0xFFFF0000) >> 16;
	/* Data */
	vcpu->vc_exit.vei.vei_data = vmcb->v_rax;

	vcpu->vc_exit.vei.vei_insn_len = (uint8_t)insn_length;

	return (0);
}

/*
 * svm_handle_xsetbv
 *
 * SVM-specific part of the xsetbv instruction exit handler
 *
 * Parameters:
 *  vcpu: vcpu structure containing instruction info causing the exit
 *
 * Return value:
 *  0: The operation was successful
 *  EINVAL: An error occurred
 */
static int
svm_handle_xsetbv(struct vcpu *vcpu)
{
	uint64_t insn_length, *rax;
	int ret;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	/* All XSETBV instructions are 3 bytes */
	insn_length = 3;

	rax = &vmcb->v_rax;

	ret = vmm_handle_xsetbv(vcpu, rax);

	vcpu->vc_gueststate.vg_rip += insn_length;

	return ret;
}

/*
 * vmm_handle_xsetbv
 *
 * Handler for xsetbv instructions. We allow the guest VM to set xcr0 values
 * limited to the xsave_mask in use in the host.
 *
 * Parameters:
 *  vcpu: vcpu structure containing instruction info causing the exit
 *  rax: pointer to guest %rax
 *
 * Return value:
 *  0: The operation was successful
 *  EINVAL: An error occurred
 */
static int
vmm_handle_xsetbv(struct vcpu *vcpu, uint64_t *rax)
{
	uint64_t *rdx, *rcx, val, mask = xsave_mask;

	rcx = &vcpu->vc_gueststate.vg_rcx;
	rdx = &vcpu->vc_gueststate.vg_rdx;

	if (vmm_get_guest_cpu_cpl(vcpu) != 0) {
		DPRINTF("%s: guest cpl not zero\n", __func__);
		vmm_inject_gp(vcpu);
		return (0);
	}

	if (*rcx != 0) {
		DPRINTF("%s: guest specified invalid xcr register number "
		    "%lu\n", __func__, (u_long)*rcx);
		vmm_inject_gp(vcpu);
		return (0);
	}

	val = *rax + (*rdx << 32);
	if ((val & ~mask) != 0 || (val & XFEATURE_ENABLED_X87) == 0) {
		DPRINTF("%s: guest specified xcr0 outside xsave_mask %lu\n",
		    __func__, (u_long)val);
		vmm_inject_gp(vcpu);
		return (0);
	}

	vcpu->vc_gueststate.vg_xcr0 = val;

	return (0);
}

/*
 * svm_handle_msr
 *
 * Handler for MSR instructions.
 *
 * Parameters:
 *  vcpu: vcpu structure containing instruction info causing the exit
 *
 * Return value:
 *  Always 0 (successful)
 */
static int
svm_handle_msr(struct vcpu *vcpu)
{
	uint64_t insn_length, val;
	uint64_t *rax, *rcx, *rdx;
	struct vmcb *vmcb = (struct vmcb *)vcpu->vc_control_va;

	/* XXX: Validate RDMSR / WRMSR insn_length */
	insn_length = 2;

	rax = &vmcb->v_rax;
	rcx = &vcpu->vc_gueststate.vg_rcx;
	rdx = &vcpu->vc_gueststate.vg_rdx;

	if (vmcb->v_exitinfo1 == 1) {
		/* WRMSR */
		val = (*rdx << 32) | (*rax & 0xFFFFFFFFULL);

		switch (*rcx) {
		case MSR_PAT:
			if (!vmm_pat_is_valid(val)) {
				vmm_inject_gp(vcpu);
				return (0);
			}
			vcpu->vc_shadow_pat = val;
			break;
		case MSR_EFER:
			vmcb->v_efer = *rax | EFER_SVM;
			break;
		case KVM_MSR_SYSTEM_TIME:
			vmm_init_pvclock(vcpu,
			    (*rax & 0xFFFFFFFFULL) | (*rdx  << 32));
			break;
		case KVM_MSR_WALL_CLOCK:
			vmm_pv_wall_clock(vcpu,
			    (*rax & 0xFFFFFFFFULL) | (*rdx  << 32));
			break;
		default:
			/* Log the access, to be able to identify unknown MSRs */
			DPRINTF("%s: wrmsr exit, msr=0x%lx, discarding data "
			    "written from guest=0x%lx:0x%lx\n", __func__,
			    (u_long)*rcx, (u_long)*rdx, (u_long)*rax);
		}
	} else {
		/* RDMSR */
		switch (*rcx) {
		case MSR_BIOS_SIGN:
		case OVMM_MSR_INT_PEN_MSG:
		case MSR_IA32_PLATFORM_ID:
		case MSR_SYSCFG:
			/* Ignored */
			*rax = 0;
			*rdx = 0;
			break;
		case MSR_PAT:
			*rax = (vcpu->vc_shadow_pat & 0xFFFFFFFFULL);
			*rdx = (vcpu->vc_shadow_pat >> 32);
			break;
		case MSR_DE_CFG:
			/* LFENCE serializing bit is set by host */
			*rax = OVMM_DE_CFG_SERIALIZE_LFENCE;
			*rdx = 0;
			break;
		default:
			/*
			 * Unsupported MSRs causes #GP exception, don't advance
			 * %rip
			 */
			DPRINTF("%s: unsupported rdmsr (msr=0x%lx), "
			    "injecting #GP\n", __func__, (u_long)*rcx);
			vmm_inject_gp(vcpu);
			return (0);
		}
	}

	vcpu->vc_gueststate.vg_rip += insn_length;

	return (0);
}

/* Handle cpuid(0xd) and its subleafs */
static void
vmm_handle_cpuid_0xd(struct vcpu *vcpu, uint32_t subleaf, uint64_t *rax,
    uint32_t eax, uint32_t ebx, uint32_t ecx, uint32_t edx)
{
	uint64_t xcr0 = vcpu->vc_gueststate.vg_xcr0;
	uint64_t host_xcr0 = vmm_softc->sc_md.host_xcr0;
	u_int regs[4];
	register_t s;

	if (subleaf == 0) {
		/*
		 * CPUID(0xd.0) depends on the value in XCR0 and MSR_XSS.  If
		 * the guest XCR0 isn't the same as the host then set it, redo
		 * the CPUID, and restore it.
		 */

		/*
		 * "ecx enumerates the size required ... for an area
		 *  containing all the ... components supported by this
		 *  processor"
		 * "ebx enumerates the size required ... for an area
		 *  containing all the ... components corresponding to bits
		 *  currently set in xcr0"
		 * So: since the VMM 'processor' is what our base kernel uses,
		 * the VMM ecx is our ebx
		 */
		ecx = ebx;
		if (xsave_mask != 0 && xcr0 != host_xcr0) {
			s = intr_disable();
			load_xcr(0, xcr0);
			cpuid_count(0xd, subleaf, regs);
			load_xcr(0, host_xcr0);
			intr_restore(s);
			ebx = regs[1];
		}
		eax = xsave_mask & 0xffffffff;
		edx = xsave_mask >> 32;
	} else if (subleaf == 1) {
		/* mask out XSAVEC, XSAVES, and XFD support */
		eax &= OVMM_XSAVE_XSAVEOPT | OVMM_XSAVE_XGETBV1;
		ebx = 0;	/* no xsavec or xsaves for now */
		ecx = edx = 0;	/* no xsaves for now */
	} else if (subleaf >= 63 ||
	    ((1ULL << subleaf) & xsave_mask) == 0) {
		/* disclaim subleaves of features we don't expose */
		eax = ebx = ecx = edx = 0;
	} else {
		/* disclaim compressed alignment or xfd support */
		ecx = 0;
	}

	*rax = eax;
	vcpu->vc_gueststate.vg_rbx = ebx;
	vcpu->vc_gueststate.vg_rcx = ecx;
	vcpu->vc_gueststate.vg_rdx = edx;
}

/*
 * vmm_handle_cpuid
 *
 * Exit handler for CPUID instruction
 *
 * Parameters:
 *  vcpu: vcpu causing the CPUID exit
 *
 * Return value:
 *  0: the exit was processed successfully
 */
static int
vmm_handle_cpuid(struct vcpu *vcpu)
{
	uint64_t insn_length, cr4;
	uint64_t *rax, *rbx, *rcx, *rdx;
	struct vmcb *vmcb;
	uint32_t leaf, subleaf, eax, ebx, ecx, edx;
	u_int regs[4];
	uint32_t vmm_cpuid_level;

	/* what's the cpuid level we support/advertise? */
	vmm_cpuid_level = cpu_high;
	if (vmm_cpuid_level < 0x15 && tsc_is_invariant)
		vmm_cpuid_level = 0x15;

	/* XXX: validate insn_length 2 */
	insn_length = 2;
	vmcb = (struct vmcb *)vcpu->vc_control_va;
	rax = &vmcb->v_rax;
	cr4 = vmcb->v_cr4;

	rbx = &vcpu->vc_gueststate.vg_rbx;
	rcx = &vcpu->vc_gueststate.vg_rcx;
	rdx = &vcpu->vc_gueststate.vg_rdx;
	vcpu->vc_gueststate.vg_rip += insn_length;

	leaf = *rax;
	subleaf = *rcx;

	/*
	 * "If a value entered for CPUID.EAX is higher than the maximum input
	 *  value for basic or extended function for that processor then the
	 *  data for the highest basic information leaf is returned."
	 *
	 * "When CPUID returns the highest basic leaf information as a result
	 *  of an invalid input EAX value, any dependence on input ECX value
	 *  in the basic leaf is honored."
	 *
	 * This means if leaf is between vmm_cpuid_level and 0x40000000 (the
	 * start of the hypervisor info leaves), clamp to vmm_cpuid_level, but
	 * without altering subleaf.  Also, if leaf is greater than the
	 * extended function info, clamp also to vmm_cpuid_level.
	 */
	if ((leaf > vmm_cpuid_level && leaf < 0x40000000) ||
	    (leaf > cpu_exthigh)) {
		DPRINTF("%s: invalid cpuid input leaf 0x%x, guest rip="
		    "0x%lx - resetting to 0x%x\n", __func__, leaf,
		    (u_long)(vcpu->vc_gueststate.vg_rip - insn_length),
		    vmm_cpuid_level);
		leaf = vmm_cpuid_level;
	}

	/* we fake up values in the range (cpu_high, vmm_cpuid_level] */
	if (leaf <= cpu_high || leaf > 0x80000000) {
		cpuid_count(leaf, subleaf, regs);
		eax = regs[0];
		ebx = regs[1];
		ecx = regs[2];
		edx = regs[3];
	} else
		eax = ebx = ecx = edx = 0;

	switch (leaf) {
	case 0x00:	/* Max level and vendor ID */
		*rax = vmm_cpuid_level;
		*rbx = *((const uint32_t *)&cpu_vendor[0]);
		*rdx = *((const uint32_t *)&cpu_vendor[4]);
		*rcx = *((const uint32_t *)&cpu_vendor[8]);
		break;
	case 0x01:	/* Version, brand, feature info */
		*rax = cpu_id;
		/* mask off host's APIC ID, reset to vcpu id */
		*rbx = cpu_procinfo & 0x0000FFFF;
		*rbx |= (vcpu->vc_id & 0xFF) << 24;
		*rcx = (cpu_feature2 | CPUID2_HV) & VMM_CPUIDECX_MASK;

		/* Guest CR4.OSXSAVE determines presence of CPUIDECX_OSXSAVE */
		if (cr4 & CR4_XSAVE)
			*rcx |= CPUID2_OSXSAVE;
		else
			*rcx &= ~CPUID2_OSXSAVE;

		*rdx = cpu_feature & VMM_CPUIDEDX_MASK;
		break;
	case 0x02:	/* Cache and TLB information */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x03:	/* Processor serial number (not supported) */
		DPRINTF("%s: function 0x03 (processor serial number) not "
		"supported\n", __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x04:	/* Deterministic cache info */
		*rax = eax & VMM_CPUID4_CACHE_TOPOLOGY_MASK;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x05:	/* MONITOR/MWAIT (not supported) */
		DPRINTF("%s: function 0x05 (monitor/mwait) not supported\n",
		    __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x06:	/* Thermal / Power management (not supported) */
		DPRINTF("%s: function 0x06 (thermal/power mgt) not supported\n",
		    __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x07:	/* SEFF */
		if (subleaf == 0) {
			*rax = 0;	/* Highest subleaf supported */
			*rbx = cpu_stdext_feature & VMM_SEFF0EBX_MASK;
			*rcx = cpu_stdext_feature2 & VMM_SEFF0ECX_MASK;
			*rdx = cpu_stdext_feature3 & VMM_SEFF0EDX_MASK;
			/*
			 * Only expose PKU support if we've detected it in use
			 * on the host.
			 */
			if (vmm_softc->sc_md.pkru_enabled)
				*rcx |= CPUID_STDEXT2_PKU;
			else
				*rcx &= ~CPUID_STDEXT2_PKU;
		} else {
			/* Unsupported subleaf */
			DPRINTF("%s: function 0x07 (SEFF) unsupported subleaf "
			    "0x%x not supported\n", __func__, subleaf);
			*rax = 0;
			*rbx = 0;
			*rcx = 0;
			*rdx = 0;
		}
		break;
	case 0x09:	/* Direct Cache Access (not supported) */
		DPRINTF("%s: function 0x09 (direct cache access) not "
		    "supported\n", __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x0a:	/* Architectural perf monitoring (not supported) */
		DPRINTF("%s: function 0x0a (arch. perf mon) not supported\n",
		    __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x0b:	/* Extended topology enumeration (not supported) */
		DPRINTF("%s: function 0x0b (topology enumeration) not "
		    "supported\n", __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x0d:	/* Processor ext. state information */
		vmm_handle_cpuid_0xd(vcpu, subleaf, rax, eax, ebx, ecx, edx);
		break;
	case 0x0f:	/* QoS info (not supported) */
		DPRINTF("%s: function 0x0f (QoS info) not supported\n",
		    __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x14:	/* Processor Trace info (not supported) */
		DPRINTF("%s: function 0x14 (processor trace info) not "
		    "supported\n", __func__);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x15:
		if (cpu_high >= 0x15) {
			*rax = eax;
			*rbx = ebx;
			*rcx = ecx;
			*rdx = edx;
		} else {
			KASSERT(tsc_is_invariant, ("ovmm: leaf 0x15 faked "
			    "without invariant TSC"));
			*rax = 1;
			*rbx = 100;
			*rcx = tsc_freq / 100;
			*rdx = 0;
		}
		break;
	case 0x16:	/* Processor frequency info */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x40000000:	/* Hypervisor information */
		*rax = 0;
		*rbx = *((const uint32_t *)&vmm_hv_signature[0]);
		*rcx = *((const uint32_t *)&vmm_hv_signature[4]);
		*rdx = *((const uint32_t *)&vmm_hv_signature[8]);
		break;
	case 0x40000001:	/* KVM hypervisor features */
		if (tsc_freq > 0)
			*rax = KVM_FEATURE_CLOCKSOURCE2 |
			    KVM_FEATURE_CLOCKSOURCE_STABLE_BIT;
		else
			*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x40000100:	/* Hypervisor information KVM */
		*rax = 0x40000101;
		*rbx = *((const uint32_t *)&kvm_hv_signature[0]);
		*rcx = *((const uint32_t *)&kvm_hv_signature[4]);
		*rdx = *((const uint32_t *)&kvm_hv_signature[8]);
		break;
	case 0x40000101:	/* KVM hypervisor features */
		*rax = OVMM_KVM_FEATURE_NOP_IO_DELAY;
		if (tsc_freq > 0)
			*rax |= KVM_FEATURE_CLOCKSOURCE2 |
			    KVM_FEATURE_CLOCKSOURCE_STABLE_BIT;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x80000000:	/* Extended function level */
		/* We don't emulate past 0x8000001f currently. */
		*rax = min(cpu_exthigh, 0x8000001f);
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
		break;
	case 0x80000001:	/* Extended function info */
		*rax = eax;
		*rbx = 0;	/* Reserved */
		*rcx = amd_feature2 & VMM_ECPUIDECX_MASK;
		*rdx = amd_feature & VMM_FEAT_EFLAGS_MASK;
		break;
	case 0x80000002:	/* Brand string */
	case 0x80000003:	/* Brand string */
	case 0x80000004:	/* Brand string */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x80000005:	/* Reserved (Intel), cacheinfo (AMD) */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x80000006:	/* ext. cache info */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x80000007:	/* apmi */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx & VMM_APMI_EDX_INCLUDE_MASK;
		break;
	case 0x80000008:	/* Phys bits info and topology (AMD) */
		*rax = eax;
		*rbx = ebx & VMM_AMDSPEC_EBX_MASK;
		/* Reset %rcx (topology) */
		*rcx = 0;
		*rdx = edx;
		break;
	case 0x8000001d:	/* cache topology (AMD) */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	case 0x8000001f:	/* encryption features (AMD) */
		*rax = eax;
		*rbx = ebx;
		*rcx = ecx;
		*rdx = edx;
		break;
	default:
		DPRINTF("%s: unsupported rax=0x%lx\n", __func__, (u_long)*rax);
		*rax = 0;
		*rbx = 0;
		*rcx = 0;
		*rdx = 0;
	}

	/*
	 * %rax lives in the VMCB; the rest of the registers get loaded in
	 * svm_enter_guest.
	 */
	vmcb->v_rax = *rax;

	return (0);
}

/*
 * vmm_alloc_asid
 *
 * Sets the memory location pointed to by "asid" to the next available ASID.
 *
 * Parameters:
 *  asid: Pointer to location to receive the next ASID
 *
 * Return Values:
 *  0: The operation completed successfully
 *  ENOMEM: No ASIDs were available. Content of 'asid' is unchanged.
 */
static int
vmm_alloc_asid(uint16_t *asid)
{
	uint16_t i;
	uint8_t idx, bit;
	struct vmm_softc *sc = vmm_softc;

	mtx_lock(&sc->vpid_lock);
	for (i = 1; i <= sc->max_vpid; i++) {
		idx = i / 8;
		bit = i - (idx * 8);

		if (!(sc->vpids[idx] & (1 << bit))) {
			sc->vpids[idx] |= (1 << bit);
			*asid = i;
			DPRINTF("%s: allocated ASID %d\n", __func__, i);
			mtx_unlock(&sc->vpid_lock);
			return 0;
		}
	}

	printf("%s: no available ASIDs\n", __func__);
	mtx_unlock(&sc->vpid_lock);
	return ENOMEM;
}

/*
 * vmm_free_asid
 *
 * Frees the ASID supplied in "asid".
 *
 * Parameters:
 *  asid: ASID to free.
 */
static void
vmm_free_asid(uint16_t asid)
{
	uint8_t idx, bit;
	struct vmm_softc *sc = vmm_softc;

	mtx_lock(&sc->vpid_lock);
	idx = asid / 8;
	bit = asid - (idx * 8);
	sc->vpids[idx] &= ~(1 << bit);

	DPRINTF("%s: freed ASID %d\n", __func__, asid);
	mtx_unlock(&sc->vpid_lock);
}

/* vmm_gpa_is_valid
 *
 * Check if the given gpa is within guest memory space.
 *
 * Parameters:
 *	vcpu: The virtual cpu we are running on.
 *	gpa: The address to check.
 *	obj_size: The size of the object assigned to gpa
 *
 * Return values:
 *	1: gpa is within the memory ranges allocated for the vcpu
 *	0: otherwise
 */
static int
vmm_gpa_is_valid(struct vcpu *vcpu, vm_paddr_t gpa, size_t obj_size)
{
	struct vm *vm = vcpu->vc_parent;
	struct vm_mem_range *vmr;
	size_t i;

	for (i = 0; i < vm->vm_nmemranges; ++i) {
		vmr = &vm->vm_memranges[i];
		if (vmr->vmr_type != VM_MEM_MMIO &&
		    vmr->vmr_size >= obj_size &&
		    vmr->vmr_gpa <= gpa &&
		    gpa < (vmr->vmr_gpa + vmr->vmr_size - obj_size)) {
		    return 1;
		}
	}
	return 0;
}

/*
 * ovmm_gpa_hold
 *
 * Fault in and hold the guest page containing 'gpa' and return a kernel
 * virtual address for it via the direct map.  The caller must release the
 * page with ovmm_gpa_release().  'gpa' must not cross a page boundary for
 * the object being accessed.
 *
 * Return values:
 *  NULL: the page could not be faulted in
 */
static void *
ovmm_gpa_hold(struct vm *vm, vm_paddr_t gpa, vm_prot_t prot, vm_page_t *mp)
{
	vm_map_t map = &vm->vm_vmspace->vm_map;
	int count;

	count = vm_fault_quick_hold_pages(map, trunc_page(gpa), PAGE_SIZE,
	    prot, mp, 1);
	if (count != 1)
		return (NULL);

	return ((void *)((uintptr_t)PHYS_TO_DMAP(VM_PAGE_TO_PHYS(*mp)) +
	    (gpa & PAGE_MASK)));
}

static void
ovmm_gpa_release(vm_page_t *mp)
{
	vm_page_unhold_pages(mp, 1);
}

static void
vmm_init_pvclock(struct vcpu *vcpu, vm_paddr_t gpa)
{
	vm_paddr_t pvclock_gpa = gpa & 0xFFFFFFFFFFFFFFF0;

	if (!vmm_gpa_is_valid(vcpu, pvclock_gpa,
	    sizeof(struct pvclock_vcpu_time_info))) {
		/* XXX: Kill guest? */
		vmm_inject_gp(vcpu);
		return;
	}

	/* XXX: handle case when this struct goes over page boundaries */
	if ((pvclock_gpa & PAGE_MASK) + sizeof(struct pvclock_vcpu_time_info) >
	    PAGE_SIZE) {
		vmm_inject_gp(vcpu);
		return;
	}

	vcpu->vc_pvclock_system_gpa = gpa;
	if (tsc_freq > 0)
		vcpu->vc_pvclock_system_tsc_mul =
		    (int) ((1000000000L << 20) / tsc_freq);
	else
		vcpu->vc_pvclock_system_tsc_mul = 0;
	vmm_update_pvclock(vcpu);
}

#define	PVCLOCK_SYSTEM_TIME_ENABLE	0x1

static int
vmm_update_pvclock(struct vcpu *vcpu)
{
	struct pvclock_vcpu_time_info *pvclock_ti;
	struct timespec tv;
	struct vm *vm = vcpu->vc_parent;
	vm_paddr_t pvclock_gpa;
	vm_page_t m;

	if (vcpu->vc_pvclock_system_gpa & PVCLOCK_SYSTEM_TIME_ENABLE) {
		pvclock_gpa = vcpu->vc_pvclock_system_gpa & 0xFFFFFFFFFFFFFFF0;
		pvclock_ti = ovmm_gpa_hold(vm, pvclock_gpa, VM_PROT_WRITE, &m);
		if (pvclock_ti == NULL)
			return (EINVAL);

		/* START next cycle (must be odd) */
		pvclock_ti->version =
		    (++vcpu->vc_pvclock_version << 1) | 0x1;

		pvclock_ti->tsc_timestamp = rdtsc();
		nanouptime(&tv);
		pvclock_ti->system_time =
		    tv.tv_sec * 1000000000L + tv.tv_nsec;
		pvclock_ti->tsc_shift = 12;
		pvclock_ti->tsc_to_system_mul =
		    vcpu->vc_pvclock_system_tsc_mul;
		pvclock_ti->flags = PVCLOCK_FLAG_TSC_STABLE;

		/* END (must be even) */
		pvclock_ti->version &= ~0x1;

		ovmm_gpa_release(&m);
	}
	return (0);
}

static void
vmm_pv_wall_clock(struct vcpu *vcpu, vm_paddr_t gpa)
{
	struct pvclock_wall_clock *pvclock_wc;
	struct timespec tv;
	struct vm *vm = vcpu->vc_parent;
	vm_page_t m;

	if (!vmm_gpa_is_valid(vcpu, gpa, sizeof(struct pvclock_wall_clock)))
		goto err;

	/* XXX: handle case when this struct goes over page boundaries */
	if ((gpa & PAGE_MASK) + sizeof(struct pvclock_wall_clock) > PAGE_SIZE)
		goto err;

	pvclock_wc = ovmm_gpa_hold(vm, gpa, VM_PROT_WRITE, &m);
	if (pvclock_wc == NULL)
		goto err;

	pvclock_wc->version |= 0x1;
	nanotime(&tv);
	pvclock_wc->sec = tv.tv_sec;
	pvclock_wc->nsec = tv.tv_nsec;
	pvclock_wc->version += 1;

	ovmm_gpa_release(&m);
	return;
err:
	vmm_inject_gp(vcpu);
}

static int
vmm_pat_is_valid(uint64_t pat)
{
	int i;
	uint8_t *byte = (uint8_t *)&pat;

	/* Intel SDM Vol 3A, 11.12.2: 0x02, 0x03, and 0x08-0xFF result in #GP */
	for (i = 0; i < 8; i++) {
		if (byte[i] == 0x02 || byte[i] == 0x03 || byte[i] > 0x07) {
			DPRINTF("%s: invalid pat %lx\n", __func__, (u_long)pat);
			return 0;
		}
	}

	return 1;
}

/*
 * svm_exit_reason_decode
 *
 * Returns a human readable string describing exit type 'code'
 */
const char *
svm_exit_reason_decode(uint32_t code)
{
	switch (code) {
	case SVM_VMEXIT_CR0_READ ... SVM_VMEXIT_CR15_READ:
		return "CR read";
	case SVM_VMEXIT_CR0_WRITE ... SVM_VMEXIT_CR15_WRITE:
		return "CR write";
	case SVM_VMEXIT_DR0_READ ... SVM_VMEXIT_DR15_READ:
		return "DR read";
	case SVM_VMEXIT_DR0_WRITE ... SVM_VMEXIT_DR15_WRITE:
		return "DR write";
	case SVM_VMEXIT_EXCP0 ... SVM_VMEXIT_EXCP31:
		return "exception";
	case SVM_VMEXIT_INTR: return "external interrupt";
	case SVM_VMEXIT_NMI: return "NMI";
	case SVM_VMEXIT_SMI: return "SMI";
	case SVM_VMEXIT_INIT: return "INIT";
	case SVM_VMEXIT_VINTR: return "interrupt window";
	case SVM_VMEXIT_CR0_SEL_WRITE: return "selective CR0 write";
	case SVM_VMEXIT_IDTR_READ: return "IDTR read";
	case SVM_VMEXIT_GDTR_READ: return "GDTR read";
	case SVM_VMEXIT_LDTR_READ: return "LDTR read";
	case SVM_VMEXIT_TR_READ: return "TR read";
	case SVM_VMEXIT_IDTR_WRITE: return "IDTR write";
	case SVM_VMEXIT_GDTR_WRITE: return "GDTR write";
	case SVM_VMEXIT_LDTR_WRITE: return "LDTR write";
	case SVM_VMEXIT_TR_WRITE: return "TR write";
	case SVM_VMEXIT_RDTSC: return "RDTSC instruction";
	case SVM_VMEXIT_RDPMC: return "RDPMC instruction";
	case SVM_VMEXIT_PUSHF: return "PUSHF instruction";
	case SVM_VMEXIT_POPF: return "POPF instruction";
	case SVM_VMEXIT_CPUID: return "CPUID instruction";
	case SVM_VMEXIT_RSM: return "RSM instruction";
	case SVM_VMEXIT_IRET: return "IRET instruction";
	case SVM_VMEXIT_SWINT: return "software interrupt";
	case SVM_VMEXIT_INVD: return "INVD instruction";
	case SVM_VMEXIT_PAUSE: return "PAUSE instruction";
	case SVM_VMEXIT_HLT: return "HLT instruction";
	case SVM_VMEXIT_INVLPG: return "INVLPG instruction";
	case SVM_VMEXIT_INVLPGA: return "INVLPGA instruction";
	case SVM_VMEXIT_IOIO: return "I/O instruction";
	case SVM_VMEXIT_MSR: return "RDMSR/WRMSR instruction";
	case SVM_VMEXIT_TASK_SWITCH: return "task switch";
	case SVM_VMEXIT_FERR_FREEZE: return "FERR freeze";
	case SVM_VMEXIT_SHUTDOWN: return "triple fault";
	case SVM_VMEXIT_VMRUN: return "VMRUN instruction";
	case SVM_VMEXIT_VMMCALL: return "VMMCALL instruction";
	case SVM_VMEXIT_VMLOAD: return "VMLOAD instruction";
	case SVM_VMEXIT_VMSAVE: return "VMSAVE instruction";
	case SVM_VMEXIT_STGI: return "STGI instruction";
	case SVM_VMEXIT_CLGI: return "CLGI instruction";
	case SVM_VMEXIT_SKINIT: return "SKINIT instruction";
	case SVM_VMEXIT_RDTSCP: return "RDTSCP instruction";
	case SVM_VMEXIT_ICEBP: return "ICEBP instruction";
	case SVM_VMEXIT_WBINVD: return "WBINVD instruction";
	case SVM_VMEXIT_MONITOR: return "MONITOR instruction";
	case SVM_VMEXIT_MWAIT: return "MWAIT instruction";
	case SVM_VMEXIT_MWAIT_CONDITIONAL: return "MWAIT (conditional)";
	case SVM_VMEXIT_XSETBV: return "XSETBV instruction";
	case SVM_VMEXIT_NPF: return "nested page fault";
	case SVM_VMEXIT_INVALID: return "invalid guest state in VMCB";
	default: return "unknown";
	}
}
