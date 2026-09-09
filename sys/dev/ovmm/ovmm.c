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
 * ovmm(4): machine independent part of the OpenBSD vmm(4) port.
 *
 * This is a port of OpenBSD's sys/dev/vmm/vmm.c.  Guest memory handling
 * was rewritten for the FreeBSD VM system:
 *
 *  - every guest RAM range is backed by an anonymous (swap) vm_object,
 *  - the object is mapped into the creating process (like the uvm aobj in
 *    the original) so the monitor can access guest memory directly, and
 *  - the same object is mapped into the VM's private nested vmspace at the
 *    guest physical address, which lets the SVM nested page fault handler
 *    resolve faults with vm_fault(9).
 *
 * pledge(2) related access control was replaced by a simple rule: a VM can
 * only be manipulated by the process that created it, or by a process with
 * PRIV_DRIVER.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sx.h>
#include <sys/proc.h>
#include <sys/priv.h>
#include <sys/refcount.h>
#include <sys/signalvar.h>
#include <sys/smp.h>
#include <sys/uio.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_extern.h>

#include <dev/ovmm/ovmm.h>

MALLOC_DEFINE(M_OVMM, "ovmm", "OpenBSD vmm port");

struct vmm_softc *vmm_softc;

static d_open_t ovmm_open;
static d_ioctl_t ovmm_ioctl;

static struct cdevsw ovmm_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	"ovmm",
	.d_open =	ovmm_open,
	.d_ioctl =	ovmm_ioctl,
};

static int ovmm_attach(void);
static void ovmm_detach(void);

/*
 * ovmm_open
 *
 * Called during open of /dev/ovmm.
 *
 * Return values:
 *  ENODEV: if ovmm(4) didn't attach or no supported CPUs detected
 *  0: successful open
 */
static int
ovmm_open(struct cdev *dev, int flag, int mode, struct thread *td)
{
	/* Don't allow open if we didn't attach */
	if (vmm_softc == NULL)
		return (ENODEV);

	/* Don't allow open if we didn't detect any supported CPUs */
	if (vmm_softc->mode == VMM_MODE_UNKNOWN)
		return (ENODEV);

	return 0;
}

/*
 * vm_find
 *
 * Function to find an existing VM by its identifier.
 *
 * Parameters:
 *  id: The VM identifier.
 *  *res: A pointer to the VM or NULL if not found
 *
 * Return values:
 *  0: if successful; the caller holds a reference that must be dropped
 *     with vm_rele()
 *  ENOENT: if the VM defined by 'id' cannot be found
 *  EPERM: if the VM cannot be accessed by the current process
 */
int
vm_find(uint32_t id, struct vm **res)
{
	struct thread *td = curthread;
	struct vm *vm;
	int ret = ENOENT;

	*res = NULL;

	sx_slock(&vmm_softc->vm_lock);
	SLIST_FOREACH(vm, &vmm_softc->vm_list, vm_link) {
		if (vm->vm_id == id) {
			/*
			 * Only allow the creating process (or a privileged
			 * one) to find the VM.
			 */
			if (vm->vm_creator_pid != td->td_proc->p_pid &&
			    priv_check(td, PRIV_DRIVER) != 0)
				ret = EPERM;
			else {
				refcount_acquire(&vm->vm_refcnt);
				*res = vm;
				ret = 0;
			}
			break;
		}
	}
	sx_sunlock(&vmm_softc->vm_lock);

	return (ret);
}

/*
 * vm_rele
 *
 * Drop a reference obtained with vm_find(). Wakes up a thread waiting in
 * vm_terminate() for the last reference to go away.
 */
void
vm_rele(struct vm *vm)
{
	if (refcount_release(&vm->vm_refcnt))
		wakeup(__DEVOLATILE(void *, &vm->vm_refcnt));
}

/*
 * ovmm_ioctl
 *
 * Main ioctl dispatch routine for /dev/ovmm. Parses ioctl type and calls
 * appropriate lower level handler routine. Returns result to ioctl caller.
 */
static int
ovmm_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flag,
    struct thread *td)
{
	int ret;

	switch (cmd) {
	case VMM_IOC_CREATE:
		ovmm_start();
		ret = vm_create((struct vm_create_params *)data, td);
		break;
	case VMM_IOC_RUN:
		ret = vm_run((struct vm_run_params *)data);
		break;
	case VMM_IOC_INFO:
		ret = vm_get_info((struct vm_info_params *)data);
		break;
	case VMM_IOC_TERM:
		ret = vm_terminate((struct vm_terminate_params *)data);
		break;
	case VMM_IOC_RESETCPU:
		ret = vm_resetcpu((struct vm_resetcpu_params *)data);
		break;
	case VMM_IOC_READREGS:
		ret = vm_rwregs((struct vm_rwregs_params *)data, 0);
		break;
	case VMM_IOC_WRITEREGS:
		ret = vm_rwregs((struct vm_rwregs_params *)data, 1);
		break;
	case VMM_IOC_READVMPARAMS:
		ret = vm_rwvmparams((struct vm_rwvmparams_params *)data, 0);
		break;
	case VMM_IOC_WRITEVMPARAMS:
		ret = vm_rwvmparams((struct vm_rwvmparams_params *)data, 1);
		break;
	case VMM_IOC_SHAREMEM:
		ret = vm_share_mem((struct vm_sharemem_params *)data, td);
		break;
	default:
		ret = ovmm_ioctl_machdep(cmd, data, flag, td);
		break;
	}

	return (ret);
}

/*
 * vm_find_vcpu
 *
 * Lookup VMM VCPU by ID number
 *
 * Parameters:
 *  vm: vm structure
 *  id: index id of vcpu
 *
 * Returns pointer to vcpu structure if successful, NULL otherwise
 */
struct vcpu *
vm_find_vcpu(struct vm *vm, uint32_t id)
{
	struct vcpu *vcpu;

	if (vm == NULL)
		return (NULL);

	SLIST_FOREACH(vcpu, &vm->vm_vcpu_list, vc_vcpu_link) {
		if (vcpu->vc_id == id)
			return (vcpu);
	}

	return (NULL);
}

/*
 * ovmm_map_range
 *
 * Map guest memory object 'obj' into 'map' (a process address space).
 * The kernel picks the address, which is returned in *va.  The mapping is
 * not inherited across fork(2), matching the original's MAP_INHERIT_NONE.
 *
 * On success the map owns one reference to 'obj'.
 */
static int
ovmm_map_range(vm_map_t map, vm_object_t obj, size_t size, uint64_t *va)
{
	vm_offset_t addr;
	int rv;

	addr = vm_map_min(map);
	vm_object_reference(obj);
	rv = vm_map_find(map, obj, 0, &addr, size, 0, VMFS_OPTIMAL_SPACE,
	    VM_PROT_RW, VM_PROT_RW, 0);
	if (rv != KERN_SUCCESS) {
		vm_object_deallocate(obj);
		return (vm_mmap_to_errno(rv));
	}
	(void)vm_map_inherit(map, addr, addr + size, VM_INHERIT_NONE);

	*va = addr;
	return (0);
}

/*
 * vm_create
 *
 * Creates the in-memory VMM structures for the VM defined by 'vcp'. The
 * parent of this VM shall be the process of thread 'td'.
 * This function does not start the VCPU(s) - see vm_run.
 *
 * Return Values:
 *  0: the create operation was successful
 *  ENOMEM: out of memory
 *  EINVAL: invalid parameters (memory ranges, vcpu count, SEV requested)
 *  various other errors from vcpu_init/vm_impl_init
 */
int
vm_create(struct vm_create_params *vcp, struct thread *td)
{
	int i, ret = EINVAL;
	size_t memsize;
	struct vm *vm;
	struct vcpu *vcpu;
	vm_object_t obj;
	vm_map_t pmap_map;
	vm_offset_t gpa;
	struct vm_mem_range *vmr;

	memsize = vm_create_check_mem_ranges(vcp);
	if (memsize == 0)
		return (EINVAL);

	/* XXX - support UP only (for now) */
	if (vcp->vcp_ncpus != 1)
		return (EINVAL);

	/* SEV/SEV-ES are not supported. */
	if (vcp->vcp_sev != 0 || vcp->vcp_seves != 0)
		return (EINVAL);

	/*
	 * Increment global counts early to see if the capacity limits
	 * would be violated and prevent ovmm(4) from disabling
	 * virtualization extensions on the host while creating a vm.
	 */
	sx_xlock(&vmm_softc->vm_lock);
	if (vmm_softc->vcpu_ct + vcp->vcp_ncpus > vmm_softc->vcpu_max) {
		DPRINTF("%s: maximum vcpus (%lu) reached\n", __func__,
		    vmm_softc->vcpu_max);
		sx_xunlock(&vmm_softc->vm_lock);
		return (ENOMEM);
	}
	vmm_softc->vcpu_ct += vcp->vcp_ncpus;
	vmm_softc->vm_ct++;
	sx_xunlock(&vmm_softc->vm_lock);

	/* Instantiate and configure the new vm. */
	vm = malloc(sizeof(*vm), M_OVMM, M_WAITOK | M_ZERO);

	/* Create the VM's identity. */
	vm->vm_creator_pid = td->td_proc->p_pid;
	strncpy(vm->vm_name, vcp->vcp_name, VMM_MAX_NAME_LEN - 1);

	sx_init(&vm->vm_vcpu_lock, "ovmm vcpu list");
	SLIST_INIT(&vm->vm_vcpu_list);
	refcount_init(&vm->vm_refcnt, 1);

	/* Initialize memory slots. */
	vm->vm_nmemranges = vcp->vcp_nmemranges;
	memcpy(vm->vm_memranges, vcp->vcp_memranges,
	    vm->vm_nmemranges * sizeof(vm->vm_memranges[0]));
	vm->vm_memory_size = memsize; /* Calculated above. */

	/* Create the nested paging address space. */
	if (vm_impl_init(vm)) {
		printf("failed to init arch-specific features for vm %p\n", vm);
		ret = ENOMEM;
		goto err;
	}
	pmap_map = &vm->vm_vmspace->vm_map;

	for (i = 0; i < vm->vm_nmemranges; i++) {
		vmr = &vm->vm_memranges[i];
		if (vmr->vmr_type == VM_MEM_MMIO)
			continue;

		obj = vm_object_allocate(OBJT_SWAP, atop(vmr->vmr_size));
		if (obj == NULL) {
			printf("%s: failed to initialize memory slot\n",
			    __func__);
			ret = ENOMEM;
			goto err;
		}
		/* The vm owns this reference. */
		vm->vm_memory_slot[i] = obj;

		/* Map the object into the guest physical address space. */
		gpa = vmr->vmr_gpa;
		vm_object_reference(obj);
		ret = vm_map_find(pmap_map, obj, 0, &gpa, vmr->vmr_size, 0,
		    VMFS_NO_SPACE, VM_PROT_ALL, VM_PROT_ALL, 0);
		if (ret != KERN_SUCCESS) {
			printf("%s: vm_map_find (gpa) failed: %d\n", __func__,
			    ret);
			vm_object_deallocate(obj);
			ret = ENOMEM;
			goto err;
		}

		/* Map the object into the process. */
		ret = ovmm_map_range(&td->td_proc->p_vmspace->vm_map, obj,
		    vmr->vmr_size, &vmr->vmr_va);
		if (ret) {
			printf("%s: vm_map_find (hva) failed: %d\n", __func__,
			    ret);
			goto err;
		}
	}

	vm->vm_vcpu_ct = 0;

	/* Initialize each VCPU defined in 'vcp' */
	for (i = 0; i < vcp->vcp_ncpus; i++) {
		vcpu = malloc(sizeof(*vcpu), M_OVMM, M_WAITOK | M_ZERO);

		vcpu->vc_parent = vm;
		vcpu->vc_id = vm->vm_vcpu_ct;
		vm->vm_vcpu_ct++;

		if ((ret = vcpu_init(vcpu, vcp)) != 0) {
			printf("failed to init vcpu %d for vm %p\n", i, vm);
			free(vcpu, M_OVMM);
			goto err;
		}
		/* Publish vcpu to list, inheriting the reference. */
		SLIST_INSERT_HEAD(&vm->vm_vcpu_list, vcpu, vc_vcpu_link);
	}

	/* Increment the global index and insert into the list. */
	sx_xlock(&vmm_softc->vm_lock);
	vmm_softc->vm_idx++;
	vm->vm_id = vmm_softc->vm_idx;
	vcp->vcp_id = vm->vm_id;

	SLIST_INSERT_HEAD(&vmm_softc->vm_list, vm, vm_link);
	sx_xunlock(&vmm_softc->vm_lock);

	/* Update the userland process's view of guest memory. */
	memcpy(vcp->vcp_memranges, vm->vm_memranges,
	    vcp->vcp_nmemranges * sizeof(vcp->vcp_memranges[0]));

	return (0);

err:
	vm_teardown(&vm);
	sx_xlock(&vmm_softc->vm_lock);
	vmm_softc->vm_ct--;
	vmm_softc->vcpu_ct -= vcp->vcp_ncpus;
	if (vmm_softc->vm_ct < 1)
		ovmm_stop();
	sx_xunlock(&vmm_softc->vm_lock);
	return (ret);
}

/*
 * vm_create_check_mem_ranges
 *
 * Make sure that the guest physical memory ranges given by the user process
 * do not overlap and are in ascending order.
 *
 * The last physical address may not exceed VMM_MAX_VM_MEM_SIZE.
 *
 * Return Values:
 *   The total memory size in bytes if the checks were successful
 *   0: One of the memory ranges was invalid or VMM_MAX_VM_MEM_SIZE was
 *   exceeded
 */
size_t
vm_create_check_mem_ranges(struct vm_create_params *vcp)
{
	size_t i, memsize = 0;
	struct vm_mem_range *vmr, *pvmr = NULL;
	const uint64_t maxgpa = VMM_MAX_VM_MEM_SIZE;

	if (vcp->vcp_nmemranges == 0 ||
	    vcp->vcp_nmemranges > VMM_MAX_MEM_RANGES) {
		DPRINTF("invalid number of guest memory ranges\n");
		return (0);
	}

	for (i = 0; i < vcp->vcp_nmemranges; i++) {
		vmr = &vcp->vcp_memranges[i];

		/* Only page-aligned addresses and sizes are permitted */
		if ((vmr->vmr_gpa & PAGE_MASK) || (vmr->vmr_va & PAGE_MASK) ||
		    (vmr->vmr_size & PAGE_MASK) || vmr->vmr_size == 0) {
			DPRINTF("memory range %zu is not page aligned\n", i);
			return (0);
		}

		/* Make sure that VMM_MAX_VM_MEM_SIZE is not exceeded */
		if (vmr->vmr_gpa >= maxgpa ||
		    vmr->vmr_size > maxgpa - vmr->vmr_gpa) {
			DPRINTF("exceeded max memory size\n");
			return (0);
		}

		/*
		 * Make sure that guest physical memory ranges do not overlap
		 * and that they are ascending.
		 */
		if (i > 0 && pvmr->vmr_gpa + pvmr->vmr_size > vmr->vmr_gpa) {
			DPRINTF("guest range %zu overlaps or !ascending\n", i);
			return (0);
		}

		/*
		 * No memory is mappable in MMIO ranges, so don't count towards
		 * the total guest memory size.
		 */
		if (vmr->vmr_type != VM_MEM_MMIO)
			memsize += vmr->vmr_size;
		pvmr = vmr;
	}

	return (memsize);
}

/*
 * vm_teardown
 *
 * Tears down (destroys) the vm indicated by 'vm'.
 *
 * Assumes the vm is already removed from the global vm list (or was never
 * added).
 *
 * Parameters:
 *  vm: vm to be torn down
 */
void
vm_teardown(struct vm **target)
{
	size_t i;
	struct vcpu *vcpu, *tmp;
	struct vm *vm = *target;
	vm_object_t obj;

	/* Free VCPUs */
	SLIST_FOREACH_SAFE(vcpu, &vm->vm_vcpu_list, vc_vcpu_link, tmp) {
		SLIST_REMOVE(&vm->vm_vcpu_list, vcpu, vcpu, vc_vcpu_link);
		vcpu_deinit(vcpu);
		free(vcpu, M_OVMM);
	}

	/*
	 * Destroy the nested address space.  This removes the guest
	 * mappings from the nested page tables and drops the references
	 * the nested map held on the memory objects.
	 */
	vm_impl_deinit(vm);

	/* Release the objects backing our guest memory. */
	for (i = 0; i < vm->vm_nmemranges; i++) {
		obj = vm->vm_memory_slot[i];
		vm->vm_memory_slot[i] = NULL;
		if (obj != NULL)
			vm_object_deallocate(obj);
	}

	sx_destroy(&vm->vm_vcpu_lock);
	free(vm, M_OVMM);
	*target = NULL;
}

/*
 * vm_get_info
 *
 * Returns information about the VM indicated by 'vip'. The 'vip_size' field
 * in the 'vip' parameter is used to indicate the size of the caller's buffer.
 * If insufficient space exists in that buffer, the required size needed is
 * returned in vip_size and the number of VM information structures returned
 * in vip_info_count is set to 0. The caller should then try the ioctl again
 * after allocating a sufficiently large buffer.
 *
 * Parameters:
 *  vip: information structure identifying the VM to query
 *
 * Return values:
 *  0: the operation succeeded
 *  ENOMEM: memory allocation error during processing
 *  EFAULT: error copying data to user process
 */
int
vm_get_info(struct vm_info_params *vip)
{
	struct vm_info_result *out;
	struct vm *vm;
	struct vcpu *vcpu;
	int i = 0, j, error;
	size_t need, vm_ct;

	sx_slock(&vmm_softc->vm_lock);
	vm_ct = vmm_softc->vm_ct;
	sx_sunlock(&vmm_softc->vm_lock);

	need = vm_ct * sizeof(struct vm_info_result);
	if (vip->vip_size < need) {
		vip->vip_info_ct = 0;
		vip->vip_size = need;
		return (0);
	}

	out = malloc(need, M_OVMM, M_WAITOK | M_ZERO);
	if (out == NULL) {
		vip->vip_info_ct = 0;
		return (ENOMEM);
	}

	vip->vip_info_ct = vm_ct;

	sx_slock(&vmm_softc->vm_lock);
	SLIST_FOREACH(vm, &vmm_softc->vm_list, vm_link) {
		if (i == vm_ct)
			break;	/* Truncate to keep within bounds of 'out'. */

		out[i].vir_memory_size = vm->vm_memory_size;
		out[i].vir_used_size =
		    vmspace_resident_count(vm->vm_vmspace) * PAGE_SIZE;
		out[i].vir_ncpus = vm->vm_vcpu_ct;
		out[i].vir_id = vm->vm_id;
		out[i].vir_creator_pid = vm->vm_creator_pid;
		strlcpy(out[i].vir_name, vm->vm_name, VMM_MAX_NAME_LEN);

		for (j = 0; j < vm->vm_vcpu_ct; j++) {
			out[i].vir_vcpu_state[j] = VCPU_STATE_UNKNOWN;
			SLIST_FOREACH(vcpu, &vm->vm_vcpu_list,
			    vc_vcpu_link) {
				if (vcpu->vc_id == j)
					out[i].vir_vcpu_state[j] =
					    vcpu->vc_state;
			}
		}

		i++;
	}
	sx_sunlock(&vmm_softc->vm_lock);

	error = copyout(out, vip->vip_info, need);
	free(out, M_OVMM);
	return (error);
}

/*
 * vm_terminate
 *
 * Terminates the VM indicated by 'vtp'.
 *
 * Parameters:
 *  vtp: structure defining the VM to terminate
 *
 * Return values:
 *  0: the VM was terminated
 *  !0: the VM could not be located
 */
int
vm_terminate(struct vm_terminate_params *vtp)
{
	struct vm *vm;
	struct vcpu *vcpu;
	int error, nvcpu, vm_id;

	/*
	 * Find desired VM
	 */
	error = vm_find(vtp->vtp_vm_id, &vm);
	if (error)
		return (error);

	/* Only proceed through remove and teardown once. */
	if (atomic_cmpset_int(&vm->vm_dying, 0, 1) == 0) {
		vm_rele(vm);
		return (EBUSY);
	}

	/* Ask running vcpus to come back to us. */
	SLIST_FOREACH(vcpu, &vm->vm_vcpu_list, vc_vcpu_link)
		(void)atomic_cmpset_int(&vcpu->vc_state, VCPU_STATE_RUNNING,
		    VCPU_STATE_REQTERM);

	/* Pop the vm out of the global vm list. */
	sx_xlock(&vmm_softc->vm_lock);
	SLIST_REMOVE(&vmm_softc->vm_list, vm, vm, vm_link);
	sx_xunlock(&vmm_softc->vm_lock);

	/* Drop the vm_list's reference to the vm. */
	if (refcount_release(&vm->vm_refcnt))
		panic("%s: vm %d(%p) vm_list refcnt drop was the last",
		    __func__, vm->vm_id, vm);

	/* Wait until our reference (taken from vm_find) is the last one. */
	while (atomic_load_int(&vm->vm_refcnt) > 1)
		tsleep(__DEVOLATILE(void *, &vm->vm_refcnt), PWAIT, "ovmmterm",
		    hz / 10);

	vm_id = vm->vm_id;
	nvcpu = vm->vm_vcpu_ct;

	vm_teardown(&vm);

	if (vm_id > 0) {
		sx_xlock(&vmm_softc->vm_lock);
		vmm_softc->vm_ct--;
		vmm_softc->vcpu_ct -= nvcpu;
		if (vmm_softc->vm_ct < 1)
			ovmm_stop();
		sx_xunlock(&vmm_softc->vm_lock);
	}

	return (0);
}

/*
 * vm_resetcpu
 *
 * Resets the vcpu defined in 'vrp' to power-on-init register state
 *
 * Parameters:
 *  vrp: ioctl structure defining the vcpu to reset (see ovmm_svm.h)
 *
 * Returns 0 if successful, or various error codes on failure:
 *  ENOENT if the VM id contained in 'vrp' refers to an unknown VM or
 *      if vrp describes an unknown vcpu for this VM
 *  EBUSY if the indicated VCPU is not stopped
 *  EIO if the indicated VCPU failed to reset
 */
int
vm_resetcpu(struct vm_resetcpu_params *vrp)
{
	struct vm *vm;
	struct vcpu *vcpu;
	int error, ret = 0;

	/* Find the desired VM */
	error = vm_find(vrp->vrp_vm_id, &vm);

	/* Not found? exit. */
	if (error != 0) {
		DPRINTF("%s: vm id %u not found\n", __func__,
		    vrp->vrp_vm_id);
		return (error);
	}

	vcpu = vm_find_vcpu(vm, vrp->vrp_vcpu_id);

	if (vcpu == NULL) {
		DPRINTF("%s: vcpu id %u of vm %u not found\n", __func__,
		    vrp->vrp_vcpu_id, vrp->vrp_vm_id);
		ret = ENOENT;
		goto out;
	}

	sx_xlock(&vcpu->vc_lock);
	if (vcpu->vc_state != VCPU_STATE_STOPPED)
		ret = EBUSY;
	else {
		if (vcpu_reset_regs(vcpu, &vrp->vrp_init_state)) {
			printf("%s: failed\n", __func__);
			ret = EIO;
		}
	}
	sx_xunlock(&vcpu->vc_lock);
out:
	vm_rele(vm);

	return (ret);
}

/*
 * vcpu_must_yield
 *
 * Check if we need to (temporarily) stop running the VCPU for some reason,
 * such as:
 * - the VM was requested to terminate
 * - the thread running this VCPU has pending signals
 * - the scheduler asks us to yield (AST pending)
 *
 * Parameters:
 *  vcpu: the VCPU to check
 *
 * Return values:
 *  1: the VM owning this VCPU should stop
 *  0: no stop is needed
 */
int
vcpu_must_yield(struct vcpu *vcpu)
{
	struct thread *td = curthread;

	if (vcpu->vc_state == VCPU_STATE_REQTERM)
		return (1);

	if (SIGPENDING(td))
		return (1);

	if (td->td_ast != 0 || td->td_owepreempt != 0)
		return (1);

	return (0);
}

/*
 * vm_share_mem
 *
 * Share the vm guest memory ranges into the calling process.
 *
 * Return values:
 *  0: if successful
 *  ENOENT: if the vm cannot be found by vm_find
 *  EINVAL: if the ranges do not match the vm's ranges
 *  other errno on vm_map_find failures
 */
int
vm_share_mem(struct vm_sharemem_params *vsp, struct thread *td)
{
	int ret = EINVAL;
	size_t i, n, mapped = 0;
	struct vm *vm;
	struct vm_mem_range *src, *dst;
	vm_object_t obj;
	vm_map_t map = &td->td_proc->p_vmspace->vm_map;

	ret = vm_find(vsp->vsp_vm_id, &vm);
	if (ret)
		return (ret);

	/* Check we have the expected number of ranges. */
	if (vm->vm_nmemranges != vsp->vsp_nmemranges) {
		ret = EINVAL;
		goto out;
	}
	n = vm->vm_nmemranges;

	/* Check their types, sizes, and gpa's (implying page alignment). */
	for (i = 0; i < n; i++) {
		src = &vm->vm_memranges[i];
		dst = &vsp->vsp_memranges[i];

		/*
		 * The vm memranges were already checked during creation, so
		 * compare to them to confirm validity of mapping request.
		 */
		if (src->vmr_type != dst->vmr_type ||
		    src->vmr_gpa != dst->vmr_gpa ||
		    src->vmr_size != dst->vmr_size) {
			ret = EINVAL;
			goto out;
		}

		/* The virtual addresses will be chosen by the kernel. */
		if (vsp->vsp_va[i] != 0) {
			ret = EINVAL;
			goto out;
		}
	}

	/* Share each object with the calling process. */
	for (i = 0; i < n; i++) {
		dst = &vsp->vsp_memranges[i];
		if (dst->vmr_type == VM_MEM_MMIO)
			continue;

		obj = vm->vm_memory_slot[i];
		KASSERT(obj != NULL, ("%s: no object for range %zu", __func__,
		    i));

		ret = ovmm_map_range(map, obj, dst->vmr_size, &vsp->vsp_va[i]);
		if (ret) {
			printf("%s: vm_map_find failed: %d\n", __func__, ret);
			goto out;
		}
		mapped = i + 1;
	}
	ret = 0;
out:
	if (ret != 0 && mapped > 0) {
		/* Unmap mapped objects, which drops the process's references. */
		for (i = 0; i < mapped; i++) {
			dst = &vsp->vsp_memranges[i];
			if (dst->vmr_type == VM_MEM_MMIO || vsp->vsp_va[i] == 0)
				continue;
			(void)vm_map_remove(map, vsp->vsp_va[i],
			    vsp->vsp_va[i] + dst->vmr_size);
			vsp->vsp_va[i] = 0;
		}
	}
	vm_rele(vm);
	return (ret);
}

/*
 * ovmm_attach
 *
 * Module initialisation: probe the hardware, set up the softc and create
 * /dev/ovmm.
 */
static int
ovmm_attach(void)
{
	struct vmm_softc *sc;
	struct make_dev_args mda;
	int error;

	sc = malloc(sizeof(*sc), M_OVMM, M_WAITOK | M_ZERO);

	error = ovmm_probe_machdep(&sc->sc_md);
	if (error != 0) {
		free(sc, M_OVMM);
		return (error);
	}

	sx_init(&sc->sc_slock, "ovmm slock");
	sx_init(&sc->vm_lock, "ovmm vm list");
	mtx_init(&sc->vpid_lock, "ovmm asid", NULL, MTX_DEF);

	sc->mode = VMM_MODE_RVI;
	sc->vcpu_ct = 0;
	sc->vcpu_max = VMM_MAX_VCPUS;
	sc->vm_ct = 0;
	sc->vm_idx = 0;
	SLIST_INIT(&sc->vm_list);

	/* ASID 0 is reserved for the host; the bitmap holds 4096 entries. */
	sc->max_vpid = min(sc->sc_md.svm_nasid - 1, 4095);
	bzero(&sc->vpids, sizeof(sc->vpids));

	vmm_softc = sc;

	make_dev_args_init(&mda);
	mda.mda_devsw = &ovmm_cdevsw;
	mda.mda_uid = UID_ROOT;
	mda.mda_gid = GID_WHEEL;
	mda.mda_mode = 0660;
	error = make_dev_s(&mda, &sc->sc_cdev, "ovmm");
	if (error != 0) {
		vmm_softc = NULL;
		ovmm_deinit_machdep(&sc->sc_md);
		mtx_destroy(&sc->vpid_lock);
		sx_destroy(&sc->vm_lock);
		sx_destroy(&sc->sc_slock);
		free(sc, M_OVMM);
		return (error);
	}

	return (0);
}

static void
ovmm_detach(void)
{
	struct vmm_softc *sc = vmm_softc;

	if (sc == NULL)
		return;

	destroy_dev(sc->sc_cdev);
	ovmm_stop();
	vmm_softc = NULL;
	ovmm_deinit_machdep(&sc->sc_md);
	mtx_destroy(&sc->vpid_lock);
	sx_destroy(&sc->vm_lock);
	sx_destroy(&sc->sc_slock);
	free(sc, M_OVMM);
}

static int
ovmm_modevent(module_t mod, int type, void *data)
{
	int error;

	switch (type) {
	case MOD_LOAD:
		error = ovmm_attach();
		break;
	case MOD_UNLOAD:
		error = 0;
		sx_slock(&vmm_softc->vm_lock);
		if (vmm_softc->vm_ct > 0)
			error = EBUSY;
		sx_sunlock(&vmm_softc->vm_lock);
		if (error == 0)
			ovmm_detach();
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

static moduledata_t ovmm_mod = {
	"ovmm",
	ovmm_modevent,
	NULL
};

DECLARE_MODULE(ovmm, ovmm_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(ovmm, 1);
