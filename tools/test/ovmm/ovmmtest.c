/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * ovmmtest: smoke test for the ovmm(4) hypervisor.
 *
 * Creates a VM with 1 MB of RAM, loads a tiny 16-bit real mode program
 * that prints a string on the COM1 data port (0x3f8) and halts, runs it
 * with VMM_IOC_RUN and echoes the guest's output to stdout.
 *
 * Usage: ovmmtest [-v]
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/param.h>

#include <dev/ovmm/ovmm.h>

#include <err.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define	GUEST_MEM_SIZE	(1024 * 1024)
#define	MAX_EXITS	100000

/*
 * 16-bit real mode guest, assembled by hand.  Loaded at guest physical
 * address 0, executed with CS=0, IP=0, DS=0.
 *
 *  0000: BA F8 03        mov    dx, 0x3f8
 *  0003: BE 11 00        mov    si, msg
 *  0006: AC          1:  lodsb
 *  0007: 84 C0           test   al, al
 *  0009: 74 03           jz     2f
 *  000B: EE              out    dx, al
 *  000C: EB F8           jmp    1b
 *  000E: F4          2:  hlt
 *  000F: EB FD           jmp    2b
 *  0011: msg:            .asciz "..."
 */
static const uint8_t guest_code[] = {
	0xBA, 0xF8, 0x03,
	0xBE, 0x11, 0x00,
	0xAC,
	0x84, 0xC0,
	0x74, 0x03,
	0xEE,
	0xEB, 0xF8,
	0xF4,
	0xEB, 0xFD,
};
static const char guest_msg[] = "Hello from an ovmm(4) guest!\r\n";

/* Power-on register state for a flat 16-bit real mode guest at 0:0. */
static const struct vcpu_reg_state init_state = {
	.vrs_gprs[VCPU_REGS_RFLAGS] = 0x202,	/* IF set so HLT is clean */
	.vrs_gprs[VCPU_REGS_RIP] = 0x0,
	.vrs_gprs[VCPU_REGS_RSP] = 0x0,
	.vrs_crs[VCPU_REGS_CR0] = 0x60000010,
	.vrs_crs[VCPU_REGS_CR3] = 0,
	.vrs_crs[VCPU_REGS_XCR0] = 0x1,
	.vrs_sregs[VCPU_REGS_CS] = { 0x0, 0xFFFF, 0x009B, 0x0 },
	.vrs_sregs[VCPU_REGS_DS] = { 0x0, 0xFFFF, 0x0093, 0x0 },
	.vrs_sregs[VCPU_REGS_ES] = { 0x0, 0xFFFF, 0x0093, 0x0 },
	.vrs_sregs[VCPU_REGS_FS] = { 0x0, 0xFFFF, 0x0093, 0x0 },
	.vrs_sregs[VCPU_REGS_GS] = { 0x0, 0xFFFF, 0x0093, 0x0 },
	.vrs_sregs[VCPU_REGS_SS] = { 0x0, 0xFFFF, 0x0093, 0x0 },
	.vrs_gdtr = { 0x0, 0xFFFF, 0x0, 0x0 },
	.vrs_idtr = { 0x0, 0xFFFF, 0x0, 0x0 },
	.vrs_sregs[VCPU_REGS_LDTR] = { 0x0, 0xFFFF, 0x0082, 0x0 },
	.vrs_sregs[VCPU_REGS_TR] = { 0x0, 0xFFFF, 0x008B, 0x0 },
	.vrs_msrs[VCPU_REGS_EFER] = 0ULL,
	.vrs_drs[VCPU_REGS_DR6] = 0xFFFF0FF0,
	.vrs_drs[VCPU_REGS_DR7] = 0x400,
};

static int verbose;

static void
usage(void)
{
	fprintf(stderr, "usage: ovmmtest [-v]\n");
	exit(1);
}

int
main(int argc, char *argv[])
{
	struct vm_create_params vcp;
	struct vm_resetcpu_params vresetp;
	struct vm_run_params vrp;
	struct vm_terminate_params vtp;
	struct vm_exit *exit;
	uint8_t *mem;
	int ch, fd, n, ret;

	while ((ch = getopt(argc, argv, "v")) != -1) {
		switch (ch) {
		case 'v':
			verbose = 1;
			break;
		default:
			usage();
		}
	}

	fd = open("/dev/ovmm", O_RDWR);
	if (fd == -1)
		err(1, "open /dev/ovmm");

	/* Create the VM: one RAM range at gpa 0. */
	memset(&vcp, 0, sizeof(vcp));
	vcp.vcp_ncpus = 1;
	vcp.vcp_nmemranges = 1;
	vcp.vcp_memranges[0].vmr_gpa = 0;
	vcp.vcp_memranges[0].vmr_size = GUEST_MEM_SIZE;
	vcp.vcp_memranges[0].vmr_type = VM_MEM_RAM;
	strlcpy(vcp.vcp_name, "ovmmtest", sizeof(vcp.vcp_name));

	if (ioctl(fd, VMM_IOC_CREATE, &vcp) == -1)
		err(1, "VMM_IOC_CREATE");

	printf("created vm %u, asid %u, guest memory mapped at %#lx\n",
	    vcp.vcp_id, vcp.vcp_asid[0],
	    (unsigned long)vcp.vcp_memranges[0].vmr_va);

	/* Load the guest program. */
	mem = (uint8_t *)(uintptr_t)vcp.vcp_memranges[0].vmr_va;
	memcpy(mem, guest_code, sizeof(guest_code));
	memcpy(mem + 0x11, guest_msg, sizeof(guest_msg));

	/* Reset the vcpu to the initial register state. */
	memset(&vresetp, 0, sizeof(vresetp));
	vresetp.vrp_vm_id = vcp.vcp_id;
	vresetp.vrp_vcpu_id = 0;
	vresetp.vrp_init_state = init_state;
	if (ioctl(fd, VMM_IOC_RESETCPU, &vresetp) == -1)
		err(1, "VMM_IOC_RESETCPU");

	exit = calloc(1, sizeof(*exit));
	if (exit == NULL)
		err(1, "calloc");

	memset(&vrp, 0, sizeof(vrp));
	vrp.vrp_vm_id = vcp.vcp_id;
	vrp.vrp_vcpu_id = 0;
	vrp.vrp_exit = exit;
	vrp.vrp_inject.vie_type = VCPU_INJECT_NONE;

	ret = 1;
	for (n = 0; n < MAX_EXITS; n++) {
		if (ioctl(fd, VMM_IOC_RUN, &vrp) == -1) {
			warn("VMM_IOC_RUN");
			break;
		}

		if (verbose)
			fprintf(stderr, "exit %d: reason %#x rip %#lx\n", n,
			    vrp.vrp_exit_reason,
			    (unsigned long)exit->vrs.vrs_gprs[VCPU_REGS_RIP]);

		switch (vrp.vrp_exit_reason) {
		case VM_EXIT_NONE:
			/* The kernel yielded; just run again. */
			break;
		case SVM_VMEXIT_IOIO:
			if (exit->vei.vei_dir == VEI_DIR_OUT &&
			    exit->vei.vei_port == 0x3f8) {
				putchar(exit->vei.vei_data & 0xff);
				fflush(stdout);
			} else if (exit->vei.vei_dir == VEI_DIR_IN) {
				/* Unhandled port: read as all ones. */
				exit->vei.vei_data = 0xffffffff;
			}
			/*
			 * Like vmd(8): the monitor advances %rip past the
			 * I/O instruction, the kernel picks it up from
			 * vrs on the next VMM_IOC_RUN.
			 */
			exit->vrs.vrs_gprs[VCPU_REGS_RIP] +=
			    exit->vei.vei_insn_len;
			break;
		case SVM_VMEXIT_HLT:
			printf("guest halted at rip %#lx after %d exits: OK\n",
			    (unsigned long)exit->vrs.vrs_gprs[VCPU_REGS_RIP],
			    n + 1);
			ret = 0;
			goto done;
		case SVM_VMEXIT_SHUTDOWN:
			printf("guest triple faulted\n");
			goto done;
		case VM_EXIT_TERMINATED:
			printf("vcpu terminated\n");
			goto done;
		default:
			printf("unexpected exit reason %#x\n",
			    vrp.vrp_exit_reason);
			goto done;
		}
	}
	if (n == MAX_EXITS)
		printf("giving up after %d exits\n", n);

done:
	memset(&vtp, 0, sizeof(vtp));
	vtp.vtp_vm_id = vcp.vcp_id;
	if (ioctl(fd, VMM_IOC_TERM, &vtp) == -1)
		warn("VMM_IOC_TERM");

	close(fd);
	return (ret);
}
