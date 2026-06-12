/*
* SPDX-License-Identifier: BSD-2-Clause
*
* Copyright (c) 2026 Martin Filla
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*/

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_cpu.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/psci/psci.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpu.h>
#include <machine/intr.h>
#include <machine/fdt.h>
#include <machine/smp.h>
#include <machine/platformvar.h>
#include <machine/pmap.h>

#include <arm/mediatek/mt7623_mp.h>

#define MT_MAX_CPU         8
#define MT_SMP_REG_SIZE    0x1000
#define GICD_BASE          0x10211000
#define GICD_SGIR	   0xF00
#define GICD_CTLR	   0x000

struct mtk_smp_boot_info {
	bus_addr_t  smp_base;
	bus_size_t  jump_reg;
	uint32_t    core_keys[MT_MAX_CPU - 1];
	uint32_t    core_regs[MT_MAX_CPU - 1];
};

static const struct mtk_smp_boot_info mt7623_smp_boot = {
	.smp_base  = 0x10202000,
	.jump_reg  = 0x34,
	.core_keys = { 0x534c4131, 0x4c415332, 0x41534c33 },
	.core_regs = { 0x38,       0x3c,       0x40       },
};

void
mt7623_mp_setmaxid(platform_t plat)
{
	int ncpu;

	if (mp_ncpus != 0)
		return;

	ncpu = cp15_l2ctlr_get();
	ncpu = CPUV7_L2CTLR_NPROC(ncpu);

	mp_ncpus = ncpu;
	mp_maxid = ncpu - 1;
}

void
mt7623_mp_start_ap(platform_t plat)
{
	const struct mtk_smp_boot_info *info = &mt7623_smp_boot;
	bus_space_handle_t smp, gicd;
	int i;

	if (bus_space_map(fdtbus_bs_tag, info->smp_base, MT_SMP_REG_SIZE,
		0, &smp) != 0)
		panic("Couldn't map the MT SMP block");
	if (bus_space_map(fdtbus_bs_tag, GICD_BASE, 0x1000, 0, &gicd) != 0)
		panic("Couldn't map GIC distributor");

	/* jump register */
	bus_space_write_4(fdtbus_bs_tag, smp, info->jump_reg,
	    pmap_kextract((vm_offset_t)mpentry));
	/* enable GIC */
	bus_space_write_4(fdtbus_bs_tag, gicd, GICD_CTLR, 3);

	for (i = 1; i < mp_ncpus && i < MT_MAX_CPU; i++) {
		if (info->core_keys[i - 1] == 0)
			continue;
		bus_space_write_4(fdtbus_bs_tag, smp,
		    info->core_regs[i - 1], info->core_keys[i - 1]);

		bus_space_write_4(fdtbus_bs_tag, gicd, GICD_SGIR,
		    (1u << (16 + i)));
	}
	dsb();
	sev();

	bus_space_unmap(fdtbus_bs_tag, gicd, 0x1000);
	bus_space_unmap(fdtbus_bs_tag, smp, MT_SMP_REG_SIZE);
}