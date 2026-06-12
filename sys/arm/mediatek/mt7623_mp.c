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

/*
 * GIC distributor, first "reg" entry of interrupt-controller@10211000.  It
 * is programmed here because platform_mp_start_ap() runs at SI_SUB_CPU,
 * before gic(4) attaches, and only an interrupt gets a parked core out of
 * WFI.
 */
#define	GICD_BASE		0x10211000
#define	GICD_SIZE		0x1000
#define	GICD_CTLR		0x000
#define	 GICD_CTLR_ENABLE_GRP0	(1 << 0)
#define	 GICD_CTLR_ENABLE_GRP1	(1 << 1)
#define	GICD_SGIR		0xF00
#define	 GICD_SGIR_TO_OTHERS	(1 << 24)	/* SGI 0 to all other CPUs */

static const uint32_t mt7623_keys[MT_NKEYS] = {
    MT_KEY_CPU1, MT_KEY_CPU2, MT_KEY_CPU3
};

void
mt7623_mp_setmaxid(platform_t plat)
{
        uint32_t reg;
        int ncpu;

        if (mp_ncpus != 0)
                return;

        reg = cp15_l2ctlr_get();
        ncpu = CPUV7_L2CTLR_NPROC(reg);

        mp_ncpus = ncpu;
        mp_maxid = ncpu - 1;
}

void
mt7623_mp_start_ap(platform_t plat)
{
        bus_space_handle_t cpucfg, gicd;
        int cpu;

        if (bus_space_map(fdtbus_bs_tag, MT_CPUCFG_BASE, MT_CPUCFG_SIZE,
            0, &cpucfg) != 0)
                panic("Couldn't map the MT CPU configuration block");
        if (bus_space_map(fdtbus_bs_tag, GICD_BASE, GICD_SIZE, 0, &gicd) != 0)
                panic("Couldn't map GIC distributor");

        /* Where a released core continues.  It starts with the MMU off. */
        bus_space_write_4(fdtbus_bs_tag, cpucfg, MT_JUMP_REG,
            pmap_kextract((vm_offset_t)mpentry));

        bus_space_write_4(fdtbus_bs_tag, gicd, GICD_CTLR,
            GICD_CTLR_ENABLE_GRP0 | GICD_CTLR_ENABLE_GRP1);

        for (cpu = 1; cpu < mp_ncpus && cpu <= MT_NKEYS; cpu++) {
                bus_space_write_4(fdtbus_bs_tag, cpucfg, MT_KEY_REG(cpu),
                    mt7623_keys[cpu - 1]);

                dsb();

                bus_space_write_4(fdtbus_bs_tag, gicd, GICD_SGIR,
                    GICD_SGIR_TO_OTHERS);
                dsb();
                sev();

                if (bootverbose)
                        printf("MT7623 SMP: CPU%d up\n", cpu);
        }

        bus_space_unmap(fdtbus_bs_tag, gicd, GICD_SIZE);
        bus_space_unmap(fdtbus_bs_tag, cpucfg, MT_CPUCFG_SIZE);
}
