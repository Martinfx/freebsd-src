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
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/devmap.h>
#include <sys/lock.h>
#include <sys/reboot.h>
#include <sys/systm.h>

#include <vm/vm.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/fdt.h>
#include <machine/intr.h>
#include <machine/machdep.h>
#include <machine/platformvar.h>

#include <dev/ofw/openfirm.h>
#include <arm/mediatek/mt7623_mp.h>

#include "platform_if.h"

static platform_attach_t mt7623_attach;
static platform_devmap_init_t mt7623_devmap_init;
static platform_late_init_t mt7623_late_init;
static platform_cpu_reset_t mt7623_cpu_reset;

#define GPT_BASE        0x10008000
#define GPT4_CNT        0x48
#define GPT4_FREQ       13000000
#define GPT6_CON        0x60
#define  GPT_CON_ENABLE        (1 << 0)
#define  GPT_CON_MODE_FREERUN  (3 << 4)

static bus_space_handle_t gpt_h;

static int
mt7623_attach(platform_t plat)
{
	return (0);
}


static void
mt7623_delay(int usec, void *arg __unused)
{
	uint32_t first, counts;

	counts = (uint32_t)((uint64_t)usec * GPT4_FREQ / 1000000);
	first = bus_space_read_4(fdtbus_bs_tag, gpt_h, GPT4_CNT);

	while ((bus_space_read_4(fdtbus_bs_tag, gpt_h, GPT4_CNT) - first) < counts)
		;
}

static void
mt7623_late_init(platform_t plat)
{
	uint64_t a, b;

	if (bus_space_map(fdtbus_bs_tag, GPT_BASE, 0x100, 0, &gpt_h) != 0) {
		printf("MT: cannot map APXGPT block\n");
		return;
	}

	/*
	 * The input clock of the ARM generic timer is gated by GPT6 on
	 * MT6589/MT7623/MT8127/MT8135.  Enable GPT6 in free-running mode
	 * to ungate it, otherwise CNTPCT never ticks and the system hangs
	 * once the generic timer is used as the event timer.
	 */
	bus_space_write_4(fdtbus_bs_tag, gpt_h, GPT6_CON,
	    GPT_CON_ENABLE | GPT_CON_MODE_FREERUN);
	dsb();

	/* Firmware does not program CNTFRQ on this board. */
	cp15_cntfrq_set(GPT4_FREQ);
	isb();

	a = cp15_cntpct_get();
	for (volatile int n = 0; n < 100000; n++)
		;
	b = cp15_cntpct_get();
	printf("MT: CNTPCT after GPT6 ungate: %llu -> %llu\n",
	    (unsigned long long)a, (unsigned long long)b);

	if (b > a) {
		printf("MT: system counter running!\n");
		return;
	}

	printf("MT: CNTPCT still dead, using GPT4 for DELAY\n");
	arm_set_delay(mt7623_delay, NULL);
}

static int
mt7623_devmap_init(platform_t plat)
{
	devmap_add_entry(0x10000000, 0x10000000);
	return (0);
}

static void
mt7623_cpu_reset(platform_t plat)
{
	panic("CPU reset not supported");
}

static platform_method_t mt7623_methods[] = {
	PLATFORMMETHOD(platform_attach,        mt7623_attach),
	PLATFORMMETHOD(platform_devmap_init,   mt7623_devmap_init),
	PLATFORMMETHOD(platform_late_init,     mt7623_late_init),
	PLATFORMMETHOD(platform_cpu_reset,     mt7623_cpu_reset),
#ifdef SMP
	PLATFORMMETHOD(platform_mp_start_ap,   mt7623_mp_start_ap),
	PLATFORMMETHOD(platform_mp_setmaxid,   mt7623_mp_setmaxid),
#endif
	PLATFORMMETHOD_END,
};

FDT_PLATFORM_DEF2(mt7623, mt7623, "MediaTek MT7623", 0, "mediatek,mt7623", 250);