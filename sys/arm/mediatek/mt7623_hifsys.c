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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/syscon/syscon.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_fixed.h>
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/mt2701-clk.h>
#include "syscon_if.h"
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"

/* HIF_SYS_RST; this block has a single bank. */
#define	HIFSYS_RST_BASE	0x0034
#define	HIFSYS_RST_BANKS	1

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7623-hifsys", 1},
	{"mediatek,mt2701-hifsys", 1},
	{NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
PDN_GATE(CLK_HIFSYS_USB0PHY, "usb0_phy_clk", "ethpll_500m_ck", 0x0030, 21),
PDN_GATE(CLK_HIFSYS_USB1PHY, "usb1_phy_clk", "ethpll_500m_ck", 0x0030, 22),
	PDN_GATE(CLK_HIFSYS_PCIE0, "pcie0_clk", "ethpll_500m_ck", 0x0030, 24),
	PDN_GATE(CLK_HIFSYS_PCIE1, "pcie1_clk", "ethpll_500m_ck", 0x0030, 25),
	PDN_GATE(CLK_HIFSYS_PCIE2, "pcie2_clk", "ethpll_500m_ck", 0x0030, 26),
};

static struct mdtk_clk_def clk_def = {
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
};

static int
hifsys_clk_detach(device_t dev)
{

	return (EBUSY);
}

static int
hifsys_clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Mediatek hifsys clocks");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
hifsys_clk_attach(device_t dev)
{
	struct mdtk_clk_softc *sc;
	int rid, rv;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/*
	 * A node that also claims to be a syscon serves its registers to
	 * other drivers; the clocks work either way.
	 */
	if (ofw_bus_is_compatible(dev, "syscon")) {
		sc->syscon = syscon_create_ofw_node(dev, &syscon_class,
		    ofw_bus_get_node(dev));
		if (sc->syscon == NULL) {
			device_printf(dev, "cannot register syscon\n");
			rv = ENXIO;
			goto fail;
		}
	}

	rv = mdtk_register_clocks(dev, &clk_def);
	if (rv != 0)
		goto fail;

	return (0);

fail:
	mtx_destroy(&sc->mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
	sc->mem_res = NULL;
	return (rv);
}

static int
hifsys_clk_hwreset_assert(device_t dev, intptr_t idx, bool value)
{

	return (mdtk_clk_hwreset_assert(dev, HIFSYS_RST_BASE,
	    HIFSYS_RST_BANKS, idx, value));
}

static int
hifsys_clk_syscon_get_handle(device_t dev, struct syscon **syscon)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	*syscon = sc->syscon;
	if (*syscon == NULL) {
		return (ENODEV);
	}

	return (0);
}

static void
hifsys_clk_syscon_lock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
hifsys_clk_syscon_unlock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t mt7623_hifsys_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		 hifsys_clk_probe),
	DEVMETHOD(device_attach,	 hifsys_clk_attach),
	DEVMETHOD(device_detach, 	 hifsys_clk_detach),

	/* Clkdev interface*/
	DEVMETHOD(clkdev_read_4,        mdtk_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	    mdtk_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	    mdtk_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

	DEVMETHOD(hwreset_assert,	hifsys_clk_hwreset_assert),

	/* Syscon interface */
	DEVMETHOD(syscon_get_handle,    hifsys_clk_syscon_get_handle),
	DEVMETHOD(syscon_device_lock,   hifsys_clk_syscon_lock),
	DEVMETHOD(syscon_device_unlock, hifsys_clk_syscon_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7623_hifsys, mt7623_hifsys_driver, mt7623_hifsys_methods,
   sizeof(struct mdtk_clk_softc), syscon_class);

EARLY_DRIVER_MODULE(mt7623_hifsys, simplebus, mt7623_hifsys_driver, NULL, NULL,
   BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 4);