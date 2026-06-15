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
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/mt2701-clk.h>
#include "syscon_if.h"
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7623-imgsys", 1},
	{"mediatek,mt2701-imgsys", 1},
	{NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
	GATE(CLK_IMG_SMI_COMM, "img_smi_comm", "mm_sel", 0x0000, 0),
	GATE(CLK_IMG_RESZ, "img_resz", "mm_sel", 0x0000, 1),
	GATE(CLK_IMG_JPGDEC_SMI, "img_jpgdec_smi", "mm_sel", 0x0000, 5),
	GATE(CLK_IMG_JPGDEC, "img_jpgdec", "mm_sel", 0x0000, 6),
	GATE(CLK_IMG_VENC_LT, "img_venc_lt", "mm_sel", 0x0000, 8),
	GATE(CLK_IMG_VENC, "img_venc", "mm_sel", 0x0000, 9),
};

static struct mdtk_clk_def clk_def = {
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
};

static int
imgsys_clk_detach(device_t dev)
{
	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static int
imgsys_clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Mediatek imgsys clocks");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
imgsys_clk_attach(device_t dev) {
	struct mdtk_clk_softc *sc = device_get_softc(dev);
	int rid = 0;

	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (ofw_bus_is_compatible(dev, "syscon")) {
		sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
		    RF_ACTIVE);
		if (sc->mem_res == NULL) {
			device_printf(dev,
			    "Cannot allocate memory resource\n");
			return (ENXIO);
		}


	}

	mdtk_register_clocks(dev, &clk_def);
	return (0);
}

static int
imgsys_clk_hwreset_assert(device_t dev, intptr_t idx, bool value)
{
	struct mdtk_clk_softc *sc = device_get_softc(dev);
	uint32_t mask, reset_reg;

	CLKDEV_DEVICE_LOCK(sc->dev);
	KASSERT((idx > 0 && idx < 32), ("%s: idx out of range",__func__));


	mask = 1 << (idx % 32);
	reset_reg = (idx / 32) * 4;

	CLKDEV_MODIFY_4(sc->dev, reset_reg, mask, value ? mask : 0);
	CLKDEV_DEVICE_UNLOCK(sc->dev);

	return(0);
}

static int
imgsys_clk_syscon_get_handle(device_t dev, struct syscon **syscon)
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
imgsys_clk_syscon_lock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
imgsys_clk_syscon_unlock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t mt7622_imgsys_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		 imgsys_clk_probe),
	DEVMETHOD(device_attach,	 imgsys_clk_attach),
	DEVMETHOD(device_detach, 	 imgsys_clk_detach),

	/* Clkdev interface*/
	DEVMETHOD(clkdev_read_4,        mdtk_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	    mdtk_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	    mdtk_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

	DEVMETHOD(hwreset_assert,	imgsys_clk_hwreset_assert),

	/* Syscon interface */
	DEVMETHOD(syscon_get_handle,    imgsys_clk_syscon_get_handle),
	DEVMETHOD(syscon_device_lock,   imgsys_clk_syscon_lock),
	DEVMETHOD(syscon_device_unlock, imgsys_clk_syscon_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_imgsys, mt7622_imgsys_driver, mt7622_imgsys_methods,
    sizeof(struct mdtk_clk_softc), syscon_class);

EARLY_DRIVER_MODULE(mt7622_imgsys, simplebus, mt7622_imgsys_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 4);