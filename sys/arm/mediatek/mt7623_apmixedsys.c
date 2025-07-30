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

#include <dt-bindings/clock/mt2701-clk.h>
#include <dev/clk/clk_fixed.h>
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"
#include "mt_clk_pll.h"

#define PLL_AO	(1u << 1)

static struct ofw_compat_data compat_data[] = {
       {"mediatek,mt7623-apmixedsys", 1},
	{"mediatek,mt2712-apmixedsys", 1},
       {NULL, 0},
};

static struct clk_pll_def pll_clk[] = {
	PLL(CLK_APMIXED_ARMPLL, "armpll", "clk26m", 0x200, 0x20c, 0x80000000,
	    PLL_AO, 21, 0x204, 24, 0x0, 0x204, 0),
	PLL(CLK_APMIXED_MAINPLL, "mainpll", "clk26m", 0x210, 0x21c, 0xf0000000,
	    0, 21, 0x210, 4, 0x0, 0x214, 0),
	PLL(CLK_APMIXED_UNIVPLL, "univpll", "clk26m", 0x220, 0x22c, 0xf3000000,
	    0, 7, 0x220, 4, 0x0, 0x224, 14),
	PLL(CLK_APMIXED_MMPLL, "mmpll", "clk26m", 0x230, 0x23c, 0, 0,
	    21, 0x230, 4, 0x0, 0x234, 0),
	PLL(CLK_APMIXED_MSDCPLL, "msdcpll", "clk26m", 0x240, 0x24c, 0x00000001, 0,
	    21, 0x240, 4, 0x0, 0x244, 0),
	PLL(CLK_APMIXED_TVDPLL, "tvdpll", "clk26m", 0x250, 0x25c, 0x00000001, 0,
	    21, 0x250, 4, 0x0, 0x254, 0),
	PLL(CLK_APMIXED_AUD1PLL, "aud1pll", "clk26m", 0x270, 0x27c, 0x00000001, 0,
	    31, 0x270, 4, 0x0, 0x274, 0),
	PLL(CLK_APMIXED_TRGPLL, "trgpll", "clk26m", 0x280, 0x28c, 0x00000001, 0,
	    31, 0x280, 4, 0x0, 0x284, 0),
	PLL(CLK_APMIXED_ETHPLL, "ethpll", "clk26m", 0x290, 0x29c, 0x00000001, 0,
	    31, 0x290, 4, 0x0, 0x294, 0),
	PLL(CLK_APMIXED_VDECPLL, "vdecpll", "clk26m", 0x2a0, 0x2ac, 0x00000001, 0,
	    31, 0x2a0, 4, 0x0, 0x2a4, 0),
	PLL(CLK_APMIXED_HADDS2PLL, "hadds2pll", "clk26m", 0x2b0, 0x2bc, 0x00000001, 0,
	    31, 0x2b0, 4, 0x0, 0x2b4, 0),
	PLL(CLK_APMIXED_AUD2PLL, "aud2pll", "clk26m", 0x2c0, 0x2cc, 0x00000001, 0,
	    31, 0x2c0, 4, 0x0, 0x2c4, 0),
	PLL(CLK_APMIXED_TVD2PLL, "tvd2pll", "clk26m", 0x2d0, 0x2dc, 0x00000001, 0,
	    21, 0x2d0, 4, 0x0, 0x2d4, 0),
};

static struct clk_fixed_def fixed_clk[] = {
	FFACT(CLK_APMIXED_HDMI_REF, "hdmi_ref", "tvdpll", 1, 1),
};

static struct mdtk_clk_def clk_def = {
       .pll_def = pll_clk,
       .num_pll = nitems(pll_clk),
       .fixed_def = fixed_clk,
       .num_fixed = nitems(fixed_clk),
};

static int
apmixedsys_clk_detach(device_t dev)
{
       device_printf(dev, "Error: Clock driver cannot be detached\n");
       return (EBUSY);
}

static int
apmixedsys_clk_probe(device_t dev)
{
       if (!ofw_bus_status_okay(dev))
	       return (ENXIO);

       if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
	       device_set_desc(dev, "Mediatek mt7623 apmixedsys clocks");
	       return (BUS_PROBE_DEFAULT);
       }

       return (ENXIO);
}

static int
apmixedsys_clk_attach(device_t dev) {
       struct mdtk_clk_softc *sc = device_get_softc(dev);
       int rid, rv;

       sc->dev = dev;

       mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

       /* Resource setup. */
       rid = 0;
       sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	   RF_ACTIVE);
       if (!sc->mem_res) {
	       device_printf(dev, "cannot allocate memory resource\n");
	       rv = ENXIO;
	       goto fail;
       }

       mdtk_register_clocks(dev,  &clk_def);
       return (0);

fail:
       if (sc->mem_res)
	       bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

       return (rv);
}

static device_method_t mt7623_apmixedsys_methods[] = {
       /* Device interface */
       DEVMETHOD(device_probe,		 apmixedsys_clk_probe),
       DEVMETHOD(device_attach,	 apmixedsys_clk_attach),
       DEVMETHOD(device_detach, 	 apmixedsys_clk_detach),

       /* Clkdev interface*/
       DEVMETHOD(clkdev_read_4,        mdtk_clkdev_read_4),
       DEVMETHOD(clkdev_write_4,	    mdtk_clkdev_write_4),
       DEVMETHOD(clkdev_modify_4,	    mdtk_clkdev_modify_4),
       DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
       DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),
       DEVMETHOD_END
};

DEFINE_CLASS_0(mt7623_apmixedsys, mt7623_apmixedsys_driver, mt7623_apmixedsys_methods,
   sizeof(struct mdtk_clk_softc));
EARLY_DRIVER_MODULE(mt7623_apmixedsys, simplebus, mt7623_apmixedsys_driver, NULL, NULL,
   BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);