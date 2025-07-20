/*-
 * Copyright (c) 2025 Martin Filla
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

#include <dt-bindings/clock/mt7622-clk.h>

#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <arm64/mediatek/mdtk_clk.h>
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-ethsys", 1},
        {NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
        /* eth */
        GATE(CLK_ETH_HSDMA_EN, "eth_hsdma_en", "eth_sel", 0x30, 5),
        GATE(CLK_ETH_ESW_EN, "eth_esw_en", "eth_500m", 0x30, 6),
        GATE(CLK_ETH_GP2_EN, "eth_gp2_en", "txclk_src_pre", 0x30, 7),
        GATE(CLK_ETH_GP1_EN, "eth_gp1_en", "txclk_src_pre", 0x30, 8),
        GATE(CLK_ETH_GP0_EN, "eth_gp0_en", "txclk_src_pre", 0x30, 9),
};

static struct mdtk_clk_def clk_def = {
        .linked_def = NULL,
        .num_linked = 0,
        .fixed_def = NULL,
        .num_fixed = 0,
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
        .muxes_def = NULL,
        .num_muxes = 0,
};

static int
eth_clk_detach(device_t dev)
{
    device_printf(dev, "Error: Clock driver cannot be detached\n");
    return (EBUSY);
}

static int
eth_clk_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
        device_set_desc(dev, "Mediatek mt7622 ethernet clocks");
        return (BUS_PROBE_DEFAULT);
    }

    return (ENXIO);
}

static int
eth_clk_attach(device_t dev) {
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

static device_method_t mt7622_eth_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		 eth_clk_probe),
        DEVMETHOD(device_attach,	 eth_clk_attach),
        DEVMETHOD(device_detach, 	 eth_clk_detach),

        /* Clkdev interface*/
        DEVMETHOD(clkdev_read_4,        mdtk_clkdev_read_4),
        DEVMETHOD(clkdev_write_4,	    mdtk_clkdev_write_4),
        DEVMETHOD(clkdev_modify_4,	    mdtk_clkdev_modify_4),
        DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
        DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),
        DEVMETHOD_END
};

DEFINE_CLASS_0(mt7622_eth, mt7622_eth_driver, mt7622_eth_methods,
sizeof(struct mdtk_clk_softc));

EARLY_DRIVER_MODULE(mt7622_eth, simplebus, mt7622_eth_driver, NULL, NULL,
        BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 5);