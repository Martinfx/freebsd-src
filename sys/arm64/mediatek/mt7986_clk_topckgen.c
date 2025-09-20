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

#include <dt-bindings/clock/mt7986-clk.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <arm64/mediatek/mdtk_clk.h>
#include <dev/hwreset/hwreset.h>
#include "clkdev_if.h"

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7986-topckgen",	1},
        {NULL,		 	0},
};

PLIST(fi1x_ck_parents) = {
    "top_xtal",
    "top_mmpll_d8",
    "top_net1pll_d8_d2",
    "top_net2pll_d3_d2",
    "top_mpll_d4",
    "top_mmpll_d8_d2",
    "top_wedmcupll_d5_d2",
    "top_mpll_d8"
};

PLIST(spinfi_ck_parents) = {
        "top_xtal_d2",
        "top_xtal",
        "top_net1pll_d5_d4",
        "top_mpll_d4",
        "top_mmpll_d8_d2",
        "top_wedmcupll_d5_d2",
        "top_mmpll_d3_d8",
        "top_mpll_d8"
};

PLIST(spi_ck_parents) = {
        "top_xtal",
        "top_mpll_d2",
        "top_mmpll_d8",
        "top_net1pll_d8_d2",
        "top_net2pll_d3_d2",
        "top_net1pll_d5_d4",
        "top_mpll_d4",
        "top_wedmcupll_d5_d2"
};

static struct clk_fixed_def fixed_clk[] = {
        FRATE(CLK_TOP_XTAL, "top_xtal", "clkxtal", 40000000),
        FRATE(CLK_TOP_JTAG, "top_jtag", "clkxtal", 50000000),
        /* XTAL */
        FFACT(CLK_TOP_XTAL_D2, "top_xtal_d2", "top_xtal", 1, 2),
        FFACT(CLK_TOP_RTC_32K, "top_rtc_32k", "top_xtal", 1, 1250),
        FFACT(CLK_TOP_RTC_32P7K, "top_rtc_32p7k", "top_xtal", 1, 1220),
        /* MPLL */
        FFACT(CLK_TOP_MPLL_D2, "top_mpll_d2", "mpll", 1, 2),
        FFACT(CLK_TOP_MPLL_D4, "top_mpll_d4", "mpll", 1, 4),
        FFACT(CLK_TOP_MPLL_D8, "top_mpll_d8", "mpll", 1, 8),
        FFACT(CLK_TOP_MPLL_D8_D2, "top_mpll_d8_d2", "mpll", 1, 16),
        FFACT(CLK_TOP_MPLL_D3_D2, "top_mpll_d3_d2", "mpll", 1, 6),
        /* MMPLL */
        FFACT(CLK_TOP_MMPLL_D2, "top_mmpll_d2", "mmpll", 1, 2),
        FFACT(CLK_TOP_MMPLL_D4, "top_mmpll_d4", "mmpll", 1, 4),
        FFACT(CLK_TOP_MMPLL_D8, "top_mmpll_d8", "mmpll", 1, 8),
        FFACT(CLK_TOP_MMPLL_D8_D2, "top_mmpll_d8_d2", "mmpll", 1, 16),
        FFACT(CLK_TOP_MMPLL_D3_D8, "top_mmpll_d3_d8", "mmpll", 1, 24),
        FFACT(CLK_TOP_MMPLL_U2PHY, "top_mmpll_u2phy", "mmpll", 1, 30),
        /* APLL2 */
        FFACT(CLK_TOP_APLL2_D4, "top_apll2_d4", "apll2", 1, 4),
        /* NET1PLL */
        FFACT(CLK_TOP_NET1PLL_D4, "top_net1pll_d4", "net1pll", 1, 4),
        FFACT(CLK_TOP_NET1PLL_D5, "top_net1pll_d5", "net1pll", 1, 5),
        FFACT(CLK_TOP_NET1PLL_D5_D2, "top_net1pll_d5_d2", "net1pll", 1, 10),
        FFACT(CLK_TOP_NET1PLL_D5_D4, "top_net1pll_d5_d4", "net1pll", 1, 20),
        FFACT(CLK_TOP_NET1PLL_D8_D2, "top_net1pll_d8_d2", "net1pll", 1, 16),
        FFACT(CLK_TOP_NET1PLL_D8_D4, "top_net1pll_d8_d4", "net1pll", 1, 32),
        /* NET2PLL */
        FFACT(CLK_TOP_NET2PLL_D4, "top_net2pll_d4", "net2pll", 1, 4),
        FFACT(CLK_TOP_NET2PLL_D4_D2, "top_net2pll_d4_d2", "net2pll", 1, 8),
        FFACT(CLK_TOP_NET2PLL_D3_D2, "top_net2pll_d3_d2", "net2pll", 1, 2),
        /* WEDMCUPLL */
        FFACT(CLK_TOP_WEDMCUPLL_D5_D2, "top_wedmcupll_d5_d2", "wedmcupll", 1, 10),
};

static struct clk_gate_def gatesclk[] = {
        /* CLK_CFG_0 */
        GATE(CLK_TOP_NFI1X, "nfi1x_sel", "nfi1x_sel_mux", 0x000, 3),
        GATE(CLK_TOP_SPINFI, "spinfi_sel", "spinfi_sel_mux",       0x000, 15),
        GATE(CLK_TOP_SPI, "spi_sel",  "spi_sel_mux",          0x000, 23),
        GATE(CLK_TOP_SPIM_MST, "spim_mst_sel", "spim_mst_sel_mux",     0x000, 31),s
};

static struct clk_mux_def muxes_clk[] = {
        MUX0(0, "nfi1x_sel_mux", fi1x_ck_parents, 0x000, 24, 3),
        MUX0(0, "spinfi_sel_mux", spinfi_ck_parents, 0x000, 16, 1),
        MUX0(0, "spi_sel_mux", mem_ck_parents, 0x000, 8, 1),
};

static struct mdtk_clk_def clk_def = {
        .fixed_def = fixed_clk,
        .num_fixed = nitems(fixed_clk),
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
        .muxes_def = muxes_clk,
        .num_muxes = nitems(muxes_clk),
};

static int
topckgen_clk_detach(device_t dev)
{
    device_printf(dev, "Error: Clock driver cannot be detached\n");
    return (EBUSY);
}

static int
topckgen_clk_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
        device_set_desc(dev, "Mediatek 7986 Topckgen clocks");
        return (BUS_PROBE_DEFAULT);
    }

    return (ENXIO);
}

static int
topckgen_clk_attach(device_t dev) {
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

static device_method_t mt7986_topckgen_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		 topckgen_clk_probe),
        DEVMETHOD(device_attach,	     topckgen_clk_attach),
        DEVMETHOD(device_detach, 	 topckgen_clk_detach),

        /* Clkdev interface*/
        DEVMETHOD(clkdev_read_4,	mdtk_clkdev_read_4),
        DEVMETHOD(clkdev_write_4,	mdtk_clkdev_write_4),
        DEVMETHOD(clkdev_modify_4,	mdtk_clkdev_modify_4),
        DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
        DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

        DEVMETHOD_END
};

DEFINE_CLASS_0(mt7986_topckgen, mt7986_topckgen_driver, mt7986_topckgen_methods,
sizeof(struct mdtk_clk_softc));

EARLY_DRIVER_MODULE(mt7986_topckgen, simplebus, mt7986_topckgen_driver, NULL, NULL,
        BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 2);