/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 * Copyright (c) 2025 Michal Meloun <mmel@FreeBSD.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/clock/mt7622-clk.h>
#include <dev/clk/clk_gate.h>

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
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
};

static int
eth_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data,
            "Mediatek mt7622 ethernet clocks"));
}

static int
eth_clk_attach(device_t dev)
{
        struct mdtk_clk_softc *sc;
        static const uint16_t reset_offset[] = { 0x34 };
        sc = device_get_softc(dev);

        sc->clk_def = &clk_def;
        sc->reset_offset = reset_offset;
        sc->reset_num = nitems(reset_offset);
        return (mdtk_clk_attach(dev));
}

static device_method_t mt7622_eth_methods[] = {
        DEVMETHOD(device_probe,  eth_clk_probe),
        DEVMETHOD(device_attach, eth_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_eth, mt7622_eth_driver, mt7622_eth_methods,
sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mt7622_eth, simplebus, mt7622_eth_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 5);
