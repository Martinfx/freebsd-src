/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
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
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>

#include "mdtk_clk.h"

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-infracfg", 1},
        {NULL, 0},
};

PLIST(infra_mux1_parents) = {
    "clkxtal",
    "armpll",
    "main_core_en",
    "armpll"
};

static struct clk_gate_def gates_clk[] = {
    GATE(CLK_INFRA_DBGCLK_PD, "infra_dbgclk_pd", "axi_sel", 0x44,  0),
    GATE(CLK_INFRA_TRNG, "trng_ck", "axi_sel", 0x44, 2),
    GATE(CLK_INFRA_AUDIO_PD, "infra_audio_pd", "aud_intbus_sel", 0x44, 5),
    GATE(CLK_INFRA_IRRX_PD, "infra_irrx_pd", "irrx_sel", 0x44, 16),
    GATE(CLK_INFRA_APXGPT_PD, "infra_apxgpt_pd", "f10m_ref_sel", 0x44, 18),
    GATE(CLK_INFRA_PMIC_PD, "infra_pmic_pd", "pmicspi_sel", 0x44, 22),
};

static struct clk_mux_def muxes_clk[] = {
    MUX0(CLK_INFRA_MUX1_SEL, "infra_mux1_sel", infra_mux1_parents, 0x000, 2, 2),
};

static struct mdtk_clk_def clk_def = {
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
        .muxes_def = muxes_clk,
        .num_muxes = nitems(muxes_clk),
};

static int
infracfg_clk_probe(device_t dev)
{

        return (mdtk_clk_probe(dev, compat_data, "Mediatek infracfg clocks"));
}

static int
infracfg_clk_attach(device_t dev)
{
        struct mdtk_clk_softc *sc;
        static uint16_t reset_offset[] = { 0x30 };
        sc = device_get_softc(dev);

        sc->clk_def = &clk_def;
        sc->reset_offset = reset_offset;
        sc->reset_num = nitems(reset_offset);
        return (mdtk_clk_attach(dev));
}

static device_method_t mt7622_infracfg_methods[] = {
        DEVMETHOD(device_probe,  infracfg_clk_probe),
        DEVMETHOD(device_attach, infracfg_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_infracfg, mt7622_infracfg_driver, mt7622_infracfg_methods,
sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mt7622_infracfg, simplebus, mt7622_infracfg_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 4);
