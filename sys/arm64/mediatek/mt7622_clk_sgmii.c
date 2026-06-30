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
#include <dev/clk/clk_gate.h>

#include "mdtk_clk.h"

static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-sgmiisys", 1},
    {NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
    /* sgmii */
    GATE(CLK_SGMII_TX250M_EN, "sgmii_tx250m_en", "ssusb_tx250m", 0xE4, 2),
    GATE(CLK_SGMII_RX250M_EN, "sgmii_rx250m_en", "ssusb_eq_rx250m", 0xE4, 3),
    GATE(CLK_SGMII_CDR_REF, "sgmii_cdr_ref", "ssusb_cdr_ref", 0xE4, 4),
    GATE(CLK_SGMII_CDR_FB, "sgmii_cdr_fb", "ssusb_cdr_fb", 0xE4, 5),
};

static struct mdtk_clk_def clk_def = {
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
};

static int
sgmii_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data, "Mediatek mt7622 sgmii clocks"));
}

static int
sgmii_clk_attach(device_t dev)
{
        struct mdtk_clk_softc *sc;
        sc = device_get_softc(dev);

        sc->clk_def = &clk_def;
        return (mdtk_clk_attach(dev));
}

static device_method_t mt7622_sgmii_methods[] = {
        DEVMETHOD(device_probe,  sgmii_clk_probe),
        DEVMETHOD(device_attach, sgmii_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_sgmii, mt7622_sgmii_driver, mt7622_sgmii_methods,
sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mt7622_sgmii, simplebus, mt7622_sgmii_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 5);
