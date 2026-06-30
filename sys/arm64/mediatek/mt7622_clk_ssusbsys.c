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
    {"mediatek,mt7622-ssusbsys", 1},
    {NULL, 0},
};

static struct clk_gate_def gates_ssusb_clk[] = {
    GATE(CLK_SSUSB_U2_PHY_1P_EN, "ssusb_u2_phy_1p", "to_u2_phy_1p", 0x30, 0),
    GATE(CLK_SSUSB_U2_PHY_EN, "ssusb_u2_phy_en", "to_u2_phy", 0x30, 1),
    GATE(CLK_SSUSB_REF_EN, "ssusb_ref_en", "to_usb3_ref", 0x30, 5),
    GATE(CLK_SSUSB_SYS_EN, "ssusb_sys_en", "to_usb3_sys", 0x30, 6),
    GATE(CLK_SSUSB_MCU_EN, "ssusb_mcu_en", "axi_sel", 0x30, 7),
    GATE(CLK_SSUSB_DMA_EN, "ssusb_dma_en", "hif_sel", 0x30, 8),
};

static struct mdtk_clk_def clk_ssusb_def = {
        .gates_def = gates_ssusb_clk,
        .num_gates = nitems(gates_ssusb_clk),
};

static int
mt7622_ssusbsys_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data, "Mediatek mt7622 ssusbsys clocks"));
}

static int
mt7622_ssusbsys_clk_attach(device_t dev)
{
        struct mdtk_clk_softc *sc;
        static const uint16_t reset_offset[] = { 0x34 };
        sc = device_get_softc(dev);

        sc->clk_def = &clk_ssusb_def;
        sc->reset_offset = reset_offset;
        sc->reset_num = nitems(reset_offset);
        return (mdtk_clk_attach(dev));
}

static device_method_t mt7622_ssusbsys_methods[] = {
        DEVMETHOD(device_probe,  mt7622_ssusbsys_clk_probe),
        DEVMETHOD(device_attach, mt7622_ssusbsys_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_ssusbsys, mt7622_ssusbsys_driver, mt7622_ssusbsys_methods,
sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mt7622_ssusbsys, simplebus, mt7622_ssusbsys_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 5);
