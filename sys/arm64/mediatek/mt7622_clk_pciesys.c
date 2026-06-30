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
    {"mediatek,mt7622-pciesys", 1},
    {NULL,                      0},
};

static struct clk_gate_def gates_pcie_clk[] = {
    GATE(CLK_PCIE_P1_AUX_EN, "pcie_p1_aux_en", "p1_1mhz", 0x30, 12),
    GATE(CLK_PCIE_P1_OBFF_EN, "pcie_p1_obff_en", "free_run_4mhz", 0x30, 13),
    GATE(CLK_PCIE_P1_AHB_EN, "pcie_p1_ahb_en", "axi_sel", 0x30, 14),
    GATE(CLK_PCIE_P1_AXI_EN, "pcie_p1_axi_en", "hif_sel", 0x30, 15),
    GATE(CLK_PCIE_P1_MAC_EN, "pcie_p1_mac_en", "pcie1_mac_en", 0x30, 16),
    GATE(CLK_PCIE_P1_PIPE_EN, "pcie_p1_pipe_en", "pcie1_pipe_en", 0x30, 17),
    GATE(CLK_PCIE_P0_AUX_EN, "pcie_p0_aux_en", "p0_1mhz", 0x30, 18),
    GATE(CLK_PCIE_P0_OBFF_EN, "pcie_p0_obff_en", "free_run_4mhz", 0x30, 19),
    GATE(CLK_PCIE_P0_AHB_EN, "pcie_p0_ahb_en", "axi_sel", 0x30, 20),
    GATE(CLK_PCIE_P0_AXI_EN, "pcie_p0_axi_en", "hif_sel", 0x30, 21),
    GATE(CLK_PCIE_P0_MAC_EN, "pcie_p0_mac_en", "pcie0_mac_en", 0x30, 22),
    GATE(CLK_PCIE_P0_PIPE_EN, "pcie_p0_pipe_en", "pcie0_pipe_en", 0x30, 23),
    GATE(CLK_SATA_AHB_EN, "sata_ahb_en", "axi_sel", 0x30, 26),
    GATE(CLK_SATA_AXI_EN, "sata_axi_en", "hif_sel", 0x30, 27),
    GATE(CLK_SATA_ASIC_EN, "sata_asic_en", "sata_asic", 0x30, 28),
    GATE(CLK_SATA_RBC_EN, "sata_rbc_en", "sata_rbc", 0x30, 29),
    GATE(CLK_SATA_PM_EN, "sata_pm_en", "univpll2_d4", 0x30, 30),
};

static struct mdtk_clk_def clk_pcie_def = {
        .gates_def = gates_pcie_clk,
        .num_gates = nitems(gates_pcie_clk),
};

static int
mt7622_pciesys_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data, "Mediatek mt7622 pciesys clocks"));
}

static int
mt7622_pciesys_clk_attach(device_t dev)
{
        struct mdtk_clk_softc *sc;
        static const uint16_t reset_offset[] = { 0x34 };
        sc = device_get_softc(dev);

        sc->clk_def = &clk_pcie_def;
        sc->reset_offset = reset_offset;
        sc->reset_num = nitems(reset_offset);
        return (mdtk_clk_attach(dev));
}

static device_method_t mt7622_pciesys_clk_methods[] = {
        DEVMETHOD(device_probe,  mt7622_pciesys_clk_probe),
        DEVMETHOD(device_attach, mt7622_pciesys_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_pciesys, mt7622_pciesys_driver, mt7622_pciesys_clk_methods,
sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mt7622_pciesys, simplebus, mt7622_pciesys_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 5);
