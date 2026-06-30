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

struct audsys_softc {
    struct simplebus_softc simplebus_sc;
    struct mdtk_clk_softc clk_sc;
};

static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-audsys", 1},
    {NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
    /* AUDIO0 */
    GATE(CLK_AUDIO_AFE, "audio_afe", "rtc", 0x0, 2),
    GATE(CLK_AUDIO_HDMI, "audio_hdmi", "apll1_ck_sel", 0x0, 20),
    GATE(CLK_AUDIO_SPDF, "audio_spdf", "apll1_ck_sel", 0x0, 21),
    GATE(CLK_AUDIO_APLL, "audio_apll", "apll1_ck_sel", 0x0, 23),
    /* AUDIO1 */
    GATE(CLK_AUDIO_I2SIN1, "audio_i2sin1", "a1sys_hp_sel", 0x10, 0),
    GATE(CLK_AUDIO_I2SIN2, "audio_i2sin2", "a1sys_hp_sel", 0x10, 1),
    GATE(CLK_AUDIO_I2SIN3, "audio_i2sin3", "a1sys_hp_sel", 0x10, 2),
    GATE(CLK_AUDIO_I2SIN4, "audio_i2sin4", "a1sys_hp_sel", 0x10, 3),
    GATE(CLK_AUDIO_I2SO1, "audio_i2so1", "a1sys_hp_sel", 0x10, 6),
    GATE(CLK_AUDIO_I2SO2, "audio_i2so2", "a1sys_hp_sel", 0x10, 7),
    GATE(CLK_AUDIO_I2SO3, "audio_i2so3", "a1sys_hp_sel", 0x10, 8),
    GATE(CLK_AUDIO_I2SO4, "audio_i2so4", "a1sys_hp_sel", 0x10, 9),
    GATE(CLK_AUDIO_ASRCI1, "audio_asrci1", "asm_h_sel", 0x10, 12),
    GATE(CLK_AUDIO_ASRCI2, "audio_asrci2", "asm_h_sel", 0x10, 13),
    GATE(CLK_AUDIO_ASRCO1, "audio_asrco1", "asm_h_sel", 0x10, 14),
    GATE(CLK_AUDIO_ASRCO2, "audio_asrco2", "asm_h_sel", 0x10, 15),
    GATE(CLK_AUDIO_INTDIR, "audio_intdir", "intdir_sel", 0x10, 20),
    GATE(CLK_AUDIO_A1SYS, "audio_a1sys", "a1sys_hp_sel", 0x10, 21),
    GATE(CLK_AUDIO_A2SYS, "audio_a2sys", "a2sys_hp_sel", 0x10, 22),
    GATE(CLK_AUDIO_AFE_CONN, "audio_afe_conn", "a1sys_hp_sel", 0x10, 23),
    /* AUDIO2 */
    GATE(CLK_AUDIO_UL1, "audio_ul1", "a1sys_hp_sel", 0x14, 0),
    GATE(CLK_AUDIO_UL2, "audio_ul2", "a1sys_hp_sel", 0x14, 1),
    GATE(CLK_AUDIO_UL3, "audio_ul3", "a1sys_hp_sel", 0x14, 2),
    GATE(CLK_AUDIO_UL4, "audio_ul4", "a1sys_hp_sel", 0x14, 3),
    GATE(CLK_AUDIO_UL5, "audio_ul5", "a1sys_hp_sel", 0x14, 4),
    GATE(CLK_AUDIO_UL6, "audio_ul6", "a1sys_hp_sel", 0x14, 5),
    GATE(CLK_AUDIO_DL1, "audio_dl1", "a1sys_hp_sel", 0x14, 6),
    GATE(CLK_AUDIO_DL2, "audio_dl2", "a1sys_hp_sel", 0x14, 7),
    GATE(CLK_AUDIO_DL3, "audio_dl3", "a1sys_hp_sel", 0x14, 8),
    GATE(CLK_AUDIO_DL4, "audio_dl4", "a1sys_hp_sel", 0x14, 9),
    GATE(CLK_AUDIO_DL5, "audio_dl5", "a1sys_hp_sel", 0x14, 10),
    GATE(CLK_AUDIO_DL6, "audio_dl6", "a1sys_hp_sel", 0x14, 11),
    GATE(CLK_AUDIO_DLMCH, "audio_dlmch", "a1sys_hp_sel", 0x14, 12),
    GATE(CLK_AUDIO_ARB1, "audio_arb1", "a1sys_hp_sel", 0x14, 13),
    GATE(CLK_AUDIO_AWB, "audio_awb", "a1sys_hp_sel", 0x14, 14),
    GATE(CLK_AUDIO_AWB2, "audio_awb2", "a1sys_hp_sel", 0x14, 15),
    GATE(CLK_AUDIO_DAI, "audio_dai", "a1sys_hp_sel", 0x14, 16),
    GATE(CLK_AUDIO_MOD, "audio_mod", "a1sys_hp_sel", 0x14, 17),
    /* AUDIO3 */
    GATE(CLK_AUDIO_ASRCI3, "audio_asrci3", "asm_h_sel", 0x634, 2),
    GATE(CLK_AUDIO_ASRCI4, "audio_asrci4", "asm_h_sel", 0x634, 3),
    GATE(CLK_AUDIO_ASRCO3, "audio_asrco3", "asm_h_sel", 0x634, 6),
    GATE(CLK_AUDIO_ASRCO4, "audio_asrco4", "asm_h_sel", 0x634, 7),
    GATE(CLK_AUDIO_MEM_ASRC1, "audio_mem_asrc1", "asm_h_sel", 0x634, 10),
    GATE(CLK_AUDIO_MEM_ASRC2, "audio_mem_asrc2", "asm_h_sel", 0x634, 11),
    GATE(CLK_AUDIO_MEM_ASRC3, "audio_mem_asrc3", "asm_h_sel", 0x634, 12),
    GATE(CLK_AUDIO_MEM_ASRC4, "audio_mem_asrc4", "asm_h_sel", 0x634, 13),
    GATE(CLK_AUDIO_MEM_ASRC5, "audio_mem_asrc5", "asm_h_sel", 0x634, 14),
};

static struct mdtk_clk_def clk_def = {
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
};

static int
audio_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data, "Mediatek mt7622 audio clocks"));
}

static int
audio_clk_attach(device_t dev)
{
        struct audsys_softc *sc;
        phandle_t node;

        sc = device_get_softc(dev);
        sc->clk_sc.clk_def = &clk_def;
        node = ofw_bus_get_node(dev);
        if (node == -1) {
                return (ENXIO);
        }

        bus_identify_children(dev);

        simplebus_init(dev, node);
        for (node = OF_child(node); node > 0; node = OF_peer(node)) {
                simplebus_add_device(dev, node, 0, NULL, -1, NULL);
        }

        bus_attach_children(dev);

        return (mdtk_clk_attach_sc(dev, &sc->clk_sc));
}

static device_method_t mt7622_audio_methods[] = {
        DEVMETHOD(device_probe,  audio_clk_probe),
        DEVMETHOD(device_attach, audio_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_audio, mt7622_audio_driver, mt7622_audio_methods,
sizeof(struct audsys_softc), simplebus_driver);

EARLY_DRIVER_MODULE(mt7622_audio, simplebus, mt7622_audio_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 4);
MODULE_VERSION(mt7622_audio, 1);