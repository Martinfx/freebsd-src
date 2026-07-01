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

#define PERICFG_CG0  0x10
#define PERICFG_CG1  0x14

static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-pericfg",	1},
    {NULL,		 	0},
};

PLIST(peribus_ck_parents) = {
    "syspll1_d8",
    "syspll1_d4"
};

static struct clk_gate_def gates_clk[] = {
    GATE(CLK_PERI_THERM_PD, "peri_therm_pd", "axi_sel", PERICFG_CG0, 1),
    GATE(CLK_PERI_PWM1_PD, "peri_pwm1_pd", "clkxtal", PERICFG_CG0, 2),
    GATE(CLK_PERI_PWM2_PD, "peri_pwm2_pd", "clkxtal", PERICFG_CG0, 3),
    GATE(CLK_PERI_PWM3_PD, "peri_pwm3_pd", "clkxtal", PERICFG_CG0, 4),
    GATE(CLK_PERI_PWM4_PD, "peri_pwm4_pd", "clkxtal", PERICFG_CG0, 5),
    GATE(CLK_PERI_PWM5_PD, "peri_pwm5_pd", "clkxtal", PERICFG_CG0, 6),
    GATE(CLK_PERI_PWM6_PD, "peri_pwm6_pd", "clkxtal", PERICFG_CG0, 7),
    GATE(CLK_PERI_PWM7_PD, "peri_pwm7_pd", "clkxtal", PERICFG_CG0, 8),
    GATE(CLK_PERI_PWM_PD, "peri_pwm_pd", "clkxtal", PERICFG_CG0, 9),
    GATE(CLK_PERI_AP_DMA_PD, "peri_ap_dma_pd", "axi_sel", PERICFG_CG0, 12),
    GATE(CLK_PERI_MSDC30_0_PD, "peri_msdc30_0", "msdc30_0_sel", PERICFG_CG0, 13),
    GATE(CLK_PERI_MSDC30_1_PD, "peri_msdc30_1", "msdc30_1_sel", PERICFG_CG0, 14),
    GATE(CLK_PERI_UART0_PD, "peri_uart0_pd", "axi_sel", PERICFG_CG0, 17),
    GATE(CLK_PERI_UART1_PD, "peri_uart1_pd", "axi_sel", PERICFG_CG0, 18),
    GATE(CLK_PERI_UART2_PD, "peri_uart2_pd", "axi_sel", PERICFG_CG0, 19),
    GATE(CLK_PERI_UART3_PD, "peri_uart3_pd", "axi_sel", PERICFG_CG0, 20),
    GATE(CLK_PERI_UART4_PD, "peri_uart4_pd", "axi_sel", PERICFG_CG0, 21),
    GATE(CLK_PERI_BTIF_PD, "peri_btif_pd", "axi_sel", PERICFG_CG0, 22),
    GATE(CLK_PERI_I2C0_PD, "peri_i2c0_pd", "axi_sel", PERICFG_CG0, 23),
    GATE(CLK_PERI_I2C1_PD, "peri_i2c1_pd", "axi_sel", PERICFG_CG0, 24),
    GATE(CLK_PERI_I2C2_PD, "peri_i2c2_pd", "axi_sel", PERICFG_CG0, 25),
    GATE(CLK_PERI_SPI1_PD, "peri_spi1_pd", "spi1_sel", PERICFG_CG0, 26),
    GATE(CLK_PERI_AUXADC_PD, "peri_auxadc_pd", "clkxtal", PERICFG_CG0, 27),
    GATE(CLK_PERI_SPI0_PD, "peri_spi0_pd", "spi0_sel", PERICFG_CG0, 28),
    GATE(CLK_PERI_SNFI_PD, "peri_snfi_pd", "spinfi_infra_bclk_sel", PERICFG_CG0, 29),
    GATE(CLK_PERI_NFI_PD, "peri_nfi_pd", "axi_sel", PERICFG_CG0, 30),
    GATE(CLK_PERI_NFIECC_PD, "peri_nfiecc_pd", "axi_sel", PERICFG_CG0, 31),

    GATE(CLK_PERI_FLASH_PD, "peri_flash_pd", "flash_sel", PERICFG_CG1, 1),
    GATE(CLK_PERI_IRTX_PD, "peri_irtx_pd", "irtx_sel", PERICFG_CG1, 2),
};

static struct clk_mux_def muxes_clk[] = {
    MUX0(0, "peribus_ck_sel_mux", peribus_ck_parents, 0x05C, 0 , 1),
};

static struct mdtk_clk_def clk_def = {
    .gates_def = gates_clk,
    .num_gates = nitems(gates_clk),
    .muxes_def = muxes_clk,
    .num_muxes = nitems(muxes_clk),
};

static int
pericfg_clk_probe(device_t dev)
{
        return (mdtk_clk_probe(dev, compat_data, "Mediatek pericfg clocks"));
}

static int
pericfg_clk_attach(device_t dev)
{
        struct mdtk_clk_reset_softc *sc;
        static const uint16_t reset_offset[] = { 0x0, 0x4 };
        sc = device_get_softc(dev);

        device_printf(dev, "sc=%p driver=%p\n", sc, dev->driver);

        sc->clk_sc->clk_def = &clk_def;
        sc->reset_offset = reset_offset;
        sc->reset_num = nitems(reset_offset);
        return (mdtk_clk_attach(dev));
}

static device_method_t mdtk_mt7622_pericfg_methods[] = {
        DEVMETHOD(device_probe,  pericfg_clk_probe),
        DEVMETHOD(device_attach, pericfg_clk_attach),
        DEVMETHOD_END
};

DEFINE_CLASS_1(mdtk_mt7622_pericfg, mdtk_mt7622_pericfg_driver,
    mdtk_mt7622_pericfg_methods, sizeof(struct mdtk_clk_reset_softc),
        mdtk_clk_reset_driver);

EARLY_DRIVER_MODULE(mdtk_mt7622_pericfg, simplebus, mdtk_mt7622_pericfg_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 3);
