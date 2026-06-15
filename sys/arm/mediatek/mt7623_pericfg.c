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
#include <dev/syscon/syscon.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/mt2701-clk.h>
#include "syscon_if.h"
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"

#define PERICFG_CG0  0x10
#define PERICFG_CG1  0x14

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7623-pericfg", 1},
        {"mediatek,mt2701-pericfg", 1},
        {NULL,                      0},
};

PLIST(uart_ck_sel_parents) = {
        "clk26m",
        "uart_sel",
};

static struct clk_gate_def gates_clk[] = {
        GATE(CLK_PERI_USB0_MCU, "usb0_mcu_ck", "axi_sel", 0x0018, 31),
        GATE(CLK_PERI_ETH, "eth_ck", "clk26m", 0x0018, 30),
        GATE(CLK_PERI_SPI0, "spi0_ck", "spi0_sel", 0x0018, 29),
        GATE(CLK_PERI_AUXADC, "auxadc_ck", "clk26m", 0x0018, 28),
        GATE(CLK_PERI_I2C3, "i2c3_ck", "clk26m", 0x0018, 27),
        GATE(CLK_PERI_I2C2, "i2c2_ck", "axi_sel", 0x0018, 26),
        GATE(CLK_PERI_I2C1, "i2c1_ck", "axi_sel", 0x0018, 25),
        GATE(CLK_PERI_I2C0, "i2c0_ck", "axi_sel", 0x0018, 24),
        GATE(CLK_PERI_BTIF, "bitif_ck", "axi_sel", 0x0018, 23),
        GATE(CLK_PERI_UART3, "uart3_ck", "axi_sel", 0x0018, 22),
        GATE(CLK_PERI_UART2, "uart2_ck", "axi_sel", 0x0018, 21),
        GATE(CLK_PERI_UART1, "uart1_ck", "axi_sel", 0x0018, 20),
        GATE(CLK_PERI_UART0, "uart0_ck", "axi_sel", 0x0018, 19),
        GATE(CLK_PERI_NLI, "nli_ck", "axi_sel", 0x0018, 18),
        GATE(CLK_PERI_MSDC50_3, "msdc50_3_ck", "emmc_hclk_sel", 0x0018, 17),
        GATE(CLK_PERI_MSDC30_3, "msdc30_3_ck", "msdc30_3_sel", 0x0018, 16),
        GATE(CLK_PERI_MSDC30_2, "msdc30_2_ck", "msdc30_2_sel", 0x0018, 15),
        GATE(CLK_PERI_MSDC30_1, "msdc30_1_ck", "msdc30_1_sel", 0x0018, 14),
        GATE(CLK_PERI_MSDC30_0, "msdc30_0_ck", "msdc30_0_sel", 0x0018, 13),
        GATE(CLK_PERI_AP_DMA, "ap_dma_ck", "axi_sel", 0x0018, 12),
        GATE(CLK_PERI_USB1, "usb1_ck", "usb20_sel", 0x0018, 11),
        GATE(CLK_PERI_USB0, "usb0_ck", "usb20_sel", 0x0018, 10),
        GATE(CLK_PERI_PWM, "pwm_ck", "axi_sel", 0x0018, 9),
        /*GATE(CLK_PERI_PWM7, "pwm7_ck", "axisel_d4", 0x0018, 8),
        GATE(CLK_PERI_PWM6, "pwm6_ck", "axisel_d4", 0x0018, 7),
        GATE(CLK_PERI_PWM5, "pwm5_ck", "axisel_d4", 0x0018, 6),
        GATE(CLK_PERI_PWM4, "pwm4_ck", "axisel_d4", 0x0018, 5),
        GATE(CLK_PERI_PWM3, "pwm3_ck", "axisel_d4", 0x0018, 4),
        GATE(CLK_PERI_PWM2, "pwm2_ck", "axisel_d4", 0x0018, 3),
        GATE(CLK_PERI_PWM1, "pwm1_ck", "axisel_d4", 0x0018, 2),*/
        GATE(CLK_PERI_PWM7, "pwm7_ck", "axi_sel", 0x0018, 8),
        GATE(CLK_PERI_PWM6, "pwm6_ck", "axi_sel", 0x0018, 7),
        GATE(CLK_PERI_PWM5, "pwm5_ck", "axi_sel", 0x0018, 6),
        GATE(CLK_PERI_PWM4, "pwm4_ck", "axi_sel", 0x0018, 5),
        GATE(CLK_PERI_PWM3, "pwm3_ck", "axi_sel", 0x0018, 4),
        GATE(CLK_PERI_PWM2, "pwm2_ck", "axi_sel", 0x0018, 3),
        GATE(CLK_PERI_PWM1, "pwm1_ck", "axi_sel", 0x0018, 2),
        GATE(CLK_PERI_THERM, "therm_ck", "axi_sel", 0x0018, 1),
        GATE(CLK_PERI_NFI, "nfi_ck", "nfi2x_sel", 0x0018, 0),

        GATE(CLK_PERI_FCI, "fci_ck", "ms_card_sel", 0x001c, 11),
        GATE(CLK_PERI_SPI2, "spi2_ck", "spi2_sel", 0x001c, 10),
        GATE(CLK_PERI_SPI1, "spi1_ck", "spi1_sel", 0x001c, 9),
        GATE(CLK_PERI_HOST89_DVD, "host89_dvd_ck", "aud2dvd_sel", 0x001c, 8),
        GATE(CLK_PERI_HOST89_SPI, "host89_spi_ck", "spi0_sel", 0x001c, 7),
        GATE(CLK_PERI_HOST89_INT, "host89_int_ck", "axi_sel", 0x001c, 6),
        GATE(CLK_PERI_FLASH, "flash_ck", "nfi2x_sel", 0x001c, 5),
        GATE(CLK_PERI_NFI_PAD, "nfi_pad_ck", "nfi1x_pad", 0x001c, 4),
        GATE(CLK_PERI_NFI_ECC, "nfi_ecc_ck", "nfi1x_pad", 0x001c, 3),
        GATE(CLK_PERI_GCPU, "gcpu_ck", "axi_sel", 0x001c, 2),
        GATE(CLK_PERI_USB_SLV, "usbslv_ck", "axi_sel", 0x001c, 1),
        GATE(CLK_PERI_USB1_MCU, "usb1_mcu_ck", "axi_sel", 0x001c, 0),
};

static struct clk_mux_def muxes_clk[] = {
        MUX0(CLK_PERI_UART0_SEL, "uart0_ck_sel", uart_ck_sel_parents,
             0x40c, 0, 1),
        MUX0(CLK_PERI_UART1_SEL, "uart1_ck_sel", uart_ck_sel_parents,
             0x40c, 1, 1),
        MUX0(CLK_PERI_UART2_SEL, "uart2_ck_sel", uart_ck_sel_parents,
             0x40c, 2, 1),
        MUX0(CLK_PERI_UART3_SEL, "uart3_ck_sel", uart_ck_sel_parents,
             0x40c, 3, 1),
};


static struct mdtk_clk_def clk_def = {
        .gates_def = gates_clk,
        .num_gates = nitems(gates_clk),
        .muxes_def = muxes_clk,
        .num_muxes = nitems(muxes_clk),
};

static int
pericfg_clk_detach(device_t dev) {
    device_printf(dev, "Error: Clock driver cannot be detached\n");
    return (EBUSY);
}

static int
pericfg_clk_probe(device_t dev) {
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
        device_set_desc(dev, "Mediatek pericfg clocks");
        return (BUS_PROBE_DEFAULT);
    }

    return (ENXIO);
}

static int
pericfg_clk_attach(device_t dev) {
    struct mdtk_clk_softc *sc = device_get_softc(dev);
    int rid = 0;

    sc->dev = dev;

    if (ofw_bus_is_compatible(dev, "syscon")) {
        sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                             RF_ACTIVE);
        if (sc->mem_res == NULL) {
            device_printf(dev,
                          "Cannot allocate memory resource\n");
            return (ENXIO);
        }

        mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
        sc->syscon = syscon_create_ofw_node(dev,
                                            &syscon_class, ofw_bus_get_node(dev));
        if (sc->syscon == NULL) {
            device_printf(dev,
                          "Failed to create/register syscon\n");
            return (ENXIO);
        }
    }

    mdtk_register_clocks(dev, &clk_def);
    return (0);
}

static int
pericfg_clk_hwreset_assert(device_t dev, intptr_t idx, bool value) {
    struct mdtk_clk_softc *sc = device_get_softc(dev);
    uint32_t mask, reset_reg;

    CLKDEV_DEVICE_LOCK(sc->dev);
    KASSERT((idx > 0 && idx < 32), ("%s: idx out of range", __func__));


    mask = 1 << (idx % 32);
    reset_reg = (idx / 32) * 4;

    CLKDEV_MODIFY_4(sc->dev, reset_reg, mask, value ? mask : 0);
    CLKDEV_DEVICE_UNLOCK(sc->dev);

    return (0);
}

static int
pericfg_clk_syscon_get_handle(device_t dev, struct syscon **syscon) {
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    *syscon = sc->syscon;
    if (*syscon == NULL) {
        return (ENODEV);
    }

    return (0);
}

static void
pericfg_clk_syscon_lock(device_t dev) {
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_lock(&sc->mtx);
}

static void
pericfg_clk_syscon_unlock(device_t dev) {
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_unlock(&sc->mtx);
}

static device_method_t mt7623_pericfg_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe, pericfg_clk_probe),
        DEVMETHOD(device_attach, pericfg_clk_attach),
        DEVMETHOD(device_detach, pericfg_clk_detach),

        /* Clkdev interface*/
        DEVMETHOD(clkdev_read_4, mdtk_clkdev_read_4),
        DEVMETHOD(clkdev_write_4, mdtk_clkdev_write_4),
        DEVMETHOD(clkdev_modify_4, mdtk_clkdev_modify_4),
        DEVMETHOD(clkdev_device_lock, mdtk_clkdev_device_lock),
        DEVMETHOD(clkdev_device_unlock, mdtk_clkdev_device_unlock),

        DEVMETHOD(hwreset_assert, pericfg_clk_hwreset_assert),

        /* Syscon interface */
        DEVMETHOD(syscon_get_handle, pericfg_clk_syscon_get_handle),
        DEVMETHOD(syscon_device_lock, pericfg_clk_syscon_lock),
        DEVMETHOD(syscon_device_unlock, pericfg_clk_syscon_unlock),

        DEVMETHOD_END
};

DEFINE_CLASS_1(mt7623_pericfg, mt7623_pericfg_driver, mt7623_pericfg_methods,
sizeof(struct mdtk_clk_softc), syscon_class);

EARLY_DRIVER_MODULE(mt7623_pericfg, simplebus, mt7623_pericfg_driver, NULL, NULL,
        BUS_PASS_BUS
+ BUS_PASS_ORDER_MIDDLE + 3);