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
#include <sys/libkern.h>
#include <sys/kernel.h>
#include <sys/cdefs.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/simplebus.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>
#include <dev/syscon/syscon.h>
#include <dt-bindings/pinctrl/mt7623-pinfunc.h>
#include "syscon_if.h"

/* GPIO_MODE registers: five 3-bit fields per 32-bit register, 0x10 apart. */
#define    MT7623_GPIO_MODE_BASE    0x760
#define    MT7623_GPIO_MODE_STRIDE    0x10
#define    MT7623_PINS_PER_REG    5
#define    MT7623_PIN_MODE_BITS    3
#define    MT7623_PIN_MODE_MASK    0x7
#define    MT7623_MAX_PIN        278

#define    MT7623_PINCTRL_LOCK(sc)        mtx_lock_spin(&(sc)->mtx)
#define    MT7623_PINCTRL_UNLOCK(sc)    mtx_unlock_spin(&(sc)->mtx)

static struct ofw_compat_data compat_data[] =
        {{"mediatek,mt7623-pinctrl", 1},
         {NULL,                      0}};


struct mt7623_pinctrl_softc {
    device_t dev;
    struct resource *mem_res;
    struct syscon *pctl_syscon;
    struct mtx mtx;
    int mem_rid;
};

/*
 * Set the mux field of a single pin.  pin and func come straight from a
 * "pinmux" cell; the register offset and bit position are derived from the
 * pin number.
 */
static void
mt7623_pinctrl_set_mux(struct mt7623_pinctrl_softc *sc, uint32_t pin,
                       uint32_t func) {
    bus_size_t reg;
    int shift;

    if (pin > MT7623_MAX_PIN) {
        device_printf(sc->dev, "pinmux: pin %u out of range\n", pin);
        return;
    }
    if (func > MT7623_PIN_MODE_MASK) {
        device_printf(sc->dev, "pinmux: pin %u: invalid function %u\n",
                      pin, func);
        return;
    }

    reg = MT7623_GPIO_MODE_BASE +
          (pin / MT7623_PINS_PER_REG) * MT7623_GPIO_MODE_STRIDE;
    shift = (pin % MT7623_PINS_PER_REG) * MT7623_PIN_MODE_BITS;

    MT7623_PINCTRL_LOCK(sc);
    SYSCON_MODIFY_4(sc->pctl_syscon, reg,
                    MT7623_PIN_MODE_MASK << shift, func << shift);
    MT7623_PINCTRL_UNLOCK(sc);

    if (bootverbose) {
        uint32_t v = SYSCON_READ_4(sc->pctl_syscon, reg);
        uint32_t got = (v >> shift) & MT7623_PIN_MODE_MASK;
        if (got != func) {
            device_printf(sc->dev, "pin %u: FAIL want %u got %u (reg %#jx=0x%08x)\n",
                          pin, func, got, (uintmax_t) reg, v);
        }

        device_printf(sc->dev, "pin %u -> func %u (reg %#jx[%d])\n",
                      pin, func, (uintmax_t) reg, shift);
    }
}

/*
 * Apply one pin-config sub-node (e.g. "pins-i2c0").  Each such node carries
 * a "pinmux" array; every cell encodes (pin << 8) | function.
 */
static int
mt7623_pinctrl_process_node(struct mt7623_pinctrl_softc *sc, phandle_t node) {
    uint32_t *cells;
    uint32_t pin, func;
    int i, ncells;

    /* Allocation must stay outside the spin lock. */
    ncells = OF_getencprop_alloc_multi(node, "pinmux", sizeof(*cells),
                                       (void **) &cells);
    if (ncells <= 0)
        return (ENOENT);

    for (i = 0; i < ncells; i++) {
        pin = MTK_GET_PIN_NO(cells[i]);
        func = MTK_GET_PIN_FUNC(cells[i]);
        mt7623_pinctrl_set_mux(sc, pin, func);
    }
    OF_prop_free(cells);

    /*
     * TODO: bias-disable / bias-pull-up / bias-pull-down and
     * drive-strength (the GENERIC_PINCONFIG properties) also apply to the
     * pins listed above; they live in other registers of the same syscon.
     */
    return (0);
}

static int
mt7623_pinctrl_configure(device_t dev, phandle_t cfgxref) {
    struct mt7623_pinctrl_softc *sc;
    phandle_t child, node;
    sc = device_get_softc(dev);
    node = OF_node_from_xref(cfgxref);

    for (child = OF_child(node); child != 0 && child != -1; child = OF_peer(child)) {
        mt7623_pinctrl_process_node(sc, child);
    }

    return (0);
}

static int
mt7623_pinctrl_probe(device_t dev) {
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "Mediatek 7623 pinctrl");
    return (BUS_PROBE_DEFAULT);
}

static int
mt7623_pinctrl_attach(device_t dev) {
    struct mt7623_pinctrl_softc *sc;
    phandle_t node;
    int error;

    sc = device_get_softc(dev);
    sc->dev = dev;
    node = ofw_bus_get_node(dev);

    /* Map memory resource */
    sc->mem_rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Could not map memory resource\n");
        return (ENXIO);
    }

    error = syscon_get_by_ofw_property(dev, node,
                                       "mediatek,pctl-regmap", &sc->pctl_syscon);

    if (error != 0) {
        device_printf(dev, "cannot get mediatek,pctl-regmap syscon\n");
        return (error);
    }

    mtx_init(&sc->mtx, "mt pinctrl", "pinctrl", MTX_SPIN);

    fdt_pinctrl_register(dev, NULL);
    fdt_pinctrl_configure_tree(dev);

    return (0);
}

static int
mt7623_pinctrl_detach(device_t dev) {
    struct mt7623_pinctrl_softc *sc;
    sc = device_get_softc(dev);

    if (sc->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
    }

    return (0);
}

static device_method_t mt7623_pinctrl_methods[] = {
        DEVMETHOD(device_probe, mt7623_pinctrl_probe),
        DEVMETHOD(device_attach, mt7623_pinctrl_attach),
        DEVMETHOD(device_detach, mt7623_pinctrl_detach),
        DEVMETHOD(fdt_pinctrl_configure, mt7623_pinctrl_configure),
        DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7623_pinctrl, mt7623_pinctrl_driver, mt7623_pinctrl_methods,
sizeof(struct mt7623_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7623_pinctrl, simplebus, mt7623_pinctrl_driver,
        NULL, NULL,
71);
