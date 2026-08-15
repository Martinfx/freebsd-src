/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 * Copyright (c) 2025 Michal Meloun <mmel@FreeBSD.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
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
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>

#include <dt-bindings/clock/mt7622-clk.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <dev/syscon/syscon.h>
#include <dev/hwreset/hwreset.h>

#include "mt_clk.h"

#include "clkdev_if.h"
#include "syscon_if.h"
#include "hwreset_if.h"

static int
init_fixeds(struct mt_clk_softc *sc, struct clk_fixed_def *clks, int nclks)
{
        int i, rv;

        for (i = 0; i < nclks; i++) {
                rv = clknode_fixed_register(sc->clkdom, clks + i);
                if (rv != 0) {
                        device_printf(sc->dev, "clknode_fixed_register failed");
                        return (ENXIO);
                }
        }

        return (0);
}

static int
init_linked(struct mt_clk_softc *sc, struct clk_link_def *clks, int nclks)
{
        for (int i = 0; i < nclks; i++) {
                int rv = clknode_link_register(sc->clkdom, clks + i);
                if (rv != 0) {
                        device_printf(sc->dev, "clknode_link_register failed");
                        return (ENXIO);
                }
        }

        return (0);
}

static int
init_muxes(struct mt_clk_softc *sc, struct clk_mux_def *clks, int nclks)
{
        int i, rv;

        for (i = 0; i < nclks; i++) {
                rv = clknode_mux_register(sc->clkdom, clks + i);
                if (rv != 0) {
                        device_printf(sc->dev, "clknode_mux_register failed");
                        return (ENXIO);
                }
        }

        return (0);
}

static int
init_gates(struct mt_clk_softc *sc, struct clk_gate_def *clks, int nclks)
{
        int i, rv;

        for (i = 0; i < nclks; i++) {
                rv = clknode_gate_register(sc->clkdom, clks + i);
                if (rv != 0) {
                        device_printf(sc->dev, "clknode_gate_register failed");
                        return (ENXIO);
                }
        }

        return (0);
}

static int
init_div(struct mt_clk_softc *sc, struct clk_div_def *clks, int nclks)
{
        int i, rv;

        for (i = 0; i < nclks; i++) {
                rv = clknode_div_register(sc->clkdom, clks + i);
                if (rv != 0) {
                        device_printf(sc->dev, "clknode_div_register failed");
                        return (ENXIO);
                }
        }

        return (0);
}

int
mt_clkdev_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
        struct mt_clk_softc *sc;
        sc = device_get_softc(dev);

        *val = bus_read_4(sc->mem_res, addr);
        return (0);
}

int
mt_clkdev_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
        struct mt_clk_softc *sc;
        sc = device_get_softc(dev);

        bus_write_4(sc->mem_res, addr, val);
        return (0);
}

int
mt_clkdev_modify_4(device_t dev, bus_addr_t addr, uint32_t clear_mask,
                     uint32_t set_mask)
{
        struct mt_clk_softc *sc;
        uint32_t reg;

        sc = device_get_softc(dev);

        reg = bus_read_4(sc->mem_res, addr);
        reg &= ~clear_mask;
        reg |= set_mask;
        bus_write_4(sc->mem_res, addr, reg);
        return (0);
}

void
mt_clkdev_device_lock(device_t dev)
{
        struct mt_clk_softc *sc;
        sc = device_get_softc(dev);
        mtx_lock(&sc->mtx);
}

void
mt_clkdev_device_unlock(device_t dev)
{
        struct mt_clk_softc *sc;
        sc = device_get_softc(dev);
        mtx_unlock(&sc->mtx);
}

int
mt_register_clocks(device_t dev, struct mt_clk_softc *sc,
                     const struct mt_clk_def *cldef)
{
        int rv;

        sc->clkdom = clkdom_create(dev);
        if (sc->clkdom == NULL)
                panic("clkdom == NULL");

        rv = init_fixeds(sc, cldef->fixed_def, cldef->num_fixed);
        if(rv != 0) {
                return (ENXIO);
        }

        rv = init_linked(sc, cldef->linked_def, cldef->num_linked);
        if(rv != 0) {
                return (ENXIO);
        }

        rv = init_muxes(sc, cldef->muxes_def, cldef->num_muxes);
        if(rv != 0) {
                return (ENXIO);
        }

        rv = init_gates(sc, cldef->gates_def, cldef->num_gates);
        if(rv != 0) {
                return (ENXIO);
        }

        rv = init_div(sc, cldef->dived_def, cldef->num_dived);
        if(rv != 0) {
                return (ENXIO);
        }


        rv = clkdom_finit(sc->clkdom);
        if (rv != 0) {
                device_printf(dev, "clkdom_finit failed: %d\n", rv);
                return (ENXIO);
        }
        else if (bootverbose) {
                clkdom_dump(sc->clkdom);
        }

        return (0);
}

int
mt_clk_probe(device_t dev, struct ofw_compat_data *compat, const char *desc)
{
        if (!ofw_bus_status_okay(dev))
                return (ENXIO);
        if (ofw_bus_search_compatible(dev, compat)->ocd_data == 0)
                return (ENXIO);
        device_set_desc(dev, desc);
        return (BUS_PROBE_DEFAULT);
}

int
mt_clk_attach_sc(device_t dev, struct mt_clk_softc *sc)
{
        int rv, rid = 0;

        sc->dev = dev;
        mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

        sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
        if (sc->mem_res == NULL) {
                device_printf(dev, "cannot allocate memory resource\n");
                mtx_destroy(&sc->mtx);
                return (ENXIO);
        }

        /* Register as a syscon provider only when the node asks for it. */
        if (ofw_bus_is_compatible(dev, "syscon")) {
                sc->syscon = syscon_create_ofw_node(dev, &syscon_class,
                    ofw_bus_get_node(dev));
                if (sc->syscon == NULL) {
                        device_printf(dev, "failed to create/register syscon\n");
                        if(sc->mem_res != NULL) {
                                bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
                        }
                        mtx_destroy(&sc->mtx);
                        return (ENXIO);
                }
        }

        rv = mt_register_clocks(dev, sc, sc->clk_def);
        if (rv != 0) {
                device_printf(dev, "failed to register clocks\n");
                if (sc->syscon != NULL) {
                        syscon_destroy(sc->syscon);
                }

                if(sc->mem_res != NULL) {
                        bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem_res);
                }

                mtx_destroy(&sc->mtx);
                return (ENXIO);
        }

        return (0);
}

int
mt_clk_attach(device_t dev)
{
        return (mt_clk_attach_sc(dev, device_get_softc(dev)));
}

static int
mt_clk_detach(device_t dev)
{
        device_printf(dev, "Error: Clock driver cannot be detached\n");
        return (EBUSY);
}

static device_method_t mt_clk_methods[] = {
        DEVMETHOD(device_detach,        mt_clk_detach),

        DEVMETHOD(clkdev_read_4,        mt_clkdev_read_4),
        DEVMETHOD(clkdev_write_4,       mt_clkdev_write_4),
        DEVMETHOD(clkdev_modify_4,      mt_clkdev_modify_4),
        DEVMETHOD(clkdev_device_lock,   mt_clkdev_device_lock),
        DEVMETHOD(clkdev_device_unlock, mt_clkdev_device_unlock),

        DEVMETHOD_END
};

DEFINE_CLASS_0(mt_clk, mt_clk_driver, mt_clk_methods,
sizeof(struct mt_clk_softc));


static int
mt_clk_hwreset_assert(device_t dev, intptr_t id, bool value)
{
        struct mt_clk_reset_softc *sc;
        uint32_t mask;
        uint32_t reg;

        sc = device_get_softc(dev);

        if (id < 0 || id / 32 >= sc->reset_num) {
                return (ENXIO);
        }

        reg = sc->reset_offset[id / 32];
        mask = 1u << (id % 32);

        CLKDEV_DEVICE_LOCK(dev);
        CLKDEV_MODIFY_4(dev, reg, mask, value ? mask : 0);
        CLKDEV_DEVICE_UNLOCK(dev);
        return (0);
}

static device_method_t mt_clk_reset_methods[] = {
    DEVMETHOD(hwreset_assert,       mt_clk_hwreset_assert),

    DEVMETHOD_END
};

DEFINE_CLASS_1(mt_reset_clk, mt_clk_reset_driver, mt_clk_reset_methods,
sizeof(struct mt_clk_reset_softc), mt_clk_driver);
