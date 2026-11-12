/*-
 * Copyright (c) 2025 Martin Filla, Michal Meloun <mmel@FreeBSD.org>
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

#include <dt-bindings/clock/mt7622-clk.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <dev/syscon/syscon.h>

#include "mdtk_clk.h"
#include "mt_clk_pll.h"
#include "mt_clk_cpumux.h"
#include "mt_clk_gate.h"
#include "syscon_if.h"

static void
init_pll(struct mdtk_clk_softc *sc, struct mt_clk_pll_def *clks,
            int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = mt_clk_pll_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("mt_clk_pll_register failed");
    }

}

static void
init_cpumuxes(struct mdtk_clk_softc *sc, struct mt_clk_cpumux_def *clks,
            int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = mt_clk_cpumux_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("mt_clk_cpumux_register failed");
    }
}

static void
init_fixeds(struct mdtk_clk_softc *sc, struct clk_fixed_def *clks,
            int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = clknode_fixed_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("clknode_fixed_register failed");
    }

}

static void
init_linked(struct mdtk_clk_softc *sc, struct clk_link_def *clks,
            int nclks)
{
    for (int i = 0; i < nclks; i++) {
        int rv = clknode_link_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("clknode_link_register failed");
    }

}

static void
init_muxes(struct mdtk_clk_softc *sc, struct clk_mux_def *clks, int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = clknode_mux_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("clknode_mux_register failed");
    }
}

static void
init_gates(struct mdtk_clk_softc *sc, struct clk_gate_def *clks, int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = clknode_gate_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("clknode_gate_register failed");
    }
}

static void
init_div(struct mdtk_clk_softc *sc, struct clk_div_def *clks, int nclks)
{
    int i, rv;

    for (i = 0; i < nclks; i++) {
        rv = clknode_div_register(sc->clkdom, clks + i);
        if (rv != 0)
            panic("clknode_div_register failed");
    }
}

static void
init_mtgates(struct mdtk_clk_softc *sc, struct mt_clk_gate_def *clks,
    int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = mt_clk_gate_register(sc->clkdom, clks + i);
		if (rv != 0)
			panic("mt_clk_gate_register failed");
	}
}

int
mdtk_clkdev_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    *val = bus_read_4(sc->mem_res, addr);
    return (0);
}

int
mdtk_clkdev_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    bus_write_4(sc->mem_res, addr, val);
    return (0);
}

int
mdtk_clkdev_modify_4(device_t dev, bus_addr_t addr, uint32_t clear_mask,
                     uint32_t set_mask)
{
    struct mdtk_clk_softc *sc;
    uint32_t reg;

    sc = device_get_softc(dev);
    reg = bus_read_4(sc->mem_res, addr);
    reg &= ~clear_mask;
    reg |= set_mask;
    bus_write_4(sc->mem_res, addr, reg);
    return (0);
}

void
mdtk_clkdev_device_lock(device_t dev)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_lock(&sc->mtx);
}

void
mdtk_clkdev_device_unlock(device_t dev)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_unlock(&sc->mtx);
}

void
mdtk_register_clocks(device_t dev, struct mdtk_clk_def *cldef)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    sc->clkdom = clkdom_create(dev);
    if (sc->clkdom == NULL)
        panic("clkdom == NULL");

    init_pll(sc, cldef->pll_def, cldef->num_pll);
    init_cpumuxes(sc, cldef->cpumux_def, cldef->num_cpumux);
    init_fixeds(sc, cldef->fixed_def, cldef->num_fixed);
    init_linked(sc, cldef->linked_def, cldef->num_linked);
    init_muxes(sc, cldef->muxes_def, cldef->num_muxes);
    init_gates(sc, cldef->gates_def, cldef->num_gates);
    init_div(sc, cldef->dived_def, cldef->num_dived);
    init_mtgates(sc, cldef->mtgates_def, cldef->num_mtgates);

    clkdom_finit(sc->clkdom);
    if (bootverbose)
        clkdom_dump(sc->clkdom);
}

/*
 * Syscon backed by the clock controller registers.  The MediaTek clock
 * controller nodes are also "syscon" compatible and other drivers reach
 * their registers through it.
 */
static uint32_t
mdtk_syscon_unlocked_read_4(struct syscon *syscon, bus_size_t offset)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(syscon->pdev);
	return (bus_read_4(sc->mem_res, offset));
}

static int
mdtk_syscon_unlocked_write_4(struct syscon *syscon, bus_size_t offset,
    uint32_t val)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(syscon->pdev);
	bus_write_4(sc->mem_res, offset, val);
	return (0);
}

static int
mdtk_syscon_unlocked_modify_4(struct syscon *syscon, bus_size_t offset,
    uint32_t clear_bits, uint32_t set_bits)
{
	struct mdtk_clk_softc *sc;
	uint32_t val;

	sc = device_get_softc(syscon->pdev);
	val = bus_read_4(sc->mem_res, offset);
	val &= ~clear_bits;
	val |= set_bits;
	bus_write_4(sc->mem_res, offset, val);
	return (0);
}

static syscon_method_t mdtk_syscon_methods[] = {
	SYSCONMETHOD(syscon_unlocked_read_4,	mdtk_syscon_unlocked_read_4),
	SYSCONMETHOD(syscon_unlocked_write_4,	mdtk_syscon_unlocked_write_4),
	SYSCONMETHOD(syscon_unlocked_modify_4,	mdtk_syscon_unlocked_modify_4),
	SYSCONMETHOD_END
};
DEFINE_CLASS_1(mdtk_syscon, mdtk_syscon_class, mdtk_syscon_methods, 0,
    syscon_class);

int
mdtk_clk_syscon_get_handle(device_t dev, struct syscon **syscon)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->syscon == NULL)
		return (ENODEV);
	*syscon = sc->syscon;
	return (0);
}

void
mdtk_clk_syscon_lock(device_t dev)
{

	mdtk_clkdev_device_lock(dev);
}

void
mdtk_clk_syscon_unlock(device_t dev)
{

	mdtk_clkdev_device_unlock(dev);
}

/*
 * Common attach: map the registers, export them as a syscon when the node
 * asks for it and register the clocks.
 */
int
mdtk_clk_attach(device_t dev, struct mdtk_clk_def *cldef)
{
	struct mdtk_clk_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	if (ofw_bus_is_compatible(dev, "syscon")) {
		sc->syscon = syscon_create_ofw_node(dev, &mdtk_syscon_class,
		    ofw_bus_get_node(dev));
		if (sc->syscon == NULL) {
			device_printf(dev, "cannot register syscon\n");
			bus_release_resource(dev, SYS_RES_MEMORY, rid,
			    sc->mem_res);
			mtx_destroy(&sc->mtx);
			return (ENXIO);
		}
	}

	mdtk_register_clocks(dev, cldef);
	return (0);
}

int
mdtk_hwreset_by_idx(struct mdtk_clk_softc *sc, intptr_t idx, bool reset)
{
    device_printf(sc->dev, "idx %ld, reset %d\n", idx, reset);
    return(0);
}
