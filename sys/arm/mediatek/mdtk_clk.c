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

#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <arm/mediatek/mdtk_clk.h>
#include "clkdev_if.h"
#include "mt_clk_pll.h"

/*
 * Register every clock of one kind.  A clock the hardware description got
 * wrong must not take the machine down with it, so failures are reported to
 * the caller, which gives up on the whole controller.
 */
static int
init_pll(struct mdtk_clk_softc *sc, struct clk_pll_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = mt_clk_pll_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register pll %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
}

static int
init_fixeds(struct mdtk_clk_softc *sc, struct clk_fixed_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = clknode_fixed_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register clock %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
}

static int
init_linked(struct mdtk_clk_softc *sc, struct clk_link_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = clknode_link_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register link %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
}

static int
init_muxes(struct mdtk_clk_softc *sc, struct clk_mux_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = clknode_mux_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register mux %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
}

static int
init_gates(struct mdtk_clk_softc *sc, struct clk_gate_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = clknode_gate_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register gate %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
}

static int
init_div(struct mdtk_clk_softc *sc, struct clk_div_def *clks, int nclks)
{
	int i, rv;

	for (i = 0; i < nclks; i++) {
		rv = clknode_div_register(sc->clkdom, clks + i);
		if (rv != 0) {
			device_printf(sc->dev, "cannot register divider %s\n",
			    clks[i].clkdef.name);
			return (rv);
		}
	}
	return (0);
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

/*
 * Assert or deassert one reset line of a clock controller.  The reset bits
 * live in one or more consecutive 32 bit banks starting at "base"; which
 * bank a line belongs to follows from its index, exactly as the reset ids in
 * dt-bindings/reset/mt2701-resets.h are numbered.
 */
int
mdtk_clk_hwreset_assert(device_t dev, bus_size_t base, int nbanks,
						intptr_t idx, bool value)
{
	struct mdtk_clk_softc *sc;
	bus_size_t reg;
	uint32_t mask;

	if (idx < 0 || idx >= (intptr_t)nbanks * 32)
		return (EINVAL);

	sc = device_get_softc(dev);
	mask = 1u << (idx % 32);
	reg = base + (idx / 32) * 4;

	CLKDEV_DEVICE_LOCK(sc->dev);
	CLKDEV_MODIFY_4(sc->dev, reg, mask, value ? mask : 0);
	CLKDEV_DEVICE_UNLOCK(sc->dev);

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

int
mdtk_register_clocks(device_t dev, struct mdtk_clk_def *cldef)
{
	struct mdtk_clk_softc *sc;
	int rv;

	sc = device_get_softc(dev);
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "cannot create clock domain\n");
		return (ENXIO);
	}

	rv = init_pll(sc, cldef->pll_def, cldef->num_pll);
	if (rv == 0)
		rv = init_fixeds(sc, cldef->fixed_def, cldef->num_fixed);
	if (rv == 0)
		rv = init_linked(sc, cldef->linked_def, cldef->num_linked);
	if (rv == 0)
		rv = init_muxes(sc, cldef->muxes_def, cldef->num_muxes);
	if (rv == 0)
		rv = init_gates(sc, cldef->gates_def, cldef->num_gates);
	if (rv == 0)
		rv = init_div(sc, cldef->dived_def, cldef->num_dived);
	/*
	 * There is no way to tear a half built domain down again, so on
	 * failure it is simply left behind and the controller does not
	 * attach.
	 */
	if (rv != 0)
		return (rv);

	rv = clkdom_finit(sc->clkdom);
	if (rv != 0) {
		device_printf(dev, "cannot finalize clock domain\n");
		return (rv);
	}
	if (bootverbose)
		clkdom_dump(sc->clkdom);

	return (0);
}
