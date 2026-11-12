/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Martin Filla
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
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <dev/clk/clk_mux.h>
#include <dev/syscon/syscon.h>

#include <dt-bindings/clock/mt8167-clk.h>

#include <arm64/mediatek/mdtk_clk.h>
#include <arm64/mediatek/mt_clk_pll.h>

#include "clkdev_if.h"
#include "syscon_if.h"

/*
 * MT8167 infrastructure configuration (infracfg) clocks: the CPU clock
 * source select and a few peripheral clock multiplexers.
 */

#define	INFRA_CLKSEL		0x000
#define	INFRA_MISC		0x080

PLIST(p_ifr_mux1) = {
	"clk26m_ck", "armpll", "univpll", "mainpll_d2"
};

PLIST(p_ifr_eth_25m) = {
	"eth_d2_ck", "rg_eth"
};

PLIST(p_ifr_i2c0) = {
	"ahb_infra_d2", "rg_i2c"
};

static struct clk_mux_def muxes_clk[] = {
	MUX0(CLK_IFR_MUX1_SEL, "ifr_mux1_sel", p_ifr_mux1, INFRA_CLKSEL, 2, 2),
	MUX0(CLK_IFR_ETH_25M_SEL, "ifr_eth_25m_sel", p_ifr_eth_25m, INFRA_MISC,
	    0, 1),
	MUX0(CLK_IFR_I2C0_SEL, "ifr_i2c0_sel", p_ifr_i2c0, INFRA_MISC, 1, 1),
	MUX0(CLK_IFR_I2C1_SEL, "ifr_i2c1_sel", p_ifr_i2c0, INFRA_MISC, 2, 1),
	MUX0(CLK_IFR_I2C2_SEL, "ifr_i2c2_sel", p_ifr_i2c0, INFRA_MISC, 3, 1),
};

static struct mdtk_clk_def clk_def = {
	.muxes_def = muxes_clk,
	.num_muxes = nitems(muxes_clk),
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt8167-infracfg",	1},
	{NULL,				0},
};

static int
mt8167_infracfg_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek MT8167 infracfg clocks");
	return (BUS_PROBE_DEFAULT);
}

static int
mt8167_infracfg_attach(device_t dev)
{

	return (mdtk_clk_attach(dev, &clk_def));
}

static int
mt8167_infracfg_detach(device_t dev)
{

	device_printf(dev, "clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t mt8167_infracfg_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mt8167_infracfg_probe),
	DEVMETHOD(device_attach,	mt8167_infracfg_attach),
	DEVMETHOD(device_detach,	mt8167_infracfg_detach),

	/* Clkdev interface */
	DEVMETHOD(clkdev_read_4,	mdtk_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	mdtk_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	mdtk_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

	/* Syscon interface */
	DEVMETHOD(syscon_get_handle,	mdtk_clk_syscon_get_handle),
	DEVMETHOD(syscon_device_lock,	mdtk_clk_syscon_lock),
	DEVMETHOD(syscon_device_unlock,	mdtk_clk_syscon_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(mt8167_infracfg, mt8167_infracfg_driver,
    mt8167_infracfg_methods, sizeof(struct mdtk_clk_softc));
EARLY_DRIVER_MODULE(mt8167_infracfg, simplebus, mt8167_infracfg_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 3);
