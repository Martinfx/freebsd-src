/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
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

#include "mt_clk.h"
#include "mt_clk_pll.h"

#define	MT7622_PLL_FMAX		2500000000ULL
#define	CON0_MT7622_RST_BAR	(1u << 27)

#define	PLL(_id, _name, _reg, _pwr_reg, _en_mask, _flags, _pcwbits,	\
    _pd_reg, _pd_shift, _pcw_reg, _pcw_shift)				\
	MT_PLL(_id, _name, "clkxtal", _reg, _pwr_reg, _en_mask, _flags,	\
	    _pcwbits, _pd_reg, _pd_shift, _pcw_reg, _pcw_shift,		\
	    CON0_MT7622_RST_BAR, MT7622_PLL_FMAX)

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7622-apmixedsys",	1},
	{NULL,				0},
};

/* Register layout from Linux clk-mt7622-apmixedsys.c */
static struct mt_clk_pll_def plls_clk[] = {
	PLL(CLK_APMIXED_ARMPLL, "armpll", 0x0200, 0x020C, 0,
	    MT_PLL_AO, 21, 0x0204, 24, 0x0204, 0),
	PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x0210, 0x021C, 0,
	    MT_PLL_HAVE_RST_BAR | MT_PLL_CRITICAL, 21, 0x0214, 24, 0x0214, 0),
	PLL(CLK_APMIXED_UNIV2PLL, "univ2pll", 0x0220, 0x022C, 0,
	    MT_PLL_HAVE_RST_BAR | MT_PLL_CRITICAL, 7, 0x0224, 24, 0x0224, 14),
	PLL(CLK_APMIXED_ETH1PLL, "eth1pll", 0x0300, 0x0310, 0,
	    0, 21, 0x0300, 1, 0x0304, 0),
	PLL(CLK_APMIXED_ETH2PLL, "eth2pll", 0x0314, 0x0320, 0,
	    0, 21, 0x0314, 1, 0x0318, 0),
	PLL(CLK_APMIXED_AUD1PLL, "aud1pll", 0x0324, 0x0330, 0,
	    0, 31, 0x0324, 1, 0x0328, 0),
	PLL(CLK_APMIXED_AUD2PLL, "aud2pll", 0x0334, 0x0340, 0,
	    0, 31, 0x0334, 1, 0x0338, 0),
	PLL(CLK_APMIXED_TRGPLL, "trgpll", 0x0344, 0x0354, 0,
	    0, 21, 0x0344, 1, 0x0348, 0),
	PLL(CLK_APMIXED_SGMIPLL, "sgmipll", 0x0358, 0x0368, 0,
	    0, 21, 0x0358, 1, 0x035C, 0),
};

static struct clk_gate_def gates_clk[] = {
	GATE(CLK_APMIXED_MAIN_CORE_EN, "main_core_en", "mainpll", 0x0008, 5),
};

static struct mt_clk_def clk_def = {
	.pll_def = plls_clk,
	.num_pll = nitems(plls_clk),
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
};

static int
apmixedsys_clk_probe(device_t dev)
{

	return (mt_clk_probe(dev, compat_data,
	    "Mediatek mt7622 apmixedsys clocks"));
}

static int
apmixedsys_clk_attach(device_t dev)
{
	struct mt_clk_softc *sc;

	sc = device_get_softc(dev);
	sc->clk_def = &clk_def;

	return (mt_clk_attach(dev));
}

static device_method_t mt7622_apmixedsys_methods[] = {
	DEVMETHOD(device_probe,		apmixedsys_clk_probe),
	DEVMETHOD(device_attach,	apmixedsys_clk_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_apmixedsys, mt7622_apmixedsys_driver,
    mt7622_apmixedsys_methods, sizeof(struct mt_clk_softc), mt_clk_driver);

/* PLLs must be registered before topckgen, which derives its clocks from them. */
EARLY_DRIVER_MODULE(mt7622_apmixedsys, simplebus, mt7622_apmixedsys_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);
