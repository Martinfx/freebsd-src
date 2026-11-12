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
#include <arm64/mediatek/mt_clk_gate.h>
#include <arm64/mediatek/mt_clk_pll.h>

#include "clkdev_if.h"
#include "syscon_if.h"

/*
 * MT8167 analog PLLs (apmixedsys).
 *
 * armpll, mainpll and univpll feed the CPU and the bus clocks, which have no
 * consumer holding a reference, so they are never switched off.
 */

#define	MT8167_PLL_FMAX		2500000000ULL
#define	MT8167_PLL_RST_BAR	(1u << 27)

/* Register block of each PLL: CON0, CON1, ... */
#define	ARMPLL_BASE		0x0100
#define	MAINPLL_BASE		0x0120
#define	UNIVPLL_BASE		0x0140
#define	MMPLL_BASE		0x0160
#define	APLL1_BASE		0x0180
#define	APLL2_BASE		0x01a0
#define	TVDPLL_BASE		0x01c0
#define	LVDSPLL_BASE		0x01e0

#define	PLL_CON0(_base)		((_base) + 0x00)
#define	PLL_CON1(_base)		((_base) + 0x04)
#define	PLL_PWR_CON(_base)	((_base) + 0x10)

#define	PLL_EN_UNIV		0x30000001	/* univpll also has dividers */
#define	PLL_EN_DEFAULT		0x00000001

/*
 * Integer PLL: 21 (or fewer) bit PCW in CON1, post divider in CON1[26:24].
 */
#define	PLL_INT(_id, _name, _base, _en, _flags, _pcwbits)		\
	MT_PLL(_id, _name, "clk26m", PLL_CON0(_base),			\
	    PLL_PWR_CON(_base), _en, _flags, _pcwbits,			\
	    PLL_CON1(_base), 24, PLL_CON1(_base), 0,			\
	    MT8167_PLL_RST_BAR, MT8167_PLL_FMAX)

/* Audio PLL: 31 bit PCW in CON1, post divider in CON0[3:1]. */
#define	PLL_AUD(_id, _name, _base)					\
	MT_PLL(_id, _name, "clk26m", PLL_CON0(_base),			\
	    PLL_PWR_CON(_base), PLL_EN_DEFAULT, 0, 31,			\
	    PLL_CON0(_base), 1, PLL_CON1(_base), 0,			\
	    MT8167_PLL_RST_BAR, MT8167_PLL_FMAX)

static struct mt_clk_pll_def pll_clk[] = {
	PLL_INT(CLK_APMIXED_ARMPLL, "armpll", ARMPLL_BASE, PLL_EN_DEFAULT,
	    MT_PLL_AO, 21),
	PLL_INT(CLK_APMIXED_MAINPLL, "mainpll", MAINPLL_BASE, PLL_EN_DEFAULT,
	    MT_PLL_HAVE_RST_BAR | MT_PLL_AO, 21),
	PLL_INT(CLK_APMIXED_UNIVPLL, "univpll", UNIVPLL_BASE, PLL_EN_UNIV,
	    MT_PLL_HAVE_RST_BAR | MT_PLL_AO, 7),
	PLL_INT(CLK_APMIXED_MMPLL, "mmpll", MMPLL_BASE, PLL_EN_DEFAULT, 0, 21),
	PLL_AUD(CLK_APMIXED_APLL1, "apll1", APLL1_BASE),
	PLL_AUD(CLK_APMIXED_APLL2, "apll2", APLL2_BASE),
	PLL_INT(CLK_APMIXED_TVDPLL, "tvdpll", TVDPLL_BASE, PLL_EN_DEFAULT, 0,
	    21),
	PLL_INT(CLK_APMIXED_LVDSPLL, "lvdspll", LVDSPLL_BASE, PLL_EN_DEFAULT,
	    0, 21),
};

/* hdmi_ref: read-only power of two divider, TVDPLL CON1[26:24]. */
static struct clk_div_table hdmi_ref_div_table[] = {
	{ .value = 0, .divider = 1 },
	{ .value = 1, .divider = 2 },
	{ .value = 2, .divider = 4 },
	{ .value = 3, .divider = 8 },
	{ .value = 4, .divider = 16 },
	{ .value = 5, .divider = 32 },
	{ .value = 6, .divider = 64 },
	{ .value = 7, .divider = 128 },
	{ .value = 0, .divider = 0 },
};

static struct clk_div_def dived_clk[] = {
	{
		.clkdef.id = CLK_APMIXED_HDMI_REF,
		.clkdef.name = "hdmi_ref",
		.clkdef.parent_names = (const char *[]){"tvdpll"},
		.clkdef.parent_cnt = 1,
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
		.offset = PLL_CON1(TVDPLL_BASE),
		.i_shift = 24,
		.i_width = 3,
		.div_flags = CLK_DIV_WITH_TABLE,
		.div_table = hdmi_ref_div_table,
	},
};

static struct mdtk_clk_def clk_def = {
	.pll_def = pll_clk,
	.num_pll = nitems(pll_clk),
	.dived_def = dived_clk,
	.num_dived = nitems(dived_clk),
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt8167-apmixedsys",	1},
	{NULL,				0},
};

static int
mt8167_apmixedsys_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek MT8167 apmixedsys clocks");
	return (BUS_PROBE_DEFAULT);
}

static int
mt8167_apmixedsys_attach(device_t dev)
{

	return (mdtk_clk_attach(dev, &clk_def));
}

static int
mt8167_apmixedsys_detach(device_t dev)
{

	device_printf(dev, "clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t mt8167_apmixedsys_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mt8167_apmixedsys_probe),
	DEVMETHOD(device_attach,	mt8167_apmixedsys_attach),
	DEVMETHOD(device_detach,	mt8167_apmixedsys_detach),

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

DEFINE_CLASS_0(mt8167_apmixedsys, mt8167_apmixedsys_driver,
    mt8167_apmixedsys_methods, sizeof(struct mdtk_clk_softc));
EARLY_DRIVER_MODULE(mt8167_apmixedsys, simplebus, mt8167_apmixedsys_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);
