/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * MediaTek MT7622 APMIXEDSYS: the PLLs, all fed by the 25 MHz crystal, and
 * the gate of the mainpll clock that the CPUs run from while ARMPLL is
 * reprogrammed.
 *
 * The post divider and the PCW share the register after CON0 on ARMPLL,
 * MAINPLL and UNIV2PLL.  On the other PLLs the post divider is CON0 bits 3:1
 * and the PCW is the register after CON0.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/mediatek/mtk_clk.h>

#include <dt-bindings/clock/mt7622-clk.h>

#define	MT7622_PLL(_id, _name, _con0, _pcwbits, _pd_reg, _pd_shift,	\
    _pcw_reg, _pcw_shift, _flags)					\
	{ .type = MTK_CLK_PLL_MT7622, .pll_mt7622 = {			\
	    .clkdef = { .name = (_name), .id = (_id),			\
		.parent_names = (const char *[]){ "clkxtal" },		\
		.parent_cnt = 1, .flags = (_flags) },			\
	    .con0 = (_con0), .pd_reg = (_pd_reg), .pd_shift = (_pd_shift),\
	    .pcw_reg = (_pcw_reg), .pcw_shift = (_pcw_shift),		\
	    .pcwbits = (_pcwbits) } }

/* Gate bit in a read-write register, set means enabled. */
#define	MT7622_GATE_INV(_id, _name, _parent, _reg, _bit, _flags)	\
	{ .type = MTK_CLK_GATE, .gate = {				\
	    .clkdef = { .name = (_name), .id = (_id),			\
		.parent_names = (const char *[]){ _parent },		\
		.parent_cnt = 1, .flags = (_flags) },			\
	    .reg = (_reg), .bit = (_bit), .flags = MTK_GATE_INVERTED } }

static struct mtk_clk_def mt7622_apmixedsys_clks[] = {
	/* 25 MHz crystal, provided by a fixed-clock node */
	MTK_LINK("clkxtal"),

	/*
	 * MAINPLL and UNIV2PLL feed the bus, DRAM and UART clocks: neither may
	 * ever stop.  ARMPLL clocks the CPUs through a mux in INFRACFG, where
	 * a cpufreq driver moves them to another clock while it changes the
	 * rate.
	 */
	MT7622_PLL(CLK_APMIXED_ARMPLL, "armpll", 0x200, 21,
	    0x204, 24, 0x204, 0, 0),
	MT7622_PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x210, 21,
	    0x214, 24, 0x214, 0, CLK_NODE_CANNOT_STOP),
	MT7622_PLL(CLK_APMIXED_UNIV2PLL, "univ2pll", 0x220, 7,
	    0x224, 24, 0x224, 14, CLK_NODE_CANNOT_STOP),
	MT7622_PLL(CLK_APMIXED_ETH1PLL, "eth1pll", 0x300, 21,
	    0x300, 1, 0x304, 0, 0),
	MT7622_PLL(CLK_APMIXED_ETH2PLL, "eth2pll", 0x314, 21,
	    0x314, 1, 0x318, 0, 0),
	MT7622_PLL(CLK_APMIXED_AUD1PLL, "aud1pll", 0x324, 31,
	    0x324, 1, 0x328, 0, 0),
	MT7622_PLL(CLK_APMIXED_AUD2PLL, "aud2pll", 0x334, 31,
	    0x334, 1, 0x338, 0, 0),
	MT7622_PLL(CLK_APMIXED_TRGPLL, "trgpll", 0x344, 21,
	    0x344, 1, 0x348, 0, 0),
	MT7622_PLL(CLK_APMIXED_SGMIPLL, "sgmipll", 0x358, 21,
	    0x358, 1, 0x35c, 0, 0),

	/* The intermediate CPU clock, critical in Linux. */
	MT7622_GATE_INV(CLK_APMIXED_MAIN_CORE_EN, "main_core_en", "mainpll",
	    0x008, 5, CLK_NODE_CANNOT_STOP),
};

static const struct mtk_clk_desc mt7622_apmixedsys_desc = {
	.clks = mt7622_apmixedsys_clks,
	.nclks = nitems(mt7622_apmixedsys_clks),
};

static struct ofw_compat_data compat_data[] = {
	{ "mediatek,mt7622-apmixedsys",	1 },
	{ NULL,				0 }
};

static int
mt7622_apmixedsys_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "MediaTek MT7622 PLL controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mt7622_apmixedsys_attach(device_t dev)
{
	return (mtk_clk_attach(dev, &mt7622_apmixedsys_desc));
}

static device_method_t mt7622_apmixedsys_methods[] = {
	DEVMETHOD(device_probe,		mt7622_apmixedsys_probe),
	DEVMETHOD(device_attach,	mt7622_apmixedsys_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_apmixedsys, mt7622_apmixedsys_driver,
    mt7622_apmixedsys_methods, sizeof(struct mtk_clk_softc), mtk_clk_driver);

/* After the fixed clocks, so the PLL reference reaches the crystal. */
EARLY_DRIVER_MODULE(mt7622_apmixedsys, simplebus, mt7622_apmixedsys_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);
