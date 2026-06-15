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
#include <dev/clk/clk_gate.h>
#include <dev/hwreset/hwreset.h>

#include <dt-bindings/clock/mt2701-clk.h>
#include "syscon_if.h"
#include "clkdev_if.h"
#include "hwreset_if.h"
#include "mdtk_clk.h"

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7623-bdpsys", 1},
	{"mediatek,mt2701-bdpsys", 1},
	{NULL, 0},
};

static struct clk_gate_def gates_clk[] = {
	GATE(CLK_BDP_BRG_BA, "brg_baclk", "mm_sel", 0x0100, 0),
	GATE(CLK_BDP_BRG_DRAM, "brg_dram", "mm_sel", 0x0100, 1),
	GATE(CLK_BDP_LARB_DRAM, "larb_dram", "mm_sel", 0x0100, 2),
	GATE(CLK_BDP_WR_VDI_PXL, "wr_vdi_pxl", "hdmi_0_deep340m", 0x0100, 3),
	GATE(CLK_BDP_WR_VDI_DRAM, "wr_vdi_dram", "mm_sel", 0x0100, 4),
	GATE(CLK_BDP_WR_B, "wr_bclk", "mm_sel", 0x0100, 5),
	GATE(CLK_BDP_DGI_IN, "dgi_in", "dpi1_sel", 0x0100, 6),
	GATE(CLK_BDP_DGI_OUT, "dgi_out", "dpi1_sel", 0x0100, 7),
	GATE(CLK_BDP_FMT_MAST_27, "fmt_mast_27", "dpi1_sel", 0x0100, 8),
	GATE(CLK_BDP_FMT_B, "fmt_bclk", "mm_sel", 0x0100, 9),
	GATE(CLK_BDP_OSD_B, "osd_bclk", "mm_sel", 0x0100, 10),
	GATE(CLK_BDP_OSD_DRAM, "osd_dram", "mm_sel", 0x0100, 11),
	GATE(CLK_BDP_OSD_AGENT, "osd_agent", "osd_sel", 0x0100, 12),
	GATE(CLK_BDP_OSD_PXL, "osd_pxl", "dpi1_sel", 0x0100, 13),
	GATE(CLK_BDP_RLE_B, "rle_bclk", "mm_sel", 0x0100, 14),
	GATE(CLK_BDP_RLE_AGENT, "rle_agent", "mm_sel", 0x0100,  15),
	GATE(CLK_BDP_RLE_DRAM, "rle_dram", "mm_sel", 0x0100, 16),
	GATE(CLK_BDP_F27M, "f27m", "di_sel", 0x0100, 17),
	GATE(CLK_BDP_F27M_VDOUT, "f27m_vdout", "di_sel", 0x0100, 18),
	GATE(CLK_BDP_F27_74_74, "f27_74_74", "di_sel", 0x0100, 19),
	GATE(CLK_BDP_F2FS, "f2fs", "di_sel", 0x0100, 20),
	GATE(CLK_BDP_F2FS, "f2fs", "di_sel", 0x0100, 20),
	GATE(CLK_BDP_F2FS74_148, "f2fs74_148", "di_sel", 0x0100, 21),
	GATE(CLK_BDP_FB, "fbclk", "mm_sel", 0x0100, 22),
	GATE(CLK_BDP_VDO_DRAM, "vdo_dram", "mm_sel", 0x0100, 23),
	GATE(CLK_BDP_VDO_2FS, "vdo_2fs", "di_sel", 0x0100, 24),
	GATE(CLK_BDP_VDO_B, "vdo_bclk", "mm_sel", 0x0100, 25),
	GATE(CLK_BDP_WR_DI_PXL, "wr_di_pxl", "di_sel", 0x0100, 26),
	GATE(CLK_BDP_WR_DI_DRAM, "wr_di_dram", "mm_sel", 0x0100, 27),
	GATE(CLK_BDP_WR_DI_B, "wr_di_bclk", "mm_sel", 0x0100, 28),
	GATE(CLK_BDP_NR_PXL, "nr_pxl", "nr_sel", 0x0100, 29),
	GATE(CLK_BDP_NR_DRAM, "nr_dram", "mm_sel", 0x0100, 30),
	GATE(CLK_BDP_NR_B, "nr_bclk", "mm_sel", 0x0100, 31),

	GATE(CLK_BDP_RX_F, "rx_fclk", "hadds2_fbclk", 0x0110, 0),
	GATE(CLK_BDP_RX_X, "rx_xclk", "clk26m", 0x0110, 1),
	GATE(CLK_BDP_RXPDT, "rxpdtclk", "hdmi_0_pix340m", 0x0110, 2),
	GATE(CLK_BDP_RX_CSCL_N, "rx_cscl_n", "clk26m", 0x0110, 3),
	GATE(CLK_BDP_RX_CSCL, "rx_cscl", "clk26m", 0x0110, 4),
	GATE(CLK_BDP_RX_DDCSCL_N, "rx_ddcscl_n", "hdmi_scl_rx", 0x0110, 5),
	GATE(CLK_BDP_RX_DDCSCL, "rx_ddcscl", "hdmi_scl_rx", 0x0110, 6),
	GATE(CLK_BDP_RX_VCO, "rx_vcoclk", "hadds2pll_294m", 0x0110, 7),
	GATE(CLK_BDP_RX_DP, "rx_dpclk", "hdmi_0_pll340m", 0x0110, 8),
	GATE(CLK_BDP_RX_P, "rx_pclk", "hdmi_0_pll340m", 0x0110, 9),
	GATE(CLK_BDP_RX_M, "rx_mclk", "hadds2pll_294m", 0x0110, 10),
	GATE(CLK_BDP_RX_PLL, "rx_pllclk", "hdmi_0_pix340m", 0x0110, 11),
	GATE(CLK_BDP_BRG_RT_B, "brg_rt_bclk", "mm_sel", 0x0110, 12),
	GATE(CLK_BDP_BRG_RT_DRAM, "brg_rt_dram", "mm_sel", 0x0110, 13),
	GATE(CLK_BDP_LARBRT_DRAM, "larbrt_dram", "mm_sel", 0x0110, 14),
	GATE(CLK_BDP_TMDS_SYN, "tmds_syn", "hdmi_0_pll340m", 0x0110, 15),
	GATE(CLK_BDP_HDMI_MON, "hdmi_mon", "hdmi_0_pll340m", 0x0110, 16),
};

static struct mdtk_clk_def clk_def = {
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
};

static int
bdpsys_clk_detach(device_t dev)
{
	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static int
bdpsys_clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Mediatek bdpsys clocks");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
bdpsys_clk_attach(device_t dev) {
	struct mdtk_clk_softc *sc = device_get_softc(dev);
	int rid = 0;

	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (ofw_bus_is_compatible(dev, "syscon")) {
		sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
		    RF_ACTIVE);
		if (sc->mem_res == NULL) {
			device_printf(dev,
			    "Cannot allocate memory resource\n");
			return (ENXIO);
		}


	}

	mdtk_register_clocks(dev, &clk_def);
	return (0);
}

static int
bdpsys_clk_hwreset_assert(device_t dev, intptr_t idx, bool value)
{
	struct mdtk_clk_softc *sc = device_get_softc(dev);
	uint32_t mask, reset_reg;

	CLKDEV_DEVICE_LOCK(sc->dev);
	KASSERT((idx > 0 && idx < 32), ("%s: idx out of range",__func__));


	mask = 1 << (idx % 32);
	reset_reg = (idx / 32) * 4;

	CLKDEV_MODIFY_4(sc->dev, reset_reg, mask, value ? mask : 0);
	CLKDEV_DEVICE_UNLOCK(sc->dev);

	return(0);
}

static int
bdpsys_clk_syscon_get_handle(device_t dev, struct syscon **syscon)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	*syscon = sc->syscon;
	if (*syscon == NULL) {
		return (ENODEV);
	}

	return (0);
}

static void
bdpsys_clk_syscon_lock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
bdpsys_clk_syscon_unlock(device_t dev)
{
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t mt7622_bdpsys_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		 bdpsys_clk_probe),
	DEVMETHOD(device_attach,	 bdpsys_clk_attach),
	DEVMETHOD(device_detach, 	 bdpsys_clk_detach),

	/* Clkdev interface*/
	DEVMETHOD(clkdev_read_4,        mdtk_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	    mdtk_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	    mdtk_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

	DEVMETHOD(hwreset_assert,	bdpsys_clk_hwreset_assert),

	/* Syscon interface */
	DEVMETHOD(syscon_get_handle,    bdpsys_clk_syscon_get_handle),
	DEVMETHOD(syscon_device_lock,   bdpsys_clk_syscon_lock),
	DEVMETHOD(syscon_device_unlock, bdpsys_clk_syscon_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_bdpsys, mt7622_bdpsys_driver, mt7622_bdpsys_methods,
    sizeof(struct mdtk_clk_softc), syscon_class);

EARLY_DRIVER_MODULE(mt7622_bdpsys, simplebus, mt7622_bdpsys_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 4);