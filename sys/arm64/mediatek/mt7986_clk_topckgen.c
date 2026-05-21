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

#include <dt-bindings/clock/mt7986-clk.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <arm64/mediatek/mdtk_clk.h>
#include <dev/hwreset/hwreset.h>
#include "clkdev_if.h"

#define CLK_CFG_0 0x000
#define CLK_CFG_1 0x010
#define CLK_CFG_2 0x020
#define CLK_CFG_3 0x030
#define CLK_CFG_4 0x040
#define CLK_CFG_5 0x050
#define CLK_CFG_6 0x060
#define CLK_CFG_7 0x070
#define CLK_CFG_8 0x080
#define CLK_CFG_9 0x090

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7986-topckgen",	1},
	{NULL,		 	0},
};

PLIST(nfi1x_parents) = {
	"top_xtal",
	"top_mmpll_d8",
	"top_net1pll_d8_d2",
	"top_net2pll_d3_d2",
	"top_mpll_d4",
	"top_mmpll_d8_d2",
	"top_wedmcupll_d5_d2",
	"top_mpll_d8"
};

PLIST(spinfi_parents) = {
	"top_xtal_d2",
	"top_xtal",
	"top_net1pll_d5_d4",
	"top_mpll_d4",
	"top_mmpll_d8_d2",
	"top_wedmcupll_d5_d2",
	"top_mmpll_d3_d8",
	"top_mpll_d8"
};

PLIST(spi_parents) = {
	"top_xtal",
	"top_mpll_d2",
	"top_mmpll_d8",
	"top_net1pll_d8_d2",
	"top_net2pll_d3_d2",
	"top_net1pll_d5_d4",
	"top_mpll_d4",
	"top_wedmcupll_d5_d2"
};

PLIST(uart_parents) = {
	"top_xtal",
	"top_mpll_d8",
	"top_mpll_d8_d2"
};

PLIST(pwm_parents) = {
	"top_xtal",
	"top_net1pll_d8_d2",
	"top_net1pll_d5_d4",
	"top_mpll_d4"
};

PLIST(i2c_parents) = {
	"top_xtal",
	"top_net1pll_d5_d4",
	"top_mpll_d4",
	"top_net1pll_d8_d4"
};

PLIST(pextp_tl_ck_parents) = {
	"top_xtal",
	"top_net1pll_d5_d4",
	"top_net2pll_d4_d2",
	"top_rtc_32k"
};

PLIST(emmc_250m_parents) = {
	"top_xtal",
	"top_net1pll_d5_d2"
};

PLIST(emmc_416m_parents) = {
	"top_xtal",
	"mpll" };

PLIST(f_26m_adc_parents) = {
	"top_xtal",
	"top_mpll_d8_d2" };

PLIST(dramc_md32_parents) = {
	"top_xtal",
	"top_mpll_d2" };

PLIST(sysaxi_parents) = {
	"top_xtal",
	"top_net1pll_d8_d2",
	"top_net2pll_d4" };

PLIST(sysapb_parents) = {
	"top_xtal",
	"top_mpll_d3_d2",
	"top_net2pll_d4_d2" };

PLIST(arm_db_main_parents) = {
	"top_xtal",
	"top_net2pll_d3_d2"
};

PLIST(arm_db_jtsel_parents) = {
	"top_jtag",
	"top_xtal" };

PLIST(netsys_parents) = {
	"top_xtal",
	"top_mmpll_d4" };

PLIST(netsys_500m_parents) = {
	"top_xtal",
	"top_net1pll_d5"
};

PLIST(netsys_mcu_parents) = {
	"top_xtal",
	"wedmcupll",
	"top_mmpll_d2",
	"top_net1pll_d4",
	"top_net1pll_d5"
};

PLIST(netsys_2x_parents) = {
	"top_xtal",
	"net2pll",
	"wedmcupll",
	"top_mmpll_d2"
};

PLIST(sgm_325m_parents) = {
	"top_xtal",
	"sgmpll" };

PLIST(sgm_reg_parents) = {
	"top_xtal",
	"top_net1pll_d8_d4"
};

PLIST(a1sys_parents) = {
	"top_xtal",
	"top_apll2_d4"
};

PLIST(conn_mcusys_parents) = {
	"top_xtal",
	"top_mmpll_d2"
};

PLIST(eip_b_parents) = {
	"top_xtal",
	"net2pll"
};

PLIST(aud_l_parents) = {
	"top_xtal",
	"apll2",
	"top_mpll_d8_d2"
};

PLIST(a_tuner_parents) = {
	"top_xtal",
	"top_apll2_d4",
	"top_mpll_d8_d2"
};

PLIST(u2u3_sys_parents) = {
	"top_xtal",
	"top_net1pll_d5_d4"
};

PLIST(da_u2_refsel_parents) = {
	"top_xtal",
	"top_mmpll_u2phy"
};

static struct clk_fixed_def fixed_clk[] = {
	FRATE(CLK_TOP_XTAL, "top_xtal", 40000000),
	FRATE(CLK_TOP_JTAG, "top_jtag", 50000000),

	FFACT(CLK_TOP_XTAL_D2, "top_xtal_d2", "top_xtal", 1, 2),
	FFACT(CLK_TOP_RTC_32K, "top_rtc_32k", "top_xtal", 1, 1250),
	FFACT(CLK_TOP_RTC_32P7K, "top_rtc_32p7k", "top_xtal", 1, 1220),

	/* we have not pll clock infrastructure */
	FRATE(CLK_APMIXED_ARMPLL,   "armpll",    120000000ULL),
	FRATE(CLK_APMIXED_NET2PLL,  "net2pll",  1120000000ULL),
	FRATE(CLK_APMIXED_MMPLL, "mmpll", 2400000000ULL),
	FRATE(CLK_APMIXED_SGMPLL,  "sgmpll",   500000000ULL),
	FRATE(CLK_APMIXED_WEDMCUPLL,  "wedmcupll",   650000000ULL),
	FRATE(CLK_APMIXED_NET1PLL,  "net1pll",   147456000ULL),
	FRATE(CLK_APMIXED_MPLL,  "mpll",   135475000ULL),
	FRATE(CLK_APMIXED_APLL2,  "apll2",   135475000ULL),

	/* MPLL */
	FFACT(CLK_TOP_MPLL_D2, "top_mpll_d2", "mpll", 1, 2),
	FFACT(CLK_TOP_MPLL_D4, "top_mpll_d4", "mpll", 1, 4),
	FFACT(CLK_TOP_MPLL_D8, "top_mpll_d8", "mpll", 1, 8),
	FFACT(CLK_TOP_MPLL_D8_D2, "top_mpll_d8_d2", "mpll", 1, 16),
	FFACT(CLK_TOP_MPLL_D3_D2, "top_mpll_d3_d2", "mpll", 1, 6),
	/* MMPLL */
	FFACT(CLK_TOP_MMPLL_D2, "top_mmpll_d2", "mmpll", 1, 2),
	FFACT(CLK_TOP_MMPLL_D4, "top_mmpll_d4", "mmpll", 1, 4),
	FFACT(CLK_TOP_MMPLL_D8, "top_mmpll_d8", "mmpll", 1, 8),
	FFACT(CLK_TOP_MMPLL_D8_D2, "top_mmpll_d8_d2", "mmpll", 1, 16),
	FFACT(CLK_TOP_MMPLL_D3_D8, "top_mmpll_d3_d8", "mmpll", 1, 24),
	FFACT(CLK_TOP_MMPLL_U2PHY, "top_mmpll_u2phy", "mmpll", 1, 30),
	/* APLL2 */
	FFACT(CLK_TOP_APLL2_D4, "top_apll2_d4", "apll2", 1, 4),
	/* NET1PLL */
	FFACT(CLK_TOP_NET1PLL_D4, "top_net1pll_d4", "net1pll", 1, 4),
	FFACT(CLK_TOP_NET1PLL_D5, "top_net1pll_d5", "net1pll", 1, 5),
	FFACT(CLK_TOP_NET1PLL_D5_D2, "top_net1pll_d5_d2", "net1pll", 1, 10),
	FFACT(CLK_TOP_NET1PLL_D5_D4, "top_net1pll_d5_d4", "net1pll", 1, 20),
	FFACT(CLK_TOP_NET1PLL_D8_D2, "top_net1pll_d8_d2", "net1pll", 1, 16),
	FFACT(CLK_TOP_NET1PLL_D8_D4, "top_net1pll_d8_d4", "net1pll", 1, 32),
	/* NET2PLL */
	FFACT(CLK_TOP_NET2PLL_D4, "top_net2pll_d4", "net2pll", 1, 4),
	FFACT(CLK_TOP_NET2PLL_D4_D2, "top_net2pll_d4_d2", "net2pll", 1, 8),
	FFACT(CLK_TOP_NET2PLL_D3_D2, "top_net2pll_d3_d2", "net2pll", 1, 2),
	/* WEDMCUPLL */
	FFACT(CLK_TOP_WEDMCUPLL_D5_D2, "top_wedmcupll_d5_d2", "wedmcupll", 1,
	    10),
};

static struct clk_gate_def gates_clk[] = {
	GATE(CLK_TOP_NFI1X_SEL, "nfi1x_sel", "nfi1x_sel_mux", CLK_CFG_0, 7),
	GATE(CLK_TOP_SPINFI_SEL, "spinfi_sel", "spinfi_sel_mux", CLK_CFG_0, 15),
	GATE(CLK_TOP_SPI_SEL, "spi_sel", "spi_sel_mux", CLK_CFG_0, 23),
	GATE(CLK_TOP_SPIM_MST_SEL, "spim_mst_sel", "spim_mst_sel_mux", CLK_CFG_0, 31),

	GATE(CLK_TOP_UART_SEL, "uart_sel", "uart_sel_mux", CLK_CFG_1, 7),
	GATE(CLK_TOP_PWM_SEL, "pwm_sel", "pwm_sel_mux", CLK_CFG_1, 15),
	GATE(CLK_TOP_I2C_SEL, "i2c_sel", "i2c_sel_mux", CLK_CFG_1, 23),
	GATE(CLK_TOP_PEXTP_TL_SEL, "pextp_tl_ck_sel", "pextp_tl_ck_sel_mux", CLK_CFG_1, 31),

	GATE(CLK_TOP_EMMC_250M_SEL, "emmc_250m_sel", "emmc_250m_sel_mux", CLK_CFG_2, 7),
	GATE(CLK_TOP_EMMC_416M_SEL, "emmc_416m_sel", "emmc_416m_sel_mux", CLK_CFG_2, 15),
	GATE(CLK_TOP_F_26M_ADC_SEL, "f_26m_adc_sel", "f_26m_adc_sel_mux", CLK_CFG_2, 23),
	GATE(CLK_TOP_DRAMC_SEL, "dramc_sel", "dramc_sel_mux", CLK_CFG_2, 31),

	GATE(CLK_TOP_DRAMC_MD32_SEL, "dramc_md32_sel", "dramc_md32_sel_mux", CLK_CFG_3, 7),
	GATE(CLK_TOP_SYSAXI_SEL, "sysaxi_sel", "sysaxi_sel_mux", CLK_CFG_3, 15),
	GATE(CLK_TOP_SYSAPB_SEL, "sysapb_sel", "sysapb_sel_mux", CLK_CFG_3, 23),
	GATE(CLK_TOP_ARM_DB_MAIN_SEL, "arm_db_main_sel", "arm_db_main_sel_mux", CLK_CFG_3, 31),

	GATE(CLK_TOP_ARM_DB_JTSEL, "arm_db_jtsel", "arm_db_jtsel_mux", CLK_CFG_4, 7),
	GATE(CLK_TOP_NETSYS_SEL, "netsys_sel", "netsys_sel_mux", CLK_CFG_4, 15),
	GATE(CLK_TOP_NETSYS_500M_SEL, "netsys_500m_sel", "netsys_500m_sel_mux", CLK_CFG_4, 23),
	GATE(CLK_TOP_NETSYS_MCU_SEL, "netsys_mcu_sel", "netsys_mcu_sel_mux", CLK_CFG_4, 31),

	GATE(CLK_TOP_NETSYS_2X_SEL, "netsys_2x_sel", "netsys_2x_sel_mux", CLK_CFG_5, 7),
	GATE(CLK_TOP_SGM_325M_SEL, "sgm_325m_sel", "sgm_325m_sel_mux", CLK_CFG_5, 15),
	GATE(CLK_TOP_SGM_REG_SEL, "sgm_reg_sel", "sgm_reg_sel_mux", CLK_CFG_5, 23),
	GATE(CLK_TOP_A1SYS_SEL, "a1sys_sel", "a1sys_sel_mux", CLK_CFG_5, 31),

	GATE(CLK_TOP_CONN_MCUSYS_SEL, "conn_mcusys_sel", "conn_mcusys_sel_mux", CLK_CFG_6, 7),
	GATE(CLK_TOP_EIP_B_SEL, "eip_b_sel", "eip_b_sel_mux", CLK_CFG_6, 15),
	GATE(CLK_TOP_PCIE_PHY_SEL, "pcie_phy_sel", "pcie_phy_sel_mux", CLK_CFG_6, 23),
	GATE(CLK_TOP_USB3_PHY_SEL, "usb3_phy_sel", "usb3_phy_sel_mux", CLK_CFG_6, 31),

	GATE(CLK_TOP_F26M_SEL, "csw_f26m_sel", "csw_f26m_sel_mux", CLK_CFG_7, 7),
	GATE(CLK_TOP_AUD_L_SEL, "aud_l_sel", "aud_l_sel_mux", CLK_CFG_7, 15),
	GATE(CLK_TOP_A_TUNER_SEL, "a_tuner_sel", "a_tuner_sel_mux", CLK_CFG_7, 23),
	GATE(CLK_TOP_U2U3_SEL, "u2u3_sel", "u2u3_sel_mux", CLK_CFG_7, 31),

	GATE(CLK_TOP_U2U3_SYS_SEL, "u2u3_sys_sel", "u2u3_sys_sel_mux", CLK_CFG_8, 7),
	GATE(CLK_TOP_U2U3_XHCI_SEL, "u2u3_xhci_sel", "u2u3_xhci_sel_mux", CLK_CFG_8, 15),
	GATE(CLK_TOP_DA_U2_REFSEL, "da_u2_refsel", "da_u2_refsell_mux", CLK_CFG_8, 23),
	GATE(CLK_TOP_DA_U2_CK_1P_SEL, "da_u2_ck_1p_sel", "da_u2_ck_1p_sel_mux", CLK_CFG_8, 31),

	GATE(CLK_TOP_AP2CNN_HOST_SEL, "ap2cnn_host_sel", "ap2cnn_host_sel_mux", CLK_CFG_9, 7),
};

static struct clk_mux_def muxes_clk[] = {
	MUX0(0, "nfi1x_sel_mux", nfi1x_parents, CLK_CFG_0, 0, 3),
	MUX0(0, "spinfi_sel_mux", spinfi_parents, CLK_CFG_0, 8, 3),
	MUX0(0, "spi_sel_mux", spi_parents, CLK_CFG_0, 16, 3),
	MUX0(0, "spim_mst_sel_mux", spi_parents, CLK_CFG_0, 24, 3),

	MUX0(0, "uart_sel_mux", uart_parents, CLK_CFG_1, 0, 2),
	MUX0(0, "pwm_sel_mux", pwm_parents, CLK_CFG_1, 8, 2),
	MUX0(0, "i2c_sel_mux", i2c_parents, CLK_CFG_1, 16, 2),
	MUX0(0, "pextp_tl_ck_sel_mux", pextp_tl_ck_parents, CLK_CFG_1, 24, 2),

	MUX0(0, "emmc_250m_sel_mux", emmc_250m_parents, CLK_CFG_2, 0, 1),
	MUX0(0, "emmc_416m_sel_mux", emmc_416m_parents, CLK_CFG_2, 8, 1),
	MUX0(0, "f_26m_adc_sel_mux", f_26m_adc_parents, CLK_CFG_2, 16, 1),
	MUX0(0, "dramc_sel_mux", f_26m_adc_parents, CLK_CFG_2, 24, 1),

	MUX0(0, "dramc_md32_sel_mux", dramc_md32_parents, CLK_CFG_3, 0, 1),
	MUX0(0, "sysaxi_sel_mux", sysaxi_parents, CLK_CFG_3, 8, 2),
	MUX0(0, "sysapb_sel_mux", sysapb_parents, CLK_CFG_3, 16, 2),
	MUX0(0, "arm_db_main_sel_mux", arm_db_main_parents, CLK_CFG_3, 24, 1),

	MUX0(0, "arm_db_jtsel_mux", arm_db_jtsel_parents, CLK_CFG_4, 0, 1),
	MUX0(0, "netsys_sel_mux", netsys_parents, CLK_CFG_4, 8, 1),
	MUX0(0, "netsys_500m_sel_mux", netsys_500m_parents, CLK_CFG_4, 16, 1),
	MUX0(0, "netsys_mcu_sel_mux", netsys_mcu_parents, CLK_CFG_4, 24, 3),

	MUX0(0, "netsys_2x_sel_mux", netsys_2x_parents, CLK_CFG_5, 0, 1),
	MUX0(0, "sgm_325m_sel_mux", sgm_325m_parents, CLK_CFG_5, 8, 1),
	MUX0(0, "sgm_reg_sel_mux", sgm_reg_parents, CLK_CFG_5, 16, 1),
	MUX0(0, "a1sys_sel_mux", a1sys_parents, CLK_CFG_5, 24, 3),

	MUX0(0, "conn_mcusys_sel_mux", conn_mcusys_parents, CLK_CFG_6, 0, 1),
	MUX0(0, "eip_b_sel_mux", eip_b_parents, CLK_CFG_6, 8, 1),
	MUX0(0, "pcie_phy_sel_mux", f_26m_adc_parents, CLK_CFG_6, 16, 1),
	MUX0(0, "usb3_phy_sel_mux", f_26m_adc_parents, CLK_CFG_6, 24, 1),

	MUX0(0, "csw_f26m_sel_mux", conn_mcusys_parents, CLK_CFG_7, 0, 1),
	MUX0(0, "aud_l_sel_mux", aud_l_parents, CLK_CFG_7, 8, 2),
	MUX0(0, "a_tuner_sel_mux", a_tuner_parents, CLK_CFG_7, 16, 2),
	MUX0(0, "u2u3_sel_mux", f_26m_adc_parents, CLK_CFG_7, 24, 1),

	MUX0(0, "u2u3_sys_sel_mux", u2u3_sys_parents, CLK_CFG_8, 0, 1),
	MUX0(0, "u2u3_xhci_sel_mux", u2u3_sys_parents, CLK_CFG_8, 8, 2),
	MUX0(0, "da_u2_refsell_mux", da_u2_refsel_parents, CLK_CFG_8, 16, 2),
	MUX0(0, "da_u2_ck_1p_sel_mux", da_u2_refsel_parents, CLK_CFG_8, 24, 1),

	MUX0(0, "ap2cnn_host_sel_mux", sgm_reg_parents, CLK_CFG_9, 0, 1),
};

static struct mdtk_clk_def clk_def = {
	.linked_def = NULL,
	.num_linked = 0,
	.fixed_def = fixed_clk,
	.num_fixed = nitems(fixed_clk),
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
	.dived_def = NULL,
	.num_dived = 0,
	.muxes_def = muxes_clk,
	.num_muxes = nitems(muxes_clk),
};

static int
topckgen_clk_detach(device_t dev)
{
	device_printf(dev, "Error: Clock driver cannot be detached\n");
	return (EBUSY);
}

static int
topckgen_clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Mediatek Topckgen clocks");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
topckgen_clk_attach(device_t dev) {
	struct mdtk_clk_softc *sc = device_get_softc(dev);
	int rid, rv;

	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Resource setup. */
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->mem_res) {
		device_printf(dev, "cannot allocate memory resource\n");
		rv = ENXIO;
		goto fail;
	}

	mdtk_register_clocks(dev,  &clk_def);
	return (0);

fail:
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

	return (rv);
}

static device_method_t mdtk_mt7986_topckgen_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		 topckgen_clk_probe),
	DEVMETHOD(device_attach,	     topckgen_clk_attach),
	DEVMETHOD(device_detach, 	 topckgen_clk_detach),

	/* Clkdev interface*/
	DEVMETHOD(clkdev_read_4,	mdtk_clkdev_read_4),
	DEVMETHOD(clkdev_write_4,	mdtk_clkdev_write_4),
	DEVMETHOD(clkdev_modify_4,	mdtk_clkdev_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_clkdev_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_clkdev_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(mdtk_mt7986_topckgen, mdtk_mt7986_topckgen_driver, mdtk_mt7986_topckgen_methods,
    sizeof(struct mdtk_clk_softc));

EARLY_DRIVER_MODULE(mdtk_mt7986_topckgen, simplebus, mdtk_mt7986_topckgen_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 2);