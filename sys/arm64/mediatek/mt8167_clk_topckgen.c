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
 * MT8167 top clock generator (topckgen).
 *
 * Fixed clocks, the fixed dividers behind the PLLs, the clock source
 * multiplexers, the audio dividers and six banks of clock gates.
 *
 * Several multiplexers use a sparse encoding.  The unused selector values are
 * routed to "clk_null" and every parent list is padded to the full size of
 * its bit field, so that whatever the firmware left in the register maps to a
 * valid parent.
 */

/* Clock source select registers. */
#define	TOP_CLKSEL0		0x000
#define	TOP_CLKSEL1		0x004
#define	TOP_CLKSEL8		0x040
#define	TOP_CLKSEL9		0x044
#define	TOP_CLKSEL13		0x07c

/* Audio clock dividers. */
#define	TOP_CLKDIV0		0x048
#define	TOP_CLKDIV1		0x04c
#define	TOP_CLKDIV2		0x078

/*
 * Clock gate banks.  Banks 0 to 4 have a status register and write-one
 * SET and CLR registers; bank 5 shares TOP_CLKSEL9 and is modified in place.
 */
#define	TOP_CG0_STA		0x020
#define	TOP_CG0_SET		0x050
#define	TOP_CG0_CLR		0x080
#define	TOP_CG1_STA		0x024
#define	TOP_CG1_SET		0x054
#define	TOP_CG1_CLR		0x084
#define	TOP_CG2_STA		0x03c
#define	TOP_CG2_SET		0x06c
#define	TOP_CG2_CLR		0x09c
#define	TOP_CG3_STA		0x070
#define	TOP_CG3_SET		0x0a0
#define	TOP_CG3_CLR		0x0b0
#define	TOP_CG4_STA		0x074
#define	TOP_CG4_SET		0x0a4
#define	TOP_CG4_CLR		0x0b4
#define	TOP_CG5_STA		TOP_CLKSEL9
#define	TOP_CG5_SET		TOP_CLKSEL9
#define	TOP_CG5_CLR		TOP_CLKSEL9

#define	CG(_bank, _id, _name, _pname, _bit, _flags)			\
	MT_GATE(_id, _name, _pname, TOP_CG ## _bank ## _SET,		\
	    TOP_CG ## _bank ## _CLR, TOP_CG ## _bank ## _STA, _bit, _flags)

/*
 * Clocks of the infracfg controller that are used as parents here.  That
 * controller in turn takes its parents from this one, so neither can be
 * registered first with all its parents present.  The stubs below stand in
 * until infracfg attaches and replaces them with the real clocks.
 */
#define	TOP_LINK(_n, _name)						\
{									\
	.clkdef.id = MT8167_CLK_TOP_NR_CLK + (_n),			\
	.clkdef.name = _name,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
}

static struct clk_link_def linked_clk[] = {
	TOP_LINK(0, "ifr_eth_25m_sel"),
	TOP_LINK(1, "ifr_i2c0_sel"),
	TOP_LINK(2, "ifr_i2c1_sel"),
	TOP_LINK(3, "ifr_i2c2_sel"),
};

static struct clk_fixed_def fixed_clk[] = {
	FRATE(CLK_TOP_CLK_NULL, "clk_null", 0),
	FRATE(CLK_TOP_I2S_INFRA_BCK, "i2s_infra_bck", 26000000),
	FRATE(CLK_TOP_MEMPLL, "mempll", 800000000),
	FRATE(CLK_TOP_DSI0_LNTC_DSICK, "dsi0_lntc_dsick", 75000000),
	FRATE(CLK_TOP_VPLL_DPIX, "vpll_dpix", 75000000),
	FRATE(CLK_TOP_LVDSTX_CLKDIG_CTS, "lvdstx_dig_cts", 52500000),

	FFACT(CLK_TOP_DMPLL, "dmpll_ck", "mempll", 1, 1),
	FFACT(CLK_TOP_MAINPLL_D2, "mainpll_d2", "mainpll", 1, 2),
	FFACT(CLK_TOP_MAINPLL_D4, "mainpll_d4", "mainpll", 1, 4),
	FFACT(CLK_TOP_MAINPLL_D8, "mainpll_d8", "mainpll", 1, 8),
	FFACT(CLK_TOP_MAINPLL_D16, "mainpll_d16", "mainpll", 1, 16),
	FFACT(CLK_TOP_MAINPLL_D11, "mainpll_d11", "mainpll", 1, 11),
	FFACT(CLK_TOP_MAINPLL_D22, "mainpll_d22", "mainpll", 1, 22),
	FFACT(CLK_TOP_MAINPLL_D3, "mainpll_d3", "mainpll", 1, 3),
	FFACT(CLK_TOP_MAINPLL_D6, "mainpll_d6", "mainpll", 1, 6),
	FFACT(CLK_TOP_MAINPLL_D12, "mainpll_d12", "mainpll", 1, 12),
	FFACT(CLK_TOP_MAINPLL_D5, "mainpll_d5", "mainpll", 1, 5),
	FFACT(CLK_TOP_MAINPLL_D10, "mainpll_d10", "mainpll", 1, 10),
	FFACT(CLK_TOP_MAINPLL_D20, "mainpll_d20", "mainpll", 1, 20),
	FFACT(CLK_TOP_MAINPLL_D40, "mainpll_d40", "mainpll", 1, 40),
	FFACT(CLK_TOP_MAINPLL_D7, "mainpll_d7", "mainpll", 1, 7),
	FFACT(CLK_TOP_MAINPLL_D14, "mainpll_d14", "mainpll", 1, 14),
	FFACT(CLK_TOP_UNIVPLL_D2, "univpll_d2", "univpll", 1, 2),
	FFACT(CLK_TOP_UNIVPLL_D4, "univpll_d4", "univpll", 1, 4),
	FFACT(CLK_TOP_UNIVPLL_D8, "univpll_d8", "univpll", 1, 8),
	FFACT(CLK_TOP_UNIVPLL_D16, "univpll_d16", "univpll", 1, 16),
	FFACT(CLK_TOP_UNIVPLL_D3, "univpll_d3", "univpll", 1, 3),
	FFACT(CLK_TOP_UNIVPLL_D6, "univpll_d6", "univpll", 1, 6),
	FFACT(CLK_TOP_UNIVPLL_D12, "univpll_d12", "univpll", 1, 12),
	FFACT(CLK_TOP_UNIVPLL_D24, "univpll_d24", "univpll", 1, 24),
	FFACT(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5),
	FFACT(CLK_TOP_UNIVPLL_D20, "univpll_d20", "univpll", 1, 20),
	FFACT(CLK_TOP_MMPLL380M, "mmpll380m", "mmpll", 1, 1),
	FFACT(CLK_TOP_MMPLL_D2, "mmpll_d2", "mmpll", 1, 2),
	FFACT(CLK_TOP_MMPLL_200M, "mmpll_200m", "mmpll", 1, 3),
	FFACT(CLK_TOP_LVDSPLL, "lvdspll_ck", "lvdspll", 1, 1),
	FFACT(CLK_TOP_LVDSPLL_D2, "lvdspll_d2", "lvdspll", 1, 2),
	FFACT(CLK_TOP_LVDSPLL_D4, "lvdspll_d4", "lvdspll", 1, 4),
	FFACT(CLK_TOP_LVDSPLL_D8, "lvdspll_d8", "lvdspll", 1, 8),
	FFACT(CLK_TOP_USB_PHY48M, "usb_phy48m_ck", "univpll", 1, 26),
	FFACT(CLK_TOP_APLL1, "apll1_ck", "apll1", 1, 1),
	FFACT(CLK_TOP_APLL1_D2, "apll1_d2", "apll1_ck", 1, 2),
	FFACT(CLK_TOP_APLL1_D4, "apll1_d4", "rg_apll1_d2_en", 1, 2),
	FFACT(CLK_TOP_APLL1_D8, "apll1_d8", "rg_apll1_d4_en", 1, 2),
	FFACT(CLK_TOP_APLL2, "apll2_ck", "apll2", 1, 1),
	FFACT(CLK_TOP_APLL2_D2, "apll2_d2", "apll2_ck", 1, 2),
	FFACT(CLK_TOP_APLL2_D4, "apll2_d4", "rg_apll2_d2_en", 1, 2),
	FFACT(CLK_TOP_APLL2_D8, "apll2_d8", "rg_apll2_d4_en", 1, 2),
	FFACT(CLK_TOP_CLK26M, "clk26m_ck", "clk26m", 1, 1),
	FFACT(CLK_TOP_CLK26M_D2, "clk26m_d2", "clk26m", 1, 2),
	FFACT(CLK_TOP_MIPI_26M, "mipi_26m", "clk26m", 1, 1),
	FFACT(CLK_TOP_TVDPLL, "tvdpll_ck", "tvdpll", 1, 1),
	FFACT(CLK_TOP_TVDPLL_D2, "tvdpll_d2", "tvdpll_ck", 1, 2),
	FFACT(CLK_TOP_TVDPLL_D4, "tvdpll_d4", "tvdpll_ck", 1, 4),
	FFACT(CLK_TOP_TVDPLL_D8, "tvdpll_d8", "tvdpll_ck", 1, 8),
	FFACT(CLK_TOP_TVDPLL_D16, "tvdpll_d16", "tvdpll_ck", 1, 16),
	FFACT(CLK_TOP_AHB_INFRA_D2, "ahb_infra_d2", "ahb_infra_sel", 1, 2),
	FFACT(CLK_TOP_NFI1X, "nfi1x_ck", "nfi2x_pad_sel", 1, 2),
	FFACT(CLK_TOP_ETH_D2, "eth_d2_ck", "eth_sel", 1, 2),
};

PLIST(p_uart0) = {
	"clk26m_ck", "univpll_d24"
};

PLIST(p_gfmux_emi1x) = {
	"clk26m_ck", "dmpll_ck"
};

PLIST(p_emi_ddrphy) = {
	"gfmux_emi1x_sel", "gfmux_emi1x_sel"
};

PLIST(p_ahb_infra) = {
	"clk_null", "clk26m_ck", "mainpll_d11", "clk_null", "mainpll_d12",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "mainpll_d10", "clk_null", "clk_null",
	"clk_null"
};

PLIST(p_csw_mux_mfg) = {
	"clk_null", "clk_null", "univpll_d3", "univpll_d2", "clk26m_ck",
	"mainpll_d4", "univpll_d24", "mmpll380m"
};

PLIST(p_msdc0) = {
	"clk26m_ck", "univpll_d6", "mainpll_d8", "univpll_d8", "mainpll_d16",
	"mmpll_200m", "mainpll_d12", "mmpll_d2"
};

PLIST(p_camtg_mm) = {
	"clk_null", "clk26m_ck", "usb_phy48m_ck", "clk_null", "univpll_d6",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_pwm_mm) = {
	"clk26m_ck", "univpll_d12"
};

PLIST(p_uart1) = {
	"clk26m_ck", "univpll_d24"
};

PLIST(p_msdc1) = {
	"clk26m_ck", "univpll_d6", "mainpll_d8", "univpll_d8", "mainpll_d16",
	"mmpll_200m", "mainpll_d12", "mmpll_d2"
};

PLIST(p_spm_52m) = {
	"clk26m_ck", "univpll_d24"
};

PLIST(p_pmicspi) = {
	"univpll_d20", "usb_phy48m_ck", "univpll_d16", "clk26m_ck"
};

PLIST(p_qaxi_aud26m) = {
	"clk26m_ck", "ahb_infra_sel"
};

PLIST(p_aud_intbus) = {
	"clk_null", "clk26m_ck", "mainpll_d22", "clk_null", "mainpll_d11",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_nfi2x_pad) = {
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk26m_ck", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "mainpll_d12", "mainpll_d8", "clk_null",
	"mainpll_d6", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "mainpll_d4", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "mainpll_d10", "mainpll_d7", "clk_null", "mainpll_d5",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_nfi1x_pad) = {
	"ahb_infra_sel", "nfi1x_ck"
};

PLIST(p_mfg_mm) = {
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "csw_mux_mfg_sel", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "mainpll_d3", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "mainpll_d5", "mainpll_d7",
	"clk_null", "mainpll_d14", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null"
};

PLIST(p_ddrphycfg) = {
	"clk26m_ck", "mainpll_d16"
};

PLIST(p_smi_mm) = {
	"clk26m_ck", "clk_null", "clk_null", "clk_null", "clk_null",
	"clk_null", "clk_null", "clk_null", "clk_null", "univpll_d4",
	"mainpll_d7", "clk_null", "mainpll_d14", "clk_null", "clk_null",
	"clk_null"
};

PLIST(p_usb_78m) = {
	"clk_null", "clk26m_ck", "univpll_d16", "clk_null", "mainpll_d20",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_scam_mm) = {
	"clk_null", "clk26m_ck", "mainpll_d14", "clk_null", "mainpll_d12",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_spinor) = {
	"clk26m_d2", "clk26m_ck", "mainpll_d40", "univpll_d24", "univpll_d20",
	"mainpll_d20", "mainpll_d16", "univpll_d12"
};

PLIST(p_msdc2) = {
	"clk26m_ck", "univpll_d6", "mainpll_d8", "univpll_d8", "mainpll_d16",
	"mmpll_200m", "mainpll_d12", "mmpll_d2"
};

PLIST(p_eth) = {
	"clk26m_ck", "mainpll_d40", "univpll_d24", "univpll_d20",
	"mainpll_d20", "clk_null", "clk_null", "clk_null"
};

PLIST(p_vdec_mm) = {
	"clk26m_ck", "univpll_d4", "mainpll_d4", "univpll_d5", "univpll_d6",
	"mainpll_d6", "clk_null", "clk_null"
};

PLIST(p_dpi0_mm) = {
	"clk26m_ck", "lvdspll_ck", "lvdspll_d2", "lvdspll_d4", "lvdspll_d8",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_dpi1_mm) = {
	"clk26m_ck", "tvdpll_d2", "tvdpll_d4", "tvdpll_d8", "tvdpll_d16",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_axi_mfg_in) = {
	"clk26m_ck", "mainpll_d11", "univpll_d24", "mmpll380m"
};

PLIST(p_slow_mfg) = {
	"clk26m_ck", "univpll_d12", "univpll_d24", "clk_null"
};

PLIST(p_aud1) = {
	"clk26m_ck", "apll1_ck"
};

PLIST(p_aud2) = {
	"clk26m_ck", "apll2_ck"
};

PLIST(p_aud_engen1) = {
	"clk26m_ck", "rg_apll1_d2_en", "rg_apll1_d4_en", "rg_apll1_d8_en"
};

PLIST(p_aud_engen2) = {
	"clk26m_ck", "rg_apll2_d2_en", "rg_apll2_d4_en", "rg_apll2_d8_en"
};

PLIST(p_i2c) = {
	"clk26m_ck", "univpll_d20", "univpll_d16", "univpll_d12"
};

PLIST(p_aud_i2s0_m) = {
	"rg_aud1", "rg_aud2"
};

PLIST(p_pwm) = {
	"clk26m_ck", "univpll_d12"
};

PLIST(p_spi) = {
	"univpll_d20", "usb_phy48m_ck", "univpll_d16", "clk26m_ck"
};

PLIST(p_aud_spdifin) = {
	"clk26m_ck", "univpll_d2"
};

PLIST(p_uart2) = {
	"clk26m_ck", "univpll_d24"
};

PLIST(p_bsi) = {
	"clk26m_ck", "mainpll_d10", "mainpll_d12", "mainpll_d20"
};

PLIST(p_dbg_atclk) = {
	"clk_null", "clk26m_ck", "mainpll_d5", "clk_null", "univpll_d5",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_csw_nfiecc) = {
	"clk_null", "mainpll_d7", "mainpll_d6", "clk_null", "mainpll_d5",
	"clk_null", "clk_null", "clk_null"
};

PLIST(p_nfiecc) = {
	"clk_null", "mainpll_d7", "mainpll_d6", "clk_null", "mainpll_d5",
	"clk_null", "clk_null", "clk_null"
};

static struct clk_mux_def muxes_clk[] = {
	MUX0(CLK_TOP_UART0_SEL, "uart0_sel", p_uart0, TOP_CLKSEL0, 0, 1),
	MUX0(CLK_TOP_GFMUX_EMI1X_SEL, "gfmux_emi1x_sel", p_gfmux_emi1x,
	    TOP_CLKSEL0, 1, 1),
	MUX0(CLK_TOP_EMI_DDRPHY_SEL, "emi_ddrphy_sel", p_emi_ddrphy,
	    TOP_CLKSEL0, 2, 1),
	MUX0(CLK_TOP_AHB_INFRA_SEL, "ahb_infra_sel", p_ahb_infra, TOP_CLKSEL0,
	    4, 4),
	MUX0(CLK_TOP_CSW_MUX_MFG_SEL, "csw_mux_mfg_sel", p_csw_mux_mfg,
	    TOP_CLKSEL0, 8, 3),
	MUX0(CLK_TOP_MSDC0_SEL, "msdc0_sel", p_msdc0, TOP_CLKSEL0, 11, 3),
	MUX0(CLK_TOP_CAMTG_MM_SEL, "camtg_mm_sel", p_camtg_mm, TOP_CLKSEL0, 15,
	    3),
	MUX0(CLK_TOP_PWM_MM_SEL, "pwm_mm_sel", p_pwm_mm, TOP_CLKSEL0, 18, 1),
	MUX0(CLK_TOP_UART1_SEL, "uart1_sel", p_uart1, TOP_CLKSEL0, 19, 1),
	MUX0(CLK_TOP_MSDC1_SEL, "msdc1_sel", p_msdc1, TOP_CLKSEL0, 20, 3),
	MUX0(CLK_TOP_SPM_52M_SEL, "spm_52m_sel", p_spm_52m, TOP_CLKSEL0, 23, 1),
	MUX0(CLK_TOP_PMICSPI_SEL, "pmicspi_sel", p_pmicspi, TOP_CLKSEL0, 24, 2),
	MUX0(CLK_TOP_QAXI_AUD26M_SEL, "qaxi_aud26m_sel", p_qaxi_aud26m,
	    TOP_CLKSEL0, 26, 1),
	MUX0(CLK_TOP_AUD_INTBUS_SEL, "aud_intbus_sel", p_aud_intbus,
	    TOP_CLKSEL0, 27, 3),
	MUX0(CLK_TOP_NFI2X_PAD_SEL, "nfi2x_pad_sel", p_nfi2x_pad, TOP_CLKSEL1,
	    0, 7),
	MUX0(CLK_TOP_NFI1X_PAD_SEL, "nfi1x_pad_sel", p_nfi1x_pad, TOP_CLKSEL1,
	    7, 1),
	MUX0(CLK_TOP_MFG_MM_SEL, "mfg_mm_sel", p_mfg_mm, TOP_CLKSEL1, 8, 6),
	MUX0(CLK_TOP_DDRPHYCFG_SEL, "ddrphycfg_sel", p_ddrphycfg, TOP_CLKSEL1,
	    15, 1),
	MUX0(CLK_TOP_SMI_MM_SEL, "smi_mm_sel", p_smi_mm, TOP_CLKSEL1, 16, 4),
	MUX0(CLK_TOP_USB_78M_SEL, "usb_78m_sel", p_usb_78m, TOP_CLKSEL1, 20, 3),
	MUX0(CLK_TOP_SCAM_MM_SEL, "scam_mm_sel", p_scam_mm, TOP_CLKSEL1, 23, 3),
	MUX0(CLK_TOP_SPINOR_SEL, "spinor_sel", p_spinor, TOP_CLKSEL8, 0, 3),
	MUX0(CLK_TOP_MSDC2_SEL, "msdc2_sel", p_msdc2, TOP_CLKSEL8, 3, 3),
	MUX0(CLK_TOP_ETH_SEL, "eth_sel", p_eth, TOP_CLKSEL8, 6, 3),
	MUX0(CLK_TOP_VDEC_MM_SEL, "vdec_mm_sel", p_vdec_mm, TOP_CLKSEL8, 9, 3),
	MUX0(CLK_TOP_DPI0_MM_SEL, "dpi0_mm_sel", p_dpi0_mm, TOP_CLKSEL8, 12, 3),
	MUX0(CLK_TOP_DPI1_MM_SEL, "dpi1_mm_sel", p_dpi1_mm, TOP_CLKSEL8, 15, 3),
	MUX0(CLK_TOP_AXI_MFG_IN_SEL, "axi_mfg_in_sel", p_axi_mfg_in,
	    TOP_CLKSEL8, 18, 2),
	MUX0(CLK_TOP_SLOW_MFG_SEL, "slow_mfg_sel", p_slow_mfg, TOP_CLKSEL8, 20,
	    2),
	MUX0(CLK_TOP_AUD1_SEL, "aud1_sel", p_aud1, TOP_CLKSEL8, 22, 1),
	MUX0(CLK_TOP_AUD2_SEL, "aud2_sel", p_aud2, TOP_CLKSEL8, 23, 1),
	MUX0(CLK_TOP_AUD_ENGEN1_SEL, "aud_engen1_sel", p_aud_engen1,
	    TOP_CLKSEL8, 24, 2),
	MUX0(CLK_TOP_AUD_ENGEN2_SEL, "aud_engen2_sel", p_aud_engen2,
	    TOP_CLKSEL8, 26, 2),
	MUX0(CLK_TOP_I2C_SEL, "i2c_sel", p_i2c, TOP_CLKSEL8, 28, 2),
	MUX0(CLK_TOP_AUD_I2S0_M_SEL, "aud_i2s0_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 12, 1),
	MUX0(CLK_TOP_AUD_I2S1_M_SEL, "aud_i2s1_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 13, 1),
	MUX0(CLK_TOP_AUD_I2S2_M_SEL, "aud_i2s2_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 14, 1),
	MUX0(CLK_TOP_AUD_I2S3_M_SEL, "aud_i2s3_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 15, 1),
	MUX0(CLK_TOP_AUD_I2S4_M_SEL, "aud_i2s4_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 16, 1),
	MUX0(CLK_TOP_AUD_I2S5_M_SEL, "aud_i2s5_m_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 17, 1),
	MUX0(CLK_TOP_AUD_SPDIF_B_SEL, "aud_spdif_b_sel", p_aud_i2s0_m,
	    TOP_CLKSEL9, 18, 1),
	MUX0(CLK_TOP_PWM_SEL, "pwm_sel", p_pwm, TOP_CLKSEL13, 0, 1),
	MUX0(CLK_TOP_SPI_SEL, "spi_sel", p_spi, TOP_CLKSEL13, 1, 2),
	MUX0(CLK_TOP_AUD_SPDIFIN_SEL, "aud_spdifin_sel", p_aud_spdifin,
	    TOP_CLKSEL13, 3, 1),
	MUX0(CLK_TOP_UART2_SEL, "uart2_sel", p_uart2, TOP_CLKSEL13, 4, 1),
	MUX0(CLK_TOP_BSI_SEL, "bsi_sel", p_bsi, TOP_CLKSEL13, 5, 2),
	MUX0(CLK_TOP_DBG_ATCLK_SEL, "dbg_atclk_sel", p_dbg_atclk, TOP_CLKSEL13,
	    7, 3),
	MUX0(CLK_TOP_CSW_NFIECC_SEL, "csw_nfiecc_sel", p_csw_nfiecc,
	    TOP_CLKSEL13, 10, 3),
	MUX0(CLK_TOP_NFIECC_SEL, "nfiecc_sel", p_nfiecc, TOP_CLKSEL13, 13, 3),
};

static struct clk_div_def dived_clk[] = {
	DIV(CLK_TOP_APLL12_CK_DIV0, "apll12_ck_div0", "aud_i2s0_m_sel",
	    TOP_CLKDIV0, 0, 8),
	DIV(CLK_TOP_APLL12_CK_DIV1, "apll12_ck_div1", "aud_i2s1_m_sel",
	    TOP_CLKDIV0, 8, 8),
	DIV(CLK_TOP_APLL12_CK_DIV2, "apll12_ck_div2", "aud_i2s2_m_sel",
	    TOP_CLKDIV0, 16, 8),
	DIV(CLK_TOP_APLL12_CK_DIV3, "apll12_ck_div3", "aud_i2s3_m_sel",
	    TOP_CLKDIV0, 24, 8),
	DIV(CLK_TOP_APLL12_CK_DIV4, "apll12_ck_div4", "aud_i2s4_m_sel",
	    TOP_CLKDIV1, 0, 8),
	DIV(CLK_TOP_APLL12_CK_DIV4B, "apll12_ck_div4b", "apll12_div4",
	    TOP_CLKDIV1, 8, 8),
	DIV(CLK_TOP_APLL12_CK_DIV5, "apll12_ck_div5", "aud_i2s5_m_sel",
	    TOP_CLKDIV1, 16, 8),
	DIV(CLK_TOP_APLL12_CK_DIV5B, "apll12_ck_div5b", "apll12_div5",
	    TOP_CLKDIV1, 24, 8),
	DIV(CLK_TOP_APLL12_CK_DIV6, "apll12_ck_div6", "aud_spdif_b_sel",
	    TOP_CLKDIV2, 0, 8),
};

static struct mt_clk_gate_def gates_clk[] = {
	CG(0, CLK_TOP_PWM_MM, "pwm_mm", "pwm_mm_sel", 0, 0),
	CG(0, CLK_TOP_CAM_MM, "cam_mm", "camtg_mm_sel", 1, 0),
	CG(0, CLK_TOP_MFG_MM, "mfg_mm", "mfg_mm_sel", 2, 0),
	CG(0, CLK_TOP_SPM_52M, "spm_52m", "spm_52m_sel", 3, 0),
	CG(0, CLK_TOP_MIPI_26M_DBG, "mipi_26m_dbg", "mipi_26m", 4, MT_GATE_INV),
	CG(0, CLK_TOP_SCAM_MM, "scam_mm", "scam_mm_sel", 5, 0),
	CG(0, CLK_TOP_SMI_MM, "smi_mm", "smi_mm_sel", 9, 0),
	CG(1, CLK_TOP_THEM, "them", "ahb_infra_sel", 1, 0),
	CG(1, CLK_TOP_APDMA, "apdma", "ahb_infra_sel", 2, 0),
	CG(1, CLK_TOP_I2C0, "i2c0", "ifr_i2c0_sel", 3, 0),
	CG(1, CLK_TOP_I2C1, "i2c1", "ifr_i2c1_sel", 4, 0),
	CG(1, CLK_TOP_AUXADC1, "auxadc1", "ahb_infra_sel", 5, 0),
	CG(1, CLK_TOP_NFI, "nfi", "nfi1x_pad_sel", 6, 0),
	CG(1, CLK_TOP_NFIECC, "nfiecc", "rg_nfiecc", 7, 0),
	CG(1, CLK_TOP_DEBUGSYS, "debugsys", "rg_dbg_atclk", 8, 0),
	CG(1, CLK_TOP_PWM, "pwm", "ahb_infra_sel", 9, 0),
	CG(1, CLK_TOP_UART0, "uart0", "uart0_sel", 10, 0),
	CG(1, CLK_TOP_UART1, "uart1", "uart1_sel", 11, 0),
	CG(1, CLK_TOP_BTIF, "btif", "ahb_infra_sel", 12, 0),
	CG(1, CLK_TOP_USB, "usb", "usb_78m", 13, 0),
	CG(1, CLK_TOP_FLASHIF_26M, "flashif_26m", "clk26m_ck", 14, 0),
	CG(1, CLK_TOP_AUXADC2, "auxadc2", "ahb_infra_sel", 15, 0),
	CG(1, CLK_TOP_I2C2, "i2c2", "ifr_i2c2_sel", 16, 0),
	CG(1, CLK_TOP_MSDC0, "msdc0", "msdc0_sel", 17, 0),
	CG(1, CLK_TOP_MSDC1, "msdc1", "msdc1_sel", 18, 0),
	CG(1, CLK_TOP_NFI2X, "nfi2x", "nfi2x_pad_sel", 19, 0),
	CG(1, CLK_TOP_PMICWRAP_AP, "pwrap_ap", "clk26m_ck", 20, 0),
	CG(1, CLK_TOP_SEJ, "sej", "ahb_infra_sel", 21, 0),
	CG(1, CLK_TOP_MEMSLP_DLYER, "memslp_dlyer", "clk26m_ck", 22, 0),
	CG(1, CLK_TOP_SPI, "spi", "spi_sel", 23, 0),
	CG(1, CLK_TOP_APXGPT, "apxgpt", "clk26m_ck", 24, 0),
	CG(1, CLK_TOP_AUDIO, "audio", "clk26m_ck", 25, 0),
	CG(1, CLK_TOP_PMICWRAP_MD, "pwrap_md", "clk26m_ck", 27, 0),
	CG(1, CLK_TOP_PMICWRAP_CONN, "pwrap_conn", "clk26m_ck", 28, 0),
	CG(1, CLK_TOP_PMICWRAP_26M, "pwrap_26m", "clk26m_ck", 29, 0),
	CG(1, CLK_TOP_AUX_ADC, "aux_adc", "clk26m_ck", 30, 0),
	CG(1, CLK_TOP_AUX_TP, "aux_tp", "clk26m_ck", 31, 0),
	CG(2, CLK_TOP_MSDC2, "msdc2", "ahb_infra_sel", 0, 0),
	CG(2, CLK_TOP_RBIST, "rbist", "univpll_d12", 1, 0),
	CG(2, CLK_TOP_NFI_BUS, "nfi_bus", "ahb_infra_sel", 2, 0),
	CG(2, CLK_TOP_GCE, "gce", "ahb_infra_sel", 4, 0),
	CG(2, CLK_TOP_TRNG, "trng", "ahb_infra_sel", 5, 0),
	CG(2, CLK_TOP_SEJ_13M, "sej_13m", "clk26m_ck", 6, 0),
	CG(2, CLK_TOP_AES, "aes", "ahb_infra_sel", 7, 0),
	CG(2, CLK_TOP_PWM_B, "pwm_b", "rg_pwm_infra", 8, 0),
	CG(2, CLK_TOP_PWM1_FB, "pwm1_fb", "rg_pwm_infra", 9, 0),
	CG(2, CLK_TOP_PWM2_FB, "pwm2_fb", "rg_pwm_infra", 10, 0),
	CG(2, CLK_TOP_PWM3_FB, "pwm3_fb", "rg_pwm_infra", 11, 0),
	CG(2, CLK_TOP_PWM4_FB, "pwm4_fb", "rg_pwm_infra", 12, 0),
	CG(2, CLK_TOP_PWM5_FB, "pwm5_fb", "rg_pwm_infra", 13, 0),
	CG(2, CLK_TOP_USB_1P, "usb_1p", "usb_78m", 14, 0),
	CG(2, CLK_TOP_FLASHIF_FREERUN, "flashif_freerun", "ahb_infra_sel", 15,
	    0),
	CG(2, CLK_TOP_26M_HDMI_SIFM, "hdmi_sifm_26m", "clk26m_ck", 16, 0),
	CG(2, CLK_TOP_26M_CEC, "cec_26m", "clk26m_ck", 17, 0),
	CG(2, CLK_TOP_32K_CEC, "cec_32k", "clk32k", 18, 0),
	CG(2, CLK_TOP_66M_ETH, "eth_66m", "ahb_infra_d2", 19, 0),
	CG(2, CLK_TOP_133M_ETH, "eth_133m", "ahb_infra_sel", 20, 0),
	CG(2, CLK_TOP_FETH_25M, "feth_25m", "ifr_eth_25m_sel", 21, 0),
	CG(2, CLK_TOP_FETH_50M, "feth_50m", "rg_eth", 22, 0),
	CG(2, CLK_TOP_FLASHIF_AXI, "flashif_axi", "ahb_infra_sel", 23, 0),
	CG(2, CLK_TOP_USBIF, "usbif", "ahb_infra_sel", 24, 0),
	CG(2, CLK_TOP_UART2, "uart2", "rg_uart2", 25, 0),
	CG(2, CLK_TOP_BSI, "bsi", "ahb_infra_sel", 26, 0),
	CG(2, CLK_TOP_GCPU_B, "gcpu_b", "ahb_infra_sel", 27, 0),
	CG(2, CLK_TOP_MSDC0_INFRA, "msdc0_infra", "msdc0", 28, MT_GATE_INV),
	CG(2, CLK_TOP_MSDC1_INFRA, "msdc1_infra", "msdc1", 29, MT_GATE_INV),
	CG(2, CLK_TOP_MSDC2_INFRA, "msdc2_infra", "rg_msdc2", 30, MT_GATE_INV),
	CG(2, CLK_TOP_USB_78M, "usb_78m", "usb_78m_sel", 31, 0),
	CG(3, CLK_TOP_RG_SPINOR, "rg_spinor", "spinor_sel", 0, 0),
	CG(3, CLK_TOP_RG_MSDC2, "rg_msdc2", "msdc2_sel", 1, 0),
	CG(3, CLK_TOP_RG_ETH, "rg_eth", "eth_sel", 2, 0),
	CG(3, CLK_TOP_RG_VDEC, "rg_vdec", "vdec_mm_sel", 3, 0),
	CG(3, CLK_TOP_RG_FDPI0, "rg_fdpi0", "dpi0_mm_sel", 4, 0),
	CG(3, CLK_TOP_RG_FDPI1, "rg_fdpi1", "dpi1_mm_sel", 5, 0),
	CG(3, CLK_TOP_RG_AXI_MFG, "rg_axi_mfg", "axi_mfg_in_sel", 6, 0),
	CG(3, CLK_TOP_RG_SLOW_MFG, "rg_slow_mfg", "slow_mfg_sel", 7, 0),
	CG(3, CLK_TOP_RG_AUD1, "rg_aud1", "aud1_sel", 8, 0),
	CG(3, CLK_TOP_RG_AUD2, "rg_aud2", "aud2_sel", 9, 0),
	CG(3, CLK_TOP_RG_AUD_ENGEN1, "rg_aud_engen1", "aud_engen1_sel", 10, 0),
	CG(3, CLK_TOP_RG_AUD_ENGEN2, "rg_aud_engen2", "aud_engen2_sel", 11, 0),
	CG(3, CLK_TOP_RG_I2C, "rg_i2c", "i2c_sel", 12, 0),
	CG(3, CLK_TOP_RG_PWM_INFRA, "rg_pwm_infra", "pwm_sel", 13, 0),
	CG(3, CLK_TOP_RG_AUD_SPDIF_IN, "rg_aud_spdif_in", "aud_spdifin_sel",
	    14, 0),
	CG(3, CLK_TOP_RG_UART2, "rg_uart2", "uart2_sel", 15, 0),
	CG(3, CLK_TOP_RG_BSI, "rg_bsi", "bsi_sel", 16, 0),
	CG(3, CLK_TOP_RG_DBG_ATCLK, "rg_dbg_atclk", "dbg_atclk_sel", 17, 0),
	CG(3, CLK_TOP_RG_NFIECC, "rg_nfiecc", "nfiecc_sel", 18, 0),
	CG(4, CLK_TOP_RG_APLL1_D2_EN, "rg_apll1_d2_en", "apll1_d2", 8,
	    MT_GATE_INV),
	CG(4, CLK_TOP_RG_APLL1_D4_EN, "rg_apll1_d4_en", "apll1_d4", 9,
	    MT_GATE_INV),
	CG(4, CLK_TOP_RG_APLL1_D8_EN, "rg_apll1_d8_en", "apll1_d8", 10,
	    MT_GATE_INV),
	CG(4, CLK_TOP_RG_APLL2_D2_EN, "rg_apll2_d2_en", "apll2_d2", 11,
	    MT_GATE_INV),
	CG(4, CLK_TOP_RG_APLL2_D4_EN, "rg_apll2_d4_en", "apll2_d4", 12,
	    MT_GATE_INV),
	CG(4, CLK_TOP_RG_APLL2_D8_EN, "rg_apll2_d8_en", "apll2_d8", 13,
	    MT_GATE_INV),
	CG(5, CLK_TOP_APLL12_DIV0, "apll12_div0", "apll12_ck_div0", 0,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV1, "apll12_div1", "apll12_ck_div1", 1,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV2, "apll12_div2", "apll12_ck_div2", 2,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV3, "apll12_div3", "apll12_ck_div3", 3,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV4, "apll12_div4", "apll12_ck_div4", 4,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV4B, "apll12_div4b", "apll12_ck_div4b", 5,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV5, "apll12_div5", "apll12_ck_div5", 6,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV5B, "apll12_div5b", "apll12_ck_div5b", 7,
	    MT_GATE_NO_SETCLR),
	CG(5, CLK_TOP_APLL12_DIV6, "apll12_div6", "apll12_ck_div6", 8,
	    MT_GATE_NO_SETCLR),
};

static struct mdtk_clk_def clk_def = {
	.linked_def = linked_clk,
	.num_linked = nitems(linked_clk),
	.fixed_def = fixed_clk,
	.num_fixed = nitems(fixed_clk),
	.muxes_def = muxes_clk,
	.num_muxes = nitems(muxes_clk),
	.dived_def = dived_clk,
	.num_dived = nitems(dived_clk),
	.mtgates_def = gates_clk,
	.num_mtgates = nitems(gates_clk),
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt8167-topckgen",	1},
	{NULL,				0},
};

static int
mt8167_topckgen_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek MT8167 topckgen clocks");
	return (BUS_PROBE_DEFAULT);
}

static int
mt8167_topckgen_attach(device_t dev)
{

	return (mdtk_clk_attach(dev, &clk_def));
}

static int
mt8167_topckgen_detach(device_t dev)
{

	device_printf(dev, "clock driver cannot be detached\n");
	return (EBUSY);
}

static device_method_t mt8167_topckgen_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mt8167_topckgen_probe),
	DEVMETHOD(device_attach,	mt8167_topckgen_attach),
	DEVMETHOD(device_detach,	mt8167_topckgen_detach),

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

DEFINE_CLASS_0(mt8167_topckgen, mt8167_topckgen_driver,
    mt8167_topckgen_methods, sizeof(struct mdtk_clk_softc));
EARLY_DRIVER_MODULE(mt8167_topckgen, simplebus, mt8167_topckgen_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 2);
