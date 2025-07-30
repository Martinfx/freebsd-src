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

#include <dt-bindings/clock/mt2701-clk.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_link.h>
#include <arm/mediatek/mdtk_clk.h>
#include <dev/hwreset/hwreset.h>
#include "clkdev_if.h"

#define MHZ	(1000 * 1000)

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7623-topckgen",	1},
	{"mediatek,mt2701-topckgen",	1},
	{ NULL,				0},
};

PLIST(axi_parents) = {
	"clk26m",
	"syspll1_d2",
	"syspll_d5",
	"syspll1_d4",
	"univpll_d5",
	"univpll2_d2",
	"msdcpll_d2",
	"mmpll_d2"
};

PLIST(mem_parents) = {
	"clk26m",
	"dmpll_ck"
};

PLIST(ddrphycfg_parents) = {
	"clk26m",
	"syspll1_d8"
};

PLIST(mm_parents) = {
	"clk26m",
	"vencpll_ck",
	"syspll1_d2",
	"syspll1_d4",
	"univpll_d5",
	"univpll1_d2",
	"univpll2_d2",
	"dmpll_ck"
};

PLIST(pwm_parents) = {
	"clk26m",
	"univpll2_d4",
	"univpll3_d2",
	"univpll1_d4",
};

PLIST(vdec_parents) = {
	"clk26m",
	"vdecpll_ck",
	"syspll_d5",
	"syspll1_d4",
	"univpll_d5",
	"univpll2_d2",
	"vencpll_ck",
	"msdcpll_d2",
	"mmpll_d2"
};

PLIST(mfg_parents) = {
	"clk26m",
	"mmpll_ck",
	//"dmpll_x2_ck",
	"msdcpll_ck",
	"syspll_d3",
	"univpll_d3",
	"univpll1_d2",
	"univpll2_d2"
};

PLIST(camtg_parents) = {
	"clk26m",
	"univpll_d26",
	"univpll2_d2",
	"syspll3_d2",
	"syspll3_d4",
	"msdcpll_d2",
	"mmpll_d2"
};

PLIST(uart_parents) = {
	"clk26m",
	"univpll2_d8"
};

PLIST(spi_parents) = {
	"clk26m",
	"syspll3_d2",
	"syspll4_d2",
	"univpll2_d4",
	"univpll1_d8"
};

PLIST(usb20_parents) = {
	"clk26m",
	"univpll1_d8",
	"univpll3_d4"
};

PLIST(msdc30_parents) = {
	"clk26m",
	"msdcpll_d2",
	"syspll2_d2",
	"syspll1_d4",
	"univpll1_d4",
	"univpll2_d4"
};

PLIST(aud_intbus_parents) = {
	"clk26m",
	"syspll1_d4",
	"syspll3_d2",
	"syspll4_d2",
	"univpll3_d2",
	"univpll2_d4"
};

PLIST(pmicspi_parents) = {
	"clk26m",
	"syspll1_d8",
	"syspll2_d4",
	"syspll4_d2",
	"syspll3_d4",
	"syspll2_d8",
	"syspll1_d16",
	"univpll3_d4",
	"univpll_d26",
	"dmpll_d2",
	"dmpll_d4"
};

PLIST(scp_parents)= {
	"clk26m",
	"syspll1_d8",
	"dmpll_d2",
	"dmpll_d4"
};

PLIST(dpi0_parents) = {
	"clk26m",
	"mipipll",
	"mipipll_d2",
	"mipipll_d4",
	"clk26m",
	"tvdpll_ck",
	"tvdpll_d2",
	"tvdpll_d4"
};

PLIST(dpi1_parents) = {
	"clk26m",
	"tvdpll_ck",
	"tvdpll_d2",
	"tvdpll_d4"
};

PLIST(tve_parents) = {
	"clk26m",
	"mipipll",
	"mipipll_d2",
	"mipipll_d4",
	"clk26m",
	"tvdpll_ck",
	"tvdpll_d2",
	"tvdpll_d4"
};
/*
PLIST(hdmi_parents) = {
	"clk26m",
	"hdmipll_ck",
	"hdmipll_d2",
	"hdmipll_d3"
};
*/
PLIST(apll_parents) = {
	"clk26m",
	"audpll",
	"audpll_d4",
	"audpll_d8",
	"audpll_d16",
	"audpll_d24",
	"clk26m",
	"clk26m"
};

PLIST(rtc_parents) = {
	"32k_internal",
	"32k_external",
	"clk26m",
	"univpll3_d8"
};

PLIST(nfi2x_parents) = {
	"clk26m",
	"syspll2_d2",
	"syspll_d7",
	"univpll3_d2",
	"syspll2_d4",
	"univpll3_d4",
	"syspll4_d4",
	"clk26m"
};

PLIST(emmc_hclk_parents) = {
	"clk26m",
	"syspll1_d2",
	"syspll1_d4",
	"syspll2_d2"
};

PLIST(flash_parents) = {
	"clk26m_d8",
	"clk26m",
	"syspll2_d8",
	"syspll3_d4",
	"univpll3_d4",
	"syspll4_d2",
	"syspll2_d4",
	"univpll2_d4"
};

PLIST(di_parents) = {
	"clk26m",
	"tvd2pll_ck",
	"tvd2pll_d2",
	"clk26m"
};

PLIST(nr_osd_parents) = {
	"clk26m",
	"vencpll_ck",
	"syspll1_d2",
	"syspll1_d4",
	"univpll_d5",
	"univpll1_d2",
	"univpll2_d2",
	"dmpll_ck"
};

PLIST(hdmirx_bist_parents) = {
	"clk26m",
	"syspll_d3",
	"clk26m",
	"syspll1_d16",
	"syspll4_d2",
	"syspll1_d4",
	"vencpll_ck",
	"clk26m"
};

PLIST(intdir_parents) = {
	"clk26m",
	"mmpll_ck",
	"syspll_d2",
	"univpll_d2"
};

PLIST(asm_parents) = {
	"clk26m",
	"univpll2_d4",
	"univpll2_d2",
	"syspll_d5"
};

PLIST(ms_card_parents) = {
	"clk26m",
	"univpll3_d8",
	"syspll4_d4"
};

PLIST(ethif_parents) = {
	"clk26m",
	"syspll1_d2",
	"syspll_d5",
	"syspll1_d4",
	"univpll_d5",
	"univpll1_d2",
	"msdcpll_d2",
	"mmpll_d2"
};

PLIST(hdmirx_parents) = {
	"clk26m",
	"univpll_d52"
};

PLIST(cmsys_parents) = {
	"clk26m",
	"syspll1_d2",
	"univpll1_d2",
	"univpll_d5",
	"syspll_d5",
	"syspll2_d2",
	"syspll1_d4",
	"syspll3_d2",
	"syspll2_d4",
	"syspll1_d8",
	"clk26m",
	"clk26m",
	"clk26m",
	"clk26m",
	"clk26m"
};

PLIST(clk_8bdac_parents) = {
	"32k_internal",
	"8bdac_ck",
	"clk26m",
	"clk26m"
};

PLIST(aud2dvd_parents) = {
	"a1sys_hp_ck",
	"a2sys_hp_ck"
};

PLIST(padmclk_parents) = {
	"clk26m",
	"univpll_d26",
	"univpll_d52",
	"univpll_d108",
	"univpll2_d8",
	"univpll2_d16",
	"univpll2_d32"
};

PLIST(aud_mux_parents) = {
	"clk26m",
	"aud1pll_98m_ck",
	"aud2pll_90m_ck",
	"hadds2pll_98m",
	"audio_ext1_ck",
	"audio_ext2_ck"
};

PLIST(aud_src_parents) = {
	"aud_mux1_sel",
	"aud_mux2_sel"
};

PLIST(cpu_parents) = {
	"clk26m",
	"armpll",
	"mainpll",
	"mmpll"
};

static struct clk_fixed_def fixed_clk[] = {
	FRATE(CLK_TOP_DPI, "dpi_ck", /*"clk26m",*/
	    108 * MHZ),
	FRATE(CLK_TOP_DMPLL, "dmpll_ck", /*"clk26m",*/
	    400 * MHZ),
	FRATE(CLK_TOP_VENCPLL, "vencpll_ck",/*"clk26m",*/
	    295750000),
	FRATE(CLK_TOP_HDMI_0_PIX340M, "hdmi_0_pix340m", /*"clk26m",*/
	    340 * MHZ),
	FRATE(CLK_TOP_HDMI_0_DEEP340M, "hdmi_0_deep340m", /*"clk26m",*/
	    340 * MHZ),
	FRATE(CLK_TOP_HDMI_0_PLL340M, "hdmi_0_pll340m", /*"clk26m",*/
	    340 * MHZ),
	FRATE(CLK_TOP_HADDS2_FB, "hadds2_fbclk", /*"clk26m",*/
	    27 * MHZ),
	FRATE(CLK_TOP_WBG_DIG_416M, "wbg_dig_ck_416m",/*"clk26m",*/
	    416 * MHZ),
	FRATE(CLK_TOP_DSI0_LNTC_DSI, "dsi0_lntc_dsi", /*"clk26m",*/
	    143 * MHZ),
	FRATE(CLK_TOP_HDMI_SCL_RX, "hdmi_scl_rx", /*"clk26m",*/
	    27 * MHZ),
	FRATE(CLK_TOP_AUD_EXT1, "aud_ext1", /*"clk26m",*/
	    0),
	FRATE(CLK_TOP_AUD_EXT2, "aud_ext2", /*"clk26m",*/
	    0),
	FRATE(CLK_TOP_NFI1X_PAD, "nfi1x_pad",/*"clk26m",*/
	    0),
	
	FFACT(CLK_TOP_SYSPLL, "syspll_ck", "mainpll", 1, 1),
	FFACT(CLK_TOP_SYSPLL_D2, "syspll_d2", "mainpll", 1, 2),
	FFACT(CLK_TOP_SYSPLL_D3, "syspll_d3", "mainpll", 1, 3),
	FFACT(CLK_TOP_SYSPLL_D5, "syspll_d5", "mainpll", 1, 5),
	FFACT(CLK_TOP_SYSPLL_D7, "syspll_d7", "mainpll", 1, 7),
	FFACT(CLK_TOP_SYSPLL1_D2, "syspll1_d2", "syspll_d2", 1, 2),
	FFACT(CLK_TOP_SYSPLL1_D4, "syspll1_d4", "syspll_d2", 1, 4),
	FFACT(CLK_TOP_SYSPLL1_D8, "syspll1_d8", "syspll_d2", 1, 8),
	FFACT(CLK_TOP_SYSPLL1_D16, "syspll1_d16", "syspll_d2", 1, 16),
	FFACT(CLK_TOP_SYSPLL2_D2, "syspll2_d2", "syspll_d3", 1, 2),
	FFACT(CLK_TOP_SYSPLL2_D4, "syspll2_d4", "syspll_d3", 1, 4),
	FFACT(CLK_TOP_SYSPLL2_D8, "syspll2_d8", "syspll_d3", 1, 8),
	FFACT(CLK_TOP_SYSPLL3_D2, "syspll3_d2", "syspll_d5", 1, 2),
	FFACT(CLK_TOP_SYSPLL3_D4, "syspll3_d4", "syspll_d5", 1, 4),
	FFACT(CLK_TOP_SYSPLL4_D2, "syspll4_d2", "syspll_d7", 1, 2),
	FFACT(CLK_TOP_SYSPLL4_D4, "syspll4_d4", "syspll_d7", 1, 4),

	FFACT(CLK_TOP_UNIVPLL, "univpll_ck", "univpll", 1, 1),
	FFACT(CLK_TOP_UNIVPLL_D2, "univpll_d2", "univpll", 1, 2),
	FFACT(CLK_TOP_UNIVPLL_D3, "univpll_d3", "univpll", 1, 3),
	FFACT(CLK_TOP_UNIVPLL_D5, "univpll_d5", "univpll", 1, 5),
	FFACT(CLK_TOP_UNIVPLL_D7, "univpll_d7", "univpll", 1, 7),
	FFACT(CLK_TOP_UNIVPLL_D26, "univpll_d26", "univpll", 1, 26),
	FFACT(CLK_TOP_UNIVPLL_D52, "univpll_d52", "univpll", 1, 52),
	FFACT(CLK_TOP_UNIVPLL_D108, "univpll_d108", "univpll", 1, 108),
	FFACT(CLK_TOP_USB_PHY48M, "usb_phy48m_ck", "univpll", 1, 26),
	FFACT(CLK_TOP_UNIVPLL1_D2, "univpll1_d2", "univpll_d2", 1, 2),
	FFACT(CLK_TOP_UNIVPLL1_D4, "univpll1_d4", "univpll_d2", 1, 4),
	FFACT(CLK_TOP_UNIVPLL1_D8, "univpll1_d8", "univpll_d2", 1, 8),
	FFACT(CLK_TOP_8BDAC, "8bdac_ck", "univpll_d2", 1, 1),
	FFACT(CLK_TOP_UNIVPLL2_D2, "univpll2_d2", "univpll_d3", 1, 2),
	FFACT(CLK_TOP_UNIVPLL2_D4, "univpll2_d4", "univpll_d3", 1, 4),
	FFACT(CLK_TOP_UNIVPLL2_D8, "univpll2_d8", "univpll_d3", 1, 8),
	FFACT(CLK_TOP_UNIVPLL2_D16, "univpll2_d16", "univpll_d3", 1, 16),
	FFACT(CLK_TOP_UNIVPLL2_D32, "univpll2_d32", "univpll_d3", 1, 32),
	FFACT(CLK_TOP_UNIVPLL3_D2, "univpll3_d2", "univpll_d5", 1, 2),
	FFACT(CLK_TOP_UNIVPLL3_D4, "univpll3_d4", "univpll_d5", 1, 4),
	FFACT(CLK_TOP_UNIVPLL3_D8, "univpll3_d8", "univpll_d5", 1, 8),

	FFACT(CLK_TOP_MSDCPLL, "msdcpll_ck", "msdcpll", 1, 1),
	FFACT(CLK_TOP_MSDCPLL_D2, "msdcpll_d2", "msdcpll", 1, 2),
	FFACT(CLK_TOP_MSDCPLL_D4, "msdcpll_d4", "msdcpll", 1, 4),
	FFACT(CLK_TOP_MSDCPLL_D8, "msdcpll_d8", "msdcpll", 1, 8),

	FFACT(CLK_TOP_MMPLL, "mmpll_ck", "mmpll", 1, 1),
	FFACT(CLK_TOP_MMPLL_D2, "mmpll_d2", "mmpll", 1, 2),

	FFACT(CLK_TOP_DMPLL_D2, "dmpll_d2", "dmpll_ck", 1, 2),
	FFACT(CLK_TOP_DMPLL_D4, "dmpll_d4", "dmpll_ck", 1, 4),
	FFACT(CLK_TOP_DMPLL_X2, "dmpll_x2", "dmpll_ck", 1, 1),

	FFACT(CLK_TOP_TVDPLL, "tvdpll_ck", "tvdpll", 1, 1),
	FFACT(CLK_TOP_TVDPLL_D2, "tvdpll_d2", "tvdpll", 1, 2),
	FFACT(CLK_TOP_TVDPLL_D4, "tvdpll_d4", "tvdpll", 1, 4),

	FFACT(CLK_TOP_VDECPLL, "vdecpll_ck", "vdecpll", 1, 1),
	FFACT(CLK_TOP_TVD2PLL, "tvd2pll_ck", "tvd2pll", 1, 1),
	FFACT(CLK_TOP_TVD2PLL_D2, "tvd2pll_d2", "tvd2pll", 1, 2),

	FFACT(CLK_TOP_MIPIPLL, "mipipll", "dpi_ck", 1, 1),
	FFACT(CLK_TOP_MIPIPLL_D2, "mipipll_d2", "dpi_ck", 1, 2),
	FFACT(CLK_TOP_MIPIPLL_D4, "mipipll_d4", "dpi_ck", 1, 4),

	/*FFACT(CLK_TOP_HDMIPLL, "hdmipll_ck", "hdmitx_dig_cts", 1, 1),
	FFACT(CLK_TOP_HDMIPLL_D2, "hdmipll_d2", "hdmitx_dig_cts", 1, 2),
	FFACT(CLK_TOP_HDMIPLL_D3, "hdmipll_d3", "hdmitx_dig_cts", 1, 3),*/

	FFACT(CLK_TOP_ARMPLL_1P3G, "armpll_1p3g_ck", "armpll", 1, 1),

	FFACT(CLK_TOP_AUDPLL, "audpll", "audpll_sel", 1, 1),
	FFACT(CLK_TOP_AUDPLL_D4, "audpll_d4", "audpll_sel", 1, 4),
	FFACT(CLK_TOP_AUDPLL_D8, "audpll_d8", "audpll_sel", 1, 8),
	FFACT(CLK_TOP_AUDPLL_D16, "audpll_d16", "audpll_sel", 1, 16),
	FFACT(CLK_TOP_AUDPLL_D24, "audpll_d24", "audpll_sel", 1, 24),

	FFACT(CLK_TOP_AUD1PLL_98M, "aud1pll_98m_ck", "aud1pll", 1, 3),
	FFACT(CLK_TOP_AUD2PLL_90M, "aud2pll_90m_ck", "aud2pll", 1, 3),
	FFACT(CLK_TOP_HADDS2PLL_98M, "hadds2pll_98m", "hadds2pll", 1, 3),
	FFACT(CLK_TOP_HADDS2PLL_294M, "hadds2pll_294m", "hadds2pll", 1, 1),
	FFACT(CLK_TOP_ETHPLL_500M, "ethpll_500m_ck", "ethpll", 1, 1),
	FFACT(CLK_TOP_CLK26M_D8, "clk26m_d8", "clk26m", 1, 8),
	FFACT(CLK_TOP_32K_INTERNAL, "32k_internal", "clk26m", 1, 793),
	FFACT(CLK_TOP_32K_EXTERNAL, "32k_external", "rtc32k", 1, 1),
	FFACT(CLK_TOP_AXISEL_D4, "axisel_d4", "axi_sel", 1, 4),
};

static struct clk_gate_def gates_clk[] = {
	GATE(CLK_TOP_AXI_SEL, "axi_sel", "axi_sel_mux", 0x0040, 7),
	GATE(CLK_TOP_MEM_SEL, "mem_sel", "mem_sel_mux", 0x0040, 15),
	GATE(CLK_TOP_DDRPHYCFG_SEL, "ddrphycfg_sel", "ddrphycfg_sel_mux", 0x0040, 23),
	GATE(CLK_TOP_MM_SEL, "mm_sel", "mm_sel_mux", 0x0040, 31),

	GATE(CLK_TOP_PWM_SEL, "pwm_sel", "pwm_sel_mux", 0x0050, 7),
	GATE(CLK_TOP_VDEC_SEL, "vdec_sel", "vdec_sel_mux", 0x0050, 15),
	GATE(CLK_TOP_MFG_SEL, "mfg_sel", "mfg_sel_mux", 0x0050, 23),
	GATE(CLK_TOP_CAMTG_SEL, "camtg_sel", "camtg_sel_mux", 0x0050, 31),

	GATE(CLK_TOP_UART_SEL, "uart_sel", "uart_sel_mux", 0x0060, 7),
	GATE(CLK_TOP_SPI0_SEL, "spi0_sel", "spi0_sel_mux", 0x0060, 15),
	GATE(CLK_TOP_USB20_SEL, "usb20_sel", "usb20_sel_mux", 0x0060, 23),
	GATE(CLK_TOP_MSDC30_0_SEL, "msdc30_0_sel", "msdc30_0_sel_mux", 0x0060, 31),

	GATE(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel", "msdc30_1_sel_mux", 0x0070, 7),
	GATE(CLK_TOP_MSDC30_2_SEL, "msdc30_2_sel", "msdc30_2_sel_mux", 0x0070, 15),
	GATE(CLK_TOP_AUDIO_SEL, "audio_sel", "audio_sel_mux", 0x0070, 23),
	GATE(CLK_TOP_AUDINTBUS_SEL, "aud_intbus_sel", "aud_intbus_sel_mux", 0x0070, 31),

	GATE(CLK_TOP_PMICSPI_SEL, "pmicspi_sel", "pmicspi_sel_mux", 0x0080, 7),
	GATE(CLK_TOP_SCP_SEL, "scp_sel", "scp_sel_mux", 0x0080, 15),
	GATE(CLK_TOP_DPI0_SEL, "dpi0_sel", "dpi0_sel_mux", 0x0080, 23),
	GATE(CLK_TOP_DPI1_SEL, "dpi1_sel", "dpi1_sel_mux", 0x0080, 31),

	GATE(CLK_TOP_TVE_SEL, "tve_sel", "tve_sel_mux", 0x0090, 7),
	//GATE(CLK_TOP_HDMI_SEL, "hdmi_sel", "hdmi_sel_mux", 0x0090, 15),
	GATE(CLK_TOP_APLL_SEL, "apll_sel", "apll_sel_mux", 0x0090, 23),

	GATE(CLK_TOP_RTC_SEL, "rtc_sel", "rtc_sel_mux", 0x00A0, 7),
	GATE(CLK_TOP_NFI2X_SEL, "nfi2x_sel", "nfi2x_sel_mux", 0x00A0, 15),
	GATE(CLK_TOP_EMMC_HCLK_SEL, "emmc_hclk_sel", "emmc_hclk_sel_mux", 0x00A0, 31),

	GATE(CLK_TOP_FLASH_SEL, "flash_sel", "flash_sel_mux", 0x00B0, 7),
	GATE(CLK_TOP_DI_SEL, "di_sel", "di_sel_mux", 0x00B0, 15),
	GATE(CLK_TOP_NR_SEL, "nr_sel", "nr_sel_mux", 0x00B0, 23),
	GATE(CLK_TOP_EMMC_HCLK_SEL, "osd_sel", "osd_sel_mux", 0x00B0, 31),

	GATE(CLK_TOP_HDMIRX_BIST_SEL, "hdmirx_bist_sel", "hdmirx_bist_sel_mux", 0x00C0, 7),
	GATE(CLK_TOP_INTDIR_SEL, "intdir_sel", "intdir_sel_mux", 0x00C0, 15),
	GATE(CLK_TOP_ASM_I_SEL, "asm_i_sel", "asm_i_sel_mux", 0x00C0, 23),
	GATE(CLK_TOP_ASM_M_SEL, "asm_m_sel", "asm_m_sel_mux", 0x00C0, 31),

	GATE(CLK_TOP_ASM_H_SEL, "asm_h_sel", "asm_h_sel_mux", 0x00D0, 7),
	GATE(CLK_TOP_MS_CARD_SEL, "ms_card_sel", "ms_card_sel_mux", 0x00D0, 23),
	GATE(CLK_TOP_ETHIF_SEL, "ethif_sel", "ethif_sel_mux", 0x00D0, 31),

	GATE(CLK_TOP_HDMIRX26_24_SEL, "hdmirx26_24_sel", "hdmirx26_24_sel_mux", 0x00E0, 7),
	GATE(CLK_TOP_MSDC30_3_SEL, "msdc30_3_sel", "msdc30_3_sel_mux", 0x00E0, 15),
	GATE(CLK_TOP_CMSYS_SEL, "cmsys_sel", "cmsys_sel_mux", 0x00E0, 23),
	GATE(CLK_TOP_SPI1_SEL, "spi2_sel", "spi2_sel_mux", 0x00E0, 31),

	GATE(CLK_TOP_SPI2_SEL, "spi1_sel", "spi1_sel_mux", 0x00F0, 7),
	GATE(CLK_TOP_8BDAC_SEL, "8bdac_sel", "8bdac_sel_mux", 0x00F0, 15),
	GATE(CLK_TOP_AUD2DVD_SEL, "aud2dvd_sel", "aud2dvd_sel_mux", 0x00F0, 23),

	GATE(CLK_TOP_AUD_K1_SRC_SEL, "aud_k1_src_sel", "aud_k1_src_sel_mux", 0x012c, 23),
	GATE(CLK_TOP_AUD_K2_SRC_SEL, "aud_k2_src_sel", "aud_k2_src_sel_mux", 0x012c, 24),
	GATE(CLK_TOP_AUD_K3_SRC_SEL, "aud_k3_src_sel", "aud_k3_src_sel_mux", 0x012c, 25),
	GATE(CLK_TOP_AUD_K4_SRC_SEL, "aud_k4_src_sel", "aud_k4_src_sel_mux", 0x012c, 26),
	GATE(CLK_TOP_AUD_K5_SRC_SEL, "aud_k5_src_sel", "aud_k5_src_sel_mux", 0x012c, 27),
	GATE(CLK_TOP_AUD_K6_SRC_SEL, "aud_k6_src_sel", "aud_k6_src_sel_mux", 0x012c, 28),

	GATE(CLK_TOP_AUD_48K_TIMING, "a1sys_hp_ck", "aud_mux1_div", 0x012C, 21),
	GATE(CLK_TOP_AUD_44K_TIMING, "a2sys_hp_ck", "aud_mux2_div", 0x012C, 22),
	GATE(CLK_TOP_AUD_I2S1_MCLK, "aud_i2s1_mclk", "aud_k1_src_div", 0x012C, 23),
	GATE(CLK_TOP_AUD_I2S2_MCLK, "aud_i2s2_mclk", "aud_k2_src_div", 0x012C, 24),
	GATE(CLK_TOP_AUD_I2S3_MCLK, "aud_i2s3_mclk", "aud_k3_src_div", 0x012C, 25),
	GATE(CLK_TOP_AUD_I2S4_MCLK, "aud_i2s4_mclk", "aud_k4_src_div", 0x012C, 26),
	GATE(CLK_TOP_AUD_I2S5_MCLK, "aud_i2s5_mclk", "aud_k5_src_div", 0x012C, 27),
	GATE(CLK_TOP_AUD_I2S6_MCLK, "aud_i2s6_mclk", "aud_k6_src_div", 0x012C, 28),
};

static struct clk_mux_def muxes_clk[] = {
	MUX0(CLK_INFRA_CPUSEL, "infra_cpu_sel", cpu_parents, 0x0000, 2, 2),

	MUX0(0, "axi_sel_mux", axi_parents, 0x0040, 0, 3),
	MUX0(0, "mem_sel_mux", mem_parents, 0x0040, 8, 1),
	MUX0(0, "ddrphycfg_sel_mux", ddrphycfg_parents, 0x0040, 16, 1),
	MUX0(0, "mm_sel_mux", mm_parents, 0x0040, 24, 3),

	MUX0(0, "pwm_sel_mux", pwm_parents, 0x0050, 0, 2),
	MUX0(0, "vdec_sel_mux", vdec_parents, 0x0050, 8, 4),
	MUX0(0, "mfg_sel_mux", mfg_parents, 0x0050, 16, 3),
	MUX0(0, "camtg_sel_mux", camtg_parents, 0x0050, 24, 3),

	MUX0(0, "uart_sel_mux", uart_parents, 0x0060, 0, 1),
	MUX0(0, "spi0_sel_mux", spi_parents, 0x0060, 8, 3),
	MUX0(0, "usb20_sel_mux", usb20_parents, 0x0060, 16, 2),
	MUX0(0, "msdc30_0_sel_mux", msdc30_parents, 0x0060, 24, 3),

	MUX0(0, "msdc30_1_sel_mux", msdc30_parents, 0x0070, 0, 3),
	MUX0(0, "msdc30_2_sel_mux", msdc30_parents, 0x0070, 8, 3),
	MUX0(0, "audio_sel_mux", msdc30_parents, 0x0070, 16, 1),
	MUX0(0, "aud_intbus_sel_mux", aud_intbus_parents, 0x0070, 24, 3),

	MUX0(0, "pmicspi_sel_mux", pmicspi_parents, 0x0080, 0, 4),
	MUX0(0, "scp_sel_mux", scp_parents, 0x0080, 8, 2),
	MUX0(0, "dpi0_sel_mux", dpi0_parents, 0x0080, 16, 3),
	MUX0(0, "dpi1_sel_mux", dpi1_parents, 0x0080, 24, 2),

	MUX0(0, "tve_sel_mux", tve_parents, 0x0090, 0, 3),
	//MUX0(0, "hdmi_sel_mux", hdmi_parents, 0x0090, 8, 2),
	MUX0(0, "apll_sel_mux", apll_parents, 0x0090, 16, 3),

	MUX0(0, "rtc_sel_mux", rtc_parents, 0x0090, 0, 2),
	MUX0(0, "nfi2x_sel_mux", nfi2x_parents, 0x0090, 8, 3),
	MUX0(0, "emmc_hclk_sel_mux", emmc_hclk_parents, 0x0090, 24, 2),

	MUX0(0, "flash_sel_mux", flash_parents, 0x00B0, 0, 3),
	MUX0(0, "di_sel_mux", di_parents, 0x00B0, 8, 3),
	MUX0(0, "nr_sel_mux", nr_osd_parents, 0x00B0, 16, 3),
	MUX0(0, "osd_sel_mux", nr_osd_parents, 0x00B0, 24, 3),

	MUX0(0, "hdmirx_bist_sel_mux", hdmirx_bist_parents, 0x00C0, 0, 3),
	MUX0(0, "intdir_sel_mux", intdir_parents, 0x00C0, 8, 2),
	MUX0(0, "asm_i_sel_mux", asm_parents, 0x00C0, 16, 3),
	MUX0(0, "asm_m_sel_mux", asm_parents, 0x00C0, 24, 3),

	MUX0(0, "asm_h_sel_mux", asm_parents, 0x00D0, 0, 2),
	MUX0(0, "ms_card_sel_mux", ms_card_parents, 0x00D0, 16, 2),
	MUX0(0, "ethif_sel_mux", ethif_parents, 0x00D0, 24, 3),

	MUX0(0, "hdmirx26_24_sel_mux", hdmirx_parents, 0x00D0, 0, 1),
	MUX0(0, "msdc30_3_sel_mux", msdc30_parents, 0x00E0, 8, 3),
	MUX0(0, "cmsys_sel_mux", cmsys_parents, 0x00E0, 16, 4),
	MUX0(0, "spi2_sel_mux", spi_parents, 0x00E0, 24, 3),

	MUX0(0, "spi1_sel_mux", spi_parents, 0x00F0, 0, 3),
	MUX0(0, "8bdac_sel_mux", clk_8bdac_parents, 0x00F0, 8, 2),
	MUX0(0, "aud2dvd_sel_mux", aud2dvd_parents, 0x00F0, 16, 1),

	/* only mux */
	MUX0(CLK_TOP_PADMCLK_SEL, "padmclk_sel", padmclk_parents,
	    0x0100, 0, 3),

	MUX0(CLK_TOP_AUD_MUX1_SEL, "aud_mux1_sel", aud_mux_parents,
	    0x012c, 0, 3),
	MUX0(CLK_TOP_AUD_MUX2_SEL, "aud_mux2_sel", aud_mux_parents,
	    0x012c, 3, 3),
	MUX0(CLK_TOP_AUDPLL_MUX_SEL, "audpll_sel", aud_mux_parents,
	    0x012c, 6, 3),

	MUX0(0, "aud_k1_src_sel_mux", aud_src_parents, 0x012c, 15, 1),
	MUX0(0, "aud_k2_src_sel_mux", aud_src_parents, 0x012c, 16, 1),
	MUX0(0, "aud_k3_src_sel_mux", aud_src_parents, 0x012c, 17, 1),
	MUX0(0, "aud_k4_src_sel_mux", aud_src_parents, 0x012c, 18, 1),
	MUX0(0, "aud_k5_src_sel_mux", aud_src_parents, 0x012c, 19, 1),
	MUX0(0, "aud_k6_src_sel_mux", aud_src_parents, 0x012c, 20, 1),
};

static struct clk_div_def dived_clk[] = {
	DIV(CLK_TOP_AUD_EXTCK1_DIV, "audio_ext1_ck", "aud_ext1",
	    0x0120, 0, 8),
	DIV(CLK_TOP_AUD_EXTCK2_DIV, "audio_ext2_ck", "aud_ext2",
	    0x0120, 8, 8),
	DIV(CLK_TOP_AUD_MUX1_DIV, "aud_mux1_div", "aud_mux1_sel",
	    0x0120, 16, 8),
	DIV(CLK_TOP_AUD_MUX2_DIV, "aud_mux2_div", "aud_mux2_sel",
	    0x0120, 24, 8),
	DIV(CLK_TOP_AUD_K1_SRC_DIV, "aud_k1_src_div", "aud_k1_src_sel",
	    0x0124, 0, 8),
	DIV(CLK_TOP_AUD_K2_SRC_DIV, "aud_k2_src_div", "aud_k2_src_sel",
	    0x0124, 8, 8),
	DIV(CLK_TOP_AUD_K3_SRC_DIV, "aud_k3_src_div", "aud_k3_src_sel",
	    0x0124, 16, 8),
	DIV(CLK_TOP_AUD_K4_SRC_DIV, "aud_k4_src_div", "aud_k4_src_sel",
	    0x0124, 24, 8),
	DIV(CLK_TOP_AUD_K5_SRC_DIV, "aud_k5_src_div", "aud_k5_src_sel",
	    0x0128, 0, 8),
	DIV(CLK_TOP_AUD_K6_SRC_DIV, "aud_k6_src_div", "aud_k6_src_sel",
	    0x0128, 8, 8),
};

static struct mdtk_clk_def clk_def = {
	.linked_def = NULL,
	.num_linked = 0,
	.fixed_def = fixed_clk,
	.num_fixed = nitems(fixed_clk),
	.gates_def = gates_clk,
	.num_gates = nitems(gates_clk),
	.dived_def = dived_clk,
	.num_dived = nitems(dived_clk),
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

static device_method_t mdtk_mt7623_topckgen_methods[] = {
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

DEFINE_CLASS_0(mt7623_topckgen, mdtk_mt7623_topckgen_driver, mdtk_mt7623_topckgen_methods,
    sizeof(struct mdtk_clk_softc));

EARLY_DRIVER_MODULE(mt7623_topckgen, simplebus, mdtk_mt7623_topckgen_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 2);
