/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * MediaTek MT7622 TOPCKGEN: fixed rate clocks, fixed dividers of the PLL
 * outputs and the top level clock muxes.
 *
 * The muxes live in CLK_CFG_n registers (0x10 apart), four 8-bit fields per
 * register.  Each field holds the selection in its low bits and a gate bit
 * (set means gated) in bit 7.  The registers are changed by
 * read-modify-write, a new selection takes effect at once.  The audio clock
 * selection muxes in CLK_AUDDIV_0 have no gate.
 *
 * The adjustable audio dividers (CLK_AUDDIV_n) and their power down gates
 * are not described yet: the common driver has no divider type.
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

#define	TOP_CLK_CFG_0		0x040
#define	TOP_CLK_CFG_1		0x050
#define	TOP_CLK_CFG_2		0x060
#define	TOP_CLK_CFG_3		0x070
#define	TOP_CLK_CFG_4		0x080
#define	TOP_CLK_CFG_5		0x090
#define	TOP_CLK_CFG_6		0x0a0
#define	TOP_CLK_CFG_7		0x0b0
#define	TOP_CLK_AUDDIV_0	0x120

#define	TOP_MUX_F(_id, _name, _parents, _reg, _shift, _width, _flags)	\
	{ .type = MTK_CLK_MUX, .mux = {					\
	    .clkdef = { .name = (_name), .id = (_id),			\
		.parent_names = (_parents),				\
		.parent_cnt = nitems(_parents), .flags = (_flags) },	\
	    .reg = (_reg), .shift = (_shift), .width = (_width),	\
	    .gate_bit = (_shift) + 7, .upd_bit = -1 } }
#define	TOP_MUX(_id, _name, _parents, _reg, _shift, _width)		\
	TOP_MUX_F(_id, _name, _parents, _reg, _shift, _width, 0)

/* Mux without a gate bit. */
#define	TOP_MUX0(_id, _name, _parents, _reg, _shift, _width)		\
	{ .type = MTK_CLK_MUX, .mux = {					\
	    .clkdef = { .name = (_name), .id = (_id),			\
		.parent_names = (_parents),				\
		.parent_cnt = nitems(_parents) },			\
	    .reg = (_reg), .shift = (_shift), .width = (_width),	\
	    .gate_bit = -1, .upd_bit = -1 } }

/* NULL marks a selection with no clock behind it. */
static const char *axi_parents[] = {
	"clkxtal", "syspll1_d2", "syspll_d5", "syspll1_d4", "univpll_d5",
	"univpll2_d2", "univpll_d7"
};
static const char *mem_parents[] = { "clkxtal", "dmpll_ck" };
static const char *ddrphycfg_parents[] = { "clkxtal", "syspll1_d8" };
static const char *eth_parents[] = {
	"clkxtal", "syspll1_d2", "univpll1_d2", "syspll1_d4", "univpll_d5",
	NULL, "univpll_d7"
};
static const char *pwm_parents[] = { "clkxtal", "univpll2_d4" };
static const char *f10m_ref_parents[] = { "clkxtal", "syspll4_d16" };
static const char *nfi_infra_parents[] = {
	"clkxtal", "clkxtal", "clkxtal", "clkxtal", "clkxtal", "clkxtal",
	"clkxtal", "clkxtal", "univpll2_d8", "syspll1_d8", "univpll1_d8",
	"syspll4_d2", "univpll2_d4", "univpll3_d2", "syspll1_d4"
};
static const char *flash_parents[] = {
	"clkxtal", "univpll_d80_d4", "syspll2_d8", "syspll3_d4", "univpll3_d4",
	"univpll1_d8", "syspll2_d4", "univpll2_d4"
};
static const char *uart_parents[] = { "clkxtal", "univpll2_d8" };
static const char *spi0_parents[] = {
	"clkxtal", "syspll3_d2", "clkxtal", "syspll2_d4", "syspll4_d2",
	"univpll2_d4", "univpll1_d8", "clkxtal"
};
static const char *spi1_parents[] = {
	"clkxtal", "syspll3_d2", "clkxtal", "syspll4_d4", "syspll4_d2",
	"univpll2_d4", "univpll1_d8", "clkxtal"
};
static const char *msdc30_0_parents[] = {
	"clkxtal", "univpll2_d16", "univ48m"
};
static const char *a1sys_hp_parents[] = {
	"clkxtal", "aud1pll_ck", "aud2pll_ck", "clkxtal"
};
static const char *intdir_parents[] = {
	"clkxtal", "syspll_d2", "univpll_d2", "sgmiipll_ck"
};
static const char *aud_intbus_parents[] = {
	"clkxtal", "syspll1_d4", "syspll4_d2", "syspll3_d2"
};
static const char *pmicspi_parents[] = {
	"clkxtal", NULL, NULL, NULL, NULL, "univpll2_d16"
};
static const char *atb_parents[] = { "clkxtal", "syspll1_d2", "syspll_d5" };
static const char *audio_parents[] = {
	"clkxtal", "syspll3_d4", "syspll4_d4", "univpll1_d16"
};
static const char *usb20_parents[] = {
	"clkxtal", "univpll3_d4", "syspll1_d8", "clkxtal"
};
static const char *aud1_parents[] = { "clkxtal", "aud1pll_ck" };
static const char *aud2_parents[] = { "clkxtal", "aud2pll_ck" };
static const char *asm_l_parents[] = {
	"clkxtal", "syspll_d5", "univpll2_d2", "univpll2_d4"
};
static const char *apll1_ck_parents[] = { "aud1_sel", "aud2_sel" };

static struct mtk_clk_def mt7622_topckgen_clks[] = {
	/* Inputs from the crystal and from APMIXEDSYS */
	MTK_LINK("clkxtal"),
	MTK_LINK("mainpll"),
	MTK_LINK("univ2pll"),
	MTK_LINK("eth1pll"),
	MTK_LINK("sgmipll"),
	MTK_LINK("aud1pll"),
	MTK_LINK("aud2pll"),

	/*
	 * Clocks generated outside TOPCKGEN, by the PHYs.  The rates are
	 * nominal, as in the Linux driver.
	 */
	MTK_FIXED(CLK_TOP_TO_U2_PHY,	"to_u2_phy",	31250000),
	MTK_FIXED(CLK_TOP_TO_U2_PHY_1P,	"to_u2_phy_1p",	31250000),
	MTK_FIXED(CLK_TOP_PCIE0_PIPE_EN, "pcie0_pipe_en", 125000000),
	MTK_FIXED(CLK_TOP_PCIE1_PIPE_EN, "pcie1_pipe_en", 125000000),
	MTK_FIXED(CLK_TOP_SSUSB_TX250M,	"ssusb_tx250m",	250000000),
	MTK_FIXED(CLK_TOP_SSUSB_EQ_RX250M, "ssusb_eq_rx250m", 250000000),
	MTK_FIXED(CLK_TOP_SSUSB_CDR_REF, "ssusb_cdr_ref", 33333333),
	MTK_FIXED(CLK_TOP_SSUSB_CDR_FB,	"ssusb_cdr_fb",	50000000),
	MTK_FIXED(CLK_TOP_SATA_ASIC,	"sata_asic",	50000000),
	MTK_FIXED(CLK_TOP_SATA_RBC,	"sata_rbc",	50000000),

	MTK_FACTOR(CLK_TOP_TO_USB3_SYS,	"to_usb3_sys",	"eth1pll", 1, 4),
	MTK_FACTOR(CLK_TOP_P1_1MHZ,	"p1_1mhz",	"eth1pll", 1, 500),
	MTK_FACTOR(CLK_TOP_4MHZ,	"free_run_4mhz", "eth1pll", 1, 125),
	MTK_FACTOR(CLK_TOP_P0_1MHZ,	"p0_1mhz",	"eth1pll", 1, 500),
	MTK_FACTOR(CLK_TOP_TXCLK_SRC_PRE, "txclk_src_pre", "sgmiipll_d2",
	    1, 1),
	MTK_FACTOR(CLK_TOP_RTC,		"rtc",		"clkxtal", 1, 1024),
	MTK_FACTOR(CLK_TOP_MEMPLL,	"mempll",	"clkxtal", 32, 1),
	MTK_FACTOR(CLK_TOP_DMPLL,	"dmpll_ck",	"mempll", 1, 1),
	MTK_FACTOR(CLK_TOP_SYSPLL_D2,	"syspll_d2",	"mainpll", 1, 2),
	MTK_FACTOR(CLK_TOP_SYSPLL1_D2,	"syspll1_d2",	"mainpll", 1, 4),
	MTK_FACTOR(CLK_TOP_SYSPLL1_D4,	"syspll1_d4",	"mainpll", 1, 8),
	MTK_FACTOR(CLK_TOP_SYSPLL1_D8,	"syspll1_d8",	"mainpll", 1, 16),
	MTK_FACTOR(CLK_TOP_SYSPLL2_D4,	"syspll2_d4",	"mainpll", 1, 12),
	MTK_FACTOR(CLK_TOP_SYSPLL2_D8,	"syspll2_d8",	"mainpll", 1, 24),
	MTK_FACTOR(CLK_TOP_SYSPLL_D5,	"syspll_d5",	"mainpll", 1, 5),
	MTK_FACTOR(CLK_TOP_SYSPLL3_D2,	"syspll3_d2",	"mainpll", 1, 10),
	MTK_FACTOR(CLK_TOP_SYSPLL3_D4,	"syspll3_d4",	"mainpll", 1, 20),
	MTK_FACTOR(CLK_TOP_SYSPLL4_D2,	"syspll4_d2",	"mainpll", 1, 14),
	MTK_FACTOR(CLK_TOP_SYSPLL4_D4,	"syspll4_d4",	"mainpll", 1, 28),
	MTK_FACTOR(CLK_TOP_SYSPLL4_D16,	"syspll4_d16",	"mainpll", 1, 112),
	MTK_FACTOR(CLK_TOP_UNIVPLL,	"univpll",	"univ2pll", 1, 2),
	MTK_FACTOR(CLK_TOP_UNIVPLL_D2,	"univpll_d2",	"univpll", 1, 2),
	MTK_FACTOR(CLK_TOP_UNIVPLL1_D2,	"univpll1_d2",	"univpll", 1, 4),
	MTK_FACTOR(CLK_TOP_UNIVPLL1_D4,	"univpll1_d4",	"univpll", 1, 8),
	MTK_FACTOR(CLK_TOP_UNIVPLL1_D8,	"univpll1_d8",	"univpll", 1, 16),
	MTK_FACTOR(CLK_TOP_UNIVPLL1_D16, "univpll1_d16", "univpll", 1, 32),
	MTK_FACTOR(CLK_TOP_UNIVPLL2_D2,	"univpll2_d2",	"univpll", 1, 6),
	MTK_FACTOR(CLK_TOP_UNIVPLL2_D4,	"univpll2_d4",	"univpll", 1, 12),
	MTK_FACTOR(CLK_TOP_UNIVPLL2_D8,	"univpll2_d8",	"univpll", 1, 24),
	MTK_FACTOR(CLK_TOP_UNIVPLL2_D16, "univpll2_d16", "univpll", 1, 48),
	MTK_FACTOR(CLK_TOP_UNIVPLL_D5,	"univpll_d5",	"univpll", 1, 5),
	MTK_FACTOR(CLK_TOP_UNIVPLL3_D2,	"univpll3_d2",	"univpll", 1, 10),
	MTK_FACTOR(CLK_TOP_UNIVPLL3_D4,	"univpll3_d4",	"univpll", 1, 20),
	MTK_FACTOR(CLK_TOP_UNIVPLL3_D16, "univpll3_d16", "univpll", 1, 80),
	MTK_FACTOR(CLK_TOP_UNIVPLL_D7,	"univpll_d7",	"univpll", 1, 7),
	MTK_FACTOR(CLK_TOP_UNIVPLL_D80_D4, "univpll_d80_d4", "univpll",
	    1, 320),
	MTK_FACTOR(CLK_TOP_UNIV48M,	"univ48m",	"univpll", 1, 25),
	MTK_FACTOR(CLK_TOP_SGMIIPLL,	"sgmiipll_ck",	"sgmipll", 1, 1),
	MTK_FACTOR(CLK_TOP_SGMIIPLL_D2,	"sgmiipll_d2",	"sgmipll", 1, 2),
	MTK_FACTOR(CLK_TOP_AUD1PLL,	"aud1pll_ck",	"aud1pll", 1, 1),
	MTK_FACTOR(CLK_TOP_AUD2PLL,	"aud2pll_ck",	"aud2pll", 1, 1),
	MTK_FACTOR(CLK_TOP_AUD_I2S2_MCK, "aud_i2s2_mck", "i2s2_mck_sel",
	    1, 2),
	MTK_FACTOR(CLK_TOP_TO_USB3_REF,	"to_usb3_ref",	"univpll2_d4", 1, 4),
	MTK_FACTOR(CLK_TOP_PCIE1_MAC_EN, "pcie1_mac_en", "univpll1_d4", 1, 1),
	MTK_FACTOR(CLK_TOP_PCIE0_MAC_EN, "pcie0_mac_en", "univpll1_d4", 1, 1),
	MTK_FACTOR(CLK_TOP_ETH_500M,	"eth_500m",	"eth1pll", 1, 1),

	/* CLK_CFG_0: the bus and DRAM clocks may never stop. */
	TOP_MUX_F(CLK_TOP_AXI_SEL, "axi_sel", axi_parents,
	    TOP_CLK_CFG_0, 0, 3, CLK_NODE_CANNOT_STOP),
	TOP_MUX_F(CLK_TOP_MEM_SEL, "mem_sel", mem_parents,
	    TOP_CLK_CFG_0, 8, 1, CLK_NODE_CANNOT_STOP),
	TOP_MUX_F(CLK_TOP_DDRPHYCFG_SEL, "ddrphycfg_sel", ddrphycfg_parents,
	    TOP_CLK_CFG_0, 16, 1, CLK_NODE_CANNOT_STOP),
	TOP_MUX(CLK_TOP_ETH_SEL, "eth_sel", eth_parents,
	    TOP_CLK_CFG_0, 24, 3),
	/* CLK_CFG_1 */
	TOP_MUX(CLK_TOP_PWM_SEL, "pwm_sel", pwm_parents,
	    TOP_CLK_CFG_1, 0, 2),
	TOP_MUX(CLK_TOP_F10M_REF_SEL, "f10m_ref_sel", f10m_ref_parents,
	    TOP_CLK_CFG_1, 8, 1),
	TOP_MUX(CLK_TOP_NFI_INFRA_SEL, "nfi_infra_sel", nfi_infra_parents,
	    TOP_CLK_CFG_1, 16, 4),
	TOP_MUX(CLK_TOP_FLASH_SEL, "flash_sel", flash_parents,
	    TOP_CLK_CFG_1, 24, 3),
	/* CLK_CFG_2 */
	TOP_MUX(CLK_TOP_UART_SEL, "uart_sel", uart_parents,
	    TOP_CLK_CFG_2, 0, 1),
	TOP_MUX(CLK_TOP_SPI0_SEL, "spi0_sel", spi0_parents,
	    TOP_CLK_CFG_2, 8, 3),
	TOP_MUX(CLK_TOP_SPI1_SEL, "spi1_sel", spi1_parents,
	    TOP_CLK_CFG_2, 16, 3),
	TOP_MUX(CLK_TOP_MSDC50_0_SEL, "msdc50_0_sel", uart_parents,
	    TOP_CLK_CFG_2, 24, 3),
	/* CLK_CFG_3 */
	TOP_MUX(CLK_TOP_MSDC30_0_SEL, "msdc30_0_sel", msdc30_0_parents,
	    TOP_CLK_CFG_3, 0, 3),
	TOP_MUX(CLK_TOP_MSDC30_1_SEL, "msdc30_1_sel", msdc30_0_parents,
	    TOP_CLK_CFG_3, 8, 3),
	TOP_MUX(CLK_TOP_A1SYS_HP_SEL, "a1sys_hp_sel", a1sys_hp_parents,
	    TOP_CLK_CFG_3, 16, 2),
	TOP_MUX(CLK_TOP_A2SYS_HP_SEL, "a2sys_hp_sel", a1sys_hp_parents,
	    TOP_CLK_CFG_3, 24, 2),
	/* CLK_CFG_4 */
	TOP_MUX(CLK_TOP_INTDIR_SEL, "intdir_sel", intdir_parents,
	    TOP_CLK_CFG_4, 0, 2),
	TOP_MUX(CLK_TOP_AUD_INTBUS_SEL, "aud_intbus_sel", aud_intbus_parents,
	    TOP_CLK_CFG_4, 8, 2),
	TOP_MUX(CLK_TOP_PMICSPI_SEL, "pmicspi_sel", pmicspi_parents,
	    TOP_CLK_CFG_4, 16, 3),
	TOP_MUX(CLK_TOP_SCP_SEL, "scp_sel", ddrphycfg_parents,
	    TOP_CLK_CFG_4, 24, 2),
	/* CLK_CFG_5 */
	TOP_MUX(CLK_TOP_ATB_SEL, "atb_sel", atb_parents,
	    TOP_CLK_CFG_5, 0, 2),
	TOP_MUX(CLK_TOP_HIF_SEL, "hif_sel", eth_parents,
	    TOP_CLK_CFG_5, 8, 3),
	TOP_MUX(CLK_TOP_AUDIO_SEL, "audio_sel", audio_parents,
	    TOP_CLK_CFG_5, 16, 2),
	TOP_MUX(CLK_TOP_U2_SEL, "usb20_sel", usb20_parents,
	    TOP_CLK_CFG_5, 24, 2),
	/* CLK_CFG_6 */
	TOP_MUX(CLK_TOP_AUD1_SEL, "aud1_sel", aud1_parents,
	    TOP_CLK_CFG_6, 0, 1),
	TOP_MUX(CLK_TOP_AUD2_SEL, "aud2_sel", aud2_parents,
	    TOP_CLK_CFG_6, 8, 1),
	TOP_MUX(CLK_TOP_IRRX_SEL, "irrx_sel", f10m_ref_parents,
	    TOP_CLK_CFG_6, 16, 1),
	TOP_MUX(CLK_TOP_IRTX_SEL, "irtx_sel", f10m_ref_parents,
	    TOP_CLK_CFG_6, 24, 1),
	/* CLK_CFG_7 */
	TOP_MUX(CLK_TOP_ASM_L_SEL, "asm_l_sel", asm_l_parents,
	    TOP_CLK_CFG_7, 0, 2),
	TOP_MUX(CLK_TOP_ASM_M_SEL, "asm_m_sel", asm_l_parents,
	    TOP_CLK_CFG_7, 8, 2),
	TOP_MUX(CLK_TOP_ASM_H_SEL, "asm_h_sel", asm_l_parents,
	    TOP_CLK_CFG_7, 16, 2),
	/* CLK_AUDDIV_0 */
	TOP_MUX0(CLK_TOP_APLL1_SEL, "apll1_ck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 6, 1),
	TOP_MUX0(CLK_TOP_APLL2_SEL, "apll2_ck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 7, 1),
	TOP_MUX0(CLK_TOP_I2S0_MCK_SEL, "i2s0_mck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 8, 1),
	TOP_MUX0(CLK_TOP_I2S1_MCK_SEL, "i2s1_mck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 9, 1),
	TOP_MUX0(CLK_TOP_I2S2_MCK_SEL, "i2s2_mck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 10, 1),
	TOP_MUX0(CLK_TOP_I2S3_MCK_SEL, "i2s3_mck_sel", apll1_ck_parents,
	    TOP_CLK_AUDDIV_0, 11, 1),
};

static const struct mtk_clk_desc mt7622_topckgen_desc = {
	.clks = mt7622_topckgen_clks,
	.nclks = nitems(mt7622_topckgen_clks),
};

static struct ofw_compat_data compat_data[] = {
	{ "mediatek,mt7622-topckgen",	1 },
	{ NULL,				0 }
};

static int
mt7622_topckgen_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "MediaTek MT7622 top clock generator");
	return (BUS_PROBE_DEFAULT);
}

static int
mt7622_topckgen_attach(device_t dev)
{
	return (mtk_clk_attach(dev, &mt7622_topckgen_desc));
}

static device_method_t mt7622_topckgen_methods[] = {
	DEVMETHOD(device_probe,		mt7622_topckgen_probe),
	DEVMETHOD(device_attach,	mt7622_topckgen_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(mt7622_topckgen, mt7622_topckgen_driver,
    mt7622_topckgen_methods, sizeof(struct mtk_clk_softc), mtk_clk_driver);

/* After APMIXEDSYS, which provides the PLLs. */
EARLY_DRIVER_MODULE(mt7622_topckgen, simplebus, mt7622_topckgen_driver,
    NULL, NULL, BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 2);
