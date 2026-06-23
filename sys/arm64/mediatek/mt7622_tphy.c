/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Martin Filla
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
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/clk/clk.h>
#include <dev/phy/phy.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/fdt_common.h>
#include <dt-bindings/phy/phy.h>
#include "phydev_if.h"
#include "phynode_if.h"

#define	U3P_USBPHYACR0		0x000
#define	  PA0_RG_USB20_INTR_EN		(1U << 5)
#define	U3P_USBPHYACR5		0x014
#define	  PA5_RG_U2_HS_100U_U3_EN	(1U << 11)
#define	U3P_USBPHYACR6		0x018
#define	  PA6_RG_U2_BC11_SW_EN		(1U << 23)
#define	  PA6_RG_U2_SQTH_MASK		0x0000000f
#define	  PA6_RG_U2_SQTH_SHIFT		0
#define	U3P_U2PHYACR4		0x020
#define	  P2C_RG_USB20_GPIO_CTL		(1U << 9)
#define	  P2C_USB20_GPIO_MODE		(1U << 8)
#define	  P2C_U2_GPIO_CTR_MSK	(P2C_RG_USB20_GPIO_CTL | P2C_USB20_GPIO_MODE)
#define	U3P_U2PHYDTM0		0x068
#define	  P2C_FORCE_DATAIN		(1U << 23)
#define	  P2C_FORCE_DM_PULLDOWN		(1U << 21)
#define	  P2C_FORCE_DP_PULLDOWN		(1U << 20)
#define	  P2C_FORCE_XCVRSEL		(1U << 19)
#define	  P2C_FORCE_SUSPENDM		(1U << 18)
#define	  P2C_FORCE_TERMSEL		(1U << 17)
#define	  P2C_FORCE_UART_EN		(1U << 26)
#define	  P2C_RG_DATAIN			((3U << 10))
#define	  P2C_RG_DMPULLDOWN		(1U << 7)
#define	  P2C_RG_DPPULLDOWN		(1U << 6)
#define	  P2C_RG_XCVRSEL		((3U << 4))
#define	  P2C_RG_SUSPENDM		(1U << 3)
#define	  P2C_RG_TERMSEL		(1U << 2)
#define	  P2C_DTM0_PART_MASK	(P2C_FORCE_DATAIN | P2C_FORCE_DM_PULLDOWN | \
				 P2C_FORCE_DP_PULLDOWN | P2C_FORCE_XCVRSEL | \
				 P2C_FORCE_TERMSEL | P2C_RG_DMPULLDOWN | \
				 P2C_RG_DPPULLDOWN | P2C_RG_TERMSEL)
#define	U3P_U2PHYDTM1		0x06c
#define	  P2C_RG_UART_EN		(1U << 16)
#define	  PA6_RG_U2_OTG_VBUSCMP_EN	(1U << 20)   /* v U3P_USBPHYACR6 */
#define	  P2C_FORCE_IDDIG		(1U << 9)    /* v U3P_U2PHYDTM1 */
#define	  P2C_RG_VBUSVALID		(1U << 5)
#define	  P2C_RG_SESSEND		(1U << 4)
#define	  P2C_RG_AVALID			(1U << 2)
#define	  P2C_RG_IDDIG			(1U << 1)

/* ---- USB3 / PCIe --------------------------------------------------------- */
#define	U3P_U3_CHIP_GPIO_CTLD	0x0c
#define	  P3C_MCU_BUS_CK_GATE_EN		(1U << 30)
#define	  P3C_FORCE_IP_SW_RST		(1U << 29)
#define	U3P_U3_CHIP_GPIO_CTLE	0x10
#define	  P3C_RG_SWRST_U3_PHYD		(1U << 25)
#define	  P3C_RG_SWRST_U3_PHYD_FORCE_EN	(1U << 24)
#define	U3P_SPLLC_XTALCTL3	0x018
#define	  XC3_RG_U3_XTAL_RX_PWD		(1U << 9)
#define	  XC3_RG_U3_FRC_XTAL_RX_PWD	(1U << 8)
#define	U3P_U3_PHYA_DA_REG0	0x100
#define	  P3A_RG_XTAL_EXT_EN_U3_MASK	0x00000c00
#define	  P3A_RG_XTAL_EXT_EN_U3_SHIFT	10
#define	U3P_U3_PHYA_REG9	0x024
#define	  P3A_RG_RX_DAC_MUX_MASK		0x0000003e
#define	  P3A_RG_RX_DAC_MUX_SHIFT	1
#define	U3P_U3_PHYA_REG6	0x018
#define	  P3A_RG_TX_EIDLE_CM_MASK	0xf0000000
#define	  P3A_RG_TX_EIDLE_CM_SHIFT	28
#define	U3P_U3_PHYD_CDR1	0x05c
#define	  P3D_RG_CDR_BIR_LTD1_MASK	0x1f000000
#define	  P3D_RG_CDR_BIR_LTD1_SHIFT	24
#define	  P3D_RG_CDR_BIR_LTD0_MASK	0x00001f00
#define	  P3D_RG_CDR_BIR_LTD0_SHIFT	8
#define	U3P_U3_PHYD_LFPS1	0x00c
#define	  P3D_RG_FWAKE_TH_MASK		0x003f0000
#define	  P3D_RG_FWAKE_TH_SHIFT		16
#define	U3P_U3_PHYD_RXDET1	0x128
#define	  P3D_RG_RXDET_STB2_SET_MASK	0x0003fe00
#define	  P3D_RG_RXDET_STB2_SET_SHIFT	9
#define	U3P_U3_PHYD_RXDET2	0x12c
#define	  P3D_RG_RXDET_STB2_SET_P3_MASK	0x000001ff
#define	  P3D_RG_RXDET_STB2_SET_P3_SHIFT	0

/* ---- SATA (offsets relative to the SATA port's PHYD bank) ---------------- */
#define	ANA_RG_CTRL_SIGNAL1	0x4c
#define	  RG_IDRV_0DB_GEN1_MASK		0x00003f00
#define	  RG_IDRV_0DB_GEN1_SHIFT		8
#define	ANA_RG_CTRL_SIGNAL4	0x58
#define	  RG_CDR_BICLTR_GEN1_MASK	0x00f00000
#define	  RG_CDR_BICLTR_GEN1_SHIFT	20
#define	  RG_CDR_BR_GEN2_MASK		0x00000700
#define	  RG_CDR_BR_GEN2_SHIFT		8
#define	ANA_RG_CTRL_SIGNAL6	0x60
#define	  RG_CDR_BC_GEN1_MASK		0x1f000000
#define	  RG_CDR_BC_GEN1_SHIFT		24
#define	  RG_CDR_BIRLTR_GEN1_MASK	0x0000001f
#define	  RG_CDR_BIRLTR_GEN1_SHIFT	0
#define	ANA_EQ_EYE_CTRL_SIGNAL1	0x6c
#define	  RG_EQ_DLEQ_LFI_GEN1_MASK	0x00000f00
#define	  RG_EQ_DLEQ_LFI_GEN1_SHIFT	8
#define	ANA_EQ_EYE_CTRL_SIGNAL4	0xd8
#define	  RG_CDR_BIRLTD0_GEN1_MASK	0x001f0000
#define	  RG_CDR_BIRLTD0_GEN1_SHIFT	16
#define	ANA_EQ_EYE_CTRL_SIGNAL5	0xdc
#define	  RG_CDR_BIRLTD0_GEN3_MASK	0x0000001f
#define	  RG_CDR_BIRLTD0_GEN3_SHIFT	0
#define	PHYD_CTRL_SIGNAL_MODE4	0x1c
#define	  RG_CDR_BICLTD1_GEN1_MASK	0x00f00000
#define	  RG_CDR_BICLTD1_GEN1_SHIFT	20
#define	  RG_CDR_BICLTD0_GEN1_MASK	0x00000f00
#define	  RG_CDR_BICLTD0_GEN1_SHIFT	8
#define	PHYD_DESIGN_OPTION2	0x24
#define	  RG_LOCK_CNT_SEL_MASK		0x00000030
#define	  RG_LOCK_CNT_SEL_SHIFT		4
#define	PHYD_DESIGN_OPTION9	0x40
#define	  RG_TG_MAX_MASK		0x001f0000
#define	  RG_TG_MAX_SHIFT		16
#define	  RG_T2_MAX_MASK		0x00003f00
#define	  RG_T2_MAX_SHIFT		8
#define	  RG_TG_MIN_MASK		0x000000e0
#define	  RG_TG_MIN_SHIFT		5
#define	  RG_T2_MIN_MASK		0x0000001f
#define	  RG_T2_MIN_SHIFT		0

#define	MT7622_TPHY_MAX_PORTS	4

struct mt_phynode_softc {
	device_t		dev;
	struct resource		*res;
	clk_t			ref_clk;
	phandle_t		node;
	int			rid;
	int			mode;
	int			port_id;
};

struct mt_tphy_softc {
	device_t		dev;
	phandle_t		node;
	struct resource		*mem_res;
	int			mem_rid;
	struct mt_phynode_softc	ports[MT7622_TPHY_MAX_PORTS];
	int			nports;
};

static struct ofw_compat_data compat_data[] = {
		{"mediatek,mt7622-tphy",	1},
		{NULL,				0}
};

static int mt7622_tphy_detach(device_t dev);

#define	RD4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

/* reg = (reg & ~mask) | ((val << shift) & mask) */
static inline uint32_t
field_set(uint32_t reg, uint32_t val, uint32_t shift, uint32_t mask)
{
return ((reg & ~mask) | ((val << shift) & mask));
}

static void
mt_phy_sata_init(struct mt_phynode_softc *sc)
{
	uint32_t tmp;

	/* Charge current / CDR tuning - values taken from phy-mtk-tphy. */
	tmp = RD4(sc, ANA_RG_CTRL_SIGNAL6);
	tmp = field_set(tmp, 0x6, RG_CDR_BIRLTR_GEN1_SHIFT,
	                RG_CDR_BIRLTR_GEN1_MASK);
	tmp = field_set(tmp, 0x1a, RG_CDR_BC_GEN1_SHIFT, RG_CDR_BC_GEN1_MASK);
	WR4(sc, ANA_RG_CTRL_SIGNAL6, tmp);

	tmp = RD4(sc, ANA_EQ_EYE_CTRL_SIGNAL4);
	tmp = field_set(tmp, 0x18, RG_CDR_BIRLTD0_GEN1_SHIFT,
	                RG_CDR_BIRLTD0_GEN1_MASK);
	WR4(sc, ANA_EQ_EYE_CTRL_SIGNAL4, tmp);

	tmp = RD4(sc, ANA_EQ_EYE_CTRL_SIGNAL5);
	tmp = field_set(tmp, 0x06, RG_CDR_BIRLTD0_GEN3_SHIFT,
	                RG_CDR_BIRLTD0_GEN3_MASK);
	WR4(sc, ANA_EQ_EYE_CTRL_SIGNAL5, tmp);

	tmp = RD4(sc, ANA_RG_CTRL_SIGNAL4);
	tmp = field_set(tmp, 0x0c, RG_CDR_BICLTR_GEN1_SHIFT,
	                RG_CDR_BICLTR_GEN1_MASK);
	tmp = field_set(tmp, 0x07, RG_CDR_BR_GEN2_SHIFT, RG_CDR_BR_GEN2_MASK);
	WR4(sc, ANA_RG_CTRL_SIGNAL4, tmp);

	tmp = RD4(sc, PHYD_CTRL_SIGNAL_MODE4);
	tmp = field_set(tmp, 0x08, RG_CDR_BICLTD0_GEN1_SHIFT,
	                RG_CDR_BICLTD0_GEN1_MASK);
	tmp = field_set(tmp, 0x02, RG_CDR_BICLTD1_GEN1_SHIFT,
	                RG_CDR_BICLTD1_GEN1_MASK);
	WR4(sc, PHYD_CTRL_SIGNAL_MODE4, tmp);

	tmp = RD4(sc, PHYD_DESIGN_OPTION2);
	tmp = field_set(tmp, 0x02, RG_LOCK_CNT_SEL_SHIFT, RG_LOCK_CNT_SEL_MASK);
	WR4(sc, PHYD_DESIGN_OPTION2, tmp);

	tmp = RD4(sc, PHYD_DESIGN_OPTION9);
	tmp = field_set(tmp, 0x12, RG_T2_MIN_SHIFT, RG_T2_MIN_MASK);
	tmp = field_set(tmp, 0x04, RG_TG_MIN_SHIFT, RG_TG_MIN_MASK);
	WR4(sc, PHYD_DESIGN_OPTION9, tmp);

	tmp = RD4(sc, PHYD_DESIGN_OPTION9);
	tmp = field_set(tmp, 0x31, RG_T2_MAX_SHIFT, RG_T2_MAX_MASK);
	tmp = field_set(tmp, 0x0e, RG_TG_MAX_SHIFT, RG_TG_MAX_MASK);
	WR4(sc, PHYD_DESIGN_OPTION9, tmp);

	tmp = RD4(sc, ANA_RG_CTRL_SIGNAL1);
	tmp = field_set(tmp, 0x20, RG_IDRV_0DB_GEN1_SHIFT, RG_IDRV_0DB_GEN1_MASK);
	WR4(sc, ANA_RG_CTRL_SIGNAL1, tmp);

	tmp = RD4(sc, ANA_EQ_EYE_CTRL_SIGNAL1);
	tmp = field_set(tmp, 0x03, RG_EQ_DLEQ_LFI_GEN1_SHIFT,
	                RG_EQ_DLEQ_LFI_GEN1_MASK);
	WR4(sc, ANA_EQ_EYE_CTRL_SIGNAL1, tmp);
}

static void
mt_phy_usb2_init(struct mt_phynode_softc *sc)
{
	uint32_t tmp;

	tmp = RD4(sc, U3P_U2PHYDTM0);
	tmp &= ~(P2C_FORCE_UART_EN | P2C_FORCE_SUSPENDM);
	WR4(sc, U3P_U2PHYDTM0, tmp);

	tmp = RD4(sc, U3P_U2PHYDTM0);
	tmp &= ~(P2C_RG_XCVRSEL | P2C_RG_DATAIN | P2C_DTM0_PART_MASK);
	WR4(sc, U3P_U2PHYDTM0, tmp);

	tmp = RD4(sc, U3P_U2PHYDTM1);
	tmp &= ~P2C_RG_UART_EN;
	WR4(sc, U3P_U2PHYDTM1, tmp);

	tmp = RD4(sc, U3P_USBPHYACR0);
	tmp |= PA0_RG_USB20_INTR_EN;
	WR4(sc, U3P_USBPHYACR0, tmp);

	tmp = RD4(sc, U3P_USBPHYACR5);
	tmp &= ~PA5_RG_U2_HS_100U_U3_EN;
	WR4(sc, U3P_USBPHYACR5, tmp);

	tmp = RD4(sc, U3P_U2PHYACR4);
	tmp &= ~P2C_U2_GPIO_CTR_MSK;
	WR4(sc, U3P_U2PHYACR4, tmp);

	tmp = RD4(sc, U3P_USBPHYACR6);
	tmp &= ~PA6_RG_U2_BC11_SW_EN;
	WR4(sc, U3P_USBPHYACR6, tmp);

	tmp = RD4(sc, U3P_USBPHYACR6);
	tmp = field_set(tmp, 0x2, PA6_RG_U2_SQTH_SHIFT, PA6_RG_U2_SQTH_MASK);
	WR4(sc, U3P_USBPHYACR6, tmp);

    /* OTG VBUS comparator on */
    tmp = RD4(sc, U3P_USBPHYACR6);
    tmp |= PA6_RG_U2_OTG_VBUSCMP_EN;
    WR4(sc, U3P_USBPHYACR6, tmp);

    /* VBUS/session valid, session NOT ended -> host sees a device */
    tmp = RD4(sc, U3P_U2PHYDTM1);
    tmp |= (P2C_RG_VBUSVALID | P2C_RG_AVALID);
    tmp &= ~P2C_RG_SESSEND;
    WR4(sc, U3P_U2PHYDTM1, tmp);
}

static void
mt_phy_usb3_init(struct mt_phynode_softc *sc)
{
	uint32_t tmp;

	tmp = RD4(sc, U3P_U3_CHIP_GPIO_CTLD);
	tmp |= (P3C_FORCE_IP_SW_RST | P3C_MCU_BUS_CK_GATE_EN);
	WR4(sc, U3P_U3_CHIP_GPIO_CTLD, tmp);

	tmp = RD4(sc, U3P_U3_CHIP_GPIO_CTLE);
	tmp |= (P3C_RG_SWRST_U3_PHYD | P3C_RG_SWRST_U3_PHYD_FORCE_EN);
	WR4(sc, U3P_U3_CHIP_GPIO_CTLE, tmp);

	tmp = RD4(sc, U3P_SPLLC_XTALCTL3);
	tmp |= (XC3_RG_U3_XTAL_RX_PWD | XC3_RG_U3_FRC_XTAL_RX_PWD);
	WR4(sc, U3P_SPLLC_XTALCTL3, tmp);

	tmp = RD4(sc, U3P_U3_PHYA_DA_REG0);
	tmp = field_set(tmp, 2, P3A_RG_XTAL_EXT_EN_U3_SHIFT,
	                P3A_RG_XTAL_EXT_EN_U3_MASK);
	WR4(sc, U3P_U3_PHYA_DA_REG0, tmp);

	tmp = RD4(sc, U3P_U3_PHYA_REG9);
	tmp = field_set(tmp, 4, P3A_RG_RX_DAC_MUX_SHIFT, P3A_RG_RX_DAC_MUX_MASK);
	WR4(sc, U3P_U3_PHYA_REG9, tmp);

	tmp = RD4(sc, U3P_U3_PHYA_REG6);
	tmp = field_set(tmp, 0xe, P3A_RG_TX_EIDLE_CM_SHIFT,
	                P3A_RG_TX_EIDLE_CM_MASK);
	WR4(sc, U3P_U3_PHYA_REG6, tmp);

	tmp = RD4(sc, U3P_U3_PHYD_CDR1);
	tmp = field_set(tmp, 0xc, P3D_RG_CDR_BIR_LTD0_SHIFT,
	                P3D_RG_CDR_BIR_LTD0_MASK);
	tmp = field_set(tmp, 0x3, P3D_RG_CDR_BIR_LTD1_SHIFT,
	                P3D_RG_CDR_BIR_LTD1_MASK);
	WR4(sc, U3P_U3_PHYD_CDR1, tmp);

	tmp = RD4(sc, U3P_U3_PHYD_LFPS1);
	tmp = field_set(tmp, 0x34, P3D_RG_FWAKE_TH_SHIFT, P3D_RG_FWAKE_TH_MASK);
	WR4(sc, U3P_U3_PHYD_LFPS1, tmp);

	tmp = RD4(sc, U3P_U3_PHYD_RXDET1);
	tmp = field_set(tmp, 0x10, P3D_RG_RXDET_STB2_SET_SHIFT,
	                P3D_RG_RXDET_STB2_SET_MASK);
	WR4(sc, U3P_U3_PHYD_RXDET1, tmp);

	tmp = RD4(sc, U3P_U3_PHYD_RXDET2);
	tmp = field_set(tmp, 0x10, P3D_RG_RXDET_STB2_SET_P3_SHIFT,
	                P3D_RG_RXDET_STB2_SET_P3_MASK);
	WR4(sc, U3P_U3_PHYD_RXDET2, tmp);

    /* Power on: enable SuperSpeed RX detection */
    tmp = RD4(sc, U3P_U3_PHYD_RXDET1);
    tmp &= ~P3D_RG_RXDET_STB2_SET_MASK;  
    WR4(sc, U3P_U3_PHYD_RXDET1, tmp);

    tmp = RD4(sc, U3P_U3_PHYD_RXDET2);
    tmp &= ~P3D_RG_RXDET_STB2_SET_P3_MASK;
    WR4(sc, U3P_U3_PHYD_RXDET2, tmp);
}

static int
mt_phynode_enable(struct phynode *phynode, bool enable)
{
	struct mt_phynode_softc *sc;
	struct mt_tphy_softc *tphy_sc;
	device_t dev;
	phandle_t node;
	int mode = PHY_NONE;

	sc = phynode_get_softc(phynode);
	dev = phynode_get_device(phynode);
	node = phynode_get_ofw_node(phynode);
	tphy_sc = device_get_softc(dev);

	for (int i = 0; i < tphy_sc->nports; i++) {
		if (tphy_sc->ports[i].node == node) {
			mode = tphy_sc->ports[i].mode;
			break;
		}
	}

	if (sc->res == NULL) {
		device_printf(dev, "port %d: mem_res is NULL\n", sc->port_id);
		return (ENXIO);
	}

	if (!enable)
		return (0);

	switch (mode) {
		case PHY_TYPE_USB2:
			device_printf(dev, "port %d: init USB2 PHY\n", sc->port_id);
			mt_phy_usb2_init(sc);
			break;
		case PHY_TYPE_USB3:
			device_printf(dev, "port %d: init USB3 PHY\n", sc->port_id);
			mt_phy_usb3_init(sc);
			break;
		case PHY_TYPE_PCIE:
			device_printf(dev, "port %d: PCIe PHY (no extra init)\n",
			              sc->port_id);
			break;
		case PHY_TYPE_SATA:
			device_printf(dev, "port %d: init SATA PHY\n", sc->port_id);
			mt_phy_sata_init(sc);
			break;
		default:
			device_printf(dev, "port %d: unsupported PHY mode %d\n",
			              sc->port_id, mode);
			return (EINVAL);
	}

	return (0);
}

/* Phy controller class and methods. */
static phynode_method_t mt_phynode_methods[] = {
		PHYNODEMETHOD(phynode_enable,	mt_phynode_enable),
		PHYNODEMETHOD_END
};
DEFINE_CLASS_1(mt_phynode, mt_phynode_class, mt_phynode_methods,
sizeof(struct mt_phynode_softc), phynode_class);

static int
mt7622_tphy_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Mediatek T-PHY driver");
	return (BUS_PROBE_DEFAULT);
}

static int
mt7622_tphy_attach(device_t dev)
{
	struct mt_tphy_softc *sc;
	struct phynode_init_def phy_init;
	struct phynode *phynode;
	struct mt_phynode_softc *phynode_sc;
	phandle_t child;
	clk_t clk;
	uint64_t freq;
	int phy_rid, phy_id;
	char name[64];
	u_long start, size;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);
	sc->nports = 0;

	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	                                     RF_ACTIVE);
	phy_id = 0;
	phy_rid = 1;
	for (child = OF_child(sc->node); child != 0; child = OF_peer(child)) {
		if (phy_id >= MT7622_TPHY_MAX_PORTS) {
			device_printf(dev, "too many PHY ports, ignoring rest\n");
			break;
		}
		if (OF_getprop(child, "name", name, sizeof(name)) <= 0)
			continue;

		sc->ports[phy_id].dev = dev;
		sc->ports[phy_id].port_id = phy_id;
		sc->ports[phy_id].node = child;

		/* Optional per-port reference clock. */
		if (clk_get_by_ofw_name(dev, child, "ref", &clk) == 0) {
			if (clk_enable(clk) != 0) {
				device_printf(dev,
				              "port %d: cannot enable clock '%s'\n",
				              phy_id, clk_get_name(clk));
				clk_release(clk);
				mt7622_tphy_detach(dev);
				return (ENXIO);
			}
			sc->ports[phy_id].ref_clk = clk;
			if (bootverbose) {
				clk_get_freq(clk, &freq);
				device_printf(dev,
				              "port %d: clock '%s' @ %ju Hz\n", phy_id,
				              clk_get_name(clk), (uintmax_t)freq);
			}
		}

		if (fdt_regsize(child, &start, &size) != 0) {
			device_printf(dev, "cannot parse reg for port %d\n",
			              phy_id);
			mt7622_tphy_detach(dev);
			return (ENXIO);
		}

		sc->ports[phy_id].rid = phy_rid;
		sc->ports[phy_id].res = bus_alloc_resource(dev, SYS_RES_MEMORY,
		                                           &sc->ports[phy_id].rid, start, start + size - 1, size,
		                                           RF_ACTIVE);
		if (sc->ports[phy_id].res == NULL) {
			device_printf(dev, "port %d: cannot map regs\n", phy_id);
			mt7622_tphy_detach(dev);
			return (ENXIO);
		}

		memset(&phy_init, 0, sizeof(phy_init));
		phy_init.ofw_node = child;
		phy_init.id = phy_id;
		phynode = phynode_create(dev, &mt_phynode_class, &phy_init);
		if (phynode == NULL) {
			device_printf(dev, "port %d: phynode_create failed\n",
			              phy_id);
			mt7622_tphy_detach(dev);
			return (ENXIO);
		}

		phynode_sc = phynode_get_softc(phynode);
		phynode_sc->dev = dev;
		phynode_sc->port_id = phy_id;
		phynode_sc->node = child;
		phynode_sc->rid = sc->ports[phy_id].rid;
		phynode_sc->res = sc->ports[phy_id].res;

		if (phynode_register(phynode) == NULL) {
			device_printf(dev, "port %d: phynode_register failed\n",
			              phy_id);
			mt7622_tphy_detach(dev);
			return (ENXIO);
		}

		OF_device_register_xref(OF_xref_from_node(child), dev);

		sc->nports++;
		phy_id++;
		phy_rid++;
	}

	bus_attach_children(dev);

	return (0);
}

static int
mt7622_tphy_map(device_t dev, phandle_t xref, int ncells, pcell_t *cells,
                intptr_t *id)
{
	struct mt_tphy_softc *sc;
	phandle_t node;
	int i;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(xref);
	if (node <= 0)
		return (ERANGE);

	if (ncells < 1)
		return (ERANGE);

	for (i = 0; i < sc->nports; i++) {
		if (sc->ports[i].node == node) {
			sc->ports[i].mode = cells[0];
			*id = sc->ports[i].port_id;
			return (0);
		}
	}

	return (ERANGE);
}

static int
mt7622_tphy_detach(device_t dev)
{
	struct mt_tphy_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < MT7622_TPHY_MAX_PORTS; i++) {
		if (sc->ports[i].res != NULL) {
			bus_release_resource(dev, SYS_RES_MEMORY,
			                     sc->ports[i].rid, sc->ports[i].res);
			sc->ports[i].res = NULL;
		}
		if (sc->ports[i].ref_clk != NULL) {
			clk_disable(sc->ports[i].ref_clk);
			clk_release(sc->ports[i].ref_clk);
			sc->ports[i].ref_clk = NULL;
		}
	}
	if (sc->mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		                     sc->mem_res);
		sc->mem_res = NULL;
	}
	return (0);
}

static device_method_t mt7622_tphy_methods[] = {
		/* Device interface */
		DEVMETHOD(device_probe,		mt7622_tphy_probe),
		DEVMETHOD(device_attach,	mt7622_tphy_attach),
		DEVMETHOD(device_detach,	mt7622_tphy_detach),

		/* Phy device interface */
		DEVMETHOD(phydev_map,		mt7622_tphy_map),

		DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7622_tphy, mt7622_tphy_driver, mt7622_tphy_methods,
sizeof(struct mt_tphy_softc));
EARLY_DRIVER_MODULE(mt7622_tphy, simplebus, mt7622_tphy_driver, NULL, NULL,
		BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);