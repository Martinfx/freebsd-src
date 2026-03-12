/*-
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

#define U3P_U2PHYDTM1		0x06C
#define P2C_RG_UART_EN		(1U << 16)
#define P2C_FORCE_IDDIG		(1U << 9)
#define P2C_RG_VBUSVALID	(1U << 5)
#define P2C_RG_SESSEND		(1U << 4)
#define P2C_RG_AVALID		(1U << 2)
#define P2C_RG_IDDIG		(1U << 1)

#define U3P_USBPHYACR5		 0x014
#define PA5_RG_U2_HSTX_SRCAL_EN (1U << 15)
#define PA5_RG_U2_HSTX_SRCTRL	((1U << 14) | (1U << 12))
#define PA5_RG_U2_HS_100U_U3_EN	(1U << 11)

#define U3P_U2PHYACR4		0x020
#define P2C_RG_USB20_GPIO_CTL	(1U << 9)
#define P2C_USB20_GPIO_MODE	(1U << 8)
#define P2C_U2_GPIO_CTR_MSK	(P2C_RG_USB20_GPIO_CTL | P2C_USB20_GPIO_MODE)

#define U3P_USBPHYACR6		 0x018
#define PA6_RG_U2_PRE_EMP	 ((1U << 31) | (1U << 30))
#define PA6_RG_U2_BC11_SW_EN	 (1U << 23)
#define PA6_RG_U2_OTG_VBUSCMP_EN (1U << 20)
#define PA6_RG_U2_DISCTH	 ((1U << 7) | (1U << 4))
#define PA6_RG_U2_SQTH		 ((1U << 3) | (1U << 0))

// For usb2
#define U3P_USBPHYACR0		0x000
#define U3P_U2PHYDTM0		0x068
#define P2C_FORCE_UART_EN	(1U << 26)
#define P2C_FORCE_DATAIN	(1U << 23)
#define P2C_FORCE_DM_PULLDOWN	(1U << 21)
#define P2C_FORCE_DP_PULLDOWN	(1U << 20)
#define P2C_FORCE_XCVRSEL	(1U << 19)
#define P2C_FORCE_SUSPENDM	(1U << 18)
#define P2C_FORCE_TERMSEL	(1U << 17)
#define P2C_RG_XCVRSEL		((1U << 5) | (1U << 4))
#define P2C_RG_DATAIN		((1U << 13) | (1U << 10))
#define P2C_RG_DMPULLDOWN	(1U << 7)
#define P2C_RG_DPPULLDOWN	(1U << 6)
#define P2C_RG_SUSPENDM		(1U << 3)
#define P2C_RG_TERMSEL		(1U << 2)
#define P2C_DTM0_PART_MASK 	(P2C_FORCE_DATAIN | P2C_FORCE_DM_PULLDOWN | \
				P2C_FORCE_DP_PULLDOWN | P2C_FORCE_XCVRSEL | \
				P2C_FORCE_TERMSEL | P2C_RG_DMPULLDOWN | \
				P2C_RG_DPPULLDOWN | P2C_RG_TERMSEL)
#define U3P_U2PHYDTM1		0x06C
#define P2C_RG_UART_EN		(1U << 16)
#define PA0_RG_USB20_INTR_EN	(1U << 5)

#define __SHIFTIN(mask, val) \
    (((val) << __builtin_ctzl(mask)) & (mask))

#define ANA_RG_CTRL_SIGNAL6		0x60
#define RG_CDR_BC_GEN1_MSK		((1U << 28) | (1U << 24))
#define RG_CDR_BIRLTR_GEN1_MSK		((1U << 4) | (1U << 0))

#define ANA_EQ_EYE_CTRL_SIGNAL4		0xd8
#define RG_CDR_BIRLTD0_GEN1_MSK		((1U << 20) | (1U << 16))

#define ANA_EQ_EYE_CTRL_SIGNAL5		0xdc
#define RG_CDR_BIRLTD0_GEN3_MSK		((1U << 4) | (1U << 0))

#define ANA_RG_CTRL_SIGNAL4		0x58
#define RG_CDR_BICLTR_GEN1_MSK		((1U << 23) | (1U << 20))
#define RG_CDR_BR_GEN2_MSK		((1U << 10) | (1U << 8))

#define PHYD_CTRL_SIGNAL_MODE4		0x1c
#define RG_CDR_BICLTD1_GEN1_MSK		((1U << 23) | (1U << 20))
#define RG_CDR_BICLTD0_GEN1_MSK		((1U << 11) | (1U << 8))

#define PHYD_DESIGN_OPTION2		0x24
/* Symbol lock count selection */
#define RG_LOCK_CNT_SEL_MSK		((1U << 5) | (1U << 4))

#define PHYD_DESIGN_OPTION9	0x40
/* COMWAK GAP width window */
#define RG_TG_MAX_MSK		((1U << 20) | (1U << 16))
/* COMINIT GAP width window */
#define RG_T2_MAX_MSK		((1U << 13) | (1U << 8))
/* COMWAK GAP width window */
#define RG_TG_MIN_MSK		((1U << 7) | (1U << 5))
/* COMINIT GAP width window */
#define RG_T2_MIN_MSK		((1U << 4) | (1U << 0))

#define ANA_RG_CTRL_SIGNAL1		0x4c
/* TX driver tail current control for 0dB de-empahsis mdoe for Gen1 speed */
#define RG_IDRV_0DB_GEN1_MSK	((1U << 13) | (1U << 8))
/* I-path capacitance adjustment for Gen1 */
#define RG_CDR_BC_GEN1_MSK		((1U << 28) | (1U << 24))
#define RG_CDR_BIRLTR_GEN1_MSK		((1U << 4) | (1U << 0))

#define ANA_EQ_EYE_CTRL_SIGNAL1		0x6c
/* RX Gen1 LEQ tuning step */
#define RG_EQ_DLEQ_LFI_GEN1_MSK		((1U << 11) | (1U << 8))

#define MT7622_TPHY_MAX_PORTS  3

struct mt_phynode_softc {
	device_t dev;
	struct resource *res;
	phandle_t node;
	uint32_t base;
	int	rid;
	int mode;
	int port_id;
};

struct mt_tphy_softc {
	device_t dev;
	phandle_t node;
	struct resource *mem_res;
	int mem_rid;
	struct mt_phynode_softc ports[MT7622_TPHY_MAX_PORTS];
	struct phynode *phynode;
	phy_mode_t mode;
	int nports;
};


static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-tphy",     1},
    {NULL,                       0}
};

static int
mt_phynode_enable(struct phynode *phynode, bool enable)
{
	struct mt_phynode_softc *sc;
	device_t dev;
	phandle_t node;
	//intptr_t phy_id;
	char namebuf[64];
	///int rv;

	sc = phynode_get_softc(phynode);
	dev = phynode_get_device(phynode);

	//phy_id = phynode_get_id(phynode);
	node = phynode_get_ofw_node(phynode);

	if (sc->res == NULL) {
		device_printf(phynode_get_device(phynode),
		    "port %d: mem_res is NULL\n", sc->port_id);
		return (ENXIO);
	}

	if (OF_getprop(node, "name", namebuf, sizeof(namebuf)) > 0) {
		/* u2port0, u2port1 */
		if(strstr(namebuf, "1a0c4800") != NULL ||
	          (strstr(namebuf, "1a0c5000") != NULL)) {
			device_printf(dev, "LINE %d\n", __LINE__);
			uint32_t tmp = bus_read_4(sc->res, U3P_U2PHYDTM0);
			uint32_t bitset = P2C_FORCE_UART_EN | P2C_FORCE_SUSPENDM;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_U2PHYDTM0, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_U2PHYDTM0);
			bitset = P2C_RG_XCVRSEL | P2C_RG_DATAIN | P2C_DTM0_PART_MASK;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_U2PHYDTM0, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_U2PHYDTM1);
			bitset = P2C_RG_UART_EN;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_U2PHYDTM1, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_USBPHYACR0);
			bitset = PA0_RG_USB20_INTR_EN;
			tmp |= bitset;
			bus_write_4(sc->res, U3P_USBPHYACR0, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_USBPHYACR5);
			bitset = PA5_RG_U2_HS_100U_U3_EN;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_USBPHYACR5, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_U2PHYACR4);
			bitset = P2C_U2_GPIO_CTR_MSK;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_U2PHYACR4, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_USBPHYACR6);
			bitset = PA6_RG_U2_BC11_SW_EN;
			tmp &= ~bitset;
			bus_write_4(sc->res, U3P_USBPHYACR6, tmp);

			device_printf(dev, "U2PHYDTM0 = 0x%x, %d\n", tmp, __LINE__);
			tmp = bus_read_4(sc->res, U3P_USBPHYACR6);
			bitset = PA6_RG_U2_SQTH;
			tmp &= ~bitset;
			tmp |= 2 & bitset;
			bus_write_4(sc->res, U3P_USBPHYACR6, tmp);
		} else if(strstr(namebuf, "1a0c4900") != NULL) {
			device_printf(phynode_get_device(phynode),
			    "port %d: name = %s\n", sc->port_id, namebuf);
		} else if(strstr(namebuf, "1a243000") != NULL) {
			device_printf(dev, "LINE %d\n", __LINE__);
			uint32_t tmp = bus_read_4(sc->res, ANA_RG_CTRL_SIGNAL6);
			uint32_t bitset = RG_CDR_BIRLTR_GEN1_MSK | RG_CDR_BC_GEN1_MSK;
			tmp &= ~bitset;
			tmp |= (__SHIFTIN(0x6, RG_CDR_BIRLTR_GEN1_MSK) |
				   __SHIFTIN(0x1a, RG_CDR_BC_GEN1_MSK)) & bitset;
			bus_write_4(sc->res, ANA_RG_CTRL_SIGNAL6, tmp);

			tmp = bus_read_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL4);
			bitset = RG_CDR_BIRLTD0_GEN1_MSK;
			tmp &= ~bitset;
			tmp |= __SHIFTIN(0x18, RG_CDR_BIRLTD0_GEN1_MSK) & bitset;
			bus_write_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL4, tmp);

			tmp = bus_read_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL5);
			bitset = RG_CDR_BIRLTD0_GEN3_MSK;
			tmp &= ~bitset;
			tmp |= __SHIFTIN(0x06, RG_CDR_BIRLTD0_GEN3_MSK) & bitset;
			bus_write_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL5, tmp);

			tmp = bus_read_4(sc->res, ANA_RG_CTRL_SIGNAL4);
			bitset = RG_CDR_BICLTR_GEN1_MSK | RG_CDR_BR_GEN2_MSK;
			tmp &= ~bitset;
			tmp |= (__SHIFTIN(0x0c, RG_CDR_BICLTR_GEN1_MSK) |
				   __SHIFTIN(0x07, RG_CDR_BR_GEN2_MSK)) & bitset;
			bus_write_4(sc->res, ANA_RG_CTRL_SIGNAL4, tmp);

			tmp = bus_read_4(sc->res, PHYD_CTRL_SIGNAL_MODE4);
			bitset = RG_CDR_BICLTD0_GEN1_MSK | RG_CDR_BICLTD1_GEN1_MSK,
			tmp &= ~bitset;
			tmp |= (__SHIFTIN(0x08, RG_CDR_BICLTD0_GEN1_MSK) |
				   __SHIFTIN(0x02, RG_CDR_BICLTD1_GEN1_MSK)) & bitset;
			bus_write_4(sc->res, PHYD_CTRL_SIGNAL_MODE4, tmp);

			tmp = bus_read_4(sc->res, PHYD_DESIGN_OPTION2);
			bitset = RG_LOCK_CNT_SEL_MSK;
			tmp &= ~bitset;
			tmp |= 0x02 & bitset;
			bus_write_4(sc->res, PHYD_DESIGN_OPTION2, tmp);

			tmp = bus_read_4(sc->res, PHYD_DESIGN_OPTION9);
			bitset = RG_T2_MIN_MSK | RG_TG_MIN_MSK,
			tmp &= ~bitset;
			tmp |= (__SHIFTIN(0x12, RG_T2_MIN_MSK) |
				   __SHIFTIN(0x04, RG_TG_MIN_MSK)) & bitset;
			bus_write_4(sc->res, PHYD_DESIGN_OPTION9, tmp);

			tmp = bus_read_4(sc->res, PHYD_DESIGN_OPTION9);
			bitset = RG_T2_MAX_MSK | RG_TG_MAX_MSK,
			tmp &= ~bitset;
			tmp |= (__SHIFTIN(0x31, RG_T2_MAX_MSK) |
				   __SHIFTIN(0x0e, RG_TG_MAX_MSK)) & bitset;
			bus_write_4(sc->res, PHYD_DESIGN_OPTION9, tmp);

			tmp = bus_read_4(sc->res, ANA_RG_CTRL_SIGNAL1);
			bitset = RG_IDRV_0DB_GEN1_MSK;
			tmp &= ~bitset;
			tmp |= 0x20 & bitset;
			bus_write_4(sc->res, ANA_RG_CTRL_SIGNAL1, tmp);

			tmp = bus_read_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL1);
			bitset = RG_EQ_DLEQ_LFI_GEN1_MSK;
			tmp &= ~bitset;
			tmp |= 0x03 & bitset;
			bus_write_4(sc->res, ANA_EQ_EYE_CTRL_SIGNAL1, tmp);

		} else {
			device_printf(phynode_get_device(phynode),
			    "port %d: unknown name '%s'\n", sc->port_id, namebuf);
			return (ENXIO);
		}
	}

	return (0);
}

static int
mt_phynode_set_mode(struct phynode *phynode, phy_mode_t mode,
		    phy_submode_t submode)
{
	struct mt_phynode_softc *sc;
	device_t dev;
	phandle_t node;
	intptr_t phy_id;
	char namebuf[64];
	int rv;

	sc = phynode_get_softc(phynode);
	dev = phynode_get_device(phynode);
	phy_id = phynode_get_id(phynode);

	node = phynode_get_ofw_node(phynode);
	if (node > 0) {
		rv = OF_getprop(node, "name", namebuf, sizeof(namebuf));
		if (rv > 0) {
			device_printf(dev, "mt_phynode_set_mode: node name = %s, pky_id=%ld\n", namebuf, phy_id);
		}
		device_printf(dev, "mt_phynode_set_mode: node = %d\n", node);
	} else {
		device_printf(dev, "mt_phynode_set_mode: node UNKNOWN (<=0)\n");
	}

	device_printf(dev, "Mode phy %d\n", sc->mode);

	/*if (sc->mode == mode) {
	    return (0);
	}*/

	uint32_t tmp = bus_read_4(sc->res, U3P_U2PHYDTM1);
	switch (mode) {
	    case PHY_MODE_USB_DEVICE:
		tmp |= P2C_FORCE_IDDIG | P2C_RG_IDDIG;
		device_printf(dev,"PHY_USB_MODE_DEVICE\n");
		break;
	    case PHY_MODE_USB_HOST:
		tmp |= P2C_FORCE_IDDIG;
		tmp &= ~P2C_RG_IDDIG;
		device_printf(dev,"PHY_USB_MODE_HOST\n");
		break;
	    case PHY_MODE_USB_OTG:
		tmp &= ~(P2C_FORCE_IDDIG | P2C_RG_IDDIG);
		device_printf(dev,"PHY_USB_MODE_OTG\n");
		break;
	    case PHY_MODE_INVALID:
		device_printf(dev,"PHY_MODE_INVALID\n");
		break;
	    default:
		device_printf(dev,"default \n");
		return (EINVAL);
	}

	bus_write_4(sc->res, U3P_U2PHYDTM1, tmp);

	sc->mode = mode;

	return (0);
}

/* Phy controller class and methods. */
static phynode_method_t mt_phynode_methods[] = {
    PHYNODEMETHOD(phynode_enable, mt_phynode_enable),
    PHYNODEMETHOD(phynode_set_mode,	mt_phynode_set_mode),
    PHYNODEMETHOD_END
};

DEFINE_CLASS_1(mt_phynode, mt_phynode_class, mt_phynode_methods,
sizeof(struct mt_phynode_softc), phynode_class);

static int
mt7622_tphy_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0) {
		return (ENXIO);
	}

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
	int phy_rid, phy_id = 0, rv;
	char name[64];
	u_long start, size;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(sc->dev);
	sc->nports = 0;

	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					     &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Cannot allocate base memory\n");
		//return ENXIO;
	}
	device_printf(dev, "Base MMIO mapped at rid=%d\n", sc->mem_rid);

	phy_rid = 1;

	for (child = OF_child(sc->node); child != 0; child = OF_peer(child)) {
		if (OF_getprop(child, "name", &name, sizeof(name)) > 0) {

			sc->ports[phy_id].port_id = phy_id;
			sc->ports[phy_id].node = child;

			if (clk_get_by_ofw_name(dev, child, "ref", &clk) ==0) {
				if (clk_enable(clk) != 0) {
					device_printf(dev,
						      "Port %d: cannot enable clock '%s'\n",
						      phy_id, clk_get_name(clk));
					clk_release(clk);
					return ENXIO;
				}

				if (bootverbose) {
					clk_get_freq(clk, &freq);
					device_printf(dev,
						      "Port %d: clock '%s' @ %lu Hz\n",
						      phy_id, clk_get_name(clk),
						      freq);
				}
			}

			rv = fdt_regsize(child, &start, &size);
			if (rv != 0) {
				device_printf(dev, "cannot parse reg for port %d\n", phy_id);
				return (ENXIO);
			}
			bus_set_resource(dev, SYS_RES_MEMORY, phy_rid, start, size);

			memset(&phy_init, 0, sizeof(phy_init));
			phy_init.ofw_node = child;        /* předat node phynode */
			phy_init.id = phy_id;
			phynode = phynode_create(dev, &mt_phynode_class,
						 &phy_init);

			phynode_sc = phynode_get_softc(phynode);
			phynode_sc->port_id = phy_id;
			phynode_sc->rid     = phy_rid;
			phynode_sc->res     = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
			    &phy_rid, RF_ACTIVE);
			if (phynode_sc->res == NULL) {
				device_printf(dev, "Port %d: cannot map regs\n", phy_id);
				return (ENXIO);
			}


			if (phynode == NULL) {
				device_printf(dev,
					      "Port %d: phynode_create failed\n",
					      phy_id);
				return ENXIO;
			}

			if (phynode_register(phynode) == NULL) {
				device_printf(dev,
					      "Port %d: phynode_register failed\n",
					      phy_id);
				return ENXIO;
			}

			sc->nports++;
			phy_id++;
			phy_rid++;

			OF_device_register_xref(OF_xref_from_node(child),
			    dev);
		}
	}

	bus_attach_children(sc->dev);

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

	for (i = 0; i < sc->nports; i++) {
		if (sc->ports[i].node == node) {
			*id = sc->ports[i].port_id;
		}
	}

	return 0;
}

static int
mt7622_tphy_detach(device_t dev)
{
	device_printf(dev, "Cannot detach tphy\n");
	return (EBUSY);
}

static device_method_t mt7622_tphy_methods[] = {
    	/* Device interface */
    	DEVMETHOD(device_probe, mt7622_tphy_probe),
    	DEVMETHOD(device_attach, mt7622_tphy_attach),
    	DEVMETHOD(device_detach, mt7622_tphy_detach),
    	DEVMETHOD(phydev_map, mt7622_tphy_map),
    	DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7622_tphy, mt7622_tphy_driver, mt7622_tphy_methods,
sizeof(struct mt_tphy_softc));
EARLY_DRIVER_MODULE(mt7622_tphy, simplebus, mt7622_tphy_driver, NULL, NULL,
79);
