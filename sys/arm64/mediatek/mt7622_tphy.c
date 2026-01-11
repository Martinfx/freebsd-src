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

#define U3P_SPLLC_XTALCTL3		    0x018
#define XC3_RG_U3_XTAL_RX_PWD		(1u << 9)
#define XC3_RG_U3_FRC_XTAL_RX_PWD	(1u << 8)

#define U3P_U3_PHYA_DA_REG0	        0x100
//#define P3A_RG_XTAL_EXT_PE2H        __BITS(17, 16)
#define P3A_RG_XTAL_EXT_PE1H        ((1u << 13) | (1u << 12))
#define P3A_RG_XTAL_EXT_EN_U3       ((1u << 11) | (1u << 10))

#define U3P_U3_PHYD_CDR1		    0x05c
#define P3D_RG_CDR_BIR_LTD1		    ((1u << 28) | (1u << 24))
#define P3D_RG_CDR_BIR_LTD0         ((1u << 12) | (1u << 8))

#define U3P_U3_PHYA_REG9	        0x024
#define P3A_RG_RX_DAC_MUX		    ((1u << 5) | (1u << 2))

#define U3P_U3_PHYD_LFPS1		    0x00c
#define P3D_RG_FWAKE_TH		        ((1u << 21) | (1u << 10))

#define U3P_U3_PHYA_DA_REG0	        0x100
#define P3A_RG_XTAL_EXT_PE2H		((1u << 17) | (1u << 16))
//#define P3A_RG_XTAL_EXT_PE1H		GENMASK(13, 12)
//#define P3A_RG_XTAL_EXT_EN_U3		GENMASK(11, 10)

#define U3P_U2PHYDTM1		0x06C
#define P2C_RG_UART_EN	    (1U << 16)
#define P2C_FORCE_IDDIG		(1u << 9)
#define P2C_RG_IDDIG        (1u << 1)

#define USB_PHY_SWITCH_CTRL	0x0
#define RG_PHY_SW_TYPE		(1u << 3)
#define RG_PHY_SW_PCIE		0x0
#define RG_PHY_SW_USB3		0x1
#define RG_PHY_SW_SGMII		0x2
#define RG_PHY_SW_SATA		0x3

#define MT7622_TPHY_MAX_PORTS  3

struct mt_phynode_softc {
    device_t dev;
    struct resource  *mem_res;
    struct resource *res;
    phandle_t node;
    uint32_t base;
    int	rid;
    int mode;
    int port_id;
};

struct mt_tphy_softc {
    device_t dev;
    struct resource  *mem_res;
    struct mt_phynode_softc ports[MT7622_TPHY_MAX_PORTS];
    phy_mode_t       mode;
    phandle_t node;
    int             nports;
};


static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-tphy",     1},
        {"mediatek,generic-tphy-v1", 1},
        {NULL,                       0}
};

static int
mt_phynode_enable(struct phynode *phynode, bool enable)
{
    //struct mt_phynode_softc *sc;
    device_t dev;
    phandle_t node;
    intptr_t phy_id;
    char namebuf[64];
    int rv;

    //sc = phynode_get_softc(phynode);
    dev = phynode_get_device(phynode);
    phy_id = phynode_get_id(phynode);

    node = phynode_get_ofw_node(phynode);
    if (node > 0) {
        rv = OF_getprop(node, "name", namebuf, sizeof(namebuf));
        if (rv > 0) {
            device_printf(dev, "mt_phynode_enable: node name = %s, pky_id=%ld\n", namebuf, phy_id);
        }
        device_printf(dev, "mt_phynode_enable: node = %d\n", node);
    } else {
        device_printf(dev, "mt_phynode_enable: node UNKNOWN (<=0)\n");
    }
    /*tmp = bus_read_4(sc->mem_res, U3P_U3_PHYA_DA_REG0);
    tmp &= ~0x00000c00;
    tmp |= ((2 << __builtin_ffs(0x00000c00) - 1) & 0x00000c00);
    bus_write_4(sc->mem_res, U3P_U3_PHYA_DA_REG0, tmp);

    tmp = bus_read_4(sc->mem_res, U3P_U3_PHYA_REG9);
    tmp = (tmp & ~P3A_RG_TX_EIDLE_CM) |
          ((0xe << __builtin_ffs(P3A_RG_TX_EIDLE_CM) - 1) & P3A_RG_TX_EIDLE_CM);
    bus_write_4(sc->mem_res, U3P_U3_PHYA_REG9, tmp);*/

    /*tmp = bus_read_4(sc->mem_res, U3P_U3_PHYD_CDR1);
    tmp &= ~(P3D_RG_CDR_BIR_LTD0 | P3D_RG_CDR_BIR_LTD1);
    tmp |= ((0xc << (__builtin_ffs(P3D_RG_CDR_BIR_LTD0)-1)) & P3D_RG_CDR_BIR_LTD0) |
           ((0x3 << (__builtin_ffs(P3D_RG_CDR_BIR_LTD1)-1)) & P3D_RG_CDR_BIR_LTD1);
    bus_write_4(sc->mem_res, U3P_U3_PHYD_CDR1, tmp);

    tmp = bus_read_4(sc->mem_res, U3P_U3_PHYD_LFPS1);
    tmp = (tmp & ~P3D_RG_FWAKE_TH) |
          ((0x34 << (__builtin_ffs(P3D_RG_FWAKE_TH)-1)) & P3D_RG_FWAKE_TH);
    bus_write_4(sc->mem_res, U3P_U3_PHYD_LFPS1, tmp);*/


    ///device_printf(dev, "phy enable() enable=%d, node=%u\n",enable, node);

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

    /*uint32_t tmp = bus_read_4(sc->res, U3P_U2PHYDTM1);
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
        default:
            device_printf(dev,"default \n");
            return (EINVAL);
    }

    bus_write_4(sc->res, U3P_U2PHYDTM1, tmp);*/

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
    //struct mt_phynode_softc *phy_sc;

    phandle_t child;
    char *usbname;
    int rid = 0, phy_id = 0, rv;
    clk_t clk;
    uint64_t freq;
    //uint32_t tmp;
    ///uint32_t base;
    int port_id;

    sc = device_get_softc(dev);
    sc->dev = dev;
    sc->node = ofw_bus_get_node(sc->dev);
    sc->nports = 0;
    /* USB PHY */
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                         RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Cannot allocate mem_res memory resources\n");
        return (ENXIO);
    }

    rid = 1;
    for (child = OF_child(sc->node); child != 0; child = OF_peer(child)) {
        struct phynode *phynode;
        struct mt_phynode_softc *phy_sc;
        struct phynode_init_def phy_init;
        u_long start, size;

        if(clk_get_by_ofw_name(sc->dev, child, "ref", &clk) == 0) {
            if (clk) {
                if (clk_enable(clk) != 0) {
                    device_printf(sc->dev, "Couldn't enable clock %s\n",
                                  clk_get_name(clk));
                    return (ENXIO);
                }

                if (bootverbose) {
                    clk_get_freq(clk, &freq);
                    device_printf(sc->dev,"Tphy clock %s frequency: %lu\n", clk_get_name(clk), freq);
                }
            }
        }

        port_id = sc->nports;
        phy_sc = &sc->ports[port_id];
        phy_sc->port_id = port_id;
        phy_sc->node = child;
        phy_sc->mem_res = sc->mem_res;
        //phy_sc->xref = OF_xref_from_node(child);

        OF_device_register_xref(OF_xref_from_node(child), dev);

        rv = fdt_regsize(child, &start, &size);
        if (rv != 0) {
            device_printf(dev, "cannot parse reg for port %d\n", port_id);
            return (ENXIO);
        }

        bus_set_resource(dev, SYS_RES_MEMORY, rid, start, size);
        phy_sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE | RF_SHAREABLE);
        if (phy_sc->res == NULL) {
            device_printf(dev, "cannot map port %d regs\n", port_id);
            return (ENXIO);
        }

        rv = OF_getprop_alloc(child, "name", (void **)&usbname);
        if (rv == -1) {
            continue;
        }

        if (strcasecmp(usbname, "usb-phy@1a0c4800")) {
            device_printf(sc->dev , "inicialize u2port0\n");
        }
        else if (strcasecmp(usbname, "usb-phy@1a0c4900")) {
            device_printf(sc->dev , "inicialize u3port0\n");

            /*tmp = bus_read_4(sc->mem_res, U3P_SPLLC_XTALCTL3);
            tmp |= (XC3_RG_U3_XTAL_RX_PWD | XC3_RG_U3_FRC_XTAL_RX_PWD);
            bus_write_4(sc->mem_res, U3P_SPLLC_XTALCTL3, tmp);*/

            /*tmp = bus_read_4(sc->mem_res, U3P_U3_PHYA_DA_REG0);
            tmp &= ~mask;
            tmp |= val & mask;
            bus_write_4(sc->mem_res,*/

        }
        else if (strcasecmp(usbname, "usb-phy@1a0c5000")) {
            device_printf(sc->dev , "inicialize u2port1\n");
        }

        bzero(&phy_init, sizeof(phy_init));
        phy_init.id = phy_id;
        phy_id++;
        phy_init.ofw_node = child;
        phynode = phynode_create(sc->dev, &mt_phynode_class, &phy_init);
        if(phynode == NULL) {
            device_printf(sc->dev , "failed to create phynode PHY\n");
            return (ENXIO);
        }
        else {
            device_printf(sc->dev , "create phynode PHY\n");
        }

        //phy_sc-> = phynode_get_softc(phynode);
        //psc->port = p;
        //psc->parent = sc;

        if (phynode_register(phynode) == NULL) {
            device_printf(sc->dev, "Cannot register phy.\n");
            return (ENXIO);
        }
        else {
            device_printf(sc->dev , "create phynode_register PHY\n");
        }

        OF_prop_free(usbname);
        rid++;
        sc->nports++;
        sc->mode = 0;
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
    return (0);
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
