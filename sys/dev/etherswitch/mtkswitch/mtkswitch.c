/*-
 * Copyright (c) 2016 Stanislav Galabov.
 * Copyright (c) 2011-2012 Stefan Bethke.
 * Copyright (c) 2012 Adrian Chadd.
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
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <machine/bus.h>
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mdio/mdio.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/etherswitch/mtkswitch/mtkswitchvar.h>

#include <dev/ofw/ofw_bus_subr.h>

#include "mdio_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

#define DEBUG

#if defined(DEBUG)
static SYSCTL_NODE(_debug, OID_AUTO, mtkswitch, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
"mtkswitch");
#endif

static inline int mtkswitch_portforphy(int phy);
static int mtkswitch_ifmedia_upd(if_t ifp);
static void mtkswitch_ifmedia_sts(if_t ifp, struct ifmediareq *ifmr);
static void mtkswitch_tick(void *arg);

static const struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7531",		MTK_SWITCH_MT7531 },
    { "ralink,rt3050-esw",		MTK_SWITCH_RT3050 },
    { "ralink,rt3352-esw",		MTK_SWITCH_RT3352 },
    { "ralink,rt5350-esw",		MTK_SWITCH_RT5350 },
    { "mediatek,mt7620-gsw",	MTK_SWITCH_MT7620 },
    { "mediatek,mt7621-gsw",	MTK_SWITCH_MT7621 },
    { "mediatek,mt7628-esw",	MTK_SWITCH_MT7628 },

    /* Sentinel */
    { NULL,				MTK_SWITCH_NONE }
};

/*
 * Legacy self-attach for generic "mdio" buses (e.g. the MAC's hinted mdio
 * child, or another MAC driver's MDIO bus), which do not enumerate their
 * FDT children.  On OF-enumerating parents such as mtkmdio(4) the bus adds
 * us itself from the "mdio-bus" node, so only act when the parent really is
 * a plain mdio bus.
 */
static void
mtkswitch_identify(driver_t *driver, device_t parent)
{
        if (strcmp(device_get_name(parent), "mdio") != 0)
                return;
        if (device_find_child(parent, "mtkswitch", -1) == NULL)
                BUS_ADD_CHILD(parent, 0, "mtkswitch", -1);
}

static int
mtkswitch_probe(device_t dev)
{
        struct mtkswitch_softc *sc;
        const struct ofw_compat_data *ocd;
        phandle_t node;
        mtk_switch_type type;

        node = ofw_bus_get_node(dev);
        if (node != 0 && node != (phandle_t)-1) {
                /*
                 * Preferred path: we are a proper OF child of the MAC's
                 * MDIO bus (the switch@1f node under "mdio-bus"); match on
                 * our own compatible and use our own node.
                 */
                if (!ofw_bus_status_okay(dev))
                        return (ENXIO);
                ocd = ofw_bus_search_compatible(dev, compat_data);
                type = (mtk_switch_type)ocd->ocd_data;
                if (type == MTK_SWITCH_NONE)
                        return (ENXIO);
        } else {
                /*
                 * Legacy path: we were added by identify on a generic mdio
                 * bus and have no OF node of our own, so look the switch
                 * node up in the device tree.
                 */
                type = MTK_SWITCH_NONE;
                for (ocd = compat_data; ocd->ocd_str != NULL; ocd++) {
                        node = ofw_bus_find_compatible(OF_finddevice("/"),
                            ocd->ocd_str);
                        if (node != 0) {
                                type = (mtk_switch_type)ocd->ocd_data;
                                break;
                        }
                }
                if (type == MTK_SWITCH_NONE)
                        return (ENXIO);
        }

        sc = device_get_softc(dev);
        bzero(sc, sizeof(*sc));
        sc->node = node;
        sc->sc_switchtype = type;

        device_set_desc(dev, "MediaTek MT7531 Switch");

        return (BUS_PROBE_DEFAULT);
}

static int
mtkswitch_attach_phys(struct mtkswitch_softc *sc)
{
        int phy, err, nattached;
        char name[IFNAMSIZ];

        nattached = 0;

        /* PHYs need an interface, so we generate a dummy one */
        snprintf(name, IFNAMSIZ, "%sport", device_get_nameunit(sc->sc_dev));
        for (phy = 0; phy < sc->numphys; phy++) {
                if ((sc->phymap & (1u << phy)) == 0) {
                        sc->ifp[phy] = NULL;
                        sc->ifname[phy] = NULL;
                        sc->miibus[phy] = NULL;
                        continue;
                }
                sc->ifp[phy] = if_alloc(IFT_ETHER);
                if_setsoftc(sc->ifp[phy], sc);
                if_setflagbits(sc->ifp[phy], IFF_UP | IFF_BROADCAST |
                                             IFF_DRV_RUNNING | IFF_SIMPLEX, 0);
                sc->ifname[phy] = malloc(strlen(name) + 1, M_DEVBUF, M_WAITOK);
                bcopy(name, sc->ifname[phy], strlen(name) + 1);
                if_initname(sc->ifp[phy], sc->ifname[phy],
                    mtkswitch_portforphy(phy));
                err = mii_attach(sc->sc_dev, &sc->miibus[phy], sc->ifp[phy],
                    mtkswitch_ifmedia_upd, mtkswitch_ifmedia_sts,
                    BMSR_DEFCAPMASK, phy, MII_OFFSET_ANY, 0);
                if (err != 0) {
                        /*
                         * One unusable PHY is not a reason to throw the whole
                         * switch away: drop this port's pseudo interface and
                         * carry on with the rest.  mii_attach() has already
                         * removed the miibus child it may have created.
                         */
                        device_printf(sc->sc_dev,
                            "attaching PHY %d failed: %d\n", phy, err);
                        if_free(sc->ifp[phy]);
                        sc->ifp[phy] = NULL;
                        free(sc->ifname[phy], M_DEVBUF);
                        sc->ifname[phy] = NULL;
                        sc->miibus[phy] = NULL;
                        continue;
                }
                nattached++;
                DPRINTF(sc->sc_dev, "%s attached to pseudo interface "
                                    "%s\n", device_get_nameunit(sc->miibus[phy]),
                    if_name(sc->ifp[phy]));
        }

        /*
         * Not a single PHY answered although the device tree describes some:
         * the switch is not usable, so fail the attach rather than presenting
         * an etherswitch device with no ports.
         */
        if (sc->phymap != 0 && nattached == 0) {
                device_printf(sc->sc_dev, "no PHY responded (phymap 0x%x)\n",
                    sc->phymap);
                return (ENXIO);
        }

        return (0);
}

static void
mtkswitch_free_phys(struct mtkswitch_softc *sc)
{
        int phy;

        for (phy = 0; phy < MTKSWITCH_MAX_PHYS; phy++) {
                if (sc->ifp[phy] != NULL) {
                        if_free(sc->ifp[phy]);
                        sc->ifp[phy] = NULL;
                }
                free(sc->ifname[phy], M_DEVBUF);
                sc->ifname[phy] = NULL;
        }
}

static int
mtkswitch_set_vlan_mode(struct mtkswitch_softc *sc, uint32_t mode)
{

        /* Check for invalid modes. */
        if ((mode & sc->info.es_vlan_caps) != mode)
                return (EINVAL);

        sc->vlan_mode = mode;

        /* Reset VLANs. */
        sc->hal.mtkswitch_vlan_init_hw(sc);

        return (0);
}

/*
 * Parse the switch's "ports" device-tree node to learn the port geometry
 * instead of hard-coding it (modelled on felix(4)):
 *   - which ports exist                -> portmap / numports
 *   - which ports carry an "ethernet"  -> CPU (trunk) ports
 *     phandle
 *   - which ports have a "fixed-link"  -> no autonegotiating PHY; the forced
 *     subnode                             media (speed/duplex/pause) is taken
 *                                         from there
 *   - the remaining user ports         -> phymap (internal PHYs)
 * Returns 0 on success; on any problem the caller keeps the HAL defaults so
 * behaviour can never regress below the hard-coded geometry.
 */
/* Map a device-tree "phy-mode" string onto the modes we act on. */
static mtk_phy_mode
mtkswitch_parse_phy_mode(phandle_t node)
{
        char *mode;
        mtk_phy_mode ret;

        if (OF_getprop_alloc(node, "phy-mode", (void **)&mode) == -1 &&
            OF_getprop_alloc(node, "phy-connection-type",
            (void **)&mode) == -1)
                return (MTK_PHY_MODE_OTHER);

        if (strcmp(mode, "2500base-x") == 0)
                ret = MTK_PHY_MODE_2500BASEX;
        else if (strcmp(mode, "1000base-x") == 0)
                ret = MTK_PHY_MODE_1000BASEX;
        else if (strcmp(mode, "sgmii") == 0)
                ret = MTK_PHY_MODE_SGMII;
        else if (strncmp(mode, "rgmii", 5) == 0)
                ret = MTK_PHY_MODE_RGMII;
        else
                ret = MTK_PHY_MODE_OTHER;

        OF_prop_free(mode);

        return (ret);
}

static int
mtkswitch_parse_ports_fdt(struct mtkswitch_softc *sc)
{
        phandle_t ports, child, fl;
        char *label;
        uint32_t reg, speed, media, portmap, phymap, wanmap, lanmap;
        uint32_t wancpu;
        int cpuport, nports, i;

        if (sc->node == 0 || sc->node == (phandle_t)-1)
                return (ENXIO);

        ports = ofw_bus_find_child(sc->node, "ports");
        if (ports == 0)
                ports = ofw_bus_find_child(sc->node, "ethernet-ports");
        if (ports == 0)
                return (ENXIO);

        portmap = 0;
        phymap = 0;
        wanmap = 0;
        cpuport = -1;
        nports = 0;

        for (child = OF_child(ports); child != 0; child = OF_peer(child)) {
                if (ofw_bus_node_status_okay(child) == 0)
                        continue;
                if (OF_getencprop(child, "reg", &reg, sizeof(reg)) <= 0)
                        continue;
                if (reg >= MTKSWITCH_MAX_PORTS)
                        continue;

                portmap |= (1u << reg);
                if ((int)reg + 1 > nports)
                        nports = reg + 1;

                sc->port_mode[reg] = mtkswitch_parse_phy_mode(child);

                /*
                 * A port with an "ethernet" phandle is a CPU (trunk) port.
                 * A board may have more than one (port 5 and 6 on the
                 * BananaPi R64); pick the highest-numbered one as the primary
                 * CPU port, which matches the MT7531 default and is independent
                 * of DT child ordering.
                 */
                if (OF_getproplen(child, "ethernet") > 0) {
                        sc->cpu_port[reg] = true;
                        if ((int)reg > cpuport)
                                cpuport = reg;
                }

	       /* Remember which jacks the board calls WAN. */
	       label = NULL;
	       if (OF_getprop_alloc(child, "label", (void **)&label) > 0) {
		       if (strncmp(label, "wan", 3) == 0)
			       wanmap |= (1u << reg);
		       OF_prop_free(label);
	       }

                /* A "fixed-link" subnode means there is no PHY to poll. */
                fl = ofw_bus_find_child(child, "fixed-link");
                if (fl != 0) {
                        sc->fixed_port[reg] = true;

                        if (OF_getencprop(fl, "speed", &speed,
                            sizeof(speed)) <= 0)
                                speed = 1000;
                        media = IFM_ETHER;
                        switch (speed) {
                                case 10:
                                        media |= IFM_10_T;
                                        break;
                                case 100:
                                        media |= IFM_100_TX;
                                        break;
                                case 2500:
                                        /*
                                         * IFM_2500_X, not IFM_2500_T: a
                                         * 2500 fixed-link on this switch is
                                         * the serdes trunk to the MAC, not
                                         * NBaseT over twisted pair.  mtge(4)
                                         * names its end of the same link the
                                         * same way.
                                         */
                                        media |= IFM_2500_X;
                                        break;
                                case 1000:
                                default:
                                        media |= IFM_1000_T;
                                        break;
                        }
                        media |= OF_hasprop(fl, "full-duplex") ?
                                 IFM_FDX : IFM_HDX;
                        if (OF_hasprop(fl, "pause"))
                                media |= IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE;
                        sc->fixed_link_status[reg] = media;
                } else if (!sc->cpu_port[reg]) {
                        /* User port reached through an internal PHY. */
                        phymap |= (1u << reg);
                }
        }

        if (portmap == 0)
                return (ENXIO);

        sc->portmap = portmap;
        sc->phymap = phymap;
        sc->numports = nports;
        sc->numphys = nports;
        if (cpuport >= 0)
                sc->cpuport = cpuport;
        sc->info.es_nports = nports;

       /*
	* Carve the WAN segment out, when there is one to carve and a second
	* CPU trunk to carry it.  The wan jack(s) and the lower-numbered trunk
	* become one L2 segment, everything else and the primary trunk the
	* other, expressed both as a forwarding matrix and as VLAN membership
	* (the ports run in secure mode, where both are consulted).  The two
	* segments then only meet in the router.
	*
	* On the BPI-R64 that is: "wan" + port 5 (gmac1/mtge1) on one side,
	* lan0-3 + port 6 (gmac0/mtge0) on the other.
	*/
       wancpu = 0;
       for (i = 0; i < sc->numports; i++)
	       if (sc->cpu_port[i] && i != sc->cpuport) {
		       wancpu = (1u << i);
		       break;
	       }
       sc->segmented = (wanmap != 0 && wancpu != 0);

       if (sc->segmented) {
	       wanmap |= wancpu;
	       lanmap = portmap & ~wanmap;
       } else {
	       wanmap = 0;
	       lanmap = portmap;
       }
       for (i = 0; i < sc->numports; i++) {
	       if ((portmap & (1u << i)) == 0)
		       continue;
	       if ((wanmap & (1u << i)) != 0) {
		       sc->port_matrix[i] = wanmap & ~(1u << i);
		       sc->port_pvid[i] = 2;
	       } else {
		       sc->port_matrix[i] = lanmap & ~(1u << i);
		       sc->port_pvid[i] = 1;
	       }
       }

       if (bootverbose && sc->segmented)
	       device_printf(sc->sc_dev,
		   "wan segment 0x%x (vlan 2), lan segment 0x%x (vlan 1)\n",
		   wanmap, lanmap);

        if (bootverbose)
                device_printf(sc->sc_dev,
                    "DT ports: portmap=0x%x phymap=0x%x cpuport=%d nports=%d\n",
                    sc->portmap, sc->phymap, sc->cpuport, sc->numports);

        return (0);
}

static int
mtkswitch_attach(device_t dev)
{
        struct mtkswitch_softc *sc;
        int err = 0;
        int port;

        sc = device_get_softc(dev);

        /* sc->sc_switchtype is already decided in mtkswitch_probe() */
        sc->numports = MTKSWITCH_MAX_PORTS;
        sc->numphys = MTKSWITCH_MAX_PHYS;
        sc->cpuport = MTKSWITCH_CPU_PORT;
        sc->sc_dev = dev;

        /* Attach the chip-specific HAL (sets the hard-coded default geometry). */
        switch (sc->sc_switchtype)
        {
                case MTK_SWITCH_MT7531:
                        mtk_attach_switch_mt7531(sc);
                        break;
                case MTK_SWITCH_MT7620:
                case MTK_SWITCH_MT7621:
                        mtk_attach_switch_mt7620(sc);
                        break;
                case MTK_SWITCH_RT3050:
                        mtk_attach_switch_rt3050(sc);
                        break;
                default:
                        device_printf(dev, "unsupported switch type\n");
                        return (ENXIO);
        }

        /*
         * Override the geometry from the device tree where described.  This is
         * how the switch is "hung off" the MDIO bus properly: the CPU port,
         * port map and per-port fixed-link media all come from the DT instead
         * of being hard-coded.  On failure the HAL defaults above are kept.
         */
        mtkswitch_parse_ports_fdt(sc);

        /*
         * The MT7531 registers are reached indirectly over MDIO (see
         * mtkswitch_mt7531.c), so there is no memory window to map here.
         */

        mtx_init(&sc->sc_mtx, "mtkswitch", NULL, MTX_DEF);

        /*
         * Reset the switch.  This also validates that a MT7531 really answers
         * on the MDIO bus, so a wedged or absent chip fails the attach here
         * instead of leaving a switch device around whose registers all read
         * back as 0xffffffff.
         */
        err = sc->hal.mtkswitch_reset(sc);
        if (err != 0) {
                device_printf(dev, "switch reset failed\n");
                goto fail;
        }

        err = sc->hal.mtkswitch_hw_setup(sc);
        DPRINTF(dev, "%s: hw_setup: err=%d\n", __func__, err);
        if (err != 0)
                goto fail;

        err = sc->hal.mtkswitch_hw_global_setup(sc);
        DPRINTF(dev, "%s: hw_global_setup: err=%d\n", __func__, err);
        if (err != 0)
                goto fail;

        /* Initialize the switch ports */
        for (port = 0; port < sc->numports; port++) {
                sc->hal.mtkswitch_port_init(sc, port);
        }

        /* Attach the PHYs and complete the bus enumeration */
        err = mtkswitch_attach_phys(sc);
        DPRINTF(dev, "%s: attach_phys: err=%d\n", __func__, err);
        if (err != 0)
                goto fail;

        /* Default to ingress filters off. */
        err = mtkswitch_set_vlan_mode(sc, ETHERSWITCH_VLAN_DOT1Q);
        DPRINTF(dev, "%s: set_vlan_mode: err=%d\n", __func__, err);
        if (err != 0)
                goto fail;

        bus_identify_children(dev);
        bus_attach_children(dev);

        callout_init_mtx(&sc->callout_tick, &sc->sc_mtx, 0);

        MTKSWITCH_LOCK(sc);
        mtkswitch_tick(sc);
        MTKSWITCH_UNLOCK(sc);

        mt7531_sysctl_attach(sc);

        return (0);

fail:
        /* Drop any miibus children before the interfaces they point at. */
        device_delete_children(dev);
        mtkswitch_free_phys(sc);
        mtx_destroy(&sc->sc_mtx);

        return (err);
}

static int
mtkswitch_detach(device_t dev)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);
        int error;

        error = bus_generic_detach(dev);
        if (error != 0)
                return (error);

        callout_drain(&sc->callout_tick);

        mtkswitch_free_phys(sc);

        mtx_destroy(&sc->sc_mtx);

        return (0);
}

/* PHY <-> port mapping is currently 1:1 */
static inline int
mtkswitch_portforphy(int phy)
{

        return (phy);
}

static inline int
mtkswitch_phyforport(int port)
{

        return (port);
}

static inline struct mii_data *
mtkswitch_miiforport(struct mtkswitch_softc *sc, int port)
{
        int phy = mtkswitch_phyforport(port);

        if (phy < 0 || phy >= MTKSWITCH_MAX_PHYS || sc->miibus[phy] == NULL)
                return (NULL);

        return (device_get_softc(sc->miibus[phy]));
}

static inline if_t
mtkswitch_ifpforport(struct mtkswitch_softc *sc, int port)
{
        int phy = mtkswitch_phyforport(port);

        if (phy < 0 || phy >= MTKSWITCH_MAX_PHYS)
                return (NULL);

        return (sc->ifp[phy]);
}

/*
 * Convert port status to ifmedia.
 */
static void
mtkswitch_update_ifmedia(uint32_t portstatus, u_int *media_status,
                         u_int *media_active)
{
        *media_active = IFM_ETHER;
        *media_status = IFM_AVALID;

        if ((portstatus & MTKSWITCH_LINK_UP) != 0)
                *media_status |= IFM_ACTIVE;
        else {
                *media_active |= IFM_NONE;
                return;
        }

        switch (portstatus & MTKSWITCH_SPEED_MASK) {
                case MTKSWITCH_SPEED_10:
                        *media_active |= IFM_10_T;
                        break;
                case MTKSWITCH_SPEED_100:
                        *media_active |= IFM_100_TX;
                        break;
                case MTKSWITCH_SPEED_1000:
                        *media_active |= IFM_1000_T;
                        break;
        }

        if ((portstatus & MTKSWITCH_DUPLEX) != 0)
                *media_active |= IFM_FDX;
        else
                *media_active |= IFM_HDX;

        if ((portstatus & MTKSWITCH_TXFLOW) != 0)
                *media_active |= IFM_ETH_TXPAUSE;
        if ((portstatus & MTKSWITCH_RXFLOW) != 0)
                *media_active |= IFM_ETH_RXPAUSE;
}

static void
mtkswitch_miipollstat(struct mtkswitch_softc *sc)
{
        struct mii_data *mii;
        struct mii_softc *miisc;
        uint32_t portstatus;
        int i, up, port_flap = 0;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);

        for (i = 0; i < sc->numphys; i++) {
                if (sc->miibus[i] == NULL)
                        continue;
                mii = device_get_softc(sc->miibus[i]);
                portstatus = sc->hal.mtkswitch_get_port_status(sc,
                    mtkswitch_portforphy(i));

                /* If a port has flapped - mark it so we can flush the ATU */
                if (((mii->mii_media_status & IFM_ACTIVE) == 0 &&
                     (portstatus & MTKSWITCH_LINK_UP) != 0) ||
                    ((mii->mii_media_status & IFM_ACTIVE) != 0 &&
                     (portstatus & MTKSWITCH_LINK_UP) == 0)) {
                        port_flap = 1;
                }

                mtkswitch_update_ifmedia(portstatus, &mii->mii_media_status,
                    &mii->mii_media_active);
                LIST_FOREACH(miisc, &mii->mii_phys, mii_list) {
                        if (IFM_INST(mii->mii_media.ifm_cur->ifm_media) !=
                            miisc->mii_inst)
                                continue;
                        mii_phy_update(miisc, MII_POLLSTAT);
                }
        }

        /*
         * The trunk ports have no PHY and hence no miibus above, so nothing
         * would ever report their state.  Poll them here and log transitions:
         * on this board the whole data path hangs off the CPU trunk, and a
         * trunk that never comes up is otherwise indistinguishable from a
         * switch that simply forwards nothing.
         */
        for (i = 0; i < sc->numports && i < MTKSWITCH_MAX_PORTS; i++) {
                if (!sc->fixed_port[i])
                        continue;
                portstatus = sc->hal.mtkswitch_get_port_status(sc, i);
                up = (portstatus & MTKSWITCH_LINK_UP) != 0 ? 2 : 1;
                if (sc->fixed_link_last[i] == up)
                        continue;
                if (sc->fixed_link_last[i] != 0)
                        port_flap = 1;
                sc->fixed_link_last[i] = up;
                device_printf(sc->sc_dev, "port %d (%s) link %s\n", i,
                    sc->cpu_port[i] ? "cpu" : "fixed",
                    up == 2 ? "UP" : "DOWN");
        }

        if (port_flap)
                sc->hal.mtkswitch_atu_flush(sc);
}

static void
mtkswitch_tick(void *arg)
{
        struct mtkswitch_softc *sc = arg;

        mtkswitch_miipollstat(sc);
        callout_reset(&sc->callout_tick, hz, mtkswitch_tick, sc);
}

static void
mtkswitch_lock(device_t dev)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);
}

static void
mtkswitch_unlock(device_t dev)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);
        MTKSWITCH_UNLOCK(sc);
}

static etherswitch_info_t *
mtkswitch_getinfo(device_t dev)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (&sc->info);
}

static inline int
mtkswitch_is_cpuport(struct mtkswitch_softc *sc, int port)
{

        return (sc->cpuport == port);
}

static int
mtkswitch_getport(device_t dev, etherswitch_port_t *p)
{
        struct mtkswitch_softc *sc;
        struct mii_data *mii;
        struct ifmediareq *ifmr;
        uint32_t status;
        int err;

        sc = device_get_softc(dev);
        if (p->es_port < 0 || p->es_port >= sc->info.es_nports)
                return (ENXIO);

        err = sc->hal.mtkswitch_port_vlan_get(sc, p);
        if (err != 0)
                return (err);

        mii = mtkswitch_miiforport(sc, p->es_port);
        if (sc->fixed_port[p->es_port]) {
                /*
                 * Fixed-link port (CPU/trunk or a board fixed link): there is
                 * no PHY to poll, so the media (speed/duplex/pause) comes from
                 * the DT "fixed-link" node - e.g. 2500base-T on the BananaPi
                 * R64 trunk.  The link state itself is real though: read it
                 * out of the port's PMSR rather than claiming the port is up
                 * unconditionally, so a trunk that never came up (an
                 * unconfigured SGMII PCS, say) is visible in
                 * etherswitchcfg(8) instead of silently looking healthy.
                 */
                if (sc->cpu_port[p->es_port])
                        p->es_flags |= ETHERSWITCH_PORT_CPU;
                MTKSWITCH_LOCK(sc);
                status = sc->hal.mtkswitch_get_port_status(sc, p->es_port);
                MTKSWITCH_UNLOCK(sc);
                ifmr = &p->es_ifmr;
                ifmr->ifm_count = 0;
                ifmr->ifm_current = ifmr->ifm_active =
                    sc->fixed_link_status[p->es_port];
                ifmr->ifm_mask = 0;
                ifmr->ifm_status = IFM_AVALID;
                if ((status & MTKSWITCH_LINK_UP) != 0)
                        ifmr->ifm_status |= IFM_ACTIVE;
                else
                        ifmr->ifm_active = IFM_ETHER | IFM_NONE;
        } else if (mtkswitch_is_cpuport(sc, p->es_port)) {
                /* Fallback for a CPU port the DT did not describe. */
                p->es_flags |= ETHERSWITCH_PORT_CPU;
                ifmr = &p->es_ifmr;
                ifmr->ifm_count = 0;
                ifmr->ifm_current = ifmr->ifm_active =
                    IFM_ETHER | IFM_1000_T | IFM_FDX;
                ifmr->ifm_mask = 0;
                ifmr->ifm_status = IFM_ACTIVE | IFM_AVALID;
        } else if (mii != NULL) {
                err = ifmedia_ioctl(mii->mii_ifp, &p->es_ifr,
                    &mii->mii_media, SIOCGIFMEDIA);
                if (err)
                        return (err);
        } else {
                ifmr = &p->es_ifmr;
                ifmr->ifm_count = 0;
                ifmr->ifm_current = ifmr->ifm_active = IFM_NONE;
                ifmr->ifm_mask = 0;
                ifmr->ifm_status = 0;
        }
        return (0);
}

static int
mtkswitch_setport(device_t dev, etherswitch_port_t *p)
{
        int err;
        struct mtkswitch_softc *sc;
        struct ifmedia *ifm;
        struct mii_data *mii;
        if_t ifp;

        sc = device_get_softc(dev);
        if (p->es_port < 0 || p->es_port >= sc->info.es_nports)
                return (ENXIO);

        /* Port flags. */
        if (sc->vlan_mode == ETHERSWITCH_VLAN_DOT1Q) {
                err = sc->hal.mtkswitch_port_vlan_setup(sc, p);
                if (err)
                        return (err);
        }

        /* No media changes on CPU/trunk or other fixed-link ports. */
        if (sc->fixed_port[p->es_port] || mtkswitch_is_cpuport(sc, p->es_port))
                return (0);

        mii = mtkswitch_miiforport(sc, p->es_port);
        if (mii == NULL)
                return (ENXIO);

        ifp = mtkswitch_ifpforport(sc, p->es_port);

        ifm = &mii->mii_media;
        return (ifmedia_ioctl(ifp, &p->es_ifr, ifm, SIOCSIFMEDIA));
}

static void
mtkswitch_statchg(device_t dev)
{

        DPRINTF(dev, "%s\n", __func__);
}

static int
mtkswitch_ifmedia_upd(if_t ifp)
{
        struct mtkswitch_softc *sc = if_getsoftc(ifp);
        struct mii_data *mii = mtkswitch_miiforport(sc, if_getdunit(ifp));

        if (mii == NULL)
                return (ENXIO);
        mii_mediachg(mii);
        return (0);
}

static void
mtkswitch_ifmedia_sts(if_t ifp, struct ifmediareq *ifmr)
{
        struct mtkswitch_softc *sc = if_getsoftc(ifp);
        struct mii_data *mii = mtkswitch_miiforport(sc, if_getdunit(ifp));

        DPRINTF(sc->sc_dev, "%s\n", __func__);

        if (mii == NULL)
                return;
        mii_pollstat(mii);
        ifmr->ifm_active = mii->mii_media_active;
        ifmr->ifm_status = mii->mii_media_status;
}

static int
mtkswitch_getconf(device_t dev, etherswitch_conf_t *conf)
{
        struct mtkswitch_softc *sc;

        sc = device_get_softc(dev);

        /* Return the VLAN mode. */
        conf->cmd = ETHERSWITCH_CONF_VLAN_MODE;
        conf->vlan_mode = sc->vlan_mode;

        return (0);
}

static int
mtkswitch_setconf(device_t dev, etherswitch_conf_t *conf)
{
        struct mtkswitch_softc *sc;
        int err;

        sc = device_get_softc(dev);

        /* Set the VLAN mode. */
        if (conf->cmd & ETHERSWITCH_CONF_VLAN_MODE) {
                err = mtkswitch_set_vlan_mode(sc, conf->vlan_mode);
                if (err != 0)
                        return (err);
        }

        return (0);
}

static int
mtkswitch_getvgroup(device_t dev, etherswitch_vlangroup_t *e)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_vlan_getvgroup(sc, e));
}

static int
mtkswitch_setvgroup(device_t dev, etherswitch_vlangroup_t *e)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_vlan_setvgroup(sc, e));
}

static int
mtkswitch_readphy(device_t dev, int phy, int reg)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_phy_read(dev, phy, reg));
}

static int
mtkswitch_writephy(device_t dev, int phy, int reg, int val)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_phy_write(dev, phy, reg, val));
}

static int
mtkswitch_readreg(device_t dev, int addr)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_reg_read(dev, addr));
}

static int
mtkswitch_writereg(device_t dev, int addr, int value)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);

        return (sc->hal.mtkswitch_reg_write(dev, addr, value));
}

static device_method_t mtkswitch_methods[] = {
    /* Device interface */
    DEVMETHOD(device_identify,	mtkswitch_identify),
    DEVMETHOD(device_probe,		mtkswitch_probe),
    DEVMETHOD(device_attach,	mtkswitch_attach),
    DEVMETHOD(device_detach,	mtkswitch_detach),

    /* bus interface */
    DEVMETHOD(bus_add_child,	device_add_child_ordered),

    /* MII interface */
    DEVMETHOD(miibus_readreg,	mtkswitch_readphy),
    DEVMETHOD(miibus_writereg,	mtkswitch_writephy),
    DEVMETHOD(miibus_statchg,	mtkswitch_statchg),

    /* etherswitch interface */
    DEVMETHOD(etherswitch_lock,	mtkswitch_lock),
    DEVMETHOD(etherswitch_unlock,	mtkswitch_unlock),
    DEVMETHOD(etherswitch_getinfo,	mtkswitch_getinfo),
    DEVMETHOD(etherswitch_readreg,	mtkswitch_readreg),
    DEVMETHOD(etherswitch_writereg,	mtkswitch_writereg),
    DEVMETHOD(etherswitch_readphyreg,	mtkswitch_readphy),
    DEVMETHOD(etherswitch_writephyreg,	mtkswitch_writephy),
    DEVMETHOD(etherswitch_getport,	mtkswitch_getport),
    DEVMETHOD(etherswitch_setport,	mtkswitch_setport),
    DEVMETHOD(etherswitch_getvgroup,	mtkswitch_getvgroup),
    DEVMETHOD(etherswitch_setvgroup,	mtkswitch_setvgroup),
    DEVMETHOD(etherswitch_getconf,	mtkswitch_getconf),
    DEVMETHOD(etherswitch_setconf,	mtkswitch_setconf),
    DEVMETHOD(etherswitch_fetch_table,		mt7531_atu_fetch_table),
    DEVMETHOD(etherswitch_fetch_table_entry,	mt7531_atu_fetch_table_entry),

    DEVMETHOD_END
};

DEFINE_CLASS_0(mtkswitch, mtkswitch_driver, mtkswitch_methods,
sizeof(struct mtkswitch_softc));

DRIVER_MODULE(mtkswitch, mtmdio, mtkswitch_driver, 0, 0);
DRIVER_MODULE(mtkswitch, mdio, mtkswitch_driver, 0, 0);
DRIVER_MODULE(miibus, mtkswitch, miibus_driver, 0, 0);
DRIVER_MODULE(etherswitch, mtkswitch, etherswitch_driver, 0, 0);
MODULE_VERSION(mtkswitch, 1);
MODULE_DEPEND(mtkswitch, mdio, 1, 1, 1);
MODULE_DEPEND(mtkswitch, miibus, 1, 1, 1);
MODULE_DEPEND(mtkswitch, etherswitch, 1, 1, 1);
