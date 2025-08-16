/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2025 Martin Filla
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
#include <sys/rman.h>

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
#include <dev/ofw/ofw_bus_subr.h>
#include "mdio_if.h"
#include "miibus_if.h"
#include "etherswitch_if.h"

#define DRVNAME     "mt7531"
#define MT7531_PHY_ADDR_DEFAULT   0x1f
#define MII_MMD_CTRL              0x0d
#define MII_MMD_DATA              0x0e
#define MDIO_MMD_VEND2            0x1f
#define MDIO_MMD_CTRL_ADDR        0x0000
#define MDIO_MMD_CTRL_DATA_NOINCR 0x4000

/* Port MAC Control/Status base  */
#define MT753X_PMCR_P(p)  (0x3000 + ((p) * 0x100))
#define MT753X_PMSR_P(p)  (0x3008 + ((p) * 0x100))

/* PMCR bits */
#define PMCR_TX_EN        (1U << 14)
#define PMCR_RX_EN        (1U << 13)
#define PMCR_FORCE_MODE   (1U << 15)
#define PMCR_FORCE_SPEED_1000 (1U << 3)
#define PMCR_FORCE_FDX    (1U << 1)

/* PMSR bits (výběr) */
#define PMSR_LINK         (1U << 0)
#define PMSR_SPEED_MASK   (3U << 2)  /* 00=10M, 01=100M, 10=1000M */
#define PMSR_FDX          (1U << 1)

/* Helpers */
#define REG32(lo,hi)    (((uint32_t)(hi) << 16) | ((uint32_t)(lo) & 0xffff))

#define MT7531_SWITCH_MAX_PORTS 7
#define MT7531_SWITCH_MAX_PHYS  5
#define MT7531_SWITCH_HDR_LEN   4
#define MT7530_SWITCH_NUM_FDB_RECORDS  2048
#define MT7530_SWITCH_ALL_MEMBERS      0xff

struct mt7531_switch_softc {
    struct mtx	mtx;
    device_t	dev;
    struct resource *res;
    int		numphys;
    uint32_t	phymap;
    int		numports;
    uint32_t	portmap;
    int		cpuport;
    uint32_t	valid_vlans;
    //mtk_switch_type	sc_switchtype;
    char		*ifname[MT7531_SWITCH_MAX_PHYS];
    device_t	miibus[MT7531_SWITCH_MAX_PHYS];
    if_t	*ifp[MT7531_SWITCH_MAX_PHYS];
    struct callout	callout_tick;
    etherswitch_info_t info;
    uint32_t	vlan_mode;

    struct {
        /* Global setup */
        int (* mt7531_switch_reset) (struct mt7531_switch_softc *);
        int (* mt7531_switch_hw_setup) (struct mt7531_switch_softc *);
        int (* mt7531_switch_hw_global_setup) (struct mt7531_switch_softc *);

        /* Port functions */
        void (* mt7531_switchport_init) (struct mt7531_switch_softc *, int);
        uint32_t (* mt7531_switch_get_port_status) (struct mt7531_switch_softc *, int);

        /* ATU functions */
        int (* mt7531_switch_atu_flush) (struct mt7531_switch_softc *);

        /* VLAN functions */
        int (* mt7531_switch_port_vlan_setup) (struct mt7531_switch_softc *,
                                               etherswitch_port_t *);
        int (* mt7531_switch_port_vlan_get) (struct mt7531_switch_softc *,
                                             etherswitch_port_t *);
        void (* mt7531_switch_vlan_init_hw) (struct mt7531_switch_softc *);
        int (* mt7531_switch_vlan_getvgroup) (struct mt7531_switch_softc *,
                                              etherswitch_vlangroup_t *);
        int (* mt7531_switch_vlan_setvgroup) (struct mt7531_switch_softc *,
                                              etherswitch_vlangroup_t *);
        int (* mt7531_switch_vlan_get_pvid) (struct mt7531_switch_softc *,
                                             int, int *);
        int (* mt7531_switch_vlan_set_pvid) (struct mt7531_switch_softc *,
                                             int, int);

        /* PHY functions */
        int (* mt7531_switch_phy_read) (device_t, int, int);
        int (* mt7531_switch_phy_write) (device_t, int, int, int);

        /* Register functions */
        int (* mt7531_switch_reg_read) (device_t, int);
        int (* mt7531_switch_reg_write) (device_t, int, int);

        /* Internal register access functions */
        uint32_t (* mt7531_switch_read) (struct mt7531_switch_softc *, int);
        uint32_t (* mt7531_switchwrite) (struct mt7531_switch_softc *, int,
                                         uint32_t);
    } hal;
};

static struct ofw_compat_data compat_data[] = {
        { "mediatek,mt7531",	1 },
        { NULL, 0 }
};

static inline void
mt7531_lock(struct mt7531_switch_softc *sc) { mtx_lock(&sc->mtx); }
static inline void
mt7531_unlock(struct mt7531_switch_softc *sc) { mtx_unlock(&sc->mtx); }

/* Clause-45 MMD 16-bit read/write on devad=VEND2 */
static int
mt7531_mmd_read16(struct mt7531_switch_softc *sc, uint16_t reg, uint16_t *val)
{
    //int pa = sc->phy_addr;

    /* Set MMD device (VEND2) and target address */
    //MDIO_WRITEREG(sc->dev, pa, MII_MMD_CTRL, MDIO_MMD_VEND2);
    //MDIO_WRITEREG(sc->dev, pa, MII_MMD_DATA, reg);
    /* Switch to data (no incr) */
    //MDIO_WRITEREG(sc->dev, pa, MII_MMD_CTRL,
    // MDIO_MMD_VEND2 | MDIO_MMD_CTRL_DATA_NOINCR);
    //*val = MDIO_READREG(sc->dev, pa, MII_MMD_DATA);
    return (0);
}

static int
mt7531_mmd_write16(struct mt7531_switch_softc *sc, uint16_t reg, uint16_t val)
{
    /*int pa = sc->phy_addr;

    MDIO_WRITEREG(sc->dev, pa, MII_MMD_CTRL, MDIO_MMD_VEND2);
    MDIO_WRITEREG(sc->dev, pa, MII_MMD_DATA, reg);
    MDIO_WRITEREG(sc->dev, pa, MII_MMD_CTRL,
                  MDIO_MMD_VEND2 | MDIO_MMD_CTRL_DATA_NOINCR);
    MDIO_WRITEREG(sc->dev, pa, MII_MMD_DATA, val);*/
    return (0);
}
static int
mt7531_read32(struct mt7531_switch_softc *sc, uint16_t reg, uint32_t *val)
{
    uint16_t lo, hi;
    mt7531_mmd_read16(sc, reg + 0, &lo);
    mt7531_mmd_read16(sc, reg + 1, &hi);
    *val = REG32(lo, hi);
    return (0);
}

static int
mt7531_write32(struct mt7531_switch_softc *sc, uint16_t reg, uint32_t v)
{
    mt7531_mmd_write16(sc, reg + 0, (uint16_t)(v & 0xffff));
    mt7531_mmd_write16(sc, reg + 1, (uint16_t)(v >> 16));
    return (0);
}

static etherswitch_info_t *
mt7531_getinfo(device_t dev)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    return (&sc->info);
}

static int
mt7531_getport(device_t dev, etherswitch_port_t *p)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    uint32_t pmsr = 0;
    int port = p->es_port;

    if (port < 0 || port > 6)
        return (EINVAL);

    mt7531_lock(sc);
    mt7531_read32(sc, (uint16_t)MT753X_PMSR_P(port), &pmsr);
    mt7531_unlock(sc);

    p->es_pvid = 1;
    p->es_flags = 0;

    /*p->es_link = (pmsr & PMSR_LINK) != 0;
    p->es_duplex = (pmsr & PMSR_FDX) ? 1 : 0;

    switch ((pmsr & PMSR_SPEED_MASK) >> 2) {
        case 0: p->es_speed = 10; break;
        case 1: p->es_speed = 100; break;
        case 2: p->es_speed = 1000; break;
        default: p->es_speed = 0; break;
    }*/
    return (0);
}

static int
mt7531_setport(device_t dev, etherswitch_port_t *p)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    int port = p->es_port;
    uint32_t pmcr = 0;

    if (port < 0 || port > 6)
        return (EINVAL);

    /* Force MAC side for CPU/uplink a basic 1G full-duplex */
    pmcr = PMCR_FORCE_MODE | PMCR_TX_EN | PMCR_RX_EN | PMCR_FORCE_FDX | PMCR_FORCE_SPEED_1000;

    mt7531_lock(sc);
    mt7531_write32(sc, (uint16_t)MT753X_PMCR_P(port), pmcr);
    mt7531_unlock(sc);

    return (0);
}

static int
mt7531_getvlangroup(device_t dev, etherswitch_vlangroup_t *vg)
{
    vg->es_vid = vg->es_vlangroup + 1;
    vg->es_member_ports = 0;
    vg->es_untagged_ports = 0;
    return (0);
}

static int
mt7531_setvlangroup(device_t dev, etherswitch_vlangroup_t *vg)
{
    return (0);
}

static void
mt7531_reset(struct mt7531_switch_softc *sc)
{
}

static int
mt7531_readphy(device_t dev, int phy, int reg)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);

    return (sc->hal.mt7531_switch_phy_read(dev, phy, reg));
}

static int
mt7531_writephy(device_t dev, int phy, int reg, int val)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);

    return (sc->hal.mt7531_switch_phy_write(dev, phy, reg, val));
}

static int
mt7531_readreg(device_t dev, int addr)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);

    return (sc->hal.mt7531_switch_reg_read(dev, addr));
}

static int
mt7531_writereg(device_t dev, int addr, int value)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);

    return (sc->hal.mt7531_switch_reg_write(dev, addr, value));
}

static void
mt7531_statchg(device_t dev)
{
    device_printf(dev, "func %s\n", __func__ );
}

static int
mt7531_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "MediaTek MT7531 Gigabit Switch");
    return (BUS_PROBE_DEFAULT);
}

static int
mt7531_attach(device_t dev)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    //phandle_t node = ofw_bus_get_node(dev);
    int err, rid;

    sc->dev = dev;
    /* Allocate resources */
    rid = 0;
    sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
    if (sc->res == NULL) {
        device_printf(dev, "Could not map memory\n");
        return (ENXIO);
    }

    mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

    if (sc->hal.mt7531_switch_reset(sc)) {
        device_printf(dev, "%s: mtkswitch_reset: failed\n", __func__);
        return (ENXIO);
    }

    err = sc->hal.mt7531_switch_hw_setup(sc);
    device_printf(dev, "%s: hw_setup: err=%d\n", __func__, err);
    if (err != 0) {
        return (err);
    }

    err = sc->hal.mt7531_switch_hw_global_setup(sc);
    device_printf(dev, "%s: hw_global_setup: err=%d\n", __func__, err);
    if (err != 0) {
        return (err);
    }

    device_printf(dev, "Inicialize device....");

    return (0);
}

static int
mt7531_detach(device_t dev)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    mtx_destroy(&sc->mtx);
    return (0);
}

static device_method_t mt7531_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,     mt7531_probe),
        DEVMETHOD(device_attach,    mt7531_attach),
        DEVMETHOD(device_detach,    mt7531_detach),

        /* bus interface */
        DEVMETHOD(bus_add_child,	device_add_child_ordered),

        /* etherswitch interface */
        DEVMETHOD(etherswitch_getinfo,      mt7531_getinfo),
        DEVMETHOD(etherswitch_readreg,      mt7531_readreg),
        DEVMETHOD(etherswitch_writereg,     mt7531_writereg),
        DEVMETHOD(etherswitch_getport,      mt7531_getport),
        DEVMETHOD(etherswitch_setport,      mt7531_setport),
        DEVMETHOD(etherswitch_getvgroup, mt7531_getvlangroup),
        DEVMETHOD(etherswitch_setvgroup, mt7531_setvlangroup),

        /* MII interface */
        DEVMETHOD(miibus_readreg,	mt7531_readphy),
        DEVMETHOD(miibus_writereg,	mt7531_writephy),
        DEVMETHOD(miibus_statchg,	mt7531_statchg),

        /* MDIO interface */
        DEVMETHOD(mdio_readreg,		mt7531_readphy),
        DEVMETHOD(mdio_writereg,	mt7531_writephy),

        DEVMETHOD_END
};


DEFINE_CLASS_0(mt7531_switch, mt7531_switch_driver, mt7531_methods, sizeof(struct mt7531_switch_softc));
DRIVER_MODULE(mt7531_switch, simplebus, mt7531_switch_driver, 0, 0);
DRIVER_MODULE(miibus, mt7531_switch, miibus_driver, 0, 0);
DRIVER_MODULE(mdio, mt7531_switch, mdio_driver, 0, 0);
DRIVER_MODULE(etherswitch, mt7531_switch, etherswitch_driver, 0, 0);
MODULE_VERSION(mt7531_switch, 1);
MODULE_DEPEND(mt7531_switch, miibus, 1, 1, 1);
MODULE_DEPEND(mt7531_switch, etherswitch, 1, 1, 1);
