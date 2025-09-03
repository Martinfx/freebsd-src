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

/* ---------- Helpers/macros ---------- */

#define MT7531_NAME           "mt7531"
#define MT7531_DEFAULT_ADDR   0x1f      /* Typical switch MDIO "phy" addr */

#define MDIO_READ(dev, addr, reg) \
  MDIO_READREG(device_get_parent(dev), (addr), (reg))
#define MDIO_WRITE(dev, addr, reg, val) \
  MDIO_WRITEREG(device_get_parent(dev), (addr), (reg), (val))

/* Clause 45 registers (std) */
#define MII_MMD_CTRL          0x0d
#define MII_MMD_ADDR          0x0e
#define MMD_FUNC_ADDR         0x0000
#define MMD_FUNC_DATA_NOINC   0x4000

/* MTK uses vendor MMD 2 for core/PCS space */
#define MDIO_MMD_VEND2        0x1e

/* SGMII/PCS base (C45 space in manual) */
#define MT7531_PCS_BASE       0x5000
#define PCS_CONTROL_1         (MT7531_PCS_BASE + 0x000) /* 0x5000 */
#define PCS_LINK_TIMER        (MT7531_PCS_BASE + 0x018) /* 0x5018 */
#define SGMII_MODE            (MT7531_PCS_BASE + 0x020) /* 0x5020 */
#define PCS_QPHY_PWR_STATE    (MT7531_PCS_BASE + 0x0E8) /* 0x50E8 */
#define PCS_GEN2_SPEED        (MT7531_PCS_BASE + 0x128) /* 0x5128 */

/* Bits (per MT7531 manual) */
#define PCS_CTRL1_RESTART_AN  (1 << 9)
#define PCS_CTRL1_AN_ENABLE   (1 << 12)

#define QPHY_PWRDN_BIT        (1 << 4)      /* 0x50E8[4] */
#define GEN2_2P5G_BIT         (1 << 2)      /* 0x5128[2]=1 -> 2.5G (Gen2) */

#define SGMII_FORCE_MODE      (1 << 1)      /* 0x5020[1]=0 for force mode? (manual: set 0) */
#define SGMII_DUPLEX_BIT      (1 << 4)      /* 0=full in manual’s wording; we’ll map explicit */
#define SGMII_SPD_SHIFT       2             /* 0x5020[3:2]: 00=10M,01=100M,10=1000/2500M */
#define SGMII_REMOTE_FAULT_DIS (1 << 8)     /* 0x5020[8]=1 in AN guide */

/* MAC control block (per-port), port 0 base at 0x3000, stride 0x100 */
#define GMAC_BASE_P0          0x3000
#define GMAC_STRIDE           0x100
#define PMCR(ofs_port)        (GMAC_BASE_P0 + (ofs_port) * GMAC_STRIDE + 0x000) /* PMCR_Pn */
#define GMACCR                0x30E0

/* Very small subset of PMCR bits we care about (naming illustrative) */
#define PMCR_FORCE_MODE_LNK   (1U << 31)
#define PMCR_FORCE_MODE_SPD   (1U << 30)
#define PMCR_FORCE_MODE_DPX   (1U << 29)
#define PMCR_FORCE_LNK_UP     (1U << 0)
#define PMCR_FORCE_DPX_FULL   (1U << 5)
#define PMCR_FORCE_SPD_1000   (1U << 4)     /* Simplified example */

#define MT7531_SWITCH_MAX_PORTS 7
#define MT7531_SWITCH_MAX_PHYS  5

/* ---------- Softc ---------- */

struct mt7531_softc {
    device_t      dev;
    struct mtx     mtx;
    int           sw_addr;
    int           cpu_port;
    enum { IF_RGMII, IF_SGMII, IF_2500X } cpu_if;
    bool          an_enable;
    uint32_t	valid_vlans;
    char		*ifname[MT7531_SWITCH_MAX_PHYS];
    device_t	miibus[MT7531_SWITCH_MAX_PHYS];
    if_t ifp[MT7531_SWITCH_MAX_PHYS];
    struct callout	callout_tick;
    etherswitch_info_t info;
    uint32_t	vlan_mode;
    int         mdio_addr;
};

/* ---------- C45 MMD helpers (16/32-bit) ---------- */

static int
c45_write16(struct mt7531_softc *sc, uint16_t reg, uint16_t val)
{
    device_t dev = sc->dev;
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_CTRL, (MMD_FUNC_ADDR | MDIO_MMD_VEND2));
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_ADDR, reg);
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_CTRL, (MMD_FUNC_DATA_NOINC | MDIO_MMD_VEND2));
    return MDIO_WRITE(dev, sc->sw_addr, MII_MMD_ADDR, val);
}

static uint16_t
c45_read16(struct mt7531_softc *sc, uint16_t reg)
{
    device_t dev = sc->dev;
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_CTRL, (MMD_FUNC_ADDR | MDIO_MMD_VEND2));
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_ADDR, reg);
    MDIO_WRITE(dev, sc->sw_addr, MII_MMD_CTRL, (MMD_FUNC_DATA_NOINC | MDIO_MMD_VEND2));
    return (uint16_t)MDIO_READ(dev, sc->sw_addr, MII_MMD_ADDR);
}

static int
c45_write32(struct mt7531_softc *sc, uint16_t reg, uint32_t val)
{
    int rc;
    rc  = c45_write16(sc, reg + 0, (uint16_t)(val & 0xffff));
    rc |= c45_write16(sc, reg + 1, (uint16_t)((val >> 16) & 0xffff));
    return rc;
}

static uint32_t
c45_read32(struct mt7531_softc *sc, uint16_t reg)
{
    uint32_t lo = c45_read16(sc, reg + 0);
    uint32_t hi = c45_read16(sc, reg + 1);
    return (lo | (hi << 16));
}

/* ---------- SGMII/PCS setup for CPU port ---------- */

static void
mt7531_setup_pcs(struct mt7531_softc *sc)
{
    /* Power down PHY analog block during config */
    uint32_t v;
    v = c45_read32(sc, PCS_QPHY_PWR_STATE);
    v |= QPHY_PWRDN_BIT;
    c45_write32(sc, PCS_QPHY_PWR_STATE, v);

    if (sc->an_enable) {
        /* AN mode (1G typical): GEN2=0, RF disable, restart AN */
        v = c45_read32(sc, PCS_GEN2_SPEED);
        v &= ~GEN2_2P5G_BIT;                    /* 1G path */
        c45_write32(sc, PCS_GEN2_SPEED, v);

        v = c45_read32(sc, SGMII_MODE);
        v |= SGMII_REMOTE_FAULT_DIS;            /* disable remote fault injection */
        c45_write32(sc, SGMII_MODE, v);

        v = c45_read32(sc, PCS_CONTROL_1);
        v |= (PCS_CTRL1_AN_ENABLE | PCS_CTRL1_RESTART_AN);
        c45_write32(sc, PCS_CONTROL_1, v);

    } else {
        /* Force mode:
         *  - 2.5G when IF_2500X (set GEN2[2]=1), else 1G/100M/10M (GEN2[2]=0)
         *  - SGMII_MODE[3:2] = 10 for 1000/2500
         *  - SGMII_MODE[4] duplex (0 = full)
         *  - SGMII_MODE[1] = 0 (force mode enable per manual wording)
         */
        v = c45_read32(sc, PCS_GEN2_SPEED);
        if (sc->cpu_if == IF_2500X)
            v |= GEN2_2P5G_BIT;
        else
            v &= ~GEN2_2P5G_BIT;
        c45_write32(sc, PCS_GEN2_SPEED, v);

        v = c45_read32(sc, SGMII_MODE);
        v &= ~(0x3 << SGMII_SPD_SHIFT);         /* clear speed */
        v |= (2 << SGMII_SPD_SHIFT);            /* 10b -> 1000/2500 */
        v &= ~SGMII_DUPLEX_BIT;                 /* full duplex */
        v &= ~SGMII_FORCE_MODE;                 /* set 0 per Force-mode guide */
        c45_write32(sc, SGMII_MODE, v);

        v = c45_read32(sc, PCS_CONTROL_1);
        v &= ~PCS_CTRL1_AN_ENABLE;              /* AN disable */
        c45_write32(sc, PCS_CONTROL_1, v);
    }

    /* Link timer sane default (datasheet suggests 0x0098968 etc.) */
    c45_write32(sc, PCS_LINK_TIMER, c45_read32(sc, PCS_LINK_TIMER));

    /* Release power-down */
    v = c45_read32(sc, PCS_QPHY_PWR_STATE);
    v &= ~QPHY_PWRDN_BIT;
    c45_write32(sc, PCS_QPHY_PWR_STATE, v);
}

/* Minimal MAC enable on chosen CPU port (force link UP) */
static void
mt7531_enable_cpu_port(struct mt7531_softc *sc)
{
    int p = sc->cpu_port;               /* 5 or 6 */
    uint32_t pmcr = 0;

    /* Force link up at 1G (or 2.5G is handled by PCS/SGMII side) */
    pmcr |= PMCR_FORCE_MODE_LNK | PMCR_FORCE_MODE_SPD | PMCR_FORCE_MODE_DPX;
    pmcr |= PMCR_FORCE_LNK_UP | PMCR_FORCE_DPX_FULL | PMCR_FORCE_SPD_1000;

    c45_write32(sc, PMCR(p), pmcr);

    /* Optional: global MAC control tweaks (IFG/EEE), keep defaults */
    (void)c45_read32(sc, GMACCR);
}

/* ---------- etherswitch(4) skeleton ---------- */

static etherswitch_info_t esw_info = {
        .es_nports      = 7,             /* P0..P6 */
        .es_nvlangroups = 0,
        .es_vlan_caps   = ETHERSWITCH_VLAN_DOT1Q | ETHERSWITCH_VLAN_PORT,
        .es_name        = "MediaTek MT7531 (minimal)"
};

static etherswitch_info_t *
mt7531_getinfo(device_t dev)
{
    return &esw_info;
}

static int
mt7531_getport(device_t dev, etherswitch_port_t *p)
{
    /* TODO: read PMSR_Pn etc. For now report link as unknown. */
    p->es_port = p->es_port;
    p->es_flags = 0;
    return (0);
}

static int
mt7531_setport(device_t dev, etherswitch_port_t *p)
{
    /* TODO: per-port enable/disable, speed/duplex */
    return (0);
}

/* ---------- newbus ---------- */

static int
mt7531_probe(device_t dev)
{
    if (!ofw_bus_is_compatible(dev, "mediatek,mt7531") &&
        !ofw_bus_is_compatible(dev, "mediatek,mt7531be"))
        return (ENXIO);

    device_set_desc(dev, "MediaTek MT7531 Switch");
    return (BUS_PROBE_DEFAULT);
}

static int
mt7531_attach(device_t dev)
{
    struct mt7531_softc *sc = device_get_softc(dev);
    phandle_t node = ofw_bus_get_node(dev);
    const char *ifmode = NULL;
    uint32_t cpu_port = 6;

    sc->dev = dev;
    mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

    /* MDIO address (reg), default 0x1f */
    if (OF_hasprop(node, "reg")) {
        OF_getencprop(node, "reg", &sc->sw_addr, sizeof(sc->sw_addr));
    }
    else {
        sc->sw_addr = MT7531_DEFAULT_ADDR;
    }

    /* Bring up PCS/SGMII and MAC for CPU port */
    mt7531_setup_pcs(sc);
    mt7531_enable_cpu_port(sc);

    device_printf(dev, "MT7531 ready at MDIO addr 0x%x, CPU port %d, mode %d, AN=%d\n",
                  sc->sw_addr, sc->cpu_port, sc->cpu_if, sc->an_enable);

    bus_identify_children(dev);
    bus_enumerate_hinted_children(dev);
    bus_attach_children(dev);
    device_printf(dev, "%s: bus_generic_attach: err=%d\n", __func__, err);

    return (0);
}

static int
mt7531_detach(device_t dev)
{
    struct mt7531_switch_softc *sc = device_get_softc(dev);
    mtx_destroy(&sc->mtx);
    return (0);
}

/* Methods */
static device_method_t mt7531_methods[] = {
        /* device */
        DEVMETHOD(device_probe,   mt7531_probe),
        DEVMETHOD(device_attach,  mt7531_attach),
        DEVMETHOD(device_detach,  mt7531_detach),

        /* etherswitch minimal */
        DEVMETHOD(etherswitch_getinfo,  mt7531_getinfo),
        DEVMETHOD(etherswitch_getport,  mt7531_getport),
        DEVMETHOD(etherswitch_setport,  mt7531_setport),

        DEVMETHOD_END
};


DEFINE_CLASS_0(mt7531_switch, mt7531_switch_driver, mt7531_methods, sizeof(struct mt7531_switch_softc));
DRIVER_MODULE(mt7531_switch, mdio, mt7531_switch_driver, 0, 0);
MODULE_DEPEND(mt7531_switch, mdio, 1, 1, 1);
DRIVER_MODULE(etherswitch, mt7531_switch, etherswitch_driver, 0, 0);
MODULE_VERSION(mt7531_switch, 1);
MODULE_VERSION(mt7531_switch, 1);
