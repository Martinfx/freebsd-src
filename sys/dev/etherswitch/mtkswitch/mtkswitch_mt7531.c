/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 * Copyright (c) 2023 Priit Trees.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sbuf.h>
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

#include <sys/gpio.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/etherswitch/etherswitch.h>
#include <dev/etherswitch/mtkswitch/mtkswitchvar.h>
#include <dev/etherswitch/mtkswitch/mtkswitch_mt7531.h>

#include "etherswitch_if.h"
#include "miibus_if.h"
#include "mdio_if.h"

#define MDIO_READ(dev, addr, reg)                                       \
   MDIO_READREG(device_get_parent(dev), (addr), (reg))
#define MDIO_WRITE(dev, addr, reg, val)                                 \
   MDIO_WRITEREG(device_get_parent(dev), (addr), (reg), (val))

/* ~20 ms at 10 us per iteration. */
#define	MTKSWITCH_BUSY_RETRIES	2000

/* ~1 s at 1 ms per iteration, matching Linux' post-reset settling timeout. */
#define	MTKSWITCH_RESET_RETRIES	1000

/*
 * Poll a switch register until the given busy bit(s) clear, with a bounded
 * timeout.  The switch is reached over MDIO, so a missing or wedged chip reads
 * back as all-ones; without a timeout the BUSY/ACS poll loops (e.g. MT7531_ATC_BUSY,
 * which is also set in a 0xffff read) would spin forever and hang the kernel.
 * Returns the last register value read.
 */
static uint32_t
mtkswitch_busy_wait(struct mtkswitch_softc *sc, int reg, uint32_t mask)
{
        uint32_t val = mask;
        int retry;

        for (retry = MTKSWITCH_BUSY_RETRIES; retry > 0; retry--) {
                val = sc->hal.mtkswitch_read(sc, reg);
                if ((val & mask) == 0)
                        return (val);
                DELAY(10);
        }

        device_printf(sc->sc_dev, "timeout waiting for reg 0x%x mask 0x%x\n",
            reg, mask);
        return (val);
}

static int
mtkswitch_phy_read_locked(struct mtkswitch_softc *sc, int phy, int reg)
{
        uint32_t data;

        mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);

        sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC,
            PHY_ACS_ST | PHY_MDIO_ST | (reg << PHY_MDIO_REG_ADDR_OFF) |
            (phy << PHY_MDIO_PHY_ADDR_OFF) | PHY_MDIO_CMD_READ);

        data = mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);

        return ((int)(data & PHY_MDIO_RW_DATA_MASK));

}

static int
mtkswitch_phy_read(device_t dev, int phy, int reg)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);
        int data;

        if ((phy < 0 || phy >= 32) || (reg < 0 || reg >= 32))
                return (0xffff);

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);
        data = mtkswitch_phy_read_locked(sc, phy, reg);
        MTKSWITCH_UNLOCK(sc);

        return (data);
}

static int
mtkswitch_phy_write_locked(struct mtkswitch_softc *sc, int phy, int reg,
                           int val)
{
        sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC,
            PHY_ACS_ST | PHY_MDIO_ST | (reg << PHY_MDIO_REG_ADDR_OFF) |
            (phy << PHY_MDIO_PHY_ADDR_OFF) | PHY_MDIO_CMD_WRITE |
            (val & PHY_MDIO_RW_DATA_MASK));
        mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);

        return (0);
}

static int
mtkswitch_phy_write(device_t dev, int phy, int reg, int val)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);
        int res;

        if ((phy < 0 || phy >= 32) || (reg < 0 || reg >= 32))
                return (ENXIO);

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);
        res = mtkswitch_phy_write_locked(sc, phy, reg, val);
        MTKSWITCH_UNLOCK(sc);

        return (res);
}

static uint32_t
mtkswitch_reg_read32(struct mtkswitch_softc *sc, int reg)
{
        uint32_t low, hi;

        MDIO_WRITE(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_GLOBAL_REG, MTKSWITCH_REG_ADDR(reg));
        low = MDIO_READ(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_REG_LO(reg));
        hi = MDIO_READ(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_REG_HI(reg));
        return (low | (hi << 16));
}

static uint32_t
mtkswitch_reg_write32(struct mtkswitch_softc *sc, int reg, uint32_t val)
{

        MDIO_WRITE(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_GLOBAL_REG, MTKSWITCH_REG_ADDR(reg));
        MDIO_WRITE(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_REG_LO(reg), MTKSWITCH_VAL_LO(val));
        MDIO_WRITE(sc->sc_dev, MTKSWITCH_GLOBAL_PHY,
            MTKSWITCH_REG_HI(reg), MTKSWITCH_VAL_HI(val));
        return (0);
}

static int
mtkswitch_reg_read(device_t dev, int reg)
{

        struct mtkswitch_softc *sc = device_get_softc(dev);
        uint32_t val;

        val = sc->hal.mtkswitch_read(sc, MTKSWITCH_REG32(reg));
        return val;
}

static int
mtkswitch_reg_write(device_t dev, int reg, int val)
{
        struct mtkswitch_softc *sc = device_get_softc(dev);
        sc->hal.mtkswitch_write(sc, MTKSWITCH_REG32(reg), val);

        return (0);
}

/*
 * Pulse the switch's reset line, when the device tree describes one
 * ("reset-gpios", GPIO 54 on the BananaPi R64).  Until this was done the chip
 * could still be held in reset by the boot loader or by the pin's power-on
 * default, in which case every indirect register read floats back as
 * 0xffffffff - which the driver happily mistook for a live switch.
 */
static void
mtkswitch_gpio_reset(struct mtkswitch_softc *sc)
{
       gpio_pin_t rstpin;

       if (sc->node == 0 || sc->node == (phandle_t)-1)
	       return;
       if (gpio_pin_get_by_ofw_property(sc->sc_dev, sc->node, "reset-gpios",
	   &rstpin) != 0)
	       return;

       if (gpio_pin_setflags(rstpin, GPIO_PIN_OUTPUT) != 0) {
	       gpio_pin_release(rstpin);
	       return;
       }

       /* Assert for >1 ms, then release and let the chip settle. */
       gpio_pin_set_active(rstpin, false);
       DELAY(1100);
       gpio_pin_set_active(rstpin, true);
       gpio_pin_release(rstpin);
       DELAY(1000);
}

static int
mtkswitch_reset(struct mtkswitch_softc *sc)
{
       uint32_t val;
       int i;

       mtkswitch_gpio_reset(sc);

       /*
	* Wait for the chip to answer on the MDIO bus.  A switch that is
	* absent, held in reset or unreachable reads back as all-zeroes or
	* all-ones, so require a plausible HWTRAP value before going on.
	*/
       val = 0;
       for (i = 0; i < MTKSWITCH_RESET_RETRIES; i++) {
	       val = sc->hal.mtkswitch_read(sc, MTKSWITCH_STRAP);
	       if (val != 0 && val != 0xffffffff)
		       break;
	       DELAY(1000);
       }
       if (i == MTKSWITCH_RESET_RETRIES) {
	       device_printf(sc->sc_dev,
		   "switch did not respond after reset (HWTRAP=0x%08x)\n", val);
	       return (ENXIO);
       }

       /*
	* Refuse to drive a chip we do not recognise rather than scribbling
	* over whatever is really on the bus.
	*/
       val = sc->hal.mtkswitch_read(sc, MTKSWITCH_CREV);
       if (CREV_CHIP_NAME(val) != MT7531_CHIP_ID) {
	       device_printf(sc->sc_dev,
		   "unexpected chip id 0x%04x (CREV=0x%08x)\n",
		   CREV_CHIP_NAME(val), val);
	       return (ENXIO);
       }

       /* All port MACs must be forced link-down before the internal reset. */
       for (i = 0; i < MTKSWITCH_MAX_PORTS; i++)
	       sc->hal.mtkswitch_write(sc, MTKSWITCH_PMCR(i),
		   MT7531_PMCR_FORCE_LINK);

       /* Reset the switch core, its registers and the internal PHYs. */
       sc->hal.mtkswitch_write(sc, MTKSWITCH_SYS_CTRL,
	   SYS_CTRL_PHY_RST | SYS_CTRL_SW_RST | SYS_CTRL_REG_RST);
       DELAY(20000);

        return (0);
}

static void
mtkswitch_setup_xtal(struct mtkswitch_softc *sc, uint32_t xtal)
{
        uint32_t val;

        /* Step 1 : Disable MT7531 COREPLL */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP);
        val &= ~PLLGP_COREPLL;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP, val);

        /* Step 2: switch to XTAL output */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP);
        val |= PLLGP_SW_CLKSW;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP, val);

        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val &= ~PLLGP_RG_COREPLL;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);

        /* Step 3: disable PLLGP and enable program PLLGP */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP);
        val |= PLLGP_SW_PLLGP;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP, val);

        /* Step 4: program COREPLL output frequency to 500MHz */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val &= ~PLLGP_RG_COREPLL_POSDIV_M;
        val |= 2 << PLLGP_RG_COREPLL_POSDIV_S;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);
        DELAY(35);

        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val &= ~PLLGP_RG_COREPLL_SDM_PCW_M;
        val |= xtal << PLLGP_RG_COREPLL_SDM_PCW_S;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);

        /* Set feedback divide ratio update signal to high */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val |= PLLGP_RG_COREPLL_SDM_PCW_CHG;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);
        /* Wait for at least 16 XTAL clocks */
        DELAY(20);

        /* Step 5: set feedback divide ratio update signal to low */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val &= ~PLLGP_RG_COREPLL_SDM_PCW_CHG;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);

        /* Enable 325M clock for SGMII */
        sc->hal.mtkswitch_write(sc, MT7531_ANA_PLLGP_CR5, 0xad0000);

        /* Enable 250SSC clock for RGMII */
        sc->hal.mtkswitch_write(sc, MT7531_ANA_PLLGP_CR2, 0x4f40000);

        /* Step 6: Enable MT7531 PLL */
        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP_CR0);
        val |= PLLGP_RG_COREPLL;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP_CR0, val);

        val = sc->hal.mtkswitch_read(sc, MT7531_PLLGP);
        val |= PLLGP_COREPLL;
        sc->hal.mtkswitch_write(sc, MT7531_PLLGP, val);
        DELAY(35);
}

/*
 * Clause 45 flavour of the indirect PHY access above.  It is only needed to
 * reach the vendor MMD that holds the internal PHYs' core PLL, before any
 * PHY device exists, so a minimal read/modify/write pair is enough.
 */
static uint32_t
mt7531_c45_phy_read(struct mtkswitch_softc *sc, int phy, int devad, int reg)
{
       uint32_t val;

       mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);
       sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC, PHY_ACS_ST |
	   PHY_MDIO_CL45_ADDR | (phy << PHY_MDIO_PHY_ADDR_OFF) |
	       (devad << PHY_MDIO_REG_ADDR_OFF) |
	       (reg & PHY_MDIO_RW_DATA_MASK));

       mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);
       sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC, PHY_ACS_ST |
	   PHY_MDIO_CL45_READ | (phy << PHY_MDIO_PHY_ADDR_OFF) |
	       (devad << PHY_MDIO_REG_ADDR_OFF));

       val = mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);

       return (val & PHY_MDIO_RW_DATA_MASK);
}

static void
mt7531_c45_phy_write(struct mtkswitch_softc *sc, int phy, int devad, int reg,
   uint32_t val)
{

       mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);
       sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC, PHY_ACS_ST |
	   PHY_MDIO_CL45_ADDR | (phy << PHY_MDIO_PHY_ADDR_OFF) |
	       (devad << PHY_MDIO_REG_ADDR_OFF) |
	       (reg & PHY_MDIO_RW_DATA_MASK));

       mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);
       sc->hal.mtkswitch_write(sc, MT7531_PHY_IAC, PHY_ACS_ST |
	   PHY_MDIO_CL45_WRITE | (phy << PHY_MDIO_PHY_ADDR_OFF) |
	       (devad << PHY_MDIO_REG_ADDR_OFF) |
	       (val & PHY_MDIO_RW_DATA_MASK));

       mtkswitch_busy_wait(sc, MT7531_PHY_IAC, PHY_ACS_ST);
}

/*
 * Take the internal gigabit PHYs' core PLL out of power-down.  SYS_CTRL's
 * PHY reset leaves it off, so without this none of the five user ports ever
 * links up.
 */
static void
mtkswitch_phy_pll_enable(struct mtkswitch_softc *sc)
{
       uint32_t val;

       val = mt7531_c45_phy_read(sc, MT7531_CTRL_PHY_ADDR, MT7531_MMD_VEND2,
	   MT7531_CORE_PLL_GROUP4);
       val |= PHY_PLL_BYPASS_MODE;
       val &= ~PHY_PLL_OFF;
       mt7531_c45_phy_write(sc, MT7531_CTRL_PHY_ADDR, MT7531_MMD_VEND2,
	   MT7531_CORE_PLL_GROUP4, val);
}

static int
mtkswitch_hw_setup(struct mtkswitch_softc *sc)
{

        /*
         * TODO: parse the device tree and see if we need to configure
         *       ports, etc. differently. For now we fallback to defaults.
         */

       uint32_t crev, topsig, hwstrap, xtal;

        crev = sc->hal.mtkswitch_read(sc, MTKSWITCH_CREV);
        topsig  = sc->hal.mtkswitch_read(sc, MT7531_TOP_SIG_SR);
       hwstrap = sc->hal.mtkswitch_read(sc, MTKSWITCH_STRAP);

        /* Print chip name and revision. In future need
         * to move other place and mayby can use it.
         */
        device_printf(sc->sc_dev, "chip %s rev 0x%x\n",
            topsig & PAD_DUAL_SGMII ? "MT7531AE" : "MT7531BE",
            CREV_CHIP_REV(crev));

       /*
	* MT7531AE has got two SGMII units. One for port 5, one for port 6.
	* MT7531BE has got only one SGMII unit which is for port 6.  Only the
	* latter drives COREPLL from the crystal; on the AE variant the PLL is
	* left as strapped.
         */
       if ((topsig & PAD_DUAL_SGMII) == 0) {
	       /*
		* From revision 1 on, the crystal frequency is not strapped
		* into HWTRAP any more: it is 40 MHz when the chip is wired up
		* as an MCM over SMI and 25 MHz otherwise.
         */
	       if (CREV_CHIP_REV(crev) > 0)
		       xtal = (topsig & PAD_MCM_SMI) ? MT7531_XTAL_40MHZ :
			   MT7531_XTAL_25MHZ;
	       else if (hwstrap & STRAP_XTAL)	/* strap set: 25MHz */
                xtal = MT7531_XTAL_25MHZ;
	       else				/* strap clear: 40MHz */
                xtal = MT7531_XTAL_40MHZ;

        mtkswitch_setup_xtal(sc, xtal);
       }

       mtkswitch_phy_pll_enable(sc);

        /* Called early and hence unlocked */
        return (0);
}

/* Bitmap of the ports that act as trunks towards the host MAC. */
static uint32_t
mtkswitch_cpu_portmap(struct mtkswitch_softc *sc)
{
       uint32_t mask;
       int port;

       mask = 0;
       for (port = 0; port < sc->numports && port < MTKSWITCH_MAX_PORTS;
	   port++) {
	       if ((sc->portmap & (1u << port)) != 0 && sc->cpu_port[port])
		       mask |= (1u << port);
       }
       if (mask == 0 && sc->cpuport >= 0)
	       mask = (1u << sc->cpuport);

       return (mask);
}

static int
mtkswitch_hw_global_setup(struct mtkswitch_softc *sc)
{
       uint32_t cpumask, val;

       cpumask = mtkswitch_cpu_portmap(sc);

       /*
	* Tell the switch which ports the host MAC hangs off.  Frames the
	* switch traps are forwarded to the CPU port affine to the ingress
	* user port, so leaving this at its power-on value means nothing ever
	* reaches the MAC.
	*/
       val = sc->hal.mtkswitch_read(sc, MT7531_CFC);
       val &= ~CFC_CPU_PMAP_MASK;
       val |= CFC_CPU_PMAP(cpumask);
       sc->hal.mtkswitch_write(sc, MT7531_CFC, val);

       /*
	* Flood broadcast, unknown multicast and unknown unicast frames to the
	* CPU port(s) as well as to the user ports.  Without the CPU port in
	* these maps not even ARP or DHCP gets through.
	*/
       val = sc->hal.mtkswitch_read(sc, MT7531_MFC);
       val |= MFC_BC_FFP(cpumask) | MFC_UNM_FFP(cpumask) |
	   MFC_UNU_FFP(cpumask);
       sc->hal.mtkswitch_write(sc, MT7531_MFC, val);

       if (bootverbose)
	       device_printf(sc->sc_dev, "CFC=0x%08x MFC=0x%08x\n",
		   sc->hal.mtkswitch_read(sc, MT7531_CFC),
		   sc->hal.mtkswitch_read(sc, MT7531_MFC));

        /* Called early and hence unlocked */
        return (0);
}

/*
 * Bring up the SGMII PCS behind a trunk port.
 *
 * Ports 5 and 6 reach the SoC over a serdes rather than a PHY.  The switch
 * leaves that PCS unprogrammed after the SYS_CTRL reset in mtkswitch_reset(),
 * so it has to be set up here or the trunk never carries a frame - which on
 * this board means no traffic at all, since the entire data path runs over
 * the port 6 trunk to gmac0.
 *
 * Both links on this board are fixed (no in-band auto-negotiation), so the
 * PCS runs in BASE-X mode with AN disabled and the rate forced.  This follows
 * Linux' pcs-mtk-lynxi driver, which drives the very same PCS block.
 */
static void
mtkswitch_sgmii_setup(struct mtkswitch_softc *sc, int port)
{
       uint32_t base, val;

       base = MT7531_SGMII_BASE(port);

       /* Power the PHYA down and reset the PCS while it is reprogrammed. */
       val = sc->hal.mtkswitch_read(sc, base + SGMII_QPHY_PWR_STATE_CTRL);
       val |= SGMII_PHYA_PWD;
       sc->hal.mtkswitch_write(sc, base + SGMII_QPHY_PWR_STATE_CTRL, val);

       val = sc->hal.mtkswitch_read(sc, base + SGMII_RESERVED_0);
       val |= SGMII_SW_RESET;
       sc->hal.mtkswitch_write(sc, base + SGMII_RESERVED_0, val);

       /*
	* Serdes line rate.  2500base-x runs the same 8b/10b link 2.5 times
	* faster rather than using a different line code, which is why the
	* speed forced further down stays at the 1000 encoding.
	*/
       val = sc->hal.mtkswitch_read(sc, base + SGMII_PHYA_CTRL_SIGNAL3);
       val &= ~SGMII_PHY_SPEED_MASK;
       val |= (sc->port_mode[port] == MTK_PHY_MODE_2500BASEX) ?
	   SGMII_PHY_SPEED_3_125G : SGMII_PHY_SPEED_1_25G;
       sc->hal.mtkswitch_write(sc, base + SGMII_PHYA_CTRL_SIGNAL3, val);

       sc->hal.mtkswitch_write(sc, base + SGMII_PCS_LINK_TIMER,
	   sc->port_mode[port] == MTK_PHY_MODE_SGMII ?
	   SGMII_LINK_TIMER_SGMII : SGMII_LINK_TIMER_BASEX);

       /*
	* Clearing SGMII_IF_MODE_SGMII selects BASE-X, where the line rate is
	* the bit rate; together with AN off that gives a plain forced link.
	*/
       val = sc->hal.mtkswitch_read(sc, base + SGMII_MODE);
       val &= ~(SGMII_IF_MODE_SGMII | SGMII_SPEED_DUPLEX_AN |
	   SGMII_REMOTE_FAULT_DIS);
       sc->hal.mtkswitch_write(sc, base + SGMII_MODE, val);

       val = sc->hal.mtkswitch_read(sc, base + SGMII_PCS_CONTROL_1);
       val &= ~PCS_AN_ENABLE;
       sc->hal.mtkswitch_write(sc, base + SGMII_PCS_CONTROL_1, val);

       /*
	* Release the PHYA power-down.  Clearing SGMII_PHYA_PWD alone is not
	* enough: the register can come back holding other bits that let the
	* link come up but pass no traffic, so write the whole word.  The QPHY
	* needs a moment before it is written, or the release is racy.
	*/
       DELAY(100);
       sc->hal.mtkswitch_write(sc, base + SGMII_QPHY_PWR_STATE_CTRL, 0);

       /* Force the speed and duplex the fixed link runs at. */
       val = sc->hal.mtkswitch_read(sc, base + SGMII_MODE);
       val &= ~(SGMII_FORCE_SPEED_MASK | SGMII_DUPLEX_HALF);
       val |= SGMII_FORCE_SPEED_1000;
       sc->hal.mtkswitch_write(sc, base + SGMII_MODE, val);

       if (bootverbose)
	       device_printf(sc->sc_dev,
		   "port %d: SGMII PCS ctrl=0x%08x mode=0x%08x rgc3=0x%08x\n",
		   port,
		   sc->hal.mtkswitch_read(sc, base + SGMII_PCS_CONTROL_1),
		   sc->hal.mtkswitch_read(sc, base + SGMII_MODE),
		   sc->hal.mtkswitch_read(sc, base + SGMII_PHYA_CTRL_SIGNAL3));
}

static void
mtkswitch_port_init(struct mtkswitch_softc *sc, int port)
{
        uint32_t val;

        /* Called early and hence unlocked */

       /*
	* Set the port to secure mode and program its forwarding matrix
	* explicitly: every port may forward to every other port that the
	* device tree describes.  The matrix used to be left at whatever the
	* chip powered up with, which is not something to rely on.
	*/
        val = sc->hal.mtkswitch_read(sc, MTKSWITCH_PCR(port));
        val |= PCR_PORT_VLAN_SECURE;
       val &= ~PCR_MATRIX_MASK;
       /*
	* An all-zero matrix means the FDT ports parse never ran; fall back
	* to everyone-to-everyone rather than a port that forwards nothing.
	*/
       val |= PCR_MATRIX((sc->port_matrix[port] != 0) ?
	   sc->port_matrix[port] : (sc->portmap & ~(1u << port)));
        sc->hal.mtkswitch_write(sc, MTKSWITCH_PCR(port), val);

        /* Set port's vlan_attr to user port */
        val = sc->hal.mtkswitch_read(sc, MTKSWITCH_PVC(port));
        val &= ~PVC_VLAN_ATTR_MASK;
        sc->hal.mtkswitch_write(sc, MTKSWITCH_PVC(port), val);

        /*
	* A trunk port reached over a serdes needs its PCS programmed before
	* the MAC link below is forced up.
	*/
       if (port >= 5 && port <= 6 && MTK_PHY_MODE_IS_SERDES(sc->port_mode[port]))
	       mtkswitch_sgmii_setup(sc, port);

       val = PMCR_CFG_DEFAULT;
       if (sc->fixed_port[port] || port == sc->cpuport) {
                /*
		* A port with a "fixed-link" device-tree node - the CPU trunk
		* on port 6 and the second MAC on port 5 - has no PHY to
		* negotiate with, so force its link up.  The speed field is
		* kept at 1000 here to match the upstream Linux default; the
		* actual line rate (2500base-x on the BananaPi R64 trunk) is
                 * established by the SGMII PCS rather than this field.
                 */
                val |= PMCR_FORCE_LINK | PMCR_FORCE_DPX | PMCR_FORCE_SPD_1000 |
                       MT7531_PMCR_FORCE_MODE | PMCR_MAC_MODE;
        }
        /* Set port's MAC to default settings */
        sc->hal.mtkswitch_write(sc, MTKSWITCH_PMCR(port), val);

       if (bootverbose)
	       device_printf(sc->sc_dev,
		   "port %d: PCR=0x%08x PMCR=0x%08x\n", port,
		   sc->hal.mtkswitch_read(sc, MTKSWITCH_PCR(port)),
		   sc->hal.mtkswitch_read(sc, MTKSWITCH_PMCR(port)));
}

static uint32_t
mtkswitch_get_port_status(struct mtkswitch_softc *sc, int port)
{
        uint32_t val, res, tmp;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);
        res = 0;
        val = sc->hal.mtkswitch_read(sc, MTKSWITCH_PMSR(port));

        if (val & PMSR_MAC_LINK_STS)
                res |= MTKSWITCH_LINK_UP;
        if (val & PMSR_MAC_DPX_STS)
                res |= MTKSWITCH_DUPLEX;
        tmp = PMSR_MAC_SPD(val);
        if (tmp == 0)
                res |= MTKSWITCH_SPEED_10;
        else if (tmp == 1)
                res |= MTKSWITCH_SPEED_100;
        else if (tmp == PMSR_MAC_SPD_1000 || tmp == PMSR_MAC_SPD_2500)
                /*
                 * mtkswitchvar.h has no 2.5G encoding and the ifmedia
                 * conversion leaves the speed unset for anything it does
                 * not know, so report the 2.5G trunks as 1G here.
                 */
                res |= MTKSWITCH_SPEED_1000;
        if (val & PMSR_TX_FC_STS)
                res |= MTKSWITCH_TXFLOW;
        if (val & PMSR_RX_FC_STS)
                res |= MTKSWITCH_RXFLOW;

        return (res);
}

static int
mtkswitch_atu_flush(struct mtkswitch_softc *sc)
{
        int val;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);

        /* Flush all non-static MAC addresses */
        mtkswitch_busy_wait(sc, MT7531_ATC, MT7531_ATC_BUSY);
        sc->hal.mtkswitch_write(sc, MT7531_ATC, MT7531_ATC_BUSY |
            MT7531_ATC_AC_MAT(0) | MT7531_ATC_AC_CMD_CLEAN);
        val = mtkswitch_busy_wait(sc, MT7531_ATC, MT7531_ATC_BUSY);

        /* check invalid value from ATC */
        if(val & MT7531_ATC_ADDR_INVLD) {
                device_printf(sc->sc_dev, "mtkswitch_atu_flush: MT7531_ATC_ADDR_INVLD\n");
                return (0);
        }

        return (0);
}

static int
mtkswitch_port_vlan_setup(struct mtkswitch_softc *sc, etherswitch_port_t *p)
{
        int err;

        /*
         * Port behaviour wrt tag/untag/stack is currently defined per-VLAN.
         * So we say we don't support it here.
         */
        if ((p->es_flags & (ETHERSWITCH_PORT_DOUBLE_TAG |
                            ETHERSWITCH_PORT_ADDTAG | ETHERSWITCH_PORT_STRIPTAG)) != 0)
                return (ENOTSUP);

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);

        /* Set the PVID */
        if (p->es_pvid != 0) {
                err = sc->hal.mtkswitch_vlan_set_pvid(sc, p->es_port,
                    p->es_pvid);
                if (err != 0) {
                        MTKSWITCH_UNLOCK(sc);
                        return (err);
                }
        }

        MTKSWITCH_UNLOCK(sc);

        return (0);
}

static int
mtkswitch_port_vlan_get(struct mtkswitch_softc *sc, etherswitch_port_t *p)
{

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);

        /* Retrieve the PVID */
        sc->hal.mtkswitch_vlan_get_pvid(sc, p->es_port, &p->es_pvid);

        /*
         * Port flags are not supported at the moment.
         * Port's tag/untag/stack behaviour is defined per-VLAN.
         */
        p->es_flags = 0;

        MTKSWITCH_UNLOCK(sc);

        return (0);
}

static void
mtkswitch_invalidate_vlan(struct mtkswitch_softc *sc, uint32_t vid)
{

        mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);
        sc->hal.mtkswitch_write(sc, MT7531_VTCR, VTCR_BUSY |
                                                    VTCR_FUNC_VID_INVALID | (vid & VTCR_VID_MASK));
        mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);
}


static int
mtkswitch_update_vlan_entry(struct mtkswitch_softc *sc, uint16_t vid,
                            uint8_t members, uint16_t untag)
{
        uint32_t val;

        mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);

        /* We use FID 0 */
        val = VAWD1_IVL_MAC | VAWD1_VTAG_EN | VAWD1_VALID |
              ((members & VAWD1_MEMBER_MASK) << VAWD1_MEMBER_OFF);
        sc->hal.mtkswitch_write(sc, MT7531_VAWD1, val);

        /* Set tagged ports */
        sc->hal.mtkswitch_write(sc, MT7531_VAWD2, (untag &0xFFFF));

        /* Write the VLAN entry */
        sc->hal.mtkswitch_write(sc, MT7531_VTCR, VTCR_BUSY |
                                                    VTCR_FUNC_VID_WRITE | (vid & VTCR_VID_MASK));
        val = mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);
        return (val);
}

static void
mtkswitch_vlan_init_hw(struct mtkswitch_softc *sc)
{
        uint8_t members = 0;
        uint32_t i;

        uint8_t wan_members = 0;

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);
        /* Reset all VLANs to defaults first */
        for (i = 0; i < sc->info.es_nvlangroups; i++) {
                mtkswitch_invalidate_vlan(sc, i);
        }

       /*
	* Membership mirrors the segments computed from the device-tree
	* labels: everything in VLAN 1, except that a wan segment, when the
	* board defines one, lives in VLAN 2.  The ports run in secure mode,
	* so this and the port matrix are enforced together.
	*/
        for (i = 0; i < sc->info.es_nports; i++) {
	       if (sc->segmented && sc->port_pvid[i] == 2)
		       wan_members |= ((1u)<<(i));
	       else
		       members |= ((1u)<<(i));
       }

        mtkswitch_update_vlan_entry(sc, 1, members, 0);
       if (wan_members != 0)
	       mtkswitch_update_vlan_entry(sc, 2, wan_members, 0);

        /* Reset internal VLAN table. */
        for (i = 0; i < nitems(sc->vlans); i++)
                sc->vlans[i] = 0;

        sc->vlans[0] = 1;
       if (wan_members != 0)
	       sc->vlans[1] = 2;

       /* PVIDs follow the same split. */
        for (i = 0; i < sc->info.es_nports; i++) {
	       sc->hal.mtkswitch_vlan_set_pvid(sc, i,
		   sc->segmented ? sc->port_pvid[i] : 1);
        }

        MTKSWITCH_UNLOCK(sc);
}

static int
mtkswitch_vlan_getvgroup(struct mtkswitch_softc *sc, etherswitch_vlangroup_t *v)
{
        uint32_t val, i;
        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);

        /* Reset the member ports. */
        v->es_untagged_ports = 0;
        v->es_member_ports = 0;

        /* Not supported for now */
        v->es_fid = 0;

        MTKSWITCH_LOCK(sc);
        v->es_vid = sc->vlans[v->es_vlangroup];

        if (v->es_vid == 0)
        {
                MTKSWITCH_UNLOCK(sc);
                return (0);
        }
        mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);
        sc->hal.mtkswitch_write(sc, MT7531_VTCR, VTCR_BUSY |
                                                    VTCR_FUNC_VID_READ | (v->es_vid & VTCR_VID_MASK));
        val = mtkswitch_busy_wait(sc, MT7531_VTCR, VTCR_BUSY);
        if (val & VTCR_IDX_INVALID) {
                MTKSWITCH_UNLOCK(sc);
                return (0);
        }

        val = sc->hal.mtkswitch_read(sc, MT7531_VAWD1);
        if (val & VAWD1_VALID)
                v->es_vid |= ETHERSWITCH_VID_VALID;
        else {
                MTKSWITCH_UNLOCK(sc);
                return (0);
        }
        v->es_member_ports = (val >> VAWD1_MEMBER_OFF) & VAWD1_MEMBER_MASK;

        val = sc->hal.mtkswitch_read(sc, MT7531_VAWD2);
        for (i = 0; i < sc->info.es_nports; i++) {
                if ((val & VAWD2_PORT_MASK(i)) == VAWD2_PORT_UNTAGGED(i))
                        v->es_untagged_ports |= (1<<i);
        }

        MTKSWITCH_UNLOCK(sc);
        return (0);
}

static int
mtkswitch_vlan_setvgroup(struct mtkswitch_softc *sc, etherswitch_vlangroup_t *v)
{
        uint16_t untagged_ports = 0;
        uint32_t val;
        int i, vlan;

        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);

        vlan = v->es_vid & ETHERSWITCH_VID_MASK;
        if (vlan == 0)
        {
                mtkswitch_invalidate_vlan(sc, sc->vlans[v->es_vlangroup]);
                sc->vlans[v->es_vlangroup] = 0;
                return (0);
        }

        /* Is this VLAN already in table ? */
        for (i = 0; i < sc->info.es_nvlangroups; i++)
                if (i != v->es_vlangroup && vlan == sc->vlans[i])
                        return (EINVAL);

        sc->vlans[v->es_vlangroup] = vlan;

        /* We currently don't support FID */
        if (v->es_fid != 0)
                return (EINVAL);

        MTKSWITCH_LOCK(sc);
        /* Set tagged ports and Write the VLAN entry*/
        for (i = 0; i < sc->info.es_nports; i++)
                if (((1<<i) & v->es_untagged_ports) == 0)
                        untagged_ports |= VAWD2_PORT_TAGGED(i);

        val = mtkswitch_update_vlan_entry(sc, v->es_vid, v->es_member_ports,
            untagged_ports);
        MTKSWITCH_UNLOCK(sc);

        if (val & VTCR_IDX_INVALID)
                return (EINVAL);

        return (0);
}

static int
mtkswitch_vlan_get_pvid(struct mtkswitch_softc *sc, int port, int *pvid)
{

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);

        *pvid = sc->hal.mtkswitch_read(sc, MTKSWITCH_PPBV1(port));
        *pvid = PPBV_VID_FROM_REG(*pvid);

        return (0);
}

static int
mtkswitch_vlan_set_pvid(struct mtkswitch_softc *sc, int port, int pvid)
{
        uint32_t val;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);
        val = PPBV_VID(pvid & PPBV_VID_MASK);
        sc->hal.mtkswitch_write(sc, MTKSWITCH_PPBV1(port), val);

        return (0);
}

extern void
mtk_attach_switch_mt7531(struct mtkswitch_softc *sc)
{
        sc->portmap = 0x7f;
        sc->phymap = 0x1f;
        sc->info.es_nports = 7;
        sc->info.es_vlan_caps = ETHERSWITCH_VLAN_DOT1Q;
        sprintf(sc->info.es_name, "Mediatek GSW");
        sc->hal.mtkswitch_read = mtkswitch_reg_read32;
        sc->hal.mtkswitch_write = mtkswitch_reg_write32;
        sc->info.es_nvlangroups = 4096;
        sc->hal.mtkswitch_reset = mtkswitch_reset;
        sc->hal.mtkswitch_hw_setup = mtkswitch_hw_setup;
        sc->hal.mtkswitch_hw_global_setup = mtkswitch_hw_global_setup;
        sc->hal.mtkswitch_port_init = mtkswitch_port_init;
        sc->hal.mtkswitch_get_port_status = mtkswitch_get_port_status;
        sc->hal.mtkswitch_atu_flush = mtkswitch_atu_flush;
        sc->hal.mtkswitch_port_vlan_setup = mtkswitch_port_vlan_setup;
        sc->hal.mtkswitch_port_vlan_get = mtkswitch_port_vlan_get;
        sc->hal.mtkswitch_vlan_init_hw = mtkswitch_vlan_init_hw;
        sc->hal.mtkswitch_vlan_getvgroup = mtkswitch_vlan_getvgroup;
        sc->hal.mtkswitch_vlan_setvgroup = mtkswitch_vlan_setvgroup;
        sc->hal.mtkswitch_vlan_get_pvid = mtkswitch_vlan_get_pvid;
        sc->hal.mtkswitch_vlan_set_pvid = mtkswitch_vlan_set_pvid;
        sc->hal.mtkswitch_phy_read = mtkswitch_phy_read;
        sc->hal.mtkswitch_phy_write = mtkswitch_phy_write;
        sc->hal.mtkswitch_reg_read = mtkswitch_reg_read;
        sc->hal.mtkswitch_reg_write = mtkswitch_reg_write;
}

#define MT7530_PORT_MIB_COUNTER(x)	(0x4000 + (x) * 0x100)

struct mt7530_mib_desc {
    unsigned int size;
    unsigned int offset;
    const char *name;
    const char *desc;
};

#define MIB_DESC(_s, _o, _n, _d)\
{				\
       .size = (_s),		\
       .offset = (_o),		\
       .name = (_n),		\
       .desc = (_d),		\
}

// vaata üle
static const
struct mt7530_mib_desc mt7530_mib[] = {
    MIB_DESC(1, 0x00, "tx_drop",		"Transmit droped frames"),
    MIB_DESC(1, 0x04, "tx_crcerrs",		"Transmit CRC errors"),
    MIB_DESC(1, 0x08, "tx_ucast_frames",	"Transmit good unicast frames"),
    MIB_DESC(1, 0x0c, "tx_mcast_frames",	"Transmit good multicast frames"),
    MIB_DESC(1, 0x10, "tx_bcast_frames",	"Transmit good broadcast frames"),
    MIB_DESC(1, 0x14, "tx_colls",		"Transmit collisions"),
    MIB_DESC(1, 0x18, "tx_single_colls",	"Transmit single collisions"),
    MIB_DESC(1, 0x1c, "tx_multi_colls",	"Transmit multiple collisions"),
    MIB_DESC(1, 0x20, "tx_deferred",	"Transmit deferred frames"),
    MIB_DESC(1, 0x24, "tx_late_colls",	"Transmit late collisions"),
    MIB_DESC(1, 0x28, "tx_excess_colls",	"Transmit excessive collisions"),
    MIB_DESC(1, 0x2c, "tx_pause_frames",	"Transmit pause frames"),
    MIB_DESC(1, 0x30, "tx_frames_64",	"Transmit 64 bytes frames"),
    MIB_DESC(1, 0x34, "tx_frames_65_127",	"Transmit 65 to 127 bytes frames"),
    MIB_DESC(1, 0x38, "tx_frames_128_255",	"Transmit 128 to 255 bytes frames"),
    MIB_DESC(1, 0x3c, "tx_frames_256_511",	"Transmit 256 to 511 bytes frames"),
    MIB_DESC(1, 0x40, "tx_frames_512_1023",	"Transmit 512 to 1023 bytes frames"),
    MIB_DESC(1, 0x44, "tx_frame_1024_max",	"Transmit 1024 to max bytes frames"),
    MIB_DESC(2, 0x48, "tx_bytes",		"Transmit good bytes"),
    MIB_DESC(1, 0x60, "rx_drop",		"Receive droped frames"),
    MIB_DESC(1, 0x64, "rx_pkts_filtered",	"Receive frames is filtered"),
    MIB_DESC(1, 0x68, "rx_ucast_frames",	"Receive unicast frames"),
    MIB_DESC(1, 0x6c, "rx_mcast_frames",	"Receive multicast frames"),
    MIB_DESC(1, 0x70, "rx_bcast_frames",	"Receive broadcast frames"),
    MIB_DESC(1, 0x74, "rx_align_errs",	"Receive alignment errors"),
    MIB_DESC(1, 0x78, "rx_crcerrs",		"Receive CRC errors"),
    MIB_DESC(1, 0x7c, "rx_runts",		"Receive undersized frames"),
    MIB_DESC(1, 0x80, "rx_fragments",	"Receive fragmented frames"),
    MIB_DESC(1, 0x84, "rx_oversize_frames",	"Receive oversize frames"),
    MIB_DESC(1, 0x88, "rx_jabbers",		"Receive jabbers frames"),
    MIB_DESC(1, 0x8c, "rx_pause_frames",	"Receive pause control frames"),
    MIB_DESC(1, 0x90, "rx_frames_64",	"Receive 64 bytes frames"),
    MIB_DESC(1, 0x94, "rx_frames_65_127",	"Receive 65 to 127 bytes frames"),
    MIB_DESC(1, 0x98, "rx_frames_128_255",	"Receive 128 to 255 bytes frames"),
    MIB_DESC(1, 0x9c, "rx_frames_256_511",	"Receive 256 to 511 bytes frames"),
    MIB_DESC(1, 0xa0, "rx_frames_512_1023",	"Receive 512 to 1023 bytes frames"),
    MIB_DESC(1, 0xa4, "rx_frame_1024_max",	"Receive 1024 to max bytes frames"),
    MIB_DESC(2, 0xa8, "rx_bytes",		"Receive good bytes"),
    MIB_DESC(1, 0xb0, "rx_ctrl_drop",	"Receive droped frames"),
    MIB_DESC(1, 0xb4, "rx_ingress_drop",	"Receive droped by ingress rate limited"),
    MIB_DESC(1, 0xb8, "rx_arl_drop",	"Receive droped by ACL"),
};

static int64_t
mt7531_hw_port_mib_read_count(struct mtkswitch_softc *sc, int port, int index)
{
        const struct mt7530_mib_desc *mib;
        uint64_t val;
        uint32_t reg, hi;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);

        mib = &mt7530_mib[index];
        reg = MT7530_PORT_MIB_COUNTER(port) + mib->offset;

        val = sc->hal.mtkswitch_read(sc, reg);
        if (mib->size == 2) {
                hi = sc->hal.mtkswitch_read(sc, reg + 4);
                val |= ((uint64_t) hi << 32);
        }

        return val;
}

static int
mt7531_hw_port_mib_clear(struct mtkswitch_softc *sc, int port,
                         int index, uint32_t val)
{
        uint32_t reg;

        MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);

        reg = MT7530_PORT_MIB_COUNTER(port) + index;

        return sc->hal.mtkswitch_write(sc, reg, val);
}

struct mt7531_sysctl_mib {
    struct mtkswitch_softc  *sc;
    int                     index;
    int                     port;
};

static int
mt7531_sysctl_port_mib_read_count(SYSCTL_HANDLER_ARGS)
{
        struct mtkswitch_softc *sc;
        uint64_t val = 0;
        uint32_t reg = (uint32_t)arg2;
        uint32_t port  = ((reg >> 16) & 0xffff);
        uint32_t index = (reg & 0xffff);

        sc = (struct mtkswitch_softc *)arg1;
        if (sc == NULL)
                return (EINVAL);

        if (index >= nitems(mt7530_mib))
                return (EINVAL);

        if (port >= MTKSWITCH_MAX_PORTS)
                return (EINVAL);

        MTKSWITCH_LOCK(sc);
        val = mt7531_hw_port_mib_read_count(sc, port, index);
        MTKSWITCH_UNLOCK(sc);

        return sysctl_handle_64(oidp, &val, 0, req);
}

static int
mt7531_sysctl_port_mib_clear_count(SYSCTL_HANDLER_ARGS)
{
        struct mtkswitch_softc *sc;
        uint32_t val = 0;
        uint32_t reg = (uint32_t)arg2;
        uint32_t port  = ((reg >> 16) & 0xffff);
        uint32_t index = (reg & 0xffff);
        int error;

        sc = (struct mtkswitch_softc *)arg1;
        if (sc == NULL)
                return (EINVAL);

        if (port >= MTKSWITCH_MAX_PORTS)
                return (EINVAL);

        error = sysctl_handle_32(oidp, &val, 0, req);

        if (error || !req->newptr)
                return (error);

        //MTKSWITCH_LOCK_ASSERT(sc, MA_OWNED);
        MTKSWITCH_LOCK(sc);
        mt7531_hw_port_mib_clear(sc, port, index, val);
        MTKSWITCH_UNLOCK(sc);

        return (0);
}

/*
 * mt7531_sysctl_dump - one-shot dump of the switch forwarding state.
 *
 * Reading dev.mtkswitch.<unit>.dump answers what the etherswitch API cannot:
 * mtkswitch_getport() reports every fixed-link port - the CPU trunk included
 * - as ACTIVE straight from the device tree without ever reading the
 * hardware, so PMSR (and, on a serdes port, the PCS link bit) is the only
 * real link state.  MFC/CFC say whether flooded frames may reach the CPU port
 * at all, and PCR's port matrix says which ports each port may forward to.
 *
 * The registers are reached over MDIO, so they are snapshotted under the
 * softc lock and formatted afterwards: sbuf(9) drains into user memory and
 * may sleep.
 */
static int
mt7531_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
        struct mtkswitch_softc *sc = arg1;
        struct sbuf *sb;
        uint32_t mfc, cfc;
        uint32_t pcr[MTKSWITCH_MAX_PORTS], pvc[MTKSWITCH_MAX_PORTS];
        uint32_t ppbv[MTKSWITCH_MAX_PORTS], pmcr[MTKSWITCH_MAX_PORTS];
        uint32_t pmsr[MTKSWITCH_MAX_PORTS], pcs[MTKSWITCH_MAX_PORTS];
        bool serdes[MTKSWITCH_MAX_PORTS];
        int error, port, nports;

        nports = sc->info.es_nports;
        if (nports > MTKSWITCH_MAX_PORTS)
                nports = MTKSWITCH_MAX_PORTS;

        sb = sbuf_new_for_sysctl(NULL, NULL, 1024, req);
        if (sb == NULL)
                return (ENOMEM);

        MTKSWITCH_LOCK(sc);
        mfc = sc->hal.mtkswitch_read(sc, MT7531_MFC);
        cfc = sc->hal.mtkswitch_read(sc, MT7531_CFC);
        for (port = 0; port < nports; port++) {
                pcr[port] = sc->hal.mtkswitch_read(sc, MTKSWITCH_PCR(port));
                pvc[port] = sc->hal.mtkswitch_read(sc, MTKSWITCH_PVC(port));
                ppbv[port] = sc->hal.mtkswitch_read(sc, MTKSWITCH_PPBV1(port));
                pmcr[port] = sc->hal.mtkswitch_read(sc, MTKSWITCH_PMCR(port));
                pmsr[port] = sc->hal.mtkswitch_read(sc, MTKSWITCH_PMSR(port));

                /* Only ports 5 and 6 carry a PCS (see MT7531_SGMII_BASE). */
                serdes[port] = (port >= 5 && port <= 6 &&
                    MTK_PHY_MODE_IS_SERDES(sc->port_mode[port]));
                pcs[port] = serdes[port] ?
                    sc->hal.mtkswitch_read(sc,
                    MT7531_SGMII_BASE(port) + SGMII_PCS_CONTROL_1) : 0;
        }
        MTKSWITCH_UNLOCK(sc);

        sbuf_printf(sb, "\ncpuport %d  portmap 0x%02x  phymap 0x%02x\n",
            sc->cpuport, sc->portmap, sc->phymap);
        sbuf_printf(sb, "MFC 0x%08x  flood bc 0x%02x unm 0x%02x unu 0x%02x\n",
            mfc, MFC_BC_FFP_GET(mfc), MFC_UNM_FFP_GET(mfc),
            MFC_UNU_FFP_GET(mfc));
        sbuf_printf(sb, "CFC 0x%08x  cpu port bitmap 0x%02x\n",
            cfc, CFC_CPU_PMAP_GET(cfc));

        for (port = 0; port < nports; port++) {
                sbuf_printf(sb, "port%d%s matrix 0x%02x pvid %u "
                    "PCR 0x%08x PVC 0x%08x PMCR 0x%08x PMSR 0x%08x "
                    "link=%s %s %s",
                    port, port == sc->cpuport ? "*" : " ",
                    PCR_MATRIX_GET(pcr[port]), PPBV_VID_FROM_REG(ppbv[port]),
                    pcr[port], pvc[port], pmcr[port], pmsr[port],
                    (pmsr[port] & PMSR_MAC_LINK_STS) ? "UP" : "DOWN",
                    (pmsr[port] & PMSR_MAC_DPX_STS) ? "full" : "half",
                    PMSR_MAC_SPD(pmsr[port]) == PMSR_MAC_SPD_2500 ? "2500" :
                    PMSR_MAC_SPD(pmsr[port]) == PMSR_MAC_SPD_1000 ? "1000" :
                    PMSR_MAC_SPD(pmsr[port]) == PMSR_MAC_SPD_100 ? "100" :
                    "10");
                if (serdes[port])
                        sbuf_printf(sb, "  PCS 0x%08x pcs_link=%s",
                            pcs[port],
                            (pcs[port] & PCS_LINK_STATUS) ? "UP" : "DOWN");
                sbuf_printf(sb, "\n");
        }

        error = sbuf_finish(sb);
        sbuf_delete(sb);

        return (error);
}

int
mt7531_sysctl_attach(struct mtkswitch_softc *sc)
{
        struct sysctl_ctx_list *ctx;
        struct sysctl_oid *tree;
        struct sysctl_oid *ptree;
        struct sysctl_oid_list *children;
        struct sysctl_oid_list *pchildren;
        struct sysctl_oid_list *ichildren;

        char port_num_buf[32];
        uint32_t reg;
        int index, port;

        ctx = device_get_sysctl_ctx(sc->sc_dev);
        children = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->sc_dev));

        SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "dump",
            CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
            mt7531_sysctl_dump, "A",
            "dump per-port link, VLAN and forwarding state");

        tree = SYSCTL_ADD_NODE(ctx, children, OID_AUTO, "port",
            CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
            "ethernet stndard mib counters of ports");
        pchildren = SYSCTL_CHILDREN(tree);

        for (port = 0; port < MTKSWITCH_MAX_PORTS; port++) {
                snprintf(port_num_buf, sizeof(port_num_buf), "%d", port);
                ptree = SYSCTL_ADD_NODE(ctx, pchildren, port,
                    port_num_buf, CTLFLAG_RD | CTLFLAG_MPSAFE,
                    NULL, "port mib counters");
                ichildren = SYSCTL_CHILDREN(ptree);
                for (index = 0; index <  nitems(mt7530_mib); index++) {
                        reg = ((port << 16) | index);

                        SYSCTL_ADD_PROC(ctx, ichildren, index,
                            mt7530_mib[index].name,
                            CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE,
                            sc, reg, mt7531_sysctl_port_mib_read_count,
                            "LU", mt7530_mib[index].desc);
                }
                reg = ((port << 16) | 0xD0);
                SYSCTL_ADD_PROC(ctx, ichildren, OID_AUTO,
                    "clear_tx", CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE,
                    sc, reg, mt7531_sysctl_port_mib_clear_count,
                    "IU", "Clear TX Counters");
                reg = ((port << 16) | 0xD4);
                SYSCTL_ADD_PROC(ctx, ichildren, OID_AUTO,
                    "clear_rx", CTLTYPE_UINT | CTLFLAG_WR | CTLFLAG_MPSAFE,
                    sc, reg, mt7531_sysctl_port_mib_clear_count,
                    "IU", "Clear RX Counters");
        }
        return (0);
}

static int
mt7531_arl_fetch_entry(struct mtkswitch_softc *sc, etherswitch_atu_entry_t *e)
{

        uint32_t tsra1, tsra2, tsrd;

        tsra1 = sc->hal.mtkswitch_read(sc, MT7531_TSRA1);
        tsra2 = sc->hal.mtkswitch_read(sc, MT7531_TSRA2);
        tsrd  = sc->hal.mtkswitch_read(sc, MT7531_TSRD);

        /* MAC address */
        e->es_macaddr[5] = ((tsra2 >> 16) & 0xFF);
        e->es_macaddr[4] = ((tsra2 >> 24) & 0xFF);
        e->es_macaddr[3] = ((tsra1 >>  0) & 0xFF);
        e->es_macaddr[2] = ((tsra1 >>  8) & 0xFF);
        e->es_macaddr[1] = ((tsra1 >> 16) & 0xFF);
        e->es_macaddr[0] = ((tsra1 >> 24) & 0xFF);

        /* Bitmask of ports this entry is for */
        e->es_portmask = ((tsrd >> 4) & 0xFF);

        return 0;
}

static void mt7531_is_atc_busy(struct mtkswitch_softc *sc)
{

        mtkswitch_busy_wait(sc, MT7531_ATC, MT7531_ATC_BUSY);
}

int
mt7531_atu_fetch_table(device_t dev, etherswitch_atu_table_t *table)
{
        struct mtkswitch_softc *sc;
        int cnt;
        uint32_t val;

        sc = device_get_softc(dev);

        /*
         * The etherswitch ioctl layer calls this without holding our lock,
         * so acquire it here.  (The previous code asserted MA_OWNED and then
         * re-locked, which paniced under INVARIANTS / on the non-recursive
         * mutex.)
         */
        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);
        MTKSWITCH_LOCK(sc);

        memset(&sc->atu.entries, 0, sizeof(sc->atu.entries));
        sc->atu.count = 0;
        cnt = 0;

        sc->hal.mtkswitch_write(sc, MT7531_ATC,
            (MT7531_ATC_BUSY | MT7531_AC_CMD_SSC));
        mt7531_is_atc_busy(sc);
        val = sc->hal.mtkswitch_read(sc, MT7531_ATC);
        while (!(val & MT7531_ATC_SRCH_END) &&
               cnt < MTKSWITCH_NUM_ARL_ENTRIES) {
                mt7531_arl_fetch_entry(sc, &sc->atu.entries[cnt]);
                sc->atu.entries[cnt].id = cnt;
                cnt++;
                sc->hal.mtkswitch_write(sc, MT7531_ATC,
                    (MT7531_ATC_BUSY | MT7531_AC_CMD_NSC));
                mt7531_is_atc_busy(sc);
                val = sc->hal.mtkswitch_read(sc, MT7531_ATC);
        }
        sc->atu.count = cnt;
        table->es_nitems = cnt;
        MTKSWITCH_UNLOCK(sc);

        return (0);
}

int
mt7531_atu_fetch_table_entry(device_t dev, etherswitch_atu_entry_t *e)
{
        struct mtkswitch_softc *sc;
        int id, err = 0;

        sc = device_get_softc(dev);
        MTKSWITCH_LOCK_ASSERT(sc, MA_NOTOWNED);

        id = e->id;

        MTKSWITCH_LOCK(sc);
        if (id < 0 || id >= sc->atu.count) {
                err = ENOENT;
                goto done;
        }
        memcpy(e, &sc->atu.entries[id], sizeof(*e));
done:
        MTKSWITCH_UNLOCK(sc);
        return (err);
}
