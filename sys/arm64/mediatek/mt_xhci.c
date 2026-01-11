/*-
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

#include "opt_bus.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/condvar.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>
#include <dev/regulator/regulator.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/xhci.h>
#include <dev/usb/controller/xhcireg.h>
#include "usbdevs.h"

#define MTXHCI_MAX_PORTS	4

/* registers */
#define MTXHCI_RESET		0x00
#define  RESET_ASSERT		(1 << 0)
#define MTXHCI_CFG_HOST		0x04
#define MTXHCI_CFG_DEV		0x08
#define MTXHCI_CFG_PCIE		0x0c
#define  CFG_PWRDN		(1 << 0)
#define MTXHCI_STA		0x10
#define  STA_USB3		(1 << 16)
#define  STA_XHCI		(1 << 11)
#define  STA_SYS		(1 << 10)
#define  STA_REF		(1 << 8)
#define  STA_PLL		(1 << 0)
#define MTXHCI_CAPS		0x24
#define  CAP_USB2_PORTS(x)	(((x) >> 8) & 0x7)
#define  CAP_USB3_PORTS(x)	(((x) >> 0) & 0x7)
#define MTXHCI_USB3_PORT(x)	0x30 + (x) * 8
#define MTXHCI_USB2_PORT(x)	0x50 + (x) * 8
#define  CFG_PORT_HOST		(1 << 2)
#define  CFG_PORT_PWRDN		(1 << 1)
#define  CFG_PORT_DISABLE	(1 << 0)

#define	LOCK(_sc)		mtx_lock(&(_sc)->mtx)
#define	UNLOCK(_sc)		mtx_unlock(&(_sc)->mtx)
#define	SLEEP(_sc, timeout)						\
    mtx_sleep(sc, &sc->mtx, 0, "mt_xhci", timeout);
#define	LOCK_INIT(_sc)							\
    mtx_init(&_sc->mtx, device_get_nameunit(_sc->dev), "mt_xhci", MTX_DEF)
#define	LOCK_DESTROY(_sc)	mtx_destroy(&_sc->mtx)
#define	ASSERT_LOCKED(_sc)	mtx_assert(&_sc->mtx, MA_OWNED)
#define	ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->mtx, MA_NOTOWNED)

struct xhci_soc;
struct mt_xhci_softc {
    struct xhci_softc xhci_softc;
    device_t  dev;
    struct xhci_soc	*soc;
    struct mtx	mtx;
    struct resource *mem_mac;
    struct resource *mem_ippc;
    void *intr_cookie;
    int ports_usb2;
    int ports_usb3;
    clk_t clk_xusb_sys_ck;
    clk_t clk_xusb_ref_ck;
    clk_t clk_xusb_mcu_ck;
    clk_t clk_xusb_dma_ck;
    phy_t phys[8];
    regulator_t	regulators[16];
    struct intr_config_hook	irq_hook;
    bool xhci_inited;
};

struct xhci_soc {
    char 		**regulator_names;
    char 		**phy_names;
};

static char *mt_xhci_reg_names[] = {
        "vusb33-supply",
        "vbus-supply",
        NULL
};

static char *mt_xhci_phy_names[] = {
        "u2port0",
        "u3port0",
        "u2port1",
        NULL
};

static struct xhci_soc mt_soc = {
        .regulator_names = mt_xhci_reg_names,
        .phy_names = mt_xhci_phy_names,
};

static struct ofw_compat_data compat_data[] = {
        { "mediatek,mt7622-xhci",	(uintptr_t)&mt_soc},
        { "mediatek,mtk-xhci", (uintptr_t)&mt_soc},
        { NULL, 0 }
};

static int
mt_xhci_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "MediaTek xhci driver");
    return (BUS_PROBE_DEFAULT);
}

static int
mt_xhci_attach(device_t dev)
{
    struct mt_xhci_softc *sc;
    struct xhci_softc *xsc;
    int ntries, i, rv, rid;
    uint32_t mask, val;
    phandle_t node;

    sc = device_get_softc(dev);
    sc->dev = dev;
    sc->soc = (struct xhci_soc *)ofw_bus_search_compatible(dev,
                                                           compat_data)->ocd_data;
    node = ofw_bus_get_node(dev);
    xsc = &sc->xhci_softc;

    LOCK_INIT(sc);

    rv = clk_get_by_ofw_name(sc->dev, 0, "sys_ck",
                             &sc->clk_xusb_sys_ck);
    if (rv != 0) {
        device_printf(sc->dev, "Cannot get 'sys_ck' clock\n");
        return (ENXIO);
    }

    rv = clk_get_by_ofw_name(sc->dev, 0, "ref_ck",
                             &sc->clk_xusb_ref_ck);
    if (rv != 0) {
        device_printf(sc->dev, "Cannot get 'ref_ck' clock\n");
        return (ENXIO);
    }

    rv = clk_get_by_ofw_name(sc->dev, 0, "mcu_ck",
                             &sc->clk_xusb_mcu_ck);
    if (rv != 0) {
        device_printf(sc->dev, "Cannot get 'mcu_ck' clock\n");
        return (ENXIO);
    }

    rv = clk_get_by_ofw_name(sc->dev, 0, "dma_ck",
                             &sc->clk_xusb_dma_ck);
    if (rv != 0) {
        device_printf(sc->dev, "Cannot get 'dma_ck' clock\n");
        return (ENXIO);
    }

    /* Enable clocks */
    rv = clk_enable(sc->clk_xusb_sys_ck);
    if (rv != 0) {
        device_printf(sc->dev,
                        "Cannot enable 'clk_xusb_sys_ck' clock\n");
        return (rv);
    }

    rv = clk_enable(sc->clk_xusb_ref_ck);
    if (rv != 0) {
        device_printf(sc->dev,
                        "Cannot enable 'clk_xusb_ref_ck' clock\n");
        return (rv);
    }

    rv = clk_enable(sc->clk_xusb_mcu_ck);
    if (rv != 0) {
        device_printf(sc->dev,
                        "Cannot enable 'clk_mcu_ck' clock\n");
        return (rv);
    }

    rv = clk_enable(sc->clk_xusb_dma_ck);
    if (rv != 0) {
        device_printf(sc->dev,
                        "Cannot enable 'clk_dma_ck' clock\n");
        return (rv);
     }

    /* Regulators. */
    for (i = 0; sc->soc->regulator_names[i] != NULL; i++) {
        if (i >= nitems(sc->regulators)) {
            device_printf(sc->dev,
                          "Too many regulators present in DT.\n");
            return (EOVERFLOW);
        }
        rv = regulator_get_by_name(sc->dev,
                                   sc->soc->regulator_names[i],
                                   sc->regulators + i);
        if (rv != 0) {
            device_printf(sc->dev,
                          "Cannot get '%s' regulator\n",
                          sc->soc->regulator_names[i]);
            continue;
        }
    }

    for (i = 0; i < nitems(sc->regulators); i++) {
        if (sc->regulators[i] == NULL)
            continue;
        rv = regulator_enable(sc->regulators[i]);
        if (rv != 0) {
            device_printf(sc->dev,
                          "Cannot enable '%s' regulator\n",
                          sc->soc->regulator_names[i]);
            return (rv);
        }
    }

    /* Phys. */
    for (i = 0; sc->soc->phy_names[i] != NULL; i++) {
        if (i >= nitems(sc->phys)) {
            device_printf(sc->dev,
                          "Too many phys present in DT.\n");
            return (EOVERFLOW);
        }

        device_printf(sc->dev, "Requesting PHY index %d: %s\n",
                      i, sc->soc->phy_names[i]);

        rv = phy_get_by_ofw_idx(sc->dev, node, i, sc->phys + i);
        if (rv != 0) {

            device_printf(sc->dev, "Cannot get '%s' phy, error: %d.\n",
                          sc->soc->phy_names[i], rv);
            return (rv);
        }

        rv = phy_set_mode(sc->phys[i], PHY_MODE_USB_HOST, 0);
        if (rv != 0) {
            device_printf(sc->dev, "Could not set phy %d to host mode: %d\n",
                          i, rv);
            phy_release(sc->phys[i]);
            return (rv);
        }

        rv = phy_set_mode(sc->phys[i], PHY_MODE_USB_DEVICE, 0);
        if (rv != 0) {
            device_printf(sc->dev, "Could not set phy %d to host mode: %d\n",
                          i, rv);
            phy_release(sc->phys[i]);
            return (rv);
        }

        rv = phy_set_mode(sc->phys[i], PHY_MODE_USB_OTG, 0);
        if (rv != 0) {
            device_printf(sc->dev, "Could not set phy %d to host mode: %d\n",
                          i, rv);
            phy_release(sc->phys[i]);
            return (rv);
        }

        rv = phy_set_mode(sc->phys[i], PHY_MODE_INVALID, 0);
        if (rv != 0) {
            device_printf(sc->dev, "Could not set phy %d to host mode: %d\n",
                          i, rv);
            phy_release(sc->phys[i]);
            return (rv);
        }

        rv = phy_enable(sc->phys[i]);
        if (rv != 0) {
            device_printf(sc->dev, "Could not enable phy %d: %d\n", i, rv);
            phy_release(sc->phys[i]);
            return (rv);
        }
        else {
            device_printf(sc->dev, "Enable phy %d - %s\n", i, sc->soc->phy_names[i]);
        }
    }


    /* Allocate resources. */
    rid = 0;
    sc->mem_mac  = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
    if (sc->mem_mac  == NULL) {
        device_printf(dev, "Cannot allocate 'mac' resource (rid 0)\n");
        return (ENXIO);
    }

    rid = 1;
    sc->mem_ippc = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
    if (sc->mem_ippc == NULL) {
        device_printf(dev, "Cannot allocate 'mac' resource (rid 1)\n");
        return (ENXIO);
    }

    rid = 0;
    xsc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
    if (xsc->sc_irq_res == NULL) {
        device_printf(dev, "Could not allocate IRQ resource (rid 0)\n");
        return (ENXIO);
    }

    /* inicialize HW */

    /* port capabilities */
    val = bus_read_4(sc->mem_ippc, MTXHCI_CAPS);
    sc->ports_usb3 = MIN(MTXHCI_MAX_PORTS, CAP_USB3_PORTS(val));
    sc->ports_usb2 = MIN(MTXHCI_MAX_PORTS, CAP_USB2_PORTS(val));
    if (sc->ports_usb3 == 0 && sc->ports_usb2 == 0) {
        return (ENXIO);
    }

    /* reset */
    val = bus_read_4(sc->mem_ippc, MTXHCI_RESET);
    val |= RESET_ASSERT;
    bus_write_4(sc->mem_ippc, MTXHCI_RESET, val);
    DELAY(10);
    val &= ~RESET_ASSERT;
    bus_write_4(sc->mem_ippc, MTXHCI_RESET, val);
    DELAY(10);

    /* disable device mode */
    val = bus_read_4(sc->mem_ippc, MTXHCI_CFG_DEV);
    val |= CFG_PWRDN;
    bus_write_4(sc->mem_ippc, MTXHCI_CFG_DEV, val);

    /* enable host mode */
    val = bus_read_4(sc->mem_ippc, MTXHCI_CFG_HOST);
    val &= ~CFG_PWRDN;
    bus_write_4(sc->mem_ippc, MTXHCI_CFG_HOST, val);

    mask = (STA_XHCI | STA_PLL | STA_SYS | STA_REF);
    if (sc->ports_usb3) {
        mask |= STA_USB3;

        /* disable PCIe mode */
        val = bus_read_4(sc->mem_ippc, MTXHCI_CFG_PCIE);
        val |= CFG_PWRDN;
        bus_write_4(sc->mem_ippc, MTXHCI_CFG_PCIE, val);
    }

    /* configure host ports */
    for (i = 0; i < sc->ports_usb3; i++) {
        val = bus_read_4(sc->mem_ippc, MTXHCI_USB3_PORT(i));
        val &= ~(CFG_PORT_DISABLE | CFG_PORT_PWRDN);
        val |= CFG_PORT_HOST;
        bus_write_4(sc->mem_ippc, MTXHCI_USB3_PORT(i), val);
    }
    for (i = 0; i < sc->ports_usb2; i++) {
        val = bus_read_4(sc->mem_ippc, MTXHCI_USB2_PORT(i));
        val &= ~(CFG_PORT_DISABLE | CFG_PORT_PWRDN);
        val |= CFG_PORT_HOST;
        bus_write_4(sc->mem_ippc, MTXHCI_USB2_PORT(i), val);
    }

    for (ntries = 0; ntries < 10000; ntries++) {
        val = bus_read_4(sc->mem_ippc, MTXHCI_STA);
        if ((val & mask) == mask)
            break;
        DELAY(50);
    }

    if (ntries == 10000) {
        device_printf(dev, "ETIMEDOUT\n");
        return ETIMEDOUT;
    }

    /* Fill data for XHCI driver. */
    /* xHCI core uses MAC registers */
    xsc->sc_io_res = sc->mem_mac;
    xsc->sc_bus.parent = dev;
    xsc->sc_bus.devices = xsc->sc_devices;
    xsc->sc_bus.devices_max = XHCI_MAX_DEVICES;

    xsc->sc_io_tag = rman_get_bustag(xsc->sc_io_res);
    xsc->sc_io_hdl = rman_get_bushandle(xsc->sc_io_res);
    xsc->sc_io_size = rman_get_size(xsc->sc_io_res);
    strlcpy(xsc->sc_vendor, "Mediatek", sizeof(xsc->sc_vendor));

    /* Add USB bus device. */
    xsc->sc_bus.bdev = device_add_child(sc->dev, "usbus", DEVICE_UNIT_ANY);
    if (xsc->sc_bus.bdev == NULL) {
        device_printf(sc->dev, "Could not add USB device\n");
        return (ENXIO);
    }
    device_set_ivars(xsc->sc_bus.bdev, &xsc->sc_bus);
    device_set_desc(xsc->sc_bus.bdev, "Mediatek USB 3.0 controller");

    rv = xhci_init(xsc, sc->dev, 1);
    if (rv != 0) {
        device_printf(sc->dev, "USB init failed: %d\n", rv);
        return (ENXIO);
    }

    rv = bus_setup_intr(dev, xsc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
                        NULL, (driver_intr_t *)xhci_interrupt, xsc, &xsc->sc_intr_hdl);
    if (rv != 0) {
        device_printf(dev, "Could not setup error IRQ: %d\n",rv);
        xsc->sc_intr_hdl = NULL;
        return (ENXIO);
    }

    sc->xhci_inited = true;
    rv = xhci_start_controller(xsc);
    if (rv != 0) {
        device_printf(sc->dev,
                      "Could not start XHCI controller: %d\n", rv);
        return (ENXIO);
    }

    rv = device_probe_and_attach(xsc->sc_bus.bdev);
    if (rv != 0) {
        device_printf(sc->dev, "Could not initialize USB: %d\n", rv);
        return (ENXIO);
    }
    return 0;
}

static int
mt_xhci_detach(device_t dev)
{
    struct mt_xhci_softc *sc;
    struct xhci_softc *xsc;
    int error;

    sc = device_get_softc(dev);
    xsc = &sc->xhci_softc;

    error = bus_generic_detach(dev);
    if (error != 0) {
        return (error);
    }

    if (sc->xhci_inited) {
        usb_callout_drain(&xsc->sc_callout);
        xhci_halt_controller(xsc);
    }

    if (xsc->sc_irq_res && xsc->sc_intr_hdl) {
        bus_teardown_intr(dev, xsc->sc_irq_res, xsc->sc_intr_hdl);
        xsc->sc_intr_hdl = NULL;
    }

    if (xsc->sc_io_res != NULL) {
        bus_release_resource(dev, SYS_RES_MEMORY,
                             rman_get_rid(xsc->sc_io_res), xsc->sc_io_res);
        xsc->sc_io_res = NULL;
    }

    if (sc->xhci_inited) {
        xhci_uninit(xsc);
    }

    LOCK_DESTROY(sc);

    return 0;
}

static device_method_t xhci_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe, mt_xhci_probe),
        DEVMETHOD(device_attach, mt_xhci_attach),
        DEVMETHOD(device_detach, mt_xhci_detach),
        DEVMETHOD(device_suspend, bus_generic_suspend),
        DEVMETHOD(device_resume, bus_generic_resume),
        DEVMETHOD(device_shutdown, bus_generic_shutdown),

        /* Bus interface */
        DEVMETHOD(bus_print_child, bus_generic_print_child),
        DEVMETHOD_END
};

static DEFINE_CLASS_0(xhci, xhci_driver, xhci_methods,  sizeof(struct mt_xhci_softc));
DRIVER_MODULE(mt_xhci, simplebus, xhci_driver, 0, 0);
MODULE_DEPEND(mt_xhci, usb, 1, 1, 1);
