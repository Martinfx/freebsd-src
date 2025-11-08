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
#include <sys/proc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>
#include <dev/regulator/regulator.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include <dev/ofw/ofw_bus.h>

#include "pcib_if.h"

struct mt7622_pcie_softc {
    struct ofw_pci_softc	ofw_pci;
    device_t dev;
    struct resource *res_mem;
    int rid;
    struct resource	*pcie_irq_res;
    void *pcie_irq_cookie;
    phandle_t node;
    clk_t ays_ck0, ahb_ck0, aux_ck0, axi_ck0, obff_ck0, pipe_ck0;
    struct ofw_pci_range	mem_range;
    struct ofw_pci_range	pref_mem_range;
    struct ofw_pci_range	io_range;
};

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-pcie",	1},
        {NULL,				0}
};

static int
mt7622_pcie_sys_irq(void *arg)
{
    return (FILTER_HANDLED);
}

static int
mt7622_pcie_decode_ranges(struct mt7622_pcie_softc *sc, struct ofw_pci_range *ranges,
                      int nranges)
{
    int i;

    for (i = 0; i < nranges; i++) {
        switch(ranges[i].pci_hi & OFW_PCI_PHYS_HI_SPACEMASK) {
            case OFW_PCI_PHYS_HI_SPACE_IO:
                if (sc->io_range.size != 0) {
                    device_printf(sc->dev,
                                  "Duplicated IO range found in DT\n");
                    return (ENXIO);
                }
                sc->io_range = ranges[i];
                break;
            case OFW_PCI_PHYS_HI_SPACE_MEM32:
            case OFW_PCI_PHYS_HI_SPACE_MEM64:
                if (ranges[i].pci_hi & OFW_PCI_PHYS_HI_PREFETCHABLE) {
                    if (sc->pref_mem_range.size != 0) {
                        device_printf(sc->dev,
                                      "Duplicated memory range found "
                                      "in DT\n");
                        return (ENXIO);
                    }
                    sc->pref_mem_range = ranges[i];
                } else {
                    if (sc->mem_range.size != 0) {
                        device_printf(sc->dev,
                                      "Duplicated memory range found "
                                      "in DT\n");
                        return (ENXIO);
                    }
                    sc->mem_range = ranges[i];
                }
        }
    }
    if (sc->mem_range.size == 0) {
        device_printf(sc->dev,
                      " At least memory range should be defined in DT.\n");
        return (ENXIO);
    }
    return (0);
}

static int
mt7622_pcie_detach(device_t dev) {
    struct mt7622_pcie_softc *sc = device_get_softc(dev);

    if (sc->ays_ck0) {
        clk_release(sc->ays_ck0);
    }
    if (sc->ahb_ck0) {
        clk_release(sc->ahb_ck0);
    }
    if (sc->aux_ck0) {
        clk_release(sc->aux_ck0);
    }
    if (sc->axi_ck0) {
        clk_release(sc->axi_ck0);
    }
    if (sc->obff_ck0) {
        clk_release(sc->obff_ck0);
    }
    if (sc->pcie_irq_res) {
        bus_release_resource(dev, SYS_RES_IRQ, sc->rid,
                             sc->pcie_irq_res);
    }
    if (sc->res_mem) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->rid,
                             sc->res_mem);
    }

    return (0);
}

static int
mt7622_pcie_attach(device_t dev) {
    struct mt7622_pcie_softc *sc = device_get_softc(dev);
    int error = 0;

    sc->dev = dev;
    sc->node = ofw_bus_get_node(dev);

    if ((error = ofw_bus_find_string_index(sc->node, "reg-names", "port0",
                                           &sc->rid))) {
        device_printf(dev, "Cannot get port0 memory: %d\n", error);
        return (ENXIO);
    }

    sc->res_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
                                         RF_ACTIVE);
    if (sc->res_mem == NULL) {
        device_printf(dev, "Cannot allocate resource\n");
        return (ENXIO);
    }

    error = ofw_bus_find_string_index(sc->node, "interrupt-names",
                                   "pcie_irq", &sc->rid);
    if (error != 0) {
        device_printf(dev, "Cannot get 'pcie_irq' IRQ\n");
        return (ENXIO);
    }
    sc->pcie_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, ,
                                             RF_ACTIVE | RF_SHAREABLE);
    if (sc->pcie_irq_res == NULL) {
        device_printf(dev, "Cannot allocate 'pcie' IRQ resource\n");
        return (ENXIO);
    }

    error = bus_setup_intr(dev, sc->pcie_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
                           mt7622_pcie_sys_irq, NULL, sc, &sc->pcie_irq_cookie);
    if (error != 0) {
        device_printf(dev, "cannot setup client interrupt handler\n");
        return (ENXIO);
    }

    if (clk_get_by_ofw_name(dev, 0, "ays_ck0", &sc->ays_ck0)) {
        device_printf(dev, "Can not get ays_ck0 clk\n");
        return (ENXIO);
    }
    if (clk_get_by_ofw_name(dev, 0, "ahb_ck0", &sc->ahb_ck0)) {
        device_printf(dev, "Can not get ahb_ck0 clk\n");
        return (ENXIO);
    }
    if (clk_get_by_ofw_name(dev, 0, "aux_ck0", &sc->aux_ck0)) {
        device_printf(dev, "Can not get aux_ck0 clk\n");
        return (ENXIO);
    }
    if (clk_get_by_ofw_name(dev, 0, "axi_ck0", &sc->axi_ck0)) {
        device_printf(dev, "Can not get axi_ck0 clk\n");
        return (ENXIO);
    }
    if (clk_get_by_ofw_name(dev, 0, "obff_ck0", &sc->obff_ck0)) {
        device_printf(dev, "Can not get obff_ck0 clk\n");
        return (ENXIO);
    }
    if (clk_get_by_ofw_name(dev, 0, "pipe_ck0", &sc->pipe_ck0)) {
        device_printf(dev, "Can not get pipe_ck0 clk\n");
        return (ENXIO);
    }

    error = ofw_pcib_init(dev);
    if (error != 0) {
        return (ENXIO);
    }

    error = mt7622_pcie_decode_ranges(sc, sc->ofw_pci.sc_range,
                               sc->ofw_pci.sc_nrange);
    if (error != 0) {
        bus_teardown_intr(dev, sc->pcie_irq_res, sc->pcie_irq_cookie);
    }

    bus_attach_children(dev);
    return (0);
}

static int
mt7622_pcie_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);
    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);
    device_set_desc(dev, "Mediatek 7622 PCI controller");
    return (BUS_PROBE_DEFAULT);
}

static device_method_t mt7622_pcie_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		mt7622_pcie_probe),
        DEVMETHOD(device_attach,	mt7622_pcie_attach),
        DEVMETHOD(device_detach,	mt7622_pcie_detach),

        /* PCI DW interface */
        //DEVMETHOD(pci_dw_get_link,	rk3568_pcie_get_link),

        DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, mt7622_pcie_driver, mt7622_pcie_methods,
sizeof(struct mt7622_pcie_softc), ofw_pcib_driver);
DRIVER_MODULE(mt7622_pcie, simplebus, mt7622_pcie_driver, NULL, NULL);
