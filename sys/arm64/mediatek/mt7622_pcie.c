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
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>
#include <dev/pci/pci_dw.h>
#include <dev/syscon/syscon.h>

#include "pcib_if.h"
#include "syscon_if.h"

#define PCIE_SYS_CFG_V2          0x000
#define PCIE_CSR_LTSSM_EN(p)     (1U << (0 + (p) * 8))
#define PCIE_CSR_ASPM_L1_EN(p)   (1U << (1 + (p) * 8))

#define PCIE_RST_CTRL            0x510
#define PCIE_PHY_RSTB            (1U << 0)
#define PCIE_PIPE_SRSTB          (1U << 1)
#define PCIE_MAC_SRSTB           (1U << 2)
#define PCIE_CRSTB               (1U << 3)
#define PCIE_PERSTB              (1U << 8)
#define PCIE_LINKDOWN_RST_EN     (7U << 13)

#define PCIE_LINK_STATUS_V2       0x804
#define PCIE_PORT_LINKUP_V2       (1U << 10)
#define MTK_LINK_TIMEOUT_US       2000000
#define MTK_LINK_POLL_US          1000

#define PCIE_CONF_VEND_ID	    0x100
#define PCIE_CONF_DEVICE_ID	    0x102
#define PCIE_CONF_CLASS_ID	    0x106

#define PCI_VENDOR_ID_MEDIATEK		0x14c3
#define PCI_CLASS_BRIDGE_PCI		0x0604

struct mt7622_pcie_softc {
    struct ofw_pci_softc ofw_pci;
    device_t dev;
    struct resource *res_mem;
    int rid;
    struct resource *pcie_irq_res;
    int irq_rid;
    void *pcie_irq_cookie;
    phandle_t node;
    clk_t sys_ck0, ahb_ck0, aux_ck0, axi_ck0, obff_ck0, pipe_ck0;
    struct ofw_pci_range pref_mem_range;
    struct ofw_pci_range io_range;
    struct ofw_pci_range *mem_ranges;
    int	num_mem_ranges;
    int port;
    bus_dma_tag_t dmat;
    struct syscon		*syscon;
};

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-pcie", 1},
        {NULL,                   0}
};

static int
mt7622_pcie_sys_irq(void *arg) {
    return (FILTER_HANDLED);
}

static bus_dma_tag_t
mt7622_pci_get_dma_tag(device_t dev, device_t child)
{
    struct mt7622_pcie_softc *sc;
    sc = device_get_softc(dev);
    return (sc->dmat);
}

static int
mt7622_pcie_get_port(phandle_t node)
{
    int idx;
    if (ofw_bus_find_string_index(node, "reg-names", "port0", &idx) == 0) {
        return 0;
    }

    if (ofw_bus_find_string_index(node, "reg-names", "port1", &idx) == 0) {
        return 1;
    }

    return -1;
}

static int
mt7622_pcie_get_link(device_t dev, bool *status)
{
    struct mt7622_pcie_softc *sc = device_get_softc(dev);
    uint32_t val;

    val = bus_read_4(sc->res_mem, PCIE_LINK_STATUS_V2);
    if (val & PCIE_PORT_LINKUP_V2) {
        *status = true;
    }
    else {
        *status = false;
    }

    return (0);
}

static int
mt7622_pcie_port_start(device_t dev, int port)
{
    struct mt7622_pcie_softc *sc = device_get_softc(dev);
    int count; //, error;
    bool status;

    uint32_t val;
    val = bus_read_4(sc->res_mem, PCIE_SYS_CFG_V2);
    val |= PCIE_CSR_LTSSM_EN(port) | PCIE_CSR_ASPM_L1_EN(port);
    bus_write_4(sc->res_mem, PCIE_SYS_CFG_V2, val);

    /* Assert all reset signals */
    bus_write_4(sc->res_mem, PCIE_RST_CTRL, 0x0);

    /* Enable PCIe link down reset */
    bus_write_4(sc->res_mem, PCIE_RST_CTRL, PCIE_LINKDOWN_RST_EN);

    /*Described in PCIe CEM specification sections 2.2 (PERST# Signal) and
    * 2.2.1 (Initial Power-Up (G3 to S0))*/
    DELAY(1000);

    /* De-assert PHY, PE, PIPE, MAC and configuration reset	*/
    val = bus_read_4(sc->res_mem, PCIE_RST_CTRL);
    val |= PCIE_PHY_RSTB | PCIE_PERSTB | PCIE_PIPE_SRSTB |
           PCIE_MAC_SRSTB | PCIE_CRSTB;
    bus_write_4(sc->res_mem, PCIE_RST_CTRL, val);

    /* Set up vendor ID and class code */
    //bus_write_4(sc->res_mem, PCIE_CONF_VEND_ID, PCI_VENDOR_ID_MEDIATEK);
    //bus_write_4(sc->res_mem, PCIE_CONF_CLASS_ID, PCI_CLASS_BRIDGE_PCI);

    /* Wait for link up/stable */
    for (count = 20; count; count--) {
        mt7622_pcie_get_link(dev, &status);
        if (status) {
            break;
        }

        DELAY(100000);
        if (count == 0) {
            device_printf(sc->dev, "PCIe port%d: link-up timeout (status=0x%08x)\n",
                          port, val);
            return (ENXIO);
        }
    }

    /* Delay to have things settle */
    DELAY(100000);

    return (0);
}

static int
mt7622_pcie_decode_ranges(struct mt7622_pcie_softc *sc, struct ofw_pci_range *ranges, int nranges)
{
    int i, nmem, rv;

    nmem = 0;
    for (i = 0; i < nranges; i++) {
        if ((ranges[i].pci_hi & OFW_PCI_PHYS_HI_SPACEMASK) ==
            OFW_PCI_PHYS_HI_SPACE_MEM32)
            ++nmem;
    }

    sc->mem_ranges = malloc(nmem * sizeof(*sc->mem_ranges), M_DEVBUF,
                            M_WAITOK);
    sc->num_mem_ranges = nmem;

    nmem = 0;
    for (i = 0; i < nranges; i++) {
        if ((ranges[i].pci_hi & OFW_PCI_PHYS_HI_SPACEMASK)  ==
            OFW_PCI_PHYS_HI_SPACE_IO) {
            if (sc->io_range.size != 0) {
                device_printf(sc->dev,
                              "Duplicated IO range found in DT\n");
                rv = ENXIO;
                goto out;
            }

            sc->io_range = ranges[i];
            if (sc->io_range.size > UINT32_MAX) {
                device_printf(sc->dev,
                              "ATU IO window size is too large. "
                              "Up to 4GB windows are supported, "
                              "trimming window size to 4GB\n");
                sc->io_range.size = UINT32_MAX;
            }
        }
        if ((ranges[i].pci_hi & OFW_PCI_PHYS_HI_SPACEMASK) ==
            OFW_PCI_PHYS_HI_SPACE_MEM32) {
            MPASS(nmem < sc->num_mem_ranges);
            sc->mem_ranges[nmem] = ranges[i];
            if (sc->mem_ranges[nmem].size > UINT32_MAX) {
                device_printf(sc->dev,
                              "ATU MEM window size is too large. "
                              "Up to 4GB windows are supported, "
                              "trimming window size to 4GB\n");
                sc->mem_ranges[nmem].size = UINT32_MAX;
            }
            ++nmem;
        }
    }

    MPASS(nmem == sc->num_mem_ranges);

    if (nmem == 0) {
        device_printf(sc->dev,
                      "Missing required memory range in DT\n");
        return (ENXIO);
    }

    return (0);

    out:
        free(sc->mem_ranges, M_DEVBUF);
         return (rv);
}


static void
mt7622_pcie_startup_port(device_t dev, int port)
{
    struct mt7622_pcie_softc *sc = device_get_softc(dev);
    uint32_t val;

    val = SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2);
    val |= PCIE_CSR_LTSSM_EN(port) | PCIE_CSR_ASPM_L1_EN(port);
    SYSCON_WRITE_4(sc->syscon, PCIE_SYS_CFG_V2, val);

    device_printf(sc->dev,
                  "enabled LTSSM+ASPM L1 for port %u (0x%08x)\n",
                  port, val);
}

static int
mt7622_pcie_detach(device_t dev)
{
    struct mt7622_pcie_softc *sc = device_get_softc(dev);

    if (sc->sys_ck0) {
        clk_disable(sc->sys_ck0);
        clk_release(sc->sys_ck0);
    }

    if (sc->ahb_ck0) {
        clk_disable(sc->ahb_ck0);
        clk_release(sc->ahb_ck0);
    }

    if (sc->aux_ck0) {
        clk_disable(sc->aux_ck0);
        clk_release(sc->aux_ck0);
    }

    if (sc->axi_ck0) {
        clk_disable(sc->axi_ck0);
        clk_release(sc->axi_ck0);
    }

    if (sc->obff_ck0) {
        clk_disable(sc->obff_ck0);
        clk_release(sc->obff_ck0);
    }

    if (sc->pcie_irq_cookie != NULL) {
        bus_teardown_intr(dev, sc->pcie_irq_res, sc->pcie_irq_cookie);
        sc->pcie_irq_cookie = NULL;
    }

    if (sc->res_mem) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->rid,
                             sc->res_mem);
    }

    if (sc->pcie_irq_res) {
        bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
                             sc->pcie_irq_res);
    }

    ofw_pcib_fini(sc->dev);

    return (0);
}

static int
mt7622_pcie_attach(device_t dev) {
    struct mt7622_pcie_softc *sc = device_get_softc(dev);
    int error = 0;
    phandle_t nodecfg, root;

    sc->dev = dev;
    sc->node = ofw_bus_get_node(dev);

    root = OF_finddevice("/");
    if (root == -1) {
        device_printf(sc->dev, "No FDT root\n");
        return (ENXIO);
    }

    nodecfg = ofw_bus_find_compatible(root, "mediatek,generic-pciecfg");

    if (nodecfg == 0) {
        device_printf(sc->dev,
                      "Cannot mediatek,generic-pciecfg syscon node found\n");
        return (ENXIO);
    }

    error = syscon_get_by_ofw_node(sc->dev, nodecfg, &sc->syscon);
    if (error != 0) {
        device_printf(sc->dev,
                      "Cannot get syscon handle for pciecfg: %d\n", error);
        return (error);
    }

    mt7622_pcie_startup_port(sc->dev, 0);
    mt7622_pcie_startup_port(sc->dev, 1);

    sc->port = mt7622_pcie_get_port(sc->node);
    if (sc->port == -1) {
        device_printf(dev, "Cannot determine port (reg-names missing)\n");
        return (ENXIO);
    }

    error = ofw_bus_find_string_index(sc->node, "reg-names",
                                      sc->port == 0 ? "port0" : "port1",
                                      &sc->rid);
    if (error != 0) {
        device_printf(dev, "Cannot get port memory: %d\n", error);
        return (ENXIO);
    }

    sc->res_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
                                         RF_ACTIVE);
    if (sc->res_mem == NULL) {
        device_printf(dev, "Cannot allocate resource\n");
        return (ENXIO);
    }

    error = ofw_bus_find_string_index(sc->node, "interrupt-names",
                                      "pcie_irq", &sc->irq_rid);
    if (error != 0) {
        device_printf(dev, "Cannot get 'pcie_irq' IRQ\n");
        return (ENXIO);
    }
    sc->pcie_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
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

    if (sc->port == 0) {
        if (clk_get_by_ofw_name(dev, 0, "sys_ck0", &sc->sys_ck0)) {
            device_printf(dev, "Can not get sys_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->sys_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable sys_ck0 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "ahb_ck0", &sc->ahb_ck0)) {
            device_printf(dev, "Can not get ahb_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->ahb_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable ahb_ck0 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "aux_ck0", &sc->aux_ck0)) {
            device_printf(dev, "Can not get aux_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->aux_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable aux_ck0 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "axi_ck0", &sc->axi_ck0)) {
            device_printf(dev, "Can not get axi_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->axi_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable axi_ck0 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "obff_ck0", &sc->obff_ck0)) {
            device_printf(dev, "Can not get obff_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->obff_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable obff_ck0 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "pipe_ck0", &sc->pipe_ck0)) {
            device_printf(dev, "Can not get pipe_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->pipe_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable pipe_ck0 clock\n");
            return (ENXIO);
        }
    } else if (sc->port == 1) {
        if (clk_get_by_ofw_name(dev, 0, "sys_ck1", &sc->sys_ck0)) {
            device_printf(dev, "Can not get sys_ck1 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->sys_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable sys_ck1 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "ahb_ck1", &sc->ahb_ck0)) {
            device_printf(dev, "Can not get ahb_ck1 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->ahb_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable ahb_ck1 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "aux_ck1", &sc->aux_ck0)) {
            device_printf(dev, "Can not get aux_ck1 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->aux_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable aux_ck1 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "axi_ck1", &sc->axi_ck0)) {
            device_printf(dev, "Can not get axi_ck1 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->axi_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable axi_ck1 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "obff_ck1", &sc->obff_ck0)) {
            device_printf(dev, "Can not get obff_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->obff_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable obff_ck1 clock\n");
            return (ENXIO);
        }

        if (clk_get_by_ofw_name(dev, 0, "pipe_ck1", &sc->pipe_ck0)) {
            device_printf(dev, "Can not get pipe_ck0 clk\n");
            return (ENXIO);
        }

        error = clk_enable(sc->pipe_ck0);
        if (error != 0) {
            device_printf(sc->dev, "could not enable pipe_ck1 clock\n");
            return (ENXIO);
        }
    } else {
        device_printf(dev, "CLocks not found.\n");
        return (ENXIO);
    }

    error = mt7622_pcie_port_start(dev, sc->port);
    if (error != 0) {
        device_printf(dev, "port%d: link bring-up failed: %d\n", sc->port, error);
        return (ENXIO);
    }

    error = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
                            1, 0,				/* alignment, bounds */
                            BUS_SPACE_MAXADDR,			/* lowaddr */
                            BUS_SPACE_MAXADDR,			/* highaddr */
                            NULL, NULL,				/* filter, filterarg */
                            BUS_SPACE_MAXSIZE,			/* maxsize */
                            BUS_SPACE_UNRESTRICTED,		/* nsegments */
                            BUS_SPACE_MAXSIZE,			/* maxsegsize */
                            0, /* flags */
                            NULL, NULL,				/* lockfunc, lockarg */
                            &sc->dmat);
    if (error != 0) {
        return (ENXIO);
    }

    error = ofw_pcib_init(dev);
    if (error != 0) {
        return (ENXIO);
    }

    error = mt7622_pcie_decode_ranges(sc, sc->ofw_pci.sc_range,
                                      sc->ofw_pci.sc_nrange);
    if (error != 0) {
        return (ENXIO);
    }

    /*device_add_child(dev, "pci", DEVICE_UNIT_ANY);

    error = ofw_pcib_attach(dev);
    if (error != 0) {
        return (ENXIO);
    }*/

    device_add_child(dev, "pci", DEVICE_UNIT_ANY);
    bus_attach_children(dev);

    return (0);
}

static int
mt7622_pcie_probe(device_t dev) {
    if (!ofw_bus_status_okay(dev)) {
        return (ENXIO);
    }
    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data) {
        return (ENXIO);
    }
    device_set_desc(dev, "Mediatek 7622 PCI controller");
    return (BUS_PROBE_DEFAULT);
}

static device_method_t mt7622_pcie_methods[] = {
        /* Bus interface */
        DEVMETHOD(bus_get_dma_tag,	mt7622_pci_get_dma_tag),

        /* Device interface */
        DEVMETHOD(device_probe, mt7622_pcie_probe),
        DEVMETHOD(device_attach, mt7622_pcie_attach),
        DEVMETHOD(device_detach, mt7622_pcie_detach),
        DEVMETHOD(device_shutdown, bus_generic_shutdown),
        DEVMETHOD(device_suspend, bus_generic_suspend),
        DEVMETHOD(device_resume, bus_generic_resume),

        /* PCI DW interface */
        //DEVMETHOD(pci_dw_get_link,	mt7622_pcie_get_link),

        /* OFW bus interface */
        DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
        DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
        DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
        DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
        DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

        DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, mt7622_pcie_driver, mt7622_pcie_methods,
sizeof(struct mt7622_pcie_softc), ofw_pcib_driver);
DRIVER_MODULE(mt7622_pcie, simplebus, mt7622_pcie_driver, NULL, NULL);
