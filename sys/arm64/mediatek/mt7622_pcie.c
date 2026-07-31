/*
* Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
*
* SPDX-License-Identifier: BSD-2-Clause
*/

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/clk/clk.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcib_private.h>

#include "pcib_if.h"
#include "pic_if.h"
#include "msi_if.h"

#define	PCIE_SYS_CFG_V2			0x00
#define	 PCIE_CSR_LTSSM_EN(x)		(1U << (0 + (x) * 8))
#define	 PCIE_CSR_ASPM_L1_EN(x)		(1U << (1 + (x) * 8))

/* Per-port registers. */
#define	PCIE_MSI_VECTOR			0x0c0

#define	PCIE_CONF_VEND_ID		0x100
#define	PCIE_CONF_DEVICE_ID		0x102
#define	PCIE_CONF_CLASS_ID		0x106

#define	PCIE_INT_MASK			0x420
#define	 PCIE_INTX_SHIFT		16
#define	 PCIE_INTX_MASK			(0xfU << PCIE_INTX_SHIFT)
#define	 PCIE_MSI_MASK			(1U << 23)
#define	PCIE_INT_STATUS			0x424
#define	PCIE_IMSI_STATUS		0x42c
#define	PCIE_IMSI_ADDR			0x430

#define	PCIE_AHB_TRANS_BASE0_L		0x438
#define	PCIE_AHB_TRANS_BASE0_H		0x43c
#define	 AHB2PCIE_SIZE(x)		((x) & 0x1fU)
#define	PCIE_AXI_WINDOW0		0x448
#define	 WIN_ENABLE			(1U << 7)
/*
 * A PCIe to AXI window of 2^32 lets an endpoint reach the whole 32-bit
 * address space, which covers all of DRAM (it starts at 0x40000000 on
 * MT7622).  The value is also mirrored in BAR0 of the root port's
 * configuration space, so keep it at the width the hardware actually
 * needs rather than oversizing it.
 */
#define	 PCIE2AHB_SIZE			32

/* Configuration transaction header registers. */
#define	PCIE_CFG_HEADER0		0x460
#define	PCIE_CFG_HEADER1		0x464
#define	PCIE_CFG_HEADER2		0x468
#define	PCIE_CFG_WDATA			0x470
#define	PCIE_APP_TLP_REQ		0x488
#define	PCIE_CFG_RDATA			0x48c
#define	 APP_CFG_REQ			(1U << 0)
#define	 APP_CPL_STATUS			(0x7U << 5)

#define	CFG_WRRD_TYPE			0x4
#define	CFG_RD_FMT			0x0
#define	CFG_WR_FMT			0x2

#define	CFG_DW0_LENGTH(length)		((length) & 0x3ffU)
#define	CFG_DW0_TYPE(type)		(((type) & 0x1fU) << 24)
#define	CFG_DW0_FMT(fmt)		(((fmt) & 0x7U) << 29)
#define	CFG_DW2_REGN(regn)		((regn) & 0xffcU)
#define	CFG_DW2_FUN(fun)		(((fun) & 0x7U) << 16)
#define	CFG_DW2_DEV(dev)		(((dev) & 0x1fU) << 19)
#define	CFG_DW2_BUS(bus)		(((bus) & 0xffU) << 24)
#define	CFG_HEADER_DW0(type, fmt)					\
	(CFG_DW0_LENGTH(1) | CFG_DW0_TYPE(type) | CFG_DW0_FMT(fmt))
#define	CFG_HEADER_DW1(where, size)					\
	((((1U << (size)) - 1) << ((where) & 0x3)) & 0xffffU)
#define	CFG_HEADER_DW2(regn, fun, dev, bus)				\
	(CFG_DW2_REGN(regn) | CFG_DW2_FUN(fun) |			\
	 CFG_DW2_DEV(dev) | CFG_DW2_BUS(bus))

#define	PCIE_RST_CTRL			0x510
#define	 PCIE_PHY_RSTB			(1U << 0)
#define	 PCIE_PIPE_SRSTB		(1U << 1)
#define	 PCIE_MAC_SRSTB			(1U << 2)
#define	 PCIE_CRSTB			(1U << 3)
#define	 PCIE_PERSTB			(1U << 8)
#define	 PCIE_LINKDOWN_RST_EN		(0x7U << 13)
#define	PCIE_LINK_STATUS_V2		0x804
#define	 PCIE_PORT_LINKUP_V2		(1U << 10)

#define	MTK_PCIE_VENDOR_MEDIATEK	0x14c3

#define	MTK_PCIE_NINTX			4

/* The port implements 32 message-signalled interrupt vectors. */
#define	MTK_PCIE_NMSI			32

/* 100ms should be enough for Gen1/2 link training. */
#define	MTK_PCIE_LINKUP_TIMEOUT_US	100000
#define	MTK_PCIE_CFG_TIMEOUT_US		100000

struct mt_pcie_irqsrc {
    struct intr_irqsrc	        isrc;
    u_int			irq;
    bool			allocated;
};

struct mt_pcie_softc {
    struct ofw_pci_softc		fdt_sc;	/* Must be first. */
    device_t			        dev;
    struct mtx			        mtx;
    u_int				slot;
    struct resource			*mem_res;
    struct resource			*irq_res;
    void				*irq_cookie;
    struct mt_pcie_irqsrc		isrcs[MTK_PCIE_NINTX];
    struct ofw_pci_range		*mem_range;
    int				        mem_ranges;
    /* MSI state. */
    bus_dma_tag_t			dmat;
    struct mt_pcie_irqsrc		msi_isrcs[MTK_PCIE_NMSI];
    uint64_t			        msi_addr;
    intptr_t			        msi_xref;
};

#define	MTK_PCIE_RD4(sc, reg)						\
	bus_read_4((sc)->mem_res, (reg))
#define	MTK_PCIE_WR4(sc, reg, val)					\
	bus_write_4((sc)->mem_res, (reg), (val))
#define	MTK_PCIE_WR2(sc, reg, val)					\
	bus_write_2((sc)->mem_res, (reg), (val))

static struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7622-pcie",	1 },
    { NULL,				0 },
};

static void
mt_pcie_intx_mask(struct mt_pcie_softc *sc, u_int irq, bool mask)
{
        uint32_t val;

        val = MTK_PCIE_RD4(sc, PCIE_INT_MASK);
        if (mask)
        {
                val |= 1U << (PCIE_INTX_SHIFT + irq);
        }
        else
        {
                val &= ~(1U << (PCIE_INTX_SHIFT + irq));
        }

        MTK_PCIE_WR4(sc, PCIE_INT_MASK, val);
}

static void
mt_pcie_intx_eoi(struct mt_pcie_softc *sc, u_int irq)
{
        MTK_PCIE_WR4(sc, PCIE_INT_STATUS, 1U << (PCIE_INTX_SHIFT + irq));
}

static int
mt_pcie_intr(void *arg)
{
        struct mt_pcie_softc *sc;
        struct trapframe *tf;
        uint32_t imsi_status, status;
        u_int irq;

        sc = arg;
        tf = curthread->td_intr_frame;

        status = MTK_PCIE_RD4(sc, PCIE_INT_STATUS) &
                 ~MTK_PCIE_RD4(sc, PCIE_INT_MASK);

        for (irq = 0; irq < MTK_PCIE_NINTX; irq++) {
                if ((status & (1U << (PCIE_INTX_SHIFT + irq))) == 0)
                        continue;
                if (intr_isrc_dispatch(&sc->isrcs[irq].isrc, tf) != 0) {
                        mt_pcie_intx_mask(sc, irq, true);
                        mt_pcie_intx_eoi(sc, irq);
                        device_printf(sc->dev, "stray INTx %u disabled\n",
                            irq);
                }
        }

        if ((status & PCIE_MSI_MASK) != 0) {
                while ((imsi_status =
                            MTK_PCIE_RD4(sc, PCIE_IMSI_STATUS)) != 0) {
                        for (irq = 0; irq < MTK_PCIE_NMSI; irq++) {
                                if ((imsi_status & (1U << irq)) == 0)
                                        continue;
                                /* Acknowledge the vector. */
                                MTK_PCIE_WR4(sc, PCIE_IMSI_STATUS, 1U << irq);
                                if (intr_isrc_dispatch(
                                    &sc->msi_isrcs[irq].isrc, tf) != 0)
                                        device_printf(sc->dev,
                                            "stray MSI %u\n", irq);
                        }
                }
                /* Clear the aggregated MSI status. */
                MTK_PCIE_WR4(sc, PCIE_INT_STATUS, PCIE_MSI_MASK);
        }

        return (FILTER_HANDLED);
}

/*
 * INTx interrupt controller.
 *
 * The four legacy interrupts are demultiplexed by the port out of the
 * single line it takes from the GIC, so the port has to present itself
 * as an interrupt controller in its own right.  The device tree already
 * expects that: interrupt-map sends INTA-D to a child node
 *
 *	pcie_intc0: interrupt-controller {
 *		interrupt-controller;
 *		#interrupt-cells = <1>;
 *	};
 *
 * and ofw_pcib_route_interrupt() maps a child's pin through it.  Note
 * what that mapping produces: intr_map_irq() returns an index into the
 * INTRNG mapping table with isrc left NULL, not a resolved interrupt.
 * Resolving it - which is what allocating the resource does - needs a
 * PIC registered against the child node's xref.  Without one, a device
 * behind this port can be routed and still never get an interrupt, and
 * the failure surfaces far away as an unexplained allocation failure.
 *
 * The lines are level-triggered: the status bit follows the device
 * rather than latching, so a handler that has not yet run must be
 * masked to keep it from re-firing.
 */

static struct mt_pcie_irqsrc *
mt_pcie_isrc_to_intx(struct intr_irqsrc *isrc)
{

        /* isrc is the first member, so this is the enclosing entry. */
        return ((struct mt_pcie_irqsrc *)isrc);
}

static int
mt_pcie_pic_map_intr(device_t dev, struct intr_map_data *data,
                     struct intr_irqsrc **isrcp)
{
        struct mt_pcie_softc *sc;
        struct intr_map_data_fdt *daf;

        if (data->type != INTR_MAP_DATA_FDT)
                return (ENOTSUP);

        daf = (struct intr_map_data_fdt *)data;
        if (daf->ncells != 1)
                return (EINVAL);
        if (daf->cells[0] >= MTK_PCIE_NINTX)
                return (EINVAL);

        sc = device_get_softc(dev);
        *isrcp = &sc->isrcs[daf->cells[0]].isrc;

        return (0);
}

static void
mt_pcie_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
        struct mt_pcie_softc *sc;

        sc = device_get_softc(dev);
        mt_pcie_intx_mask(sc, mt_pcie_isrc_to_intx(isrc)->irq, false);
}

static void
mt_pcie_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
        struct mt_pcie_softc *sc;

        sc = device_get_softc(dev);
        mt_pcie_intx_mask(sc, mt_pcie_isrc_to_intx(isrc)->irq, true);
}

static void
mt_pcie_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
        struct mt_pcie_softc *sc;

        sc = device_get_softc(dev);
        mt_pcie_intx_eoi(sc, mt_pcie_isrc_to_intx(isrc)->irq);
}

static void
mt_pcie_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
        struct mt_pcie_softc *sc;

        /*
         * Level-triggered, so the source stays asserted until the
         * thread has dealt with the device; mask it meanwhile.
         */
        sc = device_get_softc(dev);
        mt_pcie_intx_mask(sc, mt_pcie_isrc_to_intx(isrc)->irq, true);
}

static void
mt_pcie_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
        struct mt_pcie_softc *sc;
        u_int irq;

        sc = device_get_softc(dev);
        irq = mt_pcie_isrc_to_intx(isrc)->irq;
        mt_pcie_intx_eoi(sc, irq);
        mt_pcie_intx_mask(sc, irq, false);
}

/*
 * Claim the child interrupt-controller node and stand the four sources
 * up behind it.  Must run before the PCI bus is attached, because that
 * is when children are probed and start asking for their interrupts.
 */
static int
mt_pcie_setup_intx(struct mt_pcie_softc *sc)
{
        device_t dev;
        const char *name;
        phandle_t node, intc;
        u_int irq;
        int error;

        dev = sc->dev;
        node = ofw_bus_get_node(dev);

        for (intc = OF_child(node); intc != 0; intc = OF_peer(intc)) {
                if (OF_hasprop(intc, "interrupt-controller"))
                        break;
        }
        if (intc == 0) {
                device_printf(dev, "no interrupt-controller node; legacy "
                                   "interrupts will not work\n");
                return (0);
        }

        /* Masked and clear before anything can be routed to them. */
        for (irq = 0; irq < MTK_PCIE_NINTX; irq++) {
                mt_pcie_intx_mask(sc, irq, true);
                mt_pcie_intx_eoi(sc, irq);
        }

        name = device_get_nameunit(dev);
        for (irq = 0; irq < MTK_PCIE_NINTX; irq++) {
                sc->isrcs[irq].irq = irq;
                error = intr_isrc_register(&sc->isrcs[irq].isrc, dev, 0,
                    "%s,intx%u", name, irq);
                if (error != 0) {
                        device_printf(dev,
                            "could not register INTx source %u: %d\n", irq,
                            error);
                        return (error);
                }
        }

        if (intr_pic_register(dev, OF_xref_from_node(intc)) == NULL) {
                device_printf(dev, "could not register the INTx "
                                   "controller\n");
                return (ENXIO);
        }

        return (0);
}

static int
mt_pcie_msi_release_msix(device_t pci, device_t child, int irq)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);
        if (rv != 0)
                return (rv);
        return (intr_release_msix(pci, child, msi_parent, irq));
}

static int
mt_pcie_alloc_msi(device_t pci, device_t child, int count, int maxcount,
                  int *irqs)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);
        if (rv != 0)
                return (rv);

        rv = intr_alloc_msi(pci, child, msi_parent, count, maxcount,
            irqs);
        return (rv);
}

static int
mt_pcie_release_msi(device_t pci, device_t child, int count, int *irqs)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);
        if (rv != 0)
                return (rv);
        return (intr_release_msi(pci, child, msi_parent, count, irqs));
}

static int
mt_pcie_alloc_msix(device_t pci, device_t child, int *irq)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);
        if (rv != 0)
                return (rv);

        return (intr_alloc_msix(pci, child, msi_parent, irq));
}

static int
mt_pcie_release_msix(device_t pci, device_t child, int irq)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);
        if (rv != 0)
                return (rv);
        return (intr_release_msix(pci, child, msi_parent, irq));
}

static int
mt_pcie_map_msi(device_t pci, device_t child, int irq, uint64_t *addr,
                uint32_t *data)
{
        phandle_t msi_parent;
        int rv;

        rv = ofw_bus_msimap(ofw_bus_get_node(pci), pci_get_rid(child),
            &msi_parent, NULL);

        if (rv != 0)
                return (rv);

        return (intr_map_msi(pci, child, msi_parent, irq, addr, data));
}

static int
mt_pcie_maxslots(device_t dev)
{
        return (PCI_SLOTMAX);
}

static bool
mt_pcie_valid_addr(struct mt_pcie_softc *sc, u_int bus, u_int slot,
                   u_int func, u_int reg)
{

        if (bus < (u_int)sc->fdt_sc.sc_bus || bus > PCI_BUSMAX)
                return (false);
        if (slot > PCI_SLOTMAX || func > PCI_FUNCMAX || reg > PCIE_REGMAX)
                return (false);
        /*
         * On the root bus only the root port itself is visible.  Its
         * device number matches the port index (0 for pcie0, 1 for
         * pcie1).
         */
        if (bus == (u_int)sc->fdt_sc.sc_bus && (slot != sc->slot || func != 0))
                return (false);

        return (true);
}

static int
mt_pcie_check_cfg_cpld(struct mt_pcie_softc *sc)
{
        uint32_t val;
        int i;

        for (i = 0; i < MTK_PCIE_CFG_TIMEOUT_US / 10; i++) {
                val = MTK_PCIE_RD4(sc, PCIE_APP_TLP_REQ);
                if ((val & APP_CFG_REQ) == 0)
                        break;
                DELAY(10);
        }
        if ((val & APP_CFG_REQ) != 0)
                return (ETIMEDOUT);

        if ((MTK_PCIE_RD4(sc, PCIE_APP_TLP_REQ) & APP_CPL_STATUS) != 0)
                return (EIO);

        return (0);
}

static uint32_t
mt_pcie_read_config(device_t dev, u_int bus, u_int slot, u_int func,
                    u_int reg, int bytes)
{
        struct mt_pcie_softc *sc;
        uint32_t data;
        int error;

        sc = device_get_softc(dev);
        if (!mt_pcie_valid_addr(sc, bus, slot, func, reg))
                return (~0U);

        mtx_lock(&sc->mtx);

        /* Build the CfgRd TLP header and fire the request. */
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER0,
            CFG_HEADER_DW0(CFG_WRRD_TYPE, CFG_RD_FMT));
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER1, CFG_HEADER_DW1(reg, bytes));
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER2,
            CFG_HEADER_DW2(reg, func, slot, bus));

        MTK_PCIE_WR4(sc, PCIE_APP_TLP_REQ,
            MTK_PCIE_RD4(sc, PCIE_APP_TLP_REQ) | APP_CFG_REQ);

        error = mt_pcie_check_cfg_cpld(sc);
        if (error != 0) {
                mtx_unlock(&sc->mtx);
                return (~0U);
        }

        data = MTK_PCIE_RD4(sc, PCIE_CFG_RDATA);
        mtx_unlock(&sc->mtx);

        switch (bytes) {
                case 1:
                        data = (data >> (8 * (reg & 3))) & 0xff;
                        break;
                case 2:
                        data = (data >> (8 * (reg & 3))) & 0xffff;
                        break;
                case 4:
                        break;
                default:
                        data = ~0U;
                        break;
        }

        return (data);
}

static void
mt_pcie_write_config(device_t dev, u_int bus, u_int slot, u_int func,
                     u_int reg, uint32_t val, int bytes)
{
        struct mt_pcie_softc *sc;

        sc = device_get_softc(dev);
        if (!mt_pcie_valid_addr(sc, bus, slot, func, reg))
                return;
        if (bytes != 1 && bytes != 2 && bytes != 4)
                return;

        mtx_lock(&sc->mtx);

        /* Build the CfgWr TLP header and fire the request. */
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER0,
            CFG_HEADER_DW0(CFG_WRRD_TYPE, CFG_WR_FMT));
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER1, CFG_HEADER_DW1(reg, bytes));
        MTK_PCIE_WR4(sc, PCIE_CFG_HEADER2,
            CFG_HEADER_DW2(reg, func, slot, bus));

        MTK_PCIE_WR4(sc, PCIE_CFG_WDATA, val << (8 * (reg & 3)));

        MTK_PCIE_WR4(sc, PCIE_APP_TLP_REQ,
            MTK_PCIE_RD4(sc, PCIE_APP_TLP_REQ) | APP_CFG_REQ);

        (void)mt_pcie_check_cfg_cpld(sc);

        mtx_unlock(&sc->mtx);
}

static int
mt_pcie_get_id(device_t pci, device_t child, enum pci_id_type type,
               uintptr_t *id)
{

        if (type != PCI_ID_MSI)
                return (pcib_get_id(pci, child, type, id));

        /*
         * The MSI controller is the port itself and does not use a
         * sideband device ID, so the requester ID is all that is
         * needed.
         */
        *id = pci_get_rid(child);
        return (0);
}

static bus_dma_tag_t
mt_pcie_get_dma_tag(device_t dev, device_t child)
{
        struct mt_pcie_softc *sc;

        sc = device_get_softc(dev);
        return (sc->dmat);
}

static int
mt_pcie_setup_msi(struct mt_pcie_softc *sc)
{
        device_t dev;
        const char *name;
        phandle_t node;
        u_int irq;
        int error;

        dev = sc->dev;
        node = ofw_bus_get_node(dev);

        name = device_get_nameunit(dev);
        for (irq = 0; irq < MTK_PCIE_NMSI; irq++) {
                sc->msi_isrcs[irq].irq = irq;
                error = intr_isrc_register(&sc->msi_isrcs[irq].isrc, dev, 0,
                    "%s,msi%u", name, irq);
                if (error != 0)
                        return (error);
        }

        sc->msi_xref = OF_xref_from_node(node);
        OF_device_register_xref(sc->msi_xref, dev);

        error = intr_msi_register(dev, sc->msi_xref);
        if (error != 0) {
                device_printf(dev, "could not register MSI controller\n");
                return (error);
        }

        /*
         * Endpoints signal an MSI by writing to PCIE_MSI_VECTOR in the
         * port register window.  Only the low 32 bits of the address
         * are programmable, so the register block must live below 4GB.
         */
        sc->msi_addr = rman_get_start(sc->mem_res) + PCIE_MSI_VECTOR;
        MTK_PCIE_WR4(sc, PCIE_IMSI_ADDR, (uint32_t)sc->msi_addr);

        /* Clear pending vectors and unmask the MSI summary interrupt. */
        MTK_PCIE_WR4(sc, PCIE_IMSI_STATUS, ~0U);
        MTK_PCIE_WR4(sc, PCIE_INT_MASK,
            MTK_PCIE_RD4(sc, PCIE_INT_MASK) & ~PCIE_MSI_MASK);

        return (0);
}

/*
 * Enable the clocks listed in the FDT node.  MT7622 boards typically
 * come up with these clocks already running (the boot loader uses
 * PCIe too), and FreeBSD has no driver for the MT7622 pciesys clock
 * controller yet, so failures are not fatal.
 */
static void
mt_pcie_enable_clocks(device_t dev)
{
        clk_t clk;
        int error, i;

        for (i = 0; clk_get_by_ofw_index(dev, 0, i, &clk) == 0; i++) {
                error = clk_enable(clk);
                if (error != 0)
                        device_printf(dev,
                            "warning: could not enable clock %d: %d\n",
                            i, error);
        }
}

/*
 * MT7622 needs LTSSM and ASPM enabled through the shared
 * "mediatek,generic-pciecfg" register block.  The FDT node is not
 * referenced by a phandle, so look it up by compatible and map it
 * directly.
 */
static void
mt_pcie_ltssm_enable(struct mt_pcie_softc *sc)
{
        bus_space_tag_t tag;
        bus_space_handle_t handle;
        bus_size_t size;
        phandle_t node;
        uint32_t val;

        node = ofw_bus_find_compatible(OF_finddevice("/"),
            "mediatek,generic-pciecfg");
        if (node == 0) {
                device_printf(sc->dev,
                    "warning: no pciecfg node, not enabling LTSSM\n");
                return;
        }

        if (OF_decode_addr(node, 0, &tag, &handle, &size) != 0) {
                device_printf(sc->dev, "warning: could not map pciecfg\n");
                return;
        }

        val = bus_space_read_4(tag, handle, PCIE_SYS_CFG_V2);
        val |= PCIE_CSR_LTSSM_EN(sc->slot) | PCIE_CSR_ASPM_L1_EN(sc->slot);
        bus_space_write_4(tag, handle, PCIE_SYS_CFG_V2, val);

        bus_space_unmap(tag, handle, size);
}

static int
mt_pcie_startup_port(struct mt_pcie_softc *sc)
{
        uint64_t mem_base, mem_size;
        uint32_t val;
        int i, mems;

        /* Assert all reset signals. */
        MTK_PCIE_WR4(sc, PCIE_RST_CTRL, 0);

        /*
         * Enable PCIe link down reset: if the link goes from up to
         * down the MAC control registers and configuration space are
         * reset by hardware.
         */
        MTK_PCIE_WR4(sc, PCIE_RST_CTRL, PCIE_LINKDOWN_RST_EN);

        /* T_PVPERL: power stable to PERST# inactive. */
        DELAY(100 * 1000);

        /* De-assert PHY, PERST#, PIPE, MAC and configuration reset. */
        val = MTK_PCIE_RD4(sc, PCIE_RST_CTRL);
        val |= PCIE_PHY_RSTB | PCIE_PERSTB | PCIE_PIPE_SRSTB |
               PCIE_MAC_SRSTB | PCIE_CRSTB;
        MTK_PCIE_WR4(sc, PCIE_RST_CTRL, val);

        /*
         * The MT7622 host bridge comes up with a wrong ID/class; fix
         * them up so the root port is enumerated as a PCI-PCI bridge.
         */
        MTK_PCIE_WR2(sc, PCIE_CONF_VEND_ID, MTK_PCIE_VENDOR_MEDIATEK);
        MTK_PCIE_WR2(sc, PCIE_CONF_CLASS_ID,
            (PCIC_BRIDGE << 8) | PCIS_BRIDGE_PCI);

        /* Wait for link training to complete. */
        for (i = 0; i < MTK_PCIE_LINKUP_TIMEOUT_US / 20; i++) {
                val = MTK_PCIE_RD4(sc, PCIE_LINK_STATUS_V2);
                if ((val & PCIE_PORT_LINKUP_V2) != 0)
                        break;
                DELAY(20);
        }
        if ((val & PCIE_PORT_LINKUP_V2) == 0)
                return (ETIMEDOUT);

        /* Mask all INTx and MSI; they are unmasked on demand. */
        val = MTK_PCIE_RD4(sc, PCIE_INT_MASK);
        val |= PCIE_INTX_MASK | PCIE_MSI_MASK;
        MTK_PCIE_WR4(sc, PCIE_INT_MASK, val);

        /* Clear stale interrupt status. */
        MTK_PCIE_WR4(sc, PCIE_INT_STATUS, PCIE_INTX_MASK | PCIE_MSI_MASK);

        /* Collect the memory ranges described in the FDT. */
        mems = 0;
        for (i = 0; i < sc->fdt_sc.sc_nrange; i++) {
                switch (sc->fdt_sc.sc_range[i].pci_hi &
                        OFW_PCI_PHYS_HI_SPACEMASK) {
                        case OFW_PCI_PHYS_HI_SPACE_MEM32:
                        case OFW_PCI_PHYS_HI_SPACE_MEM64:
                                ++mems;
                                break;
                        default:
                                break;
                }
        }
        if (mems == 0) {
                device_printf(sc->dev,
                    "Missing required memory range(s) in DT\n");
                return (ENXIO);
        }

        sc->mem_range = malloc(mems * sizeof(*sc->mem_range), M_DEVBUF,
            M_WAITOK);
        sc->mem_ranges = mems;

        mems = 0;
        for (i = 0; i < sc->fdt_sc.sc_nrange; i++) {
                switch (sc->fdt_sc.sc_range[i].pci_hi &
                        OFW_PCI_PHYS_HI_SPACEMASK) {
                        case OFW_PCI_PHYS_HI_SPACE_MEM32:
                        case OFW_PCI_PHYS_HI_SPACE_MEM64:
                                MPASS(mems < sc->mem_ranges);
                                sc->mem_range[mems] = sc->fdt_sc.sc_range[i];
                                ++mems;
                                break;
                        default:
                                device_printf(sc->dev,
                                    "%s: Unsupported range type (0x%X)\n", __func__,
                                    sc->fdt_sc.sc_range[i].pci_hi &
                                    OFW_PCI_PHYS_HI_SPACEMASK);
                                break;
                }
        }
        MPASS(mems == sc->mem_ranges);

        /*
         * The port has a single AHB to PCIe translation window, so
         * program it from the first memory range.  The window is
         * described by the host (CPU side) address.
         */
        mem_base = sc->mem_range[0].host;
        mem_size = sc->mem_range[0].size;

        MTK_PCIE_WR4(sc, PCIE_AHB_TRANS_BASE0_L,
            (uint32_t)mem_base | AHB2PCIE_SIZE(flsll(mem_size)));
        MTK_PCIE_WR4(sc, PCIE_AHB_TRANS_BASE0_H,
            (uint32_t)(mem_base >> 32));

        /* Set the PCIe to AXI (inbound, DMA) translation window. */
        MTK_PCIE_WR4(sc, PCIE_AXI_WINDOW0, PCIE2AHB_SIZE | WIN_ENABLE);

        return (0);
}

static int
mt_pcie_probe(device_t dev)
{
        if (!ofw_bus_status_okay(dev))
                return (ENXIO);
        if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
                return (ENXIO);

        device_set_desc(dev, "MediaTek MT7622 PCIe controller");
        return (BUS_PROBE_DEFAULT);
}

static int
mt_pcie_attach(device_t dev)
{
        struct mt_pcie_softc *sc;
        phandle_t node;
        pcell_t domain;
        int coherent, error, irq_rid, rid;

        sc = device_get_softc(dev);
        sc->dev = dev;
        node = ofw_bus_get_node(dev);

        /* The port index is exposed as the PCI domain number. */
        sc->slot = 0;
        if (OF_getencprop(node, "linux,pci-domain", &domain,
            sizeof(domain)) > 0)
                sc->slot = domain;

        mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

        /* Map the port registers. */
        rid = 0;
        sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
            RF_ACTIVE);
        if (sc->mem_res == NULL) {
                device_printf(dev, "could not allocate memory resource\n");
                error = ENXIO;
                goto fail;
        }

        coherent = OF_hasprop(node, "dma-coherent");
        error = bus_dma_tag_create(bus_get_dma_tag(dev), /* parent */
            1, 0,				/* alignment, bounds */
            BUS_SPACE_MAXADDR,			/* lowaddr */
            BUS_SPACE_MAXADDR,			/* highaddr */
            NULL, NULL,				/* filter, filterarg */
            BUS_SPACE_MAXSIZE,			/* maxsize */
            BUS_SPACE_UNRESTRICTED,		/* nsegments */
            BUS_SPACE_MAXSIZE,			/* maxsegsize */
            coherent ? BUS_DMA_COHERENT : 0,	/* flags */
            NULL, NULL,				/* lockfunc, lockarg */
            &sc->dmat);
        if (error != 0) {
                device_printf(dev, "could not create the DMA tag\n");
                goto fail;
        }

        /* The line INTx and MSI are both delivered on. */
        error = ofw_bus_find_string_index(node, "interrupt-names", "pcie_irq",
            &irq_rid);
        if (error != 0) {
                device_printf(dev, "could not find the 'pcie_irq' interrupt\n");
                goto fail;
        }
        sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &irq_rid,
            RF_ACTIVE | RF_SHAREABLE);
        if (sc->irq_res == NULL) {
                device_printf(dev, "could not allocate IRQ resource\n");
                error = ENXIO;
                goto fail;
        }

        mt_pcie_enable_clocks(dev);

        error = ofw_pcib_init(dev);
        if (error != 0)
                goto fail;

        mt_pcie_ltssm_enable(sc);

        error = mt_pcie_startup_port(sc);
        if (error != 0) {
                device_printf(dev, "link down\n");
                goto fail;
        }

        error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
            mt_pcie_intr, NULL, sc, &sc->irq_cookie);
        if (error != 0) {
                device_printf(dev, "could not setup interrupt handler\n");
                goto fail;
        }

        /* The interrupt handler is in place, MSI can be enabled now. */
        error = mt_pcie_setup_msi(sc);
        if (error != 0)
                goto fail;

        /*
         * Legacy interrupts too, and before ofw_pcib_attach(): that is
         * what probes the children, and a child asks for its interrupt
         * as it attaches.
         */
        error = mt_pcie_setup_intx(sc);
        if (error != 0)
                goto fail;

        return (ofw_pcib_attach(dev));

fail:
        if (sc->irq_cookie != NULL)
        {
                bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
        }

        if(sc->mem_range != NULL)
        {
                free(sc->mem_range, M_DEVBUF);
        }

        if(sc->mem_res != NULL)
        {
                bus_release_resource(dev, SYS_RES_MEMORY, irq_rid, sc->mem_res);
        }

        if(sc->irq_res != NULL)
        {
                bus_release_resource(dev, SYS_RES_IRQ, irq_rid, sc->irq_res);
        }

        if(sc->dmat)
        {
                bus_dma_tag_destroy(sc->dmat);
        }

        mtx_destroy(&sc->mtx);
        ofw_pcib_fini(dev);

        return (error);
}

static device_method_t mtk_pcie_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		mt_pcie_probe),
    DEVMETHOD(device_attach,		mt_pcie_attach),

    /* Bus interface */
    DEVMETHOD(bus_setup_intr,		bus_generic_setup_intr),
    DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),
    DEVMETHOD(bus_get_dma_tag,		mt_pcie_get_dma_tag),

    /* pcib interface */
    DEVMETHOD(pcib_maxslots,		mt_pcie_maxslots),
    DEVMETHOD(pcib_read_config,		mt_pcie_read_config),
    DEVMETHOD(pcib_write_config,	mt_pcie_write_config),
    DEVMETHOD(pcib_get_id,		mt_pcie_get_id),
    DEVMETHOD(pcib_request_feature,	pcib_request_feature_allow),

    /* INTx interrupt controller */
    DEVMETHOD(pic_map_intr,		mt_pcie_pic_map_intr),
    DEVMETHOD(pic_enable_intr,		mt_pcie_pic_enable_intr),
    DEVMETHOD(pic_disable_intr,		mt_pcie_pic_disable_intr),
    DEVMETHOD(pic_post_filter,		mt_pcie_pic_post_filter),
    DEVMETHOD(pic_pre_ithread,		mt_pcie_pic_pre_ithread),
    DEVMETHOD(pic_post_ithread,		mt_pcie_pic_post_ithread),

    /* MSI/MSI-X */
    DEVMETHOD(pcib_alloc_msi,		mt_pcie_alloc_msi),
    DEVMETHOD(pcib_release_msi,		mt_pcie_release_msi),
    DEVMETHOD(pcib_alloc_msix,		mt_pcie_alloc_msix),
    DEVMETHOD(pcib_release_msix,	mt_pcie_release_msix),
    DEVMETHOD(pcib_map_msi,		mt_pcie_map_msi),

    DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, mtk_pcie_driver, mtk_pcie_methods,
sizeof(struct mt_pcie_softc), ofw_pcib_driver);
DRIVER_MODULE(mtk_pcie, simplebus, mtk_pcie_driver, NULL, NULL);
