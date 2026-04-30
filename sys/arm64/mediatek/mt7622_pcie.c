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
#include <dev/syscon/syscon.h>

#include "pcib_if.h"
#include "syscon_if.h"

#define PCIE_SYS_CFG_V2         0x0
#define	PCIE_CSR_LTSSM_EN(x)	(1U << (0 + (x) * 8))
#define	PCIE_CSR_ASPM_L1_EN(x)	(1U << (1 + (x) * 8))
#define PCIE_RST_CTRL           0x510
#define PCIE_PHY_RSTB           (1U << 0)
#define PCIE_PIPE_SRSTB         (1U << 1)
#define PCIE_MAC_SRSTB          (1U << 2)
#define PCIE_CRSTB              (1U << 3)
#define PCIE_PERSTB             (1U << 8)
#define	PCIE_LINKDOWN_RST_EN	(0x7U << 13)	/* bits 15:13 */
#define	PCIE_LINK_STATUS_V2	0x804
#define	PCIE_PORT_LINKUP_V2	(1U << 10)
//#define	PCIE_PORT_LINKUP_V2	(1U << 0)

#define PCIE_INT_MASK		0x420
#define INTX_MASK    		(0xF << 16)
#define INTX_SHIFT		16
#define PCIE_INT_STATUS		0x424
#define	AHB2PCIE_SIZE(x)	((x) & 0x1f)	/* bits 4:0 */
#define	PCIE_AXI_WINDOW0	0x448
#define	WIN_ENABLE		(1U << 7)
#define PCIE_AHB_TRANS_BASE0_L	0x438
#define PCIE_AHB_TRANS_BASE0_H	0x43c
#define PCIE2AHB_SIZE		0x21

/* PCIe V2 configuration transaction header */
#define PCIE_CFG_HEADER0	0x460
#define PCIE_CFG_HEADER1	0x464
#define PCIE_CFG_HEADER2	0x468
#define PCIE_CFG_WDATA		0x470
#define PCIE_APP_TLP_REQ	0x488
#define PCIE_CFG_RDATA		0x48c
#define	APP_CFG_REQ		(1U << 0)
#define CFG_WRRD_TYPE_0		4
#define CFG_WR_FMT		2
#define CFG_RD_FMT		0
#define	APP_CPL_STATUS		(0x7U << 5)	/* bits 7:5 */

#define	CFG_DW0_LENGTH(length)	((length) & 0x3ffU)		/* bits  9:0  */
#define	CFG_DW0_TYPE(type)	(((type) << 24) & 0x1f000000U)	/* bits 28:24 */
#define	CFG_DW0_FMT(fmt)	(((fmt) << 29) & 0xe0000000U)	/* bits 31:29 */

#define	CFG_DW2_REGN(regn)	((regn) & 0xffcU)		/* bits 11:2  */
#define	CFG_DW2_FUN(fun)	(((fun) << 16) & 0x70000U)	/* bits 18:16 */
#define	CFG_DW2_DEV(dev)	(((dev) << 19) & 0xf80000U)	/* bits 23:19 */
#define	CFG_DW2_BUS(bus)	(((bus) << 24) & 0xff000000U)	/* bits 31:24 */

#define	CFG_HEADER_DW0(type, fmt) \
	(CFG_DW0_LENGTH(1) | CFG_DW0_TYPE(type) | CFG_DW0_FMT(fmt))

#define	CFG_HEADER_DW1(where, size) \
	(((1U << (size)) - 1) << ((where) & 0x3))

#define	CFG_HEADER_DW2(regn, fun, dev, bus) \
	(CFG_DW2_REGN(regn) | CFG_DW2_FUN(fun) | \
	 CFG_DW2_DEV(dev) | CFG_DW2_BUS(bus))

#define	PCIE_CONF_VEND_ID	 0x100
#define	PCIE_CONF_DEVICE_ID	 0x102
#define	PCIE_CONF_CLASS_ID	 0x106
#define PCI_VENDOR_ID_MEDIATEK	 0x14c3
#define PCI_DEVICE_ID_MT7622     0x5396
#define PCI_CLASS_BRIDGE_PCI	 0x0604

struct mt_pcie_port {
	struct resource *res_mem;
	int rid_mem;
	int slot;
	clk_t sys_ck, ahb_ck, aux_ck, axi_ck, obff_ck, pipe_ck;
	struct mtx mtx;
};

struct mt7622_pcie_softc {
	struct ofw_pci_softc ofw_pci;
	struct ofw_pci_range pref_mem_range;
	struct ofw_pci_range io_range;
	struct ofw_pci_range mem_range;
	struct resource *cfg_res;
	struct resource *irq_res;
	device_t dev;
	int irq_rid;
	struct mt_pcie_port ports[2];
	int nports;
	phandle_t node;
	struct syscon *syscon;
	void *irq_cookie;
};

static const char * const mt_clk_names[] = {
	"sys_ck", "ahb_ck", "axi_ck", "aux_ck", "obff_ck", "pipe_ck"
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt7622-pcie", 1},
	{NULL,                   0}
};

static int
mt7622_pcib_maxslots(device_t dev) {
	return (0);   /* slot 0 */
}

static int
mt7622_pcie_sys_irq(void *arg)
{
	struct mt7622_pcie_softc *sc = arg;
	struct mt_pcie_port *port;
	uint32_t status;
	int i;

	for (i = 0; i < sc->nports; i++) {
		port = &sc->ports[i];
		if (port->res_mem == NULL)
			continue;
		status = bus_read_4(port->res_mem, PCIE_INT_STATUS);
		if (status & INTX_MASK) {
			/* clear pending */
			bus_write_4(port->res_mem, PCIE_INT_STATUS,
			    status & INTX_MASK);
		}
	}
	return (FILTER_HANDLED);
}

static struct mt_pcie_port *
mt7622_pcie_find_port(struct mt7622_pcie_softc *sc, u_int bus, u_int slot)
{
	int i;

	if (bus == 0) {
		for (i = 0; i < sc->nports; i++) {
			if (sc->ports[i].res_mem != NULL &&
			    sc->ports[i].slot == (int)slot)
				return (&sc->ports[i]);
		}
		return (NULL);
	}
	/* deeper bus: vrať libovolný aktivní port — bridge ti nasměruje TLP */
	for (i = 0; i < sc->nports; i++) {
		if (sc->ports[i].res_mem != NULL)
			return (&sc->ports[i]);
	}
	return (NULL);
}

static int
mt_pcie_hw_wr_cfg(struct mt_pcie_port *port, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes, uint32_t val)
{
	uint32_t v;
	int i;

	mtx_lock(&port->mtx);

	bus_write_4(port->res_mem, PCIE_CFG_HEADER0,
	    CFG_HEADER_DW0(CFG_WRRD_TYPE_0, CFG_WR_FMT));
	bus_write_4(port->res_mem, PCIE_CFG_HEADER1,
	    CFG_HEADER_DW1(reg, bytes));
	bus_write_4(port->res_mem, PCIE_CFG_HEADER2,
	    CFG_HEADER_DW2(reg, func, slot, bus));

	val <<= 8 * (reg & 3);
	bus_write_4(port->res_mem, PCIE_CFG_WDATA, val);

	v = bus_read_4(port->res_mem, PCIE_APP_TLP_REQ) | APP_CFG_REQ;
	bus_write_4(port->res_mem, PCIE_APP_TLP_REQ, v);

	for (i = 0; i < 10000; i++) {
		v = bus_read_4(port->res_mem, PCIE_APP_TLP_REQ);
		if ((v & APP_CFG_REQ) == 0)
			break;
		DELAY(10);
	}
	mtx_unlock(&port->mtx);

	if (i == 10000)
		return (-1);
	if (v & APP_CPL_STATUS)
		return (-1);
	return (0);
}


static uint32_t
mt7622_pcib_read_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, int bytes)
{
	struct mt7622_pcie_softc *sc = device_get_softc(dev);
	struct mt_pcie_port *port;
	uint32_t v;
	int i;
	device_printf(dev, "rd cfg b=%u s=%u f=%u r=0x%x bytes=%d\n",
	    bus, slot, func, reg, bytes);
	port = mt7622_pcie_find_port(sc, bus, slot);
	if (port == NULL) {
		device_printf(dev, "rd cfg b=%u s=%u f=%u r=0x%x bytes=%d\n",
		    bus, slot, func, reg, bytes);
		return (~0U);
	}

	mtx_lock(&port->mtx);

	bus_write_4(port->res_mem, PCIE_CFG_HEADER0,
	    CFG_HEADER_DW0(CFG_WRRD_TYPE_0, CFG_RD_FMT));
	bus_write_4(port->res_mem, PCIE_CFG_HEADER1, CFG_HEADER_DW1(reg, bytes));
	bus_write_4(port->res_mem, PCIE_CFG_HEADER2, CFG_HEADER_DW2(reg, func, slot, bus));

	v = bus_read_4(port->res_mem, PCIE_APP_TLP_REQ) | APP_CFG_REQ;
	bus_write_4(port->res_mem, PCIE_APP_TLP_REQ, v);

	for (i = 0; i < 10000; i++) {
		v = bus_read_4(port->res_mem, PCIE_APP_TLP_REQ);
		if ((v & APP_CFG_REQ) == 0)
			break;
		DELAY(10);
	}
	if (i == 10000) {
		mtx_unlock(&port->mtx);
		return (~0U);
	}
	if (bus_read_4(port->res_mem, PCIE_APP_TLP_REQ) & APP_CPL_STATUS) {
		mtx_unlock(&port->mtx);
		return (~0U);
	}

	v = bus_read_4(port->res_mem, PCIE_CFG_RDATA);
	mtx_unlock(&port->mtx);

	uint32_t result;
	switch (bytes) {
	case 1: result = (v >> (8 * (reg & 3))) & 0xff; break;
	case 2: result = (v >> (8 * (reg & 3))) & 0xffff; break;
	case 4: result = v; break;
	default: result = ~0U; break;
	}

	device_printf(dev, "rd b=%u s=%u r=0x%x b=%d -> 0x%x (i=%d)\n",
	    bus, slot, reg, bytes, result, i);

	return (result);
}

static void
mt7622_pcib_write_config(device_t dev, u_int bus, u_int slot, u_int func,
    u_int reg, uint32_t val, int bytes)
{
	struct mt7622_pcie_softc *sc = device_get_softc(dev);
	struct mt_pcie_port *port;

	port = mt7622_pcie_find_port(sc, bus, slot);
	if (port == NULL) {
		return;
	}

	(void)mt_pcie_hw_wr_cfg(port, bus, slot, func, reg, bytes, val);
}

static int
mt7622_pcie_port_clocks(device_t dev, struct mt_pcie_port *port)
{
	clk_t *cks[] = {
		&port->sys_ck, &port->ahb_ck, &port->axi_ck,
		&port->aux_ck, &port->obff_ck, &port->pipe_ck
	};
	char name[16];
	int i, err;

	for (i = 0; i < nitems(mt_clk_names); i++) {
		snprintf(name, sizeof(name), "%s%d",
		    mt_clk_names[i], port->slot);
		err = clk_get_by_ofw_name(dev, 0, name, cks[i]);
		if (err != 0) {
			if (i == 0) {
				device_printf(dev, "missing %s\n", name);
				return (ENXIO);
			}
			*cks[i] = NULL;
			continue;
		}
		err = clk_enable(*cks[i]);
		if (err != 0) {
			device_printf(dev, "enable %s failed: %d\n", name, err);
			return (err);
		}
		if(bootverbose) {
			device_printf(dev, "clock pcie enabled %s\n", name);
		}
	}
	return (0);
}

static int
mt7622_pcie_port_start(struct mt7622_pcie_softc *sc, struct mt_pcie_port *port)
{
	uint32_t v;
	int i;

	device_printf(sc->dev,
	    "port%d ENTER: RST=0x%08x LINK=0x%08x SYS=0x%08x INT_MASK=0x%08x\n",
	    port->slot,
	    bus_read_4(port->res_mem, PCIE_RST_CTRL),
	    bus_read_4(port->res_mem, PCIE_LINK_STATUS_V2),
	    SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2),
	    bus_read_4(port->res_mem, PCIE_INT_MASK));

	v = SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2);
	v |= PCIE_CSR_LTSSM_EN(port->slot) | PCIE_CSR_ASPM_L1_EN(port->slot);
	SYSCON_WRITE_4(sc->syscon, PCIE_SYS_CFG_V2, v);
	DELAY(100000);
	bus_write_4(port->res_mem, PCIE_RST_CTRL, 0);
	bus_write_4(port->res_mem, PCIE_RST_CTRL, PCIE_LINKDOWN_RST_EN);
	DELAY(100* 1000);

	v = bus_read_4(port->res_mem, PCIE_RST_CTRL);
	v |= PCIE_PHY_RSTB | PCIE_PERSTB | PCIE_PIPE_SRSTB |
	    PCIE_MAC_SRSTB | PCIE_CRSTB;
	bus_write_4(port->res_mem, PCIE_RST_CTRL, v);

	device_printf(sc->dev,
	    "port%d POST-RST: RST=0x%08x LINK=0x%08x SYS=0x%08x\n",
	    port->slot,
	    bus_read_4(port->res_mem, PCIE_RST_CTRL),
	    bus_read_4(port->res_mem, PCIE_LINK_STATUS_V2),
	    SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2));

	for (i = 0; i < 100000; i++) {
		if (bus_read_4(port->res_mem, PCIE_LINK_STATUS_V2) &
		    PCIE_PORT_LINKUP_V2) {
			break;
		}
		DELAY(100);
	}
	device_printf(sc->dev, "port%d:  us (i=%d)\n", port->slot, i);
	device_printf(sc->dev,
	    "port%d AFTER-POLL (i=%d): RST=0x%08x LINK=0x%08x\n",
	    port->slot, i,
	    bus_read_4(port->res_mem, PCIE_RST_CTRL),
	    bus_read_4(port->res_mem, PCIE_LINK_STATUS_V2));

	v = SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2);
	device_printf(sc->dev, "SYS pre-OR  = 0x%08x\n", v);
	v |= PCIE_CSR_LTSSM_EN(port->slot) | PCIE_CSR_ASPM_L1_EN(port->slot);
	device_printf(sc->dev, "SYS to-write= 0x%08x\n", v);
	SYSCON_WRITE_4(sc->syscon, PCIE_SYS_CFG_V2, v);
	device_printf(sc->dev, "SYS post-WR = 0x%08x\n",
	    SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2));

	SYSCON_WRITE_4(sc->syscon, PCIE_SYS_CFG_V2, 0xdeadbeef);
	device_printf(sc->dev, "SYS sanity = 0x%08x\n",
	    SYSCON_READ_4(sc->syscon, PCIE_SYS_CFG_V2));

	if (i == 100000) {
		return (ETIMEDOUT);
	}

	device_printf(sc->dev, "port%d: link up after %d us (i=%d)\n",
	    port->slot, i * 100, i);

	v = bus_read_4(port->res_mem, PCIE_INT_MASK);
	v &= ~INTX_MASK;
	bus_write_4(port->res_mem, PCIE_INT_MASK, v);

	v = (uint32_t)sc->mem_range.pci |
	    AHB2PCIE_SIZE(flsl((u_long)sc->mem_range.size));
	bus_write_4(port->res_mem, PCIE_AHB_TRANS_BASE0_L, v);
	bus_write_4(port->res_mem, PCIE_AHB_TRANS_BASE0_H,
	    (uint32_t)(sc->mem_range.pci >> 32));

	bus_write_4(port->res_mem, PCIE_AXI_WINDOW0,
	    PCIE2AHB_SIZE | WIN_ENABLE);

	/* set right vendor id and device id */
	v = PCI_VENDOR_ID_MEDIATEK | (PCI_DEVICE_ID_MT7622 << 16);
	bus_write_4(port->res_mem, PCIE_CONF_VEND_ID, v);
	bus_write_2(port->res_mem, PCIE_CONF_CLASS_ID, PCI_CLASS_BRIDGE_PCI);

	return (0);
}

static int
mt7622_pcie_setup_port(device_t dev, int slot)
{
	struct mt7622_pcie_softc *sc = device_get_softc(dev);
	struct mt_pcie_port *port = &sc->ports[slot];
	char name[8];
	int err;

	snprintf(name, sizeof(name), "port%d", slot);
	if (ofw_bus_find_string_index(sc->node, "reg-names", name,
		&port->rid_mem) != 0) {
		return (ENOENT);
	}

	port->slot = slot;
	mtx_init(&port->mtx, "mt_pcie", NULL, MTX_DEF);

	port->res_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &port->rid_mem, RF_ACTIVE);
	if (port->res_mem == NULL) {
		device_printf(dev, "port%d: cannot map mmio\n", slot);
		mtx_destroy(&port->mtx);
		return (ENXIO);
	}

	if ((err = mt7622_pcie_port_clocks(dev, port)) != 0)
		return (err);

	return (mt7622_pcie_port_start(sc, port));
}

static int
mt7622_pcie_decode_ranges(struct mt7622_pcie_softc *sc, struct ofw_pci_range *ranges, int nranges)
{
	int i;
	for (i = 0; i < nranges; i++) {
		switch (ranges[i].pci_hi & OFW_PCI_PHYS_HI_SPACEMASK) {
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
			break;
		default:
			device_printf(sc->dev,
			    "Unknown PCI space type in range #%d: 0x%08x\n",
			    i, ranges[i].pci_hi);
			break;
		}
	}

	if (sc->mem_range.size == 0) {
		device_printf(sc->dev,
		    "At least one memory range must be defined in DT for MT7622\n");
		return (ENXIO);
	}

	return (0);
}

static int
mt7622_pcie_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Mediatek PCIe controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mt7622_pcie_attach(device_t dev)
{
	struct mt7622_pcie_softc *sc;
	phandle_t syscon_node;
	bus_addr_t cfg_addr;
	bus_size_t cfg_size;
	int i, slot, error, cfg_rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	syscon_node = ofw_bus_find_compatible(OF_finddevice("/"),
	    "mediatek,generic-pciecfg");
	if (syscon_node <= 0) {
		device_printf(dev, "no pciecfg node\n");
		return (ENXIO);
	}

	if (ofw_reg_to_paddr(syscon_node, 0, &cfg_addr, &cfg_size, NULL) != 0) {
		device_printf(dev, "pciecfg reg parse failed\n");
		return (ENXIO);
	}

	sc->cfg_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &cfg_rid,
	    cfg_addr, cfg_addr + cfg_size - 1, cfg_size, RF_ACTIVE);
	if (sc->cfg_res == NULL) {
		device_printf(dev, "cannot map pciecfg\n");
		return (ENXIO);
	}

	/* 2. shared IRQ */
	if (ofw_bus_find_string_index(sc->node, "interrupt-names",
		"pcie_irq", &sc->irq_rid) != 0) {
		device_printf(dev, "no pcie_irq\n");
		return (ENXIO);
	}

	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL)
		return (ENXIO);

	/* 3. ranges  ofw_pci */
	if ((error = ofw_pcib_init(dev)) != 0) {
		device_printf(dev, "ofw_pcib_init: %d\n", error);
		goto fail;
	}

	if ((error = mt7622_pcie_decode_ranges(sc, sc->ofw_pci.sc_range,
		 sc->ofw_pci.sc_nrange)) != 0)
		goto fail;

	/* 4. bring up ports */
	sc->nports = 0;
	for (slot = 0; slot < 2; slot++) {
		error = mt7622_pcie_setup_port(dev, slot);
		if (error == ENOENT) {
			// port is not in DTS, skip it
			continue;
		}

		if (error != 0) {
			device_printf(dev, "port%d error: %d\n", slot, error);
			continue;
		}

		if(bootverbose) {
			device_printf(dev, "port%d up, error: %d\n", slot, error);
		}

 		sc->nports++;
	}

	//sc->nports = 2;

	if(sc->nports == 0) {
		device_printf(dev, "no port came up\n");
		return (ENXIO);
	}

	/* 5. shared interrupt */
	if ((error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
		 mt7622_pcie_sys_irq, NULL, sc, &sc->irq_cookie)) != 0) {
		device_printf(dev, "bus_setup_intr: %d\n", error);
		goto fail;
	}

	error = ofw_pcib_attach(dev);
	if (error != 0) {
		return (ENXIO);
	}

	return (0);

fail:

	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);

	for (i = 0; i < sc->nports; i++) {
		struct mt_pcie_port *p = &sc->ports[i];
		if (p->res_mem == NULL)
			continue;
		if (p->pipe_ck) clk_disable(p->pipe_ck), clk_release(p->pipe_ck);
		if (p->obff_ck) clk_disable(p->obff_ck), clk_release(p->obff_ck);
		if (p->axi_ck)  clk_disable(p->axi_ck),  clk_release(p->axi_ck);
		if (p->aux_ck)  clk_disable(p->aux_ck),  clk_release(p->aux_ck);
		if (p->ahb_ck)  clk_disable(p->ahb_ck),  clk_release(p->ahb_ck);
		if (p->sys_ck)  clk_disable(p->sys_ck),  clk_release(p->sys_ck);
		bus_release_resource(dev, SYS_RES_MEMORY, p->rid_mem, p->res_mem);
		mtx_destroy(&p->mtx);
	}

	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
	}

	ofw_pcib_fini(dev);

	return (error);
}

static int
mt7622_pcie_detach(device_t dev)
{
	struct mt7622_pcie_softc *sc = device_get_softc(dev);
	int i;

	if (sc->irq_cookie != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);

	for (i = 0; i < sc->nports; i++) {
		struct mt_pcie_port *p = &sc->ports[i];
		if (p->res_mem == NULL)
			continue;
		if (p->pipe_ck) clk_disable(p->pipe_ck), clk_release(p->pipe_ck);
		if (p->obff_ck) clk_disable(p->obff_ck), clk_release(p->obff_ck);
		if (p->axi_ck)  clk_disable(p->axi_ck),  clk_release(p->axi_ck);
		if (p->aux_ck)  clk_disable(p->aux_ck),  clk_release(p->aux_ck);
		if (p->ahb_ck)  clk_disable(p->ahb_ck),  clk_release(p->ahb_ck);
		if (p->sys_ck)  clk_disable(p->sys_ck),  clk_release(p->sys_ck);
		bus_release_resource(dev, SYS_RES_MEMORY, p->rid_mem, p->res_mem);
		mtx_destroy(&p->mtx);
	}

	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);

	ofw_pcib_fini(dev);
	return (0);
}

static device_method_t mt7622_pcie_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, mt7622_pcie_probe),
	DEVMETHOD(device_attach, mt7622_pcie_attach),
	DEVMETHOD(device_detach, mt7622_pcie_detach),

	/* Bus interface */
	DEVMETHOD(bus_setup_intr, bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr, bus_generic_teardown_intr),

	/* pcib interface */
	DEVMETHOD(pcib_maxslots,		mt7622_pcib_maxslots),
	DEVMETHOD(pcib_read_config,		mt7622_pcib_read_config),
	DEVMETHOD(pcib_write_config,    	mt7622_pcib_write_config),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	DEVMETHOD_END
};

DEFINE_CLASS_1(pcib, mt7622_pcie_driver, mt7622_pcie_methods, sizeof(struct
	mt7622_pcie_softc), ofw_pcib_driver); DRIVER_MODULE(mt7622_pcie, simplebus,
    	mt7622_pcie_driver, NULL, NULL);
