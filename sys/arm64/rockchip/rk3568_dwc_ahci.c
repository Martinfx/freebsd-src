/*-
* SPDX-License-Identifier: BSD-2-Clause
*
* Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer,
*    without modification, immediately at the beginning of the file.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <sys/param.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ahci/ahci.h>
#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
	{"rockchip,rk3568-dwc-ahci", 1},
	{NULL,			0}
};

struct rk3568_dwc_ahci_softc {
	struct ahci_controller	ahci_ctlr;
	device_t		dev;
	clk_t			clk_sata;
	clk_t			clk_gpmalive;
	clk_t			clk_rxoob;
	phy_t			phy;
};

static int
rk3568_dwc_ahci_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0) {
		return (ENXIO);
	}
	
	device_set_desc(dev, "Synopsys DWC AHCI SATA controller");
	return (BUS_PROBE_VENDOR);
}

static void
rk3568_dump_ahci_status(struct rk3568_dwc_ahci_softc *sc)
{
	uint32_t v;

	/* AHCI controller registers */
	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_P_SSTS);
	device_printf(sc->dev, "AHCI PxSSTS = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_P_SCTL);
	device_printf(sc->dev, "AHCI PxSCTL = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_P_SERR); /* or AHCI_P_SERR */
	device_printf(sc->dev, "AHCI PxSERR = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_P_CMD);
	device_printf(sc->dev, "AHCI PxCMD = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_P_IS);
	device_printf(sc->dev, "AHCI PxIS = 0x%08x\n", v);

	/* AHCI global */
	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_CAP);
	device_printf(sc->dev, "AHCI CAP = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_CAP2);
	device_printf(sc->dev, "AHCI CAP2 = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_PI);
	device_printf(sc->dev, "AHCI PI = 0x%08x\n", v);

	v = ATA_INL(sc->ahci_ctlr.r_mem, AHCI_GHC);
	device_printf(sc->dev, "AHCI GHC = 0x%08x\n", v);
}

static int
rk3568_dwc_ahci_attach(device_t dev)
{
	struct rk3568_dwc_ahci_softc *sc;
	struct ahci_controller *ctlr;
	phandle_t node;
	int rv, error;
	uint32_t ports, len;

	sc = device_get_softc(dev);
	sc->dev = dev;;
	node = ofw_bus_get_node(dev);
	ctlr = &sc->ahci_ctlr;

	ctlr->r_rid = 0;
	ctlr->r_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &ctlr->r_rid, RF_ACTIVE);
	if (ctlr->r_mem == NULL) {
		panic("Cannot allocate memory resources");
		return (ENXIO);
	}

	len = OF_getencprop(node, "ports-implemented",
	    &ports, sizeof(ports));
	if (len != sizeof(ports)) {
		device_printf(dev, "Cannot get ports-implemented\n");
		return (ENXIO);
	}

	device_printf(dev, "ports-implemented: %d\n", ports);

	error = clk_get_by_ofw_name(sc->dev, 0,"sata",&sc->clk_sata);
	if (error != 0) {
		device_printf(dev, "Cannot get clk_sata clock\n");
		goto fail;
	}
	error = clk_get_by_ofw_name(sc->dev, 0,"pmalive",&sc->clk_gpmalive);
	if (error != 0) {
		device_printf(dev, "Cannot get clk_gpmalive clock\n");
		goto fail;
	}
	error = clk_get_by_ofw_name(sc->dev, 0,"rxoob",&sc->clk_rxoob);
	if (error != 0) {
		device_printf(dev, "Cannot get clk_rxoob clock\n");
		goto fail;
	}

	error = clk_enable(sc->clk_sata);
	if (error != 0) {
		device_printf(dev, "Cannot enable clk_sata\n");
		goto fail;
	}
	error = clk_enable(sc->clk_gpmalive);
	if (error != 0) {
		device_printf(dev, "Cannot enable clk_gpmalive\n");
		goto fail;
	}
	error = clk_enable(sc->clk_rxoob);
	if (error != 0) {
		device_printf(dev, "Cannot enable clk clk_rxoob\n");
		goto fail;
	}

	rv = phy_get_by_ofw_name(sc->dev, 0, "sata-phy", &sc->phy);
	if (rv != 0) {
		rv = phy_get_by_ofw_idx(sc->dev, 0, 0, &sc->phy);
		if (rv != 0) {
			device_printf(sc->dev, "Cannot get 'sata-phy' phy\n");
			return (ENXIO);
		}
	}

	rk3568_dump_ahci_status(sc);


	uint32_t val;
	phy_status(sc->phy, &val);
	device_printf(sc->dev, "phy status: 0x%08x\n", val);

	error = phy_enable(sc->phy);
	if (error != 0) {
		device_printf(sc->dev, "Cannot enable sata-phy phy\n");
		goto fail;
	}
	
	phy_status(sc->phy, &val);
	device_printf(sc->dev, "phy status: 0x%08x\n", val);

	rk3568_dump_ahci_status(sc);

	/* Reset controller */
	error = ahci_ctlr_reset(dev);
	if (error != 0) {
		device_printf(dev, "Failed to reset controller\n");
		goto fail;
	}

	rk3568_dump_ahci_status(sc);

	/* Setup controller defaults. */
	//device_printf(dev, "Defaults value ctlr->msi: %d \n", ctlr->msi);
	ctlr->quirks = AHCI_Q_FORCE_PI | AHCI_Q_SLOWDEV;
	ctlr->numirqs = 1;

	/* call generic AHCI attach */
	rv = ahci_attach(dev);
	if (rv != 0) {
		device_printf(dev, "ahci_attach failed: %d\n", rv);
		goto fail;
	}

	return (0);

fail:
	if (sc->clk_sata != NULL) {
		clk_release(sc->clk_sata);
	}
	if (sc->clk_gpmalive != NULL) {
		clk_release(sc->clk_gpmalive);
	}
	if (sc->clk_rxoob != NULL) {
		clk_release(sc->clk_rxoob);
	}

	bus_release_resource(dev, SYS_RES_MEMORY, ctlr->r_rid, ctlr->r_mem);
	return (error);
}

static int
rk3568_dwc_ahci_detach(device_t dev)
{
	ahci_detach(dev);
	return (0);
}

static int
rk3568_dwc_ahci_suspend(device_t dev)
{
	struct rk3568_dwc_ahci_softc *sc = device_get_softc(dev);

	bus_generic_suspend(dev);

	ATA_OUTL(sc->ahci_ctlr.r_mem, AHCI_GHC,
	    ATA_INL(sc->ahci_ctlr.r_mem, AHCI_GHC) & (~AHCI_GHC_IE));
	return (0);
}

static int
rk3568_dwc_ahci_resume(device_t dev)
{
	int res;
	if ((res = ahci_ctlr_reset(dev)) != 0) {
		return (res);
	}
	ahci_ctlr_setup(dev);
	return (bus_generic_resume(dev));
}


static device_method_t rk3568_dwc_ahci_methods[] = {
	DEVMETHOD(device_probe,		rk3568_dwc_ahci_probe),
	DEVMETHOD(device_attach,	rk3568_dwc_ahci_attach),
	DEVMETHOD(device_detach,	rk3568_dwc_ahci_detach),
	DEVMETHOD(device_suspend,	rk3568_dwc_ahci_suspend),
	DEVMETHOD(device_resume,	rk3568_dwc_ahci_resume),
	DEVMETHOD(bus_print_child,	ahci_print_child),
	DEVMETHOD(bus_alloc_resource,	ahci_alloc_resource),
	DEVMETHOD(bus_release_resource,	ahci_release_resource),
	DEVMETHOD(bus_setup_intr,	ahci_setup_intr),
	DEVMETHOD(bus_teardown_intr,	ahci_teardown_intr),
	DEVMETHOD(bus_child_location,	ahci_child_location),
	DEVMETHOD(bus_get_dma_tag,	ahci_get_dma_tag),

	DEVMETHOD_END
};

static DEFINE_CLASS_0(ahci, rk3568_dwc_ahci_driver, rk3568_dwc_ahci_methods,
    sizeof(struct rk3568_dwc_ahci_softc));
DRIVER_MODULE(rk3568_dwc_ahci, simplebus,rk3568_dwc_ahci_driver, NULL, NULL);