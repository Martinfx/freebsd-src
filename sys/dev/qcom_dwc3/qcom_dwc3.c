/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Adrian Chadd <adrian@FreeBSD.Org>
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

/*
 * Qualcomm DWC3 glue
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/gpio.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/clk/clk.h>
#include <dev/hwreset/hwreset.h>
#include <dev/phy/phy_usb.h>
#include <dev/syscon/syscon.h>

static struct ofw_compat_data compat_data[] = {
	{ "qcom,dwc3",			1},
	{ NULL,				0 }
};

struct qcom_dwc3_softc {
	struct simplebus_softc	sc;
	device_t		dev;
	clk_t			clk_core;
	clk_t			clk_sleep;
	clk_t			clk_mock_utmi;
	int			type;
};

static int
qcom_dwc3_probe(device_t dev)
{
	phandle_t node;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	/* Binding says that we need a child node for the actual dwc3 controller */
	node = ofw_bus_get_node(dev);
	if (OF_child(node) <= 0)
		return (ENXIO);

	device_set_desc(dev, "Qualcomm DWC3");
	return (BUS_PROBE_DEFAULT);
}

static int
qcom_dwc3_attach(device_t dev)
{
	struct qcom_dwc3_softc *sc;
	device_t cdev;
	phandle_t node, child;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	/* Mandatory clocks */
	if (clk_get_by_ofw_name(dev, 0, "core", &sc->clk_core) != 0) {
		device_printf(dev, "Cannot get core clock\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "sleep", &sc->clk_sleep) != 0) {
		device_printf(dev, "Cannot get sleep clock\n");
		return (ENXIO);
	}

	if (clk_get_by_ofw_name(dev, 0, "mock_utmi", &sc->clk_mock_utmi) != 0) {
		device_printf(dev, "Cannot get mock_utmi clock\n");
		return (ENXIO);
	}

	/*
	 * TODO: when we support optional reset blocks, take things
	 * out of reset (well, put them into reset, then take out of reset.)
	 */

	/*
	 * Now, iterate over the clocks and enable them.
	 */
	err = clk_enable(sc->clk_core);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->clk_core));
		return (ENXIO);
	}
	err = clk_enable(sc->clk_sleep);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->clk_sleep));
		return (ENXIO);
	}
	err = clk_enable(sc->clk_mock_utmi);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
		    clk_get_name(sc->clk_mock_utmi));
		return (ENXIO);
	}

	/*
	 * Rest is glue code.
	 */

	simplebus_init(dev, node);
	if (simplebus_fill_ranges(node, &sc->sc) < 0) {
		device_printf(dev, "could not get ranges\n");
		return (ENXIO);
	}

	for (child = OF_child(node); child > 0; child = OF_peer(child)) {
		cdev = simplebus_add_device(dev, child, 0, NULL, -1, NULL);
		if (cdev != NULL)
			device_probe_and_attach(cdev);
	}

	bus_attach_children(dev);
	return (0);
}

static device_method_t qcom_dwc3_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		qcom_dwc3_probe),
	DEVMETHOD(device_attach,	qcom_dwc3_attach),
	/* XXX TODO suspend */
	/* XXX TODO resume */

	DEVMETHOD_END
};

DEFINE_CLASS_1(qcom_dwc3, qcom_dwc3_driver, qcom_dwc3_methods,
    sizeof(struct qcom_dwc3_softc), simplebus_driver);
DRIVER_MODULE(qcom_dwc3, simplebus, qcom_dwc3_driver, 0, 0);
