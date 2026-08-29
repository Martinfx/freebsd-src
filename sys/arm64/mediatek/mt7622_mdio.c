/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mdio/mdio.h>

#include "mdio_if.h"
#include "ofw_bus_if.h"

struct mtmdio_ofw_devinfo {
	struct ofw_bus_devinfo	di_dinfo;
	struct resource_list	di_rl;
};

static int
mtmdio_probe(device_t dev)
{
	phandle_t node;

	node = ofw_bus_get_node(dev);
	if (node == 0) {
                return (ENXIO);
        }

	device_set_desc(dev, "MT7622 MDIO bus");

	return (BUS_PROBE_DEFAULT);
}

static int
mtmdio_attach(device_t dev)
{
	struct mtmdio_ofw_devinfo *di;
	device_t cdev;
	phandle_t node, child;
	int nchildren;

	node = ofw_bus_get_node(dev);
	nchildren = 0;

	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (ofw_bus_node_status_okay(child) == 0)
			continue;
		di = malloc(sizeof(*di), M_DEVBUF, M_WAITOK | M_ZERO);
		if (ofw_bus_gen_setup_devinfo(&di->di_dinfo, child) != 0) {
			free(di, M_DEVBUF);
			continue;
		}
		resource_list_init(&di->di_rl);
		cdev = device_add_child(dev, NULL, DEVICE_UNIT_ANY);
		if (cdev == NULL) {
			device_printf(dev, "could not add child %s\n",
			    di->di_dinfo.obd_name);
			resource_list_free(&di->di_rl);
			ofw_bus_gen_destroy_devinfo(&di->di_dinfo);
			free(di, M_DEVBUF);
			continue;
		}
		device_set_ivars(cdev, di);
		nchildren++;
	}

	if (nchildren == 0)
		device_printf(dev,
		    "no devices described on this MDIO bus\n");

	bus_attach_children(dev);

	return (0);
}

static int
mtmdio_detach(device_t dev)
{
        return (bus_generic_detach(dev));
}

static void
mtmdio_probe_nomatch(device_t bus, device_t child)
{
	const char *name, *compat;

	name = ofw_bus_get_name(child);
	compat = ofw_bus_get_compat(child);

	device_printf(bus, "<%s> compat %s (no driver attached)\n",
	    name != NULL ? name : "unknown",
	    compat != NULL ? compat : "unknown");
}

static int
mtmdio_readreg(device_t dev, int phy, int reg)
{
	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
mtmdio_writereg(device_t dev, int phy, int reg, int val)
{
	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}

static const struct ofw_bus_devinfo *
mtmdio_get_devinfo(device_t bus, device_t child)
{
	struct mtmdio_ofw_devinfo *di;

	di = device_get_ivars(child);
	if (di == NULL)
		return (NULL);
	return (&di->di_dinfo);
}

static struct resource_list *
mtmdio_get_resource_list(device_t bus, device_t child)
{
	struct mtmdio_ofw_devinfo *di;

	di = device_get_ivars(child);
	if (di == NULL)
		return (NULL);
	return (&di->di_rl);
}

static device_method_t mtmdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtmdio_probe),
	DEVMETHOD(device_attach,	mtmdio_attach),
	DEVMETHOD(device_detach,	mtmdio_detach),

	/* Bus interface */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_probe_nomatch,	mtmdio_probe_nomatch),
	DEVMETHOD(bus_alloc_resource,	bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_get_resource_list, mtmdio_get_resource_list),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_devinfo,	mtmdio_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		mtmdio_readreg),
	DEVMETHOD(mdio_writereg,	mtmdio_writereg),

	DEVMETHOD_END
};

static driver_t mtmdio_driver = {
	"mtmdio",
	mtmdio_methods,
	0
};

DRIVER_MODULE(mtmdio, mtge, mtmdio_driver, 0, 0);
MODULE_DEPEND(mtmdio, mdio, 1, 1, 1);
