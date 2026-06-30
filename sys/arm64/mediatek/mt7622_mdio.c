/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * OFW MDIO bus for the MediaTek MT7622 ethernet MAC.
 *
 * The MAC (rt) creates one instance of this bus from the "mdio-bus"
 * subnode of its device-tree node.  This driver enumerates the children
 * of that node (e.g. the MT7531 switch at switch@1f) as plain OF devices,
 * so they probe on their own compatible strings, and forwards all MDIO
 * register accesses to the parent MAC, which owns the MDIO registers.
 * The MAC itself carries no knowledge of what hangs off the bus.
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

struct mtkmdio_ofw_devinfo {
	struct ofw_bus_devinfo	di_dinfo;
	struct resource_list	di_rl;
};

static int
mtkmdio_probe(device_t dev)
{
	phandle_t node;

	node = ofw_bus_get_node(dev);
	if (node == 0 || node == (phandle_t)-1)
		return (ENXIO);

	device_set_desc(dev, "MT7622 MDIO bus");

	return (BUS_PROBE_DEFAULT);
}

static int
mtkmdio_attach(device_t dev)
{
	struct mtkmdio_ofw_devinfo *di;
	device_t cdev;
	phandle_t node, child;

	node = ofw_bus_get_node(dev);

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
	}

	bus_attach_children(dev);

	return (0);
}

static int
mtkmdio_detach(device_t dev)
{

	return (bus_generic_detach(dev));
}

/* The parent MAC owns the MDIO registers; forward accesses to it. */
static int
mtkmdio_readreg(device_t dev, int phy, int reg)
{

	return (MDIO_READREG(device_get_parent(dev), phy, reg));
}

static int
mtkmdio_writereg(device_t dev, int phy, int reg, int val)
{

	return (MDIO_WRITEREG(device_get_parent(dev), phy, reg, val));
}

static const struct ofw_bus_devinfo *
mtkmdio_get_devinfo(device_t bus, device_t child)
{
	struct mtkmdio_ofw_devinfo *di;

	di = device_get_ivars(child);
	if (di == NULL)
		return (NULL);
	return (&di->di_dinfo);
}

static struct resource_list *
mtkmdio_get_resource_list(device_t bus, device_t child)
{
	struct mtkmdio_ofw_devinfo *di;

	di = device_get_ivars(child);
	if (di == NULL)
		return (NULL);
	return (&di->di_rl);
}

static device_method_t mtkmdio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtkmdio_probe),
	DEVMETHOD(device_attach,	mtkmdio_attach),
	DEVMETHOD(device_detach,	mtkmdio_detach),

	/* Bus interface */
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_alloc_resource,	bus_generic_rl_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_rl_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_get_resource_list, mtkmdio_get_resource_list),

	/* OFW bus interface */
	DEVMETHOD(ofw_bus_get_devinfo,	mtkmdio_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	/* MDIO interface */
	DEVMETHOD(mdio_readreg,		mtkmdio_readreg),
	DEVMETHOD(mdio_writereg,	mtkmdio_writereg),

	DEVMETHOD_END
};

static driver_t mtkmdio_driver = {
	"mtkmdio",
	mtkmdio_methods,
	0
};

DRIVER_MODULE(mtkmdio, rt, mtkmdio_driver, 0, 0);
MODULE_DEPEND(mtkmdio, mdio, 1, 1, 1);
