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

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_pci.h>
#include <dev/ofw/ofwpci.h>
#include <dev/syscon/syscon.h>
#include "syscon_if.h"

struct mt_pciecfg_softc {
	device_t dev;
	struct resource *mem_res;
	struct syscon *syscon;
	struct mtx mtx;
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,generic-pciecfg", 1},
	{NULL,                   0}
};

static int
mt_pciecfg_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek pciecfg controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mt_pciecfg_attach(device_t dev)
{
	struct mt_pciecfg_softc *sc;
	int rid = 0;

	sc = device_get_softc(dev);
	sc->dev = dev;

	if (ofw_bus_is_compatible(dev, "syscon")) {
		sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
		    RF_ACTIVE);
		if (sc->mem_res == NULL) {
			device_printf(dev,
			    "Cannot allocate memory resource\n");
			return (ENXIO);
		}

		mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
		sc->syscon = syscon_create_ofw_node(dev,
		    &syscon_class, ofw_bus_get_node(dev));
		if (sc->syscon == NULL) {
			device_printf(dev,
			    "Failed to create/register syscon\n");
			return (ENXIO);
		}
	}

	return (0);
}

static int
mt_pciecfg_detach(device_t dev)
{
	device_printf(dev, "Error: Pciecfg driver cannot be detached\n");
	return (EBUSY);
}

static int
mt_pciecfg_syscon_get_handle(device_t dev, struct syscon **syscon)
{
	struct mt_pciecfg_softc *sc;

	sc = device_get_softc(dev);
	*syscon = sc->syscon;
	if (*syscon == NULL) {
		return (ENODEV);
	}

	return (0);
}

static void
mt_pciecfg_syscon_lock(device_t dev)
{
	struct mt_pciecfg_softc *sc;

	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
mt_pciecfg_syscon_unlock(device_t dev)
{
	struct mt_pciecfg_softc *sc;

	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t mt_pcie_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, mt_pciecfg_probe),
	DEVMETHOD(device_attach, mt_pciecfg_attach),
	DEVMETHOD(device_detach, mt_pciecfg_detach),

	/* Syscon interface */
	DEVMETHOD(syscon_get_handle,	mt_pciecfg_syscon_get_handle),
	DEVMETHOD(syscon_device_lock,	mt_pciecfg_syscon_lock),
	DEVMETHOD(syscon_device_unlock,	mt_pciecfg_syscon_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_1(mt_pciecfg, mt_pciecfg_driver, mt_pcie_methods,
    sizeof(struct mt_pciecfg_softc), syscon_class);

EARLY_DRIVER_MODULE(mt_pciecfg, simplebus, mt_pciecfg_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 3);