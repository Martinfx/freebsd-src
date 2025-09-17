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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/hwreset/hwreset.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "hwreset_if.h"

struct sophgo_reset_clk_softc {
	device_t            dev;
	struct resource     *res;
	int                 rid;
	struct mtx          mtx;
};

static int
sophgo_reset_clk_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "cvitek,clk-reset"))
		return (ENXIO);

	device_set_desc(dev, "Cv181 clock reset");

	return (BUS_PROBE_DEFAULT);
}

static int
sophfo_reset_clk_attach(device_t dev)
{
	struct sophgo_reset_clk_softc *sc;
	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
	    RF_ACTIVE);
	if(sc->res == NULL) {
		device_printf(sc->dev, "Cannot allocation resourcer\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "cvitek_reset", MTX_DEF);

	return (0);
}

static int
sophfo_reset_clk_detach(device_t dev)
{
	struct sophgo_reset_clk_softc *sc;
	sc= device_get_softc(dev);
	if (sc->res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
		sc->res = NULL;
	}
	return (0);
}

static int
sophfo_reset_clk_assert(device_t dev, intptr_t id, bool reset)
{
	struct sophgo_reset_clk_softc *sc;
	uint32_t offset, val, bank;

	sc = device_get_softc(dev);
	bank = id % 32;
	offset = id / 32;

	mtx_lock(&sc->mtx);
	val = bus_read_4(sc->res, bank * 4);
	val &= ~(1 << offset);
	bus_write_4(sc->res, bank * 4, val);
	mtx_unlock(&sc->mtx);
	return (0);
}

static device_method_t sophgo_reset_clk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,     sophgo_reset_clk_probe),
	DEVMETHOD(device_attach,    sophfo_reset_clk_attach),
	DEVMETHOD(device_detach,    sophfo_reset_clk_detach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,	sophfo_reset_clk_assert),
	DEVMETHOD_END
};

static DEFINE_CLASS_0(sophgo_reset_clk,  sophgo_reset_clk_driver, sophgo_reset_clk_methods,
    sizeof(struct sophgo_reset_clk_softc));
DRIVER_MODULE(sophgo_reset_clk, simplebus, sophgo_reset_clk_driver, NULL, NULL);