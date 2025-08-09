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
#include <sys/eventhandler.h>
#include <sys/systm.h>
#include <sys/watchdog.h>
#include <sys/reboot.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <machine/bus.h>

#define TOPRGUWDT_MAX_TIMEOUT	31
#define TOPRGUWDT_MIN_TIMEOUT	2
/* Watchdog Mode Register */
#define TOPRGUWDT_MODE          0x0
/* Watchdog Counter Setting Register */
#define TOPRGUWDT_LENGTH        0x4
#define TOPRGUWDT_LENGTH_KEY    0x8
/* Watchdog Counter Restart Register */
#define TOPRGUWDT_RESTART		0x08
#define TOPRGUWDT_RESTART_RELOAD 0x1971
/* Software Watchdog Reset Register */
#define TOPRGUWDT_SWRST         0x18
/* Watchdog Non-Reset Register */
#define TOPRGUWDT_NONRST_REG    0x20

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-wdt",	1},
        {"mediatek,mt6589-wdt", 1},
        {NULL,			0}
};

struct mt7622_watchdog_softc {
    device_t		dev;
    struct resource *	res;
    struct mtx		mtx;
};

static int
mt7622_watchdog_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
        return (ENXIO);

    device_set_desc(dev, "Mediatek mt7622 watchdog driver.");

    return (BUS_PROBE_DEFAULT);
}

static int
mt7622_watchdog_attach(device_t dev)
{
    struct mt7622_watchdog_softc *sc;
    int rid = 0;

    sc = device_get_softc(dev);
    sc->dev = dev;

    sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
    if (sc->res == NULL) {
        device_printf(dev, "Could not allocate memory resource\n");
        return (ENXIO);
    }

    mtx_init(&sc->mtx, "Mediatek Watchdog", "mt7622_wdog", MTX_SPIN);

    return (0);
}

static device_method_t mt7622_watchdog_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		mt7622_watchdog_probe),
        DEVMETHOD(device_attach,	mt7622_watchdog_attach),

        DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7622_watchdog, mt7622_watchdog_driver, mt7622_watchdog_methods,
sizeof(struct mt7622_watchdog_softc));
DRIVER_MODULE(mt7622_watchdog, simplebus, mt7622_watchdog_driver, NULL, NULL);