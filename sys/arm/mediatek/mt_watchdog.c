/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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
#include <sys/eventhandler.h>
#include <sys/systm.h>
#include <sys/watchdog.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

/* Watchdog Mode Register */
#define	TOPRGUWDT_MODE			0x00
#define	 TOPRGUWDT_MODE_EN		0x1
#define	 TOPRGUWDT_MODE_EXT_POL_LOW	0x0
#define	 TOPRGUWDT_MODE_EXT_POL_HIGH	0x2
#define	 TOPRGUWDT_MODE_EXRST_EN	0x4
#define	 TOPRGUWDT_MODE_IRQ_EN		0x8
#define	 TOPRGUWDT_MODE_AUTO_START	0x10
#define	 TOPRGUWDT_MODE_DUAL_EN		0x40
#define	 TOPRGUWDT_MODE_CNT_SEL		0x100
#define	 TOPRGUWDT_MODE_KEY		0x22000000
/* Watchdog Counter Setting Register */
#define	TOPRGUWDT_LENGTH		0x04
#define	 TOPRGUWDT_LENGTH_TIMEOUT(n)	((n) << 5)
#define	 TOPRGUWDT_LENGTH_KEY		0x8
/* Watchdog Counter Restart Register */
#define	TOPRGUWDT_RESTART		0x08
#define	 TOPRGUWDT_RESTART_RELOAD	0x1971
/* Watchdog Status Register */
#define	TOPRGUWDT_STA			0x0c
/* Software Watchdog Reset Register */
#define	TOPRGUWDT_SWRST			0x14
#define	 TOPRGUWDT_SWRST_KEY		0x1209
/* System Software Reset Register */
#define	TOPRGUWDT_SWSYSRST		0x18
#define	 TOPRGUWDT_SWSYS_RST_KEY	0x88000000
#define	 TOPRGUWDT_SWSYSRST_EN		0xfc
/* Watchdog Non-Reset Register */
#define	TOPRGUWDT_NONRST_REG		0x20
/* Reset Request Mode Register */
#define	TOPRGUWDT_REQ_MOD		0x30
/* Reset Request IRQ Enable Register */
#define	TOPRGUWDT_REQ_IRQ_EN		0x34

/*
 * The counter ticks every 512 cycles of the 32768 Hz clock, so one length
 * unit is 15.625 ms and 64 units make a second.  The length field is 11
 * bits wide; stay within the documented 2 to 31 second range.
 */
#define	TOPRGUWDT_UNIT_NS		15625000ULL
#define	TOPRGUWDT_UNITS_PER_SEC		64
#define	TOPRGUWDT_MIN_TIMEOUT		2
#define	TOPRGUWDT_MAX_TIMEOUT		31
#define	TOPRGUWDT_MIN_UNITS	\
	(TOPRGUWDT_MIN_TIMEOUT * TOPRGUWDT_UNITS_PER_SEC)
#define	TOPRGUWDT_MAX_UNITS	\
	(TOPRGUWDT_MAX_TIMEOUT * TOPRGUWDT_UNITS_PER_SEC)

static struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7622-wdt",	1 },
    { "mediatek,mt7623-wdt",	1 },
    { "mediatek,mt6589-wdt",	1 },
    { NULL,				0 }
};

struct mt_watchdog_softc {
    device_t		dev;
    struct resource		*res;
};

static void
mt_watchdog_disable(struct mt_watchdog_softc *sc)
{
	uint32_t mode;

	mode = bus_read_4(sc->res, TOPRGUWDT_MODE);
	mode &= ~TOPRGUWDT_MODE_EN;
	mode |= TOPRGUWDT_MODE_KEY;
	bus_write_4(sc->res, TOPRGUWDT_MODE, mode);
}

/*
 * Convert a watchdog(9) interval, the log2 of the timeout in nanoseconds,
 * to counter units.  Round up so the timer never fires before the caller
 * expects it to.  Return 0 when the hardware cannot wait that long.
 */
static uint32_t
mt_watchdog_units(u_int interval)
{
	uint64_t units;

	if (interval >= 64)
		return (0);
	units = howmany(1ULL << interval, TOPRGUWDT_UNIT_NS);
	if (units > TOPRGUWDT_MAX_UNITS)
		return (0);
	if (units < TOPRGUWDT_MIN_UNITS)
		units = TOPRGUWDT_MIN_UNITS;
	return (units);
}

static void
mt_watchdog_fn(void *private, u_int cmd, int *error)
{
	struct mt_watchdog_softc *sc;
	uint32_t mode, units;

	sc = private;
	cmd &= WD_INTERVAL;

	/* watchdog(9): on disable and on failure to arm, leave *error alone. */
	if (cmd == 0) {
		mt_watchdog_disable(sc);
		return;
	}

	units = mt_watchdog_units(cmd);
	if (units == 0) {
		if (bootverbose)
			device_printf(sc->dev,
			    "cannot arm, timeout 2^%u ns is over %u s\n",
			    cmd, TOPRGUWDT_MAX_TIMEOUT);
		mt_watchdog_disable(sc);
		return;
	}

	if (bootverbose)
		device_printf(sc->dev, "timeout 2^%u ns, %u units of %u us\n",
		    cmd, units, (u_int)(TOPRGUWDT_UNIT_NS / 1000));

	/* The length lives in bits 15:5; the key unlocks the write. */
	bus_write_4(sc->res, TOPRGUWDT_LENGTH,
	    TOPRGUWDT_LENGTH_TIMEOUT(units) | TOPRGUWDT_LENGTH_KEY);

	mode = TOPRGUWDT_MODE_KEY | TOPRGUWDT_MODE_EN |
	       TOPRGUWDT_MODE_EXRST_EN | TOPRGUWDT_MODE_EXT_POL_LOW;
	bus_write_4(sc->res, TOPRGUWDT_MODE, mode);
	bus_write_4(sc->res, TOPRGUWDT_RESTART, TOPRGUWDT_RESTART_RELOAD);

	*error = 0;
}

static int
mt_watchdog_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek TOPRGU watchdog");
	return (BUS_PROBE_DEFAULT);
}

static int
mt_watchdog_attach(device_t dev)
{
	struct mt_watchdog_softc *sc;
	int rid;

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "Could not allocate memory resource\n");
		return (ENXIO);
	}

	EVENTHANDLER_REGISTER(watchdog_list, mt_watchdog_fn, sc, 0);

	/* Keep the watchdog off until watchdog(9) asks for it. */
	mt_watchdog_disable(sc);

	return (0);
}

static device_method_t mt_watchdog_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		mt_watchdog_probe),
    DEVMETHOD(device_attach,	mt_watchdog_attach),

    DEVMETHOD_END
};

static DEFINE_CLASS_0(mt_wdog, mt_wdog_driver, mt_watchdog_methods,
sizeof(struct mt_watchdog_softc));
DRIVER_MODULE(mt_wdog, simplebus, mt_wdog_driver, NULL, NULL);
