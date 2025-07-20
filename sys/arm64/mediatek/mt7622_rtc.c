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
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/module.h>
#include <sys/resource.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <dev/clk/clk.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "clock_if.h"

#define RTC_PWRCHK1		    0x4
/* RTC power ready check 1 (0xC6) */
#define	RTC_PWRCHK1_MAGIC	0xc6
#define RTC_PWRCHK2		    0x8
/* RTC power ready check 2 (0x9A) */
#define	RTC_PWRCHK2_MAGIC	0x9a
#define RTC_KEY		        0xc
#define	RTC_KEY_MAGIC		0x59
#define RTC_PROT1		    0x10
#define	RTC_PROT1_MAGIC		0xa3
#define RTC_PROT2		    0x14
#define	RTC_PROT2_MAGIC		0x57
#define RTC_PROT3		    0x18
#define	RTC_PROT3_MAGIC		0x67
#define RTC_PROT4		    0x1c
#define	RTC_PROT4_MAGIC		0xd2
#define RTC_DEBNCE		    0x2c
#define RTC_INT 		    0x30
#define RTC_CTL             0x20
/* Stop the ripple counter */
#define RC_STOP             0x00
#define RTC_INT 	    	0x30

static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt7622-rtc",	1},
        {"mediatek,soc-rtc", 1},
        {NULL,			0}
};

struct mt7622_rtc_softc {
    device_t		dev;
    struct mtx		mtx;

    struct resource		*mem_res;
    struct resource		*irq_res;
    void			*irq_h;

    clk_t			clk;
    uint32_t		core_freq;
};

static int
mt7622_rtc_gettime(device_t dev, struct timespec *ts)
{
    return (0);
}

static int
mt7622_rtc_settime(device_t dev, struct timespec *ts)
{
    return (0);
}

static void
mt7622_rtc_intr(void *arg)
{
    struct mt7622_rtc_softc *sc;
    uint32_t status;

    sc = (struct mt7622_rtc_softc *)arg;
    mtx_lock(&(sc)->mtx);
    status = bus_read_1(sc->mem_res, RTC_INT);
    bus_write_4(sc->mem_res, RTC_INT, status);
    mtx_unlock(&(sc)->mtx);
}

static int
mt7622_rtc_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
        return (ENXIO);

    device_set_desc(dev, "Mediatek mt7622 rtc driver.");

    return (BUS_PROBE_DEFAULT);
}

static int
mt7622_rtc_attach(device_t dev)
{
    int rv, rid;
    struct mt7622_rtc_softc *sc;

    sc = device_get_softc(dev);
    sc->dev = dev;

    mtx_init(&sc->mtx, device_get_nameunit(sc->dev), "mt7622_rtc", MTX_DEF);

    rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                         RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Cannot map registers.\n");
        rv = ENXIO;
        goto fail;
    }

    /* Allocate our IRQ resource. */
    rid = 0;
    sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
                                         RF_ACTIVE);
    if (sc->irq_res == NULL) {
        device_printf(dev, "Cannot allocate interrupt.\n");
        rv = ENXIO;
        goto fail;
    }

    rv = clk_get_by_ofw_index(dev, 0, 0, &sc->clk);
    if (rv != 0) {
        device_printf(dev, "Cannot get clock: %d\n", rv);
        goto fail;
    }
    rv = clk_enable(sc->clk);
    if (rv != 0) {
        device_printf(dev, "Cannot enable clock: %d\n", rv);
        goto fail;
    }

    /* Init hardware */
    bus_write_1(sc->mem_res, RTC_PWRCHK1_MAGIC, RTC_PWRCHK1);
    bus_write_1(sc->mem_res, RTC_PWRCHK2_MAGIC, RTC_PWRCHK2);
    bus_write_1(sc->mem_res, RTC_KEY_MAGIC, RTC_KEY);
    bus_write_1(sc->mem_res, RTC_PROT1_MAGIC, RTC_PROT1);
    bus_write_1(sc->mem_res, RTC_PROT2_MAGIC, RTC_PROT2);
    bus_write_1(sc->mem_res, RTC_PROT3_MAGIC, RTC_PROT3);
    bus_write_1(sc->mem_res, RTC_PROT4_MAGIC, RTC_PROT4);

    uint8_t v;
    v = bus_read_1(sc->mem_res, RTC_DEBNCE);
    v &= ~(0x07);
    bus_write_1(sc->mem_res, RTC_DEBNCE, v);

    v = bus_read_1(sc->mem_res, RTC_CTL);
    v &= ~RC_STOP;
    v |= 0;
    bus_write_1(sc->mem_res, RTC_CTL, v);

    rv = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
                        NULL, mt7622_rtc_intr, sc, &sc->irq_h);
    if (rv) {
        device_printf(dev, "Cannot setup interrupt.\n");
        goto fail;
    }

    bus_attach_children(dev);
    return (0);

    fail:
    if (sc->clk != NULL)
        clk_release(sc->clk);
    if (sc->irq_h != NULL)
        bus_teardown_intr(dev, sc->irq_res, sc->irq_h);
    if (sc->irq_res != NULL)
        bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
    if (sc->mem_res != NULL)
        bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
    mtx_destroy(&sc->mtx);

    return (rv);
}

static int
mt7622_rtc_detach(device_t dev)
{
    struct mt7622_rtc_softc *sc;
    int error;

    error = bus_generic_detach(dev);
    if (error != 0)
        return (error);

    sc = device_get_softc(dev);
    if (sc->irq_h != NULL)
        bus_teardown_intr(dev, sc->irq_res, sc->irq_h);
    if (sc->irq_res != NULL)
        bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
    if (sc->mem_res != NULL)
        bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

    mtx_destroy(&sc->mtx);
    return (0);
}

static device_method_t mt7622_rtc_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		mt7622_rtc_probe),
        DEVMETHOD(device_attach,	mt7622_rtc_attach),
        DEVMETHOD(device_detach,	mt7622_rtc_detach),

        /* clock interface */
        DEVMETHOD(clock_gettime,	mt7622_rtc_gettime),
        DEVMETHOD(clock_settime,	mt7622_rtc_settime),

        DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7622_rtc, mt7622_rtc_driver, mt7622_rtc_methods,
sizeof(struct mt7622_rtc_softc));
DRIVER_MODULE(mt7622_rtc, simplebus, mt7622_rtc_driver, NULL, NULL);