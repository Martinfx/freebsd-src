/*
 * Copyright (c) 2026 Martin Filla
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * Driver for the MediaTek "pciecfg" register block (compatible
 * "mediatek,generic-pciecfg").  This is a small block of shared
 * configuration bits used to bring up the MT7622 PCIe root complexes;
 * it is consumed through the syscon(9) interface by the mt7622_pcie
 * driver.
 *
 * The driver follows the same model as the generic syscon driver: it
 * implements the unlocked accessors plus the device lock/unlock methods
 * and lets the base syscon class provide the locked read/write/modify
 * wrappers.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/syscon/syscon.h>

#include "syscon_if.h"

MALLOC_DECLARE(M_SYSCON);

struct mt_pciecfg_softc {
    device_t		dev;
    struct resource		*mem_res;
    struct syscon		*syscon;
    struct mtx		mtx;
};

#define	PCIECFG_LOCK(_sc)		mtx_lock_spin(&(_sc)->mtx)
#define	PCIECFG_UNLOCK(_sc)		mtx_unlock_spin(&(_sc)->mtx)
#define	PCIECFG_LOCK_INIT(_sc)		mtx_init(&(_sc)->mtx,		\
	    device_get_nameunit((_sc)->dev), "mt_pciecfg", MTX_SPIN)
#define	PCIECFG_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->mtx)
#define	PCIECFG_ASSERT_LOCKED(_sc)	mtx_assert(&(_sc)->mtx, MA_OWNED)

static int mt_pciecfg_detach(device_t dev);

static struct ofw_compat_data compat_data[] = {
        {"mediatek,generic-pciecfg",	1},
        {NULL,				0}
};

/*
 * Syscon (unlocked) accessors.  The base syscon class takes the device
 * lock around these via the locked read_4/write_4/modify_4 wrappers.
 */
static uint32_t
mt_pciecfg_unlocked_read_4(struct syscon *syscon, bus_size_t offset)
{
    struct mt_pciecfg_softc *sc;

    sc = device_get_softc(syscon->pdev);
    PCIECFG_ASSERT_LOCKED(sc);
    return (bus_read_4(sc->mem_res, offset));
}

static int
mt_pciecfg_unlocked_write_4(struct syscon *syscon, bus_size_t offset,
                            uint32_t val)
{
    struct mt_pciecfg_softc *sc;

    sc = device_get_softc(syscon->pdev);
    PCIECFG_ASSERT_LOCKED(sc);
    bus_write_4(sc->mem_res, offset, val);
    return (0);
}

static int
mt_pciecfg_unlocked_modify_4(struct syscon *syscon, bus_size_t offset,
                             uint32_t clear_bits, uint32_t set_bits)
{
    struct mt_pciecfg_softc *sc;
    uint32_t val;

    sc = device_get_softc(syscon->pdev);
    PCIECFG_ASSERT_LOCKED(sc);
    val = bus_read_4(sc->mem_res, offset);
    val &= ~clear_bits;
    val |= set_bits;
    bus_write_4(sc->mem_res, offset, val);
    return (0);
}

static syscon_method_t mt_pciecfg_syscon_methods[] = {
        SYSCONMETHOD(syscon_unlocked_read_4,	mt_pciecfg_unlocked_read_4),
        SYSCONMETHOD(syscon_unlocked_write_4,	mt_pciecfg_unlocked_write_4),
        SYSCONMETHOD(syscon_unlocked_modify_4,	mt_pciecfg_unlocked_modify_4),

        SYSCONMETHOD_END
};
DEFINE_CLASS_1(mt_pciecfg_syscon, mt_pciecfg_syscon_class,
        mt_pciecfg_syscon_methods, 0, syscon_class);

/*
 * Device lock/unlock methods used by the base syscon class.
 */
static void
mt_pciecfg_syscon_lock(device_t dev)
{
    struct mt_pciecfg_softc *sc;

    sc = device_get_softc(dev);
    PCIECFG_LOCK(sc);
}

static void
mt_pciecfg_syscon_unlock(device_t dev)
{
    struct mt_pciecfg_softc *sc;

    sc = device_get_softc(dev);
    PCIECFG_UNLOCK(sc);
}

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
    int rid;

    sc = device_get_softc(dev);
    sc->dev = dev;
    rid = 0;

    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                         RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Cannot allocate memory resource\n");
        return (ENXIO);
    }

    PCIECFG_LOCK_INIT(sc);

    sc->syscon = syscon_create_ofw_node(dev, &mt_pciecfg_syscon_class,
                                        ofw_bus_get_node(dev));
    if (sc->syscon == NULL) {
        device_printf(dev, "Failed to create/register syscon\n");
        mt_pciecfg_detach(dev);
        return (ENXIO);
    }

    return (0);
}

static int
mt_pciecfg_detach(device_t dev)
{
    struct mt_pciecfg_softc *sc;

    sc = device_get_softc(dev);

    if (sc->syscon != NULL) {
        syscon_unregister(sc->syscon);
        free(sc->syscon, M_SYSCON);
        sc->syscon = NULL;
    }

    PCIECFG_LOCK_DESTROY(sc);

    if (sc->mem_res != NULL)
        bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);

    return (0);
}

static device_method_t mt_pciecfg_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		mt_pciecfg_probe),
        DEVMETHOD(device_attach,	mt_pciecfg_attach),
        DEVMETHOD(device_detach,	mt_pciecfg_detach),

        /* Syscon interface */
        DEVMETHOD(syscon_device_lock,	mt_pciecfg_syscon_lock),
        DEVMETHOD(syscon_device_unlock,	mt_pciecfg_syscon_unlock),

        DEVMETHOD_END
};

DEFINE_CLASS_0(mt_pciecfg, mt_pciecfg_driver, mt_pciecfg_methods,
sizeof(struct mt_pciecfg_softc));

EARLY_DRIVER_MODULE(mt_pciecfg, simplebus, mt_pciecfg_driver, NULL, NULL,
        BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 6);
MODULE_VERSION(mt_pciecfg, 1);
